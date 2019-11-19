/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "SoundTriggerEngineGsl"

#include "SoundTriggerEngineGsl.h"

#include "Session.h"
#include "SessionGsl.h"
#include "Stream.h"
#include "StreamSoundTrigger.h"
#include "ResourceManager.h"
#include "kvh2xml.h"

// TODO: Move to sound trigger xml files
#define HIST_BUFFER_DURATION_MS 1750
#ifdef FEATURE_IPQ_OPENWRT
#define PRE_ROLL_DURATION_IN_MS 500
#else
#define PRE_ROLL_DURATION_IN_MS 250
#endif
#define DWNSTRM_SETUP_DURATION_MS 300

void SoundTriggerEngineGsl::BufferThreadLoop(
    SoundTriggerEngineGsl *gsl_engine)
{
    StreamSoundTrigger *s = nullptr;

    QAL_INFO(LOG_TAG, "Enter. start thread loop");
    if (!gsl_engine) {
        QAL_ERR(LOG_TAG, "Invalid sound trigger engine");
        return;
    }

    std::unique_lock<std::mutex> lck(gsl_engine->event_mutex_);
    while (!gsl_engine->exit_thread_) {
        QAL_VERBOSE(LOG_TAG, "waiting on cond");
        /* Wait for keyword buffer data from DSP */
        if (!gsl_engine->event_detected_ || gsl_engine->exit_buffering_)
            gsl_engine->cv_.wait(lck);
        QAL_DBG(LOG_TAG, "done waiting on cond, exit buffering %d",
                gsl_engine->exit_buffering_);

        if (gsl_engine->exit_thread_) {
            QAL_VERBOSE(LOG_TAG, "Exit buffer thread");
            break;
        }

        if (gsl_engine->exit_buffering_) {
            continue; /* skip over processing if we want to exit already*/
        }

        s = dynamic_cast<StreamSoundTrigger *>(gsl_engine->stream_handle_);
        if (gsl_engine->StartBuffering())
            gsl_engine->event_detected_ = false;

        if (gsl_engine->timestamp_recorded_ &&
            !gsl_engine->is_stream_notified_) {
            s->SetDetectionState(GMM_DETECTED);
            gsl_engine->is_stream_notified_ = true;
        }
    }
    QAL_DBG(LOG_TAG, "Exit");
}

static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineGsl::StartSoundEngine()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    event_detected_ = false;

    if (capture_requested_) {
        exit_thread_ = false;
        exit_buffering_ = false;

        buffer_thread_handler_ =
            std::thread(SoundTriggerEngineGsl::BufferThreadLoop, this);

        if (!buffer_thread_handler_.joinable()) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "failed to create buffer thread, status = %d",
                    status);
        }
    }

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::StopSoundEngine()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");

    event_detected_ = false;
    if (capture_requested_) {
        {
            std::lock_guard<std::mutex> lck(event_mutex_);
            exit_thread_ = true;
            exit_buffering_ = true;
            timestamp_recorded_ = false;
            is_stream_notified_ = false;
            cv_.notify_one();
        }
        buffer_thread_handler_.join();
        QAL_INFO(LOG_TAG, "Thread joined");
    }
    QAL_DBG(LOG_TAG, "Exit");

    return status;
}

int32_t SoundTriggerEngineGsl::StartBuffering()
{
    int32_t status = 0;
    int32_t size;
    struct qal_buffer buf;
    size_t input_buf_size = 0;
    size_t input_buf_num = 0;
    size_t output_buf_size = 0;
    size_t output_buf_num = 0;
    size_t size_to_write = 0;
    uint64_t timestamp = 0;
    uint64_t start_timestamp = 0;
    uint64_t end_timestamp = 0;
    size_t start_indice = 0;
    size_t end_indice = 0;
    struct detection_event_info *info = nullptr;
    StreamSoundTrigger *s = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (exit_buffering_ || !event_detected_) {
        QAL_VERBOSE(LOG_TAG, "Exit, exit buffering %d, event detected %d",
                    exit_buffering_, event_detected_);
        goto exit;
    }

    stream_handle_->getBufInfo(&input_buf_size, &input_buf_num,
                               &output_buf_size, &output_buf_num);
    memset(&buf, 0, sizeof(struct qal_buffer));
    buf.size = input_buf_size * input_buf_num;
    buf.buffer = (uint8_t *)calloc(1, buf.size);
    if (!buf.buffer) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "buffer calloc failed, status %d", status);
        goto exit;
    }

    if (!timestamp_recorded_) {
        buf.ts = (struct timespec *)calloc(1, sizeof(struct timespec));
        if (!buf.ts) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "buffer calloc failed, status %d", status);
            goto exit;
        }
    }

    /*TODO: add max retry num to avoid dead lock*/
    QAL_VERBOSE(LOG_TAG, "trying to read %u from gsl", buf.size);

    // read data from session
    if (!size_to_write && buffer_->getFreeSize() >= buf.size) {
        status = session_->read(stream_handle_, SHMEM_ENDPOINT, &buf, &size);
        QAL_INFO(LOG_TAG, "%d read from session, %u to be read",
                 size, buf.size);
        size_to_write = size;
    }

    // write data to ring buffer
    if (size_to_write) {
        if (!timestamp_recorded_) {
            timestamp = ((uint64_t)buf.ts->tv_sec * 1000000000 +
                        (uint64_t)buf.ts->tv_nsec) / 1000;
            s = dynamic_cast<StreamSoundTrigger *>(stream_handle_);
            info = s->getDetectionEventInfo();
            if (!info) {
                status = -EINVAL;
                QAL_ERR(LOG_TAG, "Invalid detection event info");
                goto exit;
            }

            start_timestamp = (uint64_t)info->kw_start_timestamp_lsw +
                              ((uint64_t)info->kw_start_timestamp_msw << 32);
            end_timestamp = (uint64_t)info->kw_end_timestamp_lsw +
                            ((uint64_t)info->kw_end_timestamp_msw << 32);
            start_indice = us_to_bytes(start_timestamp - timestamp);
            end_indice = us_to_bytes(end_timestamp - timestamp);
            buffer_->updateIndices(start_indice, end_indice);
            timestamp_recorded_ = true;
        }
        size_t ret = buffer_->write(buf.buffer, size_to_write);
        size_to_write -= ret;
        QAL_INFO(LOG_TAG, "%u written to ring buffer", ret);
    }

exit:
    if (buf.buffer)
        free(buf.buffer);
    if (buf.ts)
        free(buf.ts);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

SoundTriggerEngineGsl::SoundTriggerEngineGsl(
    Stream *s,
    uint32_t id,
    uint32_t stage_id,
    QalRingBufferReader **reader,
    std::shared_ptr<QalRingBuffer> buffer)
{
    struct qal_stream_attributes sAttr;
    std::shared_ptr<ResourceManager> rm = nullptr;
    uint32_t sampleRate;
    uint32_t bitWidth;
    uint32_t channels;
    uint32_t bufferSize = DEFAULT_QAL_RING_BUFFER_SIZE;
    size_t num_output_ports;
    uint32_t size;
    engine_id_ = id;
    stage_id_ = stage_id;
    event_detected_ = false;
    exit_thread_ = false;
    exit_buffering_ = false;
    timestamp_recorded_ = false;
    is_stream_notified_ = false;
    capture_requested_ = false;
    stream_handle_ = s;
    sm_data_ = nullptr;
    dam_setup_duration_ = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    // Create session
    rm = ResourceManager::getInstance();
    if (!rm) {
        QAL_ERR(LOG_TAG, "Failed to get ResourceManager instance");
        throw std::runtime_error("Failed to get ResourceManager instance");
    }
    s->getStreamAttributes(&sAttr);
    session_ = Session::makeSession(rm, &sAttr);
    if (!session_) {
        QAL_ERR(LOG_TAG, "Failed to create session");
        throw std::runtime_error("Failed to create session");
    }

    session_->registerCallBack(HandleSessionCallBack, this);

    // Init internal structures
    event_config_.event_mode = CONFIDENCE_LEVEL_INFO |
                               KEYWORD_INDICES_INFO |
                               TIME_STAMP_INFO |
                               FTRT_INFO;

    buffer_config_.hist_buffer_duration_in_ms = 0;
    buffer_config_.pre_roll_duration_in_ms = 0;

    num_output_ports = 1;
    size = sizeof(struct audio_dam_downstream_setup_duration) +
                  num_output_ports *
                  sizeof(struct audio_dam_downstream_setup_duration_t);
    dam_setup_duration_ = (struct audio_dam_downstream_setup_duration *)
                      calloc(1, size);
    if (!dam_setup_duration_) {
        QAL_ERR(LOG_TAG, "Failed to allocate dam setup duration");
        delete session_;
        throw std::runtime_error("Failed to allocate dam setup duration");
    }
    dam_setup_duration_->num_output_ports = num_output_ports;

    for (int i = 0; i < dam_setup_duration_->num_output_ports; i++) {
        dam_setup_duration_->port_cfgs[i].output_port_id = 1;
        dam_setup_duration_->port_cfgs[i].dwnstrm_setup_duration_ms =
            DWNSTRM_SETUP_DURATION_MS;
    }

    // Create ring buffer when reader passed is not specified
    if (!buffer) {
        QAL_INFO(LOG_TAG, "creating new ring buffer");
        struct qal_stream_attributes sAttr;
        s->getStreamAttributes(&sAttr);
        if (sAttr.direction == QAL_AUDIO_INPUT) {
            sampleRate = sAttr.in_media_config.sample_rate;
            bitWidth = sAttr.in_media_config.bit_width;
            channels = sAttr.in_media_config.ch_info->channels;
            // ring buffer size equals to 3s' audio data
            // as second stage may need 2-2.5s data to detect
            bufferSize = sampleRate * bitWidth * channels *
                         RING_BUFFER_DURATION / BITS_PER_BYTE;
        }

        buffer_ = new QalRingBuffer(bufferSize);
        reader_ = nullptr;
        *reader = buffer_->newReader();
    } else {
        // Avoid this engine write data to existing ring buffer
        buffer_ = nullptr;
        reader_ = buffer->newReader();
    }

    QAL_DBG(LOG_TAG, "Exit");
}

SoundTriggerEngineGsl::~SoundTriggerEngineGsl()
{
    QAL_DBG(LOG_TAG, "Enter");

    if (dam_setup_duration_)
        free(dam_setup_duration_);

    if (buffer_)
        free(buffer_);

    QAL_DBG(LOG_TAG, "Exit");
}

int32_t SoundTriggerEngineGsl::LoadSoundModel(Stream *s, uint8_t *data,
                                              uint32_t data_size)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;
    struct qal_st_sound_model *common_sm = nullptr;
    SML_BigSoundModelTypeV3 *big_sm = nullptr;
    uint8_t *sm_payload = nullptr;

    std::lock_guard<std::mutex> lck(mutex_);
    if (!data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound model data status %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter");
    sm_data_ = data;

    status = session_->setParameters(stream_handle_, DEVICE_SVA,
                                     PARAM_ID_DETECTION_ENGINE_SOUND_MODEL,
                                     (void *)sm_data_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to load sound model, status = %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::UnloadSoundModel(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    sm_data_ = nullptr;
    sm_data_size_ = 0;

exit:
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StartRecognition(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP,
        &wakeup_config_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set wake up config, status = %d", status);
        goto exit;
    }

    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG,
        &event_config_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set event config, status = %d", status);
        goto exit;
    }

    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG,
        &buffer_config_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set wake-up buffer config, status = %d",
                status);
        goto exit;
    }

    status = session_->setParameters(
        stream_handle_,
        DEVICE_ADAM,
        PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION,
        dam_setup_duration_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set downstream setup duration, status = %d",
                status);
        goto exit;
    }

    status = StartSoundEngine();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

    status = session_->prepare(stream_handle_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to prepare session, status = %d", status);
        goto exit;
    }

    status = session_->start(stream_handle_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to start session, status = %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::StopBuffering(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);

    event_detected_ = false;
    exit_buffering_ = true;
    timestamp_recorded_ = false;
    is_stream_notified_ = false;
    if (buffer_) {
        buffer_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_RESET,
        nullptr);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d",
                status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::StopRecognition(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    status = StopSoundEngine();
    std::lock_guard<std::mutex> lck(mutex_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to stop sound engine, status = %d", status);
        goto exit;
    }

    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_RESET,
        nullptr);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d",
                status);
        goto exit;
    }

    if (buffer_) {
        buffer_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

    status = session_->stop(stream_handle_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to stop session, status = %d", status);
        goto exit;
    }

exit:
    QAL_ERR(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::UpdateConfLevels(
    Stream *s __unused,
    struct qal_st_recognition_config *config,
    uint8_t *conf_levels,
    uint32_t num_conf_levels)
{
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mutex_);
    if (!config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid config, status %d", status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "Enter, config: %pK", config);
    wakeup_config_.mode = config->phrases[0].recognition_modes;
    wakeup_config_.custom_payload_size = config->data_size;
    wakeup_config_.num_active_models = num_conf_levels;
    wakeup_config_.reserved = 0;
    for (int i = 0; i < wakeup_config_.num_active_models; i++) {
        wakeup_config_.confidence_levels[i] = conf_levels[i];
        wakeup_config_.keyword_user_enables[i] = 1;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::UpdateBufConfig(uint32_t hist_buffer_duration,
                                               uint32_t pre_roll_duration)
{
    int32_t status = 0;

    buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
    buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;

    return status;
}

void SoundTriggerEngineGsl::SetDetected(bool detected)
{
    QAL_INFO(LOG_TAG, "isDetected = %d", detected);
    if (!capture_requested_) {
        event_detected_ = detected;
        return;
    }

    std::lock_guard<std::mutex> lck(event_mutex_);
    if (detected != event_detected_) {
        QAL_INFO(LOG_TAG, "notify condition variable");
        event_detected_ = detected;
        exit_buffering_ = !event_detected_;
        QAL_INFO(LOG_TAG, "event detected: %d", detected);
        cv_.notify_one();
    } else {
        QAL_VERBOSE(LOG_TAG, "event detected unchanged");
    }
}

void SoundTriggerEngineGsl::HandleSessionCallBack(void *hdl, uint32_t event_id,
                                                  void *data)
{
    int status = 0;
    StreamSoundTrigger *s = nullptr;
    SoundTriggerEngineGsl *engine = nullptr;

    QAL_DBG(LOG_TAG, "Enter, event detected on GECKO, event id = %u", event_id);
    if (!hdl || !data) {
        QAL_ERR(LOG_TAG, "No engine handle or event data provided");
        return;
    }

    engine = (SoundTriggerEngineGsl *)hdl;
    s = dynamic_cast<StreamSoundTrigger *>(engine->stream_handle_);
    status = s->ParseDetectionPayload(event_id, (uint32_t *)data);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to parse detection payload with ret = %d",
                status);
        return;
    }

    engine->SetDetected(true);

    if (!engine->capture_requested_)
        s->SetDetectionState(GMM_DETECTED);

    QAL_DBG(LOG_TAG, "Exit");

    return;
}

int32_t SoundTriggerEngineGsl::getParameters(uint32_t param_id, void **payload)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    switch (param_id) {
        case QAL_PARAM_ID_DIRECTION_OF_ARRIVAL:
            status = session_->getParameters(stream_handle_, TAG_FLUENCE,
                                            param_id, payload);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Unsupported param id %u status %d",
                    param_id, status);
            goto exit;
    }

    if (status)
        QAL_ERR(LOG_TAG, "Failed to get parameters, param id %d, status %d",
                param_id, status);

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::Open(Stream *s)
{
    int32_t status = 0;

    status = session_->open(s);

    return status;
}

int32_t SoundTriggerEngineGsl::Close(Stream *s)
{
    int32_t status = 0;

    status = session_->close(s);

    return status;
}

int32_t SoundTriggerEngineGsl::Prepare(Stream *s)
{
    int32_t status = 0;

    status = session_->prepare(s);

    return status;
}

int32_t SoundTriggerEngineGsl::SetConfig(Stream * s, configType type, int tag)
{
    int32_t status = 0;

    status = session_->setConfig(s, type, tag);

    return status;
}

int32_t SoundTriggerEngineGsl::ConnectSessionDevice(
    Stream* stream_handle,
    qal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_connect)
{
    int32_t status = 0;

    status = session_->connectSessionDevice(stream_handle, stream_type,
                                            device_to_connect);

    return status;
}

int32_t SoundTriggerEngineGsl::DisconnectSessionDevice(
    Stream* stream_handle,
    qal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_disconnect)
{
    int32_t status = 0;

    status = session_->disconnectSessionDevice(stream_handle, stream_type,
                                               device_to_disconnect);

    return status;
}

void SoundTriggerEngineGsl::SetCaptureRequested(bool is_requested)
{
    QAL_DBG(LOG_TAG, "setting capture requested %d", is_requested);
    capture_requested_ = is_requested;
}

