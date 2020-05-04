/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QAL: SoundTriggerEngineGsl"

#include "SoundTriggerEngineGsl.h"

#include "Session.h"
#include "SessionGsl.h"
#include "Stream.h"
#include "StreamSoundTrigger.h"
#include "ResourceManager.h"
#include "kvh2xml.h"

// TODO: find another way to print debug logs by default
#define ST_DBG_LOGS
#ifdef ST_DBG_LOGS
#define QAL_DBG(...)  QAL_INFO(__VA_ARGS__)
#endif

// TODO: Move to sound trigger xml files
#define HIST_BUFFER_DURATION_MS 1750
#ifdef FEATURE_IPQ_OPENWRT
#define PRE_ROLL_DURATION_IN_MS 500
#else
#define PRE_ROLL_DURATION_IN_MS 250
#endif
#define DWNSTRM_SETUP_DURATION_MS 300

int SoundTriggerEngineGsl::instance_count_ = 0;

void SoundTriggerEngineGsl::EventProcessingThread(
    SoundTriggerEngineGsl *gsl_engine) {

    int32_t status = 0;

    QAL_INFO(LOG_TAG, "Enter. start thread loop");
    if (!gsl_engine) {
        QAL_ERR(LOG_TAG, "Invalid sound trigger engine");
        return;
    }

    std::unique_lock<std::mutex> lck(gsl_engine->mutex_);
    while (!gsl_engine->exit_thread_) {
        QAL_VERBOSE(LOG_TAG, "waiting on cond");
        gsl_engine->cv_.wait(lck);
        QAL_DBG(LOG_TAG, "done waiting on cond");

        if (gsl_engine->exit_thread_) {
            QAL_VERBOSE(LOG_TAG, "Exit thread");
            break;
        }
        if (gsl_engine->capture_requested_) {
            gsl_engine->StartBuffering();
        } else {
            // TODO: Work around to resume further detections.
            QAL_DBG(LOG_TAG, "HandleSessionEvent: reset engine");
            status = gsl_engine->session_->setParameters(
                gsl_engine->stream_handle_,
                DEVICE_SVA,
                PARAM_ID_DETECTION_ENGINE_RESET,
                nullptr);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d",
                        status);
            }
            StreamSoundTrigger *s =
                dynamic_cast<StreamSoundTrigger *>(gsl_engine->stream_handle_);
            lck.unlock();
            s->SetEngineDetectionState(GMM_DETECTED);
            lck.lock();
        }
    }
    QAL_DBG(LOG_TAG, "Exit");
}

static uint32_t us_to_bytes(uint64_t input_us) {
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineGsl::StartBuffering() {
    int32_t status = 0;
    int32_t size = 0;
    struct qal_buffer buf;
    size_t input_buf_size = 0;
    size_t input_buf_num = 0;
    size_t output_buf_size = 0;
    size_t output_buf_num = 0;
    uint64_t timestamp = 0;
    uint64_t start_timestamp = 0;
    uint64_t end_timestamp = 0;
    size_t start_index = 0;
    size_t end_index = 0;
    bool timestamp_recorded = false;
    StreamSoundTrigger *s = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    stream_handle_->getBufInfo(&input_buf_size, &input_buf_num,
                               &output_buf_size, &output_buf_num);
    std::memset(&buf, 0, sizeof(struct qal_buffer));
    buf.size = input_buf_size * input_buf_num;
    buf.buffer = (uint8_t *)calloc(1, buf.size);
    if (!buf.buffer) {
        QAL_ERR(LOG_TAG, "buf.buffer allocation failed");
        status = -ENOMEM;
        goto exit;
    }
    buf.ts = (struct timespec *)calloc(1, sizeof(struct timespec));
    if (!buf.ts) {
        QAL_ERR(LOG_TAG, "buf.ts allocation failed");
        status = -ENOMEM;
        goto exit;
    }
    buffer_->reset();

    while (!exit_buffering_) {
        QAL_VERBOSE(LOG_TAG, "request read %u from gsl", buf.size);
        // read data from session
        if (buffer_->getFreeSize() >= buf.size) {
            status = session_->read(stream_handle_, SHMEM_ENDPOINT, &buf, &size);
            if (status) {
                break;
            }
            QAL_INFO(LOG_TAG, "requested %u, read %d", buf.size, size);
        }
        // write data to ring buffer
        if (size) {
            size_t ret = buffer_->write(buf.buffer, size);
            QAL_INFO(LOG_TAG, "%u written to ring buffer", ret);

            if (!timestamp_recorded) {
                timestamp = ((uint64_t)buf.ts->tv_sec * 1000000000 +
                            (uint64_t)buf.ts->tv_nsec) / 1000;

                start_timestamp =
                    (uint64_t)detection_event_info_.kw_start_timestamp_lsw +
                    ((uint64_t)detection_event_info_.kw_start_timestamp_msw << 32);
                end_timestamp =
                    (uint64_t)detection_event_info_.kw_end_timestamp_lsw +
                    ((uint64_t)detection_event_info_.kw_end_timestamp_msw << 32);
                start_index = us_to_bytes(start_timestamp - timestamp);
                end_index = us_to_bytes(end_timestamp - timestamp);
                buffer_->updateIndices(start_index, end_index);
                timestamp_recorded = true;
                // Notify detection to client
                s = dynamic_cast<StreamSoundTrigger *>(stream_handle_);
                mutex_.unlock();
                s->SetEngineDetectionState(GMM_DETECTED);
                mutex_.lock();
            }
        }
    }

exit:
    if (buf.buffer) {
        free(buf.buffer);
    }
    if (buf.ts) {
        free(buf.ts);
    }
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::ParseDetectionPayload(void *event_data) {
    int32_t status = 0;
    int32_t i = 0;
    uint32_t parsed_size = 0;
    uint32_t payload_size = 0;
    uint32_t event_size = 0;
    uint8_t *ptr = nullptr;
    struct event_id_detection_engine_generic_info_t *generic_info = nullptr;
    struct detection_event_info_header_t *event_header = nullptr;
    struct confidence_level_info_t *confidence_info = nullptr;
    struct keyword_position_info_t *keyword_position_info = nullptr;
    struct detection_timestamp_info_t *detection_timestamp_info = nullptr;
    struct ftrt_data_info_t *ftrt_info = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    if (!event_data) {
        QAL_ERR(LOG_TAG, "Invalid event data");
        return -EINVAL;
    }

    std::memset(&detection_event_info_, 0, sizeof(struct detection_event_info));

    generic_info =
        (struct event_id_detection_engine_generic_info_t *)event_data;
    payload_size = sizeof(struct event_id_detection_engine_generic_info_t);
    detection_event_info_.status = generic_info->status;
    event_size = generic_info->payload_size;
    ptr = (uint8_t *)event_data + payload_size;
    QAL_INFO(LOG_TAG, "status = %u, event_size = %u",
             detection_event_info_.status, event_size);
    if (status || !event_size) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid detection payload");
        goto exit;
    }

    // parse variable payload
    while (parsed_size < event_size) {
        QAL_DBG(LOG_TAG, "parsed_size = %u, event_size = %u",
                parsed_size, event_size);
        event_header = (struct detection_event_info_header_t *)ptr;
        uint32_t keyId = event_header->key_id;
        payload_size = event_header->payload_size;
        QAL_DBG(LOG_TAG, "key id = %u, payload_size = %u",
                keyId, payload_size);
        ptr += sizeof(struct detection_event_info_header_t);
        parsed_size += sizeof(struct detection_event_info_header_t);

        switch (keyId) {
        case KEY_ID_CONFIDENCE_LEVELS_INFO:
            confidence_info = (struct confidence_level_info_t *)ptr;
            detection_event_info_.num_confidence_levels =
                confidence_info->number_of_confidence_values;
            QAL_DBG(LOG_TAG, "num_confidence_levels = %u",
                    detection_event_info_.num_confidence_levels);
            for (i = 0; i < detection_event_info_.num_confidence_levels; i++) {
                detection_event_info_.confidence_levels[i] =
                    confidence_info->confidence_levels[i];
                QAL_VERBOSE(LOG_TAG, "confidence_levels[%d] = %u", i,
                            detection_event_info_.confidence_levels[i]);
            }
            break;
        case KEY_ID_KWD_POSITION_INFO:
            keyword_position_info = (struct keyword_position_info_t *)ptr;
            detection_event_info_.kw_start_timestamp_lsw =
                keyword_position_info->kw_start_timestamp_lsw;
            detection_event_info_.kw_start_timestamp_msw =
                keyword_position_info->kw_start_timestamp_msw;
            detection_event_info_.kw_end_timestamp_lsw =
                keyword_position_info->kw_end_timestamp_lsw;
            detection_event_info_.kw_end_timestamp_msw =
                keyword_position_info->kw_end_timestamp_msw;
            QAL_DBG(LOG_TAG, "start_lsw = %u, start_msw = %u, "
                    "end_lsw = %u, end_msw = %u",
                    detection_event_info_.kw_start_timestamp_lsw,
                    detection_event_info_.kw_start_timestamp_msw,
                    detection_event_info_.kw_end_timestamp_lsw,
                    detection_event_info_.kw_end_timestamp_msw);
            break;
        case KEY_ID_TIMESTAMP_INFO:
            detection_timestamp_info = (struct detection_timestamp_info_t *)ptr;
            detection_event_info_.detection_timestamp_lsw =
                detection_timestamp_info->detection_timestamp_lsw;
            detection_event_info_.detection_timestamp_msw =
                detection_timestamp_info->detection_timestamp_msw;
            QAL_DBG(LOG_TAG, "timestamp_lsw = %u, timestamp_msw = %u",
                    detection_event_info_.detection_timestamp_lsw,
                    detection_event_info_.detection_timestamp_msw);
            break;
        case KEY_ID_FTRT_DATA_INFO:
            ftrt_info = (struct ftrt_data_info_t *)ptr;
            detection_event_info_.ftrt_data_length_in_us =
                ftrt_info->ftrt_data_length_in_us;
            QAL_DBG(LOG_TAG, "ftrt_data_length_in_us = %u",
                    detection_event_info_.ftrt_data_length_in_us);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid key id %u status %d", keyId, status);
            goto exit;
        }
        ptr += payload_size;
        parsed_size += payload_size;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

SoundTriggerEngineGsl::SoundTriggerEngineGsl(
    Stream *s,
    uint32_t id,
    uint32_t stage_id) {

    struct qal_stream_attributes sAttr;
    std::shared_ptr<ResourceManager> rm = nullptr;
    engine_id_ = id;
    stage_id_ = stage_id;
    exit_thread_ = false;
    exit_buffering_ = false;
    capture_requested_ = false;
    stream_handle_ = s;
    sm_data_ = nullptr;
    dam_setup_duration_ = nullptr;
    reader_ = nullptr;
    buffer_ = nullptr;

    std::memset(&detection_event_info_, 0, sizeof(struct detection_event_info));

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

    instance_count_++;

    QAL_DBG(LOG_TAG, "Exit");
}

SoundTriggerEngineGsl::~SoundTriggerEngineGsl() {
    QAL_INFO(LOG_TAG, "Enter");
    if (dam_setup_duration_) {
        free(dam_setup_duration_);
    }
    if (buffer_) {
        delete buffer_;
    }
    if (reader_) {
        delete reader_;
    }
    instance_count_--;
    QAL_INFO(LOG_TAG, "Exit");
}

int32_t SoundTriggerEngineGsl::LoadSoundModel(Stream *s, uint8_t *data,
                                              uint32_t data_size) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    if (!data) {
        QAL_ERR(LOG_TAG, "Invalid sound model data status %d", status);
        status = -EINVAL;
        return status;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    status = session_->open(s);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to open session, status = %d", status);
        return status;

    }
    status = session_->setParameters(stream_handle_, DEVICE_SVA,
                                     PARAM_ID_DETECTION_ENGINE_SOUND_MODEL,
                                     (void *)data);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to load sound model, status = %d", status);
        session_->close(s);
        return status;
    }
    sm_data_ = data;
    sm_data_size_ = data_size;

    exit_thread_ = false;
    buffer_thread_handler_ =
        std::thread(SoundTriggerEngineGsl::EventProcessingThread, this);

    if (!buffer_thread_handler_.joinable()) {
        QAL_ERR(LOG_TAG, "failed to create even processing thread, status = %d",
                status);
        session_->close(s);
        status = -EINVAL;
    }
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::UnloadSoundModel(Stream *s) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::unique_lock<std::mutex> lck(mutex_);
    exit_thread_ = true;
    exit_buffering_ = true;
    if (buffer_thread_handler_.joinable()) {
        cv_.notify_one();
        lck.unlock();
        buffer_thread_handler_.join();
        lck.lock();
        QAL_INFO(LOG_TAG, "Thread joined");
    }
    status = session_->close(s);
    if (status)
        QAL_ERR(LOG_TAG, "Failed to close session, status = %d", status);
    sm_data_ = nullptr;
    sm_data_size_ = 0;

    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StartRecognition(Stream *s __unused) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    exit_buffering_ = false;

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

    status = UpdateDAMSetupDuration(instance_count_);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to update dam setup duration, status = %d",
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

int32_t SoundTriggerEngineGsl::RestartRecognition(Stream *s __unused) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    exit_buffering_ = false;
    /*
     * TODO: This sequence RESET->STOP->START is currently required from spf
     * as ENGINE_RESET alone can't reset the graph (including DAM etc..) ready
     * for next detection.
     */
    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_RESET,
        nullptr);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to reset engine, status = %d",
                status);
    }
    status = session_->stop(stream_handle_);
    if (!status) {
        status = session_->start(stream_handle_);
        if (status) {
            QAL_ERR(LOG_TAG, "start session failed, status = %d",
                    status);
        }
    } else {
        QAL_ERR(LOG_TAG, "stop session failed, status = %d",
                status);
    }
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StopBuffering(Stream *s __unused) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    exit_buffering_ = true;
    std::lock_guard<std::mutex> lck(mutex_);
    if (buffer_) {
        buffer_->reset();
    }
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StopRecognition(Stream *s __unused) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    exit_buffering_ = true;
    std::lock_guard<std::mutex> lck(mutex_);
    if (buffer_) {
        buffer_->reset();
    }
    /*
     * TODO: Currently spf requires ENGINE_RESET to close the DAM gate as stop
     * will not close the gate, rather just flushes the buffers, resulting in no
     * further detections.
     */
    status = session_->setParameters(
        stream_handle_,
        DEVICE_SVA,
        PARAM_ID_DETECTION_ENGINE_RESET,
        nullptr);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d",
                status);
    }

    status = session_->stop(stream_handle_);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to stop session, status = %d", status);
    }

    QAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::UpdateConfLevels(
    Stream *s __unused,
    struct qal_st_recognition_config *config,
    uint8_t *conf_levels,
    uint32_t num_conf_levels) {

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

int32_t SoundTriggerEngineGsl::UpdateBufConfig(
    uint32_t hist_buffer_duration,
    uint32_t pre_roll_duration) {

    int32_t status = 0;

    buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
    buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;

    return status;
}

void SoundTriggerEngineGsl::HandleSessionEvent(uint32_t event_id __unused,
                                               void *data) {
    int32_t status = 0;

    std::unique_lock<std::mutex> lck(mutex_);
    status = ParseDetectionPayload(data);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to parse detection payload, status %d",
                status);
        return;
    }
    QAL_INFO(LOG_TAG, "singal event processing thread");
    cv_.notify_one();
}

void SoundTriggerEngineGsl::HandleSessionCallBack(void *hdl, uint32_t event_id,
                                                  void *data) {
    SoundTriggerEngineGsl *engine = nullptr;

    QAL_DBG(LOG_TAG, "Enter, event detected on SPF, event id = 0x%x", event_id);
    if (!hdl || !data) {
        QAL_ERR(LOG_TAG, "Invalid engine handle or event data");
        return;
    }

    // Possible that AGM_EVENT_EOS_RENDERED could be sent during spf stop.
    // Check and handle only required detection event.
    if (event_id != EVENT_ID_DETECTION_ENGINE_GENERIC_INFO)
        return;

    engine = (SoundTriggerEngineGsl *)hdl;
    engine->HandleSessionEvent(event_id, data);

    QAL_DBG(LOG_TAG, "Exit");
    return;
}

int32_t SoundTriggerEngineGsl::GetParameters(uint32_t param_id,
                                             void **payload) {
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

    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get parameters, param id %d, status %d",
                param_id, status);
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::ConnectSessionDevice(
    Stream* stream_handle,
    qal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_connect) {

    int32_t status = 0;

    status = session_->connectSessionDevice(stream_handle, stream_type,
                                            device_to_connect);

    return status;
}

int32_t SoundTriggerEngineGsl::DisconnectSessionDevice(
    Stream* stream_handle,
    qal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_disconnect) {

    int32_t status = 0;

    status = session_->disconnectSessionDevice(stream_handle, stream_type,
                                               device_to_disconnect);

    return status;
}

int32_t SoundTriggerEngineGsl::SetupSessionDevice(
    Stream* stream_handle,
    qal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_disconnect) {

    int32_t status = 0;

    status = session_->setupSessionDevice(stream_handle, stream_type,
                                          device_to_disconnect);

    return status;
}

void SoundTriggerEngineGsl::SetCaptureRequested(bool is_requested) {
    QAL_DBG(LOG_TAG, "setting capture requested %d", is_requested);
    capture_requested_ = is_requested;
}

struct detection_event_info* SoundTriggerEngineGsl::GetDetectionEventInfo() {
    return &detection_event_info_;
}

int32_t SoundTriggerEngineGsl::setECRef(Stream *s, std::shared_ptr<Device> dev, bool is_enable) {
    if (!session_) {
        QAL_ERR(LOG_TAG, "Invalid session");
        return -EINVAL;
    }

    return session_->setECRef(s, dev, is_enable);
}

int32_t SoundTriggerEngineGsl::GetSetupDuration(
    struct audio_dam_downstream_setup_duration **duration) {
    int32_t status = 0;

    *duration = dam_setup_duration_;

    return status;
}

int32_t SoundTriggerEngineGsl::UpdateDAMSetupDuration(int port_num) {
    uint32_t size = 0;

    if (dam_setup_duration_) {
        if (dam_setup_duration_->num_output_ports == port_num) {
            QAL_DBG(LOG_TAG, "No need to update DAM setup duration");
            return 0;
        } else {
            free(dam_setup_duration_);
            dam_setup_duration_ = nullptr;
        }
    }

    size = sizeof(struct audio_dam_downstream_setup_duration) +
        port_num * sizeof(struct audio_dam_downstream_setup_duration_t);
    dam_setup_duration_ =
        (struct audio_dam_downstream_setup_duration *)calloc(1, size);
    if (!dam_setup_duration_) {
        QAL_ERR(LOG_TAG, "Failed to allocate dam setup duration");
        return -ENOMEM;
    }

    dam_setup_duration_->num_output_ports = port_num;
    for (int i = 0; i < dam_setup_duration_->num_output_ports; i++) {
        dam_setup_duration_->port_cfgs[i].output_port_id = i * 2 + 1;
        dam_setup_duration_->port_cfgs[i].dwnstrm_setup_duration_ms =
            DWNSTRM_SETUP_DURATION_MS;
    }

    return 0;
}
