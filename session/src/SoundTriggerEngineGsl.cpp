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
#include "kvh2xml.h"

// TODO: Move to sound trigger xml files
#define HIST_BUFFER_DURATION_MS 1750
#ifdef FEATURE_IPQ_OPENWRT
#define PRE_ROLL_DURATION_IN_MS 500
#else
#define PRE_ROLL_DURATION_IN_MS 250
#endif
#define DWNSTRM_SETUP_DURATION_MS 300

std::shared_ptr<SoundTriggerEngineGsl> SoundTriggerEngineGsl::sndEngGsl_ = NULL;

void SoundTriggerEngineGsl::buffer_thread_loop()
{
    QAL_INFO(LOG_TAG, "Enter. start thread loop");
    std::unique_lock<std::mutex> lck(sndEngGsl_->mutex_);
    while (!sndEngGsl_->exit_thread_) {
        QAL_VERBOSE(LOG_TAG, "waiting on cond");
        /* Wait for keyword buffer data from DSP */
        if (!sndEngGsl_->eventDetected || sndEngGsl_->exit_buffering_)
            sndEngGsl_->cv_.wait(lck);
        QAL_INFO(LOG_TAG, "done waiting on cond, exit_buffering = %d, exit_thread = %d",
            sndEngGsl_->exit_buffering_, sndEngGsl_->exit_thread_);

        if (sndEngGsl_->exit_thread_) {
            QAL_VERBOSE(LOG_TAG, "Exit buffer thread");
            break;
        }

        if (sndEngGsl_->exit_buffering_) {
            continue; /* skip over processing if we want to exit already*/
        }

        if (sndEngGsl_->start_buffering()) {
            sndEngGsl_->eventDetected = false;
        }
    }
    QAL_DBG(LOG_TAG, "Exit.");
}

static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineGsl::prepare_sound_engine()
{
    int32_t status = 0;
    return status;
}

int32_t SoundTriggerEngineGsl::start_sound_engine()
{
    int32_t status = 0;
    eventDetected = false;
    exit_thread_ = false;
    exit_buffering_ = false;
    bufferThreadHandler_ = std::thread(SoundTriggerEngineGsl::buffer_thread_loop);

    if (!bufferThreadHandler_.joinable()) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "failed to create buffer thread, status = %d", status);
    }
    return status;
}

int32_t SoundTriggerEngineGsl::stop_sound_engine()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    {
        eventDetected = false;
        std::lock_guard<std::mutex> lck(sndEngGsl_->mutex_);
        QAL_VERBOSE(LOG_TAG, "Event lock acquired");
        mutex.lock();
        exit_thread_ = true;
        exit_buffering_ = true;
        timestampRecorded = false;
        cv_.notify_one();
    }
    bufferThreadHandler_.join();
    QAL_INFO(LOG_TAG, "Thread joined");
    mutex.unlock();
    QAL_DBG(LOG_TAG, "Exit.");
    return status;
}

int32_t SoundTriggerEngineGsl::start_buffering()
{
    QAL_DBG(LOG_TAG, "Enter.");
    int32_t status = 0;
    int32_t size;
    struct qal_buffer buf;
    size_t inputBufSize;
    size_t inputBufNum;
    size_t outputBufSize;
    size_t outputBufNum;
    size_t toWrite = 0;
    uint64_t timestamp = 0;
    uint64_t startTs = 0;
    uint64_t endTs = 0;
    size_t startIndice = 0;
    size_t endIndice = 0;
    struct detection_event_info *info;

    mutex.lock();
    if (exit_buffering_ || !eventDetected) {
        mutex.unlock();
        return status;
    }

    streamHandle->getBufInfo(&inputBufSize, &inputBufNum, &outputBufSize,
                             &outputBufNum);
    memset(&buf, 0, sizeof(struct qal_buffer));
    buf.size = inputBufSize * inputBufNum;
    buf.buffer = (uint8_t *)calloc(1, buf.size);
    if (!buf.buffer) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "buffer calloc failed, status %d", status);
        goto exit;
    }

    buf.ts = (struct timespec *)calloc(1, sizeof(struct timespec));
    if (!buf.ts) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "buffer calloc failed, status %d", status);
        goto exit;
    }

    /*TODO: add max retry num to avoid dead lock*/
    QAL_VERBOSE(LOG_TAG, "trying to read %u from gsl", buf.size);

    // read data from session
    if (!toWrite && buffer_->getFreeSize() >= buf.size) {
        status = session->read(streamHandle, SHMEM_ENDPOINT, &buf, &size);
        QAL_INFO(LOG_TAG, "%d read from session, %u to be read", size, buf.size);
        toWrite = size;
    }

    // write data to ring buffer
    if (toWrite) {
        if (!timestampRecorded) {
            timestamp = ((uint64_t)buf.ts->tv_sec * 1000000000 +
                        (uint64_t)buf.ts->tv_nsec) / 1000;
            StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>(streamHandle);
            s->getDetectionEventInfo(&info);
            startTs = (uint64_t)info->kw_start_timestamp_lsw +
                      (uint64_t)(info->kw_start_timestamp_msw << 32);
            endTs = (uint64_t)info->kw_end_timestamp_lsw +
                    (uint64_t)(info->kw_end_timestamp_msw << 32);
            startIndice = us_to_bytes(startTs - timestamp);
            endIndice = us_to_bytes(endTs - timestamp);
            buffer_->updateIndices(startIndice, endIndice);
            timestampRecorded = true;
            s->setDetectionState(GMM_DETECTED);
        }
        size_t ret = buffer_->write(buf.buffer, toWrite);
        toWrite -= ret;
        QAL_INFO(LOG_TAG, "%u written to ring buffer", ret);
    }

exit:
    if (buf.buffer)
        free(buf.buffer);
    if (buf.ts)
        free(buf.ts);
    QAL_DBG(LOG_TAG, "Exit. status %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::start_keyword_detection()
{
    int32_t status = 0;
    return status;
}

SoundTriggerEngineGsl::SoundTriggerEngineGsl(Stream *s, uint32_t id,
    uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer)
{
    struct qal_stream_attributes sAttr;
    uint32_t sampleRate;
    uint32_t bitWidth;
    uint32_t channels;
    uint32_t bufferSize = DEFAULT_QAL_RING_BUFFER_SIZE;
    engineId = id;
    stageId = stage_id;
    eventDetected = false;
    exit_thread_ = false;
    exit_buffering_ = false;
    timestampRecorded = false;
    sndEngGsl_ = (std::shared_ptr<SoundTriggerEngineGsl>)this;
    s->getAssociatedSession(&session);
    streamHandle = s;
    sm_data = NULL;
    pSetupDuration = NULL;
    pSoundModel = NULL;
    QAL_DBG(LOG_TAG, "Enter.");
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
        reader_ = NULL;
        *reader = buffer_->newReader();
    } else {
        // Avoid this engine write data to existing ring buffer
        buffer_ = NULL;
        reader_ = buffer->newReader();
    }
    QAL_DBG(LOG_TAG, "Exit.");
}

SoundTriggerEngineGsl::~SoundTriggerEngineGsl()
{
    QAL_DBG(LOG_TAG, "Enter.");
    if (sm_data)
        free(sm_data);

    if (pSetupDuration)
        free(pSetupDuration);
    QAL_DBG(LOG_TAG, "Exit.");
}

int32_t SoundTriggerEngineGsl::load_sound_model(Stream *s, uint8_t *data,
                                                uint32_t num_models)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;
    SML_BigSoundModelTypeV3 *big_sm;
    uint8_t *sm_payload = NULL;

    mutex.lock();
    if (!data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound model data status %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter.");
    common_sm = (struct qal_st_sound_model *)data;
    if (common_sm->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct qal_st_phrase_sound_model *)data;
        if (num_models > 1) {
            sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
            for (int i = 0; i < num_models; i++) {
                big_sm = (SML_BigSoundModelTypeV3 *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) + (i * sizeof(SML_BigSoundModelTypeV3)));

                if (big_sm->type == ST_SM_ID_SVA_GMM) {
                    sm_data_size = big_sm->size + sizeof(struct qal_st_phrase_sound_model);
                    sm_data = (uint8_t *)malloc(sm_data_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "sm_data malloc failed, status %d", status);
                        goto exit;
                    }
                    casa_osal_memcpy(sm_data, sizeof(*phrase_sm), (char *)phrase_sm,
                                     sizeof(*phrase_sm));
                    common_sm = (struct qal_st_sound_model *)sm_data;
                    common_sm->data_size = big_sm->size;
                    common_sm->data_offset += sizeof(SML_GlobalHeaderType) +
                                              sizeof(SML_HeaderTypeV3) +
                        (num_models * sizeof(SML_BigSoundModelTypeV3)) + big_sm->offset;
                    casa_osal_memcpy(sm_data + sizeof(*phrase_sm), common_sm->data_size,
                           (char *)phrase_sm + common_sm->data_offset,
                           common_sm->data_size);
                    common_sm->data_offset = sizeof(struct qal_st_phrase_sound_model);
                    common_sm = (struct qal_st_sound_model *)data;
                    break;
                }
            }
        } else {
            sm_data_size = common_sm->data_size +
                       sizeof(struct qal_st_phrase_sound_model);
            sm_data = (uint8_t *)malloc(sm_data_size);
            if (!sm_data) {
                status = -ENOMEM;
                QAL_ERR(LOG_TAG, "sm_data malloc failed, status %d", status);
                goto exit;
            }
            casa_osal_memcpy(sm_data, sizeof(*phrase_sm), (char *)phrase_sm,
                             sizeof(*phrase_sm));
            casa_osal_memcpy(sm_data + sizeof(*phrase_sm), phrase_sm->common.data_size,
                   (char *)phrase_sm + phrase_sm->common.data_offset,
                   phrase_sm->common.data_size);
        }
    } else if (common_sm->type == QAL_SOUND_MODEL_TYPE_GENERIC) {
        sm_data_size = common_sm->data_size +
                       sizeof(struct qal_st_sound_model);
        sm_data = (uint8_t *)malloc(sm_data_size);
        if (!sm_data) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "sm_data malloc failed, status %d", status);
            goto exit;
        }
        casa_osal_memcpy(sm_data, sizeof(*common_sm), (char *)common_sm,
                         sizeof(*common_sm));
        casa_osal_memcpy(sm_data + sizeof(*common_sm), common_sm->data_size,
               (char *)common_sm + common_sm->data_offset,
               common_sm->data_size);
    }
    pSoundModel = (struct qal_st_sound_model *)sm_data;

    status = session->setParameters(streamHandle, DEVICE_SVA,
                                    PARAM_ID_DETECTION_ENGINE_SOUND_MODEL,
                                    (void *)pSoundModel);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to load sound model, status = %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Exit. Load sound model success status %d", status);
    mutex.unlock();
    return status;

exit:
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::unload_sound_model(Stream *s)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");

    mutex.lock();
    if (!sm_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,  "No sound model can be unloaded, status %d", status);
        goto exit;
    }
    free(sm_data);
    sm_data_size = 0;

    QAL_DBG(LOG_TAG, "Exit. Unload sound model success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to unload sound model, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::start_recognition(Stream *s)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    mutex.lock();
    status = session->setParameters(streamHandle, DEVICE_SVA,
                                    PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP,
                                    &pWakeUpConfig);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set wake up config, status = %d", status);
        goto exit;
    }

    status = session->setParameters(streamHandle, DEVICE_SVA,
                                    PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG,
                                    &pEventConfig);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set event config, status = %d", status);
        goto exit;
    }

    status = session->setParameters(streamHandle, DEVICE_SVA,
                                    PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG,
                                    &pBufConfig);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set wake-up buffer config, status = %d", status);
        goto exit;
    }

    status = session->setParameters(streamHandle, DEVICE_ADAM,
                                    PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION,
                                    pSetupDuration);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to set downstream setup duration, status = %d", status);
        goto exit;
    }

    status = start_sound_engine();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Exit. start recognition success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to start recognition, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::stop_buffering(Stream *s)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    mutex.lock();
    eventDetected = false;
    exit_buffering_ = true;
    timestampRecorded = false;
    if (buffer_)
        buffer_->reset();
    else {
        status = -EINVAL;
        goto exit;
    }

    status = session->setParameters(streamHandle, DEVICE_SVA, PARAM_ID_DETECTION_ENGINE_RESET, NULL);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d", status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "stop buffering success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to stop buffering, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::stop_recognition(Stream *s)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    status = stop_sound_engine();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to stop sound engine, status = %d", status);
        goto exit;
    }

    mutex.lock();
    status = session->setParameters(streamHandle, DEVICE_SVA, PARAM_ID_DETECTION_ENGINE_RESET, NULL);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d", status);
        goto exit;
    }

    if (buffer_)
        buffer_->reset();
    else {
        status = -EINVAL;
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Exit. stop recognition success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG,  "Failed to stop recognition, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineGsl::update_config(Stream *s, struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    size_t config_size;
    size_t num_output_ports;
    uint32_t size;

    mutex.lock();
    if (!config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid config, status %d", status);
        goto exit;
    }

    status = generate_wakeup_config(config);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to generate wakeup config status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Enter. config: %pK", config);

    // event mode indicates info provided by DSP when event detected
    // TODO: set this from StreamSoundTrigger
    pEventConfig.event_mode = CONFIDENCE_LEVEL_INFO |
                              KEYWORD_INDICES_INFO |
                              TIME_STAMP_INFO;/* |
                              FTRT_INFO;*/

    pBufConfig.hist_buffer_duration_in_ms = HIST_BUFFER_DURATION_MS;
    pBufConfig.pre_roll_duration_in_ms = PRE_ROLL_DURATION_IN_MS;

    num_output_ports = 1;
    size = sizeof(struct audio_dam_downstream_setup_duration) +
                  num_output_ports * sizeof(struct audio_dam_downstream_setup_duration_t);
    pSetupDuration = (struct audio_dam_downstream_setup_duration *)calloc(1, size);
    if (!pSetupDuration) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "Failed to allocate pSetupDuration, status %d", status);
        goto exit;
    }
    pSetupDuration->num_output_ports = num_output_ports;

    for (int i = 0; i < pSetupDuration->num_output_ports; i++) {
        pSetupDuration->port_cfgs[i].output_port_id = 1;
        pSetupDuration->port_cfgs[i].dwnstrm_setup_duration_ms = DWNSTRM_SETUP_DURATION_MS;
    }

    QAL_DBG(LOG_TAG, "Exit. Update config success");
    mutex.unlock();
    return status;

exit:
    QAL_DBG(LOG_TAG, "Exit. Update config failure");
    mutex.unlock();
    return status;
}

void SoundTriggerEngineGsl::setDetected(bool detected)
{
    QAL_INFO(LOG_TAG, "setDetected %d", detected);
    mutex.lock();
    std::lock_guard<std::mutex> lck(sndEngGsl_->mutex_);
    if (detected != eventDetected) {
        QAL_INFO(LOG_TAG, "notify condition variable");
        eventDetected = detected;
        exit_buffering_ = !eventDetected;
        QAL_INFO(LOG_TAG, "eventDetected set to %d", detected);
        cv_.notify_one();
    }
    else
        QAL_VERBOSE(LOG_TAG, "eventDetected unchanged");
    mutex.unlock();
}

int32_t SoundTriggerEngineGsl::generate_wakeup_config(struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    unsigned int num_conf_levels = 0;
    unsigned int user_level, user_id;
    unsigned int i = 0, j = 0;
    unsigned char *conf_levels = NULL;
    unsigned char *user_id_tracker = NULL;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;

    phrase_sm = (struct qal_st_phrase_sound_model *)pSoundModel;

    QAL_DBG(LOG_TAG, "Enter.");

    if (!phrase_sm || !config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "invalid input status %d", status);
        goto exit;
    }

    if ((config->num_phrases == 0) ||
        (config->num_phrases > phrase_sm->num_phrases)) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid phrase data status %d", status);
        goto exit;
    }

    for (i = 0; i < config->num_phrases; i++) {
        num_conf_levels++;
        for (j = 0; j < config->phrases[i].num_levels; j++)
            num_conf_levels++;
    }

    conf_levels = (unsigned char*)calloc(1, num_conf_levels);
    if (!conf_levels) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
        goto exit;
    }

    user_id_tracker = (unsigned char *) calloc(1, num_conf_levels);
    if (!user_id_tracker) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d", status);
        goto exit;
    }

    /* for debug */
    for (i = 0; i < config->num_phrases; i++) {
        QAL_VERBOSE(LOG_TAG, "[%d] kw level %d", i,
        config->phrases[i].confidence_level);
        for (j = 0; j < config->phrases[i].num_levels; j++) {
            QAL_VERBOSE(LOG_TAG, "[%d] user_id %d level %d ", i,
                        config->phrases[i].levels[j].user_id,
                        config->phrases[i].levels[j].level);
        }
    }

/* Example: Say the recognition structure has 3 keywords with users
        [0] k1 |uid|
                [0] u1 - 1st trainer
                [1] u2 - 4th trainer
                [3] u3 - 3rd trainer
        [1] k2
                [2] u2 - 2nd trainer
                [4] u3 - 5th trainer
        [2] k3
                [5] u4 - 6th trainer

      Output confidence level array will be
      [k1, k2, k3, u1k1, u2k1, u2k2, u3k1, u3k2, u4k3]
*/
    for (i = 0; i < config->num_phrases; i++) {
        conf_levels[i] = config->phrases[i].confidence_level;
        for (j = 0; j < config->phrases[i].num_levels; j++) {
            user_level = config->phrases[i].levels[j].level;
            user_id = config->phrases[i].levels[j].user_id;
            if ((user_id < config->num_phrases) ||
                (user_id >= num_conf_levels)) {
                status = -EINVAL;
                QAL_ERR(LOG_TAG, "Invalid params user id %d status %d", user_id,
                        status);
                goto exit;
            }
            else {
                if (user_id_tracker[user_id] == 1) {
                    status = -EINVAL;
                    QAL_ERR(LOG_TAG, "Duplicate user id %d status %d", user_id,
                            status);
                    goto exit;
                }
                conf_levels[user_id] = (user_level < 100) ? user_level : 100;
                user_id_tracker[user_id] = 1;
                QAL_VERBOSE(LOG_TAG, "user_conf_levels[%d] = %d",user_id,
                            conf_levels[user_id]);
            }
        }
    }

    pWakeUpConfig.mode = config->phrases[0].recognition_modes;
    pWakeUpConfig.custom_payload_size = config->data_size;
    pWakeUpConfig.num_active_models = num_conf_levels;
    pWakeUpConfig.reserved = 0;
    for (int i = 0; i < pWakeUpConfig.num_active_models; i++) {
        pWakeUpConfig.confidence_levels[i] = conf_levels[i];
        pWakeUpConfig.keyword_user_enables[i] = 1;
    }
exit:
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);
    if (conf_levels)
        free(conf_levels);
    if (user_id_tracker)
        free(user_id_tracker);
    return status;
}

int32_t SoundTriggerEngineGsl::getParameters(uint32_t param_id, void **payload)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");

    switch (param_id) {
        case QAL_PARAM_ID_DIRECTION_OF_ARRIVAL:
            status = session->getParameters(streamHandle, TAG_FLUENCE,
                                            param_id, payload);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Unsupported param id %u status %d",
                    param_id, status);
            goto exit;
    }

    if (status)
        QAL_ERR(LOG_TAG, "Failed to get parameters, param_id %d, status %d",
                param_id, status);

exit:
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);
    return status;
}
