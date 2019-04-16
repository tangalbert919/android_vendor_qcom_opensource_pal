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

std::shared_ptr<SoundTriggerEngineGsl> SoundTriggerEngineGsl::sndEngGsl_ = NULL;

void SoundTriggerEngineGsl::buffer_thread_loop()
{
    QAL_INFO(LOG_TAG, "start thread loop");
    std::unique_lock<std::mutex> lck(sndEngGsl_->mutex_);
    while (!sndEngGsl_->exit_thread_)
    {
        sndEngGsl_->exit_buffering_ = false;
        QAL_INFO(LOG_TAG,"%s: waiting on cond", __func__);
        /* Wait for keyword buffer data from DSP */
        if (!sndEngGsl_->eventDetected)
            sndEngGsl_->cv_.wait(lck);
        QAL_INFO(LOG_TAG,"%s: done waiting on cond, exit_buffering = %d", __func__,
            sndEngGsl_->exit_buffering_);

        if (sndEngGsl_->exit_thread_) {
            break;
        }

        if (sndEngGsl_->exit_buffering_)
        {
            continue; /* skip over processing if we want to exit already*/
        }

        sndEngGsl_->start_buffering();
    }
}

int32_t SoundTriggerEngineGsl::prepare_sound_engine()
{
    int32_t status = 0;
    return status;
}

int32_t SoundTriggerEngineGsl::start_sound_engine()
{
    int32_t status = 0;
    bufferThreadHandler_ = std::thread(SoundTriggerEngineGsl::buffer_thread_loop);

    if (!bufferThreadHandler_.joinable())
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: failed to create buffer thread = %d", __func__, status);
    }
    return status;
}

int32_t SoundTriggerEngineGsl::stop_sound_engine()
{
    int32_t status = 0;
    std::lock_guard<std::mutex> lck(sndEngGsl_->mutex_);
    exit_thread_ = true;
    exit_buffering_ = true;
    cv_.notify_one();
    bufferThreadHandler_.join();
    return status;
}

int32_t SoundTriggerEngineGsl::start_buffering()
{
    QAL_INFO(LOG_TAG, "Enter");
    int32_t status = 0;
    int32_t size;
    struct qal_buffer buf;
    size_t toWrite = 0;

    memset(&buf, 0, sizeof(struct qal_buffer));
    buf.size = 3840;
    buf.buffer = (uint8_t *)calloc(1, buf.size);

    /*TODO: add max retry num to avoid dead lock*/
    QAL_INFO(LOG_TAG, "trying to read %u from gsl", buf.size);

    // read data from session
    if (!toWrite && buffer_->getFreeSize() >= buf.size)
    {
        status = session->read(streamHandle, SHMEM_ENDPOINT, &buf, &size);
        QAL_INFO(LOG_TAG, "%d read from session, %u to be read", size, buf.size);
        toWrite = size;
    }

    // write data to ring buffer
    if (toWrite)
    {
        size_t ret = buffer_->write(buf.buffer, toWrite);
        toWrite -= ret;
        QAL_INFO(LOG_TAG, "%u written to ring buffer", ret);
    }

    if (buf.buffer)
        free(buf.buffer);
    return status;
}

int32_t SoundTriggerEngineGsl::start_keyword_detection()
{
    int32_t status = 0;
    return status;
}

SoundTriggerEngineGsl::SoundTriggerEngineGsl(Stream *s, uint32_t id, uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer)
{
    engineId = id;
    stageId = stage_id;
    eventDetected = false;
    exit_thread_ = false;
    exit_buffering_ = false;
    sndEngGsl_ = (std::shared_ptr<SoundTriggerEngineGsl>)this;
    s->getAssociatedSession(&session);
    streamHandle = s;

    // Create ring buffer when reader passed is not specified
    if (!buffer)
    {
        QAL_ERR(LOG_TAG, "creating new ring buffer");
        buffer_ = new QalRingBuffer(DEFAULT_QAL_RING_BUFFER_SIZE);
        reader_ = NULL;
        *reader = buffer_->newReader();
        QAL_ERR(LOG_TAG, "reader %p", *reader);
    }
    else
    {
        // Avoid this engine write data to existing ring buffer
        buffer_ = NULL;
        reader_ = buffer->newReader();
    }
}

SoundTriggerEngineGsl::~SoundTriggerEngineGsl()
{
    if (sm_data)
        free(sm_data);

    if (sm_params_data)
        free(sm_params_data);

    if (pSoundModel)
        free(pSoundModel);
}

int32_t SoundTriggerEngineGsl::load_sound_model(Stream *s, uint8_t *data)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;

    if (!data)
    {
        QAL_ERR(LOG_TAG, "%s: Invalid sound model data", __func__);
        status = -EINVAL;
        goto exit;
    }

    common_sm = (struct qal_st_sound_model *)data;
    if (common_sm->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE)
        sm_data_size = common_sm->data_size +
                       sizeof(struct qal_st_phrase_sound_model);
    else if (common_sm->type == QAL_SOUND_MODEL_TYPE_GENERIC)
        sm_data_size = common_sm->data_size +
                       sizeof(struct qal_st_sound_model);
    sm_data = (uint8_t *)malloc(sm_data_size);
    memcpy(sm_data, data, sm_data_size);
    pSoundModel = (struct qal_st_sound_model *)sm_data;

    status = session->setParameters(streamHandle, PARAM_ID_DETECTION_ENGINE_SOUND_MODEL, (void *)pSoundModel);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to load sound model, status = %d", __func__, status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "%s: Load sound model success", __func__);
    return status;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to load sound model, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineGsl::unload_sound_model(Stream *s)
{
    int32_t status = 0;

    if (!sm_data)
    {
        QAL_ERR(LOG_TAG, "%s: No sound model can be unloaded", __func__);
        status = -EINVAL;
        goto exit;
    }

    free(sm_data);
    sm_data_size = 0;

    QAL_VERBOSE(LOG_TAG, "%s: Unload sound model success", __func__);
    return status;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to unload sound model, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineGsl::start_recognition(Stream *s)
{
    int32_t status = 0;

    status = session->setParameters(streamHandle, PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP, &pWakeUpConfig);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to set wake up config, status = %d", __func__, status);
        goto exit;
    }

    status = session->setParameters(streamHandle, PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG, &pEventConfig);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to set event config, status = %d", __func__, status);
        goto exit;
    }

    status = session->setParameters(streamHandle, PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG, &pBufConfig);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to set wake-up buffer config, status = %d", __func__, status);
        goto exit;
    }

    status = session->setParameters(streamHandle, PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION, pSetupDuration);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to set downstream setup duration, status = %d", __func__, status);
        goto exit;
    }

    status = start_sound_engine();
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to start sound engine, status = %d", __func__, status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "%s: start recognition success", __func__);
    return status;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to start recognition, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineGsl::stop_recognition(Stream *s)
{
    int32_t status = 0;

    status = session->setParameters(streamHandle, PARAM_ID_DETECTION_ENGINE_RESET, NULL);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to reset detection engine, status = %d", __func__, status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "%s: stop recognition success", __func__);
    return status;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to stop recognition, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineGsl::update_config(Stream *s, struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    size_t config_size;

    if (!config)
    {
        QAL_ERR(LOG_TAG, "%s: Invalid config", __func__);
        return -EINVAL;
    }

    QAL_ERR(LOG_TAG, "config: %p", config);
    pWakeUpConfig.mode = config->phrases[0].recognition_modes;
    pWakeUpConfig.custom_payload_size = config->data_size;
    pWakeUpConfig.num_active_models = config->num_phrases;
    pWakeUpConfig.reserved = 0;
    for (int i = 0; i < pWakeUpConfig.num_active_models; i++)
    {
        pWakeUpConfig.confidence_levels[i] = config->phrases[i].confidence_level;
        pWakeUpConfig.keyword_user_enables[i] = 1;
    }

    // event mode indicates info provided by DSP when event detected
    // TODO: set this from StreamSoundTrigger
    pEventConfig.event_mode = CONFIDENCE_LEVEL_INFO |
                              KEYWORD_INDICES_INFO |
                              TIME_STAMP_INFO;/* |
                              FTRT_INFO;*/

    pBufConfig.hist_buffer_duration_in_ms = 1750;
    pBufConfig.pre_roll_duration_in_ms = 250;

    size_t num_output_ports = 1;
    uint32_t size = sizeof(struct audio_dam_downstream_setup_duration) +
                    num_output_ports * sizeof(struct audio_dam_downstream_setup_duration_t);
    pSetupDuration = (struct audio_dam_downstream_setup_duration *)calloc(1, size);
    pSetupDuration->num_output_ports = num_output_ports;

    for (int i = 0; i < pSetupDuration->num_output_ports; i++)
    {
        pSetupDuration->port_cfgs[i].output_port_id = 1;
        pSetupDuration->port_cfgs[i].dwnstrm_setup_duration_ms = 300;
    }

    QAL_VERBOSE(LOG_TAG, "%s: Update config success", __func__);
    return status;
}

void SoundTriggerEngineGsl::setDetected(bool detected)
{
    QAL_INFO(LOG_TAG, "setDetected %d", detected);
    std::lock_guard<std::mutex> lck(sndEngGsl_->mutex_);
    if (detected != eventDetected)
    {
        QAL_INFO(LOG_TAG, "notify condition variable");
        eventDetected = detected;
        QAL_INFO(LOG_TAG, "eventDetected set to %d", detected);
        cv_.notify_one();
    }
    else
        QAL_VERBOSE(LOG_TAG, "eventDetected unchanged");
}

