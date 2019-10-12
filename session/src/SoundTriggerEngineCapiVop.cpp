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

#define LOG_TAG "SoundTriggerEngineCapiVop"

#include "SoundTriggerEngineCapiVop.h"
#include "StreamSoundTrigger.h"
#include <dlfcn.h>
#include "Stream.h"
#include <fstream>

std::shared_ptr<SoundTriggerEngineCapiVop> SoundTriggerEngineCapiVop::sndEngCapiVop_ = NULL;

void SoundTriggerEngineCapiVop::buffer_thread_loop()
{
    std::unique_lock<std::mutex> lck(sndEngCapiVop_->mutex_);

    while (!sndEngCapiVop_->exit_thread_)
    {
        sndEngCapiVop_->exit_buffering_ = false;
        QAL_VERBOSE(LOG_TAG,"waiting on cond, eventDetected = %d", sndEngCapiVop_->eventDetected);
        /* Wait for keyword buffer data from DSP */
        if (!sndEngCapiVop_->eventDetected)
            sndEngCapiVop_->cv_.wait(lck);
        QAL_VERBOSE(LOG_TAG,"done waiting on cond, exit_buffering = %d", sndEngCapiVop_->exit_buffering_);

        if (sndEngCapiVop_->exit_thread_) {
            break;
        }

        /*
        * If 1st stage buffering overflows before 2nd stage starts processing,
        * the below functions need to be called to reset the 1st stage session
        * for the next detection.
        */
        /* we might be able to check states of the engine to avoid this buffering flag */
        if (sndEngCapiVop_->exit_buffering_)
        {
            continue; /* skip over processing if we want to exit already*/
        }

        if (sndEngCapiVop_->eventDetected)
            sndEngCapiVop_->start_keyword_detection();
    }
}


static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineCapiVop::start_keyword_detection()
{
    int32_t result = 0;

    char *process_input_buff = NULL;
    capi_v2_err_t rc;
    capi_v2_stream_data_t *stream_input = NULL;
    capi_v2_buf_t capi_uv_ptr;
    voiceprint2_result_t *result_cfg_ptr = NULL;
    voiceprint2_sva_uv_score_t *uv_cfg_ptr = NULL;
    unsigned int det_status = 0;
    int32_t readFillSize = 0;
    capi_v2_buf_t capi_result;

    mutex.lock();
    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        QAL_ERR(LOG_TAG,"failed to allocate process_input_buff");
        result = -ENOMEM;
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        QAL_ERR(LOG_TAG,"failed to allocate stream_input");
        result = -ENOMEM;
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        QAL_ERR(LOG_TAG,"failed to allocate stream_input->buf_ptr");
        result = -ENOMEM;
        goto exit;
    }

    result_cfg_ptr = (voiceprint2_result_t*)calloc(1, sizeof(voiceprint2_result_t));
    if (!result_cfg_ptr) {
        QAL_ERR(LOG_TAG, "failed to allocate result_cfg_ptr");
        result = -ENOMEM;
        goto exit;
    }

    uv_cfg_ptr = (voiceprint2_sva_uv_score_t *)calloc(1, sizeof(voiceprint2_sva_uv_score_t));
    if (!uv_cfg_ptr) {
        QAL_ERR(LOG_TAG, "failed to allocate uv_cfg_ptr");
        result = -ENOMEM;
        goto exit;
    }

    uv_cfg_ptr->sva_uv_confidence_score = confidence_score_;
    capi_uv_ptr.data_ptr = (int8_t *)uv_cfg_ptr;
    capi_uv_ptr.actual_data_len = sizeof(voiceprint2_sva_uv_score_t);
    capi_uv_ptr.max_data_len = sizeof(voiceprint2_sva_uv_score_t);

    QAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d", VOICEPRINT2_ID_SVA_UV_SCORE);
    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
        VOICEPRINT2_ID_SVA_UV_SCORE, NULL, &capi_uv_ptr);
    if (CAPI_V2_EOK != rc) {
        QAL_ERR(LOG_TAG, "set_param VOICEPRINT2_ID_SVA_UV_SCORE failed, result = %d", rc);
        result = -EINVAL;
        goto exit;
    }

    if (kw_end_timestamp_ > 0)
        buffer_end_ = us_to_bytes(kw_end_timestamp_);

    if (kw_start_timestamp_ > 0)
        buffer_start_ = us_to_bytes(kw_start_timestamp_);

    while (!exit_buffering_ &&
        (bytes_processed_ < buffer_end_ - buffer_start_))
    {
        /* Original code had some time of wait will need to revisit*/
        /* need to take into consideration the start and end buffer*/

        /* advance the offset to ensure we are reading at the right place */
        if (buffer_start_ > 0)
            reader_->advanceReadOffset(buffer_start_);

        if (reader_->getUnreadSize() < buffer_size_)
            continue;

        readFillSize = reader_->read((void*)process_input_buff, buffer_size_);
        if (readFillSize == 0)
            continue;
        QAL_INFO(LOG_TAG, "Processed : %u, start: %u, end: %u", bytes_processed_, buffer_start_, buffer_end_);
        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = buffer_size_;
        stream_input->buf_ptr->actual_data_len = readFillSize;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        QAL_VERBOSE(LOG_TAG, "Calling Capi Process\n", __func__);

        rc = capi_handle_->vtbl_ptr->process(capi_handle_, &stream_input, NULL);

        if (CAPI_V2_EFAILED == rc)
        {
            QAL_ERR(LOG_TAG, "capi process failed\n");
            result = -EINVAL;
            goto exit;
        }

        bytes_processed_ += readFillSize;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(voiceprint2_result_t);
        capi_result.max_data_len = sizeof(voiceprint2_result_t);

        QAL_VERBOSE(LOG_TAG, "Calling Capi get param for result\n");

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_, VOICEPRINT2_ID_RESULT, NULL,
            &capi_result);

        if (CAPI_V2_EFAILED == rc)
        {
            QAL_ERR(LOG_TAG, "capi get param failed\n");
            result = -EINVAL;
            goto exit;
        }

        if (result_cfg_ptr->is_detected)
        {
            exit_buffering_ = true;

            // TODO: Notify StreamSoundTrigger
            StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>(streamHandle);
            s->setDetectionState(VOP_DETECTED);
            reader_->updateState(READER_DISABLED);
            eventDetected = false;
            //we detected the keyword using second stage
            //can break out of the loop now
            //report back to stream on other key
            //pieces of info
            QAL_INFO(LOG_TAG, "KW Second Stage Detected")
        }
        exit_buffering_ = true;
    }

exit:
    if (process_input_buff)
        free(process_input_buff);
    if (stream_input)
    {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);
    // add code to handle unwind
    mutex.unlock();
    return result;
}


int32_t SoundTriggerEngineCapiVop::prepare_sound_engine()
{
    int32_t result = 0;

    //not sure if we need prepare as we are starting the thread @ start sound engine

    return result;
}

SoundTriggerEngineCapiVop::SoundTriggerEngineCapiVop(Stream *s, uint32_t id, uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer)
{
    int32_t result = 0;
    const char *lib = "libcapiv2vop.so";
    sndEngCapiVop_ = (std::shared_ptr<SoundTriggerEngineCapiVop>)this;

    engineId = id;
    stageId = stage_id;
    eventDetected = false;
    sm_data = NULL;
    sm_params_data = NULL;
    exit_thread_ = false;
    exit_buffering_ = false;

    buffer_size_ = CNN_BUFFER_SIZE; //480ms of 16k 16bit mono worth;

    kw_start_timestamp_ = 0;
    kw_end_timestamp_ = CNN_DURATION_US;
    buffer_start_ = 0;
    buffer_end_ = 0;
    bytes_processed_ = 0;

    capi_handle_ = (capi_v2_t *)calloc(1, sizeof(capi_v2_t) + (3 * sizeof(char *)));

    if (!capi_handle_)
    {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate capi handle = %d", result);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib, RTLD_NOW);
    if (!capi_lib_handle_)
    {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to open capi so = %d", result);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init)
    {
        QAL_ERR(LOG_TAG, "failed to map capi init function = %d", result);
        /* handle here */
        goto err_exit;
    }

    streamHandle = s;
    if (!buffer)
    {
        buffer_ = new QalRingBuffer(DEFAULT_QAL_RING_BUFFER_SIZE);
        reader_ = NULL;
        *reader = buffer_->newReader();
    }
    else
    {
        buffer_ = NULL;
        reader_ = buffer->newReader();
    }
    return;
err_exit:
    QAL_ERR(LOG_TAG, "constructor exit result = %d", result);
}

SoundTriggerEngineCapiVop::~SoundTriggerEngineCapiVop()
{
    if (sm_data)
        free(sm_data);

    if (sm_params_data)
        free(sm_params_data);

    if (capi_lib_handle_)
    {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = NULL;
    }

    if (capi_handle_)
    {
        capi_handle_->vtbl_ptr = NULL;
        free(capi_handle_);
        capi_handle_ = NULL;
    }
}

int32_t SoundTriggerEngineCapiVop::start_sound_engine()
{
    int32_t result = 0;

    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;
    voiceprint2_threshold_config_t *threshold_cfg = NULL;

    bufferThreadHandler_ = std::thread(SoundTriggerEngineCapiVop::buffer_thread_loop);

    if (!bufferThreadHandler_.joinable())
    {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "failed to create buffer thread = %d", result);
        goto exit;
    }

    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                VOICEPRINT2_ID_REINIT, NULL, NULL);
    if (CAPI_V2_EOK != rc)
    {
        result = -EINVAL;
        QAL_ERR(LOG_TAG,"set_param VOICEPRINT2_ID_REINIT failed, result = %d", result);
        goto exit;
    }

    threshold_cfg = (voiceprint2_threshold_config_t *)calloc(1, sizeof(voiceprint2_threshold_config_t));
    if (!threshold_cfg) {
        QAL_ERR(LOG_TAG, "failed to allocate threshold_cfg");
        result = -ENOMEM;
        goto exit;
    }

    capi_buf.data_ptr = (int8_t *)threshold_cfg;
    capi_buf.actual_data_len = sizeof(voiceprint2_threshold_config_t);
    capi_buf.max_data_len = sizeof(voiceprint2_threshold_config_t);
    threshold_cfg->user_verification_threshold = confidence_threshold_;
    QAL_VERBOSE(LOG_TAG, "Keyword detection (VOP) confidence level = %d", confidence_threshold_);

    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
             VOICEPRINT2_ID_THRESHOLD_CONFIG, NULL, &capi_buf);

    if (CAPI_V2_EOK != rc)
    {
        result = -EINVAL;
        QAL_ERR(LOG_TAG,"set_param VOICEPRINT2_ID_THRESHOLD_CONFIG failed, result = %d", rc);
        goto exit;
    }

exit:
    if (threshold_cfg)
        free(threshold_cfg);

    return result;
}

int32_t SoundTriggerEngineCapiVop::stop_sound_engine()
{
    int32_t result = 0;
    capi_v2_err_t rc;

    QAL_VERBOSE(LOG_TAG,"%s: Issuing capi_end", __func__);
    result = capi_handle_->vtbl_ptr->end(capi_handle_);
    if (result != CAPI_V2_EOK)
    {
        QAL_ERR(LOG_TAG, "Capi end function failed, result = %d",
            result);

        result = -EINVAL;
    }
    {
        eventDetected = false;
        std::lock_guard<std::mutex> lck(sndEngCapiVop_->mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;

        cv_.notify_one();
    }
    bufferThreadHandler_.join();

    return result;
}

int32_t SoundTriggerEngineCapiVop::load_sound_model(Stream *s, uint8_t *data, uint32_t num_models)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;
    uint8_t *sm_payload = NULL;
    SML_BigSoundModelTypeV3 *big_sm;
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_err_t rc;

    mutex.lock();
    if (!data || num_models <= 1)
    {
        QAL_ERR(LOG_TAG, "Invalid sound model data");
        status = -EINVAL;
        goto exit;
    }

    common_sm = (struct qal_st_sound_model *)data;
    QAL_INFO(LOG_TAG, "Sound model type: %u", common_sm->type);
    if (common_sm->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE)
    {
        sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
        for (int i = 0; i < num_models; i++)
        {
            big_sm = (SML_BigSoundModelTypeV3 *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                sizeof(SML_HeaderTypeV3) + (i * sizeof(SML_BigSoundModelTypeV3)));
            QAL_INFO(LOG_TAG, "type = %u, size = %u", big_sm->type, big_sm->size);
            if (big_sm->type == ST_SM_ID_SVA_VOP)
            {
                sm_data_size = big_sm->size;
                uint8_t *ptr = (uint8_t *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) + (num_models * sizeof(SML_BigSoundModelTypeV3)) +
                    big_sm->offset);
                sm_data = (uint8_t *)calloc(1, sm_data_size);
                memcpy(sm_data, ptr, sm_data_size);
            }
        }
    }

    sm_prop_ptr.id = CAPI_V2_CUSTOM_INIT_DATA;
    sm_prop_ptr.payload.data_ptr = (int8_t *)sm_data;
    sm_prop_ptr.payload.actual_data_len = sm_data_size;
    sm_prop_ptr.payload.max_data_len = sm_data_size;
    init_set_proplist.props_num = 1;
    init_set_proplist.prop_ptr = &sm_prop_ptr;

    QAL_VERBOSE(LOG_TAG, "Issuing capi_init");
    rc = capi_init(capi_handle_, &init_set_proplist);

    if (rc != CAPI_V2_EOK) {
        QAL_ERR(LOG_TAG, "capi_init result is %d, exiting", rc);
        status = -EINVAL;
        goto exit;
    }

    if (NULL == capi_handle_) {
        QAL_ERR(LOG_TAG, "capi_handle is NULL, exiting");
        status = -EINVAL;
        goto exit;
    }

    if (NULL == capi_handle_->vtbl_ptr) {
        QAL_ERR(LOG_TAG, "capi_handle->vtbl_ptr is NULL, exiting");
        status = -EINVAL;
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "Load sound model success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to load sound model, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineCapiVop::unload_sound_model(Stream *s)
{
    int32_t status = 0;

exit:
    QAL_ERR(LOG_TAG, "Failed to unload sound model, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineCapiVop::start_recognition(Stream *s)
{
    int32_t status = 0;
    mutex.lock();
    status = start_sound_engine();
    if (status)
    {
        QAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "start recognition success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to start recognition, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineCapiVop::stop_buffering(Stream *s)
{
    int32_t status = 0;

    mutex.lock();
    eventDetected = false;
    exit_buffering_ = true;
    if (reader_)
        reader_->reset();
    else {
        status = -EINVAL;
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "stop buffering success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to stop buffering, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineCapiVop::stop_recognition(Stream *s)
{
    int32_t status = 0;
    mutex.lock();
    status = stop_sound_engine();
    if (status)
    {
        QAL_ERR(LOG_TAG, "Failed to stop sound engine, status = %d", status);
        goto exit;
    }

    if (reader_)
        reader_->reset();
    else {
        status = -EINVAL;
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "stop recognition success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to stop recognition, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineCapiVop::update_config(Stream *s, struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    size_t config_size;
    if (!config)
    {
        QAL_ERR(LOG_TAG, "Invalid config");
        return -EINVAL;
    }

    mutex.lock();
    confidence_threshold_ = config->phrases[0].levels[0].level;
    mutex.unlock();
    QAL_INFO(LOG_TAG, "confidence threshold: %d", confidence_threshold_);
    QAL_VERBOSE(LOG_TAG, "update config success");
    return status;
}

void SoundTriggerEngineCapiVop::setDetected(bool detected)
{
    QAL_INFO(LOG_TAG, "setDetected %d", detected);
    mutex.lock();
    std::lock_guard<std::mutex> lck(sndEngCapiVop_->mutex_);
    if (detected != eventDetected)
    {
        // TODO: update indices/timestamp info also
        // for now we just estimate the values
        eventDetected = detected;
        QAL_INFO(LOG_TAG, "eventDetected set to %d", detected);
        cv_.notify_one();

        struct detection_event_info *info = NULL;
        StreamSoundTrigger *str = dynamic_cast<StreamSoundTrigger *>(streamHandle);
        str->getDetectionEventInfo(&info);
        confidence_score_ = info->confidence_levels[1];
    }
    else
        QAL_VERBOSE(LOG_TAG, "eventDetected unchanged");
    mutex.unlock();
}

int32_t SoundTriggerEngineCapiVop::getParameters(uint32_t param_id, void **payload)
{
    return 0;
}
