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

#define LOG_TAG "SoundTriggerEngineCapiCnn"

#include "SoundTriggerEngineCapiCnn.h"

#include <dlfcn.h>
#include "Stream.h"

std::shared_ptr<SoundTriggerEngineCapiCnn> SoundTriggerEngineCapiCnn::sndEngCapiCnn_ = NULL;

void SoundTriggerEngineCapiCnn::buffer_thread_loop()
{
    std::unique_lock<std::mutex> lck(sndEngCapiCnn_->mutex_);

    while (!sndEngCapiCnn_->exit_thread_)
    {
        sndEngCapiCnn_->exit_buffering_ = false;
        QAL_VERBOSE(LOG_TAG,"%s: waiting on cond", __func__);
        /* Wait for keyword buffer data from DSP */
        sndEngCapiCnn_->cv_.wait(lck);
        QAL_VERBOSE(LOG_TAG,"%s: done waiting on cond, exit_buffering = %d", __func__,
            sndEngCapiCnn_->exit_buffering_);

        if (sndEngCapiCnn_->exit_thread_) {
            break;
        }

        /*
        * If 1st stage buffering overflows before 2nd stage starts processing,
        * the below functions need to be called to reset the 1st stage session
        * for the next detection.
        */
        /* we might be able to check states of the engine to avoid this buffering flag */
        if (sndEngCapiCnn_->exit_buffering_)
        {
            continue; /* skip over processing if we want to exit already*/
        }

        sndEngCapiCnn_->start_keyword_detection();
    }
}


static uint32_t us_to_bytes(uint64_t input_us)
{
    uint32_t sample_rate = 16000;
    uint32_t bitwidth = 2;
    uint32_t num_of_channels = 1;

    return (sample_rate * bitwidth * num_of_channels * input_us/1000000);
}
int32_t SoundTriggerEngineCapiCnn::start_keyword_detection()
{
    int32_t result = 0;

    char *process_input_buff = NULL;
    capi_v2_err_t rc;
    capi_v2_stream_data_t *stream_input = NULL;
    sva_result_t *result_cfg_ptr = NULL;
    unsigned int det_status = 0;
    int32_t readFillSize = 0;
    capi_v2_buf_t capi_result;

    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        QAL_ERR(LOG_TAG,"%s: failed to allocate process_input_buff", __func__);
        result = -ENOMEM;
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        QAL_ERR(LOG_TAG,"%s: failed to allocate stream_input", __func__);
        result = -ENOMEM;
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        QAL_ERR(LOG_TAG,"%s: failed to allocate stream_input->buf_ptr", __func__);
        result = -ENOMEM;
        goto exit;
    }

    result_cfg_ptr = (sva_result_t*)calloc(1, sizeof(sva_result_t));
    if (!result_cfg_ptr) {
        QAL_ERR(LOG_TAG,"%s: failed to allocate result_cfg_ptr", __func__);
        result = -ENOMEM;
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
            ringBufferReader->advanceReadOffset(buffer_start_);

        readFillSize = ringBufferReader->read((void*)process_input_buff, buffer_size_);

        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = buffer_size_;
        stream_input->buf_ptr->actual_data_len = readFillSize;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        QAL_VERBOSE(LOG_TAG, "%s :Calling Capi Process\n", __func__);

        rc = capi_handle_->vtbl_ptr->process(capi_handle_, &stream_input, NULL);

        if (CAPI_V2_EFAILED == rc)
        {
            QAL_ERR(LOG_TAG, "%s: capi process failed\n");
            result = -EINVAL;
            goto exit;
        }

        bytes_processed_ += readFillSize;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(sva_result_t);
        capi_result.max_data_len = sizeof(sva_result_t);

        QAL_VERBOSE(LOG_TAG, "%s :Calling Capi get param for result\n", __func__);

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_, SVA_ID_RESULT, NULL,
            &capi_result);

        if (CAPI_V2_EFAILED == rc)
        {
            QAL_ERR(LOG_TAG, "%s: capi get param failed\n");
            result = -EINVAL;
            goto exit;
        }

        if (result_cfg_ptr->is_detected)
        {
            exit_buffering_ = true;

            //we detected the keyword using second stage
            //can break out of the loop now
            //report back to stream on other key
            //pieces of info
            QAL_ERR(LOG_TAG, "KW Second Stage Detected")
        }
        bytes_processed_ += readFillSize;
    }

exit:
    // add code to handle unwind
    return result;
}


int32_t SoundTriggerEngineCapiCnn::prepare_sound_engine()
{
    int32_t result = 0;

    //not sure if we need prepare as we are starting the thread @ start sound engine

    return result;
}

SoundTriggerEngineCapiCnn::SoundTriggerEngineCapiCnn(Stream *s, uint32_t id, uint32_t stage_id)
{
    int32_t result = 0;
    const char *lib = "libcapiv2svacnn.so";
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_err_t rc;
    sndEngCapiCnn_ = (std::shared_ptr<SoundTriggerEngineCapiCnn>)this;

    engineId = id;
    stageId = stage_id;

    exit_thread_ = false;
    exit_buffering_ = false;

    buffer_size_ = 15360; //480ms of 16k 16bit mono worth;

    capi_handle_ = (capi_v2_t *)calloc(1, sizeof(capi_v2_t)+sizeof(char *));

    if (!capi_handle_)
    {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "%s: failed to allocate capi handle = %d", __func__, result);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib, RTLD_NOW);
    if (!capi_handle_)
    {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "%s: failed to open capi so = %d", __func__, result);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init)
    {
        QAL_ERR(LOG_TAG, "%s: failed to map capi init function = %d", __func__, result);
        /* handle here */
        goto err_exit;
    }

    sm_prop_ptr.id = CAPI_V2_CUSTOM_INIT_DATA;
    sm_prop_ptr.payload.data_ptr = (int8_t *)(pSoundModel + pSoundModel->data_offset);
    sm_prop_ptr.payload.actual_data_len = (uint32_t)(pSoundModel->data_size);
    sm_prop_ptr.payload.max_data_len = (uint32_t)(pSoundModel->data_size);
    init_set_proplist.props_num = 1;
    init_set_proplist.prop_ptr = &sm_prop_ptr;

    QAL_VERBOSE(LOG_TAG, "%s: Issuing capi_init", __func__);
    rc = capi_init(capi_handle_, &init_set_proplist);

    if (rc != CAPI_V2_EOK) {
        QAL_ERR(LOG_TAG, "%s: capi_init result is %d, exiting", __func__, rc);
        result = -EINVAL;
        goto err_exit;
    }
    if (NULL == capi_handle_) {
        QAL_ERR(LOG_TAG, "%s: capi_handle is NULL, exiting", __func__);
        result = -EINVAL;
        goto err_exit;
    }
    if (NULL == capi_handle_->vtbl_ptr) {
        QAL_ERR(LOG_TAG, "%s: capi_handle->vtbl_ptr is NULL, exiting", __func__);
        result = -EINVAL;
        goto err_exit;
    }
    streamHandle = s;

err_exit:
    QAL_ERR(LOG_TAG, "%s: constructor exit result = %d", __func__, result);
}

SoundTriggerEngineCapiCnn::~SoundTriggerEngineCapiCnn()
{
    if (sm_data)
        free(sm_data);

    if (sm_params_data)
        free(sm_params_data);

    if (pSoundModel)
        free(pSoundModel);

    if (pWakeUpConfig)
        free(pWakeUpConfig);

    if (pEventConfig)
        free(pEventConfig);

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

int32_t SoundTriggerEngineCapiCnn::start_sound_engine()
{
    int32_t result = 0;

    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;
    sva_threshold_config_t *threshold_cfg;

    bufferThreadHandler_ = std::thread(SoundTriggerEngineCapiCnn::buffer_thread_loop);

    if (bufferThreadHandler_.joinable())
    {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: failed to create buffer thread = %d", __func__, result);
    }

    threshold_cfg = (sva_threshold_config_t*) calloc(1, sizeof(sva_threshold_config_t));

    capi_buf.data_ptr = (int8_t*) threshold_cfg;
    capi_buf.actual_data_len = sizeof(sva_threshold_config_t);
    capi_buf.max_data_len = sizeof(sva_threshold_config_t);
    threshold_cfg->smm_threshold = confidence_threshold_;
    QAL_VERBOSE("%s: Keyword detection (CNN) confidence level = %d", __func__,
        confidence_threshold_);

    result = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                SVA_ID_THRESHOLD_CONFIG, NULL, &capi_buf);

    if (CAPI_V2_EOK != result)
    {
        result = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: set_param SVA_ID_THRESHOLD_CONFIG failed, result = %d",
            __func__, rc);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG,"%s: Issuing capi_set_param for param %d", __func__, SVA_ID_REINIT_ALL);
    result = capi_handle_->vtbl_ptr->set_param(capi_handle_,SVA_ID_REINIT_ALL, NULL, NULL);

    if (CAPI_V2_EOK != result) {
        result = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: set_param SVA_ID_REINIT_ALL failed, result = %d", __func__, rc);
        goto exit;
    }

exit:
    if (threshold_cfg)
        free(threshold_cfg);

    return result;
}

int32_t SoundTriggerEngineCapiCnn::stop_sound_engine()
{
    int32_t result = 0;
    capi_v2_err_t rc;

    QAL_VERBOSE(LOG_TAG,"%s: Issuing capi_end", __func__);
    result = capi_handle_->vtbl_ptr->end(capi_handle_);
    if (result != CAPI_V2_EOK)
    {
        QAL_ERR(LOG_TAG, "%s: Capi end function failed, result = %d",
            __func__, result);

        result = -EINVAL;
    }
    {
        std::lock_guard<std::mutex> lck(sndEngCapiCnn_->mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;

        cv_.notify_one();
    }
    bufferThreadHandler_.join();

    return result;
}

int32_t SoundTriggerEngineCapiCnn::load_sound_model(Stream *s, uint8_t *data)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to load sound model, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineCapiCnn::unload_sound_model(Stream *s)
{
    int32_t status = 0;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to unload sound model, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineCapiCnn::start_recognition(Stream *s)
{
    int32_t status = 0;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to start recognition, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineCapiCnn::stop_recognition(Stream *s)
{
    int32_t status = 0;
exit:
    QAL_ERR(LOG_TAG, "%s: Failed to stop recognition, status = %d", __func__, status);
    return status;
}

int32_t SoundTriggerEngineCapiCnn::update_config(Stream *s, struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    size_t config_size;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to update config, status = %d", __func__, status);
    return status;
}

