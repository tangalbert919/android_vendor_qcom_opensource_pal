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
#include "StreamSoundTrigger.h"
#include <dlfcn.h>
#include "Stream.h"

std::shared_ptr<SoundTriggerEngineCapiCnn> SoundTriggerEngineCapiCnn::sndEngCapiCnn_ = NULL;

void SoundTriggerEngineCapiCnn::buffer_thread_loop()
{
    QAL_DBG(LOG_TAG, "Enter.");
    std::unique_lock<std::mutex> lck(sndEngCapiCnn_->mutex_);

    while (!sndEngCapiCnn_->exit_thread_) {
        //sndEngCapiCnn_->exit_buffering_ = false;
        QAL_VERBOSE(LOG_TAG, "waiting on cond, eventDetected = %d",
                    sndEngCapiCnn_->eventDetected);
        /* Wait for keyword buffer data from DSP */
        if (!sndEngCapiCnn_->eventDetected)
            sndEngCapiCnn_->cv_.wait(lck);
        QAL_VERBOSE(LOG_TAG, "done waiting on cond, exit_buffering = %d",
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
        if (sndEngCapiCnn_->exit_buffering_) {
            continue; /* skip over processing if we want to exit already*/
        }

        if (sndEngCapiCnn_->eventDetected)
            sndEngCapiCnn_->start_keyword_detection();
    }
    QAL_DBG(LOG_TAG, "Exit.");
}


static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
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

    QAL_DBG(LOG_TAG, "Enter.");
    mutex.lock();
    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate process_input_buff, result %d", result);
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate stream_input, result %d", result);
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate stream_input->buf_ptr, result %d", result);
        goto exit;
    }

    result_cfg_ptr = (sva_result_t*)calloc(1, sizeof(sva_result_t));
    if (!result_cfg_ptr) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate result_cfg_ptr result %d", result);
        goto exit;
    }

    if (kw_end_timestamp_ > 0)
        buffer_end_ = us_to_bytes(kw_end_timestamp_);

    if (kw_start_timestamp_ > 0)
        buffer_start_ = us_to_bytes(kw_start_timestamp_);

    while (!exit_buffering_ &&
        (bytes_processed_ < buffer_end_ - buffer_start_)) {
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

        QAL_INFO(LOG_TAG, "Processed : %u, start: %u, end: %u", bytes_processed_,
                 buffer_start_, buffer_end_);
        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = buffer_size_;
        stream_input->buf_ptr->actual_data_len = readFillSize;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        QAL_VERBOSE(LOG_TAG, "Calling Capi Process");

        rc = capi_handle_->vtbl_ptr->process(capi_handle_, &stream_input, NULL);

        if (CAPI_V2_EFAILED == rc) {
            result = -EINVAL;
            QAL_ERR(LOG_TAG, "capi process failed, result %d", result);
            goto exit;
        }

        bytes_processed_ += readFillSize;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(sva_result_t);
        capi_result.max_data_len = sizeof(sva_result_t);

        QAL_VERBOSE(LOG_TAG, "Calling Capi get param for result");

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_, SVA_ID_RESULT, NULL,
            &capi_result);

        if (CAPI_V2_EFAILED == rc) {
            result = -EINVAL;
            QAL_ERR(LOG_TAG, "capi get param failed, result %d", result);
            goto exit;
        }

        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;

            // TODO: Notify StreamSoundTrigger
            StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>(streamHandle);
            s->setDetectionState(CNN_DETECTED);
            reader_->updateState(READER_DISABLED);
            eventDetected = false;
            //we detected the keyword using second stage
            //can break out of the loop now
            //report back to stream on other key
            //pieces of info
            QAL_INFO(LOG_TAG, "KW Second Stage Detected")
        }
    }
    QAL_DBG(LOG_TAG, "Exit. result %d", result);
    goto exit;

exit:
    if (process_input_buff)
        free(process_input_buff);
    if (stream_input) {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);
    //TODO: add code to handle unwind
    mutex.unlock();
    return result;
}


int32_t SoundTriggerEngineCapiCnn::prepare_sound_engine()
{
    int32_t result = 0;

    //not sure if we need prepare as we are starting the thread @ start sound engine

    return result;
}

SoundTriggerEngineCapiCnn::SoundTriggerEngineCapiCnn(Stream *s, uint32_t id,
    uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer)
{
    int32_t result = 0;
    const char *lib = "libcapiv2svacnn.so";
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_err_t rc;
    QAL_DBG (LOG_TAG, "Enter.");
    sndEngCapiCnn_ = (std::shared_ptr<SoundTriggerEngineCapiCnn>)this;

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

    capi_handle_ = (capi_v2_t *)calloc(1, sizeof(capi_v2_t)+sizeof(char *));

    if (!capi_handle_) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate capi handle = %d", result);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib, RTLD_NOW);
    if (!capi_lib_handle_) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG,  "failed to open capi so = %d", result);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init) {
        QAL_ERR(LOG_TAG,  "failed to map capi init function = %d", result);
        /* handle here */
        goto err_exit;
    }

    if (!capi_handle_) {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "capi_handle is NULL, exiting result %d", result);
        goto err_exit;
    }
    streamHandle = s;
    if (!buffer) {
        buffer_ = new QalRingBuffer(DEFAULT_QAL_RING_BUFFER_SIZE);
        reader_ = NULL;
        *reader = buffer_->newReader();
    } else {
        buffer_ = NULL;
        reader_ = buffer->newReader();
    }
    return;
err_exit:
    QAL_ERR(LOG_TAG, "constructor exit result = %d", result);
}

SoundTriggerEngineCapiCnn::~SoundTriggerEngineCapiCnn()
{
    QAL_DBG(LOG_TAG,"Enter.");
    if (sm_data)
        free(sm_data);

    if (sm_params_data)
        free(sm_params_data);

    if (capi_lib_handle_) {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = NULL;
    }

    if (capi_handle_) {
        capi_handle_->vtbl_ptr = NULL;
        free(capi_handle_);
        capi_handle_ = NULL;
    }
    QAL_DBG(LOG_TAG, "Exit.");
}

int32_t SoundTriggerEngineCapiCnn::start_sound_engine()
{
    int32_t result = 0;
    eventDetected = false;
    exit_thread_ = false;
    exit_buffering_ = false;

    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;
    sva_threshold_config_t *threshold_cfg = NULL;

    QAL_DBG(LOG_TAG, "Enter.");
    bufferThreadHandler_ = std::thread(SoundTriggerEngineCapiCnn::buffer_thread_loop);

    if (!bufferThreadHandler_.joinable()) {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "failed to create buffer thread = %d", result);
        goto exit;
    }

    threshold_cfg = (sva_threshold_config_t*) calloc(1, sizeof(sva_threshold_config_t));
    if (!threshold_cfg) {
        result = -ENOMEM;
        QAL_ERR(LOG_TAG, "threshold_cfg calloc failed, result %d", result);
        goto exit;
    }
    capi_buf.data_ptr = (int8_t*) threshold_cfg;
    capi_buf.actual_data_len = sizeof(sva_threshold_config_t);
    capi_buf.max_data_len = sizeof(sva_threshold_config_t);
    threshold_cfg->smm_threshold = confidence_threshold_;
    QAL_VERBOSE( LOG_TAG, "Keyword detection (CNN) confidence level = %d",
        confidence_threshold_);

    result = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                SVA_ID_THRESHOLD_CONFIG, NULL, &capi_buf);

    if (CAPI_V2_EOK != result) {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "set_param SVA_ID_THRESHOLD_CONFIG failed, result = %d", result);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d", SVA_ID_REINIT_ALL);
    result = capi_handle_->vtbl_ptr->set_param(capi_handle_,SVA_ID_REINIT_ALL, NULL, NULL);

    if (CAPI_V2_EOK != result) {
        result = -EINVAL;
        QAL_ERR(LOG_TAG, "set_param SVA_ID_REINIT_ALL failed, result = %d", result);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. result %d", result);
exit:
    if (threshold_cfg)
        free(threshold_cfg);

    return result;
}

int32_t SoundTriggerEngineCapiCnn::stop_sound_engine()
{
    int32_t result = 0;
    capi_v2_err_t rc;

    QAL_DBG(LOG_TAG, "Enter. Issuing capi_end");
    result = capi_handle_->vtbl_ptr->end(capi_handle_);
    if (result != CAPI_V2_EOK) {
        QAL_ERR(LOG_TAG, "Capi end function failed, result = %d",
            result);
        result = -EINVAL;
    }
    {
        eventDetected = false;
        std::lock_guard<std::mutex> lck(sndEngCapiCnn_->mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;

        cv_.notify_one();
    }
    bufferThreadHandler_.join();
    QAL_DBG(LOG_TAG, "Exit. result %d", result);
    return result;
}

int32_t SoundTriggerEngineCapiCnn::load_sound_model(Stream *s, uint8_t *data,
                                                    uint32_t num_models)
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
    if (!data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound model data, status %d", status);
        goto exit;
    }

    common_sm = (struct qal_st_sound_model *)data;
    QAL_INFO(LOG_TAG, "Sound model type: %u", common_sm->type);
    if (common_sm->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
        for (int i = 0; i < num_models; i++) {
            big_sm = (SML_BigSoundModelTypeV3 *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                sizeof(SML_HeaderTypeV3) + (i * sizeof(SML_BigSoundModelTypeV3)));
            QAL_INFO(LOG_TAG, "type = %u, size = %u", big_sm->type, big_sm->size);
            if (big_sm->type == ST_SM_ID_SVA_CNN) {
                sm_data_size = big_sm->size;
                uint8_t *ptr = (uint8_t *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) + (num_models * sizeof(SML_BigSoundModelTypeV3)) +
                    big_sm->offset);
                sm_data = (uint8_t *)calloc(1, sm_data_size);
                casa_osal_memcpy(sm_data, sm_data_size, ptr, sm_data_size);
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
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "capi_init result is %d, exiting, status %d",  rc, status);
        goto exit;
    }

    if (!(capi_handle_->vtbl_ptr)) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "capi_handle->vtbl_ptr is NULL, exiting, status %d", status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "Exit. Load sound model success");
    mutex.unlock();
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to load sound model, status = %d", status);
    mutex.unlock();
    return status;
}

int32_t SoundTriggerEngineCapiCnn::unload_sound_model(Stream *s)
{
    int32_t status = 0;

exit:
    QAL_ERR(LOG_TAG, "Failed to unload sound model, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineCapiCnn::start_recognition(Stream *s)
{
    int32_t status = 0;
    mutex.lock();
    status = start_sound_engine();
    if (0 != status) {
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

int32_t SoundTriggerEngineCapiCnn::stop_buffering(Stream *s)
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

int32_t SoundTriggerEngineCapiCnn::stop_recognition(Stream *s)
{
    int32_t status = 0;
    mutex.lock();
    status = stop_sound_engine();
    if (status) {
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

int32_t SoundTriggerEngineCapiCnn::update_config(Stream *s,
                                       struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    size_t config_size;
    if (!config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid config, status %d", status);
        return status;
    }

    mutex.lock();
    // TODO: remove hard-coded value
    confidence_threshold_ = 20;
    QAL_VERBOSE(LOG_TAG, "update config success, status %d", status);
    mutex.unlock();
    return status;
}

void SoundTriggerEngineCapiCnn::setDetected(bool detected)
{
    QAL_DBG(LOG_TAG, "setDetected %d", detected);
    mutex.lock();
    std::lock_guard<std::mutex> lck(sndEngCapiCnn_->mutex_);
    if (detected != eventDetected) {
        // TODO: update indices/timestamp info also
        // for now we just estimate the values
        eventDetected = detected;
        QAL_INFO(LOG_TAG, "eventDetected set to %d", detected);
        cv_.notify_one();
    }
    else
        QAL_VERBOSE(LOG_TAG, "eventDetected unchanged");
    mutex.unlock();
}

int32_t SoundTriggerEngineCapiCnn::getParameters(uint32_t param_id, void **payload)
{
    return 0;
}

