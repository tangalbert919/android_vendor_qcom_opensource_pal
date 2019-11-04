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

#include "StreamSoundTrigger.h"
#include "Stream.h"

void SoundTriggerEngineCapiCnn::buffer_thread_loop(
    SoundTriggerEngineCapiCnn *cnn_engine)
{
    StreamSoundTrigger *s = nullptr;
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    if (!cnn_engine) {
        QAL_ERR(LOG_TAG, "Invalid sound trigger engine");
        return;
    }

    std::unique_lock<std::mutex> lck(cnn_engine->event_mutex_);
    while (!cnn_engine->exit_thread_) {
        QAL_VERBOSE(LOG_TAG, "waiting on cond, processing started  = %d",
                    cnn_engine->processing_started_);
        // Wait for keyword buffer data from DSP
        if (!cnn_engine->processing_started_)
            cnn_engine->cv_.wait(lck);
        QAL_VERBOSE(LOG_TAG, "done waiting on cond, exit buffering = %d",
                    cnn_engine->exit_buffering_);

        if (cnn_engine->exit_thread_) {
            break;
        }

        /*
         * If 1st stage buffering overflows before 2nd stage starts processing,
         * the below functions need to be called to reset the 1st stage session
         * for the next detection. We might be able to check states of the engine
         * to avoid this buffering flag.
         */
        if (cnn_engine->exit_buffering_) {
            continue;  // skip over processing if we want to exit already
        }

        if (cnn_engine->processing_started_) {
            s = dynamic_cast<StreamSoundTrigger *>(cnn_engine->stream_handle_);

            status = cnn_engine->StartDetection();
            if (status || !cnn_engine->keyword_detected_)
                s->setDetectionState(CNN_REJECTED);
            else
                s->setDetectionState(CNN_DETECTED);

            cnn_engine->keyword_detected_ = false;
            cnn_engine->processing_started_ = false;
        }
    }
    QAL_DBG(LOG_TAG, "Exit");
}


static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineCapiCnn::StartDetection()
{
    int32_t status = 0;
    char *process_input_buff = nullptr;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_stream_data_t *stream_input = nullptr;
    sva_result_t *result_cfg_ptr = nullptr;
    unsigned int det_status = 0;
    int32_t read_size = 0;
    capi_v2_buf_t capi_result;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (!reader_) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid ring buffer reader");
        goto exit;
    }

    memset(&capi_result, 0, sizeof(capi_result));
    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate process input buff, status %d",
                status);
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)
                   calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate stream input, status %d", status);
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate stream_input->buf_ptr, status %d",
                status);
        goto exit;
    }

    result_cfg_ptr = (sva_result_t*)calloc(1, sizeof(sva_result_t));
    if (!result_cfg_ptr) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate result cfg ptr status %d", status);
        goto exit;
    }

    if (kw_end_timestamp_ > 0)
        buffer_end_ = us_to_bytes(kw_end_timestamp_);

    if (kw_start_timestamp_ > 0)
        buffer_start_ = us_to_bytes(kw_start_timestamp_);

    bytes_processed_ = 0;

    while (!exit_buffering_ &&
        (bytes_processed_ < buffer_end_ - buffer_start_)) {
        /* Original code had some time of wait will need to revisit*/
        /* need to take into consideration the start and end buffer*/

        /* advance the offset to ensure we are reading at the right place */
        if (buffer_start_ > 0)
            reader_->advanceReadOffset(buffer_start_);

        if (reader_->getUnreadSize() < buffer_size_)
            continue;

        read_size = reader_->read((void*)process_input_buff, buffer_size_);
        if (read_size == 0)
            continue;

        QAL_INFO(LOG_TAG, "Processed: %u, start: %u, end: %u",
                 bytes_processed_, buffer_start_, buffer_end_);
        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = buffer_size_;
        stream_input->buf_ptr->actual_data_len = read_size;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        QAL_VERBOSE(LOG_TAG, "Calling Capi Process");

        rc = capi_handle_->vtbl_ptr->process(capi_handle_, &stream_input,
                                             nullptr);

        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "capi process failed, status %d", status);
            goto exit;
        }

        bytes_processed_ += read_size;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(sva_result_t);
        capi_result.max_data_len = sizeof(sva_result_t);

        QAL_VERBOSE(LOG_TAG, "Calling Capi get param for status");

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_, SVA_ID_RESULT,
                                               nullptr, &capi_result);

        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "capi get param failed, status %d", status);
            goto exit;
        }

        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;
            keyword_detected_ = true;
            QAL_INFO(LOG_TAG, "KW Second Stage Detected")
        }
    }

exit:
    if (reader_)
        reader_->updateState(READER_DISABLED);

    if (process_input_buff)
        free(process_input_buff);
    if (stream_input) {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

SoundTriggerEngineCapiCnn::SoundTriggerEngineCapiCnn(
    Stream *s,
    uint32_t id,
    uint32_t stage_id,
    QalRingBufferReader **reader,
    std::shared_ptr<QalRingBuffer> buffer)
{
    int32_t status = 0;
    const char *lib = "libcapiv2svacnn.so";
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_err_t rc = CAPI_V2_EOK;

    QAL_DBG(LOG_TAG, "Enter");
    engine_id_ = id;
    stage_id_ = stage_id;
    processing_started_ = false;
    keyword_detected_ = false;
    sm_data_ = nullptr;
    exit_thread_ = false;
    exit_buffering_ = false;

    buffer_size_ = CNN_BUFFER_SIZE;  // 480ms of 16k 16bit mono worth;

    kw_start_timestamp_ = 0;
    kw_end_timestamp_ = CNN_DURATION_US;
    buffer_start_ = 0;
    buffer_end_ = 0;
    bytes_processed_ = 0;

    capi_handle_ = (capi_v2_t *)calloc(1, sizeof(capi_v2_t)+sizeof(char *));

    if (!capi_handle_) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate capi handle = %d", status);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib, RTLD_NOW);
    if (!capi_lib_handle_) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG,  "failed to open capi so = %d", status);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init_ = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init_) {
        QAL_ERR(LOG_TAG,  "failed to map capi init function = %d", status);
        /* handle here */
        goto err_exit;
    }

    if (!capi_handle_) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "capi handle is nullptr, exiting status %d", status);
        goto err_exit;
    }
    stream_handle_ = s;
    if (!buffer) {
        buffer_ = new QalRingBuffer(DEFAULT_QAL_RING_BUFFER_SIZE);
        reader_ = nullptr;
        *reader = buffer_->newReader();
    } else {
        buffer_ = nullptr;
        reader_ = buffer->newReader();
    }
    return;
err_exit:
    QAL_ERR(LOG_TAG, "constructor exit status = %d", status);
}

SoundTriggerEngineCapiCnn::~SoundTriggerEngineCapiCnn()
{
    QAL_DBG(LOG_TAG, "Enter");

    if (capi_lib_handle_) {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = nullptr;
    }

    if (capi_handle_) {
        capi_handle_->vtbl_ptr = nullptr;
        free(capi_handle_);
        capi_handle_ = nullptr;
    }
    QAL_DBG(LOG_TAG, "Exit");
}

int32_t SoundTriggerEngineCapiCnn::StartSoundEngine()
{
    int32_t status = 0;
    processing_started_ = false;
    exit_thread_ = false;
    exit_buffering_ = false;

    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;
    sva_threshold_config_t *threshold_cfg = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    buffer_thread_handler_ =
        std::thread(SoundTriggerEngineCapiCnn::buffer_thread_loop, this);

    if (!buffer_thread_handler_.joinable()) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "failed to create buffer thread = %d", status);
        goto exit;
    }

    threshold_cfg = (sva_threshold_config_t*)
                    calloc(1, sizeof(sva_threshold_config_t));
    if (!threshold_cfg) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "threshold cfg calloc failed, status %d", status);
        goto exit;
    }
    capi_buf.data_ptr = (int8_t*) threshold_cfg;
    capi_buf.actual_data_len = sizeof(sva_threshold_config_t);
    capi_buf.max_data_len = sizeof(sva_threshold_config_t);
    threshold_cfg->smm_threshold = confidence_threshold_;
    QAL_VERBOSE(LOG_TAG, "Keyword detection (CNN) confidence level = %d",
        confidence_threshold_);

    status = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                SVA_ID_THRESHOLD_CONFIG, nullptr, &capi_buf);

    if (CAPI_V2_EOK != status) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "set param SVA_ID_THRESHOLD_CONFIG failed with %d",
                status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d",
                SVA_ID_REINIT_ALL);
    status = capi_handle_->vtbl_ptr->set_param(capi_handle_, SVA_ID_REINIT_ALL,
                                               nullptr, nullptr);

    if (CAPI_V2_EOK != status) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "set param SVA_ID_REINIT_ALL failed, status = %d",
                status);
        goto exit;
    }

exit:
    if (threshold_cfg)
        free(threshold_cfg);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::StopSoundEngine()
{
    int32_t status = 0;
    capi_v2_err_t rc = CAPI_V2_EOK;

    QAL_DBG(LOG_TAG, "Enter, Issuing capi_end");
    status = capi_handle_->vtbl_ptr->end(capi_handle_);
    if (status != CAPI_V2_EOK) {
        QAL_ERR(LOG_TAG, "Capi end function failed, status = %d",
            status);
        status = -EINVAL;
    }
    {
        processing_started_ = false;
        std::lock_guard<std::mutex> lck(event_mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;

        cv_.notify_one();
    }
    buffer_thread_handler_.join();
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::LoadSoundModel(Stream *s, uint8_t *data,
                                                  uint32_t data_size)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;
    struct qal_st_sound_model *common_sm = nullptr;
    uint8_t *sm_payload = nullptr;
    SML_BigSoundModelTypeV3 *big_sm = nullptr;
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_err_t rc = CAPI_V2_EOK;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (!data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound model data, status %d", status);
        goto exit;
    }

    sm_data_ = data;
    sm_data_size_ = data_size;

    sm_prop_ptr.id = CAPI_V2_CUSTOM_INIT_DATA;
    sm_prop_ptr.payload.data_ptr = (int8_t *)sm_data_;
    sm_prop_ptr.payload.actual_data_len = sm_data_size_;
    sm_prop_ptr.payload.max_data_len = sm_data_size_;
    init_set_proplist.props_num = 1;
    init_set_proplist.prop_ptr = &sm_prop_ptr;

    QAL_VERBOSE(LOG_TAG, "Issuing capi_init");
    rc = capi_init_(capi_handle_, &init_set_proplist);

    if (rc != CAPI_V2_EOK) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "capi_init status is %d, exiting, status %d",
                rc, status);
        goto exit;
    }

    if (!capi_handle_->vtbl_ptr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "capi_handle->vtbl_ptr is nullptr, exiting, status %d",
                status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::UnloadSoundModel(Stream *s)
{
    return 0;
}

int32_t SoundTriggerEngineCapiCnn::StartRecognition(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    status = StartSoundEngine();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::StopBuffering(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    processing_started_ = false;
    exit_buffering_ = true;
    if (reader_) {
        reader_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::StopRecognition(Stream *s)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    status = StopSoundEngine();
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to stop sound engine, status = %d", status);
        goto exit;
    }

    if (reader_) {
        reader_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiCnn::UpdateConfLevels(
    Stream *s,
    struct qal_st_recognition_config *config,
    uint8_t *conf_levels,
    uint32_t num_conf_levels)
{
    int32_t status = 0;
    size_t config_size;

    QAL_DBG(LOG_TAG, "Enter");
    if (!conf_levels || !num_conf_levels) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid config, status %d", status);
        return status;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    confidence_threshold_ = *conf_levels;
    QAL_VERBOSE(LOG_TAG, "confidence threshold: %d", confidence_threshold_);

    return status;
}

void SoundTriggerEngineCapiCnn::SetDetected(bool detected)
{
    QAL_DBG(LOG_TAG, "SetDetected %d", detected);
    std::lock_guard<std::mutex> lck(event_mutex_);
    if (detected != processing_started_) {
        processing_started_ = detected;
        exit_buffering_ = !processing_started_;
        QAL_INFO(LOG_TAG, "setting processing started %d", detected);
        cv_.notify_one();
    } else {
        QAL_VERBOSE(LOG_TAG, "processing started unchanged");
    }
}
