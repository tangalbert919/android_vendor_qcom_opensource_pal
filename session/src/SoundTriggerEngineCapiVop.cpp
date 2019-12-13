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

#include <dlfcn.h>

#include <fstream>

#include "StreamSoundTrigger.h"
#include "Stream.h"

void SoundTriggerEngineCapiVop::BufferThreadLoop(
    SoundTriggerEngineCapiVop *vop_engine)
{
    StreamSoundTrigger *s = nullptr;
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    if (!vop_engine) {
        QAL_ERR(LOG_TAG, "Invalid sound trigger engine");
        return;
    }

    std::unique_lock<std::mutex> lck(vop_engine->event_mutex_);
    while (!vop_engine->exit_thread_)
    {
        vop_engine->exit_buffering_ = false;
        QAL_VERBOSE(LOG_TAG, "waiting on cond, processing started  = %d",
                    vop_engine->processing_started_);
        /* Wait for keyword buffer data from DSP */
        if (!vop_engine->processing_started_)
            vop_engine->cv_.wait(lck);
        QAL_VERBOSE(LOG_TAG, "done waiting on cond, exit buffering = %d",
                    vop_engine->exit_buffering_);

        if (vop_engine->exit_thread_) {
            break;
        }

        /*
        * If 1st stage buffering overflows before 2nd stage starts processing,
        * the below functions need to be called to reset the 1st stage session
        * for the next detection. We might be able to check states of the engine
        * to avoid this buffering flag.
        */
        if (vop_engine->exit_buffering_)
            continue; /* skip over processing if we want to exit already*/

        if (vop_engine->processing_started_) {
            s = dynamic_cast<StreamSoundTrigger *>(vop_engine->stream_handle_);

            status = vop_engine->StartDetection();
            if (status || !vop_engine->keyword_detected_)
                s->SetEngineDetectionState(VOP_REJECTED);
            else
                s->SetEngineDetectionState(VOP_DETECTED);

            vop_engine->keyword_detected_ = false;
            vop_engine->processing_started_ = false;
        }
    }
    QAL_DBG(LOG_TAG, "Exit");
}


static uint32_t us_to_bytes(uint64_t input_us)
{
    return (CNN_SAMPLE_RATE * CNN_BITWIDTH * CNN_CHANNELS * input_us /
            (BITS_PER_BYTE * US_PER_SEC));
}

int32_t SoundTriggerEngineCapiVop::StartDetection()
{
    int32_t status = 0;
    char *process_input_buff = nullptr;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_stream_data_t *stream_input = nullptr;
    capi_v2_buf_t capi_uv_ptr;
    voiceprint2_result_t *result_cfg_ptr = nullptr;
    voiceprint2_sva_uv_score_t *uv_cfg_ptr = nullptr;
    int32_t read_size = 0;
    capi_v2_buf_t capi_result;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (!reader_) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid ring buffer reader");
        goto exit;
    }

    memset(&capi_uv_ptr, 0, sizeof(capi_uv_ptr));
    memset(&capi_result, 0, sizeof(capi_result));

    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        QAL_ERR(LOG_TAG, "failed to allocate process input buff");
        status = -ENOMEM;
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)
                   calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        QAL_ERR(LOG_TAG, "failed to allocate stream input");
        status = -ENOMEM;
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        QAL_ERR(LOG_TAG, "failed to allocate buf ptr");
        status = -ENOMEM;
        goto exit;
    }

    result_cfg_ptr = (voiceprint2_result_t*)
                     calloc(1, sizeof(voiceprint2_result_t));
    if (!result_cfg_ptr) {
        QAL_ERR(LOG_TAG, "failed to allocate result cfg ptr");
        status = -ENOMEM;
        goto exit;
    }

    uv_cfg_ptr = (voiceprint2_sva_uv_score_t *)
                 calloc(1, sizeof(voiceprint2_sva_uv_score_t));
    if (!uv_cfg_ptr) {
        QAL_ERR(LOG_TAG, "failed to allocate uv cfg ptr");
        status = -ENOMEM;
        goto exit;
    }

    uv_cfg_ptr->sva_uv_confidence_score = confidence_score_;
    capi_uv_ptr.data_ptr = (int8_t *)uv_cfg_ptr;
    capi_uv_ptr.actual_data_len = sizeof(voiceprint2_sva_uv_score_t);
    capi_uv_ptr.max_data_len = sizeof(voiceprint2_sva_uv_score_t);

    QAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d",
                VOICEPRINT2_ID_SVA_UV_SCORE);
    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
        VOICEPRINT2_ID_SVA_UV_SCORE, nullptr, &capi_uv_ptr);
    if (CAPI_V2_EOK != rc) {
        QAL_ERR(LOG_TAG, "set param VOICEPRINT2_ID_SVA_UV_SCORE failed with %d",
                rc);
        status = -EINVAL;
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

        QAL_VERBOSE(LOG_TAG, "Calling Capi Process\n", __func__);

        rc = capi_handle_->vtbl_ptr->process(capi_handle_, &stream_input,
                                             nullptr);

        if (CAPI_V2_EFAILED == rc) {
            QAL_ERR(LOG_TAG, "capi process failed\n");
            status = -EINVAL;
            goto exit;
        }

        bytes_processed_ += read_size;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(voiceprint2_result_t);
        capi_result.max_data_len = sizeof(voiceprint2_result_t);

        QAL_VERBOSE(LOG_TAG, "Calling Capi get param for result\n");

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
                                               VOICEPRINT2_ID_RESULT,
                                               nullptr, &capi_result);

        if (CAPI_V2_EFAILED == rc) {
            QAL_ERR(LOG_TAG, "capi get param failed\n");
            status = -EINVAL;
            goto exit;
        }

        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;
            keyword_detected_ = true;
            QAL_INFO(LOG_TAG, "KW Second Stage Detected");
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
    if (uv_cfg_ptr)
        free(uv_cfg_ptr);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

SoundTriggerEngineCapiVop::SoundTriggerEngineCapiVop(
    Stream *s,
    uint32_t id,
    uint32_t stage_id,
    QalRingBufferReader **reader,
    QalRingBuffer *buffer)
{
    int32_t status = 0;
    const char *lib = "libcapiv2vop.so";
    engine_id_ = id;
    stage_id_ = stage_id;
    processing_started_ = false;
    keyword_detected_ = false;
    sm_data_ = nullptr;
    exit_thread_ = false;
    exit_buffering_ = false;
    uint32_t bufferSize = DEFAULT_QAL_RING_BUFFER_SIZE;
    struct qal_stream_attributes sAttr;

    buffer_size_ = CNN_BUFFER_SIZE;  // 480ms of 16k 16bit mono worth;
    kw_start_timestamp_ = 0;
    kw_end_timestamp_ = CNN_DURATION_US;
    buffer_start_ = 0;
    buffer_end_ = 0;
    bytes_processed_ = 0;

    capi_handle_ =
        (capi_v2_t *)calloc(1, sizeof(capi_v2_t) + (3 * sizeof(char *)));

    if (!capi_handle_) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate capi handle = %d", status);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib, RTLD_NOW);
    if (!capi_lib_handle_) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to open capi so = %d", status);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init_ = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init_) {
        QAL_ERR(LOG_TAG, "failed to map capi init function = %d", status);
        /* handle here */
        goto err_exit;
    }

    stream_handle_ = s;
    if (!buffer) {
        QAL_INFO(LOG_TAG, "creating new ring buffer");
        s->getStreamAttributes(&sAttr);
        if (sAttr.direction == QAL_AUDIO_INPUT) {
            bufferSize = sAttr.in_media_config.sample_rate *
                sAttr.in_media_config.bit_width *
                sAttr.in_media_config.ch_info->channels *
                RING_BUFFER_DURATION / BITS_PER_BYTE;
        }
        buffer_ = new QalRingBuffer(bufferSize);
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

SoundTriggerEngineCapiVop::~SoundTriggerEngineCapiVop()
{
    QAL_DBG(LOG_TAG, "TODO-STDBG: deleting buffer_ ");
    if (buffer_) {
        delete buffer_;
    }
    QAL_DBG(LOG_TAG, "TODO-STDBG: deleting reader_ ");
    if (reader_) {
        delete reader_;
    }
    if (capi_lib_handle_) {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = nullptr;
    }
    if (capi_handle_) {
        capi_handle_->vtbl_ptr = nullptr;
        free(capi_handle_);
        capi_handle_ = nullptr;
    }
}

int32_t SoundTriggerEngineCapiVop::StartSoundEngine()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;
    voiceprint2_threshold_config_t *threshold_cfg = nullptr;

    buffer_thread_handler_ =
        std::thread(SoundTriggerEngineCapiVop::BufferThreadLoop, this);

    if (!buffer_thread_handler_.joinable()) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "failed to create buffer thread = %d", status);
        goto exit;
    }

    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_, VOICEPRINT2_ID_REINIT,
                                           nullptr, nullptr);
    if (CAPI_V2_EOK != rc) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "set_param VOICEPRINT2_ID_REINIT failed, status = %d",
                status);
        goto exit;
    }

    threshold_cfg = (voiceprint2_threshold_config_t *)
                    calloc(1, sizeof(voiceprint2_threshold_config_t));
    if (!threshold_cfg) {
        QAL_ERR(LOG_TAG, "failed to allocate threshold cfg");
        status = -ENOMEM;
        goto exit;
    }

    capi_buf.data_ptr = (int8_t *)threshold_cfg;
    capi_buf.actual_data_len = sizeof(voiceprint2_threshold_config_t);
    capi_buf.max_data_len = sizeof(voiceprint2_threshold_config_t);
    threshold_cfg->user_verification_threshold = confidence_threshold_;
    QAL_VERBOSE(LOG_TAG, "Keyword detection (VOP) confidence level = %d",
                confidence_threshold_);

    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                                           VOICEPRINT2_ID_THRESHOLD_CONFIG,
                                           nullptr,
                                           &capi_buf);

    if (CAPI_V2_EOK != rc) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "set param %d failed with %d",
                VOICEPRINT2_ID_THRESHOLD_CONFIG, rc);
        goto exit;
    }

exit:
    if (threshold_cfg)
        free(threshold_cfg);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiVop::StopSoundEngine()
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG, "%s: Issuing capi_end", __func__);
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
    if (buffer_thread_handler_.joinable()) {
        buffer_thread_handler_.join();
    }

    return status;
}

int32_t SoundTriggerEngineCapiVop::LoadSoundModel(Stream *s __unused,
    uint8_t *data, uint32_t data_size)
{
    int32_t status = 0;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (!data) {
        QAL_ERR(LOG_TAG, "Invalid sound model data");
        status = -EINVAL;
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
        QAL_ERR(LOG_TAG, "capi_init status is %d, exiting", rc);
        status = -EINVAL;
        goto exit;
    }

    if (nullptr == capi_handle_) {
        QAL_ERR(LOG_TAG, "capi_handle is nullptr, exiting");
        status = -EINVAL;
        goto exit;
    }

    if (nullptr == capi_handle_->vtbl_ptr) {
        QAL_ERR(LOG_TAG, "capi_handle->vtbl_ptr is nullptr, exiting");
        status = -EINVAL;
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiVop::UnloadSoundModel(Stream *s __unused)
{
    return 0;
}

int32_t SoundTriggerEngineCapiVop::StartRecognition(Stream *s __unused)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    status = StartSoundEngine();
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiVop::StopBuffering(Stream *s __unused)
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
    QAL_DBG(LOG_TAG, "Exit, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineCapiVop::StopRecognition(Stream *s __unused)
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

int32_t SoundTriggerEngineCapiVop::UpdateConfLevels(
    Stream *s __unused,
    struct qal_st_recognition_config *config __unused,
    uint8_t *conf_levels,
    uint32_t num_conf_levels)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    if (!conf_levels || !num_conf_levels) {
        QAL_ERR(LOG_TAG, "Invalid config");
        return -EINVAL;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    confidence_threshold_ = *conf_levels;
    QAL_INFO(LOG_TAG, "confidence threshold: %d", confidence_threshold_);

    return status;
}

void SoundTriggerEngineCapiVop::SetDetected(bool detected)
{
    StreamSoundTrigger *str = nullptr;
    struct detection_event_info *info = nullptr;

    QAL_INFO(LOG_TAG, "SetDetected %d", detected);
    std::lock_guard<std::mutex> lck(event_mutex_);
    if (detected != processing_started_) {
        // check if we can get detection info if detected = true
        if (detected) {
            str = dynamic_cast<StreamSoundTrigger *>(stream_handle_);
            info = str->GetDetectionEventInfo();
            if (!info) {
                QAL_ERR(LOG_TAG, "Failed to get detection event info");
                return;
            }
            confidence_score_ = info->confidence_levels[1];
        }
        processing_started_ = detected;
        exit_buffering_ = !processing_started_;
        QAL_INFO(LOG_TAG, "setting processing started %d", detected);
        cv_.notify_one();
    } else {
        QAL_VERBOSE(LOG_TAG, "processing started unchanged");
    }
}
