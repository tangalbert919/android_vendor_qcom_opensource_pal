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


#ifndef SOUNDTRIGGERENGINECAPIVOP_H
#define SOUNDTRIGGERENGINECAPIVOP_H

#include "SoundTriggerEngine.h"
#include "capi_v2.h"
#include "capi_v2_extn.h"
#include "QalRingBuffer.h"

#include <condition_variable>
#include <thread>

class Stream;

class SoundTriggerEngineCapiVop : public SoundTriggerEngine
{
protected:
    struct qal_st_sound_model *pSoundModel; /* are we planning parse and repopulate this from stream side?*/
    int32_t confidence_threshold_;
    int32_t prepare_sound_engine();
    int32_t start_sound_engine();
    int32_t stop_sound_engine();
    int32_t start_keyword_detection();


    static std::shared_ptr<SoundTriggerEngineCapiVop> sndEngCapiVop_;
    static void buffer_thread_loop();
    std::thread bufferThreadHandler_;
    uint32_t buffer_size_;
    std::mutex mutex_;
    std::condition_variable cv_;
    capi_v2_t *capi_handle_;
    void* capi_lib_handle_;
    capi_v2_init_f  capi_init;
    bool exit_thread_;
    bool exit_buffering_;
    std::shared_ptr<QalRingBufferReader> ringBufferReader;

    uint32_t buffer_start_;
    uint32_t buffer_end_; /* externally to allow engine to know where it can stop and start processing*/

    uint64_t kw_start_timestamp_; /* input from 1st stage*/
    uint64_t kw_end_timestamp_;


    uint32_t bytes_processed_;

    uint32_t kw_start_idx_;
    uint32_t kw_end_idx_;
    uint32_t confidence_score_;

public:
    SoundTriggerEngineCapiVop(Stream *s, uint32_t id, uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer);
    ~SoundTriggerEngineCapiVop();
    int32_t load_sound_model(Stream *s, uint8_t *data, uint32_t num_models) override;
    int32_t unload_sound_model(Stream *s) override;
    int32_t start_recognition(Stream *s) override;
    int32_t stop_buffering(Stream *s) override;
    int32_t stop_recognition(Stream *s) override;
    int32_t update_config(Stream *s, struct qal_st_recognition_config *config) override;
    int32_t getParameters(uint32_t param_id, void **payload) override;
    void setDetected(bool detected) override;
};

#endif //SOUNDTRIGGERENGINECAPIVOP_H

