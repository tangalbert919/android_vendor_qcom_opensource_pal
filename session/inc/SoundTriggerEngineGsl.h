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


#ifndef SOUNDTRIGGERENGINEGSL_H
#define SOUNDTRIGGERENGINEGSL_H

#include <condition_variable>
#include <thread>
#include "SoundTriggerEngine.h"

class Session;
class Stream;

class SoundTriggerEngineGsl : public SoundTriggerEngine
{
protected:
    struct qal_st_sound_model *pSoundModel;
    struct detection_engine_config_voice_wakeup pWakeUpConfig;
    struct detection_engine_generic_event_cfg pEventConfig;
    struct detection_engine_voice_wakeup_buffer_config pBufConfig;
    struct audio_dam_downstream_setup_duration *pSetupDuration;

    int32_t prepare_sound_engine();
    int32_t start_sound_engine();
    int32_t stop_sound_engine();
    int32_t start_keyword_detection();
    int32_t start_buffering();

    static std::shared_ptr<SoundTriggerEngineGsl> sndEngGsl_;
    static void buffer_thread_loop();
    std::thread bufferThreadHandler_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool exit_thread_;
    bool exit_buffering_;
public:
    SoundTriggerEngineGsl(Stream *s, uint32_t id, uint32_t stage_id,
                   QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer);
    ~SoundTriggerEngineGsl();
    int32_t load_sound_model(Stream *s, uint8_t *data, uint32_t num_models) override;
    int32_t unload_sound_model(Stream *s) override;
    int32_t start_recognition(Stream *s) override;
    int32_t stop_recognition(Stream *s) override;
    int32_t update_config(Stream *s, struct qal_st_recognition_config *config) override;
    void setDetected(bool detected) override;
    int32_t generate_wakeup_config(struct qal_st_recognition_config *config);
};

#endif //SOUNDTRIGGERENGINEGSL_H
