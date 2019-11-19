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

#include "SoundTriggerEngine.h"
#include "QalRingBuffer.h"

class Session;
class Stream;

class SoundTriggerEngineGsl : public SoundTriggerEngine
{
 public:
    SoundTriggerEngineGsl(Stream *s, uint32_t id, uint32_t stage_id,
                          QalRingBufferReader **reader,
                          std::shared_ptr<QalRingBuffer> buffer);
    ~SoundTriggerEngineGsl();
    int32_t LoadSoundModel(Stream *s, uint8_t *data,
                           uint32_t data_size) override;
    int32_t UnloadSoundModel(Stream *s) override;
    int32_t StartRecognition(Stream *s) override;
    int32_t StopBuffering(Stream *s) override;
    int32_t StopRecognition(Stream *s) override;
    int32_t UpdateConfLevels(
        Stream *s __unused,
        struct qal_st_recognition_config *config,
        uint8_t *conf_levels,
        uint32_t num_conf_levels) override;
    int32_t UpdateBufConfig(uint32_t hist_buffer_duration,
                            uint32_t pre_roll_duration) override;
    void SetDetected(bool detected) override;
    int32_t getParameters(uint32_t param_id, void **payload) override;
    int32_t Open(Stream *s) override;
    int32_t Close(Stream *s) override;
    int32_t Prepare(Stream *s) override;
    int32_t SetConfig(Stream * s, configType type, int tag) override;
    int32_t ConnectSessionDevice(
        Stream* stream_handle,
        qal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_connect) override;
    int32_t DisconnectSessionDevice(
        Stream* stream_handle,
        qal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_disconnect) override;
    void SetCaptureRequested(bool is_requested) override;

 protected:
    int32_t Init();
    int32_t PrepareSoundEngine() {return 0;}
    int32_t StartSoundEngine();
    int32_t StopSoundEngine();
    int32_t StartBuffering();
    static void BufferThreadLoop(SoundTriggerEngineGsl *gsl_engine);
    static void HandleSessionCallBack(void *hdl, uint32_t event_id, void *data);

    bool event_detected_;
    bool timestamp_recorded_;
    bool is_stream_notified_;
    struct detection_engine_config_voice_wakeup wakeup_config_;
    struct detection_engine_generic_event_cfg event_config_;
    struct detection_engine_voice_wakeup_buffer_config buffer_config_;
    struct audio_dam_downstream_setup_duration *dam_setup_duration_;
};

#endif  // SOUNDTRIGGERENGINEGSL_H
