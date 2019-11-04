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


#ifndef STREAMSOUNDTRIGGER_H_
#define STREAMSOUNDTRIGGER_H_

#include <utility>

#include "Stream.h"
#include "SoundTriggerEngine.h"
#include "QalRingBuffer.h"

/* Event Mode
 * Indicating info GECKO will notify to client
 */
#define CONFIDENCE_LEVEL_INFO    0x1
#define KEYWORD_INDICES_INFO     0x2
#define TIME_STAMP_INFO          0x4
#define FTRT_INFO                0x8

enum
{
    ENGINE_IDLE  = 0x0,
    GMM_DETECTED = 0x1,
    CNN_DETECTED = 0x2,
    CNN_REJECTED = 0x3,
    VOP_DETECTED = 0x4,
    VOP_REJECTED = 0x5,
};

struct detection_event_info
{
    uint16_t status;
    uint16_t num_confidence_levels;
    uint8_t confidence_levels[20];
    uint32_t kw_start_timestamp_lsw;
    uint32_t kw_start_timestamp_msw;
    uint32_t kw_end_timestamp_lsw;
    uint32_t kw_end_timestamp_msw;
    uint32_t detection_timestamp_lsw;
    uint32_t detection_timestamp_msw;
    uint32_t ftrt_data_length_in_us;
};

class ResourceManager;
class SoundTriggerEngine;

class StreamSoundTrigger : public Stream
{
 public:
    StreamSoundTrigger(struct qal_stream_attributes *sattr,
                       struct qal_device *dattr,
                       uint32_t no_of_devices,
                       struct modifier_kv *modifiers __unused,
                       uint32_t no_of_modifiers __unused,
                       std::shared_ptr<ResourceManager> rm);
    ~StreamSoundTrigger() {}
    int32_t open() override;
    int32_t close() override;
    int32_t start() override;
    int32_t stop() override;
    int32_t prepare() override;
    int32_t setStreamAttributes(struct qal_stream_attributes *sattr) override;
    int32_t setVolume(struct qal_volume_data * volume __unused) override;
    int32_t setMute(bool state __unused) override;
    int32_t setPause() override;
    int32_t setResume() override;
    int32_t read(struct qal_buffer *buf) override;
    int32_t write(struct qal_buffer *buf __unused) override;
    int32_t registerCallBack(qal_stream_callback cb,
        void *cookie __unused) override;
    int32_t getCallBack(qal_stream_callback *cb) override;
    int32_t getParameters(uint32_t param_id, void **payload) override;
    int32_t setParameters(uint32_t param_id, void *payload) override;
    int32_t addRemoveEffect(qal_audio_effect_t effect, bool enable) override;

    void registerSoundTriggerEngine(uint32_t id, SoundTriggerEngine *stEngine);
    void deregisterSoundTriggerEngine(uint32_t id);
    int32_t getSoundTriggerEngine(int *index, uint32_t sm_id);
    void registerSoundModelData(uint32_t id, uint8_t *data);
    void deregisterSoundModelData(uint32_t id);
    int32_t getSoundModelData(int *index, uint32_t sm_id);
    int32_t SetDetected(bool detected);
    struct detection_event_info * getDetectionEventInfo() {
        return &detection_event_info_;
    }
    int32_t notifyClient();
    int32_t setDetectionState(uint32_t state);
    static int32_t isSampleRateSupported(uint32_t sampleRate);
    static int32_t isChannelSupported(uint32_t numChannels);
    static int32_t isBitWidthSupported(uint32_t bitWidth);
    int switchDevice(Stream* stream_handle, uint32_t no_of_devices,
                     struct qal_device *device_array);

    friend class QalRingBufferReader;

 private:
    int32_t LoadSoundModel(struct qal_st_sound_model *sm_data);
    int32_t SendRecognitionConfig(struct qal_st_recognition_config *config);
    int32_t ParseDetectionPayload(uint32_t event_id, uint32_t *event_data);
    int32_t ParseOpaqueConfLevels(void *opaque_conf_levels,
                                  uint32_t version,
                                  uint8_t **out_conf_levels,
                                  uint32_t *out_num_conf_levels);
    int32_t FillConfLevels(struct qal_st_recognition_config *config,
                           uint8_t **out_conf_levels,
                           uint32_t *out_num_conf_levels);
    int32_t FillOpaqueConfLevels(const void *sm_levels_generic,
                                 uint8_t **out_payload,
                                 uint32_t *out_payload_size,
                                 uint32_t version);
    int32_t GenerateCallbackEvent(struct qal_st_recognition_event **event);
    static int32_t handleDetectionEvent(qal_stream_handle_t *stream_handle,
                                        uint32_t event_id,
                                        uint32_t *event_data,
                                        void *cookie __unused);

    int32_t stages_;
    uint8_t *sm_data_;
    std::vector<std::pair<uint32_t, uint8_t *>> active_sm_data_;
    qal_st_sound_model_type_t sound_model_type_;
    uint32_t recognition_mode_;
    struct qal_st_recognition_config *rec_config_;
    struct qal_st_recognition_event *rec_event_;
    struct detection_event_info detection_event_info_;
    uint32_t detection_state_;
    uint32_t notification_state_;
    qal_stream_callback callback_;
    QalRingBufferReader *reader_;
    SoundTriggerEngine *gsl_engine_;
    std::vector<std::pair<uint32_t, SoundTriggerEngine *>> active_engines_;
};
#endif  // STREAMSOUNDTRIGGER_H_
