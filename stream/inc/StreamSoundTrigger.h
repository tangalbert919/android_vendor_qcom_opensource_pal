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
class Device;
class Session;
class SoundTriggerEngine;

class StreamSoundTrigger : public Stream
{

    union {};
protected:
    qal_stream_callback callBack;
    std::vector<std::pair<uint32_t, SoundTriggerEngine *>> activeEngines;
    QalRingBufferReader *reader_;

public:
    StreamSoundTrigger(struct qal_stream_attributes *sattr, struct qal_device *dattr, uint32_t no_of_devices,
             struct modifier_kv *modifiers, uint32_t no_of_modifiers, std::shared_ptr<ResourceManager> rm);
    ~StreamSoundTrigger();
    int32_t open() override;
    int32_t close() override;
    int32_t start() override;
    int32_t stop() override;
    int32_t prepare() override;
    int32_t setStreamAttributes( struct qal_stream_attributes *sattr) override;
    int32_t setVolume( struct qal_volume_data *volume) override;
    int32_t setMute( bool state) override;
    int32_t setPause() override;
    int32_t setResume() override;
    int32_t read(struct qal_buffer *buf) override;
    int32_t write(struct qal_buffer *buf) override;
    int32_t registerCallBack(qal_stream_callback cb) override;
    int32_t getCallBack(qal_stream_callback *cb) override;
    int32_t setParameters(uint32_t param_id, void *payload) override;
    void registerSoundTriggerEngine(uint32_t id, SoundTriggerEngine *stEngine);
    void deregisterSoundTriggerEngine(uint32_t id);
    int32_t getSoundTriggerEngine(int *index, uint32_t sm_id);
    int32_t setDetected(bool detected);
    int32_t getDetectionEventInfo(struct detection_event_info **info);
    friend class QalRingBufferReader;
private:
    int32_t stages;
    qal_st_sound_model_type_t sound_model_type;
    uint32_t recognition_mode;
    uint8_t *sm_data;                   //This needs to be moved down to individual classes
    struct qal_st_recognition_config *sm_rc_config;
    uint8_t* conf_levels_payload;       //confidence Level
    uint32_t num_of_conf_levels;        //these 2 need to be moved down to SM classes
    struct detection_event_info detectionEventInfo;
    /* functions*/
    int32_t parse_sound_model(struct qal_st_sound_model *sm_data);
    int32_t parse_rc_config(struct qal_st_recognition_config *rc_config);
    int32_t generate_recognition_config_payload(unsigned char **out_payload, unsigned int *out_payload_size); //Need to track the buffer where the payload is allocated
    static int32_t handleDetectionEvent(qal_stream_handle_t *stream_handle, uint32_t event_id, uint32_t *event_data, void *cookie);
    int32_t parse_detection_payload(uint32_t event_id, uint32_t *event_data);
};
#endif//STREAMSOUNDTRIGGER_H_
