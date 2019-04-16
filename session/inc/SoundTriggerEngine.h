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


#ifndef SOUNDTRIGGERENGINE_H
#define SOUNDTRIGGERENGINE_H

#include "QalDefs.h"
#include <mutex>
#include <vector>
#include "QalCommon.h"
#include "QalRingBuffer.h"

class Session;
class Stream;

typedef enum {
    IDLE,
    READY,
    ACTIVE,
    BUFFERING,
    HOLD_READY,
    HOLD_ACTIVE,
} sound_model_state_t;

/* Structure representing a single sound model,
   sound model data and recognition data of all
   sound models registered merged to single blob
   of sound model data and recogntion data, which
   is set to Session. */
struct SoundModel
{
    sound_model_state_t state;
    uint32_t sm_id;
    uint32_t sm_host_id;
    void *sm_data;
    void *sm_params_data;
};

class SoundTriggerEngine
{
protected:
    uint32_t engineId;
    uint8_t *sm_data;
    uint32_t sm_data_size;
    uint8_t *sm_params_data;
    int stageId;
    Session *session;
    Stream *streamHandle;
    std::vector<struct SoundModel*> SoundModels;
    QalRingBuffer *buffer_;
    QalRingBufferReader *reader_;
    bool eventDetected;
public:
    static SoundTriggerEngine* create(Stream *s, uint32_t id, uint32_t stage_id, QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer);
    virtual int32_t load_sound_model(Stream *s, uint8_t *data) = 0;
    virtual int32_t unload_sound_model(Stream *s) = 0;
    virtual int32_t start_recognition(Stream *s) = 0;
    virtual int32_t stop_recognition(Stream *s) = 0;
    virtual int32_t update_config(Stream *s, struct qal_st_recognition_config *config) = 0;
    virtual void setDetected(bool detected) = 0;
};

#endif //SOUNDTRIGGERENGINE_H
