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

#ifndef SESSION_H
#define SESSION_H

#include "QalDefs.h"
#include <mutex>
#include <algorithm>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <errno.h>
#include "QalCommon.h"
#include "Device.h"



typedef enum {
    GRAPH = 0,
    MODULE ,
    CALIBRATION,
    CODEC_DMA_CONFIG,
    MEDIA_CONFIG,
    IN_MEDIA_CONFIG,
    OUT_MEDIA_CONFIG
}configType;

typedef enum {
    SESSION_IDLE,
    SESSION_OPENED,
    SESSION_STARTED,
    SESSION_STOPPED,
}sessionState;

typedef void (*session_callback)(void *hdl, uint32_t event_id, void *event_data);

class Stream;
class ResourceManager;
class Session
{
protected:
    void * handle_t;
    std::mutex mutex;
    Session();
    std::shared_ptr<ResourceManager> rm;
public:
    virtual ~Session();
    static Session* makeSession(const std::shared_ptr<ResourceManager>& rm, const struct qal_stream_attributes *sAttr);
    virtual int open(Stream * s) = 0;
    virtual int prepare(Stream * s) = 0;
    virtual int setConfig(Stream * s, configType type, int tag) = 0;
    virtual int setConfig(Stream * s, configType type, uint32_t tag1,
            uint32_t tag2, uint32_t tag3) {return 0;};
    virtual int setTKV(Stream * s, configType type, effect_qal_payload_t *payload) {return 0;};
    //virtual int getConfig(Stream * s) = 0;
    virtual int start(Stream * s) = 0;
    virtual int stop(Stream * s) = 0;
    virtual int close(Stream * s) = 0;
    virtual int readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) {return 0;};
    virtual int writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) {return 0;};
    virtual int read(Stream *s, int tag, struct qal_buffer *buf, int * size) {return 0;};
    virtual int write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) {return 0;};
    virtual int getParameters(Stream *s, int tagId, uint32_t param_id, void **payload) {return 0;};
    virtual int setParameters(Stream *s, int tagId, uint32_t param_id, void *payload) {return 0;};
    virtual int registerCallBack(session_callback cb, void *cookie) {return 0;};
    virtual int drain(qal_drain_type_t type) {return 0;};
    virtual int flush() {return 0;};
    virtual int getTimestamp(struct qal_session_time *stime) {return 0;};
    /*TODO need to implement connect/disconnect in basecase*/
    virtual int connectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToCconnect) = 0;
    virtual int disconnectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToDisconnect)= 0;
    void getSamplerateChannelBitwidthTags(struct qal_media_config *config,
        uint32_t &sr_tag, uint32_t &ch_tag, uint32_t &bitwidth_tag);

};

#endif //SESSION_H
