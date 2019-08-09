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



typedef enum {
    GRAPH = 0,
    MODULE ,
    CALIBRATION,
    CODEC_DMA_CONFIG,
    MEDIA_CONFIG,
    IN_MEDIA_CONFIG,
    OUT_MEDIA_CONFIG
}configType;

class Stream;
class ResourceManager;
class Session
{
protected:
    void * handle_t;
    std::mutex mutex;
    Session();
    struct audio_route;
    struct audio_mixer;
    std::shared_ptr<ResourceManager> rm;
public:
    ~Session();
    virtual int open(Stream * s) = 0;
    virtual int prepare(Stream * s) = 0;
    virtual int setConfig(Stream * s, configType type, int tag) = 0;
    //virtual int getConfig(Stream * s) = 0;
    virtual int start(Stream * s) = 0;
    virtual int stop(Stream * s) = 0;
    virtual int close(Stream * s) = 0;
    virtual int readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) = 0;
    virtual int writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) = 0;
    virtual int read(Stream *s, int tag, struct qal_buffer *buf, int * size) = 0;
    virtual int write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) = 0;
    virtual int getParameters(Stream *s, int tagId, uint32_t param_id, void **payload) = 0;
    virtual int setParameters(Stream *s, int tagId, uint32_t param_id, void *payload) = 0;
};

#endif //SESSION_H
