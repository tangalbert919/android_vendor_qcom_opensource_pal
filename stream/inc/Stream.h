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

#ifndef STREAM_H_
#define STREAM_H_

#include "QalDefs.h"
#include <algorithm>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <mutex>
#include <exception>
#include <errno.h>
#include "QalCommon.h"

typedef enum {
    DATA_MODE_SHMEM = 0,
    DATA_MODE_BLOCKING ,
    DATA_MODE_NONBLOCKING
} dataFlags;

#define BUF_SIZE 1024
#define BUF_SIZE_CAPTURE 1920
#define NO_OF_BUF 4

class Device;
class ResourceManager;
class Session;

class Stream
{
protected:
    uint32_t noOfDevices;
    std::vector <std::shared_ptr<Device>> devices;
    std::shared_ptr <Device> dev;
    Session* session;
    struct qal_stream_attributes* attr;
    std::mutex mutex;
    static std::mutex mtx;
    static std::shared_ptr<ResourceManager> rm;
    struct modifier_kv *modifiers_;
    uint32_t uNoOfModifiers;

public:
    virtual int32_t open() = 0;
    virtual int32_t close() = 0;
    virtual int32_t start() = 0;
    virtual int32_t stop() = 0;
    virtual int32_t prepare() = 0;
    virtual int32_t setStreamAttributes(struct qal_stream_attributes *sattr) = 0;
    virtual int32_t read(struct qal_buffer *buf) = 0;
    int32_t getStreamAttributes(struct qal_stream_attributes *sattr);
    int32_t getModifiers(struct modifiers_kv *modifiers,uint32_t noOfModifiers);
    int32_t getStreamType(qal_stream_type_t* streamType);
    virtual int32_t write(struct qal_buffer *buf) = 0;
    virtual int32_t registerCallBack() = 0;
    int32_t getAssociatedDevices(std::vector <std::shared_ptr<Device>> adevices);
    int32_t getAssociatedSession(Session* session);
    /* static so that this method can be accessed wihtout object */
    static Stream* create(struct qal_stream_attributes *sattr, struct qal_device *dattr,
         uint32_t no_of_devices, struct modifier_kv *modifiers, uint32_t no_of_modifiers);
};

#endif//STREAM_H_
