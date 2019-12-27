/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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

#ifndef DEVICE_H
#define DEVICE_H
#include <iostream>
#include <mutex>
#include <memory>
#include "QalApi.h"
#include "QalDefs.h"
#include <string.h>
#include "QalCommon.h"
#include "Device.h"
#include "QalAudioRoute.h"

class ResourceManager;

class Device
{
protected:
    std::shared_ptr<Device> devObj;
    std::mutex mDeviceMutex;
    std::string mQALDeviceName;
    struct qal_device deviceAttr;
    std::shared_ptr<ResourceManager> rm;
    int deviceCount = 0;
    struct audio_route *audioRoute = NULL;   //getAudioRoute() from RM and store
    struct audio_mixer *audioMixer = NULL;   //getAudioMixer() from RM and store
    char mSndDeviceName[128] = {0};
    void *deviceHandle;
    bool initialized = false;
    void *customPayload;
    size_t customPayloadSize;

    Device(struct qal_device *device, std::shared_ptr<ResourceManager> Rm);
    Device();
public:
    virtual int init(qal_param_device_connection_t device_conn);
    virtual int deinit(qal_param_device_connection_t device_conn);
    virtual int getDefaultConfig(qal_param_device_capability_t capability);
    int open();
    int close();
    virtual int start();
    virtual int stop();
    int prepare();
    static std::shared_ptr<Device> getInstance(struct qal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    int getSndDeviceId();
    int getDeviceCount() { return deviceCount; }
    std::string getQALDeviceName();
    int setDeviceAttributes (struct qal_device dattr);
    virtual int getDeviceAttributes (struct qal_device *dattr);
    static std::shared_ptr<Device> getObject(qal_device_id_t dev_id);
    int updateCustomPayload(void *payload, size_t size);
    virtual int32_t setDeviceParameter(uint32_t param_id, void *param);
    virtual int32_t getDeviceParameter(uint32_t param_id, void **param);
    virtual bool isDeviceReady() { return true;}
    virtual ~Device();
};


#endif //DEVICE_H
