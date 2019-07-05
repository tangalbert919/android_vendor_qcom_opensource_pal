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

#ifndef CODEC_DEVICE_H
#define CODEC_DEVICE_H

#include "Device.h"
#include "QalAudioRoute.h"

class CodecDevice : public Device
{
protected:
    std::shared_ptr<Device> devObj;
    int deviceCount = 0;
    struct audio_route *audioRoute = NULL;   //getAudioRoute() from RM and store
    struct audio_mixer *audioMixer = NULL;   //getAudioMixer() from RM and store
    char deviceName[128] = {0};
    void *deviceHandle;
    struct pcm *pcmFd = NULL; //pcm_open fd

    CodecDevice(struct qal_device *device, std::shared_ptr<ResourceManager> Rm);
    CodecDevice();
public:
    int open() override;
    int close() override;
    int start() override;
    int stop() override;
    int prepare() override;
    static std::shared_ptr<Device> getInstance(struct qal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    ~CodecDevice();
};


#endif //CODEC_DEVICE_H
