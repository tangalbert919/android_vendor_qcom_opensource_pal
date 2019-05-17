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

#define LOG_TAG "TriMic"

#include "TriMic.h"
#include <tinyalsa/asoundlib.h>
#include "QalAudioRoute.h"

#include "ResourceManager.h"
#include "Device.h"

std::shared_ptr<Device> TriMic::obj = nullptr;

std::shared_ptr<Device> TriMic::getInstance(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    if(obj == nullptr) {
        std::shared_ptr<Device> sp(new TriMic(device, Rm));
        obj = sp;
    }
    return obj;
}


TriMic::TriMic(struct qal_device *device, std::shared_ptr<ResourceManager> Rm) :
CodecDevice(device, Rm)
{
   
}

TriMic::~TriMic()
{

}

int32_t TriMic::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    QAL_ERR(LOG_TAG,"%s:%d",__func__,__LINE__);
    switch(sampleRate) {
        case 48000:
        case 96000:
            break;
        default:
            QAL_ERR(LOG_TAG,"sample rate not supported");
            rc = -EINVAL;
            break;
    }
    return rc;
}

int32_t TriMic::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    QAL_ERR(LOG_TAG,"%s:%d",__func__,__LINE__);
    switch(numChannels) {
        case 1:
        case 2:
        case 4:
        case 8:
            break;
        default:
            QAL_ERR(LOG_TAG,"channels not supported");
            rc = -EINVAL;
            break;
    }
    return rc;
}

int32_t TriMic::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    QAL_ERR(LOG_TAG,"%s:%d",__func__,__LINE__);
    switch(bitWidth) {
        case 16:
        case 24:
        case 32:
            break;
        default:
            QAL_ERR(LOG_TAG,"bit width not supported");
            rc = -EINVAL;
            break;
    }
    return rc;
}
