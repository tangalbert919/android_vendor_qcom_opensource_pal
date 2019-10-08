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

#define LOG_TAG "Device"
#include "Device.h"
#include "CodecDevice.h"
#include "ResourceManager.h"

Device::Device(){}

Device::~Device(){}

std::shared_ptr<Device> Device::create(struct qal_device *device,
                                              std::shared_ptr<ResourceManager> Rm)
{

    const bool isNonALSACodec = Rm->isNonALSACodec(device);

    if (isNonALSACodec) {
        //TBD: Create new ALSA codec here.
        QAL_ERR(LOG_TAG, "%s: Going to create NON ALSA Codec", __func__);
        return NULL;

    } else {
        return CodecDevice::getInstance(device, Rm); //could be NULL if device is unsupported
    }
}

int Device::getDeviceAtrributes(struct qal_device *dattr)
{
    int status = 0;
    if (!dattr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"Invalid device attributes status %d", status);
        goto exit;
    }
    casa_osal_memcpy(dattr, sizeof(struct qal_device), &deviceAttr, sizeof(struct qal_device));
exit:
    return status;
}


int Device::setDeviceAttributes(struct qal_device dattr)
{
    int status = 0;
    casa_osal_memcpy(&deviceAttr, sizeof(struct qal_device), &dattr, sizeof(struct qal_device));
    return status;
}


int Device::getSndDeviceId()
{
    QAL_VERBOSE(LOG_TAG,"Device Id %d acquired", deviceAttr.id);
    return deviceAttr.id;
}

std::string Device::getQALDeviceName()
{
    QAL_VERBOSE(LOG_TAG, "%s: Device name %s acquired", __func__, mQALDeviceName.c_str());
    return mQALDeviceName;
}

