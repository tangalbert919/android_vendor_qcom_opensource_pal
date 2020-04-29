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

#define LOG_TAG "QAL: RTProxy"
#include "RTProxy.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

#include "PayloadBuilder.h"
#include "Stream.h"
#include "Session.h"

std::shared_ptr<Device> RTProxy::obj = nullptr;

std::shared_ptr<Device> RTProxy::getObject()
{
    return obj;
}


std::shared_ptr<Device> RTProxy::getInstance(struct qal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    if (!obj) {
        std::shared_ptr<Device> sp(new RTProxy(device, Rm));
        obj = sp;
    }
    return obj;
}


RTProxy::RTProxy(struct qal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

    struct qal_channel_info *device_ch_info;
    uint16_t channels = CHANNELS_2;
    uint16_t ch_info_size;

    rm = Rm;

    if (device->config.ch_info) {
        channels = device->config.ch_info->channels;
    }

    QAL_DBG(LOG_TAG, "channels %d", channels);
    ch_info_size = sizeof(uint16_t) + sizeof(uint8_t)*channels;
    device_ch_info = (struct qal_channel_info *) calloc(1, ch_info_size);
    if (device_ch_info == NULL) {
        QAL_ERR(LOG_TAG, "Allocation failed for channel map");
    }

    memset(&mDeviceAttr, 0, sizeof(struct qal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct qal_device));

    mDeviceAttr.config.ch_info = device_ch_info;
    if (device->config.ch_info)
        memcpy(mDeviceAttr.config.ch_info,
                        device->config.ch_info, ch_info_size);
    else
        QAL_ERR(LOG_TAG, "Channel Map info in NULL");

}

RTProxy::~RTProxy()
{
    if(mDeviceAttr.config.ch_info) {
        free(mDeviceAttr.config.ch_info);
        mDeviceAttr.config.ch_info = NULL;
    }
}

int32_t RTProxy::isSampleRateSupported(uint32_t sampleRate)
{
    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    /* Proxy supports all sample rates, accept by default */
    return 0;
}

int32_t RTProxy::isChannelSupported(uint32_t numChannels)
{
    QAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    /* Proxy supports all channels, accept by default */
    return 0;
}

int32_t RTProxy::isBitWidthSupported(uint32_t bitWidth)
{
    QAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    /* Proxy supports all bitwidths, accept by default */
    return 0;
}


int RTProxy::start() {
    int status = 0;
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    uint32_t ratMiid = 0;
    //struct qal_media_config config;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activestreams;
    std::shared_ptr<Device> dev = nullptr;
    std::string backEndName;
    PayloadBuilder* builder = new PayloadBuilder();

    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;

    rm->getBackendName(mDeviceAttr.id, backEndName);

    dev = Device::getInstance(&deviceAttr, rm);

    status = rm->getActiveStream_l(dev, activestreams);
    if ((0 != status) || (activestreams.size() == 0)) {
        QAL_ERR(LOG_TAG, "%s: no active stream available", __func__);
        return -EINVAL;
    }
    stream = static_cast<Stream *>(activestreams[0]);
    stream->getAssociatedSession(&session);

    status = session->getMIID(backEndName.c_str(), RAT_RENDER, &ratMiid);
    if (status) {
        QAL_ERR(LOG_TAG,
         "Failed to get tag info %x Skipping RAT Configuration Setup, status = %d",
          RAT_RENDER, status);
        //status = -EINVAL;
        status = 0;
        goto start;
    }

    builder->payloadRATConfig(&paramData, &paramSize, ratMiid, &mDeviceAttr.config);
    if (paramSize) {
        dev->updateCustomPayload(paramData, paramSize);
        free(paramData);
        paramData = NULL;
        paramSize = 0;
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid RAT module param size", __func__);
        goto error;
    }
start:
    status = Device::start();
error:
    delete builder;
    return status;
}
