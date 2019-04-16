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

#define LOG_TAG "StreamCompress"
#include "StreamCompress.h"
#include "Session.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"

StreamCompress::StreamCompress(struct qal_stream_attributes *sAttr, struct qal_device *dAttr, uint32_t noOfDevices,
                   struct modifier_kv *modifiers, uint32_t noOfModifiers, std::shared_ptr<ResourceManager> rm)
{
    Session* session = NULL;
    dev = nullptr;

    QAL_ERR(LOG_TAG,"%s: enter", __func__);

    #ifdef CONFIG_GSL
        session = new SessionGsl(rm);
    #else
        session = new SessionAlsaCompress();
    #endif

    if (session == NULL){
        QAL_ERR(LOG_TAG,"%s: session creation failed", __func__);
        throw std::runtime_error("failed to create session object");
    }

    dev = Device::create(dAttr, rm);
    if (dev == nullptr) {
        QAL_ERR(LOG_TAG,"%s: Device creation is failed", __func__);
        throw std::runtime_error("failed to create device object");
    }
    devices.push_back(dev);
}

int32_t StreamCompress::open()
{
    return 0;
}

int32_t StreamCompress::close()
{
    return 0;
}

int32_t StreamCompress::stop()
{
    return 0;
}

int32_t StreamCompress::start()
{
    return 0;
}

int32_t StreamCompress::prepare()
{
    return 0;
}

int32_t StreamCompress::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    return 0;
}

int32_t StreamCompress::read(struct qal_buffer *buf)
{
    return 0;
}

int32_t StreamCompress::write(struct qal_buffer *buf)
{
    return 0;
}

int32_t StreamCompress::registerCallBack(qal_stream_callback cb)
{
    return 0;
}

int32_t StreamCompress::getCallBack(qal_stream_callback *cb)
{
    return 0;
}

int32_t StreamCompress::setParameters(uint32_t param_id, void *payload)
{
    return 0;
}

int32_t  StreamCompress::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;
    return status;
}
int32_t  StreamCompress::setMute( bool state)
{
    int32_t status = 0;
    return status;
}

int32_t  StreamCompress::setPause()
{
    int32_t status = 0;
    return status;
}

int32_t  StreamCompress::setResume()
{
    int32_t status = 0;
    return status;
}
