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

#define LOG_TAG "Stream"
#include "Stream.h"
#include "StreamPCM.h"
#include "StreamCompress.h"
#include "StreamSoundTrigger.h"
#include "Session.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"

std::shared_ptr<ResourceManager> Stream::rm = nullptr;
std::mutex Stream::mtx;

Stream* Stream::create(struct qal_stream_attributes *sAttr, struct qal_device *dAttr,
    uint32_t noOfDevices, struct modifier_kv *modifiers, uint32_t noOfModifiers)
{
    std::lock_guard<std::mutex> lock(mtx);
    Stream* stream = NULL;

    if (!sAttr) {
        QAL_ERR(LOG_TAG,"%s: Invalid stream attributes", __func__);
        goto exit;
    }

    /* get RM instance */
    if (rm == nullptr) {
        rm = ResourceManager::getInstance();
        if (rm == nullptr) {
            QAL_ERR(LOG_TAG,"ResourceManager getInstance failed");
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG,"%s: get RM instance success", __func__);

    if (rm->isStreamSupported(sAttr, dAttr, noOfDevices)) {
        switch (sAttr->type) {
            case QAL_STREAM_LOW_LATENCY:
            case QAL_STREAM_DEEP_BUFFER:
            case QAL_STREAM_GENERIC:
            case QAL_STREAM_VOIP_TX :
            case QAL_STREAM_VOIP_RX :
                //TODO:for now keeping QAL_STREAM_PLAYBACK_GENERIC for ULLA need to check
                stream = new StreamPCM(sAttr, dAttr, noOfDevices, modifiers, noOfModifiers, rm);
                break;
            case QAL_STREAM_COMPRESSED:
                stream = new StreamCompress(sAttr, dAttr, noOfDevices, modifiers, noOfModifiers, rm);
                break;
            case QAL_STREAM_VOICE_UI:
                stream = new StreamSoundTrigger(sAttr, dAttr, noOfDevices, modifiers, noOfModifiers, rm);
                break;
            default:
                QAL_ERR(LOG_TAG,"%s: unsupported stream type %d", __func__, sAttr->type);
                break;
        }
    } else {
        QAL_ERR(LOG_TAG,"Requested config not supported");
        goto exit;
    }
exit:
    if (!stream) {
        QAL_ERR(LOG_TAG,"%s: stream creation success", __func__);
    } else {
        QAL_ERR(LOG_TAG,"%s: stream creation failed", __func__);
    }
    return stream;
}

int32_t  Stream::getStreamAttributes(struct qal_stream_attributes *sAttr)
{
    int32_t status = 0;

    if (!sAttr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream attribute pointer", __func__);
        goto exit;
    }

    memcpy(sAttr, attr, sizeof(qal_stream_attributes));
    QAL_ERR(LOG_TAG,"%s: stream_type - %d stream_flags - %d direction - %d",
           __func__, sAttr->type, sAttr->flags, sAttr->direction);

exit:
    return status;
}

int32_t  Stream::getModifiers(struct modifier_kv *modifiers,uint32_t *noOfModifiers)
{
    int32_t status = 0;

    if (!modifiers) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid modifers pointer", __func__);
        goto exit;
    }
    memcpy (modifiers, modifiers_, sizeof(modifier_kv));
    *noOfModifiers = uNoOfModifiers;

exit:
    return status;
}

int32_t  Stream::getStreamType (qal_stream_type_t* streamType)
{
    int32_t status = 0;

    if (!streamType) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream type", __func__);
        goto exit;
    }
    *streamType = attr->type;
    QAL_ERR(LOG_TAG,"%s: streamType - %d", __func__, *streamType);

exit:
    return status;
}

int32_t  Stream::getAssociatedDevices(std::vector <std::shared_ptr<Device>> &aDevices)
{
    int32_t status = 0;
    QAL_ERR(LOG_TAG,"%s: no. of devices - %d", __func__, devices.size());
    for (int32_t i=0; i < devices.size(); i++) {
        aDevices.push_back(devices[i]);
    }

exit:
    return status;
}

int32_t  Stream::getAssociatedSession(Session **s)
{
    int32_t status = 0;

    if (!s) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid session\n", __func__);
        goto exit;
    }
    *s = session;
    QAL_ERR(LOG_TAG,"%s: session - %p", __func__, s);
exit:
    return status;
}

int32_t  Stream::getVolumeData(struct qal_volume_data *vData)
{
    int32_t status = 0;

    if (!vData) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream attribute pointer", __func__);
        goto exit;
    }
    
    if (vdata != NULL) {
    memcpy(vData, vdata,sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (vdata->no_of_volpair)));

    QAL_ERR(LOG_TAG,"%s num config %x \n",__func__, (vdata->no_of_volpair));
    for(int32_t i=0; i < (vdata->no_of_volpair); i++) {
        QAL_ERR(LOG_TAG,"%s: Volume payload mask:%x vol:%f\n",
                  __func__, (vdata->volume_pair[i].channel_mask), (vdata->volume_pair[i].vol));
    }
    }
exit:
    return status;
}
