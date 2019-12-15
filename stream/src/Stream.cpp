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
std::mutex Stream::mBaseStreamMutex;
struct qal_device* Stream::mQalDevice = nullptr;

Stream* Stream::create(struct qal_stream_attributes *sAttr, struct qal_device *dAttr,
    uint32_t noOfDevices, struct modifier_kv *modifiers, uint32_t noOfModifiers)
{
    std::lock_guard<std::mutex> lock(mBaseStreamMutex);
    Stream* stream = NULL;
    int status = 0;
    if (!sAttr || !dAttr) {
        QAL_ERR(LOG_TAG, "Invalid input paramters");
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Enter. %s", __func__);
    /* get RM instance */
    if (!rm) {
        rm = ResourceManager::getInstance();
        if (!rm) {
            QAL_ERR(LOG_TAG, "ResourceManager getInstance failed");
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "get RM instance success and noOfDevices %d \n", noOfDevices);
    mQalDevice = new qal_device [noOfDevices];
    if (!mQalDevice) {
        QAL_ERR(LOG_TAG, "mQalDevice not created");
        goto exit;
    }
    for (int i = 0; i < noOfDevices; i++) {
        //TODO: shift this to rm or somewhere else where we can read the supported config from xml
        mQalDevice[i].id = dAttr[i].id;
        status = rm->getDeviceConfig((struct qal_device *)&mQalDevice[i], sAttr);
        if (status) {
           QAL_ERR(LOG_TAG, "Device config not overwritten %d", status);
        }
    }
    if (rm->isStreamSupported(sAttr, mQalDevice, noOfDevices)) {
        switch (sAttr->type) {
            case QAL_STREAM_LOW_LATENCY:
            case QAL_STREAM_DEEP_BUFFER:
            case QAL_STREAM_GENERIC:
            case QAL_STREAM_VOIP_TX:
            case QAL_STREAM_VOIP_RX:
            case QAL_STREAM_PCM_OFFLOAD:
            case QAL_STREAM_VOICE_CALL:
                //TODO:for now keeping QAL_STREAM_PLAYBACK_GENERIC for ULLA need to check
                stream = new StreamPCM(sAttr, mQalDevice, noOfDevices, modifiers,
                                   noOfModifiers, rm);
                break;
            case QAL_STREAM_COMPRESSED:
                stream = new StreamCompress(sAttr, mQalDevice, noOfDevices, modifiers,
                                        noOfModifiers, rm);
                break;
            case QAL_STREAM_VOICE_UI:
                stream = new StreamSoundTrigger(sAttr, mQalDevice, noOfDevices, modifiers,
                                            noOfModifiers, rm);
                break;
            default:
                QAL_ERR(LOG_TAG, "unsupported stream type 0x%x", sAttr->type);
                break;
        }
    } else {
        QAL_ERR(LOG_TAG,"Requested config not supported");
        goto exit;
    }
exit:
    if (stream) {
        QAL_DBG(LOG_TAG, "Exit. stream creation success");
    } else {
        QAL_ERR(LOG_TAG, "stream creation failed");
    }
    return stream;
}

int32_t  Stream::getStreamAttributes(struct qal_stream_attributes *sAttr)
{
    int32_t status = 0;

    if (!sAttr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream attribute pointer, status %d", status);
        goto exit;
    }
    casa_osal_memcpy(sAttr, sizeof(qal_stream_attributes), mStreamAttr,
                     sizeof(qal_stream_attributes));
    QAL_DBG(LOG_TAG, "stream_type %d stream_flags %d direction %d",
            sAttr->type, sAttr->flags, sAttr->direction);

exit:
    return status;
}

int32_t  Stream::getModifiers(struct modifier_kv *modifiers,uint32_t *noOfModifiers)
{
    int32_t status = 0;

    if (!mModifiers || !noOfModifiers) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid modifers pointer, status %d", status);
        goto exit;
    }
    casa_osal_memcpy (modifiers, sizeof(modifier_kv), mModifiers,
                      sizeof(modifier_kv));
    *noOfModifiers = mNoOfModifiers;
    QAL_DBG(LOG_TAG, "noOfModifiers %u", *noOfModifiers);
exit:
    return status;
}

int32_t  Stream::getStreamType (qal_stream_type_t* streamType)
{
    int32_t status = 0;

    if (!streamType || !mStreamAttr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream type, status %d", status);
        goto exit;
    }
    *streamType = mStreamAttr->type;
    QAL_DBG(LOG_TAG, "streamType - %d", *streamType);

exit:
    return status;
}

int32_t  Stream::getAssociatedDevices(std::vector <std::shared_ptr<Device>> &aDevices)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "no. of devices %d", mDevices.size());
    for (int32_t i=0; i < mDevices.size(); i++) {
        aDevices.push_back(mDevices[i]);
    }

exit:
    return status;
}

int32_t  Stream::getAssociatedSession(Session **s)
{
    int32_t status = 0;

    if (!s) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid session, status %d", status);
        goto exit;
    }
    *s = session;
    QAL_DBG(LOG_TAG, "session %pK", *s);
exit:
    return status;
}

int32_t  Stream::getVolumeData(struct qal_volume_data *vData)
{
    int32_t status = 0;

    if (!vData) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream attribute pointer, status %d", status);
        goto exit;
    }

    if (mVolumeData != NULL) {
        casa_osal_memcpy(vData, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (mVolumeData->no_of_volpair)),
                      mVolumeData, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (mVolumeData->no_of_volpair)));

        QAL_DBG(LOG_TAG, "num config %x", (mVolumeData->no_of_volpair));
        for(int32_t i=0; i < (mVolumeData->no_of_volpair); i++) {
            QAL_VERBOSE(LOG_TAG, "Volume payload mask:%x vol:%f",
                (mVolumeData->volume_pair[i].channel_mask), (mVolumeData->volume_pair[i].vol));
        }
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "volume has not been set, status %d", status);
    }
exit:
    return status;
}
int32_t Stream::setBufInfo(size_t *in_buf_size, size_t in_buf_count,
                           size_t *out_buf_size, size_t out_buf_count)
{
    int32_t status = 0;
    int16_t nBlockAlignIn, nBlockAlignOut ;        // block size of data
    struct qal_stream_attributes *sattr = NULL;
    sattr = (struct qal_stream_attributes *)calloc(1, sizeof(struct qal_stream_attributes));
    if (!sattr) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "stream attribute malloc failed %s, status %d", strerror(errno), status);
        goto exit;
    }
    
    if (!in_buf_size)
        QAL_DBG(LOG_TAG, "%s: In Buffer size %d, In Buffer count %d",
            __func__, *in_buf_size, in_buf_count);

    if (!out_buf_size)
        QAL_DBG(LOG_TAG, "%s: Out Buffer size %d and Out Buffer count %d",
        __func__, *out_buf_size, out_buf_count);
    inBufCount = in_buf_count;
    outBufCount = out_buf_count;

    status = getStreamAttributes(sattr);
    if (sattr->direction == QAL_AUDIO_OUTPUT) {
        if(!out_buf_size) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid output buffer size status %d", status);
            goto exit;
        }
        outBufSize = *out_buf_size;
        nBlockAlignOut = ((sattr->out_media_config.bit_width) / 8) *
                      (sattr->out_media_config.ch_info->channels);
        QAL_ERR(LOG_TAG, "no of buf %d and send buf %x", outBufCount, outBufSize);

        //If the read size is not a multiple of BlockAlign;
        //Make sure we read blockaligned bytes from the file.
        if ((outBufSize % nBlockAlignOut) != 0) {
            outBufSize = ((outBufSize / nBlockAlignOut) * nBlockAlignOut);
        }
        *out_buf_size = outBufSize;

    } else if (sattr->direction == QAL_AUDIO_INPUT) {
        if(!in_buf_size) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid input buffer size status %d", status);
            goto exit;
        }
        inBufSize = *in_buf_size;
        //inBufSize = (sattr->in_media_config.bit_width) * (sattr->in_media_config.ch_info->channels) * 32;
        nBlockAlignIn = ((sattr->in_media_config.bit_width) / 8) *
                      (sattr->in_media_config.ch_info->channels);
        //If the read size is not a multiple of BlockAlign;
        //Make sure we read blockaligned bytes from the file.
        if ((inBufSize % nBlockAlignIn) != 0) {
            inBufSize = ((inBufSize / nBlockAlignIn) * nBlockAlignIn);
        }
        *in_buf_size = inBufSize;
    } else {
        if(!in_buf_size || !out_buf_size) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid buffer size status %d", status);
            goto exit;
        }
        outBufSize = *out_buf_size;
        nBlockAlignOut = ((sattr->out_media_config.bit_width) / 8) *
                      (sattr->out_media_config.ch_info->channels);
        QAL_DBG(LOG_TAG, "no of buf %d and send buf %x", outBufCount, outBufSize);

        //If the read size is not a multiple of BlockAlign;
        //Make sure we read blockaligned bytes from the file.
        if ((outBufSize % nBlockAlignOut) != 0) {
            outBufSize = ((outBufSize / nBlockAlignOut) * nBlockAlignOut);
        }
        *out_buf_size = outBufSize;

        inBufSize = *in_buf_size;
        nBlockAlignIn = ((sattr->in_media_config.bit_width) / 8) *
                      (sattr->in_media_config.ch_info->channels);
        //If the read size is not a multiple of BlockAlign;
        //Make sure we read blockaligned bytes from the file.
        if ((inBufSize % nBlockAlignIn) != 0) {
            inBufSize = ((inBufSize / nBlockAlignIn) * nBlockAlignIn);
        }
        *in_buf_size = inBufSize;
    }
exit:
    return status;
}

int32_t Stream::getBufInfo(size_t *in_buf_size, size_t *in_buf_count,
                           size_t *out_buf_size, size_t *out_buf_count)
{
    int32_t status = 0;

    if (in_buf_size)
        *in_buf_size = inBufSize;
    if (in_buf_count)
        *in_buf_count = inBufCount;
    if (out_buf_size)
        *out_buf_size = outBufSize;
    if (out_buf_count)
        *out_buf_count = outBufCount;

    if (!in_buf_size)
        QAL_DBG(LOG_TAG, "%s: In Buffer size %d, In Buffer count %d",
        __func__, *in_buf_size, in_buf_count);

    if (!out_buf_size)
        QAL_DBG(LOG_TAG, "%s: Out Buffer size %d and Out Buffer count %d",
        __func__, *out_buf_size, out_buf_count);

    return status;
}

bool Stream::isStreamAudioOutFmtSupported(qal_audio_fmt_t format)
{
    switch (format) {
    case QAL_AUDIO_FMT_DEFAULT_PCM:
    case QAL_AUDIO_FMT_MP3:
    case QAL_AUDIO_FMT_AAC:
    case QAL_AUDIO_FMT_AAC_ADTS:
    case QAL_AUDIO_FMT_AAC_ADIF:
    case QAL_AUDIO_FMT_AAC_LATM:
    case QAL_AUDIO_FMT_WMA_STD:
    case QAL_AUDIO_FMT_ALAC:
    case QAL_AUDIO_FMT_APE:
    case QAL_AUDIO_FMT_WMA_PRO:
    case QAL_AUDIO_FMT_FLAC:
    case QAL_AUDIO_FMT_VORBIS:
        return true;
    default:
        return false;
    }
}

int32_t Stream::getTimestamp(struct qal_session_time *stime)
{
    int32_t status = 0;
    if (!stime) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid session time pointer, status %d", status);
        goto exit;
    }
    status = session->getTimestamp(stime);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to get session timestamp status %d", status);
    }
exit:
    return status;
}

int32_t Stream::switchDevice(Stream* streamHandle, uint32_t no_of_devices, struct qal_device *deviceArray)
{
    int32_t status = -EINVAL;
    mStreamMutex.lock();
    std::shared_ptr<Device> dev = nullptr;

    if ((no_of_devices == 0) || (!deviceArray)) {
        QAL_ERR(LOG_TAG, "invalid param for device switch");
        status = -EINVAL;
        goto error_1;
    }

    //tell rm we are disabling existing mDevices, so that it can disable any streams running on
    // 1. mDevices with common backend
    //TBD: as there are no devices with common backend now.
    //rm->disableDevice(mDevices);

    QAL_ERR(LOG_TAG, "device %d name %s, going to stop %d devices",
        mDevices[0]->getSndDeviceId(), mDevices[0]->getQALDeviceName().c_str(), mDevices.size());

    for (int i = 0; i < mDevices.size(); i++) {
        session->disconnectSessionDevice(streamHandle, mStreamAttr->type, mDevices[i]);
        QAL_ERR(LOG_TAG, "device %d name %s, going to stop",
            mDevices[i]->getSndDeviceId(), mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx device stop failed with status %d", status);
            goto error_1;
        }

        rm->deregisterDevice(mDevices[i]);

        status = mDevices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close failed with status %d", status);
            goto error_1;
        }

    }

    //clear existing devices and enable new devices
    mDevices.clear();

    QAL_ERR(LOG_TAG, "Incoming device count %d, first id %d, stream_type = %d", no_of_devices, deviceArray[0].id, mStreamAttr->type);

    /* overwrite device config with default one for speaker and stream rate for headset */
    /*TODO: handle other devices */
    for (int i = 0; i < no_of_devices; i++) {
        status = rm->getDeviceConfig((struct qal_device *)&deviceArray[i], mStreamAttr);
        if (status)
           QAL_ERR(LOG_TAG, "Device config not overwritten %d", status);

        //Check with RM if the configuration given can work or not
        //for e.g., if incoming stream needs 24 bit device thats also
        //being used by another stream, then the other stream should route

        dev = Device::getInstance((struct qal_device *)&deviceArray[i], rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation failed");
            free(mStreamAttr);
            //TBD::free session too
            mStreamMutex.unlock();
            throw std::runtime_error("failed to create device object");
        }

        status = dev->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device %d open failed with status %d",
                dev->getSndDeviceId(), status);
            goto error_2;
        }

        QAL_ERR(LOG_TAG, "device %d name %s, going to start",
            dev->getSndDeviceId(), dev->getQALDeviceName().c_str());

        mDevices.push_back(dev);
        status = session->setupSessionDevice(streamHandle, mStreamAttr->type, dev);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "setupSessionDevice for %d failed with status %d",
                    dev->getSndDeviceId(), status);
            mDevices.pop_back();
            dev->close();
            goto error_2;
        }

        rm->registerDevice(dev);

        status = dev->start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device %d name %s, start failed with status %d",
                dev->getSndDeviceId(), dev->getQALDeviceName().c_str(), status);
            goto error_3;
        }
        session->connectSessionDevice(streamHandle, mStreamAttr->type, dev);
        dev = nullptr;
    }

    goto error_1;

error_3:
    mDevices.pop_back();
    rm->deregisterDevice(dev);
    dev->close();
error_2:
    if (mStreamAttr) {
        free(mStreamAttr->out_media_config.ch_info);
        free(mStreamAttr);
        mStreamAttr = NULL;
    }
error_1:
    mStreamMutex.unlock();
    return status;

}


