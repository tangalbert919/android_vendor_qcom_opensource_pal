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

    if (!sAttr || !dAttr) {
        QAL_ERR(LOG_TAG, "Invalid input paramters");
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Enter.");
    /* get RM instance */
    if (!rm) {
        rm = ResourceManager::getInstance();
        if (!rm) {
            QAL_ERR(LOG_TAG, "ResourceManager getInstance failed");
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "get RM instance success and noOfDevices %d \n", noOfDevices);

    if (rm->isStreamSupported(sAttr, dAttr, noOfDevices)) {
        switch (sAttr->type) {
            case QAL_STREAM_LOW_LATENCY:
            case QAL_STREAM_DEEP_BUFFER:
            case QAL_STREAM_GENERIC:
            case QAL_STREAM_VOIP_TX:
            case QAL_STREAM_VOIP_RX:
                //TODO:for now keeping QAL_STREAM_PLAYBACK_GENERIC for ULLA need to check
                stream = new StreamPCM(sAttr, dAttr, noOfDevices, modifiers,
                                   noOfModifiers, rm);
                break;
            case QAL_STREAM_COMPRESSED:
                stream = new StreamCompress(sAttr, dAttr, noOfDevices, modifiers,
                                        noOfModifiers, rm);
                break;
            case QAL_STREAM_VOICE_UI:
                stream = new StreamSoundTrigger(sAttr, dAttr, noOfDevices, modifiers,
                                            noOfModifiers, rm);
                break;
            default:
                QAL_ERR(LOG_TAG, "unsupported stream type %x", sAttr->type);
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
    casa_osal_memcpy(sAttr, sizeof(qal_stream_attributes), attr,
                     sizeof(qal_stream_attributes));
    QAL_DBG(LOG_TAG, "stream_type %d stream_flags %d direction %d",
            sAttr->type, sAttr->flags, sAttr->direction);

exit:
    return status;
}

int32_t  Stream::getModifiers(struct modifier_kv *modifiers,uint32_t *noOfModifiers)
{
    int32_t status = 0;

    if (!modifiers) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid modifers pointer, status %d", status);
        goto exit;
    }
    casa_osal_memcpy (modifiers, sizeof(modifier_kv), modifiers_,
                      sizeof(modifier_kv));
    *noOfModifiers = uNoOfModifiers;
    QAL_DBG(LOG_TAG, "noOfModifiers %u", *noOfModifiers);
exit:
    return status;
}

int32_t  Stream::getStreamType (qal_stream_type_t* streamType)
{
    int32_t status = 0;

    if (!streamType) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream type, status %d", status);
        goto exit;
    }
    *streamType = attr->type;
    QAL_DBG(LOG_TAG, "streamType - %d", *streamType);

exit:
    return status;
}

int32_t  Stream::getAssociatedDevices(std::vector <std::shared_ptr<Device>> &aDevices)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "no. of devices %d", devices.size());
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
    
    if (vdata != NULL) {
        casa_osal_memcpy(vData,sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (vdata->no_of_volpair)),
                      vdata, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (vdata->no_of_volpair)));

        QAL_DBG(LOG_TAG, "num config %x", (vdata->no_of_volpair));
        for(int32_t i=0; i < (vdata->no_of_volpair); i++) {
            QAL_VERBOSE(LOG_TAG, "Volume payload mask:%x vol:%f",
                (vdata->volume_pair[i].channel_mask), (vdata->volume_pair[i].vol));
        }
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
    sattr = (struct qal_stream_attributes *)malloc(sizeof(struct qal_stream_attributes));
    if (!sattr) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "sattr malloc failed %s, status %d", strerror(errno), status);
        goto exit;
    }
    memset (sattr, 0, sizeof(struct qal_stream_attributes));
    QAL_DBG(LOG_TAG, "In Buffer size %d, In Buffer count %d, Out Buffer size %d and Out Buffer count %d",
            *in_buf_size, in_buf_count, *out_buf_size,
            out_buf_count);

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
        QAL_ERR(LOG_TAG, "no of buf %d and send buf %x", outBufCount, outBufSize);

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
    *in_buf_size = inBufSize;
    *in_buf_count = inBufCount;
    *out_buf_size = outBufSize;
    *out_buf_count = outBufCount;
    QAL_VERBOSE(LOG_TAG, "In Buffer size %d, In Buffer count %d, Out Buffer size %d and Out Buffer count %d",
                *in_buf_size,*in_buf_count,
                *out_buf_size,*out_buf_count);
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
            return true;
	default:
	    return false;
	}
}

