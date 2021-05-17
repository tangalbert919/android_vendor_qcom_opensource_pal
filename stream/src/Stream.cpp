/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: Stream"
#include "Stream.h"
#include "StreamPCM.h"
#include "StreamInCall.h"
#include "StreamCompress.h"
#include "StreamSoundTrigger.h"
#include "StreamACD.h"
#include "StreamContextProxy.h"
#include "StreamUltraSound.h"
#include "Session.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"

std::shared_ptr<ResourceManager> Stream::rm = nullptr;
std::mutex Stream::mBaseStreamMutex;
struct pal_device* Stream::mPalDevice = nullptr;
std::mutex Stream::pauseMutex;
std::condition_variable Stream::pauseCV;

void Stream::handleSoftPauseCallBack(uint64_t hdl, uint32_t event_id,
                                        void *data __unused,
                                        uint32_t event_size __unused) {

    PAL_DBG(LOG_TAG,"Event id %x ", event_id);

    if (event_id == EVENT_ID_SOFT_PAUSE_PAUSE_COMPLETE) {
        PAL_DBG(LOG_TAG, "Pause done");
        pauseCV.notify_all();
    }
}

Stream* Stream::create(struct pal_stream_attributes *sAttr, struct pal_device *dAttr,
    uint32_t noOfDevices, struct modifier_kv *modifiers, uint32_t noOfModifiers)
{
    std::lock_guard<std::mutex> lock(mBaseStreamMutex);
    Stream* stream = NULL;
    int status = 0;
    uint32_t count = 0;


    if (!sAttr || ((noOfDevices > 0) && !dAttr)) {
        PAL_ERR(LOG_TAG, "Invalid input paramters");
        goto exit;
    }
    PAL_DBG(LOG_TAG, "Enter.");
    /* get RM instance */
    if (!rm) {
        rm = ResourceManager::getInstance();
        if (!rm) {
            PAL_ERR(LOG_TAG, "ResourceManager getInstance failed");
            goto exit;
        }
    }
    PAL_VERBOSE(LOG_TAG,"get RM instance success and noOfDevices %d \n", noOfDevices);

    if (sAttr->type == PAL_STREAM_NON_TUNNEL || sAttr->type == PAL_STREAM_CONTEXT_PROXY)
        goto stream_create;

    mPalDevice = new pal_device [noOfDevices];
    if (!mPalDevice) {
        PAL_ERR(LOG_TAG, "mPalDevice not created");
        goto exit;
    }
    if (sAttr->type == PAL_STREAM_VOICE_CALL_MUSIC)
        goto stream_create;
    for (int i = 0; i < noOfDevices; i++) {
        struct pal_device_info devinfo = {};

        if (!rm->isDeviceReady(dAttr[i].id)) {
            PAL_ERR(LOG_TAG, "Device %d is not ready\n", dAttr[i].id);
            continue;  //  continue with other devices for combo usecase
        }

        //TODO: shift this to rm or somewhere else where we can read the supported config from xml
        mPalDevice[count].id = dAttr[i].id;
        if (mPalDevice[count].id == PAL_DEVICE_OUT_USB_DEVICE ||
            mPalDevice[count].id == PAL_DEVICE_OUT_USB_HEADSET ||
            mPalDevice[count].id == PAL_DEVICE_IN_USB_DEVICE ||
            mPalDevice[count].id == PAL_DEVICE_IN_USB_HEADSET) {
            mPalDevice[count].address = dAttr[i].address;
        }

        if (strlen(dAttr[i].custom_config.custom_key)) {
            strlcpy(mPalDevice[count].custom_config.custom_key, dAttr[i].custom_config.custom_key, PAL_MAX_CUSTOM_KEY_SIZE);
            PAL_DBG(LOG_TAG, "found custom key %s", dAttr[i].custom_config.custom_key);

        } else {
            strlcpy(mPalDevice[count].custom_config.custom_key, "", PAL_MAX_CUSTOM_KEY_SIZE);
            PAL_DBG(LOG_TAG, "no custom key found");
        }
        rm->getDeviceInfo(mPalDevice[count].id, sAttr->type, dAttr[i].custom_config.custom_key, &devinfo);
        if (devinfo.channels == 0 || devinfo.channels > devinfo.max_channels) {
            PAL_ERR(LOG_TAG, "Invalid num channels[%d], max channels[%d] failed to create stream",
                    devinfo.channels,
                    devinfo.max_channels);
            goto exit;
        }
        status = rm->getDeviceConfig((struct pal_device *)&mPalDevice[count], sAttr, devinfo.channels);
        if (status) {
           PAL_ERR(LOG_TAG, "Not able to get Device config %d", status);
           goto exit;
        }
        count++;
    }

    if (!count)
        goto exit;

stream_create:
    PAL_DBG(LOG_TAG, "stream type 0x%x", sAttr->type);
    if (rm->isStreamSupported(sAttr, mPalDevice, count)) {
        switch (sAttr->type) {
            case PAL_STREAM_LOW_LATENCY:
            case PAL_STREAM_DEEP_BUFFER:
            case PAL_STREAM_GENERIC:
            case PAL_STREAM_VOIP_TX:
            case PAL_STREAM_VOIP_RX:
            case PAL_STREAM_PCM_OFFLOAD:
            case PAL_STREAM_VOICE_CALL:
            case PAL_STREAM_LOOPBACK:
            case PAL_STREAM_ULTRA_LOW_LATENCY:
            case PAL_STREAM_PROXY:
            case PAL_STREAM_HAPTICS:
            case PAL_STREAM_RAW:
                //TODO:for now keeping PAL_STREAM_PLAYBACK_GENERIC for ULLA need to check
                stream = new StreamPCM(sAttr, mPalDevice, count, modifiers,
                                   noOfModifiers, rm);
                break;
            case PAL_STREAM_COMPRESSED:
                stream = new StreamCompress(sAttr, mPalDevice, count, modifiers,
                                        noOfModifiers, rm);
                break;
            case PAL_STREAM_VOICE_UI:
                stream = new StreamSoundTrigger(sAttr, mPalDevice, count, modifiers,
                                            noOfModifiers, rm);
                break;
            case PAL_STREAM_VOICE_CALL_RECORD:
            case PAL_STREAM_VOICE_CALL_MUSIC:
                stream = new StreamInCall(sAttr, mPalDevice, count, modifiers,
                                            noOfModifiers, rm);
                break;
            case PAL_STREAM_NON_TUNNEL:
                stream = new StreamNonTunnel(sAttr, NULL, 0, modifiers,
                                            noOfModifiers, rm);
                break;
            case PAL_STREAM_ACD:
                stream = new StreamACD(sAttr, mPalDevice, count, modifiers,
                                            noOfModifiers, rm);
                break;
            case PAL_STREAM_CONTEXT_PROXY:
                stream = new StreamContextProxy(sAttr, NULL, 0, modifiers,
                                            noOfModifiers, rm);
                break;
            case PAL_STREAM_ULTRASOUND:
                stream = new StreamUltraSound(sAttr, mPalDevice, count, modifiers,
                                            noOfModifiers, rm);
                break;
            default:
                PAL_ERR(LOG_TAG, "unsupported stream type 0x%x", sAttr->type);
                break;
        }
    } else {
        PAL_ERR(LOG_TAG,"Requested config not supported");
        goto exit;
    }
exit:
    if (stream) {
        PAL_DBG(LOG_TAG, "Exit. stream creation success");
    } else {
        if (mPalDevice)
            delete mPalDevice;
        PAL_ERR(LOG_TAG, "stream creation failed");
    }

    PAL_DBG(LOG_TAG, "stream %pK created", stream);
    return stream;
}

int32_t  Stream::getStreamAttributes(struct pal_stream_attributes *sAttr)
{
    int32_t status = 0;

    if (!sAttr) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid stream attribute pointer, status %d", status);
        goto exit;
    }
    ar_mem_cpy(sAttr, sizeof(pal_stream_attributes), mStreamAttr,
                     sizeof(pal_stream_attributes));
    PAL_DBG(LOG_TAG, "stream_type %d stream_flags %d direction %d",
            sAttr->type, sAttr->flags, sAttr->direction);

exit:
    return status;
}

const KeyVect_t& Stream::getDevPpModifiers() const {
    return mDevPpModifiers;
}

const KeyVect_t& Stream::getStreamModifiers() const {
    return mStreamModifiers;
}
int32_t  Stream::getModifiers(struct modifier_kv *modifiers,uint32_t *noOfModifiers)
{
    int32_t status = 0;

    if (!mModifiers || !noOfModifiers) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid modifers pointer, status %d", status);
        goto exit;
    }
    ar_mem_cpy (modifiers, sizeof(modifier_kv), mModifiers,
                      sizeof(modifier_kv));
    *noOfModifiers = mNoOfModifiers;
    PAL_DBG(LOG_TAG, "noOfModifiers %u", *noOfModifiers);
exit:
    return status;
}

int32_t Stream::getEffectParameters(void *effect_query)
{
    int32_t status = 0;

    if (!effect_query) {
        PAL_ERR(LOG_TAG, "invalid query");
        return -EINVAL;
    }

    pal_param_payload *pal_param = (pal_param_payload *)effect_query;
    effect_pal_payload_t *effectPayload = (effect_pal_payload_t *)pal_param->payload;
    status = session->getEffectParameters(this, effectPayload);
    if (status) {
       PAL_ERR(LOG_TAG, "getParameters failed with %d", status);
    }

    return status;
}

int32_t Stream::rwACDBParameters(void *payload, uint32_t sampleRate,
                                    bool isParamWrite)
{
    return session->rwACDBParameters(payload, sampleRate, isParamWrite);
}

int32_t Stream::getStreamType (pal_stream_type_t* streamType)
{
    int32_t status = 0;

    if (!streamType || !mStreamAttr) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid stream type, status %d", status);
        goto exit;
    }
    *streamType = mStreamAttr->type;
    PAL_DBG(LOG_TAG, "streamType - %d", *streamType);

exit:
    return status;
}

int32_t Stream::getStreamDirection(pal_stream_direction_t *dir)
{
    int32_t status = 0;

    if (!dir || !mStreamAttr) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid inputs, status %d", status);
    } else {
        *dir = mStreamAttr->direction;
        PAL_DBG(LOG_TAG, "stream direction - %d", *dir);
    }
    return status;
}

uint32_t Stream::getRenderLatency()
{
    uint32_t delayMs = 0;

    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "invalid mStreamAttr");
        return delayMs;
    }

    switch (mStreamAttr->type) {
    case PAL_STREAM_DEEP_BUFFER:
        delayMs = PAL_DEEP_BUFFER_PLATFORM_DELAY / 1000;
        break;
    case PAL_STREAM_LOW_LATENCY:
        delayMs = PAL_LOW_LATENCY_PLATFORM_DELAY / 1000;
        break;
    case PAL_STREAM_COMPRESSED:
    case PAL_STREAM_PCM_OFFLOAD:
        delayMs = PAL_PCM_OFFLOAD_PLATFORM_DELAY / 1000;
        break;
    case PAL_STREAM_ULTRA_LOW_LATENCY:
        delayMs = PAL_ULL_PLATFORM_DELAY / 1000;
        break;
    default:
        break;
    }

    return delayMs;
}

uint32_t Stream::getLatency()
{
    uint32_t latencyMs = 0;

    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "invalid mStreamAttr");
        return latencyMs;
    }

    switch (mStreamAttr->type) {
    case PAL_STREAM_DEEP_BUFFER:
        latencyMs = PAL_DEEP_BUFFER_OUTPUT_PERIOD_DURATION *
            PAL_DEEP_BUFFER_PLAYBACK_PERIOD_COUNT;
        break;
    case PAL_STREAM_LOW_LATENCY:
        latencyMs = PAL_LOW_LATENCY_OUTPUT_PERIOD_DURATION *
            PAL_LOW_LATENCY_PLAYBACK_PERIOD_COUNT;
        break;
    case PAL_STREAM_COMPRESSED:
    case PAL_STREAM_PCM_OFFLOAD:
        latencyMs = PAL_PCM_OFFLOAD_OUTPUT_PERIOD_DURATION *
            PAL_PCM_OFFLOAD_PLAYBACK_PERIOD_COUNT;
        break;
    default:
        break;
    }

    latencyMs += getRenderLatency();
    return latencyMs;
}

int32_t Stream::getAssociatedDevices(std::vector <std::shared_ptr<Device>> &aDevices)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "no. of devices %zu", mDevices.size());
    for (int32_t i=0; i < mDevices.size(); i++) {
        aDevices.push_back(mDevices[i]);
    }

    return status;
}

int32_t Stream::getAssociatedSession(Session **s)
{
    int32_t status = 0;

    if (!s) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid session, status %d", status);
        goto exit;
    }
    *s = session;
    PAL_DBG(LOG_TAG, "session %pK", *s);
exit:
    return status;
}

int32_t Stream::getVolumeData(struct pal_volume_data *vData)
{
    int32_t status = 0;

    if (!vData) {
        status = -EINVAL;
        PAL_INFO(LOG_TAG, "Volume Data not set yet");
        goto exit;
    }

    if (mVolumeData != NULL) {
        ar_mem_cpy(vData, sizeof(uint32_t) +
                      (sizeof(struct pal_channel_vol_kv) * (mVolumeData->no_of_volpair)),
                      mVolumeData, sizeof(uint32_t) +
                      (sizeof(struct pal_channel_vol_kv) * (mVolumeData->no_of_volpair)));

        PAL_DBG(LOG_TAG, "num config %x", (mVolumeData->no_of_volpair));
        for(int32_t i=0; i < (mVolumeData->no_of_volpair); i++) {
            PAL_VERBOSE(LOG_TAG,"Volume payload mask:%x vol:%f",
                (mVolumeData->volume_pair[i].channel_mask), (mVolumeData->volume_pair[i].vol));
        }
    } else {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "volume has not been set, status %d", status);
    }
exit:
    return status;
}

int32_t Stream::setBufInfo(pal_buffer_config *in_buffer_cfg,
                           pal_buffer_config *out_buffer_cfg)
{
    int32_t status = 0;
    struct pal_stream_attributes sattr;
    int16_t nBlockAlignIn, nBlockAlignOut ;        // block size of data

    status = getStreamAttributes(&sattr);
    /*In case of extern mem mode PAL wont get any buffer size info from clients
      If clients still call this api then it could be to notify the metadata size*/
    if (sattr.flags & PAL_STREAM_FLAG_EXTERN_MEM) {
        if (out_buffer_cfg)
            outMaxMetadataSz = out_buffer_cfg->max_metadata_size;
        if (in_buffer_cfg)
            inMaxMetadataSz = in_buffer_cfg->max_metadata_size;
         /*in EXTERN_MEM mode set buf count and size to 0*/
         inBufCount = 0;
         inBufSize = 0;
         outBufCount = 0;
         outBufSize = 0;
    } else {
        if (in_buffer_cfg) {
            PAL_DBG(LOG_TAG, "In Buffer size %zu, In Buffer count %zu metadata sz %zu",
                    in_buffer_cfg->buf_size, in_buffer_cfg->buf_count, in_buffer_cfg->max_metadata_size);
            inBufCount = in_buffer_cfg->buf_count;
            inMaxMetadataSz = in_buffer_cfg->max_metadata_size;
        }
        if (out_buffer_cfg) {
            PAL_DBG(LOG_TAG, "Out Buffer size %zu and Out Buffer count %zu metaData sz %zu",
                    out_buffer_cfg->buf_size, out_buffer_cfg->buf_count, out_buffer_cfg->max_metadata_size);
            outBufCount = out_buffer_cfg->buf_count;
            outMaxMetadataSz = out_buffer_cfg->max_metadata_size;
        }

        if (sattr.direction == PAL_AUDIO_OUTPUT) {
            if(!out_buffer_cfg) {
               status = -EINVAL;
               PAL_ERR(LOG_TAG, "Invalid output buffer size status %d", status);
               goto exit;
            }
            outBufSize = out_buffer_cfg->buf_size;
            nBlockAlignOut = ((sattr.out_media_config.bit_width) / 8) *
                          (sattr.out_media_config.ch_info.channels);
            PAL_DBG(LOG_TAG, "no of buf %zu and send buf %zu", outBufCount, outBufSize);

            //If the read size is not a multiple of BlockAlign;
            //Make sure we read blockaligned bytes from the file.
            if ((outBufSize % nBlockAlignOut) != 0) {
                outBufSize = ((outBufSize / nBlockAlignOut) * nBlockAlignOut);
            }
            out_buffer_cfg->buf_size = outBufSize;

        } else if (sattr.direction == PAL_AUDIO_INPUT) {
            if (!in_buffer_cfg) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid input buffer size status %d", status);
                goto exit;
            }
            inBufSize = in_buffer_cfg->buf_size;
            //inBufSize = (sattr->in_media_config.bit_width) * (sattr->in_media_config.ch_info.channels) * 32;
            nBlockAlignIn = ((sattr.in_media_config.bit_width) / 8) *
                          (sattr.in_media_config.ch_info.channels);
            //If the read size is not a multiple of BlockAlign;
            //Make sure we read blockaligned bytes from the file.
            if ((inBufSize % nBlockAlignIn) != 0) {
                inBufSize = ((inBufSize / nBlockAlignIn) * nBlockAlignIn);
            }
            in_buffer_cfg->buf_size = inBufSize;
        } else {
            if (!in_buffer_cfg || !out_buffer_cfg) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid buffer size status %d", status);
                goto exit;
            }
            outBufSize = out_buffer_cfg->buf_size;
            nBlockAlignOut = ((sattr.out_media_config.bit_width) / 8) *
                          (sattr.out_media_config.ch_info.channels);
            PAL_DBG(LOG_TAG, "no of buf %zu and send buf %zu", outBufCount, outBufSize);

            //If the read size is not a multiple of BlockAlign;
            //Make sure we read blockaligned bytes from the file.
            if ((outBufSize % nBlockAlignOut) != 0) {
                outBufSize = ((outBufSize / nBlockAlignOut) * nBlockAlignOut);
            }
            out_buffer_cfg->buf_size = outBufSize;

            inBufSize = in_buffer_cfg->buf_size;
            nBlockAlignIn = ((sattr.in_media_config.bit_width) / 8) *
                          (sattr.in_media_config.ch_info.channels);
            //If the read size is not a multiple of BlockAlign;
            //Make sure we read blockaligned bytes from the file.
            if ((inBufSize % nBlockAlignIn) != 0) {
                inBufSize = ((inBufSize / nBlockAlignIn) * nBlockAlignIn);
            }
            in_buffer_cfg->buf_size = inBufSize;
        }
    }
exit:
    return status;
}

int32_t Stream::getMaxMetadataSz(size_t *in_max_metdata_sz, size_t *out_max_metadata_sz)
{
    int32_t status = 0;

    if (in_max_metdata_sz)
        *in_max_metdata_sz = inMaxMetadataSz;
    if (out_max_metadata_sz)
        *out_max_metadata_sz = outMaxMetadataSz;

    if (in_max_metdata_sz)
        PAL_DBG(LOG_TAG, "in max metadataSize %zu",
                *in_max_metdata_sz);

    if (out_max_metadata_sz)
        PAL_DBG(LOG_TAG, "in max metadataSize %zu",
                *out_max_metadata_sz);

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

    if (in_buf_size && in_buf_count)
        PAL_DBG(LOG_TAG, "In Buffer size %zu, In Buffer count %zu",
                *in_buf_size, *in_buf_count);

    if (out_buf_size && out_buf_count)
        PAL_DBG(LOG_TAG, "Out Buffer size %zu and Out Buffer count %zu",
                *out_buf_size, *out_buf_count);

    return status;
}

bool Stream::isStreamAudioOutFmtSupported(pal_audio_fmt_t format)
{
    switch (format) {
    case PAL_AUDIO_FMT_PCM_S8:
    case PAL_AUDIO_FMT_PCM_S16_LE:
    case PAL_AUDIO_FMT_PCM_S24_3LE:
    case PAL_AUDIO_FMT_PCM_S24_LE:
    case PAL_AUDIO_FMT_PCM_S32_LE:
    case PAL_AUDIO_FMT_MP3:
    case PAL_AUDIO_FMT_AAC:
    case PAL_AUDIO_FMT_AAC_ADTS:
    case PAL_AUDIO_FMT_AAC_ADIF:
    case PAL_AUDIO_FMT_AAC_LATM:
    case PAL_AUDIO_FMT_WMA_STD:
    case PAL_AUDIO_FMT_ALAC:
    case PAL_AUDIO_FMT_APE:
    case PAL_AUDIO_FMT_WMA_PRO:
    case PAL_AUDIO_FMT_FLAC:
    case PAL_AUDIO_FMT_VORBIS:
    case PAL_AUDIO_FMT_AMR_NB:
    case PAL_AUDIO_FMT_AMR_WB:
    case PAL_AUDIO_FMT_AMR_WB_PLUS:
    case PAL_AUDIO_FMT_EVRC:
    case PAL_AUDIO_FMT_G711:
    case PAL_AUDIO_FMT_QCELP:
        return true;
    default:
        return false;
    }
}

int32_t Stream::getTimestamp(struct pal_session_time *stime)
{
    int32_t status = 0;
    if (!stime) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid session time pointer, status %d", status);
        goto exit;
    }
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Sound card offline, status %d", status);
        goto exit;
    }
    status = session->getTimestamp(stime);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to get session timestamp status %d", status);
        if (errno == -ENETRESET &&
            rm->cardState != CARD_STATUS_OFFLINE) {
            PAL_ERR(LOG_TAG, "Sound card offline, informing RM");
            rm->ssrHandler(CARD_STATUS_OFFLINE);
            status = -EINVAL;
        }
    }
exit:
    return status;
}

int32_t Stream::disconnectStreamDevice(Stream* streamHandle, pal_device_id_t dev_id)
{
    int32_t status = -EINVAL;
    mStreamMutex.lock();
    status = disconnectStreamDevice_l(streamHandle, dev_id);
    mStreamMutex.unlock();

    return status;
}

int32_t Stream::disconnectStreamDevice_l(Stream* streamHandle, pal_device_id_t dev_id)
{
    int32_t status = -EINVAL;

    // Stream does not know if the same device is being used by other streams or not
    // So if any other streams are using the same device that has to be handled outside of stream
    // resouce manager ??

    for (int i = 0; i < mDevices.size(); i++) {
        if (dev_id == mDevices[i]->getSndDeviceId()) {
            PAL_DBG(LOG_TAG, "device %d name %s, going to stop",
                mDevices[i]->getSndDeviceId(), mDevices[i]->getPALDeviceName().c_str());

            status = session->disconnectSessionDevice(streamHandle, mStreamAttr->type, mDevices[i]);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "disconnectSessionDevice failed:%d", status);
                goto error_1;
            }

            status = mDevices[i]->stop();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device stop failed with status %d", status);
                goto error_1;
            }
            rm->deregisterDevice(mDevices[i], this);

            status = mDevices[i]->close();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                goto error_1;
            }
            mDevices.erase(mDevices.begin() + i);
            break;
        }
    }

error_1:
    return status;
}

int32_t Stream::connectStreamDevice(Stream* streamHandle, struct pal_device *dattr)
{
    int32_t status = -EINVAL;
    mStreamMutex.lock();
    status = connectStreamDevice_l(streamHandle, dattr);
    mStreamMutex.unlock();

    return status;
}

int32_t Stream::connectStreamDevice_l(Stream* streamHandle, struct pal_device *dattr)
{
    int32_t status = -EINVAL;
    std::shared_ptr<Device> dev = nullptr;


    if (!dattr) {
        PAL_ERR(LOG_TAG, "invalid params");
        status = -EINVAL;
        goto error_1;
    }

    dev = Device::getInstance(dattr, rm);
    if (!dev) {
        PAL_ERR(LOG_TAG, "Device creation failed");
        goto error_1;
    }

    /* Check if we need to check here or above if bt_Sco is on for sco usecase
     * Device::getInstance will not set device attributes if the device instance
     * created previously so set device config explictly.
     */
    dev->setDeviceAttributes(*dattr);

    PAL_DBG(LOG_TAG, "device %d name %s, going to start",
        dev->getSndDeviceId(), dev->getPALDeviceName().c_str());

    status = dev->open();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "device %d open failed with status %d",
            dev->getSndDeviceId(), status);
        goto error_1;
    }

    mDevices.push_back(dev);
    status = session->setupSessionDevice(streamHandle, mStreamAttr->type, dev);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "setupSessionDevice for %d failed with status %d",
                dev->getSndDeviceId(), status);
        mDevices.pop_back();
        dev->close();
        goto error_1;
    }

    status = dev->start();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "device %d name %s, start failed with status %d",
            dev->getSndDeviceId(), dev->getPALDeviceName().c_str(), status);
        goto error_2;
    }
    status = session->connectSessionDevice(streamHandle, mStreamAttr->type, dev);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "connectSessionDevice failed:%d", status);
        goto error_3;
    }
    rm->registerDevice(dev, this);
    goto error_1;
error_3:
    dev->stop();
error_2:
    mDevices.pop_back();
    rm->deregisterDevice(dev, this);
    dev->close();
error_1:
    return status;
}

/*
  legend:
  s1 - current stream
  s2 - existing stream
  d1, d11, d2 - PAL_DEVICE enums
  B1, B2 - backend strings

case 1
  a) s1:  d1(B1) ->  d2(B2)
     OR
  b) s1:  d1(B1) ->  d2(B1)

  resolution: tear down d1 and setup d2.

case 2
  s1:  d1(B1) ->  d1(B1) , d2(B2)

  resolution: check if d1 needs device switch by calling isDeviceSwitchRequired.
    If needed, tear down and setup d1 again. Also setup d2.

case 3:
  s1:  d1(B1), d2(B2) ->  d1(B1)

  resolution: check if d1 needs device switch by calling isDeviceSwitchRequired.
    If needed, tear down and setup d1 again. Also teardown d2.

case 4
  s2->dev d1(B1)
  s1: d2(B2) -> d1(B1)

  resolution: check if s2 is voice call stream, then use same device attribute
    for s1. Else check if isDeviceSwitchRequired() returns true for s2 d1.
    If yes, tear down and setup d1 again for s2. Also, setup d1 for s2.

case 5
  s2->dev d2(B1)
  s1: d11(B2) -> d1(B1)
  resolution: If s2 is voice call stream, then setup d2 for s1 instead of d1
    and do nothing for s2. Else, tear down d2 from s2 and setup d1 for both
    s1 and s2.

case 6
  s2->dev d2(B2)
  s1: d2(B2) -> d1(B2)
  resolution: same handling as case 5

case 7
  s2->dev d1(B1), d2(B2)
  s1: d1(B1) -> d2(B2)

  resolution: nothing to be done for s2+d1. For s2+d2, it is
    same as case 4.

*/
int32_t Stream::switchDevice(Stream* streamHandle, uint32_t numDev, struct pal_device *newDevices)
{
    int32_t status = 0;
    int32_t connectCount = 0, disconnectCount = 0;
    bool isNewDeviceA2dp = false;
    bool isCurDeviceA2dp = false;
    bool matchFound = false;
    uint32_t curDeviceSlots[PAL_DEVICE_IN_MAX], newDeviceSlots[PAL_DEVICE_IN_MAX];
    std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnect, sharedBEStreamDev;
    std::vector <std::tuple<Stream *, struct pal_device *>> StreamDevConnect;
    struct pal_device dAttr;

    mStreamMutex.lock();

    if ((numDev == 0) || (!newDevices)) {
        PAL_ERR(LOG_TAG, "invalid param for device switch");
        status = -EINVAL;
        goto done;
    }

    if (rm->cardState == CARD_STATUS_OFFLINE) {
        PAL_ERR(LOG_TAG, "Sound card offline");
        goto done;
    }

    for (int i = 0; i < mDevices.size(); i++) {
        pal_device_id_t curDevId = (pal_device_id_t)mDevices[i]->getSndDeviceId();
        uint32_t tmp = numDev;

        mDevices[i]->getDeviceAttributes(&dAttr);
        if (curDevId == PAL_DEVICE_OUT_BLUETOOTH_A2DP)
            isCurDeviceA2dp = true;

        /* If stream is currently running on same device, then check if
         * it needs device switch. If not needed, then do not add it to
         * streamDevConnect and streamDevDisconnect list. This handles case 2,3.
         */
        matchFound = false;
        for (int j = 0; j < tmp; j++) {
            if (curDevId == newDevices[j].id) {
                matchFound = true;
                if (rm->isDeviceSwitchRequired(&dAttr, &newDevices[j], mStreamAttr)) {
                    curDeviceSlots[disconnectCount] = i;
                    disconnectCount++;
                } else {
                    for (int k = j; k < (numDev - 1); k++)
                        newDevices[k] = newDevices[k+1];

                    numDev--;
                }
            }
        }
        /* If mDevice[i] does not match with any of new device,
           then update disconnect list */
        if (matchFound == false) {
            curDeviceSlots[disconnectCount] = i;
            disconnectCount++;
        }
    }

    for (int i = 0; i < numDev; i++) {
        struct pal_device_info devinfo = {};
        /*
         * When A2DP is disconnected the
         * music playback is paused and the policy manager sends routing=0
         * But the audioflinger continues to write data until standby time
         * (3sec). As BT is turned off, the write gets blocked.
         * Avoid this by routing audio to speaker until standby.
         */
        if ((newDevices[i].id == PAL_DEVICE_NONE) &&   /* This assumes that PAL_DEVICE_NONE comes as single device */
            (isCurDeviceA2dp == true) && !rm->isDeviceReady(PAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
            newDevices[i].id = PAL_DEVICE_OUT_SPEAKER;

            rm->getDeviceInfo(newDevices[i].id, mStreamAttr->type,
                              newDevices[i].custom_config.custom_key, &devinfo);
            if (devinfo.channels == 0 || devinfo.channels > devinfo.max_channels) {
                PAL_ERR(LOG_TAG, "Invalid num channels[%d], failed to create stream",
                        devinfo.channels);
                continue;
            }
            rm->getDeviceConfig(&newDevices[i], mStreamAttr, devinfo.channels);
        }

        if (newDevices[i].id == PAL_DEVICE_NONE)
            continue;

        if (!rm->isDeviceReady(newDevices[i].id)) {
            PAL_ERR(LOG_TAG, "Device %d is not ready\n", newDevices[i].id);
        } else {
            newDeviceSlots[connectCount] = i;
            connectCount++;

            if (newDevices[i].id == PAL_DEVICE_OUT_BLUETOOTH_A2DP)
                isNewDeviceA2dp = true;
        }
    }

    /*  No new device is ready */
    if ((numDev != 0) && (connectCount == 0)) {
        PAL_ERR(LOG_TAG, "No new device is ready to connect");
        status = -ENODEV;
        goto done;
    }

    if (a2dpMuted && !isNewDeviceA2dp) {
        mute_l(false);
        a2dpMuted = false;
        suspendedDevId = PAL_DEVICE_NONE;
    }

    PAL_INFO(LOG_TAG,"number of active devices %zu, new devices %d", mDevices.size(), connectCount);

    /* created stream device connect and disconnect list */
    streamDevDisconnect.clear();
    StreamDevConnect.clear();

    for (int i = 0; i < connectCount; i++) {
        std::vector <Stream *> activeStreams;
        std::shared_ptr<Device> dev = nullptr;

        sharedBEStreamDev.clear();
        // get active stream device pairs sharing the same backend with new devices.
        rm->getSharedBEActiveStreamDevs(sharedBEStreamDev, newDevices[newDeviceSlots[i]].id);

        /* This can result in below situation:
         * 1) No matching SharedBEStreamDev (handled in else part).
         *    e.g. - case 1a, 2.
         * 2) sharedBEStreamDev having same stream as current stream but different device id.
         *    e.g. case 1b.
         * 3) sharedBEStreamDev having different stream from current but same id as current id.
         *    e.g. - case 4,7.
         * 4) sharedBEStreamDev having different stream from current and different device id.
         *    e.g. case 5,6.
         * 5) sharedBEStreamDev having same stream as current stream and same device id.
         *    e.g. spkr -> spkr + hs. This will only come in list if speaker needs device switch.
         * This list excludes same stream and same id which does not need device switch as it
         * is removed above.
         */
        if (sharedBEStreamDev.size() > 0) {
            for (const auto &elem : sharedBEStreamDev) {
                struct pal_stream_attributes sAttr;
                Stream *sharedStream = std::get<0>(elem);
                struct pal_device curDevAttr;
                std::shared_ptr<Device> curDev = nullptr;

                curDevAttr.id = (pal_device_id_t)std::get<1>(elem);
                curDev = Device::getInstance(&curDevAttr, rm);
                if (!curDev) {
                    PAL_ERR(LOG_TAG, "Getting Device instance failed");
                    continue;
                }
                curDev->getDeviceAttributes(&curDevAttr);

                /* If stream is currently running on same device, then
                 * check if it needs device switch. If not needed, then do not
                 * add it to streamDevConnect and streamDevDisconnect list and
                 * continue for next streamDev.
                 */
                if (curDevAttr.id == newDevices[newDeviceSlots[i]].id) {
                    if (!rm->isDeviceSwitchRequired(&curDevAttr,
                                &newDevices[newDeviceSlots[i]], mStreamAttr)) {
                        PAL_DBG(LOG_TAG, "DS not required, updating new device attributes");
                        curDev->getDeviceAttributes(&newDevices[newDeviceSlots[i]]);
                        continue;
                    }
                }

                sharedStream->getStreamAttributes(&sAttr);
                /* Check here for stream handle too along with stream type.
                 * In case of voice call or VOIP switch from speaker to handset,
                 * spkr needs to be disconnected and hanset needs to be
                 * connected and not just update its attributes.
                 */
                if ((PAL_STREAM_VOICE_CALL == sAttr.type ||
                     PAL_STREAM_VOIP_RX == sAttr.type ||
                     PAL_STREAM_VOIP == sAttr.type) &&
                                 (sharedStream != streamHandle)) {
                    PAL_INFO(LOG_TAG, "Active voice or voip stream running on %d, Force switch",
                                      curDevAttr.id);
                    curDev->getDeviceAttributes(&newDevices[newDeviceSlots[i]]);
                } else {
                    streamDevDisconnect.push_back(elem);
                    StreamDevConnect.push_back({std::get<0>(elem), &newDevices[newDeviceSlots[i]]});
                }
            }
        }
        /* Add device associated with current stream to streamDevDisconnect/StreamDevConnect list */
        for (int j = 0; j < disconnectCount; j++) {
            // check to make sure device direction is the same
            if (rm->matchDevDir(mDevices[curDeviceSlots[j]]->getSndDeviceId(), newDevices[newDeviceSlots[i]].id))
                streamDevDisconnect.push_back({streamHandle, mDevices[curDeviceSlots[j]]->getSndDeviceId()});
        }
        StreamDevConnect.push_back({streamHandle, &newDevices[newDeviceSlots[i]]});
    }

    /* Handle scenario when there is only device to disconnect.
       e.g. case 3 : device switch from spkr+hs to spkr */
    if (connectCount == 0) {
        for (int j = 0; j < disconnectCount; j++) {
            if (rm->matchDevDir(mDevices[curDeviceSlots[j]]->getSndDeviceId(), newDevices[0].id))
                streamDevDisconnect.push_back({streamHandle, mDevices[curDeviceSlots[j]]->getSndDeviceId()});
        }
    }

    /* remove duplicates */
    std::sort(streamDevDisconnect.begin(), streamDevDisconnect.end());
    streamDevDisconnect.erase(
            std::unique(streamDevDisconnect.begin(), streamDevDisconnect.end()),
            streamDevDisconnect.end());
    std::sort(StreamDevConnect.begin(), StreamDevConnect.end());
    StreamDevConnect.erase(
            std::unique(StreamDevConnect.begin(), StreamDevConnect.end()),
            StreamDevConnect.end());

    PAL_DBG(LOG_TAG, "disconnectList size is %zu, connectList size is %zu",
            streamDevDisconnect.size(), StreamDevConnect.size());
    for (const auto &elem : streamDevDisconnect)
        PAL_DBG(LOG_TAG, "disconnectList: stream handler 0x%p, device id %d",
                std::get<0>(elem), std::get<1>(elem));
    for (const auto &elem : StreamDevConnect)
        PAL_DBG(LOG_TAG, "connectList: stream handler 0x%p, device id %d",
                std::get<0>(elem), std::get<1>(elem)->id);

    /* Check if there is device to disconnect or connect */
    if (!streamDevDisconnect.size() && !StreamDevConnect.size()) {
        PAL_INFO(LOG_TAG, "No device to switch, returning");
        goto done;
    }

    mStreamMutex.unlock();

    status = rm->streamDevSwitch(streamDevDisconnect, StreamDevConnect);
    if (status) {
        PAL_ERR(LOG_TAG, "Device switch failed");
    }

    return status;

done:
    mStreamMutex.unlock();
    return status;
}

bool Stream::checkStreamMatch(pal_device_id_t pal_device_id,
                                        pal_stream_type_t pal_stream_type)
{
    int status = 0;
    struct pal_device dAttr;
    bool match = false;

    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "stream attribute is null");
        return false;
    }

    if (pal_stream_type == mStreamAttr->type ||
            pal_stream_type == PAL_STREAM_GENERIC)
        match = true;
    else
        return false;

    //device
    for (int i = 0; i < mDevices.size();i++) {
       status = mDevices[i]->getDeviceAttributes(&dAttr);
       if (0 != status) {
          PAL_ERR(LOG_TAG,"getDeviceAttributes Failed \n");
          return false;
       }
       if (pal_device_id == dAttr.id || pal_device_id == PAL_DEVICE_NONE) {
           match = true;
           // as long as one device matches, it is enough.
           break;
       }
    }

    return match;
}

