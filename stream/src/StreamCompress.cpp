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
#include "SessionAlsaCompress.h"
#include "ResourceManager.h"
#include "Device.h"

#define COMPRESS_OFFLOAD_FRAGMENT_SIZE (32 * 1024)
#define COMPRESS_OFFLOAD_NUM_FRAGMENTS 4

static void handleSessionCallBack(void *hdl, uint32_t event_id, void *data)
{
    Stream *s = NULL;
    qal_stream_callback cb;
    s = static_cast<Stream *>(hdl);
    if (s->getCallBack(&cb) == 0)
       cb(static_cast<qal_stream_handle_t *>(s), event_id, (uint32_t *)data, s->cookie);
}

StreamCompress::StreamCompress(const struct qal_stream_attributes *sattr, struct qal_device *dattr,
                               const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                               const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm)
{
    mStreamMutex.lock();
    std::shared_ptr<Device> dev = nullptr;
    bool isDeviceConfigUpdated = false;
    struct qal_channel_info * ch_info = NULL;

    session = nullptr;
    inBufSize = COMPRESS_OFFLOAD_FRAGMENT_SIZE;
    outBufSize = COMPRESS_OFFLOAD_FRAGMENT_SIZE;
    inBufCount = COMPRESS_OFFLOAD_NUM_FRAGMENTS;
    outBufCount = COMPRESS_OFFLOAD_NUM_FRAGMENTS;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);

    //TBD handle modifiers later
    mNoOfModifiers = 0; //no_of_modifiers;
    mModifiers = (struct modifier_kv *) (NULL);
    std::ignore = modifiers;
    std::ignore = no_of_modifiers;

    mStreamAttr = (struct qal_stream_attributes *) calloc(1, sizeof(struct qal_stream_attributes));
    if (!mStreamAttr) {
        QAL_ERR(LOG_TAG,"malloc for stream attributes failed");
        mStreamMutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }
    ch_info = (struct qal_channel_info *) calloc(1, sizeof(struct qal_channel_info));
    if (!ch_info) {
          QAL_ERR(LOG_TAG,"malloc for ch_info failed");
          free(mStreamAttr);
          mStreamMutex.unlock();
          throw std::runtime_error("failed to malloc for ch_info");
       }

    casa_osal_memcpy(mStreamAttr, sizeof(qal_stream_attributes), sattr, sizeof(qal_stream_attributes));
    mStreamAttr->out_media_config.ch_info = ch_info;
    casa_osal_memcpy(mStreamAttr->out_media_config.ch_info, sizeof(qal_channel_info),
    sattr->out_media_config.ch_info, sizeof(qal_channel_info));
    QAL_VERBOSE(LOG_TAG,"Create new compress session");

    session = Session::makeSession(rm, sattr);
    if (session == NULL){
       QAL_ERR(LOG_TAG,"session (compress) creation failed");
       free(mStreamAttr->out_media_config.ch_info);
       free(mStreamAttr);
       mStreamMutex.unlock();
       throw std::runtime_error("failed to create session object");
    }
    isDeviceConfigUpdated = rm->updateDeviceConfigs(mStreamAttr, no_of_devices,
        dattr);

    if (isDeviceConfigUpdated)
        QAL_VERBOSE(LOG_TAG, "Device config updated");

    session->registerCallBack(handleSessionCallBack, (void *)this);
    QAL_VERBOSE(LOG_TAG,"Create new Devices with no_of_devices - %d", no_of_devices);
    for (uint32_t i = 0; i < no_of_devices; i++) {
       dev = Device::getInstance((struct qal_device *)&dattr[i] , rm);
       if (dev == nullptr) {
          QAL_ERR(LOG_TAG, "Device creation is failed");
          free(mStreamAttr->out_media_config.ch_info);
          free(mStreamAttr);
          mStreamMutex.unlock();
          throw std::runtime_error("failed to create device object");
        }
        mDevices.push_back(dev);
        rm->registerDevice(dev);
        dev = nullptr;
    }
    mStreamMutex.unlock();
    rm->registerStream(this);
    QAL_VERBOSE(LOG_TAG,"exit");
}

int32_t StreamCompress::open()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_VERBOSE(LOG_TAG,"start, session handle - %p device count - %d", session, mDevices.size());
    status = session->open(this);
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session open failed with status %d", status);
       goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session open successful");
    for (int32_t i=0; i < mDevices.size(); i++) {
        QAL_ERR(LOG_TAG, "device %d name %s, going to open",
            mDevices[i]->getSndDeviceId(), mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG,"device open failed with status %d", status);
            goto exit;
         }
    }
    QAL_VERBOSE(LOG_TAG,"device open successful");
    QAL_VERBOSE(LOG_TAG,"exit stream compress opened");
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t StreamCompress::close()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_VERBOSE(LOG_TAG,"start, session handle - %p mDevices count - %d", session, mDevices.size());
    for (int32_t i=0; i < mDevices.size(); i++) {
        QAL_ERR(LOG_TAG, "device %d name %s, going to close",
            mDevices[i]->getSndDeviceId(), mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->close();
        rm->deregisterDevice(mDevices[i]);
        QAL_ERR(LOG_TAG,"deregister\n");
        if (0 != status) {
            QAL_ERR(LOG_TAG,"device close failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG,"closed the devices successfully");
    status = session->close(this);
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session close failed with status %d",status);
       goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"end, closed the session successfully");
exit:
    mStreamMutex.unlock();
    status = rm->deregisterStream(this);
    
    
    if (mStreamAttr) {
        free(mStreamAttr->out_media_config.ch_info);
        free(mStreamAttr);
        mStreamAttr = (struct qal_stream_attributes *)NULL;
    }

    if(mVolumeData)  {
        free(mVolumeData);
        mVolumeData = (struct qal_volume_data *)NULL;
    }
    
    delete session;
    session = nullptr;
    QAL_VERBOSE(LOG_TAG,"%d status - %d",__LINE__,status);
    return status;
}

int32_t StreamCompress::stop()
{
    int32_t status = 0;

    mStreamMutex.lock();
    QAL_VERBOSE(LOG_TAG,"Enter");
    QAL_VERBOSE(LOG_TAG,"stop session handle - %p mStreamAttr->direction - %d", session, mStreamAttr->direction);
    switch (mStreamAttr->direction) {
        case QAL_AUDIO_OUTPUT:
            QAL_VERBOSE(LOG_TAG,"In QAL_AUDIO_OUTPUT case, device count - %d", mDevices.size());
            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"Rx session stop failed with status %d",status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"session stop successful");
            for (int32_t i = 0; i < mDevices.size(); i++) {

                QAL_ERR(LOG_TAG, "device %d name %s, going to stop",
                    mDevices[i]->getSndDeviceId(), mDevices[i]->getQALDeviceName().c_str());

                status = mDevices[i]->stop();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"Rx device stop failed with status %d",status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"devices stop successful");
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "invalid direction %d", mStreamAttr->direction);
            break;
    }
exit:
    mStreamMutex.unlock();
    QAL_VERBOSE(LOG_TAG,"Exit status - %d", status);
    return status;
}

int32_t StreamCompress::start()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_VERBOSE(LOG_TAG,"start, session handle - %p mStreamAttr->direction - %d", session, mStreamAttr->direction);
    switch (mStreamAttr->direction) {
        case QAL_AUDIO_OUTPUT:
            QAL_VERBOSE(LOG_TAG,"Inside QAL_AUDIO_OUTPUT device count - %d", mDevices.size());
            for (int32_t i=0; i < mDevices.size(); i++) {

                QAL_ERR(LOG_TAG, "device %d name %s, going to start",
                    mDevices[i]->getSndDeviceId(), mDevices[i]->getQALDeviceName().c_str());

                status = mDevices[i]->start();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"Rx device start failed with status %d", status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"devices started successfully");
            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"Rx session prepare is failed with status %d",status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"session prepare successful");
            status = session->start(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"Rx session start is failed with status %d",status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"session start successful");
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "direction %d not supported for compress streams", mStreamAttr->direction);
            break;
    }

exit:
    mStreamMutex.unlock();
    return status;
}

int32_t StreamCompress::prepare()
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG,"Enter, session handle - %p", session);
    mStreamMutex.lock();
    status = session->prepare(this);
    if (status)
       QAL_ERR(LOG_TAG,"session prepare failed with status = %d", status);

   mStreamMutex.lock();
   QAL_VERBOSE(LOG_TAG,"%s: Exit, status - %d", __func__, status);
   return status;
}

int32_t StreamCompress::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG,"start, session handle - %p", session);
    memset(mStreamAttr, 0, sizeof(struct qal_stream_attributes));
    mStreamMutex.lock();
    memcpy (mStreamAttr, sattr, sizeof(struct qal_stream_attributes));
    mStreamMutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //gkv or ckv or tkv need to pass
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session setConfig failed with status %d",status);
       goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"session setConfig successful");
    QAL_VERBOSE(LOG_TAG,"end");
exit:
    return status;
}

int32_t StreamCompress::read(struct qal_buffer * /*buf*/)
{
    return 0;
}

int32_t StreamCompress::write(struct qal_buffer *buf)
{
    int32_t status = 0;
    int32_t size;
    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);

    mStreamMutex.lock();
    status = session->write(this, SHMEM_ENDPOINT, buf, &size, 0);
    mStreamMutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session write failed with status %d", status);
        return -status;
    }
    QAL_DBG(LOG_TAG, "Exit. session write successful size - %d", size);
    return size;
}

int32_t StreamCompress::registerCallBack(qal_stream_callback cb, void *cookie)
{
    streamCb = cb;
    this->cookie = cookie;
    return 0;
}

int32_t StreamCompress::getCallBack(qal_stream_callback *cb)
{
    *cb = streamCb;
    return 0;
}

int32_t StreamCompress::getParameters(uint32_t /*param_id*/, void ** /*payload*/)
{
    return 0;
}

int32_t StreamCompress::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    qal_param_payload *param_payload = NULL;
    uint32_t isPayloadTKV = false;
    uint32_t payloadSize = 0;
    effect_qal_payload_t *effectQalPayload = nullptr;

    switch (param_id) {
        case QAL_PARAM_ID_UIEFFECT:
        {
            param_payload = (qal_param_payload *)payload;
            if (!param_payload->has_effect) {
                QAL_ERR(LOG_TAG, "This is not effect param");
                status = -EINVAL;
                goto exit;
            }
            effectQalPayload = (effect_qal_payload_t *)(param_payload->effect_payload);
            if (effectQalPayload->isTKV) {
                status = session->setTKV(this, MODULE, effectQalPayload);
            } else {
                status = session->setParameters(this, effectQalPayload->tag, param_id, payload);
            }
            if (status) {
               QAL_ERR(LOG_TAG, "setParameters %d failed with %d", param_id, status);
            }
            break;
        }
        default:
            status = session->setParameters(this, 0, param_id, payload);
            break;
    }

exit:
    QAL_VERBOSE(LOG_TAG, "end, session parameter %u set with status %d", param_id, status);
    return status;
}

int32_t  StreamCompress::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG, "start, session handle - %p", session);
    if (!volume|| volume->no_of_volpair == 0) {
       QAL_ERR(LOG_TAG,"%s: Invalid arguments", __func__);
       status = -EINVAL;
       goto exit;
    }

    mVolumeData = (struct qal_volume_data *)calloc(1, sizeof(uint32_t) +
             (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair)));

    mStreamMutex.lock();
    memcpy (mVolumeData, volume, (sizeof(uint32_t) +
             (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair))));
    mStreamMutex.unlock();
    for(int32_t i = 0; i < (mVolumeData->no_of_volpair); i++) {
       QAL_VERBOSE(LOG_TAG,"Volume payload mask:%x vol:%f\n",
               (mVolumeData->volume_pair[i].channel_mask), (mVolumeData->volume_pair[i].vol));
    }
    status = session->setConfig(this, CALIBRATION, TAG_STREAM_VOLUME);
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session setConfig for VOLUME_TAG failed with status %d",status);
       goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"Volume payload No.of vol pair:%d ch mask:%x gain:%f",
             (volume->no_of_volpair), (volume->volume_pair->channel_mask),(volume->volume_pair->vol));
exit:
    return status;
}

int32_t  StreamCompress::setMute( bool state)
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG,"start, session handle - %p", session);
    switch (state) {
        case TRUE:
            QAL_VERBOSE(LOG_TAG,"Mute");
            status = session->setConfig(this, MODULE, MUTE_TAG);
            break;
        case FALSE:
            QAL_VERBOSE(LOG_TAG,"Unmute");
            status = session->setConfig(this, MODULE, UNMUTE_TAG);
            break;
    }
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session setConfig for mute failed with status %d",status);
       goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"session setMute successful");
exit:
    return status;
}

int32_t  StreamCompress::setPause()
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG,"Pause, session handle - %p", session);

    status = session->setConfig(this, MODULE, PAUSE_TAG);
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session setConfig for pause failed with status %d",status);
       goto exit;
    }
    isPaused = true;

    QAL_VERBOSE(LOG_TAG,"%s: session setPause successful", __func__);

exit:
    return status;
}

int32_t  StreamCompress::setResume()
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG,"Resume, session handle - %p", session);

    status = session->setConfig(this, MODULE, RESUME_TAG);
    if (0 != status) {
       QAL_ERR(LOG_TAG,"session setConfig for pause failed with status %d",status);
       goto exit;
    }

    isPaused = false;
    QAL_VERBOSE(LOG_TAG,"session setResume successful");

exit:
    return status;
}

int32_t StreamCompress::drain(qal_drain_type_t type)
{
    return session->drain(type);
}

int32_t StreamCompress::flush()
{
    if (isPaused == false) {
         QAL_ERR(LOG_TAG, "Error, flush called while stream is not Paused isPaused:%d", isPaused);
         return -EINVAL;
    }

    return session->flush();
}

int32_t StreamCompress::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch(sampleRate) {
        case SAMPLINGRATE_8K:
        case SAMPLINGRATE_16K:
        case SAMPLINGRATE_22K:
        case SAMPLINGRATE_24K:
        case SAMPLINGRATE_32K:
        case SAMPLINGRATE_44K:
        case SAMPLINGRATE_48K:
        case SAMPLINGRATE_96K:
        case SAMPLINGRATE_192K:
        case SAMPLINGRATE_384K:
            break;
       default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "sample rate not supported %d rc %d", sampleRate, rc);
            break;
    }
    return rc;
}

int32_t StreamCompress::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch(numChannels) {
        case CHANNELS_1:
        case CHANNELS_2:
        case CHANNELS_3:
        case CHANNELS_4:
        case CHANNELS_5:
        case CHANNELS_5_1:
        case CHANNELS_7:
        case CHANNELS_8:
            break;
        default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "channels not supported %d rc %d", numChannels, rc);
            break;
    }
    return rc;
}

int32_t StreamCompress::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch(bitWidth) {
        case BITWIDTH_16:
        case BITWIDTH_24:
        case BITWIDTH_32:
            break;
        default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "bit width not supported %d rc %d", bitWidth, rc);
            break;
    }
    return rc;
}

int32_t StreamCompress::addRemoveEffect(qal_audio_effect_t /*effect*/, bool /*enable*/)
{
    QAL_ERR(LOG_TAG, " Function not supported");
    return -ENOSYS;
}
