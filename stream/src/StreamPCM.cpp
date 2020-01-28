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

#define LOG_TAG "StreamPCM"
#define RXDIR 0
#define TXDIR 1

#include "StreamPCM.h"
#include "Session.h"
#include "kvh2xml.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include <unistd.h>

StreamPCM::StreamPCM(const struct qal_stream_attributes *sattr, struct qal_device *dattr,
                    const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                    const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm)
{
    mStreamMutex.lock();
    session = NULL;
    std::shared_ptr<Device> dev = nullptr;
    mStreamAttr = (struct qal_stream_attributes *)nullptr;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;
    mDevices.clear();
    bool isDeviceConfigUpdated = false;

    QAL_DBG(LOG_TAG, "Enter");

    //TBD handle modifiers later
    mNoOfModifiers = 0; //no_of_modifiers;
    mModifiers = (struct modifier_kv *) (NULL);
    std::ignore = modifiers;
    std::ignore = no_of_modifiers;

    if (!sattr || !dattr) {
        QAL_ERR(LOG_TAG,"invalid arguments");
        mStreamMutex.unlock();
        throw std::runtime_error("invalid arguments");
    }

    mStreamAttr = (struct qal_stream_attributes *) calloc(1, sizeof(struct qal_stream_attributes));
    if (!mStreamAttr) {
        QAL_ERR(LOG_TAG, "%s: malloc for stream attributes failed %s", __func__, strerror(errno));
        mStreamMutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }

    casa_osal_memcpy(mStreamAttr, sizeof(qal_stream_attributes), sattr, sizeof(qal_stream_attributes));

    if ((sattr->direction == QAL_AUDIO_OUTPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT)) {
        struct qal_channel_info *ch_info = (struct qal_channel_info *) calloc(1, sizeof(struct qal_channel_info));
        if (!ch_info) {
            QAL_ERR(LOG_TAG,"malloc for ch_info failed");
            free(mStreamAttr);
            mStreamMutex.unlock();
            throw std::runtime_error("failed to malloc for ch_info output");
        }

        mStreamAttr->out_media_config.ch_info = ch_info;
        casa_osal_memcpy(mStreamAttr->out_media_config.ch_info, sizeof(qal_channel_info),
                        sattr->out_media_config.ch_info, sizeof(qal_channel_info));
    } else if ((sattr->direction == QAL_AUDIO_INPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT)) {
        struct qal_channel_info *ch_info = (struct qal_channel_info *) calloc(1, sizeof(struct qal_channel_info));
        if (!ch_info) {
            QAL_ERR(LOG_TAG,"malloc for ch_info failed");
            free(mStreamAttr);
            mStreamMutex.unlock();
            throw std::runtime_error("failed to malloc for ch_info output");
        }
        mStreamAttr->in_media_config.ch_info = ch_info;
        casa_osal_memcpy(mStreamAttr->in_media_config.ch_info, sizeof(qal_channel_info),
        sattr->in_media_config.ch_info, sizeof(qal_channel_info));
   }

    QAL_VERBOSE(LOG_TAG, "Create new Session");
    session = Session::makeSession(rm, sattr);
    if (!session) {
        QAL_ERR(LOG_TAG, "%s: session creation failed", __func__);
        if ((sattr->direction == QAL_AUDIO_OUTPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT))
            free(mStreamAttr->out_media_config.ch_info);
        if ((sattr->direction == QAL_AUDIO_INPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT))
            free(mStreamAttr->in_media_config.ch_info);
        free(mStreamAttr);
        mStreamMutex.unlock();
        throw std::runtime_error("failed to create session object");
    }

    QAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d", no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        //Check with RM if the configuration given can work or not
        //for e.g., if incoming stream needs 24 bit device thats also
        //being used by another stream, then the other stream should route

        dev = Device::getInstance((struct qal_device *)&dattr[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation failed");
            if ((sattr->direction == QAL_AUDIO_OUTPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT))
                free(mStreamAttr->out_media_config.ch_info);
            if ((sattr->direction == QAL_AUDIO_INPUT) || (sattr->direction == QAL_AUDIO_INPUT_OUTPUT))
                free(mStreamAttr->in_media_config.ch_info);

            free(mStreamAttr);

            //TBD::free session too
            mStreamMutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        isDeviceConfigUpdated = rm->updateDeviceConfig(dev, &dattr[i], sattr);

        if (isDeviceConfigUpdated)
            QAL_VERBOSE(LOG_TAG, "Device config updated");

        /* Create only update device attributes first time so update here using set*/
        /* this will have issues if same device is being currently used by different stream */
       // dev->setDeviceAttributes((struct qal_device)dattr[i]);
        mDevices.push_back(dev);
        //rm->registerDevice(dev);
        dev = nullptr;
    }


    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit.");
    return;
}

int32_t  StreamPCM::open()
{
    int32_t status = 0;

    mStreamMutex.lock();
    QAL_VERBOSE(LOG_TAG, "Enter. session handle - %pK device count - %d", session,
                mDevices.size());
    status = session->open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session open failed with status %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session open successful");

    for (int32_t i=0; i < mDevices.size(); i++) {
        status = mDevices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device open failed with status %d", status);
            goto exit;
        }
    }
    rm->registerStream(this);
    QAL_DBG(LOG_TAG, "Exit. streamLL opened");
exit:
    mStreamMutex.unlock();
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t  StreamPCM::close()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_ERR(LOG_TAG, "Enter. session handle - %pK device count - %d",
            session,
            mDevices.size());
    for (int32_t i=0; i < mDevices.size(); i++) {
        status = mDevices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "closed the devices successfully");

    rm->lockGraph();
    status = session->close(this);
    rm->unlockGraph();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session close failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. closed the session successfully");

exit:
    mStreamMutex.unlock();
    status = rm->deregisterStream(this);
    if (mStreamAttr) {
        if ((mStreamAttr->direction == QAL_AUDIO_OUTPUT) || (mStreamAttr->direction == QAL_AUDIO_INPUT_OUTPUT))
            free(mStreamAttr->out_media_config.ch_info);
        if ((mStreamAttr->direction == QAL_AUDIO_INPUT) || (mStreamAttr->direction == QAL_AUDIO_INPUT_OUTPUT))
            free(mStreamAttr->in_media_config.ch_info);
        free(mStreamAttr);
        mStreamAttr = (struct qal_stream_attributes *)NULL;
    }

    if(mVolumeData)  {
        free(mVolumeData);
        mVolumeData = (struct qal_volume_data *)NULL;
    }
    delete session;
    session = nullptr;
    QAL_ERR(LOG_TAG, "status - %d", status);
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamPCM::start()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d", session,
            mStreamAttr->direction);

    switch (mStreamAttr->direction) {
    case QAL_AUDIO_OUTPUT:
        rm->lockGraph();
        QAL_VERBOSE(LOG_TAG, "Inside QAL_AUDIO_OUTPUT device count - %d",
                        mDevices.size());
        for (int32_t i=0; i < mDevices.size(); i++) {
            mStreamMutex.unlock();
            status = mDevices[i]->start();
            mStreamMutex.lock();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device start is failed with status %d",
                        status);
                rm->unlockGraph();
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices started successfully");

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session prepare is failed with status %d",
                    status);
            rm->unlockGraph();
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session prepare successful");

        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session start is failed with status %d",
                    status);
            rm->unlockGraph();
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session start successful");
        rm->unlockGraph();
        break;

    case QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "%s: Inside QAL_AUDIO_INPUT device count - %d",
                    __func__, mDevices.size());

        for (int32_t i=0; i < mDevices.size(); i++) {
            status = mDevices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device start is failed with status %d",
                        status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices started successfully");

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx session prepare is failed with status %d",
                    status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session prepare successful");

        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx session start is failed with status %d",
                    status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session start successful");

        break;
    case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "Inside Loopback case device count - %d",
                    mDevices.size());
        // start output device
        for (int32_t i=0; i < mDevices.size(); i++)
        {
            int32_t dev_id = mDevices[i]->getSndDeviceId();
            if (dev_id <= QAL_DEVICE_OUT_MIN || dev_id >= QAL_DEVICE_OUT_MAX)
                continue;
            mStreamMutex.unlock();
            status = mDevices[i]->start();
            mStreamMutex.lock();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device start is failed with status %d",
                        status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "output devices started successfully");

        // start input device
        for (int32_t i=0; i < mDevices.size(); i++) {
            int32_t dev_id = mDevices[i]->getSndDeviceId();
            if (dev_id <= QAL_DEVICE_IN_MIN || dev_id >= QAL_DEVICE_IN_MAX)
                continue;
            status = mDevices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device start is failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "input devices started successfully");

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session prepare is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session prepare successful");

        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session start is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session start successful");

        break;
    default:
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Stream type is not supported, status %d", status);
        break;
   }

    for (int i = 0; i < mDevices.size(); i++) {
        rm->registerDevice(mDevices[i]);
    }
    QAL_DBG(LOG_TAG, "Exit.");
exit:
    mStreamMutex.unlock();
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamPCM::stop()
{
    int32_t status = 0;

    mStreamMutex.lock();

    QAL_ERR(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d",
                session, mStreamAttr->direction);
    switch (mStreamAttr->direction) {
    case QAL_AUDIO_OUTPUT:
        QAL_VERBOSE(LOG_TAG, "In QAL_AUDIO_OUTPUT case, device count - %d",
                    mDevices.size());

        status = session->stop(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session stop failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session stop successful");

        for (int32_t i=0; i < mDevices.size(); i++) {
            status = mDevices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device stop failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices stop successful");
        break;

    case QAL_AUDIO_INPUT:
        QAL_ERR(LOG_TAG, "In QAL_AUDIO_INPUT case, device count - %d",
                    mDevices.size());

        for (int32_t i=0; i < mDevices.size(); i++) {
            status = mDevices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device stop failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices stop successful");

        status = session->stop(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx session stop failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session stop successful");
        break;

    case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "In LOOPBACK case, device count - %d", mDevices.size());

        for (int32_t i=0; i < mDevices.size(); i++) {
            int32_t dev_id = mDevices[i]->getSndDeviceId();
            if (dev_id <= QAL_DEVICE_IN_MIN || dev_id >= QAL_DEVICE_IN_MAX)
                continue;
            status = mDevices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device stop is failed with status %d",
                        status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "TX devices stop successful");

        status = session->stop(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session stop is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session stop successful");

        for (int32_t i=0; i < mDevices.size(); i++) {
             int32_t dev_id = mDevices[i]->getSndDeviceId();
             if (dev_id <= QAL_DEVICE_OUT_MIN || dev_id >= QAL_DEVICE_OUT_MAX)
                 continue;
             status = mDevices[i]->stop();
             if (0 != status) {
                 QAL_ERR(LOG_TAG, "Rx device stop is failed with status %d",
                         status);
                 goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "RX devices stop successful");
        break;
    default:
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Stream type is not supported with status %d", status);
        break;
    }
    for (int i = 0; i < mDevices.size(); i++) {
        rm->deregisterDevice(mDevices[i]);
    }
    QAL_DBG(LOG_TAG, "Exit. status %d", status);

exit:
   mStreamMutex.unlock();
   return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamPCM::prepare()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    mStreamMutex.lock();
    status = session->prepare(this);
    if (0 != status)
        QAL_ERR(LOG_TAG, "session prepare failed with status = %d", status);
    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);

    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t  StreamPCM::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = -EINVAL;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    if (!sattr)
    {
        QAL_ERR(LOG_TAG, "%s: NULL stream attributes sent %d", __func__);
        goto exit;
    }
    memset(mStreamAttr, 0, sizeof(struct qal_stream_attributes));
    mStreamMutex.lock();
    casa_osal_memcpy (mStreamAttr, sizeof(struct qal_stream_attributes), sattr,
                      sizeof(struct qal_stream_attributes));
    mStreamMutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //TODO:gkv or ckv or tkv need to pass
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");

exit:
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t  StreamPCM::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    if (!volume || volume->no_of_volpair == 0) {
        QAL_ERR(LOG_TAG, "%s: Error no of vol pair is %d", __func__, (volume->no_of_volpair));
        status = -EINVAL;
        goto exit;
    }

    mVolumeData = (struct qal_volume_data *)calloc(1, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair))));
    if (!mVolumeData) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "mVolumeData malloc failed %s", strerror(errno));
        goto exit;
    }

    //mStreamMutex.lock();
    casa_osal_memcpy (mVolumeData, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) *
                      (volume->no_of_volpair))), volume, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) *
                      (volume->no_of_volpair))));
    //mStreamMutex.unlock();
    for(int32_t i=0; i < (mVolumeData->no_of_volpair); i++) {
        QAL_ERR(LOG_TAG, "Volume payload mask:%x vol:%f",
                      (mVolumeData->volume_pair[i].channel_mask), (mVolumeData->volume_pair[i].vol));
    }
    status = session->setConfig(this, CALIBRATION, TAG_STREAM_VOLUME);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for VOLUME_TAG failed with status %d",
                status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. Volume payload No.of vol pair:%d ch mask:%x gain:%f",
                      (volume->no_of_volpair), (volume->volume_pair->channel_mask),
                      (volume->volume_pair->vol));
exit:
    return status;
}

int32_t  StreamPCM::read(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    mStreamMutex.lock();
    status = session->read(this, SHMEM_ENDPOINT, buf, &size);
    mStreamMutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session read is failed with status %d", status);
        return -status;
    }
    QAL_DBG(LOG_TAG, "Exit. session read successful size - %d", size);

    return size;
}

int32_t  StreamPCM::write(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    mStreamMutex.lock();
    if (standBy) {
        /* calculate sleep time based on buf->size, sleep and return buf->size */
        uint32_t stream_size;
        uint32_t byte_width = mStreamAttr->out_media_config.bit_width / 8;;
        uint32_t srate = mStreamAttr->out_media_config.sample_rate;
        struct qal_channel_info *ch_info = mStreamAttr->out_media_config.ch_info;

        if(!ch_info) {
            QAL_ERR(LOG_TAG, "%s: channle info is null", __func__);
            mStreamMutex.unlock();
            return -EINVAL;
        }

        stream_size = byte_width * ch_info->channels;
        if ((stream_size == 0) || (srate == 0)) {
            QAL_ERR(LOG_TAG, "%s: stream_size= %d, srate = %d", __func__, stream_size, srate);
            mStreamMutex.unlock();
            return -EINVAL;
        }
        size = buf->size;
        usleep((uint64_t)size * 1000000 / stream_size / srate);
        QAL_DBG(LOG_TAG, "in standby, dropped buffer size - %d", size);
        mStreamMutex.unlock();
        return size;
    }
    status = session->write(this, SHMEM_ENDPOINT, buf, &size, 0);
    mStreamMutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session write is failed with status %d", status);
        return -status;
    }
    QAL_DBG(LOG_TAG, "Exit. session write successful size - %d", size);
    return size;
}

int32_t  StreamPCM::registerCallBack(qal_stream_callback /*cb*/, void */*cookie*/)
{
    return 0;
}

int32_t  StreamPCM::getCallBack(qal_stream_callback * /*cb*/)
{
    return 0;
}

int32_t StreamPCM::getParameters(uint32_t /*param_id*/, void ** /*payload*/)
{
    return 0;
}

int32_t  StreamPCM::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    bool fluence_flag;
    qal_param_payload *param_payload = NULL;
    effect_qal_payload_t *effectQalPayload = nullptr;

    if (!payload)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "wrong params");
        goto error;
    }

    QAL_DBG(LOG_TAG, "%s: start, set parameter %u, session handle - %p", __func__, param_id, session);

    mStreamMutex.lock();
    // Stream may not know about tags, so use setParameters instead of setConfig
    switch (param_id) {
        case QAL_PARAM_ID_FLUENCE_ON_OFF:
        {
            param_payload = (qal_param_payload *)payload;
            fluence_flag = param_payload->has_fluence;
            QAL_DBG(LOG_TAG,"fluence flag is %d",fluence_flag);
            uint32_t fluenceTag = fluence_flag ? FLUENCE_ON_TAG : FLUENCE_OFF_TAG;
            status = session->setConfig(this, MODULE, fluenceTag);
            if (status)
               QAL_ERR(LOG_TAG, "setConfig for fluence %s failed with %d",
                       (fluence_flag ? "ON" : "OFF"), status);
            break;
        }
        case QAL_PARAM_ID_UIEFFECT:
        {
            param_payload = (qal_param_payload *)payload;
            if (!param_payload->has_effect) {
                QAL_ERR(LOG_TAG, "This is not effect param");
                status = -EINVAL;
                goto error;
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
            QAL_ERR(LOG_TAG, "Unsupported param id %u", param_id);
            status = -EINVAL;
            break;
    }

    mStreamMutex.unlock();
    QAL_VERBOSE(LOG_TAG, "exit, session parameter %u set with status %d", param_id, status);
error:
    return status;
}

int32_t  StreamPCM::setMute( bool state)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK state %d", session, state);
    mStreamMutex.lock();
    if (state)
        status = session->setConfig(this, MODULE, MUTE_TAG, TXDIR);
    else
        status = session->setConfig(this, MODULE, UNMUTE_TAG, TXDIR);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for mute failed with status %d",
                status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t  StreamPCM::setPause()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    mStreamMutex.lock();
    status = session->setConfig(this, MODULE, PAUSE_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d",
                status);
        goto exit;
    }
    isPaused = true;
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t  StreamPCM::setResume()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    mStreamMutex.lock();
    status = session->setConfig(this, MODULE, RESUME_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d",
                status);
        goto exit;
    }
    isPaused = false;
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t StreamPCM::flush()
{
    int32_t status = 0;

    mStreamMutex.lock();
    if (isPaused == false) {
         QAL_ERR(LOG_TAG, "Error, flush called while stream is not Paused isPaused:%d", isPaused);
         goto exit;
    }

    if (mStreamAttr->type != QAL_STREAM_PCM_OFFLOAD) {
         QAL_VERBOSE(LOG_TAG, "flush called for non PCM OFFLOAD stream, ignore");
         goto exit;
    }

    status = session->flush();
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t StreamPCM::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch(sampleRate) {
        case SAMPLINGRATE_8K:
        case SAMPLINGRATE_16K:
        case SAMPLINGRATE_22K:
        case SAMPLINGRATE_32K:
        case SAMPLINGRATE_44K:
        case SAMPLINGRATE_48K:
        case SAMPLINGRATE_96K:
        case SAMPLINGRATE_192K:
        case SAMPLINGRATE_384K:
            break;
       default:
            rc = 0;
            QAL_VERBOSE(LOG_TAG, "sample rate received %d rc %d", sampleRate, rc);
            break;
    }
    return rc;
}

int32_t StreamPCM::isChannelSupported(uint32_t numChannels)
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

int32_t StreamPCM::isBitWidthSupported(uint32_t bitWidth)
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

int32_t StreamPCM::addRemoveEffect(qal_audio_effect_t effect, bool enable)
{
    int32_t status = 0;
    int32_t tag = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    mStreamMutex.lock();

    if (!enable) {
        tag = FLUENCE_OFF_TAG;
    } else {
        if (QAL_AUDIO_EFFECT_EC == effect) {
            tag = FLUENCE_EC_TAG;
        } else if (QAL_AUDIO_EFFECT_NS == effect) {
            tag = FLUENCE_NS_TAG;
        } else if (QAL_AUDIO_EFFECT_ECNS == effect) {
            tag = FLUENCE_ON_TAG;
        } else {
            QAL_ERR(LOG_TAG, "Invalid effect ID %d");
            status = -EINVAL;
            goto exit;
        }
    }
    status = session->setConfig(this, MODULE, tag);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for addRemoveEffect failed with status %d",
                status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mStreamMutex.unlock();
    return status;
}
