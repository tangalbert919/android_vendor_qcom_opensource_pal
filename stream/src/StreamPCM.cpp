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

#define LOG_TAG "StreamPCM"

#include "StreamPCM.h"
#include "Session.h"
#include "kvh2xml.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include <unistd.h>

StreamPCM::StreamPCM(struct qal_stream_attributes *sattr, struct qal_device *dattr, uint32_t no_of_devices,
                   struct modifier_kv *modifiers, uint32_t no_of_modifiers,std::shared_ptr<ResourceManager> rm)
{
    mutex.lock();
    session = NULL;
    dev = nullptr;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;

    QAL_VERBOSE(LOG_TAG,"%s : Start", __func__);
    uNoOfModifiers = no_of_modifiers;
    attr = (struct qal_stream_attributes *) malloc(sizeof(struct qal_stream_attributes));
    if (!attr) {
        QAL_ERR(LOG_TAG,"%s: malloc for stream attributes failed", __func__);
        mutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }
    //memset (attr, 0, sizeof(qal_stream_attributes));
    memcpy (attr, sattr, sizeof(qal_stream_attributes));

    //modifiers_ = (struct modifier_kv *)malloc(sizeof(struct modifier_kv));
    //if(!modifiers_)
    //{
      //  QAL_ERR(LOG_TAG,"%s: Malloc for modifiers failed", __func__);
        //mutex.unlock();
        //throw std::runtime_error("failed to malloc for modifiers");
    //}
    //memset (modifiers_, 0, sizeof(modifier_kv));
    //memcpy (modifiers_, modifiers, sizeof(modifier_kv));

    QAL_VERBOSE(LOG_TAG,"%s: Create new Session", __func__);
    #ifdef CONFIG_GSL
        session = new SessionGsl(rm);
    #else
        session = new SessionAlsapcm();
    #endif

    if (NULL == session) {
        QAL_ERR(LOG_TAG,"%s: session creation failed", __func__);
        mutex.unlock();
        throw std::runtime_error("failed to create session object");
    }

    QAL_VERBOSE(LOG_TAG,"%s: Create new Devices with no_of_devices - %d", __func__, no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        dev = Device::create(&dattr[i] , rm);
        if (dev == nullptr) {
            QAL_ERR(LOG_TAG,"%s: Device creation is failed", __func__);
            mutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        devices.push_back(dev);
        dev = nullptr;
    }
    mutex.unlock();
    rm->registerStream(this);
    QAL_VERBOSE(LOG_TAG,"%s:end", __func__);
}

/*StreamPCM::StreamPCM() {

}*/

int32_t  StreamPCM::open()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p device count - %d", __func__, session, devices.size());
    status = session->open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session open failed with status %d", __func__, status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: session open successful", __func__);

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: device open failed with status %d", __func__, status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG,"%s: device open successful", __func__);

    QAL_VERBOSE(LOG_TAG,"%s:exit streamLL opened", __func__);
exit:
    mutex.unlock();
    return status;
}

int32_t  StreamPCM::close()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p device count - %d", __func__, session, devices.size());
    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: device close is failed with status %d",__func__,status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG,"%s: closed the devices successfully",__func__);

    status = session->close(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session close failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s:end, closed the session successfully", __func__);

exit:
    mutex.unlock();
    status = rm->deregisterStream(this);
    QAL_ERR(LOG_TAG,"%s:%d status - %d",__func__,__LINE__,status);
    return status;
}


int32_t StreamPCM::start()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p attr->direction - %d", __func__, session, attr->direction);

    switch (attr->direction) {
        case QAL_AUDIO_OUTPUT:

            QAL_VERBOSE(LOG_TAG,"%s: Inside QAL_AUDIO_OUTPUT device count - %d", __func__, devices.size());
            for (int32_t i=0; i < devices.size(); i++) {
                status = devices[i]->start();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Rx device start is failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: devices started successfully", __func__);

            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Rx session prepare is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session prepare successful", __func__);

            status = session->start(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Rx session start is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session start successful", __func__);
            break;

        case QAL_AUDIO_INPUT:

            QAL_VERBOSE(LOG_TAG,"%s: Inside QAL_AUDIO_OUTPUT device count - %d", __func__, devices.size());

            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Tx session prepare is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session prepare successful", __func__);

            status = session->start(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Tx session start is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session start successful", __func__);

            for (int32_t i=0; i < devices.size(); i++) {
                status = devices[i]->start();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Tx device start is failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: devices started successfully", __func__);
            break;
        case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:

            QAL_VERBOSE(LOG_TAG,"%s: Inside Loopback case device count - %d", __func__, devices.size());
            // start output device
            for (int32_t i=0; i < devices.size(); i++)
            {
                int32_t dev_id = devices[i]->getDeviceId();
                if (dev_id < QAL_DEVICE_OUT_EARPIECE || dev_id > QAL_DEVICE_OUT_PROXY)
                    continue;
                status = devices[i]->start();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Rx device start is failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: output devices started successfully", __func__);

            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: session prepare is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session prepare successful", __func__);

            status = session->start(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: session start is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session start successful", __func__);

            // start input device
            for (int32_t i=0; i < devices.size(); i++) {
                int32_t dev_id = devices[i]->getDeviceId();
                if (dev_id < QAL_DEVICE_IN_HANDSET_MIC || dev_id > QAL_DEVICE_IN_PROXY)
                    continue;
                status = devices[i]->start();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Tx device start is failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: output devices started successfully", __func__);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG,"%s: Stream type is not supported, status %d",__func__,status);
            break;
   }
exit:
    mutex.unlock();
    return status;
}

int32_t StreamPCM::stop()
{
    int32_t status = 0;

    mutex.lock();

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p attr->direction - %d", __func__, session, attr->direction);
    switch (attr->direction) {
        case QAL_AUDIO_OUTPUT:
            QAL_VERBOSE(LOG_TAG,"%s: In QAL_AUDIO_OUTPUT case, device count - %d", __func__, devices.size());

            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Rx session stop failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session stop successful", __func__);

            for (int32_t i=0; i < devices.size(); i++) {
                status = devices[i]->stop();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Rx device stop failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: devices stop successful", __func__);
            break;

        case QAL_AUDIO_INPUT:
            QAL_VERBOSE(LOG_TAG,"%s: In QAL_AUDIO_INPUT case, device count - %d", __func__, devices.size());

            for (int32_t i=0; i < devices.size(); i++) {
                status = devices[i]->stop();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Tx device stop failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: devices stop successful", __func__);

            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Tx session stop failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session stop successful", __func__);
            break;

        case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:
            QAL_VERBOSE(LOG_TAG,"%s: In LOOPBACK case, device count - %d", __func__, devices.size());

            for (int32_t i=0; i < devices.size(); i++) {
                int32_t dev_id = devices[i]->getDeviceId();
                if (dev_id < QAL_DEVICE_IN_HANDSET_MIC || dev_id > QAL_DEVICE_IN_PROXY)
                    continue;
                status = devices[i]->stop();
                if (0 != status) {
                    QAL_ERR(LOG_TAG,"%s: Tx device stop is failed with status %d",__func__,status);
                    goto exit;
                }
            }
            QAL_VERBOSE(LOG_TAG,"%s: TX devices stop successful", __func__);

            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: session stop is failed with status %d",__func__,status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG,"%s: session stop successful", __func__);

            for (int32_t i=0; i < devices.size(); i++) {
                 int32_t dev_id = devices[i]->getDeviceId();
                 if (dev_id < QAL_DEVICE_OUT_EARPIECE || dev_id > QAL_DEVICE_OUT_PROXY)
                     continue;
                 status = devices[i]->stop();
                 if (0 != status) {
                     QAL_ERR(LOG_TAG,"%s: Rx device stop is failed with status %d",__func__,status);
                     goto exit;
                 }
            }
            QAL_VERBOSE(LOG_TAG,"%s: RX devices stop successful", __func__);

            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG,"%s: Stream type is not supported with status %d",__func__,status);
            break;
    }
    QAL_VERBOSE(LOG_TAG,"%s: end", __func__);

exit:
   mutex.unlock();
   return status;
}

int32_t StreamPCM::prepare()
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);

    mutex.lock();
    status = session->prepare(this);
    if (status)
        QAL_ERR(LOG_TAG,"%s: session prepare failed with status = %d", __func__, status);
    mutex.unlock();
    QAL_VERBOSE(LOG_TAG,"%s: end, status - %d", __func__, status);

    return status;
}

int32_t  StreamPCM::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);

    memset(attr, 0, sizeof(struct qal_stream_attributes));
    mutex.lock();
    memcpy (attr, sattr, sizeof(struct qal_stream_attributes));
    mutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //gkv or ckv or tkv need to pass
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session setConfig failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: session setConfig successful", __func__);

    QAL_VERBOSE(LOG_TAG,"%s: end", __func__);
exit:
    return status;
}

int32_t  StreamPCM::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;
    QAL_ERR(LOG_TAG,"%s: start, session handle - %p", __func__, session);
    if (volume->no_of_volpair == 0) {
        QAL_ERR(LOG_TAG,"%s: Error no of vol pair is %d",__func__,(volume->no_of_volpair));
        status = -EINVAL;
        goto exit;
    }
    vdata = (struct qal_volume_data *)malloc(sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair)));
    memset(vdata, 0, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair)));
    mutex.lock();
    memcpy (vdata, volume, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair))));
    mutex.unlock();
    for(int32_t i=0; i < (vdata->no_of_volpair); i++) {
    QAL_ERR(LOG_TAG,"%s: Volume payload mask:%x vol:%f\n",
                  __func__, (vdata->volume_pair[i].channel_mask), (vdata->volume_pair[i].vol));
    }
    status = session->setConfig(this, CALIBRATION, TAG_STREAM_VOLUME);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session setConfig for VOLUME_TAG failed with status %d",__func__,status);
        goto exit;
    }
    QAL_ERR(LOG_TAG,"%s: Volume payload No.of vol pair:%d ch mask:%x gain:%f",
                      __func__, (volume->no_of_volpair), (volume->volume_pair->channel_mask),(volume->volume_pair->vol));
exit:
    return status;
}

int32_t  StreamPCM::read(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;
    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);

    mutex.lock();
    status = session->read(this, SHMEM_ENDPOINT, buf, &size);
    mutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session read is failed with status %d",__func__,status);
        return -status;
    }
    QAL_VERBOSE(LOG_TAG,"%s: end, session read successful size - %d", __func__, size);

    return size;
}

int32_t  StreamPCM::write(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;
    /*if (flag == EOS) {
        uflag = EOS;
    }
    status = session->write(this, 0, buf, size, uflag);*/

    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);

    mutex.lock();
    status = session->write(this, SHMEM_ENDPOINT, buf, &size, 0);
    mutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session write is failed with status %d",__func__,status);
        return -status;
    }
    QAL_VERBOSE(LOG_TAG,"%s: end, session write successful size - %d", __func__, size);
    return size;
}

int32_t  StreamPCM::registerCallBack(qal_stream_callback cb)
{
    return 0;
}

int32_t  StreamPCM::getCallBack(qal_stream_callback *cb)
{
    return 0;
}

int32_t  StreamPCM::setParameters(uint32_t param_id, void *payload)
{
    return 0;
}

int32_t  StreamPCM::setMute( bool state)
{
    int32_t status = 0;
    QAL_ERR(LOG_TAG,"%s: start, session handle - %p", __func__, session);
    switch (state) {
    case TRUE:
       QAL_ERR(LOG_TAG,"%s: Mute", __func__);
       status = session->setConfig(this, MODULE, MUTE_TAG);
       break;
    case FALSE:
       QAL_ERR(LOG_TAG,"%s: Unmute", __func__);
       status = session->setConfig(this, MODULE, UNMUTE_TAG);
       break;
    }
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session setConfig for mute failed with status %d",__func__,status);
        goto exit;
    }
    QAL_ERR(LOG_TAG,"%s: session setConfig successful", __func__);
    QAL_ERR(LOG_TAG,"%s: end", __func__);
exit:
    return status;
}

int32_t  StreamPCM::setPause()
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);
    status = session->setConfig(this, MODULE, PAUSE_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session setConfig for pause failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: session setConfig successful", __func__);
    QAL_VERBOSE(LOG_TAG,"%s: end", __func__);
exit:
    return status;
}

int32_t  StreamPCM::setResume()
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: start, session handle - %p", __func__, session);
    status = session->setConfig(this, MODULE, RESUME_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: session setConfig for pause failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: session setConfig successful", __func__);
    QAL_VERBOSE(LOG_TAG,"%s: end", __func__);
exit:
    return status;
}

int32_t StreamPCM::isSampleRateSupported(uint32_t sampleRate) {
    int32_t rc = 0;
    QAL_ERR(LOG_TAG,"%s:%d",__func__,__LINE__);
    switch(sampleRate) {
        case 8000:
        case 16000:
        case 32000:
        case 44100:
        case 48000:
        case 96000:
        case 192000:
        case 384000:
            break;
        default:
            QAL_ERR(LOG_TAG,"sample rate not supported");
            rc = -EINVAL;
            break;
    }
    return rc;
}

int32_t StreamPCM::isChannelSupported(uint32_t numChannels) {
    int32_t rc = 0;
    QAL_ERR(LOG_TAG,"%s:%d",__func__,__LINE__);
    switch(numChannels) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            break;
        default:
            QAL_ERR(LOG_TAG,"channels not supported");
            rc = -EINVAL;
            break;
    }
    return rc;
}

int32_t StreamPCM::isBitWidthSupported(uint32_t bitWidth) {
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

