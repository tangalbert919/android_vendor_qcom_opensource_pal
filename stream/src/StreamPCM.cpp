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

    QAL_DBG(LOG_TAG, "Enter.");
    uNoOfModifiers = no_of_modifiers;
    attr = (struct qal_stream_attributes *) malloc(sizeof(struct qal_stream_attributes));
    if (!attr) {
        QAL_ERR(LOG_TAG, "malloc for stream attributes failed %s", strerror(errno));
        mutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }
    memcpy (attr, sattr, sizeof(qal_stream_attributes));

    QAL_VERBOSE(LOG_TAG, "Create new Session");
    #ifdef CONFIG_GSL
        session = new SessionGsl(rm);
    #else
        session = new SessionAlsapcm();
    #endif

    if (!session) {
        QAL_ERR(LOG_TAG, "session creation failed");
        free(attr);
        mutex.unlock();
        throw std::runtime_error("failed to create session object");
    }

    QAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d", no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        dev = Device::create(&dattr[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation is failed");
            free(attr);
            mutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        devices.push_back(dev);
        rm->registerDevice(dev);
        dev = nullptr;
    }
    mutex.unlock();
    rm->registerStream(this);
    QAL_DBG(LOG_TAG, "Exit.");
}

int32_t  StreamPCM::open()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG, "Enter. session handle - %p device count - %d", session, devices.size());
    status = session->open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session open failed with status %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session open successful");

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device open failed with status %d", status);
            goto exit;
        }
    }

    QAL_DBG(LOG_TAG, "Exit. streamLL opened");
exit:
    mutex.unlock();
    return status;
}

int32_t  StreamPCM::close()
{
    int32_t status = 0;
    mutex.lock();

    QAL_DBG(LOG_TAG, "Enter. session handle - %p device count - %d", session, devices.size());
    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->close();
        rm->deregisterDevice(devices[i]);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "closed the devices successfully");

    status = session->close(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session close failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. closed the session successfully");

exit:
    mutex.unlock();
    status = rm->deregisterStream(this);
    QAL_ERR(LOG_TAG, "status - %d", status);
    return status;
}


int32_t StreamPCM::start()
{
    int32_t status = 0;
    mutex.lock();

    QAL_DBG(LOG_TAG, "Enter. session handle - %p attr->direction - %d", session, attr->direction);

    switch (attr->direction) {
    case QAL_AUDIO_OUTPUT:
        QAL_VERBOSE(LOG_TAG, "Inside QAL_AUDIO_OUTPUT device count - %d", devices.size());
        for (int32_t i=0; i < devices.size(); i++) {
            status = devices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device start is failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices started successfully");

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session prepare is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session prepare successful");

        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session start is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session start successful");
        break;

    case QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "Inside QAL_AUDIO_OUTPUT device count - %d", devices.size());

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx session prepare is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session prepare successful");

        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx session start is failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session start successful");

        for (int32_t i=0; i < devices.size(); i++) {
            status = devices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device start is failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices started successfully");
        break;
    case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "Inside Loopback case device count - %d", devices.size());
        // start output device
        for (int32_t i=0; i < devices.size(); i++)
        {
            int32_t dev_id = devices[i]->getDeviceId();
            if (dev_id < QAL_DEVICE_OUT_EARPIECE || dev_id > QAL_DEVICE_OUT_PROXY)
                continue;
            status = devices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device start is failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "output devices started successfully");

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

        // start input device
        for (int32_t i=0; i < devices.size(); i++) {
            int32_t dev_id = devices[i]->getDeviceId();
            if (dev_id < QAL_DEVICE_IN_HANDSET_MIC || dev_id > QAL_DEVICE_IN_PROXY)
                continue;
            status = devices[i]->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device start is failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "output devices started successfully");
        break;
    default:
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Stream type is not supported, status %d", status);
        break;
   }
   QAL_DBG(LOG_TAG, "Exit.");
exit:
    mutex.unlock();
    return status;
}

int32_t StreamPCM::stop()
{
    int32_t status = 0;

    mutex.lock();

    QAL_VERBOSE(LOG_TAG, "Enter. session handle - %p attr->direction - %d", session, attr->direction);
    switch (attr->direction) {
    case QAL_AUDIO_OUTPUT:
        QAL_VERBOSE(LOG_TAG, "In QAL_AUDIO_OUTPUT case, device count - %d", devices.size());

        status = session->stop(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session stop failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session stop successful");

        for (int32_t i=0; i < devices.size(); i++) {
            status = devices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx device stop failed with status %d", status);
                goto exit;
            }
        }
        QAL_VERBOSE(LOG_TAG, "devices stop successful");
        break;

    case QAL_AUDIO_INPUT:
        QAL_VERBOSE(LOG_TAG, "In QAL_AUDIO_INPUT case, device count - %d", devices.size());

        for (int32_t i=0; i < devices.size(); i++) {
            status = devices[i]->stop();
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
        QAL_VERBOSE(LOG_TAG, "In LOOPBACK case, device count - %d", devices.size());

        for (int32_t i=0; i < devices.size(); i++) {
            int32_t dev_id = devices[i]->getDeviceId();
            if (dev_id < QAL_DEVICE_IN_HANDSET_MIC || dev_id > QAL_DEVICE_IN_PROXY)
                continue;
            status = devices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx device stop is failed with status %d", status);
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

        for (int32_t i=0; i < devices.size(); i++) {
             int32_t dev_id = devices[i]->getDeviceId();
             if (dev_id < QAL_DEVICE_OUT_EARPIECE || dev_id > QAL_DEVICE_OUT_PROXY)
                 continue;
             status = devices[i]->stop();
             if (0 != status) {
                 QAL_ERR(LOG_TAG, "Rx device stop is failed with status %d", status);
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
    QAL_DBG(LOG_TAG, "Exit. status %d", status);

exit:
   mutex.unlock();
   return status;
}

int32_t StreamPCM::prepare()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);

    mutex.lock();
    status = session->prepare(this);
    if (0 != status)
        QAL_ERR(LOG_TAG, "session prepare failed with status = %d", status);
    mutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);

    return status;
}

int32_t  StreamPCM::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);

    memset(attr, 0, sizeof(struct qal_stream_attributes));
    mutex.lock();
    memcpy (attr, sattr, sizeof(struct qal_stream_attributes));
    mutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //TODO:gkv or ckv or tkv need to pass
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");

exit:
    return status;
}

int32_t  StreamPCM::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);
    if (volume->no_of_volpair == 0) {
        QAL_ERR(LOG_TAG, "Error no of vol pair is %d", (volume->no_of_volpair));
        status = -EINVAL;
        goto exit;
    }
    vdata = (struct qal_volume_data *)malloc(sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair)));
    if (!vdata) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "vdata malloc failed %s", strerror(errno));
        goto exit;
    }
    memset(vdata, 0, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair)));
    mutex.lock();
    memcpy (vdata, volume, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair))));
    mutex.unlock();
    for(int32_t i=0; i < (vdata->no_of_volpair); i++) {
    QAL_ERR(LOG_TAG, "Volume payload mask:%x vol:%f",
                  (vdata->volume_pair[i].channel_mask), (vdata->volume_pair[i].vol));
    }
    status = session->setConfig(this, CALIBRATION, TAG_STREAM_VOLUME);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for VOLUME_TAG failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. Volume payload No.of vol pair:%d ch mask:%x gain:%f",
                      (volume->no_of_volpair), (volume->volume_pair->channel_mask),(volume->volume_pair->vol));
exit:
    return status;
}

int32_t  StreamPCM::read(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;
    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);

    mutex.lock();
    status = session->read(this, SHMEM_ENDPOINT, buf, &size);
    mutex.unlock();
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

    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);

    mutex.lock();
    status = session->write(this, SHMEM_ENDPOINT, buf, &size, 0);
    mutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session write is failed with status %d", status);
        return -status;
    }
    QAL_DBG(LOG_TAG, "Exit. session write successful size - %d", size);
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
    QAL_DBG(LOG_TAG, "Enter. session handle - %p state %d", session, state);
    mutex.lock();
    switch (state) {
    case TRUE:
       status = session->setConfig(this, MODULE, MUTE_TAG);
       break;
    case FALSE:
       status = session->setConfig(this, MODULE, UNMUTE_TAG);
       break;
    }
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for mute failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mutex.unlock();
    return status;
}

int32_t  StreamPCM::setPause()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);
    mutex.lock();
    status = session->setConfig(this, MODULE, PAUSE_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mutex.unlock();
    return status;
}

int32_t  StreamPCM::setResume()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. session handle - %p", session);
    mutex.lock();
    status = session->setConfig(this, MODULE, RESUME_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mutex.unlock();
    return status;
}

int32_t StreamPCM::isSampleRateSupported(uint32_t sampleRate) {
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch(sampleRate) {
    case SAMPLINGRATE_8K:
    case SAMPLINGRATE_16K:
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

int32_t StreamPCM::isChannelSupported(uint32_t numChannels) {
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch(numChannels) {
    case CHANNEL1:
    case CHANNEL2:
    case CHANNEL3:
    case CHANNEL4:
    case CHANNEL5:
    case CHANNEL6:
    case CHANNEL7:
    case CHANNEL8:
        break;
    default:
        rc = -EINVAL;
        QAL_ERR(LOG_TAG, "channels not supported %d rc %d", numChannels, rc);
        break;
    }
    return rc;
}

int32_t StreamPCM::isBitWidthSupported(uint32_t bitWidth) {
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
