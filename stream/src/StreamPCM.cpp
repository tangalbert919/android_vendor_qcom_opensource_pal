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

#define LOG_TAG "QAL: StreamPCM"
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
    uint32_t in_channels, out_channels = 0;
    uint32_t attribute_size = 0;

    if (rm->cardState == CARD_STATUS_OFFLINE) {
        QAL_ERR(LOG_TAG, "Sound card offline, can not create stream");
        usleep(SSR_RECOVERY);
        mStreamMutex.unlock();
        throw std::runtime_error("Sound card offline");
    }

    session = NULL;
    std::shared_ptr<Device> dev = nullptr;
    mStreamAttr = (struct qal_stream_attributes *)nullptr;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;
    mDevices.clear();
    currentState = STREAM_IDLE;
    //Modify cached values only at time of SSR down.
    cachedState = STREAM_IDLE;
    cachedNumDev = 0;
    cachedDev = NULL;
    bool isDeviceConfigUpdated = false;

    QAL_DBG(LOG_TAG, "Enter");

    //TBD handle modifiers later
    mNoOfModifiers = 0; //no_of_modifiers;
    mModifiers = (struct modifier_kv *) (NULL);
    std::ignore = modifiers;
    std::ignore = no_of_modifiers;

    // Setting default volume to unity
    mVolumeData = (struct qal_volume_data *)malloc(sizeof(struct qal_volume_data)
                      +sizeof(struct qal_channel_vol_kv));
    mVolumeData->no_of_volpair = 1;
    mVolumeData->volume_pair[0].channel_mask = 0x03;
    mVolumeData->volume_pair[0].vol = 1.0f;

    if (!sattr || !dattr) {
        QAL_ERR(LOG_TAG,"invalid arguments");
        mStreamMutex.unlock();
        throw std::runtime_error("invalid arguments");
    }

    attribute_size = sizeof(struct qal_stream_attributes);
    mStreamAttr = (struct qal_stream_attributes *) calloc(1, attribute_size);
    if (!mStreamAttr) {
        QAL_ERR(LOG_TAG, "%s: malloc for stream attributes failed %s", __func__, strerror(errno));
        mStreamMutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }

    ar_mem_cpy(mStreamAttr, sizeof(qal_stream_attributes), sattr, sizeof(qal_stream_attributes));

    if (mStreamAttr->in_media_config.ch_info.channels > QAL_MAX_CHANNELS_SUPPORTED) {
        QAL_ERR(LOG_TAG,"in_channels is invalid %d", in_channels);
        mStreamAttr->in_media_config.ch_info.channels = QAL_MAX_CHANNELS_SUPPORTED;
    }
    if (mStreamAttr->out_media_config.ch_info.channels > QAL_MAX_CHANNELS_SUPPORTED) {
        QAL_ERR(LOG_TAG,"out_channels is invalid %d", out_channels);
        mStreamAttr->out_media_config.ch_info.channels = QAL_MAX_CHANNELS_SUPPORTED;
    }

    QAL_VERBOSE(LOG_TAG, "Create new Session");
    session = Session::makeSession(rm, sattr);
    if (!session) {
        QAL_ERR(LOG_TAG, "%s: session creation failed", __func__);
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

    rm->registerStream(this);

    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. state %d", currentState);
    return;
}

int32_t  StreamPCM::open()
{
    int32_t status = 0;

    mStreamMutex.lock();
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        QAL_ERR(LOG_TAG, "Sound card offline, can not open stream");
        usleep(SSR_RECOVERY);
        status = -EIO;
        goto exit;
    }

    if (currentState == STREAM_IDLE) {
        QAL_VERBOSE(LOG_TAG, "Enter. session handle - %pK device count - %d", session,
                mDevices.size());
        status = session->open(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session open failed with status %d", status);
            goto exit;
        }
        QAL_VERBOSE(LOG_TAG, "session open successful");

        for (int32_t i = 0; i < mDevices.size(); i++) {
            status = mDevices[i]->open();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "device open failed with status %d", status);
                goto exit;
            }
        }
        currentState = STREAM_INIT;
        QAL_DBG(LOG_TAG, "Exit. streamLL opened. state %d", currentState);
    } else if (currentState == STREAM_INIT) {
        QAL_INFO(LOG_TAG, "Stream is already opened, state %d", currentState);
        status = 0;
        goto exit;
    } else {
        QAL_ERR(LOG_TAG, "Stream is not in correct state %d", currentState);
        //TBD : which error code to return here.
        status = -EINVAL;
        goto exit;
    }
exit:
    mStreamMutex.unlock();
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t  StreamPCM::close()
{
    int32_t status = 0;
    mStreamMutex.lock();

    QAL_INFO(LOG_TAG, "Enter. session handle - %pK device count - %d state %d",
            session, mDevices.size(), currentState);

    if (currentState == STREAM_IDLE) {
        /* If current state is STREAM_IDLE, that means :
         * 1. SSR down has happened
         * Session is already closed as part of ssr handling, so just
         * close device and destroy the objects.
         * 2. Stream created but opened failed.
         * No need to call session close for this case too.
         */
        for (int32_t i=0; i < mDevices.size(); i++) {
            status = mDevices[i]->close();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
            }
        }
        QAL_VERBOSE(LOG_TAG, "closed the devices successfully");
        goto exit;
    } else if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        status = stop();
        if (0 != status)
            QAL_ERR(LOG_TAG, "stream stop failed. status %d",  status);
    }

    for (int32_t i=0; i < mDevices.size(); i++) {
        status = mDevices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
        }
    }
    QAL_VERBOSE(LOG_TAG, "closed the devices successfully");

    rm->lockGraph();
    status = session->close(this);
    rm->unlockGraph();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session close failed with status %d", status);
    }

exit:
    mStreamMutex.unlock();
    status = rm->deregisterStream(this);
    if (mStreamAttr) {
        free(mStreamAttr);
        mStreamAttr = (struct qal_stream_attributes *)NULL;
    }

    if(mVolumeData)  {
        free(mVolumeData);
        mVolumeData = (struct qal_volume_data *)NULL;
    }
    delete session;
    session = nullptr;
    currentState = STREAM_IDLE;
    QAL_INFO(LOG_TAG, "Exit. closed the stream successfully %d status %d",
             currentState, status);
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamPCM::start()
{
    int32_t status = 0;

    mStreamMutex.lock();
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        cachedState = STREAM_STARTED;
        QAL_ERR(LOG_TAG, "Sound card offline. Update the cached state %d",
                cachedState);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d state %d",
              session, mStreamAttr->direction, currentState);

    if (currentState == STREAM_INIT || currentState == STREAM_STOPPED) {
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
        /*pcm_open and pcm_start done at once here,
         *so directly jump to STREAM_STARTED state.
         */
        currentState = STREAM_STARTED;
    } else if (currentState == STREAM_STARTED) {
        QAL_INFO(LOG_TAG, "Stream already started, state %d", currentState);
        goto exit;
    } else {
        QAL_ERR(LOG_TAG, "Stream is not opened yet");
        status = -EINVAL;
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. state %d", currentState);
exit:
    mStreamMutex.unlock();
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamPCM::stop()
{
    int32_t status = 0;

    mStreamMutex.lock();
    QAL_ERR(LOG_TAG, "Enter. session handle - %pK mStreamAttr->direction - %d state %d",
                session, mStreamAttr->direction, currentState);

    if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        switch (mStreamAttr->direction) {
        case QAL_AUDIO_OUTPUT:
            QAL_VERBOSE(LOG_TAG, "In QAL_AUDIO_OUTPUT case, device count - %d",
                        mDevices.size());

            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx session stop failed with status %d", status);
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
                }
            }
            QAL_VERBOSE(LOG_TAG, "TX devices stop successful");
            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "session stop is failed with status %d", status);
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
        currentState = STREAM_STOPPED;
    } else if (currentState == STREAM_STOPPED || currentState == STREAM_IDLE) {
        QAL_INFO(LOG_TAG, "Stream is already in Stopped state %d", currentState);
        goto exit;
    } else {
        QAL_ERR(LOG_TAG, "Stream should be in start/pause state, %d", currentState);
        status = -EINVAL;
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. status %d, state %d", status, currentState);

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
        QAL_ERR(LOG_TAG, "%s: NULL stream attributes sent", __func__);
        goto exit;
    }
    memset(mStreamAttr, 0, sizeof(struct qal_stream_attributes));
    mStreamMutex.lock();
    ar_mem_cpy (mStreamAttr, sizeof(struct qal_stream_attributes), sattr,
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

    /*if already allocated free and reallocate */
    if (mVolumeData) {
        free(mVolumeData);
    }

    mVolumeData = (struct qal_volume_data *)calloc(1, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (volume->no_of_volpair))));
    if (!mVolumeData) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "mVolumeData malloc failed %s", strerror(errno));
        goto exit;
    }

    //mStreamMutex.lock();
    ar_mem_cpy (mVolumeData, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) *
                      (volume->no_of_volpair))), volume, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) *
                      (volume->no_of_volpair))));
    //mStreamMutex.unlock();
    for(int32_t i=0; i < (mVolumeData->no_of_volpair); i++) {
        QAL_INFO(LOG_TAG, "Volume payload mask:%x vol:%f",
                      (mVolumeData->volume_pair[i].channel_mask), (mVolumeData->volume_pair[i].vol));
    }
    /* Allow caching of stream volume as part of mVolumeData
     * till the pcm_open is not done or if sound card is
     * offline.
     */
    if (rm->cardState == CARD_STATUS_ONLINE && currentState != STREAM_IDLE
        && currentState != STREAM_INIT) {
        status = session->setConfig(this, CALIBRATION, TAG_STREAM_VOLUME);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session setConfig for VOLUME_TAG failed with status %d",
                    status);
            goto exit;
        }
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
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK, state %d",
            session, currentState);

    if ((rm->cardState == CARD_STATUS_OFFLINE) || cachedState != STREAM_IDLE) {
       /* calculate sleep time based on buf->size, sleep and return buf->size */
        uint32_t streamSize;
        uint32_t byteWidth = mStreamAttr->in_media_config.bit_width / 8;
        uint32_t sampleRate = mStreamAttr->in_media_config.sample_rate;
        struct qal_channel_info chInfo = mStreamAttr->in_media_config.ch_info;

        streamSize = byteWidth * chInfo.channels;
        if ((streamSize == 0) || (sampleRate == 0)) {
            QAL_ERR(LOG_TAG, "stream_size= %d, srate = %d",
                    streamSize, sampleRate);
            status =  -EINVAL;
            goto exit;
        }
        size = buf->size;
        memset(buf->buffer, 0, size);
        usleep((uint64_t)size * 1000000 / streamSize / sampleRate);
        QAL_DBG(LOG_TAG, "Sound card offline, dropped buffer size - %d", size);
        status = size;
        goto exit;
    }

    if (currentState == STREAM_STARTED) {
        status = session->read(this, SHMEM_ENDPOINT, buf, &size);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session read is failed with status %d", status);
            //TODO : Modify after GSL fix.
            if (status == -EPERM) {
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                if (rm->cardState != CARD_STATUS_OFFLINE) {
                    QAL_ERR(LOG_TAG, "Sound card offline");
                    rm->cardState = CARD_STATUS_OFFLINE;
                }
                goto exit;
            } else if (rm->cardState == CARD_STATUS_OFFLINE) {
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else {
                goto exit;
            }
        }
    } else {
        QAL_ERR(LOG_TAG, "Stream not started yet, state %d", currentState);
        status = -EINVAL;
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session read successful size - %d", size);
    return size;
exit :
    QAL_DBG(LOG_TAG, "session read failed status %d", status);
    return status;
}

int32_t StreamPCM::write(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size = 0;
    bool isA2dp = false;
    bool isSpkr = false;
    bool isA2dpSuspended = false;
    uint32_t frameSize = 0;
    uint32_t byteWidth = 0;
    uint32_t sampleRate = 0;
    uint32_t channelCount = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK, state %d",
            session, currentState);

    mStreamMutex.lock();
    for (int i = 0; i < mDevices.size(); i++) {
        if (mDevices[i]->getSndDeviceId() == QAL_DEVICE_OUT_BLUETOOTH_A2DP)
            isA2dp = true;
        if (mDevices[i]->getSndDeviceId() == QAL_DEVICE_OUT_SPEAKER)
            isSpkr = true;
    }

    if (isA2dp && !isSpkr) {
        qal_param_bta2dp_t *paramA2dp = NULL;
        size_t paramSize = 0;
        int ret = rm->getParameter(QAL_PARAM_ID_BT_A2DP_SUSPENDED,
                (void **)&paramA2dp,
                &paramSize,
                NULL);
        if (!ret && paramA2dp)
            isA2dpSuspended = paramA2dp->a2dp_suspended;
    }

    // If cached state is not STREAM_IDLE, we are still processing SSR up.
    if (isA2dpSuspended || (mDevices.size() == 0)
            || (rm->cardState == CARD_STATUS_OFFLINE)
            || cachedState != STREAM_IDLE) {
        byteWidth = mStreamAttr->out_media_config.bit_width / 8;
        sampleRate = mStreamAttr->out_media_config.sample_rate;
        channelCount = mStreamAttr->out_media_config.ch_info.channels;

        frameSize = byteWidth * channelCount;
        if ((frameSize == 0) || (sampleRate == 0)) {
            QAL_ERR(LOG_TAG, "frameSize=%d, sampleRate=%d", frameSize, sampleRate);
            mStreamMutex.unlock();
            return -EINVAL;
        }
        size = buf->size;
        usleep((uint64_t)size * 1000000 / frameSize / sampleRate);
        QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
        mStreamMutex.unlock();
        return size;
    }

    if (standBy) {
        rm->lockGraph();
        status = session->open(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session open failed with status %d", status);
            goto error;
        }

        status = session->prepare(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session prepare is failed with status %d",
                    status);
            goto error;
        }
        status = session->start(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session start is failed with status %d",
                    status);
            goto error;
        }
        standBy = false;
        rm->unlockGraph();
    }
    mStreamMutex.unlock();

    if (currentState == STREAM_STARTED) {
        status = session->write(this, SHMEM_ENDPOINT, buf, &size, 0);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session write is failed with status %d", status);
            mStreamMutex.lock();
            if (standBy) {
                QAL_INFO(LOG_TAG, "in standby state, ignore write failure");
                mStreamMutex.unlock();
                return buf->size;
            }
            mStreamMutex.unlock();

            //TODO : Currently we are getting -EPERM from agm during SSR,
            //need to be changed once agm gives fix to send -ENETRESET
            if (status == -EPERM) {
                size = buf->size;
                status = size;
                if (rm->cardState != CARD_STATUS_OFFLINE) {
                    QAL_ERR(LOG_TAG, "Sound card offline");
                    rm->cardState = CARD_STATUS_OFFLINE;
                }
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else if (rm->cardState == CARD_STATUS_OFFLINE) {
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else {
                goto exit;
            }
         }
         QAL_DBG(LOG_TAG, "Exit. session write successful size - %d", size);
         return size;
    } else {
        QAL_ERR(LOG_TAG, "Stream not started yet, state %d", currentState);
        status = -EINVAL;
        goto exit;
    }

error:
    if (session->close(this) != 0) {
        QAL_ERR(LOG_TAG, "session close failed");
    }
    rm->unlockGraph();
    mStreamMutex.unlock();
exit :
    QAL_DBG(LOG_TAG, "session write failed status %d", status);
    return status;
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
            if (param_payload->payload_size > sizeof(bool)) {
                QAL_ERR(LOG_TAG, "Invalid payload size %d", param_payload->payload_size);
                status = -EINVAL;
                break;
            }
            fluence_flag = *((bool *)param_payload->payload);
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

            effectQalPayload = (effect_qal_payload_t *)(param_payload->payload);
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
        case QAL_PARAM_ID_TTY_MODE:
        {
            param_payload = (qal_param_payload *)payload;
            if (param_payload->payload_size > sizeof(uint32_t)) {
                QAL_ERR(LOG_TAG, "Invalid payload size %d", param_payload->payload_size);
                status = -EINVAL;
                break;
            }
            uint32_t tty_mode = *((uint32_t *)param_payload->payload);
            status = session->setParameters(this, TTY_MODE, param_id, payload);
            if (status)
               QAL_ERR(LOG_TAG, "setParam for tty mode %d failed with %d",
                       tty_mode, status);
            break;
        }
        case QAL_PARAM_ID_DEVICE_ROTATION:
        {
            // Call Session for Setting the parameter.
            if (NULL != session) {
                status = session->setParameters(this, 0,
                                                QAL_PARAM_ID_DEVICE_ROTATION,
                                                payload);
            } else {
                QAL_ERR(LOG_TAG, "Session is null");
                status = -EINVAL;
            }
        }
        break;
        case QAL_PARAM_ID_VOLUME_BOOST:
        {
            status = session->setParameters(this, VOICE_VOLUME_BOOST, param_id, payload);
            if (status)
               QAL_ERR(LOG_TAG, "setParam for volume boost failed with %d",
                       status);
            break;
        }
        case QAL_PARAM_ID_SLOW_TALK:
        {
            bool slow_talk = false;
            param_payload = (qal_param_payload *)payload;
            slow_talk = *((bool *)param_payload->payload);
            QAL_ERR(LOG_TAG,"slow talk %d", slow_talk);

            uint32_t slow_talk_tag =
                          slow_talk ? VOICE_SLOW_TALK_ON : VOICE_SLOW_TALK_OFF;
            status = session->setParameters(this, slow_talk_tag,
                                            param_id, payload);
            if (status)
               QAL_ERR(LOG_TAG, "setParam for slow talk failed with %d",
                       status);
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
        status = session->setConfig(this, MODULE, MUTE_TAG);
    else
        status = session->setConfig(this, MODULE, UNMUTE_TAG);

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
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        cachedState = STREAM_PAUSED;
        isPaused = true;
        QAL_ERR(LOG_TAG, "Sound Card Offline, cached state %d", cachedState);
        goto exit;
    }

    status = session->setConfig(this, MODULE, PAUSE_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d",
                status);
        goto exit;
    }
    usleep(VOLUME_RAMP_PERIOD);
    isPaused = true;
    currentState = STREAM_PAUSED;
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
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        cachedState = STREAM_STARTED;
        QAL_ERR(LOG_TAG, "Sound Card offline, cached state %d", cachedState);
        goto exit;
    }

    status = session->setConfig(this, MODULE, RESUME_TAG);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig for resume failed with status %d",
                status);
        goto exit;
    }
    isPaused = false;
    currentState = STREAM_STARTED;
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");
exit:
    mStreamMutex.unlock();
    return status;
}

int32_t StreamPCM::standby()
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter.");

    mStreamMutex.lock();
    if (!standBy) {
        status = session->stop(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Rx session stop failed with status %d", status);
            goto exit;
        }

        for (int i = 0; i < mDevices.size(); i++) {
            status = mDevices[i]->stop();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "device stop failed with status %d", status);
                goto exit;
            }

            rm->deregisterDevice(mDevices[i]);

            status = mDevices[i]->close();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "device close failed with status %d", status);
                goto exit;
            }
            mDevices.erase(mDevices.begin() + i);
        }

        rm->lockGraph();
        status = session->close(this);
        rm->unlockGraph();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session close failed with status %d", status);
            goto exit;
        }
        standBy = true;
    }
exit:
    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. status: %d", status);
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

    if (currentState == STREAM_STOPPED) {
        QAL_ERR(LOG_TAG, "Already flushed, state %d", currentState);
        goto exit;
    }

    status = session->flush();
    currentState = STREAM_STOPPED;
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

int32_t StreamPCM::setECRef(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;

    if (!session)
        return -EINVAL;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    mStreamMutex.lock();
    status = session->setECRef(this, dev, is_enable);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to set ec ref in session");
    }
    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamPCM::ssrDownHandler() {
    int status = 0;

    cachedState = currentState;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK cached State %d",
            session, cachedState);

    if (currentState == STREAM_INIT || currentState == STREAM_STOPPED) {
        //Not calling stream close here, as we don't want to delete the session
        //and device objects.
        rm->lockGraph();
        status = session->close(this);
        rm->unlockGraph();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session close failed. status %d", status);
            goto exit;
        }
    } else if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        status = stop();
        if (0 != status)
            QAL_ERR(LOG_TAG, "stream stop failed. status %d",  status);
        rm->lockGraph();
        status = session->close(this);
        rm->unlockGraph();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session close failed. status %d", status);
            goto exit;
        }
    } else {
       QAL_ERR(LOG_TAG, "stream state is %d, nothing to handle", currentState);
       goto exit;
    }

exit :
    currentState = STREAM_IDLE;
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamPCM::ssrUpHandler() {
    int status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK state %d",
            session, cachedState);

    if (cachedState == STREAM_INIT) {
        status = open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stream open failed. status %d", status);
            goto exit;
        }
    } else if (cachedState == STREAM_STARTED) {
        status = open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stream open failed. status %d", status);
            goto exit;
        }
        status = start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stream start failed. status %d", status);
            goto exit;
        }
    } else if (cachedState == STREAM_PAUSED) {
        status = open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stream open failed. status %d", status);
            goto exit;
        }
        status = start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stream start failed. status %d", status);
            goto exit;
        }
        status = setPause();
        if (0 != status) {
           QAL_ERR(LOG_TAG, "stream set pause failed. status %d", status);
            goto exit;
        }
    } else {
        QAL_ERR(LOG_TAG, "stream not in correct state to handle %d", cachedState);
        goto exit;
    }
exit :
    cachedState = STREAM_IDLE;
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamPCM::createMmapBuffer(int32_t min_size_frames,
                                   struct qal_mmap_buffer *info)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    if (currentState == STREAM_INIT) {
        mStreamMutex.lock();
        status = session->createMmapBuffer(this, min_size_frames, info);
        if (0 != status)
            QAL_ERR(LOG_TAG, "session prepare failed with status = %d", status);
        mStreamMutex.unlock();
    } else {
        status = -EINVAL;
    }
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);

    return status;
}

int32_t StreamPCM::GetMmapPosition(struct qal_mmap_position *position)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    mStreamMutex.lock();
    status = session->GetMmapPosition(this, position);
    if (0 != status)
        QAL_ERR(LOG_TAG, "session prepare failed with status = %d", status);
    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);

    return status;
}

