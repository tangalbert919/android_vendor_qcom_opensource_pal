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

#define LOG_TAG "StreamInCall"
#define RXDIR 0
#define TXDIR 1

#include "StreamInCall.h"
#include "Session.h"
#include "kvh2xml.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include <unistd.h>

StreamInCall::StreamInCall(const struct qal_stream_attributes *sattr, struct qal_device *dattr,
                    const uint32_t no_of_devices, const struct modifier_kv *modifiers,
                    const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm)
{
    mStreamMutex.lock();
    uint32_t in_channels = 0, out_channels = 0;
    uint32_t attribute_size = 0;

    if (rm->cardState == CARD_STATUS_OFFLINE) {
        QAL_ERR(LOG_TAG, "Sound card offline, can not create stream");
        usleep(SSR_RECOVERY);
        mStreamMutex.unlock();
        throw std::runtime_error("Sound card offline");
    }

    session = NULL;
    mStreamAttr = (struct qal_stream_attributes *)nullptr;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;
    mDevices.clear();
    currentState = STREAM_IDLE;
    //Modify cached values only at time of SSR down.
    cachedState = STREAM_IDLE;

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

    rm->registerStream(this);
    mStreamMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. state %d", currentState);
    return;
}

int32_t  StreamInCall::open()
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
int32_t  StreamInCall::close()
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
        QAL_VERBOSE(LOG_TAG, "closed the devices successfully");
        goto exit;
    } else if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        status = stop();
        if (0 != status)
            QAL_ERR(LOG_TAG, "stream stop failed. status %d",  status);
    }

    rm->lockGraph();
    status = session->close(this);
    rm->unlockGraph();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session close failed with status %d", status);
    }

exit:
    currentState = STREAM_IDLE;
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
    QAL_INFO(LOG_TAG, "Exit. closed the stream successfully %d status %d",
             currentState, status);
    return status;
}

//TBD: move this to Stream, why duplicate code?
int32_t StreamInCall::start()
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

            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Rx session prepare is failed with status %d",
                        status);
                rm->unlockGraph();
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "session prepare successful");

            status = session->start(this);
            if (errno == -ENETRESET &&
                rm->cardState != CARD_STATUS_OFFLINE) {
                QAL_ERR(LOG_TAG, "Sound card offline, informing RM");
                rm->ssrHandler(CARD_STATUS_OFFLINE);
                cachedState = STREAM_STARTED;
                rm->unlockGraph();
                goto exit;
            }
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

            status = session->prepare(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx session prepare is failed with status %d",
                        status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "session prepare successful");

            status = session->start(this);
            if (errno == -ENETRESET &&
                rm->cardState != CARD_STATUS_OFFLINE) {
                QAL_ERR(LOG_TAG, "Sound card offline, informing RM");
                rm->ssrHandler(CARD_STATUS_OFFLINE);
                cachedState = STREAM_STARTED;
                rm->unlockGraph();
                goto exit;
            }
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx session start is failed with status %d",
                        status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "session start successful");
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Stream type is not supported, status %d", status);
            break;
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
int32_t StreamInCall::stop()
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
            break;

        case QAL_AUDIO_INPUT:
            QAL_ERR(LOG_TAG, "In QAL_AUDIO_INPUT case, device count - %d",
                        mDevices.size());

            status = session->stop(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Tx session stop failed with status %d", status);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "session stop successful");
            break;

        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Stream type is not supported with status %d", status);
            break;
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
int32_t StreamInCall::prepare()
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
int32_t  StreamInCall::setStreamAttributes(struct qal_stream_attributes *sattr)
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
int32_t  StreamInCall::setVolume(struct qal_volume_data *volume)
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
        QAL_ERR(LOG_TAG, "Volume payload mask:%x vol:%f",
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

int32_t  StreamInCall::read(struct qal_buffer* buf)
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
            if (errno == -ENETRESET &&
                rm->cardState != CARD_STATUS_OFFLINE) {
                QAL_ERR(LOG_TAG, "Sound card offline, informing RM");
                rm->ssrHandler(CARD_STATUS_OFFLINE);
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else if (rm->cardState == CARD_STATUS_OFFLINE) {
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else {
                status = errno;
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

int32_t  StreamInCall::write(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size = 0;
    uint32_t frameSize = 0;
    uint32_t byteWidth = 0;
    uint32_t sampleRate = 0;
    uint32_t channelCount = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK, state %d",
            session, currentState);

    mStreamMutex.lock();

    // If cached state is not STREAM_IDLE, we are still processing SSR up.
    if ((rm->cardState == CARD_STATUS_OFFLINE)
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
        currentState = STREAM_INIT;

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
        currentState = STREAM_STARTED;
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

            /* ENETRESET is the error code returned by AGM during SSR */
            if (errno == -ENETRESET &&
                rm->cardState != CARD_STATUS_OFFLINE) {
                QAL_ERR(LOG_TAG, "Sound card offline, informing RM");
                rm->ssrHandler(CARD_STATUS_OFFLINE);
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else if (rm->cardState == CARD_STATUS_OFFLINE) {
                size = buf->size;
                status = size;
                QAL_DBG(LOG_TAG, "dropped buffer size - %d", size);
                goto exit;
            } else {
                status = errno;
                goto exit;
            }
         }
         QAL_DBG(LOG_TAG, "Exit. session write successful size - %d", size);
         return size;
    } else {
        QAL_ERR(LOG_TAG, "Stream not started yet, state %d", currentState);
        if (currentState == STREAM_STOPPED)
            status = -EIO;
        else
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

int32_t  StreamInCall::registerCallBack(qal_stream_callback /*cb*/, void */*cookie*/)
{
    return 0;
}

int32_t  StreamInCall::getCallBack(qal_stream_callback * /*cb*/)
{
    return 0;
}

int32_t StreamInCall::getParameters(uint32_t /*param_id*/, void ** /*payload*/)
{
    return 0;
}

int32_t  StreamInCall::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;

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

int32_t  StreamInCall::setMute( bool state)
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

int32_t  StreamInCall::setPause()
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

int32_t  StreamInCall::setResume()
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
        QAL_ERR(LOG_TAG, "session setConfig for pause failed with status %d",
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

int32_t StreamInCall::flush()
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

int32_t StreamInCall::isSampleRateSupported(uint32_t sampleRate)
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

int32_t StreamInCall::isChannelSupported(uint32_t numChannels)
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

int32_t StreamInCall::isBitWidthSupported(uint32_t bitWidth)
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

int32_t StreamInCall::setECRef(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;

    mStreamMutex.lock();
    status = setECRef_l(dev, is_enable);
    mStreamMutex.unlock();

    return status;
}

int32_t StreamInCall::setECRef_l(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;

    if (!session)
        return -EINVAL;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    status = session->setECRef(this, dev, is_enable);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to set ec ref in session");
    }

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamInCall::ssrDownHandler()
{
    int status = 0;

    mStreamMutex.lock();
    /* Updating cached state here only if it's STREAM_IDLE,
     * Otherwise we can assume it is updated by hal thread
     * already.
     */
    if (cachedState == STREAM_IDLE)
        cachedState = currentState;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK cached State %d",
            session, cachedState);

    if (currentState == STREAM_INIT || currentState == STREAM_STOPPED) {
        //Not calling stream close here, as we don't want to delete the session
        //and device objects.
        rm->lockGraph();
        status = session->close(this);
        rm->unlockGraph();
        currentState = STREAM_IDLE;
        mStreamMutex.unlock();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session close failed. status %d", status);
            goto exit;
        }
    } else if (currentState == STREAM_STARTED || currentState == STREAM_PAUSED) {
        mStreamMutex.unlock();
        status = stop();
        if (0 != status)
            QAL_ERR(LOG_TAG, "stream stop failed. status %d",  status);
        mStreamMutex.lock();
        rm->lockGraph();
        status = session->close(this);
        rm->unlockGraph();
        currentState = STREAM_IDLE;
        mStreamMutex.unlock();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "session close failed. status %d", status);
            goto exit;
        }
    } else {
       QAL_ERR(LOG_TAG, "stream state is %d, nothing to handle", currentState);
       mStreamMutex.unlock();
       goto exit;
    }

exit :
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamInCall::addRemoveEffect(qal_audio_effect_t /*effect*/, bool /*enable*/)
{
    QAL_ERR(LOG_TAG, " Function not supported");
    return -ENOSYS;
}

int32_t StreamInCall::ssrUpHandler()
{
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

