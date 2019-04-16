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

#define LOG_TAG "CodecDeviceGsl"
#include "CodecDevice.h"
#include "CodecDeviceGsl.h"
#include "ResourceManager.h"
#include <tinyalsa/asoundlib.h>
#include <errno.h>
#include <stdio.h>
extern "C" {
#include <fcntl.h>
}

#define CODEC_DEVICE_GSL_DEFAULT_PERIED_SIZE 1920
#define CODEC_DEVICE_GSL_DEFAULT_PERIED_COUNT 2
#define CODEC_DEVICE_GSL_DEFAULT_START_THRESHOLD 1920/4
#define CODEC_DEVICE_GSL_DEFAULT_STOP_THRESHOLD 32767
#define CODEC_DEVICE_GSL_DEFAULT_AVAIL_MIN 1920/4

struct pcm * CodecDeviceGsl::open(struct qal_device *device, std::shared_ptr<ResourceManager> rm_)
{
    int status = 0;
    int sndCard = rm_->getSndCard();
   //int snd_card = 0;
    int pcmId = rm_->getPcmDeviceId(device->id);
    QAL_VERBOSE(LOG_TAG,"pcm id %d and soundcard %d", pcmId, sndCard);
    //int pcm_id = 0;
    int flags;
    struct pcm_config config;
    int fd = 0;
    if(device->id >= QAL_DEVICE_OUT_EARPIECE && device->id <= QAL_DEVICE_OUT_PROXY)
        flags = PCM_OUT;
    else
        flags = PCM_IN;
    if(0 != status)
    {
        QAL_ERR(LOG_TAG,"%s: Failed to obtain the device attributes", __func__);
        return NULL;
    }
    //config.channels = device_->config.ch_info->channels;
    config.channels = 2;
    //config.rate = device_->config.sample_rate;
    config.rate = 48000;
    config.period_size = 512;
    config.period_count = 8;
    config.format = PCM_FORMAT_S16_LE;
    config.start_threshold = 0;
    config.stop_threshold = CODEC_DEVICE_GSL_DEFAULT_STOP_THRESHOLD;
    config.silence_threshold = 0;
    config.silence_size = 0;
    config.avail_min = 512;
    struct pcm *pcmFd = NULL;

    pcmFd = pcm_open(sndCard, pcmId, flags, &config);
    if (NULL == pcmFd)
        QAL_ERR(LOG_TAG,"%s: Failed to open the the device",__func__);
    QAL_ERR(LOG_TAG,"%s:PCMFd %p %d",__func__, pcmFd, errno);
    return pcmFd;
}

int CodecDeviceGsl::close(struct pcm *pcmFd)
{
     int status = 0;
     if (NULL == pcmFd)
     {
         QAL_ERR(LOG_TAG,"%s: Invalid pcmFd to close the device",__func__);
         return -EINVAL;
     }
     status = pcm_close(pcmFd);
     return status;
}

int CodecDeviceGsl::start(struct pcm *pcmFd)
{
     int status = 0;
     if (NULL == pcmFd)
     {
         QAL_ERR(LOG_TAG,"%s: Invalid pcmFd to start the device",__func__);
         return -EINVAL;
     }
     QAL_ERR(LOG_TAG,"%s:PCMFd %p",__func__, pcmFd);
     //status = pcm_start(pcmFd);
     //QAL_ERR(LOG_TAG,"%d status %d", errno, status);
     return status;
}

int CodecDeviceGsl::prepare (struct pcm *pcmFd)
{
     int status = 0;
     if (NULL == pcmFd)
     {
         QAL_ERR(LOG_TAG,"%s: Invalid pcmFd to prepare the device",__func__);
         return -EINVAL;
     }
     status = pcm_prepare(pcmFd);
     return status;
}

int CodecDeviceGsl::stop(struct pcm *pcmFd)
{
     int status = 0;
     if (NULL == pcmFd)
     {
         QAL_ERR(LOG_TAG,"%s: Invalid pcmFd to stop the device",__func__);
         return -EINVAL;
     }
     status = pcm_stop(pcmFd);
     return status;
}

CodecDeviceGsl::CodecDeviceGsl()
{

}

CodecDeviceGsl::~CodecDeviceGsl()
{

}
