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

#define LOG_TAG "DeviceGsl"
#include "Device.h"
#include "DeviceGsl.h"
#include "ResourceManager.h"
#include <tinyalsa/asoundlib.h>
#include <errno.h>
#include <stdio.h>
extern "C" {
#include <fcntl.h>
}

#define DEVICE_GSL_DEFAULT_PERIED_SIZE 1920
#define DEVICE_GSL_DEFAULT_PERIED_COUNT 2
#define DEVICE_GSL_DEFAULT_START_THRESHOLD 1920/4
#define DEVICE_GSL_DEFAULT_STOP_THRESHOLD 32767
#define DEVICE_GSL_DEFAULT_AVAIL_MIN 1920/4
#define DEVICE_GSL_PERIOD_512 512
#define DEVICE_GSL_PERIOD_COUNT_8 8

int DeviceGsl::open(struct qal_device *device,
                                  std::shared_ptr<ResourceManager> rm_)
{
    if (!device || !rm_) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return -EINVAL;
    }
    int sndCard = rm_->getSndCard();
    int pcmId = rm_->getPcmDeviceId(device->id);
    int flags;
    struct pcm_config config;
    int fd = 0;
    QAL_DBG(LOG_TAG, "Enter. pcm id %d and soundcard %d", pcmId, sndCard);
    if(device->id >= QAL_DEVICE_OUT_EARPIECE && device->id <= QAL_DEVICE_OUT_PROXY)
        flags = PCM_OUT;
    else
        flags = PCM_IN;
    
    config.channels = device->config.ch_info->channels;
    config.rate = device->config.sample_rate;
    QAL_DBG(LOG_TAG,"channels %d and samplerate %d", config.channels, config.rate);
    config.period_size = DEVICE_GSL_PERIOD_512;
    config.period_count = DEVICE_GSL_PERIOD_COUNT_8;
    config.format = PCM_FORMAT_S16_LE;
    config.start_threshold = 0;
    config.stop_threshold = DEVICE_GSL_DEFAULT_STOP_THRESHOLD;
    config.silence_threshold = 0;
    config.silence_size = 0;
    config.avail_min = 512;

    pcmFd = pcm_open(sndCard, pcmId, flags, &config);
    if (!pcmFd) {
        QAL_ERR(LOG_TAG, "Failed to open the device %s", strerror(errno));
        return -EIO;
    }
    QAL_DBG(LOG_TAG, "Exit. PCMFd %pK", pcmFd);
    return 0;
}

int DeviceGsl::close()
{
     int status = 0;
     if (!pcmFd) {
         status = -EINVAL;
         QAL_ERR(LOG_TAG, "Invalid pcmFd to close the device status %d", status);
         return status;
     }
     QAL_DBG(LOG_TAG, "Enter. pcmfd %pK", pcmFd);
     status = pcm_close(pcmFd);
     if (0 != status) {
         QAL_ERR(LOG_TAG, "failed to close the device %s status %d", strerror(errno), status);
         return status;
     }
     pcmFd = NULL;
     QAL_DBG(LOG_TAG, "Exit. status %d", status);
     return status;
}

int DeviceGsl::start()
{
     int status = 0;
     if (!pcmFd) {
         status = -EINVAL;
         QAL_ERR(LOG_TAG, "Invalid pcmFd to start the device status %d", status);
         return status;
     }
     QAL_ERR(LOG_TAG, "PCMFd %pK", pcmFd);
     return status;
}

int DeviceGsl::prepare ()
{
     int status = 0;
     if (!pcmFd) {
         status = -EINVAL;
         QAL_ERR(LOG_TAG, "Invalid pcmFd to prepare the device status %d", status);
         return status;
     }
     QAL_DBG(LOG_TAG, "Enter. pcmfd %pK", pcmFd);
     status = pcm_prepare(pcmFd);
     if (0 != status) {
         QAL_ERR(LOG_TAG, "failed to prepare and start the device %s status %d",
                 strerror(errno), status);
         return status;
     }
     QAL_DBG(LOG_TAG, "Exit. status %d", status);
     return status;
}

int DeviceGsl::stop()
{
     int status = 0;
     if (!pcmFd) {
         status = -EINVAL;
         QAL_ERR(LOG_TAG, "Invalid pcmFd to stop the device status %d", status);
         return status;
     }
     QAL_DBG(LOG_TAG, "Enter. pcmfd %pK", pcmFd);
     status = pcm_stop(pcmFd);
     if (0 != status) {
         QAL_ERR(LOG_TAG, "failed to stop the device %s status %d", strerror(errno), status);
         return status;
     }
     QAL_DBG(LOG_TAG, "Exit. status %d", status);
     return status;
}

DeviceGsl::DeviceGsl()
{
   pcmFd = NULL;
}

DeviceGsl::~DeviceGsl()
{
    pcmFd = NULL;
}
