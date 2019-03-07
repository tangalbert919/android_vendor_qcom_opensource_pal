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

#define LOG_TAG "CodecDevice"

#include "CodecDevice.h"
#include <tinyalsa/asoundlib.h>
#include "CodecDeviceAlsa.h"
#include "ResourceManager.h"
#include "Device.h"
#include "Speaker.h"
#include "SpeakerMic.h"
#include "CodecDeviceGsl.h"


std::shared_ptr<Device> CodecDevice::devObj = nullptr;

std::shared_ptr<Device> CodecDevice::getInstance(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    switch(device->id)
    {
        case QAL_DEVICE_OUT_SPEAKER:
                QAL_VERBOSE(LOG_TAG,"%s: speaker device", __func__);
                devObj = Speaker::getInstance(device, Rm);
                return devObj;
                break;
        case QAL_DEVICE_IN_SPEAKER_MIC:
                QAL_VERBOSE(LOG_TAG,"speakerMic device", __func__);
                devObj = SpeakerMic::getInstance(device, Rm);
                return devObj;
                break;
        default:
                return nullptr;
    }
}


CodecDevice::CodecDevice(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    rm = Rm;
    memset(&deviceAttr, 0, sizeof(struct qal_device));
    memcpy(&deviceAttr, device, sizeof(struct qal_device));
}

CodecDevice::CodecDevice()
{

}
CodecDevice::~CodecDevice()
{

}

int CodecDevice::open()
{
    int status = 0;
    mutex.lock();
    //#ifdef CONFIG_GSL
    if(pcmFd == NULL){
        CodecDeviceGsl *gsl = new CodecDeviceGsl();
        pcmFd = gsl->open(&(this->deviceAttr), rm);
        if(NULL == pcmFd)
        {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG,"%s: Failed to open the device", __func__);
        }
	    deviceHandle = static_cast<void *>(gsl);
    }

	//#endif
    mutex.unlock();

//#else
    //status = SpeakerAlsa::open(devObj);
    //goto exit;

    return status;
}

int CodecDevice::close()
{
    int status = 0;
    mutex.lock();
    if(deviceCount == 0)
    {
        //#ifdef CONFIG_GSL
        CodecDeviceGsl *gsl= static_cast<CodecDeviceGsl *>(deviceHandle);
        status = gsl->close(pcmFd);
        if(0 != status)
        {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG,"%s: Failed to close the device", __func__);
        }

        //#else
        //status = SpeakerAlsa::close(devObj);
        //goto exit;
        //#endif
    }
    mutex.unlock();

     return status;
}

int CodecDevice::prepare()
{
    int status = 0;
    mutex.lock();
    if(deviceCount == 0)
    {
        //#ifdef CONFIG_GSL
            CodecDeviceGsl *gsl= static_cast<CodecDeviceGsl *>(deviceHandle);
            status = gsl->prepare(pcmFd);
            if (0 != status)
            {
                QAL_ERR(LOG_TAG,"%s: GSL Prepare failed", __func__);
            }
       //#endif
    }
    mutex.unlock();
    return status;
}

int CodecDevice::start()
{
    int status = 0;
    mutex.lock();
    if(deviceCount == 0)
    {
        status = rm->getaudioroute(&audioRoute);
        QAL_VERBOSE(LOG_TAG,"%s:audio_route %p", __func__,audioRoute);
        if(0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: Failed to get the audio_route address", __func__);
        }
        status = rm->getDeviceName(deviceAttr.id , deviceName); 
        if(0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: Failed to obtain the device name from ResourceManager", __func__);
        }
        enableDevice(audioRoute, deviceName);

        //#ifdef CONFIG_GSL
        CodecDeviceGsl *gsl= static_cast<CodecDeviceGsl *>(deviceHandle);
        status = gsl->prepare(pcmFd);
        if(0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: Failed to prepare the device", __func__);
        }

        status = gsl->start(pcmFd);
        if(0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: Failed to start the device", __func__);
        }
        //#endif
    }

    deviceCount += 1;
    mutex.unlock();
    return status;
}

int CodecDevice::stop()
{
    int status = 0;
    mutex.lock();
    if(deviceCount == 1)
    {
        disableDevice(audioRoute, deviceName);
        //#ifdef CONFIG_GSL
        CodecDeviceGsl *gsl= static_cast<CodecDeviceGsl *>(deviceHandle);
        status = gsl->stop(pcmFd);
        if (0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: GSL Stop failed", __func__);
        }
        //#endif
    }
    deviceCount -= 1;
    mutex.unlock();
    return status;
}
