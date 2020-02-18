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

#define LOG_TAG "Device"

#include "Device.h"
#include <tinyalsa/asoundlib.h>
#include "DeviceAlsa.h"
#include "DeviceGsl.h"
#include "ResourceManager.h"
#include "SessionAlsaUtils.h"
#include "Device.h"
#include "Speaker.h"
#include "Headphone.h"
#include "USBAudio.h"
#include "SpeakerMic.h"
#include "DeviceImpl.h"
#include "Stream.h"
#include "HeadsetMic.h"
#include "HandsetMic.h"
#include "HandsetVaMic.h"
#include "HeadsetVaMic.h"
#include "Handset.h"
#include "Bluetooth.h"
#include "DisplayPort.h"

#define MAX_CHANNEL_SUPPORTED 2

std::shared_ptr<Device> Device::getInstance(struct qal_device *device,
                                                 std::shared_ptr<ResourceManager> Rm)
{
    if (!device || !Rm) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return NULL;
    }

    QAL_DBG(LOG_TAG, "Enter device id %d", device->id);

    //TBD: decide on supported devices from XML and not in code
    switch (device->id) {
    case QAL_DEVICE_OUT_HANDSET:
        QAL_VERBOSE(LOG_TAG, "handset device");
        return Handset::getInstance(device, Rm);
    case QAL_DEVICE_OUT_SPEAKER:
        QAL_VERBOSE(LOG_TAG, "speaker device");
        return Speaker::getInstance(device, Rm);
    case QAL_DEVICE_OUT_WIRED_HEADSET:
    case QAL_DEVICE_OUT_WIRED_HEADPHONE:
        QAL_VERBOSE(LOG_TAG, "headphone device");
        return Headphone::getInstance(device, Rm);
    case QAL_DEVICE_OUT_USB_DEVICE:
    case QAL_DEVICE_OUT_USB_HEADSET:
    case QAL_DEVICE_IN_USB_DEVICE:
    case QAL_DEVICE_IN_USB_HEADSET:
        QAL_VERBOSE(LOG_TAG, "USB device");
        return USB::getInstance(device, Rm);
    case QAL_DEVICE_IN_HANDSET_MIC:
        QAL_VERBOSE(LOG_TAG, "HandsetMic device");
        return HandsetMic::getInstance(device, Rm);
    case QAL_DEVICE_IN_SPEAKER_MIC:
        QAL_VERBOSE(LOG_TAG, "speakerMic device");
        return SpeakerMic::getInstance(device, Rm);
    case QAL_DEVICE_IN_WIRED_HEADSET:
        QAL_VERBOSE(LOG_TAG, "HeadsetMic device");
        return HeadsetMic::getInstance(device, Rm);
    case QAL_DEVICE_IN_HANDSET_VA_MIC:
        QAL_VERBOSE(LOG_TAG, "HandsetVaMic device");
        return HandsetVaMic::getInstance(device, Rm);
    case QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
    case QAL_DEVICE_OUT_BLUETOOTH_SCO:
        QAL_VERBOSE(LOG_TAG, "BTSCO device");
        return BtSco::getInstance(device, Rm);
    case QAL_DEVICE_IN_BLUETOOTH_A2DP:
    case QAL_DEVICE_OUT_BLUETOOTH_A2DP:
        QAL_VERBOSE(LOG_TAG, "BTA2DP device");
        return BtA2dp::getInstance(device, Rm);
    case QAL_DEVICE_OUT_AUX_DIGITAL:
    case QAL_DEVICE_OUT_AUX_DIGITAL_1:
    case QAL_DEVICE_OUT_HDMI:
        QAL_ERR(LOG_TAG, "Display Port device");
        return DisplayPort::getInstance(device, Rm);
        break;
    case QAL_DEVICE_IN_HEADSET_VA_MIC:
        QAL_VERBOSE(LOG_TAG, "HeadsetVaMic device");
        return HeadsetVaMic::getInstance(device, Rm);
        break;
    default:
        QAL_ERR(LOG_TAG,"Unsupported device id %d",device->id);
        return nullptr;
    }
}

std::shared_ptr<Device> Device::getObject(qal_device_id_t dev_id)
{

    switch(dev_id) {
    case QAL_DEVICE_OUT_HANDSET:
        QAL_VERBOSE(LOG_TAG, "handset device");
        return Handset::getObject();
    case QAL_DEVICE_OUT_SPEAKER:
        QAL_VERBOSE(LOG_TAG, "speaker device");
        return Speaker::getObject();
    default:
        QAL_ERR(LOG_TAG,"Unsupported device id %d",dev_id);
        return nullptr;
    }
}

Device::Device(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    struct qal_channel_info *device_ch_info;
    uint16_t channels;
    uint16_t ch_info_size;
    rm = Rm;
    if (device->config.ch_info) {
        channels = device->config.ch_info->channels;
    } else {
        channels = MAX_CHANNEL_SUPPORTED;
    }
    QAL_DBG(LOG_TAG, "channels %d", channels);
    ch_info_size = sizeof(uint16_t) + sizeof(uint8_t)*channels;
    device_ch_info = (struct qal_channel_info *) calloc(1, ch_info_size);
    if (device_ch_info == NULL) {
        QAL_ERR(LOG_TAG, "Allocation failed for channel map");
    }
    memset(&deviceAttr, 0, sizeof(struct qal_device));
    casa_osal_memcpy(&deviceAttr, sizeof(struct qal_device), device,
                     sizeof(struct qal_device));
    // copy channel info
    deviceAttr.config.ch_info = device_ch_info;
    if (device->config.ch_info)
        casa_osal_memcpy(deviceAttr.config.ch_info, ch_info_size, device->config.ch_info,
                         ch_info_size);
    else
        QAL_ERR(LOG_TAG, "Channel Map info in NULL");

    mQALDeviceName.clear();
    customPayload = NULL;
    customPayloadSize = 0;
    QAL_ERR(LOG_TAG,"device instance for id %d created", device->id);

}

Device::Device()
{
    initialized = false;
    mQALDeviceName.clear();
}

Device::~Device()
{
    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;
    QAL_ERR(LOG_TAG,"device instance for id %d destroyed", deviceAttr.id);
    if (deviceAttr.config.ch_info)
        free(deviceAttr.config.ch_info);
}

int Device::getDeviceAttributes(struct qal_device *dattr)
{
    int status = 0;

    if (!dattr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"Invalid device attributes status %d", status);
        goto exit;
    }
    casa_osal_memcpy(dattr, sizeof(struct qal_device), &deviceAttr, sizeof(struct qal_device));

exit:
    return status;
}

int Device::getDefaultConfig(qal_param_device_capability_t capability __unused) {
    return 0;
}

int Device::setDeviceAttributes(struct qal_device dattr)
{
    int status = 0;
    int ch_info_size = 0;
    struct qal_channel_info *device_ch_info = NULL;

    QAL_INFO(LOG_TAG,"DeviceAttributes for Device Id %d updated", dattr.id);

    if (dattr.config.ch_info) {
        ch_info_size = sizeof(struct qal_channel_info) +
            dattr.config.ch_info->channels * sizeof(uint8_t);

        device_ch_info =
            (struct qal_channel_info *)calloc(1, ch_info_size);
        if (!device_ch_info) {
            QAL_ERR(LOG_TAG, "Allocation failed for channel map");
            return -EINVAL;
        }

        if (deviceAttr.config.ch_info)
            free(deviceAttr.config.ch_info);
    }

    casa_osal_memcpy(&deviceAttr, sizeof(struct qal_device), &dattr,
                     sizeof(struct qal_device));

    // copy channel info
    deviceAttr.config.ch_info = device_ch_info;
    if (dattr.config.ch_info)
        casa_osal_memcpy(deviceAttr.config.ch_info, ch_info_size,
                         dattr.config.ch_info, ch_info_size);

    return status;
}

int Device::updateCustomPayload(void *payload, size_t size)
{
    if (!customPayloadSize) {
        customPayload = calloc(1, size);
    } else {
        customPayload = realloc(customPayload, customPayloadSize + size);
    }

    if (!customPayload) {
        QAL_ERR(LOG_TAG, "failed to allocate memory for custom payload");
        return -ENOMEM;
    }

    memcpy((uint8_t *)customPayload + customPayloadSize, payload, size);
    customPayloadSize += size;
    QAL_INFO(LOG_TAG, "customPayloadSize = %d", customPayloadSize);
    return 0;
}

int Device::getSndDeviceId()
{
    QAL_VERBOSE(LOG_TAG,"Device Id %d acquired", deviceAttr.id);
    return deviceAttr.id;
}

std::string Device::getQALDeviceName()
{
    QAL_VERBOSE(LOG_TAG, "%s: Device name %s acquired", __func__, mQALDeviceName.c_str());
    return mQALDeviceName;
}


int Device::init(qal_param_device_connection_t device_conn)
{
    return 0;
}

int Device::deinit(qal_param_device_connection_t device_conn)
{
    return 0;
}

int Device::open()
{
    int status = 0;
    mDeviceMutex.lock();
    QAL_DBG(LOG_TAG, "Enter. device count %d for device id %d, initialized %d",
        deviceCount, this->deviceAttr.id, initialized);
    void *stream;
    DeviceImpl *devImpl;

    if (!initialized) {
        const qal_alsa_or_gsl alsaConf = rm->getQALConfigALSAOrGSL();
        if (ALSA == alsaConf) {
            devImpl = new DeviceAlsa();
        } else {
            devImpl = new DeviceGsl();
        }
        if (!devImpl) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "DeviceImpl instantiation failed status %d", status);
            goto exit;
        }
        status = devImpl->open(&(this->deviceAttr), rm);
        if (0!= status) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG,"Failed to open the device");
            delete devImpl;
            goto exit;
        }

        deviceHandle = static_cast<void *>(devImpl);
        mQALDeviceName = rm->getQALDeviceName(this->deviceAttr.id);
        initialized = true;
        QAL_DBG(LOG_TAG, "Device name %s, device id %d initialized %d", mQALDeviceName.c_str(), this->deviceAttr.id, initialized);
    }

    devObj = Device::getInstance(&deviceAttr, rm);

    QAL_DBG(LOG_TAG, "Exit. device count %d", deviceCount);
exit:
    mDeviceMutex.unlock();
    return status;
}

int Device::close()
{
    int status = 0;
    mDeviceMutex.lock();
    QAL_DBG(LOG_TAG, "Enter. device id %d, device name %s, count %d", deviceAttr.id, mQALDeviceName.c_str(), deviceCount);
    if (deviceCount == 0 && initialized) {
        DeviceImpl *dev = static_cast<DeviceImpl *>(deviceHandle);
        if (!dev) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid device handle status %d", status);
            goto exit;
        }
        status = dev->close();
        if (0 != status) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "Failed to close the device status %d", status);
        }
        delete dev;
        initialized = false;
        deviceHandle = nullptr;
    }
    QAL_DBG(LOG_TAG, "Exit. device count %d", deviceCount);
exit :
    mDeviceMutex.unlock();
    return status;
}

int Device::prepare()
{
    int status = 0;
    mDeviceMutex.lock();
    QAL_DBG(LOG_TAG, "Enter. device id %d, device name %s, count %d", deviceAttr.id, mQALDeviceName.c_str(), deviceCount);
    if (deviceCount == 0 && initialized) {
        DeviceImpl *dev = static_cast<DeviceImpl *>(deviceHandle);
         if (!dev) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid device handle status %d", status);
            goto exit;
        }
        status = dev->prepare();
        if (0 != status) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Device Prepare failed status %d", status);
            goto exit;
        }
    }
    QAL_DBG(LOG_TAG, "%s: Exit. device count %d", deviceCount);
exit :
    mDeviceMutex.unlock();
    return status;
}

int Device::start()
{
    int status = 0;
    std::string backEndName;
    mDeviceMutex.lock();

    QAL_DBG(LOG_TAG, "Enter %d count, initialized %d", deviceCount, initialized);
    if (deviceCount == 0 && initialized) {
        status = rm->getAudioRoute(&audioRoute);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", status);
            goto exit;
        }
        status = rm->getSndDeviceName(deviceAttr.id , mSndDeviceName); //getsndName

        QAL_VERBOSE(LOG_TAG, "%s: audio_route %pK SND device name %s", __func__, audioRoute, mSndDeviceName);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to obtain the device name from ResourceManager status %d", status);
            goto exit;
        }

        enableDevice(audioRoute, mSndDeviceName);

        rm->getBackendName(deviceAttr.id, backEndName);
        if (!strlen(backEndName.c_str())) {
            QAL_ERR(LOG_TAG, "Error: Backend name not defined for %d in xml file\n", deviceAttr.id);
            status = -EINVAL;
            goto disable_dev;
        }

        SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &deviceAttr);

        if (customPayloadSize) {
            status = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndName,
                                        customPayload, customPayloadSize);
            if (status) {
                 QAL_ERR(LOG_TAG, "Error: Dev setParam failed for %d\n",
                                   deviceAttr.id);
                 goto disable_dev;
            }
        }

        DeviceImpl *dev = static_cast<DeviceImpl *>(deviceHandle);
        if (!dev) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid device handle status %d", status);
            goto disable_dev;
        }
        status = dev->prepare();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to prepare the device status %d", status);
            goto disable_dev;
        }
        status = dev->start();
        if (0 != status)
        {
            QAL_ERR(LOG_TAG,"%s: Failed to start the device", __func__);
            goto disable_dev;
        }
    }
    deviceCount += 1;
    QAL_DBG(LOG_TAG, "Exit. device count %d", deviceCount);
    goto exit;

disable_dev:
    disableDevice(audioRoute, mSndDeviceName);
exit :
    mDeviceMutex.unlock();
    return status;
}

int Device::stop()
{
    int status = 0;
    mDeviceMutex.lock();
    QAL_DBG(LOG_TAG, "Enter. device id %d, device name %s, count %d", deviceAttr.id, mQALDeviceName.c_str(), deviceCount);
    if (deviceCount == 1 && initialized) {
        disableDevice(audioRoute, mSndDeviceName);
        DeviceImpl *dev = static_cast<DeviceImpl *>(deviceHandle);
        if (!dev) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid device handle status %d", status);
            goto exit;
        }
        status = dev->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Device Stop failed status %d", status);
            goto exit;
        }
    }
    deviceCount -= 1;
    QAL_DBG(LOG_TAG, "Exit. device count %d", deviceCount);
exit :
    mDeviceMutex.unlock();
    return status;
}

int32_t Device::setDeviceParameter(uint32_t param_id, void *param)
{
    return 0;
}

int32_t Device::getDeviceParameter(uint32_t param_id, void **param)
{
    return 0;
}
