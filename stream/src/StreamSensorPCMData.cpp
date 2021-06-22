/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: StreamSensorPCMData"

#include "StreamSensorPCMData.h"
#include "Session.h"
#include "kvh2xml.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include <unistd.h>

StreamSensorPCMData::StreamSensorPCMData(const struct pal_stream_attributes *sattr __unused,
                    struct pal_device *dattr __unused,
                    const uint32_t no_of_devices __unused,
                    const struct modifier_kv *modifiers __unused,
                    const uint32_t no_of_modifiers __unused,
                    const std::shared_ptr<ResourceManager> rm):
                    StreamCommon(sattr,dattr,no_of_devices,modifiers,no_of_modifiers,rm)
{
    int32_t enable_concurrency_count = 0;
    int32_t disable_concurrency_count = 0;

    PAL_DBG(LOG_TAG, "Enter");
    // get ACD platform info
    acd_info_ = ACDPlatformInfo::GetInstance();
    if (!acd_info_) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to get acd platform info", -EINVAL);
        throw std::runtime_error("Failed to get acd platform info");
    }

    rm->registerStream(this);

    // Print the concurrency feature flags supported
    PAL_INFO(LOG_TAG, "capture conc enable %d,voice conc enable %d,voip conc enable %d",
             acd_info_->GetConcurrentCaptureEnable(),
             acd_info_->GetConcurrentVoiceCallEnable(),
             acd_info_->GetConcurrentVoipCallEnable());

    // check concurrency count from rm
    rm->GetSoundTriggerConcurrencyCount(PAL_STREAM_SENSOR_PCM_DATA,
                                        &enable_concurrency_count,
                                        &disable_concurrency_count);

    // check if lpi should be used
    if (rm->IsLPISupported(PAL_STREAM_SENSOR_PCM_DATA) &&
        !enable_concurrency_count)
        use_lpi_ = true;
    else
        use_lpi_ = false;

    PAL_DBG(LOG_TAG, "Exit");
}

StreamSensorPCMData::~StreamSensorPCMData()
{
    cachedState = STREAM_IDLE;
    rm->resetStreamInstanceID(this);
    rm->deregisterStream(this);
    mDevices.clear();
    PAL_DBG(LOG_TAG, "Exit");
}

int32_t StreamSensorPCMData::open()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter.");

    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        PAL_ERR(LOG_TAG, "Error:Sound card offline, can not open stream");
        usleep(SSR_RECOVERY);
        status = -EIO;
        goto exit;
    }

    if (currentState == STREAM_IDLE) {
        currentState = STREAM_INIT;
        PAL_DBG(LOG_TAG, "streamLL opened. state %d", currentState);
    } else if (currentState == STREAM_INIT) {
        PAL_INFO(LOG_TAG, "Stream is already opened, state %d", currentState);
        status = 0;
        goto exit;
    } else {
        PAL_ERR(LOG_TAG, "Error:Stream is not in correct state %d", currentState);
        status = -EINVAL;
        goto exit;
    }
exit:
    PAL_DBG(LOG_TAG, "Exit ret %d", status)
    return status;
}

int32_t StreamSensorPCMData::start()
{
    int32_t status = 0, devStatus = 0;

    PAL_DBG(LOG_TAG, "Enter. session handle - %pK, state - %d,  device count - %zu\n",
            session, currentState, mDevices.size());

    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (rm->cardState == CARD_STATUS_OFFLINE) {
        cachedState = STREAM_STARTED;
        PAL_ERR(LOG_TAG, "Error:Sound card offline. Update the cached state %d",
                cachedState);
        goto exit;
    }

    if (currentState == STREAM_INIT || currentState == STREAM_STOPPED) {
        status = session->open(this);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Error:session open failed with status %d", status);
            goto exit;
        }
        PAL_DBG(LOG_TAG, "session open successful");

        for (int32_t i = 0; i < mDevices.size(); i++) {
            status = mDevices[i]->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error: device [%d] open failed with status %d",
                        mDevices[i]->getSndDeviceId(), status);
                goto exit;
            }

            status = mDevices[i]->start();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error: device [%d] start failed with status %d",
                        mDevices[i]->getSndDeviceId(), status);
                goto exit;
            }
            rm->registerDevice(mDevices[i], this);
            PAL_DBG(LOG_TAG, "device [%d] opened and started successfully",
                    mDevices[i]->getSndDeviceId());
        }

        status = startSession();
        if (0 != status)
            goto exit;
        PAL_DBG(LOG_TAG, "session start successful");

        /*
         * pcm_open and pcm_start done at once here,
         * so directly jump to STREAM_STARTED state.
         */
        currentState = STREAM_STARTED;
    } else if (currentState == STREAM_STARTED) {
        PAL_INFO(LOG_TAG, "Stream already started, state %d", currentState);
    } else {
        PAL_ERR(LOG_TAG, "Error:Stream is not opened yet");
        status = -EINVAL;
    }

exit:
    PAL_DBG(LOG_TAG, "Exit. state %d, status %d", currentState, status);
    return status;
}

void StreamSensorPCMData::GetUUID(class SoundTriggerUUID *uuid,
                                  const struct st_uuid *vendor_uuid)
{
    uuid->timeLow = (uint32_t)vendor_uuid->timeLow;
    uuid->timeMid = (uint16_t)vendor_uuid->timeMid;
    uuid->timeHiAndVersion = (uint16_t)vendor_uuid->timeHiAndVersion;
    uuid->clockSeq = (uint16_t)vendor_uuid->clockSeq;
    uuid->node[0] = (uint8_t)vendor_uuid->node[0];
    uuid->node[1] = (uint8_t)vendor_uuid->node[1];
    uuid->node[2] = (uint8_t)vendor_uuid->node[2];
    uuid->node[3] = (uint8_t)vendor_uuid->node[3];
    uuid->node[4] = (uint8_t)vendor_uuid->node[4];
    uuid->node[5] = (uint8_t)vendor_uuid->node[5];
    PAL_INFO(LOG_TAG, "Input vendor uuid : %08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
             uuid->timeLow, uuid->timeMid, uuid->timeHiAndVersion, uuid->clockSeq,
             uuid->node[0], uuid->node[1], uuid->node[2], uuid->node[3], uuid->node[4],
             uuid->node[5]);
}

/* Use below UUID for Sensor PCM Data usecase */
static const struct st_uuid qc_sensor_pcm_data_uuid =
    { 0xc88a2c89, 0x7a55, 0x498c, 0x836f, { 0x5d, 0x7e, 0xc8, 0x58, 0x29, 0x90 } };

int32_t StreamSensorPCMData::SetupStreamConfig(const struct st_uuid *vendor_uuid)
{
    int32_t status = 0;
    class SoundTriggerUUID uuid;

    PAL_DBG(LOG_TAG, "Enter");
    GetUUID(&uuid, vendor_uuid);

    if (NULL == sm_cfg_) {
        sm_cfg_ = acd_info_->GetStreamConfig(uuid);
        if (!sm_cfg_) {
            status = -EINVAL;
            goto exit;
        }
    }

    mStreamSelector = sm_cfg_->GetStreamConfigName();
    mInstanceID = rm->getStreamInstanceID(this);

exit:
    if (status)
        PAL_ERR(LOG_TAG, "SetupStreamConfig failed with status %d", status);
    PAL_DBG(LOG_TAG, "Exit, status %d, mInstanceID is %d", status, mInstanceID);
    return status;
}

pal_device_id_t StreamSensorPCMData::GetAvailCaptureDevice()
{
    if (acd_info_->GetSupportDevSwitch() &&
        rm->isDeviceAvailable(PAL_DEVICE_IN_WIRED_HEADSET))
        return PAL_DEVICE_IN_HEADSET_VA_MIC;
    else
        return PAL_DEVICE_IN_HANDSET_VA_MIC;
}

std::shared_ptr<CaptureProfile> StreamSensorPCMData::GetCurrentCaptureProfile()
{
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<Device> device = nullptr;
    struct pal_device dev;
    enum StInputModes input_mode = ST_INPUT_MODE_HANDSET;
    enum StOperatingModes operating_mode = ST_OPERATING_MODE_HIGH_PERF_NS;

    PAL_DBG(LOG_TAG, "Enter");

    if (GetAvailCaptureDevice() == PAL_DEVICE_IN_HEADSET_VA_MIC)
        input_mode = ST_INPUT_MODE_HEADSET;

    /* Check use_lpi_ here again to determine the actual operating_mode */
    if ((DEVICEPP_TX_RAW_LPI == pcm_data_stream_effect ||
        DEVICEPP_TX_FLUENCE_FFEC == pcm_data_stream_effect)
        && use_lpi_)
        operating_mode = ST_OPERATING_MODE_LOW_POWER;
    else if ((DEVICEPP_TX_RAW_LPI == pcm_data_stream_effect ||
             DEVICEPP_TX_FLUENCE_FFEC == pcm_data_stream_effect)
             && !use_lpi_)
        operating_mode = ST_OPERATING_MODE_HIGH_PERF;
    else if ((DEVICEPP_TX_FLUENCE_FFNS == pcm_data_stream_effect ||
             DEVICEPP_TX_FLUENCE_FFECNS == pcm_data_stream_effect)
             && use_lpi_)
        operating_mode = ST_OPERATING_MODE_LOW_POWER_NS;
    else if ((DEVICEPP_TX_FLUENCE_FFNS == pcm_data_stream_effect ||
             DEVICEPP_TX_FLUENCE_FFECNS == pcm_data_stream_effect)
             && !use_lpi_)
        operating_mode = ST_OPERATING_MODE_HIGH_PERF_NS;

    cap_prof = sm_cfg_->GetCaptureProfile(
                                std::make_pair(operating_mode, input_mode));

    PAL_DBG(LOG_TAG, "cap_prof %s: dev_id=0x%x, chs=%d, sr=%d, snd_name=%s",
            cap_prof->GetName().c_str(), cap_prof->GetDevId(),
            cap_prof->GetChannels(), cap_prof->GetSampleRate(),
            cap_prof->GetSndName().c_str());

    // update the best device
    dev.id = GetAvailCaptureDevice();
    dev.config.bit_width = cap_prof->GetBitWidth();
    dev.config.ch_info.channels = cap_prof->GetChannels();
    dev.config.sample_rate = cap_prof->GetSampleRate();
    dev.config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;

    device = Device::getInstance(&dev, rm);
    if (!device) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to get device instance", -EINVAL);
        return nullptr;
    }

    device->setDeviceAttributes(dev);
    device->setSndName(cap_prof->GetSndName());
    mDevices.clear();
    mDevices.push_back(device);

    return cap_prof;
}

int32_t StreamSensorPCMData::addRemoveEffect(pal_audio_effect_t effect,
                                             bool enable)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    std::lock_guard<std::mutex> lck(mStreamMutex);

    /* Check use_lpi_ here to determine if EC is needed */
    if (enable) {
        if (PAL_AUDIO_EFFECT_NONE == effect && use_lpi_) {
           pcm_data_stream_effect = DEVICEPP_TX_RAW_LPI;
        } else if (PAL_AUDIO_EFFECT_NS == effect && use_lpi_) {
           pcm_data_stream_effect = DEVICEPP_TX_FLUENCE_FFNS;
        } else if (PAL_AUDIO_EFFECT_NONE == effect && !use_lpi_) {
           pcm_data_stream_effect = DEVICEPP_TX_FLUENCE_FFEC;
        } else if (PAL_AUDIO_EFFECT_NS == effect && !use_lpi_) {
           pcm_data_stream_effect = DEVICEPP_TX_FLUENCE_FFECNS;
        } else {
            PAL_ERR(LOG_TAG, "Invalid effect ID %d", effect);
            status = -EINVAL;
            goto exit;
        }
    } else {
        PAL_DBG(LOG_TAG, "Exit, no stream effect is set");
        status = -EINVAL;
        goto exit;
    }

    /* Use QC Sensor PCM Data as default streamConfig */
    status = SetupStreamConfig(&qc_sensor_pcm_data_uuid);
    if (status)
        goto exit;

    cap_prof_ = GetCurrentCaptureProfile();
    if (cap_prof_) {
        mDevPPSelector = cap_prof_->GetName();
    } else {
        status = -EINVAL;
        goto exit;
    }
    PAL_DBG(LOG_TAG, "Exit. addRemoveEffect successful");

exit:
    if (status)
        PAL_ERR(LOG_TAG, "addRemoveEffect failed with status %d", status);
    return status;
}

int32_t StreamSensorPCMData::resume() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);

    status = start();
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Resume Stream failed", status);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::pause() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);

    status = stop();
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Pause Stream failed", status);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::HandleConcurrentStream(bool active) {
    int32_t status = 0;
    std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;

    PAL_DBG(LOG_TAG, "Enter, active:%d", active);
    std::lock_guard<std::mutex> lck(mStreamMutex);

    new_cap_prof = GetCurrentCaptureProfile();
    if (new_cap_prof && cap_prof_ != new_cap_prof) {
        PAL_DBG(LOG_TAG,
            "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
            cap_prof_->GetName().c_str(),
            cap_prof_->GetDevId(),
            cap_prof_->GetChannels(),
            cap_prof_->GetSampleRate());
        PAL_DBG(LOG_TAG,
            "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
            new_cap_prof->GetName().c_str(),
            new_cap_prof->GetDevId(),
            new_cap_prof->GetChannels(),
            new_cap_prof->GetSampleRate());

        if (!active) {
            PAL_DBG(LOG_TAG, "disconnect device %d", GetAvailCaptureDevice());
            /* disconnect the backend device */
            status = DisconnectDevice_l(GetAvailCaptureDevice());
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to disconnect device %d",
                        status, GetAvailCaptureDevice());
        } else {
            /* store the pre-proc KV selected in the config file */
            if (new_cap_prof)
                mDevPPSelector = new_cap_prof->GetName();
            /* connect the backend device */
            PAL_DBG(LOG_TAG, "connect device %d", GetAvailCaptureDevice());
            status = ConnectDevice_l(GetAvailCaptureDevice());
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to connect device %d",
                        status, GetAvailCaptureDevice());
        }
    } else {
      PAL_INFO(LOG_TAG, "no action needed, same capture profile");
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::EnableLPI(bool is_enable) {
    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (!rm->IsLPISupported(PAL_STREAM_SENSOR_PCM_DATA)) {
        PAL_DBG(LOG_TAG, "Ignored as LPI not supported");
    } else {
        use_lpi_ = is_enable;
    }

    PAL_DBG(LOG_TAG, "%s is enabled", is_enable ? "LPI" : "NLPI");
    return 0;
}

int32_t StreamSensorPCMData::DisconnectDevice_l(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, device_id: %d", device_id);
    if (mDevices[0]->getSndDeviceId() != device_id) {
        PAL_ERR(LOG_TAG, "Error:%d Device %d not connected, ignore",
                -EINVAL, device_id);
        goto exit;
    }

    if (session)
        status = session->disconnectSessionDevice(this,
                                                  this->mStreamAttr->type,
                                                  mDevices[0]);
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Failed to disconnect device %d",
                status, device_id);

    status = mDevices[0]->stop();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d device stop failed", status);
        goto disconnect_err;
    }

    this->rm->deregisterDevice(mDevices[0], this);

    status = mDevices[0]->close();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d device close failed", status);
        goto disconnect_err;
    }

disconnect_err:
    mDevices.clear();
exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::DisconnectDevice(pal_device_id_t device_id) {
    int32_t status = -EINVAL;

    PAL_DBG(LOG_TAG, "Enter, device_id: %d", device_id);
    /*
     * NOTE: mStreamMutex will be unlocked after ConnectDevice handled
     * because device disconnect/connect should be handled sequencely,
     * and no other commands from client should be handled between
     * device disconnect and connect.
     */
    mStreamMutex.lock();
    status = DisconnectDevice_l(device_id);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::ConnectDevice_l(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, device_id: %d", device_id);

    status = mDevices[0]->open();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d device %d open failed", status,
                mDevices[0]->getSndDeviceId());
        goto connect_err;
    }

    if (session)
        status = session->setupSessionDevice(this,
                                             this->mStreamAttr->type,
                                             mDevices[0]);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d setupSessionDevice for %d failed",
                status, mDevices[0]->getSndDeviceId());
        mDevices[0]->close();
        goto connect_err;
    }

    status = mDevices[0]->start();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d device %d start failed",
                status, mDevices[0]->getSndDeviceId());
        goto connect_err;
    }
    this->rm->registerDevice(mDevices[0], this);

    if (session)
        status = session->connectSessionDevice(this,
                                               this->mStreamAttr->type,
                                               mDevices[0]);
    if (status) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to connect device %d",
                status, device_id);
        mDevices[0]->close();
    } else {
        PAL_DBG(LOG_TAG, "Update capture profile after device switch");
        cap_prof_ = GetCurrentCaptureProfile();
        if (cap_prof_)
            mDevPPSelector = cap_prof_->GetName();
    }

connect_err:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSensorPCMData::ConnectDevice(pal_device_id_t device_id) {
    int32_t status = -EINVAL;

    PAL_DBG(LOG_TAG, "Enter, device_id: %d", device_id);

    status = ConnectDevice_l(device_id);
    mStreamMutex.unlock();

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}
