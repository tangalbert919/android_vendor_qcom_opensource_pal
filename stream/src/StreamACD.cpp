/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: StreamACD"

#include "StreamACD.h"

#include <unistd.h>
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

StreamACD::StreamACD(struct pal_stream_attributes *sattr,
                                       struct pal_device *dattr __unused,
                                       uint32_t no_of_devices __unused,
                                       struct modifier_kv *modifiers __unused,
                                       uint32_t no_of_modifiers __unused,
                                       std::shared_ptr<ResourceManager> rm)
{
    int32_t enable_concurrency_count = 0;
    int32_t disable_concurrency_count = 0;

    rec_config_ = nullptr;
    context_config_ = nullptr;
    paused_ = false;
    currentState = STREAM_IDLE;

    // Setting default volume to unity
    mVolumeData = (struct pal_volume_data *)malloc(sizeof(struct pal_volume_data)
                      +sizeof(struct pal_channel_vol_kv));
    mVolumeData->no_of_volpair = 1;
    mVolumeData->volume_pair[0].channel_mask = 0x03;
    mVolumeData->volume_pair[0].vol = 1.0f;

    PAL_DBG(LOG_TAG, "Enter");
    mNoOfModifiers = 0;
    mModifiers = (struct modifier_kv *) (nullptr);

    mStreamAttr = (struct pal_stream_attributes *)calloc(1,
        sizeof(struct pal_stream_attributes));
    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "Error:%d stream attributes allocation failed", -EINVAL);
        throw std::runtime_error("stream attributes allocation failed");
    }

    ar_mem_cpy(mStreamAttr, sizeof(pal_stream_attributes),
                     sattr, sizeof(pal_stream_attributes));
    mStreamAttr->in_media_config.sample_rate = 16000;
    mStreamAttr->in_media_config.bit_width = 16;
    mStreamAttr->in_media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;
    mStreamAttr->in_media_config.ch_info.channels = 1;
    mStreamAttr->direction = PAL_AUDIO_INPUT;

    // get ACD platform info
    acd_info_ = ACDPlatformInfo::GetInstance();
    if (!acd_info_) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to get acd platform info", -EINVAL);
        throw std::runtime_error("Failed to get acd platform info");
    }

    if (!acd_info_->IsACDEnabled()) {
        PAL_ERR(LOG_TAG, "Error:%d ACD not enabled, exiting", -EINVAL);
        throw std::runtime_error("ACD not enabled, exiting");
    }

    rm->registerStream(this);

    // Create internal states
    acd_idle_ = new ACDIdle(*this);
    acd_loaded_ = new ACDLoaded(*this);
    acd_active = new ACDActive(*this);
    acd_detected_ = new ACDDetected(*this);
    acd_ssr_ = new ACDSSR(*this);

    AddState(acd_idle_);
    AddState(acd_loaded_);
    AddState(acd_active);
    AddState(acd_detected_);
    AddState(acd_ssr_);

    // Set initial state
    cur_state_ = acd_idle_;
    prev_state_ = nullptr;
    state_for_restore_ = ACD_STATE_NONE;

    // Print the concurrency feature flags supported
    PAL_INFO(LOG_TAG, "capture conc enable %d,voice conc enable %d,voip conc enable %d",
        acd_info_->GetConcurrentCaptureEnable(), acd_info_->GetConcurrentVoiceCallEnable(),
        acd_info_->GetConcurrentVoipCallEnable());

    // check concurrency count from rm
    rm->GetSoundTriggerConcurrencyCount(PAL_STREAM_ACD, &enable_concurrency_count,
        &disable_concurrency_count);

    // check if lpi should be used
    if (rm->IsLPISupported(PAL_STREAM_ACD) && !enable_concurrency_count) {
        use_lpi_ = true;
    } else {
        use_lpi_ = false;
    }

    /*
     * When voice/voip/record is active and concurrency is not
     * supported, mark paused as true, so that start recognition
     * will be skipped and when voice/voip/record stops, stream
     * will be resumed.
     */
    if (disable_concurrency_count) {
        paused_ = true;
    }

    PAL_DBG(LOG_TAG, "Exit");
}

StreamACD::~StreamACD()
{
    acd_states_.clear();

    rm->deregisterStream(this);
    if (mStreamAttr) {
        free(mStreamAttr);
    }
    mDevices.clear();
    PAL_DBG(LOG_TAG, "Exit");
}

int32_t StreamACD::getTagsWithModuleInfo(size_t *size, uint8_t *payload)
{
    int32_t status = 0;

    if (!payload) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d, Invalid payload", status);
        goto exit;
    }

    if (!engine_) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d, Engine not initialized yet", status);
        goto exit;
    }
    status = engine_->getTagsWithModuleInfo(this, size, payload);
exit:
    return status;
}

int32_t StreamACD::close()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);

    std::shared_ptr<ACDEventConfig> ev_cfg(new ACDUnloadEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);

    if (rec_config_) {
        free(rec_config_);
        rec_config_ = nullptr;
    }

    if (context_config_) {
        free(context_config_);
        context_config_ = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::start()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<ACDEventConfig> ev_cfg(
       new ACDStartRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (!status) {
        currentState = STREAM_STARTED;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::stop()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<ACDEventConfig> ev_cfg(
       new ACDStopRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (!status) {
        currentState = STREAM_STOPPED;
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::Resume() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<ACDEventConfig> ev_cfg(new ACDResumeEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Resume failed", status);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamACD::Pause() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<ACDEventConfig> ev_cfg(new ACDPauseEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Pause failed", status);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamACD::HandleConcurrentStream(bool active) {
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    PAL_DBG(LOG_TAG, "Enter");
    std::shared_ptr<ACDEventConfig> ev_cfg(
        new ACDConcurrentStreamEventConfig(active));
    status = cur_state_->ProcessEvent(ev_cfg);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamACD::EnableLPI(bool is_enable) {
    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (!rm->IsLPISupported(PAL_STREAM_ACD)) {
        PAL_DBG(LOG_TAG, "Ignore as LPI not supported");
    } else {
        use_lpi_ = is_enable;
    }

    return 0;
}

int32_t StreamACD::getParameters(uint32_t param_id __unused, void **payload __unused)
{
    return 0;
}

int32_t StreamACD::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    pal_param_payload *param_payload = (pal_param_payload *)payload;

    if (!param_payload) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Invalid payload for param ID: %d", status, param_id);
        return status;
    }

    PAL_DBG(LOG_TAG, "Enter, param id %d", param_id);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    switch (param_id) {
    case PAL_PARAM_ID_LOAD_SOUND_MODEL: {
        std::shared_ptr<ACDEventConfig> ev_cfg(
                new ACDLoadEventConfig((void *)param_payload->payload));
        status = cur_state_->ProcessEvent(ev_cfg);
        break;
        }
    case PAL_PARAM_ID_RECOGNITION_CONFIG: {
        uint8_t *opaque_ptr = NULL;
        struct pal_st_recognition_config *config =
            (struct pal_st_recognition_config *)param_payload->payload;

        opaque_ptr = (uint8_t *)config + config->data_offset;
        opaque_ptr += sizeof(struct st_param_header);
        std::shared_ptr<ACDEventConfig> ev_cfg(
              new ACDRecognitionCfgEventConfig((void *)opaque_ptr));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      case PAL_PARAM_ID_CONTEXT_LIST: {
          std::shared_ptr<ACDEventConfig> ev_cfg(
              new ACDContextCfgEventConfig((void *)param_payload->payload));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      default: {
          status = -EINVAL;
          PAL_ERR(LOG_TAG, "Error:%d Unsupported param %u", status, param_id);
          break;
      }
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::registerCallBack(pal_stream_callback cb,
                                             uint64_t cookie)
{
    callback_ = cb;
    cookie_ = cookie;

    PAL_VERBOSE(LOG_TAG, "callback_ = %pK", callback_);

    return 0;
}

int32_t StreamACD::getCallBack(pal_stream_callback *cb)
{
    if (!cb) {
        PAL_ERR(LOG_TAG, "Error:%d Invalid cb", -EINVAL);
        return -EINVAL;
    }
    // Do not expect this to be called.
    *cb = callback_;
    return 0;
}

int32_t StreamACD::setECRef(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (use_lpi_) {
        PAL_DBG(LOG_TAG, "EC ref will be handled in LPI/NLPI switch");
        return status;
    }
    status = setECRef_l(dev, is_enable);

    return status;
}

int32_t StreamACD::setECRef_l(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;
    std::shared_ptr<ACDEventConfig> ev_cfg(
        new ACDECRefEventConfig(dev, is_enable));

    PAL_DBG(LOG_TAG, "Enter, enable %d", is_enable);

    if (mDevPpModifiers.size() == 0 ||
         (mDevPpModifiers[0].second != DEVICEPP_TX_FLUENCE_FFECNS)) {
        PAL_DBG(LOG_TAG, "No need to set ec ref");
        goto exit;
    }

    if (dev && !rm->checkECRef(dev, mDevices[0])) {
        PAL_DBG(LOG_TAG, "No need to set ec ref for unmatching rx device");
        goto exit;
    }

    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to handle ec ref event", status);
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

void StreamACD::PopulateCallbackPayload(struct acd_context_event *event, void *payload)
{
    struct pal_st_recognition_event *recognition_event = NULL;
    struct st_param_header *st_header = NULL;
    int offset = 0;
    int data_size = sizeof(struct st_param_header) +
                    sizeof(struct acd_context_event) +
                    (event->num_contexts * sizeof(struct acd_per_context_event_info));

    recognition_event = (struct pal_st_recognition_event *) payload;
    recognition_event->status = PAL_RECOGNITION_STATUS_SUCCESS;
    recognition_event->type = PAL_SOUND_MODEL_TYPE_GENERIC;
    recognition_event->st_handle = (pal_st_handle_t *)this;
    recognition_event->data_size = data_size;
    recognition_event->data_offset = sizeof(struct pal_st_recognition_event);
    recognition_event->media_config.bit_width = BITWIDTH_16;
    recognition_event->media_config.ch_info.channels = CHANNELS_1;
    recognition_event->media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

    st_header = (struct st_param_header *)((uint8_t *)payload + sizeof(struct pal_st_recognition_event));
    st_header->key_id = ST_PARAM_KEY_CONTEXT_EVENT_INFO;
    st_header->payload_size = recognition_event->data_size - sizeof(struct st_param_header);

    offset = sizeof(struct pal_st_recognition_event) + sizeof(struct st_param_header);
    memcpy((uint8_t *)payload + offset, event, st_header->payload_size);
}

void StreamACD::CacheEventData(struct acd_context_event *event)
{
    size_t new_event_size = sizeof(struct pal_st_recognition_event) +
                            sizeof(st_param_header) +
                            sizeof(struct acd_context_event) +
                            (event->num_contexts * sizeof(struct acd_per_context_event_info));
    struct acd_context_event *current_context_event;
    uint8_t *event_data = NULL;
    struct acd_per_context_event_info *per_context_info;
    int offset = 0;

    if (cached_event_data_) {
        offset = cached_event_data_->data_offset + sizeof(st_param_header);
        current_context_event =  (struct acd_context_event *)((uint8_t *)cached_event_data_ + offset);
        new_event_size +=  current_context_event->num_contexts * sizeof(struct acd_per_context_event_info);

        cached_event_data_ = (struct pal_st_recognition_event *)realloc(cached_event_data_, new_event_size);
        current_context_event =  (struct acd_context_event *)((uint8_t *)cached_event_data_ + offset);
        cached_event_data_->data_size += event->num_contexts * sizeof(struct acd_per_context_event_info);
        per_context_info = (struct acd_per_context_event_info *) ((uint8_t *) current_context_event +
                            sizeof(struct acd_context_event) +
                            (current_context_event->num_contexts * sizeof(struct acd_per_context_event_info)));
        event_data = (uint8_t *)event + sizeof(struct acd_context_event);
        for (int i = 0; i < event->num_contexts; i++) {
            memcpy(per_context_info, event_data, sizeof(struct acd_per_context_event_info));
            event_data += sizeof(struct acd_per_context_event_info);
        }
        current_context_event->num_contexts += event->num_contexts;
    } else {
        cached_event_data_ = (struct pal_st_recognition_event *) calloc(1, new_event_size);
        PopulateCallbackPayload(event, cached_event_data_);
    }
}

void StreamACD::SendCachedEventData()
{
    size_t event_size = cached_event_data_->data_size + sizeof(struct pal_st_recognition_event);
    callback_((pal_stream_handle_t *)this, 0, (uint32_t *)cached_event_data_,
                  event_size, cookie_);
}
void StreamACD::NotifyClient(struct acd_context_event *event)
{
    uint8_t *event_data = NULL;
    int offset = 0;
    struct pal_st_recognition_event *recognition_event = NULL;
    size_t event_size = sizeof(struct pal_st_recognition_event) +
                        sizeof(struct st_param_header) +
                        sizeof(struct acd_context_event) +
                        (event->num_contexts * sizeof(struct acd_per_context_event_info));

    event_data = (uint8_t *)calloc(1, event_size);

    if (callback_) {
        PAL_INFO(LOG_TAG, "Notify detection event to client");
        PopulateCallbackPayload(event, event_data);
        callback_((pal_stream_handle_t *)this, 0, (uint32_t *)event_data,
                  event_size, cookie_);
    }
    free(event_data);
}

void StreamACD::SetEngineDetectionData(struct acd_context_event *event)
{
    mStreamMutex.lock();
    std::shared_ptr<ACDEventConfig> ev_cfg(
       new ACDDetectedEventConfig((void *)event));
    cur_state_->ProcessEvent(ev_cfg);
    mStreamMutex.unlock();
}

pal_device_id_t StreamACD::GetAvailCaptureDevice()
{
    if (rm->isDeviceAvailable(PAL_DEVICE_IN_WIRED_HEADSET))
        return PAL_DEVICE_IN_HEADSET_VA_MIC;
    else
        return PAL_DEVICE_IN_HANDSET_VA_MIC;
}

std::shared_ptr<CaptureProfile> StreamACD::GetCurrentCaptureProfile()
{
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    enum StInputModes input_mode = ST_INPUT_MODE_HANDSET;
    enum StOperatingModes operating_mode = ST_OPERATING_MODE_HIGH_PERF;

    if (GetAvailCaptureDevice() == PAL_DEVICE_IN_HEADSET_VA_MIC)
        input_mode = ST_INPUT_MODE_HEADSET;

    if (use_lpi_)
        operating_mode = ST_OPERATING_MODE_LOW_POWER;

    cap_prof = sm_cfg_->GetCaptureProfile(
                std::make_pair(operating_mode, input_mode));

    PAL_DBG(LOG_TAG, "cap_prof %s: dev_id=0x%x, chs=%d, sr=%d, snd_name=%s",
        cap_prof->GetName().c_str(), cap_prof->GetDevId(),
        cap_prof->GetChannels(), cap_prof->GetSampleRate(),
        cap_prof->GetSndName().c_str());

    return cap_prof;
}

int32_t StreamACD::DisconnectDevice(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    /*
     * NOTE: mStreamMutex will be unlocked after ConnectDevice handled
     * because device disconnect/connect should be handled sequencely,
     * and no other commands from client should be handled between
     * device disconnect and connect.
     */
    mStreamMutex.lock();
    std::shared_ptr<ACDEventConfig> ev_cfg(
        new ACDDeviceDisconnectedEventConfig(device_id));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Failed to disconnect device %d", status, device_id);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamACD::ConnectDevice(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::shared_ptr<ACDEventConfig> ev_cfg(
        new ACDDeviceConnectedEventConfig(device_id));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status)
        PAL_ERR(LOG_TAG, "Error:%d Failed to connect device %d", status, device_id);
    mStreamMutex.unlock();
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

std::shared_ptr<Device> StreamACD::GetPalDevice(pal_device_id_t dev_id, bool use_rm_profile)
{
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<Device> device = nullptr;
    struct pal_device dev;

    dev.id = dev_id;

    if (use_rm_profile) {
        /* TODO : Select profile based on priority */
        cap_prof = GetCurrentCaptureProfile();
    } else {
        cap_prof = GetCurrentCaptureProfile();
    }

    dev.config.bit_width = cap_prof->GetBitWidth();
    dev.config.ch_info.channels = cap_prof->GetChannels();
    dev.config.sample_rate = cap_prof->GetSampleRate();
    dev.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

    device = Device::getInstance(&dev, rm);
    if (!device) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to get device instance", -EINVAL);
        goto exit;
    }

    device->setDeviceAttributes(dev);
    device->setSndName(cap_prof->GetSndName());

exit:
    return device;
}

void StreamACD::AddState(ACDState* state)
{
   acd_states_.insert(std::make_pair(state->GetStateId(), state));
}

int32_t StreamACD::GetCurrentStateId()
{
    if (cur_state_)
        return cur_state_->GetStateId();

    return ACD_STATE_NONE;
}

int32_t StreamACD::GetPreviousStateId()
{
    if (prev_state_)
        return prev_state_->GetStateId();

    return ACD_STATE_NONE;
}

void StreamACD::TransitTo(int32_t state_id)
{
    auto it = acd_states_.find(state_id);

    if (it == acd_states_.end()) {
        PAL_ERR(LOG_TAG, "Error:%d Unknown transit state %d", -EINVAL, state_id);
        return;
    }
    prev_state_ = cur_state_;
    cur_state_ = it->second;
    auto oldState = acdStateNameMap.at(prev_state_->GetStateId());
    auto newState = acdStateNameMap.at(it->first);
    PAL_DBG(LOG_TAG, "Stream : state transitioned from %s to %s",
           oldState.c_str(), newState.c_str());
}

int32_t StreamACD::ProcessInternalEvent(
    std::shared_ptr<ACDEventConfig> ev_cfg) {
    return cur_state_->ProcessEvent(ev_cfg);
}

struct acd_recognition_cfg* StreamACD::GetRecognitionConfig()
{
    return rec_config_;
}

void StreamACD::GetUUID(class SoundTriggerUUID *uuid,
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
                        uuid->timeLow,
                        uuid->timeMid,
                        uuid->timeHiAndVersion,
                        uuid->clockSeq,
                        uuid->node[0],
                        uuid->node[1],
                        uuid->node[2],
                        uuid->node[3],
                        uuid->node[4],
                        uuid->node[5]);
}

/* Use below UUID for ACM usecase */
static const struct st_uuid qc_acd_uuid =
    { 0x4e93281b, 0x296e, 0x4d73, 0x9833, { 0x27, 0x10, 0xc3, 0xc7, 0xc1, 0xdb } };

int32_t StreamACD::SetupStreamConfig(const struct st_uuid *vendor_uuid)
{
    int32_t status = 0;
    class SoundTriggerUUID uuid;

    PAL_DBG(LOG_TAG, "Enter");
    GetUUID(&uuid, vendor_uuid);

    sm_cfg_ = acd_info_->GetStreamConfig(uuid);
    if (!sm_cfg_) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Failed to get stream config", status);
        goto exit;
    }

    mStreamModifiers.clear();
    mStreamModifiers.push_back(sm_cfg_->GetStreamMetadata());

    mInstanceID = rm->getStreamInstanceID(this);
exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::SetupDetectionEngine()
{
    int status = 0;
    pal_device_id_t dev_id;
    std::shared_ptr<Device> dev = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    if (sm_cfg_ == NULL) {
        /* Use QC ACD as default streamConfig */
        status = SetupStreamConfig(&qc_acd_uuid);
        if (status) {
            PAL_ERR(LOG_TAG, "Error:%d Failed to setup Stream Config", status);
            goto error_exit;
        }
    }

    // update best device
    dev_id = GetAvailCaptureDevice();
    PAL_DBG(LOG_TAG, "Select available caputre device %d", dev_id);

    dev = GetPalDevice(dev_id, false);
    if (!dev) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Device creation is failed", status);
        goto error_exit;
    }
    mDevices.clear();
    mDevices.push_back(dev);

    cap_prof_ = GetCurrentCaptureProfile();
    /* store the pre-proc KV selected in the config file */
    mDevPpModifiers.clear();
    if (cap_prof_->GetDevicePpKv().first != 0)
        mDevPpModifiers.push_back(cap_prof_->GetDevicePpKv());

    status = mDevices[0]->open();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Error:%d Device open failed", status);
        goto error_exit;
    }

    engine_ = ContextDetectionEngine::Create(this, sm_cfg_);
    if (!engine_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d engine creation failed", status);
        goto error_exit;
    }

    status = engine_->SetupEngine(this, (void *)context_config_);
    if (status) {
        PAL_ERR(LOG_TAG, "Error:%d setup engine failed", status);
        goto error_exit;
    }

    TransitTo(ACD_STATE_LOADED);
error_exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamACD::SendRecognitionConfig(struct acd_recognition_cfg *acd_recog_cfg)
{
    int32_t status = 0;
    struct acd_per_context_cfg *context_cfg = NULL;
    struct pal_param_context_list *old_ctx_cfg = NULL;
    uint32_t i, num_contexts = 0;
    uint8_t *opaque_ptr = (uint8_t *)acd_recog_cfg;
    size_t len = 0;

    PAL_DBG(LOG_TAG, "Enter");

    opaque_ptr += sizeof(struct acd_recognition_cfg);
    context_cfg = (struct acd_per_context_cfg *)opaque_ptr;

    num_contexts = acd_recog_cfg->num_contexts;
    PAL_INFO(LOG_TAG, "Num Contexts =  %d", acd_recog_cfg->num_contexts);

    /* During transition from LPI<->NLPI and vice-versa, rec_config_ remains same.
     * Continue with engine setup if it is same.
     */
    if (rec_config_ == acd_recog_cfg)
        goto setup_engine;

    if (rec_config_)
        free(rec_config_);

    rec_config_ = (struct acd_recognition_cfg *)calloc(1,
        sizeof(struct acd_recognition_cfg) +
        (num_contexts * sizeof(struct acd_per_context_cfg)));
    if (!rec_config_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d Failed to allocate rec_config", status);
        goto exit;
    }
    ar_mem_cpy(rec_config_, sizeof(struct acd_recognition_cfg),
                     acd_recog_cfg, sizeof(struct acd_recognition_cfg));
    ar_mem_cpy((uint8_t *)rec_config_ + sizeof(struct acd_recognition_cfg),
                     (num_contexts * sizeof(struct acd_per_context_cfg)),
                     context_cfg,
                     (num_contexts * sizeof(struct acd_per_context_cfg)));

    // dump acd recognition config data
    if (acd_info_->GetEnableDebugDumps()) {
        ST_DBG_DECLARE(FILE *rec_opaque_fd = NULL; static int rec_opaque_cnt = 0);
        ST_DBG_FILE_OPEN_WR(rec_opaque_fd, ST_DEBUG_DUMP_LOCATION,
            "acd_rec_config", "bin", rec_opaque_cnt);
        ST_DBG_FILE_WRITE(rec_opaque_fd,
            (uint8_t *)rec_config_, sizeof(struct acd_recognition_cfg) +
            (num_contexts * sizeof(struct acd_per_context_cfg)));
        ST_DBG_FILE_CLOSE(rec_opaque_fd);
        PAL_DBG(LOG_TAG, "acd_recognition_cfg data stored in: acd_rec_config_%d.bin",
            rec_opaque_cnt);
        rec_opaque_cnt++;
    }

   /* Fill context_cfg from recog_cfg data and use that for engine setup */
    if (context_config_) {
         old_ctx_cfg = context_config_;
         context_config_ = NULL;
    }

    len = sizeof(struct pal_param_context_list) + (num_contexts * sizeof(uint32_t));
    context_config_ = (struct pal_param_context_list *) calloc(1, len);
    if (!context_config_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d Failed to allocate context_config", status);
        goto exit;
    }
    context_config_->num_contexts = num_contexts;
    for (i = 0; i < num_contexts; i++)
        context_config_->context_id[i] = context_cfg[i].context_id;

    // dump acd context config data
    if (acd_info_->GetEnableDebugDumps()) {
        ST_DBG_DECLARE(FILE *ctx_opaque_fd = NULL; static int ctx_opaque_cnt = 0);
        ST_DBG_FILE_OPEN_WR(ctx_opaque_fd, ST_DEBUG_DUMP_LOCATION,
            "acd_context_config", "bin", ctx_opaque_cnt);
        ST_DBG_FILE_WRITE(ctx_opaque_fd,
            (uint8_t *)context_config_, len);
        ST_DBG_FILE_CLOSE(ctx_opaque_fd);
        PAL_DBG(LOG_TAG, "acd_context_cfg data stored in: acd_context_config_%d.bin",
            ctx_opaque_cnt);
        ctx_opaque_cnt++;
    }

setup_engine:
    if (cur_state_->GetStateId() >= ACD_STATE_LOADED)
        status = engine_->ReconfigureEngine(this, (void *)old_ctx_cfg, (void *)context_config_);
    else
        status = SetupDetectionEngine();

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    if (old_ctx_cfg)
        free(old_ctx_cfg);

    return status;
}

int32_t StreamACD::SendContextConfig(struct pal_param_context_list *config)
{
    int32_t status = 0;
    size_t len  = 0;
    struct pal_param_context_list *old_ctx_cfg = NULL;

    PAL_DBG(LOG_TAG, "Enter");
    if (context_config_)
        old_ctx_cfg = context_config_;

    if (context_config_ == config)
        goto setup_engine;

    len = sizeof(struct pal_param_context_list) +
        (config->num_contexts * sizeof(uint32_t));

    context_config_ = (struct pal_param_context_list *) calloc(1, len);
    if (!context_config_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d Failed to allocate context_config", status);
        goto exit;
    }
    ar_mem_cpy(context_config_, len, config, len);

    // dump acd context config data
    if (acd_info_->GetEnableDebugDumps()) {
        ST_DBG_DECLARE(FILE *ctx_opaque_fd = NULL; static int ctx_opaque_cnt = 0);
        ST_DBG_FILE_OPEN_WR(ctx_opaque_fd, ST_DEBUG_DUMP_LOCATION,
            "acd_context_config", "bin", ctx_opaque_cnt);
        ST_DBG_FILE_WRITE(ctx_opaque_fd,
            (uint8_t *)context_config_, len);
        ST_DBG_FILE_CLOSE(ctx_opaque_fd);
        PAL_DBG(LOG_TAG, "acd_context_cfg data stored in: acd_context_config_%d.bin",
            ctx_opaque_cnt);
        ctx_opaque_cnt++;
    }

setup_engine:
    if (cur_state_->GetStateId() >= ACD_STATE_LOADED)
        status = engine_->ReconfigureEngine(this, (void *)old_ctx_cfg, (void *)context_config_);
    else
        status = SetupDetectionEngine();

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    if (old_ctx_cfg)
        free(old_ctx_cfg);

    return status;
}

int32_t StreamACD::ACDIdle::ProcessEvent(
    std::shared_ptr<ACDEventConfig> ev_cfg)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "ACDIdle: handle event %d for stream instance %u",
        ev_cfg->id_, acd_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ACD_EV_LOAD_SOUND_MODEL: {
            ACDLoadEventConfigData *data =
                (ACDLoadEventConfigData *)ev_cfg->data_.get();
            struct pal_st_sound_model *pal_acd_sm;

            pal_acd_sm = (struct pal_st_sound_model *)data->data_;
            acd_stream_.SetupStreamConfig(&pal_acd_sm->vendor_uuid);
            break;
        }
        case ACD_EV_RECOGNITION_CONFIG: {
            ACDRecognitionCfgEventConfigData *data =
                (ACDRecognitionCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendRecognitionConfig(
               (struct acd_recognition_cfg *)data->data_);
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to send recog config", status);

            break;
        }
        case ACD_EV_CONTEXT_CONFIG: {
            ACDContextCfgEventConfigData *data =
                (ACDContextCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendContextConfig(
               (struct pal_param_context_list *)data->data_);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to send recog config, status %d",
                    status);
            }
            break;
        }
        case ACD_EV_PAUSE: {
            acd_stream_.paused_ = true;
            break;
        }
        case ACD_EV_RESUME: {
            acd_stream_.paused_ = false;
            break;
        }
        case ACD_EV_DEVICE_DISCONNECTED: {
            ACDDeviceDisconnectedEventConfigData *data =
                (ACDDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;
            if (acd_stream_.mDevices.size() == 0) {
                PAL_DBG(LOG_TAG, "No device to disconnect");
                break;
            } else {
                int curr_device_id = acd_stream_.mDevices[0]->getSndDeviceId();
                pal_device_id_t curr_device =
                    static_cast<pal_device_id_t>(curr_device_id);
                if (curr_device != device_id) {
                    PAL_ERR(LOG_TAG, "Error:%d Device %d not connected, ignore",
                        -EINVAL, device_id);
                    break;
                }
            }
            acd_stream_.mDevices.clear();
            break;
        }
        case ACD_EV_DEVICE_CONNECTED: {
            std::shared_ptr<Device> dev = nullptr;
            ACDDeviceConnectedEventConfigData *data =
                (ACDDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;

            dev = acd_stream_.GetPalDevice(dev_id, false);
            if (!dev) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Error:%d Device creation failed", status);
                goto connect_err;
            }

            acd_stream_.mDevices.clear();
            acd_stream_.mDevices.push_back(dev);
        connect_err:
            break;
        }
        case ACD_EV_CONCURRENT_STREAM: {
            // Avoid handling concurrency if context_config_ is not received
            if (!acd_stream_.context_config_)
                break;

            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            ACDConcurrentStreamEventConfigData *data =
                (ACDConcurrentStreamEventConfigData *)ev_cfg->data_.get();
            active = data->is_active_;
            new_cap_prof = acd_stream_.GetCurrentCaptureProfile();
            if (acd_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    acd_stream_.cap_prof_->GetName().c_str(),
                    acd_stream_.cap_prof_->GetDevId(),
                    acd_stream_.cap_prof_->GetChannels(),
                    acd_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (active) {
                    if (acd_stream_.rec_config_) {
                        status = acd_stream_.SendRecognitionConfig(
                            acd_stream_.rec_config_);
                        if (0 != status) {
                            PAL_ERR(LOG_TAG, "Error:%d Failed to send recognition config",
                                    status);
                            break;
                        }
                    } else if (acd_stream_.context_config_) {
                         status = acd_stream_.SendContextConfig(acd_stream_.context_config_);
                         if (0 != status)
                             PAL_ERR(LOG_TAG, "Error:%d Failed to send context config", status);
                         break;
                    }
                    if (acd_stream_.isActive()) {
                        std::shared_ptr<ACDEventConfig> ev_cfg2(
                            new ACDStartRecognitionEventConfig(false));
                        status = acd_stream_.ProcessInternalEvent(ev_cfg2);
                        if (status)
                            PAL_ERR(LOG_TAG, "Error:%d Failed to Start", status);
                    }
                }
            } else {
                PAL_INFO(LOG_TAG,"no action needed, same capture profile");
            }
            break;
        }
        default:
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
    }
    return status;
}

int32_t StreamACD::ACDLoaded::ProcessEvent(
    std::shared_ptr<ACDEventConfig> ev_cfg)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "ACDLoaded: handle event %d for stream instance %u",
        ev_cfg->id_, acd_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ACD_EV_UNLOAD_SOUND_MODEL: {
            if (acd_stream_.mDevices.size() > 0) {
                auto& dev = acd_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Close device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->close();
                if (status)
                    PAL_ERR(LOG_TAG, "Error:%d Device close failed", status);
            }

            status = acd_stream_.engine_->TeardownEngine(&acd_stream_, acd_stream_.context_config_);
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Unload engine failed", status);

            acd_stream_.rm->resetStreamInstanceID(&acd_stream_, acd_stream_.mInstanceID);

            TransitTo(ACD_STATE_IDLE);
            break;
        }
        case ACD_EV_RESUME: {
            acd_stream_.paused_ = false;
            if (!acd_stream_.isActive()) {
                // Possible if App has stopped recognition during active
                // concurrency.
                break;
            }
            // fall through to start
            [[fallthrough]];
        }
        case ACD_EV_START_RECOGNITION: {
            if (acd_stream_.paused_) {
               break; // Concurrency is active, start later.
            }
            auto& dev = acd_stream_.mDevices[0];

            /* Update cap dev based on mode and configuration and start it */
            struct pal_device dattr;
            bool backend_update = false;
            std::shared_ptr<CaptureProfile> cap_prof = nullptr;

            // Do not update capture profile when resuming stream
            if (ev_cfg->id_ == ACD_EV_START_RECOGNITION) {
                backend_update = acd_stream_.rm->UpdateSoundTriggerCaptureProfile(
                    &acd_stream_, true);
                if (backend_update) {
                    status = rm->StopOtherDetectionStreams(&acd_stream_);
                    if (status)
                        PAL_ERR(LOG_TAG, "Error:%d Failed to stop other Detection streams", status);

                    status = rm->StartOtherDetectionStreams(&acd_stream_);
                    if (status)
                        PAL_ERR(LOG_TAG, "Error:%d Failed to start other Detection streams", status);
                }
            }

            dev->getDeviceAttributes(&dattr);

            cap_prof = acd_stream_.rm->GetSoundTriggerCaptureProfile();
            if (!cap_prof) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Error:%d Invalid capture profile", status);
                break;
            }

            dattr.config.bit_width = cap_prof->GetBitWidth();
            dattr.config.ch_info.channels = cap_prof->GetChannels();
            dattr.config.sample_rate = cap_prof->GetSampleRate();
            dev->setDeviceAttributes(dattr);

            /* now start the device */
            PAL_DBG(LOG_TAG, "Start device %d-%s", dev->getSndDeviceId(),
                    dev->getPALDeviceName().c_str());
            dev->setSndName(cap_prof->GetSndName());

            status = dev->start();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d Device start failed", status);
                break;
            } else {
                acd_stream_.rm->registerDevice(dev, &acd_stream_);
            }
            PAL_DBG(LOG_TAG, "device started");

            /* Start the engines */
            status = acd_stream_.engine_->StartEngine(&acd_stream_);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d Start acd engine failed", status);
                goto err_exit;
            }
            TransitTo(ACD_STATE_ACTIVE);
            break;
        err_exit:
            if (acd_stream_.mDevices.size() > 0) {
                acd_stream_.rm->deregisterDevice(acd_stream_.mDevices[0], &acd_stream_);
                acd_stream_.mDevices[0]->stop();
            }
            break;
        }
        case ACD_EV_PAUSE: {
            acd_stream_.paused_ = true;
            break;
        }
        case ACD_EV_DEVICE_DISCONNECTED:{
            ACDDeviceDisconnectedEventConfigData *data =
                (ACDDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;

            if (acd_stream_.mDevices.size() == 0) {
                PAL_DBG(LOG_TAG, "No device to disconnect");
                break;
            } else {
                int curr_device_id = acd_stream_.mDevices[0]->getSndDeviceId();
                pal_device_id_t curr_device =
                    static_cast<pal_device_id_t>(curr_device_id);
                if (curr_device != device_id) {
                    PAL_ERR(LOG_TAG, "Error:%d Device %d not connected, ignore",
                        -EINVAL, device_id);
                    break;
                }
            }
            auto& dev = acd_stream_.mDevices[0];

            acd_stream_.engine_->DisconnectSessionDevice(&acd_stream_,
                    acd_stream_.mStreamAttr->type, dev);

            status = dev->close();
            if (0 != status)
                PAL_ERR(LOG_TAG, "Error:%d dev close failed", status);

            acd_stream_.mDevices.clear();
            break;
        }
        case ACD_EV_DEVICE_CONNECTED: {
            std::shared_ptr<Device> dev = nullptr;
            ACDDeviceConnectedEventConfigData *data =
                (ACDDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;

            dev = acd_stream_.GetPalDevice(dev_id, false);
            if (!dev) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Error:%d Dev creation failed", status);
                goto connect_err;
            }

            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d device %d open failed",
                    status, dev->getSndDeviceId());
                goto connect_err;
            }
            acd_stream_.mDevices.clear();
            acd_stream_.mDevices.push_back(dev);

        connect_err:
            break;
        }
        case ACD_EV_RECOGNITION_CONFIG: {
            ACDRecognitionCfgEventConfigData *data =
                (ACDRecognitionCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendRecognitionConfig(
               (struct acd_recognition_cfg *)data->data_);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to send recog config", status);

            break;
        }
        case ACD_EV_CONTEXT_CONFIG: {
            ACDContextCfgEventConfigData *data =
                (ACDContextCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendContextConfig(
               (struct pal_param_context_list *)data->data_);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to send context config", status);
            break;
        }
        case ACD_EV_EC_REF: {
            ACDECRefEventConfigData *data =
                (ACDECRefEventConfigData *)ev_cfg->data_.get();
            Stream *s = static_cast<Stream *>(&acd_stream_);
            status = acd_stream_.engine_->setECRef(s, data->dev_,
                data->is_enable_);
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to set EC Ref in engine", status);
            break;
        }
        case ACD_EV_CONCURRENT_STREAM: {
            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            ACDConcurrentStreamEventConfigData *data =
                (ACDConcurrentStreamEventConfigData *)ev_cfg->data_.get();
            active = data->is_active_;
            new_cap_prof = acd_stream_.GetCurrentCaptureProfile();
            if (acd_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    acd_stream_.cap_prof_->GetName().c_str(),
                    acd_stream_.cap_prof_->GetDevId(),
                    acd_stream_.cap_prof_->GetChannels(),
                    acd_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (!active) {
                    std::shared_ptr<ACDEventConfig> ev_cfg1(
                        new ACDUnloadEventConfig());
                    status = acd_stream_.ProcessInternalEvent(ev_cfg1);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Error:%d Failed to Unload", status);
                        break;
                    }
                } else {
                    status = -EINVAL;
                    PAL_ERR(LOG_TAG, "Error:%d Invalid operation", -EINVAL);
                }
            } else {
              PAL_INFO(LOG_TAG,"no action needed, same capture profile");
            }
            break;
        }
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }
    return status;
}

int32_t StreamACD::ACDActive::ProcessEvent(
    std::shared_ptr<ACDEventConfig> ev_cfg)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "ACDActive handle event %d for stream instance %u",
        ev_cfg->id_, acd_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ACD_EV_DETECTED: {
            ACDDetectedEventConfigData *data =
                (ACDDetectedEventConfigData *) ev_cfg->data_.get();
            acd_stream_.NotifyClient((struct acd_context_event *)data->data_);
            TransitTo(ACD_STATE_DETECTED);
            break;
        }
        case ACD_EV_PAUSE: {
            acd_stream_.paused_ = true;
            // fall through to stop
            [[fallthrough]];
        }
        case ACD_EV_STOP_RECOGNITION: {
            // Do not update capture profile when pausing stream
            if (ev_cfg->id_ == ACD_EV_STOP_RECOGNITION) {
                bool backend_update = false;
                backend_update = acd_stream_.rm->UpdateSoundTriggerCaptureProfile(
                    &acd_stream_, false);
                if (backend_update) {
                    status = rm->StopOtherDetectionStreams(&acd_stream_);
                    if (status)
                        PAL_ERR(LOG_TAG, "Error:%d Failed to stop other Detection streams", status);

                    status = rm->StartOtherDetectionStreams(&acd_stream_);
                    if (status)
                        PAL_ERR(LOG_TAG, "Error:%d Failed to start other Detection streams", status);
                }
            }

            PAL_VERBOSE(LOG_TAG, "Stop engine");
            status = acd_stream_.engine_->StopEngine(&acd_stream_);
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Stop engine failed", status);

            auto& dev = acd_stream_.mDevices[0];
            PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                    dev->getPALDeviceName().c_str());
            status = dev->stop();
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Device stop failed", status);

            acd_stream_.rm->deregisterDevice(dev, &acd_stream_);

            TransitTo(ACD_STATE_LOADED);
            break;
        }
        case ACD_EV_DEVICE_DISCONNECTED: {
            ACDDeviceDisconnectedEventConfigData *data =
                (ACDDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;

            int curr_device_id = acd_stream_.mDevices[0]->getSndDeviceId();
            if (curr_device_id != device_id) {
                PAL_ERR(LOG_TAG, "Error:%d Device %d not connected, ignore",
                    -EINVAL, device_id);
                if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                    TransitTo(ACD_STATE_DETECTED);
                    acd_stream_.state_for_restore_ = ACD_STATE_NONE;
                }
                break;
            }
            auto& dev = acd_stream_.mDevices[0];

            acd_stream_.engine_->DisconnectSessionDevice(&acd_stream_,
                acd_stream_.mStreamAttr->type, dev);

            status = dev->stop();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d device stop failed", status);
                goto disconnect_err;
            }

            acd_stream_.rm->deregisterDevice(dev, &acd_stream_);

            status = dev->close();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d device close failed", status);
                goto disconnect_err;
            }
        disconnect_err:
            acd_stream_.mDevices.clear();
            if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                TransitTo(ACD_STATE_DETECTED);
                acd_stream_.state_for_restore_ = ACD_STATE_NONE;
            }
            break;
        }
        case ACD_EV_DEVICE_CONNECTED: {
            ACDDeviceConnectedEventConfigData *data =
                (ACDDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;
            std::shared_ptr<Device> dev = nullptr;

            dev = acd_stream_.GetPalDevice(dev_id, false);
            if (!dev) {
                PAL_ERR(LOG_TAG, "Error:%d Device creation failed", -EINVAL);
                status = -EINVAL;
                goto connect_err;
            }
            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d device %d open failed", status,
                    dev->getSndDeviceId());
                goto connect_err;
            }

            status = acd_stream_.engine_->SetupSessionDevice(&acd_stream_,
                acd_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d setupSessionDevice for %d failed",
                        status, dev->getSndDeviceId());
                dev->close();
                goto connect_err;
            }

            status = dev->start();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d device %d start failed",
                    status, dev->getSndDeviceId());
                goto connect_err;
            }
            acd_stream_.rm->registerDevice(dev, &acd_stream_);

            status = acd_stream_.engine_->ConnectSessionDevice(&acd_stream_,
                acd_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Error:%d connectSessionDevice for %d failed",
                      status, dev->getSndDeviceId());
                dev->close();
            } else {
                PAL_DBG(LOG_TAG, "Update capture profile after device switch");
                acd_stream_.cap_prof_ = acd_stream_.GetCurrentCaptureProfile();
            }
            acd_stream_.mDevices.clear();
            acd_stream_.mDevices.push_back(dev);

        connect_err:
            if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                TransitTo(ACD_STATE_DETECTED);
                acd_stream_.state_for_restore_ = ACD_STATE_NONE;
            }
            break;
        }
        case ACD_EV_RECOGNITION_CONFIG: {
            ACDRecognitionCfgEventConfigData *data =
                (ACDRecognitionCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendRecognitionConfig(
               (struct acd_recognition_cfg *)data->data_);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to send recog config", status);

            if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                TransitTo(ACD_STATE_DETECTED);
                acd_stream_.state_for_restore_ = ACD_STATE_NONE;
            }
            break;
        }
        case ACD_EV_CONTEXT_CONFIG: {
            ACDContextCfgEventConfigData *data =
                (ACDContextCfgEventConfigData *)ev_cfg->data_.get();
            status = acd_stream_.SendContextConfig(
               (struct pal_param_context_list *)data->data_);
            if (0 != status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to send context config", status);

            if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                TransitTo(ACD_STATE_DETECTED);
                acd_stream_.state_for_restore_ = ACD_STATE_NONE;
            }
            break;
        }
        case ACD_EV_EC_REF: {
            ACDECRefEventConfigData *data =
                (ACDECRefEventConfigData *)ev_cfg->data_.get();
            Stream *s = static_cast<Stream *>(&acd_stream_);
            status = acd_stream_.engine_->setECRef(s, data->dev_,
                data->is_enable_);
            if (status) {
                PAL_ERR(LOG_TAG, "Error:%d Failed to set EC Ref in engine", status);
            }
            if (acd_stream_.state_for_restore_ == ACD_STATE_DETECTED) {
                TransitTo(ACD_STATE_DETECTED);
                acd_stream_.state_for_restore_ = ACD_STATE_NONE;
            }
            break;
        }
        case ACD_EV_CONCURRENT_STREAM: {
            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            ACDConcurrentStreamEventConfigData *data =
                (ACDConcurrentStreamEventConfigData *)ev_cfg->data_.get();
            active = data->is_active_;
            new_cap_prof = acd_stream_.GetCurrentCaptureProfile();
            if (acd_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    acd_stream_.cap_prof_->GetName().c_str(),
                    acd_stream_.cap_prof_->GetDevId(),
                    acd_stream_.cap_prof_->GetChannels(),
                    acd_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (!active) {
                    std::shared_ptr<ACDEventConfig> ev_cfg1(
                        new ACDStopRecognitionEventConfig(false));
                    status = acd_stream_.ProcessInternalEvent(ev_cfg1);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to Stop, status %d", status);
                        break;
                    }
                    std::shared_ptr<ACDEventConfig> ev_cfg2(new ACDUnloadEventConfig());
                    status = acd_stream_.ProcessInternalEvent(ev_cfg2);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to Unload, status %d", status);
                        break;
                    }
                } else {
                    PAL_ERR(LOG_TAG, "Invalid operation");
                    status = -EINVAL;
                }
            } else {
              PAL_INFO(LOG_TAG,"no action needed, same capture profile");
            }
            break;
        }
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }
    return status;
}

int32_t StreamACD::ACDDetected::ProcessEvent(
    std::shared_ptr<ACDEventConfig> ev_cfg)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "ACDDetected: handle event %d for stream instance %u",
        ev_cfg->id_, acd_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ACD_EV_DETECTED: {
            ACDDetectedEventConfigData *data =
                (ACDDetectedEventConfigData *) ev_cfg->data_.get();
            acd_stream_.CacheEventData((struct acd_context_event *)data->data_);
            break;
        }
        case ACD_EV_START_RECOGNITION: {
            /* notify client if events are present , else move to active */
            if (acd_stream_.cached_event_data_)
                acd_stream_.SendCachedEventData();
            else
                TransitTo(ACD_STATE_ACTIVE);

            break;
        }
        case ACD_EV_RECOGNITION_CONFIG:
        case ACD_EV_CONTEXT_CONFIG:
        case ACD_EV_EC_REF:
        case ACD_EV_DEVICE_DISCONNECTED:
        case ACD_EV_DEVICE_CONNECTED:
            acd_stream_.state_for_restore_ = ACD_STATE_DETECTED;
        default: {
            TransitTo(ACD_STATE_ACTIVE);
            status = acd_stream_.ProcessInternalEvent(ev_cfg);
            if (status)
                PAL_ERR(LOG_TAG, "Error:%d Failed to process event %d", status, ev_cfg->id_);

            break;
        }
    }

    return status;
}

int32_t StreamACD::ACDSSR::ProcessEvent(
   std::shared_ptr<ACDEventConfig> ev_cfg __unused)
{
    int32_t status = 0;
    return status;
}
