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

#define LOG_TAG "PAL: StreamSoundTrigger"

#include "StreamSoundTrigger.h"

#include <unistd.h>

#include "Session.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

// TODO: find another way to print debug logs by default
#define ST_DBG_LOGS
#ifdef ST_DBG_LOGS
#define PAL_DBG(LOG_TAG,...)  PAL_INFO(LOG_TAG,__VA_ARGS__)
#endif

#define ST_DEFERRED_STOP_DEALY_MS (1000)

StreamSoundTrigger::StreamSoundTrigger(struct pal_stream_attributes *sattr,
                                       struct pal_device *dattr __unused,
                                       uint32_t no_of_devices,
                                       struct modifier_kv *modifiers __unused,
                                       uint32_t no_of_modifiers __unused,
                                       std::shared_ptr<ResourceManager> rm) {

    class SoundTriggerUUID uuid;
    reader_ = nullptr;
    detection_state_ = ENGINE_IDLE;
    notification_state_ = ENGINE_IDLE;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;
    sm_config_ = nullptr;
    rec_config_ = nullptr;
    paused_ = false;
    pending_stop_ = false;
    currentState = STREAM_IDLE;

    // Setting default volume to unity
    mVolumeData = (struct pal_volume_data *)malloc(sizeof(struct pal_volume_data)
                      +sizeof(struct pal_channel_vol_kv));
    mVolumeData->no_of_volpair = 1;
    mVolumeData->volume_pair[0].channel_mask = 0x03;
    mVolumeData->volume_pair[0].vol = 1.0f;

    PAL_DBG(LOG_TAG, "Enter");
    // TODO: handle modifiers later
    mNoOfModifiers = 0;
    mModifiers = (struct modifier_kv *) (nullptr);

    charging_state_ = rm->GetChargingState();
    PAL_DBG(LOG_TAG, "Charging State %d", charging_state_);

    // get sound trigger platform info
    st_info_ = SoundTriggerPlatformInfo::GetInstance();
    if (!st_info_) {
        PAL_ERR(LOG_TAG, "Failed to get sound trigger platform info");
        throw std::runtime_error("Failed to get sound trigger platform info");
    }
    mStreamAttr = (struct pal_stream_attributes *)calloc(1,
        sizeof(struct pal_stream_attributes));
    if (!mStreamAttr) {
        PAL_ERR(LOG_TAG, "stream attributes allocation failed");
        throw std::runtime_error("stream attributes allocation failed");
    }

    ar_mem_cpy(mStreamAttr, sizeof(pal_stream_attributes),
                     sattr, sizeof(pal_stream_attributes));

    PAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d",
                no_of_devices);

    /* assume only one input device */
    if (no_of_devices > 1) {
        std::string err;
        err = "incorrect number of devices expected 1, got " +
            std::to_string(no_of_devices);
        PAL_ERR(LOG_TAG, "%s", err.c_str());
        free(mStreamAttr);
        throw std::runtime_error(err);
    }

    rm->registerStream(this);

    // Create internal states
    st_idle_ = new StIdle(*this);
    st_loaded_ = new StLoaded(*this);
    st_active = new StActive(*this);
    st_detected_ = new StDetected(*this);
    st_buffering_ = new StBuffering(*this);
    st_ssr_ = new StSSR(*this);

    AddState(st_idle_);
    AddState(st_loaded_);
    AddState(st_active);
    AddState(st_detected_);
    AddState(st_buffering_);
    AddState(st_ssr_);

    // Set initial state
    cur_state_ = st_idle_;
    prev_state_ = nullptr;
    state_for_restore_ = ST_STATE_NONE;

    // check if lpi should be used
    if (rm->IsVoiceUILPISupported() &&
        !rm->CheckForActiveConcurrentNonLPIStream()) {
        use_lpi_ = true;
    } else {
        use_lpi_ = false;
    }

    timer_thread_ = std::thread(TimerThread, std::ref(*this));
    timer_stop_waiting_ = false;
    exit_timer_thread_ = false;

    PAL_DBG(LOG_TAG, "Exit");
}

StreamSoundTrigger::~StreamSoundTrigger() {
    std::lock_guard<std::mutex> lck(mStreamMutex);
    {
        std::lock_guard<std::mutex> lck(timer_mutex_);
        exit_timer_thread_ = true;
        timer_stop_waiting_ = true;
        timer_wait_cond_.notify_one();
        timer_start_cond_.notify_one();
    }
    if (timer_thread_.joinable()) {
        PAL_DBG(LOG_TAG, "Join timer thread");
        timer_thread_.join();
    }

    st_states_.clear();
    engines_.clear();

    rm->deregisterStream(this);
    if (mStreamAttr) {
        free(mStreamAttr);
    }
    mDevices.clear();
    PAL_DBG(LOG_TAG, "Exit");
}

int32_t StreamSoundTrigger::close() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(new StUnloadEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);

    if (sm_config_) {
        free(sm_config_);
        sm_config_ = nullptr;
    }

    if (rec_config_) {
        free(rec_config_);
        rec_config_ = nullptr;
    }

    if (reader_) {
        delete reader_;
        reader_ = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::start() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StStartRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (!status) {
        currentState = STREAM_STARTED;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::stop() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StStopRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (!status) {
        currentState = STREAM_STOPPED;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::read(struct pal_buffer* buf) {
    int32_t size = 0;

    PAL_VERBOSE(LOG_TAG, "Enter");

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
        new StReadBufferEventConfig((void *)buf));
    size = cur_state_->ProcessEvent(ev_cfg);

    PAL_VERBOSE(LOG_TAG, "Exit, read size %d", size);

    return size;
}

int32_t StreamSoundTrigger::getParameters(uint32_t param_id, void **payload) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, get parameter %u", param_id);
    if (gsl_engine_) {
        status = gsl_engine_->GetParameters(param_id, payload);
        if (status)
            PAL_ERR(LOG_TAG, "Failed to get parameters from engine");
    } else {
        PAL_ERR(LOG_TAG, "No gsl engine present");
        status = -EINVAL;
    }
    return status;
}

int32_t StreamSoundTrigger::setParameters(uint32_t param_id, void *payload) {
    int32_t status = 0;
    pal_param_payload *param_payload = (pal_param_payload *)payload;

    if (!param_payload) {
        PAL_ERR(LOG_TAG, "Invalid payload");
        return -EINVAL;
    }

    PAL_DBG(LOG_TAG, "Enter, param id %d", param_id);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    switch (param_id) {
      case PAL_PARAM_ID_LOAD_SOUND_MODEL: {
          std::shared_ptr<StEventConfig> ev_cfg(
              new StLoadEventConfig((void *)param_payload->payload));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      case PAL_PARAM_ID_RECOGNITION_CONFIG: {
          /*
           * Currently spf needs graph stop and start for next detection.
           * Handle this event similar to fresh start config.
           */
          std::shared_ptr<StEventConfig> ev_cfg(
              new StRecognitionCfgEventConfig((void *)param_payload->payload));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      case PAL_PARAM_ID_STOP_BUFFERING: {
          /*
           * Currently spf needs graph stop and start for next detection.
           * Handle this event similar to STOP_RECOGNITION.
           */
          std::shared_ptr<StEventConfig> ev_cfg(
              new StStopRecognitionEventConfig(false));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      default: {
          status = -EINVAL;
          PAL_ERR(LOG_TAG, "Unsupported param %u", param_id);
          break;
      }
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::HandleConcurrentStream(bool active) {
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    PAL_DBG(LOG_TAG, "Enter");
    std::shared_ptr<StEventConfig> ev_cfg(
        new StConcurrentStreamEventConfig(active));
    status = cur_state_->ProcessEvent(ev_cfg);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::EnableLPI(bool is_enable) {
    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (!rm->IsVoiceUILPISupported()) {
        PAL_DBG(LOG_TAG, "Ignore as LPI not supported");
    }
    use_lpi_ = is_enable;

    return 0;
}

int32_t StreamSoundTrigger::setECRef(std::shared_ptr<Device> dev, bool is_enable) {
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (use_lpi_) {
        PAL_DBG(LOG_TAG, "EC ref will be handled in LPI/NLPI switch");
        return status;
    }
    status = setECRef_l(dev, is_enable);

    return status;
}

int32_t StreamSoundTrigger::setECRef_l(std::shared_ptr<Device> dev, bool is_enable) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, enable %d", is_enable);

    if (mDevPpModifiers.size() == 0 ||
        (mDevPpModifiers[0].first == DEVICEPP_TX &&
        mDevPpModifiers[0].second == DEVICEPP_TX_VOICE_UI_FLUENCE_FFNS)) {
        PAL_DBG(LOG_TAG, "No need to set ec ref in LPI mode or IDLE state");
        return status;
    }

    if (dev && !rm->checkECRef(dev, mDevices[0])) {
        PAL_DBG(LOG_TAG, "No need to set ec ref for unmatching rx device");
        return status;
    }

    std::shared_ptr<StEventConfig> ev_cfg(
        new StECRefEventConfig(dev, is_enable));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to handle ec ref event");
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::DisconnectDevice(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    /*
     * NOTE: mStreamMutex will be unlocked after ConnectDevice handled
     * because device disconnect/connect should be handled sequencely,
     * and no other commands from client should be handled between
     * device disconnect and connect.
     */
    mStreamMutex.lock();
    std::shared_ptr<StEventConfig> ev_cfg(
        new StDeviceDisconnectedEventConfig(device_id));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to disconnect device %d", device_id);
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::ConnectDevice(pal_device_id_t device_id) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::shared_ptr<StEventConfig> ev_cfg(
        new StDeviceConnectedEventConfig(device_id));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to connect device %d", device_id);
    }
    mStreamMutex.unlock();
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::HandleChargingStateUpdate(bool state, bool active) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, state %d", state);
    std::lock_guard<std::mutex> lck(mStreamMutex);

    std::shared_ptr<StEventConfig> ev_cfg(
        new StChargingStateEventConfig(state, active));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to update charging state");
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::Resume() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(new StResumeEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Resume failed");
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::Pause() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(new StPauseEventConfig());
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        PAL_ERR(LOG_TAG, "Pause failed");
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

std::shared_ptr<Device> StreamSoundTrigger::GetPalDevice(
    pal_device_id_t dev_id, struct pal_device *dev, bool use_rm_profile) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<Device> device = nullptr;

    if (!dev) {
        PAL_ERR(LOG_TAG, "Invalid pal device object");
        goto exit;
    }

    dev->id = dev_id;

    if (use_rm_profile) {
        cap_prof = rm->GetSVACaptureProfile();
        if (!cap_prof) {
            PAL_DBG(LOG_TAG, "Failed to get common capture profile");
            cap_prof = GetCurrentCaptureProfile();
        }
    } else {
        /* TODO: we may need to add input param for this function
        * to indicate the device id we need for capture profile
        */
        cap_prof = GetCurrentCaptureProfile();
    }

    dev->config.bit_width = cap_prof->GetBitWidth();
    dev->config.ch_info.channels = cap_prof->GetChannels();
    dev->config.sample_rate = cap_prof->GetSampleRate();
    dev->config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

    device = Device::getInstance(dev, rm);
    if (!device) {
        PAL_ERR(LOG_TAG, "Failed to get device instance");
        goto exit;
    }

    device->setDeviceAttributes(*dev);
    device->setSndName(cap_prof->GetSndName());

exit:

    return device;
}

int32_t StreamSoundTrigger::GetSetupDuration(
    struct audio_dam_downstream_setup_duration **duration) {
    /* NOTE: stream lock should not be handled here
     * as it's calling from session with stream locked
     */
    return gsl_engine_->GetSetupDuration(duration);
}

int32_t StreamSoundTrigger::isSampleRateSupported(uint32_t sampleRate) {
    int32_t rc = 0;

    PAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch (sampleRate) {
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
          PAL_ERR(LOG_TAG, "sample rate not supported rc %d", rc);
          break;
    }

    return rc;
}

int32_t StreamSoundTrigger::isChannelSupported(uint32_t numChannels) {
    int32_t rc = 0;

    PAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch (numChannels) {
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
          PAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
          break;
    }
    return rc;
}

int32_t StreamSoundTrigger::isBitWidthSupported(uint32_t bitWidth) {
    int32_t rc = 0;

    PAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (bitWidth) {
      case BITWIDTH_16:
      case BITWIDTH_24:
      case BITWIDTH_32:
          break;
      default:
          rc = -EINVAL;
          PAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
          break;
    }
    return rc;
}

int32_t StreamSoundTrigger::registerCallBack(pal_stream_callback cb,
                                             void * cookie) {
    callback_ = cb;
    cookie_ = cookie;

    PAL_VERBOSE(LOG_TAG, "callback_ = %pK", callback_);

    return 0;
}

int32_t StreamSoundTrigger::getCallBack(pal_stream_callback *cb) {
    if (!cb) {
        PAL_ERR(LOG_TAG, "Invalid cb");
        return -EINVAL;
    }
    // Do not expect this to be called.
    *cb = callback_;
    return 0;
}

struct detection_event_info* StreamSoundTrigger::GetDetectionEventInfo() {
    return gsl_engine_->GetDetectionEventInfo();
}

int32_t StreamSoundTrigger::SetEngineDetectionState(int32_t det_type) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, det_type %d", det_type);
    if ((det_type < GMM_DETECTED) || (det_type > USER_VERIFICATION_REJECT)) {
        PAL_ERR(LOG_TAG, "Invalid detection type %d", det_type);
        return -EINVAL;
    }

    std::unique_lock<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StDetectedEventConfig(det_type));
    status = cur_state_->ProcessEvent(ev_cfg);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

void StreamSoundTrigger::InternalStopRecognition() {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (pending_stop_) {
        std::shared_ptr<StEventConfig> ev_cfg(
           new StStopRecognitionEventConfig(true));
        status = cur_state_->ProcessEvent(ev_cfg);
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
}

void StreamSoundTrigger::TimerThread(StreamSoundTrigger& st_stream) {
    PAL_DBG(LOG_TAG, "Enter");

    std::unique_lock<std::mutex> lck(st_stream.timer_mutex_);
    while (!st_stream.exit_timer_thread_) {
        st_stream.timer_start_cond_.wait(lck);
        if (st_stream.exit_timer_thread_)
            break;

        st_stream.timer_wait_cond_.wait_for(lck,
            std::chrono::milliseconds(ST_DEFERRED_STOP_DEALY_MS));

        if (!st_stream.timer_stop_waiting_ && !st_stream.exit_timer_thread_) {
            st_stream.timer_mutex_.unlock();
            st_stream.InternalStopRecognition();
            st_stream.timer_mutex_.lock();
        }
    }
    PAL_DBG(LOG_TAG, "Exit");
}

void StreamSoundTrigger::PostDelayedStop() {
    PAL_VERBOSE(LOG_TAG, "Post Delayed Stop for %p", this);
    pending_stop_ = true;
    std::lock_guard<std::mutex> lck(timer_mutex_);
    timer_stop_waiting_ = false;
    timer_start_cond_.notify_one();
}

void StreamSoundTrigger::CancelDelayedStop() {
    PAL_VERBOSE(LOG_TAG, "Cancel Delayed stop for %p", this);
    pending_stop_ = false;
    std::lock_guard<std::mutex> lck(timer_mutex_);
    timer_stop_waiting_ = true;
    timer_wait_cond_.notify_one();
}

/* TODO:
 *   - Need to track vendor UUID
 */
int32_t StreamSoundTrigger::LoadSoundModel(
    struct pal_st_sound_model *sound_model) {

    int32_t status = 0;
    int32_t i = 0;
    int32_t engine_id = 0;
    std::shared_ptr<SoundTriggerEngine> engine = nullptr;
    struct pal_st_phrase_sound_model *phrase_sm = nullptr;
    struct pal_st_sound_model *common_sm = nullptr;
    uint8_t *ptr = nullptr;
    uint8_t *sm_payload = nullptr;
    uint8_t *sm_data = nullptr;
    int32_t sm_size = 0;
    SML_GlobalHeaderType *global_hdr = nullptr;
    SML_HeaderTypeV3 *hdr_v3 = nullptr;
    SML_BigSoundModelTypeV3 *big_sm = nullptr;
    uint32_t sm_version = SML_MODEL_V2;

    PAL_DBG(LOG_TAG, "Enter");

    if (!sound_model) {
        PAL_ERR(LOG_TAG, "Invalid sound_model param status %d", status);
        return -EINVAL;
    }

    sound_model_type_ = sound_model->type;

    if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct pal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(*phrase_sm)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            PAL_ERR(LOG_TAG, "Invalid phrase sound model params data size=%d, "
                   "data offset=%d, type=%d phrases=%d status %d",
                   phrase_sm->common.data_size, phrase_sm->common.data_offset,
                   phrase_sm->common.type, phrase_sm->num_phrases, status);
            return -EINVAL;
        }
        common_sm = (struct pal_st_sound_model*)&phrase_sm->common;
        sm_size = sizeof(*phrase_sm) + common_sm->data_size;

    } else if (sound_model->type == PAL_SOUND_MODEL_TYPE_GENERIC) {
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct pal_st_sound_model))) {
            PAL_ERR(LOG_TAG, "Invalid generic sound model params data size=%d,"
                    " data offset=%d status %d", sound_model->data_size,
                    sound_model->data_offset, status);
            return -EINVAL;
        }
        common_sm = sound_model;
        sm_size = sizeof(*common_sm) + common_sm->data_size;
    } else {
        PAL_ERR(LOG_TAG, "Unknown sound model type - %d status %d",
                sound_model->type, status);
        return -EINVAL;
    }
    if ((struct pal_st_sound_model *)sm_config_ != sound_model) {
        // Cache to use during SSR and other internal events handling.
        if (sm_config_) {
            free(sm_config_);
        }
        sm_config_ = (struct pal_st_phrase_sound_model *)calloc(1, sm_size);
        if (!sm_config_) {
            PAL_ERR(LOG_TAG, "sound model config allocation failed, status %d",
                    status);
            status = -ENOMEM;
            return status;
        }

        if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
            ar_mem_cpy(sm_config_, sizeof(*phrase_sm),
                             phrase_sm, sizeof(*phrase_sm));
            ar_mem_cpy((uint8_t *)sm_config_ + common_sm->data_offset,
                             common_sm->data_size,
                             (uint8_t *)phrase_sm + common_sm->data_offset,
                             common_sm->data_size);
        } else {
            ar_mem_cpy(sm_config_, sizeof(*common_sm),
                             common_sm, sizeof(*common_sm));
            ar_mem_cpy((uint8_t *)sm_config_ +  common_sm->data_offset,
                             common_sm->data_size,
                             (uint8_t *)common_sm + common_sm->data_offset,
                             common_sm->data_size);
        }
    }

    if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
        global_hdr = (SML_GlobalHeaderType *)sm_payload;
        if (global_hdr->magicNumber == SML_GLOBAL_HEADER_MAGIC_NUMBER) {
            // Parse sound model 3.0
            sm_version = SML_MODEL_V3;
            hdr_v3 = (SML_HeaderTypeV3 *)(sm_payload +
                                          sizeof(SML_GlobalHeaderType));
            PAL_DBG(LOG_TAG, "num of sound models = %u", hdr_v3->numModels);
            for (i = 0; i < hdr_v3->numModels; i++) {
                big_sm = (SML_BigSoundModelTypeV3 *)(
                    sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) +
                    (i * sizeof(SML_BigSoundModelTypeV3)));
                engine_id = static_cast<int32_t>(big_sm->type);

                PAL_INFO(LOG_TAG, "type = %u, size = %u",
                         big_sm->type, big_sm->size);
                if (big_sm->type == ST_SM_ID_SVA_GMM) {
                    sm_size = big_sm->size +
                              sizeof(struct pal_st_phrase_sound_model);
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        PAL_ERR(LOG_TAG, "sm_data allocation failed, status %d",
                                status);
                        goto error_exit;
                    }
                    ar_mem_cpy(sm_data, sizeof(*phrase_sm),
                                     (char *)phrase_sm, sizeof(*phrase_sm));
                    common_sm = (struct pal_st_sound_model *)sm_data;
                    common_sm->data_size = big_sm->size;
                    common_sm->data_offset += sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (hdr_v3->numModels * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    ar_mem_cpy(sm_data + sizeof(*phrase_sm), big_sm->size,
                                     (char *)phrase_sm + common_sm->data_offset,
                                     big_sm->size);
                    common_sm->data_offset = sizeof(*phrase_sm);
                    common_sm = (struct pal_st_sound_model *)&phrase_sm->common;

                    gsl_engine_ = SoundTriggerEngine::Create(this,
                                                             ST_SM_ID_SVA_GMM);
                    if (!gsl_engine_) {
                        status = -ENOMEM;
                        PAL_ERR(LOG_TAG, "big_sm: gsl engine creation failed");
                        goto error_exit;
                    }

                    status = gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
                    if (status) {
                        PAL_ERR(LOG_TAG, "big_sm: gsl engine loading model"
                                "failed, status %d", status);
                        goto error_exit;
                    }

                    engine_id = static_cast<int32_t>(ST_SM_ID_SVA_GMM);
                    std::shared_ptr<EngineCfg> engine_cfg(new EngineCfg(
                       engine_id, gsl_engine_, (void *) sm_data, sm_size));

                    AddEngine(engine_cfg);
                } else {
                    sm_size = big_sm->size;
                    ptr = (uint8_t *)sm_payload +
                        sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (hdr_v3->numModels * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        PAL_ERR(LOG_TAG, "Failed to alloc memory for sm_data");
                        goto error_exit;
                    }
                    ar_mem_cpy(sm_data, sm_size, ptr, sm_size);

                    engine = SoundTriggerEngine::Create(this, big_sm->type);
                    if (!engine) {
                        PAL_ERR(LOG_TAG, "Failed to create engine for type %d",
                                big_sm->type);
                        status = -ENOENT;
                        goto error_exit;
                    }

                    status = engine->LoadSoundModel(this, sm_data, sm_size);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Loading model to engine type %d"
                                "failed, status %d", big_sm->type, status);
                        goto error_exit;
                    }

                    if (big_sm->type & ST_SM_ID_SVA_KWD) {
                        notification_state_ |= KEYWORD_DETECTION_SUCCESS;
                    } else if (big_sm->type == ST_SM_ID_SVA_VOP) {
                        notification_state_ |= USER_VERIFICATION_SUCCESS;
                    }

                    std::shared_ptr<EngineCfg> engine_cfg(new EngineCfg(
                       engine_id, engine, (void *)sm_data, sm_size));

                    AddEngine(engine_cfg);
                }
            }
            if (!gsl_engine_) {
                PAL_ERR(LOG_TAG, "First stage sound model not present!!");
                goto error_exit;
            }
        } else {
            // Parse sound model 2.0
            sm_size = sizeof(*phrase_sm) + common_sm->data_size;
            sm_data = (uint8_t *)calloc(1, sm_size);
            if (!sm_data) {
                PAL_ERR(LOG_TAG, "Failed to allocate memory for sm_data");
                status = -ENOMEM;
                goto error_exit;
            }
            ar_mem_cpy(sm_data, sizeof(*phrase_sm),
                             (uint8_t *)phrase_sm, sizeof(*phrase_sm));
            ar_mem_cpy(sm_data + sizeof(*phrase_sm), common_sm->data_size,
                             (uint8_t*)phrase_sm + common_sm->data_offset,
                             common_sm->data_size);
            gsl_engine_ = SoundTriggerEngine::Create(this, ST_SM_ID_SVA_GMM);
            if (!gsl_engine_) {
                PAL_ERR(LOG_TAG, "gsl engine creation failed");
                status = -ENOMEM;
                goto error_exit;
            }

            status = gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
            if (status) {
                PAL_ERR(LOG_TAG, "gsl engine loading model failed, status %d",
                        status);
                goto error_exit;
            }

            engine_id = static_cast<int32_t>(ST_SM_ID_SVA_GMM);
            std::shared_ptr<EngineCfg> engine_cfg(new EngineCfg(engine_id,
                                                          gsl_engine_,
                                                          sm_data, sm_size));
            AddEngine(engine_cfg);
        }
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;

error_exit:
    for (auto &eng: engines_) {
        if (eng->sm_data_) {
            free(eng->sm_data_);
        }
    }
    engines_.clear();
    gsl_engine_.reset();
    if (reader_) {
        delete reader_;
        reader_ = nullptr;
    }
    if (sm_config_) {
        free(sm_config_);
        sm_config_ = nullptr;
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::UpdateSoundModel(
    struct pal_st_sound_model *sound_model) {
    int32_t status = 0;
    int32_t sm_size = 0;
    struct pal_st_phrase_sound_model *phrase_sm = nullptr;
    struct pal_st_sound_model *common_sm = nullptr;

    PAL_DBG(LOG_TAG, "Enter");

    if (!sound_model) {
        PAL_ERR(LOG_TAG, "Invalid sound_model param status %d", status);
        return -EINVAL;
    }
    sound_model_type_ = sound_model->type;

    if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct pal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(*phrase_sm)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            PAL_ERR(LOG_TAG, "Invalid phrase sound model params data size=%d, "
                   "data offset=%d, type=%d phrases=%d status %d",
                   phrase_sm->common.data_size, phrase_sm->common.data_offset,
                   phrase_sm->common.type,phrase_sm->num_phrases, status);
            return -EINVAL;
        }
        common_sm = (struct pal_st_sound_model*)&phrase_sm->common;
        sm_size = sizeof(*phrase_sm) + common_sm->data_size;

    } else if (sound_model->type == PAL_SOUND_MODEL_TYPE_GENERIC) {
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct pal_st_sound_model))) {
            PAL_ERR(LOG_TAG, "Invalid generic sound model params data size=%d,"
                    " data offset=%d status %d", sound_model->data_size,
                    sound_model->data_offset, status);
            return -EINVAL;
        }
        common_sm = sound_model;
        sm_size = sizeof(*common_sm) + common_sm->data_size;
    } else {
        PAL_ERR(LOG_TAG, "Unknown sound model type - %d status %d",
                sound_model->type, status);
        return -EINVAL;
    }
    if ((struct pal_st_sound_model *)sm_config_ != sound_model) {
        // Cache to use during SSR and other internal events handling.
        if (sm_config_) {
            free(sm_config_);
        }
        sm_config_ = (struct pal_st_phrase_sound_model *)calloc(1, sm_size);
        if (!sm_config_) {
            PAL_ERR(LOG_TAG, "sound model config allocation failed, status %d",
                    status);
            status = -ENOMEM;
            return status;
        }

        if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
            ar_mem_cpy(sm_config_, sizeof(*phrase_sm),
                         phrase_sm, sizeof(*phrase_sm));
            ar_mem_cpy((uint8_t *)sm_config_ + common_sm->data_offset,
                         common_sm->data_size,
                         (uint8_t *)phrase_sm + common_sm->data_offset,
                         common_sm->data_size);
        } else {
            ar_mem_cpy(sm_config_, sizeof(*common_sm),
                         common_sm, sizeof(*common_sm));
            ar_mem_cpy((uint8_t *)sm_config_ +  common_sm->data_offset,
                         common_sm->data_size,
                         (uint8_t *)common_sm + common_sm->data_offset,
                         common_sm->data_size);
        }
    }

    return status;
}

// TODO: look into how cookies are used here
int32_t StreamSoundTrigger::SendRecognitionConfig(
    struct pal_st_recognition_config *config) {

    int32_t status = 0;
    int32_t i = 0;
    struct st_param_header *param_hdr = NULL;
    struct st_hist_buffer_info *hist_buf = NULL;
    struct st_det_perf_mode_info *det_perf_mode = NULL;
    uint8_t *opaque_ptr = NULL;
    unsigned int opaque_size = 0, conf_levels_payload_size = 0;
    uint32_t hist_buffer_duration = 0;
    uint32_t pre_roll_duration = 0;
    uint32_t client_capture_read_delay = 0;
    uint32_t conf_levels_intf_version = 0;
    uint8_t *conf_levels = NULL;
    uint32_t num_conf_levels = 0;
    uint32_t ring_buffer_len = 0;
    uint32_t ring_buffer_size = 0;
    bool capture_requested = false;
    std::vector<PalRingBufferReader *> reader_list;

    PAL_DBG(LOG_TAG, "Enter");
    if (!config) {
        PAL_ERR(LOG_TAG, "Invalid config");
        return -EINVAL;
    }
    if (rec_config_ != config) {
        // Possible due to subsequent detections.
        if (rec_config_) {
            free(rec_config_);
        }
        rec_config_ = (struct pal_st_recognition_config *)calloc(1,
            sizeof(struct pal_st_recognition_config) + config->data_size);
        if (!rec_config_) {
            PAL_ERR(LOG_TAG, "Failed to allocate rec_config status %d",
                status);
            return -ENOMEM;
        }
        ar_mem_cpy(rec_config_, sizeof(struct pal_st_recognition_config),
                         config, sizeof(struct pal_st_recognition_config));
        ar_mem_cpy((uint8_t *)rec_config_ + config->data_offset,
                         config->data_size,
                         (uint8_t *)config + config->data_offset,
                         config->data_size);
    }

    // Parse recognition config
    if (config->data_size > CUSTOM_CONFIG_OPAQUE_DATA_SIZE) {
        opaque_ptr = (uint8_t *)config + config->data_offset;
        while (opaque_size < config->data_size) {
            param_hdr = (struct st_param_header *)opaque_ptr;
            PAL_VERBOSE(LOG_TAG, "key %d, payload size %d",
                        param_hdr->key_id, param_hdr->payload_size);

            switch (param_hdr->key_id) {
              case ST_PARAM_KEY_CONFIDENCE_LEVELS:
                  conf_levels_intf_version = *(uint32_t *)(
                      opaque_ptr + sizeof(struct st_param_header));
                  PAL_VERBOSE(LOG_TAG, "conf_levels_intf_version = %u",
                      conf_levels_intf_version);
                  if (conf_levels_intf_version !=
                      CONF_LEVELS_INTF_VERSION_0002) {
                      conf_levels_payload_size =
                          sizeof(struct st_confidence_levels_info);
                  } else {
                      conf_levels_payload_size =
                          sizeof(struct st_confidence_levels_info_v2);
                  }
                  if (param_hdr->payload_size != conf_levels_payload_size) {
                      PAL_ERR(LOG_TAG, "Conf level format error, exiting");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  status = ParseOpaqueConfLevels(opaque_ptr,
                                                 conf_levels_intf_version,
                                                 &conf_levels,
                                                 &num_conf_levels);
                if (status) {
                    PAL_ERR(LOG_TAG, "Failed to parse opaque conf levels");
                    goto error_exit;
                }

                opaque_size += sizeof(struct st_param_header) +
                    conf_levels_payload_size;
                opaque_ptr += sizeof(struct st_param_header) +
                    conf_levels_payload_size;
                if (status) {
                    PAL_ERR(LOG_TAG, "Parse conf levels failed(status=%d)",
                            status);
                    status = -EINVAL;
                    goto error_exit;
                }
                break;
              case ST_PARAM_KEY_HISTORY_BUFFER_CONFIG:
                  if (param_hdr->payload_size !=
                      sizeof(struct st_hist_buffer_info)) {
                      PAL_ERR(LOG_TAG, "History buffer config format error");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  hist_buf = (struct st_hist_buffer_info *)(opaque_ptr +
                      sizeof(struct st_param_header));
                  hist_buffer_duration = hist_buf->hist_buffer_duration_msec;
                  pre_roll_duration = hist_buf->pre_roll_duration_msec;

                  opaque_size += sizeof(struct st_param_header) +
                      sizeof(struct st_hist_buffer_info);
                  opaque_ptr += sizeof(struct st_param_header) +
                      sizeof(struct st_hist_buffer_info);
                  break;
              case ST_PARAM_KEY_DETECTION_PERF_MODE:
                  if (param_hdr->payload_size !=
                      sizeof(struct st_det_perf_mode_info)) {
                      PAL_ERR(LOG_TAG, "Opaque data format error, exiting");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  det_perf_mode = (struct st_det_perf_mode_info *)
                      (opaque_ptr + sizeof(struct st_param_header));
                  PAL_DBG(LOG_TAG, "set perf mode %d", det_perf_mode->mode);
                  opaque_size += sizeof(struct st_param_header) +
                      sizeof(struct st_det_perf_mode_info);
                  opaque_ptr += sizeof(struct st_param_header) +
                      sizeof(struct st_det_perf_mode_info);
                  break;
              default:
                  PAL_ERR(LOG_TAG, "Unsupported opaque data key id, exiting");
                  status = -EINVAL;
                  goto error_exit;
            }
        }
    } else {
        // get history buffer duration from sound trigger platform xml
        hist_buffer_duration = sm_info_->GetKwDuration();
        pre_roll_duration = 0;

        status = FillConfLevels(config, &conf_levels, &num_conf_levels);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to parse conf levels from rc config");
            goto error_exit;
        }
    }

    client_capture_read_delay = sm_info_->GetCaptureReadDelay();
    PAL_DBG(LOG_TAG, "history buf len = %d, preroll len = %d, read delay = %d",
        hist_buffer_duration, pre_roll_duration, client_capture_read_delay);

    status = gsl_engine_->UpdateBufConfig(hist_buffer_duration,
                                          pre_roll_duration);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to update buf config, status %d", status);
        goto error_exit;
    }

    // create ring buffer for lab transfer in gsl_engine
    ring_buffer_len = hist_buffer_duration + pre_roll_duration +
        client_capture_read_delay;
    ring_buffer_size = (ring_buffer_len / 1000) * sm_info_->GetSampleRate() *
                       sm_info_->GetBitWidth() *
                       sm_info_->GetOutChannels() / 8;
    status = gsl_engine_->CreateBuffer(ring_buffer_size,
                                       engines_.size(), reader_list);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get ring buf reader, status %d", status);
        goto error_exit;
    }

    // NOTE: First stage engine is the writer to the buffer
    // PAL client reader should be first in the reader list.
    // The remaining readers are for seconds stage engines.
    reader_ = reader_list[0];
    for (i = 1; i < engines_.size(); i++) {
        status = engines_[i]->GetEngine()->SetBufferReader(reader_list[i]);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to set ring buffer reader");
            goto error_exit;
        }
    }

    gsl_engine_->UpdateConfLevels(this, config,
                                  conf_levels, num_conf_levels);

    // Update capture requested flag to gsl engine
    if (!config->capture_requested && engines_.size() == 1)
        capture_requested = false;
    else
        capture_requested = true;
    gsl_engine_->SetCaptureRequested(capture_requested);
    return status;

error_exit:
    if (conf_levels)
        free(conf_levels);

    if (rec_config_) {
        free(rec_config_);
        rec_config_ = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::UpdateRecognitionConfig(
    struct pal_st_recognition_config *config) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    if (!config) {
        PAL_ERR(LOG_TAG, "Invalid config");
        return -EINVAL;
    }
    if (rec_config_ != config) {
        // Possible due to subsequent detections.
        if (rec_config_) {
            free(rec_config_);
        }
        rec_config_ = (struct pal_st_recognition_config *)calloc(1,
            sizeof(struct pal_st_recognition_config) + config->data_size);
        if (!rec_config_) {
            PAL_ERR(LOG_TAG, "Failed to allocate rec_config status %d", status);
            return -ENOMEM;
        }
        ar_mem_cpy(rec_config_, sizeof(struct pal_st_recognition_config),
                     config, sizeof(struct pal_st_recognition_config));
        ar_mem_cpy((uint8_t *)rec_config_ + config->data_offset,
                     config->data_size,
                     (uint8_t *)config + config->data_offset,
                     config->data_size);
    }

    return status;
}

bool StreamSoundTrigger::compareRecognitionConfig(
   const struct pal_st_recognition_config *current_config,
   struct pal_st_recognition_config *new_config) {
    uint32_t i = 0, j = 0;

    /*
     * Sometimes if the number of user confidence levels is 0, the
     * pal_st_confidence_level struct will be different between the two
     * configs. So all the values must be checked instead of a memcmp of the
     * whole configs.
     */
    if ((current_config->capture_handle != new_config->capture_handle) ||
        (current_config->capture_device != new_config->capture_device) ||
        (current_config->capture_requested != new_config->capture_requested) ||
        (current_config->num_phrases != new_config->num_phrases) ||
        (current_config->data_size != new_config->data_size) ||
        (current_config->data_offset != new_config->data_offset) ||
        std::memcmp((char *) current_config + current_config->data_offset,
               (char *) new_config + new_config->data_offset,
               current_config->data_size)) {
        return false;
    } else {
        for (i = 0; i < current_config->num_phrases; i++) {
            if ((current_config->phrases[i].id !=
                 new_config->phrases[i].id) ||
                (current_config->phrases[i].recognition_modes !=
                 new_config->phrases[i].recognition_modes) ||
                (current_config->phrases[i].confidence_level !=
                 new_config->phrases[i].confidence_level) ||
                (current_config->phrases[i].num_levels !=
                 new_config->phrases[i].num_levels)) {
                return false;
            } else {
                for (j = 0; j < current_config->phrases[i].num_levels; j++) {
                    if ((current_config->phrases[i].levels[j].user_id !=
                         new_config->phrases[i].levels[j].user_id) ||
                        (current_config->phrases[i].levels[j].level !=
                         new_config->phrases[i].levels[j].level))
                        return false;
                }
            }
        }
        return true;
    }
}

int32_t StreamSoundTrigger::notifyClient() {
    int32_t status = 0;
    struct pal_st_recognition_event *rec_event = nullptr;
    uint32_t event_size;

    status = GenerateCallbackEvent(&rec_event, &event_size);
    if (status || !rec_event) {
        PAL_ERR(LOG_TAG, "Failed to generate callback event");
        return status;
    }
    if (callback_) {
        PAL_INFO(LOG_TAG, "Notify detection event to client");
        mStreamMutex.unlock();
        callback_((pal_stream_handle_t *)this, 0, (uint32_t *)rec_event,
                  event_size, rec_config_->cookie);
        mStreamMutex.lock();
    }

    free(rec_event);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::GenerateCallbackEvent(
    struct pal_st_recognition_event **event, uint32_t *evt_size) {

    struct pal_st_phrase_recognition_event *phrase_event = nullptr;
    struct st_param_header *param_hdr = nullptr;
    struct st_confidence_levels_info *conf_levels = nullptr;
    struct st_keyword_indices_info *kw_indices = nullptr;
    struct st_timestamp_info *timestamps = nullptr;
    size_t opaque_size = 0;
    size_t event_size = 0;
    uint8_t *opaque_data = nullptr;
    uint32_t start_index = 0, end_index = 0;

    PAL_DBG(LOG_TAG, "Enter");
    *event = nullptr;
    if (sound_model_type_ == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        struct detection_event_info *det_ev_info =
            gsl_engine_->GetDetectionEventInfo();
        if (!det_ev_info) {
            PAL_ERR(LOG_TAG, "detection info not available");
            return -EINVAL;
        }

        opaque_size = (3 * sizeof(struct st_param_header)) +
            sizeof(struct st_timestamp_info) +
            sizeof(struct st_keyword_indices_info) +
            sizeof(struct st_confidence_levels_info);

        event_size = sizeof(struct pal_st_phrase_recognition_event) +
                     opaque_size;
        phrase_event = (struct pal_st_phrase_recognition_event *)
                       calloc(1, event_size);
        if (!phrase_event) {
            PAL_ERR(LOG_TAG, "Failed to alloc memory for recognition event");
            return -ENOMEM;
        }

        phrase_event->num_phrases = rec_config_->num_phrases;
        memcpy(phrase_event->phrase_extras, rec_config_->phrases,
               phrase_event->num_phrases *
               sizeof(struct pal_st_phrase_recognition_extra));

        *event = &(phrase_event->common);
        (*event)->status = PAL_RECOGNITION_STATUS_SUCCESS;
        (*event)->type = sound_model_type_;
        (*event)->st_handle = (pal_st_handle_t *)this;
        (*event)->capture_available = rec_config_->capture_requested;
        // TODO: generate capture session
        (*event)->capture_session = 0;
        (*event)->capture_delay_ms = 0;
        (*event)->capture_preamble_ms = 0;
        (*event)->trigger_in_data = true;
        (*event)->data_size = opaque_size;
        (*event)->data_offset = sizeof(struct pal_st_phrase_recognition_event);

        (*event)->media_config.sample_rate = SAMPLINGRATE_16K;
        (*event)->media_config.bit_width = BITWIDTH_16;
        (*event)->media_config.ch_info.channels = CHANNELS_1;
        (*event)->media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

        // Filling Opaque data
        opaque_data = (uint8_t *)phrase_event +
                      phrase_event->common.data_offset;

        /* Pack the opaque data confidence levels structure */
        param_hdr = (struct st_param_header *)opaque_data;
        param_hdr->key_id = ST_PARAM_KEY_CONFIDENCE_LEVELS;
        param_hdr->payload_size = sizeof(struct st_confidence_levels_info);
        opaque_data += sizeof(struct st_param_header);
        conf_levels = (struct st_confidence_levels_info *)opaque_data;
        conf_levels->version = 0x1;
        conf_levels->num_sound_models = engines_.size();
        // TODO: update user conf levels
        for (int i = 0; i < conf_levels->num_sound_models; i++) {
            conf_levels->conf_levels[i].sm_id = ST_SM_ID_SVA_GMM;
            conf_levels->conf_levels[i].num_kw_levels = 1;
            conf_levels->conf_levels[i].kw_levels[0].kw_level =
                det_ev_info->confidence_levels[i];
            conf_levels->conf_levels[i].kw_levels[0].num_user_levels = 0;
        }
        opaque_data += param_hdr->payload_size;

        /* Pack the opaque data keyword indices structure */
        param_hdr = (struct st_param_header *)opaque_data;
        param_hdr->key_id = ST_PARAM_KEY_KEYWORD_INDICES;
        param_hdr->payload_size = sizeof(struct st_keyword_indices_info);
        opaque_data += sizeof(struct st_param_header);
        kw_indices = (struct st_keyword_indices_info *)opaque_data;
        kw_indices->version = 0x1;
        reader_->getIndices(&start_index, &end_index);
        kw_indices->start_index = start_index;
        kw_indices->end_index = end_index;
        opaque_data += sizeof(struct st_keyword_indices_info);

        /*
         * Pack the opaque data detection time structure
         * TODO: add support for 2nd stage detection timestamp
         */
        param_hdr = (struct st_param_header *)opaque_data;
        param_hdr->key_id = ST_PARAM_KEY_TIMESTAMP;
        param_hdr->payload_size = sizeof(struct st_timestamp_info);
        opaque_data += sizeof(struct st_param_header);
        timestamps = (struct st_timestamp_info *)opaque_data;
        timestamps->version = 0x1;
        timestamps->first_stage_det_event_time = 1000 *
            ((uint64_t)det_ev_info->detection_timestamp_lsw +
            ((uint64_t)det_ev_info->detection_timestamp_msw << 32));
    }
    *evt_size = event_size;
    // TODO: handle for generic sound model
    PAL_DBG(LOG_TAG, "Exit");

    return 0;
}

int32_t StreamSoundTrigger::ParseOpaqueConfLevels(
    void *opaque_conf_levels,
    uint32_t version,
    uint8_t **out_conf_levels,
    uint32_t *out_num_conf_levels) {

    int32_t status = 0;
    struct st_confidence_levels_info *conf_levels = nullptr;
    struct st_confidence_levels_info_v2 *conf_levels_v2 = nullptr;
    struct st_sound_model_conf_levels *sm_levels = nullptr;
    struct st_sound_model_conf_levels_v2 *sm_levels_v2 = nullptr;
    uint8_t confidence_level = 0;
    uint8_t confidence_level_v2 = 0;
    bool gmm_conf_found = false;

    PAL_DBG(LOG_TAG, "Enter");
    if (version != CONF_LEVELS_INTF_VERSION_0002) {
        conf_levels = (struct st_confidence_levels_info *)
            ((char *)opaque_conf_levels + sizeof(struct st_param_header));
        for (int i = 0; i < conf_levels->num_sound_models; i++) {
            sm_levels = &conf_levels->conf_levels[i];
            if (sm_levels->sm_id == ST_SM_ID_SVA_GMM) {
                gmm_conf_found = true;
                FillOpaqueConfLevels((void *)sm_levels, out_conf_levels,
                                     out_num_conf_levels, version);
            } else if (sm_levels->sm_id & ST_SM_ID_SVA_KWD ||
                       sm_levels->sm_id & ST_SM_ID_SVA_VOP) {
                confidence_level =
                    (sm_levels->sm_id & ST_SM_ID_SVA_KWD) ?
                    sm_levels->kw_levels[0].kw_level:
                    sm_levels->kw_levels[0].user_levels[0].level;
                PAL_DBG(LOG_TAG, "confidence level = %d", confidence_level);
                for (auto& eng: engines_) {
                    if (sm_levels->sm_id == eng->GetEngineId()) {
                        eng->GetEngine()->UpdateConfLevels(this, rec_config_,
                            &confidence_level, 1);
                    }
                }
            }
        }
    } else {
        conf_levels_v2 = (struct st_confidence_levels_info_v2 *)
            ((char *)opaque_conf_levels + sizeof(struct st_param_header));
        for (int i = 0; i < conf_levels_v2->num_sound_models; i++) {
            sm_levels_v2 = &conf_levels_v2->conf_levels[i];
            if (sm_levels_v2->sm_id == ST_SM_ID_SVA_GMM) {
                gmm_conf_found = true;
                FillOpaqueConfLevels((void *)sm_levels_v2, out_conf_levels,
                                     out_num_conf_levels, version);
            } else if (sm_levels_v2->sm_id & ST_SM_ID_SVA_KWD ||
                       sm_levels_v2->sm_id & ST_SM_ID_SVA_VOP) {
                confidence_level_v2 =
                    (sm_levels_v2->sm_id & ST_SM_ID_SVA_KWD) ?
                    sm_levels_v2->kw_levels[0].kw_level:
                    sm_levels_v2->kw_levels[0].user_levels[0].level;
                PAL_DBG(LOG_TAG, "confidence level = %d", confidence_level_v2);
                for (auto& eng: engines_) {
                    PAL_VERBOSE(LOG_TAG, "sm id %d, engine id %d ",
                        sm_levels_v2->sm_id , eng->GetEngineId());
                    if (sm_levels_v2->sm_id == eng->GetEngineId()) {
                        eng->GetEngine()->UpdateConfLevels(this, rec_config_,
                            &confidence_level_v2, 1);
                    }
                }
            }
        }
    }

    if (!gmm_conf_found) {
        PAL_ERR(LOG_TAG, "Did not receive GMM confidence threshold, error!");
        status = -EINVAL;
        goto exit;
    }

exit:
    PAL_DBG(LOG_TAG, "Exit");

    return status;
}

int32_t StreamSoundTrigger::FillConfLevels(
    struct pal_st_recognition_config *config,
    uint8_t **out_conf_levels,
    uint32_t *out_num_conf_levels) {

    int32_t status = 0;
    uint32_t num_conf_levels = 0;
    unsigned int user_level, user_id;
    unsigned int i = 0, j = 0;
    uint8_t *conf_levels = nullptr;
    unsigned char *user_id_tracker = nullptr;
    struct pal_st_phrase_sound_model *phrase_sm = nullptr;

    PAL_DBG(LOG_TAG, "Enter");

    if (!config) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "invalid input status %d", status);
        goto exit;
    }

    for (auto& eng: engines_) {
        if (eng->GetEngineId() == ST_SM_ID_SVA_GMM) {
            phrase_sm = (struct pal_st_phrase_sound_model *)eng->sm_data_;
            break;
        }
    }

    if ((config->num_phrases == 0) ||
        (config->num_phrases > phrase_sm->num_phrases)) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid phrase data status %d", status);
        goto exit;
    }

    for (i = 0; i < config->num_phrases; i++) {
        num_conf_levels++;
        for (j = 0; j < config->phrases[i].num_levels; j++)
            num_conf_levels++;
    }

    conf_levels = (unsigned char*)calloc(1, num_conf_levels);
    if (!conf_levels) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
        goto exit;
    }

    user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
    if (!user_id_tracker) {
        free(conf_levels);
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                status);
        goto exit;
    }

    /* for debug */
    for (i = 0; i < config->num_phrases; i++) {
        PAL_VERBOSE(LOG_TAG, "[%d] kw level %d", i,
        config->phrases[i].confidence_level);
        for (j = 0; j < config->phrases[i].num_levels; j++) {
            PAL_VERBOSE(LOG_TAG, "[%d] user_id %d level %d ", i,
                        config->phrases[i].levels[j].user_id,
                        config->phrases[i].levels[j].level);
        }
    }

    /* Example: Say the recognition structure has 3 keywords with users
     *      [0] k1 |uid|
     *              [0] u1 - 1st trainer
     *              [1] u2 - 4th trainer
     *              [3] u3 - 3rd trainer
     *      [1] k2
     *              [2] u2 - 2nd trainer
     *              [4] u3 - 5th trainer
     *      [2] k3
     *              [5] u4 - 6th trainer
     *    Output confidence level array will be
     *    [k1, k2, k3, u1k1, u2k1, u2k2, u3k1, u3k2, u4k3]
     */

    for (i = 0; i < config->num_phrases; i++) {
        conf_levels[i] = config->phrases[i].confidence_level;
        for (j = 0; j < config->phrases[i].num_levels; j++) {
            user_level = config->phrases[i].levels[j].level;
            user_id = config->phrases[i].levels[j].user_id;
            if ((user_id < config->num_phrases) ||
                (user_id >= num_conf_levels)) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid params user id %d status %d",
                        user_id, status);
                goto exit;
            } else {
                if (user_id_tracker[user_id] == 1) {
                    status = -EINVAL;
                    PAL_ERR(LOG_TAG, "Duplicate user id %d status %d", user_id,
                            status);
                    goto exit;
                }
                conf_levels[user_id] = (user_level < 100) ? user_level : 100;
                user_id_tracker[user_id] = 1;
                PAL_VERBOSE(LOG_TAG, "user_conf_levels[%d] = %d", user_id,
                            conf_levels[user_id]);
            }
        }
    }

    *out_conf_levels = conf_levels;
    *out_num_conf_levels = num_conf_levels;

exit:
    if (user_id_tracker)
        free(user_id_tracker);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::FillOpaqueConfLevels(
    const void *sm_levels_generic,
    uint8_t **out_payload,
    uint32_t *out_payload_size,
    uint32_t version) {

    int status = 0;
    unsigned int num_conf_levels = 0;
    unsigned int user_level = 0, user_id = 0;
    unsigned char *conf_levels = nullptr;
    unsigned int i = 0, j = 0;
    unsigned char *user_id_tracker = nullptr;
    struct st_sound_model_conf_levels *sm_levels = nullptr;
    struct st_sound_model_conf_levels_v2 *sm_levels_v2 = nullptr;

    PAL_VERBOSE(LOG_TAG, "Enter");

    /*  Example: Say the recognition structure has 3 keywords with users
     *  |kid|
     *  [0] k1 |uid|
     *         [3] u1 - 1st trainer
     *         [4] u2 - 4th trainer
     *         [6] u3 - 3rd trainer
     *  [1] k2
     *         [5] u2 - 2nd trainer
     *         [7] u3 - 5th trainer
     *  [2] k3
     *         [8] u4 - 6th trainer
     *
     *  Output confidence level array will be
     *  [k1, k2, k3, u1k1, u2k1, u2k2, u3k1, u3k2, u4k3]
     */

    if (version != CONF_LEVELS_INTF_VERSION_0002) {
        sm_levels = (struct st_sound_model_conf_levels *)sm_levels_generic;
        if (!sm_levels) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "ERROR. Invalid inputs");
            goto exit;
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            num_conf_levels++;
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++)
                num_conf_levels++;
        }

        if (!num_conf_levels) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "ERROR. Invalid num_conf_levels input");
            goto exit;
        }

        conf_levels = (unsigned char*)calloc(1, num_conf_levels);
        if (!conf_levels) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
            goto exit;
        }

        user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
        if (!user_id_tracker) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                    status);
            goto exit;
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            PAL_ERR(LOG_TAG, "[%d] kw level %d", i,
                sm_levels->kw_levels[i].kw_level);
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++) {
                PAL_ERR(LOG_TAG, "[%d] user_id %d level %d ", i,
                    sm_levels->kw_levels[i].user_levels[j].user_id,
                    sm_levels->kw_levels[i].user_levels[j].level);
            }
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            if (i < num_conf_levels) {
                conf_levels[i] = sm_levels->kw_levels[i].kw_level;
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "ERROR. Invalid numver of kw levels");
                goto exit;
            }
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++) {
                user_level = sm_levels->kw_levels[i].user_levels[j].level;
                user_id = sm_levels->kw_levels[i].user_levels[j].user_id;
                if ((user_id < sm_levels->num_kw_levels) ||
                    (user_id >= num_conf_levels)) {
                    status = -EINVAL;
                    PAL_ERR(LOG_TAG, "ERROR. Invalid params user id %d>%d",
                        user_id, num_conf_levels);
                    goto exit;
                } else {
                    if (user_id_tracker[user_id] == 1) {
                        status = -EINVAL;
                        PAL_ERR(LOG_TAG, "ERROR. Duplicate user id %d",
                            user_id);
                        goto exit;
                    }
                    conf_levels[user_id] = (user_level < 100) ?
                                           user_level: 100;
                    user_id_tracker[user_id] = 1;
                    PAL_ERR(LOG_TAG, "user_conf_levels[%d] = %d",
                        user_id, conf_levels[user_id]);
                }
            }
        }
    } else {
        sm_levels_v2 =
            (struct st_sound_model_conf_levels_v2 *)sm_levels_generic;
        if (!sm_levels_v2) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "ERROR. Invalid inputs");
            goto exit;
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            num_conf_levels++;
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++)
                num_conf_levels++;
        }

        if (!num_conf_levels) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "ERROR. Invalid num_conf_levels input");
            goto exit;
        }

        conf_levels = (unsigned char*)calloc(1, num_conf_levels);
        if (!conf_levels) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
            goto exit;
        }

        user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
        if (!user_id_tracker) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                    status);
            goto exit;
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            PAL_VERBOSE(LOG_TAG, "[%d] kw level %d", i,
                sm_levels_v2->kw_levels[i].kw_level);
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++) {
                PAL_VERBOSE(LOG_TAG, "[%d] user_id %d level %d ", i,
                     sm_levels_v2->kw_levels[i].user_levels[j].user_id,
                     sm_levels_v2->kw_levels[i].user_levels[j].level);
            }
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            if (i < num_conf_levels) {
                conf_levels[i] = sm_levels_v2->kw_levels[i].kw_level;
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "ERROR. Invalid numver of kw levels");
                goto exit;
            }
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++) {
                user_level = sm_levels_v2->kw_levels[i].user_levels[j].level;
                user_id = sm_levels_v2->kw_levels[i].user_levels[j].user_id;
                if ((user_id < sm_levels_v2->num_kw_levels) ||
                    (user_id >= num_conf_levels)) {
                    status = -EINVAL;
                    PAL_ERR(LOG_TAG, "ERROR. Invalid params user id %d>%d",
                         user_id, num_conf_levels);
                    goto exit;
                } else {
                    if (user_id_tracker[user_id] == 1) {
                        status = -EINVAL;
                        PAL_ERR(LOG_TAG, "ERROR. Duplicate user id %d",
                            user_id);
                        goto exit;
                    }
                    conf_levels[user_id] = (user_level < 100) ?
                                            user_level: 100;
                    user_id_tracker[user_id] = 1;
                    PAL_VERBOSE(LOG_TAG, "user_conf_levels[%d] = %d",
                        user_id, conf_levels[user_id]);
                }
            }
        }
    }

    *out_payload = conf_levels;
    *out_payload_size = num_conf_levels;
exit:
    if (user_id_tracker)
        free(user_id_tracker);

    return status;
}

void StreamSoundTrigger::SetDetectedToEngines(bool detected) {
    for (auto& eng: engines_) {
        if (eng->GetEngineId() != ST_SM_ID_SVA_GMM) {
            PAL_VERBOSE(LOG_TAG, "Notify detection event %d to engine %d",
                    detected, eng->GetEngineId());
            eng->GetEngine()->SetDetected(detected);
        }
    }
}

pal_device_id_t StreamSoundTrigger::GetAvailCaptureDevice()
{
    if (st_info_->GetSupportDevSwitch() &&
        rm->isDeviceAvailable(PAL_DEVICE_IN_WIRED_HEADSET))
        return PAL_DEVICE_IN_HEADSET_VA_MIC;
    else
        return PAL_DEVICE_IN_HANDSET_VA_MIC;
}

void StreamSoundTrigger::AddEngine(std::shared_ptr<EngineCfg> engine_cfg) {
    for (int32_t i = 0; i < engines_.size(); i++) {
        if (engines_[i] == engine_cfg) {
            PAL_VERBOSE(LOG_TAG, "engine type %d already exists",
                        engine_cfg->id_);
            return;
        }
    }
    PAL_VERBOSE(LOG_TAG, "Add engine %d, gsl_engine %p", engine_cfg->id_,
                gsl_engine_.get());
    engines_.push_back(engine_cfg);
}

std::shared_ptr<CaptureProfile> StreamSoundTrigger::GetCurrentCaptureProfile() {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    bool is_transit_to_nlpi = false;

    is_transit_to_nlpi = rm->CheckForForcedTransitToNonLPI();

    if (GetAvailCaptureDevice() == PAL_DEVICE_IN_HEADSET_VA_MIC) {
        if (is_transit_to_nlpi) {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_HIGH_PERF_AND_CHARGING,
                    ST_INPUT_MODE_HEADSET));
        } else if (use_lpi_) {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_LOW_POWER,
                    ST_INPUT_MODE_HEADSET));
        } else {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_HIGH_PERF,
                    ST_INPUT_MODE_HEADSET));
        }
    } else {
        if (is_transit_to_nlpi) {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_HIGH_PERF_AND_CHARGING,
                    ST_INPUT_MODE_HANDSET));
        } else if (use_lpi_) {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_LOW_POWER,
                    ST_INPUT_MODE_HANDSET));
        } else {
            cap_prof = sm_info_->GetCaptureProfile(
                std::make_pair(ST_OPERATING_MODE_HIGH_PERF,
                    ST_INPUT_MODE_HANDSET));
        }
    }

    PAL_DBG(LOG_TAG, "cap_prof %s: dev_id=0x%x, chs=%d, sr=%d, snd_name=%s",
        cap_prof->GetName().c_str(), cap_prof->GetDevId(),
        cap_prof->GetChannels(), cap_prof->GetSampleRate(),
        cap_prof->GetSndName().c_str());

    return cap_prof;
}

uint32_t StreamSoundTrigger::GetKwStartTolerance() {
    uint32_t start_tolerance = 0;

    if (sm_info_) {
        start_tolerance = sm_info_->GetKwStartTolerance() * 1000;
    }

    PAL_DBG(LOG_TAG, "start tolerance %u us", start_tolerance);
    return start_tolerance;
}

uint32_t StreamSoundTrigger::GetKwEndTolerance() {
    uint32_t end_tolerance = 0;

    if (sm_info_) {
        end_tolerance = sm_info_->GetKwEndTolerance() * 1000;
    }

    PAL_DBG(LOG_TAG, "end tolerance %u us", end_tolerance);
    return end_tolerance;
}

int32_t StreamSoundTrigger::GetEngineConfig(uint32_t &sample_rate,
    uint32_t &bit_width, uint32_t &channels, listen_model_indicator_enum type) {
    uint32_t sm_id = static_cast<uint32_t>(type);

    if (sm_info_) {
        if (type == ST_SM_ID_SVA_GMM) {
            sample_rate = sm_info_->GetSampleRate();
            bit_width = sm_info_->GetBitWidth();
            channels = sm_info_->GetOutChannels();
        } else if (type == ST_SM_ID_SVA_CNN ||
                    type == ST_SM_ID_SVA_VOP ||
                    type == ST_SM_ID_SVA_RNN) {
            auto ss_config = sm_info_->GetSecondStageConfig(sm_id);
            if (!ss_config) {
                PAL_ERR(LOG_TAG, "No matching second stage config");
                return -EINVAL;
            }
            sample_rate = ss_config->GetSampleRate();
            bit_width = ss_config->GetBitWidth();
            channels = ss_config->GetChannels();
        }
    } else {
        PAL_ERR(LOG_TAG, "No sound model info present");
        return -EINVAL;
    }

    return 0;
}

int32_t StreamSoundTrigger::GetSecondStageConfig(
    st_sound_model_type_t &detection_type,
    std::string &lib_name,
    listen_model_indicator_enum type) {
    uint32_t sm_id = static_cast<uint32_t>(type);

    if (sm_info_) {
        if (type == ST_SM_ID_SVA_CNN ||
            type == ST_SM_ID_SVA_VOP ||
            type == ST_SM_ID_SVA_RNN) {
            auto ss_config = sm_info_->GetSecondStageConfig(sm_id);
            if (!ss_config) {
                PAL_ERR(LOG_TAG, "No matching second stage config");
                return -EINVAL;
            }
            detection_type = ss_config->GetDetectionType();
            lib_name = ss_config->GetLibName();
        }
    } else {
        PAL_ERR(LOG_TAG, "No sound model info present");
        return -EINVAL;
    }

    return 0;
}

void StreamSoundTrigger::AddState(StState* state) {
   st_states_.insert(std::make_pair(state->GetStateId(), state));
}

int32_t StreamSoundTrigger::GetCurrentStateId() {
    if (cur_state_)
        return cur_state_->GetStateId();

    return ST_STATE_NONE;
}

int32_t StreamSoundTrigger::GetPreviousStateId() {
    if (prev_state_)
        return prev_state_->GetStateId();

    return ST_STATE_NONE;
}

void StreamSoundTrigger::TransitTo(int32_t state_id) {
    auto it = st_states_.find(state_id);
    if (it == st_states_.end()) {
        PAL_ERR(LOG_TAG, "Unknown transit state %d ", state_id);
        return;
    }
    prev_state_ = cur_state_;
    cur_state_ = it->second;
    auto oldState = stStateNameMap.at(prev_state_->GetStateId());
    auto newState = stStateNameMap.at(it->first);
    PAL_DBG(LOG_TAG, "Stream instance %u: state transitioned from %s to %s",
            mInstanceID, oldState.c_str(), newState.c_str());
}

int32_t StreamSoundTrigger::ProcessInternalEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {
    return cur_state_->ProcessEvent(ev_cfg);
}

int32_t StreamSoundTrigger::StIdle::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StIdle: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_LOAD_SOUND_MODEL: {
            std::shared_ptr<CaptureProfile> cap_prof = nullptr;
            StLoadEventConfigData *data =
                (StLoadEventConfigData *)ev_cfg->data_.get();
            class SoundTriggerUUID uuid;
            struct pal_st_sound_model * pal_st_sm;
            std::pair<int32_t,int32_t> streamConfigKV;

            pal_st_sm = (struct pal_st_sound_model *)data->data_;

            uuid.timeLow = (uint32_t)pal_st_sm->vendor_uuid.timeLow;
            uuid.timeMid = (uint16_t)pal_st_sm->vendor_uuid.timeMid;
            uuid.timeHiAndVersion = (uint16_t)pal_st_sm->vendor_uuid.timeHiAndVersion;
            uuid.clockSeq = (uint16_t)pal_st_sm->vendor_uuid.clockSeq;
            uuid.node[0] = (uint8_t)pal_st_sm->vendor_uuid.node[0];
            uuid.node[1] = (uint8_t)pal_st_sm->vendor_uuid.node[1];
            uuid.node[2] = (uint8_t)pal_st_sm->vendor_uuid.node[2];
            uuid.node[3] = (uint8_t)pal_st_sm->vendor_uuid.node[3];
            uuid.node[4] = (uint8_t)pal_st_sm->vendor_uuid.node[4];
            uuid.node[5] = (uint8_t)pal_st_sm->vendor_uuid.node[5];

            PAL_INFO(LOG_TAG, "Input vendor uuid : %08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
                        uuid.timeLow,
                        uuid.timeMid,
                        uuid.timeHiAndVersion,
                        uuid.clockSeq,
                        uuid.node[0],
                        uuid.node[1],
                        uuid.node[2],
                        uuid.node[3],
                        uuid.node[4],
                        uuid.node[5]);

            st_stream_.sm_info_ = st_stream_.st_info_->GetSmConfig(uuid);

            if (!st_stream_.sm_info_) {
                PAL_ERR(LOG_TAG, "Failed to get sound model platform info");
                throw std::runtime_error("Failed to get sound model platform info");
            }

            if (!st_stream_.mDevices.size()) {
                struct pal_device* dattr = new (struct pal_device);
                std::shared_ptr<Device> dev = nullptr;

                // update best device
                pal_device_id_t dev_id = st_stream_.GetAvailCaptureDevice();
                PAL_DBG(LOG_TAG, "Select available caputre device %d", dev_id);

                dev = st_stream_.GetPalDevice(dev_id, dattr, false);
                if (!dev) {
                    PAL_ERR(LOG_TAG, "Device creation is failed");
                    free(st_stream_.mStreamAttr);
                    throw std::runtime_error("failed to create device object");
                }
                st_stream_.mDevices.push_back(dev);
                dev = nullptr;
                delete dattr;
            }
            if (st_stream_.mDevices.size() > 0) {
                status = st_stream_.mDevices[0]->open();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Device open failed, status %d", status);
                    goto err_exit;
                }
            }

            cap_prof = st_stream_.GetCurrentCaptureProfile();
            st_stream_.cap_prof_ = cap_prof;
            /* store the pre-proc KV selected in the config file */
            st_stream_.mDevPpModifiers.clear();
            st_stream_.mDevPpModifiers.push_back(
                st_stream_.cap_prof_->GetDevicePpKv());

            streamConfigKV = st_stream_.sm_info_->GetStreamConfig();
            st_stream_.mStreamModifiers.clear();
            st_stream_.mStreamModifiers.push_back(streamConfigKV);

            st_stream_.mInstanceID = st_stream_.rm->getStreamInstanceID(&st_stream_);

            status = st_stream_.LoadSoundModel(pal_st_sm);

            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to load sm, status %d", status);
                goto err_exit;
            } else {
                PAL_VERBOSE(LOG_TAG, "Opened the engine and dev successfully");
                TransitTo(ST_STATE_LOADED);
                break;
            }
        err_exit:
            break;
        }
        case ST_EV_PAUSE: {
            st_stream_.paused_ = true;
            break;
        }
        case ST_EV_RESUME: {
            st_stream_.paused_ = false;
            break;
        }
        case ST_EV_READ_BUFFER: {
            status = -EIO;
            break;
        }
        case ST_EV_DEVICE_DISCONNECTED: {
            StDeviceDisconnectedEventConfigData *data =
                (StDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;
            if (st_stream_.mDevices.size() == 0) {
                PAL_DBG(LOG_TAG, "No device to disconnect");
                break;
            } else {
                int curr_device_id = st_stream_.mDevices[0]->getSndDeviceId();
                pal_device_id_t curr_device =
                    static_cast<pal_device_id_t>(curr_device_id);
                if (curr_device != device_id) {
                    PAL_ERR(LOG_TAG, "Device %d not connected, ignore",
                        device_id);
                    break;
                }
            }
            st_stream_.mDevices.clear();
            break;
        }
        case ST_EV_DEVICE_CONNECTED: {
            struct pal_device *pal_dev = new struct pal_device;
            std::shared_ptr<Device> dev = nullptr;
            StDeviceConnectedEventConfigData *data =
                (StDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;

            // mDevices should be empty as we have just disconnected device
            if (st_stream_.mDevices.size() != 0) {
                PAL_ERR(LOG_TAG, "Invalid operation");
                status = -EINVAL;
                goto connect_err;
            }

            dev = st_stream_.GetPalDevice(dev_id, pal_dev, false);
            if (!dev) {
                PAL_ERR(LOG_TAG, "Device creation failed");
                status = -EINVAL;
                goto connect_err;
            }

            st_stream_.mDevices.push_back(dev);
        connect_err:
            delete pal_dev;
            break;
        }
        case ST_EV_CONCURRENT_STREAM:
        case ST_EV_CHARGING_STATE: {
            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            if (ev_cfg->id_ == ST_EV_CONCURRENT_STREAM) {
                StConcurrentStreamEventConfigData *data =
                    (StConcurrentStreamEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            } else if (ev_cfg->id_ == ST_EV_CHARGING_STATE) {
                StChargingStateEventConfigData *data =
                    (StChargingStateEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            }
            new_cap_prof = st_stream_.GetCurrentCaptureProfile();
            if (st_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    st_stream_.cap_prof_->GetName().c_str(),
                    st_stream_.cap_prof_->GetDevId(),
                    st_stream_.cap_prof_->GetChannels(),
                    st_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (active) {
                    if (st_stream_.sm_config_) {
                        std::shared_ptr<StEventConfig> ev_cfg1(
                            new StLoadEventConfig(st_stream_.sm_config_));
                        status = st_stream_.ProcessInternalEvent(ev_cfg1);
                        if (status) {
                            PAL_ERR(LOG_TAG, "Failed to load, status %d", status);
                            break;
                        }
                    }
                    if (st_stream_.rec_config_) {
                        status = st_stream_.SendRecognitionConfig(
                            st_stream_.rec_config_);
                        if (0 != status) {
                            PAL_ERR(LOG_TAG, "Failed to send recognition config, status %d",
                                    status);
                            break;
                        }
                    }
                    if (st_stream_.isActive()) {
                        std::shared_ptr<StEventConfig> ev_cfg2(
                            new StStartRecognitionEventConfig(false));
                        status = st_stream_.ProcessInternalEvent(ev_cfg2);
                        if (status) {
                            PAL_ERR(LOG_TAG, "Failed to Start, status %d", status);
                        }
                    }
                }
            } else {
                PAL_INFO(LOG_TAG,"no action needed, same capture profile");
            }
            break;
        }
        case ST_EV_SSR_OFFLINE:
            if (st_stream_.state_for_restore_ == ST_STATE_NONE) {
                st_stream_.state_for_restore_ = ST_STATE_IDLE;
            }
            TransitTo(ST_STATE_SSR);
            break;
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }

    return status;
}

int32_t StreamSoundTrigger::StLoaded::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StLoaded: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_UNLOAD_SOUND_MODEL: {
            int ret = 0;
            std::pair<uint32_t,uint32_t> streamConfigKV;

            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Close device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                ret = dev->close();
                if (0 != ret) {
                    PAL_ERR(LOG_TAG, "Device open failed, status %d", status);
                    status = ret;
                }
            }

            for (auto& eng: st_stream_.engines_) {
                PAL_DBG(LOG_TAG, "Unload engine %d", eng->GetEngineId());
                status = eng->GetEngine()->UnloadSoundModel(&st_stream_);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Unload engine %d failed, status %d",
                            eng->GetEngineId(), status);
                    status = ret;
                }
                free(eng->sm_data_);
            }
            if (st_stream_.reader_) {
                delete st_stream_.reader_;
                st_stream_.reader_ = nullptr;
            }
            st_stream_.engines_.clear();

            streamConfigKV = st_stream_.sm_info_->GetStreamConfig();

            st_stream_.rm->resetStreamInstanceID(
                &st_stream_,
                st_stream_.mInstanceID);
            st_stream_.notification_state_ = ENGINE_IDLE;

            TransitTo(ST_STATE_IDLE);
            break;
        }
        case ST_EV_RECOGNITION_CONFIG: {
            StRecognitionCfgEventConfigData *data =
                (StRecognitionCfgEventConfigData *)ev_cfg->data_.get();
            status = st_stream_.SendRecognitionConfig(
               (struct pal_st_recognition_config *)data->data_);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to send recog config, status %d",
                        status);
            }
            break;
        }
        case ST_EV_RESUME: {
            if (!st_stream_.paused_) {
                // Possible if App has stopped recognition during active
                // concurrency.
                break;
            }
            st_stream_.paused_ = false;
            // fall through to start
            [[fallthrough]];
        }
        case ST_EV_START_RECOGNITION: {
            if (st_stream_.paused_) {
               break; // Concurrency is active, start later.
            }
            StStartRecognitionEventConfigData *data =
                (StStartRecognitionEventConfigData *)ev_cfg->data_.get();
            if (!st_stream_.rec_config_) {
                PAL_ERR(LOG_TAG, "Recognition config not set %d", data->restart_);
                status = -EINVAL;
                break;
            }

            /* Update cap dev based on mode and configuration and start it */
            struct pal_device dattr;
            bool backend_update = false;
            std::vector<std::shared_ptr<SoundTriggerEngine>> tmp_engines;
            std::shared_ptr<CaptureProfile> cap_prof = nullptr;

            // Do not update capture profile when resuming stream
            if (ev_cfg->id_ == ST_EV_START_RECOGNITION) {
                backend_update = st_stream_.rm->UpdateSVACaptureProfile(
                    &st_stream_, true);
                if (backend_update) {
                    status = rm->StopOtherSVAStreams(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to stop other SVA streams");
                    }

                    status = rm->StartOtherSVAStreams(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to start other SVA streams");
                    }
                }
            }

            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                dev->getDeviceAttributes(&dattr);

                cap_prof = st_stream_.rm->GetSVACaptureProfile();
                if (!cap_prof) {
                    PAL_ERR(LOG_TAG, "Invalid capture profile");
                    goto err_exit;
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
                    PAL_ERR(LOG_TAG, "Device start failed, status %d", status);
                    break;
                } else {
                    st_stream_.rm->registerDevice(dev, &st_stream_);
                }
                PAL_DBG(LOG_TAG, "device started");
            }

            /* Start the engines */
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Start st engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StartRecognition(&st_stream_);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Start st engine %d failed, status %d",
                            eng->GetEngineId(), status);
                    goto err_exit;
                } else {
                    tmp_engines.push_back(eng->GetEngine());
                }
            }

            if (st_stream_.reader_)
                st_stream_.reader_->reset();

            TransitTo(ST_STATE_ACTIVE);
            break;

        err_exit:
            for (auto& eng: tmp_engines)
                eng->StopRecognition(&st_stream_);

            if (st_stream_.mDevices.size() > 0) {
                st_stream_.rm->deregisterDevice(st_stream_.mDevices[0], &st_stream_);
                st_stream_.mDevices[0]->stop();
            }

            break;
        }
        case ST_EV_PAUSE: {
            st_stream_.paused_ = true;
            break;
        }
        case ST_EV_STOP_RECOGNITION: {
            // Possible if client is stopping during active concurrency.
            // Reset puase flag to avoid restarting when concurrency inactive.
            st_stream_.paused_ = false;
            break;
        }
        case ST_EV_READ_BUFFER: {
            status = -EIO;
            break;
        }
        case ST_EV_DEVICE_DISCONNECTED:{
            StDeviceDisconnectedEventConfigData *data =
                (StDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;
            if (st_stream_.mDevices.size() == 0) {
                PAL_DBG(LOG_TAG, "No device to disconnect");
                break;
            } else {
                int curr_device_id = st_stream_.mDevices[0]->getSndDeviceId();
                pal_device_id_t curr_device =
                    static_cast<pal_device_id_t>(curr_device_id);
                if (curr_device != device_id) {
                    PAL_ERR(LOG_TAG, "Device %d not connected, ignore",
                        device_id);
                    break;
                }
            }
            for (auto& device: st_stream_.mDevices) {
                st_stream_.gsl_engine_->DisconnectSessionDevice(&st_stream_,
                    st_stream_.mStreamAttr->type, device);

                status = device->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "dev close failed, status %d", status);
                }
            }
            st_stream_.mDevices.clear();
            break;
        }
        case ST_EV_DEVICE_CONNECTED: {
            struct pal_device *pal_dev = new struct pal_device;
            std::shared_ptr<Device> dev = nullptr;
            StDeviceConnectedEventConfigData *data =
                (StDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;

            // mDevices should be empty as we have just disconnected device
            if (st_stream_.mDevices.size() != 0) {
                PAL_ERR(LOG_TAG, "Invalid operation");
                status = -EINVAL;
                goto connect_err;
            }

            dev = st_stream_.GetPalDevice(dev_id, pal_dev, false);
            if (!dev) {
                PAL_ERR(LOG_TAG, "Dev creation failed");
                status = -EINVAL;
                goto connect_err;
            }
            st_stream_.mDevices.push_back(dev);

            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device %d open failed with status %d",
                    dev->getSndDeviceId(), status);
                goto connect_err;
            }

            status = st_stream_.gsl_engine_->SetupSessionDevice(&st_stream_,
                st_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG,
                        "setupSessionDevice for %d failed with status %d",
                        dev->getSndDeviceId(), status);
                st_stream_.mDevices.pop_back();
                dev->close();
                goto connect_err;
            }

            if (st_stream_.isActive()) {
                status = dev->start();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device %d start failed with status %d",
                        dev->getSndDeviceId(), status);
                    goto connect_err;
                }
                st_stream_.rm->registerDevice(dev, &st_stream_);
            }

            status = st_stream_.gsl_engine_->ConnectSessionDevice(&st_stream_,
                st_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG,
                        "connectSessionDevice for %d failed with status %d",
                        dev->getSndDeviceId(), status);
                st_stream_.mDevices.pop_back();
                dev->close();
            } else {
                PAL_DBG(LOG_TAG, "Update capture profile after device switch");
                st_stream_.cap_prof_ = st_stream_.GetCurrentCaptureProfile();
            }

        connect_err:
            delete pal_dev;
            break;
        }
        case ST_EV_CONCURRENT_STREAM:
        case ST_EV_CHARGING_STATE: {
            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            if (ev_cfg->id_ == ST_EV_CONCURRENT_STREAM) {
                StConcurrentStreamEventConfigData *data =
                    (StConcurrentStreamEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            } else if (ev_cfg->id_ == ST_EV_CHARGING_STATE) {
                StChargingStateEventConfigData *data =
                    (StChargingStateEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            }
            new_cap_prof = st_stream_.GetCurrentCaptureProfile();
            if (st_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    st_stream_.cap_prof_->GetName().c_str(),
                    st_stream_.cap_prof_->GetDevId(),
                    st_stream_.cap_prof_->GetChannels(),
                    st_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (!active) {
                    std::shared_ptr<StEventConfig> ev_cfg1(
                        new StUnloadEventConfig());
                    status = st_stream_.ProcessInternalEvent(ev_cfg1);
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
        case ST_EV_SSR_OFFLINE: {
            if (st_stream_.state_for_restore_ == ST_STATE_NONE) {
                st_stream_.state_for_restore_ = ST_STATE_LOADED;
            }
            std::shared_ptr<StEventConfig> ev_cfg(new StUnloadEventConfig());
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            TransitTo(ST_STATE_SSR);
            break;
        }
        case ST_EV_EC_REF: {
            StECRefEventConfigData *data =
                (StECRefEventConfigData *)ev_cfg->data_.get();
            Stream *s = static_cast<Stream *>(&st_stream_);
            status = st_stream_.gsl_engine_->setECRef(s, data->dev_,
                data->is_enable_);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set EC Ref in gsl engine");
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

int32_t StreamSoundTrigger::StActive::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StActive: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_DETECTED: {
            StDetectedEventConfigData *data =
                (StDetectedEventConfigData *) ev_cfg->data_.get();
            if (data->det_type_ != GMM_DETECTED)
                break;
            if (!st_stream_.rec_config_->capture_requested &&
                st_stream_.engines_.size() == 1) {
                TransitTo(ST_STATE_DETECTED);
                if (st_stream_.GetCurrentStateId() == ST_STATE_DETECTED) {
                    st_stream_.PostDelayedStop();
                }
            } else {
                TransitTo(ST_STATE_BUFFERING);
                st_stream_.SetDetectedToEngines(true);
            }
            if (st_stream_.engines_.size() == 1) {
                st_stream_.notifyClient();
            }
            break;
        }
        case ST_EV_PAUSE: {
            st_stream_.paused_ = true;
            // fall through to stop
            [[fallthrough]];
        }
        case ST_EV_STOP_RECOGNITION: {
            // Do not update capture profile when pausing stream
            if (ev_cfg->id_ == ST_EV_STOP_RECOGNITION) {
                bool backend_update = false;
                backend_update = st_stream_.rm->UpdateSVACaptureProfile(
                    &st_stream_, false);
                if (backend_update) {
                    status = rm->StopOtherSVAStreams(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to stop other SVA streams");
                    }

                    status = rm->StartOtherSVAStreams(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to start other SVA streams");
                    }
                }
            }

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            break;
        }
        case ST_EV_EC_REF: {
            StECRefEventConfigData *data =
                (StECRefEventConfigData *)ev_cfg->data_.get();
            Stream *s = static_cast<Stream *>(&st_stream_);
            status = st_stream_.gsl_engine_->setECRef(s, data->dev_,
                data->is_enable_);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set EC Ref in gsl engine");
            }
            break;
        }
        case ST_EV_READ_BUFFER: {
            status = -EIO;
            break;
        }
        case ST_EV_DEVICE_DISCONNECTED: {
            StDeviceDisconnectedEventConfigData *data =
                (StDeviceDisconnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t device_id = data->dev_id_;
            if (st_stream_.mDevices.size() == 0) {
                PAL_DBG(LOG_TAG, "No device to disconnect");
                break;
            } else {
                int curr_device_id = st_stream_.mDevices[0]->getSndDeviceId();
                pal_device_id_t curr_device =
                    static_cast<pal_device_id_t>(curr_device_id);
                if (curr_device != device_id) {
                    PAL_ERR(LOG_TAG, "Device %d not connected, ignore",
                        device_id);
                    break;
                }
            }
            for (auto& device: st_stream_.mDevices) {
                st_stream_.gsl_engine_->DisconnectSessionDevice(&st_stream_,
                    st_stream_.mStreamAttr->type, device);

                status = device->stop();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device stop failed with status %d", status);
                    goto disconnect_err;
                }

                st_stream_.rm->deregisterDevice(device, &st_stream_);

                status = device->close();
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "device close failed with status %d", status);
                    goto disconnect_err;
                }
            }
        disconnect_err:
            st_stream_.mDevices.clear();
            break;
        }
        case ST_EV_DEVICE_CONNECTED: {
            struct pal_device *pal_dev = new struct pal_device;
            std::shared_ptr<Device> dev = nullptr;
            StDeviceConnectedEventConfigData *data =
                (StDeviceConnectedEventConfigData *)ev_cfg->data_.get();
            pal_device_id_t dev_id = data->dev_id_;

            // mDevices should be empty as we have just disconnected device
            if (st_stream_.mDevices.size() != 0) {
                PAL_ERR(LOG_TAG, "Invalid operation");
                status = -EINVAL;
                goto connect_err;
            }

            dev = st_stream_.GetPalDevice(dev_id, pal_dev, false);
            if (!dev) {
                PAL_ERR(LOG_TAG, "Device creation failed");
                status = -EINVAL;
                goto connect_err;
            }
            st_stream_.mDevices.push_back(dev);

            status = dev->open();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device %d open failed with status %d",
                    dev->getSndDeviceId(), status);
                goto connect_err;
            }

            status = st_stream_.gsl_engine_->SetupSessionDevice(&st_stream_,
                st_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "setupSessionDevice for %d failed with status %d",
                        dev->getSndDeviceId(), status);
                st_stream_.mDevices.pop_back();
                dev->close();
                goto connect_err;
            }

            status = dev->start();
            if (0 != status) {
                PAL_ERR(LOG_TAG, "device %d start failed with status %d",
                    dev->getSndDeviceId(), status);
                goto connect_err;
            }
            st_stream_.rm->registerDevice(dev, &st_stream_);

            status = st_stream_.gsl_engine_->ConnectSessionDevice(&st_stream_,
                st_stream_.mStreamAttr->type, dev);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "connectSessionDevice for %d failed with status %d",
                        dev->getSndDeviceId(), status);
                st_stream_.mDevices.pop_back();
                dev->close();
            } else {
                PAL_DBG(LOG_TAG, "Update capture profile after device switch");
                st_stream_.cap_prof_ = st_stream_.GetCurrentCaptureProfile();
            }

        connect_err:
            delete pal_dev;
            break;
        }
        case ST_EV_CONCURRENT_STREAM:
        case ST_EV_CHARGING_STATE: {
            std::shared_ptr<CaptureProfile> new_cap_prof = nullptr;
            bool active = false;

            if (ev_cfg->id_ == ST_EV_CONCURRENT_STREAM) {
                StConcurrentStreamEventConfigData *data =
                    (StConcurrentStreamEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            } else if (ev_cfg->id_ == ST_EV_CHARGING_STATE) {
                StChargingStateEventConfigData *data =
                    (StChargingStateEventConfigData *)ev_cfg->data_.get();
                active = data->is_active_;
            }
            new_cap_prof = st_stream_.GetCurrentCaptureProfile();
            if (st_stream_.cap_prof_ != new_cap_prof) {
                PAL_DBG(LOG_TAG,
                    "current capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    st_stream_.cap_prof_->GetName().c_str(),
                    st_stream_.cap_prof_->GetDevId(),
                    st_stream_.cap_prof_->GetChannels(),
                    st_stream_.cap_prof_->GetSampleRate());
                PAL_DBG(LOG_TAG,
                    "new capture profile %s: dev_id=0x%x, chs=%d, sr=%d\n",
                    new_cap_prof->GetName().c_str(),
                    new_cap_prof->GetDevId(),
                    new_cap_prof->GetChannels(),
                    new_cap_prof->GetSampleRate());
                if (!active) {
                    std::shared_ptr<StEventConfig> ev_cfg1(
                        new StStopRecognitionEventConfig(false));
                    status = st_stream_.ProcessInternalEvent(ev_cfg1);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Failed to Stop, status %d", status);
                        break;
                    }
                    std::shared_ptr<StEventConfig> ev_cfg2(new StUnloadEventConfig());
                    status = st_stream_.ProcessInternalEvent(ev_cfg2);
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
        case ST_EV_SSR_OFFLINE: {
            if (st_stream_.state_for_restore_ == ST_STATE_NONE) {
                st_stream_.state_for_restore_ = ST_STATE_ACTIVE;
            }
            std::shared_ptr<StEventConfig> ev_cfg1(
                new StStopRecognitionEventConfig(false));
            status = st_stream_.ProcessInternalEvent(ev_cfg1);

            std::shared_ptr<StEventConfig> ev_cfg2(
                new StUnloadEventConfig());
            status = st_stream_.ProcessInternalEvent(ev_cfg2);
            TransitTo(ST_STATE_SSR);
            break;
        }
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }
    return status;
}

int32_t StreamSoundTrigger::StDetected::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StDetected: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_START_RECOGNITION: {
            // Client restarts next recognition without config changed.
            st_stream_.CancelDelayedStop();

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
                status = eng->GetEngine()->RestartRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Restart engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.reader_)
                st_stream_.reader_->reset();
            if (!status) {
                TransitTo(ST_STATE_ACTIVE);
            } else {
                TransitTo(ST_STATE_LOADED);
            }
            break;
        }
        case ST_EV_PAUSE: {
            st_stream_.CancelDelayedStop();
            st_stream_.paused_ = true;
            // fall through to stop
            [[fallthrough]];
        }
        case ST_EV_STOP_RECOGNITION: {
            st_stream_.CancelDelayedStop();
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            break;
        }
        case ST_EV_RECOGNITION_CONFIG: {
            /*
             * Client can update config for next recognition.
             * Get to loaded state as START event will start recognition.
             */
            st_stream_.CancelDelayedStop();

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to handle recognition config, status %d",
                        status);
            }
            // START event will be handled in loaded state.
            break;
        }

        case ST_EV_CONCURRENT_STREAM:
        case ST_EV_CHARGING_STATE: {
            st_stream_.CancelDelayedStop();
            // Reuse from Active state.
            TransitTo(ST_STATE_ACTIVE);
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to process CONCURRENT_STREAM event,"
                                 "status %d", status);
            }
            break;
        }
        case ST_EV_RESUME: {
            st_stream_.paused_ = false;
            break;
        }
        case ST_EV_DEVICE_DISCONNECTED:
        case ST_EV_DEVICE_CONNECTED: {
            st_stream_.CancelDelayedStop();
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to handle device connection, status %d",
                        status);
            }
            break;
        }
        case ST_EV_SSR_OFFLINE: {
            if (st_stream_.state_for_restore_ == ST_STATE_NONE) {
                st_stream_.state_for_restore_ = ST_STATE_LOADED;
            }
            std::shared_ptr<StEventConfig> ev_cfg1(
                new StStopRecognitionEventConfig(false));
            status = st_stream_.ProcessInternalEvent(ev_cfg1);

            std::shared_ptr<StEventConfig> ev_cfg2(
                new StUnloadEventConfig());
            status = st_stream_.ProcessInternalEvent(ev_cfg2);
            TransitTo(ST_STATE_SSR);
            break;
        }
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }
    return status;
}

int32_t StreamSoundTrigger::StBuffering::ProcessEvent(
   std::shared_ptr<StEventConfig> ev_cfg) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StBuffering: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_READ_BUFFER: {
            StReadBufferEventConfigData *data =
                (StReadBufferEventConfigData *)ev_cfg->data_.get();
            struct pal_buffer *buf = (struct pal_buffer *)data->data_;

            if (!st_stream_.reader_) {
                PAL_ERR(LOG_TAG, "no reader exists");
                status = -EINVAL;
                break;
            }
            status = st_stream_.reader_->read(buf->buffer, buf->size);
            break;
        }
        case ST_EV_STOP_BUFFERING: {
            PAL_DBG(LOG_TAG, "StBuffering: stop buffering");
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                            eng->GetEngineId());
                status = eng->GetEngine()->StopBuffering(&st_stream_);
                if (status){
                    PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.reader_) {
                st_stream_.reader_->reset();
            }
            st_stream_.PostDelayedStop();
            break;
        }
        case ST_EV_START_RECOGNITION: {
          /*
           * Can happen if client requests next recognition without any config
           * change with/without reading buffers after sending detection event.
           */
          StStartRecognitionEventConfigData *data =
              (StStartRecognitionEventConfigData *)ev_cfg->data_.get();
          PAL_DBG(LOG_TAG, "StBuffering: start recognition, is restart %d",
                  data->restart_);
          st_stream_.CancelDelayedStop();

          for (auto& eng: st_stream_.engines_) {
              PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                          eng->GetEngineId());
              status = eng->GetEngine()->StopBuffering(&st_stream_);
              if (status)
                  PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                          eng->GetEngineId(), status);
          }
          if (st_stream_.reader_)
              st_stream_.reader_->reset();

          for (auto& eng: st_stream_.engines_) {
              PAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
              status = eng->GetEngine()->RestartRecognition(&st_stream_);
              if (status) {
                  PAL_ERR(LOG_TAG, "Restart engine %d buffering failed, status %d",
                          eng->GetEngineId(), status);
                  break;
              }
          }
          if (!status) {
              TransitTo(ST_STATE_ACTIVE);
          } else {
              TransitTo(ST_STATE_LOADED);
          }
          break;
        }
        case ST_EV_RECOGNITION_CONFIG: {
            /*
             * Can happen if client doesn't read buffers after sending detection
             * event, but requests next recognition with config change.
             * Get to loaded state as START event will start the recognition.
             */
            st_stream_.CancelDelayedStop();

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                            eng->GetEngineId());
                status = eng->GetEngine()->StopBuffering(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                    // continue stopping recognition
                }
            }
            if (st_stream_.reader_) {
              st_stream_.reader_->reset();
            }

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to handle recognition config, status %d",
                        status);
            }
            // START event will be handled in loaded state.
            break;
        }
        case ST_EV_PAUSE: {
            st_stream_.paused_ = true;
            PAL_DBG(LOG_TAG, "StBuffering: Pause");
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                            eng->GetEngineId());
                status = eng->GetEngine()->StopBuffering(&st_stream_);
                if (status){
                    PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.reader_) {
                st_stream_.reader_->reset();
            }
            // fall through to stop
            [[fallthrough]];
        }
        case ST_EV_STOP_RECOGNITION:  {
            // Possible with deffered stop if client doesn't start next recognition.
            st_stream_.CancelDelayedStop();

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            break;
        }
        case ST_EV_DETECTED: {
            // Second stage detections fall here.
            StDetectedEventConfigData *data =
                (StDetectedEventConfigData *)ev_cfg->data_.get();
            if (data->det_type_ == GMM_DETECTED) {
                break;
            }
            // If second stage has rejected, stop buffering and restart recognition
            if (data->det_type_ == KEYWORD_DETECTION_REJECT ||
                data->det_type_ == USER_VERIFICATION_REJECT) {
                PAL_DBG(LOG_TAG, "Second stage rejected, type %d",
                        data->det_type_);
                st_stream_.detection_state_ = ENGINE_IDLE;
                for (auto& eng: st_stream_.engines_) {
                    PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                                eng->GetEngineId());
                    status = eng->GetEngine()->StopBuffering(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed,"
                                         "status %d", eng->GetEngineId(), status);
                        // Continue to restart
                    }
                }
                if (st_stream_.reader_) {
                    st_stream_.reader_->reset();
                }
                for (auto& eng: st_stream_.engines_) {
                    PAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
                    status = eng->GetEngine()->RestartRecognition(&st_stream_);
                    if (status) {
                        PAL_ERR(LOG_TAG, "Restart engine %d failed, status %d",
                                eng->GetEngineId(), status);
                        break;
                    }
                }
                if (!status) {
                    TransitTo(ST_STATE_ACTIVE);
                } else {
                    TransitTo(ST_STATE_LOADED);
                }
                break;
            }
            if (data->det_type_ == KEYWORD_DETECTION_SUCCESS ||
                data->det_type_ == USER_VERIFICATION_SUCCESS) {
                st_stream_.detection_state_ |=  data->det_type_;
            }
            // notify client until both keyword detection/user verification done
            if (st_stream_.detection_state_ == st_stream_.notification_state_) {
                PAL_DBG(LOG_TAG, "Second stage detected");
                st_stream_.detection_state_ = ENGINE_IDLE;
                if (!st_stream_.rec_config_->capture_requested) {
                    for (auto& eng: st_stream_.engines_) {
                        PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                                    eng->GetEngineId());
                        status = eng->GetEngine()->StopBuffering(&st_stream_);
                        if (status) {
                            PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed,"
                                    "status %d", eng->GetEngineId(), status);
                        }
                    }
                    if (st_stream_.reader_) {
                        st_stream_.reader_->reset();
                    }
                    TransitTo(ST_STATE_DETECTED);
                }
                st_stream_.notifyClient();
                if (!st_stream_.rec_config_->capture_requested &&
                    (st_stream_.GetCurrentStateId() == ST_STATE_BUFFERING ||
                     st_stream_.GetCurrentStateId() == ST_STATE_DETECTED)) {
                    st_stream_.PostDelayedStop();
                }
            }
            break;
        }
        case ST_EV_CHARGING_STATE:
        case ST_EV_CONCURRENT_STREAM: {
            st_stream_.CancelDelayedStop();
            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                            eng->GetEngineId());
                status = eng->GetEngine()->StopBuffering(&st_stream_);
                if (status)
                    PAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                            eng->GetEngineId(), status);
            }
            if (st_stream_.reader_)
                st_stream_.reader_->reset();
            // Reuse from Active state.
            TransitTo(ST_STATE_ACTIVE);
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to process CONCURRENT_STREAM event,"
                                 "status %d", status);
            }
            break;
        }
        case ST_EV_DEVICE_DISCONNECTED:
        case ST_EV_DEVICE_CONNECTED: {
            st_stream_.CancelDelayedStop();

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                            eng->GetEngineId());
                status = eng->GetEngine()->StopBuffering(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.reader_) {
                st_stream_.reader_->reset();
            }

            for (auto& eng: st_stream_.engines_) {
                PAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
                status = eng->GetEngine()->StopRecognition(&st_stream_);
                if (status) {
                    PAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                            eng->GetEngineId(), status);
                }
            }
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                PAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                        dev->getPALDeviceName().c_str());
                status = dev->stop();
                if (status)
                    PAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
            }
            TransitTo(ST_STATE_LOADED);
            // make sure disable ec ref handled in LOADED/ACTIVE state
            if (st_stream_.mDevices.size() > 0) {
                auto& dev = st_stream_.mDevices[0];
                st_stream_.rm->deregisterDevice(dev, &st_stream_);
            }
            status = st_stream_.ProcessInternalEvent(ev_cfg);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to handle device connection, status %d",
                        status);
            }
            // device connection event will be handled in loaded state.
            break;
        }
        case ST_EV_SSR_OFFLINE: {
            if (st_stream_.state_for_restore_ == ST_STATE_NONE) {
                st_stream_.state_for_restore_ = ST_STATE_LOADED;
            }
            std::shared_ptr<StEventConfig> ev_cfg1(
                new StStopBufferingEventConfig());
            status = st_stream_.ProcessInternalEvent(ev_cfg1);

            std::shared_ptr<StEventConfig> ev_cfg2(
                new StStopRecognitionEventConfig(false));
            status = st_stream_.ProcessInternalEvent(ev_cfg2);

            std::shared_ptr<StEventConfig> ev_cfg3(
                new StUnloadEventConfig());
            status = st_stream_.ProcessInternalEvent(ev_cfg3);
            TransitTo(ST_STATE_SSR);
            break;
        }
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }
    return status;
}

int32_t StreamSoundTrigger::StSSR::ProcessEvent(
   std::shared_ptr<StEventConfig> ev_cfg) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "StSSR: handle event %d for stream instance %u",
        ev_cfg->id_, st_stream_.mInstanceID);

    switch (ev_cfg->id_) {
        case ST_EV_SSR_ONLINE: {
            TransitTo(ST_STATE_IDLE);

            if (st_stream_.state_for_restore_ == ST_STATE_LOADED ||
                st_stream_.state_for_restore_ == ST_STATE_ACTIVE) {
                std::shared_ptr<StEventConfig> ev_cfg1(
                    new StLoadEventConfig(st_stream_.sm_config_));
                status = st_stream_.ProcessInternalEvent(ev_cfg1);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Failed to load sound model, status %d",
                        status);
                    break;
                }
            }

            if (st_stream_.state_for_restore_ == ST_STATE_ACTIVE) {
                status = st_stream_.SendRecognitionConfig(
                    st_stream_.rec_config_);
                if (0 != status) {
                    PAL_ERR(LOG_TAG,
                        "Failed to send recognition config, status %d", status);
                    break;
                }
                std::shared_ptr<StEventConfig> ev_cfg2(
                    new StStartRecognitionEventConfig(false));
                status = st_stream_.ProcessInternalEvent(ev_cfg2);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Failed to Start, status %d", status);
                    break;
                }
            }
            PAL_DBG(LOG_TAG, "StSSR: event %d handled", ev_cfg->id_);
            st_stream_.state_for_restore_ = ST_STATE_NONE;
            break;
        }
        case ST_EV_LOAD_SOUND_MODEL: {
            if (st_stream_.state_for_restore_ != ST_STATE_IDLE) {
                PAL_ERR(LOG_TAG, "Invalid operation, client state = %d now",
                    st_stream_.state_for_restore_);
                status = -EINVAL;
            } else {
                StLoadEventConfigData *data =
                    (StLoadEventConfigData *)ev_cfg->data_.get();
                status = st_stream_.UpdateSoundModel(
                    (struct pal_st_sound_model *)data->data_);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Failed to update sound model, status %d",
                        status);
                } else {
                    st_stream_.state_for_restore_ = ST_STATE_LOADED;
                }
            }
            break;
        }
        case ST_EV_UNLOAD_SOUND_MODEL: {
            if (st_stream_.state_for_restore_ != ST_STATE_LOADED) {
                PAL_ERR(LOG_TAG, "Invalid operation, client state = %d now",
                    st_stream_.state_for_restore_);
                status = -EINVAL;
            } else {
                st_stream_.state_for_restore_ = ST_STATE_IDLE;
            }
            break;
        }
        case ST_EV_RECOGNITION_CONFIG: {
            if (st_stream_.state_for_restore_ != ST_STATE_LOADED) {
                PAL_ERR(LOG_TAG, "Invalid operation, client state = %d now",
                    st_stream_.state_for_restore_);
                status = -EINVAL;
            } else {
                StRecognitionCfgEventConfigData *data =
                    (StRecognitionCfgEventConfigData *)ev_cfg->data_.get();
                status = st_stream_.UpdateRecognitionConfig(
                    (struct pal_st_recognition_config *)data->data_);
                if (0 != status) {
                    PAL_ERR(LOG_TAG, "Failed to update recognition config,"
                        "status %d", status);
                }
            }
            break;
        }
        case ST_EV_START_RECOGNITION: {
            if (st_stream_.state_for_restore_ != ST_STATE_LOADED) {
                PAL_ERR(LOG_TAG, "Invalid operation, client state = %d now",
                    st_stream_.state_for_restore_);
                status = -EINVAL;
            } else {
                StStartRecognitionEventConfigData *data =
                    (StStartRecognitionEventConfigData *)ev_cfg->data_.get();
                if (!st_stream_.rec_config_) {
                    PAL_ERR(LOG_TAG, "Recognition config not set %d", data->restart_);
                    status = -EINVAL;
                    break;
                }
                st_stream_.state_for_restore_ = ST_STATE_ACTIVE;
            }
            break;
        }
        case ST_EV_STOP_RECOGNITION: {
            if (st_stream_.state_for_restore_ != ST_STATE_ACTIVE) {
                PAL_ERR(LOG_TAG, "Invalid operation, client state = %d now",
                    st_stream_.state_for_restore_);
                status = -EINVAL;
            } else {
                st_stream_.state_for_restore_ = ST_STATE_LOADED;
            }
            break;
        }
        case ST_EV_READ_BUFFER:
            status = -EIO;
            break;
        default: {
            PAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
            break;
        }
    }

    return status;
}

int32_t StreamSoundTrigger::ssrDownHandler() {
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(new StSSROfflineConfig());
    status = cur_state_->ProcessEvent(ev_cfg);

    return status;
}

int32_t StreamSoundTrigger::ssrUpHandler() {
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(new StSSROnlineConfig());
    status = cur_state_->ProcessEvent(ev_cfg);

    return status;
}
