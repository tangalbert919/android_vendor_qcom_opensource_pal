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

#define LOG_TAG "StreamSoundTrigger"

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
#define QAL_DBG(...)  QAL_INFO(__VA_ARGS__)
#endif

#define ST_DEFERRED_STOP_DEALY_MS (1000)

StreamSoundTrigger::StreamSoundTrigger(struct qal_stream_attributes *sattr,
                                       struct qal_device *dattr,
                                       uint32_t no_of_devices,
                                       struct modifier_kv *modifiers __unused,
                                       uint32_t no_of_modifiers __unused,
                                       std::shared_ptr<ResourceManager> rm) {
    std::shared_ptr<Device> dev = nullptr;
    struct qal_ec_info ecinfo = {};
    int status = 0;

    reader_ = nullptr;
    detection_state_ = ENGINE_IDLE;
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;
    sm_config_ = nullptr;
    rec_config_ = nullptr;
    paused_ = false;
    pending_stop_ = false;
    conc_tx_cnt_ = 0;

    QAL_DBG(LOG_TAG, "Enter");
    // TODO: handle modifiers later
    mNoOfModifiers = 0;
    mModifiers = (struct modifier_kv *) (nullptr);

    mStreamAttr = (struct qal_stream_attributes *)calloc(1,
        sizeof(struct qal_stream_attributes));
    if (!mStreamAttr) {
        QAL_ERR(LOG_TAG, "stream attributes allocation failed");
        throw std::runtime_error("stream attributes allocation failed");
    }

    struct qal_channel_info *ch_info = (struct qal_channel_info *)calloc(1,
        sizeof(struct qal_channel_info));
    if (!ch_info) {
        QAL_ERR(LOG_TAG, "channel info allocation failed");
        free(mStreamAttr);
        throw std::runtime_error("channel info allocation failed");
    }

    casa_osal_memcpy(mStreamAttr, sizeof(qal_stream_attributes),
                     sattr, sizeof(qal_stream_attributes));
    mStreamAttr->in_media_config.ch_info = ch_info;
    casa_osal_memcpy(mStreamAttr->in_media_config.ch_info,
                     sizeof(qal_channel_info),
                     sattr->in_media_config.ch_info,
                     sizeof(qal_channel_info));

    QAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d",
                no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        // update best device
        qal_device_id_t dev_id = GetAvailCaptureDevice();
        if (dattr[i].id != dev_id) {
            QAL_DBG(LOG_TAG, "Select available caputre device %d", dev_id);
            dattr[i].id = dev_id;
            if (dattr[i].config.ch_info)
                free(dattr[i].config.ch_info);
            status = rm->getDeviceInfo(dattr[i].id, mStreamAttr->type, &ecinfo);
            if(status) {
               QAL_ERR(LOG_TAG, "get ec info failed");
            }
            if (rm->getDeviceConfig((struct qal_device *)&dattr[i], sattr, ecinfo.channels)) {
                QAL_ERR(LOG_TAG, "Failed to get config for dev %d", dev_id);
                throw std::runtime_error("failed to get device config");
            }
        }

        dev = Device::getInstance(&dattr[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation is failed");
            free(mStreamAttr->in_media_config.ch_info);
            free(mStreamAttr);
            throw std::runtime_error("failed to create device object");
        }
        //Is this required for Voice UI?
        QAL_DBG(LOG_TAG, "Updating device config for voice UI");
        bool isDeviceConfigUpdated = rm->updateDeviceConfig(dev, &dattr[i],
                                                            sattr);

        if (isDeviceConfigUpdated)
            QAL_VERBOSE(LOG_TAG, "%s: Device config updated", __func__);

        mDevices.push_back(dev);
        dev = nullptr;
    }

    rm->registerStream(this);
    // Create internal states
    st_idle_ = new StIdle(*this);
    st_loaded_ = new StLoaded(*this);
    st_active = new StActive(*this);
    st_detected_ = new StDetected(*this);
    st_buffering_ = new StBuffering(*this);

    AddState(st_idle_);
    AddState(st_loaded_);
    AddState(st_active);
    AddState(st_detected_);
    AddState(st_buffering_);

    // Set initial state
    cur_state_ = st_idle_;
    prev_state_ = nullptr;

    timer_thread_ = std::thread(TimerThread, std::ref(*this));
    timer_stop_waiting_ = false;
    exit_timer_thread_ = false;

    QAL_DBG(LOG_TAG, "Exit");
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
        QAL_DBG(LOG_TAG, "Join timer thread");
        timer_thread_.join();
    }

    st_states_.clear();
    engines_.clear();
    gsl_engine_.reset();

    rm->deregisterStream(this);
    if (mStreamAttr) {
        if (mStreamAttr->in_media_config.ch_info) {
            free(mStreamAttr->in_media_config.ch_info);
        }
        free(mStreamAttr);
    }
    mDevices.clear();
    QAL_DBG(LOG_TAG, "Exit");
}

int32_t StreamSoundTrigger::close() {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

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

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::start() {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StStartRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::stop() {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, stream direction %d", mStreamAttr->direction);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StStopRecognitionEventConfig(false));
    status = cur_state_->ProcessEvent(ev_cfg);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::read(struct qal_buffer* buf) {
    int32_t size = 0;

    QAL_VERBOSE(LOG_TAG, "Enter");

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
        new StReadBufferEventConfig((void *)buf));
    size = cur_state_->ProcessEvent(ev_cfg);

    QAL_VERBOSE(LOG_TAG, "Exit, read size %d", size);

    return size;
}

int32_t StreamSoundTrigger::getParameters(uint32_t param_id, void **payload) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, get parameter %u", param_id);
    if (gsl_engine_) {
        status = gsl_engine_->GetParameters(param_id, payload);
        if (status)
            QAL_ERR(LOG_TAG, "Failed to get parameters from engine");
    } else {
        QAL_ERR(LOG_TAG, "No gsl engine present");
        status = -EINVAL;
    }
    return status;
}

int32_t StreamSoundTrigger::setParameters(uint32_t param_id, void *payload) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, param id %d", param_id);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    switch (param_id) {
      case QAL_PARAM_ID_LOAD_SOUND_MODEL: {
          std::shared_ptr<StEventConfig> ev_cfg(
              new StLoadEventConfig(payload));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      case QAL_PARAM_ID_RECOGNITION_CONFIG: {
          if (!payload) {
              QAL_ERR(LOG_TAG, "Invalid config payload");
              status = -EINVAL;
              break;
          }
          /*
           * Currently gecko needs graph stop and start for next detection.
           * Handle this event similar to fresh start config.
           */
          std::shared_ptr<StEventConfig> ev_cfg(
              new StRecognitionCfgEventConfig(payload));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      case QAL_PARAM_ID_STOP_BUFFERING: {
          /*
           * Currently gecko needs graph stop and start for next detection.
           * Handle this event similar to STOP_RECOGNITION.
           */
          std::shared_ptr<StEventConfig> ev_cfg(
              new StStopRecognitionEventConfig(false));
          status = cur_state_->ProcessEvent(ev_cfg);
          break;
      }
      default: {
          status = -EINVAL;
          QAL_ERR(LOG_TAG, "Unsupported param %u", param_id);
          break;
      }
    }

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

void StreamSoundTrigger::ConcurrentStreamStatus(qal_stream_type_t type,
                                                qal_stream_direction_t dir,
                                                bool active) {
    int32_t status = 0;
    bool conc_en = true;

    QAL_DBG(LOG_TAG, "Enter, type %d direction %d active %d", type, dir, active);
    if (dir == QAL_AUDIO_OUTPUT && type != QAL_STREAM_LOW_LATENCY) {
        if (rm->IsVoiceUILPISupported()) {
            std::shared_ptr<StEventConfig> ev_cfg(
                new StConcurrentStreamEventConfig(type, active));
            status = cur_state_->ProcessEvent(ev_cfg);
        }
    } else if (dir == QAL_AUDIO_INPUT || dir == QAL_AUDIO_INPUT_OUTPUT) {
        if (rm->IsAudioCaptureAndVoiceUIConcurrencySupported()) {
            if ((!rm->IsVoiceCallAndVoiceUIConcurrencySupported() &&
                 (type == QAL_STREAM_VOICE_CALL_TX ||
                  type == QAL_STREAM_VOICE_CALL_RX_TX ||
                  type == QAL_STREAM_VOICE_CALL)) ||
                (!rm->IsVoipAndVoiceUIConcurrencySupported() &&
                 type == QAL_STREAM_VOIP_TX)) {
                QAL_DBG(LOG_TAG, "pause on voip/voice concurrency");
                conc_en = false;
            }
        } else if (type == QAL_STREAM_LOW_LATENCY || // LL or mmap record
                   type == QAL_STREAM_RAW || // regular audio record
                   type == QAL_STREAM_VOICE_CALL_TX ||
                   type == QAL_STREAM_VOICE_CALL_RX_TX ||
                   type == QAL_STREAM_VOICE_CALL ||
                   type == QAL_STREAM_VOIP_TX) {
            conc_en = false;
        }
        if (!conc_en) {
            std::lock_guard<std::mutex> lck(mStreamMutex);
            if (active) {
                ++conc_tx_cnt_;
                if (conc_tx_cnt_ == 1) {
                    std::shared_ptr<StEventConfig> ev_cfg(
                       new StPauseEventConfig());
                    status = cur_state_->ProcessEvent(ev_cfg);
                }
            } else {
                --conc_tx_cnt_;
                if (conc_tx_cnt_ == 0) {
                    std::shared_ptr<StEventConfig> ev_cfg(
                        new StResumeEventConfig());
                    status = cur_state_->ProcessEvent(ev_cfg);
                }
            }
        }
    }
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
}

int32_t StreamSoundTrigger::setECRef(std::shared_ptr<Device> dev, bool is_enable)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, enable %d", is_enable);
    if (!dev) {
        QAL_ERR(LOG_TAG, "Invalid device");
        return -EINVAL;
    }

    std::shared_ptr<StEventConfig> ev_cfg(
        new StECRefEventConfig(dev, is_enable));
    status = cur_state_->ProcessEvent(ev_cfg);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to handle ec ref event");
    }

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

// TBD: to be tested, Yidong, is this enough?
int32_t StreamSoundTrigger::switchDevice(Stream* stream_handle,
                                         uint32_t no_of_devices,
                                         struct qal_device *device_array) {
    int32_t status = -EINVAL;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<Device> dev = nullptr;

    if (no_of_devices == 0 || !device_array) {
        QAL_ERR("%s: invalid param for device switch", __func__);
        status = -EINVAL;
        goto error_1;
    }

    for (auto& eng: engines_) {
        QAL_VERBOSE(LOG_TAG, "stop engine %d",
                    eng->GetEngineId());
        status = eng->GetEngine()->StopRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                    eng->GetEngineId(), status);
            goto error_1;
        }
    }

    /*
     * tell rm we are disabling existing mDevices,
     * so that it can disable any streams running on
     * 1. mDevices with common backend
     * TBD: as there are no devices with common backend now.
     * rm->disableDevice(mDevices);
     */

    for (int i = 0; i < mDevices.size(); i++) {
        QAL_VERBOSE(LOG_TAG, "device %d name %s, going to stop",
            mDevices[i]->getSndDeviceId(),
            mDevices[i]->getQALDeviceName().c_str());

        gsl_engine_->DisconnectSessionDevice(stream_handle, mStreamAttr->type,
                                             mDevices[i]);
        status = mDevices[i]->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "%s: Rx device stop failed with status %d",
                __func__, status);
            goto error_1;
        }

        rm->deregisterDevice(mDevices[i]);

        status = mDevices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close failed with status %d", status);
            goto error_1;
        }
    }

    // clear existing devices and enable new devices
    mDevices.clear();

    for (int i = 0; i < no_of_devices; i++) {
        // Check with RM if the configuration given can work or not
        // for e.g., if incoming stream needs 24 bit device thats also
        // being used by another stream, then the other stream should route
        dev = Device::getInstance((struct qal_device *)&mDevices[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "%s: Device creation failed", __func__);
            if (mStreamAttr) {
                if (mStreamAttr->in_media_config.ch_info) {
                    free(mStreamAttr->in_media_config.ch_info);
                }
                free(mStreamAttr);
                mStreamAttr = nullptr;
            }
            // TBD::free session too
            throw std::runtime_error("failed to create device object");
        }

        QAL_ERR(LOG_TAG, "device %d name %s, going to start",
                mDevices[i]->getSndDeviceId(),
                mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device %d name %s, start failed with status %d",
                    mDevices[i]->getSndDeviceId(),
                    mDevices[i]->getQALDeviceName().c_str(), status);
            goto error_2;
        }

        mDevices.push_back(dev);
        // enable sessions
        gsl_engine_->ConnectSessionDevice(stream_handle, mStreamAttr->type,
                                          mDevices[i]);
        rm->registerDevice(dev);
        dev = nullptr;
    }

    for (auto& eng: engines_) {
        QAL_VERBOSE(LOG_TAG, "Start engine %d",
                    eng->GetEngineId());
        status = eng->GetEngine()->StopRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Start engine %d failed, status %d",
                    eng->GetEngineId(), status);
            goto error_2;
        }
    }

error_2:
    if (mStreamAttr) {
        free(mStreamAttr->in_media_config.ch_info);
        free(mStreamAttr);
        mStreamAttr = nullptr;
    }

error_1:
    return status;
}

int32_t StreamSoundTrigger::isSampleRateSupported(uint32_t sampleRate) {
    int32_t rc = 0;

    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
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
          QAL_ERR(LOG_TAG, "sample rate not supported rc %d", rc);
          break;
    }

    return rc;
}

int32_t StreamSoundTrigger::isChannelSupported(uint32_t numChannels) {
    int32_t rc = 0;

    QAL_DBG(LOG_TAG, "numChannels %u", numChannels);
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
          QAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
          break;
    }
    return rc;
}

int32_t StreamSoundTrigger::isBitWidthSupported(uint32_t bitWidth) {
    int32_t rc = 0;

    QAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (bitWidth) {
      case BITWIDTH_16:
      case BITWIDTH_24:
      case BITWIDTH_32:
          break;
      default:
          rc = -EINVAL;
          QAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
          break;
    }
    return rc;
}

int32_t StreamSoundTrigger::registerCallBack(qal_stream_callback cb,
                                             void * cookie) {
    callback_ = cb;
    cookie_ = cookie;

    QAL_VERBOSE(LOG_TAG, "callback_ = %pK", callback_);

    return 0;
}

int32_t StreamSoundTrigger::getCallBack(qal_stream_callback *cb) {
    if (!cb) {
        QAL_ERR(LOG_TAG, "Invalid cb");
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

    QAL_DBG(LOG_TAG, "Enter, det_type %d", det_type);
    if ((det_type < GMM_DETECTED) || (det_type > VOP_REJECTED)) {
        QAL_ERR(LOG_TAG, "Invalid detection type %d", det_type);
        return -EINVAL;
    }

    std::unique_lock<std::mutex> lck(mStreamMutex);
    std::shared_ptr<StEventConfig> ev_cfg(
       new StDetectedEventConfig(det_type));
    status = cur_state_->ProcessEvent(ev_cfg);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

void StreamSoundTrigger::InternalStopRecognition() {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);
    if (pending_stop_) {
        std::shared_ptr<StEventConfig> ev_cfg(
           new StStopRecognitionEventConfig(true));
        status = cur_state_->ProcessEvent(ev_cfg);
    }
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
}

void StreamSoundTrigger::TimerThread(StreamSoundTrigger& st_stream) {
    QAL_DBG(LOG_TAG, "Enter");

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
    QAL_DBG(LOG_TAG, "Exit");
}

void StreamSoundTrigger::PostDelayedStop() {
    QAL_VERBOSE(LOG_TAG, "Post Delayed Stop for %p", this);
    pending_stop_ = true;
    std::lock_guard<std::mutex> lck(timer_mutex_);
    timer_stop_waiting_ = false;
    timer_start_cond_.notify_one();
}

void StreamSoundTrigger::CancelDelayedStop() {
    QAL_VERBOSE(LOG_TAG, "Cancel Delayed stop for %p", this);
    pending_stop_ = false;
    std::lock_guard<std::mutex> lck(timer_mutex_);
    timer_stop_waiting_ = true;
    timer_wait_cond_.notify_one();
}

/* TODO:
 *   - Need to track vendor UUID
 */
int32_t StreamSoundTrigger::LoadSoundModel(
    struct qal_st_sound_model *sound_model) {

    int32_t status = 0;
    int32_t i = 0;
    int32_t engine_id = 0;
    std::shared_ptr<SoundTriggerEngine> engine = nullptr;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;
    struct qal_st_sound_model *common_sm = nullptr;
    uint8_t *ptr = nullptr;
    uint8_t *sm_payload = nullptr;
    uint8_t *sm_data = nullptr;
    int32_t sm_size = 0;
    SML_GlobalHeaderType *global_hdr = nullptr;
    SML_HeaderTypeV3 *hdr_v3 = nullptr;
    SML_BigSoundModelTypeV3 *big_sm = nullptr;
    uint32_t sm_version = SML_MODEL_V2;

    QAL_DBG(LOG_TAG, "Enter");

    if (!sound_model) {
        QAL_ERR(LOG_TAG, "Invalid sound_model param status %d", status);
        return -EINVAL;
    }
    sound_model_type_ = sound_model->type;

    if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct qal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(*phrase_sm)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            QAL_ERR(LOG_TAG, "Invalid phrase sound model params data size=%d, "
                   "data offset=%d, type=%d phrases=%d status %d",
                   phrase_sm->common.data_size, phrase_sm->common.data_offset,
                   phrase_sm->num_phrases, status);
            return -EINVAL;
        }
        common_sm = (struct qal_st_sound_model*)&phrase_sm->common;
        sm_size = sizeof(*phrase_sm) + common_sm->data_size;

    } else if (sound_model->type == QAL_SOUND_MODEL_TYPE_GENERIC) {
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct qal_st_sound_model))) {
            QAL_ERR(LOG_TAG, "Invalid generic sound model params data size=%d,"
                    " data offset=%d status %d", sound_model->data_size,
                    sound_model->data_offset, status);
            return -EINVAL;
        }
        common_sm = sound_model;
        sm_size = sizeof(*common_sm) + common_sm->data_size;
    } else {
        QAL_ERR(LOG_TAG, "Unknown sound model type - %d status %d",
                sound_model->type, status);
        return -EINVAL;
    }
    if ((struct qal_st_sound_model *)sm_config_ != sound_model) {
        // Cache to use during SSR and other internal events handling.
        if (sm_config_) {
            free(sm_config_);
        }
        sm_config_ = (struct qal_st_phrase_sound_model *)calloc(1, sm_size);
        if (!sm_config_) {
            QAL_ERR(LOG_TAG, "sound model config allocation failed, status %d",
                    status);
            status = -ENOMEM;
            return status;
        }

        if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
            casa_osal_memcpy(sm_config_, sizeof(*phrase_sm),
                             phrase_sm, sizeof(*phrase_sm));
            casa_osal_memcpy((uint8_t *)sm_config_ + common_sm->data_offset,
                             common_sm->data_size,
                             (uint8_t *)phrase_sm + common_sm->data_offset,
                             common_sm->data_size);
        } else {
            casa_osal_memcpy(sm_config_, sizeof(*common_sm),
                             common_sm, sizeof(*common_sm));
            casa_osal_memcpy((uint8_t *)sm_config_ +  common_sm->data_offset,
                             common_sm->data_size,
                             (uint8_t *)common_sm + common_sm->data_offset,
                             common_sm->data_size);
        }
    }

    if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
        global_hdr = (SML_GlobalHeaderType *)sm_payload;
        if (global_hdr->magicNumber == SML_GLOBAL_HEADER_MAGIC_NUMBER) {
            // Parse sound model 3.0
            sm_version = SML_MODEL_V3;
            hdr_v3 = (SML_HeaderTypeV3 *)(sm_payload +
                                          sizeof(SML_GlobalHeaderType));
            QAL_DBG(LOG_TAG, "num of sound models = %u", hdr_v3->numModels);
            for (i = 0; i < hdr_v3->numModels; i++) {
                big_sm = (SML_BigSoundModelTypeV3 *)(
                    sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) +
                    (i * sizeof(SML_BigSoundModelTypeV3)));
                engine_id = static_cast<int32_t>(big_sm->type);

                QAL_INFO(LOG_TAG, "type = %u, size = %u",
                         big_sm->type, big_sm->size);
                if (big_sm->type == ST_SM_ID_SVA_GMM) {
                    sm_size = big_sm->size +
                              sizeof(struct qal_st_phrase_sound_model);
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "sm_data allocation failed, status %d",
                                status);
                        goto error_exit;
                    }
                    casa_osal_memcpy(sm_data, sizeof(*phrase_sm),
                                     (char *)phrase_sm, sizeof(*phrase_sm));
                    common_sm = (struct qal_st_sound_model *)sm_data;
                    common_sm->data_size = big_sm->size;
                    common_sm->data_offset += sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (hdr_v3->numModels * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    casa_osal_memcpy(sm_data + sizeof(*phrase_sm), big_sm->size,
                                     (char *)phrase_sm + common_sm->data_offset,
                                     big_sm->size);
                    common_sm->data_offset = sizeof(*phrase_sm);
                    common_sm = (struct qal_st_sound_model *)&phrase_sm->common;

                    gsl_engine_ = SoundTriggerEngine::Create(this,
                                      ST_SM_ID_SVA_GMM, &reader_, nullptr);
                    if (!gsl_engine_) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "big_sm: gsl engine creation failed");
                        goto error_exit;
                    }
                    if (!reader_) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "big_sm: gsl engine reader creation"
                                "failed");
                        goto error_exit;
                    }

                    status = gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
                    if (status) {
                        QAL_ERR(LOG_TAG, "big_sm: gsl engine loading model"
                                "failed, status %d", status);
                        goto error_exit;
                    }

                    engine_id = static_cast<int32_t>(ST_SM_ID_SVA_GMM);
                    std::shared_ptr<EngineCfg> engine_cfg(new EngineCfg(
                       engine_id, gsl_engine_, (void *) sm_data, sm_size));

                    AddEngine(engine_cfg);
                } else {
                    sm_size = big_sm->size;
                    ptr = (uint8_t *)sm_payload + sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (hdr_v3->numModels * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "Failed to alloc memory for sm_data");
                        goto error_exit;
                    }
                    casa_osal_memcpy(sm_data, sm_size, ptr, sm_size);

                    engine = SoundTriggerEngine::Create(this, big_sm->type,
                                                        &reader_,
                                                        reader_->ringBuffer_);
                    if (!engine) {
                        QAL_ERR(LOG_TAG, "Failed to create engine for type %d",
                                big_sm->type);
                        status = -ENOENT;
                        goto error_exit;
                    }

                    status = engine->LoadSoundModel(this, sm_data, sm_size);
                    if (status) {
                        QAL_ERR(LOG_TAG, "Loading model to engine type %d"
                                "failed, status %d", big_sm->type, status);
                        goto error_exit;
                    }

                    std::shared_ptr<EngineCfg> engine_cfg(new EngineCfg(
                       engine_id, engine, (void *)sm_data, sm_size));

                    AddEngine(engine_cfg);
                }
            }
            if (!gsl_engine_) {
                QAL_ERR(LOG_TAG, "First stage sound model not present!!");
                goto error_exit;
            }
        } else {
            // Parse sound model 2.0
            sm_size = sizeof(*phrase_sm) + common_sm->data_size;
            sm_data = (uint8_t *)calloc(1, sm_size);
            if (!sm_data) {
                QAL_ERR(LOG_TAG, "Failed to allocate memory for sm_data");
                status = -ENOMEM;
                goto error_exit;
            }
            casa_osal_memcpy(sm_data, sizeof(*phrase_sm),
                             (uint8_t *)phrase_sm, sizeof(*phrase_sm));
            casa_osal_memcpy(sm_data + sizeof(*phrase_sm), common_sm->data_size,
                             (uint8_t*)phrase_sm + common_sm->data_offset,
                             common_sm->data_size);
            gsl_engine_ = SoundTriggerEngine::Create(this, ST_SM_ID_SVA_GMM,
                                                &reader_, nullptr);
            if (!gsl_engine_) {
                QAL_ERR(LOG_TAG, "gsl engine creation failed");
                status = -ENOMEM;
                goto error_exit;
            }
            if (!reader_) {
                QAL_ERR(LOG_TAG, "gsl engine reader creation failed");
                status = -ENOMEM;
                goto error_exit;
            }

            status = gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
            if (status) {
                QAL_ERR(LOG_TAG, "gsl engine loading model failed, status %d",
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
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
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
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

// TODO: look into how cookies are used here
int32_t StreamSoundTrigger::SendRecognitionConfig(
    struct qal_st_recognition_config *config) {

    int32_t status = 0;
    struct st_param_header *param_hdr = NULL;
    struct st_hist_buffer_info *hist_buf = NULL;
    struct st_det_perf_mode_info *det_perf_mode = NULL;
    uint8_t *opaque_ptr = NULL;
    unsigned int opaque_size = 0, conf_levels_payload_size = 0;
    uint32_t hist_buffer_duration = 0;
    uint32_t pre_roll_duration = 0;
    uint32_t conf_levels_intf_version = 0;
    uint8_t *conf_levels = NULL;
    uint32_t num_conf_levels = 0;
    bool capture_requested = false;

    QAL_DBG(LOG_TAG, "Enter");
    if (!config) {
        QAL_ERR(LOG_TAG, "Invalid config");
        return -EINVAL;
    }
    if (rec_config_ != config) {
        // Possible due to subsequent detections.
        if (rec_config_) {
            free(rec_config_);
        }
        rec_config_ = (struct qal_st_recognition_config *)calloc(1,
            sizeof(struct qal_st_recognition_config) + config->data_size);
        if (!rec_config_) {
            QAL_ERR(LOG_TAG, "Failed to allocate rec_config status %d", status);
            return -ENOMEM;
        }
        casa_osal_memcpy(rec_config_, sizeof(struct qal_st_recognition_config),
                         config, sizeof(struct qal_st_recognition_config));
        casa_osal_memcpy((uint8_t *)rec_config_ + config->data_offset,
                         config->data_size,
                         (uint8_t *)config + config->data_offset,
                         config->data_size);
    }

    // Parse recognition config
    if (config->data_size > CUSTOM_CONFIG_OPAQUE_DATA_SIZE) {
        opaque_ptr = (uint8_t *)config + config->data_offset;
        while (opaque_size < config->data_size) {
            param_hdr = (struct st_param_header *)opaque_ptr;
            QAL_VERBOSE(LOG_TAG, "key %d, payload size %d",
                        param_hdr->key_id, param_hdr->payload_size);

            switch (param_hdr->key_id) {
              case ST_PARAM_KEY_CONFIDENCE_LEVELS:
                  conf_levels_intf_version = *(uint32_t *)(
                      opaque_ptr + sizeof(struct st_param_header));
                  QAL_VERBOSE(LOG_TAG, "conf_levels_intf_version = %u",
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
                      QAL_ERR(LOG_TAG, "Conf level format error, exiting");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  status = ParseOpaqueConfLevels(opaque_ptr,
                                                 conf_levels_intf_version,
                                                 &conf_levels,
                                                 &num_conf_levels);
                if (status) {
                    QAL_ERR(LOG_TAG, "Failed to parse opaque conf levels");
                    goto error_exit;
                }

                opaque_size += sizeof(struct st_param_header) +
                    conf_levels_payload_size;
                opaque_ptr += sizeof(struct st_param_header) +
                    conf_levels_payload_size;
                if (status) {
                    QAL_ERR(LOG_TAG, "Parse conf levels failed(status=%d)",
                            status);
                    status = -EINVAL;
                    goto error_exit;
                }
                break;
              case ST_PARAM_KEY_HISTORY_BUFFER_CONFIG:
                  if (param_hdr->payload_size !=
                      sizeof(struct st_hist_buffer_info)) {
                      QAL_ERR(LOG_TAG, "History buffer config format error");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  hist_buf = (struct st_hist_buffer_info *)(opaque_ptr +
                      sizeof(struct st_param_header));
                  hist_buffer_duration = hist_buf->hist_buffer_duration_msec;
                  pre_roll_duration = hist_buf->pre_roll_duration_msec;
                  QAL_DBG(LOG_TAG, "recognition config history buf len = %d"
                          "preroll len = %d, minor version = %d",
                          hist_buf->hist_buffer_duration_msec,
                          hist_buf->pre_roll_duration_msec,
                          hist_buf->version);
                  gsl_engine_->UpdateBufConfig(hist_buffer_duration,
                                               pre_roll_duration);
                  opaque_size += sizeof(struct st_param_header) +
                      sizeof(struct st_hist_buffer_info);
                  opaque_ptr += sizeof(struct st_param_header) +
                      sizeof(struct st_hist_buffer_info);
                  break;
              case ST_PARAM_KEY_DETECTION_PERF_MODE:
                  if (param_hdr->payload_size !=
                      sizeof(struct st_det_perf_mode_info)) {
                      QAL_ERR(LOG_TAG, "Opaque data format error, exiting");
                      status = -EINVAL;
                      goto error_exit;
                  }
                  det_perf_mode = (struct st_det_perf_mode_info *)
                      (opaque_ptr + sizeof(struct st_param_header));
                  QAL_DBG(LOG_TAG, "set perf mode %d", det_perf_mode->mode);
                  opaque_size += sizeof(struct st_param_header) +
                      sizeof(struct st_det_perf_mode_info);
                  opaque_ptr += sizeof(struct st_param_header) +
                      sizeof(struct st_det_perf_mode_info);
                  break;
              default:
                  QAL_ERR(LOG_TAG, "Unsupported opaque data key id, exiting");
                  status = -EINVAL;
                  goto error_exit;
            }
        }
    } else {
        status = FillConfLevels(config, &conf_levels, &num_conf_levels);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to parse conf levels from rc config");
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

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

bool StreamSoundTrigger::compareRecognitionConfig(
   const struct qal_st_recognition_config *current_config,
   struct qal_st_recognition_config *new_config) {
    uint32_t i = 0, j = 0;

    /*
     * Sometimes if the number of user confidence levels is 0, the
     * qal_st_confidence_level struct will be different between the two
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
    struct qal_st_recognition_event *rec_event = nullptr;

    status = GenerateCallbackEvent(&rec_event);
    if (status || !rec_event) {
        QAL_ERR(LOG_TAG, "Failed to generate callback event");
        return status;
    }
    if (callback_) {
        QAL_INFO(LOG_TAG, "Notify detection event to client");
        mStreamMutex.unlock();
        callback_(this, 0, (uint32_t *)rec_event, rec_config_->cookie);
        mStreamMutex.lock();
    }

    if (rec_event->media_config.ch_info) {
        free(rec_event->media_config.ch_info);
    }
    free(rec_event);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t StreamSoundTrigger::GenerateCallbackEvent(
    struct qal_st_recognition_event **event) {

    struct qal_st_phrase_recognition_event *phrase_event = nullptr;
    struct qal_channel_info *ch_info = nullptr;
    struct st_param_header *param_hdr = nullptr;
    struct st_confidence_levels_info *conf_levels = nullptr;
    struct st_keyword_indices_info *kw_indices = nullptr;
    struct st_timestamp_info *timestamps = nullptr;
    size_t opaque_size = 0;
    size_t event_size = 0;
    uint8_t *opaque_data = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    *event = nullptr;
    if (sound_model_type_ == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        struct detection_event_info *det_ev_info =
            gsl_engine_->GetDetectionEventInfo();
        if (!det_ev_info) {
            QAL_ERR(LOG_TAG, "detection info not available");
            return -EINVAL;
        }

        opaque_size = (3 * sizeof(struct st_param_header)) +
            sizeof(struct st_timestamp_info) +
            sizeof(struct st_keyword_indices_info) +
            sizeof(struct st_confidence_levels_info);

        event_size = sizeof(struct qal_st_phrase_recognition_event) +
                     opaque_size;
        phrase_event = (struct qal_st_phrase_recognition_event *)
                       calloc(1, event_size);
        if (!phrase_event) {
            QAL_ERR(LOG_TAG, "Failed to alloc memory for recognition event");
            return -ENOMEM;
        }

        ch_info = (struct qal_channel_info *)
                  calloc(1, sizeof(struct qal_channel_info));
        if (!ch_info) {
            QAL_ERR(LOG_TAG, "Failed to alloc memory for channel info");
            free(phrase_event);
            return -ENOMEM;
        }

        phrase_event->num_phrases = rec_config_->num_phrases;
        memcpy(phrase_event->phrase_extras, rec_config_->phrases,
               phrase_event->num_phrases *
               sizeof(struct qal_st_phrase_recognition_extra));

        *event = &(phrase_event->common);
        (*event)->media_config.ch_info = nullptr;
        (*event)->status = QAL_RECOGNITION_STATUS_SUCCESS;
        (*event)->type = sound_model_type_;
        (*event)->st_handle = (qal_st_handle_t *)this;
        (*event)->capture_available = rec_config_->capture_requested;
        // TODO: generate capture session
        (*event)->capture_session = 0;
        (*event)->capture_delay_ms = 0;
        (*event)->capture_preamble_ms = 0;
        (*event)->trigger_in_data = true;
        (*event)->data_size = opaque_size;
        (*event)->data_offset = sizeof(struct qal_st_phrase_recognition_event);

        (*event)->media_config.sample_rate = SAMPLINGRATE_16K;
        (*event)->media_config.bit_width = BITWIDTH_16;
        (*event)->media_config.ch_info = ch_info;
        (*event)->media_config.ch_info->channels = CHANNELS_1;
        (*event)->media_config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

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
        reader_->getIndices(&kw_indices->start_index, &kw_indices->end_index);
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
    // TODO: handle for generic sound model
    QAL_DBG(LOG_TAG, "Exit");

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

    QAL_DBG(LOG_TAG, "Enter");
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
                QAL_DBG(LOG_TAG, "confidence level = %d", confidence_level);
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
                QAL_DBG(LOG_TAG, "confidence level = %d", confidence_level_v2);
                for (auto& eng: engines_) {
                    QAL_VERBOSE(LOG_TAG, "sm id %d, engine id %d ",
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
        QAL_ERR(LOG_TAG, "Did not receive GMM confidence threshold, error!");
        status = -EINVAL;
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit");

    return status;
}

int32_t StreamSoundTrigger::FillConfLevels(
    struct qal_st_recognition_config *config,
    uint8_t **out_conf_levels,
    uint32_t *out_num_conf_levels) {

    int32_t status = 0;
    uint32_t num_conf_levels = 0;
    unsigned int user_level, user_id;
    unsigned int i = 0, j = 0;
    uint8_t *conf_levels = nullptr;
    unsigned char *user_id_tracker = nullptr;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;

    QAL_DBG(LOG_TAG, "Enter");

    if (!config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "invalid input status %d", status);
        goto exit;
    }

    for (auto& eng: engines_) {
        if (eng->GetEngineId() == ST_SM_ID_SVA_GMM) {
            phrase_sm = (struct qal_st_phrase_sound_model *)eng->sm_data_;
            break;
        }
    }

    if ((config->num_phrases == 0) ||
        (config->num_phrases > phrase_sm->num_phrases)) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid phrase data status %d", status);
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
        QAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
        goto exit;
    }

    user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
    if (!user_id_tracker) {
        free(conf_levels);
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                status);
        goto exit;
    }

    /* for debug */
    for (i = 0; i < config->num_phrases; i++) {
        QAL_VERBOSE(LOG_TAG, "[%d] kw level %d", i,
        config->phrases[i].confidence_level);
        for (j = 0; j < config->phrases[i].num_levels; j++) {
            QAL_VERBOSE(LOG_TAG, "[%d] user_id %d level %d ", i,
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
                QAL_ERR(LOG_TAG, "Invalid params user id %d status %d", user_id,
                        status);
                goto exit;
            } else {
                if (user_id_tracker[user_id] == 1) {
                    status = -EINVAL;
                    QAL_ERR(LOG_TAG, "Duplicate user id %d status %d", user_id,
                            status);
                    goto exit;
                }
                conf_levels[user_id] = (user_level < 100) ? user_level : 100;
                user_id_tracker[user_id] = 1;
                QAL_VERBOSE(LOG_TAG, "user_conf_levels[%d] = %d", user_id,
                            conf_levels[user_id]);
            }
        }
    }

    *out_conf_levels = conf_levels;
    *out_num_conf_levels = num_conf_levels;

exit:
    if (user_id_tracker)
        free(user_id_tracker);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

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

    QAL_VERBOSE(LOG_TAG, "Enter");

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
            QAL_ERR(LOG_TAG, "ERROR. Invalid inputs");
            goto exit;
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            num_conf_levels++;
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++)
                num_conf_levels++;
        }

        if (!num_conf_levels) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "ERROR. Invalid num_conf_levels input");
            goto exit;
        }

        conf_levels = (unsigned char*)calloc(1, num_conf_levels);
        if (!conf_levels) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
            goto exit;
        }

        user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
        if (!user_id_tracker) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                    status);
            goto exit;
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            QAL_ERR(LOG_TAG, "[%d] kw level %d", i,
                sm_levels->kw_levels[i].kw_level);
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++) {
                QAL_ERR(LOG_TAG, "[%d] user_id %d level %d ", i,
                    sm_levels->kw_levels[i].user_levels[j].user_id,
                    sm_levels->kw_levels[i].user_levels[j].level);
            }
        }

        for (i = 0; i < sm_levels->num_kw_levels; i++) {
            if (i < num_conf_levels) {
                conf_levels[i] = sm_levels->kw_levels[i].kw_level;
            } else {
                status = -EINVAL;
                QAL_ERR(LOG_TAG, "ERROR. Invalid numver of kw levels");
                goto exit;
            }
            for (j = 0; j < sm_levels->kw_levels[i].num_user_levels; j++) {
                user_level = sm_levels->kw_levels[i].user_levels[j].level;
                user_id = sm_levels->kw_levels[i].user_levels[j].user_id;
                if ((user_id < sm_levels->num_kw_levels) ||
                    (user_id >= num_conf_levels)) {
                    status = -EINVAL;
                    QAL_ERR(LOG_TAG, "ERROR. Invalid params user id %d>%d",
                        user_id, num_conf_levels);
                    goto exit;
                } else {
                    if (user_id_tracker[user_id] == 1) {
                        status = -EINVAL;
                        QAL_ERR(LOG_TAG, "ERROR. Duplicate user id %d",
                            user_id);
                        goto exit;
                    }
                    conf_levels[user_id] = (user_level < 100) ?
                                           user_level: 100;
                    user_id_tracker[user_id] = 1;
                    QAL_ERR(LOG_TAG, "user_conf_levels[%d] = %d",
                        user_id, conf_levels[user_id]);
                }
            }
        }
    } else {
        sm_levels_v2 =
            (struct st_sound_model_conf_levels_v2 *)sm_levels_generic;
        if (!sm_levels_v2) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "ERROR. Invalid inputs");
            goto exit;
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            num_conf_levels++;
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++)
                num_conf_levels++;
        }

        if (!num_conf_levels) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "ERROR. Invalid num_conf_levels input");
            goto exit;
        }

        conf_levels = (unsigned char*)calloc(1, num_conf_levels);
        if (!conf_levels) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "conf_levels calloc failed, status %d", status);
            goto exit;
        }

        user_id_tracker = (unsigned char *)calloc(1, num_conf_levels);
        if (!user_id_tracker) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "failed to allocate user_id_tracker status %d",
                    status);
            goto exit;
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            QAL_VERBOSE(LOG_TAG, "[%d] kw level %d", i,
                sm_levels_v2->kw_levels[i].kw_level);
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++) {
                QAL_VERBOSE(LOG_TAG, "[%d] user_id %d level %d ", i,
                     sm_levels_v2->kw_levels[i].user_levels[j].user_id,
                     sm_levels_v2->kw_levels[i].user_levels[j].level);
            }
        }

        for (i = 0; i < sm_levels_v2->num_kw_levels; i++) {
            if (i < num_conf_levels) {
                conf_levels[i] = sm_levels_v2->kw_levels[i].kw_level;
            } else {
                status = -EINVAL;
                QAL_ERR(LOG_TAG, "ERROR. Invalid numver of kw levels");
                goto exit;
            }
            for (j = 0; j < sm_levels_v2->kw_levels[i].num_user_levels; j++) {
                user_level = sm_levels_v2->kw_levels[i].user_levels[j].level;
                user_id = sm_levels_v2->kw_levels[i].user_levels[j].user_id;
                if ((user_id < sm_levels_v2->num_kw_levels) ||
                    (user_id >= num_conf_levels)) {
                    status = -EINVAL;
                    QAL_ERR(LOG_TAG, "ERROR. Invalid params user id %d>%d",
                         user_id, num_conf_levels);
                    goto exit;
                } else {
                    if (user_id_tracker[user_id] == 1) {
                        status = -EINVAL;
                        QAL_ERR(LOG_TAG, "ERROR. Duplicate user id %d",
                            user_id);
                        goto exit;
                    }
                    conf_levels[user_id] = (user_level < 100) ?
                                            user_level: 100;
                    user_id_tracker[user_id] = 1;
                    QAL_VERBOSE(LOG_TAG, "user_conf_levels[%d] = %d",
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
            QAL_VERBOSE(LOG_TAG, "Notify detection event %d to engine %d",
                    detected, eng->GetEngineId());
            eng->GetEngine()->SetDetected(detected);
        }
    }
}

qal_device_id_t StreamSoundTrigger::GetAvailCaptureDevice()
{
    if (rm->isDeviceAvailable(QAL_DEVICE_IN_WIRED_HEADSET))
        return QAL_DEVICE_IN_HEADSET_VA_MIC;
    else
        return QAL_DEVICE_IN_HANDSET_VA_MIC;
}

void StreamSoundTrigger::AddEngine(std::shared_ptr<EngineCfg> engine_cfg) {
    for (int32_t i = 0; i < engines_.size(); i++) {
        if (engines_[i] == engine_cfg) {
            QAL_VERBOSE(LOG_TAG, "engine type %d already exists",
                        engine_cfg->id_);
            return;
        }
    }
    QAL_VERBOSE(LOG_TAG, "Add engine %d, gsl_engine %p", engine_cfg->id_,
                gsl_engine_.get());
    engines_.push_back(engine_cfg);
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
        QAL_ERR(LOG_TAG, "Unknown transit state %d ", state_id);
        return;
    }
    prev_state_ = cur_state_;
    cur_state_ = it->second;
    QAL_DBG(LOG_TAG, "state transitioned from %d to %d",
            prev_state_->GetStateId(), it->first);
}

int32_t StreamSoundTrigger::ProcessInternalEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {
    return cur_state_->ProcessEvent(ev_cfg);
}

int32_t StreamSoundTrigger::StIdle::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

    QAL_DBG(LOG_TAG, "StIdle: handle event %d", ev_cfg->id_);

    switch (ev_cfg->id_) {
      case ST_EV_LOAD_SOUND_MODEL: {
          StLoadEventConfigData *data =
              (StLoadEventConfigData *)ev_cfg->data_.get();
          std::vector<std::shared_ptr<Device>> tmp_devices;
          for (auto& dev: st_stream_.mDevices) {
              status = dev->open();
              if (0 != status) {
                  QAL_ERR(LOG_TAG, "Device open failed, status %d", status);
                  goto err_exit;
              }
              tmp_devices.push_back(dev);
          }
          status = st_stream_.LoadSoundModel(
              (struct qal_st_sound_model *)data->data_);

          if (0 != status) {
              QAL_ERR(LOG_TAG, "Failed to load sound model, status %d", status);
              goto err_exit;
          } else {
              QAL_VERBOSE(LOG_TAG, "Opened the engine and device successfully");
              TransitTo(ST_STATE_LOADED);
              break;
          }
      err_exit:
          for (auto& tmp_dev: tmp_devices)
              tmp_dev->close();
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
      default: {
          QAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
          break;
      }
    }

    return status;
}

int32_t StreamSoundTrigger::StLoaded::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

    QAL_DBG(LOG_TAG, "StLoaded: handle event %d", ev_cfg->id_);

    switch (ev_cfg->id_) {
    case ST_EV_UNLOAD_SOUND_MODEL: {
        int ret = 0;
        for (auto& dev: st_stream_.mDevices) {
            QAL_DBG(LOG_TAG, "Close device %d-%s", dev->getSndDeviceId(),
                    dev->getQALDeviceName().c_str());
            ret = dev->close();
            if (0 != ret) {
                QAL_ERR(LOG_TAG, "Device open failed, status %d", status);
                status = ret;
            }
        }

        for (auto& eng: st_stream_.engines_) {
            QAL_DBG(LOG_TAG, "Unload engine %d", eng->GetEngineId());
            status = eng->GetEngine()->UnloadSoundModel(&st_stream_);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Unload engine %d failed, status %d",
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

        TransitTo(ST_STATE_IDLE);
        break;
    }
    case ST_EV_RECOGNITION_CONFIG: {
        StRecognitionCfgEventConfigData *data =
            (StRecognitionCfgEventConfigData *)ev_cfg->data_.get();
        status = st_stream_.SendRecognitionConfig(
           (struct qal_st_recognition_config *)data->data_);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to send recognition config, status %d",
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
            QAL_ERR(LOG_TAG, "Recognition config not set", data->restart_);
            status = -EINVAL;
            break;
        }

        std::vector<std::shared_ptr<SoundTriggerEngine>> tmp_engines;
        std::vector<std::shared_ptr<Device>> tmp_devices;

        for (auto& dev: st_stream_.mDevices){
            QAL_DBG(LOG_TAG, "Start device %d-%s", dev->getSndDeviceId(),
                    dev->getQALDeviceName().c_str());

            status = dev->start();
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Device start failed, status %d", status);
                goto err_exit;
            } else {
                st_stream_.rm->registerDevice(dev);
                tmp_devices.push_back(dev);
            }
        }

        for (auto& eng: st_stream_.engines_) {
            QAL_VERBOSE(LOG_TAG, "Start st engine %d", eng->GetEngineId());
            status = eng->GetEngine()->StartRecognition(&st_stream_);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Start st engine %d failed, status %d",
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

        for (auto& dev: tmp_devices) {
            dev->stop();
            st_stream_.rm->deregisterDevice(dev);
        }
        break;
    }
    case ST_EV_CONCURRENT_STREAM: {
        std::shared_ptr<StEventConfig> ev_cfg1(new StUnloadEventConfig());
        status = st_stream_.ProcessInternalEvent(ev_cfg1);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to Unload, status %d", status);
            break;
        }
        std::shared_ptr<StEventConfig> ev_cfg2(
            new StLoadEventConfig(st_stream_.sm_config_));
        status = st_stream_.ProcessInternalEvent(ev_cfg2);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to load, status %d", status);
            break;
        }
        if (st_stream_.rec_config_) {
            status = st_stream_.SendRecognitionConfig(st_stream_.rec_config_);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Failed to send recognition config, status %d",
                        status);
                break;
            }
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
    default: {
        QAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
        break;
    }
    }
    return status;
}

int32_t StreamSoundTrigger::StActive::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {

    int32_t status = 0;

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
          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
              status = eng->GetEngine()->StopRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
              }
          }
          for (auto& dev: st_stream_.mDevices) {
              QAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                      dev->getQALDeviceName().c_str());
              status = dev->stop();
              if (status) {
                  QAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
              }
              st_stream_.rm->deregisterDevice(dev);
          }
          TransitTo(ST_STATE_LOADED);
          break;
      }
      case ST_EV_CONCURRENT_STREAM: {
          std::shared_ptr<StEventConfig> ev_cfg1(
              new StStopRecognitionEventConfig(false));
          status = st_stream_.ProcessInternalEvent(ev_cfg1);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to Stop, status %d", status);
              break;
          }
          std::shared_ptr<StEventConfig> ev_cfg2(new StUnloadEventConfig());
          status = st_stream_.ProcessInternalEvent(ev_cfg2);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to Unload, status %d", status);
              break;
          }
          std::shared_ptr<StEventConfig> ev_cfg3(
              new StLoadEventConfig(st_stream_.sm_config_));
          status = st_stream_.ProcessInternalEvent(ev_cfg3);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to Load, status %d", status);
              break;

          }
          status = st_stream_.SendRecognitionConfig(st_stream_.rec_config_);
          if (0 != status) {
              QAL_ERR(LOG_TAG, "Failed to send recognition config, status %d",
                      status);
              break;
          }
          std::shared_ptr<StEventConfig> ev_cfg4(
              new StStartRecognitionEventConfig(false));
          status = st_stream_.ProcessInternalEvent(ev_cfg4);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to Start, status %d", status);
          }
          break;
      }
      case ST_EV_EC_REF: {
          StECRefEventConfigData *data =
              (StECRefEventConfigData *)ev_cfg->data_.get();
          Stream *s = dynamic_cast<Stream *>(&st_stream_);
          status = st_stream_.gsl_engine_->setECRef(s, data->dev_,
              data->is_enable_);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to set EC Ref in gsl engine");
          }
          break;
      }
      case ST_EV_READ_BUFFER: {
          status = -EIO;
          break;
      }
      default: {
          QAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
          break;
      }
    }
    return status;
}

int32_t StreamSoundTrigger::StDetected::ProcessEvent(
    std::shared_ptr<StEventConfig> ev_cfg) {
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "StDetected: handle event %d", ev_cfg->id_);

    switch (ev_cfg->id_) {
      case ST_EV_START_RECOGNITION: {
          // Client restarts next recognition without config changed.
          st_stream_.CancelDelayedStop();

          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
              status = eng->GetEngine()->RestartRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Restart engine %d failed, status %d",
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
          // gets dispatched after internal delayed stop timer times out.
          StStopRecognitionEventConfigData *data =
              (StStopRecognitionEventConfigData *)ev_cfg->data_.get();
          if (!data->deferred_) {
              QAL_ERR(LOG_TAG, "Not a deferred stop");
              status = -EINVAL;
              break;
          }
          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
              status = eng->GetEngine()->StopRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
              }
          }
          for (auto& dev: st_stream_.mDevices){
              QAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                      dev->getQALDeviceName().c_str());
              status = dev->stop();
              if (status)
                  QAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
              st_stream_.rm->deregisterDevice(dev);
          }
          TransitTo(ST_STATE_LOADED);
          break;
      }
      case ST_EV_RECOGNITION_CONFIG: {
          /*
           * Client can update config for next recognition.
           * Get to loaded state as START event will start recognition.
           */
          st_stream_.CancelDelayedStop();

          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
              status = eng->GetEngine()->StopRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
              }
          }
          for (auto& dev: st_stream_.mDevices) {
              QAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                      dev->getQALDeviceName().c_str());
              status = dev->stop();
              if (status) {
                  QAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
              }
              st_stream_.rm->deregisterDevice(dev);
          }
          TransitTo(ST_STATE_LOADED);
          status = st_stream_.ProcessInternalEvent(ev_cfg);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to handle recognition config, status %d",
                      status);
          }
          // START event will be handled in loaded state.
          break;
      }
      case ST_EV_CONCURRENT_STREAM: {
          st_stream_.CancelDelayedStop();
          // Reuse from Active state.
          TransitTo(ST_STATE_ACTIVE);
          status = st_stream_.ProcessInternalEvent(ev_cfg);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to process CONCURRENT_STREAM event,"
                               "status %d", status);
          }
          break;
      }
      case ST_EV_RESUME: {
          st_stream_.paused_ = false;
          break;
      }

      default: {
          QAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
          break;
      }
    }
    return status;
}

int32_t StreamSoundTrigger::StBuffering::ProcessEvent(
   std::shared_ptr<StEventConfig> ev_cfg) {
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG, "StBuffering: handle event %d", ev_cfg->id_);

    switch (ev_cfg->id_) {
      case ST_EV_READ_BUFFER: {
          StReadBufferEventConfigData *data =
              (StReadBufferEventConfigData *)ev_cfg->data_.get();
          struct qal_buffer *buf = (struct qal_buffer *)data->data_;

          if (!st_stream_.reader_) {
              QAL_ERR(LOG_TAG, "no reader exists");
              status = -EINVAL;
              break;
          }
          status = st_stream_.reader_->read(buf->buffer, buf->size);
          break;
      }
      case ST_EV_STOP_BUFFERING: {
          QAL_DBG(LOG_TAG, "StBuffering: stop buffering");
          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                          eng->GetEngineId());
              status = eng->GetEngine()->StopBuffering(&st_stream_);
              if (status){
                  QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
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
        QAL_DBG(LOG_TAG, "StBuffering: start recognition, is restart %d",
                data->restart_);
        st_stream_.CancelDelayedStop();

        for (auto& eng: st_stream_.engines_) {
            QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                        eng->GetEngineId());
            status = eng->GetEngine()->StopBuffering(&st_stream_);
            if (status)
                QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                        eng->GetEngineId(), status);
        }
        if (st_stream_.reader_)
            st_stream_.reader_->reset();

        for (auto& eng: st_stream_.engines_) {
            QAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
            status = eng->GetEngine()->RestartRecognition(&st_stream_);
            if (status) {
                QAL_ERR(LOG_TAG, "Restart engine %d buffering failed, status %d",
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
              QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                          eng->GetEngineId());
              status = eng->GetEngine()->StopBuffering(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
                  // continue stopping recognition
              }
          }
          if (st_stream_.reader_) {
            st_stream_.reader_->reset();
          }

          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
              status = eng->GetEngine()->StopRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
              }
          }
          for (auto& dev: st_stream_.mDevices) {
              QAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                      dev->getQALDeviceName().c_str());
              status = dev->stop();
              if (status) {
                  QAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
              }
              st_stream_.rm->deregisterDevice(dev);
          }
          TransitTo(ST_STATE_LOADED);
          status = st_stream_.ProcessInternalEvent(ev_cfg);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to handle recognition config, status %d",
                      status);
          }
          // START event will be handled in loaded state.
          break;
      }
      case ST_EV_PAUSE: {
          st_stream_.paused_ = true;
          QAL_DBG(LOG_TAG, "StBuffering: Pause");
          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                          eng->GetEngineId());
              status = eng->GetEngine()->StopBuffering(&st_stream_);
              if (status){
                  QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
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
              QAL_VERBOSE(LOG_TAG, "Stop engine %d", eng->GetEngineId());
              status = eng->GetEngine()->StopRecognition(&st_stream_);
              if (status) {
                  QAL_ERR(LOG_TAG, "Stop engine %d failed, status %d",
                          eng->GetEngineId(), status);
              }
          }
          for (auto& dev: st_stream_.mDevices) {
              QAL_DBG(LOG_TAG, "Stop device %d-%s", dev->getSndDeviceId(),
                      dev->getQALDeviceName().c_str());
              status = dev->stop();
              if (status) {
                  QAL_ERR(LOG_TAG, "Device stop failed, status %d", status);
              }
              st_stream_.rm->deregisterDevice(dev);
          }
          TransitTo(ST_STATE_LOADED);
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
          if (data->det_type_ == CNN_REJECTED ||
              data->det_type_ == VOP_REJECTED) {
              QAL_DBG(LOG_TAG, "Second stage rejected, type %d",
                      data->det_type_);
              st_stream_.detection_state_ = ENGINE_IDLE;
              for (auto& eng: st_stream_.engines_) {
                  QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                              eng->GetEngineId());
                  status = eng->GetEngine()->StopBuffering(&st_stream_);
                  if (status) {
                      QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed,"
                                       "status %d", eng->GetEngineId(), status);
                      // Continue to restart
                  }
              }
              if (st_stream_.reader_) {
                  st_stream_.reader_->reset();
              }
              for (auto& eng: st_stream_.engines_) {
                  QAL_VERBOSE(LOG_TAG, "Restart engine %d", eng->GetEngineId());
                  status = eng->GetEngine()->RestartRecognition(&st_stream_);
                  if (status) {
                      QAL_ERR(LOG_TAG, "Restart engine %d failed, status %d",
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
          st_stream_.detection_state_ |=  data->det_type_;
          if (st_stream_.detection_state_ & (CNN_DETECTED | VOP_DETECTED)) {
              QAL_DBG(LOG_TAG, "Second stage detected");
              st_stream_.detection_state_ = ENGINE_IDLE;
              if (!st_stream_.rec_config_->capture_requested) {
                  for (auto& eng: st_stream_.engines_) {
                      QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                                  eng->GetEngineId());
                      status = eng->GetEngine()->StopBuffering(&st_stream_);
                      if (status) {
                          QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed,"
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
      case ST_EV_CONCURRENT_STREAM: {
          st_stream_.CancelDelayedStop();
          for (auto& eng: st_stream_.engines_) {
              QAL_VERBOSE(LOG_TAG, "Stop buffering of engine %d",
                          eng->GetEngineId());
              status = eng->GetEngine()->StopBuffering(&st_stream_);
              if (status)
                  QAL_ERR(LOG_TAG, "Stop buffering of engine %d failed, status %d",
                          eng->GetEngineId(), status);
          }
          if (st_stream_.reader_)
              st_stream_.reader_->reset();
          // Reuse from Active state.
          TransitTo(ST_STATE_ACTIVE);
          status = st_stream_.ProcessInternalEvent(ev_cfg);
          if (status) {
              QAL_ERR(LOG_TAG, "Failed to process CONCURRENT_STREAM event,"
                               "status %d", status);
          }
          break;
      }
      default: {
          QAL_DBG(LOG_TAG, "Unhandled event %d", ev_cfg->id_);
          break;
      }
    }
    return status;
}
