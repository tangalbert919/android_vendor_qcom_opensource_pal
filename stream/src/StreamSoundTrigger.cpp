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

#define LOG_TAG "StreamSoundTrigger"

#include <unistd.h>

#include "StreamSoundTrigger.h"

#include "Session.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

StreamSoundTrigger::StreamSoundTrigger(struct qal_stream_attributes *sattr,
                                       struct qal_device *dattr,
                                       uint32_t no_of_devices,
                                       struct modifier_kv *modifiers __unused,
                                       uint32_t no_of_modifiers __unused,
                                       std::shared_ptr<ResourceManager> rm)
{
    uint32_t engine_id = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    stages_ = 1;
    session = nullptr;
    std::shared_ptr<Device> dev = nullptr;
    reader_ = nullptr;
    rec_event_ = nullptr;
    detection_state_ = ENGINE_IDLE;
    notification_state_ = GMM_DETECTED;
    gsl_engine_ = nullptr;
    memset(&detection_event_info_, 0, sizeof(struct detection_event_info));
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;

    QAL_DBG(LOG_TAG, "Enter");
    // TODO: handle modifiers later
    mNoOfModifiers = 0;
    mModifiers = (struct modifier_kv *) (nullptr);

    mStreamAttr = (struct qal_stream_attributes *)calloc
                  (1, sizeof(struct qal_stream_attributes));
    if (!mStreamAttr) {
        QAL_ERR(LOG_TAG, "malloc for stream attributes failed %s",
                strerror(errno));
        throw std::runtime_error("failed to malloc for stream attributes");
    }

    struct qal_channel_info *ch_info = (struct qal_channel_info *)calloc
                                       (1, sizeof(struct qal_channel_info));
    if (!ch_info) {
          QAL_ERR(LOG_TAG, "malloc for ch_info failed");
          free(mStreamAttr);
          throw std::runtime_error("failed to malloc for ch_info");
    }

    casa_osal_memcpy(mStreamAttr, sizeof(qal_stream_attributes),
                     sattr, sizeof(qal_stream_attributes));
    mStreamAttr->in_media_config.ch_info = ch_info;
    casa_osal_memcpy(mStreamAttr->in_media_config.ch_info,
                     sizeof(qal_channel_info),
                     sattr->in_media_config.ch_info,
                     sizeof(qal_channel_info));

    gsl_engine_ = SoundTriggerEngine::create(this, ST_SM_ID_SVA_GMM,
                                             &reader_, nullptr);
    if (!gsl_engine_ || !reader_) {
        QAL_ERR(LOG_TAG, "gsl engine creation failed");
        free(mStreamAttr->in_media_config.ch_info);
        free(mStreamAttr);
        throw std::runtime_error("failed to create gsl engine object");
    }
    engine_id = static_cast<uint32_t>(ST_SM_ID_SVA_GMM);
    registerSoundTriggerEngine(engine_id, gsl_engine_);

    // Is this required for Voice UI?
    QAL_ERR(LOG_TAG, "Updating device config for voice UI");
    bool isDeviceConfigUpdated = rm->updateDeviceConfigs(mStreamAttr,
                                                         no_of_devices,
                                                         dattr);

    if (isDeviceConfigUpdated)
        QAL_VERBOSE(LOG_TAG, "%s: Device config updated", __func__);

    QAL_VERBOSE(LOG_TAG, "gsl engine %pK created", gsl_engine_);

    QAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d",
                no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        dev = Device::getInstance(&dattr[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation is failed");
            free(mStreamAttr->in_media_config.ch_info);
            free(mStreamAttr);
            throw std::runtime_error("failed to create device object");
        }
        mDevices.push_back(dev);
        dev = nullptr;
    }

    rm->registerStream(this);
    QAL_DBG(LOG_TAG, "Exit");
}

int32_t StreamSoundTrigger::open()
{
    int32_t status = 0;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    QAL_DBG(LOG_TAG, "Enter, device count - %d", mDevices.size());
    if (!gsl_engine_) {
        QAL_ERR(LOG_TAG, "No GMM engine present, error!");
        status = -EINVAL;
        goto exit;
    }

    status = gsl_engine_->Open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "gsl engine open failed with status %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "gsl engine open successful");

    for (int32_t i=0; i < mDevices.size(); i++) {
        status = mDevices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device open failed with status %d", status);
            goto exit;
        }
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::close()
{
    // TODO: handle stop by stream states
    int32_t status = 0;
    int32_t i = 0;
    uint32_t id = 0;
    uint8_t *data = nullptr;
    SoundTriggerEngine *st_engine = nullptr;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    QAL_DBG(LOG_TAG, "Enter, device count - %d", mDevices.size());
    for (i = 0; i < mDevices.size(); i++) {
        QAL_ERR(LOG_TAG, "device %d name %s, going to close",
                mDevices[i]->getSndDeviceId(),
                mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->close();
        rm->deregisterDevice(mDevices[i]);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "closed the devices successfully");

    for (i = 0; i < active_engines_.size(); i++) {
        id = active_engines_[i].first;
        st_engine = active_engines_[i].second;
        if (!st_engine) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid st engine");
            goto exit;
        }
        status = st_engine->Close(this);
        if (status) {
            QAL_ERR(LOG_TAG, "st engine close failed with status %d", status);
            goto exit;
        }
        delete st_engine;
        st_engine = nullptr;
    }
    active_engines_.clear();
    gsl_engine_ = nullptr;
    QAL_DBG(LOG_TAG, "closed the engines successfully");

    for (i = 0; i < active_sm_data_.size(); i++) {
        id = active_sm_data_[i].first;
        data = active_sm_data_[i].second;
        if (data) {
            free(data);
            data = nullptr;
        }
    }
    active_sm_data_.clear();

exit:
    status = rm->deregisterStream(this);

    if (mStreamAttr) {
        free(mStreamAttr->in_media_config.ch_info);
        free(mStreamAttr);
        mStreamAttr = (struct qal_stream_attributes *)nullptr;
    }

    if (mVolumeData) {
        free(mVolumeData);
        mVolumeData = (struct qal_volume_data *)nullptr;
    }

    if (rec_config_) {
        free(rec_config_);
        rec_config_ = nullptr;
    }

    if (reader_) {
        free(reader_);
        reader_ = nullptr;
    }

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}


int32_t StreamSoundTrigger::start()
{
    int32_t status = 0;
    uint32_t id = 0;
    SoundTriggerEngine *st_engine = nullptr;

    std::lock_guard<std::mutex> lck(mStreamMutex);

    QAL_DBG(LOG_TAG, "Enter, mStreamAttr->direction - %d",
            mStreamAttr->direction);

    for (int i = 0; i < active_engines_.size(); i++) {
        id = active_engines_[i].first;
        st_engine = active_engines_[i].second;
        if (!st_engine) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid st engine");
            goto exit;
        }

        QAL_VERBOSE(LOG_TAG, "start recognition: sound trigger engine %u", id);
        status = st_engine->StartRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "StartRecognition failed, ST engine %u, status %d",
                    id, status);
            goto exit;
        }
    }

    for (int32_t i=0; i < mDevices.size(); i++) {
        QAL_ERR(LOG_TAG, "device %d name %s, going to start",
                mDevices[i]->getSndDeviceId(),
                mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx device start failed with status %d", status);
            goto exit;
        }
    }
    QAL_DBG(LOG_TAG, "Exit. mDevices started successfully status %d", status);
    for (int i = 0; i < mDevices.size(); i++) {
        rm->registerDevice(mDevices[i]);
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::stop()
{
    int32_t status = 0;
    uint32_t id = 0;
    SoundTriggerEngine *st_engine = nullptr;

    std::lock_guard<std::mutex> lck(mStreamMutex);

    QAL_DBG(LOG_TAG, "Enter, mStreamAttr->direction - %d",
            mStreamAttr->direction);

    for (int i = 0; i < active_engines_.size(); i++) {
        id = active_engines_[i].first;
        st_engine = active_engines_[i].second;
        if (!st_engine) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid st engine");
            goto exit;
        }

        QAL_VERBOSE(LOG_TAG, "stop recognition: sound trigger engine %u", id);
        status = st_engine->StopRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "StopRecognition failed, ST engine %u, status %d",
                    id, status);
            goto exit;
        }
    }

    for (int32_t i=0; i < mDevices.size(); i++) {
        QAL_ERR(LOG_TAG, "device %d name %s, going to stop",
                mDevices[i]->getSndDeviceId(),
                mDevices[i]->getQALDeviceName().c_str());

        status = mDevices[i]->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx device stop failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "devices stop successful");

    for (int i = 0; i < mDevices.size(); i++) {
        rm->deregisterDevice(mDevices[i]);
    }

    // reset ring buffer reader
    if (reader_) {
        reader_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

    // reset detection state
    status = setDetectionState(ENGINE_IDLE);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to set detection state to IDLE");
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::prepare()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);

    if (!gsl_engine_) {
        QAL_ERR(LOG_TAG, "No GMM engine present, error!");
        status = -EINVAL;
        goto exit;
    }

    status = gsl_engine_->Prepare(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "gsl engine prepare failed with status = %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::setStreamAttributes(
    struct qal_stream_attributes *sattr)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);

    if (!gsl_engine_) {
        QAL_ERR(LOG_TAG, "No GMM engine present, error!");
        status = -EINVAL;
        goto exit;
    }

    if (!sattr) {
        QAL_ERR(LOG_TAG, "Invalid param", session);
        status = -EINVAL;
        goto exit;
    }

    memset(mStreamAttr, 0, sizeof(qal_stream_attributes));
    casa_osal_memcpy(mStreamAttr, sizeof(qal_stream_attributes), sattr,
                     sizeof(qal_stream_attributes));
    status = gsl_engine_->SetConfig(this, MODULE, 0);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "gsl engine setConfig failed with status %d", status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::read(struct qal_buffer* buf)
{
    int32_t size = 0;

    QAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mStreamMutex);

    if (reader_) {
        size = reader_->read(buf->buffer, buf->size);
    } else {
        QAL_ERR(LOG_TAG, "Failed to read data as no valid reader present");
        return -EINVAL;
    }
    QAL_DBG(LOG_TAG, "Exit, stream read successful size - %d", size);

    return size;
}

int32_t StreamSoundTrigger::write(struct qal_buffer* buf __unused)
{
    return 0;
}

int32_t StreamSoundTrigger::registerCallBack(qal_stream_callback cb,
                                             void * cookie __unused)
{
    callback_ = cb;
    QAL_DBG(LOG_TAG, "callback_ = %pK", callback_);

    return 0;
}

int32_t StreamSoundTrigger::getCallBack(qal_stream_callback *cb)
{
    if (!cb) {
        QAL_ERR(LOG_TAG, "Invalid cb");
        return -EINVAL;
    }
    *cb = callback_;
    QAL_DBG(LOG_TAG, "callback_ = %pK", (*cb));

    return 0;
}

/* TODO:
    - Need to track vendor UUID
    - Need to parse BigSM for SVA 3.0
*/
int32_t StreamSoundTrigger::LoadSoundModel(
    struct qal_st_sound_model *sound_model)
{
    int32_t status = 0;
    int32_t i = 0;
    uint32_t engine_id = 0;
    SoundTriggerEngine *engine = nullptr;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;
    struct qal_st_sound_model *common_sm = nullptr;
    uint8_t *ptr = nullptr;
    uint8_t *sm_data = nullptr;
    uint8_t *sm_payload = nullptr;
    int32_t sm_size = 0;
    SML_GlobalHeaderType *global_hdr = nullptr;
    SML_HeaderTypeV3 *hdr_v3 = nullptr;
    SML_BigSoundModelTypeV3 *big_sm = nullptr;
    uint32_t sm_version = SML_MODEL_V2;

    QAL_DBG(LOG_TAG, "Enter");

    if (!gsl_engine_) {
        QAL_ERR(LOG_TAG, "No GMM engine present, error!");
        status = -EINVAL;
        goto exit;
    }

    if (!sound_model) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound_model param status %d", status);
        goto exit;
    }

    sound_model_type_ = sound_model->type;

    if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct qal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(*phrase_sm)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid phrase sound model params data size=%d, "
                   "data offset=%d, type=%d phrases=%d status %d",
                   phrase_sm->common.data_size, phrase_sm->common.data_offset,
                   phrase_sm->num_phrases, status);
            goto exit;
        }
        common_sm = (struct qal_st_sound_model*)&phrase_sm->common;
        recognition_mode_ = phrase_sm->phrases[0].recognition_mode;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        global_hdr = (SML_GlobalHeaderType *)sm_payload;
        if (global_hdr->magicNumber == SML_GLOBAL_HEADER_MAGIC_NUMBER) {
            // Parse sound model 3.0
            sm_version = SML_MODEL_V3;
            hdr_v3 = (SML_HeaderTypeV3 *)(sm_payload +
                                          sizeof(SML_GlobalHeaderType));
            stages_ = hdr_v3->numModels;
            QAL_DBG(LOG_TAG, "stages = %u", stages_);
            for (i = 0; i < stages_; i++) {
                big_sm = (SML_BigSoundModelTypeV3 *)(
                    sm_payload + sizeof(SML_GlobalHeaderType) +
                    sizeof(SML_HeaderTypeV3) +
                    (i * sizeof(SML_BigSoundModelTypeV3)));
                engine_id = static_cast<uint32_t>(big_sm->type);

                QAL_INFO(LOG_TAG, "type = %u, size = %u",
                         big_sm->type, big_sm->size);
                if (big_sm->type == ST_SM_ID_SVA_GMM) {
                    sm_size = big_sm->size +
                              sizeof(struct qal_st_phrase_sound_model);
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "sm_data malloc failed, status %d",
                                status);
                        goto exit;
                    }
                    casa_osal_memcpy(sm_data, sizeof(*phrase_sm),
                                     (char *)phrase_sm, sizeof(*phrase_sm));
                    common_sm = (struct qal_st_sound_model *)sm_data;
                    common_sm->data_size = big_sm->size;
                    common_sm->data_offset += sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (stages_ * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    casa_osal_memcpy(sm_data + sizeof(*phrase_sm), big_sm->size,
                                     (char *)phrase_sm + common_sm->data_offset,
                                     big_sm->size);
                    common_sm->data_offset = sizeof(*phrase_sm);
                    common_sm = (struct qal_st_sound_model *)&phrase_sm->common;
                    registerSoundModelData(engine_id, sm_data);
                    gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
                } else {
                    sm_size = big_sm->size;
                    ptr = (uint8_t *)sm_payload + sizeof(SML_GlobalHeaderType) +
                        sizeof(SML_HeaderTypeV3) +
                        (stages_ * sizeof(SML_BigSoundModelTypeV3)) +
                        big_sm->offset;
                    sm_data = (uint8_t *)calloc(1, sm_size);
                    if (!sm_data) {
                        status = -ENOMEM;
                        QAL_ERR(LOG_TAG, "Failed to alloc memory for sm_data");
                        goto exit;
                    }

                    casa_osal_memcpy(sm_data, sm_size, ptr, sm_size);

                    notification_state_ = notification_state_ | engine_id;
                    engine = SoundTriggerEngine::create(this, big_sm->type,
                                                        &reader_,
                                                        reader_->ringBuffer_);
                    if (!engine) {
                        QAL_ERR(LOG_TAG, "Failed to create engine for type %d",
                                big_sm->type);
                        status = -ENOENT;
                        goto exit;
                    }
                    registerSoundTriggerEngine(engine_id, engine);
                    registerSoundModelData(engine_id, sm_data);
                    engine->LoadSoundModel(this, sm_data, sm_size);
                }
            }
        } else {
            // Parse sound model 2.0
            sm_size = sizeof(*phrase_sm) + common_sm->data_size;
            sm_data = (uint8_t *)calloc(1, sm_size);
            if (!sm_data) {
                status = -ENOMEM;
                QAL_ERR(LOG_TAG, "Failed to allocate memory for sm_data");
                goto exit;
            }
            casa_osal_memcpy(sm_data, sizeof(*phrase_sm),
                             (uint8_t *)phrase_sm, sizeof(*phrase_sm));
            casa_osal_memcpy(sm_data + sizeof(*phrase_sm), common_sm->data_size,
                             (uint8_t*)phrase_sm + common_sm->data_offset,
                             common_sm->data_size);
            engine_id = static_cast<uint32_t>(ST_SM_ID_SVA_GMM);
            registerSoundModelData(engine_id, sm_data);
            gsl_engine_->LoadSoundModel(this, sm_data, sm_size);
        }
    } else if (sound_model->type == QAL_SOUND_MODEL_TYPE_GENERIC) {
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct qal_st_sound_model))) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid generic sound model params data size=%d,"
                    " data offset=%d status %d", sound_model->data_size,
                    sound_model->data_offset, status);
            goto exit;
        }
        // TODO: add enum
        recognition_mode_ = 0x1;
        common_sm = sound_model;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        sm_size = sizeof(struct qal_st_sound_model) + common_sm->data_size;
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Unknown sound model type - %d status %d",
                sound_model->type, status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status, %d", status);

    return status;
}

// TODO: look into how cookies are used here
int32_t StreamSoundTrigger::SendRecognitionConfig(
    struct qal_st_recognition_config *config)
{
    int32_t status = 0;
    struct st_param_header *param_hdr = NULL;
    struct st_hist_buffer_info *hist_buf = NULL;
    struct st_det_perf_mode_info *det_perf_mode = NULL;
    uint8_t *opaque_ptr = NULL;
    unsigned int opaque_size = 0, conf_levels_payload_size = 0;
    int32_t confidence_threshold = 0;
    uint32_t hist_buffer_duration = 0;
    uint32_t pre_roll_duration = 0;
    listen_model_indicator_enum type;
    uint32_t conf_levels_intf_version = 0;
    uint8_t *conf_levels = NULL;
    uint32_t num_conf_levels = 0;
    bool capture_requested = false;

    QAL_DBG(LOG_TAG, "Enter");
    if (!gsl_engine_) {
        QAL_ERR(LOG_TAG, "No GMM engine present, error!");
        status = -EINVAL;
        goto exit;
    }

    if (!active_sm_data_.size()) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sm data status %d", status);
        goto exit;
    }

    // TODO: need logic to check for existing config
    // free that before allocating

    // Keep a copy of recognition config in StreamSoundTrigger
    rec_config_ = (struct qal_st_recognition_config *)calloc(1,
        sizeof(struct qal_st_recognition_config) + config->data_size);
    if (!rec_config_) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "Failed to allocate rec_config_ status %d", status);
        goto exit;
    }

    casa_osal_memcpy(rec_config_, sizeof(struct qal_st_recognition_config),
                     config, sizeof(struct qal_st_recognition_config));
    casa_osal_memcpy((uint8_t *)rec_config_ + config->data_offset,
                     config->data_size,
                     (uint8_t *)config + config->data_offset,
                     config->data_size);

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
                        goto exit;
                    }
                    status = ParseOpaqueConfLevels(opaque_ptr,
                                                   conf_levels_intf_version,
                                                   &conf_levels,
                                                   &num_conf_levels);
                    if (status) {
                        QAL_ERR(LOG_TAG, "Failed to parse opaque conf levels");
                        goto exit;
                    }

                    opaque_size += sizeof(struct st_param_header) +
                        conf_levels_payload_size;
                    opaque_ptr += sizeof(struct st_param_header) +
                        conf_levels_payload_size;
                    if (status) {
                        QAL_ERR(LOG_TAG, "Parse conf levels failed(status=%d)",
                                status);
                        status = -EINVAL;
                        goto exit;
                    }
                    break;
                case ST_PARAM_KEY_HISTORY_BUFFER_CONFIG:
                    if (param_hdr->payload_size !=
                        sizeof(struct st_hist_buffer_info)) {
                        QAL_ERR(LOG_TAG, "History buffer config format error");
                        status = -EINVAL;
                        goto exit;
                    }
                    hist_buf = (struct st_hist_buffer_info *)(opaque_ptr +
                        sizeof(struct st_param_header));
                    hist_buffer_duration =
                        hist_buf->hist_buffer_duration_msec;
                    pre_roll_duration =
                        hist_buf->pre_roll_duration_msec;
                    QAL_DBG(LOG_TAG, "recognition config history buf len = %d, "
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
                        goto exit;
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
                    goto exit;
            }
        }
    } else {
        status = FillConfLevels(config, &conf_levels, &num_conf_levels);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to parse conf levels from rc config");
            goto exit;
        }
    }

    gsl_engine_->UpdateConfLevels(this, rec_config_,
                                  conf_levels, num_conf_levels);

    // Update capture requested flag to gsl engine
    if (!rec_config_->capture_requested && active_engines_.size() == 1)
        capture_requested = false;
    else
        capture_requested = true;
    gsl_engine_->SetCaptureRequested(capture_requested);

exit:
    if (conf_levels)
        free(conf_levels);

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::getParameters(uint32_t param_id, void **payload)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter, get parameter %u", param_id);
    if (active_engines_.size()) {
        SoundTriggerEngine *stEngine = active_engines_[0].second;
        status = stEngine->getParameters(param_id, payload);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to get parameters from st engine");
        }
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "No sound trigger engine present");
    }

    return status;
}

int32_t StreamSoundTrigger::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    struct qal_st_sound_model *sound_model = nullptr;
    struct qal_st_recognition_config *rc_config = nullptr;
    SoundTriggerEngine *engine = nullptr;

    QAL_DBG(LOG_TAG, "Enter, param id %d", param_id);

    std::lock_guard<std::mutex> lck(mStreamMutex);
    // Stream may not know about tags, so use setParameters instead of setConfig
    switch (param_id) {
    case QAL_PARAM_ID_LOAD_SOUND_MODEL:
    {
        sound_model = (struct qal_st_sound_model *)payload;
        status = LoadSoundModel(sound_model);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to parse sound model status %d", status);
            goto exit;
        }
        break;
    }
    case QAL_PARAM_ID_START_RECOGNITION:
    {
        rc_config = (struct qal_st_recognition_config *)payload;
        status = SendRecognitionConfig(rc_config);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to parse recognition config status %d",
                    status);
            goto exit;
        }

        // register callback function in engine
        registerCallBack(handleDetectionEvent, nullptr);
        break;
    }
    case QAL_PARAM_ID_STOP_BUFFERING:
    {
        for (int i = 0; i < active_engines_.size(); i++) {
            engine = active_engines_[i].second;
            status = engine->StopBuffering(this);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Failed to stop buffering, status = %d",
                        status);
                goto exit;
            }
        }

        // reset ring buffer reader
        if (reader_) {
            reader_->reset();
        } else {
            status = -EINVAL;
            goto exit;
        }

        // reset detection state
        status = setDetectionState(ENGINE_IDLE);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to set detection state to IDLE");
            goto exit;
        }

        break;
    }
    default:
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Unsupported param id %u status %d", param_id, status);
        goto exit;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

void StreamSoundTrigger::registerSoundTriggerEngine(
    uint32_t id,
    SoundTriggerEngine *stEngine)
{
    int index;

    if (!getSoundTriggerEngine(&index, id)) {
        QAL_ERR(LOG_TAG, "Sound Model with id %u already registered", id);
        return;
    }
    active_engines_.push_back(std::make_pair(id, stEngine));
}

void StreamSoundTrigger::deregisterSoundTriggerEngine(uint32_t id)
{
    int index;
    SoundTriggerEngine *engine = nullptr;
    std::vector<std::pair<uint32_t, SoundTriggerEngine *>>::iterator iter;

    if (getSoundTriggerEngine(&index, id)) {
        QAL_ERR(LOG_TAG,  "No Sound trigger engine found for id %u", id);
        return;
    }
    iter += index;
    engine = (*iter).second;
    if (engine) {
        QAL_VERBOSE(LOG_TAG, "Release sound trigger engine %pK", engine);
        delete engine;
    }
    active_engines_.erase(iter);
}

int32_t StreamSoundTrigger::getSoundTriggerEngine(int *index, uint32_t sm_id)
{
    int32_t status = -ENOENT;

    for (int i = 0; i < active_engines_.size(); i++) {
        if (active_engines_[i].first == sm_id) {
            QAL_VERBOSE(LOG_TAG, "Sound trigger engine found for id %u", sm_id);
            *index = i;
            status = 0;
            break;
        }
    }
    return status;
}

void StreamSoundTrigger::registerSoundModelData(uint32_t id, uint8_t *data)
{
    int index;

    if (!getSoundModelData(&index, id)) {
        QAL_ERR(LOG_TAG, "Sound Model data with id %u already registered", id);
        return;
    }
    active_sm_data_.push_back(std::make_pair(id, data));
}

void StreamSoundTrigger::deregisterSoundModelData(uint32_t id)
{
    int index = 0;
    uint8_t *data = nullptr;
    std::vector<std::pair<uint32_t, uint8_t *>>::iterator iter;

    if (getSoundModelData(&index, id)) {
        QAL_ERR(LOG_TAG,  "No Sound trigger engine found for id %u", id);
        return;
    }
    iter += index;
    data = (*iter).second;
    if (data) {
        QAL_VERBOSE(LOG_TAG, "Release sound model data %pK", data);
        free(data);
    }
    active_sm_data_.erase(iter);
}

int32_t StreamSoundTrigger::getSoundModelData(int *index, uint32_t sm_id)
{
    int32_t status = -ENOENT;

    for (int i = 0; i < active_sm_data_.size(); i++) {
        if (active_sm_data_[i].first == sm_id) {
            QAL_VERBOSE(LOG_TAG, "Sound trigger engine found for id %u", sm_id);
            *index = i;
            status = 0;
            break;
        }
    }

    return status;
}

// Callback function for detection engine of first stage
int32_t StreamSoundTrigger::handleDetectionEvent(
    qal_stream_handle_t *stream_handle,
    uint32_t event_id,
    uint32_t *event_data,
    void * cookie __unused)
{
    int32_t status = 0;
    StreamSoundTrigger *s = nullptr;

    QAL_DBG(LOG_TAG, "Enter. Event detected on GECKO, event id = %u", event_id);
    if (!stream_handle || !event_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "No stream handle or event data provided status %d",
                status);
        goto exit;
    }

    s = static_cast<StreamSoundTrigger *>(stream_handle);
    // Parse info from event_data for second stage detection
    status = s->ParseDetectionPayload(event_id, event_data);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to parse detection payload with ret = %d",
                status);
        goto exit;
    }

    // Mark GMM detected here if no lab needed
    if (!s->rec_config_->capture_requested && s->active_engines_.size() == 1)
        s->setDetectionState(GMM_DETECTED);

    // Notify Engine and client
    status = s->SetDetected(true);

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::SetDetected(bool detected)
{
    int32_t status = 0;
    SoundTriggerEngine *engine = nullptr;

    for (int i = 0; i < active_engines_.size(); i++) {
        QAL_VERBOSE(LOG_TAG, "Notify detection event %d to ST engine %d",
                    detected, i);
        engine = active_engines_[i].second;
        engine->SetDetected(detected);
    }

    return status;
}

int32_t StreamSoundTrigger::ParseDetectionPayload(uint32_t event_id,
                                                  uint32_t *event_data)
{
    int32_t status = 0;
    int32_t i = 0;
    uint32_t parsed_size = 0;
    uint32_t payload_size = 0;
    uint32_t event_size = 0;
    uint8_t *ptr = nullptr;
    struct event_id_detection_engine_generic_info_t *generic_info = nullptr;
    struct detection_event_info_header_t *event_header = nullptr;
    struct confidence_level_info_t *confidence_info = nullptr;
    struct keyword_position_info_t *keyword_position_info = nullptr;
    struct detection_timestamp_info_t *detection_timestamp_info = nullptr;
    struct ftrt_data_info_t *ftrt_info = nullptr;

    if (!event_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid event data status %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter");
    // parse event_id_detection_engine_generic_info
    generic_info =
        (struct event_id_detection_engine_generic_info_t *)event_data;
    payload_size = sizeof(struct event_id_detection_engine_generic_info_t);
    detection_event_info_.status = generic_info->status;
    event_size = generic_info->payload_size;
    ptr = (uint8_t *)event_data + payload_size;
    QAL_INFO(LOG_TAG, "status = %u, event_size = %u",
             detection_event_info_.status, event_size);
    if (status || !event_size) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid detection payload");
        goto exit;
    }

    // parse variable payload
    while (parsed_size < event_size) {
        QAL_DBG(LOG_TAG, "parsed_size = %u, event_size = %u",
                parsed_size, event_size);
        event_header = (struct detection_event_info_header_t *)ptr;
        uint32_t keyId = event_header->key_id;
        payload_size = event_header->payload_size;
        QAL_DBG(LOG_TAG, "key id = %u, payload_size = %u",
                keyId, payload_size);
        ptr += sizeof(struct detection_event_info_header_t);
        parsed_size += sizeof(struct detection_event_info_header_t);

        switch (keyId) {
        case KEY_ID_CONFIDENCE_LEVELS_INFO:
            confidence_info = (struct confidence_level_info_t *)ptr;
            detection_event_info_.num_confidence_levels =
                confidence_info->number_of_confidence_values;
            QAL_DBG(LOG_TAG, "num_confidence_levels = %u",
                    detection_event_info_.num_confidence_levels);
            for (i = 0; i < detection_event_info_.num_confidence_levels; i++) {
                detection_event_info_.confidence_levels[i] =
                    confidence_info->confidence_levels[i];
                QAL_VERBOSE(LOG_TAG, "confidence_levels[%d] = %u", i,
                            detection_event_info_.confidence_levels[i]);
            }
            break;
        case KEY_ID_KWD_POSITION_INFO:
            keyword_position_info = (struct keyword_position_info_t *)ptr;
            detection_event_info_.kw_start_timestamp_lsw =
                keyword_position_info->kw_start_timestamp_lsw;
            detection_event_info_.kw_start_timestamp_msw =
                keyword_position_info->kw_start_timestamp_msw;
            detection_event_info_.kw_end_timestamp_lsw =
                keyword_position_info->kw_end_timestamp_lsw;
            detection_event_info_.kw_end_timestamp_msw =
                keyword_position_info->kw_end_timestamp_msw;
            QAL_DBG(LOG_TAG, "start_lsw = %u, start_msw = %u, "
                    "end_lsw = %u, end_msw = %u",
                    detection_event_info_.kw_start_timestamp_lsw,
                    detection_event_info_.kw_start_timestamp_msw,
                    detection_event_info_.kw_end_timestamp_lsw,
                    detection_event_info_.kw_end_timestamp_msw);
            break;
        case KEY_ID_TIMESTAMP_INFO:
            detection_timestamp_info = (struct detection_timestamp_info_t *)ptr;
            detection_event_info_.detection_timestamp_lsw =
                detection_timestamp_info->detection_timestamp_lsw;
            detection_event_info_.detection_timestamp_msw =
                detection_timestamp_info->detection_timestamp_msw;
            QAL_DBG(LOG_TAG, "timestamp_lsw = %u, timestamp_msw = %u",
                    detection_event_info_.detection_timestamp_lsw,
                    detection_event_info_.detection_timestamp_msw);
            break;
        case KEY_ID_FTRT_DATA_INFO:
            ftrt_info = (struct ftrt_data_info_t *)ptr;
            detection_event_info_.ftrt_data_length_in_us =
                ftrt_info->ftrt_data_length_in_us;
            QAL_DBG(LOG_TAG, "ftrt_data_length_in_us = %u",
                    detection_event_info_.ftrt_data_length_in_us);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid key id %u status %d", keyId, status);
            goto exit;
        }
        ptr += payload_size;
        parsed_size += payload_size;
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::notifyClient()
{
    int32_t status = 0;
    SoundTriggerEngine *engine = nullptr;

    QAL_INFO(LOG_TAG, "Notify detection event back to client");
    status = GenerateCallbackEvent(&rec_event_);
    if (!status)
        rec_config_->callback(rec_event_, rec_config_->cookie);
    // release rec_event_ after callback so that
    // rec_event_ can be used in next detection
    if (rec_event_) {
        if (rec_event_->media_config.ch_info) {
            free(rec_event_->media_config.ch_info);
            rec_event_->media_config.ch_info = nullptr;
        }
        if (sound_model_type_ == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
            struct qal_st_phrase_recognition_event *phrase_event =
                (struct qal_st_phrase_recognition_event *)rec_event_;
            free(phrase_event);
        } else {
            free(rec_event_);
        }
        rec_event_ = nullptr;
    }

    if (!rec_config_->capture_requested)
        setParameters(QAL_PARAM_ID_STOP_BUFFERING, nullptr);

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::setDetectionState(uint32_t state)
{
    int32_t status = 0;
    SoundTriggerEngine *engine = nullptr;

    QAL_DBG(LOG_TAG, "Enter");
    switch (state)
    {
        case ENGINE_IDLE:
            detection_state_ = ENGINE_IDLE;
            break;
        case GMM_DETECTED:
        case CNN_DETECTED:
        case VOP_DETECTED:
            detection_state_ = detection_state_ | state;
            break;
        case CNN_REJECTED:
        case VOP_REJECTED:
            QAL_DBG(LOG_TAG, "Second stage rejected, stop buffering");
            status = setParameters(QAL_PARAM_ID_STOP_BUFFERING, nullptr);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to stop buffering");
                goto exit;
            }
            break;
        default:
            QAL_ERR(LOG_TAG, "Invalid state %x", state);
            status = -EINVAL;
            goto exit;
    }

    QAL_INFO(LOG_TAG, "detection state = %u, notification state = %u",
             detection_state_, notification_state_);
    if (detection_state_ == notification_state_) {
        QAL_INFO(LOG_TAG, "Notify detection event back to client");
        notifyClient();
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t StreamSoundTrigger::setVolume(struct qal_volume_data * volume __unused)
{
    return 0;
}
int32_t StreamSoundTrigger::setMute(bool state __unused)
{
    return 0;
}

int32_t StreamSoundTrigger::setPause()
{
    return 0;
}

int32_t StreamSoundTrigger::setResume()
{
    return 0;
}

int32_t StreamSoundTrigger::GenerateCallbackEvent(
    struct qal_st_recognition_event **event)
{
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
    if (sound_model_type_ == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
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
            return ENOMEM;
        }

        phrase_event->num_phrases = rec_config_->num_phrases;
        memcpy(phrase_event->phrase_extras, rec_config_->phrases,
               phrase_event->num_phrases *
               sizeof(struct qal_st_phrase_recognition_extra));

        *event = &(phrase_event->common);
        (*event)->media_config.ch_info = nullptr;
        (*event)->status = 0;
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

        ch_info = (struct qal_channel_info *)
                  calloc(1, sizeof(struct qal_channel_info));
        if (!ch_info) {
            QAL_ERR(LOG_TAG, "Failed to alloc memory for channel info");
            return ENOMEM;
        }
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
        conf_levels->num_sound_models = stages_;
        // TODO: update user conf levels
        for (int i = 0; i < conf_levels->num_sound_models; i++) {
            conf_levels->conf_levels[i].sm_id = ST_SM_ID_SVA_GMM;
            conf_levels->conf_levels[i].num_kw_levels = 1;
            conf_levels->conf_levels[i].kw_levels[0].kw_level =
                detection_event_info_.confidence_levels[i];
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

        /* Pack the opaque data detection time structure
           TODO: add support for 2nd stage detection timestamp */
        param_hdr = (struct st_param_header *)opaque_data;
        param_hdr->key_id = ST_PARAM_KEY_TIMESTAMP;
        param_hdr->payload_size = sizeof(struct st_timestamp_info);
        opaque_data += sizeof(struct st_param_header);
        timestamps = (struct st_timestamp_info *)opaque_data;
        timestamps->version = 0x1;
        timestamps->first_stage_det_event_time = 1000 *
            ((uint64_t)detection_event_info_.detection_timestamp_lsw +
            ((uint64_t)detection_event_info_.detection_timestamp_msw << 32));
    }
    // TODO: handle for generic sound model
    QAL_DBG(LOG_TAG, "Exit");

    return 0;
}

int32_t StreamSoundTrigger::isSampleRateSupported(uint32_t sampleRate)
{
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

int32_t StreamSoundTrigger::isChannelSupported(uint32_t numChannels)
{
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

int32_t StreamSoundTrigger::isBitWidthSupported(uint32_t bitWidth)
{
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

int32_t StreamSoundTrigger::addRemoveEffect(qal_audio_effect_t effect,
                                            bool enable)
{
    QAL_ERR(LOG_TAG, "Function not supported");

    return -ENOSYS;
}

// TBD: to be tested, Yidong, is this enough?
int32_t StreamSoundTrigger::switchDevice(Stream* stream_handle,
                                         uint32_t no_of_devices,
                                         struct qal_device *device_array)
{
    int32_t status = -EINVAL;
    SoundTriggerEngine *engine = nullptr;

    std::lock_guard<std::mutex> lck(mStreamMutex);
    std::shared_ptr<Device> dev = nullptr;
    if (no_of_devices == 0 || !device_array) {
        QAL_ERR("%s: invalid param for device switch", __func__);
        status = -EINVAL;
        goto error_1;
    }

    for (int i = 0; i < active_engines_.size(); i++) {
        uint32_t id = active_engines_[i].first;
        engine = active_engines_[i].second;
        QAL_VERBOSE(LOG_TAG, "stop recognition for sound trigger engine %u",
                    id);
        status = engine->StopRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "StopRecognition failed for sound trigger "
                    "engine %u with status %d", id, status);
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
        QAL_ERR(LOG_TAG, "device %d name %s, going to stop",
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

        //Check with RM if the configuration given can work or not
        //for e.g., if incoming stream needs 24 bit device thats also
        //being used by another stream, then the other stream should route

        dev = Device::getInstance((struct qal_device *)&mDevices[i] , rm);

        if (!dev) {
            QAL_ERR(LOG_TAG, "%s: Device creation failed", __func__);
            if (mStreamAttr) {
                free(mStreamAttr->in_media_config.ch_info);
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

    for (int i = 0; i < active_engines_.size(); i++) {
        uint32_t id = active_engines_[i].first;
        engine = active_engines_[i].second;
        QAL_VERBOSE(LOG_TAG, "start recognition for sound trigger engine %u",
                    id);
        status = engine->StartRecognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "StartRecognition failed for sound trigger "
                    "engine %u with status %d", id, status);
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

int32_t StreamSoundTrigger::ParseOpaqueConfLevels(
    void *opaque_conf_levels,
    uint32_t version,
    uint8_t **out_conf_levels,
    uint32_t *out_num_conf_levels)
{
    int32_t status = 0;
    struct st_confidence_levels_info *conf_levels = nullptr;
    struct st_confidence_levels_info_v2 *conf_levels_v2 = nullptr;
    struct st_sound_model_conf_levels *sm_levels = nullptr;
    struct st_sound_model_conf_levels_v2 *sm_levels_v2 = nullptr;
    uint8_t confidence_level = 0;
    uint8_t confidence_level_v2 = 0;
    bool gmm_conf_found = false;
    listen_model_indicator_enum type;
    SoundTriggerEngine *engine = nullptr;
    uint32_t engine_id;

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
                for (int j = 0; j < active_engines_.size(); j++) {
                    engine_id = active_engines_[i].first;
                    type = static_cast<listen_model_indicator_enum>(engine_id);
                    if (type == sm_levels->sm_id) {
                        engine = active_engines_[i].second;
                        engine->UpdateConfLevels(this, rec_config_,
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
                for (int j = 0; j < active_engines_.size(); j++) {
                    engine_id = active_engines_[i].first;
                    type = static_cast<listen_model_indicator_enum>(engine_id);
                    if (type == sm_levels_v2->sm_id) {
                        engine = active_engines_[i].second;
                        engine->UpdateConfLevels(this, rec_config_,
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
    uint32_t *out_num_conf_levels)
{
    int32_t status = 0;
    uint32_t engine_id = 0;
    uint8_t *sm_data = nullptr;
    uint32_t num_conf_levels = 0;
    unsigned int user_level, user_id;
    unsigned int i = 0, j = 0;
    uint8_t *conf_levels = nullptr;
    unsigned char *user_id_tracker = nullptr;
    struct qal_st_phrase_sound_model *phrase_sm = nullptr;
    listen_model_indicator_enum type = ST_SM_ID_SVA_NONE;

    for (i = 0; i < active_sm_data_.size(); i++) {
        engine_id = active_sm_data_[i].first;
        type = static_cast<listen_model_indicator_enum>(engine_id);
        if (type == ST_SM_ID_SVA_GMM) {
            sm_data = active_sm_data_[i].second;
        }
    }

    if (!sm_data) {
        QAL_ERR(LOG_TAG, "No GMM sound model found");
        status = -EINVAL;
        goto exit;
    }
    phrase_sm = (struct qal_st_phrase_sound_model *)sm_data;

    QAL_DBG(LOG_TAG, "Enter");

    if (!phrase_sm || !config) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "invalid input status %d", status);
        goto exit;
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
    uint32_t version)
{
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

    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}
