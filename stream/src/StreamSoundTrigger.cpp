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

#include "StreamSoundTrigger.h"
#include "Session.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"
#include <unistd.h>

StreamSoundTrigger::StreamSoundTrigger(struct qal_stream_attributes *sattr,
                                       struct qal_device *dattr,
                                       uint32_t no_of_devices,
                                       struct modifier_kv *modifiers,
                                       uint32_t no_of_modifiers,std::shared_ptr<ResourceManager> rm)
{
    mutex.lock();
    stages = 1;
    session = NULL;
    dev = nullptr;
    reader_ = NULL;
    recEvent = NULL;
    detectionState = ENGINE_IDLE;
    notificationState = GMM_DETECTED;
    memset(&detectionEventInfo, 0, sizeof(struct detection_event_info));
    inBufSize = BUF_SIZE_CAPTURE;
    outBufSize = BUF_SIZE_PLAYBACK;
    inBufCount = NO_OF_BUF;
    outBufCount = NO_OF_BUF;

    QAL_DBG(LOG_TAG, "Enter.");
    uNoOfModifiers = no_of_modifiers;
    attr = (struct qal_stream_attributes *) malloc(sizeof(struct qal_stream_attributes));
    if (!attr) {
        QAL_ERR(LOG_TAG, "malloc for stream attributes failed %s", strerror(errno));
        mutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }
    casa_osal_memcpy (attr, sizeof(qal_stream_attributes), sattr,
                      sizeof(qal_stream_attributes));

    session = Session::makeSession(rm, sattr);

    if (!session) {
        QAL_ERR(LOG_TAG, "session creation failed");
        free(attr);
        mutex.unlock();
        throw std::runtime_error("failed to create session object");
    }
    QAL_VERBOSE(LOG_TAG, "session %pK created", session);

    QAL_VERBOSE(LOG_TAG, "Create new Devices with no_of_devices - %d", no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        dev = Device::create(&dattr[i] , rm);
        if (!dev) {
            QAL_ERR(LOG_TAG, "Device creation is failed");
            free(attr);
            mutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        devices.push_back(dev);
        dev = nullptr;
    }
    mutex.unlock();
    rm->registerStream(this);
    QAL_DBG(LOG_TAG, "Exit.");
}

int32_t StreamSoundTrigger::open()
{
    int32_t status = 0;
    mutex.lock();
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK device count - %d", session,
            devices.size());
    status = session->open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session open failed with status %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session open successful");

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device open failed with status %d", status);
            goto exit;
        }
    }

    QAL_DBG(LOG_TAG, "Exit. StreamSoundTrigger opened status %d", status);

exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::close()
{
    // TODO: handle stop by stream states
    int32_t status = 0;
    mutex.lock();

    QAL_DBG(LOG_TAG,  "Enter. session handle - %pK device count - %d", session,
            devices.size());
    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->close();
        rm->deregisterDevice(devices[i]);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "device close is failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "closed the devices successfully");

    status = session->close(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session close failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. closed the session successfully status %d", status);

exit:
    mutex.unlock();
    status = rm->deregisterStream(this);
    QAL_ERR(LOG_TAG, "status - %d", status);
    return status;
}


int32_t StreamSoundTrigger::start()
{
    int32_t status = 0;
    mutex.lock();

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK attr->direction - %d", session,
            attr->direction);

    for (int i = 0; i < activeEngines.size(); i++) {
        uint32_t id = activeEngines[i].first;
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        QAL_VERBOSE(LOG_TAG, "start recognition for sound trigger engine %u", id);
        status = stEngine->start_recognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "start_recognition failed for sound trigger engine %u with status %d",
                    id, status);
            goto exit;
        }
    }

    status = session->prepare(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Tx session prepare is failed with status %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session prepare successful");

    status = session->start(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to start session, status = %d", status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "session start successful");

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx device start is failed with status %d", status);
            goto exit;
        }
    }

    for (int i = 0; i < devices.size(); i++) {
        rm->registerDevice(devices[i]);
    }
    QAL_DBG(LOG_TAG, "Exit. devices started successfully status %d", status);
    for (int i = 0; i < devices.size(); i++) {
        rm->registerDevice(devices[i]);
    }

exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::stop()
{
    int32_t status = 0;

    mutex.lock();

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK attr->direction - %d device count %d",
            session, attr->direction, devices.size());

    for (int i = 0; i < activeEngines.size(); i++) {
        uint32_t id = activeEngines[i].first;
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        QAL_VERBOSE(LOG_TAG, "stop recognition for sound trigger engine %u", id);
        status = stEngine->stop_recognition(this);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "stop_recognition failed for sound trigger engine %u with status %d",
                    id, status);
            goto exit;
        }
    }

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Tx device stop failed with status %d", status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "devices stop successful");

    status = session->stop(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to stop session, status = %d", status);
        goto exit;
    }

    for (int i = 0; i < devices.size(); i++) {
        rm->deregisterDevice(devices[i]);
    }
    QAL_DBG(LOG_TAG, "Exit. session stop successful");

exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::prepare()
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    mutex.lock();
    status = session->prepare(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session prepare failed with status = %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. status - %d", status);
exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = 0;

    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);

    memset(attr, 0, sizeof(qal_stream_attributes));
    mutex.lock();
    casa_osal_memcpy (attr, sizeof(qal_stream_attributes), sattr,
                      sizeof(qal_stream_attributes));
    mutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //TODO:gkv or ckv or tkv need to pass
    if (0 != status) {
        QAL_ERR(LOG_TAG, "session setConfig failed with status %d", status);
        goto exit;
    }
    QAL_DBG(LOG_TAG, "Exit. session setConfig successful");

exit:
    return status;
}

int32_t StreamSoundTrigger::read(struct qal_buffer* buf)
{
    int32_t size;
    QAL_DBG(LOG_TAG, "Enter. session handle - %pK", session);
    mutex.lock();
    if (reader_)
        size = reader_->read(buf->buffer, buf->size);
    else {
        QAL_ERR(LOG_TAG, "Failed to read data as no valid reader present");
        return -EINVAL;
    }
    mutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. session read successful size - %d", size);
    return size;
}

int32_t StreamSoundTrigger::write(struct qal_buffer* buf)
{
    return 0;
}

int32_t StreamSoundTrigger::registerCallBack(qal_stream_callback cb)
{
    callBack = cb;
    QAL_DBG(LOG_TAG, "callBack = %pK", callBack);
    return 0;
}

int32_t StreamSoundTrigger::getCallBack(qal_stream_callback *cb)
{
    if (!cb) {
        QAL_ERR(LOG_TAG, "Invalid cb");
        return -EINVAL;
    }
    *cb = callBack;
    QAL_DBG(LOG_TAG, "callBack = %pK", (*cb));
    return 0;
}

/* TODO:
    - Need to track vendor UUID
    - Need to parse BigSM for SVA 3.0
*/
int32_t StreamSoundTrigger::parse_sound_model(struct qal_st_sound_model *sound_model)
{
    int32_t status = 0;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;
    uint8_t *sm_payload = NULL;
    int32_t sm_size = 0;
    SML_GlobalHeaderType *global_hdr;
    SML_HeaderTypeV3 *hdr_v3;
    uint32_t sm_version = SML_MODEL_V2;

    QAL_DBG(LOG_TAG, "Enter.");

    if (!sound_model) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid sound_model param status %d", status);
        goto exit;
    }

    if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        phrase_sm = (struct qal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(struct qal_st_phrase_sound_model)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid phrase sound model params data size=%d, data offset=%d, type=%d phrases=%d status %d",
                   phrase_sm->common.data_size, phrase_sm->common.data_offset,
                   phrase_sm->num_phrases, status);
            goto exit;
        }
        common_sm = (struct qal_st_sound_model*)&phrase_sm->common;
        recognition_mode = phrase_sm->phrases[0].recognition_mode;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        sm_size = sizeof(*phrase_sm) + common_sm->data_size;
        global_hdr = (SML_GlobalHeaderType *)sm_payload;
        if (global_hdr->magicNumber == SML_GLOBAL_HEADER_MAGIC_NUMBER) {
            sm_version = SML_MODEL_V3;
            hdr_v3 = (SML_HeaderTypeV3 *)(sm_payload + sizeof(SML_GlobalHeaderType));
            stages = hdr_v3->numModels;
            QAL_DBG(LOG_TAG, "stages = %u", stages);
        }
    } else if (sound_model->type == QAL_SOUND_MODEL_TYPE_GENERIC) {
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct qal_st_sound_model))) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid generic sound model params data size=%d, data offset=%d status %d",
                    sound_model->data_size,
                    sound_model->data_offset, status);
            goto exit;
        }
        recognition_mode = 0x1; //TO-DO: add enum
        common_sm = sound_model;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        sm_size = sizeof(struct qal_st_sound_model) + common_sm->data_size;
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Unknown sound model type - %d status %d",
                sound_model->type, status);
        goto exit;
    }

    sm_data = (uint8_t*) calloc(1, sm_size);
    if (!sm_data) {
        status = -ENOMEM;
        goto exit;
    }

    sound_model_type = sound_model->type;

    if (sound_model_type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        casa_osal_memcpy(sm_data, sizeof(*phrase_sm), (uint8_t*)phrase_sm,
                         sizeof(*phrase_sm));
        casa_osal_memcpy((uint8_t*)sm_data + sizeof(*phrase_sm), phrase_sm->common.data_size,
               (uint8_t*)phrase_sm + phrase_sm->common.data_offset,
               phrase_sm->common.data_size);
        recognition_mode = phrase_sm->phrases[0].recognition_mode;

        QAL_VERBOSE(LOG_TAG, "phrase recognition mode - %d", recognition_mode);

    } else {
        casa_osal_memcpy(sm_data, sizeof(*common_sm), (uint8_t*)common_sm,
                         sizeof(*common_sm));
        casa_osal_memcpy((uint8_t*)sm_data + sizeof(*common_sm), common_sm->data_size,
               (uint8_t*)common_sm + common_sm->data_offset, common_sm->data_size);
    }

    QAL_DBG(LOG_TAG, "Exit. status - %d", status);
exit:
    return status;
}

//TODO:	- look into how cookies are used here
int32_t StreamSoundTrigger::parse_rc_config(struct qal_st_recognition_config *rc_config)
{
    int32_t status = 0;

    unsigned char **out_payload;
    unsigned int *out_payload_size;

    QAL_DBG(LOG_TAG, "Enter.");

    if (!sm_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid smdata status %d", status);
        goto exit;
    }

    //TODO: need logic to check for existing config / free that before allocating
    //Need to support parsing of opaque data

    sm_rc_config = (struct qal_st_recognition_config *) calloc(1,
               sizeof(struct qal_st_recognition_config) + rc_config->data_size);
    if (!sm_rc_config) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "Failed to allocate sm_rc_config status %d", status);
        goto exit;
    }

    casa_osal_memcpy(sm_rc_config, sizeof(struct qal_st_recognition_config),
                     rc_config, sizeof(struct qal_st_recognition_config));
    casa_osal_memcpy((uint8_t *)sm_rc_config + rc_config->data_offset, rc_config->data_size,
                 (uint8_t *)rc_config + rc_config->data_offset, rc_config->data_size);

    QAL_DBG(LOG_TAG, "Exit. status - %d", status);
exit:
    return status;
}

int32_t StreamSoundTrigger::getParameters(uint32_t param_id, void **payload)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. get parameter %u, session handle - %pK", param_id,
            session);

    if (activeEngines.size()) {
        SoundTriggerEngine *stEngine = activeEngines[0].second;
        status = stEngine->getParameters(param_id, payload);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to get parameters from sound trigger engine");
        }
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "No primary sound trigger engine present");
    }
    return status;
}

int32_t StreamSoundTrigger::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    struct qal_st_sound_model *sound_model = NULL;
    struct qal_st_recognition_config *rc_config = NULL;

    QAL_DBG(LOG_TAG, "Enter. set parameter %u, session handle - %pK", param_id,
            session);

    mutex.lock();
    // Stream may not know about tags, so use setParameters instead of setConfig
    switch (param_id) {
    case QAL_PARAM_ID_LOAD_SOUND_MODEL:
    {
        sound_model = (struct qal_st_sound_model *)payload;
        status = parse_sound_model(sound_model);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to parse sound model status %d", status);
            goto exit;
        }

        // TODO: Move to SoundTriggerEngineUtil
        status = create_st_engine(activeEngines);
        if (status || !reader_) {
            QAL_ERR(LOG_TAG, "Failed to load sound model status %d", status);
            goto exit;
        }
        break;
    }
    case QAL_PARAM_ID_START_RECOGNITION:
    {
        rc_config = (struct qal_st_recognition_config *)payload;
        status = parse_rc_config(rc_config);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to parse recognition config status %d",
                    status);
            goto exit;
        }

        // register callback function in engine
        registerCallBack(handleDetectionEvent);
        for (int i = 0; i < activeEngines.size(); i++) {
            SoundTriggerEngine *stEngine = activeEngines[i].second;

            // parse recognition config to wakeup config and event config structure
            QAL_VERBOSE(LOG_TAG, "sm_rc_config: %pK", sm_rc_config);
            status = stEngine->update_config(this, sm_rc_config);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Failed to update config status %d", status);
                goto exit;
            }
        }
        break;
    }
    default:
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Unsupported param id %u status %d", param_id, status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Exit. session parameter %u set successful status %d",
            param_id, status);

exit:
    mutex.unlock();
    return status;
}

void StreamSoundTrigger::registerSoundTriggerEngine(uint32_t id,
                                                   SoundTriggerEngine *stEngine)
{
    int index;
    if (!getSoundTriggerEngine(&index, id)) {
        QAL_ERR(LOG_TAG, "Sound Model with id %u already registered", id);
        return;
    }
    activeEngines.push_back(std::make_pair(id, stEngine));
}

void StreamSoundTrigger::deregisterSoundTriggerEngine(uint32_t id)
{
    int index;
    std::vector<std::pair<uint32_t, SoundTriggerEngine *>>::iterator iter;

    if (getSoundTriggerEngine(&index, id)) {
        QAL_ERR(LOG_TAG,  "No Sound trigger engine found for id %u", id);
        return;
    }
    iter += index;
    activeEngines.erase(iter);
}

int32_t StreamSoundTrigger::getSoundTriggerEngine(int *index, uint32_t sm_id)
{
    int32_t status = -1;
    for (int i = 0; i < activeEngines.size(); i++) {
        if (activeEngines[i].first == sm_id) {
            QAL_VERBOSE(LOG_TAG, "Sound trigger engine found for id %u", sm_id);
            *index = i;
            status = 0;
            break;
        }
    }
    return status;
}

// Callback function for detection engine of first stage
int32_t StreamSoundTrigger::handleDetectionEvent(qal_stream_handle_t *stream_handle,
                              uint32_t event_id, uint32_t *event_data, void *cookie)
{
    int32_t status = 0;
    QAL_DBG(LOG_TAG, "Enter. Event detected on GECKO, event id = %u", event_id);

    if (!stream_handle || !event_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "No stream handle or event data provided status %d", status);
        return status;
    }

    StreamSoundTrigger *s;
    s = static_cast<StreamSoundTrigger *>(stream_handle);
    // Parse info from event_data for second stage detection
    status = s->parse_detection_payload(event_id, event_data);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to parse detection payload with ret = %d", status);
        return status;
    }

    // Notify Engine and client
    status = s->setDetected(true);
    QAL_DBG(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t StreamSoundTrigger::setDetected(bool detected)
{
    int32_t status = 0;
    for (int i = 0; i < activeEngines.size(); i++) {
        QAL_VERBOSE(LOG_TAG, "Notify detection event %d to ST engine %d", detected, i);
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        stEngine->setDetected(detected);
    }
    return status;
}

int32_t StreamSoundTrigger::parse_detection_payload(uint32_t event_id, uint32_t *event_data)
{
    int32_t status = 0;
    uint32_t parsedSize = 0;
    uint32_t payloadSize;
    uint32_t eventSize;

    uint8_t *ptr;
    struct event_id_detection_engine_generic_info_t *pGenericInfo;
    struct detection_event_info_header_t *pEventHeader;
    struct confidence_level_info_t *pConfidenceInfo;
    struct keyword_position_info_t *pKeywordPositionInfo;
    struct detection_timestamp_info_t *pDetectionTimeStampInfo;
    struct ftrt_data_info_t *pFtrtInfo;

    if (!event_data) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid event data status %d", status);
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter.");
    // parse event_id_detection_engine_generic_info
    pGenericInfo = (struct event_id_detection_engine_generic_info_t *)event_data;
    payloadSize = sizeof(struct event_id_detection_engine_generic_info_t);
    detectionEventInfo.status = pGenericInfo->status;
    eventSize = pGenericInfo->payload_size;
    ptr = (uint8_t *)event_data + payloadSize;
    QAL_INFO(LOG_TAG, "status = %u, eventSize = %u", detectionEventInfo.status, eventSize);

    // parse variable payload
    while (parsedSize < eventSize) {
        QAL_VERBOSE(LOG_TAG, "parsedSize = %u, eventSize = %u", parsedSize, eventSize);
        pEventHeader = (struct detection_event_info_header_t *)ptr;
        uint32_t keyId = pEventHeader->key_id;
        payloadSize = pEventHeader->payload_size;
        QAL_VERBOSE(LOG_TAG, "key id = %u, payloadSize = %u", keyId, payloadSize);
        ptr += sizeof(struct detection_event_info_header_t);
        parsedSize += sizeof(struct detection_event_info_header_t);

        switch (keyId) {
        case KEY_ID_CONFIDENCE_LEVELS_INFO:
            pConfidenceInfo = (struct confidence_level_info_t *)ptr;
            detectionEventInfo.num_confidence_levels =
                                   pConfidenceInfo->number_of_confidence_values;
            QAL_DBG(LOG_TAG, "num_confidence_levels = %u",
                                      detectionEventInfo.num_confidence_levels);
            for (int i = 0; i < detectionEventInfo.num_confidence_levels; i++) {
                detectionEventInfo.confidence_levels[i] =
                                          pConfidenceInfo->confidence_levels[i];
                QAL_VERBOSE(LOG_TAG, "confidence_levels[%d] = %u", i,
                            detectionEventInfo.confidence_levels[i]);
            }
            break;
        case KEY_ID_KWD_POSITION_INFO:
            pKeywordPositionInfo = (struct keyword_position_info_t *)ptr;
            detectionEventInfo.kw_start_timestamp_lsw =
                                   pKeywordPositionInfo->kw_start_timestamp_lsw;
            detectionEventInfo.kw_start_timestamp_msw =
                                   pKeywordPositionInfo->kw_start_timestamp_msw;
            detectionEventInfo.kw_end_timestamp_lsw =
                                     pKeywordPositionInfo->kw_end_timestamp_lsw;
            detectionEventInfo.kw_end_timestamp_msw =
                                     pKeywordPositionInfo->kw_end_timestamp_msw;
            QAL_DBG(LOG_TAG, "start_lsw = %u, start_msw = %u, end_lsw = %u, end_msw = %u",
                    detectionEventInfo.kw_start_timestamp_lsw,
                    detectionEventInfo.kw_start_timestamp_msw,
                    detectionEventInfo.kw_end_timestamp_lsw,
                    detectionEventInfo.kw_end_timestamp_msw);
            break;
        case KEY_ID_TIMESTAMP_INFO:
            pDetectionTimeStampInfo = (struct detection_timestamp_info_t *)ptr;
            detectionEventInfo.detection_timestamp_lsw =
                               pDetectionTimeStampInfo->detection_timestamp_lsw;
            detectionEventInfo.detection_timestamp_msw =
                               pDetectionTimeStampInfo->detection_timestamp_msw;
            QAL_DBG(LOG_TAG, "timestamp_lsw = %u, timestamp_msw = %u",
                     detectionEventInfo.detection_timestamp_lsw,
                     detectionEventInfo.detection_timestamp_msw);
            break;
        case KEY_ID_FTRT_DATA_INFO:
            pFtrtInfo = (struct ftrt_data_info_t *)ptr;
            detectionEventInfo.ftrt_data_length_in_us =
                                              pFtrtInfo->ftrt_data_length_in_us;
            QAL_DBG(LOG_TAG, "ftrt_data_length_in_us = %u",
                                     detectionEventInfo.ftrt_data_length_in_us);
            break;
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid key id %u status %d", keyId, status);
            goto exit;
        }
        ptr += payloadSize;
        parsedSize += payloadSize;
    }

    QAL_DBG(LOG_TAG, "Exit. Detection payload parsing finished status %d", status);

exit:
    return status;
}

int32_t StreamSoundTrigger::getDetectionEventInfo(struct detection_event_info **info)
{
    int32_t status = 0;
    *info = &detectionEventInfo;
    return status;
}

int32_t StreamSoundTrigger::notifyClient()
{
    int32_t status = 0;
    QAL_INFO(LOG_TAG, "Notify detection event back to client");
    status = generate_callback_event(&recEvent);
    if (!status)
        sm_rc_config->callback(recEvent, sm_rc_config->cookie);
    // release recEvent after callback so that
    // recEvent can be used in next detection
    if (recEvent) {
        if (recEvent->media_config.ch_info) {
            free(recEvent->media_config.ch_info);
            recEvent->media_config.ch_info = NULL;
        }
        if (sound_model_type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
            struct qal_st_phrase_recognition_event *phrase_event =
                (struct qal_st_phrase_recognition_event *)recEvent;
            free(phrase_event);
        } else
            free(recEvent);
        recEvent = NULL;
    }

    return status;
}

int32_t StreamSoundTrigger::setDetectionState(uint32_t state)
{
    switch (state)
    {
        case ENGINE_IDLE:
        case GMM_DETECTED:
        case CNN_DETECTED:
        case VOP_DETECTED:
            detectionState = detectionState | state;
            break;
        default:
            QAL_ERR(LOG_TAG, "Invalid state %x", state);
            return EINVAL;
    }

    QAL_INFO(LOG_TAG, "detectionState = %u, notificationState = %u", detectionState, notificationState);
    if (detectionState == notificationState) {
        QAL_INFO(LOG_TAG, "Notify detection event back to client");
        notifyClient();
    }
    return 0;
}

int32_t StreamSoundTrigger::setVolume(struct qal_volume_data *volume)
{
    int32_t status = 0;
    return status;
}
int32_t StreamSoundTrigger::setMute(bool state)
{
    int32_t status = 0;
    return status;
}

int32_t StreamSoundTrigger::setPause()
{
    int32_t status = 0;
    return status;
}

int32_t StreamSoundTrigger::setResume()
{
    int32_t status = 0;
    return status;
}

int32_t StreamSoundTrigger::create_st_engine(std::vector<std::pair<uint32_t, SoundTriggerEngine *>> &engines)
{
    int32_t status = 0;
    uint32_t engineId;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;
    struct qal_st_sound_model *common_sm = NULL;
    SML_BigSoundModelTypeV3 *big_sm;
    uint8_t *sm_payload = NULL;
    SoundTriggerEngine *stEngine = NULL;

    if (!sm_data) {
        QAL_ERR(LOG_TAG, "Invalid sound model");
        status = EINVAL;
        goto exit;
    }

    // Create Engine GMM
    if (!reader_)
        stEngine = SoundTriggerEngine::create(this, ST_SM_ID_SVA_GMM, &reader_, NULL);
    else
        stEngine = SoundTriggerEngine::create(this, ST_SM_ID_SVA_GMM, &reader_, reader_->ringBuffer_);
    if (!stEngine || !reader_) {
        QAL_ERR(LOG_TAG, "Creation of GMM Engine or reader failed");
        status = ENOENT;
        goto exit;
    }
    engineId = static_cast<uint32_t>(ST_SM_ID_SVA_GMM);
    registerSoundTriggerEngine(engineId, stEngine);
    stEngine->load_sound_model(this, sm_data, stages);

    if (stages == 1)
        goto exit;
    // Create other engines if any
    common_sm = (struct qal_st_sound_model *)sm_data;
    QAL_INFO(LOG_TAG, "Sound model type: %u", common_sm->type);
    if (common_sm->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        sm_payload = (uint8_t *)common_sm + common_sm->data_offset;
        for (int i = 0; i < stages; i++) {
            big_sm = (SML_BigSoundModelTypeV3 *)(sm_payload + sizeof(SML_GlobalHeaderType) +
                sizeof(SML_HeaderTypeV3) + (i * sizeof(SML_BigSoundModelTypeV3)));
            QAL_INFO(LOG_TAG, "type = %u, size = %u", big_sm->type, big_sm->size);
            if (big_sm->type == ST_SM_ID_SVA_GMM)
                continue;

            uint32_t engineId = static_cast<uint32_t>(big_sm->type);
            notificationState = notificationState | engineId;
            stEngine = SoundTriggerEngine::create(this, big_sm->type, &reader_, reader_->ringBuffer_);
            if (!stEngine) {
                QAL_ERR(LOG_TAG, "Failed to create sound trigger engine for sound model type %d", big_sm->type);
                status = ENOENT;
                goto exit;
            }
            registerSoundTriggerEngine(engineId, stEngine);
            stEngine->load_sound_model(this, sm_data, stages);
        }
    } else if (common_sm->type == QAL_SOUND_MODEL_TYPE_GENERIC) {
        //TODO: handle for generic sound model type also
    }

exit:
    return status;
}

int32_t StreamSoundTrigger::generate_callback_event(struct qal_st_recognition_event **event)
{
    struct qal_st_phrase_recognition_event *phrase_event = NULL;
    struct qal_channel_info *ch_info = NULL;
    struct st_param_header *param_hdr = NULL;
    struct st_confidence_levels_info *conf_levels = NULL;
    struct st_keyword_indices_info *kw_indices = NULL;
    struct st_timestamp_info *timestamps = NULL;
    size_t opaque_size = 0;
    uint8_t *opaque_data = NULL;
    QAL_DBG(LOG_TAG, "Enter");

    if (sound_model_type == QAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        opaque_size = (3 * sizeof(struct st_param_header)) +
            sizeof(struct st_timestamp_info) +
            sizeof(struct st_keyword_indices_info) +
            sizeof(struct st_confidence_levels_info);

        phrase_event = (struct qal_st_phrase_recognition_event *)
                       calloc(1, (sizeof(struct qal_st_phrase_recognition_event) + opaque_size));
        if (!phrase_event) {
            QAL_ERR(LOG_TAG, "Failed to alloc memory for recognition event");
            return ENOMEM;
        }

        phrase_event->num_phrases = sm_rc_config->num_phrases;
        memcpy(phrase_event->phrase_extras, sm_rc_config->phrases,
               phrase_event->num_phrases * sizeof(struct qal_st_phrase_recognition_extra));

        *event = &(phrase_event->common);
        (*event)->media_config.ch_info = NULL;
        (*event)->status = 0;
        (*event)->type = sound_model_type;
        (*event)->st_handle = (qal_st_handle_t *)this;
        (*event)->capture_available = true;
        // TODO: generate capture session
        (*event)->capture_session = 0;
        (*event)->capture_delay_ms = 0;
        (*event)->capture_preamble_ms = 0;
        (*event)->trigger_in_data = true;
        (*event)->data_size = opaque_size;
        (*event)->data_offset = sizeof(struct qal_st_phrase_recognition_event);

        ch_info = (struct qal_channel_info *)calloc(1, sizeof(struct qal_channel_info));
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
        opaque_data = (uint8_t *)phrase_event + phrase_event->common.data_offset;

        /* Pack the opaque data confidence levels structure */
        param_hdr = (struct st_param_header *)opaque_data;
        param_hdr->key_id = ST_PARAM_KEY_CONFIDENCE_LEVELS;
        param_hdr->payload_size = sizeof(struct st_confidence_levels_info);
        opaque_data += sizeof(struct st_param_header);
        conf_levels = (struct st_confidence_levels_info *)opaque_data;
        conf_levels->version = 0x1;
        conf_levels->num_sound_models = stages;
        // TODO: update user conf levels
        for (int i = 0; i < conf_levels->num_sound_models; i++) {
            conf_levels->conf_levels[i].sm_id = ST_SM_ID_SVA_GMM;
            conf_levels->conf_levels[i].num_kw_levels = 1;
            conf_levels->conf_levels[i].kw_levels[0].kw_level =
                detectionEventInfo.confidence_levels[i];
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
        timestamps->first_stage_det_event_time =
                ((uint64_t)detectionEventInfo.detection_timestamp_lsw +
                (uint64_t)(detectionEventInfo.detection_timestamp_msw << 32)) * 1000;

    }
    // TODO: handle for generic sound model
    QAL_DBG(LOG_TAG, "Exit");
    return 0;
}

int32_t StreamSoundTrigger::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "sampleRate %u", sampleRate);
    switch(sampleRate) {
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
            QAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t StreamSoundTrigger::isBitWidthSupported(uint32_t bitWidth)
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
            QAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
            break;
    }
    return rc;
}
