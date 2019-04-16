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
#include <unistd.h>

StreamSoundTrigger::StreamSoundTrigger(struct qal_stream_attributes *sattr, struct qal_device *dattr, uint32_t no_of_devices,
                   struct modifier_kv *modifiers, uint32_t no_of_modifiers,std::shared_ptr<ResourceManager> rm)
{
    mutex.lock();
    session = NULL;
    dev = nullptr;
    reader_ = NULL;
    memset(&detectionEventInfo, 0, sizeof(struct detection_event_info));

    QAL_VERBOSE(LOG_TAG, "%s : Start", __func__);
    uNoOfModifiers = no_of_modifiers;
    attr = (struct qal_stream_attributes *) malloc(sizeof(struct qal_stream_attributes));
    if (!attr) {
        QAL_ERR(LOG_TAG, "%s: malloc for stream attributes failed", __func__);
        mutex.unlock();
        throw std::runtime_error("failed to malloc for stream attributes");
    }
    //memset (attr, 0, sizeof(qal_stream_attributes));
    memcpy (attr, sattr, sizeof(qal_stream_attributes));

    //modifiers_ = (struct modifier_kv *)malloc(sizeof(struct modifier_kv));
    //if(!modifiers_)
    //{
      //  ALOGE("%s: Malloc for modifiers failed", __func__);
        //mutex.unlock();
        //throw std::runtime_error("failed to malloc for modifiers");
    //}
    //memset (modifiers_, 0, sizeof(modifier_kv));
    //memcpy (modifiers_, modifiers, sizeof(modifier_kv));

    QAL_VERBOSE(LOG_TAG, "%s: Create new Session", __func__);
    //#ifdef CONFIG_GSL
        session = new SessionGsl(rm);
    QAL_ERR(LOG_TAG, "session %p created", session);
    //#else
      //  session = new SessionAlsapcm();
    //#endif

    if (NULL == session) {
        QAL_ERR(LOG_TAG, "%s: session creation failed", __func__);
        mutex.unlock();
        throw std::runtime_error("failed to create session object");
    }

    QAL_VERBOSE(LOG_TAG, "%s: Create new Devices with no_of_devices - %d", __func__, no_of_devices);
    for (int i = 0; i < no_of_devices; i++) {
        dev = Device::create(&dattr[i] , rm);
        if (dev == nullptr) {
            QAL_ERR(LOG_TAG, "%s: Device creation is failed", __func__);
            mutex.unlock();
            throw std::runtime_error("failed to create device object");
        }
        devices.push_back(dev);
        dev = nullptr;
    }
    mutex.unlock();
    rm->registerStream(this);
    QAL_VERBOSE(LOG_TAG, "%s:end", __func__);
}

int32_t StreamSoundTrigger::open()
{
    int32_t status = 0;
    mutex.lock();
    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p device count - %d", __func__, session, devices.size());
    status = session->open(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: session open failed with status %d", __func__, status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session open successful", __func__);

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->open();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "%s: device open failed with status %d", __func__, status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "%s: device open successful", __func__);

    QAL_VERBOSE(LOG_TAG, "%s:exit StreamSoundTrigger opened", __func__);

exit:
    mutex.unlock();
    QAL_VERBOSE(LOG_TAG, "%s:%d return %d",__func__,__LINE__, status);
    return status;
}

int32_t StreamSoundTrigger::close()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p device count - %d", __func__, session, devices.size());
    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->close();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "%s: device close is failed with status %d",__func__,status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "%s: closed the devices successfully",__func__);

    status = session->close(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: session close failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s:end, closed the session successfully", __func__);

exit:
    mutex.unlock();
    status = rm->deregisterStream(this);
    QAL_ERR(LOG_TAG,"%s:%d status - %d",__func__,__LINE__,status);
    return status;
}


int32_t StreamSoundTrigger::start()
{
    int32_t status = 0;
    mutex.lock();

    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p attr->direction - %d", __func__, session, attr->direction);
    /*status = session->prepare(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Tx session prepare is failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session prepare successful", __func__); */

    for (int i = 0; i < activeEngines.size(); i++)
    {
        uint32_t id = activeEngines[i].first;
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        QAL_VERBOSE(LOG_TAG, "%s: start recognition for sound trigger engine %u", __func__, id);
        status = stEngine->start_recognition(this);
        if (status)
        {
            QAL_ERR(LOG_TAG, "%s: start_recognition failed for sound trigger engine %u with status %d", __func__, id, status);
            goto exit;
        }
    }

    status = session->prepare(this);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Tx session prepare is failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session prepare successful", __func__);

    status = session->start(this);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to start session, status = %d", __func__, status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session start successful", __func__);

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->start();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "%s: Tx device start is failed with status %d", __func__, status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "%s: devices started successfully", __func__);

exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::stop()
{
    int32_t status = 0;

    mutex.lock();

    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p attr->direction - %d", __func__, session, attr->direction);
    QAL_VERBOSE(LOG_TAG, "%s: In QAL_AUDIO_INPUT case, device count - %d", __func__, devices.size());

    for (int32_t i=0; i < devices.size(); i++) {
        status = devices[i]->stop();
        if (0 != status) {
            QAL_ERR(LOG_TAG, "%s: Tx device stop failed with status %d",__func__,status);
            goto exit;
        }
    }
    QAL_VERBOSE(LOG_TAG, "%s: devices stop successful", __func__);

    for (int i = 0; i < activeEngines.size(); i++)
    {
        uint32_t id = activeEngines[i].first;
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        QAL_VERBOSE(LOG_TAG, "%s: stop recognition for sound trigger engine %u", __func__, id);
        status = stEngine->stop_recognition(this);
        if (status)
        {
            QAL_ERR(LOG_TAG, "%s: stop_recognition failed for sound trigger engine %u with status %d", __func__, id, status);
            goto exit;
        }
    }

    status = session->stop(this);
    if (status)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to stop session, status = %d", __func__, status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session stop successful", __func__);
    QAL_VERBOSE(LOG_TAG, "%s: end", __func__);

exit:
    mutex.unlock();
    return status;
}

int32_t StreamSoundTrigger::prepare()
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p", __func__, session);

    mutex.lock();
    status = session->prepare(this);
    if (status)
        QAL_ERR(LOG_TAG, "%s: session prepare failed with status = %d", __func__, status);
    mutex.unlock();
    QAL_VERBOSE(LOG_TAG, "%s: end, status - %d", __func__, status);

    return status;
}

int32_t StreamSoundTrigger::setStreamAttributes(struct qal_stream_attributes *sattr)
{
    int32_t status = 0;

    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p", __func__, session);

    memset(attr, 0, sizeof(qal_stream_attributes));
    mutex.lock();
    memcpy (attr, sattr, sizeof(qal_stream_attributes));
    mutex.unlock();
    status = session->setConfig(this, MODULE, 0);  //gkv or ckv or tkv need to pass
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: session setConfig failed with status %d",__func__,status);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG, "%s: session setConfig successful", __func__);

    QAL_VERBOSE(LOG_TAG, "%s: end", __func__);
exit:
    return status;
}

int32_t StreamSoundTrigger::read(struct qal_buffer* buf)
{
    int32_t status = 0;
    int32_t size;
    QAL_VERBOSE(LOG_TAG, "%s: start, session handle - %p", __func__, session);

    mutex.lock();
    size = reader_->read(buf->buffer, buf->size);
    //status = session->read(this, SHMEM_ENDPOINT, buf, &size);
    mutex.unlock();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: session read is failed with status %d",__func__,status);
        return -status;
    }
    QAL_VERBOSE(LOG_TAG, "%s: end, session read successful size - %d", __func__, size);
    return size;
}

int32_t StreamSoundTrigger::write(struct qal_buffer* buf)
{
    return 0;
}

int32_t StreamSoundTrigger::registerCallBack(qal_stream_callback cb)
{
   // mutex.lock();
    callBack = cb;
    QAL_INFO(LOG_TAG, "callBack = %p", callBack);
   // mutex.unlock();
    return 0;
}

int32_t StreamSoundTrigger::getCallBack(qal_stream_callback *cb)
{
    *cb = callBack;
    QAL_INFO(LOG_TAG, "callBack = %p", (*cb));
    return 0;
}

/* To-Do:
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

    QAL_VERBOSE(LOG_TAG, "%s: start", __func__);

    if (sound_model == NULL){
        status = -EINVAL;
        goto exit;
    }

    if (sound_model->type == QAL_SOUND_MODEL_TYPE_KEYPHRASE){
        phrase_sm = (struct qal_st_phrase_sound_model *)sound_model;
        if ((phrase_sm->common.data_offset < sizeof(struct qal_st_phrase_sound_model)) ||
            (phrase_sm->common.data_size == 0) ||
            (phrase_sm->num_phrases == 0)) {
            QAL_ERR(LOG_TAG, "%s:  Invalid phrase sound model params data size=%d, data offset=%d, "
                    "type=%d phrases=%d", __func__, phrase_sm->common.data_size,
                    phrase_sm->common.data_offset,phrase_sm->num_phrases);
            status = -EINVAL;
            goto exit;
        }
        common_sm = (struct qal_st_sound_model*)&phrase_sm->common;
        recognition_mode = phrase_sm->phrases[0].recognition_mode;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        sm_size = sizeof(*phrase_sm) + common_sm->data_size;
    }
    else if (sound_model->type == QAL_SOUND_MODEL_TYPE_GENERIC){
        if ((sound_model->data_size == 0) ||
            (sound_model->data_offset < sizeof(struct qal_st_sound_model))){
            QAL_ERR(LOG_TAG, "%s:  Invalid generic sound model params data size=%d, data offset=%d", 
                    __func__, sound_model->data_size,sound_model->data_offset);
            status = -EINVAL;
            goto exit;
        }
        recognition_mode = 0x1; //TO-DO: add enum
        common_sm = sound_model;
        sm_payload = (uint8_t*)common_sm + common_sm->data_offset;
        sm_size = sizeof(struct qal_st_sound_model) + common_sm->data_size;
    }
    else{
        QAL_ERR(LOG_TAG, "%s: Unknown sound model type - %d", __func__,sound_model->type);
        status = -EINVAL;
        goto exit;
    }

    sm_data = (uint8_t*) calloc(1, sm_size);
    if (!sm_data){
        status = -ENOMEM;
        goto exit;
    }

    sound_model_type = sound_model->type;

    if (sound_model_type == QAL_SOUND_MODEL_TYPE_KEYPHRASE){
        memcpy(sm_data, (uint8_t*)phrase_sm, sizeof(*phrase_sm));
        memcpy((uint8_t*)sm_data + sizeof(*phrase_sm),
               (uint8_t*)phrase_sm + phrase_sm->common.data_offset,
               phrase_sm->common.data_size);
        QAL_ERR(LOG_TAG, "%s:%d",__func__,__LINE__);
        recognition_mode = phrase_sm->phrases[0].recognition_mode;

        QAL_VERBOSE(LOG_TAG, "%s: phrase recognition mode - %d", __func__, recognition_mode);

    }
    else{
        memcpy(sm_data, (uint8_t*)common_sm, sizeof(*common_sm));
        memcpy((uint8_t*)sm_data + sizeof(*common_sm),
               (uint8_t*)common_sm + common_sm->data_offset, common_sm->data_size);
    }

exit:
    QAL_VERBOSE(LOG_TAG, "%s: end, status - %d", __func__, status);
    return status;
}

int32_t StreamSoundTrigger::generate_recognition_config_payload(unsigned char **out_payload, unsigned int *out_payload_size)
{
    int status = 0;
    unsigned int num_conf_levels = 0;
    unsigned int user_level, user_id;
    unsigned int i = 0, j = 0;
    unsigned char *conf_levels = NULL;
    unsigned char *user_id_tracker;
    struct qal_st_phrase_sound_model *phrase_sm = NULL;

    phrase_sm = (struct qal_st_phrase_sound_model *) sm_data;

    QAL_VERBOSE(LOG_TAG, "%s: start", __func__);

    if (!phrase_sm || !sm_rc_config /*|| !out_payload || !out_payload_size*/){
        QAL_ERR(LOG_TAG, "%s: invalid input", __func__);
        status = -EINVAL;
        goto exit;
    }

    //*out_payload = NULL;
    //*out_payload_size = 0;

    if ((sm_rc_config->num_phrases == 0) ||
        (sm_rc_config->num_phrases > phrase_sm->num_phrases)){
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid phrase data", __func__);
        goto exit;
    }

    for (i = 0; i < sm_rc_config->num_phrases; i++) {
        num_conf_levels++;
        for (j = 0; j < sm_rc_config->phrases[i].num_levels; j++)
            num_conf_levels++;
    }

    conf_levels = (unsigned char*)calloc(1, num_conf_levels);

    user_id_tracker = (unsigned char *) calloc(1, num_conf_levels);
    if (!user_id_tracker) {
        QAL_ERR(LOG_TAG,"%s: failed to allocate user_id_tracker", __func__);
        return -ENOMEM;
    }

    /* for debug */
    for (i = 0; i < sm_rc_config->num_phrases; i++) {
        QAL_VERBOSE(LOG_TAG, "%s: [%d] kw level %d", __func__, i,
        sm_rc_config->phrases[i].confidence_level);
        for (j = 0; j < sm_rc_config->phrases[i].num_levels; j++) {
            QAL_VERBOSE(LOG_TAG, "%s: [%d] user_id %d level %d ", __func__, i,
                        sm_rc_config->phrases[i].levels[j].user_id,
                        sm_rc_config->phrases[i].levels[j].level);
        }
    }

/* Example: Say the recognition structure has 3 keywords with users
        [0] k1 |uid|
                [0] u1 - 1st trainer
                [1] u2 - 4th trainer
                [3] u3 - 3rd trainer
        [1] k2
                [2] u2 - 2nd trainer
                [4] u3 - 5th trainer
        [2] k3
                [5] u4 - 6th trainer

      Output confidence level array will be
      [k1, k2, k3, u1k1, u2k1, u2k2, u3k1, u3k2, u4k3]
*/
    for (i = 0; i < sm_rc_config->num_phrases; i++) {
        conf_levels[i] = sm_rc_config->phrases[i].confidence_level;
        for (j = 0; j < sm_rc_config->phrases[i].num_levels; j++) {
            user_level = sm_rc_config->phrases[i].levels[j].level;
            user_id = sm_rc_config->phrases[i].levels[j].user_id;
            if ((user_id < sm_rc_config->num_phrases) ||
                (user_id >= num_conf_levels)) {
                QAL_ERR(LOG_TAG, "%s: ERROR. Invalid params user id %d>%d",
                        __func__, user_id);
                status = -EINVAL;
                goto exit;
            }
            else {
                if (user_id_tracker[user_id] == 1) {
                    QAL_ERR(LOG_TAG, "%s: ERROR. Duplicate user id %d",
                            __func__, user_id);
                    status = -EINVAL;
                    goto exit;
                }
                conf_levels[user_id] = (user_level < 100) ? user_level : 100;
                user_id_tracker[user_id] = 1;
                QAL_VERBOSE(LOG_TAG, "%s: user_conf_levels[%d] = %d", __func__,
                            user_id, conf_levels[user_id]);
            }
        }
    }

exit:
    QAL_VERBOSE(LOG_TAG, "%s: end, status - %d", __func__, status);
    return status;
}

//TODO:	- look into how cookies are used here
int32_t StreamSoundTrigger::parse_rc_config(struct qal_st_recognition_config *rc_config){
    int32_t status = 0;

    unsigned char **out_payload;
    unsigned int *out_payload_size;

    QAL_VERBOSE(LOG_TAG, "%s: start", __func__);

    if (sm_data == NULL){
        status = -EINVAL;
        goto exit;
    }

    //TODO: need logic to check for existing config / free that before allocating
    //Need to support parsing of opaque data

    sm_rc_config = (struct qal_st_recognition_config *) calloc(1, sizeof(struct qal_st_recognition_config) + rc_config->data_size);

    memcpy(sm_rc_config, rc_config, sizeof(struct qal_st_recognition_config));
    memcpy(sm_rc_config + rc_config->data_offset, rc_config + rc_config->data_offset,
           rc_config->data_size);

    status = generate_recognition_config_payload(out_payload,out_payload_size);

exit:
    QAL_VERBOSE(LOG_TAG, "%s: end, status - %d", __func__, status);
    return status;
}

int32_t StreamSoundTrigger::setParameters(uint32_t param_id, void *payload)
{
    int32_t status = 0;
    struct qal_st_sound_model *sound_model = NULL;
    struct qal_st_recognition_config *rc_config = NULL;

    QAL_INFO(LOG_TAG, "%s: start, set parameter %u, session handle - %p", __func__, param_id, session);

    mutex.lock();
    // Stream may not know about tags, so use setParameters instead of setConfig
    switch (param_id) {
        case QAL_PARAM_ID_LOAD_SOUND_MODEL:
        {
            sound_model = (struct qal_st_sound_model *)payload;
            status = parse_sound_model(sound_model);
            if (status)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to parse sound model", __func__);
                goto exit;
            }
            // TODO: parse big sound model to check model numbers
            // as well as population of model ids and host ids
            uint32_t id = 0;
            uint32_t stageId = 0;
            SoundTriggerEngine *stEngine;
            if (!reader_)
                stEngine = SoundTriggerEngine::create(this, id, stageId, &reader_, NULL);
            else
                stEngine = SoundTriggerEngine::create(this, id, stageId, &reader_, reader_->ringBuffer_);
            registerSoundTriggerEngine(id, stEngine);
            // parse sound model to structure which can be set to GSL
            status = stEngine->load_sound_model(this, sm_data);
            if (status)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to load sound model", __func__);
                goto exit;
            }
            break;
        }
        case QAL_PARAM_ID_START_RECOGNITION:
        {
            rc_config = (struct qal_st_recognition_config *)payload;
            status = parse_rc_config(rc_config);
            if (status)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to parse recognition config", __func__);
                goto exit;
            }

            // register callback function in engine
            registerCallBack(handleDetectionEvent);
            for (int i = 0; i < activeEngines.size(); i++)
            {
                SoundTriggerEngine *stEngine = activeEngines[i].second;

                // parse recognition config to wakeup config and event config structure
                QAL_ERR(LOG_TAG, "sm_rc_config: %p", sm_rc_config);
                status = stEngine->update_config(this, sm_rc_config);
                if (status)
                {
                    QAL_ERR(LOG_TAG, "%s: Failed to update config", __func__);
                    goto exit;
                }
            }
            break;
        }
        default:
            QAL_ERR(LOG_TAG, "%s: Unsupported param id %u", __func__, param_id);
            status = -EINVAL;
            break;
    }
    mutex.unlock();

    QAL_VERBOSE(LOG_TAG, "%s: end, session parameter %u set successful", __func__, param_id);
    return status;

exit:
    QAL_ERR(LOG_TAG, "%s: Failed to setParameters for %u, ret = %d", __func__, param_id, status);
    return status;
}

void StreamSoundTrigger::registerSoundTriggerEngine(uint32_t id, SoundTriggerEngine *stEngine)
{
    int index;
    if (!getSoundTriggerEngine(&index, id))
    {
        QAL_ERR(LOG_TAG, "%s: Sound Model with id %u already registered", __func__, id);
        return;
    }
    activeEngines.push_back(std::make_pair(id, stEngine));
}

void StreamSoundTrigger::deregisterSoundTriggerEngine(uint32_t id)
{
    int index;
    std::vector<std::pair<uint32_t, SoundTriggerEngine *>>::iterator iter;

    if (getSoundTriggerEngine(&index, id))
    {
        QAL_ERR(LOG_TAG, "%s: No Sound trigger engine found for id %u", __func__, id);
        return;
    }
    iter += index;
    activeEngines.erase(iter);
}

int32_t StreamSoundTrigger::getSoundTriggerEngine(int *index, uint32_t sm_id)
{
    int32_t status = -1;
    for (int i = 0; i < activeEngines.size(); i++)
    {
        if (activeEngines[i].first == sm_id)
        {
            QAL_VERBOSE(LOG_TAG, "%s: Sound trigger engine found for id %u", __func__, sm_id);
            *index = i;
            status = 0;
            break;
        }
    }
    return status;
}

// Callback function for detection engine of first stage
int32_t StreamSoundTrigger::handleDetectionEvent(qal_stream_handle_t *stream_handle, uint32_t event_id,
                                                 uint32_t *event_data, void *cookie)
{
    int32_t status = 0;
    QAL_INFO(LOG_TAG, "Event detected on GECKO, event id = %u", event_id);

    if (!stream_handle || !event_data)
    {
        QAL_ERR(LOG_TAG, "No stream handle or event data provided");
        status = -EINVAL;
        return status;
    }

    StreamSoundTrigger *s;
    s = static_cast<StreamSoundTrigger *>(stream_handle);
    // Parse info from event_data for second stage detection
    status = s->parse_detection_payload(event_id, event_data);
    if (status)
    {
        QAL_ERR(LOG_TAG, "Failed to parse detection payload with ret = %d", status);
        return status;
    }

    // Notify Engine and client
    status = s->setDetected(true);
    return status;
}

int32_t StreamSoundTrigger::setDetected(bool detected)
{
    int32_t status = 0;
    for (int i = 0; i < activeEngines.size(); i++)
    {
        QAL_INFO(LOG_TAG, "Notify detection event %d to ST engine %d", detected, i);
        SoundTriggerEngine *stEngine = activeEngines[i].second;
        /* Each engine has a bool member indicating
           if event detected in first stage
           TODO: implement setDetected in second
           stage engine so that it can fetch data
           and start keyword detection*/
        stEngine->setDetected(detected);
    }
    QAL_INFO(LOG_TAG, "Notify detection event back to client");
    // notify Client also, for now just set event null
    sm_rc_config->callback(NULL, NULL);
    return status;
}

int32_t StreamSoundTrigger::parse_detection_payload(uint32_t event_id, uint32_t *event_data)
{
    QAL_INFO(LOG_TAG, "Enter");
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

    if (!event_data)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid event data");
        goto exit;
    }

    // parse event_id_detection_engine_generic_info
    pGenericInfo = (struct event_id_detection_engine_generic_info_t *)event_data;
    payloadSize = sizeof(struct event_id_detection_engine_generic_info_t);
    detectionEventInfo.status = pGenericInfo->status;
    eventSize = pGenericInfo->payload_size;
    ptr = (uint8_t *)event_data + payloadSize;
    QAL_INFO(LOG_TAG, "status = %u, eventSize = %u", detectionEventInfo.status, eventSize);

    // parse variable payload
    while (parsedSize < eventSize)
    {
        QAL_INFO(LOG_TAG, "parsedSize = %u, eventSize = %u", parsedSize, eventSize);
        pEventHeader = (struct detection_event_info_header_t *)ptr;
        uint32_t keyId = pEventHeader->Key_id;
        payloadSize = pEventHeader->payload_size;
        QAL_INFO(LOG_TAG, "key id = %u, payloadSize = %u", keyId, payloadSize);
        ptr += sizeof(struct detection_event_info_header_t);
        parsedSize += sizeof(struct detection_event_info_header_t);

        switch (keyId)
        {
            case KEY_ID_CONFIDENCE_LEVELS_INFO:
                pConfidenceInfo = (struct confidence_level_info_t *)ptr;
                detectionEventInfo.num_confidence_levels = pConfidenceInfo->num_confidence_levels;
                QAL_INFO(LOG_TAG, "num_confidence_levels = %u", detectionEventInfo.num_confidence_levels);
                for (int i = 0; i < detectionEventInfo.num_confidence_levels; i++)
                {
                    detectionEventInfo.confidence_levels[i] = pConfidenceInfo->confidence_levels[i];
                    QAL_INFO(LOG_TAG, "confidence_levels[%d] = %u", i, detectionEventInfo.confidence_levels[i]);
                }
                break;
            case KEY_ID_KWD_POSITION_INFO:
                pKeywordPositionInfo = (struct keyword_position_info_t *)ptr;
                detectionEventInfo.kw_start_timestamp_lsw = pKeywordPositionInfo->kw_start_timestamp_lsw;
                detectionEventInfo.kw_start_timestamp_msw = pKeywordPositionInfo->kw_start_timestamp_msw;
                detectionEventInfo.kw_end_timestamp_lsw = pKeywordPositionInfo->kw_end_timestamp_lsw;
                detectionEventInfo.kw_end_timestamp_msw = pKeywordPositionInfo->kw_end_timestamp_msw;
                QAL_INFO(LOG_TAG, "start_lsw = %u, start_msw = %u, end_lsw = %u, end_msw = %u",
                         detectionEventInfo.kw_start_timestamp_lsw, detectionEventInfo.kw_start_timestamp_msw,
                         detectionEventInfo.kw_end_timestamp_lsw, detectionEventInfo.kw_end_timestamp_msw);
                break;
            case KEY_ID_TIMESTAMP_INFO:
                pDetectionTimeStampInfo = (struct detection_timestamp_info_t *)ptr;
                detectionEventInfo.detection_timestamp_lsw = pDetectionTimeStampInfo->detection_timestamp_lsw;
                detectionEventInfo.detection_timestamp_msw = pDetectionTimeStampInfo->detection_timestamp_msw;
                QAL_INFO(LOG_TAG, "timestamp_lsw = %u, timestamp_msw = %u",
                         detectionEventInfo.detection_timestamp_lsw, detectionEventInfo.detection_timestamp_msw);
                break;
            case KEY_ID_FTRT_DATA_INFO:
                pFtrtInfo = (struct ftrt_data_info_t *)ptr;
                detectionEventInfo.ftrt_data_length_in_us = pFtrtInfo->ftrt_data_length_in_us;
                QAL_INFO(LOG_TAG, "ftrt_data_length_in_us = %u", detectionEventInfo.ftrt_data_length_in_us);
                break;
            default:
                QAL_ERR(LOG_TAG, "Invalid key id %u", keyId);
                status = -EINVAL;
                goto exit;
                break;
        }
        ptr += payloadSize;
        parsedSize += payloadSize;
    }

    QAL_ERR(LOG_TAG, "Detection payload parsing finished");
    return status;

exit:
    QAL_ERR(LOG_TAG, "Failed to parse detection payload, status = %d", status);
    return status;
}

int32_t StreamSoundTrigger::getDetectionEventInfo(struct detection_event_info **info)
{
    int32_t status = 0;
    *info = &detectionEventInfo;
    return status;
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
