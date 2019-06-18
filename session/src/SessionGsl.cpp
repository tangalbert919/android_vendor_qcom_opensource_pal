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

#define LOG_TAG "SessionGsl"

#include "SessionGsl.h"
#include "Stream.h"
#include "QalDefs.h"
#include "Device.h"
#include<algorithm>
#include<vector>
#include<fstream>
#include "PayloadBuilder.h"
#include "ResourceManager.h"

#define GSL_LIB  "libgsl.so"
#define PLAYBACK 0x1
#define RECORD 0x2
#define BUFFER_EOS 1



typedef int32_t (*gsl_init_t)(struct gsl_init_data *);
typedef int32_t (*gsl_open_t)(const struct gsl_key_vector *,const struct gsl_key_vector *, gsl_handle_t *);
typedef int32_t (*gsl_close_t)(gsl_handle_t);
typedef int32_t (*gsl_set_cal_t)(gsl_handle_t , const struct gsl_key_vector *);
typedef int32_t (*gsl_set_config_t)(gsl_handle_t, uint32_t, const struct gsl_key_vector *);
typedef int32_t (*gsl_set_custom_config_t)(gsl_handle_t, const uint8_t *, const size_t);
typedef int32_t (*gsl_get_custom_config_t)(gsl_handle_t, uint8_t *, size_t *);
typedef int32_t (*gsl_ioctl_t)(gsl_handle_t, enum gsl_cmd_id, void *, size_t);
typedef int32_t (*gsl_read_t)(gsl_handle_t, uint32_t, struct gsl_buff *, uint32_t *);
typedef int32_t (*gsl_write_t)(gsl_handle_t, uint32_t, struct gsl_buff *, uint32_t *);
typedef int32_t (*gsl_get_tagged_module_info_t)(const struct gsl_key_vector *, uint32_t, struct gsl_module_id_info **, size_t *);
typedef int32_t (*gsl_register_event_cb_t)(gsl_handle_t, gsl_cb_func_ptr, void *);
typedef void (*gsl_deinit_t)(void);

gsl_ioctl_t gslIoctl;
gsl_get_custom_config_t gslGetCustomConfig;
gsl_set_cal_t gslSetCal;
gsl_set_config_t gslSetConfig;
gsl_set_custom_config_t gslSetCustomConfig;
gsl_get_tagged_module_info_t gslGetTaggedModuleInfo;
gsl_register_event_cb_t gslRegisterEventCallBack;
gsl_read_t gslRead;
gsl_open_t gslOpen;
gsl_close_t gslClose;
gsl_write_t gslWrite;
gsl_init_t gslInit;
gsl_deinit_t gslDeinit;

int SessionGsl::seek = 0;
void *SessionGsl::gslLibHandle = NULL;

SessionGsl::SessionGsl() {

}

SessionGsl::SessionGsl(std::shared_ptr<ResourceManager> Rm) {
    rm = Rm;
}

SessionGsl::~SessionGsl() {

}

int SessionGsl::init(std::string acdbFile) {
    int ret = 0;
    struct gsl_acdb_data_files acdb_files;
    gsl_init_data init_data;
    acdb_files.num_files = 1;
    strlcpy(acdb_files.acdbFiles[0].fileName, acdbFile.c_str(),
              acdbFile.size()+1);
    acdb_files.acdbFiles[0].fileNameLen = acdbFile.size();
    init_data.acdb_files = &acdb_files;
    init_data.acdb_delta_file = NULL;
    init_data.acdb_addr = 0x0;
    init_data.max_num_ready_checks = 1;
    init_data.ready_check_interval_ms = 100;
    if(gslLibHandle == NULL) {
        gslLibHandle = dlopen(GSL_LIB, RTLD_NOW);
        if (NULL == gslLibHandle) {
            const char *err_str = dlerror();
            QAL_ERR(LOG_TAG,"%s:: DLOPEN failed for %s, %s", __func__,
                  GSL_LIB, err_str?err_str:"unknown");
            return -EINVAL;
        }

    }

    /*loading the gsl function symbols*/
    gslInit = (gsl_init_t)dlsym(gslLibHandle, "gsl_init");
    if (gslInit == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_init", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslSetCal = (gsl_set_cal_t)dlsym(gslLibHandle, "gsl_set_cal");
    if (gslSetCal == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_cal", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslSetConfig = (gsl_set_config_t)dlsym(gslLibHandle, "gsl_set_config");
    if (gslSetConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_config", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslSetCustomConfig = (gsl_set_custom_config_t)dlsym(gslLibHandle, "gsl_set_custom_config");
    if (gslSetCustomConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_custom_config", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslGetCustomConfig = (gsl_get_custom_config_t)dlsym(gslLibHandle, "gsl_get_custom_config");
    if (gslGetCustomConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_get_custom_config", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslIoctl = (gsl_ioctl_t)dlsym(gslLibHandle, "gsl_ioctl");
    if (gslIoctl == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_ioctl", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslGetTaggedModuleInfo = (gsl_get_tagged_module_info_t)dlsym(gslLibHandle, "gsl_get_tagged_module_info");
    if (gslGetTaggedModuleInfo == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_get_tagged_module_info", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslRegisterEventCallBack = (gsl_register_event_cb_t)dlsym(gslLibHandle, "gsl_register_event_cb");
    if (gslRegisterEventCallBack == NULL) {
        QAL_ERR(LOG_TAG, "%s: dlsym error %s for gsl_register_event_cb", __func__, dlerror());
        return -EINVAL;
    }
    gslRead = (gsl_read_t)dlsym(gslLibHandle, "gsl_read");
    if (gslRead == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_read", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslOpen = (gsl_open_t)dlsym(gslLibHandle, "gsl_open");
    if (gslOpen == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_open", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslClose = (gsl_close_t)dlsym(gslLibHandle, "gsl_close");
    if (gslClose == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_close", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslWrite = (gsl_write_t)dlsym(gslLibHandle, "gsl_write");
    if (gslWrite == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_write", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    gslDeinit = (gsl_deinit_t)dlsym(gslLibHandle, "gsl_deinit");
    if (gslDeinit == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_deinit", __func__, dlerror());
        ret = -EINVAL;
        goto error;
    }
    ret = PayloadBuilder::init();
    if (ret) {
        QAL_ERR(LOG_TAG,"%s: payload builder init failed with err = %d", __func__, ret);
        goto error;
    }
    ret = gslInit(&init_data);
    if (ret) {
        QAL_ERR(LOG_TAG,"%s: gsl init failed with err = %d", __func__, ret);
        goto error;
    }
    QAL_ERR(LOG_TAG,"%s: gsl init success with acdb file %s", __func__, acdbFile.c_str());
//#endif
    goto exit;
error:
    dlclose(gslLibHandle);
    gslLibHandle = NULL;
exit:
    return ret;
}

void SessionGsl::deinit() {
    if(NULL != gslLibHandle){
        gslDeinit();
        dlclose(gslLibHandle);
    }
}

int populateGkv(Stream *s, struct gsl_key_vector *gkv) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);
    int32_t dev_id = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::vector <std::pair<int,int>> keyVector;
    struct qal_stream_attributes *sattr = NULL;
    sattr = (struct qal_stream_attributes *)malloc(sizeof(struct qal_stream_attributes)); 
    memset (sattr, 0, sizeof(struct qal_stream_attributes)); 
    status = s->getStreamAttributes(sattr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }
    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }
    switch (sattr->type) {
    case QAL_STREAM_LOW_LATENCY:
        if (sattr->direction == QAL_AUDIO_OUTPUT) {
            QAL_VERBOSE(LOG_TAG,"%s: Stream \n", __func__);
            keyVector.push_back(std::make_pair(STREAM_TYPE,PCM_LL_PLAYBACK));
        }
        else if (sattr->direction == QAL_AUDIO_INPUT) {
            keyVector.push_back(std::make_pair(STREAM_TYPE,PCM_RECORD));
        }
        else if (sattr->direction == (QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT)) {
            keyVector.push_back(std::make_pair(STREAM_TYPE,PCM_LOOPBACK));
        }
        else {
            QAL_ERR(LOG_TAG,"%s: Invalid direction \n", __func__);
            status = -EINVAL;
            return status;
        }
        break;
    case QAL_STREAM_DEEP_BUFFER:
        break;
    case QAL_STREAM_GENERIC:
        break;
    case QAL_STREAM_COMPRESSED:
        break;
    case QAL_STREAM_VOIP_TX:
        keyVector.push_back(std::make_pair(STREAM_TYPE,VOIP_TX_RECORD));
        break;
    case QAL_STREAM_VOIP_RX:
        keyVector.push_back(std::make_pair(STREAM_TYPE,VOIP_RX_PLAYBACK));
        break;
    case QAL_STREAM_VOICE_UI:
        QAL_ERR(LOG_TAG, "%s:%d voice ui",__func__,__LINE__);
        keyVector.push_back(std::make_pair(STREAM_TYPE,VOICE_UI));
        break;
    default:
        QAL_ERR(LOG_TAG,"unsupported stream type %s", sattr->type);
        status = -EINVAL;
        return status;
    }
    
    //TODO: add stream PP and device PP population
    for (int32_t i=0; i<(associatedDevices.size()); i++) {
        dev_id = associatedDevices[i]->getDeviceId();
        switch(dev_id) {
        case QAL_DEVICE_OUT_SPEAKER :
            keyVector.push_back(std::make_pair(DEVICERX,SPEAKER));
            break;
        case QAL_DEVICE_IN_SPEAKER_MIC:
            keyVector.push_back(std::make_pair(DEVICETX,HANDSETMIC));
            break;
        case QAL_DEVICE_IN_HANDSET_MIC:
           keyVector.push_back(std::make_pair(DEVICETX,HANDSETMIC));
           break;
        case QAL_DEVICE_IN_TRI_MIC:
           keyVector.push_back(std::make_pair(DEVICETX,HANDSETMIC));
           break;
        default:
            QAL_ERR(LOG_TAG,"%s: Invalid device id %d\n", __func__,dev_id);
            status = -EINVAL;
            return status;
        }
    }
    gkv->num_kvps = keyVector.size();
    gkv->kvp = new struct gsl_key_value_pair[keyVector.size()];
    for(int32_t i=0; i < (keyVector.size()); i++) {
        gkv->kvp[i].key = keyVector[i].first;
        gkv->kvp[i].value = keyVector[i].second;
    }
    QAL_VERBOSE(LOG_TAG,"%s: gkv size %d", __func__,(gkv->num_kvps));
    for(int32_t i=0; i < (keyVector.size()); i++) {
       QAL_VERBOSE(LOG_TAG,"%s: gkv key %x value %x", __func__,(gkv->kvp[i].key),(gkv->kvp[i].value));
    }
    return status;
}

int populateCkv(Stream *s, struct gsl_key_vector *ckv, int tag, void* graphHandle, SessionGsl *sg) {
    int status = 0;
    float voldB = 0;
    QAL_ERR(LOG_TAG,"%s: enter \n", __func__);
    std::vector <std::pair<int,int>> keyVector;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    PayloadBuilder *builder = new PayloadBuilder();
    struct qal_volume_data *voldata = NULL;
    voldata = (struct qal_volume_data *)malloc(sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (0xFFFF)));
    memset (voldata, 0, sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (0xFFFF)));

    status = s->getVolumeData(voldata);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getVolumeData Failed \n", __func__);
        return status;
    }
    QAL_ERR(LOG_TAG,"%s: volume sent:%f \n",__func__, (voldata->volume_pair[0].vol));
    voldB = (voldata->volume_pair[0].vol);
    switch (tag) {
    case TAG_STREAM_VOLUME:
       QAL_ERR(LOG_TAG,"%s: voldb:%f \n",__func__, (voldB));
       if (0 <= voldB < 0.1) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_15));
       }
       else if (0.1 <= voldB < 0.2) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_13));
       }
       else if (0.2 <= voldB < 0.3) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_11));
       }
       else if (0.3 <= voldB < 0.4) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_9));
       }
       else if (0.4 <= voldB < 0.5) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_7));
       }
       else if (0.5 <= voldB < 0.6) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_6));
       }
       else if (0.6 <= voldB < 0.7) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_4));
       }
       else if (0.7 <= voldB < 0.8) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_3));
       }
       else if (0.8 <= voldB < 0.9) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_2));
       }
       else if (0.9 <= voldB < 1) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_1));
       }
       else if (voldB >= 1) {
          keyVector.push_back(std::make_pair(VOLUME,LEVEL_0));
          QAL_ERR(LOG_TAG,"%s:max %d \n",__func__, (voldata->no_of_volpair));
       }
       status = gslGetTaggedModuleInfo(sg->gkv, tag,
                                         &moduleInfo, &moduleInfoSize);
       if (status != 0) {
           QAL_ERR(LOG_TAG,"%s: Failed to get tag info %x module size\n", __func__, tag);
           goto exit;
       }

       builder->payloadVolume(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, voldata, tag);
       QAL_ERR(LOG_TAG,"%s:%x - payload and %d size\n",__func__, payload , payloadSize);

       status = gslSetCustomConfig(graphHandle, payload, payloadSize);
       if (0 != status) {
           QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
       }

       if (moduleInfo)
           free(moduleInfo);

       if (payload)
           free(payload);
       delete builder;
       break;
    default:
    keyVector.push_back(std::make_pair(VOLUME,LEVEL_0)); /*TODO Decide what to send as ckv in graph open*/
    QAL_ERR(LOG_TAG,"%s: Entered default\n", __func__);
    break;
    }
    ckv->num_kvps = keyVector.size();
    ckv->kvp = new struct gsl_key_value_pair[keyVector.size()];
    for (int i = 0; i < keyVector.size(); i++) {
        ckv->kvp[i].key = keyVector[i].first;
        ckv->kvp[i].value = keyVector[i].second;
        QAL_ERR(LOG_TAG,"%s: ckv key %x value %x", __func__,(ckv->kvp[i].key),(ckv->kvp[i].value));
    }
    QAL_ERR(LOG_TAG,"%s: exit status- %d", __func__, status);
exit:
    return status;
}

int populateTkv(Stream *s, struct gsl_key_vector *tkv, int tag, uint32_t* gsltag) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);
    std::vector <std::pair<int,int>> keyVector;
    switch (tag){
    case MUTE_TAG:
       keyVector.push_back(std::make_pair(MUTE,ON));
       *gsltag = TAG_MUTE;
       break;
    case UNMUTE_TAG:
       keyVector.push_back(std::make_pair(MUTE,OFF));
       *gsltag = TAG_MUTE;
       break;
    case PAUSE_TAG:
       keyVector.push_back(std::make_pair(PAUSE,ON));
       *gsltag = TAG_PAUSE;
       break;
    case RESUME_TAG:
       keyVector.push_back(std::make_pair(PAUSE,OFF));
       *gsltag = TAG_PAUSE;
       break;
    case MFC_SR_8K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_8K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_16K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_16K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_32K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_32K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_44K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_44K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_48K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_48K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_96K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_96K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_192K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_192K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    case MFC_SR_384K:
       keyVector.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_384K));
       *gsltag = TAG_STREAM_MFC_SR;
       break;
    default:
       QAL_ERR(LOG_TAG,"%s: Tag not supported \n", __func__);
       break;
    }
    tkv->num_kvps = keyVector.size();
    tkv->kvp = new struct gsl_key_value_pair[keyVector.size()];
    for (int i = 0; i < keyVector.size(); i++) {
        tkv->kvp[i].key = keyVector[i].first;
        tkv->kvp[i].value = keyVector[i].second;
    }
    QAL_ERR(LOG_TAG,"%s: tkv size %d", __func__,(tkv->num_kvps));
    for(int32_t i=0; i < (keyVector.size()); i++) {
       QAL_ERR(LOG_TAG,"%s: tkv key %x value %x", __func__,(tkv->kvp[i].key),(tkv->kvp[i].value));
    }  
    QAL_ERR(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

void printCustomConfig(const uint8_t* payload, size_t size) {
    size_t loop;
    uint32_t *temp = (uint32_t *)payload;
    QAL_ERR(LOG_TAG,"%s: enter size - %d", __func__, size);
    for (loop = 0; loop < size;) {
        QAL_ERR(LOG_TAG,"%0x %0x %0x %0x\n",temp[0+loop],temp[1+loop],temp[2+loop],temp[3+loop]);
        loop = loop + 4;
    }
    QAL_ERR(LOG_TAG,"%s: exit", __func__);
}

int populateSVASoundModel(SessionGsl *s, int tagId, void* graphHandle, struct qal_st_sound_model *pSoundModel)
{
    int32_t status = 0;
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
              __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVASoundModel(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, pSoundModel);
    QAL_VERBOSE(LOG_TAG,"%s:%d %p - payload and %d size", payload , payloadSize);

    status = gslSetCustomConfig(graphHandle, payload, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);

    QAL_VERBOSE(LOG_TAG, "%s: exit status- %d", __func__, status);
    return status;
}

int populateSVAWakeUpConfig(SessionGsl *s, int tagId, void* graphHandle, struct detection_engine_config_voice_wakeup *pWakeUpConfig)
{
    int32_t status = 0;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
              __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }

    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVAWakeUpConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, pWakeUpConfig);
    status = gslSetCustomConfig(graphHandle, payload, payloadSize);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    QAL_VERBOSE(LOG_TAG, "%s: exit status- %d", __func__, status);
    return status;
}

int populateSVAWakeUpBufferConfig(SessionGsl *s, int tagId, void* graphHandle, struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufConfig)
{
    int32_t status = 0;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    QAL_VERBOSE(LOG_TAG, "%s:%d", __func__, __LINE__);
    status = gslGetTaggedModuleInfo(s->gkv, tagId,
        &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
            __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }
    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVAWakeUpBufferConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, pWakeUpBufConfig);
    QAL_VERBOSE(LOG_TAG, "%s:%d History Buffer: %d", __func__, __LINE__, pWakeUpBufConfig->hist_buffer_duration_in_ms);
    QAL_VERBOSE(LOG_TAG, "%s:%d Pre-roll: %d", __func__, __LINE__, pWakeUpBufConfig->pre_roll_duration_in_ms);

    status = gslSetCustomConfig(graphHandle, payload, payloadSize);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    QAL_VERBOSE(LOG_TAG, "%s:%d", __func__, __LINE__);
    QAL_VERBOSE(LOG_TAG, "%s: exit status- %d", __func__, status);
    return status;
}

int populateSVAStreamSetupDuration(SessionGsl *s, int tagId, void* graphHandle, struct audio_dam_downstream_setup_duration *pSetupDuration)
{
    int32_t status = 0;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
              __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }

    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVAStreamSetupDuration(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, pSetupDuration);
    status = gslSetCustomConfig(graphHandle, payload, payloadSize);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);

    QAL_VERBOSE(LOG_TAG, "exit status- %d", status);
    return status;
}

int populateSVAEventConfig(SessionGsl *s, int tagId, void* graphHandle, struct detection_engine_generic_event_cfg *pEventConfig)
{
    int32_t status = 0;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
              __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }

    // Register custom config
    struct gsl_cmd_register_custom_event reg_ev_payload;
    reg_ev_payload.event_id = EVENT_ID_DETECTION_ENGINE_GENERIC_INFO;
    reg_ev_payload.module_instance_id = moduleInfo->module_entry[0].module_iid;
    reg_ev_payload.event_config_payload_size = 0;
    reg_ev_payload.is_register = 1;
    status = gslIoctl(graphHandle, GSL_CMD_REGISTER_CUSTOM_EVENT, &reg_ev_payload, sizeof(reg_ev_payload));
    if (status)
        QAL_ERR(LOG_TAG, "Failed to register custom event with status = %d\n", status);

    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVAEventConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, pEventConfig);
    status = gslSetCustomConfig(graphHandle, payload, payloadSize);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }
    QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);

    QAL_VERBOSE(LOG_TAG, "%s: exit status- %d", __func__, status);
    return status;
}

int populateSVAEngineReset(SessionGsl *s, int tagId, void* graphHandle, Stream *st)
{
    int32_t status = 0;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "%s: Failed to get tag info %x module size\n structure values %x :%x and %x:%x",
              __func__, tagId, s->gkv->kvp[0].key, s->gkv->kvp[0].value, s->gkv->kvp[1].key, s->gkv->kvp[1].value);
        return status;
    }

    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadSVAEngineReset(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid);
    status = gslSetCustomConfig(graphHandle, payload, payloadSize);

    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: Set custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);

    QAL_VERBOSE(LOG_TAG, "%s: exit status- %d", __func__, status);
    return status;
}

int SessionGsl::open(Stream *s) {
    int status = 0;
    struct qal_stream_attributes sAttr;
    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    QAL_VERBOSE(LOG_TAG,"%s: enter direction:%d ", __func__, sAttr.direction);
    gkv = new gsl_key_vector;
    status = populateGkv(s, gkv);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to populate gkv", __func__);
        goto exit;
    }

    ckv = new gsl_key_vector;
    status = populateCkv(s, ckv, 0, 0, 0);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to populate ckv", __func__);
        goto exit;
    }

    status = gslOpen(gkv, ckv, &graphHandle);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to open the graph with status %d", __func__,status);
        goto exit;
    }
    //status = gsl_set_cal_(graphHandle, ckv);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to set the calibration data\n", __func__);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: graph_set_cal success. handle:%p", __func__, graphHandle);
    checkAndConfigConcurrency(s);
    setPayloadConfig(s);

    QAL_ERR(LOG_TAG,"%s: end status:%d", __func__, status);
exit:
     return status;
}

int SessionGsl::setPayloadConfig(Stream *s) {
    std::vector <int> streamTag;
    std::vector <int> streamPpTag;
    std::vector <int> mixerTag;
    std::vector <int> devicePpTag;
    std::vector <int> deviceTag;
    struct gsl_module_id_info* moduleInfo;
    size_t moduleInfoSize;
    struct sessionToPayloadParam* sessionData;
    struct sessionToPayloadParam* deviceData;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    sessionData = (struct sessionToPayloadParam *)malloc(sizeof(struct sessionToPayloadParam));
    deviceData = (struct sessionToPayloadParam *)malloc(sizeof(struct sessionToPayloadParam)); 
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    int32_t status = 0;
    int32_t i;
    int32_t dev_id;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttr;
    s->getStreamAttributes(&sAttr);
    char epName[128] = {0};
    std::string epname;

    PayloadBuilder *builder = new PayloadBuilder();
    status = rm->getStreamTag(streamTag);
    
    sessionData->direction = sAttr.direction;
    //decision based on stream attributes
    if (sessionData->direction == QAL_AUDIO_INPUT) {
        sessionData->sampleRate = sAttr.in_media_config.sample_rate;
        sessionData->bitWidth = sAttr.in_media_config.bit_width;
        sessionData->numChannel = sAttr.in_media_config.ch_info->channels;
    } else {
        sessionData->sampleRate = sAttr.out_media_config.sample_rate;
        sessionData->bitWidth = sAttr.out_media_config.bit_width;
        sessionData->numChannel = sAttr.out_media_config.ch_info->channels;
    }
    sessionData->metadata = NULL;
    QAL_ERR(LOG_TAG,"%s,: session bit width %d, sample rate %d, and channels %d\n",__func__, sessionData->bitWidth,sessionData->sampleRate,sessionData->numChannel);

    for (i=0; i<streamTag.size(); i++) {
        moduleInfo = NULL;
        moduleInfoSize = 0;
        QAL_ERR(LOG_TAG,"tag id %x",streamTag[i]);
        status = gslGetTaggedModuleInfo(gkv, streamTag[i], &moduleInfo, &moduleInfoSize);
        if (status != 0)
            continue;
        else {
            payload = NULL;
            payloadSize = 0;
            builder->payloadStreamConfig(&payload, &payloadSize, moduleInfo, streamTag[i], sessionData);
            status = gslSetCustomConfig(graphHandle, payload, payloadSize);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
                //goto exit;
            }
        }
    }
    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }

    for (int32_t i=0; i<(associatedDevices.size()); i++) {
        dev_id = associatedDevices[i]->getDeviceId();
        rm->getDeviceEpName(dev_id, epname);
        QAL_ERR(LOG_TAG,"%s: epname = %s", __func__, epname.c_str());
    }
    
    status = rm->getDeviceTag(deviceTag);
    for (i=0; i < deviceTag.size(); i++) {
        moduleInfo = NULL;
        moduleInfoSize = 0;
        deviceData->metadata = NULL;
        status = gslGetTaggedModuleInfo(gkv, deviceTag[i], &moduleInfo, &moduleInfoSize);
        if (status != 0)
            continue;
        else {
            payload = NULL;
            payloadSize = 0;
            if(deviceTag[i] == DEVICE_HW_ENDPOINT_RX) {
                for (int32_t i=0; i<(associatedDevices.size()); i++) {
                    dev_id = associatedDevices[i]->getDeviceId();
                    if(dev_id >=QAL_DEVICE_NONE && dev_id <= QAL_DEVICE_OUT_PROXY) {
                        rm->getDeviceEpName(dev_id, epname);
                        associatedDevices[i]->getDeviceAtrributes(&dAttr);
                    } else 
                        continue;
                    sessionData->direction = PLAYBACK;
                }
            } else if (deviceTag[i] == DEVICE_HW_ENDPOINT_TX) {
                for (int32_t i=0; i<(associatedDevices.size()); i++) {
                    dev_id = associatedDevices[i]->getDeviceId();
                    if(dev_id >=QAL_DEVICE_IN_HANDSET_MIC && dev_id <= QAL_DEVICE_IN_PROXY) {
                        rm->getDeviceEpName(dev_id, epname);
                        associatedDevices[i]->getDeviceAtrributes(&dAttr);
                    } else 
                        continue;
                    sessionData->direction = RECORD;
                }
            } else {
                continue;
            }
            deviceData->bitWidth = dAttr.config.bit_width;
            deviceData->sampleRate = dAttr.config.sample_rate;
            deviceData->numChannel = dAttr.config.ch_info->channels;
            QAL_ERR(LOG_TAG,"%s,: EP Device bit width %d, sample rate %d, and channels %d\n",__func__, deviceData->bitWidth,deviceData->sampleRate,deviceData->numChannel);
            switch (deviceData->sampleRate) {
                case SAMPLINGRATE_8K :
                    setConfig(s,MODULE,MFC_SR_8K);
                    break;
                case SAMPLINGRATE_16K :
                    setConfig(s,MODULE,MFC_SR_16K);
                    break;
                case SAMPLINGRATE_32K :
                    setConfig(s,MODULE,MFC_SR_32K);
                    break;
                case SAMPLINGRATE_44K :
                    setConfig(s,MODULE,MFC_SR_44K);
                    break;
                case SAMPLINGRATE_48K :
                    setConfig(s,MODULE,MFC_SR_48K);
                    break;
                case SAMPLINGRATE_96K :
                    setConfig(s,MODULE,MFC_SR_96K);
                    break;
                case SAMPLINGRATE_192K :
                    setConfig(s,MODULE,MFC_SR_192K);
                    break;
                case SAMPLINGRATE_384K :
                    setConfig(s,MODULE,MFC_SR_384K);
                    break;
                default:
                    QAL_ERR(LOG_TAG,"%s: Invalid sample rate = %d\n", __func__, deviceData->sampleRate);
            }
            
            builder->payloadDeviceEpConfig(&payload, &payloadSize, moduleInfo, deviceTag[i], deviceData, epname);
            status = gslSetCustomConfig(graphHandle, payload, payloadSize);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
                //goto exit;
            }
        } 
    }

    for (i=0; i < deviceTag.size(); i++) {
        moduleInfo = NULL;
        moduleInfoSize = 0;
        status = gslGetTaggedModuleInfo(gkv, deviceTag[i], &moduleInfo, &moduleInfoSize);
        deviceData->metadata = NULL;
        if (status != 0)
            continue;
        else {
            payload = NULL;
            payloadSize = 0;
            if(deviceTag[i] == DEVICE_HW_ENDPOINT_RX) {
                for (int32_t i=0; i<(associatedDevices.size()); i++) {
                    dev_id = associatedDevices[i]->getDeviceId();
                    if(dev_id >=QAL_DEVICE_NONE && dev_id <= QAL_DEVICE_OUT_PROXY) {
                       associatedDevices[i]->getDeviceAtrributes(&dAttr);
		                } else
                        continue;
                    sessionData->direction = PLAYBACK;
                }
            } else if (deviceTag[i] == DEVICE_HW_ENDPOINT_TX) {
                for (int32_t i=0; i<(associatedDevices.size()); i++) {
                    dev_id = associatedDevices[i]->getDeviceId();
                    if(dev_id >=QAL_DEVICE_IN_HANDSET_MIC && dev_id <= QAL_DEVICE_IN_PROXY) {
                       associatedDevices[i]->getDeviceAtrributes(&dAttr);
                    } else
                        continue;               
                    sessionData->direction = RECORD; 
		            }
            } else
                continue;
            deviceData->bitWidth = dAttr.config.bit_width;
            deviceData->sampleRate = dAttr.config.sample_rate;
            deviceData->numChannel = dAttr.config.ch_info->channels;
            QAL_ERR(LOG_TAG,"%s,: Device bit width %d, sample rate %d, and channels %d\n",__func__, deviceData->bitWidth,deviceData->sampleRate,deviceData->numChannel);
            builder->payloadDeviceConfig(&payload, &payloadSize, moduleInfo, deviceTag[i], deviceData);
            status = gslSetCustomConfig(graphHandle, payload, payloadSize);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
                //goto exit;
            }
        } 
    }

exit:    
    return status;
}

int SessionGsl::prepare(Stream * s) {
    int status = 0;
    size_t in_buf_size = 0,in_buf_count = 0;
    size_t out_buf_size = 0,out_buf_count = 0;
    struct qal_stream_attributes sAttr;
    s->getStreamAttributes(&sAttr);

    QAL_ERR(LOG_TAG,"%s: enter direction:%d ", __func__, sAttr.direction);

    status = gslIoctl(graphHandle, GSL_CMD_PREPARE, NULL, 0);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to prepare the graph", __func__);
        goto exit;
    }
    s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
    switch (sAttr.direction) {
    case QAL_AUDIO_INPUT:
            status = readBufferInit(s, in_buf_count, in_buf_size, DATA_MODE_BLOCKING);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Tx session readBufferInit is failed with status %d",__func__,status);
                goto exit;
            }
            break;
    case QAL_AUDIO_OUTPUT:
            status = writeBufferInit(s, out_buf_count, out_buf_size, DATA_MODE_BLOCKING);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Rx session writeBufferInit is failed with status %d",__func__,status);
                goto exit;
            }
            break;
        case QAL_AUDIO_INPUT_OUTPUT:
            break;
        default:
            break;
   }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::readBufferInit(Stream * s, size_t noOfBuf, size_t bufSize, int flag) {
    int status = 0;

    QAL_ERR(LOG_TAG,"%s: enter \n bufSize:%d noOfBuf:%d flag:%d", __func__, bufSize, noOfBuf, flag);

    infoBuffer = (struct gslCmdGetReadWriteBufInfo*)malloc(sizeof(struct gslCmdGetReadWriteBufInfo));
    infoBuffer->buff_size = bufSize;
    infoBuffer->num_buffs = noOfBuf;
    infoBuffer->attritubes = flag;
    infoBuffer->start_threshold = 0;
    infoBuffer->stop_threshold = 0;

    size = sizeof(struct gslCmdGetReadWriteBufInfo);
    status = gslIoctl(graphHandle, GSL_CMD_CONFIGURE_READ_PARAMS, infoBuffer, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to initialize the read buffer in gsl", __func__);
        goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::writeBufferInit(Stream * s, size_t noOfBuf, size_t bufSize, int flag) {
    int status = 0;
    struct gslCmdGetReadWriteBufInfo buf;

    QAL_ERR(LOG_TAG,"%s: enter \n bufSize:%d noOfBuf:%d flag:%d", __func__, bufSize, noOfBuf, flag);

    infoBuffer = (struct gslCmdGetReadWriteBufInfo*)malloc(sizeof(struct gslCmdGetReadWriteBufInfo));
    infoBuffer->buff_size = bufSize;
    infoBuffer->num_buffs = noOfBuf;
    infoBuffer->attritubes = flag;
    infoBuffer->start_threshold = 0;
    infoBuffer->stop_threshold = 0;

    size = sizeof(struct gslCmdGetReadWriteBufInfo);
    status = gslIoctl(graphHandle, GSL_CMD_CONFIGURE_WRITE_PARAMS, infoBuffer, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to initialize write buffer in gsl", __func__);
        goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::close(Stream *s) {
    int status = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslClose(graphHandle);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to close the graph", __func__);
        goto exit;
    }
    QAL_ERR(LOG_TAG,"%s: gsl_close successful", __func__);

    free(gkv->kvp);
    free(gkv);
    gkv = NULL;
    free(ckv->kvp);
    free(ckv);
    ckv = NULL;
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::start(Stream *s) {
    int status = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslIoctl(graphHandle, GSL_CMD_START, payload, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to start the graph", __func__);
        goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionGsl::pause()
{
    int status = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);
    status = gsl_ioctl_(graphHandle, GSL_CMD_PAUSE, payload, size);
    if(0 != status)
    {
         goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}*/

int SessionGsl::stop(Stream * s) {
    int status = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslIoctl(graphHandle, GSL_CMD_STOP, payload, size);
    if(0 != status) {
         QAL_ERR(LOG_TAG,"%s: Failed to stop the graph", __func__);
         goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionGsl::changeGraph(struct gslKeyVector * graphKeyVectors,
                            struct gslKeyVector * calKeyVectors)
{
    int status = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);
    status = gsl_ioctl_(graphHandle, , payload, size);
    if(0 != status)
    {
         goto exit;
    }
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
*/

int SessionGsl::setConfig(Stream *s, configType type, int tag) {
    int status = 0;
    uint32_t tagsent;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p type:%d tag:%d\n", __func__, graphHandle, type, tag);
    switch (type) {
        case GRAPH:
/*          status = stop();
            if (0 != status) {
                goto exit;
            }
            status = changeGraph(graphKeyVectors, calKeyVectors);
            if (0 != status) {
                goto exit;
            }*/
            break;
        case MODULE:
            tkv = new gsl_key_vector;
            status = populateTkv(s, tkv, tag, &tagsent);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the tag configuration\n", __func__);
                goto exit;
            }
            
            QAL_ERR(LOG_TAG,"%s: enter MODULE: tag:%d tagsent:%x \n", __func__, tag, tagsent);            
            QAL_ERR(LOG_TAG,"%s: tkv key %x value %x\n", __func__,(tkv->kvp[0].key),(tkv->kvp[0].value));
            status = gslSetConfig(graphHandle, tagsent, tkv);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set tag data\n", __func__);
                goto exit;
            } 
            break;
        case CALIBRATION:
            ckv = new gsl_key_vector;
            status = populateCkv(s, ckv, tag, graphHandle, this);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate calibration data\n", __func__);
                goto exit;
            }
            QAL_ERR(LOG_TAG,"%s: graph handle %x\n", __func__, graphHandle);
            QAL_ERR(LOG_TAG,"%s: ckv key %x value %x\n", __func__,(ckv->kvp[0].key),(ckv->kvp[0].value));
            status = gslSetCal(graphHandle, ckv);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the calibration data\n", __func__);
                goto exit;
            }
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
            goto exit;
    }

exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::fileRead(Stream *s, int tag, struct qal_buffer *buf, int * size) {
    std::fstream fs;
    QAL_ERR(LOG_TAG,"%s: enter", __func__);

    fs.open ("/data/test.wav", std::fstream::binary | std::fstream::in | std::fstream::app);
    QAL_ERR(LOG_TAG,"file open success");
    char * buff = static_cast<char *>(buf->buffer);
    if(seek != 0) {
       fs.seekp(seek);
    }
    fs.read (buff,buf->size);
    seek += buf->size;
    QAL_ERR(LOG_TAG,"file read success");
    fs.close();
    QAL_ERR(LOG_TAG,"file close success");
    *size = (int)(buf->size);
    QAL_ERR(LOG_TAG,"size: %d", *size);
    return 0;
}

int SessionGsl::read(Stream *s, int tag, struct qal_buffer *buf, int * size) {
    int status = 0, bytesRead = 0, bytesToRead = 0, offset = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p buf:%p tag:%d", __func__, graphHandle, buf, tag);
    //fileRead(s,tag,buf,size);
    //return 0;
    if (!buf || !s) {
        QAL_ERR(LOG_TAG,"%s: Invalid stream or buffer", __func__);
        status = -EINVAL;
        goto exit;
    }

    struct gsl_buff gslBuff;
    QAL_ERR(LOG_TAG,"%s: bufsize:%d bufNo:%d", __func__, infoBuffer->buff_size, infoBuffer->num_buffs);

    while (1) {
        offset = bytesRead + buf->offset;
        bytesToRead = buf->size - offset;
        if (!bytesToRead)
            break;
        gslBuff.flags = 0;
        if ((bytesToRead / infoBuffer->buff_size) >= 1)
            gslBuff.size = infoBuffer->buff_size;
        else
            gslBuff.size = bytesToRead;


        uint32_t sizeRead;
        void *data = buf->buffer;
        data = static_cast<char*>(data) + offset;
        gslBuff.addr = static_cast<uint8_t*>(data);
        status = gslRead(graphHandle, tag, &gslBuff, &sizeRead);
        if ((0 != status) || (sizeRead == 0)) {
            QAL_ERR(LOG_TAG,"%s: Failed to read data from gsl %d bytes read %d", __func__, status, sizeRead);
            break;
        }
        QAL_ERR(LOG_TAG,"%s:%d sizeRead variable", __func__, __LINE__, sizeRead);
        //buf->ts = gsl_buf.timestamp;
        QAL_ERR(LOG_TAG,"%s: bytes read %d and %d",__func__, bytesRead,sizeRead);
        bytesRead += sizeRead;
    }
exit:
    *size = bytesRead;
    QAL_ERR(LOG_TAG,"%s: exit bytesRead:%d status:%d ", __func__, bytesRead, status);
    return status;
}

int SessionGsl::fileWrite(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) {
    std::fstream fs;
    QAL_ERR(LOG_TAG,"%s: enter", __func__);

    fs.open ("/data/test.wav", std::fstream::binary | std::fstream::out | std::fstream::app);
    QAL_ERR(LOG_TAG,"file open success");
    char * buff=static_cast<char *>(buf->buffer);
    fs.write (buff,buf->size);
    QAL_ERR(LOG_TAG,"file write success");
    fs.close();
    QAL_ERR(LOG_TAG,"file close success");
    *size = (int)(buf->size);
    QAL_ERR(LOG_TAG,"size: %d", *size);
    return 0;
}

int SessionGsl::write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) {
    int status = 0, bytesWritten = 0, bytesRemaining = 0, offset = 0;
    uint32_t sizeWritten = 0;
    QAL_ERR(LOG_TAG,"%s: enter graphHanlde:%p buf:%p tag:%d flag:%d", __func__, graphHandle, buf, tag, flag);

    void *data = nullptr;
    struct gsl_buff gslBuff;
    gslBuff.timestamp = (uint64_t) buf->ts;

    bytesRemaining = buf->size;

    while ((bytesRemaining / infoBuffer->buff_size) > 1) {
        gslBuff.flags = 0;

        offset = bytesWritten + buf->offset;
        gslBuff.size = infoBuffer->buff_size;
        data = buf->buffer;
        data = static_cast<char *>(data) + offset;

        gslBuff.addr = static_cast<uint8_t *>(data);
        sizeWritten = 0;  //initialize 0
        status = gslWrite(graphHandle, tag, &gslBuff, &sizeWritten);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: Failed to write the data to gsl", __func__);
            return status;
        }
        bytesWritten += sizeWritten;
        bytesRemaining -= sizeWritten;
    }

    if (BUFFER_EOS == flag)
        gslBuff.flags = BUFF_FLAG_EOS;
    else
        gslBuff.flags = 0;
    offset = bytesWritten + buf->offset;
    gslBuff.size = bytesRemaining;
    data = buf->buffer;
    data = static_cast<char *>(data) + offset;
    gslBuff.addr = static_cast<uint8_t *>(data);
    sizeWritten = 0;  //0
    status = gslWrite(graphHandle, tag, &gslBuff, &sizeWritten);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to write the data to gsl", __func__);
        goto exit;
    }
    bytesWritten += sizeWritten;
    *size = bytesWritten;

exit:
    QAL_ERR(LOG_TAG,"%s: exit bytesWritten:%d status:%d ", __func__, bytesWritten, status);
    return 0;
}

int SessionGsl::setParameters(Stream *s, uint32_t param_id, void *payload)
{
    int status = 0;
    switch (param_id)
    {
        case PARAM_ID_DETECTION_ENGINE_SOUND_MODEL:
        {
            struct qal_st_sound_model *pSoundModel = NULL;
            pSoundModel = (struct qal_st_sound_model *)payload;
            status = populateSVASoundModel(this, DEVICE_SVA, graphHandle, pSoundModel);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to set sound model to gsl", __func__);
                goto exit;
            }
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP:
        {
            struct detection_engine_config_voice_wakeup *pWakeUpConfig = NULL;
            pWakeUpConfig = (struct detection_engine_config_voice_wakeup *)payload;
            status = populateSVAWakeUpConfig(this, DEVICE_SVA, graphHandle, pWakeUpConfig);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to set wake up config to gsl", __func__);
                goto exit;
            }
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG:
        {
            // register callback in gsl
            status = gslRegisterEventCallBack(graphHandle, stCallBack, (void*)s);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to register recognition callback in gsl", __func__);
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "%s:%d",__func__,__LINE__);
            struct detection_engine_generic_event_cfg *pEventConfig = NULL;
            pEventConfig = (struct detection_engine_generic_event_cfg *)payload;
            // set custom config for detection event
            status = populateSVAEventConfig(this, DEVICE_SVA, graphHandle, pEventConfig);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to set detection event config to gsl", __func__);
                goto exit;
            }
            break;
        }
        case PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG:
        {
            struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufConfig = NULL;
            pWakeUpBufConfig = (struct detection_engine_voice_wakeup_buffer_config *)payload;
            status = populateSVAWakeUpBufferConfig(this, DEVICE_SVA, graphHandle, pWakeUpBufConfig);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to set wake up config to gsl", __func__);
                goto exit;
            }
            break;
        }
        case PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION:
        {
            struct audio_dam_downstream_setup_duration *pSetupDuration = NULL;
            pSetupDuration = (struct audio_dam_downstream_setup_duration *)payload;
            status = populateSVAStreamSetupDuration(this, DEVICE_ADAM, graphHandle, pSetupDuration);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to set down stream setup duration to gsl", __func__);
                goto exit;
            }
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_RESET:
            status = populateSVAEngineReset(this, DEVICE_SVA, graphHandle, s);
            if (status != 0)
            {
                QAL_ERR(LOG_TAG, "%s: Failed to reset detection engine", __func__);
                goto exit;
            }
            break;
        default:
            QAL_ERR(LOG_TAG, "%s: Unsupported param id %u", __func__, param_id);
            status = -EINVAL;
            goto exit;
    }

exit:
    return status;
}

void SessionGsl::stCallBack(struct gsl_event_cb_params *event_params, void *client_data)
{
    QAL_ERR(LOG_TAG, "stCallBack invoked");
    qal_stream_callback callBack;
    Stream *s = (Stream *)client_data;
    s->getCallBack(&callBack);
    QAL_INFO(LOG_TAG, "CallBack acquired %p", callBack);
    qal_stream_handle_t *stream_handle = static_cast<void*>(s);
    uint32_t event_id = event_params->event_id;
    uint32_t *event_data = (uint32_t *)(event_params->event_payload);
    callBack(stream_handle, event_id, event_data, NULL);
}

void SessionGsl::checkAndConfigConcurrency(Stream *s)
{
    int32_t status = 0;
    std::shared_ptr<Device> rxDevice = nullptr;
    std::shared_ptr<Device> txDevice = nullptr;
    std::vector <std::pair<int,int>> keyVector;
    struct gsl_cmd_graph_select device_graph = {{0, nullptr}, {0, nullptr}};
    std::vector <Stream *> activeStreams;
    qal_stream_type_t txStreamType;
    std::vector <std::shared_ptr<Device>> activeDevices;
    std::vector <std::shared_ptr<Device>> deviceList;

    // get all active devices from rm and
    // determine Rx and Tx for concurrency usecase
    rm->getactivedevices(activeDevices);
    for (int i = 0; i < activeDevices.size(); i++)
    {
        int deviceId = activeDevices[i]->getDeviceId();
        if (deviceId == QAL_DEVICE_OUT_SPEAKER)
            rxDevice = activeDevices[i];

        if (deviceId >= QAL_DEVICE_IN_HANDSET_MIC &&
            deviceId <= QAL_DEVICE_IN_TRI_MIC)
            txDevice = activeDevices[i];
    }

    if (!rxDevice || !txDevice)
    {
        QAL_VERBOSE(LOG_TAG, "No need to handle for concurrency");
        return;
    }
    QAL_INFO(LOG_TAG, "rx device %d, tx device %d", rxDevice->getDeviceId(), txDevice->getDeviceId());
    // get associated device list
    status = s->getAssociatedDevices(deviceList);
    if (status != 0)
    {
        QAL_ERR(LOG_TAG, "Failed to get associated device");
        return;
    }

    // determine concurrency usecase
    for (int i = 0; i < deviceList.size(); i++)
    {
        std::shared_ptr<Device> dev = deviceList[i];
        if (dev == rxDevice)
        {
            rm->getactivestreams(txDevice, activeStreams);
            for (int j = 0; j < activeStreams.size(); j++)
            {
                activeStreams[j]->getStreamType(&txStreamType);
            }
        }
        else if (dev == txDevice)
            s->getStreamType(&txStreamType);
        else
        {
            QAL_VERBOSE(LOG_TAG, "Concurrency usecase exists, not related to current stream");
            return;
        }
    }
    QAL_INFO(LOG_TAG, "tx stream type = %d", txStreamType);
    // TODO: use table to map types/devices to key values
    if (txStreamType == QAL_STREAM_VOICE_UI)
        keyVector.push_back(std::make_pair(STREAM_TYPE, VOICE_UI_EC_REF_PATH));
    else
        // TODO: handle for other concurrency usecases also
        return;

    if (txDevice->getDeviceId() >= QAL_DEVICE_IN_HANDSET_MIC ||
        txDevice->getDeviceId() <= QAL_DEVICE_IN_TRI_MIC)
        keyVector.push_back(std::make_pair(DEVICETX, HANDSETMIC));

    if (rxDevice->getDeviceId() == QAL_DEVICE_OUT_SPEAKER)
        keyVector.push_back(std::make_pair(DEVICERX, SPEAKER));
    device_graph.graph_key_vector.num_kvps = keyVector.size();
    device_graph.graph_key_vector.kvp = new struct gsl_key_value_pair[keyVector.size()];
    for(int32_t i = 0; i < (keyVector.size()); i++) {
        device_graph.graph_key_vector.kvp[i].key = keyVector[i].first;
        device_graph.graph_key_vector.kvp[i].value = keyVector[i].second;
    }
    status = gslIoctl(graphHandle, GSL_CMD_ADD_GRAPH, &device_graph, sizeof(device_graph));
    if (status)
        QAL_ERR(LOG_TAG, "Failed to add graph");
    delete(device_graph.graph_key_vector.kvp);
}

