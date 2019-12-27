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

#define LOG_TAG "SessionAlsaUtils"

#include "SessionAlsaUtils.h"

#include <sstream>
#include <string>
//#include "SessionAlsa.h"
//#include "SessionAlsaPcm.h"
//#include "SessionAlsaCompress.h"
#include "ResourceManager.h"
#include <agm_api.h>
#include "detection_cmn_api.h"
#include "spr_api.h"
#include "apm_api.h"
#include <tinyalsa/asoundlib.h>
#include <sound/asound.h>


static constexpr const char* const COMPRESS_SND_DEV_NAME_PREFIX = "COMPRESS";
static constexpr const char* const PCM_SND_DEV_NAME_PREFIX = "PCM";

static const char *feCtrlNames[] = {
    " control",
    " metadata",
    " connect",
    " disconnect",
    " setParam",
    " getTaggedInfo",
    " SetParamTag",
    " GetParam",
    " echoReference",
    " Sidetone",
    " loopback",
    " event",
    " setcal",
};

static const char *beCtrlNames[] = {
    " metadata",
    " rate ch fmt",
};

struct agmMetaData {
    uint8_t *buf;
    uint32_t size;
    agmMetaData(uint8_t *b, uint32_t s)
        :buf(b),size(s) {}
};

SessionAlsaUtils::~SessionAlsaUtils()
{

}

bool SessionAlsaUtils::isRxDevice(uint32_t devId)
{
    if ((devId >= QAL_DEVICE_OUT_HANDSET) && (devId < QAL_DEVICE_IN_HANDSET_MIC))
        return true;

    return false;
}

std::shared_ptr<Device> SessionAlsaUtils::getDeviceObj(int32_t beDevId,
        std::vector<std::shared_ptr<Device>> &associatedDevices)
{
    for (int i = 0; i < associatedDevices.size(); ++i) {
        if (beDevId == associatedDevices[i]->getSndDeviceId())
            return associatedDevices[i];
    }
    return nullptr;
}

unsigned int SessionAlsaUtils::bitsToAlsaFormat(unsigned int bits)
{
    switch (bits) {
        case 32:
            return SNDRV_PCM_FORMAT_S32_LE;
        case 8:
            return SNDRV_PCM_FORMAT_S8;
        case 24:
            return SNDRV_PCM_FORMAT_S24_3LE;
        default:
        case 16:
            return SNDRV_PCM_FORMAT_S16_LE;
    };
}



int SessionAlsaUtils::setMixerCtlData(struct mixer_ctl *ctl, MixerCtlType id, void *data, int size)
{

    int rc = -EINVAL;

    if (!ctl || !data || size == 0) {
        QAL_ERR(LOG_TAG,"%s: invalid mixer ctrl data passed", __func__);
        goto error;
    }

    switch (id) {
        case MixerCtlType::MIXER_SET_ID_STRING:
            mixer_ctl_set_enum_by_string(ctl, (const char *)data);
            break;
        case MixerCtlType::MIXER_SET_ID_VALUE:
            mixer_ctl_set_value(ctl, SNDRV_CTL_ELEM_TYPE_BYTES, *((int *)data));
            break;
        case MixerCtlType::MIXER_SET_ID_ARRAY:
            mixer_ctl_set_array(ctl, data, size);
            break;
    }

error:
    return rc;

}

void SessionAlsaUtils::getAgmMetaData(const std::vector <std::pair<int, int>> &kv,
        const std::vector <std::pair<int, int>> &ckv, struct prop_data *propData,
        struct agmMetaData &md)
{
    uint8_t *metaData = NULL;
    uint8_t *ptr = NULL;
    struct agm_key_value *kvPtr = NULL;
    uint32_t mdSize = 0;

    md.buf = nullptr;
    md.size = 0;

    if (kv.size() == 0) {
        QAL_DBG(LOG_TAG, "key vector size 0");
        return;
    }

    mdSize = sizeof(uint32_t) * 4 +
        ((kv.size() + ckv.size()) * sizeof(struct agm_key_value));
    if (propData)
        mdSize += sizeof(uint32_t) * propData->num_values;

    metaData = (uint8_t*)calloc(1, mdSize);
    if (!metaData) {
        QAL_ERR(LOG_TAG, "Failed to allocate memory for agm meta data");
        return;
    }

    // Fill in gkv part
    ptr = metaData;
    *((uint32_t *)ptr) = kv.size();
    ptr += sizeof(uint32_t);
    kvPtr = (struct agm_key_value *)ptr;
    for (int i = 0; i < kv.size(); i++) {
        kvPtr[i].key = kv[i].first;
        kvPtr[i].value = kv[i].second;
    }

    // Fill in ckv part
    ptr += kv.size() * sizeof(struct agm_key_value);
    *((uint32_t *)ptr) = ckv.size();
    ptr += sizeof(uint32_t);
    kvPtr = (struct agm_key_value *)ptr;
    for (int i = 0; i < ckv.size(); i++) {
        kvPtr[i].key = ckv[i].first;
        kvPtr[i].value = ckv[i].second;
    }

    // Fill in prop info if any
    if (propData) {
        ptr += ckv.size() * sizeof(struct agm_key_value);
        *((uint32_t *)ptr) = propData->prop_id;
        ptr += sizeof(uint32_t);
        *((uint32_t *)ptr) = propData->num_values;
        ptr += sizeof(uint32_t);
        if (propData->num_values != 0)
            memcpy(ptr, (uint8_t *)propData->values, sizeof(uint32_t) * propData->num_values);
    }
    md.buf = metaData;
    md.size = mdSize;
}

int SessionAlsaUtils::getTagMetadata(int32_t tagsent, std::vector <std::pair<int, int>> &tkv,
        struct agm_tag_config *tagConfig)
{
    int status = 0;
    if (tkv.size() == 0 || !tagConfig) {
        QAL_ERR(LOG_TAG,"%s: invalid key values passed", __func__);
        status = -EINVAL;
        goto error;
    }

    tagConfig->tag = tagsent;
    tagConfig->num_tkvs = tkv.size();

    for (int i=0;i < tkv.size(); i++) {
        tagConfig->kv[i].key = tkv[i].first;
        tagConfig->kv[i].value = tkv[i].second;
    }
error:
    return status;

}

int SessionAlsaUtils::getCalMetadata(std::vector <std::pair<int,int>> &ckv, struct agm_cal_config* calConfig)
{
    int status = 0;
    if (ckv.size() == 0 || !calConfig) {
        QAL_ERR(LOG_TAG,"%s: invalid key values passed", __func__);
        status = -EINVAL;
        goto error;
    }

    calConfig->num_ckvs = ckv.size();

    for (int i = 0; i < ckv.size(); i++) {
        calConfig->kv[i].key = ckv[i].first;
        calConfig->kv[i].value = ckv[i].second;
    }

error:
    return status;
}

struct mixer_ctl *SessionAlsaUtils::getFeMixerControl(struct mixer *am, std::string feName,
        uint32_t idx)
{
    std::ostringstream cntrlName;

    cntrlName << feName << feCtrlNames[idx];
    QAL_DBG(LOG_TAG,"mixer control %s", cntrlName.str().data());
    return mixer_get_ctl_by_name(am, cntrlName.str().data());
}

struct mixer_ctl *SessionAlsaUtils::getBeMixerControl(struct mixer *am, std::string beName,
        uint32_t idx)
{
    std::ostringstream cntrlName;

    cntrlName << beName << beCtrlNames[idx];
    QAL_DBG(LOG_TAG,"mixer control %s", cntrlName.str().data());
    return mixer_get_ctl_by_name(am, cntrlName.str().data());
}

int SessionAlsaUtils::open(Stream * streamHandle, std::shared_ptr<ResourceManager> rmHandle,
    const std::vector<int> &DevIds, const std::vector<std::pair<int32_t, std::string>> &BackEnds)
{
    std::vector <std::pair<int, int>> streamKV;
    std::vector <std::pair<int, int>> streamCKV;
    std::vector <std::pair<int, int>> streamDeviceKV;
    std::vector <std::pair<int, int>> deviceKV;
    std::vector <std::pair<int, int>> emptyKV;
    int status = 0;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttr;
    struct agmMetaData streamMetaData(nullptr, 0);
    struct agmMetaData deviceMetaData(nullptr, 0);
    struct agmMetaData streamDeviceMetaData(nullptr, 0);
    std::ostringstream feName;
    struct mixer_ctl *feMixerCtrls[FE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    struct mixer_ctl *beMixerCtrls[BE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::shared_ptr<Device> beDevObj = nullptr;
    long aif_media_config[3];
    struct mixer *mixerHandle;
    uint32_t i;
    uint32_t streamPropId[] = {0x08000010, 1, 0x1}; /** gsl_subgraph_platform_driver_props.xml */
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */

//    PayloadBuilder* builder = new PayloadBuilder();

    status = streamHandle->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    status = streamHandle->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }

    PayloadBuilder* builder = new PayloadBuilder();
    // get streamKV
    if ((status = builder->populateStreamKV(streamHandle, streamKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get stream KV failed %d", status);
        goto exit;
    }
    status = builder->populateStreamCkv(streamHandle, streamCKV, 0,
            (struct qal_volume_data **)nullptr);
    if (status) {
        QAL_ERR(LOG_TAG, "get stream ckv failed %d", status);
        goto exit;
    }
    getAgmMetaData(streamKV, streamCKV, (struct prop_data *)streamPropId,
            streamMetaData);
    status = rmHandle->getAudioMixer(&mixerHandle);

    /** Get mixer controls (struct mixer_ctl *) for both FE and BE */
    if (sAttr.type == QAL_STREAM_COMPRESSED)
        feName << COMPRESS_SND_DEV_NAME_PREFIX << DevIds.at(0);
    else
        feName << PCM_SND_DEV_NAME_PREFIX << DevIds.at(0);

    for (i = FE_CONTROL; i <= FE_CONNECT; ++i) {
        feMixerCtrls[i] = SessionAlsaUtils::getFeMixerControl(mixerHandle, feName.str(), i);
        if (!feMixerCtrls[i]) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s %s", feName.str().data(),
                   feCtrlNames[i]); 
            status = -EINVAL;
            goto freeStreamMetaData;
        }
    }
    mixer_ctl_set_enum_by_string(feMixerCtrls[FE_CONTROL], "ZERO");
    if (streamMetaData.size)
        mixer_ctl_set_array(feMixerCtrls[FE_METADATA], (void *)streamMetaData.buf,
                streamMetaData.size);
    for (std::vector<std::pair<int32_t, std::string>>::const_iterator be = BackEnds.begin();
           be != BackEnds.end(); ++be) {
        if ((status = builder->populateDeviceKV(streamHandle, be->first, deviceKV)) != 0) {
            QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
            goto freeStreamMetaData;
        }

        if (sAttr.direction == QAL_AUDIO_OUTPUT)
            status = builder->populateDevicePPKV(streamHandle, be->first, deviceKV, 0,
                    emptyKV);
        else
            status = builder->populateDevicePPKV(streamHandle, 0, emptyKV, be->first, deviceKV);
        if (status != 0) {
            QAL_VERBOSE(LOG_TAG, "%s: get device PP KV failed %d", status);
            status = 0; /**< ignore device PP KV failures */
        }
        status = builder->populateStreamDeviceKV(streamHandle, be->first, streamDeviceKV);
        if (status) {
            QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
            status = 0; /**< ignore stream device KV failures */
        }
        getAgmMetaData(deviceKV, emptyKV, (struct prop_data *)devicePropId,
                deviceMetaData);
        getAgmMetaData(streamDeviceKV, emptyKV, (struct prop_data *)streamDevicePropId,
                streamDeviceMetaData);
        if (!streamMetaData.size && !deviceMetaData.size &&
                !streamDeviceMetaData.size) {
            QAL_ERR(LOG_TAG, "stream/device metadata is zero");
            status = -EINVAL;
            goto freeMetaData;
        }
        for (int n = BE_METADATA; n <= BE_MEDIAFMT; ++n) {
            beMixerCtrls[n] = SessionAlsaUtils::getBeMixerControl(mixerHandle, be->second, n);
            if (!beMixerCtrls[n]) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s %s", be->second.data(),
                        beCtrlNames[n]); 
                status = -EINVAL;
                goto freeMetaData;
            }
        }
        /** set mixer controls */
        if (deviceMetaData.size)
            mixer_ctl_set_array(beMixerCtrls[BE_METADATA], (void *)deviceMetaData.buf,
                    deviceMetaData.size);
        if (streamDeviceMetaData.size) {
            mixer_ctl_set_enum_by_string(feMixerCtrls[FE_CONTROL], be->second.data());
            mixer_ctl_set_array(feMixerCtrls[FE_METADATA], (void *)streamDeviceMetaData.buf,
                    streamDeviceMetaData.size);
        }
        mixer_ctl_set_enum_by_string(feMixerCtrls[FE_CONNECT], (be->second).data());

        if ((beDevObj = SessionAlsaUtils::getDeviceObj(be->first, associatedDevices)) != nullptr) {
            beDevObj->getDeviceAtrributes(&dAttr);
            aif_media_config[0] = dAttr.config.sample_rate;
            aif_media_config[1] = dAttr.config.ch_info->channels;
            aif_media_config[2] = bitsToAlsaFormat(dAttr.config.bit_width);

            mixer_ctl_set_array(beMixerCtrls[BE_MEDIAFMT], &aif_media_config,
                    sizeof(aif_media_config)/sizeof(aif_media_config[0]));
            QAL_DBG(LOG_TAG,"rate ch fmt %d %d %d", dAttr.config.sample_rate, dAttr.config.ch_info->channels,
                    dAttr.config.bit_width);
        }
        deviceKV.clear();
        streamDeviceKV.clear();
        free(streamDeviceMetaData.buf);
        free(deviceMetaData.buf);
        streamDeviceMetaData.buf = nullptr;
        deviceMetaData.buf = nullptr;
    }
freeMetaData:
    free(streamDeviceMetaData.buf);
    free(deviceMetaData.buf);
freeStreamMetaData:
    free(streamMetaData.buf);
exit:
   delete builder;
   return status;
}

int SessionAlsaUtils::getTimestamp(struct mixer *mixer, bool isCompress, const std::vector<int> &DevIds,
                                   uint32_t spr_miid, struct qal_session_time *stime)
{
    int status = 0;
    const char *getParamControl = "getParam";
    const char *stream = "PCM";
    struct mixer_ctl *ctl;
    std::ostringstream CntrlName;
    struct param_id_spr_session_time_t *spr_session_time;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    if (isCompress)
        stream = "COMPRESS";

    PayloadBuilder* builder = new PayloadBuilder();
    CntrlName<<stream<<DevIds.at(0)<<" "<<getParamControl;
    ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        return -ENOENT;
    }

    builder->payloadTimestamp(&payload, &payloadSize, spr_miid);
    status = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (0 != status) {
         QAL_ERR(LOG_TAG, "Set failed status = %d", status);
         goto exit;
    }
    memset(payload, 0, payloadSize);
    status = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (0 != status) {
         QAL_ERR(LOG_TAG, "Get failed status = %d", status);
         goto exit;
    }
    spr_session_time = (struct param_id_spr_session_time_t *)
                     (payload + sizeof(struct apm_module_param_data_t));
    stime->session_time.value_lsw = spr_session_time->session_time.value_lsw;
    stime->session_time.value_msw = spr_session_time->session_time.value_msw;
    stime->absolute_time.value_lsw = spr_session_time->absolute_time.value_lsw;
    stime->absolute_time.value_msw = spr_session_time->absolute_time.value_msw;
    stime->timestamp.value_lsw = spr_session_time->timestamp.value_lsw;
    stime->timestamp.value_msw = spr_session_time->timestamp.value_msw;
    //flags from Gecko are igonred
exit:
    delete builder;
    return status;
}

int SessionAlsaUtils::getModuleInstanceId(struct mixer *mixer, int device, const char *intf_name,
                       bool isCompress, int tag_id, uint32_t *miid)
{
    char const *stream = "PCM";
    char const *control = "getTaggedInfo";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0, i;
    void *payload;
    struct gsl_tag_module_info *tag_info;
    struct gsl_tag_module_info_entry *tag_entry;
    int offset = 0;

    ret = setStreamMetadataType(mixer, device, intf_name, isCompress);
    if (ret)
        return ret;

    if (isCompress)
        stream = "COMPRESS";

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if (!mixer_str)
        return -ENOMEM;

    snprintf(mixer_str, ctl_len, "%s%d %s", stream, device, control);

    QAL_DBG(LOG_TAG, " - mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }

    payload = calloc(1024, sizeof(char));
    if (!payload) {
        free(mixer_str);
        return -ENOMEM;
    }

    ret = mixer_ctl_get_array(ctl, payload, 1024);
    if (ret < 0) {
        QAL_ERR(LOG_TAG, "Failed to mixer_ctl_get_array\n");
        free(payload);
        free(mixer_str);
        return ret;
    }
    tag_info = (struct gsl_tag_module_info *)payload;
    QAL_DBG(LOG_TAG, "num of tags associated with stream %d is %d\n", device, tag_info->num_tags);
    ret = -1;
    tag_entry = (struct gsl_tag_module_info_entry *)(&tag_info->tag_module_entry[0]);
    offset = 0;
    for (i = 0; i < tag_info->num_tags; i++) {
        tag_entry += offset/sizeof(struct gsl_tag_module_info_entry);

        QAL_DBG(LOG_TAG, "tag id[%d] = %lx, num_modules = %lx\n", i, tag_entry->tag_id, tag_entry->num_modules);
        offset = sizeof(struct gsl_tag_module_info_entry) + (tag_entry->num_modules * sizeof(struct gsl_module_id_info_entry));
        if (tag_entry->tag_id == tag_id) {
            struct gsl_module_id_info_entry *mod_info_entry;

            if (tag_entry->num_modules) {
                 mod_info_entry = &tag_entry->module_entry[0];
                 *miid = mod_info_entry->module_iid;
                 QAL_DBG(LOG_TAG, "MIID is %x\n", *miid);
                 ret = 0;
                 break;
            }
        }
    }

    free(payload);
    free(mixer_str);
    return ret;
}

int SessionAlsaUtils::setMixerParameter(struct mixer *mixer, int device,
                        bool isCompress, void *payload, int size)
{
    char const *stream = "PCM";
    char const *control = "setParam";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;

    if (isCompress)
        stream = "COMPRESS";

    QAL_DBG(LOG_TAG, "- mixer -%s-\n", stream);
    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if (!mixer_str) {
        free(payload);
        return -ENOMEM;
    }
    snprintf(mixer_str, ctl_len, "%s%d %s", stream, device, control);

    QAL_DBG(LOG_TAG, "- mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }
    ret = mixer_ctl_set_array(ctl, payload, size);

    QAL_DBG(LOG_TAG, "ret = %d, cnt = %d\n", ret, size);
    free(mixer_str);
    return ret;
}

int SessionAlsaUtils::setStreamMetadataType(struct mixer *mixer, int device, const char *val, bool isCompress)
{
    char const *stream = "PCM";
    char const *control = "control";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;

    if (isCompress)
        stream = "COMPRESS";

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if(mixer_str == NULL) {
        ret = -ENOMEM;
        QAL_ERR(LOG_TAG,"%s:calloc failed",__func__);
        return ret;
    }
    snprintf(mixer_str, ctl_len, "%s%d %s", stream, device, control);
    QAL_DBG(LOG_TAG, "- mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }

    ret = mixer_ctl_set_enum_by_string(ctl, val);
    free(mixer_str);
    return ret;
}

int SessionAlsaUtils::registerMixerEvent(struct mixer *mixer, int device, const char *intf_name, bool isCompress, int tag_id, bool is_register)
{
    char const *stream = "PCM";
    char const *control = "event";
    char *mixer_str;
    struct mixer_ctl *ctl;
    struct agm_event_reg_cfg *event_cfg;
    int payload_size = 0;
    int ctl_len = 0,status = 0;
    uint32_t miid;

    // get module instance id
    status = SessionAlsaUtils::getModuleInstanceId(mixer, device, intf_name, false, tag_id, &miid);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", tag_id, status);
        return EINVAL;
    }


    if (isCompress)
        stream = "COMPRESS";

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if (!mixer_str)
        return -ENOMEM;

    snprintf(mixer_str, ctl_len, "%s%d %s", stream, device, control);

    QAL_DBG(LOG_TAG, "- mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }

    ctl_len = sizeof(struct agm_event_reg_cfg) + payload_size;
    event_cfg = (struct agm_event_reg_cfg *)calloc(1, ctl_len);
    if (!event_cfg) {
        free(mixer_str);
        return -ENOMEM;
    }

    event_cfg->module_instance_id = miid;
    event_cfg->event_id = EVENT_ID_DETECTION_ENGINE_GENERIC_INFO;
    event_cfg->event_config_payload_size = payload_size;
    event_cfg->is_register = is_register ? 1 : 0;

    status = mixer_ctl_set_array(ctl, event_cfg, ctl_len);
    free(event_cfg);
    free(mixer_str);
    return status;
}

int SessionAlsaUtils::setECRefPath(struct mixer *mixer, int device, bool isCompress, const char *intf_name)
{
    char const *stream = "PCM";
    char const *control = "echoReference";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0;
    int ret = 0;

    if (isCompress)
        stream = "COMPRESS";

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if(mixer_str == NULL) {
        ret = -ENOMEM;
        QAL_ERR(LOG_TAG,"%s:calloc failed",__func__);
        return ret;
    }
    snprintf(mixer_str, ctl_len, "%s%d %s", stream, device, control);

    printf("%s - mixer -%s-\n", __func__, mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        printf("Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }

    ret = mixer_ctl_set_enum_by_string(ctl, intf_name);
    free(mixer_str);
    return ret;
}

int SessionAlsaUtils::open(Stream * streamHandle, std::shared_ptr<ResourceManager> rmHandle,
    const std::vector<int> &RxDevIds, const std::vector<int> &TxDevIds,
    const std::vector<std::pair<int32_t, std::string>> &rxBackEnds,
    const std::vector<std::pair<int32_t, std::string>> &txBackEnds)
{
    std::vector <std::pair<int, int>> streamRxKV, streamTxKV;
    std::vector <std::pair<int, int>> streamRxCKV, streamTxCKV;
    std::vector <std::pair<int, int>> streamDeviceRxKV, streamDeviceTxKV;
    std::vector <std::pair<int, int>> deviceRxKV, deviceTxKV;
    // Using as empty key vector pairs
    std::vector <std::pair<int, int>> emptyKV;
    int status = 0;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttrRx, dAttrTx;
    struct agmMetaData streamRxMetaData(nullptr, 0);
    struct agmMetaData streamTxMetaData(nullptr, 0);
    struct agmMetaData deviceRxMetaData(nullptr, 0);
    struct agmMetaData deviceTxMetaData(nullptr, 0);
    struct agmMetaData streamDeviceRxMetaData(nullptr, 0);
    struct agmMetaData streamDeviceTxMetaData(nullptr, 0);
    std::ostringstream rxFeName, txFeName;
    struct mixer_ctl *rxFeMixerCtrls[FE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    struct mixer_ctl *rxBeMixerCtrls[BE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    struct mixer_ctl *txFeMixerCtrls[FE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    struct mixer_ctl *txBeMixerCtrls[BE_MAX_NUM_MIXER_CONTROLS] = { nullptr };
    std::vector<std::shared_ptr<Device>> associatedDevices;
    long aif_media_config_rx[3];
    long aif_media_config_tx[3];
    struct mixer *mixerHandle;
    uint32_t streamPropId[] = {0x08000010, 1, 0x1}; /** gsl_subgraph_platform_driver_props.xml */
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */
	uint32_t i, rxDevNum, txDevNum;

//    PayloadBuilder* builder = new PayloadBuilder();


    status = streamHandle->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    status = streamHandle->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }
    if (associatedDevices.size() != 2) {
        QAL_ERR(LOG_TAG, "%s: Loopback num devices expected 2, given:$d",
                associatedDevices.size());
        return status;
    }

    PayloadBuilder* builder = new PayloadBuilder();

    status = rmHandle->getAudioMixer(&mixerHandle);
    // get streamKV
    if ((status = builder->populateStreamKV(streamHandle, streamRxKV,
                    streamTxKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get stream KV for Rx/Tx failed %d", status);
        goto exit;
    }
    // get streamCKV
    status = builder->populateStreamCkv(streamHandle, streamRxCKV, 0,
            (struct qal_volume_data **)nullptr);
    if (status) {
        QAL_ERR(LOG_TAG, "%s: get stream ckv failed %d", status);
        goto exit;
    }
    // get deviceKV
    if ((status = builder->populateDeviceKV(streamHandle, rxBackEnds[0].first,
                    deviceRxKV, txBackEnds[0].first, deviceTxKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV for Rx/Tx failed %d", status);
        goto exit;
    }
     // get devicePP
    if ((status = builder->populateDevicePPKV(streamHandle,
                    rxBackEnds[0].first, deviceRxKV, txBackEnds[0].first,
                    deviceTxKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        goto exit;
    }
    // get streamdeviceKV
    status = builder->populateStreamDeviceKV(streamHandle, rxBackEnds[0].first,
            streamDeviceRxKV, txBackEnds[0].first, streamDeviceTxKV);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV for Rx/Tx failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }
    // get audio mixer
    SessionAlsaUtils::getAgmMetaData(streamRxKV, streamRxCKV, 
            (struct prop_data *)streamPropId, streamRxMetaData);
    SessionAlsaUtils::getAgmMetaData(deviceRxKV, emptyKV,
            (struct prop_data *)devicePropId, deviceRxMetaData);
    SessionAlsaUtils::getAgmMetaData(streamDeviceRxKV, emptyKV, 
            (struct prop_data *)streamDevicePropId, streamDeviceRxMetaData);
    if (!streamRxMetaData.size && !deviceRxMetaData.size &&
            !streamDeviceRxMetaData.size) {
        QAL_ERR(LOG_TAG, "stream/device RX metadata is zero");
        status = -EINVAL;
        goto freeRxMetaData;
    }

    SessionAlsaUtils::getAgmMetaData(streamTxKV, streamTxCKV, 
            (struct prop_data *)streamPropId, streamTxMetaData);
    SessionAlsaUtils::getAgmMetaData(deviceTxKV, emptyKV,
            (struct prop_data *)devicePropId, deviceTxMetaData);
    SessionAlsaUtils::getAgmMetaData(streamDeviceTxKV, emptyKV,
            (struct prop_data *)streamDevicePropId, streamDeviceTxMetaData);
    if (!streamTxMetaData.size && !deviceTxMetaData.size &&
            !streamDeviceTxMetaData.size) {
        QAL_ERR(LOG_TAG, "stream/device TX metadata is zero");
        status = -EINVAL;
        goto freeTxMetaData;
    }

    rxFeName << PCM_SND_DEV_NAME_PREFIX << RxDevIds.at(0);
    txFeName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0);

    for (i = FE_CONTROL; i <= FE_CONNECT; ++i) {
        rxFeMixerCtrls[i] = SessionAlsaUtils::getFeMixerControl(mixerHandle, rxFeName.str(), i);
        txFeMixerCtrls[i] = SessionAlsaUtils::getFeMixerControl(mixerHandle, txFeName.str(), i);
        if (!rxFeMixerCtrls[i] || !txFeMixerCtrls[i]) {
            QAL_ERR(LOG_TAG, "invalid mixer control: (%s%s)/(%s%s)",
                    rxFeName.str().data(), feCtrlNames[i],
                    txFeName.str().data(), feCtrlNames[i]);
            status = -EINVAL;
            goto freeTxMetaData;
        }
    }
    for (i = BE_METADATA; i <= BE_MEDIAFMT; ++i) {
        rxBeMixerCtrls[i] = SessionAlsaUtils::getBeMixerControl(mixerHandle,
                rxBackEnds[0].second, i);
        txBeMixerCtrls[i] = SessionAlsaUtils::getBeMixerControl(mixerHandle, txBackEnds[0].second, i);
        if (!rxBeMixerCtrls[i] || !txBeMixerCtrls[i]) {
            QAL_ERR(LOG_TAG, "invalid mixer control: (%s%s)/(%s%s)",
                    rxBackEnds[0].second.data(), beCtrlNames[i],
                    txBackEnds[0].second.data(), beCtrlNames[i]);
            status = -EINVAL;
            goto freeTxMetaData;
        }
    }
    if (SessionAlsaUtils::isRxDevice(associatedDevices[0]->getSndDeviceId()))
        rxDevNum = 0;
    else
        rxDevNum = 1;

    txDevNum = !rxDevNum;

    associatedDevices[rxDevNum]->getDeviceAtrributes(&dAttrRx);
    associatedDevices[txDevNum]->getDeviceAtrributes(&dAttrTx);

    /** set TX mixer controls */
    mixer_ctl_set_enum_by_string(txFeMixerCtrls[FE_CONTROL], "ZERO");
    if (streamTxMetaData.size)
        mixer_ctl_set_array(txFeMixerCtrls[FE_METADATA], (void *)streamTxMetaData.buf,
                streamTxMetaData.size);
    if (deviceTxMetaData.size)
        mixer_ctl_set_array(txBeMixerCtrls[BE_METADATA], (void *)deviceTxMetaData.buf,
                deviceTxMetaData.size);
    if (streamDeviceTxMetaData.size) {
        mixer_ctl_set_enum_by_string(txFeMixerCtrls[FE_CONTROL], txBackEnds[0].second.data());
        mixer_ctl_set_array(txFeMixerCtrls[FE_METADATA], (void *)streamDeviceTxMetaData.buf,
                streamDeviceTxMetaData.size);
    }
    mixer_ctl_set_enum_by_string(txFeMixerCtrls[FE_CONNECT], txBackEnds[0].second.data());

    aif_media_config_tx[0] = dAttrTx.config.sample_rate;
    aif_media_config_tx[1] = dAttrTx.config.ch_info->channels;
    aif_media_config_tx[2] = bitsToAlsaFormat(dAttrTx.config.bit_width);

    mixer_ctl_set_array(txBeMixerCtrls[BE_MEDIAFMT], &aif_media_config_tx,
            sizeof(aif_media_config_tx)/sizeof(aif_media_config_tx[0]));

    /** set RX mixer controls */
    mixer_ctl_set_enum_by_string(rxFeMixerCtrls[FE_CONTROL], "ZERO");
    if (streamRxMetaData.size)
        mixer_ctl_set_array(rxFeMixerCtrls[FE_METADATA], (void *)streamRxMetaData.buf,
                streamRxMetaData.size);
    if (deviceRxMetaData.size)
        mixer_ctl_set_array(rxBeMixerCtrls[BE_METADATA], (void *)deviceRxMetaData.buf,
                deviceRxMetaData.size);
    if (streamDeviceRxMetaData.size) {
        mixer_ctl_set_enum_by_string(rxFeMixerCtrls[FE_CONTROL], rxBackEnds[0].second.data());
        mixer_ctl_set_array(rxFeMixerCtrls[FE_METADATA], (void *)streamDeviceRxMetaData.buf,
                streamDeviceRxMetaData.size);
    }
    mixer_ctl_set_enum_by_string(rxFeMixerCtrls[FE_CONNECT], rxBackEnds[0].second.data());

    aif_media_config_rx[0] = dAttrRx.config.sample_rate;
    aif_media_config_rx[1] = dAttrRx.config.ch_info->channels;
    aif_media_config_rx[2] = bitsToAlsaFormat(dAttrRx.config.bit_width);

    mixer_ctl_set_array(rxBeMixerCtrls[BE_MEDIAFMT], &aif_media_config_rx,
            sizeof(aif_media_config_rx)/sizeof(aif_media_config_rx[0]));

    txFeMixerCtrls[FE_LOOPBACK] = getFeMixerControl(mixerHandle, txFeName.str(), FE_LOOPBACK);
    if (!txFeMixerCtrls[FE_LOOPBACK]) {
        QAL_ERR(LOG_TAG, "invalid mixer control %s%s",
                txFeName.str().data(), feCtrlNames[i]);
        status = -EINVAL;
        goto freeTxMetaData;
    }
    mixer_ctl_set_enum_by_string(txFeMixerCtrls[FE_LOOPBACK], rxFeName.str().data());
freeTxMetaData:
    free(streamDeviceTxMetaData.buf);
    free(deviceTxMetaData.buf);
    free(streamTxMetaData.buf);
freeRxMetaData:
    free(streamDeviceRxMetaData.buf);
    free(deviceRxMetaData.buf);
    free(streamRxMetaData.buf);
exit:
   return status;
}

int SessionAlsaUtils::disconnectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<ResourceManager> rmHandle, struct qal_device &dAttr,
        const std::vector<int> &pcmDevIds,
        const std::vector<std::pair<int32_t, std::string>> &aifBackEndsToDisconnect)
{
    std::ostringstream disconnectCtrlName;
    int status = 0;
    struct mixer *mixerHandle = nullptr;
    struct mixer_ctl *disconnectCtrl = nullptr;

    switch (streamType) {
        case QAL_STREAM_COMPRESSED:
            disconnectCtrlName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " disconnect";
            break;
        default:
            disconnectCtrlName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " disconnect";
            break;
    }
    status = rmHandle->getAudioMixer(&mixerHandle);
    disconnectCtrl = mixer_get_ctl_by_name(mixerHandle, disconnectCtrlName.str().data());
    if (!disconnectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", disconnectCtrlName.str().data());
        return -EINVAL;
    }
    /** Disconnect FE to BE */
    mixer_ctl_set_enum_by_string(disconnectCtrl, aifBackEndsToDisconnect[0].second.data());

    return status;
}

int SessionAlsaUtils::connectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<ResourceManager> rmHandle, struct qal_device &dAttr,
        const std::vector<int> &pcmDevIds,
        const std::vector<std::pair<int32_t, std::string>> &aifBackEndsToConnect)
{
    std::ostringstream connectCtrlName;
    std::ostringstream cntrlName;
    std::ostringstream aifMdName;
    std::ostringstream aifMfCtrlName;
    std::ostringstream feMdName;
    struct mixer_ctl *connectCtrl;
    std::vector <std::pair<int, int>> streamDeviceKV;
    std::vector <std::pair<int, int>> deviceKV;
    std::vector <std::pair<int, int>> emptyKV;
    int status = 0;
    struct agmMetaData deviceMetaData(nullptr, 0);
    struct agmMetaData streamDeviceMetaData(nullptr, 0);
    struct mixer_ctl *feCtrl = nullptr;
    struct mixer_ctl *feMdCtrl = nullptr;
    struct mixer_ctl *aifMdCtrl = nullptr;
    struct mixer_ctl *aifMfCtrl = nullptr;
    long aif_media_config[3];
    PayloadBuilder* builder = new PayloadBuilder();
    struct mixer *mixerHandle = nullptr;
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */
    struct sessionToPayloadParam deviceData;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    uint32_t miid;
    bool is_compress = false;


    status = rmHandle->getAudioMixer(&mixerHandle);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }

    status = builder->populateStreamDeviceKV(streamHandle,
            aifBackEndsToConnect[0].first, streamDeviceKV);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }
    if ((status = builder->populateDeviceKV(streamHandle,
                    aifBackEndsToConnect[0].first, deviceKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        return status;
    }
    if (SessionAlsaUtils::isRxDevice(aifBackEndsToConnect[0].first))
        status = builder->populateDevicePPKV(streamHandle,
                aifBackEndsToConnect[0].first, deviceKV,
                0, emptyKV);
    else
        status = builder->populateDevicePPKV(streamHandle, 0, emptyKV,
                aifBackEndsToConnect[0].first, deviceKV);

    if (status != 0) {
        QAL_ERR(LOG_TAG, "%s: get device PP KV failed %d", status);
        status = 0; /** ignore error */
    }
    SessionAlsaUtils::getAgmMetaData(deviceKV, emptyKV, (struct prop_data *)devicePropId,
             deviceMetaData);
    if (!deviceMetaData.size || !deviceMetaData.buf) {
        QAL_ERR(LOG_TAG, "get device meta data failed %d", status);
        return -EINVAL;
    }

    if (streamDeviceKV.size()) {
        SessionAlsaUtils::getAgmMetaData(streamDeviceKV, emptyKV,
                (struct prop_data *)streamDevicePropId,
                streamDeviceMetaData);
        if (!streamDeviceMetaData.size || !streamDeviceMetaData.buf) {
            QAL_ERR(LOG_TAG, "get stream device meta data failed %d", status);
            status = -EINVAL;
            goto free_devicemd;
        }
    }

    switch (streamType) {
        case QAL_STREAM_COMPRESSED:
            cntrlName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " control";
            aifMdName << aifBackEndsToConnect[0].second.data() << " metadata";
            connectCtrlName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " connect";
            aifMfCtrlName << aifBackEndsToConnect[0].second.data() << " rate ch fmt";
            feMdName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " metadata";
            is_compress = true;
            break;
        default:
            cntrlName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " control";
            aifMdName << aifBackEndsToConnect[0].second.data() << " metadata";
            connectCtrlName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " connect";
            aifMfCtrlName << aifBackEndsToConnect[0].second.data() << " rate ch fmt";
            feMdName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " metadata";
            break;
    }

    status = rmHandle->getAudioMixer(&mixerHandle);

    aifMdCtrl = mixer_get_ctl_by_name(mixerHandle, aifMdName.str().data());
    QAL_ERR(LOG_TAG,"mixer control %s", aifMdName.str().data());
    if (!aifMdCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMdName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_array(aifMdCtrl, (void *)deviceMetaData.buf, deviceMetaData.size);

    feCtrl = mixer_get_ctl_by_name(mixerHandle, cntrlName.str().data());
    QAL_ERR(LOG_TAG,"mixer control %s", cntrlName.str().data());
    if (!feCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", cntrlName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_enum_by_string(feCtrl, aifBackEndsToConnect[0].second.data());

    if (streamDeviceMetaData.size) {
        feMdCtrl = mixer_get_ctl_by_name(mixerHandle, feMdName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", feMdName.str().data());
        if (!feMdCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", feMdName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(feMdCtrl, (void *)streamDeviceMetaData.buf, streamDeviceMetaData.size);
    }
    aif_media_config[0] = dAttr.config.sample_rate;
    aif_media_config[1] = dAttr.config.ch_info->channels;
    aif_media_config[2] = bitsToAlsaFormat(dAttr.config.bit_width);

    aifMfCtrl = mixer_get_ctl_by_name(mixerHandle, aifMfCtrlName.str().data());
    QAL_ERR(LOG_TAG,"mixer control %s", aifMfCtrlName.str().data());
    if (!aifMfCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMfCtrlName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_array(aifMfCtrl, &aif_media_config,
            sizeof(aif_media_config)/sizeof(aif_media_config[0]));

    /* Get PSPD MFC MIID and configure to match to device config */
    /* This has to be done after sending all mixer controls and before connect */
    status = SessionAlsaUtils::getModuleInstanceId(mixerHandle, pcmDevIds.at(0),
                                                   aifBackEndsToConnect[0].second.data(),
                                                   is_compress, TAG_DEVICE_MFC_SR, &miid);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"getModuleInstanceId failed");
        return status;
    }
    QAL_ERR(LOG_TAG, "miid : %x id = %d, data %s, dev id = %d\n", miid,
            pcmDevIds.at(0), aifBackEndsToConnect[0].second.data(), dAttr.id);
    deviceData.bitWidth = dAttr.config.bit_width;
    deviceData.sampleRate = dAttr.config.sample_rate;
    deviceData.numChannel = dAttr.config.ch_info->channels;
    builder->payloadMFCConfig(&payload, &payloadSize, miid, &deviceData);
    status = SessionAlsaUtils::setMixerParameter(mixerHandle, pcmDevIds.at(0), is_compress, payload, payloadSize);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"setMixerParameter failed");
        return status;
    }

    connectCtrl = mixer_get_ctl_by_name(mixerHandle, connectCtrlName.str().data());
    if (!connectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_enum_by_string(connectCtrl, aifBackEndsToConnect[0].second.data());

free_streamdevicemd:
    free(streamDeviceMetaData.buf);
free_devicemd:
    free(deviceMetaData.buf);

    delete builder;
    return status;
}

