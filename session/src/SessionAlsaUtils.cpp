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




SessionAlsaUtils::~SessionAlsaUtils()
{

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

int SessionAlsaUtils::getAgmMetaData(const std::vector <std::pair<int, int>> &kv,
        const std::vector <std::pair<int, int>> &ckv, struct prop_data *propData,
        uint32_t &mdSize, uint8_t **data)
{
    int status = 0;
    uint8_t *metaData = NULL;
    uint8_t *ptr = NULL;
    struct agm_key_value *kvPtr = NULL;

    if (kv.size() == 0) {
        QAL_ERR(LOG_TAG, "Invalid key values passed");
        status = EINVAL;
        goto exit;
    }

    mdSize = sizeof(uint32_t) * 4 +
        ((kv.size() + ckv.size()) * sizeof(struct agm_key_value));
    if (propData)
        mdSize += sizeof(uint32_t) * propData->num_values;

    metaData = (uint8_t*)calloc(1, mdSize);
    if (!metaData) {
        QAL_ERR(LOG_TAG, "Failed to allocate memory for agm meta data");
        status = ENOMEM;
        goto exit;
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
    *data = metaData;

exit:
    return status;
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


int SessionAlsaUtils::open(Stream * streamHandle, std::shared_ptr<ResourceManager> rmHandle,
    const std::vector<int> &DevIds, const std::vector<std::string> &BackEnds)
{
    std::vector <std::pair<int, int>> streamKV;
    std::vector <std::pair<int, int>> streamCKV;
    std::vector <std::pair<int, int>> streamDeviceKV;
    std::vector <std::pair<int, int>> deviceKV;
    // Using as empty key vector pairs
    std::vector <std::pair<int, int>> emptyKV;
    int status = 0;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttr;
    uint8_t *streamMetaData = nullptr;
    uint8_t *deviceMetaData = nullptr;
    uint8_t *streamDeviceMetaData = nullptr;
    std::ostringstream cntrlName;
    std::ostringstream pcmMdName;
    std::ostringstream aifMdName;
    std::ostringstream disconnectCtrlName;
    std::ostringstream aifMfCtrlName;
    std::ostringstream connectCtrlName;
    std::ostringstream compressMdName;
    struct mixer_ctl *ctl = NULL;
    struct mixer_ctl *feCtrl= NULL;
    struct mixer_ctl *feMdCtrl = NULL;
    struct mixer_ctl *aifMdCtrl = NULL;
    struct mixer_ctl *connectCtrl = NULL;
    struct mixer_ctl *aifMfCtrl = NULL;
    struct mixer_ctl *disconnectCtrl;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    long aif_media_config[2][3];
    struct mixer *mixerHandle;
    int ioMode;
    uint32_t streamMetaDataSize = 0, deviceMetaDataSize = 0, streamDeviceMetaDataSize = 0;
    uint32_t streamPropId[] = {0x08000010, 1, 0x1}; /** gsl_subgraph_platform_driver_props.xml */
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */

    PayloadBuilder* builder = new PayloadBuilder();


    status = streamHandle->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        goto exit;
    }

    status = streamHandle->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        goto exit;
    }


    // get streamKV
    if ((status = builder->populateStreamKV(streamHandle, streamKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get stream KV failed %d", status);
        goto exit;
    }
    // get deviceKV
    if ((status = builder->populateDeviceKV(streamHandle, deviceKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        goto exit;
    }
    // get streamdeviceKV
    status = builder->populateStreamDeviceKV(streamHandle, streamDeviceKV);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }
    status = builder->populateStreamCkv(streamHandle, streamCKV, 0, (struct qal_volume_data **)nullptr);
    if (status) {
        QAL_ERR(LOG_TAG, "get stream ckv failed %d", status);
        goto exit;
    }
    // get audio mixer
    status = rmHandle->getAudioMixer(&mixerHandle);
    status = getAgmMetaData(streamKV, streamCKV, (struct prop_data *)streamPropId,
            streamMetaDataSize, &streamMetaData);
    //streamMetaData = getAgmMetaData(streamKV, streamMetaDataSize);
    if (status != 0 || !streamMetaData) {
        QAL_ERR(LOG_TAG, "get stream meta data failed %d", status);
        status = -EINVAL;
        goto exit;
    }
    status = getAgmMetaData(deviceKV, emptyKV, (struct prop_data *)devicePropId,
            deviceMetaDataSize, &deviceMetaData);
    if (status != 0 || !deviceMetaData) {
        QAL_ERR(LOG_TAG, "get device meta data failed %d", status);
        status = -EINVAL;
        goto free_streammd;
    }
    if (streamDeviceKV.size()) {
        status = getAgmMetaData(streamDeviceKV, emptyKV, (struct prop_data *)streamDevicePropId,
                streamDeviceMetaDataSize, &streamDeviceMetaData);
        if (status != 0 || !streamDeviceMetaData) {
            QAL_ERR(LOG_TAG, "get stream device meta data failed %d", status);
            status = -EINVAL;
            goto free_devicemd;
        }
    }

    //TODO: print front end and backends that we got
    for(int i=0; i<BackEnds.size(); i++) {
        switch (sAttr.type) {
            case QAL_STREAM_COMPRESSED:
                /** create compress mixer controls - stream, device and stream-device */
                cntrlName << COMPRESS_SND_DEV_NAME_PREFIX << DevIds.at(0) << " control";
                compressMdName << COMPRESS_SND_DEV_NAME_PREFIX << DevIds.at(0) << " metadata";
                aifMdName << (BackEnds.at(i)).data() << " metadata";
                connectCtrlName << COMPRESS_SND_DEV_NAME_PREFIX << DevIds.at(0) << " connect";
                aifMfCtrlName << (BackEnds.at(i)).data() << " rate ch fmt";
                disconnectCtrlName << COMPRESS_SND_DEV_NAME_PREFIX << DevIds.at(0) << " disconnect";
                break;
            default:
                /** create compress mixer controls - stream, device and stream-device */
                cntrlName << PCM_SND_DEV_NAME_PREFIX << DevIds.at(0) << " control";
                compressMdName << PCM_SND_DEV_NAME_PREFIX << DevIds.at(0) << " metadata";
                aifMdName << (BackEnds.at(i)).data() << " metadata";
                connectCtrlName << PCM_SND_DEV_NAME_PREFIX << DevIds.at(0) << " connect";
                aifMfCtrlName << (BackEnds.at(i)).data() << " rate ch fmt";
                disconnectCtrlName << PCM_SND_DEV_NAME_PREFIX << DevIds.at(0) << " disconnect";
                break;
        }


            /**
            *  Set Stream, Device and SD mixer controls
            *  cntrlName: COMPRESS0 control
            *  pcmMdName: COMPRESS0 metadata
            *  aifMdName: SLIMBUS_0_RX metadata
            */
            // ctl name: 'COMPRESS<DEVID> control' 'ZERO'
            feCtrl = mixer_get_ctl_by_name(mixerHandle, cntrlName.str().data());
            QAL_DBG(LOG_TAG,"mixer control %s", cntrlName.str().data());
            if (!feCtrl) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s", cntrlName.str().data());
                status = -EINVAL;
                goto free_streamdevicemd;
            }
            mixer_ctl_set_enum_by_string(feCtrl, "ZERO");

            // ctl name: 'COMPRESS<DEVID> metadata' 'streamMetaData'
            feMdCtrl = mixer_get_ctl_by_name(mixerHandle, compressMdName.str().data());
            QAL_DBG(LOG_TAG,"mixer control %s", compressMdName.str().data());
            if (!feMdCtrl) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s", compressMdName.str().data());
                status = -EINVAL;
                goto free_streamdevicemd;
            }
            mixer_ctl_set_array(feMdCtrl, (void *)streamMetaData, streamMetaDataSize);

            // ctl name: 'SLIMBUS_0_RX metadata' 'deviceMetaData'
            aifMdCtrl = mixer_get_ctl_by_name(mixerHandle, aifMdName.str().data());
            QAL_DBG(LOG_TAG,"mixer control %s", aifMdName.str().data());
            if (!aifMdCtrl) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMdName.str().data());
                status = -EINVAL;
                goto free_streamdevicemd;
            }
            mixer_ctl_set_array(aifMdCtrl, (void *)deviceMetaData, deviceMetaDataSize);

            // ctl name: 'COMPRESS<FEID> control' 'SLIMBUS_0_RX'
            mixer_ctl_set_enum_by_string(feCtrl, (BackEnds.at(i)).data());
            QAL_DBG(LOG_TAG,"device %s", (BackEnds.at(i)).data());
            // ctl name: 'COMPRESS<FEID> metadata' 'streamDeviceMetaData'
            if (streamDeviceMetaDataSize)
                mixer_ctl_set_array(feMdCtrl, (void *)streamDeviceMetaData, streamDeviceMetaDataSize);

            //ctl name: 'COMPRESS<DevID> connect' 'SLIMBUS_0_RX'
            connectCtrl = mixer_get_ctl_by_name(mixerHandle, connectCtrlName.str().data());
            if (!connectCtrl) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
                status = -EINVAL;
                goto free_streamdevicemd;
            }
            QAL_DBG(LOG_TAG,"mixer control %s", connectCtrlName.str().data());
            QAL_DBG(LOG_TAG,"device %s", (BackEnds.at(i)).data());
            mixer_ctl_set_enum_by_string(connectCtrl, (BackEnds.at(i)).data());

            /**TODO: add only SSSD support for now */
            associatedDevices[i]->getDeviceAtrributes(&dAttr);
            // ctl name: 'SLIMBUS_0_RX rate ch fmt'
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

            disconnectCtrl = mixer_get_ctl_by_name(mixerHandle, disconnectCtrlName.str().data());
            if (!disconnectCtrl) {
                QAL_ERR(LOG_TAG, "invalid mixer control: %s", disconnectCtrlName.str().data());
                status = -EINVAL;
                goto free_streamdevicemd;
            }
    }

free_streamdevicemd:
    free(streamDeviceMetaData);
free_devicemd:
    free(deviceMetaData);
free_streammd:
    free(streamMetaData);

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
    char *stream = "PCM";
    char *control = "getTaggedInfo";
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
    char *stream = "PCM";
    char *control = "setParam";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;

    if (isCompress)
        stream = "COMPRESS";


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
    char *stream = "PCM";
    char *control = "control";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;

    if (isCompress)
        stream = "COMPRESS";

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if(mixer_str == NULL) {
        ret = -ENOMEM;
        QAL_ERR(LOG_TAG,"%s:calloc failed",_func_);
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
    char *stream = "PCM";
    char *control = "event";
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
    char *stream = "PCM";
    char *control = "echoReference";
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
        QAL_ERR(LOG_TAG,"%s:calloc failed",_func_);
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
    const std::vector<int> &RxDevIds, const std::vector<int> &TxDevIds, const std::vector<std::string> &BackEnds)
{
    std::vector <std::pair<int, int>> streamKV;
    std::vector <std::pair<int, int>> streamCKV;
    std::vector <std::pair<int, int>> streamDeviceKV;
    std::vector <std::pair<int, int>> deviceRxKV;
    std::vector <std::pair<int, int>> deviceTxKV;
    // Using as empty key vector pairs
    std::vector <std::pair<int, int>> emptyKV;
    int status = 0;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttrRx, dAttrTx;
    uint8_t *streamMetaData = nullptr;
    uint8_t *deviceRxMetaData = nullptr;
    uint8_t *deviceTxMetaData = nullptr;
    uint8_t *streamDeviceMetaData = nullptr;
    std::ostringstream cntrlName;
    std::ostringstream pcmMdName;
    std::ostringstream aifMdRxName;
    std::ostringstream aifMdTxName;
    std::ostringstream disconnectCtrlName;
    std::ostringstream aifMfRxCtrlName;
    std::ostringstream aifMfTxCtrlName;
    std::ostringstream connectRxCtrlName;
    std::ostringstream connectTxCtrlName;
    std::ostringstream compressMdName;
    std::ostringstream loopbackCtrlName;
    std::ostringstream loopbackRxCtrlName;
    struct mixer_ctl *ctl;
    struct mixer_ctl *feCtrl;
    struct mixer_ctl *feMdCtrl;
    struct mixer_ctl *aifMdRxCtrl;
    struct mixer_ctl *aifMdTxCtrl;
    struct mixer_ctl *aifMfRxCtrl;
    struct mixer_ctl *connectRxCtrl;
    struct mixer_ctl *connectTxCtrl;
    struct mixer_ctl *aifMfTxCtrl;
    struct mixer_ctl *disconnectCtrl;
    struct mixer_ctl *loopbackCtrl;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    long aif_media_config_rx[3];
    long aif_media_config_tx[3];
    uint32_t streamMetaDataSize = 0, deviceRxMetaDataSize = 0, deviceTxMetaDataSize = 0, streamDeviceMetaDataSize = 0;
    struct mixer *mixerHandle;
    uint32_t streamPropId[] = {0x08000010, 1, 0x1}; /** gsl_subgraph_platform_driver_props.xml */
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */

    PayloadBuilder* builder = new PayloadBuilder();


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

    status = rmHandle->getAudioMixer(&mixerHandle);
    // get streamKV
    if ((status = builder->populateStreamKV(streamHandle, streamKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get stream KV failed %d", status);
        goto exit;
    }
    // get deviceKV
    if ((status = builder->populateDeviceRxKV(streamHandle, deviceRxKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        goto exit;
    }
    if ((status = builder->populateDeviceTxKV(streamHandle, deviceTxKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        goto exit;
    }
    // get streamdeviceKV
    status = builder->populateStreamDeviceKV(streamHandle, streamDeviceKV);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }
    status = builder->populateStreamCkv(streamHandle, streamCKV, 0, (struct qal_volume_data **)nullptr);
    if (status) {
        QAL_ERR(LOG_TAG, "%s: get stream ckv failed %d", status);
        goto exit;
    }
    // get audio mixer
    status = getAgmMetaData(streamKV, streamCKV, (struct prop_data *)streamPropId,
           streamMetaDataSize, &streamMetaData);
    //streamMetaData = getAgmMetaData(streamKV, streamMetaDataSize);
    if (!streamMetaData) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: get stream meta data failed %d", _func_, status);
        goto exit;
    }
    status = getAgmMetaData(deviceRxKV, emptyKV, (struct prop_data *)devicePropId,
           deviceRxMetaDataSize, &deviceRxMetaData);
    if (!deviceRxMetaData) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: get device meta data failed %d", _func_, status);
        goto free_streammd;
    }
    status = getAgmMetaData(deviceTxKV, emptyKV, (struct prop_data *)devicePropId,
            deviceTxMetaDataSize, &deviceTxMetaData);
    if (!deviceTxMetaData) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: get device meta data failed %d", _func_, status);
        goto free_streammd;
    }
    if (streamDeviceKV.size()) {
        status = getAgmMetaData(streamDeviceKV, emptyKV, (struct prop_data *)streamDevicePropId,
                streamDeviceMetaDataSize, &streamDeviceMetaData);
        if (!streamDeviceMetaData) {
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "%s: get stream device meta data failed %d", _func_, status);
            goto free_devicemd;
        }
    }

    switch (sAttr.type) {
        case QAL_STREAM_COMPRESSED:
            break;
         default:
             /** create compress mixer controls - stream, device and stream-device */
            cntrlName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0) << " control";
            compressMdName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0) << " metadata";
            aifMdRxName << (BackEnds.at(0)).data() << " metadata";
            aifMdTxName << (BackEnds.at(1)).data() << " metadata";
            connectRxCtrlName << PCM_SND_DEV_NAME_PREFIX << RxDevIds.at(0) << " connect";
            connectTxCtrlName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0) << " connect";
            aifMfRxCtrlName << (BackEnds.at(0)).data() << " rate ch fmt";
            aifMfTxCtrlName << (BackEnds.at(1)).data() << " rate ch fmt";
            disconnectCtrlName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0) << " disconnect";
            loopbackCtrlName << PCM_SND_DEV_NAME_PREFIX << TxDevIds.at(0) << " loopback";
            loopbackRxCtrlName << PCM_SND_DEV_NAME_PREFIX << RxDevIds.at(0);
            break;
    }


        /**
         *  Set Stream, Device and SD mixer controls
         *  cntrlName: COMPRESS0 control
         *  pcmMdName: COMPRESS0 metadata
         *  aifMdName: SLIMBUS_0_RX metadata
         */
        // ctl name: 'COMPRESS<DEVID> control' 'ZERO'
        feCtrl = mixer_get_ctl_by_name(mixerHandle, cntrlName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", cntrlName.str().data());
        if (!feCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", cntrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_enum_by_string(feCtrl, "ZERO");

        // ctl name: 'COMPRESS<DEVID> metadata' 'streamMetaData'
        feMdCtrl = mixer_get_ctl_by_name(mixerHandle, compressMdName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", compressMdName.str().data());
        if (!feMdCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", compressMdName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(feMdCtrl, (void *)streamMetaData, streamMetaDataSize);

        // ctl name: 'SLIMBUS_0_RX metadata' 'deviceMetaData'
        aifMdRxCtrl = mixer_get_ctl_by_name(mixerHandle, aifMdRxName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", aifMdRxName.str().data());
        if (!aifMdRxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMdRxName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(aifMdRxCtrl, (void *)deviceRxMetaData, deviceRxMetaDataSize);
        aifMdTxCtrl = mixer_get_ctl_by_name(mixerHandle, aifMdTxName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", aifMdTxName.str().data());
        if (!aifMdTxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMdTxName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(aifMdTxCtrl, (void *)deviceTxMetaData, deviceTxMetaDataSize);

        // ctl name: 'COMPRESS<FEID> control' 'SLIMBUS_0_RX'
        mixer_ctl_set_enum_by_string(feCtrl, (BackEnds.at(1)).data());
        QAL_DBG(LOG_TAG,"device %s", (BackEnds.at(1)).data());
        // ctl name: 'COMPRESS<FEID> metadata' 'streamDeviceMetaData'
        if (streamDeviceMetaDataSize)
            mixer_ctl_set_array(feMdCtrl, (void *)streamDeviceMetaData, streamDeviceMetaDataSize);

        //ctl name: 'COMPRESS<DevID> connect' 'SLIMBUS_0_RX'
        connectRxCtrl = mixer_get_ctl_by_name(mixerHandle, connectRxCtrlName.str().data());
        if (!connectRxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectRxCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        QAL_DBG(LOG_TAG,"mixer control %s", connectRxCtrlName.str().data());
        QAL_DBG(LOG_TAG,"device %s", (BackEnds.at(0)).data());
        mixer_ctl_set_enum_by_string(connectRxCtrl, (BackEnds.at(0)).data());

        connectTxCtrl = mixer_get_ctl_by_name(mixerHandle, connectTxCtrlName.str().data());
        if (!connectTxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectTxCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        QAL_DBG(LOG_TAG,"mixer control %s", connectTxCtrlName.str().data());
        QAL_DBG(LOG_TAG,"device %s", (BackEnds.at(1)).data());
        mixer_ctl_set_enum_by_string(connectTxCtrl, (BackEnds.at(1)).data());

        /**TODO: add only SSSD support for now */
        associatedDevices[0]->getDeviceAtrributes(&dAttrRx);
        // ctl name: 'SLIMBUS_0_RX rate ch fmt'
        aif_media_config_rx[0] = dAttrRx.config.sample_rate;
        aif_media_config_rx[1] = dAttrRx.config.ch_info->channels;
        aif_media_config_rx[2] = bitsToAlsaFormat(dAttrRx.config.bit_width);

        associatedDevices[1]->getDeviceAtrributes(&dAttrTx);
        // ctl name: 'SLIMBUS_0_RX rate ch fmt'
        aif_media_config_tx[0] = dAttrTx.config.sample_rate;
        aif_media_config_tx[1] = dAttrTx.config.ch_info->channels;
        aif_media_config_tx[2] = bitsToAlsaFormat(dAttrTx.config.bit_width);

        aifMfRxCtrl = mixer_get_ctl_by_name(mixerHandle, aifMfRxCtrlName.str().data());
        QAL_ERR(LOG_TAG,"mixer control %s", aifMfRxCtrlName.str().data());
        if (!aifMfRxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMfRxCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(aifMfRxCtrl, &aif_media_config_rx,
                sizeof(aif_media_config_rx)/sizeof(aif_media_config_rx[0]));
        aifMfTxCtrl = mixer_get_ctl_by_name(mixerHandle, aifMfTxCtrlName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", aifMfTxCtrlName.str().data());
        if (!aifMfTxCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMfTxCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(aifMfTxCtrl, &aif_media_config_tx,
                sizeof(aif_media_config_tx)/sizeof(aif_media_config_tx[0]));

        loopbackCtrl = mixer_get_ctl_by_name(mixerHandle, loopbackCtrlName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", loopbackCtrlName.str().data());
        if (!loopbackCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", loopbackCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_enum_by_string(loopbackCtrl, loopbackRxCtrlName.str().data());
        disconnectCtrl = mixer_get_ctl_by_name(mixerHandle, disconnectCtrlName.str().data());
        if (!disconnectCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", disconnectCtrlName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }

free_streamdevicemd:
    free(streamDeviceMetaData);
free_devicemd:
    free(deviceRxMetaData);
    free(deviceTxMetaData);
free_streammd:
    free(streamMetaData);
exit:
   return status;
}

int SessionAlsaUtils::disconnectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<ResourceManager> rmHandle, struct qal_device &dAttr,
        const std::vector<int> &pcmDevIds,
        const std::vector<std::string> &aifBackEndsToDisconnect)
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
    mixer_ctl_set_enum_by_string(disconnectCtrl, (aifBackEndsToDisconnect.at(0)).data());

    return status;
}

int SessionAlsaUtils::connectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<ResourceManager> rmHandle, struct qal_device &dAttr,
        const std::vector<int> &pcmDevIds, const std::vector<std::string> &aifBackEndsToConnect)
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
    uint32_t deviceMetaDataSize = 0, streamDeviceMetaDataSize = 0;
    uint8_t *deviceMetaData = nullptr;
    uint8_t *streamDeviceMetaData = nullptr;
    struct mixer_ctl *feCtrl = nullptr;
    struct mixer_ctl *feMdCtrl = nullptr;
    struct mixer_ctl *aifMdCtrl = nullptr;
    struct mixer_ctl *aifMfCtrl = nullptr;
    long aif_media_config[3];
    PayloadBuilder* builder = new PayloadBuilder();
    struct mixer *mixerHandle = nullptr;
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t streamDevicePropId[] = {0x08000010, 1, 0x3}; /** gsl_subgraph_platform_driver_props.xml */


    status = rmHandle->getAudioMixer(&mixerHandle);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }

    status = builder->populateStreamDeviceKV(streamHandle, streamDeviceKV);
    if (status) {
        QAL_VERBOSE(LOG_TAG, "get stream device KV failed %d", status);
        status = 0; /**< ignore stream device KV failures */
    }
    if ((status = builder->populateDeviceKV(streamHandle, deviceKV)) != 0) {
        QAL_ERR(LOG_TAG, "%s: get device KV failed %d", status);
        return status;
    }
    status = SessionAlsaUtils::getAgmMetaData(deviceKV, emptyKV, (struct prop_data *)devicePropId,
            deviceMetaDataSize, &deviceMetaData);
    if (status != 0 || !deviceMetaData) {
        QAL_ERR(LOG_TAG, "get device meta data failed %d", status);
        return status;
    }

    if (streamDeviceKV.size()) {
        status = SessionAlsaUtils::getAgmMetaData(streamDeviceKV, emptyKV,
                (struct prop_data *)streamDevicePropId,
                streamDeviceMetaDataSize, &streamDeviceMetaData);
        if (status != 0 || !streamDeviceMetaData) {
            QAL_ERR(LOG_TAG, "get stream device meta data failed %d", status);
            goto free_devicemd;
        }
    }

    switch (streamType) {
        case QAL_STREAM_COMPRESSED:
            cntrlName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " control";
            aifMdName << (aifBackEndsToConnect.at(0)).data() << " metadata";
            connectCtrlName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " connect";
            aifMfCtrlName << (aifBackEndsToConnect.at(0)).data() << " rate ch fmt";
            feMdName << COMPRESS_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " metadata";
            break;
        default:
            cntrlName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " control";
            aifMdName << (aifBackEndsToConnect.at(0)).data() << " metadata";
            connectCtrlName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " connect";
            aifMfCtrlName << (aifBackEndsToConnect.at(0)).data() << " rate ch fmt";
            feMdName << PCM_SND_DEV_NAME_PREFIX << pcmDevIds.at(0) << " metadata";
            break;
    }

    status = rmHandle->getAudioMixer(&mixerHandle);

    aifMdCtrl = mixer_get_ctl_by_name(mixerHandle, aifMdName.str().data());
    QAL_DBG(LOG_TAG,"mixer control %s", aifMdName.str().data());
    if (!aifMdCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", aifMdName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_array(aifMdCtrl, (void *)deviceMetaData, deviceMetaDataSize);

    feCtrl = mixer_get_ctl_by_name(mixerHandle, cntrlName.str().data());
    QAL_DBG(LOG_TAG,"mixer control %s", cntrlName.str().data());
    if (!feCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", cntrlName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_enum_by_string(feCtrl, (aifBackEndsToConnect.at(0)).data());

    if (streamDeviceMetaDataSize) {
        feMdCtrl = mixer_get_ctl_by_name(mixerHandle, feMdName.str().data());
        QAL_DBG(LOG_TAG,"mixer control %s", feMdName.str().data());
        if (!feMdCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", feMdName.str().data());
            status = -EINVAL;
            goto free_streamdevicemd;
        }
        mixer_ctl_set_array(feMdCtrl, (void *)streamDeviceMetaData, streamDeviceMetaDataSize);
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
    connectCtrl = mixer_get_ctl_by_name(mixerHandle, connectCtrlName.str().data());
    if (!connectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
        status = -EINVAL;
        goto free_streamdevicemd;
    }
    mixer_ctl_set_enum_by_string(connectCtrl, (aifBackEndsToConnect.at(0)).data());

free_streamdevicemd:
    free(streamDeviceMetaData);
free_devicemd:
    free(deviceMetaData);

    delete builder;
    return status;
}

