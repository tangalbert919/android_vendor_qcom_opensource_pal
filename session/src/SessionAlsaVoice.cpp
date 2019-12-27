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


#define LOG_TAG "SessionAlsaVoice"

#include "SessionAlsaVoice.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include "apm_api.h"
#include <sstream>
#include <string>
#include <agm_api.h>

#define QAL_PADDING_8BYTE_ALIGN(x)  ((((x) + 7) & 7) ^ 7)
#define MAX_VOL_INDEX 5
#define MIN_VOL_INDEX 0
#define percent_to_index(val, min, max) \
            ((val) * ((max) - (min)) * 0.01 + (min) + .5)

SessionAlsaVoice::SessionAlsaVoice(std::shared_ptr<ResourceManager> Rm)
{
   rm = Rm;
   builder = new PayloadBuilder();
   customPayload = NULL;
   customPayloadSize = 0;
}

SessionAlsaVoice::~SessionAlsaVoice()
{
   delete builder;

}


int SessionAlsaVoice::prepare(Stream * s)
{
   return 0;
}

int SessionAlsaVoice::open(Stream * s)
{
    int status = -EINVAL;
    struct qal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }

    if (sAttr.direction != (QAL_AUDIO_INPUT|QAL_AUDIO_OUTPUT)) {
        QAL_ERR(LOG_TAG,"%s: Voice session dir must be input and output  \n"
        , __func__);
        return status;
    }

    pcmDevRxIds = rm->allocateFrontEndIds(sAttr, RXDIR);
    pcmDevTxIds = rm->allocateFrontEndIds(sAttr, TXDIR);

    vsid = sAttr.info.voice_call_info.VSID;

    rm->getBackEndNames(associatedDevices, rxAifBackEnds, txAifBackEnds);

    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error");
        return status;
    }

    status = SessionAlsaUtils::open(s, rm, pcmDevRxIds, pcmDevTxIds,
                                    rxAifBackEnds, txAifBackEnds);

    if (status) {
        QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
        rm->freeFrontEndIds(pcmDevRxIds, sAttr, RXDIR);
        rm->freeFrontEndIds(pcmDevTxIds, sAttr, TXDIR);
    }

    /* not need right now */
    /*status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevRxIds.at(0),
                                                   aifBackEnds[0].data(),
                                                   false, STREAM_SPR,
                                                   &spr_miid);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d",
                STREAM_SPR, status);
        return status;
    }*/
exit:
    return status;
}

int SessionAlsaVoice::start(Stream * s)
{
    struct pcm_config config;
    struct qal_stream_attributes sAttr;
    int32_t status = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
    memset(&config, 0, sizeof(config));

    config.rate = sAttr.out_media_config.sample_rate;
    if (sAttr.out_media_config.bit_width == 32)
        config.format = PCM_FORMAT_S32_LE;
    else if (sAttr.out_media_config.bit_width == 24)
        config.format = PCM_FORMAT_S24_3LE;
    else if (sAttr.out_media_config.bit_width == 16)
        config.format = PCM_FORMAT_S16_LE;
    config.channels = sAttr.out_media_config.ch_info->channels;
    config.period_size = out_buf_size;
    config.period_count = out_buf_count;
    config.start_threshold = 0;
    config.stop_threshold = 0;
    config.silence_threshold = 0;

    pcmRx = pcm_open(rm->getSndCard(), pcmDevRxIds.at(0), PCM_OUT, &config);
    if (!pcmRx) {
        QAL_ERR(LOG_TAG, "pcm-rx open failed");
        return -EINVAL;
    }

    if (!pcm_is_ready(pcmRx)) {
        QAL_ERR(LOG_TAG, "pcm-rx open not ready");
        return -EINVAL;
    }

    config.rate = sAttr.in_media_config.sample_rate;
    if (sAttr.in_media_config.bit_width == 32)
        config.format = PCM_FORMAT_S32_LE;
    else if (sAttr.in_media_config.bit_width == 24)
        config.format = PCM_FORMAT_S24_3LE;
    else if (sAttr.in_media_config.bit_width == 16)
        config.format = PCM_FORMAT_S16_LE;
    config.channels = sAttr.in_media_config.ch_info->channels;
    config.period_size = in_buf_size;
    config.period_count = in_buf_count;

    pcmTx = pcm_open(rm->getSndCard(), pcmDevTxIds.at(0), PCM_IN, &config);
    if (!pcmTx) {
        QAL_ERR(LOG_TAG, "pcm-tx open failed");
        return -EINVAL;
    }

    if (!pcm_is_ready(pcmTx)) {
        QAL_ERR(LOG_TAG, "pcm-tx open not ready");
        return -EINVAL;
    }

    SessionAlsaVoice::setConfig(s, MODULE, VSID, RXDIR);
    /*if no volume is set set a default volume*/
    struct qal_volume_data *volume;
    if ((s->getVolumeData(volume))) {
        QAL_INFO(LOG_TAG, "no volume set, setting default vol to %f",
                 default_volume);
        volume = (struct qal_volume_data *)malloc(sizeof(uint32_t) +
                                                  (sizeof(struct qal_channel_vol_kv)));
        if (!volume) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "volume malloc failed %s", strerror(errno));
            goto exit;
        }
        volume->no_of_volpair = 1;
        volume->volume_pair[0].channel_mask = 1;
        volume->volume_pair[0].vol = default_volume;
        s->setVolume(volume);
        free(volume);
    };

    status = pcm_start(pcmRx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_start rx failed %d", status);
        goto exit;
    }

    status = pcm_start(pcmTx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_start tx failed %d", status);
        pcm_close(pcmRx);
        goto exit;
    }

exit:
    free(volume);
    return status;
}

int SessionAlsaVoice::stop(Stream * s)
{
    int status = 0;

    status = pcm_stop(pcmRx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_stop - rx failed %d", status);
    }
    status = pcm_stop(pcmTx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_stop - tx failed %d", status);
    }

    return status;
}

int SessionAlsaVoice::close(Stream * s)
{
    int status = 0;
    struct qal_stream_attributes sAttr;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    status = pcm_close(pcmRx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_close - rx failed %d", status);
    }
    rm->freeFrontEndIds(pcmDevRxIds, sAttr, 0);
    status = pcm_close(pcmTx);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_close - tx failed %d", status);
    }
    rm->freeFrontEndIds(pcmDevTxIds, sAttr, 1);
    pcmRx = NULL;
    pcmTx = NULL;


    return status;
}
/*
int SessionAlsaVoice::setParameters(Stream *s, int tagId, uint32_t param_id, void *payload)
{
    int status = 0;
    int device = pcmDevRxIds.at(0);
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    QAL_DBG(LOG_TAG, "Enter.");

    CntrlName << stream << device << " " << control;
    ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        status = -ENOENT;
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter.");
    switch (param_id) {
        case QAL_PARAM_ID_SET_VOICE_VOLUME:
            builder->payloadVoiceCalKeys(&paramData, &paramSize, vsid,
                                         VCPM_MODULE_INSTANCE_ID);
            device = pcmDevRxIds.at(0);
            break;

        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Unsupported param id %u status %d", param_id, status);
            goto exit;
    }

    if (!paramData) {
        status = -ENOMEM;
        QAL_ERR(LOG_TAG, "failed to get payload status %d", status);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "%x - payload and %d size", paramData , paramSize);

    status = SessionAlsaUtils::setMixerParameter(mixer,device,false,
                                                 paramData, paramSize);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to set voice params status = %d", status);
    }

    QAL_DBG(LOG_TAG, "Exit. status %d", status);
free_payload :
    free(paramData);
exit:
    return status;
}
*/

int SessionAlsaVoice::setConfig(Stream * s, configType type, int tag)
{
    return 0;
}

int SessionAlsaVoice::setConfig(Stream * s, configType type, int tag, int dir)
{
    int status = 0;
    int device = pcmDevRxIds.at(0);
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    struct mixer_ctl *ctl;

    switch (static_cast<uint32_t>(tag)) {
        case TAG_STREAM_VOLUME:
            device = pcmDevRxIds.at(0);
            status = payloadCalKeysVolume(s, &paramData, &paramSize);
            status = SessionAlsaVoice::setVoiceMixerParameter(s, mixer,
                                                              paramData,
                                                              paramSize,
                                                              dir);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
            }
            if (!paramData) {
                status = -ENOMEM;
                QAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }
            break;
        case VSID:
            device = pcmDevRxIds.at(0);
            status = payloadSetVSID(&paramData, &paramSize);
            status = SessionAlsaVoice::setVoiceMixerParameter(s, mixer,
                                                              paramData,
                                                              paramSize,
                                                              dir);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to set voice params status = %d",
                        status);
                break;
            }

            if (!paramData) {
                status = -ENOMEM;
                QAL_ERR(LOG_TAG, "failed to get payload status %d", status);
                goto exit;
            }

            break;
        case MUTE_TAG:
        case UNMUTE_TAG:
            device = pcmDevTxIds.at(0);
            status = payloadTaged(s, type, tag, device, TXDIR);
            break;

        default:
            QAL_ERR(LOG_TAG,"%s: Failed unsupported tag type %d \n", __func__, static_cast<uint32_t>(tag));
            status = -EINVAL;
            break;
    }
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to set config data\n", __func__);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG, "%x - payload and %d size", paramData , paramSize);

exit:
if (paramData) {
    free(paramData);
}
    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionAlsaVoice::payloadTaged(Stream * s, configType type, int tag,
                                   int device, int dir){
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig;
    const char *setParamTagControl = "setParamTag";
    struct mixer_ctl *ctl;
    std::ostringstream tagCntrlName;
    int tkv_size = 0;
    const char *stream = SessionAlsaVoice::getMixerVoiceStream(s, dir);
    switch (type) {
        case MODULE:
            tkv.clear();
            status = builder->populateTagKeyVector(s, tkv, tag, &tagsent);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the tag configuration\n", __func__);
                goto exit;
            }

            if (tkv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }

            tagConfig = (struct agm_tag_config*)malloc (sizeof(struct agm_tag_config) +
                            (tkv.size() * sizeof(agm_key_value)));

            if(!tagConfig) {
                status = -EINVAL;
                goto exit;
            }

            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName<<stream<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                return -ENOENT;
            }

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            tkv.clear();
            if (tagConfig) {
                free(tagConfig);
            }
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
    }

exit:
    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionAlsaVoice::payloadSetVSID(uint8_t **payload, size_t *size){
    int status = 0;
    apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *phrase_pl;
    vcpm_param_vsid_payload_t vsid_payload;

    payloadSize = sizeof(struct apm_module_param_data_t)+
                  sizeof(vcpm_param_vsid_payload_t);
    padBytes = QAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return -EINVAL;
    }
    header = (apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = VCPM_MODULE_INSTANCE_ID;
    header->param_id = VCPM_PARAM_ID_VSID;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    vsid_payload.vsid = vsid;
    phrase_pl = (uint8_t*)payloadInfo + sizeof(apm_module_param_data_t);
    casa_osal_memcpy(phrase_pl,  sizeof(vcpm_param_vsid_payload_t),
                     &vsid_payload,  sizeof(vcpm_param_vsid_payload_t));

    *size = payloadSize + padBytes;
    *payload = payloadInfo;


    return status;
}

int SessionAlsaVoice::payloadCalKeysVolume(Stream * s, uint8_t **payload, size_t *size)
{
    int status = 0;
    apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *phrase_pl;
    vcpm_param_cal_keys_payload_t cal_keys;
    vcpm_ckv_pair_t cal_key_pair;
    float volume = 0.0;
    int vol;
    struct qal_volume_data *voldata = NULL;

    voldata = (struct qal_volume_data *)calloc(1, (sizeof(uint32_t) +
                      (sizeof(struct qal_channel_vol_kv) * (0xFFFF))));
    if (!voldata) {
        status = -ENOMEM;
        goto exit;
    }
    status = s->getVolumeData(voldata);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getVolumeData Failed \n", __func__);
        goto exit;
    }

    QAL_VERBOSE(LOG_TAG,"%s: volume sent:%f \n",__func__, (voldata->volume_pair[0].vol));
    volume = (voldata->volume_pair[0].vol);

    payloadSize = sizeof(apm_module_param_data_t) +
                  sizeof(vcpm_param_cal_keys_payload_t) +
                  sizeof(vcpm_ckv_pair_t);
    padBytes = QAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return -EINVAL;
    }
    header = (apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = VCPM_MODULE_INSTANCE_ID;
    header->param_id = VCPM_PARAM_ID_CAL_KEYS;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    cal_keys.vsid = vsid;
    cal_keys.num_ckv_pairs = 1;
    cal_key_pair.cal_key_id = VCPM_CAL_KEY_ID_VOLUME_LEVEL;
    if (volume < 0.0) {
            volume = 0.0;
    } else if (volume > 1.0) {
        volume = 1.0;
    }

    vol = lrint(volume * 100.0);

    // Voice volume levels from android are mapped to driver volume levels as follows.
    // 0 -> 5, 20 -> 4, 40 ->3, 60 -> 2, 80 -> 1, 100 -> 0
    // So adjust the volume to get the correct volume index in driver
    vol = 100 - vol;
    cal_key_pair.value = percent_to_index(vol, MIN_VOL_INDEX, MAX_VOL_INDEX);

    phrase_pl = (uint8_t*)payloadInfo + sizeof(apm_module_param_data_t);
    casa_osal_memcpy(phrase_pl, sizeof(vcpm_param_cal_keys_payload_t),
                     &cal_keys, sizeof(vcpm_param_cal_keys_payload_t));

    phrase_pl += sizeof(vcpm_param_cal_keys_payload_t);
    casa_osal_memcpy(phrase_pl, sizeof(vcpm_ckv_pair_t),
                     &cal_key_pair, sizeof(vcpm_ckv_pair_t));


    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    QAL_VERBOSE(LOG_TAG, "payload %u size %d", *payload, *size);

exit:
    if (voldata) {
        free(voldata);
    }
    return status;
}

int SessionAlsaVoice::disconnectSessionDevice(Stream *streamHandle,
                                              qal_stream_type_t streamType,
                                              std::shared_ptr<Device> deviceToDisconnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::vector<std::string> aifBackEndsToDisconnect;
    struct qal_device dAttr;
    int status = 0;

    deviceList.push_back(deviceToDisconnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds,txAifBackEnds);

    deviceToDisconnect->getDeviceAttributes(&dAttr);

    if (rxAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::disconnectSessionDevice(streamHandle,
                                                            streamType, rm,
                                                            dAttr, pcmDevRxIds,
                                                            rxAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: disconnectSessionDevice on RX Failed \n", __func__);
            return status;
        }
    } else if (txAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::disconnectSessionDevice(streamHandle,
                                                            streamType, rm,
                                                            dAttr, pcmDevTxIds,
                                                            txAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: disconnectSessionDevice on TX Failed \n", __func__);
        }
    }

    return status;
}

int SessionAlsaVoice::setupSessionDevice(Stream* streamHandle,
                                 qal_stream_type_t streamType,
                                 std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::vector<std::string> aifBackEndsToConnect;
    struct qal_device dAttr;
    int status = 0;

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds, txAifBackEnds);
    deviceToConnect->getDeviceAttributes(&dAttr);

    if (rxAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::setupSessionDevice(streamHandle, streamType,
                                                       rm, dAttr, pcmDevRxIds,
                                                       rxAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: setupSessionDevice on RX Failed \n", __func__);
            return status;
        }
    } else if (txAifBackEnds.size() > 0) {

        status =  SessionAlsaUtils::setupSessionDevice(streamHandle, streamType,
                                                       rm, dAttr, pcmDevTxIds,
                                                       txAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: setupSessionDevice on TX Failed \n", __func__);
        }
    }
    return status;
}

int SessionAlsaVoice::connectSessionDevice(Stream* streamHandle,
                                           qal_stream_type_t streamType,
                                           std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    std::vector<std::string> aifBackEndsToConnect;
    struct qal_device dAttr;
    int status = 0;

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEnds, txAifBackEnds);
    deviceToConnect->getDeviceAttributes(&dAttr);

    if (rxAifBackEnds.size() > 0) {
        status =  SessionAlsaUtils::connectSessionDevice(this, streamHandle,
                                                         streamType, rm,
                                                         dAttr, pcmDevRxIds,
                                                         rxAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: connectSessionDevice on RX Failed \n", __func__);
            return status;
        }
    } else if (txAifBackEnds.size() > 0) {

        status =  SessionAlsaUtils::connectSessionDevice(this, streamHandle,
                                                         streamType, rm,
                                                         dAttr, pcmDevTxIds,
                                                         txAifBackEnds);
        if(0 != status) {
            QAL_ERR(LOG_TAG,"%s: connectSessionDevice on TX Failed \n", __func__);
        }
    }
    return status;
}

int SessionAlsaVoice::setVoiceMixerParameter(Stream * s, struct mixer *mixer,
                                             void *payload, int size, int dir)
{
    char *control = "setParam";
    char *mixer_str;
    struct mixer_ctl *ctl;
    int ctl_len = 0,ret = 0;
    struct qal_stream_attributes sAttr;
    char *stream = SessionAlsaVoice::getMixerVoiceStream(s, dir);

    ret = s->getStreamAttributes(&sAttr);

    if (ret) {
         QAL_ERR(LOG_TAG, "could not get stream attributes\n");
        return ret;
    }

    ctl_len = strlen(stream) + 4 + strlen(control) + 1;
    mixer_str = (char *)calloc(1, ctl_len);
    if (!mixer_str) {
        free(payload);
        return -ENOMEM;
    }
    snprintf(mixer_str, ctl_len, "%s %s", stream, control);

    QAL_VERBOSE(LOG_TAG, "- mixer -%s-\n", mixer_str);
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_str);
        free(mixer_str);
        return ENOENT;
    }


    ret = mixer_ctl_set_array(ctl, payload, size);

    QAL_VERBOSE(LOG_TAG, "ret = %d, cnt = %d\n", ret, size);
    free(mixer_str);
    return ret;
}

char* SessionAlsaVoice::getMixerVoiceStream(Stream *s, int dir){
    char *stream = "VOICEMMODE1p";
    struct qal_stream_attributes sAttr;

    s->getStreamAttributes(&sAttr);
    if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
        sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
        if (dir == TXDIR) {
            stream = "VOICEMMODE1c";
        } else {
            stream = "VOICEMMODE1p";
        }
    } else {
        if (dir == TXDIR) {
            stream = "VOICEMMODE2c";
        } else {
            stream = "VOICEMMODE2p";
        }
    }

    return stream;
}

int SessionAlsaVoice::setECRef(Stream *s, std::shared_ptr<Device> rx_dev, bool is_enable)
{
    return 0;
}

