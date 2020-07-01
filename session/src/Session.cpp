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

#define LOG_TAG "QAL: Session"

#include "Session.h"
#include "Stream.h"
#include "ResourceManager.h"
#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "SessionAlsaCompress.h"
#include "SessionAlsaUtils.h"
#include "SessionAlsaVoice.h"
#include "SessionAlsaUtils.h"

#include <sstream>


Session::Session()
{

}

Session::~Session()
{

}


Session* Session::makeSession(const std::shared_ptr<ResourceManager>& rm, const struct qal_stream_attributes *sAttr)
{
    if (!rm || !sAttr) {
        QAL_ERR(LOG_TAG,"Invalid parameters passed");
        return nullptr;
    }

    Session* s = (Session*) nullptr;

    switch (sAttr->type) {
        //create compressed if the stream type is compressed
        case QAL_STREAM_COMPRESSED:
            s =  new SessionAlsaCompress(rm);
            break;
        case QAL_STREAM_VOICE_CALL:
            s = new SessionAlsaVoice(rm);
            break;
        default:
            s = new SessionAlsaPcm(rm);
            break;
    }
    return s;
}

void Session::getSamplerateChannelBitwidthTags(struct qal_media_config *config,
        uint32_t &mfc_sr_tag, uint32_t &ch_tag, uint32_t &bitwidth_tag)
{
    switch (config->sample_rate) {
        case SAMPLINGRATE_8K :
            mfc_sr_tag = MFC_SR_8K;
            break;
        case SAMPLINGRATE_16K :
            mfc_sr_tag = MFC_SR_16K;
            break;
        case SAMPLINGRATE_32K :
            mfc_sr_tag = MFC_SR_32K;
            break;
        case SAMPLINGRATE_44K :
            mfc_sr_tag = MFC_SR_44K;
            break;
        case SAMPLINGRATE_48K :
            mfc_sr_tag = MFC_SR_48K;
            break;
        case SAMPLINGRATE_96K :
            mfc_sr_tag = MFC_SR_96K;
            break;
        case SAMPLINGRATE_192K :
            mfc_sr_tag = MFC_SR_192K;
            break;
        case SAMPLINGRATE_384K :
            mfc_sr_tag = MFC_SR_384K;
            break;
        default:
            mfc_sr_tag = MFC_SR_48K;
            break;
    }
    switch (config->ch_info.channels) {
        case CHANNELS_1:
            ch_tag = CHS_1;
            break;
        case CHANNELS_2:
            ch_tag = CHS_2;
            break;
        case CHANNELS_3:
            ch_tag = CHS_3;
            break;
        case CHANNELS_4:
            ch_tag = CHS_4;
            break;
        case CHANNELS_5:
            ch_tag = CHS_5;
            break;
        case CHANNELS_6:
            ch_tag = CHS_6;
            break;
        case CHANNELS_7:
            ch_tag = CHS_7;
            break;
        case CHANNELS_8:
            ch_tag = CHS_8;
            break;
        default:
            ch_tag = CHS_1;
            break;
    }
    switch (config->bit_width) {
        case BITWIDTH_16:
            bitwidth_tag = BW_16;
            break;
        case BITWIDTH_24:
            bitwidth_tag = BW_24;
            break;
        case BITWIDTH_32:
            bitwidth_tag = BW_32;
            break;
        default:
            bitwidth_tag = BW_16;
            break;
    }
}

int Session::getEffectParameters(Stream *s __unused, effect_qal_payload_t *effectPayload)
{
    int status = 0;
    uint8_t *ptr = NULL;

    uint8_t *payloadData = NULL;
    size_t payloadSize = 0;
    int device = 0;
    uint32_t miid = 0;
    const char *control = "getParam";
    struct mixer_ctl *ctl;
    qal_effect_custom_payload_t *effectCustomPayload = nullptr;
    std::ostringstream CntrlName;
    PayloadBuilder builder;

    QAL_DBG(LOG_TAG, "Enter.");
    ctl = getFEMixerCtl(control, &device);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        status = -ENOENT;
        goto exit;
    }

    if (!rxAifBackEnds.empty()) { /** search in RX GKV */
        status = SessionAlsaUtils::getModuleInstanceId(mixer, device, rxAifBackEnds[0].second.data(),
                effectPayload->tag, &miid);
        if (status) /** if not found, reset miid to 0 again */
            miid = 0;
    }

    if (!txAifBackEnds.empty()) { /** search in TX GKV */
        status = SessionAlsaUtils::getModuleInstanceId(mixer, device, txAifBackEnds[0].second.data(),
                effectPayload->tag, &miid);
        if (status)
            miid = 0;
    }

    if (miid == 0) {
        QAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x",
                    effectPayload->tag);
        status = -EINVAL;
        goto exit;
    }

    effectCustomPayload = (qal_effect_custom_payload_t *)(effectPayload->payload);
    if (effectPayload->payloadSize < sizeof(qal_effect_custom_payload_t)) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "memory for retrieved data is too small");
        goto exit;
    }

    builder.payloadQuery(&payloadData, &payloadSize,
                            miid, effectCustomPayload->paramId,
                            effectPayload->payloadSize - sizeof(uint32_t));
    status = mixer_ctl_set_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Set custom config failed, status = %d", status);
        goto exit;
    }

    status = mixer_ctl_get_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Get custom config failed, status = %d", status);
        goto exit;
    }

    ptr = (uint8_t *)payloadData + sizeof(struct apm_module_param_data_t);
    ar_mem_cpy(effectCustomPayload->data, effectPayload->payloadSize,
                        ptr, effectPayload->payloadSize);

exit:
    if (payloadData)
        free(payloadData);
    QAL_ERR(LOG_TAG, "Exit. status %d", status);
    return status;
}

int Session::updateCustomPayload(void *payload, size_t size)
{
    if (!customPayloadSize) {
        customPayload = calloc(1, size);
    } else {
        customPayload = realloc(customPayload, customPayloadSize + size);
    }

    if (!customPayload) {
        QAL_ERR(LOG_TAG, "failed to allocate memory for custom payload");
        return -ENOMEM;
    }

    memcpy((uint8_t *)customPayload + customPayloadSize, payload, size);
    customPayloadSize += size;
    QAL_INFO(LOG_TAG, "customPayloadSize = %d", customPayloadSize);
    return 0;
}

int Session::pause(Stream * s __unused)
{
    return 0;
}

int Session::resume(Stream * s __unused)
{
     return 0;
}

int Session::handleDeviceRotation(Stream *s, qal_speaker_rotation_type rotation_type,
        int device, struct mixer *mixer, PayloadBuilder* builder,
        std::vector<std::pair<int32_t, std::string>> rxAifBackEnds)
{
    int status = 0;
    struct qal_stream_attributes sAttr;
    struct qal_device dAttr;
    struct sessionToPayloadParam deviceData;
    uint32_t miid = 0;
    uint8_t* alsaParamData = NULL;
    size_t alsaPayloadSize = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    if (QAL_AUDIO_OUTPUT== sAttr.direction) {
        status = s->getAssociatedDevices(associatedDevices);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed\n", __func__);
            return status;
        }

        for (int i = 0; i < associatedDevices.size(); i++) {
             status = associatedDevices[i]->getDeviceAttributes(&dAttr);
             if (0 != status) {
                 QAL_ERR(LOG_TAG,"%s: get Device Attributes Failed\n", __func__);
                 return status;
             }

             if ((QAL_DEVICE_OUT_SPEAKER == dAttr.id) &&
                  (2 == dAttr.config.ch_info.channels)) {
                 /* Get PSPD MFC MIID and configure to match to device config */
                 /* This has to be done after sending all mixer controls and
                  * before connect
                  */
                status =
                        SessionAlsaUtils::getModuleInstanceId(mixer,
                                                              device,
                                                              rxAifBackEnds[i].second.data(),
                                                              TAG_DEVICE_MFC_SR,
                                                              &miid);
                if (status != 0) {
                    QAL_ERR(LOG_TAG,"getModuleInstanceId failed");
                    return status;
                }
                QAL_DBG(LOG_TAG, "miid : %x id = %d, data %s, dev id = %d\n", miid,
                    device, rxAifBackEnds[i].second.data(), dAttr.id);

                deviceData.bitWidth = dAttr.config.bit_width;
                deviceData.sampleRate = dAttr.config.sample_rate;
                deviceData.numChannel = dAttr.config.ch_info.channels;
                deviceData.rotation_type = rotation_type;
                builder->payloadMFCConfig((uint8_t **)&alsaParamData,
                                           &alsaPayloadSize, miid, &deviceData);

                if (alsaPayloadSize) {
                    status = updateCustomPayload(alsaParamData, alsaPayloadSize);
                    delete alsaParamData;
                    if (0 != status) {
                        QAL_ERR(LOG_TAG,"%s: updateCustomPayload Failed\n", __func__);
                        return status;
                    }
                }
                status = SessionAlsaUtils::setMixerParameter(mixer,
                                                             device,
                                                             customPayload,
                                                             customPayloadSize);
                if (status != 0) {
                    QAL_ERR(LOG_TAG,"setMixerParameter failed");
                    return status;
                }
            }
        }
    }
    return status;
}

#if 0
int setConfig(Stream * s, qal_stream_type_t sType, configType type, uint32_t tag1,
        uint32_t tag2, uint32_t tag3)
{
    int status = 0;
    uint32_t tagsent = 0;
    struct agm_tag_config* tagConfig = nullptr;
    std::ostringstream tagCntrlName;
    char const *stream = "PCM";
    const char *setParamTagControl = "setParamTag";
    struct mixer_ctl *ctl = nullptr;
    uint32_t tkv_size = 0;

    if (sType == QAL_STREAM_COMPRESSED)
        stream = "COMPRESS";

    switch (type) {
        case MODULE:
            tkv.clear();
            if (tag1)
                builder->populateTagKeyVector(s, tkv, tag1, &tagsent);
            if (tag2)
                builder->populateTagKeyVector(s, tkv, tag2, &tagsent);
            if (tag3)
                builder->populateTagKeyVector(s, tkv, tag3, &tagsent);

            if (tkv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }
            tagConfig = (struct agm_tag_config*)malloc (sizeof(struct agm_tag_config) +
                            (tkv.size() * sizeof(agm_key_value)));
            if(!tagConfig) {
                status = -ENOMEM;
                goto exit;
            }
            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName << stream << pcmDevIds.at(0) << " " << setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                return -ENOENT;
            }

            tkv_size = tkv.size() * sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            tkv.clear();
            break;
        default:
            status = 0;
            break;
    }

exit:
    return status;
}
#endif

