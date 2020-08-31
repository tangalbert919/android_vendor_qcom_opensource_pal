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

#define LOG_TAG "PAL: SessionAlsaPcm"

#include "SessionAlsaPcm.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include "detection_cmn_api.h"
#include <agm_api.h>
#include <sstream>
#include <string>
#include "detection_cmn_api.h"
#include "audio_dam_buffer_api.h"
#include "apm_api.h"

#define SESSION_ALSA_MMAP_DEFAULT_OUTPUT_SAMPLING_RATE (48000)
#define SESSION_ALSA_MMAP_PERIOD_SIZE (SESSION_ALSA_MMAP_DEFAULT_OUTPUT_SAMPLING_RATE/1000)
#define SESSION_ALSA_MMAP_PERIOD_COUNT_MIN 32
#define SESSION_ALSA_MMAP_PERIOD_COUNT_MAX 512
#define SESSION_ALSA_MMAP_PERIOD_COUNT_DEFAULT (SESSION_ALSA_MMAP_PERIOD_COUNT_MAX)

SessionAlsaPcm::SessionAlsaPcm(std::shared_ptr<ResourceManager> Rm)
{
   rm = Rm;
   builder = new PayloadBuilder();
   customPayload = NULL;
   customPayloadSize = 0;
   pcm = NULL;
   pcmRx = NULL;
   pcmTx = NULL;
   mState = SESSION_IDLE;
   ecRefDevId = PAL_DEVICE_OUT_MIN;
}

SessionAlsaPcm::~SessionAlsaPcm()
{
   delete builder;

}


int SessionAlsaPcm::prepare(Stream * s __unused)
{
   return 0;
}

int SessionAlsaPcm::open(Stream * s)
{
    int status = 0;
    struct pal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    status = s->getStreamAttributes(&sAttr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        return status;
    }
    if (sAttr.type != PAL_STREAM_VOICE_CALL_RECORD && sAttr.type != PAL_STREAM_VOICE_CALL_MUSIC) {
        status = s->getAssociatedDevices(associatedDevices);
        if (0 != status) {
            PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
            return status;
        }

        rm->getBackEndNames(associatedDevices, rxAifBackEnds, txAifBackEnds);
        if (rxAifBackEnds.empty() && txAifBackEnds.empty()) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "no backend specified for this stream");
            return status;

        }
    }
    status = rm->getAudioMixer(&mixer);
    if (status) {
        PAL_ERR(LOG_TAG,"mixer error");
        return status;
    }
    if (sAttr.direction == PAL_AUDIO_INPUT) {
        pcmDevIds = rm->allocateFrontEndIds(sAttr, 0);
        if (pcmDevIds.size() == 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            return -EINVAL;
        }
    } else if (sAttr.direction == PAL_AUDIO_OUTPUT) {
        pcmDevIds = rm->allocateFrontEndIds(sAttr, 0);
        if (pcmDevIds.size() == 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            return -EINVAL;
        }
    } else {
        pcmDevRxIds = rm->allocateFrontEndIds(sAttr, RXLOOPBACK);
        pcmDevTxIds = rm->allocateFrontEndIds(sAttr, TXLOOPBACK);
        if (!pcmDevRxIds.size() || !pcmDevTxIds.size()) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            return -EINVAL;
        }
    }
    switch (sAttr.direction) {
        case PAL_AUDIO_INPUT:
            status = SessionAlsaUtils::open(s, rm, pcmDevIds, txAifBackEnds);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
                rm->freeFrontEndIds(pcmDevIds, sAttr, 0);
            }
            break;
        case PAL_AUDIO_OUTPUT:
            status = SessionAlsaUtils::open(s, rm, pcmDevIds, rxAifBackEnds);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
                rm->freeFrontEndIds(pcmDevIds, sAttr, 0);
            }
            break;
        case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
            status = SessionAlsaUtils::open(s, rm, pcmDevRxIds, pcmDevTxIds,
                    rxAifBackEnds, txAifBackEnds);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
                rm->freeFrontEndIds(pcmDevRxIds, sAttr, RXLOOPBACK);
                rm->freeFrontEndIds(pcmDevTxIds, sAttr, TXLOOPBACK);
            }
            break;
        default:
            PAL_ERR(LOG_TAG,"unsupported direction");
            break;
    }

    if (!status && sAttr.type == PAL_STREAM_VOICE_UI) {
        status = rm->registerMixerEventCallback(pcmDevIds,
            sessionCb, cbCookie, true);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "Failed to register callback to rm");
        }
    }
    return status;
}

int SessionAlsaPcm::setConfig(Stream * s, configType type, uint32_t tag1,
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
            if (!tagConfig) {
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
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                return -ENOENT;
            }

            tkv_size = tkv.size() * sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            if (tagConfig)
                free(tagConfig);
            tkv.clear();
            break;
        default:
            status = 0;
            break;
    }

exit:
    return status;
}

struct mixer_ctl* SessionAlsaPcm::getFEMixerCtl(const char *controlName, int *device)
{
    *device = pcmDevIds.at(0);
    std::ostringstream CntrlName;
    struct mixer_ctl *ctl;

    CntrlName << "PCM" <<pcmDevIds.at(0) << " " << controlName;
    ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        return nullptr;
    }

    return ctl;
}

uint32_t SessionAlsaPcm::getMIID(const char *backendName, uint32_t tagId, uint32_t *miid)
{
    int status = 0;
    int device = pcmDevIds.at(0);
/* REPLACE THIS WITH STORED INFO DURING INITIAL SETUP */
    status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                                                   backendName,
                                                   tagId, miid);
    if (0 != status)
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", tagId, status);

    return status;
}

int SessionAlsaPcm::setConfig(Stream * s, configType type, int tag)
{
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig;
    const char *setParamTagControl = "setParamTag";
    const char *stream = "PCM";
    const char *setCalibrationControl = "setCalibration";
    struct mixer_ctl *ctl;
    struct agm_cal_config *calConfig;
    std::ostringstream tagCntrlName;
    std::ostringstream calCntrlName;
    pal_stream_attributes sAttr;
    int tkv_size = 0;
    int ckv_size = 0;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    switch (type) {
        case MODULE:
            tkv.clear();
            status = builder->populateTagKeyVector(s, tkv, tag, &tagsent);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"Failed to set the tag configuration\n");
                goto exit;
            }

            if (tkv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }

            tagConfig = (struct agm_tag_config*)malloc (sizeof(struct agm_tag_config) +
                            (tkv.size() * sizeof(agm_key_value)));

            if (!tagConfig) {
                status = -EINVAL;
                goto exit;
            }

            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                return -ENOENT;
            }

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            if (tagConfig)
                free(tagConfig);
            tkv.clear();
            break;
            //todo calibration
        case CALIBRATION:
            status = builder->populateCalKeyVector(s, ckv, tag);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"Failed to set the calibration data\n");
                goto exit;
            }

            if (ckv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }

            calConfig = (struct agm_cal_config*)malloc (sizeof(struct agm_cal_config) +
                            (ckv.size() * sizeof(agm_key_value)));

            if (!calConfig) {
                status = -EINVAL;
                goto exit;
            }

            status = SessionAlsaUtils::getCalMetadata(ckv, calConfig);
            if (PAL_STREAM_LOOPBACK == sAttr.type) {
                calCntrlName<<stream<<pcmDevRxIds.at(0)<<" "<<setCalibrationControl;
            } else {
                calCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setCalibrationControl;
            }

            ctl = mixer_get_ctl_by_name(mixer, calCntrlName.str().data());
            if (!ctl) {
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", calCntrlName.str().data());
                return -ENOENT;
            }
            ckv_size = ckv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, calConfig, sizeof(struct agm_cal_config) + ckv_size);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            if (calConfig)
                free(calConfig);
            ckv.clear();
            break;
        default:
            PAL_ERR(LOG_TAG,"invalid type ");
            status = -EINVAL;
            goto exit;
    }

exit:
    PAL_DBG(LOG_TAG,"exit status:%d ", status);
    return status;
}

int SessionAlsaPcm::setTKV(Stream * s __unused, configType type, effect_pal_payload_t *effectPayload)
{
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig = nullptr;
    const char *setParamTagControl = "setParamTag";
    const char *stream = "PCM";
    struct mixer_ctl *ctl;
    std::ostringstream tagCntrlName;
    int tkv_size = 0;

    switch (type) {
        case MODULE:
        {
            pal_key_vector_t *pal_kvpair = (pal_key_vector_t *)effectPayload->payload;
            uint32_t num_tkvs =  pal_kvpair->num_tkvs;
            for (int i = 0; i < num_tkvs; i++) {
                    tkv.push_back(std::make_pair(pal_kvpair->kvp[i].key, pal_kvpair->kvp[i].value));
            }

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

            tagsent = effectPayload->tag;
            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                status = -ENOENT;
                goto exit;
            }
            PAL_VERBOSE(LOG_TAG, "mixer control: %s\n", tagCntrlName.str().data());

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            tkv.clear();

            break;
        }
        default:
            PAL_ERR(LOG_TAG,"invalid type ");
            status = -EINVAL;
            goto exit;
    }

exit:
    PAL_DBG(LOG_TAG,"exit status:%d ", status);
    if (tagConfig) {
        free(tagConfig);
        tagConfig = nullptr;
    }

    return status;
}

void SessionAlsaPcm::deRegisterAdmStream(Stream *s)
{
    if (rm->admDeregisterStreamFn)
        rm->admDeregisterStreamFn(rm->admData, static_cast<void *>(s));
}

void SessionAlsaPcm::registerAdmStream(Stream *s, pal_stream_direction_t dir,
        pal_stream_flags_t flags, struct pcm *pcm, struct pcm_config *cfg)
{
    switch (dir) {
    case PAL_AUDIO_INPUT:
        if (rm->admRegisterInputStreamFn) {
            rm->admRegisterInputStreamFn(rm->admData, static_cast<void *>(s));

            if (flags & PAL_STREAM_FLAG_MMAP_MASK) {
                if (rm->admSetConfigFn)
                    rm->admSetConfigFn(rm->admData, static_cast<void *>(s),
                            pcm, cfg);
            }
        }
        break;
    case PAL_AUDIO_OUTPUT:
        if (rm->admRegisterOutputStreamFn) {
            rm->admRegisterOutputStreamFn(rm->admData, static_cast<void *>(s));

            if (flags & PAL_STREAM_FLAG_MMAP_MASK) {
                if (rm->admSetConfigFn)
                    rm->admSetConfigFn(rm->admData, static_cast<void *>(s),
                            pcm, cfg);
            }
        }
        break;
    default:
        break;
    }
}

void SessionAlsaPcm::requestAdmFocus(Stream *s,  long ns)
{
    if (rm->admRequestFocusV2Fn)
        rm->admRequestFocusV2Fn(rm->admData, static_cast<void *>(s),
                ns);
    else if (rm->admRequestFocusFn)
        rm->admRequestFocusFn(rm->admData, static_cast<void *>(s));
}

void SessionAlsaPcm::releaseAdmFocus(Stream *s)
{
    if (rm->admAbandonFocusFn)
        rm->admAbandonFocusFn(rm->admData, static_cast<void *>(s));
}

int SessionAlsaPcm::start(Stream * s)
{
    struct pcm_config config;
    struct pal_stream_attributes sAttr;
    int32_t status = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct pal_device dAttr;
    struct sessionToPayloadParam deviceData;
    struct sessionToPayloadParam streamData;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    uint32_t miid;
    int payload_size = 0;
    struct agm_event_reg_cfg *event_cfg;
    int tagId;


    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    if (mState == SESSION_IDLE) {
        s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
        memset(&config, 0, sizeof(config));

        if (sAttr.direction == PAL_AUDIO_INPUT) {
            config.rate = sAttr.in_media_config.sample_rate;
            if (sAttr.in_media_config.bit_width == 32)
                config.format = PCM_FORMAT_S32_LE;
            else if (sAttr.in_media_config.bit_width == 24)
                config.format = PCM_FORMAT_S24_3LE;
            else if (sAttr.in_media_config.bit_width == 16)
                config.format = PCM_FORMAT_S16_LE;
            config.channels = sAttr.in_media_config.ch_info.channels;
            config.period_size = SessionAlsaUtils::bytesToFrames(in_buf_size,
                config.channels, config.format);
            config.period_count = in_buf_count;
        } else {
            config.rate = sAttr.out_media_config.sample_rate;
            if (sAttr.out_media_config.bit_width == 32)
                config.format = PCM_FORMAT_S32_LE;
            else if (sAttr.out_media_config.bit_width == 24)
                config.format = PCM_FORMAT_S24_3LE;
            else if (sAttr.out_media_config.bit_width == 16)
                config.format = PCM_FORMAT_S16_LE;
            config.channels = sAttr.out_media_config.ch_info.channels;
            config.period_size = SessionAlsaUtils::bytesToFrames(out_buf_size,
                config.channels, config.format);
            config.period_count = out_buf_count;
        }
        config.start_threshold = 0;
        config.stop_threshold = 0;
        config.silence_threshold = 0;

        switch(sAttr.direction) {
            case PAL_AUDIO_INPUT:
                if (sAttr.type == PAL_STREAM_VOICE_UI) {
                    SessionAlsaUtils::setMixerParameter(mixer, pcmDevIds.at(0), customPayload, customPayloadSize);
                    if (customPayload) {
                        free(customPayload);
                        customPayload = NULL;
                        customPayloadSize = 0;
                    }
                }

                if(SessionAlsaUtils::isMmapUsecase(sAttr)) {
                    config.start_threshold = 0;
                    config.stop_threshold = INT32_MAX;
                    config.silence_threshold = 0;
                    config.silence_size = 0;
                    config.avail_min = config.period_size;
                    pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0),
                        PCM_IN |PCM_MMAP| PCM_NOIRQ, &config);
                }
                else {
                    pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_IN, &config);
                }

                if (!pcm) {
                    PAL_ERR(LOG_TAG, "pcm open failed");
                    return -EINVAL;
                }

                if (!pcm_is_ready(pcm)) {
                    PAL_ERR(LOG_TAG, "pcm open not ready");
                    return -EINVAL;
                }
                break;
            case PAL_AUDIO_OUTPUT:
                if(SessionAlsaUtils::isMmapUsecase(sAttr)) {
                    config.start_threshold = config.period_size * 8;
                    config.stop_threshold = INT32_MAX;
                    config.silence_threshold = 0;
                    config.silence_size = 0;
                    config.avail_min = config.period_size;
                    pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0),
                        PCM_OUT |PCM_MMAP| PCM_NOIRQ, &config);
                }
                else {
                    if (sAttr.type != PAL_STREAM_VOICE_CALL_MUSIC) {
                        status = SessionAlsaUtils::getModuleInstanceId(mixer,
                                 pcmDevIds.at(0), rxAifBackEnds[0].second.data(),
                                 STREAM_SPR, &spr_miid);
                        if (0 != status) {
                            PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", STREAM_SPR, status);
                            status = 0; //TODO: add this to some policy in pal
                        }
                    }
                    pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_OUT, &config);
                }

                if (!pcm) {
                    PAL_ERR(LOG_TAG, "pcm open failed");
                    return -EINVAL;
                }

                if (!pcm_is_ready(pcm)) {
                    PAL_ERR(LOG_TAG, "pcm open not ready");
                    return -EINVAL;
                }
                break;
            case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
                pcmRx = pcm_open(rm->getSndCard(), pcmDevRxIds.at(0), PCM_OUT, &config);
                if (!pcmRx) {
                    PAL_ERR(LOG_TAG, "pcm-rx open failed");
                    return -EINVAL;
                }

                if (!pcm_is_ready(pcmRx)) {
                    PAL_ERR(LOG_TAG, "pcm-rx open not ready");
                    return -EINVAL;
                }
                pcmTx = pcm_open(rm->getSndCard(), pcmDevTxIds.at(0), PCM_IN, &config);
                if (!pcmTx) {
                    PAL_ERR(LOG_TAG, "pcm-tx open failed");
                    return -EINVAL;
                }

                if (!pcm_is_ready(pcmTx)) {
                    PAL_ERR(LOG_TAG, "pcm-tx open not ready");
                    return -EINVAL;
                }
                break;
        }
        mState = SESSION_OPENED;

        if (SessionAlsaUtils::isMmapUsecase(sAttr) &&
                !(sAttr.flags & PAL_STREAM_FLAG_MMAP_NO_IRQ_MASK))
            registerAdmStream(s, sAttr.direction, sAttr.flags, pcm, &config);
    }
    if (sAttr.type == PAL_STREAM_VOICE_UI) {
        payload_size = sizeof(struct agm_event_reg_cfg);

        event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
        if (!event_cfg) {
            status = -ENOMEM;
        }
        event_cfg->event_id = EVENT_ID_DETECTION_ENGINE_GENERIC_INFO;
        event_cfg->event_config_payload_size = 0;
        event_cfg->is_register = 1;
        SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIds.at(0),
                txAifBackEnds[0].second.data(), DEVICE_SVA, (void *)event_cfg,
                payload_size);
    }

    switch (sAttr.direction) {
        case PAL_AUDIO_INPUT:
            if ((sAttr.type != PAL_STREAM_VOICE_UI) &&
                  (SessionAlsaUtils::isMmapUsecase(sAttr) == false)
                ) {
                /* Get MFC MIID and configure to match to stream config */
                /* This has to be done after sending all mixer controls and before connect */
                if (sAttr.type != PAL_STREAM_VOICE_CALL_RECORD)
                    status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIds.at(0),
                                                                txAifBackEnds[0].second.data(),
                                                                TAG_STREAM_MFC_SR, &miid);
                else
                    status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIds.at(0),
                                                                "ZERO", TAG_STREAM_MFC_SR, &miid);
                if (status != 0) {
                    PAL_ERR(LOG_TAG,"getModuleInstanceId failed");
                    return status;
                }
                if (sAttr.type != PAL_STREAM_VOICE_CALL_RECORD) {
                    PAL_ERR(LOG_TAG, "miid : %x id = %d, data %s\n", miid,
                        pcmDevIds.at(0), txAifBackEnds[0].second.data());
                } else
                    PAL_ERR(LOG_TAG, "miid : %x id = %d\n", miid, pcmDevIds.at(0));
                streamData.bitWidth = sAttr.in_media_config.bit_width;
                streamData.sampleRate = sAttr.in_media_config.sample_rate;
                streamData.numChannel = sAttr.in_media_config.ch_info.channels;
                streamData.rotation_type = PAL_SPEAKER_ROTATION_LR;
                streamData.ch_info = nullptr;
                builder->payloadMFCConfig(&payload, &payloadSize, miid, &streamData);
                if (payloadSize) {
                    status = updateCustomPayload(payload, payloadSize);
                    delete payload;
                    if (0 != status) {
                        PAL_ERR(LOG_TAG,"updateCustomPayload Failed\n");
                        return status;
                    }
                }
                status = SessionAlsaUtils::setMixerParameter(mixer, pcmDevIds.at(0),
                                                             customPayload, customPayloadSize);
                if (customPayload) {
                    free(customPayload);
                    customPayload = NULL;
                    customPayloadSize = 0;
                }
                if (status != 0) {
                    PAL_ERR(LOG_TAG,"setMixerParameter failed");
                    return status;
                }
                if (sAttr.type == PAL_STREAM_VOICE_CALL_RECORD) {
                    switch (sAttr.info.voice_rec_info.record_direction) {
                        case INCALL_RECORD_VOICE_UPLINK:
                            tagId = INCALL_RECORD_UPLINK;
                            break;
                        case INCALL_RECORD_VOICE_DOWNLINK:
                            tagId = INCALL_RECORD_DOWNLINK;
                            break;
                        case INCALL_RECORD_VOICE_UPLINK_DOWNLINK:
                            if (sAttr.in_media_config.ch_info.channels == 2)
                                tagId = INCALL_RECORD_UPLINK_DOWNLINK_STEREO;
                            else
                                tagId = INCALL_RECORD_UPLINK_DOWNLINK_MONO;
                            break;
                    }
                    status = setConfig(s, MODULE, tagId);
                    if (status)
                        PAL_ERR(LOG_TAG, "Failed to set incall record params status = %d", status);
                }
            }
            status = pcm_start(pcm);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start failed %d", status);
            }
            break;
        case PAL_AUDIO_OUTPUT:
            if (sAttr.type == PAL_STREAM_VOICE_CALL_MUSIC)
                goto pcm_start;
            status = s->getAssociatedDevices(associatedDevices);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"getAssociatedDevices Failed\n");
                return status;
            }
            for (int i = 0; i < associatedDevices.size();i++) {
                status = associatedDevices[i]->getDeviceAttributes(&dAttr);
                if (0 != status) {
                    PAL_ERR(LOG_TAG,"get Device Attributes Failed\n");
                    return status;
                }

                if(!(SessionAlsaUtils::isMmapUsecase(sAttr))) {
                    /* Get PSPD MFC MIID and configure to match to device config */
                    /* This has to be done after sending all mixer controls and before connect */
                    status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIds.at(0),
                                                                   rxAifBackEnds[i].second.data(),
                                                                   TAG_DEVICE_MFC_SR, &miid);
                    if (status != 0) {
                        PAL_ERR(LOG_TAG,"getModuleInstanceId failed");
                        return status;
                    }
                    PAL_DBG(LOG_TAG, "miid : %x id = %d, data %s, dev id = %d\n", miid,
                            pcmDevIds.at(0), rxAifBackEnds[i].second.data(), dAttr.id);
                    deviceData.bitWidth = dAttr.config.bit_width;
                    deviceData.sampleRate = dAttr.config.sample_rate;
                    deviceData.numChannel = dAttr.config.ch_info.channels;
                    deviceData.rotation_type = PAL_SPEAKER_ROTATION_LR;
                    deviceData.ch_info = nullptr;

                    if ((PAL_DEVICE_OUT_SPEAKER == dAttr.id) &&
                        (2 == dAttr.config.ch_info.channels)) {
                    // Stereo Speakers. Check for the rotation type
                        if (PAL_SPEAKER_ROTATION_RL ==
                                                rm->getCurrentRotationType()) {
                        // Rotation is of RL, so need to swap the channels
                            deviceData.rotation_type = PAL_SPEAKER_ROTATION_RL;
                        }
                    }
                    if (dAttr.id == PAL_DEVICE_OUT_AUX_DIGITAL ||
                        dAttr.id == PAL_DEVICE_OUT_AUX_DIGITAL_1 ||
                        dAttr.id == PAL_DEVICE_OUT_HDMI)
                        deviceData.ch_info = &dAttr.config.ch_info;

                    builder->payloadMFCConfig((uint8_t **)&payload, &payloadSize, miid, &deviceData);
                    if (payloadSize) {
                        status = updateCustomPayload(payload, payloadSize);
                        delete payload;
                        if (0 != status) {
                            PAL_ERR(LOG_TAG,"updateCustomPayload Failed\n");
                            return status;
                        }
                    }

                    status = SessionAlsaUtils::setMixerParameter(mixer, pcmDevIds.at(0),
                                                                 customPayload, customPayloadSize);
                    if (customPayload) {
                        free(customPayload);
                        customPayload = NULL;
                        customPayloadSize = 0;
                    }
                    if (status != 0) {
                        PAL_ERR(LOG_TAG,"setMixerParameter failed");
                        return status;
                    }
                }
            }

            //status = pcm_prepare(pcm);
            //if (status) {
            //    PAL_ERR(LOG_TAG, "pcm_prepare failed %d", status);
            //}
pcm_start:
            status = pcm_start(pcm);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start failed %d", status);
            }
            break;
        case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
            status = s->getAssociatedDevices(associatedDevices);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed\n", __func__);
                return status;
            }
            for (int i = 0; i < associatedDevices.size(); i++) {
                if (!SessionAlsaUtils::isRxDevice(
                            associatedDevices[i]->getSndDeviceId()))
                    continue;

                status = associatedDevices[i]->getDeviceAttributes(&dAttr);
                if (0 != status) {
                    PAL_ERR(LOG_TAG,"%s: get Device Attributes Failed\n", __func__);
                    return status;
                }

                status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevRxIds.at(0),
                        rxAifBackEnds[0].second.data(),
                        TAG_DEVICE_MFC_SR, &miid);
                if (status != 0) {
                    PAL_ERR(LOG_TAG,"getModuleInstanceId failed");
                    status = 0;
                    goto pcm_start_loopback;
                }
                PAL_DBG(LOG_TAG, "miid : %x id = %d, data %s, dev id = %d\n", miid,
                        pcmDevRxIds.at(0), rxAifBackEnds[0].second.data(), dAttr.id);
                deviceData.bitWidth = dAttr.config.bit_width;
                deviceData.sampleRate = dAttr.config.sample_rate;
                deviceData.numChannel = dAttr.config.ch_info.channels;
                deviceData.rotation_type = PAL_SPEAKER_ROTATION_LR;
                deviceData.ch_info = nullptr;

                builder->payloadMFCConfig((uint8_t **)&payload, &payloadSize, miid, &deviceData);
                if (payloadSize) {
                    status = updateCustomPayload(payload, payloadSize);
                    delete payload;
                    if (0 != status) {
                        PAL_ERR(LOG_TAG,"%s: updateCustomPayload Failed\n", __func__);
                        return status;
                    }
                }
                status = SessionAlsaUtils::setMixerParameter(mixer, pcmDevRxIds.at(0),
                        customPayload, customPayloadSize);
                if (status != 0) {
                    PAL_ERR(LOG_TAG,"setMixerParameter failed");
                    return status;
                }
           }
pcm_start_loopback:
            status = pcm_start(pcmRx);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start rx failed %d", status);
            }
            status = pcm_start(pcmTx);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start tx failed %d", status);
            }
           break;
    }
    // Setting the volume as in stream open, no default volume is set.
    if (setConfig(s, CALIBRATION, TAG_STREAM_VOLUME) != 0) {
            PAL_ERR(LOG_TAG,"Setting volume failed");
    }

    mState = SESSION_STARTED;

    return status;
}

int SessionAlsaPcm::stop(Stream * s)
{
    int status = 0;
    struct pal_stream_attributes sAttr;
    struct agm_event_reg_cfg *event_cfg;
    int payload_size = 0;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }
    switch (sAttr.direction) {
        case PAL_AUDIO_INPUT:
        case PAL_AUDIO_OUTPUT:
            if (pcm && isActive()) {
                status = pcm_stop(pcm);
                if (status) {
                    PAL_ERR(LOG_TAG, "pcm_stop failed %d", status);
                }
            }
            break;
        case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
            if (pcmRx && isActive()) {
                status = pcm_stop(pcmRx);
                if (status) {
                    PAL_ERR(LOG_TAG, "pcm_stop - rx failed %d", status);
                }
            }
            if (pcmTx && isActive()) {
                status = pcm_stop(pcmTx);
                if (status) {
                    PAL_ERR(LOG_TAG, "pcm_stop - tx failed %d", status);
                }
            }
            break;
    }
    mState = SESSION_STOPPED;

    if (sAttr.type == PAL_STREAM_VOICE_UI) {
        payload_size = sizeof(struct agm_event_reg_cfg);

        event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
        if (!event_cfg) {
            status = -ENOMEM;
        }
        event_cfg->event_id = EVENT_ID_DETECTION_ENGINE_GENERIC_INFO;
        event_cfg->event_config_payload_size = 0;
        event_cfg->is_register = 0;
        SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIds.at(0),
            txAifBackEnds[0].second.data(), DEVICE_SVA, (void *) event_cfg,
            payload_size);
    }
    return status;
}

int SessionAlsaPcm::close(Stream * s)
{
    int status = 0;
    struct pal_stream_attributes sAttr;
    std::shared_ptr<Device> dev = nullptr;
    std::string backendname;
    int32_t beDevId = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        goto exit;
    }
    if (sAttr.type != PAL_STREAM_VOICE_CALL_RECORD && sAttr.type != PAL_STREAM_VOICE_CALL_MUSIC) {
        status = s->getAssociatedDevices(associatedDevices);
        if (status != 0) {
            PAL_ERR(LOG_TAG,"getAssociatedDevices Failed\n");
            return status;
        }
    }

    switch (sAttr.direction) {
        case PAL_AUDIO_INPUT:
            for (auto dev: associatedDevices) {
                beDevId = dev->getSndDeviceId();
                rm->getBackendName(beDevId, backendname);
                PAL_DBG(LOG_TAG, "backendname %s", backendname.c_str());
                if (dev->getDeviceCount() != 0) {
                    PAL_DBG(LOG_TAG, "Tx dev still active\n");
                    freeDeviceMetadata.push_back(std::make_pair(backendname, 0));
                } else {
                    freeDeviceMetadata.push_back(std::make_pair(backendname, 1));
                    PAL_DBG(LOG_TAG, "Tx dev not active");
                }
            }
            status = SessionAlsaUtils::close(s, rm, pcmDevIds, txAifBackEnds, freeDeviceMetadata);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa close failed with %d", status);
            }
            if (SessionAlsaUtils::isMmapUsecase(sAttr) &&
                !(sAttr.flags & PAL_STREAM_FLAG_MMAP_NO_IRQ_MASK))
                deRegisterAdmStream(s);
            if (pcm)
                status = pcm_close(pcm);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_close failed %d", status);
            }
            rm->freeFrontEndIds(pcmDevIds, sAttr, 0);
            pcm = NULL;
            break;
        case PAL_AUDIO_OUTPUT:
            for (auto dev: associatedDevices) {
                beDevId = dev->getSndDeviceId();
                rm->getBackendName(beDevId, backendname);
                PAL_DBG(LOG_TAG, "backendname %s", backendname.c_str());
                if (dev->getDeviceCount() != 0) {
                    PAL_DBG(LOG_TAG, "Rx dev still active");
                    freeDeviceMetadata.push_back(std::make_pair(backendname, 0));
                } else {
                    PAL_DBG(LOG_TAG, "Rx dev not active");
                    freeDeviceMetadata.push_back(std::make_pair(backendname, 1));
                }
            }
            status = SessionAlsaUtils::close(s, rm, pcmDevIds, rxAifBackEnds, freeDeviceMetadata);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa close failed with %d", status);
            }
            if (SessionAlsaUtils::isMmapUsecase(sAttr) &&
                !(sAttr.flags & PAL_STREAM_FLAG_MMAP_NO_IRQ_MASK))
                deRegisterAdmStream(s);
            if (pcm)
                status = pcm_close(pcm);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_close failed %d", status);
            }
            rm->freeFrontEndIds(pcmDevIds, sAttr, 0);
            pcm = NULL;
            break;
        case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
            status = SessionAlsaUtils::close(s, rm, pcmDevRxIds, pcmDevTxIds,
                    rxAifBackEnds, txAifBackEnds);
            if (status) {
                PAL_ERR(LOG_TAG, "session alsa close failed with %d", status);
            }
            if (pcmRx)
                status = pcm_close(pcmRx);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_close - rx failed %d", status);
            }
            if (pcmTx)
                status = pcm_close(pcmTx);
            if (status) {
               PAL_ERR(LOG_TAG, "pcm_close - tx failed %d", status);
            }
            rm->freeFrontEndIds(pcmDevRxIds, sAttr, RXLOOPBACK);
            rm->freeFrontEndIds(pcmDevTxIds, sAttr, TXLOOPBACK);
            pcmRx = NULL;
            pcmTx = NULL;
            break;
    }

    mState = SESSION_IDLE;

    if (sAttr.type == PAL_STREAM_VOICE_UI) {
        status = rm->registerMixerEventCallback(pcmDevIds,
            sessionCb, cbCookie, false);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "Failed to deregister callback to rm");
        }
    }

    if (customPayload) {
        free(customPayload);
        customPayload = NULL;
        customPayloadSize = 0;
    }
exit:
    return status;
}
/* TODO: Check if this can be moved to Session class */
int SessionAlsaPcm::disconnectSessionDevice(Stream *streamHandle,
        pal_stream_type_t streamType, std::shared_ptr<Device> deviceToDisconnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    struct pal_device dAttr;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEndsToDisconnect;
    std::vector<std::pair<int32_t, std::string>> txAifBackEndsToDisconnect;
    int32_t status = 0;

    deviceList.push_back(deviceToDisconnect);
    rm->getBackEndNames(deviceList, rxAifBackEndsToDisconnect,
            txAifBackEndsToDisconnect);
    deviceToDisconnect->getDeviceAttributes(&dAttr);

    if (!rxAifBackEndsToDisconnect.empty()) {
        int cnt = 0;
        status = SessionAlsaUtils::disconnectSessionDevice(streamHandle, streamType, rm,
            dAttr, pcmDevIds, rxAifBackEndsToDisconnect);

        for (const auto &elem : rxAifBackEnds) {
            cnt++;
            for (const auto &disConnectElem : rxAifBackEndsToDisconnect) {
                if (std::get<0>(elem) == std::get<0>(disConnectElem))
                    rxAifBackEnds.erase(rxAifBackEnds.begin() + cnt - 1, rxAifBackEnds.begin() + cnt);
            }
        }
    }
    if (!txAifBackEndsToDisconnect.empty()) {
        int cnt = 0;

        status = SessionAlsaUtils::disconnectSessionDevice(streamHandle, streamType, rm,
            dAttr, pcmDevIds, txAifBackEndsToDisconnect);

        for (const auto &elem : txAifBackEnds) {
            cnt++;
            for (const auto &disConnectElem : txAifBackEndsToDisconnect) {
                if (std::get<0>(elem) == std::get<0>(disConnectElem))
                    txAifBackEnds.erase(txAifBackEnds.begin() + cnt - 1, txAifBackEnds.begin() + cnt);
            }
        }
    }

    return status;
}

int SessionAlsaPcm::setupSessionDevice(Stream* streamHandle, pal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    struct pal_device dAttr;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEndsToConnect;
    std::vector<std::pair<int32_t, std::string>> txAifBackEndsToConnect;
    int32_t status = 0;

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEndsToConnect,
            txAifBackEndsToConnect);
    deviceToConnect->getDeviceAttributes(&dAttr);

    if (!rxAifBackEndsToConnect.empty())
        status = SessionAlsaUtils::setupSessionDevice(streamHandle, streamType, rm,
            dAttr, pcmDevIds, rxAifBackEndsToConnect);

    if (!txAifBackEndsToConnect.empty())
        status = SessionAlsaUtils::setupSessionDevice(streamHandle, streamType, rm,
            dAttr, pcmDevIds, txAifBackEndsToConnect);

    return status;
}

int SessionAlsaPcm::connectSessionDevice(Stream* streamHandle, pal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    struct pal_device dAttr;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEndsToConnect;
    std::vector<std::pair<int32_t, std::string>> txAifBackEndsToConnect;
    int32_t status = 0;

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEndsToConnect,
            txAifBackEndsToConnect);
    deviceToConnect->getDeviceAttributes(&dAttr);

    if (!rxAifBackEndsToConnect.empty()) {
        status = SessionAlsaUtils::connectSessionDevice(NULL, streamHandle, streamType, rm,
            dAttr, pcmDevIds, rxAifBackEndsToConnect);
        for (const auto &elem : rxAifBackEndsToConnect)
            rxAifBackEnds.push_back(elem);
    }

    if (!txAifBackEndsToConnect.empty()) {
        status = SessionAlsaUtils::connectSessionDevice(NULL, streamHandle, streamType, rm,
            dAttr, pcmDevIds, txAifBackEndsToConnect);
        for (const auto &elem : txAifBackEndsToConnect)
            txAifBackEnds.push_back(elem);
    }

    return status;
}

int SessionAlsaPcm::read(Stream *s, int tag __unused, struct pal_buffer *buf, int * size)
{
    int status = 0, bytesRead = 0, bytesToRead = 0, offset = 0, pcmReadSize = 0;
    uint64_t timestamp = 0;
    const char *control = "bufTimestamp";
    const char *stream = "PCM";
    struct mixer_ctl *ctl;
    std::ostringstream CntrlName;
    struct pal_stream_attributes sAttr;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }
    while (1) {
        offset = bytesRead + buf->offset;
        bytesToRead = buf->size - offset;
        if (!bytesToRead)
            break;
        if ((bytesToRead / in_buf_size) >= 1)
            pcmReadSize = in_buf_size;
        else
            pcmReadSize = bytesToRead;
        void *data = buf->buffer;
        data = static_cast<char*>(data) + offset;

        if(SessionAlsaUtils::isMmapUsecase(sAttr))
        {
            long ns = 0;
            if (sAttr.in_media_config.sample_rate)
                ns = pcm_bytes_to_frames(pcm, pcmReadSize)*1000000000LL/
                    sAttr.in_media_config.sample_rate;
            requestAdmFocus(s, ns);
            status =  pcm_mmap_read(pcm, data,  pcmReadSize);
            releaseAdmFocus(s);
        } else {
            status =  pcm_read(pcm, data,  pcmReadSize);
        }

        if ((0 != status) || (pcmReadSize == 0)) {
            PAL_ERR(LOG_TAG,"Failed to read data %d bytes read %d", status, pcmReadSize);
            break;
        }

        if (!bytesRead && buf->ts &&
            (sAttr.type == PAL_STREAM_VOICE_UI)) {
            CntrlName << stream << pcmDevIds.at(0) << " " << control;
            ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
            if (!ctl) {
                PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
                status = -ENOENT;
                goto exit;
            }

            status = mixer_ctl_get_array(ctl, (void *)&timestamp, sizeof(uint64_t));
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Get timestamp failed, status = %d", status);
                goto exit;
            }

            buf->ts->tv_sec = timestamp / 1000000;
            buf->ts->tv_nsec = (timestamp - buf->ts->tv_sec * 1000000) * 1000;
            PAL_VERBOSE(LOG_TAG, "Timestamp %llu, tv_sec = %ld, tv_nsec = %ld",
                       (long long)timestamp, buf->ts->tv_sec, buf->ts->tv_nsec);
        }
        bytesRead += pcmReadSize;
    }
exit:
    *size = bytesRead;
    PAL_DBG(LOG_TAG,"exit bytesRead:%d status:%d ", bytesRead, status);
    return status;
}

int SessionAlsaPcm::write(Stream *s, int tag, struct pal_buffer *buf, int * size,
                          int flag)
{
    int status = 0, bytesWritten = 0, bytesRemaining = 0, offset = 0;
    uint32_t sizeWritten = 0;
    struct pal_stream_attributes sAttr;


    PAL_VERBOSE(LOG_TAG,"enter buf:%p tag:%d flag:%d", buf, tag, flag);

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    void *data = nullptr;

    bytesRemaining = buf->size;

    while ((bytesRemaining / out_buf_size) > 1) {
        offset = bytesWritten + buf->offset;
        data = buf->buffer;
        data = static_cast<char *>(data) + offset;
        sizeWritten = out_buf_size;  //initialize 0
        if (pcm && (mState == SESSION_FLUSHED)) {
            status = pcm_start(pcm);
            if (status) {
                PAL_ERR(LOG_TAG, "pcm_start failed %d", status);
                return -EINVAL;
            }
            mState = SESSION_STARTED;
        }

        if(SessionAlsaUtils::isMmapUsecase(sAttr))
        {
            long ns = 0;
            if (sAttr.out_media_config.sample_rate)
                ns = pcm_bytes_to_frames(pcm, sizeWritten)*1000000000LL/
                    sAttr.out_media_config.sample_rate;
            PAL_DBG(LOG_TAG,"1.bufsize:%u ns:%ld", sizeWritten, ns);
            requestAdmFocus(s, ns);
            status =  pcm_mmap_write(pcm, data,  sizeWritten);
            releaseAdmFocus(s);
        } else {
            status =  pcm_write(pcm, data,  sizeWritten);
        }

        if (0 != status) {
            PAL_ERR(LOG_TAG,"Failed to write the data");
            return status;
        }
        bytesWritten += sizeWritten;
        bytesRemaining -= sizeWritten;
    }
    offset = bytesWritten + buf->offset;
    sizeWritten = bytesRemaining;
    data = buf->buffer;
    if (pcm && (mState == SESSION_FLUSHED)) {
        status = pcm_start(pcm);
        if (status) {
            PAL_ERR(LOG_TAG, "pcm_start failed %d", status);
            return -EINVAL;
        }
        mState = SESSION_STARTED;
    }
    data = static_cast<char *>(data) + offset;
    if (SessionAlsaUtils::isMmapUsecase(sAttr)) {
        if (sizeWritten) {
            long ns = 0;
            if (sAttr.out_media_config.sample_rate)
                ns = pcm_bytes_to_frames(pcm, sizeWritten)*1000000000LL/
                    sAttr.out_media_config.sample_rate;
            PAL_DBG(LOG_TAG,"2.bufsize:%u ns:%ld", sizeWritten, ns);
            requestAdmFocus(s, ns);
            status =  pcm_mmap_write(pcm, data,  sizeWritten);
            releaseAdmFocus(s);
            if (status != 0) {
                PAL_ERR(LOG_TAG,"Error! pcm_mmap_write failed");
                return status;
            }
        }
    } else {
        status =  pcm_write(pcm, data,  sizeWritten);
        if (status != 0) {
            PAL_ERR(LOG_TAG,"Error! pcm_write failed");
            return status;
        }
    }
    bytesWritten += sizeWritten;
    *size = bytesWritten;
    return status;
}

int SessionAlsaPcm::readBufferInit(Stream * /*streamHandle*/, size_t /*noOfBuf*/, size_t /*bufSize*/,
                                   int /*flag*/)
{
    return 0;
}
int SessionAlsaPcm::writeBufferInit(Stream * /*streamHandle*/, size_t /*noOfBuf*/, size_t /*bufSize*/,
                                    int /*flag*/)
{
    return 0;
}

int SessionAlsaPcm::setParameters(Stream *streamHandle __unused, int tagId __unused, uint32_t param_id, void *payload)
{
    int status = 0;
    int device = pcmDevIds.at(0);
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    uint32_t miid = 0;
    effect_pal_payload_t *effectPalPayload = nullptr;

    PAL_DBG(LOG_TAG, "Enter.");
    switch (param_id) {
        case PAL_PARAM_ID_DEVICE_ROTATION:
        {
            pal_param_device_rotation_t *rotation =
                                         (pal_param_device_rotation_t *)payload;
            status = handleDeviceRotation(streamHandle, rotation->rotation_type,
                                          device, mixer, builder, rxAifBackEnds);
            goto exit;
        }
        case PARAM_ID_DETECTION_ENGINE_SOUND_MODEL:
        {
            struct pal_st_sound_model *pSoundModel = NULL;
            pSoundModel = (struct pal_st_sound_model *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_SVA, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVASoundModel(&paramData, &paramSize, miid, pSoundModel);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP:
        {
            struct detection_engine_config_voice_wakeup *pWakeUpConfig = NULL;
            pWakeUpConfig = (struct detection_engine_config_voice_wakeup *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_SVA, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAWakeUpConfig(&paramData, &paramSize, miid, pWakeUpConfig);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG:
        {
            struct detection_engine_generic_event_cfg *pEventConfig = NULL;
            pEventConfig = (struct detection_engine_generic_event_cfg *)payload;
            // set custom config for detection event
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_SVA, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAEventConfig(&paramData, &paramSize, miid, pEventConfig);
            break;
        }
        case PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG:
        {
            struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufConfig = NULL;
            pWakeUpBufConfig = (struct detection_engine_voice_wakeup_buffer_config *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_SVA, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAWakeUpBufferConfig(&paramData, &paramSize, miid, pWakeUpBufConfig);
            break;
        }
        case PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION:
        {
            struct audio_dam_downstream_setup_duration *pSetupDuration = NULL;
            pSetupDuration = (struct audio_dam_downstream_setup_duration *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_ADAM, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_ADAM, status);
                goto exit;
            }
            builder->payloadSVAStreamSetupDuration(&paramData, &paramSize, miid, pSetupDuration);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_RESET:
        {
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                    txAifBackEnds[0].second.data(), DEVICE_SVA, &miid);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAEngineReset(&paramData, &paramSize, miid);
            status = SessionAlsaUtils::setMixerParameter(mixer, pcmDevIds.at(0), paramData, paramSize);
            if (status) {
                PAL_ERR(LOG_TAG, "Failed to set mixer param, status = %d", status);
                goto exit;
            }
            break;
        }
        case PAL_PARAM_ID_UIEFFECT:
        {
            pal_effect_custom_payload_t *customPayload;
            pal_param_payload *param_payload = (pal_param_payload *)payload;
            effectPalPayload = (effect_pal_payload_t *)(param_payload->payload);
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                                                           rxAifBackEnds[0].second.data(),
                                                           tagId, &miid);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", tagId, status);
                break;
            } else {
                customPayload = (pal_effect_custom_payload_t *)effectPalPayload->payload;
                status = builder->payloadCustomParam(&paramData, &paramSize,
                            customPayload->data,
                            effectPalPayload->payloadSize - sizeof(uint32_t),
                            miid, customPayload->paramId);
                if (status != 0) {
                    PAL_ERR(LOG_TAG, "payloadCustomParam failed. status = %d",
                                status);
                    break;
                }
                status = SessionAlsaUtils::setMixerParameter(mixer,
                                                             pcmDevIds.at(0),
                                                             paramData,
                                                             paramSize);
                PAL_INFO(LOG_TAG, "mixer set param status=%d\n", status);
            }
            break;
        }
        case PAL_PARAM_ID_BT_A2DP_TWS_CONFIG:
        {
            pal_bt_tws_payload *tws_payload = (pal_bt_tws_payload *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                               rxAifBackEnds[0].second.data(), tagId, &miid);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", tagId, status);
                return status;
            }

            builder->payloadTWSConfig(&paramData, &paramSize, miid,
                    tws_payload->isTwsMonoModeOn, tws_payload->codecFormat);
            if (paramSize) {
                status = SessionAlsaUtils::setMixerParameter(mixer, device,
                                               paramData, paramSize);
                PAL_INFO(LOG_TAG, "mixer set tws config status=%d\n", status);
                free(paramData);
            }
            return 0;
        }
        default:
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "Unsupported param id %u status %d", param_id, status);
            goto exit;
    }

    if (!paramData) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to get payload status %d", status);
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG, "%pK - payload and %zu size", paramData , paramSize);

    if (param_id != PARAM_ID_DETECTION_ENGINE_RESET &&
        param_id != PAL_PARAM_ID_UIEFFECT) {
        if (!customPayloadSize) {
            customPayload = (uint8_t *)calloc(1, paramSize);
        } else {
            customPayload = (uint8_t *)realloc(customPayload, customPayloadSize + paramSize);
        }

        if (!customPayload) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "failed to allocate memory for custom payload");
            goto free_payload;
        }

        memcpy((uint8_t *)customPayload + customPayloadSize, paramData, paramSize);
        customPayloadSize += paramSize;
        PAL_INFO(LOG_TAG, "customPayloadSize = %zu", customPayloadSize);
    }

    PAL_DBG(LOG_TAG, "Exit. status %d", status);
free_payload :
    free(paramData);
exit:
    return status;
}

int SessionAlsaPcm::setECRef(Stream *s, std::shared_ptr<Device> rx_dev, bool is_enable)
{
    int status = 0;
    struct pal_stream_attributes sAttr;
    std::vector <std::shared_ptr<Device>> rxDeviceList;
    std::vector <std::string> backendNames;

    PAL_DBG(LOG_TAG, "Enter");
    if (!s) {
        PAL_ERR(LOG_TAG, "Invalid stream or rx device");
        return -EINVAL;
    }

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        return status;
    }

    if (sAttr.direction != PAL_AUDIO_INPUT) {
        PAL_ERR(LOG_TAG, "EC Ref cannot be set to output stream");
        return -EINVAL;
    }

    if (!is_enable) {
        if (ecRefDevId == PAL_DEVICE_OUT_MIN) {
            PAL_DBG(LOG_TAG, "EC ref not enabled, skip disabling");
            return 0;
        } else if (rx_dev && ecRefDevId != rx_dev->getSndDeviceId()) {
            PAL_DBG(LOG_TAG, "Invalid rx dev %d for disabling EC ref, "
                "rx dev %d already enabled", rx_dev->getSndDeviceId(), ecRefDevId);
            return 0;
        }
        status = SessionAlsaUtils::setECRefPath(mixer, pcmDevIds.at(0), "ZERO");
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to disable EC Ref, status %d", status);
            return status;
        }
        ecRefDevId = PAL_DEVICE_OUT_MIN;
    } else if (is_enable && rx_dev) {
        if (rx_dev && ecRefDevId == rx_dev->getSndDeviceId()) {
            PAL_DBG(LOG_TAG, "EC Ref already set for dev %d", ecRefDevId);
            return 0;
        }
        // TODO: handle EC Ref switch case also
        rxDeviceList.push_back(rx_dev);
        backendNames = rm->getBackEndNames(rxDeviceList);
        status = SessionAlsaUtils::setECRefPath(mixer, pcmDevIds.at(0),
            backendNames[0].c_str());
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to disable EC Ref, status %d", status);
            return status;
        }
        ecRefDevId = static_cast<pal_device_id_t>(rx_dev->getSndDeviceId());
    } else {
        PAL_ERR(LOG_TAG, "Invalid operation");
        return -EINVAL;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int SessionAlsaPcm::getParameters(Stream *s __unused, int tagId, uint32_t param_id, void **payload)
{
    int status = 0;
    uint8_t *ptr = NULL;
    uint8_t *config = NULL;
    uint8_t *payloadData = NULL;
    size_t payloadSize = 0;
    size_t configSize = 0;
    int device = pcmDevIds.at(0);
    uint32_t miid = 0;
    const char *control = "getParam";
    const char *stream = "PCM";
    struct mixer_ctl *ctl;
    std::ostringstream CntrlName;
    PAL_DBG(LOG_TAG, "Enter.");

    CntrlName << stream << pcmDevIds.at(0) << " " << control;
    ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
        status = -ENOENT;
        goto exit;
    }

    if (!rxAifBackEnds.empty()) { /** search in RX GKV */
        status = SessionAlsaUtils::getModuleInstanceId(mixer, device, rxAifBackEnds[0].second.data(),
                tagId, &miid);
        if (status) /** if not found, reset miid to 0 again */
            miid = 0;
    }

    if (!txAifBackEnds.empty()) { /** search in TX GKV */
        status = SessionAlsaUtils::getModuleInstanceId(mixer, device, txAifBackEnds[0].second.data(),
                tagId, &miid);
        if (status)
            miid = 0;
    }

    if (miid == 0) {
        PAL_ERR(LOG_TAG, "failed to look for module with tagID 0x%x", tagId);
        status = -EINVAL;
        goto exit;
    }


    switch (param_id) {
        case PAL_PARAM_ID_DIRECTION_OF_ARRIVAL:
        {
            configSize = sizeof(struct ffv_doa_tracking_monitor_t);
            builder->payloadDOAInfo(&payloadData, &payloadSize, miid);
            break;
        }
        default:
            status = EINVAL;
            PAL_ERR(LOG_TAG, "Unsupported param id %u status %d", param_id, status);
            goto exit;
    }

    status = mixer_ctl_set_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Set custom config failed, status = %d", status);
        goto exit;
    }

    status = mixer_ctl_get_array(ctl, payloadData, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Get custom config failed, status = %d", status);
        goto exit;
    }

    ptr = (uint8_t *)payloadData + sizeof(struct apm_module_param_data_t);
    config = (uint8_t *)calloc(1, configSize);
    if (!config) {
        PAL_ERR(LOG_TAG, "Failed to allocate memory for config");
        status = -ENOMEM;
        goto exit;
    }

    ar_mem_cpy(config, configSize, ptr, configSize);
    *payload = (void *)config;


exit:
    if (payloadData)
        free(payloadData);
    PAL_DBG(LOG_TAG, "Exit. status %d", status);
    return status;
}

int SessionAlsaPcm::registerCallBack(session_callback cb, void *cookie)
{
    sessionCb = cb;
    cbCookie = cookie;
    return 0;
}

int SessionAlsaPcm::getTimestamp(struct pal_session_time *stime)
{
    int status = 0;
    status = SessionAlsaUtils::getTimestamp(mixer, pcmDevIds, spr_miid, stime);
    if (0 != status) {
       PAL_ERR(LOG_TAG, "getTimestamp failed status = %d", status);
       return status;
    }
    return status;
}
int SessionAlsaPcm::drain(pal_drain_type_t type __unused)
{
    return 0;
}

int SessionAlsaPcm::flush()
{
    int status = 0;

    if (!pcm) {
        PAL_ERR(LOG_TAG, "Pcm is invalid");
        return -EINVAL;
    }
    PAL_VERBOSE(LOG_TAG,"Enter flush\n");
    if (pcm && isActive()) {
        status = pcm_stop(pcm);

        if (!status)
            mState = SESSION_FLUSHED;
    }

    PAL_VERBOSE(LOG_TAG,"status %d\n", status);

    return status;
}

bool SessionAlsaPcm::isActive()
{
    PAL_VERBOSE(LOG_TAG, "state = %d", mState);
    return mState == SESSION_STARTED;
}


void SessionAlsaPcm::adjustMmapPeriodCount(struct pcm_config *config, int32_t min_size_frames)
{
    int periodCountRequested = (min_size_frames + config->period_size - 1)
                               / config->period_size;
    int periodCount = SESSION_ALSA_MMAP_PERIOD_COUNT_MIN;

    PAL_VERBOSE(LOG_TAG, "original config.period_size = %d config.period_count = %d",
                config->period_size, config->period_count);

    while (periodCount < periodCountRequested &&
        (periodCount * 2) < SESSION_ALSA_MMAP_PERIOD_COUNT_MAX) {
        periodCount *= 2;
    }
    config->period_count = periodCount;

    PAL_VERBOSE(LOG_TAG, "requested config.period_count = %d",
                config->period_count);

}


int SessionAlsaPcm::createMmapBuffer(Stream *s, int32_t min_size_frames,
                                   struct pal_mmap_buffer *info)
{
    unsigned int offset1 = 0;
    unsigned int frames1 = 0;
    const char *step = "enter";
    uint32_t buffer_size;
    struct pcm_config config;
    struct pal_stream_attributes sAttr;
    int32_t status = 0;
    unsigned int pcm_flags = 0;
    const char *control = "getBufInfo";
    const char *stream = "PCM";
    struct mixer_ctl *ctl;
    std::ostringstream CntrlName;
    struct agm_buf_info buf_info;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "stream get attributes failed");
        return status;
    }

    if (info == NULL || !(min_size_frames > 0 && min_size_frames < INT32_MAX)) {
        PAL_ERR(LOG_TAG, "info = %p, min_size_frames = %d",
                info, min_size_frames);
        return -EINVAL;
    }

    if (!((sAttr.type == PAL_STREAM_ULTRA_LOW_LATENCY) &&
                    (sAttr.flags & PAL_STREAM_FLAG_MMAP_NO_IRQ))) {
         PAL_ERR(LOG_TAG, "called on stream type [%d] flags[%d]",
            sAttr.type, sAttr.flags);
         return -ENOSYS;
     }

    if (mState == SESSION_IDLE) {
        s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
        memset(&config, 0, sizeof(config));

        switch(sAttr.direction) {
            case PAL_AUDIO_INPUT:
                pcm_flags = PCM_IN | PCM_MMAP | PCM_NOIRQ | PCM_MONOTONIC;
                config.rate = sAttr.in_media_config.sample_rate;
                if (sAttr.in_media_config.bit_width == 32)
                    config.format = PCM_FORMAT_S32_LE;
                else if (sAttr.in_media_config.bit_width == 24)
                    config.format = PCM_FORMAT_S24_3LE;
                else if (sAttr.in_media_config.bit_width == 16)
                    config.format = PCM_FORMAT_S16_LE;
                config.channels = sAttr.in_media_config.ch_info.channels;
                config.period_size = SessionAlsaUtils::bytesToFrames(in_buf_size,
                    config.channels, config.format);
                config.period_count = in_buf_count;
                config.start_threshold = 0;
                config.stop_threshold = INT32_MAX;
                config.silence_threshold = 0;
                config.silence_size = 0;
                config.avail_min = config.period_size;
                break;
            case PAL_AUDIO_OUTPUT:
                pcm_flags = PCM_OUT | PCM_MMAP | PCM_NOIRQ | PCM_MONOTONIC;
                config.rate = sAttr.out_media_config.sample_rate;
                if (sAttr.out_media_config.bit_width == 32)
                    config.format = PCM_FORMAT_S32_LE;
                else if (sAttr.out_media_config.bit_width == 24)
                    config.format = PCM_FORMAT_S24_3LE;
                else if (sAttr.out_media_config.bit_width == 16)
                    config.format = PCM_FORMAT_S16_LE;
                config.channels = sAttr.out_media_config.ch_info.channels;
                config.period_size = SessionAlsaUtils::bytesToFrames(out_buf_size,
                    config.channels, config.format);
                config.period_count = out_buf_count;
                config.start_threshold = config.period_size * 8;
                config.stop_threshold = INT32_MAX;
                config.silence_threshold = 0;
                config.silence_size = 0;
                config.avail_min = config.period_size;
                break;
            case PAL_AUDIO_INPUT | PAL_AUDIO_OUTPUT:
                return -EINVAL;
                break;
        }

        this->adjustMmapPeriodCount(&config, min_size_frames);

        PAL_DBG(LOG_TAG, "Opening PCM device card_id(%d) device_id(%d), channels %d",
                rm->getSndCard(), pcmDevIds.at(0), config.channels);

        pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0),
                             pcm_flags, &config);
        if (!pcm) {
            PAL_ERR(LOG_TAG, "pcm open failed");
            step = "open";
            status = -EINVAL;
            goto exit;
        }

        if (!pcm_is_ready(pcm)) {
            PAL_ERR(LOG_TAG, "pcm open not ready");
            pcm = nullptr;
            step = "open";
            status = -EINVAL;
            goto exit;
        }

         status = pcm_mmap_begin(pcm, &info->buffer, &offset1, &frames1);
         if (status < 0)  {
             step = "begin";
             goto exit;
         }

         info->flags = 0;
         info->buffer_size_frames = pcm_get_buffer_size(pcm);
         buffer_size = pcm_frames_to_bytes(pcm, info->buffer_size_frames);
         info->burst_size_frames = config.period_size;


        CntrlName << stream << pcmDevIds.at(0) << " " << control;
        ctl = mixer_get_ctl_by_name(mixer, CntrlName.str().data());
        if (!ctl) {
            PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", CntrlName.str().data());
            status = -ENOENT;
            goto exit;
        }

        //TODO call a mixer control to get the fd.
        memset(&buf_info, 0, sizeof(buf_info));
        status = mixer_ctl_get_array(ctl, (void *)&buf_info, sizeof(struct agm_buf_info));
        if (status < 0) {
            // Fall back to non exclusive mode
            info->fd = pcm_get_poll_fd(pcm);
        } else {
            info->fd = buf_info.data_buf_fd;
            //mmap_shared_memory_fd = buf_info->shared_memory_fd; // for closing later

            PAL_VERBOSE(LOG_TAG, "opened shared_memory_fd = %d",
                info->fd);

            if (buf_info.data_buf_size < buffer_size) {
                status = -EINVAL;
                step = "mmap";
                goto exit;
            }
            info->flags |= PAL_MMMAP_BUFF_FLAGS_APP_SHAREABLE;
        }
        memset(info->buffer, 0, pcm_frames_to_bytes(pcm,info->buffer_size_frames));

        status = pcm_mmap_commit(pcm, 0, SESSION_ALSA_MMAP_PERIOD_SIZE);
        if (status < 0) {
            step = "commit";
            goto exit;
        }

        //TODO
        //out->mmap_time_offset_nanos = get_mmap_out_time_offset();
        PAL_DBG(LOG_TAG, "got mmap buffer address %pK info->buffer_size_frames %d",
                info->buffer, info->buffer_size_frames);
        mState = SESSION_OPENED;
    }

 exit:
     if (status < 0) {
         if (pcm == NULL) {
             PAL_ERR(LOG_TAG,"%s - %d",step, status);
         } else {
             //status = -errno;
             PAL_ERR(LOG_TAG,"%s - %d",step, status);
             if (pcm) {
                 pcm_close(pcm);
                 pcm = NULL;
             }
         }
     } else {
         status = 0;
     }
     return status;
 }

 int SessionAlsaPcm::GetMmapPosition(Stream *s, struct pal_mmap_position *position)
 {
    int status = 0;
    struct pal_stream_attributes sAttr;
    struct timespec ts = { 0, 0 };

    PAL_DBG(LOG_TAG,"enter");

    if (pcm == NULL) {
        return -ENOSYS;
    }

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

     if (position == NULL) {
         return -EINVAL;
     }

     if (!((sAttr.type == PAL_STREAM_ULTRA_LOW_LATENCY) &&
                    (sAttr.flags & PAL_STREAM_FLAG_MMAP_NO_IRQ))) {
         PAL_ERR(LOG_TAG, "called on stream type [%d] flags[%d]",
            sAttr.type, sAttr.flags);
         return -ENOSYS;
     }

     status = pcm_mmap_get_hw_ptr(pcm, (unsigned int *)&position->position_frames, &ts);
     if (status < 0) {
         status = -errno;
         PAL_ERR(LOG_TAG, "%d", status);
         return status;
     }
     position->time_nanoseconds = ts.tv_sec*1000000000LL + ts.tv_nsec
             /*+ out->mmap_time_offset_nanos*/;
     return status;
 }
