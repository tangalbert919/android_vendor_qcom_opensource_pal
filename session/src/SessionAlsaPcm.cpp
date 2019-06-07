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


 #define LOG_TAG "SessionAlsaPcm"

#include "SessionAlsaPcm.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include <agm_api.h>
#include <sstream>
#include <string>
#include "detection_cmn_api.h"
#include "audio_dam_buffer_api.h"

SessionAlsaPcm::SessionAlsaPcm(std::shared_ptr<ResourceManager> Rm)
{
   rm = Rm;
   builder = new PayloadBuilder();
   customPayload = NULL;
   customPayloadSize = 0;
}

SessionAlsaPcm::~SessionAlsaPcm()
{
   delete builder;

}


int SessionAlsaPcm::prepare(Stream * s)
{
   return 0;
}

int SessionAlsaPcm::open(Stream * s)
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

    if(sAttr.direction == QAL_AUDIO_INPUT) {
        pcmDevIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction, 0);
    } else if (sAttr.direction == QAL_AUDIO_OUTPUT) {
        pcmDevIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction, 0);
    } else {
        pcmDevRxIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction, RXLOOPBACK);
        pcmDevTxIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction, TXLOOPBACK);
    }
    aifBackEnds = rm->getBackEndNames(associatedDevices);
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error");
        return status;
    }
    switch(sAttr.direction) {
        case QAL_AUDIO_INPUT:
        case QAL_AUDIO_OUTPUT:
            status = SessionAlsaUtils::open(s, rm, pcmDevIds, aifBackEnds);
            if (status) {
                QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
                rm->freeFrontEndIds(pcmDevIds, sAttr.type, sAttr.direction, 0);
            }
            break;
        case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
            status = SessionAlsaUtils::open(s, rm, pcmDevRxIds, pcmDevTxIds, aifBackEnds);
            if (status) {
                QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
                rm->freeFrontEndIds(pcmDevRxIds, sAttr.type, sAttr.direction, RXLOOPBACK);
                rm->freeFrontEndIds(pcmDevTxIds, sAttr.type, sAttr.direction, TXLOOPBACK);
            }
            break;
        default:
            QAL_ERR(LOG_TAG,"unsupported direction");
            break;
    }
    status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIds.at(0), aifBackEnds[0].data(), false, STREAM_SPR, &spr_miid);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", STREAM_SPR, status);
        return status;
    }
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
    int tkv_size = 0;
    int ckv_size = 0;

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
            tagCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setParamTagControl;
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
            break;
            //todo calibration
        case CALIBRATION:
            status = builder->populateCalKeyVector(s, ckv, tag);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the calibration data\n", __func__);
                goto exit;
            }


            if (ckv.size() == 0) {
                status = -EINVAL;
                goto exit;
            }


            calConfig = (struct agm_cal_config*)malloc (sizeof(struct agm_cal_config) +
                            (ckv.size() * sizeof(agm_key_value)));

            if(!calConfig) {
                status = -EINVAL;
                goto exit;
            }

            status = SessionAlsaUtils::getCalMetadata(ckv, calConfig);
            calCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setCalibrationControl;
            ctl = mixer_get_ctl_by_name(mixer, calCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", calCntrlName.str().data());
                return -ENOENT;
            }
            ckv_size = ckv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, calConfig, sizeof(struct agm_cal_config) + ckv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            ckv.clear();
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
            goto exit;
    }

exit:
    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionAlsaPcm::getConfig(Stream * s)
{
   return 0;
}
*/
int SessionAlsaPcm::start(Stream * s)
{
    struct pcm_config config;
    struct qal_stream_attributes sAttr;
    int32_t status = 0;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct qal_device dAttr;

    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
    memset(&config, 0, sizeof(config));

    if (sAttr.direction == QAL_AUDIO_INPUT) {
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

    } else {
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
    }
    config.start_threshold = 0;
    config.stop_threshold = 0;
    config.silence_threshold = 0;

    switch(sAttr.direction) {
        case QAL_AUDIO_INPUT:
            if (sAttr.type == QAL_STREAM_VOICE_UI) {
                SessionAlsaUtils::setMixerParameter(mixer, pcmDevIds.at(0), false, customPayload, customPayloadSize);
            }
 
            pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_IN, &config);
            if (!pcm) {
                QAL_ERR(LOG_TAG, "pcm open failed");
                return -EINVAL;
            }

            if (!pcm_is_ready(pcm)) {
                QAL_ERR(LOG_TAG, "pcm open not ready");
                return -EINVAL;
            }
            break;
        case QAL_AUDIO_OUTPUT:
            pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_OUT, &config);
            if (!pcm) {
                QAL_ERR(LOG_TAG, "pcm open failed");
                return -EINVAL;
            }

            if (!pcm_is_ready(pcm)) {
                QAL_ERR(LOG_TAG, "pcm open not ready");
                return -EINVAL;
            }
            break;
        case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
            pcmRx = pcm_open(rm->getSndCard(), pcmDevRxIds.at(0), PCM_OUT, &config);
            if (!pcmRx) {
                QAL_ERR(LOG_TAG, "pcm-rx open failed");
                return -EINVAL;
            }

            if (!pcm_is_ready(pcmRx)) {
                QAL_ERR(LOG_TAG, "pcm-rx open not ready");
                return -EINVAL;
            }
            status = pcm_start(pcmRx);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_start rx failed %d", status);
            }
            pcmTx = pcm_open(rm->getSndCard(), pcmDevTxIds.at(0), PCM_IN, &config);
            if (!pcmTx) {
                QAL_ERR(LOG_TAG, "pcm-tx open failed");
                return -EINVAL;
            }

            if (!pcm_is_ready(pcmTx)) {
                QAL_ERR(LOG_TAG, "pcm-tx open not ready");
                return -EINVAL;
            }
            status = pcm_start(pcmTx);
            if (status) {
               QAL_ERR(LOG_TAG, "pcm_start tx failed %d", status);
            }
            break;
    }

    if (sAttr.type == QAL_STREAM_VOICE_UI) {
        SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIds.at(0), aifBackEnds[0].data(), false, DEVICE_SVA, true);
    }

    checkAndConfigConcurrency(s);

    switch(sAttr.direction) {
        case QAL_AUDIO_INPUT:
        case QAL_AUDIO_OUTPUT:
            status = s->getAssociatedDevices(associatedDevices);
            if(0 != status) {
                QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
                return status;
            }
            for (int i = 0; i < associatedDevices.size();i++) {
                status = associatedDevices[i]->getDeviceAtrributes(&dAttr);
                if(0 != status) {
                    QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
                    return status;
                }
                switch (dAttr.config.sample_rate) {
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
                }
            }
            //status = pcm_prepare(pcm);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_prepare failed %d", status);
            }
            status = pcm_start(pcm);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_start failed %d", status);
            }
            break;
        case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
            break;
    }

    if (sAttr.type == QAL_STREAM_VOICE_UI) {
        threadHandler = std::thread(SessionAlsaPcm::eventWaitThreadLoop, (void *)mixer, this);
        if (!threadHandler.joinable()) {
            QAL_ERR(LOG_TAG, "Failed to create threadHandler");
            status = -EINVAL;
        }
    }
    return status;
}

int SessionAlsaPcm::stop(Stream * s)
{
    int status = 0;
    struct qal_stream_attributes sAttr;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }
    switch(sAttr.direction) {
        case QAL_AUDIO_INPUT:
        case QAL_AUDIO_OUTPUT:
            status = pcm_stop(pcm);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_stop failed %d", status);
            }
            break;
        case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
            status = pcm_stop(pcmRx);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_stop - rx failed %d", status);
            }
            status = pcm_stop(pcmTx);
            if (status) {
               QAL_ERR(LOG_TAG, "pcm_stop - tx failed %d", status);
            }
            break;
    }

    if (sAttr.type == QAL_STREAM_VOICE_UI) {
        threadHandler.join();
        QAL_DBG(LOG_TAG, "threadHandler joined");
    }
    return status;
}

int SessionAlsaPcm::close(Stream * s)
{
    int status = 0;
    struct qal_stream_attributes sAttr;
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }
    switch(sAttr.direction) {
        case QAL_AUDIO_INPUT:
        case QAL_AUDIO_OUTPUT:
            status = pcm_close(pcm);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_close failed %d", status);
            }
            rm->freeFrontEndIds(pcmDevIds, sAttr.type, sAttr.direction, 0);
            pcm = NULL;
            break;
        case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
            status = pcm_close(pcmRx);
            if (status) {
                QAL_ERR(LOG_TAG, "pcm_close - rx failed %d", status);
            }
            status = pcm_close(pcmTx);
            if (status) {
               QAL_ERR(LOG_TAG, "pcm_close - tx failed %d", status);
            }
            pcmRx = NULL;
            pcmTx = NULL;
            break;
    }

    if (customPayload) {
        free(customPayload);
        customPayload = NULL;
        customPayloadSize = 0;
    }
exit:
    return status;
}

int SessionAlsaPcm::read(Stream *s, int tag, struct qal_buffer *buf, int * size)
{
    int status = 0, bytesRead = 0, bytesToRead = 0, offset = 0, pcmReadSize = 0;
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
        status =  pcm_read(pcm, data,  pcmReadSize);
        if ((0 != status) || (pcmReadSize == 0)) {
            QAL_ERR(LOG_TAG,"%s: Failed to read data %d bytes read %d", __func__, status, pcmReadSize);
            break;
        }
        bytesRead += pcmReadSize;
    }
exit:
    *size = bytesRead;
    QAL_DBG(LOG_TAG,"%s: exit bytesRead:%d status:%d ", __func__, bytesRead, status);
    return status;
}

int SessionAlsaPcm::write(Stream *s, int tag, struct qal_buffer *buf, int * size,
                          int flag)
{
    int status = 0, bytesWritten = 0, bytesRemaining = 0, offset = 0;
    uint32_t sizeWritten = 0;
    QAL_DBG(LOG_TAG,"%s: enter buf:%p tag:%d flag:%d", __func__, buf, tag, flag);

    void *data = nullptr;
    struct gsl_buff gslBuff;
    gslBuff.timestamp = (uint64_t) buf->ts;

    bytesRemaining = buf->size;

    while ((bytesRemaining / out_buf_size) > 1) {
        offset = bytesWritten + buf->offset;
        data = buf->buffer;
        data = static_cast<char *>(data) + offset;
        sizeWritten = out_buf_size;  //initialize 0
        status = pcm_write(pcm, data, sizeWritten);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: Failed to write the data to gsl", __func__);
            return status;
        }
        bytesWritten += sizeWritten;
        bytesRemaining -= sizeWritten;
    }
    offset = bytesWritten + buf->offset;
    sizeWritten = bytesRemaining;
    data = buf->buffer;
    data = static_cast<char *>(data) + offset;
    status = pcm_write(pcm, data, sizeWritten);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"Error! pcm_write failed");
        return status;
    }
    bytesWritten += sizeWritten;
    *size = bytesWritten;
    return status;
}

int SessionAlsaPcm::readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize,
                                   int flag)
{
    return 0;
}
int SessionAlsaPcm::writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize,
                                    int flag)
{
    return 0;
}

int SessionAlsaPcm::setParameters(Stream *s, int tagId, uint32_t param_id, void *payload)
{
    int status = 0;
    int device = pcmDevIds.at(0);
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    uint32_t miid = 0;

    QAL_DBG(LOG_TAG, "Enter.");
    switch (param_id) {
        case PARAM_ID_DETECTION_ENGINE_SOUND_MODEL:
        {
            struct qal_st_sound_model *pSoundModel = NULL;
            pSoundModel = (struct qal_st_sound_model *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_SVA, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVASoundModel(&paramData, &paramSize, miid, pSoundModel);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP:
        {
            struct detection_engine_config_voice_wakeup *pWakeUpConfig = NULL;
            pWakeUpConfig = (struct detection_engine_config_voice_wakeup *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_SVA, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAWakeUpConfig(&paramData, &paramSize, miid, pWakeUpConfig);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG:
        {
            // Restore stream pointer for event callback
            cookie = (void *)s;
            struct detection_engine_generic_event_cfg *pEventConfig = NULL;
            pEventConfig = (struct detection_engine_generic_event_cfg *)payload;
            // set custom config for detection event
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_SVA, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAEventConfig(&paramData, &paramSize, miid, pEventConfig);
            break;
        }
        case PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG:
        {
            struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufConfig = NULL;
            pWakeUpBufConfig = (struct detection_engine_voice_wakeup_buffer_config *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_SVA, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAWakeUpBufferConfig(&paramData, &paramSize, miid, pWakeUpBufConfig);
            break;
        }
        case PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION:
        {
            struct audio_dam_downstream_setup_duration *pSetupDuration = NULL;
            pSetupDuration = (struct audio_dam_downstream_setup_duration *)payload;
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_ADAM, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_ADAM, status);
                goto exit;
            }
            builder->payloadSVAStreamSetupDuration(&paramData, &paramSize, miid, pSetupDuration);
            break;
        }
        case PARAM_ID_DETECTION_ENGINE_RESET:
        {
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device, aifBackEnds[0].data(), false, DEVICE_SVA, &miid);
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to get tage info %x, status = %d", DEVICE_SVA, status);
                goto exit;
            }
            builder->payloadSVAEngineReset(&paramData, &paramSize, miid);
            break;
        }
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

    if (param_id != PARAM_ID_DETECTION_ENGINE_RESET) {
        if (!customPayloadSize) {
            customPayload = (uint8_t *)calloc(1, paramSize);
        } else {
            customPayload = (uint8_t *)realloc(customPayload, customPayloadSize + paramSize);
        }

        if (!customPayload) {
            status = -ENOMEM;
            QAL_ERR(LOG_TAG, "failed to allocate memory for custom payload");
            goto free_payload;
        }

        memcpy((uint8_t *)customPayload + customPayloadSize, paramData, paramSize);
        customPayloadSize += paramSize;
        QAL_INFO(LOG_TAG, "customPayloadSize = %d", customPayloadSize);
    }

    QAL_DBG(LOG_TAG, "Exit. status %d", status);
free_payload :
    free(paramData);
exit:
    return status;
}

void SessionAlsaPcm::eventWaitThreadLoop(void *context, SessionAlsaPcm *session)
{
    struct mixer *mixer = (struct mixer *)context;
    int ret = 0;
    struct snd_ctl_event mixer_event = {0};

    QAL_VERBOSE(LOG_TAG, "subscribing for event");
    mixer_subscribe_events(mixer, 1);

    while (1) {
        QAL_VERBOSE(LOG_TAG, "going to wait for event");
        ret = mixer_wait_event(mixer, -1);
        if (ret < 0) {
            QAL_DBG(LOG_TAG, "mixer_wait_event err! ret = %d", ret);
        } else if (ret > 0) {
            ret = mixer_read_event(mixer, &mixer_event);
            if (ret >= 0) {
                QAL_INFO(LOG_TAG, "Event Received %s", mixer_event.data.elem.id.name);
                ret = session->handleMixerEvent(mixer, (char *)mixer_event.data.elem.id.name);
            } else {
                QAL_DBG(LOG_TAG, "mixer_read failed, ret = %d", ret);
            }
            if (!ret)
                break;
            //done = 1;
        }
    }
    mixer_subscribe_events(mixer, 0);
}

int SessionAlsaPcm::handleMixerEvent(struct mixer *mixer, char *mixer_str)
{
    struct mixer_ctl *ctl;
    char *buf = NULL;
    unsigned int num_values;
    int i, ret = 0;
    struct agm_event_cb_params *params;
    qal_stream_callback callBack;
    Stream *s;

    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    num_values = mixer_ctl_get_num_values(ctl);
    QAL_VERBOSE(LOG_TAG, "num_values: %d", num_values);
    buf = (char *)calloc(1, num_values);

    ret = mixer_ctl_get_array(ctl, buf, num_values);
    if (ret < 0) {
        QAL_ERR(LOG_TAG, "Failed to mixer_ctl_get_array");
        free(buf);
        return ret;
    }

    params = (struct agm_event_cb_params *)buf;
    QAL_DBG(LOG_TAG, "params.source_module_id %x", params->source_module_id);
    QAL_DBG(LOG_TAG, "params.event_id %d", params->event_id);
    QAL_DBG(LOG_TAG, "params.event_payload_size %x", params->event_payload_size);

    s = (Stream *)cookie;
    s->getCallBack(&callBack);
    uint32_t event_id = params->event_id;
    uint32_t *event_data = (uint32_t *)(params->event_payload);
    qal_stream_handle_t *stream_handle = static_cast<void*>(s);
    callBack(stream_handle, event_id, event_data, NULL);
    return ret;
}

void SessionAlsaPcm::checkAndConfigConcurrency(Stream *s)
{
    int32_t status = 0;
    std::shared_ptr<Device> rxDevice = nullptr;
    std::shared_ptr<Device> txDevice = nullptr;
    struct qal_stream_attributes sAttr;
    std::vector <Stream *> activeStreams;
    qal_stream_type_t txStreamType;
    std::vector <std::shared_ptr<Device>> activeDevices;
    std::vector <std::shared_ptr<Device>> deviceList;

    // get stream attributes
    status = s->getStreamAttributes(&sAttr);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"stream get attributes failed");
        return;
    }

    // get associated device list
    status = s->getAssociatedDevices(deviceList);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to get associated device, status %d", status);
        return;
    }

    // get all active devices from rm and
    // determine Rx and Tx for concurrency usecase
    rm->getActiveDevices(activeDevices);
    for (int i = 0; i < activeDevices.size(); i++) {
        int deviceId = activeDevices[i]->getDeviceId();
        if (deviceId == QAL_DEVICE_OUT_SPEAKER &&
            sAttr.direction == QAL_AUDIO_INPUT) {
            rxDevice = activeDevices[i];
            for (int j = 0; j < deviceList.size(); j++) {
                std::shared_ptr<Device> dev = deviceList[j];
                if (dev->getDeviceId() >= QAL_DEVICE_IN_HANDSET_MIC &&
                    dev->getDeviceId() <= QAL_DEVICE_IN_TRI_MIC)
                    txDevice = dev;
            }
        }

        if (deviceId >= QAL_DEVICE_IN_HANDSET_MIC &&
            deviceId <= QAL_DEVICE_IN_TRI_MIC &&
            sAttr.direction == QAL_AUDIO_OUTPUT) {
            txDevice = activeDevices[i];
            for (int j = 0; j < deviceList.size(); j++) {
                std::shared_ptr<Device> dev = deviceList[j];
                if (dev->getDeviceId() == QAL_DEVICE_OUT_SPEAKER) {
                    rxDevice = dev;
                    break;
                }
            }
        }
    }

    if (!rxDevice || !txDevice) {
        QAL_ERR(LOG_TAG, "No need to handle for concurrency");
        return;
    }

    QAL_DBG(LOG_TAG, "rx device %d, tx device %d", rxDevice->getDeviceId(), txDevice->getDeviceId());
    // determine concurrency usecase
    for (int i = 0; i < deviceList.size(); i++) {
        std::shared_ptr<Device> dev = deviceList[i];
        if (dev == rxDevice) {
            rm->getActiveStream(txDevice, activeStreams);
            for (int j = 0; j < activeStreams.size(); j++) {
                activeStreams[j]->getStreamType(&txStreamType);
            }
        }
        else if (dev == txDevice)
            s->getStreamType(&txStreamType);
        else {
            QAL_ERR(LOG_TAG, "Concurrency usecase exists, not related to current stream");
            return;
        }
    }
    QAL_DBG(LOG_TAG, "tx stream type = %d", txStreamType);
    // TODO: use table to map types/devices to key values
    if (txStreamType == QAL_STREAM_VOICE_UI) {
        status = SessionAlsaUtils::setECRefPath(mixer, 1, false, "CODEC_DMA-LPAIF_WSA-RX-0");
        if (status)
            QAL_ERR(LOG_TAG, "Failed to set EC ref path");
    }
}

int SessionAlsaPcm::getParameters(Stream *s, int tagId, uint32_t param_id, void **payload)
{
    return 0;
}

int SessionAlsaPcm::registerCallBack(session_callback cb, void *cookie)
{
    return 0;
}

int SessionAlsaPcm::getTimestamp(struct qal_session_time *stime)
{
    int status = 0;
    status = SessionAlsaUtils::getTimestamp(mixer, false, pcmDevIds, spr_miid, stime);
    if (0 != status) {
       QAL_ERR(LOG_TAG, "getTimestamp failed status = %d", status);
       return status;
    }
    return status;
}
int SessionAlsaPcm::drain(qal_drain_type_t type)
{
    return 0;
}

