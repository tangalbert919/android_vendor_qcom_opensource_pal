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
#include <agm/agm_api.h>
#include <sstream>
#include <string>

SessionAlsaPcm::SessionAlsaPcm(std::shared_ptr<ResourceManager> Rm)
{
   rm = Rm;
   builder = new PayloadBuilder();
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

    pcmDevIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction);
    if(sAttr.direction == QAL_AUDIO_INPUT) {
        for (int i = 0; i < pcmDevIds.size(); i++)
            pcmDevIds[i] = 101;
    } else {
        for (int i = 0; i < pcmDevIds.size(); i++)
            pcmDevIds[i] = 100;
    }
    aifBackEnds = rm->getBackEndNames(associatedDevices);
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error");
        return status;
    }
    status = SessionAlsaUtils::open(s, rm, pcmDevIds, aifBackEnds);
    if (status) {
        QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
        rm->freeFrontEndIds(pcmDevIds);
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
            //TODO: how to get the id '0'
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
            //TODO: how to get the id '0'
            calCntrlName<<stream<<pcmDevIds.at(0)<<" "<<setCalibrationControl;
            ctl = mixer_get_ctl_by_name(mixer, calCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", calCntrlName.str().data());
                return -ENOENT;
            }
            ckv_size = ckv.size()*sizeof(struct agm_key_value);
            //TODO make struct mixer and struct pcm as class private variables.
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

    if (sAttr.direction == QAL_AUDIO_INPUT) {
        pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_IN, &config);
    } else {
        pcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), PCM_OUT, &config);
    }

    if (!pcm) {
        QAL_ERR(LOG_TAG, "pcm open failed");
        return -EINVAL;
    }

    if (!pcm_is_ready(pcm)) {
        QAL_ERR(LOG_TAG, "pcm open not ready");
        return -EINVAL;
    }

    status = pcm_start(pcm);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_start failed %d", status);
    }
    return status;
}

int SessionAlsaPcm::stop(Stream * s)
{
    int status = 0;
    status = pcm_stop(pcm);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_stop failed %d", status);
    }
    return status;
}

int SessionAlsaPcm::close(Stream * s)
{
    int status = 0;
    status = pcm_close(pcm);
    if (status) {
        QAL_ERR(LOG_TAG, "pcm_stop failed %d", status);
        goto exit;
    }
    pcm = NULL;
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
    return 0;
}

int SessionAlsaPcm::getParameters(Stream *s, int tagId, uint32_t param_id, void **payload)
{
    return 0;
}
