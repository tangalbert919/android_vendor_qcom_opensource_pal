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

#define LOG_TAG "SessionAlsaCompress"

#include "SessionAlsaCompress.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include <agm_api.h>
#include <sstream>
#include <mutex>
#include <fstream>


void SessionAlsaCompress::getSndCodecParam(struct snd_codec &codec, struct qal_stream_attributes &sAttr)
{
    struct qal_media_config *config = &sAttr.out_media_config;

    codec.id = getSndCodecId(config->aud_fmt_id);
    codec.ch_in = config->ch_info->channels;
    codec.ch_out = codec.ch_in;
    codec.sample_rate = config->sample_rate;
    codec.bit_rate = config->bit_width;
}

int SessionAlsaCompress::getSndCodecId(qal_audio_fmt_t fmt)
{
    int id = -1;

    switch (fmt) {
        case QAL_AUDIO_FMT_MP3:
            id = SND_AUDIOCODEC_MP3;
            break;
#ifdef SND_COMPRESS_DEC_HDR
        case QAL_AUDIO_FMT_AAC:
        case QAL_AUDIO_FMT_AAC_ADTS:
        case QAL_AUDIO_FMT_AAC_ADIF:
        case QAL_AUDIO_FMT_AAC_LATM:
            id = SND_AUDIOCODEC_AAC;
            break;
        case QAL_AUDIO_FMT_WMA_STD:
            id = SND_AUDIOCODEC_WMA;
            break;
        case QAL_AUDIO_FMT_DEFAULT_PCM:
            id = SND_AUDIOCODEC_PCM;
            break;
        case QAL_AUDIO_FMT_ALAC:
            id = SND_AUDIOCODEC_ALAC;
            break;
        case QAL_AUDIO_FMT_APE:
            id = SND_AUDIOCODEC_APE;
            break;
        case QAL_AUDIO_FMT_WMA_PRO:
            id = SND_AUDIOCODEC_WMA_PRO;
            break;
       case QAL_AUDIO_FMT_FLAC:
       case QAL_AUDIO_FMT_FLAC_OGG:
            id = SND_AUDIOCODEC_FLAC;
            break;
#endif
    }

    return id;
}

void SessionAlsaCompress::offloadThreadLoop(SessionAlsaCompress* compressObj)
{
    std::shared_ptr<offload_msg> msg;
    uint32_t event_id;
    int ret = 0;
    bool is_drain_called = false;
    std::unique_lock<std::mutex> lock(compressObj->cv_mutex_);

    while (1) {
        if (compressObj->msg_queue_.empty())
            compressObj->cv_.wait(lock);  /* wait for incoming requests */

        if (!compressObj->msg_queue_.empty()) {
            msg = compressObj->msg_queue_.front();
            compressObj->msg_queue_.pop();
            lock.unlock();

            if (msg->cmd == OFFLOAD_CMD_EXIT)
                break; // exit the thread

            if (msg->cmd == OFFLOAD_CMD_WAIT_FOR_BUFFER) {
                QAL_VERBOSE(LOG_TAG, "calling compress_wait");
                compress_wait(compressObj->compress, -1);
                QAL_VERBOSE(LOG_TAG, "out of compress_wait");
                event_id = QAL_STREAM_CBK_EVENT_WRITE_READY;
            } else if (msg->cmd == OFFLOAD_CMD_DRAIN) {
                if (!is_drain_called && compressObj->playback_started) {
                   QAL_ERR(LOG_TAG, "calling compress_drain");
                   ret = compress_drain(compressObj->compress);
                   is_drain_called = false;
                   QAL_ERR(LOG_TAG, "out of compress_drain");
                }
                /*TODO: check for ret code and handle SSR */
                event_id = QAL_STREAM_CBK_EVENT_DRAIN_READY;
            } else if (msg->cmd == OFFLOAD_CMD_PARTIAL_DRAIN) {
                QAL_ERR(LOG_TAG, "calling partial compress_drain");
                //return compress_partial_drain(compressObj->compress);
                ret = compress_drain(compressObj->compress);
                is_drain_called = true;
                QAL_ERR(LOG_TAG, "out of partial compress_drain");
                event_id = QAL_STREAM_CBK_EVENT_DRAIN_READY;
            }
            if (compressObj->sessionCb)
                compressObj->sessionCb(compressObj->cbCookie, event_id, NULL);

            lock.lock();
        }

    }
    QAL_DBG(LOG_TAG, "exit offloadThreadLoop");
}

SessionAlsaCompress::SessionAlsaCompress(std::shared_ptr<ResourceManager> Rm)
{
    rm = Rm;
    builder = new PayloadBuilder();

    /** set default snd codec params */
    codec.id = getSndCodecId(QAL_AUDIO_FMT_DEFAULT_PCM);
    codec.ch_in = 2;
    codec.ch_out = codec.ch_in;
    codec.sample_rate = 48000;
    codec.bit_rate = 16;

    compress = NULL;
    sessionCb = NULL;
    this->cbCookie = NULL;
    playback_started = false;
}

SessionAlsaCompress::~SessionAlsaCompress()
{
    delete builder;
}

int SessionAlsaCompress::open(Stream * s)
{
    int status = -EINVAL;
    struct qal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::vector<std::pair<int32_t, std::string>> emptyBackEnds;

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        return status;
    }

    ioMode = sAttr.flags & QAL_STREAM_FLAG_NON_BLOCKING_MASK;
    if (!ioMode) {
        QAL_ERR(LOG_TAG, "IO mode 0x%x not supported", ioMode);
        return -EINVAL;
    }
    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
        return status;
    }

    compressDevIds = rm->allocateFrontEndIds(sAttr, 0);
    for (int i = 0; i < compressDevIds.size(); i++) {
       //compressDevIds[i] = 5;
    QAL_DBG(LOG_TAG, "devid size %d, compressDevIds[%d] %d", compressDevIds.size(), i, compressDevIds[i]);
    }
    rm->getBackEndNames(associatedDevices, rxAifBackEnds, emptyBackEnds);
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error");
        return status;
    }
    status = SessionAlsaUtils::open(s, rm, compressDevIds, rxAifBackEnds);
    if (status) {
        QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
        rm->freeFrontEndIds(compressDevIds, sAttr, 0);
    }
    audio_fmt = sAttr.out_media_config.aud_fmt_id;

    status = SessionAlsaUtils::getModuleInstanceId(mixer, (compressDevIds.at(0)),
            rxAifBackEnds[0].second.data(), true, STREAM_SPR, &spr_miid);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", STREAM_SPR, status);
        return status;
    }
    return status;
}

int SessionAlsaCompress::disconnectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToDisconnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    struct qal_device dAttr;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEndsToDisconnect;
    std::vector<std::pair<int32_t, std::string>> txAifBackEndsToDisconnect;
    int32_t status = 0;

    deviceList.push_back(deviceToDisconnect);
    rm->getBackEndNames(deviceList, rxAifBackEndsToDisconnect,
            txAifBackEndsToDisconnect);
    deviceToDisconnect->getDeviceAtrributes(&dAttr);

    if (!rxAifBackEndsToDisconnect.empty())
        status = SessionAlsaUtils::disconnectSessionDevice(streamHandle, streamType, rm,
            dAttr, compressDevIds, rxAifBackEndsToDisconnect);

    if (!txAifBackEndsToDisconnect.empty())
        status = SessionAlsaUtils::disconnectSessionDevice(streamHandle, streamType, rm,
            dAttr, compressDevIds, txAifBackEndsToDisconnect);

    return status;
}

int SessionAlsaCompress::connectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect)
{
    std::vector<std::shared_ptr<Device>> deviceList;
    struct qal_device dAttr;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEndsToConnect;
    std::vector<std::pair<int32_t, std::string>> txAifBackEndsToConnect;
    int32_t status = 0;

    deviceList.push_back(deviceToConnect);
    rm->getBackEndNames(deviceList, rxAifBackEndsToConnect,
            txAifBackEndsToConnect);
    deviceToConnect->getDeviceAtrributes(&dAttr);

    if (!rxAifBackEndsToConnect.empty())
        status = SessionAlsaUtils::connectSessionDevice(streamHandle, streamType, rm,
            dAttr, compressDevIds, rxAifBackEndsToConnect);

    if (!txAifBackEndsToConnect.empty())
        status = SessionAlsaUtils::connectSessionDevice(streamHandle, streamType, rm,
            dAttr, compressDevIds, txAifBackEndsToConnect);

    return status;
}
 
int SessionAlsaCompress::prepare(Stream * s)
{
   return 0;
}

int SessionAlsaCompress::setTKV(Stream * s, configType type, effect_qal_payload_t *effectPayload)
{
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig = nullptr;
    const char *setParamTagControl = "setParamTag";
    const char *stream = "COMPRESS";
    struct mixer_ctl *ctl;
    std::ostringstream tagCntrlName;
    int tkv_size = 0;

    switch (type) {
        case MODULE:
        {
            qal_key_vector_t *qal_kvpair = (qal_key_vector_t *)effectPayload->payload;
            for (int i = 0; i < qal_kvpair->num_tkvs; i++) {
                tkv.push_back(std::make_pair(qal_kvpair->kvp[i].key, qal_kvpair->kvp[i].value));
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
            tagCntrlName<<stream<<compressDevIds.at(0)<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                status = -ENOENT;
                goto exit;
            }
            QAL_VERBOSE(LOG_TAG, "mixer control: %s\n", tagCntrlName.str().data());

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
                goto exit;
            }
            ctl = NULL;
            tkv.clear();

            break;
        }
        default:
            QAL_ERR(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
            goto exit;
    }

exit:
    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    if (tagConfig) {
        free(tagConfig);
        tagConfig = nullptr;
    }

    return status;
}

int SessionAlsaCompress::setConfig(Stream * s, configType type, uint32_t tag1,
        uint32_t tag2, uint32_t tag3)
{
    int status = 0;
    uint32_t tagsent = 0;
    struct agm_tag_config* tagConfig = nullptr;
    std::ostringstream tagCntrlName;
    char const *stream = "COMPRESS";
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
            if(!tagConfig) {
                status = -ENOMEM;
                goto exit;
            }
            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            tagCntrlName << stream << compressDevIds.at(0) << " " << setParamTagControl;
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

int SessionAlsaCompress::setConfig(Stream * s, configType type, int tag)
{
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig;
    const char *setParamTagControl = "setParamTag";
    const char *stream = "COMPRESS";
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
                status = -ENOMEM;
                goto exit;
            }

            status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
            if (0 != status) {
                goto exit;
            }
            //TODO: how to get the id '5'
            tagCntrlName<<stream<<compressDevIds.at(0)<<" "<<setParamTagControl;
            ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
                return -ENOENT;
            }
            QAL_VERBOSE(LOG_TAG, "mixer control: %s\n", tagCntrlName.str().data());

            tkv_size = tkv.size()*sizeof(struct agm_key_value);
            status = mixer_ctl_set_array(ctl, tagConfig, sizeof(struct agm_tag_config) + tkv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
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
            calCntrlName<<stream<<compressDevIds.at(0)<<" "<<setCalibrationControl;
            ctl = mixer_get_ctl_by_name(mixer, calCntrlName.str().data());
            if (!ctl) {
                QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", calCntrlName.str().data());
                return -ENOENT;
            }
            QAL_VERBOSE(LOG_TAG, "mixer control: %s\n", calCntrlName.str().data());
            ckv_size = ckv.size()*sizeof(struct agm_key_value);
            //TODO make struct mixer and struct pcm as class private variables.
            status = mixer_ctl_set_array(ctl, calConfig, sizeof(struct agm_cal_config) + ckv_size);
            if (status != 0) {
                QAL_ERR(LOG_TAG,"failed to set the tag calibration %d", status);
            }
            ctl = NULL;
            ckv.clear();
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
            break;
    }

exit:
    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionAlsaCompress::getConfig(Stream * s)
{
   return 0;
}
*/
int SessionAlsaCompress::start(Stream * s)
{
    struct compr_config compress_config;
    struct qal_stream_attributes sAttr;
    int32_t status = 0;
    size_t in_buf_size, in_buf_count, out_buf_size, out_buf_count;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct qal_device dAttr;
    uint32_t ch_tag = 0, bitwidth_tag = 16, mfc_sr_tag = 0;
    struct sessionToPayloadParam deviceData;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    uint32_t miid;

    /** create an offload thread for posting callbacks */
    worker_thread = std::make_unique<std::thread>(offloadThreadLoop, this);

    s->getStreamAttributes(&sAttr);
    getSndCodecParam(codec, sAttr);
    s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
    compress_config.fragment_size = out_buf_size;
    compress_config.fragments = out_buf_count;
    compress_config.codec = &codec;
    // compress_open
    compress = compress_open(rm->getSndCard(), compressDevIds.at(0), COMPRESS_IN, &compress_config);
    if (!compress) {
        QAL_ERR(LOG_TAG, "compress open failed");
        goto free_feIds;
    }
    if (!is_compress_ready(compress)) {
        QAL_ERR(LOG_TAG, "compress open not ready %s", compress_get_error(compress));
        goto free_feIds;
    }
    /** set non blocking mode for writes */
    compress_nonblock(compress, !!ioMode);

    switch(sAttr.direction) {
        case QAL_AUDIO_OUTPUT:
            status = s->getAssociatedDevices(associatedDevices);
            if(0 != status) {
                QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
                return status;
            }
            rm->getBackEndNames(associatedDevices, rxAifBackEnds, txAifBackEnds);
            if (rxAifBackEnds.empty() && txAifBackEnds.empty()) {
                QAL_ERR(LOG_TAG, "no backend specified for this stream");
                return status;

            }

            for (int i = 0; i < associatedDevices.size();i++) {
                status = associatedDevices[i]->getDeviceAtrributes(&dAttr);
                if(0 != status) {
                    QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
                    return status;
                }
                /* Get PSPD MFC MIID and configure to match to device config */
                /* This has to be done after sending all mixer controls and before connect */
                status = SessionAlsaUtils::getModuleInstanceId(mixer, compressDevIds.at(0),
                                                               rxAifBackEnds[i].second.data(),
                                                               true, TAG_DEVICE_MFC_SR, &miid);
                if (status != 0) {
                    QAL_ERR(LOG_TAG,"getModuleInstanceId failed");
                    return status;
                }
                QAL_DBG(LOG_TAG, "miid : %x id = %d, data %s, dev id = %d\n", miid,
                        compressDevIds.at(0), rxAifBackEnds[i].second.data(), dAttr.id);
                deviceData.bitWidth = dAttr.config.bit_width;
                deviceData.sampleRate = dAttr.config.sample_rate;
                deviceData.numChannel = dAttr.config.ch_info->channels;
                builder->payloadMFCConfig(&payload, &payloadSize, miid, &deviceData);
                status = SessionAlsaUtils::setMixerParameter(mixer, compressDevIds.at(0), true,
                                                             payload, payloadSize);
                if (status != 0) {
                    QAL_ERR(LOG_TAG,"setMixerParameter failed");
                    return status;
                }

/*                QAL_ERR(LOG_TAG,"%s: device sample rate %d \n", __func__, dAttr.config.sample_rate);
                getSamplerateChannelBitwidthTags(&dAttr.config,
                    mfc_sr_tag, ch_tag, bitwidth_tag);

                setConfig(s, MODULE, mfc_sr_tag, ch_tag, bitwidth_tag);*/
            }
            break;
        default:
            break;
    }

free_feIds:
   return 0;
}

int SessionAlsaCompress::stop(Stream * s)
{
    int32_t status = 0;

    if (compress && playback_started)
        status = compress_stop(compress);
   return status;
}

int SessionAlsaCompress::close(Stream * s)
{
    int status = 0;
    struct qal_stream_attributes sAttr;
    std::ostringstream disconnectCtrlName;
    s->getStreamAttributes(&sAttr);
    if (!compress)
        return -EINVAL;
    disconnectCtrlName << "COMPRESS" << compressDevIds.at(0) << " disconnect";
    disconnectCtrl = mixer_get_ctl_by_name(mixer, disconnectCtrlName.str().data());
    if (!disconnectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", disconnectCtrlName.str().data());
        return -EINVAL;
    }
    /** Disconnect FE to BE */
    mixer_ctl_set_enum_by_string(disconnectCtrl, rxAifBackEnds[0].second.data());
    compress_close(compress);

    QAL_DBG(LOG_TAG, "out of compress close");
    {
    std::shared_ptr<offload_msg> msg = std::make_shared<offload_msg>(OFFLOAD_CMD_EXIT);
    std::lock_guard<std::mutex> lock(cv_mutex_);
    msg_queue_.push(msg);
    }
    cv_.notify_all();

    /* wait for handler to exit */
    worker_thread->join();
    worker_thread.reset(NULL);

    /* empty the pending messages in queue */
    while(!msg_queue_.empty())
        msg_queue_.pop();
    rm->freeFrontEndIds(compressDevIds, sAttr, 0);
    return 0;
}

int SessionAlsaCompress::read(Stream *s, int tag, struct qal_buffer *buf, int * size)
{
    return 0;
}

int SessionAlsaCompress::fileWrite(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag)
{
    std::fstream fs;
    QAL_DBG(LOG_TAG, "Enter.");

    fs.open ("/data/testcompr.wav", std::fstream::binary | std::fstream::out | std::fstream::app);
    QAL_ERR(LOG_TAG, "file open success");
    char * buff=static_cast<char *>(buf->buffer);
    fs.write (buff,buf->size);
    QAL_ERR(LOG_TAG, "file write success");
    fs.close();
    QAL_ERR(LOG_TAG, "file close success");
    *size = (int)(buf->size);
    QAL_ERR(LOG_TAG,"iExit. size: %d", *size);
    return 0;
}

int SessionAlsaCompress::write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag)
{
    int bytes_written = 0;
    int status;
    bool non_blocking = (!!ioMode);
    if (!buf || !(buf->buffer) || !(buf->size)){
        QAL_VERBOSE(LOG_TAG, "%s: buf: %pK, size: %d",
            __func__, buf, (buf ? buf->size : 0));
        return -EINVAL;
    }
    if (!compress) {
        QAL_ERR(LOG_TAG, "NULL pointer access,compress is invalid");
        return -EINVAL;
    }
    QAL_DBG(LOG_TAG, "%s: buf->size is %d buf->buffer is %pK ",
            __func__, buf->size, buf->buffer);

    bytes_written = compress_write(compress, buf->buffer, buf->size);

    QAL_VERBOSE(LOG_TAG, "writing buffer (%zu bytes) to compress device returned %d",
             buf->size, bytes_written);

    if (bytes_written >= 0 && bytes_written < (ssize_t)buf->size && non_blocking) {
        QAL_ERR(LOG_TAG, "No space available in compress driver, post msg to cb thread");
        std::shared_ptr<offload_msg> msg = std::make_shared<offload_msg>(OFFLOAD_CMD_WAIT_FOR_BUFFER);
        std::lock_guard<std::mutex> lock(cv_mutex_);
        msg_queue_.push(msg);

        cv_.notify_all();
    }

    if (!playback_started && bytes_written > 0) {
        status = compress_start(compress);
        if (status) {
            QAL_ERR(LOG_TAG, "compress start failed with err %d", status);
            return status;
        }
        playback_started = true;
    }

    if (size)
        *size = bytes_written;
    return 0;
}

int SessionAlsaCompress::readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag)
{
    return 0;
}
int SessionAlsaCompress::writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag)
{
    return 0;
}

int SessionAlsaCompress::setParameters(Stream *s, int tagId, uint32_t param_id, void *payload)
{
    qal_param_payload *param_payload = (qal_param_payload *)payload;
    int status = 0;
    int device = compressDevIds.at(0);
    uint8_t* alsaParamData = NULL;
    size_t alsaPayloadSize = 0;
    uint32_t miid = 0;
    uint32_t isPayloadTKV = PARAM_NONTKV;
    uint32_t payloadSize = 0;
    effect_qal_payload_t *effectQalPayload = nullptr;

    switch (param_id) {
        case QAL_PARAM_ID_UIEFFECT:
        {
            qal_effect_custom_payload_t *customPayload;
            param_payload = (qal_param_payload *)payload;
            effectQalPayload = (effect_qal_payload_t *)(param_payload->effect_payload);
            status = SessionAlsaUtils::getModuleInstanceId(mixer, device,
                                                           rxAifBackEnds[0].second.data(),
                                                           true, tagId, &miid);
            if (0 != status) {
                QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", tagId, status);
                return status;
            } else {
                customPayload = (qal_effect_custom_payload_t *)effectQalPayload->payload;
                builder->payloadCustomParam(&alsaParamData, &alsaPayloadSize,
                            customPayload->data, effectQalPayload->payloadSize - sizeof(uint32_t),
                            miid, customPayload->paramId);
                status = SessionAlsaUtils::setMixerParameter(mixer,
                                                             compressDevIds.at(0),
                                                             true, alsaParamData,
                                                             alsaPayloadSize);
                QAL_INFO(LOG_TAG, "mixer set param status=%d\n", status);
                free(alsaParamData);
            }
            break;
        }
        default:
            QAL_INFO(LOG_TAG, "Unsupported param id %u", param_id);
            break;
    }

    QAL_INFO(LOG_TAG, "compress format %x", audio_fmt);
    switch (audio_fmt) {
        case QAL_AUDIO_FMT_MP3:
            break;
#ifdef SND_COMPRESS_DEC_HDR
        case QAL_AUDIO_FMT_AAC:
            codec.format = SND_AUDIOSTREAMFORMAT_RAW;
            codec.options.aac_dec.audio_obj_type = param_payload->qal_snd_dec.aac_dec.audio_obj_type;
            codec.options.aac_dec.pce_bits_size = param_payload->qal_snd_dec.aac_dec.pce_bits_size;
            QAL_VERBOSE(LOG_TAG, "format - %x, audio_obj_type - %x, pce_bits_size - %x", codec.format, codec.options.aac_dec.audio_obj_type, codec.options.aac_dec.pce_bits_size);
            break;
        case QAL_AUDIO_FMT_AAC_ADTS:
            codec.format = SND_AUDIOSTREAMFORMAT_MP4ADTS;
            codec.options.aac_dec.audio_obj_type = param_payload->qal_snd_dec.aac_dec.audio_obj_type;
            codec.options.aac_dec.pce_bits_size = param_payload->qal_snd_dec.aac_dec.pce_bits_size;
            QAL_VERBOSE(LOG_TAG, "format - %x, audio_obj_type - %x, pce_bits_size - %x", codec.format, codec.options.aac_dec.audio_obj_type, codec.options.aac_dec.pce_bits_size);
            break;
        case QAL_AUDIO_FMT_AAC_ADIF:
            codec.format = SND_AUDIOSTREAMFORMAT_ADIF;
            codec.options.aac_dec.audio_obj_type = param_payload->qal_snd_dec.aac_dec.audio_obj_type;
            codec.options.aac_dec.pce_bits_size = param_payload->qal_snd_dec.aac_dec.pce_bits_size;
            QAL_VERBOSE(LOG_TAG, "format - %x, audio_obj_type - %x, pce_bits_size - %x", codec.format, codec.options.aac_dec.audio_obj_type, codec.options.aac_dec.pce_bits_size);
            break;
        case QAL_AUDIO_FMT_AAC_LATM:
            codec.format = SND_AUDIOSTREAMFORMAT_MP4LATM;
            codec.options.aac_dec.audio_obj_type = param_payload->qal_snd_dec.aac_dec.audio_obj_type;
            codec.options.aac_dec.pce_bits_size = param_payload->qal_snd_dec.aac_dec.pce_bits_size;
            QAL_VERBOSE(LOG_TAG, "format - %x, audio_obj_type - %x, pce_bits_size - %x", codec.format, codec.options.aac_dec.audio_obj_type, codec.options.aac_dec.pce_bits_size);
            break;
        case QAL_AUDIO_FMT_WMA_STD:
            codec.format = param_payload->qal_snd_dec.wma_dec.fmt_tag;
            codec.options.wma_dec.super_block_align = param_payload->qal_snd_dec.wma_dec.super_block_align;
            codec.options.wma_dec.bits_per_sample = param_payload->qal_snd_dec.wma_dec.bits_per_sample;
            codec.options.wma_dec.channelmask = param_payload->qal_snd_dec.wma_dec.channelmask;
            codec.options.wma_dec.encodeopt = param_payload->qal_snd_dec.wma_dec.encodeopt;
            codec.options.wma_dec.encodeopt1 = param_payload->qal_snd_dec.wma_dec.encodeopt1;
            codec.options.wma_dec.encodeopt2 = param_payload->qal_snd_dec.wma_dec.encodeopt2;
            codec.options.wma_dec.avg_bit_rate = param_payload->qal_snd_dec.wma_dec.avg_bit_rate;
            QAL_VERBOSE(LOG_TAG, "format - %x, super_block_align - %x, bits_per_sample - %x, channelmask - %x \n", codec.format, codec.options.wma_dec.super_block_align,
                        codec.options.wma_dec.bits_per_sample, codec.options.wma_dec.channelmask);
            QAL_VERBOSE(LOG_TAG, "encodeopt - %x, encodeopt1 - %x, encodeopt2 - %x, avg_bit_rate - %x \n", codec.options.wma_dec.encodeopt, codec.options.wma_dec.encodeopt1,
                        codec.options.wma_dec.encodeopt2, codec.options.wma_dec.avg_bit_rate);
            break;
        case QAL_AUDIO_FMT_WMA_PRO:
            codec.format = param_payload->qal_snd_dec.wma_dec.fmt_tag;
            codec.options.wma_dec.super_block_align = param_payload->qal_snd_dec.wma_dec.super_block_align;
            codec.options.wma_dec.bits_per_sample = param_payload->qal_snd_dec.wma_dec.bits_per_sample;
            codec.options.wma_dec.channelmask = param_payload->qal_snd_dec.wma_dec.channelmask;
            codec.options.wma_dec.encodeopt = param_payload->qal_snd_dec.wma_dec.encodeopt;
            codec.options.wma_dec.encodeopt1 = param_payload->qal_snd_dec.wma_dec.encodeopt1;
            codec.options.wma_dec.encodeopt2 = param_payload->qal_snd_dec.wma_dec.encodeopt2;
            codec.options.wma_dec.avg_bit_rate = param_payload->qal_snd_dec.wma_dec.avg_bit_rate;
            QAL_VERBOSE(LOG_TAG, "format - %x, super_block_align - %x, bits_per_sample - %x, channelmask - %x \n",codec.format, codec.options.wma_dec.super_block_align, codec.options.wma_dec.bits_per_sample,
                        codec.options.wma_dec.channelmask);
            QAL_VERBOSE(LOG_TAG, "encodeopt - %x, encodeopt1 - %x, encodeopt2 - %x, avg_bit_rate - %x \n", codec.options.wma_dec.encodeopt, codec.options.wma_dec.encodeopt1,
                        codec.options.wma_dec.encodeopt2, codec.options.wma_dec.avg_bit_rate);
            break;
        case QAL_AUDIO_FMT_ALAC:
            codec.options.alac_dec.frame_length = param_payload->qal_snd_dec.alac_dec.frame_length;
            codec.options.alac_dec.compatible_version = param_payload->qal_snd_dec.alac_dec.compatible_version;
            codec.options.alac_dec.bit_depth =  param_payload->qal_snd_dec.alac_dec.bit_depth;
            codec.options.alac_dec.pb =  param_payload->qal_snd_dec.alac_dec.pb;
            codec.options.alac_dec.mb =  param_payload->qal_snd_dec.alac_dec.mb;
            codec.options.alac_dec.kb =  param_payload->qal_snd_dec.alac_dec.kb;
            codec.options.alac_dec.num_channels = param_payload->qal_snd_dec.alac_dec.num_channels;
            codec.options.alac_dec.max_run = param_payload->qal_snd_dec.alac_dec.max_run;
            codec.options.alac_dec.max_frame_bytes = param_payload->qal_snd_dec.alac_dec.max_frame_bytes;
            codec.options.alac_dec.avg_bit_rate = param_payload->qal_snd_dec.alac_dec.avg_bit_rate;
            codec.options.alac_dec.sample_rate = param_payload->qal_snd_dec.alac_dec.sample_rate;
            codec.options.alac_dec.channel_layout_tag = param_payload->qal_snd_dec.alac_dec.channel_layout_tag;
            QAL_VERBOSE(LOG_TAG, "frame_length - %x, compatible_version - %x, bit_depth - %x, pb - %x, mb - %x, kb - %x", codec.options.alac_dec.frame_length, codec.options.alac_dec.compatible_version
                    ,codec.options.alac_dec.bit_depth, codec.options.alac_dec.pb, codec.options.alac_dec.mb, codec.options.alac_dec.kb);
            QAL_VERBOSE(LOG_TAG, "num_channels - %x, max_run - %x, max_frame_bytes - %x, avg_bit_rate - %x, sample_rate - %x, channel_layout_tag - %x", codec.options.alac_dec.num_channels, codec.options.alac_dec.max_run
                    ,codec.options.alac_dec.max_frame_bytes, codec.options.alac_dec.avg_bit_rate, codec.options.alac_dec.sample_rate, codec.options.alac_dec.channel_layout_tag);
            break;
       case QAL_AUDIO_FMT_APE:
            codec.options.ape_dec.compatible_version = param_payload->qal_snd_dec.ape_dec.compatible_version;
            codec.options.ape_dec.compression_level = param_payload->qal_snd_dec.ape_dec.compression_level;
            codec.options.ape_dec.format_flags = param_payload->qal_snd_dec.ape_dec.format_flags;
            codec.options.ape_dec.blocks_per_frame = param_payload->qal_snd_dec.ape_dec.blocks_per_frame;
            codec.options.ape_dec.final_frame_blocks = param_payload->qal_snd_dec.ape_dec.final_frame_blocks;
            codec.options.ape_dec.total_frames = param_payload->qal_snd_dec.ape_dec.total_frames;
            codec.options.ape_dec.bits_per_sample = param_payload->qal_snd_dec.ape_dec.bits_per_sample;
            codec.options.ape_dec.num_channels = param_payload->qal_snd_dec.ape_dec.num_channels;
            codec.options.ape_dec.sample_rate = param_payload->qal_snd_dec.ape_dec.sample_rate;
            codec.options.ape_dec.seek_table_present = param_payload->qal_snd_dec.ape_dec.seek_table_present;
            QAL_VERBOSE(LOG_TAG, "compatible_version - %x, compression_level - %x, format_flags - %x, blocks_per_frame - %x, final_frame_blocks - %x", codec.options.ape_dec.compatible_version, codec.options.ape_dec.compression_level,
                    codec.options.ape_dec.format_flags, codec.options.ape_dec.blocks_per_frame, codec.options.ape_dec.final_frame_blocks);
            QAL_VERBOSE(LOG_TAG, "total_frames - %x, bits_per_sample - %x, num_channels - %x, sample_rate - %x, seek_table_present - %x",  codec.options.ape_dec.total_frames, codec.options.ape_dec.bits_per_sample,
                    codec.options.ape_dec.num_channels, codec.options.ape_dec.sample_rate, codec.options.ape_dec.seek_table_present);
            break;
       case QAL_AUDIO_FMT_FLAC:
            codec.format = SND_AUDIOSTREAMFORMAT_FLAC;
            codec.options.flac_dec.sample_size = param_payload->qal_snd_dec.flac_dec.sample_size;
            codec.options.flac_dec.min_blk_size = param_payload->qal_snd_dec.flac_dec.min_blk_size;
            codec.options.flac_dec.max_blk_size = param_payload->qal_snd_dec.flac_dec.max_blk_size;
            codec.options.flac_dec.min_frame_size = param_payload->qal_snd_dec.flac_dec.min_frame_size;
            codec.options.flac_dec.max_frame_size = param_payload->qal_snd_dec.flac_dec.max_frame_size;
            QAL_VERBOSE(LOG_TAG, "sample_size - %x, min_blk_size - %x, max_blk_size - %x, min_frame_size - %x, max_frame_size - %x", codec.options.flac_dec.sample_size, codec.options.flac_dec.min_blk_size,
                    codec.options.flac_dec.max_blk_size, codec.options.flac_dec.min_frame_size, codec.options.flac_dec.max_frame_size);
            break;
        case QAL_AUDIO_FMT_FLAC_OGG:
            codec.format = SND_AUDIOSTREAMFORMAT_FLAC_OGG;
            codec.options.flac_dec.sample_size = param_payload->qal_snd_dec.flac_dec.sample_size;
            codec.options.flac_dec.min_blk_size = param_payload->qal_snd_dec.flac_dec.min_blk_size;
            codec.options.flac_dec.max_blk_size = param_payload->qal_snd_dec.flac_dec.max_blk_size;
            codec.options.flac_dec.min_frame_size = param_payload->qal_snd_dec.flac_dec.min_frame_size;
            codec.options.flac_dec.max_frame_size = param_payload->qal_snd_dec.flac_dec.max_frame_size;
            QAL_VERBOSE(LOG_TAG, "sample_size - %x, min_blk_size - %x, max_blk_size - %x, min_frame_size - %x, max_frame_size - %x", codec.options.flac_dec.sample_size, codec.options.flac_dec.min_blk_size,
                    codec.options.flac_dec.max_blk_size, codec.options.flac_dec.min_frame_size, codec.options.flac_dec.max_frame_size);
            break;
#endif
    }
    return 0;
}

int SessionAlsaCompress::registerCallBack(session_callback cb, void *cookie)
{
    sessionCb = cb;
    cbCookie = cookie;
    return 0;
}

int SessionAlsaCompress::flush()
{
    int status = 0;

    if (!compress) {
        QAL_ERR(LOG_TAG, "Compress is invalid");
        return -EINVAL;
    }
    if (playback_started) {
        QAL_VERBOSE(LOG_TAG,"Enter flush\n");
        status = compress_stop(compress);
        if (!status) {
            playback_started = false;
        }
    }
    QAL_VERBOSE(LOG_TAG,"playback_started %d status %d\n", playback_started,
            status);
    return status;
}

int SessionAlsaCompress::drain(qal_drain_type_t type)
{
    std::shared_ptr<offload_msg> msg;

    if (!compress) {
       QAL_ERR(LOG_TAG, "compress is invalid");
       return -EINVAL;
    }

    QAL_VERBOSE(LOG_TAG, "drain type = %d", type);

    switch(type) {
    case QAL_DRAIN:
    {
        msg = std::make_shared<offload_msg>(OFFLOAD_CMD_DRAIN);
        std::lock_guard<std::mutex> drain_lock(cv_mutex_);
        msg_queue_.push(msg);
        cv_.notify_all();
    }
    break;

    case QAL_DRAIN_PARTIAL:
    {
        msg = std::make_shared<offload_msg>(OFFLOAD_CMD_PARTIAL_DRAIN);
        std::lock_guard<std::mutex> partial_lock(cv_mutex_);
        msg_queue_.push(msg);
        cv_.notify_all();
    }
    break;

    default:
        QAL_ERR(LOG_TAG, "invalid drain type = %d", type);
        return -EINVAL;
    }

    return 0;
}

int SessionAlsaCompress::getParameters(Stream *s, int tagId, uint32_t param_id, void **payload)
{
    return 0;
}

int SessionAlsaCompress::getTimestamp(struct qal_session_time *stime)
{
    int status = 0;
    status = SessionAlsaUtils::getTimestamp(mixer, true, compressDevIds, spr_miid, stime);
    if (0 != status) {
       QAL_ERR(LOG_TAG, "getTimestamp failed status = %d", status);
       return status;
    }
    return status;
}


