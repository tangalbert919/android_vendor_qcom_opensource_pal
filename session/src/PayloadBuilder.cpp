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

#define LOG_TAG "PayloadBuilder"
#include "PayloadBuilder.h"
#include "SessionGsl.h"

void PayloadBuilder::payloadInMediaConfig(uint8_t **payload, size_t *size, uint32_t moduleId, struct qal_stream_attributes *sAttr) {
	struct media_format_t* mediaFmtHdr;
    struct payload_media_fmt_pcm_t* mediaFmtPayload;
    size_t payloadSize;
    uint8_t *payloadInfo = NULL;
    int numChannels = 2;
    uint8_t* pcmChannel;
    struct apm_module_param_data_t* header;

    //numChannels = config->ch_info->channels; 
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct media_format_t) +
                  sizeof(struct payload_media_fmt_pcm_t) +
                  sizeof(uint8_t)*numChannels;
    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);
    
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    header = (struct apm_module_param_data_t*)payloadInfo;

    mediaFmtHdr = (struct media_format_t*)(payloadInfo +
                     sizeof(struct apm_module_param_data_t));

    mediaFmtPayload = (struct payload_media_fmt_pcm_t*)(payloadInfo +
                         sizeof(struct apm_module_param_data_t) +
                         sizeof(struct media_format_t));

    pcmChannel = (uint8_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct media_format_t) +
                                       sizeof(struct payload_media_fmt_pcm_t));

    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_MEDIA_FORMAT;
    header->error_code = 0x0;
    header->param_size = payloadSize;
    mediaFmtHdr->data_format = DATA_FORMAT_FIXED_POINT;
    mediaFmtHdr->fmt_id = MEDIA_FMT_ID_PCM;
    mediaFmtHdr->payload_size = sizeof(payload_media_fmt_pcm_t) +
        sizeof(uint8_t) * numChannels;
    QAL_VERBOSE(LOG_TAG,"%s: mediaFmtHdr \n data_format:%d fmt_id:%d payload_size:%d channels:%d",
                      __func__, mediaFmtHdr->data_format, mediaFmtHdr->fmt_id,
                      mediaFmtHdr->payload_size, numChannels);

    /* TODO: Remove hardcoding */
    mediaFmtPayload->endianness = PCM_LITTLE_ENDIAN;
    mediaFmtPayload->alignment = 1;
    mediaFmtPayload->num_channels = numChannels;
    mediaFmtPayload->sample_rate = 48000;
    mediaFmtPayload->bit_width = 16;
    mediaFmtPayload->bits_per_sample = 16;
    mediaFmtPayload->q_factor = 15;
    QAL_VERBOSE(LOG_TAG,"%s: mediaFmtPayload \n sample_rate:%d bit_width:%d bits_per_sample:%d q_factor:%d",
                      __func__, mediaFmtPayload->sample_rate, mediaFmtPayload->bit_width,
                      mediaFmtPayload->bits_per_sample, mediaFmtPayload->q_factor);

    if (numChannels == 1) {
        pcmChannel[0] = PCM_CHANNEL_C;
    } else if (numChannels == 2) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
    }
    QAL_VERBOSE(LOG_TAG,"customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadOutMediaConfig(uint8_t **payload, size_t *size, uint32_t moduleId, struct qal_stream_attributes *sAttr) {
    struct media_format_t* mediaFmtHdr;
    struct payload_pcm_output_format_cfg_t* mediaFmtPayload;
    size_t payloadSize;
    uint8_t *payloadInfo = NULL;
    int numChannels = 2;
    uint8_t* pcmChannel;
    struct apm_module_param_data_t* header;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct media_format_t) +
                  sizeof(struct payload_pcm_output_format_cfg_t) +
                  sizeof(uint8_t)*numChannels;
    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);
    
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    
    header = (struct apm_module_param_data_t*)payloadInfo;

    mediaFmtHdr = (struct media_format_t*)(payloadInfo +
                     sizeof(struct apm_module_param_data_t));

    mediaFmtPayload = (struct payload_pcm_output_format_cfg_t*)(payloadInfo +
                         sizeof(struct apm_module_param_data_t) +
                         sizeof(struct media_format_t));

    pcmChannel = (uint8_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct media_format_t) +
                                       sizeof(struct payload_pcm_output_format_cfg_t));

    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_PCM_OUTPUT_FORMAT_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize;
    QAL_VERBOSE(LOG_TAG,"%s: header params \n IID:%d param_id:%d error_code:%d param_size:%d",
                  __func__, header->module_instance_id, header->param_id,
                  header->error_code, header->param_size);
    
    mediaFmtHdr->data_format = DATA_FORMAT_FIXED_POINT;
    mediaFmtHdr->fmt_id = MEDIA_FMT_ID_PCM;
    mediaFmtHdr->payload_size = sizeof(payload_pcm_output_format_cfg_t) +
                                    sizeof(uint8_t) * numChannels;
    QAL_VERBOSE(LOG_TAG,"%s: mediaFmtHdr \n data_format:%d fmt_id:%d payload_size:%d channels:%d",
                  __func__, mediaFmtHdr->data_format, mediaFmtHdr->fmt_id,
                  mediaFmtHdr->payload_size, numChannels);
    mediaFmtPayload->endianness = PCM_LITTLE_ENDIAN;
    mediaFmtPayload->alignment = 1;
    mediaFmtPayload->num_channels = numChannels;
    mediaFmtPayload->bit_width = 16;
    mediaFmtPayload->bits_per_sample = 16;
    mediaFmtPayload->q_factor = 15;
    if (sAttr->direction == 0x1)
        mediaFmtPayload->interleaved = PCM_DEINTERLEAVED_UNPACKED;
    else
        mediaFmtPayload->interleaved = PCM_INTERLEAVED;
    QAL_VERBOSE(LOG_TAG,"%s: mediaFmtPayload \n interleaved:%d bit_width:%d bits_per_sample:%d q_factor:%d",
                  __func__, mediaFmtPayload->interleaved, mediaFmtPayload->bit_width,
                  mediaFmtPayload->bits_per_sample, mediaFmtPayload->q_factor);
    if (numChannels == 1) {
        pcmChannel[0] = PCM_CHANNEL_C;
    } else if (numChannels == 2) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
    }
    QAL_VERBOSE(LOG_TAG,"customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}
    
void PayloadBuilder::payloadCodecDmaConfig(uint8_t **payload, size_t *size, uint32_t moduleId, int tagId) {
    struct apm_module_param_data_t* header;
    struct codecDmaIntfConfig* codecConfig;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    
    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof(struct codecDmaIntfConfig);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);

    header = (struct apm_module_param_data_t*)payloadInfo;
    codecConfig = (struct codecDmaIntfConfig*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_CODEC_DMA_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize;
    QAL_VERBOSE(LOG_TAG,"%s: header params \n IID:%d param_id:%d error_code:%d param_size:%d",
                      __func__, header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    if (HW_ENDPOINT_RX == tagId) {
        codecConfig->cdc_dma_type = WSA_CODEC_DMA_CORE;
        codecConfig->intf_idx = CODEC_RX0;
        codecConfig->active_channels_mask = 3;
    } else if (HW_ENDPOINT_TX == tagId){
        codecConfig->cdc_dma_type = VA_CODEC_DMA_CORE;
        codecConfig->intf_idx = CODEC_TX0;
        codecConfig->active_channels_mask = 3;
    }
    QAL_VERBOSE(LOG_TAG,"%s: Codec Config \n cdc_dma_type:%d intf_idx:%d active_channels_mask:%d",
                      __func__, codecConfig->cdc_dma_type, codecConfig->intf_idx,
                      codecConfig->active_channels_mask);
    QAL_VERBOSE(LOG_TAG,"customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadHwEpConfig(uint8_t **payload, size_t *size, uint32_t moduleId, int tagId) {
    struct apm_module_param_data_t* header;
    struct hwEpConfig* hwEpConf;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct hwEpConfig);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);

    header = (struct apm_module_param_data_t*)payloadInfo;
    hwEpConf = (struct hwEpConfig*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_HW_EP_MF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize;
    QAL_VERBOSE(LOG_TAG,"%s: header params \n IID:%d param_id:%d error_code:%d param_size:%d",
                      __func__, header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    hwEpConf->sample_rate = 48000;
    hwEpConf->bit_width = 16;

    if (HW_ENDPOINT_RX == tagId) {
        hwEpConf->num_channels = 2;
    } else if(HW_ENDPOINT_TX == tagId) {
        hwEpConf->num_channels = 2;
    }
    hwEpConf->data_format = DATA_FORMAT_FIXED_POINT;
    QAL_VERBOSE(LOG_TAG,"%s: Codec Config \n sample_rate:%d bit_width:%d num_channels:%d data_format:%d",
                      __func__, hwEpConf->sample_rate, hwEpConf->bit_width,
                      hwEpConf->num_channels, hwEpConf->data_format);
    QAL_VERBOSE(LOG_TAG,"%s:customPayload address %p and size %d", __func__, payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

PayloadBuilder::PayloadBuilder() {

}

PayloadBuilder::~PayloadBuilder() {
	
}
