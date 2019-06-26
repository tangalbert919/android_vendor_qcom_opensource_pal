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

#define XML_FILE "/etc/hw_ep_info.xml"

std::vector<codecDmaConfig> PayloadBuilder::codecConf;
std::vector<i2sConfig> PayloadBuilder::i2sConf;
std::vector<tdmConfig> PayloadBuilder::tdmConf;
std::vector<auxpcmConfig> PayloadBuilder::auxpcmConf;

const std::map<std::string, uint32_t> i2sIntfIdxLUT {
    {std::string{ "i2s-pri" }, I2S_INTF_TYPE_PRIMARY},
    {std::string{ "i2s-sec" }, I2S_INTF_TYPE_PRIMARY}
};

const std::map<std::string, uint32_t> tdmIntfIdxLUT {
    {std::string{ "tdm-pri" }, TDM_INTF_TYPE_PRIMARY},
    {std::string{ "tdm-sec" }, TDM_INTF_TYPE_PRIMARY}
};

const std::map<std::string, uint32_t> auxpcmIntfIdxLUT {
    {std::string{ "auxpcm-pri-rx" }, PCM_INTF_TYPE_PRIMARY},
    {std::string{ "auxpcm-pri-tx" }, PCM_INTF_TYPE_PRIMARY}
};

const std::map<std::string, uint32_t> tdmSyncSrc {
    {std::string{ "TDM_SYNC_SRC_INTERNAL" }, TDM_SYNC_SRC_INTERNAL },
    {std::string{ "TDM_SYNC_SRC_EXTERNAL" }, TDM_SYNC_SRC_EXTERNAL }
};

const std::map<std::string, uint32_t> tdmCtrlDataEnable {
    {std::string{ "TDM_CTRL_DATA_OE_DISABLE" }, TDM_CTRL_DATA_OE_DISABLE },
    {std::string{ "TDM_CTRL_DATA_OE_ENABLE" }, TDM_CTRL_DATA_OE_ENABLE }
};

const std::map<std::string, uint32_t> tdmSyncMode {
    {std::string{ "TDM_LONG_SYNC_MODE" }, TDM_LONG_SYNC_MODE },
    {std::string{ "TDM_SHORT_SYNC_BIT_MODE" }, TDM_SHORT_SYNC_BIT_MODE},
    {std::string{ "TDM_SHORT_SYNC_SLOT_MODE" }, TDM_SHORT_SYNC_SLOT_MODE}
};

const std::map<std::string, uint32_t> tdmCtrlInvertPulse {
    {std::string{ "TDM_SYNC_NORMAL" }, TDM_SYNC_NORMAL },
    {std::string{ "TDM_SYNC_INVERT" }, TDM_SYNC_INVERT }
};

const std::map<std::string, uint32_t> tdmCtrlSyncDataDelay {
    {std::string{ "TDM_DATA_DELAY_0_BCLK_CYCLE" }, TDM_DATA_DELAY_0_BCLK_CYCLE },
    {std::string{ "TDM_DATA_DELAY_1_BCLK_CYCLE" }, TDM_DATA_DELAY_1_BCLK_CYCLE },
    {std::string{ "TDM_DATA_DELAY_2_BCLK_CYCLE" }, TDM_DATA_DELAY_2_BCLK_CYCLE }
};

const std::map<std::string, uint32_t> auxpcmSyncSource {
    {std::string{ "PCM_SYNC_SRC_EXTERNAL" }, PCM_SYNC_SRC_EXTERNAL },
    {std::string{ "PCM_SYNC_SRC_INTERNAL" }, PCM_SYNC_SRC_INTERNAL },
};

const std::map<std::string, uint32_t> auxpcmctrlDataOutEnable {
    {std::string{ "PCM_CTRL_DATA_OE_DISABLE" }, PCM_CTRL_DATA_OE_DISABLE },
    {std::string{ "PCM_CTRL_DATA_OE_ENABLE" }, PCM_CTRL_DATA_OE_ENABLE },
};

const std::map<std::string, uint32_t> auxpcmFrameSetting {
    {std::string{ "PCM_BITS_PER_FRAME_16" }, PCM_BITS_PER_FRAME_16 },
    {std::string{ "PCM_BITS_PER_FRAME_32" }, PCM_BITS_PER_FRAME_32 },
    {std::string{ "PCM_BITS_PER_FRAME_64" }, PCM_BITS_PER_FRAME_64 },
    {std::string{ "PCM_BITS_PER_FRAME_128" }, PCM_BITS_PER_FRAME_128 },
    {std::string{ "PCM_BITS_PER_FRAME_256" }, PCM_BITS_PER_FRAME_256 },
};

const std::map<std::string, uint32_t> auxpcmAuxMode {
    {std::string{ "PCM_MODE" }, PCM_MODE },
    {std::string{ "AUX_MODE" }, AUX_MODE },
};

const std::map<std::string, uint32_t> codecIntfIdxLUT {
    {std::string{ "CODEC_TX0" }, CODEC_TX0},
    {std::string{ "CODEC_RX0" }, CODEC_RX0},
    {std::string{ "CODEC_TX1" }, CODEC_TX1},
    {std::string{ "CODEC_RX1" }, CODEC_RX1},
    {std::string{ "CODEC_TX2" }, CODEC_TX2},
    {std::string{ "CODEC_RX2" }, CODEC_RX2},
    {std::string{ "CODEC_TX3" }, CODEC_TX3},
    {std::string{ "CODEC_RX3" }, CODEC_RX3},
    {std::string{ "CODEC_TX4" }, CODEC_TX4},
    {std::string{ "CODEC_RX4" }, CODEC_RX4},
    {std::string{ "CODEC_TX5" }, CODEC_TX5},
    {std::string{ "CODEC_RX5" }, CODEC_RX5},
    {std::string{ "CODEC_RX6" }, CODEC_RX6},
    {std::string{ "CODEC_RX7" }, CODEC_RX7},
};

const std::map<std::string, uint32_t> i2sWsSrcLUT {
    {std::string{ "CONFIG_I2S_WS_SRC_INTERNAL" }, CONFIG_I2S_WS_SRC_INTERNAL },
    {std::string{ "CONFIG_I2S_WS_SRC_EXTERNAL" }, CONFIG_I2S_WS_SRC_EXTERNAL }
};

const std::map<std::string, uint32_t> intfLinkIdxLUT {
    {std::string{ "cdc-pri" }, 1},
    {std::string{ "cdc-sec" }, 2},
    {std::string{ "i2s-pri" }, 3},
    {std::string{ "i2s-sec" }, 4},
    {std::string{ "tdm-pri" }, 5},
    {std::string{ "tdm-sec" }, 6},
    {std::string{ "auxpcm-pri-rx" }, 7},
    {std::string{ "auxpcm-pri-tx" }, 8}
};

const std::map<std::string, uint32_t> lpaifIdxLUT {
    {std::string{ "LPAIF"},      0},
    {std::string{ "LPAIF_RXTX"}, 1},
    {std::string{ "LPAIF_WSA"},  2},
    {std::string{ "LPAIF_VA"},   3},
    {std::string{ "LPAIF_AXI"},  4}
};

void PayloadBuilder::payloadInMediaConfig(uint8_t** payload, size_t* size,
        struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data)
{
    if (!moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    struct media_format_t* mediaFmtHdr = NULL;
    struct payload_media_fmt_pcm_t* mediaFmtPayload;
    size_t payloadSize;
    uint8_t *payloadInfo = NULL;
    int numChannels = data->numChannel;
    uint8_t* pcmChannel = NULL;
    struct apm_module_param_data_t* header = NULL;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct media_format_t) +
                  sizeof(struct payload_media_fmt_pcm_t) +
                  sizeof(uint8_t)*numChannels;
    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);
    
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;

    mediaFmtHdr = (struct media_format_t*)(payloadInfo +
                     sizeof(struct apm_module_param_data_t));

    mediaFmtPayload = (struct payload_media_fmt_pcm_t*)(payloadInfo +
                         sizeof(struct apm_module_param_data_t) +
                         sizeof(struct media_format_t));

    pcmChannel = (uint8_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct media_format_t) +
                                       sizeof(struct payload_media_fmt_pcm_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_MEDIA_FORMAT;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    QAL_VERBOSE(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                  header->module_instance_id, header->param_id,
                  header->error_code, header->param_size);

    mediaFmtHdr->data_format = DATA_FORMAT_FIXED_POINT;
    mediaFmtHdr->fmt_id = MEDIA_FMT_ID_PCM;
    mediaFmtHdr->payload_size = sizeof(payload_media_fmt_pcm_t) +
        sizeof(uint8_t) * numChannels;
    QAL_VERBOSE(LOG_TAG, "mediaFmtHdr data_format:%x fmt_id:%x payload_size:%d channels:%d",
                      mediaFmtHdr->data_format, mediaFmtHdr->fmt_id,
                      mediaFmtHdr->payload_size, numChannels);

    /* TODO: Remove hardcoding */
    mediaFmtPayload->endianness = PCM_LITTLE_ENDIAN;
    mediaFmtPayload->alignment = 1;
    mediaFmtPayload->num_channels = data->numChannel;
    mediaFmtPayload->sample_rate = data->sampleRate;
    mediaFmtPayload->bit_width = data->bitWidth;

    if (data->bitWidth == 16 || data->bitWidth == 32) {
        mediaFmtPayload->bits_per_sample = data->bitWidth;
        mediaFmtPayload->q_factor =  data->bitWidth - 1;
    } else if (data->bitWidth == 24) {
        mediaFmtPayload->bits_per_sample = 32;
        mediaFmtPayload->q_factor = 27;
    }

    QAL_VERBOSE(LOG_TAG, "sample_rate:%d bit_width:%d bits_per_sample:%d q_factor:%d",
                      mediaFmtPayload->sample_rate, mediaFmtPayload->bit_width,
                      mediaFmtPayload->bits_per_sample, mediaFmtPayload->q_factor);

    if (data->numChannel == 1) {
        pcmChannel[0] = PCM_CHANNEL_C;
    } else if (data->numChannel == 2) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
    } else if (data->numChannel == 3) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
    }  else if (data->numChannel == 4) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_LB;
        pcmChannel[3] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 5) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LB;
        pcmChannel[4] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 6) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LFE;
        pcmChannel[4] = PCM_CHANNEL_LB;
        pcmChannel[5] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 7) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_LB;
        pcmChannel[6] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 8) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_CS;
        pcmChannel[6] = PCM_CHANNEL_LB;
        pcmChannel[7] = PCM_CHANNEL_RB;
    }
    QAL_DBG(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadOutMediaConfig(uint8_t** payload, size_t* size,
        struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data)
{
    if (!moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    struct media_format_t* mediaFmtHdr = NULL;
    struct payload_pcm_output_format_cfg_t* mediaFmtPayload = NULL;
    size_t payloadSize;
    uint8_t *payloadInfo = NULL;
    int numChannels = data->numChannel;
    uint8_t* pcmChannel = NULL;
    struct apm_module_param_data_t* header = NULL;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct media_format_t) +
                  sizeof(struct payload_pcm_output_format_cfg_t) +
                  sizeof(uint8_t)*numChannels;
    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);
    
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;

    mediaFmtHdr = (struct media_format_t*)(payloadInfo +
                     sizeof(struct apm_module_param_data_t));

    mediaFmtPayload = (struct payload_pcm_output_format_cfg_t*)(payloadInfo +
                         sizeof(struct apm_module_param_data_t) +
                         sizeof(struct media_format_t));

    pcmChannel = (uint8_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct media_format_t) +
                                       sizeof(struct payload_pcm_output_format_cfg_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_PCM_OUTPUT_FORMAT_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    QAL_VERBOSE(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                  header->module_instance_id, header->param_id,
                  header->error_code, header->param_size);
    
    mediaFmtHdr->data_format = DATA_FORMAT_FIXED_POINT;
    mediaFmtHdr->fmt_id = MEDIA_FMT_ID_PCM;
    mediaFmtHdr->payload_size = sizeof(payload_pcm_output_format_cfg_t) +
                                    sizeof(uint8_t) * numChannels;
    QAL_VERBOSE(LOG_TAG, "mediaFmtHdr data_format:%x fmt_id:%x payload_size:%d channels:%d",
                  mediaFmtHdr->data_format, mediaFmtHdr->fmt_id,
                  mediaFmtHdr->payload_size, numChannels);
    mediaFmtPayload->endianness = PCM_LITTLE_ENDIAN;
    mediaFmtPayload->alignment = 1;
    mediaFmtPayload->num_channels = data->numChannel;
    mediaFmtPayload->bit_width = data->bitWidth;
    if (data->bitWidth == 16 || data->bitWidth == 32) {
        mediaFmtPayload->bits_per_sample = data->bitWidth;
        mediaFmtPayload->q_factor =  data->bitWidth - 1;
    } else if (data->bitWidth == 24) {
        mediaFmtPayload->bits_per_sample = 32;
        mediaFmtPayload->q_factor = 27;
    }

    if (data->direction == 0x1)
        mediaFmtPayload->interleaved = PCM_DEINTERLEAVED_UNPACKED;
    else
        mediaFmtPayload->interleaved = PCM_INTERLEAVED;
    QAL_VERBOSE(LOG_TAG, "interleaved:%d bit_width:%d bits_per_sample:%d q_factor:%d",
                  mediaFmtPayload->interleaved, mediaFmtPayload->bit_width,
                  mediaFmtPayload->bits_per_sample, mediaFmtPayload->q_factor);
    if (data->numChannel == 1) {
        pcmChannel[0] = PCM_CHANNEL_C;
    } else if (data->numChannel == 2) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
    } else if (data->numChannel == 3) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
    } else if (data->numChannel == 4) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_LB;
        pcmChannel[3] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 5) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LB;
        pcmChannel[4] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 6) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LFE;
        pcmChannel[4] = PCM_CHANNEL_LB;
        pcmChannel[5] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 7) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_LB;
        pcmChannel[6] = PCM_CHANNEL_RB;
    } else if (data->numChannel == 8) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_CS;
        pcmChannel[6] = PCM_CHANNEL_LB;
        pcmChannel[7] = PCM_CHANNEL_RB;
    }
    QAL_DBG(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}
    
void PayloadBuilder::payloadCodecDmaConfig(uint8_t** payload, size_t* size, 
    struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data, std::string epName)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_codec_dma_intf_cfg_t* codecConfig = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    int32_t codecLinkIdx = 0;

    if (!moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    
    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof(struct param_id_codec_dma_intf_cfg_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    codecConfig = (struct param_id_codec_dma_intf_cfg_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_CODEC_DMA_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    QAL_VERBOSE(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d \n", header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    int32_t linkIdx = intfLinkIdxLUT.at(epName);
    for (int32_t j = 0; j < codecConf.size(); j++) {
        if (linkIdx != codecConf[j].intfLinkIdx)
            continue;
        codecLinkIdx = j;
        break;
    }
    codecConfig->lpaif_type = codecConf[codecLinkIdx].lpaifType;
    codecConfig->intf_indx = codecConf[codecLinkIdx].intfIdx;
    if (data->numChannel == 1) {
       codecConfig->active_channels_mask = 0x1;
    } else if (data->numChannel == 2) {
       codecConfig->active_channels_mask = 0x3;
    } else if (data->numChannel == 3) {
       codecConfig->active_channels_mask = 0x7;
    } else if (data->numChannel == 4) {
       codecConfig->active_channels_mask = 0xF;
    } else if (data->numChannel == 5) {
       codecConfig->active_channels_mask = 0x1F;
    } else if (data->numChannel == 6) {
       codecConfig->active_channels_mask = 0x3F;
    } else if (data->numChannel == 7) {
       codecConfig->active_channels_mask = 0x7F;
    } else if (data->numChannel == 8) {
       codecConfig->active_channels_mask = 0xFF;
    }
    QAL_DBG(LOG_TAG, "Codec Config cdc_dma_type:%d intf_idx:%d active_channels_mask:%d", codecConfig->lpaif_type, codecConfig->intf_indx,
                      codecConfig->active_channels_mask);
    QAL_DBG(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadI2sConfig(uint8_t** payload, size_t* size, 
    struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data, std::string epName)
{
    struct apm_module_param_data_t* header;
    struct  param_id_i2s_intf_cfg_t* i2sConfig;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    int32_t i2sLinkIdx = 0;

    if (! moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof(struct param_id_i2s_intf_cfg_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    i2sConfig = (struct param_id_i2s_intf_cfg_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_I2S_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    QAL_VERBOSE(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    int32_t linkIdx = intfLinkIdxLUT.at(epName);
    for (int32_t j = 0; j < i2sConf.size(); j++) {
        if (linkIdx != i2sConf[j].intfLinkIdx)
            continue;
        i2sLinkIdx = j;
        break;
    }
    i2sConfig->lpaif_type = i2sConf[i2sLinkIdx].lpaifType;
    i2sConfig->intf_idx = i2sConf[i2sLinkIdx].intfIdx;
    i2sConfig->sd_line_idx = i2sConf[i2sLinkIdx].sdLineIdx;
    i2sConfig->ws_src = i2sConf[i2sLinkIdx].wsSrc;
    QAL_VERBOSE(LOG_TAG, "i2s Config intf_idx:%x sd_line_idx:%x ws_src:%x",
         i2sConfig->intf_idx, i2sConfig->sd_line_idx, i2sConfig->ws_src);
    QAL_DBG(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadTdmConfig(uint8_t** payload, size_t* size, 
    struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data, std::string epName)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_tdm_intf_cfg_t* TdmConfig = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    int32_t tdmLinkIdx = 0;

    if (!moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof( struct param_id_tdm_intf_cfg_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    TdmConfig = (struct param_id_tdm_intf_cfg_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_TDM_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    QAL_VERBOSE(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    int32_t linkIdx = intfLinkIdxLUT.at(epName);
    for (int32_t j = 0; j < tdmConf.size(); j++) {
        if (linkIdx != tdmConf[j].intfLinkIdx)
            continue;
        tdmLinkIdx = j;
        break;
    }
    if (data->direction == 0x1) {
        TdmConfig->lpaif_type = tdmConf[tdmLinkIdx].lpaifType;
    } else {
        TdmConfig->lpaif_type = tdmConf[tdmLinkIdx].lpaifType;
    }
    TdmConfig->intf_idx = tdmConf[tdmLinkIdx].intfIdx;
    TdmConfig->sync_src = tdmConf[tdmLinkIdx].syncSrc;
    TdmConfig->ctrl_data_out_enable = tdmConf[tdmLinkIdx].ctrlDataOutEnable;
    TdmConfig->slot_mask = 0x3;
    TdmConfig->nslots_per_frame = 8;
    TdmConfig->slot_width = 32;
    TdmConfig->sync_mode = tdmConf[tdmLinkIdx].syncMode;
    TdmConfig->ctrl_invert_sync_pulse = tdmConf[tdmLinkIdx].ctrlInvertSyncPulse;
    TdmConfig->ctrl_sync_data_delay = tdmConf[tdmLinkIdx].ctrlSyncDataDelay;
    TdmConfig->reserved = 0;
    QAL_VERBOSE(LOG_TAG, "TDM Config intf_idx:%d sync_src:%d ctrl_data_out_enable:%d slot_mask:%d slot_per_frame:%d", TdmConfig->intf_idx,
                TdmConfig->sync_src, TdmConfig->ctrl_data_out_enable, TdmConfig->slot_mask, TdmConfig->nslots_per_frame);
    QAL_VERBOSE(LOG_TAG, "slot_width:%d sync_mode:%d ctrl_invert_sync_pulse:%d ctrl_sync_data_delay:%d", TdmConfig->slot_width,
                TdmConfig->sync_mode, TdmConfig->ctrl_invert_sync_pulse, TdmConfig->ctrl_sync_data_delay);
    QAL_DBG(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadAuxpcmConfig(uint8_t** payload, size_t* size,
    struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data, std::string epName) {
    struct apm_module_param_data_t* header = NULL;
    struct param_id_hw_pcm_intf_cfg_t* AuxpcmConfig = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    int32_t AuxpcmLinkIdx = 0,index;

    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof( struct param_id_hw_pcm_intf_cfg_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed");
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    AuxpcmConfig = (struct param_id_hw_pcm_intf_cfg_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_HW_PCM_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize;
    QAL_DBG(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    int32_t linkIdx = intfLinkIdxLUT.at(epName);
    for (int32_t j = 0; j < auxpcmConf.size(); j++) {
        if (linkIdx != auxpcmConf[j].intfLinkIdx)
            continue;
        AuxpcmLinkIdx = j;
        break;
    }


    AuxpcmConfig->lpaif_type = auxpcmConf[AuxpcmLinkIdx].lpaifType;
    AuxpcmConfig->intf_idx = auxpcmConf[AuxpcmLinkIdx].intfIdx;
    AuxpcmConfig->sync_src = auxpcmConf[AuxpcmLinkIdx].syncSrc;
    AuxpcmConfig->ctrl_data_out_enable = auxpcmConf[AuxpcmLinkIdx].ctrlDataOutEnable;

    if (data->numChannel == 1) {
         AuxpcmConfig->slot_mask = 0x1;
    } else if (data->numChannel == 2) {
         AuxpcmConfig->slot_mask = 0x3;
    } else if (data->numChannel == 3) {
         AuxpcmConfig->slot_mask = 0x7;
    } else if (data->numChannel == 4) {
         AuxpcmConfig->slot_mask = 0xF;
    } else if (data->numChannel == 5) {
         AuxpcmConfig->slot_mask = 0x1F;
    } else if (data->numChannel == 6) {
         AuxpcmConfig->slot_mask = 0x3F;
    } else if (data->numChannel == 7) {
         AuxpcmConfig->slot_mask = 0x7F;
    } else if (data->numChannel == 8) {
         AuxpcmConfig->slot_mask = 0xFF;
    }

    AuxpcmConfig->frame_setting = auxpcmConf[AuxpcmLinkIdx].frameSetting;
    AuxpcmConfig->aux_mode = auxpcmConf[AuxpcmLinkIdx].auxMode;

    QAL_DBG(LOG_TAG,"customPayload address %p and size %d", payloadInfo, payloadSize);

    QAL_DBG(LOG_TAG,"PCM Config intf_idx:%d sync_src:%d ctrl_data_out_enable:%d slot_mask:%d frame_setting:%d aux_mode:%d",
                AuxpcmConfig->intf_idx, AuxpcmConfig->sync_src, AuxpcmConfig->ctrl_data_out_enable,
                AuxpcmConfig->slot_mask, AuxpcmConfig->frame_setting, AuxpcmConfig->aux_mode);


    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadHwEpConfig(uint8_t** payload, size_t* size,
        struct gsl_module_id_info* moduleInfo, struct sessionToPayloadParam* data) {
    struct apm_module_param_data_t* header = NULL;
    struct param_id_hw_ep_mf_t* hwEpConf = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    if (!moduleInfo || !data) {
        QAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_hw_ep_mf_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    hwEpConf = (struct param_id_hw_ep_mf_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleInfo->module_entry[0].module_iid;
    header->param_id = PARAM_ID_HW_EP_MF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    header->param_size = payloadSize;
    QAL_VERBOSE(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    hwEpConf->sample_rate = data->sampleRate;
    hwEpConf->bit_width = data->bitWidth;

    hwEpConf->num_channels = data->numChannel;
    hwEpConf->data_format = DATA_FORMAT_FIXED_POINT;
    QAL_VERBOSE(LOG_TAG, "sample_rate:%d bit_width:%d num_channels:%d data_format:%d",
                      hwEpConf->sample_rate, hwEpConf->bit_width,
                      hwEpConf->num_channels, hwEpConf->data_format);
    QAL_VERBOSE(LOG_TAG, "customPayload address %p and size %d", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

PayloadBuilder::PayloadBuilder()
{

}

PayloadBuilder::~PayloadBuilder()
{
	
}

void PayloadBuilder::payloadStreamConfig(uint8_t** payload, size_t* size,
        struct gsl_module_id_info* moduleInfo, int payloadTag,
        struct sessionToPayloadParam* data)
{
    switch(payloadTag) {
    case IN_MEDIA:
        payloadInMediaConfig(payload, size, moduleInfo, data);
        break;
    case PCM_ENCODER:
        payloadOutMediaConfig(payload, size, moduleInfo, data);
        break;
    case PCM_DECODER:
        payloadOutMediaConfig(payload, size, moduleInfo, data);
        break;
    case PCM_CONVERTOR:
        payloadOutMediaConfig(payload, size, moduleInfo, data);
        break;
    default :
        QAL_ERR(LOG_TAG, "invalid stream config tagid %x", payloadTag);
        break;
    }
}

void PayloadBuilder::payloadDeviceConfig(uint8_t** payload, size_t* size, 
    struct gsl_module_id_info* moduleInfo, int payloadTag, struct sessionToPayloadParam* data) {
    payloadHwEpConfig(payload, size, moduleInfo, data);
}

void PayloadBuilder::payloadDeviceEpConfig(uint8_t **payload, size_t *size, 
    struct gsl_module_id_info* moduleInfo, int payloadTag, struct sessionToPayloadParam *data, std::string epName)
{
    int found = 0;
    std::string cdc("cdc");
    found = epName.find(cdc);
    if (found != std::string::npos) {
        payloadCodecDmaConfig(payload, size, moduleInfo, data, epName);
    }
    found = 0;
    std::string i2s("i2s");
    found = epName.find(i2s);
    if (found != std::string::npos) {
        payloadI2sConfig(payload, size, moduleInfo, data, epName);
    }

    found = 0;
    std::string tdm("tdm");
    found = epName.find(tdm);
    if (found !=std::string::npos) {
        payloadTdmConfig(payload, size, moduleInfo, data, epName);
    }
    found = 0;
    std::string auxpcm("auxpcm");
    found = epName.find(auxpcm);
    if (found !=std::string::npos) {
        payloadAuxpcmConfig(payload, size, moduleInfo, data, epName);
    }
}

void PayloadBuilder::processCodecInfo(const XML_Char **attr)
{
    struct codecDmaConfig cdc;
    if(strcmp(attr[0], "name" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'name' not found");
        return;
    }
    std::string linkName(attr[1]);
    cdc.intfLinkIdx = intfLinkIdxLUT.at(linkName);

    if(strcmp(attr[2],"lpaif_type") !=0 ) {
        QAL_ERR(LOG_TAG, "'lpaif_type' not found %s is the tag", attr[2]);
        return;
    }
    std::string cdcType(attr[3]);
    cdc.lpaifType = lpaifIdxLUT.at(cdcType);

    if(strcmp(attr[4],"intf_idx") !=0 ) {
        QAL_ERR(LOG_TAG, "'intf_idx' not found");
        return;
    }
    std::string intf((char*)attr[5]);

    cdc.intfIdx = codecIntfIdxLUT.at(intf);
    codecConf.push_back(cdc);
}

uint16_t numOfBitsSet(uint32_t lines)
{
    uint16_t numBitsSet = 0;
    while (lines) {
        numBitsSet++;
        lines = lines & (lines - 1);
    }
    return numBitsSet;
}


void PayloadBuilder::processI2sInfo(const XML_Char **attr)
{
    struct i2sConfig i2sCnf;
    if (strcmp(attr[0], "name") !=0 ) {
        QAL_ERR(LOG_TAG, "'name' not found");
        return;
    }
    std::string linkName(attr[1]);
    i2sCnf.intfIdx =  i2sIntfIdxLUT.at(linkName);
    i2sCnf.intfLinkIdx = intfLinkIdxLUT.at(linkName);

    if(strcmp(attr[2],"line_mask") !=0 ) {
        QAL_ERR(LOG_TAG, "line_mask' not found %s is the tag", attr[2]);
        return;
    }
    uint16_t lines =  atoi(attr[3]);
    uint16_t numOfSdLines = numOfBitsSet(lines);
    switch (numOfSdLines) {
    case 0:
        QAL_ERR(LOG_TAG, "no line is assigned");
        break;
    case 1:
        switch (lines) {
        case MSM_MI2S_SD0:
            i2sCnf.sdLineIdx = I2S_SD0;
            break;
        case MSM_MI2S_SD1:
            i2sCnf.sdLineIdx = I2S_SD1;
            break;
        case MSM_MI2S_SD2:
            i2sCnf.sdLineIdx = I2S_SD2;
            break;
        case MSM_MI2S_SD3:
            i2sCnf.sdLineIdx = I2S_SD3;
            break;
        default:
             QAL_ERR(LOG_TAG, "invalid SD lines %d", lines);
             return;
        }
        break;
    case 2:
        switch (lines) {
        case MSM_MI2S_SD0 | MSM_MI2S_SD1:
            i2sCnf.sdLineIdx = I2S_QUAD01;
            break;
        case MSM_MI2S_SD2 | MSM_MI2S_SD3:
            i2sCnf.sdLineIdx = I2S_QUAD23;
            break;
        default:
            QAL_ERR(LOG_TAG, "invalid SD lines %d", lines);
            return;
        }
        break;
    case 3:
        switch (lines) {
        case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2:
            i2sCnf.sdLineIdx = I2S_6CHS;
            break;
        default:
            QAL_ERR(LOG_TAG, "invalid SD lines %d", lines);
            return;
        }
        break;
    case 4:
        switch (lines) {
        case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2 | MSM_MI2S_SD3:
            i2sCnf.sdLineIdx = I2S_8CHS;
            break;
        default:
            QAL_ERR(LOG_TAG, "invalid SD lines %d", lines);
            return;
        }
        break;
    default:
        QAL_ERR(LOG_TAG, "invalid SD lines %d", numOfSdLines);
        return;
    }

    if(strcmp(attr[4],"ws_src") !=0 ) {
        QAL_ERR(LOG_TAG, "'line_mask' not found %s is the tag", attr[4]);
        return;
    }
    std::string src(attr[5]);
    i2sCnf.wsSrc = i2sWsSrcLUT.at(src);
    if(strcmp(attr[6],"lpaif_type") !=0 ) {
        QAL_ERR(LOG_TAG, "'lpaif_type' not found %s is the tag", attr[7]);
        return;
    }
    std::string lpaifName(attr[7]);
    i2sCnf.lpaifType = lpaifIdxLUT.at(lpaifName);
    i2sConf.push_back(i2sCnf);
}

void PayloadBuilder::processTdmInfo(const XML_Char **attr)
{
    struct tdmConfig tdmCnf;

    if(strcmp(attr[0], "name" ) !=0 ) {
        QAL_ERR(LOG_TAG, "name' not found");
        return;
    }
    std::string linkName(attr[1]);
    tdmCnf.intfIdx =  tdmIntfIdxLUT.at(linkName);
    tdmCnf.intfLinkIdx = intfLinkIdxLUT.at(linkName);
    if(strcmp(attr[2], "lpaif_type" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'lpaif_type' not found");
        return;
    }
    std::string lpaifName(attr[3]);
    tdmCnf.lpaifType = lpaifIdxLUT.at(lpaifName);
    if(strcmp(attr[4], "sync_src" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'sync_src' not found");
        return;
    }

    std::string syncSrc(attr[5]);
    tdmCnf.syncSrc = tdmSyncSrc.at(syncSrc);

    if(strcmp(attr[6], "ctrl_data" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'ctrl_data' not found");
        return;
    }

    std::string ctrlData(attr[7]);
    tdmCnf.ctrlDataOutEnable = tdmCtrlDataEnable.at(ctrlData);

    if(strcmp(attr[8], "sync_mode" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'sync_mode' not found");
        return;
    }

    std::string syncmode(attr[9]);
    tdmCnf.syncMode = tdmSyncMode.at(syncmode);

    if(strcmp(attr[10], "ctrl_invert_sync_pulse" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'ctrl_invert_sync_pulse' not found");
        return;
    }

    std::string ctrlInvert(attr[11]);
    tdmCnf.ctrlInvertSyncPulse = tdmCtrlInvertPulse.at(ctrlInvert);

    if(strcmp(attr[12], "ctrl_sync_data_delay" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'ctrl_sync_data_delay' not found");
        return;
    }

    std::string ctrlSyncData(attr[13]);
    tdmCnf.ctrlSyncDataDelay = tdmCtrlSyncDataDelay.at(ctrlSyncData);
    tdmConf.push_back(tdmCnf);
}

void PayloadBuilder::processAuxpcmInfo(const XML_Char **attr) {
    struct auxpcmConfig auxpcmCnf;

    if(strcmp(attr[0], "name" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'name' not found");
        return;
    }
    std::string linkName(attr[1]);
    auxpcmCnf.intfIdx =  auxpcmIntfIdxLUT.at(linkName);
    auxpcmCnf.intfLinkIdx = intfLinkIdxLUT.at(linkName);
    if(strcmp(attr[2], "lpaif_type" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'lpaif_type' not found");
        return;
    }
    std::string lpaifName(attr[3]);
    auxpcmCnf.lpaifType = lpaifIdxLUT.at(lpaifName);
    if(strcmp(attr[4], "sync_src" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'sync_src' not found");
        return;
    }

    std::string syncSrc(attr[5]);
    auxpcmCnf.syncSrc = auxpcmSyncSource.at(syncSrc);

    if(strcmp(attr[6], "ctrl_data" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'ctrl_data' not found");
        return;
    }

    std::string ctrlData(attr[7]);
    auxpcmCnf.ctrlDataOutEnable = auxpcmctrlDataOutEnable.at(ctrlData);


    if(strcmp(attr[8], "frame_setting" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'frame_setting' not found");
        return;
    }

    std::string frameSetting(attr[9]);
    auxpcmCnf.frameSetting = auxpcmFrameSetting.at(frameSetting);


    if(strcmp(attr[10], "aux_mode" ) !=0 ) {
        QAL_ERR(LOG_TAG, "'aux_mode' not found");
        return;
    }

    std::string auxMode(attr[11]);
    auxpcmCnf.auxMode = auxpcmAuxMode.at(auxMode);
    auxpcmConf.push_back(auxpcmCnf);
}

void PayloadBuilder::startTag(void *userdata __unused, const XML_Char *tag_name,
    const XML_Char **attr)
{
    if (strcmp(tag_name, "codec_hw_intf") == 0) {
        processCodecInfo(attr);
    } else if (strcmp(tag_name, "i2s_hw_intf") == 0) {
        processI2sInfo(attr);
    } else if (strcmp(tag_name, "tdm_hw_intf") == 0) {
        processTdmInfo(attr);
    } else if (strcmp(tag_name, "auxpcm_hw_intf") == 0) {
        processAuxpcmInfo(attr);
    }
}

void PayloadBuilder::endTag(void *userdata __unused, const XML_Char *tag_name)
{
    return;
}

int PayloadBuilder::init()
{
    XML_Parser parser;
    FILE *file = NULL;
    int ret = 0;
    int bytes_read;
    void *buf = NULL;

    QAL_DBG(LOG_TAG, "Enter.");
    file = fopen(XML_FILE, "r");
    if(!file) {
        QAL_ERR(LOG_TAG, "Failed to open xml");
        ret = -EINVAL;
        goto done;
    }

    parser = XML_ParserCreate(NULL);
    if (!parser) {
        QAL_ERR(LOG_TAG, "Failed to create XML");
        goto closeFile;
    }

    XML_SetElementHandler(parser, startTag, endTag);

    while(1) {
        buf = XML_GetBuffer(parser, 1024);
        if(buf == NULL) {
            QAL_ERR(LOG_TAG, "XML_Getbuffer failed");
            ret = -EINVAL;
            goto freeParser;
        }

        bytes_read = fread(buf, 1, 1024, file);
        if(bytes_read < 0) {
            QAL_ERR(LOG_TAG, "fread failed");
            ret = -EINVAL;
            goto freeParser;
        }

        if(XML_ParseBuffer(parser, bytes_read, bytes_read == 0) == XML_STATUS_ERROR) {
            QAL_ERR(LOG_TAG, "XML ParseBuffer failed ");
            ret = -EINVAL;
            goto freeParser;
        }
        if (bytes_read == 0)
            break;
    }
    QAL_DBG(LOG_TAG, "Exit.");

freeParser:
    XML_ParserFree(parser);
closeFile:
    fclose(file);
done:
    return ret;
}

void PayloadBuilder::payloadVolume(uint8_t **payload, size_t *size, uint32_t moduleId, struct qal_volume_data *volumedata, int tag)
{
    struct volume_ctrl_multichannel_gain_t* volCtrlPayload;
    struct volume_ctrl_master_gain_t* volMasterPayload;
    size_t payloadSize;
    uint8_t *payloadInfo = NULL;
    struct apm_module_param_data_t* header;

    if (!volumedata) {
        QAL_ERR(LOG_TAG, "Invalid volumedata param");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
                      sizeof(uint32_t) +
                      (sizeof(struct volume_ctrl_channels_gain_config_t) * (volumedata->no_of_volpair));

    if (payloadSize % 8 != 0)
            payloadSize = payloadSize + (8 - payloadSize % 8);
        
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;

    volCtrlPayload = (struct volume_ctrl_multichannel_gain_t*)(payloadInfo +
                         sizeof(struct apm_module_param_data_t));

    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    QAL_DBG(LOG_TAG, "header params IID:%x param_id:%x error_code:%d param_size:%d",
                  header->module_instance_id, header->param_id,
                  header->error_code, header->param_size);
    
    volCtrlPayload->num_config = (volumedata->no_of_volpair);
    QAL_DBG(LOG_TAG, "num config %x and given %x", (volCtrlPayload->num_config), (volumedata->no_of_volpair));
    for (int32_t i=0; i < (volumedata->no_of_volpair); i++) {
           QAL_VERBOSE(LOG_TAG, "volume sent:%f", (volumedata->volume_pair[i].vol));
           volCtrlPayload->gain_data[i].channel_mask_lsb = (volumedata->volume_pair[i].channel_mask);
           volCtrlPayload->gain_data[i].channel_mask_msb = 0x0;
           volCtrlPayload->gain_data[i].gain = (volumedata->volume_pair[i].vol)*(PLAYBACK_VOLUME_MASTER_GAIN_DEFAULT)*(1 << 15);
           QAL_VERBOSE(LOG_TAG, "Volume payload lsb:%x msb:%x gain:%x",
                  (volCtrlPayload->gain_data[i].channel_mask_lsb),
                  (volCtrlPayload->gain_data[i].channel_mask_msb),(volCtrlPayload->gain_data[i].gain));
    }

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVASoundModel(uint8_t **payload, size_t *size, uint32_t moduleId, struct qal_st_sound_model *soundModel)
{
    struct apm_module_param_data_t* header;
    uint8_t *phrase_sm;
    uint8_t *sm_data;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    size_t soundModelSize = 0;

    if (!soundModel) {
        QAL_ERR(LOG_TAG, "Invalid soundModel param");
        return;
    }
    soundModelSize = soundModel->data_size;
    payloadSize = sizeof(struct apm_module_param_data_t) + soundModelSize;
    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);
    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_DETECTION_ENGINE_SOUND_MODEL;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    phrase_sm = (uint8_t *)payloadInfo + sizeof(struct apm_module_param_data_t);
    sm_data = (uint8_t *)soundModel + soundModel->data_offset;
    memcpy(phrase_sm, sm_data, soundModelSize);
    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVAWakeUpConfig(uint8_t **payload, size_t *size, uint32_t moduleId, struct detection_engine_config_voice_wakeup *pWakeUp)
{
    struct apm_module_param_data_t* header;
    struct detection_engine_config_voice_wakeup *wakeUpConfig;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    uint8_t *confidence_level;
    uint8_t *kw_user_enable;
    uint32_t fixedConfigVoiceWakeupSize = 0;

    if (!pWakeUp) {
        QAL_ERR(LOG_TAG, "Invalid pWakeUp param");
        return;
    }
    fixedConfigVoiceWakeupSize = sizeof(struct detection_engine_config_voice_wakeup) -
                                  QAL_SOUND_TRIGGER_MAX_USERS * 2;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  fixedConfigVoiceWakeupSize +
                     pWakeUp->num_active_models * 2;

    QAL_VERBOSE(LOG_TAG, "payloadSize %d", payloadSize);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    wakeUpConfig = (struct detection_engine_config_voice_wakeup*)
                   (payloadInfo + sizeof(struct apm_module_param_data_t));
    memcpy(wakeUpConfig, pWakeUp, fixedConfigVoiceWakeupSize);
    confidence_level = (uint8_t*)((uint8_t*)wakeUpConfig + fixedConfigVoiceWakeupSize);
    kw_user_enable = (uint8_t*)((uint8_t*)wakeUpConfig + fixedConfigVoiceWakeupSize + pWakeUp->num_active_models);

    QAL_VERBOSE(LOG_TAG, "mode=%d custom_payload_size=%d", wakeUpConfig->mode, wakeUpConfig->custom_payload_size);
    QAL_VERBOSE(LOG_TAG, "num_active_models=%d reserved=%d", wakeUpConfig->num_active_models, wakeUpConfig->reserved);

    for (int i = 0; i < pWakeUp->num_active_models; i++) {
        confidence_level[i] = pWakeUp->confidence_levels[i];
        kw_user_enable[i] = pWakeUp->keyword_user_enables[i];
        QAL_VERBOSE(LOG_TAG, "confidence_level[%d] = %d KW_User_enable[%d] = %d", i, confidence_level[i], i, kw_user_enable[i]);
    }

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVAWakeUpBufferConfig(uint8_t **payload, size_t *size, uint32_t moduleId, struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufConfig)
{
    struct apm_module_param_data_t* header;
    struct detection_engine_voice_wakeup_buffer_config *pWakeUpBufCfg;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    if (!pWakeUpBufConfig) {
        QAL_ERR(LOG_TAG, "Invalid pWakeUpBufConfig param");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct detection_engine_voice_wakeup_buffer_config);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    pWakeUpBufCfg = (struct detection_engine_voice_wakeup_buffer_config *)
                    (payloadInfo + sizeof(struct apm_module_param_data_t));
    memcpy(pWakeUpBufCfg, pWakeUpBufConfig, sizeof(struct detection_engine_voice_wakeup_buffer_config));

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVAStreamSetupDuration(uint8_t **payload, size_t *size, uint32_t moduleId, struct audio_dam_downstream_setup_duration *pSetupDuration)
{
    struct apm_module_param_data_t* header;
    struct audio_dam_downstream_setup_duration *pDownStreamSetupDuration;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;
    if (!pSetupDuration) {
        QAL_ERR(LOG_TAG, "Invalid pSetupDuration param");
        return;
    }
    size_t structSize = sizeof(struct audio_dam_downstream_setup_duration) +
                        (pSetupDuration->num_output_ports *
                         sizeof(struct audio_dam_downstream_setup_duration_t));

    payloadSize = sizeof(struct apm_module_param_data_t) + structSize;

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_AUDIO_DAM_DOWNSTREAM_SETUP_DURATION;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    pDownStreamSetupDuration = (struct audio_dam_downstream_setup_duration *)
                               (payloadInfo + sizeof(struct apm_module_param_data_t));
    memcpy(pDownStreamSetupDuration, pSetupDuration, structSize);

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVAEventConfig(uint8_t **payload, size_t *size, uint32_t moduleId, struct detection_engine_generic_event_cfg *pEventConfig)
{
    struct apm_module_param_data_t* header;
    struct detection_engine_generic_event_cfg *pEventCfg;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    if (!pEventConfig) {
        QAL_ERR(LOG_TAG, "Invalid pEventConfig param");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct detection_engine_generic_event_cfg);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    pEventCfg = (struct detection_engine_generic_event_cfg *)
                (payloadInfo + sizeof(struct apm_module_param_data_t));
    memcpy(pEventCfg, pEventConfig, sizeof(struct detection_engine_generic_event_cfg));

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}

void PayloadBuilder::payloadSVAEngineReset(uint8_t **payload, size_t *size, uint32_t moduleId)
{
    struct apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    payloadSize = sizeof(struct apm_module_param_data_t);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);
    if (!payloadInfo) {
        QAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_DETECTION_ENGINE_RESET;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    *size = payloadSize;
    *payload = payloadInfo;
    QAL_DBG(LOG_TAG, "payload %u size %d", *payload, *size);
}
