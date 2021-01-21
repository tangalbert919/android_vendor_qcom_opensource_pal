/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: PayloadBuilder"
#include "ResourceManager.h"
#include "PayloadBuilder.h"
#include "SessionGsl.h"
#include "StreamSoundTrigger.h"
#include "spr_api.h"
#include <bt_intf.h>
#include <bt_ble.h>
#include "sp_vi.h"
#include "sp_rx.h"

#define XML_FILE "/vendor/etc/hw_ep_info.xml"
#define PARAM_ID_DISPLAY_PORT_INTF_CFG   0x8001154

#define PARAM_ID_USB_AUDIO_INTF_CFG                               0x080010D6

/* ID of the Output Media Format parameters used by MODULE_ID_MFC */
#define PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT            0x08001024
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/* Payload of the PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT parameter in the
 Media Format Converter Module. Following this will be the variable payload for channel_map. */

struct param_id_mfc_output_media_fmt_t
{
   int32_t sampling_rate;
   /**< @h2xmle_description  {Sampling rate in samples per second\n
                              ->If the resampler type in the MFC is chosen to be IIR,
                              ONLY the following sample rates are ALLOWED:
                              PARAM_VAL_NATIVE =-1;\n
                              PARAM_VAL_UNSET = -2;\n
                              8 kHz = 8000;\n
                              16kHz = 16000;\n
                              24kHz = 24000;\n
                              32kHz = 32000;\n
                              48kHz = 48000 \n
                              -> For resampler type FIR, all values in the range
                              below are allowed}
        @h2xmle_rangeList   {"PARAM_VAL_UNSET" = -2;
                             "PARAM_VAL_NATIVE" =-1;
                             "8 kHz"=8000;
                             "11.025 kHz"=11025;
                             "12 kHz"=12000;
                             "16 kHz"=16000;
                             "22.05 kHz"=22050;
                             "24 kHz"=24000;
                             "32 kHz"=32000;
                             "44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000;
                             "352.8 kHz"=352800;
                             "384 kHz"=384000}
        @h2xmle_default      {-1} */

   int16_t bit_width;
   /**< @h2xmle_description  {Bit width of audio samples \n
                              ->Samples with bit width of 16 (Q15 format) are stored in 16 bit words \n
                              ->Samples with bit width 24 bits (Q27 format) or 32 bits (Q31 format) are stored in 32 bit words}
        @h2xmle_rangeList    {"PARAM_VAL_NATIVE"=-1;
                              "PARAM_VAL_UNSET"=-2;
                              "16-bit"= 16;
                              "24-bit"= 24;
                              "32-bit"=32}
        @h2xmle_default      {-1}
   */

   int16_t num_channels;
   /**< @h2xmle_description  {Number of channels. \n
                              ->Ranges from -2 to 32 channels where \n
                              -2 is PARAM_VAL_UNSET and -1 is PARAM_VAL_NATIVE}
        @h2xmle_range        {-2..32}
        @h2xmle_default      {-1}
   */

   uint16_t channel_type[0];
   /**< @h2xmle_description  {Channel mapping array. \n
                              ->Specify a channel mapping for each output channel \n
                              ->If the number of channels is not a multiple of four, zero padding must be added
                              to the channel type array to align the packet to a multiple of 32 bits. \n
                              -> If num_channels field is set to PARAM_VAL_NATIVE (-1) or PARAM_VAL_UNSET(-2)
                              this field will be ignored}
        @h2xmle_variableArraySize {num_channels}
        @h2xmle_range        {1..63}
        @h2xmle_default      {1}    */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_mfc_output_media_fmt_t param_id_mfc_output_media_fmt_t;

struct param_id_usb_audio_intf_cfg_t
{
   uint32_t usb_token;
   uint32_t svc_interval;
};

std::vector<std::pair<uint32_t, uint32_t>> VSIDtoKV {
    /*for now map everything to default */
    { VOICEMMODE1,   0},
    { VOICEMMODE2,   0},
    { VOICELBMMODE1, 0},
    { VOICELBMMODE2, 0},
};

template <typename T>
void PayloadBuilder::populateChannelMap(T pcmChannel, uint8_t numChannel)
{
    if (numChannel == 1) {
        pcmChannel[0] = PCM_CHANNEL_C;
    } else if (numChannel == 2) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
    } else if (numChannel == 3) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
    } else if (numChannel == 4) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_LB;
        pcmChannel[3] = PCM_CHANNEL_RB;
    } else if (numChannel == 5) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LB;
        pcmChannel[4] = PCM_CHANNEL_RB;
    } else if (numChannel == 6) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LFE;
        pcmChannel[4] = PCM_CHANNEL_LB;
        pcmChannel[5] = PCM_CHANNEL_RB;
    } else if (numChannel == 7) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_LB;
        pcmChannel[6] = PCM_CHANNEL_RB;
    } else if (numChannel == 8) {
        pcmChannel[0] = PCM_CHANNEL_L;
        pcmChannel[1] = PCM_CHANNEL_R;
        pcmChannel[2] = PCM_CHANNEL_C;
        pcmChannel[3] = PCM_CHANNEL_LS;
        pcmChannel[4] = PCM_CHANNEL_RS;
        pcmChannel[5] = PCM_CHANNEL_CS;
        pcmChannel[6] = PCM_CHANNEL_LB;
        pcmChannel[7] = PCM_CHANNEL_RB;
    }
}

void PayloadBuilder::payloadUsbAudioConfig(uint8_t** payload, size_t* size,
    uint32_t miid, struct usbAudioConfig *data)
{
    struct apm_module_param_data_t* header;
    struct param_id_usb_audio_intf_cfg_t *usbConfig;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    payloadSize = sizeof(struct apm_module_param_data_t) +
       sizeof(struct param_id_usb_audio_intf_cfg_t);


    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);

    header = (struct apm_module_param_data_t*)payloadInfo;
    usbConfig = (struct param_id_usb_audio_intf_cfg_t*)(payloadInfo + sizeof(struct apm_module_param_data_t));
    header->module_instance_id = miid;
    header->param_id = PARAM_ID_USB_AUDIO_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_ERR(LOG_TAG,"header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                     header->module_instance_id, header->param_id,
                     header->error_code, header->param_size);

    usbConfig->usb_token = data->usb_token;
    usbConfig->svc_interval = data->svc_interval;
    PAL_VERBOSE(LOG_TAG,"customPayload address %pK and size %zu", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;

}

void PayloadBuilder::payloadDpAudioConfig(uint8_t** payload, size_t* size,
    uint32_t miid, struct dpAudioConfig *data)
{
    PAL_DBG(LOG_TAG, "Enter:");
    struct apm_module_param_data_t* header;
    struct dpAudioConfig *dpConfig;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0;

    payloadSize = sizeof(struct apm_module_param_data_t) +
        sizeof(struct dpAudioConfig);

    if (payloadSize % 8 != 0)
        payloadSize = payloadSize + (8 - payloadSize % 8);

    payloadInfo = (uint8_t*)malloc((size_t)payloadSize);

    header = (struct apm_module_param_data_t*)payloadInfo;
    dpConfig = (struct dpAudioConfig*)(payloadInfo + sizeof(struct apm_module_param_data_t));
    header->module_instance_id = miid;
    header->param_id = PARAM_ID_DISPLAY_PORT_INTF_CFG;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_ERR(LOG_TAG,"header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    dpConfig->channel_allocation = data->channel_allocation;
    dpConfig->mst_idx = data->mst_idx;
    dpConfig->dptx_idx = data->dptx_idx;
    PAL_ERR(LOG_TAG,"customPayload address %pK and size %zu", payloadInfo, payloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "Exit:");
}

void PayloadBuilder::payloadMFCConfig(uint8_t** payload, size_t* size,
        uint32_t miid, struct sessionToPayloadParam* data)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_mfc_output_media_fmt_t *mfcConf;
    int numChannels = data->numChannel;
    uint16_t* pcmChannel = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    if (!data) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_mfc_output_media_fmt_t) +
                  sizeof(uint16_t)*numChannels;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    mfcConf = (struct param_id_mfc_output_media_fmt_t*)(payloadInfo +
               sizeof(struct apm_module_param_data_t));
    pcmChannel = (uint16_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct param_id_mfc_output_media_fmt_t));

    header->module_instance_id = miid;
    header->param_id = PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_DBG(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    mfcConf->sampling_rate = data->sampleRate;
    mfcConf->bit_width = data->bitWidth;
    mfcConf->num_channels = data->numChannel;
    if (data->ch_info) {
        for (int i = 0; i < data->numChannel; ++i) {
            pcmChannel[i] = (uint16_t) data->ch_info->ch_map[i];
        }
    } else {
        populateChannelMap(pcmChannel, data->numChannel);
    }

    if ((2 == data->numChannel) && (PAL_SPEAKER_ROTATION_RL == data->rotation_type))
    {
        // Swapping the channel
        pcmChannel[0] = PCM_CHANNEL_R;
        pcmChannel[1] = PCM_CHANNEL_L;
    }

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "sample_rate:%d bit_width:%d num_channels:%d Miid:%d",
                      mfcConf->sampling_rate, mfcConf->bit_width,
                      mfcConf->num_channels, header->module_instance_id);
    PAL_DBG(LOG_TAG, "customPayload address %pK and size %zu", payloadInfo,
                *size);
}

PayloadBuilder::PayloadBuilder()
{

}

PayloadBuilder::~PayloadBuilder()
{

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


void PayloadBuilder::startTag(void *userdata __unused, const XML_Char *tag_name __unused,
    const XML_Char **attr __unused)
{
	return;
}

void PayloadBuilder::endTag(void *userdata __unused, const XML_Char *tag_name __unused)
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

    PAL_DBG(LOG_TAG, "Enter.");
    file = fopen(XML_FILE, "r");
    if (!file) {
        PAL_ERR(LOG_TAG, "Failed to open xml");
        ret = -EINVAL;
        goto done;
    }

    parser = XML_ParserCreate(NULL);
    if (!parser) {
        PAL_ERR(LOG_TAG, "Failed to create XML");
        goto closeFile;
    }

    XML_SetElementHandler(parser, startTag, endTag);

    while (1) {
        buf = XML_GetBuffer(parser, 1024);
        if (buf == NULL) {
            PAL_ERR(LOG_TAG, "XML_Getbuffer failed");
            ret = -EINVAL;
            goto freeParser;
        }

        bytes_read = fread(buf, 1, 1024, file);
        if (bytes_read < 0) {
            PAL_ERR(LOG_TAG, "fread failed");
            ret = -EINVAL;
            goto freeParser;
        }

        if (XML_ParseBuffer(parser, bytes_read, bytes_read == 0) == XML_STATUS_ERROR) {
            PAL_ERR(LOG_TAG, "XML ParseBuffer failed ");
            ret = -EINVAL;
            goto freeParser;
        }
        if (bytes_read == 0)
            break;
    }
    PAL_DBG(LOG_TAG, "Exit.");

freeParser:
    XML_ParserFree(parser);
closeFile:
    fclose(file);
done:
    return ret;
}

void PayloadBuilder::payloadTimestamp(uint8_t **payload, size_t *size, uint32_t moduleId)
{
    size_t payloadSize, padBytes;
    uint8_t *payloadInfo = NULL;
    struct apm_module_param_data_t* header;
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_spr_session_time_t);
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_SPR_SESSION_TIME;
    header->error_code = 0x0;
    header->param_size = payloadSize -  sizeof(struct apm_module_param_data_t);
    PAL_VERBOSE(LOG_TAG,"header params IID:%x param_id:%x error_code:%d param_size:%d\n",
                  header->module_instance_id, header->param_id,
                  header->error_code, header->param_size);
    *size = payloadSize + padBytes;;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "payload %pK size %zu", *payload, *size);
}

int PayloadBuilder::payloadCustomParam(uint8_t **alsaPayload, size_t *size,
            uint32_t *customPayload, uint32_t customPayloadSize,
            uint32_t moduleInstanceId, uint32_t paramId) {
    struct apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t alsaPayloadSize = 0;

    alsaPayloadSize = PAL_ALIGN_8BYTE(sizeof(struct apm_module_param_data_t)
                                        + customPayloadSize);
    payloadInfo = (uint8_t *)calloc(1, (size_t)alsaPayloadSize);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "failed to allocate memory.");
        return -ENOMEM;
    }

    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleInstanceId;
    header->param_id = paramId;
    header->error_code = 0x0;
    header->param_size = customPayloadSize;
    if (customPayloadSize)
        ar_mem_cpy(payloadInfo + sizeof(struct apm_module_param_data_t),
                         customPayloadSize,
                         customPayload,
                         customPayloadSize);
    *size = alsaPayloadSize;
    *alsaPayload = payloadInfo;

    PAL_DBG(LOG_TAG, "ALSA payload %pK size %zu", *alsaPayload, *size);

    return 0;
}

int PayloadBuilder::payloadSVAConfig(uint8_t **payload, size_t *size,
            uint8_t *config, size_t config_size,
            uint32_t miid, uint32_t param_id) {
    struct apm_module_param_data_t* header = nullptr;
    uint8_t* payloadInfo = nullptr;
    size_t payloadSize = 0;

    payloadSize = PAL_ALIGN_8BYTE(
        sizeof(struct apm_module_param_data_t) + config_size);
    payloadInfo = (uint8_t *)calloc(1, payloadSize);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "failed to allocate memory.");
        return -ENOMEM;
    }

    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = miid;
    header->param_id = param_id;
    header->error_code = 0x0;
    header->param_size = config_size;
    if (config_size)
        ar_mem_cpy(payloadInfo + sizeof(struct apm_module_param_data_t),
            config_size, config, config_size);
    *size = payloadSize;
    *payload = payloadInfo;

    PAL_INFO(LOG_TAG, "PID 0x%x, payload %pK size %zu", param_id, *payload, *size);

    return 0;
}

void PayloadBuilder::payloadQuery(uint8_t **payload, size_t *size,
                    uint32_t moduleId, uint32_t paramId, uint32_t querySize)
{
    struct apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    payloadSize = sizeof(struct apm_module_param_data_t) + querySize;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }

    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = paramId;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadDOAInfo(uint8_t **payload, size_t *size, uint32_t moduleId)
{
    struct apm_module_param_data_t* header;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct ffv_doa_tracking_monitor_t);
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = moduleId;
    header->param_id = PARAM_ID_FFV_DOA_TRACKING_MONITOR;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "payload %pK size %zu", *payload, *size);
}

void PayloadBuilder::payloadTWSConfig(uint8_t** payload, size_t* size,
        uint32_t miid, bool isTwsMonoModeOn, uint32_t codecFormat)
{
    struct apm_module_param_data_t* header = NULL;
    uint8_t* payloadInfo = NULL;
    uint32_t param_id = 0, val = 2;
    size_t payloadSize = 0, customPayloadSize = 0;
    param_id_aptx_classic_switch_enc_pcm_input_payload_t *aptx_classic_payload;
    param_id_aptx_adaptive_enc_switch_to_mono_t *aptx_adaptive_payload;

    if (codecFormat == CODEC_TYPE_APTX_DUAL_MONO) {
        param_id = PARAM_ID_APTX_CLASSIC_SWITCH_ENC_PCM_INPUT;
        customPayloadSize = sizeof(param_id_aptx_classic_switch_enc_pcm_input_payload_t);
    } else {
        param_id = PARAM_ID_APTX_ADAPTIVE_ENC_SWITCH_TO_MONO;
        customPayloadSize = sizeof(param_id_aptx_adaptive_enc_switch_to_mono_t);
    }
    payloadSize = PAL_ALIGN_8BYTE(sizeof(struct apm_module_param_data_t)
                                        + customPayloadSize);
    payloadInfo = (uint8_t *)calloc(1, (size_t)payloadSize);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "failed to allocate memory.");
        return;
    }

    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = miid;
    header->param_id = param_id;
    header->error_code = 0x0;
    header->param_size = customPayloadSize;
    val = (isTwsMonoModeOn == true) ? 1 : 2;
    if (codecFormat == CODEC_TYPE_APTX_DUAL_MONO) {
        aptx_classic_payload =
            (param_id_aptx_classic_switch_enc_pcm_input_payload_t*)(payloadInfo +
             sizeof(struct apm_module_param_data_t));
        aptx_classic_payload->transition_direction = val;
        ar_mem_cpy(payloadInfo + sizeof(struct apm_module_param_data_t),
                         customPayloadSize,
                         aptx_classic_payload,
                         customPayloadSize);
    } else {
        aptx_adaptive_payload =
            (param_id_aptx_adaptive_enc_switch_to_mono_t*)(payloadInfo +
             sizeof(struct apm_module_param_data_t));
        aptx_adaptive_payload->switch_between_mono_and_stereo = val;
        ar_mem_cpy(payloadInfo + sizeof(struct apm_module_param_data_t),
                         customPayloadSize,
                         aptx_adaptive_payload,
                         customPayloadSize);
    }

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadLC3Config(uint8_t** payload, size_t* size,
        uint32_t miid, bool isLC3MonoModeOn)
{
    struct apm_module_param_data_t* header = NULL;
    uint8_t* payloadInfo = NULL;
    uint32_t param_id = 0, val = 2;
    size_t payloadSize = 0, customPayloadSize = 0;
    param_id_lc3_encoder_switch_enc_pcm_input_payload_t *lc3_payload;

    param_id = PARAM_ID_LC3_ENC_DOWNMIX_2_MONO;
    customPayloadSize = sizeof(param_id_lc3_encoder_switch_enc_pcm_input_payload_t);

    payloadSize = PAL_ALIGN_8BYTE(sizeof(struct apm_module_param_data_t)
                                        + customPayloadSize);
    payloadInfo = (uint8_t *)calloc(1, (size_t)payloadSize);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "failed to allocate memory.");
        return;
    }

    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = miid;
    header->param_id = param_id;
    header->error_code = 0x0;
    header->param_size = customPayloadSize;
    val = (isLC3MonoModeOn == true) ? 1 : 2;

    lc3_payload =
        (param_id_lc3_encoder_switch_enc_pcm_input_payload_t *)(payloadInfo +
         sizeof(struct apm_module_param_data_t));
    lc3_payload->transition_direction = val;
    ar_mem_cpy(payloadInfo + sizeof(struct apm_module_param_data_t),
                     customPayloadSize,
                     lc3_payload,
                     customPayloadSize);

    *size = payloadSize;
    *payload = payloadInfo;
}

void PayloadBuilder::payloadRATConfig(uint8_t** payload, size_t* size,
        uint32_t miid, struct pal_media_config *data)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_rat_mf_t *ratConf;
    int numChannel;
    uint32_t bitWidth;
    uint16_t* pcmChannel = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    if (!data) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }

    numChannel = data->ch_info.channels;
    bitWidth = data->bit_width;
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_rat_mf_t) +
                  sizeof(uint16_t)*numChannel;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    ratConf = (struct param_id_rat_mf_t*)(payloadInfo +
               sizeof(struct apm_module_param_data_t));
    pcmChannel = (uint16_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                                       sizeof(struct param_id_rat_mf_t));

    header->module_instance_id = miid;
    header->param_id = PARAM_ID_RAT_MEDIA_FORMAT;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_DBG(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    ratConf->sample_rate = data->sample_rate;
    if (bitWidth == 16 || bitWidth == 32) {
        ratConf->bits_per_sample = bitWidth;
        ratConf->q_factor =  bitWidth - 1;
    } else if (bitWidth == 24) {
        ratConf->bits_per_sample = 32;
        ratConf->q_factor = 27;
    }
    ratConf->data_format = DATA_FORMAT_FIXED_POINT;
    ratConf->num_channels = numChannel;
    populateChannelMap(pcmChannel, numChannel);
    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "sample_rate:%d bits_per_sample:%d q_factor:%d data_format:%d num_channels:%d",
                      ratConf->sample_rate, ratConf->bits_per_sample, ratConf->q_factor,
                      ratConf->data_format, ratConf->num_channels);
    PAL_DBG(LOG_TAG, "customPayload address %pK and size %zu", payloadInfo,
                *size);
}

void PayloadBuilder::payloadPcmCnvConfig(uint8_t** payload, size_t* size,
        uint32_t miid, struct pal_media_config *data)
{
    struct apm_module_param_data_t* header = NULL;
    struct media_format_t *mediaFmtHdr;
    struct payload_pcm_output_format_cfg_t *mediaFmtPayload;
    int numChannels;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint8_t *pcmChannel;

    if (!data) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }

    numChannels = data->ch_info.channels;
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct media_format_t) +
                  sizeof(struct payload_pcm_output_format_cfg_t) +
                  sizeof(uint8_t)*numChannels;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
        return;
    }
    header          = (struct apm_module_param_data_t*)payloadInfo;
    mediaFmtHdr     = (struct media_format_t*)(payloadInfo +
                      sizeof(struct apm_module_param_data_t));
    mediaFmtPayload = (struct payload_pcm_output_format_cfg_t*)(payloadInfo +
                      sizeof(struct apm_module_param_data_t) +
                      sizeof(struct media_format_t));
    pcmChannel      = (uint8_t*)(payloadInfo + sizeof(struct apm_module_param_data_t) +
                      sizeof(struct media_format_t) +
                      sizeof(struct payload_pcm_output_format_cfg_t));

    header->module_instance_id = miid;
    header->param_id           = PARAM_ID_PCM_OUTPUT_FORMAT_CFG;
    header->error_code         = 0x0;
    header->param_size         = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_DBG(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    mediaFmtHdr->data_format  = DATA_FORMAT_FIXED_POINT;
    mediaFmtHdr->fmt_id       = MEDIA_FMT_ID_PCM;
    mediaFmtHdr->payload_size = sizeof(payload_pcm_output_format_cfg_t) +
                                sizeof(uint8_t) * numChannels;
    PAL_DBG(LOG_TAG, "mediaFmtHdr data_format:%x fmt_id:%x payload_size:%d channels:%d",
                      mediaFmtHdr->data_format, mediaFmtHdr->fmt_id,
                      mediaFmtHdr->payload_size, numChannels);

    mediaFmtPayload->endianness      = PCM_LITTLE_ENDIAN;
    mediaFmtPayload->num_channels    = data->ch_info.channels;
    if ((data->bit_width == 16) || (data->bit_width == 32)) {
        mediaFmtPayload->bit_width       = data->bit_width;
        mediaFmtPayload->bits_per_sample = data->bit_width;
        mediaFmtPayload->q_factor        = data->bit_width - 1;
        mediaFmtPayload->alignment       = PCM_LSB_ALIGNED;
    } else if (data->bit_width == 24) {
        // convert to Q31 that's expected by HD encoders.
        mediaFmtPayload->bit_width       = BIT_WIDTH_24;
        mediaFmtPayload->bits_per_sample = BITS_PER_SAMPLE_32;
        mediaFmtPayload->q_factor        = PCM_Q_FACTOR_31;
        mediaFmtPayload->alignment       = PCM_MSB_ALIGNED;
    } else {
        PAL_ERR(LOG_TAG, "invalid bit width %d", data->bit_width);
        delete[] payloadInfo;
        *size = 0;
        *payload = NULL;
        return;
    }
    mediaFmtPayload->interleaved     = PCM_INTERLEAVED;
    PAL_DBG(LOG_TAG, "interleaved:%d bit_width:%d bits_per_sample:%d q_factor:%d",
                  mediaFmtPayload->interleaved, mediaFmtPayload->bit_width,
                  mediaFmtPayload->bits_per_sample, mediaFmtPayload->q_factor);
    populateChannelMap(pcmChannel, numChannels);
    *size = (payloadSize + padBytes);
    *payload = payloadInfo;

    PAL_DBG(LOG_TAG, "customPayload address %pK and size %zu", payloadInfo,
                *size);
}

void PayloadBuilder::payloadCopPackConfig(uint8_t** payload, size_t* size,
        uint32_t miid, struct pal_media_config *data)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_cop_pack_output_media_fmt_t *copPack  = NULL;
    int numChannel;
    uint16_t* pcmChannel = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    if (!data) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }

    numChannel = data->ch_info.channels;
    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_cop_pack_output_media_fmt_t) +
                  sizeof(uint16_t)*numChannel;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo alloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    copPack = (struct param_id_cop_pack_output_media_fmt_t*)(payloadInfo +
               sizeof(struct apm_module_param_data_t));
    pcmChannel = (uint16_t*)(payloadInfo +
                          sizeof(struct apm_module_param_data_t) +
                          sizeof(struct param_id_cop_pack_output_media_fmt_t));

    header->module_instance_id = miid;
    header->param_id = PARAM_ID_COP_PACKETIZER_OUTPUT_MEDIA_FORMAT;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_DBG(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    copPack->sampling_rate = data->sample_rate;
    copPack->bits_per_sample = data->bit_width;
    copPack->num_channels = numChannel;
    populateChannelMap(pcmChannel, numChannel);
    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "sample_rate:%d bits_per_sample:%d num_channels:%d",
                      copPack->sampling_rate, copPack->bits_per_sample, copPack->num_channels);
    PAL_DBG(LOG_TAG, "customPayload address %pK and size %zu", payloadInfo,
                *size);
}

void PayloadBuilder::payloadCopV2PackConfig(uint8_t** payload, size_t* size,
        uint32_t miid, void *codecInfo)
{
    struct apm_module_param_data_t* header = NULL;
    struct param_id_cop_v2_stream_info_t *streamInfo = NULL;
    uint8_t* payloadInfo = NULL;
    audio_lc3_codec_cfg_t *bleCfg = NULL;
    struct cop_v2_stream_info_map_t* streamMap = NULL;
    size_t payloadSize = 0, padBytes = 0;
    uint64_t channel_mask = 0;
    int i = 0;

    bleCfg = (audio_lc3_codec_cfg_t *)codecInfo;
    if (!bleCfg) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }

    payloadSize = sizeof(struct apm_module_param_data_t) +
                  sizeof(struct param_id_cop_pack_output_media_fmt_t) +
                  sizeof(struct cop_v2_stream_info_map_t) * bleCfg->enc_cfg.stream_map_size;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = new uint8_t[payloadSize + padBytes]();
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "payloadInfo alloc failed %s", strerror(errno));
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    streamInfo = (struct param_id_cop_v2_stream_info_t*)(payloadInfo +
                  sizeof(struct apm_module_param_data_t));
    streamMap = (struct cop_v2_stream_info_map_t*)(payloadInfo +
                 sizeof(struct apm_module_param_data_t) +
                 sizeof(struct param_id_cop_v2_stream_info_t));

    header->module_instance_id = miid;
    header->param_id = PARAM_ID_COP_V2_STREAM_INFO;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    PAL_DBG(LOG_TAG, "header params \n IID:%x param_id:%x error_code:%d param_size:%d",
                      header->module_instance_id, header->param_id,
                      header->error_code, header->param_size);

    streamInfo->num_streams = bleCfg->enc_cfg.stream_map_size;;
    for (i = 0; i < streamInfo->num_streams; i++) {
        channel_mask = convert_channel_map(bleCfg->enc_cfg.streamMapOut[i].audio_location);
        streamMap[i].stream_id = bleCfg->enc_cfg.streamMapOut[i].stream_id;
        streamMap[i].direction = bleCfg->enc_cfg.streamMapOut[i].direction;
        streamMap[i].channel_mask_lsw = channel_mask  & 0x00000000FFFFFFFF;
        streamMap[i].channel_mask_msw = (channel_mask & 0xFFFFFFFF00000000) >> 32;
    }

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
    PAL_DBG(LOG_TAG, "customPayload address %pK and size %zu", payloadInfo, *size);
}

/** Used for Loopback stream types only */
int PayloadBuilder::populateStreamKV(Stream* s, std::vector <std::pair<int,int>> &keyVectorRx,
        std::vector <std::pair<int,int>> &keyVectorTx, struct vsid_info vsidinfo)
{
    int status = 0;
    struct pal_stream_attributes *sattr = NULL;

    PAL_DBG(LOG_TAG,"enter");
    sattr = new struct pal_stream_attributes();
    if (!sattr) {
        PAL_ERR(LOG_TAG,"sattr alloc failed %s status %d", strerror(errno), status);
        status = -ENOMEM;
        goto exit;
    }
    status = s->getStreamAttributes(sattr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed status %d\n", status);
        goto free_sattr;
    }

    PAL_DBG(LOG_TAG, "stream attribute type %d", sattr->type);
    switch (sattr->type) {
        case PAL_STREAM_LOOPBACK:
            if (sattr->info.opt_stream_info.loopback_type == PAL_STREAM_LOOPBACK_HFP_RX) {
                keyVectorRx.push_back(std::make_pair(STREAMRX, HFP_RX_PLAYBACK));
                keyVectorTx.push_back(std::make_pair(STREAMTX, HFP_RX_CAPTURE));
            } else if (sattr->info.opt_stream_info.loopback_type == PAL_STREAM_LOOPBACK_HFP_TX) {
                /** no StreamKV for HFP TX */
            } else /** pcm loopback*/ {
                keyVectorRx.push_back(std::make_pair(STREAMRX, PCM_RX_LOOPBACK));
            }
            break;
    case PAL_STREAM_VOICE_CALL:
            /*need to update*/
            for (int size= 0; size < vsidinfo.modepair.size(); size++) {
                for (int count1 = 0; count1 < VSIDtoKV.size(); count1++) {
                    if (vsidinfo.modepair[size].key == VSIDtoKV[count1].first)
                        VSIDtoKV[count1].second = vsidinfo.modepair[size].value;
                }
            }

            keyVectorRx.push_back(std::make_pair(STREAMRX,VOICE_CALL_RX));
            keyVectorTx.push_back(std::make_pair(STREAMTX,VOICE_CALL_TX));
            for (int index = 0; index < VSIDtoKV.size(); index++) {
                if (sattr->info.voice_call_info.VSID == VSIDtoKV[index].first) {
                    keyVectorRx.push_back(std::make_pair(vsidinfo.vsid,VSIDtoKV[index].second));
                    keyVectorTx.push_back(std::make_pair(vsidinfo.vsid,VSIDtoKV[index].second));
                }
            }
            break;
        default:
            status = -EINVAL;
            PAL_ERR(LOG_TAG,"unsupported stream type %d", sattr->type);
    }
free_sattr:
    delete sattr;
exit:
    return status;
}

/** Used for Loopback stream types only */
int PayloadBuilder::populateStreamPPKV(Stream* s, std::vector <std::pair<int,int>> &keyVectorRx,
        std::vector <std::pair<int,int>> &keyVectorTx __unused)
{
    int status = 0;
    struct pal_stream_attributes *sattr = NULL;

    PAL_DBG(LOG_TAG,"enter");
    sattr = new struct pal_stream_attributes();
    if (!sattr) {
        PAL_ERR(LOG_TAG,"sattr alloc failed %s status %d", strerror(errno), status);
        status = -ENOMEM;
        goto exit;
    }
    status = s->getStreamAttributes(sattr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed status %d\n",status);
        goto free_sattr;
    }

    PAL_DBG(LOG_TAG, "stream attribute type %d", sattr->type);
    switch (sattr->type) {
        case PAL_STREAM_VOICE_CALL:
            /*need to update*/
            keyVectorRx.push_back(std::make_pair(STREAMPP_RX, STREAMPP_RX_DEFAULT));
            break;
        default:
            PAL_ERR(LOG_TAG,"unsupported stream type %d", sattr->type);
    }
free_sattr:
    delete sattr;
exit:
    return status;
}

int PayloadBuilder::populateStreamKV(Stream* s,
        std::vector <std::pair<int,int>> &keyVector)
{
    int status = -EINVAL;
    uint32_t instance_id = 0;
    struct pal_stream_attributes *sattr = NULL;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG,"enter");
    sattr = new struct pal_stream_attributes;
    if (!sattr) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG,"sattr malloc failed %s status %d", strerror(errno), status);
        goto exit;
    }
    memset (sattr, 0, sizeof(struct pal_stream_attributes));

    status = s->getStreamAttributes(sattr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed status %d\n", status);
        goto free_sattr;
    }

    //todo move the keys to a to an xml of stream type to key
    //something like stream_type=PAL_STREAM_LOW_LATENCY, key=PCM_LL_PLAYBACK
    //from there create a map and retrieve the right keys
    PAL_DBG(LOG_TAG, "stream attribute type %d", sattr->type);
    switch (sattr->type) {
        case PAL_STREAM_LOW_LATENCY:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_LL_PLAYBACK));
                keyVector.push_back(std::make_pair(INSTANCE, INSTANCE_1));
            } else if (sattr->direction == PAL_AUDIO_INPUT) {
                keyVector.push_back(std::make_pair(STREAMTX,RAW_RECORD));
            } else if (sattr->direction == (PAL_AUDIO_OUTPUT | PAL_AUDIO_INPUT)) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_RX_LOOPBACK));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            break;
        case PAL_STREAM_ULTRA_LOW_LATENCY:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_ULL_PLAYBACK));
                //keyVector.push_back(std::make_pair(INSTANCE,INSTANCE_1));
            } else if (sattr->direction == PAL_AUDIO_INPUT) {
                keyVector.push_back(std::make_pair(STREAMTX,PCM_ULL_RECORD));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            break;
        case PAL_STREAM_PROXY:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_PROXY_PLAYBACK));
                //keyVector.push_back(std::make_pair(INSTANCE,INSTANCE_1));
            } else if (sattr->direction == PAL_AUDIO_INPUT) {
                keyVector.push_back(std::make_pair(STREAMTX,PCM_PROXY_RECORD));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            if (sattr->direction == PAL_AUDIO_INPUT) {
                if (sattr->info.opt_stream_info.tx_proxy_type == PAL_STREAM_PROXY_TX_WFD)
                    keyVector.push_back(std::make_pair(PROXY_TX_TYPE, PROXY_TX_WFD));
                else if (sattr->info.opt_stream_info.tx_proxy_type == PAL_STREAM_PROXY_TX_TELEPHONY_RX)
                    keyVector.push_back(std::make_pair(PROXY_TX_TYPE, PROXY_TX_VOICE_RX));
            }
            break;
        case PAL_STREAM_DEEP_BUFFER:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_DEEP_BUFFER));
            } else if (sattr->direction == PAL_AUDIO_INPUT) {
                keyVector.push_back(std::make_pair(STREAMTX,PCM_RECORD));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            instance_id = rm->getStreamInstanceID(s);
            if (instance_id < INSTANCE_1) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid instance id %d for deep buffer stream", instance_id);
                goto free_sattr;
            }
            keyVector.push_back(std::make_pair(INSTANCE, instance_id));
            break;
        case PAL_STREAM_NON_TUNNEL:
            instance_id = rm->getStreamInstanceID(s);
            if (instance_id < INSTANCE_1) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid instance id %d for deep buffer stream", instance_id);
                goto free_sattr;
            }
            keyVector.push_back(std::make_pair(INSTANCE, instance_id));

            if (sattr->out_media_config.aud_fmt_id != PAL_AUDIO_FMT_DEFAULT_PCM)
                keyVector.push_back(std::make_pair(STREAM, NT_DECODE));
            else if (sattr->out_media_config.aud_fmt_id == PAL_AUDIO_FMT_DEFAULT_PCM)
                keyVector.push_back(std::make_pair(STREAM, NT_ENCODE));
            else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid format %d on write path", sattr->out_media_config.aud_fmt_id);
                goto free_sattr;
            }
            break;
        case PAL_STREAM_PCM_OFFLOAD:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,PCM_OFFLOAD_PLAYBACK));
                keyVector.push_back(std::make_pair(INSTANCE, INSTANCE_1));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            break;
        case PAL_STREAM_GENERIC:
            if (sattr->direction == PAL_AUDIO_OUTPUT) {
                keyVector.push_back(std::make_pair(STREAMRX,GENERIC_PLAYBACK));
            } else {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid direction status %d", status);
                goto free_sattr;
            }
            break;
        case PAL_STREAM_COMPRESSED:
           if (sattr->direction == PAL_AUDIO_OUTPUT) {
               PAL_VERBOSE(LOG_TAG,"Stream compressed \n");
               keyVector.push_back(std::make_pair(STREAMRX, COMPRESSED_OFFLOAD_PLAYBACK));
               keyVector.push_back(std::make_pair(INSTANCE, INSTANCE_1));
           }
            break;
        case PAL_STREAM_VOIP_TX:
            keyVector.push_back(std::make_pair(STREAMTX, VOIP_TX_RECORD));
            break;
        case PAL_STREAM_VOIP_RX:
            keyVector.push_back(std::make_pair(STREAMRX, VOIP_RX_PLAYBACK));
            break;
        case PAL_STREAM_VOICE_UI:
            if (!s) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid stream");
                goto free_sattr;
            }
            keyVector.push_back(std::make_pair(STREAMTX, VOICE_UI));

            // add key-vector for stream configuration
            for (auto& kv: s->getStreamModifiers()) {
                keyVector.push_back(kv);
            }

            instance_id = s->getInstanceId();
            if (instance_id < INSTANCE_1) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid instance id %d for Voice UI stream",
                    instance_id);
                goto free_sattr;
            }
            keyVector.push_back(std::make_pair(INSTANCE, instance_id));
            break;
        case PAL_STREAM_VOICE_CALL_RECORD:
            keyVector.push_back(std::make_pair(STREAMTX,INCALL_RECORD));
            break;
        case PAL_STREAM_VOICE_CALL_MUSIC:
            keyVector.push_back(std::make_pair(STREAMRX,INCALL_MUSIC));
            break;
        default:
            status = -EINVAL;
            PAL_ERR(LOG_TAG,"unsupported stream type %d", sattr->type);
            goto free_sattr;
        }

free_sattr:
    delete sattr;
exit:
    return status;

}

int PayloadBuilder::populateStreamDeviceKV(Stream* s __unused, int32_t beDevId __unused,
        std::vector <std::pair<int,int>> &keyVector __unused)
{
    int status = 0;

    PAL_VERBOSE(LOG_TAG,"enter");
    return status;
}

int PayloadBuilder::populateStreamDeviceKV(Stream* s, int32_t rxBeDevId,
        std::vector <std::pair<int,int>> &keyVectorRx, int32_t txBeDevId,
        std::vector <std::pair<int,int>> &keyVectorTx, struct vsid_info vsidinfo,
                                           sidetone_mode_t sidetoneMode)
{
    int status = 0;
    std::vector <std::pair<int, int>> emptyKV;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_VERBOSE(LOG_TAG,"enter");
    if (rm->isOutputDevId(rxBeDevId)) {
        status = populateStreamKV(s, keyVectorRx, emptyKV, vsidinfo);
        if (status)
            goto exit;
    }
    if (rm->isInputDevId(txBeDevId)) {
        status = populateStreamKV(s, emptyKV, keyVectorTx, vsidinfo);
        if (status)
            goto exit;
    }

    status = populateDeviceKV(s, rxBeDevId, keyVectorRx, txBeDevId,
            keyVectorTx, sidetoneMode);

exit:
    return status;
}

int PayloadBuilder::populateDeviceKV(Stream* s, int32_t beDevId,
        std::vector <std::pair<int,int>> &keyVector)
{
    int status = 0;

    PAL_DBG(LOG_TAG,"enter");
    //todo move the keys to a to an xml  of device type to key
    //something like device_type=DEVICETX, key=SPEAKER
    //from there create a map and retrieve the right keys

//TODO change this mapping to xml
    switch (beDevId) {
        case PAL_DEVICE_OUT_SPEAKER :
            keyVector.push_back(std::make_pair(DEVICERX, SPEAKER));
            break;
        case PAL_DEVICE_OUT_HANDSET :
            keyVector.push_back(std::make_pair(DEVICERX, HANDSET));
            break;
        case PAL_DEVICE_OUT_BLUETOOTH_A2DP:
            // device gkv of A2DP is sent elsewhere, skip here.
            break;
        case PAL_DEVICE_OUT_BLUETOOTH_SCO:
            keyVector.push_back(std::make_pair(DEVICERX, BT_RX));
            keyVector.push_back(std::make_pair(BT_PROFILE, SCO));
            break;
        case PAL_DEVICE_OUT_AUX_DIGITAL:
        case PAL_DEVICE_OUT_AUX_DIGITAL_1:
        case PAL_DEVICE_OUT_HDMI:
           keyVector.push_back(std::make_pair(DEVICERX, HDMI_RX));
           break;
        case PAL_DEVICE_OUT_WIRED_HEADSET:
        case PAL_DEVICE_OUT_WIRED_HEADPHONE:
            keyVector.push_back(std::make_pair(DEVICERX,HEADPHONES));
            break;
        case PAL_DEVICE_OUT_USB_HEADSET:
        case PAL_DEVICE_OUT_USB_DEVICE:
            keyVector.push_back(std::make_pair(DEVICERX, USB_RX));
            break;
        case PAL_DEVICE_IN_SPEAKER_MIC:
            keyVector.push_back(std::make_pair(DEVICETX, SPEAKER_MIC));
            break;
        case PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
            keyVector.push_back(std::make_pair(DEVICETX, BT_TX));
            keyVector.push_back(std::make_pair(BT_PROFILE, SCO));
            break;
        case PAL_DEVICE_IN_WIRED_HEADSET:
           keyVector.push_back(std::make_pair(DEVICETX, HEADPHONE_MIC));
           break;
        case PAL_DEVICE_IN_USB_DEVICE:
        case PAL_DEVICE_IN_USB_HEADSET:
            keyVector.push_back(std::make_pair(DEVICETX, USB_TX));
            break;
        case PAL_DEVICE_IN_HANDSET_MIC:
           keyVector.push_back(std::make_pair(DEVICETX, HANDSETMIC));
           break;
        case PAL_DEVICE_IN_HANDSET_VA_MIC:
            keyVector.push_back(std::make_pair(DEVICETX, HANDSETMIC_VA));
            break;
        case PAL_DEVICE_IN_HEADSET_VA_MIC:
            keyVector.push_back(std::make_pair(DEVICETX, HEADSETMIC_VA));
            break;
        case PAL_DEVICE_IN_PROXY:
            {
                struct pal_stream_attributes sAttr;
                int32_t status = 0;
                keyVector.push_back(std::make_pair(DEVICETX, PROXY_TX));
                status = s->getStreamAttributes(&sAttr);
                if (status == 0) {
                    if (sAttr.info.opt_stream_info.tx_proxy_type == PAL_STREAM_PROXY_TX_WFD)
                        keyVector.push_back(std::make_pair(PROXY_TX_TYPE, PROXY_TX_WFD));
                }
            }
            break;
        case PAL_DEVICE_IN_TELEPHONY_RX:
            {
                struct pal_stream_attributes sAttr;
                int32_t status = 0;
                keyVector.push_back(std::make_pair(DEVICETX, PROXY_TX));
                status = s->getStreamAttributes(&sAttr);
                PAL_DBG(LOG_TAG,"enter status %d %d", status, sAttr.info.opt_stream_info.tx_proxy_type);
                if (status == 0) {
                    if (sAttr.info.opt_stream_info.tx_proxy_type == PAL_STREAM_PROXY_TX_TELEPHONY_RX)
                        keyVector.push_back(std::make_pair(PROXY_TX_TYPE, PROXY_TX_VOICE_RX));
                }
            }
            break;
        case PAL_DEVICE_OUT_PROXY:
            keyVector.push_back(std::make_pair(DEVICERX, PROXY_RX));
            break;
        case PAL_DEVICE_IN_VI_FEEDBACK:
            keyVector.push_back(std::make_pair(DEVICETX, VI_TX));
            break;
        case PAL_DEVICE_OUT_HEARING_AID:
            keyVector.push_back(std::make_pair(DEVICERX, PROXY_RX_VOICE));
            break;
        case PAL_DEVICE_IN_FM_TUNER:
            keyVector.push_back(std::make_pair(DEVICETX, FM_TX));
            break;
        default:
            PAL_DBG(LOG_TAG,"Invalid device id %d\n",beDevId);
            break;
    }

    return status;

}

int PayloadBuilder::populateDeviceKV(Stream* s, int32_t rxBeDevId,
        std::vector <std::pair<int,int>> &keyVectorRx, int32_t txBeDevId,
        std::vector <std::pair<int,int>> &keyVectorTx, sidetone_mode_t sidetoneMode)
{
    int status = 0;
    struct pal_stream_attributes sAttr;

    PAL_DBG(LOG_TAG,"enter");

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed \n");
        return status;
    }

    populateDeviceKV(s, rxBeDevId, keyVectorRx);
    populateDeviceKV(s, txBeDevId, keyVectorTx);

    /*add sidetone kv if needed*/
    if (sAttr.type == PAL_STREAM_VOICE_CALL && sidetoneMode == SIDETONE_SW) {
        PAL_DBG(LOG_TAG, "SW sidetone mode push kv");
        keyVectorTx.push_back(std::make_pair(SW_SIDETONE, SW_SIDETONE_ON));
    }

    return status;
}

int PayloadBuilder::populateDevicePPKV(Stream* s, int32_t rxBeDevId,
        std::vector <std::pair<int,int>> &keyVectorRx, int32_t txBeDevId,
        std::vector <std::pair<int,int>> &keyVectorTx, std::vector<kvpair_info> kvpair)
{
    int status = 0;
    struct pal_stream_attributes *sattr = NULL;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct pal_device dAttr;
    PAL_DBG(LOG_TAG,"enter");
    sattr = new struct pal_stream_attributes;
    if (!sattr) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG,"sattr malloc failed %s status %d", strerror(errno), status);
        goto exit;
    }
    memset (&dAttr, 0, sizeof(struct pal_device));
    memset (sattr, 0, sizeof(struct pal_stream_attributes));

    status = s->getStreamAttributes(sattr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed status %d\n", status);
        goto free_sattr;
    }
    status = s->getAssociatedDevices(associatedDevices);
    if (0 != status) {
       PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
       return status;
    }
    for (int i = 0; i < associatedDevices.size();i++) {
       status = associatedDevices[i]->getDeviceAttributes(&dAttr);
       if (0 != status) {
          PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
          return status;
       }
       if ((dAttr.id == rxBeDevId) || (dAttr.id == txBeDevId)) {
          PAL_DBG(LOG_TAG,"channels %d, id %d\n",dAttr.config.ch_info.channels, dAttr.id);
       }

        //todo move the keys to a to an xml of stream type to key
        //something like stream_type=PAL_STREAM_LOW_LATENCY, key=PCM_LL_PLAYBACK
        //from there create a map and retrieve the right keys
        PAL_DBG(LOG_TAG, "stream attribute type %d", sattr->type);
        switch (sattr->type) {
            case PAL_STREAM_VOICE_CALL:
                if (dAttr.id == rxBeDevId){
                    keyVectorRx.push_back(std::make_pair(DEVICEPP_RX, DEVICEPP_RX_VOICE_DEFAULT));
                }
                if (dAttr.id == txBeDevId){
                    for (int32_t kvsize = 0; kvsize < kvpair.size(); kvsize++) {
                         keyVectorTx.push_back(std::make_pair(kvpair[kvsize].key,
                                               kvpair[kvsize].value));
                    }
                }
                break;
            case PAL_STREAM_LOW_LATENCY:
            case PAL_STREAM_COMPRESSED:
            case PAL_STREAM_DEEP_BUFFER:
            case PAL_STREAM_PCM_OFFLOAD:
            case PAL_STREAM_GENERIC:
                if (sattr->direction == PAL_AUDIO_OUTPUT) {
                  if(dAttr.id == PAL_DEVICE_OUT_PROXY) {
                    PAL_DBG(LOG_TAG,"Device PP for Proxy is Rx Default");
                    keyVectorRx.push_back(std::make_pair(DEVICEPP_RX, DEVICEPP_RX_DEFAULT));
                  }
                  else {
                    keyVectorRx.push_back(std::make_pair(DEVICEPP_RX, DEVICEPP_RX_AUDIO_MBDRC));
                  }
                }
                else if (sattr->direction == PAL_AUDIO_INPUT) {
                    for (int32_t kvsize = 0; kvsize < kvpair.size(); kvsize++) {
                         keyVectorTx.push_back(std::make_pair(kvpair[kvsize].key,
                                               kvpair[kvsize].value));
                    }
                }
                break;
            case PAL_STREAM_VOIP_RX:
                keyVectorRx.push_back(std::make_pair(DEVICEPP_RX, DEVICEPP_RX_VOIP_MBDRC));
                break;
            case PAL_STREAM_LOOPBACK:
                if (sattr->info.opt_stream_info.loopback_type ==
                                                    PAL_STREAM_LOOPBACK_HFP_RX) {
                    keyVectorRx.push_back(std::make_pair(DEVICEPP_RX,
                                                         DEVICEPP_RX_HFPSINK));
                } else if(sattr->info.opt_stream_info.loopback_type ==
                                                    PAL_STREAM_LOOPBACK_HFP_TX) {
                    if (kvpair.size() == 0) { //use default HFP_SINK_FLUENCE_SMECNS
                        keyVectorTx.push_back(std::make_pair(DEVICEPP_TX,
                                                            DEVICEPP_TX_HFP_SINK_FLUENCE_SMECNS));
                    } else { //use configuration get from resourcemanager.xml
                        for (int32_t kvsize = 0; kvsize < kvpair.size(); kvsize++) {
                            keyVectorTx.push_back(std::make_pair(kvpair[kvsize].key,
                                                kvpair[kvsize].value));
                        }
                    }
                }
                break;
            case PAL_STREAM_VOIP_TX:
                for (int32_t kvsize = 0; kvsize < kvpair.size(); kvsize++) {
                     keyVectorTx.push_back(std::make_pair(kvpair[kvsize].key,
                                           kvpair[kvsize].value));
                }
                break;
            case PAL_STREAM_VOICE_UI:
                /*
                 * add key-vector for the device pre-proc that was selected
                 * by the stream
                 */
                for (auto& kv: s->getDevPpModifiers())
                    keyVectorTx.push_back(kv);
                break;
            default:
                PAL_DBG(LOG_TAG,"stream type %d doesn't support populateDevicePPKV ", sattr->type);
                goto free_sattr;
        }
    }
    populateDeviceKV(s, rxBeDevId, keyVectorRx);
    populateDeviceKV(s, txBeDevId, keyVectorTx);
free_sattr:
    delete sattr;
exit:
    return status;
}

int PayloadBuilder::populateStreamCkv(Stream *s __unused, std::vector <std::pair<int,int>> &keyVector __unused, int tag __unused,
        struct pal_volume_data **volume_data __unused)
{
    int status = 0;

    PAL_DBG(LOG_TAG, "Enter");

    /*
     * Sending volume minimum as we want to ramp up instead of ramping
     * down while setting the desired volume. Thus avoiding glitch
     * TODO: Decide what to send as ckv in graph open
     */
    keyVector.push_back(std::make_pair(VOLUME,LEVEL_15));
    PAL_DBG(LOG_TAG, "Entered default %x %x", VOLUME, LEVEL_15);

    return status;
}

int PayloadBuilder::populateDevicePPCkv(Stream *s, std::vector <std::pair<int,int>> &keyVector)
{
    int status = 0;
    struct pal_stream_attributes *sattr = NULL;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct pal_device dAttr;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    PAL_DBG(LOG_TAG,"enter");
    sattr = new struct pal_stream_attributes;
    if (!sattr) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG,"sattr malloc failed %s status %d", strerror(errno), status);
        goto exit;
    }
    memset (&dAttr, 0, sizeof(struct pal_device));
    memset (sattr, 0, sizeof(struct pal_stream_attributes));

    status = s->getStreamAttributes(sattr);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getStreamAttributes Failed status %d\n",status);
        goto free_sattr;
    }
    status = s->getAssociatedDevices(associatedDevices);
    if (0 != status) {
       PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
       return status;
    }
    for (int i = 0; i < associatedDevices.size();i++) {
        status = associatedDevices[i]->getDeviceAttributes(&dAttr);
        if (0 != status) {
            PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
            return status;
        }

        switch (sattr->type) {
            case PAL_STREAM_VOICE_UI:
                PAL_INFO(LOG_TAG,"channels %d, id %d\n",dAttr.config.ch_info.channels, dAttr.id);
                /* Push Channels CKV for FFNS or FFECNS channel based calibration */
                keyVector.push_back(std::make_pair(CHANNELS,
                                                   dAttr.config.ch_info.channels));
                break;
            case PAL_STREAM_LOW_LATENCY:
            case PAL_STREAM_DEEP_BUFFER:
            case PAL_STREAM_PCM_OFFLOAD:
            case PAL_STREAM_COMPRESSED:
                if (dAttr.id == PAL_DEVICE_OUT_SPEAKER) {
                    PAL_INFO(LOG_TAG,"SpeakerProt Status[%d], RAS Status[%d]\n",
                            rm->isSpeakerProtectionEnabled, rm->isRasEnabled);
                }
                if (rm->isSpeakerProtectionEnabled == true &&
                    rm->isRasEnabled == true &&
                    dAttr.id == PAL_DEVICE_OUT_SPEAKER) {
                    if (dAttr.config.ch_info.channels == 2) {
                        PAL_INFO(LOG_TAG,"Enabling RAS - device channels[%d]\n",
                                dAttr.config.ch_info.channels);
                        keyVector.push_back(std::make_pair(RAS_SWITCH, RAS_ON));
                    } else {
                        PAL_INFO(LOG_TAG,"Disabling RAS - device channels[%d] \n",
                                dAttr.config.ch_info.channels);
                        keyVector.push_back(std::make_pair(RAS_SWITCH, RAS_OFF));
                    }
                }

                if ((dAttr.id == PAL_DEVICE_OUT_SPEAKER) ||
                    (dAttr.id == PAL_DEVICE_OUT_WIRED_HEADSET) ||
                    (dAttr.id == PAL_DEVICE_OUT_WIRED_HEADPHONE)) {
                    PAL_DBG(LOG_TAG, "Entered default %x %x", GAIN, GAIN_0);
                    keyVector.push_back(std::make_pair(GAIN, GAIN_0));
                }

                /* TBD: Push Channels for these types once Channels are added */
                //keyVector.push_back(std::make_pair(CHANNELS,
                //                                   dAttr.config.ch_info.channels));
                break;
            default:
                PAL_VERBOSE(LOG_TAG,"stream type %d doesn't support DevicePP CKV ", sattr->type);
                goto free_sattr;
        }
    }
free_sattr:
    delete sattr;
exit:
    return status;
}

int PayloadBuilder::populateCalKeyVector(Stream *s, std::vector <std::pair<int,int>> &ckv, int tag) {
    int status = 0;
    PAL_VERBOSE(LOG_TAG,"enter \n");
    std::vector <std::pair<int,int>> keyVector;
    struct pal_stream_attributes sAttr;
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    KeyVect_t stream_config_kv;
    struct pal_device dAttr;
    int level = -1;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        PAL_ERR(LOG_TAG, "getStreamAttributes Failed");
        return status;
    }

    float voldB = 0.0f;
    struct pal_volume_data *voldata = NULL;
    voldata = (struct pal_volume_data *)calloc(1, (sizeof(uint32_t) +
                      (sizeof(struct pal_channel_vol_kv) * (0xFFFF))));
    if (!voldata) {
        status = -ENOMEM;
        goto exit;
    }

    status = s->getVolumeData(voldata);
    if (0 != status) {
        PAL_ERR(LOG_TAG,"getVolumeData Failed \n");
        goto error_1;
    }

    PAL_VERBOSE(LOG_TAG,"volume sent:%f \n",(voldata->volume_pair[0].vol));
    voldB = (voldata->volume_pair[0].vol);

    switch (static_cast<uint32_t>(tag)) {
    case TAG_STREAM_VOLUME:
        if (voldB == 0.0f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_15));
        }
        else if (voldB < 0.002172f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_15));
        }
        else if (voldB < 0.004660f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_14));
        }
        else if (voldB < 0.01f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_13));
        }
        else if (voldB < 0.014877f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_12));
        }
        else if (voldB < 0.023646f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_11));
        }
        else if (voldB < 0.037584f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_10));
        }
        else if (voldB < 0.055912f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_9));
        }
        else if (voldB < 0.088869f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_8));
        }
        else if (voldB < 0.141254f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_7));
        }
        else if (voldB < 0.189453f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_6));
        }
        else if (voldB < 0.266840f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_5));
        }
        else if (voldB < 0.375838f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_4));
        }
        else if (voldB < 0.504081f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_3));
        }
        else if (voldB < 0.709987f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_2));
        }
        else if (voldB < 0.9f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_1));
        }
        else if (voldB <= 1.0f) {
            ckv.push_back(std::make_pair(VOLUME,LEVEL_0));
        }
        break;
    case TAG_DEVICE_PP_MBDRC:
        level = s->getGainLevel();
        if (level != -1)
            ckv.push_back(std::make_pair(GAIN, level));
        break;
    case TAG_MODULE_CHANNELS:
        if (sAttr.type == PAL_STREAM_VOICE_UI) {
            stream_config_kv = s->getStreamModifiers();
            if (stream_config_kv.size() == 0 ||
                stream_config_kv[0].second != VUI_STREAM_CFG_SVA) {
                PAL_DBG(LOG_TAG, "Skip fluence ckv for non-SVA case");
                break;
            }

            cap_prof = rm->GetSVACaptureProfile();
            if (!cap_prof) {
                PAL_ERR(LOG_TAG, "Invalid capture profile");
                status = -EINVAL;
                break;
            }

            if (!cap_prof->GetChannels()) {
                PAL_ERR(LOG_TAG, "Invalid channels");
                status = -EINVAL;
                break;
            }
            ckv.push_back(std::make_pair(CHANNELS,
                cap_prof->GetChannels()));
        }
        break;
    case SPKR_PROT_ENABLE :
        status = s->getAssociatedDevices(associatedDevices);
        if (0 != status) {
            PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
            return status;
        }

        for (int i = 0; i < associatedDevices.size(); i++) {
            status = associatedDevices[i]->getDeviceAttributes(&dAttr);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"getAssociatedDevices Failed \n");
                return status;
            }
            if (dAttr.id == PAL_DEVICE_OUT_SPEAKER) {
                if (dAttr.config.ch_info.channels > 1) {
                    PAL_DBG(LOG_TAG, "Multi channel speaker");
                    ckv.push_back(std::make_pair(SPK_PRO_DEV_MAP, LEFT_RIGHT));
                }
                else {
                    PAL_DBG(LOG_TAG, "Mono channel speaker");
                    ckv.push_back(std::make_pair(SPK_PRO_DEV_MAP, RIGHT_MONO));
                }
                break;
            }
        }
        break;
    case SPKR_VI_ENABLE :
        status = s->getAssociatedDevices(associatedDevices);
        if (0 != status) {
            PAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
            return status;
        }

        for (int i = 0; i < associatedDevices.size(); i++) {
            status = associatedDevices[i]->getDeviceAttributes(&dAttr);
            if (0 != status) {
                PAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
                return status;
            }
            if (dAttr.id == PAL_DEVICE_IN_VI_FEEDBACK) {
                if (dAttr.config.ch_info.channels > 1) {
                    PAL_DBG(LOG_TAG, "Multi channel speaker");
                    ckv.push_back(std::make_pair(SPK_PRO_VI_MAP, STEREO_SPKR));
                }
                else {
                    PAL_DBG(LOG_TAG, "Mono channel speaker");
                    ckv.push_back(std::make_pair(SPK_PRO_VI_MAP, RIGHT_SPKR));
                }
                break;
            }
        }
    break;
    default:
        break;
    }

    PAL_VERBOSE(LOG_TAG,"exit status- %d", status);
error_1:
    free(voldata);
exit:
    return status;
}

int PayloadBuilder::populateTagKeyVector(Stream *s, std::vector <std::pair<int,int>> &tkv, int tag, uint32_t* gsltag)
{
    int status = 0;
    PAL_VERBOSE(LOG_TAG,"enter, tag 0x%x", tag);
    struct pal_stream_attributes sAttr;

    status = s->getStreamAttributes(&sAttr);

    if (status != 0) {
        PAL_ERR(LOG_TAG,"stream get attributes failed");
        return status;
    }

    switch (tag) {
    case MUTE_TAG:
       tkv.push_back(std::make_pair(MUTE,ON));
       *gsltag = TAG_MUTE;
       break;
    case UNMUTE_TAG:
       tkv.push_back(std::make_pair(MUTE,OFF));
       *gsltag = TAG_MUTE;
       break;
    case VOICE_SLOW_TALK_OFF:
       tkv.push_back(std::make_pair(TAG_KEY_SLOW_TALK, TAG_VALUE_SLOW_TALK_OFF));
       *gsltag = TAG_STREAM_SLOW_TALK;
       break;
    case VOICE_SLOW_TALK_ON:
       tkv.push_back(std::make_pair(TAG_KEY_SLOW_TALK, TAG_VALUE_SLOW_TALK_ON));
       *gsltag = TAG_STREAM_SLOW_TALK;
       break;
    case PAUSE_TAG:
       tkv.push_back(std::make_pair(PAUSE,ON));
       *gsltag = TAG_PAUSE;
       break;
    case RESUME_TAG:
       tkv.push_back(std::make_pair(PAUSE,OFF));
       *gsltag = TAG_PAUSE;
       break;
    case MFC_SR_8K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_8K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_16K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_16K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_32K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_32K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_44K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_44K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_48K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_48K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_96K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_96K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_192K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_192K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case MFC_SR_384K:
       tkv.push_back(std::make_pair(SAMPLINGRATE,SAMPLINGRATE_384K));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case ECNS_ON_TAG:
       tkv.push_back(std::make_pair(ECNS,ECNS_ON));
       *gsltag = TAG_ECNS;
       break;
    case ECNS_OFF_TAG:
       tkv.push_back(std::make_pair(ECNS,ECNS_OFF));
       *gsltag = TAG_ECNS;
       break;
    case EC_ON_TAG:
       tkv.push_back(std::make_pair(ECNS,EC_ON));
       *gsltag = TAG_ECNS;
       break;
    case NS_ON_TAG:
       tkv.push_back(std::make_pair(ECNS,NS_ON));
       *gsltag = TAG_ECNS;
       break;
    case CHS_1:
       tkv.push_back(std::make_pair(CHANNELS, CHANNELS_1));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case CHS_2:
       tkv.push_back(std::make_pair(CHANNELS, CHANNELS_2));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case CHS_3:
       tkv.push_back(std::make_pair(CHANNELS, CHANNELS_3));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case CHS_4:
       tkv.push_back(std::make_pair(CHANNELS, CHANNELS_4));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case BW_16:
       tkv.push_back(std::make_pair(BITWIDTH, BITWIDTH_16));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case BW_24:
       tkv.push_back(std::make_pair(BITWIDTH, BITWIDTH_24));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case BW_32:
       tkv.push_back(std::make_pair(BITWIDTH, BITWIDTH_32));
       if (sAttr.direction == PAL_AUDIO_INPUT)
            *gsltag = TAG_STREAM_MFC_SR;
       else
            *gsltag = TAG_DEVICE_MFC_SR;
       break;
    case INCALL_RECORD_UPLINK:
       tkv.push_back(std::make_pair(TAG_KEY_MUX_DEMUX_CONFIG, TAG_VALUE_MUX_DEMUX_CONFIG_UPLINK));
       *gsltag = TAG_STREAM_MUX_DEMUX;
       break;
    case INCALL_RECORD_DOWNLINK:
       tkv.push_back(std::make_pair(TAG_KEY_MUX_DEMUX_CONFIG, TAG_VALUE_MUX_DEMUX_CONFIG_DOWNLINK));
       *gsltag = TAG_STREAM_MUX_DEMUX;
       break;
    case INCALL_RECORD_UPLINK_DOWNLINK_MONO:
       tkv.push_back(std::make_pair(TAG_KEY_MUX_DEMUX_CONFIG, TAG_VALUE_MUX_DEMUX_CONFIG_UPLINK_DOWNLINK_MONO));
       *gsltag = TAG_STREAM_MUX_DEMUX;
       break;
    case INCALL_RECORD_UPLINK_DOWNLINK_STEREO:
       tkv.push_back(std::make_pair(TAG_KEY_MUX_DEMUX_CONFIG, TAG_VALUE_MUX_DEMUX_CONFIG_UPLINK_DOWNLINK_STEREO));
       *gsltag = TAG_STREAM_MUX_DEMUX;
       break;
    default:
       PAL_ERR(LOG_TAG,"Tag not supported \n");
       break;
    }

    PAL_VERBOSE(LOG_TAG,"exit status- %d", status);
    return status;
}

void PayloadBuilder::payloadSPConfig(uint8_t** payload, size_t* size, uint32_t miid,
                int param_id, void *param)
{
    struct apm_module_param_data_t* header = NULL;
    uint8_t* payloadInfo = NULL;
    size_t payloadSize = 0, padBytes = 0;

    if (!param) {
        PAL_ERR(LOG_TAG, "Invalid input parameters");
        return;
    }


    switch(param_id) {
        case PARAM_ID_SP_TH_VI_R0T0_CFG :
            {
                param_id_sp_th_vi_r0t0_cfg_t *spConf;
                param_id_sp_th_vi_r0t0_cfg_t *data = NULL;
                vi_r0t0_cfg_t* r0t0 = NULL;
                data = (param_id_sp_th_vi_r0t0_cfg_t *) param;

                payloadSize = sizeof(struct apm_module_param_data_t) +
                              sizeof(param_id_sp_th_vi_r0t0_cfg_t) +
                              sizeof(vi_r0t0_cfg_t) * data->num_speakers;

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;

                spConf = (param_id_sp_th_vi_r0t0_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));
                r0t0 = (vi_r0t0_cfg_t*) (payloadInfo +
                                sizeof(struct apm_module_param_data_t)
                                + sizeof(param_id_sp_th_vi_r0t0_cfg_t));

                spConf->num_speakers = data->num_speakers;
                for(int i = 0; i < data->num_speakers; i++) {
                    r0t0[i].r0_cali_q24 = data->vi_r0t0_cfg[i].r0_cali_q24;
                    r0t0[i].t0_cali_q6 = data->vi_r0t0_cfg[i].t0_cali_q6;
                }
            }
        break;
        case PARAM_ID_SP_VI_OP_MODE_CFG :
            {
                param_id_sp_vi_op_mode_cfg_t *spConf;
                param_id_sp_vi_op_mode_cfg_t *data;
                uint32_t *channelMap;

                data = (param_id_sp_vi_op_mode_cfg_t *) param;

                payloadSize = sizeof(struct apm_module_param_data_t) +
                              sizeof(param_id_sp_vi_op_mode_cfg_t) +
                              sizeof(uint32_t) * data->num_speakers;

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;

                spConf = (param_id_sp_vi_op_mode_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));

                channelMap = (uint32_t *) (payloadInfo +
                                    sizeof(struct apm_module_param_data_t)
                                    + sizeof(param_id_sp_vi_op_mode_cfg_t));

                spConf->num_speakers = data->num_speakers;
                spConf->th_operation_mode = data->th_operation_mode;
                spConf->th_quick_calib_flag = data->th_quick_calib_flag;
                for(int i = 0; i < data->num_speakers; i++) {
                    if (spConf->th_operation_mode == 0) {
                        channelMap[i] = 0;
                    }
                    else if (spConf->th_operation_mode == 1) {
                        channelMap[i] = 0;
                    }
                }
            }
        break;
        case PARAM_ID_SP_VI_CHANNEL_MAP_CFG :
            {
                param_id_sp_vi_channel_map_cfg_t *spConf;
                param_id_sp_vi_channel_map_cfg_t *data;
                int32_t *channelMap;

                data = (param_id_sp_vi_channel_map_cfg_t *) param;

                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_vi_channel_map_cfg_t) +
                                    (sizeof(int32_t) * data->num_ch);

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;

                spConf = (param_id_sp_vi_channel_map_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));
                channelMap = (int32_t *) (payloadInfo +
                                    sizeof(struct apm_module_param_data_t)
                                    + sizeof(param_id_sp_vi_channel_map_cfg_t));

                spConf->num_ch = data->num_ch;
                for (int i = 0; i < data->num_ch; i++) {
                    channelMap[i] = i+1;
                }
            }
        break;
        case PARAM_ID_SP_OP_MODE :
            {
                param_id_sp_op_mode_t *spConf;
                param_id_sp_op_mode_t *data;

                data = (param_id_sp_op_mode_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_op_mode_t);

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;

                spConf = (param_id_sp_op_mode_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));

                spConf->operation_mode = data->operation_mode;
            }
        break;
        case PARAM_ID_SP_EX_VI_MODE_CFG :
            {
                param_id_sp_ex_vi_mode_cfg_t *spConf;
                param_id_sp_ex_vi_mode_cfg_t *data;

                data = (param_id_sp_ex_vi_mode_cfg_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_ex_vi_mode_cfg_t);

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;

                spConf = (param_id_sp_ex_vi_mode_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));

                spConf->operation_mode = data->operation_mode;
            }
        break;
        case PARAM_ID_SP_TH_VI_FTM_CFG:
        case PARAM_ID_SP_TH_VI_V_VALI_CFG :
            {
                param_id_sp_th_vi_ftm_cfg_t *spConf;
                param_id_sp_th_vi_ftm_cfg_t *data;
                vi_th_ftm_cfg_t *ftmCfg;
                std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();

                data = (param_id_sp_th_vi_ftm_cfg_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_th_vi_ftm_cfg_t) +
                                    sizeof(vi_th_ftm_cfg_t) * data->num_ch;

                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;
                spConf = (param_id_sp_th_vi_ftm_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));
                ftmCfg = (vi_th_ftm_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t)
                                + sizeof(param_id_sp_th_vi_ftm_cfg_t));

                spConf->num_ch = data->num_ch;
                for (int i = 0; i < data->num_ch; i++) {
                    ftmCfg[i].wait_time_ms =
                            rm->mSpkrProtModeValue.spkrHeatupTime;
                    ftmCfg[i].ftm_time_ms =
                            rm->mSpkrProtModeValue.operationModeRunTime;
                }
            }
        break;
        case PARAM_ID_SP_TH_VI_FTM_PARAMS:
            {
                param_id_sp_th_vi_ftm_params_t *data;
                data = (param_id_sp_th_vi_ftm_params_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_th_vi_ftm_params_t) +
                                    sizeof(vi_th_ftm_params_t) * data->num_ch;
                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;
            }
        break;
        case PARAM_ID_SP_TH_VI_V_VALI_PARAMS:
            {
                param_id_sp_th_vi_v_vali_params_t *data;
                data = (param_id_sp_th_vi_v_vali_params_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_th_vi_v_vali_params_t) +
                                    sizeof(vi_th_v_vali_params_t) * data->num_ch;
                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);
                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;
            }
        break;
        case PARAM_ID_CPS_LPASS_HW_INTF_CFG:
            {
                lpass_swr_hw_reg_cfg_t *data = NULL;
                lpass_swr_hw_reg_cfg_t *cfgPayload = NULL;
                param_id_cps_lpass_hw_intf_cfg_t *spConf = NULL;
                data = (lpass_swr_hw_reg_cfg_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(lpass_swr_hw_reg_cfg_t) +
                                    sizeof(pkd_reg_addr_t) * data->num_spkr +
                                    sizeof(uint32_t);
                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;
                spConf = (param_id_cps_lpass_hw_intf_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));
                cfgPayload = (lpass_swr_hw_reg_cfg_t * ) (payloadInfo +
                                sizeof(struct apm_module_param_data_t) +
                                sizeof(uint32_t));
                spConf->lpass_hw_intf_cfg_mode = 1;

                memcpy(cfgPayload, data, sizeof(lpass_swr_hw_reg_cfg_t) +
                                sizeof(pkd_reg_addr_t) * data->num_spkr);
            }
        break;
        case PARAM_ID_SP_CPS_STATIC_CFG:
            {
                param_id_sp_cps_static_cfg_t *data = NULL;
                param_id_sp_cps_static_cfg_t *spConf = NULL;

                data = (param_id_sp_cps_static_cfg_t *) param;
                payloadSize = sizeof(struct apm_module_param_data_t) +
                                    sizeof(param_id_sp_cps_static_cfg_t);
                padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

                payloadInfo = (uint8_t*) calloc(1, payloadSize + padBytes);
                if (!payloadInfo) {
                    PAL_ERR(LOG_TAG, "payloadInfo malloc failed %s", strerror(errno));
                    return;
                }
                header = (struct apm_module_param_data_t*) payloadInfo;
                spConf = (param_id_sp_cps_static_cfg_t *) (payloadInfo +
                                sizeof(struct apm_module_param_data_t));
                memcpy(spConf, data, sizeof(param_id_sp_cps_static_cfg_t));
            }
        break;
    }

    header->module_instance_id = miid;
    header->param_id = param_id;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
}
