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

#ifndef _BT_PLUGIN_INTF_H_
#define _BT_PLUGIN_INTF_H_

// import data definition expected from Spf
#include <ar_osal_types.h>
#include <common_enc_dec_api.h>
#include <media_fmt_api.h>
#include <system/audio-base.h>

/*
 * Below enum values are extended from audio_base.h to
 * to keep encoder and decoder type local to bthost_ipc
 * and audio_hal as these are intended only for handshake
 * between IPC lib and Audio HAL.
 */
typedef enum {
    CODEC_TYPE_INVALID = AUDIO_FORMAT_INVALID, // 0xFFFFFFFFUL
    CODEC_TYPE_AAC = AUDIO_FORMAT_AAC,         // 0x04000000UL
    CODEC_TYPE_SBC = AUDIO_FORMAT_SBC,         // 0x1F000000UL
    CODEC_TYPE_APTX = AUDIO_FORMAT_APTX,       // 0x20000000UL
    CODEC_TYPE_APTX_HD = AUDIO_FORMAT_APTX_HD, // 0x21000000UL
    CODEC_TYPE_APTX_DUAL_MONO = 570425344u,    // 0x22000000UL
    CODEC_TYPE_LDAC = AUDIO_FORMAT_LDAC,       // 0x23000000UL
    CODEC_TYPE_CELT = 603979776u,              // 0x24000000UL
    CODEC_TYPE_APTX_AD = 620756992u,           // 0x25000000UL
    CODEC_TYPE_APTX_AD_SPEECH = 637534208u,    // 0x26000000UL
    CODEC_TYPE_PCM = AUDIO_FORMAT_PCM_16_BIT,  // 0x1u
} codec_format_t;

typedef enum {
    ENC = 1,
    DEC
} codec_type;

/* encoder payload sent for BT device */
typedef struct custom_block_s {
    uint32_t param_id;
    uint32_t payload_sz;
    uint8_t  *payload;
} custom_block_t;

typedef struct bt_enc_payload {
    uint32_t channel_count;
    uint32_t bit_format;
    uint32_t sample_rate;
    bool is_abr_enabled;
    uint32_t  num_blks;
    custom_block_t *blocks[];
} bt_enc_payload_t;

typedef struct bt_codec_library_s bt_codec_t;

struct bt_codec_library_s {
    const char *name;
    uint32_t codecFmt;
    codec_type direction;
    bt_enc_payload_t *payload;
    int (*plugin_populate_payload)(bt_codec_t *codec, void *src, void **dst);
    uint64_t (*plugin_get_codec_latency)(bt_codec_t *codec, uint32_t slatency);
    int (*plugin_query_num_codecs)(bt_codec_t *codec);
    void (*close_plugin) (bt_codec_t *codec);
};

typedef int (*config_fn_t) (bt_codec_t *codec, void *src, void **dst);
typedef int (*open_fn_t) (bt_codec_t **codec, uint32_t codecFmt, codec_type direction);

#endif /* _BT_PLUGIN_INTF_H_ */
