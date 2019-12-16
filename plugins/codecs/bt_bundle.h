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

#ifndef _BT_BUNDLE_H_
#define _BT_BUNDLE_H_

#include "bt_intf.h"

#define ENCODER_LATENCY_SBC        10
#define ENCODER_LATENCY_AAC        70
#define DEFAULT_SINK_LATENCY_SBC   140
#define DEFAULT_SINK_LATENCY_AAC   180
#define MODULE_ID_AAC_ENC          0x0700101E
#define MODULE_ID_AAC_DEC          0x0700101F
#define MODULE_ID_SBC_DEC          0x07001039
#define MODULE_ID_SBC_ENC          0x0700103A
#define NUM_CODEC                  4

/* Information about BT SBC encoder configuration
 * This data is used between audio HAL module and
 * BT IPC library to configure DSP encoder
 */
/* Structure to control frame size of AAC encoded frames. */
struct aac_frame_size_control_t {
    /* Type of frame size control: MTU_SIZE / PEAK_BIT_RATE*/
    uint32_t ctl_type;
    /* Control value
     * MTU_SIZE: MTU size in bytes
     * PEAK_BIT_RATE: Peak bitrate in bits per second.
     */
    uint32_t ctl_value;
};

typedef struct audio_aac_encoder_config_s {
    uint32_t enc_mode;       /* LC, SBR, PS */
    uint16_t format_flag;    /* RAW, ADTS */
    uint16_t channels;       /* 1-Mono, 2-Stereo */
    uint32_t sampling_rate;
    uint32_t bitrate;
    uint32_t bits_per_sample;
    struct aac_frame_size_control_t frame_ctl;
} audio_aac_encoder_config_t;

typedef struct audio_sbc_encoder_config_s {
    uint32_t subband;        /* 4, 8 */
    uint32_t blk_len;        /* 4, 8, 12, 16 */
    uint16_t sampling_rate;  /* 44.1khz,48khz */
    uint8_t  channels;       /* 0(Mono),1(Dual_mono),2(Stereo),3(JS) */
    uint8_t  alloc;          /* 0(Loudness),1(SNR) */
    uint8_t  min_bitpool;    /* 2 */
    uint8_t  max_bitpool;    /* 53(44.1khz),51 (48khz) */
    uint32_t bitrate;        /* 320kbps to 512kbps */
    uint32_t bits_per_sample;
} audio_sbc_encoder_config_t;

#endif /* _BT_PLUGIN_BUNDLE_H_ */
