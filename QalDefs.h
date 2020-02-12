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

/** \file qal_defs.h
 *  \brief struture, enum constant defintions of the
 *         QAL(QTI Audio Layer).
 *
 *  This file contains macros, constants, or global variables
 *  exposed to the client of QAL(QTI Audio Layer).
 */

#ifndef QAL_DEFS_H
#define QAL_DEFS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus

#include <map>
#include <string>

extern "C" {
#endif

#define MIXER_PATH_MAX_LENGTH 100

/** Audio stream handle */
typedef void qal_stream_handle_t;

/** Sound Trigger handle */
typedef void qal_st_handle_t;

/** QAL Audio format enumeration */
typedef enum {
    QAL_AUDIO_FMT_DEFAULT_PCM = 0x1,                   /**< Default PCM*/
    QAL_AUDIO_FMT_DEFAULT_COMPRESSED = 0x2,            /**< Default Compressed*/
    QAL_AUDIO_FMT_MP3 = QAL_AUDIO_FMT_DEFAULT_COMPRESSED,
    QAL_AUDIO_FMT_AAC = 0x3,
    QAL_AUDIO_FMT_AAC_ADTS = 0x4,
    QAL_AUDIO_FMT_AAC_ADIF = 0x5,
    QAL_AUDIO_FMT_AAC_LATM = 0x6,
    QAL_AUDIO_FMT_WMA_STD = 0x7,
    QAL_AUDIO_FMT_ALAC = 0x8,
    QAL_AUDIO_FMT_APE = 0x9,
    QAL_AUDIO_FMT_WMA_PRO = 0xA,
    QAL_AUDIO_FMT_FLAC = 0xB,
    QAL_AUDIO_FMT_FLAC_OGG = 0xC,
    QAL_AUDIO_FMT_VORBIS = 0xD,
    QAL_AUDIO_FMT_COMPRESSED_RANGE_BEGIN = 0xF0000000,  /* Reserved for beginning of compressed codecs */
    QAL_AUDIO_FMT_COMPRESSED_EXTENDED_RANGE_BEGIN   = 0xF0000F00,  /* Reserved for beginning of 3rd party codecs */
    QAL_AUDIO_FMT_COMPRESSED_EXTENDED_RANGE_END     = 0xF0000FFF,  /* Reserved for beginning of 3rd party codecs */
    QAL_AUDIO_FMT_COMPRESSED_RANGE_END   = QAL_AUDIO_FMT_COMPRESSED_EXTENDED_RANGE_END /* Reserved for beginning of 3rd party codecs */
} qal_audio_fmt_t;

#define PCM_24_BIT_PACKED (0x6u)
#define PCM_32_BIT (0x3u)
#define PCM_16_BIT (0x1u)

#define SAMPLE_RATE_192000 192000

#ifdef __cplusplus
static const std::map<std::string, qal_audio_fmt_t> QalAudioFormatMap
{
    { "PCM",  QAL_AUDIO_FMT_DEFAULT_PCM},
    { "MP3",  QAL_AUDIO_FMT_MP3},
    { "AAC",  QAL_AUDIO_FMT_AAC},
    { "AAC_ADTS",  QAL_AUDIO_FMT_AAC_ADTS},
    { "AAC_ADIF",  QAL_AUDIO_FMT_AAC_ADIF},
    { "AAC_LATM",  QAL_AUDIO_FMT_AAC_LATM},
    { "WMA_STD",  QAL_AUDIO_FMT_WMA_STD},
    { "ALAC", QAL_AUDIO_FMT_ALAC},
    { "APE", QAL_AUDIO_FMT_APE},
    { "WMA_PRO", QAL_AUDIO_FMT_WMA_PRO},
    { "FLAC", QAL_AUDIO_FMT_FLAC},
    { "FLAC_OGG", QAL_AUDIO_FMT_FLAC_OGG},
    { "VORBIS", QAL_AUDIO_FMT_VORBIS}

};
#endif

struct qal_snd_dec_aac {
    uint16_t audio_obj_type;
    uint16_t pce_bits_size;
};

struct qal_snd_dec_wma {
    uint32_t fmt_tag;
    uint32_t super_block_align;
    uint32_t bits_per_sample;
    uint32_t channelmask;
    uint32_t encodeopt;
    uint32_t encodeopt1;
    uint32_t encodeopt2;
    uint32_t avg_bit_rate;
};

struct qal_snd_dec_alac {
    uint32_t frame_length;
    uint8_t compatible_version;
    uint8_t bit_depth;
    uint8_t pb;
    uint8_t mb;
    uint8_t kb;
    uint8_t num_channels;
    uint16_t max_run;
    uint32_t max_frame_bytes;
    uint32_t avg_bit_rate;
    uint32_t sample_rate;
    uint32_t channel_layout_tag;
};

struct qal_snd_dec_ape {
    uint16_t compatible_version;
    uint16_t compression_level;
    uint32_t format_flags;
    uint32_t blocks_per_frame;
    uint32_t final_frame_blocks;
    uint32_t total_frames;
    uint16_t bits_per_sample;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t seek_table_present;
};

struct qal_snd_dec_flac {
    uint16_t sample_size;
    uint16_t min_blk_size;
    uint16_t max_blk_size;
    uint16_t min_frame_size;
    uint16_t max_frame_size;
};

struct qal_snd_dec_vorbis {
    uint32_t bit_stream_fmt;
};

typedef struct qal_key_value_pair_s {
    uint32_t key; /**< key */
    uint32_t value; /**< value */
} qal_key_value_pair_t;

typedef struct qal_key_vector_s {
    size_t num_tkvs;  /**< number of key value pairs */
    qal_key_value_pair_t *kvp;  /**< vector of key value pairs */
} qal_key_vector_t;

typedef enum {
    PARAM_NONTKV,
    PARAM_TKV,
} qal_param_type_t;

typedef struct qal_effect_custom_payload_s {
    uint32_t paramId;
    uint32_t *data;
} qal_effect_custom_payload_t;

typedef struct effect_qal_payload_s {
    qal_param_type_t isTKV;      /* payload type: 0->non-tkv 1->tkv*/
    uint32_t tag;
    uint32_t  payloadSize;
    uint32_t  *payload; /* TKV uses qal_key_vector_t, while nonTKV uses qal_effect_custom_payload_t */
} effect_qal_payload_t;

/** Audio parameter data*/
typedef union {
    struct qal_snd_dec_aac aac_dec;
    struct qal_snd_dec_wma wma_dec;
    struct qal_snd_dec_alac alac_dec;
    struct qal_snd_dec_ape ape_dec;
    struct qal_snd_dec_flac flac_dec;
    struct qal_snd_dec_vorbis vorbis_dec;
} qal_snd_dec_t;

/** Audio parameter data*/
typedef struct qal_param_payload_s {
    bool has_fluence;                     /**  true if fluence is to be enabled */
    bool has_effect;
    qal_snd_dec_t qal_snd_dec;
    uint32_t *effect_payload;
} qal_param_payload;

/** Audio channel map enumeration*/
typedef enum {
    QAL_CHMAP_CHANNEL_FL = 1,               /**< Front right channel. */
    QAL_CHMAP_CHANNEL_FR = 2,               /**< Front center channel. */
    QAL_CHMAP_CHANNEL_C = 3,                /**< Left surround channel. */
    QAL_CHMAP_CHANNEL_LS = 4,               /** Right surround channel. */
    QAL_CHMAP_CHANNEL_RS = 5,               /** Low frequency effect channel. */
    QAL_CHMAP_CHANNEL_LFE = 6,              /** Center surround channel; */
    QAL_CHMAP_CHANNEL_RC = 7,               /**< rear center channel. */
    QAL_CHMAP_CHANNEL_LB = 8,               /**< rear left channel. */
    QAL_CHMAP_CHANNEL_RB = 9,               /**<  rear right channel. */
    QAL_CHMAP_CHANNEL_TS = 10,              /**< Top surround channel. */
    QAL_CHMAP_CHANNEL_TFC = 11,             /**< Top front center channel. or Center vertical height channel.*/
    QAL_CHMAP_CHANNEL_MS = 12,              /**< Mono surround channel. */
    QAL_CHMAP_CHANNEL_FLC = 13,             /**< Front left of center channel. */
    QAL_CHMAP_CHANNEL_FRC = 14,             /**< Front right of center channel. */
    QAL_CHMAP_CHANNEL_RLC = 15,             /**< Rear left of center channel. */
    QAL_CHMAP_CHANNEL_RRC = 16,             /**< Rear right of center channel. */
    QAL_CHMAP_CHANNEL_LFE2 = 17,            /**< Secondary low frequency effect channel. */
    QAL_CHMAP_CHANNEL_SL = 18,              /**< Side left channel. */
    QAL_CHMAP_CHANNEL_SR = 19,              /**< Side right channel. */
    QAL_CHMAP_CHANNEL_TFL = 20,             /**< Top front left channel or Left vertical height channel */
    QAL_CHMAP_CHANNEL_TFR = 21,             /**< Top front right channel or Right vertical height channel. */
    QAL_CHMAP_CHANNEL_TC = 22,              /**< Top center channel. */
    QAL_CHMAP_CHANNEL_TBL = 23,             /**< Top back left channel. */
    QAL_CHMAP_CHANNEL_TBR = 24,             /**< Top back right channel. */
    QAL_CHMAP_CHANNEL_TSL = 25,             /**< Top side left channel. */
    QAL_CHMAP_CHANNEL_TSR = 26,             /**< Top side right channel. */
    QAL_CHMAP_CHANNEL_TBC = 27,             /**< Top back center channel. */
    QAL_CHMAP_CHANNEL_BFC = 28,             /**< Bottom front center channel. */
    QAL_CHMAP_CHANNEL_BFL = 29,             /**< Bottom front left channel. */
    QAL_CHMAP_CHANNEL_BFR = 30,             /**< Bottom front right channel. */
    QAL_CHMAP_CHANNEL_LW = 31,              /**< Left wide channel. */
    QAL_CHMAP_CHANNEL_RW = 32,              /**< Right wide channel. */
    QAL_CHMAP_CHANNEL_LSD = 33,             /**< Left side direct channel. */
    QAL_CHMAP_CHANNEL_RSD = 34,             /**< Left side direct channel. */
} qal_channel_map;

/** Audio channel info data structure */
struct qal_channel_info {
    uint16_t channels;      /**< number of channels*/
    uint8_t  ch_map[0];     /**< ch_map value per channel. */
};

/** Audio stream direction enumeration */
typedef enum {
    QAL_AUDIO_OUTPUT        = 0x1, /**< playback usecases*/
    QAL_AUDIO_INPUT         = 0x2, /**< capture/voice activation usecases*/
    QAL_AUDIO_INPUT_OUTPUT  = 0x3, /**< transcode usecases*/
} qal_stream_direction_t;

/** Audio Voip TX Effect enumeration */
typedef enum {
    QAL_AUDIO_EFFECT_EC        = 0x1, /**< Echo Cancellation*/
    QAL_AUDIO_EFFECT_NS        = 0x2, /**< Noise Suppression*/
    QAL_AUDIO_EFFECT_ECNS      = 0x3, /**< EC + NS*/
} qal_audio_effect_t;

/** Audio stream types */
typedef enum {
    QAL_STREAM_LOW_LATENCY = 1,      /**< :low latency, higher power*/
    QAL_STREAM_DEEP_BUFFER,          /**< :low power, higher latency*/
    QAL_STREAM_COMPRESSED,           /**< :compresssed audio*/
    QAL_STREAM_VOIP,                 /**< :pcm voip audio*/
    QAL_STREAM_VOIP_RX,              /**< :pcm voip audio downlink*/
    QAL_STREAM_VOIP_TX,              /**< :pcm voip audio uplink*/
    QAL_STREAM_VOICE_CALL_MUSIC,     /**< :incall music */
    QAL_STREAM_GENERIC,              /**< :generic playback audio*/
    QAL_STREAM_RAW,                  /**< pcm no post processing*/
    QAL_STREAM_VOICE_ACTIVATION,     /**< voice activation*/
    QAL_STREAM_VOICE_CALL_RX,        /**< incall record, downlink */
    QAL_STREAM_VOICE_CALL_TX,        /**< incall record, uplink */
    QAL_STREAM_VOICE_CALL_RX_TX,     /**< incall record, uplink & Downlink */

    QAL_STREAM_VOICE_CALL,           /**< voice call */

    QAL_STREAM_LOOPBACK,             /**< loopback */
    QAL_STREAM_TRANSCODE,            /**< audio transcode */
    QAL_STREAM_VOICE_UI,             /**< voice ui */
    QAL_STREAM_PCM_OFFLOAD,          /**< pcm offload audio */
} qal_stream_type_t;

/** Audio devices available for enabling streams */
typedef enum {
    //OUTPUT DEVICES
    QAL_DEVICE_OUT_MIN = 0,
    QAL_DEVICE_NONE = 1, /**< for transcode usecases*/
    QAL_DEVICE_OUT_HANDSET = 2,
    QAL_DEVICE_OUT_SPEAKER = 3,
    QAL_DEVICE_OUT_WIRED_HEADSET = 4,
    QAL_DEVICE_OUT_WIRED_HEADPHONE = 5, /**< Wired headphones without mic*/
    QAL_DEVICE_OUT_LINE = 6,
    QAL_DEVICE_OUT_BLUETOOTH_SCO = 7,
    QAL_DEVICE_OUT_BLUETOOTH_A2DP = 8,
    QAL_DEVICE_OUT_AUX_DIGITAL = 9,
    QAL_DEVICE_OUT_HDMI = 10,
    QAL_DEVICE_OUT_USB_DEVICE = 11,
    QAL_DEVICE_OUT_USB_HEADSET = 12,
    QAL_DEVICE_OUT_SPDIF = 13,
    QAL_DEVICE_OUT_FM = 14,
    QAL_DEVICE_OUT_AUX_LINE = 15,
    QAL_DEVICE_OUT_PROXY = 16,
    // Add new OUT devices here, increment MAX and MIN below when you do so
    QAL_DEVICE_OUT_MAX = 17,
    //INPUT DEVICES
    QAL_DEVICE_IN_MIN = QAL_DEVICE_OUT_MAX,
    QAL_DEVICE_IN_HANDSET_MIC = QAL_DEVICE_IN_MIN +1,
    QAL_DEVICE_IN_SPEAKER_MIC = QAL_DEVICE_IN_MIN + 2,
    QAL_DEVICE_IN_TRI_MIC = QAL_DEVICE_IN_MIN + 3,
    QAL_DEVICE_IN_QUAD_MIC = QAL_DEVICE_IN_MIN + 4,
    QAL_DEVICE_IN_EIGHT_MIC = QAL_DEVICE_IN_MIN + 5,
    QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET = QAL_DEVICE_IN_MIN + 6,
    QAL_DEVICE_IN_WIRED_HEADSET = QAL_DEVICE_IN_MIN + 7,
    QAL_DEVICE_IN_AUX_DIGITAL = QAL_DEVICE_IN_MIN + 8,
    QAL_DEVICE_IN_HDMI = QAL_DEVICE_IN_MIN + 9,
    QAL_DEVICE_IN_USB_ACCESSORY = QAL_DEVICE_IN_MIN + 10,
    QAL_DEVICE_IN_USB_DEVICE = QAL_DEVICE_IN_MIN + 11,
    QAL_DEVICE_IN_USB_HEADSET = QAL_DEVICE_IN_MIN + 12,
    QAL_DEVICE_IN_FM_TUNER = QAL_DEVICE_IN_MIN + 13,
    QAL_DEVICE_IN_LINE = QAL_DEVICE_IN_MIN + 14,
    QAL_DEVICE_IN_SPDIF = QAL_DEVICE_IN_MIN + 15,
    QAL_DEVICE_IN_PROXY = QAL_DEVICE_IN_MIN + 16,
    QAL_DEVICE_IN_HANDSET_VA_MIC = QAL_DEVICE_IN_MIN + 17,
    QAL_DEVICE_IN_BLUETOOTH_A2DP = QAL_DEVICE_IN_MIN + 18,
    QAL_DEVICE_IN_HEADSET_VA_MIC = QAL_DEVICE_IN_MIN + 19,
    // Add new IN devices here, increment MAX and MIN below when you do so
    QAL_DEVICE_IN_MAX = QAL_DEVICE_IN_MIN + 20,
} qal_device_id_t;

#ifdef __cplusplus
static const std::map<std::string, qal_device_id_t> deviceIdLUT {
    {std::string{ "QAL_DEVICE_NONE" },                     QAL_DEVICE_NONE},
    {std::string{ "QAL_DEVICE_OUT_HANDSET" },             QAL_DEVICE_OUT_HANDSET},
    {std::string{ "QAL_DEVICE_OUT_SPEAKER" },              QAL_DEVICE_OUT_SPEAKER},
    {std::string{ "QAL_DEVICE_OUT_WIRED_HEADSET" },        QAL_DEVICE_OUT_WIRED_HEADSET},
    {std::string{ "QAL_DEVICE_OUT_WIRED_HEADPHONE" },      QAL_DEVICE_OUT_WIRED_HEADPHONE},
    {std::string{ "QAL_DEVICE_OUT_LINE" },                 QAL_DEVICE_OUT_LINE},
    {std::string{ "QAL_DEVICE_OUT_BLUETOOTH_SCO" },        QAL_DEVICE_OUT_BLUETOOTH_SCO},
    {std::string{ "QAL_DEVICE_OUT_BLUETOOTH_A2DP" },       QAL_DEVICE_OUT_BLUETOOTH_A2DP},
    {std::string{ "QAL_DEVICE_OUT_AUX_DIGITAL" },          QAL_DEVICE_OUT_AUX_DIGITAL},
    {std::string{ "QAL_DEVICE_OUT_HDMI" },                 QAL_DEVICE_OUT_HDMI},
    {std::string{ "QAL_DEVICE_OUT_USB_DEVICE" },           QAL_DEVICE_OUT_USB_DEVICE},
    {std::string{ "QAL_DEVICE_OUT_USB_HEADSET" },          QAL_DEVICE_OUT_USB_HEADSET},
    {std::string{ "QAL_DEVICE_OUT_SPDIF" },                QAL_DEVICE_OUT_SPDIF},
    {std::string{ "QAL_DEVICE_OUT_FM" },                   QAL_DEVICE_OUT_FM},
    {std::string{ "QAL_DEVICE_OUT_AUX_LINE" },             QAL_DEVICE_OUT_AUX_LINE},
    {std::string{ "QAL_DEVICE_OUT_PROXY" },                QAL_DEVICE_OUT_PROXY},
    {std::string{ "QAL_DEVICE_IN_HANDSET_MIC" },           QAL_DEVICE_IN_HANDSET_MIC},
    {std::string{ "QAL_DEVICE_IN_SPEAKER_MIC" },           QAL_DEVICE_IN_SPEAKER_MIC},
    {std::string{ "QAL_DEVICE_IN_TRI_MIC" },               QAL_DEVICE_IN_TRI_MIC},
    {std::string{ "QAL_DEVICE_IN_QUAD_MIC" },              QAL_DEVICE_IN_QUAD_MIC},
    {std::string{ "QAL_DEVICE_IN_EIGHT_MIC" },             QAL_DEVICE_IN_EIGHT_MIC},
    {std::string{ "QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET" }, QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET},
    {std::string{ "QAL_DEVICE_IN_WIRED_HEADSET" },         QAL_DEVICE_IN_WIRED_HEADSET},
    {std::string{ "QAL_DEVICE_IN_AUX_DIGITAL" },           QAL_DEVICE_IN_AUX_DIGITAL},
    {std::string{ "QAL_DEVICE_IN_HDMI" },                  QAL_DEVICE_IN_HDMI},
    {std::string{ "QAL_DEVICE_IN_USB_ACCESSORY" },         QAL_DEVICE_IN_USB_ACCESSORY},
    {std::string{ "QAL_DEVICE_IN_USB_DEVICE" },            QAL_DEVICE_IN_USB_DEVICE},
    {std::string{ "QAL_DEVICE_IN_USB_HEADSET" },           QAL_DEVICE_IN_USB_HEADSET},
    {std::string{ "QAL_DEVICE_IN_FM_TUNER" },              QAL_DEVICE_IN_FM_TUNER},
    {std::string{ "QAL_DEVICE_IN_LINE" },                  QAL_DEVICE_IN_LINE},
    {std::string{ "QAL_DEVICE_IN_SPDIF" },                 QAL_DEVICE_IN_SPDIF},
    {std::string{ "QAL_DEVICE_IN_PROXY" },                 QAL_DEVICE_IN_PROXY},
    {std::string{ "QAL_DEVICE_IN_HANDSET_VA_MIC" },        QAL_DEVICE_IN_HANDSET_VA_MIC},
    {std::string{ "QAL_DEVICE_IN_BLUETOOTH_A2DP" },        QAL_DEVICE_IN_BLUETOOTH_A2DP},
    {std::string{ "QAL_DEVICE_IN_HEADSET_VA_MIC" },        QAL_DEVICE_IN_HEADSET_VA_MIC}
};

//reverse mapping
static const std::map<uint32_t, std::string> deviceNameLUT {
    {QAL_DEVICE_NONE,                     std::string{"QAL_DEVICE_NONE"}},
    {QAL_DEVICE_OUT_HANDSET,              std::string{"QAL_DEVICE_OUT_HANDSET"}},
    {QAL_DEVICE_OUT_SPEAKER,              std::string{"QAL_DEVICE_OUT_SPEAKER"}},
    {QAL_DEVICE_OUT_WIRED_HEADSET,        std::string{"QAL_DEVICE_OUT_WIRED_HEADSET"}},
    {QAL_DEVICE_OUT_WIRED_HEADPHONE,      std::string{"QAL_DEVICE_OUT_WIRED_HEADPHONE"}},
    {QAL_DEVICE_OUT_LINE,                 std::string{"QAL_DEVICE_OUT_LINE"}},
    {QAL_DEVICE_OUT_BLUETOOTH_SCO,        std::string{"QAL_DEVICE_OUT_BLUETOOTH_SCO"}},
    {QAL_DEVICE_OUT_BLUETOOTH_A2DP,       std::string{"_DEVICE_OUT_BLUETOOTH_A2DP"}},
    {QAL_DEVICE_OUT_AUX_DIGITAL,          std::string{"_DEVICE_OUT_AUX_DIGITAL"}},
    {QAL_DEVICE_OUT_HDMI,                 std::string{"_DEVICE_OUT_HDMI"}},
    {QAL_DEVICE_OUT_USB_DEVICE,           std::string{"_DEVICE_OUT_USB_DEVICE"}},
    {QAL_DEVICE_OUT_USB_HEADSET,          std::string{"_DEVICE_OUT_USB_HEADSET"}},
    {QAL_DEVICE_OUT_SPDIF,                std::string{"_DEVICE_OUT_SPDIF"}},
    {QAL_DEVICE_OUT_FM,                   std::string{"_DEVICE_OUT_FM"}},
    {QAL_DEVICE_OUT_AUX_LINE,             std::string{"_DEVICE_OUT_AUX_LINE"}},
    {QAL_DEVICE_OUT_PROXY,                std::string{"_DEVICE_OUT_PROXY"}},
    {QAL_DEVICE_IN_HANDSET_MIC,           std::string{"_DEVICE_IN_HANDSET_MIC"}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           std::string{"_DEVICE_IN_SPEAKER_MIC"}},
    {QAL_DEVICE_IN_TRI_MIC,               std::string{"_DEVICE_IN_TRI_MIC"}},
    {QAL_DEVICE_IN_QUAD_MIC,              std::string{"_DEVICE_IN_QUAD_MIC"}},
    {QAL_DEVICE_IN_EIGHT_MIC,             std::string{"_DEVICE_IN_EIGHT_MIC"}},
    {QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, std::string{"_DEVICE_IN_BLUETOOTH_SCO_HEADSET"}},
    {QAL_DEVICE_IN_WIRED_HEADSET,         std::string{"_DEVICE_IN_WIRED_HEADSET"}},
    {QAL_DEVICE_IN_AUX_DIGITAL,           std::string{"_DEVICE_IN_AUX_DIGITAL"}},
    {QAL_DEVICE_IN_HDMI,                  std::string{"_DEVICE_IN_HDMI"}},
    {QAL_DEVICE_IN_USB_ACCESSORY,         std::string{"_DEVICE_IN_USB_ACCESSORY"}},
    {QAL_DEVICE_IN_USB_DEVICE,            std::string{"_DEVICE_IN_USB_DEVICE"}},
    {QAL_DEVICE_IN_USB_HEADSET,           std::string{"_DEVICE_IN_USB_HEADSET"}},
    {QAL_DEVICE_IN_FM_TUNER,              std::string{"_DEVICE_IN_FM_TUNER"}},
    {QAL_DEVICE_IN_LINE,                  std::string{"_DEVICE_IN_LINE"}},
    {QAL_DEVICE_IN_SPDIF,                 std::string{"_DEVICE_IN_SPDIF"}},
    {QAL_DEVICE_IN_PROXY,                 std::string{"_DEVICE_IN_PROXY"}},
    {QAL_DEVICE_IN_HANDSET_VA_MIC,        std::string{"_DEVICE_IN_HANDSET_VA_MIC"}},
    {QAL_DEVICE_IN_BLUETOOTH_A2DP,        std::string{"_QAL_DEVICE_IN_BLUETOOTH_A2DP"}},
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        std::string{"QAL_DEVICE_IN_HEADSET_VA_MIC"}}
};
#endif


/* type of asynchronous write callback events. Mutually exclusive */
typedef enum {
    QAL_STREAM_CBK_EVENT_WRITE_READY, /* non blocking write completed */
    QAL_STREAM_CBK_EVENT_DRAIN_READY,  /* drain completed */
    QAL_STREAM_CBK_EVENT_ERROR, /* stream hit some error, let AF take action */
} qal_stream_callback_event_t;

typedef enum {
    QAL_STREAM_LOOPBACK_PCM,
    QAL_STREAM_LOOPBACK_HFP_RX,
    QAL_STREAM_LOOPBACK_HFP_TX,
    QAL_STREAM_LOOPBACK_COMPRESS,
} qal_stream_loopback_type_t;

struct qal_stream_info {
    int64_t version;                    /** version of structure*/
    int64_t size;                       /** size of structure*/
    int64_t duration_us;                /** duration in microseconds, -1 if unknown */
    bool has_video;                     /** optional, true if stream is tied to a video stream */
    bool is_streaming;                  /** true if streaming, false if local playback */
    int32_t loopback_type;              /** used only if stream_type is LOOPBACK. One of the */
                                        /** enums defined in enum qal_stream_loopback_type */
    //qal_audio_attributes_t usage;       /** Not sure if we make use of this */
};

struct qal_voice_record_info {
    int64_t version;                    /** version of structure*/
    int64_t size;                       /** size of structure*/
    uint32_t record_direction;         /** use direction enum to indicate content to be record */
};

struct qal_voice_call_info {
     uint32_t VSID;
     uint32_t tty_mode;
};

typedef enum {
    VOICEMMODE1 = 0x11C05000,
    VOICEMMODE2 = 0x11DC5000,
    VOICELBMMODE1 = 0x12006000,
    VOICELBMMODE2 = 0x121C6000,
}qal_VSID_t;

typedef enum {
    QAL_TTY_OFF = 0,
    QAL_TTY_FULL = 1,
    QAL_TTY_VCO = 2,
    QAL_TTY_HCO = 3,
}qal_tty_t;

typedef union {
    struct qal_stream_info opt_stream_info; /* optional */
    struct qal_voice_record_info voice_rec_info; /* mandatory */
    struct qal_voice_call_info voice_call_info; /* manatory for voice call*/
} qal_stream_info_t;

/** Media configuraiton */
struct qal_media_config {
    uint32_t sample_rate;                /**< sample rate */
    uint32_t bit_width;                  /**< bit width */
    struct qal_channel_info *ch_info;    /**< channel info */
    qal_audio_fmt_t aud_fmt_id;          /**< audio format id*/
//    qal_audio_fmt_cfg_t aud_fmt_cfg;     /**< audio format configuration */
};

/** Android Media configuraiton  */
typedef struct dynamic_media_config {
    uint32_t sample_rate;                /**< sample rate */
    uint32_t format;                     /**< format */
    uint32_t mask;                       /**< channel mask */
} dynamic_media_config_t;

/**  Available stream flags of an audio session*/
typedef enum {
    QAL_STREAM_FLAG_TIMESTAMP,          /**< Enable time stamps associated to audio buffers  */
    QAL_STREAM_FLAG_NON_BLOCKING,       /**< Stream IO operations are non blocking */
} qal_stream_flags_t;

#define QAL_STREAM_FLAG_NON_BLOCKING_MASK 0x2

/**< QAL stream attributes to be specified, used in qal_stream_open cmd */
struct qal_stream_attributes {
    qal_stream_type_t type;                      /**<  stream type */
    qal_stream_info_t info;
    qal_stream_flags_t flags;                    /**<  stream flags */
    qal_stream_direction_t direction;            /**<  direction of the streams */
    struct qal_media_config in_media_config;     /**<  media config of the input audio samples */
    struct qal_media_config out_media_config;    /**<  media config of the output audio samples */
};


/**< Key value pair to identify the topology of a usecase from default  */
struct modifier_kv  {
    uint32_t key;
    uint32_t value;
};

/** Metadata flags */
enum {
    QAL_META_DATA_FLAGS_NONE = 0,
};

/** metadata flags, can be OR'able */
typedef uint32_t qal_meta_data_flags_t;

/** QAL buffer structure used for reading/writing buffers from/to the stream */
struct qal_buffer {
    void *buffer;                  /**<  buffer pointer */
    size_t size;                   /**< number of bytes */
    size_t offset;                 /**< offset in buffer from where valid byte starts */
    struct timespec *ts;           /**< timestmap */
    qal_meta_data_flags_t flags;   /**< meta data flags */
};

/** channel mask and volume pair */
struct qal_channel_vol_kv {
    uint32_t channel_mask;       /**< channel mask */
    float vol;                   /**< gain of the channel mask */
};

/** Volume data strucutre defintion used as argument for volume command */
struct qal_volume_data {
    uint32_t no_of_volpair;                    /**< no of volume pairs*/
    struct qal_channel_vol_kv volume_pair[0];     /**< channel mask and volume pair */
};

struct qal_time_us {
    uint32_t value_lsw;   /** Lower 32 bits of 64 bit time value in microseconds */
    uint32_t value_msw;   /** Upper 32 bits of 64 bit time value in microseconds */
};

/** Timestamp strucutre defintion used as argument for
 *  gettimestamp api */
struct qal_session_time {
    struct qal_time_us session_time;   /** Value of the current session time in microseconds */
    struct qal_time_us absolute_time;  /** Value of the absolute time in microseconds */
    struct qal_time_us timestamp;      /** Value of the last processed time stamp in microseconds */
};

/** EVENT configurations data strucutre defintion used as
 *  argument for mute command */
//typedef union {
//} qal_event_cfg_t;

/** event id of the event generated*/
typedef uint32_t qal_event_id;

typedef enum {
    /** request notification when all accumlated data has be
     *  drained.*/
    QAL_DRAIN,
    /** request notification when drain completes shortly before all
     *  accumlated data of the current track has been played out */
    QAL_DRAIN_PARTIAL,
} qal_drain_type_t;

typedef enum {
    QAL_PARAM_ID_LOAD_SOUND_MODEL,
    QAL_PARAM_ID_RECOGNITION_CONFIG,
    QAL_PARAM_ID_FLUENCE_ON_OFF,
    QAL_PARAM_ID_DIRECTION_OF_ARRIVAL,
    QAL_PARAM_ID_UIEFFECT,
    QAL_PARAM_ID_STOP_BUFFERING,

    /* Non-Stream Specific Parameters*/
    QAL_PARAM_ID_DEVICE_CONNECTION,
    QAL_PARAM_ID_SCREEN_STATE,
    QAL_PARAM_ID_CHARGING_STATE,
    QAL_PARAM_ID_DEVICE_ROTATION,
    QAL_PARAM_ID_BT_SCO,
    QAL_PARAM_ID_BT_SCO_WB,
    QAL_PARAM_ID_BT_A2DP_RECONFIG,
    QAL_PARAM_ID_BT_A2DP_RECONFIG_SUPPORTED,
    QAL_PARAM_ID_BT_A2DP_SUSPENDED,
    QAL_PARAM_ID_DEVICE_CAPABILITY,
}qal_param_id_type_t;

/** HDMI/DP */
// START: MST ==================================================
#define MAX_CONTROLLERS 1
#define MAX_STREAMS_PER_CONTROLLER 2
// END: MST ==================================================

/** Audio parameter data*/

struct qal_param_disp_port_config_params {
    int controller;
    int stream;
};

struct qal_usb_device_address {
    int card_id;
    int device_num;
};

typedef union {
    struct qal_param_disp_port_config_params dp_config;
    struct qal_usb_device_address usb_addr;
} qal_device_config_t;

/* Payload For ID: QAL_PARAM_ID_DEVICE_CONNECTION
 * Description   : Device Connection
*/
typedef struct qal_param_device_connection {
    qal_device_id_t   id;
    bool              connection_state;
    qal_device_config_t device_config;
}qal_param_device_connection_t;

/* Payload For ID: QAL_PARAM_ID_DEVICE_CAPABILITY
 * Description   : get Device Capability
*/
 typedef struct qal_param_device_capability {
  qal_device_id_t   id;
  struct qal_usb_device_address addr;
  bool              is_playback;
  struct dynamic_media_config *config;
}qal_param_device_capability_t;

/* Payload For ID: QAL_PARAM_ID_SCREEN_STATE
 * Description   : Screen State
*/
typedef struct qal_param_screen_state {
    bool              screen_state;
} qal_param_screen_state_t;


/* Payload For ID: QAL_PARAM_ID_BT_SCO*
 * Description   : BT SCO related device parameters
*/
typedef struct qal_param_btsco {
    bool     bt_sco_on;
    bool     bt_wb_speech_enabled;
} qal_param_btsco_t;

/* Payload For ID: QAL_PARAM_ID_BT_A2DP*
 * Description   : A2DP related device setParameters
*/
typedef struct qal_param_bta2dp {
    int32_t  reconfig_supported;
    bool     reconfigured;
    bool     a2dp_suspended;
} qal_param_bta2dp_t;

/**< QAL device */
struct qal_device {
    qal_device_id_t id;                     /**<  device id */
    struct qal_media_config config;         /**<  media config of the device */
    struct qal_usb_device_address address;
};

#define QAL_SOUND_TRIGGER_MAX_STRING_LEN 64     /* max length of strings in properties or descriptor structs */
#define QAL_SOUND_TRIGGER_MAX_LOCALE_LEN 6      /* max length of locale string. e.g en_US */
#define QAL_SOUND_TRIGGER_MAX_USERS 10      /* max number of concurrent users */
#define QAL_SOUND_TRIGGER_MAX_PHRASES 10    /* max number of concurrent phrases */

/** used to identify the sound model type for the session */
typedef enum {
    QAL_SOUND_MODEL_TYPE_UNKNOWN = -1,        /* use for unspecified sound model type */
    QAL_SOUND_MODEL_TYPE_KEYPHRASE = 0,       /* use for key phrase sound models */
    QAL_SOUND_MODEL_TYPE_GENERIC = 1          /* use for all models other than keyphrase */
} qal_st_sound_model_type_t;

struct st_uuid_t {
    uint32_t timeLow;
    uint16_t timeMid;
    uint16_t timeHiAndVersion;
    uint16_t clockSeq;
    uint8_t node[6];
};

/** sound model structure passed in by ST Client during qal_st_load_sound_model() */
struct qal_st_sound_model {
    qal_st_sound_model_type_t type;           /* model type. e.g. QAL_SOUND_MODEL_TYPE_KEYPHRASE */
    struct st_uuid_t          uuid;           /* unique sound model ID. */
    struct st_uuid_t          vendor_uuid;    /* unique vendor ID. Identifies the engine the
                                                  sound model was build for */
    uint32_t                  data_size;      /* size of opaque model data */
    uint32_t                  data_offset;    /* offset of opaque data start from head of struct
                                                  e.g sizeof struct qal_st_sound_model) */
};

/** key phrase descriptor */
struct qal_st_phrase {
    uint32_t    id;                                 /**< keyphrase ID */
    uint32_t    recognition_mode;                   /**< recognition modes supported by this key phrase */
    uint32_t    num_users;                          /**< number of users in the key phrase */
    uint32_t    users[QAL_SOUND_TRIGGER_MAX_USERS]; /**< users ids: (not uid_t but sound trigger
                                                        specific IDs */
    char        locale[QAL_SOUND_TRIGGER_MAX_LOCALE_LEN]; /**< locale - JAVA Locale style (e.g. en_US) */
    char        text[QAL_SOUND_TRIGGER_MAX_STRING_LEN];   /**< phrase text in UTF-8 format. */
};

/**
 * Specialized sound model for key phrase detection.
 * Proprietary representation of key phrases in binary data must match information indicated
 * by phrases field use this when not sending
 */
struct qal_st_phrase_sound_model {
    struct qal_st_sound_model   common;         /** common sound model */
    uint32_t                    num_phrases;    /** number of key phrases in model */
    struct qal_st_phrase        phrases[QAL_SOUND_TRIGGER_MAX_PHRASES];
};

struct qal_st_confidence_level {
    uint32_t user_id;   /* user ID */
    uint32_t level;     /* confidence level in percent (0 - 100).
                               - min level for recognition configuration
                               - detected level for recognition event */
};

/** Specialized recognition event for key phrase detection */
struct qal_st_phrase_recognition_extra {
    uint32_t id;                /* keyphrase ID */
    uint32_t recognition_modes; /* recognition modes used for this keyphrase */
    uint32_t confidence_level;  /* confidence level for mode RECOGNITION_MODE_VOICE_TRIGGER */
    uint32_t num_levels;        /* number of user confidence levels */
    struct qal_st_confidence_level levels[QAL_SOUND_TRIGGER_MAX_USERS];
};

struct qal_st_recognition_event {
    int32_t                          status;              /**< recognition status e.g.
                                                              RECOGNITION_STATUS_SUCCESS */
    qal_st_sound_model_type_t        type;                /**< event type, same as sound model type.
                                                              e.g. SOUND_MODEL_TYPE_KEYPHRASE */
    qal_st_handle_t                  *st_handle;           /**< handle of sound trigger session */
    bool                             capture_available;   /**< it is possible to capture audio from this
                                                              utterance buffered by the
                                                              implementation */
    int32_t                          capture_session;     /**< audio session ID. framework use */
    int32_t                          capture_delay_ms;    /**< delay in ms between end of model
                                                              detection and start of audio available
                                                              for capture. A negative value is possible
                                                              (e.g. if key phrase is also available for
                                                              capture */
    int32_t                          capture_preamble_ms; /**< duration in ms of audio captured
                                                              before the start of the trigger.
                                                              0 if none. */
    bool                             trigger_in_data;     /**< the opaque data is the capture of
                                                              the trigger sound */
    struct qal_media_config          media_config;        /**< media format of either the trigger in
                                                              event data or to use for capture of the
                                                              rest of the utterance */
    uint32_t                         data_size;           /**< size of opaque event data */
    uint32_t                         data_offset;         /**< offset of opaque data start from start of
                                                              this struct (e.g sizeof struct
                                                              sound_trigger_phrase_recognition_event) */
};

typedef void(*qal_st_recognition_callback_t)(struct qal_st_recognition_event *event,
                                             void *cookie);

/* Payload for qal_st_start_recognition() */
struct qal_st_recognition_config {
    int32_t       capture_handle;             /**< IO handle that will be used for capture.
                                                N/A if capture_requested is false */
    uint32_t      capture_device;             /**< input device requested for detection capture */
    bool          capture_requested;          /**< capture and buffer audio for this recognition
                                                instance */
    uint32_t      num_phrases;                /**< number of key phrases recognition extras */
    struct qal_st_phrase_recognition_extra phrases[QAL_SOUND_TRIGGER_MAX_PHRASES];
                                              /**< configuration for each key phrase */
    qal_st_recognition_callback_t callback;   /**< callback for recognition events */
    void *        cookie;                     /**< cookie set from client*/
    uint32_t      data_size;                  /**< size of opaque capture configuration data */
    uint32_t      data_offset;                /**< offset of opaque data start from start of this struct
                                              (e.g sizeof struct sound_trigger_recognition_config) */
};

struct qal_st_phrase_recognition_event {
    struct qal_st_recognition_event common;
    unsigned int                           num_phrases;
    struct qal_st_phrase_recognition_extra phrase_extras[QAL_SOUND_TRIGGER_MAX_PHRASES];
};

struct qal_st_generic_recognition_event {
    struct qal_st_recognition_event common;
};

struct detection_engine_config_voice_wakeup {
    uint16_t mode;
    uint16_t custom_payload_size;
    uint8_t num_active_models;
    uint8_t reserved;
    uint8_t confidence_levels[QAL_SOUND_TRIGGER_MAX_USERS];
    uint8_t keyword_user_enables[QAL_SOUND_TRIGGER_MAX_USERS];
};

struct detection_engine_voice_wakeup_buffer_config {
    uint32_t hist_buffer_duration_in_ms;
    uint32_t pre_roll_duration_in_ms;
};

struct detection_engine_generic_event_cfg {
    uint32_t event_mode;
};

struct ffv_doa_tracking_monitor_t
{
    int16_t target_angle_L16[2];
    int16_t interf_angle_L16[2];
    int8_t polarActivityGUI[360];
};

/** @brief Callback function prototype to be given for
 *         qal_open_stream.
 *
 * \param[in] stream_handle - stream handle associated with the
 * callback event.
 * \param[in] event_id - event id of the event raised on the
 *       stream.
 * \param[in] event_data - event_data specific to the event
 *       raised.
 * \param[in] cookie - cookie speificied in the
 *       qal_stream_open()
 */
typedef int32_t (*qal_stream_callback)(qal_stream_handle_t *stream_handle,
                                       uint32_t event_id, uint32_t *event_data,
                                       void *cookie);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /*QAL_DEFS_H*/
