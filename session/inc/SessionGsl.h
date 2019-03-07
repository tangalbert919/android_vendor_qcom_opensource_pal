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

#ifndef SESSION_GSL_H
#define SESSION_GSL_H

#define BUFF_FLAG_EOS 0x1

#include "gsl_intf.h"
#include "Session.h"
#include <dlfcn.h>
//struct gsl_key_value_pair {
//	uint32_t key; /**< key */
//	uint32_t value; /**< value */
//};

//struct gsl_buff {
//	uint64_t timestamp; /**< timestamp in micro-secs */
//	uint32_t flags; /**< bitmasked flags for e.g. GSL_BUFF_FLAG_EOS */
//	uint32_t size; /**< size of buffer in bytes */
//	uint8_t *addr; /**< data buffer */
//};

/* Tag ID definitions */
#define SHMEM_ENDPOINT      0xC0000001
#define INPUT_MEDIA_FORMAT  0xC0000002
#define OUTPUT_MEDIA_FORMAT 0xC0000003
#define HW_ENDPOINT_RX      0xC0000004
#define HW_ENDPOINT_TX      0xC0000005

/* Module ID definitions */
#define MODULE_ID_CODEC_DMA_SINK   0x07001023
#define MODULE_ID_CODEC_DMA_SOURCE 0x07001024

/* Param ID definitions */
#define PARAM_ID_CODEC_DMA_INTF_CFG 0x08001063
#define PARAM_ID_HW_EP_MF_CFG       0x08001017
#define PARAM_ID_PCM_OUTPUT_FORMAT_CFG 0x08001008
#define PARAM_ID_MEDIA_FORMAT 0x0800100C

#define DATA_FORMAT_FIXED_POINT 0x00000001

/* Media format ID for identifying PCM streams */
#define MEDIA_FMT_ID_PCM                  0x09001000

/* Endianness */
#define PCM_LITTLE_ENDIAN                 1
#define PCM_BIG_ENDIAN                    2

/* Interleaved PCM */
#define PCM_INTERLEAVED                   1
/*
 * Packed Deinterleaved PCM:
 * A buffer of max size M with C channels and N/C actual bytes per channel
 * is deinterleaved-packed if (M - N) is zero.
 */
#define PCM_DEINTERLEAVED_PACKED          2
/*
 * Unpacked Deinterleaved PCM:
 * A buffer of max size M with C channels and N/C actual bytes per channel
 * is deinterleaved-unpacked if (M - N) is nonzero.
 * OR each channel has its own buffers with actual length being less than max length.
 */
#define PCM_DEINTERLEAVED_UNPACKED        3

/* Zero is invalid value */
#define INVALID_VALUE                     0

/* Channel definitions */
#define PCM_CHANNEL_L                     1
#define PCM_CHANNEL_R                     2
#define PCM_CHANNEL_C                     3

#define WSA_CODEC_DMA_CORE  0
#define VA_CODEC_DMA_CORE   1
#define RXTX_CODEC_DMA_CORE 2

#define CODEC_RX0 1
#define CODEC_TX0 1
#define CODEC_RX1 2
#define CODEC_TX1 2
#define CODEC_RX2 3
#define CODEC_TX2 3
#define CODEC_RX3 4
#define CODEC_TX3 4
#define CODEC_RX4 5
#define CODEC_TX4 5
#define CODEC_RX5 6
#define CODEC_TX5 6
#define CODEC_RX6 7
#define CODEC_RX7 8

struct gslCmdGetReadWriteBufInfo {
    uint32_t buff_size;
    uint32_t num_buffs;
    uint32_t start_threshold;
    uint32_t stop_threshold;
    uint32_t attritubes;
};

//struct gsl_key_vector {
//	size_t num_kvps;  /**< number of key value pairs */
//	struct gsl_key_value_pair *kvp;  /**< vector of key value pairs */
//};

struct __attribute__((__packed__)) apm_module_param_data_t
{
    uint32_t module_instance_id;
    uint32_t param_id;
    uint32_t param_size;
    uint32_t error_code;
};

struct __attribute__((__packed__)) hwEpConfig {
    uint32_t sample_rate;
    uint16_t bit_width;
    uint16_t num_channels;
    uint32_t data_format;
};

struct __attribute__((__packed__)) codecDmaIntfConfig {
    uint32_t cdc_dma_type;
    uint32_t intf_idx;
    uint32_t active_channels_mask;
};

struct __attribute__((__packed__)) media_format_t
{
    uint32_t data_format;
    uint32_t fmt_id;
    uint32_t payload_size;
};

struct __attribute__((__packed__)) payload_pcm_output_format_cfg_t
{
    int16_t bit_width;
    int16_t alignment;
    int16_t bits_per_sample;
    int16_t q_factor;
    int16_t endianness;
    int16_t interleaved;
    int16_t reserved;
    int16_t num_channels;
};

struct __attribute__((__packed__)) payload_media_fmt_pcm_t
{
    uint32_t sample_rate;
    uint16_t bit_width;
    uint16_t alignment;
    uint16_t bits_per_sample;
    uint16_t q_factor;
    uint16_t endianness;
    uint16_t num_channels;
};

class Stream;
class Session;

class SessionGsl : public Session
{
private:
    void * graphHandle;
    void * payload;
    size_t size = 0;
    size_t gkvLen, ckvLen, tkvLen;
    struct gslCmdGetReadWriteBufInfo *infoBuffer;
    static int seek;
    static void* gslLibHandle;
    int fileWrite(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag);
    int fileRead(Stream *s, int tag, struct qal_buffer *buf, int * size);
    
public:
    SessionGsl();
    ~SessionGsl();
    static int init(std::string acdbFile);
    static void deinit();
    int open(Stream * s) override;
    int prepare(Stream * s) override;
    int setConfig(Stream * s, configType type, int tag = 0) override;
    //int getConfig(Stream * s) override;
    int start(Stream * s) override;
    int stop(Stream * s) override;
    int close(Stream * s) override;
    int readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int read(Stream *s, int tag, struct qal_buffer *buf, int * size) override;
    int write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) override;
    struct gsl_key_vector *gkv;
    struct gsl_key_vector *ckv;
    struct gsl_key_vector *tkv;
};

#endif //SESSION_GSL_H
