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

#ifndef PAYLOAD_BUILDER_H_
#define PAYLOAD_BUILDER_H_

#include "PalDefs.h"
#include "gsl_intf.h"
#include "kvh2xml.h"
#include "PalCommon.h"
#include <vector>
#include <algorithm>
#include <expat.h>
#include <map>
#include "Stream.h"
#include "Device.h"
#include "ResourceManager.h"

#define PAL_ALIGN_8BYTE(x) (((x) + 7) & (~7))
#define PAL_PADDING_8BYTE_ALIGN(x)  ((((x) + 7) & 7) ^ 7)

#define MSM_MI2S_SD0 (1 << 0)
#define MSM_MI2S_SD1 (1 << 1)
#define MSM_MI2S_SD2 (1 << 2)
#define MSM_MI2S_SD3 (1 << 3)
#define MSM_MI2S_SD4 (1 << 4)
#define MSM_MI2S_SD5 (1 << 5)
#define MSM_MI2S_SD6 (1 << 6)
#define MSM_MI2S_SD7 (1 << 7)

#define PCM_ENCODER STREAM_PCM_ENCODER
#define PCM_DECODER STREAM_PCM_DECODER
#define PCM_CONVERTOR STREAM_PCM_CONVERTER
#define IN_MEDIA STREAM_INPUT_MEDIA_FORMAT
#define CODEC_DMA 0
#define I2S 1
#define HW_EP_TX DEVICE_HW_ENDPOINT_TX
#define HW_EP_RX DEVICE_HW_ENDPOINT_RX
#define TAG_STREAM_MFC_SR TAG_STREAM_MFC
#define TAG_DEVICE_MFC_SR PER_STREAM_PER_DEVICE_MFC

struct sessionToPayloadParam {
    uint32_t sampleRate;                /**< sample rate */
    uint32_t bitWidth;                  /**< bit width */
    uint32_t numChannel;
    struct pal_channel_info *ch_info;    /**< channel info */
    int direction;
    int native;
    int rotation_type;
    void *metadata;
    sessionToPayloadParam():sampleRate(48000),bitWidth(16),
    numChannel(2),ch_info(nullptr), direction(0),
    native(0),rotation_type(0),metadata(nullptr) {}
};

struct usbAudioConfig {
    uint32_t usb_token;
    uint32_t svc_interval;
};

struct dpAudioConfig{
  uint32_t channel_allocation;
  uint32_t mst_idx;
  uint32_t dptx_idx;
};

class SessionGsl;

class PayloadBuilder
{
public:
    void payloadUsbAudioConfig(uint8_t** payload, size_t* size,
                           uint32_t miid,
                           struct usbAudioConfig *data);
    void payloadDpAudioConfig(uint8_t** payload, size_t* size,
                           uint32_t miid,
                           struct dpAudioConfig *data);
    void payloadMFCConfig(uint8_t** payload, size_t* size,
                           uint32_t miid,
                           struct sessionToPayloadParam* data);
    int payloadCustomParam(uint8_t **alsaPayload, size_t *size,
                            uint32_t *customayload, uint32_t customPayloadSize,
                            uint32_t moduleInstanceId, uint32_t dspParamId);
    int payloadSVAConfig(uint8_t **payload, size_t *size,
                        uint8_t *config, size_t config_size,
                        uint32_t miid, uint32_t param_id);
    void payloadDOAInfo(uint8_t **payload, size_t *size, uint32_t moduleId);
    void payloadQuery(uint8_t **payload, size_t *size, uint32_t moduleId,
                            uint32_t paramId, uint32_t querySize);
    template <typename T>
    void populateChannelMap(T pcmChannel, uint8_t numChannel);
    void payloadLC3Config(uint8_t** payload, size_t* size,
                          uint32_t miid, bool isLC3MonoModeOn);
    void payloadRATConfig(uint8_t** payload, size_t* size, uint32_t miid,
                          struct pal_media_config *data);
    void payloadPcmCnvConfig(uint8_t** payload, size_t* size, uint32_t miid,
                             struct pal_media_config *data);
    void payloadCopPackConfig(uint8_t** payload, size_t* size, uint32_t miid,
                          struct pal_media_config *data);
    void payloadCopV2PackConfig(uint8_t** payload, size_t* size, uint32_t miid, void *data);
    void payloadTWSConfig(uint8_t** payload, size_t* size, uint32_t miid,
                          bool isTwsMonoModeOn, uint32_t codecFormat);
    void payloadSPConfig(uint8_t** payload, size_t* size, uint32_t miid,
                         int paramId, void *data);
    int populateStreamKV(Stream* s, std::vector <std::pair<int,int>> &keyVector);
    int populateStreamKV(Stream* s, std::vector <std::pair<int,int>> &keyVectorRx,
        std::vector <std::pair<int,int>> &keyVectorTx ,struct vsid_info vsidinfo);
    int populateStreamPPKV(Stream* s, std::vector <std::pair<int,int>> &keyVectorRx,
        std::vector <std::pair<int,int>> &keyVectorTx);
    int populateStreamDeviceKV(Stream* s, int32_t beDevId, std::vector <std::pair<int,int>> &keyVector);
    int populateStreamDeviceKV(Stream* s, int32_t rxBeDevId, std::vector <std::pair<int,int>> &keyVectorRx,
        int32_t txBeDevId, std::vector <std::pair<int,int>> &keyVectorTx, struct vsid_info vsidinfo, sidetone_mode_t sidetoneMode);
    int populateDeviceKV(Stream* s, int32_t beDevId, std::vector <std::pair<int,int>> &keyVector);
    int populateDeviceKV(Stream* s, int32_t rxBeDevId, std::vector <std::pair<int,int>> &keyVectorRx,
        int32_t txBeDevId, std::vector <std::pair<int,int>> &keyVectorTx, sidetone_mode_t sidetoneMode);
    int populateDevicePPKV(Stream* s, int32_t rxBeDevId, std::vector <std::pair<int,int>> &keyVectorRx,
        int32_t txBeDevId, std::vector <std::pair<int,int>> &keyVectorTx,
        std::vector<kvpair_info> kvpair);
    int populateDevicePPCkv(Stream *s, std::vector <std::pair<int,int>> &keyVector);
    int populateStreamCkv(Stream *s, std::vector <std::pair<int,int>> &keyVector, int tag, struct pal_volume_data **);
    int populateCalKeyVector(Stream *s, std::vector <std::pair<int,int>> &ckv, int tag);
    int populateTagKeyVector(Stream *s, std::vector <std::pair<int,int>> &tkv, int tag, uint32_t* gsltag);
    void payloadTimestamp(uint8_t **payload, size_t *size, uint32_t moduleId);
    static int init();
    static void endTag(void *userdata __unused, const XML_Char *tag_name);
    static void startTag(void *userdata __unused, const XML_Char *tag_name, const XML_Char **attr);
    PayloadBuilder();
    ~PayloadBuilder();
};
#endif //SESSION_H
