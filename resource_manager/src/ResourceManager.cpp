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

#define LOG_TAG "QAL: ResourceManager"
#include "ResourceManager.h"
#include "Session.h"
#include "Device.h"
#include "Stream.h"
#include "StreamPCM.h"
#include "StreamCompress.h"
#include "StreamSoundTrigger.h"
#include "gsl_intf.h"
#include "SessionGsl.h"
#include "Headphone.h"
#include "PayloadBuilder.h"
#include "Bluetooth.h"
#include "SpeakerMic.h"
#include "Speaker.h"
#include "USBAudio.h"
#include "HeadsetMic.h"
#include "HandsetMic.h"
#include "DisplayPort.h"
#include "Handset.h"
#include "SndCardMonitor.h"
#include "agm_api.h"
#include <unistd.h>

#ifndef FEATURE_IPQ_OPENWRT
#include <cutils/str_parms.h>
#endif

#define XML_FILE_DELIMITER "_"
#define XML_FILE_EXT ".xml"
#define XML_PATH_MAX_LENGTH 100
#define HW_INFO_ARRAY_MAX_SIZE 32

#if defined(FEATURE_IPQ_OPENWRT) || defined(LINUX_ENABLED)
#define MIXER_XML_BASE_STRING "/etc/mixer_paths"
#define MIXER_XML_DEFAULT_PATH "/etc/mixer_paths_wsa.xml"
#define DEFAULT_ACDB_FILES "/etc/acdbdata/MTP/acdb_cal.acdb"
#define XMLFILE "/etc/resourcemanager.xml"
#define RMNGR_XMLFILE_BASE_STRING  "/etc/resourcemanager"
#define SPFXMLFILE "/etc/kvh2xml.xml"
#define SNDPARSER "/etc/card-defs.xml"
#define STXMLFILE "/etc/sound_trigger_platform_info.xml"
#else
#define MIXER_XML_BASE_STRING "/vendor/etc/mixer_paths"
#define MIXER_XML_DEFAULT_PATH "/vendor/etc/mixer_paths_wsa.xml"
#define DEFAULT_ACDB_FILES "/vendor/etc/acdbdata/MTP/acdb_cal.acdb"
#define XMLFILE "/vendor/etc/resourcemanager.xml"
#define RMNGR_XMLFILE_BASE_STRING  "/vendor/etc/resourcemanager"
#define SPFXMLFILE "/vendor/etc/kvh2xml.xml"
#define SNDPARSER "/vendor/etc/card-defs.xml"
#define STXMLFILE "/vendor/etc/sound_trigger_platform_info.xml"
#endif

#define MAX_SND_CARD 10
#define MAX_RETRY_CNT 20
#define LOWLATENCY_PCM_DEVICE 15
#define DEEP_BUFFER_PCM_DEVICE 0
#define DEVICE_NAME_MAX_SIZE 128

#define DEFAULT_BIT_WIDTH 16
#define DEFAULT_SAMPLE_RATE 48000
#define DEFAULT_CHANNELS 2
#define DEFAULT_FORMAT 0x00000000u
// TODO: double check and confirm actual
// values for max sessions number
#define MAX_SESSIONS_LOW_LATENCY 8
#define MAX_SESSIONS_ULTRA_LOW_LATENCY 8
#define MAX_SESSIONS_DEEP_BUFFER 1
#define MAX_SESSIONS_COMPRESSED 10
#define MAX_SESSIONS_GENERIC 1
#define MAX_SESSIONS_PCM_OFFLOAD 1
#define MAX_SESSIONS_VOICE_UI 2
#define MAX_SESSIONS_PROXY 8
#define DEFAULT_MAX_SESSIONS 8

static struct str_parms *configParamKVPairs;

char rmngr_xml_file[XML_PATH_MAX_LENGTH] = {0};

struct snd_card_split {
    char device[HW_INFO_ARRAY_MAX_SIZE];
    char form_factor[HW_INFO_ARRAY_MAX_SIZE];
};

static struct snd_card_split cur_snd_card_split{
    .device = {0},
    .form_factor = {0},
};

// default properties which will be updated based on platform configuration
static struct qal_st_properties qst_properties = {
        "QUALCOMM Technologies, Inc",  // implementor
        "Sound Trigger HAL",  // description
        1,  // version
        { 0x68ab2d40, 0xe860, 0x11e3, 0x95ef,
         { 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b } },  // uuid
        8,  // max_sound_models
        10,  // max_key_phrases
        10,  // max_users
        QAL_RECOGNITION_MODE_VOICE_TRIGGER |
        QAL_RECOGNITION_MODE_GENERIC_TRIGGER,  // recognition_modes
        true,  // capture_transition
        0,  // max_capture_ms
        false,  // concurrent_capture
        false,  // trigger_in_event
        0  // power_consumption_mw
};

/*
To be defined in detail, if GSL is defined,
pcm device id is directly related to device,
else using legacy design for alsa
*/
// Will update actual value when numbers got for VT

std::vector<std::pair<int32_t, std::string>> ResourceManager::deviceLinkName {
    {QAL_DEVICE_OUT_MIN,                  {std::string{ "none" }}},
    {QAL_DEVICE_NONE,                     {std::string{ "none" }}},
    {QAL_DEVICE_OUT_HANDSET,              {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPEAKER,              {std::string{ "" }}},
    {QAL_DEVICE_OUT_WIRED_HEADSET,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_WIRED_HEADPHONE,      {std::string{ "" }}},
    {QAL_DEVICE_OUT_LINE,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_SCO,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_A2DP,       {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_HDMI,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_DEVICE,           {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_HEADSET,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPDIF,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_FM,                   {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_LINE,             {std::string{ "" }}},
    {QAL_DEVICE_OUT_PROXY,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL_1,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_MAX,                  {std::string{ "none" }}},

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "tdm-pri" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "tdm-pri" }}},
    {QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, {std::string{ "" }}},
    {QAL_DEVICE_IN_WIRED_HEADSET,         {std::string{ "" }}},
    {QAL_DEVICE_IN_AUX_DIGITAL,           {std::string{ "" }}},
    {QAL_DEVICE_IN_HDMI,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_ACCESSORY,         {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_DEVICE,            {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_HEADSET,           {std::string{ "" }}},
    {QAL_DEVICE_IN_FM_TUNER,              {std::string{ "" }}},
    {QAL_DEVICE_IN_LINE,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_SPDIF,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_PROXY,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_HANDSET_VA_MIC,        {std::string{ "" }}},
    {QAL_DEVICE_IN_BLUETOOTH_A2DP,        {std::string{ "" }}},
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "" }}},
};

std::vector<std::pair<int32_t, int32_t>> ResourceManager::devicePcmId {
    {QAL_DEVICE_OUT_MIN,                  0},
    {QAL_DEVICE_NONE,                     0},
    {QAL_DEVICE_OUT_HANDSET,              1},
    {QAL_DEVICE_OUT_SPEAKER,              1},
    {QAL_DEVICE_OUT_WIRED_HEADSET,        1},
    {QAL_DEVICE_OUT_WIRED_HEADPHONE,      1},
    {QAL_DEVICE_OUT_LINE,                 0},
    {QAL_DEVICE_OUT_BLUETOOTH_SCO,        0},
    {QAL_DEVICE_OUT_BLUETOOTH_A2DP,       0},
    {QAL_DEVICE_OUT_AUX_DIGITAL,          0},
    {QAL_DEVICE_OUT_HDMI,                 0},
    {QAL_DEVICE_OUT_USB_DEVICE,           0},
    {QAL_DEVICE_OUT_USB_HEADSET,          0},
    {QAL_DEVICE_OUT_SPDIF,                0},
    {QAL_DEVICE_OUT_FM,                   0},
    {QAL_DEVICE_OUT_AUX_LINE,             0},
    {QAL_DEVICE_OUT_PROXY,                0},
    {QAL_DEVICE_OUT_AUX_DIGITAL_1,        0},
    {QAL_DEVICE_OUT_MAX,                  0},

    {QAL_DEVICE_IN_HANDSET_MIC,           0},
    {QAL_DEVICE_IN_SPEAKER_MIC,           0},
    {QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, 0},
    {QAL_DEVICE_IN_WIRED_HEADSET,         0},
    {QAL_DEVICE_IN_AUX_DIGITAL,           0},
    {QAL_DEVICE_IN_HDMI,                  0},
    {QAL_DEVICE_IN_USB_ACCESSORY,         0},
    {QAL_DEVICE_IN_USB_DEVICE,            0},
    {QAL_DEVICE_IN_USB_HEADSET,           0},
    {QAL_DEVICE_IN_FM_TUNER,              0},
    {QAL_DEVICE_IN_LINE,                  0},
    {QAL_DEVICE_IN_SPDIF,                 0},
    {QAL_DEVICE_IN_PROXY,                 0},
    {QAL_DEVICE_IN_HANDSET_VA_MIC,        0},
    {QAL_DEVICE_IN_BLUETOOTH_A2DP,        0},
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        0},
};

// To be defined in detail
std::vector<std::pair<int32_t, std::string>> ResourceManager::sndDeviceNameLUT {
    {QAL_DEVICE_OUT_MIN,                  {std::string{ "" }}},
    {QAL_DEVICE_NONE,                     {std::string{ "none" }}},
    {QAL_DEVICE_OUT_HANDSET,              {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPEAKER,              {std::string{ "" }}},
    {QAL_DEVICE_OUT_WIRED_HEADSET,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_WIRED_HEADPHONE,      {std::string{ "" }}},
    {QAL_DEVICE_OUT_LINE,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_SCO,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_A2DP,       {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_HDMI,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_DEVICE,           {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_HEADSET,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPDIF,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_FM,                   {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_LINE,             {std::string{ "" }}},
    {QAL_DEVICE_OUT_PROXY,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL_1,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_MAX,                  {std::string{ "" }}},

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "" }}},
    {QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, {std::string{ "" }}},
    {QAL_DEVICE_IN_WIRED_HEADSET,         {std::string{ "" }}},
    {QAL_DEVICE_IN_AUX_DIGITAL,           {std::string{ "" }}},
    {QAL_DEVICE_IN_HDMI,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_ACCESSORY,         {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_DEVICE,            {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_HEADSET,           {std::string{ "" }}},
    {QAL_DEVICE_IN_FM_TUNER,              {std::string{ "" }}},
    {QAL_DEVICE_IN_LINE,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_SPDIF,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_PROXY,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_HANDSET_VA_MIC,        {std::string{ "" }}},
    {QAL_DEVICE_IN_BLUETOOTH_A2DP,        {std::string{ "" }}},
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "" }}}
};

const std::map<std::string, uint32_t> usecaseIdLUT {
    {std::string{ "QAL_STREAM_LOW_LATENCY" },              QAL_STREAM_LOW_LATENCY},
    {std::string{ "QAL_STREAM_DEEP_BUFFER" },              QAL_STREAM_DEEP_BUFFER},
    {std::string{ "QAL_STREAM_COMPRESSED" },               QAL_STREAM_COMPRESSED},
    {std::string{ "QAL_STREAM_VOIP" },                     QAL_STREAM_VOIP},
    {std::string{ "QAL_STREAM_VOIP_RX" },                  QAL_STREAM_VOIP_RX},
    {std::string{ "QAL_STREAM_VOIP_TX" },                  QAL_STREAM_VOIP_TX},
    {std::string{ "QAL_STREAM_VOICE_CALL_MUSIC" },         QAL_STREAM_VOICE_CALL_MUSIC},
    {std::string{ "QAL_STREAM_GENERIC" },                  QAL_STREAM_GENERIC},
    {std::string{ "QAL_STREAM_RAW" },                      QAL_STREAM_RAW},
    {std::string{ "QAL_STREAM_VOICE_ACTIVATION" },         QAL_STREAM_VOICE_ACTIVATION},
    {std::string{ "QAL_STREAM_VOICE_CALL_RX" },            QAL_STREAM_VOICE_CALL_RX},
    {std::string{ "QAL_STREAM_VOICE_CALL_TX" },            QAL_STREAM_VOICE_CALL_TX},
    {std::string{ "QAL_STREAM_VOICE_CALL_RX_TX" },         QAL_STREAM_VOICE_CALL_RX_TX},
    {std::string{ "QAL_STREAM_VOICE_CALL" },               QAL_STREAM_VOICE_CALL},
    {std::string{ "QAL_STREAM_LOOPBACK" },                 QAL_STREAM_LOOPBACK},
    {std::string{ "QAL_STREAM_TRANSCODE" },                QAL_STREAM_TRANSCODE},
    {std::string{ "QAL_STREAM_VOICE_UI" },                 QAL_STREAM_VOICE_UI},
    {std::string{ "QAL_STREAM_ULTRA_LOW_LATENCY" },        QAL_STREAM_ULTRA_LOW_LATENCY},
    {std::string{ "QAL_STREAM_PROXY" },                    QAL_STREAM_PROXY},
};

const std::map<std::string, sidetone_mode_t> sidetoneModetoId {
    {std::string{ "OFF" }, SIDETONE_OFF},
    {std::string{ "HW" },  SIDETONE_HW},
    {std::string{ "SW" },  SIDETONE_SW},
};

std::shared_ptr<ResourceManager> ResourceManager::rm = nullptr;
std::vector <int> ResourceManager::streamTag = {0};
std::vector <int> ResourceManager::streamPpTag = {0};
std::vector <int> ResourceManager::mixerTag = {0};
std::vector <int> ResourceManager::devicePpTag = {0};
std::vector <int> ResourceManager::deviceTag = {0};
std::mutex ResourceManager::mResourceManagerMutex;
std::mutex ResourceManager::mGraphMutex;
std::mutex ResourceManager::ssrMutex;
std::vector <int> ResourceManager::listAllFrontEndIds = {0};
std::vector <int> ResourceManager::listFreeFrontEndIds = {0};
std::vector <int> ResourceManager::listAllPcmPlaybackFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmRecordFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmLoopbackRxFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmLoopbackTxFrontEnds = {0};
std::vector <int> ResourceManager::listAllCompressPlaybackFrontEnds = {0};
std::vector <int> ResourceManager::listAllCompressRecordFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmVoice1RxFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmVoice1TxFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmVoice2RxFrontEnds = {0};
std::vector <int> ResourceManager::listAllPcmVoice2TxFrontEnds = {0};
struct audio_mixer* ResourceManager::audio_mixer = NULL;
struct audio_route* ResourceManager::audio_route = NULL;
int ResourceManager::snd_card = 0;
std::vector<deviceCap> ResourceManager::devInfo;
static struct nativeAudioProp na_props;
SndCardMonitor* ResourceManager::sndmon = NULL;
int ResourceManager::mixerEventRegisterCount = 0;
static int max_session_num;

//TODO:Needs to define below APIs so that functionality won't break
#ifdef FEATURE_IPQ_OPENWRT
int str_parms_get_str(struct str_parms *str_parms, const char *key,
                      char *out_val, int len){return 0;}
char *str_parms_to_str(struct str_parms *str_parms){return NULL;}
int str_parms_add_str(struct str_parms *str_parms, const char *key,
                      const char *value){return 0;}
struct str_parms *str_parms_create(void){return NULL;}
void str_parms_del(struct str_parms *str_parms, const char *key){return;}
void str_parms_destroy(struct str_parms *str_parms){return;}

#endif

std::vector<deviceIn> ResourceManager::deviceInfo;
std::vector<tx_ecinfo> ResourceManager::txEcInfo;
struct vsid_info ResourceManager::vsidInfo;

std::map<std::pair<uint32_t, std::string>, std::string> ResourceManager::btCodecMap;

#define MAKE_STRING_FROM_ENUM(string) { {#string}, string }
std::map<std::string, uint32_t> ResourceManager::btFmtTable = {
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_AAC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_SBC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_HD),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_DUAL_MONO),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_LDAC),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_CELT),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_APTX_AD_SPEECH),
    MAKE_STRING_FROM_ENUM(CODEC_TYPE_PCM)
};

std::vector<std::pair<int32_t, std::string>> ResourceManager::listAllBackEndIds {
    {QAL_DEVICE_OUT_MIN,                  {std::string{ "" }}},
    {QAL_DEVICE_NONE,                     {std::string{ "" }}},
    {QAL_DEVICE_OUT_HANDSET,              {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPEAKER,              {std::string{ "none" }}},
    {QAL_DEVICE_OUT_WIRED_HEADSET,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_WIRED_HEADPHONE,      {std::string{ "" }}},
    {QAL_DEVICE_OUT_LINE,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_SCO,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_BLUETOOTH_A2DP,       {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_HDMI,                 {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_DEVICE,           {std::string{ "" }}},
    {QAL_DEVICE_OUT_USB_HEADSET,          {std::string{ "" }}},
    {QAL_DEVICE_OUT_SPDIF,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_FM,                   {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_LINE,             {std::string{ "" }}},
    {QAL_DEVICE_OUT_PROXY,                {std::string{ "" }}},
    {QAL_DEVICE_OUT_AUX_DIGITAL_1,        {std::string{ "" }}},
    {QAL_DEVICE_OUT_MAX,                  {std::string{ "" }}},

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "none" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "none" }}},
    {QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET, {std::string{ "" }}},
    {QAL_DEVICE_IN_WIRED_HEADSET,         {std::string{ "" }}},
    {QAL_DEVICE_IN_AUX_DIGITAL,           {std::string{ "" }}},
    {QAL_DEVICE_IN_HDMI,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_ACCESSORY,         {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_DEVICE,            {std::string{ "" }}},
    {QAL_DEVICE_IN_USB_HEADSET,           {std::string{ "" }}},
    {QAL_DEVICE_IN_FM_TUNER,              {std::string{ "" }}},
    {QAL_DEVICE_IN_LINE,                  {std::string{ "" }}},
    {QAL_DEVICE_IN_SPDIF,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_PROXY,                 {std::string{ "" }}},
    {QAL_DEVICE_IN_HANDSET_VA_MIC,        {std::string{ "none" }}},
    {QAL_DEVICE_IN_BLUETOOTH_A2DP,        {std::string{ "" }}},
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "none" }}},
};

void agmServiceCrashHandler(uint64_t cookie)
{
    QAL_ERR(LOG_TAG, "AGM service crashed :( ");
    _exit(1);
}

void ResourceManager::split_snd_card(const char* in_snd_card_name)
{
    /* Sound card name follows below mentioned convention:
       <target name>-<form factor>-snd-card.
       Parse target name and form factor.
    */
    char *snd_card_name = NULL;
    char *tmp = NULL;
    char *device = NULL;
    char *form_factor = NULL;

    if (in_snd_card_name == NULL) {
        QAL_ERR(LOG_TAG, "%s: snd_card_name passed is NULL", __func__);
        goto err;
    }
    snd_card_name = strdup(in_snd_card_name);

    device = strtok_r(snd_card_name, "-", &tmp);
    if (device == NULL) {
        QAL_ERR(LOG_TAG, "%s: called on invalid snd card name", __func__);
        goto err;
    }
    strlcpy(cur_snd_card_split.device, device, HW_INFO_ARRAY_MAX_SIZE);

    form_factor = strtok_r(NULL, "-", &tmp);
    if (form_factor == NULL) {
        QAL_ERR(LOG_TAG, "%s: called on invalid snd card name", __func__);
        goto err;
    }
    strlcpy(cur_snd_card_split.form_factor, form_factor, HW_INFO_ARRAY_MAX_SIZE);

    QAL_ERR(LOG_TAG, "%s: snd_card_name(%s) device(%s) form_factor(%s)",
               __func__, in_snd_card_name, device, form_factor);

err:
    if (snd_card_name)
        free(snd_card_name);
}

ResourceManager::ResourceManager()
{
    QAL_INFO(LOG_TAG, "Enter.");
    int ret = 0;
    const qal_alsa_or_gsl ag = getQALConfigALSAOrGSL();
    // Init audio_route and audio_mixer

    na_props.rm_na_prop_enabled = false;
    na_props.ui_na_prop_enabled = false;
    na_props.na_mode = NATIVE_AUDIO_MODE_INVALID;

    max_session_num = DEFAULT_MAX_SESSIONS;
    //TODO: parse the tag and populate in the tags
    streamTag.clear();
    deviceTag.clear();
    btCodecMap.clear();

    ret = ResourceManager::init_audio();
    QAL_INFO(LOG_TAG, "Enter.");
    if (ret) {
        QAL_ERR(LOG_TAG, "error in init audio route and audio mixer ret %d", ret);
    }

    if (ag == GSL) {
        ret = SessionGsl::init(DEFAULT_ACDB_FILES);
    }

    ret = ResourceManager::XmlParser(SPFXMLFILE);
    if (ret) {
        QAL_ERR(LOG_TAG, "error in spf xml parsing ret %d", ret);
    }

    ret = ResourceManager::XmlParser(rmngr_xml_file);
    if (ret) {
        QAL_ERR(LOG_TAG, "error in resource xml parsing ret %d", ret);
    }

    if (ag == ALSA) {
        listAllFrontEndIds.clear();
        listFreeFrontEndIds.clear();
        listAllPcmPlaybackFrontEnds.clear();
        listAllPcmRecordFrontEnds.clear();
        listAllPcmLoopbackRxFrontEnds.clear();
        listAllPcmLoopbackTxFrontEnds.clear();
        listAllCompressPlaybackFrontEnds.clear();
        listAllCompressRecordFrontEnds.clear();
        listAllPcmVoice1RxFrontEnds.clear();
        listAllPcmVoice1TxFrontEnds.clear();
        listAllPcmVoice2RxFrontEnds.clear();
        listAllPcmVoice2TxFrontEnds.clear();

        ret = ResourceManager::XmlParser(SNDPARSER);
        if (ret) {
            QAL_ERR(LOG_TAG, "error in snd xml parsing ret %d", ret);
        }
        for (int i=0; i < devInfo.size(); i++) {
            if (devInfo[i].type == PCM) {
                if (devInfo[i].loopback == 1 && devInfo[i].playback == 1) {
                    listAllPcmLoopbackRxFrontEnds.push_back(devInfo[i].deviceId);
                } else if (devInfo[i].loopback == 1 && devInfo[i].record == 1) {
                    listAllPcmLoopbackTxFrontEnds.push_back(devInfo[i].deviceId);
                } else if (devInfo[i].playback == 1 && devInfo[i].loopback == 0) {
                    listAllPcmPlaybackFrontEnds.push_back(devInfo[i].deviceId);
                } else if (devInfo[i].record == 1 && devInfo[i].loopback == 0) {
                    listAllPcmRecordFrontEnds.push_back(devInfo[i].deviceId);
                }
            } else if (devInfo[i].type == COMPRESS) {
                if (devInfo[i].playback == 1) {
                    listAllCompressPlaybackFrontEnds.push_back(devInfo[i].deviceId);
                } else if (devInfo[i].record == 1) {
                    listAllCompressRecordFrontEnds.push_back(devInfo[i].deviceId);
                }
            } else if (devInfo[i].type == VOICE1) {
                if (devInfo[i].loopback == 1 && devInfo[i].playback == 1) {
                    listAllPcmVoice1RxFrontEnds.push_back(devInfo[i].deviceId);
                }
                if (devInfo[i].loopback == 1 && devInfo[i].record == 1) {
                    listAllPcmVoice1TxFrontEnds.push_back(devInfo[i].deviceId);
                }
            } else if (devInfo[i].type == VOICE2) {
                if (devInfo[i].loopback == 1 && devInfo[i].playback == 1) {
                    listAllPcmVoice2RxFrontEnds.push_back(devInfo[i].deviceId);
                }
                if (devInfo[i].loopback == 1 && devInfo[i].record == 1) {
                    listAllPcmVoice2TxFrontEnds.push_back(devInfo[i].deviceId);
                }
            }
        }
    }
    // Get AGM service handle
    ret = agm_register_service_crash_callback(&agmServiceCrashHandler,
                                               (uint64_t)this);
    if (ret) {
        QAL_ERR(LOG_TAG, "AGM service not up%d", ret);
    }

    QAL_INFO(LOG_TAG, "Exit. ret %d", ret);
}

ResourceManager::~ResourceManager()
{
    streamTag.clear();
    streamPpTag.clear();
    mixerTag.clear();
    devicePpTag.clear();
    deviceTag.clear();

    listAllFrontEndIds.clear();
    listAllPcmPlaybackFrontEnds.clear();
    listAllPcmRecordFrontEnds.clear();
    listAllPcmLoopbackRxFrontEnds.clear();
    listAllPcmLoopbackTxFrontEnds.clear();
    listAllCompressPlaybackFrontEnds.clear();
    listAllCompressRecordFrontEnds.clear();
    listFreeFrontEndIds.clear();
    listAllPcmVoice1RxFrontEnds.clear();
    listAllPcmVoice1TxFrontEnds.clear();
    listAllPcmVoice2RxFrontEnds.clear();
    listAllPcmVoice2TxFrontEnds.clear();
    devInfo.clear();
    deviceInfo.clear();
    txEcInfo.clear();

    STInstancesLists.clear();
    listAllBackEndIds.clear();
    sndDeviceNameLUT.clear();
    devicePcmId.clear();
    deviceLinkName.clear();
}

int ResourceManager::initSndMonitor()
{
    int ret = 0;
    sndmon = new SndCardMonitor(snd_card);
    if (!sndmon) {
        ret = -EINVAL;
        QAL_ERR(LOG_TAG, "Sound monitor creation failed, ret %d", ret);
        return ret;
    } else {
        cardState = CARD_STATUS_ONLINE;
        QAL_INFO(LOG_TAG, "Sound monitor initialized");
        return ret;
    }
}

int ResourceManager::ssrHandler(card_status_t state)
{
    int ret = 0;

    QAL_DBG(LOG_TAG, "Enter. %d ssrStarted %d size %d rm %p",
            state, ssrStarted, mActiveStreams.size(), this);

    cardState = state;
    mResourceManagerMutex.lock();
    if (rm->mActiveStreams.empty()) {
        QAL_INFO(LOG_TAG, "Idle SSR : No streams registered yet.");
        goto exit;
    }

    if (state == CARD_STATUS_OFFLINE) {
        std::lock_guard<std::mutex> lock(ResourceManager::ssrMutex);
        if (!ssrStarted) {
            ssrStarted = true;
            for (auto& str: mActiveStreams) {
                mResourceManagerMutex.unlock();
                ret = str->ssrDownHandler();
                if (0 != ret) {
                    QAL_ERR(LOG_TAG, "Ssr down handling failed for %pK ret %d",
                            str, ret);
                }
                mResourceManagerMutex.lock();
            }
            ssrStarted = false;
            /* Returning 0 even if we fail to close some streams
             * as sound card monitor will not be handling the
             * failures.
             */
            ret = 0;
            goto exit;
        } else {
            QAL_INFO(LOG_TAG, "SSR down handling already started");
            goto exit;
        }
    } else if (state == CARD_STATUS_ONLINE) {
        std::lock_guard<std::mutex> lock(ResourceManager::ssrMutex);
        for (auto& str: mActiveStreams) {
            mResourceManagerMutex.unlock();
            ret = str->ssrUpHandler();
            if (0 != ret) {
                QAL_ERR(LOG_TAG, "Ssr up handling failed for %pK ret %d",
                        str, ret);
            }
            mResourceManagerMutex.lock();
        }
        ret = 0;
        goto exit;
    } else {
        ret = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid state. state %d", state);
        goto exit;
    }

exit:
    mResourceManagerMutex.unlock();
    return ret;
}

char* ResourceManager::getDeviceNameFromID(uint32_t id)
{
    for (int i=0; i < devInfo.size(); i++) {
        if (devInfo[i].deviceId == id) {
            QAL_DBG(LOG_TAG, "pcm id name is %s ", devInfo[i].name);
            return devInfo[i].name;
        }
    }

    return NULL;
}

int ResourceManager::init_audio()
{
    int ret = 0, retry = 0;
    bool snd_card_found = false;
    char snd_macro[] = "snd";
    char *snd_card_name = NULL;
    char *tmp = NULL;
    char mixer_xml_file[XML_PATH_MAX_LENGTH] = {0};

    QAL_DBG(LOG_TAG, "Enter.");

    do {
        /* Look for only default codec sound card */
        /* Ignore USB sound card if detected */
        snd_card = 0;
        while (snd_card < MAX_SND_CARD) {
            struct audio_mixer* tmp_mixer = NULL;
            tmp_mixer = mixer_open(snd_card);
            if (tmp_mixer) {
                snd_card_name = strdup(mixer_get_name(tmp_mixer));
                if (!snd_card_name) {
                    QAL_ERR(LOG_TAG, "failed to allocate memory for snd_card_name");
                    mixer_close(tmp_mixer);
                    return -EINVAL;
                }
                QAL_INFO(LOG_TAG, "mixer_open success. snd_card_num = %d, snd_card_name %s, am:%p",
                snd_card, snd_card_name, rm->audio_mixer);

                /* TODO: Needs to extend for new targets */
                if (strstr(snd_card_name, "kona") ||
                    strstr(snd_card_name, "sm8150")||
                    strstr(snd_card_name, "lahaina") ) {
                    QAL_VERBOSE(LOG_TAG, "Found Codec sound card");
                    snd_card_found = true;
                    audio_mixer = tmp_mixer;
                    break;
                } else {
                    if (snd_card_name) {
                        free(snd_card_name);
                        snd_card_name = NULL;
                    }
                    mixer_close(tmp_mixer);
                }
            }
            snd_card++;
        }

        if (!snd_card_found) {
            QAL_INFO(LOG_TAG, "No audio mixer, retry %d", retry++);
            sleep(1);
        }
    } while (!snd_card_found && retry <= MAX_RETRY_CNT);

    if (snd_card >= MAX_SND_CARD || !audio_mixer) {
        QAL_ERR(LOG_TAG, "audio mixer open failure");
        return -EINVAL;
    }

    split_snd_card(snd_card_name);

    strlcpy(mixer_xml_file, MIXER_XML_BASE_STRING, XML_PATH_MAX_LENGTH);
    strlcpy(rmngr_xml_file, RMNGR_XMLFILE_BASE_STRING, XML_PATH_MAX_LENGTH);
    /* Note: This assumes IDP/MTP form factor will use mixer_paths.xml /
             resourcemanager.xml.
       TODO: Add support for form factors other than IDP/QRD.
    */
    if (!strncmp(cur_snd_card_split.form_factor, "qrd", sizeof ("qrd"))){
            strlcat(mixer_xml_file, XML_FILE_DELIMITER, XML_PATH_MAX_LENGTH);
            strlcat(mixer_xml_file, cur_snd_card_split.form_factor, XML_PATH_MAX_LENGTH);

            strlcat(rmngr_xml_file, XML_FILE_DELIMITER, XML_PATH_MAX_LENGTH);
            strlcat(rmngr_xml_file, cur_snd_card_split.form_factor, XML_PATH_MAX_LENGTH);
    }
    strlcat(mixer_xml_file, XML_FILE_EXT, XML_PATH_MAX_LENGTH);
    strlcat(rmngr_xml_file, XML_FILE_EXT, XML_PATH_MAX_LENGTH);

    audio_route = audio_route_init(snd_card, mixer_xml_file);
    QAL_INFO(LOG_TAG, "audio route %pK, mixer path %s", audio_route, mixer_xml_file);
    if (!audio_route) {
        QAL_ERR(LOG_TAG, "audio route init failed");
        mixer_close(audio_mixer);
        if (snd_card_name)
            free(snd_card_name);
        return -EINVAL;
    }
    // audio_route init success

    QAL_ERR(LOG_TAG, "Exit. audio route init success with card %d mixer path %s",
            snd_card, mixer_xml_file);
    return 0;
}

int ResourceManager::init()
{
    return 0;
}

bool ResourceManager::getEcRefStatus(qal_stream_type_t tx_streamtype,qal_stream_type_t rx_streamtype)
{
    bool ecref_status = true;
    for (int i = 0; i < txEcInfo.size(); i++) {
        if (tx_streamtype == txEcInfo[i].tx_stream_type) {
            for (auto rx_type = txEcInfo[i].disabled_rx_streams.begin();
                  rx_type != txEcInfo[i].disabled_rx_streams.end(); rx_type++) {
               if (rx_streamtype == *rx_type) {
                   ecref_status = false;
                   QAL_DBG(LOG_TAG, "given rx %d disabled %d status %d",rx_streamtype, *rx_type, ecref_status);
                   break;
               }
            }
        }
    }
    return ecref_status;
}

void ResourceManager::getDeviceInfo(qal_device_id_t deviceId, qal_stream_type_t type,
                                         struct qal_device_info *devinfo)
{
    struct kvpair_info kv = {};

    for (int32_t size1 = 0; size1 < deviceInfo.size(); size1++) {
        if (deviceId == deviceInfo[size1].deviceId) {
            devinfo->channels = deviceInfo[size1].channel;
            devinfo->max_channels = deviceInfo[size1].max_channel;
            for (int32_t size2 = 0; size2 < deviceInfo[size1].usecase.size(); size2++) {
                if (type == deviceInfo[size1].usecase[size2].type) {
                    for (int32_t kvsize = 0;
                    kvsize < deviceInfo[size1].usecase[size2].kvpair.size(); kvsize++) {
                       kv.key =  deviceInfo[size1].usecase[size2].kvpair[kvsize].key;
                       kv.value =  deviceInfo[size1].usecase[size2].kvpair[kvsize].value;
                       devinfo->kvpair.push_back(kv);
                    }
                    return;
               }
            }
        }
     }
}

int32_t ResourceManager::getSidetoneMode(qal_device_id_t deviceId,
                                         qal_stream_type_t type,
                                         sidetone_mode_t *mode){
    int32_t status = 0;

    *mode = SIDETONE_OFF;
    for (int32_t size1 = 0; size1 < deviceInfo.size(); size1++) {
        if (deviceId == deviceInfo[size1].deviceId) {
            for (int32_t size2 = 0; size2 < deviceInfo[size1].usecase.size(); size2++) {
                if (type == deviceInfo[size1].usecase[size2].type) {
                    *mode = deviceInfo[size1].usecase[size2].sidetoneMode;
                    QAL_DBG(LOG_TAG, "found sidetoneMode %d for dev %d", *mode, deviceId);
                    break;
                }
            }
        }
    }
    return status;
}

int32_t ResourceManager::getVsidInfo(struct vsid_info  *info) {
    int status = 0;
    struct vsid_modepair modePair = {};

    info->vsid = vsidInfo.vsid;
    for (int size = 0; size < vsidInfo.modepair.size(); size++) {
        modePair.key = vsidInfo.modepair[size].key;
        modePair.value = vsidInfo.modepair[size].value;
        info->modepair.push_back(modePair);
    }
    return status;

}

void ResourceManager::getChannelMap(uint8_t *channel_map, int channels)
{
    switch (channels) {
    case CHANNELS_1:
       channel_map[0] = QAL_CHMAP_CHANNEL_C;
       break;
    case CHANNELS_2:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       break;
    case CHANNELS_3:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_C;
       break;
    case CHANNELS_4:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_LB;
       channel_map[3] = QAL_CHMAP_CHANNEL_RB;
       break;
    case CHANNELS_5:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_C;
       channel_map[3] = QAL_CHMAP_CHANNEL_LB;
       channel_map[4] = QAL_CHMAP_CHANNEL_RB;
       break;
    case CHANNELS_6:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_C;
       channel_map[3] = QAL_CHMAP_CHANNEL_LFE;
       channel_map[4] = QAL_CHMAP_CHANNEL_LB;
       channel_map[5] = QAL_CHMAP_CHANNEL_RB;
       break;
    case CHANNELS_7:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_C;
       channel_map[3] = QAL_CHMAP_CHANNEL_LFE;
       channel_map[4] = QAL_CHMAP_CHANNEL_LB;
       channel_map[5] = QAL_CHMAP_CHANNEL_RB;
       channel_map[6] = QAL_CHMAP_CHANNEL_RC;
       break;
    case CHANNELS_8:
       channel_map[0] = QAL_CHMAP_CHANNEL_FL;
       channel_map[1] = QAL_CHMAP_CHANNEL_FR;
       channel_map[2] = QAL_CHMAP_CHANNEL_C;
       channel_map[3] = QAL_CHMAP_CHANNEL_LFE;
       channel_map[4] = QAL_CHMAP_CHANNEL_LB;
       channel_map[5] = QAL_CHMAP_CHANNEL_RB;
       channel_map[6] = QAL_CHMAP_CHANNEL_LS;
       channel_map[7] = QAL_CHMAP_CHANNEL_RS;
       break;
   }
}

int32_t ResourceManager::getDeviceConfig(struct qal_device *deviceattr,
                                         struct qal_stream_attributes *sAttr, int32_t channel)
{
    int32_t status = 0;
    struct qal_channel_info dev_ch_info;

    QAL_ERR(LOG_TAG, "deviceattr->id %d", deviceattr->id);
    switch (deviceattr->id) {
        case QAL_DEVICE_IN_SPEAKER_MIC:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info.channels %d", deviceattr->config.ch_info.channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_IN_HANDSET_MIC:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info.channels %d", deviceattr->config.ch_info.channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_IN_WIRED_HEADSET:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info.channels %d", deviceattr->config.ch_info.channels);
            deviceattr->config.sample_rate = sAttr->in_media_config.sample_rate;
            deviceattr->config.bit_width = sAttr->in_media_config.bit_width;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            status = (HeadsetMic::checkAndUpdateBitWidth(&deviceattr->config.bit_width) |
                HeadsetMic::checkAndUpdateSampleRate(&deviceattr->config.sample_rate));
            if (status) {
                QAL_ERR(LOG_TAG, "failed to update samplerate/bitwidth");
                status = -EINVAL;
            }
            QAL_DBG(LOG_TAG, "device samplerate %d, bitwidth %d", deviceattr->config.sample_rate, deviceattr->config.bit_width);
            break;
        case QAL_DEVICE_OUT_HANDSET:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info.channels %d", deviceattr->config.ch_info.channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_SPEAKER:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_WIRED_HEADPHONE:
        case QAL_DEVICE_OUT_WIRED_HEADSET:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = sAttr->out_media_config.sample_rate;
            deviceattr->config.bit_width = sAttr->out_media_config.bit_width;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            status = (Headphone::checkAndUpdateBitWidth(&deviceattr->config.bit_width) |
                Headphone::checkAndUpdateSampleRate(&deviceattr->config.sample_rate));
            if (status) {
                QAL_ERR(LOG_TAG, "failed to update samplerate/bitwidth");
                status = -EINVAL;
            }
            QAL_DBG(LOG_TAG, "device samplerate %d, bitwidth %d", deviceattr->config.sample_rate, deviceattr->config.bit_width);
            break;
        case QAL_DEVICE_IN_HANDSET_VA_MIC:
        case QAL_DEVICE_IN_HEADSET_VA_MIC:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info.channels %d", deviceattr->config.ch_info.channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_BLUETOOTH_A2DP:
        case QAL_DEVICE_IN_BLUETOOTH_A2DP:
            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_44K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_COMPRESSED;
            break;
        case QAL_DEVICE_OUT_BLUETOOTH_SCO:
        case QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
        {
            std::shared_ptr<BtSco> scoDev;

            dev_ch_info.channels = channel;
            getChannelMap(&(dev_ch_info.ch_map[0]), channel);
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_8K;  /* Updated when WBS set param is received */
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            scoDev = std::dynamic_pointer_cast<BtSco>(BtSco::getInstance(deviceattr, rm));
            if (!scoDev) {
                QAL_ERR(LOG_TAG, "failed to get BtSco singleton object.");
                return -EINVAL;
            }
            scoDev->updateSampleRate(&deviceattr->config.sample_rate);
            QAL_DBG(LOG_TAG, "BT SCO device samplerate %d, bitwidth %d",
                  deviceattr->config.sample_rate, deviceattr->config.bit_width);
        }
            break;
        case QAL_DEVICE_OUT_USB_DEVICE:
        case QAL_DEVICE_OUT_USB_HEADSET:
            {
                deviceattr->config.sample_rate = SAMPLINGRATE_44K;//SAMPLINGRATE_48K;
                deviceattr->config.bit_width = BITWIDTH_16;
                deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
                // config.ch_info memory is allocated in selectBestConfig below
                std::shared_ptr<USB> USB_out_device;
                USB_out_device = std::dynamic_pointer_cast<USB>(USB::getInstance(deviceattr, rm));
                if (!USB_out_device) {
                    QAL_ERR(LOG_TAG, "failed to get USB singleton object.");
                    return -EINVAL;
                }
                status = USB_out_device->selectBestConfig(deviceattr, sAttr, true);
                QAL_ERR(LOG_TAG, "device samplerate %d, bitwidth %d, ch %d", deviceattr->config.sample_rate, deviceattr->config.bit_width,
                        deviceattr->config.ch_info.channels);
            }
            break;
        case QAL_DEVICE_IN_USB_DEVICE:
        case QAL_DEVICE_IN_USB_HEADSET:
            {
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            std::shared_ptr<USB> USB_in_device;
            USB_in_device = std::dynamic_pointer_cast<USB>(USB::getInstance(deviceattr, rm));
            if (!USB_in_device) {
                QAL_ERR(LOG_TAG, "failed to get USB singleton object.");
                return -EINVAL;
            }
            USB_in_device->selectBestConfig(deviceattr, sAttr, false);
            }
            break;
        case QAL_DEVICE_IN_PROXY:
            {
            /* For QAL_DEVICE_IN_PROXY, copy all config from stream attributes */
            deviceattr->config.ch_info =sAttr->in_media_config.ch_info;
            deviceattr->config.sample_rate = sAttr->in_media_config.sample_rate;
            deviceattr->config.bit_width = sAttr->in_media_config.bit_width;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

            QAL_DBG(LOG_TAG, "QAL_DEVICE_IN_PROXY sample rate %d bitwidth %d",
                    deviceattr->config.sample_rate, deviceattr->config.bit_width);
            }
            break;
        case QAL_DEVICE_OUT_PROXY:
            {
            deviceattr->config.ch_info = sAttr->out_media_config.ch_info;
            deviceattr->config.sample_rate = sAttr->out_media_config.sample_rate;
            deviceattr->config.bit_width = sAttr->out_media_config.bit_width;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

            QAL_DBG(LOG_TAG, "QAL_DEVICE_OUT_PROXY sample rate %d bitwidth %d",
                    deviceattr->config.sample_rate, deviceattr->config.bit_width);
            }
            break;
        case QAL_DEVICE_OUT_AUX_DIGITAL:
        case QAL_DEVICE_OUT_AUX_DIGITAL_1:
        case QAL_DEVICE_OUT_HDMI:
            {
                std::shared_ptr<DisplayPort> dp_device;
                dp_device = std::dynamic_pointer_cast<DisplayPort>
                                    (DisplayPort::getInstance(deviceattr, rm));
                if (!dp_device) {
                    QAL_ERR(LOG_TAG, "Failed to get DisplayPort object.");
                    return -EINVAL;
                }
                /**
                 * Comparision of stream channel and device supported max channel.
                 * If stream channel is less than or equal to device supported
                 * channel then the channel of stream is taken othewise it is of
                 * device
                 */
                int channels = dp_device->getMaxChannel();

                if (channels > sAttr->out_media_config.ch_info.channels)
                    channels = sAttr->out_media_config.ch_info.channels;

                dev_ch_info.channels = channels;

                getChannelMap(&(dev_ch_info.ch_map[0]), channels);
                deviceattr->config.ch_info = dev_ch_info;
                QAL_DBG(LOG_TAG, "Channel map set for %d", channels);

                if (dp_device->isSupportedSR(NULL,
                            sAttr->out_media_config.sample_rate)) {
                    deviceattr->config.sample_rate =
                            sAttr->out_media_config.sample_rate;
                } else {
                    int sr = dp_device->getHighestSupportedSR();
                    if (sAttr->out_media_config.sample_rate > sr)
                        deviceattr->config.sample_rate = sr;
                    else
                        deviceattr->config.sample_rate = SAMPLINGRATE_48K;
                }

                QAL_DBG(LOG_TAG, "SR %d", deviceattr->config.sample_rate);

                if (DisplayPort::isBitWidthSupported(
                            sAttr->out_media_config.bit_width)) {
                    deviceattr->config.bit_width =
                            sAttr->out_media_config.bit_width;
                } else {
                    int bps = dp_device->getHighestSupportedBps();
                    if (sAttr->out_media_config.bit_width > bps)
                        deviceattr->config.bit_width = bps;
                    else
                        deviceattr->config.bit_width = BITWIDTH_16;
                }
                QAL_DBG(LOG_TAG, "Bit Width %d", deviceattr->config.bit_width);

                deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            }
            break;
        default:
            QAL_ERR(LOG_TAG, "No matching device id %d", deviceattr->id);
            status = -EINVAL;
            //do nothing for rest of the devices
            break;
    }
    return status;
}

bool ResourceManager::isStreamSupported(struct qal_stream_attributes *attributes,
                                        struct qal_device *devices, int no_of_devices)
{
    bool result = false;
    uint16_t channels;
    uint32_t samplerate, bitwidth;
    uint32_t rc;
    size_t cur_sessions = 0;
    size_t max_sessions = 0;

    if (!attributes || !devices || !no_of_devices) {
        QAL_ERR(LOG_TAG, "Invalid input parameter ret %d", result);
        return result;
    }

    // check if stream type is supported
    // and new stream session is allowed
    qal_stream_type_t type = attributes->type;
    QAL_DBG(LOG_TAG, "Enter. type %d", type);
    switch (type) {
        case QAL_STREAM_VOICE_CALL_RX:
        case QAL_STREAM_VOICE_CALL_TX:
        case QAL_STREAM_VOICE_CALL_RX_TX:
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
            cur_sessions = active_streams_ll.size();
            max_sessions = MAX_SESSIONS_LOW_LATENCY;
            break;
        case QAL_STREAM_ULTRA_LOW_LATENCY:
            cur_sessions = active_streams_ull.size();
            max_sessions = MAX_SESSIONS_ULTRA_LOW_LATENCY;
            break;
        case QAL_STREAM_DEEP_BUFFER:
            cur_sessions = active_streams_db.size();
            max_sessions = MAX_SESSIONS_DEEP_BUFFER;
            break;
        case QAL_STREAM_COMPRESSED:
            cur_sessions = active_streams_comp.size();
            max_sessions = MAX_SESSIONS_COMPRESSED;
            break;
        case QAL_STREAM_VOICE_CALL_MUSIC:
            break;
        case QAL_STREAM_GENERIC:
            cur_sessions = active_streams_ulla.size();
            max_sessions = MAX_SESSIONS_GENERIC;
            break;
        case QAL_STREAM_RAW:
        case QAL_STREAM_VOICE_ACTIVATION:
        case QAL_STREAM_VOICE_CALL:
        case QAL_STREAM_LOOPBACK:
        case QAL_STREAM_TRANSCODE:
        case QAL_STREAM_VOICE_UI:
            cur_sessions = active_streams_st.size();
            max_sessions = MAX_SESSIONS_VOICE_UI;
            break;
        case QAL_STREAM_PCM_OFFLOAD:
            cur_sessions = active_streams_po.size();
            max_sessions = MAX_SESSIONS_PCM_OFFLOAD;
            break;
        case QAL_STREAM_PROXY:
            cur_sessions = active_streams_proxy.size();
            max_sessions = MAX_SESSIONS_PROXY;
            break;
        default:
            QAL_ERR(LOG_TAG, "Invalid stream type = %d", type);
        return result;
    }
    if (cur_sessions == max_sessions) {
        QAL_ERR(LOG_TAG, "no new session allowed for stream %d", type);
        return result;
    }

    // check if param supported by audio configruation
    switch (type) {
        case QAL_STREAM_VOICE_CALL_RX:
        case QAL_STREAM_VOICE_CALL_TX:
        case QAL_STREAM_VOICE_CALL_RX_TX:
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_ULTRA_LOW_LATENCY:
        case QAL_STREAM_DEEP_BUFFER:
        case QAL_STREAM_GENERIC:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_PCM_OFFLOAD:
        case QAL_STREAM_LOOPBACK:
        case QAL_STREAM_PROXY:
            if (attributes->direction == QAL_AUDIO_INPUT) {
                channels = attributes->in_media_config.ch_info.channels;
                samplerate = attributes->in_media_config.sample_rate;
                bitwidth = attributes->in_media_config.bit_width;
            } else {
                channels = attributes->out_media_config.ch_info.channels;
                samplerate = attributes->out_media_config.sample_rate;
                bitwidth = attributes->out_media_config.bit_width;
            }
            rc = (StreamPCM::isBitWidthSupported(bitwidth) |
                  StreamPCM::isSampleRateSupported(samplerate) |
                  StreamPCM::isChannelSupported(channels));
            if (0 != rc) {
               QAL_ERR(LOG_TAG, "config not supported rc %d", rc);
               return result;
            }
            QAL_INFO(LOG_TAG, "config suppported");
            result = true;
            break;
        case QAL_STREAM_COMPRESSED:
            if (attributes->direction == QAL_AUDIO_INPUT) {
               channels = attributes->in_media_config.ch_info.channels;
               samplerate = attributes->in_media_config.sample_rate;
               bitwidth = attributes->in_media_config.bit_width;
            } else {
               channels = attributes->out_media_config.ch_info.channels;
               samplerate = attributes->out_media_config.sample_rate;
               bitwidth = attributes->out_media_config.bit_width;
            }
            rc = (StreamCompress::isBitWidthSupported(bitwidth) |
                  StreamCompress::isSampleRateSupported(samplerate) |
                  StreamCompress::isChannelSupported(channels));
            if (0 != rc) {
               QAL_ERR(LOG_TAG, "config not supported rc %d", rc);
               return result;
            }
            QAL_INFO(LOG_TAG, "config suppported");
            result = true;
            break;
        case QAL_STREAM_VOICE_UI:
            if (attributes->direction == QAL_AUDIO_INPUT) {
               channels = attributes->in_media_config.ch_info.channels;
               samplerate = attributes->in_media_config.sample_rate;
               bitwidth = attributes->in_media_config.bit_width;
            } else {
               channels = attributes->out_media_config.ch_info.channels;
               samplerate = attributes->out_media_config.sample_rate;
               bitwidth = attributes->out_media_config.bit_width;
            }
            rc = (StreamSoundTrigger::isBitWidthSupported(bitwidth) |
                  StreamSoundTrigger::isSampleRateSupported(samplerate) |
                  StreamSoundTrigger::isChannelSupported(channels));
            if (0 != rc) {
               QAL_ERR(LOG_TAG, "config not supported rc %d", rc);
               return result;
            }
            QAL_INFO(LOG_TAG, "config suppported");
            result = true;
            break;
        case QAL_STREAM_VOICE_CALL:
            channels = attributes->out_media_config.ch_info.channels;
            samplerate = attributes->out_media_config.sample_rate;
            bitwidth = attributes->out_media_config.bit_width;
            rc = (StreamPCM::isBitWidthSupported(bitwidth) |
                  StreamPCM::isSampleRateSupported(samplerate) |
                  StreamPCM::isChannelSupported(channels));
            if (0 != rc) {
               QAL_ERR(LOG_TAG, "config not supported rc %d", rc);
               return result;
            }
            QAL_INFO(LOG_TAG, "config suppported");
            result = true;
            break;

        default:
            QAL_ERR(LOG_TAG, "unknown type");
            return false;
    }
    QAL_DBG(LOG_TAG, "Exit. result %d", result);
    return result;
}

template <class T>
int registerstream(T s, std::vector<T> &streams)
{
    int ret = 0;
    streams.push_back(s);
    return ret;
}

int ResourceManager::registerStream(Stream *s)
{
    int ret = 0;
    qal_stream_type_t type;
    QAL_DBG(LOG_TAG, "Enter. stream %pK", s);
    ret = s->getStreamType(&type);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "getStreamType failed with status = %d", ret);
        return ret;
    }
    QAL_DBG(LOG_TAG, "stream type %d", type);
    mResourceManagerMutex.lock();
    switch (type) {
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_VOICE_CALL:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sPCM, active_streams_ll);
            break;
        }
        case QAL_STREAM_PCM_OFFLOAD:
        case QAL_STREAM_LOOPBACK:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sPCM, active_streams_po);
            break;
        }
        case QAL_STREAM_DEEP_BUFFER:
        {
            StreamPCM* sDB = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sDB, active_streams_db);
            break;
        }
        case QAL_STREAM_COMPRESSED:
        {
            StreamCompress* sComp = dynamic_cast<StreamCompress*>(s);
            ret = registerstream(sComp, active_streams_comp);
            break;
        }
        case QAL_STREAM_GENERIC:
        {
            StreamPCM* sULLA = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sULLA, active_streams_ulla);
            break;
        }
        case QAL_STREAM_VOICE_UI:
        {
            StreamSoundTrigger* sST = dynamic_cast<StreamSoundTrigger*>(s);
            ret = registerstream(sST, active_streams_st);
            break;
        }
        case QAL_STREAM_ULTRA_LOW_LATENCY:
        {
            StreamPCM* sULL = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sULL, active_streams_ull);
            break;
        }
        case QAL_STREAM_PROXY:
        {
            StreamPCM* sProxy = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sProxy, active_streams_proxy);
            break;
        }
        default:
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, " Invalid stream type = %d ret %d", type, ret);
            break;
    }
    mActiveStreams.push_back(s);

#if 0
    s->getStreamAttributes(&incomingStreamAttr);
    int incomingPriority = getStreamAttrPriority(incomingStreamAttr);
    if (incomingPriority > mPriorityHighestPriorityActiveStream) {
        QAL_INFO(LOG_TAG, "%s: Added new stream with highest priority %d", __func__, incomingPriority);
        mPriorityHighestPriorityActiveStream = incomingPriority;
        mHighestPriorityActiveStream = s;
    }
    calculalte priority and store in Stream

    mAllActiveStreams.push_back(s);
#endif

    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

///private functions


// template function to deregister stream
template <class T>
int deregisterstream(T s, std::vector<T> &streams)
{
    int ret = 0;
    typename std::vector<T>::iterator iter = std::find(streams.begin(), streams.end(), s);
    if (iter != streams.end())
        streams.erase(iter);
    else
        ret = -ENOENT;
    return ret;
}

int ResourceManager::deregisterStream(Stream *s)
{
    int ret = 0;
    qal_stream_type_t type;
    QAL_DBG(LOG_TAG, "Enter. stream %pK", s);
    ret = s->getStreamType(&type);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, " getStreamType failed with status = %d", ret);
        return ret;
    }
#if 0
    remove s from mAllActiveStreams
    get priority from remaining streams and find highest priority stream
    and store in mHighestPriorityActiveStream
#endif
    QAL_ERR(LOG_TAG, "stream type %d", type);
    mResourceManagerMutex.lock();
    switch (type) {
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_VOICE_CALL:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sPCM, active_streams_ll);
            break;
        }
        case QAL_STREAM_PCM_OFFLOAD:
        case QAL_STREAM_LOOPBACK:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sPCM, active_streams_po);
            break;
        }
        case QAL_STREAM_DEEP_BUFFER:
        {
            StreamPCM* sDB = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sDB, active_streams_db);
            break;
        }
        case QAL_STREAM_COMPRESSED:
        {
            StreamCompress* sComp = dynamic_cast<StreamCompress*>(s);
            ret = deregisterstream(sComp, active_streams_comp);
            break;
        }
        case QAL_STREAM_GENERIC:
        {
            StreamPCM* sULLA = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sULLA, active_streams_ulla);
            break;
        }
        case QAL_STREAM_VOICE_UI:
        {
            StreamSoundTrigger* sST = dynamic_cast<StreamSoundTrigger*>(s);
            ret = deregisterstream(sST, active_streams_st);
            break;
        }
        case QAL_STREAM_ULTRA_LOW_LATENCY:
        {
            StreamPCM* sULL = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sULL, active_streams_ull);
            break;
        }
        case QAL_STREAM_PROXY:
        {
            StreamPCM* sProxy = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sProxy, active_streams_proxy);
            break;
        }
        default:
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid stream type = %d ret %d", type, ret);
            break;
    }

    deregisterstream(s, mActiveStreams);
    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

int ResourceManager::registerDevice_l(std::shared_ptr<Device> d)
{
    QAL_DBG(LOG_TAG, "Enter.");
    active_devices.push_back(d);
    QAL_DBG(LOG_TAG, "Exit.");
    return 0;
}

int ResourceManager::registerDevice(std::shared_ptr<Device> d)
{
    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    registerDevice_l(d);
    mResourceManagerMutex.unlock();
    return 0;
}

int ResourceManager::deregisterDevice_l(std::shared_ptr<Device> d)
{
    int ret = 0;
    QAL_DBG(LOG_TAG, "Enter.");

    auto iter = std::find(active_devices.begin(), active_devices.end(), d);
    if (iter != active_devices.end())
        active_devices.erase(iter);
    else {
        ret = -ENOENT;
        QAL_ERR(LOG_TAG, "no device %d found in active device list ret %d",
                d->getSndDeviceId(), ret);
    }
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

int ResourceManager::deregisterDevice(std::shared_ptr<Device> d)
{
    int ret = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    deregisterDevice_l(d);
    mResourceManagerMutex.unlock();
    return ret;
}

bool ResourceManager::isDeviceActive(std::shared_ptr<Device> d)
{
    bool is_active = false;

    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    is_active = isDeviceActive_l(d);
    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit.");
    return is_active;
}

bool ResourceManager::isDeviceActive_l(std::shared_ptr<Device> d)
{
    bool is_active = false;
    int deviceId = d->getSndDeviceId();

    QAL_DBG(LOG_TAG, "Enter.");
    typename std::vector<std::shared_ptr<Device>>::iterator iter =
        std::find(active_devices.begin(), active_devices.end(), d);
    if (iter != active_devices.end()) {
        is_active = true;
    }

    QAL_DBG(LOG_TAG, "Exit. device %d is active %d", deviceId, is_active);
    return is_active;
}

int ResourceManager::addPlugInDevice(std::shared_ptr<Device> d,
                            qal_param_device_connection_t connection_state)
{
    int ret = 0;

    ret = d->init(connection_state);
    if (ret) {
        QAL_ERR(LOG_TAG, "failed to init deivce.");
        return ret;
    }

    plugin_devices_.push_back(d);
    return 0;
}

int ResourceManager::removePlugInDevice(qal_device_id_t device_id,
                            qal_param_device_connection_t connection_state)
{
    int ret = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    typename std::vector<std::shared_ptr<Device>>::iterator iter;

    for (iter = plugin_devices_.begin(); iter != plugin_devices_.end(); iter++) {
        if ((*iter)->getSndDeviceId() == device_id)
            break;
    }

    if (iter != plugin_devices_.end()) {
        (*iter)->deinit(connection_state);
        plugin_devices_.erase(iter);
    } else {
        ret = -ENOENT;
        QAL_ERR(LOG_TAG, "no device %d found in plugin device list ret %d",
                device_id, ret);
    }
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

int ResourceManager::getActiveDevices(std::vector<std::shared_ptr<Device>> &deviceList)
{
    int ret = 0;
    mResourceManagerMutex.lock();
    typename std::vector<std::shared_ptr<Device>>::iterator iter;
    for (iter = active_devices.begin(); iter != active_devices.end(); iter++)
        deviceList.push_back(*iter);
    mResourceManagerMutex.unlock();
    return ret;
}

int ResourceManager::getAudioRoute(struct audio_route** ar)
{
    if (!audio_route) {
        QAL_ERR(LOG_TAG, "no audio route found");
        return -ENOENT;
    }
    *ar = audio_route;
    QAL_DBG(LOG_TAG, "ar %pK audio_route %pK", ar, audio_route);
    return 0;
}

int ResourceManager::getAudioMixer(struct audio_mixer ** am)
{
    if (!audio_mixer || !am) {
        QAL_ERR(LOG_TAG, "no audio mixer found");
        return -ENOENT;
    }
    *am = audio_mixer;
    QAL_DBG(LOG_TAG, "ar %pK audio_mixer %pK", am, audio_mixer);
    return 0;
}

void ResourceManager::GetVoiceUIProperties(struct qal_st_properties *qstp)
{
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (!qstp) {
        return;
    }

    memcpy(qstp, &qst_properties, sizeof(struct qal_st_properties));

    if (st_info) {
        qstp->concurrent_capture = st_info->GetConcurrentCaptureEnable();
    }
}

void ResourceManager::GetVoiceUIStreams(std::vector<Stream*> &vui_streams) {

    mResourceManagerMutex.lock();
    for (auto& st: active_streams_st) {
        vui_streams.push_back(st);
    }
    mResourceManagerMutex.unlock();
}

bool ResourceManager::IsVoiceUILPISupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (st_info) {
        return st_info->GetLpiEnable();
    } else {
        return false;
    }
}

bool ResourceManager::CheckForActiveConcurrentNonLPIStream() {
    qal_stream_attributes st_attr;

    for (auto& s: mActiveStreams) {
        s->getStreamAttributes(&st_attr);
        if (st_attr.direction != QAL_AUDIO_INPUT &&
            st_attr.type != QAL_STREAM_LOW_LATENCY) {
                return true;
        }
    }
    return false;
}

bool ResourceManager::IsAudioCaptureAndVoiceUIConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (st_info) {
        return st_info->GetConcurrentCaptureEnable();
    }
    return false;
}

bool ResourceManager::IsVoiceCallAndVoiceUIConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (st_info) {
        return st_info->GetConcurrentVoiceCallEnable();
    }
    return false;
}

bool ResourceManager::IsVoipAndVoiceUIConcurrencySupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (st_info) {
        return st_info->GetConcurrentVoipCallEnable();
    }
    return false;
}

bool ResourceManager::IsTransitToNonLPIOnChargingSupported() {
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (st_info) {
        return st_info->GetTransitToNonLpiOnCharging();
    }
    return false;
}

bool ResourceManager::CheckForForcedTransitToNonLPI() {
    if (IsTransitToNonLPIOnChargingSupported() && charging_state_)
      return true;

    return false;
}

std::shared_ptr<CaptureProfile> ResourceManager::GetCaptureProfileByPriority(
    StreamSoundTrigger *s) {
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<CaptureProfile> cap_prof_priority = nullptr;

    for (int i = 0; i < active_streams_st.size(); i++) {
        // NOTE: input param s can be nullptr here
        if (active_streams_st[i] == s) {
            continue;
        }

        /*
         * Ignore capture profile for streams in below states:
         * 1. sound model loaded but not started by sthal
         * 2. stop recognition called by sthal
         */
        if (!active_streams_st[i]->GetActiveState())
            continue;

        cap_prof = active_streams_st[i]->GetCurrentCaptureProfile();
        if (!cap_prof) {
            QAL_ERR(LOG_TAG, "Failed to get capture profile");
            continue;
        } else if (cap_prof->ComparePriority(cap_prof_priority) ==
                   CAPTURE_PROFILE_PRIORITY_HIGH) {
            cap_prof_priority = cap_prof;
        }
    }

    return cap_prof_priority;
}

bool ResourceManager::UpdateSVACaptureProfile(StreamSoundTrigger *s, bool is_active) {
    int status = 0;
    bool backend_update = false;
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<CaptureProfile> cap_prof_priority = nullptr;

    if (!s) {
        QAL_ERR(LOG_TAG, "Invalid stream");
        return false;
    }

    // backend config update
    if (is_active) {
        cap_prof = s->GetCurrentCaptureProfile();
        if (!cap_prof) {
            QAL_ERR(LOG_TAG, "Failed to get capture profile");
            return false;
        }

        if (!SVACaptureProfile) {
            SVACaptureProfile = cap_prof;
        } else if (SVACaptureProfile->ComparePriority(cap_prof) < 0){
            SVACaptureProfile = cap_prof;
            backend_update = true;
        }
    } else {
        cap_prof_priority = GetCaptureProfileByPriority(s);

        if (!cap_prof_priority) {
            QAL_DBG(LOG_TAG, "No SVA session active, reset capture profile");
            SVACaptureProfile = nullptr;
        } else if (cap_prof_priority->ComparePriority(SVACaptureProfile) ==
                   CAPTURE_PROFILE_PRIORITY_HIGH) {
            SVACaptureProfile = cap_prof_priority;
            backend_update = true;
        }

    }

    return backend_update;
}

int ResourceManager::SwitchSVADevices(bool connect_state,
    qal_device_id_t device_id) {
    int32_t status = 0;
    qal_device_id_t dest_device;
    std::shared_ptr<CaptureProfile> cap_prof = nullptr;
    std::shared_ptr<CaptureProfile> cap_prof_priority = nullptr;

    QAL_DBG(LOG_TAG, "Enter");

    // TODO: add support for other devices
    if (device_id == QAL_DEVICE_IN_HANDSET_MIC ||
        device_id == QAL_DEVICE_IN_SPEAKER_MIC) {
        dest_device = QAL_DEVICE_IN_HANDSET_VA_MIC;
    } else if (device_id == QAL_DEVICE_IN_WIRED_HEADSET) {
        dest_device = QAL_DEVICE_IN_HEADSET_VA_MIC;
    } else {
        QAL_DBG(LOG_TAG, "Unsupported device %d", device_id);
        return status;
    }

    SVACaptureProfile = nullptr;
    cap_prof_priority = GetCaptureProfileByPriority(nullptr);

    if (!cap_prof_priority) {
        QAL_DBG(LOG_TAG, "No SVA session active, reset capture profile");
        SVACaptureProfile = nullptr;
    } else if (cap_prof_priority->ComparePriority(SVACaptureProfile) ==
               CAPTURE_PROFILE_PRIORITY_HIGH) {
        SVACaptureProfile = cap_prof_priority;
    }

    // handle device switch
    for (int i = 0; i < active_streams_st.size(); i++) {
        status = active_streams_st[i]->UpdateDeviceConnectionState(
            connect_state, dest_device);
        if (status) {
            QAL_ERR(LOG_TAG, "Failed to switch device for SVA");
        }
    }
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

std::shared_ptr<CaptureProfile> ResourceManager::GetSVACaptureProfile() {
    return SVACaptureProfile;
}

/* NOTE: there should be only one callback for each pcm id
 * so when new different callback register with same pcm id
 * older one will be overwritten
 */
int ResourceManager::registerMixerEventCallback(const std::vector<int> &DevIds,
                                                session_callback callback,
                                                void *cookie,
                                                bool is_register) {
    int status = 0;
    std::map<int, std::pair<session_callback, void *>>::iterator it;

    if (!callback || DevIds.size() <= 0) {
        QAL_ERR(LOG_TAG, "Invalid callback or pcm ids");
        return -EINVAL;
    }

    if (mixerEventRegisterCount == 0 && !is_register) {
        QAL_ERR(LOG_TAG, "Cannot deregister unregistered callback");
        return -EINVAL;
    }

    if (is_register) {
        for (int i = 0; i < DevIds.size(); i++) {
            it = mixerEventCallbackMap.find(DevIds[i]);
            if (it != mixerEventCallbackMap.end()) {
                QAL_DBG(LOG_TAG, "callback exists for pcm id %d, overwrite",
                    DevIds[i]);
                mixerEventCallbackMap.erase(it);
            }
            mixerEventCallbackMap.insert(std::make_pair(DevIds[i],
                std::make_pair(callback, cookie)));

        }
        if (mixerEventRegisterCount++ == 0) {
            QAL_DBG(LOG_TAG, "Creating mixer event thread");
            mixerEventTread = std::thread(mixerEventWaitThreadLoop, rm);
        }
    } else {
        for (int i = 0; i < DevIds.size(); i++) {
            it = mixerEventCallbackMap.find(DevIds[i]);
            if (it != mixerEventCallbackMap.end()) {
                QAL_DBG(LOG_TAG, "callback found for pcm id %d, remove",
                    DevIds[i]);
                if (callback == it->second.first) {
                    mixerEventCallbackMap.erase(it);
                } else {
                    QAL_ERR(LOG_TAG, "No matching callback found for pcm id %d",
                        DevIds[i]);
                }
            } else {
                QAL_ERR(LOG_TAG, "No callback found for pcm id %d", DevIds[i]);
            }
        }
        if (mixerEventRegisterCount-- == 1) {
            if (mixerEventTread.joinable()) {
                mixerEventTread.join();
            }
            QAL_DBG(LOG_TAG, "Mixer event thread joined");
        }
    }

    return status;
}

void ResourceManager::mixerEventWaitThreadLoop(
    std::shared_ptr<ResourceManager> rm) {
    int ret = 0;
    struct snd_ctl_event mixer_event = {0};
    struct mixer *mixer = nullptr;

    ret = rm->getAudioMixer(&mixer);
    if (ret) {
        QAL_ERR(LOG_TAG, "Failed to get audio mxier");
        return;
    }

    QAL_VERBOSE(LOG_TAG, "subscribing for event");
    mixer_subscribe_events(mixer, 1);

    while (1) {
        QAL_VERBOSE(LOG_TAG, "going to wait for event");
        /* TODO: set timeout here to avoid stuck during stop
         * Better if AGM side can provide one event indicating stop
         */
        ret = mixer_wait_event(mixer, 1000);
        QAL_VERBOSE(LOG_TAG, "mixer_wait_event returns %d", ret);
        if (ret <= 0) {
            QAL_DBG(LOG_TAG, "mixer_wait_event err! ret = %d", ret);
        } else if (ret > 0) {
            ret = mixer_read_event(mixer, &mixer_event);
            if (ret >= 0) {
                QAL_INFO(LOG_TAG, "Event Received %s",
                    mixer_event.data.elem.id.name);
                ret = rm->handleMixerEvent(mixer,
                    (char *)mixer_event.data.elem.id.name);
            } else {
                QAL_DBG(LOG_TAG, "mixer_read failed, ret = %d", ret);
            }
        }
        if (!rm->isCallbackRegistered()) {
            QAL_VERBOSE(LOG_TAG, "Exit thread as no session registered");
            break;
        }
    }
    QAL_VERBOSE(LOG_TAG, "unsubscribing for event");
    mixer_subscribe_events(mixer, 0);
}

int ResourceManager::handleMixerEvent(struct mixer *mixer, char *mixer_str) {
    int status = 0;
    int pcm_id = 0;
    void *cookie = nullptr;
    session_callback session_cb = nullptr;
    std::string event_str(mixer_str);
    // TODO: hard code in common defs
    std::string pcm_prefix = "PCM";
    std::string compress_prefix = "COMPRESS";
    std::string event_suffix = "event";
    size_t prefix_idx = 0;
    size_t suffix_idx = 0;
    size_t length = 0;
    struct mixer_ctl *ctl = nullptr;
    char *buf = nullptr;
    unsigned int num_values;
    struct agm_event_cb_params *params = nullptr;
    std::map<int, std::pair<session_callback, void *>>::iterator it;

    QAL_DBG(LOG_TAG, "Enter");
    ctl = mixer_get_ctl_by_name(mixer, mixer_str);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s", mixer_str);
        status = -EINVAL;
        goto exit;
    }

    // parse event payload
    num_values = mixer_ctl_get_num_values(ctl);
    QAL_VERBOSE(LOG_TAG, "num_values: %d", num_values);
    buf = (char *)calloc(1, num_values);
    if (!buf) {
        QAL_ERR(LOG_TAG, "Failed to allocate buf");
        status = -ENOMEM;
        goto exit;
    }

    status = mixer_ctl_get_array(ctl, buf, num_values);
    if (status < 0) {
        QAL_ERR(LOG_TAG, "Failed to mixer_ctl_get_array");
        goto exit;
    }

    params = (struct agm_event_cb_params *)buf;
    QAL_DBG(LOG_TAG, "source module id %x, event id %d, payload size %d",
            params->source_module_id, params->event_id,
            params->event_payload_size);

    if (!params->source_module_id || !params->event_payload_size) {
        QAL_ERR(LOG_TAG, "Invalid source module id or payload size");
        goto exit;
    }

    // NOTE: event we get should be in format like "PCM100 event"
    prefix_idx = event_str.find(pcm_prefix);
    if (prefix_idx == event_str.npos) {
        prefix_idx = event_str.find(compress_prefix);
        if (prefix_idx == event_str.npos) {
            QAL_ERR(LOG_TAG, "Invalid mixer event");
            status = -EINVAL;
            goto exit;
        } else {
            prefix_idx += compress_prefix.length();
        }
    } else {
        prefix_idx += pcm_prefix.length();
    }

    suffix_idx = event_str.find(event_suffix);
    if (suffix_idx == event_str.npos || suffix_idx - prefix_idx <= 1) {
        QAL_ERR(LOG_TAG, "Invalid mixer event");
        status = -EINVAL;
        goto exit;
    }

    length = suffix_idx - prefix_idx;
    pcm_id = std::stoi(event_str.substr(prefix_idx, length));

    // acquire callback/cookie with pcm dev id
    it = mixerEventCallbackMap.find(pcm_id);
    if (it != mixerEventCallbackMap.end()) {
        session_cb = it->second.first;
        cookie = it->second.second;
    }

    if (!session_cb) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid session callback");
        goto exit;
    }

    // callback
    session_cb(cookie, params->event_id, (void *)params->event_payload,
                 params->event_payload_size);

exit:
    if (buf)
        free(buf);
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int ResourceManager::StopOtherSVAStreams(StreamSoundTrigger *st) {
    int status = 0;
    StreamSoundTrigger *st_str = nullptr;

    mResourceManagerMutex.lock();
    for (int i = 0; i < active_streams_st.size(); i++) {
        st_str = active_streams_st[i];
        if (st_str && st_str != st) {
            status = st_str->ExternalStop();
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to do external stop");
            }
        }
    }
    mResourceManagerMutex.unlock();

    return status;
}

int ResourceManager::StartOtherSVAStreams(StreamSoundTrigger *st) {
    int status = 0;
    StreamSoundTrigger *st_str = nullptr;

    mResourceManagerMutex.lock();
    for (int i = 0; i < active_streams_st.size(); i++) {
        st_str = active_streams_st[i];
        if (st_str && st_str != st) {
            status = st_str->ExternalStart();
            if (status) {
                QAL_ERR(LOG_TAG, "Failed to do external start");
            }
        }
    }
    mResourceManagerMutex.unlock();

    return status;
}

std::shared_ptr<Device> ResourceManager::getActiveEchoReferenceRxDevices_l(
    Stream *tx_str)
{
    int status = 0;
    int deviceId = 0;
    std::shared_ptr<Device> rx_device = nullptr;
    std::shared_ptr<Device> tx_device = nullptr;
    struct qal_stream_attributes tx_attr;
    struct qal_stream_attributes rx_attr;
    std::vector <std::shared_ptr<Device>> tx_device_list;
    std::vector <std::shared_ptr<Device>> rx_device_list;

    // check stream direction
    status = tx_str->getStreamAttributes(&tx_attr);
    if (status) {
        QAL_ERR(LOG_TAG, "stream get attributes failed");
        goto exit;
    }
    if (tx_attr.direction != QAL_AUDIO_INPUT) {
        QAL_ERR(LOG_TAG, "invalid stream direction %d", tx_attr.direction);
        status = -EINVAL;
        goto exit;
    }

    // get associated device list
    status = tx_str->getAssociatedDevices(tx_device_list);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get associated device, status %d", status);
        goto exit;
    }

    for (auto& rx_str: mActiveStreams) {
        rx_str->getStreamAttributes(&rx_attr);
        rx_device_list.clear();
        if (rx_attr.direction != QAL_AUDIO_INPUT) {
            if (!getEcRefStatus(tx_attr.type, rx_attr.type)) {
                QAL_DBG(LOG_TAG, "No need to enable ec ref for rx %d tx %d",
                        rx_attr.type, tx_attr.type);
                continue;
            }
            rx_str->getAssociatedDevices(rx_device_list);
            for (int i = 0; i < rx_device_list.size(); i++) {
                if (!isDeviceActive_l(rx_device_list[i]))
                    continue;
                deviceId = rx_device_list[i]->getSndDeviceId();
                if (deviceId > QAL_DEVICE_OUT_MIN &&
                    deviceId < QAL_DEVICE_OUT_MAX)
                    rx_device = rx_device_list[i];
                else
                    rx_device = nullptr;
                for (int j = 0; j < tx_device_list.size(); j++) {
                    tx_device = tx_device_list[j];
                    if (checkECRef(rx_device, tx_device))
                        goto exit;
                }
            }
            rx_device = nullptr;
        } else {
            continue;
        }
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return rx_device;
}

std::shared_ptr<Device> ResourceManager::getActiveEchoReferenceRxDevices(
    Stream *tx_str)
{
    std::shared_ptr<Device> rx_device = nullptr;
    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    rx_device = getActiveEchoReferenceRxDevices_l(tx_str);
    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit.");
    return rx_device;
}

std::vector<Stream*> ResourceManager::getConcurrentTxStream_l(
    Stream *rx_str,
    std::shared_ptr<Device> rx_device)
{
    int deviceId = 0;
    int status = 0;
    std::vector<Stream*> tx_stream_list;
    struct qal_stream_attributes tx_attr;
    struct qal_stream_attributes rx_attr;
    std::shared_ptr<Device> tx_device = nullptr;
    std::vector <std::shared_ptr<Device>> tx_device_list;

    // check stream direction
    status = rx_str->getStreamAttributes(&rx_attr);
    if (status) {
        QAL_ERR(LOG_TAG, "stream get attributes failed");
        goto exit;
    }
    if (rx_attr.direction != QAL_AUDIO_OUTPUT) {
        QAL_ERR(LOG_TAG, "Invalid stream direction %d", rx_attr.direction);
        status = -EINVAL;
        goto exit;
    }

    for (auto& tx_str: mActiveStreams) {
        tx_device_list.clear();
        tx_str->getStreamAttributes(&tx_attr);
        if (tx_attr.direction == QAL_AUDIO_INPUT) {
            if (!getEcRefStatus(tx_attr.type, rx_attr.type)) {
                QAL_DBG(LOG_TAG, "No need to enable ec ref for rx %d tx %d",
                        rx_attr.type, tx_attr.type);
                continue;
            }
            tx_str->getAssociatedDevices(tx_device_list);
            for (int i = 0; i < tx_device_list.size(); i++) {
                if (!isDeviceActive_l(tx_device_list[i]))
                    continue;
                deviceId = tx_device_list[i]->getSndDeviceId();
                if (deviceId > QAL_DEVICE_IN_MIN &&
                    deviceId < QAL_DEVICE_IN_MAX)
                    tx_device = tx_device_list[i];
                else
                    tx_device = nullptr;

                if (checkECRef(rx_device, tx_device)) {
                    tx_stream_list.push_back(tx_str);
                    break;
                }
            }
        }
    }
exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);
    return tx_stream_list;
}

std::vector<Stream*> ResourceManager::getConcurrentTxStream(
    Stream *rx_str,
    std::shared_ptr<Device> rx_device)
{
    std::vector<Stream*> tx_stream_list;
    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    tx_stream_list = getConcurrentTxStream_l(rx_str, rx_device);
    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit.");
    return tx_stream_list;
}

bool ResourceManager::checkECRef(std::shared_ptr<Device> rx_dev,
                                 std::shared_ptr<Device> tx_dev)
{
    bool result = false;
    int rx_dev_id = 0;
    int tx_dev_id = 0;

    if (!rx_dev || !tx_dev)
        return result;

    rx_dev_id = rx_dev->getSndDeviceId();
    tx_dev_id = tx_dev->getSndDeviceId();
    // TODO: address all possible combinations
    if ((rx_dev_id == QAL_DEVICE_OUT_SPEAKER) ||
        (rx_dev_id == QAL_DEVICE_OUT_HANDSET &&
         tx_dev_id == QAL_DEVICE_IN_HANDSET_MIC) ||
        (rx_dev_id == QAL_DEVICE_OUT_WIRED_HEADSET &&
         tx_dev_id == QAL_DEVICE_IN_WIRED_HEADSET) ||
        (rx_dev_id == QAL_DEVICE_OUT_HANDSET &&
         tx_dev_id == QAL_DEVICE_IN_HANDSET_VA_MIC) ||
        (rx_dev_id == QAL_DEVICE_OUT_WIRED_HEADSET &&
         tx_dev_id == QAL_DEVICE_IN_HEADSET_VA_MIC) ||
        (rx_dev_id == QAL_DEVICE_OUT_BLUETOOTH_A2DP &&
         tx_dev_id == QAL_DEVICE_IN_HEADSET_VA_MIC) ||
        (rx_dev_id == QAL_DEVICE_OUT_BLUETOOTH_A2DP &&
         tx_dev_id == QAL_DEVICE_IN_HANDSET_VA_MIC))
        result = true;

    QAL_DBG(LOG_TAG, "EC Ref: %d, rx dev: %d, tx dev: %d",
        result, rx_dev_id, tx_dev_id);

    return result;
}

//TBD: test this piece later, for concurrency
#if 1
template <class T>
void ResourceManager::getHigherPriorityActiveStreams(const int inComingStreamPriority, std::vector<Stream*> &activestreams,
                      std::vector<T> sourcestreams)
{
    int existingStreamPriority = 0;
    qal_stream_attributes sAttr;


    typename std::vector<T>::iterator iter = sourcestreams.begin();


    for(iter; iter != sourcestreams.end(); iter++) {
        (*iter)->getStreamAttributes(&sAttr);

        existingStreamPriority = getStreamAttrPriority(&sAttr);
        if (existingStreamPriority > inComingStreamPriority)
        {
            activestreams.push_back(*iter);
        }
    }
}
#endif


template <class T>
void getActiveStreams(std::shared_ptr<Device> d, std::vector<Stream*> &activestreams,
                      std::vector<T> sourcestreams)
{
    for(typename std::vector<T>::iterator iter = sourcestreams.begin();
                 iter != sourcestreams.end(); iter++) {
        std::vector <std::shared_ptr<Device>> devices;
        (*iter)->getAssociatedDevices(devices);
        typename std::vector<std::shared_ptr<Device>>::iterator result =
                 std::find(devices.begin(), devices.end(), d);
        if (result != devices.end())
            activestreams.push_back(*iter);
    }
}

int ResourceManager::getActiveStream_l(std::shared_ptr<Device> d,
                                     std::vector<Stream*> &activestreams)
{
    int ret = 0;
    // merge all types of active streams into activestreams
    getActiveStreams(d, activestreams, active_streams_ll);
    getActiveStreams(d, activestreams, active_streams_ull);
    getActiveStreams(d, activestreams, active_streams_ulla);
    getActiveStreams(d, activestreams, active_streams_db);
    getActiveStreams(d, activestreams, active_streams_comp);
    getActiveStreams(d, activestreams, active_streams_st);
    getActiveStreams(d, activestreams, active_streams_po);
    getActiveStreams(d, activestreams, active_streams_proxy);

    if (activestreams.empty()) {
        ret = -ENOENT;
        QAL_ERR(LOG_TAG, "no active streams found for device %d ret %d", d->getSndDeviceId(), ret);
    }

    return ret;
}

int ResourceManager::getActiveStream(std::shared_ptr<Device> d,
                                     std::vector<Stream*> &activestreams)
{
    int ret = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    mResourceManagerMutex.lock();
    ret = getActiveStream_l(d, activestreams);
    mResourceManagerMutex.unlock();
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

/*blsUpdated - to specify if the config is updated by rm*/
int ResourceManager::checkAndGetDeviceConfig(struct qal_device *device, bool* blsUpdated)
{
    int ret = -EINVAL;
    if (!device || !blsUpdated) {
        QAL_ERR(LOG_TAG, "Invalid input parameter ret %d", ret);
        return ret;
    }
    //TODO:check if device config is supported
    bool dev_supported = false;
    *blsUpdated = false;
    uint16_t channels = device->config.ch_info.channels;
    uint32_t samplerate = device->config.sample_rate;
    uint32_t bitwidth = device->config.bit_width;

    QAL_DBG(LOG_TAG, "Enter.");
    //TODO: check and rewrite params if needed
    // only compare with default value for now
    // because no config file parsed in init
    if (channels != DEFAULT_CHANNELS) {
        if (bOverwriteFlag) {
            device->config.ch_info.channels = DEFAULT_CHANNELS;
            *blsUpdated = true;
        }
    } else if (samplerate != DEFAULT_SAMPLE_RATE) {
        if (bOverwriteFlag) {
            device->config.sample_rate = DEFAULT_SAMPLE_RATE;
            *blsUpdated = true;
        }
    } else if (bitwidth != DEFAULT_BIT_WIDTH) {
        if (bOverwriteFlag) {
            device->config.bit_width = DEFAULT_BIT_WIDTH;
            *blsUpdated = true;
        }
    } else {
        ret = 0;
        dev_supported = true;
    }
    QAL_DBG(LOG_TAG, "Exit. ret %d", ret);
    return ret;
}

std::shared_ptr<ResourceManager> ResourceManager::getInstance()
{
    QAL_INFO(LOG_TAG, "Enter.");
    if(!rm) {
        std::lock_guard<std::mutex> lock(ResourceManager::mResourceManagerMutex);
        if (!rm) {
            std::shared_ptr<ResourceManager> sp(new ResourceManager());
            rm = sp;
        }
    }
    QAL_INFO(LOG_TAG, "Exit.");
    return rm;
}

int ResourceManager::getSndCard()
{
    return snd_card;
}

int ResourceManager::getSndDeviceName(int deviceId, char *device_name)
{
    if (isValidDevId(deviceId)) {
        strlcpy(device_name, sndDeviceNameLUT[deviceId].second.c_str(), DEVICE_NAME_MAX_SIZE);
    } else {
        strlcpy(device_name, "", DEVICE_NAME_MAX_SIZE);
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
        return -EINVAL;
    }
    return 0;
}

int ResourceManager::getDeviceEpName(int deviceId, std::string &epName)
{
    if (isValidDevId(deviceId)) {
        epName.assign(deviceLinkName[deviceId].second);
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
        return -EINVAL;
    }
    return 0;
}

// TODO: Should pcm device be related to usecases used(ll/db/comp/ulla)?
// Use Low Latency as default by now
int ResourceManager::getPcmDeviceId(int deviceId)
{
    int pcm_device_id = -1;
    if (!isValidDevId(deviceId)) {
        QAL_ERR(LOG_TAG, " Invalid device id %d", deviceId);
        return -EINVAL;
    }

    pcm_device_id = devicePcmId[deviceId].second;
    return pcm_device_id;
}

void ResourceManager::deinit()
{
    const qal_alsa_or_gsl ag = rm->getQALConfigALSAOrGSL();
    rm = nullptr;
    mixer_close(audio_mixer);
    if (ag == GSL) {
        SessionGsl::deinit();
    }
    if (audio_route)
    {
        audio_route_free(audio_route);
    }

    if (sndmon)
        delete sndmon;

}

int ResourceManager::getStreamTag(std::vector <int> &tag)
{
    int status = 0;
    for (int i=0; i < streamTag.size(); i++) {
        tag.push_back(streamTag[i]);
    }
    return status;
}

int ResourceManager::getStreamPpTag(std::vector <int> &tag)
{
    int status = 0;
    for (int i=0; i < streamPpTag.size(); i++) {
        tag.push_back(streamPpTag[i]);
    }
    return status;
}

int ResourceManager::getMixerTag(std::vector <int> &tag)
{
    int status = 0;
    for (int i=0; i < mixerTag.size(); i++) {
        tag.push_back(mixerTag[i]);
    }
    return status;
}

int ResourceManager::getDeviceTag(std::vector <int> &tag)
{
    int status = 0;
    for (int i=0; i < deviceTag.size(); i++) {
        tag.push_back(deviceTag[i]);
    }
    return status;
}

int ResourceManager::getDevicePpTag(std::vector <int> &tag)
{
    int status = 0;
    for (int i=0; i < devicePpTag.size(); i++) {
        tag.push_back(devicePpTag[i]);
    }
    return status;
}

qal_alsa_or_gsl ResourceManager::getQALConfigALSAOrGSL() const {

//TODO move this to xml configuration

   return ALSA;
//#ifdef GSL
   // return GSL;

}

int ResourceManager::getNumFEs(const qal_stream_type_t sType) const
{
    int n = 1;

    switch (sType) {
        case QAL_STREAM_LOOPBACK:
        case QAL_STREAM_TRANSCODE:
            n = 1;
            break;
        default:
            n = 1;
            break;
    }

    return n;
}

const std::vector<int> ResourceManager::allocateFrontEndIds(const struct qal_stream_attributes sAttr, int lDirection)
{
    //TODO: lock resource manager
    std::vector<int> f;
    f.clear();
    const int howMany = getNumFEs(sAttr.type);
    int id = 0;
    std::vector<int>::iterator it;

    switch(sAttr.type) {
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_ULTRA_LOW_LATENCY:
        case QAL_STREAM_DEEP_BUFFER:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_VOICE_UI:
        case QAL_STREAM_PCM_OFFLOAD:
        case QAL_STREAM_LOOPBACK:
        case QAL_STREAM_PROXY:
            switch (sAttr.direction) {
                case QAL_AUDIO_INPUT:
                    if ( howMany > listAllPcmRecordFrontEnds.size()) {
                        QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                          howMany, listAllPcmRecordFrontEnds.size());
                        goto error;
                    }
                    id = (listAllPcmRecordFrontEnds.size() - 1);
                    it =  (listAllPcmRecordFrontEnds.begin() + id);
                    for (int i = 0; i < howMany; i++) {
                        f.push_back(listAllPcmRecordFrontEnds.at(id));
                        listAllPcmRecordFrontEnds.erase(it);
                        QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                        it -= 1;
                        id -= 1;
                    }
                    break;
                case QAL_AUDIO_OUTPUT:
                    if ( howMany > listAllPcmPlaybackFrontEnds.size()) {
                        QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                          howMany, listAllPcmPlaybackFrontEnds.size());
                        goto error;
                    }
                    id = (listAllPcmPlaybackFrontEnds.size() - 1);
                    it =  (listAllPcmPlaybackFrontEnds.begin() + id);
                    for (int i = 0; i < howMany; i++) {
                        f.push_back(listAllPcmPlaybackFrontEnds.at(id));
                        listAllPcmPlaybackFrontEnds.erase(it);
                        QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                        it -= 1;
                        id -= 1;
                    }
                    break;
                case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
                    if (lDirection == RXLOOPBACK) {
                        if ( howMany > listAllPcmLoopbackRxFrontEnds.size()) {
                            QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                              howMany, listAllPcmLoopbackRxFrontEnds.size());
                            goto error;
                        }
                        id = (listAllPcmLoopbackRxFrontEnds.size() - 1);
                        it =  (listAllPcmLoopbackRxFrontEnds.begin() + id);
                        for (int i = 0; i < howMany; i++) {
                           f.push_back(listAllPcmLoopbackRxFrontEnds.at(id));
                           listAllPcmLoopbackRxFrontEnds.erase(it);
                           QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                           it -= 1;
                           id -= 1;
                        }
                    } else {
                        if ( howMany > listAllPcmLoopbackTxFrontEnds.size()) {
                            QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                              howMany, listAllPcmLoopbackTxFrontEnds.size());
                            goto error;
                        }
                        id = (listAllPcmLoopbackTxFrontEnds.size() - 1);
                        it =  (listAllPcmLoopbackTxFrontEnds.begin() + id);
                        for (int i = 0; i < howMany; i++) {
                           f.push_back(listAllPcmLoopbackTxFrontEnds.at(id));
                           listAllPcmLoopbackTxFrontEnds.erase(it);
                           QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                           it -= 1;
                           id -= 1;
                        }
                    }
                    break;
                default:
                    QAL_ERR(LOG_TAG,"direction unsupported");
                    break;
            }
            break;
        case QAL_STREAM_COMPRESSED:
            switch (sAttr.direction) {
                case QAL_AUDIO_INPUT:
                    if ( howMany > listAllCompressRecordFrontEnds.size()) {
                        QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                          howMany, listAllCompressRecordFrontEnds.size());
                        goto error;
                    }
                    id = (listAllCompressRecordFrontEnds.size() - 1);
                    it =  (listAllCompressRecordFrontEnds.begin() + id);
                    for (int i = 0; i < howMany; i++) {
                        f.push_back(listAllCompressRecordFrontEnds.at(id));
                        listAllCompressRecordFrontEnds.erase(it);
                        QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                        it -= 1;
                        id -= 1;
                    }
                    break;
                case QAL_AUDIO_OUTPUT:
                    if ( howMany > listAllCompressPlaybackFrontEnds.size()) {
                        QAL_ERR(LOG_TAG, "allocateFrontEndIds: requested for %d front ends, have only %d error",
                                          howMany, listAllCompressPlaybackFrontEnds.size());
                        goto error;
                    }
                    id = (listAllCompressPlaybackFrontEnds.size() - 1);
                    it =  (listAllCompressPlaybackFrontEnds.begin() + id);
                    for (int i = 0; i < howMany; i++) {
                        f.push_back(listAllCompressPlaybackFrontEnds.at(id));
                        listAllCompressPlaybackFrontEnds.erase(it);
                        QAL_INFO(LOG_TAG, "allocateFrontEndIds: front end %d", f[i]);
                        it -= 1;
                        id -= 1;
                    }
                    break;
                default:
                    QAL_ERR(LOG_TAG,"direction unsupported");
                    break;
                }
                break;
        case QAL_STREAM_VOICE_CALL:
            switch (sAttr.direction) {
              case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
                    if (lDirection == RXLOOPBACK) {
                        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
                            sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
                            f = allocateVoiceFrontEndIds(listAllPcmVoice1RxFrontEnds, howMany);
                        } else if(sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                            sAttr.info.voice_call_info.VSID == VOICELBMMODE2){
                            f = allocateVoiceFrontEndIds(listAllPcmVoice2RxFrontEnds, howMany);
                        } else {
                            QAL_ERR(LOG_TAG,"invalid VSID 0x%x provided",
                                    sAttr.info.voice_call_info.VSID);
                        }
                    } else {
                        if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
                            sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
                            f = allocateVoiceFrontEndIds(listAllPcmVoice1TxFrontEnds, howMany);
                        } else if(sAttr.info.voice_call_info.VSID == VOICEMMODE2 ||
                            sAttr.info.voice_call_info.VSID == VOICELBMMODE2){
                            f = allocateVoiceFrontEndIds(listAllPcmVoice2TxFrontEnds, howMany);
                        } else {
                            QAL_ERR(LOG_TAG,"invalid VSID 0x%x provided",
                                    sAttr.info.voice_call_info.VSID);
                        }
                    }
                    break;
              default:
                  QAL_ERR(LOG_TAG,"direction unsupported voice must be RX and TX");
                  break;
            }
            break;
        default:
            break;
    }

error:
    return f;
}


const std::vector<int> ResourceManager::allocateVoiceFrontEndIds(std::vector<int> listAllPcmVoiceFrontEnds, const int howMany)
{
    std::vector<int> f;
    f.clear();
    int id = 0;
    std::vector<int>::iterator it;
    if ( howMany > listAllPcmVoiceFrontEnds.size()) {
        QAL_ERR(LOG_TAG, "allocate voice FrontEndIds: requested for %d front ends, have only %d error",
                howMany, listAllPcmVoiceFrontEnds.size());
        return f;
    }
    id = (listAllPcmVoiceFrontEnds.size() - 1);
    it =  (listAllPcmVoiceFrontEnds.begin() + id);
    for (int i = 0; i < howMany; i++) {
        f.push_back(listAllPcmVoiceFrontEnds.at(id));
        listAllPcmVoiceFrontEnds.erase(it);
        QAL_INFO(LOG_TAG, "allocate VoiceFrontEndIds: front end %d", f[i]);
        it -= 1;
        id -= 1;
    }

    return f;
}
void ResourceManager::freeFrontEndIds(const std::vector<int> frontend,
                                      const struct qal_stream_attributes sAttr,
                                      int lDirection)
{
    QAL_INFO(LOG_TAG, "stream type %d, freeing %d\n", sAttr.type,
             frontend.at(0));

    switch(sAttr.type) {
        case QAL_STREAM_LOW_LATENCY:
        case QAL_STREAM_ULTRA_LOW_LATENCY:
        case QAL_STREAM_PROXY:
        case QAL_STREAM_DEEP_BUFFER:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_VOICE_UI:
        case QAL_STREAM_PCM_OFFLOAD:
            switch (sAttr.direction) {
                case QAL_AUDIO_INPUT:
                    for (int i = 0; i < frontend.size(); i++) {
                        listAllPcmRecordFrontEnds.push_back(frontend.at(i));
                    }
                    break;
                case QAL_AUDIO_OUTPUT:
                    for (int i = 0; i < frontend.size(); i++) {
                        listAllPcmPlaybackFrontEnds.push_back(frontend.at(i));
                    }
                    break;
                case QAL_AUDIO_INPUT | QAL_AUDIO_OUTPUT:
                    if (lDirection == RXLOOPBACK) {
                        for (int i = 0; i < frontend.size(); i++) {
                            listAllPcmLoopbackRxFrontEnds.push_back(frontend.at(i));
                        }
                    } else {
                        for (int i = 0; i < frontend.size(); i++) {
                            listAllPcmLoopbackTxFrontEnds.push_back(frontend.at(i));
                        }
                    }
                    break;
                default:
                    QAL_ERR(LOG_TAG,"direction unsupported");
                    break;
            }
            break;

        case QAL_STREAM_VOICE_CALL:
            if (lDirection == RXLOOPBACK) {
                for (int i = 0; i < frontend.size(); i++) {
                    if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
                        sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
                        listAllPcmVoice1RxFrontEnds.push_back(frontend.at(i));
                    } else {
                        listAllPcmVoice2RxFrontEnds.push_back(frontend.at(i));
                    }

                }
            } else {
                for (int i = 0; i < frontend.size(); i++) {
                    if (sAttr.info.voice_call_info.VSID == VOICEMMODE1 ||
                        sAttr.info.voice_call_info.VSID == VOICELBMMODE1) {
                        listAllPcmVoice1TxFrontEnds.push_back(frontend.at(i));
                    } else {
                        listAllPcmVoice2TxFrontEnds.push_back(frontend.at(i));
                    }
                }
            }
            break;

        case QAL_STREAM_COMPRESSED:
            switch (sAttr.direction) {
                case QAL_AUDIO_INPUT:
                    for (int i = 0; i < frontend.size(); i++) {
                        listAllCompressRecordFrontEnds.push_back(frontend.at(i));
                    }
                    break;
                case QAL_AUDIO_OUTPUT:
                    for (int i = 0; i < frontend.size(); i++) {
                        listAllCompressPlaybackFrontEnds.push_back(frontend.at(i));
                    }
                    break;
                default:
                    QAL_ERR(LOG_TAG,"direction unsupported");
                    break;
                }
            break;
        default:
            break;
    }
    return;
}

void ResourceManager::getSharedBEActiveStreamDevs(std::vector <std::tuple<Stream *, uint32_t>> &activeStreamsDevices,
                                                  int dev_id)
{
    std::string backEndName;
    std::shared_ptr<Device> dev;
    std::vector <Stream *> activeStreams;

    if (isValidDevId(dev_id) && (dev_id != QAL_DEVICE_NONE))
        backEndName = listAllBackEndIds[dev_id].second;
    for (int i = QAL_DEVICE_OUT_MIN; i < QAL_DEVICE_IN_MAX; i++) {
        if ((i != dev_id) && (backEndName == listAllBackEndIds[i].second)) {
            dev = Device::getObject((qal_device_id_t) i);
            if(dev) {
                getActiveStream_l(dev, activeStreams);
                QAL_DBG(LOG_TAG, "got dev %d active streams on dev is %d", i, activeStreams.size() );
                for (int j=0; j < activeStreams.size(); j++) {
                    activeStreamsDevices.push_back({activeStreams[j], i});
                    QAL_DBG(LOG_TAG, "found shared BE stream %pK with dev %d", activeStreams[j], i );
                }
            }
            activeStreams.clear();
        }
    }
}

const std::vector<std::string> ResourceManager::getBackEndNames(
        const std::vector<std::shared_ptr<Device>> &deviceList) const
{
    std::vector<std::string> backEndNames;
    std::string epname;
    backEndNames.clear();

    int dev_id;

    for (int i = 0; i < deviceList.size(); i++) {
        dev_id = deviceList[i]->getSndDeviceId();
        QAL_ERR(LOG_TAG, "device id %d", dev_id);
        if (isValidDevId(dev_id)) {
            epname.assign(listAllBackEndIds[dev_id].second);
            backEndNames.push_back(epname);
        } else {
            QAL_ERR(LOG_TAG, "Invalid device id %d", dev_id);
        }
    }

    for (int i = 0; i < backEndNames.size(); i++) {
        QAL_ERR(LOG_TAG, "getBackEndNames: going to return %s", backEndNames[i].c_str());
    }

    return backEndNames;
}

void ResourceManager::getBackEndNames(
        const std::vector<std::shared_ptr<Device>> &deviceList,
        std::vector<std::pair<int32_t, std::string>> &rxBackEndNames,
        std::vector<std::pair<int32_t, std::string>> &txBackEndNames) const
{
    std::string epname;
    rxBackEndNames.clear();
    txBackEndNames.clear();

    int dev_id;

    for (int i = 0; i < deviceList.size(); i++) {
        dev_id = deviceList[i]->getSndDeviceId();
        if (dev_id > QAL_DEVICE_OUT_MIN && dev_id < QAL_DEVICE_OUT_MAX) {
            epname.assign(listAllBackEndIds[dev_id].second);
            rxBackEndNames.push_back(std::make_pair(dev_id, epname));
        } else if (dev_id > QAL_DEVICE_IN_MIN && dev_id < QAL_DEVICE_IN_MAX) {
            epname.assign(listAllBackEndIds[dev_id].second);
            txBackEndNames.push_back(std::make_pair(dev_id, epname));
        } else {
            QAL_ERR(LOG_TAG, "Invalid device id %d", dev_id);
        }
    }

    for (int i = 0; i < rxBackEndNames.size(); i++)
        QAL_ERR(LOG_TAG, "getBackEndNames (RX): %s", rxBackEndNames[i].second.c_str());
    for (int i = 0; i < txBackEndNames.size(); i++)
        QAL_ERR(LOG_TAG, "getBackEndNames (TX): %s", txBackEndNames[i].second.c_str());
}
#if 0
const bool ResourceManager::shouldDeviceSwitch(const qal_stream_attributes* sExistingAttr,
    const qal_stream_attributes* sIncomingAttr) const {

    bool dSwitch = false;
    int existingPriority = 0;
    int incomingPriority = 0;
    bool ifVoice

    if (!sExistingAttr || !sIncomingAttr)
        goto error;

    existingPriority = getStreamAttrPriority(existingStream->getStreamAttributes(&sExistingAttr));
    incomingPriority = getStreamAttrPriority(incomingStream->getStreamAttributes(&sIncomingAttr));

    dSwitch = (incomingPriority > existingPriority);

    QAL_VERBOSE(LOG_TAG, "should Device switch or not %d, incoming Stream priority %d, existing stream priority %d",
        dSwitch, incomingPriority, existingPriority);

error:
    return dSwitch;
}
#endif

bool ResourceManager::isDeviceSwitchRequired(struct qal_device *activeDevAttr,
         struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr)
{
    bool is_ds_required = false;
    /*  This API may need stream attributes also to decide the priority like voice call has high priority */
    /* Right now assume all playback streams are same priority and decide based on Active Device config */

    if (!activeDevAttr || !inDevAttr || !inStrAttr) {
        QAL_ERR(LOG_TAG, "Invalid input parameter ");
        return is_ds_required;
    }

    switch (inDevAttr->id) {
    /* speaker is always at 48k, 16 bit, 2 ch */
    case QAL_DEVICE_OUT_SPEAKER:
        is_ds_required = false;
        break;
    case QAL_DEVICE_OUT_USB_HEADSET:
    case QAL_DEVICE_OUT_USB_DEVICE:
        if ((activeDevAttr->config.sample_rate == SAMPLINGRATE_44K) &&
            (inStrAttr->type == QAL_STREAM_LOW_LATENCY) ) {
            QAL_INFO(LOG_TAG, "active stream is at 44.1kHz.");
            is_ds_required = false;
        } else if ((QAL_AUDIO_OUTPUT == inStrAttr->direction) &&
            (inDevAttr->config.sample_rate % SAMPLINGRATE_44K == 0)) {
            //Native Audio usecase
            QAL_ERR(LOG_TAG, "1 inDevAttr->config.sample_rate = %d  ", inDevAttr->config.sample_rate);
            is_ds_required = true;
        } else if ((activeDevAttr->config.sample_rate < inDevAttr->config.sample_rate) ||
            (activeDevAttr->config.bit_width < inDevAttr->config.bit_width) ||
            (activeDevAttr->config.ch_info.channels < inDevAttr->config.ch_info.channels)) {
            is_ds_required = true;
        }
        break;
    case QAL_DEVICE_OUT_WIRED_HEADSET:
    case QAL_DEVICE_OUT_WIRED_HEADPHONE:
        if ((QAL_STREAM_VOICE_CALL == inStrAttr->type) && ((activeDevAttr->config.sample_rate != inDevAttr->config.sample_rate) ||
            (activeDevAttr->config.bit_width != inDevAttr->config.bit_width) ||
            (activeDevAttr->config.ch_info.channels != inDevAttr->config.ch_info.channels))) {
            is_ds_required = true;
        } else if ((QAL_STREAM_COMPRESSED == inStrAttr->type || QAL_STREAM_PCM_OFFLOAD == inStrAttr->type) &&
            (NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_DSP == getNativeAudioSupport()) &&
            (QAL_AUDIO_OUTPUT == inStrAttr->direction) &&
            (inStrAttr->out_media_config.sample_rate % SAMPLINGRATE_44K == 0)) {

            //Native Audio usecase
            if (activeDevAttr->config.sample_rate != inStrAttr->out_media_config.sample_rate) {
                inDevAttr->config.sample_rate = inStrAttr->out_media_config.sample_rate;
                is_ds_required = true;
            }
        } else if ((activeDevAttr->config.sample_rate < inDevAttr->config.sample_rate) ||
            (activeDevAttr->config.bit_width < inDevAttr->config.bit_width) ||
            (activeDevAttr->config.ch_info.channels < inDevAttr->config.ch_info.channels)) {
            is_ds_required = true;
        }
        break;
    default:
        is_ds_required = false;
        break;
    }

    return is_ds_required;
}

int32_t ResourceManager::streamDevDisconnect(std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnectList){
    int status = 0;
    std::vector <std::tuple<Stream *, uint32_t>>::iterator sIter;

    /* disconnect active list from the current devices they are attached to */
    for (sIter = streamDevDisconnectList.begin(); sIter != streamDevDisconnectList.end(); sIter++) {
        status = (std::get<0>(*sIter))->disconnectStreamDevice(std::get<0>(*sIter), (qal_device_id_t)std::get<1>(*sIter));
        if (status) {
            QAL_ERR(LOG_TAG, "failed to disconnect stream %pK from device %d",
                    std::get<0>(*sIter), std::get<1>(*sIter));
            goto error;
        } else {
            QAL_DBG(LOG_TAG, "disconnect stream %pK from device %d",
                    std::get<0>(*sIter), std::get<1>(*sIter));
        }
    }
error:
    return status;
}

int32_t ResourceManager::streamDevConnect(std::vector <std::tuple<Stream *, struct qal_device *>> streamDevConnectList){
    int status = 0;
    std::vector <std::tuple<Stream *, struct qal_device *>>::iterator sIter;

    /* connect active list from the current devices they are attached to */
    for (sIter = streamDevConnectList.begin(); sIter != streamDevConnectList.end(); sIter++) {
        status = std::get<0>(*sIter)->connectStreamDevice(std::get<0>(*sIter), std::get<1>(*sIter));
        if (status) {
            QAL_ERR(LOG_TAG,"failed to connect stream %pK from device %d",
                    std::get<0>(*sIter), (std::get<1>(*sIter))->id);
            goto error;
        } else {
            QAL_DBG(LOG_TAG,"connected stream %pK from device %d",
                    std::get<0>(*sIter), (std::get<1>(*sIter))->id);
        }
    }
error:
    return status;
}


int32_t ResourceManager::streamDevSwitch(std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnectList,
                                         std::vector <std::tuple<Stream *, struct qal_device *>> streamDevConnectList)
{
    int status = 0;
    status = streamDevDisconnect(streamDevDisconnectList);
    if (status) {
        QAL_ERR(LOG_TAG,"disconnect failed");
        goto error;
    }
    status = streamDevConnect(streamDevConnectList);
    if (status) {
        QAL_ERR(LOG_TAG,"Connect failed");
    }
error:
    return status;
}

//when returning from this function, the device config will be updated with
//the device config of the highest priority stream

//TBD: manage re-routing of existing lower priority streams if incoming
//stream is a higher priority stream. Priority defined in ResourceManager.h
//(details below)
bool ResourceManager::updateDeviceConfig(std::shared_ptr<Device> inDev,
           struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr)
{
    bool isDeviceSwitch = false;
    bool isVoiceCall = false;
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct qal_device dattr;
    std::vector<Stream*>::iterator sIter;
    std::vector<std::shared_ptr<Device>>::iterator dIter;
    std::shared_ptr<Device> inDevice = nullptr;
    qal_stream_type_t streamType;
    std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream *, struct qal_device *>> StreamDevConnect;
    std::vector <std::tuple<Stream *, uint32_t>>::iterator disIter;

    if (!inDev || !inDevAttr) {
        goto error;
    }

    //get the active streams on the device
    //if higher priority stream exists on any of the incoming device, update the config of incoming device
    //based on device config of higher priority stream

    //TBD: if incoming stream is a higher priority,
    //call callback into all streams
    //for all devices matching incoming device id
    //and route the lower priority to new device (disable session, disable device, enable session, enable device
    //return from callback

    //check if there are shared backends
    // if yes add them to streams to device switch
    getSharedBEActiveStreamDevs(streamDevDisconnect, inDevAttr->id);
    if (streamDevDisconnect.size() > 0) {
        //add the shared backends to the connect list
        for(disIter = streamDevDisconnect.begin(); disIter != streamDevDisconnect.end(); disIter++)
            StreamDevConnect.push_back({std::get<0>(*disIter),inDevAttr});
        QAL_ERR(LOG_TAG, "shared BE found device switch will be needed");
    }

    getActiveStream_l(inDev, activeStreams);
    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no other active streams found so update device cfg");
        inDev->setDeviceAttributes(*inDevAttr);
        goto error;
    }

    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        status = (*sIter)->getStreamType(&streamType);
        if (QAL_STREAM_VOICE_CALL == streamType) {
            /* overwrite in attr with current device config of voice call */
            status = inDev->getDeviceAttributes(inDevAttr);
            QAL_DBG(LOG_TAG,"voice active updating attributes to voice");
            goto error;
        }
    }


    // All the activesteams using device A should use same device config so no need
    // to run through on all activestreams for device A
    for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        status = (*sIter)->getAssociatedDevices(associatedDevices);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"getAssociatedDevices Failed");
            goto error;
        }

        for (dIter = associatedDevices.begin();
            dIter != associatedDevices.end(); dIter++) {
            status = (*dIter)->getDeviceAttributes(&dattr);
            if(0 != status) {
                QAL_ERR(LOG_TAG,"getDeviceAttributes Failed");
                goto error;
            }

            if (dattr.id == inDevAttr->id) {
                isDeviceSwitch = isDeviceSwitchRequired(&dattr, inDevAttr, inStrAttr);
                if (isDeviceSwitch) {
                    streamDevDisconnect.push_back({*sIter,inDevAttr->id});
                    StreamDevConnect.push_back({*sIter,inDevAttr});
                } else {
                    // case 2. If incoming device config has lower priority then update incoming
                    //  device config with currently running device config
                    QAL_ERR(LOG_TAG, "%s: device %d is already running with higher priority device config",
                            __func__, inDevAttr->id);
                    memcpy(inDevAttr, (void*)&dattr, sizeof(struct qal_device));
                }
            }
        }
    }

error:
    //if device switch is need perform it
    if (streamDevDisconnect.size()) {
        status = streamDevSwitch(streamDevDisconnect, StreamDevConnect);
        if (status) {
            QAL_ERR(LOG_TAG,"deviceswitch failed with %d", status);
        }
    }
    return isDeviceSwitch;
}

int32_t ResourceManager::forceDeviceSwitch(std::shared_ptr<Device> inDev,
                                              struct qal_device *newDevAttr)
{
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnect;
    std::vector <std::tuple<Stream *, struct qal_device *>> StreamDevConnect;
    std::vector<Stream*>::iterator sIter;

    if (!inDev || !newDevAttr) {
        return -EINVAL;
    }

    //get the active streams on the device
    getActiveStream_l(inDev, activeStreams);
    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no other active streams found");
        goto done;
    }
    //created dev switch vectors

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        streamDevDisconnect.push_back({(*sIter), inDev->getSndDeviceId()});
        StreamDevConnect.push_back({(*sIter), newDevAttr});
    }
    status = streamDevSwitch(streamDevDisconnect, StreamDevConnect);
    if (status) {
         QAL_ERR(LOG_TAG, "forceDeviceSwitch failed %d", status);
    }

done:
    return 0;
}

const std::string ResourceManager::getQALDeviceName(const qal_device_id_t id) const
{
    QAL_DBG(LOG_TAG, "%s: id %d", __func__, id);
    if (isValidDevId(id)) {
        return deviceNameLUT.at(id);
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", id);
        return std::string("");
    }
}

int ResourceManager::getBackendName(int deviceId, std::string &backendName)
{
    if (isValidDevId(deviceId) && (deviceId != QAL_DEVICE_NONE)) {
        backendName.assign(listAllBackEndIds[deviceId].second);
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
        return -EINVAL;
    }
    return 0;
}

bool ResourceManager::isValidDevId(int deviceId)
{
    if (((deviceId >= QAL_DEVICE_NONE) && (deviceId < QAL_DEVICE_OUT_MAX))
        || ((deviceId > QAL_DEVICE_IN_MIN) && (deviceId < QAL_DEVICE_IN_MAX)))
        return true;

    return false;
}

bool ResourceManager::isOutputDevId(int deviceId)
{
    if ((deviceId > QAL_DEVICE_NONE) && (deviceId < QAL_DEVICE_OUT_MAX))
        return true;

    return false;
}

bool ResourceManager::isInputDevId(int deviceId)
{
    if ((deviceId > QAL_DEVICE_IN_MIN) && (deviceId < QAL_DEVICE_IN_MAX))
        return true;

    return false;
}

bool ResourceManager::matchDevDir(int devId1, int devId2)
{
    if (isOutputDevId(devId1) && isOutputDevId(devId2))
        return true;
    if (isInputDevId(devId1) && isInputDevId(devId2))
        return true;

    return false;
}

bool ResourceManager::isNonALSACodec(const struct qal_device * /*device*/) const
{

    //return false on our target, move configuration to xml

    return false;
}

bool ResourceManager::ifVoiceorVoipCall (const qal_stream_type_t streamType) const {

   bool voiceOrVoipCall = false;

   switch (streamType) {
       case QAL_STREAM_VOIP:
       case QAL_STREAM_VOIP_RX:
       case QAL_STREAM_VOIP_TX:
       case QAL_STREAM_VOICE_CALL_RX:
       case QAL_STREAM_VOICE_CALL_TX:
       case QAL_STREAM_VOICE_CALL_RX_TX:
       case QAL_STREAM_VOICE_CALL:
           voiceOrVoipCall = true;
           break;
       default:
           voiceOrVoipCall = false;
           break;
    }

    return voiceOrVoipCall;
}

int ResourceManager::getCallPriority(bool ifVoiceCall) const {

//TBD: replace this with XML based priorities
    if (ifVoiceCall) {
        return 100;
    } else {
        return 0;
    }
}

int ResourceManager::getStreamAttrPriority (const qal_stream_attributes* sAttr) const {
    int priority = 0;

    if (!sAttr)
        goto exit;


    priority = getCallPriority(ifVoiceorVoipCall(sAttr->type));


    //44.1 or multiple or 24 bit

    if ((sAttr->in_media_config.sample_rate % 44100) == 0) {
        priority += 50;
    }

    if (sAttr->in_media_config.bit_width == 24) {
        priority += 25;
    }

exit:
    return priority;
}

int ResourceManager::getNativeAudioSupport()
{
    int ret = NATIVE_AUDIO_MODE_INVALID;
    if (na_props.rm_na_prop_enabled &&
        na_props.ui_na_prop_enabled) {
        ret = na_props.na_mode;
    }
    QAL_ERR(LOG_TAG,"napb: ui Prop enabled(%d) mode(%d)",
           na_props.ui_na_prop_enabled, na_props.na_mode);
    return ret;
}

int ResourceManager::setNativeAudioSupport(int na_mode)
{
    if (NATIVE_AUDIO_MODE_SRC == na_mode || NATIVE_AUDIO_MODE_TRUE_44_1 == na_mode
        || NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_CODEC == na_mode
        || NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_DSP == na_mode) {
        na_props.rm_na_prop_enabled = na_props.ui_na_prop_enabled = true;
        na_props.na_mode = na_mode;
        QAL_ERR(LOG_TAG,"napb: native audio playback enabled in (%s) mode",
              ((na_mode == NATIVE_AUDIO_MODE_SRC)?"SRC":
               (na_mode == NATIVE_AUDIO_MODE_TRUE_44_1)?"True":
               (na_mode == NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_CODEC)?"Multiple_Mix_Codec":"Multiple_Mix_DSP"));
    }
    else {
        na_props.rm_na_prop_enabled = false;
        na_props.na_mode = NATIVE_AUDIO_MODE_INVALID;
        QAL_VERBOSE(LOG_TAG,"napb: native audio playback disabled");
    }

    return 0;
}

void ResourceManager::getNativeAudioParams(struct str_parms *query,
                             struct str_parms *reply,
                             char *value, int len)
{
    int ret;
    ret = str_parms_get_str(query, AUDIO_PARAMETER_KEY_NATIVE_AUDIO,
                            value, len);
    if (ret >= 0) {
        if (na_props.rm_na_prop_enabled) {
            str_parms_add_str(reply, AUDIO_PARAMETER_KEY_NATIVE_AUDIO,
                          na_props.ui_na_prop_enabled ? "true" : "false");
            QAL_VERBOSE(LOG_TAG,"napb: na_props.ui_na_prop_enabled: %d",
                  na_props.ui_na_prop_enabled);
        } else {
            str_parms_add_str(reply, AUDIO_PARAMETER_KEY_NATIVE_AUDIO,
                              "false");
            QAL_VERBOSE(LOG_TAG,"napb: native audio not supported: %d",
                  na_props.rm_na_prop_enabled);
        }
    }
}

int ResourceManager::setConfigParams(struct str_parms *parms)
{
    char *value=NULL;
    int len;
    int ret = 0;
    char *kv_pairs = str_parms_to_str(parms);

    if(kv_pairs == NULL) {
        ret = -ENOMEM;
        QAL_ERR(LOG_TAG," key-value pair is NULL");
        goto done;
    }

    QAL_ERR(LOG_TAG," enter: %s", kv_pairs);

    len = strlen(kv_pairs);
    value = (char*)calloc(len, sizeof(char));
    if(value == NULL) {
        ret = -ENOMEM;
        QAL_ERR(LOG_TAG,"failed to allocate memory");
        goto done;
    }
    ret = setNativeAudioParams(parms, value, len);
done:
    QAL_VERBOSE(LOG_TAG," exit with code(%d)", ret);
    if(value != NULL)
        free(value);
    return ret;
}


int ResourceManager::setNativeAudioParams(struct str_parms *parms,
                                          char *value, int len)
{
    int ret = -EINVAL;
    int mode = NATIVE_AUDIO_MODE_INVALID;

    if (!value || !parms)
        return ret;

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_MAX_SESSIONS,
                                value, len);
    if (ret >= 0) {
        max_session_num = std::stoi(value);
        QAL_INFO(LOG_TAG, "Max sessions supported for each stream type are %d",
                 max_session_num);

    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_NATIVE_AUDIO_MODE,
                             value, len);
    QAL_VERBOSE(LOG_TAG," value %s", value);
    if (ret >= 0) {
        if (value && !strncmp(value, "src", sizeof("src")))
            mode = NATIVE_AUDIO_MODE_SRC;
        else if (value && !strncmp(value, "true", sizeof("true")))
            mode = NATIVE_AUDIO_MODE_TRUE_44_1;
        else if (value && !strncmp(value, "multiple_mix_codec", sizeof("multiple_mix_codec")))
            mode = NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_CODEC;
        else if (value && !strncmp(value, "multiple_mix_dsp", sizeof("multiple_mix_dsp")))
            mode = NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_DSP;
        else {
            mode = NATIVE_AUDIO_MODE_INVALID;
            QAL_ERR(LOG_TAG,"napb:native_audio_mode in RM xml,invalid mode(%s) string", value);
        }
        QAL_VERBOSE(LOG_TAG,"napb: updating mode (%d) from XML", mode);
        setNativeAudioSupport(mode);
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_NATIVE_AUDIO,
                             value, len);
    QAL_VERBOSE(LOG_TAG," value %s", value);
    if (ret >= 0) {
        if (na_props.rm_na_prop_enabled) {
            if (!strncmp("true", value, sizeof("true"))) {
                na_props.ui_na_prop_enabled = true;
                QAL_VERBOSE(LOG_TAG,"napb: native audio feature enabled from UI");
            } else {
                na_props.ui_na_prop_enabled = false;
                QAL_VERBOSE(LOG_TAG,"napb: native audio feature disabled from UI");
            }

            str_parms_del(parms, AUDIO_PARAMETER_KEY_NATIVE_AUDIO);
            //TO-DO
            // Update the concurrencies
        } else {
              QAL_VERBOSE(LOG_TAG,"napb: native audio cannot be enabled from UI");
        }
    }
    return ret;
}
void ResourceManager::updatePcmId(int32_t deviceId, int32_t pcmId)
{
    if (isValidDevId(deviceId)) {
        devicePcmId[deviceId].second = pcmId;
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
    }
}

void ResourceManager::updateLinkName(int32_t deviceId, std::string linkName)
{
    if (isValidDevId(deviceId)) {
        deviceLinkName[deviceId].second = linkName;
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
    }
}

void ResourceManager::updateSndName(int32_t deviceId, std::string sndName)
{
    if (isValidDevId(deviceId)) {
        sndDeviceNameLUT[deviceId].second = sndName;
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
    }
}

void ResourceManager::updateBackEndName(int32_t deviceId, std::string backEndName)
{
    if (isValidDevId(deviceId)) {
        listAllBackEndIds[deviceId].second = backEndName;
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
    }
}

int convertCharToHex(std::string num)
{
    uint64_t hexNum = 0;
    uint32_t base = 1;
    const char * charNum = num.c_str();
    int32_t len = strlen(charNum);
    for (int i = len-1; i>=2; i--) {
        if (charNum[i] >= '0' && charNum[i] <= '9') {
            hexNum += (charNum[i] - 48) * base;
            base = base << 4;
        } else if (charNum[i] >= 'A' && charNum[i] <= 'F') {
            hexNum += (charNum[i] - 55) * base;
            base = base << 4;
        } else if (charNum[i] >= 'a' && charNum[i] <= 'f') {
            hexNum += (charNum[i] - 87) * base;
            base = base << 4;
        }
    }
    return (int32_t) hexNum;
}

void ResourceManager::updateStreamTag(int32_t tagId)
{
    streamTag.push_back(tagId);
}

void ResourceManager::updateDeviceTag(int32_t tagId)
{
    deviceTag.push_back(tagId);
}

// must be called with mResourceManagerMutex held
int32_t ResourceManager::a2dpSuspend()
{
    std::shared_ptr<Device> dev = nullptr;
    struct qal_device dattr;
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector<Stream*>::iterator sIter;
    struct qal_device_info devinfo = {};

    dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
    dev = Device::getInstance(&dattr , rm);

    getActiveStream_l(dev, activeStreams);

    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no active streams found");
        goto exit;
    }

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        int ret = 0;
        struct qal_stream_attributes sAttr = {};

        ret = (*sIter)->getStreamAttributes(&sAttr);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "getStreamType failed with status = %d", ret);
            goto exit;
        }
        if (sAttr.type == QAL_STREAM_COMPRESSED) {
            if (!((*sIter)->a2dp_compress_mute)) {
                struct qal_device speakerDattr;

                QAL_DBG(LOG_TAG, "%s: selecting speaker and muting stream", __func__);
                (*sIter)->pause(); // compress_pause
                (*sIter)->setMute(true); // mute the stream, unmute during a2dp_resume
                (*sIter)->a2dp_compress_mute = true;
                // force switch to speaker
                speakerDattr.id = QAL_DEVICE_OUT_SPEAKER;

                getDeviceInfo(speakerDattr.id, sAttr.type, &devinfo);
                if ((devinfo.channels == 0) ||
                       (devinfo.channels > devinfo.max_channels)) {
                    status = -EINVAL;
                    QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                    goto exit;
                }
                getDeviceConfig(&speakerDattr, &sAttr, devinfo.channels);
                mResourceManagerMutex.unlock();
                forceDeviceSwitch(dev, &speakerDattr);
                mResourceManagerMutex.lock();
                (*sIter)->resume(); //compress_resume
                /* backup actual device name in stream class */
                (*sIter)->suspendedDevId = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            }
        } else {
            // put to standby for non offload usecase
            mResourceManagerMutex.unlock();
            (*sIter)->standby();
            mResourceManagerMutex.lock();
        }
    }

exit:
    return status;
}

// must be called with mResourceManagerMutex held
int32_t ResourceManager::a2dpResume()
{
    std::shared_ptr<Device> dev = nullptr;
    struct qal_device dattr;
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector<Stream*>::iterator sIter;
    struct qal_stream_attributes sAttr;

    dattr.id = QAL_DEVICE_OUT_SPEAKER;
    dev = Device::getInstance(&dattr , rm);

    getActiveStream_l(dev, activeStreams);

    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no active streams found");
        goto exit;
    }

    // check all active stream associated with speaker
    // if the stream actual device is a2dp, then switch back to a2dp
    // unmute the stream
    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        int ret = 0;
        struct qal_device_info devinfo = {};

        status = (*sIter)->getStreamAttributes(&sAttr);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
            goto exit;
        }
        if (sAttr.type == QAL_STREAM_COMPRESSED &&
            ((*sIter)->suspendedDevId == QAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
            struct qal_device a2dpDattr;

            a2dpDattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            QAL_DBG(LOG_TAG, "%s: restoring A2dp and unmuting stream", __func__);

            getDeviceInfo(a2dpDattr.id, sAttr.type, &devinfo);
            if ((devinfo.channels == 0) ||
                   (devinfo.channels > devinfo.max_channels)) {
                status = -EINVAL;
                QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                goto exit;
            }

            getDeviceConfig(&a2dpDattr, &sAttr, devinfo.channels);
            mResourceManagerMutex.unlock();
            forceDeviceSwitch(dev, &a2dpDattr);
            mResourceManagerMutex.lock();
            (*sIter)->suspendedDevId = QAL_DEVICE_NONE;
            if ((*sIter)->a2dp_compress_mute) {
                (*sIter)->setMute(false);
                (*sIter)->a2dp_compress_mute = false;
            }
        }
    }
exit:
    return status;
}

int ResourceManager::getParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size, void *query)
{
    int status = 0;

    QAL_INFO(LOG_TAG, "%s param_id=%d", __func__, param_id);
    mResourceManagerMutex.lock();
    switch (param_id) {
        case QAL_PARAM_ID_UIEFFECT:
        {
#if 0
            gef_payload_t *gef_payload = (gef_payload_t *)query;
            int index = 0;
            int qal_device_id = 0;
            int stream_type = 0;
            bool match = false;
            std::vector<Stream*>::iterator sIter;
            for(sIter = mActiveStreams.begin(); sIter != mActiveStreams.end(); sIter++) {
                match = (*sIter)->isGKVMatch(gef_payload->graph);
                if (match) {
                    qal_param_payload *qal_payload;
                    qal_payload.payload = (uint8_t *)&gef_payload->data;
                    status = (*sIter)->getEffectParameters((void *)&qal_payload, payload_size);
                    break;
                }
            }
#endif
            break;
        }
        case QAL_PARAM_ID_BT_A2DP_RECONFIG_SUPPORTED:
        case QAL_PARAM_ID_BT_A2DP_SUSPENDED:
        case QAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;
            qal_param_bta2dp_t *param_bt_a2dp = nullptr;

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr , rm);
                status = dev->getDeviceParameter(param_id, (void **)&param_bt_a2dp);
                if (status) {
                    QAL_ERR(LOG_TAG, "get Parameter %d failed\n", param_id);
                    goto exit;
                }
                *param_payload = param_bt_a2dp;
                *payload_size = sizeof(qal_param_bta2dp_t);
            }
            break;
        }
        case QAL_PARAM_ID_DEVICE_CAPABILITY:
        {
            qal_param_device_capability_t *param_device_capability = (qal_param_device_capability_t *)(*param_payload);
            QAL_INFO(LOG_TAG, "Device %d card = %d qalid=%x",
                        param_device_capability->addr.device_num,
                        param_device_capability->addr.card_id,
                        param_device_capability->id);
            status = getDeviceDefaultCapability(*param_device_capability);
            break;
        }
        case QAL_PARAM_ID_GET_SOUND_TRIGGER_PROPERTIES:
        {
            QAL_INFO(LOG_TAG, "get sound trigge properties, status %d", status);
            struct qal_st_properties *qstp =
                (struct qal_st_properties *)calloc(1, sizeof(struct qal_st_properties));

            GetVoiceUIProperties(qstp);

            *param_payload = qstp;
            *payload_size = sizeof(qal_st_properties);
            break;
        }
        default:
            status = -EINVAL;
            QAL_ERR(LOG_TAG, "Unknown ParamID:%d", param_id);
            break;
    }
exit:
    mResourceManagerMutex.unlock();
    return status;
}

int ResourceManager::setParameter(uint32_t param_id, void *param_payload,
                                  size_t payload_size)
{
    int status = 0;

    QAL_INFO(LOG_TAG, "%s param_id=%d", __func__, param_id);

    mResourceManagerMutex.lock();
    switch (param_id) {
        case QAL_PARAM_ID_SCREEN_STATE:
        {
            qal_param_screen_state_t* param_screen_st = (qal_param_screen_state_t*) param_payload;
            QAL_INFO(LOG_TAG, "Screen State:%d", param_screen_st->screen_state);
            if (payload_size == sizeof(qal_param_screen_state_t)) {
                status = handleScreenStatusChange(*param_screen_st);
            } else {
                QAL_ERR(LOG_TAG,"Incorrect size : expected (%d), received(%d)",
                        sizeof(qal_param_screen_state_t), payload_size);
                status = -EINVAL;
            }
        }
        break;
        case QAL_PARAM_ID_DEVICE_ROTATION:
        {
            qal_param_device_rotation_t* param_device_rot =
                                   (qal_param_device_rotation_t*) param_payload;
            QAL_INFO(LOG_TAG, "Device Rotation :%d", param_device_rot->rotation_type);
            if (payload_size == sizeof(qal_param_device_rotation_t)) {
                status = handleDeviceRotationChange(*param_device_rot);
            } else {
                QAL_ERR(LOG_TAG,"Incorrect size : expected (%d), received(%d)",
                        sizeof(qal_param_device_rotation_t), payload_size);
                status = -EINVAL;
            }

        }
        break;
        case QAL_PARAM_ID_DEVICE_CONNECTION:
        {
            qal_param_device_connection_t *device_connection =
                (qal_param_device_connection_t *)param_payload;
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;

            QAL_INFO(LOG_TAG, "Device %d connected = %d",
                        device_connection->id,
                        device_connection->connection_state);
            if (payload_size == sizeof(qal_param_device_connection_t)) {
                status = handleDeviceConnectionChange(*device_connection);
                if (!status && (device_connection->id == QAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
                    dattr.id = device_connection->id;
                    dev = Device::getInstance(&dattr, rm);
                    status = dev->setDeviceParameter(param_id, param_payload);
                } else {
                    status = SwitchSVADevices(
                        device_connection->connection_state,
                        device_connection->id);
                    if (status) {
                        QAL_ERR(LOG_TAG, "Failed to switch device for SVA");
                    }
                }
            } else {
                QAL_ERR(LOG_TAG,"Incorrect size : expected (%d), received(%d)",
                      sizeof(qal_param_device_connection_t), payload_size);
                status = -EINVAL;
            }
        }
        break;
        case QAL_PARAM_ID_CHARGING_STATE:
        {
            qal_param_charging_state *battery_charging_state =
                (qal_param_charging_state *)param_payload;

            if (IsTransitToNonLPIOnChargingSupported()) {
                if (payload_size == sizeof(qal_param_charging_state)) {
                    QAL_INFO(LOG_TAG, "Charging State = %d",
                              battery_charging_state->charging_state);
                    charging_state_ = battery_charging_state->charging_state;
                    for (int i = 0; i < active_streams_st.size(); i++) {
                        status = active_streams_st[i]->UpdateChargingState(
                            battery_charging_state->charging_state);
                        if (status) {
                            QAL_ERR(LOG_TAG,
                                    "Failed to handling charging state\n");
                        }
                    }
                } else {
                    QAL_ERR(LOG_TAG,
                            "Incorrect size : expected (%d), received(%d)",
                            sizeof(qal_param_charging_state), payload_size);
                    status = -EINVAL;
                }
            } else {
                QAL_DBG(LOG_TAG,
                          "transit_to_non_lpi_on_charging set to false\n");
            }
        }
        break;
        case QAL_PARAM_ID_BT_SCO_WB:
        case QAL_PARAM_ID_BT_SCO_SWB:
        case QAL_PARAM_ID_BT_SCO:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_SCO;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr, rm);
                status = dev->setDeviceParameter(param_id, param_payload);
            }

            dattr.id = QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr, rm);
                status = dev->setDeviceParameter(param_id, param_payload);
            }
        }
        break;
        case QAL_PARAM_ID_BT_A2DP_RECONFIG:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;
            qal_param_bta2dp_t *param_bt_a2dp = nullptr;
            struct qal_device_info devinfo = {};

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr, rm);
                status = dev->setDeviceParameter(param_id, param_payload);
                if (status) {
                    QAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                    goto exit;
                }
                status = dev->getDeviceParameter(param_id, (void **)&param_bt_a2dp);
                if (status) {
                    QAL_ERR(LOG_TAG, "get Parameter %d failed\n", param_id);
                    goto exit;
                }
                if (param_bt_a2dp->reconfigured == true) {
                    struct qal_device spkrDattr;
                    std::shared_ptr<Device> spkrDev = nullptr;
                    struct qal_stream_attributes sAttr;
                    struct qal_device_info devinfo = {};

                    QAL_DBG(LOG_TAG, "Switching A2DP Device\n");
                    spkrDattr.id = QAL_DEVICE_OUT_SPEAKER;
                    spkrDev = Device::getInstance(&spkrDattr, rm);

                    /* Num channels for Rx devices is same for all usecases,
                       stream type is irrelevant here. */
                    getDeviceInfo(spkrDattr.id, QAL_STREAM_LOW_LATENCY, &devinfo);
                    if ((devinfo.channels == 0) ||
                          (devinfo.channels > devinfo.max_channels)) {
                        status = -EINVAL;
                        QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                        goto exit;
                    }

                    getDeviceConfig(&spkrDattr, NULL, devinfo.channels);
                    getDeviceInfo(dattr.id, QAL_STREAM_LOW_LATENCY, &devinfo);
                    if ((devinfo.channels == 0) ||
                          (devinfo.channels > devinfo.max_channels)) {
                        status = -EINVAL;
                        QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                        goto exit;
                    }
                    getDeviceConfig(&dattr, NULL, devinfo.channels);

                    mResourceManagerMutex.unlock();
                    forceDeviceSwitch(dev, &spkrDattr);
                    forceDeviceSwitch(spkrDev, &dattr);
                    mResourceManagerMutex.lock();
                    param_bt_a2dp->reconfigured = false;
                    dev->setDeviceParameter(param_id, param_bt_a2dp);
                }
            }
        }
        break;
        case QAL_PARAM_ID_BT_A2DP_SUSPENDED:
        {
            std::shared_ptr<Device> a2dp_dev = nullptr;
            struct qal_device a2dp_dattr;
            qal_param_bta2dp_t *current_param_bt_a2dp = nullptr;
            qal_param_bta2dp_t *param_bt_a2dp = nullptr;

            a2dp_dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            if (!isDeviceAvailable(a2dp_dattr.id)) {
                QAL_ERR(LOG_TAG, "%s device %d is inactive, set param %d failed\n",
                                  __func__, a2dp_dattr.id,  param_id);
                status = -EIO;
                goto exit;
            }

            param_bt_a2dp = (qal_param_bta2dp_t *)param_payload;
            a2dp_dev = Device::getInstance(&a2dp_dattr , rm);
            status = a2dp_dev->getDeviceParameter(param_id, (void **)&current_param_bt_a2dp);
            if (current_param_bt_a2dp->a2dp_suspended == param_bt_a2dp->a2dp_suspended) {
                QAL_INFO(LOG_TAG, "A2DP already in requested state, ignoring\n");
                goto exit;
            }

            if (param_bt_a2dp->a2dp_suspended == false) {
                /* Handle bt sco mic running usecase */
                struct qal_device sco_tx_dattr;
                struct qal_device_info devinfo = {};
                struct qal_stream_attributes sAttr;
                Stream *stream = NULL;
                std::vector<Stream*> activestreams;

                sco_tx_dattr.id = QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                QAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco mic to handset mic");
                if (isDeviceAvailable(sco_tx_dattr.id)) {
                    struct qal_device handset_tx_dattr;
                    std::shared_ptr<Device> sco_tx_dev = nullptr;

                    handset_tx_dattr.id = QAL_DEVICE_IN_HANDSET_MIC;
                    sco_tx_dev = Device::getInstance(&sco_tx_dattr , rm);
                    getActiveStream_l(sco_tx_dev, activestreams);
                    if (activestreams.size() == 0) {
                       QAL_ERR(LOG_TAG, "no other active streams found");
                       goto setdevparam;
                    }
                    stream = static_cast<Stream *>(activestreams[0]);
                    stream->getStreamAttributes(&sAttr);
                    getDeviceInfo(handset_tx_dattr.id, sAttr.type, &devinfo);
                    QAL_DBG(LOG_TAG, "devinfo.channels %d sAttr.type %d \n", devinfo.channels, sAttr.type);
                    getDeviceConfig(&handset_tx_dattr, &sAttr, devinfo.channels);
                    mResourceManagerMutex.unlock();
                    rm->forceDeviceSwitch(sco_tx_dev, &handset_tx_dattr);
                    mResourceManagerMutex.lock();
                }
                /* TODO : Handle other things in BT class */
            }
setdevparam:
            status = a2dp_dev->setDeviceParameter(param_id, param_payload);
            if (status) {
                QAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                goto exit;
            }
        }
        break;
        case QAL_PARAM_ID_BT_A2DP_TWS_CONFIG:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr, rm);
                status = dev->setDeviceParameter(param_id, param_payload);
                if (status) {
                    QAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                    goto exit;
                }
            }
        }
        break;
        case QAL_PARAM_ID_UIEFFECT:
        {
#if 0
            gef_payload_t *gef_payload = (gef_payload_t*)param_payload;
            int index = 0;
            int qal_device_id = 0;
            int stream_type = 0;
            bool match = false;
            std::vector<Stream*>::iterator sIter;
            for(sIter = mActiveStreams.begin(); sIter != mActiveStreams.end(); sIter++) {
                match = (*sIter)->isGKVMatch(gef_payload->graph);
                if (match) {
                    qal_param_payload qal_payload;
                    qal_payload.payload = (uint8_t *)&gef_payload->data;
                    status = (*sIter)->setParameters(param_id, (void *)&qal_payload);
                }
            }
#endif
        }
        break;
        default:
            QAL_ERR(LOG_TAG, "Unknown ParamID:%d", param_id);
            break;
    }

exit:
    mResourceManagerMutex.unlock();
    return status;
}

int ResourceManager::handleScreenStatusChange(qal_param_screen_state_t screen_state)
{
    int status = 0;

    if (screen_state_ != screen_state.screen_state) {
        if (screen_state.screen_state == false) {
            /* have appropriate streams transition to LPI */
            QAL_VERBOSE(LOG_TAG, "Screen State printout");
        }
        else {
            /* have appropriate streams transition out of LPI */
            QAL_ERR(LOG_TAG, "Screen State printout");
        }
        screen_state_ = screen_state.screen_state;
        /* update
         * for (typename std::vector<StreamSoundTrigger*>::iterator iter = active_streams_st.begin();
         *    iter != active_streams_st.end(); iter++) {
         *   status = (*iter)->handleScreenState(screen_state_);
         *  }
         */
    }
    return status;
}

int ResourceManager::handleDeviceRotationChange (qal_param_device_rotation_t
                                                         rotation_type) {
    std::vector<Stream*>::iterator sIter;
    qal_stream_type_t streamType;
    struct qal_device dattr;
    int status = 0;
    QAL_INFO(LOG_TAG, "Device Rotation Changed %d", rotation_type.rotation_type);
    rotation_type_ = rotation_type.rotation_type;

    /**Get the active device list and check if speaker is present.
     */
    for (int i = 0; i < active_devices.size(); i++) {
        int deviceId = active_devices[i]->getSndDeviceId();
        status = active_devices[i]->getDeviceAttributes(&dattr);
        if(0 != status) {
           QAL_ERR(LOG_TAG,"getDeviceAttributes Failed");
           goto error;
        }
        QAL_INFO(LOG_TAG, "Device Got %d with channel %d",deviceId,
                                                 dattr.config.ch_info.channels);
        if ((QAL_DEVICE_OUT_SPEAKER == deviceId) &&
            (2 == dattr.config.ch_info.channels)) {

            QAL_INFO(LOG_TAG, "Device is Stereo Speaker");
            std::vector <Stream *> activeStreams;
            getActiveStream_l(active_devices[i], activeStreams);
            for (sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
                status = (*sIter)->getStreamType(&streamType);
                if(0 != status) {
                   QAL_ERR(LOG_TAG,"setParameters Failed");
                   goto error;
                }
                /** Check for the Streams which can require Stereo speaker functionality.
                 * Mainly these will need :
                 * 1. Deep Buffer
                 * 2. PCM offload
                 * 3. Compressed
                 */
                if ((QAL_STREAM_DEEP_BUFFER == streamType) ||
                    (QAL_STREAM_COMPRESSED == streamType) ||
                    (QAL_STREAM_PCM_OFFLOAD == streamType)) {

                    QAL_INFO(LOG_TAG, "Rotation for stream %d", streamType);
                    // Need to set the rotation now.
                    status = (*sIter)->setParameters(QAL_PARAM_ID_DEVICE_ROTATION,
                                                     (void*)&rotation_type);
                    if(0 != status) {
                       QAL_ERR(LOG_TAG,"setParameters Failed");
                       goto error;
                    }
                }
            }
            //As we got the speaker and it is reversed. No need to further
            // iterate the list.
            break;
        }
    }
error :
    QAL_INFO(LOG_TAG, "Exiting handleDeviceRotationChange");
    return status;
}

bool ResourceManager::getScreenState()
{
    return screen_state_;
}

qal_speaker_rotation_type ResourceManager::getCurrentRotationType()
{
    return rotation_type_;
}

int ResourceManager::getDeviceDefaultCapability(qal_param_device_capability_t capability) {
    int status = 0;
    qal_device_id_t device_qal_id = capability.id;
    bool device_available = isDeviceAvailable(device_qal_id);

    struct qal_device conn_device;
    std::shared_ptr<Device> dev = nullptr;
    std::shared_ptr<Device> candidate_device;

    memset(&conn_device, 0, sizeof(struct qal_device));
    conn_device.id = device_qal_id;
    QAL_DBG(LOG_TAG, "device qal id=%x available=%x", device_qal_id, device_available);
    dev = Device::getInstance(&conn_device, rm);
    if (dev)
        status = dev->getDefaultConfig(capability);
    else
        QAL_ERR(LOG_TAG, "failed to get device instance.");

    return status;
}

int ResourceManager::handleDeviceConnectionChange(qal_param_device_connection_t connection_state) {
    int status = 0;
    qal_device_id_t device_id = connection_state.id;
    bool is_connected = connection_state.connection_state;
    bool device_available = isDeviceAvailable(device_id);
    struct qal_device dAttr;
    struct qal_device conn_device;
    std::shared_ptr<Device> dev = nullptr;
    struct qal_device_info devinfo = {};

    QAL_DBG(LOG_TAG, "%s Enter", __func__);
    memset(&conn_device, 0, sizeof(struct qal_device));
    if (is_connected && !device_available) {
        if (isPluginDevice(device_id)) {
            conn_device.id = device_id;
            dev = Device::getInstance(&conn_device, rm);
            if (dev) {
                addPlugInDevice(dev, connection_state);
            } else {
                QAL_ERR(LOG_TAG, "Device creation failed");
                throw std::runtime_error("failed to create device object");
            }
        } else if (isDpDevice(device_id)) {
            conn_device.id = device_id;
            dev = Device::getInstance(&conn_device, rm);
            if (dev) {
                addPlugInDevice(dev, connection_state);
            } else {
                QAL_ERR(LOG_TAG, "Device creation failed");
                throw std::runtime_error("failed to create device object");
            }
        }

        QAL_DBG(LOG_TAG, "Mark device %d as available", device_id);
        if (device_id == QAL_DEVICE_OUT_BLUETOOTH_A2DP) {
            dAttr.id = device_id;
            /* Stream type is irrelevant here as we need device num channels
               which is independent of stype for BT devices */
            rm->getDeviceInfo(dAttr.id, QAL_STREAM_LOW_LATENCY, &devinfo);
            if ((devinfo.channels == 0) ||
                   (devinfo.channels > devinfo.max_channels)) {
                QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                return -EINVAL;
            }
            status = getDeviceConfig(&dAttr, NULL, devinfo.channels);
            if (status) {
                QAL_ERR(LOG_TAG, "Device config not overwritten %d", status);
                return status;
            }
            dev = Device::getInstance(&dAttr, rm);
            if (!dev) {
                QAL_ERR(LOG_TAG, "Device creation failed");
                return -EINVAL;
            }
        } else if (isBtScoDevice(device_id)) {
            dAttr.id = device_id;
            /* Stream type is irrelevant here as we need device num channels
               which is independent of stype for BT devices */
            rm->getDeviceInfo(dAttr.id, QAL_STREAM_LOW_LATENCY, &devinfo);
            if ((devinfo.channels == 0) ||
                   (devinfo.channels > devinfo.max_channels)) {
                QAL_ERR(LOG_TAG, "Invalid num channels [%d], exiting", devinfo.channels);
                return -EINVAL;
            }
            status = getDeviceConfig(&dAttr, NULL, devinfo.channels);
            if (status) {
                QAL_ERR(LOG_TAG, "Device config not overwritten %d", status);
                return status;
            }
            dev = Device::getInstance(&dAttr, rm);
            if (!dev) {
                QAL_ERR(LOG_TAG, "Device creation failed");
                throw std::runtime_error("failed to create device object");
                return -EIO;
            }
        }
        avail_devices_.push_back(device_id);
    } else if (!is_connected && device_available) {
        if (isPluginDevice(device_id)) {
            conn_device.id = device_id;
            removePlugInDevice(device_id, connection_state);
        } else if (isDpDevice(device_id)) {
            conn_device.id = device_id;
            removePlugInDevice(device_id, connection_state);
        }

        QAL_DBG(LOG_TAG, "Mark device %d as unavailable", device_id);
        avail_devices_.erase(std::find(avail_devices_.begin(), avail_devices_.end(), device_id));
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid operation, connection state %d, device avalibilty %d",
                is_connected, device_available);
    }

    QAL_DBG(LOG_TAG, "%s Exit, status %d", __func__, status);
    return status;
}

int ResourceManager::resetStreamInstanceID(Stream *str, uint32_t sInstanceID) {
    int status = 0;
    int listNodeIndex = -1;
    qal_stream_attributes StrAttr;
    KeyVect_t streamConfigModifierKV;

    status = str->getStreamAttributes(&StrAttr);

    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    mResourceManagerMutex.lock();

    switch (StrAttr.type) {
        case QAL_STREAM_VOICE_UI:
            streamConfigModifierKV = str->getStreamModifiers();

            if (streamConfigModifierKV.size() == 0) {
                QAL_DBG(LOG_TAG, "%s: no streamConfigModifierKV");
                break;
            }

            for (int x = 0; x < STInstancesLists.size(); x++) {
                if (STInstancesLists[x].first == streamConfigModifierKV[0].second) {
                    QAL_DBG(LOG_TAG,"Found matching StreamConfig(%x) in STInstancesLists(%d)",
                        streamConfigModifierKV[0].second, x);

                    for (int i = 0; i < max_session_num; i++) {
                        if (STInstancesLists[x].second[i].first == sInstanceID){
                            STInstancesLists[x].second[i].second = false;
                            QAL_DBG(LOG_TAG,"ListNodeIndex(%d), InstanceIndex(%d)"
                                  "Instance(%d) to false",
                                  x,
                                  i,
                                  sInstanceID);
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        default:
            QAL_ERR(LOG_TAG, "Invalid streamtype %d",
                    StrAttr.type);
            break;
    }

exit:
    mResourceManagerMutex.unlock();
    return status;

}

int ResourceManager::getStreamInstanceID(Stream *str) {
    int status = 0;
    int i;
    int listNodeIndex = -1;
    qal_stream_attributes StrAttr;
    KeyVect_t streamConfigModifierKV;

    status = str->getStreamAttributes(&StrAttr);

    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    mResourceManagerMutex.lock();

    switch (StrAttr.type) {
        case QAL_STREAM_VOICE_UI:
            QAL_DBG(LOG_TAG,"STInstancesLists.size (%d)", STInstancesLists.size());

            streamConfigModifierKV = str->getStreamModifiers();

            if (streamConfigModifierKV.size() == 0) {
                QAL_DBG(LOG_TAG, "%s: no streamConfigModifierKV");
                break;
            }

            for (int x = 0; x < STInstancesLists.size(); x++) {
                if (STInstancesLists[x].first == streamConfigModifierKV[0].second) {
                    QAL_DBG(LOG_TAG,"Found list for StreamConfig(%x),index(%d)",
                        streamConfigModifierKV[0].second, x);
                    listNodeIndex = x;
                    break;
                }
            }

            if (listNodeIndex < 0) {
                InstanceListNode_t streamConfigInstanceList;
                QAL_DBG(LOG_TAG,"Create InstanceID list for streamConfig(%x)",
                    streamConfigModifierKV[0].second);

                STInstancesLists.push_back(make_pair(
                    streamConfigModifierKV[0].second,
                    streamConfigInstanceList));
                //Initialize List
                for (i = 1; i <= max_session_num; i++) {
                    STInstancesLists.back().second.push_back(std::make_pair(i, false));
                }
                listNodeIndex = STInstancesLists.size() - 1;
            }

            for (i = 0; i < max_session_num; i++) {
                if (!STInstancesLists[listNodeIndex].second[i].second) {
                    STInstancesLists[listNodeIndex].second[i].second = true;
                    status = STInstancesLists[listNodeIndex].second[i].first;
                    QAL_DBG(LOG_TAG,"ListNodeIndex(%d), InstanceIndex(%d)"
                          "Instance(%d) to true",
                          listNodeIndex,
                          i,
                          status);
                    break;
                }
            }
            break;
        default:
            QAL_ERR(LOG_TAG, "Invalid streamtype %d",
                    StrAttr.type);
            break;
    }

exit:
    mResourceManagerMutex.unlock();
    return status;

}

bool ResourceManager::isDeviceAvailable(qal_device_id_t id)
{
    bool is_available = false;
    typename std::vector<qal_device_id_t>::iterator iter =
        std::find(avail_devices_.begin(), avail_devices_.end(), id);

    if (iter != avail_devices_.end())
        is_available = true;

    QAL_DBG(LOG_TAG, "Device %d, is_available = %d", id, is_available);

    return is_available;
}

bool ResourceManager::isDeviceReady(qal_device_id_t id)
{
    struct qal_device dAttr;
    std::shared_ptr<Device> dev = nullptr;
    bool is_ready = false;

    switch (id) {
        case QAL_DEVICE_OUT_BLUETOOTH_SCO:
        case QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
        case QAL_DEVICE_OUT_BLUETOOTH_A2DP:
        case QAL_DEVICE_IN_BLUETOOTH_A2DP:
        {
            if (!isDeviceAvailable(id))
                return is_ready;

            dAttr.id = id;
            dev = Device::getInstance((struct qal_device *)&dAttr , rm);
            if (!dev) {
                QAL_ERR(LOG_TAG, "Device getInstance failed");
                return false;
            }
            is_ready = dev->isDeviceReady();
            break;
        }
        default:
            is_ready = true;
            break;
    }

    return is_ready;
}

bool ResourceManager::isBtScoDevice(qal_device_id_t id)
{
    if (id == QAL_DEVICE_OUT_BLUETOOTH_SCO ||
        id == QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET)
        return true;
    else
        return false;
}

void ResourceManager::updateBtCodecMap(std::pair<uint32_t, std::string> key, std::string value)
{
    btCodecMap.insert(std::make_pair(key, value));
}

std::string ResourceManager::getBtCodecLib(uint32_t codecFormat, std::string codecType)
{
    std::map<std::pair<uint32_t, std::string>, std::string>::iterator iter;

    iter = btCodecMap.find(std::make_pair(codecFormat, codecType));
    if (iter != btCodecMap.end()) {
        return iter->second;
    }

    return std::string();
}

void ResourceManager::processBTCodecInfo(const XML_Char **attr)
{
    char *saveptr = NULL;
    char *token = NULL;
    std::vector<std::string> codec_formats, codec_types;
    std::vector<std::string>::iterator iter1, iter2;
    std::map<std::string, uint32_t>::iterator iter;

    if (strcmp(attr[0], "codec_format") != 0) {
        QAL_ERR(LOG_TAG,"'codec_format' not found");
        goto done;
    }

    if (strcmp(attr[2], "codec_type") != 0) {
        QAL_ERR(LOG_TAG,"'codec_type' not found");
        goto done;
    }

    if (strcmp(attr[4], "codec_library") != 0) {
        QAL_ERR(LOG_TAG,"'codec_library' not found");
        goto done;
    }

    token = strtok_r((char *)attr[1], "|", &saveptr);
    while (token != NULL) {
        if (strlen(token) != 0) {
            codec_formats.push_back(std::string(token));
        }
        token = strtok_r(NULL, "|", &saveptr);
    }

    token = strtok_r((char *)attr[3], "|", &saveptr);
    while (token != NULL) {
        if (strlen(token) != 0) {
            codec_types.push_back(std::string(token));
        }
        token = strtok_r(NULL, "|", &saveptr);
    }

    for (iter1 = codec_formats.begin(); iter1 != codec_formats.end(); ++iter1) {
        for (iter2 = codec_types.begin(); iter2 != codec_types.end(); ++iter2) {
            QAL_VERBOSE(LOG_TAG, "BT Codec Info %s=%s, %s=%s, %s=%s",
                    attr[0], (*iter1).c_str(), attr[2], (*iter2).c_str(), attr[4], attr[5]);

            iter = btFmtTable.find(*iter1);
            if (iter != btFmtTable.end()) {
                updateBtCodecMap(std::make_pair(btFmtTable[*iter1], *iter2),  std::string(attr[5]));
            }
        }
    }

done:
    return;
}

bool ResourceManager::isPluginDevice(qal_device_id_t id) {
    if (id == QAL_DEVICE_OUT_USB_DEVICE ||
        id == QAL_DEVICE_OUT_USB_HEADSET ||
        id == QAL_DEVICE_IN_USB_DEVICE ||
        id == QAL_DEVICE_IN_USB_HEADSET)
        return true;
    else
        return false;
}

bool ResourceManager::isDpDevice(qal_device_id_t id) {
    if (id == QAL_DEVICE_OUT_AUX_DIGITAL || id == QAL_DEVICE_OUT_AUX_DIGITAL_1 ||
        id == QAL_DEVICE_OUT_HDMI)
        return true;
    else
        return false;
}

void ResourceManager::processTagInfo(const XML_Char **attr)
{
    int32_t tagId;
    int32_t found = 0;
    if (strcmp(attr[0], "id" ) !=0 ) {
        QAL_ERR(LOG_TAG, " 'id' not found");
        return;
    }
    std::string tagCh(attr[1]);

    tagId = convertCharToHex(tagCh);
    if (strcmp(attr[2], "name") != 0) {
        QAL_ERR(LOG_TAG, " 'name' not found");
        return;
    }

    std::string name(attr[3]);
    std::string String("stream");
    found = name.find(String);
    if (found != std::string::npos) {
        updateStreamTag(tagId);
        QAL_ERR(LOG_TAG,"%s:%d    %x", __func__, __LINE__, tagId);
    }
    found = 0;
    found = name.find(std::string("device"));
    if (found != std::string::npos) {
        updateDeviceTag(tagId);
    }

}

void ResourceManager::processConfigParams(const XML_Char **attr)
{
    if (strcmp(attr[0], "key") != 0) {
        QAL_ERR(LOG_TAG,"'key' not found");
        goto done;
    }

    if (strcmp(attr[2], "value") != 0) {
        QAL_ERR(LOG_TAG,"'value' not found");
        goto done;
    }
    QAL_VERBOSE(LOG_TAG, " String %s %s %s %s ",attr[0],attr[1],attr[2],attr[3]);
    configParamKVPairs = str_parms_create();
    str_parms_add_str(configParamKVPairs, (char*)attr[1], (char*)attr[3]);
    setConfigParams(configParamKVPairs);
    str_parms_destroy(configParamKVPairs);
done:
    return;
}

void ResourceManager::processCardInfo(struct xml_userdata *data, const XML_Char *tag_name)
{
    int card;
    if (!strcmp(tag_name, "id")) {
        card = atoi(data->data_buf);
        data->card_found = true;
    }
}

void ResourceManager::processDeviceIdProp(struct xml_userdata *data, const XML_Char *tag_name)
{
    int device, size = -1;
    struct deviceCap dev;

    memset(&dev, 0, sizeof(struct deviceCap));
    if (!strcmp(tag_name, "pcm-device") ||
        !strcmp(tag_name, "compress-device") ||
        !strcmp(tag_name, "mixer"))
        return;

    if (!strcmp(tag_name, "id")) {
        device = atoi(data->data_buf);
        dev.deviceId = device;
        devInfo.push_back(dev);
    } else if (!strcmp(tag_name, "name")) {
        size = devInfo.size() - 1;
        strlcpy(devInfo[size].name, data->data_buf, strlen(data->data_buf)+1);
        if(strstr(data->data_buf,"PCM")) {
            devInfo[size].type = PCM;
        } else if (strstr(data->data_buf,"COMP")) {
            devInfo[size].type = COMPRESS;
        } else if (strstr(data->data_buf,"VOICEMMODE1")){
            devInfo[size].type = VOICE1;
        } else if (strstr(data->data_buf,"VOICEMMODE2")){
            devInfo[size].type = VOICE2;
        }
    }
}

void ResourceManager::processDeviceCapability(struct xml_userdata *data, const XML_Char *tag_name)
{
    int size = -1;
    int val = -1;
    if (!strlen(data->data_buf) || !strlen(tag_name))
        return;
    if (strcmp(tag_name, "props") == 0)
        return;
    size = devInfo.size() - 1;
    if (strcmp(tag_name, "playback") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].playback = val;
    } else if (strcmp(tag_name, "capture") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].record = val;
    } else if (strcmp(tag_name, "hostless") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].loopback = val;
    }
}

void ResourceManager::snd_reset_data_buf(struct xml_userdata *data)
{
    data->offs = 0;
    data->data_buf[data->offs] = '\0';
}

void ResourceManager::process_voicemode_info(const XML_Char **attr)
{
    std::string tagkey(attr[1]);
    std::string tagvalue(attr[3]);
    struct vsid_modepair modepair = {};

    if (strcmp(attr[0], "key") !=0) {
        QAL_ERR(LOG_TAG, "key not found");
        return;
    }
    modepair.key = convertCharToHex(tagkey);

    if (strcmp(attr[2], "value") !=0) {
        QAL_ERR(LOG_TAG, "value not found");
        return;
    }
    modepair.value = convertCharToHex(tagvalue);
    QAL_INFO(LOG_TAG, "key  %x value  %x", modepair.key, modepair.value);
    vsidInfo.modepair.push_back(modepair);
}

void ResourceManager::process_config_voice(struct xml_userdata *data, const XML_Char *tag_name)
{
    if(data->voice_info_parsed)
        return;

    if (data->offs <= 0)
        return;
    data->data_buf[data->offs] = '\0';
    if (data->tag == TAG_CONFIG_VOICE) {
        if (strcmp(tag_name, "vsid") == 0) {
            std::string vsidvalue(data->data_buf);
            vsidInfo.vsid = convertCharToHex(vsidvalue);
        }
    }
    if (!strcmp(tag_name, "modepair")) {
        data->tag = TAG_CONFIG_MODE_MAP;
    } else if (!strcmp(tag_name, "mode_map")) {
        data->tag = TAG_CONFIG_VOICE;
    } else if (!strcmp(tag_name, "config_voice")) {
        data->tag = TAG_RESOURCE_MANAGER_INFO;
        data->voice_info_parsed = true;
    }
}

void ResourceManager::process_kvinfo(const XML_Char **attr)
{
    struct kvpair_info kv;
    int size = 0, sizeusecase = 0;
    std::string tagkey(attr[1]);
    std::string tagvalue(attr[3]);

    if (strcmp(attr[0], "key") !=0) {
        QAL_ERR(LOG_TAG, "key not found");
        return;
    }
    kv.key = convertCharToHex(tagkey);
    if (strcmp(attr[2], "value") !=0) {
        QAL_ERR(LOG_TAG, "value not found");
        return;
    }
    kv.value = convertCharToHex(tagvalue);

    size = deviceInfo.size() - 1;
    sizeusecase = deviceInfo[size].usecase.size() - 1;
    deviceInfo[size].usecase[sizeusecase].kvpair.push_back(kv);
    QAL_DBG(LOG_TAG, "key  %x value  %x", kv.key, kv.value);
}

void ResourceManager::process_device_info(struct xml_userdata *data, const XML_Char *tag_name)
{

    struct deviceIn dev = {};
    struct usecase_info usecase_data = {};
    int size = -1 , sizeusecase = -1;

    if (data->offs <= 0)
        return;
    data->data_buf[data->offs] = '\0';

    if (data->resourcexml_parsed)
      return;

    if ((data->tag == TAG_IN_DEVICE) || (data->tag == TAG_OUT_DEVICE)) {
        if (!strcmp(tag_name, "id")) {
            std::string deviceName(data->data_buf);
            dev.deviceId  = deviceIdLUT.at(deviceName);
            deviceInfo.push_back(dev);
        } else if (!strcmp(tag_name, "back_end_name")) {
            std::string backendname(data->data_buf);
            size = deviceInfo.size() - 1;
            updateBackEndName(deviceInfo[size].deviceId, backendname);
        } else if (!strcmp(tag_name, "max_channels")) {
            size = deviceInfo.size() - 1;
            deviceInfo[size].max_channel = atoi(data->data_buf);
        } else if (!strcmp(tag_name, "channels")) {
            size = deviceInfo.size() - 1;
            deviceInfo[size].channel = atoi(data->data_buf);
        } else if (!strcmp(tag_name, "snd_device_name")) {
            size = deviceInfo.size() - 1;
            std::string snddevname(data->data_buf);
            updateSndName(deviceInfo[size].deviceId, snddevname);
        }
    } else if (data->tag == TAG_USECASE) {
        if (!strcmp(tag_name, "name")) {
            std::string userIdname(data->data_buf);
            usecase_data.type  = usecaseIdLUT.at(userIdname);
            size = deviceInfo.size() - 1;
            deviceInfo[size].usecase.push_back(usecase_data);
        } else if (!strcmp(tag_name, "sidetone_mode")) {
            std::string mode(data->data_buf);
            size = deviceInfo.size() - 1;
            sizeusecase = deviceInfo[size].usecase.size() - 1;
            deviceInfo[size].usecase[sizeusecase].sidetoneMode = sidetoneModetoId.at(mode);
        }
    }
    if (!strcmp(tag_name, "kvpair")) {
        data->tag = TAG_DEVICEPP;
    } else if (!strcmp(tag_name, "devicePP-metadata")) {
        data->tag = TAG_USECASE;
    } else if (!strcmp(tag_name, "usecase")) {
        data->tag = TAG_IN_DEVICE;
    } else if (!strcmp(tag_name, "in-device") || !strcmp(tag_name, "out-device")) {
        data->tag = TAG_DEVICE_PROFILE;
    } else if (!strcmp(tag_name, "device_profile")) {
        data->tag = TAG_RESOURCE_MANAGER_INFO;
    } else if (!strcmp(tag_name, "sidetone_mode")) {
        data->tag = TAG_USECASE;
    } else if (!strcmp(tag_name, "resource_manager_info")) {
        data->tag = TAG_RESOURCE_ROOT;
        data->resourcexml_parsed = true;
    }
}

void ResourceManager::process_input_streams(struct xml_userdata *data, const XML_Char *tag_name)
{
    struct tx_ecinfo txecinfo = {};
    int type = 0;
    int size = -1 , typesize = -1;

    if (data->offs <= 0)
        return;
    data->data_buf[data->offs] = '\0';

    if (data->resourcexml_parsed)
      return;

    if (data->tag == TAG_INSTREAM) {
        if (!strcmp(tag_name, "name")) {
            std::string userIdname(data->data_buf);
            txecinfo.tx_stream_type  = usecaseIdLUT.at(userIdname);
            txEcInfo.push_back(txecinfo);
            QAL_DBG(LOG_TAG, "name %d", txecinfo.tx_stream_type);
        }
    } else if (data->tag == TAG_ECREF) {
        if (!strcmp(tag_name, "disabled_stream")) {
            std::string userIdname(data->data_buf);
            type  = usecaseIdLUT.at(userIdname);
            size = txEcInfo.size() - 1;
            txEcInfo[size].disabled_rx_streams.push_back(type);
            QAL_DBG(LOG_TAG, "ecref %d", type);
        }
    }
    if (!strcmp(tag_name, "in_streams")) {
        data->tag = TAG_INSTREAMS;
    } else if (!strcmp(tag_name, "in_stream")) {
        data->tag = TAG_INSTREAM;
    } else if (!strcmp(tag_name, "policies")) {
        data->tag = TAG_POLICIES;
    } else if (!strcmp(tag_name, "ec_ref")) {
        data->tag = TAG_ECREF;
    } else if (!strcmp(tag_name, "resource_manager_info")) {
        data->tag = TAG_RESOURCE_ROOT;
        data->resourcexml_parsed = true;
    }
}

void ResourceManager::snd_process_data_buf(struct xml_userdata *data, const XML_Char *tag_name)
{
    if (data->offs <= 0)
        return;

    data->data_buf[data->offs] = '\0';

    if (data->card_parsed)
        return;

    if (data->current_tag == TAG_ROOT)
        return;

    if (data->current_tag == TAG_CARD) {
        processCardInfo(data, tag_name);
    } else if (data->current_tag == TAG_PLUGIN) {
        //snd_parse_plugin_properties(data, tag_name);
    } else if (data->current_tag == TAG_DEVICE) {
        //QAL_ERR(LOG_TAG,"tag %s", (char*)tag_name);
        processDeviceIdProp(data, tag_name);
    } else if (data->current_tag == TAG_DEV_PROPS) {
        processDeviceCapability(data, tag_name);
    }
}

void ResourceManager::startTag(void *userdata, const XML_Char *tag_name,
    const XML_Char **attr)
{
    stream_supported_type type;
    struct xml_userdata *data = (struct xml_userdata *)userdata;
    static std::shared_ptr<SoundTriggerPlatformInfo> st_info = nullptr;

    if (data->is_parsing_sound_trigger) {
        st_info->HandleStartTag((const char *)tag_name, (const char **)attr);
        return;
    }

    if (!strcmp(tag_name, "sound_trigger_platform_info")) {
        data->is_parsing_sound_trigger = true;
        st_info = SoundTriggerPlatformInfo::GetInstance();
        return;
    }

    if (strcmp(tag_name, "device") == 0) {
        return;
    } else if (strcmp(tag_name, "Tag") == 0) {
        processTagInfo(attr);
        return;
    } else if (strcmp(tag_name, "TAG") == 0) {
        processTagInfo(attr);
        return;
    } else if(strcmp(tag_name, "param") == 0) {
        processConfigParams(attr);
    } else if (strcmp(tag_name, "codec") == 0) {
        processBTCodecInfo(attr);
        return;
    }

    if (data->card_parsed)
        return;

    snd_reset_data_buf(data);

    if (!strcmp(tag_name, "resource_manager_info")) {
        data->tag = TAG_RESOURCE_MANAGER_INFO;
    } else if (!strcmp(tag_name, "config_voice")) {
        data->tag = TAG_CONFIG_VOICE;
    } else if (!strcmp(tag_name, "mode_map")) {
        data->tag = TAG_CONFIG_MODE_MAP;
    } else if (!strcmp(tag_name, "modepair")) {
        data->tag = TAG_CONFIG_MODE_PAIR;
        process_voicemode_info(attr);
    } else if (!strcmp(tag_name, "device_profile")) {
        data->tag = TAG_DEVICE_PROFILE;
    } else if (!strcmp(tag_name, "in-device")) {
        data->tag = TAG_IN_DEVICE;
    } else if (!strcmp(tag_name, "out-device")) {
        data->tag = TAG_OUT_DEVICE;
    } else if (!strcmp(tag_name, "usecase")) {
        data->tag = TAG_USECASE;
    } else if (!strcmp(tag_name, "devicePP-metadata")) {
        data->tag = TAG_DEVICEPP;
    } else if (!strcmp(tag_name, "kvpair")) {
        process_kvinfo(attr);
        data->tag = TAG_KVPAIR;
    } else if (!strcmp(tag_name, "in_streams")) {
        data->tag = TAG_INSTREAMS;
    } else if (!strcmp(tag_name, "in_stream")) {
        data->tag = TAG_INSTREAM;
    } else if (!strcmp(tag_name, "policies")) {
        data->tag = TAG_POLICIES;
    } else if (!strcmp(tag_name, "ec_ref")) {
        data->tag = TAG_ECREF;
    }else if (!strcmp(tag_name, "sidetone_mode")) {
        data->tag = TAG_USECASE;
    }

    if (!strcmp(tag_name, "card"))
        data->current_tag = TAG_CARD;
    if (strcmp(tag_name, "pcm-device") == 0) {
        type = PCM;
        data->current_tag = TAG_DEVICE;
    } else if (strcmp(tag_name, "compress-device") == 0) {
        data->current_tag = TAG_DEVICE;
        type = COMPRESS;
    } else if (strcmp(tag_name, "mixer") == 0) {
        data->current_tag = TAG_MIXER;
    } else if (strstr(tag_name, "plugin")) {
        data->current_tag = TAG_PLUGIN;
    } else if (!strcmp(tag_name, "props")) {
        data->current_tag = TAG_DEV_PROPS;
    }
    if (data->current_tag != TAG_CARD && !data->card_found)
        return;
}

void ResourceManager::endTag(void *userdata, const XML_Char *tag_name)
{
    struct xml_userdata *data = (struct xml_userdata *)userdata;
    std::shared_ptr<SoundTriggerPlatformInfo> st_info =
        SoundTriggerPlatformInfo::GetInstance();

    if (!strcmp(tag_name, "sound_trigger_platform_info")) {
        data->is_parsing_sound_trigger = false;
        return;
    }

    if (data->is_parsing_sound_trigger) {
        st_info->HandleEndTag((const char *)tag_name);
        return;
    }

    process_config_voice(data,tag_name);
    process_device_info(data,tag_name);
    process_input_streams(data,tag_name);

    if (data->card_parsed)
        return;
    if (data->current_tag != TAG_CARD && !data->card_found)
        return;
    snd_process_data_buf(data, tag_name);
    snd_reset_data_buf(data);
    if (!strcmp(tag_name, "mixer") || !strcmp(tag_name, "pcm-device") || !strcmp(tag_name, "compress-device"))
        data->current_tag = TAG_CARD;
    else if (strstr(tag_name, "plugin") || !strcmp(tag_name, "props"))
        data->current_tag = TAG_DEVICE;
    else if(!strcmp(tag_name, "card")) {
        data->current_tag = TAG_ROOT;
        if (data->card_found)
            data->card_parsed = true;
    }
}

void ResourceManager::snd_data_handler(void *userdata, const XML_Char *s, int len)
{
   struct xml_userdata *data = (struct xml_userdata *)userdata;

    if (data->is_parsing_sound_trigger) {
        SoundTriggerPlatformInfo::GetInstance()->HandleCharData(
            (const char *)s);
        return;
    }

   if (len + data->offs >= sizeof(data->data_buf) ) {
       data->offs += len;
       /* string length overflow, return */
       return;
   } else {
       memcpy(data->data_buf + data->offs, s, len);
       data->offs += len;
   }
}

int ResourceManager::XmlParser(std::string xmlFile)
{
    XML_Parser parser;
    FILE *file = NULL;
    int ret = 0;
    int bytes_read;
    void *buf = NULL;
    struct xml_userdata card_data;
    memset(&card_data, 0, sizeof(card_data));

    QAL_INFO(LOG_TAG, "Enter. XML parsing started - file name %s", xmlFile.c_str());
    file = fopen(xmlFile.c_str(), "r");
    if(!file) {
        ret = EINVAL;
        QAL_ERR(LOG_TAG, "Failed to open xml file name %s ret %d", xmlFile.c_str(), ret);
        goto done;
    }

    parser = XML_ParserCreate(NULL);
    if (!parser) {
        ret = -EINVAL;
        QAL_ERR(LOG_TAG, "Failed to create XML ret %d", ret);
        goto closeFile;
    }
    XML_SetUserData(parser, &card_data);
    XML_SetElementHandler(parser, startTag, endTag);
    XML_SetCharacterDataHandler(parser, snd_data_handler);

    while (1) {
        buf = XML_GetBuffer(parser, 1024);
        if(buf == NULL) {
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, "XML_Getbuffer failed ret %d", ret);
            goto freeParser;
        }

        bytes_read = fread(buf, 1, 1024, file);
        if(bytes_read < 0) {
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, "fread failed ret %d", ret);
            goto freeParser;
        }

        if(XML_ParseBuffer(parser, bytes_read, bytes_read == 0) == XML_STATUS_ERROR) {
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, "XML ParseBuffer failed for %s file ret %d", xmlFile.c_str(), ret);
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
