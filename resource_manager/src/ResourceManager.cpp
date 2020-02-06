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

#define LOG_TAG "ResourceManager"
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
#include "SoundTriggerPlatformInfo.h"

#ifndef FEATURE_IPQ_OPENWRT
#include <cutils/str_parms.h>
#endif

#define MIXER_FILE_DELIMITER "_"
#define MIXER_FILE_EXT ".xml"
#define MIXER_PATH_MAX_LENGTH 100

#if defined(FEATURE_IPQ_OPENWRT) || defined(LINUX_ENABLED)
#define MIXER_XML_BASE_STRING "/etc/mixer_paths"
#define MIXER_XML_DEFAULT_PATH "/etc/mixer_paths_wsa.xml"
#define DEFAULT_ACDB_FILES "/etc/acdbdata/MTP/acdb_cal.acdb"
#define XMLFILE "/etc/resourcemanager.xml"
#define GECKOXMLFILE "/etc/kvh2xml.xml"
#define SNDPARSER "/etc/card-defs.xml"
#define STXMLFILE "/etc/sound_trigger_platform_info.xml"
#else
#define MIXER_XML_BASE_STRING "/vendor/etc/mixer_paths"
#define MIXER_XML_DEFAULT_PATH "/vendor/etc/mixer_paths_wsa.xml"
#define DEFAULT_ACDB_FILES "/vendor/etc/acdbdata/MTP/acdb_cal.acdb"
#define XMLFILE "/vendor/etc/resourcemanager.xml"
#define GECKOXMLFILE "/vendor/etc/kvh2xml.xml"
#define SNDPARSER "/vendor/etc/card-defs.xml"
#define STXMLFILE "/vendor/etc/sound_trigger_platform_info.xml"
#endif

#define MAX_SND_CARD 110
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
#define MAX_SESSIONS_DEEP_BUFFER 1
#define MAX_SESSIONS_COMPRESSED 10
#define MAX_SESSIONS_GENERIC 1
#define MAX_SESSIONS_PCM_OFFLOAD 1
#define MAX_SESSIONS_VOICE_UI 2

static struct str_parms *configParamKVPairs;

/*
To be defined in detail, if GSL is defined,
pcm device id is directly related to device,
else using legacy design for alsa
*/
// Will update actual value when numbers got for VT

std::vector<std::pair<int32_t, std::string>> ResourceManager::deviceLinkName {
    {QAL_DEVICE_NONE,                     {std::string{ "none" }}},
    {QAL_DEVICE_OUT_HANDSET,             {std::string{ "" }}},
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

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "tdm-pri" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "tdm-pri" }}},
    {QAL_DEVICE_IN_TRI_MIC,               {std::string{ "tdm-pri" }}},
    {QAL_DEVICE_IN_QUAD_MIC,              {std::string{ "" }}},
    {QAL_DEVICE_IN_EIGHT_MIC,             {std::string{ "" }}},
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
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "" }}}
};

std::vector<std::pair<int32_t, int32_t>> ResourceManager::devicePcmId {
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
    {QAL_DEVICE_IN_HANDSET_MIC,           0},
    {QAL_DEVICE_IN_SPEAKER_MIC,           0},
    {QAL_DEVICE_IN_TRI_MIC,               0},
    {QAL_DEVICE_IN_QUAD_MIC,              0},
    {QAL_DEVICE_IN_EIGHT_MIC,             0},
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
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        0},
};

// To be defined in detail

std::vector<std::pair<int32_t, std::string>> ResourceManager::sndDeviceNameLUT {
    {QAL_DEVICE_NONE,                     {std::string{ "none" }}},
    {QAL_DEVICE_OUT_HANDSET,             {std::string{ "" }}},
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

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "" }}},
    {QAL_DEVICE_IN_TRI_MIC,               {std::string{ "" }}},
    {QAL_DEVICE_IN_QUAD_MIC,              {std::string{ "" }}},
    {QAL_DEVICE_IN_EIGHT_MIC,             {std::string{ "" }}},
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
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "" }}}
};

std::shared_ptr<ResourceManager> ResourceManager::rm = nullptr;
std::vector <int> ResourceManager::streamTag = {0};
std::vector <int> ResourceManager::streamPpTag = {0};
std::vector <int> ResourceManager::mixerTag = {0};
std::vector <int> ResourceManager::devicePpTag = {0};
std::vector <int> ResourceManager::deviceTag = {0};
std::mutex ResourceManager::mResourceManagerMutex;
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


//TODO:Needs to define below APIs so that functionality won't break
#ifdef FEATURE_IPQ_OPENWRT
int str_parms_get_str(struct str_parms *str_parms, const char *key,
                      char *out_val, int len){return 0;}
char *str_parms_to_str(struct str_parms *str_parms){return NULL;}
int str_parms_add_str(struct str_parms *str_parms, const char *key,
                      const char *value){return 0;}
struct str_parms *str_parms_create(void){return NULL;}
void str_parms_del(struct str_parms *str_parms, const char *key){return;}
#endif
//std::multimap <int, std::string> ResourceManager::listAllBackEndIds;

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
    {QAL_DEVICE_NONE,                     {std::string{ "" }}},
    {QAL_DEVICE_OUT_HANDSET,             {std::string{ "" }}},
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

    {QAL_DEVICE_IN_HANDSET_MIC,           {std::string{ "none" }}},
    {QAL_DEVICE_IN_SPEAKER_MIC,           {std::string{ "none" }}},
    {QAL_DEVICE_IN_TRI_MIC,               {std::string{ "none" }}},
    {QAL_DEVICE_IN_QUAD_MIC,              {std::string{ "" }}},
    {QAL_DEVICE_IN_EIGHT_MIC,             {std::string{ "" }}},
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
    {QAL_DEVICE_IN_HEADSET_VA_MIC,        {std::string{ "none" }}}
};

ResourceManager::ResourceManager()
{
    QAL_INFO(LOG_TAG, "Enter.");
    int ret = 0;
    const qal_alsa_or_gsl ag = getQALConfigALSAOrGSL();
    // TODO: set bOverwriteFlag to true by default
    // should we add api for client to set this value?
    bool bOverwriteFlag = true;
    // Init audio_route and audio_mixer

    na_props.rm_na_prop_enabled = false;
    na_props.ui_na_prop_enabled = false;
    na_props.na_mode = NATIVE_AUDIO_MODE_INVALID;

    //TODO: parse the tag and populate in the tags
    streamTag.clear();
    deviceTag.clear();
    btCodecMap.clear();
    ret = ResourceManager::XmlParser(GECKOXMLFILE);
    if (ret) {
        QAL_ERR(LOG_TAG, "error in gecko xml parsing ret %d", ret);
    }
    ret = ResourceManager::XmlParser(XMLFILE);
    if (ret) {
        QAL_ERR(LOG_TAG, "error in resource xml parsing ret %d", ret);
    }
    ret = ResourceManager::XmlParser(STXMLFILE);
    if (ret) {
        QAL_ERR(LOG_TAG, "error in sound trigger xml parsing ret %d", ret);
        /*
         * clear ret as we want to allow the case where ST xml file is not
         * present
         */
        ret = 0;
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

    ret = ResourceManager::init_audio();
    QAL_INFO(LOG_TAG, "Enter.");
    if (ret) {
        QAL_ERR(LOG_TAG, "error in init audio route and audio mixer ret %d", ret);
    }

    if (ag == GSL) {
        ret = SessionGsl::init(DEFAULT_ACDB_FILES);
    }
    QAL_INFO(LOG_TAG, "Exit. ret %d", ret);
}

ResourceManager::~ResourceManager()
{

}

int ResourceManager::init_audio()
{
    int ret = 0;
    char snd_macro[] = "snd";
    char *snd_card_name = NULL, *snd_card_name_t = NULL;
    char *snd_internal_name = NULL;
    char *tmp = NULL;
    char mixer_xml_file[MIXER_PATH_MAX_LENGTH] = {0};
    QAL_DBG(LOG_TAG, "Enter.");
    snd_card = 0;
    while (snd_card < MAX_SND_CARD) {
        audio_mixer = mixer_open(snd_card);
        if (!audio_mixer) {
            snd_card++;
            continue;
        } else {
            QAL_INFO(LOG_TAG, "mixer open success. snd_card_num = %d, am:%p",
                    snd_card, rm->audio_mixer);
            break;
        }
    }

    if (snd_card >= MAX_SND_CARD || !audio_mixer) {
        QAL_ERR(LOG_TAG, "audio mixer open failure");
        return -EINVAL;
    }

    snd_card_name = strdup(mixer_get_name(audio_mixer));
    if (!snd_card_name) {
        QAL_ERR(LOG_TAG, "failed to allocate memory for snd_card_name");
        mixer_close(audio_mixer);
        return -EINVAL;
    }

    snd_card_name_t = strdup(snd_card_name);
    snd_internal_name = strtok_r(snd_card_name_t, "-", &tmp);

    if (snd_internal_name != NULL)
        snd_internal_name = strtok_r(NULL, "-", &tmp);

    if (snd_internal_name != NULL) {
        QAL_ERR(LOG_TAG, "snd_internal_name: %s", snd_internal_name);
        strlcpy(mixer_xml_file, MIXER_XML_BASE_STRING, MIXER_PATH_MAX_LENGTH);
        ret = strcmp(snd_internal_name, snd_macro);
        if (ret == 0) {
            strlcat(mixer_xml_file, MIXER_FILE_EXT, MIXER_PATH_MAX_LENGTH);
        } else {
            strlcat(mixer_xml_file, MIXER_FILE_DELIMITER, MIXER_PATH_MAX_LENGTH);
            strlcat(mixer_xml_file, snd_internal_name, MIXER_PATH_MAX_LENGTH);
            strlcat(mixer_xml_file, MIXER_FILE_EXT, MIXER_PATH_MAX_LENGTH);
        }
    } else
        strlcpy(mixer_xml_file, MIXER_XML_DEFAULT_PATH, MIXER_PATH_MAX_LENGTH);

    audio_route = audio_route_init(snd_card, mixer_xml_file);
    QAL_INFO(LOG_TAG, "audio route %pK, mixer path %s", audio_route, mixer_xml_file);
    if (!audio_route) {
        QAL_ERR(LOG_TAG, "audio route init failed");
        mixer_close(audio_mixer);
        if (snd_card_name)
            free(snd_card_name);
        if (snd_card_name_t)
            free(snd_card_name_t);
        return -EINVAL;
    }
    // audio_route init success
    QAL_DBG(LOG_TAG, "Exit. audio route init success with card %d mixer path %s",
            snd_card, mixer_xml_file);
    return 0;
}

int ResourceManager::init()
{
    return 0;
}

int32_t ResourceManager::getDeviceConfig(struct qal_device *deviceattr,
                                         struct qal_stream_attributes *sAttr)
{
    int32_t status = 0;
    struct qal_channel_info *dev_ch_info = NULL;

    QAL_ERR(LOG_TAG, "deviceattr->id %d", deviceattr->id);
    switch (deviceattr->id) {
        case QAL_DEVICE_IN_SPEAKER_MIC:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*3);
            dev_ch_info->channels = CHANNELS_3;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
            dev_ch_info->ch_map[2] = QAL_CHMAP_CHANNEL_C;
            //dev_ch_info->ch_map[3] = QAL_CHMAP_CHANNEL_LS;
            //dev_ch_info->ch_map[4] = QAL_CHMAP_CHANNEL_RS;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_IN_TRI_MIC:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*3);
            dev_ch_info->channels = CHANNELS_3;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
            dev_ch_info->ch_map[2] = QAL_CHMAP_CHANNEL_C;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            deviceattr->id = QAL_DEVICE_IN_TRI_MIC;
            break;
        case QAL_DEVICE_IN_HANDSET_MIC:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*1);
            dev_ch_info->channels = CHANNELS_1;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_IN_WIRED_HEADSET:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*1);
            dev_ch_info->channels = CHANNELS_1;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
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
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*1);
            dev_ch_info->channels = CHANNELS_1;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_SPEAKER:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*2);
            dev_ch_info->channels = CHANNELS_2;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_WIRED_HEADPHONE:
        case QAL_DEVICE_OUT_WIRED_HEADSET:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*2);
            dev_ch_info->channels = CHANNELS_2;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
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
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*1);
            dev_ch_info->channels = CHANNELS_2;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            break;
        case QAL_DEVICE_OUT_BLUETOOTH_A2DP:
            dev_ch_info = (struct qal_channel_info *)calloc(1, sizeof(uint16_t) + sizeof(uint8_t));
            dev_ch_info->channels = CHANNELS_1;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_44K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_COMPRESSED;
            break;
        case QAL_DEVICE_OUT_BLUETOOTH_SCO:
        case QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET:
            dev_ch_info =(struct qal_channel_info *) calloc(1, sizeof(uint16_t) + sizeof(uint8_t)*1);
            if (!dev_ch_info) {
                QAL_ERR(LOG_TAG, "out of memory");
                status = -EINVAL;
                break;
            }
            dev_ch_info->channels = CHANNELS_1;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            deviceattr->config.ch_info = dev_ch_info;
            deviceattr->config.sample_rate = SAMPLINGRATE_8K;  /* Updated when WBS set param is received */
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
            QAL_DBG(LOG_TAG, "BT SCO RX device samplerate %d, bitwidth %d", deviceattr->config.sample_rate, deviceattr->config.bit_width);
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
                        deviceattr->config.ch_info->channels);
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
        case QAL_DEVICE_OUT_AUX_DIGITAL:
        case QAL_DEVICE_OUT_HDMI:
            dev_ch_info =(struct qal_channel_info *) calloc(1,sizeof(uint16_t) + sizeof(uint8_t)*2);
            dev_ch_info->channels = CHANNELS_2;
            dev_ch_info->ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            dev_ch_info->ch_map[1] = QAL_CHMAP_CHANNEL_FR;
            deviceattr->config.ch_info = dev_ch_info;
            QAL_DBG(LOG_TAG, "deviceattr->config.ch_info->channels %d", deviceattr->config.ch_info->channels);
            deviceattr->config.sample_rate = SAMPLINGRATE_48K;
            deviceattr->config.bit_width = BITWIDTH_16;
            deviceattr->config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
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
    uint16_t channels, dev_channels;
    uint32_t samplerate, bitwidth;
    uint32_t rc;
    size_t cur_sessions = 0;
    size_t max_sessions = 0;
    qal_audio_fmt_t format, dev_format;

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
        case QAL_STREAM_DEEP_BUFFER:
        case QAL_STREAM_GENERIC:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_PCM_OFFLOAD:
            if (attributes->direction == QAL_AUDIO_INPUT) {
                channels = attributes->in_media_config.ch_info->channels;
                samplerate = attributes->in_media_config.sample_rate;
                bitwidth = attributes->in_media_config.bit_width;
            } else {
                channels = attributes->out_media_config.ch_info->channels;
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
               channels = attributes->in_media_config.ch_info->channels;
               samplerate = attributes->in_media_config.sample_rate;
               bitwidth = attributes->in_media_config.bit_width;
            } else {
               channels = attributes->out_media_config.ch_info->channels;
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
               channels = attributes->in_media_config.ch_info->channels;
               samplerate = attributes->in_media_config.sample_rate;
               bitwidth = attributes->in_media_config.bit_width;
            } else {
               channels = attributes->out_media_config.ch_info->channels;
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
            channels = attributes->out_media_config.ch_info->channels;
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
    qal_stream_attributes incomingStreamAttr;
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

    // Currently inform Rx stream types to soundtrigger streams.
    qal_stream_attributes st_attr;
    s->getStreamAttributes(&st_attr);
    if (st_attr.direction != QAL_AUDIO_INPUT &&
        type != QAL_STREAM_LOW_LATENCY) {
        for (auto& st_stream: active_streams_st) {
            st_stream->ConcurrentStreamStatus(type, true);
        }
    }

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
    QAL_DBG(LOG_TAG, "stream type %d", type);
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
        default:
            ret = -EINVAL;
            QAL_ERR(LOG_TAG, "Invalid stream type = %d ret %d", type, ret);
            break;
    }

    deregisterstream(s, mActiveStreams);
    mResourceManagerMutex.unlock();

    // Currently inform Rx stream types to soundtrigger streams.
    qal_stream_attributes st_attr;
    s->getStreamAttributes(&st_attr);
    if (st_attr.direction != QAL_AUDIO_INPUT &&
        type != QAL_STREAM_LOW_LATENCY) {
        for (auto& st_stream: active_streams_st) {
            st_stream->ConcurrentStreamStatus(type, false);
        }
    }

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
    typename std::vector<std::shared_ptr<Device>>::iterator iter =
        std::find(active_devices.begin(), active_devices.end(), d);
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
    getActiveStreams(d, activestreams, active_streams_ulla);
    getActiveStreams(d, activestreams, active_streams_db);
    getActiveStreams(d, activestreams, active_streams_comp);
    getActiveStreams(d, activestreams, active_streams_st);
    getActiveStreams(d, activestreams, active_streams_po);

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
    uint16_t channels = device->config.ch_info->channels;
    uint32_t samplerate = device->config.sample_rate;
    uint32_t bitwidth = device->config.bit_width;

    QAL_DBG(LOG_TAG, "Enter.");
    //TODO: check and rewrite params if needed
    // only compare with default value for now
    // because no config file parsed in init
    if (channels != DEFAULT_CHANNELS) {
        if (bOverwriteFlag) {
            device->config.ch_info->channels = DEFAULT_CHANNELS;
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
    if (deviceId > QAL_DEVICE_OUT_MIN && deviceId < QAL_DEVICE_IN_MAX) {
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
    if (deviceId > QAL_DEVICE_OUT_MIN && deviceId < QAL_DEVICE_IN_MAX) {
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
    if (deviceId <= QAL_DEVICE_OUT_MIN || deviceId >= QAL_DEVICE_IN_MAX) {
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
        case QAL_STREAM_DEEP_BUFFER:
        case QAL_STREAM_VOIP:
        case QAL_STREAM_VOIP_RX:
        case QAL_STREAM_VOIP_TX:
        case QAL_STREAM_VOICE_UI:
        case QAL_STREAM_PCM_OFFLOAD:
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
    switch(sAttr.type) {
        case QAL_STREAM_LOW_LATENCY:
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
                            listAllPcmLoopbackRxFrontEnds.push_back(frontend.at(i));
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

void ResourceManager::getSharedBEDevices(std::vector<std::shared_ptr<Device>> &deviceList,
        std::shared_ptr<Device> inDevice) const
{
    int dev_id;
    std::string backEndName;
    std::shared_ptr<Device> dev;

    deviceList.clear();
    dev_id = inDevice->getSndDeviceId();
    if (dev_id >= QAL_DEVICE_OUT_HANDSET && dev_id <= QAL_DEVICE_IN_PROXY)
        backEndName = listAllBackEndIds[dev_id].second;

    for (int i = QAL_DEVICE_OUT_HANDSET; i <= QAL_DEVICE_IN_PROXY; i++) {
        if ((i != dev_id) && (backEndName == listAllBackEndIds[i].second)) {
            dev = Device::getObject((qal_device_id_t) i);
            if(dev)
                deviceList.push_back(dev);
        }
    }

    return;
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
        if (dev_id > QAL_DEVICE_OUT_MIN && dev_id < QAL_DEVICE_IN_MAX) {
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
        if ((QAL_AUDIO_OUTPUT == inStrAttr->direction) &&
            (inDevAttr->config.sample_rate % SAMPLINGRATE_44K == 0)) {
            //Native Audio usecase
            QAL_ERR(LOG_TAG, "1 inDevAttr->config.sample_rate = %d  ", inDevAttr->config.sample_rate);
            is_ds_required = true;
        } else if ((activeDevAttr->config.sample_rate < inDevAttr->config.sample_rate) ||
            (activeDevAttr->config.bit_width < inDevAttr->config.bit_width) ||
            (activeDevAttr->config.ch_info->channels < inDevAttr->config.ch_info->channels)) {
            is_ds_required = true;
        }
        break;
    case QAL_DEVICE_OUT_WIRED_HEADSET:
    case QAL_DEVICE_OUT_WIRED_HEADPHONE:
        if ((QAL_STREAM_VOICE_CALL == inStrAttr->type) && ((activeDevAttr->config.sample_rate != inDevAttr->config.sample_rate) ||
            (activeDevAttr->config.bit_width != inDevAttr->config.bit_width) ||
            (activeDevAttr->config.ch_info->channels != inDevAttr->config.ch_info->channels))) {
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
            (activeDevAttr->config.ch_info->channels < inDevAttr->config.ch_info->channels)) {
            is_ds_required = true;
        }
        break;
    default:
        is_ds_required = false;
        break;
    }

    return is_ds_required;
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
    int status = -EINVAL;
    std::vector <Stream *> activeStreams;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    struct qal_device dattr;
    std::vector<Stream*>::iterator sIter;
    std::vector<std::shared_ptr<Device>>::iterator dIter;
    std::shared_ptr<Device> inDevice = nullptr;
    qal_stream_type_t streamType;
    std::vector<std::shared_ptr<Device>> deviceList;

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

    mResourceManagerMutex.lock();

    getActiveStream_l(inDev, activeStreams);
    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no other active streams found so update device cfg");
        inDev->setDeviceAttributes(*inDevAttr);
        goto be_check;
    }

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        status = (*sIter)->getStreamType(&streamType);
        if (QAL_STREAM_VOICE_CALL == streamType)
            isVoiceCall = true;
    }

    if (!isVoiceCall) {
        // All the activesteams using device A should use same device config so no need
        // to run through on all activestreams for device A
        for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
            status = (*sIter)->getAssociatedDevices(associatedDevices);
            if(0 != status) {
                QAL_ERR(LOG_TAG,"getAssociatedDevices Failed");
                goto be_check;
            }

            for(dIter = associatedDevices.begin();
                     dIter != associatedDevices.end(); dIter++) {
                status = (*dIter)->getDeviceAttributes(&dattr);
                if(0 != status) {
                    QAL_ERR(LOG_TAG,"getDeviceAttributes Failed");
                    goto error;
                }

                if (dattr.id == inDevAttr->id) {
                    isDeviceSwitch = isDeviceSwitchRequired(&dattr, inDevAttr, inStrAttr);
                    goto check_ds;
                }
            }
        }
check_ds:
        if (isDeviceSwitch) {
            // case 1. if incoming device config has more priority then do device switch
            // stop all the streams connected to the device
            for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
                (*sIter)->disconnectStreamDevice(*sIter, inDevAttr->id);

            // start all the streams with new device config.
            for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
                (*sIter)->connectStreamDevice(*sIter, inDevAttr);
        } else {
            // case 2. If incoming device config has lower priority then update incoming
            //  device config with currently running device config
            QAL_ERR(LOG_TAG, "%s: device %d is already running with higher priority device config",
                    __func__, inDevAttr->id);
            memcpy(inDevAttr, (void*)&dattr, sizeof(struct qal_device));
        }
    } else {
        // Voice call is active - change incoming device to voice call device config
        memcpy(inDevAttr, (void*)&dattr, sizeof(struct qal_device));
    }

be_check:
    getSharedBEDevices(deviceList, inDev);
    if (deviceList.size() <= 0)
        goto error;

    status = inDev->getDeviceAttributes(&dattr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"getDeviceAttributes Failed");
        goto error;
    }

    for(dIter = deviceList.begin(); dIter != deviceList.end(); dIter++) {
        getActiveStream(*dIter, activeStreams);
        if (activeStreams.size() <= 0)
            continue;
        for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
            (*sIter)->disconnectStreamDevice(*sIter, inDevAttr->id);

        // start all the streams with new device config.
        for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
            (*sIter)->connectStreamDevice(*sIter, inDevAttr);
    }

error:
    mResourceManagerMutex.unlock();

    return isDeviceSwitch;
}

int32_t ResourceManager::forceDeviceSwitch(std::shared_ptr<Device> inDev,
                                              struct qal_device *newDevAttr)
{
    int status = -EINVAL;
    std::vector <Stream *> activeStreams;
    std::vector<std::shared_ptr<Device>> associatedDevices;
    std::vector<Stream*>::iterator sIter;
    std::shared_ptr<Device> inDevice = nullptr;

    if (!inDev || !newDevAttr) {
        return -EINVAL;
    }
    mResourceManagerMutex.lock();

    //get the active streams on the device
    getActiveStream_l(inDev, activeStreams);
    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no other active streams found");
        goto done;
    }

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
        (*sIter)->disconnectStreamDevice(*sIter, newDevAttr->id);

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++)
        (*sIter)->connectStreamDevice(*sIter, newDevAttr);

done:
    mResourceManagerMutex.unlock();
    return 0;
}

const std::string ResourceManager::getQALDeviceName(const qal_device_id_t id) const
{
    QAL_DBG(LOG_TAG, "%s: id %d", __func__, id);
#if 0
    android::CallStack cs;
    cs.update();
    cs.dump(1);
#endif
    return deviceNameLUT.at(id);
}

int ResourceManager::getBackendName(int deviceId, std::string &backendName)
{
    if (deviceId > QAL_DEVICE_NONE && deviceId < QAL_DEVICE_IN_MAX) {
        backendName.assign(listAllBackEndIds[deviceId].second);
    } else {
        QAL_ERR(LOG_TAG, "Invalid device id %d", deviceId);
        return -EINVAL;
    }
    return 0;
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
    int ret = 0, err;
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
    if(kv_pairs != NULL)
        free(kv_pairs);
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
    devicePcmId[deviceId].second = pcmId;
}

void ResourceManager::updateLinkName(int32_t deviceId, std::string linkName)
{
    deviceLinkName[deviceId].second = linkName;
}

void ResourceManager::updateSndName(int32_t deviceId, std::string sndName)
{
    sndDeviceNameLUT[deviceId].second = sndName;
}

void ResourceManager::updateBackEndName(int32_t deviceId, std::string backEndName)
{
    listAllBackEndIds[deviceId].second = backEndName;
}

int convertCharToHex(std::string num)
{
    int32_t hexNum = 0;
    int32_t base = 1;
    const char * charNum = num.c_str();
    int32_t len = strlen(charNum);
    for (int i = len-1; i>=2; i--) {
        if (charNum[i] >= '0' && charNum[i] <= '9') {
            hexNum += (charNum[i] - 48) * base;
            base = base * 16;
        } else if (charNum[i] >= 'A' && charNum[i] <= 'F') {
            hexNum += (charNum[i] - 55) * base;
            base = base * 16;
        } else if (charNum[i] >= 'a' && charNum[i] <= 'f') {
            hexNum += (charNum[i] - 87) * base;
            base = base * 16;
        }
    }
    return hexNum;
}

void ResourceManager::updateStreamTag(int32_t tagId)
{
    streamTag.push_back(tagId);
}

void ResourceManager::updateDeviceTag(int32_t tagId)
{
    deviceTag.push_back(tagId);
}

int32_t ResourceManager::a2dpSuspend()
{
    std::shared_ptr<Device> dev = nullptr;
    struct qal_device dattr;
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector<Stream*>::iterator sIter;

    dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
    dev = Device::getInstance(&dattr , rm);

    getActiveStream(dev, activeStreams);

    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no active streams found");
        goto exit;
    }

    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        int ret = 0;
        qal_stream_type_t type;

        ret = (*sIter)->getStreamType(&type);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "getStreamType failed with status = %d", ret);
            goto exit;
        }
        if (type == QAL_STREAM_COMPRESSED) {
            if (!((*sIter)->a2dp_compress_mute)) {
                struct qal_stream_attributes sAttr;
                struct qal_device speakerDattr;

                QAL_DBG(LOG_TAG, "%s: selecting speaker and muting stream", __func__);
                (*sIter)->pause(); // compress_pause
                (*sIter)->setMute(true); // mute the stream, unmute during a2dp_resume
                (*sIter)->a2dp_compress_mute = true;
                // force switch to speaker
                speakerDattr.id = QAL_DEVICE_OUT_SPEAKER;
                getDeviceConfig(&speakerDattr, &sAttr);
                mResourceManagerMutex.unlock();
                rm->forceDeviceSwitch(dev, &speakerDattr);
                mResourceManagerMutex.lock();
                (*sIter)->resume(); //compress_resume
                /* backup actual device name in stream class */
                (*sIter)->suspendedDevId = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            }
        } else {
            // put to standby for non offload usecase
            (*sIter)->setStandby(true);
        }
    }

exit:
    return status;
}

int32_t ResourceManager::a2dpResume()
{
    std::shared_ptr<Device> dev = nullptr;
    struct qal_device dattr;
    int status = 0;
    std::vector <Stream *> activeStreams;
    std::vector<Stream*>::iterator sIter;

    dattr.id = QAL_DEVICE_OUT_SPEAKER;
    dev = Device::getInstance(&dattr , rm);

    getActiveStream(dev, activeStreams);

    if (activeStreams.size() == 0) {
        QAL_ERR(LOG_TAG, "no active streams found");
        goto exit;
    }

// check all active stream associated with speaker
// if the stream actual device is a2dp, then switch back to a2dp
// unmute the stream
    for(sIter = activeStreams.begin(); sIter != activeStreams.end(); sIter++) {
        int ret = 0;
        qal_stream_type_t type;

        ret = (*sIter)->getStreamType(&type);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "getStreamType failed with status = %d", ret);
            goto exit;
        }
        if (type == QAL_STREAM_COMPRESSED &&
            ((*sIter)->suspendedDevId == QAL_DEVICE_OUT_BLUETOOTH_A2DP)) {
            struct qal_device a2dpDattr;

            a2dpDattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            QAL_DBG(LOG_TAG, "%s: restoring A2dp and unmuting stream", __func__);
            getDeviceConfig(&a2dpDattr, NULL);
            mResourceManagerMutex.unlock();
            rm->forceDeviceSwitch(dev, &a2dpDattr);
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
                     size_t *payload_size)
{
    int status = 0;

    QAL_INFO(LOG_TAG, "%s param_id=%d", __func__, param_id);
    mResourceManagerMutex.lock();
    switch (param_id) {
        case QAL_PARAM_ID_BT_A2DP_RECONFIG_SUPPORTED:
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
        }
            break;

        case QAL_PARAM_ID_DEVICE_CAPABILITY:
        {
            qal_param_device_capability_t *param_device_capability = (qal_param_device_capability_t *)(*param_payload);
            QAL_INFO(LOG_TAG, "Device %d card = %d qalid=%x",
                        param_device_capability->addr.device_num,
                        param_device_capability->addr.card_id,
                        param_device_capability->id);
            status = getDeviceDefaultCapability(*param_device_capability);
        }
            break;

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
        case QAL_PARAM_ID_DEVICE_CONNECTION:
        {
            qal_param_device_connection_t *param_device_connection = (qal_param_device_connection_t *)param_payload;
            QAL_INFO(LOG_TAG, "Device %d connected = %d",
                        param_device_connection->id,
                        param_device_connection->connection_state);
            if (payload_size == sizeof(qal_param_device_connection_t)) {
                status = handleDeviceConnectionChange(*param_device_connection);
            } else {
                QAL_ERR(LOG_TAG,"Incorrect size : expected (%d), received(%d)",
                      sizeof(qal_param_device_connection_t), payload_size);
                status = -EINVAL;
            }
        }
        break;
        case QAL_PARAM_ID_BT_SCO_WB:
        case QAL_PARAM_ID_BT_SCO:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_SCO;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr , rm);
                status = dev->setDeviceParameter(param_id, param_payload);
            }

            dattr.id = QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr , rm);
                status = dev->setDeviceParameter(param_id, param_payload);
            }
        }
        break;
        case QAL_PARAM_ID_BT_A2DP_RECONFIG:
        {
            std::shared_ptr<Device> dev = nullptr;
            struct qal_device dattr;
            qal_param_bta2dp_t *param_bt_a2dp = nullptr;

            dattr.id = QAL_DEVICE_OUT_BLUETOOTH_A2DP;
            if (isDeviceAvailable(dattr.id)) {
                dev = Device::getInstance(&dattr , rm);
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
                    QAL_DBG(LOG_TAG, "Switching A2DP Device\n");
                    getDeviceConfig(&dattr, NULL);
                    mResourceManagerMutex.unlock();
                    rm->forceDeviceSwitch(dev, &dattr);
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

                sco_tx_dattr.id = QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
                QAL_DBG(LOG_TAG, "a2dp resumed, switch bt sco mic to handset mic");
                if (isDeviceAvailable(sco_tx_dattr.id)) {
                    struct qal_device handset_tx_dattr;
                    std::shared_ptr<Device> sco_tx_dev = nullptr;

                    handset_tx_dattr.id = QAL_DEVICE_IN_HANDSET_MIC;
                    sco_tx_dev = Device::getInstance(&sco_tx_dattr , rm);
                    getDeviceConfig(&handset_tx_dattr, NULL);
                    mResourceManagerMutex.unlock();
                    rm->forceDeviceSwitch(sco_tx_dev, &handset_tx_dattr);
                    mResourceManagerMutex.lock();
                }
                /* TODO : Handle other things in BT class */
            }

            mResourceManagerMutex.unlock();
            status = a2dp_dev->setDeviceParameter(param_id, param_payload);
            mResourceManagerMutex.lock();
            if (status) {
                QAL_ERR(LOG_TAG, "set Parameter %d failed\n", param_id);
                goto exit;
            }
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

#if 0
int ResourceManager::getParameter(uint32_t param_id, void **param_payload,
                                  size_t *payload_size)
{
    int status = 0;

    QAL_INFO(LOG_TAG, "ID:%d", param_id);
    std::lock_guard<std::mutex> lock(mResourceManagerMutex);
    switch (param_id) {
        case QAL_PARAM_ID_DEVICE_CAPABILITY: {
            qal_param_device_capability_t *param_device_capability = (qal_param_device_capability_t *)(*param_payload);
            QAL_INFO(LOG_TAG, "Device %d card = %d qalid=%x",
                        param_device_capability->addr.device_num,
                        param_device_capability->addr.card_id,
                        param_device_capability->id);
            status = getDeviceDefaultCapability(*param_device_capability);
        }
        break;
        default:
            QAL_ERR(LOG_TAG,"Unsupported ParamID:%d", param_id);
            break;
    }

    return status;
}
#endif

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

bool ResourceManager::getScreenState()
{
    return screen_state_;
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
    struct qal_stream_attributes sAttr;
    struct qal_device conn_device;
    std::shared_ptr<Device> dev = nullptr;

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
            status = getDeviceConfig(&dAttr, &sAttr);
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
            status = getDeviceConfig(&dAttr, &sAttr);
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
            if (dAttr.config.ch_info)
                free(dAttr.config.ch_info);
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
        }
            break;
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
    if (id == QAL_DEVICE_OUT_AUX_DIGITAL ||
        id == QAL_DEVICE_OUT_HDMI)
        return true;
    else
        return false;
}
void ResourceManager::processTagInfo(const XML_Char **attr)
{
    int32_t tagId;
    int32_t found = 0;
    char tagChar[128] = {0};
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

void ResourceManager::processDeviceInfo(const XML_Char **attr)
{
    int32_t deviceId;
    int32_t pcmId;
    if(strcmp(attr[0], "name" ) !=0 ) {
        QAL_ERR(LOG_TAG, " 'name' not found");
        return;
    }

    std::string deviceName(attr[1]);
    deviceId = deviceIdLUT.at(deviceName);

    if (strcmp(attr[2],"pcm_id") !=0 ) {
        QAL_ERR(LOG_TAG, " 'pcm_id' not found %s is the tag", attr[2]);
        return;
    }
    pcmId = atoi(attr[3]);
    updatePcmId(deviceId, pcmId);
    if(strcmp(attr[4],"hw_intf") !=0 ) {
        QAL_ERR(LOG_TAG, " 'hw_intf' not found");
        return;
    }
    std::string linkName(attr[5]);
    updateLinkName(deviceId, linkName);

    if(strcmp(attr[6], "snd_device_name") != 0) {
        QAL_ERR(LOG_TAG, " 'snd_device_name' not found");
        return;
    }
    std::string sndName(attr[7]);
    updateSndName(deviceId, sndName);
    const qal_alsa_or_gsl ag = rm->getQALConfigALSAOrGSL();
    if (ag == ALSA) {
        if (strcmp(attr[8], "back_end_name") != 0) {
            QAL_ERR(LOG_TAG, "'back_end_name' not found");
            return;
        }
        std::string backName(attr[9]);
        updateBackEndName(deviceId, backName);
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
    str_parms_add_str(configParamKVPairs, (char*)attr[1], (char*)attr[3]);
    setConfigParams(configParamKVPairs);
done:
    return;
}

void ResourceManager::processCardInfo(struct xml_userdata *data, const XML_Char *tag_name)
{
    int card;
    if (!strcmp(tag_name, "id")) {
        card = atoi(data->data_buf);
        snd_card = card;
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
    if (strcmp(tag_name,"props") == 0)
        return;
    size = devInfo.size() - 1;
    if(strcmp(tag_name,"playback") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].playback = val;
    } else if (strcmp(tag_name,"capture") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].record = val;
    } else if (strcmp(tag_name,"hostless") == 0) {
        val = atoi(data->data_buf);
        devInfo[size].loopback = val;
    }
}

void ResourceManager::snd_reset_data_buf(struct xml_userdata *data)
{
    data->offs = 0;
    data->data_buf[data->offs] = '\0';
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
    }
    else if (data->current_tag == TAG_PLUGIN) {
        //snd_parse_plugin_properties(data, tag_name);
    }
    else if (data->current_tag == TAG_DEVICE) {
        //QAL_ERR(LOG_TAG,"tag %s", (char*)tag_name);
        processDeviceIdProp(data, tag_name);
    }
    else if (data->current_tag == TAG_DEV_PROPS) {
        processDeviceCapability(data, tag_name);
    }
}

void ResourceManager::startTag(void *userdata, const XML_Char *tag_name,
    const XML_Char **attr)
{
    snd_card_defs_xml_tags_t tagId;
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
        processDeviceInfo(attr);
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
    configParamKVPairs = str_parms_create();
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
