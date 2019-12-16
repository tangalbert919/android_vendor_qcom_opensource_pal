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

#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H
#include <algorithm>
#include <vector>
#include <memory>
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include "audio_route/audio_route.h"
#include <tinyalsa/asoundlib.h>
#include "QalCommon.h"
#include <map>
#include <expat.h>
#include "QalDefs.h"
#define RXLOOPBACK 0
#define TXLOOPBACK 1
#define audio_mixer mixer

#define AUDIO_PARAMETER_KEY_NATIVE_AUDIO "audio.nat.codec.enabled"
#define AUDIO_PARAMETER_KEY_NATIVE_AUDIO_MODE "native_audio_mode"

enum qal_alsa_or_gsl {
    ALSA = 0,
    GSL
};

typedef enum {
    TAG_ROOT,
    TAG_CARD,
    TAG_DEVICE,
    TAG_PLUGIN,
    TAG_DEV_PROPS,
    TAG_NONE,
    TAG_MIXER,
} snd_card_defs_xml_tags_t;

typedef enum {
    PCM,
    COMPRESS,
    VOICE1,
    VOICE2,
} stream_supported_type;

struct xml_userdata {
    char data_buf[1024];
    size_t offs;

    unsigned int card;
    bool card_found;
    bool card_parsed;
    snd_card_defs_xml_tags_t current_tag;
    bool is_parsing_sound_trigger;
};

struct deviceCap {
    int deviceId;
    stream_supported_type type;
    int playback;
    int record;
    int loopback;
};

enum {
    NATIVE_AUDIO_MODE_SRC = 1,
    NATIVE_AUDIO_MODE_TRUE_44_1,
    NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_CODEC,
    NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_DSP,
    NATIVE_AUDIO_MODE_INVALID
};

struct nativeAudioProp {
   bool rm_na_prop_enabled;
   bool ui_na_prop_enabled;
   int na_mode;
};

class Device;
class Stream;
class StreamPCM;
class StreamCompress;
class StreamSoundTrigger;
class SoundTriggerEngine;

class ResourceManager
{

private:
    //both of the below are update on register and deregister stream
    int mPriorityHighestPriorityActiveStream; //priority of the highest priority active stream
    Stream* mHighestPriorityActiveStream; //pointer to the highest priority active stream

    int getNumFEs(const qal_stream_type_t sType) const;
    /* shouldDeviceSwitch will return true, if the incoming stream properties and device
     * properties should force a device switch, these are the points to consider
     *
     * order of priority
     *
     *     100 - voice and VOIP call streams have the highest priority (10 points or 0 points),
     *     and device properties should be derived from that
     *
     *     50 - points if the stream is a 44.1 stream. This has the second highest priority
     *
     *     25 - points if the stream is a 24 bit stream. This has the third highest priority
     *
     * const bool shouldDeviceSwitch(const qal_stream_attributes* sExistingAttr,
     * const qal_stream_attributes* sIncomingAttr) const
     */

    bool shouldDeviceSwitch(const qal_stream_attributes* sExistingAttr,
         const qal_stream_attributes* sIncomingAttr) const;
    bool isDeviceSwitchRequired(struct qal_device *activeDevAttr,
         struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr);
    bool ifVoiceorVoipCall (qal_stream_type_t streamType) const;
    int getCallPriority(bool ifVoiceCall) const;
    int getStreamAttrPriority (const qal_stream_attributes* sAttr) const;
    template <class T>

    void getHigherPriorityActiveStreams(const int inComingStreamPriority,
                                        std::vector<Stream*> &activestreams,
                                        std::vector<T> sourcestreams);
    const std::vector<int> allocateVoiceFrontEndIds(std::vector<int> listAllPcmVoiceFrontEnds,
                                  const int howMany);

    int handleScreenStatusChange(qal_param_screen_state_t screen_state);
    int handleDeviceConnectionChange(qal_param_device_connection_t connection_state);

protected:
    std::vector <Stream*> mActiveStreams;
    std::vector <StreamPCM*> active_streams_ll;
    std::vector <StreamPCM*> active_streams_ulla;
    std::vector <StreamPCM*> active_streams_db;
    std::vector <StreamPCM*> active_streams_po;
    std::vector <StreamCompress*> active_streams_comp;
    std::vector <StreamSoundTrigger*> active_streams_st;
    std::vector <SoundTriggerEngine*> active_engines_st;
    std::vector <std::shared_ptr<Device>> active_devices;
    std::vector <qal_device_id_t> avail_devices_;
    bool bOverwriteFlag;
    bool screen_state_;
    static std::mutex mResourceManagerMutex;
    static int snd_card;
    static std::shared_ptr<ResourceManager> rm;
    static struct audio_route* audio_route;
    static struct audio_mixer* audio_mixer;
    static std::vector <int> streamTag;
    static std::vector <int> streamPpTag;
    static std::vector <int> mixerTag;
    static std::vector <int> devicePpTag;
    static std::vector <int> deviceTag;
    static std::vector<std::pair<int32_t, int32_t>> devicePcmId;
    static std::vector<std::pair<int32_t, std::string>> deviceLinkName;
    static std::vector<int> listAllFrontEndIds;
    static std::vector<int> listAllPcmPlaybackFrontEnds;
    static std::vector<int> listAllPcmRecordFrontEnds;
    static std::vector<int> listAllPcmLoopbackRxFrontEnds;
    static std::vector<int> listAllPcmLoopbackTxFrontEnds;
    static std::vector<int> listAllCompressPlaybackFrontEnds;
    static std::vector<int> listAllCompressRecordFrontEnds;
    static std::vector<int> listFreeFrontEndIds;
    static std::vector<int> listAllPcmVoice1RxFrontEnds;
    static std::vector<int> listAllPcmVoice1TxFrontEnds;
    static std::vector<int> listAllPcmVoice2RxFrontEnds;
    static std::vector<int> listAllPcmVoice2TxFrontEnds;
    static std::vector<std::pair<int32_t, std::string>> listAllBackEndIds;
    static std::vector<std::pair<int32_t, std::string>> sndDeviceNameLUT;
    static std::vector<deviceCap> devInfo;
    static std::map<std::pair<uint32_t, std::string>, std::string> btCodecMap;
    static std::map<std::string, uint32_t> btFmtTable;
    ResourceManager();
public:
    ~ResourceManager();
    /* checks config for both stream and device */
    bool isStreamSupported(struct qal_stream_attributes *attributes,
                           struct qal_device *devices, int no_of_devices);
    int32_t getDeviceConfig(struct qal_device *deviceattr,
                            struct qal_stream_attributes *attributes);
    int registerStream(Stream *s);
    int deregisterStream(Stream *s);
    int registerDevice(std::shared_ptr<Device> d);
    int deregisterDevice(std::shared_ptr<Device> d);
    /* bIsUpdated - to specify if the config is updated by rm */
    int checkAndGetDeviceConfig(struct qal_device *device ,bool* bIsUpdated);
    int init_audio();
    static int init();
    static void deinit();
    static std::shared_ptr<ResourceManager> getInstance();
    static int XmlParser(std::string xmlFile);
    static void updatePcmId(int32_t deviceId, int32_t pcmId);
    static void updateLinkName(int32_t deviceId, std::string linkName);
    static void updateSndName(int32_t deviceId, std::string sndName);
    static void updateBackEndName(int32_t deviceId, std::string backEndName);
    static void updateStreamTag(int32_t tagId);
    static void updateDeviceTag(int32_t tagId);
    static void updateBtCodecMap(std::pair<uint32_t, std::string> key, std::string value);
    static std::string getBtCodecLib(uint32_t codecFormat, std::string codecType);

    int setParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size);

    int getParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size);

    int getSndCard();
    int getPcmDeviceId(int deviceId);
    int getAudioRoute(struct audio_route** ar);
    int getAudioMixer(struct audio_mixer **am);
    int getActiveStream(std::shared_ptr<Device> d, std::vector<Stream*> &activestreams);
    int getActiveDevices(std::vector<std::shared_ptr<Device>> &deviceList);
    int getSndDeviceName(int deviceId, char *device_name);
    int getDeviceEpName(int deviceId, std::string &epName);
    int getBackendName(int deviceId, std::string &backendName);
    int getStreamTag(std::vector <int> &tag);
    int getDeviceTag(std::vector <int> &tag);
    int getMixerTag(std::vector <int> &tag);
    int getStreamPpTag(std::vector <int> &tag);
    int getDevicePpTag(std::vector <int> &tag);
    qal_alsa_or_gsl getQALConfigALSAOrGSL() const;
    const std::vector<int> allocateFrontEndIds (const struct qal_stream_attributes,
                                                int lDirection);
    void freeFrontEndIds (const std::vector<int> f,
                          const struct qal_stream_attributes,
                          int lDirection);
    const std::vector<std::string> getBackEndNames(const std::vector<std::shared_ptr<Device>> &deviceList) const;
    void getBackEndNames( const std::vector<std::shared_ptr<Device>> &deviceList,
                          std::vector<std::pair<int32_t, std::string>> &rxBackEndNames,
                          std::vector<std::pair<int32_t, std::string>> &txBackEndNames) const;
    bool updateDeviceConfig(std::shared_ptr<Device> inDev,
             struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr);
    int32_t forceDeviceSwitch(std::shared_ptr<Device> inDev, struct qal_device *newDevAttr);
    const std::string getQALDeviceName(const qal_device_id_t id) const;
    bool isNonALSACodec(const struct qal_device *device) const;
    bool IsVoiceUILPISupported();
    bool CheckForActiveConcurrentNonLPIStream();

    static void endTag(void *userdata __unused, const XML_Char *tag_name);
    static void snd_reset_data_buf(struct xml_userdata *data);
    static void snd_process_data_buf(struct xml_userdata *data, const XML_Char *tag_name);
    static void processCardInfo(struct xml_userdata *data, const XML_Char *tag_name);
    static void processDeviceInfo(const XML_Char **attr);
    static void processTagInfo(const XML_Char **attr);
    static void processBTCodecInfo(const XML_Char **attr);
    static void startTag(void *userdata __unused, const XML_Char *tag_name, const XML_Char **attr);
    static void snd_data_handler(void *userdata, const XML_Char *s, int len);
    static void processDeviceIdProp(struct xml_userdata *data, const XML_Char *tag_name);
    static void processDeviceCapability(struct xml_userdata *data, const XML_Char *tag_name);
    static int getNativeAudioSupport();
    static int setNativeAudioSupport(int na_mode);
    static void getNativeAudioParams(struct str_parms *query,struct str_parms *reply,char *value, int len);
    static int setConfigParams(struct str_parms *parms);
    static int setNativeAudioParams(struct str_parms *parms,char *value, int len);
    static void processConfigParams(const XML_Char **attr);
    bool getScreenState();
    bool isDeviceAvailable(qal_device_id_t id);
    bool isDeviceReady(qal_device_id_t id);
    static bool isBtScoDevice(qal_device_id_t id);
    int32_t a2dpSuspend();
    int32_t a2dpResume();
};

#endif
