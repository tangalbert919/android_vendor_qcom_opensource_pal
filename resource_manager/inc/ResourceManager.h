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
#include <stdio.h>
#include <queue>
#include <deque>
#include "QalDefs.h"
#include "SndCardMonitor.h"
#include "SoundTriggerPlatformInfo.h"
#define RXLOOPBACK 0
#define TXLOOPBACK 1
#define audio_mixer mixer

#define AUDIO_PARAMETER_KEY_NATIVE_AUDIO "audio.nat.codec.enabled"
#define AUDIO_PARAMETER_KEY_NATIVE_AUDIO_MODE "native_audio_mode"
#define AUDIO_PARAMETER_KEY_MAX_SESSIONS "max_sessions"
#define AUDIO_PARAMETER_KEY_LOG_LEVEL "logging_level"
#define MAX_PCM_NAME_SIZE 50
#if LINUX_ENABLED
#if defined(__LP64__)
#define ADM_LIBRARY_PATH "/usr/lib64/libadm.so"
#else
#define ADM_LIBRARY_PATH "/usr/lib/libadm.so"
#endif
#else
#define ADM_LIBRARY_PATH "/vendor/lib/libadm.so"
#endif

using InstanceListNode_t = std::vector<std::pair<int32_t, bool>> ;

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
    TAG_RESOURCE_ROOT,
    TAG_RESOURCE_MANAGER_INFO,
    TAG_DEVICE_PROFILE,
    TAG_IN_DEVICE,
    TAG_OUT_DEVICE,
    TAG_USECASE,
    TAG_DEVICEPP,
    TAG_KVPAIR,
    TAG_CONFIG_VOICE,
    TAG_CONFIG_MODE_MAP,
    TAG_CONFIG_MODE_PAIR,
    TAG_INSTREAMS,
    TAG_INSTREAM,
    TAG_POLICIES,
    TAG_ECREF,
} resource_xml_tags_t;

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
    bool resourcexml_parsed;
    bool voice_info_parsed;
    snd_card_defs_xml_tags_t current_tag;
    bool is_parsing_sound_trigger;
    resource_xml_tags_t tag;
};
typedef enum {
    DEFAULT = 0,
    HOSTLESS,
    NON_TUNNEL,
} sess_mode_t;

struct deviceCap {
    int deviceId;
    char name[MAX_PCM_NAME_SIZE];
    stream_supported_type type;
    int playback;
    int record;
    sess_mode_t sess_mode;
};

struct kvpair_info {
    unsigned int key;
    unsigned int value;
};

typedef enum {
    SIDETONE_OFF,
    SIDETONE_HW,
    SIDETONE_SW,
} sidetone_mode_t;

struct usecase_info {
    int type;
    std::vector<kvpair_info> kvpair;
    sidetone_mode_t sidetoneMode;
};

struct deviceIn {
    int deviceId;
    int max_channel;
    int channel;
    std::vector<usecase_info> usecase;
};

struct qal_device_info {
     int channels;
     int max_channels;
     std::vector<kvpair_info> kvpair;
};

struct vsid_modepair {
    unsigned int key;
    unsigned int value;
};

struct vsid_info {
     int vsid;
     std::vector<vsid_modepair> modepair;
};

struct tx_ecinfo {
    int tx_stream_type;
    std::vector<int> disabled_rx_streams;
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

typedef void (*session_callback)(void *hdl, uint32_t event_id, void *event_data,
                                   uint32_t event_size);

typedef void* (*adm_init_t)();
typedef void (*adm_deinit_t)(void *);
typedef void (*adm_register_output_stream_t)(void *, void*);
typedef void (*adm_register_input_stream_t)(void *, void*);
typedef void (*adm_deregister_stream_t)(void *, void*);
typedef void (*adm_request_focus_t)(void *, void*);
typedef void (*adm_abandon_focus_t)(void *, void*);
typedef void (*adm_set_config_t)(void *, void*,
        struct pcm *, struct pcm_config *);
typedef void (*adm_request_focus_v2_t)(void *, void*, long);
typedef void (*adm_on_routing_change_t)(void *, void*);
typedef int (*adm_request_focus_v2_1_t)(void *, void*, long);

class Device;
class Stream;
class StreamPCM;
class StreamCompress;
class StreamSoundTrigger;
class StreamInCall;
class SoundTriggerEngine;
class SndCardMonitor;

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
    bool ifVoiceorVoipCall (qal_stream_type_t streamType) const;
    int getCallPriority(bool ifVoiceCall) const;
    int getStreamAttrPriority (const qal_stream_attributes* sAttr) const;
    template <class T>

    void getHigherPriorityActiveStreams(const int inComingStreamPriority,
                                        std::vector<Stream*> &activestreams,
                                        std::vector<T> sourcestreams);
    const std::vector<int> allocateVoiceFrontEndIds(std::vector<int> listAllPcmVoiceFrontEnds,
                                  const int howMany);
    int getDeviceDefaultCapability(qal_param_device_capability_t capability);

    int handleScreenStatusChange(qal_param_screen_state_t screen_state);
    int handleDeviceRotationChange(qal_param_device_rotation_t rotation_type);
    int handleDeviceConnectionChange(qal_param_device_connection_t connection_state);
    int32_t streamDevDisconnect(std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnectList);
    int32_t streamDevConnect(std::vector <std::tuple<Stream *, struct qal_device *>> streamDevConnectList);
    void ssrHandlingLoop(std::shared_ptr<ResourceManager> rm);

protected:
    std::vector <Stream*> mActiveStreams;
    std::vector <StreamPCM*> active_streams_ll;
    std::vector <StreamPCM*> active_streams_ulla;
    std::vector <StreamPCM*> active_streams_ull;
    std::vector <StreamPCM*> active_streams_db;
    std::vector <StreamPCM*> active_streams_po;
    std::vector <StreamPCM*> active_streams_proxy;
    std::vector <StreamInCall*> active_streams_incall_record;
    std::vector <StreamInCall*> active_streams_incall_music;
    std::vector <StreamCompress*> active_streams_comp;
    std::vector <StreamSoundTrigger*> active_streams_st;
    std::vector <SoundTriggerEngine*> active_engines_st;
    std::vector <std::pair<std::shared_ptr<Device>, Stream*>> active_devices;
    std::vector <std::shared_ptr<Device>> plugin_devices_;
    std::vector <qal_device_id_t> avail_devices_;
    bool bOverwriteFlag;
    bool screen_state_;
    bool charging_state_;
    qal_speaker_rotation_type rotation_type_;
    static std::mutex mResourceManagerMutex;
    static std::mutex mGraphMutex;
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
    static std::vector<int> listAllPcmInCallRecordFrontEnds;
    static std::vector<int> listAllPcmInCallMusicFrontEnds;
    static std::vector<std::pair<int32_t, std::string>> listAllBackEndIds;
    static std::vector<std::pair<int32_t, std::string>> sndDeviceNameLUT;
    static std::vector<deviceCap> devInfo;
    static std::map<std::pair<uint32_t, std::string>, std::string> btCodecMap;
    static std::map<std::string, uint32_t> btFmtTable;
    static std::vector<deviceIn> deviceInfo;
    static std::vector<tx_ecinfo> txEcInfo;
    static struct vsid_info vsidInfo;
    static SndCardMonitor *sndmon;
    /* condition variable for which ssrHandlerLoop will wait */
    static std::condition_variable cv;
    static std::mutex cvMutex;
    static std::queue<card_status_t> msgQ;
    static std::thread workerThread;
    std::vector<std::pair<int32_t, InstanceListNode_t>> STInstancesLists;
    static int mixerEventRegisterCount;
    static int concurrentRxStreamCount;
    static int concurrentTxStreamCount;
    std::map<int, std::pair<session_callback, void *>> mixerEventCallbackMap;
    std::thread mixerEventTread;
    std::shared_ptr<CaptureProfile> SVACaptureProfile;
    ResourceManager();
public:
    ~ResourceManager();
    enum card_status_t cardState;
    bool ssrStarted = false;
    static bool isSpeakerProtectionEnabled;
    static bool isRasEnabled;
    static int spQuickCalTime;
    qal_spkr_prot_payload mSpkrProtModeValue;
    qal_global_callback globalCb = NULL;
    void *cookie;
    int initSndMonitor();
    adm_init_t admInitFn = NULL;
    adm_deinit_t admDeInitFn = NULL;
    adm_register_output_stream_t admRegisterOutputStreamFn = NULL;
    adm_register_input_stream_t admRegisterInputStreamFn = NULL;
    adm_deregister_stream_t admDeregisterStreamFn = NULL;
    adm_request_focus_t admRequestFocusFn = NULL;
    adm_abandon_focus_t admAbandonFocusFn = NULL;
    adm_set_config_t admSetConfigFn = NULL;
    adm_request_focus_v2_t admRequestFocusV2Fn = NULL;
    adm_on_routing_change_t admOnRoutingChangeFn = NULL;
    adm_request_focus_v2_1_t  admRequestFocus_v2_1Fn = NULL;
    void *admData = NULL;
    void *admLibHdl = NULL;

    /* checks config for both stream and device */
    bool isStreamSupported(struct qal_stream_attributes *attributes,
                           struct qal_device *devices, int no_of_devices);
    int32_t getDeviceConfig(struct qal_device *deviceattr,
                            struct qal_stream_attributes *attributes, int32_t channel);
    /*getDeviceInfo - updates channels, fluence info of the device*/
    void  getDeviceInfo(qal_device_id_t deviceId, qal_stream_type_t type,
                       struct qal_device_info *devinfo);
    bool getEcRefStatus(qal_stream_type_t tx_streamtype,qal_stream_type_t rx_streamtype);
    int32_t getVsidInfo(struct vsid_info  *info);
    void getChannelMap(uint8_t *channel_map, int channels);
    int registerStream(Stream *s);
    int deregisterStream(Stream *s);
    int registerDevice(std::shared_ptr<Device> d, Stream *s);
    int deregisterDevice(std::shared_ptr<Device> d, Stream *s);
    int registerDevice_l(std::shared_ptr<Device> d, Stream *s);
    int deregisterDevice_l(std::shared_ptr<Device> d, Stream *s);
    int registerMixerEventCallback(const std::vector<int> &DevIds,
                                   session_callback callback,
                                   void *cookie, bool is_register);
    bool isDeviceActive(std::shared_ptr<Device> d, Stream *s);
    bool isDeviceActive_l(std::shared_ptr<Device> d, Stream *s);
    int addPlugInDevice(std::shared_ptr<Device> d,
                        qal_param_device_connection_t connection_state);
    int removePlugInDevice(qal_device_id_t device_id,
                           qal_param_device_connection_t connection_state);
    /* bIsUpdated - to specify if the config is updated by rm */
    int checkAndGetDeviceConfig(struct qal_device *device ,bool* bIsUpdated);
    void split_snd_card(const char* in_snd_card_name);
    int init_audio();
    void loadAdmLib();
    static int init();
    static void deinit();
    static std::shared_ptr<ResourceManager> getInstance();
    static int XmlParser(std::string xmlFile);
    static void updatePcmId(int32_t deviceId, int32_t pcmId);
    static void updateLinkName(int32_t deviceId, std::string linkName);
    static void updateSndName(int32_t deviceId, std::string sndName);
    static void updateBackEndName(int32_t deviceId, std::string backEndName);
    static void updateBtCodecMap(std::pair<uint32_t, std::string> key, std::string value);
    static std::string getBtCodecLib(uint32_t codecFormat, std::string codecType);

    int setParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size);
    int setParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size, qal_device_id_t qal_device_id,
                     qal_stream_type_t qal_stream_type);
    int getParameter(uint32_t param_id, void **param_payload,
                     size_t *payload_size, void *query = nullptr);
    int getParameter(uint32_t param_id, void *param_payload,
                     size_t payload_size, qal_device_id_t qal_device_id,
                     qal_stream_type_t qal_stream_type);
    int getSndCard();
    int getPcmDeviceId(int deviceId);
    int getAudioRoute(struct audio_route** ar);
    int getAudioMixer(struct audio_mixer **am);
    int getActiveStream(std::shared_ptr<Device> d, std::vector<Stream*> &activestreams);
    int getActiveStream_l(std::shared_ptr<Device> d, std::vector<Stream*> &activestreams);
    int getActiveDevices(std::vector<std::shared_ptr<Device>> &deviceList);
    int getSndDeviceName(int deviceId, char *device_name);
    int getDeviceEpName(int deviceId, std::string &epName);
    int getBackendName(int deviceId, std::string &backendName);
    int getStreamTag(std::vector <int> &tag);
    int getDeviceTag(std::vector <int> &tag);
    int getMixerTag(std::vector <int> &tag);
    int getStreamPpTag(std::vector <int> &tag);
    int getDevicePpTag(std::vector <int> &tag);
    const std::vector<int> allocateFrontEndIds (const struct qal_stream_attributes,
                                                int lDirection);
    void freeFrontEndIds (const std::vector<int> f,
                          const struct qal_stream_attributes,
                          int lDirection);
    const std::vector<std::string> getBackEndNames(const std::vector<std::shared_ptr<Device>> &deviceList) const;
    void getSharedBEDevices(std::vector<std::shared_ptr<Device>> &deviceList, std::shared_ptr<Device> inDevice) const;
    void getBackEndNames( const std::vector<std::shared_ptr<Device>> &deviceList,
                          std::vector<std::pair<int32_t, std::string>> &rxBackEndNames,
                          std::vector<std::pair<int32_t, std::string>> &txBackEndNames) const;
    bool updateDeviceConfig(std::shared_ptr<Device> inDev,
             struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr);
    int32_t forceDeviceSwitch(std::shared_ptr<Device> inDev, struct qal_device *newDevAttr);
    const std::string getQALDeviceName(const qal_device_id_t id) const;
    bool isNonALSACodec(const struct qal_device *device) const;
    bool IsVoiceUILPISupported();
    bool IsAudioCaptureAndVoiceUIConcurrencySupported();
    bool IsVoiceCallAndVoiceUIConcurrencySupported();
    bool IsVoipAndVoiceUIConcurrencySupported();
    bool IsTransitToNonLPIOnChargingSupported();
    bool CheckForActiveConcurrentNonLPIStream();
    bool GetChargingState() const { return charging_state_; }
    bool CheckForForcedTransitToNonLPI();
    void GetVoiceUIProperties(struct qal_st_properties *qstp);
    std::shared_ptr<CaptureProfile> GetCaptureProfileByPriority(
        StreamSoundTrigger *s);
    bool UpdateSVACaptureProfile(StreamSoundTrigger *s, bool is_active);
    std::shared_ptr<CaptureProfile> GetSVACaptureProfile();
    int SwitchSVADevices(bool connect_state, qal_device_id_t device_id);
    static void mixerEventWaitThreadLoop(std::shared_ptr<ResourceManager> rm);
    bool isCallbackRegistered() { return (mixerEventRegisterCount > 0); }
    int handleMixerEvent(struct mixer *mixer, char *mixer_str);
    int StopOtherSVAStreams(StreamSoundTrigger *st);
    int StartOtherSVAStreams(StreamSoundTrigger *st);
    void ConcurrentStreamStatus(qal_stream_type_t type,
                                qal_stream_direction_t dir,
                                bool active);
    std::shared_ptr<Device> getActiveEchoReferenceRxDevices(Stream *tx_str);
    std::shared_ptr<Device> getActiveEchoReferenceRxDevices_l(Stream *tx_str);
    std::vector<Stream*> getConcurrentTxStream(
        Stream *rx_str, std::shared_ptr<Device> rx_device);
    std::vector<Stream*> getConcurrentTxStream_l(
        Stream *rx_str, std::shared_ptr<Device> rx_device);
    bool checkECRef(std::shared_ptr<Device> rx_dev,
                    std::shared_ptr<Device> tx_dev);

    static void endTag(void *userdata __unused, const XML_Char *tag_name);
    static void snd_reset_data_buf(struct xml_userdata *data);
    static void snd_process_data_buf(struct xml_userdata *data, const XML_Char *tag_name);
    static void process_device_info(struct xml_userdata *data, const XML_Char *tag_name);
    static void process_input_streams(struct xml_userdata *data, const XML_Char *tag_name);
    static void process_config_voice(struct xml_userdata *data, const XML_Char *tag_name);
    static void process_kvinfo(const XML_Char **attr);
    static void process_voicemode_info(const XML_Char **attr);
    static void processCardInfo(struct xml_userdata *data, const XML_Char *tag_name);
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
    static int setLoggingLevelParams(struct str_parms *parms,char *value, int len);
    static void processConfigParams(const XML_Char **attr);
    static bool isValidDevId(int deviceId);
    static bool isOutputDevId(int deviceId);
    static bool isInputDevId(int deviceId);
    static bool matchDevDir(int devId1, int devId2);
    bool getScreenState();
    bool isDeviceAvailable(qal_device_id_t id);
    bool isDeviceReady(qal_device_id_t id);
    static bool isBtScoDevice(qal_device_id_t id);
    int32_t a2dpSuspend();
    int32_t a2dpResume();
    bool isPluginDevice(qal_device_id_t id);
    bool isDeviceSwitchRequired(struct qal_device *activeDevAttr,
         struct qal_device *inDevAttr, const qal_stream_attributes* inStrAttr);
    bool isDpDevice(qal_device_id_t id);
    void lockGraph() { mGraphMutex.lock(); };
    void unlockGraph() { mGraphMutex.unlock(); };
    void getSharedBEActiveStreamDevs(std::vector <std::tuple<Stream *, uint32_t>> &activeStreamDevs,
                                     int dev_id);
    int32_t streamDevSwitch(std::vector <std::tuple<Stream *, uint32_t>> streamDevDisconnectList,
                            std::vector <std::tuple<Stream *, struct qal_device *>> streamDevConnectList);
    char* getDeviceNameFromID(uint32_t id);
    int getQalValueFromGKV(qal_key_vector_t *gkv, int key);
    qal_speaker_rotation_type getCurrentRotationType();
    void ssrHandler(card_status_t state);
    int32_t getSidetoneMode(qal_device_id_t deviceId, qal_stream_type_t type,
                            sidetone_mode_t *mode);
    int getStreamInstanceID(Stream *str);
    int resetStreamInstanceID(Stream *str, uint32_t sInstanceID);
};

#endif
