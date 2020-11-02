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


#ifndef SOUNDTRIGGERENGINE_H
#define SOUNDTRIGGERENGINE_H

#include <condition_variable>
#include <thread>
#include <mutex>
#include <vector>

#include "PalDefs.h"
#include "PalCommon.h"
#include "PalRingBuffer.h"
#include "Device.h"

// TODO: Move to sound trigger xml files
#define BITS_PER_BYTE 8
#define US_PER_SEC 1000000
#define CNN_BUFFER_SIZE 15360
#define CNN_FRAME_SIZE 320
#define CUSTOM_CONFIG_OPAQUE_DATA_SIZE 12
#define CONF_LEVELS_INTF_VERSION_0002 0x02

class Stream;

enum {
    SML_PARSER_SUCCESS = 0,
    SML_PARSER_ERROR = 1,
    SML_PARSER_NOT_SUPPORTED_VERSION,
    SML_PARSER_ID_NOT_EXIST,
    SML_PARSER_NOT_ALIGNED_SIZE,
    SML_PARSER_SIZE_MISSMATCH,
    SML_PARSER_VALUE_MISSMATCH,
    SML_PARSER_REF_UNINIT_VALUE,
    SML_PARSER_OUT_OF_RANGE,
    SML_PARSER_WRONG_MODEL,
    SML_PARSER_WRONG_VALUE,
    SML_PARSER_DELETING_LAST_KEYWORD,
    SML_PARSER_KEYWORD_NOT_FOUND,
    SML_PARSER_USER_NOT_FOUND,
    SML_PARSER_USER_ALREADY_EXIST,
    SML_PARSER_KEYWORD_USER_PAIR_EXIST,
    SML_PARSER_KEYWORD_USER_PAIR_NOT_EXIST,
    SML_PARSER_ACTIVE_USER_REMOVE_UDK,
    SML_PARSER_KEYWORD_ALREADY_EXIST,
    SML_PARSER_NOTHING_TO_CHANGE,
};

enum {
    SML_COMPARATOR_ERROR = -1,
    SML_COMPARATOR_SAME = 0,
    SML_COMPARATOR_DIFF = 1,
};

enum {
    SML_V3_MAX_PAYLOAD = 10000000,
    SML_V3_MIN_KEYWORDS = 1,
    SML_V3_MAX_KEYWORDS = 1,
    SML_V3_MIN_USERS    = 0,
    SML_V3_MAX_USERS    = 1,
};

enum {
    SML_GLOBAL_HEADER_MAGIC_NUMBER = 0x00180cc8,    // SML03
    SML_MAX_MODEL_NUM = 3,
    SML_MAX_STRING_LEN = 200,
};

typedef enum {
    ST_SM_ID_SVA_NONE     = 0x0000,
    ST_SM_ID_SVA_GMM      = 0x0001,
    ST_SM_ID_SVA_CNN      = 0x0002,
    ST_SM_ID_SVA_VOP      = 0x0004,
    ST_SM_ID_SVA_RNN      = 0x0008,
    ST_SM_ID_SVA_KWD      = 0x000A,            // ST_SM_ID_SVA_CNN | ST_SM_ID_SVA_RNN
    SML_ID_SVA_S_STAGE_UBM = 0x0010,
    ST_SM_ID_SVA_END      = 0x00F0,
    ST_SM_ID_CUSTOM_START = 0x0100,
    ST_SM_ID_CUSTOM_END   = 0xF000,
} listen_model_indicator_enum;

typedef struct _SML_GlobalHeaderType {
    uint32_t    magicNumber;                    // Magic number
    uint32_t    payloadBytes;                   // Payload bytes size
    uint32_t    modelVersion;                   // Model version
} SML_GlobalHeaderType;

typedef struct _SML_HeaderTypeV3 {
    uint32_t    numModels;                      // number of models
    uint32_t    keywordSpellLength;             // length of keyword spell ( include '\0' )
    uint32_t    userNameLength;                 // length of user name ( include '\0' )
    char    keywordSpell[SML_MAX_STRING_LEN];   // keyword spell
    char    userName[SML_MAX_STRING_LEN];       // user name
} SML_HeaderTypeV3;

typedef struct _SML_BigSoundModelTypeV3 {
    uint32_t version;                           // version of sound model ( always 3 for now )
    uint32_t offset;                            // start address of model data
    uint32_t size;                              // model size
    listen_model_indicator_enum type;           // type : Lower 1 byte : 1Stage KW model,
                                                //                       2Stage KW model,
                                                //                       2Stage User Model
                                                //        Upper 1 byte : 3rd Party - 1Stage KW model,
                                                //                       2Stage KW model,
                                                //                       2Stage User Model
}SML_BigSoundModelTypeV3;


typedef struct _SML_ModelTypeV3 {
    SML_HeaderTypeV3 header;                    // header for v3 model
    SML_BigSoundModelTypeV3 *modelInfo;         // model info, used for dynamic memory allocation.
    uint8_t* rawData;                           // actual model data
} SML_ModelTypeV3;

// all of model versions about SML
typedef enum _SML_ModelVersion {
    SML_MODEL_V2 = 0x0200,
    SML_MODEL_V3 = 0x0300,
} SML_ModelVersion;

// universial SML model structure
typedef struct _SML_ModelType {
    SML_GlobalHeaderType header;                // global header

    union _sml_model {
         SML_ModelTypeV3 SMLModelV3;             // sml3.0 -- kwihyuk
    } SMLModel;
} SML_ModelType;

#define ST_MAX_SOUND_MODELS 10
#define ST_MAX_KEYWORDS 10
#define ST_MAX_USERS 10

enum st_param_key {
    ST_PARAM_KEY_CONFIDENCE_LEVELS,
    ST_PARAM_KEY_HISTORY_BUFFER_CONFIG,
    ST_PARAM_KEY_KEYWORD_INDICES,
    ST_PARAM_KEY_TIMESTAMP,
    ST_PARAM_KEY_DETECTION_PERF_MODE,
};

typedef enum st_param_key st_param_key_t;

struct __attribute__((__packed__)) st_param_header
{
    st_param_key_t key_id;
    uint32_t payload_size;
};

struct __attribute__((__packed__)) st_user_levels
{
    uint32_t user_id;
    uint32_t level;
};

struct __attribute__((__packed__)) st_keyword_levels
{
    uint32_t kw_level;
    uint32_t num_user_levels;
    struct st_user_levels user_levels[ST_MAX_USERS];
};

struct __attribute__((__packed__)) st_sound_model_conf_levels
{
    listen_model_indicator_enum sm_id;
    uint32_t num_kw_levels;
    struct st_keyword_levels kw_levels[ST_MAX_KEYWORDS];
};

struct __attribute__((__packed__)) st_confidence_levels_info
{
    uint32_t version; /* value: 0x1 */
    uint32_t num_sound_models;
    struct st_sound_model_conf_levels conf_levels[ST_MAX_SOUND_MODELS];
};

struct __attribute__((__packed__)) st_user_levels_v2
{
    uint32_t user_id;
    int32_t level;
};

struct __attribute__((__packed__)) st_keyword_levels_v2
{
    int32_t kw_level;
    uint32_t num_user_levels;
    struct st_user_levels_v2 user_levels[ST_MAX_USERS];
};

struct __attribute__((__packed__)) st_sound_model_conf_levels_v2
{
    listen_model_indicator_enum sm_id;
    uint32_t num_kw_levels;
    struct st_keyword_levels_v2 kw_levels[ST_MAX_KEYWORDS];
};

struct __attribute__((__packed__)) st_confidence_levels_info_v2
{
    uint32_t version; /* value: 0x02 */
    uint32_t num_sound_models;
    struct st_sound_model_conf_levels_v2 conf_levels[ST_MAX_SOUND_MODELS];
};

struct __attribute__((__packed__)) st_hist_buffer_info
{
    uint32_t version; /* value: 0x02 */
    uint32_t hist_buffer_duration_msec;
    uint32_t pre_roll_duration_msec;
};

struct __attribute__((__packed__)) st_keyword_indices_info
{
    uint32_t version; /* value: 0x01 */
    uint32_t start_index; /* in bytes */
    uint32_t end_index;   /* in bytes */
};

struct __attribute__((__packed__)) st_timestamp_info
{
    uint32_t version; /* value: 0x01 */
    uint64_t first_stage_det_event_time;  /* in nanoseconds */
    uint64_t second_stage_det_event_time; /* in nanoseconds */
};

struct __attribute__((__packed__)) st_det_perf_mode_info
{
    uint32_t version; /* value: 0x01 */
    uint32_t mode; /* 0 -Low Power, 1 -High performance */
};

typedef enum st_sound_model_type {
    ST_SM_TYPE_NONE,
    ST_SM_TYPE_KEYWORD_DETECTION,
    ST_SM_TYPE_USER_VERIFICATION,
    ST_SM_TYPE_CUSTOM_DETECTION,
    ST_SM_TYPE_MAX
}st_sound_model_type_t;

typedef enum st_param_id_type {
    LOAD_SOUND_MODEL = 0,
    UNLOAD_SOUND_MODEL,
    WAKEUP_CONFIG,
    BUFFERING_CONFIG,
    ENGINE_RESET,
    MODULE_VERSION,
    CUSTOM_CONFIG,
    MAX_PARAM_IDS
} st_param_id_type_t;

struct detection_event_info
{
    uint16_t status;
    uint16_t num_confidence_levels;
    uint8_t confidence_levels[20];
    uint32_t kw_start_timestamp_lsw;
    uint32_t kw_start_timestamp_msw;
    uint32_t kw_end_timestamp_lsw;
    uint32_t kw_end_timestamp_msw;
    uint32_t detection_timestamp_lsw;
    uint32_t detection_timestamp_msw;
    uint32_t ftrt_data_length_in_us;
};

class SoundTriggerEngine {
public:
    static std::shared_ptr<SoundTriggerEngine> Create(Stream *s,
        listen_model_indicator_enum type);

    virtual ~SoundTriggerEngine() {}

    virtual int32_t LoadSoundModel(Stream *s, uint8_t *data,
                                   uint32_t data_size) = 0;
    virtual int32_t UnloadSoundModel(Stream *s) = 0;
    virtual int32_t StartRecognition(Stream *s) = 0;
    virtual int32_t RestartRecognition(Stream *s) = 0;
    virtual int32_t StopRecognition(Stream *s) = 0;
    virtual int32_t UpdateConfLevels(
        Stream *s,
        struct pal_st_recognition_config *config,
        uint8_t *conf_levels,
        uint32_t num_conf_levels) = 0;
    virtual int32_t UpdateBufConfig(uint32_t hist_buffer_duration,
                                    uint32_t pre_roll_duration) = 0;
    virtual void SetDetected(bool detected) = 0;
    virtual int32_t GetParameters(uint32_t param_id, void **payload) = 0;
    virtual int32_t ConnectSessionDevice(
        Stream* stream_handle,
        pal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_connect) = 0;
    virtual int32_t DisconnectSessionDevice(
        Stream* stream_handle,
        pal_stream_type_t stream_type,
        std::shared_ptr<Device> device_to_disconnect) = 0;
    virtual int32_t SetupSessionDevice(
        Stream* streamHandle,
        pal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect) = 0;
    virtual void SetCaptureRequested(bool is_requested) = 0;
    virtual struct detection_event_info* GetDetectionEventInfo() = 0;
    virtual int32_t setECRef(
        Stream *s,
        std::shared_ptr<Device> dev,
        bool is_enable) = 0;
    virtual int32_t GetCustomDetectionEvent(uint8_t **event __unused,
        size_t *size __unused) { return 0; }

    int32_t CreateBuffer(uint32_t buffer_size, uint32_t engine_size,
        std::vector<PalRingBufferReader *> &reader_list);
    int32_t SetBufferReader(PalRingBufferReader *reader);
    uint32_t UsToBytes(uint64_t input_us);

protected:
    uint32_t engine_id_;
    listen_model_indicator_enum engine_type_;
    uint8_t *sm_data_;
    uint32_t sm_data_size_;
    bool capture_requested_;
    Stream *stream_handle_;
    PalRingBuffer *buffer_;
    PalRingBufferReader *reader_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t channels_;

    std::thread buffer_thread_handler_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool exit_thread_;
    bool exit_buffering_;
};

#endif  // SOUNDTRIGGERENGINE_H
