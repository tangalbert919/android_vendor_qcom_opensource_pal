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


#ifndef SOUNDTRIGGERENGINE_H
#define SOUNDTRIGGERENGINE_H

#include "QalDefs.h"
#include <mutex>
#include <vector>
#include "QalCommon.h"
#include "QalRingBuffer.h"

// TODO: Move to sound trigger xml files
#define CNN_SAMPLE_RATE 16000
#define CNN_BITWIDTH 16
#define CNN_CHANNELS 1
#define BITS_PER_BYTE 8
#define US_PER_SEC 1000000
#define CNN_DURATION_US 2500000
#define CNN_BUFFER_SIZE 15360
#define RING_BUFFER_DURATION 3

class Session;
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
    uint8_t level;
};

struct __attribute__((__packed__)) st_keyword_levels
{
    uint8_t kw_level;
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
    uint8_t mode; /* 0 -Low Power, 1 -High performance */
};


typedef enum {
    IDLE,
    READY,
    ACTIVE,
    BUFFERING,
    HOLD_READY,
    HOLD_ACTIVE,
} sound_model_state_t;

/* Structure representing a single sound model,
   sound model data and recognition data of all
   sound models registered merged to single blob
   of sound model data and recogntion data, which
   is set to Session. */
struct SoundModel
{
    sound_model_state_t state;
    uint32_t sm_id;
    uint32_t sm_host_id;
    void *sm_data;
    void *sm_params_data;
};

class SoundTriggerEngine
{
protected:
    uint32_t engineId;
    uint8_t *sm_data;
    uint32_t sm_data_size;
    uint8_t *sm_params_data;
    int stageId;
    Session *session;
    Stream *streamHandle;
    std::vector<struct SoundModel*> SoundModels;
    QalRingBuffer *buffer_;
    QalRingBufferReader *reader_;
    bool eventDetected;
    std::mutex mutex;
public:
    static SoundTriggerEngine* create(Stream *s, listen_model_indicator_enum type,
           QalRingBufferReader **reader, std::shared_ptr<QalRingBuffer> buffer);
    virtual ~SoundTriggerEngine() {};
    virtual int32_t load_sound_model(Stream *s, uint8_t *data, uint32_t num_models) = 0;
    virtual int32_t unload_sound_model(Stream *s) = 0;
    virtual int32_t start_recognition(Stream *s) = 0;
    virtual int32_t stop_recognition(Stream *s) = 0;
    virtual int32_t update_config(Stream *s, struct qal_st_recognition_config *config) = 0;
    virtual void setDetected(bool detected) = 0;
    virtual int32_t getParameters(uint32_t param_id, void **payload) = 0;
};

#endif //SOUNDTRIGGERENGINE_H
