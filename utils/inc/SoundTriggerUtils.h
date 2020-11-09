/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
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

#ifndef SOUND_TRIGGER_UTILS_H
#define SOUND_TRIGGER_UTILS_H

#include "ListenSoundModelLib.h"

#define SML_LIB "liblistensoundmodel2.so"
#define MAX_KW_USERS_NAME_LEN (2 * MAX_STRING_LEN)
#define MAX_CONF_LEVEL_VALUE 100

/* Listen Sound Model Library APIs */
typedef listen_status_enum (*smlib_getSoundModelHeader_t)
(
    listen_model_type         *pSoundModel,
    listen_sound_model_header *pListenSoundModelHeader
);

typedef listen_status_enum (*smlib_releaseSoundModelHeader_t)
(
    listen_sound_model_header *pListenSoundModelHeader
);

typedef listen_status_enum (*smlib_getKeywordPhrases_t)
(
    listen_model_type *pSoundModel,
    uint16_t          *numKeywords,
    keywordId_t       *keywords
);

typedef listen_status_enum (*smlib_getUserNames_t)
(
    listen_model_type *pSoundModel,
    uint16_t          *numUsers,
    userId_t          *users
);

typedef listen_status_enum (*smlib_getMergedModelSize_t)
(
     uint16_t          numModels,
     listen_model_type *pModels[],
     uint32_t          *nOutputModelSize
);

typedef listen_status_enum (*smlib_mergeModels_t)
(
     uint16_t          numModels,
     listen_model_type *pModels[],
     listen_model_type *pMergedModel
);

typedef listen_status_enum (*smlib_getSizeAfterDeleting_t)
(
    listen_model_type *pInputModel,
    keywordId_t       keywordId,
    userId_t          userId,
    uint32_t          *nOutputModelSize
);

typedef listen_status_enum (*smlib_deleteFromModel_t)
(
    listen_model_type *pInputModel,
    keywordId_t       keywordId,
    userId_t          userId,
    listen_model_type *pResultModel
);

class SoundModelLib {
 public:
    static std::shared_ptr<SoundModelLib> GetInstance();
    SoundModelLib & operator=(SoundModelLib &rhs) = delete;
    SoundModelLib();
    ~SoundModelLib();
    smlib_getSoundModelHeader_t GetSoundModelHeader_;
    smlib_releaseSoundModelHeader_t ReleaseSoundModelHeader_;
    smlib_getKeywordPhrases_t GetKeywordPhrases_;
    smlib_getUserNames_t GetUserNames_;
    smlib_getMergedModelSize_t GetMergedModelSize_;
    smlib_mergeModels_t MergeModels_;
    smlib_getSizeAfterDeleting_t GetSizeAfterDeleting_;
    smlib_deleteFromModel_t DeleteFromModel_;

 private:
    static std::shared_ptr<SoundModelLib> sml_;
    void *sml_lib_handle_;
};

class SoundModelInfo {
public:
    SoundModelInfo();
    SoundModelInfo(SoundModelInfo &rhs) = delete;
    SoundModelInfo & operator=(SoundModelInfo &rhs);
    ~SoundModelInfo();
    int32_t SetKeyPhrases(listen_model_type *model, uint32_t num_phrases);
    int32_t SetUsers(listen_model_type *model, uint32_t num_users);
    int32_t SetConfLevels(uint16_t num_user_kw_pairs, uint16_t *num_users_per_kw,
                          uint16_t **user_kw_pair_flags);
    void SetModelData(uint8_t *data, uint32_t size) {
        if (sm_data_) {
            free(sm_data_);
            sm_data_ = nullptr;
        }
        sm_size_ = size;
        sm_data_ = (uint8_t*) calloc(1, sm_size_);
        if (!sm_data_)
            return;
        memcpy(sm_data_, data, sm_size_);
    }
    void UpdateConfLevel(uint32_t index, uint8_t conf_level) {
        if (index < cf_levels_size_)
            cf_levels_[index] = conf_level;
    }
    int32_t UpdateConfLevelArray(uint8_t *conf_levels, uint32_t cfl_size);
    void ResetDetConfLevels() {
        memset(det_cf_levels_, 0, cf_levels_size_);
    }
    void UpdateDetConfLevel(uint32_t index, uint8_t conf_level) {
        if (index < cf_levels_size_)
            det_cf_levels_[index] = conf_level;
    }
    uint8_t* GetModelData() { return sm_data_; };
    uint32_t GetModelSize() { return sm_size_; };
    char** GetKeyPhrases() { return keyphrases_; };
    char** GetConfLevelsKwUsers() { return cf_levels_kw_users_; };
    uint8_t* GetConfLevels() { return cf_levels_; };
    uint8_t* GetDetConfLevels() { return det_cf_levels_; };
    uint32_t GetConfLevelsSize() { return cf_levels_size_; };
    uint32_t GetNumKeyPhrases() { return num_keyphrases_; };
    static void AllocArrayPtrs(char ***arr, uint32_t arr_len, uint32_t elem_len);
    static void FreeArrayPtrs(char **arr, uint32_t arr_len);

private:
    uint8_t *sm_data_;
    uint32_t sm_size_;
    uint32_t num_keyphrases_;
    uint32_t num_users_;
    char **keyphrases_;
    char **users_;
    char **cf_levels_kw_users_;
    uint8_t *cf_levels_;
    uint8_t *det_cf_levels_;
    uint32_t cf_levels_size_;
};
#endif // SOUND_TRIGGER_UTILS_H
