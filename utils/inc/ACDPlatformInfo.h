/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#ifndef ACD_PLATFORM_INFO_H
#define ACD_PLATFORM_INFO_H

#include <stdint.h>
#include <map>
#include <vector>
#include <memory>
#include <string>
#include "PalDefs.h"
#include "ResourceManager.h"
#include "SoundTriggerUtils.h"

#define ACD_CAPTURE_PROFILE_PRIORITY_HIGH 1
#define ACD_CAPTURE_PROFILE_PRIORITY_LOW -1
#define ACD_CAPTURE_PROFILE_PRIORITY_SAME 0

enum ACDOperatingModes {
    ACD_OPERATING_MODE_LOW_POWER,
    ACD_OPERATING_MODE_HIGH_PERF,
    ACD_OPERATING_MODE_HIGH_PERF_AND_CHARGING
};

enum ACDInputModes {
    ACD_INPUT_MODE_HANDSET,
    ACD_INPUT_MODE_HEADSET
};

typedef enum {
    ACD_SOUND_MODEL_ID_ENV,
    ACD_SOUND_MODEL_ID_EVENT,
    ACD_SOUND_MODEL_ID_SPEECH,
    ACD_SOUND_MODEL_ID_MUSIC,
    ACD_SOUND_MODEL_AMBIENCE_NOISE_SILENCE,
    ACD_SOUND_MODEL_ID_MAX,
} acd_model_id_t;

using ACDUUID = SoundTriggerUUID;
class StreamConfig;

class ACDXml {
 public:
    virtual void HandleStartTag(const char *tag, const char **attribs) = 0;
    virtual void HandleEndTag(struct xml_userdata *data, const char *tag) = 0;
    virtual ~ACDXml() {};
};

class ACDCaptureProfile : public ACDXml {
 public:
    ACDCaptureProfile(std::string name);
    ACDCaptureProfile() = delete;
    ACDCaptureProfile(ACDCaptureProfile &rhs) = delete;
    ACDCaptureProfile & operator=(ACDCaptureProfile &rhs) = delete;

    void HandleStartTag(const char* tag, const char* * attribs) override;
    void HandleEndTag(struct xml_userdata *data, const char* tag) override;

    std::string GetName() const { return name_; }
    pal_device_id_t GetDevId() const { return device_id_; }
    uint32_t GetSampleRate() const { return sample_rate_; }
    uint32_t GetBitWidth() const { return bitwidth_; }
    uint32_t GetChannels() const { return channels_; }
    std::string GetSndName() const { return snd_name_; }
    void SetSampleRate(uint32_t sample_rate) { sample_rate_ = sample_rate; }
    void SetBitWidth(uint32_t bit_width) { bitwidth_ = bit_width; }
    void SetChannels(uint32_t channels) { channels_ = channels; }
    void SetSndName(std::string snd_name) { snd_name_ = snd_name; }
    std::pair<uint32_t,uint32_t> GetDevicePpKv() const { return device_pp_kv_; }

    int32_t ComparePriority(std::shared_ptr<ACDCaptureProfile> cap_prof);

 private:
    std::string name_;
    pal_device_id_t device_id_;
    uint32_t sample_rate_;
    uint32_t channels_;
    uint32_t bitwidth_;
    std::pair<uint32_t,uint32_t> device_pp_kv_;
    std::string snd_name_;
};

using acd_cap_profile_map_t =
    std::map<std::string, std::shared_ptr<ACDCaptureProfile>>;

class ACDContextInfo {
 public:
    ACDContextInfo(uint32_t context_id, uint32_t type);
    uint32_t GetContextId();
    uint32_t GetContextType();
 private:
    uint32_t context_id_;
    uint32_t context_type_;
};

class ACDSoundModelInfo : public ACDXml {
 public:
    ACDSoundModelInfo(StreamConfig *sm_cfg);

    void HandleStartTag(const char *tag, const char **attribs) override;
    void HandleEndTag(struct xml_userdata *data, const char *tag) override;

    std::string GetModelType();
    std::string GetModelBinName();
    uint32_t GetModelUUID();
    uint32_t GetModelId();
    size_t GetNumContexts();
    std::vector<std::shared_ptr<ACDContextInfo>> GetSupportedContextList();

 private:
    std::string  model_type_;
    std::string  model_bin_name_;
    uint32_t     model_id_;
    uint32_t     model_uuid_;
    bool         is_parsing_contexts;
    StreamConfig *sm_cfg_;
    std::vector<std::shared_ptr<ACDContextInfo>> acd_context_info_list_;
};

class StreamConfig : public ACDXml {
 public:
    StreamConfig(const acd_cap_profile_map_t&);
    StreamConfig(StreamConfig &rhs) = delete;
    StreamConfig & operator=(StreamConfig &rhs) = delete;

    void HandleStartTag(const char *tag, const char **attribs) override;
    void HandleEndTag(struct xml_userdata *data, const char *tag) override;

    ACDUUID GetUUID();
    uint32_t GetSampleRate();
    uint32_t GetBitWidth();
    uint32_t GetOutChannels();
    std::string GetStreamConfigName();
    std::shared_ptr<ACDCaptureProfile> GetCaptureProfile(
        std::pair<ACDOperatingModes, ACDInputModes> mode_pair);
    std::shared_ptr<ACDSoundModelInfo> GetSoundModelInfoByModelId(uint32_t model_id);
    std::vector<std::shared_ptr<ACDSoundModelInfo>> GetSoundModelList();
    std::shared_ptr<ACDSoundModelInfo> GetSoundModelInfoByContextId(uint32_t context_id);
    void UpdateContextModelMap(uint32_t context_id);
    std::pair<uint32_t,uint32_t> GetStreamMetadata();
 private:
    std::string name_;
    ACDUUID     vendor_uuid_;
    uint32_t    sample_rate_;
    uint32_t    bit_width_;
    uint32_t    out_channels_;
    std::pair<uint32_t,uint32_t> stream_metadata_;
    const acd_cap_profile_map_t& cap_profile_map_;
    std::map<std::pair<ACDOperatingModes, ACDInputModes>, std::shared_ptr<ACDCaptureProfile>> op_modes_;
    std::shared_ptr<ACDXml> curr_child_;
    std::vector<std::shared_ptr<ACDSoundModelInfo>> acd_soundmodel_info_list_;
    std::map<uint32_t, std::shared_ptr<ACDSoundModelInfo>> context_model_map_;
    std::map<uint32_t, std::shared_ptr<ACDSoundModelInfo>> acd_modelinfo_map_;

    /* reads capture profile names into member variables */
    void ReadCapProfileNames(ACDOperatingModes mode, const char **attribs);
};

class ACDPlatformInfo : public ACDXml {
 public:
    ACDPlatformInfo(StreamConfig &rhs) = delete;
    ACDPlatformInfo & operator=(ACDPlatformInfo &rhs) =
         delete;

    void HandleStartTag(const char *tag, const char **attribs) override;
    void HandleEndTag(struct xml_userdata *data, const char *tag) override;

    static std::shared_ptr<ACDPlatformInfo> GetInstance();
    bool GetSupportDevSwitch() const;
    bool GetDedicatedSvaPath() const;
    bool GetDedicatedHeadsetPath() const;
    bool GetLpiEnable() const;
    bool GetEnableDebugDumps() const;
    bool GetConcurrentCaptureEnable() const;
    bool GetConcurrentVoiceCallEnable() const;
    bool GetConcurrentVoipCallEnable() const;
    bool GetLowLatencyBargeinEnable() const;
    bool IsACDEnabled() const;
    std::shared_ptr<StreamConfig> GetStreamConfig(const ACDUUID& uuid) const;
    std::shared_ptr<ACDCaptureProfile> GetCapProfile(const std::string& name) const;

 private:
    ACDPlatformInfo();
    static std::shared_ptr<ACDPlatformInfo> me_;
    bool acd_enable_;
    bool support_device_switch_;
    bool dedicated_sva_path_;
    bool dedicated_headset_path_;
    bool lpi_enable_;
    bool enable_debug_dumps_;
    bool concurrent_capture_;
    bool concurrent_voice_call_;
    bool concurrent_voip_call_;
    bool low_latency_bargein_enable_;
    std::map<ACDUUID, std::shared_ptr<StreamConfig>> acd_cfg_list_;
    acd_cap_profile_map_t capture_profile_map_;
    std::shared_ptr<ACDXml> curr_child_;
};
#endif
