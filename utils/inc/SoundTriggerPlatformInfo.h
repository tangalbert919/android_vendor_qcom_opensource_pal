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

#ifndef SOUND_TRIGGER_PLATFORM_INFO_H
#define SOUND_TRIGGER_PLATFORM_INFO_H

#include <stdint.h>
#include <map>
#include <memory>
#include <string>
#include "PalDefs.h"
#include "SoundTriggerEngine.h"

#define CAPTURE_PROFILE_PRIORITY_HIGH 1
#define CAPTURE_PROFILE_PRIORITY_LOW -1
#define CAPTURE_PROFILE_PRIORITY_SAME 0

enum StOperatingModes {
    ST_OPERATING_MODE_LOW_POWER,
    ST_OPERATING_MODE_HIGH_PERF,
    ST_OPERATING_MODE_HIGH_PERF_AND_CHARGING
};

enum StInputModes {
    ST_INPUT_MODE_HANDSET,
    ST_INPUT_MODE_HEADSET
};

class SoundTriggerUUID {
 public:
    SoundTriggerUUID();
    SoundTriggerUUID & operator=(SoundTriggerUUID &rhs);
    bool operator<(const SoundTriggerUUID &rhs) const;

    uint32_t timeLow;
    uint16_t timeMid;
    uint16_t timeHiAndVersion;
    uint16_t clockSeq;
    uint8_t  node[6];

};
using UUID = SoundTriggerUUID;

class SoundTriggerXml {
 public:
    virtual void HandleStartTag(const char *tag, const char **attribs) = 0;
    virtual void HandleEndTag(const char *tag) = 0;
    virtual void HandleCharData(const char *data) = 0;
    virtual ~SoundTriggerXml() {};
};

class CaptureProfile : public SoundTriggerXml {
 public:
    CaptureProfile(std::string name);
    CaptureProfile() = delete;
    CaptureProfile(CaptureProfile &rhs) = delete;
    CaptureProfile & operator=(CaptureProfile &rhs) = delete;

    void HandleStartTag(const char* tag, const char* * attribs) override;
    void HandleEndTag(const char* tag) override;
    void HandleCharData(const char* data) override;

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

    int32_t ComparePriority(std::shared_ptr<CaptureProfile> cap_prof);

 private:
    std::string name_;
    pal_device_id_t device_id_;
    uint32_t sample_rate_;
    uint32_t channels_;
    uint32_t bitwidth_;
    std::pair<uint32_t,uint32_t> device_pp_kv_;
    std::string snd_name_;
};

using st_cap_profile_map_t =
    std::map<std::string, std::shared_ptr<CaptureProfile>>;

class SecondStageConfig : public SoundTriggerXml {
 public:
    SecondStageConfig();

    void HandleStartTag(const char *tag, const char **attribs) override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

    st_sound_model_type_t GetDetectionType() const { return detection_type_; }
    uint32_t GetSoundModelID() const { return sm_id_; }
    std::string GetLibName() const { return module_lib_; }
    uint32_t GetSampleRate() const { return sample_rate_; }
    uint32_t GetBitWidth() const { return bit_width_; }
    uint32_t GetChannels() const { return channels_; }

 private:
    st_sound_model_type_t detection_type_;
    uint32_t sm_id_;
    std::string module_lib_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t channels_;
};

class SoundModelConfig : public SoundTriggerXml {
 public:
    /*
     * constructor takes a reference to map of capture profiles as it has to
     * look-up the capture profiles for this sound-model
     */
    SoundModelConfig(const st_cap_profile_map_t&);
    SoundModelConfig(SoundModelConfig &rhs) = delete;
    SoundModelConfig & operator=(SoundModelConfig &rhs) = delete;

    SoundTriggerUUID GetUUID() const { return vendor_uuid_; }
    bool GetMergeFirstStageSoundModels() const {
        return merge_first_stage_sound_models_;
    }
    uint32_t GetSampleRate() const { return sample_rate_; }
    uint32_t GetBitWidth() const { return bit_width_; }
    uint32_t GetOutChannels() const { return out_channels_; }
    uint32_t GetKwDuration() const { return capture_keyword_; }
    uint32_t GetCaptureReadDelay() const { return client_capture_read_delay_; }
    uint32_t GetKwStartTolerance() const { return kw_start_tolerance_; }
    uint32_t GetKwEndTolerance() const { return kw_end_tolerance_; }
    std::pair<uint32_t,uint32_t> GetStreamConfig() const { return stream_config_; }
    std::shared_ptr<CaptureProfile> GetCaptureProfile(
        std::pair<StOperatingModes, StInputModes> mode_pair) const {
        return op_modes_.at(mode_pair);
    }
    std::shared_ptr<SecondStageConfig> GetSecondStageConfig(
        const uint32_t& sm_id) const;

    void HandleStartTag(const char *tag, const char **attribs)
        override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

 private:
    /* reads capture profile names into member variables */
    void ReadCapProfileNames(StOperatingModes mode, const char* * attribs);

    SoundTriggerUUID vendor_uuid_;
    bool merge_first_stage_sound_models_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t out_channels_;
    uint32_t capture_keyword_;
    uint32_t client_capture_read_delay_;
    uint32_t kw_start_tolerance_;
    uint32_t kw_end_tolerance_;
    std::pair<uint32_t,uint32_t> stream_config_;
    const st_cap_profile_map_t& cap_profile_map_;
    std::map<std::pair<StOperatingModes, StInputModes>, std::shared_ptr<CaptureProfile>> op_modes_;
    std::shared_ptr<SoundTriggerXml> curr_child_;
    std::map<uint32_t, std::shared_ptr<SecondStageConfig>> ss_config_list_;
};

class SoundTriggerPlatformInfo : public SoundTriggerXml {
 public:
    SoundTriggerPlatformInfo(SoundModelConfig &rhs) = delete;
    SoundTriggerPlatformInfo & operator=(SoundTriggerPlatformInfo &rhs) =
         delete;

    static std::shared_ptr<SoundTriggerPlatformInfo> GetInstance();
    bool GetEnableFailureDetection() const {
        return enable_failure_detection_; }
    bool GetSupportDevSwitch() const { return support_device_switch_; }
    bool GetTransitToNonLpiOnCharging() const {
        return transit_to_non_lpi_on_charging_;
    }
    bool GetDedicatedSvaPath() const { return dedicated_sva_path_; }
    bool GetDedicatedHeadsetPath() const { return dedicated_headset_path_; }
    bool GetLpiEnable() const { return lpi_enable_; }
    bool GetEnableDebugDumps() const { return enable_debug_dumps_; }
    bool GetNonLpiWithoutEc() const { return non_lpi_without_ec_; }
    bool GetConcurrentCaptureEnable() const { return concurrent_capture_; }
    bool GetConcurrentVoiceCallEnable() const { return concurrent_voice_call_; }
    bool GetConcurrentVoipCallEnable() const { return concurrent_voip_call_; }
    std::shared_ptr<SoundModelConfig> GetSmConfig(const UUID& uuid) const;
    std::shared_ptr<CaptureProfile> GetCapProfile(const std::string& name) const;

    void HandleStartTag(const char *tag, const char **attribs)
        override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

    static int StringToUUID(const char* str, SoundTriggerUUID& UUID);

 private:
    SoundTriggerPlatformInfo();
    static std::shared_ptr<SoundTriggerPlatformInfo> me_;
    uint32_t version_;
    bool enable_failure_detection_;
    bool support_device_switch_;
    bool transit_to_non_lpi_on_charging_;
    bool dedicated_sva_path_;
    bool dedicated_headset_path_;
    bool lpi_enable_;
    bool enable_debug_dumps_;
    bool non_lpi_without_ec_;
    bool concurrent_capture_;
    bool concurrent_voice_call_;
    bool concurrent_voip_call_;
    std::map<UUID, std::shared_ptr<SoundModelConfig>> sound_model_cfg_list_;
    st_cap_profile_map_t capture_profile_map_;
    std::shared_ptr<SoundTriggerXml> curr_child_;
};
#endif
