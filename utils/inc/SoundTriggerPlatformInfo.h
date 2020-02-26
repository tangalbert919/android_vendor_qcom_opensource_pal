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

class SecondStageCfg : public SoundTriggerXml {
 public:
    SecondStageCfg();
    SecondStageCfg(SecondStageCfg &rhs) = delete;
    SecondStageCfg & operator=(SecondStageCfg &rhs) = delete;

    enum SmDetectionType {
        KEYWORD_DETECTION,
        USER_VERIFICATION,
        CUSTOM_DETECTION
    };

    uint32_t GetSmId() { return sm_id_; }
    std::string & GetModuleLib() { return *module_lib_; }

    void HandleStartTag(const char *tag, const char **attribs)
        override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

 private:
    SmDetectionType sm_detection_type_;
    uint32_t sm_id_;
    std::unique_ptr<std::string> module_lib_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t channel_count_;
};

class SoundModelConfig : public SoundTriggerXml {
 public:
    SoundModelConfig();
    SoundModelConfig(SoundModelConfig &rhs) = delete;
    SoundModelConfig & operator=(SoundModelConfig &rhs) = delete;

    SoundTriggerUUID GetUUID() const { return vendor_uuid_; }
    bool GetMergeFirstStageSoundModels() const {
        return merge_first_stage_sound_models_;
    }
    uint32_t GetSampleRate() const { return sample_rate_; }
    uint32_t GetBitWidth() const { return bit_width_; }
    uint32_t GetOutChannels() const { return out_channels_; }
    std::shared_ptr<SecondStageCfg> GetArmSsUsecase(uint32_t sm_id) const;

    void HandleStartTag(const char *tag, const char **attribs)
        override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

 private:
    SoundTriggerUUID vendor_uuid_;
    bool merge_first_stage_sound_models_;
    uint32_t sample_rate_;
    uint32_t bit_width_;
    uint32_t out_channels_;
    uint32_t capture_keyword_;
    uint32_t client_capture_read_delay_;
    std::map<uint32_t, std::shared_ptr<SecondStageCfg>> arm_ss_uc_list_;
    std::shared_ptr<SoundTriggerXml> curr_child_;
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

    void HandleStartTag(const char *tag, const char **attribs)
        override;
    void HandleEndTag(const char *tag) override;
    void HandleCharData(const char *data) override;

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
    std::shared_ptr<SoundTriggerXml> curr_child_;
};
#endif
