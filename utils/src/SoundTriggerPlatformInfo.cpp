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

#include "SoundTriggerPlatformInfo.h"

#include <errno.h>

#include <string>
#include <map>

#include "PalCommon.h"
#include "PalDefs.h"
#include "kvh2xml.h"

#define LOG_TAG "PAL: SoundTriggerPlatformInf"

const std::map<std::string, uint32_t> devicePPKeyLUT {
    {std::string{ "DEVICEPP_TX" }, DEVICEPP_TX},
};

const std::map<std::string, uint32_t> devicePPValueLUT {
    {std::string{ "DEVICEPP_TX_FLUENCE_FFNS" }, DEVICEPP_TX_FLUENCE_FFNS},
    {std::string{ "DEVICEPP_TX_FLUENCE_FFECNS" }, DEVICEPP_TX_FLUENCE_FFECNS},
    {std::string{ "DEVICEPP_TX_RAW_LPI" }, DEVICEPP_TX_RAW_LPI},
    {std::string{ "DEVICEPP_TX_RAW_NLPI" }, DEVICEPP_TX_RAW_NLPI},
};

static const struct st_uuid qcva_uuid =
    { 0x68ab2d40, 0xe860, 0x11e3, 0x95ef, { 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b } };

static const struct st_uuid qcmd_uuid =
    { 0x876c1b46, 0x9d4d, 0x40cc, 0xa4fd, { 0x4d, 0x5e, 0xc7, 0xa8, 0x0e, 0x47 } };

CaptureProfile::CaptureProfile(const std::string name) :
    name_(name),
    device_id_(PAL_DEVICE_IN_MIN),
    sample_rate_(16000),
    channels_(1),
    bitwidth_(16),
    device_pp_kv_(std::make_pair(DEVICEPP_TX,
        DEVICEPP_TX_FLUENCE_FFNS)),
    snd_name_("va-mic")
{

}

void CaptureProfile::HandleCharData(const char* data __unused) {
}

void CaptureProfile::HandleEndTag(const char* tag) {
    PAL_DBG(LOG_TAG, "Got end tag %s", tag);
    return;
}

void CaptureProfile::HandleStartTag(const char* tag, const char** attribs) {

    PAL_DBG(LOG_TAG, "Got start tag %s", tag);
    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "device_id")) {
                auto itr = deviceIdLUT.find(attribs[++i]);
                if (itr == deviceIdLUT.end()) {
                    PAL_ERR(LOG_TAG, "could not find key %s in lookup table",
                        attribs[i]);
                } else {
                    device_id_ = itr->second;
                }
            } else if (!strcmp(attribs[i], "sample_rate")) {
                sample_rate_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "bit_width")) {
                bitwidth_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "channels")) {
                channels_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "snd_name")) {
                snd_name_ = attribs[++i];
            } else {
                PAL_INFO(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else if (!strcmp(tag, "kvpair")) {
        uint32_t i = 0;
        uint32_t key = 0, value = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "key")) {
                auto keyItr = devicePPKeyLUT.find(attribs[++i]);
                if (keyItr == devicePPKeyLUT.end()) {
                    PAL_ERR(LOG_TAG, "could not find key %s in lookup table",
                        attribs[i]);
                } else {
                    key = keyItr->second;
                }
            } else if(!strcmp(attribs[i], "value")) {
                auto valItr = devicePPValueLUT.find(attribs[++i]);
                if (valItr == devicePPValueLUT.end()) {
                    PAL_ERR(LOG_TAG, "could not find value %s in lookup table",
                        attribs[i]);
                } else {
                    value = valItr->second;
                }

                device_pp_kv_ = std::make_pair(key, value);
            }
            ++i; /* move to next attribute */
        }
    } else {
        PAL_INFO(LOG_TAG, "Invalid tag %s", (char *)tag);
    }
}

/*
 * Priority compare result indicated by return value as below:
 * 1. CAPTURE_PROFILE_PRIORITY_HIGH
 *     current capture profile has higher priority than cap_prof
 * 2. CAPTURE_PROFILE_PRIORITY_LOW
 *     current capture profile has lower priority than cap_prof
 * 3. CAPTURE_PROFILE_PRIORITY_SAME
 *     current capture profile has same priority than cap_prof
 */
int32_t CaptureProfile::ComparePriority(std::shared_ptr<CaptureProfile> cap_prof) {
    int32_t priority_check = 0;

    if (!cap_prof) {
        priority_check = CAPTURE_PROFILE_PRIORITY_HIGH;
    } else {
        // only compare channels for priority for now
        if (channels_ < cap_prof->GetChannels()) {
            priority_check = CAPTURE_PROFILE_PRIORITY_LOW;
        } else if (channels_ > cap_prof->GetChannels()) {
            priority_check = CAPTURE_PROFILE_PRIORITY_HIGH;
        } else {
            priority_check = CAPTURE_PROFILE_PRIORITY_SAME;
        }
    }

    return priority_check;
}

SecondStageConfig::SecondStageConfig() :
    detection_type_(ST_SM_TYPE_NONE),
    sm_id_(0),
    module_lib_(""),
    sample_rate_(16000),
    bit_width_(16),
    channels_(1)
{
}

void SecondStageConfig::HandleStartTag(const char *tag, const char **attribs) {
    PAL_DBG(LOG_TAG, "Got tag %s", tag);

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "sm_detection_type")) {
                i++;
                if (!strcmp(attribs[i], "KEYWORD_DETECTION")) {
                    detection_type_ = ST_SM_TYPE_KEYWORD_DETECTION;
                } else if (!strcmp(attribs[i], "USER_VERIFICATION")) {
                    detection_type_ = ST_SM_TYPE_USER_VERIFICATION;
                } else if (!strcmp(attribs[i], "CUSTOM_DETECTION")) {
                    detection_type_ = ST_SM_TYPE_CUSTOM_DETECTION;
                }
            } else if (!strcmp(attribs[i], "sm_id")) {
                sm_id_ = std::strtoul(attribs[++i], nullptr, 16);
            } else if (!strcmp(attribs[i], "module_lib")) {
                module_lib_ = attribs[++i];
            } else if (!strcmp(attribs[i], "sample_rate")) {
                sample_rate_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "bit_width")) {
                bit_width_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "channel_count")) {
                channels_ = std::stoi(attribs[++i]);
            }
            ++i;
        }
    } else {
        PAL_INFO(LOG_TAG, "Invalid tag %s", tag);
    }
}

void SecondStageConfig::HandleEndTag(const char *tag __unused) {
}

void SecondStageConfig::HandleCharData(const char *data __unused) {
}

SoundTriggerModuleInfo::SoundTriggerModuleInfo() :
    model_type_(ST_MODULE_TYPE_GMM)
{
    for (int i = 0; i < MAX_PARAM_IDS; i++) {
        module_tag_ids_[i] = 0;
        param_ids_[i] = 0;
    }
}

void SoundTriggerModuleInfo::HandleStartTag(const char *tag, const char **attribs) {
    PAL_DBG(LOG_TAG, "Got tag %s", tag);

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "module_type")) {
                i++;
                if (!strcmp(attribs[i], "GMM")) {
                    model_type_ = ST_MODULE_TYPE_GMM;
                    PAL_DBG(LOG_TAG, "GMM module");
                } else if (!strcmp(attribs[i], "PDK")) {
                    model_type_ = ST_MODULE_TYPE_PDK;
                    PAL_DBG(LOG_TAG, "PDK module");
                }
            } else {
                uint32_t index = 0;
                if (!strcmp(attribs[i], "load_sound_model_ids")) {
                    index = LOAD_SOUND_MODEL;
                } else if (!strcmp(attribs[i], "unload_sound_model_ids")) {
                    index = UNLOAD_SOUND_MODEL;
                } else if (!strcmp(attribs[i], "wakeup_config_ids")) {
                    index = WAKEUP_CONFIG;
                } else if (!strcmp(attribs[i], "buffering_config_ids")) {
                    index = BUFFERING_CONFIG;
                } else if (!strcmp(attribs[i], "engine_reset_ids")) {
                    index = ENGINE_RESET;
                } else if (!strcmp(attribs[i], "custom_config_ids")) {
                    index = CUSTOM_CONFIG;
                } else if (!strcmp(attribs[i], "version_ids")) {
                    index = MODULE_VERSION;
                }
                sscanf(attribs[++i], "%x, %x", &module_tag_ids_[index],
                    &param_ids_[index]);
                PAL_DBG(LOG_TAG, "index : %u, module_id : %x, param : %x",
                index, module_tag_ids_[index], param_ids_[index]);
            }
            ++i;
        }
    } else if (!strcmp(tag, "kvpair")) {
        uint32_t key = 0, value = 0;
        if (strcmp(attribs[0], "key") || strcmp(attribs[2], "value")) {
            PAL_ERR(LOG_TAG, "stream key/value not found");
            return;
        }
        key = strtoul(attribs[1], NULL, 0);
        value = strtoul(attribs[3], NULL, 0);
        stream_config_ = std::make_pair(key, value);
        PAL_DBG(LOG_TAG, "stream_config_, key = %x, value = %x, %x",
            value, stream_config_.first, stream_config_.second);
    } else {
        PAL_ERR(LOG_TAG, "Invalid tag %s", tag);
    }
}

void SoundTriggerModuleInfo::HandleEndTag(const char *tag __unused) {
}

void SoundTriggerModuleInfo::HandleCharData(const char *data __unused) {
}

SoundModelConfig::SoundModelConfig(const st_cap_profile_map_t& cap_prof_map) :
    is_qcva_uuid_(false),
    is_qcmd_uuid_(false),
    merge_first_stage_sound_models_(false),
    sample_rate_(16000),
    bit_width_(16),
    out_channels_(1),
    capture_keyword_(2000),
    client_capture_read_delay_(2000),
    cap_profile_map_(cap_prof_map),
    curr_child_(nullptr)
{
}

std::pair<uint32_t, uint32_t> SoundModelConfig::GetStreamConfig(
    uint32_t type) {
    return GetSoundTriggerModuleInfo(type)->getStreamConfigKV();
}

void SoundModelConfig::ReadCapProfileNames(StOperatingModes mode,
    const char** attribs) {
    uint32_t i = 0;
    while (attribs[i]) {
        if (!strcmp(attribs[i], "capture_profile_handset")) {
            op_modes_[std::make_pair(mode, ST_INPUT_MODE_HANDSET)] =
                cap_profile_map_.at(std::string(attribs[++i]));
        } else if(!strcmp(attribs[i], "capture_profile_headset")) {
            op_modes_[std::make_pair(mode, ST_INPUT_MODE_HEADSET)] =
                cap_profile_map_.at(std::string(attribs[++i]));
        } else {
            PAL_ERR(LOG_TAG, "got unexpected attribute: %s", attribs[i]);
        }
        ++i; /* move to next attribute */
    }
}

std::shared_ptr<SecondStageConfig> SoundModelConfig::GetSecondStageConfig(
    const listen_model_indicator_enum& sm_type) const {
    uint32_t sm_id = static_cast<uint32_t>(sm_type);
    auto ss_config = ss_config_list_.find(sm_id);
    if (ss_config != ss_config_list_.end())
        return ss_config->second;
    else
        return nullptr;
}


std::shared_ptr<SoundTriggerModuleInfo> SoundModelConfig::GetSoundTriggerModuleInfo(
    const uint32_t type) const {

    uint32_t module_type = type;

    PAL_DBG(LOG_TAG, "search module for model type %u", type);
    if (IS_MODULE_TYPE_PDK(type)) {
        PAL_DBG(LOG_TAG, "PDK module");
        module_type = ST_MODULE_TYPE_PDK;
    }
    auto module_config = st_module_info_list_.find(module_type);
    if (module_config != st_module_info_list_.end())
        return module_config->second;
    else
        return nullptr;
}

void SoundModelConfig::HandleCharData(const char* data __unused) {
}

void SoundModelConfig::HandleStartTag(const char* tag, const char** attribs) {
    PAL_DBG(LOG_TAG, "Got tag %s", tag);

    /* Delegate to child element if currently active */
    if (curr_child_) {
        curr_child_->HandleStartTag(tag, attribs);
        return;
    }

    if (!strcmp(tag, "arm_ss_usecase")) {
        curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
            std::make_shared<SecondStageConfig>());
        return;
    } if (!strcmp(tag, "module_params")) {
        curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
            std::make_shared<SoundTriggerModuleInfo>());
        return;
    }

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "vendor_uuid")) {
                SoundTriggerUUID::StringToUUID(attribs[++i],
                    vendor_uuid_);
                if (vendor_uuid_.CompareUUID(qcva_uuid)) {
                    is_qcva_uuid_ = true;
                } else if (vendor_uuid_.CompareUUID(qcmd_uuid)) {
                    is_qcmd_uuid_ = true;
                }
            } else if (!strcmp(attribs[i], "get_module_version")) {
                get_module_version_supported_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "merge_first_stage_sound_models")) {
                merge_first_stage_sound_models_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "sample_rate")) {
                sample_rate_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "bit_width")) {
                bit_width_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "out_channels")) {
                SetOutChannels(std::stoi(attribs[++i]));
            } else if (!strcmp(attribs[i], "client_capture_read_delay")) {
                client_capture_read_delay_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "capture_keyword")) {
                capture_keyword_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "kw_start_tolerance")) {
                kw_start_tolerance_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "kw_end_tolerance")) {
                kw_end_tolerance_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "data_before_kw_start")) {
                data_before_kw_start_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "data_after_kw_end")) {
                data_after_kw_end_ = std::stoi(attribs[++i]);
            } else {
                PAL_INFO(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else if (!strcmp(tag, "low_power")) {
        ReadCapProfileNames(ST_OPERATING_MODE_LOW_POWER, attribs);
    } else if (!strcmp(tag, "high_performance")) {
        ReadCapProfileNames(ST_OPERATING_MODE_HIGH_PERF, attribs);
    } else if (!strcmp(tag, "high_performance_and_charging")) {
        ReadCapProfileNames(ST_OPERATING_MODE_HIGH_PERF_AND_CHARGING, attribs);
    } else {
          PAL_INFO(LOG_TAG, "Invalid tag %s", (char *)tag);
    }
}

void SoundModelConfig::HandleEndTag(const char* tag __unused) {
    PAL_DBG(LOG_TAG, "Got end tag %s", tag);

    if (!strcmp(tag, "arm_ss_usecase")) {
        std::shared_ptr<SecondStageConfig> ss_cfg(
            std::static_pointer_cast<SecondStageConfig>(curr_child_));
        const auto res = ss_config_list_.insert(
            std::make_pair(ss_cfg->GetSoundModelID(), ss_cfg));
        if (!res.second)
            PAL_ERR(LOG_TAG, "Failed to insert to map");
        curr_child_ = nullptr;
    } else if (!strcmp(tag, "module_params")) {
        std::shared_ptr<SoundTriggerModuleInfo> st_module_info(
            std::static_pointer_cast<SoundTriggerModuleInfo>(curr_child_));
        const auto res = st_module_info_list_.insert(
            std::make_pair(st_module_info->GetModelType(), st_module_info));
        if (!res.second)
            PAL_ERR(LOG_TAG, "Failed to insert to map");
        curr_child_ = nullptr;
    }

    if (curr_child_)
        curr_child_->HandleEndTag(tag);

    return;
}

std::shared_ptr<SoundTriggerPlatformInfo> SoundTriggerPlatformInfo::me_ =
    nullptr;

SoundTriggerPlatformInfo::SoundTriggerPlatformInfo() :
    enable_failure_detection_(false),
    support_device_switch_(false),
    transit_to_non_lpi_on_charging_(false),
    dedicated_sva_path_(true),
    dedicated_headset_path_(false),
    lpi_enable_(true),
    enable_debug_dumps_(false),
    non_lpi_without_ec_(false),
    concurrent_capture_(false),
    concurrent_voice_call_(false),
    concurrent_voip_call_(false),
    low_latency_bargein_enable_(false),
    mmap_enable_(false),
    mmap_buffer_duration_(0),
    curr_child_(nullptr)
{
}

std::shared_ptr<SoundModelConfig> SoundTriggerPlatformInfo::GetSmConfig(
    const UUID& uuid) const {
    auto smCfg = sound_model_cfg_list_.find(uuid);
    if (smCfg != sound_model_cfg_list_.end())
        return smCfg->second;
    else
        return nullptr;
}

std::shared_ptr<CaptureProfile> SoundTriggerPlatformInfo::GetCapProfile(
    const std::string& name) const {
    auto capProfile = capture_profile_map_.find(name);
    if (capProfile != capture_profile_map_.end())
        return capProfile->second;
    else
        return nullptr;
}

// We can assume only Hotword sm config supports version api
void SoundTriggerPlatformInfo::GetSmConfigForVersionQuery(
    std::vector<std::shared_ptr<SoundModelConfig>> &sm_cfg_list) const {

    std::shared_ptr<SoundModelConfig> sm_cfg = nullptr;
    for (auto iter = sound_model_cfg_list_.begin();
        iter != sound_model_cfg_list_.end(); iter++) {
        sm_cfg = iter->second;
        if (sm_cfg && sm_cfg->GetModuleVersionSupported()) {
            sm_cfg_list.push_back(sm_cfg);
        }
    }
}

std::shared_ptr<SoundTriggerPlatformInfo>
SoundTriggerPlatformInfo::GetInstance() {

    if (!me_)
        me_ = std::shared_ptr<SoundTriggerPlatformInfo>
            (new SoundTriggerPlatformInfo);

    return me_;
}

void SoundTriggerPlatformInfo::HandleStartTag(const char* tag,
                                              const char** attribs) {
    /* Delegate to child element if currently active */
    if (curr_child_) {
        curr_child_->HandleStartTag(tag, attribs);
        return;
    }

    PAL_DBG(LOG_TAG, "Got start tag %s", tag);
    if (!strcmp(tag, "sound_model_config")) {
        curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
            std::make_shared<SoundModelConfig>(capture_profile_map_));
        return;
    }

    if (!strcmp(tag, "capture_profile")) {
        if (attribs[0] && !strcmp(attribs[0], "name")) {
            curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
                    std::make_shared<CaptureProfile>(attribs[1]));
            return;
        } else {
            PAL_ERR(LOG_TAG,"missing name attrib for tag %s", tag);
            return;
        }
    }

    if (!strcmp(tag, "common_config") || !strcmp(tag, "capture_profile_list")) {
        PAL_INFO(LOG_TAG, "tag:%s appeared, nothing to do", tag);
        return;
    }

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!attribs[i]) {
                PAL_ERR(LOG_TAG,"missing attrib value for tag %s", tag);
            } else if (!strcmp(attribs[i], "version")) {
                version_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "enable_failure_detection")) {
                enable_failure_detection_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "support_device_switch")) {
                support_device_switch_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "transit_to_non_lpi_on_charging")) {
                transit_to_non_lpi_on_charging_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "dedicated_sva_path")) {
                dedicated_sva_path_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "dedicated_headset_path")) {
                dedicated_headset_path_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "lpi_enable")) {
                lpi_enable_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "enable_debug_dumps")) {
                enable_debug_dumps_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "non_lpi_without_ec")) {
                non_lpi_without_ec_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "concurrent_capture")) {
                concurrent_capture_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "concurrent_voice_call") &&
                       concurrent_capture_) {
                concurrent_voice_call_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "low_latency_bargein_enable") &&
                       concurrent_capture_) {
                low_latency_bargein_enable_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "concurrent_voip_call") &&
                       concurrent_capture_) {
                concurrent_voip_call_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "mmap_enable")) {
                mmap_enable_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "mmap_buffer_duration")) {
                mmap_buffer_duration_ = std::stoi(attribs[++i]);
            } else {
                PAL_INFO(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else {
        PAL_INFO(LOG_TAG, "Invalid tag %s", tag);
    }
}

void SoundTriggerPlatformInfo::HandleEndTag(const char* tag) {
    PAL_DBG(LOG_TAG, "Got end tag %s", tag);

    if (!strcmp(tag, "sound_model_config")) {
        std::shared_ptr<SoundModelConfig> sm_cfg(
            std::static_pointer_cast<SoundModelConfig>(curr_child_));
        const auto res = sound_model_cfg_list_.insert(
            std::make_pair(sm_cfg->GetUUID(), sm_cfg));
        if (!res.second)
            PAL_ERR(LOG_TAG, "Failed to insert to map");
        curr_child_ = nullptr;
    } else if (!strcmp(tag, "capture_profile")) {
        std::shared_ptr<CaptureProfile> cap_prof(
            std::static_pointer_cast<CaptureProfile>(curr_child_));
        const auto res = capture_profile_map_.insert(
            std::make_pair(cap_prof->GetName(), cap_prof));
        if (!res.second)
            PAL_ERR(LOG_TAG, "Failed to insert to map");
        curr_child_ = nullptr;
    }


    if (curr_child_)
        curr_child_->HandleEndTag(tag);

    return;
}

void SoundTriggerPlatformInfo::HandleCharData(const char* data __unused) {
}
