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
    {std::string{ "DEVICEPP_TX_VOICE_UI_FLUENCE_FFNS" }, DEVICEPP_TX_VOICE_UI_FLUENCE_FFNS},
    {std::string{ "DEVICEPP_TX_VOICE_UI_FLUENCE_FFECNS" }, DEVICEPP_TX_VOICE_UI_FLUENCE_FFECNS},
    {std::string{ "DEVICEPP_TX_VOICE_UI_RAW" }, DEVICEPP_TX_VOICE_UI_RAW},
};

const std::map<std::string, uint32_t> streamConfigKeyLUT {
    {std::string{ "VOICE_UI_STREAM_CONFIG" }, VOICE_UI_STREAM_CONFIG},
};

const std::map<std::string, uint32_t> streamConfigValueLUT {
    {std::string{ "VUI_STREAM_CFG_SVA" }, VUI_STREAM_CFG_SVA},
    {std::string{ "VUI_STREAM_CFG_HW" }, VUI_STREAM_CFG_HW},
};

SoundTriggerUUID::SoundTriggerUUID() :
    timeLow(0),
    timeMid(0),
    timeHiAndVersion(0),
    clockSeq(0) {
}

bool SoundTriggerUUID::operator<(const SoundTriggerUUID& rhs) const {
    if (timeLow > rhs.timeLow)
        return false;
    else if (timeLow < rhs.timeLow)
        return true;
    /* timeLow is equal */

    if (timeMid > rhs.timeMid)
        return false;
    else if (timeMid < rhs.timeMid)
        return true;
    /* timeLow and timeMid are equal */

    if (timeHiAndVersion > rhs.timeHiAndVersion)
        return false;
    else if (timeHiAndVersion < rhs.timeHiAndVersion)
        return true;
    /* timeLow, timeMid and timeHiAndVersion are equal */

    if (clockSeq > rhs.clockSeq)
        return false;
    else if (clockSeq < rhs.clockSeq)
        return true;
    /* everything is equal */

    return false;
}

SoundTriggerUUID& SoundTriggerUUID::operator = (SoundTriggerUUID& rhs) {
    this->clockSeq = rhs.clockSeq;
    this->timeLow = rhs.timeLow;
    this->timeMid = rhs.timeMid;
    this->timeHiAndVersion = rhs.timeHiAndVersion;
    memcpy(node, rhs.node, sizeof(node));

    return *this;
}

CaptureProfile::CaptureProfile(const std::string name) :
    name_(name),
    device_id_(PAL_DEVICE_IN_MIN),
    sample_rate_(16000),
    channels_(1),
    bitwidth_(16),
    device_pp_kv_(std::make_pair(DEVICEPP_TX,
        DEVICEPP_TX_VOICE_UI_FLUENCE_FFNS)),
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
            } else if (!strcmp(attribs[i], "bit_wdith")) {
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

SoundModelConfig::SoundModelConfig(const st_cap_profile_map_t& cap_prof_map) :
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
    const uint32_t& sm_id) const {
    auto ss_config = ss_config_list_.find(sm_id);
    if (ss_config != ss_config_list_.end())
        return ss_config->second;
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
    }

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "vendor_uuid")) {
                SoundTriggerPlatformInfo::StringToUUID(attribs[++i],
                    vendor_uuid_);
            } else if (!strcmp(attribs[i], "merge_first_stage_sound_models")) {
                merge_first_stage_sound_models_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
            } else if (!strcmp(attribs[i], "sample_rate")) {
                sample_rate_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "bit_width")) {
                bit_width_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "out_channels")) {
                out_channels_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "client_capture_read_delay")) {
                client_capture_read_delay_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "capture_keyword")) {
                capture_keyword_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "kw_start_tolerance")) {
                kw_start_tolerance_ = std::stoi(attribs[++i]);
            } else if (!strcmp(attribs[i], "kw_end_tolerance")) {
                kw_end_tolerance_ = std::stoi(attribs[++i]);
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
                auto keyItr = streamConfigKeyLUT.find(attribs[++i]);
                if (keyItr == streamConfigKeyLUT.end()) {
                    PAL_ERR(LOG_TAG, "could not find key %s in lookup table",
                        attribs[i]);
                } else {
                    key = keyItr->second;
                }
            } else if(!strcmp(attribs[i], "value")) {
                auto valItr = streamConfigValueLUT.find(attribs[++i]);
                if (valItr == streamConfigValueLUT.end()) {
                    PAL_ERR(LOG_TAG, "could not find value %s in lookup table",
                        attribs[i]);
                } else {
                    PAL_DBG(LOG_TAG, "LUT Value %d in lookup table",
                        valItr->second);
                    value = valItr->second;
                }
                stream_config_ = std::make_pair(key, value);
            }
            ++i; /* move to next attribute */
        }
    }  else if (!strcmp(tag, "low_power")) {
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
            } else if (!strcmp(attribs[i], "concurrent_voip_call") &&
                       concurrent_capture_) {
                concurrent_voip_call_ =
                    !strncasecmp(attribs[++i], "true", 4) ? true : false;
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

int SoundTriggerPlatformInfo::StringToUUID(const char* str,
                                           SoundTriggerUUID& UUID) {
    int tmp[10];

    if (str == NULL) {
        return -EINVAL;
    }

    if (sscanf(str, "%08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
               tmp, tmp + 1, tmp + 2, tmp + 3, tmp + 4, tmp + 5, tmp + 6,
               tmp + 7, tmp + 8, tmp + 9) < 10) {
        return -EINVAL;
    }
    UUID.timeLow = (uint32_t)tmp[0];
    UUID.timeMid = (uint16_t)tmp[1];
    UUID.timeHiAndVersion = (uint16_t)tmp[2];
    UUID.clockSeq = (uint16_t)tmp[3];
    UUID.node[0] = (uint8_t)tmp[4];
    UUID.node[1] = (uint8_t)tmp[5];
    UUID.node[2] = (uint8_t)tmp[6];
    UUID.node[3] = (uint8_t)tmp[7];
    UUID.node[4] = (uint8_t)tmp[8];
    UUID.node[5] = (uint8_t)tmp[9];

    return 0;
}
