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

#include "QalCommon.h"

#define LOG_TAG "qal:SoundTriggerPlatformInf"

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


SecondStageCfg::SecondStageCfg() :
    sm_detection_type_(KEYWORD_DETECTION),
    sm_id_(0),
    module_lib_(nullptr),
    sample_rate_(16000),
    bit_width_(16),
    channel_count_(1) {

}

void SecondStageCfg::HandleStartTag(const char* tag, const char** attribs) {
    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!strcmp(attribs[i], "sm_detection_type")) {
                ++i;
                if (!strcmp(attribs[i],"KEYWORD_DETECTION"))
                    sm_detection_type_ = KEYWORD_DETECTION;
                else if (!strcmp(attribs[i],"USER_VERIFICATION"))
                    sm_detection_type_ = USER_VERIFICATION;
                else
                    QAL_ERR(LOG_TAG, "Invalid sm_detection_type");
            } else if(!strcmp(attribs[i], "sm_id")) {
                sm_id_ = std::stoi(attribs[++i], 0, 16);
            } else if(!strcmp(attribs[i], "module_lib")) {
                module_lib_ =
                    std::make_unique<std::string>(attribs[++i]);
            } else if(!strcmp(attribs[i], "sample_rate")) {
                sample_rate_ = std::stoi(attribs[++i]);
            } else if(!strcmp(attribs[i], "bit_wdith")) {
                bit_width_ = std::stoi(attribs[++i]);
            } else if(!strcmp(attribs[i], "channel_count")) {
                channel_count_ = std::stoi(attribs[++i]);
            } else {
                QAL_ERR(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else {
        QAL_ERR(LOG_TAG, "Invalid tag %s", tag);
    }
}

void SecondStageCfg::HandleEndTag(const char* tag) {
    /* no-op */
}

void SecondStageCfg::HandleCharData(const char* data) {
    /* no-op */
}


SoundModelConfig::SoundModelConfig() :
    merge_first_stage_sound_models_(false),
    sample_rate_(16000),
    bit_width_(16),
    out_channels_(1),
    capture_keyword_(2000),
    client_capture_read_delay_(2000),
    curr_child_(nullptr)
{
}

std::shared_ptr<SecondStageCfg> SoundModelConfig::
    GetArmSsUsecase(uint32_t sm_id) const {
    auto armSsUc = arm_ss_uc_list_.find(sm_id);
    if (armSsUc != arm_ss_uc_list_.end())
        return armSsUc->second;
    else
        return nullptr;
}

void SoundModelConfig::HandleCharData(const char* data) {

}

void SoundModelConfig::HandleStartTag(const char* tag, const char** attribs) {

    /* Delegate to child element if currently active */
    if (curr_child_) {
        curr_child_->HandleStartTag(tag, attribs);
        return;
    }

    if (!strcmp(tag, "arm_ss_usecase")) {
        curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
            std::make_shared<SecondStageCfg>());
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
            } else {
                QAL_ERR(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else {
        QAL_ERR(LOG_TAG, "Invalid tag %d", (char *)tag);
    }
}

void SoundModelConfig::HandleEndTag(const char* tag) {

    if (!strcmp(tag, "arm_ss_usecase")) {
        std::shared_ptr<SecondStageCfg> sec_stg_cfg(
            std::dynamic_pointer_cast<SecondStageCfg>(curr_child_));
        const auto res = arm_ss_uc_list_.insert(
            std::make_pair(sec_stg_cfg->GetSmId(), sec_stg_cfg));
        if (!res.second)
            QAL_ERR(LOG_TAG, "Failed to insert to map");
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

    if (!strcmp(tag, "sound_model_config")) {
        curr_child_ = std::static_pointer_cast<SoundTriggerXml>(
            std::make_shared<SoundModelConfig>());
        return;
    }

    if (!strcmp(tag, "param")) {
        uint32_t i = 0;
        while (attribs[i]) {
            if (!attribs[i]) {
                QAL_ERR("missing attrib value for tag %s", tag);
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
                QAL_ERR(LOG_TAG, "Invalid attribute %s", attribs[i++]);
            }
            ++i; /* move to next attribute */
        }
    } else if (!strcmp(tag, "common_config")) {
         QAL_DBG(LOG_TAG, "Got tag %s", tag);
    } else {
        QAL_ERR(LOG_TAG, "Invalid tag %s", tag);
    }
}

void SoundTriggerPlatformInfo::HandleEndTag(const char* tag) {
    if (!strcmp(tag, "sound_model_config")) {
        std::shared_ptr<SoundModelConfig> sm_cfg(
        std::dynamic_pointer_cast<SoundModelConfig>(curr_child_));
        const auto res = sound_model_cfg_list_.insert(
            std::make_pair(sm_cfg->GetUUID(), sm_cfg));
        if (!res.second)
            QAL_ERR(LOG_TAG, "Failed to insert to map");
        curr_child_ = nullptr;
    }

    if (curr_child_)
        curr_child_->HandleEndTag(tag);

    return;
}

void SoundTriggerPlatformInfo::HandleCharData(const char* data) {
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
