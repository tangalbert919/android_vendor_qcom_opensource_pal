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

#define ATRACE_TAG (ATRACE_TAG_AUDIO | ATRACE_TAG_HAL)
#define LOG_TAG "PAL: SoundTriggerEngineGsl"

#include "SoundTriggerEngineGsl.h"

#include <cutils/trace.h>

#include "Session.h"
#include "Stream.h"
#include "StreamSoundTrigger.h"
#include "ResourceManager.h"
#include "kvh2xml.h"

// TODO: find another way to print debug logs by default
#define ST_DBG_LOGS
#ifdef ST_DBG_LOGS
#define PAL_DBG(LOG_TAG,...)  PAL_INFO(LOG_TAG,__VA_ARGS__)
#endif

std::map<st_module_type_t,std::shared_ptr<SoundTriggerEngineGsl>>
                 SoundTriggerEngineGsl::eng_;

void SoundTriggerEngineGsl::EventProcessingThread(
    SoundTriggerEngineGsl *gsl_engine) {

    int32_t status = 0;

    PAL_INFO(LOG_TAG, "Enter. start thread loop");
    if (!gsl_engine) {
        PAL_ERR(LOG_TAG, "Invalid sound trigger engine");
        return;
    }

    std::unique_lock<std::mutex> lck(gsl_engine->mutex_);
    while (!gsl_engine->exit_thread_) {
        PAL_VERBOSE(LOG_TAG, "waiting on cond");
        gsl_engine->cv_.wait(lck);
        PAL_DBG(LOG_TAG, "done waiting on cond");

        if (gsl_engine->exit_thread_) {
            PAL_VERBOSE(LOG_TAG, "Exit thread");
            break;
        }

        if (gsl_engine->module_type_ != ST_MODULE_TYPE_PDK5) {
            StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>
                                     (gsl_engine->GetDetectedStream());
            if (s) {
               if (gsl_engine->capture_requested_) {
                    gsl_engine->StartBuffering(s);
                } else {
                    status = gsl_engine->UpdateSessionPayload(ENGINE_RESET);
                    gsl_engine->CheckAndSetDetectionConfLevels(s);
                    s->SetEngineDetectionState(GMM_DETECTED);
                }
            }
        } else {
            PAL_DBG(LOG_TAG, "Detection happened for SVA5");
            for (int i = 0;
                i < gsl_engine->detection_event_info_multi_model_.
                               num_detected_models; i++) {
                StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>
                                (gsl_engine->GetDetectedStream(
                                 gsl_engine->detection_event_info_multi_model_.
                                 detected_model_stats[i].
                                 detected_model_id));
                if (s) {
                    if (gsl_engine->capture_requested_) {
                        gsl_engine->StartBuffering(s);
                    } else {
                        status = gsl_engine->UpdateSessionPayload(ENGINE_RESET);
                        s->SetEngineDetectionState(GMM_DETECTED);
                    }
                }
            }
        }
     }
    PAL_DBG(LOG_TAG, "Exit");
}

void SoundTriggerEngineGsl::CheckAndSetDetectionConfLevels(Stream *s) {

   StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);

    PAL_DBG(LOG_TAG, "Enter");
    if (detection_event_info_.num_confidence_levels <
            eng_sm_info_->GetConfLevelsSize()) {
        PAL_ERR(LOG_TAG, "detection event conf lvls %d < eng conf lvl size %d",
            detection_event_info_.num_confidence_levels,
            eng_sm_info_->GetConfLevelsSize());
        return;
    }
    /* Reset any cached previous detection conf level values */
    st->GetSoundModelInfo()->ResetDetConfLevels();

    /* Extract the stream conf level values from SPF detection payload */
    for (uint32_t i = 0; i < eng_sm_info_->GetConfLevelsSize(); i++) {
        if (!detection_event_info_.confidence_levels[i])
            continue;
        for (uint32_t j = 0; j < st->GetSoundModelInfo()->GetConfLevelsSize(); j++) {
            if (!strcmp(st->GetSoundModelInfo()->GetConfLevelsKwUsers()[j],
                 eng_sm_info_->GetConfLevelsKwUsers()[i])) {
                 st->GetSoundModelInfo()->UpdateDetConfLevel(j,
                   detection_event_info_.confidence_levels[i]);
            }
        }
    }

    for (uint32_t i = 0; i < st->GetSoundModelInfo()->GetConfLevelsSize(); i++)
        PAL_DBG(LOG_TAG, "det_cf_levels[%d]-%d", i,
            st->GetSoundModelInfo()->GetDetConfLevels()[i]);
}

int32_t SoundTriggerEngineGsl::StartBuffering(Stream *s) {
    int32_t status = 0;
    int32_t size = 0;
    struct pal_buffer buf;
    size_t input_buf_size = 0;
    size_t input_buf_num = 0;
    size_t output_buf_size = 0;
    size_t output_buf_num = 0;
    uint64_t timestamp = 0;
    uint64_t start_timestamp = 0;
    uint64_t end_timestamp = 0;
    size_t start_index = 0;
    size_t end_index = 0;
    size_t total_read_size = 0;
    size_t ftrt_size = 0;
    bool timestamp_recorded = false;
    bool event_notified = false;

    PAL_DBG(LOG_TAG, "Enter");
    s->getBufInfo(&input_buf_size, &input_buf_num,
                               &output_buf_size, &output_buf_num);
    std::memset(&buf, 0, sizeof(struct pal_buffer));
    buf.size = input_buf_size * input_buf_num;
    buf.buffer = (uint8_t *)calloc(1, buf.size);
    if (!buf.buffer) {
        PAL_ERR(LOG_TAG, "buf.buffer allocation failed");
        status = -ENOMEM;
        goto exit;
    }
    buf.ts = (struct timespec *)calloc(1, sizeof(struct timespec));
    if (!buf.ts) {
        PAL_ERR(LOG_TAG, "buf.ts allocation failed");
        status = -ENOMEM;
        goto exit;
    }
    buffer_->reset();

    if (module_type_ != ST_MODULE_TYPE_PDK5)
        ftrt_size = UsToBytes(detection_event_info_.ftrt_data_length_in_us);
    else
        ftrt_size = UsToBytes(detection_event_info_multi_model_.ftrt_data_length_in_us);

    ATRACE_BEGIN("stEngine: read FTRT data");

    while (!exit_buffering_) {
        PAL_VERBOSE(LOG_TAG, "request read %zu from gsl", buf.size);
        // read data from session
        if (buffer_->getFreeSize() >= buf.size) {
            ATRACE_BEGIN("stEngine: lab read");
            if (total_read_size < ftrt_size &&
                ftrt_size - total_read_size < buf.size) {
                buf.size = ftrt_size - total_read_size;
                status = session_->read(s, SHMEM_ENDPOINT, &buf, &size);
                buf.size = input_buf_size * input_buf_num;
            } else {
                status = session_->read(s, SHMEM_ENDPOINT, &buf, &size);
            }
            ATRACE_END();
            if (status) {
                break;
            }
            PAL_DBG(LOG_TAG, "requested %zu, read %d", buf.size, size);
            total_read_size += size;
        }
        // write data to ring buffer
        if (size) {
            size_t ret = buffer_->write(buf.buffer, size);
            PAL_INFO(LOG_TAG, "%zu written to ring buffer", ret);

            if (!timestamp_recorded) {
                timestamp = ((uint64_t)buf.ts->tv_sec * 1000000000 +
                            (uint64_t)buf.ts->tv_nsec) / 1000;

                if (module_type_ != ST_MODULE_TYPE_PDK5) {
                    start_timestamp =
                     (uint64_t)detection_event_info_.kw_start_timestamp_lsw +
                     ((uint64_t)detection_event_info_.kw_start_timestamp_msw << 32);
                    end_timestamp =
                     (uint64_t)detection_event_info_.kw_end_timestamp_lsw +
                     ((uint64_t)detection_event_info_.kw_end_timestamp_msw << 32);
                } else {
                    start_timestamp = ((uint64_t)detection_event_info_multi_model_.
                                        detected_model_stats[0].
                                        kw_start_timestamp_lsw) +
                                      ((uint64_t)detection_event_info_multi_model_.
                                        detected_model_stats[0].
                                        kw_start_timestamp_msw<<32);
                    end_timestamp = ((uint64_t)detection_event_info_multi_model_.
                                        detected_model_stats[0].
                                        kw_end_timestamp_lsw) +
                                    ((uint64_t)detection_event_info_multi_model_.
                                        detected_model_stats[0].
                                        kw_end_timestamp_msw<<32);
                    PAL_DBG(LOG_TAG, "start_timestamp_lsw : %u, start_timestamp_msw : %u",
                    detection_event_info_multi_model_.detected_model_stats[0].kw_start_timestamp_lsw,
                    detection_event_info_multi_model_.detected_model_stats[0].kw_start_timestamp_msw);

                    PAL_DBG(LOG_TAG, "end_timestamp_lsw : %u, end_timestamp_msw : %u",
                    detection_event_info_multi_model_.detected_model_stats[0].kw_end_timestamp_lsw,
                    detection_event_info_multi_model_.detected_model_stats[0].kw_end_timestamp_msw);
                }
                PAL_DBG(LOG_TAG, "Timestamp : %zu", (size_t)timestamp);
                start_index = UsToBytes(start_timestamp - timestamp);
                end_index = UsToBytes(end_timestamp - timestamp);
                PAL_DBG(LOG_TAG, "start_index : %zu, end_index : %zu", start_index, end_index);
                buffer_->updateIndices(start_index, end_index);
                timestamp_recorded = true;
            }
        }

        // notify client until ftrt data read
        if (!event_notified && total_read_size >= ftrt_size) {
            PAL_INFO(LOG_TAG, "FTRT data read done! total_read_size %zu, ftrt_size %zu",
                    total_read_size, ftrt_size);
            ATRACE_END();

            if (module_type_ != ST_MODULE_TYPE_PDK5) {
                StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>
                                                     (GetDetectedStream());
                if (s) {
                    this->CheckAndSetDetectionConfLevels(s);
                    s->SetEngineDetectionState(GMM_DETECTED);
                }
            } else {
                for ( int i = 0;
                    i < detection_event_info_multi_model_.num_detected_models;
                    i++){
                    StreamSoundTrigger *s = dynamic_cast<StreamSoundTrigger *>
                                            (GetDetectedStream(
                                            detection_event_info_multi_model_.
                                            detected_model_stats[i].
                                            detected_model_id));

                    if (s)
                        s->SetEngineDetectionState(GMM_DETECTED);
                }
            }
            event_notified = true;
        }
    }

exit:
    if (buf.buffer) {
        free(buf.buffer);
    }
    if (buf.ts) {
        free(buf.ts);
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::ParseDetectionPayloadSVA5(void *event_data) {
    int32_t status = 0;
    uint32_t payload_size = 0;
    uint32_t parsed_size = 0;
    uint32_t event_size = 0;
    uint32_t keyId = 0;
    uint8_t *ptr = nullptr;
    struct event_id_detection_engine_generic_info_t *generic_info = nullptr;
    struct detection_event_info_header_t *event_header = nullptr;
    struct ftrt_data_info_t *ftrt_info = nullptr;
    struct voice_ui_multi_model_result_info_t *multi_model_result = nullptr;
    struct model_stats *model_stat = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    if (!event_data) {
        PAL_ERR(LOG_TAG, "Invalid event data");
        return -EINVAL;
    }

    std::memset(&detection_event_info_multi_model_, 0,
                    sizeof(struct voice_ui_multi_model_result_info_t));

    generic_info = (struct event_id_detection_engine_generic_info_t *)
                    event_data;
    payload_size = sizeof(struct event_id_detection_engine_generic_info_t);
    event_size = generic_info->payload_size;
    ptr = (uint8_t *)event_data + payload_size;

    if (!event_size) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid detection payload");
        goto exit;
    }

    PAL_INFO(LOG_TAG, "event_size = %u", event_size);

    while (parsed_size < event_size) {
        PAL_DBG(LOG_TAG, "parsed_size = %u, event_size = %u", parsed_size,
                                                              event_size);
        event_header = (struct detection_event_info_header_t *)ptr;
        keyId = event_header->key_id;
        payload_size = event_header->payload_size;
        ptr += sizeof(struct detection_event_info_header_t);
        parsed_size += sizeof(struct detection_event_info_header_t);

        switch (keyId) {
            case KEY_ID_FTRT_DATA_INFO :
                PAL_DBG(LOG_TAG, "ftrt structure size : %u", payload_size);

                ftrt_info = (struct ftrt_data_info_t *)ptr;
                detection_event_info_multi_model_.ftrt_data_length_in_us =
                                        ftrt_info->ftrt_data_length_in_us;
                PAL_DBG(LOG_TAG, "ftrt_data_length_in_us = %u",
                detection_event_info_multi_model_.ftrt_data_length_in_us);
                break;

            case KEY_ID_VOICE_UI_MULTI_MODEL_RESULT_INFO :
                PAL_DBG(LOG_TAG, "voice_ui_multi_model_result_info : %u",
                        payload_size );

                multi_model_result = (struct voice_ui_multi_model_result_info_t *)
                                      ptr;
                detection_event_info_multi_model_.num_detected_models =
                                 multi_model_result->num_detected_models;
                PAL_DBG(LOG_TAG, "Number of detected models : %d",
                detection_event_info_multi_model_.num_detected_models);

                model_stat = (struct model_stats *)(ptr +
                             sizeof(struct voice_ui_multi_model_result_info_t));
                for (int i = 0; i < detection_event_info_multi_model_.
                                    num_detected_models; ++i) {

                    detection_event_info_multi_model_.detected_model_stats[i].
                    detected_model_id = model_stat->detected_model_id;

                    detection_event_info_multi_model_.detected_model_stats[i].
                    detected_keyword_id = model_stat->detected_keyword_id + 1;
                    PAL_DBG(LOG_TAG, "detected keyword id : %u",
                            detection_event_info_multi_model_.detected_model_stats[i].
                            detected_keyword_id );

                    detection_event_info_multi_model_.detected_model_stats[i].
                    best_channel_idx = model_stat->best_channel_idx;

                    detection_event_info_multi_model_.detected_model_stats[i].
                    best_confidence_level = model_stat->best_confidence_level;

                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_start_timestamp_lsw = model_stat->kw_start_timestamp_lsw;
                    PAL_DBG(LOG_TAG, "kw_start_timestamp_lsw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_start_timestamp_lsw);

                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_start_timestamp_msw = model_stat->kw_start_timestamp_msw;
                    PAL_DBG(LOG_TAG, "kw_start_timestamp_msw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_start_timestamp_msw);

                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_end_timestamp_lsw = model_stat->kw_end_timestamp_lsw;
                    PAL_DBG(LOG_TAG, "kw_end_timestamp_lsw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_end_timestamp_lsw);


                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_end_timestamp_msw = model_stat->kw_end_timestamp_msw;
                    PAL_DBG(LOG_TAG, "kw_end_timestamp_msw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    kw_end_timestamp_msw);


                    detection_event_info_multi_model_.detected_model_stats[i].
                    detection_timestamp_lsw = model_stat->detection_timestamp_lsw;
                    PAL_DBG(LOG_TAG, "detection_timestamp_lsw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    detection_timestamp_lsw);

                    detection_event_info_multi_model_.detected_model_stats[i].
                    detection_timestamp_msw = model_stat->detection_timestamp_msw;
                    PAL_DBG(LOG_TAG, "detection_timestamp_msw : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    detection_timestamp_msw);

                    PAL_INFO(LOG_TAG," Detection made for model id : %u",
                    detection_event_info_multi_model_.detected_model_stats[i].
                    detected_model_id);
                    model_stat += sizeof(struct model_stats);
                }
                break;
            default :
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid key id %u status %d", keyId, status);
                goto exit;
        }
        ptr += payload_size;
        parsed_size += payload_size;

    }

exit :
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::ParseDetectionPayload(void *event_data) {
    int32_t status = 0;
    int32_t i = 0;
    uint32_t parsed_size = 0;
    uint32_t payload_size = 0;
    uint32_t event_size = 0;
    uint8_t *ptr = nullptr;
    struct event_id_detection_engine_generic_info_t *generic_info = nullptr;
    struct detection_event_info_header_t *event_header = nullptr;
    struct confidence_level_info_t *confidence_info = nullptr;
    struct keyword_position_info_t *keyword_position_info = nullptr;
    struct detection_timestamp_info_t *detection_timestamp_info = nullptr;
    struct ftrt_data_info_t *ftrt_info = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    if (!event_data) {
        PAL_ERR(LOG_TAG, "Invalid event data");
        return -EINVAL;
    }

    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        std::memset(&detection_event_info_, 0, sizeof(struct detection_event_info));

        generic_info =
            (struct event_id_detection_engine_generic_info_t *)event_data;
        payload_size = sizeof(struct event_id_detection_engine_generic_info_t);
        detection_event_info_.status = generic_info->status;
        event_size = generic_info->payload_size;
        ptr = (uint8_t *)event_data + payload_size;
        PAL_INFO(LOG_TAG, "status = %u, event_size = %u",
                 detection_event_info_.status, event_size);
        if (status || !event_size) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "Invalid detection payload");
            goto exit;
        }

        // parse variable payload
        while (parsed_size < event_size) {
            PAL_DBG(LOG_TAG, "parsed_size = %u, event_size = %u",
                    parsed_size, event_size);
            event_header = (struct detection_event_info_header_t *)ptr;
            uint32_t keyId = event_header->key_id;
            payload_size = event_header->payload_size;
            PAL_DBG(LOG_TAG, "key id = %u, payload_size = %u",
                    keyId, payload_size);
            ptr += sizeof(struct detection_event_info_header_t);
            parsed_size += sizeof(struct detection_event_info_header_t);

            switch (keyId) {
            case KEY_ID_CONFIDENCE_LEVELS_INFO:
                confidence_info = (struct confidence_level_info_t *)ptr;
                detection_event_info_.num_confidence_levels =
                    confidence_info->number_of_confidence_values;
                PAL_DBG(LOG_TAG, "num_confidence_levels = %u",
                        detection_event_info_.num_confidence_levels);
                for (i = 0; i < detection_event_info_.num_confidence_levels; i++) {
                    detection_event_info_.confidence_levels[i] =
                        confidence_info->confidence_levels[i];
                    PAL_VERBOSE(LOG_TAG, "confidence_levels[%d] = %u", i,
                                detection_event_info_.confidence_levels[i]);
                }
                break;
            case KEY_ID_KWD_POSITION_INFO:
                keyword_position_info = (struct keyword_position_info_t *)ptr;
                detection_event_info_.kw_start_timestamp_lsw =
                    keyword_position_info->kw_start_timestamp_lsw;
                detection_event_info_.kw_start_timestamp_msw =
                    keyword_position_info->kw_start_timestamp_msw;
                detection_event_info_.kw_end_timestamp_lsw =
                    keyword_position_info->kw_end_timestamp_lsw;
                detection_event_info_.kw_end_timestamp_msw =
                    keyword_position_info->kw_end_timestamp_msw;
                PAL_DBG(LOG_TAG, "start_lsw = %u, start_msw = %u, "
                        "end_lsw = %u, end_msw = %u",
                        detection_event_info_.kw_start_timestamp_lsw,
                        detection_event_info_.kw_start_timestamp_msw,
                        detection_event_info_.kw_end_timestamp_lsw,
                        detection_event_info_.kw_end_timestamp_msw);
                break;
            case KEY_ID_TIMESTAMP_INFO:
                detection_timestamp_info = (struct detection_timestamp_info_t *)ptr;
                detection_event_info_.detection_timestamp_lsw =
                    detection_timestamp_info->detection_timestamp_lsw;
                detection_event_info_.detection_timestamp_msw =
                    detection_timestamp_info->detection_timestamp_msw;
                PAL_DBG(LOG_TAG, "timestamp_lsw = %u, timestamp_msw = %u",
                        detection_event_info_.detection_timestamp_lsw,
                        detection_event_info_.detection_timestamp_msw);
                break;
            case KEY_ID_FTRT_DATA_INFO:
                ftrt_info = (struct ftrt_data_info_t *)ptr;
                detection_event_info_.ftrt_data_length_in_us =
                    ftrt_info->ftrt_data_length_in_us;
                PAL_DBG(LOG_TAG, "ftrt_data_length_in_us = %u",
                        detection_event_info_.ftrt_data_length_in_us);
                break;
            default:
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "Invalid key id %u status %d", keyId, status);
                goto exit;
            }
            ptr += payload_size;
            parsed_size += payload_size;
        }
    } else {
        return ParseDetectionPayloadSVA5(event_data);
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

Stream* SoundTriggerEngineGsl::GetDetectedStream(uint32_t model_id) {

    StreamSoundTrigger *st = nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    if (eng_streams_.empty()) {
        PAL_ERR(LOG_TAG, "Unexpected, No streams attached to engine!");
        return nullptr;
    }
    /*
     * If only single stream exists, this detection is not for merged/multi
     * sound model, hence return this as only available stream
     */
    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        if (eng_streams_.size() == 1) {
            st = dynamic_cast<StreamSoundTrigger *>(eng_streams_[0]);
            if (st->GetCurrentStateId() == ST_STATE_ACTIVE ||
                st->GetCurrentStateId() == ST_STATE_DETECTED ||
                st->GetCurrentStateId() == ST_STATE_BUFFERING) {
                return eng_streams_[0];
            } else {
                PAL_ERR(LOG_TAG, "Detected stream is not in active state");
                return nullptr;
            }
        }

        if (detection_event_info_.num_confidence_levels <
                eng_sm_info_->GetNumKeyPhrases()) {
            PAL_ERR(LOG_TAG, "detection event conf levels %d < num of keyphrases %d",
                detection_event_info_.num_confidence_levels,
                eng_sm_info_->GetNumKeyPhrases());
            return nullptr;
        }

        /*
         * The DSP payload contains the keyword conf levels from the beginning.
         * Only one keyword conf level is expected to be non-zero from keyword
         * detection. Find non-zero conf level up to number of keyphrases and
         * if one is found, match it to the corresponding keyphrase from list
         * of streams to obtain the detected stream.
         */
        for (uint32_t i = 0; i < eng_sm_info_->GetNumKeyPhrases(); i++) {
            if (!detection_event_info_.confidence_levels[i])
                continue;
            for (uint32_t j = 0; j < eng_streams_.size(); j++) {
                st = dynamic_cast<StreamSoundTrigger *>(eng_streams_[j]);
                for (uint32_t k = 0; k < st->GetSoundModelInfo()->GetNumKeyPhrases(); k++) {
                    if (!strcmp(eng_sm_info_->GetKeyPhrases()[i],
                                st->GetSoundModelInfo()->GetKeyPhrases()[k])) {
                        if (st->GetCurrentStateId() == ST_STATE_ACTIVE ||
                                st->GetCurrentStateId() == ST_STATE_DETECTED ||
                                st->GetCurrentStateId() == ST_STATE_BUFFERING) {
                            return eng_streams_[j];
                        } else {
                            PAL_ERR(LOG_TAG, "detected stream is not active");
                            return nullptr;
                        }
                    }
                }
            }
        }
    } else {
        st = dynamic_cast<StreamSoundTrigger *>(mid_stream_map_[model_id]);
        if (!st){
            PAL_ERR(LOG_TAG, "Invalid model id = %u", model_id);
        }
        return st;
    }
    return nullptr;
}

SoundTriggerEngineGsl::SoundTriggerEngineGsl(
    Stream *s,
    uint32_t id,
    listen_model_indicator_enum type,
    st_module_type_t module_type) {

    struct pal_stream_attributes sAttr;
    std::shared_ptr<ResourceManager> rm = nullptr;
    engine_id_ = id;
    engine_type_ = type;
    module_type_ = module_type;
    exit_thread_ = false;
    exit_buffering_ = false;
    capture_requested_ = false;
    stream_handle_ = s;
    sm_data_ = nullptr;
    reader_ = nullptr;
    buffer_ = nullptr;
    is_qcva_uuid_ = false;
    is_qcmd_uuid_ = false;
    ses_started_ = false;
    custom_data = nullptr;
    custom_data_size = 0;
    custom_detection_event = nullptr;
    custom_detection_event_size = 0;
    builder_ = new PayloadBuilder();
	eng_sm_info_ = new SoundModelInfo();
    dev_disconnect_count_ = 0;

    std::memset(&detection_event_info_, 0, sizeof(struct detection_event_info));
    std::memset(&sva5_wakeup_config_, 0, sizeof(sva5_wakeup_config_));

    PAL_DBG(LOG_TAG, "Enter");
    StreamSoundTrigger *st_str = dynamic_cast<StreamSoundTrigger *>(s);
    int32_t status = st_str->GetEngineConfig(sample_rate_,
        bit_width_, channels_, type);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get engine config");
        throw std::runtime_error("Failed to get engine config");
    }

    status = st_str->GetModuleIds(module_tag_ids_, param_ids_);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to update module ids");
        throw std::runtime_error("Failed to update module ids");
    }

    status = st_str->CheckVendorUUID(&is_qcva_uuid_, &is_qcmd_uuid_);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to check vendor uuid");
        throw std::runtime_error("Failed to check vendor uuid");
    }
    // Create session
    rm = ResourceManager::getInstance();
    if (!rm) {
        PAL_ERR(LOG_TAG, "Failed to get ResourceManager instance");
        throw std::runtime_error("Failed to get ResourceManager instance");
    }
    stream_handle_->getStreamAttributes(&sAttr);
    session_ = Session::makeSession(rm, &sAttr);
    if (!session_) {
        PAL_ERR(LOG_TAG, "Failed to create session");
        throw std::runtime_error("Failed to create session");
    }

    session_->registerCallBack(HandleSessionCallBack, this);

    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        buffer_config_.hist_buffer_duration_in_ms = 0;
        buffer_config_.pre_roll_duration_in_ms = 0;
    } else {
        sva5_buffer_config_.hist_buffer_duration_in_ms = 0;
        sva5_buffer_config_.pre_roll_duration_in_ms = 0;
    }

    PAL_DBG(LOG_TAG, "Exit");
}

SoundTriggerEngineGsl::~SoundTriggerEngineGsl() {
    PAL_INFO(LOG_TAG, "Enter");
    /*
     * join thread if it is not joined, sometimes
     * stop/unload may fail before deconstruction.
     */
    if (buffer_thread_handler_.joinable()) {
        exit_buffering_ = true;
        std::unique_lock<std::mutex> lck(mutex_);
        exit_thread_ = true;
        cv_.notify_one();
        lck.unlock();
        buffer_thread_handler_.join();
        lck.lock();
        PAL_INFO(LOG_TAG, "Thread joined");
    }

    if (buffer_) {
        delete buffer_;
    }
    if (reader_) {
        delete reader_;
    }
    if (eng_sm_info_) {
        delete eng_sm_info_;
    }
    if (builder_) {
        delete builder_;
    }
    if (session_) {
        delete session_;
    }
    PAL_INFO(LOG_TAG, "Exit");
}

uint32_t SoundTriggerEngineGsl::GenerateModelID() {

     uint32_t range = MAX_MODEL_ID_VALUE - MIN_MODEL_ID_VALUE + 1 ;
     uint32_t id = (rand() % range) + 1 ;

     if (id == 0)
         id = 1;
     while (mid_stream_map_.find(id) != mid_stream_map_.end())
         id = (rand() % range) + 1 ;

    return id;
}

uint32_t SoundTriggerEngineGsl::AddModelID(Stream *s) {

    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);
    uint32_t model_id = GenerateModelID();
    mid_stream_map_[model_id] = s;
    st->SetModelId(model_id);

    return model_id ;
}


int32_t SoundTriggerEngineGsl::QuerySoundModel(SoundModelInfo *sm_info,
                                               uint8_t *data,
                                               uint32_t data_size) {

    listen_sound_model_header sml_header = {};
    listen_model_type model = {};
    listen_status_enum sml_ret = kSucess;
    uint32_t status = 0;
    std::shared_ptr<SoundModelLib>sml = SoundModelLib::GetInstance();

    PAL_VERBOSE(LOG_TAG, "Enter: sound model size %d", data_size);

    if (!sml || !sm_info) {
        PAL_ERR(LOG_TAG, "soundmodel lib handle or model info NULL");
        return -ENOSYS;
    }

    model.data = data;
    model.size = data_size;

    sml_ret = sml->GetSoundModelHeader_(&model, &sml_header);
    if (sml_ret != kSucess) {
        PAL_ERR(LOG_TAG, "GetSoundModelHeader_ failed, err %d ", sml_ret);
        return -EINVAL;
    }
    if (sml_header.numKeywords == 0) {
        PAL_ERR(LOG_TAG, "num keywords zero!");
        return -EINVAL;
    }

    if (sml_header.numActiveUserKeywordPairs < sml_header.numUsers) {
        PAL_ERR(LOG_TAG, "smlib activeUserKwPairs(%d) < total users (%d)",
                sml_header.numActiveUserKeywordPairs, sml_header.numUsers);
        goto cleanup;
    }
    if (sml_header.numUsers && !sml_header.userKeywordPairFlags) {
        PAL_ERR(LOG_TAG, "userKeywordPairFlags is NULL, numUsers (%d)",
                sml_header.numUsers);
        goto cleanup;
    }

    PAL_VERBOSE(LOG_TAG, "SML model.data %pK, model.size %d", model.data,
            model.size);
    status = sm_info->SetKeyPhrases(&model, sml_header.numKeywords);
    if (status)
        goto cleanup;

    status = sm_info->SetUsers(&model, sml_header.numUsers);
    if (status)
        goto cleanup;

    status = sm_info->SetConfLevels(sml_header.numActiveUserKeywordPairs,
                                    sml_header.numUsersSetPerKw,
                                    sml_header.userKeywordPairFlags);
    if (status)
        goto cleanup;

    sml_ret = sml->ReleaseSoundModelHeader_(&sml_header);
    if (sml_ret != kSucess) {
        PAL_ERR(LOG_TAG, "ReleaseSoundModelHeader failed, err %d ", sml_ret);
        status = -EINVAL;
        goto cleanup_1;
    }
    PAL_VERBOSE(LOG_TAG, "exit");
    return 0;

cleanup:
    sml_ret = sml->ReleaseSoundModelHeader_(&sml_header);
    if (sml_ret != kSucess)
        PAL_ERR(LOG_TAG, "ReleaseSoundModelHeader_ failed, err %d ", sml_ret);

cleanup_1:
    return status;
}

int32_t SoundTriggerEngineGsl::MergeSoundModels(uint32_t num_models,
             listen_model_type *in_models[],
             listen_model_type *out_model) {

    listen_status_enum sm_ret = kSucess;
    int32_t status = 0;
    std::shared_ptr<SoundModelLib>sml = SoundModelLib::GetInstance();

    if (!sml) {
        PAL_ERR(LOG_TAG, "soundmodel lib handle NULL");
        return -ENOSYS;
    }

    PAL_VERBOSE(LOG_TAG, "num_models to merge %d", num_models);
    sm_ret = sml->GetMergedModelSize_(num_models, in_models,
        &out_model->size);
    if ((sm_ret != kSucess) || !out_model->size) {
        PAL_ERR(LOG_TAG, "GetMergedModelSize failed, err %d, size %d",
            sm_ret, out_model->size);
        return -EINVAL;
    }
    PAL_INFO(LOG_TAG, "merged sound model size %d", out_model->size);

    out_model->data = (uint8_t *)calloc(1, out_model->size * sizeof(char));
    if (!out_model->data) {
        PAL_ERR(LOG_TAG, "Merged sound model allocation failed");
        return -ENOMEM;
    }

    sm_ret = sml->MergeModels_(num_models, in_models, out_model);
    if (sm_ret != kSucess) {
        PAL_ERR(LOG_TAG, "MergeModels failed, err %d", sm_ret);
        status = -EINVAL;
        goto cleanup;
    }
    if (!out_model->data || !out_model->size) {
        PAL_ERR(LOG_TAG, "MergeModels returned NULL data or size %d",
              out_model->size);
        status = -EINVAL;
        goto cleanup;
    }
    PAL_DBG(LOG_TAG, "Exit, status: %d", status);
    return 0;

cleanup:
    if (out_model->data) {
        free(out_model->data);
        out_model->data = nullptr;
        out_model->size = 0;
    }
    return status;
}

int32_t SoundTriggerEngineGsl::AddSoundModel(Stream *s, uint8_t *data,
                                              uint32_t data_size){

    int32_t status = 0;
    uint32_t num_models = 0;
    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);
    listen_model_type **in_models = nullptr;
    listen_model_type out_model = {};
    SoundModelInfo *sm_info;

    PAL_VERBOSE(LOG_TAG, "Enter");
    if (st->GetSoundModelInfo()->GetModelData()) {
        PAL_DBG(LOG_TAG, "Stream model already added");
        return 0;
    }

    if (!is_qcva_uuid_ && !is_qcmd_uuid_) {
        st->GetSoundModelInfo()->SetModelData(data, data_size);
        *eng_sm_info_ = *(st->GetSoundModelInfo());
        sm_merged_ = false;
        return 0;
    }

    /* Populate sound model info for the incoming stream model */
    status = QuerySoundModel(st->GetSoundModelInfo(), data, data_size);
    if (status) {
        PAL_ERR(LOG_TAG, "QuerySoundModel failed status: %d", status);
        return status;
    }

    st->GetSoundModelInfo()->SetModelData(data, data_size);

    /* Check for remaining stream sound models to merge */
    for (int i = 0; i < eng_streams_.size(); i++) {
        StreamSoundTrigger *sst = dynamic_cast<StreamSoundTrigger *>(eng_streams_[i]);
        if (s != eng_streams_[i] && sst && sst->GetSoundModelInfo()->GetModelData())
             num_models++;
    }

    if (!num_models) {
        PAL_DBG(LOG_TAG, "Copy model info from incoming stream to engine");
        *eng_sm_info_ = *(st->GetSoundModelInfo());
        sm_merged_ = false;
        return 0;
    }

    PAL_VERBOSE(LOG_TAG, "number of existing models: %d", num_models);
    /*
     * Merge this stream model with already existing merged model due to other
     * streams models.
     */
    if (!eng_sm_info_) {
        PAL_ERR(LOG_TAG, "eng_sm_info is NULL");
        status = -EINVAL;
        goto cleanup;
    }

    if (!eng_sm_info_->GetModelData()) {
        if (num_models == 1) {
            /*
             * Its not a merged model yet, but engine sm_data is valid
             * and must be pointing to single stream sm_data
             */
            PAL_ERR(LOG_TAG, "Model data is NULL, num_models: %d", num_models);
            status = -EINVAL;
            goto cleanup;
        } else if (!sm_merged_) {
            PAL_ERR(LOG_TAG, "Unexpected, no pre-existing merged model,"
                  "num current models %d", num_models);
            status = -EINVAL;
            goto cleanup;
        }
    }

    /* Merge this stream model with remaining streams models */
    num_models = 2;
    SoundModelInfo::AllocArrayPtrs((char***)&in_models, num_models,
                                   sizeof(listen_model_type));
    if (!in_models) {
        PAL_ERR(LOG_TAG, "in_models allocation failed");
        status = -ENOMEM;
        goto cleanup;
    }
    /* Add existing model */
    in_models[0]->data = eng_sm_info_->GetModelData();
    in_models[0]->size = eng_sm_info_->GetModelSize();
    /* Add incoming stream model */
    in_models[1]->data = data;
    in_models[1]->size = data_size;

    status = MergeSoundModels(num_models, in_models, &out_model);
    if (status) {
        PAL_ERR(LOG_TAG, "merge models failed");
        goto cleanup;
    }
    sm_info = new SoundModelInfo();
    sm_info->SetModelData(out_model.data, out_model.size);

    /* Populate sound model info for the merged stream models */
    status = QuerySoundModel(sm_info, out_model.data, out_model.size);
    if (status) {
        delete sm_info;
        goto cleanup;
    }

    if (out_model.size < eng_sm_info_->GetModelSize()) {
        PAL_ERR(LOG_TAG, "Unexpected, merged model sz %d < current sz %d",
            out_model.size, eng_sm_info_->GetModelSize());
        delete sm_info;
        status = -EINVAL;
        goto cleanup;
    }
    SoundModelInfo::FreeArrayPtrs((char **)in_models, num_models);
    in_models = nullptr;

    /* Update the new merged model */
    PAL_INFO(LOG_TAG, "Updated sound model: current size %d, new size %d",
        eng_sm_info_->GetModelSize(), out_model.size);
    *eng_sm_info_ = *sm_info;
    sm_merged_ = true;

    delete sm_info;
    PAL_DBG(LOG_TAG, "Exit: status %d", status);
    return 0;
cleanup:
    if (out_model.data)
        free(out_model.data);

    if (in_models)
        SoundModelInfo::FreeArrayPtrs((char **)in_models, num_models);

    return status;
}

int32_t SoundTriggerEngineGsl::DeleteFromMergedModel(char **keyphrases,
    uint32_t num_keyphrases, listen_model_type *in_model,
    listen_model_type *out_model) {

    listen_model_type merge_model = {};
    listen_status_enum sm_ret = kSucess;
    uint32_t out_model_sz = 0;
    int32_t status = 0;
    std::shared_ptr<SoundModelLib>sml = SoundModelLib::GetInstance();

    out_model->data = nullptr;
    out_model->size = 0;
    merge_model.data = in_model->data;
    merge_model.size = in_model->size;

    for (uint32_t i = 0; i < num_keyphrases; i++) {
        sm_ret = sml->GetSizeAfterDeleting_(&merge_model, keyphrases[i],
                                                   nullptr, &out_model_sz);
        if (sm_ret != kSucess) {
            PAL_ERR(LOG_TAG, "GetSizeAfterDeleting failed %d", sm_ret);
            status = -EINVAL;
            goto cleanup;
        }
        if (out_model_sz >= in_model->size) {
            PAL_ERR(LOG_TAG, "unexpected, GetSizeAfterDeleting returned size %d"
                  "not less than merged model size %d",
                  out_model_sz, in_model->size);
            status = -EINVAL;
            goto cleanup;
        }
        PAL_VERBOSE(LOG_TAG, "Size after deleting kw[%d] = %d", i, out_model_sz);
        if (!out_model->data) {
            /* Valid if deleting multiple keyphrases one after other */
            free(out_model->data);
            out_model->size = 0;
        }
        out_model->data = (uint8_t *)calloc(1, out_model_sz * sizeof(char));
        if (!out_model->data) {
            PAL_ERR(LOG_TAG, "Merge sound model allocation failed, size %d ",
                  out_model_sz);
            status = -ENOMEM;
            goto cleanup;
        }
        out_model->size = out_model_sz;

        sm_ret = sml->DeleteFromModel_(&merge_model, keyphrases[i],
                                              nullptr, out_model);
        if (sm_ret != kSucess) {
            PAL_ERR(LOG_TAG, "DeleteFromModel failed %d", sm_ret);
            status = -EINVAL;
            goto cleanup;
        }
        if (out_model->size != out_model_sz) {
            PAL_ERR(LOG_TAG, "unexpected, out_model size %d != expected size %d",
                  out_model->size, out_model_sz);
            status = -EINVAL;
            goto cleanup;
        }
        /* Used if deleting multiple keyphrases one after other */
        merge_model.data = out_model->data;
        merge_model.size = out_model->size;
    }
    return 0;

cleanup:
    if (out_model->data) {
        free(out_model->data);
        out_model->data = nullptr;
    }
    return status;
}

int32_t SoundTriggerEngineGsl::DeleteSoundModel(Stream *s) {

    int32_t status = 0;
    uint32_t num_models = 0;
    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);
    StreamSoundTrigger *rem_st = nullptr;
    listen_model_type in_model = {};
    listen_model_type out_model = {};
    SoundModelInfo *sm_info = nullptr;

    PAL_VERBOSE(LOG_TAG, "Enter");
    if (!st->GetSoundModelInfo()->GetModelData()) {
        PAL_DBG(LOG_TAG, "Stream model data already deleted");
        return 0;
    }

    PAL_VERBOSE(LOG_TAG, "sm_data %pK, sm_size %d",
          st->GetSoundModelInfo()->GetModelData(),
          st->GetSoundModelInfo()->GetModelSize());

    /* Check for remaining streams sound models to merge */
    for (int i = 0; i < eng_streams_.size(); i++) {
        StreamSoundTrigger *sst = dynamic_cast<StreamSoundTrigger *>(eng_streams_[i]);
        if (s != eng_streams_[i] && sst) {
             if (sst->GetSoundModelInfo() &&
                 sst->GetSoundModelInfo()->GetModelData()) {
                 rem_st = sst;
                 num_models++;
                 PAL_DBG(LOG_TAG, "num_models: %d", num_models);
             }
        }
    }

    if (num_models == 0) {
        PAL_DBG(LOG_TAG, "No remaining models");
        return 0;
    }
    if (num_models == 1) {
        PAL_DBG(LOG_TAG, "reuse only remaining stream model, size %d",
            rem_st->GetSoundModelInfo()->GetModelSize());
        /* If only one remaining stream model exists, re-use it */
        *eng_sm_info_ = *(rem_st->GetSoundModelInfo());
        wakeup_config_.num_active_models = eng_sm_info_->GetConfLevelsSize();
        for (int i = 0; i < eng_sm_info_->GetConfLevelsSize(); i++) {
            wakeup_config_.confidence_levels[i] = eng_sm_info_->GetConfLevels()[i];
            wakeup_config_.keyword_user_enables[i] =
                (wakeup_config_.confidence_levels[i] == 100) ? 0 : 1;
            PAL_DBG(LOG_TAG, "cf levels[%d] = %d", i, wakeup_config_.confidence_levels[i]);
        }
        sm_merged_ = false;
        return 0;
    }

    /*
     * Delete this stream model with already existing merged model due to other
     * streams models.
     */
    if (!sm_merged_ || !(eng_sm_info_->GetModelData())) {
        PAL_ERR(LOG_TAG, "Unexpected, no pre-existing merged model to delete from,"
              "num current models %d", num_models);
        goto cleanup;
    }

    /* Existing merged model from which the current stream model to be deleted */
    in_model.data = eng_sm_info_->GetModelData();
    in_model.size = eng_sm_info_->GetModelSize();

    status = DeleteFromMergedModel(st->GetSoundModelInfo()->GetKeyPhrases(),
        st->GetSoundModelInfo()->GetNumKeyPhrases(),
        &in_model, &out_model);

    if (status)
        goto cleanup;
    sm_info = new SoundModelInfo();
    sm_info->SetModelData(out_model.data, out_model.size);

    /* Update existing merged model info with new merged model */
    status = QuerySoundModel(sm_info, out_model.data,
                               out_model.size);
    if (status)
        goto cleanup;

    if (out_model.size > eng_sm_info_->GetModelSize()) {
        PAL_ERR(LOG_TAG, "Unexpected, merged model sz %d > current sz %d",
            out_model.size, eng_sm_info_->GetModelSize());
        delete sm_info;
        status = -EINVAL;
        goto cleanup;
    }

    PAL_INFO(LOG_TAG, "Updated sound model: current size %d, new size %d",
        eng_sm_info_->GetModelSize(), out_model.size);

    *eng_sm_info_ = *sm_info;
    sm_merged_ = true;

    return 0;

cleanup:
    if (out_model.data)
        free(out_model.data);

    return status;
}

int32_t SoundTriggerEngineGsl::UpdateEngineModel(Stream *s, uint8_t *data,
                                                 uint32_t data_size, bool add) {

    int32_t status = 0;

    if (add)
        status = AddSoundModel(s, data, data_size);
    else
        status = DeleteSoundModel(s);

    PAL_DBG(LOG_TAG, "Exit, status: %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::UpdateMergeConfLevelsPayload(
                               SoundModelInfo *src_sm_info,
                               bool set) {

    if (!src_sm_info) {
        PAL_ERR(LOG_TAG, "src sm info NULL");
        return -EINVAL;
    }

    if (!sm_merged_) {
        PAL_DBG(LOG_TAG, "Soundmodel is not merged, return");
        return 0;
    }

    if (src_sm_info->GetConfLevelsSize() > eng_sm_info_->GetConfLevelsSize()) {
        PAL_ERR(LOG_TAG, "Unexpected, stream conf levels sz > eng conf levels sz");
        return -EINVAL;
    }

    for (uint32_t i = 0; i < src_sm_info->GetConfLevelsSize(); i++)
        PAL_VERBOSE(LOG_TAG, "source cf levels[%d] = %d for %s", i,
            src_sm_info->GetConfLevels()[i], src_sm_info->GetConfLevelsKwUsers()[i]);

    /* Populate DSP merged sound model conf levels */
    for (uint32_t i = 0; i < src_sm_info->GetConfLevelsSize(); i++) {
        for (uint32_t j = 0; j < eng_sm_info_->GetConfLevelsSize(); j++) {
            if (!strcmp(eng_sm_info_->GetConfLevelsKwUsers()[j],
                        src_sm_info->GetConfLevelsKwUsers()[i])) {
                if (set) {
                    eng_sm_info_->UpdateConfLevel(j, src_sm_info->GetConfLevels()[i]);
                    PAL_DBG(LOG_TAG, "set: cf_levels[%d]=%d",
                          j, eng_sm_info_->GetConfLevels()[j]);
                } else {
                    eng_sm_info_->UpdateConfLevel(j, MAX_CONF_LEVEL_VALUE);
                    PAL_DBG(LOG_TAG, "reset: cf_levels[%d]=%d",
                          j, eng_sm_info_->GetConfLevels()[j]);
                }
            }
        }
    }

    for (uint32_t i = 0; i < eng_sm_info_->GetConfLevelsSize(); i++)
        PAL_ERR(LOG_TAG, "engine cf_levels[%d] = %d",
            i, eng_sm_info_->GetConfLevels()[i]);

    return 0;
}

int32_t SoundTriggerEngineGsl::UpdateMergeConfLevelsWithActiveStreams() {

    int32_t status = 0;
    StreamSoundTrigger *st_str = nullptr;

    for (int i = 0; i < eng_streams_.size(); i++) {
        st_str = dynamic_cast<StreamSoundTrigger *>(eng_streams_[i]);
        if (st_str && st_str->GetCurrentStateId() == ST_STATE_ACTIVE) {
            PAL_VERBOSE(LOG_TAG, "update merge conf levels with other active streams");
            status = UpdateMergeConfLevelsPayload(st_str->GetSoundModelInfo(),
                        true);
            if (status)
                return status;
        }
    }
    return status;
}

int32_t SoundTriggerEngineGsl::HandleMultiStreamLoad(Stream *s, uint8_t *data,
                                                     uint32_t data_size) {

    int32_t status = 0;
    bool restore_ses_state = false;

    PAL_DBG(LOG_TAG, "Enter");
    std::unique_lock<std::mutex> lck(mutex_);

    if (ses_started_) {
        this->ProcessStopRecognition(eng_streams_[0]);
        restore_ses_state = true;
    }

    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        /* Unload the current sound model */
        exit_thread_ = true;
        exit_buffering_ = true;
        if (buffer_thread_handler_.joinable()) {
            cv_.notify_one();
            lck.unlock();
            buffer_thread_handler_.join();
            lck.lock();
           PAL_INFO(LOG_TAG, "Thread joined");
        }
        status = session_->close(eng_streams_[0]);
        if (status)
            PAL_ERR(LOG_TAG, "Failed to close session, status = %d", status);

        /* Update the engine with merged sound model */
        status = UpdateEngineModel(s, data, data_size, true);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to update engine model, status = %d", status);
            goto exit;
        }

        /*
         * Sound model merge would have changed the order of merge conf levels,
         * which need to be re-updated for all current active streams, if any.
        */
        status = UpdateMergeConfLevelsWithActiveStreams();
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to update merge conf levels, status = %d", 
                                                                      status);
            goto exit;
        }

        /* Load the updated/merged sound model */
        status = session_->open(eng_streams_[0]);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to open session, status = %d", status);
            goto exit;
        }

        status = UpdateSessionPayload(LOAD_SOUND_MODEL);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to update session payload, status = %d",
                                                                    status);
            session_->close(eng_streams_[0]);
            goto exit;
        }

        exit_thread_ = false;
        buffer_thread_handler_ =
            std::thread(SoundTriggerEngineGsl::EventProcessingThread, this);

        if (!buffer_thread_handler_.joinable()) {
            PAL_ERR(LOG_TAG, "failed to create event processing thread, status = %d",
                    status);
            session_->close(eng_streams_[0]);
            status = -EINVAL;
            goto exit;
        }
    } else {
        status = UpdateSessionPayload(LOAD_MULTI_SOUND_MODEL);

       if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to update session payload, status = %d",
                                                                     status);
            session_->close(s);
            goto exit;
        }

     }

     if (restore_ses_state)
        status = ProcessStartRecognition(eng_streams_[0]);
exit:
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

uint32_t SoundTriggerEngineGsl::GetModelID(Stream *s) {

    for(std::map<uint32_t, Stream*>::iterator itr = mid_stream_map_.begin();
                                      itr != mid_stream_map_.end(); ++itr){
         if (s == itr->second)
             return itr->first;
    }
    return 0 ;
}

int32_t SoundTriggerEngineGsl::HandleMultiStreamUnloadSVA5(Stream *s) {

    int32_t status = 0;
    uint32_t model_id = 0;
    uint32_t deleted_enteries = 0;

    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);

    model_id = st->GetModelId();

    if (model_id == 0) {
        PAL_ERR(LOG_TAG, "Sound model not found");
        return -EINVAL;
    }

    deregister_config_.model_id = model_id;
    status = UpdateSessionPayload(DEREGISTER_MULTI_SOUND_MODEL);
    if (status != 0) {
        PAL_ERR(LOG_TAG,
        "Failed to update session payload for deregister multi sound model");
        return -EINVAL;
    }

    deleted_enteries = mid_stream_map_.erase(model_id);
    if (deleted_enteries == 0) {
        PAL_ERR(LOG_TAG, "Sound model not deleted");
        return 2;
    }

    return status;
}

int32_t SoundTriggerEngineGsl::HandleMultiStreamUnload(Stream *s) {

    int32_t status = 0;

    bool restore_ses_state = false;

    PAL_DBG(LOG_TAG, "Enter");
    std::unique_lock<std::mutex> lck(mutex_);

    if (ses_started_) {
        ProcessStopRecognition(eng_streams_[0]);
        restore_ses_state = true;
    }

    if (module_type_ == ST_MODULE_TYPE_PDK5) {
        status = HandleMultiStreamUnloadSVA5(s);
    } else {
        /* Unload the current sound model */
        exit_thread_ = true;
        exit_buffering_ = true;
        if (buffer_thread_handler_.joinable()) {
            cv_.notify_one();
            lck.unlock();
            buffer_thread_handler_.join();
            lck.lock();
            PAL_INFO(LOG_TAG, "Thread joined");
        }
        status = session_->close(eng_streams_[0]);
        if (status)
            PAL_ERR(LOG_TAG, "Failed to close session, status = %d", status);

        /* Update the engine with modified sound model after deletion */
        status = UpdateEngineModel(s, nullptr, 0, false);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to open session, status = %d", status);
            goto exit;
        }
        /*
         * Sound model merge would have changed the order of merge conf levels,
         * which need to be re-updated for all current active streams, if any.
         */
        status = UpdateMergeConfLevelsWithActiveStreams();
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to update merge conf levels, status = %d",
                                                                          status);
        goto exit;
        }

        /* Load the updated/merged sound model */
        status = session_->open(eng_streams_[0]);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to open session, status = %d", status);
            goto exit;
        }

        status = UpdateSessionPayload(LOAD_SOUND_MODEL);
        if (0 != status) {
            PAL_ERR(LOG_TAG, "Failed to update session payload, status = %d",
                                                                     status);
            session_->close(eng_streams_[0]);
            goto exit;
        }

        exit_thread_ = false;
        buffer_thread_handler_ =
            std::thread(SoundTriggerEngineGsl::EventProcessingThread, this);

        if (!buffer_thread_handler_.joinable()) {
            PAL_ERR(LOG_TAG, "failed to create event processing thread, status = %d",
                    status);
            session_->close(eng_streams_[0]);
            status = -EINVAL;
            goto exit;
        }
    }

    if (restore_ses_state)
        status = ProcessStartRecognition(eng_streams_[0]);
exit:
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::LoadSoundModel(Stream *s, uint8_t *data,
                                              uint32_t data_size) {
    int32_t status = 0;
    uint32_t model_id = 0;
    struct param_id_detection_engine_register_multi_sound_model_t *pdk_data =
           nullptr;

    PAL_DBG(LOG_TAG, "Enter");
    if (!data) {
        PAL_ERR(LOG_TAG, "Invalid sound model data status %d", status);
        status = -EINVAL;
        return status;
    }

    if (module_type_ == ST_MODULE_TYPE_PDK5) {
        model_id = AddModelID(s);

        pdk_data = (struct param_id_detection_engine_register_multi_sound_model_t *)
             malloc(sizeof(
             struct param_id_detection_engine_register_multi_sound_model_t) +
             data_size);

        if (pdk_data == nullptr) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for pdk_data");
            return -EINVAL;
        }

        PAL_DBG(LOG_TAG,
         "param_id_detection_engine_register_multi_sound_model :%zu, data_size : %u",
          sizeof(struct param_id_detection_engine_register_multi_sound_model_t),
          data_size);
        pdk_data->model_id = model_id;
        pdk_data->model_size = data_size;
        ar_mem_cpy(pdk_data->model, data_size, data, data_size);
        sm_data_ = (uint8_t *)pdk_data;
        data_size += (sizeof(param_id_detection_engine_register_multi_sound_model_t));
        sm_data_size_ = data_size;
    }
    std::unique_lock<std::mutex> lck(mutex_);

    /* Check whether any stream is already attached to this engine */
    if (CheckIfOtherStreamsAttached(s)) {
        lck.unlock();
        status = HandleMultiStreamLoad(s, data, data_size);
        lck.lock();
        goto exit;
    }

    status = session_->open(s);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to open session, status = %d", status);
        goto exit;
    }

    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        /* Update the engine with sound model */
        status = UpdateEngineModel(s, data, data_size, true);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to update engine model, status = %d", status);
            session_->close(s);
            goto exit;
        }

        status = UpdateSessionPayload(LOAD_SOUND_MODEL);
    } else  {
        status = UpdateSessionPayload(LOAD_MULTI_SOUND_MODEL);
    }
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to update session payload, status = %d", status);
        session_->close(s);
        goto exit;
    }

    exit_thread_ = false;
    buffer_thread_handler_ =
        std::thread(SoundTriggerEngineGsl::EventProcessingThread, this);

    if (!buffer_thread_handler_.joinable()) {
        PAL_ERR(LOG_TAG, "failed to create even processing thread, status = %d",
                status);
        session_->close(s);
        status = -EINVAL;
        goto exit;
    }
exit:
    if (!status)
        eng_streams_.push_back(s);

    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::UnloadSoundModel(Stream *s) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::unique_lock<std::mutex> lck(mutex_);

    /* Check whether any stream is already attached to this engine */
    if (CheckIfOtherStreamsAttached(s)) {
        lck.unlock();
        status = HandleMultiStreamUnload(s);
        lck.lock();
        goto exit;
    }

    exit_thread_ = true;
    exit_buffering_ = true;
    if (buffer_thread_handler_.joinable()) {
        cv_.notify_one();
        lck.unlock();
        buffer_thread_handler_.join();
        lck.lock();
        PAL_INFO(LOG_TAG, "Thread joined");
    }

    status = session_->close(s);
    if (status)
        PAL_ERR(LOG_TAG, "Failed to close session, status = %d", status);

    /* Delete the sound model in engine */
    status = UpdateEngineModel(s, nullptr, 0, false);
    if (status)
        PAL_ERR(LOG_TAG, "Failed to update engine model, status = %d", status);
exit:
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::ProcessStartRecognition(Stream *s) {

    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");

    exit_buffering_ = false;
    // release custom detection event before start
    if (custom_detection_event) {
        free(custom_detection_event);
        custom_detection_event = nullptr;
        custom_detection_event_size = 0;
    }

    if (is_qcva_uuid_ || is_qcmd_uuid_) {
        if (module_type_ == ST_MODULE_TYPE_PDK5)
            status = UpdateSessionPayload(WAKEUP_CONFIG_STAGE1_SVA5);
        else
            status = UpdateSessionPayload(WAKEUP_CONFIG);
    }
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to set wake up config, status = %d", status);
        goto exit;
    }

    if (module_type_ == ST_MODULE_TYPE_PDK5)
        status = UpdateSessionPayload(MULTI_MODEL_BUFFERING_CONFIG);
    else
        status = UpdateSessionPayload(BUFFERING_CONFIG);

    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to set wake-up buffer config, status = %d",
                status);
        goto exit;
    }

    status = session_->prepare(s);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to prepare session, status = %d", status);
        goto exit;
    }

    status = session_->start(s);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to start session, status = %d", status);
        goto exit;
    }

    ses_started_ = true;
exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StartRecognition(Stream *s) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);

    if (ses_started_)
        ProcessStopRecognition(eng_streams_[0]);

    status = ProcessStartRecognition(s);
    if (0 != status)
        PAL_ERR(LOG_TAG, "Failed to start recognition, status = %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::RestartRecognition(Stream *s) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    exit_buffering_ = true;
    std::lock_guard<std::mutex> lck(mutex_);
    UpdateEngineConfigOnRestart(s);
    if (buffer_) {
        buffer_->reset();
    }
    // release custom detection event before start
    if (custom_detection_event) {
        free(custom_detection_event);
        custom_detection_event_size = 0;
    }

    /*
     * TODO: This sequence RESET->STOP->START is currently required from spf
     * as ENGINE_RESET alone can't reset the graph (including DAM etc..) ready
     * for next detection.
     */
    status = UpdateSessionPayload(ENGINE_RESET);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to reset engine, status = %d",
                status);
    }

    status = session_->stop(s);
    if (!status) {
        exit_buffering_ = false;
        ses_started_ = false;
        status = session_->start(s);
        if (status) {
            PAL_ERR(LOG_TAG, "start session failed, status = %d",
                    status);
        } else {
            ses_started_ = true;
        }
    } else {
        PAL_ERR(LOG_TAG, "stop session failed, status = %d",
                status);
    }
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::ProcessStopRecognition(Stream *s) {
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    if (buffer_) {
        buffer_->reset();
    }

    /*
     * TODO: Currently spf requires ENGINE_RESET to close the DAM gate as stop
     * will not close the gate, rather just flushes the buffers, resulting in no
     * further detections.
     */
    status = UpdateSessionPayload(ENGINE_RESET);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to reset detection engine, status = %d",
                status);
    }

    status = session_->stop(s);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to stop session, status = %d", status);
    }

    ses_started_ = false;
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t SoundTriggerEngineGsl::StopRecognition(Stream *s) {
    int32_t status = 0;
    bool restore_ses_state = false;

    PAL_DBG(LOG_TAG, "Enter");

    exit_buffering_ = true;

    std::lock_guard<std::mutex> lck(mutex_);

    if (ses_started_)
        restore_ses_state = true;

    status = ProcessStopRecognition(s);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to stop recognition, status = %d", status);
        goto exit;
    }

    if (CheckIfOtherStreamsAttached(s)) {
        if (restore_ses_state) {
            PAL_INFO(LOG_TAG, "Other stream is active, restart engine recognition");
            UpdateEngineConfigOnStop(s);
            status = ProcessStartRecognition(eng_streams_[0]);
            if (0 != status) {
                PAL_ERR(LOG_TAG, "Failed to start recognition, status = %d", status);
                goto exit;
            }
        }
    }
exit:
    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

bool SoundTriggerEngineGsl::CheckIfOtherStreamsAttached(Stream *s) {
    for (uint32_t i = 0; i < eng_streams_.size(); i++)
        if (s != eng_streams_[i])
            return true;

    return false;
}

int32_t SoundTriggerEngineGsl::UpdateConfLevels(
    Stream *s,
    struct pal_st_recognition_config *config,
    uint8_t *conf_levels,
    uint32_t num_conf_levels) {

    int32_t status = 0;
    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);

    std::lock_guard<std::mutex> lck(mutex_);
    if (!is_qcva_uuid_ && !is_qcmd_uuid_) {
        custom_data_size = config->data_size;
        custom_data = (uint8_t *)calloc(1, custom_data_size);
        if (!custom_data) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for custom data");
            status = -ENOMEM;
            goto exit;
        }
        ar_mem_cpy(custom_data, custom_data_size,
            (uint8_t *)config + config->data_offset, custom_data_size);
        goto exit;
    }

    if (!config || !conf_levels) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid config or conf levels, status %d", status);
        goto exit;
    }

    PAL_VERBOSE(LOG_TAG, "Enter, config: %pK", config);

    if (module_type_ != ST_MODULE_TYPE_PDK5
        && st->GetSoundModelInfo()->GetConfLevelsSize() != num_conf_levels) {
        PAL_ERR(LOG_TAG, "Unexpected, stream cf levels %d != sm_info cf levels %d",
                num_conf_levels, st->GetSoundModelInfo()->GetConfLevelsSize());
        status = -EINVAL;
        goto exit;
    }
    /*
     * Cache it to use when stream restarts without config update or
     * during only one remaining stream model as there won't be a
     * merged model yet.
     */
    if (module_type_ != ST_MODULE_TYPE_PDK5) {
        st->GetSoundModelInfo()->UpdateConfLevelArray(conf_levels,
            num_conf_levels);

        status = UpdateMergeConfLevelsPayload(st->GetSoundModelInfo(), true);
        if (status) {
            PAL_ERR(LOG_TAG, "Update merge conf levels failed %d", status);
            goto exit;
        }
    }

    if (module_type_ == ST_MODULE_TYPE_PDK5){
        sva5_wakeup_config_.mode = config->phrases[0].recognition_modes;
        sva5_wakeup_config_.num_keywords = num_conf_levels;
        sva5_wakeup_config_.model_id = st->GetModelId();
        sva5_wakeup_config_.custom_payload_size = sizeof(uint32_t) * 2 +
                sva5_wakeup_config_.num_keywords * sizeof(uint32_t);
        PAL_DBG(LOG_TAG,
        "sva5_wakeup_config_ mode : %u, custome_payload_size : %u, num_keywords : %u, model_id : %u",
        sva5_wakeup_config_.mode,
        sva5_wakeup_config_.custom_payload_size,
        sva5_wakeup_config_.num_keywords, sva5_wakeup_config_.model_id);
        for (int i = 0; i < sva5_wakeup_config_.num_keywords; ++i){
            sva5_wakeup_config_.confidence_levels[i] = conf_levels[i];
            PAL_DBG(LOG_TAG, "%dth keyword confidence level : %u", i,
                    sva5_wakeup_config_.confidence_levels[i]);
        }
    } else if (!CheckIfOtherStreamsAttached(s)) {
        wakeup_config_.mode = config->phrases[0].recognition_modes;
        wakeup_config_.custom_payload_size = config->data_size;
        wakeup_config_.num_active_models = num_conf_levels;
        wakeup_config_.reserved = 0;
        for (int i = 0; i < wakeup_config_.num_active_models; i++) {
            wakeup_config_.confidence_levels[i] = conf_levels[i];
            wakeup_config_.keyword_user_enables[i] =
                (wakeup_config_.confidence_levels[i] == 100) ? 0 : 1;
            PAL_DBG(LOG_TAG, "cf levels[%d] = %d", i,
                    wakeup_config_.confidence_levels[i]);
        }
    } else {
        /* Update recognition mode considering all streams */
        if (wakeup_config_.mode != config->phrases[0].recognition_modes)
            wakeup_config_.mode |= config->phrases[0].recognition_modes;
            wakeup_config_.custom_payload_size = config->data_size;
            wakeup_config_.num_active_models = eng_sm_info_->GetConfLevelsSize();
            wakeup_config_.reserved = 0;
            for (int i = 0; i < wakeup_config_.num_active_models; i++) {
            wakeup_config_.confidence_levels[i] = eng_sm_info_->
                                                        GetConfLevels()[i];
            wakeup_config_.keyword_user_enables[i] =
                (wakeup_config_.confidence_levels[i] == 100) ? 0 : 1;
            PAL_DBG(LOG_TAG, "cf levels[%d] = %d", i,
                    wakeup_config_.confidence_levels[i]);
            }
        }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

void SoundTriggerEngineGsl::GetUpdatedBufConfig(uint32_t *hist_buffer_duration,
                                                uint32_t *pre_roll_duration){

    if (module_type_ == ST_MODULE_TYPE_PDK5){
        *hist_buffer_duration = sva5_buffer_config_.hist_buffer_duration_in_ms;
        *pre_roll_duration = sva5_buffer_config_.pre_roll_duration_in_ms;
    } else {
        *hist_buffer_duration = buffer_config_.hist_buffer_duration_in_ms;
        *pre_roll_duration = buffer_config_.pre_roll_duration_in_ms;
    }
}

int32_t SoundTriggerEngineGsl::UpdateBufConfig(Stream *s,
    uint32_t hist_buffer_duration,
    uint32_t pre_roll_duration) {

    int32_t status = 0;
    StreamSoundTrigger *st = dynamic_cast<StreamSoundTrigger *>(s);
    if (module_type_ == ST_MODULE_TYPE_PDK5){
        if (!CheckIfOtherStreamsAttached(s)){
            sva5_buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
            sva5_buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;
        } else {
            if (hist_buffer_duration > sva5_buffer_config_.
                                       hist_buffer_duration_in_ms)
               sva5_buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
            if (pre_roll_duration > sva5_buffer_config_.pre_roll_duration_in_ms)
                sva5_buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;
        }
        sva5_buffer_config_.model_id = st->GetModelId();
        PAL_DBG(LOG_TAG, "Updated hist buf : %d msec, pre roll : %d msec",
                sva5_buffer_config_.hist_buffer_duration_in_ms,
                sva5_buffer_config_.pre_roll_duration_in_ms);
    } else {
        if (!CheckIfOtherStreamsAttached(s)) {
            buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
            buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;
        } else {
            if (hist_buffer_duration > buffer_config_.hist_buffer_duration_in_ms)
                buffer_config_.hist_buffer_duration_in_ms = hist_buffer_duration;
            if (pre_roll_duration > buffer_config_.pre_roll_duration_in_ms)
                buffer_config_.pre_roll_duration_in_ms = pre_roll_duration;
        }
        PAL_DBG(LOG_TAG, "updated hist buf:%d msec, pre roll:%d msec",
           buffer_config_.hist_buffer_duration_in_ms,
           buffer_config_.pre_roll_duration_in_ms);
   }
   return status;
}

int32_t SoundTriggerEngineGsl::UpdateEngineConfigOnStop(Stream *s) {

    int32_t status = 0;
    StreamSoundTrigger *st = nullptr;
    bool is_any_stream_active = false, enable_lab = false;
    uint32_t hb_duration = 0, pr_duration = 0;

    /* If there is only single stream, do nothing */
    if (!CheckIfOtherStreamsAttached(s))
        return 0;

    /*
     * Adjust history buffer and preroll durations to highest of
     * remaining streams.
     */
    for (uint32_t i = 0; i < eng_streams_.size(); i++) {
        st = dynamic_cast<StreamSoundTrigger *>(eng_streams_[i]);
        if (s != eng_streams_[i] && st && st->GetCurrentStateId() == ST_STATE_ACTIVE) {
            is_any_stream_active = true;
            if (hb_duration < st->GetHistBufDuration())
                hb_duration = st->GetHistBufDuration();
            if (pr_duration < st->GetPreRollDuration())
                pr_duration = st->GetPreRollDuration();
            if (!enable_lab)
                enable_lab = st->IsCaptureRequested();
        }
    }

    if (!is_any_stream_active) {
        PAL_DBG(LOG_TAG, "No stream is active, reset engine config");
        buffer_config_.hist_buffer_duration_in_ms = 0;
        buffer_config_.pre_roll_duration_in_ms = 0;
        capture_requested_ = false;
        return 0;
    }

    buffer_config_.hist_buffer_duration_in_ms = hb_duration;
    buffer_config_.pre_roll_duration_in_ms = pr_duration;
    capture_requested_ = enable_lab;

    /* Update the merged conf levels considering this stream stop */
    StreamSoundTrigger *stopped_st = dynamic_cast<StreamSoundTrigger *>(s);
    status = UpdateMergeConfLevelsPayload(stopped_st->GetSoundModelInfo(), false);
    for (int i = 0; i < eng_sm_info_->GetConfLevelsSize(); i++) {
        wakeup_config_.confidence_levels[i] = eng_sm_info_->GetConfLevels()[i];
        wakeup_config_.keyword_user_enables[i] =
            (wakeup_config_.confidence_levels[i] == 100) ? 0 : 1;
        PAL_DBG(LOG_TAG, "cf levels[%d] = %d", i, wakeup_config_.confidence_levels[i]);
    }

    return status;
}

int32_t SoundTriggerEngineGsl::UpdateEngineConfigOnRestart(Stream *s) {

    int32_t status = 0;
    StreamSoundTrigger *st = nullptr;
    uint32_t hb_duration = 0, pr_duration = 0;
    bool enable_lab = false;

    /*
     * Adjust history buffer and preroll durations to highest of
     * all streams, including current restarting stream.
     */
    for (uint32_t i = 0; i < eng_streams_.size(); i++) {
        st = dynamic_cast<StreamSoundTrigger *>(eng_streams_[i]);
        if (s == eng_streams_[i] || st->GetCurrentStateId() == ST_STATE_ACTIVE) {
            if (hb_duration < st->GetHistBufDuration())
                hb_duration = st->GetHistBufDuration();
            if (pr_duration < st->GetPreRollDuration())
                pr_duration = st->GetPreRollDuration();
            if (!enable_lab)
                enable_lab = st->IsCaptureRequested();
        }
    }

    buffer_config_.hist_buffer_duration_in_ms = hb_duration;
    buffer_config_.pre_roll_duration_in_ms = pr_duration;
    capture_requested_ = enable_lab;

    /* Update the merged conf levels considering this stream restarted as well */
    StreamSoundTrigger *restarted_st = dynamic_cast<StreamSoundTrigger *>(s);
    status = UpdateMergeConfLevelsPayload(restarted_st->GetSoundModelInfo(), true);
    for (int i = 0; i < eng_sm_info_->GetConfLevelsSize(); i++) {
        wakeup_config_.confidence_levels[i] = eng_sm_info_->GetConfLevels()[i];
        wakeup_config_.keyword_user_enables[i] =
            (wakeup_config_.confidence_levels[i] == 100) ? 0 : 1;
        PAL_DBG(LOG_TAG, "cf levels[%d] = %d", i, wakeup_config_.confidence_levels[i]);
    }

    return status;
}

void SoundTriggerEngineGsl::HandleSessionEvent(uint32_t event_id __unused,
                                               void *data, uint32_t size) {
    int32_t status = 0;

    std::unique_lock<std::mutex> lck(mutex_);
    if (is_qcva_uuid_ || is_qcmd_uuid_) {
        status = ParseDetectionPayload(data);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to parse detection payload, status %d",
                    status);
            return;
        }
    } else {
        // store custom detection event for further use
        custom_detection_event_size = size;
        custom_detection_event = (uint8_t *)calloc(1, size);
        if (!custom_detection_event) {
            PAL_ERR(LOG_TAG, "Failed to allocate custom detection event");
            return;
        }
        ar_mem_cpy(custom_detection_event, size, data, size);
    }
    PAL_INFO(LOG_TAG, "singal event processing thread");
    ATRACE_BEGIN("stEngine: keyword detected");
    ATRACE_END();
    cv_.notify_one();
}

void SoundTriggerEngineGsl::HandleSessionCallBack(void *hdl, uint32_t event_id,
                                                  void *data, uint32_t event_size) {
    SoundTriggerEngineGsl *engine = nullptr;

    PAL_DBG(LOG_TAG, "Enter, event detected on SPF, event id = 0x%x", event_id);
    if (!hdl || !data) {
        PAL_ERR(LOG_TAG, "Invalid engine handle or event data");
        return;
    }

    // Possible that AGM_EVENT_EOS_RENDERED could be sent during spf stop.
    // Check and handle only required detection event.
    if (event_id != EVENT_ID_DETECTION_ENGINE_GENERIC_INFO)
        return;

    engine = (SoundTriggerEngineGsl *)hdl;
    engine->HandleSessionEvent(event_id, data, event_size);

    PAL_DBG(LOG_TAG, "Exit");
    return;
}

int32_t SoundTriggerEngineGsl::GetParameters(uint32_t param_id,
                                             void **payload) {
    int32_t status = 0;
    size_t size = 0;
    uint32_t miid = 0;

    PAL_DBG(LOG_TAG, "Enter");
    switch (param_id) {
        case PAL_PARAM_ID_DIRECTION_OF_ARRIVAL:
            status = session_->getParameters(stream_handle_, TAG_ECNS,
                                            param_id, payload);
            break;
        case PAL_PARAM_ID_WAKEUP_MODULE_VERSION:
            status = session_->openGraph(stream_handle_);
            if (status != 0) {
                PAL_ERR(LOG_TAG, "Failed to open graph, status = %d", status);
                return status;
            }
            status = session_->getMIID(nullptr,
                module_tag_ids_[MODULE_VERSION], &miid);
            if (status != 0) {
                PAL_ERR(LOG_TAG, "Failed to get instance id for tag %x, status = %d",
                    module_tag_ids_[MODULE_VERSION], status);
                return status;
            }
            // TODO: update query size here
            builder_->payloadQuery((uint8_t **)payload, &size, miid,
                param_ids_[MODULE_VERSION],
                sizeof(struct version_arch_payload));
            status = session_->getParameters(stream_handle_,
                module_tag_ids_[MODULE_VERSION], param_id, payload);
            status = session_->close(stream_handle_);
            if (status != 0) {
                PAL_ERR(LOG_TAG, "Failed to close session, status = %d", status);
                return status;
            }
        default:
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "Unsupported param id %u status %d",
                    param_id, status);
            goto exit;
    }

    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get parameters, param id %d, status %d",
                param_id, status);
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineGsl::ConnectSessionDevice(
    Stream* stream_handle,
    pal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_connect) {

    int32_t status = 0;

    if (dev_disconnect_count_ == 0)
        status = session_->connectSessionDevice(stream_handle, stream_type,
                                            device_to_connect);
    if (status != 0)
        dev_disconnect_count_++;

    PAL_DBG(LOG_TAG, "dev_disconnect_count_: %d", dev_disconnect_count_);
    return status;
}

int32_t SoundTriggerEngineGsl::DisconnectSessionDevice(
    Stream* stream_handle,
    pal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_disconnect) {

    int32_t status = 0;

    dev_disconnect_count_++;
    if (dev_disconnect_count_ == eng_streams_.size())
        status = session_->disconnectSessionDevice(stream_handle, stream_type,
                                               device_to_disconnect);
    if (status != 0)
        dev_disconnect_count_--;
    PAL_DBG(LOG_TAG, "dev_disconnect_count_: %d", dev_disconnect_count_);
    return status;
}

int32_t SoundTriggerEngineGsl::SetupSessionDevice(
    Stream* stream_handle,
    pal_stream_type_t stream_type,
    std::shared_ptr<Device> device_to_disconnect) {

    int32_t status = 0;

    dev_disconnect_count_--;
    if (dev_disconnect_count_ < 0)
        dev_disconnect_count_ = 0;

    if (dev_disconnect_count_ == 0)
        status = session_->setupSessionDevice(stream_handle, stream_type,
                                          device_to_disconnect);
    if (status != 0)
        dev_disconnect_count_++;

    PAL_DBG(LOG_TAG, "dev_disconnect_count_: %d", dev_disconnect_count_);
    return status;
}

void SoundTriggerEngineGsl::SetCaptureRequested(bool is_requested) {
    capture_requested_ |= is_requested;
    PAL_DBG(LOG_TAG, "capture requested %d, set to engine %d",
        is_requested, capture_requested_);
}

void* SoundTriggerEngineGsl::GetDetectionEventInfo() {
    if (module_type_ == ST_MODULE_TYPE_PDK5) {
       return &detection_event_info_multi_model_;
    }
    return &detection_event_info_;
}

int32_t SoundTriggerEngineGsl::setECRef(Stream *s, std::shared_ptr<Device> dev, bool is_enable) {
    if (!session_) {
        PAL_ERR(LOG_TAG, "Invalid session");
        return -EINVAL;
    }

    return session_->setECRef(s, dev, is_enable);
}

int32_t SoundTriggerEngineGsl::GetCustomDetectionEvent(uint8_t **event,
    size_t *size) {

    *event = custom_detection_event;
    *size = custom_detection_event_size;
    return 0;
}

int32_t SoundTriggerEngineGsl::UpdateSessionPayload(st_param_id_type_t param) {
    int32_t status = 0;
    uint32_t miid = 0;
    uint32_t tag_id = 0;
    uint32_t param_id = 0;
    uint8_t *payload = nullptr;
    size_t payload_size = 0;
    uint32_t ses_param_id = 0;

    PAL_DBG(LOG_TAG, "Enter, param : %u", param);

    if (param < LOAD_SOUND_MODEL || param >= MAX_PARAM_IDS) {
        PAL_ERR(LOG_TAG, "Invalid param id %u", param);
        return -EINVAL;
    }

    tag_id = module_tag_ids_[param];
    param_id = param_ids_[param];
    if (!tag_id || !param_id) {
        PAL_ERR(LOG_TAG, "Invalid tag/param id %u", param);
        return -EINVAL;
    }

    status = session_->getMIID(nullptr, tag_id, &miid);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to get instance id for tag %x, status = %d",
            tag_id, status);
        return status;
    }

    switch(param) {
        case LOAD_MULTI_SOUND_MODEL :
        {
            ses_param_id = PAL_PARAM_ID_LOAD_MULTI_SOUND_MODEL;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                     sm_data_, sm_data_size_, miid, param_id);
            break;
        }
        case DEREGISTER_MULTI_SOUND_MODEL :
        {
            ses_param_id = PAL_PARAM_ID_UNLOAD_MULTI_SOUND_MODEL;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                    (uint8_t *) &deregister_config_, sizeof(
                     struct param_id_detection_engine_deregister_multi_sound_model_t),
                     miid, param_id);
            break;
        }
        case WAKEUP_CONFIG_STAGE1_SVA5 :
        {
            ses_param_id = PAL_PARAM_ID_WAKEUP_MULTI_MODEL_ENGINE_CONFIG;
            size_t fixedConfigVoiceWakeupSize = sizeof(
                                 struct detection_engine_config_stage1_sva5)
                                 - (MAX_KEYWORD_SUPPORTED * sizeof(uint32_t));
            size_t payloadSize = fixedConfigVoiceWakeupSize +
                                       (sva5_wakeup_config_.num_keywords * sizeof(uint32_t));
            uint8_t *wakeup_payload = new uint8_t[payloadSize];
            if (!wakeup_payload){
                PAL_ERR(LOG_TAG, "payload malloc failed %s", strerror(errno));
                return -EINVAL;
            }
            ar_mem_cpy(wakeup_payload, fixedConfigVoiceWakeupSize,
                       &sva5_wakeup_config_, fixedConfigVoiceWakeupSize);
            uint32_t *confidence_level = (uint32_t *)(wakeup_payload + fixedConfigVoiceWakeupSize);

            for (int i = 0; i < sva5_wakeup_config_.num_keywords; ++i){
                confidence_level[i] = sva5_wakeup_config_.confidence_levels[i];

            }
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                      (uint8_t *)wakeup_payload, payloadSize, miid, param_id);
            break;
        }
        case MULTI_MODEL_BUFFERING_CONFIG :
        {
            ses_param_id = PAL_PARAM_ID_WAKEUP_MULTI_MODEL_BUFFERING_CONFIG;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                     (uint8_t *)&sva5_buffer_config_,
                     sizeof(struct detection_engine_multi_model_buffering_config),
                     miid, param_id);
            break;
        }
        case LOAD_SOUND_MODEL:
        {
            ses_param_id = PAL_PARAM_ID_LOAD_SOUND_MODEL;
            /* Set payload data and size from engine's sound model info */
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                eng_sm_info_->GetModelData(), eng_sm_info_->GetModelSize(),
                miid, param_id);
            break;
        }
        case UNLOAD_SOUND_MODEL:
            // TODO: add check when SVA5 integrated
            ses_param_id = PAL_PARAM_ID_UNLOAD_SOUND_MODEL;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                nullptr, 0, miid, param_id);
            break;
        case WAKEUP_CONFIG:
        {
            ses_param_id = PAL_PARAM_ID_WAKEUP_ENGINE_CONFIG;
            size_t fixed_wakeup_payload_size =
                sizeof(struct detection_engine_config_voice_wakeup) -
                PAL_SOUND_TRIGGER_MAX_USERS * 2;
            size_t wakeup_payload_size = fixed_wakeup_payload_size +
                wakeup_config_.num_active_models * 2;
            uint8_t *wakeup_payload = new uint8_t[wakeup_payload_size];
            ar_mem_cpy(wakeup_payload, fixed_wakeup_payload_size,
                &wakeup_config_, fixed_wakeup_payload_size);
            uint8_t *confidence_level = wakeup_payload +
                fixed_wakeup_payload_size;
            uint8_t *kw_user_enable = wakeup_payload +
                fixed_wakeup_payload_size +
                wakeup_config_.num_active_models;
            for (int i = 0; i < wakeup_config_.num_active_models; i++) {
                confidence_level[i] = wakeup_config_.confidence_levels[i];
                kw_user_enable[i] = wakeup_config_.keyword_user_enables[i];
                PAL_VERBOSE(LOG_TAG,
                    "confidence_level[%d] = %d KW_User_enable[%d] = %d",
                    i, confidence_level[i], i, kw_user_enable[i]);
            }
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                (uint8_t *)wakeup_payload, wakeup_payload_size,
                miid, param_id);
            delete[] wakeup_payload;
            break;
        }
        case BUFFERING_CONFIG:
            ses_param_id = PAL_PARAM_ID_WAKEUP_BUFFERING_CONFIG;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                (uint8_t *)&buffer_config_,
                sizeof(struct detection_engine_voice_wakeup_buffer_config),
                miid, param_id);
            break;
        case ENGINE_RESET:
            ses_param_id = PAL_PARAM_ID_WAKEUP_ENGINE_RESET;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                nullptr, 0, miid, param_id);
            break;
        case CUSTOM_CONFIG:
            ses_param_id = PAL_PARAM_ID_WAKEUP_CUSTOM_CONFIG;
            status = builder_->payloadSVAConfig(&payload, &payload_size,
                custom_data, custom_data_size, miid, param_id);
            // release local custom data
            if (custom_data) {
                free(custom_data);
                custom_data = nullptr;
                custom_data_size = 0;
            }
            break;
        default:
            PAL_ERR(LOG_TAG, "Invalid param id %u", param);
            return -EINVAL;
    }

    if (status || !payload) {
        PAL_ERR(LOG_TAG, "Failed to construct SVA payload, status = %d",
            status);
        return -ENOMEM;
    }

    status = session_->setParameters(stream_handle_, tag_id, ses_param_id, payload);
    if (status != 0) {
        PAL_ERR(LOG_TAG, "Failed to set payload for param id %x, status = %d",
            ses_param_id, status);
    }

    return status;
}

std::shared_ptr<SoundTriggerEngineGsl> SoundTriggerEngineGsl::GetInstance(
                  Stream *s, uint32_t id, listen_model_indicator_enum type,
                                             st_module_type_t module_type){

     if (eng_.find(module_type) == eng_.end()){
         eng_[module_type] = std::make_shared<SoundTriggerEngineGsl>
                                           (s, id, type, module_type);
     }
     return eng_[module_type];
}

void SoundTriggerEngineGsl::ResetEngineInstance(Stream *s) {

    if (s) {
        auto iter = std::find(eng_streams_.begin(), eng_streams_.end(), s);
        if (iter != eng_streams_.end())
            eng_streams_.erase(iter);
    }
    if (!eng_streams_.size()) {
        PAL_VERBOSE(LOG_TAG, "reset the engine instance to be freed");
        eng_.erase(this->module_type_);
    }
}
