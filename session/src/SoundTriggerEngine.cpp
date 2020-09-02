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

#define LOG_TAG "QAL: SoundTriggerEngine"

#include "SoundTriggerEngine.h"

#include "SoundTriggerEngineGsl.h"
#include "SoundTriggerEngineCapi.h"
#include "Stream.h"

std::shared_ptr<SoundTriggerEngine> SoundTriggerEngine::Create(
    Stream *s,
    listen_model_indicator_enum type)
{
    QAL_VERBOSE(LOG_TAG, "Enter, type %d", type);

    if (!s) {
        QAL_ERR(LOG_TAG, "Invalid stream handle");
        return nullptr;
    }

    uint32_t id = static_cast<uint32_t>(type);
    std::shared_ptr<SoundTriggerEngine> st_engine(nullptr);

    switch (type) {
    case ST_SM_ID_SVA_GMM:
        st_engine = std::make_shared<SoundTriggerEngineGsl>(s, id, type);
        if (!st_engine)
            QAL_ERR(LOG_TAG, "SoundTriggerEngine GSL creation failed");
        break;

    case ST_SM_ID_SVA_CNN:
    case ST_SM_ID_SVA_RNN:
    case ST_SM_ID_SVA_VOP:
        st_engine = std::make_shared<SoundTriggerEngineCapi>(s, id, type);
        if (!st_engine)
            QAL_ERR(LOG_TAG, "SoundTriggerEngine capi creation failed");
        break;

    default:
        QAL_ERR(LOG_TAG, "Invalid model type: %u", id);
        break;
    }

    QAL_VERBOSE(LOG_TAG, "Exit, engine %p", st_engine.get());

    return st_engine;
}

int32_t SoundTriggerEngine::CreateBuffer(uint32_t buffer_size,
    uint32_t engine_size, std::vector<QalRingBufferReader *> &reader_list)
{
    int32_t status = 0;
    int32_t i = 0;
    QalRingBufferReader *reader = nullptr;

    if (!buffer_size || !engine_size) {
        QAL_ERR(LOG_TAG, "Invalid buffer size or engine number");
        status = -EINVAL;
        goto exit;
    }

    if (engine_id_ != static_cast<uint32_t>(ST_SM_ID_SVA_GMM)) {
        QAL_ERR(LOG_TAG, "Cannot create buffer in non-GMM engine");
        status = -EINVAL;
        goto exit;
    }

    QAL_DBG(LOG_TAG, "Enter");
    if (buffer_) {
        delete buffer_;
        buffer_ = nullptr;
    }

    buffer_ = new QalRingBuffer(buffer_size);
    if (!buffer_) {
        QAL_ERR(LOG_TAG, "Failed to allocate memory for ring buffer");
        status = -ENOMEM;
        goto exit;
    }

    for (i = 0; i < engine_size; i++) {
        reader = buffer_->newReader();
        if (!reader) {
            QAL_ERR(LOG_TAG, "Failed to create new ring buffer reader");
            status = -ENOMEM;
            goto exit;
        }
        reader_list.push_back(reader);
    }

exit:
    QAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngine::SetBufferReader(QalRingBufferReader *reader)
{
    int32_t status = 0;

    if (engine_id_ == static_cast<uint32_t>(ST_SM_ID_SVA_GMM)) {
        QAL_DBG(LOG_TAG, "No need to set reader for GMM engine");
        return status;
    }

    // release reader first if exists
    if (reader_)
        delete reader_;
    reader_ = reader;

    return status;
}

uint32_t SoundTriggerEngine::UsToBytes(uint64_t input_us) {
    uint32_t bytes = 0;

    bytes = sample_rate_ * bit_width_ * channels_ * input_us /
        (BITS_PER_BYTE * US_PER_SEC);

    return bytes;
}
