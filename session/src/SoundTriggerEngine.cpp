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

#define LOG_TAG "SoundTriggerEngine"

#include "SoundTriggerEngine.h"

#include "SoundTriggerEngineGsl.h"
#include "SoundTriggerEngineCapiCnn.h"
#include "SoundTriggerEngineCapiVop.h"
#include "Stream.h"

std::shared_ptr<SoundTriggerEngine> SoundTriggerEngine::Create(
    Stream *s,
    listen_model_indicator_enum type,
    QalRingBufferReader **reader,
    QalRingBuffer *buffer)
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
        st_engine = std::make_shared<SoundTriggerEngineGsl>(s, id, id,
                                                            reader, buffer);
        if (!st_engine)
            QAL_ERR(LOG_TAG, "SoundTriggerEngine GSL creation failed");
        break;

    case ST_SM_ID_SVA_CNN:
        st_engine = std::make_shared<SoundTriggerEngineCapiCnn>(s, id, id,
                                                                reader, buffer);
        if (!st_engine)
            QAL_ERR(LOG_TAG, "SoundTriggerEngine CNN creation failed");
        break;

    case ST_SM_ID_SVA_VOP:
        st_engine = std::make_shared<SoundTriggerEngineCapiVop>(s, id, id,
                                                                reader, buffer);
        if (!st_engine)
            QAL_ERR(LOG_TAG, "SoundTriggerEngine VOP creation failed");
        break;

    default:
        QAL_ERR(LOG_TAG, "Invalid model type: %u", id);
        break;
    }

    QAL_VERBOSE(LOG_TAG, "Exit, engine %p", st_engine.get());

    return st_engine;
}
