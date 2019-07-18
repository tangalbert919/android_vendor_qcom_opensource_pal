/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#ifndef STREAMCOMPRESS_H_
#define STREAMCOMPRESS_H_

#include "Stream.h"

class ResourceManager;
class Device;
class Session;

class StreamCompress : public Stream
{
public:
    StreamCompress(const struct qal_stream_attributes *sattr, const struct qal_device *dattr, const uint32_t no_of_devices,
                  const struct modifier_kv *modifiers, const uint32_t no_of_modifiers, const std::shared_ptr<ResourceManager> rm);
    ~StreamCompress();
    int32_t open() override;
    int32_t close() override;
    int32_t start() override;
    int32_t stop() override;
    int32_t prepare() override;
    int32_t setStreamAttributes(struct qal_stream_attributes *sattr) override;
    int32_t setVolume( struct qal_volume_data *volume) override;
    int32_t setMute( bool state) override;
    int32_t setPause() override;
    int32_t setResume() override;
    int32_t read(struct qal_buffer *buf) override;
    int32_t write(struct qal_buffer *buf) override;
    int32_t registerCallBack(qal_stream_callback cb) override;
    int32_t getCallBack(qal_stream_callback *cb) override;
    int32_t getParameters(uint32_t param_id, void **payload) override;
    int32_t setParameters(uint32_t param_id, void *payload) override;
};

#endif//STREAMCOMPRESS_H_
