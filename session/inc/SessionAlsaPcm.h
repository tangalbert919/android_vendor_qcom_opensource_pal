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

#ifndef SESSION_ALSAPCM_H
#define SESSION_ALSAPCM_H

#include "PayloadBuilder.h"
#include "Session.h"
#include "QalAudioRoute.h"
#include <tinyalsa/asoundlib.h>
#include <thread>

#define PARAM_ID_DETECTION_ENGINE_CONFIG_VOICE_WAKEUP 0x08001049
#define PARAM_ID_VOICE_WAKEUP_BUFFERING_CONFIG 0x08001044

class Stream;
class Session;

class SessionAlsaPcm : public Session
{
private:
    void * graphHandle;
    void * customPayload;
    size_t customPayloadSize;
    size_t size = 0;
    PayloadBuilder* builder;
    struct pcm *pcm;
    std::shared_ptr<ResourceManager> rm;
    struct mixer *mixer;
    size_t in_buf_size, in_buf_count, out_buf_size, out_buf_count;
    std::vector<int> pcmDevIds;
    std::vector<std::string> aifBackEnds;
    std::vector <std::pair<int, int>> gkv;
    std::vector <std::pair<int, int>> ckv;
    std::vector <std::pair<int, int>> tkv;
    void *cookie;
    std::thread threadHandler;

public:

    SessionAlsaPcm(std::shared_ptr<ResourceManager> Rm);
    ~SessionAlsaPcm();
    int open(Stream * s) override;
    int prepare(Stream * s) override;
    int setConfig(Stream * s, configType type, int tag = 0) override;
    //int getConfig(Stream * s) override;
    int start(Stream * s) override;
    int stop(Stream * s) override;
    int close(Stream * s) override;
    int readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int read(Stream *s, int tag, struct qal_buffer *buf, int * size) override;
    int write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) override;
    int setParameters(Stream *s, int tagId, uint32_t param_id, void *payload) override;
    int getParameters(Stream *s, int tagId, uint32_t param_id, void **payload) override;
    static void eventWaitThreadLoop(void *context, SessionAlsaPcm *session);
    int handleMixerEvent(struct mixer *mixer, char *mixer_str);
    void checkAndConfigConcurrency(Stream *s);
};

#endif //SESSION_ALSAPCM_H
