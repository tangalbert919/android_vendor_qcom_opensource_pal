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
    uint32_t spr_miid = 0;
    PayloadBuilder* builder;
    struct pcm *pcm;
    struct pcm *pcmRx;
    struct pcm *pcmTx;
    std::shared_ptr<ResourceManager> rm;
    struct mixer *mixer;
    size_t in_buf_size, in_buf_count, out_buf_size, out_buf_count;
    std::vector<int> pcmDevIds;
    std::vector<int> pcmDevRxIds;
    std::vector<int> pcmDevTxIds;
    std::vector<std::pair<int32_t, std::string>> rxAifBackEnds;
    std::vector<std::pair<int32_t, std::string>> txAifBackEnds;
    std::vector <std::pair<int, int>> gkv;
    std::vector <std::pair<int, int>> ckv;
    std::vector <std::pair<int, int>> tkv;
    void *cookie;
    std::thread threadHandler;
    sessionState mState;
    session_callback sessionCb;
    void *cbCookie;
    bool isECRefSet;

public:

    SessionAlsaPcm(std::shared_ptr<ResourceManager> Rm);
    ~SessionAlsaPcm();
    int open(Stream * s) override;
    int prepare(Stream * s) override;
    int setTKV(Stream * s, configType type, effect_qal_payload_t *payload) override;
    int setConfig(Stream * s, configType type, int tag = 0) override;
    int setConfig(Stream * s, configType type, uint32_t tag1,
            uint32_t tag2, uint32_t tag3) override;
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
    int getTimestamp(struct qal_session_time *stime) override;
    int registerCallBack(session_callback cb, void *cookie) override;
    int drain(qal_drain_type_t type) override;
    int flush();
    int connectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect) override;
    int disconnectSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToDisconnect) override;
    bool isActive();
};

#endif //SESSION_ALSAPCM_H
