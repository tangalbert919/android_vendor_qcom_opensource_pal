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

#ifndef SESSION_ALSAVOICE_H
#define SESSION_ALSAVOICE_H

#include "PayloadBuilder.h"
#include "Session.h"
#include "QalAudioRoute.h"
#include "vcpm_api.h"
#include <tinyalsa/asoundlib.h>
#include <thread>

#define RXDIR 0
#define TXDIR 1

class Stream;
class Session;

class SessionAlsaVoice : public Session
{
private:
    void * customPayload;
    size_t customPayloadSize;
    uint32_t spr_miid = 0;
    PayloadBuilder* builder;
    struct pcm *pcmRx;
    struct pcm *pcmTx;
    size_t in_buf_size, in_buf_count, out_buf_size, out_buf_count;
    std::vector<int> pcmDevRxIds;
    std::vector<int> pcmDevTxIds;
    std::vector <std::pair<int, int>> gkv;
    std::vector <std::pair<int, int>> ckv;
    std::vector <std::pair<int, int>> tkv;
    std::thread threadHandler;
    uint32_t vsid = 0x11C0500; /*defualt*/
    float default_volume = 0.5;
    uint32_t ttyMode = QAL_TTY_OFF;
    bool volume_boost = vol_boost_disable;
    bool slow_talk = false;

public:

    SessionAlsaVoice(std::shared_ptr<ResourceManager> Rm);
    ~SessionAlsaVoice();
    int open(Stream * s) override;
    int prepare(Stream * s) override;
    int setConfig(Stream * s, configType type, int tag = 0) override;
    int setConfig(Stream * s, configType type, int tag = 0, int dir = 0) override;
    int setParameters(Stream *streamHandle, int tagId, uint32_t param_id,
                      void *payload) override;
    int start(Stream * s) override;
    int stop(Stream * s) override;
    int close(Stream * s) override;
    int setupSessionDevice(Stream* streamHandle, qal_stream_type_t streamType,
        std::shared_ptr<Device> deviceToConnect) override;
    int disconnectSessionDevice(Stream *streamHandle,
                                qal_stream_type_t streamType,
                                std::shared_ptr<Device> deviceToDisconnect);
    int connectSessionDevice(Stream* streamHandle,
                             qal_stream_type_t streamType,
                             std::shared_ptr<Device> deviceToConnect);
    int setECRef(Stream *s, std::shared_ptr<Device> rx_dev, bool is_enable) override;
private:
    int payloadCalKeys(Stream * s, uint8_t **payload, size_t *size);
    int payloadTaged(Stream * s, configType type, int tag, int device, int dir);
    int payloadSetVSID(uint8_t **payload, size_t *size);
    int payloadSetTTYMode(uint8_t **payload, size_t *size, uint32_t mode);
    int setVoiceMixerParameter(Stream * s, struct mixer *mixer, void *payload,
                          int size, int dir);
    char* getMixerVoiceStream(Stream *s, int dir);
    uint32_t getMIID(const char *backendName, uint32_t tagId, uint32_t *miid) override;
};

#endif //SESSION_ALSAVOICE_H
