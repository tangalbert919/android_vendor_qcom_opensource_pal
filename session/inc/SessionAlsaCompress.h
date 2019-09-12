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

#ifndef SESSION_ALSACOMPRESS_H
#define SESSION_ALSACOMPRESS_H


#include "Session.h"

#include "ResourceManager.h"
#include "PayloadBuilder.h"
#include <algorithm>
#include <queue>
#include <deque>
#include "QalAudioRoute.h"
#include <tinyalsa/asoundlib.h>
#include <condition_variable>
#include <sound/compress_params.h>
#include <tinycompress/tinycompress.h>

class Stream;
class Session;

enum {
    OFFLOAD_CMD_EXIT,               /* exit compress offload thread loop*/
    OFFLOAD_CMD_DRAIN,              /* send a full drain request to DSP */
    OFFLOAD_CMD_PARTIAL_DRAIN,      /* send a partial drain request to DSP */
    OFFLOAD_CMD_WAIT_FOR_BUFFER,    /* wait for buffer released by DSP */
    OFFLOAD_CMD_ERROR               /* offload playback hit some error */
};

struct offload_msg {
    offload_msg(int c)
        :cmd(c) {}
    int cmd; /**< command */
};

class SessionAlsaCompress : public Session
{
private:

    void * graphHandle;
    struct compress *compress;
    void * payload;
    size_t size = 0;
    uint32_t spr_miid = 0;
    PayloadBuilder* builder;
    struct snd_codec codec;
    struct mixer *mixer;
    //  unsigned int compressDevId;
    std::vector<int> compressDevIds;
    std::vector<std::string> aifBackEnds;
    std::unique_ptr<std::thread> worker_thread;
    std::queue<std::shared_ptr<offload_msg>> msg_queue_;

    std::condition_variable cv_; /* used to wait for incoming requests */
    std::mutex cv_mutex_; /* mutex used in conjunction with above cv */
    struct mixer_ctl *feCtrl;
    struct mixer_ctl *feMdCtrl;
    struct mixer_ctl *aifMdCtrl;
    struct mixer_ctl *connectCtrl;
    struct mixer_ctl *aifMfCtrl;
    struct mixer_ctl *disconnectCtrl;
    void getSndCodecParam(struct snd_codec &codec, struct qal_stream_attributes &sAttr);
    int getSndCodecId(qal_audio_fmt_t fmt);
    bool playback_started;
    int ioMode;
    session_callback sessionCb;
    void *cbCookie;
    qal_audio_fmt_t audio_fmt;
    int fileWrite(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag);
    std::vector <std::pair<int, int>> ckv;
    std::vector <std::pair<int, int>> tkv;

public:
    SessionAlsaCompress(std::shared_ptr<ResourceManager> Rm);
    virtual ~SessionAlsaCompress();
    int open(Stream * s) override;
    int prepare(Stream * s) override;
    int setConfig(Stream * s, configType type, int tag = 0) override;
    //int getConfig(Stream * s) override;
    int start(Stream * s) override;
    int stop(Stream * s) override;
    int close(Stream * s) override;
    int readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag) override;
    int setParameters(Stream *s, int tagId, uint32_t param_id, void *payload);
    int getParameters(Stream *s, int tagId, uint32_t param_id, void **payload);
    int read(Stream *s, int tag, struct qal_buffer *buf, int * size) override;
    int write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) override;
    static void offloadThreadLoop(SessionAlsaCompress *ob);
    int registerCallBack(session_callback cb, void *cookie);
    int drain(qal_drain_type_t type);
    int getTimestamp(struct qal_session_time *stime) override;
};

#endif //SESSION_ALSACOMPRESS_H
