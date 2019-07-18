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

#define LOG_TAG "SessionAlsaCompress"

#include "SessionAlsaCompress.h"
#include "SessionAlsaUtils.h"
#include "Stream.h"
#include "ResourceManager.h"
#include <agm/agm_api.h>
#include <mutex>


void SessionAlsaCompress::getSndCodecParam(struct snd_codec &codec, struct qal_stream_attributes &sAttr)
{
    struct qal_media_config *config = &sAttr.out_media_config;

    codec.id = getSndCodecId(config->aud_fmt_id);
    codec.ch_in = config->ch_info->channels;
    codec.ch_out = codec.ch_in;
    codec.sample_rate = config->sample_rate;
    codec.bit_rate = config->bit_width;
}

int SessionAlsaCompress::getSndCodecId(qal_audio_fmt_t fmt)
{
    int id = -1;

    switch (fmt) {
        case QAL_AUDIO_FMT_MP3:
            id = SND_AUDIOCODEC_MP3;
            break;
        case QAL_AUDIO_FMT_DEFAULT_PCM:
            id = SND_AUDIOCODEC_PCM;
            break;
    }

    return id;
}

void SessionAlsaCompress::offloadThreadLoop(SessionAlsaCompress* compressObj)
{
    std::shared_ptr<offload_msg> msg;
    uint32_t event_id;

    std::unique_lock<std::mutex> lock(compressObj->cv_mutex_);

    while (1) {
        if (compressObj->msg_queue_.empty())
            compressObj->cv_.wait(lock);  /* wait for incoming requests */

        if (!compressObj->msg_queue_.empty()) {
            msg = compressObj->msg_queue_.front();
            compressObj->msg_queue_.pop();
            lock.unlock();

            if (msg->cmd == OFFLOAD_CMD_EXIT)
                break; // exit the thread

            if (msg->cmd == OFFLOAD_CMD_WAIT_FOR_BUFFER) {
                QAL_VERBOSE(LOG_TAG, "calling compress_wait");
                compress_wait(compressObj->compress, -1);
                QAL_VERBOSE(LOG_TAG, "out of compress_wait");
                event_id = QAL_STREAM_CBK_EVENT_WRITE_READY;
            }
            if (compressObj->sessionCb)
                compressObj->sessionCb(compressObj->cbCookie, event_id, NULL);
        }

    }
}

SessionAlsaCompress::SessionAlsaCompress(std::shared_ptr<ResourceManager> Rm)
{
    rm = Rm;
    builder = new PayloadBuilder();

    /** set default snd codec params */
    codec.id = getSndCodecId(QAL_AUDIO_FMT_DEFAULT_PCM);
    codec.ch_in = 2;
    codec.ch_out = codec.ch_in;
    codec.sample_rate = 48000;
    codec.bit_rate = 16;

    compress = NULL;
    sessionCb = NULL;
    this->cbCookie = NULL;
    playback_started = false;
}

SessionAlsaCompress::~SessionAlsaCompress()
{
    delete builder;
}

int SessionAlsaCompress::open(Stream * s)
{
    int status = -EINVAL;
    struct qal_stream_attributes sAttr;
    std::vector<std::shared_ptr<Device>> associatedDevices;

    status = s->getStreamAttributes(&sAttr);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getStreamAttributes Failed \n", __func__);
        return status;
    }

    status = s->getAssociatedDevices(associatedDevices);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: getAssociatedDevices Failed \n", __func__);
        return status;
    }

    compressDevIds = rm->allocateFrontEndIds(sAttr.type, sAttr.direction);
    aifBackEnds = rm->getBackEndNames(associatedDevices);

    status = SessionAlsaUtils::open(s, rm, compressDevIds, aifBackEnds);
    if (status) {
        QAL_ERR(LOG_TAG, "session alsa open failed with %d", status);
        rm->freeFrontEndIds(compressDevIds);
    }
    return status;
}

int SessionAlsaCompress::prepare(Stream * s)
{
   return 0;
}

int SessionAlsaCompress::setConfig(Stream * s, configType type, int tag)
{
   return 0;
}
/*
int SessionAlsaCompress::getConfig(Stream * s)
{
   return 0;
}
*/
int SessionAlsaCompress::start(Stream * s)
{
    struct compr_config compress_config;
    struct qal_stream_attributes sAttr;
    int32_t status = 0;
    size_t in_buf_size, in_buf_count, out_buf_size, out_buf_count;

    /** create an offload thread for posting callbacks */
    worker_thread = std::make_unique<std::thread>(offloadThreadLoop, this);

    s->getStreamAttributes(&sAttr);
    getSndCodecParam(codec, sAttr);
    s->getBufInfo(&in_buf_size,&in_buf_count,&out_buf_size,&out_buf_count);
    compress_config.fragment_size = out_buf_size;
    compress_config.fragments = out_buf_count;
    compress_config.codec = &codec;
    // compress_open
    compress = compress_open(rm->getSndCard(), compressDevIds.at(0), COMPRESS_IN, &compress_config);
    if (!compress) {
        QAL_ERR(LOG_TAG, "compress open failed");
        goto free_feIds;
    }
    if (!is_compress_ready(compress)) {
        QAL_ERR(LOG_TAG, "compress open not ready");
        goto free_feIds;
    }
    /** set non blocking mode for writes */
    compress_nonblock(compress, (ioMode == QAL_STREAM_FLAG_NON_BLOCKING));

free_feIds:
   return 0;
}

int SessionAlsaCompress::stop(Stream * s)
{
    int32_t status = 0;

    if (compress)
        status = compress_stop(compress);

   return status;
}

int SessionAlsaCompress::close(Stream * s)
{
    int status = 0;
    if (!compress)
        return -EINVAL;

    /** Disconnect FE to BE */
    mixer_ctl_set_enum_by_string(disconnectCtrl, (aifBackEnds.at(0)).data());
    compress_close(compress);

    std::shared_ptr<offload_msg> msg = std::make_shared<offload_msg>(OFFLOAD_CMD_EXIT);
std::lock_guard<std::mutex> lock(cv_mutex_);
    msg_queue_.push(msg);
    cv_.notify_all();

    /* wait for handler to exit */
    worker_thread->join();
    worker_thread.reset(NULL);

    /* empty the pending messages in queue */
    while(!msg_queue_.empty())
        msg_queue_.pop();

   return 0;
}

int SessionAlsaCompress::read(Stream *s, int tag, struct qal_buffer *buf, int * size)
{
    return 0;
}

int SessionAlsaCompress::write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag)
{
    int bytes_written = 0;
    int status;
    bool non_blocking = (ioMode == QAL_STREAM_FLAG_NON_BLOCKING);

    if (!buf || buf->buffer || buf->size)
        return -EINVAL;

    bytes_written = compress_write(compress, buf->buffer, buf->size);

    QAL_VERBOSE(LOG_TAG, "%s: writing buffer (%zu bytes) to compress device returned %d",
            __func__, buf->size, bytes_written);

    if (bytes_written >= 0 && bytes_written < (ssize_t)buf->size && non_blocking) {
        QAL_ERR(LOG_TAG, "No space available in compress driver, post msg to cb thread");
        std::shared_ptr<offload_msg> msg = std::make_shared<offload_msg>(OFFLOAD_CMD_WAIT_FOR_BUFFER);
//        msg->cmd = OFFLOAD_CMD_WAIT_FOR_BUFFER;
        std::lock_guard<std::mutex> lock(cv_mutex_);
        msg_queue_.push(msg);

        cv_.notify_all();
    }

    if (!playback_started && bytes_written > 0) {
        status = compress_start(compress);
        if (status) {
            QAL_ERR(LOG_TAG, "compress start failed with err %d", status);
            return status;
        }
        playback_started = true;
    }

    if (size)
        *size = bytes_written;
    return 0;
}

int SessionAlsaCompress::readBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag)
{
    return 0;
}
int SessionAlsaCompress::writeBufferInit(Stream *s, size_t noOfBuf, size_t bufSize, int flag)
{
    return 0;
}

int SessionAlsaCompress::setParameters(Stream *s, int tagId, uint32_t param_id, void *payload)
{
    return 0;
}

int SessionAlsaCompress::registerCallBack(session_callback cb, void *cookie)
{
    sessionCb = cb;
    cbCookie = cookie;
    return 0;
}

int SessionAlsaCompress::drain(qal_drain_type_t type)
{
    if (compress)
        compress_drain(compress);
    return 0;
}

int SessionAlsaCompress::getParameters(Stream *s, int tagId, uint32_t param_id, void **payload)
{
    return 0;
}

