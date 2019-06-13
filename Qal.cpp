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

#define LOG_TAG "qal"

#include <unistd.h>
#include <stdlib.h>
#include <QalApi.h>
#include "Stream.h"
#include "ResourceManager.h"
#include "QalCommon.h"
class Stream;

/*
 * qal_init - Initialize QAL
 *
 * Return 0 on success or error code otherwise
 *
 * Prerequisites
 *    None.
 */
int32_t qal_init(void)
{
    int32_t ret = 0;
    ret = ResourceManager::init();
    return ret;
}

/*
 * qal_deinit - De-initialize QAL
 *
 * Return 0 on success or error code otherwise
 *
 * Prerequisites
 *    QAL must be initialized.
 */
int32_t qal_deinit(void)
{
    int32_t ret = 0;
    ResourceManager::deinit();
    return ret;
}


int32_t qal_stream_open(struct qal_stream_attributes *attributes,
                        uint32_t no_of_devices, struct qal_device *devices,
                        uint32_t no_of_modifiers, struct modifier_kv *modifiers,
                        qal_stream_callback cb, void *cookie,
                        qal_stream_handle_t **stream_handle)
{
    void * stream = NULL;
    Stream *s = NULL;
    int status;
    try{
        s = Stream::create(attributes, devices, no_of_devices, modifiers, no_of_modifiers);
    }
    catch(const std::exception& e)
    {
        QAL_ERR(LOG_TAG,"Stream create failed: %s",e.what());
        return -EINVAL;
    }
    if (s == NULL) {
        QAL_ERR(LOG_TAG,"stream creation failed");
        return -EINVAL;
    }
    status = s->open();
    if (status) {
        QAL_ERR(LOG_TAG,"%s: qal_stream_open failed with status %d", __func__, status);
        return status;
    }
    stream = static_cast<void *>(s);
    QAL_INFO(LOG_TAG,"Stream handle :%p",s);
    QAL_INFO(LOG_TAG,"coverted handle:%p",stream);
    *stream_handle = stream;
    QAL_INFO(LOG_TAG,"value of stream_handle %p",*stream_handle);
    return status;
}

int32_t qal_stream_close(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"%s:%d:Stream handle :%p",__func__, __LINE__, stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s = static_cast<Stream *>(stream_handle);
        status = s->close();
        //free(stream_handle);
    }
    QAL_INFO(LOG_TAG,"%s:%d status %d", __func__, __LINE__, status);
    return status;
}

int32_t qal_stream_start(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->start();
    }
    return status;
}

int32_t qal_stream_stop(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->stop();
    }
    return status;
}

ssize_t qal_stream_write(qal_stream_handle_t *stream_handle, struct qal_buffer *buf)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"%s:Stream handle :%p", __func__, stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        QAL_VERBOSE(LOG_TAG,"%s:Stream handle :%p calling write", __func__, stream_handle);
        s =  static_cast<Stream *>(stream_handle);
        status = s->write(buf);
    }
    QAL_INFO(LOG_TAG,"%s: return status %d",__func__, status);
    return status;
}

ssize_t qal_stream_read(qal_stream_handle_t *stream_handle, struct qal_buffer *buf)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->read(buf);
    }
    return status;
}

int32_t qal_stream_set_param(qal_stream_handle_t *stream_handle, uint32_t param_id, qal_param_payload *param_payload)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG, "Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setParameters(param_id, (void *)param_payload);
    }
    return status;
}

int32_t qal_stream_set_volume(qal_stream_handle_t *stream_handle, struct qal_volume_data *volume)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setVolume(volume);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: setVolume failed with status %d", __func__, status);
        }
    }
    return status;
}

int32_t qal_stream_set_mute(qal_stream_handle_t *stream_handle, bool state)
{
    Stream *s = NULL;
    //return 0;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setMute(state);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: setMute failed with status %d", __func__, status);
        }
    }
    return status;
}

int32_t qal_stream_pause(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setPause();
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: qal_stream_pause failed with status %d", __func__, status);
        }
    }
    return status;
}

int32_t qal_stream_resume(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    QAL_INFO(LOG_TAG,"Stream handle :%p",stream_handle);
    int status;
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setResume();
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: qal_stream_resume failed with status %d", __func__, status);
        }
    }
    return status;
}
int32_t qal_stream_set_buffer_size (qal_stream_handle_t *stream_handle, size_t *in_buf_size,
                                    const size_t in_buf_count,size_t *out_buf_size,
                                    const size_t out_buf_count)
{
   Stream *s = NULL;
   int status;
   QAL_INFO(LOG_TAG,"%s : Stream handle :%p ",__func__,stream_handle);
    if (!stream_handle)
    {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"%s: Invalid stream handle", __func__);
    }
    else
    {
        s =  static_cast<Stream *>(stream_handle);
        status = s->setBufInfo(in_buf_size,in_buf_count,out_buf_size,out_buf_count);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: qal_stream_set_buffer_size failed with status %d", __func__, status);
        }
    }
    return status;
}

