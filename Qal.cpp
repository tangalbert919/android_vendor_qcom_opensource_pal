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
 * enable_gcov - Enable gcov for qal
 *
 * Prerequisites
 *   Should be call from CATF
 */

/*void enable_gcov()
{
    __gcov_flush();
}*/

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
    QAL_INFO(LOG_TAG, "Enter.");
    int32_t ret = 0;
    std::shared_ptr<ResourceManager> ri = NULL;
    try {
        ri = ResourceManager::getInstance();
    } catch (const std::exception& e) {
        QAL_ERR(LOG_TAG, "qal init failed: %s", e.what());
        return -EINVAL;
    }
    QAL_INFO(LOG_TAG, "Exit. ret : %d ", ret);
    return ret;
}

/*
 * qal_deinit - De-initialize QAL
 *
 * Prerequisites
 *    QAL must be initialized.
 */
void qal_deinit(void)
{
    QAL_INFO(LOG_TAG, "Enter.");
    ResourceManager::deinit();
    QAL_INFO(LOG_TAG, "Exit.");
    return;
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
    if (!attributes || !devices) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid input parameters status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter.");
    try {
        s = Stream::create(attributes, devices, no_of_devices, modifiers,
                           no_of_modifiers);
    } catch (const std::exception& e) {
        QAL_ERR(LOG_TAG, "Stream create failed: %s", e.what());
        return -EINVAL;
    }
    if (!s) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "stream creation failed status %d", status);
        return status;
    }
    status = s->open();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "qal_stream_open failed with status %d", status);
        delete s;
        return status;
    }
    if (cb)
       s->registerCallBack(cb, cookie);
    stream = static_cast<void *>(s);
    *stream_handle = stream;
    QAL_DBG(LOG_TAG, "Exit. Value of stream_handle %pK, status %d", stream, status);
    return status;
}

int32_t qal_stream_close(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s = static_cast<Stream *>(stream_handle);
    status = s->close();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "stream closed failed. status %d", status);
        return status;
    }

    delete s;
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_start(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_DBG(LOG_TAG, "Enter. Stream handle %pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->start();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "stream start failed. status %d", status);
        return status;
    }
    QAL_DBG(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_stop(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->stop();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "stream stop failed. status : %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

ssize_t qal_stream_write(qal_stream_handle_t *stream_handle, struct qal_buffer *buf)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle || !buf) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid input parameters status %d", status);
        return status;
    }
    QAL_VERBOSE(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->write(buf);
    if (status < 0) {
        QAL_ERR(LOG_TAG, "stream write failed status %d", status);
        return status;
    }
    QAL_VERBOSE(LOG_TAG, "Exit. status %d", status);
    return status;
}

ssize_t qal_stream_read(qal_stream_handle_t *stream_handle, struct qal_buffer *buf)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle || !buf) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid input parameters status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->read(buf);
    if (status < 0) {
        QAL_ERR(LOG_TAG, "stream read failed status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_get_param(qal_stream_handle_t *stream_handle,
                             uint32_t param_id, qal_param_payload **param_payload)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,  "Invalid input parameters status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->getParameters(param_id, (void **)param_payload);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "get parameters failed status %d param_id %u", status, param_id);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_set_param(qal_stream_handle_t *stream_handle, uint32_t param_id,
                             qal_param_payload *param_payload)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,  "Invalid stream handle, status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->setParameters(param_id, (void *)param_payload);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "set parameters failed status %d param_id %u", status, param_id);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_set_volume(qal_stream_handle_t *stream_handle,
                              struct qal_volume_data *volume)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle || !volume) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"Invalid input parameters status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->setVolume(volume);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "setVolume failed with status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_set_mute(qal_stream_handle_t *stream_handle, bool state)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->setMute(state);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "setMute failed with status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_pause(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->setPause();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "qal_stream_pause failed with status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_resume(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;

    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);

    status = s->setResume();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "resume failed with status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_drain(qal_stream_handle_t *stream_handle, qal_drain_type_t type)
{
    Stream *s = NULL;
    int status;

    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);

    status = s->drain(type);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "drain failed with status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_flush(qal_stream_handle_t *stream_handle)
{
    Stream *s = NULL;
    int status;

    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);

    status = s->flush();
    if (0 != status) {
        QAL_ERR(LOG_TAG, "flush failed with status %d", status);
        return status;
    }

    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_stream_set_buffer_size (qal_stream_handle_t *stream_handle,
                                    size_t *in_buf_size, const size_t in_buf_count,
                                    size_t *out_buf_size, const size_t out_buf_count)
{
   Stream *s = NULL;
   int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid input parameters status %d", status);
        return status;
    }
    QAL_DBG(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->setBufInfo(in_buf_size,in_buf_count,out_buf_size,out_buf_count);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "qal_stream_set_buffer_size failed with status %d", status);
        return status;
    }
    QAL_DBG(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_get_timestamp(qal_stream_handle_t *stream_handle,
                          struct qal_session_time *stime)
{
    Stream *s = NULL;
    int status;
    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid input parameters status %d\n", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK\n", stream_handle);
    s =  static_cast<Stream *>(stream_handle);
    status = s->getTimestamp(stime);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "qal_get_timestamp failed with status %d\n", status);
        return status;
    }
    QAL_VERBOSE(LOG_TAG, "stime->session_time.value_lsw = %u, stime->session_time.value_msw = %u \n", stime->session_time.value_lsw, stime->session_time.value_msw);
    QAL_VERBOSE(LOG_TAG, "stime->absolute_time.value_lsw = %u, stime->absolute_time.value_msw = %u \n", stime->absolute_time.value_lsw, stime->absolute_time.value_msw);
    QAL_VERBOSE(LOG_TAG, "stime->timestamp.value_lsw = %u, stime->timestamp.value_msw = %u \n", stime->timestamp.value_lsw, stime->timestamp.value_msw);
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;
}

int32_t qal_add_remove_effect(qal_stream_handle_t *stream_handle,
                       qal_audio_effect_t effect, bool enable)
{
    Stream *s = NULL;
    int status = EINVAL;
    qal_stream_type_t type;

    if (!stream_handle) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "Invalid stream handle status %d", status);
        return status;
    }
    QAL_INFO(LOG_TAG, "Enter. Stream handle :%pK", stream_handle);

    status = s->getStreamType(&type);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "getStreamType failed with status = %d", status);
        return status;
    }
    if (QAL_STREAM_VOIP_TX == type) {
        s =  static_cast<Stream *>(stream_handle);
        status = s->addRemoveEffect(effect, enable);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "qal_add_effect failed with status %d", status);
            return status;
        }
    }
    QAL_INFO(LOG_TAG, "Exit. status %d", status);
    return status;

}
int32_t qal_stream_set_device(qal_stream_handle_t *stream_handle,
                           uint32_t no_of_devices, struct qal_device *devices)
{
    int status = -EINVAL;
    Stream *s = NULL;
    std::shared_ptr<ResourceManager> rm = NULL;
    struct qal_stream_attributes sattr;

    if (!stream_handle ) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid stream handle status %d", __func__, status);
        return status;
    }

    if (!devices) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid device status %d", __func__, status);
        return status;
    }

    rm = ResourceManager::getInstance();
    if (!rm) {
        QAL_ERR(LOG_TAG, "%s: Invalid resource manager", __func__);
        return status;
    }

    /* Choose best device config for this stream */
    /* TODO: Decide whether to update device config or not based on flag */
    s =  static_cast<Stream *>(stream_handle);
    s->getStreamAttributes(&sattr);
    for (int i = 0; i < no_of_devices; i++) {
        status = rm->getDeviceConfig((struct qal_device *)&devices[i], &sattr);
        if (status) {
           QAL_ERR(LOG_TAG, "Failed to get Device config, err: %d", status);
           return status;
        }
    }
    // TODO: Check with RM if the same device is being used by other stream with different
    // configuration then update corresponding stream device configuration also based on priority.
    QAL_ERR(LOG_TAG, "%s: Enter. Stream handle :%pK no_of_devices %d first_device id %d", __func__, stream_handle, no_of_devices, devices[0].id);
    status = s->switchDevice(s, no_of_devices, devices);
    if (0 != status) {
        QAL_ERR(LOG_TAG, "%s: failed with status %d", __func__, status);
        return status;
    }

    QAL_INFO(LOG_TAG, "%s: Exit. status %d", __func__, status);

    return status;
}

int32_t qal_set_param(uint32_t param_id, void *param_payload,
                      size_t payload_size)
{
    QAL_DBG(LOG_TAG, "%s: Enter:", __func__);
    int status = 0;
    std::shared_ptr<ResourceManager> rm = NULL;

    rm = ResourceManager::getInstance();

    if (rm) {
        status = rm->setParameter(param_id, param_payload, payload_size);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to set global parameter %u, status %d",
                    param_id, status);
        }
    } else {
        QAL_ERR(LOG_TAG, "%s: Qal has not been initialized yet", __func__);
        status = -EINVAL;
    }
    QAL_DBG(LOG_TAG, "%s: Exit:", __func__);
  return status;
}

int32_t qal_get_param(uint32_t param_id, void **param_payload,
                      size_t *payload_size)
{
    int status = 0;
    std::shared_ptr<ResourceManager> rm = NULL;

    rm = ResourceManager::getInstance();

    if (rm) {
        status = rm->getParameter(param_id, param_payload, payload_size);
        if (0 != status) {
            QAL_ERR(LOG_TAG, "Failed to set global parameter %u, status %d",
                    param_id, status);
        }
    } else {
        QAL_ERR(LOG_TAG, "%s: Qal has not been initialized yet", __func__);
        status = -EINVAL;
    }

    return status;
}
