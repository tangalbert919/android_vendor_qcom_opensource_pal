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

#define LOG_TAG "MCS"
#include "mcs.h"
#include "kvh2xml.h"
#include "QalDefs.h"
#include "QalApi.h"
#include <unistd.h>
#define NO_OF_BUFS 4
#define BUFFER_COUNT 32

struct mcs_play_ctxt {
    unsigned int cur_state;
    struct qts_cmd_mcs_play_req_t cur_param;
    casa_fhandle file_handle;
    casa_osal_thread_t tid;
    casa_osal_thread_attr_t tattr;
    void *stream_handle;
    size_t buffer_size;
};

struct mcs_rec_ctxt {
    unsigned int cur_state;
    struct qts_cmd_mcs_record_req_t cur_param;
    casa_fhandle file_handle;
    casa_osal_thread_t tid;
    casa_osal_thread_attr_t tattr;
    void *stream_handle;
    size_t buffer_size;
};

struct acdb_mcs {
    struct mcs_play_ctxt* pb_ctxt;
    struct mcs_rec_ctxt* rec_ctxt;
    casa_osal_mutex_t lock;
};

static struct acdb_mcs *mcs_info;

enum {
    MCS_STATE_IDLE,
    MCS_STATE_RUNNING,
    MCS_STATE_STOPPING
};



int32_t mcs_record_prepare(struct mcs_rec_ctxt* ctxt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter: ctxt - %pK", ctxt);
    int ret = 0;
    size_t size;
    size_t out_buf_size = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG, " null pointer");
        ret = CASA_EBADPARAM;
        return ret;
    }
    size = ctxt->cur_param.stream_properties.num_channels *
           ctxt->cur_param.stream_properties.bit_width * BUFFER_COUNT;

    if (ctxt->cur_param.write_to_file == 1) {
        ret = casa_fopen(&ctxt->file_handle, ctxt->cur_param.filename,
              CASA_FOPEN_WRITE_ONLY);
        CASA_LOG_VERBOSE(LOG_TAG,"casa_fread file handle %pK file name %s",
                         ctxt->file_handle, ctxt->cur_param.filename);
        if (ret != 0) {
           CASA_LOG_ERR(LOG_TAG,"file open error");
           goto exit;
        }
    }
    ret = qal_stream_set_buffer_size(ctxt->stream_handle, &size, NO_OF_BUFS, &out_buf_size, 0);
    if (ret != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream set buffer failed = %d", ret);
        goto exit;
    }
    ctxt->buffer_size = size;
    ret = qal_stream_start(ctxt->stream_handle);
    if (ret != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream start failed = %d", ret);
        goto exit;
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return ret;
exit:
    if (ctxt->file_handle != NULL){
        CASA_LOG_INFO(LOG_TAG,"file closed");
        casa_fclose(ctxt->file_handle);
    }
    return ret;
}

int32_t mcs_play_prepare(struct mcs_play_ctxt* ctxt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter: ctxt - %pK", ctxt);
    int ret = 0;
    size_t size;
    size_t in_buf_size = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG, " null pointer");
        ret = CASA_EBADPARAM;
        return ret;
    }

    size = ctxt->cur_param.stream_properties.num_channels *
           ctxt->cur_param.stream_properties.bit_width * BUFFER_COUNT;

    ret = casa_fopen(&ctxt->file_handle, ctxt->cur_param.filename,
          CASA_FOPEN_READ_ONLY);
    CASA_LOG_ERR(LOG_TAG,"%s:%d casa_fread file handle %pK",
                 __func__,__LINE__, ctxt->file_handle);
    if (ret != 0) {
        CASA_LOG_ERR(LOG_TAG,"file open error");
        goto exit;
    }
    ret = qal_stream_set_buffer_size(ctxt->stream_handle, &in_buf_size, 0, &size, NO_OF_BUFS);
    if (ret != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream set buffer failed = %d", ret);
        goto exit;
    }
    ctxt->buffer_size = size;
    ret = qal_stream_start(ctxt->stream_handle);
    if (ret != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream start failed = %d", ret);
        goto exit;
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return ret;
exit:
    if (ctxt->file_handle != NULL)
        casa_fclose(ctxt->file_handle);
    return ret;
}

int32_t mcs_play_close(struct mcs_play_ctxt* ctxt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter: ctxt - %pK", ctxt);
    int status = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"null pointer");
        return CASA_EBADPARAM;
    }
    status = qal_stream_stop(ctxt->stream_handle);
    if (status != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream stop failed");
    }
    status = qal_stream_close(ctxt->stream_handle);
    if (status != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream close failed");
    }
    if (ctxt->file_handle != NULL) {
        status = casa_fclose(ctxt->file_handle);
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return status;
}

void mcs_play_sample(void *txt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter: txt - %pK", txt);
    if (txt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"null pointer");
        return;
    }
    struct mcs_play_ctxt *ctxt = (struct mcs_play_ctxt *)txt;
    char * buffer = NULL;
    int status;
    size_t file_size = 0;
    size_t size, num_read, num_read1;
    int total_bytes_to_play = 0;
    int bytes_count = 0;
    int replay = 0;
    casa_fhandle file;
    if (ctxt->cur_param.playback_duration_sec > 0) {
        total_bytes_to_play = ctxt->cur_param.playback_duration_sec *
                              ctxt->cur_param.stream_properties.sample_rate *
                              ctxt->cur_param.stream_properties.num_channels *
                              ctxt->cur_param.stream_properties.bit_width / 8;
        replay = 1;
    }
    else
        return;
    size = ctxt->buffer_size;
    buffer = malloc(size);
    if (!buffer) {
        CASA_LOG_ERR(LOG_TAG,"Unable to allocate %d bytes\n",size);
        goto fileclose;
    }
    file_size = casa_fsize(ctxt->file_handle);
    if (file_size == CASA_EFAILED || file_size == CASA_EBADPARAM) {
        CASA_LOG_ERR(LOG_TAG,"file size error %d", file_size);
        if (replay == 1)
            goto err;
    }

    CASA_LOG_INFO(LOG_TAG,"buffer address %pK", buffer);
    do {
        status = casa_fread(ctxt->file_handle, (void*)buffer, size, &num_read);
        CASA_LOG_INFO(LOG_TAG,"casa_fread file handle %pK", ctxt->file_handle);
        if (status < 0) {
            CASA_LOG_ERR(LOG_TAG,"casa read failed");
            break;
        }
        if (num_read > 0) {
            struct qal_buffer buf;
            memset(&buf, 0, sizeof(struct qal_buffer));
            buf.buffer = (void *)buffer;
            buf.size = size;
            if (qal_stream_write(ctxt->stream_handle, &buf) < 0) {
                CASA_LOG_ERR(LOG_TAG," Error playing sample");
                break;
            }
            bytes_count += num_read;
            CASA_LOG_INFO(LOG_TAG,"%d bytes played", bytes_count);
            if (bytes_count >= total_bytes_to_play) {
                CASA_LOG_INFO(LOG_TAG,"%d total bytes played", bytes_count);
                break;
            }
        } else if (num_read == 0) {
            if ((bytes_count % file_size) == 0) {
                status = qal_stream_stop(ctxt->stream_handle);
                CASA_LOG_INFO(LOG_TAG," pause playing sample");
                status = casa_fseek(ctxt->file_handle, 0, CASA_FSEEK_BEGIN);
                CASA_LOG_INFO(LOG_TAG," fseek status = %d", status);
                num_read = size;
                status = qal_stream_start(ctxt->stream_handle);
                CASA_LOG_INFO(LOG_TAG," resume playing sample");
            }
        }
    } while ((ctxt->cur_state = MCS_STATE_RUNNING) && num_read > 0);
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    free(buffer);
    return;
err:
    if (buffer)
        free(buffer);
fileclose:
    if (ctxt->file_handle)
        casa_fclose(ctxt->file_handle);
}

int32_t process_playback_request(uint8_t cmd,
                                struct mcs_play_ctxt* ctxt,
                                struct qts_cmd_mcs_play_req_t *param)
{
    CASA_LOG_INFO(LOG_TAG,"enter cmd = %x", cmd);
    int ret = 0;
    struct qal_stream_attributes sattr;
    struct qal_device *dattr = NULL;
    int no_of_devices = 0;
    int i = 0;
    int j = 0;
    void *tid1 = NULL;
    void *tid2 = NULL;
    struct qal_channel_info streamch, devicech;
    int no_of_kv_pairs = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s:%d null pointer",__func__,__LINE__);
        return CASA_EBADPARAM;
    }

    switch(cmd) {
        case MCS_START:
            if (param == NULL || param->graph_key_vector.graph_key_vector == NULL ) {
                CASA_LOG_ERR(LOG_TAG,"%s:%d null pointer",__func__,__LINE__);
                return CASA_EBADPARAM;
            }
            if(ctxt->cur_state == MCS_STATE_IDLE) {
                memcpy(&ctxt->cur_param, param, sizeof(ctxt->cur_param));
                no_of_kv_pairs = param->graph_key_vector.num_keys;
                for (i = 0; i < no_of_kv_pairs; i++) {
                    if (param->graph_key_vector.graph_key_vector[i].key == STREAMRX) {
                        switch(param->graph_key_vector.graph_key_vector[i].value){
                            case PCM_LL_PLAYBACK:
                                sattr.type = QAL_STREAM_LOW_LATENCY;
                                sattr.direction = QAL_AUDIO_OUTPUT;
                                sattr.info.opt_stream_info.version = 0;
                                sattr.info.opt_stream_info.size = 0;
                                sattr.info.opt_stream_info.duration_us = 0;
                                sattr.info.opt_stream_info.has_video = false;
                                sattr.info.opt_stream_info.is_streaming = false;
                                sattr.flags = QAL_META_DATA_FLAGS_NONE;
                                sattr.out_media_config.sample_rate = param->stream_properties.sample_rate;
                                sattr.out_media_config.bit_width = param->stream_properties.bit_width;
                                sattr.out_media_config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
                                streamch.channels = param->stream_properties.num_channels;
                                sattr.out_media_config.ch_info = &streamch;
                                break;
                            default:
                                CASA_LOG_ERR(LOG_TAG,"unsupported stream type");
                                return CASA_EFAILED;
                        }
                    }
                    else if (param->graph_key_vector.graph_key_vector[i].key == DEVICERX)
                        no_of_devices++;
                }
                dattr = (struct qal_device*)malloc(sizeof(struct qal_device));
                if (dattr == NULL) {
                    CASA_LOG_ERR(LOG_TAG,"malloc failure");
                    return CASA_ENOMEMORY;
                }
                j = 0;
                for (i = 0; i < no_of_kv_pairs; i++) {
                    if (param->graph_key_vector.graph_key_vector[i].key == DEVICERX) {
                        dattr[j].config.sample_rate = param->stream_properties.sample_rate;
                        dattr[j].config.bit_width = param->stream_properties.bit_width;
                        devicech.channels = param->stream_properties.num_channels;
                        dattr[j].config.ch_info = &devicech;
                        switch (param->graph_key_vector.graph_key_vector[i].value) {
                            case SPEAKER:
                                dattr[j].id = QAL_DEVICE_OUT_SPEAKER;
                                break;
                            default:
                                CASA_LOG_ERR(LOG_TAG,"unsupported device");
                                ret = CASA_EFAILED;
                                break;
                        }
                        j++;
                    }
                }
            }
            CASA_LOG_INFO(LOG_TAG,"no of devices %d %pK", no_of_devices, ctxt);
            ret = qal_stream_open(&sattr, no_of_devices, dattr, 0, NULL, NULL, NULL, &ctxt->stream_handle);
            if (ret != 0) {
                CASA_LOG_ERR(LOG_TAG,"qal_stream_open failed");
                return ret;
            }
            ret = mcs_play_prepare(ctxt);
            if(ret == 0) {
                ret = casa_osal_thread_create(&mcs_info->pb_ctxt->tid, &mcs_info->pb_ctxt->tattr, mcs_play_sample, (void *)ctxt);
                if (ret == 0) {
                    ctxt->cur_state = MCS_STATE_RUNNING;
                }
            } else {
                qal_stream_close(ctxt->stream_handle);
                if (dattr)
                    free(dattr);
            }
            break;
        case MCS_STOP:
            if (ctxt->cur_state != MCS_STATE_IDLE) {
                CASA_LOG_INFO(LOG_TAG,"%s:%d mcs_stop called", __func__, __LINE__);
                ctxt->cur_state = MCS_STATE_STOPPING;
                ret = casa_osal_thread_join_destroy(mcs_info->pb_ctxt->tid);
                ret = mcs_play_close(ctxt);
                ctxt->cur_state = MCS_STATE_IDLE;
                free(dattr);
                return ret;
            }
            break;
        default:
            CASA_LOG_ERR(LOG_TAG,"cmd not found  %x ", cmd);
            ret = CASA_EFAILED;
            break;
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return ret;
}

void mcs_record_sample(void *txt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter : txt %pK", txt);
    struct mcs_rec_ctxt* ctxt = (struct mcs_rec_ctxt*)txt;
    char * buffer;
    int status;
    size_t size, num_read;
    int total_bytes_to_record = 0;
    int bytes_count = 0;
    ssize_t bytes_read = 0;
    int replay = 0;

    struct qal_buffer in_buffer;
    memset(&in_buffer, 0, sizeof(struct qal_buffer));

    if (ctxt->cur_param.record_duration_sec > 0) {
        total_bytes_to_record = ctxt->cur_param.record_duration_sec *
                    ctxt->cur_param.stream_properties.sample_rate *
                    ctxt->cur_param.stream_properties.num_channels *
                    ctxt->cur_param.stream_properties.bit_width / 8;
        replay = 1;
    }

    size = ctxt->buffer_size;
    buffer = malloc(size);
    if (!buffer) {
        CASA_LOG_ERR(LOG_TAG,"Unable to allocate %d bytes\n",size);
        goto fileclose;
    }

    while (ctxt->cur_state == MCS_STATE_RUNNING) {
        in_buffer.buffer = buffer;
        in_buffer.size = size;
        bytes_read = qal_stream_read(ctxt->stream_handle, &in_buffer);
        if (ctxt->cur_param.write_to_file == 1) {
            CASA_LOG_INFO(LOG_TAG,"bytes recorded =%d ", bytes_read);
            status = casa_fwrite(ctxt->file_handle, buffer, bytes_read, &num_read);
            CASA_LOG_INFO(LOG_TAG,"casa_fread file handle %pK", ctxt->file_handle);
            bytes_count += num_read;
            if ((ctxt->cur_param.record_duration_sec > 0) &&
                (bytes_count >= total_bytes_to_record)) {
                CASA_LOG_INFO(LOG_TAG,"total bytes recorded =%d ", bytes_count);
                break;
            }
        }
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    if (buffer)
        free(buffer);
    return;
fileclose:
    if (ctxt->file_handle)
        casa_fclose(ctxt->file_handle);

}

int32_t mcs_record_close(struct mcs_rec_ctxt* ctxt)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter - ctxt %pK", ctxt);
    int status = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG, " null pointer");
        return CASA_EBADPARAM;
    }
    status = qal_stream_stop(ctxt->stream_handle);
    if (status != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream stop failed");
    }
    status = qal_stream_close(ctxt->stream_handle);
    if (status != 0) {
        CASA_LOG_ERR(LOG_TAG,"stream close failed");
    }
    if (ctxt->cur_param.write_to_file == 1) {
        if (ctxt->file_handle != NULL) {
            status = casa_fclose(ctxt->file_handle);
        }
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return status;
}

int32_t process_record_request(uint8_t cmd,
                                struct mcs_rec_ctxt* ctxt,
                                struct qts_cmd_mcs_record_req_t *param)
{
    CASA_LOG_VERBOSE(LOG_TAG,"enter - ctxt %pK", ctxt);
    int ret = 0;
    struct qal_stream_attributes sattr;
    struct qal_device *dattr = NULL;
    int no_of_devices = 0;
    int i = 0;
    int j = 0;
    void *tid1 = NULL;
    void *tid2 = NULL;
    struct qal_channel_info streamch, devicech;
    int no_of_kv_pairs = 0;
    if (ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s:%d null pointer",__func__,__LINE__);
        return CASA_EBADPARAM;
    }

    switch(cmd) {
        case MCS_START:
            if (param == NULL || param->graph_key_vector.graph_key_vector == NULL) {
                CASA_LOG_ERR(LOG_TAG,"%s:%d null pointer",__func__,__LINE__);
                return CASA_EBADPARAM;
            }
            if(ctxt->cur_state == MCS_STATE_IDLE) {
                memcpy(&ctxt->cur_param, param, sizeof(ctxt->cur_param));
                no_of_kv_pairs = param->graph_key_vector.num_keys;
                for (i = 0; i < no_of_kv_pairs; i++) {
                    if (param->graph_key_vector.graph_key_vector[i].key == STREAMTX) {
                        switch(param->graph_key_vector.graph_key_vector[i].value){
                            case PCM_RECORD:
                                sattr.type = QAL_STREAM_LOW_LATENCY;
                                sattr.direction = QAL_AUDIO_INPUT;
                                sattr.info.opt_stream_info.version = 0;
                                sattr.info.opt_stream_info.size = 0;
                                sattr.info.opt_stream_info.duration_us = 0;
                                sattr.info.opt_stream_info.has_video = false;
                                sattr.info.opt_stream_info.is_streaming = false;
                                sattr.flags = QAL_META_DATA_FLAGS_NONE;
                                sattr.in_media_config.sample_rate = param->stream_properties.sample_rate;
                                sattr.in_media_config.bit_width = param->stream_properties.bit_width;
                                sattr.in_media_config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;
                                streamch.channels = param->stream_properties.num_channels;
                                sattr.in_media_config.ch_info = &streamch;
                                break;
                            default:
                                CASA_LOG_ERR(LOG_TAG,"unsupported stream type");
                                return CASA_EFAILED;
                        }
                    }
                    if (param->graph_key_vector.graph_key_vector[i].key == DEVICETX)
                        no_of_devices++;
                }
                dattr = (struct qal_device*)calloc(no_of_devices,sizeof(struct qal_device));
                if (dattr == NULL) {
                    CASA_LOG_ERR(LOG_TAG,"malloc failure");
                    return CASA_ENOMEMORY;
                }
                j = 0;
                for (i = 0; i < no_of_kv_pairs; i++) {
                    if (param->graph_key_vector.graph_key_vector[i].key == DEVICETX) {
                        dattr[j].config.sample_rate = param->stream_properties.sample_rate;
                        dattr[j].config.bit_width = param->stream_properties.bit_width;
                        devicech.channels = param->stream_properties.num_channels;
                        dattr[j].config.ch_info = &devicech;
                        switch (param->graph_key_vector.graph_key_vector[i].value) {
                            case HANDSETMIC:
                                switch(devicech.channels) {
                                    case 1:
                                        dattr[j].id = QAL_DEVICE_IN_HANDSET_MIC;
                                        break;
                                    case 2:
                                    case 3:
                                    case 4:
                                        dattr[j].id = QAL_DEVICE_IN_SPEAKER_MIC;
                                        break;
                                    default:
                                        CASA_LOG_ERR(LOG_TAG,"unsupported mic");
                                        ret = CASA_EFAILED;
                                        break;
                                }
                            default:
                                CASA_LOG_ERR(LOG_TAG,"unsupported device");
                                ret = CASA_EFAILED;
                                break;
                        }
                        j++;
                    }
                }
            }

            ret = qal_stream_open(&sattr, no_of_devices, dattr, 0, NULL, NULL, NULL, &ctxt->stream_handle);
            if (ret != 0) {
                CASA_LOG_ERR(LOG_TAG,"qal_stream_open failed");
                return ret;
            }
            ret = mcs_record_prepare(ctxt);
            if (ret == 0) {
                ret = casa_osal_thread_create(&mcs_info->rec_ctxt->tid, &mcs_info->rec_ctxt->tattr, mcs_record_sample, (void *)ctxt);
                if (ret == 0) {
                    ctxt->cur_state = MCS_STATE_RUNNING;
                }
            } else {
                qal_stream_close(ctxt->stream_handle);
                if (dattr)
                    free(dattr);
            }
            break;

        case MCS_STOP:
            if (ctxt->cur_state != MCS_STATE_IDLE) {
                CASA_LOG_INFO(LOG_TAG,"%s:%d", __func__, __LINE__);
                ctxt->cur_state = MCS_STATE_STOPPING;
                ret = casa_osal_thread_join_destroy(mcs_info->rec_ctxt->tid);
                ret = mcs_record_close(ctxt);
                ctxt->cur_state = MCS_STATE_IDLE;
                free(dattr);
                return ret;
            }
            break;
        default:
            CASA_LOG_ERR(LOG_TAG,"Invalid command");
            ret = CASA_EFAILED;
            break;
    }
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return ret;
}

int32_t mcs_stream_cmd(uint32_t cmd, uint8_t *cmd_buf,
                     uint32_t cmd_buf_size, uint8_t*rsp_buf,
                     uint32_t rsp_buf_size, uint32_t *rsp_buf_bytes_filled)
{
    CASA_LOG_INFO(LOG_TAG,"Enter %x cmd", cmd);
    int ret = 0;
    struct qts_cmd_mcs_play_record_req_t *pdata;
    casa_osal_mutex_lock(mcs_info->lock);

    switch(cmd) {
        case QTS_CMD_MCS_PLAY:
            ret = process_playback_request(MCS_START, mcs_info->pb_ctxt,
                                           (struct qts_cmd_mcs_play_req_t*) cmd_buf);
            break;
        case QTS_CMD_MCS_RECORD:
            ret = process_record_request(MCS_START, mcs_info->rec_ctxt,
                                         (struct qts_cmd_mcs_record_req_t*) cmd_buf);
            break;
        case QTS_CMD_MCS_PLAY_RECORD:
            pdata = (struct qts_cmd_mcs_play_record_req_t *)cmd_buf;
            ret = process_playback_request(MCS_START, mcs_info->pb_ctxt,
                                            &pdata->playback_session);
            if (ret == 0) {
                ret = process_record_request(MCS_START, mcs_info->rec_ctxt,
                                             &pdata->record_session);
                if (ret != 0) {
                    process_playback_request(MCS_STOP, mcs_info->pb_ctxt,
                                              &pdata->playback_session);
                }
            }
            break;
        case QTS_CMD_MCS_STOP:
            ret = process_playback_request(MCS_STOP, mcs_info->pb_ctxt,
                                           NULL);
            if (ret == 0)
                ret = process_record_request(MCS_STOP, mcs_info->rec_ctxt,
                                                NULL);
            break;

        default:
            CASA_LOG_ERR(LOG_TAG,"%s: invalid command ID from QTS: 0x%x\n", __func__, cmd);
            ret = CASA_EFAILED;
            break;
    }
    casa_osal_mutex_unlock(mcs_info->lock);
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return ret;
}

int32_t mcs_init()
{
    int ret;
    CASA_LOG_VERBOSE(LOG_TAG,"enter");
    mcs_info = calloc(1, sizeof(struct acdb_mcs));
    if (mcs_info == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: memory allocation failed.", __func__);
        ret = -CASA_ENOMEMORY;
        goto err_mcs;
    }

    mcs_info->pb_ctxt = calloc(1, sizeof(struct mcs_play_ctxt));
    if (mcs_info->pb_ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: memory allocation failed.", __func__);
        ret = -CASA_ENOMEMORY;
        goto err_pb_ctxt;
    }
    mcs_info->pb_ctxt->cur_state = MCS_STATE_IDLE;

    mcs_info->rec_ctxt = calloc(1, sizeof(struct mcs_rec_ctxt));
    if (mcs_info->rec_ctxt == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: memory allocation failed.", __func__);
        ret = -CASA_ENOMEMORY;
        goto err_rec_ctxt;
    }
    mcs_info->rec_ctxt->cur_state = MCS_STATE_IDLE;

    casa_osal_mutex_create(&mcs_info->lock);

    casa_osal_thread_attr_init(&mcs_info->pb_ctxt->tattr);

    casa_osal_thread_attr_init(&mcs_info->rec_ctxt->tattr);

    mcs_info->pb_ctxt->tid = NULL;
    mcs_info->rec_ctxt->tid = NULL;
    CASA_LOG_VERBOSE(LOG_TAG,"exit");
    return 0;

err_ret:
    free(mcs_info->rec_ctxt);
    mcs_info->rec_ctxt = NULL;
err_rec_ctxt:
    free(mcs_info->pb_ctxt);
    mcs_info->pb_ctxt = NULL;
err_pb_ctxt:
    free(mcs_info);
    mcs_info = NULL;
err_mcs:
    return ret;
}
