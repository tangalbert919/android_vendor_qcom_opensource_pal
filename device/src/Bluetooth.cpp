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

#define LOG_TAG "Bluetooth"
#include "Bluetooth.h"
#include "ResourceManager.h"
#include "PayloadBuilder.h"
#include "Stream.h"
#include "Session.h"
#include "Device.h"
#include "kvh2xml.h"
#include <dlfcn.h>
#include <unistd.h>
#include <cutils/properties.h>

#define BT_IPC_SOURCE_LIB "btaudio_offload_if.so"
#define BT_IPC_SINK_LIB "libbthost_if_sink.so"

Bluetooth::Bluetooth(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
    : Device(device, Rm)
{
}

Bluetooth::~Bluetooth()
{
}

void Bluetooth::updateDeviceAttributes(codec_format_t codec_format, codec_type type)
{
    deviceAttr.config.sample_rate = encoderConfig.sample_rate;

    switch (codec_format) {
    case CODEC_TYPE_AAC:
    case CODEC_TYPE_SBC:
        if (type == DEC &&
            (encoderConfig.sample_rate == 44100 ||
             encoderConfig.sample_rate == 48000))
            deviceAttr.config.sample_rate = encoderConfig.sample_rate * 2;
        break;
    case CODEC_TYPE_LDAC:
    case CODEC_TYPE_APTX_AD:
        if (type == ENC &&
            (encoderConfig.sample_rate == 44100 ||
             encoderConfig.sample_rate == 48000))
        deviceAttr.config.sample_rate = encoderConfig.sample_rate * 2;
        break;
    default:
        break;
    }
}

bool Bluetooth::configureA2dpEncoderDecoder(codec_format_t codec_format, void *codec_info, codec_type type)
{
    void *handle = NULL;
    int status = 0, i;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activestreams;
    const char *err = NULL;
    open_fn_t plugin_open_fn = NULL;
    bt_codec_t *codec = NULL;
    bt_enc_payload_t *out_buf = NULL;
    std::string lib_path;
    PayloadBuilder* builder = new PayloadBuilder();
    std::string backEndName;
    uint8_t* paramData = NULL;
    size_t paramSize = 0;
    uint32_t miid = 0, ratMiid = 0, copMiid = 0;
    std::shared_ptr<Device> dev = nullptr;
    uint32_t num_payloads;

    is_configured = false;
    rm->getBackendName(deviceAttr.id, backEndName);

    dev = Device::getInstance(&deviceAttr, rm);
    status = rm->getActiveStream(dev, activestreams);
    if ((0 != status) || (activestreams.size() == 0)) {
        QAL_ERR(LOG_TAG, "%s: no active stream available", __func__);
        return false;
    }
    stream = static_cast<Stream *>(activestreams[0]);
    stream->getAssociatedSession(&session);
    QAL_DBG(LOG_TAG, "%s: choose BT codec format %d", __func__, codec_format);


    /* Retrieve plugin library from resource manager.
     * Map to interested symbols.
     */
    lib_path = rm->getBtCodecLib(codec_format, (type == ENC ? "enc" : "dec"));
    if (lib_path.empty()) {
        QAL_ERR(LOG_TAG, "%s: fail to get BT codec library", __func__);
        return false;
    }
    handle = dlopen(lib_path.c_str(), RTLD_NOW);
    if (handle == NULL) {
        QAL_ERR(LOG_TAG, "%s: failed to dlopen lib %s", __func__, lib_path.c_str());
        return false;
    }

    dlerror();
    plugin_open_fn = (open_fn_t)dlsym(handle, "plugin_open");
    if (!plugin_open_fn) {
        QAL_ERR(LOG_TAG, "%s: dlsym to open fn failed, err = '%s'\n",
                __func__, dlerror());
        goto error;
    }

    status = plugin_open_fn(&codec, codec_format, type);
    if (status) {
        QAL_ERR(LOG_TAG, "%s: failed to open plugin %d", __func__, status);
        goto error;
    }

    status = codec->plugin_populate_payload(codec, codec_info, (void **)&out_buf);
    if (status != 0) {
        QAL_ERR(LOG_TAG, "%s: fail to pack the encoder config %d", __func__, status);
        goto error;
    }

    encoderConfig.sample_rate = out_buf->sample_rate;
    encoderConfig.bit_width = out_buf->bit_format;
    encoderConfig.ch_info = (struct qal_channel_info *) calloc(1,sizeof(uint16_t) +
                                   (sizeof(uint8_t) * out_buf->channel_count));
    encoderConfig.ch_info->channels = out_buf->channel_count;

    // Get BT placeholder encoder MIID
    status = session->getMIID(backEndName.c_str(), BT_PLACEHOLDER_ENCODER, &miid);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", BT_PLACEHOLDER_ENCODER, status);
        goto error;
    }

    status = session->getMIID(backEndName.c_str(), RAT_RENDER, &ratMiid);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", RAT_RENDER, status);
        goto error;
    }

    status = session->getMIID(backEndName.c_str(), COP_PACKETIZER_V0, &copMiid);
    if (status) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d\n",
                                             COP_PACKETIZER_V0, status);
        goto error;
    }

    num_payloads = out_buf->num_blks;
    for (i = 0; i < num_payloads; i++) {
        custom_block_t *blk = out_buf->blocks[i];
        builder->payloadCustomParam(&paramData, &paramSize,
                  (uint32_t *)blk->payload, blk->payload_sz, miid, blk->param_id);
        if (!paramData) {
            QAL_ERR(LOG_TAG, "Failed to populateAPMHeader\n");
            status = -ENOMEM;
            goto error;
        }
        dev->updateCustomPayload(paramData, paramSize);
        free(paramData);
        paramData = NULL;
        paramSize = 0;
    }

    builder->payloadRATConfig(&paramData, &paramSize, ratMiid, &encoderConfig);
    if (paramSize) {
        dev->updateCustomPayload(paramData, paramSize);
        free(paramData);
        paramData = NULL;
        paramSize = 0;
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid RAT module param size", __func__);
        goto error;
    }

/* Update Device sampleRate based on encoder config */
    updateDeviceAttributes(codec_format, type);

    builder->payloadCopPackConfig(&paramData, &paramSize, copMiid, &deviceAttr.config);
    if (paramSize) {
        dev->updateCustomPayload(paramData, paramSize);
        free(paramData);
        paramData = NULL;
        paramSize = 0;
    } else {
        status = -EINVAL;
        QAL_ERR(LOG_TAG, "%s: Invalid RAT module param size", __func__);
        goto error;
    }

    is_configured = true;

error:
    if (codec)
        codec->close_plugin(codec);

    if (handle)
        dlclose(handle);

    return is_configured;
}

/* Scope of BtA2dp class */
// definition of static BtA2dp member variables
std::shared_ptr<Device> BtA2dp::obj = nullptr;
void *BtA2dp::bt_lib_source_handle = nullptr;
void *BtA2dp::bt_lib_sink_handle = nullptr;
bt_audio_pre_init_t BtA2dp::bt_audio_pre_init = nullptr;
audio_source_open_t BtA2dp::audio_source_open = nullptr;
audio_source_close_t BtA2dp::audio_source_close = nullptr;
audio_source_start_t BtA2dp::audio_source_start = nullptr;
audio_source_stop_t BtA2dp::audio_source_stop = nullptr;
audio_source_suspend_t BtA2dp::audio_source_suspend = nullptr;
audio_source_handoff_triggered_t BtA2dp::audio_source_handoff_triggered = nullptr;
clear_source_a2dpsuspend_flag_t BtA2dp::clear_source_a2dpsuspend_flag = nullptr;
audio_get_enc_config_t BtA2dp::audio_get_enc_config = nullptr;
audio_source_check_a2dp_ready_t BtA2dp::audio_source_check_a2dp_ready = nullptr;
audio_is_tws_mono_mode_enable_t BtA2dp::audio_is_tws_mono_mode_enable = nullptr;
audio_is_source_scrambling_enabled_t BtA2dp::audio_is_source_scrambling_enabled = nullptr;
audio_sink_get_a2dp_latency_t BtA2dp::audio_sink_get_a2dp_latency = nullptr;
audio_sink_start_t BtA2dp::audio_sink_start = nullptr;
audio_sink_stop_t BtA2dp::audio_sink_stop = nullptr;
audio_get_dec_config_t BtA2dp::audio_get_dec_config = nullptr;
audio_sink_session_setup_complete_t BtA2dp::audio_sink_session_setup_complete = nullptr;
audio_sink_check_a2dp_ready_t BtA2dp::audio_sink_check_a2dp_ready = nullptr;


BtA2dp::BtA2dp(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
      : Bluetooth(device, Rm),
        bt_state_source(A2DP_STATE_DISCONNECTED),
        a2dp_source_started(false),
        a2dp_source_total_active_session_requests(0),
        is_a2dp_offload_supported(false),
        is_handoff_in_progress(false),
        bt_state_sink(A2DP_STATE_DISCONNECTED),
        bt_decoder_format(CODEC_TYPE_INVALID),
        a2dp_sink_started(false),
        a2dp_sink_total_active_session_requests(0)
{
    a2dp_role = (device->id == QAL_DEVICE_IN_BLUETOOTH_A2DP) ? SINK : SOURCE;

    init();
    sleep(1); //TODO: to add interval properly
    open_a2dp_source();
    param_bt_a2dp.reconfigured = false;
    param_bt_a2dp.a2dp_suspended = false;
    is_a2dp_offload_supported =
            property_get_bool("ro.bluetooth.a2dp_offload.supported", false) &&
            !property_get_bool("persist.bluetooth.a2dp_offload.disabled", false);

    QAL_DBG(LOG_TAG, "%s: A2DP offload supported = %d", __func__,
            is_a2dp_offload_supported);
    param_bt_a2dp.reconfig_supported = is_a2dp_offload_supported;
    is_configured = false;
}

BtA2dp::~BtA2dp()
{
    is_handoff_in_progress = false;
}

void BtA2dp::open_a2dp_source()
{
    int ret = 0;

    QAL_DBG(LOG_TAG, "Open A2DP source start");
    if (bt_lib_source_handle && audio_source_open) {
        if (bt_state_source == A2DP_STATE_DISCONNECTED) {
            QAL_DBG(LOG_TAG, "calling BT stream open");
            ret = audio_source_open();
            if (ret != 0) {
                QAL_ERR(LOG_TAG, "Failed to open source stream for a2dp: status %d", ret);
            }
            bt_state_source = A2DP_STATE_CONNECTED;
        } else {
            QAL_DBG(LOG_TAG, "Called a2dp open with improper state %d", bt_state_source);
        }
    } else {
        QAL_ERR(LOG_TAG, "a2dp handle is not identified, Ignoring open request");
        bt_state_source = A2DP_STATE_DISCONNECTED;
    }
}

int BtA2dp::close_audio_source()
{
    QAL_VERBOSE(LOG_TAG, "%s\n", __func__);

    if (!(bt_lib_source_handle && audio_source_close)) {
        QAL_ERR(LOG_TAG, "a2dp source handle is not identified, Ignoring close request");
        return -ENOSYS;
    }

    if (bt_state_source != A2DP_STATE_DISCONNECTED) {
        QAL_DBG(LOG_TAG, "calling BT source stream close");
        if (audio_source_close() == false)
            QAL_ERR(LOG_TAG, "failed close a2dp source control path from BT library");
    }
    a2dp_source_started = false;
    a2dp_source_total_active_session_requests = 0;
    param_bt_a2dp.a2dp_suspended = false;
    param_bt_a2dp.reconfigured = false;
    bt_state_source = A2DP_STATE_DISCONNECTED;

    return 0;
}

void BtA2dp::init_a2dp_source()
{
    QAL_DBG(LOG_TAG, "init_a2dp_source START");
    if (bt_lib_source_handle == nullptr) {
        QAL_DBG(LOG_TAG, "Requesting for BT lib handle");
        bt_lib_source_handle = dlopen(BT_IPC_SOURCE_LIB, RTLD_NOW);
        if (bt_lib_source_handle == nullptr) {
            QAL_ERR(LOG_TAG, "%s: dlopen failed for %s", __func__, BT_IPC_SOURCE_LIB);
            return;
        }
    }
    bt_audio_pre_init = (bt_audio_pre_init_t)
                  dlsym(bt_lib_source_handle, "bt_audio_pre_init");
    audio_source_open = (audio_source_open_t)
                  dlsym(bt_lib_source_handle, "audio_stream_open");
    audio_source_start = (audio_source_start_t)
                  dlsym(bt_lib_source_handle, "audio_start_stream");
    audio_get_enc_config = (audio_get_enc_config_t)
                  dlsym(bt_lib_source_handle, "audio_get_codec_config");
    audio_source_suspend = (audio_source_suspend_t)
                  dlsym(bt_lib_source_handle, "audio_suspend_stream");
    audio_source_handoff_triggered = (audio_source_handoff_triggered_t)
                  dlsym(bt_lib_source_handle, "audio_handoff_triggered");
    clear_source_a2dpsuspend_flag = (clear_source_a2dpsuspend_flag_t)
                  dlsym(bt_lib_source_handle, "clear_a2dpsuspend_flag");
    audio_source_stop = (audio_source_stop_t)
                  dlsym(bt_lib_source_handle, "audio_stop_stream");
    audio_source_close = (audio_source_close_t)
                  dlsym(bt_lib_source_handle, "audio_stream_close");
    audio_source_check_a2dp_ready = (audio_source_check_a2dp_ready_t)
                  dlsym(bt_lib_source_handle, "audio_check_a2dp_ready");
    audio_sink_get_a2dp_latency = (audio_sink_get_a2dp_latency_t)
                  dlsym(bt_lib_source_handle, "audio_sink_get_a2dp_latency");
    audio_is_source_scrambling_enabled = (audio_is_source_scrambling_enabled_t)
                  dlsym(bt_lib_source_handle, "audio_is_scrambling_enabled");
    audio_is_tws_mono_mode_enable = (audio_is_tws_mono_mode_enable_t)
                  dlsym(bt_lib_source_handle, "isTwsMonomodeEnable");

    if (bt_lib_source_handle && bt_audio_pre_init) {
        QAL_DBG(LOG_TAG, "calling BT module preinit");
        bt_audio_pre_init();
    }
}

//TODO
void BtA2dp::audio_a2dp_update_tws_channel_mode()
{
}

void BtA2dp::init_a2dp_sink()
{
    QAL_DBG(LOG_TAG, "Open A2DP input start");
    if (bt_lib_sink_handle == nullptr) {
        QAL_DBG(LOG_TAG, "Requesting for BT lib handle");
        bt_lib_sink_handle = dlopen(BT_IPC_SINK_LIB, RTLD_NOW);

        if (bt_lib_sink_handle == nullptr) {
            QAL_ERR(LOG_TAG, "%s: DLOPEN failed for %s", __func__, BT_IPC_SINK_LIB);
        } else {
            audio_sink_start = (audio_sink_start_t)
                          dlsym(bt_lib_sink_handle, "audio_sink_start_capture");
            audio_get_dec_config = (audio_get_dec_config_t)
                          dlsym(bt_lib_sink_handle, "audio_get_decoder_config");
            audio_sink_stop = (audio_sink_stop_t)
                          dlsym(bt_lib_sink_handle, "audio_sink_stop_capture");
            audio_sink_check_a2dp_ready = (audio_sink_check_a2dp_ready_t)
                          dlsym(bt_lib_sink_handle, "audio_sink_check_a2dp_ready");
            audio_sink_session_setup_complete = (audio_sink_session_setup_complete_t)
                          dlsym(bt_lib_sink_handle, "audio_sink_session_setup_complete");
        }
    }
}

//TODO: reserved for BT sink
int  BtA2dp::close_a2dp_input()
{
    return 0;
}

bool BtA2dp::a2dp_send_sink_setup_complete()
{
    uint64_t system_latency = 0;
    bool is_complete = false;

    system_latency = getDecLatency();

    if (audio_sink_session_setup_complete(system_latency) == 0) {
        is_complete = true;
    }
    return is_complete;
}

void BtA2dp::init()
{
    (a2dp_role == SOURCE) ? init_a2dp_source() : init_a2dp_sink();
}

int BtA2dp::start()
{
    int status = 0;

    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;

    if (customPayload)
        free(customPayload);

    customPayload = NULL;
    customPayloadSize = 0;

    status = (a2dp_role == SOURCE) ? startPlayback() : startCapture();
    if (status != 0)
        return status;

    bt_state_source = A2DP_STATE_STARTED;
    status = Device::start();

    return status;
}

int BtA2dp::stop()
{
    int status = 0;

    status = Device::stop();
    if (status != 0)
        return status;

    status = (a2dp_role == SOURCE) ? stopPlayback() : stopCapture();

    return status;
}

int BtA2dp::startPlayback()
{
    int ret = 0;
    codec_format_t format = CODEC_TYPE_INVALID;
    void *codec_info = nullptr;
    uint8_t multi_cast = 0, num_dev = 1;

    QAL_DBG(LOG_TAG, "a2dp_start_playback start");

    if (!(bt_lib_source_handle && audio_source_start
                && audio_get_enc_config)) {
        QAL_ERR(LOG_TAG, "a2dp handle is not identified, Ignoring start playback request");
        return -ENOSYS;
    }

    if (param_bt_a2dp.a2dp_suspended) {
        //session will be restarted after suspend completion
        QAL_DBG(LOG_TAG, "a2dp start requested during suspend state");
        return -ENOSYS;
    }

    if (!a2dp_source_started && !a2dp_source_total_active_session_requests) {
        QAL_DBG(LOG_TAG, "calling BT module stream start");
        /* This call indicates BT IPC lib to start playback */
        ret =  audio_source_start();
        QAL_ERR(LOG_TAG, "BT controller start return = %d",ret);
        if (ret != 0 ) {
           QAL_ERR(LOG_TAG, "BT controller start failed");
           return ret;
        }

        QAL_DBG(LOG_TAG, "configure_a2dp_encoder_format start");
        codec_info = audio_get_enc_config(&multi_cast, &num_dev, &format);
        if (codec_info == NULL || format == CODEC_TYPE_INVALID) {
            QAL_ERR(LOG_TAG, "%s: invalid encoder config", __func__);
            audio_source_stop();
            return -EINVAL;
        }

        if (configureA2dpEncoderDecoder(format, codec_info, ENC) == true) {
            QAL_DBG(LOG_TAG, "Start playback successful to BT library");
        } else {
            QAL_ERR(LOG_TAG, "unable to configure DSP encoder");
            audio_source_stop();
            return -ETIMEDOUT;
        }
    }
    a2dp_source_started = true;

    a2dp_source_total_active_session_requests++;
    QAL_DBG(LOG_TAG, "start A2DP playback total active sessions :%d",
            a2dp_source_total_active_session_requests);
    return ret;
}

int BtA2dp::stopPlayback()
{
    int ret =0;

    QAL_VERBOSE(LOG_TAG, "a2dp_stop_playback start");
    if (!(bt_lib_source_handle && audio_source_stop)) {
        QAL_ERR(LOG_TAG, "a2dp handle is not identified, Ignoring stop request");
        return -ENOSYS;
    }

    if (a2dp_source_total_active_session_requests > 0)
        a2dp_source_total_active_session_requests--;
    else
        QAL_ERR(LOG_TAG, "%s: No active playback session requests on A2DP", __func__);

    if (a2dp_source_started && !a2dp_source_total_active_session_requests) {
        QAL_VERBOSE(LOG_TAG, "calling BT module stream stop");
        ret = audio_source_stop();
        if (ret < 0) {
            QAL_ERR(LOG_TAG, "stop stream to BT IPC lib failed");
        } else {
            QAL_VERBOSE(LOG_TAG, "stop steam to BT IPC lib successful");
        }
        a2dp_source_started = false;
        is_configured = false;
        bt_state_source = A2DP_STATE_STOPPED;
    }

    QAL_DBG(LOG_TAG, "Stop A2DP playback, total active sessions :%d",
            a2dp_source_total_active_session_requests);
    return 0;
}

bool BtA2dp::isDeviceReady()
{
    bool ret = false;

    if (param_bt_a2dp.a2dp_suspended)
        return ret;

    if ((bt_state_source != A2DP_STATE_DISCONNECTED) &&
        (is_a2dp_offload_supported) &&
        (audio_source_check_a2dp_ready))
           ret = audio_source_check_a2dp_ready();
    return ret;
}

uint32_t BtA2dp::getEncLatency()
{
    //TODO
    return 0;
}

int BtA2dp::getDeviceAttributes(struct qal_device *dattr)
{
    int status = 0;

    if (!dattr) {
        status = -EINVAL;
        QAL_ERR(LOG_TAG,"Invalid device attributes status %d", status);
        goto exit;
    }

    if (is_configured) {
        dattr->id = deviceAttr.id;
        casa_osal_memcpy(&dattr->config, sizeof(struct qal_media_config), &encoderConfig, sizeof(struct qal_media_config));
    } else {
        casa_osal_memcpy(dattr, sizeof(struct qal_device), &deviceAttr, sizeof(struct qal_device));
    }

exit:
    return status;
}

int BtA2dp::startCapture()
{
    int ret = 0;
    codec_format_t format = CODEC_TYPE_INVALID;
    void *codec_info = nullptr;

    QAL_DBG(LOG_TAG, "a2dp_start_capture start");

    if (!(bt_lib_sink_handle && audio_sink_start
       && audio_get_dec_config)) {
        QAL_ERR(LOG_TAG, "a2dp handle is not identified, Ignoring start capture request");
        return -ENOSYS;
    }

    if (!a2dp_sink_started && !a2dp_sink_total_active_session_requests) {
        QAL_DBG(LOG_TAG, "calling BT module stream start");
        /* This call indicates BT IPC lib to start capture */
        ret =  audio_sink_start();
        QAL_ERR(LOG_TAG, "BT controller start capture return = %d",ret);
        if (ret != 0 ) {
           QAL_ERR(LOG_TAG, "BT controller start capture failed");
           a2dp_sink_started = false;
           return ret;
        }

        codec_info = audio_get_dec_config(&format);
        if (codec_info == NULL || format == CODEC_TYPE_INVALID) {
            QAL_ERR(LOG_TAG, "%s: invalid encoder config", __func__);
            return -EINVAL;
        }

        if (!isSinkReady()) {
            QAL_DBG(LOG_TAG, "Wait for capture ready not successful");
            ret = -ETIMEDOUT;
        }

        if (configureA2dpEncoderDecoder(format, codec_info, DEC) == true) {
            a2dp_sink_started = true;
            ret = 0;
            QAL_DBG(LOG_TAG, "Start capture successful to BT library");
        } else {
            QAL_DBG(LOG_TAG, "unable to configure DSP decoder");
            a2dp_sink_started = false;
            ret = -ETIMEDOUT;
        }

        if (!a2dp_send_sink_setup_complete()) {
           QAL_DBG(LOG_TAG, "sink_setup_complete not successful");
           ret = -ETIMEDOUT;
        }
    }

    if (a2dp_sink_started) {
        a2dp_sink_total_active_session_requests++;
    }
    QAL_DBG(LOG_TAG, "start A2DP sink total active sessions :%d",
          a2dp_sink_total_active_session_requests);
    return ret;
}

int BtA2dp::stopCapture()
{
    int ret =0;

    QAL_VERBOSE(LOG_TAG, "a2dp_stop_capture start");
    if (!(bt_lib_sink_handle && audio_sink_stop)) {
        QAL_ERR(LOG_TAG, "a2dp handle is not identified, Ignoring stop request");
        return -ENOSYS;
    }

    if (a2dp_sink_total_active_session_requests > 0)
        a2dp_sink_total_active_session_requests--;

    if (a2dp_sink_started && !a2dp_sink_total_active_session_requests) {
        QAL_VERBOSE(LOG_TAG, "calling BT module stream stop");
        is_configured = false;
        ret = audio_sink_stop();
        if (ret < 0) {
            QAL_ERR(LOG_TAG, "stop stream to BT IPC lib failed");
        } else {
            QAL_VERBOSE(LOG_TAG, "stop steam to BT IPC lib successful");
        }
    }
    QAL_DBG(LOG_TAG, "Stop A2DP capture, total active sessions :%d",
            a2dp_sink_total_active_session_requests);
    return 0;
}

bool BtA2dp::isSinkReady()
{
    bool ret = false;

    if ((bt_state_sink != A2DP_STATE_DISCONNECTED) &&
        (is_a2dp_offload_supported) &&
        (audio_sink_check_a2dp_ready))
           ret = audio_sink_check_a2dp_ready();
    return ret;
}

uint64_t BtA2dp::getDecLatency()
{
    uint32_t latency = 0;

    switch (bt_decoder_format) {
        case CODEC_TYPE_SBC:
            latency = 140;
            break;
        case CODEC_TYPE_AAC:
            latency = 180;
            break;
        default:
            latency = 200;
            QAL_DBG(LOG_TAG, "No valid decoder defined, setting latency to %dms", latency);
            break;
    }
    return (uint64_t)latency;
}

int32_t BtA2dp::setDeviceParameter(uint32_t param_id, void *param)
{
    int32_t status = 0, ret = 0;
    qal_param_bta2dp_t* param_a2dp = (qal_param_bta2dp_t *)param;

    if (is_a2dp_offload_supported == false) {
       QAL_VERBOSE(LOG_TAG, "no supported encoders identified,ignoring a2dp setparam");
       status = -EINVAL;
       goto exit;
    }

    switch(param_id) {
    case QAL_PARAM_ID_BT_A2DP_RECONFIG:
        if (bt_state_source == A2DP_STATE_STARTED) {
            param_bt_a2dp.reconfigured = param_a2dp->reconfigured;
            is_handoff_in_progress = param_a2dp->reconfigured;
        }
        break;
    case QAL_PARAM_ID_BT_A2DP_SUSPENDED:
    {
        if (bt_lib_source_handle == nullptr)
            goto exit;

        if (param_bt_a2dp.a2dp_suspended == param_a2dp->a2dp_suspended)
            goto exit;

        if (param_a2dp->a2dp_suspended == true) {
            param_bt_a2dp.a2dp_suspended = true;
            if (bt_state_source == A2DP_STATE_DISCONNECTED)
                goto exit;

            rm->a2dpSuspend();
            if (audio_source_suspend)
                audio_source_suspend();
        } else {
            if (clear_source_a2dpsuspend_flag)
                clear_source_a2dpsuspend_flag();

            param_bt_a2dp.a2dp_suspended = false;

            if (a2dp_source_total_active_session_requests > 0) {
                if (audio_source_start) {
                    ret = audio_source_start();
                    if (ret != 0) {
                        QAL_ERR(LOG_TAG, "BT controller start failed");
                        a2dp_source_started = false;
                    }
                }
            }
            rm->a2dpResume();
        }
        break;
    }
    default:
        return -EINVAL;
    }

exit:
    return status;
}

int32_t BtA2dp::getDeviceParameter(uint32_t param_id, void **param)
{
    switch(param_id) {
    case QAL_PARAM_ID_BT_A2DP_RECONFIG:
    case QAL_PARAM_ID_BT_A2DP_RECONFIG_SUPPORTED:
    case QAL_PARAM_ID_BT_A2DP_SUSPENDED:
        *param = &param_bt_a2dp;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

std::shared_ptr<Device>
BtA2dp::getInstance(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
{
    if (!obj) {
        std::shared_ptr<Device> sp(new BtA2dp(device, Rm));
        obj = sp;
    }
    return obj;
}

/* Scope of BtScoRX/Tx class */

std::shared_ptr<Device> BtSco::objRx = nullptr;
std::shared_ptr<Device> BtSco::objTx = nullptr;

BtSco::BtSco(struct qal_device *device, std::shared_ptr<ResourceManager> Rm)
    : Bluetooth(device, Rm)
{
    bt_sco_on = false;
    bt_wb_speech_enabled = false;
}

BtSco::~BtSco()
{
}

bool BtSco::isDeviceReady()
{
    return bt_sco_on;
}

int32_t BtSco::setDeviceParameter(uint32_t param_id, void *param)
{
    qal_param_btsco_t* param_bt_sco = (qal_param_btsco_t *)param;

    switch(param_id) {
    case QAL_PARAM_ID_BT_SCO:
        bt_sco_on = param_bt_sco->bt_sco_on;
        break;
    case QAL_PARAM_ID_BT_SCO_WB:
        bt_wb_speech_enabled = param_bt_sco->bt_wb_speech_enabled;
        deviceAttr.config.sample_rate = bt_wb_speech_enabled ? SAMPLINGRATE_16K : SAMPLINGRATE_8K;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

std::shared_ptr<Device> BtSco::getInstance(struct qal_device *device,
                                           std::shared_ptr<ResourceManager> Rm)
{
    if (device->id == QAL_DEVICE_OUT_BLUETOOTH_SCO) {
        if (!objRx) {
            std::shared_ptr<Device> sp(new BtSco(device, Rm));
            objRx = sp;
        }
        return objRx;
    } else {
        if (!objTx) {
            QAL_ERR( LOG_TAG, "rohit %s creating instance for  %d\n", __func__, device->id);
            std::shared_ptr<Device> sp(new BtSco(device, Rm));
            objTx = sp;
        }
        return objTx;
    }
}
/*BtSco class end */
