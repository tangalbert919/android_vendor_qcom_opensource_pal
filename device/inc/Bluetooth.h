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

#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "Device.h"
#include <system/audio-base.h>
#include <bt_intf.h>

#define DISALLOW_COPY_AND_ASSIGN(name) \
    name(const name &); \
    name &operator=(const name &)

enum A2DP_STATE {
    A2DP_STATE_CONNECTED,
    A2DP_STATE_STARTED,
    A2DP_STATE_STOPPED,
    A2DP_STATE_DISCONNECTED,
};

enum a2dp_role {
    SOURCE = 0,
    SINK,
};

typedef void (*bt_audio_pre_init_t)(void);
typedef int (*audio_source_open_t)(void);
typedef int (*audio_source_close_t)(void);
typedef int (*audio_source_start_t)(void);
typedef int (*audio_source_stop_t)(void);
typedef int (*audio_source_suspend_t)(void);
typedef void (*audio_source_handoff_triggered_t)(void);
typedef void (*clear_source_a2dpsuspend_flag_t)(void);
typedef void * (*audio_get_enc_config_t)(uint8_t *multicast_status,
                                        uint8_t *num_dev, codec_format_t *codec_format);
typedef int (*audio_source_check_a2dp_ready_t)(void);
typedef bool (*audio_is_tws_mono_mode_enable_t)(void);
typedef int (*audio_is_source_scrambling_enabled_t)(void);
typedef int (*audio_sink_start_t)(void);
typedef int (*audio_sink_stop_t)(void);
typedef void * (*audio_get_dec_config_t)(codec_format_t *codec_format);
typedef void * (*audio_sink_session_setup_complete_t)(uint64_t system_latency);
typedef int (*audio_sink_check_a2dp_ready_t)(void);
typedef uint16_t (*audio_sink_get_a2dp_latency_t)(void);

// Abstract base class
class Bluetooth : public Device
{
protected:
    static std::shared_ptr<Device> obj;
    Bluetooth(struct qal_device *device, std::shared_ptr<ResourceManager> Rm);

    struct qal_media_config encoderConfig;
    bool configureA2dpEncoderDecoder(codec_format_t codec_format, void *codec_info, codec_type type);
    void updateDeviceAttributes(codec_format_t codec_format, codec_type type);
    bool is_configured;
public:
    virtual ~Bluetooth();
};

class BtA2dp : public Bluetooth
{
protected:
    static std::shared_ptr<Device> obj;
    BtA2dp(struct qal_device *device, std::shared_ptr<ResourceManager> Rm);
    virtual int close_audio_source();
    qal_param_bta2dp_t param_bt_a2dp;
private:
    /* BT IPC related members */
    static void                                 *bt_lib_source_handle;
    static bt_audio_pre_init_t                  bt_audio_pre_init;
    static audio_source_open_t                  audio_source_open;
    static audio_source_close_t                 audio_source_close;
    static audio_source_start_t                 audio_source_start;
    static audio_source_stop_t                  audio_source_stop;
    static audio_source_suspend_t               audio_source_suspend;
    static audio_source_handoff_triggered_t     audio_source_handoff_triggered;
    static clear_source_a2dpsuspend_flag_t      clear_source_a2dpsuspend_flag;
    static audio_get_enc_config_t               audio_get_enc_config;
    static audio_source_check_a2dp_ready_t      audio_source_check_a2dp_ready;
    static audio_is_tws_mono_mode_enable_t      audio_is_tws_mono_mode_enable;
    static audio_is_source_scrambling_enabled_t audio_is_source_scrambling_enabled;
    static audio_sink_get_a2dp_latency_t        audio_sink_get_a2dp_latency;

    static void                                 *bt_lib_sink_handle;
    static audio_sink_start_t                   audio_sink_start;
    static audio_sink_stop_t                    audio_sink_stop;
    static audio_get_dec_config_t               audio_get_dec_config;
    static audio_sink_session_setup_complete_t  audio_sink_session_setup_complete;
    static audio_sink_check_a2dp_ready_t        audio_sink_check_a2dp_ready;

    /* member variables */
    uint8_t         a2dp_role;  // source or sink

    enum A2DP_STATE bt_state_source;
    bool            a2dp_source_started;
    int             a2dp_source_total_active_session_requests;
    bool            is_a2dp_offload_supported;
    bool            is_handoff_in_progress;
    enum A2DP_STATE bt_state_sink;
    codec_format_t    bt_decoder_format;
    bool            a2dp_sink_started;
    int             a2dp_sink_total_active_session_requests;
    int       startPlayback();
    int       stopPlayback();
    int       startCapture();
    int       stopCapture();
    uint32_t  getEncLatency();
    bool      isSinkReady();
    uint64_t  getDecLatency();

    /* common member funtions */
    void init_a2dp_source();
    void open_a2dp_source();
    void audio_a2dp_update_tws_channel_mode();

    void init_a2dp_sink();
    int  close_a2dp_input();
    bool a2dp_send_sink_setup_complete(void);
    void init();

public:
    int start();
    int stop();

    int getDeviceAttributes (struct qal_device *dattr) override;
    bool      isDeviceReady() override;
    int32_t setDeviceParameter(uint32_t param_id, void *param) override;
    int32_t getDeviceParameter(uint32_t param_id, void **param) override;

    static std::shared_ptr<Device> getInstance(struct qal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    virtual ~BtA2dp();
    DISALLOW_COPY_AND_ASSIGN(BtA2dp);
};

class BtSco : public Bluetooth
{
protected:
    static std::shared_ptr<Device> objRx;
    static std::shared_ptr<Device> objTx;
    BtSco(struct qal_device *device, std::shared_ptr<ResourceManager> Rm);
    bool bt_sco_on;
    bool bt_wb_speech_enabled;

public:
    static std::shared_ptr<Device> getInstance(struct qal_device *device,
                                               std::shared_ptr<ResourceManager> Rm);
    bool isDeviceReady() override;
    int32_t setDeviceParameter(uint32_t param_id, void *param) override;
    virtual ~BtSco();
    DISALLOW_COPY_AND_ASSIGN(BtSco);
};

#endif /* _BLUETOOTH_H_ */
