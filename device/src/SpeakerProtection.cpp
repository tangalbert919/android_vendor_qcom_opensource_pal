/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QAL: SpeakerProtection"


#include "SpeakerProtection.h"
#include "QalAudioRoute.h"
#include "ResourceManager.h"
#include "SessionAlsaUtils.h"
#include "kvh2xml.h"
#include "agm_api.h"

#include<fstream>
#include<sstream>

#ifndef QAL_SP_TEMP_PATH
#define QAL_SP_TEMP_PATH "/data/misc/audio/audio.cal"
#endif

#define MIN_SPKR_IDLE_SEC (120)
#define WAKEUP_MIN_IDLE_CHECK (1000 * 30)

#define SPKR_RIGHT_WSA_TEMP "SpkrRight WSA Temp"
#define SPKR_LEFT_WSA_TEMP "SpkrLeft WSA Temp"

#define TZ_TEMP_MIN_THRESHOLD    (5)
#define TZ_TEMP_MAX_THRESHOLD    (45)

/*Set safe temp value to 40C*/
#define SAFE_SPKR_TEMP 40
#define SAFE_SPKR_TEMP_Q6 (SAFE_SPKR_TEMP * (1 << 6))

#define MIN_RESISTANCE_SPKR_Q24 (2 * (1 << 24))

#define DEFAULT_PERIOD_SIZE 256
#define DEFAULT_PERIOD_COUNT 4

//TODO : remove this and add proper file
#define EVENT_ID_VI_CALIBRATION 0x0800119F

#define NORMAL_MODE 0
#define CALIBRATION_MODE 1
#define FACTORY_TEST_MODE 2
#define V_VALIDATION_MODE 3

#define CALIBRATION_STATUS_SUCCESS 4
#define CALIBRATION_STATUS_FAILURE 5

std::thread SpeakerProtection::mCalThread;
std::condition_variable SpeakerProtection::cv;
std::mutex SpeakerProtection::cvMutex;
std::mutex SpeakerProtection::calibrationMutex;

bool SpeakerProtection::isSpkrInUse;
struct timespec SpeakerProtection::spkrLastTimeUsed;
struct mixer *SpeakerProtection::mixer;
speaker_prot_cal_state SpeakerProtection::spkrCalState;
struct pcm * SpeakerProtection::rxPcm;
struct pcm * SpeakerProtection::txPcm;
struct param_id_sp_th_vi_calib_res_cfg_t * SpeakerProtection::callback_data;
int SpeakerProtection::numberOfChannels;
int SpeakerProtection::calibrationCallbackStatus;
int SpeakerProtection::numberOfRequest;
bool SpeakerProtection::mDspCallbackRcvd;

bool SpeakerProtection::isSpeakerInUse(unsigned long *sec)
{
    struct timespec temp;
    if (!sec) {
        QAL_ERR(LOG_TAG, "Improper argument time");
    }

    if (isSpkrInUse) {
        QAL_DBG(LOG_TAG, "Speaker in use");
        *sec = 0;
        return true;
    } else {
        QAL_DBG(LOG_TAG, "Speaker not in use");
        clock_gettime(CLOCK_BOOTTIME, &temp);
        *sec = temp.tv_sec - spkrLastTimeUsed.tv_sec;
    }
    return false;
}

void SpeakerProtection::spkrProtSetSpkrStatus(bool enable)
{
    if (enable)
        isSpkrInUse = true;
    else {
        isSpkrInUse = false;
        clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);
    }
}

// Callback from DSP for the R0 value
void SpeakerProtection::mixer_ctl_callback (void *hdl, uint32_t event_id,
                                            void *event_data, uint32_t event_size)
{
    param_id_sp_th_vi_calib_res_cfg_t *param_data = nullptr;
    hdl = NULL;

    QAL_DBG(LOG_TAG, "Got event from DSP %x", event_id);

    if (event_id == EVENT_ID_VI_CALIBRATION) {
        param_data = (param_id_sp_th_vi_calib_res_cfg_t *) event_data;
        QAL_DBG(LOG_TAG, "Calibration state %d", param_data->state);
        if (param_data->state == CALIBRATION_STATUS_SUCCESS) {
            // TODO : Add a lock
            QAL_DBG(LOG_TAG, "Calibration is successfull");
            callback_data = (param_id_sp_th_vi_calib_res_cfg_t *) calloc(1, event_size);
            callback_data->num_ch = param_data->num_ch;
            callback_data->state = param_data->state;
            for (int i = 0; i < callback_data->num_ch; i++) {
                callback_data->r0_cali_q24[i] = param_data->r0_cali_q24[i];
            }
            mDspCallbackRcvd = true;
            calibrationCallbackStatus = CALIBRATION_STATUS_SUCCESS;
            cv.notify_all();
        }
        else if (param_data->state == CALIBRATION_STATUS_FAILURE) {
            QAL_DBG(LOG_TAG, "Calibration is unsuccessfull");
            // Restart the calibration and abort current fetch
            mDspCallbackRcvd = true;
            calibrationCallbackStatus = CALIBRATION_STATUS_FAILURE;
            cv.notify_all();
        }
    }


}

void SpeakerProtection::spkrCalibrateWait()
{
    std::unique_lock<std::mutex> lock(cvMutex);
    cv.wait_for(lock,
        std::chrono::milliseconds(WAKEUP_MIN_IDLE_CHECK));
}

int SpeakerProtection::getSpeakerTemperature(int spkr_pos)
{
    struct mixer_ctl *ctl;
    const char *mixer_ctl_name;
    int status = 0;
    /**
     * It is assumed that for Mono speakers only right speaker will be there.
     * Thus we will get the Temperature just for right speaker.
     */
    QAL_DBG(LOG_TAG, "Enter Speaker Get Temperature %d", spkr_pos);

    switch(spkr_pos)
    {
        case WSA_SPKR_RIGHT: mixer_ctl_name = SPKR_RIGHT_WSA_TEMP; break;
        case WSA_SPKR_LEFT: mixer_ctl_name = SPKR_LEFT_WSA_TEMP; break;
    }

    QAL_DBG(LOG_TAG, "audio_mixer %pK", mixer);

    ctl = mixer_get_ctl_by_name(mixer, mixer_ctl_name);
    if (!ctl) {
        QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_ctl_name);
        status = -ENOENT;
        return status;
    }

    status = mixer_ctl_get_value(ctl, 0);

    QAL_DBG(LOG_TAG, "Exiting Speaker Get Temperature %d", status);

    return status;
}

int SpeakerProtection::spkrStartCalibration()
{
    int ret = 0, status = 0, dir;
    int i = 0;
    FILE *fp;
    struct qal_device device;
    struct qal_device deviceRx;
    struct qal_channel_info ch_info;
    struct qal_stream_attributes sAttr;
    std::string backEndNameTx;
    std::string backEndNameRx;
    std::vector <std::pair<int, int>> keyVector;
    std::vector <std::pair<int, int>> calVector;
    std::vector<int> pcmDevIdsRx;
    std::vector<int> pcmDevIdsTx;
    struct pcm_config config;
    std::shared_ptr<ResourceManager> rm;
    struct mixer_ctl *connectCtrl = NULL;
    std::ostringstream connectCtrlName;
    std::ostringstream connectCtrlNameRx;
    std::ostringstream connectCtrlNameBe;
    std::ostringstream connectCtrlNameBeVI;
    int flags;
    char mSndDeviceName_rx[128] = {0};
    struct audio_route *audioRoute = NULL;
    char mSndDeviceName_vi[128] = {0};
    struct agm_event_reg_cfg *event_cfg = NULL;
    int payload_size = 0;
    session_callback sessionCb;
    struct agmMetaData deviceMetaData(nullptr, 0);
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    param_id_sp_vi_op_mode_cfg_t modeConfg;
    param_id_sp_vi_channel_map_cfg_t viChannelMapConfg;
    param_id_sp_op_mode_t spModeConfg;
    param_id_sp_ex_vi_mode_cfg_t viExModeConfg;
    uint32_t miid = 0;

    std::unique_lock<std::mutex> calLock(calibrationMutex);

    memset(&device, 0, sizeof(device));
    memset(&deviceRx, 0, sizeof(deviceRx));
    memset(&sAttr, 0, sizeof(sAttr));
    memset(&config, 0, sizeof(config));
    memset(&modeConfg, 0, sizeof(modeConfg));
    memset(&viChannelMapConfg, 0, sizeof(viChannelMapConfg));
    memset(&spModeConfg, 0, sizeof(spModeConfg));
    memset(&viExModeConfg, 0, sizeof(viExModeConfg));

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    sessionCb = mixer_ctl_callback;

    // Configure device attribute
    if (numberOfChannels > 1) {
        ch_info.channels = CHANNELS_2;
        ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FL;
        ch_info.ch_map[1] = QAL_CHMAP_CHANNEL_FR;
    }
    else {
        ch_info.channels = CHANNELS_1;
        ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FR;
    }

    device.config.ch_info = ch_info;
    device.config.sample_rate = SAMPLINGRATE_48K;
    device.config.bit_width = BITWIDTH_32;
    device.config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

    // Setup TX path
    ret = rm->getAudioRoute(&audioRoute);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to get the audio_route name status %d", ret);
        goto done;
    }

    device.id = QAL_DEVICE_IN_VI_FEEDBACK;
    ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to obtain the tx snd device name");
        goto done;
    }

    QAL_DBG(LOG_TAG, "got the audio_route name %s", mSndDeviceName_vi);

    rm->getBackendName(device.id, backEndNameTx);

    keyVector.clear();
    calVector.clear();

    keyVector.push_back(std::make_pair(DEVICETX, VI_TX));

    // Enable the VI module
    if (numberOfChannels > 1)
        calVector.push_back(std::make_pair(SPK_PRO_VI_MAP, STEREO_SPKR));
    else
        calVector.push_back(std::make_pair(SPK_PRO_VI_MAP, RIGHT_SPKR));

    SessionAlsaUtils::getAgmMetaData(keyVector, calVector,
                    (struct prop_data *)devicePropId, deviceMetaData);
    if (!deviceMetaData.size) {
        QAL_ERR(LOG_TAG, "VI device metadata is zero");
        ret = -ENOMEM;
        goto free_fe;
    }
    connectCtrlNameBeVI<< backEndNameTx << " metadata";
    beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameBeVI.str().data());
    if (!beMetaDataMixerCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control for VI : %s", backEndNameTx.c_str());
        ret = -EINVAL;
        goto free_fe;
    }

    if (deviceMetaData.size) {
        ret = mixer_ctl_set_array(beMetaDataMixerCtrl, (void *)deviceMetaData.buf,
                    deviceMetaData.size);
        free(deviceMetaData.buf);
        deviceMetaData.buf = nullptr;
    }
    else {
        QAL_ERR(LOG_TAG, "Device Metadata not set for TX path");
        ret = -EINVAL;
        goto free_fe;
    }

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndNameTx, &device);
    if (ret) {
        QAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
        goto done;
    }

    sAttr.type = QAL_STREAM_LOW_LATENCY;
    sAttr.direction = QAL_AUDIO_INPUT_OUTPUT;
    dir = TXLOOPBACK;
    pcmDevIdsTx = rm->allocateFrontEndIds(sAttr, dir);
    if (pcmDevIdsTx.size() == 0) {
        QAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        ret = -ENOSYS;
        goto done;
    }
    connectCtrlName << "PCM" << pcmDevIdsTx.at(0) << " connect";
    connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlName.str().data());
    if (!connectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
        goto free_fe;
    }
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndNameTx.c_str());
    if (ret) {
        QAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
        connectCtrlName.str().data(), backEndNameTx.c_str(), ret);
        goto free_fe;
    }

    config.rate = SAMPLINGRATE_48K;
    config.format = PCM_FORMAT_S32_LE;
    if (numberOfChannels > 1)
        config.channels = CHANNELS_2;
    else
        config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    flags = PCM_IN;

    // Setting the mode of VI module
    modeConfg.num_speakers = numberOfChannels;
    modeConfg.th_operation_mode = CALIBRATION_MODE;
    if (minIdleTime > 0 && minIdleTime < MIN_SPKR_IDLE_SEC) {
        // Quick calibration set
        modeConfg.th_quick_calib_flag = 1;
    }
    else
        modeConfg.th_quick_calib_flag = 0;

    ret = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIdsTx.at(0),
                                                backEndNameTx.c_str(),
                                                MODULE_VI, &miid);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
    }

    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        delete payload;
        if (0 != ret) {
            QAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
        }
    }

    // Setting Channel Map configuration for VI module
    viChannelMapConfg.num_ch = numberOfChannels * 2;
    payloadSize = 0;

    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_VI_CHANNEL_MAP_CFG,(void *)&viChannelMapConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        delete payload;
        if (0 != ret) {
            QAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
        }
    }

    // Setting Excursion mode
    viExModeConfg.operation_mode = 0; // Normal Mode
    payloadSize = 0;

    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_EX_VI_MODE_CFG,(void *)&viExModeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        delete payload;
        if (0 != ret) {
            QAL_ERR(LOG_TAG," updateCustomPayload Failed for EX_VI_MODE_CFG\n");
        }
    }

    // Setting the values for VI module
    if (customPayloadSize) {
        ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndNameTx,
                    customPayload, customPayloadSize);
        if (ret) {
            QAL_ERR(LOG_TAG, "Unable to set custom param for mode");
            goto free_fe;
        }
    }

    txPcm = pcm_open(rm->getSndCard(), pcmDevIdsTx.at(0), flags, &config);
    if (!txPcm) {
        QAL_ERR(LOG_TAG, "txPcm open failed");
        goto free_fe;
    }

    if (!pcm_is_ready(txPcm)) {
        QAL_ERR(LOG_TAG, "txPcm open not ready");
        goto err_pcm_open;
    }

    //Register to Mixture control for VI module
    QAL_DBG(LOG_TAG, "registering event for VI module");
    payload_size = sizeof(struct agm_event_reg_cfg);

    event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
    if (!event_cfg) {
        QAL_ERR (LOG_TAG, "Unable to allocate memory for DSP event");
        ret = -ENOMEM;
        goto free_fe;
    }

    event_cfg->event_id = EVENT_ID_VI_CALIBRATION;
    event_cfg->event_config_payload_size = 0;
    event_cfg->is_register = 1;

    ret = SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIdsTx.at(0),
                      backEndNameTx.c_str(), MODULE_VI, (void *)event_cfg,
                      payload_size);
    if (ret) {
        QAL_ERR(LOG_TAG, "Unable to register event to DSP");
    }

    // Setup RX path
    deviceRx.id = QAL_DEVICE_OUT_SPEAKER;
    ret = rm->getSndDeviceName(deviceRx.id, mSndDeviceName_rx);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to obtain the rx snd device name");
        goto done;
    }

    if (numberOfChannels > 1) {
        deviceRx.config.ch_info.channels = CHANNELS_2;
        deviceRx.config.ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FL;
        deviceRx.config.ch_info.ch_map[1] = QAL_CHMAP_CHANNEL_FR;
    }
    else {
        deviceRx.config.ch_info.channels = CHANNELS_1;
        deviceRx.config.ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FR;
    }
    deviceRx.config.sample_rate = SAMPLINGRATE_48K;
    deviceRx.config.bit_width = BITWIDTH_16;
    deviceRx.config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

    rm->getBackendName(deviceRx.id, backEndNameRx);

    keyVector.clear();
    calVector.clear();
    keyVector.push_back(std::make_pair(DEVICERX, SPEAKER));

    // Enable the SP module
    if (numberOfChannels > 1)
        calVector.push_back(std::make_pair(SPK_PRO_DEV_MAP, LEFT_RIGHT));
    else
        calVector.push_back(std::make_pair(SPK_PRO_DEV_MAP, RIGHT_MONO));

    SessionAlsaUtils::getAgmMetaData(keyVector, calVector,
                    (struct prop_data *)devicePropId, deviceMetaData);
    if (!deviceMetaData.size) {
        QAL_ERR(LOG_TAG, "device metadata is zero");
        ret = -ENOMEM;
        goto free_fe;
    }

    connectCtrlNameBe<< backEndNameRx << " metadata";

    beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameBe.str().data());
    if (!beMetaDataMixerCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", backEndNameRx.c_str());
        ret = -EINVAL;
        goto free_fe;
    }

    if (deviceMetaData.size) {
        ret = mixer_ctl_set_array(beMetaDataMixerCtrl, (void *)deviceMetaData.buf,
                                    deviceMetaData.size);
        free(deviceMetaData.buf);
        deviceMetaData.buf = nullptr;
    }
    else {
        QAL_ERR(LOG_TAG, "Device Metadata not set for RX path");
        ret = -EINVAL;
        goto free_fe;
    }

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndNameRx, &deviceRx);
    if (ret) {
        QAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
        goto done;
    }

    /* Retrieve Hostless PCM device id */
    sAttr.type = QAL_STREAM_LOW_LATENCY;
    sAttr.direction = QAL_AUDIO_INPUT_OUTPUT;
    dir = RXLOOPBACK;
    pcmDevIdsRx = rm->allocateFrontEndIds(sAttr, dir);
    if (pcmDevIdsRx.size() == 0) {
        QAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        ret = -ENOSYS;
        goto done;
    }

    connectCtrlNameRx << "PCM" << pcmDevIdsRx.at(0) << " connect";
    connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameRx.str().data());
    if (!connectCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlNameRx.str().data());
        ret = -ENOSYS;
        goto free_fe;
    }
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndNameRx.c_str());
    if (ret) {
        QAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
        connectCtrlNameRx.str().data(), backEndNameRx.c_str(), ret);
        goto free_fe;
    }

    config.rate = SAMPLINGRATE_48K;
    config.format = PCM_FORMAT_S16_LE;
    if (numberOfChannels > 1)
        config.channels = CHANNELS_2;
    else
        config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    flags = PCM_OUT;

    // Set the operation mode for SP module
    spModeConfg.operation_mode = CALIBRATION_MODE;
    ret = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIdsRx.at(0),
                                                backEndNameRx.c_str(),
                                                MODULE_SP, &miid);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to get miid info %x, status = %d", MODULE_SP, ret);
    }

    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
    if (payloadSize) {
        if (customPayload) {
            free(customPayload);
            customPayloadSize = 0;
        }

        ret = updateCustomPayload(payload, payloadSize);
        delete payload;
        if (0 != ret) {
            QAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }

    // Setting the values for SP module
    if (customPayloadSize) {
        ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndNameRx,
                    customPayload, customPayloadSize);
        if (ret) {
            QAL_ERR(LOG_TAG, "Unable to set custom param for SP mode");
            free(customPayload);
            customPayloadSize = 0;
            goto free_fe;
        }
    }

    rxPcm = pcm_open(rm->getSndCard(), pcmDevIdsRx.at(0), flags, &config);
    if (!rxPcm) {
        QAL_ERR(LOG_TAG, "pcm open failed for RX path");
        ret = -ENOSYS;
        goto free_fe;
    }

    if (!pcm_is_ready(rxPcm)) {
        QAL_ERR(LOG_TAG, "pcm open not ready for RX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }


    // Register to mixtureControlEvents and wait for the R0T0 values

    ret = (ResourceManager::getInstance())->registerMixerEventCallback(pcmDevIdsTx,
                    sessionCb, (void*)this, true);
    if (ret != 0) {
        QAL_ERR(LOG_TAG, "Failed to register callback to rm");
    }

    enableDevice(audioRoute, mSndDeviceName_vi);
    QAL_DBG(LOG_TAG, "pcm start for TX path");
    if (pcm_start(txPcm) < 0) {
        QAL_ERR(LOG_TAG, "pcm start failed for TX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }

    enableDevice(audioRoute, mSndDeviceName_rx);
    QAL_DBG(LOG_TAG, "pcm start for RX path");
    if (pcm_start(rxPcm) < 0) {
        QAL_ERR(LOG_TAG, "pcm start failed for RX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }

    spkrCalState = SPKR_CALIB_IN_PROGRESS;

    QAL_DBG(LOG_TAG, "Waiting for the event from DSP or QAL");

    // TODO: Make this to wait in While loop
    cv.wait(calLock);

    // Store the R0T0 values
    if (mDspCallbackRcvd) {
        if (calibrationCallbackStatus == CALIBRATION_STATUS_SUCCESS) {
            QAL_DBG(LOG_TAG, "Calibration is done");
            fp = fopen(QAL_SP_TEMP_PATH, "wb");
            if (!fp) {
                QAL_ERR(LOG_TAG, "Unable to open file for write");
            } else {
                QAL_DBG(LOG_TAG, "Write the R0T0 value to file");
                for (i = 0; i < numberOfChannels; i++) {
                    fwrite(&callback_data->r0_cali_q24[i],
                                sizeof(callback_data->r0_cali_q24[i]), 1, fp);
                    fwrite(&spkerTempList[i], sizeof(spkerTempList[i]), 1, fp);
                }
                spkrCalState = SPKR_CALIBRATED;
                free(callback_data);
                fclose(fp);
            }
        }
        else if (calibrationCallbackStatus == CALIBRATION_STATUS_FAILURE) {
            QAL_DBG(LOG_TAG, "Calibration is not done");
            spkrCalState = SPKR_NOT_CALIBRATED;
            // reset the timer for retry
            clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);
            ret = -EINVAL;
        }
    }

err_pcm_open :
    if (txPcm) {
        if (event_cfg != NULL) {
            event_cfg->is_register = 0;

            status = SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIdsTx.at(0),
                        backEndNameTx.c_str(), MODULE_VI, (void *)event_cfg,
                        payload_size);
            if (status) {
                QAL_ERR(LOG_TAG, "Unable to deregister event to DSP");
            }
            free (event_cfg);
        }
        disableDevice(audioRoute, mSndDeviceName_vi);
        pcm_stop(txPcm);
        pcm_close(txPcm);
        txPcm = NULL;
    }
    if (rxPcm) {
        disableDevice(audioRoute, mSndDeviceName_rx);
        pcm_stop(rxPcm);
        pcm_close(rxPcm);
        rxPcm = NULL;
    }

free_fe:
    if (pcmDevIdsRx.size() != 0) {
        rm->freeFrontEndIds(pcmDevIdsRx, sAttr, RXLOOPBACK);
    }

    if (pcmDevIdsTx.size() != 0) {
        status = (ResourceManager::getInstance())->registerMixerEventCallback (
                    pcmDevIdsTx, sessionCb, (void*)this, false);
        if (status != 0) {
            QAL_ERR(LOG_TAG, "Failed to deregister callback to rm");
        }

        rm->freeFrontEndIds(pcmDevIdsTx, sAttr, TXLOOPBACK);
    }
    pcmDevIdsRx.clear();
    pcmDevIdsTx.clear();

done:

    if (!mDspCallbackRcvd) {
        // the lock is unlocked due to processing mode. It will be waiting
        // for the unlock. So notify it.
        cv.notify_all();
    }

    QAL_DBG(LOG_TAG, "Exiting %s %d", __func__, __LINE__);
    return ret;
}

/**
  * This function sets the temperature of each speakers.
  * Currently values are supported like:
  * spkerTempList[0] - Right Speaker Temperature
  * spkerTempList[1] - Left Speaker Temperature
  */
void SpeakerProtection::getSpeakerTemperatureList()
{
    int i = 0;
    int value;
    QAL_DBG(LOG_TAG, "Enter Speaker Get Temperature List");

    for(i = 0; i < numberOfChannels; i++) {
         value = getSpeakerTemperature(i);
         QAL_DBG(LOG_TAG, "Temperature %d ", value);
         spkerTempList[i] = value;
    }
    QAL_DBG(LOG_TAG, "Exit Speaker Get Temperature List");
}

void SpeakerProtection::spkrCalibrationThread()
{
    unsigned long sec = 0;
    bool proceed = false;
    std::string line;
    FILE *fp;
    int i;

    fp = fopen(QAL_SP_TEMP_PATH, "rb");
    if (fp) {
        QAL_DBG(LOG_TAG, "Cal File exists. Reading from it");
        struct vi_r0t0_cfg_t r0t0Array[numberOfChannels];
        for (i = 0; i < numberOfChannels; i++) {
            fread(&r0t0Array[i].r0_cali_q24, sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
            fread(&r0t0Array[i].t0_cali_q6, sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
            QAL_DBG(LOG_TAG, "R0 %d T0 %d ", r0t0Array[i].r0_cali_q24,
                    r0t0Array[i].t0_cali_q6);
        }
        spkrCalState = SPKR_CALIBRATED;
        threadExit = true;
    }

    while (!threadExit) {
        QAL_DBG(LOG_TAG, "Inside calibration while loop");
        proceed = false;
        if (isSpeakerInUse(&sec)) {
            QAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            QAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            QAL_DBG(LOG_TAG, "Speaker not in use, let's check for idle time");
            if (sec < minIdleTime) {
                QAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %d", sec);
                spkrCalibrateWait();
                QAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }

        if (proceed) {
            QAL_DBG(LOG_TAG, "Gettingtemperature of speakers");
            getSpeakerTemperatureList();

            for (i = 0; i < numberOfChannels; i++) {
                if ((spkerTempList[i] >= 0) &&
                    (spkerTempList[i] < TZ_TEMP_MIN_THRESHOLD ||
                     spkerTempList[i] > TZ_TEMP_MAX_THRESHOLD)) {
                    QAL_ERR(LOG_TAG, "Temperature out of range. Retry");
                    spkrCalibrateWait();
                    continue;
                }
            }
            for (i = 0; i < numberOfChannels; i++) {
                // Converting to Q6 format
                if (spkerTempList[i] >= 0) {
                    spkerTempList[i] = (spkerTempList[i]*(1<<6));
                }
            }
        }
        else {
            continue;
        }

        // Check whether speaker was in use in the meantime when temperature
        // was being read.
        proceed = false;
        if (isSpeakerInUse(&sec)) {
            QAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            QAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            QAL_DBG(LOG_TAG, "Speaker not in use, let's check for idle time");
            if (sec < minIdleTime) {
                QAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %d", sec);
                spkrCalibrateWait();
                QAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }

        if (proceed) {
            // Start calibrating the speakers.
            QAL_DBG(LOG_TAG, "Speaker not in use, start calibration");
            spkrStartCalibration();
            if (spkrCalState == SPKR_CALIBRATED) {
                threadExit = true;
            }
        }
        else {
            continue;
        }
    }
    QAL_DBG(LOG_TAG, "Calibration done, exiting the thread");
}

SpeakerProtection::SpeakerProtection(struct qal_device *device,
                                     std::shared_ptr<ResourceManager> Rm)
{
    int status = 0;
    struct qal_device_info devinfo = {};

    if (ResourceManager::spQuickCalTime > 0 &&
        ResourceManager::spQuickCalTime < MIN_SPKR_IDLE_SEC)
        minIdleTime = ResourceManager::spQuickCalTime;
    else
        minIdleTime = MIN_SPKR_IDLE_SEC;

    rm = Rm;

    memset(&mDeviceAttr, 0, sizeof(struct qal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct qal_device));

    threadExit = false;
    calThrdCreated = false;

    triggerCal = false;
    spkrCalState = SPKR_NOT_CALIBRATED;
    spkrProcessingState = SPKR_PROCESSING_IN_IDLE;

    isSpkrInUse = false;

    rm->getDeviceInfo(QAL_DEVICE_OUT_SPEAKER, QAL_STREAM_PROXY, &devinfo);
    numberOfChannels = devinfo.channels;
    QAL_DBG(LOG_TAG, "Number of Channels %d", numberOfChannels);

    spkerTempList = new int [numberOfChannels];
    // Get current time
    clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);

    // Getting mixture controls from Resource Manager
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error %d", status);
    }

    calibrationCallbackStatus = 0;
    mDspCallbackRcvd = false;

    mCalThread = std::thread(&SpeakerProtection::spkrCalibrationThread,
                        this);

    calThrdCreated = true;
}

SpeakerProtection::~SpeakerProtection()
{
}

/*
 * Function to trigger Processing mode.
 * The parameter that it accepts are below:
 * true - Start Processing Mode
 * false - Stop Processing Mode
 */
int32_t SpeakerProtection::spkrProtProcessingMode(std::shared_ptr<Device> devObj,
                                                  bool flag)
{
    int ret = 0, dir = TXLOOPBACK;
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct qal_device device;
    struct qal_channel_info ch_info;
    struct qal_stream_attributes sAttr;
    std::string backEndName;
    std::vector <std::pair<int, int>> keyVector;
    std::vector <std::pair<int, int>> calVector;
    std::shared_ptr<ResourceManager> rm;
    std::ostringstream connectCtrlNameBeVI;
    struct pcm_config config;
    struct mixer_ctl *connectCtrl = NULL;
    std::ostringstream connectCtrlName;
    struct audio_route *audioRoute = NULL;
    char mSndDeviceName_vi[128] = {0};
    int flags;
    struct vi_r0t0_cfg_t r0t0Array[numberOfChannels];
    param_id_sp_th_vi_r0t0_cfg_t *spR0T0confg;
    param_id_sp_vi_op_mode_cfg_t modeConfg;
    param_id_sp_vi_channel_map_cfg_t viChannelMapConfg;
    param_id_sp_op_mode_t spModeConfg;
    param_id_sp_ex_vi_mode_cfg_t viExModeConfg;
    uint32_t miid = 0;
    FILE *fp;
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    struct agmMetaData deviceMetaData(nullptr, 0);
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;
    std::shared_ptr<Device> dev = nullptr;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activeStreams;
    std::unique_lock<std::mutex> lock(calibrationMutex);

    QAL_DBG(LOG_TAG, "Processing mode flag %d", flag);
    deviceMutex.lock();

    if (flag) {
        if (spkrCalState == SPKR_CALIB_IN_PROGRESS) {
            // Close the Graphs
            cv.notify_all();
            // Wait for cleanup
            cv.wait(lock);
            spkrCalState = SPKR_NOT_CALIBRATED;
            txPcm = NULL;
            rxPcm = NULL;
            QAL_DBG(LOG_TAG, "Stopping the calibration mode");
        }
        numberOfRequest++;
        if (numberOfRequest > 1) {
            // R0T0 already set, we don't need to process the request.
            goto done;
        }

        if (customPayloadSize) {
            free(customPayload);
            customPayloadSize = 0;
        }
        spkrProtSetSpkrStatus(flag);
        // Speaker in use. Start the Processing Mode
        rm = ResourceManager::getInstance();

        memset(&device, 0, sizeof(device));
        memset(&sAttr, 0, sizeof(sAttr));
        memset(&config, 0, sizeof(config));
        memset(&modeConfg, 0, sizeof(modeConfg));
        memset(&viChannelMapConfg, 0, sizeof(viChannelMapConfg));
        memset(&viExModeConfg, 0, sizeof(viExModeConfg));
        memset(&spModeConfg, 0, sizeof(spModeConfg));

        keyVector.clear();
        calVector.clear();

        // Configure device attribute
       if (numberOfChannels > 1) {
            ch_info.channels = CHANNELS_2;
            ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FL;
            ch_info.ch_map[1] = QAL_CHMAP_CHANNEL_FR;
        }
        else {
            ch_info.channels = CHANNELS_1;
            ch_info.ch_map[0] = QAL_CHMAP_CHANNEL_FR;
        }

        device.config.ch_info = ch_info;
        device.config.sample_rate = SAMPLINGRATE_48K;
        device.config.bit_width = BITWIDTH_32;
        device.config.aud_fmt_id = QAL_AUDIO_FMT_DEFAULT_PCM;

        // Setup TX path
        device.id = QAL_DEVICE_IN_VI_FEEDBACK;

        ret = rm->getAudioRoute(&audioRoute);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
            goto done;
        }

        ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);

        QAL_DBG(LOG_TAG, "get the audio route %s", mSndDeviceName_vi);

        rm->getBackendName(device.id, backEndName);

        keyVector.push_back(std::make_pair(DEVICETX, VI_TX));

        // Enable the VI module
        if (numberOfChannels > 1)
            calVector.push_back(std::make_pair(SPK_PRO_VI_MAP, STEREO_SPKR));
        else
            calVector.push_back(std::make_pair(SPK_PRO_VI_MAP, RIGHT_SPKR));

        SessionAlsaUtils::getAgmMetaData(keyVector, calVector,
                (struct prop_data *)devicePropId, deviceMetaData);
        if (!deviceMetaData.size) {
            QAL_ERR(LOG_TAG, "VI device metadata is zero");
            ret = -ENOMEM;
            goto done;
        }
        connectCtrlNameBeVI<< backEndName << " metadata";
        beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer,
                                    connectCtrlNameBeVI.str().data());
        if (!beMetaDataMixerCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control for VI : %s", backEndName.c_str());
            ret = -EINVAL;
            goto done;
        }

        if (deviceMetaData.size) {
            ret = mixer_ctl_set_array(beMetaDataMixerCtrl, (void *)deviceMetaData.buf,
                        deviceMetaData.size);
            free(deviceMetaData.buf);
            deviceMetaData.buf = nullptr;
        }
        else {
            QAL_ERR(LOG_TAG, "Device Metadata not set for TX path");
            ret = -EINVAL;
            goto done;
        }

        ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &device);
        if (ret) {
            QAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
            goto done;
        }

        /* Retrieve Hostless PCM device id */
        sAttr.type = QAL_STREAM_LOW_LATENCY;
        sAttr.direction = QAL_AUDIO_INPUT_OUTPUT;
        dir = TXLOOPBACK;
        pcmDevIdTx = rm->allocateFrontEndIds(sAttr, dir);
        if (pcmDevIdTx.size() == 0) {
            QAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            ret = -ENOSYS;
            goto done;
        }
        connectCtrlName << "PCM" << pcmDevIdTx.at(0) << " connect";
        connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlName.str().data());
        if (!connectCtrl) {
            QAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
            goto free_fe;
        }
        ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
        if (ret) {
            QAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
            connectCtrlName.str().data(), backEndName.c_str(), ret);
            goto free_fe;
        }

        config.rate = SAMPLINGRATE_48K;
        config.format = PCM_FORMAT_S16_LE;
        if (numberOfChannels > 1)
            config.channels = CHANNELS_2;
        else
            config.channels = CHANNELS_1;
        config.period_size = DEFAULT_PERIOD_SIZE;
        config.period_count = DEFAULT_PERIOD_COUNT;
        config.start_threshold = 0;
        config.stop_threshold = INT_MAX;
        config.silence_threshold = 0;

        flags = PCM_IN;

        // Setting the mode of VI module
        modeConfg.num_speakers = numberOfChannels;
        switch (rm->mSpkrProtModeValue.operationMode) {
            case QAL_SP_MODE_FACTORY_TEST:
                modeConfg.th_operation_mode = FACTORY_TEST_MODE;
            break;
            case QAL_SP_MODE_V_VALIDATION:
                modeConfg.th_operation_mode = V_VALIDATION_MODE;
            break;
            default:
                QAL_ERR(LOG_TAG, "Normal mode being used");
                modeConfg.th_operation_mode = NORMAL_MODE;
        }
        modeConfg.th_quick_calib_flag = 0;

        ret = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIdTx.at(0),
                        backEndName.c_str(), MODULE_VI, &miid);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
        }
        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            delete payload;
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
            }
        }

        // Setting Channel Map configuration for VI module
        viChannelMapConfg.num_ch = numberOfChannels * 2;
        payloadSize = 0;

        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_VI_CHANNEL_MAP_CFG,(void *)&viChannelMapConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            delete payload;
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
            }
        }

        // Setting Excursion mode
        viExModeConfg.operation_mode = 0; // Normal Mode
        payloadSize = 0;

        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_EX_VI_MODE_CFG,(void *)&viExModeConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            delete payload;
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed for EX_VI_MODE_CFG\n");
            }
        }

        if (rm->mSpkrProtModeValue.operationMode) {
            QAL_DBG(LOG_TAG, "Operation mode %d",
                    rm->mSpkrProtModeValue.operationMode);
            param_id_sp_th_vi_ftm_cfg_t viFtmConfg;
            viFtmConfg.num_ch = numberOfChannels;
            payloadSize = 0;
            builder->payloadSPConfig (&payload, &payloadSize, miid,
                    PARAM_ID_SP_TH_VI_FTM_CFG | PARAM_ID_SP_TH_VI_V_VALI_CFG,
                    (void *) &viFtmConfg);
            if (payloadSize) {
                ret = updateCustomPayload(payload, payloadSize);
                delete payload;
                memset(&(rm->mSpkrProtModeValue), 0,
                        sizeof(qal_spkr_prot_payload));
                if (0 != ret) {
                    QAL_ERR(LOG_TAG," Payload Failed for FTM mode\n");
                }
            }
        }

        // Setting the R0T0 values
        if (spkrCalState == SPKR_CALIBRATED) {
            QAL_DBG(LOG_TAG, "Speaker calibrated. Read from the file");
            fp = fopen(QAL_SP_TEMP_PATH, "rb");
            if (fp) {
                for (int i = 0; i < numberOfChannels; i++) {
                    fread(&r0t0Array[i].r0_cali_q24,
                            sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
                    fread(&r0t0Array[i].t0_cali_q6,
                            sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
                }
            }
        }
        else {
            QAL_DBG(LOG_TAG, "Speaker not calibrated. Send safe value");
            for (int i = 0; i < numberOfChannels; i++) {
                r0t0Array[i].r0_cali_q24 = MIN_RESISTANCE_SPKR_Q24;
                r0t0Array[i].t0_cali_q6 = SAFE_SPKR_TEMP_Q6;
            }
        }
        spR0T0confg = (param_id_sp_th_vi_r0t0_cfg_t *)calloc(1,
                            sizeof(param_id_sp_th_vi_r0t0_cfg_t) +
                            sizeof(vi_r0t0_cfg_t) * numberOfChannels);
        spR0T0confg->num_speakers = numberOfChannels;

        memcpy(spR0T0confg->vi_r0t0_cfg, r0t0Array, sizeof(vi_r0t0_cfg_t) *
                numberOfChannels);

        payloadSize = 0;
        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_TH_VI_R0T0_CFG,(void *)spR0T0confg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            delete payload;
            free(spR0T0confg);
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            }
        }

        // Setting the values for VI module
        if (customPayloadSize) {
            ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndName,
                            customPayload, customPayloadSize);
            if (ret) {
                QAL_ERR(LOG_TAG, "Unable to set custom param for mode");
                goto free_fe;
            }
        }

        txPcm = pcm_open(rm->getSndCard(), pcmDevIdTx.at(0), flags, &config);
        if (!txPcm) {
            QAL_ERR(LOG_TAG, "txPcm open failed");
            goto free_fe;
        }

        if (!pcm_is_ready(txPcm)) {
            QAL_ERR(LOG_TAG, "txPcm open not ready");
            goto err_pcm_open;
        }

        // Setting up SP mode
        rm->getBackendName(mDeviceAttr.id, backEndName);
        dev = Device::getInstance(&mDeviceAttr, rm);
        ret = rm->getActiveStream_l(dev, activeStreams);
        if ((0 != ret) || (activeStreams.size() == 0)) {
            QAL_ERR(LOG_TAG, " no active stream available");
            return -EINVAL;
        }
        stream = static_cast<Stream *>(activeStreams[0]);
        stream->getAssociatedSession(&session);
        ret = session->getMIID(backEndName.c_str(), MODULE_SP, &miid);
        if (ret) {
            QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_SP, ret);
            return ret;
        }

        // Set the operation mode for SP module
        switch (rm->mSpkrProtModeValue.operationMode) {
            case QAL_SP_MODE_FACTORY_TEST:
                spModeConfg.operation_mode = FACTORY_TEST_MODE;
            break;
            case QAL_SP_MODE_V_VALIDATION:
                spModeConfg.operation_mode = V_VALIDATION_MODE;
            break;
            default:
                QAL_ERR(LOG_TAG, "Normal mode being used");
                spModeConfg.operation_mode = NORMAL_MODE;
        }

        payloadSize = 0;
        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
        if (payloadSize) {
            ret = devObj->updateCustomPayload(payload, payloadSize);
            delete payload;
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            }
        }

        enableDevice(audioRoute, mSndDeviceName_vi);
        QAL_DBG(LOG_TAG, "pcm start for TX");
        if (pcm_start(txPcm) < 0) {
            QAL_ERR(LOG_TAG, "pcm start failed for TX path");
            disableDevice(audioRoute, mSndDeviceName_vi);
            goto err_pcm_open;
        }

        // Free up the local variables
        goto done;
    }
    else {
        numberOfRequest--;
        if (numberOfRequest > 0) {
            // R0T0 already set, we don't need to process the request.
            goto done;
        }
        spkrProtSetSpkrStatus(flag);
        // Speaker not in use anymore. Stop the processing mode
        QAL_DBG(LOG_TAG, "Closing the VI path");
        if (txPcm) {
            rm = ResourceManager::getInstance();
            device.id = QAL_DEVICE_IN_VI_FEEDBACK;

            ret = rm->getAudioRoute(&audioRoute);
            if (0 != ret) {
                QAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
                goto done;
            }

            ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);
            disableDevice(audioRoute, mSndDeviceName_vi);
            pcm_stop(txPcm);
            pcm_close(txPcm);
            txPcm = NULL;
            sAttr.type = QAL_STREAM_LOW_LATENCY;
            sAttr.direction = QAL_AUDIO_INPUT_OUTPUT;
            goto free_fe;
        }
    }

err_pcm_open :
    if (txPcm) {
        pcm_close(txPcm);
        txPcm = NULL;
    }

free_fe:
    if (pcmDevIdTx.size() != 0) {
        rm->freeFrontEndIds(pcmDevIdTx, sAttr, dir);
        pcmDevIdTx.clear();
    }
done:
    deviceMutex.unlock();
    return ret;
}
