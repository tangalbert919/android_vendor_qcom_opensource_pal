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

#define SPKR_LEFT_WSA_TEMP "SpkrLeft WSA Temp"
#define SPKR_RIGHT_WSA_TEMP "SpkrRight WSA Temp"

#define TZ_TEMP_MIN_THRESHOLD    (5)
#define TZ_TEMP_MAX_THRESHOLD    (45)

/*Set safe temp value to 40C*/
#define SAFE_SPKR_TEMP 40
#define SAFE_SPKR_TEMP_Q6 (SAFE_SPKR_TEMP * (1 << 6))

#define MIN_RESISTANCE_SPKR_Q24 (2 * (1 << 24))

#define MAX_NUMBER_OF_SPEAKERS 2
#define DEFAULT_PERIOD_SIZE 256
#define DEFAULT_PERIOD_COUNT 4

//TODO : remove this and add proper file
#define EVENT_ID_VI_CALIBRATION 0x0800119F

std::thread SpeakerProtection::mCalThread;
std::condition_variable SpeakerProtection::cv;
std::mutex SpeakerProtection::cvMutex;

bool SpeakerProtection::isSpkrInUse;
struct timespec SpeakerProtection::spkrLastTimeUsed;
struct mixer *SpeakerProtection::mixer;
speaker_prot_cal_state SpeakerProtection::spkrCalState;
struct pcm * SpeakerProtection::rxPcm;
struct pcm * SpeakerProtection::txPcm;
struct param_id_sp_th_vi_calib_res_cfg_t * SpeakerProtection::callback_data;
int SpeakerProtection::totalSpeakers;
int SpeakerProtection::numberOfChannels;
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

// TODO: Move this fucntion to SessionAlsa
int SpeakerProtection::setConfig(int type, int tag, int value, int devId, const char *aif)
{
    int status = 0;
    uint32_t tagsent;
    struct agm_tag_config* tagConfig = NULL;
    const char *setParamTagControl = "setParamTag";
    const char *stream = "PCM";
    const char *setCalibrationControl = "setCalibration";
    struct mixer_ctl *ctl;
    struct agm_cal_config *calConfig = NULL;
    std::ostringstream tagCntrlName;
    std::ostringstream calCntrlName;
    std::ostringstream cntrlString;
    int tkv_size = 0;
    int ckv_size = 0;
    std::vector <std::pair<int, int>> ckv;
    std::vector <std::pair<int, int>> tkv;

    QAL_DBG(LOG_TAG, "Key %x Value %x Backend Name %s", tag, value, aif);

    switch(type) {
    case TKV:
        tkv.clear();
        tkv.push_back(std::make_pair(tag, value));
        tagsent = tag;
        QAL_DBG(LOG_TAG, "TAG %x\n", tagsent);

        tagConfig = (struct agm_tag_config*)calloc (1, sizeof(struct agm_tag_config) +
                                (tkv.size() * sizeof(agm_key_value)));
        if (!tagConfig) {
            status = -EINVAL;
            goto exit;
        }

        status = SessionAlsaUtils::getTagMetadata(tagsent, tkv, tagConfig);
        if (0 != status) {
            goto exit;
        }

        SessionAlsaUtils::setStreamMetadataType(mixer, devId, aif);

        tagCntrlName<<stream<<devId<<" "<<setParamTagControl;
        ctl = mixer_get_ctl_by_name(mixer, tagCntrlName.str().data());
        if (!ctl) {
            QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", tagCntrlName.str().data());
            status = -ENOENT;
            goto exit;
        }
        QAL_DBG(LOG_TAG, "mixer control: %s\n", tagCntrlName.str().data());

        tkv_size = tkv.size()*sizeof(struct agm_key_value);
        status = mixer_ctl_set_array(ctl, tagConfig,
                                sizeof(struct agm_tag_config) + tkv_size);
        if (status != 0) {
            QAL_ERR(LOG_TAG,"failed to set the tag %d tag %x", status, tag);
            goto exit;
        }
        ctl = NULL;
        tkv.clear();
    break;
    case CKV:
        ckv.clear();
        ckv.push_back(std::make_pair(tag, value));

        if (ckv.size() == 0) {
            goto exit;
        }
        calConfig = (struct agm_cal_config*)calloc (1, sizeof(struct agm_cal_config) +
                                (ckv.size() * sizeof(agm_key_value)));
        if (!calConfig) {
            status = -EINVAL;
            goto exit;
        }
        status = SessionAlsaUtils::getCalMetadata(ckv, calConfig);
        SessionAlsaUtils::setStreamMetadataType(mixer, devId, aif);
        calCntrlName<<stream<<devId<<" "<<setCalibrationControl;
        ctl = mixer_get_ctl_by_name(mixer, calCntrlName.str().data());
        if (!ctl) {
            QAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", calCntrlName.str().data());
            status = -ENOENT;
            goto exit;
        }
        ckv_size = ckv.size()*sizeof(struct agm_key_value);

        QAL_DBG(LOG_TAG, "Final Cal data Key %x Value %x", calConfig->kv[0].key, calConfig->kv[0].value);

        status = mixer_ctl_set_array(ctl, calConfig, sizeof(struct agm_cal_config) + ckv_size);
        if (status != 0) {
            QAL_ERR(LOG_TAG,"failed to set the tag %d tag %x", status, tag);
            goto exit;
        }
        ctl = NULL;
    break;
    default :
        QAL_ERR(LOG_TAG,"Wrong type value is used for setConfig");
    }

exit:
    if (tagConfig)
        free(tagConfig);
    if (calConfig)
        free(calConfig);

    tkv.clear();
    ckv.clear();

    QAL_DBG(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

// Callback from DSP for the R0 value
void SpeakerProtection::mixer_ctl_callback (void *hdl, uint32_t event_id,
                                            void *event_data, uint32_t event_size)
{
    param_id_sp_th_vi_calib_res_cfg_t *param_data = nullptr;
    hdl = NULL;

    QAL_DBG(LOG_TAG, "Got event from DSP %d", event_id);

    if (event_id == EVENT_ID_VI_CALIBRATION) {
        param_data = (param_id_sp_th_vi_calib_res_cfg_t *) event_data;
        if (param_data->state == 4) {
            // TODO : Add a lock
            QAL_DBG(LOG_TAG, "Calibration is successfull");
            callback_data = (param_id_sp_th_vi_calib_res_cfg_t *) calloc(1, event_size);
            callback_data->num_ch = param_data->num_ch;
            callback_data->state = param_data->state;
            memcpy(callback_data->r0_cali_q24, param_data->r0_cali_q24,
                    sizeof(int32_t)* param_data->num_ch);
            mDspCallbackRcvd = true;
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

    QAL_DBG(LOG_TAG, "Enter Speaker Get Temperature %d", spkr_pos);

    switch(spkr_pos)
    {
        case WSA_SPKR_LEFT: mixer_ctl_name = SPKR_LEFT_WSA_TEMP; break;
        case WSA_SPKR_RIGHT: mixer_ctl_name = SPKR_RIGHT_WSA_TEMP; break;
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
    int ret = 0, dir;
    struct qal_device device;
    struct qal_device deviceRx;
    struct qal_channel_info ch_info;
    struct qal_stream_attributes sAttr;
    std::string backEndName;
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
    int flags;
    char mSndDeviceName[128] = {0};
    struct audio_route *audioRoute_vi = NULL;
    char mSndDeviceName_vi[128] = {0};
    struct agm_event_reg_cfg *event_cfg;
    int payload_size = 0;
    session_callback sessionCb;
    struct agmMetaData deviceMetaData(nullptr, 0);
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;

    memset(&device, 0, sizeof(device));
    memset(&deviceRx, 0, sizeof(deviceRx));
    memset(&sAttr, 0, sizeof(sAttr));
    memset(&config, 0, sizeof(config));


    // Configure device attribute
    if (totalSpeakers > 1) {
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
    ret = rm->getAudioRoute(&audioRoute_vi);
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


    rm->getBackendName(device.id, backEndName);

    keyVector.push_back(std::make_pair(DEVICETX, VI_TX));

    ret = SessionAlsaUtils::setDeviceMetadata(rm, backEndName, keyVector);
    if (ret) {
        QAL_ERR(LOG_TAG, "setDeviceMetadata for feedback device failed");
        goto done;
    }

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &device);
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
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
    if (ret) {
        QAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
        connectCtrlName.str().data(), backEndName.c_str(), ret);
        goto free_fe;
    }

    config.rate = SAMPLINGRATE_48K;
    config.format = PCM_FORMAT_S16_LE;
    if (totalSpeakers > 1)
        config.channels = CHANNELS_2;
    else
        config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    flags = PCM_IN;

    txPcm = pcm_open(rm->getSndCard(), pcmDevIdsTx.at(0), flags, &config);
    if (!txPcm) {
        QAL_ERR(LOG_TAG, "txPcm open failed");
        goto free_fe;
    }

    if (!pcm_is_ready(txPcm)) {
        QAL_ERR(LOG_TAG, "txPcm open not ready");
        goto err_pcm_open;
    }

    //Setting the TKVs for VI module
    if (minIdleTime > 0 && minIdleTime < MIN_SPKR_IDLE_SEC)
    setConfig(TKV, TAG_MODULE_OP_MODE, QUICK_CAL, pcmDevIdsTx.at(0), backEndName.c_str());
    else
    setConfig(TKV, TAG_MODULE_OP_MODE, CALIB, pcmDevIdsTx.at(0), backEndName.c_str());

    //Register to Mixture control for VI module
    QAL_DBG(LOG_TAG, "registering event for VI module");
    payload_size = sizeof(struct agm_event_reg_cfg);

    event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
    if (!event_cfg) {
        ret = -ENOMEM;
        goto free_fe;
    }

    event_cfg->event_id = EVENT_ID_VI_CALIBRATION;
    event_cfg->event_config_payload_size = 0;
    event_cfg->is_register = 1;

    ret = SessionAlsaUtils::registerMixerEvent(mixer, pcmDevIdsTx.at(0),
                      backEndName.c_str(), MODULE_VI, (void *)event_cfg,
                      payload_size);
    if (ret) {
        QAL_ERR(LOG_TAG, "Unable to register event to DSP");
    }

    // Setup RX path
    deviceRx.id = QAL_DEVICE_OUT_SPEAKER;
    ret = rm->getSndDeviceName(deviceRx.id, mSndDeviceName);
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "Failed to obtain the rx snd device name");
        goto done;
    }

    if (totalSpeakers > 1) {
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

    rm->getBackendName(deviceRx.id, backEndName);

    keyVector.clear();
    calVector.clear();
    keyVector.push_back(std::make_pair(DEVICERX, SPEAKER));
    // Enable the SP module
    if (totalSpeakers > 1)
        calVector.push_back(std::make_pair(SPK_PRO_CH_MAP, LEFT_RIGHT));
    else
        calVector.push_back(std::make_pair(SPK_PRO_CH_MAP, RIGHT_MONO));

    SessionAlsaUtils::getAgmMetaData(keyVector, calVector,
                    (struct prop_data *)devicePropId, deviceMetaData);
    if (!deviceMetaData.size) {
        QAL_ERR(LOG_TAG, "device metadata is zero");
        ret = -ENOMEM;
        goto free_fe;
    }

    connectCtrlNameBe<< backEndName << " metadata";

    beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameBe.str().data());
    if (!beMetaDataMixerCtrl) {
        QAL_ERR(LOG_TAG, "invalid mixer control: %s", backEndName.c_str());
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

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &deviceRx);
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
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
    if (ret) {
        QAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
        connectCtrlNameRx.str().data(), backEndName.c_str(), ret);
        goto free_fe;
    }

    config.rate = SAMPLINGRATE_48K;
    config.format = PCM_FORMAT_S16_LE;
    if (totalSpeakers > 1)
        config.channels = CHANNELS_2;
    else
        config.channels = CHANNELS_1;
    config.period_size = DEFAULT_PERIOD_SIZE;
    config.period_count = DEFAULT_PERIOD_COUNT;
    config.start_threshold = 0;
    config.stop_threshold = INT_MAX;
    config.silence_threshold = 0;

    flags = PCM_OUT;

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


    setConfig(TKV, TAG_MODULE_OP_MODE, CALIB, pcmDevIdsRx.at(0), backEndName.c_str());

    // Register to mixtureControlEvents and wait for the R0T0 values
    sessionCb = mixer_ctl_callback;

    ret = (ResourceManager::getInstance())->registerMixerEventCallback(pcmDevIdsTx,
                    sessionCb, (void*)this, true);
    if (ret != 0) {
        QAL_ERR(LOG_TAG, "Failed to register callback to rm");
    }

    enableDevice(audioRoute_vi, mSndDeviceName_vi);
    QAL_DBG(LOG_TAG, "pcm start for TX path");
    if (pcm_start(txPcm) < 0) {
        QAL_ERR(LOG_TAG, "pcm start failed for TX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }
    disableDevice(audioRoute_vi, mSndDeviceName_vi);

    enableDevice(audioRoute_vi, mSndDeviceName);
    QAL_DBG(LOG_TAG, "pcm start for RX path");
    if (pcm_start(rxPcm) < 0) {
        QAL_ERR(LOG_TAG, "pcm start failed for RX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }
    disableDevice(audioRoute_vi, mSndDeviceName);

    goto free_fe;

err_pcm_open :
    if (txPcm) {
        pcm_close(txPcm);
        txPcm = NULL;
    }
    if (rxPcm) {
        pcm_close(rxPcm);
        rxPcm = NULL;
    }

free_fe:
    if (pcmDevIdsRx.size() != 0) {
        rm->freeFrontEndIds(pcmDevIdsRx, sAttr, RXLOOPBACK);
    }
    if (pcmDevIdsTx.size() != 0) {
        rm->freeFrontEndIds(pcmDevIdsTx, sAttr, TXLOOPBACK);
    }
    pcmDevIdsRx.clear();
    pcmDevIdsTx.clear();

done:
    QAL_DBG(LOG_TAG, "Exiting %s %d", __func__, __LINE__);
    return ret;
}

/**
  * This function sets the temperature of each speakers.
  * Currently values are supported like:
  * spkerTempList[0] - Left Speaker Temperature
  * spkerTempList[1] - Right Speaker Temperature
  */
void SpeakerProtection::getSpeakerTemperatureList()
{
    int i = 0;
    int value;
    QAL_DBG(LOG_TAG, "Enter Speaker Get Temperature List");

    for(i = 0; i < MAX_NUMBER_OF_SPEAKERS; i++) {
         value = getSpeakerTemperature(i);
         QAL_DBG(LOG_TAG, "Temperature %d ", value);
         spkerTempList[i] = value;
    }
    QAL_DBG(LOG_TAG, "Exit Speaker Get Temperature List");
}

void SpeakerProtection::spkrCalibrationThread()
{
    FILE *fp;
    unsigned long sec = 0;
    bool proceed = false;
    int i = 0;
    int j = 0;
    std::string line;

    fp = fopen(QAL_SP_TEMP_PATH, "rb");
    if (fp) {
        struct vi_r0t0_cfg_t r0t0Array[numberOfChannels];
        for (i = 0; i < numberOfChannels; i++) {
            fread(&r0t0Array[i].r0_cali_q24, sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
            fread(&r0t0Array[i].t0_cali_q6, sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
        }
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
            QAL_DBG(LOG_TAG, "Getting temperature of speakers");
            getSpeakerTemperatureList();

            for (i = 0; i < MAX_NUMBER_OF_SPEAKERS; i++) {
                if ((spkerTempList[i] >= 0) &&
                    (spkerTempList[i] < TZ_TEMP_MIN_THRESHOLD ||
                     spkerTempList[i] > TZ_TEMP_MAX_THRESHOLD)) {
                    QAL_ERR(LOG_TAG, "Temperature out of range. Retry");
                    spkrCalibrateWait();
                    continue;
                }
            }
            totalSpeakers = 0;
            for (i = 0; i < MAX_NUMBER_OF_SPEAKERS; i++) {
                // Converting to Q6 format
                if (spkerTempList[i] >= 0) {
                    spkerTempList[i] = (spkerTempList[i]*(1<<6));
                    totalSpeakers++;
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
            int startCalRet = spkrStartCalibration();
            if (!startCalRet) {
                spkrCalState = SPKR_CALIB_IN_PROGRESS;
                while(!mDspCallbackRcvd && !isSpkrInUse) {
                    QAL_DBG(LOG_TAG, "Waiting for the event from DSP or QAL");
                    spkrCalibrateWait();
                }
            }
            // Send the CKVs and store the R0T0 values
            if (mDspCallbackRcvd) {
                QAL_DBG(LOG_TAG, "Calibration is done");
                fp = fopen(QAL_SP_TEMP_PATH, "wb");
                if (!fp) {
                    QAL_ERR(LOG_TAG, "Unable to open file for write");
                } else {
                    QAL_DBG(LOG_TAG, "Write the R0T0 value to file");
                    for (i = 0; i < MAX_NUMBER_OF_SPEAKERS; i++) {
                        if (spkerTempList[i] < 0)
                            continue;

                        fwrite(&callback_data->r0_cali_q24[j],
                                sizeof(callback_data->r0_cali_q24[j]), 1, fp);
                        fwrite(&spkerTempList[i], sizeof(spkerTempList[i]), 1, fp);
                        j++;
                    }
                    free(callback_data);
                    fclose(fp);
                }
                threadExit = true;
            }
            else {
                QAL_DBG(LOG_TAG, "Calibration is not done. Speaker in use");
                continue;
            }
        }
        else {
            continue;
        }
    }
}

SpeakerProtection::SpeakerProtection(struct qal_device *device,
                                     std::shared_ptr<ResourceManager> Rm)
{
    int status = 0;
    struct qal_device_info devinfo = {};
    device = nullptr;


    if (ResourceManager::spQuickCalTime > 0 &&
        ResourceManager::spQuickCalTime < MIN_SPKR_IDLE_SEC)
        minIdleTime = ResourceManager::spQuickCalTime;
    else
        minIdleTime = MIN_SPKR_IDLE_SEC;

    rm = Rm;

    threadExit = false;
    calThrdCreated = false;

    triggerCal = false;
    spkrCalState = SPKR_NOT_CALIBRATED;
    spkrProcessingState = SPKR_PROCESSING_IN_IDLE;

    isSpkrInUse = false;

    rm->getDeviceInfo(QAL_DEVICE_OUT_SPEAKER, QAL_STREAM_PROXY, &devinfo);
    numberOfChannels = devinfo.channels;
    QAL_DBG(LOG_TAG, "Number of Channels %d", numberOfChannels);

    spkerTempList = new int [MAX_NUMBER_OF_SPEAKERS];
    // Get current time
    clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);

    // Getting mixture controls from Resource Manager
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"mixer error %d", status);
    }


    mCalThread = std::thread(&SpeakerProtection::spkrCalibrationThread,
                        this);

    calThrdCreated = true;
    mDspCallbackRcvd = false;
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
int32_t SpeakerProtection::spkrProtProcessingMode(bool flag)
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
    std::vector<int> pcmDevIds;
    std::shared_ptr<ResourceManager> rm;
    struct pcm_config config;
    struct mixer_ctl *connectCtrl = NULL;
    std::ostringstream connectCtrlName;
    struct audio_route *audioRoute_vi = NULL;
    char mSndDeviceName_vi[128] = {0};
    int flags;
    struct vi_r0t0_cfg_t r0t0Array[numberOfChannels];
    param_id_sp_th_vi_r0t0_cfg_t confg;
    uint32_t miid = 0;
    FILE *fp;

    QAL_DBG(LOG_TAG, "Enter %s Line %d Flag %d", __func__, __LINE__, flag);
    spkrProtSetSpkrStatus(flag);

    if (flag) {
        if (spkrCalState == SPKR_CALIB_IN_PROGRESS) {
            // Close the Graphs
            if (rxPcm)
                pcm_close(rxPcm);
            if (txPcm)
                pcm_close(txPcm);
            spkrCalState = SPKR_NOT_CALIBRATED;
            txPcm = NULL;
            rxPcm = NULL;
            QAL_DBG(LOG_TAG, "Stopping the calibration mode");
        }
        // Speaker in use. Start the Processing Mode
        rm = ResourceManager::getInstance();

        memset(&device, 0, sizeof(device));
        memset(&sAttr, 0, sizeof(sAttr));
        memset(&config, 0, sizeof(config));
        memset(&confg, 0, sizeof(confg));

        // Configure device attribute
       if (totalSpeakers > 1) {
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

        ret = rm->getAudioRoute(&audioRoute_vi);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
            goto done;
        }

        ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);
        enableDevice(audioRoute_vi, mSndDeviceName_vi);

        QAL_ERR(LOG_TAG, "get the audio_route address %s", mSndDeviceName_vi);

        rm->getBackendName(device.id, backEndName);

        keyVector.push_back(std::make_pair(DEVICETX, VI_TX));

        ret = SessionAlsaUtils::setDeviceMetadata(rm, backEndName, keyVector);
        if (ret) {
            QAL_ERR(LOG_TAG, "setDeviceMetadata for feedback device failed");
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
        pcmDevIds = rm->allocateFrontEndIds(sAttr, dir);
        if (pcmDevIds.size() == 0) {
            QAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            ret = -ENOSYS;
            goto done;
        }
        connectCtrlName << "PCM" << pcmDevIds.at(0) << " connect";
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
        if (totalSpeakers > 1)
            config.channels = CHANNELS_2;
        else
            config.channels = CHANNELS_1;
        config.period_size = DEFAULT_PERIOD_SIZE;
        config.period_count = DEFAULT_PERIOD_COUNT;
        config.start_threshold = 0;
        config.stop_threshold = INT_MAX;
        config.silence_threshold = 0;

        flags = PCM_IN;

        txPcm = pcm_open(rm->getSndCard(), pcmDevIds.at(0), flags, &config);
        if (!txPcm) {
            QAL_ERR(LOG_TAG, "txPcm open failed");
            goto free_fe;
        }

        if (!pcm_is_ready(txPcm)) {
            QAL_ERR(LOG_TAG, "txPcm open not ready");
            goto err_pcm_open;
        }
        setConfig(TKV, TAG_MODULE_OP_MODE, CALIB, pcmDevIds.at(0), backEndName.c_str());

        QAL_DBG(LOG_TAG, "pcm start for TX");
        if (pcm_start(txPcm) < 0) {
            QAL_ERR(LOG_TAG, "pcm start failed for TX path");
            goto err_pcm_open;
        }

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
        confg.num_speakers = numberOfChannels;

        for(int i = 0; i < numberOfChannels; i++) {
            confg.vi_r0t0_cfg[i].r0_cali_q24 = r0t0Array[i].r0_cali_q24;
            confg.vi_r0t0_cfg[i].t0_cali_q6 = r0t0Array[i].t0_cali_q6;
        }

        ret = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIds.at(0),
                                                    backEndName.c_str(),
                                                    MODULE_VI, &miid);
        if (0 != ret) {
            QAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
        }
        builder->payloadSPConfig(&payload, &payloadSize, miid, (void *)&confg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            delete payload;
            if (0 != ret) {
                QAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            }
        }

        // Free up the local variables
        goto free_fe;
    }
    else {
        // Speaker not in use anymore. Stop the processing mode
        QAL_DBG(LOG_TAG, "Closing the VI path");
        if (txPcm) {
            pcm_close(txPcm);
            txPcm = NULL;
            goto done;
        }

    }

err_pcm_open :
    if (txPcm) {
        pcm_close(txPcm);
        txPcm = NULL;
    }

free_fe:
    rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
    pcmDevIds.clear();
done:
    return ret;
}
