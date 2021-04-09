/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PAL: SpeakerProtection"


#include "SpeakerProtection.h"
#include "PalAudioRoute.h"
#include "ResourceManager.h"
#include "SessionAlsaUtils.h"
#include "kvh2xml.h"
#include <agm/agm_api.h>

#include<fstream>
#include<sstream>

#ifndef PAL_SP_TEMP_PATH
#define PAL_SP_TEMP_PATH "/data/misc/audio/audio.cal"
#endif

#define MIN_SPKR_IDLE_SEC (120)
#define WAKEUP_MIN_IDLE_CHECK (1000 * 30)

#define SPKR_RIGHT_WSA_TEMP "SpkrRight WSA Temp"
#define SPKR_LEFT_WSA_TEMP "SpkrLeft WSA Temp"

#define SPKR_RIGHT_WSA_DEV_NUM "SpkrRight WSA Get DevNum"
#define SPKR_LEFT_WSA_DEV_NUM "SpkrLeft WSA Get DevNum"

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
std::shared_ptr<Device> SpeakerFeedback::obj = nullptr;
int SpeakerFeedback::numSpeaker;

int32_t FourOhmTable_GaindB_q24[SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE] =
        {0x53e1f24, 0x8a6f830, 0xae85307, 0xc6c65a9, 0xd68c132, 0xdf7b2bf,
         0xe5de0fa, 0xec86429, 0xf67e253, 0x5be1ce3, 0x8c1ef2d, 0xab7a418,
         0xc2ee9bb, 0xd454e60, 0xddc3fe5, 0xe42de78, 0xeb0c869, 0xf57a079,
         0x5dc82c3, 0x8de921e, 0xaaca757, 0xc238e9b, 0xd3d1bd7, 0xdc676a7,
         0xe2ca47b, 0xe9d6fad, 0xf41b4e3, 0x615d39f, 0x8e173aa, 0xab1369d,
         0xc333572, 0xd41bd88, 0xde61431, 0xe4b020b, 0xec0d352, 0xf68031d,
         0x5fd76b2, 0x8cb5dc7, 0xaad6a38, 0xc159c21, 0xd368cab, 0xdd9dec8,
         0xe425e68, 0xeb634cd, 0xf602c14, 0x5e5b300, 0x8bf9050, 0xa95f594,
         0xbf0ad5f, 0xd25df4c, 0xdcc69f5, 0xe372339, 0xeacb2a8, 0xf539721,
         0x5d90ba3, 0x8b86beb, 0xa84ba58, 0xbd21ac6, 0xd19be21, 0xdbf8b1c,
         0xe2b738a, 0xea05a24, 0xf4c647e, 0x5cfc05c, 0x8b13bb3, 0xa713800,
         0xbaca286, 0xd06da85, 0xdaf4a5c, 0xe114414, 0xe938ec3, 0xf3d0ea6,
         0x5c2d809, 0x8737374, 0xa385c3c, 0xb8f36d1, 0xcbdd817, 0xd6c9a21,
         0xdda3c94, 0xe524419, 0xefda658, 0x57ff542, 0x869e158, 0xa1fffdb,
         0xb7c6c3c, 0xcb1798f, 0xd5a4232, 0xdcb7c7a, 0xe4433b0, 0xef275d1,
         0x57aeed8, 0x86184b8, 0xa11a5dc, 0xb42bae6, 0xc647a7f, 0xd12c06b,
         0xd7ed8c0, 0xdf86957, 0xe922953, 0x5735a40, 0x85a64c3, 0x9f711e4,
         0xae6e146, 0xbd9f8dd, 0xc670a00, 0xcda1ac9, 0xd3917d8, 0xdd8c537,
         0x567e253, 0x7da71c3, 0x9261265, 0xa282dde, 0xad9cbad, 0xb8da906,
         0xbd8cf60, 0xc5a6f83, 0xcd9a4b0, 0x40a9f41, 0x559b459, 0x6054609,
         0x68b2f54, 0x7a4ac6e, 0x8bdc850, 0x8f172fa, 0x971e02a, 0xa0b80c1};

int32_t SixOhmTable_GaindB_q24[SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE] =
        {0x55b0044, 0x8e5c26a, 0xb9a7df8, 0xd1c3e91, 0xe88e2e8, 0xf4e7dc4,
        0xfbf1eb0, 0x10397396, 0x10eb90f5, 0x5669a3b, 0x91902d0, 0xba8b233,
        0xd0cd18b, 0xe7ab548, 0xf4438f0, 0xfb8872c, 0x102a11dc, 0x10e29522,
        0x5e24330, 0x94c5bbd, 0xbb47886, 0xd03314b, 0xe6c9670, 0xf3b756b,
        0xfb15b81, 0x102683bd, 0x10dcd3df, 0x6e5fe77, 0x9ad2993, 0xc1997d4,
        0xd859656, 0xeda235a, 0xf819619, 0xfd89525, 0xfed0d70, 0x110fd7ab,
        0x6671fa0, 0x9a53a8d, 0xc09da21, 0xd93ba58, 0xecd2f94, 0xf77c7df,
        0xfe45367, 0x10594b0d, 0x1104800e, 0x6659865, 0x9a0e094, 0xbfdaffc,
        0xd7dda1a, 0xeb9951e, 0xf6637bb, 0xfd40f3f, 0x104b71f6, 0x10f7911a,
        0x6352598, 0x993b844, 0xbe8656e, 0xd6e8552, 0xeaae0a4, 0xf5878ab,
        0xfc79d18, 0x1041898d, 0x10ed9910, 0x619a761, 0x990c64e, 0xbdb6c48,
        0xd574654, 0xe971e2e, 0xf47fc49, 0xfb8872c, 0x10326d8e, 0x10e20926,
        0x5e08a42, 0x9842b83, 0xbcc17fc, 0xd40cb68, 0xe846a87, 0xf38a7d9,
        0xfa9c73a, 0x1025618b, 0x10d2c0eb, 0x5995c68, 0x974664b, 0xbc0915c,
        0xd2e595a, 0xe6ec0d5, 0xf264304, 0xf9b616d, 0x1015b794, 0x10c753de,
        0x58c6c17, 0x96a860e, 0xbbcff49, 0xcf9256a, 0xe2a16a2, 0xeeb3f79,
        0xf6a7160, 0xff7bd23, 0x10b9173f, 0x577c7df, 0x9529cfa, 0xba926ba,
        0xcb3beb2, 0xda6982e, 0xe59e2a1, 0xedb3a95, 0xf40a545, 0x100050a7,
        0x573fc79, 0x94088eb, 0xaf28fb9, 0xba5812a, 0xc8ae680, 0xd3da2d4,
        0xda4004d, 0xe23ef3f, 0xed66bea, 0x408ddf9, 0x6d585cf, 0x7d43aec,
        0x9542be2, 0xa54542c, 0xb063af4, 0xb06c1d0, 0xb5217bf, 0xbd2b075};

int32_t EightOhmTable_GaindB_q24[SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE] =
        {0x7b924c1, 0xb4f4ce3, 0xd6343aa, 0xef54dbe, 0x1016ebe5, 0x10bfed32,
        0x112aad7e, 0x118b21ba, 0x11a5703f, 0x76b5d32, 0xb20f473, 0xd55ba6b,
        0xee5492a, 0x100beca6, 0x10b1ed61, 0x111ea87f, 0x1184d9b4, 0x11a87e2b,
        0x739f15e, 0xad73bc4, 0xd4bc779, 0xed83eb4, 0xffc57ed, 0x10a953c2,
        0x111281df, 0x117cf2d0, 0x11aa524a, 0x907e90e, 0xbd7f00f, 0xdee343e,
        0xf0d2f17, 0x1028f097, 0x10c79b91, 0x11322ab6, 0x1193ddcd, 0x11a2aead,
        0x9025663, 0xbd2b075, 0xde18a01, 0xefca9b0, 0x101bb9d6, 0x10bc5953,
        0x11266661, 0x1190c138, 0x11a0fea1, 0x8fefaf1, 0xbccd416, 0xdcd573d,
        0xeed099b, 0x100c965a, 0x10b06738, 0x111b5f8a, 0x11878639, 0x11a012b1,
        0x8f83c3b, 0xbc5c124, 0xdbdaa6a, 0xedce634, 0xfff7a28, 0x10a32bbc,
        0x11106931, 0x11801db2, 0x119f4dee, 0x8ef2d87, 0xbbcd923, 0xdad6294,
        0xecac117, 0xfee95dd, 0x1097ecc2, 0x1102553a, 0x11782e25, 0x119d7524,
        0x8e60bc7, 0xbb47886, 0xd9aecde, 0xeb89bb2, 0xfded0a5, 0x108a97e5,
        0x10f74c6d, 0x1171ca65, 0x1198354a, 0x8dcd6ab, 0xbaca286, 0xd888c4f,
        0xea7614e, 0xfccb6cb, 0x107b4d45, 0x10eb11ad, 0x11684c61, 0x1193b607,
        0x8d4b810, 0xba07880, 0xd750f54, 0xe74965d, 0xf8da937, 0x104dca8d,
        0x10bfed32, 0x113fa7d3, 0x118f0a25, 0x8ca3112, 0xb8df8a1, 0xd0b7eea,
        0xe24f66b, 0xf248fe2, 0xfd76cb2, 0x104a3e80, 0x10bc899a, 0x1189b8b6,
        0x8c1ef2d, 0xb83fa3c, 0xc89899b, 0xd8a1fbd, 0xe70c133, 0xf0a66a7,
        0xf56962a, 0xfbbbd60, 0x1054a501, 0x8ad9f1c, 0xa3c9463, 0xad2d3d0,
        0xc058865, 0xd071339, 0xd394e0b, 0xd9ff484, 0xe04ff0d, 0xe519baf};

bool SpeakerProtection::isSpeakerInUse(unsigned long *sec)
{
    struct timespec temp;
    if (!sec) {
        PAL_ERR(LOG_TAG, "Improper argument time");
    }

    if (isSpkrInUse) {
        PAL_DBG(LOG_TAG, "Speaker in use");
        *sec = 0;
        return true;
    } else {
        PAL_DBG(LOG_TAG, "Speaker not in use");
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
void SpeakerProtection::mixer_ctl_callback (uint64_t hdl __unused, uint32_t event_id,
                                            void *event_data, uint32_t event_size)
{
    param_id_sp_th_vi_calib_res_cfg_t *param_data = nullptr;

    PAL_DBG(LOG_TAG, "Got event from DSP %x", event_id);

    if (event_id == EVENT_ID_VI_CALIBRATION) {
        param_data = (param_id_sp_th_vi_calib_res_cfg_t *) event_data;
        PAL_DBG(LOG_TAG, "Calibration state %d", param_data->state);
        if (param_data->state == CALIBRATION_STATUS_SUCCESS) {
            // TODO : Add a lock
            PAL_DBG(LOG_TAG, "Calibration is successfull");
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
            PAL_DBG(LOG_TAG, "Calibration is unsuccessfull");
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

int SpeakerProtection::getCpsDevNumber(std::string mixer_name)
{
    struct mixer_ctl *ctl;
    int status = 0;

    PAL_DBG(LOG_TAG, "Mixer control %s", mixer_name.c_str());
    PAL_DBG(LOG_TAG, "audio_mixer %pK", mixer);

    ctl = mixer_get_ctl_by_name(mixer, mixer_name.c_str());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_name.c_str());
        status = -ENOENT;
        return status;
    }

    status = mixer_ctl_get_value(ctl, 0);
    PAL_DBG(LOG_TAG, "Value for Mixer control %d", status);
    return status;
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
    PAL_DBG(LOG_TAG, "Enter Speaker Get Temperature %d", spkr_pos);

    switch(spkr_pos)
    {
        case WSA_SPKR_RIGHT:
            mixer_ctl_name = SPKR_RIGHT_WSA_TEMP;
        break;
        case WSA_SPKR_LEFT:
            mixer_ctl_name = SPKR_LEFT_WSA_TEMP;
        break;
    }

    PAL_DBG(LOG_TAG, "audio_mixer %pK", mixer);

    ctl = mixer_get_ctl_by_name(mixer, mixer_ctl_name);
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", mixer_ctl_name);
        status = -ENOENT;
        return status;
    }

    status = mixer_ctl_get_value(ctl, 0);

    PAL_DBG(LOG_TAG, "Exiting Speaker Get Temperature %d", status);

    return status;
}

int SpeakerProtection::spkrStartCalibration()
{
    int ret = 0, status = 0, dir;
    int i = 0;
    FILE *fp;
    struct pal_device device;
    struct pal_device deviceRx;
    struct pal_channel_info ch_info;
    struct pal_stream_attributes sAttr;
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

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    // Configure device attribute
    if (numberOfChannels > 1) {
        ch_info.channels = CHANNELS_2;
        ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
    }
    else {
        ch_info.channels = CHANNELS_1;
        ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FR;
    }

    device.config.ch_info = ch_info;
    device.config.sample_rate = SAMPLINGRATE_48K;
    device.config.bit_width = BITWIDTH_32;
    device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

    // Setup TX path
    ret = rm->getAudioRoute(&audioRoute);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed to get the audio_route name status %d", ret);
        goto done;
    }

    device.id = PAL_DEVICE_IN_VI_FEEDBACK;
    ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed to obtain the tx snd device name");
        goto done;
    }

    PAL_DBG(LOG_TAG, "got the audio_route name %s", mSndDeviceName_vi);

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
        PAL_ERR(LOG_TAG, "VI device metadata is zero");
        ret = -ENOMEM;
        goto free_fe;
    }
    connectCtrlNameBeVI<< backEndNameTx << " metadata";
    beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameBeVI.str().data());
    if (!beMetaDataMixerCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control for VI : %s", backEndNameTx.c_str());
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
        PAL_ERR(LOG_TAG, "Device Metadata not set for TX path");
        ret = -EINVAL;
        goto free_fe;
    }

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndNameTx, &device);
    if (ret) {
        PAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
        goto done;
    }

    sAttr.type = PAL_STREAM_LOW_LATENCY;
    sAttr.direction = PAL_AUDIO_INPUT_OUTPUT;
    dir = TX_HOSTLESS;
    pcmDevIdsTx = rm->allocateFrontEndIds(sAttr, dir);
    if (pcmDevIdsTx.size() == 0) {
        PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        ret = -ENOSYS;
        goto done;
    }
    connectCtrlName << "PCM" << pcmDevIdsTx.at(0) << " connect";
    connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlName.str().data());
    if (!connectCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
        goto free_fe;
    }
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndNameTx.c_str());
    if (ret) {
        PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
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
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
    }

    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        delete payload;
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
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
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
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
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for EX_VI_MODE_CFG\n");
        }
    }

    // Setting the values for VI module
    if (customPayloadSize) {
        ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndNameTx,
                    customPayload, customPayloadSize);
        if (ret) {
            PAL_ERR(LOG_TAG, "Unable to set custom param for mode");
            goto free_fe;
        }
    }

    txPcm = pcm_open(rm->getSndCard(), pcmDevIdsTx.at(0), flags, &config);
    if (!txPcm) {
        PAL_ERR(LOG_TAG, "txPcm open failed");
        goto free_fe;
    }

    if (!pcm_is_ready(txPcm)) {
        PAL_ERR(LOG_TAG, "txPcm open not ready");
        goto err_pcm_open;
    }

    //Register to Mixture control for VI module
    PAL_DBG(LOG_TAG, "registering event for VI module");
    payload_size = sizeof(struct agm_event_reg_cfg);

    event_cfg = (struct agm_event_reg_cfg *)calloc(1, payload_size);
    if (!event_cfg) {
        PAL_ERR (LOG_TAG, "Unable to allocate memory for DSP event");
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
        PAL_ERR(LOG_TAG, "Unable to register event to DSP");
    }

    // Register to mixtureControlEvents and wait for the R0T0 values

    ret = (ResourceManager::getInstance())->registerMixerEventCallback(pcmDevIdsTx,
                    sessionCb, (uint64_t)this, true);
    if (ret != 0) {
        PAL_ERR(LOG_TAG, "Failed to register callback to rm");
    }

    enableDevice(audioRoute, mSndDeviceName_vi);
    PAL_DBG(LOG_TAG, "pcm start for TX path");
    if (pcm_start(txPcm) < 0) {
        PAL_ERR(LOG_TAG, "pcm start failed for TX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }

    // Setup RX path
    deviceRx.id = PAL_DEVICE_OUT_SPEAKER;
    ret = rm->getSndDeviceName(deviceRx.id, mSndDeviceName_rx);
    if (0 != ret) {
        PAL_ERR(LOG_TAG, "Failed to obtain the rx snd device name");
        goto done;
    }

    if (numberOfChannels > 1) {
        deviceRx.config.ch_info.channels = CHANNELS_2;
        deviceRx.config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        deviceRx.config.ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
    }
    else {
        deviceRx.config.ch_info.channels = CHANNELS_1;
        deviceRx.config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FR;
    }
    deviceRx.config.sample_rate = SAMPLINGRATE_48K;
    deviceRx.config.bit_width = BITWIDTH_16;
    deviceRx.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

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
        PAL_ERR(LOG_TAG, "device metadata is zero");
        ret = -ENOMEM;
        goto free_fe;
    }

    connectCtrlNameBe<< backEndNameRx << " metadata";

    beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameBe.str().data());
    if (!beMetaDataMixerCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s", backEndNameRx.c_str());
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
        PAL_ERR(LOG_TAG, "Device Metadata not set for RX path");
        ret = -EINVAL;
        goto free_fe;
    }

    ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndNameRx, &deviceRx);
    if (ret) {
        PAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
        goto done;
    }

    /* Retrieve Hostless PCM device id */
    sAttr.type = PAL_STREAM_LOW_LATENCY;
    sAttr.direction = PAL_AUDIO_INPUT_OUTPUT;
    dir = RX_HOSTLESS;
    pcmDevIdsRx = rm->allocateFrontEndIds(sAttr, dir);
    if (pcmDevIdsRx.size() == 0) {
        PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
        ret = -ENOSYS;
        goto done;
    }

    connectCtrlNameRx << "PCM" << pcmDevIdsRx.at(0) << " connect";
    connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlNameRx.str().data());
    if (!connectCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlNameRx.str().data());
        ret = -ENOSYS;
        goto free_fe;
    }
    ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndNameRx.c_str());
    if (ret) {
        PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
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
        PAL_ERR(LOG_TAG, "Failed to get miid info %x, status = %d", MODULE_SP, ret);
    }

    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
    if (payloadSize) {
        if (customPayloadSize) {
            free(customPayload);
            customPayloadSize = 0;
        }

        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }

    // Setting the values for SP module
    if (customPayloadSize) {
        ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndNameRx,
                    customPayload, customPayloadSize);
        if (ret) {
            PAL_ERR(LOG_TAG, "Unable to set custom param for SP mode");
            free(customPayload);
            customPayloadSize = 0;
            goto free_fe;
        }
    }

    rxPcm = pcm_open(rm->getSndCard(), pcmDevIdsRx.at(0), flags, &config);
    if (!rxPcm) {
        PAL_ERR(LOG_TAG, "pcm open failed for RX path");
        ret = -ENOSYS;
        goto free_fe;
    }

    if (!pcm_is_ready(rxPcm)) {
        PAL_ERR(LOG_TAG, "pcm open not ready for RX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }


    enableDevice(audioRoute, mSndDeviceName_rx);
    PAL_DBG(LOG_TAG, "pcm start for RX path");
    if (pcm_start(rxPcm) < 0) {
        PAL_ERR(LOG_TAG, "pcm start failed for RX path");
        ret = -ENOSYS;
        goto err_pcm_open;
    }

    spkrCalState = SPKR_CALIB_IN_PROGRESS;

    PAL_DBG(LOG_TAG, "Waiting for the event from DSP or PAL");

    // TODO: Make this to wait in While loop
    cv.wait(calLock);

    // Store the R0T0 values
    if (mDspCallbackRcvd) {
        if (calibrationCallbackStatus == CALIBRATION_STATUS_SUCCESS) {
            PAL_DBG(LOG_TAG, "Calibration is done");
            fp = fopen(PAL_SP_TEMP_PATH, "wb");
            if (!fp) {
                PAL_ERR(LOG_TAG, "Unable to open file for write");
            } else {
                PAL_DBG(LOG_TAG, "Write the R0T0 value to file");
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
            PAL_DBG(LOG_TAG, "Calibration is not done");
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
                PAL_ERR(LOG_TAG, "Unable to deregister event to DSP");
            }
            free (event_cfg);
        }
        status = (ResourceManager::getInstance())->registerMixerEventCallback (
                    pcmDevIdsTx, sessionCb, (uint64_t)this, false);
        if (status != 0) {
            PAL_ERR(LOG_TAG, "Failed to deregister callback to rm");
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
        rm->freeFrontEndIds(pcmDevIdsRx, sAttr, RX_HOSTLESS);
    }

    if (pcmDevIdsTx.size() != 0) {
        rm->freeFrontEndIds(pcmDevIdsTx, sAttr, TX_HOSTLESS);
    }
    pcmDevIdsRx.clear();
    pcmDevIdsTx.clear();

done:

    if (!mDspCallbackRcvd) {
        // the lock is unlocked due to processing mode. It will be waiting
        // for the unlock. So notify it.
        cv.notify_all();
    }

    if(builder) {
       delete builder;
       builder = NULL;
    }
    PAL_DBG(LOG_TAG, "Exiting");
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
    PAL_DBG(LOG_TAG, "Enter Speaker Get Temperature List");

    for(i = 0; i < numberOfChannels; i++) {
         value = getSpeakerTemperature(i);
         PAL_DBG(LOG_TAG, "Temperature %d ", value);
         spkerTempList[i] = value;
    }
    PAL_DBG(LOG_TAG, "Exit Speaker Get Temperature List");
}

void SpeakerProtection::spkrCalibrationThread()
{
    unsigned long sec = 0;
    bool proceed = false;
    std::string line;
    int i;

    while (!threadExit) {
        PAL_DBG(LOG_TAG, "Inside calibration while loop");
        proceed = false;
        if (isSpeakerInUse(&sec)) {
            PAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            PAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            PAL_DBG(LOG_TAG, "Speaker not in use, let's check for idle time");
            if (sec < minIdleTime) {
                PAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %lu", sec);
                spkrCalibrateWait();
                PAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }

        if (proceed) {
            PAL_DBG(LOG_TAG, "Gettingtemperature of speakers");
            getSpeakerTemperatureList();

            for (i = 0; i < numberOfChannels; i++) {
                if ((spkerTempList[i] >= 0) &&
                    (spkerTempList[i] < TZ_TEMP_MIN_THRESHOLD ||
                     spkerTempList[i] > TZ_TEMP_MAX_THRESHOLD)) {
                    PAL_ERR(LOG_TAG, "Temperature out of range. Retry");
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
            PAL_DBG(LOG_TAG, "Speaker in use. Wait for proper time");
            spkrCalibrateWait();
            PAL_DBG(LOG_TAG, "Waiting done");
            continue;
        }
        else {
            PAL_DBG(LOG_TAG, "Speaker not in use, let's check for idle time");
            if (sec < minIdleTime) {
                PAL_DBG(LOG_TAG, "Speaker not idle for minimum time. %lu", sec);
                spkrCalibrateWait();
                PAL_DBG(LOG_TAG, "Waited for speaker to be idle for min time");
                continue;
            }
            proceed = true;
        }

        if (proceed) {
            // Start calibrating the speakers.
            PAL_DBG(LOG_TAG, "Speaker not in use, start calibration");
            spkrStartCalibration();
            if (spkrCalState == SPKR_CALIBRATED) {
                threadExit = true;
            }
        }
        else {
            continue;
        }
    }
    PAL_DBG(LOG_TAG, "Calibration done, exiting the thread");
}

SpeakerProtection::SpeakerProtection(struct pal_device *device,
                        std::shared_ptr<ResourceManager> Rm):Speaker(device, Rm)
{
    int status = 0;
    struct pal_device_info devinfo = {};
    FILE *fp;

    if (ResourceManager::spQuickCalTime > 0 &&
        ResourceManager::spQuickCalTime < MIN_SPKR_IDLE_SEC)
        minIdleTime = ResourceManager::spQuickCalTime;
    else
        minIdleTime = MIN_SPKR_IDLE_SEC;

    rm = Rm;

    memset(&mDeviceAttr, 0, sizeof(struct pal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct pal_device));

    threadExit = false;
    calThrdCreated = false;

    triggerCal = false;
    spkrCalState = SPKR_NOT_CALIBRATED;
    spkrProcessingState = SPKR_PROCESSING_IN_IDLE;

    isSpkrInUse = false;

    rm->getDeviceInfo(PAL_DEVICE_OUT_SPEAKER, PAL_STREAM_PROXY, &devinfo);
    numberOfChannels = devinfo.channels;
    PAL_DBG(LOG_TAG, "Number of Channels %d", numberOfChannels);

    spkerTempList = new int [numberOfChannels];
    // Get current time
    clock_gettime(CLOCK_BOOTTIME, &spkrLastTimeUsed);

    // Getting mixture controls from Resource Manager
    status = rm->getAudioMixer(&mixer);
    if (status) {
        PAL_ERR(LOG_TAG,"mixer error %d", status);
    }

    calibrationCallbackStatus = 0;
    mDspCallbackRcvd = false;

    fp = fopen(PAL_SP_TEMP_PATH, "rb");
    if (fp) {
        PAL_DBG(LOG_TAG, "Cal File exists. Reading from it");
        spkrCalState = SPKR_CALIBRATED;
    }
    else {
        PAL_DBG(LOG_TAG, "Calibration Not done");
        mCalThread = std::thread(&SpeakerProtection::spkrCalibrationThread,
                            this);
    }
    calThrdCreated = true;
}

SpeakerProtection::~SpeakerProtection()
{
}

/*
 * CPS related custom payload
 */

void SpeakerProtection::updateCpsCustomPayload(int miid)
{
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    lpass_swr_hw_reg_cfg_t *cpsRegCfg = NULL;
    param_id_sp_cps_static_cfg_t cpsStaticConf;
    pkd_reg_addr_t pkedRegAddr[numberOfChannels];
    int dev_num;
    int val, ret = 0;

    memset(&cpsStaticConf, 0, sizeof(param_id_sp_cps_static_cfg_t));

    // Payload for ParamID : PARAM_ID_CPS_LPASS_HW_INTF_CFG
    cpsRegCfg = (lpass_swr_hw_reg_cfg_t *) calloc(1, sizeof(lpass_swr_hw_reg_cfg_t)
                       + sizeof(pkd_reg_addr_t) * numberOfChannels);
    if (cpsRegCfg == NULL) {
        PAL_ERR(LOG_TAG,"Unable to allocate Memory for CPS config\n");
        goto exit;
    }
    cpsRegCfg->num_spkr = numberOfChannels;
    cpsRegCfg->lpass_wr_cmd_reg_phy_addr = LPASS_WR_CMD_REG_PHY_ADDR;
    cpsRegCfg->lpass_rd_cmd_reg_phy_addr = LPASS_RD_CMD_REG_PHY_ADDR;
    cpsRegCfg->lpass_rd_fifo_reg_phy_addr = LPASS_RD_FIFO_REG_PHY_ADDR;

    for (int i = 0; i < numberOfChannels; i++) {
        switch (i)
        {
            case 0 :
                dev_num = getCpsDevNumber(SPKR_RIGHT_WSA_DEV_NUM);
            break;
            case 1 :
                dev_num = getCpsDevNumber(SPKR_LEFT_WSA_DEV_NUM);
            break;
        }
        pkedRegAddr[i].vbatt_pkd_reg_addr = CPS_WSA_VBATT_REG_ADDR;
        pkedRegAddr[i].temp_pkd_reg_addr = CPS_WSA_TEMP_REG_ADDR;

        pkedRegAddr[i].vbatt_pkd_reg_addr &= 0xFFFF;
        pkedRegAddr[i].temp_pkd_reg_addr &= 0xFFFF;

        val = 0;

        /* bits 20:23 carry swr device number */
        val |= dev_num << 20;
        /* bits 24:27 carry read length in bytes */
        val |= 1 << 24;

        pkedRegAddr[i].vbatt_pkd_reg_addr |= val;
        pkedRegAddr[i].temp_pkd_reg_addr |= val;
    }
    memcpy(cpsRegCfg->pkd_reg_addr, pkedRegAddr, sizeof(pkd_reg_addr_t) *
                    numberOfChannels);

    // Payload builder for ParamID : PARAM_ID_CPS_LPASS_HW_INTF_CFG
    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_CPS_LPASS_HW_INTF_CFG,(void *)cpsRegCfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        free(cpsRegCfg);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }

    // Payload for ParamID : PARAM_ID_SP_CPS_STATIC_CFG
    cpsStaticConf.limiter_cps_en_flag = 1;
    cpsStaticConf.limiter_cps_smooth_VbDT_en_flag = 1;
    cpsStaticConf.limiter_cps_margin_dB_q15 = 0xccd;

    memcpy(cpsStaticConf.FourOhmTable_GaindB_q24, FourOhmTable_GaindB_q24,
            sizeof(int32_t)*SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE);

    memcpy(cpsStaticConf.SixOhmTable_GaindB_q24, SixOhmTable_GaindB_q24,
            sizeof(int32_t)*SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE);

    memcpy(cpsStaticConf.EightOhmTable_GaindB_q24, EightOhmTable_GaindB_q24,
            sizeof(int32_t)*SP_NDTEMP_DISCRETE * SP_NVBATT_DISCRETE);

    // Payload builder for ParamID : PARAM_ID_SP_CPS_STATIC_CFG
    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
            PARAM_ID_SP_CPS_STATIC_CFG,(void *)&cpsStaticConf);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }
exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
}

/*
 * Function to trigger Processing mode.
 * The parameter that it accepts are below:
 * true - Start Processing Mode
 * false - Stop Processing Mode
 */
int32_t SpeakerProtection::spkrProtProcessingMode(bool flag)
{
    int ret = 0, dir = TX_HOSTLESS;
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct pal_device device;
    struct pal_channel_info ch_info;
    struct pal_stream_attributes sAttr;
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
    int flags, viParamId = 0;
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

    PAL_DBG(LOG_TAG, "Processing mode flag %d", flag);
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
            PAL_DBG(LOG_TAG, "Stopping the calibration mode");
        }
        numberOfRequest++;
        if (numberOfRequest > 1) {
            // R0T0 already set, we don't need to process the request.
            goto done;
        }
        PAL_DBG(LOG_TAG, "Custom payload size %zu, Payload %p", customPayloadSize,
                customPayload);

        if (customPayload) {
            free(customPayload);
        }
        customPayloadSize = 0;
        customPayload = NULL;

        spkrProtSetSpkrStatus(flag);
        // Speaker in use. Start the Processing Mode
        rm = ResourceManager::getInstance();
        if (!rm) {
            PAL_ERR(LOG_TAG, "Failed to get resource manager instance");
            goto done;
        }
        memset(&device, 0, sizeof(device));
        memset(&sAttr, 0, sizeof(sAttr));
        memset(&config, 0, sizeof(config));
        memset(&modeConfg, 0, sizeof(modeConfg));
        memset(&viChannelMapConfg, 0, sizeof(viChannelMapConfg));
        memset(&viExModeConfg, 0, sizeof(viExModeConfg));
        memset(&spModeConfg, 0, sizeof(spModeConfg));

        keyVector.clear();
        calVector.clear();

        if (customPayload) {
            free(customPayload);
            customPayloadSize = 0;
            customPayload = NULL;
        }

        // Configure device attribute
       if (numberOfChannels > 1) {
            ch_info.channels = CHANNELS_2;
            ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
            ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        }
        else {
            ch_info.channels = CHANNELS_1;
            ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FR;
        }

        device.config.ch_info = ch_info;
        device.config.sample_rate = SAMPLINGRATE_48K;
        device.config.bit_width = BITWIDTH_32;
        device.config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM;

        // Setup TX path
        device.id = PAL_DEVICE_IN_VI_FEEDBACK;

        ret = rm->getAudioRoute(&audioRoute);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
            goto done;
        }

        ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);

        PAL_DBG(LOG_TAG, "get the audio route %s", mSndDeviceName_vi);

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
            PAL_ERR(LOG_TAG, "VI device metadata is zero");
            ret = -ENOMEM;
            goto done;
        }
        connectCtrlNameBeVI<< backEndName << " metadata";
        beMetaDataMixerCtrl = mixer_get_ctl_by_name(mixer,
                                    connectCtrlNameBeVI.str().data());
        if (!beMetaDataMixerCtrl) {
            PAL_ERR(LOG_TAG, "invalid mixer control for VI : %s", backEndName.c_str());
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
            PAL_ERR(LOG_TAG, "Device Metadata not set for TX path");
            ret = -EINVAL;
            goto done;
        }

        ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &device);
        if (ret) {
            PAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
            goto done;
        }

        /* Retrieve Hostless PCM device id */
        sAttr.type = PAL_STREAM_LOW_LATENCY;
        sAttr.direction = PAL_AUDIO_INPUT_OUTPUT;
        dir = TX_HOSTLESS;
        pcmDevIdTx = rm->allocateFrontEndIds(sAttr, dir);
        if (pcmDevIdTx.size() == 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            ret = -ENOSYS;
            goto done;
        }
        connectCtrlName << "PCM" << pcmDevIdTx.at(0) << " connect";
        connectCtrl = mixer_get_ctl_by_name(mixer, connectCtrlName.str().data());
        if (!connectCtrl) {
            PAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
            goto free_fe;
        }
        ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
        if (ret) {
            PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
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
            case PAL_SP_MODE_FACTORY_TEST:
                modeConfg.th_operation_mode = FACTORY_TEST_MODE;
            break;
            case PAL_SP_MODE_V_VALIDATION:
                modeConfg.th_operation_mode = V_VALIDATION_MODE;
            break;
            case PAL_SP_MODE_DYNAMIC_CAL:
            default:
                PAL_ERR(LOG_TAG, "Normal mode being used");
                modeConfg.th_operation_mode = NORMAL_MODE;
        }
        modeConfg.th_quick_calib_flag = 0;

        ret = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIdTx.at(0),
                        backEndName.c_str(), MODULE_VI, &miid);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, ret);
        }
        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            free(payload);
            if (0 != ret) {
                PAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
            }
        }

        // Setting Channel Map configuration for VI module
        viChannelMapConfg.num_ch = numberOfChannels * 2;
        payloadSize = 0;

        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_VI_CHANNEL_MAP_CFG,(void *)&viChannelMapConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            free(payload);
            if (0 != ret) {
                PAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
            }
        }

        // Setting Excursion mode
        viExModeConfg.operation_mode = 0; // Normal Mode
        payloadSize = 0;

        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_EX_VI_MODE_CFG,(void *)&viExModeConfg);
        if (payloadSize) {
            ret = updateCustomPayload(payload, payloadSize);
            free(payload);
            if (0 != ret) {
                PAL_ERR(LOG_TAG," updateCustomPayload Failed for EX_VI_MODE_CFG\n");
            }
        }

        if (rm->mSpkrProtModeValue.operationMode) {
            PAL_DBG(LOG_TAG, "Operation mode %d",
                    rm->mSpkrProtModeValue.operationMode);
            param_id_sp_th_vi_ftm_cfg_t viFtmConfg;
            viFtmConfg.num_ch = numberOfChannels;
            payloadSize = 0;
            switch (rm->mSpkrProtModeValue.operationMode) {
                case PAL_SP_MODE_FACTORY_TEST:
                    viParamId = PARAM_ID_SP_TH_VI_FTM_CFG;
                break;
                case PAL_SP_MODE_V_VALIDATION:
                    viParamId = PARAM_ID_SP_TH_VI_V_VALI_CFG;
                break;
                case PAL_SP_MODE_DYNAMIC_CAL:
                    PAL_ERR(LOG_TAG, "Dynamic cal in Processing mode!!");
                break;
            }
            builder->payloadSPConfig (&payload, &payloadSize, miid,
                    viParamId, (void *) &viFtmConfg);
            if (payloadSize) {
                ret = updateCustomPayload(payload, payloadSize);
                free(payload);
                if (0 != ret) {
                    PAL_ERR(LOG_TAG," Payload Failed for FTM mode\n");
                }
            }
        }

        // Setting the R0T0 values
        if (spkrCalState == SPKR_CALIBRATED) {
            PAL_DBG(LOG_TAG, "Speaker calibrated. Read from the file");
            fp = fopen(PAL_SP_TEMP_PATH, "rb");
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
            PAL_DBG(LOG_TAG, "Speaker not calibrated. Send safe value");
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
            free(payload);
            free(spR0T0confg);
            if (0 != ret) {
                PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            }
        }

        // Setting the values for VI module
        if (customPayloadSize) {
            ret = SessionAlsaUtils::setDeviceCustomPayload(rm, backEndName,
                            customPayload, customPayloadSize);
            if (ret) {
                PAL_ERR(LOG_TAG, "Unable to set custom param for mode");
                goto free_fe;
            }
        }

        txPcm = pcm_open(rm->getSndCard(), pcmDevIdTx.at(0), flags, &config);
        if (!txPcm) {
            PAL_ERR(LOG_TAG, "txPcm open failed");
            goto free_fe;
        }

        if (!pcm_is_ready(txPcm)) {
            PAL_ERR(LOG_TAG, "txPcm open not ready");
            goto err_pcm_open;
        }

        // Setting up SP mode
        rm->getBackendName(mDeviceAttr.id, backEndName);
        dev = Device::getInstance(&mDeviceAttr, rm);
        ret = rm->getActiveStream_l(dev, activeStreams);
        if ((0 != ret) || (activeStreams.size() == 0)) {
            PAL_ERR(LOG_TAG, " no active stream available");
            ret = -EINVAL;
            goto done;
        }
        stream = static_cast<Stream *>(activeStreams[0]);
        stream->getAssociatedSession(&session);
        ret = session->getMIID(backEndName.c_str(), MODULE_SP, &miid);
        if (ret) {
            PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_SP, ret);
            goto done;
        }

        // Set the operation mode for SP module
        PAL_DBG(LOG_TAG, "Operation mode for SP %d",
                        rm->mSpkrProtModeValue.operationMode);
        switch (rm->mSpkrProtModeValue.operationMode) {
            case PAL_SP_MODE_FACTORY_TEST:
                spModeConfg.operation_mode = FACTORY_TEST_MODE;
            break;
            case PAL_SP_MODE_V_VALIDATION:
                spModeConfg.operation_mode = V_VALIDATION_MODE;
            break;
            default:
                PAL_ERR(LOG_TAG, "Normal mode being used");
                spModeConfg.operation_mode = NORMAL_MODE;
        }

        payloadSize = 0;
        builder->payloadSPConfig(&payload, &payloadSize, miid,
                PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
        if (payloadSize) {
            if (customPayload) {
                free (customPayload);
                customPayloadSize = 0;
                customPayload = NULL;
            }
            ret = updateCustomPayload(payload, payloadSize);
            free(payload);
            if (0 != ret) {
                PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
            }
        }

        // CPS related payload
        if (ResourceManager::isCpsEnabled) {
            updateCpsCustomPayload(miid);
        }

        enableDevice(audioRoute, mSndDeviceName_vi);
        PAL_DBG(LOG_TAG, "pcm start for TX");
        if (pcm_start(txPcm) < 0) {
            PAL_ERR(LOG_TAG, "pcm start failed for TX path");
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
        PAL_DBG(LOG_TAG, "Closing the VI path");
        if (txPcm) {
            rm = ResourceManager::getInstance();
            device.id = PAL_DEVICE_IN_VI_FEEDBACK;

            ret = rm->getAudioRoute(&audioRoute);
            if (0 != ret) {
                PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
                goto done;
            }

            ret = rm->getSndDeviceName(device.id , mSndDeviceName_vi);
            disableDevice(audioRoute, mSndDeviceName_vi);
            pcm_stop(txPcm);
            pcm_close(txPcm);
            txPcm = NULL;
            sAttr.type = PAL_STREAM_LOW_LATENCY;
            sAttr.direction = PAL_AUDIO_INPUT_OUTPUT;
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
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return ret;
}

void SpeakerProtection::updateSPcustomPayload()
{
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    std::string backEndName;
    std::shared_ptr<Device> dev = nullptr;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activeStreams;
    uint32_t miid = 0, ret;
    param_id_sp_op_mode_t spModeConfg;

    rm->getBackendName(mDeviceAttr.id, backEndName);
    dev = Device::getInstance(&mDeviceAttr, rm);
    ret = rm->getActiveStream_l(dev, activeStreams);
    if ((0 != ret) || (activeStreams.size() == 0)) {
        PAL_ERR(LOG_TAG, " no active stream available");
        goto exit;
    }
    stream = static_cast<Stream *>(activeStreams[0]);
    stream->getAssociatedSession(&session);
    ret = session->getMIID(backEndName.c_str(), MODULE_SP, &miid);
    if (ret) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_SP, ret);
        goto exit;
    }

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    spModeConfg.operation_mode = NORMAL_MODE;
    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_OP_MODE,(void *)&spModeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }

exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return;
}


int SpeakerProtection::speakerProtectionDynamicCal()
{
    int ret = 0;

    PAL_DBG(LOG_TAG, "Trigger Dynamic Cal");

    if (spkrCalState == SPKR_CALIB_IN_PROGRESS) {
        PAL_DBG(LOG_TAG, "Calibration already running");
        return ret;
    }

    threadExit = false;
    spkrCalState = SPKR_NOT_CALIBRATED;

    calibrationCallbackStatus = 0;
    mDspCallbackRcvd = false;

    mCalThread = std::thread(&SpeakerProtection::spkrCalibrationThread,
                        this);
    return ret;
}

int SpeakerProtection::start()
{
    PAL_DBG(LOG_TAG, "Inside Speaker Protection start");

    if (ResourceManager::isVIRecordStarted) {
        PAL_DBG(LOG_TAG, "record running so just update SP payload");
        updateSPcustomPayload();
    }
    else {
        spkrProtProcessingMode(true);
    }
    Device::start();
    return 0;
}

int SpeakerProtection::stop()
{
    PAL_DBG(LOG_TAG, "Inside Speaker Protection stop");
    Device::stop();
    if (ResourceManager::isVIRecordStarted) {
        PAL_DBG(LOG_TAG, "record running so no need to proceed");
        ResourceManager::isVIRecordStarted = false;
        return 0;
    }
    spkrProtProcessingMode(false);
    return 0;
}


int32_t SpeakerProtection::setParameter(uint32_t param_id, void *param)
{
    PAL_DBG(LOG_TAG, "Inside Speaker Protection Set parameters");
    (void ) param;
    if (param_id == PAL_SP_MODE_DYNAMIC_CAL)
        speakerProtectionDynamicCal();
    return 0;
}

int32_t SpeakerProtection::getParameter(uint32_t param_id, void **param)
{
    int size = 0, status = 0 ;
    const char *getParamControl = "getParam";
    std::ostringstream cntrlName;
    std::ostringstream resString;
    std::string backendName;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    char *pcmDeviceName = NULL;
    struct mixer_ctl *ctl;
    uint32_t miid = 0;
    param_id_sp_th_vi_ftm_params_t ftm;
    param_id_sp_th_vi_v_vali_params_t v_val;
    PayloadBuilder* builder = new PayloadBuilder();
    vi_th_ftm_params_t ftm_ret[numberOfChannels];
    vi_th_v_vali_params_t v_vali_ret[numberOfChannels];
    param_id_sp_th_vi_v_vali_params_t *v_valValue;
    param_id_sp_th_vi_ftm_params_t *ftmValue;
    int spkr1_status = 0;
    int spkr2_status = 0;

    memset(&ftm_ret, 0,sizeof(vi_th_ftm_params_t)*numberOfChannels);
    memset(&v_vali_ret, 0,sizeof(vi_th_v_vali_params_t)*numberOfChannels);

    if (param_id != PAL_PARAM_ID_SP_MODE)
        goto exit;

    pcmDeviceName = rm->getDeviceNameFromID(pcmDevIdTx.at(0));
    cntrlName<<pcmDeviceName<<" "<<getParamControl;

    ctl = mixer_get_ctl_by_name(mixer, cntrlName.str().data());
    if (!ctl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: %s\n", cntrlName.str().data());
        status = -ENOENT;
        goto exit;
    }
    rm->getBackendName(PAL_DEVICE_IN_VI_FEEDBACK, backendName);

    status = SessionAlsaUtils::getModuleInstanceId(mixer, pcmDevIdTx.at(0),
                        backendName.c_str(), MODULE_VI, &miid);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_VI, status);
    }

    ftm.num_ch = numberOfChannels;
    builder->payloadSPConfig (&payload, &payloadSize, miid,
            PARAM_ID_SP_TH_VI_FTM_PARAMS, &ftm);

    status = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Set failed status = %d", status);
        goto exit;
    }

    memset(payload, 0, payloadSize);

    status = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Get failed status = %d", status);
    }
    else {

        ftmValue = (param_id_sp_th_vi_ftm_params_t *) (payload +
                        sizeof(struct apm_module_param_data_t));

        for (int i = 0; i < numberOfChannels; i++) {
            ftm_ret[i].ftm_dc_res_q24 = ftmValue->vi_th_ftm_params[i].ftm_dc_res_q24;
            ftm_ret[i].ftm_temp_q22 = ftmValue->vi_th_ftm_params[i].ftm_temp_q22;
            ftm_ret[i].status = ftmValue->vi_th_ftm_params[i].status;
        }
    }

    PAL_DBG(LOG_TAG, "Got FTM value with status %d", ftm_ret[0].status);

    if (payload) {
        delete payload;
        payloadSize = 0;
        payload = NULL;
    }

    v_val.num_ch = numberOfChannels;
    builder->payloadSPConfig (&payload, &payloadSize, miid,
            PARAM_ID_SP_TH_VI_V_VALI_PARAMS, &v_val);

    status = mixer_ctl_set_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Set failed status = %d", status);
        goto exit;
    }

    memset(payload, 0, payloadSize);

    status = mixer_ctl_get_array(ctl, payload, payloadSize);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Get failed status = %d", status);
    }
    else {
        v_valValue = (param_id_sp_th_vi_v_vali_params_t *) (payload +
                            sizeof(struct apm_module_param_data_t));

        for (int i = 0; i < numberOfChannels; i++) {
            v_vali_ret[i].vrms_q24 = v_valValue->vi_th_v_vali_params[i].vrms_q24;
            v_vali_ret[i].status = v_valValue->vi_th_v_vali_params[i].status;
        }
    }
    PAL_DBG(LOG_TAG, "Got V-Val value with status %d", v_vali_ret[0].status);

    if (payload) {
        delete payload;
        payloadSize = 0;
        payload = NULL;
    }

    switch(numberOfChannels) {
        case 1 :
            if (v_vali_ret[0].status == 1 || ftm_ret[0].status == 4)
                spkr1_status = 1;
            resString << "SpkrParamStatus: " << spkr1_status << "; Rdc: "
                    << ((ftm_ret[0].ftm_dc_res_q24)/(1<<24)) << "; Temp: "
                    << ((ftm_ret[0].ftm_temp_q22)/(1<<22)) << "; Rms: "
                    << ((v_vali_ret[0].vrms_q24)/(1<<24));
        break;
        case 2 :
            if (v_vali_ret[0].status == 1 || ftm_ret[0].status == 4)
                spkr1_status = 1;
            if (v_vali_ret[1].status == 1 || ftm_ret[1].status == 4)
                spkr2_status = 1;
            resString << "SpkrParamStatus: " << spkr1_status <<", "<< spkr2_status
                    << "; Rdc: " << ((ftm_ret[0].ftm_dc_res_q24)/(1<<24)) << ", "
                    << ((ftm_ret[1].ftm_dc_res_q24)/(1<<24)) << "; Temp: "
                    << ((ftm_ret[0].ftm_temp_q22)/(1<<22)) << ", "
                    << ((ftm_ret[1].ftm_temp_q22)/(1<<22)) <<"; Rms: "
                    << ((v_vali_ret[0].vrms_q24)/(1<<24)) << ", "
                    << ((v_vali_ret[1].vrms_q24)/(1<<24));
        break;
        default :
            PAL_ERR(LOG_TAG, "No support for Speakers > 2");
        break;
    }

    PAL_DBG(LOG_TAG, "Get param value %s", resString.str().c_str());
    if (resString.str().length() > 0) {
        memcpy((char *) (param), resString.str().c_str(),
                resString.str().length());
        size = resString.str().length();

        // Get is done now, we will clear up the stored mode now
        memset(&(rm->mSpkrProtModeValue), 0, sizeof(pal_spkr_prot_payload));
    }

exit :
    if(builder) {
       delete builder;
       builder = NULL;
        }
    if(!status)
       return size;
    else
      return status;
}

/*
 * VI feedack related functionalities
 */

void SpeakerFeedback::updateVIcustomPayload()
{
    PayloadBuilder* builder = new PayloadBuilder();
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    std::string backEndName;
    std::shared_ptr<Device> dev = nullptr;
    Stream *stream = NULL;
    Session *session = NULL;
    std::vector<Stream*> activeStreams;
    uint32_t miid = 0, ret = 0;
    struct vi_r0t0_cfg_t r0t0Array[numSpeaker];
    FILE *fp = NULL;
    param_id_sp_th_vi_r0t0_cfg_t *spR0T0confg;
    param_id_sp_vi_op_mode_cfg_t modeConfg;
    param_id_sp_vi_channel_map_cfg_t viChannelMapConfg;
    param_id_sp_ex_vi_mode_cfg_t viExModeConfg;

    rm->getBackendName(mDeviceAttr.id, backEndName);
    dev = Device::getInstance(&mDeviceAttr, rm);
    ret = rm->getActiveStream_l(dev, activeStreams);
    if ((0 != ret) || (activeStreams.size() == 0)) {
        PAL_ERR(LOG_TAG, " no active stream available");
        goto exit;
    }
    stream = static_cast<Stream *>(activeStreams[0]);
    stream->getAssociatedSession(&session);
    ret = session->getMIID(backEndName.c_str(), MODULE_VI, &miid);
    if (ret) {
        PAL_ERR(LOG_TAG, "Failed to get tag info %x, status = %d", MODULE_SP, ret);
        goto exit;
    }

    if (customPayloadSize) {
        free(customPayload);
        customPayloadSize = 0;
    }

    memset(&modeConfg, 0, sizeof(modeConfg));
    memset(&viChannelMapConfg, 0, sizeof(viChannelMapConfg));
    memset(&viExModeConfg, 0, sizeof(viExModeConfg));

    // Setting the mode of VI module
    modeConfg.num_speakers = numSpeaker;
    modeConfg.th_operation_mode = NORMAL_MODE;
    modeConfg.th_quick_calib_flag = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                             PARAM_ID_SP_VI_OP_MODE_CFG,(void *)&modeConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for VI_OP_MODE_CFG\n");
        }
    }

    // Setting Channel Map configuration for VI module
    viChannelMapConfg.num_ch = numSpeaker * 2;
    payloadSize = 0;

    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_VI_CHANNEL_MAP_CFG,(void *)&viChannelMapConfg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed for CHANNEL_MAP_CFG\n");
        }
    }

    fp = fopen(PAL_SP_TEMP_PATH, "rb");
    if (fp) {
        PAL_DBG(LOG_TAG, "Speaker calibrated. Send calibrated value");
        for (int i = 0; i < numSpeaker; i++) {
            fread(&r0t0Array[i].r0_cali_q24,
                    sizeof(r0t0Array[i].r0_cali_q24), 1, fp);
            fread(&r0t0Array[i].t0_cali_q6,
                    sizeof(r0t0Array[i].t0_cali_q6), 1, fp);
        }
    }
    else {
        PAL_DBG(LOG_TAG, "Speaker not calibrated. Send safe value");
        for (int i = 0; i < numSpeaker; i++) {
            r0t0Array[i].r0_cali_q24 = MIN_RESISTANCE_SPKR_Q24;
            r0t0Array[i].t0_cali_q6 = SAFE_SPKR_TEMP_Q6;
        }
    }
    spR0T0confg = (param_id_sp_th_vi_r0t0_cfg_t *)calloc(1,
                        sizeof(param_id_sp_th_vi_r0t0_cfg_t) +
                        sizeof(vi_r0t0_cfg_t) * numSpeaker);
    spR0T0confg->num_speakers = numSpeaker;

    memcpy(spR0T0confg->vi_r0t0_cfg, r0t0Array, sizeof(vi_r0t0_cfg_t) *
            numSpeaker);

    payloadSize = 0;
    builder->payloadSPConfig(&payload, &payloadSize, miid,
                    PARAM_ID_SP_TH_VI_R0T0_CFG,(void *)spR0T0confg);
    if (payloadSize) {
        ret = updateCustomPayload(payload, payloadSize);
        free(payload);
        free(spR0T0confg);
        if (0 != ret) {
            PAL_ERR(LOG_TAG," updateCustomPayload Failed\n");
        }
    }
exit:
    if(builder) {
       delete builder;
       builder = NULL;
    }
    return;
}

SpeakerFeedback::SpeakerFeedback(struct pal_device *device,
                                std::shared_ptr<ResourceManager> Rm):Speaker(device, Rm)
{
    struct pal_device_info devinfo = {};

    memset(&mDeviceAttr, 0, sizeof(struct pal_device));
    memcpy(&mDeviceAttr, device, sizeof(struct pal_device));
    rm = Rm;


    rm->getDeviceInfo(mDeviceAttr.id, PAL_STREAM_PROXY, &devinfo);
    numSpeaker = devinfo.channels;
}

SpeakerFeedback::~SpeakerFeedback()
{
}

int32_t SpeakerFeedback::start()
{
    ResourceManager::isVIRecordStarted = true;
    // Do the customPayload configuration for VI path and call the Device::start
    PAL_DBG(LOG_TAG," Feedback start\n");
    updateVIcustomPayload();
    Device::start();

    return 0;
}

int32_t SpeakerFeedback::stop()
{
    ResourceManager::isVIRecordStarted = false;
    PAL_DBG(LOG_TAG," Feedback stop\n");
    Device::stop();

    return 0;
}

std::shared_ptr<Device> SpeakerFeedback::getInstance(struct pal_device *device,
                                                     std::shared_ptr<ResourceManager> Rm)
{
    PAL_DBG(LOG_TAG," Feedback getInstance\n");
    if (!obj) {
        std::shared_ptr<Device> sp(new SpeakerFeedback(device, Rm));
        obj = sp;
    }
    return obj;
}
