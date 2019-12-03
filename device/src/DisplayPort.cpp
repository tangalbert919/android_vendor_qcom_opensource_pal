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

#define LOG_TAG "DisplayPort"
#include "DisplayPort.h"
#include "ResourceManager.h"
#include "Device.h"
#include "kvh2xml.h"

std::shared_ptr<Device> DisplayPort::obj = nullptr;

enum {
    EXT_DISPLAY_TYPE_NONE,
    EXT_DISPLAY_TYPE_HDMI,
    EXT_DISPLAY_TYPE_DP
};

/*
 * This file will have a maximum of 38 bytes:
 *
 * 4 bytes: number of audio blocks
 * 4 bytes: total length of Short Audio Descriptor (SAD) blocks
 * Maximum 10 * 3 bytes: SAD blocks
 */
#define MAX_SAD_BLOCKS      10
#define SAD_BLOCK_SIZE      3

static struct extDispState {
    void *edidInfo = NULL;
    bool valid = false;
    int type = EXT_DISPLAY_TYPE_NONE;
} extDisp[MAX_CONTROLLERS][MAX_STREAMS_PER_CONTROLLER];

std::shared_ptr<Device> DisplayPort::getInstance(struct qal_device *device,
                                             std::shared_ptr<ResourceManager> Rm)
{
    QAL_DBG(LOG_TAG, "%s :Enter:",__func__);
    if (!obj) {
        std::shared_ptr<Device> sp(new DisplayPort(device, Rm));
        obj = sp;
    }
    QAL_DBG(LOG_TAG, "%s :Exit:",__func__);
    return obj;
}


DisplayPort::DisplayPort(struct qal_device *device, std::shared_ptr<ResourceManager> Rm) :
Device(device, Rm)
{

}

DisplayPort::~DisplayPort()
{
    QAL_ERR(LOG_TAG, "dtor called");
}

/*DisplayPort1::DisplayPort1(struct qal_device *device, std::shared_ptr<ResourceManager> Rm) :
DisplayPort(device, Rm)
{

}

DisplayPort1::~DisplayPort1()
{
    QAL_ERR(LOG_TAG, "dtor called");
}

bool DisplayPort1::isDisplayPortEnabled () {

    return DisplayPort::isDisplayPortEnabled ();

}*/

int DisplayPort::init(qal_param_device_connection_t device_conn)
{
    QAL_DBG(LOG_TAG," %s:Enter", __func__);
    int status = 0;
    struct mixer *mixer;
    status = rm->getAudioMixer(&mixer);
    if (status) {
        QAL_ERR(LOG_TAG,"%s :mixer error",__func__);
        return status;
    }
    QAL_ERR(LOG_TAG,"%s : get mixer success");
    qal_param_disp_port_config_params* dp_config = (qal_param_disp_port_config_params*) &device_conn.device_config.dp_config;
    setExtDisplayDevice(mixer, dp_config->controller, dp_config->stream);
    status = getExtDispType(mixer, dp_config->controller, dp_config->stream);
    if (status < 0) {
        QAL_ERR(LOG_TAG," Failed to query disp type, status:%d", status);
    } else {
        cacheEdid(mixer, dp_config->controller, dp_config->stream);
    }
    QAL_DBG(LOG_TAG," %s:Exit", __func__);
    return 0;
}

int DisplayPort::deinit(qal_param_device_connection_t device_conn)
{

    return 0;
}

bool DisplayPort::isDisplayPortEnabled () {
    //TBD: Check for the system prop here
    return true;
}

/*void DisplayPort1::resetEdidInfo()
{
    DisplayPort::resetEdidInfo();

}*/


void DisplayPort::resetEdidInfo() {
    QAL_VERBOSE(LOG_TAG," enter");

    int i = 0, j = 0;
    for (i = 0; i < MAX_CONTROLLERS; ++i) {
        for (j = 0; j < MAX_STREAMS_PER_CONTROLLER; ++j) {
            struct extDispState *state = &extDisp[i][j];
            state->type = EXT_DISPLAY_TYPE_NONE;
            if (state->edidInfo) {
                free(state->edidInfo);
                state->edidInfo = NULL;
            }
            state->valid = false;
        }
    }
}

/*
 * returns index for mixer controls
 *
 * example: max controllers = 2, max streams = 4
 * controller = 0, stream = 0 => Index 0
 * ...
 * controller = 0, stream = 3 => Index 3
 * controller = 1, stream = 0 => Index 4
 * ...
 * controller = 1, stream = 3 => Index 7
 */
int32_t DisplayPort::getDisplayPortCtlIndex(int controller, int stream)
{

    if (controller < 0 || controller >= MAX_CONTROLLERS ||
            stream < 0 || stream >= MAX_STREAMS_PER_CONTROLLER) {
        QAL_ERR(LOG_TAG,"Invalid controller/stream - %d/%d",
              controller, stream);
        return -EINVAL;
    }

    return ((controller % MAX_CONTROLLERS) * MAX_STREAMS_PER_CONTROLLER) +
            (stream % MAX_STREAMS_PER_CONTROLLER);
}

int32_t DisplayPort::setExtDisplayDevice(struct audio_mixer *mixer, int controller, int stream)
{
    struct mixer_ctl *ctl = NULL;
    int ctlIndex = 0;
    const char *ctlNamePrefix = "External Display";
    const char *ctlNameSuffix = "Audio Device";
    char mixerCtlName[MIXER_PATH_MAX_LENGTH] = {0};
    int deviceValues[2] = {-1, -1};

    ctlIndex = getDisplayPortCtlIndex(controller, stream);
    if (-EINVAL == ctlIndex) {
        QAL_ERR(LOG_TAG,"Unknown controller/stream %d/%d", controller, stream);
        return -EINVAL;
    }

    QAL_ERR(LOG_TAG," ctlIndex: %d controller: %d stream: %d", ctlIndex, controller, stream);

    if (0 == ctlIndex)
        snprintf(mixerCtlName, sizeof(mixerCtlName),
                 "%s %s", ctlNamePrefix, ctlNameSuffix);
    else
        snprintf(mixerCtlName, sizeof(mixerCtlName),
                 "%s%d %s", ctlNamePrefix, ctlIndex, ctlNameSuffix);

    deviceValues[0] = controller;
    deviceValues[1] = stream;

    QAL_ERR(LOG_TAG," mixer: %pK mixer ctl name: %s", mixer, mixerCtlName);

    ctl = mixer_get_ctl_by_name(mixer, mixerCtlName);
    if (!ctl) {
        QAL_ERR(LOG_TAG,"Could not get ctl for mixer cmd - %s", mixerCtlName);
        return -EINVAL;
    }

    QAL_ERR(LOG_TAG,"controller/stream: %d/%d", deviceValues[0], deviceValues[1]);

    return mixer_ctl_set_array(ctl, deviceValues, ARRAY_SIZE(deviceValues));
}

int32_t DisplayPort::getExtDispType(struct audio_mixer *mixer, int controller, int stream)
{
    int dispType = EXT_DISPLAY_TYPE_NONE;
    int ctlIndex = 0;
    struct extDispState *disp = NULL;

    ctlIndex = getDisplayPortCtlIndex(controller, stream);
    if (-EINVAL == ctlIndex) {
        QAL_ERR(LOG_TAG,"Unknown controller/stream %d/%d", controller, stream);
        return -EINVAL;
    }

    disp = &extDisp[controller][stream];
    if (disp->type != EXT_DISPLAY_TYPE_NONE) {
        QAL_DBG(LOG_TAG," Returning cached ext disp type:%s",
               (disp->type == EXT_DISPLAY_TYPE_DP) ? "DisplayPort" : "HDMI");
         return disp->type;
    }

    if (isDisplayPortEnabled()) {
        struct mixer_ctl *ctl = NULL;
        const char *ctlNamePrefix = "External Display";
        const char *ctlNameSuffix = "Type";
        char mixerCtlName[MIXER_PATH_MAX_LENGTH] = {0};

        if (0 == ctlIndex)
            snprintf(mixerCtlName, sizeof(mixerCtlName),
                     "%s %s", ctlNamePrefix, ctlNameSuffix);
        else
            snprintf(mixerCtlName, sizeof(mixerCtlName),
                     "%s%d %s", ctlNamePrefix, ctlIndex, ctlNameSuffix);

        QAL_VERBOSE(LOG_TAG,"mixer ctl name: %s", mixerCtlName);

        ctl = mixer_get_ctl_by_name(mixer, mixerCtlName);
        if (!ctl) {
            QAL_ERR(LOG_TAG,"Could not get ctl for mixer cmd - %s", mixerCtlName);
            return -EINVAL;
        }

        dispType = mixer_ctl_get_value(ctl, 0);
        if (dispType == EXT_DISPLAY_TYPE_NONE) {
            QAL_ERR(LOG_TAG,"Invalid external display type: %d", dispType);
            return -EINVAL;
        }
    } else {
        dispType = EXT_DISPLAY_TYPE_HDMI;
    }

    disp->type = dispType;

    QAL_DBG(LOG_TAG," ext disp type: %s", (dispType == EXT_DISPLAY_TYPE_DP) ? "DisplayPort" : "HDMI");

    return dispType;
}

int DisplayPort::getEdidInfo(struct audio_mixer *mixer, int controller, int stream)
{

    char block[MAX_SAD_BLOCKS * SAD_BLOCK_SIZE];
    int ret, count;
    char edidData[MAX_SAD_BLOCKS * SAD_BLOCK_SIZE + 1] = {0};
    struct extDispState *state = NULL;
    int ctlIndex = 0;
    struct mixer_ctl *ctl = NULL;
    const char *ctlNamePrefix = "Display Port";
    const char *ctlNameSuffix = "EDID";
    char mixerCtlName[MIXER_PATH_MAX_LENGTH] = {0};

    ctlIndex = getDisplayPortCtlIndex(controller, stream);
    if (-EINVAL == ctlIndex) {
        QAL_ERR(LOG_TAG," Unknown controller/stream %d/%d", controller, stream);
        return -EINVAL;
    }

    state = &extDisp[controller][stream];
    if (state->valid) {
        /* use cached edid */
        return 0;
    }

    switch(state->type) {
        case EXT_DISPLAY_TYPE_HDMI:
            snprintf(mixerCtlName, sizeof(mixerCtlName), "HDMI EDID");
            break;
        case EXT_DISPLAY_TYPE_DP:
            if (!isDisplayPortEnabled()) {
                QAL_ERR(LOG_TAG," display port is not supported");
                return -EINVAL;
            }

            if (0 == ctlIndex)
                snprintf(mixerCtlName, sizeof(mixerCtlName),
                         "%s %s", ctlNamePrefix, ctlNameSuffix);
            else
                snprintf(mixerCtlName, sizeof(mixerCtlName),
                         "%s%d %s", ctlNamePrefix, ctlIndex, ctlNameSuffix);
            break;
        default:
            QAL_ERR(LOG_TAG," Invalid disp_type %d", state->type);
            return -EINVAL;
    }

    if (state->edidInfo == NULL)
        state->edidInfo =
            (struct edidAudioInfo *)calloc(1, sizeof(struct edidAudioInfo));

    QAL_VERBOSE(LOG_TAG," mixer ctl name: %s", mixerCtlName);

    ctl = mixer_get_ctl_by_name(mixer, mixerCtlName);
    if (!ctl) {
        QAL_ERR(LOG_TAG," Could not get ctl for mixer cmd - %s", mixerCtlName);
        goto fail;
    }

    mixer_ctl_update(ctl);

    count = mixer_ctl_get_num_values(ctl);

    /* Read SAD blocks, clamping the maximum size for safety */
    if (count > (int)sizeof(block))
        count = (int)sizeof(block);

    ret = mixer_ctl_get_array(ctl, block, count);
    if (ret != 0) {
        QAL_ERR(LOG_TAG," mixer_ctl_get_array() failed to get EDID info");
        goto fail;
    }
    edidData[0] = count;
    memcpy(&edidData[1], block, count);

    QAL_VERBOSE(LOG_TAG," received edid data: count %d", edidData[0]);

    if (!getSinkCaps((struct edidAudioInfo *)state->edidInfo, edidData)) {
        QAL_ERR(LOG_TAG," Failed to get extn disp sink capabilities");
        goto fail;
    }
    state->valid = true;
    return 0;
fail:
    if (state->edidInfo) {
        free(state->edidInfo);
        state->edidInfo = NULL;
        state->valid = false;
    }
    QAL_ERR(LOG_TAG," return -EINVAL");
    return -EINVAL;
}

void DisplayPort::cacheEdid(struct audio_mixer *mixer, int controller, int stream)
{
    getEdidInfo(mixer, controller, stream);
}

int32_t DisplayPort::isSampleRateSupported(uint32_t sampleRate)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "sampleRate %d", sampleRate);

    if (sampleRate % SAMPLINGRATE_44K == 0)
        return rc;

    switch (sampleRate) {
        case SAMPLINGRATE_44K:
        case SAMPLINGRATE_48K:
        case SAMPLINGRATE_96K:
        case SAMPLINGRATE_192K:
        case SAMPLINGRATE_384K:
            break;
        default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "sample rate not supported rc %d", rc);
            break;
    }
    return rc;
}
//TBD why do these channels have to be supported, DisplayPorts support only 1/2?
int32_t DisplayPort::isChannelSupported(uint32_t numChannels)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "numChannels %u", numChannels);
    switch (numChannels) {
        case CHANNELS_1:
        case CHANNELS_2:
            break;
        default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "channels not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t DisplayPort::isBitWidthSupported(uint32_t bitWidth)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (bitWidth) {
        case BITWIDTH_16:
        case BITWIDTH_24:
        case BITWIDTH_32:
            break;
        default:
            rc = -EINVAL;
            QAL_ERR(LOG_TAG, "bit width not supported rc %d", rc);
            break;
    }
    return rc;
}

int32_t DisplayPort::checkAndUpdateBitWidth(uint32_t *bitWidth)
{
    int32_t rc = 0;
    QAL_DBG(LOG_TAG, "bitWidth %u", bitWidth);
    switch (*bitWidth) {
        case BITWIDTH_16:
        case BITWIDTH_24:
        case BITWIDTH_32:
            break;
        default:
            *bitWidth = BITWIDTH_16;
            QAL_DBG(LOG_TAG, "bit width not supported, setting to default 16 bit");
            break;
    }
    return rc;
}

int32_t DisplayPort::checkAndUpdateSampleRate(uint32_t *sampleRate)
{
    int32_t rc = 0;

    if ((*sampleRate % SAMPLINGRATE_44K == 0) &&
        (NATIVE_AUDIO_MODE_MULTIPLE_MIX_IN_DSP == ResourceManager::getNativeAudioSupport())) {
        QAL_DBG(LOG_TAG, "napb: setting sampling rate to %d", *sampleRate);
    } else if (*sampleRate <= SAMPLINGRATE_48K)
        *sampleRate = SAMPLINGRATE_48K;
    else if (*sampleRate > SAMPLINGRATE_48K && *sampleRate <= SAMPLINGRATE_96K)
        *sampleRate = SAMPLINGRATE_96K;
    else if (*sampleRate > SAMPLINGRATE_96K && *sampleRate <= SAMPLINGRATE_192K)
        *sampleRate = SAMPLINGRATE_192K;
    else if (*sampleRate > SAMPLINGRATE_192K && *sampleRate <= SAMPLINGRATE_384K)
        *sampleRate = SAMPLINGRATE_384K;

    QAL_DBG(LOG_TAG, " sampleRate %d", *sampleRate);

    return rc;
}



/* ----------------------------------------------------------------------------------
   ------------------------         Edid                          -------------------
   ----------------------------------------------------------------------------------*/
const char * DisplayPort::edidFormatToStr(unsigned char format)
{
    std::string formatStr = "??";

    switch (format) {
    case LPCM:
        formatStr = "Format:LPCM";
        break;
    case AC3:
        formatStr = "Format:AC-3";
        break;
    case MPEG1:
        formatStr = "Format:MPEG1 (Layers 1 & 2)";
        break;
    case MP3:
        formatStr =  "Format:MP3 (MPEG1 Layer 3)";
        break;
    case MPEG2_MULTI_CHANNEL:
        formatStr = "Format:MPEG2 (multichannel)";
        break;
    case AAC:
        formatStr =  "Format:AAC";
        break;
    case DTS:
        formatStr =  "Format:DTS";
        break;
    case ATRAC:
        formatStr =  "Format:ATRAC";
        break;
    case SACD:
        formatStr =  "Format:One-bit audio aka SACD";
        break;
    case DOLBY_DIGITAL_PLUS:
        formatStr =  "Format:Dolby Digital +";
        break;
    case DTS_HD:
        formatStr =  "Format:DTS-HD";
        break;
    case MAT:
        formatStr =  "Format:MAT (MLP)";
        break;
    case DST:
        formatStr =  "Format:DST";
        break;
    case WMA_PRO:
        formatStr =  "Format:WMA Pro";
        break;
    default:
        break;
    }
    return formatStr.c_str();
}

bool DisplayPort::isSampleRateSupported(unsigned char srByte, int samplingRate)
{
    int result = 0;

    QAL_VERBOSE(LOG_TAG,"%s: srByte: %d, samplingRate: %d",__func__, srByte, samplingRate);
    switch (samplingRate) {
    case 192000:
        result = (srByte & BIT(6));
        break;
    case 176400:
        result = (srByte & BIT(5));
        break;
    case 96000:
        result = (srByte & BIT(4));
        break;
    case 88200:
        result = (srByte & BIT(3));
        break;
    case 48000:
        result = (srByte & BIT(2));
        break;
    case 44100:
        result = (srByte & BIT(1));
        break;
    case 32000:
        result = (srByte & BIT(0));
        break;
     default:
        break;
    }

    if (result)
        return true;

    return false;
}

unsigned char DisplayPort::getEdidBpsByte(unsigned char byte,
                        unsigned char format)
{
    if (format == 0) {
        QAL_VERBOSE(LOG_TAG,"%s: not lpcm format, return 0",__func__);
        return 0;
    }
    return byte;
}

bool DisplayPort::isSupportedBps(unsigned char bpsByte, int bps)
{
    int result = 0;

    switch (bps) {
    case 24:
        QAL_VERBOSE(LOG_TAG,"24bit");
        result = (bpsByte & BIT(2));
        break;
    case 20:
        QAL_VERBOSE(LOG_TAG,"20bit");
        result = (bpsByte & BIT(1));
        break;
    case 16:
        QAL_VERBOSE(LOG_TAG,"16bit");
        result = (bpsByte & BIT(0));
        break;
     default:
        break;
    }

    if (result)
        return true;

    return false;
}

int DisplayPort::getHighestEdidSF(unsigned char byte)
{
    int nfreq = 0;

    if (byte & BIT(6)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 192kHz");
        nfreq = 192000;
    } else if (byte & BIT(5)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 176kHz");
        nfreq = 176000;
    } else if (byte & BIT(4)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 96kHz");
        nfreq = 96000;
    } else if (byte & BIT(3)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 88.2kHz");
        nfreq = 88200;
    } else if (byte & BIT(2)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 48kHz");
        nfreq = 48000;
    } else if (byte & BIT(1)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 44.1kHz");
        nfreq = 44100;
    } else if (byte & BIT(0)) {
        QAL_VERBOSE(LOG_TAG,"Highest: 32kHz");
        nfreq = 32000;
    }
    return nfreq;
}

void DisplayPort::updateChannelMap(edidAudioInfo* info)
{
    /* HDMI Cable follows CEA standard so SAD is received in CEA
     * Input source file channel map is fed to ASM in WAV standard(audio.h)
     * so upto 7.1 SAD bits are:
     * in CEA convention: RLC/RRC,FLC/FRC,RC,RL/RR,FC,LFE,FL/FR
     * in WAV convention: BL/BR,FLC/FRC,BC,SL/SR,FC,LFE,FL/FR
     * Corresponding ADSP IDs (apr-audio_v2.h):
     * PCM_CHANNEL_FL/PCM_CHANNEL_FR,
     * PCM_CHANNEL_LFE,
     * PCM_CHANNEL_FC,
     * PCM_CHANNEL_LS/PCM_CHANNEL_RS,
     * PCM_CHANNEL_CS,
     * PCM_CHANNEL_FLC/PCM_CHANNEL_FRC
     * PCM_CHANNEL_LB/PCM_CHANNEL_RB
     */
    if (!info)
        return;
    memset(info->channelMap, 0, MAX_CHANNELS_SUPPORTED);
    if(info->speakerAllocation[0] & BIT(0)) {
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
    }
    if(info->speakerAllocation[0] & BIT(1)) {
        info->channelMap[2] = PCM_CHANNEL_LFE;
    }
    if(info->speakerAllocation[0] & BIT(2)) {
        info->channelMap[3] = PCM_CHANNEL_FC;
    }
    if(info->speakerAllocation[0] & BIT(3)) {
    /*
     * As per CEA(HDMI Cable) standard Bit 3 is equivalent
     * to SideLeft/SideRight of WAV standard
     */
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
    }
    if(info->speakerAllocation[0] & BIT(4)) {
        if(info->speakerAllocation[0] & BIT(3)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = 0;
        } else if (info->speakerAllocation[1] & BIT(1)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = PCM_CHANNEL_TS;
        } else if (info->speakerAllocation[1] & BIT(2)) {
            info->channelMap[6] = PCM_CHANNEL_CS;
            info->channelMap[7] = PCM_CHANNEL_CVH;
        } else {
            info->channelMap[4] = PCM_CHANNEL_CS;
            info->channelMap[5] = 0;
        }
    }
    if(info->speakerAllocation[0] & BIT(5)) {
        info->channelMap[6] = PCM_CHANNEL_FLC;
        info->channelMap[7] = PCM_CHANNEL_FRC;
    }
    if(info->speakerAllocation[0] & BIT(6)) {
        // If RLC/RRC is present, RC is invalid as per specification
        info->speakerAllocation[0] &= 0xef;
        /*
         * As per CEA(HDMI Cable) standard Bit 6 is equivalent
         * to BackLeft/BackRight of WAV standard
         */
        info->channelMap[6] = PCM_CHANNEL_LB;
        info->channelMap[7] = PCM_CHANNEL_RB;
    }
    // higher channel are not defined by LPASS
    //info->nSpeakerAllocation[0] &= 0x3f;
    if(info->speakerAllocation[0] & BIT(7)) {
        info->channelMap[6] = 0; // PCM_CHANNEL_FLW; but not defined by LPASS
        info->channelMap[7] = 0; // PCM_CHANNEL_FRW; but not defined by LPASS
    }
    if(info->speakerAllocation[1] & BIT(0)) {
        info->channelMap[6] = 0; // PCM_CHANNEL_FLH; but not defined by LPASS
        info->channelMap[7] = 0; // PCM_CHANNEL_FRH; but not defined by LPASS
    }

    QAL_VERBOSE(LOG_TAG," channel map updated to [%d %d %d %d %d %d %d %d ]  [%x %x %x]"
        , info->channelMap[0], info->channelMap[1], info->channelMap[2]
        , info->channelMap[3], info->channelMap[4], info->channelMap[5]
        , info->channelMap[6], info->channelMap[7]
        , info->speakerAllocation[0], info->speakerAllocation[1]
        , info->speakerAllocation[2]);
}

void DisplayPort::dumpSpeakerAllocation(edidAudioInfo* info)
{
    if (!info)
        return;

    if (info->speakerAllocation[0] & BIT(7))
        QAL_VERBOSE(LOG_TAG,"FLW/FRW");
    if (info->speakerAllocation[0] & BIT(6))
        QAL_VERBOSE(LOG_TAG,"RLC/RRC");
    if (info->speakerAllocation[0] & BIT(5))
        QAL_VERBOSE(LOG_TAG,"FLC/FRC");
    if (info->speakerAllocation[0] & BIT(4))
        QAL_VERBOSE(LOG_TAG,"RC");
    if (info->speakerAllocation[0] & BIT(3))
        QAL_VERBOSE(LOG_TAG,"RL/RR");
    if (info->speakerAllocation[0] & BIT(2))
        QAL_VERBOSE(LOG_TAG,"FC");
    if (info->speakerAllocation[0] & BIT(1))
        QAL_VERBOSE(LOG_TAG,"LFE");
    if (info->speakerAllocation[0] & BIT(0))
        QAL_VERBOSE(LOG_TAG,"FL/FR");
    if (info->speakerAllocation[1] & BIT(2))
        QAL_VERBOSE(LOG_TAG,"FCH");
    if (info->speakerAllocation[1] & BIT(1))
        QAL_VERBOSE(LOG_TAG,"TC");
    if (info->speakerAllocation[1] & BIT(0))
        QAL_VERBOSE(LOG_TAG,"FLH/FRH");
}

void DisplayPort::updateChannelAllocation(edidAudioInfo* info)
{
    int16_t ca;
    int16_t spkrAlloc;

    if (!info)
        return;

    /* Most common 5.1 SAD is 0xF, ca 0x0b
     * and 7.1 SAD is 0x4F, ca 0x13 */
    spkrAlloc = ((info->speakerAllocation[1]) << 8) |
               (info->speakerAllocation[0]);
    QAL_VERBOSE(LOG_TAG,"info->nSpeakerAllocation %x %x\n", info->speakerAllocation[0],
                                              info->speakerAllocation[1]);
    QAL_VERBOSE(LOG_TAG,"spkrAlloc: %x", spkrAlloc);

    /* The below switch case calculates channel allocation values
       as defined in CEA-861 section 6.6.2 */
    switch (spkrAlloc) {
    case BIT(0):                                           ca = 0x00; break;
    case BIT(0)|BIT(1):                                    ca = 0x01; break;
    case BIT(0)|BIT(2):                                    ca = 0x02; break;
    case BIT(0)|BIT(1)|BIT(2):                             ca = 0x03; break;
    case BIT(0)|BIT(4):                                    ca = 0x04; break;
    case BIT(0)|BIT(1)|BIT(4):                             ca = 0x05; break;
    case BIT(0)|BIT(2)|BIT(4):                             ca = 0x06; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(4):                      ca = 0x07; break;
    case BIT(0)|BIT(3):                                    ca = 0x08; break;
    case BIT(0)|BIT(1)|BIT(3):                             ca = 0x09; break;
    case BIT(0)|BIT(2)|BIT(3):                             ca = 0x0A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3):                      ca = 0x0B; break;
    case BIT(0)|BIT(3)|BIT(4):                             ca = 0x0C; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(4):                      ca = 0x0D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4):                      ca = 0x0E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4):               ca = 0x0F; break;
    case BIT(0)|BIT(3)|BIT(6):                             ca = 0x10; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(6):                      ca = 0x11; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(6):                      ca = 0x12; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(6):               ca = 0x13; break;
    case BIT(0)|BIT(5):                                    ca = 0x14; break;
    case BIT(0)|BIT(1)|BIT(5):                             ca = 0x15; break;
    case BIT(0)|BIT(2)|BIT(5):                             ca = 0x16; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(5):                      ca = 0x17; break;
    case BIT(0)|BIT(4)|BIT(5):                             ca = 0x18; break;
    case BIT(0)|BIT(1)|BIT(4)|BIT(5):                      ca = 0x19; break;
    case BIT(0)|BIT(2)|BIT(4)|BIT(5):                      ca = 0x1A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(4)|BIT(5):               ca = 0x1B; break;
    case BIT(0)|BIT(3)|BIT(5):                             ca = 0x1C; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(5):                      ca = 0x1D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(5):                      ca = 0x1E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5):               ca = 0x1F; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(10):                     ca = 0x20; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(10):              ca = 0x21; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(9):                      ca = 0x22; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(9):               ca = 0x23; break;
    case BIT(0)|BIT(3)|BIT(8):                             ca = 0x24; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(8):                      ca = 0x25; break;
    case BIT(0)|BIT(3)|BIT(7):                             ca = 0x26; break;
    case BIT(0)|BIT(1)|BIT(3)|BIT(7):                      ca = 0x27; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4)|BIT(9):               ca = 0x28; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(9):        ca = 0x29; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(4)|BIT(10):              ca = 0x2A; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(10):       ca = 0x2B; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(9)|BIT(10):              ca = 0x2C; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(9)|BIT(10):       ca = 0x2D; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(8):                      ca = 0x2E; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(8):               ca = 0x2F; break;
    case BIT(0)|BIT(2)|BIT(3)|BIT(7):                      ca = 0x30; break;
    case BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(7):               ca = 0x31; break;
    default:                                               ca = 0x0;  break;
    }
    QAL_DBG(LOG_TAG,"%s channel allocation: %x", __func__, ca);
    info->channelAllocation = ca;
}

void DisplayPort::updateChannelMapLpass(edidAudioInfo* info)
{
    if (!info)
        return;
    if (((info->channelAllocation < 0) ||
         (info->channelAllocation > 0x1f)) &&
         (info->channelAllocation != 0x2f)) {
        QAL_ERR(LOG_TAG,"Channel allocation out of supported range");
        return;
    }
    QAL_VERBOSE(LOG_TAG,"channelAllocation 0x%x", info->channelAllocation);
    memset(info->channelMap, 0, MAX_CHANNELS_SUPPORTED);
    switch(info->channelAllocation) {
    case 0x0:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        break;
    case 0x1:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        break;
    case 0x2:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        break;
    case 0x3:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        break;
    case 0x4:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_CS;
        break;
    case 0x5:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_CS;
        break;
    case 0x6:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_CS;
        break;
    case 0x7:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_CS;
        break;
    case 0x8:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LS;
        info->channelMap[3] = PCM_CHANNEL_RS;
        break;
    case 0x9:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        break;
    case 0xa:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        break;
    case 0xb:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
        break;
    case 0xc:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LS;
        info->channelMap[3] = PCM_CHANNEL_RS;
        info->channelMap[4] = PCM_CHANNEL_CS;
        break;
    case 0xd:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_CS;
        break;
    case 0xe:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_CS;
        break;
    case 0xf:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
        info->channelMap[6] = PCM_CHANNEL_CS;
        break;
    case 0x10:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LS;
        info->channelMap[3] = PCM_CHANNEL_RS;
        info->channelMap[4] = PCM_CHANNEL_LB;
        info->channelMap[5] = PCM_CHANNEL_RB;
        break;
    case 0x11:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_LB;
        info->channelMap[6] = PCM_CHANNEL_RB;
        break;
    case 0x12:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_LB;
        info->channelMap[6] = PCM_CHANNEL_RB;
        break;
    case 0x13:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
        info->channelMap[6] = PCM_CHANNEL_LB;
        info->channelMap[7] = PCM_CHANNEL_RB;
        break;
    case 0x14:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FLC;
        info->channelMap[3] = PCM_CHANNEL_FRC;
        break;
    case 0x15:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FLC;
        info->channelMap[4] = PCM_CHANNEL_FRC;
        break;
    case 0x16:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_FLC;
        info->channelMap[4] = PCM_CHANNEL_FRC;
        break;
    case 0x17:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_FLC;
        info->channelMap[5] = PCM_CHANNEL_FRC;
        break;
    case 0x18:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_CS;
        info->channelMap[3] = PCM_CHANNEL_FLC;
        info->channelMap[4] = PCM_CHANNEL_FRC;
        break;
    case 0x19:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_CS;
        info->channelMap[4] = PCM_CHANNEL_FLC;
        info->channelMap[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1a:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_CS;
        info->channelMap[4] = PCM_CHANNEL_FLC;
        info->channelMap[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1b:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_CS;
        info->channelMap[5] = PCM_CHANNEL_FLC;
        info->channelMap[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1c:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LS;
        info->channelMap[3] = PCM_CHANNEL_RS;
        info->channelMap[4] = PCM_CHANNEL_FLC;
        info->channelMap[5] = PCM_CHANNEL_FRC;
        break;
    case 0x1d:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_FLC;
        info->channelMap[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1e:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_FC;
        info->channelMap[3] = PCM_CHANNEL_LS;
        info->channelMap[4] = PCM_CHANNEL_RS;
        info->channelMap[5] = PCM_CHANNEL_FLC;
        info->channelMap[6] = PCM_CHANNEL_FRC;
        break;
    case 0x1f:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
        info->channelMap[6] = PCM_CHANNEL_FLC;
        info->channelMap[7] = PCM_CHANNEL_FRC;
        break;
    case 0x2f:
        info->channelMap[0] = PCM_CHANNEL_FL;
        info->channelMap[1] = PCM_CHANNEL_FR;
        info->channelMap[2] = PCM_CHANNEL_LFE;
        info->channelMap[3] = PCM_CHANNEL_FC;
        info->channelMap[4] = PCM_CHANNEL_LS;
        info->channelMap[5] = PCM_CHANNEL_RS;
        info->channelMap[6] = 0; // PCM_CHANNEL_TFL; but not defined by LPASS
        info->channelMap[7] = 0; // PCM_CHANNEL_TFR; but not defined by LPASS
        break;
    default:
        break;
    }
    QAL_DBG(LOG_TAG,"%s channel map updated to [%d %d %d %d %d %d %d %d ]", __func__
        , info->channelMap[0], info->channelMap[1], info->channelMap[2]
        , info->channelMap[3], info->channelMap[4], info->channelMap[5]
        , info->channelMap[6], info->channelMap[7]);
}


void DisplayPort::updateChannelMask(edidAudioInfo* info)
{
    if (!info)
        return;
    if (((info->channelAllocation < 0) ||
         (info->channelAllocation > 0x1f)) &&
         (info->channelAllocation != 0x2f)) {
        QAL_ERR(LOG_TAG,"Channel allocation out of supported range");
        return;
    }
    QAL_VERBOSE(LOG_TAG,"channelAllocation 0x%x", info->channelAllocation);
    // Don't distinguish channel mask below?
    // AUDIO_CHANNEL_OUT_5POINT1 and AUDIO_CHANNEL_OUT_5POINT1_SIDE
    // AUDIO_CHANNEL_OUT_QUAD and AUDIO_CHANNEL_OUT_QUAD_SIDE
    switch(info->channelAllocation) {
    case 0x0:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        break;
    case 0x1:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        break;
    case 0x2:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        break;
    case 0x3:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        break;
    case 0x4:
        info->channelMask = AUDIO_CHANNEL_OUT_STEREO;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x5:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x6:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        break;
    case 0x7:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        break;
    case 0x8:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        break;
    case 0x9:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        break;
    case 0xa:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        break;
    case 0xb:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        break;
    case 0xc:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xd:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xe:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0xf:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        break;
    case 0x10:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x11:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x12:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_SIDE_RIGHT;
        break;
    case 0x13:
        info->channelMask = AUDIO_CHANNEL_OUT_7POINT1;
        break;
    case 0x14:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x15:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x16:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x17:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x18:
        info->channelMask = AUDIO_CHANNEL_OUT_FRONT_LEFT;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x19:
        info->channelMask = AUDIO_CHANNEL_OUT_2POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_BACK_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1a:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1b:
        info->channelMask = AUDIO_CHANNEL_OUT_SURROUND;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1c:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1d:
        info->channelMask = AUDIO_CHANNEL_OUT_QUAD;
        info->channelMask |= AUDIO_CHANNEL_OUT_LOW_FREQUENCY;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1e:
        info->channelMask = AUDIO_CHANNEL_OUT_PENTA;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x1f:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_LEFT_OF_CENTER;
        info->channelMask |= AUDIO_CHANNEL_OUT_FRONT_RIGHT_OF_CENTER;
        break;
    case 0x2f:
        info->channelMask = AUDIO_CHANNEL_OUT_5POINT1POINT2;
        break;
    default:
        break;
    }
    QAL_DBG(LOG_TAG,"%s channel mask updated to %d", __func__, info->channelMask);
}

void DisplayPort::dumpEdidData(edidAudioInfo *info)
{

    int i;
    for (i = 0; i < info->audioBlocks && i < MAX_EDID_BLOCKS; i++) {
        QAL_VERBOSE(LOG_TAG,"%s:FormatId:%d rate:%d bps:%d channels:%d", __func__,
              info->audioBlocksArray[i].formatId,
              info->audioBlocksArray[i].samplingFreqBitmask,
              info->audioBlocksArray[i].bitsPerSampleBitmask,
              info->audioBlocksArray[i].channels);
    }
    QAL_VERBOSE(LOG_TAG,"%s:no of audio blocks:%d", __func__, info->audioBlocks);
    QAL_VERBOSE(LOG_TAG,"%s:speaker allocation:[%x %x %x]", __func__,
           info->speakerAllocation[0], info->speakerAllocation[1],
           info->speakerAllocation[2]);
    QAL_VERBOSE(LOG_TAG,"%s:channel map:[%x %x %x %x %x %x %x %x]", __func__,
           info->channelMap[0], info->channelMap[1],
           info->channelMap[2], info->channelMap[3],
           info->channelMap[4], info->channelMap[5],
           info->channelMap[6], info->channelMap[7]);
    QAL_VERBOSE(LOG_TAG,"%s:channel allocation:%d", __func__, info->channelAllocation);
    QAL_VERBOSE(LOG_TAG,"%s:[%d %d %d %d %d %d %d %d ]", __func__,
           info->channelMap[0], info->channelMap[1],
           info->channelMap[2], info->channelMap[3],
           info->channelMap[4], info->channelMap[5],
           info->channelMap[6], info->channelMap[7]);
}

bool DisplayPort::getSinkCaps(edidAudioInfo* info, char *edidData)
{
    unsigned char channels[MAX_EDID_BLOCKS];
    unsigned char formats[MAX_EDID_BLOCKS];
    unsigned char frequency[MAX_EDID_BLOCKS];
    unsigned char bitrate[MAX_EDID_BLOCKS];
    int i = 0;
    int length, countDesc;

    if (!info || !edidData) {
        QAL_ERR(LOG_TAG,"No valid EDID");
        return false;
    }

    length = (int) *edidData++;
    QAL_VERBOSE(LOG_TAG,"Total length is %d",length);

    countDesc = length/MIN_AUDIO_DESC_LENGTH;

    if (!countDesc) {
        QAL_ERR(LOG_TAG,"insufficient descriptors");
        return false;
    }

    memset(info, 0, sizeof(edidAudioInfo));

    info->audioBlocks = countDesc-1;
    if (info->audioBlocks > MAX_EDID_BLOCKS) {
        info->audioBlocks = MAX_EDID_BLOCKS;
    }

    QAL_VERBOSE(LOG_TAG,"Total # of audio descriptors %d",countDesc);

    for (i=0; i<info->audioBlocks; i++) {
        // last block for speaker allocation;
        channels [i]   = (*edidData & 0x7) + 1;
        formats  [i]   = (*edidData++) >> 3;
        frequency[i]   = *edidData++;
        bitrate  [i]   = *edidData++;
    }
    info->speakerAllocation[0] = *edidData++;
    info->speakerAllocation[1] = *edidData++;
    info->speakerAllocation[2] = *edidData++;

    updateChannelMap(info);
    updateChannelAllocation(info);
    updateChannelMapLpass(info);
    updateChannelMask(info);

    for (i=0; i<info->audioBlocks; i++) {
        QAL_VERBOSE(LOG_TAG,"AUDIO DESC BLOCK # %d\n",i);

        info->audioBlocksArray[i].channels = channels[i];
        QAL_DBG(LOG_TAG,"info->audioBlocksArray[i].channels %d\n",
              info->audioBlocksArray[i].channels);

        QAL_VERBOSE(LOG_TAG,"Format Byte %d\n", formats[i]);
        info->audioBlocksArray[i].formatId = (edidAudioFormatId)formats[i];
        QAL_DBG(LOG_TAG,"info->audioBlocksArray[i].formatId %s",
             edidFormatToStr(formats[i]));

        QAL_VERBOSE(LOG_TAG,"Frequency Bitmask %d\n", frequency[i]);
        info->audioBlocksArray[i].samplingFreqBitmask = frequency[i];
        QAL_VERBOSE(LOG_TAG,"info->audioBlocksArray[i].samplingFreqBitmask %d",
              info->audioBlocksArray[i].samplingFreqBitmask);

        QAL_VERBOSE(LOG_TAG,"BitsPerSample Bitmask %d\n", bitrate[i]);
        info->audioBlocksArray[i].bitsPerSampleBitmask =
                   getEdidBpsByte(bitrate[i],formats[i]);
        QAL_VERBOSE(LOG_TAG,"info->audioBlocksArray[i].bitsPerSampleBitmask %d",
              info->audioBlocksArray[i].bitsPerSampleBitmask);
    }
    dumpSpeakerAllocation(info);
    dumpEdidData(info);
    return true;
}

bool DisplayPort::isSupportedSR(edidAudioInfo* info, int sr)
{
    int i = 0;
    if (info != NULL && sr != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_EDID_BLOCKS; i++) {
        if (isSampleRateSupported(info->audioBlocksArray[i].samplingFreqBitmask , sr)) {
                QAL_VERBOSE(LOG_TAG,"%s: returns true for sample rate [%d]",
                      __func__, sr);
                return true;
            }
        }
    }
    QAL_VERBOSE(LOG_TAG,"%s: returns false for sample rate [%d]",
           __func__, sr);
    return false;
}

bool DisplayPort::isSupportedBps(edidAudioInfo* info, int bps)
{
    int i = 0;

    if (bps == 16) {
        //16 bit bps is always supported
        //some oem may not update 16bit support in their edid info
        return true;
    }

    if (info != NULL && bps != 0) {
        for (i = 0; i < info->audioBlocks && i < MAX_EDID_BLOCKS; i++) {
            if (isSupportedBps(info->audioBlocksArray[i].bitsPerSampleBitmask, bps)) {
                QAL_VERBOSE(LOG_TAG,"%s: returns true for bit width [%d]",
                      __func__, bps);
                return true;
            }
        }
    }
    QAL_VERBOSE(LOG_TAG,"%s: returns false for bit width [%d]",
           __func__, bps);
    return false;
}

int DisplayPort::getHighestSupportedSR(edidAudioInfo* info)
{
    int sr = 0;
    int highestSR = 0;
    int i;

    if (info != NULL) {
        for (i = 0; i < info->audioBlocks && i < MAX_EDID_BLOCKS; i++) {
          sr = getHighestEdidSF(info->audioBlocksArray[i].samplingFreqBitmask);
          if (sr > highestSR)
            highestSR = sr;
        }
    }
    else
        QAL_ERR(LOG_TAG," info is NULL");

    QAL_VERBOSE(LOG_TAG,"%s: returns [%d] for highest supported sr",
        __func__, highestSR);
    return highestSR;
}
