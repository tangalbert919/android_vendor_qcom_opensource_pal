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

#define LOG_TAG "ResourceManager"
#include "ResourceManager.h"
#include "Session.h"
#include "Device.h"
#include "Stream.h"
#include "StreamPCM.h"
#include "StreamCompress.h"
#include "gsl_intf.h"
#include "SessionGsl.h"

#define MIXER_FILE_DELIMITER "_"
#define MIXER_FILE_EXT ".xml"
#define MIXER_XML_BASE_STRING "/etc/mixer_paths"
#define MIXER_XML_DEFAULT_PATH "/etc/mixer_paths_wsa.xml"
#define MIXER_PATH_MAX_LENGTH 100

#define MAX_SND_CARD 8
#define LOWLATENCY_PCM_DEVICE 15
#define DEEP_BUFFER_PCM_DEVICE 0
#define DEFAULT_ACDB_FILES "/etc/acdbdata/MTP/acdb_cal.acdb"
#define DEVICE_NAME_MAX_SIZE 128
// should be defined in qal_defs.h
#define QAL_DEVICE_MAX QAL_DEVICE_IN_PROXY+1

#define DEFAULT_BIT_WIDTH 16
#define DEFAULT_SAMPLE_RATE 48000
#define DEFAULT_CHANNELS 2
//#define DEFAULT_FORMAT QAL_AUDIO_FMT_DEFAULT_PCM
#define DEFAULT_FORMAT 0x00000000u
// TODO: double check and confirm actual
// values for max sessions number
#define MAX_SESSIONS_LOW_LATENCY 1
#define MAX_SESSIONS_DEEP_BUFFER 1
#define MAX_SESSIONS_COMPRESSED 10
#define MAX_SESSIONS_GENERIC 1

/*
To be defined in detail, if GSL is defined,
pcm device id is directly related to device,
else using legacy design for alsa
*/
//#ifdef CONFIG_GSL
// Will update actual value when numbers got for VT
static int pcm_device_table[QAL_DEVICE_MAX] = {
    [0] = 0,
    [QAL_DEVICE_NONE] = 0,
    [QAL_DEVICE_OUT_EARPIECE] = 1,
    [QAL_DEVICE_OUT_SPEAKER] = 2,
    [QAL_DEVICE_OUT_WIRED_HEADSET] = 0,
    [QAL_DEVICE_OUT_WIRED_HEADPHONE] = 0,
    [QAL_DEVICE_OUT_LINE] = 0,
    [QAL_DEVICE_OUT_BLUETOOTH_SCO] = 0,
    [QAL_DEVICE_OUT_BLUETOOTH_A2DP] = 0,
    [QAL_DEVICE_OUT_AUX_DIGITAL] = 0,
    [QAL_DEVICE_OUT_HDMI] = 0,
    [QAL_DEVICE_OUT_USB_DEVICE] = 0,
    [QAL_DEVICE_OUT_USB_HEADSET] = 0,
    [QAL_DEVICE_OUT_SPDIF] = 0,
    [QAL_DEVICE_OUT_FM] = 0,
    [QAL_DEVICE_OUT_AUX_LINE] = 0,
    [QAL_DEVICE_OUT_PROXY] = 0,

    [QAL_DEVICE_IN_HANDSET_MIC] = 0,
    [QAL_DEVICE_IN_SPEAKER_MIC] = 0,
    [QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET] = 0,
    [QAL_DEVICE_IN_WIRED_HEADSET] = 0,
    [QAL_DEVICE_IN_AUX_DIGITAL] = 0,
    [QAL_DEVICE_IN_HDMI] = 0,
    [QAL_DEVICE_IN_USB_ACCESSORY] = 0,
    [QAL_DEVICE_IN_USB_DEVICE] = 0,
    [QAL_DEVICE_IN_USB_HEADSET] = 0,
    [QAL_DEVICE_IN_FM_TUNER] = 0,
    [QAL_DEVICE_IN_LINE] = 0,
    [QAL_DEVICE_IN_SPDIF] = 0,
    [QAL_DEVICE_IN_PROXY] = 0,
};
#if 0
static int pcm_device_table[3][2] = {
    [0] = {0,0},
    [QAL_STREAM_PLAYBACK_LOW_LATENCY] = {LOWLATENCY_PCM_DEVICE,
                                         LOWLATENCY_PCM_DEVICE},
    [QAL_STREAM_PLAYBACK_DEEP_BUFFER] = {DEEP_BUFFER_PCM_DEVICE,
                                         DEEP_BUFFER_PCM_DEVICE},
};
#endif

// To be defined in detail
static const char * device_table[QAL_DEVICE_MAX] = {
    [0] = "",
    [QAL_DEVICE_NONE] = "none",
    [QAL_DEVICE_OUT_EARPIECE] = "handset",
    [QAL_DEVICE_OUT_SPEAKER] = "speaker",
    [QAL_DEVICE_OUT_WIRED_HEADSET] = "",
    [QAL_DEVICE_OUT_WIRED_HEADPHONE] = "",
    [QAL_DEVICE_OUT_LINE] = "",
    [QAL_DEVICE_OUT_BLUETOOTH_SCO] = "",
    [QAL_DEVICE_OUT_BLUETOOTH_A2DP] = "",
    [QAL_DEVICE_OUT_AUX_DIGITAL] = "",
    [QAL_DEVICE_OUT_HDMI] = "",
    [QAL_DEVICE_OUT_USB_DEVICE] = "",
    [QAL_DEVICE_OUT_USB_HEADSET] = "",
    [QAL_DEVICE_OUT_SPDIF] = "",
    [QAL_DEVICE_OUT_FM] = "",
    [QAL_DEVICE_OUT_AUX_LINE] = "",
    [QAL_DEVICE_OUT_PROXY] = "",

    [QAL_DEVICE_IN_HANDSET_MIC] = "handset-mic",
    [QAL_DEVICE_IN_SPEAKER_MIC] = "speaker-mic",
    [QAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET] = "",
    [QAL_DEVICE_IN_WIRED_HEADSET] = "",
    [QAL_DEVICE_IN_AUX_DIGITAL] = "",
    [QAL_DEVICE_IN_HDMI] = "",
    [QAL_DEVICE_IN_USB_ACCESSORY] = "",
    [QAL_DEVICE_IN_USB_DEVICE] = "",
    [QAL_DEVICE_IN_USB_HEADSET] = "",
    [QAL_DEVICE_IN_FM_TUNER] = "",
    [QAL_DEVICE_IN_LINE] = "",
    [QAL_DEVICE_IN_SPDIF] = "",
    [QAL_DEVICE_IN_PROXY] = "",
};

std::shared_ptr<ResourceManager> ResourceManager::rm = nullptr;

ResourceManager::ResourceManager()
{
}

ResourceManager::~ResourceManager()
{
    
}

void ResourceManager::init_audio()
{
    int snd_card_num = 0;
    char *snd_card_name = NULL, *snd_card_name_t = NULL;
    char *snd_internal_name = NULL;
    char *tmp = NULL;
    char mixer_xml_file[MIXER_PATH_MAX_LENGTH] = {0};
    while (snd_card_num < MAX_SND_CARD)
    {
        rm->audio_mixer = mixer_open(snd_card_num);
        if (!rm->audio_mixer)
        {
            snd_card_num++;
            continue;
        } else
            break;
    }

    if (snd_card_num >= MAX_SND_CARD)
    {
        QAL_ERR(LOG_TAG,"%s: audio mixer open failure", __func__);
        mixer_close(rm->audio_mixer);
        return;
    }

    snd_card_name = strdup(mixer_get_name(rm->audio_mixer));
    if (!snd_card_name)
    {
        QAL_ERR(LOG_TAG,"%s: failed to allocate memory for snd_card_name\n", __func__);
        mixer_close(rm->audio_mixer);
        return;
    }

    snd_card_name_t = strdup(snd_card_name);
    snd_internal_name = strtok_r(snd_card_name_t, "-", &tmp);

    if (snd_internal_name != NULL)
        snd_internal_name = strtok_r(NULL, "-", &tmp);

    if (snd_internal_name != NULL)
    {
        strncpy(mixer_xml_file, MIXER_XML_BASE_STRING, MIXER_PATH_MAX_LENGTH);
        strncat(mixer_xml_file, MIXER_FILE_DELIMITER, MIXER_PATH_MAX_LENGTH);
        strncat(mixer_xml_file, snd_internal_name, MIXER_PATH_MAX_LENGTH);
        strncat(mixer_xml_file, MIXER_FILE_EXT, MIXER_PATH_MAX_LENGTH);
    } else
        strncpy(mixer_xml_file, MIXER_XML_DEFAULT_PATH, MIXER_PATH_MAX_LENGTH);

    rm->audio_route = audio_route_init(snd_card_num, mixer_xml_file);
    QAL_INFO(LOG_TAG,"audio route %p, mixer path %s", rm->audio_route, mixer_xml_file);
    if (!rm->audio_route)
    {
        QAL_ERR(LOG_TAG,"%s: audio route init failure", __func__);
        mixer_close(rm->audio_mixer);
        snd_card_num++;
        if (snd_card_name)
            free(snd_card_name);
        return;
    }
    // audio_route init success
    QAL_DBG(LOG_TAG,"%s: audio route init success with card %d", __func__, snd_card_num);
    rm->snd_card = snd_card_num;
}

int ResourceManager::init()
{
    if (rm == nullptr)
    {
        std::shared_ptr<ResourceManager> sp(new ResourceManager());
        rm = sp;
    }
    int ret = 0;
    // set bOverwriteFlag to true by default
    // should we add api for client to set this value?
    rm->bOverwriteFlag = true;
    rm->snd_card = -1;

    // Parse resource_manager.xml file
    // Parse audio_policy.conf and platform_info.xml

    /*Skip the file parsing for now*/

    // Init audio_route and audio_mixer
    rm->init_audio();
    
    //#ifdef CONFIG_GSL
    SessionGsl::init(DEFAULT_ACDB_FILES);
    return ret;
}

bool ResourceManager::isStreamSupported(struct qal_stream_attributes *attributes, struct qal_device *devices, int no_of_devices)
{
    bool result = false;
    uint16_t channels, dev_channels;
    uint32_t samplerate, bitwidth;
    uint32_t dev_samplerate, dev_bitwidth;
    size_t cur_sessions, max_sessions;
    qal_audio_fmt_t format, dev_format;

    // check if stream type is supported
    // and new stream session is allowed
    qal_stream_type_t type = attributes->type;
    switch (type)
    {
        case QAL_STREAM_PLAYBACK_LOW_LATENCY:
           cur_sessions = active_streams_ll.size();
           max_sessions = MAX_SESSIONS_LOW_LATENCY;
           break;
        case QAL_STREAM_PLAYBACK_DEEP_BUFFER:
           cur_sessions = active_streams_db.size();
           max_sessions = MAX_SESSIONS_DEEP_BUFFER;
           break;
        case QAL_STREAM_PLAYBACK_COMPRESSED:
           cur_sessions = active_streams_comp.size();
           max_sessions = MAX_SESSIONS_COMPRESSED;
           break;
        case QAL_STREAM_PLAYBACK_VOIP:
        case QAL_STREAM_PLAYBACK_VOICE_CALL_MUSIC:
           break;
        case QAL_STREAM_PLAYBACK_GENERIC:
           cur_sessions = active_streams_ulla.size();
           max_sessions = MAX_SESSIONS_GENERIC;
           break;
        case QAL_STREAM_CAPTURE_LOW_LATENCY:
        case QAL_STREAM_CAPTURE_DEEP_BUFFER:
        case QAL_STREAM_CAPTURE_COMPRESSED:
        case QAL_STREAM_CAPTURE_RAW:
        case QAL_STREAM_CAPTURE_VOIP:
        case QAL_STREAM_CAPTURE_VOICE_ACTIVATION:
        case QAL_STREAM_CAPTURE_VOICE_CALL_RX:
        case QAL_STREAM_CAPTURE_VOICE_CALL_TX:
        case QAL_STREAM_CAPTURE_VOICE_CALL_RX_TX:
        case QAL_STREAM_CAPTURE_GENERIC:
        case QAL_STREAM_VOICE_CALL:
        case QAL_STREAM_LOOPBACK:
        case QAL_STREAM_TRANSCODE:
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: Invalid stream type = %d", __func__, type);
            return result;
    }

    if (cur_sessions == max_sessions) {
        QAL_ERR(LOG_TAG,"%s: no new session allowed for stream %d", __func__, type);
        return result;
    }
    // check if param supported by audio configruation
    switch (attributes->direction)
    {
        case QAL_AUDIO_OUTPUT:
            channels = attributes->out_media_config.ch_info->channels;
            samplerate = attributes->out_media_config.sample_rate;
            bitwidth = attributes->out_media_config.bit_width;
            format = attributes->out_media_config.aud_fmt_id;
            if (channels != DEFAULT_CHANNELS ||
                samplerate != DEFAULT_SAMPLE_RATE ||
                bitwidth != DEFAULT_BIT_WIDTH ||
                format != DEFAULT_FORMAT)
            {
                QAL_ERR(LOG_TAG,"%s: Stream output attributes not supported", __func__);
                return result;
            }
            break;
        case QAL_AUDIO_INPUT:
            channels = attributes->in_media_config.ch_info->channels;
            samplerate = attributes->in_media_config.sample_rate;
            bitwidth = attributes->in_media_config.bit_width;
            format = attributes->in_media_config.aud_fmt_id;
            if (channels != DEFAULT_CHANNELS ||
                samplerate != DEFAULT_SAMPLE_RATE ||
                bitwidth != DEFAULT_BIT_WIDTH ||
                format != DEFAULT_FORMAT)
            {
                QAL_ERR(LOG_TAG,"%s: Stream input attributes not supported", __func__);
                return result;
            }
            break;
        case QAL_AUDIO_OUTPUT | QAL_AUDIO_INPUT:
            break;
        default:
            return result;
    }

    // check if param supported by any of the devices
    for (int i = 0; i < no_of_devices; i++)
    {
        dev_channels = devices[i].config.ch_info->channels;
        dev_samplerate = devices[i].config.sample_rate;
        dev_bitwidth = devices[i].config.bit_width;
        //dev_format = devices[i].config.aud_fmt_id;
        if (channels == dev_channels &&
            samplerate == dev_samplerate &&
            bitwidth == dev_bitwidth/* &&
            format == dev_format*/)
        {
            result = true;
            break;
        }
    }
    result = true;

    if (!result)
    {
        QAL_ERR(LOG_TAG,"%s: Attributes not supported by devices", __func__);
    }
    return result;
}

template <class T>
int registerstream(T s, std::vector<T> streams)
{
    int ret = 0;
    streams.push_back(s);
    return ret;
}

int ResourceManager::registerStream(Stream *s)
{
    int ret = 0;
    qal_stream_type_t type;
    ret = s->getStreamType(&type);
    if (ret)
    {
        QAL_ERR(LOG_TAG,"%s: getStreamType failed with status = %d", __func__, ret);
        return ret;
    }

    mutex.lock();
    switch (type)
    {
        case QAL_STREAM_PLAYBACK_LOW_LATENCY:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sPCM, active_streams_ll);
            break;
        }
        case QAL_STREAM_PLAYBACK_DEEP_BUFFER:
        {
            StreamPCM* sDB = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sDB, active_streams_db);
            break;
        }
        case QAL_STREAM_PLAYBACK_COMPRESSED:
        {
            StreamCompress* sComp = dynamic_cast<StreamCompress*>(s);
            ret = registerstream(sComp, active_streams_comp);
            break;
        }
        case QAL_STREAM_PLAYBACK_GENERIC:
        {
            StreamPCM* sULLA = dynamic_cast<StreamPCM*>(s);
            ret = registerstream(sULLA, active_streams_ulla);
            break;
        }
        default:
        {
            QAL_ERR(LOG_TAG,"%s: Invalid stream type = %d", __func__, type);
            ret = -EINVAL;
            break;
        }
    }
    mutex.unlock();
    return ret;
}

// template function to deregister stream
template <class T>
int deregisterstream(T s, std::vector<T> streams)
{
    int ret = 0;
    typename std::vector<T>::iterator iter = std::find(streams.begin(), streams.end(), s);
    if (iter != streams.end())
        streams.erase(iter);
    else
        ret = -ENOENT;
    return ret;
}

int ResourceManager::deregisterStream(Stream *s)
{
    int ret = 0;
    qal_stream_type_t type;
    ret = s->getStreamType(&type);
    if (ret)
    {
        QAL_ERR(LOG_TAG,"%s: getStreamType failed with status = %d", __func__, ret);
        return ret;
    }

    mutex.lock();
    switch (type)
    {
        case QAL_STREAM_PLAYBACK_LOW_LATENCY:
        {
            StreamPCM* sPCM = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sPCM, active_streams_ll);
            break;
        }
        case QAL_STREAM_PLAYBACK_DEEP_BUFFER:
        {
            StreamPCM* sDB = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sDB, active_streams_db);
            break;
        }
        case QAL_STREAM_PLAYBACK_COMPRESSED:
        {
            StreamCompress* sComp = dynamic_cast<StreamCompress*>(s);
            ret = deregisterstream(sComp, active_streams_comp);
            break;
        }
        case QAL_STREAM_PLAYBACK_GENERIC:
        {
            StreamPCM* sULLA = dynamic_cast<StreamPCM*>(s);
            ret = deregisterstream(sULLA, active_streams_ulla);
            break;
        }
        default:
        {
            QAL_ERR(LOG_TAG,"%s: Invalid stream type = %d", __func__, type);
            ret = -EINVAL;
            break;
        }
    }
    mutex.unlock();
    return ret;
}

int ResourceManager::registerDevice(std::shared_ptr<Device> d)
{
    mutex.lock();
    active_devices.push_back(d);
    mutex.unlock();
    return 0;
}

int ResourceManager::deregisterDevice(std::shared_ptr<Device> d)
{
    int ret = 0;
    mutex.lock();
    typename std::vector<std::shared_ptr<Device>>::iterator iter =
        std::find(active_devices.begin(), active_devices.end(), d);
    if (iter != active_devices.end())
        active_devices.erase(iter);
    else
    {
        QAL_ERR(LOG_TAG,"%s: no device %d found in active device list", __func__, d->getDeviceId());
        ret = -ENOENT;
    }
    mutex.unlock();
    return ret;
}

int ResourceManager::getaudioroute(struct audio_route** ar)
{
    if (audio_route)
    {
        *ar = audio_route;
        QAL_ERR(LOG_TAG,"%s:ar %p  audio_route %p", __func__,ar, audio_route);
        return 0;
    }
    else
    {
        QAL_ERR(LOG_TAG,"%s: no audio route found", __func__);
        return -ENOENT;
    }
}

int ResourceManager::getaudiomixer(struct audio_mixer * am)
{
    if (audio_mixer)
    {
        am = audio_mixer;
        return 0;
    }
    else
    {
        QAL_ERR(LOG_TAG,"%s: no audio mixer found", __func__);
        return -ENOENT;
    }
}

template <class T>
void getActiveStreams(std::shared_ptr<Device> d, std::vector<Stream*> activestreams, std::vector<T> sourcestreams)
{
    for(typename std::vector<T>::iterator iter = sourcestreams.begin(); iter != sourcestreams.end(); iter++)
    {
        std::vector <std::shared_ptr<Device>> devices;
        (*iter)->getAssociatedDevices(devices);
        typename std::vector<std::shared_ptr<Device>>::iterator result = std::find(devices.begin(), devices.end(), d);
        if (result != devices.end())
            activestreams.push_back(*iter);
    }
}

int ResourceManager::getactivestreams(std::shared_ptr<Device> d, std::vector<Stream*> activestreams)
{
    int ret = 0;
    mutex.lock();
    // merge all types of active streams into activestreams
    getActiveStreams(d, activestreams, active_streams_ll);
    getActiveStreams(d, activestreams, active_streams_ulla);
    getActiveStreams(d, activestreams, active_streams_db);
    getActiveStreams(d, activestreams, active_streams_comp);

    if (activestreams.empty())
    {
        QAL_ERR(LOG_TAG,"%s: no active streams found for device %d", __func__, d->getDeviceId());
        ret = -ENOENT;
    }
    mutex.unlock();
    return ret;
}

/*blsUpdated - to specify if the config is updated by rm*/
int ResourceManager::checkAndGetDeviceConfig(struct qal_device *device, bool* blsUpdated)
{
    // check if device config is supported
    bool dev_supported = false;
    *blsUpdated = false;
    int ret = -EINVAL;;
    uint16_t channels = device->config.ch_info->channels;
    uint32_t samplerate = device->config.sample_rate;
    uint32_t bitwidth = device->config.bit_width;

    // check and rewrite params if needed
    // only compare with default value for now
    // because no config file parsed in init
    if (channels != DEFAULT_CHANNELS)
    {
        if (bOverwriteFlag)
        {
            device->config.ch_info->channels = DEFAULT_CHANNELS;
            *blsUpdated = true;
        }
    }
    else if (samplerate != DEFAULT_SAMPLE_RATE)
    {
        if (bOverwriteFlag)
        {
            device->config.sample_rate = DEFAULT_SAMPLE_RATE;
            *blsUpdated = true;
        }
    }
    else if (bitwidth != DEFAULT_BIT_WIDTH)
    {
        if (bOverwriteFlag)
        {
            device->config.bit_width = DEFAULT_BIT_WIDTH;
            *blsUpdated = true;
        }
    }
    else
    {
        ret = 0;
        dev_supported = true;
    }

    return ret;
}

std::shared_ptr<ResourceManager> ResourceManager::getInstance()
{
    if (!rm)
        init();
    return rm;
}

int ResourceManager::getSndCard()
{
    return snd_card;
}

int ResourceManager::getDeviceName(int deviceId, char *device_name)
{
    if (deviceId >= QAL_DEVICE_OUT_EARPIECE && deviceId <= QAL_DEVICE_IN_PROXY)
    {
        strncpy(device_name, device_table[deviceId], DEVICE_NAME_MAX_SIZE);
    }
    else
    {
        strncpy(device_name, "", DEVICE_NAME_MAX_SIZE);
        QAL_ERR(LOG_TAG,"%s: Invalide device id %d", __func__, deviceId);
        return -EINVAL;
    }
    return 0;
}

// TODO: Should pcm device be related to usecases used(ll/db/comp/ulla)?
// Use Low Latency as default by now
int ResourceManager::getPcmDeviceId(int deviceId)
{
    int pcm_device_id = -1;
    if (deviceId < QAL_DEVICE_OUT_EARPIECE || deviceId > QAL_DEVICE_IN_PROXY)
    {
        QAL_ERR(LOG_TAG,"%s: Invalide device id %d", __func__, deviceId);
        return -EINVAL;
    }

    pcm_device_id = pcm_device_table[deviceId];
//#else
#if 0
    qal_stream_type_t stream = QAL_STREAM_PLAYBACK_LOW_LATENCY;
    if (dev_id >= QAL_DEVICE_OUT_EARPIECE && dev_id <= QAL_DEVICE_OUT_PROXY)
    {
        pcm_device_id = pcm_device_table[stream][0];
    }
    else if (dev_id >= QAL_DEVICE_IN_HANDSET_MIC && dev_id <= QAL_DEVICE_IN_PROXY)
    {
        pcm_device_id = pcm_device_table[stream][1];
    }
#endif

    return pcm_device_id;
}

void ResourceManager::deinit()
{
    QAL_VERBOSE(LOG_TAG,"deinit called\n");
    SessionGsl::deinit();
}
