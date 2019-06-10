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

#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H
#include <algorithm>
#include <vector>
#include <memory>
#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include "audio_route/audio_route.h"
#include <tinyalsa/asoundlib.h>
#include "QalCommon.h"
#include <map>
#include <expat.h>

#define audio_mixer mixer

class Device;
class Stream;
class StreamPCM;
class StreamCompress;
class StreamSoundTrigger;
class SoundTriggerEngine;

class ResourceManager
{
protected:
 std::vector <StreamPCM*> active_streams_ll;
 std::vector <StreamPCM*> active_streams_ulla;
 std::vector <StreamPCM*> active_streams_db;
 std::vector <StreamCompress*> active_streams_comp;
 std::vector <StreamSoundTrigger*> active_streams_st;
 std::vector <SoundTriggerEngine*> active_engines_st;
 std::vector <std::shared_ptr<Device>> active_devices;
 bool bOverwriteFlag;
 std::mutex mutex;
 int snd_card;
 static std::shared_ptr<ResourceManager> rm;
 struct audio_route* audio_route = NULL;
 struct audio_mixer* audio_mixer;
 static std::vector <int> streamTag;
 static std::vector <int> streamPpTag;
 static std::vector <int> mixerTag;
 static std::vector <int> devicePpTag;
 static std::vector <int> deviceTag;
 static std::vector<std::pair<int32_t, int32_t>> devicePcmId;
 static std::vector<std::pair<int32_t, std::string>> deviceLinkName;
 static std::vector<std::pair<int32_t, std::string>> sndDeviceNameLUT;
 ResourceManager();
public:
 ~ResourceManager();
 /* checks config for both stream and device */
 bool isStreamSupported(struct qal_stream_attributes *attributes, struct qal_device *devices, int no_of_devices);
 int registerStream(Stream *s);
 int deregisterStream(Stream *s);
 int registerDevice(std::shared_ptr<Device> d);
 int deregisterDevice(std::shared_ptr<Device> d);
 /* bIsUpdated - to specify if the config is updated by rm */
 int checkAndGetDeviceConfig(struct qal_device *device ,bool* bIsUpdated);
 static void init_audio();
 static int init();
 static void deinit();
 static std::shared_ptr<ResourceManager> getInstance();
 static int XmlParser(std::string xmlFile);
 static void updatePcmId(int32_t deviceId, int32_t pcmId);
 static void updateLinkName(int32_t deviceId, std::string linkName);
 static void updateSndName(int32_t deviceId, std::string sndName);
 static void updateStreamTag(int32_t tagId);
 static void updateDeviceTag(int32_t tagId);
 int getSndCard();
 int getPcmDeviceId(int deviceId);
 int getaudioroute(struct audio_route** ar);
 int getaudiomixer(struct audio_mixer *am);
 int getactivestreams(std::shared_ptr<Device> d, std::vector<Stream*> &activestreams);
 int getDeviceName(int deviceId, char *device_name);
 int getDeviceEpName(int deviceId, std::string &epName);
 int getStreamTag(std::vector <int> &tag);
 int getDeviceTag(std::vector <int> &tag);
 int getMixerTag(std::vector <int> &tag);
 int getStreamPpTag(std::vector <int> &tag);
 int getDevicePpTag(std::vector <int> &tag);

 static void endTag(void *userdata __unused, const XML_Char *tag_name);
 static void processDeviceInfo(const XML_Char **attr);
 static void processTagInfo(const XML_Char **attr);
 static void startTag(void *userdata __unused, const XML_Char *tag_name, const XML_Char **attr);
};

#endif
