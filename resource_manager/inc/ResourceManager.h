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
#include "audio_route/audio_route.h"
#include <tinyalsa/asoundlib.h>
#include "QalCommon.h"

#define audio_mixer mixer

class Device;
class Stream;
class StreamPCM;
class StreamCompress;


class ResourceManager
{
protected:
 std::vector <StreamPCM*> active_streams_ll;
 std::vector <StreamPCM*> active_streams_ulla;
 std::vector <StreamPCM*> active_streams_db;
 std::vector <StreamCompress*> active_streams_comp;
 std::vector <std::shared_ptr<Device>> active_devices;
 bool bOverwriteFlag;
 std::mutex mutex;
 int snd_card;
 static std::shared_ptr<ResourceManager> rm;
 struct audio_route* audio_route = NULL;
 struct audio_mixer* audio_mixer;
 
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
 int getSndCard();
 int getPcmDeviceId(int deviceId);
 int getaudioroute(struct audio_route** ar);
 int getaudiomixer(struct audio_mixer *am);
 int getactivestreams(std::shared_ptr<Device> d, std::vector<Stream*> activestreams);
 int getDeviceName(int deviceId, char *device_name);
};

#endif
