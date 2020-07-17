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

#ifndef STREAM_H_
#define STREAM_H_

#include "QalDefs.h"
#include <algorithm>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <memory>
#include <mutex>
#include <exception>
#include <errno.h>
#include "QalCommon.h"

typedef enum {
    DATA_MODE_SHMEM = 0,
    DATA_MODE_BLOCKING ,
    DATA_MODE_NONBLOCKING
} dataFlags;

typedef enum {
    STREAM_IDLE = 0,
    STREAM_INIT,
    STREAM_OPENED,
    STREAM_STARTED,
    STREAM_PAUSED,
    STREAM_STOPPED
} stream_state_t;

#define BUF_SIZE_PLAYBACK 1024
#define BUF_SIZE_CAPTURE 960
#define NO_OF_BUF 4
#define MUTE_TAG 0
#define UNMUTE_TAG 1
#define PAUSE_TAG 2
#define RESUME_TAG 3
#define MFC_SR_8K 4
#define MFC_SR_16K 5
#define MFC_SR_32K 6
#define MFC_SR_44K 7
#define MFC_SR_48K 8
#define MFC_SR_96K 9
#define MFC_SR_192K 10
#define MFC_SR_384K 11
#define FLUENCE_ON_TAG 12
#define FLUENCE_OFF_TAG 13
#define FLUENCE_EC_TAG 14
#define FLUENCE_NS_TAG 15
#define CHS_1 16
#define CHS_2 17
#define CHS_3 18
#define CHS_4 19
#define CHS_5 20
#define CHS_6 21
#define CHS_7 22
#define CHS_8 23
#define BW_16 24
#define BW_24 25
#define BW_32 26
#define TTY_MODE 27
#define VOICE_SLOW_TALK_OFF 28
#define VOICE_SLOW_TALK_ON 29
#define VOICE_VOLUME_BOOST 30
#define OP_MODE 31
#define SPKR_PROT_ENABLED 32
#define SPKR_PROT_DISABLED 33

#define INCALL_RECORD_UPLINK 34
#define INCALL_RECORD_DOWNLINK 35
#define INCALL_RECORD_UPLINK_DOWNLINK_MONO 36
#define INCALL_RECORD_UPLINK_DOWNLINK_STEREO 37
/* This sleep is added to give time to kernel and
 * spf to recover from SSR so that audio-hal will
 * not continously try to open a session if it fails
 * during SSR.
 */
#define SSR_RECOVERY 50000

/* Soft pause has to wait for ramp period to ensure volume stepping finishes.
 * This period of time was previously consumed in elite before acknowleging
 * pause completion. But it's not the case in Gecko.
 * FIXME: load the ramp period config from acdb.
 */
#define VOLUME_RAMP_PERIOD (40*1000)

using KeyVect_t = std::vector<std::pair<uint32_t, uint32_t>>;
class Device;
class ResourceManager;
class Session;

class Stream
{
protected:
    uint32_t mNoOfDevices;
    std::vector <std::shared_ptr<Device>> mDevices;
    static struct qal_device* mQalDevice;
    Session* session;
    struct qal_stream_attributes* mStreamAttr;
    struct qal_volume_data* mVolumeData = NULL;
    std::mutex mStreamMutex;
    static std::mutex mBaseStreamMutex; //TBD change this. as having a single static mutex for all instances of Stream is incorrect. Replace
    static std::shared_ptr<ResourceManager> rm;
    struct modifier_kv *mModifiers;
    uint32_t mNoOfModifiers;
    KeyVect_t mDevPpModifiers;
    KeyVect_t mStreamModifiers;
    size_t inBufSize;
    size_t outBufSize;
    size_t inBufCount;
    size_t outBufCount;
    bool standBy = false;
    bool ssrDone = true;
    stream_state_t currentState;
    stream_state_t cachedState;
    uint32_t mInstanceID = 0; // used for Voice UI only now
public:
    virtual ~Stream() {};
    qal_stream_callback streamCb;
    void *cookie;
    bool isPaused = false;
    bool a2dp_compress_mute = false;  /* TODO : Check if this can be removed */
    qal_device_id_t suspendedDevId = QAL_DEVICE_NONE;
    virtual int32_t open() = 0;
    virtual int32_t close() = 0;
    virtual int32_t start() = 0;
    virtual int32_t stop() = 0;
    virtual int32_t prepare() = 0;
    virtual int32_t pause() {return 0;}
    virtual int32_t resume() {return 0;}
    virtual int32_t drain(qal_drain_type_t type __unused) {return 0;}
    virtual int32_t setStreamAttributes(struct qal_stream_attributes *sattr) = 0;
    virtual int32_t setVolume( struct qal_volume_data *volume) = 0;
    virtual int32_t setMute( bool state) = 0;
    virtual int32_t setPause() = 0;
    virtual int32_t setResume()= 0;
    virtual int32_t flush() {return 0;}
    virtual int32_t read(struct qal_buffer *buf) = 0;
    virtual int32_t standby() {return 0;};

    virtual int32_t addRemoveEffect(qal_audio_effect_t effect, bool enable) = 0; //TBD: make this non virtual and prrovide implementation as StreamPCM and StreamCompressed are doing the same things
    virtual int32_t setParameters(uint32_t param_id, void *payload) = 0;
    virtual int32_t write(struct qal_buffer *buf) = 0; //TBD: make this non virtual and prrovide implementation as StreamPCM and StreamCompressed are doing the same things
    virtual int32_t registerCallBack(qal_stream_callback cb, void *cookie) = 0;
    virtual int32_t getCallBack(qal_stream_callback *cb) = 0;
    virtual int32_t getParameters(uint32_t param_id, void **payload) = 0;
    virtual int32_t setECRef(std::shared_ptr<Device> dev, bool is_enable) = 0;
    virtual int32_t setECRef_l(std::shared_ptr<Device> dev, bool is_enable) = 0;
    virtual int32_t ssrDownHandler() = 0;
    virtual int32_t ssrUpHandler() = 0;
    virtual int32_t createMmapBuffer(int32_t min_size_frames __unused,
                                   struct qal_mmap_buffer *info __unused) {return -EINVAL;}
    virtual int32_t GetMmapPosition(struct qal_mmap_position *position __unused) {return -EINVAL;}

    int32_t getStreamAttributes(struct qal_stream_attributes *sattr);
    int32_t getModifiers(struct modifier_kv *modifiers,uint32_t *noOfModifiers);
    const KeyVect_t& getDevPpModifiers() const;
    const KeyVect_t& getStreamModifiers() const;
    int32_t getStreamType(qal_stream_type_t* streamType);
    int32_t getStreamDirection(qal_stream_direction_t *dir);
    int32_t getAssociatedDevices(std::vector <std::shared_ptr<Device>> &adevices);
    int32_t getAssociatedSession(Session** session);
    int32_t setBufInfo(size_t *in_buf_size, size_t in_buf_count,
                       size_t *out_buf_size, size_t out_buf_count);
    int32_t getBufInfo(size_t *in_buf_size, size_t *in_buf_count,
                       size_t *out_buf_size, size_t *out_buf_count);
    int32_t getVolumeData(struct qal_volume_data *vData);
    /* static so that this method can be accessed wihtout object */
    static Stream* create(struct qal_stream_attributes *sattr, struct qal_device *dattr,
         uint32_t no_of_devices, struct modifier_kv *modifiers, uint32_t no_of_modifiers);
    bool isStreamAudioOutFmtSupported(qal_audio_fmt_t format);
    int32_t getTimestamp(struct qal_session_time *stime);
    int disconnectStreamDevice(Stream* streamHandle,  qal_device_id_t dev_id);
    int disconnectStreamDevice_l(Stream* streamHandle,  qal_device_id_t dev_id);
    int connectStreamDevice(Stream* streamHandle, struct qal_device *dattr);
    int connectStreamDevice_l(Stream* streamHandle, struct qal_device *dattr);
    int switchDevice(Stream* streamHandle, uint32_t no_of_devices, struct qal_device *deviceArray);
    bool isGKVMatch(qal_key_vector_t* gkv);
    int32_t getEffectParameters(void *effect_query, size_t *payload_size);
    uint32_t getInstanceId() { return mInstanceID; }
    bool checkStreamMatch(qal_device_id_t qal_device_id,
                                qal_stream_type_t qal_stream_type);
    int32_t getEffectParameters(void *effect_query);
    bool isActive() { return currentState == STREAM_STARTED; }
};

#endif//STREAM_H_
