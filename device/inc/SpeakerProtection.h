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

#ifndef SPEAKER_PROT
#define SPEAKER_PROT

#include "Speaker.h"
#include "sp_vi.h"
#include <tinyalsa/asoundlib.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include<vector>

class Speaker;

typedef enum speaker_prot_cal_state {
    SPKR_NOT_CALIBRATED,     /* Speaker not calibrated  */
    SPKR_CALIBRATED,         /* Speaker calibrated  */
    SPKR_CALIB_IN_PROGRESS,  /* Speaker calibration in progress  */
}spkr_prot_cal_state;

typedef enum speaker_prot_proc_state {
    SPKR_PROCESSING_IN_IDLE,     /* Processing mode in idle state */
    SPKR_PROCESSING_IN_PROGRESS, /* Processing mode in running state */
}spkr_prot_proc_state;

enum {
    WSA_SPKR_LEFT = 0, /* Left Speaker */
    WSA_SPKR_RIGHT,    /* Right Speaker */
};

enum {
    TKV,
    CKV,
};

struct agmMetaData {
    uint8_t *buf;
    uint32_t size;
    agmMetaData(uint8_t *b, uint32_t s)
        :buf(b),size(s) {}
};

class SpeakerProtection : public Speaker
{
protected :
    bool spkrProtEnable;
    bool threadExit;
    bool calThrdCreated;
    bool triggerCal;
    int minIdleTime;
    static speaker_prot_cal_state spkrCalState;
    spkr_prot_proc_state spkrProcessingState;
    int *spkerTempList;
    static bool isSpkrInUse;
    static struct timespec spkrLastTimeUsed;
    static struct mixer *mixer;
    static int totalSpeakers;
    static struct pcm *rxPcm;
    static struct pcm *txPcm;
    static int numberOfChannels;
    static bool mDspCallbackRcvd;
    static param_id_sp_th_vi_calib_res_cfg_t *callback_data;

private :

public:
    static std::thread mCalThread;
    static std::condition_variable cv;
    static std::mutex cvMutex;
    void spkrCalibrationThread();
    int getSpeakerTemperature(int spkr_pos);
    void spkrCalibrateWait();
    int spkrStartCalibration();
    void speakerProtectionInit();
    void speakerProtectionDeinit();
    void getSpeakerTemperatureList();
    static void spkrProtSetSpkrStatus(bool enable);
    static int setConfig(int type, int tag, int tagValue, int devId, const char *aif);
    bool isSpeakerInUse(unsigned long *sec);

    SpeakerProtection(struct qal_device *device,
                      std::shared_ptr<ResourceManager> Rm);

    ~SpeakerProtection();

    int32_t spkrProtProcessingMode(bool flag);
    static int32_t spkrProtSetR0T0Value(vi_r0t0_cfg_t r0t0Array[]);
    static void mixer_ctl_callback (void *hdl, uint32_t event_id, void *event_data,
                             uint32_t event_size);

};

#endif
