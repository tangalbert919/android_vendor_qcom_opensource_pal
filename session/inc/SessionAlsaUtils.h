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

#ifndef SESSION_ALSAUTILS_H
#define SESSION_ALSAUTILS_H

#include "Session.h"
#include "ResourceManager.h"
#include "PayloadBuilder.h"


#include <tinyalsa/asoundlib.h>
#include <sound/asound.h>


class Stream;
class Session;

enum class MixerCtlType: uint32_t {
    MIXER_SET_ID_STRING,
    MIXER_SET_ID_VALUE,
    MIXER_SET_ID_ARRAY,
};

class SessionAlsaUtils
{
private:
    SessionAlsaUtils() {};
public:
    ~SessionAlsaUtils();
    static int setMixerCtlData(struct mixer_ctl *ctl, MixerCtlType id, void *data, int size);
    static int getAgmMetaData(const std::vector <std::pair<int, int>> &kv,
                              const std::vector <std::pair<int, int>> &ckv,
                              struct prop_data *propData, uint32_t &mdSize, uint8_t **data);
    static int getTagMetadata(int32_t tagsent, std::vector <std::pair<int, int>> &tkv, struct agm_tag_config *tagConfig);
    static int getCalMetadata(std::vector <std::pair<int, int>> &ckv, struct agm_cal_config* calConfig);
    static unsigned int bitsToAlsaFormat(unsigned int bits);
    static int open(Stream * s, std::shared_ptr<ResourceManager> rm, const std::vector<int> &DevIds, const std::vector<std::string> &BackEnds);
};

#endif //SESSION_ALSA_UTILS
