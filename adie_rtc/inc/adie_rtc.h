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

#ifndef ADIE_RTC_H
#define ADIE_RTC_H
#include <stdint.h>
#include "casa_osal_log.h"
#include "casa_osal_file_io.h"
enum cdc
{
    MSM8X52 = 1,
    WCD9335 = 2,
    WSA881X_ANALOG = 3,
    WSA881X_SOUNDWIRE = 4,
    WSA881X = 5,
    MSM8909 = 6,
    WCD9330 = 7,
    WCD9326 = 8,
    WCD9320 = 9,
    WCD9310 = 10,
    WCD9306 = 11,
    WCD9302 = 12,
    MSM8X16 = 13,
    MSM8X10 = 14,
    WCD9340 = 16,
    WCD9341 = 17,
    WHS9410 = 21,
    WHS9420 = 22,
    WCD9360 = 24,
    AQT1000 = 25,
    WCD937X = 27,
    WCD938X = 28,
    BOLERO = 26,
    CODEC_UNDEFINED = UInt16.MaxValue - 1
};

struct chipset_id_info {
    char    *name;
    int id;
};

struct adie_rtc_version {
    uint32_t major;
    uint32_t minor;
};

struct adie_rtc_codec_handle {
    uint32_t handle;
    uint32_t chipset_id;
    uint32_t chipset_major_version;
    uint32_t chipset_minor_version;
};

struct adie_rtc_codec_info {
    uint32_t num_of_entries; /**< number of codec entries */
    struct adie_rtc_codec_handle *handle;
    /**< codec entry list (in-band). */
};

struct adie_rtc_register_req {
    uint32_t codec_handle;
    uint32_t register_id;
    uint32_t register_mask;
    uint32_t value;
    /**< value is output in get_register and acts as input in set_register */
};

int32_t adie_rtc_init();
int32_t adie_rtc_deinit();

int32_t adie_rtc_get_version(struct adie_rtc_version *ver);
int32_t adie_rtc_get_codec_info(struct adie_rtc_codec_info *codec_info);
int32_t adie_rtc_get_register(struct adie_rtc_register_req *req);
int32_t adie_rtc_set_register(struct adie_rtc_register_req *req);

#endif

