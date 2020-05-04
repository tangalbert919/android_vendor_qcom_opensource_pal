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

#define LOG_TAG "ADIE_RTC"

#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <limits.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "adie_rtc.h"
#include <unistd.h>


int main()
{
    int val;
    struct adie_rtc_codec_info cdcInfo, cdcInfotest;
    struct adie_rtc_version ver, *verP;
    struct adie_rtc_register_req regReq;
    uint32_t numEntries, status;
    adie_rtc_init() ;
    adie_rtc_get_version(&ver);
    AR_LOG_INFO(LOG_TAG,"major= %d \n & minor= %d",ver.major,ver.minor);
    adie_rtc_get_codec_info(&cdcInfo);
    cdcInfotest.num_of_entries = cdcInfo.num_of_entries;
    cdcInfotest.handle = (struct adie_rtc_codec_handle *)calloc(cdcInfotest.num_of_entries, sizeof(struct adie_rtc_codec_handle));
    status = adie_rtc_get_codec_info(&cdcInfotest) ;
    AR_LOG_INFO(LOG_TAG,"num_of_entries=%d \n & handle=%d \n & chipset_id=%d \n & major_version= %d \n& minor_version=%d \n ",cdcInfotest.num_of_entries,cdcInfotest.handle[0].handle,cdcInfotest.handle[0].chipset_id,cdcInfotest.handle[0].chipset_major_version,cdcInfotest.handle[0].chipset_minor_version);
    regReq.codec_handle = 1;
    regReq.register_id = 0x2604;
    regReq.register_mask = 0xff;
    adie_rtc_get_register(&regReq);
    AR_LOG_INFO(LOG_TAG,"Received value= %x\n ",regReq.value);
    regReq.value = 0x38;
    adie_rtc_set_register(&regReq);
    adie_rtc_get_register(&regReq);
    AR_LOG_INFO(LOG_TAG,"Received value= %x\n ",regReq.value);

}
