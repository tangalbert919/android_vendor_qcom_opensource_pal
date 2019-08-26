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


#include "SessionQts.h"
#include <errno.h>
#define QTS_LIB  "libqts.so"
#define LOG_TAG "SessionQts"

void *SessionQts::qtsLibHandle = NULL;

typedef int32_t (*qts_init_t)(void);
typedef void (*qts_deinit_t)(void);
qts_init_t qtsInit;
qts_deinit_t qtsDeinit;

int SessionQts::init()
{
    int ret = 0;
    QAL_DBG(LOG_TAG, "Enter.");
    if(!qtsLibHandle) {
        qtsLibHandle = dlopen(QTS_LIB, RTLD_NOW);
        if (NULL == qtsLibHandle) {
            const char *err_str = dlerror();
            QAL_ERR(LOG_TAG, "DLOPEN failed for %s, %s",
                  QTS_LIB, err_str?err_str:"unknown");
            return -EINVAL;
        }

    }

    /*loading the qts function symbols*/
    qtsInit = (qts_init_t)dlsym(qtsLibHandle, "qts_init");
    if (!qtsInit) {
        QAL_ERR(LOG_TAG, "dlsym error %s for qts_init", dlerror());
        ret = -EINVAL;
        goto error;
    }
    qtsDeinit = (qts_deinit_t)dlsym(qtsLibHandle, "qts_deinit");
    if (!qtsDeinit) {
        QAL_ERR(LOG_TAG, "dlsym error %s for qts_deinit", dlerror());
        ret = -EINVAL;
        goto error;
    }

    QAL_INFO(LOG_TAG, "Initializing QTS...");
    ret = qtsInit();
    if (0 != ret) {
        QAL_ERR(LOG_TAG, "qts init failed with err = %d", ret);
        goto error;
    }
    goto exit;
error:
    dlclose(qtsLibHandle);
    qtsLibHandle = NULL;
exit:
    return ret;
}

void SessionQts::deinit()
{
    QAL_INFO(LOG_TAG, "Deinitializing QTS...");
    if(NULL != qtsLibHandle){
        qtsDeinit();
        dlclose(qtsLibHandle);
    }
}
