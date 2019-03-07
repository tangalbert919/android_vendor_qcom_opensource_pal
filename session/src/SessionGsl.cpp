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

#define LOG_TAG "SessionGsl"

#include "SessionGsl.h"
#include "Stream.h"
#include "QalDefs.h"
#include<algorithm>
#include<vector>
#include<fstream>
#include "PayloadBuilder.h"

#define GSL_LIB  "/usr/lib64/libgsl.so"
#define BUFFER_EOS 1



typedef int32_t (*gsl_init_t)(struct gsl_init_data *);
typedef int32_t (*gsl_open_t)(const struct gsl_key_vector *,const struct gsl_key_vector *, gsl_handle_t *);
typedef int32_t (*gsl_close_t)(gsl_handle_t);
typedef int32_t (*gsl_set_cal_t)(gsl_handle_t , const struct gsl_key_vector *);
typedef int32_t (*gsl_set_config_t)(gsl_handle_t, uint32_t, const struct gsl_key_vector *);
typedef int32_t (*gsl_set_custom_config_t)(gsl_handle_t, const uint8_t *, const size_t);
typedef int32_t (*gsl_get_custom_config_t)(gsl_handle_t, uint8_t *, size_t *);
typedef int32_t (*gsl_ioctl_t)(gsl_handle_t, enum gsl_cmd_id, void *, size_t);
typedef int32_t (*gsl_read_t)(gsl_handle_t, uint32_t, struct gsl_buff *, uint32_t *);
typedef int32_t (*gsl_write_t)(gsl_handle_t, uint32_t, struct gsl_buff *, uint32_t *);
typedef int32_t (*gsl_get_tagged_module_info_t)(const struct gsl_key_vector *, uint32_t, struct gsl_module_id_info **, size_t *);
typedef void (*gsl_deinit_t)(void);

gsl_ioctl_t gslIoctl;
gsl_get_custom_config_t gslGetCustomConfig;
gsl_set_cal_t gslSetCal;
gsl_set_config_t gslSetConfig;
gsl_set_custom_config_t gslSetCustomConfig;
gsl_get_tagged_module_info_t gslGetTaggedModuleInfo;
gsl_read_t gslRead;
gsl_open_t gslOpen;
gsl_close_t gslClose;
gsl_write_t gslWrite;
gsl_init_t gslInit;
gsl_deinit_t gslDeinit;

int SessionGsl::seek = 0;
void *SessionGsl::gslLibHandle = NULL;

SessionGsl::SessionGsl() {
    
}

SessionGsl::~SessionGsl() {

}

int SessionGsl::init(std::string acdbFile) {
    int ret = 0;
    struct gsl_acdb_data_files acdb_files;
    gsl_init_data init_data;
    acdb_files.num_files = 1;
    strncpy(acdb_files.acdbFiles[0].fileName, acdbFile.c_str(),
              acdbFile.size());
    acdb_files.acdbFiles[0].fileNameLen = acdbFile.size();
    init_data.acdb_files = &acdb_files;
    init_data.acdb_delta_file = NULL;
    init_data.acdb_addr = 0x0;
    if(gslLibHandle == NULL) {
        gslLibHandle = dlopen(GSL_LIB, RTLD_NOW);
        if (NULL == gslLibHandle) {
            const char *err_str = dlerror();
            QAL_ERR(LOG_TAG,"%s:: DLOPEN failed for %s, %s", __func__,
                  GSL_LIB, err_str?err_str:"unknown");
            return -EINVAL;
        }

    }

    /*loading the gsl function symbols*/
    gslInit = (gsl_init_t)dlsym(gslLibHandle, "gsl_init");
    if (gslInit == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_init", __func__, dlerror());
        return -EINVAL;
    }
    gslSetCal = (gsl_set_cal_t)dlsym(gslLibHandle, "gsl_set_cal");
    if (gslSetCal == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_cal", __func__, dlerror());
        return -EINVAL;
    }
    gslSetConfig = (gsl_set_config_t)dlsym(gslLibHandle, "gsl_set_config");
    if (gslSetConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_config", __func__, dlerror());
        return -EINVAL;
    }
    gslSetCustomConfig = (gsl_set_custom_config_t)dlsym(gslLibHandle, "gsl_set_custom_config");
    if (gslSetCustomConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_set_custom_config", __func__, dlerror());
        return -EINVAL;
    }
    gslGetCustomConfig = (gsl_get_custom_config_t)dlsym(gslLibHandle, "gsl_get_custom_config");
    if (gslGetCustomConfig == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_get_custom_config", __func__, dlerror());
        return -EINVAL;
    }
    gslIoctl = (gsl_ioctl_t)dlsym(gslLibHandle, "gsl_ioctl");
    if (gslIoctl == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_ioctl", __func__, dlerror());
        return -EINVAL;
    }
    gslGetTaggedModuleInfo = (gsl_get_tagged_module_info_t)dlsym(gslLibHandle, "gsl_get_tagged_module_info");
    if (gslGetTaggedModuleInfo == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_get_tagged_module_info", __func__, dlerror());
        return -EINVAL;
    }
    gslRead = (gsl_read_t)dlsym(gslLibHandle, "gsl_read");
    if (gslRead == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_read", __func__, dlerror());
        return -EINVAL;
    }
    gslOpen = (gsl_open_t)dlsym(gslLibHandle, "gsl_open");
    if (gslOpen == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_open", __func__, dlerror());
        return -EINVAL;
    }
    gslClose = (gsl_close_t)dlsym(gslLibHandle, "gsl_close");
    if (gslClose == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_close", __func__, dlerror());
        return -EINVAL;
    }
    gslWrite = (gsl_write_t)dlsym(gslLibHandle, "gsl_write");
    if (gslWrite == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_write", __func__, dlerror());
        return -EINVAL;
    }
    gslDeinit = (gsl_deinit_t)dlsym(gslLibHandle, "gsl_deinit");
    if (gslDeinit == NULL) {
        QAL_ERR(LOG_TAG,"%s: dlsym error %s for gsl_deinit", __func__, dlerror());
        return -EINVAL;
    }
    ret = gslInit(&init_data);
    if (ret) {
        QAL_ERR(LOG_TAG,"%s: gsl init failed with err = %d", __func__, ret);
        return ret;
    }
    QAL_VERBOSE(LOG_TAG,"%s: gsl init success with acdb file %s", __func__, acdbFile.c_str());
//#endif
    return ret;
}

void SessionGsl::deinit() {
    QAL_ERR(LOG_TAG,"deinit called\n");
    gslDeinit();
    dlclose(gslLibHandle);

}

int populateGkv(Stream *s, struct gsl_key_vector *gkv) {
    //unsigned int c1 = 0x1234;
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);
    /*struct qal_stream_attributes *sAttr = null;
    status = s->getStreamAttributes(sAttr);
    if (0 != status) {
        goto exit;
    }
    std::vector<std::shared_ptr<Device>> associatedDevices;
    status = s->getAssociatedDevices(associatedDevices);
    if (0 != status) {
        goto exit;
    }
    struct modifiers *modifiers = null;
    uint32_t noOfModifiers = 0;
    status = s->getModifiers(modifiers, noOfModifiers);
    if (0 != status) {
        goto exit;
    }*/

    /*std::vector <std::pair<int,int>> keyVector;
    keyVector.push_back(std::make_pair(1,2));
    keyVector.push_back(std::make_pair(3,4));
    auto kvps = new gsl_key_value_pair[keyVector.size()];
    //struct gsl_key_vector *kvs = new gsl_key_vector;
    gkv->num_kvps = keyVector.size();
    gkv->kvp = kvps;
    for (int i=0; i < keyVector.size(); i++) {
        kvps[i].key = keyVector[i].first;
        kvps[i].value = keyVector[i].second;
    }*/
    gkv->num_kvps = 2;

    struct qal_stream_attributes sAttr;
    s->getStreamAttributes(&sAttr);
    if (sAttr.direction == 0x1) {
        gkv->kvp[0].key = 0xA1000000;
        gkv->kvp[0].value = 0xA1000001;
        gkv->kvp[1].key = 0xA2000000;
        gkv->kvp[1].value = 0xA2000001;
    } else if(sAttr.direction == 0x2) {
        gkv->kvp[0].key = 0xA1000000;
        gkv->kvp[0].value = 0xA1000002;
        gkv->kvp[1].key = 0xA3000000;
        gkv->kvp[1].value = 0xA3000001;
    } else {
        gkv->num_kvps = 3;
        gkv->kvp[0].key = 0xA3000000;
        gkv->kvp[0].value = 0xA3000001;
        gkv->kvp[1].key = 0xA1000000;
        gkv->kvp[1].value = 0xA1000003;
        gkv->kvp[2].key = 0xA2000000;
        gkv->kvp[2].value = 0xA2000001;
    }
    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

int populateCkv(Stream *s, struct gsl_key_vector *ckv) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);
    /*std::vector <std::pair<int,int>> keyVector;
    keyVector.push_back(std::make_pair(1,2));
    keyVector.push_back(std::make_pair(3,4));
    auto kvps = new gsl_key_value_pair[keyVector.size()];
    //struct gsl_key_vector *kvs = new gsl_key_vector;
    ckv->num_kvps = keyVector.size();
    ckv->kvp = kvps;

    for (int i=0; i < keyVector.size(); i++) {
        kvps[i].key = keyVector[i].first;
        kvps[i].value = keyVector[i].second;
    }*/
    ckv->num_kvps = 2;
    ckv->kvp[0].key = 0xA1000000;
    ckv->kvp[0].value = 0xA1000002;
    ckv->kvp[1].key = 0xA3000000;
    ckv->kvp[1].value = 0xA3000001;
    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

void printCustomConfig(const uint8_t* payload, size_t size) {
    size_t loop;
    uint32_t *temp = (uint32_t *)payload;
    QAL_VERBOSE(LOG_TAG,"%s: enter size - %d", __func__, size);
    for (loop = 0; loop < size;) {
        QAL_ERR(LOG_TAG,"%0x %0x %0x %0x\n",temp[0+loop],temp[1+loop],temp[2+loop],temp[3+loop]);
        loop = loop + 4;
    }
    QAL_VERBOSE(LOG_TAG,"%s: exit", __func__);
}

int populateInMediaConfig(SessionGsl *s, int tagId, void* graphHandle, Stream *st) {
    int32_t status;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct qal_stream_attributes sAttr;

    st->getStreamAttributes(&sAttr);

    QAL_VERBOSE(LOG_TAG,"%s: enter ", __func__);
    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                         &moduleInfo, &moduleInfoSize);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: Failed to get tag info %x module size\n", __func__, tagId);
        return status;
    }
    
    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadInMediaConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, &sAttr);
    QAL_VERBOSE(LOG_TAG,"%s:%d %p - payload and %d size", payload , payloadSize);
    
    status = gslSetCustomConfig(graphHandle, payload, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    delete builder;
    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

int populateOutMediaConfig(SessionGsl *s, int tagId, void* graphHandle, Stream *st) {
    int32_t status;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;
    struct qal_stream_attributes sAttr;

    st->getStreamAttributes(&sAttr);

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: Failed to get tag info\n", __func__);
        return status;
    }

    PayloadBuilder *builder = new PayloadBuilder();
    for (int i = 0;i < moduleInfo->num_modules;i++) {
        payload = NULL;
        payloadSize = 0;
        builder->payloadOutMediaConfig(&payload, &payloadSize, moduleInfo->module_entry[i].module_iid, &sAttr);
        QAL_VERBOSE(LOG_TAG,"%s:%d %p - payload and %d size", payload , payloadSize);
        status = gslSetCustomConfig(graphHandle, payload, payloadSize);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
        }
        
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    delete builder; 
    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

int populateCodecDmaConfig(SessionGsl *s, int tagId, void* graphHandle, Stream *st) {
    int32_t status;
    struct gsl_module_id_info *moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: Failed to get tag info\n", __func__);
        return status;
    }
    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadCodecDmaConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, tagId);
    QAL_VERBOSE(LOG_TAG,"%s:%d %p - payload and %d size", payload , payloadSize);

    status = gslSetCustomConfig(graphHandle, payload, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    delete builder;
    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

int populateHwEpConfig(SessionGsl *s, int tagId, void* graphHandle, Stream *st) {
    int32_t status;
    struct gsl_module_id_info* moduleInfo;
    size_t moduleInfoSize;
    uint8_t* payload = NULL;
    size_t payloadSize = 0;

    status = gslGetTaggedModuleInfo(s->gkv, tagId,
                                    &moduleInfo, &moduleInfoSize);
    if (status != 0) {
        QAL_ERR(LOG_TAG,"%s: Failed to get tag info\n", __func__);
        return status;
    }
    PayloadBuilder *builder = new PayloadBuilder();
    builder->payloadHwEpConfig(&payload, &payloadSize, moduleInfo->module_entry[0].module_iid, tagId);
    QAL_VERBOSE(LOG_TAG,"%s:%d %p - payload and %d size", payload , payloadSize);

    status = gslSetCustomConfig(graphHandle, payload, payloadSize);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Get custom config failed with status = %d\n", __func__, status);
    }

    if (moduleInfo)
        free(moduleInfo);

    if (payload)
        free(payload);
    delete builder;

    QAL_VERBOSE(LOG_TAG,"%s: exit status- %d", __func__, status);
    return status;
}

int SessionGsl::open(Stream *s) {
    int status = 0;
    struct qal_stream_attributes sAttr;
    s->getStreamAttributes(&sAttr);

    QAL_VERBOSE(LOG_TAG,"%s: enter direction:%d ", __func__, sAttr.direction);

    gkv = new gsl_key_vector;
    gkv->kvp = new gsl_key_value_pair[3];
    status = populateGkv(s, gkv);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to populate gkv", __func__);
        goto exit;
    }

    ckv = new gsl_key_vector;
    ckv->kvp = new gsl_key_value_pair[3];
    status = populateCkv(s, ckv);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to populate ckv", __func__);
        goto exit;
    }

    status = gslOpen(gkv, ckv, &graphHandle);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to open the graph", __func__);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: graph open success. handle:%p", __func__, graphHandle);

    //status = gsl_set_cal_(graphHandle, ckv);
    if(0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to set the calibration data\n", __func__);
        goto exit;
    }
    QAL_VERBOSE(LOG_TAG,"%s: graph_set_cal success. handle:%p", __func__, graphHandle);

    switch (sAttr.direction) {
        case QAL_AUDIO_INPUT:

            status = setConfig(s, OUT_MEDIA_CONFIG, OUTPUT_MEDIA_FORMAT);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Output media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,MEDIA_CONFIG,HW_ENDPOINT_TX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,CODEC_DMA_CONFIG,HW_ENDPOINT_TX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Codec DMA setconfig failed with status %d",__func__,status);
                goto exit;
            }
            break;
        case QAL_AUDIO_OUTPUT:

            status = setConfig(s, IN_MEDIA_CONFIG, INPUT_MEDIA_FORMAT);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Input media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s, OUT_MEDIA_CONFIG, OUTPUT_MEDIA_FORMAT);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Output media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,MEDIA_CONFIG,HW_ENDPOINT_RX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,CODEC_DMA_CONFIG,HW_ENDPOINT_RX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Codec DMA setconfig failed with status %d",__func__,status);
                goto exit;
            }
            break;
        case QAL_AUDIO_INPUT_OUTPUT:

            //status = setConfig(s, OUT_MEDIA_CONFIG, OUTPUT_MEDIA_FORMAT);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Output media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,MEDIA_CONFIG,HW_ENDPOINT_RX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,CODEC_DMA_CONFIG,HW_ENDPOINT_RX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Codec DMA setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,MEDIA_CONFIG,HW_ENDPOINT_TX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Media setconfig failed with status %d",__func__,status);
                goto exit;
            }

            status = setConfig(s,CODEC_DMA_CONFIG,HW_ENDPOINT_TX);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Codec DMA setconfig failed with status %d",__func__,status);
                goto exit;
            }
            break;
        default:
            QAL_ERR(LOG_TAG,"%s: Invalid direction \n",__func__);
            break;
    }
    QAL_VERBOSE(LOG_TAG,"%s: end status:%d", __func__, status);
exit:
     return status;
}

int SessionGsl::prepare(Stream * s) {
    int status = 0;
    struct qal_stream_attributes sAttr;
    s->getStreamAttributes(&sAttr);

    QAL_VERBOSE(LOG_TAG,"%s: enter direction:%d ", __func__, sAttr.direction);

    status = gslIoctl(graphHandle, GSL_CMD_PREPARE, NULL, 0);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to prepare the graph", __func__);
        goto exit;
    }

    switch (sAttr.direction) {
        case QAL_AUDIO_INPUT:
            status = readBufferInit(s, NO_OF_BUF, BUF_SIZE_CAPTURE, DATA_MODE_BLOCKING);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Tx session readBufferInit is failed with status %d",__func__,status);
                goto exit;
            }
            break;
        case QAL_AUDIO_OUTPUT:
            status = writeBufferInit(s, NO_OF_BUF, BUF_SIZE, DATA_MODE_BLOCKING);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Rx session writeBufferInit is failed with status %d",__func__,status);
                goto exit;
            }
            break;
        case QAL_AUDIO_INPUT_OUTPUT:
            break;
        default:
            break;
   }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::readBufferInit(Stream * s, size_t noOfBuf, size_t bufSize, int flag) {
    int status = 0;

    QAL_VERBOSE(LOG_TAG,"%s: enter \n bufSize:%d noOfBuf:%d flag:%d", __func__, bufSize, noOfBuf, flag);

    infoBuffer = (struct gslCmdGetReadWriteBufInfo*)malloc(sizeof(struct gslCmdGetReadWriteBufInfo));
    infoBuffer->buff_size = bufSize;
    infoBuffer->num_buffs = noOfBuf;
    infoBuffer->attritubes = flag;
    infoBuffer->start_threshold = 0;
    infoBuffer->stop_threshold = 0;

    size = sizeof(struct gslCmdGetReadWriteBufInfo);
    status = gslIoctl(graphHandle, GSL_CMD_CONFIGURE_READ_PARAMS, infoBuffer, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to initialize the read buffer in gsl", __func__);
        goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::writeBufferInit(Stream * s, size_t noOfBuf, size_t bufSize, int flag) {
    int status = 0;
    struct gslCmdGetReadWriteBufInfo buf;

    QAL_VERBOSE(LOG_TAG,"%s: enter \n bufSize:%d noOfBuf:%d flag:%d", __func__, bufSize, noOfBuf, flag);

    infoBuffer = (struct gslCmdGetReadWriteBufInfo*)malloc(sizeof(struct gslCmdGetReadWriteBufInfo));
    infoBuffer->buff_size = bufSize;
    infoBuffer->num_buffs = noOfBuf;
    infoBuffer->attritubes = flag;
    infoBuffer->start_threshold = 0;
    infoBuffer->stop_threshold = 0;

    size = sizeof(struct gslCmdGetReadWriteBufInfo);
    status = gslIoctl(graphHandle, GSL_CMD_CONFIGURE_WRITE_PARAMS, infoBuffer, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to initialize write buffer in gsl", __func__);
        goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::close(Stream *s) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslClose(graphHandle);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to close the graph", __func__);
        goto exit;
    }
    QAL_ERR(LOG_TAG,"%s: gsl_close successful", __func__);

    free(gkv->kvp);
    free(gkv);
    gkv = NULL;
    free(ckv->kvp);
    free(ckv);
    ckv = NULL;
exit:
    QAL_ERR(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::start(Stream *s) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslIoctl(graphHandle, GSL_CMD_START, payload, size);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to start the graph", __func__);
        goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionGsl::pause()
{
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);
    status = gsl_ioctl_(graphHandle, GSL_CMD_PAUSE, payload, size);
    if(0 != status)
    {
         goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}*/

int SessionGsl::stop(Stream * s) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);

    status = gslIoctl(graphHandle, GSL_CMD_STOP, payload, size);
    if(0 != status) {
         QAL_ERR(LOG_TAG,"%s: Failed to stop the graph", __func__);
         goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
/*
int SessionGsl::changeGraph(struct gslKeyVector * graphKeyVectors,
                            struct gslKeyVector * calKeyVectors)
{
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p", __func__, graphHandle);
    status = gsl_ioctl_(graphHandle, , payload, size);
    if(0 != status)
    {
         goto exit;
    }
exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}
*/

int SessionGsl::setConfig(Stream *s, configType type, int tag) {
    int status = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p type:%d tag:%d", __func__, graphHandle, type, tag);
    switch (type) {
        case GRAPH:
/*          status = stop();
            if (0 != status) {
                goto exit;
            }
            status = changeGraph(graphKeyVectors, calKeyVectors);
            if (0 != status) {
                goto exit;
            }*/
            break;
        case MODULE:
            tkv = NULL;
            //status = populateTkv(s, tkv, tag);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the tag configuration", __func__);
                goto exit;
            }
            //status = gsl_set_cfg(graphHandle, tag, tkv);
            if (0 != status) {
                goto exit;
            }
            break;
        case CALIBRATION:
            ckv = NULL;
            status = populateCkv(s, ckv);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate calibration data", __func__);
                goto exit;
            }
            status = gslSetCal(graphHandle, ckv);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to set the calibration data\n", __func__);
                goto exit;
            }
            break;
        case CODEC_DMA_CONFIG:
            status = populateCodecDmaConfig(this, tag, graphHandle, s);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate codec config", __func__);
                goto exit;
            }
            break;
        case MEDIA_CONFIG:
            status = populateHwEpConfig(this, tag, graphHandle, s);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate media config", __func__);
                goto exit;
            }
            break;
        case IN_MEDIA_CONFIG:
            status = populateInMediaConfig(this, tag, graphHandle, s);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate input media config", __func__);
                goto exit;
            }
            break;
        case OUT_MEDIA_CONFIG:
            status = populateOutMediaConfig(this, tag, graphHandle, s);
            if (0 != status) {
                QAL_ERR(LOG_TAG,"%s: Failed to populate output media config", __func__);
                goto exit;
            }
            break;
        default:
            QAL_VERBOSE(LOG_TAG,"%s: invalid type ", __func__);
            status = -EINVAL;
            goto exit;
    }

exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit status:%d ", __func__, status);
    return status;
}

int SessionGsl::fileRead(Stream *s, int tag, struct qal_buffer *buf, int * size) {
    std::fstream fs;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);

    fs.open ("/data/test.wav", std::fstream::binary | std::fstream::in | std::fstream::app);
    QAL_VERBOSE(LOG_TAG,"file open success");
    char * buff = static_cast<char *>(buf->buffer);
    if(seek != 0) {
       fs.seekp(seek);
    }
    fs.read (buff,buf->size);
    seek += buf->size;
    QAL_VERBOSE(LOG_TAG,"file read success");
    fs.close();
    QAL_VERBOSE(LOG_TAG,"file close success");
    *size = (int)(buf->size);
    QAL_VERBOSE(LOG_TAG,"size: %d", *size);
    return 0;
}

int SessionGsl::read(Stream *s, int tag, struct qal_buffer *buf, int * size) {
    int status = 0, bytesRead = 0, bytesToRead = 0, offset = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p buf:%p tag:%d", __func__, graphHandle, buf, tag);
    //fileRead(s,tag,buf,size);
    //return 0;
    if (!buf || !s) {
        QAL_ERR(LOG_TAG,"%s: Invalid stream or buffer", __func__);
        status = -EINVAL;
        goto exit;
    }

    struct gsl_buff gslBuff;
    QAL_VERBOSE(LOG_TAG,"%s: bufsize:%d bufNo:%d", __func__, infoBuffer->buff_size, infoBuffer->num_buffs);

    while (1) {
        offset = bytesRead + buf->offset;
        bytesToRead = buf->size - offset;
        if (!bytesToRead)
            break;
        gslBuff.flags = 0;
        if ((bytesToRead / infoBuffer->buff_size) >= 1)
            gslBuff.size = infoBuffer->buff_size;
        else
            gslBuff.size = bytesToRead;


        uint32_t sizeRead;
        void *data = buf->buffer;
        data = static_cast<char*>(data) + offset;
        gslBuff.addr = static_cast<uint8_t*>(data);
        status = gslRead(graphHandle, tag, &gslBuff, &sizeRead);
        if ((0 != status) || (sizeRead == 0)) {
            QAL_ERR(LOG_TAG,"%s: Failed to read data from gsl", __func__);
            goto exit;
        }
        //buf->ts = gsl_buf.timestamp;
        bytesRead += sizeRead;
    }
exit:
    *size = bytesRead;
    QAL_VERBOSE(LOG_TAG,"%s: exit bytesRead:%d status:%d ", __func__, bytesRead, status);
    return status;
}

int SessionGsl::fileWrite(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) {
    std::fstream fs;
    QAL_VERBOSE(LOG_TAG,"%s: enter", __func__);

    fs.open ("/data/test.wav", std::fstream::binary | std::fstream::out | std::fstream::app);
    QAL_VERBOSE(LOG_TAG,"file open success");
    char * buff=static_cast<char *>(buf->buffer);
    fs.write (buff,buf->size);
    QAL_VERBOSE(LOG_TAG,"file write success");
    fs.close();
    QAL_VERBOSE(LOG_TAG,"file close success");
    *size = (int)(buf->size);
    QAL_VERBOSE(LOG_TAG,"size: %d", *size);
    return 0;
}

int SessionGsl::write(Stream *s, int tag, struct qal_buffer *buf, int * size, int flag) {

    int status = 0, bytesWritten = 0, bytesRemaining = 0, offset = 0;
    uint32_t sizeWritten = 0;
    QAL_VERBOSE(LOG_TAG,"%s: enter graphHanlde:%p buf:%p tag:%d flag:%d", __func__, graphHandle, buf, tag, flag);
    
    void *data = nullptr;
    struct gsl_buff gslBuff;
    gslBuff.timestamp = (uint64_t) buf->ts;

    bytesRemaining = buf->size;

    while ((bytesRemaining / infoBuffer->buff_size) > 1) {
        gslBuff.flags = 0;

        offset = bytesWritten + buf->offset;
        gslBuff.size = infoBuffer->buff_size;
        data = buf->buffer;
        data = static_cast<char *>(data) + offset;

        gslBuff.addr = static_cast<uint8_t *>(data);
        sizeWritten = 0;  //initialize 0
        status = gslWrite(graphHandle, tag, &gslBuff, &sizeWritten);
        if (0 != status) {
            QAL_ERR(LOG_TAG,"%s: Failed to write the data to gsl", __func__);
            return status;
        }
        bytesWritten += sizeWritten;
        bytesRemaining -= sizeWritten;
    }

    if (BUFFER_EOS == flag)
        gslBuff.flags = BUFF_FLAG_EOS;
    else
        gslBuff.flags = 0;
    offset = bytesWritten + buf->offset;
    gslBuff.size = bytesRemaining;
    data = buf->buffer;
    data = static_cast<char *>(data) + offset;
    gslBuff.addr = static_cast<uint8_t *>(data);
    sizeWritten = 0;  //0
    status = gslWrite(graphHandle, tag, &gslBuff, &sizeWritten);
    if (0 != status) {
        QAL_ERR(LOG_TAG,"%s: Failed to write the data to gsl", __func__);
        goto exit;
    }
    bytesWritten += sizeWritten;
    *size = bytesWritten;

exit:
    QAL_VERBOSE(LOG_TAG,"%s: exit bytesWritten:%d status:%d ", __func__, bytesWritten, status);
    return 0;
}
