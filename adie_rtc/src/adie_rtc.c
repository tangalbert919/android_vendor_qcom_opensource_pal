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

#define LOG_TAG "ADIE_RTC"
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
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

#undef __packed
#define CODEC_INFO_PATH "/proc/asound/card0/codecs/"
#define __packed __attribute__((packed))
#define ADIE_RTAC_MINOR_VERSION    0
#define ADIE_RTAC_MAJOR_VERSION    2
#define CDC_REG_DIG_BASE_READ    0x200
#define CDC_REG_DIG_OFFSET    0x000
#define CDC_REG_DIG_BASE_WRITE    0x200
#define MAX_NUMBER_OF_CODECS    20
#define FILE_NAME_LENGTH    200
static uint32_t found_codec_path = 0;
static uint32_t register_length = 4;
#define NUMBER_OF_SUBSTRING 3
#define READ_STEP_SIZE 4000
#define ADIE_RTC_HEADER_SIZE 2

static struct chipset_id_info codec_chipset[] = {
    {"BOLERO", BOLERO},
    {"WCD938X", WCD938X},
    {"WCD937X", WCD937X},
    {"WCD9360", WCD9360},
    {"AQT1000", AQT1000},
    {"WCD9341", WCD9341},
    {"WCD9340", WCD9340},
    {"MSM8X52", MSM8X52},
    {"WHS9410", WHS9410},
    {"WHS9420", WHS9420},
    {"WCD9335", WCD9335},
    {"WSA881X-ANALOG", WSA881X_ANALOG},
    {"WSA881X-SOUNDWIRE", WSA881X_SOUNDWIRE},
    {"WSA881X", WSA881X},
    {"MSM8909", MSM8909},
    {"WCD9330", WCD9330},
    {"WCD9326", WCD9326},
    {"WCD9320", WCD9320},
    {"WCD9310", WCD9310},
    {"WCD9306", WCD9306},
    {"WCD9302", WCD9302},
    {"MSM8X16", MSM8X16},
    {"MSM8X10", MSM8X10},
    {"CODEC_UNDEFINED", CODEC_UNDEFINED}
 };

struct codec_info {
    uint32_t    handle;
    uint32_t    chipset_id;
    uint32_t    major_version;
    uint32_t    minor_version;
    char        codec_name[FILE_NAME_LENGTH];
    char        reg_path[FILE_NAME_LENGTH];
 };

static uint32_t number_of_codecs = 0;
static struct codec_info *codec_info;

int find_codec_index(uint32_t handle)
 {
    int i;
    for (i=0; i < (int)number_of_codecs; i++)
        if (codec_info[i].handle == handle)
            goto done;
    CASA_LOG_ERR(LOG_TAG,"%s->could not find codec with handle %d\n", __func__, handle);
    i = -EINVAL;
 done:
    return i;
 }

 static int get_chipset_id(char *chipset_name)
 {
    int ret = 0;
    unsigned int i;
    if(chipset_name == NULL){
        CASA_LOG_ERR(LOG_TAG,"chipset_name is null");
        ret = -EINVAL;
        goto done;
    }

    for (i = 0; i < sizeof(codec_chipset)/sizeof(codec_chipset[0]);i++) {
        if (!strncmp(chipset_name, codec_chipset[i].name, strlen(chipset_name)+1)) {
            codec_info[number_of_codecs].chipset_id = codec_chipset[i].id;
            goto done;
        }
    }
    ret = -EINVAL;
 done:
    return ret;
 }

static int read_version_file(char *version_path)
 {
    int ret = 0;
    casa_fhandle file_handle;
    int fd;
    char    *token;
    char    *token2;
    char    *save;
    size_t  num_read;
    char    version_entry[FILE_NAME_LENGTH];
    if(version_path == NULL) {
        CASA_LOG_ERR(LOG_TAG,"version_path is null");
        ret = -EINVAL;
        goto done;
    }

    fd = casa_fopen(&file_handle, version_path, CASA_FOPEN_READ_ONLY);
    if (fd != 0) {
        CASA_LOG_ERR(LOG_TAG,"%s: file open failed for path %s\n", __func__, version_path);
        ret = -ENODEV;
        goto done;
    }

    fd = casa_fread(file_handle, version_entry, sizeof(version_entry),&num_read);
    if (fd != 0) {
        CASA_LOG_ERR(LOG_TAG,"%s: file read failed for path %s\n", __func__);
        ret = -ENODEV;
        goto done;
    }

    token = strtok_r(version_entry, "_", &save);
    if (token == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: strtok failed to get chipset name\n", __func__);
        ret = -EINVAL;
        goto close;
    }

    ret = get_chipset_id(token);
    if (ret < 0) {
        CASA_LOG_ERR(LOG_TAG,"%s: get_chipset_id failed error %d\n", __func__, ret);
        ret = -EINVAL;
        goto close;
    }

    token = strtok_r(NULL, "_", &save);
    if (token == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: strtok failed to get chipset major version\n", __func__);
        ret = -EINVAL;
        goto close;
    }

    token2 = strtok_r(NULL, "_", &save);
    if (token2 == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: strtok failed to get chipset minor version\n", __func__);
        ret = -EINVAL;
        goto close;
    }

    codec_info[number_of_codecs].major_version = atoi(token);
    codec_info[number_of_codecs].minor_version = atoi(token2);

 close:
    fclose(file_handle);
 done:
    return ret;
 }

static int get_reg_path(char **match_strings, int array_size)
 {
    DIR *dir;
    int i;
    int ret = -EINVAL;
    struct dirent *dirent;
    char path[FILE_NAME_LENGTH] = "/sys/kernel/debug";

    if(*match_strings == NULL){
        ret = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"Empty array of string");
    }

    for(i = 0; i <  array_size; i++) {
        dir = opendir(path);
        if (dir == NULL) {
            CASA_LOG_INFO(LOG_TAG,"%d (%s) opendir %s failed\n", errno, strerror(errno), path);
            return -EINVAL;
        }

        while (NULL != (dirent = readdir(dir))) {
            if (strstr(dirent->d_name, match_strings[i]))
            {
                strlcat(path, "/", sizeof(path));
                strlcat(path, dirent->d_name, sizeof(path));

                /* If "reg" found don't search anymore */
                if (i ==  array_size - 1) {
                    strlcpy(codec_info[number_of_codecs].reg_path, path,
                        sizeof(codec_info[number_of_codecs].reg_path));
                    found_codec_path = 1;
                    ret = 0;
                    closedir(dir);
                    goto done;
                }
            }
        }
        closedir(dir);
    }
 done:
    return ret;
 }

static int find_codecs_info(void)
 {
    DIR     *dir;
    struct dirent   *dentry;
    int ret = 0;
    char version_path[FILE_NAME_LENGTH] = CODEC_INFO_PATH;
    dir = opendir(version_path);
    if (dir == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: %d (%s) opendir %s failed\n", __func__, errno, strerror(errno), CODEC_INFO_PATH);
        ret = -ENODEV;
        goto done;
    }

    while ((dentry = readdir(dir)) != NULL) {
        if (!strncmp(dentry->d_name, ".", 1))
            continue;
        ret = snprintf(codec_info[number_of_codecs].codec_name,
            sizeof(codec_info[number_of_codecs].codec_name), "%s", dentry->d_name);
        if (ret < 0) {
            CASA_LOG_ERR(LOG_TAG,"%s: snprintf failed: %s in %s, err %d\n",
                __func__, dentry->d_name, CODEC_INFO_PATH, ret);
            continue;
        }
        ret = snprintf(version_path, sizeof(version_path),
            "%s%s/version", CODEC_INFO_PATH, dentry->d_name);
        if (ret < 0) {
            CASA_LOG_ERR(LOG_TAG,"%s: snprintf failed: %s in %s, err %d\n",
                __func__, dentry->d_name, CODEC_INFO_PATH, ret);
            continue;
        }
        ret = read_version_file(version_path);
        if (ret < 0) {
            CASA_LOG_ERR(LOG_TAG,"%s: read_version_file failed, err %d path %s\n",
                __func__, ret, version_path);
            continue;
        }
        char *match_strings[] = {"regmap", codec_info[number_of_codecs].codec_name, "registers"};
        ret = get_reg_path(match_strings, sizeof(match_strings)/sizeof(char *));
        if (ret < 0) {
            CASA_LOG_ERR(LOG_TAG,"%s: get_reg_path failed, err %d, path %s\n",
                __func__, ret, version_path);
            continue;
        }
        codec_info[number_of_codecs].handle = number_of_codecs+1;
        number_of_codecs++;
    }
    closedir(dir);
 done:
    return ret;
 }

static int find_codecs(void)
{
    int ret = 0;
    unsigned int i;
    char *old_match_strings[] = {"regmap", "-000d", "registers"};
    number_of_codecs = 0;
    ret = find_codecs_info();
    if (number_of_codecs == 0) {
        ret = get_reg_path(old_match_strings, sizeof(old_match_strings)/sizeof(char *));
        if (ret < 0) {
            CASA_LOG_ERR(LOG_TAG,"%s: get_reg_path failed, err %d\n",__func__, ret);
            goto done;
        }

        codec_info[number_of_codecs].handle = number_of_codecs+1;
        number_of_codecs++;
    }
    for (i=0; i < number_of_codecs; i++)
        CASA_LOG_VERBOSE(LOG_TAG,"%s: codec %s: handle %d, chipset id %d, major %d, minor %d, reg path %s\n",
            __func__, codec_info[i].codec_name, codec_info[i].handle, codec_info[i].chipset_id,
            codec_info[i].major_version, codec_info[i].minor_version, codec_info[i].reg_path);
 done:
    return ret;
 }

int32_t adie_rtc_get_codec_info(struct adie_rtc_codec_info *cdc_info)
{
    int i,ret=0;
    if(cdc_info->num_of_entries <= 0) {
        cdc_info->num_of_entries = number_of_codecs;
        return ret;
    }
    else{
        if(cdc_info->handle == NULL) {
            CASA_LOG_ERR(LOG_TAG,"handle is null");
            ret = -EINVAL;
            goto done;
        }

        for(i = 0;i<number_of_codecs;i++) {
            cdc_info->handle[i].handle = codec_info[i].handle;
            cdc_info->handle[i].chipset_id = codec_info[i].chipset_id;
            cdc_info->handle[i].chipset_major_version = codec_info[i].major_version;
            cdc_info->handle[i].chipset_minor_version = codec_info[i].minor_version;
            }
        goto done;
    }
 done:
    return ret;

}

int32_t adie_rtc_get_version(struct adie_rtc_version *ver)
{
    int ret=0;
    ver->major = ADIE_RTAC_MAJOR_VERSION;
    ver->minor = ADIE_RTAC_MINOR_VERSION;
    return ret;
}

static int parse_codec_reg_file(char_t **rtc_io_buf_base, int32_t *rtc_io_buf_size,
                int fd, int32_t codec_idx)
 {
    char_t *rtc_io_buf = NULL;
    char_t *temp;
    int32_t buf_size = 0, rc = 0;
    int32_t numBytes;

    if (fd < 0)
    {
        rc = -EINVAL;
        *rtc_io_buf_base = NULL;
        CASA_LOG_ERR(LOG_TAG,"Invalid fd");
        goto done;
    }

    while (1)
    {
        temp = realloc(*rtc_io_buf_base, buf_size + READ_STEP_SIZE);
        if (!temp)
        {
            rc = -ENOMEM;
            free(*rtc_io_buf_base);
            *rtc_io_buf_base = NULL;
            buf_size = 0;
            CASA_LOG_ERR(LOG_TAG,"cannot allocate memory: %d, path: %s",
                 READ_STEP_SIZE, codec_info[codec_idx].reg_path);
            goto done;
        }
        *rtc_io_buf_base = temp;
        rtc_io_buf = *rtc_io_buf_base + buf_size;
        numBytes = read(fd, rtc_io_buf, READ_STEP_SIZE);
        if (numBytes <= 0)
            break;

        buf_size += numBytes;
    }
 done:
    *rtc_io_buf_size = buf_size;

    return rc;
 }

int32_t adie_rtc_get_register(struct adie_rtc_register_req *req)
{
    int res = 0;
    int result = 0;
    int fd = -1;
    char_t *rtc_io_buf = NULL;
    char_t *rtc_io_buf_base = NULL;
    int found = 0;
    uint32_t ultempRegAddr = 0;
    int32_t lRegValue = 0;
    uint32_t handle = req->codec_handle;
    int32_t codec_idx = 0;
    uint32_t regAddr = req->register_id;
    uint32_t regMask = req->register_mask;
    size_t numBytes = 0;
    char_t *pCurInputBuf = NULL;
    int32_t rtc_io_buf_size = 0;
    char_t reg[5], val[3];
    reg[4] = '\0', val[2] = '\0';
    uint32_t offset = 0;
    int reg_len;
    char *save;
    char *token;

    codec_idx = find_codec_index(handle);
    if (codec_idx < 0) {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"could not find codec index for handle\n %d", handle);
        goto done;
    } else if (strlen(codec_info[codec_idx].reg_path) == 0) {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"codec path is empty\n %d", handle);
        goto done;
    }
    //CASA_LOG_INFO(LOG_TAG,"reg path= %s",codec_info[codec_idx].reg_path);
    fd = open(codec_info[codec_idx].reg_path, O_RDWR);
    if(fd < 0)
    {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"cannot open adie peek error: %d, path: %s",
                fd, codec_info[codec_idx].reg_path);
        goto done;
    }
    result = parse_codec_reg_file(&rtc_io_buf_base, &rtc_io_buf_size, fd, codec_idx);
    if (rtc_io_buf_base == NULL || result < 0)
    {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"cannot allocate memory: %d, path: %s",
                READ_STEP_SIZE, codec_info[codec_idx].reg_path);
            goto close_fd;
    }
    if (rtc_io_buf_size <= 0)
    {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"length of written bytes does not match expected value %d", rtc_io_buf_size);
        goto close_fd;
    }
    numBytes = rtc_io_buf_size;
    rtc_io_buf = rtc_io_buf_base;
    memcpy((void*)reg, (void*)(rtc_io_buf+offset), sizeof(uint32_t));
    token = strtok_r(reg, ":", &save);
    if (token == NULL) {
        CASA_LOG_ERR(LOG_TAG,"%s: strtok failed to find register length : delimiter!\n", __func__);
        result = -EINVAL;
        goto done;
    }
    reg_len = strlen(token);
    if (reg_len <= 0) {
        CASA_LOG_ERR(LOG_TAG,"%s: register length is %d!\n", __func__, reg_len);
        result = -EINVAL;
        goto done;
    }
    register_length = reg_len;
    CASA_LOG_DEBUG(LOG_TAG,"%s: valid register length is %d\n", __func__, register_length);
    while(numBytes>offset)
    {
        memcpy((void*)reg, (void*)(rtc_io_buf+offset), sizeof(uint32_t));
        offset += register_length;
        ultempRegAddr = strtoul(reg, NULL, 16);
        offset += 2;
        memcpy((void*)val, (void*)(rtc_io_buf+offset), sizeof(uint16_t));
        lRegValue = strtol(val, NULL, 16);
        offset += 3;
        if (ultempRegAddr >= CDC_REG_DIG_BASE_READ)
            ultempRegAddr -= CDC_REG_DIG_OFFSET;
        if(ultempRegAddr == regAddr)
        {
            found = 1;
            CASA_LOG_INFO(LOG_TAG,"register[%08X] found from the file!\n",regAddr);
            break;
        }
    }
    if (found == 0)
    {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"get adie register[0x%x] failed Peek(%s) Poke(%s)",
        regAddr, codec_info[codec_idx].reg_path, codec_info[codec_idx].reg_path);
        goto close_fd;
    }
    else
    {
        CASA_LOG_ERR(LOG_TAG,"Found the value for register = 0x%X, value = 0x%X\n",regAddr,lRegValue);
    }
    /* return a masked value */
    lRegValue &= regMask;
    req->value = lRegValue;/* output value*/

close_fd:
    close(fd);
done:
    free(rtc_io_buf_base);
    return result;
}

int32_t adie_rtc_set_register(struct adie_rtc_register_req *req)
{
    int result = 0;
    int fd = -1;
    uint32_t ulRegValue = req->value;
    uint32_t handle = req->codec_handle;
    int32_t codec_idx = 0;
    uint32_t regAddr = req->register_id;
    uint32_t regMask = req->register_mask;
    size_t numBytes1 = 0;
    size_t numBytes2 = 0;
    char *temp = NULL;

    codec_idx = find_codec_index(handle);
    if (codec_idx < 0) {
        CASA_LOG_ERR(LOG_TAG,"could not find codec index for handle\n %d", handle);
        goto done;
    } else if (strlen(codec_info[codec_idx].reg_path) == 0) {
        CASA_LOG_ERR(LOG_TAG,"codec path is empty\n %d", handle);
        goto done;
    }

    /* set the value as masked one*/
    ulRegValue &= regMask;

    if (regAddr >= CDC_REG_DIG_BASE_WRITE)
        regAddr += CDC_REG_DIG_OFFSET;

    numBytes1 = asprintf(&temp, "0x%x 0x%x", regAddr, ulRegValue);
    CASA_LOG_DEBUG(LOG_TAG,"set register request received for ==> reg[%X], val[%X], bytes[%zu]\n",
            regAddr, ulRegValue, numBytes1);
    fd = open(codec_info[codec_idx].reg_path, O_RDWR);
    if(fd < 0) {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"ERROR! cannot open adie poke error: %d, path: %s",
                fd, codec_info[codec_idx].reg_path);
        if (temp != NULL)
        {
            free(temp);
            temp = NULL;
        }

        goto done;
    }
    numBytes2 = write(fd, temp, numBytes1);
    CASA_LOG_DEBUG(LOG_TAG,"set register ==> actual bytes written[%zu]\n", numBytes2);
    if (temp != NULL) {
        free(temp);
        temp = NULL;
    }
    close(fd);
    /* make sure the write is successful */
    if (numBytes1 != numBytes2) {
        result = -EINVAL;
        CASA_LOG_ERR(LOG_TAG,"set adie register failed for Register[0x%X], numBytes[%zu]",regAddr ,numBytes1);
        goto done;
    }
    //*resp_buf_bytes_filled = 0;
    CASA_LOG_INFO(LOG_TAG,"Set Register Success\n");

 done:
    return result;
 }

int32_t adie_rtc_init()
{
    int ret = 0;
    codec_info = (struct codec_info *)malloc(MAX_NUMBER_OF_CODECS*sizeof(struct codec_info));
    if(codec_info == NULL) {
        ret = -ENOMEM;
        CASA_LOG_ERR(LOG_TAG,"malloc failed\n");
        goto done;
    }
    if (!found_codec_path) {
        if (find_codecs() < 0) {
            ret = -EINVAL;
            CASA_LOG_ERR(LOG_TAG,"failed to get register paths \n");
        }
    }

 done:
    return ret;
}

int32_t adie_rtc_deinit()
{
    int ret = 0;
    free(codec_info);
    codec_info = NULL;
    return ret;
}