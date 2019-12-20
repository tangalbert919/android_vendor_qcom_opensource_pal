#ifndef _QTS_H_
#define _QTS_H_
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
/*============================================================================
*                      EDIT HISTORY FOR FILE
*
*  This section contains comments describing changes made to this file.
*  Notice that changes are listed in reverse chronological order.
*
*  when         who     what, where, why
*  --------   ---     ----------------------------------------------------------
*  01/09/19   kde     Initial Draft
********************************************************************************
*/
/* $Header: //components/dev/audio.acdb.casa/1.0/kofie.audio.acdb.casa.1.0.qts_updates/qts/api/qts.h#2 $ */

/*------------------------------------------
* Includes
*------------------------------------------*/
#include "casa_osal_error.h"
#include "casa_osal_types.h"
#include "acdb.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------
* ERROR CODE definitions 4 char_t
*------------------------------------------*/
enum QTS_ERROR_CODES
{
    /**< General success*/
    QTS_EOK = 0x00,
    /**< General failure*/
    QTS_EFAILED = 0x01,
    /**< Invalid parameter(s) passed*/
    QTS_EBAD_PARAM,
    /**< Unable to allocate enough memory resources*/
    QTS_EINSUFFICIENT_MEM,
    /**< There is n active audio session*/
    QTS_EACTIVE_SESSION_FOUND,
    /**< The provided playback/record duration is invalid*/
    QTS_EINVALID_DURATION,
    /**< The provided playback/record duration is invalid*/
    QTS_ELENGTH_NOT_MATCH,
    /**< The provided command ID is not supported*/
    QTS_EINVALID_COMMAND,
    /**< The provided service ID is not supported*/
    QTS_EINVALID_SERVICE_ID,
    /**< ...*/
    QTS_EPARTIAL_SUCCESS,
};

typedef struct _qts_buffer_t QtsBuffer;
struct _qts_buffer_t
{
    uint32_t max_size;
    uint8_t *buffer;
};

typedef struct _qts_buffer_man_t QtsBufferManager;
struct _qts_buffer_man_t
{
    /**< Main QTS buffer used for storing the response of commands*/
    QtsBuffer main_buffer;
    /**< Main QTS Server message buffer used for storing complete messages recieved from TCP/IP clients*/
    QtsBuffer msg_buffer;
    /**< Main QTS Server recieve buffer used for storing messages recieved from TCP/IP clients*/
    QtsBuffer recieve_buffer;
};

extern QtsBufferManager *qts_buffer_manager;
extern uint8_t *qts_main_buffer;

//TODO: Move to bottom of file
//void GetBufferManager(QtsBufferManager** buffer_man)
//{
//    *buffer_man = qts_buffer_manager;
//}


/*-----------------------------------------------------------------------------
** flag, page size, and buffer length definition for qts_main_buffer
*----------------------------------------------------------------------------*/
#define QTS_BUFFER_LENGTH                   0x200000
#define QTS_HEADER_LENGTH                   8

#define QTS_SERVICE_COMMAND_ID_LENGTH       4
#define QTS_DATA_LENGTH_LENGTH              4
#define QTS_ERROR_CODE_LENGTH               4
#define QTS_ERROR_FRAME_LENGTH              12

#define QTS_SERVICE_COMMAND_ID_POSITION     0
#define QTS_DATA_LENGTH_POSITION            4
#define QTS_ACDB_BUFFER_POSITION            8

/*-----------------------------------------------------------------------------
* Macros for QTS Services
*----------------------------------------------------------------------------*/

/**< Extract the service id from the MSW of the 4byte service command id x*/
#define QTS_GET_SERVICE_ID(x) ((0xFFFF0000 & x))

#define QTS_SERVICE_ID_INITIALIZER(x){ (char)(x >> 24), (char)(x >> 16), 0}
#define QTS_SEVICE_ID_STR_LEN (3)
/**< Set the 'x'(the service command id) to the arr to create a service command id string x.
* arr must be initialized with 3 bytes
*/
#define QTS_SERVICE_ID_STR(arr, sz, x) \
do {\
int svc_str_len = QTS_SEVICE_ID_STR_LEN; \
if(sz == svc_str_len)\
{ \
arr[0] = (char)(x >> 24); \
arr[1] = (char)(x >> 16); \
arr[2] = 0; \
}\
break; \
}while(0)

/**< Extract the command id from the LSW of the 4byte service command id x*/
#define QTS_GET_COMMAND_ID(x) ((0x0000FFFF & x))

/**< Defines command IDs for the Online Calibration service */
#define QTS_ONLINE_CMD_ID(x) ((0x4F4E << 16) | (0xFFFF & x))
#define QTS_ONLINE_SERVICE_ID (0x4F4E << 16)

/**< Defines command IDs for the Real Time Tuning service */
#define QTS_RTC_SERVICE_ID (0x5254 << 16)
#define QTS_RTC_CMD_ID(x) ((0x5254 << 16) | (0xFFFF & x))

/**< Defines command IDs for the File Transfer service */
#define QTS_FTS_CMD_ID(x) ((0x4654 << 16) | (0xFFFF & x))
#define QTS_FTS_SERVICE_ID (0x4654 << 16)

/**< Defines command IDs for the Media Control Service */
#define QTS_MCS_CMD_ID(x) ((0x4D43 << 16) | (0xFFFF & x))
#define QTS_MCS_SERVICE_ID (0x4D43 << 16)

/**< Defines command IDs for the Adie Codec RTC service */
#define QTS_CODEC_RTC_CMD_ID(x) ((0x4152 << 16) | (0xFFFF & x))
#define QTS_CODEC_RTC_SERVICE_ID (0x4152 << 16)

#define QTS_MAX_FILENAME_LENGTH 256

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_GET_VERSION Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_GET_VERSION
@{ */

/**
    Queries for the QTS Real Time Calibration service version.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_GET_VERSION.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        Pointer to QtsCmdRtGetVersionRsp
    @param[in] rsp_size
        Size of QtsCmdRtGetVersionRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_GET_VERSION QTS_RTC_CMD_ID(1)
/**< The response structure for QTS_CMD_RT_GET_VERSION. Holds the version information for the Real Time Tuning service*/
typedef struct _qts_version_t QtsVersion;
#include "acdb_begin_pack.h"
struct _qts_version_t
{
    /**< Major version*/
    uint32_t major;
    /**< Minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;


/* ---------------------------------------------------------------------------
* QTS_CMD_RT_GET_ACTIVE_INFO Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_GET_ACTIVE_INFO
@{ */

/**
    Queries QTS for the active usecase information. The active usecase is
    provided by GSL.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_GET_ACTIVE_INFO.
    @param[in] cmd
        Pointer to QtsActiveUsecaseInfo.
    @param[in] cmd_size
        Size of QtsActiveUsecaseInfo.
    @param[out] rsp
        Pointer to QtsCmdRtGetActiveInfoRsp
    @param[in] rsp_size
        Size of QtsCmdRtGetActiveInfoRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_GET_ACTIVE_INFO QTS_RTC_CMD_ID(2)
/**< Holds the active usecase information provided by GSL*/
typedef struct qts_cmd_rt_active_usecase_info_t QtsActiveUsecaseInfo;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_active_usecase_info_t
{
    /**< Handle to a usecase(collection of graphs) provided by gsl_open*/
    uint32_t usecase_handle;
    /**< Number of active graphs*/
    uint32_t num_graphs;
    /**< List of Graph Key vectors for each graph*/
    AcdbGraphKeyVector *graph_key_vector;
    /**< Number of Calibration Key vectors. There is one for each Graph Key Vector*/
    uint32_t num_cal_key_vectors;
    /**< Calibration key vector*/
    AcdbGraphKeyVector *cal_key_vector;
}
#include "acdb_end_pack.h"
;

/**< The response structure for QTS_CMD_RT_GET_ACTIVE_INFO*/
typedef struct qts_cmd_rt_get_active_info_rsp_t QtsCmdRtGetActiveInfoRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_get_active_info_rsp_t
{
    /**< Total size of the active usecase information*/
    uint32_t size;
    /**< Number of active usecases*/
    uint32_t num_usecases;
    /**< List of currently active usecases*/
    QtsActiveUsecaseInfo *active_usecases;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_GET_ACTIVE_INFO */

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT
@{ */

/**
    Queries QTS for the non-persistent calibration data for all GECKO module
    instances for a given subgraph.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT.
    @param[in] cmd
        Pointer to QtsCmdRtGetCalDataNonPersistentReq.
    @param[in] cmd_size
        Size of QtsCmdRtGetCalDataNonPersistentReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT QTS_RTC_CMD_ID(3)
/**< The request structure for QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT*/
typedef struct qts_cmd_rt_get_cal_data_non_persist_req_t QtsCmdRtGetCalDataNonPersistentReq;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_get_cal_data_non_persist_req_t
{
    /**< Handle to a graph provided by gsl_open*/
    uint32_t graph_handle;
    /**< Subgraph ID*/
    uint32_t subgraph_id;
    /**< Size of the subgraph calibration data*/
    uint32_t data_size;
    /**< [in/out] Subgraph calibration data. Format:
    [module_id, param_id, param_size, err_code, payload[param_size]]
    */
    uint8_t *subgraph_cal_data;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT */

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_GET_CAL_DATA_PERSISTENT Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_GET_CAL_DATA_PERSISTENT
@{ */

/**
    Queries QTS for the persistent calibration data for all GECKO module
    instances for a given subgraph.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_GET_CAL_DATA_PERSISTENT.
    @param[in] cmd
        Pointer to QtsCmdRtGetCalDataPersistentReq.
    @param[in] cmd_size
        Size of QtsCmdRtGetCalDataPersistentReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_GET_CAL_DATA_PERSISTENT QTS_RTC_CMD_ID(4)
/**< The request structure for QTS_CMD_RT_GET_CAL_DATA_PERSISTENT*/
typedef struct qts_cmd_rt_get_cal_data_persist_req_t QtsCmdRtGetCalDataPersistentReq;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_get_cal_data_persist_req_t
{
    /**< Handle to a graph provided by gsl_open*/
    uint32_t graph_handle;
    /**< Subgraph ID*/
    uint32_t subgraph_id;
    /**< Size of the subgraph calibration data*/
    uint32_t data_size;
    /**< [in/out] Subgraph calibration data. Format:
    [module_id, param_id, param_size, err_code, payload[param_size]]
    */
    uint8_t *subgraph_cal_data;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_GET_CAL_DATA_PERSISTENT */

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT
@{ */

/**
    Queries QTS for the globally-persisent calibration data for a signle
    PID of a module instance from GSL.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT.
    @param[in] cmd
        Pointer to QtsCmdRtGetCalDataGblPersistentReq.
    @param[in] cmd_size
        Size of QtsCmdRtGetCalDataGblPersistentReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT QTS_RTC_CMD_ID(5)
/**< The request structure for QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT*/
typedef struct qts_cmd_rt_get_cal_data_global_persist_req_t QtsCmdRtGetCalDataGblPersistentReq;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_get_cal_data_global_persist_req_t
{
    /**< Calibration identifier*/
    uint32_t cal_id;
    /**< Size of the parameter calibration data*/
    uint32_t data_size;
    /**< [in/out] Parameter calibration data. Format:
    [param_id, param_size, err_code, payload[param_size]]
    */
    uint8_t *param_cal_data;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT */

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT
@{ */

/**
    Set non-persistent subgraph calibration data to GECKO.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT.
    @param[in] cmd
        Pointer to QtsCmdRtSetCalDataNonPersistentReq.
    @param[in] cmd_size
        Size of QtsCmdRtSetCalDataNonPersistentReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT QTS_RTC_CMD_ID(6)
/**< The request structure for QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT*/
typedef struct qts_cmd_rt_set_cal_data_non_persist_req_t QtsCmdRtSetCalDataNonPersistentReq;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_set_cal_data_non_persist_req_t
{
    /**< Handle to a graph provided by gsl_open*/
    uint32_t graph_handle;
    /**< Subgraph ID*/
    uint32_t subgraph_id;
    /**< Size of the subgraph calibration data*/
    uint32_t data_size;
    /**< [out] Subgraph calibration data. Format:
    [module_id, param_id, param_size, err_code, payload[param_size]]
    */
    uint8_t *subgraph_cal_data;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT */

/* ---------------------------------------------------------------------------
* QTS_CMD_RT_SET_CAL_DATA_PERSISTENT Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_RT_SET_CAL_DATA_PERSISTENT
@{ */

/**
    Set persistent subgraph calibration data to GECKO.

    @param[in] cmd_id
        Command ID is QTS_CMD_RT_SET_CAL_DATA_PERSISTENT.
    @param[in] cmd
        Pointer to QtsCmdRtSetCalDataPersistentReq.
    @param[in] cmd_size
        Size of QtsCmdRtSetCalDataPersistentReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_RT_SET_CAL_DATA_PERSISTENT QTS_RTC_CMD_ID(7)
/**< The request structure for QTS_CMD_RT_SET_CAL_DATA_PERSISTENT*/
typedef struct qts_cmd_rt_set_cal_data_persist_req_t QtsCmdRtSetCalDataPersistentReq;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_set_cal_data_persist_req_t
{
    /**< Handle to a graph provided by gsl_open*/
    uint32_t graph_handle;
    /**< Subgraph ID*/
    uint32_t subgraph_id;
    /**< Size of the subgraph calibration data*/
    uint32_t data_size;
    /**< [out] Subgraph calibration data. Format:
    [module_id, param_id, param_size, err_code, payload[param_size]]
    */
    uint8_t *subgraph_cal_data;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_SET_CAL_DATA_PERSISTENT */

/* ---------------------------------------------------------------------------
* QTS_CMD_MCS_PLAY Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_MCS_PLAY
@{ */

/**
    Start a playback session with the provided usecase, calibration, and
    audio file.

    @param[in] cmd_id
        Command ID is QTS_CMD_MCS_PLAY.
    @param[in] cmd
        Pointer to QtsCmdMcsPlayReq.
    @param[in] cmd_size
        Size of QtsCmdMcsPlayReq.
    @param[out] rsp
        There is no ouput structure; set this to NULL.
    @param[in] rsp_size
        There is no ouput structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_MCS_PLAY QTS_MCS_CMD_ID(1)

/**< Specifies the set of playback modes used in the QTS_CMD_MCS_PLAY request*/
typedef enum QTS_MCS_PLAYBACK_MODE
{
    QTS_PLAYBACK_REGULAR,
    QTS_PLAYBACK_ANC_S_PATH,
    QTS_PLAYBACK_ANC_E_PATH,
    QTS_PLAYBACK_RAS,
} QtsMcsPlaybackMode;

/**< Media properties for a device or stream*/
typedef struct qts_mcs_properties_t McsProperties;
#include "acdb_begin_pack.h"
struct qts_mcs_properties_t
{
    /**< The device/stream sample rate*/
    uint32_t sample_rate;
    /**< The bit width of each sample*/
    uint16_t bit_width;
    /**< Indicates the alignment of bits_per_sample in sample_word_size.Relevant only when bits_per_sample is 24 and word_size is 32*/
    uint16_t alignment;
    /**< Bits needed to store one sample*/
    uint16_t bits_per_sample;
    /**< Q factor of the PCM data*/
    uint16_t q_factor;
    /**< Indicates whether PCM samples are stored in little endian or big endian format.*/
    uint16_t endianness;
    /**< Number of channels*/
    uint16_t num_channels;
    /**< Determines what channels are used. Up to 32 channels are supported. If not supported set to 0*/
    uint8_t *channel_mapping;
}
#include "acdb_end_pack.h"
;

/**< The request structure for QTS_CMD_MCS_PLAY*/
typedef struct qts_cmd_mcs_play_req_t QtsCmdMcsPlayReq;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_play_req_t
{
    /**< playback device properties*/
    McsProperties device_properties;
    /**< playback stream properties*/
    McsProperties stream_properties;
    /**< Graph Key Vector representing the playback usecase*/
    AcdbGraphKeyVector graph_key_vector;
    /**< Type of playback: 0(regular), 1(anc_spath), 2(anc_epath), 3(ras)*/
    QtsMcsPlaybackMode playback_mode;
    /**< Playback duration in seconds*/
    int32_t playback_duration_sec;
    /**< Length of the file name/path*/
    int32_t filename_len;
    /**< Name of/path to file to save recording to*/
    char filename[QTS_MAX_FILENAME_LENGTH];
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_MCS_PLAY */

/* ---------------------------------------------------------------------------
* QTS_CMD_MCS_RECORD Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_MCS_RECORD
@{ */

/**
    Start a record session and save the output to the specified file using
    the provided record usecase, duration, and calibration.

    @param[in] cmd_id
        Command ID is QTS_CMD_MCS_RECORD.
    @param[in] cmd
        Pointer to QtsCmdMcsRecordReq.
    @param[in] cmd_size
        Size of QtsCmdMcsRecordReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_MCS_RECORD QTS_MCS_CMD_ID(2)

/**< Specifies the set of recording modes used in the QTS_CMD_MCS_RECORD request*/
typedef enum QTS_MCS_RECORD_MODE
{
    QTS_RECORD_REGULAR,
    QTS_RECORD_ANC_P_PATH,
    QTS_RECORD_ANC_S_PATH,
    QTS_RECORD_ANC_E_PATH
} QtsMcsRecordMode;

/**< The request structure for QTS_CMD_MCS_RECORD*/
typedef struct qts_cmd_mcs_record_req_t QtsCmdMcsRecordReq;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_record_req_t
{
    /**< record device properties*/
    McsProperties device_properties;
    /**< record stream properties*/
    McsProperties stream_properties;
    /**< Graph Key Vector representing the record usecase*/
    AcdbGraphKeyVector graph_key_vector;
    /**< Type of record: 0(regular), 1(aanc_path), 2(aanc_spath), 3(aanc_epath)*/
    QtsMcsRecordMode record_mode;
    /**< Save recording to file: 0 (ignore write), 1 (write data)*/
    uint32_t write_to_file;
    /**< Record duration in seconds*/
    int32_t record_duration_sec;
    /**< Length of the file name/path*/
    int32_t filename_len;
    /**< Name of/path to file to save recording to*/
    char filename[QTS_MAX_FILENAME_LENGTH];
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_MCS_RECORD */

/* ---------------------------------------------------------------------------
* QTS_CMD_MCS_PLAY_RECORD Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_MCS_PLAY_RECORD
@{ */

/**
    Start playback and record simultaneously.

    @param[in] cmd_id
        Command ID is QTS_CMD_MCS_PLAY_RECORD.
    @param[in] cmd
        Pointer to QtsCmdMcsPlayRecordReq.
    @param[in] cmd_size
        Size of QtsCmdMcsPlayRecordReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
        - QtsCmdMcsPlayReq
        - QtsCmdMcsRecordReq
*/
#define QTS_CMD_MCS_PLAY_RECORD QTS_MCS_CMD_ID(3)
/**< The request structure for QTS_CMD_MCS_PLAY_RECORD*/
typedef struct qts_cmd_mcs_play_record_req_t QtsCmdMcsPlayRecordReq;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_play_record_req_t
{
    QtsCmdMcsPlayReq playback_session;
    QtsCmdMcsRecordReq record_session;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_MCS_PLAY_RECORD */

/* ---------------------------------------------------------------------------
* QTS_CMD_MCS_STOP Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_MCS_STOP
@{ */

/**
    Stop the active playback, record, or playback-record session. Releases all
    resources allocated to MCS.

    @param[in] cmd_id
        Command ID is QTS_CMD_MCS_STOP.
    @param[in] cmd
        There is no intput structure; set this to NULL.
    @param[in] cmd_size
        There is no intput structure; set this to 0.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.
    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_MCS_STOP QTS_MCS_CMD_ID(4)
/** @} */ /* end_addtogroup QTS_CMD_MCS_STOP */

/* ---------------------------------------------------------------------------
* QTS_CMD_FTS_OPEN_FILE Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_FTS_OPEN_FILE
@{ */

/**
    Open or create a file on the file system with write permissions.

    @param[in] cmd_id
        Command ID is QTS_CMD_FTS_OPEN_FILE.
    @param[in] cmd
        Pointer to QtsCmdFtsOpenFileReq.
    @param[in] cmd_size
        Size of QtsCmdFtsOpenFileReq.
    @param[out] rsp
        4 byte file index
    @param[in] rsp_size
        Size of uint32_t

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_FTS_OPEN_FILE QTS_FTS_CMD_ID(1)
/**< The request structure for QTS_CMD_FTS_OPEN_FILE*/
typedef struct qts_cmd_fts_open_file_req_t QtsCmdFtsOpenFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_open_file_req_t
{
    /**< File path length*/
    uint32_t file_path_length;
    /**< File path*/
    char *file_path;
    /**< Casa file access flag*/
    uint32_t access;
}
#include "acdb_end_pack.h"
;

/** @} */ /* end_addtogroup QTS_CMD_FTS_OPEN_FILE */

/* ---------------------------------------------------------------------------
* QTS_CMD_FTS_WRITE_FILE Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_FTS_WRITE_FILE
@{ */

/**
    Writes data to a file using the provided file handle.

    @param[in] cmd_id
        Command ID is QTS_CMD_FTS_WRITE_FILE.
    @param[in] cmd
        Pointer to QtsCmdFtsWriteFileReq.
    @param[in] cmd_size
        Size of QtsCmdFtsWriteFileReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
        - QTS_CMD_FTS_OPEN_FILE
*/
#define QTS_CMD_FTS_WRITE_FILE QTS_FTS_CMD_ID(2)
/**< The request structure for QTS_CMD_FTS_WRITE_FILE*/
typedef struct qts_cmd_fts_write_file_req_t QtsCmdFtsWriteFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_write_file_req_t
{
    /**< File index for a file to write to*/
    uint32_t findex;
    /**< [in] Size of the data to write*/
    uint32_t write_size;
    /**< Pointer to the data to be written*/
    void *buf_ptr;
    /**< [out] Number of bytes that are written*/
    uint32_t *bytes_writen;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_FTS_WRITE_FILE */

/* ---------------------------------------------------------------------------
* QTS_CMD_FTS_CLOSE_FILE Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_FTS_CLOSE_FILE
@{ */

/**
    Closes a file using the provided file handle.

    @param[in] cmd_id
        Command ID is QTS_CMD_FTS_CLOSE_FILE.
    @param[in] cmd
        Pointer to QtsCmdFtsCloseFileReq.
    @param[in] cmd_size
        Size of QtsCmdFtsCloseFileReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
        - QTS_CMD_FTS_OPEN_FILE
*/
#define QTS_CMD_FTS_CLOSE_FILE QTS_FTS_CMD_ID(3)
/**< The request structure for QTS_CMD_FTS_CLOSE_FILE*/
typedef struct qts_cmd_fts_close_file_req_t QtsCmdFtsCloseFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_close_file_req_t
{
    /**< File index for a file to close*/
    uint32_t findex;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_FTS_CLOSE_FILE */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_GET_VERSION Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_GET_VERSION
@{ */

/**
    Queries QTS for the ADIE(Codec) Service version.

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_GET_VERSION.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        Pointer to QtsVersion
    @param[in] rsp_size
        Size of QtsVersion

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ADIE_GET_VERSION QTS_CODEC_RTC_CMD_ID(1)
/** @} */ /* end_addtogroup QTS_CMD_ADIE_GET_VERSION */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_GET_CODEC_INFO Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_GET_CODEC_INFO
@{ */

/**
    Queries QTS for the Real Time Tuning service version.

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_GET_CODEC_INFO.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        Pointer to QtsCmdAdieGetCodecInfoRsp
    @param[in] rsp_size
        Size of QtsCmdAdieGetCodecInfoRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ADIE_GET_CODEC_INFO QTS_CODEC_RTC_CMD_ID(2)
/**< Holds the chipset ID and version information for a codec*/
typedef struct codec_properties_t CodecProperties;
#include "acdb_begin_pack.h"
struct codec_properties_t
{
    /**< Codec handle*/
    uint32_t handle;
    /**< ID of the codec chipset*/
    uint32_t chipset_id;
    /**< Major version of the chipset*/
    uint32_t chipset_major;
    /**< Minor version of the chipset*/
    uint32_t chipset_minor;
}
#include "acdb_end_pack.h"
;

/**< The response structure for QTS_CMD_ADIE_GET_CODEC_INFO.*/
typedef struct qts_cmd_adie_get_codec_info_rsp_t QtsCmdAdieGetCodecInfoRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_adie_get_codec_info_rsp_t
{
    /**< Number of codec properties*/
    uint32_t property_count;
    /**< List of codec properties*/
    CodecProperties *codec_properties;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ADIE_GET_CODEC_INFO */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_GET_REGISTER Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_GET_REGISTER
@{ */

/**
    Queries QTS for the register value for a given register.

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_GET_REGISTER.
    @param[in] cmd
        Pointer to QtsCmdAdieGetRegisterReq.
    @param[in] cmd_size
        Size of QtsCmdAdieGetRegisterReq.
    @param[out] rsp
        Pointer to QtsCmdAdieGetRegisterRsp.
    @param[in] rsp_size
        Size of QtsCmdAdieGetRegisterRsp.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ADIE_GET_REGISTER QTS_CODEC_RTC_CMD_ID(3)
/**< The request structure for QTS_CMD_ADIE_GET_REGISTER.*/
typedef struct qts_cmd_adie_get_register_req_t QtsCmdAdieGetRegisterReq;
#include "acdb_begin_pack.h"
struct qts_cmd_adie_get_register_req_t
{
    /**< Codec handle*/
    uint32_t codec_handle;
    /**< ID of the register*/
    uint32_t register_id;
    /**< The bit-mask for the register*/
    uint32_t register_mask;
}
#include "acdb_end_pack.h"
;

/**< The response structure for QTS_CMD_ADIE_GET_REGISTER.*/
typedef struct qts_cmd_adie_get_register_rsp_t QtsCmdAdieGetRegisterRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_adie_get_register_rsp_t
{
    /**< The register value*/
    uint32_t register_value;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ADIE_GET_REGISTER */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_GET_MULTIPLE_REGISTER Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_GET_MULTIPLE_REGISTER
@{ */

/**
    Calls QTS_CMD_ADIE_GET_REGISTER for each ADIE register

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_GET_MULTIPLE_REGISTER.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
        - QTS_CMD_ADIE_GET_REGISTER
*/
#define QTS_CMD_ADIE_GET_MULTIPLE_REGISTER QTS_CODEC_RTC_CMD_ID(4)
/** @} */ /* end_addtogroup QTS_CMD_ADIE_GET_MULTIPLE_REGISTER */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_SET_REGISTER Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_SET_REGISTER
@{ */

/**
    Set the register value for the given ADIE register.

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_SET_REGISTER.
    @param[in] cmd
        Pointer to QtsCmdAdieSetRegisterReq.
    @param[in] cmd_size
        Size of QtsCmdAdieSetRegisterReq.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
        - TODO
*/
#define QTS_CMD_ADIE_SET_REGISTER QTS_CODEC_RTC_CMD_ID(5)
/**< The request structure for QTS_CMD_ADIE_SET_REGISTER.*/
typedef struct qts_cmd_adie_set_register_req_t QtsCmdAdieSetRegisterReq;
#include "acdb_begin_pack.h"
struct qts_cmd_adie_set_register_req_t
{
    /**< Codec handle*/
    uint32_t codec_handle;
    /**< ID of the register*/
    uint32_t register_id;
    /**< The bit-mask for the register*/
    uint32_t register_mask;
    /**< The value stored in the register*/
    uint32_t register_value;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ADIE_SET_REGISTER */

/* ---------------------------------------------------------------------------
* QTS_CMD_ADIE_SET_MULTIPLE_REGISTER Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ADIE_SET_MULTIPLE_REGISTER
@{ */

/**
    Calls QTS_CMD_ADIE_SET_REGISTER for each ADIE register

    @param[in] cmd_id
        Command ID is QTS_CMD_ADIE_SET_MULTIPLE_REGISTER.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ADIE_SET_MULTIPLE_REGISTER QTS_CODEC_RTC_CMD_ID(6)
/** @} */ /* end_addtogroup QTS_CMD_ADIE_SET_MULTIPLE_REGISTER */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_SERVICE_INFO Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_SERVICE_INFO
@{ */

/**
    Queries the Online Calibration Service Version

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_GET_SERVICE_INFO.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to QtsCmdGetServiceInfoRsp
    @param[in] rsp_size
        There is the size of QtsCmdGetServiceInfoRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ONC_GET_SERVICE_INFO QTS_ONLINE_CMD_ID(1)
/**< Holds the service information of a service*/
typedef struct qts_service_info_t QtsServiceInfo;
#include "acdb_begin_pack.h"
struct qts_service_info_t
{
    /**< Service ID*/
    uint32_t service_id;
    /**< Major version*/
    uint32_t major;
    /**< Minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;

/**< The response structure for retrieving information about the registered services*/
typedef struct qts_cmd_get_service_info_rsp_t QtsCmdGetServiceInfoRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_get_service_info_rsp_t
{
    /**< Number of QTS services*/
    uint32_t service_count;
    /**< Array of service information*/
    QtsServiceInfo *services_info;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_SERVICE_INFO */

/*----------------------------------------------------------------------------
* QTS_CMD_ONC_CHECK_CONNECTION Declarations and Documentation
*---------------------------------------------------------------------------*/

/** @addtogroup QTS_CMD_ONC_CHECK_CONNECTION
@{ */

/**
    Checks to see if a connection between QTS and its client can be
    established or is still active.

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_CHECK_CONNECTION.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to a uint32_t
    @param[in] rsp_size
        This is the size of a uint32_t

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_CHECK_CONNECTION QTS_ONLINE_CMD_ID(2)
/** @} */ /* end_addtogroup QTS_CMD_ONC_CHECK_CONNECTION */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_MAX_BUFFER_LENGTH Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_MAX_BUFFER_LENGTH
@{ */

/**
    Queries QTS for its maximum buffer length.

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_GET_MAX_BUFFER_LENGTH.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to a uint32_t
    @param[in] rsp_size
        This is the size of a uint32_t

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_MAX_BUFFER_LENGTH QTS_ONLINE_CMD_ID(3)
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_MAX_BUFFER_LENGTH */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_SET_MAX_BUFFER_LENGTH Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_SET_MAX_BUFFER_LENGTH
@{ */

/**
    Set the maximum buffer size to be used by QTS

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_SET_MAX_BUFFER_LENGTH.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_SET_MAX_BUFFER_LENGTH QTS_ONLINE_CMD_ID(4)
/** @} */ /* end_addtogroup QTS_CMD_ONC_SET_MAX_BUFFER_LENGTH */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_ACDB_FILES_INFO Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_ACDB_FILES_INFO
@{ */

/**
    Queries QTS for ACDB file information for all ACDB files loaded into RAM.
    File info includes the ACDB file name and file size for each ACDB file.

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_GET_ACDB_FILES_INFO.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to QtsCmdGetAcdbFileInfoRsp
    @param[in] rsp_size
        This is the size of QtsCmdGetAcdbFileInfoRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_ACDB_FILES_INFO QTS_ONLINE_CMD_ID(5)

/**< Holds basic Acdb file information */
typedef struct qts_get_acdb_file_info_t QtsGetAcdbFileInfo;
#include "acdb_begin_pack.h"
struct qts_get_acdb_file_info_t
{
    /**< Acdb file size*/
    uint32_t file_size;
    /**< Acdb file name length*/
    uint32_t file_name_len;
    /**< Acdb file name*/
    char* file_name;
}
#include "acdb_end_pack.h"
;

/**< The response structure for retrieving information about the Acdb files loaded in memory*/
typedef struct qts_cmd_get_acdb_file_info_rsp_t QtsCmdGetAcdbFileInfoRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_get_acdb_file_info_rsp_t
{
    /**< Number of acdb files*/
    uint32_t file_count;
    /**< Array of acdb file info*/
    QtsGetAcdbFileInfo *files_info;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_ACDB_FILES_INFO */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_ACDB_FILE Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_ACDB_FILE
@{ */

/**
    Downloads the requested ACDB data and delta data file(s) from the target
    to the client. The client retrieves the file name from
    QTS_CMD_ONC_GET_ACDB_FILES_INFO

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_GET_ACDB_FILE.
    @param[in] cmd
        There is a pointer to QtsOcsGetAcdbFile
    @param[in] cmd_size
        There is the size of QtsOcsGetAcdbFile
    @param[out] rsp
        This is a pointer to AcdbBlob
    @param[in] rsp_size
        This is the size of AcdbBlob

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
    qts_online_ioctl
    QTS_CMD_ONC_GET_ACDB_FILES_INFO
*/
#define QTS_CMD_ONC_GET_ACDB_FILE QTS_ONLINE_CMD_ID(6)

typedef struct _qts_ocs_get_acdb_file_req_t QtsOcsGetAcdbFile;
#include "acdb_begin_pack.h"
struct _qts_ocs_get_acdb_file_req_t
{
    /**<File offset to start reading from*/
    uint32_t file_offset;
    /**<The size of the file read*/
    uint32_t file_read_size;
    /**<The length of the file name*/
    uint32_t file_name_len;
    /**<The name of the file to retrieve data for*/
    uint8_t* file_name;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_ACDB_FILE */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_HEAP_ENTRY_INFO Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_HEAP_ENTRY_INFO
@{ */

/**
    Get information about the entries on the ACDB heap

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_GET_HEAP_ENTRY_INFO.
    @param[in] cmd
        There is a pointer to
    @param[in] cmd_size
        There is the size of
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
    qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_HEAP_ENTRY_INFO QTS_ONLINE_CMD_ID(7)
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_HEAP_ENTRY_INFO */

/* ---------------------------------------------------------------------------
* #define QTS_CMD_ONC_GET_HEAP_ENTRY_DATA QTS_ONLINE_CMD_ID(8)
 Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup #define QTS_CMD_ONC_GET_HEAP_ENTRY_DATA QTS_ONLINE_CMD_ID(8)

@{ */

/**
    Queries for the requested heap entries from the ACDB heap.

    @param[in] cmd_id
        Command ID is #define QTS_CMD_ONC_GET_HEAP_ENTRY_DATA QTS_ONLINE_CMD_ID(8)
.
    @param[in] cmd
        There is a pointer to QtsOcsGetAcdbFile
    @param[in] cmd_size
        There is the size of QtsOcsGetAcdbFile
    @param[out] rsp
        This is a pointer to AcdbBlob
    @param[in] rsp_size
        This is the size of AcdbBlob

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa
    qts_online_ioctl
    QTS_CMD_ONC_GET_HEAP_ENTRY_INFO
*/
#define QTS_CMD_ONC_GET_HEAP_ENTRY_DATA QTS_ONLINE_CMD_ID(8)
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_HEAP_ENTRY_DATA */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_CAL_DATA_NON_PERSIST Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_CAL_DATA_NON_PERSIST
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_GET_CAL_DATA_NON_PERSIST
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_CAL_DATA_NON_PERSIST QTS_ONLINE_CMD_ID(9)

typedef struct _qts_get_subgraph_cal_data_req_t QtsGetSubgraphCalDataReq;
#include "acdb_begin_pack.h"
struct _qts_get_subgraph_cal_data_req_t {
    uint32_t subgraph_id;
    uint32_t module_iid;
    uint32_t param_id;
    AcdbGraphKeyVector cal_key_vector;
}
#include "acdb_end_pack.h"
;

typedef struct _qts_set_subgraph_cal_data_req_t QtsSetSubgraphCalDataReq;
#include "acdb_begin_pack.h"
struct _qts_set_subgraph_cal_data_req_t {
    uint32_t subgraph_id;
    uint32_t module_iid;
    uint32_t param_id;
    AcdbGraphKeyVector cal_key_vector;
    AcdbBlob payload;
}
#include "acdb_end_pack.h"
;

/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_CAL_DATA_NON_PERSIST */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_SET_CAL_DATA_NON_PERSIST Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_SET_CAL_DATA_NON_PERSIST
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_SET_CAL_DATA_NON_PERSIST
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_SET_CAL_DATA_NON_PERSIST QTS_ONLINE_CMD_ID(10)
/** @} */ /* end_addtogroup QTS_CMD_ONC_SET_CAL_DATA_NON_PERSIST */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_CAL_DATA_PERSIST Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_CAL_DATA_PERSIST
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_GET_CAL_DATA_PERSIST
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_CAL_DATA_PERSIST QTS_ONLINE_CMD_ID(11)
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_CAL_DATA_PERSIST */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_SET_CAL_DATA_PERSIST Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_SET_CAL_DATA_PERSIST
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_SET_CAL_DATA_PERSIST
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_SET_CAL_DATA_PERSIST QTS_ONLINE_CMD_ID(12)
/** @} */ /* end_addtogroup QTS_CMD_ONC_SET_CAL_DATA_PERSIST */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_GET_TAG_DATA Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_GET_TAG_DATA
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_GET_TAG_DATA
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_GET_TAG_DATA QTS_ONLINE_CMD_ID(13)

typedef struct _qts_get_subgraph_tag_data_req_t QtsGetSubgraphTagDataReq;
#include "acdb_begin_pack.h"
struct _qts_get_subgraph_tag_data_req_t {
    uint32_t subgraph_id;
    uint32_t tag_id;
    uint32_t module_iid;
    uint32_t param_id;
    AcdbGraphKeyVector tag_key_vector;
}
#include "acdb_end_pack.h"
;

typedef struct _qts_set_subgraph_tag_data_req_t QtsSetSubgraphTagDataReq;
#include "acdb_begin_pack.h"
struct _qts_set_subgraph_tag_data_req_t {
    uint32_t subgraph_id;
    uint32_t tag_id;
    uint32_t module_iid;
    uint32_t param_id;
    AcdbGraphKeyVector cal_key_vector;
    AcdbBlob payload;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_ONC_GET_TAG_DATA */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_SET_TAG_DATA Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_SET_TAG_DATA
@{ */

/**
    TBD

    @param[in] cmd_id QTS_CMD_ONC_SET_TAG_DATA
        Command ID is
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to
    @param[in] rsp_size
        This is the size of

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_SET_TAG_DATA QTS_ONLINE_CMD_ID(14)
/** @} */ /* end_addtogroup QTS_CMD_ONC_SET_TAG_DATA */
/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_DELETE_DELTA_FILES Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_DELETE_DELTA_FILES
@{ */

/**
    Removes the *.acdbdelta files from the targets file system

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_DELETE_DELTA_FILES.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is no output structure; set this to NULL.
    @param[in] rsp_size
        This is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_DELETE_DELTA_FILES QTS_ONLINE_CMD_ID(15)
/** @} */ /* end_addtogroup QTS_CMD_ONC_DELETE_DELTA_FILES */

/* ---------------------------------------------------------------------------
* QTS_CMD_ONC_IS_DELTA_DATA_SUPPORTED Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_ONC_IS_DELTA_DATA_SUPPORTED
@{ */

/**
    Determines whether the target has ACDB Delta data persistence enabled or
    disabled.

    Is persistance supported? Zero for supported and non-zero for unsupported.

    @param[in] cmd_id
        Command ID is QTS_CMD_ONC_IS_DELTA_DATA_SUPPORTED.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        This is a pointer to a uint32_t
    @param[in] rsp_size
        This is the size of QtsCmdGetAcdbFileInfoRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa qts_online_ioctl
*/
#define QTS_CMD_ONC_IS_DELTA_DATA_SUPPORTED QTS_ONLINE_CMD_ID(16)
/** @} */ /* end_addtogroup QTS_CMD_ONC_IS_DELTA_DATA_SUPPORTED */

/* ---------------------------------------------------------------------------
* Public API Definitions
*-------------------------------------------------------------------------- */

/**
* \brief QTS_CALLBACK
*       A function callback for used for various QTS services
* \param [in] svc_cmd_id: The service command to be issued. Contains the 2 byte
        service id and 2byte command id
* \param [in/out] cmd_buf: Pointer to the command structure.
* \param [in] cmd_buf_size:
* \param [out] rsp_buf: The response structre
* \param [in] rsp_buf_size: The size of the response
* \param [out] rsp_buf_bytes_filled: Number of bytes written to the response buffer
* \return 0 on success, non-zero on failure
*/
typedef int32_t(*QTS_CALLBACK)(
    uint32_t svc_cmd_id,
    uint8_t *cmd_buf,
    uint32_t cmd_buf_size,
    uint8_t *rsp_buf,
    uint32_t rsp_buf_size,
    uint32_t *rsp_buf_bytes_filled);

typedef int32_t(*QTS_VERSION_CALLBACK)(
    QtsServiceInfo *service_info);

typedef struct _qts_callback_table_t QtsCallbackTable;
struct _qts_callback_table_t
{
    QTS_CALLBACK *service_callback;
    QTS_VERSION_CALLBACK *version_callback;
};

/**
* \brief qts_init
*       Initializes the QACT Tuning Service.
*
* \return 0 on success, non-zero on failure
*/
int32_t qts_init(void);

/**
* \brief qts_deinit
*       Resets the QACT Tuning Service.
*
* \return 0 on success, non-zero on failure
*/
int32_t qts_deinit(void);

/**
* \brief qts_register_service
*       Registers the specified service.
* \param [in] servie_id: The service ID of a QTS service
* \param [in] service_callback: The callback used to execute commands for the service.
* \return 0 on success, non-zero on failure
*/
int32_t qts_register_service(uint32_t service_id, QTS_CALLBACK service_callback);

/**
* \brief qts_deregister_service
*       Deregisters the specified service.
* \param [in] servie_id: The service ID of a QTS service
* \return 0 on success, non-zero on failure
*/
int32_t qts_deregister_service(uint32_t servie_id);

/**
* \brief qts_get_service_info
*       Get service information for each service registered
* \param [in] servie_id: The service ID of a QTS service
* \return 0 on success, non-zero on failure
*/
int32_t qts_get_service_info(QtsCmdGetServiceInfoRsp *svc_info_rsp, uint32_t rsp_buf_len);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /*__QTS_H__*/
