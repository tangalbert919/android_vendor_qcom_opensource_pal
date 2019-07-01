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
/* $Header: //source/qcom/qct/multimedia2/Audio/audcal4/acdb_sw/dev/casa/qts/api/qts.h#5 $ */

/*------------------------------------------
* Includes
*------------------------------------------*/
#include "casa_osal_error.h"
#include "casa_osal_types.h"
#include "acdb.h"
#include "acdb_common.h"
#include "acdb_utility.h"


#ifdef __cplusplus
//extern "C" {
#endif /*__cplusplus*/

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

/*------------------------------------------
* Type Declarations
*------------------------------------------*/
#define QTS_MAJOR_VERSION 0x1
#define QTS_MINOR_VERSION 0x0
#define QTS_REVISION_VERSION 0x0

/**< Defines command IDs for the Real Time Tuning service */
#define QTS_RT_SERVICE_ID (0x5254 << 16)
#define QTS_RT_CMD_ID(x) ((0x5254 << 16) | (0xFFFF & x))


/**< Defines command IDs for the File Transfer service */
#define QTS_FTS_CMD_ID(x) ((0x000B << 16) | (0xFFFF & x))
#define QTS_FTS_SERVICE_ID (0x000B << 16)

/**< Defines command IDs for the Media Control Service */
#define QTS_MCS_CMD_ID(x) ((0x000C << 16) | (0xFFFF & x))
#define QTS_MCS_SERVICE_ID (0x000C << 16)

/**< Defines command IDs for the Adie Codec service */
#define QTS_CODEC_CMD_ID(x) ((0x000D << 16) | (0xFFFF & x))
#define QTS_CODEC_SERVICE_ID (0x000D << 16)

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
#define QTS_CMD_RT_GET_VERSION QTS_RT_CMD_ID(1)
/**< The response structure for QTS_CMD_RT_GET_VERSION. Holds the version information for the Real Time Tuning service*/
typedef struct qts_cmd_rt_get_version_t QtsCmdRtGetVersionRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_rt_get_version_t
{
    /**< QTS SW major version*/
    uint32_t major;
    /**< QTS SW minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_RT_GET_VERSION */

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
#define QTS_CMD_RT_GET_ACTIVE_INFO QTS_RT_CMD_ID(2)
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
#define QTS_CMD_RT_GET_CAL_DATA_NON_PERSISTENT QTS_RT_CMD_ID(3)
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
#define QTS_CMD_RT_GET_CAL_DATA_PERSISTENT QTS_RT_CMD_ID(4)
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
#define QTS_CMD_RT_GET_CAL_DATA_GBL_PERSISTENT QTS_RT_CMD_ID(5)
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
#define QTS_CMD_RT_SET_CAL_DATA_NON_PERSISTENT QTS_RT_CMD_ID(6)
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
#define QTS_CMD_RT_SET_CAL_DATA_PERSISTENT QTS_RT_CMD_ID(7)
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
* QTS_CMD_MCS_GET_VERSION Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_MCS_GET_VERSION
@{ */

/**
    Queries QTS for the Media Control Service version.

    @param[in] cmd_id
        Command ID is QTS_CMD_MCS_GET_VERSION.
    @param[in] cmd
        There is no input structure; set this to NULL;
    @param[in] cmd_size
        There is no input structure; set this to 0;
    @param[out] rsp
        Pointer to QtsCmdMcsGetVersionRsp
    @param[in] rsp_size
        Size of QtsCmdMcsGetVersionRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_MCS_GET_VERSION QTS_MCS_CMD_ID(1)
/**< The response structure for QTS_CMD_MCS_GET_VERSION. Holds the version information for the Media Control service*/
typedef struct qts_cmd_mcs_get_version_rsp_t QtsCmdMcsGetVersionRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_get_version_rsp_t
{
    /**< MCS major version*/
    uint32_t major;
    /**< MCS minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_MCS_GET_VERSION */

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
#define QTS_CMD_MCS_PLAY QTS_MCS_CMD_ID(2)

typedef struct qts_stream_properties_t StreamProperties;
#include "acdb_begin_pack.h"
struct qts_stream_properties_t
{
    /**< The stream sample rate*/
    uint32_t sample_rate;
    /**< Number of channels used by the stream*/
    uint32_t num_channels;
    /**< Determines what channels are used by the stream. Up to 32 channels are supported*/
    uint32_t channel_mask;
    /**< The bit rate used by the stream which determines the size of a sample*/
    uint32_t bit_width;
}
#include "acdb_end_pack.h"
;

/**< The request structure for QTS_CMD_MCS_PLAY*/
typedef struct qts_cmd_mcs_play_req_t QtsCmdMcsPlayReq;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_play_req_t
{
    /**< Properties used by the playback stream*/
    StreamProperties stream_properties;
    /**< Graph Key Vector representing the playback usecase*/
    AcdbGraphKeyVector graph_key_vector;
    /**< Type of playback: 1 (regular), 2(anc)*/
    uint32_t playback_mode;
    /**< Playback duration in seconds*/
    int32_t playback_duration_sec;
    /**< Length of the file name/path*/
    int32_t filename_len;
    /**< Name of/path to file to be played*/
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
#define QTS_CMD_MCS_RECORD QTS_MCS_CMD_ID(3)
/**< The request structure for QTS_CMD_MCS_RECORD*/
typedef struct qts_cmd_mcs_record_req_t QtsCmdMcsRecordReq;
#include "acdb_begin_pack.h"
struct qts_cmd_mcs_record_req_t
{
    /**< Properties used by the record stream*/
    StreamProperties stream_properties;
    /**< Graph Key Vector representing the record usecase*/
    AcdbGraphKeyVector graph_key_vector;
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
#define QTS_CMD_MCS_PLAY_RECORD QTS_MCS_CMD_ID(4)
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
#define QTS_CMD_MCS_STOP QTS_MCS_CMD_ID(5)
/** @} */ /* end_addtogroup QTS_CMD_MCS_STOP */

/* ---------------------------------------------------------------------------
* QTS_CMD_FTS_GET_VERSION Declarations and Documentation
*-------------------------------------------------------------------------- */

/** @addtogroup QTS_CMD_FTS_GET_VERSION
@{ */

/**
    Queries QTS for the File Transfer Service version.

    @param[in] cmd_id
        Command ID is QTS_CMD_FTS_GET_VERSION.
    @param[in] cmd
        There is no input structure; set this to NULL.
    @param[in] cmd_size
        There is no input structure; set this to 0.
    @param[out] rsp
        Pointer to QtsCmdFtsGetVersionRsp.
    @param[in] rsp_size
        Size of QtsCmdFtsGetVersionRsp.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_FTS_GET_VERSION QTS_FTS_CMD_ID(1)
/**< The response structure for QTS_CMD_FTS_GET_VERSION. Holds the version information for the File Transfer service*/
typedef struct qts_cmd_fts_get_version_rsp_t QtsCmdFtsGetVersionRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_get_version_rsp_t
{
    /**< FTS major version*/
    uint32_t major;
    /**< FTS minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;
/** @} */ /* end_addtogroup QTS_CMD_FTS_GET_VERSION */

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
        There is no output structure; set this to NULL.
    @param[in] rsp_size
        There is no output structure; set this to 0.

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_FTS_OPEN_FILE QTS_FTS_CMD_ID(2)
/**< The request structure for QTS_CMD_FTS_OPEN_FILE*/
typedef struct qts_cmd_fts_open_file_req_t QtsCmdFtsOpenFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_open_file_req_t
{
    /**< Pointer to a handle for a file to open*/
    casa_fhandle *fhandle;
    /**< File handle*/
    const char *file_path;
    /**< File handle*/
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
#define QTS_CMD_FTS_WRITE_FILE QTS_FTS_CMD_ID(3)
/**< The request structure for QTS_CMD_FTS_WRITE_FILE*/
typedef struct qts_cmd_fts_write_file_req_t QtsCmdFtsWriteFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_write_file_req_t
{
    /**< Handle for a file to write to*/
    casa_fhandle fhandle;
    /**< Pointer to the data to be written*/
    void *buf_ptr;
    /**< [in] Size of the data to write*/
    uint32_t write_size;
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
#define QTS_CMD_FTS_CLOSE_FILE QTS_FTS_CMD_ID(4)
/**< The request structure for QTS_CMD_FTS_CLOSE_FILE*/
typedef struct qts_cmd_fts_close_file_req_t QtsCmdFtsCloseFileReq;
#include "acdb_begin_pack.h"
struct qts_cmd_fts_close_file_req_t
{
    /**< Handle for a file to close*/
    casa_fhandle fhandle;
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
        Pointer to QtsCmdAdieGetVersionRsp
    @param[in] rsp_size
        Size of QtsCmdAdieGetVersionRsp

    @return
        - CASA_EOK -- Command executed successfully.
        - CASA_EBADPARAM -- Invalid input parameters were provided.
        - CASA_EFAILED -- Command execution failed.

    @sa TODO
*/
#define QTS_CMD_ADIE_GET_VERSION QTS_CODEC_CMD_ID(1)
/**< The response structure for QTS_CMD_ADIE_GET_VERSION. Holds the version information for the Codec service*/
typedef struct qts_cmd_adie_get_version_rsp_t QtsCmdAdieGetVersionRsp;
#include "acdb_begin_pack.h"
struct qts_cmd_adie_get_version_rsp_t
{
    /**< ADIE major version*/
    uint32_t major;
    /**< ADIE minor version*/
    uint32_t minor;
}
#include "acdb_end_pack.h"
;
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
#define QTS_CMD_ADIE_GET_CODEC_INFO QTS_CODEC_CMD_ID(2)
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
#define QTS_CMD_ADIE_GET_REGISTER QTS_CODEC_CMD_ID(3)
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
#define QTS_CMD_ADIE_GET_MULTIPLE_REGISTER QTS_CODEC_CMD_ID(4)
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
#define QTS_CMD_ADIE_SET_REGISTER QTS_CODEC_CMD_ID(5)
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
#define QTS_CMD_ADIE_SET_MULTIPLE_REGISTER QTS_CODEC_CMD_ID(6)
/** @} */ /* end_addtogroup QTS_CMD_ADIE_SET_MULTIPLE_REGISTER */

/**
* \brief QTS_CALLBACK
*       A function callback for used for various QTS services
* \param [in] cmd_id: The command to be issued
* \param [in/out] cmd_buf: Pointer to the command structure.
* \param [in] cmd_buf_size:
* \param [out] rsp_buf: The response structre
* \param [in] rsp_buf_size: The size of the response
* \param [out] rsp_buf_bytes_filled: Number of bytes written to the response buffer
* \return 0 on success, non-zero on failure
*/
typedef int32_t(*QTS_CALLBACK)(
    uint16_t cmd_id,
    uint8_t *cmd_buf,
    uint32_t cmd_buf_size,
    uint8_t *rsp_buf,
    uint32_t rsp_buf_size,
    uint32_t *rsp_buf_bytes_filled);

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
int32_t qts_register_service(uint32_t servie_id, QTS_CALLBACK service_callback);

/**
* \brief qts_deregister_service
*       Deregisters the specified service.
* \param [in] servie_id: The service ID of a QTS service
* \return 0 on success, non-zero on failure
*/
int32_t qts_deregister_service(uint32_t servie_id);

#ifdef __cplusplus
//}
#endif /*__cplusplus*/

#endif //_QTS_H_