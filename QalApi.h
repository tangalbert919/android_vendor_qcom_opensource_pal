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

/** \file qal_api.h
 *  \brief Function prototypes for the QAL(QTI Audio
 *   Layer).
 *
 *  This contains the prototypes for the QAL(QTI Audio
 *  layer) and  any macros, constants, or global variables
 *  needed.
 */

#ifndef QAL_API_H
#define QAL_API_H

#include "QalDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  Get QAL version in the form of Major and Minor number
 *  seperated by period.
 *
 *  @return the version string in the form of Major and Minor
 *  e.g '1.0'
 */
char* qal_get_version( );

/**
 *  Initialize QAL. Increases ref count.
 *
 *  @return 0 on success, error code on failure.
 */
int32_t qal_init( );

/**
 *  De-Initialize QAL. Decreases the ref count.
 */
void qal_deinit();

/**
  * \brief Open the stream with specified configuration.
  *
  * \param[in] attributes - Valid stream attributes obtained
  *       from qal_stream_open
  * \param[in] no_of_devices - no of audio devices that the
  *       stream should be initially started with.
  * \param[in] qal_device - an array of qal_devices. The size of
  *       the array is based on the no_of_devices specified by
  *       the client.
  *       If qal_media_config in qal_device is specified as NULL,
  *       QAL uses the default device configuration or appropriate
  *       configuration based on the usecases running.
  *       Clients can query the device configuration by using
  *       qal_get_device().
  * \param[in] no_of_modifiers - no of modifiers.
  * \param[in] modifiers - an array of modifiers. Modifiers are
  *       used to add additional key-value pairs. e.g to
  *       identify the topology of usecase from default set.
  * \param[in] cb - callback function associated with stream.
  *        Any event will notified through this callback
  *        function.
  * \param[in] cookie - client data associated with the stream.
  *       This cookie will be returned back in the callback
  *       function.
  * \param[out] stream_handle - Updated with valid stream handle
  *       if the operation is successful.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_open(struct qal_stream_attributes *attributes,
                        uint32_t no_of_devices, struct qal_device *devices,
                        uint32_t no_of_modifiers, struct modifier_kv *modifiers,
                        qal_stream_callback cb, void *cookie,
                        qal_stream_handle_t **stream_handle);

/**
  * \brief Close the stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_close(qal_stream_handle_t *stream_handle);

/**
  * \brief Register for specific event on a given stream. Events
  *        will be notified via the callback function registered
  *        in qal_stream_open cmd.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] event_id - Valid event id that the client would like notificaiton.
  * \param[in] event_data - Event configuration data.
  *
  * \return 0 on success, error code otherwise
  */
//int32_t qal_stream_register_for_event(qal_stream_handle_t *stream_handle,
  //                               uint32_t event_id, qal_event_cfg_t *event_cfg);

/**
  * \brief Start the stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_start(qal_stream_handle_t *stream_handle);

/**
  * \brief Stop the stream. Stream must be in started/paused
  *        state before stoping.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_stop(qal_stream_handle_t *stream_handle);

/**
  * \brief Pause the stream. Stream must be in started state
  *        before resuming.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_pause(qal_stream_handle_t *stream_handle);

/**
  * \brief Resume the stream. Stream must be in paused state
  *        before resuming.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_resume(qal_stream_handle_t *stream_handle);

/**
  * \brief Flush accumlated data from the stream. Stream must be
  *        in paused state before flushing.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_flush(qal_stream_handle_t *stream_handle);

/**
  * Drain audio data buffered by the driver/hardware has been
  * played depending on the drain type specified. If stream is
  * opened with AUDIO_STREAM_FLAG_NON_BLOCKING and callback
  * function is set in qal_open_stream(), drain complete
  * notificaiton will be sent via the callback function
  * otherwise will block until drain is completed.
  *
  * Drain will return immediately on stop() and flush() call.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] type - drain type, DRAIN or DRAIN_PARTIAL.
  *
  * \return 0 on success, error code otherwise
  */

int32_t qal_stream_drain(qal_stream_handle_t *stream_handle, qal_drain_type_t type);

/**
  * Get audio buffer size based on the direction of the stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open.
  * \param[out] in_buffer - filled if stream was opened with
  *       QAL_AUDIO_INPUT direction.
  * \param[out] out_buffer - filled if stream was opened with
  *       QAL_AUDIO_OUTPUT direction.
  *
  * \return - 0 on success, error code otherwise.
  */
int32_t qal_stream_get_buffer_size(qal_stream_handle_t *stream_handle,
                                   size_t *in_buffer, size_t *out_buffer);

/**
  * Set audio buffer size based on the direction of the stream.
  * This overwrites the default buffer size configured for
  * certain stream types.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open.
  * \param[in] in_buf_size - input buffer size when stream is
  *       opened with QAL_AUDIO_INPUT direction.
  * \param[in] in_buf_count - input buffer count when stream is
  *       opened with QAL_AUDIO_INPUT direction.
  * \param[in] out_buf_size - output buffer size when stream is
  *       opened with QAL_AUDIO_OUTPUT direction..
  * \param[in] out_buf_count - output buffer count when stream is
  *       opened with QAL_AUDIO_OUTPUT direction..
  *
  * \return - 0 on success, error code otherwise.
  */
int32_t qal_stream_set_buffer_size (qal_stream_handle_t *stream_handle, size_t *in_buf_size,
                                    const size_t in_buf_count, size_t *out_buf_size,
                                    const size_t out_buf_count);

/**
  * Read audio buffer captured from in the audio stream.
  * an error code.
  * Capture timestamps will be populated if session was
  * opened with timetamp flag.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] buf - pointer to qal_buffer containing audio
  *       samples and metadata.
  *
  * \return - number of bytes read or error code on failure
  */
ssize_t qal_stream_read(qal_stream_handle_t *stream_handle, struct qal_buffer *buf);

/**
  * Write audio buffer of a stream for rendering.If at least one
  * frame was written successfully prior to the error, QAL will
  * return number of bytes returned.
  *
  * Timestamp is honored if the stream was opened with timestamp
  * flag otherwise it is ignored.
  *
  * If the stream was opened with non-blocking mode, the write()
  * will operate in non-blocking mode. QAL will write only the
  * number of bytes that currently fit in the driver/hardware
  * buffer. If the callback function is set during
  * qal_stream_open, the callback function will be called when
  * more space is available in the driver/hardware buffer.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] buf - pointer to qal_buffer containing audio
  *       samples and metadata.
  *
  * \return number of bytes written or error code.
  */
ssize_t qal_stream_write(qal_stream_handle_t *stream_handle, struct qal_buffer *buf);

/**
  * \brief get current device on stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] no_of_devices - no of audio devices that the
  *       stream should be initially started with.
  * \param[in] qal_device - an array of qal_device. The size of
  *       the array is based on the no_of_devices the stream is
  *       associated.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_get_device(qal_stream_handle_t *stream_handle,
                            uint32_t no_of_devices, struct qal_device *devices);

/**
  * \brief set new device on stream. This api will disable the
  *        existing device and set the new device. If the new
  *        device is a combo device and includes previously set
  *        device, it will retain the old device to avoid
  *        setting the same device again unless device
  *        configuration chagnes.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] no_of_devices - no of audio devices that the
  *       stream should be initially started with.
  * \param[in] qal_device - an array of qal_device. The size of
  *       the array is based on the no_of_devices specified by
  *       the client.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_set_device(qal_stream_handle_t *stream_handle,
                           uint32_t no_of_devices, struct qal_device *devices);

/**
  * \brief Get audio parameters specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] param_id - param id whose parameters are
  *       retrieved.
  * \param[out] param_payload - param data applicable to the
  *       param_id
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_get_param(qal_stream_handle_t *stream_handle,
                           uint32_t param_id, qal_param_payload **param_payload);

/**
  * \brief Set audio parameters specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] param_id - param id whose parameters are to be
  *       set.
  * \param[out] param_payload - param data applicable to the
  *       param_id
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_set_param(qal_stream_handle_t *stream_handle,
                           uint32_t param_id, qal_param_payload *param_payload);

/**
  * \brief Get audio volume specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] volume - volume data to be set on a stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_get_volume(qal_stream_handle_t *stream_handle,
                              struct qal_volume_data *volume);

/**
  * \brief Set audio volume specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[out] volume - volume data to be retrieved from the
  *       stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_set_volume(qal_stream_handle_t *stream_handle,
                              struct qal_volume_data *volume);

/**
  * \brief Get current audio audio mute state to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] mute - mute state to be retrieved from the
  *       stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_get_mute(qal_stream_handle_t *stream_handle, bool *state);

/**
  * \brief Set mute specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[out] mute - mute state to be set to the stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_set_mute(qal_stream_handle_t *stream, bool state);

/**
  * \brief Get microphone mute state.
  *
  *\param[out] mute - global mic mute flag to be retrieved.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_get_mic_mute(bool *state);

/**
  * \brief Set global mic mute state.
  *
  * \param[out] mute - global mic mute state
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_set_mic_mute(bool state);

/**
  * \brief Get time stamp.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[out] stime - time stamp data to be retrieved from the
  *       stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_get_timestamp(qal_stream_handle_t *stream_handle, struct qal_session_time *stime);

/**
  * \brief Add remove effects for Voip TX path.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] effect - effect to be enabled or disable
  * \param[in] enable - enable/disable
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_add_remove_effect(qal_stream_handle_t *stream_handle, qal_audio_effect_t effect, bool enable);

/**
  * \brief Set qal parameters
  *
  * \param[in] param_id - param id whose parameters are to be
  *       set.
  * \param[in] param_payload - param data applicable to the
  *       param_id
  * \param[in] payload_size - size of payload
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_set_param(uint32_t param_id, void *param_payload, size_t payload_size);

/**
  * \brief Get qal parameters
  *
  *
  * \param[in] param_id - param id whose parameters are
  *       retrieved.
  * \param[out] param_payload - param data applicable to the
  *       param_id
  * \param[out] payload_size - size of payload
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_get_param(uint32_t param_id, void **param_payload,
                        size_t *payload_size, void *query);

/**
  * \brief Set audio volume specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[in] min_size_frames - minimum frame size required.
  * \param[out] info - map buffer descriptor returned by
  *       stream.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_create_mmap_buffer(qal_stream_handle_t *stream_handle,
                              int32_t min_size_frames,
                              struct qal_mmap_buffer *info);

/**
  * \brief Set audio volume specific to a stream.
  *
  * \param[in] stream_handle - Valid stream handle obtained
  *       from qal_stream_open
  * \param[out] position - Mmap buffer read/write position returned.
  *
  * \return 0 on success, error code otherwise
  */
int32_t qal_stream_get_mmap_position(qal_stream_handle_t *stream_handle,
                              struct qal_mmap_position *position);

extern void  __gcov_flush();

/**
 *  Enable Gcov for QAL.
 */
//void enable_gcov();

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /*QAL_API_H*/
