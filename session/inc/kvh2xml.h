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

typedef enum {
	PLATFORM_LA = 1, 	/**< @h2xmle_name {LA} */
}platforms;
/**
  	@h2xml_defType{Key}
  	@h2xml_platforms{PLATFORM_LA}
*/

enum AllKeyIds{
	STREAM_TYPE = 0xA1000000,    /**< @h2xmle_name{Stream} */
	DEVICERX = 0xA2000000,       /**< @h2xmle_name{DeviceRX} */
	DEVICETX = 0xA3000000,       /**< @h2xmle_name{DeviceTX} */
	VOLUME = 0xA4000000,         /**< @h2xmle_name{Volume} */
	SAMPLINGRATE = 0xA5000000,   /**< @h2xmle_name{SamplingRate} */
	BITWIDTH = 0xA6000000,       /**< @h2xmle_name{BitWidth} */
	PAUSE = 0xA7000000,          /**< @h2xmle_name{Pause} */
	MUTE = 0xA8000000,           /**< @h2xmle_name{Mute} */
	CHANNELS = 0xA9000000,       /**< @h2xmle_name{Channels} */
};
/**
	@h2xmlk_key {STREAM_TYPE}
	@h2xmlk_description {Type of Stream}
*/
enum Key_Stream {
	PCM_LL_PLAYBACK = 0xA1000001, /**< @h2xmle_name {PCM_Playback}*/
	PCM_RECORD = 0xA1000002,      /**< @h2xmle_name {PCM_Record}*/
	PCM_LOOPBACK = 0xA1000003,    /**< @h2xmle_name {PCM_Loopback}*/
	VOICE_UI = 0xA1000004,        /**< @h2xmle_name {Voice_UI}*/
	VOIP_RX_PLAYBACK = 0xA1000005,/**< @h2xmle_name {Voip_Rx}*/
	VOIP_TX_RECORD = 0xA1000006,   /**< @h2xmle_name {Voip_Tx}*/
};
/**
	@h2xmlk_key {DEVICERX}
	@h2xmlk_description {Rx Device}
*/
enum Key_DeviceRX {
	SPEAKER = 0xA2000001, /**< @h2xmle_name {Speaker}*/
};
/**
	@h2xmlk_key {DEVICETX}
	@h2xmlk_description {Tx Device}
*/
enum Key_DeviceTX {
	HANDSETMIC = 0xA3000001, /**< @h2xmle_name {HandsetMic}*/
};
/**
	@h2xmlk_key {VOLUME}
	@h2xmlk_description {Volume}
*/
enum Key_Volume {
	LEVEL_0 = 0, /**< @h2xmle_name {Level_0}*/
	LEVEL_1 = 1, /**< @h2xmle_name {Level_1}*/
	LEVEL_2 = 2, /**< @h2xmle_name {Level_2}*/
	LEVEL_3 = 3, /**< @h2xmle_name {Level_3}*/
	LEVEL_4 = 4, /**< @h2xmle_name {Level_4}*/
	LEVEL_5 = 5, /**< @h2xmle_name {Level_5}*/
	LEVEL_6 = 6, /**< @h2xmle_name {Level_6}*/
	LEVEL_7 = 7, /**< @h2xmle_name {Level_7}*/
	LEVEL_8 = 8, /**< @h2xmle_name {Level_8}*/
	LEVEL_9 = 9, /**< @h2xmle_name {Level_9}*/
	LEVEL_10 = 10, /**< @h2xmle_name {Level_10}*/
	LEVEL_11 = 11, /**< @h2xmle_name {Level_11}*/
	LEVEL_12 = 12, /**< @h2xmle_name {Level_12}*/
	LEVEL_13 = 13, /**< @h2xmle_name {Level_13}*/
	LEVEL_14 = 14, /**< @h2xmle_name {Level_14}*/
	LEVEL_15 = 15, /**< @h2xmle_name {Level_15}*/
};
/**
	@h2xmlk_key {SAMPLINGRATE}
	@h2xmlk_description {Sampling Rate}
*/
enum Key_SamplingRate {
	SAMPLINGRATE_8K = 8000, /**< @h2xmle_description {8k}*/
	SAMPLINGRATE_16K = 16000, /**< @h2xmle_description {16k}*/
	SAMPLINGRATE_32K = 32000, /**< @h2xmle_description {32k}*/
	SAMPLINGRATE_44K = 44100, /**< @h2xmle_description {44.1k}*/
	SAMPLINGRATE_48K = 48000, /**< @h2xmle_description {48k}*/
	SAMPLINGRATE_96K = 96000, /**< @h2xmle_description {96k}*/
	SAMPLINGRATE_192K = 192000, /**< @h2xmle_description {192k}*/
	SAMPLINGRATE_384K = 384000, /**< @h2xmle_description {384k}*/
};
/**
	@h2xmlk_key {BITWIDTH}
	@h2xmlk_description {Bit Width}
*/
enum Key_BitWidth {
	BITWIDTH_16 = 16, /**< @h2xmle_name {Bit16}*/
	BITWIDTH_24 = 24, /**< @h2xmle_name {Bit24}*/
	BITWIDTH_32 = 32, /**< @h2xmle_name {Bit32}*/
};
/**
	@h2xmlk_key {PAUSE}
	@h2xmlk_description {Pause}
*/
enum Key_Pause {
	OFF = 0, /**< @h2xmle_name {Off}*/
	ON = 1, /**< @h2xmle_name {On}*/
};
/**
	@h2xmlk_key {MUTE}
	@h2xmlk_description {Mute}
*/
enum Key_Mute {
	MUTE_OFF = 0, /**< @h2xmle_name {Off}*/
	MUTE_ON = 1, /**< @h2xmle_name {On}*/
};
/**
	@h2xmlk_key {CHANNELS}
	@h2xmlk_description {Channels}
*/
enum Key_Channels {
	CHANNEL1 = 1, /**< @h2xmle_name {CH1}*/
	CHANNEL2 = 2, /**< @h2xmle_name {CH2}*/
};
/**
	@h2xmlk_gkeys
	@h2xmlk_description {Graph kv}
*/
enum Graph_Keys {
	gk_Stream = STREAM_TYPE,
	gk_DeviceRX = DEVICERX,
	gk_DeviceTX = DEVICETX,
};
/**
	@h2xmlk_ckeys
	@h2xmlk_description {ckv}
*/
enum Cal_Keys {
	ck_volume = VOLUME,
};

#define DEVICE_HW_ENDPOINT_RX        0xC0000006
/**
        @h2xmlk_modTag {"device_hw_ep_rx",DEVICE_HW_ENDPOINT_RX}
	@h2xmlk_description {Hw EP Rx}
*/
enum HW_ENDPOINT_RX_Keys {
	tk1_hweprx = DEVICERX,
	tk2_hweprx = SAMPLINGRATE,
	tk3_hweprx = BITWIDTH,
	tk4_hweprx = CHANNELS,
};
#define DEVICE_HW_ENDPOINT_TX        0xC0000007
/**
        @h2xmlk_modTag {"device_hw_ep_tx",DEVICE_HW_ENDPOINT_TX}
	@h2xmlk_description {Hw EP Tx}
*/
enum HW_ENDPOINT_TX_Keys {
	tk1_hweptx = DEVICETX,
	tk2_hweptx = SAMPLINGRATE,
	tk3_hweptx = BITWIDTH,
	tk4_hweptx = CHANNELS,
};
#define TAG_PAUSE       0xC0000008
/**
	@h2xmlk_modTag {"pause", TAG_PAUSE}
	@h2xmlk_description {Pause}
*/
enum TAG_PAUSE_Keys {
	tk1_Pause = PAUSE,
};
#define TAG_MUTE        0xC0000009
/**
	@h2xmlk_modTag {"mute", TAG_MUTE}
	@h2xmlk_description {Mute}
*/
enum TAG_MUTE_Keys {
	tk1_Mute = MUTE,
};
#define TAG_STREAM_VOLUME  0xC000000A
/**
	@h2xmlk_modTag {"svolume", TAG_STREAM_VOLUME}
	@h2xmlk_description {StreamVolume}
*/
enum TAG_STREAM_VOLUME_Keys {
	tk1_Volume = VOLUME,
};
#define TAG_STREAM_MFC_SR  0xC000000B
/**
        @h2xmlk_modTag{"TAG_STREAM_MFC_SR",TAG_STREAM_MFC_SR}
	@h2xmlk_description {StreamMFCSamplingrate}
*/
enum TAG_STREAM_MFC_SR_Keys {
	tk1_MFC_SR = SAMPLINGRATE,
};
/**
  	@h2xml_defType{Key}
	@h2xmlk_modTagList
*/
typedef enum {
        /**@h2xmle_name {sh_ep} */                        SHMEM_ENDPOINT              = 0xC0000001,  
	/**@h2xmle_name {stream_input_media_format} */    STREAM_INPUT_MEDIA_FORMAT   = 0xC0000002,  
	/**@h2xmle_name {stream_pcm_decoder} */           STREAM_PCM_DECODER          = 0xC0000003,  
	/**@h2xmle_name {stream_pcm_encoder} */           STREAM_PCM_ENCODER          = 0xC0000004,  
	/**@h2xmle_name {stream_pcm_converter} */         STREAM_PCM_CONVERTER        = 0xC0000005,  
	/**@h2xmle_name {device_sva} */                   DEVICE_SVA                  = 0xC0000010,  
	/**@h2xmle_name {device_adam} */                  DEVICE_ADAM                 = 0xC0000011,  
}TAGS_DEFINITIONS;


