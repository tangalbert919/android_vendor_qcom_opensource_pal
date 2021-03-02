ifneq ($(AUDIO_USE_STUB_HAL), true)
ifeq ($(call is-board-platform-in-list, sdm845 msmnile kona lahaina taro bengal),true)

ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)
PAL_BASE_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_USE_VNDK := true

#----------------------------------------------------------------------------
#                 Common definitons
#----------------------------------------------------------------------------

pal-def += -D_ANDROID_

#----------------------------------------------------------------------------
#             Make the Shared library (libar-pal)
#----------------------------------------------------------------------------

#LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc

#LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/gsl

LOCAL_C_INCLUDES := $(LOCAL_PATH)/stream/inc \
    $(LOCAL_PATH)/device/inc                 \
    $(LOCAL_PATH)/session/inc                \
    $(LOCAL_PATH)/resource_manager/inc       \
    $(LOCAL_PATH)/utils/inc                  \
    $(LOCAL_PATH)/plugins/codecs

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/ar_osal
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/gsl
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/spf/api/apm
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/spf/api/vcpm
LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include
LOCAL_C_INCLUDES += $(TOP)/system/media/audio/include
LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/opensource/tinyalsa/include
LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/opensource/tinycompress/include

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/techpack/audio/include
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS   := $(pal-def)
LOCAL_CFLAGS   += -Wno-macro-redefined
LOCAL_CFLAGS   += -Wall -Werror
LOCAL_CFLAGS   += -DCONFIG_GSL
LOCAL_CFLAGS   += -D_GNU_SOURCE
LOCAL_CFLAGS   += -DPAL_SP_TEMP_PATH=\"/data/vendor/audio/audio.cal\"
LOCAL_CPPFLAGS += -fexceptions -frtti


LOCAL_SRC_FILES        := Pal.cpp\
    stream/src/Stream.cpp\
    stream/src/StreamCompress.cpp\
    stream/src/StreamPCM.cpp\
    stream/src/StreamInCall.cpp\
    stream/src/StreamNonTunnel.cpp\
    stream/src/StreamSoundTrigger.cpp\
    device/src/Headphone.cpp \
    device/src/USBAudio.cpp \
    device/src/Device.cpp \
    device/src/Speaker.cpp \
    device/src/Bluetooth.cpp \
    device/src/SpeakerMic.cpp \
    device/src/HeadsetMic.cpp \
    device/src/HandsetMic.cpp \
    device/src/Handset.cpp \
    device/src/HandsetVaMic.cpp \
    device/src/DisplayPort.cpp \
    device/src/HeadsetVaMic.cpp \
    device/src/RTProxy.cpp \
    device/src/SpeakerProtection.cpp \
    device/src/FMDevice.cpp\
    device/src/HapticsDev.cpp \
    session/src/Session.cpp \
    session/src/PayloadBuilder.cpp \
    session/src/SessionAlsaPcm.cpp \
    session/src/SessionAgm.cpp \
    session/src/SessionAlsaUtils.cpp \
    session/src/SessionAlsaCompress.cpp \
    session/src/SessionAlsaVoice.cpp \
    session/src/SoundTriggerEngine.cpp \
    session/src/SoundTriggerEngineCapi.cpp \
    session/src/SoundTriggerEngineGsl.cpp \
    resource_manager/src/ResourceManager.cpp \
    resource_manager/src/SndCardMonitor.cpp \
    utils/src/SoundTriggerPlatformInfo.cpp \
    utils/src/PalRingBuffer.cpp \
    utils/src/SoundTriggerUtils.cpp

LOCAL_MODULE       := libar-pal
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_TAGS  := optional

LOCAL_SHARED_LIBRARIES := \
    libar-gsl\
    liblog\
    libexpat\
    liblx-osal\
    libaudioroute\
    libcutils \
    libqti-tinyalsa \
    libqti-tinycompress\
    libagmclient

LOCAL_HEADER_LIBRARIES := \
    libspf-headers \
    capiv2-headers \
    libagm_headers \
    libacdb_headers

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)

LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
include $(PAL_BASE_PATH)/plugins/Android.mk
include $(PAL_BASE_PATH)/ipc/HwBinders/Android.mk

endif # BUILD_TINY_ANDROID
endif # is-board-platform-in-list
endif #AUDIO_USE_STUB_HAL
