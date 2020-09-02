ifeq ($(call is-board-platform-in-list, sdm845 msmnile kona lahaina),true)

ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_USE_VNDK := true

#----------------------------------------------------------------------------
#                 Common definitons
#----------------------------------------------------------------------------

qal-def += -D_ANDROID_

#----------------------------------------------------------------------------
#             Make the Shared library (libqal)
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
LOCAL_C_INCLUDES += $(TOP)/external/tinyalsa/include
LOCAL_C_INCLUDES += $(TOP)/external/tinycompress/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/agm

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/techpack/audio/include
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS   := $(qal-def)
LOCAL_CFLAGS   += -Wno-macro-redefined
LOCAL_CFLAGS   += -Wall -Werror
LOCAL_CFLAGS   += -DCONFIG_GSL
LOCAL_CFLAGS   += -D_GNU_SOURCE
LOCAL_CFLAGS   += -DQAL_SP_TEMP_PATH=\"/data/vendor/audio/audio.cal\"
LOCAL_CPPFLAGS += -fexceptions -frtti


LOCAL_SRC_FILES        := Qal.cpp\
    stream/src/Stream.cpp\
    stream/src/StreamCompress.cpp\
    stream/src/StreamPCM.cpp\
    stream/src/StreamInCall.cpp\
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
    session/src/Session.cpp \
    session/src/PayloadBuilder.cpp \
    session/src/SessionAlsaPcm.cpp \
    session/src/SessionAlsaUtils.cpp \
    session/src/SessionAlsaCompress.cpp \
    session/src/SessionAlsaVoice.cpp \
    session/src/SoundTriggerEngine.cpp \
    session/src/SoundTriggerEngineCapi.cpp \
    session/src/SoundTriggerEngineGsl.cpp \
    resource_manager/src/ResourceManager.cpp \
    resource_manager/src/SndCardMonitor.cpp \
    utils/src/SoundTriggerPlatformInfo.cpp \
    utils/src/QalRingBuffer.cpp

LOCAL_MODULE       := libqal
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_TAGS  := optional

LOCAL_SHARED_LIBRARIES := \
    libar-gsl\
    liblog\
    libexpat\
    liblx-osal\
    libaudioroute\
    libcutils \
    libtinyalsa \
    libtinycompress\
    libagmclient

LOCAL_HEADER_LIBRARIES := \
    libar-acdbdata \
    libspf-headers \
    capiv2-headers

LOCAL_COPY_HEADERS_TO := mm-audio/qal
LOCAL_COPY_HEADERS    := QalApi.h \
                         QalDefs.h

LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
include $(LOCAL_PATH)/plugins/Android.mk

endif # BUILD_TINY_ANDROID
endif # is-board-platform-in-list
