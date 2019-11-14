ifeq ($(call is-board-platform-in-list, sdm845 msmnile),true)

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

LOCAL_C_INCLUDES := $(LOCAL_PATH)/stream/inc\
    $(LOCAL_PATH)/device/inc\
    $(LOCAL_PATH)/session/inc\
    $(LOCAL_PATH)/resource_manager/inc\
    $(LOCAL_PATH)/utils/inc

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/casa/casa_osal
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/casa/gsl
LOCAL_C_INCLUDES += $(TOP)/system/media/audio_route/include
LOCAL_C_INCLUDES += $(TOP)/external/tinyalsa/include
LOCAL_C_INCLUDES += $(TOP)/external/tinycompress/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/agm

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/techpack/audio/include
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS     := $(qal-def)
LOCAL_CFLAGS += -Wno-macro-redefined
#LOCAL_CFLAGS += -Werror
LOCAL_CFLAGS += -DCONFIG_GSL
LOCAL_CFLAGS += -D_GNU_SOURCE
LOCAL_CPPFLAGS += -fexceptions -frtti


LOCAL_SRC_FILES        := Qal.cpp\
    stream/src/Stream.cpp\
    stream/src/StreamCompress.cpp\
    stream/src/StreamPCM.cpp\
    stream/src/StreamSoundTrigger.cpp\
    device/src/Headphone.cpp \
    device/src/Device.cpp \
    device/src/Speaker.cpp \
    device/src/DeviceAlsa.cpp \
    device/src/SpeakerMic.cpp \
    device/src/DeviceGsl.cpp \
    session/src/Session.cpp \
    session/src/PayloadBuilder.cpp \
    session/src/SessionGsl.cpp \
    session/src/SessionAlsaPcm.cpp \
    session/src/SessionAlsaUtils.cpp \
    session/src/SessionAlsaCompress.cpp \
    session/src/SoundTriggerEngine.cpp \
    session/src/SoundTriggerEngineCapiCnn.cpp \
        session/src/SoundTriggerEngineCapiVop.cpp \
    session/src/SoundTriggerEngineGsl.cpp \
    resource_manager/src/ResourceManager.cpp \
    utils/src/QalRingBuffer.cpp \
session/src/SessionQts.cpp

LOCAL_MODULE               := libqal
LOCAL_MODULE_OWNER         := qti
LOCAL_MODULE_TAGS          := optional

LOCAL_SHARED_LIBRARIES := \
    libcasa-gsl\
    liblog\
    libexpat\
    liblx-osal\
    libaudioroute\
    libtinyalsa \
    libtinycompress\
    libagm

LOCAL_HEADER_LIBRARIES := \
libcasa-acdbdata \
libgecko-headers \
capiv2-headers

LOCAL_COPY_HEADERS_TO   := mm-audio/qal
LOCAL_COPY_HEADERS      := QalApi.h \
               QalDefs.h

LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_TINY_ANDROID
endif # is-board-platform-in-list
