ifeq ($(call is-board-platform-in-list, sdm845),true)

ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

#----------------------------------------------------------------------------
#                 Common definitons
#----------------------------------------------------------------------------

qal-def += -D_ANDROID_

#----------------------------------------------------------------------------
#             Make the Shared library (libqal)
#----------------------------------------------------------------------------

#LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc

#LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/gsl

LOCAL_CFLAGS     := $(qal-def)

LOCAL_SRC_FILES        := Qal.c

LOCAL_MODULE               := libqal
LOCAL_MODULE_OWNER         := qti
LOCAL_MODULE_TAGS          := optional

LOCAL_COPY_HEADERS_TO   := mm-audio/qal
LOCAL_COPY_HEADERS      := QalApi.h \
			   QalDefs.h


include $(BUILD_SHARED_LIBRARY)

endif # BUILD_TINY_ANDROID
endif # is-board-platform-in-list
