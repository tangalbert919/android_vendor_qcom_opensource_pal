ifeq ($(call is-board-platform-in-list, msmnile kona),true)

ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
adie_rtc-def += -D_ANDROID_
#LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_CFLAGS += -Wall -Werror

LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/ar_osal
LOCAL_SRC_FILES := \
    src/adie_rtc.c

LOCAL_MODULE := libadie_rtc
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := \
    liblog\
    libexpat\
    liblx-osal

LOCAL_EXPORT_C_INCLUDE_DIRS := inc
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/ar/ar_osal
LOCAL_MODULE := adie_rtc_test
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true
LOCAL_SRC_FILES := \
     test/src/adie_rtc_test.c

LOCAL_SHARED_LIBRARIES := \
    liblog\
    libexpat\
    libadie_rtc\
    liblx-osal

include $(BUILD_EXECUTABLE)

endif # BUILD_TINY_ANDROID
endif # is-board-platform-in-list
