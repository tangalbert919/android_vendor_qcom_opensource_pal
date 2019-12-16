LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    bt_base.c \
    bt_bundle.c

LOCAL_CFLAGS += -O2 -fvisibility=hidden

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libdl

LOCAL_C_INCLUDES += $(TOP)/system/media/audio/include
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/casa/casa_osal

LOCAL_HEADER_LIBRARIES := \
    libgecko-headers

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := lib_bt_bundle
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
