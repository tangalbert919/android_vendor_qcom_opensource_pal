LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/include/mm-audio/pal
LOCAL_MODULE := vendor.qti.hardware.pal@1.0-impl
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true
LOCAL_CFLAGS += -v
LOCAL_SRC_FILES := \
    src/pal_server_wrapper.cpp

LOCAL_COPY_HEADERS_TO := mm-audio/pal
LOCAL_COPY_HEADERS    := inc/pal_server_wrapper.h

LOCAL_SHARED_LIBRARIES := \
    libhidlbase \
    libhidltransport \
    libutils \
    liblog \
    libcutils \
    libhardware \
    libbase \
    vendor.qti.hardware.pal@1.0 \
    libar-pal

include $(BUILD_SHARED_LIBRARY)

