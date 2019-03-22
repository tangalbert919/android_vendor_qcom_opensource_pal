pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = qal.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
h_sources = ./stream/inc/Stream.h \
            ./stream/inc/StreamCompress.h \
            ./stream/inc/StreamPCM.h \
            ./device/inc/Device.h \
            ./device/inc/Speaker.h \
            ./device/inc/CodecDevice.h \
            ./device/inc/CodecDeviceAlsa.h \
            ./device/inc/CodecDeviceGsl.h \
            ./device/inc/SpeakerMic.h \
            ./session/inc/Session.h \
            ./session/inc/PayloadBuilder.h \
            ./session/inc/SessionGsl.h \
            ./session/inc/SessionAlsaPcm.h \
            ./resource_manager/inc/ResourceManager.h \
            ./QalDefs.h \
            ./QalApi.h \
            ./QalAudioRoute.h \
            ./QalCommon.h

AM_CPPFLAGS := -I ./stream/inc
AM_CPPFLAGS += -I ./device/inc
AM_CPPFLAGS += -I ./session/inc
AM_CPPFLAGS += -I ./resource_manager/inc
AM_CPPFLAGS += -I ./

qal_sources = ./stream/src/Stream.cpp \
              ./stream/src/StreamCompress.cpp \
              ./stream/src/StreamPCM.cpp \
              ./device/src/Device.cpp \
              ./device/src/Speaker.cpp \
              ./device/src/CodecDeviceAlsa.cpp \
              ./device/src/CodecDevice.cpp \
              ./device/src/SpeakerMic.cpp \
              ./device/src/CodecDeviceGsl.cpp \
              ./session/src/Session.cpp \
              ./session/src/PayloadBuilder.cpp \
              ./session/src/SessionGsl.cpp \
              ./session/src/SessionAlsaPcm.cpp \
              ./resource_manager/src/ResourceManager.cpp \
              ./Qal.cpp
else
h_sources = ${top_srcdir}/stream/inc/Stream.h \
            ${top_srcdir}/stream/inc/StreamCompress.h \
            ${top_srcdir}/stream/inc/StreamPCM.h \
            ${top_srcdir}/device/inc/Device.h \
            ${top_srcdir}/device/inc/Speaker.h \
            ${top_srcdir}/device/inc/CodecDevice.h \
            ${top_srcdir}/device/inc/CodecDeviceAlsa.h \
            ${top_srcdir}/device/inc/CodecDeviceGsl.h \
            ${top_srcdir}/device/inc/SpeakerMic.h \
            ${top_srcdir}/session/inc/Session.h \
            ${top_srcdir}/session/inc/PayloadBuilder.h \
            ${top_srcdir}/session/inc/SessionGsl.h \
            ${top_srcdir}/session/inc/SessionAlsaPcm.h \
            ${top_srcdir}/resource_manager/inc/ResourceManager.h \
            ${top_srcdir}/QalDefs.h \
            ${top_srcdir}/QalApi.h \
            ${top_srcdir}/QalAudioRoute.h \
            ${top_srcdir}/QalCommon.h

AM_CPPFLAGS := -I $(top_srcdir)/stream/inc
AM_CPPFLAGS += -I $(top_srcdir)/device/inc
AM_CPPFLAGS += -I $(top_srcdir)/session/inc
AM_CPPFLAGS += -I $(top_srcdir)/resource_manager/inc
AM_CPPFLAGS += -I $(top_srcdir)/

qal_sources = ${top_srcdir}/stream/src/Stream.cpp \
              ${top_srcdir}/stream/src/StreamCompress.cpp \
              ${top_srcdir}/stream/src/StreamPCM.cpp \
              ${top_srcdir}/device/src/Device.cpp \
              ${top_srcdir}/device/src/Speaker.cpp \
              ${top_srcdir}/device/src/CodecDeviceAlsa.cpp \
              ${top_srcdir}/device/src/CodecDevice.cpp \
              ${top_srcdir}/device/src/SpeakerMic.cpp \
              ${top_srcdir}/device/src/CodecDeviceGsl.cpp \
              ${top_srcdir}/session/src/Session.cpp \
              ${top_srcdir}/session/src/PayloadBuilder.cpp \
              ${top_srcdir}/session/src/SessionGsl.cpp \
              ${top_srcdir}/session/src/SessionAlsaPcm.cpp \
              ${top_srcdir}/resource_manager/src/ResourceManager.cpp \
              ${top_srcdir}/Qal.cpp

endif

AM_CPPFLAGS += -D__unused=__attribute__\(\(__unused__\)\)
AM_CPPFLAGS += -DCONFIG_GSL

library_include_HEADERS = $(h_sources)
library_includedir = $(includedir)/qal

lib_LTLIBRARIES     = libqal.la
libqal_la_SOURCES   = $(qal_sources)
libqal_la_LIBADD    = -ltinyalsa -laudioroute -lcasa_osal
libqal_la_CPPFLAGS := $(AM_CPPFLAGS)
libqal_la_CPPFLAGS += -std=c++14 -DHAVE_PTHREADS -DHAVE_ANDROID_OS
libqal_la_CPPFLAGS += -DDEBUG_REFS_CALLSTACK_ENABLED=0
libqal_la_CPPFLAGS += -DNDEBUG
libqal_la_LDFLAGS   = -shared -avoid-version
