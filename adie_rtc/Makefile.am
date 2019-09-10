pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = adie_rtc.pc
EXTRA_DIST = $(pkgconfig_DATA)

h_sources = ${top_srcdir}/inc/adie_rtc.h

AM_CFLAGS := -I ${top_srcdir}/inc

lib_include_HEADERS = $(h_sources)
AM_CFLAGS +=  -I $(PKG_CONFIG_SYSROOT_DIR)/usr/include/qal/
lib_includedir = $(includedir)/adie_rtc

rtc_c_sources = ${top_srcdir}/src/adie_rtc.c

lib_LTLIBRARIES  = libadie_rtc.la
libadie_rtc_la_SOURCES = $(rtc_c_sources)
libadie_rtc_la_LIBADD = -lcasa_osal $(GLIB_LIBS)
libadie_rtc_la_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libadie_rtc_la_LDFLAGS = -shared -avoid-version

test_sources = ${top_srcdir}/test/src/adie_rtc_test.c
bin_PROGRAMS = adie_rtc_test
adie_rtc_test_CC = @CC@
adie_rtc_test_SOURCES = $(test_sources)
adie_rtc_test_LDADD = libadie_rtc.la
adie_rtc_test_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h

