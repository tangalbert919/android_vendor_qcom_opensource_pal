pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = mcs.pc
EXTRA_DIST = $(pkgconfig_DATA)

if BUILDSYSTEM_OPENWRT
h_sources = ./inc/mcs.h \
            ./inc/qts.h

AM_CFLAGS := -I ./inc
else
h_sources = ${top_srcdir}/inc/mcs.h \
            ${top_srcdir}/inc/qts.h

AM_CFLAGS := -I ${top_srcdir}/inc
AM_CFLAGS +=  -I $(PKG_CONFIG_SYSROOT_DIR)/usr/include/qal/
endif

AM_CFLAGS += $(ACDBDATA_CFLAGS)

library_include_HEADERS = $(h_sources)
library_includedir = $(includedir)/mcs

if BUILDSYSTEM_OPENWRT
mcs_sources = ./src/mcs.c
lib_LTLIBRARIES = libaudiomcs.la
libaudiomcs_la_SOURCES = $(mcs_sources)

libaudiomcs_la_LIBADD = -lqal -lcasa_osal $(GLIB_LIBS)

libaudiomcs_la_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libaudiomcs_la_LDFLAGS = -shared -avoid-version
else
lib_LTLIBRARIES = libmcs.la
mcs_sources = ${top_srcdir}/src/mcs.c
libmcs_la_SOURCES = $(mcs_sources)

libmcs_la_LIBADD = -lqal -lcasa_osal $(GLIB_LIBS)

libmcs_la_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libmcs_la_LDFLAGS = -shared -avoid-version
endif

if BUILDSYSTEM_OPENWRT
test_sources = ./test/src/mcs-test.c
else
test_sources = ${top_srcdir}/test/src/mcs-test.c
endif

bin_PROGRAMS = mcs_test
mcs_test_CC = @CC@
mcs_test_SOURCES = $(test_sources)
if BUILDSYSTEM_OPENWRT
mcs_test_LDADD = libaudiomcs.la
else
mcs_test_LDADD = libmcs.la
endif
mcs_test_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
