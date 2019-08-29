pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = mcs.pc
EXTRA_DIST = $(pkgconfig_DATA)

h_sources = ${top_srcdir}/inc/mcs.h \
            ${top_srcdir}/inc/qts.h

AM_CFLAGS := -I ${top_srcdir}/inc
AM_CFLAGS += $(ACDBDATA_CFLAGS)

library_include_HEADERS = $(h_sources)
AM_CFLAGS +=  -I $(PKG_CONFIG_SYSROOT_DIR)/usr/include/qal/
library_includedir = $(includedir)/mcs

mcs_sources = ${top_srcdir}/src/mcs.c

lib_LTLIBRARIES  = libmcs.la
libmcs_la_SOURCES = $(mcs_sources)
libmcs_la_LIBADD = -lqal -lcasa_osal $(GLIB_LIBS)
libmcs_la_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
libmcs_la_LDFLAGS = -shared -avoid-version

test_sources = ${top_srcdir}/test/src/mcs-test.c
bin_PROGRAMS = mcs_test
mcs_test_CC = @CC@
mcs_test_SOURCES = $(test_sources)
mcs_test_LDADD = libmcs.la
mcs_test_CFLAGS = $(AM_CFLAGS) $(GLIB_CFLAGS) -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat -include glib.h
