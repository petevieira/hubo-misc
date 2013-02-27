ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib -I/usr/include/eigen3
#fastrak_balance_CPPFLAGS = -I./include -g -I/usr/include/eigen3 -I/usr/local/include/eigen3

techlibs = -lm -lach -lc -lhubotech -ldaemonizer -lrt -lhuboparams -ldartlite

# List the binary files you want to generate:
bin_PROGRAMS = oneLeg #legIKTest fastrak-oneLeg #fastrak-balance fastrak-dremelIK fastrak-twoArm fastrak-oneLeg legTraj sensors

include_HEADERS = include/Fastrak.h

fastrak_balance_SOURCES = src/fastrak-balance.cpp
fastrak_balance_LDADD = $(techlibs) libfastrak.la

fastrak_dremelIK_SOURCES = src/fastrak-dremelIK.cpp
fastrak_dremelIK_LDADD = $(techlibs) libfastrak.la

fastrak_twoArm_SOURCES = src/fastrak-twoArm.cpp
fastrak_twoArm_LDADD = $(techlibs) libfastrak.la

fastrak_oneLeg_SOURCES = src/fastrak-oneleg.cpp
fastrak_oneLeg_LDADD = $(techlibs) libfastrak.la

legIKTest_SOURCES = src/legIKTest.cpp
legIKTest_LDADD = $(techlibs)

oneLeg_SOURCES = src/oneLeg.cpp
oneLeg_LDADD = $(techlibs)

sensors_SOURCES = src/sensors.cpp
sensors_LDADD = $(techlibs)

dartTest_SOURCES = src/dartTest.cpp
dartTest_LDADD = $(techlibs)

fastrakdir = /usr/lib
fastrak_LTLIBRARIES = libfastrak.la
libfastrak_la_SOURCES = src/Fastrak.cpp
libfastrak_la_LDFLAGS = -module -avoid-version -shared
