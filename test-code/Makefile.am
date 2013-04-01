ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib -I/usr/include/eigen3
#fastrak_balance_CPPFLAGS = -I./include -g -I/usr/include/eigen3 -I/usr/local/include/eigen3

ctrllibs = -lm -lach -lc -lhuboctrl -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = impCtrl-arms impCtrl-legs #fastrak-arms fastrak-legs#legIKTest fastrak-balance fastrak-dremelIK legTraj

include_HEADERS = include/Fastrak.h

fastrak_balance_SOURCES = src/fastrak-balance.cpp
fastrak_balance_LDADD = $(ctrllibs) libfastrak.la

fastrak_dremelIK_SOURCES = src/fastrak-dremelIK.cpp
fastrak_dremelIK_LDADD = $(ctrllibs) libfastrak.la

fastrak_arms_SOURCES = src/fastrak-arms.cpp
fastrak_arms_LDADD = $(ctrllibs) libfastrak.la

fastrak_legs_SOURCES = src/fastrak-legs.cpp
fastrak_legs_LDADD = $(ctrllibs) libfastrak.la

legIKTest_SOURCES = src/legIKTest.cpp
legIKTest_LDADD = $(ctrllibs)

oneLeg_SOURCES = src/oneLeg.cpp
oneLeg_LDADD = $(ctrllibs)

sensors_SOURCES = src/sensors.cpp
sensors_LDADD = $(ctrllibs)

dartTest_SOURCES = src/dartTest.cpp
dartTest_LDADD = $(ctrllibs)

impCtrl_arms_SOURCES = src/impCtrl-arms.cpp
impCtrl_arms_LDADD = $(ctrllibs)

impCtrl_legs_SOURCES = src/impCtrl-legs.cpp
impCtrl_legs_LDADD = $(ctrllibs)

fastrakdir = /usr/lib
fastrak_LTLIBRARIES = libfastrak.la
libfastrak_la_SOURCES = src/Fastrak.cpp
libfastrak_la_LDFLAGS = -module -avoid-version -shared
