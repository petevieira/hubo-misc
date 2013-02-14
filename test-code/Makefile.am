ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib
fastrak_balance_CPPFLAGS = -I./include -g -I/usr/include/eigen3 -I/usr/local/include/eigen3

techlibs = -lm -lach -lc -lhubotech -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = fastrak-balance fastrak-dremelIK fastrak-twoArm

include_HEADERS = include/Fastrak.h

fastrak_balance_SOURCES = src/fastrak-balance.cpp
fastrak_balance_LDADD = $(techlibs) libfastrak.la

fastrak_dremelIK_SOURCES = src/fastrak-dremelIK.cpp
fastrak_dremelIK_LDADD = $(techlibs) libfastrak.la

fastrak_twoArm_SOURCES = src/fastrak-twoArm.cpp
fastrak_twoArm_LDADD = $(techlibs) libfastrak.la

fastrakdir = /usr/lib
fastrak_LTLIBRARIES = libfastrak.la
libfastrak_la_SOURCES = src/Fastrak.cpp
libfastrak_la_LDFLAGS = -module -avoid-version -shared


