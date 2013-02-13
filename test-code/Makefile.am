ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib
fastrak_balance_CPPFLAGS = -I./include -g -I/usr/include/eigen3 -I/usr/local/include/eigen3

techlibs = -lm -lach -lc -lhubotech -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = fastrak-balance legIKTest #oneLeg


# For each one, list the .cpp sources:
#oneLeg_SOURCES = src/oneLeg.cpp
# And then the libraries which they should link to:
#oneLeg_LDADD = $(techlibs)

legIKTest_SOURCES = src/legIKTest.cpp
legIKTest_LDADD = $(techlibs)

fastrak_balance_SOURCES = src/fastrak-balance.cpp
fastrak_balance_LDADD = $(techlibs) libfastrak.la

fastrakdir = /usr/lib
fastrak_LTLIBRARIES = libfastrak.la
libfastrak_la_SOURCES = src/Fastrak.cpp
libfastrak_la_LDFLAGS = -module -avoid-version -shared


