ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib

techlibs = -lm -lach -lc -lhubotech -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = oneLeg


# For each one, list the .cpp sources:
oneLeg_SOURCES = src/oneLeg.cpp
# And then the libraries which they should link to:
oneLeg_LDADD = $(techlibs)
