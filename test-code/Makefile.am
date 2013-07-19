ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib -I/usr/include/eigen3
#fastrak_balance_CPPFLAGS = -I./include -g -I/usr/include/eigen3 -I/usr/local/include/eigen3

ctrllibs = -lm -lach -lc -lhuboctrl -ldaemonizer -lrt -lhuboparams

# List the binary files you want to generate:
bin_PROGRAMS = joint-manip-test tweak sensors impCtrl-arms impCtrl-legs teleop-arms teleop-legs go-home #keyboardTest

include_HEADERS = include/Teleop.h #include/KeyboardInput.h

#keyboardTest_SOURCES = src/keyboardTest.cpp
#keyboardTest_LDADD = $(ctrllibs) libkeyboardInput.la

joint_manip_test_SOURCES = src/joint-manip-test.cpp
joint_manip_test_LDADD = $(ctrllibs)

go_home_SOURCES = src/go-home.cpp
go_home_LDADD = $(ctrllibs)

fastrak_balance_SOURCES = src/fastrak-balance.cpp
fastrak_balance_LDADD = $(ctrllibs) libfastrak.la

fastrak_dremelIK_SOURCES = src/fastrak-dremelIK.cpp
fastrak_dremelIK_LDADD = $(ctrllibs) libfastrak.la

teleop_arms_SOURCES = src/teleop-arms.cpp src/Collision_Checker.cpp
teleop_arms_LDADD = $(ctrllibs) libteleop.la

teleop_legs_SOURCES = src/teleop-legs.cpp src/Collision_Checker.cpp
teleop_legs_LDADD = $(ctrllibs) libteleop.la

legIKTest_SOURCES = src/legIKTest.cpp
legIKTest_LDADD = $(ctrllibs)

sensors_SOURCES = src/sensors.cpp
sensors_LDADD = $(ctrllibs)

dartTest_SOURCES = src/dartTest.cpp
dartTest_LDADD = $(ctrllibs)

impCtrl_arms_SOURCES = src/impCtrl-arms.cpp
impCtrl_arms_LDADD = $(ctrllibs)

impCtrl_legs_SOURCES = src/impCtrl-legs.cpp
impCtrl_legs_LDADD = $(ctrllibs)

tweak_SOURCES = src/tweak.cpp
tweak_LDADD = $(ctrllibs)

teleopdir = /usr/lib
teleop_LTLIBRARIES = libteleop.la
libteleop_la_SOURCES = src/Teleop.cpp
libteleop_la_LDFLAGS = -module -avoid-version -shared

#keyboardInputdir = /usr/lib
#keyboardInput_LTLIBRARIES = libkeyboardInput.la
#libkeyboardInput_la_SOURCES = src/KeyboardInput.cpp
#libkeyboardInput_la_LDFLAGS = -module -avoid-version -shared
