#Set this to @ to keep the makefile quiet
SILENCE = @

#---- Outputs ----#
COMPONENT_NAME = your

#--- Inputs ----#
PROJECT_HOME_DIR = .
CPPUTEST_HOME = ./../../../../cpputest
ifeq "$(CPPUTEST_HOME)" ""
$(error The environment variable CPPUTEST_HOME is not set. \
Set it to where cpputest is installed)
endif

# ---------------------- SRC_FILES and SRC_DIRS ----------------------
# Production code files are compiled and put into
# a library to link with the test runner.
#
# Test code of the same name overrides
# production code at link time.
#
# SRC_FILES specifies individual production
# code files.
#
# SRC_DIRS specifies directories containing
# production code C and CPP files.
#

SRC_FILES += 
SRC_DIRS += 

# System code 
SRC_FILES += ./../../autopilot/system/rtos.cpp

# Vehicle code
SRC_FILES += ./../../autopilot/vehicle/control.cpp
SRC_FILES += ./../../autopilot/vehicle/memory.cpp
SRC_FILES += ./../../autopilot/vehicle/navigation.cpp
SRC_FILES += ./../../autopilot/vehicle/telemetry.cpp
SRC_FILES += ./../../autopilot/vehicle/vehicle.cpp

# --------------------------------------------------------------------


# ----------------- TEST_SRC_FILES and TEST_SRC_DIRS -----------------
# Test files are always included in the build.
# Production code is pulled into the build unless
# it is overriden by code of the same name in the
# test code.
#
# TEST_SRC_FILES specifies individual test files to build.
# TEST_SRC_DIRS, builds everything in the directory

# All tests 
TEST_SRC_DIRS += tests
TEST_SRC_FILES += 

# --------------------------------------------------------------------


# -------------------------- MOCKS_SRC_DIRS --------------------------
# MOCKS_SRC_DIRS specifies a directories where you can put your
# mocks, stubs and fakes.  You can also just put them
# in TEST_SRC_DIRS

# ------------- BOAT ---------------

MOCKS_SRC_DIRS += mocks

# ----------------------------------

# Turn on CppUMock
CPPUTEST_USE_EXTENSIONS = Y

# --------------------------------------------------------------------


# ----------------------------- INCLUDES -----------------------------

# INCLUDE_DIRS are searched in order after the included file's
# containing directory

# This includes all the headers in the subfolders of 'include' 
INCLUDE_DIRS += $(CPPUTEST_HOME)/include

# Production code 
INCLUDE_DIRS += ./../../autopilot/vehicle
INCLUDE_DIRS += ./../../autopilot/system

# # Library 
INCLUDE_DIRS += ./../../../../mavlink/c_library_v2
INCLUDE_DIRS += ./../../../../CMSIS_6/CMSIS/RTOS2/Include
INCLUDE_DIRS += ./../../../../CMSIS-FreeRTOS/CMSIS/RTOS2/FreeRTOS/Include
INCLUDE_DIRS += ./../../../../CMSIS-FreeRTOS/Source/include
INCLUDE_DIRS += ./../../../../CMSIS-FreeRTOS/Source/portable/GCC/ARM_CM4F

# Test files 
INCLUDE_DIRS += mocks 
INCLUDE_DIRS += tests 

# --------------------------------------------------------------------


# ------------------------ CPPUTEST_OBJS_DIR -------------------------
# CPPUTEST_OBJS_DIR lets you control where the
# build artifact (.o and .d) files are stored.
#
# If you have to use "../" to get to your source path
# the makefile will put the .o and .d files in surprising
# places.
#
# To make up for each level of "../"in the source path,
# add place holder subdirectories to CPPUTEST_OBJS_DIR
# each.
# e.g. if you have "../../src", set to "test-objs/1/2"
#
# This is kind of a kludge, but it causes the
# .o and .d files to be put under objs.
CPPUTEST_OBJS_DIR = test-obj
CPPUTEST_OBJS_DIR = test-obj/1/2
CPPUTEST_OBJS_DIR = test-obj/1/2/3/4

CPPUTEST_LIB_DIR = test-lib

# --------------------------------------------------------------------


# You may have to tweak these compiler flags
#    CPPUTEST_WARNINGFLAGS - apply to C and C++
#    CPPUTEST_CFLAGS - apply to C files only
#    CPPUTEST_CXXFLAGS - apply to C++ files only
#    CPPUTEST_CPPFLAGS - apply to C and C++ Pre-Processor
#
# If you get an error like this
#     TestPlugin.h:93:59: error: 'override' keyword is incompatible
#        with C++98 [-Werror,-Wc++98-compat] ...
# The compiler is basically telling you how to fix the
# build problem.  You would add this flag setting
#     CPPUTEST_CXXFLAGS += -Wno-c++14-compat



# Some flags to quiet clang
ifeq ($(shell $(CC) -v 2>&1 | grep -c "clang"), 1)
CPPUTEST_WARNINGFLAGS += -Wno-unknown-warning-option
CPPUTEST_WARNINGFLAGS += -Wno-covered-switch-default
CPPUTEST_WARNINGFLAGS += -Wno-reserved-id-macro
CPPUTEST_WARNINGFLAGS += -Wno-keyword-macro
CPPUTEST_WARNINGFLAGS += -Wno-documentation
CPPUTEST_WARNINGFLAGS += -Wno-missing-noreturn
endif

# CppUTest flags 
CPPUTEST_WARNINGFLAGS += -Wall
CPPUTEST_WARNINGFLAGS += -Werror
CPPUTEST_WARNINGFLAGS += -Wfatal-errors
CPPUTEST_WARNINGFLAGS += -Wswitch-default
CPPUTEST_WARNINGFLAGS += -Wno-format-nonliteral
CPPUTEST_WARNINGFLAGS += -Wno-sign-conversion
CPPUTEST_WARNINGFLAGS += -Wno-pedantic
CPPUTEST_WARNINGFLAGS += -Wno-shadow
CPPUTEST_WARNINGFLAGS += -Wno-missing-field-initializers
CPPUTEST_WARNINGFLAGS += -Wno-unused-parameter
CPPUTEST_WARNINGFLAGS += -Wno-switch-default
CPPUTEST_WARNINGFLAGS += -Wno-address-of-packed-member
CPPUTEST_CFLAGS += -pedantic
CPPUTEST_CFLAGS += -Wno-missing-prototypes
CPPUTEST_CFLAGS += -Wno-strict-prototypes
CPPUTEST_CXXFLAGS += -Wno-c++14-compat
CPPUTEST_CXXFLAGS += --std=c++11
CPPUTEST_CXXFLAGS += -Wno-c++98-compat-pedantic
CPPUTEST_CXXFLAGS += -Wno-c++98-compat

# Additional exceptions added by me 
CPPUTEST_WARNINGFLAGS += -Wno-int-to-pointer-cast

# Coloroze output
CPPUTEST_EXE_FLAGS += -c

# --- LD_LIBRARIES -- Additional needed libraries can be added here.
# commented out example specifies math library
LD_LIBRARIES += -lm

# Look at $(CPPUTEST_HOME)/build/MakefileWorker.mk for more controls

include $(CPPUTEST_HOME)/build/MakefileWorker.mk
