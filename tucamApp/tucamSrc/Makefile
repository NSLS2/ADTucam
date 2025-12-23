TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += ADTucam

LIB_SRCS += ADTucam.cpp
# Only include real SDK handler and hardware libraries when not building tests
ifndef UNIT_TESTING
LIB_SRCS += TUCAMSDKHandler.cpp
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_LIBS += TUCam
endif
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_SYS_LIBS += TUCam phxapi-x86_64
endif
else
# Define preprocessor flag for test builds
USR_CPPFLAGS += -DUNIT_TESTING
# Add include path for SDK headers (types only, no .so needed)
USR_INCLUDES += -I$(TOP)/include
endif

DBD += tucamSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
