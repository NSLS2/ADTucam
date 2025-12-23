TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================


ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
SRC_DIRS += ../os/windows-x64 
LIB_INSTALLS_WIN32    += ../os/windows-x64/TUCam.lib
endif

ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
SRC_DIRS += ../os/linux-x86_64
ifndef UNIT_TESTING
LIB_INSTALLS_Linux += ../os/linux-x86_64/libphxapi-x86_64.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0.0
endif
endif

INC += TUCamApi.h TUDefine.h

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

