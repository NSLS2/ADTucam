TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += ADTucam

LIB_SRCS += ADTucam.cpp
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_LIBS += TUCam
endif
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_SYS_LIBS += TUCam phxapi-x86_64
endif

DBD += tucamSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
