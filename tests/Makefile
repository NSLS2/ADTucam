TOP = ..
include $(TOP)/configure/CONFIG

# Only build tests when UNIT_TESTING=YES is passed
ifeq ($(UNIT_TESTING),YES)

# Test executable
TESTPROD_HOST = test_adtucam

# Tell build system to look for sources in the driver directory
SRC_DIRS += $(TOP)/tucamApp/tucamSrc

# Source files - compile ADTucam.cpp directly instead of linking the library
# This allows us to avoid the SDK .so dependency
test_adtucam_SRCS += test_main.cpp
test_adtucam_SRCS += MockCameraSDK.cpp
test_adtucam_SRCS += test_connection.cpp
test_adtucam_SRCS += test_parameters.cpp
test_adtucam_SRCS += test_acquisition.cpp
test_adtucam_SRCS += ADTucam.cpp

# Include paths
USR_INCLUDES += -I$(TOP)/tucamApp/tucamSrc
USR_INCLUDES += -I$(TOP)/include

# Libraries - use ADBase/asyn but not ADTucam (we compile it directly)
test_adtucam_LIBS += ADBase asyn
test_adtucam_LIBS += $(EPICS_BASE_IOC_LIBS)
test_adtucam_SYS_LIBS += gtest gmock pthread

# Library paths - add /usr/local/lib64 for gtest/gmock
USR_LDFLAGS += -L/usr/local/lib64 -Wl,-rpath,/usr/local/lib64

# Compile flags - UNIT_TESTING excludes TUCAMSDKHandler and real SDK calls
USR_CPPFLAGS += -DUNIT_TESTING=1

endif

include $(ADCORE)/ADApp/commonLibraryMakefile
include $(TOP)/configure/RULES

# Test target
ifeq ($(UNIT_TESTING),YES)
test: test_adtucam
	./$(INSTALL_LOCATION_BIN)/$(T_A)/test_adtucam
else
test:
	@echo "Run with: make UNIT_TESTING=YES test"
endif

.PHONY: test
