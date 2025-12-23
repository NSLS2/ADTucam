TOP = ..
include $(TOP)/configure/CONFIG

# Only build tests when UNIT_TESTING=YES is passed
ifeq ($(UNIT_TESTING),YES)

# Test executable
TESTPROD_HOST = test_adtucam

# Source files
test_adtucam_SRCS += test_main.cpp
test_adtucam_SRCS += MockCameraSDK.cpp
test_adtucam_SRCS += test_connection.cpp
test_adtucam_SRCS += test_parameters.cpp
test_adtucam_SRCS += test_acquisition.cpp

# Include paths
USR_INCLUDES += -I$(TOP)/tucamApp/tucamSrc
USR_INCLUDES += -I$(TOP)/include

# Libraries - link against ADTucam library (built with UNIT_TESTING)
test_adtucam_LIBS += ADTucam
test_adtucam_LIBS += ADBase asyn
test_adtucam_LIBS += $(EPICS_BASE_IOC_LIBS)
test_adtucam_SYS_LIBS += gtest gmock pthread

# Library paths - add /usr/local/lib64 for gtest/gmock
USR_LDFLAGS += -L/usr/local/lib64 -Wl,-rpath,/usr/local/lib64

# Compile flags - Updated to C++17
test_adtucam_CPPFLAGS += -DUNIT_TESTING=1 -std=c++17

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
