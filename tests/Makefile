TOP = ..
include $(TOP)/configure/CONFIG

# Test executable
TESTPROD_HOST = test_adtucam

# Source files
test_adtucam_SRCS += test_main.cpp
test_adtucam_SRCS += MockCameraSDK.cpp
test_adtucam_SRCS += test_connection.cpp
test_adtucam_SRCS += test_parameters.cpp
test_adtucam_SRCS += test_acquisition.cpp

# Include paths
USR_INCLUDES += -I../../tucamApp/tucamSrc
USR_INCLUDES += -I../../tucamSupport/os/linux-x86_64

# Library paths
USR_LDFLAGS += -L/usr/local/lib64 -Wl,-rpath,/usr/local/lib64

# Libraries
test_adtucam_LIBS += ADTucam
test_adtucam_LIBS += ADBase asyn
test_adtucam_LIBS += $(EPICS_BASE_IOC_LIBS)
test_adtucam_SYS_LIBS += gtest gmock pthread

# Compile flags - Updated to C++17
test_adtucam_CPPFLAGS += -DUNIT_TESTING=1 -std=c++17

include $(ADCORE)/ADApp/commonLibraryMakefile
include $(TOP)/configure/RULES

# Test target
test: test_adtucam
	./$(INSTALL_LOCATION_BIN)/$(T_A)/test_adtucam

.PHONY: test
