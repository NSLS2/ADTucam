#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure

# Unit testing mode: skip SDK-dependent directories, only build tests
# Usage: make UNIT_TESTING=YES
ifeq ($(UNIT_TESTING),YES)
DIRS := $(DIRS) tests
else
# Normal build: requires SDK .so files
DIRS := $(DIRS) tucamSupport
DIRS := $(DIRS) tucamApp
tucamApp_DEPEND_DIRS += tucamSupport
ifeq ($(BUILD_IOCS), YES)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
iocs_DEPEND_DIRS += tucamApp
endif
endif
include $(TOP)/configure/RULES_TOP

uninstall: uninstall_iocs
uninstall_iocs:
	$(MAKE) -C iocs uninstall
.PHONY: uninstall uninstall_iocs

realuninstall: realuninstall_iocs
realuninstall_iocs:
	$(MAKE) -C iocs realuninstall
.PHONY: realuninstall realuninstall_iocs
