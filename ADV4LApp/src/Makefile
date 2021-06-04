TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

INC += ADV4L.h

LIBRARY_IOC = ADV4L
LIB_SRCS += ADV4L.cpp

DBD += ADV4LSupport.dbd

#USR_LIBS += v4l2
# Probably an ugly hack, there must be a better way to do this
# in this maze of make-macros masquerading as a build system...
CODE_LDFLAGS += -lv4l2

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

