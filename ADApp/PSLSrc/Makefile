TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CFLAGS += 
#USR_CXXFLAGS +=

LIBRARY_IOC = PSL
PROD_IOC += PSLApp

LIB_SRCS += PSL.cpp

#==================================

# <name>.dbd will be created from <name>Include.dbd
DBD += PSLApp.dbd
DBD += PSLSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS_DEFAULT += PSLApp_registerRecordDeviceDriver.cpp PSLAppMain.cpp
PROD_SRCS_vxWorks += PSLApp_registerRecordDeviceDriver.cpp

# Add locally compiled object code
PROD_LIBS += PSL

include $(AREA_DETECTOR)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

