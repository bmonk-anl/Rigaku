TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS=-I/usr/include/mono-2.0
#USR_CXXFLAGS=-clr:safe

#==================================================
# build a support library

LIBRARY_IOC += Rigaku 

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install Rigaku.dbd into <top>/dbd
DBD += RigakuSupport.dbd

# specify all source files to be compiled and added to the library
Rigaku_SRCS += Rigaku_MM003i.cpp
# Rigaku_LIBS_Linux += Wrapper
# ifeq (win, $(findstring win, $(T_A)))
# Rigaku_LIBS += CommsWrapper
# endif

Rigaku_LIBS += asyn
Rigaku_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

