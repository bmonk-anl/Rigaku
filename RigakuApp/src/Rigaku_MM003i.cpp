#include <stdio.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>

#include "Rigaku_T96.h"

// Is this needed?
using namespace std;

// This needs to come before the Rikagu constructor to avoid compiler errors
static void pollerThreadC(void * pPvt)
{
  Rigaku *pRigaku = (Rigaku*)pPvt;
  pRigaku->pollerThread();
}

Rigaku::Rigaku(const char *portName, const epicsUInt32 commType, const epicsUInt32 commPort) : asynPortDriver(portName, MAX_CONTROLLERS,
		asynInt32Mask | asynFloat64Mask | asynDrvUserMask,
		asynInt32Mask | asynFloat64Mask,
		ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
		0, 0), /* Default priority and stack size */
    pollTime_(DEFAULT_POLL_TIME)
{
	static const char *functionName = "Rigaku";
	
	// Add validation in the future
	commType_ = commType;
	commPort_ = commPort;
	
	createParam(temperatureInValueString,	asynParamFloat64, &voltageInValue_);
	createParam(temperatureInValueString,	asynParamFloat64, &currentInValue_);
	createParam(temperatureInValueString,	asynParamFloat64, &voltageOutValue_);
	createParam(temperatureInValueString,	asynParamFloat64, &currentOutValue_);

	createParam(xrayReadyString,		    asynParamInt32,   &statusXrayReady_);
	createParam(xrayOnString,   		    asynParamInt32,   &statusXrayOn_);
	createParam(doorUnlockedString,		    asynParamInt32,   &statusDoorUnlocked_);
	createParam(doorOpenString, 		    asynParamInt32,   &statusDoorOpen_);

	createParam(curErrorString, 		    asynParamOctet,   &statusCurError_);

	createParam(alarm1String, 		        asynParamOctet,   &statusAlarm_[0]);
	createParam(alarm2String, 		        asynParamOctet,   &statusAlarm_[1]);
	createParam(alarm3String, 		        asynParamOctet,   &statusAlarm_[2]);
	createParam(alarm4String, 		        asynParamOctet,   &statusAlarm_[3]);
	createParam(alarm5String, 		        asynParamOctet,   &statusAlarm_[4]);

	createParam(warning1String, 		    asynParamOctet,   &statusWarnings_[0]);
	createParam(warning2String, 		    asynParamOctet,   &statusWarnings_[1]);
	createParam(warning3String, 		    asynParamOctet,   &statusWarnings_[2]);
	createParam(warning4String, 		    asynParamOctet,   &statusWarnings_[3]);
	createParam(warning5String, 		    asynParamOctet,   &statusWarnings_[4]);
  /*
	createParam(temperatureInValueString,	asynParamFloat64, &temperatureInValue_);
	createParam(rampLimitOutValueString,	asynParamFloat64, &rampLimitOutValue_);
	createParam(rampLimitInValueString,		asynParamFloat64, &rampLimitInValue_);
	createParam(rampRateOutValueString,		asynParamFloat64, &rampRateOutValue_);
	createParam(rampRateInValueString,		asynParamFloat64, &rampRateInValue_);
	createParam(heaterPowerInValueString,	asynParamFloat64, &heaterPowerInValue_);
	createParam(heatingOutValueString,		asynParamInt32,   &heatingOutValue_);
  //
	createParam(lnpModeOutValueString,		asynParamInt32,   &lnpModeOutValue_);
	createParam(lnpSpeedOutValueString,		asynParamInt32,   &lnpSpeedOutValue_);
	createParam(lnpSpeedInValueString,		asynParamFloat64, &lnpSpeedInValue_);
  //
	createParam(vacuumOutValueString,		    asynParamInt32,   &vacuumOutValue_);
	createParam(vacuumLimitOutValueString,	asynParamFloat64, &vacuumLimitOutValue_);
	createParam(vacuumLimitInValueString,		asynParamFloat64, &vacuumLimitInValue_);
	createParam(pressureInValueString,	    asynParamFloat64, &pressureInValue_);
  //
	createParam(controllerConfigInValueString,	asynParamInt32,   &controllerConfigInValue_);
	createParam(controllerErrorInValueString,		asynParamInt32,   &controllerErrorInValue_);
	createParam(controllerStatusInValueString,	asynParamInt32,   &controllerStatusInValue_);
	createParam(stageConfigInValueString,		    asynParamInt32,   &stageConfigInValue_);
  //
  createParam(statusControllerErrorString,    asynParamInt32, &statusControllerError_);
  createParam(statusRampSetpointString,       asynParamInt32, &statusRampSetpoint_);
  createParam(statusRampStartedString,        asynParamInt32, &statusRampStarted_);
  createParam(statusVacuumSetpointString,     asynParamInt32, &statusVacuumSetpoint_);
  createParam(statusVacuumStartedString,      asynParamInt32, &statusVacuumStarted_);
  createParam(statusLnpCoolingStartedString,  asynParamInt32, &statusLnpCoolingStarted_);
  createParam(statusLnpCoolingAutoString,     asynParamInt32, &statusLnpCoolingAuto_);
    */
  
	// Force the device to connect now
	connect(this->pasynUserSelf);
	
  // 
  readControllerConfig();
  
  //
  readStageConfig();
  
  /* 
   * Initialize parameters here that need to be set because init record order is
   * not predictable or because the corresponding records are PINI=NO 
   */
  //setIntegerParam(variable_, 1);
  
	// Start the poller
  epicsThreadCreate("RigakuPoller", 
      epicsThreadPriorityLow,
      epicsThreadGetStackSize(epicsThreadStackMedium),
      (EPICSTHREADFUNC)pollerThreadC,
      this);
	
  //epicsThreadSleep(5.0);
}

Rigaku::~Rigaku()
{
	// Force the controller to disconnect
	disconnect(this->pasynUserSelf);
}

asynStatus Rigaku::connect(asynUser *pasynUser)
{
	asynStatus status;
	static const char *functionName = "connect";

	// Disconnect first?
	//disconnect(pasynUser);

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: Connecting...\n", driverName, functionName);
	
	/* 
	 * Wrapped call to RigakuCommsDll.Comms.OpenComms()
	 * 
	 * OpenComms(
	 *		true,    bool bConnect: set to true to connect and false to disconnect
	 *		1,       UInt32 u32Commtype: Connection protocol (0: USB, 1: Serial)
	 *		3);      UInt32 u32CommPort: Set to the COMM port that the controller is connected to. (e.g. COMM port 3)
	 */
	this->lock();
	// OpenComms only works for Windows-style numbering of COM ports
	//commStatus_ = OpenComms(true, 1, 1);
	// USB on windows
	//commStatus_ = OpenComms(true, 0, 6);
	// A different call would be needed for Linux
	//commStatus_ = OpenCommsFromDevice(true, serialPort_);
	//
	commStatus_ = OpenComms(true, commType_, commPort_);
	
	this->unlock();
	
    /* We found the controller and everything is OK.  Signal to asynManager that we are connected. */
    status = pasynManager->exceptionConnect(this->pasynUserSelf);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling pasynManager->exceptionConnect, error=%s\n",
            driverName, functionName, pasynUserSelf->errorMessage);
        return asynError;
    }

 	return asynSuccess;
}

asynStatus Rigaku::disconnect(asynUser *pasynUser)
{
	asynStatus status;
	static const char *functionName = "disconnect";

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: Disconnecting...\n", driverName, functionName);

	this->lock();
	//commStatus_ = OpenCommsFromDevice(false, serialPort_);
	// OpenComms only works for Windows-style numbering of COM ports
	//commStatus_ = OpenComms(false, 1, 1);
	// USB on Windows
	//commStatus_ = OpenComms(false, 1, 6);
	//
	commStatus_ = OpenComms(false, commType_, commPort_);
	
	this->unlock();
	
    /* We found the controller and everything is OK.  Signal to asynManager that we are connected. */
    status = pasynManager->exceptionDisconnect(this->pasynUserSelf);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling pasynManager->exceptionDisonnect, error=%s\n",
            driverName, functionName, pasynUserSelf->errorMessage);
        return asynError;
    }

 	return asynSuccess;
}

// functions for writing and reading to device

asynStatus Rigaku::writeReadRiagku()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, 2.0);
}

asynStatus Rigaku::writeReadRigaku(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";
  
  status = pasynOctetSyncIO->writeRead(this->pasynUserSelf, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
                        
  return status;
}

/*
 * 
 * poller
 * 
 */
void Rigaku::pollerThread()
{
  /* This function runs in a separate thread.  It waits for the poll time. */
  static const char *functionName = "pollerThread";
  // Other variable declarations
  epicsInt32 ival;
  epicsFloat64 fval;

    asynStatus comStatus;
    int xray_on, voltage, current, xray_ready, door_unlocked, door_open;
    char curError[7]; // error will be 6 chars + 1 null char
    char warnings[5][7]; // 5 warnings each 6 chars
    char errors[5][7]; // 5 errors each 6 chars
    char statusCode = '\0';
  
  while (1)
  {
    lock();

    // get generator info
    // send format: GX_info_1
    // receive format (no error): GX_info_1 C <xrays on/off (1/0)> <voltage (0-60000 V)> <current (0-30000 uA)>
    // <generator warning present (1/0)> <generator alarm present (1/0)> <shutter status change (1/0)> 
    // <radiation safety change (1/0)> 
    // receive format (error): GX_info_1 E <error code> 
    sprintf(outString_, "GX_info_1");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_1 C %d %d %d", xray_on, voltage, current);
        setIntegerParam(statusXrayOn_, xray_on);
        setDoubleParam(voltageInVal_, (double)voltage);
        setDoubleParam(currentInVal_, (double)current);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get info about readiness to generate xrays 
    // send format: GX_info_xoff
    // receive format (no error): GX_info_xoff C <n/a> <n/a> <n/a> <xrays ready (1/0)>
    // receive format (error): GX_info_xoff E <error code>
    sprintf(outString_, "GX_info_xoff");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_xoff C %*d %*d %*d %d", xray_ready);
        setIntegerParam(statusXrayReady_, xray_ready);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get info about shutter status 
    // send format: GX_info_chng_shutter 
    // receive format (no error): GX_info_chng_shutter C <n/a> <n/a> <n/a> <shutter open (1/0)>
    // receive format (error): GX_info_chng_shutter E <error code>
    sprintf(outString_, "GX_info_chng_shutter");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_chng_shutter C %*d %*d %*d %d", xray_ready);
        setIntegerParam(statusShutterOpen_, shutter_open);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';

    // get info about door status 
    // send format: GX_info_chng_rs1 
    // receive format (no error): GX_info_chng_rs1 C <door unlocked (1/0)> <door open (1/0)> 
    // receive format (error): GX_info_chng_rs1 E <error code>
    sprintf(outString_, "GX_info_chng_rs1");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_chng_rs1 C %d %d", door_unlocked, door_open);
        setIntegerParam(statusDoorUnlocked_, door_unlocked);
        setIntegerParam(statusDoorOpen, door_open);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get current warnings (max 64, but we'll only use 5) 
    // send format: GX_info_warning 
    // receive format (no error): GX_info_warning C <number of warnings> <warning 1> <warning 2> ... 
    // receive format (error): GX_info_warning E <error code>
    sprintf(outString_, "GX_info_warning");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_warning C %*d %6s %6s %6s %6s %6s", 
            warning[0], warning[1], warning[2], warning[3], warning[4]);
        setStringParam(statusWarnings_[0], warning[0]);
        setStringParam(statusWarnings_[1], warning[1]);
        setStringParam(statusWarnings_[2], warning[2]);
        setStringParam(statusWarnings_[3], warning[3]);
        setStringParam(statusWarnings_[4], warning[4]);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';

    // get current alarms (max 5) 
    // send format: GX_info_alarm
    // receive format (no error): GX_info_alarm C <number of alarms> <alarm 1> <alarm 2> ... 
    // receive format (error): GX_info_alarm E <error code>
    sprintf(outString_, "GX_info_alarm");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "%*s %c", statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "GX_info_alarm C %*d %6s %6s %6s %6s %6s", 
            alarm[0], alarm[1], alarm[2], alarm[3], alarm[4]);
        setStringParam(statusAlarms_[0], alarm[0]);
        setStringParam(statusAlarms_[1], alarm[1]);
        setStringParam(statusAlarms_[2], alarm[2]);
        setStringParam(statusAlarms_[3], alarm[3]);
        setStringParam(statusAlarms_[4], alarm[4]);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "%*s E %6s", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';

    skip:

    callParamCallbacks();
    
    unlock();
    epicsThreadSleep(pollTime_);
  }
}


/*
 *
 * readFloat64
 *
 */
asynStatus Rigaku::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status;
    static const char *functionName = "readFloat64";

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, function = %d\n",
			driverName, functionName, this->portName, function);

	if (function == rampLimitInValue_) {

		status = readRampLimit(value);

	} else if (function == rampRateInValue_) {

		// Read the ramp rate from the controller (C/min)
		status = readRampRate(value);

	} else if (function == vacuumLimitInValue_) {

		// Read the vacuum limit from the controller (mBar)
		status = readVacuumLimit(value);

	} else {
		status = asynPortDriver::readFloat64(pasynUser,value);
	}

	// Update the temperatureInValue_ parameter
	setDoubleParam(function, *value);
	
	callParamCallbacks();

	return (status==0) ? asynSuccess : asynError;
}

asynStatus Rigaku::readRampLimit(epicsFloat64 *value)
{
	static const char *functionName = "readRampLimit";
	
	this->lock();
	*value = GetValue(u32Heater1LimitRW);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, *value);
	
	return asynSuccess;
}

asynStatus Rigaku::readRampRate(epicsFloat64 *value)
{
	static const char *functionName = "readRampRate";
	
	this->lock();
	*value = GetValue(u32Heater1RateRW);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, *value);
	
	return asynSuccess;
}

asynStatus Rigaku::readVacuumLimit(epicsFloat64 *value)
{
	static const char *functionName = "readVacuumLimit";
	
	this->lock();
	*value = GetValue(u32VacuumLimitRW);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, *value);
	
	return asynSuccess;
}


/*
 *
 * writeFloat64
 *
 */
asynStatus Rigaku::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	asynStatus status;
	static const char *functionName = "writeFloat64";

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, function = %d\n",
			driverName, functionName, this->portName, function);

	// Set the new value; it can be reverted later if commands fail
	setDoubleParam(function, value);

	if (function == rampLimitOutValue_) {

		// set the desired temperature
		status = setRampLimit(value);

	} else if (function == rampRateOutValue_) { 

		// set the desired ramp rate
		status = setRampRate(value);
	
	} else if (function == vacuumLimitOutValue_) { 

		// set the desired pressure
		status = setVacuumLimit(value);
	
	} else {
		status = asynPortDriver::writeFloat64(pasynUser,value);
	}

	callParamCallbacks();
	if (status == 0) {
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s, port %s, wrote %lf\n",
             driverName, functionName, this->portName, value);
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR writing %lf, status=%d\n",
             driverName, functionName, this->portName, value, status);
	}
	
	return (status==0) ? asynSuccess : asynError;
}

asynStatus Rigaku::setRampLimit(epicsFloat64 value)
{
	bool status;
	static const char *functionName = "setRampLimit";
	
	/*
	 * 	bool SetValue(
	 * 		unsigned int u32ValueType, 
	 * 		float fValue)
	 * 
	 * 	Returns true if set successfully, false otherwise.
	 */ 
	this->lock();
	status = SetValue(u32Heater1LimitRW, (float) value);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}

asynStatus Rigaku::setRampRate(epicsFloat64 value)
{
	bool status;
	static const char *functionName = "setRampRate";
	
	this->lock();
	status = SetValue(u32Heater1RateRW, (float) value);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}

asynStatus Rigaku::setVacuumLimit(epicsFloat64 value)
{
	bool status;
	static const char *functionName = "setVacuumLimit";
	
	this->lock();
	
  if (controllerVacuumReady_)
  {
    if (stageVacuum_)
    {
      // Vacuum control is supported and ready
      status = SetValue(u32VacuumLimitRW, (float) value);
    }
    else
    {
      status = false;
      
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
           "%s:%s, port %s, ERROR setting vacuum limit: stageVacuum_ = %d\n",
           driverName, functionName, this->portName, stageVacuum_);
    }
  }
  else
  {
    status = false;
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s, port %s, ERROR setting vacuum limit: controllerVacuumReady_ = %d\n",
        driverName, functionName, this->portName, controllerVacuumReady_);
  }
  
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %lf\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}


/*
 *
 * writeInt32
 *
 */
asynStatus Rigaku::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "writeInt32";

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, function = %d\n",
			driverName, functionName, this->portName, function);

	// Set the new value; it can be reverted later if commands fail
	setIntegerParam(function, value);

	if (function == heatingOutValue_) {

		// Enable/disable heating
		status = setHeating(value);

	} else if (function == lnpModeOutValue_) {
    
    // Set Manual/Auto LNP mode
    status = setLnpMode(value);
    
  } else if (function == lnpSpeedOutValue_) {
    
    // Set LN pump speed (0-100%)
    status = setLnpSpeed(value);
    
  } else if (function == vacuumOutValue_) {
    
    // Set vacuum on/off
    status = setVacuum(value);
    
  }

	
	callParamCallbacks();
	if (status == 0) {
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s, port %s, wrote %d\n",
             driverName, functionName, this->portName, value);
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR writing %d, status=%d\n",
             driverName, functionName, this->portName, value, status);
	}
	
	return (status==0) ? asynSuccess : asynError;
}

asynStatus Rigaku::setHeating(epicsInt32 value)
{
	bool status;
	static const char *functionName = "setHeating";
	
	this->lock();
	status = StartHeating((value==1) ? true : false);
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}

asynStatus Rigaku::setLnpMode(epicsInt32 value)
{
	bool status;
	static const char *functionName = "setLnpMode";
	
	this->lock();
  
  if (controllerLnpReady_)
  {
    if (value == 1)
    {
      // User requested Auto mode
      if (stageLnpAuto_ == 1)
      {
        status = SetLnpMode(true);
      } else {
        status = false;
        
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR setting LNP Mode: stageLnpAuto_ = %d\n",
             driverName, functionName, this->portName, stageLnpAuto_);
      }
    } else {
      // User requested Manual mode
      if (stageLnpManual_ == 1)
      {
        status = SetLnpMode(false);
      } else {
        status = false;
        
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR setting LNP Mode: stageLnpManual_ = %d\n",
             driverName, functionName, this->portName, stageLnpManual_);
      }
    }
  } else {
    status = false;
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s, port %s, ERROR setting LNP Mode: controllerLnpReady_ = %d\n",
        driverName, functionName, this->portName, controllerLnpReady_);
  }
  
  // Manual = 0 = false ; Auto = 1 = true
	//status = SetLnpMode((value==1) ? true : false);
  
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}

asynStatus Rigaku::setLnpSpeed(epicsInt32 value)
{
	bool status;
	static const char *functionName = "setLnpSpeed";
	
	this->lock();
  
  if (controllerLnpReady_)
  {
    // 0-100%
    if (stageLnpManual_ == 1)
    {
      status = SetLnpSpeed(value);
    } else {
      status = false;
      
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
           "%s:%s, port %s, ERROR setting LNP Speed: stageLnpManual_ = %d\n",
           driverName, functionName, this->portName, stageLnpManual_);
    }
  } else {
    status = false;
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s, port %s, ERROR setting LNP Speed: controllerLnpReady_ = %d\n",
        driverName, functionName, this->portName, controllerLnpReady_);
  }
  
  // 0-100%
	//status = SetLnpMode(value);
	
  this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}

asynStatus Rigaku::setVacuum(epicsInt32 value)
{
	bool status;
	static const char *functionName = "setVacuum";
	
	this->lock();
  
  // Off = 0 = false ; On = 1 = true
  if (controllerVacuumReady_)
  {
    if (stageVacuum_)
    {
      // Vacuum control is supported and ready
      status = StartVacuum((value) ? true : false);
    }
    else
    {
      status = false;
      
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
           "%s:%s, port %s, ERROR setting vacuum: stageVacuum_ = %d\n",
           driverName, functionName, this->portName, stageVacuum_);
    }
  }
  else
  {
    status = false;
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s, port %s, ERROR setting vacuum: controllerVacuumReady_ = %d\n",
        driverName, functionName, this->portName, controllerVacuumReady_);
  }
  
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return (status) ? asynSuccess : asynError;
}


/*
 *
 * readInt32
 *
 */
asynStatus Rigaku::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "readInt32";

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, function = %d\n",
			driverName, functionName, this->portName, function);

	// Get the current value; it can be reverted later if commands fail
	getIntegerParam(function, value);

	if (function == controllerConfigInValue_) {
    
		status = readControllerConfig();
    
	} else if (function == controllerErrorInValue_) {
    
    status = readControllerError();
    
	} else if (function == controllerStatusInValue_) {
    
    status = readControllerStatus();
    
	} else if (function == stageConfigInValue_) {
    
    status = readStageConfig();
    
  }
	
  // Increment the value so we can see that records are processing
  if (*value > 100000)
    *value = 0;
  else
    *value += 1;
  
	callParamCallbacks();
	if (status == 0) {
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
             "%s:%s, port %s, wrote %d\n",
             driverName, functionName, this->portName, value);
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, 
             "%s:%s, port %s, ERROR writing %d, status=%d\n",
             driverName, functionName, this->portName, value, status);
	}
	
	return (status==0) ? asynSuccess : asynError;
}

asynStatus Rigaku::readControllerConfig()
{
	static const char *functionName = "readControllerConfig";
	
	this->lock();
	this->controllerConfig_ = GetControllerConfig();
	
  if (controllerConfig_ & ( (epicsUInt64)1 << (int)u64LnpReady ) )
    controllerLnpReady_ = 1;
  else
    controllerLnpReady_ = 0;
  
  if (controllerConfig_ & ( (epicsUInt64)1 << (int)u64VacuumReady ) )
    controllerVacuumReady_ = 1;
  else
    controllerVacuumReady_ = 0;
  
  this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, controllerConfig_ = %llx; controllerLnpReady_ = %d\n",
			driverName, functionName, this->portName, this->controllerConfig_, this->controllerLnpReady_);
	
	return asynSuccess;
}

asynStatus Rigaku::readControllerError()
{
	static const char *functionName = "readControllerError";
	
	this->lock();
	this->controllerError_ = GetControllerError();
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, controllerError_ = %lx\n",
			driverName, functionName, this->portName, this->controllerError_);
	
	return asynSuccess;
}

asynStatus Rigaku::readControllerStatus()
{
	static const char *functionName = "readControllerStatus";
	
	this->lock();
	this->controllerStatus_ = GetStatus();
	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, controllerStatus_ = %llx\n",
			driverName, functionName, this->portName, this->controllerStatus_);
	
	return asynSuccess;
}

asynStatus Rigaku::readStageConfig()
{
    static const char *functionName = "readStageConfig";
    
    this->lock();
    this->stageConfig_ = GetStageConfig();
    this->unlock();
	
    if (stageConfig_ & ( (epicsUInt64)1 << (int)u64SupportsLNPMan ) )
        stageLnpManual_ = 1;
    else
        stageLnpManual_ = 0;
      
    if (stageConfig_ & ( (epicsUInt64)1 << (int)u64SupportsLNPAuto ) )
        stageLnpAuto_ = 1;
    else
        stageLnpAuto_ = 0;
      
    if (stageConfig_ & ( (epicsUInt64)1 << (int)u64SupportsVacuum ) )
        stageVacuum_ = 1;
    else
        stageVacuum_ = 0;
  
        asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
		    "%s:%s, port %s, stageConfig_ = %llx, stageLnpManual_ = %d, stageLnpAuto_ = %d\n",
		    driverName, functionName, this->portName, this->stageConfig_, stageLnpManual_, stageLnpAuto_);
	
	return asynSuccess;
}


void Rigaku::report(FILE *fp, int details)
{
	asynPortDriver::report(fp, details);
	fprintf(fp, "* Port: %s, commType=%d, commPort=%d, commStatus=%d\n", this->portName, commType_, commPort_, commStatus_);
	if (details >= 1) {
        fprintf(fp, "  controller config = %llx\n", controllerConfig_);
        // fprintf(fp, "\tLNP Ready = %d\n", controllerLnpReady_);
        // fprintf(fp, "\tVac Ready = %d\n", controllerVacuumReady_);
        fprintf(fp, "  controller error  = %lx\n", controllerError_);
        fprintf(fp, "  controller status = %llx\n", controllerStatus_);
        fprintf(fp, "  stage config      = %llx\n", stageConfig_);
        // fprintf(fp, "\tLNP Manual = %d\n", stageLnpManual_);
        // fprintf(fp, "\tLNP Auto = %d\n", stageLnpAuto_);
        // fprintf(fp, "\tVacuum = %d\n", stageVacuum_);
    }
	fprintf(fp, "\n");
}


extern "C" int RigakuConfig(const char *portName, const epicsUInt32 commType, const epicsUInt32 commPort)
{
    Rigaku *pRigaku = new Rigaku(portName, commType, commPort);
    pRigaku = NULL; /* This is just to avoid compiler warnings */
    return(asynSuccess);
}

static const iocshArg rigakuArg0 = { "Port name", iocshArgString};
static const iocshArg rigakuArg1 = { "Comm Type", iocshArgInt};
static const iocshArg rigakuArg2 = { "Comm port", iocshArgInt};
static const iocshArg * const rigakuArgs[3] = {&rigakuArg0,
											   &rigakuArg1,
											   &rigakuArg2};
static const iocshFuncDef rigakuFuncDef = {"RigakuConfig", 3, rigakuArgs};
static void rigakuCallFunc(const iocshArgBuf *args)
{
    RigakuConfig(args[0].sval, args[1].ival, args[2].ival);
}

void drvRigakuRegister(void)
{
    iocshRegister(&rigakuFuncDef,rigakuCallFunc);
}

extern "C" {
    epicsExportRegistrar(drvRigakuRegister);
}
