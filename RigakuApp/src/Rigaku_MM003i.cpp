#include <stdio.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <string.h>

#include "Rigaku_MM003i.h"

// Is this needed?
using namespace std;

// This needs to come before the Rigaku constructor to avoid compiler errors
static void pollerThreadC(void * pPvt)
{
  Rigaku *pRigaku = (Rigaku*)pPvt;
  pRigaku->pollerThread();
}

static void rampUpThreadC(void * pPvt)
{
  Rigaku *pRigaku = (Rigaku*)pPvt;
  pRigaku->rampUpThread();
}

Rigaku::Rigaku(const char *portName, const char *RigakuPortName) : asynPortDriver(portName, MAX_CONTROLLERS,
		asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask,
		asynInt32Mask | asynFloat64Mask | asynOctetMask,
		ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
		0, 0), /* Default priority and stack size */
    pollTime_(DEFAULT_POLL_TIME)
{
	static const char *functionName = "Rigaku";
    asynStatus status;
	
	createParam(voltageInValueString,   	asynParamFloat64, &voltageInVal_);
	createParam(currentInValueString,	    asynParamFloat64, &currentInVal_);
	createParam(voltageOutValueString,  	asynParamFloat64, &voltageOutVal_);
	createParam(currentOutValueString,  	asynParamFloat64, &currentOutVal_);

	createParam(setPowerString,		        asynParamInt32,   &setPowerOut_);
	createParam(setDoorLockString,	        asynParamInt32,   &setDoorLockOut_);
	createParam(setXrayOnString,	        asynParamInt32,   &setXrayOnOut_);
	createParam(setShutterOpenString,       asynParamInt32,   &setShutterOpenOut_);

	createParam(resetWarning1String,        asynParamInt32,   &resetWarningsOut_[0]);
	createParam(resetWarning2String,        asynParamInt32,   &resetWarningsOut_[1]);
	createParam(resetWarning3String,        asynParamInt32,   &resetWarningsOut_[2]);
	createParam(resetWarning4String,        asynParamInt32,   &resetWarningsOut_[3]);
	createParam(resetWarning5String,        asynParamInt32,   &resetWarningsOut_[4]);
	createParam(resetAlarm1String,          asynParamInt32,   &resetAlarmsOut_[0]);
	createParam(resetAlarm2String,          asynParamInt32,   &resetAlarmsOut_[1]);
	createParam(resetAlarm3String,          asynParamInt32,   &resetAlarmsOut_[2]);
	createParam(resetAlarm4String,          asynParamInt32,   &resetAlarmsOut_[3]);
	createParam(resetAlarm5String,          asynParamInt32,   &resetAlarmsOut_[4]);

	createParam(xrayReadyString,		    asynParamInt32,   &statusXrayReady_);
	createParam(xrayOnString,   		    asynParamInt32,   &statusXrayOn_);
	createParam(shutterOpenString, 		    asynParamInt32,   &statusShutterOpen_);
	createParam(doorUnlockedString,		    asynParamInt32,   &statusDoorUnlocked_);
	createParam(doorOpenString, 		    asynParamInt32,   &statusDoorOpen_);

	createParam(curErrorString, 		    asynParamOctet,   &statusCurError_);
	createParam(warning1String, 		    asynParamOctet,   &statusWarnings_[0]);
	createParam(warning2String, 		    asynParamOctet,   &statusWarnings_[1]);
	createParam(warning3String, 		    asynParamOctet,   &statusWarnings_[2]);
	createParam(warning4String, 		    asynParamOctet,   &statusWarnings_[3]);
	createParam(warning5String, 		    asynParamOctet,   &statusWarnings_[4]);
	createParam(alarm1String, 		        asynParamOctet,   &statusAlarms_[0]);
	createParam(alarm2String, 		        asynParamOctet,   &statusAlarms_[1]);
	createParam(alarm3String, 		        asynParamOctet,   &statusAlarms_[2]);
	createParam(alarm4String, 		        asynParamOctet,   &statusAlarms_[3]);
	createParam(alarm5String, 		        asynParamOctet,   &statusAlarms_[4]);

    createParam(rampUpDelayString,         asynParamFloat64,  &rampUpDelay_);
    createParam(rampUpDelayRemainString,   asynParamFloat64,  &rampUpDelayRemain_);
    createParam(rampUpStepTimeVString,     asynParamFloat64,  &rampUpStepTimeV_);
    createParam(rampUpStepTimeIString,     asynParamFloat64,  &rampUpStepTimeI_);
    createParam(rampUpStepRemainString,    asynParamFloat64,  &rampUpStepRemain_);
    createParam(rampUpTargetVString,       asynParamFloat64,  &rampUpTargetV_);
    createParam(rampUpTargetIString,       asynParamFloat64,  &rampUpTargetI_);
    createParam(rampUpStepVString,         asynParamFloat64,  &rampUpStepV_);
    createParam(rampUpStepIString,         asynParamFloat64,  &rampUpStepI_);
    createParam(rampUpRunString,           asynParamInt32,    &rampUpRun_);

    createParam(rampStopString,            asynParamInt32,    &rampStop_);

	// Force the device to connect now
	//connect(this->pasynUserSelf);
    status = pasynOctetSyncIO->connect(RigakuPortName, 0, &pasynUserRigaku_, NULL);
	
  // 
  // readControllerConfig();
  
  //
  // readStageConfig();
  
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

// Rigaku::~Rigaku()
// {
// 	// Force the controller to disconnect
// 	//disconnect(this->pasynUserSelf);
// }

// functions for writing and reading to device

asynStatus Rigaku::writeReadRigaku()
{
  size_t nread;
  return writeReadRigaku(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

asynStatus Rigaku::writeReadRigaku(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadRigaku";
  
  status = pasynOctetSyncIO->writeRead(pasynUserRigaku_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
                        
  return status;
}
void Rigaku::rampUpThread()
{
    double delay, stepTimeV, stepTimeI;
    double targetV, targetI, stepV, stepI;
    double curV, curI;
    
    // set stop value to 0
    rampStopVal = 0;

    // fetch needed parameters
    getDoubleParam(rampUpDelay_, &delay);
    getDoubleParam(rampUpStepTimeV_, &stepTimeV);
    getDoubleParam(rampUpStepTimeI_, &stepTimeI);
    getDoubleParam(rampUpTargetV_, &targetV);
    getDoubleParam(rampUpTargetI_, &targetI);
    getDoubleParam(rampUpStepV_, &stepV);
    getDoubleParam(rampUpStepI_, &stepI);


    if ((delay < 0) || (stepTimeV < 0) || (stepTimeI < 0)) return;
    // check steps and target

    // initial delay while posting time every second
    while (delay > 0) {
    setDoubleParam(rampUpDelayRemain_, delay);
        if (rampStopVal) {
            setDoubleParam(rampUpDelayRemain_, 0.0);
            return;
        }   
        epicsThreadSleep(1.0);
        delay -= 1;
    }
    setDoubleParam(rampUpDelayRemain_, delay);

    // get current rbv voltage and current
    getDoubleParam(voltageInVal_, &curV);
    getDoubleParam(currentInVal_, &curI);

    // source should have some current and voltage
    if ( (curV==0) || (curI==0) ) return;
    // ramp up voltage
    while (curV < targetV) {

        // increment voltage by step value
        curV = curV + stepV;         
        if (curV > targetV) curV = targetV;

        // set power
        setDoubleParam(voltageOutVal_, curV); 
        setDoubleParam(currentOutVal_, curI); 
        setPower(1);

        // avoid extra step time
        if (curI == targetI) goto endVLoop;

        while (stepTimeV > 0) {
            setDoubleParam(rampUpStepRemain_, stepTimeV);
            if (rampStopVal) {
                setDoubleParam(rampUpStepRemain_, 0.0);
                return;
            }   
            epicsThreadSleep(1.0);
            stepTimeV -= 1;
        }
        setDoubleParam(rampUpStepRemain_, stepTimeV);
        
        endVLoop:
        //reset step time
        getDoubleParam(rampUpStepTimeV_, &stepTimeV);
    }

    // get current rbv voltage and current
    getDoubleParam(voltageInVal_, &curV);
    getDoubleParam(currentInVal_, &curI);

    // source should have some current and voltage
    if ( (curV==0) || (curI==0) ) return;
    // ramp up current 
    while (curI < targetI) {
        // increment current by step value
        curI = curI + stepI;         
        if (curI > targetI) curI = targetI;

        // set power
        setDoubleParam(voltageOutVal_, curV); 
        setDoubleParam(currentOutVal_, curI); 
        setPower(1);

        // avoid extra step time
        if (curI == targetI) goto endILoop;

        while (stepTimeI > 0) {
            setDoubleParam(rampUpStepRemain_, stepTimeI);
            if (rampStopVal) {
                setDoubleParam(rampUpStepRemain_, 0.0);
                return;
            }   
            epicsThreadSleep(1.0);
            stepTimeI -= 1;
        }
        setDoubleParam(rampUpStepRemain_, stepTimeI);

        endILoop:
        //reset step time
        getDoubleParam(rampUpStepTimeI_, &stepTimeI);
    }
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
    asynStatus comStatus;
    int xray_on, voltage, current, xray_ready, door_unlocked, door_open, shutter_open;
    char curError[7] = ""; // error will be 6 chars + 1 null char

    char warnings[5][7];
    char alarms[5][7];

    char statusCode;// = '\0';
  
  while (1)
  {
    // set warnings and alarms to null so that old ones aren't set
    for (int i=0; i<5; i++) {
        for (int j=0; j<7; j++) {
            warnings[i][j] = '\0';
            alarms[i][j] = '\0';
        }
    }

    lock();

    // get generator info
    // send format: EXE(GX_info_1)
    // receive format (no err): EXE(GX_info_1 C <xrays on/off (1/0)> <voltage (0-60000 V)> <current (0-30000 uA)>
    // <generator warning present (1/0)> <generator alarm present (1/0)> <shutter status change (1/0)>)
    // <radiation safety change (1/0)> 
    // receive format (error): GX_info_1 E <error code> 
    sprintf(outString_, "EXE(GX_info_1)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_1 C %d %d %d", &xray_on, &voltage, &current);
        setIntegerParam(statusXrayOn_, xray_on);
        setDoubleParam(voltageInVal_, (double)voltage);
        setDoubleParam(currentInVal_, (double)current);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get info about readiness to generate xrays 
    // send format: GX_info_xoff
    // receive format (no error): GX_info_xoff C <n/a> <n/a> <n/a> <xrays ready (1/0)>
    // receive format (error): GX_info_xoff E <error code>
    sprintf(outString_, "EXE(GX_info_xoff)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_xoff C %*d %*d %*d %d", &xray_ready);
        setIntegerParam(statusXrayReady_, (epicsInt32)xray_ready);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get info about shutter status 
    // send format: GX_info_chng_shutter 
    // receive format (no error): GX_info_chng_shutter C <n/a> <n/a> <n/a> <shutter open (1/0)>
    // receive format (error): GX_info_chng_shutter E <error code>
    sprintf(outString_, "EXE(GX_info_chng_shutter)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_chng_shutter C %*d %*d %*d %d", &shutter_open);
        setIntegerParam(statusShutterOpen_, shutter_open);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';

    // get info about door status 
    // send format: GX_info_chng_rs1 
    // receive format (no error): GX_info_chng_rs1 C <door unlocked (1/0)> <door open (1/0)> 
    // receive format (error): GX_info_chng_rs1 E <error code>
    sprintf(outString_, "EXE(GX_info_chng_rs1)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_chng_rs1 C %d %d", &door_unlocked, &door_open);
        setIntegerParam(statusDoorUnlocked_, door_unlocked);
        setIntegerParam(statusDoorOpen_, door_open);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';
    
    // get current warnings (max 64, but we'll only use 5) 
    // send format: GX_info_warning 
    // receive format (no error): GX_info_warning C <number of warnings> <warning 1> <warning 2> ... 
    // receive format (error): GX_info_warning E <error code>
    sprintf(outString_, "EXE(GX_info_warning)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_warning C %6[^ )] %6[^ )] %6[^ )] %6[^ )] %6[^ )])", 
            warnings[0], warnings[1], warnings[2], warnings[3], warnings[4]);
        setStringParam(statusWarnings_[0], warnings[0]);
        setStringParam(statusWarnings_[1], warnings[1]);
        setStringParam(statusWarnings_[2], warnings[2]);
        setStringParam(statusWarnings_[3], warnings[3]);
        setStringParam(statusWarnings_[4], warnings[4]);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }
    // clear status code
    statusCode = '\0';

    // get current alarms (max 5) 
    // send format: GX_info_alarm
    // receive format (no error): GX_info_alarm C <number of alarms> <alarm 1> <alarm 2> ... 
    // receive format (error): GX_info_alarm E <error code>
    sprintf(outString_, "EXE(GX_info_alarm)");
    comStatus = writeReadRigaku();
    if (comStatus) goto skip;
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'C') {
        sscanf(inString_, "EXE(GX_info_alarm C %6[^ )] %6[^ )] %6[^ )] %6[^ )] %6[^ )])", 
            alarms[0], alarms[1], alarms[2], alarms[3], alarms[4]);
        setStringParam(statusAlarms_[0], alarms[0]);
        setStringParam(statusAlarms_[1], alarms[1]);
        setStringParam(statusAlarms_[2], alarms[2]);
        setStringParam(statusAlarms_[3], alarms[3]);
        setStringParam(statusAlarms_[4], alarms[4]);
        // clear curError
        curError[0] = '\0';
    } 
    else if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
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

	if (function == setPowerOut_)               status = setPower(value);
    else if (function == setDoorLockOut_)       status = setDoorLock(value);
    else if (function == setXrayOnOut_)         status = setXrayOn(value);
    else if (function == setShutterOpenOut_)    status = setShutterOpen(value); 
    else if (function == resetWarningsOut_[0])  status = resetWarning(value, 0);
    else if (function == resetWarningsOut_[1])  status = resetWarning(value, 1);
    else if (function == resetWarningsOut_[2])  status = resetWarning(value, 2);
    else if (function == resetWarningsOut_[3])  status = resetWarning(value, 3);
    else if (function == resetWarningsOut_[4])  status = resetWarning(value, 4);
    else if (function == resetAlarmsOut_[0])    status = resetAlarm(value, 0);
    else if (function == resetAlarmsOut_[1])    status = resetAlarm(value, 1);
    else if (function == resetAlarmsOut_[2])    status = resetAlarm(value, 2);
    else if (function == resetAlarmsOut_[3])    status = resetAlarm(value, 3);
    else if (function == resetAlarmsOut_[4])    status = resetAlarm(value, 4);
    else if (function == rampUpRun_)            status = rampUpRun(value); 
    else if (function == rampStop_)             status = rampStop(value); 
	
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

// set voltage and current values
// first get values from record and then use set power command
asynStatus Rigaku::setPower(epicsInt32 value)
{

	static const char *functionName = "setPower";

    // only send if record processed with value of 1 
    if (value != 1) return asynSuccess;

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";
    double voltage, current;

    getDoubleParam(voltageOutVal_, &voltage);
    getDoubleParam(currentOutVal_, &current);
	
	this->lock();

    sprintf(outString_, "EXE(CX_power %d %d)", (int)voltage, (int)current);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return comStatus;
}


asynStatus Rigaku::setDoorLock(epicsInt32 value)
{

	static const char *functionName = "setDoorLock";

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";

	this->lock();

    sprintf(outString_, "EXE(CX_door_lock1 %d)", value);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return comStatus;
}

asynStatus Rigaku::setXrayOn(epicsInt32 value)
{

	static const char *functionName = "setXrayOn";

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";

	this->lock();

    sprintf(outString_, "EXE(CX_xray %d)", value);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return comStatus;
}

asynStatus Rigaku::setShutterOpen(epicsInt32 value)
{

	static const char *functionName = "setShutterOpen";

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";

	this->lock();

    sprintf(outString_, "EXE(CX_shutter_b %d)", value);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

	this->unlock();
	
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
			"%s:%s, port %s, value = %d\n",
			driverName, functionName, this->portName, value);
	
	return comStatus;
}

asynStatus Rigaku::resetWarning(epicsInt32 value, int i_warning)
{

    static const char *functionName = "resetWarning";

    // only send if record processed with value of 1 
    if (value != 1) return asynSuccess;

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";
    char warningCode[7] = "";

    // warningCode = warnings[i_warning];
    getStringParam(statusWarnings_[i_warning], 10, warningCode);
    // if doesn't contain warning code, return
    if (warningCode[0] != 'W') return asynSuccess;

    this->lock();

    sprintf(outString_, "EXE(CX_reset_warning %6s)", warningCode);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

    this->unlock();

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s:%s, port %s, value = %d\n, warning = %s", 
        driverName, functionName, this->portName, value, warningCode);

    return comStatus;
}

asynStatus Rigaku::resetAlarm(epicsInt32 value, int i_alarm)
{

    static const char *functionName = "resetAlarm";

    // only send if record processed with value of 1 
    if (value != 1) return asynSuccess;

    asynStatus comStatus;
    char statusCode = '\0';
    char curError[7] = "";
    char alarmCode[7] = "";

    getStringParam(statusAlarms_[i_alarm], 10, alarmCode);
    // if doesn't contain alarm code, return
    if (alarmCode[0] != 'A') return asynSuccess;

    this->lock();

    sprintf(outString_, "EXE(CX_reset_alarm %s)", alarmCode);
    comStatus = writeReadRigaku();
    // get status code, either C for ok or E for error
    sscanf(inString_, "EXE(%*s %c)", &statusCode);
    if (statusCode == 'E') {
        sscanf(inString_, "EXE(%*s E %6s)", curError);
        setStringParam(statusCurError_, curError);
    }

    this->unlock();

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s:%s, port %s, value = %d\n, alarm = %s", 
        driverName, functionName, this->portName, value, alarmCode);

    return comStatus;
}

asynStatus Rigaku::rampUpRun(epicsInt32 value)
{

    static const char *functionName = "rampUpRun";

    if (value != 1) return asynSuccess;

    // start ramp thread if not running:
    if (epicsThreadGetId("RigakuRampUp") == NULL) { 
        epicsThreadCreate("RigakuRampUp", 
            epicsThreadPriorityLow,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)rampUpThreadC,
            this);
    }


    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
    		"%s:%s, port %s, value = %d\n",
    		driverName, functionName, this->portName, value);
    
    return asynSuccess;
}

// called on stop button press. sets rampStopVal which is referenced in ramp sequence
asynStatus Rigaku::rampStop(epicsInt32 value)
{

    static const char *functionName = "rampStop";

    rampStopVal = value;

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
    		"%s:%s, port %s, value = %d\n",
    		driverName, functionName, this->portName, value);
    
    return asynSuccess;
}

void Rigaku::report(FILE *fp, int details)
{
    asynPortDriver::report(fp, details);
    fprintf(fp, "* Port: %s\n", 
        this->portName);
	// if (details >= 1) {
    //     // fprintf(fp, "\tVacuum = %d\n", stageVacuum_);
    // }
    fprintf(fp, "\n");
}

extern "C" int RigakuConfig(const char *portName, const char *RigakuPortName)
{
    Rigaku *pRigaku = new Rigaku(portName, RigakuPortName);
    pRigaku = NULL; /* This is just to avoid compiler warnings */
    return(asynSuccess);
}

static const iocshArg rigakuArg0 = { "Port name", iocshArgString};
static const iocshArg rigakuArg1 = { "Rigaku port name", iocshArgString};
static const iocshArg * const rigakuArgs[2] = {&rigakuArg0,
											   &rigakuArg1};
static const iocshFuncDef rigakuFuncDef = {"RigakuConfig", 2, rigakuArgs};
static void rigakuCallFunc(const iocshArgBuf *args)
{
    RigakuConfig(args[0].sval, args[1].sval);
}

void drvRigakuRegister(void)
{
    iocshRegister(&rigakuFuncDef,rigakuCallFunc);
}

extern "C" {
    epicsExportRegistrar(drvRigakuRegister);
}
