/*
 * 
 * 
 * 
 */
// TODO:
// add number of alarms/warnings

#include <asynPortDriver.h>

static const char *driverName = "Rigaku";

#define MAX_CONTROLLERS	1
#define DEFAULT_POLL_TIME 0.05

// // eVALUETYPE Enumeration
// #define u32Heater1TempR		  0
// #define u32Heater1RateRW	  1
// #define u32Heater1LimitRW	  2
// #define u32Heater1PowerR	  3
// #define u32Heater1LnpSpeedR 4
// #define u32VacuumR          12
// #define u32VacuumLimitRW    13
// 
// // eSTAGECONFIG Enumeration
// #define u64StandardStage    0
// #define u64HighTempStage    1
// #define u64PeltierStage     2
// #define u64SupportsLNPMan   21
// #define u64SupportsLNPAuto  22
// #define u64SupportsHeater1Present   26
// #define u64Heater1SupportsTemperatureControl 27
// #define u64SupportsVacuum   48
// 
// // eCONTROLLERCONFIG Enumeration
// #define u64SupportsHeater 0
// #define u64VacuumReady    10
// #define u64LnpReady       36
// 
// // eSTATUS Enumeration
// #define u64ControllerError          0
// #define u64Heater1RampSetpoint      1
// #define u64Heater1Started           2
// #define u64VacuumSetPoint           5
// #define u64VacuumControlStarted     6
// #define u64LnpCoolingStarted        11
// #define u64LnpCoolingAuto           12

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define voltageInValueString	"VOLTAGE_IN_VALUE"	/* asynFloat64 r/o */
#define currentInValueString	"CURRENT_IN_VALUE"	/* asynFloat64 r/o */
#define voltageInValueString	"VOLTAGE_OUT_VALUE"	/* asynFloat64 r/w */
#define currentInValueString	"CURRENT_OUT_VALUE"	/* asynFloat64 r/w */

#define currentInValueString	"STATUS_XRAY_READY"	/* asynFloat64 r/w */

// #define temperatureInValueString	"TEMPERATURE_IN_VALUE"	/* asynFloat64 r/o */
// #define rampLimitOutValueString		"RAMP_LIMIT_OUT_VALUE"	/* asynFloat64 r/w */
// #define rampLimitInValueString		"RAMP_LIMIT_IN_VALUE"	/* asynFloat64 r/o */
// #define rampRateOutValueString		"RAMP_RATE_OUT_VALUE"	/* asynFloat64 r/w */
// #define rampRateInValueString		"RAMP_RATE_IN_VALUE"	/* asynFloat64 r/o */
// #define heaterPowerInValueString	"HEATER_POWER_IN_VALUE"	/* asynFloat64 r/o */
// #define heatingOutValueString		"HEATING_OUT_VALUE"		/* asynInt32  r/w */
// //
// #define lnpModeOutValueString		"LNP_MODE_OUT_VALUE"		/* asynInt32  r/w */
// #define lnpSpeedOutValueString		"LNP_SPEED_OUT_VALUE"		/* asynInt32  r/w */
// #define lnpSpeedInValueString		"LNP_SPEED_IN_VALUE"		/* asynFloat64  r/o */
// //
// #define vacuumOutValueString		"VACUUM_OUT_VALUE"		/* asynInt32  r/w */
// #define vacuumLimitOutValueString		"VACUUM_LIMIT_OUT_VALUE"	/* asynFloat64 r/w */
// #define vacuumLimitInValueString		"VACUUM_LIMIT_IN_VALUE"	/* asynFloat64 r/o */
// #define pressureInValueString	"PRESSURE_IN_VALUE"	/* asynFloat64 r/o */
// //
// #define controllerConfigInValueString	"CONTROLLER_CONFIG_IN_VALUE"	/* asynInt32 r/o */
// #define controllerErrorInValueString	"CONTROLLER_ERROR_IN_VALUE"	/* asynInt32 r/o */
// #define controllerStatusInValueString	"CONTROLLER_STATUS_IN_VALUE"	/* asynInt32 r/o */
// #define stageConfigInValueString	"STAGE_CONFIG_IN_VALUE"	/* asynInt32 r/o */
// //
// #define statusControllerErrorString   "STATUS_CONTROLLER_ERROR"     /* asynInt32 r/o */
// #define statusRampSetpointString      "STATUS_RAMP_SETPOINT"        /* asynInt32 r/o */
// #define statusRampStartedString       "STATUS_RAMP_STARTED"         /* asynInt32 r/o */
// #define statusVacuumSetpointString    "STATUS_VACUUM_SETPOINT"      /* asynInt32 r/o */
// #define statusVacuumStartedString     "STATUS_VACUUM_STARTED"       /* asynInt32 r/o */
// #define statusLnpCoolingStartedString "STATUS_LNP_COOLING_STARTED"  /* asynInt32 r/o */
// #define statusLnpCoolingAutoString    "STATUS_LNP_COOLING_AUTO"     /* asynInt32 r/o */

/*
 * Class definition for the Linkam class
 */
class Rigaku: public asynPortDriver {
public:
    Rikagu(const char *portName, const epicsUInt32 commType, const epicsUInt32 commPort);
    
    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    // value - pointer to string
    // maxChars - max num of characters to read
    //
    virtual asynStatus readOctet(asynUser *pasynUser, char* value, size_t maxChars, 
        size_t* nActual, int* eomReason);
    virtual asynStatus disconnect(asynUser *pasynUser);
    virtual asynStatus connect(asynUser *pasynUser);
    virtual ~Rigaku();
    // These should be private but are called from C
    virtual void pollerThread(void);

protected:
    // "float" values
    int voltageInVal_;
    int currentInVal_;
    int voltageOutVal_;
    int currentOutVal_;
    // binary values
    int statusXrayReady_;
    int statusXrayOn_;
    int statusShutterOpen_;
    int statusDoorUnlocked_;
    int statusDoorOpen_;
    // string values
    int statusCurError_;
    int statusAlarms_[5];
    int statusWarnings_[5];

	#define FIRST_RIGAKU_PARAM voltageInVal_;
	#define LAST_RIGAKU_PARAM statusWarning5_;

private:
    char outString_[256];
    char inString_[256];
    
    asynStatus writeReadRigaku();
    asynStatus writeReadRigaku(const char *output, char *response, size_t maxResponseLen, 
        size_t *responseLen, double timeout);
  //
	// asynStatus readRampLimit(epicsFloat64 *value);
	// asynStatus readRampRate(epicsFloat64 *value);
	// asynStatus setRampLimit(epicsFloat64 value);
	// asynStatus setRampRate(epicsFloat64 value);
	// asynStatus setHeating(epicsInt32 value);
  //
	// asynStatus setLnpMode(epicsInt32 value);
	// asynStatus setLnpSpeed(epicsInt32 value);
  //
  // asynStatus setVacuum(epicsInt32 value);
  // asynStatus setVacuumLimit(epicsFloat64 value);
  // asynStatus readVacuumLimit(epicsFloat64 *value);
  //
	asynStatus readControllerConfig();
	asynStatus readControllerError();
	asynStatus readControllerStatus();
	asynStatus readStageConfig();
	void report(FILE *fp, int details);

  double pollTime_;
	epicsUInt32 commType_;
	epicsUInt32 commPort_;
	int commStatus_;
	// float temperatureRbv_;
	// char* wrapperVersion_;
	// char* libraryVersion_;
  epicsUInt64 controllerConfig_;
  epicsUInt32 controllerError_;
  epicsUInt64 controllerStatus_;
  epicsUInt64 stageConfig_;
  short controllerLnpReady_;
  short stageLnpAuto_;
  short stageLnpManual_;
  short controllerVacuumReady_;
  short stageVacuum_;
  
};

#define NUM_PARAMS ((int)(&LAST_LINKAM_PARAM - &FIRST_LINKAM_PARAM + 1))


