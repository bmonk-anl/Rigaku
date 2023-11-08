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

#define DEFAULT_CONTROLLER_TIMEOUT 2.0

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */

#define voltageInValueString	"VOLTAGE_IN_VALUE"	/* asynFloat64 r/o */
#define currentInValueString	"CURRENT_IN_VALUE"	/* asynFloat64 r/o */
#define voltageOutValueString	"VOLTAGE_OUT_VALUE"	/* asynFloat64 r/w */
#define currentOutValueString	"CURRENT_OUT_VALUE"	/* asynFloat64 r/w */

#define setPowerString          "SET_POWER"	        /* asynInt32 r/w */
#define setDoorLockString       "SET_DOOR_LOCK"     /* asynInt32 r/w */
#define setXrayOnString         "SET_XRAY_ON"       /* asynInt32 r/w */
#define setShutterOpenString    "SET_SHUTTER_OPEN"  /* asynInt32 r/w */

#define resetWarning1String     "RESET_WARNING_1"   /* asynInt32 r/w */
#define resetWarning2String     "RESET_WARNING_2"   /* asynInt32 r/w */
#define resetWarning3String     "RESET_WARNING_3"   /* asynInt32 r/w */
#define resetWarning4String     "RESET_WARNING_4"   /* asynInt32 r/w */
#define resetWarning5String     "RESET_WARNING_5"   /* asynInt32 r/w */
#define resetAlarm1String       "RESET_ALARM_1"     /* asynInt32 r/w */
#define resetAlarm2String       "RESET_ALARM_2"     /* asynInt32 r/w */
#define resetAlarm3String       "RESET_ALARM_3"     /* asynInt32 r/w */
#define resetAlarm4String       "RESET_ALARM_4"     /* asynInt32 r/w */
#define resetAlarm5String       "RESET_ALARM_5"     /* asynInt32 r/w */

#define xrayReadyString	        "STATUS_XRAY_READY"	/* asynInt32   r/o */
#define xrayOnString            "STATUS_XRAY_ON"	/* asynInt32   r/o */
#define shutterOpenString       "STATUS_SHUTTER_OPEN"	/* asynInt32   r/o */
#define doorUnlockedString      "STATUS_DOOR_UNLOCKED"	/* asynInt32   r/o */
#define doorOpenString          "STATUS_DOOR_OPEN"	/* asynInt32   r/o */

#define curErrorString          "STATUS_CUR_ERROR"	/* asynOctet r/o */
#define warning1String          "STATUS_WARNING_1"	/* asynOctet r/o */
#define warning2String          "STATUS_WARNING_2"	/* asynOctet r/o */
#define warning3String          "STATUS_WARNING_3"	/* asynOctet r/o */
#define warning4String          "STATUS_WARNING_4"	/* asynOctet r/o */
#define warning5String          "STATUS_WARNING_5"	/* asynOctet r/o */
#define alarm1String            "STATUS_ALARM_1"	/* asynOctet r/o */
#define alarm2String            "STATUS_ALARM_2"	/* asynOctet r/o */
#define alarm3String            "STATUS_ALARM_3"	/* asynOctet r/o */
#define alarm4String            "STATUS_ALARM_4"	/* asynOctet r/o */
#define alarm5String            "STATUS_ALARM_5"	/* asynOctet r/o */

/*
 * Class definition for the Rigaku class
 */
class Rigaku: public asynPortDriver {
public:
    Rigaku(const char *portName, const char *RigakuPortName);
    
    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    // virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    // virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    // virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    // value - pointer to string
    // maxChars - max num of characters to read
    //
    // virtual asynStatus readOctet(asynUser *pasynUser, char* value, size_t maxChars, 
    //    size_t* nActual, int* eomReason);
    // virtual asynStatus disconnect(asynUser *pasynUser);
    // virtual asynStatus connect(asynUser *pasynUser);
    // virtual ~Rigaku();
    // These should be private but are called from C
    virtual void pollerThread(void);

protected:
    // "float" index values
    int voltageInVal_;
    int currentInVal_;
    int voltageOutVal_;
    int currentOutVal_;
    // binary index values
    int setPowerOut_;
    int setDoorLockOut_;
    int setXrayOnOut_;
    int setShutterOpenOut_;

    int resetWarningsOut_[5];
    int resetAlarmsOut_[5];

    int statusXrayReady_;
    int statusXrayOn_;
    int statusShutterOpen_;
    int statusDoorUnlocked_;
    int statusDoorOpen_;
    // string index values
    int statusCurError_;
    int statusWarnings_[5];
    int statusAlarms_[5];

	#define FIRST_RIGAKU_PARAM voltageInVal_;
	#define LAST_RIGAKU_PARAM statusAlarms_[4];

    asynUser* pasynUserRigaku_;

private:
    char outString_[256];
    char inString_[256];
    
    asynStatus writeReadRigaku();
    asynStatus writeReadRigaku(const char *output, char *response, size_t maxResponseLen, 
        size_t *responseLen, double timeout);

    asynStatus setPower(epicsInt32 value);
    asynStatus setDoorLock(epicsInt32 value);
    asynStatus setXrayOn(epicsInt32 value);
    asynStatus setShutterOpen(epicsInt32 value);
    asynStatus resetWarning(epicsInt32 value, int i_warning);
    asynStatus resetAlarm(epicsInt32 value, int i_alarm);

	// asynStatus readControllerConfig();
	// asynStatus readControllerError();
	// asynStatus readControllerStatus();
	// asynStatus readStageConfig();
	void report(FILE *fp, int details);

    int voltageOutBuf;
    int currentOutBuf;


  double pollTime_;
	// int commStatus_;
	// float temperatureRbv_;
	// char* wrapperVersion_;
	// char* libraryVersion_;
  // epicsUInt64 controllerConfig_;
  // epicsUInt32 controllerError_;
  // epicsUInt64 controllerStatus_;
  // epicsUInt64 stageConfig_;
  
};

// #define NUM_PARAMS ((int)(&LAST_RIGAKU_PARAM - &FIRST_RIGAKU_PARAM + 1))
#define NUM_PARAMS 34


