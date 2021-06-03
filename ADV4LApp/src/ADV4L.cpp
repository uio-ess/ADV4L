/* ADV4L.cpp
 *
 * This is a driver for Video4Linux2 device
 *
 * Author: Kyrre Sjobak
 *         University of Oslo
 *
 * Created:  3rd June 2021
 *
 */

// EPICS includes
/*
*/


// Local includes
#include "ADV4L.h"

/* System includes */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include<unistd.h>

/* EPICS includes */
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>
#include <epicsEndian.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <initHooks.h>

#include <ADDriver.h>

// Driver name for asyn trace prints
static const char *driverName = "ADV4L";

ADV4L::ADV4L(const char* portName,
             const char* V4LdeviceName,
             int maxBuffers, size_t maxMemory,
             int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0, // No interfaces beyond these set in ADDriver.cpp
               0, 1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1
               priority, stackSize),
      V4LdeviceName(V4LdeviceName) {

    //Constructor constructor constructor...
}

void ADV4L::run() {
    //Grab and publish images as they come in

    while(true) {
        //Grab image!
        usleep(1000*25);
    }
}

asynStatus ADV4L::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    epicsInt32 rbv;

    const char* reasonName = "unknownReason";
    getParamName( 0, function, &reasonName );

    // Set the parameter and readback in the parameter library.
    // This may be overwritten when we read back the status at the end, but that's OK
    getIntegerParam(function, &rbv);
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value) {
            // This was a command to start acquisition
            status = this->start();
        }
        else {
            // This was a command to stop acquisition
            status = this->stop();
        }
    }
    else {
        status = asynError;
    }

    // Do callbacks so higher layers see any changes
    callParamCallbacks();

    // Report any errors
    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:writeInt32 error, status=%d function=%d %s, value=%d\n",
                  driverName, status, function, reasonName, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:writeInt32: function=%d %s, value=%d\n",
                  driverName, function, reasonName, value);
    }

    return status;
}

// Configuration command, called directly or from iocsh
extern "C" int ADV4LConfig(const char* const portName, const char* const V4LdeviceName,
                           int maxBuffers, int maxMemory, int priority, int stackSize) {
    new ADV4L(portName, V4LdeviceName,
              (maxBuffers < 0) ? 0 : maxBuffers,
              (maxMemory < 0) ? 0 : maxMemory,
              priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADV4LConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADV4LConfigArg1 = {"V4L Device Name", iocshArgString};
static const iocshArg ADV4LConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg ADV4LConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg ADV4LConfigArg4 = {"priority", iocshArgInt};
static const iocshArg ADV4LConfigArg5 = {"stackSize", iocshArgInt};

static const iocshArg* const ADV4LConfigArgs[] =
    { &ADV4LConfigArg0, &ADV4LConfigArg1, &ADV4LConfigArg2,
      &ADV4LConfigArg3, &ADV4LConfigArg4, &ADV4LConfigArg5};

static const iocshFuncDef configADV4L = {"ADV4LConfig", 6, ADV4LConfigArgs};

static void configADV4LCallFunc(const iocshArgBuf *args) {
    ADV4LConfig(args[0].sval, args[1].sval,
                      args[2].ival, args[3].ival,
                      args[4].ival, args[5].ival);
}

static void ADV4LRegister(void) {
    iocshRegister(&configADV4L, configADV4LCallFunc);
}

extern "C" {
    epicsExportRegistrar(ADV4LRegister);
}
