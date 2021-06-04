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

/* System includes */ //Maybe too many
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include<unistd.h> //usleep()

//V4L2 includes
#include <linux/videodev2.h>
#include <libv4l2.h>

#include <fcntl.h>
#include <sys/mman.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static void xioctl(int fh, int request, void *arg) {
    int r;

    do {
        r = v4l2_ioctl(fh, request, arg);
    } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

    if (r == -1) {
        fprintf(stderr, "error %d, %s\\n", errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

/* EPICS includes */ //Maybe too many
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

// Flag to say IOC is running
static int iocRunning = 0;
// Init hook that sets iocRunning flag
static void setIocRunningFlag(initHookState state) {
    switch(state) {
        case initHookAfterIocRunning:
            iocRunning = 1;
            break;
        default:
            break;
    }
}


// Constructor
ADV4L::ADV4L(const char* portName,
             const char* V4L_deviceName,
             int maxBuffers, size_t maxMemory,
             int priority, int stackSize)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0, // No interfaces beyond these set in ADDriver.cpp
               0, 1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1
               priority, stackSize),
      V4L_deviceName(V4L_deviceName),
      pollingLoop(*this, "V4LPoll", stackSize, epicsThreadPriorityHigh){

    const char* const functionName = "ADV4L";
    /*
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: *** constructor *** -- '%s' \n", driverName, functionName, portName);
    */
    //Create some parameters specific to ADV4L
    createParam("ADV4L_DEVICENAME", asynParamOctet, &prop_V4L_deviceName);

    //Set some default values
    setStringParam(prop_V4L_deviceName, V4L_deviceName);

    // TODO: Put sensible things here; for now it is just copied from aravisGigE
    // setStringParam(NDDriverVersion, DRIVER_VERSION);
    // setStringParam(ADSDKVersion, ARAVIS_VERSION);
    setIntegerParam(ADReverseX, 0);
    setIntegerParam(ADReverseY, 0);
    setIntegerParam(ADImageMode, ADImageContinuous);
    setIntegerParam(ADNumImages, 100);

    // Register the pollingLoop to start after iocInit
    initHookRegister(setIocRunningFlag);
    this->pollingLoop.start();
}

asynStatus ADV4L::start() {
    const char* const functionName = "start";

    //TODO: Cleanup on failure


    //Configure device
    V4L_fd = v4l2_open(V4L_deviceName, O_RDWR | O_NONBLOCK, 0);
    if (V4L_fd < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Cannot open V4L2 device\n", driverName, functionName);
        return asynError;
    }

    CLEAR(V4L_fmt);
    V4L_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L_fmt.fmt.pix.width       = 640;
    V4L_fmt.fmt.pix.height      = 480;
    V4L_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    V4L_fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    xioctl(V4L_fd, VIDIOC_S_FMT, &V4L_fmt);
    if (V4L_fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Libv4l didn't accept RGB24 format. Can't proceed\n",
                  driverName, functionName);
        return asynError;
    }
    if ((V4L_fmt.fmt.pix.width != 640) || (V4L_fmt.fmt.pix.height != 480)) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Warning: driver is sending image at %dx%d\n",
                  driverName, functionName, V4L_fmt.fmt.pix.width, V4L_fmt.fmt.pix.height);
    }

    setIntegerParam(ADSizeX, V4L_fmt.fmt.pix.width);
    setIntegerParam(ADSizeY, V4L_fmt.fmt.pix.height);
    //TODO: Set the image type parameter in EPICS

    //Map buffers
    CLEAR(V4L_req);
    V4L_req.count = 2;
    V4L_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L_req.memory = V4L2_MEMORY_MMAP;
    xioctl(V4L_fd, VIDIOC_REQBUFS, &V4L_req);

    V4L_buffers = (V4L_buffer*) calloc(V4L_req.count, sizeof(*V4L_buffers));
    struct v4l2_buffer buf;
    for (size_t n_buffers = 0; n_buffers < V4L_req.count; ++n_buffers) {
        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        xioctl(V4L_fd, VIDIOC_QUERYBUF, &buf);

        V4L_buffers[n_buffers].length = buf.length;
        V4L_buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
                                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                                 V4L_fd, buf.m.offset);

        if (MAP_FAILED == V4L_buffers[n_buffers].start) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: MMAP failed.\n",
                      driverName, functionName);
            return asynError;
        }
    }

    V4L_run = true;
    return asynSuccess;
}
asynStatus ADV4L::stop() {
    return asynSuccess;
}

void ADV4L::run() {
    //Grab and publish images as they come in

    const char* const functionName = "run";

    while(true) {
        //Grab image!
        if (V4L_run) {

        }
        else {
            usleep(1000*1000);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: Grab!\n", driverName, functionName);
        }
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

        if (status != asynSuccess) {
            setIntegerParam(ADAcquire,0);
        }
    }
    else if (function == ADBinX || function == ADBinY ||
             function == ADMinX || function == ADMinY || function == ADSizeX || function == ADSizeY ||
             function == NDDataType || function == NDColorMode) {

        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:writeInt32 [FUNCTION IS TODO -- GEOM], status=%d function=%d %s, value=%d\n",
                  driverName, status, function, reasonName, value);

        status = asynSuccess;
    }
    else if (function == ADNumExposures) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:writeInt32 [FUNCTION IS TODO -- EXPOSURES], status=%d function=%d %s, value=%d\n",
                  driverName, status, function, reasonName, value);
    }
    else if (function < ADV4L_FIRSTPARAM) {
        // Base class parameter
        status = ADDriver::writeInt32(pasynUser, value);
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
