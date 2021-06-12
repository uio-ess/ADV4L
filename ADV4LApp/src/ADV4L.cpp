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

// System includes -- Maybe too many?
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>  //usleep()

#include <stdexcept> //std::runtime_error

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
ADV4L::ADV4L(const char* portName_,
             const char* V4L_deviceName_,
             int maxBuffers, size_t maxMemory,
             int priority, int stackSize)
    : ADDriver(portName_, 1, 0, maxBuffers, maxMemory,
               0, 0, // No interfaces beyond these set in ADDriver.cpp
               0, 1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1
               priority, stackSize),
      pollingLoop(*this, "V4LPoll", stackSize, epicsThreadPriorityHigh) {

    const char* const functionName = "ADV4L";

    //The argument V4L_deviceName_ points to an ephermal char-array,
    // we need to make a copy of it.
    strncpy(this->V4L_deviceName, V4L_deviceName_, sizeof(this->V4L_deviceName)-1);
    //Make sure it's zero-padded; it might crash later but that's OK.
    this->V4L_deviceName[sizeof(this->V4L_deviceName)-1] = '\0';

    //Cannot use asynPrint here
    printf("%s:%s: portName='%s', V4L_deviceName='%s' \n",
           driverName, functionName, portName, this->V4L_deviceName);
    printf("this=%p\n", (void*)this);

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

    setIntegerParam(ADAcquire, 0);

    //Safe defaults
    setIntegerParam(ADBinX, 1);
    setIntegerParam(ADBinY, 1);
    setIntegerParam(NDColorMode, NDColorModeRGB1);
    setIntegerParam(NDDataType,  NDUInt8);

    //Safe nonsense
    //setIntegerParam(ADTemperature, -1000);

    //Flags and semaphores for active acquisition
    // To start/stop the acquisition thread, first flip the _running flag,
    // and then either (a) allocate buffers etc. then set the _started semaphore to kick of the acquisition thread,
    // or (b) wait for the acquisiton thread to see that the running flag is false and issue the _stopped semaphore,
    // then tear down.
    V4L_running = false;
    V4L_started = new epicsEvent(epicsEventEmpty);
    V4L_stopped = new epicsEvent(epicsEventEmpty);

    // Register the pollingLoop to start after iocInit
    initHookRegister(setIocRunningFlag);
    this->pollingLoop.start();
}

ADV4L::~ADV4L() {
    delete V4L_started;
    delete V4L_stopped;
}

asynStatus ADV4L::start() {
    const char* const functionName = "start";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s\n", driverName, functionName);

    if(V4L_running) {
        //Already running
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s Cannot start while already running\n", driverName, functionName);
        return asynError;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Trying to open V4L2 device '%s'\n",
              driverName, functionName, this->V4L_deviceName);

    //TODO: Cleanup on failure


    //Configure device
    V4L_fd = v4l2_open(V4L_deviceName, O_RDWR | O_NONBLOCK, 0);
    if (V4L_fd < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: Cannot open V4L2 device '%s'\n",
                  driverName, functionName, V4L_deviceName);
        return asynError;
    }

    CLEAR(V4L_fmt);
    //TODO: This should really be coming from EPICS
    V4L_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L_fmt.fmt.pix.width       = 640;
    V4L_fmt.fmt.pix.height      = 480;
    V4L_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; //Corresponds to NDColorModeRGB1 -- color,x,y
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
    size_t bufferDims[3] = {3, V4L_fmt.fmt.pix.width, V4L_fmt.fmt.pix.height};

    //Map buffers
    CLEAR(V4L_req);
    V4L_req.count  = nbuff;
    V4L_req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    V4L_req.memory = V4L2_MEMORY_MMAP;
    xioctl(V4L_fd, VIDIOC_REQBUFS, &V4L_req);

    V4L_buffers = (V4L_buffer*) calloc(nbuff, sizeof(*V4L_buffers));
    struct v4l2_buffer buf;

    for (size_t n_buffers = 0; n_buffers < nbuff; ++n_buffers) {
        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        xioctl(V4L_fd, VIDIOC_QUERYBUF, &buf);

        V4L_buffers[n_buffers].length = buf.length;
        V4L_buffers[n_buffers].start  = v4l2_mmap(NULL, buf.length,
                                                  PROT_READ | PROT_WRITE, MAP_SHARED,
                                                  V4L_fd, buf.m.offset);

        if (MAP_FAILED == V4L_buffers[n_buffers].start) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: MMAP failed.\n",
                      driverName, functionName);
            return asynError;
        }

        //Create the corresponding NDArray
        pRaw[n_buffers] = this->pNDArrayPool->alloc(3,bufferDims, NDUInt8, buf.length, V4L_buffers[n_buffers].start);
        if (pRaw[n_buffers] == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Error allocating NDArray buffer",
                      driverName, functionName);
            return asynError;
        }

    }

    for (size_t i = 0; i < nbuff; ++i) {
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        xioctl(V4L_fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(V4L_fd, VIDIOC_STREAMON, &type);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s complete\n", driverName, functionName);

    //Go!
    V4L_running = true;
    V4L_started->signal();

    return asynSuccess;
}
asynStatus ADV4L::stop() {
    const char* const functionName = "stop";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s\n", driverName, functionName);

    if (V4L_running == false) {
        //Not running so cannot stop
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s Cannot stop while not running\n", driverName, functionName);

        return asynError;
    }

    V4L_running = false;
    //The grabber thread wants to lock up again
    // before stopping on the 'running' semaphore.
    this->unlock();
    //Wait for the graber to see V4L_running and halt
    V4L_stopped->wait();
    //OK, we have stopped successfully
    this->lock();

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(V4L_fd, VIDIOC_STREAMOFF, &type);

    for (size_t i = 0; i < nbuff; ++i) {
        //pRaw[i]->release();
        v4l2_munmap(V4L_buffers[i].start, V4L_buffers[i].length);
    }
    free(V4L_buffers);

    v4l2_close(V4L_fd);
    V4L_fd = -1;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s complete\n", driverName, functionName);

    return asynSuccess;
}

void ADV4L::run() {
    //Grab and publish images as they come in

    const char* const functionName = "run";

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s\n",
              driverName, functionName);

    int                r;
    fd_set             fds;
    struct timeval     tv;
    struct v4l2_buffer buf;

    epicsTimeStamp now;
    this->lock();

    while(true) {
        if (!V4L_running) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
                      "%s:%s: Waiting for start...\n", driverName, functionName);

            this->unlock();
            V4L_started->wait();
            this->lock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
                      "%s:%s: Got start!...\n", driverName, functionName);
        }

        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: Grabbing...\n", driverName, functionName);

        this->unlock();
        do {
            FD_ZERO(&fds);
            FD_SET(V4L_fd, &fds);

            /* Timeout. */
            tv.tv_sec  = 2;
            tv.tv_usec = 0;

            r = select(V4L_fd + 1, &fds, NULL, NULL, &tv);
        } while ((r == -1 && (errno = EINTR)));

        this->lock();
        if (r == -1) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: SELECT failed.\n",
                      driverName, functionName);
            continue;
        }
        if(!V4L_running) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "aborted!\n");
            V4L_stopped->signal();
            continue;
        }
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "catch!\n");

        int arrayCallbacks;
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        int imageCounter;
        getIntegerParam(NDArrayCounter, &imageCounter);

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        xioctl(V4L_fd, VIDIOC_DQBUF, &buf);

        epicsTimeGetCurrent(&now);
        //pRaw[buf.index]->timeStamp = now;
        updateTimeStamp(&(pRaw[buf.index]->epicsTS));

        this->pArrays[0] = pRaw[buf.index];

        //Attempt at using a new buffer every time
        //size_t bufferDims[3] = {3, V4L_fmt.fmt.pix.width, V4L_fmt.fmt.pix.height};
        //this->pArrays[0] = pNDArrayPool->alloc(3,bufferDims,NDUInt8,buf.length,NULL);
        //memcpy(this->pArrays[0]->pData, V4L_buffers[buf.index].start, buf.length);

        //TODO:
        //Set pRaw->uniqueId = imageCounter and pRaw=timeStamp
        // There may be other attributes to be set as well...

        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: Data example: %02X:%02X:%02X\n",
                  driverName, functionName,
                  ((uint8_t*)(V4L_buffers[buf.index].start))[0],
                  ((uint8_t*)(V4L_buffers[buf.index].start))[1],
                  ((uint8_t*)(V4L_buffers[buf.index].start))[2]);

        NDArrayInfo_t arrayInfo;
        int status = pRaw[buf.index]->getInfo(&arrayInfo);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: ArrayInfo: %d %lu %d %lu %d %d %d %lu %lu %lu\n",
                  driverName, functionName,
                  status, arrayInfo.nElements,
                  arrayInfo.bytesPerElement, arrayInfo.totalBytes,
                  arrayInfo.xDim, arrayInfo.yDim, arrayInfo.colorDim,
                  arrayInfo.xSize, arrayInfo.ySize, arrayInfo.colorSize);

        if(arrayCallbacks) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: Calling imageData callback\n",
                      driverName, functionName);

            //doCallbacksGenericPointer will always return asynSuccess
            doCallbacksGenericPointer(pRaw[buf.index], NDArrayData,0);
            //doCallbacksGenericPointer(this->pArrays[0],NDArrayData,0);
        }

        callParamCallbacks();

        //this->pArrays[0]->release();
        xioctl(V4L_fd, VIDIOC_QBUF, &buf);
    }
}

asynStatus ADV4L::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    const char* const functionName = "writeInt32";
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s device='%s'\n", driverName, functionName, this->V4L_deviceName);

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
