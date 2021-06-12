/* ADV4L.h
 *
 * This is a driver for a Video4Linux2 device
 *
 * Author: Kyrre Sjobak
 *         University of Oslo
 *
 * Created:  3rd June 2021
 *
 */


#ifndef ADV4L_H
#define ADV4L_H

#include <ADDriver.h>
#include <epicsThread.h>
#include <epicsEvent.h>

#include <linux/videodev2.h>
#include <libv4l2.h>

struct V4L_buffer {
    void*  start;
    size_t length;
};

class ADV4L : public ADDriver, epicsThreadRunable {
 public:
    ADV4L(const char* portName_,
          const char* V4L_deviceName_,
          int maxBuffers, size_t maxMemory,
          int priority, int stackSize);

    ~ADV4L();

    //From epicsThreadRunnable
    // This method grabs images and publishes them
    // as they become available
    virtual void run();

    //From ADDriver
    // Recieves data from caput etc.
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    //Variables available from outside the class

 protected:
    //Start and stop acquisition of images
    virtual asynStatus start();
    virtual asynStatus stop();

    // Variables //

    char V4L_deviceName[128]; // "/dev/video0" etc.

    //Property handles
    int prop_V4L_deviceName;
    #define ADV4L_FIRSTPARAM prop_V4L_deviceName

    //V4L buffers etc.
    epicsEvent*                V4L_started;
    epicsEvent*                V4L_stopped;
    bool                       V4L_running = false;

    int                        V4L_fd = -1;
    struct v4l2_format         V4L_fmt;
    struct v4l2_requestbuffers V4L_req;

    static const int           nbuff = 2;
    struct V4L_buffer*         V4L_buffers;
    NDArray*                   pRaw[nbuff];

 private:
    epicsThread pollingLoop;
};


#endif
