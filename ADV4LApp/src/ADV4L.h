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

class ADV4L : public ADDriver, epicsThreadRunable {
 public:
    ADV4L(const char* portName,
          const char* V4LdeviceName,
          int maxBuffers, size_t maxMemory,
          int priority, int stackSize);

    //From epicsThreadRunnable
    // This method grabs images and publishes them
    // as they become available
    virtual void run();

    //From ADDriver
    // Recieves data from caput etc.
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    //Variables available from outside class
    const char* const V4LdeviceName; // "/dev/video0" etc.

 protected:
    //Start and stop acquisition of images
    virtual asynStatus start() { return asynSuccess; };
    virtual asynStatus stop() { return asynSuccess; };

 private:
    epicsThread pollingLoop;
};


#endif
