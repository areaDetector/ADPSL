/* PSL.cpp
 *
 * This is a driver for Photonic Sciences Ltd. CCD detectors.
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  Sept. 21, 2010
 *
 */
 
#include <stdio.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsStdlib.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>

#include "ADDriver.h"

/** Messages to/from server */
#define MAX_MESSAGE_SIZE 256
#define MAX_FILENAME_LEN 256
#define PSL_SERVER_TIMEOUT 1.0 

/** Trigger mode choices */
typedef enum {
    PSLTriggerFreeRun,
    PSLTriggerSoftware,
    PSLTriggerFalling,
    PSLTriggerRising,
    PSLTriggerPipelineSoftware,
    PSLTriggerPipelineFalling,
    PSLTriggerPipelineRising
} PSLTriggerMode_t;

#define MAX_PSL_TRIGGER_MODES 7

static const char* PSLTriggerModeStrings[] = 
    {"Software", "FreeRunning", "Hardware_Falling", "Hardware_Rising",
    "Pipeline_Software", "Pipeline_Falling", "Pipeline_Rising"};

/** Trigger mode choices */
typedef enum {
    PSLFileFormatTIFF,
    PSLFileFormatJPEG,
    PSLFileFormatBMP,
    PSLFileFormatGIF,
    PSLFileFormatPNG,
    PSLFileFormatFLF,
    PSLFileFormatICO,
    PSLFileFormatPNM,
    PSLFileFormatPCX
} PSLFileFormat_t;

#define MAX_PSL_FILE_FORMATS 9

static const char* PSLFileFormatStrings[] = 
    {"tif", "jpeg", "bmp", "gif", "png", "flf", "ico", "pnm", "pcx"};

#define PSLOffOnString        "PSL_OFF_ON"
#define PSLResetString        "PSL_RESET"

static const char *driverName = "PSL";

/** Driver for Photonic Sciencies Ltd. CCD detector; communicates with the PSL servr over a TCP/IP
  * socket.
  * The server must be started with the PSL Software Menu/Connexion. 
  */
class PSL : public ADDriver {
public:
    PSL(const char *portName, const char *PSLPort,
           int maxBuffers, size_t maxMemory,
           int priority, int stackSize);
                 
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    void report(FILE *fp, int details);
    void PSLTask();             /**< This should be private but is called from C, must be public */
    epicsEventId stopEventId;   /**< This should be private but is accessed from C, must be public */

protected:
    int PSLOffOn;
    #define FIRST_PSL_PARAM PSLOffOn
    int PSLReset;
    #define LAST_PSL_PARAM PSLReset

private:                                        
    /* These are the methods that are new to this class */
    asynStatus writeReadServer(const char *output, char *input, size_t maxChars, double timeout);
    asynStatus getConfig();
    void acquireFrame();
    asynStatus getImage();
    void saveFile();
   
    /* Our data */
    epicsEventId startEventId;
    epicsTimerId timerId;
    char toServer[MAX_MESSAGE_SIZE];
    char fromServer[MAX_MESSAGE_SIZE];
    NDArray *pData;
    asynUser *pasynUserServer;
    asynUser *pasynUserCommon;
};


#define NUM_PSL_PARAMS (&LAST_PSL_PARAM - &FIRST_PSL_PARAM + 1)

asynStatus PSL::writeReadServer(const char *output, char *input, size_t maxChars, double timeout)
{
    asynStatus status;
    size_t nwrite, nread;
    int eomReason;
    asynUser *pasynUser = this->pasynUserServer;
    const char *functionName="writeReadServer";

    // We need to connect to the device each time
    status = pasynCommonSyncIO->disconnectDevice(this->pasynUserCommon);
    status = pasynCommonSyncIO->connectDevice(this->pasynUserCommon);
    status = pasynOctetSyncIO->writeRead(pasynUser, output, strlen(output), 
                                         input, maxChars, timeout,
                                         &nwrite, &nread, &eomReason);
                                        
    if (status) asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, status=%d, sent\n%s\n",
                    driverName, functionName, status, output);

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringToServer, output);
    callParamCallbacks();

    //Do we need to delay?
    status = pasynCommonSyncIO->disconnectDevice(this->pasynUserCommon);
    epicsThreadSleep(0.1);

    
    return(status);
}


asynStatus PSL::getConfig()
{
    int sizeX, sizeY, binX, binY, imageSize;
    int top, left, right, bottom;
    int i;
    char triggerModeString[50];
    char fileFormatString[50];
    char filePath[MAX_FILENAME_LEN];
    char fileName[MAX_FILENAME_LEN];
    char fileSuffix[MAX_FILENAME_LEN];
    int fileNumber;
    double exposure;
    asynStatus status;
    static const char* functionName="getConfig";
    
    status = writeReadServer("GetExposure\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        sscanf(this->fromServer, "%lf", &exposure);
        setDoubleParam(ADAcquireTime, exposure/1000.);
    }

    status = writeReadServer("GetBinning\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        sscanf(this->fromServer, "%d;%d", &binX, &binY);
        setIntegerParam(ADBinX, binX);
        setIntegerParam(ADBinY, binY);
    }

    status = writeReadServer("GetSubArea\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        sscanf(this->fromServer, "%d;%d;%d;%d", &left, &top, &right, &bottom);
        setIntegerParam(ADMinX, left);
        setIntegerParam(ADMinY, top);
        sizeX = right-left+1;
        sizeY = bottom-top+1;
        setIntegerParam(ADSizeX, sizeX);
        setIntegerParam(ADSizeY, sizeY);
        imageSize = sizeX * sizeY * sizeof(epicsInt16);
        setIntegerParam(NDArraySize, imageSize);
    }

    status = writeReadServer("GetTriggerModes\n", triggerModeString, sizeof(triggerModeString), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        for (i=0; i<MAX_PSL_TRIGGER_MODES; i++) {
            if (strcmp(triggerModeString, PSLTriggerModeStrings[i]) == 0) {
                setIntegerParam(ADTriggerMode, i);
                break;
            }
        }
        if (i == MAX_PSL_TRIGGER_MODES) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error, unknown trigger mode string = %s\n",
                driverName, functionName, triggerModeString);
            //return(asynError);
        }
    }

    status = writeReadServer("GetFileDirectory\n", filePath, sizeof(filePath), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        setStringParam(NDFilePath, filePath);
    }

    status = writeReadServer("GetFilePreffix\n", fileName, sizeof(fileName), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        setStringParam(NDFileName, fileName);
    }

    status = writeReadServer("GetFileSuffix\n", fileSuffix, sizeof(fileSuffix), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        //setStringParam(NDFileName, fileName);
    }

    status = writeReadServer("GetFileRefNumber\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        sscanf(this->fromServer, "%d", &fileNumber);
        setIntegerParam(NDFileNumber, fileNumber);
    }

    status = writeReadServer("GetFileFormat\n", fileFormatString, sizeof(fileFormatString), PSL_SERVER_TIMEOUT);
    if (status == asynSuccess) {
        for (i=0; i<MAX_PSL_FILE_FORMATS; i++) {
            if (epicsStrCaseCmp(fileFormatString, PSLFileFormatStrings[i]) == 0) {
                setIntegerParam(NDFileFormat, i);
                break;
            }
        }
        if (i == MAX_PSL_FILE_FORMATS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error, unknown file format string = %s\n",
                driverName, functionName, fileFormatString);
            //return(asynError);
        }
    }

    // How do we get the maximum size of the images?
    //setIntegerParam(ADMaxSizeX, sizeX*binX);
    //setIntegerParam(ADMaxSizeY, sizeY*binY);
    callParamCallbacks();
    return(asynSuccess);
}

void PSL::saveFile()
{
    asynStatus status;
    //static const char *functionName = "saveFile";
    
    status = writeReadServer("Save\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
}

void PSL::acquireFrame()
{
    asynStatus status;
    double acquireTime, timeout;

    getDoubleParam(ADAcquireTime, &acquireTime);
    timeout = acquireTime + 10.;
    status = writeReadServer("Snap\n", this->fromServer, sizeof(this->fromServer), timeout);
}

asynStatus PSL::getImage()
{
    int dims[2];
    char modeString[10];
    int imageCounter;
    int dataLen;
    NDDataType_t dataType;
    NDArray *pImage;
    asynStatus status;
    const char *functionName = "getImage";

    status = writeReadServer("GetMode\n", modeString, sizeof(modeString), PSL_SERVER_TIMEOUT);
    if (status) return(status);
    if (strcmp(modeString, "L") == 0) {
        dataType = NDUInt32;
    }
    else if (strcmp(modeString, "I;16") == 0) {
        dataType = NDUInt16;
    }
    else if (strcmp(modeString, "I") == 0) {
        dataType = NDUInt8;
    }
    else if (strcmp(modeString, "F") == 0) {
        dataType = NDFloat32;
    }
    else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unknown mode=%s\n", modeString);
        return(asynError);
    }

    status = writeReadServer("GetImage\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    if (status) return(status);
    sscanf(this->fromServer, "%d;%d;%d", &dims[0], &dims[1], &dataLen);

    getIntegerParam(NDArrayCounter, &imageCounter);
    pImage = this->pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
    //status = readServer((char *)pImage->pData, dataLen, PSL_SERVER_TIMEOUT);

    /* Put the frame number and time stamp into the buffer */
    pImage->uniqueId = imageCounter;

    /* Get any attributes that have been defined for this driver */        
    this->getAttributes(pImage->pAttributeList);

    /* Call the NDArray callback */
    /* Must release the lock here, or we can get into a deadlock, because we can
     * block on the plugin lock, and the plugin can be calling us */
    this->unlock();
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
         "%s:%s: calling NDArray callback\n", driverName, functionName);
    doCallbacksGenericPointer(pImage, NDArrayData, 0);
    this->lock();

    /* Free the image buffer */
    pImage->release();
    return(asynSuccess);
}


static void PSLTaskC(void *drvPvt)
{
    PSL *pPvt = (PSL *)drvPvt;
    
    pPvt->PSLTask();
}

/** This thread controls acquisition, reads TIFF files to get the image data, and
 * does the callbacks to send it to higher layers */
void PSL::PSLTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int acquire;
    double acquirePeriod;
    int autoSave;
    int arrayCallbacks;
    int shutterMode, useShutter;
    double elapsedTime, delayTime;
    epicsTimeStamp acqStartTime, acqEndTime;
    const char *functionName = "PSLTask";

    this->lock();

    /* Loop forever */
    while (1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);
        
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            setStringParam(ADStatusMessage, "Waiting for acquire command");
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
            status = epicsEventWait(this->startEventId);
            this->lock();
            getIntegerParam(ADAcquire, &acquire);
            setIntegerParam(ADNumImagesCounter, 0);
            callParamCallbacks();
        }
        
        /* Get current values of some parameters */
        getIntegerParam(NDAutoSave, &autoSave);
        getIntegerParam(ADShutterMode, &shutterMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        if (shutterMode == ADShutterModeNone) useShutter=0; else useShutter=1;
        
        epicsTimeGetCurrent(&acqStartTime);
        acquireFrame();
        if (autoSave) saveFile();
        
        getIntegerParam(NDArrayCounter, &imageCounter);
        imageCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        numImagesCounter++;
        setIntegerParam(ADNumImagesCounter, numImagesCounter);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

        /* If arrayCallbacks is set then read the data, do callbacks */
        if (arrayCallbacks) {
            getImage();
        }

        getIntegerParam(ADImageMode, &imageMode);
        if (imageMode == ADImageMultiple) {
            getIntegerParam(ADNumImages, &numImages);
            if (numImagesCounter >= numImages) setIntegerParam(ADAcquire, 0);
        }    
        if (imageMode == ADImageSingle) setIntegerParam(ADAcquire, 0);
        getIntegerParam(ADAcquire, &acquire);
        if (acquire) {
            /* We are in continuous or multiple mode.
             * Sleep until the acquire period expires or acquire is set to stop */
            epicsTimeGetCurrent(&acqEndTime);
            elapsedTime = epicsTimeDiffInSeconds(&acqEndTime, &acqStartTime);
            getDoubleParam(ADAcquirePeriod, &acquirePeriod);
            delayTime = acquirePeriod - elapsedTime;
            if (delayTime > 0.) {
                setIntegerParam(ADStatus, ADStatusWaiting);
                callParamCallbacks();
                this->unlock();
                status = epicsEventWaitWithTimeout(this->stopEventId, delayTime);
                this->lock();
            }
        }

        /* Call the callbacks to update any changes */
        callParamCallbacks();
    }
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PSL::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int binX, binY, minX, minY, sizeX, sizeY;
    char tmpString[MAX_FILENAME_LEN];
    char fileTemplate[MAX_FILENAME_LEN];
    asynStatus status = asynSuccess;
    int acquiring;
    const char *functionName = "writeInt32";

    /* Get the current acquire status */
    getIntegerParam(ADAcquire, &acquiring);
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value && !acquiring) {
            /* Send an event to wake up the PSL task.  */
            epicsEventSignal(this->startEventId);
        } 
        if (!value && acquiring) {
            /* This was a command to stop acquisition */
            status = writeReadServer("AbortSnap\n", this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
            epicsEventSignal(this->stopEventId);
        }
    } else if ((function == ADBinX) ||
               (function == ADBinY)) {
        /* Set binning */
        getIntegerParam(ADBinX, &binX);
        getIntegerParam(ADBinY, &binY);
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetBinning;%d;%d\n", binX, binY);
        writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
        getConfig();
    } else if ((function == ADMinX) ||
               (function == ADMinY) ||
               (function == ADSizeX) ||
               (function == ADSizeY)) {
        getIntegerParam(ADMinX, &minX);
        getIntegerParam(ADMinY, &minY);
        getIntegerParam(ADSizeX, &sizeX);
        getIntegerParam(ADSizeY, &sizeY);
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetSubArea;%d;%d;%d;%d\n", 
                      minX, minY, minX+sizeX-1, minY+sizeY-1);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
        getConfig();
    } else if (function == ADTriggerMode) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetTriggerModes;%s\n", 
                      PSLTriggerModeStrings[value]);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
        getConfig();
    } else if (function == NDFileFormat) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetFileFormat;%s\n",
                      PSLFileFormatStrings[value]);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
        getConfig();
    } else if (function == NDWriteFile) {
        saveFile();
        getConfig();
    } else if (function == NDFileNumber) {
        getStringParam(NDFileTemplate, sizeof(fileTemplate), fileTemplate);
        epicsSnprintf(tmpString, sizeof(tmpString), fileTemplate, "", "", value);
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetFileRefNumber;%s\n", tmpString);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PSL_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
        
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus PSL::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeFloat64";

    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetExposure;%f", value*1000.);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
        getConfig();
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PSL_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }
        
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including PilatusBadPixelFile, ADFilePath, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus PSL::writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);

    if (function == NDFilePath) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetFileDirectory;%s\n", value);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    } else if (function == NDFileName) {
        epicsSnprintf(this->toServer, sizeof(this->toServer), "SetFilePreffix;%s\n", value);
        status = writeReadServer(this->toServer, this->fromServer, sizeof(this->fromServer), PSL_SERVER_TIMEOUT);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PSL_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
    }
    
     /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, value=%s", 
                  driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%s\n", 
              driverName, functionName, function, value);
    *nActual = nChars;
    return status;
}



/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void PSL::report(FILE *fp, int details)
{
    fprintf(fp, "PSL detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

extern "C" int PSLConfig(const char *portName, const char *serverPort, 
                            int maxBuffers, size_t maxMemory,
                            int priority, int stackSize)
{
    new PSL(portName, serverPort, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/** Constructor for PSL driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] serverPort The name of the asyn port driver previously created with drvAsynIPPortConfigure
  *            connected to the PSL_server program.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
PSL::PSL(const char *portName, const char *serverPort,
                                int maxBuffers, size_t maxMemory,
                                int priority, int stackSize)

    : ADDriver(portName, 1, NUM_PSL_PARAMS, maxBuffers, maxMemory,
               0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      pData(NULL)

{
    int status = asynSuccess;
    const char *functionName = "PSL";

    createParam(PSLOffOnString,       asynParamInt32, &PSLOffOn);
    createParam(PSLResetString,       asynParamInt32, &PSLReset);
   
    /* Create the epicsEvents for signaling to the PSL task when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        printf("%s:%s epicsEventCreate failure for start event\n", 
            driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        printf("%s:%s epicsEventCreate failure for stop event\n", 
            driverName, functionName);
        return;
    }

    /* Connect to server */
    status = pasynOctetSyncIO->connect(serverPort, 0, &this->pasynUserServer, NULL);
    status = pasynCommonSyncIO->connect(serverPort, 0, &this->pasynUserCommon, NULL);
   
    /* Compute the sensor size by reading the image size and the binning */
    status = getConfig();

    /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "PSL");
    status |= setStringParam (ADModel, "CCD");
    status |= setIntegerParam(NDDataType,  NDInt16);
    status |= setIntegerParam(ADImageMode, ADImageSingle);
    status |= setIntegerParam(ADTriggerMode, PSLTriggerSoftware);
    status |= setDoubleParam (ADAcquirePeriod, 0.);
    status |= setIntegerParam(ADNumImages, 1);
       
    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }
    
    /* Create the thread that collects the data */
    status = (epicsThreadCreate("PSLTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)PSLTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for data collection task\n", 
            driverName, functionName);
        return;
    }
}

/* Code for iocsh registration */
static const iocshArg PSLConfigArg0 = {"Port name", iocshArgString};
static const iocshArg PSLConfigArg1 = {"server port name", iocshArgString};
static const iocshArg PSLConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg PSLConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg PSLConfigArg4 = {"priority", iocshArgInt};
static const iocshArg PSLConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const PSLConfigArgs[] =  {&PSLConfigArg0,
                                                     &PSLConfigArg1,
                                                     &PSLConfigArg2,
                                                     &PSLConfigArg3,
                                                     &PSLConfigArg4,
                                                     &PSLConfigArg5};
static const iocshFuncDef configPSL = {"PSLConfig", 6, PSLConfigArgs};
static void configPSLCallFunc(const iocshArgBuf *args)
{
    PSLConfig(args[0].sval, args[1].sval, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival);
}


static void PSLRegister(void)
{
    iocshRegister(&configPSL, configPSLCallFunc);
}

extern "C" {
epicsExportRegistrar(PSLRegister);
}
