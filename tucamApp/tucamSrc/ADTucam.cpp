/**
 * ADTucam.cpp
 *
 * Main source file for the ADTucam EPICS areaDetector driver.
 *
 * Author(s): Jakub Wlodek, Thomas Hopkins
 *
 * Copyright (c): Brookhaven National Laboratory 2025
 */
#include <algorithm>
#include <cstdio>

// SDK Handler
#include "ADTucam.h"
#ifndef UNIT_TESTING
#include "TUCAMSDKHandler.h"
#endif

#define DRIVER_VERSION 1
#define DRIVER_REVISION 0
#define DRIVER_MODIFICATION 0

// Error message formatters
#define ERR(msg)                                                         \
  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define ERR_ARGS(fmt, ...)                                                    \
  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: " fmt "\n", driverName, \
            functionName, __VA_ARGS__)

// Warning message formatters
#define WARN(msg)                                                          \
  asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define WARN_ARGS(fmt, ...)                                         \
  asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: " fmt "\n", \
            driverName, functionName, __VA_ARGS__)

// Info message formatters
#define INFO(msg)                                                       \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define INFO_ARGS(fmt, ...)                                                  \
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, \
            functionName, __VA_ARGS__)

// Debug message formatters
#define DEBUG(msg)                                                          \
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: %s\n", driverName, \
            functionName, msg)

#define DEBUG_ARGS(fmt, ...)                                         \
  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: " fmt "\n", \
            driverName, functionName, __VA_ARGS__)

#ifndef UNIT_TESTING
/**
 * @brief Function that instatiates a driver object. Called from IOC shell
 *
 */
extern "C" int ADTucamConfig(const char *portName, int cameraId) {
  new ADTucam(portName, cameraId);
  return (asynSuccess);
}
#endif

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create temperature
 * monitor thread
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void monitorTemperatureThreadC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  pPvt->monitorTemperatureThread();
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create acquisition
 * thread
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void acquisitionThreadC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  pPvt->acquisitionThread();
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create timeout
 * thread
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void frameTimeoutThreadC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  pPvt->frameTimeoutThread();
}

/**
 * @brief Callback function called when exit is ran from IOC shell, deletes
 * driver object instance
 * @param drvPvt Pointer to instance of ADTucam driver object
 */
static void exitCallbackC(void *drvPvt) {
  ADTucam *pPvt = (ADTucam *)drvPvt;  // NOLINT(readability/casting)
  delete pPvt;
}

/**
 * Formats currently supported by the camera SDK.
 * DO NOT CHANGE THE ORDER OF THESE VALUES.
 */
static const int frameFormats[3] = {TUFRM_FMT_RAW, TUFRM_FMT_USUAl,
                                    TUFRM_FMT_RGB888};

static const char *driverName = "ADTucam";
static int TUCAMInitialized = 0;

#ifndef UNIT_TESTING
/**
 * @brief Main constructor for ADTucam driver
 *
 * @param portName Unique asyn port name
 * @param cameraId ID of the camera used to connect
 */
ADTucam::ADTucam(const char *portName, int cameraId)
    : ADDriver(portName, 1, NUM_TUCAM_PARAMS, 0, 0, 0, 0, 0, 1, 0, 0),
      cameraId_(cameraId),
      pRaw_(NULL),
      triggerOutSupport_(0),
      sdkHandler_(new TUCAMSDKHandler()),
      initialized_(false) {
  this->cleanupHandler_ = true;
  this->setup();
}
#endif

/**
 * @brief Constructor for ADTucam driver with dependency injection
 *
 * @param portName Unique asyn port name
 * @param cameraId ID of the camera used to connect
 * @param sdkHandler Pointer to SDK handler (for testing/mocking)
 */
ADTucam::ADTucam(const char *portName, int cameraId, ICameraSDK *sdkHandler)
    : ADDriver(portName, 1, NUM_TUCAM_PARAMS, 0, 0, 0, 0, 0, 1, 0, 0),
      cameraId_(cameraId),
      pRaw_(NULL),
      triggerOutSupport_(0),
      sdkHandler_(sdkHandler),
      initialized_(false) {
  this->cleanupHandler_ = false;
  this->setup();
}

void ADTucam::createParamLibrary() {
  // Initialize new parameters in parameter library
  createParam(ADTucam_CaptureString, asynParamInt32, &ADTucam_Capture);
  createParam(ADTucam_TemperatureSetpointString, asynParamFloat64,
              &ADTucam_TemperatureSetpoint);
  createParam(ADTucam_TemperatureString, asynParamFloat64,
              &ADTucam_Temperature);
  createParam(ADTucam_TemperatureEmergencySignalString, asynParamInt32,
              &ADTucam_TemperatureEmergencySignal);
  createParam(ADTucam_AutoTECThresholdString, asynParamFloat64,
              &ADTucam_AutoTECThreshold);
  createParam(ADTucam_AutoTECString, asynParamInt32, &ADTucam_AutoTEC);
  createParam(ADTucam_TECStatusString, asynParamInt32, &ADTucam_TECStatus);
  createParam(ADTucam_GainModeString, asynParamInt32, &ADTucam_GainMode);
  createParam(ADTucam_RetryOnTimeoutString, asynParamInt32,
              &ADTucam_RetryOnTimeout);
  createParam(ADTucam_NumRetriesString, asynParamInt32, &ADTucam_NumRetries);
  createParam(ADTucam_BinModeString, asynParamInt32, &ADTucam_BinMode);
  createParam(ADTucam_BusString, asynParamOctet, &ADTucam_Bus);
  createParam(ADTucam_ProductIDString, asynParamFloat64, &ADTucam_ProductID);
  createParam(ADTucam_TransferRateString, asynParamFloat64,
              &ADTucam_TransferRate);
  createParam(ADTucam_FrameSpeedString, asynParamInt32, &ADTucam_FrameSpeed);
  createParam(ADTucam_BitDepthString, asynParamInt32, &ADTucam_BitDepth);
  createParam(ADTucam_FanGearString, asynParamInt32, &ADTucam_FanGear);
  createParam(ADTucam_ImageModeString, asynParamInt32, &ADTucam_ImageMode);
  createParam(ADTucam_AutoExposureString, asynParamInt32,
              &ADTucam_AutoExposure);
  createParam(ADTucam_AutoLevelsString, asynParamInt32, &ADTucam_AutoLevels);
  createParam(ADTucam_HistogramString, asynParamInt32, &ADTucam_Histogram);
  createParam(ADTucam_EnhanceString, asynParamInt32, &ADTucam_Enhance);
  createParam(ADTucam_DefectCorrString, asynParamInt32, &ADTucam_DefectCorr);
  createParam(ADTucam_DenoiseString, asynParamInt32, &ADTucam_Denoise);
  createParam(ADTucam_FlatCorrString, asynParamInt32, &ADTucam_FlatCorr);
  createParam(ADTucam_DynRgeCorrString, asynParamInt32, &ADTucam_DynRgeCorr);
  createParam(ADTucam_FrameFormatString, asynParamInt32, &ADTucam_FrameFormat);
  createParam(ADTucam_BrightnessString, asynParamFloat64, &ADTucam_Brightness);
  createParam(ADTucam_BlackLevelString, asynParamFloat64, &ADTucam_BlackLevel);
  createParam(ADTucam_SharpnessString, asynParamFloat64, &ADTucam_Sharpness);
  createParam(ADTucam_NoiseLevelString, asynParamFloat64, &ADTucam_NoiseLevel);
  createParam(ADTucam_HDRKString, asynParamFloat64, &ADTucam_HDRK);
  createParam(ADTucam_GammaString, asynParamFloat64, &ADTucam_Gamma);
  createParam(ADTucam_ContrastString, asynParamFloat64, &ADTucam_Contrast);
  createParam(ADTucam_LeftLevelString, asynParamFloat64, &ADTucam_LeftLevel);
  createParam(ADTucam_RightLevelString, asynParamFloat64, &ADTucam_RightLevel);
  createParam(ADTucam_TriggerEdgeString, asynParamInt32, &ADTucam_TriggerEdge);
  createParam(ADTucam_TriggerExposureString, asynParamInt32,
              &ADTucam_TriggerExposure);
  createParam(ADTucam_TriggerDelayString, asynParamFloat64,
              &ADTucam_TriggerDelay);
  createParam(ADTucam_TriggerSoftwareString, asynParamInt32,
              &ADTucam_TriggerSoftware);
  createParam(ADTucam_TriggerOut1ModeString, asynParamInt32,
              &ADTucam_TriggerOut1Mode);
  createParam(ADTucam_TriggerOut1EdgeString, asynParamInt32,
              &ADTucam_TriggerOut1Edge);
  createParam(ADTucam_TriggerOut1DelayString, asynParamFloat64,
              &ADTucam_TriggerOut1Delay);
  createParam(ADTucam_TriggerOut1WidthString, asynParamFloat64,
              &ADTucam_TriggerOut1Width);
  createParam(ADTucam_TriggerOut2ModeString, asynParamInt32,
              &ADTucam_TriggerOut2Mode);
  createParam(ADTucam_TriggerOut2EdgeString, asynParamInt32,
              &ADTucam_TriggerOut2Edge);
  createParam(ADTucam_TriggerOut2DelayString, asynParamFloat64,
              &ADTucam_TriggerOut2Delay);
  createParam(ADTucam_TriggerOut2WidthString, asynParamFloat64,
              &ADTucam_TriggerOut2Width);
  createParam(ADTucam_TriggerOut3ModeString, asynParamInt32,
              &ADTucam_TriggerOut3Mode);
  createParam(ADTucam_TriggerOut3EdgeString, asynParamInt32,
              &ADTucam_TriggerOut3Edge);
  createParam(ADTucam_TriggerOut3DelayString, asynParamFloat64,
              &ADTucam_TriggerOut3Delay);
  createParam(ADTucam_TriggerOut3WidthString, asynParamFloat64,
              &ADTucam_TriggerOut3Width);
  createParam(ADTucam_PRNUString, asynParamInt32, &ADTucam_PRNU);
  createParam(ADTucam_DataFormatString, asynParamInt32, &ADTucam_DataFormat);
  createParam(ADTucam_EnableGammaString, asynParamInt32, &ADTucam_EnableGamma);
  createParam(ADTucam_EnableBlackLevelString, asynParamInt32,
              &ADTucam_EnableBlackLevel);

  /* Initialize the idle only int params, these cannot be changed while
   * acquiring */
  this->idleOnlyIntParams_ = {ADTriggerMode,
                              ADTucam_TriggerOut1Mode,
                              ADTucam_TriggerOut2Mode,
                              ADTucam_TriggerOut3Mode,
                              ADTucam_Capture,
                              ADTucam_BinMode,
                              ADMinX,
                              ADMinY,
                              ADSizeX,
                              ADSizeY};
}

void ADTucam::setupPersistentThreads() {
  this->startEventId_ = epicsEventCreate(epicsEventEmpty);
  this->stopEventId_ = epicsEventCreate(epicsEventEmpty);
  this->startFrameTimeoutEventId_ = epicsEventCreate(epicsEventEmpty);
  this->stopFrameTimeoutEventId_ = epicsEventCreate(epicsEventEmpty);
  this->shutdownRequested_ = false;

  /* Launch acquisition thread */
  epicsThreadOpts acquisitionThreadOpts;
  acquisitionThreadOpts.priority = epicsThreadPriorityMedium;
  acquisitionThreadOpts.stackSize =
      epicsThreadGetStackSize(epicsThreadStackMedium);
  acquisitionThreadOpts.joinable = 1;
  this->acquisitionThreadId_ = epicsThreadCreateOpt(
      "ADTucamAcquisitionThread", (EPICSTHREADFUNC)acquisitionThreadC, this,
      &acquisitionThreadOpts);

  /* Launch temperature monitoring task */
  epicsThreadOpts monitorTemperatureThreadOpts;
  monitorTemperatureThreadOpts.priority = epicsThreadPriorityMedium;
  monitorTemperatureThreadOpts.stackSize =
      epicsThreadGetStackSize(epicsThreadStackMedium);
  monitorTemperatureThreadOpts.joinable = 1;
  this->monitorTemperatureThreadId_ =
      epicsThreadCreateOpt("ADTucamMonitorTemperatureThread",
                           (EPICSTHREADFUNC)monitorTemperatureThreadC, this,
                           &monitorTemperatureThreadOpts);

  /* Launch timeout thread */
  epicsThreadOpts frameTimeoutThreadOpts;
  frameTimeoutThreadOpts.priority = epicsThreadPriorityMedium;
  frameTimeoutThreadOpts.stackSize =
      epicsThreadGetStackSize(epicsThreadStackMedium);
  frameTimeoutThreadOpts.joinable = 1;
  this->frameTimeoutThreadId_ = epicsThreadCreateOpt(
      "ADTucamFrameTimeoutThread", (EPICSTHREADFUNC)frameTimeoutThreadC, this,
      &frameTimeoutThreadOpts);
}

void ADTucam::setup() {
  const char *functionName = "setup";
  char versionString[20];
  asynStatus status = asynSuccess;

  this->createParamLibrary();

  /* Set initial values for some parameters */
  setIntegerParam(ADAcquire, 0);
  setIntegerParam(NDDataType, NDUInt16);
  setIntegerParam(NDColorMode, NDColorModeMono);
  setIntegerParam(NDArraySizeZ, 0);
  setStringParam(ADStringToServer, "<not used by driver>");
  setStringParam(ADStringFromServer, "<not used by driver>");
  setStringParam(ADManufacturer, "Tucsen");
  epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
                DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
  setStringParam(NDDriverVersion, versionString);

  status = this->connectCamera();
  if (status != asynSuccess) {
    ERR("Camera connection failed");
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setStringParam(ADStatusMessage, "Camera connection failed");
    callParamCallbacks();
    report(stdout, 1);
    return;
  }
  callParamCallbacks();

  this->setupPersistentThreads();

  /* Register shutdown task */
  if (this->cleanupHandler_) {
    epicsAtExit(exitCallbackC, (void *)this);  // NOLINT(readability/casting)
  }

  // Mark initialization as complete
  this->initialized_ = true;
}

/**
 * @brief ADTucam destructor, called on exit to cleanup spawned threads.
 */
ADTucam::~ADTucam() {
  int acquisitionRequested;

  if (this->initialized_) {
    // Signal threads to shutdown
    this->shutdownRequested_ = true;
    if (this->acquisitionThreadId_) {
      // Stop acquisition if we are currently acquiring
      getIntegerParam(ADAcquire, &acquisitionRequested);
      if (acquisitionRequested == 1) {
        this->stopAcquisition();
      }
      // Wake up acquisition thread so it can exit cleanly
      this->startAcquisition();
      epicsThreadMustJoin(this->acquisitionThreadId_);
    }
    if (this->monitorTemperatureThreadId_) {
      epicsThreadMustJoin(this->monitorTemperatureThreadId_);
    }
    if (this->frameTimeoutThreadId_) {
      // Wake up the frame timeout thread so it can exit cleanly
      epicsEventSignal(this->startFrameTimeoutEventId_);
      epicsEventSignal(this->stopFrameTimeoutEventId_);
      epicsThreadMustJoin(this->frameTimeoutThreadId_);
    }

    this->disconnectCamera();

    epicsEventDestroy(this->startEventId_);
    epicsEventDestroy(this->stopEventId_);
    epicsEventDestroy(this->startFrameTimeoutEventId_);
    epicsEventDestroy(this->stopFrameTimeoutEventId_);
  }

  if (this->cleanupHandler_) {
    delete sdkHandler_;
  }
}

/**
 * @brief Thread that aborts waiting for the next frame if it exceeds a certain
 * timeout.
 */
void ADTucam::frameTimeoutThread() {
  const char *functionName = "frameTimeoutThread";
  TUCAMRET tucStatus;
  int triggerMode;
  double timeout, acquireTime, acquirePeriod;
  epicsEventStatus startEventStatus;
  epicsEventStatus stopEventStatus = epicsEventWaitTimeout;

  while (!this->shutdownRequested_) {
    startEventStatus = epicsEventWait(this->startFrameTimeoutEventId_);
    if (startEventStatus != epicsEventWaitOK) {
      ERR("Failed to wait for timeout event");
      continue;
    }

    this->lock();
    getIntegerParam(ADTriggerMode, &triggerMode);
    if (triggerMode == TUCCM_SEQUENCE ||
        triggerMode == TUCCM_TRIGGER_SOFTWARE) {
      /* Calculate the timeout */
      getDoubleParam(ADAcquireTime, &acquireTime);
      getDoubleParam(ADAcquirePeriod, &acquirePeriod);
      timeout = (acquireTime + acquirePeriod) + 30.0;
      this->unlock();

      /* Wait for the timeout */
      stopEventStatus =
          epicsEventWaitWithTimeout(this->stopFrameTimeoutEventId_, timeout);
      if (stopEventStatus == epicsEventWaitTimeout) {
        ERR_ARGS("Aborting wait for frame due to timeout: %f seconds", timeout);
        tucStatus = sdkHandler_->abortWait(this->camHandle_.hIdxTUCam);
        if (tucStatus != TUCAMRET_SUCCESS) {
          ERR_ARGS("unable to abort wait for frame, tucStatus=(0x%x)",
                   tucStatus);
        }
        epicsEventWait(this->stopFrameTimeoutEventId_);
      }
    } else {
      this->unlock();
      epicsEventWait(this->stopFrameTimeoutEventId_);
    }
  }
  INFO("Frame timeout thread shutting down.");
}

/**
 * @brief Warns the user if the temperature exceeds 40 degrees.
 */
asynStatus ADTucam::warnOnExtremeTemperature(double temperatureVal) {
  const char *functionName = "warnOnExtremeTemperature";
  asynStatus status = asynSuccess;
  int emergencySignal;
  getIntegerParam(ADTucam_TemperatureEmergencySignal, &emergencySignal);
  if (temperatureVal >= 40.0 && !emergencySignal) {
    status = setIntegerParam(ADTucam_TemperatureEmergencySignal, 1);
    WARN_ARGS("Temperature exceeds 40.0 degrees: %f!!", temperatureVal);
  } else if (temperatureVal < 40.0 && emergencySignal) {
    status = setIntegerParam(ADTucam_TemperatureEmergencySignal, 0);
    INFO_ARGS("Temperature is back to normal: %f", temperatureVal);
  }

  return status;
}

/**
 * @brief Automatically handles enabling/disabling the TEC depending on the
 * temperature gradient.
 *
 * The TEC enables heat to be dissipated from the camera *only if* a temperature
 * gradient exists on both sides of the TEC plate. I.e. a coolant must be
 * flowing on the other side of the plate to remove the heat produced by the
 * camera.
 *
 * This method handles the following scenarios:
 * - If the temperature drops under `x` degrees, we can safely enable the TEC
 * since we can assume that coolant is flowing. `x` should be at a maximum room
 * temperature.
 * - If the temperature exceeds `x` degrees, we need to disable the TEC to
 * reduce the power consumption of the camera which buys more time for an
 * operator to shut down the camera safely. We should still flag this as an
 * emergency to the user since coolant is most likely not flowing.
 *
 * If we had enabled the TEC and no coolant was flowing, the camera's
 * temperature would rapidly increase due to the increased power consumption of
 * the TEC.
 */
asynStatus ADTucam::handleTEC(double temperatureVal) {
  const char *functionName = "handleTEC";
  int status = asynSuccess;
  int tecStatus;
  double tecThreshold;

  getDoubleParam(ADTucam_AutoTECThreshold, &tecThreshold);
  // TEC is active but we are above the threshold
  // We assume no temperature gradient is present
  if (this->tecActive && temperatureVal > tecThreshold) {
    WARN_ARGS("Temperature exceeds %f degrees: %f!!", tecThreshold,
              temperatureVal);
    status |= setCapability(TUIDC_ENABLETEC, 0);
    status |= getCapability(TUIDC_ENABLETEC, &tecStatus);
    if (tecStatus == 0) {
      WARN("TEC DISABLED.");
      this->tecActive = false;
    }
    status |= setIntegerParam(ADTucam_TECStatus, tecStatus);
  } else if (!this->tecActive && temperatureVal <= tecThreshold) {
    // TEC is inactive but we are below the threshold
    // We assume a temperature gradient is present
    WARN_ARGS("Temperature has reached an acceptable value for TEC: %f",
              temperatureVal);
    status |= setCapability(TUIDC_ENABLETEC, 1);
    status |= getCapability(TUIDC_ENABLETEC, &tecStatus);
    if (tecStatus == 1) {
      WARN("TEC ENABLED.");
      this->tecActive = true;
    }
    status |= setIntegerParam(ADTucam_TECStatus, tecStatus);
  }

  return (asynStatus)status;
}

/**
 * @brief Monitors the temperature of the camera and updates the temperature PV
 */
void ADTucam::monitorTemperatureThread() {
  const char *functionName = "monitorTemperatureThread";
  double temperatureVal = 1.0f;
  int autoTEC;
  INFO("Temperature monitor thread active.");

  while (!this->shutdownRequested_) {
    this->lock();
    this->getProperty(TUIDP_TEMPERATURE, &temperatureVal);
    setDoubleParam(ADTucam_Temperature, temperatureVal);
    this->warnOnExtremeTemperature(temperatureVal);
    // Auto-enable/disable TEC
    getIntegerParam(ADTucam_AutoTEC, &autoTEC);
    if (autoTEC) {
      this->handleTEC(temperatureVal);
    }
    callParamCallbacks();
    this->unlock();
    epicsThreadSleep(1.0);
  }
  INFO("Temperature monitor thread shutting down.");
}

void ADTucam::handleStopEvent() {
  const char *functionName = "handleStopEvent";
  int imageMode;
  int triggerMode;

  /* If the trigger mode is free run, we need to stop the capture
   * Otherwise, clients should stop the capture themselves */
  getIntegerParam(ADTriggerMode, &triggerMode);
  if (triggerMode == TUCCM_SEQUENCE) {
    this->stopCapture();
    setIntegerParam(ADTucam_Capture, 0);
  }

  getIntegerParam(ADImageMode, &imageMode);
  if (imageMode == ADImageContinuous) {
    setIntegerParam(ADStatus, ADStatusIdle);
    setStringParam(ADStatusMessage, "Waiting for acquisition");
    INFO("Acquistion stopped");
  } else {
    setIntegerParam(ADStatus, ADStatusAborted);
    setStringParam(ADStatusMessage, "Acquisition aborted");
    INFO("Acquistion aborted");
  }
  callParamCallbacks();
}

void ADTucam::acquisitionThread() {
  int status = asynSuccess;
  int eventStatus;
  int imageCounter;
  int numImages, numImagesCounter;
  int imageMode;
  int triggerMode;
  int capturing;
  int arrayCallbacks;
  int numRetries, retryOnTimeout;
  int retryCounter = 0;
  bool acquire = false;
  NDArray *pImage;
  epicsTimeStamp startTime;
  const char *functionName = "acquisitionThread";

  this->lock();

  while (true) {
    /* Wait for acquisition to start */
    if (!acquire) {
      INFO("Waiting for acquisition to start...");
      this->unlock();
      eventStatus = epicsEventWait(this->startEventId_);
      this->lock();
      if (eventStatus != epicsEventWaitOK) {
        ERR("Failed to wait for acquisition start event");
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage,
                       "Failed to wait for acquisition start event");
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        acquire = false;
        continue;
      }
      if (this->shutdownRequested_) {
        break;
      }
      INFO("Starting acquisition...");
      getIntegerParam(ADTucam_Capture, &capturing);
      if (!capturing) {
        status = this->startCapture();
        if (status != asynSuccess) {
          ERR("Failed to start capture");
          setIntegerParam(ADStatus, ADStatusError);
          setStringParam(ADStatusMessage, "Failed to start capture");
          setIntegerParam(ADAcquire, 0);
          callParamCallbacks();
          continue;
        }
        setIntegerParam(ADTucam_Capture, 1);
      }
      acquire = true;
      setStringParam(ADStatusMessage, "Acquiring...");
      setIntegerParam(ADNumImagesCounter, 0);
      retryCounter = 0;
    }

    /* Acquisition is active */
    epicsTimeGetCurrent(&startTime);
    getIntegerParam(ADImageMode, &imageMode);
    setIntegerParam(ADStatus, ADStatusAcquire);
    callParamCallbacks();

    /* Check if the acquisition was stopped */
    this->unlock();
    eventStatus = epicsEventWaitWithTimeout(this->stopEventId_, 1e-5);
    this->lock();
    if (eventStatus == epicsEventWaitOK) {
      this->handleStopEvent();
      acquire = false;
      continue;
    }

    /* Grab the next frame */
    status = this->grabImage(&startTime);
    if (status != asynSuccess) {
      if (this->pRaw_) this->pRaw_->release();
      this->pRaw_ = NULL;

      /* Check if acquisition was stopped or failed */
      this->unlock();
      eventStatus = epicsEventWaitWithTimeout(this->stopEventId_, 1e-5);
      this->lock();
      if (eventStatus == epicsEventWaitOK) {
        this->handleStopEvent();
        acquire = false;
      } else {
        ERR("Failed to grab image");
        /* Handle retry logic */
        getIntegerParam(ADTucam_NumRetries, &numRetries);
        getIntegerParam(ADTucam_RetryOnTimeout, &retryOnTimeout);
        if (retryOnTimeout && retryCounter < numRetries) {
          ++retryCounter;
          setIntegerParam(ADStatus, ADStatusError);
          char retryMessage[256];
          snprintf(retryMessage, sizeof(retryMessage),
                   "Retrying acquisition (attempt %d of %d)...", retryCounter,
                   numRetries);
          setStringParam(ADStatusMessage, retryMessage);
        } else {
          this->stopCapture();
          setIntegerParam(ADTucam_Capture, 0);
          setIntegerParam(ADStatus, ADStatusError);
          setStringParam(ADStatusMessage, "Failed to grab image");
          setIntegerParam(ADAcquire, 0);
          acquire = false;
        }
        callParamCallbacks();
      }
      continue;
    }

    /* Update the status */
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    /* Update the image statistics */
    pImage = this->pRaw_;

    getIntegerParam(NDArrayCounter, &imageCounter);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);

    pImage->uniqueId = imageCounter;
    updateTimeStamp(&pImage->epicsTS);

    /* Perform plugin callbacks */
    if (arrayCallbacks) {
      INFO("Performing plugin callbacks...");
      doCallbacksGenericPointer(pImage, NDArrayData, 0);
    }

    /* Free array buffer */
    if (this->pRaw_) this->pRaw_->release();
    this->pRaw_ = NULL;

    /* Check if we have reached the end of the acquisition */
    if ((imageMode == ADImageSingle) ||
        ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
      /* If the trigger mode is free run, we need to stop the capture
       * Otherwise, clients should stop the capture themselves */
      getIntegerParam(ADTriggerMode, &triggerMode);
      if (triggerMode == TUCCM_SEQUENCE) {
        status = this->stopCapture();
        if (status != asynSuccess) {
          ERR("Failed to stop capture");
        } else {
          setIntegerParam(ADTucam_Capture, 0);
        }
      }
      setStringParam(ADStatusMessage, "Waiting for acquisition");
      setIntegerParam(ADStatus, ADStatusIdle);
      setIntegerParam(ADAcquire, 0);
      INFO("Acquisition complete");
      acquire = false;
    }

    callParamCallbacks();
  }
  this->unlock();
  INFO("Acquisition thread shutting down.");
}

/**
 * @brief Override of ADDriver function - reads the capability of the camera and
 * dynamically fills mbbi and mbbo fields at iocInit.
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param strings Array of strings to be filled with capability names
 * @param values Array of integers to be filled with capability values
 * @param severities Array of integers to be filled with capability severities
 * @param nElements Number of elements in the arrays
 * @param nIn Number of elements filled in the arrays
 *
 * @returns asynSuccess if read was successful, asynError otherwise
 */
asynStatus ADTucam::readEnum(asynUser *pasynUser, char *strings[], int values[],
                             int severities[], size_t nElements, size_t *nIn) {
  int status = asynSuccess;
  int function = pasynUser->reason;

  if (function == ADTucam_BinMode) {
    status = getCapabilityText(TUIDC_RESOLUTION, strings, values, severities,
                               nElements, nIn);
  } else {
    *nIn = 0;
    status = asynError;
  }
  return (asynStatus)status;
}

asynStatus ADTucam::handleAcquisitionRequest(int value) {
  int status = asynSuccess;
  int currentRequest;

  getIntegerParam(ADAcquire, &currentRequest);
  if (value == 1 && currentRequest == 0) {
    status |= this->startAcquisition();
    status |= setIntegerParam(ADAcquire, value);
  } else if (value == 0 && currentRequest == 1) {
    status |= this->stopAcquisition();
    status |= setIntegerParam(ADAcquire, value);
  } else {
    status = asynError;
  }
  return (asynStatus)status;
}

asynStatus ADTucam::handleCaptureRequest(int value) {
  int status = asynSuccess;
  int currentRequest;

  getIntegerParam(ADTucam_Capture, &currentRequest);
  if (value == 1 && currentRequest == 0) {
    status |= this->startCapture();
  } else if (value == 0 && currentRequest == 1) {
    status |= this->stopCapture();
  } else {
    status = asynError;
  }
  if (status == asynSuccess) {
    status |= setIntegerParam(ADTucam_Capture, value);
  }

  return (asynStatus)status;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to
 * int PVs
 *
 * NOTE: Driver is assumed to be locked when this function is called.
 *       This happens already when called from asynPortDriver::writeInt32.
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 *
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADTucam::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  const char *functionName = "writeInt32";
  int status = asynSuccess;
  int tucStatus;
  int function = pasynUser->reason;
  int readback;
  int acquiring;
  const char *paramName;
  asynStatus nameStatus;

  nameStatus = this->getParamName(function, &paramName);
  if (nameStatus == asynSuccess) {
    INFO_ARGS("requested, function=%s, value=%d", paramName, value);
  } else {
    INFO_ARGS("requested, function=%d, value=%d", function, value);
  }

  getIntegerParam(ADAcquire, &acquiring);
  if (acquiring && this->idleOnlyIntParams_.find(function) !=
                       this->idleOnlyIntParams_.end()) {
    this->getParamName(function, &paramName);
    ERR_ARGS("Cannot change parameter %s while acquiring!", paramName);
    return asynError;
  }

  if (function == ADAcquire) {
    status |= this->handleAcquisitionRequest(value);
  } else if (function == ADTucam_Capture) {
    status |= this->handleCaptureRequest(value);
  } else if (function == ADTucam_TECStatus) {
    status |= setCapability(TUIDC_ENABLETEC, value);
    status |= getCapability(TUIDC_ENABLETEC, &readback);
    status |= setIntegerParam(function, readback);
    if (readback == 1) {
      INFO("Enabled TEC!");
      this->tecActive = true;
    } else {
      INFO("Disabled TEC!");
      this->tecActive = false;
    }
  } else if (function == ADTucam_GainMode) {
    // TUIDP_GLOBALGAIN is a property which requires float64 (double)
    // Although at the EPICS level we want this to be an integer (enum)
    // Possible values are:
    //  0 -> HDR
    //  1 -> HighGain
    //  2 -> LowGain
    //  3 -> HDR NO DSNU
    //  4 -> HighGain NO DSNU
    //  5 -> LowGain NO DSNU
    double gainMode;
    status |= setProperty(TUIDP_GLOBALGAIN, static_cast<double>(value));
    status |= getProperty(TUIDP_GLOBALGAIN, &gainMode);
    readback = static_cast<int>(gainMode);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_BinMode) {
    // Set the binninig mode.
    // Possible values are:
    //  0 -> None
    //  1 -> 2x2
    //  2 -> 4x4
    tucStatus = sdkHandler_->releaseBuffer(this->camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("Failed to release camera buffer, tucStatus=(0x%x)", tucStatus);
      return asynError;
    }
    status |= setCapability(TUIDC_RESOLUTION, value);
    if (status != asynSuccess) {
      ERR("Failed to set binning mode!");
      // Re-allocate the old buffer
      tucStatus = sdkHandler_->allocateBuffer(this->camHandle_.hIdxTUCam,
                                              &this->frameHandle_);
      if (tucStatus != TUCAMRET_SUCCESS) {
        ERR_ARGS(
            "Failed to re-allocate the old camera buffer, tucStatus=(0x%x)",
            tucStatus);
      }
      return asynError;
    }
    tucStatus = sdkHandler_->allocateBuffer(this->camHandle_.hIdxTUCam,
                                            &this->frameHandle_);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("Failed to allocate camera buffer, tucStatus=(0x%x)", tucStatus);
      return asynError;
    }
    /* Readback the binning mode and new ROI */
    status |= getCapability(TUIDC_RESOLUTION, &readback);
    status |= setIntegerParam(function, readback);
    status |= setCurrentROI();
  } else if (function == ADMinX) {
    int minY, sizeX, sizeY;
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    status |= setCameraROI(value, minY, sizeX, sizeY);
    status |= setCurrentROI();
  } else if (function == ADMinY) {
    int minX, sizeX, sizeY;
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    status |= setCameraROI(minX, value, sizeX, sizeY);
    status |= setCurrentROI();
  } else if (function == ADSizeX) {
    int minX, minY, sizeY;
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeY, &sizeY);
    status |= setCameraROI(minX, minY, value, sizeY);
    status |= setCurrentROI();
  } else if (function == ADSizeY) {
    int minX, minY, sizeX;
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(ADSizeX, &sizeX);
    status |= setCameraROI(minX, minY, sizeX, value);
    status |= setCurrentROI();
  } else if (function == ADReverseX) {
    status |= setCapability(TUIDC_HORIZONTAL, value);
    status |= getCapability(TUIDC_HORIZONTAL, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADReverseY) {
    status |= setCapability(TUIDC_VERTICAL, value);
    status |= getCapability(TUIDC_VERTICAL, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTriggerMode) {
    int triggerEdge, triggerExposure;
    double triggerDelay;
    getIntegerParam(ADTucam_TriggerEdge, &triggerEdge);
    getIntegerParam(ADTucam_TriggerExposure, &triggerExposure);
    getDoubleParam(ADTucam_TriggerDelay, &triggerDelay);
    status |= this->setCameraTrigger(value, triggerEdge, triggerExposure,
                                     triggerDelay);
    status |= this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerExposure) {
    int triggerMode, triggerEdge;
    double triggerDelay;
    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(ADTucam_TriggerEdge, &triggerEdge);
    getDoubleParam(ADTucam_TriggerDelay, &triggerDelay);
    status |=
        this->setCameraTrigger(triggerMode, triggerEdge, value, triggerDelay);
    status |= this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerEdge) {
    int triggerMode, triggerExposure;
    double triggerDelay;
    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(ADTucam_TriggerExposure, &triggerExposure);
    getDoubleParam(ADTucam_TriggerDelay, &triggerDelay);
    status |= this->setCameraTrigger(triggerMode, value, triggerExposure,
                                     triggerDelay);
    status |= this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerOut1Mode) {
    if (triggerOutSupport_ == 1) {
      int triggerOut1Edge;
      double triggerOut1Delay, triggerOut1Width;
      getIntegerParam(ADTucam_TriggerOut1Edge, &triggerOut1Edge);
      getDoubleParam(ADTucam_TriggerOut1Delay, &triggerOut1Delay);
      getDoubleParam(ADTucam_TriggerOut1Width, &triggerOut1Width);
      status |= this->setCameraTriggerOut(0, value, triggerOut1Edge,
                                          triggerOut1Delay, triggerOut1Width);
      status |= this->setCurrentTriggerOut(0);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 1 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut1Edge) {
    if (triggerOutSupport_ == 1) {
      int triggerOut1Mode;
      double triggerOut1Delay, triggerOut1Width;
      getIntegerParam(ADTucam_TriggerOut1Mode, &triggerOut1Mode);
      getDoubleParam(ADTucam_TriggerOut1Delay, &triggerOut1Delay);
      getDoubleParam(ADTucam_TriggerOut1Width, &triggerOut1Width);
      status |= this->setCameraTriggerOut(0, triggerOut1Mode, value,
                                          triggerOut1Delay, triggerOut1Width);
      status |= this->setCurrentTriggerOut(0);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 1 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut2Mode) {
    if (triggerOutSupport_ == 1) {
      int triggerOut2Edge;
      double triggerOut2Delay, triggerOut2Width;
      getIntegerParam(ADTucam_TriggerOut2Edge, &triggerOut2Edge);
      getDoubleParam(ADTucam_TriggerOut2Delay, &triggerOut2Delay);
      getDoubleParam(ADTucam_TriggerOut2Width, &triggerOut2Width);
      status |= this->setCameraTriggerOut(1, value, triggerOut2Edge,
                                          triggerOut2Delay, triggerOut2Width);
      status |= this->setCurrentTriggerOut(1);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 2 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut2Edge) {
    if (triggerOutSupport_ == 1) {
      int triggerOut2Mode;
      double triggerOut2Delay, triggerOut2Width;
      getIntegerParam(ADTucam_TriggerOut2Mode, &triggerOut2Mode);
      getDoubleParam(ADTucam_TriggerOut2Delay, &triggerOut2Delay);
      getDoubleParam(ADTucam_TriggerOut2Width, &triggerOut2Width);
      status |= this->setCameraTriggerOut(1, triggerOut2Mode, value,
                                          triggerOut2Delay, triggerOut2Width);
      status |= this->setCurrentTriggerOut(1);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 2 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut3Mode) {
    if (triggerOutSupport_ == 1) {
      int triggerOut3Edge;
      double triggerOut3Delay, triggerOut3Width;
      getIntegerParam(ADTucam_TriggerOut3Edge, &triggerOut3Edge);
      getDoubleParam(ADTucam_TriggerOut3Delay, &triggerOut3Delay);
      getDoubleParam(ADTucam_TriggerOut3Width, &triggerOut3Width);
      status |= this->setCameraTriggerOut(2, value, triggerOut3Edge,
                                          triggerOut3Delay, triggerOut3Width);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 3 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut3Edge) {
    if (triggerOutSupport_ == 1) {
      int triggerOut3Mode;
      double triggerOut3Delay, triggerOut3Width;
      getIntegerParam(ADTucam_TriggerOut3Mode, &triggerOut3Mode);
      getDoubleParam(ADTucam_TriggerOut3Delay, &triggerOut3Delay);
      getDoubleParam(ADTucam_TriggerOut3Width, &triggerOut3Width);
      status |= this->setCameraTriggerOut(2, triggerOut3Mode, value,
                                          triggerOut3Delay, triggerOut3Width);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 3 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_FrameFormat) {
    frameHandle_.ucFormatGet = frameFormats[value];
    // TUIDC_DATAFORMAT is 0 for YUV and 1 for RAW
    int dataFormat = frameFormats[value] == TUFRM_FMT_RAW ? 1 : 0;
    status |= setCapability(TUIDC_DATAFORMAT, dataFormat);
    status |= getCapability(TUIDC_DATAFORMAT, &dataFormat);
    status |= setIntegerParam(ADTucam_DataFormat, dataFormat);
    status |= setIntegerParam(function, value);
  } else if (function == ADTucam_BitDepth) {
    status |= setCapability(TUIDC_BITOFDEPTH, value);
    status |= getCapability(TUIDC_BITOFDEPTH, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_FanGear) {
    status |= setCapability(TUIDC_FAN_GEAR, value);
    status |= getCapability(TUIDC_FAN_GEAR, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_ImageMode) {
    status |= setCapability(TUIDC_IMGMODESELECT, value);
    status |= getCapability(TUIDC_IMGMODESELECT, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_AutoExposure) {
    status |= setCapability(TUIDC_ATEXPOSURE, value);
    status |= getCapability(TUIDC_ATEXPOSURE, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_AutoLevels) {
    status |= setCapability(TUIDC_ATLEVELS, value);
    status |= getCapability(TUIDC_ATLEVELS, &readback);
    status |= setIntegerParam(function, readback);
    int hist = (value != 0);
    status |= setCapability(TUIDC_HISTC, hist);
    status |= getCapability(TUIDC_HISTC, &hist);
    status |= setIntegerParam(ADTucam_Histogram, hist);
  } else if (function == ADTucam_Histogram) {
    status |= setCapability(TUIDC_HISTC, value);
    status |= getCapability(TUIDC_HISTC, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_Enhance) {
    status |= setCapability(TUIDC_ENHANCE, value);
    status |= getCapability(TUIDC_ENHANCE, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_DefectCorr) {
    status |= setCapability(TUIDC_DFTCORRECTION, value);
    status |= getCapability(TUIDC_DFTCORRECTION, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_Denoise) {
    status |= setCapability(TUIDC_ENABLEDENOISE, value);
    status |= getCapability(TUIDC_ENABLEDENOISE, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_FlatCorr) {
    status |= setCapability(TUIDC_FLTCORRECTION, value);
    status |= getCapability(TUIDC_FLTCORRECTION, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_PRNU) {
    status |= setCapability(TUIDC_ENABLEIMGPRO, value ? 0x0E : 0x0A);
    status |= getCapability(TUIDC_ENABLEIMGPRO, &readback);
    status |= setIntegerParam(function, readback == 0x0E ? 1 : 0);
  } else if (function == ADTucam_DataFormat) {
    status |= setCapability(TUIDC_DATAFORMAT, value);
    status |= getCapability(TUIDC_DATAFORMAT, &readback);
    status |= setIntegerParam(function, readback);
    if (readback == 1) {
      frameHandle_.ucFormatGet = TUFRM_FMT_RAW;
      status |= setIntegerParam(ADTucam_FrameFormat, 0);
    }
  } else if (function == ADTucam_EnableGamma) {
    status |= setCapability(TUIDC_ENABLEGAMMA, value);
    status |= getCapability(TUIDC_ENABLEGAMMA, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function == ADTucam_EnableBlackLevel) {
    status |= setCapability(TUIDC_ENABLEBLACKLEVEL, value);
    status |= getCapability(TUIDC_ENABLEBLACKLEVEL, &readback);
    status |= setIntegerParam(function, readback);
  } else if (function < FIRST_TUCAM_PARAM) {
    status |= setIntegerParam(function, value);
    status |= ADDriver::writeInt32(pasynUser, value);
  } else {
    status |= setIntegerParam(function, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  nameStatus = this->getParamName(function, &paramName);
  if (nameStatus == asynSuccess) {
    if (status != asynSuccess) {
      ERR_ARGS("error, status=%d, function=%s, value=%d", status, paramName,
               value);
    } else {
      INFO_ARGS("completed, function=%s, value=%d", paramName, value);
    }
  } else {
    if (status != asynSuccess) {
      ERR_ARGS("error, status=%d, function=%d, value=%d", status, function,
               value);
    } else {
      INFO_ARGS("completed, function=%d, value=%d", function, value);
    }
  }

  return (asynStatus)status;
}

asynStatus ADTucam::setAcquirePeriod(double period) {
  static const char *functionName = "setAcquirePeriod";
  int status = asynSuccess;
  if (period <= 0) {
    ERR_ARGS(
        "Acquire period %f is less than or equal to 0! Setting it to 0.001",
        period);
    period = 0.001;
  }
  double frequency = 1 / period;
  status |= setProperty(TUIDP_FRAME_RATE, frequency);
  status |= getProperty(TUIDP_FRAME_RATE, &frequency);
  status |= setDoubleParam(ADAcquirePeriod, 1 / frequency);
  return (asynStatus)status;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to
 * float PVs
 *
 * NOTE: Driver is assumed to be locked when this function is called.
 *       This happens already when called from asynPortDriver::writeFloat64.
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 *
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADTucam::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  static const char *functionName = "writeFloat64";
  int status = asynSuccess;
  int function = pasynUser->reason;
  const char *paramName;
  asynStatus nameStatus;

  nameStatus = this->getParamName(function, &paramName);
  if (nameStatus == asynSuccess) {
    INFO_ARGS("requested, function=%s, value=%f", paramName, value);
  } else {
    INFO_ARGS("requested, function=%d, value=%f", function, value);
  }

  if (function == ADAcquirePeriod) {
    status |= this->setAcquirePeriod(value);
  } else if (function == ADAcquireTime) {
    // if exposure time is set to something longer than acquire period, change
    // acquire period to be the same
    double exposureTimeMs;
    double period;
    getDoubleParam(ADAcquirePeriod, &period);
    if (value > period) {
      status |= this->setAcquirePeriod(value);
    }
    exposureTimeMs = value * 1000.0;
    status |= setProperty(TUIDP_EXPOSURETM, exposureTimeMs);
    status |= getProperty(TUIDP_EXPOSURETM, &exposureTimeMs);
    status |= setDoubleParam(ADAcquireTime, exposureTimeMs / 1000.0);
  } else if (function == ADTucam_TemperatureSetpoint) {
    // The TUIDP_TEMPERATURE takes a value between 0 and 100 which maps to -50C
    // to 50C The user input is in the range -50C to 50C so we adjust it
    // accordingly
    double minTemperature = -50.0;
    double maxTemperature = 50.0;
    double temperature = value;
    if (temperature < minTemperature || temperature > maxTemperature) {
      WARN_ARGS(
          "Temperature setpoint %f is out of range. Valid range is %f to "
          "%f! Clamping the value to the valid range...",
          value, minTemperature, maxTemperature);
      temperature =
          std::min(maxTemperature, std::max(temperature, minTemperature));
    }
    double rescaledTemperature = temperature + 50;
    status |= setProperty(TUIDP_TEMPERATURE, rescaledTemperature);
    DEBUG_ARGS("New temperature setpoint detected: degrees=%fC, setting=%f",
               temperature, rescaledTemperature);
    status |= setDoubleParam(ADTucam_TemperatureSetpoint, temperature);
  } else if (function == ADTucam_TriggerDelay) {
    int triggerMode, triggerEdge, triggerExposure;
    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(ADTucam_TriggerEdge, &triggerEdge);
    getIntegerParam(ADTucam_TriggerExposure, &triggerExposure);
    status |= this->setCameraTrigger(triggerMode, triggerEdge, triggerExposure,
                                     value);
    status |= this->setCurrentTrigger();
  } else if (function == ADTucam_TriggerOut1Delay) {
    if (triggerOutSupport_ == 1) {
      int triggerOut1Mode, triggerOut1Edge;
      double triggerOut1Width;
      getIntegerParam(ADTucam_TriggerOut1Mode, &triggerOut1Mode);
      getIntegerParam(ADTucam_TriggerOut1Edge, &triggerOut1Edge);
      getDoubleParam(ADTucam_TriggerOut1Width, &triggerOut1Width);
      status |= this->setCameraTriggerOut(0, triggerOut1Mode, triggerOut1Edge,
                                          value, triggerOut1Width);
      status |= this->setCurrentTriggerOut(0);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 1 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut1Width) {
    if (triggerOutSupport_ == 1) {
      int triggerOut1Mode, triggerOut1Edge;
      double triggerOut1Delay;
      getIntegerParam(ADTucam_TriggerOut1Mode, &triggerOut1Mode);
      getIntegerParam(ADTucam_TriggerOut1Edge, &triggerOut1Edge);
      getDoubleParam(ADTucam_TriggerOut1Delay, &triggerOut1Delay);
      status |= this->setCameraTriggerOut(0, triggerOut1Mode, triggerOut1Edge,
                                          triggerOut1Delay, value);
      status |= this->setCurrentTriggerOut(0);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 1 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut2Delay) {
    if (triggerOutSupport_ == 1) {
      int triggerOut2Mode, triggerOut2Edge;
      double triggerOut2Width;
      getIntegerParam(ADTucam_TriggerOut2Mode, &triggerOut2Mode);
      getIntegerParam(ADTucam_TriggerOut2Edge, &triggerOut2Edge);
      getDoubleParam(ADTucam_TriggerOut2Width, &triggerOut2Width);
      status |= this->setCameraTriggerOut(1, triggerOut2Mode, triggerOut2Edge,
                                          value, triggerOut2Width);
      status |= this->setCurrentTriggerOut(1);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 2 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut2Width) {
    if (triggerOutSupport_ == 1) {
      int triggerOut2Mode, triggerOut2Edge;
      double triggerOut2Delay;
      getIntegerParam(ADTucam_TriggerOut2Mode, &triggerOut2Mode);
      getIntegerParam(ADTucam_TriggerOut2Edge, &triggerOut2Edge);
      getDoubleParam(ADTucam_TriggerOut2Delay, &triggerOut2Delay);
      status |= this->setCameraTriggerOut(1, triggerOut2Mode, triggerOut2Edge,
                                          triggerOut2Delay, value);
      status |= this->setCurrentTriggerOut(1);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 2 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut3Delay) {
    if (triggerOutSupport_ == 1) {
      int triggerOut3Mode, triggerOut3Edge;
      double triggerOut3Width;
      getIntegerParam(ADTucam_TriggerOut3Mode, &triggerOut3Mode);
      getIntegerParam(ADTucam_TriggerOut3Edge, &triggerOut3Edge);
      getDoubleParam(ADTucam_TriggerOut3Width, &triggerOut3Width);
      status |= this->setCameraTriggerOut(2, triggerOut3Mode, triggerOut3Edge,
                                          value, triggerOut3Width);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 3 not supported");
      status |= asynError;
    }
  } else if (function == ADTucam_TriggerOut3Width) {
    if (triggerOutSupport_ == 1) {
      int triggerOut3Mode, triggerOut3Edge;
      double triggerOut3Delay;
      getIntegerParam(ADTucam_TriggerOut3Mode, &triggerOut3Mode);
      getIntegerParam(ADTucam_TriggerOut3Edge, &triggerOut3Edge);
      getDoubleParam(ADTucam_TriggerOut3Delay, &triggerOut3Delay);
      status |= this->setCameraTriggerOut(2, triggerOut3Mode, triggerOut3Edge,
                                          triggerOut3Delay, value);
      status |= this->setCurrentTriggerOut(2);
    } else {
      status |= setStringParam(ADStatusMessage, "Trigger out 3 not supported");
      status |= asynError;
    }
  } else if (function == ADGain) {
    status |= setProperty(TUIDP_GLOBALGAIN, value);
    status |= getProperty(TUIDP_GLOBALGAIN, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_Brightness) {
    status |= setProperty(TUIDP_BRIGHTNESS, value);
    status |= getProperty(TUIDP_BRIGHTNESS, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_BlackLevel) {
    status |= setProperty(TUIDP_BLACKLEVEL, value);
    status |= getProperty(TUIDP_BLACKLEVEL, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_Sharpness) {
    status |= setProperty(TUIDP_SHARPNESS, value);
    status |= getProperty(TUIDP_SHARPNESS, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_NoiseLevel) {
    status |= setProperty(TUIDP_NOISELEVEL, value);
    status |= getProperty(TUIDP_NOISELEVEL, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_HDRK) {
    status |= setProperty(TUIDP_HDR_KVALUE, value);
    status |= getProperty(TUIDP_HDR_KVALUE, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_Gamma) {
    status |= setProperty(TUIDP_GAMMA, value);
    status |= getProperty(TUIDP_GAMMA, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_Contrast) {
    status |= setProperty(TUIDP_CONTRAST, value);
    status |= getProperty(TUIDP_CONTRAST, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_LeftLevel) {
    status |= setProperty(TUIDP_LFTLEVELS, value);
    status |= getProperty(TUIDP_LFTLEVELS, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_RightLevel) {
    status |= setProperty(TUIDP_RGTLEVELS, value);
    status |= getProperty(TUIDP_RGTLEVELS, &value);
    if (status) value = 0;
    status |= setDoubleParam(function, value);
  } else if (function == ADTucam_AutoTECThreshold) {
    // Dangerous to have auto TEC threshold above 40 degrees
    if (value >= 40.0) {
      WARN_ARGS(
          "Requested TEC threshold %f is above 40 degrees! Setting it to "
          "40 degrees...",
          value);
      status |= setDoubleParam(ADTucam_AutoTECThreshold, 40.0);
      status |= asynError;
    } else {
      status |= setDoubleParam(ADTucam_AutoTECThreshold, value);
    }
  } else if (function < FIRST_TUCAM_PARAM) {
    status |= setDoubleParam(function, value);
    status |= ADDriver::writeFloat64(pasynUser, value);
  } else {
    status |= setDoubleParam(function, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  nameStatus = this->getParamName(function, &paramName);
  if (nameStatus == asynSuccess) {
    if (status != asynSuccess) {
      ERR_ARGS("error, status=%d, function=%s, value=%f", status, paramName,
               value);
    } else {
      INFO_ARGS("completed, function=%s, value=%f", paramName, value);
    }
  } else {
    if (status != asynSuccess) {
      ERR_ARGS("error, status=%d, function=%d, value=%f", status, function,
               value);
    } else {
      INFO_ARGS("completed,function=%d, value=%f", function, value);
    }
  }

  return (asynStatus)status;
}

/**
 * @brief Grabs a single image from the camera and stores it in the NDArray
 * buffer.
 *
 * We assume that the driver is locked when this function is called.
 *
 * @param startTimeStamp EPICS time when the acquisition started
 *
 * @returns asynSuccess if image was successfully grabbed, asynError otherwise
 */
asynStatus ADTucam::grabImage(epicsTimeStamp *startTimeStamp) {
  static const char *functionName = "grabImage";
  int status = asynSuccess;
  TUCAMRET tucStatus;
  int nCols, nRows;
  int pixelFormat, channels, pixelBytes;
  size_t dataSize, tDataSize;
  NDDataType_t dataType = NDUInt16;
  NDColorMode_t colorMode = NDColorModeMono;
  int numColors = 1;
  int pixelSize = 2;
  size_t dims[3] = {0};
  int nDims;
  int triggerMode;
  double timeout, acquireTime, acquirePeriod;
  TUCAM_IMG_HEADER frameHeader;

  /* If the trigger mode is software, we need to issue a software trigger
   * to start exposing the next frame */
  getIntegerParam(ADTriggerMode, &triggerMode);
  if (triggerMode == TUCCM_TRIGGER_SOFTWARE) {
    INFO("Issuing software trigger to start exposure");
    tucStatus = sdkHandler_->doSoftwareTrigger(this->camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR("Failed to issue software trigger");
      return asynError;
    }
  }

  getDoubleParam(ADAcquireTime, &acquireTime);
  getDoubleParam(ADAcquirePeriod, &acquirePeriod);

  /* Convert to ms and add a 2s buffer */
  timeout = (acquireTime + acquirePeriod) * 1000 + 2000;

  INFO_ARGS("Waiting for frame... timeout=%f ms", timeout);
  this->unlock();
  epicsEventSignal(this->startFrameTimeoutEventId_);
  // NOTE: Extremely unlikely race condition here if the timeout thread finishes
  // before waiting for the frame. Unfortunately, this is the best we can do.
  tucStatus =
      sdkHandler_->waitForFrame(camHandle_.hIdxTUCam, &frameHandle_, timeout);
  epicsEventSignal(this->stopFrameTimeoutEventId_);
  this->lock();
  if (tucStatus == TUCAMRET_ABORT) {
    INFO_ARGS("Acquisition aborted, tucStatus=(0x%x)", tucStatus);
    return asynError;
  } else if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to wait for frame, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }

  // Width & height are array dimensions
  nCols = frameHandle_.usWidth;
  nRows = frameHandle_.usHeight;

  // Image format
  pixelFormat = frameHandle_.ucFormat;
  // Number of channels
  channels = frameHandle_.ucChannels;
  // Bytes per pixel
  pixelBytes = frameHandle_.ucElemBytes;
  // Frame image data size
  tDataSize = frameHandle_.uiImgSize;

  /* There is zero documentation on what the formats mean
   * Most of the below is gleaned through trial and error */
  if (pixelFormat == TUFRM_FMT_RAW) {
    // Raw data - no filtering applied
    if (pixelBytes == 1) {
      dataType = NDUInt8;
      pixelSize = 1;
    } else if (pixelBytes == 2) {
      dataType = NDUInt16;
      pixelSize = 2;
    }
    colorMode = NDColorModeMono;
    numColors = 1;
  } else if (pixelFormat == TUFRM_FMT_USUAl) {
    if (pixelBytes == 1) {
      dataType = NDUInt8;
      pixelSize = 1;
    } else if (pixelBytes == 2) {
      dataType = NDUInt16;
      pixelSize = 2;
    }

    if (channels == 1) {
      colorMode = NDColorModeMono;
      numColors = 1;
    } else if (channels == 3) {
      colorMode = NDColorModeRGB1;
      numColors = 3;
    }
  } else if (pixelFormat == TUFRM_FMT_RGB888) {
    dataType = NDUInt8;
    pixelSize = 1;
    colorMode = NDColorModeRGB1;
    numColors = 3;
  } else {
    ERR_ARGS("Unsupported pixel format %d", pixelFormat);
    return asynError;
  }

  if (numColors == 1) {
    nDims = 2;
    dims[0] = nCols;
    dims[1] = nRows;
  } else {
    nDims = 3;
    dims[0] = 3;
    dims[1] = nCols;
    dims[2] = nRows;
  }

  dataSize = dims[0] * dims[1] * pixelSize;
  if (nDims == 3) dataSize *= dims[2];

  if (dataSize != tDataSize) {
    ERR_ARGS("data size mismatch: calculated=%zu, reported=%zu", dataSize,
             tDataSize);
    return asynError;
  }

  status |= setIntegerParam(NDArraySizeX, nCols);
  status |= setIntegerParam(NDArraySizeY, nRows);
  status |= setIntegerParam(NDArraySize, static_cast<int>(dataSize));
  status |= setIntegerParam(NDDataType, dataType);
  status |= setIntegerParam(NDColorMode, colorMode);
  if (status != asynSuccess) {
    ERR("Failed to set NDArray parameters");
  }

  // Copy frame to an NDArray buffer
  this->pRaw_ = this->pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
  if (!this->pRaw_) {
    ERR_ARGS(
        "[%s] Serious problem: not enough buffers left. Aborting acquisition!",
        portName);
    return asynError;
  }
  memcpy(this->pRaw_->pData,
         this->frameHandle_.pBuffer + this->frameHandle_.usOffset, dataSize);
  // Copy the header data from the frame
  // `frameHeader.dblTimeStamp` is in ms since the TUCAM_Cap_Start (it is the
  // acquire/exposure start time) `frameHeader.dblTimeLast` is in ms since the
  // frameHeader.dblTimeStamp (it is the acquire end time, NOT the exposure end
  // time) Precision is to nanoseconds
  memcpy(&frameHeader, this->frameHandle_.pBuffer, sizeof(TUCAM_IMG_HEADER));
  double timeStamp =
      startTimeStamp->secPastEpoch + (frameHeader.dblTimeStamp * 1.e-3);
  this->pRaw_->timeStamp = timeStamp;
  this->getAttributes(this->pRaw_->pAttributeList);
  this->pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32,
                                   &colorMode);

  INFO_ARGS("Grabbed image at %f s", timeStamp);

  return (asynStatus)status;
}

/**
 * @brief Allocates camera buffer and starts acquisition.
 *
 * We assume that the driver is locked when this function is called to avoid
 * camera changes from a concurrent thread.
 *
 * @returns asynSuccess if acquisition was successfully started, asynError
 * otherwise
 */
asynStatus ADTucam::startCapture() {
  static const char *functionName = "startCapture";
  int tucStatus;
  int triggerMode;

  getIntegerParam(ADTriggerMode, &triggerMode);
  tucStatus =
      sdkHandler_->startCapture(this->camHandle_.hIdxTUCam, triggerMode);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to start acquisition, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }

  INFO_ARGS("Capturing... triggerMode=%d", triggerMode);

  return asynSuccess;
}

/**
 * @brief Aborts & stops acquisition and releases camera buffer.
 *
 * @returns asynSuccess if acquisition was successfully stopped, asynError
 * otherwise
 */
asynStatus ADTucam::stopCapture() {
  static const char *functionName = "stopCapture";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = sdkHandler_->abortWait(camHandle_.hIdxTUCam);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to abort wait for frame, tucStatus=(0x%x)", tucStatus);
    status |= asynError;
  }

  tucStatus = sdkHandler_->stopCapture(camHandle_.hIdxTUCam);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to stop capturing, tucStatus=(0x%x)", tucStatus);
    status |= asynError;
  }

  INFO("Capture stopped");

  return (asynStatus)status;
}

/**
 * @brief Starts acquisition thread.
 *
 * @returns asynSuccess if acquisition thread was successfully started,
 * asynError otherwise
 */
asynStatus ADTucam::startAcquisition() {
  static const char *functionName = "startAcquisition";
  int status = asynSuccess;

  epicsEventSignal(this->startEventId_);

  INFO("Acquisition start event signaled");
  return (asynStatus)status;
}

/**
 * @brief Stops acquisition thread.
 *
 * @returns asynSuccess if acquisition thread was successfully stopped,
 * asynError otherwise
 */
asynStatus ADTucam::stopAcquisition() {
  static const char *functionName = "stopAcquisition";
  int status = asynSuccess;
  int tucStatus;

  /* Signal a forced stop */
  epicsEventSignal(this->stopEventId_);

  /* Abort the current frame */
  tucStatus = sdkHandler_->abortWait(this->camHandle_.hIdxTUCam);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to abort wait for frame, tucStatus=(0x%x)", tucStatus);
    status |= asynError;
  }

  INFO("Acquisition stop event signaled");

  return (asynStatus)status;
}

/**
 * @brief Connect to the camera and set the PVs with the camera information.
 *
 * @returns asynSuccess if camera was successfully connected, asynError
 * otherwise
 */
asynStatus ADTucam::connectCamera() {
  static const char *functionName = "connectCamera";
  int tucStatus;
  int status = asynSuccess;

  // Init API
  char szPath[1024] = {0};
  getcwd(szPath, 1024);
  apiHandle_.pstrConfigPath = szPath;
  apiHandle_.uiCamCount = 0;

  INFO("Initializing TUCAM API");
  tucStatus = sdkHandler_->initializeAPI(&apiHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("TUCAM API init failed, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }
  TUCAMInitialized++;
  if (apiHandle_.uiCamCount < 1) {
    ERR_ARGS("No camera detected, tucStatus=(0x%x)", tucStatus);
    sdkHandler_->uninitializeAPI();
    return asynError;
  }

  // Init camera
  camHandle_.hIdxTUCam = NULL;
  camHandle_.uiIdxOpen = cameraId_;

  INFO_ARGS("Opening camera device... cameraId_=%d", cameraId_);
  tucStatus = sdkHandler_->openDevice(&camHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to open camera device, tucStatus=(0x%x)", tucStatus);
    sdkHandler_->uninitializeAPI();
    return asynError;
  }

  // Set PVs from camera information
  INFO("Setting PVs from camera information");
  status |= setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
  status |= setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
  status |= setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
  status |= setCamInfo(ADTucam_Bus, TUIDI_BUS, 0);
  status |= setCamInfo(ADTucam_ProductID, TUIDI_PRODUCT, 1);
  status |= setCamInfo(ADMaxSizeX, TUIDI_CURRENT_WIDTH, 2);
  status |= setCamInfo(ADMaxSizeY, TUIDI_CURRENT_HEIGHT, 2);
  status |= setSerialNumber();
  status |= setCurrentROI();
  status |= this->setCurrentTrigger();
  triggerOutSupport_ = 1;
  for (int port = 0; port < 3; port++) {
    if (this->setCurrentTriggerOut(port) != asynSuccess) {
      WARN_ARGS("Trigger out %d not supported", port);
      triggerOutSupport_ = 0;
      break;
    }
  }

  INFO("Allocating initial frame buffer");
  /* Allocate initial frame buffer (need to re-allocate if resolution changes)
   */
  this->frameHandle_.uiRsdSize = 1;
  tucStatus = sdkHandler_->allocateBuffer(this->camHandle_.hIdxTUCam,
                                          &this->frameHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to allocate buffer, tucStatus=(0x%x)", tucStatus);
    sdkHandler_->closeDevice(this->camHandle_.hIdxTUCam);
    sdkHandler_->uninitializeAPI();
    return asynError;
  }

  INFO("Camera connected");
  return (asynStatus)status;
}

/**
 * @brief Disconnect from the camera. Assumes that the driver is not acquiring.
 *
 * @returns asynSuccess if camera was successfully disconnected, asynError
 * otherwise
 */
asynStatus ADTucam::disconnectCamera(void) {
  static const char *functionName = "disconnectCamera";
  int tucStatus;
  int status = asynSuccess;
  int capturing;

  /* If we have a camera open, stop it and close it */
  if (this->camHandle_.hIdxTUCam != NULL) {
    getIntegerParam(ADTucam_Capture, &capturing);
    if (capturing) {
      INFO("Capture is still active, stopping it");
      this->stopCapture();
      setIntegerParam(ADTucam_Capture, 0);
    }

    INFO("Releasing camera buffer");
    tucStatus = sdkHandler_->releaseBuffer(camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      status = asynError;
      ERR_ARGS("Unable to release camera buffer, tucStatus=(0x%x)", tucStatus);
    }

    INFO("Closing camera device");
    tucStatus = sdkHandler_->closeDevice(camHandle_.hIdxTUCam);
    if (tucStatus != TUCAMRET_SUCCESS) {
      status = asynError;
      ERR_ARGS("Unable close camera, tucStatus=(0x%x)\n", tucStatus);
    }
  }

  /* If that was the last camera, uninitialize the SDK */
  TUCAMInitialized--;
  if (TUCAMInitialized == 0) {
    INFO("Uninitializing TUCAM API");
    sdkHandler_->uninitializeAPI();
  }

  INFO("Camera disconnected");

  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs to match the camera's current trigger mode.
 *
 * @returns asynSuccess if trigger mode was successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCurrentTrigger() {
  static const char *functionName = "setCurrentTrigger";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = sdkHandler_->getTrigger(camHandle_.hIdxTUCam, &triggerHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to get trigger, status=%d", tucStatus);
    return asynError;
  }

  status = setIntegerParam(ADTriggerMode, triggerHandle_.nTgrMode);
  status = setIntegerParam(ADTucam_TriggerEdge, triggerHandle_.nEdgeMode);
  status = setIntegerParam(ADTucam_TriggerExposure, triggerHandle_.nExpMode);
  status =
      setDoubleParam(ADTucam_TriggerDelay, triggerHandle_.nDelayTm / 1.0e6);

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's trigger mode.
 *
 * @param triggerMode Trigger mode to set
 * @param triggerEdge Trigger edge to set
 * @param triggerExposure Trigger exposure to set
 * @param triggerDelay Trigger delay to set
 *
 * @returns asynSuccess if trigger mode was successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCameraTrigger(int triggerMode, int triggerEdge,
                                     int triggerExposure, double triggerDelay) {
  static const char *functionName = "setCameraTrigger";
  int status = asynSuccess;
  int tucStatus;

  frameHandle_.uiRsdSize = 1;

  triggerHandle_.nTgrMode = triggerMode;
  triggerHandle_.nEdgeMode = triggerEdge;
  triggerHandle_.nExpMode = triggerExposure;
  triggerHandle_.nFrames = 1;
  triggerHandle_.nDelayTm = static_cast<int>(triggerDelay * 1e6);

  INFO_ARGS(
      "Setting trigger... triggerMode=%d, triggerEdge=%d, triggerExposure=%d, "
      "triggerDelay=%f",
      triggerMode, triggerEdge, triggerExposure, triggerDelay);
  tucStatus = sdkHandler_->setTrigger(camHandle_.hIdxTUCam, triggerHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to set trigger, status=%d", tucStatus);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs using the camera's current trigger out settings.
 *
 * @param port Trigger out port to set
 *
 * @returns asynSuccess if trigger out settings were successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCurrentTriggerOut(int port) {
  static const char *functionName = "setCurrentTriggerOut";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = sdkHandler_->getTriggerOut(camHandle_.hIdxTUCam,
                                         &triggerOutHandle_[port]);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to get trigger out, port=%d, status=%d\n", port,
             tucStatus);
    return asynError;
  }

  if (port == 0) {
    status |= setIntegerParam(ADTucam_TriggerOut1Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut1Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut1Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut1Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  } else if (port == 1) {
    status |= setIntegerParam(ADTucam_TriggerOut2Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut2Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut2Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut2Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  } else if (port == 2) {
    status |= setIntegerParam(ADTucam_TriggerOut3Mode,
                              triggerOutHandle_[port].nTgrOutMode);
    status |= setIntegerParam(ADTucam_TriggerOut3Edge,
                              triggerOutHandle_[port].nEdgeMode);
    status |= setDoubleParam(ADTucam_TriggerOut3Delay,
                             triggerOutHandle_[port].nDelayTm / 1.0e6);
    status |= setDoubleParam(ADTucam_TriggerOut3Width,
                             triggerOutHandle_[port].nWidth / 1.0e6);
  }

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's trigger out settings.
 *
 * @param port Trigger out port to set
 * @param triggerMode Trigger mode to set
 * @param triggerEdge Trigger edge to set
 * @param triggerDelay Trigger delay to set
 * @param triggerWidth Trigger width to set
 *
 * @returns asynSuccess if trigger out settings were successfully set, asynError
 * otherwise
 */
asynStatus ADTucam::setCameraTriggerOut(int port, int triggerMode,
                                        int triggerEdge, double triggerDelay,
                                        double triggerWidth) {
  static const char *functionName = "setCameraTriggerOut";
  int status = asynSuccess;
  int tucStatus;

  INFO_ARGS(
      "Setting trigger out... port=%d, triggerMode=%d, triggerEdge=%d, "
      "triggerDelay=%f, triggerWidth=%f",
      port, triggerMode, triggerEdge, triggerDelay, triggerWidth);
  triggerOutHandle_[port].nTgrOutPort = port;
  triggerOutHandle_[port].nTgrOutMode = triggerMode;
  triggerOutHandle_[port].nEdgeMode = triggerEdge;
  triggerOutHandle_[port].nDelayTm = static_cast<int>(triggerDelay * 1e6);
  triggerOutHandle_[port].nWidth = static_cast<int>(triggerWidth * 1e6);

  tucStatus =
      sdkHandler_->setTriggerOut(camHandle_.hIdxTUCam, triggerOutHandle_[port]);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to set trigger out, port=%d, status=%d\n", port,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set EPICS PVs using the camera's current ROI settings.
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCurrentROI() {
  static const char *functionName = "setCurrentROI";
  int minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
  int tucStatus;
  int status = asynSuccess;

  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);

  TUCAM_ROI_ATTR roiAttr;
  tucStatus = sdkHandler_->getROI(camHandle_.hIdxTUCam, &roiAttr);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to get ROI, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }

  if (roiAttr.bEnable) {
    minX = roiAttr.nHOffset;
    minY = roiAttr.nVOffset;
    sizeX = roiAttr.nWidth;
    sizeY = roiAttr.nHeight;
  } else {
    minX = 0;
    minY = 0;
    sizeX = maxSizeX;
    sizeY = maxSizeY;
  }

  status |= setIntegerParam(ADMinX, minX);
  status |= setIntegerParam(ADMinY, minY);
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);

  return (asynStatus)status;
}

/**
 * @brief Read EPICS PVs to set the camera's ROI settings.
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCameraROI(int minX, int minY, int sizeX, int sizeY) {
  static const char *functionName = "setCameraROI";
  int status = asynSuccess;
  int tucStatus;
  int maxSizeX, maxSizeY;

  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);

  if (minX + sizeX > maxSizeX) {
    sizeX = maxSizeX - minX;
    WARN_ARGS("Size X is too large, setting to %d", sizeX);
  }
  if (minY + sizeY > maxSizeY) {
    sizeY = maxSizeY - minY;
    WARN_ARGS("Size Y is too large, setting to %d", sizeY);
  }

  TUCAM_ROI_ATTR roiAttr;
  roiAttr.bEnable = true;
  roiAttr.nHOffset = minX;
  roiAttr.nVOffset = minY;
  roiAttr.nWidth = sizeX;
  roiAttr.nHeight = sizeY;

  /* Release the current camera frame buffer */
  tucStatus = sdkHandler_->releaseBuffer(this->camHandle_.hIdxTUCam);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to release camera buffer, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }

  /* Set the new ROI */
  tucStatus = sdkHandler_->setROI(this->camHandle_.hIdxTUCam, roiAttr);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to set new ROI, tucStatus=(0x%x)", tucStatus);
    // Re-allocate the old buffer
    tucStatus = sdkHandler_->allocateBuffer(this->camHandle_.hIdxTUCam,
                                            &this->frameHandle_);
    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("Failed to re-allocate the old camera buffer, tucStatus=(0x%x)",
               tucStatus);
    }
    return asynError;
  }

  /* Allocate the new buffer */
  tucStatus = sdkHandler_->allocateBuffer(this->camHandle_.hIdxTUCam,
                                          &this->frameHandle_);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("Failed to allocate camera buffer, tucStatus=(0x%x)", tucStatus);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Get the camera information.
 * @param nID TUIDI option to read
 * @param sBuf Buffer to store the string information (must be at least 1024
 * bytes)
 * @param val Integer to store the value
 * @return asynStatus
 */
asynStatus ADTucam::getCamInfo(int nID, char *sBuf, int &val) {
  static const char *functionName = "getCamInfo";

  // Get camera information
  int tucStatus;
  TUCAM_VALUE_INFO valInfo;
  const int sSize = 1024;
  char sInfo[sSize] = {0};
  valInfo.pText = sInfo;
  valInfo.nTextSize = sSize;

  valInfo.nID = nID;
  tucStatus = sdkHandler_->getDeviceInfo(camHandle_.hIdxTUCam, &valInfo);
  if (tucStatus == TUCAMRET_SUCCESS) {
    val = valInfo.nValue;
    strncpy(sBuf, valInfo.pText, sSize - 1);
    sBuf[sSize - 1] = '\0';
    return asynSuccess;
  } else {
    ERR_ARGS("could not get %d (0x%x)", nID, tucStatus);
    return asynError;
  }
}

/**
 * @brief Set EPICS PVs using camera information.
 * @param param EPICS parameter to set
 * @param nID TUIDI option to read
 * @param dtype Data type of the parameter
 *              0 - string
 *              1 - double
 *              2 - integer
 * @return asynStatus
 */
asynStatus ADTucam::setCamInfo(int param, int nID, int dtype) {
  static const char *functionName = "setCamInfo";
  int status = asynSuccess;
  const int sSize = 1024;
  char sInfo[sSize] = {0};
  int nValue;

  status = this->getCamInfo(nID, sInfo, nValue);
  if (status == asynSuccess) {
    if (param == ADTucam_Bus) {
      if (nValue == 768) {
        status |= setStringParam(ADTucam_Bus, "USB3.0");
      } else {
        status |= setStringParam(ADTucam_Bus, "USB2.0");
      }
    } else if (dtype == 0) {
      status |= setStringParam(param, sInfo);
    } else if (dtype == 1) {
      status |= setDoubleParam(param, nValue);
    } else if (dtype == 2) {
      status |= setIntegerParam(param, nValue);
    }
  } else {
    ERR_ARGS("param %d id %d (error=0x%x)", param, nID, status);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Set EPICS PV using the camera's serial number.
 * @return asynStatus
 */
asynStatus ADTucam::setSerialNumber() {
  static const char *functionName = "setSerialNumber";
  int status = asynSuccess;
  int tucStatus;
  char cSN[TUSN_SIZE] = {0};
  TUCAM_REG_RW regRW;

  regRW.nRegType = TUREG_SN;
  regRW.pBuf = &cSN[0];
  regRW.nBufSize = TUSN_SIZE;

  tucStatus = sdkHandler_->readRegister(camHandle_.hIdxTUCam, regRW);
  if (tucStatus == TUCAMRET_SUCCESS) {
    setStringParam(ADSerialNumber, cSN);
  } else {
    ERR_ARGS("unable to get serial number (error=0x%x)", tucStatus);
    return asynError;
  }

  return (asynStatus)status;
}

/**
 * @brief Get the camera's property value.
 *
 * @param property TUIDP property to get
 * @param value Value to store the property value
 *
 * @return asynStatus
 */
asynStatus ADTucam::getProperty(int property, double *value) {
  static const char *functionName = "getProperty";
  int status = asynSuccess;
  int tucStatus;

  tucStatus =
      sdkHandler_->getPropertyValue(camHandle_.hIdxTUCam, property, value);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get property %d (0x%x)", property, tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set the camera's property value.
 *
 * @param property TUIDP property to set
 * @param value Value to set the property to
 *
 * @return asynStatus
 */
asynStatus ADTucam::setProperty(int property, double value) {
  static const char *functionName = "setProperty";
  int status = asynSuccess;
  int tucStatus;
  TUCAM_PROP_ATTR attrProp;

  attrProp.nIdxChn = 0;
  attrProp.idProp = property;
  tucStatus = sdkHandler_->getPropertyAttr(camHandle_.hIdxTUCam, &attrProp);
  if (tucStatus == TUCAMRET_SUCCESS) {
    INFO_ARGS("property value range [%f %f]", attrProp.dbValMin,
              attrProp.dbValMax);
    if (value < attrProp.dbValMin) {
      value = attrProp.dbValMin;
      WARN_ARGS("Clipping set min value: %d, %f", property, value);
    } else if (value > attrProp.dbValMax) {
      value = attrProp.dbValMax;
      WARN_ARGS("Clipping set max value: %d, %f\n", property, value);
    }
  }
  INFO_ARGS("Setting property %d to %f", property, value);

  tucStatus =
      sdkHandler_->setPropertyValue(camHandle_.hIdxTUCam, property, value);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to set property %d to %f (0x%x)", property, value,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Get the camera's capability value.
 *
 * @param property TUIDC capability to get
 * @param value Value to store the capability value
 *
 * @return asynStatus
 */
asynStatus ADTucam::getCapability(int property, int *val) {
  static const char *functionName = "getCapability";
  int status = asynSuccess;
  int tucStatus;

  tucStatus = sdkHandler_->getCapabilityValue(this->camHandle_.hIdxTUCam,
                                              property, val);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get capability %d=%d\n", property, *val);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Set the camera's capability value.
 *
 * @param property TUIDC capability to set
 * @param value Value to set the capability to
 *
 * @return asynStatus
 */
asynStatus ADTucam::setCapability(int property, int val) {
  static const char *functionName = "setCapability";
  int status = asynSuccess;
  int tucStatus;

  INFO_ARGS("Setting capability %d to %d", property, val);
  tucStatus = sdkHandler_->setCapabilityValue(this->camHandle_.hIdxTUCam,
                                              property, val);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to set capability %d=%d (0x%x)\n", property, val,
             tucStatus);
    return asynError;
  }
  return (asynStatus)status;
}

/**
 * @brief Reads the capability of the camera.
 *
 * @param property TUIDC capability to read
 * @param strings Array of strings to be filled with capability names
 * @param values Array of integers to be filled with capability values
 * @param severities Array of integers to be filled with capability severities
 * @param nElements Number of elements in the arrays
 * @param nIn Number of elements filled in the arrays
 *
 * @returns asynSuccess if read was successful, asynError otherwise
 */
asynStatus ADTucam::getCapabilityText(int property, char *strings[],
                                      int values[], int severities[],
                                      size_t nElements, size_t *nIn) {
  static const char *functionName = "getCapabilityText";
  int status = asynSuccess;
  int tucStatus;
  int i = 0;

  TUCAM_CAPA_ATTR attrCapa;
  tucStatus =
      sdkHandler_->getCapabilityAttr(this->camHandle_.hIdxTUCam, &attrCapa);
  if (tucStatus != TUCAMRET_SUCCESS) {
    ERR_ARGS("unable to get capability %d (0x%x)\n", property, tucStatus);
    *nIn = 0;
    return asynError;
  }

  for (i = 0; i < attrCapa.nValMax - attrCapa.nValMin + 1; i++) {
    char szRes[64] = {0};
    TUCAM_VALUE_TEXT valText;
    valText.dbValue = i;
    valText.nID = property;
    valText.nTextSize = 64;
    valText.pText = &szRes[0];

    tucStatus = sdkHandler_->getCapabilityValueText(this->camHandle_.hIdxTUCam,
                                                    &valText);

    if (tucStatus != TUCAMRET_SUCCESS) {
      ERR_ARGS("unable to get capability text %d:%d (0x%x)\n", property, i,
               tucStatus);
    }

    if (strings[i]) {
      free(strings[i]);
    }

    INFO_ARGS("Capability text: %s", valText.pText);

    strings[i] = epicsStrDup(valText.pText);
    values[i] = i + attrCapa.nValMin;
    severities[i] = 0;
  }

  *nIn = i;
  return (asynStatus)status;
}

#ifndef UNIT_TESTING
/* Code for iocsh registration */
/* ADTucamConfig */
static const iocshArg ADTucamConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADTucamConfigArg1 = {"Camera Id", iocshArgInt};
static const iocshArg *const ADTucamConfigArgs[] = {&ADTucamConfigArg0,
                                                    &ADTucamConfigArg1};

static const iocshFuncDef configTucam = {"ADTucamConfig", 2, ADTucamConfigArgs};

static void configTucamCallFunc(const iocshArgBuf *args) {
  ADTucamConfig(args[0].sval, args[1].ival);
}

static void ADTucamRegister(void) {
  iocshRegister(&configTucam, configTucamCallFunc);
}

extern "C" {
epicsExportRegistrar(ADTucamRegister);
}
#endif
