/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere demonstrates Azure IoT SDK C APIs
// The application uses the Azure IoT SDK C APIs to
// 1. Use the buttons to trigger sending telemetry to Azure IoT Hub/Central.
// 2. Use IoT Hub/Device Twin to control an LED.

// You will need to provide four pieces of information to use this application, all of which are set
// in the app_manifest.json.
// 1. The Scope Id for your IoT Central application (set in 'CmdArgs')
// 2. The Tenant Id obtained from 'azsphere tenant show-selected' (set in 'DeviceAuthentication')
// 3. The Azure DPS Global endpoint address 'global.azure-devices-provisioning.net'
//    (set in 'AllowedConnections')
// 4. The IoT Hub Endpoint address for your IoT Central application (set in 'AllowedConnections')

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>
#include <sys/socket.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>
#include <applibs/eventloop.h>
#include <applibs/application.h>
#include <applibs/powermanagement.h>
#include <applibs/sysevent.h>

// By default, this sample's CMake build targets hardware that follows the MT3620
// Reference Development Board (RDB) specification, such as the MT3620 Dev Kit from
// Seeed Studios.
//
// To target different hardware, you'll need to update the CMake build. The necessary
// steps to do this vary depending on if you are building in Visual Studio, in Visual
// Studio Code or via the command line.
//
// See https://github.com/Azure/azure-sphere-samples/tree/master/Hardware for more details.
//
// This #include imports the sample_hardware abstraction from that hardware definition.
#include "hw/avnet_mt3620_sk.h"

#include "eventloop_timer_utilities.h"
#include "build_options.h"

// Azure IoT SDK
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>
#include "device_twin.h"
#include "azure_iot_utilities.h"
#include "error_codes.h"
#include "parson.h" // used to parse Device Twin messages.


#define JSON_BUFFER_SIZE                            204
#define CLOUD_MSG_SIZE                              22

#define DEFAULT_SLEEPTIME                           120
#define DEFAULT_DOUBLETAPTHRES                      6 
#define DEFAULT_DOUBLETAPDETECTION                  true
#define DEFAULT_TIMEGAPDOUBLETAPRECOGNITION         7
#define DEFAULT_QUIETTIMEAFTERATAPDETECTION         3
#define DEFAULT_MAXIMUMDURATIONOFOVERTHRESHOLDEVENT 3

#define SCOPEID_LENGTH 20

enum
{
    RTsensor_data = 0,
    RTparameters_updated = 1,
    RTdouble_tap_detection = 2
};

/*************Structs*************/
typedef struct PowerDownParameters
{
    int cyclesSinceLastUpdateCheckComplete;
    int powerdownResidencyTime;
    bool doubleTapDetection;
    int doubleTapThreshold;
    int timeGapDoubleTapRecognition;
    int quietTimeAfterATapDetection;
    int maximumDurationOfOverthresholdEvent;
}powerDownParams;


/*************Variables*************/
volatile sig_atomic_t exitCode = ExitCode_Success;

// Azure IoT Hub/Central defines.

char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in
                                     // app_manifest.json, CmdArgs
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;
//static const int keepalivePeriodSeconds = 20;
extern bool iothubAuthenticated;

// File descriptors - initialized to invalid value
// Buttons
static int sendMessageButtonGpioFd = -1;
static int sendOrientationButtonGpioFd = -1;


bool DoubleTapDetection = true;
int DoubleTapThreshold = 6;
int TimeGapDoubleTapRecognition = 7;
int QuietTimeAfterATapDetection = 3;
int MaximumDurationOfOverthresholdEvent = 3;

// LED
uint8_t String_twin[32];

// The status mode LED shows whether the application completes its business logic (red)
// or waits for updates (green).
static int blinkingLedRedFd = -1;
static int waitingUpdatesLedGreenFd = -1;

static int sockFd = -1;
static EventRegistration* socketEventReg = NULL;
static const char rtAppComponentId[] = "6583cf17-d321-4d72-8283-0b7c5b56442b";

static float acceleration_mg[3];

// Timer / polling
static EventLoop* eventLoop = NULL;
static EventLoopTimer* buttonPollTimer = NULL;
static EventLoopTimer* azureTimer = NULL;
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static EventLoopTimer* sendDataTimer = NULL;
#endif
static EventRegistration* updateEventReg = NULL;
static EventLoopTimer* variablesChangePollTimer = NULL;

// The update state the system is in
SysEvent_Events currentUpdateState = SysEvent_Events_None;

// Azure IoT poll periods
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static const int AzureIoTDefaultPollPeriodSeconds = 0;
//static const int AzureIoTMinReconnectPeriodSeconds = 60;
//static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;
static int azureIoTPollPeriodSeconds = -1;
#endif

// Button state variables
static GPIO_Value_Type sendMessageButtonState = GPIO_Value_High;
static GPIO_Value_Type sendOrientationButtonState = GPIO_Value_High;

// The maximum number of times the device can go into powerdown mode without waiting extra time
// for an update to happen
static const unsigned int MaxCyclesUntilAllowUpdateCheckComplete = 4;

// The number of cycles since the application is running
static int cyclesSinceLastUpdateCheckComplete = 4;

powerDownParams power_down_par;

// The red LED will blink for 60 seconds and then the application will power down, unless it needs
// to wait for update-related processing.
static EventLoopTimer* businessLogicCompleteTimer = NULL;
static struct timespec businessLogicCompleteTimerInterval = { .tv_sec = 60, .tv_nsec = 0 };

// Wait extra time to check for updates
static EventLoopTimer* waitForUpdatesCheckTimer = NULL;
static struct timespec waitForUpdatesCheckTimerInterval = { .tv_sec = 120, .tv_nsec = 0 };

// Wait extra time for the download to finish
static EventLoopTimer* waitForUpdatesToDownloadTimer = NULL;
static const struct timespec waitForUpdatesToDownloadTimerInterval = { .tv_sec = 300, .tv_nsec = 0 };
static const struct timespec blinkIntervalWaitForUpdates = { .tv_sec = 0,
                                                            .tv_nsec = 500 * 1000 * 1000 };

static bool deviceIsUp = false; // Orientation

// This constant defines the maximum time (in seconds) the device can be in powerdown mode. A value
// of less than 2 seconds will cause the device to resume from powerdown immediately, behaving like
// a reboot.
int powerdownResidencyTime = 10;

// By default, the system doesn't wait for updates
static bool isBusinessLogicComplete = false;
static EventLoopTimer* blinkTimer = NULL;
static const struct timespec blinkIntervalBusinessLogic = { .tv_sec = 0,
                                                           .tv_nsec = 125 * 1000 * 1000 };
static GPIO_Value_Type ledState = GPIO_Value_High;



/*************Function prototypes*************/
// Azure IoT Hub/Central
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
//static void ReportStatusCallback(int result, void *context);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
#endif
static void SendMessageToRTCore(void);

// Function to send telemetry data
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void SendSensorData(void);
#endif

// Initialization/Cleanup
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);


//static void SendTimerEventHandler(EventLoopTimer* timer);
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context);


static void UpdateCallback(SysEvent_Events event, SysEvent_Status status, const SysEvent_Info* info,
    void* context);

static void ReadProgramStateFromMutableFile(void);
static void WriteProgramStateToMutableFile(void);

static void ButtonPollTimerEventHandler(EventLoopTimer *timer);
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState);
static void SendMessageButtonHandler(void);
static void SendOrientationButtonHandler(void);

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void AzureTimerEventHandler(EventLoopTimer *timer);
static void SendDataTimerEventHandler(EventLoopTimer* timer);
#endif

static void BusinessLogicTimerEventHandler(EventLoopTimer* timer);

static void WaitForUpdatesCheckTimerEventHandler(EventLoopTimer* timer);

static void WaitForUpdatesDownloadTimerEventHandler(EventLoopTimer* timer);

static void BlinkingLedTimerEventHandler(EventLoopTimer* timer);

static void VariablesChangePollTimerEventHandler(EventLoopTimer* timer);

static void TriggerReboot(void);
static void TriggerPowerdown(void);

static void SwitchOffLeds(void);

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("IoT Hub/Central Application starting.\n");

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) 
    {
        Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
    }


#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
    if (argc == 2)
    {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    }
    else
    {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }
#endif 

    exitCode = InitPeripheralsAndHandlers();

    // Main loop
    while (exitCode == ExitCode_Success)
    {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) 
        {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    ClosePeripheralsAndHandlers();

    if (exitCode == ExitCode_TriggerPowerdown_Success)
    {
        exitCode = ExitCode_Success;
        TriggerPowerdown();
    }
    else if (exitCode == ExitCode_TriggerReboot_Success)
    {
        exitCode = ExitCode_Success;
        TriggerReboot();
    }


    return exitCode;
}

/// <summary>
/// Button timer event:  Check the status of buttons A and B
/// </summary>
static void ButtonPollTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }
    SendMessageButtonHandler();
    SendOrientationButtonHandler();
}

/// <summary>
/// Azure timer event:  Check connection status and send telemetry
/// </summary>
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void AzureTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_AzureTimer_Consume;
        return;
    }



    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) 
    {
        if (isNetworkReady && !iothubAuthenticated) 
        {
            
            //SetupAzureClient();
            AzureIoT_SetupClient();
        }
    }
    else
    {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated)
    {
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
        
    }

}

/// <summary>
/// Send data timer event:  Send data to the cloud
/// </summary>
static void SendDataTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }
    SendSensorData();
}
#endif 
/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>ExitCode_Success if all resources were allocated successfully; otherwise another
/// ExitCode value which indicates the specific failure.</returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL)
    {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    // Read and update the values thta have to persist after low power mode
    ReadProgramStateFromMutableFile();
    cyclesSinceLastUpdateCheckComplete++;

    // If powerdownResidencyTime is zero, the mutable file was not found, set default values
    if (power_down_par.powerdownResidencyTime == 0)
    {
        powerdownResidencyTime = DEFAULT_SLEEPTIME;
        power_down_par.powerdownResidencyTime = powerdownResidencyTime;
        DoubleTapThreshold = DEFAULT_DOUBLETAPTHRES;
        power_down_par.doubleTapThreshold = DoubleTapThreshold;
        DoubleTapDetection = DEFAULT_DOUBLETAPDETECTION;
        power_down_par.doubleTapDetection = DoubleTapDetection;
        TimeGapDoubleTapRecognition = DEFAULT_TIMEGAPDOUBLETAPRECOGNITION;
        power_down_par.timeGapDoubleTapRecognition = TimeGapDoubleTapRecognition;
        QuietTimeAfterATapDetection = DEFAULT_QUIETTIMEAFTERATAPDETECTION;
        power_down_par.quietTimeAfterATapDetection = QuietTimeAfterATapDetection;
        MaximumDurationOfOverthresholdEvent = DEFAULT_MAXIMUMDURATIONOFOVERTHRESHOLDEVENT;
        power_down_par.maximumDurationOfOverthresholdEvent = MaximumDurationOfOverthresholdEvent;
    }
    else
    {
        powerdownResidencyTime = power_down_par.powerdownResidencyTime;
        DoubleTapThreshold = power_down_par.doubleTapThreshold;
        DoubleTapDetection = power_down_par.doubleTapDetection;
        TimeGapDoubleTapRecognition = power_down_par.timeGapDoubleTapRecognition;
        QuietTimeAfterATapDetection = power_down_par.quietTimeAfterATapDetection;
        MaximumDurationOfOverthresholdEvent = power_down_par.maximumDurationOfOverthresholdEvent;
    }

    
    // Open connection to real-time capable application.
    sockFd = Application_Connect(rtAppComponentId);
    if (sockFd == -1)
    {
        Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_Socket;
    }

    // Register handler for incoming messages from real-time capable application.
    socketEventReg = EventLoop_RegisterIo(eventLoop, sockFd, EventLoop_Input, SocketEventHandler,
        /* context */ NULL);
    if (socketEventReg == NULL)
    {
        Log_Debug("ERROR: Unable to register socket event: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_RegisterIo;
    }

    // Send data to RT core, configure it with data read from mutable storage
    SendMessageToRTCore();

    // Open LEDs for accept mode status.
    blinkingLedRedFd =
        GPIO_OpenAsOutput(AVNET_MT3620_SK_USER_LED_RED, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (blinkingLedRedFd < 0)
    {
        Log_Debug("ERROR: Could not open start red LED: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_RedLed;
    }

    waitingUpdatesLedGreenFd =
        GPIO_OpenAsOutput(AVNET_MT3620_SK_USER_LED_GREEN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (waitingUpdatesLedGreenFd < 0)
    {
        Log_Debug("ERROR: Could not open check for updates green LED: %s (%d).\n", strerror(errno),
            errno);
        return ExitCode_Init_GreenLed;
    }

    // Open button A GPIO as input
    Log_Debug("Opening SAMPLE_BUTTON_1 as input\n");
    sendMessageButtonGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
    if (sendMessageButtonGpioFd < 0)
    {
        Log_Debug("ERROR: Could not open button A: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_MessageButton;
    }

    // Open button B GPIO as input
    Log_Debug("Opening SAMPLE_BUTTON_2 as input\n");
    sendOrientationButtonGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_B);
    if (sendOrientationButtonGpioFd < 0)
    {
        Log_Debug("ERROR: Could not open button B: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_OrientationButton;
    }

    // Set up a timer to poll for button events.
    static const struct timespec buttonPressCheckPeriod = {.tv_sec = 0, .tv_nsec = 1000 * 1000};
    buttonPollTimer = CreateEventLoopPeriodicTimer(eventLoop, &ButtonPollTimerEventHandler,
                                                   &buttonPressCheckPeriod);
    if (buttonPollTimer == NULL)
    {
        return ExitCode_Init_ButtonPollTimer;
    }

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 500000000};
    azureTimer =
        CreateEventLoopPeriodicTimer(eventLoop, &AzureTimerEventHandler, &azureTelemetryPeriod);
    if (azureTimer == NULL)
    {
        return ExitCode_Init_AzureTimer;
    }



    struct timespec sendDataPeriod = { .tv_sec = 5, .tv_nsec = 0 };
    sendDataTimer = CreateEventLoopPeriodicTimer(eventLoop, &SendDataTimerEventHandler, &sendDataPeriod);
    if (sendDataTimer == NULL)
    {
        return ExitCode_Init_AzureTimer;
    }

    AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);
#endif 

    updateEventReg = SysEvent_RegisterForEventNotifications(eventLoop, SysEvent_Events_Mask,
        UpdateCallback, NULL);
    if (updateEventReg == NULL)
    {
        Log_Debug("ERROR: could not register update event: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_RegisterEvent;
    }

    blinkTimer = CreateEventLoopPeriodicTimer(eventLoop, &BlinkingLedTimerEventHandler,
        &blinkIntervalBusinessLogic);
    if (blinkTimer == NULL)
    {
        return ExitCode_Init_CreateBlinkingTimer;
    }

    businessLogicCompleteTimer =
        CreateEventLoopDisarmedTimer(eventLoop, &BusinessLogicTimerEventHandler);
    if (businessLogicCompleteTimer == NULL)
    {
        return ExitCode_Init_CreateBusinessLogicTimer;
    }
    int result =
        SetEventLoopTimerOneShot(businessLogicCompleteTimer, &businessLogicCompleteTimerInterval);
    if (result != 0)
    {
        return ExitCode_Init_SetBusinessLogicTimer;
    }

    waitForUpdatesCheckTimer =
        CreateEventLoopDisarmedTimer(eventLoop, &WaitForUpdatesCheckTimerEventHandler);
    if (waitForUpdatesCheckTimer == NULL)
    {
        return ExitCode_Init_CreateWaitForUpdatesCheckTimer;
    }
    result = SetEventLoopTimerOneShot(waitForUpdatesCheckTimer, &waitForUpdatesCheckTimerInterval);
    if (result != 0)
    {
        exitCode = ExitCode_Init_SetWaitForUpdatesCheckTimer;
    }

    waitForUpdatesToDownloadTimer =
        CreateEventLoopDisarmedTimer(eventLoop, &WaitForUpdatesDownloadTimerEventHandler);
    if (waitForUpdatesToDownloadTimer == NULL)
    {
        return ExitCode_Init_CreateWaitForUpdatesDownloadTimer;
    }
    result = SetEventLoopTimerOneShot(waitForUpdatesToDownloadTimer,
        &waitForUpdatesToDownloadTimerInterval);

    // Set up a timer to poll for variables change.
    static const struct timespec variablesChangeCheckPeriod = { .tv_sec = 0, .tv_nsec = 1000 * 1000 };
    variablesChangePollTimer = CreateEventLoopPeriodicTimer(eventLoop, &VariablesChangePollTimerEventHandler,
        &variablesChangeCheckPeriod);
    if (buttonPollTimer == NULL)
    {
        return ExitCode_Init_VariableCheckerPollTimer;
    }

    return ExitCode_Success;
}

/// <summary>
///     Closes a file descriptor and prints an error on failure.
/// </summary>
/// <param name="fd">File descriptor to close</param>
/// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0)
        {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(azureTimer);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors\n");

    CloseFdAndPrintError(sendMessageButtonGpioFd, "SendMessageButton");
    CloseFdAndPrintError(sendOrientationButtonGpioFd, "SendOrientationButton");
}

/// <summary>
///     Sends telemetry to IoT Hub
/// </summary>
/// <param name="key">The telemetry item to update</param>
/// <param name="value">new telemetry value</param>
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void SendTelemetry(const unsigned char *key, const unsigned char *value)
{
    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;

    Log_Debug("Sending IoT Hub Message: %s\n", eventBuffer);

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Cannot send IoTHubMessage because network is not up.\n");
        return;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(eventBuffer);

    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
                                             /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }

    IoTHubMessage_Destroy(messageHandle);
}
#endif

/// <summary>
///     Callback confirming message delivered to IoT Hub.
/// </summary>
/// <param name="result">Message delivery status</param>
/// <param name="context">User specified context</param>
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Message received by IoT Hub. Result is: %d\n", result);
}

/// <summary>
///     Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
/// </summary>
static void ReportStatusCallback(int result, void *context)
{
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}

/// <summary>
///     Generates a simulated Temperature and sends to IoT Hub.
/// </summary>
void SendSensorData(void)
{
    // Allocate memory for a telemetry message to Azure
    char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
    if (pjsonBuffer == NULL)
    {
        Log_Debug("ERROR: not enough memory to send telemetry");
    }

    snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"aX\": \"%.2f\", \"aY\": \"%.2f\", \"aZ\": \"%.2f\"}",
        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

    Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
    AzureIoT_SendMessage(pjsonBuffer);

    free(pjsonBuffer);

}
#endif

/// <summary>
///     Check whether a given button has just been pressed.
/// </summary>
/// <param name="fd">The button file descriptor</param>
/// <param name="oldState">Old state of the button (pressed or released)</param>
/// <returns>true if pressed, false otherwise</returns>
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState)
{
    bool isButtonPressed = false;
    GPIO_Value_Type newState;
    int result = GPIO_GetValue(fd, &newState);
    if (result != 0) 
    {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_IsButtonPressed_GetValue;
    }
    else 
    {
        // Button is pressed if it is low and different than last known state.
        isButtonPressed = (newState != *oldState) && (newState == GPIO_Value_Low);
        *oldState = newState;
    }

    return isButtonPressed;
}

/// <summary>
/// Pressing button A will:
///     Send a 'Button Pressed' event to Azure IoT Central
/// </summary>
static void SendMessageButtonHandler(void)
{
    if (IsButtonPressed(sendMessageButtonGpioFd, &sendMessageButtonState))
    {
        Log_Debug("Button A pressed\n");
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
        SendTelemetry("ButtonPress", "True");
#endif 
    }
}

/// <summary>
/// Pressing button B will:
///     Send an 'Orientation' event to Azure IoT Central
/// </summary>
static void SendOrientationButtonHandler(void)
{
    if (IsButtonPressed(sendOrientationButtonGpioFd, &sendOrientationButtonState))
    {
        Log_Debug("Button B pressed\n");
        deviceIsUp = !deviceIsUp;
#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
        SendTelemetry("Orientation", deviceIsUp ? "Up" : "Down");
#endif
    }
}

/// <summary>
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
{
    // Read response from real-time capable application.
    char rxBuf[32];
    int bytesReceived = recv(fd, rxBuf, sizeof(rxBuf), 0);

    if (bytesReceived == -1)
    {
        Log_Debug("ERROR: Unable to receive message: %d (%s)\n", errno, strerror(errno));
        exitCode = ExitCode_SocketHandler_Recv;
        return;
    }

    switch (rxBuf[0])
    {
        case RTsensor_data: // Sensor data received from RT core
        {
            acceleration_mg[0] = *((float*)(&rxBuf[1]));
            acceleration_mg[1] = *((float*)(&rxBuf[5]));
            acceleration_mg[2] = *((float*)(&rxBuf[9]));

            Log_Debug("Acceleration[mg] : %f, %f, %f", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        }
        break;
        case RTparameters_updated: // Parameters were updated in RT core
        {
            Log_Debug("Received %d bytes (from %s): ", bytesReceived, fd == sockFd ? "CM4_A" : "CM4_B");
            Log_Debug("%s", &rxBuf[1]);
        }
        break;
        case RTdouble_tap_detection: // A double tap was detected
        {
            Log_Debug("Received %d bytes (from %s): ", bytesReceived, fd == sockFd ? "CM4_A" : "CM4_B");
            Log_Debug("%s", &rxBuf[1]);
            Log_Debug("\n");

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
            SendTelemetry("DoubleTap", "True");
#endif
        }
        break;
        default:
        {

        }
        break;
    }
    Log_Debug("\n");
}


/// <summary>
///     Write cyclesSinceLastUpdateCheckComplete to the persistent data file.
/// </summary>
static void WriteProgramStateToMutableFile(void)
{
    int fd = Storage_OpenMutableFile();
    if (fd < 0)
    {
        Log_Debug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_WriteProgramStateToMutableFile_OpenFile;
        return;
    }

    power_down_par.cyclesSinceLastUpdateCheckComplete = cyclesSinceLastUpdateCheckComplete;

    ssize_t ret =
        write(fd, &power_down_par, sizeof(power_down_par));
    Log_Debug("INFO: Wrote cyclesSinceLastUpdateCheckComplete = %lu\n",
        cyclesSinceLastUpdateCheckComplete);
    if (ret == -1)
    {
        // If the file has reached the maximum size specified in the application manifest,
        // then -1 will be returned with errno EDQUOT (122)
        Log_Debug("ERROR: An error occurred while writing to mutable file:  %s (%d).\n",
            strerror(errno), errno);
    }
    else if (ret < sizeof(power_down_par))
    {
        // For simplicity, this sample logs an error here. In the general case, this should be
        // handled by retrying the write with the remaining data until all the data has been
        // written.
        Log_Debug("ERROR: Only wrote %zd of %zu bytes requested\n", ret,
            sizeof(power_down_par));
    }

    close(fd);
}

/// <summary>
///     Read cyclesSinceLastUpdateCheckComplete from the persistent data file. If the file doesn't
///     exist, set cyclesSinceLastUpdateCheckComplete to 0.
/// </summary>
static void ReadProgramStateFromMutableFile(void)
{
    int fd = Storage_OpenMutableFile();
    if (fd == -1)
    {
        Log_Debug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_ReadProgramStateFromMutableFile_OpenFile;
        return;
    }

    cyclesSinceLastUpdateCheckComplete = 0;
    ssize_t ret =
        read(fd, &power_down_par, sizeof(power_down_par));
    cyclesSinceLastUpdateCheckComplete = power_down_par.cyclesSinceLastUpdateCheckComplete;
    Log_Debug("INFO: Read cyclesSinceLastUpdateCheckComplete = %lu\n",
        cyclesSinceLastUpdateCheckComplete);

    if (ret == -1)
    {
        Log_Debug(
            "ERROR: An error occurred while reading cyclesSinceLastUpdateCheckComplete:  %s "
            "(%d).\n",
            strerror(errno), errno);
    }
    close(fd);

    if (ret < sizeof(power_down_par))
    {
        cyclesSinceLastUpdateCheckComplete = 0;
    }
}

/// <summary>
///     Waits for updates to download.
/// </summary>
static void WaitForUpdatesDownloadTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_WaitForUpdatesDownload_Consume;
        return;
    }

    Log_Debug("INFO: Wait for update download timed out. Powering down.\n");

    exitCode = ExitCode_TriggerPowerdown_Success;
}

/// <summary>
///     Waits for an update check to happen.
/// </summary>
static void WaitForUpdatesCheckTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_WaitForUpdatesCheckTimer_Consume;
        return;
    }

    if (currentUpdateState == SysEvent_Events_UpdateStarted)
    {
        return;
    }

    Log_Debug(
        "INFO: Wait for update check timed out, and no update download in progress. Powering "
        "down.\n");

    exitCode = ExitCode_TriggerPowerdown_Success;
}

/// <summary>
///     If the waiting time has expired and there are no updates downloading put the system in
///     powerdown mode. Otherwise wait for updates before going into powerdown mode.
/// </summary>
static void BusinessLogicTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_BusinessLogicTimer_Consume;
        return;
    }

    Log_Debug("INFO: Finished business logic.\n");
    isBusinessLogicComplete = true;

    if (currentUpdateState == SysEvent_Events_UpdateStarted ||
        cyclesSinceLastUpdateCheckComplete >= MaxCyclesUntilAllowUpdateCheckComplete)
    {
        return;
    }

    exitCode = ExitCode_TriggerPowerdown_Success;
}

/// <summary>
///     Handle LED timer event: blink LED.
/// </summary>
static void BlinkingLedTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_BlinkingTimer_Consume;
        return;
    }

    // The blink interval has elapsed, so toggle the LED state
    // The LED is active-low so GPIO_Value_Low is on and GPIO_Value_High is off
    ledState = (ledState == GPIO_Value_Low ? GPIO_Value_High : GPIO_Value_Low);
    int ledFd = isBusinessLogicComplete ? waitingUpdatesLedGreenFd : blinkingLedRedFd;
    int result = GPIO_SetValue(ledFd, ledState);
    if (result != 0)
    {
        Log_Debug("ERROR: Could not set LED output value: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_BlinkingTimer_SetValue;
        return;
    }
}

/// <summary>
///     This function matches the SysEvent_EventsCallback signature, and is invoked
///     from the event loop when the system wants to perform an application or system update.
///     See <see cref="SysEvent_EventsCallback" /> for information about arguments.
/// </summary>
static void UpdateCallback(SysEvent_Events event, SysEvent_Status status, const SysEvent_Info* info,
    void* context)
{
    // Update current state
    currentUpdateState = event;

    switch (event)
    {
        case SysEvent_Events_NoUpdateAvailable: {
            Log_Debug("INFO: Update check finished. No updates available\n");
            cyclesSinceLastUpdateCheckComplete = 0;
            if (isBusinessLogicComplete)
            {
                exitCode = ExitCode_TriggerPowerdown_Success;
            }
            break;
        }

        // Downloading updates has started. Change the blink interval to indicate this event has
        // occured, and keep waiting.
        case SysEvent_Events_UpdateStarted: {
            Log_Debug("INFO: Updates have started downloading\n");
            if (SetEventLoopTimerPeriod(blinkTimer, &blinkIntervalWaitForUpdates) != 0)
            {
                exitCode = ExitCode_UpdateCallback_SetBlinkPeriod;
            }
            break;
        }

        // Updates are ready for install
        case SysEvent_Events_UpdateReadyForInstall: {
            Log_Debug("INFO: Update download finished and is ready for install.\n");

            // Stop LED blinking, and switch on the green LED, to indicate this event has occured.
            DisarmEventLoopTimer(blinkTimer);
            SwitchOffLeds();

            int result = GPIO_SetValue(waitingUpdatesLedGreenFd, GPIO_Value_Low);
            if (result == -1)
            {
                Log_Debug("ERROR: GPIO_SetValue failed: %s (%d).\n", strerror(errno), errno);
                exitCode = ExitCode_UpdateCallback_SetValue;
                break;
            }

            SysEvent_Info_UpdateData data;
            result = SysEvent_Info_GetUpdateData(info, &data);
            if (result == -1)
            {
                Log_Debug("ERROR: SysEvent_Info_GetUpdateData failed: %s (%d).\n", strerror(errno),
                    errno);
                exitCode = ExitCode_UpdateCallback_GetUpdateEvent;
                break;
            }

            if (data.update_type == SysEvent_UpdateType_App)
            {
                Log_Debug("INFO: Application update. The device will powerdown.\n");
                cyclesSinceLastUpdateCheckComplete = 0;
                exitCode = ExitCode_TriggerPowerdown_Success;
            }
            else if (data.update_type == SysEvent_UpdateType_System)
            {
                Log_Debug("INFO: System update. The device will reboot.\n");
                exitCode = ExitCode_TriggerReboot_Success;
            }
            else
            {
                exitCode = ExitCode_UpdateCallback_InvalidUpdateType;
                Log_Debug("ERROR: ExitCode_UpdateCallback_InvalidUpdateType.\n");
            }

            break;
        }

        default:
            Log_Debug("ERROR: Unexpected event\n");
            exitCode = ExitCode_UpdateCallback_UnexpectedEvent;
            break;
    }

    Log_Debug("\n");
}

/// <summary>
///     Reboot the device.
/// </summary>
static void TriggerReboot(void)
{
    // Reboot the system
    int result = PowerManagement_ForceSystemReboot();
    if (result != 0)
    {
        Log_Debug("Error PowerManagement_ForceSystemReboot: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_UpdateCallback_Reboot;
    }
}

/// <summary>
///     Power down the device.
/// </summary>
static void TriggerPowerdown(void)
{
    WriteProgramStateToMutableFile();

    // Put the device in the powerdown mode
    int result = PowerManagement_ForceSystemPowerDown((unsigned int)powerdownResidencyTime);
    if (result != 0)
    {
        Log_Debug("Error PowerManagement_ForceSystemPowerDown: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_Powerdown_Fail;
    }
}


/// <summary>
///     Switch off the Leds.
/// </summary>
static void SwitchOffLeds(void)
{
    if (blinkingLedRedFd != -1)
    {
        GPIO_SetValue(blinkingLedRedFd, GPIO_Value_High);
    }

    if (waitingUpdatesLedGreenFd != -1)
    {
        GPIO_SetValue(waitingUpdatesLedGreenFd, GPIO_Value_High);
    }
}

/// <summary>
/// Variables change timer event:  Check if variables have changed from the cloud
/// </summary>
static void VariablesChangePollTimerEventHandler(EventLoopTimer* timer)
{
    bool was_data_updated = false;
    if (ConsumeEventLoopTimerEvent(timer) != 0)
    {
        exitCode = ExitCode_VariableCheckerTimer_Consume;
        return;
    }

    // Check if variables have changed 
    if (powerdownResidencyTime != power_down_par.powerdownResidencyTime)
    {
        power_down_par.powerdownResidencyTime = powerdownResidencyTime;
    }

    if (DoubleTapThreshold != power_down_par.doubleTapThreshold)
    {
        power_down_par.doubleTapThreshold = DoubleTapThreshold;
        was_data_updated = true;
    }

    if (DoubleTapDetection != power_down_par.doubleTapDetection)
    {
        power_down_par.doubleTapDetection = DoubleTapDetection;
        was_data_updated = true;
    }

    if (TimeGapDoubleTapRecognition != power_down_par.timeGapDoubleTapRecognition)
    {
        power_down_par.timeGapDoubleTapRecognition = TimeGapDoubleTapRecognition;
        was_data_updated = true;
    }
    
    if (QuietTimeAfterATapDetection != power_down_par.quietTimeAfterATapDetection)
    {
        power_down_par.quietTimeAfterATapDetection = QuietTimeAfterATapDetection;
        was_data_updated = true;
    }
    
    if (MaximumDurationOfOverthresholdEvent != power_down_par.maximumDurationOfOverthresholdEvent)
    {
        power_down_par.maximumDurationOfOverthresholdEvent = MaximumDurationOfOverthresholdEvent;
        was_data_updated = true;
    }
    

    if (was_data_updated == true)
    {
        // Send updated varibles to RT core
        SendMessageToRTCore();
    }
}

/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void SendMessageToRTCore(void)
{
    uint8_t txMessage[32];

    txMessage[0] = (uint8_t)power_down_par.doubleTapDetection;
    txMessage[1] = (uint8_t)power_down_par.doubleTapThreshold;
    txMessage[2] = (uint8_t)((power_down_par.timeGapDoubleTapRecognition & 0x0f) << 4 |
                   (power_down_par.quietTimeAfterATapDetection & 0x03) << 2 |
                   (power_down_par.maximumDurationOfOverthresholdEvent & 0x03));

    Log_Debug("Sending data to CM4_A\n", txMessage);

    int bytesSent = send(sockFd, txMessage, 3, 0);
    if (bytesSent == -1)
    {
        Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
        exitCode = ExitCode_SendMsg_Send;
        return;
    }
}