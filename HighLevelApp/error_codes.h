/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code.  They they must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum
{
    ExitCode_Success = 0,

    ExitCode_TermHandler_SigTerm = 1,

    ExitCode_Main_EventLoopFail = 2,

    ExitCode_ButtonTimer_Consume = 3,

    ExitCode_AzureTimer_Consume = 4,

    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_MessageButton = 6,
    ExitCode_Init_OrientationButton = 7,
    ExitCode_Init_TwinStatusLed = 8,
    ExitCode_Init_ButtonPollTimer = 9,
    ExitCode_Init_AzureTimer = 10,

    ExitCode_IsButtonPressed_GetValue = 11,

    ExitCode_Init_Socket = 12,
    ExitCode_SocketHandler_Recv = 13,
    ExitCode_Init_RegisterIo = 14,

    ExitCode_WriteProgramStateToMutableFile_OpenFile = 15,

    ExitCode_ReadProgramStateFromMutableFile_OpenFile = 16,

    ExitCode_ComputeTimeDifference_Fail = 17,

    ExitCode_WaitForUpdatesDownload_Consume = 18,

    ExitCode_WaitForUpdatesCheckTimer_Consume = 19,

    ExitCode_BusinessLogicTimer_Consume = 20,

    ExitCode_BlinkingTimer_Consume = 21,
    ExitCode_BlinkingTimer_SetValue = 22,

    ExitCode_UpdateCallback_SetBlinkPeriod = 23,
    ExitCode_UpdateCallback_SetValue = 24,
    ExitCode_UpdateCallback_Reboot = 25,
    ExitCode_UpdateCallback_GetUpdateEvent = 26,
    ExitCode_UpdateCallback_InvalidUpdateType = 27,
    ExitCode_UpdateCallback_UnexpectedEvent = 28,

    ExitCode_Powerdown_Fail = 29,

    ExitCode_Init_UpdateStartedTimer = 30,

    ExitCode_Init_RedLed = 31,
    ExitCode_Init_GreenLed = 32,
    ExitCode_Init_RegisterEvent = 33,
    ExitCode_Init_CreateBlinkingTimer = 34,
    ExitCode_Init_CreateBusinessLogicTimer = 35,
    ExitCode_Init_SetBusinessLogicTimer = 36,
    ExitCode_Init_CreateWaitForUpdatesCheckTimer = 37,
    ExitCode_Init_SetWaitForUpdatesCheckTimer = 38,
    ExitCode_Init_CreateWaitForUpdatesDownloadTimer = 39,
    ExitCode_Init_SetWaitForUpdatesDownloadTimer = 40,

    ExitCode_TriggerPowerdown_Success = 41,

    ExitCode_TriggerReboot_Success = 42,

    ExitCode_Init_VariableCheckerPollTimer = 43,
    ExitCode_VariableCheckerTimer_Consume = 44,
    ExitCode_SendMsg_Send = 45,

} ExitCode;