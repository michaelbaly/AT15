/******************************************************************************
 * Copyright (c) 2017-2019 M-Labs Technologies LLC.  All rights reserved.
 *
 * M-Labs Technologies is supplying this software for use solely and
 * exclusively with M-Labs Technologies tracker products.
 *
 * This software or documentation or copies thereof are not to be
 * distributed, rented, sub-licensed or otherwise made available to others,
 * except as expressly agreed upon in writing by M-Labs Technologies.
 *
 * You may not remove this copyright notice from this software.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE.
 * M-LABS TECHNOLOGIES SHALL NOT, UNDER ANY CIRCUMSTANCES, BE LIABLE FOR
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * ----------------------------------------------------------------------------
 *
 * File:    mnt_picif.c
 * Purpose: Interface layer between baseband and the PIC I/O processor
 *
 * Author:  BD
 *
 ******************************************************************************/

#include "prot.h"
#include "monet.h"
#include "mnt_inc.h"
#include "mnt_company.h"
#include "md_lis.h"
#include "mnt_debug.h"
#include "mnt_power.h"
#include "mnt_temperature.h"
#include "core_utils.h"
#include "mnt_picif.h"
#include "core_nordic_dfu.h"
#include "mnt_timer.h"
#include "core_abuffer.h"
#ifdef SUPPORT_BLE_DFU // MNT-2624
    #include "mnt_hwble.h"
    #include "mnt_comm_mux_util.h"
#endif
#if defined(_MINITRACK)
	#include "pnt_app.h"
#endif
#if defined SUPPORT_AC61_CAMERA || defined SUPPORT_ZAZU_CAMERA
    #include "mnt_camera.h"
#endif
#if defined NORDIC_DFU || defined SUPPORT_BLE_DFU
#include "core_nordic_dfu.h"
#endif
#include "core_sequencer.h"

static uint32	gInterruptStatus    = 0;
static bool     gIsWarmWakeup       = FALSE;
static bool     peripheralPowerOn   = false;

#if defined(_TRAILER) && defined(SUPPORT_BLE_DFU)
static int32_t  update_peri_dev_type = -1;
static update_type_e srcUpdateType   = UPDATE_TYPE_NONE;    // MNT-2953
#endif

// MNT-2401, MNT-2501
#ifdef _MINITRACK
#define IS_COLLISION_ENABLED() ((gMonetData.LightCrashDetectionThreshold | gMonetData.HeavyCrashDetectionThreshold) > 0)
#else
#define IS_COLLISION_ENABLED() (gMonetData.CollisionEnable && ((gMonetData.LightCrashDetectionThreshold | gMonetData.HeavyCrashDetectionThreshold) > 0))
#endif

#ifdef _TRAILER
static int   hardResetHandle = CORE_SEQ_INVALID_HANDLE;     // MNT-3024

static void mnt_HardResetRetrySeqHandler(uint32_t param);   // MNT-3024
static void mnt_startHardResetRetrySeq(uint32_t delay);     // MNT-3024
static void mnt_stopHardResetRetrySeq(void);                // MNT-3024

static core_seqPhase_ts hardResetSeq[] = {                  // MNT-3024
    { mnt_HardResetRetrySeqHandler, 0, SECONDS(10) },       // MNT-3036 parameter is overwritten by caller
};
#endif

#if (LED_MODE != CHIPSET)
static bool  ledTestMode     = false;                       // MNT-3052
#endif
#if defined _TRAILER && defined _LIONESS
static bool  csTestMode      = false;                                 // MNT-3054
extern void mnt_handleGpioEventsTestCs(uint8_t gpio, uint8_t nEvent); // MNT-3054
#endif

bool mp_isPeripheralPowerOn(void) {
    return peripheralPowerOn;
}

/*
 *
 * mp_getInterruptStatus
 *
 * Purpose: find the reason for wakeup
 *
 */ 
uint32 mp_getInterruptStatus(void)
{
	return gInterruptStatus;
}

// MNT-2190
const char * mnt_mcuGetInterruptStr(uint32_t mask) {
    #undef ASSOC_MCU_INT_BIT
    #define ASSOC_MCU_INT_BIT(enum, str)   (str),
    static const char * interruptMaskStrs[] = {
        ALL_MCU_INT_BITS
    };

    // Keep space for the extreme case of all bits set:
    // - Place for '+' signs (number of set bits minus one)
    // - Place for the strings for all bits
    // - Place for null terminator
    #undef ASSOC_MCU_INT_BIT
    #define ASSOC_MCU_INT_BIT(enum, str)   + CONST_STRLEN(str)
    static char expandedStr[sizeof(mask) * 8 - 1 + ALL_MCU_INT_BITS + sizeof((char)'\0')];

    core_utilsConvertBitmaskToStr(mask, &interruptMaskStrs[0], &interruptMaskStrs[1], ARRAY_SIZE(interruptMaskStrs), expandedStr, sizeof(expandedStr));

    return (const char *)expandedStr;
}

#if (IO_MODE == PIC) || (IO_MODE == EFM)

// Local prototypes
static void mph_HandleCommand(uint8 Command, uint8 *pData, uint8 size);
static void mph_HandleResponse(uint8 Command, uint8 *pData, uint8 size);
#if (IO_MODE == EFM) || defined _WILDCAT
static void mph_HandleGpio(uint8 *pData, uint8 size);
static void mnt_reconfigureMcuMotionState(void);
#endif
static void mph_HandleAdc(uint8 *pData, uint8 size);
static void mph_initGetIoInfo (void);
#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
static void mph_initGetExtIoInfo (void);
#endif
extern void mnt_mfgTestAccelerometerRead(uint8_t reg, uint8_t value);
#if (IO_MODE == PIC)
static void mnt_ServiceAccelerometerEvent(uint32_t intMask);
#endif
#if defined _TRAILER || defined _WILDCAT
static void mnt_recovery_mcuApp_check(void);
#endif
static void mnt_TimerUpdateViaRtc(void);    // MNT-2623 Adjust timers based on RTC time

// Local variables
DEFINE_MODULE(MODULE_IF);
static  uint8_t     gRxState;
static  uint8_t     gRxLength;
static  uint8_t     gRxCommand;
static  uint16_t    gRxCount;
static  uint8_t     gRxData[512];   // Buffer used to get MCU response/event
static  uint32_t    gLastInterruptStatus    = 0;

// MNT-2456
static  uint8_t     gIoVersion[4]   = 
    #ifdef _WIN32
        { 0, 0xFF, 0xFF, 0xFF };    // Make simulator happy by avoiding all ones
    #else
        { 0xFF, 0xFF, 0xFF, 0xFF };
    #endif

#if defined(_TRAILER) && defined(SUPPORT_BLE_DFU)
static  uint8_t     gMuxVersion[4]   = 
    #ifdef _WIN32
        { 0, 0xFF, 0xFF, 0xFF };    // Make simulator happy by avoiding all ones
    #else
        { 0xFF, 0xFF, 0xFF, 0xFF };
    #endif

static  uint8_t     gMacId[6]   = 
    #ifdef _WIN32
        { 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };    // Make simulator happy by avoiding all ones
    #else
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // MNT-2804
    #endif
#endif

static  FLAG        gfGotModel      = FALSE;
static  uint16_t    gPowerupCount   = 0;
static  uint16_t    gResetCount     = 0;
static  uint8_t     gBaudrateChange = 0;

static  uint8_t     gSerialBuffer[MAX_AT_RESPONSE];
static  uint16_t    gSerialHead;
static  uint16_t    gSerialTail;
static  uint16_t    gSerialSize;

#ifdef DEBUGF
static const char * strOn   = "On";
static const char * strOff  = "Off";
#endif

#if defined _TRAILER || defined _LIONESS || (IO_MODE == PIC)
#if (IO_MODE!=PIC) || defined _WILDCAT
    #include "mnt_sensor.h"

    static  uint8_t gRxChecksum;
#endif

    #define KICK_COMMAND		'K'
    #define	HEADER_TO_ESCAPE	'$'
    #define	ESCAPE_CHAR			'!'
    #define	ESCAPE_4_HEADER		'0'
    #define	ESCAPE_4_ESCAPE		'1'
#else
    #define KICK_COMMAND	    'B'
#endif

static uint32_t             gCommStatusMask     = 0;
static uint32_t             gCommStatusCounter  = 0;
static uint32_t             gCommRtcTime        = 0;
static ACC_MOTION_STATES_t  gMotionState        = MOTION_STATE_BOTH;     // Current MCU accelerometer state
static ACC_MOTION_STATES_t  gMotionMode         = MOTION_STATE_BOTH;     // Target MCU accelerometer state

#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
static uint32_t             gMainComponentMask  = 0; // Holds which components to read
static uint32_t             gPeriComponentMask  = 0; // Holds which components to read
static uint32_t             gInfoTypeMask       = 0;
static uint32_t             gInfoTypeCounter    = 0;
#endif

#define COMM_STATUS_MAX_RETRIES         5
#define COMM_STATUS_RETRY_DELAY         SECONDS(3)
#define COMM_BAUD_RATE_SYNC_DELAY       SECONDS(6)  // MNT-2341, MNT-2375

#if defined (_TRAILER)

    #define COMM_STATUS_ALL_INIT_MASK   ( \
            COMM_STATUS_GET_VERSION_MASK | \
            COMM_STATUS_GET_BOOT_STATUS_MASK | \
            COMM_STATUS_GET_INT_STATUS_MASK | \
            COMM_STATUS_GET_RTC_MASK | \
            COMM_STATUS_SET_DEFAULT_WD_MASK | \
            COMM_STATUS_SET_BATTCHG_MASK )

#elif defined(_LIONESS)

    #define COMM_STATUS_ALL_INIT_MASK   ( \
            COMM_STATUS_GET_VERSION_MASK | \
            COMM_STATUS_GET_MODEL_MASK | \
            COMM_STATUS_GET_COUNTERS_MASK | \
            COMM_STATUS_SET_DEFAULT_WD_MASK )

#elif defined(_PANTHER)

    #define COMM_STATUS_ALL_INIT_MASK   ( \
            COMM_STATUS_GET_VERSION_MASK | \
            COMM_STATUS_GET_MODEL_MASK | \
            COMM_STATUS_GET_COUNTERS_MASK | \
            COMM_STATUS_SET_DEFAULT_WD_MASK | \
            COMM_STATUS_SET_UID_UPDATED_MASK )  // Could be zero

#elif defined(_WILDCAT)

    #define COMM_STATUS_ALL_INIT_MASK   ( \
            COMM_STATUS_GET_VERSION_MASK | \
            COMM_STATUS_GET_MODEL_MASK | \
            COMM_STATUS_GET_INT_STATUS_MASK | \
            COMM_STATUS_GET_RTC_MASK | \
            COMM_STATUS_SET_DEFAULT_WD_MASK | \
            COMM_STATUS_GET_ACC_STATUS_MASK | \
            COMM_STATUS_SET_ACC_MODE_MASK )     // MNT-2290, could be zero

#else

    #define COMM_STATUS_ALL_INIT_MASK   ( \
            COMM_STATUS_GET_VERSION_MASK | \
            COMM_STATUS_GET_MODEL_MASK | \
            COMM_STATUS_GET_COUNTERS_MASK )

#endif

#if defined(SUPPORT_MCU_ACC_STREAMING) // MNT-2409

void mph_sendMCUAccWorkmode(void) { // MNT-2401
    if (IS_COLLISION_ENABLED()) {
        mph_setMCUAccWorkmode(ACC_MODE_COLLISION_REPORT);
    } else {
        mph_setMCUAccWorkmode(ACC_MODE_MOTION_DETECTION); // This also mode supports Wake up and Harsh Driving
    }
}

void mph_setMCUAccWorkmode(ars_AccWorkMode_t  mode) { // MNT-2401
    uint8_t Parameters[2];

    Parameters[0] = 'C';        // Set MCU Acc working mode
    Parameters[1] = (uint8_t)mode;
    mph_sendPicFrame('X', Parameters, sizeof(Parameters));
    if (IS_COLLISION_ENABLED()) {
        mph_declareMCUAccODR(LIS_ODR_100Hz);
    } else {
        mph_declareMCUAccODR(LIS_ODR_10Hz);
    }
}
#endif

// MNT-2456
void mnt_getIOversion(uint8_t * pVersion) {
    if (NULL != pVersion) {
        mnt_memcpy(pVersion, gIoVersion, sizeof(gIoVersion));
    }
}

// MNT-2488
uint32_t mnt_getIOversionAsInt(void) {
    return VERSION_ARRAY_TO_NUM(gIoVersion);
}

uint32 mnt_GetLastChangedBaudrate(void) {
    return baudRatesValues[gBaudrateChange];
}

#if defined(_TRAILER) || defined(_PANTHER) || defined(_WILDCAT)
// MNT-608
void mph_setPowerStateThresholds(uint8_t powerStateMask, uint16_t lowPowerThreshold, uint16_t highPowerThreshold, uint16_t sourceDebounce, uint16_t thresholdDebounce) {
    uint8 Parameters[9];

    Parameters[0] = powerStateMask;
    Parameters[1] = lowPowerThreshold & 0xFF;
    Parameters[2] = (lowPowerThreshold >> 8) & 0xFF;
    Parameters[3] = highPowerThreshold & 0xFF;
    Parameters[4] = (highPowerThreshold >> 8) & 0xFF;
    Parameters[5] = sourceDebounce & 0xFF;
    Parameters[6] = (sourceDebounce >> 8) & 0xFF;
    Parameters[7] = thresholdDebounce & 0xFF;
    Parameters[8] = (thresholdDebounce >> 8) & 0xFF;
    mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(MODULE_IFADC, "%s: Mask:0x%02x, LowThres:%d, HighThres:%d, Source: %d, State: %d\r\n", __func__, 
        powerStateMask, lowPowerThreshold, highPowerThreshold, sourceDebounce, thresholdDebounce);
}
#endif

#if defined(_WILDCAT) || defined(_TRAILER)
// MNT-2187
/*
$<0x06>A<8bitTriggerMask><16bitVirtualV><16bitHysteresisV><8bitDebounce><CR>
where:
    8bitTriggerMask
        0 = disabled
        1 = Low -> High
        2 = Low <- High
        3 = Low <-> High

    16bitVirtualV = Virtual Ignition in mV

    16bitHysteresisV = Hysteresis in mV

    8bitDebounce = Debounce time in Secs for event
*/
void mph_setVirtualIgnitionThresholds(uint8_t triggerMask, uint16_t voltageMV, uint16_t hysteresisMV, uint8_t debounce) {
    uint8 Parameters[6];

    Parameters[0] = triggerMask;
    Parameters[1] = voltageMV & 0xFF;
    Parameters[2] = (voltageMV >> 8) & 0xFF;
    Parameters[3] = hysteresisMV & 0xFF;
    Parameters[4] = (hysteresisMV >> 8) & 0xFF;
    Parameters[5] = debounce;
    mph_sendPicFrame('A', Parameters, sizeof(Parameters));
    mnt_debug_printf(MODULE_IFADC, "%s: Mask: %d, V: %d, Hys: %d, Debounce: %d\r\n", __func__, triggerMask, voltageMV, hysteresisMV, debounce);
}

void mnt_setVirtualIgnitionThresholds(void) {
    // MNT-1948, MNT-2187, MNT-2202 Support wake from virtual ignition
	if (IGNITION_MODE_VOLTAGE == gMonetData.IgnitionMode) {
        // Configure to:
        // - Wake on any transition
        // - No hysteresis
        // - Set debounce time to 2 seconds maximum to avoid glitches and perform full debounce in application
        mph_setVirtualIgnitionThresholds(3, gMonetData.IgnitionVoltage, 0, MIN(gMonetData.IgnitionModeTimer, SECONDS(2)));
	}
}
#endif

/*
 *
 * mnt_IoProcessorWatchdogHandler
 *
 * Purpose: Timeout handler during shutdown when waiting for sleep event to be sent after first timeout
 *
 */
static void mnt_IoProcessorWatchdogHandler(mnt_Timer_t *pTimer)
{
    mph_initGetIoInfo();
}

#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
/*
 *
 * mnt_ExtIoProcessorWatchdogHandler
 *
 * Purpose: Timeout handler during shutdown when waiting for sleep event to be sent after first timeout
 *
 */
static void mnt_ExtIoProcessorWatchdogHandler(mnt_Timer_t *pTimer)
{
    mph_initGetExtIoInfo();
}

/*
 *
 * mph_initGetExtIoInfo
 *
 * Purpose: Get initial extended info from IO Processor
 *
 */
static void mph_initGetExtIoInfo (void) { // MNT-2812
    bool isInfoCompleted = (gMainComponentMask == 0) && (gPeriComponentMask == 0); // If response received the bit is cleared

    gInfoTypeCounter++;
    mnt_debug_printf(gThisModule, "%s: Main: %08X, Peri: %08X, Try #: %d\r\n", __func__, gMainComponentMask, gPeriComponentMask, gInfoTypeCounter);
    if (! isInfoCompleted) {
        if (gInfoTypeCounter <= COMM_STATUS_MAX_RETRIES) {
            if (prod_isNala()) {
                if (gMainComponentMask & COMPONENT_INFO_VERSION_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MCU, COMPONENT_INFO_VERSION_MASK);       // Get MCU Version
                }
                if (gMainComponentMask & COMPONENT_INFO_MAC_ADDR_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MCU, COMPONENT_INFO_MAC_ADDR_MASK);      // Get MAC Addr
                }
                if (gMainComponentMask & COMPONENT_INFO_HWID_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MCU, COMPONENT_INFO_HWID_MASK);          // Get HWID
                }
                if (gPeriComponentMask & COMPONENT_INFO_VERSION_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MUX_MCU, COMPONENT_INFO_VERSION_MASK);   // Get MCU Version
                }
                if (gPeriComponentMask & COMPONENT_INFO_MAC_ADDR_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MUX_MCU, COMPONENT_INFO_MAC_ADDR_MASK);  // Get MAC Addr
                }
                if (gPeriComponentMask & COMPONENT_INFO_HWID_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_NALA_MUX_MCU, COMPONENT_INFO_HWID_MASK);      // Get HWID
                }
            } else if (prod_isSimba()) { // MNT-2874
                if (gMainComponentMask & COMPONENT_INFO_VERSION_MASK) { 
                    mnt_requestComponentInformation(COMPONENT_TARGET_SIMBA_MCU, COMPONENT_INFO_VERSION_MASK);       // Get MCU Version
                }
                if (gPeriComponentMask & COMPONENT_INFO_VERSION_MASK) { 
//                    mnt_SimbaBleCommand('V', NULL, 0);     // Get the version
                    mnt_requestComponentInformationViaPT(COMPONENT_TARGET_SIMBA_BLE_MCU, COMPONENT_INFO_VERSION_MASK);   // Get MCU Version
                }
                if (gPeriComponentMask & COMPONENT_INFO_MAC_ADDR_MASK) { 
                    mnt_requestComponentInformationViaPT(COMPONENT_TARGET_SIMBA_BLE_MCU, COMPONENT_INFO_MAC_ADDR_MASK);  // Get MAC Addr
                }
            }
            // Start timer to check the status later
            initExtendedInfoTimer.mode      = MNT_TIMER_INTERVAL_MODE;
            initExtendedInfoTimer.interval  = 6;
            initExtendedInfoTimer.handler   = mnt_ExtIoProcessorWatchdogHandler;
            mnt_TimerStart(&initExtendedInfoTimer);
        } else {
            mnt_debug_printf(gThisModule, "%s: Failed to get ext info from IO: %d\r\n", __func__, gCommStatusCounter);
        }
    }
}

void mph_requestPeriInfo(uint32_t info) { // MNT-2874
    gPeriComponentMask = info;
    gInfoTypeCounter = 0;
    mph_initGetExtIoInfo();
}

void mp_clearPeriMask(uint32_t mask) { // MNT-2874
    gPeriComponentMask &= ~mask; // Clear this bit
}

#endif

#if (IO_MODE!=NONE) && (IO_MODE!=SELF)
void decodeMissingCommMask(uint32 mask, uint32 status) {
    uint32 missingData = mask & ~status;
    if ((missingData & COMM_STATUS_GET_VERSION_MASK))  {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_VERSION_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_GET_BOOT_STATUS_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_BOOT_STATUS_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_GET_INT_STATUS_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_INT_STATUS_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_GET_RTC_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_RTC_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_SET_DEFAULT_WD_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_SET_DEFAULT_WD_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_GET_MODEL_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_MODEL_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_GET_COUNTERS_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_GET_COUNTERS_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_SET_TEMPERATURE_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_SET_TEMPERATURE_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_SET_BATTCHG_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_SET_BATTCHG_MASK\r\n", __func__);
    }
    if ((missingData & COMM_STATUS_SET_RTC_UPDATED_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_SET_RTC_UPDATED_MASK\r\n", __func__);
    }
#ifdef _MINITRACK
    if ((missingData & COMM_STATUS_SET_UID_UPDATED_MASK)) {
        mnt_debug_printf(MODULE_IFADC, "%s: COMM_STATUS_SET_UID_UPDATED_MASK\r\n", __func__);
    }
#endif
}
#endif

// MNT-2793
/*
 *
 * mph_getComponentInfo
 *
 * Purpose: Extracting information from the components
 *
 */
void mph_getComponentInfo(uint8_t instanceId, uint8_t component, uint32_t bitMask)
{
    core_bufferAccumulative_ts  abuf;
    uint8_t                     Parameters[6]   = { 0 };
    size_t                      len             = 0;

    // MNT-2983 Support one or multiple instances and 4 bytes bitmask
    core_abufferInit(&abuf, Parameters, sizeof(Parameters));
    core_abufferAddChar(&abuf, component);
    if (IS_MULTI_INSTANCE(component)) {
        core_abufferAddChar(&abuf, instanceId);
    } else {
        UNUSED_PARAMETER(instanceId);
    }
    if (0 == (bitMask & MASK_FOR_BITFIELD(7, 31))) {
        core_abufferAddChar(&abuf, (uint8_t)bitMask);
    } else {
        core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, bitMask, 4);
    }
    len = core_abufferGetSize(&abuf);
    
    mph_sendPicFrame(0xAA, Parameters, (uint16_t)len);
    mnt_debug_printf(MODULE_IO, "%s: instanceId: %d, component: %d, bitMask: %08X\r\n", __func__, instanceId, component, bitMask);
}

// MNT-2954
void mnt_setBasebandWDTimer(void) {
    uint8_t         timedata[2] = { 0 };
    const uint16_t  timer       =
    #if (defined _LIONESS || defined _WILDCAT) && defined _TRAILER  
        gMonetData.fWDenabled ? HOURS(1) : 0xFFFF;
    #else
        HOURS(1); 
    #endif
 
    timedata[0] =  (timer & 0xFF);
    timedata[1] =  (timer >> 8) & 0xFF;
    mph_sendPicFrame('B', timedata, sizeof(timedata));
}

/*
 *
 * mph_initGetIoInfo
 *
 * Purpose: Get initial info from IO Processor
 *
 */
static void mph_initGetIoInfo (void) 
{
    bool isInitCompleted = COMM_STATUS_ALL_INIT_MASK == (gCommStatusMask & COMM_STATUS_ALL_INIT_MASK);

    gCommStatusCounter++;
    if (! isInitCompleted) {
        mnt_debug_printf(gThisModule, "%s: AllMask: %08X, Mask: %08X, Try #: %d\r\n", __func__, COMM_STATUS_ALL_INIT_MASK, gCommStatusMask, gCommStatusCounter);
        if (gCommStatusCounter <= COMM_STATUS_MAX_RETRIES) {
            if (! (gCommStatusMask & COMM_STATUS_GET_VERSION_MASK))  {
                mph_sendPicFrame('V', NULL, 0);     // Get the version
            }
    #if defined _TRAILER
            // MNT-2299, MNT-2587, Query the boot loader. If the boot loader responds, update the mcu partition.
            mnt_recovery_mcuApp_check();
            if (! (gCommStatusMask & COMM_STATUS_GET_RTC_MASK)) { // MNT-661 Read RTC before Interrupt
                mph_sendPicFrame('R', NULL, 0);		// Get RTC
            }
            if (! (gCommStatusMask & COMM_STATUS_GET_INT_STATUS_MASK)) {
                mph_sendPicFrame('I', NULL, 0);	    // Get interrupt status
            }
            if (! (gCommStatusMask & COMM_STATUS_GET_BOOT_STATUS_MASK)) {
                mph_sendPicFrame('U', NULL, 0);     // Get boot status
            }
            if (! (gCommStatusMask & COMM_STATUS_SET_DEFAULT_WD_MASK)) {
                mnt_setBasebandWDTimer();
            }
            if (! (gCommStatusMask & COMM_STATUS_SET_TEMPERATURE_MASK)) {
                mnt_setTempThresholds();    // Set the 1-Wire Temp thresholds
                mp_OneWireEnable(gMonetData.OneWireEnable); // MNT-840  Enable/disable 1-wire Temp sensor based on config. setting
            }
            if (! (gCommStatusMask & COMM_STATUS_SET_BATTCHG_MASK)) {
                mp_setChargeThreshold(gMonetData.mvBTCVT);   // Set the Battery Charging threshold
            }
            // Make sure MCU is configured to not kill itself when external power is removed
            // (enables going into the stealth mode). Use AT+XBUBX to remove power if required.
            mp_WakeUpPin(0);
    #else
        #if defined(_PANTHER)
            #ifdef _MINITRACK
            if (! (gCommStatusMask & COMM_STATUS_SET_UID_UPDATED_MASK))  {
                mp_askUID();
            }
            #endif
            // MNT-1550
            if (! (gCommStatusMask & COMM_STATUS_GET_RTC_MASK)) { // MNT-661 Read RTC before Interrupt
                mph_sendPicFrame('R', NULL, 0);		// Get RTC
            }
            if (! (gCommStatusMask & COMM_STATUS_GET_INT_STATUS_MASK))  {
	            mph_sendPicFrame('I', NULL, 0);	    // Get interrupt status
            }
        #endif
        #if defined(_WILDCAT)
            // MNT-2299 Query the boot loader. If the boot loader responds, update the mcu partition.
            mnt_recovery_mcuApp_check();
            if (! (gCommStatusMask & COMM_STATUS_GET_RTC_MASK)) {
                mph_sendPicFrame('R', NULL, 0);		// Get RTC
            }
            if (! (gCommStatusMask & COMM_STATUS_GET_INT_STATUS_MASK))  {
	            mph_sendPicFrame('I', NULL, 0);	    // Get interrupt status
            }
        #endif
        #if defined(SUPPORT_MCU_ACC_STREAMING)
            if (! (gCommStatusMask & COMM_STATUS_SET_ACC_MODE_MASK))  {
	            mph_sendMCUAccWorkmode();	        // MNT-2401 Send MCU Acc Working mode
            }
        #endif
            if (! (gCommStatusMask & COMM_STATUS_GET_MODEL_MASK))  {
                mnt_debug_printf(gThisModule, "%s: Sending mph_sendPicFrame(T)\r\n", __func__);
	            mph_sendPicFrame('T', NULL, 0);     // Get the model
            }
            if (! (gCommStatusMask & COMM_STATUS_GET_COUNTERS_MASK))  {
                mnt_debug_printf(gThisModule, "%s: Sending mph_sendPicFrame(Y)\r\n", __func__);
	            mph_sendPicFrame('Y', NULL, 0);		// Get reset counters
            }
        // MNT-2381 Send the 'B' command at the end
        #if defined(_LIONESS) || defined(_PANTHER) || defined(_WILDCAT)
            if (! (gCommStatusMask & COMM_STATUS_SET_DEFAULT_WD_MASK))  {
                mnt_setBasebandWDTimer();
            }
        #endif
    #endif // #if defined _TRAILER
        #if defined(_WILDCAT)
            if (! (gCommStatusMask & COMM_STATUS_GET_ACC_STATUS_MASK)) { // MNT-2290
                mph_sendPicFrame('X', NULL, 0);     // Get the Acc motion state
            }
        #endif
            // Start timer to check the status later
            initWatchdogTimer.mode      = MNT_TIMER_INTERVAL_MODE;
            initWatchdogTimer.interval  = 0 == gCommStatusMask ? COMM_BAUD_RATE_SYNC_DELAY : COMM_STATUS_RETRY_DELAY; // MNT-2341
            initWatchdogTimer.handler   = mnt_IoProcessorWatchdogHandler;
            mnt_TimerStart(&initWatchdogTimer);
        } else {
            mnt_debug_printf(gThisModule, "%s: Failed to get info from IO: %d\r\n", __func__, gCommStatusCounter);

        #if defined(SUPPORT_MCU_ACC_STREAMING)
            // MNT-2409 If the only remaining difference is a new mask then it means that MCU is not updated
            // Continue without reboot for backward compatibility
            if (COMM_STATUS_ALL_INIT_MASK == (COMM_STATUS_ALL_INIT_MASK & (gCommStatusMask | COMM_STATUS_SET_ACC_MODE_MASK))) {
                isInitCompleted = true;
                mnt_debug_printf(gThisModule, "%s: MCU backward compatibility\r\n", __func__);
            }
        #endif

            if (!isInitCompleted && 
                !mp_IsPicLoaderActive() &&
                (CORE_RESET_REASON_MCU_WATCHDOG != core_updownGetResetReason())) {  // MNT-3075 Don't try again if last reset was because of MCU, MNT-2591
        #if (CHIPSET_VENDOR != MSFT)
                // MNT-2075, re initialize the UART #2 does not recover, need to soft reset the module
                mnt_doSystemReset(RESET_SOFT, 0, CORE_RESET_REASON_MCU_WATCHDOG);
        #endif
            }
        }
    }

    if (isInitCompleted) {
        mp_writeAllOutputs(); // MNT-2856
    #if defined _TRAILER
        mp_OneWireDiscoverNow();    // MNT-842  Perform discovery of the 1-wire sensors when all is ready
    #endif
    #if defined(_WILDCAT) || defined(_TRAILER)
        mnt_setVirtualIgnitionThresholds();     // MNT-2202
    #endif
    }

    // If got all initial  info
#if (IO_MODE!=NONE) && (IO_MODE!=SELF)
    if ((COMM_STATUS_ALL_INIT_MASK & gCommStatusMask) == 0) {
        decodeMissingCommMask(COMM_STATUS_ALL_INIT_MASK, gCommStatusMask);
    }
#endif
}

// MNT-1791
void mnt_mcuWdHandler(mnt_Timer_t * pTimer) {
    // MNT-2378 Bootloader mode has its own baud rate re-sync mechanism, do nothing
    if (mp_IsPicLoaderActive()) {
        mnt_debug_printf(gThisModule, "%s: Ignored, BL is active\r\n", __func__);
    } else {
        static uint8_t  baudRateToTryIndex  = 0;
        // MNT-2375 First try non-default baud rate
    #if 115200 == MNT_IO_BAUD_RATE
        static uint32_t baudRatesToTry[] =
            { 9600, 115200 };
    #else
            { 115200, 9600 };
    #endif

        UNUSED_PARAMETER(pTimer);
        // MNT-1791 In case no response is received try to go through all possible baud rates
        mnt_debug_printf(gThisModule, "%s: Baud rate: %d\r\n", __func__, baudRatesToTry[baudRateToTryIndex]);
        mp_setUartSpeed(baudRatesToTry[baudRateToTryIndex]);
        baudRateToTryIndex++;
        if (baudRateToTryIndex >= ARRAY_SIZE(baudRatesToTry)) {
            baudRateToTryIndex = 0;
        }
    }
}

/*
 *
 * mph_initIO
 *
 * Purpose: Initialize the baseband I/F
 *
 */
void mph_initIO( void )
{
	// Init variables
	memset(gIoVersion, 0xFF, sizeof(gIoVersion));
#ifdef _WIN32
    gIoVersion[0] = 0;          // Make simulator happy by avoiding all ones
#endif
    mph_uartCallback        = NULL;
#if (CHIPSET_VENDOR == MSFT)
    gfGotModel              = TRUE; // Simulate that we got a model
#else
	gfGotModel              = FALSE;
#endif
	gSerialHead             = 0;
	gSerialTail             = 0;
	gSerialSize             = 0;
	gInterruptStatus        = 0;
    gLastInterruptStatus    = 0;
	gRxState                = IO_WAIT_FOR_DOLLAR;
    gCommStatusMask         = 0;
    gCommStatusCounter      = 0;

#ifdef _TRAILER
	// Turn off the LED's on the board
	GPS_LED_OFF();	
	CELLULAR_LED_OFF();
#endif

#if !defined( _TRAILER) // MNT-2591
    /* MNT-2587 - set this timer before set initWatchdogTimer timer from mph_initGetIoInfo().
                  Because of it needs to set baudrate before parsing commands */
    // MNT-1791 Timer should be always running
    mcuCommWatchdogTimer.mode       = MNT_TIMER_PERIODIC_MODE;
    mcuCommWatchdogTimer.interval   = COMM_BAUD_RATE_SYNC_DELAY; // MNT-2341
    mcuCommWatchdogTimer.handler    = mnt_mcuWdHandler;
  #ifndef _WIN32
    mnt_TimerStart(&mcuCommWatchdogTimer);
  #endif
#endif

	mnt_debug_printf(gThisModule, "%s: Config UART to %d\r\n", __func__, MNT_IO_BAUD_RATE);
	mp_setUartSpeed(MNT_IO_BAUD_RATE);

    mnt_debug_printf(gThisModule, "%s: GPIO - D: %02X, I: %02X, O: %02X\r\n", __func__, gMonetData.IoDirection, gMonetData.IoRead, gMonetData.IoWrite);
	mph_initGetIoInfo();        // Get initial information from IO processor

#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
    if (prod_isNala()) { // MNT-2812
        gMainComponentMask = COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK;    // BLE MCU
        gPeriComponentMask = COMPONENT_INFO_VERSION_MASK;                                   // MUX
    }
    if (prod_isSimba()) { // MNT-2812
        gMainComponentMask = COMPONENT_INFO_VERSION_MASK;                                   // MCU
        gPeriComponentMask = COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK;    // BLE MCU
    }
    mph_initGetExtIoInfo();     // MNT-2812 Get Extended Information
#endif
#if defined SUPPORT_AC61_CAMERA || defined SUPPORT_ZAZU_CAMERA // MNT-2793, MNT-2997
    {
        uint8_t instanceId = 0;
        while(instanceId < CAMERA_MAX_NUM){
            if (!gMonetData.bGotCameraInfo[instanceId])  {
                mph_getComponentInfo(instanceId, COMPONENT_TARGET_SLP01_MCU,         COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK);
                mph_getComponentInfo(instanceId, COMPONENT_TARGET_CAMERA_NORDIC_MCU, COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK);
                mph_getComponentInfo(instanceId, COMPONENT_TARGET_CAMERA_ESP32_MCU,  COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_HWID_MASK);
            }
            instanceId++ ;
        }
    }
#endif

#ifdef SUPPORT_BLE_DFU // MNT-2624
    if(prod_isSimba()) {
        mph_initGetBLEInfo();
    }
    if (prod_isNala()) { // MNT-2812 
        mnt_requestNalaInformation('M');     // Get the MUX version (deprecated)
        mnt_requestNalaInformation('I');     // Get the MAC ID (deprecated)
    }
#endif
#if defined _TRAILER && defined _LIONESS // MNT-3060
    if (prod_isNala() && mp_getHwRevision() >= 2) {
        mp_SetSolarCheckPeriod(gMonetData.solarCheckPeriod); // set solar checking period
    }
#endif
#ifdef _TRAILER
	mnt_csInit();       // Init the cargo sensor
	mp_SetUartCallback(mnt_csGetData);
#endif	// #ifdef _TRAILER
}

#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
void mnt_startVersionRecovery(void) { // MNT-2884
    if (prod_isSimba()) {
        gMainComponentMask = COMPONENT_INFO_VERSION_MASK;                                   // MCU
        gPeriComponentMask = COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK;    // BLE MCU
        gInfoTypeCounter = 0;
        mph_initGetExtIoInfo();
    }
}
#endif

/*
 *
 * mph_setLED
 *
 * Purpose: Handle the change of LED status
 *
 */
 #if (LED_MODE != CHIPSET)
// MNT-3052
void mnt_ledTest(uint8_t led, bool fOn)
{
    uint8 Parameters[3];

    Parameters[0] = led;
    Parameters[1] = fOn ? 0xff : 0;
    Parameters[2] = 0;

    mph_sendPicFrame('L', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s L %02x %02x %02x\r\n", __func__, Parameters[0], Parameters[1], Parameters[2]);
    ledTestMode = true;
}

// MNT-3052
void mnt_ledTestExit(void)
{
    uint8 Parameters[3];

    Parameters[0] = 2;     //LED_BIT_CHARGER
    Parameters[1] = 0xff;  
    Parameters[2] = 0xff;  //release control

    mph_sendPicFrame('L', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s L %02x %02x %02x\r\n", __func__, Parameters[0], Parameters[1], Parameters[2]);
    ledTestMode = false;
}

void mph_setLED(uint8 Event )
{
	uint8 Parameters[] = {0, 60, 60};		// Slow blink is default

    if (!ledTestMode) { // MNT-3052
    	switch (Event & 0x3F) {
    	case LED_OFF:
    		Parameters[0] =	3;					// Synchronous mode
    		Parameters[1] = 0;					// Never On
    		Parameters[2] = 255;				// Long off
    		break;
    	case LED_RESET:
    		Parameters[0] =	3;					// Synchronous mode
    		Parameters[1] = 1;					// Short blink
    		Parameters[2] = 255;				// Long off
    		break;
    	case LED_BOOT:
    		Parameters[0] = 3;					// Synchronous mode slow, other default are OK
    		break;
    	case LED_GPS_UNLOCK:
    		Parameters[0] = 0x80 | LED_GPS;		// Continuous blinking slow, other default are OK
    		break;
    	case LED_GPS_LOCK:
    		Parameters[0] = LED_GPS;			//
    		Parameters[1] = 255;				// ON, not blinking
    		Parameters[2] = 0;				    // ON, no Off time MNT-2955
    		break;
    	case LED_CELLULAR_UNLOCK:
    		Parameters[0] = 0x80 | LED_PACKET;	// Continuous blinking slow, other default are OK
    		break;
    	case LED_CELLULAR_LOCK:
    	case LED_PXP_UNLOCK:
    		Parameters[0] = 0x80 | LED_PACKET;	// Continuous blinking fast
    		Parameters[1] = 30;					// Fast blinking
    		Parameters[2] = 30;					//
    		break;
    	case LED_PXP_LOCK:
       		Parameters[0] = LED_PACKET;			//
    		Parameters[1] = 255;				// ON, not blinking
    		Parameters[2] = 0;				    // ON, no Off time MNT-2955
     		break;
    	case LED_PROVISIONING_IN_PROGRESS:
    		Parameters[0] = 2;					// Firetruck mode
    		Parameters[1] = 10;					// Very fast
    		Parameters[2] = 10;					//
    		break;
    	case LED_LOW_POWER:
    		Parameters[0] = LED_PACKET;			// Short blink every 20 seconds
    		Parameters[1] = 10;					//
    		Parameters[2] = 200;				//
    		break;
    	default:
    		break;
    	}
    #ifdef DEBUGF
        {
            uint8_t led     = Parameters[0] & 0x7f;
            uint8_t repeat  = Parameters[0] & 0x80;

            mnt_debug_printf(gThisModule, "%s: Led:(%d)%s Mode:%d On:%d Off:%d\r\n", __func__, led, (led == 0) ? "GPS" : (led == 1) ? "CELL" : "OTHER", repeat, Parameters[1], Parameters[2]);
        }
    #endif

#ifdef _TRAILER // MNT-3128
        mp_saveLastLed(Parameters[0] & 0x7f, Parameters[1], Parameters[2]);
#endif        

        mph_sendPicFrame('L', Parameters, sizeof(Parameters));
    }
}
#endif

#if defined _TRAILER || defined _LIONESS || (IO_MODE == PIC)
uint8 mph_AddWithEscape(uint8 *pLocation, uint8 nValue)
{
	uint8 added = 0;
	switch (nValue) {
	case HEADER_TO_ESCAPE:
		*pLocation++ = ESCAPE_CHAR;
		*pLocation++ = ESCAPE_4_HEADER;
		added = 2;
		break;
	case ESCAPE_CHAR:
		*pLocation++ = ESCAPE_CHAR;
		*pLocation++ = ESCAPE_4_ESCAPE;
		added = 2;
		break;
	default:
		*pLocation++ = nValue;
		added = 1;
		break;
	}
	return added;
}
#endif

/*
 *
 * mph_sendPicFrame
 *
 * Purpose: Send a frame to the I/O processor
 *
 */
void mph_sendPicFrame(uint8_t cmd, uint8_t * pParameters, uint16_t nSize) {
    mph_sendExtPicFrame(UART_TARGET_MCU, cmd, pParameters, nSize); // MNT-2797
}

/*
 *
 * mph_sendAppPicFrame
 *
 * Purpose: Send a frame to the I/O processor
 *
 */
void mph_sendAppPicFrame(uint8_t cmd, uint8_t * pParameters, uint16_t nSize) { // MNT-2797
    mph_sendExtPicFrame(UART_TARGET_APP, cmd, pParameters, nSize);
}

/*
 *
 * mph_sendExtPicFrame
 *
 * Purpose: Send a frame to the I/O processor
 *
 */
void mph_sendExtPicFrame(Uart_Target_List_e mode, uint8_t cmd, uint8_t * pParameters, uint16_t nSize) { // MNT-2797
    #define _PREAMBLE_SIZE  8    // MNT-1131
    static  uint8_t Frame[PIC_CMD_BUFFER_SIZE];
            int     nIndex      = 0;

    // mnt_debug_printf(gThisModule, "%s: Called with cmd:%c\r\n", __func__, cmd);
#if defined NORDIC_DFU || defined SUPPORT_BLE_DFU
	/* immediately return when DFU in progress */
	if (mp_IsDFUThreadActive()) {
		return;
	}
#endif

#if (IO_MODE == EFM) || defined _WILDCAT    // MNT-2066, MNT-2208, MNT-2386
    #define _MAX_PAYLOAD_SIZE   (sizeof(Frame) / 2 - _PREAMBLE_SIZE)      // Assuming the worst case when each data byte should be escaped
    if (mp_IsPicLoaderActive() && ('@' != cmd) && ('J' != cmd) && (0xAA != cmd)) { // MNT-2797
        if ('V' == cmd) {
            // MNT-2663, allow to send version command if mcu update state machine is at reset state
            if (!mnt_isMcuResetState()) {
                mnt_debug_printf(gThisModule, "%s: MCU in BootLoad Mode Wrong cmd %c\r\n", __func__, cmd);
                return;
            }
        } else {
            mnt_debug_printf(gThisModule, "%s: MCU in BootLoad Mode Wrong cmd %c\r\n", __func__, cmd);
            return;
        }
    }
#elif (IO_MODE == PIC)
    #define _MAX_PAYLOAD_SIZE   (sizeof(Frame) - 6 - _PREAMBLE_SIZE)
#endif

    if (nSize > _MAX_PAYLOAD_SIZE) {
		// Too big
		mnt_debug_printf(gThisModule, "%s: Frame too big %d (limit %d)\r\n", __func__, nSize, _MAX_PAYLOAD_SIZE);
		return;
	}
    #undef _MAX_PAYLOAD_SIZE

    // Now we build the frame
    // MNT-1131 The preamble is required to wakeup MCU from its slumber
    memset(Frame, 0x7E, _PREAMBLE_SIZE);
	// First Char
	Frame[_PREAMBLE_SIZE] = HEADER_TO_ESCAPE;

#if (IO_MODE == EFM) || defined _WILDCAT
    nIndex = _PREAMBLE_SIZE + 1;
	// Length
	nIndex += mph_AddWithEscape(&Frame[nIndex], nSize + 1); // One is for checksum byte
	// Cmd
	nIndex += mph_AddWithEscape(&Frame[nIndex], cmd);
	// Add parameters (if any) and compute checksum
    {
        uint16_t i = 0;
        uint8_t checksum = cmd;

	    for (i = 0; i < nSize; i++) {
            checksum += pParameters[i];
		    nIndex   += mph_AddWithEscape(&Frame[nIndex], pParameters[i]);
	    }
        checksum ^= 0xFF;
	    // Add Checksum
	    nIndex += mph_AddWithEscape(&Frame[nIndex], checksum);
    }
//  #if defined _WILDCAT
    Frame[nIndex++] = '\r'; // <CR>
    Frame[nIndex++] = '\n'; // <LF>
//  #endif
#elif (IO_MODE == PIC)
	Frame[_PREAMBLE_SIZE + 1] = (uint8_t)nSize;
	Frame[_PREAMBLE_SIZE + 2] = cmd;
	mnt_memcpy(&Frame[_PREAMBLE_SIZE + 3], pParameters, nSize);
    nIndex = _PREAMBLE_SIZE + 3 + nSize;    // 3 = Dollar + size + command
    Frame[nIndex++] = 0x0D;
#endif

    core_utilsDebugDumpMemory(MODULE_MCU, __func__, Frame, nIndex); // MNT-1749
#if defined(_TRAILER)
    switch(mode) { // MNT-2797
  #if defined(SUPPORT_BLE_DFU)
        case UART_TARGET_APP:
            if (mnt_isNalaMuxUpdatePortRedirectionActive()) {
                mp_uartAppWrite(Frame, nIndex); // Send frame to provisioning UART
            }
            break;
  #endif
        default:
        	mp_sendToUart(Frame, nIndex); // Send frame to MCU UART
            break;
    }
#else
	mp_sendToUart(Frame, nIndex);
#endif

    #undef _PREAMBLE_SIZE
}

/*
 *
 * mph_kickWatchdog
 *
 * Purpose: Kick the watchdog
 *
 */
void mph_kickWatchdog(uint32 time )
{
#if defined(_LIONESS) || defined(_PANTHER) || (defined _WILDCAT)
	uint8 Parameters[3] = {0};

	if (time) {
        if (gMonetData.overrideMCUWD) { // MNT-1889
            Parameters[0] = 0xFF;
            Parameters[1] = 0xFF;
            Parameters[2] = 0xFF;
        } else {
            Parameters[0] = time & 0xFF;
            Parameters[1] = (time & 0xff00)   >> 8;
            Parameters[2] = (time & 0xff0000) >> 16;
        }
        mph_sendPicFrame(KICK_COMMAND, Parameters, sizeof(Parameters));
	} else {
		mph_sendPicFrame(KICK_COMMAND, NULL, 0);
	}
#else
	uint8 Parameters[3] = {0};

	if (time) {
        if (gMonetData.overrideMCUWD) { // MNT-1889
            Parameters[0] = 0xFF;
            Parameters[1] = 0xFF;
        } else {
            Parameters[0] = time & 0xFF;
            Parameters[1] = (time & 0xff00)   >> 8;
        }
        if (time < 60) {
            Parameters[2] = 1;
        }
        mph_sendPicFrame(KICK_COMMAND, Parameters, sizeof(Parameters));
	} else {
		mph_sendPicFrame(KICK_COMMAND, NULL, 0);
	}
#endif
    //mnt_debug_printf(gThisModule, "%s: Kick WD %d\r\n", __func__, time);
}

/*
 *
 * mph_resetSystem
 *
 * Purpose: Hardware reset of the system
 *
 */
void mph_resetSystem(uint8 time)
{
#ifdef _TRAILER
    mnt_startHardResetRetrySeq((uint32_t)time);
#else
    mph_sendPicFrame('H', &time, sizeof(time));
#endif
}

/*
 *
 * mph_setExtendedSleep
 *
 * Purpose:
 *
 */
void mph_setExtendedSleep(uint32 sleepTime)
{
	uint8 Parameters[4];

	Parameters[0] = sleepTime & 0xFF;
	sleepTime >>= 8;
	Parameters[1] = sleepTime & 0xFF;
	sleepTime >>= 8;
	Parameters[2] = sleepTime & 0xFF;
	mph_sendPicFrame('S', Parameters , 4);
}

/*
 *
 * mph_powerBasebandPower
 *
 * Purpose:
 *
 */

void mph_powerBasebandPower(uint8 mode)
{
#if (IO_MODE != EFM)
	uint8 Parameters[1];

	Parameters[0] = mode;
	mph_sendPicFrame('P', Parameters, sizeof(Parameters));
#else
    UNUSED_PARAMETER(mode);
#endif
}


#if (CHIPSET_VENDOR	==	MSFT)
void mph_IO_RxPeripheral(void *MsgBufferP, uint32 size)
{
	if (mph_uartCallback != NULL) {
		(mph_uartCallback)((uint8 *)MsgBufferP, size);
	}
}
#endif

/*
 *
 * mph_IO_RxData
 *
 * Purpose: process received data from UART
 *
 */
void mph_IO_RxData(void *MsgBufferP, uint32 size)
{
#if defined _TRAILER || defined _WILDCAT    // MNT-2196
	        uint8	*pData      = (uint8 *)MsgBufferP;
	        uint8	Current     = '\0';
	        uint32	i           =  0;
	static	FLAG    fInEscape   = FALSE;
#if (!defined SUPPORT_AC61_CAMERA) && (!defined SUPPORT_ZAZU_CAMERA) //Data of image is too large to print
    core_utilsDebugDumpMemory(MODULE_MCU, __func__, pData, size); // MNT-1749
#endif
    // MNT-2863
    for (i = 0; i < size; i++) {
        Current = *pData++;

        // MNT-1801
        if (HEADER_TO_ESCAPE == Current) {
            if (IO_WAIT_FOR_DOLLAR != gRxState) {
                mnt_debug_printf(MODULE_MCU, "%s: Unexpected '%c'\r\n", __func__, HEADER_TO_ESCAPE);
            }
            gRxState = IO_GET_FRAME_LENGTH;
        } else {
            if (fInEscape) {
                // We previsouly got the escape character
                fInEscape = FALSE;
                if (Current == ESCAPE_4_HEADER) {
                    Current = HEADER_TO_ESCAPE;
                } else if (Current == ESCAPE_4_ESCAPE) {
                    Current = ESCAPE_CHAR;
                } else {
                    // Error, restart hunting
                    gRxState = IO_WAIT_FOR_DOLLAR;
                    continue;
                }
            } else {
                // We were not in escape mode
                if (Current == ESCAPE_CHAR) {
                    fInEscape = TRUE;
                    continue;
                }
            }
            // Now Current hold a valid character
            switch (gRxState) {
            case IO_GET_FRAME_LENGTH:
                gRxLength   = Current - 1;
                gRxState    = IO_GET_COMMAND;
                break;
            case IO_GET_COMMAND:
                gRxChecksum = Current;
                gRxCommand  = Current;
                gRxCount    = 0;
                gRxState    = IO_GET_PARAMETERS;
                break;
            case IO_GET_PARAMETERS:
                gRxChecksum += Current;
                if (gRxCount == gRxLength) {
                    if (gRxChecksum == 0xFF) {
                        if (gMonetData.ManufacturingMode) {
                            // Report to UART
                            mph_HandleResponse(gRxCommand, gRxData, gRxLength);
                        } else {
                            // Execute the command
                            mph_HandleCommand(gRxCommand, gRxData, gRxLength);
                        }
                   } else {
                        // MNT-2196
                        mnt_debug_printf(MODULE_MCU, "%s: Unexpected checksum %d input %d\r\n", __func__, gRxChecksum, Current);
                    }
                    gRxState = IO_WAIT_FOR_DOLLAR;
                } else {
                    if (gRxCount < sizeof(gRxData)) {
                        gRxData[gRxCount++] = Current;
                    } else {
                        gRxState = IO_WAIT_FOR_DOLLAR;
                    }
                }
                break;
            default:
                gRxState = IO_WAIT_FOR_DOLLAR;
                break;
            }
        }
    }
#else	// #ifdef _TRAILER

	uint8	*pData;
	uint8	Current;
	uint32	i;

	pData = (uint8 *)MsgBufferP;
    core_utilsDebugDumpMemory(MODULE_MCU, __func__, pData, size); // MNT-1749

	for (i = 0; i < size; i++) {
		Current = *pData++;
		switch (gRxState) {
		case IO_WAIT_FOR_DOLLAR:
            if (Current == '$') {
				gRxState = IO_GET_FRAME_LENGTH;
            }
            break;
        case IO_GET_FRAME_LENGTH:
            gRxLength = Current;
            gRxState = IO_GET_COMMAND;
            break;
        case IO_GET_COMMAND:
            gRxCommand = Current;
			gRxCount = 0;
            gRxState = IO_GET_PARAMETERS;
            break;
        case IO_GET_PARAMETERS:
			if (gRxCount == gRxLength) {
				if (Current == 0x0D) {
					if (gMonetData.ManufacturingMode) {
						// Report to UART
						mph_HandleResponse(gRxCommand, gRxData, gRxLength);
					} else {
						// Execute the command
						mph_HandleCommand(gRxCommand, gRxData, gRxLength);
					}
				}
                gRxState = IO_WAIT_FOR_DOLLAR;
			} else {
				if (gRxCount < sizeof(gRxData)) {
					gRxData[gRxCount++] = Current;
				} else {
					gRxState = IO_WAIT_FOR_DOLLAR;
				}
            }
            break;
		default:
			gRxState = IO_WAIT_FOR_DOLLAR;
			break;
		}
    }
#endif	// #ifdef _TRAILER
}

void mph_HandleJumpResponse(uint8 *pData, uint8 size) { // MNT-2797
#if defined(_TRAILER)
    if (prod_isNala()) {
        if ((pData[0] == 'M') && (pData[1] == 'U') && (size >= 2)) {
            mnt_debug_printf(MODULE_MCU, "%s: received \r\n", __func__);
        }
        if ((pData[0] == 'M') && (pData[1] == 'R') && (size >= 2)) { // MNT-2797
            mnt_debug_printf(MODULE_MCU, "%s: Mux Reset after update \r\n", __func__);
            mph_HandleBootloaderResponse('j', pData, size);
        }
        if ((pData[0] == 'M') && (pData[1] == 'Q') && (size >= 3)) {
        }
    }
#endif
}

#if defined _TRAILER && defined(SUPPORT_BLE_DFU)
void mnt_requestNalaInformation(uint8_t target) {
    uint8_t parameter[1];

    parameter[0] = target;

    mph_sendPicFrame('V', parameter, 1);     // Get the informtion
}

void mnt_requestComponentInformation(uint8_t component, uint8_t mask) { // MNT-2812
    uint8_t parameter[2] = {0};

    parameter[0] = component;
    parameter[1] = mask;

    mph_sendPicFrame(0xAA, parameter, 2);     // Get the information
}

void mnt_requestComponentInformationViaPT(uint8_t component, uint8_t mask) { // MNT-2874
    uint8_t parameter[2] = {0};

    parameter[0] = component;
    parameter[1] = mask;

    mnt_SimbaBleCommand(0xAA, parameter, 2);     // Get the information
}

#endif //#if defined _TRAILER && defined(SUPPORT_BLE_DFU)

#if (defined _TRAILER && defined(SUPPORT_BLE_DFU)) || (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
/* 
Example of Simba BLE Mac Address:
00092F92:  24 0B AA 02 02 01 06 85 04 02 67 0B FC 51           $.........g..Q
Frame is passed as is, so it has MCU header:
    24 - $
    0B - 11 bytes of payload
    AA - Information payload
02 - Simba BLE MCU
02 - Mask for MAC address
01 - Bit for MAC address
06 - 6 bytes of MAC address
85 04 02 67 0B FC - MAC address
    51 - Checksum
MCU header and checksum should be ignored in such case

Example of Nala MUX FW version:
00092F90:  05 01 00 04 01 00 00 02                             ........
05 - Nala MUX
01 - Mask for version
00 - Version bit
04 - 4 bytes of version
01 00 00 02 - Version 1.0.0.2
*/

void mnt_handleComponentInformationResponse(uint8_t *pData, uint8_t size) {  // MNT-2812
    size_t      remainingSize   = size;
    size_t      realSize        = size;
    uint8_t *   pStart          = pData;
    uint8_t *   pCurrent        = pData;
    uint8_t     instanceId      = 0xff;
    core_ComponentTarget_et     componentTarget = COMPONENT_TARGET_NONE;
    core_ComponentInfoMask_et   informationMask = 0;

    core_utilsDebugDumpMemory(gThisModule, __func__, pData, size);

    // First check if it is a full frame or not (Simba BLE case)
    if (size >= 3 && HEADER_TO_ESCAPE == pData[0] && 0xAA == pData[2]) {
        remainingSize   = pData[1];     // Use the payload
        pCurrent       += 3;            // Skip prefix ('$', size and 0xAA)
        // Update variables to notify picloader
        pStart          = pCurrent;
        realSize        = remainingSize;
    }

    // Check if it is enough size for original mask and component target and get them
    // MNT-2983 Support one or multiple instances
    if (remainingSize >= 1) {
        componentTarget  = pCurrent[0];
        pCurrent        += 1;
        remainingSize   -= 1;
    }
    if (IS_MULTI_INSTANCE(componentTarget) && remainingSize >= 1) {
        instanceId       = pCurrent[0];
        pCurrent        += 1;
        remainingSize   -= 1;
    }

    // MNT-2983 Support 4 bytes bitmask
    if (remainingSize >= 1) {
        informationMask = pCurrent[0];
        if (IS_BIT_SET(informationMask, 7) && remainingSize >= 4) {
            informationMask  = (uint32_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pCurrent, 4);
            pCurrent        += 4;
            remainingSize   -= 4;
        } else {
            pCurrent        += 1;
            remainingSize   -= 1;   
        }
    }

    // Check that it is enough bytes for the information type part (index and length)
    while (remainingSize >= 2) {
        core_ComponentInfoType_et   infoIndex   = pCurrent[0];
        uint8_t                     length      = pCurrent[1];

        // Skip index and length
        pCurrent        += 2;
        remainingSize   -= 2;

        mnt_debug_printf(MODULE_MCU, "%s: target:%d infoIndex:%d \r\n", __func__, componentTarget, infoIndex);

        if (remainingSize >= length) {
            switch (infoIndex) {
            case COMPONENT_INFO_VERSION:
                switch (componentTarget) {
#if (defined _TRAILER && defined(SUPPORT_BLE_DFU))
                case COMPONENT_TARGET_NALA_MCU:
                    if (4 == length) {
                        gMainComponentMask &= ~COMPONENT_INFO_VERSION_MASK; // Clear this bit
                        memcpy(gIoVersion, pCurrent, MIN(4, sizeof(gIoVersion)));
                    }
                    break;
                case COMPONENT_TARGET_NALA_MUX_MCU:
                    if (4 == length) {
                        gPeriComponentMask &= ~COMPONENT_INFO_VERSION_MASK; // Clear this bit
                        memcpy(gMuxVersion, pCurrent, MIN(4, sizeof(gMuxVersion)));
                    }
                    break;
                case COMPONENT_TARGET_SIMBA_BLE_MCU:
                    if (4 == length) {
                        gPeriComponentMask &= ~COMPONENT_INFO_VERSION_MASK; // Clear this bit
                        mnt_setBLEVersion(pCurrent);
                    }
                    break;
                case COMPONENT_TARGET_SIMBA_MCU: // MNT-2977
                    if (4 == length) {
                        gMainComponentMask &= ~COMPONENT_INFO_VERSION_MASK; // Clear this bit
                        memcpy(gIoVersion, pCurrent, MIN(4, sizeof(gIoVersion)));
                        core_utilsDebugDumpMemory(gThisModule, __func__, gIoVersion, MIN(4, sizeof(gIoVersion)));

                    }
                    break;
#endif
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)  // MNT-2997
                case COMPONENT_TARGET_SLP01_MCU:
                    if (4 == length) {
                        memcpy(gIoVersion, pCurrent, sizeof(gIoVersion));
                        mnt_debug_printf(gThisModule, "%s:  SLP01_MCU Version: %d.%d.%d.%d\r\n", __func__, gIoVersion[0], gIoVersion[1], gIoVersion[2], gIoVersion[3]);
                    }
                    break;
                case COMPONENT_TARGET_CAMERA_NORDIC_MCU:
                    if (4 == length) {
                        mnt_cameraSetNordicMcuVersion(pCurrent, instanceId);
                    }
                    break;
                case COMPONENT_TARGET_CAMERA_ESP32_MCU:
                    if (4 == length) {
                        mnt_cameraSetESP32McuVersion(pCurrent, instanceId);
                    }
                    break;
#endif
                default:
                    // Simply skip
                    break;
                }
                if (mp_IsPicLoaderActive()) { // MNT-2797 MUX update is active and the request for get version was issued
                    mph_HandleGetVersionResponse(0xAA, pStart, (uint8_t)realSize);
                }
                break;
            case COMPONENT_INFO_MAC_ADDR:
                switch (componentTarget) {
#if (defined _TRAILER && defined(SUPPORT_BLE_DFU))
                case COMPONENT_TARGET_NALA_MCU:
                    if (6 == length) {
                        gMainComponentMask &= ~COMPONENT_INFO_MAC_ADDR_MASK; // Clear this bit
                        memcpy(gMacId, pCurrent, MIN(6, sizeof(gMacId)));
                    }
                    break;
                case COMPONENT_TARGET_SIMBA_BLE_MCU:    // MNT-2804
                    if (6 == length) {
                        gPeriComponentMask &= ~COMPONENT_INFO_MAC_ADDR_MASK; // Clear this bit
                        memcpy(gMacId, pCurrent, MIN(6, sizeof(gMacId)));
                    }
                    break;
#endif
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)   // MNT-2997
                case COMPONENT_TARGET_SLP01_MCU:
                    if (6 == length) {
                        memcpy(gMonetData.ModuleMac, pCurrent, MIN(6, sizeof(gMonetData.ModuleMac)));
                        mnt_debug_printf(gThisModule, "%s Main MAC:%02X%02X%02X%02X%02X%02X\r\n", __func__, gMonetData.ModuleMac[0],gMonetData.ModuleMac[1],gMonetData.ModuleMac[2],gMonetData.ModuleMac[3],gMonetData.ModuleMac[4],gMonetData.ModuleMac[5]);
                    }
                    break;
                case COMPONENT_TARGET_CAMERA_NORDIC_MCU:
                    if (6 == length) {
                        char macStr[12+1] = {0};
                        sprintf(macStr, "%02X%02X%02X%02X%02X%02X", pCurrent[0], pCurrent[1], pCurrent[2], pCurrent[3], pCurrent[4], pCurrent[5]);
                        mnt_cameraSaveMac2CameraInfo(macStr, instanceId);
                    }
                    break;
#endif
                default:
                    // Simply skip
                    break;
                }
                break;
            case COMPONENT_INFO_HWID:
                switch (componentTarget) {
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)   // MNT-2997
                case COMPONENT_TARGET_CAMERA_ESP32_MCU:
                    if (3 == length) {
                        mntHwModel_t    model    = *pCurrent + (*(pCurrent+1)<<8);
                        mntHwRevision_t revision = *(pCurrent+2);
                        mnt_cameraSetHwModel(model, instanceId);
                        mnt_cameraSetHwRevision(revision, instanceId);
                        //mnt_debug_printf(gThisModule, "%s AC61HwId:%04xR%02x\r\n", __func__, model, revision);
                    }
                    break;
#endif
                default:
                    // Simply skip
                    break;
                }
                break;
        // MNT-3053 Get the information of charger chip
            case COMPONENT_INFO_CHARGER_CHIP:
                switch (componentTarget) {
#if defined _TRAILER && defined _LIONESS
                case COMPONENT_TARGET_NALA_CHARGER_CHIP:
                    if (10 == length) {
                        mnt_mfgTestChargerChipRead(pCurrent, length);
                    }
                    break;
#endif
                default:
                    // Simply skip
                    break;
                }
                break;
            default:
                // Simply skip
                break;
            }
            pCurrent        += length;
            remainingSize   -= length;
        } else {
            remainingSize = 0;      // To complete while loop
        }
    }
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)  // MNT-2997
    gMonetData.bGotCameraInfo[instanceId] = mnt_cameraInfoIsReady(instanceId);
    mnt_debug_printf(gThisModule, "%s Camera#%d bGotCameraInfo:%d\r\n", __func__, instanceId, gMonetData.bGotCameraInfo[instanceId]); 
#endif
}
#endif

#if defined NORDIC_DFU || defined SUPPORT_BLE_DFU // MNT-3196 For whoever support BLE
void mnt_handleBleInfoCommand(uint8_t *pData, uint8_t size) 
{
    switch (pData[0]) { // subcommand
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)   // MNT-2997
    case 'a':                          /* Broadcast state */
    {
        uint8_t cfgstate     = pData[1];
        uint8_t realState    = pData[2]; /* 0: OK, else: Failed */
        
        if (!realState) {
            mnt_cameraSetBroadcastState((broadcastSwitchT)cfgstate);
        }
        mnt_debug_printf(gThisModule, "%s: cmd:'a' cfgstate:%x, realState:%x\r\n", __func__, cfgstate, realState);
    }
    break;
    case 'b':                          /* Connect state */
    {
        uint8        channel   = pData[1];
        uint8        state     = pData[2];
        cameraTimeSt t         = {0};

        memcpy(&t.year, pData+3, sizeof(cameraTimeSt) - DAY_OF_WEEK_LEN); 
        mnt_debug_printf(gThisModule, "%s: cmd:'b' ch:%d, state:%x,%d-%d-%d %d:%d:%d\r\n", __func__, channel, state,
                                       t.year,
                                       t.month,
                                       t.day,
                                       t.hour,
                                       t.minute,
                                       t.second
        );
        mnt_cameraGlassBreakAck();
        mnt_sendToServer(EVT_GLASS_BREAK); // MNT-2986
    }
    case 'c':                          /* Connect state */
    {
        uint8 channel = pData[1];
        uint8 state   = pData[2];

        mnt_debug_printf(gThisModule, "%s: cmd:'c' ch:%d, state:%x\r\n", __func__, channel, state);
        mnt_cameraSetConnect(state);
        if(mnt_cameraIsCaptureNow()) {
            mnt_cameraCaptureStopTimer();
            mnt_cameraAllStartCapture(mnt_cameraGetCaptureType(), IMG_HIGH_DEFINITION);
            mnt_cameraSetCaptureNow(false);
        }
        // MNT-2793 request AC61 version once BLE connected
        mnt_debug_printf(gThisModule, "%s bGotCameraInfo:%d\r\n", __func__, gMonetData.bGotCameraInfo[channel]);         
        if( !gMonetData.bGotCameraInfo[channel] && state ) {
            mph_getComponentInfo(0, COMPONENT_TARGET_CAMERA_NORDIC_MCU, COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_MAC_ADDR_MASK);
            mph_getComponentInfo(0, COMPONENT_TARGET_CAMERA_ESP32_MCU,    COMPONENT_INFO_VERSION_MASK | COMPONENT_INFO_HWID_MASK);
        }
    }
    break;
#endif
    case 'e':
        mnt_restoreLED();              // MNT-3196 Restore LED to their normal state
    break;
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
    case 'g':                          /* Connect state */
    {
        glassBreakInfo gb;
        memcpy(&gb, pData, sizeof(glassBreakInfo));
        mnt_debug_printf(gThisModule, "%s: cmd:'g', id:%d, num:%d, level:%d, debounce:%d\r\n", __func__, gb.pinIdx, gb.imgNum, gb.level, gb.debounce); 		
    }
    break;
#endif
    case 'i':                          // MNT-3196 Information of paired BLE sensor 
    {
        uint8_t* pDataCurrent         = pData+1;
        uint8_t sensorType           = 0;
        uint8_t instanceId           = 0;
        uint8_t macId[BLE_MAC_HEX_LEN] = {0};
        uint8_t sensorNum            = 0;
        uint8_t sensorSeq            = 0;

        sensorType  = *(pDataCurrent++);
        instanceId  = *(pDataCurrent++);
        mnt_memcpy( macId, pDataCurrent, BLE_MAC_HEX_LEN );
        pDataCurrent += BLE_MAC_HEX_LEN;
        sensorNum   = *(pDataCurrent++);
        sensorSeq   = *(pDataCurrent++);

        mnt_debug_printf(gThisModule, "%s: cmd:'i', sensorType %d, instanceId %d, macId %02x%02x%02x%02x%02x%02x, sensorNum %d, sensorSeq %d\r\n", __func__, sensorType, instanceId, macId[0], macId[1], macId[2], macId[3], macId[4], macId[5], sensorNum, sensorSeq);
    }
    break;
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
    case 'm':                          /* MAC Ack */
        mnt_debug_printf(gThisModule, "%s: cmd:'m', channel:%d, moduleMac:%02x%02x%02x%02x%02x%02x\r\n", __func__, pData[1], pData[9], pData[10], pData[11], pData[12], pData[13], pData[14]);	
        mnt_cameraSetModuleBleMac(&pData[9]);
    break;
    case 'M':
        //mnt_debug_printf(gThisModule, "%s: cmd:'M':%02x%02x%02x%02x%02x%02x\r\n", __func__, pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
        //mnt_cameraSetModuleBleMac(&pData[2]);
        mnt_debug_printf(gThisModule, "%s: cmd:'M', channel:%d, AC61 MAC:%02x%02x%02x%02x%02x%02x\r\n", __func__, pData[8], pData[9], pData[10], pData[11], pData[12], pData[13], pData[14]);	
    break;    
#endif
    case 'n':                          // MNT-3196 BLE scan now response 
    {
        uint8_t* pDataCurrent         = pData+1;
        uint8_t sensorType           = 0;
        uint8_t macId[BLE_MAC_HEX_LEN] = {0};
        int16_t signalStrength            = 0;

        sensorType  = *(pDataCurrent++);
        mnt_memcpy( macId, pDataCurrent, BLE_MAC_HEX_LEN );
        pDataCurrent += BLE_MAC_HEX_LEN;
        signalStrength = (int16_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pDataCurrent, 2);

        mnt_debug_printf(gThisModule, "%s: cmd:'i', sensorType %d, macId %02x%02x%02x%02x%02x%02x, signalStrength %d\r\n", __func__, sensorType, macId[0], macId[1], macId[2], macId[3], macId[4], macId[5], signalStrength);
    }
    break;
    case 'o':                          // MNT-3196 offer scan parameter response 
    {
        uint16_t window    = (uint16_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pData + 1, 2);
        uint16_t duration  = (uint16_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pData + 3, 2);
        uint32_t interval  = (uint32_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pData + 5, 4);

        mnt_debug_printf(gThisModule, "%s: cmd 'o', window %d, duration %d, interval %d\r\n", __func__, window, duration, interval);
    }
    break;
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
    case 'p':                          /* Slow/Fast Mode */
    {
        uint8_t  channel   = pData[1];
        uint16_t interval  = pData[2] | pData[3]<<8;

        mnt_debug_printf(gThisModule, "%s: cmd:'p', channel:%d, interval:%d\r\n", __func__, channel, interval);
    }
    break;
#endif
    case 'r':                          // MNT-3196 BLE pairing response
    {
        uint32_t mask  = (uint32_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pData+1, 4);

        mnt_debug_printf(gThisModule, "%s: cmd:'r', mask:0x%x\r\n", __func__, mask);
    }
    break;
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
    case 's':                          /* Camera transmition mode switch */
    {
        uint8_t mode     = pData[1];
        if(1 == mode){
            mnt_debug_printf(gThisModule, "%s: Camera transmition mode: BLE\r\n", __func__);
        }
        else if(2 == mode){
            mnt_debug_printf(gThisModule, "%s: Camera transmition mode: CAN\r\n", __func__);
        }
    }
    break;
    case 't':                          /* White List Ack */
        mnt_debug_printf(gThisModule, "%s: cmd:'t'\r\n", __func__);			
    break;
#endif
    case 'u':                          // MNT-3196 BLE unpairing response
    {
        uint32_t mask = (uint32_t)core_bufferUnpackUint(CORE_PACK_FORMAT_LITTLE_ENDIAN, pData+1, 4);
        uint8_t* MAC  = pData + 5 ;

        mnt_debug_printf(gThisModule, "%s: cmd:'u', mask:0x%x, mac %02x%02x%02x%02x%02x%02x\r\n", __func__, mask, MAC[0],MAC[1],MAC[2],MAC[3],MAC[4],MAC[5]);
    }
    break;
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
    case 'w':                          /* White List Ack */
        mnt_debug_printf(gThisModule, "%s: cmd:'w'\r\n", __func__);			
    break;
#endif
    default:
        mnt_debug_printf(gThisModule, "%s: camera Unimplemented response: <%c>\r\n", __func__, pData[0]);			
    break;
    }
}
#endif

/*
 *
 * mph_HandleCommand
 *
 * Purpose: 
 *
 */
static void mph_HandleCommand(uint8 Command, uint8 *pData, uint8 size)
{
	int index;
    
//    mnt_debug_printf(MODULE_IFADC, "%s: command: %c\r\n", __func__, Command);

#if (IO_MODE == EFM) || (IO_MODE == PIC) // MNT-2341
#ifndef _TRAILER // MNT-2591
    // MNT-1791  Restart the MCU response WD timer
    mnt_TimerRestart(&mcuCommWatchdogTimer);
#endif
#endif

    mnt_debug_printf(MODULE_MCU, "%s: command:%c size:%d \r\n", __func__, Command, size);

	switch (Command) {
#if (IO_MODE == PIC || IO_MODE == EFM)
  #if (IO_MODE == EFM) || defined (_WILDCAT)
    case '@':
        #if defined NORDIC_DFU
            #define _IO_FILE     gDfuIoBinFile
        #else
            #define _IO_FILE     ioBinary
        #endif
        // MNT-2299 Verify if the response was from the boot loader
        if ((RD_VER == pData[0]) && (0x01 == pData[1]) && !mp_IsPicLoaderActive() && mp_verify(_IO_FILE)) {
            mnt_debug_printf(gThisModule, "%s: Update from boot loader \r\n", __func__);
        #if !defined NORDIC_DFU
            // The mcu partition is not present, update the mcu partition
            mp_UpdatePic(ioBinary, UPDATE_TYPE_RECOVERY);    // MNT-2953
        #else
            mph_HandleBootloaderResponse(Command, pData, size);
        #endif
            gMonetData.isMcuRecovery = true;    // MNT-2627 MCU firmware is recovered
        } else {
            mph_HandleBootloaderResponse(Command, pData, size);
        }
        #undef _IO_FILE
		break;
  #endif
	case 'a':
		mph_HandleAdc(pData, size);
		break;
#endif
#ifdef _TRAILER
	case 'c':
        gBaudrateChange = pData[0];
        mnt_debug_printf(gThisModule, "%s:  %s Config  %d, %02X\r\n", __func__, size >= 3 && 1 == pData[2] ? "LEUART" : "UART", pData[0], pData[1]);
        mnt_csMcuFeedback(Command, pData, size);    // Notify cargo sensor module
        break;
	case 'd':
        // This is a feedback from 'D' command (data to the peripheral device)
        mnt_csMcuFeedback(Command, pData, size);    // Notify cargo sensor module
        break;
    case '#':
		if (mph_uartCallback != NULL) {
			(mph_uartCallback)(pData, size);
		}
		break;
#endif
	case 'b':
		mnt_debug_printf(gThisModule, "%s(%c):  WD Timer %ds\r\n", __func__, Command, pData[1]*256+pData[0]);
        gCommStatusMask |= COMM_STATUS_SET_DEFAULT_WD_MASK;
		break;
#if defined (_TRAILER)
    case 'e':
		mph_HandleGpio(pData, size);
		break;
#endif
	case 'f':
        mnt_debug_printf(gThisModule, "%s(%c%c):  Addr:0x%X size:%d\r\n", __func__, Command, pData[0], pData[1], size);
        if (pData[0] == 'R') {
            if (size == 6) { // 1byte subcommand, 2byte address, 4byte value
#ifdef _MINITRACK
                {
                    uint32_t temp;
                    if (pData[1] == UID_ADDRESS) {
                        temp = (pData[5]<<24)+(pData[4]<<16)+(pData[3]<<8)+pData[2];
                        mnt_debug_printf(gThisModule, "%s(%c%c): UID:0x%04X\r\n", __func__, Command, pData[0], temp);
                        gMonetData.unitId = temp;
                        gCommStatusMask |= COMM_STATUS_SET_UID_UPDATED_MASK;
                        // Send pending reports if any
                        mnt_SendPendingReports(PRT_WAIT_FOR_UID);
                    }
                }
#endif
            }
        } else if (pData[0] == 'W') {
#ifdef _MINITRACK
            {
                uint32_t temp;
                if (pData[1] == UID_ADDRESS) {
                    temp = (pData[5]<<24)+(pData[4]<<16)+(pData[3]<<8)+pData[2];
                    mnt_debug_printf(gThisModule, "%s(%c%c):  UID 0x%04X %d state:%d\r\n", __func__, Command, pData[0], pData[1], temp, pData[6]);
                    if (pData[6] == 1) {
                        gMonetData.unitId = temp;
                        // Send pending reports if any
                        mnt_SendPendingReports(PRT_WAIT_FOR_UID);
                    }
                }
            }
#endif
        }
		break;
#if defined (_TRAILER) || defined (_WILDCAT)
    case 'g':
        // MNT-955
        if (size == 3) {
            // Copy the state into the proper position for event handler
            pData[1] = pData[2];
            mph_HandleGpio(pData, size);
        }
		break;
#endif
	case 'h':
		mnt_debug_printf(gThisModule, "%s:  Hard reset confirmation: %d\r\n", __func__,pData[0]+((uint32_t)pData[1]<<8)+((uint32_t)pData[2]<<16));
#ifdef _TRAILER
        mnt_stopHardResetRetrySeq(); // MNT-3024
#endif
		break;
    case 'i':
#if (IO_MODE == EFM)
        gLastInterruptStatus = (pData[3] << 24) + (pData[2] << 16) + (pData[1] << 8) + (pData[0] << 0);
#elif (IO_MODE == PIC)
        gLastInterruptStatus =        (0 << 24) + (pData[2] << 16) + (pData[1] << 8) + (pData[0] << 0);
#else
        gLastInterruptStatus =        (0 << 24) +        (0 << 16) + (pData[1] << 8) + (pData[0] << 0);
#endif
// MNT-2141 make sure that any product using PIC type MCU sets the timer bit when the interrupt status is 0
#if (IO_MODE == PIC)
        if (0 == gLastInterruptStatus) {
            gLastInterruptStatus |= WAKE_UP_TIMER; // MNT-741 & MNT-742 SA MCU does not use this bit when waking on timer
        }
#endif
		gInterruptStatus |= gLastInterruptStatus;
		mnt_debug_printf(gThisModule, "%s:  Interrupt %08X (%s)\r\n", __func__, gInterruptStatus, mnt_mcuGetInterruptStr(gInterruptStatus));
        if (0 != gInterruptStatus) {    // MNT-855 Do not perform RTC time adjustment after reset
            bool shouldInterruptBeHandled = true;

            // MNT-680 Adjust once only when RTC time is known
            if (! (gCommStatusMask & COMM_STATUS_SET_RTC_UPDATED_MASK) && (gCommStatusMask & COMM_STATUS_GET_RTC_MASK)) {
                if (!gfBootIsPowerUp) { // MNT-834
                    // MNT-2331, fixed mcu interrupts handler
                    shouldInterruptBeHandled = false;   // MNT-839 Will be handled by timer to guarantee adjustment happens before event generations
                    mnt_TimerUpdateViaRtc();    //MNT-2623 Adjust once only when RTC time is known
                }
                gCommStatusMask |= COMM_STATUS_SET_RTC_UPDATED_MASK;
            }
            mnt_debug_printf(gThisModule, "%s: shouldInterruptBeHandled:%d, gInterruptStatus:0x%x\r\n", __func__, shouldInterruptBeHandled, gInterruptStatus);
            if (shouldInterruptBeHandled) {
#if (IO_MODE == PIC)
                mnt_ServiceAccelerometerEvent(gInterruptStatus);
#endif
                mnt_powerMcuInterruptHandler(gInterruptStatus);
            }
        }
        gCommStatusMask |= COMM_STATUS_GET_INT_STATUS_MASK;
#ifdef _PANTHER
        {
            extern bool isSystemAwake(void);
            extern void mnt_setModeAwake(void);
            if (!isSystemAwake()) {
                mnt_setModeAwake();
            }
        }
#endif
		break;
	case 'k':
#ifdef _LIONESS
    {
        uint32_t wdTime; 
#ifdef _TRAILER
        wdTime = (pData[0]+((uint32_t)pData[1]<<8));
#else
        wdTime = (pData[0]+((uint32_t)pData[1]<<8)+((uint32_t)pData[2]<<16));
#endif
        mnt_debug_printf(gThisModule, "%s(%c):  WD Left %d (%d)\r\n", __func__, Command, wdTime, size);
    }
#else
		mnt_debug_printf(gThisModule, "%s:  WD Left %d\r\n", __func__, pData[0]+((uint32_t)pData[1]<<8));
#endif
        gCommStatusMask |= COMM_STATUS_SET_DEFAULT_WD_MASK;
		break;
    case 'l':
    {
        uint8_t led = pData[0] & 0x7f; // MNT-2989 0=GPS, 1=CELL
     #ifdef DEBUGF
        uint8_t repeat = pData[0] & 0x80;

        mnt_debug_printf(gThisModule, "%s: Led:(%d)%s Mode:%d On:%d Off:%d\r\n", __func__, led, (led == 0) ? "GPS" : (led == 1) ? "CELL" : "FIRE", repeat, pData[1], pData[2]);
    #endif
        UNUSED_BUT_SET_VARIABLE(led);
#ifdef _TRAILER
        mnt_stopLedRetrySeq(led, pData[1], pData[2]); // MNT-3128 MNT-2989
#endif
    }
        break;
	case 'm':
		mnt_debug_printf(gThisModule, "%s:  G-sensor TH: %d, Duration: %d\r\n", __func__, pData[0], pData[1]);
		break;
	case 'M':
        index = 0;
        while (index < size) {
            uint8_t inc = sizeof(uint8_t) /* letter */ + sizeof(uint8_t) /* default parameter */;
            switch (pData[index]) {
            case 'T':
				mnt_debug_printf(gThisModule, "%s:  G-sensor Threshold: %d\r\n", __func__, pData[index + 1]);
                break;
            case 'D':
                mnt_debug_printf(gThisModule, "%s:  G-sensor  Duration: %d\r\n", __func__, pData[index + 1] + (pData[index + 2] << 8));
                inc = sizeof(uint8_t) /* letter */ + sizeof(uint16_t)   /* 16-bit duration */;
                break;
            case 'M':
				mnt_debug_printf(gThisModule, "%s:  G-sensor    Motion: %d\r\n", __func__, pData[index + 1]);
                break;
            default:
                mnt_debug_printf(gThisModule, "%s:  G-sensor  Other(%c): %d\r\n", __func__, pData[index], pData[index + 1]);
                break;
			}
            index += inc;
        }
		break;
	case 'n':
#ifdef _TRAILER
		mnt_debug_printf(gThisModule, "%s:  Heartbeat %d\r\n",              __func__,  (pData[2] << 16) + (pData[1] << 8) + (pData[0] << 0));
#else
        mnt_debug_printf(gThisModule, "%s:  Will Sleep for %d secs\r\n",    __func__,  (pData[2] << 16) + (pData[1] << 8) + (pData[0] << 0));
#endif
		break;
#if defined(_TRAILER)
	case 'p':
#if defined(_TRAILER)
		mnt_debug_printf(gThisModule, "%s:  Power %d  %d\r\n", __func__, pData[0], pData[1]);
        mnt_csMcuFeedback(Command, pData, size);    // Notify cargo sensor module
#else
		mnt_debug_printf(gThisModule, "%s:(p)  mask:%d, lowPowerThreshold:%d, highPowerThreshold:%d, sourceDebounce:%d, thresholdDebounce:%d\r\n", __func__, pData[0], (uint16_t)(pData[1] + (pData[2] << 8)), (uint16_t)pData[3] + (pData[4] << 8), (uint16_t)pData[5] + (pData[6] << 8), (uint16_t)pData[7] + (pData[8] << 8));
#endif // #ifdef _TRAILER
		break;
#endif // #if defined(_TRAILER)
    case 'q':
        if (1 == pData[0]) {
//            mnt_debug_printf(gThisModule, "%s:  GPIO(all) %02X\r\n", __func__, pData[1]);
            {
                uint8 index = 0;
                uint8 gpio  = 0;
                uint8 state = 0;
                for(index = 0;index < MAX_GPIO; index++) {
                    gpio = 1 << index;
                    state = (pData[1] & gpio) >> index;
                    mi_handleGpioEvents(index + 1, state, ACTION_NORMAL);
                }
            }
        } else if (2 == pData[0]) {
#ifdef _LIONESS
#ifdef _TRAILER
//            mnt_debug_printf(gThisModule, "%s:  GPIO(%d) %d\r\n", __func__, pData[1], pData[2]);
#else
//            mnt_debug_printf(gThisModule, "%s:  GPIO(%d) %02x %d\r\n", __func__, pData[1], pData[2], pData[3]);
            if (size >= 3) {
                uint8 gpio = pData[1];
                uint8 direction = (pData[2] & GPIO_DIRECTION);
                uint8 state = pData[3];
//                mnt_debug_printf(gThisModule, "%s:  GPIO(%d) %02x %d\r\n", __func__, gpio, direction, state);
                if (direction > 0) {
//                    mnt_debug_printf(gThisModule, "%s:  GPIO(%d) %d\r\n", __func__, gpio + 1, state);
                    mi_handleGpioEvents(gpio + 1, state, ACTION_NORMAL);
                }
            }
#endif
#else
            mnt_debug_printf(gThisModule, "%s:  GPIO(%d) %d\r\n", __func__, pData[1], pData[2]);
#endif
#ifdef _TRAILER
        } else if (3 == pData[0]) { // 1-wire
            if (2 == pData[1]) {    // MNT-842 1-wire discovery 
                gMonetData.OneWireNumSensors = pData[2];
                mnt_debug_printf(gThisModule, "%s:  1W sensors discovered: %d\r\n", __func__, gMonetData.OneWireNumSensors);
                if (gMonetData.OneWireNumSensors > 0) {
                    mnt_1wireStartTimer();
                } else {
                    mnt_TimerStop(&oneWireTempTimer);
                    mp_OneWireEnable(FALSE);    // MNT-1878
                }
            } else {
                mnt_debug_printf(gThisModule, "%s:  1W enabled: %d\r\n", __func__, pData[1]);
            }
#endif
        }
        break;
	case 'r':
        {
            gCommRtcTime = (pData[3] << 24) + (pData[2] << 16) + (pData[1] << 8) + (pData[0] << 0);
		    mnt_debug_printf(gThisModule, "%s:  RTC Time %d\r\n", __func__, gCommRtcTime);
            gCommStatusMask |= COMM_STATUS_GET_RTC_MASK;
        #ifdef BATTERY_ONLY_DEVICE
            {
                // MNT-3117 update nvmHibernateSecondsTotal, minus the time from APP started to gCommRtcTime got, but gCommRtcTime is 0 when power up, don't minus when power up
                // MNT-3131 update and save to NVM
                uint32_t HibernateSecondsTotal = 
                    (gCommRtcTime > TIME_FROM_APP_START_TO_GET_MCU_RTC ? 
                        (gCommRtcTime - TIME_FROM_APP_START_TO_GET_MCU_RTC) : 0);  
                mnt_updateNvm(NVM_HIBERNATE_SECONDS, TRUE, HibernateSecondsTotal);
                mnt_debug_printf(gThisModule, "%s: nvmHibernateSecondsTotal %d\r\n", __func__, gMonetData.nvmHibernateSecondsTotal);
                mnt_writeNvm();
            }
        #endif
            if (gMonetData.TestTimeDrift > 0) {    // MNT-1077
                gCommRtcTime += gMonetData.TestTimeDrift;
                mnt_debug_printf(gThisModule, "%s:  RTC Time %d, drift: %d\r\n", __func__, gCommRtcTime, gMonetData.TestTimeDrift);
            }

            // MNT-2623 Adjust once only when interupt is known
            // MNT-855 Do not perform RTC time adjustment after reset
            // MNT-680 Adjust once only when RTC time is known
            // MNT-834
            if ((0 != gInterruptStatus) &&
                (! (gCommStatusMask & COMM_STATUS_SET_RTC_UPDATED_MASK)) &&
                (! gfBootIsPowerUp)) {
                mnt_TimerUpdateViaRtc();
            }
        }
		break;
    case 's':
    #ifdef DEBUGF
        {
            uint32  stackFreeSpace = (pData[0] << 24) + (pData[1] << 16) + (pData[2] << 8) + pData[3];
            mnt_debug_printf(gThisModule, "%s:  Stack Free Space %d\r\n", __func__, stackFreeSpace);
        }
    #endif
        break;
    case 't':
    #ifdef DEBUGF
        {
            uint16  gIoModel = pData[0] * 256 + pData[1];
            uint8   gIoRevision = pData[2];
            mnt_debug_printf(gThisModule, "%s:  PIC Model: %04X Rev: %02X\r\n", __func__, gIoModel, gIoRevision);
        }
    #endif
        // Now we know the model so we can set the GPIO properly
        gfGotModel = TRUE;
        mp_setGPIO();
        gCommStatusMask |= COMM_STATUS_GET_MODEL_MASK;
        break;
    case 'u':
        gBootFlags = (pData[0] << 24) + (pData[1] << 16) + (pData[2] << 8) + (pData[3] << 0); // MNT-514
		mnt_debug_printf(gThisModule, "%s:  Boot %08X\r\n", __func__, gBootFlags);
#if defined _TRAILER && defined DEBUGF
        mnt_DecodeResetCause(gBootFlags);
#endif
        gCommStatusMask |= COMM_STATUS_GET_BOOT_STATUS_MASK;
		break;
	case 'v':
        if (4 == size) {
    		memcpy(gIoVersion, pData, sizeof(gIoVersion));
            // MNT-2945, debug message
            mnt_debug_printf(gThisModule, "%s:  MCU Version: %d.%d.%d.%d\r\n", __func__, gIoVersion[0], gIoVersion[1], gIoVersion[2], gIoVersion[3]);
        }
#if defined(_TRAILER) && defined(SUPPORT_BLE_DFU)
        if (5 == size) { // MNT-2703 BLE version response
            switch(pData[0]) {
                case 'M':
                    memcpy(gMuxVersion, &pData[1], sizeof(gMuxVersion));
                    mnt_debug_printf(gThisModule, "%s:  MUX Version: %d.%d.%d.%d\r\n", __func__, gMuxVersion[0], gMuxVersion[1], gMuxVersion[2], gMuxVersion[3]);
                    break;
                case 'I':
                    memcpy(gMacId, &pData[1], sizeof(gMacId));
                    mnt_debug_printf(gThisModule, "%s:  MAC: %d.%d.%d.%d.%d.%d\r\n", __func__, gMacId[0], gMacId[1], gMacId[2], gMacId[3], gMacId[4], gMacId[5]);
                    break;
                default:
            		handleBLEResponse(pData, size);
                    if (prod_isSimba()) {
                        gPeriComponentMask &= ~COMPONENT_INFO_VERSION_MASK; // Clear this bit
                    }
                    break;
            }
        }
#endif
#if (IO_MODE == EFM) || defined _WILDCAT    /* MNT-2222 */
        gfGotModel = TRUE;
        if (mp_IsPicLoaderActive()) {
            // Notify MCU loader first
            mph_HandleBootloaderResponse(Command, pData, size);
        }
        mp_setGPIO();
        if (gMonetData.fReconfigMcuAccFlag) {
            // MNT-1530, force to reconfigure
            gMotionState = MOTION_STATE_NONE;
            // MNT-786 Reconfigure accelerometer to last known state
            mnt_reconfigureMcuMotionState();
            gMonetData.fReconfigMcuAccFlag = false;
        }
    #ifdef _TRAILER
        mp_OneWireEnable(gMonetData.OneWireEnable); // MNT-1820  Enable/disable 1-wire Temp sensor based on config. setting
    #endif
#elif defined(_LIONESS) || defined(_PANTHER)
		mnt_debug_printf(gThisModule, "%s:  STM Version: %d.%d.%d_%d\r\n", __func__, gIoVersion[0], gIoVersion[1], gIoVersion[2], gIoVersion[3]);
		gfGotModel = TRUE;
#else
		mnt_debug_printf(gThisModule, "%s:  PIC Version: %d.%d\r\n", __func__, gIoVersion[0], gIoVersion[1]);
#endif
        gCommStatusMask |= COMM_STATUS_GET_VERSION_MASK;
		break;
    case 'x':   // Accelerometer
        {
            uint8_t operation   =  pData[0];
            uint8_t reg         =  pData[1];
            uint8_t value       =  pData[2];
            uint8_t success     =  ((0 == pData[3]) || ('S' == operation) || ('C' == operation)); // MNT-2409 MNT-2341 mark success flag if stream detected

            // mnt_debug_printf(gThisModule, "%s:  Acc(%c) %02X -> %02X (%s)\r\n", __func__, operation, reg, value, success ? "Ok" : "Fail");
            if (success) {    // Successful
                switch (operation) {
                    case 'R':
                    // Read result
                    switch (reg) {
                        case LIS_OUT_X_L:
                        case LIS_OUT_X_H:
                        case LIS_OUT_Y_L:
                        case LIS_OUT_Y_H:
                        case LIS_OUT_Z_L:
                        case LIS_OUT_Z_H:
                            break;
                        default: {
                            mp_printf("\r\n+XTMA%c:%02X,%02X,%s\r\n", operation, reg, value, success ? "OK" : "ERROR");
                            mnt_mfgTestAccelerometerRead(reg, value);
                        }
                    }
                    break;
                    case 'M':
                    {
                        tbool               inMotion    = TBOOL_NOT_ASSIGNED;
                        uint16_t            threshold   = 0;
                        uint16_t            duration    = SECONDS(1);   // Minimum of 1 second duration
                        uint8_t             mode        = MOTION_STATE_NONE;
                        bool                isAccepted  = mnt_powerIsSafeToAbortShutdown();

                        gMotionState    = (ACC_MOTION_STATES_t)reg;     // Current MCU accelerometer state
                        gMotionMode     = (ACC_MOTION_STATES_t)value;   // Next MCU target mode (what MCU is looking for)

                        mnt_debug_printf(gThisModule, "%s:  Motion Mode Detected: %d->%d, %s\r\n", __func__, gMotionState, gMotionMode, isAccepted ? "Accepted" : "Ignored");
                        // MNT-660 Ignore acc. notification if reached non-abortable stage of the shutdown,
                        // but in such case program to wake up immediately to handle accelerometer change
                        if (isAccepted) {
                            // MNT-2261, set threshold, debounce, and mode from drive mode trigger option
                            switch (gMotionState) {
                                case MOTION_STATE_NONE:     // Could receive on application boot-up
                                default:
                                    inMotion = TBOOL_NOT_ASSIGNED;
                                    break;
                                case MOTION_STATE_START:
                                    inMotion = TBOOL_TRUE;
                                    break;
                                case MOTION_STATE_STOP:
                                    inMotion = TBOOL_FALSE;
                                    break;
                            }
                            mnt_getMotionThresholdDebouceMode(inMotion, &threshold, &duration, &mode);
                            // MNT-675 If unit was woke up on timer and MCU is debouncing accelerometer state, we'd like 
                            // to avoid reset of the debounce timer. Reprogram only:
                            // a) if current and next states are the same or
                            // b) the next state is both (remainining from the init) or
                            // c) current state is none, but next state is one of start/stop (happens after TFTP)
                        #ifndef _WILDCAT    // MNT-2330
                            if (gMotionMode == gMotionState || MOTION_STATE_BOTH == gMotionMode || MOTION_STATE_NONE == gMotionState)
                        #endif
                            {
                                int i = 0;

                                duration = MAX(SECONDS(1), duration);      // Minimum of 1 second duration
                        #if defined (_WILDCAT)
                                i = 3;                  // Send 1 time
                        #endif
                                for (; i < 4; i++) {    // MNT-758 Try few times to make sure MCU gets it
                                    mp_setMotionThreshold(threshold, duration, mode);
                                }
                            }
                            mnt_motionAccHandler(inMotion);
                        } else {
                            mnt_powerWakeupImmediately();
                        }
                    }
                    gCommStatusMask |= COMM_STATUS_GET_ACC_STATUS_MASK; // MNT-2290
                    break;
                #if defined(SUPPORT_MCU_ACC_STREAMING) // MNT-2341
                    case 'S': // MNT-2341 Acc stream URC
                        // Parse data stream
                        // mnt_debug_printf(gThisModule, "%s: command: %c, operation: %c, size: %d\r\n", __func__, Command, operation, size);
                        ars_accParseStream(pData, size);
                    break;
                    case 'C': // MNT-2409
                        // Received Acc working mode config response
                        gCommStatusMask |= COMM_STATUS_SET_ACC_MODE_MASK; // MNT-2409
                        // mnt_debug_printf(gThisModule, "%s: command: %c, operation: %c, size: %d\r\n", __func__, Command, operation, size);
                    break;
                #endif
                }
            }
        }
        break;
	case 'y':
		gPowerupCount   = pData[0]*256 + pData[1] + 1;
		gResetCount     = pData[2]*256 + pData[3] + 1;
        gCommStatusMask |= COMM_STATUS_GET_COUNTERS_MASK;
        mnt_debug_printf(gThisModule, "%s:  gPowerupCount:%d gResetCount:%d\r\n", __func__, gPowerupCount, gResetCount);
		break;
	case 'z':
    {
        switch(pData[0]) {
            case 'C':

        #ifdef _TRAILER
                gCommStatusMask |= COMM_STATUS_SET_BATTCHG_MASK;
        #endif
        #ifdef DEBUGF
            {
                uint8 loByte = pData[1];
                uint8 hiByte = pData[2];
                mnt_debug_printf(gThisModule, "%s:  Batt Chg Threshold: %d\r\n", __func__, (hiByte << 8) + loByte);
            }
        #endif
            break;
    #if defined _TRAILER && defined _LIONESS
            case 'B':    // MNT-3060
                if (prod_isNala() && mp_getHwRevision() >= 2) {
                    if ('R' == pData[1]){
                        uint32_t period = pData[2] + (pData[3] << 8) + (pData[4] << 16) + (pData[5] << 24);
                        mnt_debug_printf(gThisModule, "%s:solar checking period: %d seconds\r\n", __func__, period);
                    } else if ('W' == pData[1]){
                        mnt_debug_printf(gThisModule, "%s:solar checking period setting succeed\r\n", __func__);
                    }
                }
            break;
    #endif
    #ifdef _TRAILER
            case 'D': { // MNT-998
                if (pData[1] == 0) { // Get last data frame response
                    // TODO need to add callback about the last CS data frame sent
                }
            }
            break;
            case 'G': {

                uint8 loByte;
                uint8 hiByte;
                if (size < 5) {
                    // MCU does not support the new protocol
                    loByte = pData[1];
                    hiByte = pData[2];
                    gMonetData.oneWireTemperature[0] = (hiByte << 8) + loByte;
                } else {
                    // New MCU protocol uses a minimum of 5 bytes + 2 bytes for each additional sensor
                    uint8 sensor = 0;
                    uint8 numofsensors = pData[1];
                    uint8 sensorType = pData[2];
                    for (sensor = 0; sensor < numofsensors; sensor++) {
                        loByte = pData[(sensor * 2) + 3];
                        hiByte = pData[(sensor * 2) + 4];
                        // Predefine if old sensor format detected
                        gMonetData.oneWireID[sensor + 1] = sensor+1;  // Temperature sensor zero reserved for internal
                        gMonetData.oneWireType[sensor + 1] = sensorType;
                        gMonetData.oneWireTemperature[sensor + 1] = (hiByte << 8) + loByte;
                    #ifdef DEBUGF
                        {
                            if (gMonetData.oneWireTemperature[sensor + 1] > (int16)TEMP_NOT_AVAILABLE) {
                                double fahrenheit = (((double)gMonetData.oneWireTemperature[sensor + 1] / TEMP_SCALE) * 1.8) + 32;
                                mnt_debug_printf(gThisModule, "%s: Sensor:%d  1-Wire Temp: %0.1fC (%0.1fF)\r\n", __func__, gMonetData.oneWireID[sensor + 1], ((double)gMonetData.oneWireTemperature[sensor + 1] / TEMP_SCALE), fahrenheit);
                            }
                            else {
                                mnt_debug_printf(gThisModule, "%s: Sensor:%d  1-Wire Temp: Not Available\r\n", __func__, gMonetData.oneWireID[sensor + 1]);
                            }
                        }
                    #endif
                    }
                }
            }
            break;
        #endif
            case 'S':
        #ifdef _TRAILER
                gCommStatusMask |= COMM_STATUS_SET_TEMPERATURE_MASK;
        #endif
        #ifdef DEBUGF
            {
                uint8 loloByte = 0;
                uint8 lohiByte = 0;
                uint8 hiloByte = 0;
                uint8 hihiByte = 0;
                uint8 sensor   = 0; // Temperature sensor zero reserved for internal
                uint16 lotemp  = 0;
                uint16 hitemp  = 0;
                if (size == 5) {
                    loloByte = pData[1];
                    lohiByte = pData[2];
                    hiloByte = pData[3];
                    hihiByte = pData[4];
                } else { // New multiple sensor protocol
                    sensor   = pData[1] + 1; // Temperature sensor zero reserved for internal
                    loloByte = pData[2];
                    lohiByte = pData[3];
                    hiloByte = pData[4];
                    hihiByte = pData[5];
                }
                lotemp = (lohiByte << 8) + loloByte;
                hitemp = (hihiByte << 8) + hiloByte;
                mnt_debug_printf(gThisModule, "%s:  Set 1-Wire Thresholds:(%d) LO:%d HI:%d\r\n", __func__, sensor, lotemp, hitemp);

            }
        #endif
            break;
            case 'T':
            {
                uint32 tilt = (pData[1] << 0) + (pData[2] << 8) + (pData[3] << 16) + (pData[4] << 24);
                uint8 duration = pData[5];
                mnt_debug_printf(gThisModule, "%s(ZT): Tilt cos set to %d, %d\r\n", __func__, tilt, duration);
            }
            break;
            case 'Z':
        #ifdef DEBUGF
            {
                uint32_t wdTime = (pData[4] << 16) + (pData[3] << 8) + pData[2];
                mnt_debug_printf(gThisModule, "%s(Z): WD time left %d\r\n", __func__, wdTime);
            }
        #endif
            break;
        #ifdef _TRAILER
//            case 0x01: {
//                pData[0] = MONET_V3_ON;
//                mnt_debug_printf(gThisModule, "%s:  Power %d:%d\r\n", __func__, pData[0], pData[1]);
//                // Convert it to the same format as for Power On
//                mnt_csMcuFeedback('p', pData, 2);    // Notify cargo sensor module
//            }
//            break;
            case 0xF3: {
                uint8_t converted = MONET_V3_OFF;

                mnt_debug_printf(gThisModule, "%s:  Power %d\r\n", __func__, MONET_V3_OFF);
                // Convert it to the same format as for Power On
                mnt_csMcuFeedback('p', &converted, sizeof(converted));    // Notify cargo sensor module
            }
            break;
            case 'X':
        #ifdef DEBUGF
            {
                // There should be frequency of one clock reported
                if (sizeof(uint8_t) + sizeof(uint32_t) <= size) {
                    uint32_t * pUint = (uint32_t *)&pData[1];
                    mnt_debug_printf(gThisModule, "%s(%d):  Clock: %ld Hz\r\n", __func__, size, le32toh(*pUint));
                }
            }
        #endif
            break;
        #endif /* _TRAILER */
        }
    }
    break; //Debug information
#ifdef SUPPORT_BLE_DFU // MNT-2624
    case 0xBD: {
        handleBLEResponse(pData, size);
    }
    break;
    case 0xE0: {
        uint16_t ptTime = (pData[1] << 8) + pData[0];
        uint16_t wdTime = (pData[3] << 8) + pData[2];
        mnt_debug_printf(gThisModule, "%s(%02x): PT TIme: %d WD time left %d\r\n", __func__, Command, ptTime, wdTime);        
        mp_setDFUPassThruActive((0 < ptTime) ? true:false); // If ptTime is active set true else false
        if (mp_IsDFUPassThruActive()) { //MNT-2624
            /* enable delay reboot during io upgrade */
            // mnt_debug_printf(gThisModule, "%s: rsp %c, data %s, size %d\r\n", __func__, response, pData, size);
            mp_UpdatePic(NULL, srcUpdateType);    // MNT-2953
        }
    }
    break;
#endif
#if (defined _TRAILER && defined(SUPPORT_BLE_DFU)) || (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)  // MNT-2997
    case 0xAA: {
        mnt_handleComponentInformationResponse(pData, size); // MNT-2812
    }
    break;
#endif

#if defined NORDIC_DFU || defined SUPPORT_BLE_DFU // MNT-3196 For whoever support BLE
    case BLE_RECV_CMD:
    {
        mnt_debug_printf(gThisModule, "%s: cmd:'1', channel:%d\r\n", __func__, pData[0]);
#if (defined SUPPORT_AC61_CAMERA) || (defined SUPPORT_ZAZU_CAMERA)
        mnt_cameraParser(pData, size);	
#endif
    }
    break;
    case BLE_SEND_CMD:
        mnt_debug_printf(gThisModule, "%s: cmd:'2'\r\n", __func__);		
    break;
    case BLE_INFO_CMD:
        mnt_handleBleInfoCommand(pData, size);
    break;
#endif
#if defined BLE_SFILE || defined PRODUCT_AT15
    // simple file transfer entry
    case RCV_TYPE:
        eFile_e ft = // 
        mnt_parsesFile(ft, pData, size);
#endif
    default:
        mnt_debug_printf(gThisModule, "%s: Unimplemented response: <%c>\r\n", __func__, Command);
        break;
    }
}

/*
 *
 * mnt_HandleCommand    
 *
 * Purpose: Provide external function to process MCU messages
 *
 */
void mnt_HandleCommand(uint8 Command, uint8 *pData, uint8 size) { // MNT-2894
    mph_HandleCommand(Command, pData, size);
}

/*
 *
 * mph_HandleResponse
 *
 * Purpose: 
 *
 */
static void mph_HandleResponse(uint8 Command, uint8 *pData, uint8 size)
{
	uint8 Response[64];

	Response[0] = Command;
	mnt_bin2ascii(&Response[1], pData, size);
#if !defined SUPPORT_REMOTE_START && !defined SUPPORT_SERIAL_PASSTHROUGH   // MNT-3042, don't print to peripheral
	mp_printf("PIC:%s\r\n", Response);
#endif
}

/*
 *
 * mph_setGpioDirection
 *
 * Purpose: program a GPIO direction
 *
 */
void mph_setGpioDirection(uint8 gpio, uint8 nDirection, uint8 nValue)
{
	uint8 Parameters[3];

	Parameters[0] = gpio - 1;
	Parameters[1] = nDirection; // 1=input, 0=output
	Parameters[2] = nValue;
    mph_sendPicFrame('G', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s: G %d %d %d\r\n", __func__, gpio, nDirection, nValue);
#ifdef _WIN32
    // Simulate also actual write to be reflected in the GUI
    if (DIRECTION_OUT == nDirection) {
        mp_writeGpio(gpio, nValue);
    }
#endif
}

/*
 *
 * mph_setGpioEvent
 *
 * Purpose: program a GPIO to report on transitions
 *
 */
void mph_setGpioEvent(uint8 gpio, uint8 nEvent, uint8 debounce)
{
#if (IO_MODE != PIC)
	uint8 Parameters[3];

	Parameters[0] = (uint8)gpio - 1;
	Parameters[1] = (uint8)nEvent;
	Parameters[2] = (uint8)(debounce * 10);  // MNT-317
	mph_sendPicFrame('E', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s:  E %d %d %d(%d)\r\n", __func__, Parameters[0], Parameters[1], Parameters[2], debounce);
#else
	uint8 Parameters[4];
    uint16 debounceTimer = 100; // Revert back to 100 mSec
//    uint16 debounceTimer = debounce * 1000; // MNT-1477 MCU now debounces in mSec with a MIN of 50
//    debounceTimer = MAX(50,debounceTimer);  // MNT-1477 MCU now debounces in mSec with a MIN of 50
    
	Parameters[0] = (uint8)gpio - 1;
	Parameters[1] = (uint8)nEvent | GPIO_DIRECTION | GPIO_OUTPUT_HIGH;
	Parameters[2] = (uint8)(debounceTimer & 0xff);
    Parameters[3] = (uint8)((debounceTimer >> 8) & 0xff);
	mph_sendPicFrame('E', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s:  E %d 0x%02x %d(%d)\r\n", __func__, Parameters[0], Parameters[1], Parameters[2], debounce);
#endif
}

#if defined (_TRAILER) || defined (_WILDCAT)
/*
 *
 * mph_HandleGpio
 *
 * Purpose: Analyze a GPIO transition
 *
 */
static void mph_HandleGpio(uint8 *pData, uint8 size)
{
    if (NULL != pData && size >= 2) {
#if defined _TRAILER && defined _LIONESS
        if (csTestMode && prod_isNala()) {
            mnt_handleGpioEventsTestCs(pData[0] + 1, pData[1]); // MNT-3054
        } else 
#endif
        {
            mi_handleGpioEvents(pData[0] + 1, pData[1], ACTION_NORMAL);
        }
    }
}
#endif

#define MCU_ADC_MASK_MAIN           MASK_FOR_BIT(0)
#define MCU_ADC_MASK_BATTERY        MASK_FOR_BIT(1)
#define MCU_ADC_MASK_AUX            MASK_FOR_BIT(2)
#define MCU_ADC_MASK_SOLAR          MASK_FOR_BIT(3)
#define MCU_ADC_MASK_TEMPERATURE    MASK_FOR_BIT(4)

/*
 *
 * mph_readAdc
 *
 * Purpose: get an ADC value
 *
 */
void mph_readAdc(uint8 adc)
{
    uint8 Parameters[1];

	// Initiate the conversion
#ifdef _TRAILER
    Parameters[0] = (MCU_ADC_MASK_MAIN | MCU_ADC_MASK_BATTERY | MCU_ADC_MASK_AUX | MCU_ADC_MASK_SOLAR | MCU_ADC_MASK_TEMPERATURE);
#else
    Parameters[0] = adc;
#endif
    mnt_debug_printf(gThisModule, "%s: %d\r\n", __func__, Parameters[0]);

    mph_sendPicFrame('A', Parameters, sizeof(Parameters));
}

/*
 *
 * mph_HandleAdc
 *
 * Purpose: get an ADC value
 *
 */
static void mph_HandleAdc(uint8 *pData, uint8 size)
{
#ifdef _TRAILER
    // Formula should be: adcMain * 3300 (full scale) * 987 / 100 but this overflows a 32-bit value so the 100 is precomputed
    // Add diode offset at 60 mA (370 mV). When charging, reported V will be too low...
    #define EXT_RAW_ADC_TO_MV(adc)  ((adc) * 33 * 987 / 4096)

    uint8_t     rspMask     = 0;
    uint16_t    searchMask  = MASK_FOR_BIT(0);
    uint8_t *   pD          = pData;
    uint32_t    mvPower     = 0;
    uint16_t    mvPrimary   = (1000 > gMonetData.mvPrimary) ? 0 : gMonetData.mvPrimary; // MNT-1579
    uint16_t    mvSecondary = gMonetData.mvSecondary;   // Will be overwritten if necessary
    uint8_t     readyMask   = 0;

    if (size >= 1) {
        rspMask = pD[0];
        pD++;
        size--;
    }
    while (size >= 2) {
        uint8_t matchMask = 0;

        while (searchMask <= MASK_FOR_BIT(7) && ! matchMask) {
            uint32_t        adcValue    = pD[0] + pD[1] * 256;
            const char *    strSource   = "";
            // To avoid (when DEBUGF is undefined) Error: #150-D: variable "strSource" was set but never used
            UNUSED_BUT_SET_VARIABLE(strSource);

            matchMask = (searchMask & rspMask);

            if (matchMask & (MCU_ADC_MASK_MAIN | MCU_ADC_MASK_AUX | MCU_ADC_MASK_SOLAR)) {
                // Formula should be: adcMain * 3300 (full scale) * 987 / 100 but this overflows a 32-bit value so the 100 is precomputed
                // After SIMBAMCU-36 voltage readings are already in millivolts and no longer needs conversion in the App
                if (prod_isSimba() || prod_isNala()) { // MNT-2728, MNT-2294
                    mvPower = adcValue;
                } else {
                    mvPower = EXT_RAW_ADC_TO_MV(adcValue);
                }
                if (1000 > mvPower) { // MNT-1579
                    mvPower = 0;
                }
                if (matchMask & (MCU_ADC_MASK_MAIN | MCU_ADC_MASK_AUX)) {
                    if ((1 == gMonetData.PowerSourceSwitch && MCU_ADC_MASK_MAIN == matchMask) ||
                        (0 == gMonetData.PowerSourceSwitch && MCU_ADC_MASK_AUX  == matchMask))
                    {
                        gMonetData.mvMain   = (uint16_t)mvPower;
                        readyMask           |= VOLTAGE_READY_MAIN;
                        strSource           = "MAIN ";
                    } else {
                        gMonetData.mvAux   = (uint16_t)mvPower;
                        readyMask           |= VOLTAGE_READY_AUX;
                        strSource           = "AUX  ";
                    }
                } else {    // Solar
                    gMonetData.mvSolar  = (uint16_t)mvPower;
                    readyMask           |= VOLTAGE_READY_SOLAR;
                    strSource           = "SOLAR";
                }
            } else if (matchMask & MCU_ADC_MASK_BATTERY) {
                if (prod_isSimba() || prod_isNala()) {  // MNT-2728, MNT-1898
                    mvPower     = adcValue; // MNT-2294
                } else {
                    mvPower     = (adcValue * 3300 * 74) / (4096 * 27);
                }
                mvSecondary = (uint16_t)mvPower;
                readyMask   |= VOLTAGE_READY_BATTERY;
                strSource   = "BKUP ";
            } else if (matchMask & MCU_ADC_MASK_TEMPERATURE) {
                gMonetData.Temperature = (int16_t)adcValue;        // Convert into signed value;
		        mnt_debug_printf(MODULE_IFADC, "%s: Temperature: %4d %6.2f\r\n", __func__, adcValue, (double)gMonetData.Temperature / 100.);
            }
            // Check if it was voltage related reading
            if (matchMask & (MCU_ADC_MASK_MAIN | MCU_ADC_MASK_AUX | MCU_ADC_MASK_SOLAR | MCU_ADC_MASK_BATTERY)) {
                gMonetData.VoltageReadyMask |= readyMask;
                mnt_debug_printf(MODULE_IFADC, "%s(%02X): %s: %4d  %6.3f\r\n", __func__, gMonetData.VoltageReadyMask, strSource, adcValue, (double)mvPower / 1000);
            }
            searchMask <<= 1;
        }

        pD      +=  2;
        size    -=  2;
    }

    if (readyMask & (VOLTAGE_READY_MAIN | VOLTAGE_READY_AUX | VOLTAGE_READY_BATTERY)) {
        // Update if any of primary/secondary voltage was read
        mvPrimary = MAX(gMonetData.mvMain, gMonetData.mvAux);
        mi_handleAdcEvents(mvPrimary, mvSecondary);
    }
#else
	if (size == 4 || (size == 5 && pData[4] == 0)) {
	    uint32_t        adcPrimary  = pData[0] + pData[1] * 256;
	    uint32_t        adcBackup   = pData[2] + pData[3] * 256;
        uint32_t        mvPrimary   = 0;
        uint32_t        mvBackup    = 0;

    #if (IO_MODE == EFM)
	    mvPrimary   = adcPrimary * 1223 / 1000;
	    mvBackup    = adcBackup  * 1223 / 1000;
    #else // PIC
      #ifdef _PANTHER
        /* MNT-1056     Use calculations for Rev 2 HW
        VCAR is multiplied by 91 / (91+1000)
        Vbat is multiplied by 1000 / (1000+560)

        Therefore main voltage reading is    (Reading/1024) * 35.97+0.7
        Battery Voltage reading is          (Reading/1024) * 4.68

        Since we keep in mV,need to multiply by 1000
        */
	    mvPrimary   = ((adcPrimary * 35970) / 1024) + 700;
	    mvBackup    = ((adcBackup * 4680) / 1024);
      #elif defined _WILDCAT
        // MNT-2165 MCU sends values already converted into mV, use as is
        mvPrimary   = adcPrimary;
        mvBackup    = adcBackup;
      #else
	    mvPrimary   = adcPrimary * 51;
	    mvBackup    = (adcBackup * 638) / 100;
      #endif
    #endif
        gMonetData.VoltageReadyMask |= (VOLTAGE_READY_MAIN | VOLTAGE_READY_BATTERY);
	    mnt_debug_printf(MODULE_IFADC, "%s(%02X): MAIN: %d %6.3f\r\n", __func__, gMonetData.VoltageReadyMask, adcPrimary, (double)mvPrimary / 1000);
	    mnt_debug_printf(MODULE_IFADC, "%s(%02X): BKUP: %d %6.3f\r\n", __func__, gMonetData.VoltageReadyMask, adcBackup, (double)mvBackup / 1000);
        mi_handleAdcEvents(mvPrimary, mvBackup);
	}
	else if (size == 2 || (size == 5 && pData[4] == 1)) { // Temperature
		mnt_debug_printf(MODULE_IFADC, "%s: Temperature: %d\r\n", __func__, pData[0]);
		gMonetData.Temperature = pData[0] * 100;
	}
#endif
}

/*
 *
 * mp_setGPIO
 *
 * Purpose: Set the GPIO processor based on the model #
 *
 */
void mp_setGPIO(void)
{
	uint8 gpio  = 1;
	uint8 mask  = 1;

	// Set the direction and value of each GPIO
	for (gpio = 1; gpio <= MAX_GPIO; gpio++) {
        mask = MASK_FOR_BIT(gpio-1);
		mp_SetGpioDirection(gpio, gMonetData.IoDirection & mask);
	}
    if (gfGotModel) {
#if !(defined _TRAILER) // MNT-688
        // Repeat multiple times to ensure command is acted upon
        mp_BatteryEnable(gMonetData.fBatteryEnable);
#endif
    }
#if defined(_MINITRACK)
    {
        if (gMonetData.PanicButtonTrigger != PANIC_BUTTON_DISABLED) { // MNT-1230
            mnt_debug_printf(gThisModule, "%s: Set up panic button\r\n", __func__);
            gMonetData.IoDirection |= MASK_FOR_BIT(GPIO_FOR_BUTTON-1);  // Make sure to set GPIO B as input
            mp_SetGpioDirection(GPIO_FOR_BUTTON, DIRECTION_IN);      // and event is enabled
        }
    }
#endif
}

/*
 *
 * mp_SetGpioDirection
 *
 * Purpose: Set the GPIO direction
 *
 */
void mp_SetGpioDirection(uint8 gpio, uint8 direction)
{
	if (!gfGotModel || 0 == gpio || gpio > MAX_GPIO) {
        mnt_debug_printf(gThisModule, "%s: G %d %d %d Invalid\r\n", __func__, gpio, direction, gfGotModel);
        return;
	}

#if defined GPIO_FOR_RELAY
	gMonetData.IoDirection &= ~(MASK_FOR_BIT(GPIO_FOR_RELAY - 1));	// Relay is output only
#endif

    if (direction) {
        // Input
        mph_setGpioDirection(gpio, DIRECTION_IN, 0);
        // Register changes
        mnt_gpioConfigureInputForEvents(gpio - 1);
	} else {
		// Output
        uint8 mask      = MASK_FOR_BIT(gpio - 1);
		uint8 nValue    = (gMonetData.IoWrite & mask) != 0;
		mph_setGpioDirection(gpio, DIRECTION_OUT, nValue);
        mnt_writeIo(gpio,nValue); // MNT-1286
        mnt_writeIo(gpio,nValue); // MNT-1286
#if (IO_MODE != PIC)
		mph_setGpioEvent(gpio, 0, 0);
#endif
	}
}

/*
 *
 * mp_SetGpioOutput
 *
 * Purpose: Set the GPIO value
 *
 */
void mp_SetGpioOutput(uint8 gpio, uint8 value)
{
	uint8 mask = 0;

    if (!gfGotModel || 0 == gpio || ((gpio > MAX_GPIO) && (gpio != GPIO_BATTERY + 1))) { // MNT-664 Also allow battery gpio
		return;
	}

    mask = 1 << (gpio - 1);
	if ((gMonetData.IoDirection & mask) == 0) {
		// Output
    #if defined SUPPORT_OUTPUT_INVERSION
        // MNT-1835, MNT-2930 Invert the actual value written only after gMonetData.ioWrite was updated with the desired value
        if (mnt_gpioIsOutputInverted(gpio - 1)) {
            value = value ? 0 : 1;
        }
    #endif
		mph_setGpioDirection(gpio, DIRECTION_OUT, value);
        mnt_debug_printf(gThisModule, "%s: io(%d) V(%d)\r\n", __func__, gpio, value);
	}
}

/*
 *
 * mp_ClearResetCount
 *
 * Purpose: Clear the counters in the I/O processor
 *
 */
void mp_ClearResetCount(void)
{
	mph_sendPicFrame('0', NULL , 0);
	gPowerupCount   = 0;
	gResetCount     = 0;
}

/*
 *
 * mp_GetResetCounters
 *
 * Purpose: Get counters
 *
 */
void mp_GetResetCounters(uint16 *PowerUp, uint16 *Reset)
{
	*PowerUp = gPowerupCount;
	*Reset = gResetCount;
}

/*
 *
 * mp_SerialConfig
 *
 * Purpose: Serial Config
 *
 */
//
void mp_SerialConfig(uint8 Speed, uint8 Mode)
{
	uint8 Parameters[2];

	Parameters[0] = Speed;
	Parameters[1] = Mode;
	mph_sendPicFrame('C', Parameters, sizeof(Parameters));
	mnt_debug_printf(gThisModule, "%s: Peri Uart Config: %d 0x%02X\r\n", __func__, Speed, Mode);
#if (CHIPSET_VENDOR	== MSFT)
	mp_setUartSpeedWinPeripheral(baudRatesValues[Speed]);
#endif
}

/*
 *
 * mp_AppSerialConfig
 *
 * Purpose: Serial Config of a App
 *
 */
//
void mp_AppSerialConfig(uint8 Speed, uint8 Mode)
{
#ifdef _TRAILER
	uint8 Parameters[3];

	Parameters[0]   = Speed;
	Parameters[1]   = Mode;
    Parameters[2]   = 1;        // Application UART
	mph_sendPicFrame('C', Parameters, sizeof(Parameters));
	mnt_debug_printf(gThisModule, "%s: App Uart Config: %d 0x%02X\r\n", __func__, Speed, Mode);
#endif
}

/*
 *
 * mp_sendPic
 *
 * Purpose: 
 *
 */
void mp_sendPic(uint8 *pBuffer, uint8 nLength)
{
	mph_sendPicFrame(pBuffer[0], &pBuffer[1], nLength - 1);
}

#if defined _TRAILER || defined _LIONESS || defined _PANTHER || defined _WILDCAT

void mp_QueryAllGpios(void)
{
    uint8 Parameters[2];

    Parameters[0] = 1;      // Read all GPIOs
    Parameters[1] = 0;      // Unused

    mph_sendPicFrame('Q', Parameters, sizeof(Parameters));
//    mnt_debug_printf(gThisModule, "%s\r\n", __func__);
}

void mp_QueryGpio(uint8 gpioIndex)
{
    uint8 Parameters[2];

    Parameters[0] = 2;      // Read one GPIO
    Parameters[1] = gpioIndex < MAX_GPIO ? gpioIndex : 0;

	mph_sendPicFrame('Q', Parameters, sizeof(Parameters));
//    mnt_debug_printf(gThisModule, "%s: %d\r\n", __func__, Parameters[1]);
}

void mp_pollAllGpio(void) {
    int i;
    for (i = 0; i < MAX_GPIO; i++) {
         mp_QueryGpio(i);
    }
}

#ifdef _TRAILER // MNT-1882
static void mp_OneWireHoldHandler(mnt_Timer_t * pTimer) {
    UNUSED_PARAMETER(pTimer);
}

void mp_OneWireEnable(FLAG fEnable)
{
    uint8 Parameters[2];

    Parameters[0] = 3;      // 1-wire
    Parameters[1] = fEnable ? 0 : 1;    // The setting in MCU is for disable, so use opposite logic

    if (fEnable) {  // MNT-1878
        oneWireHoldTimer.mode       = MNT_TIMER_INTERVAL_MODE;
        oneWireHoldTimer.interval   = SECONDS(15);
        oneWireHoldTimer.handler    = mp_OneWireHoldHandler;
        mnt_TimerRestart(&oneWireHoldTimer);
    } else {
        mnt_TimerStop(&oneWireHoldTimer);
    }

	mph_sendPicFrame('Q', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s: %d\r\n", __func__, Parameters[1]);
}

// MNT-841
void mp_OneWireDiscoverNow(void)
{
    uint8 Parameters[2];

    Parameters[0] = 3;      // 1-wire
    Parameters[1] = 2;      // Discover now

	mph_sendPicFrame('Q', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s: %d\r\n", __func__, Parameters[1]);
}
#endif

void mp_GSensorLed(FLAG fTurnOn)
{
    uint8 Parameters[2];

    Parameters[0] = 1;      // G-sensor LED case
    Parameters[1] = fTurnOn ? 1 : 0;

	mph_sendPicFrame('O', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], fTurnOn ?  strOn : strOff);
}

#ifdef _TRAILER
void mp_PeripheralPower(FLAG fTurnOn)
{
    uint8 Parameters[] = { MONET_V3_OFF };
    if (fTurnOn) {
        Parameters[0] = MONET_V3_ON;
    }
    peripheralPowerOn = fTurnOn;    // MNT-1349
    
	mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], fTurnOn ?  strOn : strOff);
}

// MNT-3051 Add external power enable
void mp_ExternalCsPower(bool fTurnOn)
{
    uint8 Parameters[] = { MONET_EXT_CS_OFF };
    if (fTurnOn) {
        Parameters[0] = MONET_EXT_CS_ON;
    }

    mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], fTurnOn ?  strOn : strOff);
}
#endif

#if defined _TRAILER && defined _LIONESS
// MNT-3054 Internal cargo sensor test start, tell BLE MCU to switch MUX mode and enable CS power
void mp_CsTestStart(void)
{
    uint8_t parameters[2] = {0};

    parameters[0] = 'C';
    parameters[1] = 'P';
    mph_sendPicFrame('J', parameters, 2);
    mnt_debug_printf(gThisModule, "%s J %c %c\r\n", __func__, parameters[0], parameters[1]);

    csTestMode = true;
    mph_setGpioEvent(GPIO_1, GPIO_TRIGGER_ANY_TRANSITION, SECONDS(2)); // Enable GPIOA for any change
}

// MNT-3054 Internal cargo sensor test end, tell BLE MCU to switch MUX to default mode and disable CS power
void mp_CsTestEnd(void)
{
    uint8_t parameters[2] = {0};

    parameters[0] = 'C';
    parameters[1] = 'R';
    mph_sendPicFrame('J', parameters, 2);
    mnt_debug_printf(gThisModule, "%s J %c %c\r\n", __func__, parameters[0], parameters[1]);

    csTestMode = false;
}

// MNT-3060 Get solar checking period
void mp_GetSolarCheckPeriod(void)
{
    if (prod_isNala() && mp_getHwRevision() >= 2) {
        uint8_t Parameters[2] = {'B', 'R'};

        mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
        mnt_debug_printf(gThisModule, "%s Z %c %c\r\n", __func__, Parameters[0], Parameters[1]);
    }
}

// MNT-3060 Set solar checking period
void mp_SetSolarCheckPeriod(uint32_t period)
{
    if (prod_isNala() && mp_getHwRevision() >= 2 && period > 0) {
        core_bufferAccumulative_ts  abuf;
        uint8_t Parameters[6] = {0};

        core_abufferInit(&abuf, Parameters, sizeof(Parameters));
        core_abufferAddChar(&abuf, 'B');
        core_abufferAddChar(&abuf, 'W');
        core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, period, 4);

        mph_sendPicFrame('Z', Parameters, core_abufferGetSize(&abuf));
        mnt_debug_printf(gThisModule, "%s Z %c %c %x %x %x %x\r\n", __func__, Parameters[0], Parameters[1], Parameters[2], Parameters[3], Parameters[4], Parameters[5]);
    }
}
#endif  

void mp_McuClocks(uint8_t clockId)
{
    uint8 Parameters[2];
    Parameters[0] = 'X';
    Parameters[1] = clockId <= 3 ? clockId : 0;
	mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s: (%c%d)\r\n", __func__, Parameters[0], Parameters[1]);
}

/*
 *
 * mp_setBatteryCriticalMode
 *
 * Purpose: set MCU into EM3 mode
 *
 */
void mp_setBatteryCriticalMode(void) { // MNT-2240
    uint8_t tries = 0;
    uint16_t battCritical = mnt_powerVoltageListenerGetVoltage(VL_INT_CRITICAL_MODE);
    
    for (tries = 0; tries < 3; tries++) {	// MNT-1953
        uint8 Parameters[5] = { 0 };
        Parameters[0] = MONET_BUB_CRITICAL;
        Parameters[1] = (uint8_t)((battCritical & 0x00ff) >> 0);
        Parameters[2] = (uint8_t)((battCritical & 0xff00) >> 8);
        Parameters[3] = (uint8_t)((gMonetData.batteryCriticalSleepTime & 0x00ff) >> 0);
        Parameters[4] = (uint8_t)((gMonetData.batteryCriticalSleepTime & 0xff00) >> 8);
        mph_sendPicFrame('P', Parameters, sizeof(Parameters));
        mnt_debug_printf(gThisModule, "%s: Setting MCU Battery Critical to %d mV and %d secs\r\n", __func__, battCritical, gMonetData.batteryCriticalSleepTime);
    }
}

#if (!defined(_LIONESS) && !defined _WILDCAT) || defined _WIN32
void mp_WakeUpPin(FLAG fTurnOn)
{
    uint8 Parameters[] = { MONET_WPIN_OFF };
    if (fTurnOn) {
        Parameters[0] = MONET_WPIN_ON;
    }
	mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], fTurnOn ?  strOn : strOff);
}

#endif  // !defined _LIONESS
#endif  // defined _TRAILER || defined _LIONESS

/*
 *
 * mp_BatteryEnable
 *
 * Purpose: Enable or Disable the battery
 *
 */
void mp_BatteryEnable(uint8 OnOff)
{
    uint8_t tries = 0;
    for (tries = 0; tries < 3; tries++) {	// MNT-1953
#ifdef _TRAILER
        uint8 Parameters[] = { MONET_BLATCH_OFF };
        if (OnOff) {
            Parameters[0] = MONET_BLATCH_ON;
        } else {
            mnt_debug_printf(MODULE_POWER,"%s: Disconnect battery\r\n", __func__); // MNT-2792
        }
        mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], OnOff ?  strOn : strOff);
        mph_sendPicFrame('P', Parameters, sizeof(Parameters));
#else
        uint16_t mask = 0;

        mask = ~(1 << (GPIO_BATTERY)); // MNT-664 Make sure Batt GPIO is set for output
        gMonetData.IoDirection &= mask;

        mnt_writeIo(GPIO_BATTERY + 1, OnOff);
        mnt_debug_printf(gThisModule, "%s: (%d): %s 0x%02x\r\n", __func__, GPIO_BATTERY, OnOff ?  strOn : strOff, gMonetData.IoDirection);
#endif
    }
}

/*
 *
 * mp_ShippingMode
 *
 * Purpose: Enable or Disable shipping mode
 *
 */
void mp_ShippingMode(uint8 OnOff)
{
#if defined(_TRAILER) || defined(_TM90M)  // MNT-2677
    uint8 Parameters[] = { MONET_BUBX_OFF };
    if (OnOff) {
        Parameters[0] = MONET_BUBX_ON;
    }
	mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s(%d): %s\r\n", __func__, Parameters[0], OnOff ?  strOn : strOff);
#endif
}

/*
 *
 * mp_SerialToAccessoryDirect
 *
 * Purpose: Send data to 8 pin connector
 *
 */
void mp_SerialToAccessoryDirect(uint8 *pFrame, uint16 nLength)
{
#if (CHIPSET_VENDOR	== MSFT)
	mp_sendUartWinPeripheral(pFrame, nLength);
#endif
    mph_sendPicFrame(PIC_DATA_TO_UART, pFrame, nLength);
}

void mp_SerialIn(uint8 byte)
{
	gSerialBuffer[gSerialTail] = byte;
	gSerialTail = (gSerialTail+1) % sizeof(gSerialBuffer);
	gSerialSize++;
}

uint8 mp_SerialOut(void)
{
	uint8 byte;
	
	byte = gSerialBuffer[gSerialHead];
	gSerialHead = (gSerialHead+1) % sizeof(gSerialBuffer);
	gSerialSize--;
	return byte;
}

void mp_serialHandler(uint16 timer)
{
	uint8	serialData[32];
	uint16	thisSize;
	int32	i;

	mp_stopTimer(TIMER_SERIAL);

	if (gSerialSize == 0) {
		// Done
		return;
	}
	thisSize = gSerialSize;
	if (thisSize > 24) {
		thisSize = 24;
	}

	for (i=0; i<thisSize; i++) {
		serialData[i] = mp_SerialOut();
	}
	mph_sendPicFrame(PIC_DATA_TO_UART, serialData, thisSize);
	mp_startTimer(TIMER_SERIAL, 100, mp_serialHandler);
#if (CHIPSET_VENDOR	== MSFT)
	mp_sendUartWinPeripheral(serialData, thisSize);
#endif
}

/*
 *
 * mp_SerialToAccessory
 *
 * Purpose: Send data to 8 pin connector
 *
 */
void mp_SerialToAccessory(uint8 *pFrame, uint16 nLength)
{
	int32 i;

	mp_stopTimer(TIMER_SERIAL);
	if (nLength > (sizeof(gSerialBuffer) - gSerialSize)) {
		// no room
		return;
	}

	for (i=0; i<nLength; i++ ) {
		mp_SerialIn(pFrame[i]);
	}

	mp_startTimer(TIMER_SERIAL, 30, mp_serialHandler);
}

#if (CHIPSET_VENDOR	!= MSFT)
/*
 *
 * mp_hardReset
 *
 * Purpose: Perform a power cycle
 *
 */
void mp_hardReset(void)
{
    mnt_debug_printf(gThisModule, "%s: requested\r\n", __func__);
#if defined _TRAILER
	mph_resetSystem(4);
#elif defined _LIONESS
    // Force watchdog to trigger reset
    mph_resetSystem(1);
#elif defined _PANTHER
    // MNT-947
	mph_resetSystem(4);
    if (NULL != oem_shutdown) { // MNT-1639
        mnt_debug_printf(gThisModule, "%s: calling oem_shutdown\r\n", __func__);
        oem_shutdown();
    }
#elif defined _WILDCAT
	mph_resetSystem(4);
#else
	mph_powerBasebandPower(0x02);
#endif
    mnt_enableAppWD(false);     // MNT-810, MNT-2184
    gMonetData.HardResetRequested = 1;
}

uint16 mp_i2c_read(uint8 nAddress, uint8 nRegister, uint8 *pData, uint8 nLength)
{
	return  0;
}

uint16 mp_i2c_write(uint8 nAddress, uint8 nRegister, uint8 *pData, uint8 nLength)
{	
	return 0;
}
#endif // #if (CHIPSET_VENDOR != MSFT)
#if (IO_MODE==EFM) || (IO_MODE==PIC)
/*
 *
 * mp_setWakeUpTimer
 *
 * Purpose: Remove the power to the baseband
 *
 */
void mp_setWakeUpTimer(uint32 timer)
{
#ifdef _TRAILER
    uint8 Parameters[6];
    mnt_debug_printf(gThisModule, "%s (%d)\r\n", __func__, timer);

    Parameters[0] = 'Z';
    Parameters[1] = 2;
    Parameters[2] = (uint8)((timer & 0x000000ff) >> 0);
    Parameters[3] = (uint8)((timer & 0x0000ff00) >> 8);
    Parameters[4] = (uint8)((timer & 0x00ff0000) >> 16);
    Parameters[5] = 0;

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
#else
    uint8 Parameters[3];

    mnt_debug_printf(gThisModule, "%s (%d)\r\n", __func__, timer);
    Parameters[2] = (timer >> 16) & 0xff;
    Parameters[1] = (timer >> 8) & 0xff;
    Parameters[0] = (timer >> 0) & 0xff;
    mph_sendPicFrame('N', Parameters, sizeof(Parameters));
#endif
}

#endif


#if defined(_TRAILER) || defined(_WILDCAT)
/*
 *
 * mp_setMotionThreshold
 *
 * Purpose: Set the motion thresholds needed to wake-up the MCU
 *
 */ 
void mp_setMotionThreshold(uint16 threshold, uint16 duration, uint8 algorithm)
{
	uint8 Parameters[7];
	Parameters[0] = 'T';
    if (IS_COLLISION_ENABLED()) {   // MNT-2501
	    Parameters[1] = (uint8_t)LIS_8GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold);  // MNT-2374
    } else {
	    Parameters[1] = (uint8_t)LIS_2GTHRESHOLD_MM_BY_SEC2_TO_REG(threshold);  // MNT-2374
    }
    Parameters[1] = MAX(1,   Parameters[1]);  // Make sure it is not 0
    Parameters[1] = MIN(127, Parameters[1]);  // Make sure it fits the register
	Parameters[2] = 'D';
    Parameters[3] = duration & 0xFF;
    Parameters[4] = (duration >> 8) & 0xFF;
    Parameters[5] = 'M';
    Parameters[6] = algorithm;
    mph_sendPicFrame('m', Parameters, sizeof(Parameters));
	mnt_debug_printf(gThisModule, "%s: Motion Threshold: %d, Duration: %d, Algorithm: %d\r\n", __func__, Parameters[1], duration, Parameters[6]);
}

/*
 *
 * mp_gotoStealth
 *
 * Purpose: Remove the power to the baseband
 *
 */ 
void mp_gotoStealth(uint32 rtc)
{
	uint8 Parameters[4];

	mnt_debug_printf(gThisModule, "%s: Go to stealth (%d)\r\n", __func__, rtc);
    mnt_debug_printf(gThisModule, "\r\n\r\n\r\n%s\r\n\r\n", ART_SLEEPING_CAT);

    mnt_clearRTCandINTStates();
    
	Parameters[3] = (rtc >> 24) & 0xff;
	Parameters[2] = (rtc >> 16) & 0xff;
	Parameters[1] = (rtc >>  8) & 0xff;
	Parameters[0] = (rtc >>  0) & 0xff;
	mph_sendPicFrame('S', Parameters, sizeof(Parameters));
}

/*
 *
 * mp_accReadRegister
 *
 * Purpose: Read accelerometer register
 *
 */ 
void mp_accReadRegister(uint8 reg)
{
	uint8 Parameters[3];

	mnt_debug_printf(gThisModule, "%s (%02X)\r\n", __func__, reg);
	Parameters[0] = 'R';    // For read
	Parameters[1] = reg;    // Register
	Parameters[2] = 0;      // Dummy for read
	mph_sendPicFrame('X', Parameters, sizeof(Parameters));
}

/*
*
* mp_accWriteRegister
*
* Purpose: Write accelerometer register
*
*/
void mp_accWriteRegister(uint8 reg, uint8 value)
{
	uint8 Parameters[3];

	mnt_debug_printf(gThisModule, "%s (%02X) <- %02X\r\n", __func__, reg, value);
	Parameters[0] = 'W';    // For write
	Parameters[1] = reg;    // Register
	Parameters[2] = value;  // Value
	mph_sendPicFrame('X', Parameters, sizeof(Parameters));
}

/*
*
* mp_setTempThreshold
*
* Purpose: Set the 1-wire Temperature Thresholds
*
*/
void mp_setTempThresholds(uint8 sensor, uint16 loTemp, uint16 hiTemp)
{
	uint8 Parameters[6];

    Parameters[0] = 'S';               // Register
    Parameters[1] = (uint8)(sensor-1); // MCU temp sensors are zero-based
    Parameters[2] = (uint8)(loTemp & 0xFF);
    Parameters[3] = (uint8)(loTemp >> 8);
    Parameters[4] = (uint8)(hiTemp & 0xFF);
    Parameters[5] = (uint8)(hiTemp >> 8);
    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
	mnt_debug_printf(gThisModule, "%s:(%d) loTemp=%d hiTemp=%d\r\n", __func__, sensor, loTemp, hiTemp);
}

/*
*
* mp_askTemperature
*
* Purpose: Ask for the 1-wire Temperature
*
*/
void mp_askTemperature() {
    uint8 Parameters[1];
 
    Parameters[0] = 'G';       // Register
    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

/*
*
* mp_setChargeThreshold
*
* Purpose: Set the Backup Battery Charging Threshold
*
*/
void mp_setChargeThreshold(uint16 mvChgThreshold)
{
	uint8 Parameters[3];

	Parameters[0] = 'C';    // Register
	Parameters[1] = mvChgThreshold & 0xFF;
	Parameters[2] = mvChgThreshold >> 8;
	mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
	mnt_debug_printf(gThisModule, "%s Charging Threshold=%d\r\n", __func__, mvChgThreshold);
}

void md_QueryMotionMode(void) {
    uint8 Parameters[1];

    Parameters[0] = 'M';        // Ask MCU about current motion mode
    mph_sendPicFrame('X', Parameters, sizeof(Parameters));

}

#elif defined(_LIONESS)
void mp_accReadRegister(uint8 reg)
{
	uint8 Parameters[3];

	mnt_debug_printf(gThisModule, "%s (%02X)\r\n", __func__, reg);
	Parameters[0] = 'R';    // For read
	Parameters[1] = reg;    // Register
	Parameters[2] = 0;      // Dummy for read
	mph_sendPicFrame('X', Parameters, sizeof(Parameters));
}

void mp_accWriteRegister(uint8 reg, uint8 value) {
	uint8 Parameters[3];

	mnt_debug_printf(gThisModule, "%s (%02X)\r\n", __func__, reg);
	Parameters[0] = 'W';    // For Write
	Parameters[1] = reg;    // Register
	Parameters[2] = value;  // value to write
	mph_sendPicFrame('X', Parameters, sizeof(Parameters));
}

void mp_QueryAccId(void)
{
    mp_accReadRegister(LIS_WHO_AM_I);
}

void mp_QueryAccXL(void)
{
    mp_accReadRegister(LIS_OUT_X_L);
}

void mp_QueryAccXH(void)
{
    mp_accReadRegister(LIS_OUT_X_H);
}

void md_McuSetAccWorkmode(uint8 workmode, uint8 threshold, uint8 duration) {
    uint8 Parameters[4];

    mnt_debug_printf(gThisModule, "%s (%02X)\r\n", __func__, workmode);
    Parameters[0] = 'M';        // For Write
    Parameters[1] = workmode;   // workmode
    Parameters[2] = threshold;  // threshold
    Parameters[3] = duration;  // duration
    mph_sendPicFrame('X', Parameters, sizeof(Parameters));

}

uint8_t mcu_LisWriteReg (uint16_t regAddr, uint8_t value) {
    mp_accWriteRegister(regAddr, value);
    return 1;
}

/*
 *
 * mp_setAlarm
 *
 * Purpose: Ask MCU to wake us up after x seconds
 *
 */ 
void mp_setAlarm(uint32 alarm)
{
	uint8 Parameters[4];

	mnt_debug_printf(gThisModule, "%s: Set wakeup alert (%d secs)\r\n", __func__, alarm);

	Parameters[2] = ((alarm & 0x00ff0000) >> 16) & 0xff;
	Parameters[1] = ((alarm & 0x0000ff00) >>  8) & 0xff;
	Parameters[0] = ((alarm & 0x000000ff) >>  0) & 0xff;
	mph_sendPicFrame('N', Parameters, sizeof(Parameters));
}
#endif  // defined _TRAILER

/*
 * MNT-2623
 * Purpose: update RTC time via RTC interrupt.
 * Update only when RTC time was not updated via cellular nor GPS
 */
static void mnt_TimerUpdateViaRtc(void)
{
    if (!gMonetData.gpsRtcWakeupIsUpdated) { // MNT-2125
        mnt_debug_printf(gThisModule, "%s:  Adjust RTC by %d Secs\r\n", __func__, gCommRtcTime);
        // MNT-1717 Make sure that if wake from sleep is immediate, at least one second is reported to trigger the time
        // adjustment mechanism that releases reports that are waiting for time adjustment
        gCommRtcTime = MAX(SECONDS(1), gCommRtcTime);
        mnt_TimerSetTimersAdjustmentTime(gCommRtcTime, gIsWarmWakeup);
    }
}

/*
 * MNT-3005
 * Purpose: set buzzer frequency.
 */
void mnt_setBuzzerFrequency(uint16_t frequency)
{
    core_bufferAccumulative_ts  abuf;
    uint8_t                     Parameters[5]   = { 0 };
    size_t                      len             = 0;

    core_abufferInit(&abuf, Parameters, sizeof(Parameters));
    core_abufferAddChar(&abuf, 'F');
    core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, frequency, 2);
    len = core_abufferGetSize(&abuf);
    
    mph_sendPicFrame('b', Parameters, (uint16_t)len);
}

#else
void mp_setMotionThreshold(uint16 threshold, uint16 duration, uint8 algorithm)
{
    UNUSED_PARAMETER(threshold);
    UNUSED_PARAMETER(duration);
    UNUSED_PARAMETER(algorithm);
}

#endif	// #if (IO_MODE == PIC) || (IO_MODE == EFM)

/*
*
* mph_cmdINWU
*
* Purpose: display wake-up information
*
*/
void mph_cmdINWU(void)
{
#if (IO_MODE == EFM) || (IO_MODE == PIC)
    core_updownBootupReason_te bootupReason = core_updownGetBootupReason();

    mp_sprintf(gResponse + strlen(gResponse), "\r\nBOOT:  %08X,%d", gBootFlags, bootupReason);
    mp_sprintf(gResponse + strlen(gResponse), "\r\nWU:    %08X",    gLastInterruptStatus);
    mp_sprintf(gResponse + strlen(gResponse), "\r\nRTC:   %d",      gCommRtcTime);
    mp_sprintf(gResponse + strlen(gResponse), "\r\nACC:   %d,%d",   gMotionState, gMotionMode);
#endif
    mp_sprintf(gResponse + strlen(gResponse), "\r\nOK");
}

/*
*
* mp_clearInterruptStatus
*
* Purpose: clear a wakeup flag
*
*/
void mp_clearInterruptStatus(uint32 flag)
{
    gInterruptStatus ^= flag;
}

void mnt_clearRTCandINTStates(void) {
    // MNT-741 Make sure we ask for RTC and Interrupt info again after wake-up by clearing the bits
#if (IO_MODE == PIC) || (IO_MODE == EFM)
    gCommStatusMask &= ~(COMM_STATUS_GET_INT_STATUS_MASK | COMM_STATUS_GET_RTC_MASK | COMM_STATUS_SET_RTC_UPDATED_MASK);
#endif
}

// MNT-749
void mnt_enableWarmWakeup(bool enable)
{
    UNUSED_BUT_SET_VARIABLE(gIsWarmWakeup); // To avoid warning for Bobcat
    gIsWarmWakeup = enable;
}

/*
 *
 * mp_turnBasebandOff
 *
 * Purpose: Disable the BB (Suicide mode)  MNT-791
 *
 */
void mp_turnBasebandOff(void) {
#if defined (_TRAILER) || defined (_WILDCAT)
    uint8 Parameters[] = { MONET_BB_OFF };
	mph_sendPicFrame('P', Parameters, sizeof(Parameters));
    mnt_debug_printf(gThisModule, "%s:(%d)\r\n", __func__, Parameters[0]);
#endif
}

#if defined _TRAILER || defined (_WILDCAT)
// MNT-786
static void mnt_reconfigureMcuMotionState(void) {
    uint16_t            threshold   = 0;
    uint16_t            duration    = SECONDS(1);   // Minimum of 1 second duration
    uint8_t             mode        = MOTION_STATE_NONE;
    tbool               accMotionSt = ars_accGetMotionStatus();

    // MNT-2261, set threshold, debounce, and mode from drive mode trigger option
    mnt_getMotionThresholdDebouceMode(accMotionSt, &threshold, &duration, &mode);
    // MNT-675 If unit was woke up on timer and MCU is debouncing accelerometer state, we'd like 
    // to avoid reset of the debounce timer. Reprogram only:
    // a) if current and next states are the same or
    // b) the next state is both (remainining from the init) or
    // c) current state is none, but next state is one of start/stop (happens after TFTP)
    if (gMotionMode == gMotionState || MOTION_STATE_BOTH == gMotionMode || MOTION_STATE_NONE == gMotionState) {
        int i = 0;

        duration = MAX(SECONDS(1), duration);      // Minimum of 1 second duration
#if defined (_WILDCAT)
        i = 3;                  // Send 1 time
#endif
        for (; i < 4; i++) {    // MNT-758 Try few times to make sure MCU gets it
            mp_setMotionThreshold(threshold, duration, mode);
        }
    }
    mnt_debug_printf(gThisModule, "%s: threshold:%d, duration:%d, mode:%d\r\n", __func__, threshold, duration, mode);

#ifdef _TRAILER
    ars_ConfigTiltTamper(); // MNT-2317
#endif
}
#endif

#ifdef _MINITRACK
void mp_askUID(void) {
    uint8 Parameters[2] = {0};
    
    mnt_debug_printf(gThisModule, "%s: Addr:0x%x\r\n", __func__, UID_ADDRESS);
    
    Parameters[0] = 'R';    // Write EEProm
    Parameters[1] = (UID_ADDRESS & 0x00FF) >> 0;
    mph_sendPicFrame('F', Parameters, sizeof(Parameters));
}

void mp_setUID(uint32_t uid) {
    uint8 Parameters[6] = {0};
    uint8 index;
    
    Parameters[0] = 'W';    // Write EEProm
    Parameters[1] = (UID_ADDRESS & 0xFF) >> 0;
    // Little Endian UnitID
    Parameters[2] = (uid & 0x000000FF) >> 0;
    Parameters[3] = (uid & 0x0000FF00) >> 8;
    Parameters[4] = (uid & 0x00FF0000) >> 16;
    Parameters[5] = (uid & 0xFF000000) >> 24;
    
    for (index = 0; index < sizeof(Parameters); index++) {
        mnt_debug_printf(gThisModule, "%s: Parameters[%d]=0x%x\r\n", __func__, index, Parameters[index]);
    }
    mph_sendPicFrame('F', Parameters, sizeof(Parameters));
}
#endif

#ifdef _TRAILER
// MNT-998
void mp_getCSLastDataFrame(void) {
    uint8 Parameters[3] = {0}; // All zeros is query
    
    Parameters[0] = 'D';    // send query to get last data frame counter
    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

void mp_setCSLastDataFrame(uint8 framenumber) {
    uint8 Parameters[3] = {0};
    
    Parameters[0] = 'D';
    Parameters[0] = 1;    // send set to configure last data frame counter
    Parameters[0] = framenumber;
    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

// MNT-1319
void mnt_configMCUTiltTamper(uint8_t tiltThreshold, uint8_t tiltEventTimer) {
    uint8 Parameters[6];

    int32_t value = 0;
    double tiltCosine = cos(DEGREE_2_RADIAN(tiltThreshold));
    value = (int32_t)ceil(tiltCosine * 1000000);
    
    mnt_debug_printf(gThisModule, "%s (%d, %d, %d)\r\n", __func__, value, tiltThreshold, tiltEventTimer);
    Parameters[0] = 'T';
    Parameters[1] = (value >> 0) & 0xff;
    Parameters[2] = (value >> 8) & 0xff;
    Parameters[3] = (value >> 16) & 0xff;
    Parameters[4] = (value >> 24) & 0xff;
    Parameters[5] = tiltEventTimer;

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}


// MNT-1349
void mnt_setMCUPeripheralTXLine(FLAG bOn) {
    uint8 Parameters[2];

    mnt_debug_printf(gThisModule, "%s %d\r\n", __func__, bOn);
    Parameters[0] = 'P';
    Parameters[1] = bOn;

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

#endif

/*
 *
 * mp_setInterruptStatus
 *
 * Purpose: set the reason for wakeup
 *
 */ 
void mp_setInterruptStatus(uint32 flag)
{
    gInterruptStatus |= flag;
}

#if (IO_MODE == PIC)
static void mnt_ServiceAccelerometerEvent(uint32_t intMask) {
    if (intMask & WAKE_UP_MOTION) {
        md_LisIrq2Handler(); // Clear interrupt pins MNT-1226
        // Send a report if needed
        if (mnt_getGSensorWakeupEventPending()) {        // MNT-718
            // MNT-781 Getting G-sensor wake-up while in motion means that unit has stopped
            // MNT-859 Rely on the accelerometer motion status only
            tbool accMotion = ars_accGetMotionStatus();
            if (TBOOL_TRUE == accMotion) {
                if (gMonetData.GsensorMotionEventTrigger & MOTION_START_MASK) {
                    // SEND REPORT
                    mnt_sendToServer(EVT_MOTION_DETECTED);
                }
            } else if (TBOOL_FALSE == accMotion) {
                if (gMonetData.GsensorMotionEventTrigger & MOTION_STOP_MASK) {
                    // SEND REPORT
                    mnt_sendToServer(EVT_MOTION_STOP);
                }
            }
            mnt_setGSensorWakeupEventPending(false); // MNT-718
        }
    }
}

/*
 *
 * mp_sendDeepSleepMode
 *
 * Purpose: Send deep sleep
 *
 */
void mp_sendDeepSleepMode(mntPowerMode_t mode, uint8 offdelay) {
    uint8 Parameters[6];
    uint32 sleepTime = mp_getDeepSleepTime();
    uint8 sleepMode = (mode < POWER_MODE_SLEEP) ? 0 : (mode == POWER_MODE_SLEEP) ? 1 : 2;
    
    if (mnt_TimerStarted(&ignitionTimer)) { // MNT-1674
        sleepTime = offdelay + 1;
    }

    mnt_debug_printf(gThisModule, "%s: Request %s (%d)\r\n", __func__, mnt_stringPowerMode(mode), sleepTime);
    Parameters[0] = 'Z';
    Parameters[1] = sleepMode;
    Parameters[2] = (uint8)((sleepTime & 0x000000ff) >> 0);
    Parameters[3] = (uint8)((sleepTime & 0x0000ff00) >> 8);
    Parameters[4] = (uint8)((sleepTime & 0x00ff0000) >> 16);
    Parameters[5] = offdelay;

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

/*
 *
 * mp_mcuWakeSystem
 *
 * Purpose: Send wakeup command to MCU
 *
 */
void mp_mcuWakeSystem(void) {
    uint8 Parameters[6] = {0}; // Need all 6 fields

    mnt_debug_printf(gThisModule, "%s: Request Wakeup\r\n", __func__);
    Parameters[0] = 'Z';
    Parameters[1] = 0; // Send wakeup command

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}

#endif

#if defined _TRAILER || defined (_WILDCAT)
void mnt_updateMcuMotionState(void) { // MNT-1694
    mnt_debug_printf(gThisModule, "%s: called\r\n", __func__);
    // MNT-1530, force to reconfigure
    gMotionState = MOTION_STATE_NONE;
    // MNT-786 Reconfigure accelerometer to last known state
    mnt_reconfigureMcuMotionState();
}

/*
 *
 * MNT-2587
 * mnt_recovery_mcuApp_check
 *
 * Purpose: send command to check if mcu is in boot loader mode
 * If it is in boot loader mode, a mcu update will follow
 *
 */
static void mnt_recovery_mcuApp_check(void)
{
    if ( !mp_IsPicLoaderActive() && mp_verify(ioBinary)) {
        uint8 Parameters[1];
        Parameters[0] = 0xff;
        mph_sendPicFrame('@', Parameters, sizeof(Parameters));
        // notes: if mcu partition is active, it will respond with the same content.
    }
}
#endif

#if (IO_MODE == PIC)
void mnt_mcuResetRtc(void) {    // MNT-1725
    uint8 Parameters[2] = { 'Z', 3 };

    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));

    mnt_debug_printf(gThisModule, "%s: called\r\n", __func__);

}
#endif

#if defined (_WILDCAT)
// MNT-2027
void mnt_resetCommStatusCounter (void)
{
    gCommStatusCounter = 0;
}
#endif
/*
 * MNT-2196
 * Purpose: Initialize the global variables.
 * For example after wake up from sleep mode
 */
void mph_initIOGlobalVars (void)
{
#if (IO_MODE == PIC) || (IO_MODE == EFM)
    gSerialHead           = 0;
    gSerialTail           = 0;
    gSerialSize           = 0;
    gInterruptStatus      = 0;
    gLastInterruptStatus  = 0;

    gRxState              = IO_WAIT_FOR_DOLLAR;
    gRxCount              = 0;
    gRxLength             = 0;
    gRxCommand            = 0x00;
#endif
}

#if defined(_TRAILER) && defined(SUPPORT_BLE_DFU)
void mnt_requestMuxUpdate(update_peri_dev_type_e updatePeriDevType, update_type_e updateTypeSrc) {    // MNT-2953
    // Save update peripheral device type for later use
    update_peri_dev_type = updatePeriDevType;
    srcUpdateType        = updateTypeSrc;
    
    mnt_StartMuxUpdatePortRedirection();
    mp_startMuxUpdateProcess(updateTypeSrc);    // MNT-2953
}

void mnt_getMuxVersion(uint8_t * pVersion) { // MNT-2703
    if (NULL != pVersion) {
        mnt_memcpy(pVersion, gMuxVersion, sizeof(gMuxVersion));
    }
}

void mnt_getMacAddr(uint8_t * pMacAddr) { // MNT-2804
    if (NULL != pMacAddr) {
        mnt_memcpy(pMacAddr, gMacId, sizeof(gMacId));
    }
}

bool mnt_hasBleVersion(void) { // MNT-2969
#if !defined _WIN32
    return IS_BIT_CLEARED_BY_MASK(gPeriComponentMask, COMPONENT_INFO_VERSION_MASK);
#else
    return true;
#endif
}
#endif

// MNT-2757 
#ifdef _MA60
void mp_PowerOff(void)
{	
    uint8 Parameters[6] = {0}; 
	
    mnt_debug_printf(gThisModule, "%s: Request power off\r\n", __func__);
    Parameters[0] = 'Z';
    Parameters[1] = 4; // Send power off command
    mph_sendPicFrame('Z', Parameters, sizeof(Parameters));
}
#endif

#if defined _LIONESS || defined _WILDCAT || defined _PANTHER || defined _LEOPARD
// MNT-1464
static bool shouldPerformWdHeartbeat = true;

void mp_StopWatchdogHeartbeat (void) {
    shouldPerformWdHeartbeat = false; 
}

void mp_EnableWatchdogHeartbeat (void) {        // MNT-1240
    shouldPerformWdHeartbeat = true; 
}

bool mp_shouldKickWd(void) {
    return shouldPerformWdHeartbeat;
}
#else
void mp_StopWatchdogHeartbeat (void)    { }
void mp_EnableWatchdogHeartbeat (void)  { }      // MNT-1240
bool mp_shouldKickWd(void)              { return true; }
#endif

// MNT-2954
void mnt_enableAppWD(bool enable) {
    mnt_debug_printf(MODULE_IF, "%s: App WD: %d\r\n", __func__, enable);
    if (enable) {
        // gMonetData.overrideMCUWD = 0;
        mp_EnableWatchdogHeartbeat();
        if (0 != watchdogPeriodicTimer.interval && !mnt_TimerStarted(&watchdogPeriodicTimer)) {
            mnt_TimerStart(&watchdogPeriodicTimer);
        }
    } else {
        // gMonetData.overrideMCUWD = 1;
        mnt_killWatchdog();
        mp_StopWatchdogHeartbeat();
    }
}

#ifdef _TRAILER

static void mnt_HardResetRetrySeqHandler(uint32_t param) { // MNT-3024
    uint8_t time = (uint8_t)param;
    mph_sendPicFrame('H', &time, sizeof(time));
    mnt_debug_printf(MODULE_MCU, "%s: delay %d\r\n", __func__, time);
}

static void mnt_stopHardResetRetrySeq(void) { // MNT-3024
    if (CORE_SEQ_IS_VALID_HANDLE(hardResetHandle)) {
        core_seqSequencedActionStop(hardResetHandle);
        hardResetHandle = CORE_SEQ_INVALID_HANDLE;
    }
    mnt_debug_printf(MODULE_MCU, "%s: called\r\n", __func__);
}

static void mnt_hardResetTimeout(int handle) {
    hardResetHandle = CORE_SEQ_INVALID_HANDLE;
}

static void mnt_startHardResetRetrySeq(uint32_t delay) { // MNT-3024
    mnt_debug_printf(MODULE_MCU, "%s: delay %d\r\n", __func__, delay);

    mnt_stopHardResetRetrySeq();
    hardResetSeq[0].param = delay;
    hardResetHandle = core_seqSequencedActionStart(
        CORE_SEQ_FLAG_CYCLIC,
        hardResetSeq, 
        ARRAY_SIZE(hardResetSeq), 
        SECONDS(30), mnt_hardResetTimeout,
        "HRD-RST"); // MNT-3036 Limit the number of reset requests to 3
}
#endif

#if defined NORDIC_DFU || defined SUPPORT_BLE_DFU
void mp_BlePairing(uint32_t mask) // BLE pairing start
{
    core_bufferAccumulative_ts  abuf;
    uint8_t                     parameters[5]   = { 0 };

    core_abufferInit(&abuf, parameters, sizeof(parameters));
    core_abufferAddChar(&abuf, 'R');
    core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, mask, 4);

    mph_sendPicFrame(BLE_INFO_CMD, parameters, sizeof(parameters));
    mnt_debug_printf(MODULE_IO, "%s: mask 0x%x\r\n", __func__, mask);
}

void mp_BleUnpairing(uint32_t mask, uint8_t* macId) // BLE unpairing sensors
{
    core_bufferAccumulative_ts  abuf;
    size_t                      len             = 0;
    uint8_t                     parameters[11]  = { 0 };
    uint8_t                     success         = (macId != NULL);
    
    if (success) {    // Successful
        core_abufferInit(&abuf, parameters, sizeof(parameters));
        core_abufferAddChar(&abuf, 'U');
        core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, mask, 4);
        core_abufferAddBuffer(&abuf, macId, BLE_MAC_HEX_LEN);
        len = core_abufferGetSize(&abuf);

        mph_sendPicFrame(BLE_INFO_CMD, parameters, (uint16_t)len);
        mnt_debug_printf(MODULE_IO, "%s: mask 0x%x, mac %02x%02x%02x%02x%02x%02x\r\n", __func__, mask, macId[0], macId[1], macId[2], macId[3], macId[4], macId[5]);
    }
}

void mp_GetPairedBleInfo(void)           // Query information of paired BLE sensor
{
    uint8_t                     parameters[1]   = { 'I' };

    mph_sendPicFrame(BLE_INFO_CMD, parameters, 1);
    mnt_debug_printf(MODULE_IO, "%s called\r\n", __func__);
}

void mp_BleScanNow(void)                // Scan now and list all compatible sensor
{
    uint8_t                     parameters[1]   = { 'N' };

    mph_sendPicFrame(BLE_INFO_CMD, parameters, 1);
    mnt_debug_printf(MODULE_IO, "%s called\r\n", __func__);
}

void mp_BleScanConfig(uint16_t window, uint16_t interval, uint32_t duration) // configure BLE scanning
{
    core_bufferAccumulative_ts  abuf;
    uint8_t                     parameters[9]   = { 0 };
    size_t                      len             = 0;

    core_abufferInit(&abuf, parameters, sizeof(parameters));
    core_abufferAddChar(&abuf, 'O');
    core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, window, 2);
    core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, interval, 2);
    core_abufferAddUint(&abuf, CORE_PACK_FORMAT_LITTLE_ENDIAN, duration, 4);
    len = core_abufferGetSize(&abuf);

    mph_sendPicFrame(BLE_INFO_CMD, parameters, (uint16_t)len);
    mnt_debug_printf(MODULE_IO, "%s: window %d, interval %d, duration %d\r\n", __func__, window, interval, duration);
}
#endif // #if defined NORDIC_DFU || defined SUPPORT_BLE_DFU

