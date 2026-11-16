/**
 * @file    leak_app_main.c
 * @brief   MAX32655FTHR Water Pipe Leakage Detection BLE Application
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "wsf_types.h"
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_nvm.h"
#include "wsf_timer.h"
#include "hci_api.h"
#include "sec_api.h"
#include "dm_api.h"
#include "smp_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_wp.h"
#include "util/calc128.h"
#include "gatt/gatt_api.h"
#include "wut.h"
#include "trimsir_regs.h"
#include "pal_bb.h"
#include "pal_btn.h"
#include "pal_uart.h"
#include "tmr.h"
#include "svc_sds.h"
#include "util/terminal.h"

#include "board.h"
#include "led.h"
#include "pb.h"

#include "leak_app_api.h"
#include "audio_mic.h"
#include "leak_detector.h"

/**************************************************************************************************
  Macros & Constants
**************************************************************************************************/
#define LEAK_TIMER_EVT          0x9A
#define LEAK_TIMER_PERIOD_MS    1000    /* Acoustic check & telemetry notification every 1 second */

#define BTN_SHORT_MS            200
#define BTN_MED_MS              500
#define BTN_LONG_MS             1000

#define BTN_1_TMR               MXC_TMR2
#define BTN_2_TMR               MXC_TMR3

/*! Client characteristic configuration indices */
enum {
    LEAK_GATT_SC_CCC_IDX,
    LEAK_WP_DAT_CCC_IDX,
    LEAK_SEC_DAT_CCC_IDX,
    LEAK_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable BLE Parameters
**************************************************************************************************/

static const appAdvCfg_t leakAdvCfg = {
    { 0, 0, 0 },
    { 160, 320, 0 }
};

static const appSlaveCfg_t leakSlaveCfg = {
    1,
};

static const appSecCfg_t leakSecCfg = {
    DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG | DM_AUTH_MITM_FLAG,
    DM_KEY_DIST_IRK,
    DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,
    FALSE,
    INIT_SECURITY
};

static const smpCfg_t leakSmpCfg = {
    500,
    SMP_IO_KEY_ONLY,
    7,
    16,
    1,
    0,
    64000,
    64000,
    2
};

static const appUpdateCfg_t leakUpdateCfg = {
    0,
    (15 * 8 / 1.25),
    (15 * 12 / 1.25),
    0,
    600,
    5
};

static const attCfg_t leakAttCfg = {
    15,
    241,
    ATT_MAX_TRANS_TIMEOUT,
    4
};

static uint8_t localIrk[] = { 0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93,
                              0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85 };

/**************************************************************************************************
  Advertising & Scan Response Data
**************************************************************************************************/

static const uint8_t leakAdvDataDisc[] = {
    /*! flags */
    2,
    DM_ADV_TYPE_FLAGS,
    DM_FLAG_LE_GENERAL_DISC | DM_FLAG_LE_BREDR_NOT_SUP,

    /*! tx power */
    2,
    DM_ADV_TYPE_TX_POWER,
    0,

    /*! Complete local name (13 chars + 1 byte type = 14) */
    14,
    DM_ADV_TYPE_LOCAL_NAME,
    'P', 'I', 'P', 'E', '_', 'L', 'E', 'A', 'K', '_', 'D', 'E', 'T'
};

static const uint8_t leakScanDataDisc[] = {
    /*! 128-bit Service UUID */
    17,
    DM_ADV_TYPE_128_UUID,
    ATT_UUID_P1_SERVICE
};

static const attsCccSet_t leakCccSet[LEAK_NUM_CCC_IDX] = {
    { GATT_SC_CH_CCC_HDL, ATT_CLIENT_CFG_INDICATE, DM_SEC_LEVEL_NONE },
    { WP_DAT_CH_CCC_HDL, ATT_CLIENT_CFG_NOTIFY, DM_SEC_LEVEL_NONE },
    { SEC_DAT_CH_CCC_HDL, ATT_CLIENT_CFG_NOTIFY, DM_SEC_LEVEL_NONE }
};

/**************************************************************************************************
  Local Variables & Timers
**************************************************************************************************/

static struct {
    wsfHandlerId_t handlerId;
} leakCb;

static wsfTimer_t s_leakTimer;

extern void setAdvTxPower(void);

/*************************************************************************************************/
void LeakAppSendTelemetry(const char *msg)
{
    if (msg == NULL) {
        return;
    }

    dmConnId_t connId = AppConnIsOpen();
    if (connId != DM_CONN_ID_NONE) {
        if (AttsCccEnabled(connId, LEAK_WP_DAT_CCC_IDX)) {
            uint16_t len = (uint16_t)strlen(msg);
            AttsHandleValueNtf(connId, WP_DAT_HDL, len, (uint8_t *)msg);
        }
    }
}

/*************************************************************************************************/
static uint8_t leakWpWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                                uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    char cmd[64];
    uint16_t copy_len = (len < 63) ? len : 63;
    memcpy(cmd, pValue, copy_len);
    cmd[copy_len] = '\0';

    /* Strip newline / carriage returns */
    for (int i = copy_len - 1; i >= 0; i--) {
        if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == ' ') {
            cmd[i] = '\0';
        } else {
            break;
        }
    }

    TerminalTxPrint("[BLE_RX] Command received: '%s'\r\n", cmd);

    if (strncmp(cmd, "MIC_INT", 7) == 0 || strncmp(cmd, "TEST_INT", 8) == 0 ||
        strncmp(cmd, "INT", 3) == 0 || strncmp(cmd, "MIC=INT", 7) == 0) {
        /* Test and select INTERNAL DIGITAL MEMS MICROPHONE */
        int16_t mn = 0, mx = 0;
        uint32_t rms = 0;
        bool hw_act = false;
        Audio_Mic_Test(AUDIO_MIC_SOURCE_INTERNAL, &mn, &mx, &rms, &hw_act);
        LeakDetector_StartCalibration(10);
        char resp[128];
        snprintf(resp, sizeof(resp),
                 "[CMD] MIC=INTERNAL MEMS | Audio: RMS=%u [%d..%d] HW:%s | Calibrating...\r\n",
                 (unsigned int)rms, mn, mx, hw_act ? "OK" : "FALLBACK");
        TerminalTxStr(resp);
        LeakAppSendTelemetry(resp);
    } else if (strncmp(cmd, "MIC_EXT", 7) == 0 || strncmp(cmd, "TEST_EXT", 8) == 0 ||
               strncmp(cmd, "EXT", 3) == 0 || strncmp(cmd, "MIC=EXT", 7) == 0) {
        /* Test and select EXTERNAL 3.5mm LINE-IN MICROPHONE */
        int16_t mn = 0, mx = 0;
        uint32_t rms = 0;
        bool hw_act = false;
        Audio_Mic_Test(AUDIO_MIC_SOURCE_EXTERNAL, &mn, &mx, &rms, &hw_act);
        LeakDetector_StartCalibration(10);
        char resp[128];
        snprintf(resp, sizeof(resp),
                 "[CMD] MIC=EXTERNAL LINE-IN (+24dB) | Audio: RMS=%u [%d..%d] HW:%s | Calibrating...\r\n",
                 (unsigned int)rms, mn, mx, hw_act ? "OK" : "FALLBACK");
        TerminalTxStr(resp);
        LeakAppSendTelemetry(resp);
    } else if (strncmp(cmd, "CAL", 3) == 0) {
        LeakDetector_StartCalibration(20);
        LeakAppSendTelemetry("[CMD] Baseline calibration started (20s)\r\n");
    } else if (strncmp(cmd, "LEAK", 4) == 0 || strncmp(cmd, "TEST", 4) == 0) {
        Audio_Mic_InjectLeakSignal(true);
        LeakAppSendTelemetry("[CMD] Simulating high-pressure pipe leak vibration...\r\n");
    } else if (strncmp(cmd, "STOP", 4) == 0 || strncmp(cmd, "NORM", 4) == 0) {
        Audio_Mic_InjectLeakSignal(false);
        LeakDetector_ResetAlarm();
        LeakAppSendTelemetry("[CMD] Simulation stopped. Returned to normal baseline\r\n");
    } else if (strncmp(cmd, "RST", 3) == 0) {
        Audio_Mic_InjectLeakSignal(false);
        LeakDetector_ResetAlarm();
        LeakAppSendTelemetry("[CMD] Alarm latch cleared. State: NORMAL\r\n");
    } else if (strncmp(cmd, "THR=", 4) == 0) {
        uint32_t val = (uint32_t)atoi(&cmd[4]);
        if (val > 0) {
            LeakDetector_SetThreshold(val);
            char resp[64];
            snprintf(resp, sizeof(resp), "[CMD] Leak Threshold set to %u RMS\r\n", (unsigned int)val);
            LeakAppSendTelemetry(resp);
        }
    } else if (strncmp(cmd, "STAT", 4) == 0) {
        leak_metrics_t m;
        LeakDetector_GetMetrics(&m);
        char resp[128];
        snprintf(resp, sizeof(resp), "[STAT][%s] Leak:%s RMS:%u Base:%u Thr:%u Sev:%u%% Conf:%u%%\r\n",
                 Audio_Mic_GetSourceName(),
                 m.leak_detected ? "YES" : "NO", (unsigned int)m.rms,
                 (unsigned int)m.baseline_rms, (unsigned int)m.threshold_rms,
                 (unsigned int)m.severity_pct, (unsigned int)m.confidence_pct);
        LeakAppSendTelemetry(resp);
    } else {
        LeakAppSendTelemetry("[CMD] Commands: MIC_INT, MIC_EXT, LEAK, STOP, CAL, RST, THR=xxx, STAT\r\n");
    }

    return ATT_SUCCESS;
}

static terminalCommand_t s_termCmdMicInt;
static terminalCommand_t s_termCmdMicExt;
static terminalCommand_t s_termCmdLeak;
static terminalCommand_t s_termCmdStop;

static uint8_t termCmdMicInt(uint32_t argc, char **argv)
{
    int16_t mn = 0, mx = 0;
    uint32_t rms = 0;
    bool hw_act = false;
    Audio_Mic_Test(AUDIO_MIC_SOURCE_INTERNAL, &mn, &mx, &rms, &hw_act);
    LeakDetector_StartCalibration(10);
    TerminalTxPrint("[CMD] MIC=INTERNAL MEMS | Audio: RMS=%u [%d..%d] HW:%s | Calibrating...\r\n",
                    (unsigned int)rms, mn, mx, hw_act ? "OK" : "FALLBACK");
    return TERMINAL_ERROR_OK;
}

static uint8_t termCmdMicExt(uint32_t argc, char **argv)
{
    int16_t mn = 0, mx = 0;
    uint32_t rms = 0;
    bool hw_act = false;
    Audio_Mic_Test(AUDIO_MIC_SOURCE_EXTERNAL, &mn, &mx, &rms, &hw_act);
    LeakDetector_StartCalibration(10);
    TerminalTxPrint("[CMD] MIC=EXTERNAL LINE-IN (+24dB) | Audio: RMS=%u [%d..%d] HW:%s | Calibrating...\r\n",
                    (unsigned int)rms, mn, mx, hw_act ? "OK" : "FALLBACK");
    return TERMINAL_ERROR_OK;
}

static uint8_t termCmdLeak(uint32_t argc, char **argv)
{
    Audio_Mic_InjectLeakSignal(true);
    TerminalTxStr("[CMD] Simulating high-pressure pipe leak vibration...\r\n");
    return TERMINAL_ERROR_OK;
}

static uint8_t termCmdStop(uint32_t argc, char **argv)
{
    Audio_Mic_InjectLeakSignal(false);
    LeakDetector_ResetAlarm();
    TerminalTxStr("[CMD] Simulation stopped. Returned to normal baseline\r\n");
    return TERMINAL_ERROR_OK;
}

/*************************************************************************************************/
static uint8_t leakSecDatWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                                    uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    return ATT_SUCCESS;
}

/*************************************************************************************************/
static void leakCccCback(attsCccEvt_t *pEvt)
{
    appDbHdl_t dbHdl;

    if (pEvt->idx == LEAK_WP_DAT_CCC_IDX) {
        if (pEvt->value & ATT_CLIENT_CFG_NOTIFY) {
            TerminalTxStr("\r\n[BLE] LightBlue subscribed to Leak Notifications!\r\n");
            LeakAppSendTelemetry("=== MAX32655FTHR Pipe Leak Detector Ready ===\r\n");
        } else {
            TerminalTxStr("\r\n[BLE] Notifications disabled by client.\r\n");
        }
    }

    if ((pEvt->handle != ATT_HANDLE_NONE) &&
        ((dbHdl = AppDbGetHdl((dmConnId_t)pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
        AppCheckBonded((dmConnId_t)pEvt->hdr.param)) {
        AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
        AppDbNvmStoreCccTbl(dbHdl);
    }
}

/*************************************************************************************************/
static void leakDmCback(dmEvt_t *pDmEvt)
{
    dmEvt_t *pMsg;
    uint16_t len;

    if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND) {
        DmSecSetEccKey(&pDmEvt->eccMsg.data.key);
    } else {
        len = DmSizeOfEvt(pDmEvt);
        if ((pMsg = WsfMsgAlloc(len)) != NULL) {
            memcpy(pMsg, pDmEvt, len);
            WsfMsgSend(leakCb.handlerId, pMsg);
        }
    }
}

/*************************************************************************************************/
static void leakAttCback(attEvt_t *pEvt)
{
    attEvt_t *pMsg;
    if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL) {
        memcpy(pMsg, pEvt, sizeof(attEvt_t));
        pMsg->pValue = (uint8_t *)(pMsg + 1);
        memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
        WsfMsgSend(leakCb.handlerId, pMsg);
    }
}

/*************************************************************************************************/
static void leakSetup(dmEvt_t *pMsg)
{
    /* Set advertising and scan response data for discoverable mode */
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(leakAdvDataDisc), (uint8_t *)leakAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(leakScanDataDisc), (uint8_t *)leakScanDataDisc);

    /* Set advertising and scan response data for connectable mode */
    AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(leakAdvDataDisc), (uint8_t *)leakAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(leakScanDataDisc), (uint8_t *)leakScanDataDisc);

    /* Start advertising; automatically set connectable/discoverable mode and bondable mode */
    AppAdvStart(APP_MODE_AUTO_INIT);
    TerminalTxStr("[BLE] AppAdvStart called -> Advertising as 'PIPE_LEAK_DET'\r\n");
}

/*************************************************************************************************/
static void leakProcMsg(dmEvt_t *pMsg)
{
    switch (pMsg->hdr.event) {
    case DM_RESET_CMPL_IND:
        TerminalTxStr("[BLE] Stack reset complete. Initializing advertising...\r\n");
        AttsCalculateDbHash();
        DmSecGenerateEccKeyReq();
        AppSlaveClearAllBondingInfo();
        leakSetup(pMsg);
        setAdvTxPower();
        break;

    case DM_ADV_START_IND:
        TerminalTxStr("[BLE] >>> ADV BROADCAST ACTIVE: Search 'PIPE_LEAK_DET' in LightBlue! <<<\r\n");
        LED_Off(LED_BLUE);
        break;

    case DM_ADV_STOP_IND:
        TerminalTxStr("[BLE] Advertising stopped. Restarting...\r\n");
        AppAdvStart(APP_MODE_AUTO_INIT);
        break;

    case DM_CONN_OPEN_IND:
        TerminalTxStr("\r\n[BLE] *** Phone Connected via LightBlue! ***\r\n");
        LED_On(LED_BLUE);
        if (leakSecCfg.initiateSec) {
            AppSlaveSecurityReq((dmConnId_t)pMsg->hdr.param);
        }
        break;

    case DM_CONN_CLOSE_IND:
        TerminalTxStr("\r\n[BLE] Connection closed. Restarting advertising...\r\n");
        LED_Off(LED_BLUE);
        AppAdvStart(APP_MODE_AUTO_INIT);
        break;

    case DM_SEC_PAIR_CMPL_IND:
        DmSecGenerateEccKeyReq();
        break;

    case DM_SEC_PAIR_FAIL_IND:
        DmSecGenerateEccKeyReq();
        break;

    default:
        break;
    }
}

/*************************************************************************************************/
static bool s_sim_leak_active = false;

static void leakBtnCback(uint8_t btn)
{
    /* Button 1 (SW3) short press toggles simulated pipe leak for easy verification */
    if (btn == APP_UI_BTN_1_SHORT) {
        s_sim_leak_active = !s_sim_leak_active;
        Audio_Mic_InjectLeakSignal(s_sim_leak_active);
        if (s_sim_leak_active) {
            TerminalTxStr("\r\n>>> [BUTTON SW3] TEST LEAK SIMULATION TRIGGERED! <<<\r\n");
            LeakAppSendTelemetry("[BTN] SW3: TEST LEAK TRIGGERED\r\n");
        } else {
            TerminalTxStr("\r\n>>> [BUTTON SW3] TEST LEAK STOPPED (NORMAL PIPE) <<<\r\n");
            LeakDetector_ResetAlarm();
            LeakAppSendTelemetry("[BTN] SW3: Test Leak Stopped\r\n");
        }
    } else if (btn == APP_UI_BTN_2_SHORT) {
        TerminalTxStr("\r\n>>> [BUTTON SW4] Recalibrating outdoor baseline... <<<\r\n");
        LeakDetector_StartCalibration(20);
        LeakAppSendTelemetry("[BTN] Calibrating outdoor baseline (20s)...\r\n");
    }
}

/*************************************************************************************************/
static void btnPressHandler(uint8_t btnId, PalBtnPos_t state)
{
    if (btnId == 1) {
        if (state == PAL_BTN_POS_UP) {
            unsigned btnUs = MXC_TMR_SW_Stop(BTN_1_TMR);
            if ((btnUs > 0) && (btnUs < BTN_SHORT_MS * 1000)) {
                AppUiBtnTest(APP_UI_BTN_1_SHORT);
            }
        } else {
            MXC_TMR_SW_Start(BTN_1_TMR);
        }
    } else if (btnId == 2) {
        if (state == PAL_BTN_POS_UP) {
            unsigned btnUs = MXC_TMR_SW_Stop(BTN_2_TMR);
            if ((btnUs > 0) && (btnUs < BTN_SHORT_MS * 1000)) {
                AppUiBtnTest(APP_UI_BTN_2_SHORT);
            }
        } else {
            MXC_TMR_SW_Start(BTN_2_TMR);
        }
    }
}

/*************************************************************************************************/
void LeakAppHandlerInit(wsfHandlerId_t handlerId)
{
    leakCb.handlerId = handlerId;

    /* Set configuration pointers required by Cordio framework */
    pAppSlaveCfg = (appSlaveCfg_t *)&leakSlaveCfg;
    pAppAdvCfg = (appAdvCfg_t *)&leakAdvCfg;
    pAppSecCfg = (appSecCfg_t *)&leakSecCfg;
    pAppUpdateCfg = (appUpdateCfg_t *)&leakUpdateCfg;
    pSmpCfg = (smpCfg_t *)&leakSmpCfg;
    pAttCfg = (attCfg_t *)&leakAttCfg;

    AppSlaveInit();
    AppServerInit();

    DmSecSetLocalIrk(localIrk);

    s_leakTimer.handlerId = handlerId;
    s_leakTimer.msg.event = LEAK_TIMER_EVT;

    /* Start periodic leak detection timer */
    WsfTimerStartMs(&s_leakTimer, LEAK_TIMER_PERIOD_MS);
}

/*************************************************************************************************/
void LeakAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (pMsg != NULL) {
        if (pMsg->event == LEAK_TIMER_EVT) {
            /* 1. Capture acoustic frame from onboard microphone */
            int16_t audio_buf[AUDIO_FRAME_SIZE];
            int err = Audio_Mic_CaptureFrame(audio_buf, AUDIO_FRAME_SIZE);

            leak_metrics_t metrics;
            if (err == E_NO_ERROR) {
                LeakDetector_Process(audio_buf, AUDIO_FRAME_SIZE, &metrics);
            } else {
                LeakDetector_GetMetrics(&metrics);
            }

            /* 2. Format human-readable text telemetry string for LightBlue & Serial */
            char telem[128];
            const char *src_tag = Audio_Mic_GetSourceName();
            if (LeakDetector_IsCalibrating()) {
                snprintf(telem, sizeof(telem),
                         "[CALIBRATING][%s] RMS:%4u | Reading outdoor pipe baseline...\r\n",
                         src_tag, (unsigned int)metrics.rms);
            } else if (metrics.leak_detected) {
                snprintf(telem, sizeof(telem),
                         "*** LEAK DETECTED! ***[%s] RMS:%4u | Thr:%4u | Sev:%2u%% | Conf:%2u%%\r\n",
                         src_tag, (unsigned int)metrics.rms, (unsigned int)metrics.threshold_rms,
                         (unsigned int)metrics.severity_pct, (unsigned int)metrics.confidence_pct);
            } else if (metrics.state == LEAK_STATE_SUSPECTED) {
                snprintf(telem, sizeof(telem),
                         "[SUSPECTED][%s] RMS:%4u | Thr:%4u | Verifying persistence (%u/3)...\r\n",
                         src_tag, (unsigned int)metrics.rms, (unsigned int)metrics.threshold_rms,
                         (unsigned int)metrics.persistence_cnt);
            } else {
                snprintf(telem, sizeof(telem),
                         "[NORMAL][%s] RMS:%4u | Base:%4u | Thr:%4u | Pipe OK\r\n",
                         src_tag, (unsigned int)metrics.rms, (unsigned int)metrics.baseline_rms,
                         (unsigned int)metrics.threshold_rms);
            }

            /* 3. Output to UART0 Console (115200 8-N-1) */
            TerminalTxStr(telem);
            printf("%s", telem);

            /* 4. Stream to connected Phone via BLE Notification */
            LeakAppSendTelemetry(telem);

            /* 5. Update Board Status LEDs */
            if (metrics.leak_detected) {
                LED_On(LED_RED);       /* Red LED solid/alarm */
                LED_Off(LED_GREEN);
            } else {
                LED_Off(LED_RED);
                LED_Toggle(LED_GREEN); /* Green LED heartbeat toggle */
            }

            /* Restart timer for next interval */
            WsfTimerStartMs(&s_leakTimer, LEAK_TIMER_PERIOD_MS);
            return;
        }

        /* Process ATT messages */
        if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END) {
            AppServerProcAttMsg(pMsg);
        } else if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END) {
            AppSlaveProcDmMsg((dmEvt_t *)pMsg);
            AppSlaveSecProcDmMsg((dmEvt_t *)pMsg);
        }

        leakProcMsg((dmEvt_t *)pMsg);
    }
}

/*************************************************************************************************/
void LeakAppStart(void)
{
    /* Register for BLE stack callbacks */
    DmRegister(leakDmCback);
    DmConnRegister(DM_CLIENT_ID_APP, leakDmCback);
    AttRegister(leakAttCback);
    AttConnRegister(AppServerConnCback);
    AttsCccRegister(LEAK_NUM_CCC_IDX, (attsCccSet_t *)leakCccSet, leakCccCback);

    /* Initialize GATT services */
    SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
    SvcCoreAddGroup();
    SvcWpCbackRegister(NULL, leakWpWriteCback);
    SvcWpAddGroup();

    SvcSecDataCbackRegister(NULL, leakSecDatWriteCback);
    SvcSecDataAddGroup();

    GattSetSvcChangedIdx(LEAK_GATT_SC_CCC_IDX);

    AppUiBtnRegister(leakBtnCback);
    PalBtnInit(btnPressHandler);

    s_termCmdMicInt.pNext = NULL;
    s_termCmdMicInt.pName = "mic_int";
    s_termCmdMicInt.pHelpStr = "Switch to Internal MEMS Microphone";
    s_termCmdMicInt.handler = termCmdMicInt;
    TerminalRegisterCommand(&s_termCmdMicInt);

    s_termCmdMicExt.pNext = NULL;
    s_termCmdMicExt.pName = "mic_ext";
    s_termCmdMicExt.pHelpStr = "Switch to External Line-In Microphone (3.5mm)";
    s_termCmdMicExt.handler = termCmdMicExt;
    TerminalRegisterCommand(&s_termCmdMicExt);

    s_termCmdLeak.pNext = NULL;
    s_termCmdLeak.pName = "leak";
    s_termCmdLeak.pHelpStr = "Inject simulated water pipe leak";
    s_termCmdLeak.handler = termCmdLeak;
    TerminalRegisterCommand(&s_termCmdLeak);

    s_termCmdStop.pNext = NULL;
    s_termCmdStop.pName = "stop";
    s_termCmdStop.pHelpStr = "Stop simulated leak and return to baseline";
    s_termCmdStop.handler = termCmdStop;
    TerminalRegisterCommand(&s_termCmdStop);

    WsfNvmInit();

    /* Reset the device to start stack operation */
    DmDevReset();
}
