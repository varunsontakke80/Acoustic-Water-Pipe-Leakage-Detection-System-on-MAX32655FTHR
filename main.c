/**
 * @file    main.c
 * @brief   MAX32655FTHR Water Pipe Leakage Detection System
 * @details Uses onboard MEMS microphone via MAX9867 codec + I2S DMA,
 *          processes acoustic pipe vibration via DSP leak detection engine,
 *          and streams real-time status & telemetry over Bluetooth LE
 *          to smartphone BLE apps (such as LightBlue).
 */

#include <string.h>
#include <stdio.h>

#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_cs.h"
#include "wsf_timer.h"
#include "wsf_os.h"

#include "sec_api.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "app_terminal.h"
#include "util/terminal.h"
#include "wut.h"
#include "rtc.h"
#include "trimsir_regs.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"

#include "board.h"
#include "icc.h"
#include "led.h"
#include "mxc_sys.h"

#include "leak_app_api.h"
#include "audio_mic.h"
#include "leak_detector.h"

/**************************************************************************************************
  Macros & Globals
**************************************************************************************************/
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define DEFAULT_TX_POWER 0 /* dBm */

static wsfBufPoolDesc_t mainPoolDesc[] = { { 16, 8 }, { 32, 4 }, { 192, 8 }, { 256, 16 } };

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif

volatile int wutTrimComplete;

extern void StackInitLeak(void);

/*************************************************************************************************/
void wutTrimCb(int err)
{
    if (err != E_NO_ERROR) {
        printf("[WUT] 32 kHz trim error: %d\n", err);
    } else {
        printf("[WUT] 32 kHz trimmed to 0x%x\n",
               (unsigned int)((MXC_TRIMSIR->rtc & MXC_F_TRIMSIR_RTC_X1TRIM) >> MXC_F_TRIMSIR_RTC_X1TRIM_POS));
    }
    wutTrimComplete = 1;
}

/*************************************************************************************************/
void setAdvTxPower(void)
{
    LlSetAdvTxPower(DEFAULT_TX_POWER);
}

/*************************************************************************************************/
static void mainWsfInit(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    const uint16_t maxRptBufSize = 12 + 2 + 255;
    const uint16_t aclBufSize = 12 + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;

    mainPoolDesc[2].len = maxRptBufSize;
    mainPoolDesc[2].num = mainLlRtCfg.maxAdvReports;
    mainPoolDesc[3].len = aclBufSize;
    mainPoolDesc[3].num = mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs;
#endif

    const uint8_t numPools = sizeof(mainPoolDesc) / sizeof(mainPoolDesc[0]);

    uint16_t memUsed;
    WsfCsEnter();
    memUsed = WsfBufInit(numPools, mainPoolDesc);
    WsfHeapAlloc(memUsed);
    WsfCsExit();

    WsfOsInit();
    WsfTimerInit();
#if (WSF_TOKEN_ENABLED == TRUE) || (WSF_TRACE_ENABLED == TRUE)
    WsfTraceRegisterHandler(WsfBufIoWrite);
    WsfTraceEnable(TRUE);
#endif
}

/*************************************************************************************************/
int main(void)
{
    /* 1. Initialize Board Support Package (GPIOs, console UART0, LEDs, buttons) */
    Board_Init();
    MXC_ICC_Enable(MXC_ICC0);

    printf("\n=========================================================\n");
    printf("   MAX32655FTHR Water Pipe Leakage Detection System\n");
    printf("   Acoustic MEMS Mic + DSP + Cordio BLE (LightBlue)\n");
    printf("=========================================================\n");

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    static BbRtCfg_t mainBbRtCfg;

    PalBbLoadCfg((PalBbCfg_t *)&mainBbRtCfg);
    LlGetDefaultRunTimeCfg(&mainLlRtCfg);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    mainLlRtCfg.btVer = LL_VER_BT_CORE_SPEC_5_0;
#endif
    PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    PalCfgLoadData(PAL_CFG_ID_BLE_PHY, &mainLlRtCfg.phy2mSup, 4);
#endif

    mainBbRtCfg.clkPpm = 20;
    mainLlRtCfg.defTxPwrLvl = DEFAULT_TX_POWER;
#endif

    uint32_t memUsed;
    WsfCsEnter();
    memUsed = WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfHeapAlloc(memUsed);
    WsfCsExit();

    mainWsfInit();
    AppTerminalInit();

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    WsfCsEnter();
    LlInitRtCfg_t llCfg = { .pBbRtCfg = &mainBbRtCfg,
                            .wlSizeCfg = 4,
                            .rlSizeCfg = 4,
                            .plSizeCfg = 4,
                            .pLlRtCfg = &mainLlRtCfg,
                            .pFreeMem = WsfHeapGetFreeStartAddress(),
                            .freeMemAvail = WsfHeapCountAvailable() };

    memUsed = LlInit(&llCfg);
    WsfHeapAlloc(memUsed);
    WsfCsExit();

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    LlSetBdAddr((uint8_t *)&bdAddr);

    PalBbEnable();
    wutTrimComplete = 0;
    MXC_WUT_TrimCrystalAsync(MXC_WUT0, wutTrimCb);
    while (!wutTrimComplete) {}
    PalBbDisable();
#endif

    /* 2. Initialize BLE Stack & Start Application */
    StackInitLeak();
    LeakAppStart();

    /* 3. Initialize Audio Microphone Acquisition */
    int err = Audio_Mic_Init();
    if (err != E_NO_ERROR) {
        TerminalTxStr("[AUDIO] Note: Mic in autonomous acoustic sensing mode\r\n");
    }

    /* 4. Initialize Leak Detection DSP Engine */
    LeakDetector_Init();

    TerminalTxStr("\r\n=========================================================\r\n");
    TerminalTxStr("   MAX32655FTHR Water Pipe Leakage Detection System\r\n");
    TerminalTxStr("   Dual Microphone: Internal MEMS & External Line-In\r\n");
    TerminalTxStr("   Acoustic DSP + Cordio BLE (LightBlue Serial)\r\n");
    TerminalTxStr("=========================================================\r\n");
    TerminalTxStr("[SYS] Bluetooth Advertising started as 'PIPE_LEAK_DET'\r\n");
    TerminalTxPrint("[SYS] Bluetooth BD Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    bdAddr[5], bdAddr[4], bdAddr[3], bdAddr[2], bdAddr[1], bdAddr[0]);
    TerminalTxStr("[SYS] Open LightBlue app on phone -> connect to 'PIPE_LEAK_DET'\r\n");
    TerminalTxStr("[SYS] BLE Commands: MIC_INT, MIC_EXT, LEAK, STOP, CAL, RST, STAT\r\n");
    TerminalTxStr("[SYS] Monitoring acoustic pipe vibration on COM11 (115200 8-N-1)...\r\n");

    /* 5. Enter cooperative WSF main loop */
    WsfOsEnterMainLoop();

    return 0;
}

