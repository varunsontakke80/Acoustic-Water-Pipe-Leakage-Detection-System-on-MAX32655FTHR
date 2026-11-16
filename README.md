# Water Pipe Leakage Detection System (MAX32655FTHR)

An edge acoustic monitoring and water pipe leakage detection firmware developed for the **MAX32655FTHR** evaluation board. The system captures acoustic vibrations directly from the pipe using either the **Onboard Digital MEMS Microphone** or an **External 3.5mm Line-In Contact Microphone**, processes the audio using real-time DSP algorithms on the Arm Cortex-M4F core, and streams live telemetry over Bluetooth Low Energy (BLE) to mobile apps such as **LightBlue**.

---

## Table of Contents
1. [Overview & Key Features](#overview--key-features)
2. [Dual Microphone Support (Internal MEMS vs External Line-In)](#dual-microphone-support-internal-mems-vs-external-line-in)
3. [Physical Pipe Attachment & Outdoor Mounting](#physical-pipe-attachment--outdoor-mounting)
4. [Acoustic Detection DSP Principles](#acoustic-detection-dsp-principles)
5. [LightBlue Mobile App Guide (Step-by-Step)](#lightblue-mobile-app-guide-step-by-step)
6. [Interactive BLE & Serial Commands](#interactive-ble--serial-commands)
7. [LED Status Indicators](#led-status-indicators)
8. [Project File Structure](#project-file-structure)
9. [Build, Flash & Run Instructions](#build-flash--run-instructions)
10. [Serial Console Output](#serial-console-output)

---

## Overview & Key Features

Pressurized water leaking through small orifices, cracks, or loose fittings generates continuous acoustic turbulence and high-frequency wall vibrations. This project transforms the MAX32655FTHR into an autonomous, non-invasive pipe health monitor:

- **Dual Acoustic Sensing:** Supports switching between the onboard digital MEMS microphone and an external contact/hydrophone microphone plugged into the 3.5 mm Line-In jack.
- **Hardware Preamplifiers:** Maxim MAX9867 codec provides up to **+24 dB Line-In gain** and **+30 dB / +20 dB analog mic boost** for maximum acoustic sensitivity on weak leaks.
- **Stationary vs. Transient DSP Classifier:** Rejects ambient spikes (footsteps, traffic rumble, wind gusts, tool drops) by analyzing Crest Factor ($Peak / RMS$) and multi-frame persistence.
- **Zero-Config Baseline Calibration:** Automatically learns the normal background vibration baseline of the pipe via button press, remote BLE command, or serial terminal.
- **Serial-over-BLE Streaming:** Broadcasts real-time human-readable telemetry packets over Bluetooth 5.2 LE to any smartphone serial app (LightBlue, nRF Connect, Serial Bluetooth).
- **Two-Way Remote Control:** Configure thresholds, switch and test microphones, initiate recalibration, or simulate leaks directly from your smartphone over BLE.
- **Low-Power Ready:** Operates in a periodic evaluation loop, allowing the MCU to maintain low power while preserving BLE responsiveness.

---

## Dual Microphone Support (Internal MEMS vs External Line-In)

The firmware provides runtime switching between two microphone sources:

| Source | Physical Port | Codec Configuration | Use Case |
|---|---|---|---|
| **`INT_MIC`** (Internal) | Onboard Digital MEMS Mic | PDM/I2S Digital Mic enabled (`0x30` in reg `0x15`), analog inputs muted. | Direct surface mounting of the MAX32655FTHR board against the pipe wall. |
| **`EXT_MIC`** (External) | 3.5 mm Audio Jack (Line-In) | Analog Line-In/Mic enabled (`0xF0` in reg `0x14`), +24 dB preamp gain (`0x0F` in regs `0x0E`/`0x0F`), +30 dB analog boost (`0xD4` in regs `0x12`/`0x13`). | Attaching an external piezo sensor, hydrophone, or lavalier mic clamped to an outdoor pipe while keeping the board protected indoors/junction box. |

### How to Switch and Test Microphones

You can switch and test both microphones using either **Bluetooth (LightBlue)** or **Serial Console (COM11)**:

1. **Test Internal Microphone:**
   - **LightBlue Command:** Send `MIC_INT` (or `TEST_INT`)
   - **COM11 Terminal:** Type `mic_int`
   - **Board Action:** Switches codec to internal digital mic, performs active acoustic sampling test (measures RMS, min, max), initiates 10-second baseline recalibration, and responds:
     ```text
     [CMD] MIC=INTERNAL MEMS | Audio: RMS=95 [-82..91] HW:OK | Calibrating...
     ```
   - **Telemetry:** Periodic readings update with the `[INT_MIC]` tag:
     ```text
     [NORMAL][INT_MIC] RMS: 0092 | Base: 0090 | Thr: 0450 | Pipe OK
     ```

2. **Test External Line-In Microphone:**
   - **LightBlue Command:** Send `MIC_EXT` (or `TEST_EXT`)
   - **COM11 Terminal:** Type `mic_ext`
   - **Board Action:** Switches codec to 3.5mm Line-In with +24 dB gain, performs active sampling test, initiates baseline recalibration, and responds:
     ```text
     [CMD] MIC=EXTERNAL LINE-IN (+24dB) | Audio: RMS=140 [-120..138] HW:OK | Calibrating...
     ```
   - **Telemetry:** Periodic readings update with the `[EXT_MIC]` tag:
     ```text
     [NORMAL][EXT_MIC] RMS: 0135 | Base: 0130 | Thr: 0650 | Pipe OK
     ```

---

## Physical Pipe Attachment & Outdoor Mounting

For reliable detection of outdoor pipe leaks, proper mechanical coupling is essential:

```
          [ Outdoor Water Pipe (Metal / Copper / PVC) ]
       ====================================================
                ||||||||||||||||||||||||||||||||||
                [ Acoustic Coupling Pad / Rubber ]
                [ MAX32655FTHR Evaluation Board   ]
                [ (Microphone facing pipe body)   ]
                ==================================
                 [ Heavy-duty Zip Ties / Clamp ]
```

### Mounting Recommendations:
1. **Location:** Mount the sensor near pipe junctions, valves, elbows, or suspected leak points. Acoustic vibrations travel farthest along metal pipes (up to 50–100 m) and shorter distances in PVC pipes (10–20 m).
2. **Acoustic Coupling:** Place a thin, firm rubber pad or acoustic contact gel between the sensor and the pipe wall to transfer mechanical vibrations directly into the sensor without air dampening.
3. **Firm Attachment:** Secure the sensor tightly against the pipe using heavy-duty zip ties or a pipe bracket.
4. **Weatherproofing:** If left outdoors, enclose the board in a weatherproof junction box (IP65/IP67 rated).

---

## Acoustic Detection DSP Principles

Water leaks have distinctive acoustic and statistical characteristics compared to ambient noise:

### 1. High-Pass DC Blocker
Removes sensor DC offset and low-frequency mechanical drift ($< 300\text{ Hz}$):
$$y[n] = x[n] - x[n-1] + 0.95 \cdot y[n-1]$$

### 2. Root-Mean-Square (RMS) Energy
Calculates the continuous acoustic energy floor across each evaluation window:
$$\text{RMS} = \sqrt{\frac{1}{N} \sum_{n=0}^{N-1} y[n]^2}$$

### 3. Crest Factor Filtering ($CF = Peak / RMS$)
- **Ambient Transient Noises (Footsteps, Car Doors, Wind, Taps):** Sharp impulses have very high peak amplitudes but brief duration ($CF > 5.5$). These are tagged as `TRANSIENT` and discarded.
- **Continuous Pressurized Leak Hiss:** Turbulent fluid cavitation creates continuous, stationary Gaussian-like hiss ($CF \in [1.2, 5.0]$).

### 4. Multi-Frame Persistence Verification
An alarm is only confirmed when elevated stationary acoustic energy persists for **at least 3 consecutive seconds** (`PERSISTENCE_CONFIRM_FRAMES = 3`).

---

## LightBlue Mobile App Guide (Step-by-Step)

1. **Install LightBlue:** Available free on iOS App Store and Android Google Play Store.
2. **Enable Bluetooth** on your mobile device.
3. **Scan for Devices:** Open LightBlue. Look for the peripheral named **`PIPE_LEAK_DET`**.
4. **Connect:** Tap on **`PIPE_LEAK_DET`**. The Blue LED on the board illuminates solid.
5. **Locate Service:** Find Service UUID `0000FEF6-0000-1000-8000-00805F9B34FB` (Custom Data Service `E0262760-08C2-11E1-9073-0E8AC72E1001`).
6. **Open Telemetry Characteristic:** Tap Characteristic `E0262761-08C2-11E1-9073-0E8AC72E1001` (Properties: `Read, Write, Notify`).
7. **Enable Live Notifications:** Tap **"Listen for notifications"** (or **"Subscribe"**). Telemetry updates stream every 1 second in UTF-8 text.

---

## Interactive BLE & Serial Commands

From LightBlue, tap **"Write new value"** (set format to **UTF-8 String**), or type directly into the COM11 serial terminal:

| BLE Command | Serial Command | Action | Description |
|---|---|---|---|
| `MIC_INT` | `mic_int` | **Test & Switch to Internal Mic** | Switches to onboard MEMS mic, tests capture, runs 10s calibration. |
| `MIC_EXT` | `mic_ext` | **Test & Switch to External Mic** | Switches to 3.5mm Line-In mic (+24 dB gain), tests capture, runs 10s calibration. |
| `LEAK` or `TEST` | `leak` | **Simulate Pipe Leak** | Injects high-pressure turbulent leak vibration to trigger alarm & Red LED. |
| `STOP` or `NORM` | `stop` | **Stop Simulation** | Halts simulated vibration and restores pipe to normal baseline. |
| `CAL` | `cal` | **Calibrate Baseline** | Starts a 20-second learning window to record normal pipe baseline. |
| `RST` | `rst` | **Reset Alarm** | Clears a latched leak alarm back to `NORMAL`. |
| `THR=800` | `thr=800` | **Set Threshold** | Overrides detection threshold to custom RMS value. |
| `STAT` | `stat` | **Status Query** | Returns full diagnostic status (RMS, baseline, severity, confidence). |

> [!TIP]
> **Hardware Push Button SW3:** You can also press push button **SW3** directly on the MAX32655FTHR board to toggle leak simulation on/off for instant testing without opening an app!

---

## LED Status Indicators

| LED Color | State | Meaning |
|---|---|---|
| **Green** | **Pulsing / Blinking (1 Hz)** | Normal monitoring heartbeat; pipe is intact and quiet. |
| **Blue** | **Solid ON** | Smartphone (LightBlue) is actively connected over BLE. |
| **Blue** | **OFF** | Device is advertising and waiting for a connection. |
| **Red** | **Solid ON / Rapid Alert** | **CRITICAL LEAK ALARM!** Continuous acoustic leak vibration detected. |

---

## Project File Structure

```
Water_Leak_Detection_Project/
├── Makefile                # Core MSDK build system file (DEPTH := ../../../)
├── project.mk              # Project configuration (FTHR_Apps_P1, LIB_CORDIO=1, ADV_NAME)
├── main.c                  # System init, WSF event loop, peripheral bootstrap
├── stack_leak.c            # Cordio BLE stack handler registration
├── leak_app_api.h          # BLE application interface
├── leak_app_main.c         # BLE connection handlers, periodic 1 Hz timer, LightBlue comms
├── leak_detector.h         # Acoustic DSP leak detection API
├── leak_detector.c         # High-pass filter, RMS, Crest Factor, state machine
├── audio_mic.h             # Dual microphone acquisition interface
├── audio_mic.c             # Audio capture, I2C scanner, internal & line-in switching
├── max9867.h               # MAX9867 audio codec register definitions
├── max9867.c               # MAX9867 dual-input routing and preamplifier driver
├── services/
│   ├── svc_sds.h           # Secured Data Service definitions
│   └── svc_sds.c           # Secured Data Service implementation
├── .vscode/
│   ├── settings.json       # Configured for TARGET=MAX32655, BOARD=FTHR_Apps_P1
│   ├── tasks.json          # Build, flash, clean tasks
│   ├── launch.json         # Cortex-Debug OpenOCD configurations
│   ├── flash.gdb           # GDB flashing script
│   └── c_cpp_properties.json
└── README.md               # Complete project documentation
```

---

## Build, Flash & Run Instructions

```cmd
set PATH=C:\MaximSDK\Tools\GNUTools\Make;C:\MaximSDK\Tools\GNUTools\10.3\bin;C:\MaximSDK\Tools\OpenOCD;%PATH%
cd /d C:\MaximSDK\Examples\MAX32655\Water_Leak_Detection_Project

:: 1. Compile
make -r -j4 TARGET=MAX32655 BOARD=FTHR_Apps_P1 MAXIM_PATH=C:/MaximSDK

:: 2. Flash to board
openocd -s C:\MaximSDK\Tools\OpenOCD\scripts -f interface/cmsis-dap.cfg -f target/max32655.cfg -c "program build/max32655.elf verify reset exit"
```

---

## Serial Console Output (COM11, 115200 8-N-1)

```text
=========================================================
   MAX32655FTHR Water Pipe Leakage Detection System
   Dual Microphone: Internal MEMS & External Line-In
   Acoustic DSP + Cordio BLE (LightBlue Serial)
=========================================================
[SYS] Bluetooth Advertising started as 'PIPE_LEAK_DET'
[SYS] Bluetooth BD Address: 00:18:80:55:65:32
[SYS] Open LightBlue app on phone -> connect to 'PIPE_LEAK_DET'
[SYS] BLE Commands: MIC_INT, MIC_EXT, LEAK, STOP, CAL, RST, STAT
[SYS] Monitoring acoustic pipe vibration on COM11 (115200 8-N-1)...

[NORMAL][INT_MIC] RMS: 0055 | Base: 0262 | Thr: 0420 | Pipe OK
[NORMAL][INT_MIC] RMS: 0055 | Base: 0262 | Thr: 0420 | Pipe OK

[BLE_RX] Command received: 'MIC_EXT'
[AUDIO] Switched input source to: EXTERNAL LINE-IN MICROPHONE (3.5mm)
[CMD] MIC=EXTERNAL LINE-IN (+24dB) | Audio: RMS=140 [-120..138] HW:OK | Calibrating...

[NORMAL][EXT_MIC] RMS: 0135 | Base: 0130 | Thr: 0420 | Pipe OK
```
