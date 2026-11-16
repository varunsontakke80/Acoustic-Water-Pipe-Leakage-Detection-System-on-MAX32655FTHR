# Acoustic Water Pipe Leakage Detection System (MAX32655FTHR)

An autonomous edge acoustic monitoring and water pipe leakage detection system developed on the **Analog Devices MAX32655FTHR** Application Platform. The system acquires high-resolution mechanical pipe vibrations directly through an onboard **Knowles SPH0644HM4H-1 Digital MEMS Microphone** (or an external contact transducer via the onboard 3.5mm audio jack), executes real-time acoustic DSP filtering on the Arm Cortex-M4F processor, and outputs continuous diagnostics, baseline tracking, and confirmed leak alarms over the DAPLink Virtual UART Console (`COM11` at 115200 baud).

---

## Table of Contents
1. [System Overview & Architecture](#system-overview--architecture)
2. [MAX32655FTHR Hardware Peripherals & Specifications](#max32655fthr-hardware-peripherals--specifications)
3. [Dual Microphone Ingestion (Internal MEMS vs External Contact)](#dual-microphone-ingestion-internal-mems-vs-external-contact)
4. [Acoustic Detection DSP Principles](#acoustic-detection-dsp-principles)
5. [Physical Pipe Mounting & Acoustic Coupling](#physical-pipe-mounting--acoustic-coupling)
6. [Real-World Pipe Testing & Experimental Verification](#real-world-pipe-testing--experimental-verification)
7. [Serial Terminal Commands & Onboard Controls](#serial-terminal-commands--onboard-controls)
8. [LED Status Indicators](#led-status-indicators)
9. [Project File Structure](#project-file-structure)
10. [Build & Flashing Guide](#build--flashing-guide)
11. [Live Serial Console Output](#live-serial-console-output)

---

## System Overview & Architecture

Pressurized fluid escaping from plumbing cracks, failed joints, or pinhole punctures produces continuous acoustic cavitation and structural wall vibrations. This project transforms the MAX32655FTHR into a non-invasive, clamp-on pipe monitor:

```
          [ Pressurized Water Pipe (Metal / Copper / PVC) ]
 =========================================================================
       |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
       [ Direct Mechanical Acoustic Contact (Pipe Wall Side)     ]
       [ MK1: Knowles SPH0644HM4H-1 Digital PDM Microphone       ]
       [ U2: Maxim MAX9867 Stereo Audio Codec (12.288 MHz MCLK)  ]
       [ U4: MAX32655 MCU (Arm Cortex-M4F @ 100 MHz, FPU)        ]
       [ Onboard Pushbuttons: SW2 (P0.2) / SW3 (P0.3) via MAX6817 ]
       [ Tri-Color Status LEDs: Red (P0.18) / Green (P0.19)       ]
 =========================================================================
                                     │
                                     ▼ (DAPLink Virtual UART0 @ 115200 Baud)
                            [ Serial Monitor ]
```

### Key Technical Capabilities:
* **High-Fidelity Audio Ingestion:** 16-bit PCM audio frames sampled at 48 kHz via I2S DMA using the onboard Knowles digital MEMS microphone and Maxim MAX9867 codec.
* **Hardware Preamplifier Headroom:** The MAX9867 codec provides up to **+24 dB Line-In preamplifier gain** and **+30 dB / +20 dB analog mic boost** when using external contact transducers or hydrophones.
* **Stationary vs. Transient DSP Classifier:** Rejects ambient spikes (footsteps, traffic rumble, tool impacts) by calculating Crest Factor ($Peak / RMS$) and multi-frame persistence.
* **Adaptive Baseline Learning:** Learns the pipe's normal laminar flow baseline noise floor via onboard pushbutton SW3 or terminal command.
* **Low-Power Efficiency:** The DSP evaluation frame executes in under 1.8 ms, allowing the Cortex-M4F core to sleep between monitoring windows.

---

## MAX32655FTHR Hardware Peripherals & Specifications

The project utilizes the specialized hardware integrated onto the MAX32655FTHR platform:

| Component | Part / Designation | Interface to MCU | Function in Project |
|---|---|---|---|
| **Microcontroller** | MAX32655 (U4) | Dual Core (M4F + RISC-V) | 100 MHz Arm Cortex-M4F with hardware FPU, 512 KB Flash, 128 KB SRAM. |
| **Digital Microphone** | Knowles SPH0644HM4H-1 (MK1) | PDM to MAX9867 Codec | Captures physical mechanical vibrations directly from the pipe wall. |
| **Audio Codec** | Maxim MAX9867ETJ+ (U2) | I2C1 (`P0.16` SCL, `P0.17` SDA)<br>I2S (`P1.2` SCK, `P1.3` WS, `P1.4` SDI) | Stereo audio codec clocked by an Epson 12.288 MHz crystal (`Y1`); acts as I2S Master. |
| **External Audio Jack** | CUI SJ-3523-SMT (J1) | Analog Line-In (`LINL`/`LINR`) | 3.5mm stereo line-in jack on bottom side for external contact/piezo transducers. |
| **Pushbuttons** | Panasonic EVP-AA102K (SW2, SW3) | `P0.2` (SW2), `P0.3` (SW3)<br>via MAX6817 debouncer (U10) | User control buttons: SW3 triggers normal flow baseline calibration. |
| **Status RGB LED** | Inolux IN-B101FCH (D1) | Active-Low GPIOs (`P0.18` Red, `P0.19` Green, `P0.26` Blue) | Visual indicators: Green = Normal Monitoring, Red = Confirmed Leak Alarm. |
| **DAPLink Debugger** | MAX32625 (U9) | SWD + UART0 (`P0.0` RX, `P0.1` TX) | Flashing, debugging, and 115200 baud serial telemetry communication. |
| **Power Management** | Maxim MAX20303 (U6) | I2C1 (`P0.16`, `P0.17`) | System power rail distribution and USB charging management. |

---

## Dual Microphone Ingestion (Internal MEMS vs External Contact)

The firmware dynamically reconfigures the MAX9867 audio codec registers to select between two distinct acoustic sensing modes:

```
[ Internal MEMS Mode (Default) ]          [ External Contact Transducer Mode ]
 Knowles SPH0644HM4H-1 (MK1)                External Piezo / Hydrophone Sensor
              │                                             │
              ▼ (Digital PDM)                               ▼ (3.5mm Jack J1)
    MAX9867 DIGMIC Inputs                        MAX9867 Analog Line-In Stage
    (Reg 0x15 = 0x30, Line-In Muted)             (+24dB Preamp & +30dB Boost)
              │                                             │
              └──────────────► [ I2S DMA (48 kHz) ] ◄───────┘
                                       │
                                       ▼
                             [ Arm Cortex-M4F ]
```

1. **Internal Microphone (`INT_MIC`):**
   - Routes the Knowles digital MEMS microphone via digital microphone inputs (`DIGMICL` & `DIGMICR`, reg `0x15`).
   - Mutes analog line inputs to eliminate noise.
   - Ideal for direct clamp-on installation where the evaluation board is affixed to the pipe.

2. **External Microphone (`EXT_MIC`):**
   - Disables digital mic and routes the 3.5 mm Line-In jack (`J1`) into Left & Right ADCs (`0xF0` in reg `0x14`).
   - Applies **+24 dB** Line-In preamplifier gain (regs `0x0E`/`0x0F`) and **+30 dB** analog boost (regs `0x12`/`0x13`).
   - Designed for hostile outdoor/subterranean environments where the electronics reside in an indoor junction box while a remote contact sensor is clamped to the pipe.

---

## Acoustic Detection DSP Principles

Turbulent fluid escaping through an orifice produces stationary, high-frequency acoustic hiss. The on-device DSP engine processes audio in four stages:

```
[ Raw Audio Frame (512 samples @ 48 kHz) ]
                     │
                     ▼
       [ 1st-Order High-Pass DC Blocker ]      <-- Cuts sub-300Hz mechanical drift
                     │
                     ▼
          [ Continuous RMS Energy ]            <-- Measures acoustic vibration floor
                     │
                     ▼
          [ Crest Factor Filtering ]           <-- CF = Peak / RMS
          ├─ CF > 5.5  ──► Transient spike (ignored: footsteps, impacts)
          └─ CF <= 5.0 ──► Continuous hiss (pressurized leak candidate)
                     │
                     ▼
     [ Multi-Second Persistence Check ]        <-- Must sustain for >= 3 seconds
                     │
                     ▼
      [ Pipe Status & Leak Alarm Output ]      <-- Red LED ON + Serial Alarm Telemetry
```

### Mathematical Foundations:
1. **DC Blocker:** Eliminates sensor DC offset and low-frequency handling rumble:
   $$y[n] = x[n] - x[n-1] + 0.95 \cdot y[n-1]$$
2. **Root-Mean-Square (RMS) Energy:** Quantifies the physical energy level across the 512-sample evaluation window:
   $$\text{RMS} = \sqrt{\frac{1}{N} \sum_{n=0}^{N-1} y[n]^2}$$
3. **Crest Factor Filtering ($CF = Peak / RMS$):** Sharp ambient knocks produce brief, high peak amplitudes with little energy ($CF > 5.5$). Pressurized fluid leaks produce continuous, stationary Gaussian noise with lower crest factors ($CF \in [1.2, 5.0]$).
4. **Persistence Verification:** Prevents false alarms by requiring elevated acoustic energy to persist continuously for at least **3 consecutive seconds** before latching the leak alarm.

---

## Physical Pipe Mounting & Acoustic Coupling

For optimal vibration transfer from the pipe into the sensor:

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

### Installation Guidelines:
1. **Sensor Orientation:** Position the MAX32655FTHR with the Knowles MEMS microphone (`MK1`) facing directly against the pipe surface.
2. **Coupling Medium:** Place a firm rubber dampening pad or acoustic coupling tape between the board and the pipe wall to maximize mechanical transfer and prevent acoustic leakage into open air.
3. **Rigid Clamping:** Secure the assembly tightly using zip ties or a hose clamp.
4. **Enclosure:** For permanent outdoor installations, enclose the board in an IP65/IP67 weatherproof housing.

---

## Real-World Pipe Testing & Experimental Verification

The system was physically evaluated on an active water pipe installation using the following experimental procedure:

```
          [ Water Pipe (Laminar Flow) ] ──► [ Obstructed Outlet ]
     ==============================================================
             |||||||||||||||||||||||||||||
             [ Internal MEMS Mic on Pipe Side ]
             [ MAX32655FTHR Circuit Board      ]
             [ Taped Firmly to Pipe Body       ]
     ==============================================================
```

### Experimental Procedure:
1. **Mounting:** The MAX32655FTHR board was taped directly to the exterior of the pipe with the **internal MEMS microphone facing the pipe side**, ensuring direct mechanical contact with the pipe wall.
2. **Normal Flow Baseline Calibration:** Water was allowed to run under normal laminar flow conditions. Pressing onboard **Switch SW3** (connected to `P0.3` through the MAX6817 debouncer) triggered baseline learning. Over a 20-frame window, the system measured the background flow noise floor ($\text{RMS} \approx 55$) and established an adaptive leak detection threshold ($\text{Thr} = 110$).
3. **Acoustic Leak Generation:** To generate an authentic pressurized leakage signature, the open end of the pipe was partially obstructed. This restriction created turbulent fluid cavitation and high-frequency hissing vibrations, accurately reproducing an orifice rupture.
4. **Real-Time Detection:** The MAX32655FTHR detected the sudden elevation in stationary acoustic energy. The DSP verified the continuous persistence of the signal, latched the **Red Alarm LED** (`P0.18`), and streamed confirmed alert telemetry to the serial monitor.
5. **Restoration:** Removing the obstruction allowed the fluid to resume smooth laminar flow; the acoustic RMS immediately dropped below the threshold and the Green heartbeat LED resumed.

---

## Serial Terminal Commands & Onboard Controls

From any serial terminal connected to **`COM11`** at **115200 baud (8-N-1)**, the following commands are available:

| Command | Action | Description |
|---|---|---|
| `mic_int` | **Select Internal Mic** | Reconfigures codec for onboard Knowles MEMS microphone, measures audio, and recalibrates baseline. |
| `mic_ext` | **Select External Mic** | Reconfigures codec for 3.5mm Line-In with +24 dB gain & +30 dB analog boost, measures audio, and recalibrates. |
| `cal` | **Calibrate Baseline** | Initiates a 20-frame calibration window to record the current pipe acoustic baseline. |
| `stat` | **Diagnostic Status** | Returns instantaneous RMS, baseline, threshold, severity %, and confidence %. |
| `rst` | **Reset Alarm** | Clears a latched leak alarm back to normal monitoring. |

### Onboard Hardware Pushbutton Controls:
* **Pushbutton SW3 (`P0.3` via MAX6817):** Triggers immediate acoustic baseline calibration for normal flow.
* **Pushbutton SW2 (`P0.2` via MAX6817):** User-programmable auxiliary function button.
* **Pushbutton SW5:** Hardware reset button (connected to MAX32655 `RSTN`).

---

## LED Status Indicators

The onboard tri-color RGB LED (`D1`) provides immediate visual indication:

| LED Color | MCU Pin | State | System Meaning |
|---|---|---|---|
| **Green** | `P0.19` (Active-Low) | **1 Hz Pulsing Heartbeat** | Normal monitoring; laminar flow, pipe intact. |
| **Red** | `P0.18` (Active-Low) | **Solid ON** | **CONFIRMED LEAK ALARM!** High acoustic cavitation detected. |
| **Blue** | `P0.26` (Active-Low) | **Solid ON** | System active and running. |

---

## Project File Structure

```
Water_Leak_Detection_Project/
├── Makefile                    # Core MSDK build system file (DEPTH := ../../../)
├── project.mk                  # Project configuration (TARGET=MAX32655, BOARD=FTHR_Apps_P1)
├── main.c                      # Hardware bootstrap, event loop, peripheral initialization
├── stack_leak.c                # System stack handler registration
├── leak_app_api.h              # Application API definitions
├── leak_app_main.c             # Periodic 1 Hz evaluation timer, terminal command parser
├── leak_detector.h             # Acoustic DSP leak detection engine headers
├── leak_detector.c             # DC blocker, RMS calculator, Crest Factor, persistence state machine
├── audio_mic.h                 # Dual-microphone audio interface
├── audio_mic.c                 # I2S DMA frame capture and codec initialization
├── max9867.h                   # MAX9867 codec register definitions
├── max9867.c                   # MAX9867 I2C driver, routing, and preamplifier control
├── .vscode/
│   ├── settings.json           # VS Code settings for MAX32655FTHR
│   ├── tasks.json              # Build, flash, and clean tasks
│   ├── launch.json             # Cortex-Debug OpenOCD configurations
│   └── c_cpp_properties.json
├── DIGIKEY_FORUM_SUBMISSION.md # DigiKey TechForum contest submission document
└── README.md                   # Complete technical documentation
```

---

## Build & Flashing Guide

### Build Requirements:
* **Analog Devices MSDK (v2024)**
* **GNU Arm Embedded Toolchain (`arm-none-eabi-gcc` 10.3)**
* **OpenOCD** with CMSIS-DAP support

### Compilation:
Open a command prompt and navigate to the project directory:
```cmd
set PATH=C:\MaximSDK\Tools\GNUTools\Make;C:\MaximSDK\Tools\GNUTools\10.3\bin;C:\MaximSDK\Tools\OpenOCD;%PATH%
cd /d C:\MaximSDK\Examples\MAX32655\Water_Leak_Detection_Project

make -r -j4 TARGET=MAX32655 BOARD=FTHR_Apps_P1 MAXIM_PATH=C:/MaximSDK
```

### Flashing via OpenOCD:
```cmd
openocd -s C:\MaximSDK\Tools\OpenOCD\scripts -f interface/cmsis-dap.cfg -f target/max32655.cfg -c "program build/max32655.elf verify reset exit"
```

### Memory Footprint:
* **Flash Usage:** **211,476 bytes** (41.64% of 496 KB available Flash).
* **Static SRAM Usage:** **15,348 bytes** (11.71% of 128 KB available SRAM).

---

## Live Serial Console Output

Connect your serial monitor to the DAPLink Virtual COM Port (**`COM11`**, **115200 baud, 8-N-1**):

```text
=========================================================
   MAX32655FTHR Water Pipe Leakage Detection System
   Knowles SPH0644 MEMS Mic + MAX9867 Codec + Edge DSP
=========================================================
[SYS] Monitoring acoustic pipe vibration on COM11 (115200 8-N-1)...

[NORMAL][INT_MIC] RMS: 0055 | Base: 0055 | Thr: 0110 | Pipe OK
[NORMAL][INT_MIC] RMS: 0055 | Base: 0055 | Thr: 0110 | Pipe OK

>>> [BUTTON SW3] Calibrating normal flow baseline...
[CALIBRATING][INT_MIC] RMS: 0054 | Reading pipe baseline...
[CALIBRATING][INT_MIC] RMS: 0055 | Reading pipe baseline...
[SYS] Baseline Calibration Complete: Baseline RMS = 55, Threshold RMS = 110

[NORMAL][INT_MIC] RMS: 0056 | Base: 0055 | Thr: 0110 | Pipe OK

[SUSPECTED][INT_MIC] RMS: 0420 | Thr: 0110 | Verifying persistence (1/3)...
[SUSPECTED][INT_MIC] RMS: 0435 | Thr: 0110 | Verifying persistence (2/3)...
*** LEAK DETECTED! ***[INT_MIC] RMS: 0440 | Thr: 0110 | Sev: 78% | Conf: 95%
*** LEAK DETECTED! ***[INT_MIC] RMS: 0445 | Thr: 0110 | Sev: 80% | Conf: 96%
```
