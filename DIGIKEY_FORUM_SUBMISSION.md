# [Learn, Build & Win] Acoustic Water Pipe Leakage Detection System on MAX32655FTHR

**Contest:** DigiKey & Circuit Digest — *Learn, Build & Win Contest*  
**Category:** Maker, DIY (`/c/maker/118`)  
**Target Platform:** Analog Devices / Maxim Integrated MAX32655FTHR Evaluation Kit  

---

## 1. Introduction & Project Premise

Hello DigiKey and Circuit Digest community!

I am excited to share the project write-up for my submission in the **Learn, Build & Win Contest**: an **Acoustic Water Pipe Leakage Detection System** powered by the **Analog Devices MAX32655FTHR**.

Hidden water pipe leaks in domestic and industrial plumbing silently waste huge amounts of treated water, erode foundations, and cause structural damage before any visible moisture appears on surfaces. Pressurized fluid spraying through a crack or loose fitting produces distinct mechanical vibrations and high-frequency acoustic hiss along the pipe wall. 

The goal of this project is to build an autonomous, non-invasive edge acoustic monitor using the **MAX32655** (Arm Cortex-M4F with FPU). By mounting the board directly against a pipe, the system listens to physical acoustic vibrations via the onboard microphone, filters ambient disturbances using an on-device Digital Signal Processing (DSP) engine, and reports live pipe health and leak diagnostics directly to a serial console.

---

## 2. Hardware Architecture & System Overview

```
       [ Water Pipe with Normal Flow / Pressurized Leak ]
 ================================================================
       |||||||||||||||||||||||||||||||||||||||||||||||||||||||
       [ Direct Mechanical Acoustic Contact (Pipe Wall Side) ]
       [ Onboard Digital MEMS Microphone                     ]
       [ MAX9867 Ultra-Low-Power Stereo Audio Codec          ]
       [ MAX32655 Microcontroller (Arm Cortex-M4F @ 100 MHz) ]
       [ Onboard Pushbuttons: SW3 / SW4                      ]
       [ Tri-Color Status LEDs (Red Alarm / Green Heartbeat)  ]
 ================================================================
                                │
                                ▼ (DAPLink USB UART0 @ 115200 Baud)
                       [ PC Serial Monitor ]
```

### Key Hardware Peripherals Used:
* **Microcontroller:** MAX32655 (100 MHz Arm Cortex-M4F with hardware floating-point unit, 512 KB Flash, 128 KB SRAM).
* **Audio Frontend:** Maxim MAX9867 Ultra-Low-Power Stereo Audio Codec connected via **I²C1** (`P0.16` SCL, `P0.17` SDA).
* **Acoustic Ingestion:** Onboard Digital MEMS Microphone routed through the MAX9867 codec into the MCU via **I²S DMA** (16-bit PCM @ 48 kHz).
* **External Sensor Option:** Onboard 3.5 mm Line-In jack supported by codec analog preamplifiers (+24 dB Line-In gain, +30 dB analog boost) for optional external hydrophone or piezo contact sensors.
* **User Controls:** Onboard Pushbuttons SW3 and SW4 for baseline calibration and manual test triggering.
* **Visual Status:** Tri-color active-low RGB LED (`P0.18` Red, `P0.19` Green, `P0.20` Blue).
* **Diagnostic Console:** Virtual DAPLink UART0 streamed over USB (`COM11` at 115200 baud).

---

## 3. Toolchain & Board Bring-Up

### Development Environment:
* **Analog Devices MSDK (Maxim SDK v2024)**
* **VS Code** with GNU Arm Embedded Toolchain (`arm-none-eabi-gcc` 10.3)
* **OpenOCD** via the onboard CMSIS-DAP debugger interface

> [!TIP]
> **Developer Tip for the MSDK:**  
> When creating your project workspace, ensure that your directory path contains **no spaces** (for example, `C:\MaximSDK\Examples\MAX32655\Water_Leak_Detection_Project`). The GNU `make` build system included in the SDK will fail if spaces appear in the path.

### Configuration & Active-Low LEDs:
In `project.mk`, the target board is defined as:
```makefile
TARGET = MAX32655
BOARD = FTHR_Apps_P1
```
On the MAX32655FTHR, the onboard RGB LEDs are **active-low** (connected to $V_{DDIO}$ with the MCU sinking current):
* **Red LED (`P0.18`):** Solid ON during a confirmed leak alarm.
* **Green LED (`P0.19`):** 1 Hz heartbeat toggle during normal monitoring.
* **Blue LED (`P0.20`):** System status indicator.

---

## 4. Audio Pipeline & Codec Communication

On the MAX32655FTHR, the digital MEMS microphone connects to the **MAX9867 audio codec**, which acts as the I2S master clock generator (generating BCLK and LRCLK). The MCU acts as an I2S slave receiver with DMA channel 0 capturing 512-sample halfword audio frames.

### Probing the I²C Bus:
The MAX9867 codec is controlled over `MXC_I2C1` (`P0.16` / `P0.17`). During initial bring-up, an I2C bus scan verified the connected peripherals:
* **`0x18` (7-bit):** MAX9867 Audio Codec (Revision ID register `0xFF` returned `0x42`).
* **`0x28` (7-bit):** MAX20303 Power Management IC (PMIC).

### Debugging Note & Root Cause Analysis:
During early initialization, a call to vendor PMIC functions (`Microphone_Power`) caused a lockup inside `max20303.c:185` due to an un-timeouted check waiting for a PMIC response on an unclocked command channel. Checking the board schematic confirmed that the digital MEMS microphone is powered directly from the board's 3.3V rail. Removing the redundant PMIC power call eliminated the delay and achieved reliable, sub-150ms boot times.

---

## 5. Edge DSP Leak Detection Algorithm

Water leaking through an orifice creates continuous, stationary turbulent flow hiss, which has completely different statistical properties than ambient noises (such as footsteps, ambient voices, or objects bumping into the pipe).

The on-device DSP pipeline runs in four stages:

```
[ Raw Audio Frame (512 samples @ 48 kHz) ]
                     │
                     ▼
       [ 1st-Order High-Pass DC Blocker ]
                     │
                     ▼
          [ Continuous RMS Energy ]
                     │
                     ▼
          [ Crest Factor Filtering ]
          (CF = Peak / RMS)
          ├─ CF > 5.5  ──► Transient noise (discarded)
          └─ CF <= 5.0 ──► Continuous hiss (leak candidate)
                     │
                     ▼
     [ Multi-Second Persistence Check ]
     (Must sustain elevated RMS for >= 3 seconds)
                     │
                     ▼
      [ Pipe Status & Leak Alarm Output ]
```

### Mathematical & Statistical Principles:
1. **High-Pass DC Blocker:** Removes mechanical sensor drift and low-frequency rumble ($< 300\text{ Hz}$):
   $$y[n] = x[n] - x[n-1] + 0.95 \cdot y[n-1]$$
2. **Root-Mean-Square (RMS) Energy:** Quantifies the steady acoustic energy floor:
   $$\text{RMS} = \sqrt{\frac{1}{N} \sum_{n=0}^{N-1} y[n]^2}$$
3. **Crest Factor ($Peak / RMS$):** Sharp ambient knocks produce brief, high peak amplitudes with little energy ($CF > 5.5$). Pressurized fluid leaks produce continuous, stationary Gaussian noise with lower crest factors ($CF \in [1.2, 5.0]$).
4. **Persistence Verification:** Spurious noises are rejected by requiring elevated energy to persist for at least **3 consecutive seconds** before asserting an alarm.

---

## 6. Physical Testing Process on a Real Pipe

To test the system under realistic physical conditions, I mounted the MAX32655FTHR directly onto a water pipe:

```
          [ Water Pipe (Plastic / Copper) ]
     ===========================================
             |||||||||||||||||||||||||||||
             [ Internal MEMS Mic on Pipe Side ]
             [ MAX32655FTHR Circuit Board      ]
             [ Secured Tightly with Tape/Clamp ]
     ===========================================
```

### Step-by-Step Test Walkthrough:

1. **Mounting the Board:**  
   The MAX32655FTHR circuit board was taped firmly against the pipe with the **internal MEMS microphone facing the pipe wall side**. This direct mechanical contact allows acoustic wall vibrations to couple directly into the sensor without air dampening.
   
2. **Normal Water Flow Calibration:**  
   With normal water flowing through the pipe, I pressed onboard **Switch 3 (SW3)**. The system entered its baseline calibration mode for 20 frames, measuring the normal flow acoustic energy floor and calculating an adaptive detection threshold:
   $$\text{Threshold}_{\text{RMS}} = \text{Baseline}_{\text{RMS}} \times 2.0$$
   The Green LED resumed its 1 Hz heartbeat pulse, and the serial monitor displayed:
   ```text
   [NORMAL][INT_MIC] RMS: 0055 | Base: 0055 | Thr: 0110 | Pipe OK
   ```

3. **Generating the Pressurized Leak Sound:**  
   To produce a genuine acoustic leak signature, I partially obstructed the open end of the pipe. Restricting the pressurized outlet created turbulent fluid cavitation and high-frequency hissing vibrations, exactly mimicking an orifice pipe rupture.

4. **Detection & Serial Monitor Output:**  
   The MAX32655FTHR immediately detected the elevated acoustic vibration. The persistence counter verified the continuous nature of the sound, latched the **Red Alarm LED**, and printed real-time diagnostics on the serial monitor:

```text
=========================================================
   MAX32655FTHR Water Pipe Leakage Detection System
   Acoustic MEMS Mic + Edge DSP Leak Engine
=========================================================
[SYS] Monitoring acoustic pipe vibration on COM11 (115200 8-N-1)...

[NORMAL][INT_MIC] RMS: 0055 | Base: 0055 | Thr: 0110 | Pipe OK
[NORMAL][INT_MIC] RMS: 0056 | Base: 0055 | Thr: 0110 | Pipe OK

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

When the obstruction was removed and the water returned to normal flow, the acoustic level dropped back below threshold, demonstrating clear discrimination between normal laminar flow and turbulent leak hissing.

---

## 7. Resource & Memory Footprint

The complete build output (`max32655.elf`) demonstrates the efficiency of the MAX32655:

* **Flash Memory Usage:** **212,896 bytes** (41.92% of 496 KB available Flash).
* **Static SRAM Usage:** **15,348 bytes** (11.71% of 128 KB available SRAM).
* **DSP Processing Time:** Each audio frame evaluates in under **1.8 milliseconds**, allowing the processor to stay in low-power sleep between windows.

---

## 8. Summary & Future Roadmap

This prototype validates that the **MAX32655FTHR** can serve as an effective, self-contained acoustic leak monitor using its onboard audio hardware and DSP capabilities.

### Future Work:
* **Dedicated Mobile App via Bluetooth:**  
  While current diagnostics are viewed over the serial monitor, the MAX32655 includes an onboard 2.4 GHz radio and Bluetooth 5.2 hardware. In the future, I plan to develop a dedicated smartphone application to connect wirelessly over Bluetooth, allowing users to configure calibration thresholds, view real-time RMS charts, and receive instant push notifications whenever a leak is detected outdoors.
* **Rugged Enclosure:** Design an IP67 waterproof enclosure with a spring-loaded clamp for permanent outdoor pipe installations.
* **FFT Spectral Profiling:** Expand the filter engine using CMSIS-DSP 1024-point FFT to isolate specific 2 kHz – 5 kHz leak resonance peaks from electrical line hum.

---

### Project Repository & Commands:

* **Compile:**
  ```cmd
  make -r -j4 TARGET=MAX32655 BOARD=FTHR_Apps_P1 MAXIM_PATH=C:/MaximSDK
  ```
* **Flash via OpenOCD:**
  ```cmd
  openocd -s C:\MaximSDK\Tools\OpenOCD\scripts -f interface/cmsis-dap.cfg -f target/max32655.cfg -c "program build/max32655.elf verify reset exit"
  ```
* **Serial Monitor:** `115200 baud, 8-N-1` on DAPLink Virtual COM port (`COM11`).

Thank you to **DigiKey** and **Circuit Digest** for organizing this contest! Feedback and questions from the community are warmly welcome below!
