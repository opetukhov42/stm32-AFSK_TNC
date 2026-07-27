# STM32 Blue Pill AFSK 1200 TNC

A self-contained **1200-baud packet-radio TNC** (Terminal Node Controller) for the STM32F103C8T6 "Blue Pill"[cite: 1]. It implements the complete AFSK modem and AX.25 stack in firmware — **no external DSP or APRS library required**[cite: 1]. Everything (Bell-202 modulation/demodulation, HDLC framing, bit-stuffing, NRZI, CRC, and the AX.25 v2.2 link layer) runs on the bare microcontroller using one PWM output for transmit audio and the ADC for receive audio[cite: 1].

> ⚠️ **Amateur radio license required to transmit.**[cite: 1] Set `MYCALL` to *your own* callsign, never transmit as `NOCALL`, and bench-test into a dummy load first[cite: 1]. You are responsible for your emissions[cite: 1].

---

## Features

- **AFSK 1200 (Bell 202)** modem — 1200 Hz / 2200 Hz, NRZI, HDLC, CRC-16/X.25 FCS[cite: 1].
- **KISS mode** — works as a dumb TNC for Dire Wolf, APRSISCE/32, Xastir, UI-View, YAAC, etc.[cite: 1].
- **Command / Converse terminal** — TAPR TNC-2 style command line over USB serial[cite: 1].
- **AX.25 v2.2 connected mode** — SABM/UA/DISC/DM handshakes, I-frames, RR/RNR/REJ, go-back-N retransmission, T1/T3 timers, N2 retry cap, path reversal for incoming connects[cite: 1].
- **UI / unproto** transmit for beacons and APRS-style messages[cite: 1].
- **Beacon** — one-shot or automatic, with configurable text and path[cite: 1].
- **Digipeater** — proper WIDEn-N *New Paradigm* (fill-in and wide modes) with callsign insertion and a 30 s duplicate-suppression cache[cite: 1].
- **MHEARD** list and a **monitor** toggle[cite: 1].
- **Status LED** — steady = idle, fast blink = traffic, slow blink = connected[cite: 1].
- **GPS position source** — NMEA decoder on a separate serial port; beacons transmit live coordinates[cite: 1]. Includes toggle for hardware or software serial[cite: 1]. Also supports static hardcoded GPS coordinates.
- **Weather Station Telemetry** — Manual CLI input or automatic serial parsing for APRS weather beacons using standard telemetry formats (e.g., `270/010g015t072...`)[cite: 1].
- **Live-tunable** link parameters (`FRACK`, `RETRY`, `PACLEN`)[cite: 1].
- **Safe Direct Flash Write** — Settings save directly to Flash Memory using safe byte-level alignment to prevent USB enumeration lockups and hardware faults.

---

## Hardware

### Bill of materials

| Qty | Part | Value / Type | Purpose |
|----:|------|--------------|---------|
| 1 | MCU board | STM32F103C8T6 "Blue Pill" | The TNC[cite: 1] |
| 1 | Programmer | ST-Link V2 (clone is fine) | Flashing (easiest method)[cite: 1] |
| 2 | Resistor | 10 kΩ | RX bias divider (mid-rail)[cite: 1] |
| 1 | Trimmer pot | 10 kΩ | RX audio level[cite: 1] |
| 1 | Capacitor | 100 nF (0.1 µF) | RX AC coupling[cite: 1] |
| 1 | Capacitor | 10 nF | RX anti-alias (optional)[cite: 1] |
| 2 | Resistor | 4.7 kΩ | TX low-pass filter[cite: 1] |
| 2 | Capacitor | 10 nF | TX low-pass filter[cite: 1] |
| 1 | Capacitor | 1 µF (or 100 nF) | TX AC coupling[cite: 1] |
| 1 | Trimmer pot | 10 kΩ | TX level / deviation[cite: 1] |
| 1 | Transistor | NPN, 2N3904 / BC547 / 2N2222 | PTT switch[cite: 1] |
| 1 | Resistor | 1 kΩ | PTT base[cite: 1] |
| 1 | Resistor | 10 kΩ | PTT base pull-down (optional)[cite: 1] |
| 1 | GPS module | NMEA @ 3.3 V logic (u-blox NEO-6M/7M/8M etc.) | Position source (optional)[cite: 1] |
| 1 | Weather sensor | Serial weather instrument | Weather source (optional)[cite: 1] |
| *2* | *Audio Transformer* | *1:1 600Ω Isolation* | *(Isolated Option Only)* |
| *1* | *Optocoupler* | *PC817, 4N25, or similar* | *(Isolated Option Only)* |
| *1* | *Resistor* | *330 Ω* | *(Isolated Option Only)* |

You'll also need a cable/connector appropriate to your radio (HT 2.5/3.5 mm jacks, or a mobile's 6-pin mini-DIN "DATA" port)[cite: 1]. The onboard **PC13 LED** is used for status, so no external LED is required[cite: 1].

### Pin map

| Signal | STM32 pin | Direction | Notes |
|--------|-----------|-----------|-------|
| RX audio in | **PA0** | analog in (ADC) | AC-coupled, biased to 1.65 V[cite: 1] |
| TX audio out | **PA1** | PWM out | ~281 kHz PWM, RC-filtered to audio[cite: 1] |
| PTT | **PA4** | digital out | drives NPN, HIGH = keyed[cite: 1] |
| Status LED | **PC13** | digital out | onboard LED (active LOW)[cite: 1] |
| GPS in | **PB11** | serial RX | from GPS TX (USART3 or SoftSerial)[cite: 1] |
| GPS out | **PB10** | serial TX | to GPS RX (USART3 or SoftSerial)[cite: 1] |
| Weather in | **PA3** | serial RX | from WX TX (USART2 or SoftSerial)[cite: 1] |
| Weather out | **PA2** | serial TX | to WX RX (USART2 or SoftSerial)[cite: 1] |
| Console | USB (CDC) | serial | 115200 baud, 8N1[cite: 1] |

---

## Connecting to the radio

The three signal paths are: **receive audio in**, **transmit audio out**, and **PTT**[cite: 1]. 

Below are **two options** for wiring your TNC to the radio. Option 1 is simple and relies on a common ground. Option 2 provides complete galvanic isolation to prevent ground loops and RF interference.

### OPTION 1: Direct Connection (Common Ground)
All three lines share a **common ground** with the radio[cite: 1]. Keep leads short[cite: 1].

```text
                               STM32F103C8T6 (Blue Pill)
                              ┌─────────────────────────┐
[RADIO AF/SPKR OUT]           │                         │
       │                      │                         │
 ┌─────┴─────┐                │                         │
GND 10k Pot  │(RX level)      │                         │
 └─────┬─────┘                │                         │
     wiper   100nF            │                         │
       ├──────┤├──────────────┼─────────────────────────┤ PA0 (RX audio, ADC)
       │                      │                         │
3.3V─[10k]─┬─[10k]─GND        │                         │
           │                  │                         │
         [10nF] (optional)    │                         │
           │                  │                         │
          GND                 │                         │
                              │                         │
[RADIO MIC / DATA-IN]         │                         │
       │                      │                         │
 ┌─────┴─────┐                │                         │
GND 10k Pot  │(TX level)      │                         │
 └─────┬─────┘                │                         │
       │     100nF/1uF        │                         │
       ├────────┤├───────┬────┴──[4.7k]───┬───[4.7k]────┤ PA1 (TX audio, PWM)
       │                 │                │             │
      GND              [10nF]           [10nF]          │
                         │                │             │
                        GND              GND            │
                                                        │
[RADIO PTT LINE]              │                         │
       │                      │                         │
       ├───────── C           │                         │
       │ (2N3904)  ╲          │                         │
       │            B ────────┼───[ 1k ]────────────────┤ PA4 (PTT, HIGH=key)
       │           ╱ │        │                         │
       └───────── E  │        │                         │
                 │   └─[10k]──┼─ GND (opt. pull-down)   │
                 │            │                         │
[RADIO GND] ─────┴────────────┼─────────────────────────┤ GND
                              │                         │
                              └─────────────────────────┘
```

#### Option 1 Details:
*   **Receive Audio (PA0):** Radio receive audio is AC-coupled and biased to half the supply (1.65 V) so the 0–3.3 V ADC can capture the full AC swing[cite: 1]. Two 10 kΩ resistors form a divider from 3.3 V to GND[cite: 1]. A 10 kΩ trimmer sets the level[cite: 1]. Aim for roughly 1–1.5 V peak-to-peak at PA0[cite: 1].
*   **Transmit Audio (PA1):** PA1 emits a ~281 kHz PWM whose duty cycle traces the audio waveform[cite: 1]. An RC low-pass filter (two cascaded 4.7 kΩ / 10 nF stages) reconstructs the 1200/2200 Hz tones and removes the PWM carrier[cite: 1]. A 10 kΩ trimmer attenuates it down to microphone level[cite: 1].
*   **PTT (PA4):** A small NPN transistor keys the radio by pulling its PTT line to ground[cite: 1].

---

### OPTION 2: Galvanic-Isolated Connection (Recommended)
This option creates a physical isolation barrier between the STM32 and the radio, protecting the microcontroller from stray RF and preventing audio-corrupting ground loops. **Do not connect the STM32 Ground to the Radio Ground.**

```text
                           ISOLATION BARRIER
                                  │
    STM32 (TNC SIDE)              │                 RADIO SIDE
                                  │
================================= │ ==============================================
 1. RECEIVE AUDIO (RX)            │
================================= │
                                  │
                        100nF     │   1:1 Audio Transformer
[STM32 PA0] ──────┬──────┤├───────┼── ꨄ ꨄ ───────────────< Wiper
(ADC_IN0)         │               │   ꨄ ꨄ                  ┌─────┴─────┐
                  │               │   ꨄ ꨄ                  │  10k Pot  │(RX level)
 3.3V ──[ 10k ]───┤               │   ꨄ ꨄ                  └─────┬─────┘
                  │               │   ꨄ ꨄ                        │
  GND ──[ 10k ]───┤               │   ꨄ ꨄ                        │
                  │               │   ꨄ ꨄ                        │
  GND ───┤├───────┴───────────────┼── ꨄ ꨄ ───────────────────────┴── [RADIO AF OUT]
        10nF      (STM32 GND)     │               (RADIO GND) ───┐
                                  │                              │
================================= │ =============================│================
 2. TRANSMIT AUDIO (TX)           │                              │
================================= │                              │
                                  │                              │
         4.7k      4.7k     1uF   │   1:1 Audio Transformer      │
[STM32 ───███──┬───███──┬────┤├───┼── ꨄ ꨄ ───────────────> Wiper│
 PA1]          │        │         │   ꨄ ꨄ                  ┌─────┴─────┐
               ┴        ┴         │   ꨄ ꨄ                  │  10k Pot  │(TX level)
             10nF     10nF        │   ꨄ ꨄ                  └─────┬─────┘
               ┬        ┬         │   ꨄ ꨄ                        │
               │        │         │   ꨄ ꨄ                        │
  GND ─────────┴────────┴─────────┼── ꨄ ꨄ ───────────────────────┴── [RADIO MIC]
              (STM32 GND)         │               (RADIO GND) ───┐
                                  │                              │
================================= │ =============================│================
 3. PTT (PUSH-TO-TALK)            │                              │
================================= │                              │
                                  │        Optocoupler           │
                                  │      (PC817 / 4N25)          │
         330 ohm                  │      ┌────────────┐          │
[STM32 ───███─────────────────────┼─────>│ 1 (An)   C │──────────┼── [RADIO PTT]
 PA4]                             │      │            │          │
                                  │      │    LED   │/│          │
                                  │      │     ▽    │ │ (NPN)    │
                                  │      │    ───   │↘│          │
  GND ────────────────────────────┼─────>│ 2 (Ca)   E │──────────┴── [RADIO GND]
              (STM32 GND)         │      └────────────┘
                                  │
```

#### Option 2 Details:
*   **Receive Audio (PA0):** The 10k level trimmer is placed on the *Radio* side to prevent oversaturating the transformer coil. Audio jumps the magnetic gap to the STM32 side, where it is DC-biased to 1.65V.
*   **Transmit Audio (PA1):** The RC filter smooths the PWM wave on the STM32 side. The 1uF capacitor blocks DC current from entering the transformer. On the radio side, the 10k pot attenuates the purely isolated signal to mic level.
*   **PTT (PA4):** The STM32 drives the internal LED of the optocoupler. The isolated photo-transistor then shorts the radio's PTT line to the radio's ground. 

---

### GPS & Weather Station Configuration

Both the GPS and Weather Station interfaces can be toggled in the firmware between hardware UART and SoftwareSerial using the `#define GPS_USE_SOFTWARE_SERIAL` and `#define WX_USE_SOFTWARE_SERIAL` flags at the top of the sketch[cite: 1].

- **GPS:** Uses **PB11** (RX) and **PB10** (TX)[cite: 1]. Default NMEA rate is 9600 baud (`GPS BAUD` to change)[cite: 1].
- **Weather:** Uses **PA3** (RX) and **PA2** (TX)[cite: 1].

> **Hardware Serial Requirements:**
> If you set the toggle to `0` to use hardware serial, you must configure the core correctly[cite: 1]:
> 1. **Tools → "U(S)ART support: Enabled (generic 'Serial')"** (not "Disabled")[cite: 1]. If it's Disabled the core omits its `HardwareSerial` class and the linker will fail[cite: 1].

---

## Default Configuration Definitions

At the top of the `.ino` file, you can manually set the default configuration parameters that the board will revert to upon a fresh flash or a manual `FORMAT` command:

```cpp
#define DEF_MYCALL         "NOCALL"
#define DEF_MYSSID         0
#define DEF_BEACON_TEXT    "STM32 TNC"
#define DEF_BEACON_EN      false
#define DEF_GPS_MODE       GPS_OFF    // GPS_OFF, GPS_SERIAL, GPS_STATIC
#define DEF_WX_MODE        WX_OFF     // WX_OFF, WX_MANUAL, WX_SERIAL
// ... (and more)
```

---

## Building and flashing

This firmware targets the **official STMicroelectronics Arduino core**[cite: 1].

1. **Install the STM32 core** in the Arduino IDE[cite: 1]. Add this Boards Manager URL (*File → Preferences*)[cite: 1]:
   ```text
   [https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json](https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json)
   ```
   then install **"STM32 MCU based boards"** in *Tools → Board → Boards Manager*[cite: 1].

2. **Board settings** (*Tools* menu)[cite: 1]:
   - Board: **Generic STM32F1 series**[cite: 1]
   - Board part number: **BluePill F103C8** (or **F103C8 (128K)** if your chip has 128 KB)[cite: 1]
   - U(S)ART support: **Enabled (generic 'Serial')**[cite: 1]
   - USB support: **CDC (generic Serial supersede U(S)ART)** — needed for the console over USB[cite: 1]
   - Upload method: **STM32CubeProgrammer (SWD)** with an ST-Link (recommended)[cite: 1]

3. **Flash** `stm32-AFSK_TNC.ino` with an **ST-Link V2** on the SWD header (3V3, GND, SWDIO, SWCLK)[cite: 1].

4. **Open the Serial Monitor** at **115200 baud** and you should see the `cmd:` prompt[cite: 1].

---

## Usage

Out of reset the TNC is in **command mode** (`cmd:` prompt)[cite: 1]. Type `HELP` for the list[cite: 1]. There are three modes[cite: 1]:

- **Command** — type commands[cite: 1].
- **Converse** — everything you type is transmitted (as an unproto UI frame, or over the connected link if one is up)[cite: 1]. Press **Ctrl-C** to return to command mode[cite: 1].
- **KISS** — the TNC becomes a modem for host software; internal beacon/digi are disabled[cite: 1].

### Command reference

| Command | Alias | Description |
|---------|-------|-------------|
| `MYCALL <call>` | | Set your source callsign |
| `MYSSID <0-15>` | | Set your source SSID |
| `PATH <a,b,..>` / `PATH OFF` | | TX digipeat path, e.g. `WIDE1-1,WIDE2-1` |
| `CONNECT <call[-ssid]>` | `C` | Open an AX.25 connected session |
| `DISCONNECT` | `D`, `BYE` | Close the link |
| `STATUS` | `CS` | Show link state and V(S)/V(R)/V(A) |
| `CONV` | | Enter converse mode |
| `FRACK [sec]` | | Retransmit timeout T1 (default 4 s) |
| `RETRY [n]` | | Max retries N2 before link fails (default 10) |
| `PACLEN [n]` | | Max info bytes per I-frame, 1–255 (default 128) |
| `GPS OFF`, `GPS SERIAL`, `GPS STATIC` | | Set GPS mode |
| `GPS SET <lat><hemi> <lon><hemi>` | | Set static GPS position |
| `GPS BAUD <n>` | | GPS serial speed, 1200–115200 (default 9600) |
| `GPS`, `GPS STATUS` | | Show GPS status and current location |
| `SYMBOL <table><code>` | | APRS symbol, e.g. `SYMBOL /-` (house), `SYMBOL />` (car) |
| `WX OFF`, `WX MANUAL`, `WX SERIAL` | | Set Weather station mode |
| `WX SET <data>` | | Set manual telemetry (e.g. `270/010g015...`) |
| `MHEARD` | `MH` | List recently heard stations |
| `MONITOR ON` / `MONITOR OFF` | | Show/hide heard UI frames |
| `BTEXT <text>` | | Set beacon payload text |
| `BEACON` | | Send one beacon now |
| `BEACON EVERY <sec>` | | Auto-beacon interval (0 = off) |
| `BEACON OFF` | | Stop auto-beacon |
| `DIGI ON` / `DIGI OFF` | | Enable/disable digipeater |
| `DIGI FILL` / `DIGI WIDE` | | Fill-in (WIDE1-1 only) or full WIDEn-N |
| `WIDEMAX <1-7>` | | Max N handled in WIDE mode (default 2) |
| `MYALIAS <call>` / `MYALIAS OFF` | | Extra exact-match digi alias |
| `SAVE` | | Save settings safely to direct Flash Memory |
| `CONFIG`, `SHOW` | | Show currently saved settings |
| `FORMAT` | | Erase flash & reload `#define` Defaults |
| `KISS ON` | | Enter KISS mode |
| `HELP` | | Command list |

### Example: Weather Station setup
You can input weather telemetry directly into the beacon payload via the terminal, or stream it automatically using serial[cite: 1].
```text
cmd: WX MANUAL
Weather mode: MANUAL.
cmd: WX SET 270/010g015t072r000p000P000h50b10150
Weather data set to: 270/010g015t072r000p000P000h50b10150
cmd: BEACON
```
*If a GPS fix is acquired, the weather payload will automatically append to the coordinates using the `_` symbol identifier[cite: 1].*

### Example: APRS beacon

```text
cmd: MYCALL N0CALL
cmd: PATH WIDE1-1,WIDE2-1
cmd: BTEXT !4903.50N/07201.75W-STM32 TNC
cmd: BEACON EVERY 600
Auto-beacon every 600s.
```

### Status LED (PC13)

| Pattern | Meaning |
|---------|---------|
| Steady ON | Idle / ready[cite: 1] |
| Blink ~100 ms | Serial or on-air traffic[cite: 1] |
| Blink ~500 ms | Connected (AX.25 link up, or KISS attached)[cite: 1] |

---

## Calibration

AFSK is analog-sensitive; two quick adjustments make or break it[cite: 1].

- **TX deviation (audio level trimmer):** set `BEACON EVERY 5` and monitor your signal on a second radio or, ideally, a deviation meter[cite: 1]. Start with the trimmer low and bring it up until the tones are clearly present without sounding distorted — aim for about **3–3.5 kHz deviation** on narrow FM[cite: 1]. Too hot splatters and won't decode; too low is weak[cite: 1]. Then `BEACON OFF`[cite: 1].
- **RX level trimmer:** feed in received packets and adjust for reliable decoding[cite: 1]. If strong local signals fail but weak ones work, you're clipping — turn it down[cite: 1].

A good first test is to **transmit into a dummy load** and decode your own beacons with [Dire Wolf](https://github.com/wb2osz/direwolf) on a computer's sound card, before going on the air[cite: 1].

---

## How it works (brief)

- **TX:** an AX.25 frame is HDLC-framed (flags, bit-stuffing, CRC-16/X.25 FCS), NRZI-encoded, and clocked out at 1200 baud by a phase-accumulator DDS driving PWM on PA1 at the sample rate (9600 Hz)[cite: 1]. An RC filter turns the PWM into clean audio[cite: 1].
- **RX:** PA0 is sampled at 9600 Hz[cite: 1]. A delay-line multiply frequency discriminator plus a digital PLL bit-sampler recover the bitstream (the classic MicroModem/BeRTOS approach), which is then NRZI-decoded, bit-destuffed, and HDLC-assembled with FCS checking[cite: 1].
- **Link layer:** a mod-8 AX.25 v2.2 state machine handles the connected-mode session[cite: 1].

---

## License

GPL-3[cite: 1]