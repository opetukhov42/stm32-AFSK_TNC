# STM32 Blue Pill AFSK 1200 TNC

A self-contained **1200-baud packet-radio TNC** (Terminal Node Controller) for the
STM32F103C8T6 "Blue Pill". It implements the complete AFSK modem and AX.25 stack in
firmware — **no external DSP or APRS library required**. Everything (Bell-202
modulation/demodulation, HDLC framing, bit-stuffing, NRZI, CRC, and the AX.25 v2.2
link layer) runs on the bare microcontroller using one PWM output for transmit audio
and the ADC for receive audio.

> ⚠️ **Amateur radio license required to transmit.** Set `MYCALL` to *your own*
> callsign, never transmit as `NOCALL`, and bench-test into a dummy load first.
> You are responsible for your emissions.

---

## Features

- **AFSK 1200 (Bell 202)** modem — 1200 Hz / 2200 Hz, NRZI, HDLC, CRC-16/X.25 FCS
- **KISS mode** — works as a dumb TNC for Dire Wolf, APRSISCE/32, Xastir, UI-View, YAAC, etc.
- **Command / Converse terminal** — TAPR TNC-2 style command line over USB serial
- **AX.25 v2.2 connected mode** — SABM/UA/DISC/DM handshakes, I-frames, RR/RNR/REJ,
  go-back-N retransmission, T1/T3 timers, N2 retry cap, path reversal for incoming connects
- **UI / unproto** transmit for beacons and APRS-style messages
- **Beacon** — one-shot or automatic, with configurable text and path
- **Digipeater** — proper WIDEn-N *New Paradigm* (fill-in and wide modes) with callsign
  insertion and a 30 s duplicate-suppression cache
- **MHEARD** list and a **monitor** toggle
- **Status LED** — steady = idle, fast blink = traffic, slow blink = connected
- **Live-tunable** link parameters (`FRACK`, `RETRY`, `PACLEN`)

---

## Hardware

### Bill of materials

| Qty | Part | Value / Type | Purpose |
|----:|------|--------------|---------|
| 1 | MCU board | STM32F103C8T6 "Blue Pill" | The TNC |
| 1 | Programmer | ST-Link V2 (clone is fine) | Flashing (easiest method) |
| 2 | Resistor | 10 kΩ | RX bias divider (mid-rail) |
| 1 | Trimmer pot | 10 kΩ | RX audio level |
| 1 | Capacitor | 100 nF (0.1 µF) | RX AC coupling |
| 1 | Capacitor | 10 nF | RX anti-alias (optional) |
| 2 | Resistor | 4.7 kΩ | TX low-pass filter |
| 2 | Capacitor | 10 nF | TX low-pass filter |
| 1 | Capacitor | 1 µF (or 100 nF) | TX AC coupling |
| 1 | Trimmer pot | 10 kΩ | TX level / deviation |
| 1 | Transistor | NPN, 2N3904 / BC547 / 2N2222 | PTT switch |
| 1 | Resistor | 1 kΩ | PTT base |
| 1 | Resistor | 10 kΩ | PTT base pull-down (optional) |

You'll also need a cable/connector appropriate to your radio (HT 2.5/3.5 mm jacks, or a
mobile's 6-pin mini-DIN "DATA" port). The onboard **PC13 LED** is used for status, so no
external LED is required.

### Pin map

| Signal | STM32 pin | Direction | Notes |
|--------|-----------|-----------|-------|
| RX audio in | **PA0** | analog in (ADC) | AC-coupled, biased to 1.65 V |
| TX audio out | **PA1** | PWM out | ~281 kHz PWM, RC-filtered to audio |
| PTT | **PA2** | digital out | drives NPN, HIGH = keyed |
| Status LED | **PC13** | digital out | onboard LED (active LOW) |
| Console | USB (CDC) | serial | 115200 baud, 8N1 |

---

## Connecting to the radio

The three signal paths are: **receive audio in**, **transmit audio out**, and **PTT**.
All three share a **common ground** with the radio. Keep leads short.

### Full wiring diagram

```
                         STM32F103C8T6 (Blue Pill)
                        ┌───────────────────────────┐
                        │                           │
   RADIO AF/SPKR OUT    │                           │
        ────────┐       │                           │
                │       │                           │
        ┌───[ 10k trimmer ]───┐  (RX level)         │
        │                     │                     │
       GND                  wiper                    │
                              │                      │
                              │   100nF              │
                              ├────┤├──────┬─────────┤ PA0  (RX audio, ADC)
                              │           │          │
                   3.3V ─[10k]┴[10k]─ GND │          │      bias tap = 1.65 V
                              (bias)      │          │
                                     [10nF] (opt.)   │
                                          │          │
                                         GND         │
                                                     │
                                                     │
                      TX LOW-PASS FILTER             │
        RADIO MIC / DATA-IN                          │
             ▲                                       │
             │      100nF/1uF                        │
       [ 10k trimmer ]                               │
             ▲   (TX level / deviation)              │
             │                                       │
             ├──┤├──┬──[4.7k]──┬──[4.7k]──┬──────────┤ PA1  (TX audio, PWM)
             │      │          │          │          │
            GND   [10nF]     [10nF]      (from PA1)   │
                    │          │                      │
                   GND        GND                     │
                                                     │
                      PTT SWITCH                      │
        RADIO PTT line                                │
             │                                        │
             ├───────────── C (collector)             │
             │             ╱                          │
          (2N3904)  B ──[1k]──────────────────────────┤ PA2  (PTT, HIGH = key)
             │             ╲                          │
             │              E (emitter)               │
        ─────┴──────┬───────┘                         │
                   GND ─── [10k] ─── B  (optional)    │
                        │                             │
   RADIO GND ───────────┴─── COMMON GND ──────────────┤ GND
                                                     │
                        └───────────────────────────┘

        Blue Pill powered from USB (or 5V/3.3V). Radio powered separately.
        The COMMON GND tie between radio and Blue Pill is essential.
```

### 1. Receive audio → PA0

Radio receive audio is **AC-coupled** and **biased to half the supply (1.65 V)** so the
0–3.3 V ADC can capture the full AC swing.

- Two **10 kΩ** resistors form a divider from 3.3 V to GND; the tap sits at **1.65 V** and
  connects to **PA0**.
- The radio audio is injected onto that tap through a **100 nF** series capacitor. With the
  divider's ~5 kΩ source impedance this gives a high-pass corner near 320 Hz — well below
  the 1200/2200 Hz tones.
- A **10 kΩ trimmer** in front sets the level. Aim for roughly **1–1.5 V peak-to-peak** at
  PA0. Too much clips the ADC and kills decoding; too little buries the signal in noise.
- The optional **10 nF** cap from PA0 to GND is a light anti-alias/noise filter.

**Where to take the audio:** a fixed-level **"data out" / discriminator** pin (on a
mobile's data port) is ideal because its level doesn't change with the volume knob. If you
use the **speaker/headphone jack**, set a moderate volume and leave it there — and re-tune
the RX trimmer if you change it.

### 2. Transmit audio → radio mic

PA1 emits a ~281 kHz PWM whose duty cycle traces the audio waveform. An **RC low-pass
filter** reconstructs the 1200/2200 Hz tones and removes the PWM carrier.

- Two cascaded **4.7 kΩ / 10 nF** stages give a corner near **3.4 kHz** — passes 2200 Hz,
  strongly attenuates the carrier. One stage works; two is cleaner.
- A **100 nF (or 1 µF)** capacitor AC-couples the result.
- A **10 kΩ trimmer** attenuates it down to microphone level (typically only tens of
  millivolts). This trimmer is your **deviation control** — see *Calibration* below.

**Mobile data ports** often have a dedicated audio input expecting near line level; check
your radio manual, as you may need less attenuation there than on a mic jack.

### 3. PTT → radio

A small **NPN transistor** keys the radio by pulling its PTT line to ground (how most
radios and data ports key).

- **PA2 → 1 kΩ → base.** Emitter to GND. Collector to the radio's PTT line.
- When PA2 goes HIGH the transistor saturates and grounds PTT.
- The optional **10 kΩ** base-to-emitter resistor ensures a clean turn-off.
- On many HTs, PTT is keyed by a resistor to ground **on the mic line** rather than a
  separate pin — consult your radio's data-cable pinout and adapt accordingly.

### Grounding & isolation

Use one solid common ground and short leads. For a permanent or high-RF-environment
install, consider **audio isolation transformers** (600:600 Ω) on the TX and RX audio and
an **optocoupler** on PTT to break ground loops (hum) and keep RF out of the MCU. This is
optional for bench testing.

---

## Building and flashing

This firmware targets the **official STMicroelectronics Arduino core**.

1. **Install the STM32 core** in the Arduino IDE. Add this Boards Manager URL
   (*File → Preferences*):
   ```
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```
   then install **"STM32 MCU based boards"** in *Tools → Board → Boards Manager*.

2. **Board settings** (*Tools* menu):
   - Board: **Generic STM32F1 series**
   - Board part number: **BluePill F103C8** (or **F103C8 (128K)** if your chip has 128 KB)
   - U(S)ART support: **Enabled**
   - USB support: **CDC (generic Serial supersede U(S)ART)** — needed for the console over USB
   - Upload method: **STM32CubeProgrammer (SWD)** with an ST-Link (recommended)

3. **Flash** `stm32-AFSK_TNC.ino` with an **ST-Link V2** on the SWD header (3V3, GND,
   SWDIO, SWCLK). Serial-bootloader (BOOT0) or DFU upload also work if you prefer.

4. **Open the Serial Monitor** at **115200 baud** and you should see the `cmd:` prompt.

---

## Usage

Out of reset the TNC is in **command mode** (`cmd:` prompt). Type `HELP` for the list.
There are three modes:

- **Command** — type commands.
- **Converse** — everything you type is transmitted (as an unproto UI frame, or over the
  connected link if one is up). Press **Ctrl-C** to return to command mode.
- **KISS** — the TNC becomes a modem for host software; internal beacon/digi are disabled.

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
| `KISS ON` | | Enter KISS mode |
| `HELP` | | Command list |

Typing `FRACK`, `RETRY`, or `PACLEN` with no argument prints the current value.
Settings live in RAM and reset to defaults on reboot.

### Example: a connected-mode QSO

```
cmd: MYCALL N0CALL
MYCALL updated to: N0CALL
cmd: CONNECT W1ABC
*** Connecting to W1ABC ...
*** CONNECTED to W1ABC
(connected - type to send, Ctrl+C for command mode)
Hello, this goes out as an I-frame.
...replies from W1ABC appear here...
<Ctrl-C>
cmd: DISCONNECT
*** Disconnecting ...
*** DISCONNECTED
```

To connect *through* digipeaters, set `PATH WIDE1-1` (or the specific digi calls) before
`CONNECT`. When another station connects to **you**, the TNC automatically replies using
the reverse of their path.

### Example: APRS beacon

```
cmd: MYCALL N0CALL
cmd: PATH WIDE1-1,WIDE2-1
cmd: BTEXT !4903.50N/07201.75W-STM32 TNC
cmd: BEACON EVERY 600
Auto-beacon every 600s.
```

### Example: fill-in digipeater

```
cmd: MYCALL N0CALL
cmd: DIGI ON
cmd: DIGI FILL        (home fill-in: repeats WIDE1-1 only)
```
For a wide-area digi use `DIGI WIDE` and set `WIDEMAX` to a sane local value (2 is plenty
almost everywhere). Please understand your local digi plan before running a wide digi.

### KISS mode with host software

`KISS ON`, then point your application at the serial port at **115200 baud**. Verified
conceptually against the standard KISS framing used by Dire Wolf, APRSISCE/32, Xastir,
UI-View, YAAC, and similar. In KISS mode the TNC passes raw frames both ways and leaves
beaconing/digipeating/connected-mode to the host. Send a KISS "Return" (or reset the board)
to leave KISS mode.

### Status LED (PC13)

| Pattern | Meaning |
|---------|---------|
| Steady ON | Idle / ready |
| Blink ~100 ms | Serial or on-air traffic |
| Blink ~500 ms | Connected (AX.25 link up, or KISS attached) |

---

## Calibration

AFSK is analog-sensitive; two quick adjustments make or break it.

- **TX deviation (audio level trimmer):** set `BEACON EVERY 5` and monitor your signal on a
  second radio or, ideally, a deviation meter. Start with the trimmer low and bring it up
  until the tones are clearly present without sounding distorted — aim for about **3–3.5 kHz
  deviation** on narrow FM. Too hot splatters and won't decode; too low is weak. Then
  `BEACON OFF`.
- **RX level trimmer:** feed in received packets and adjust for reliable decoding. If strong
  local signals fail but weak ones work, you're clipping — turn it down.

A good first test is to **transmit into a dummy load** and decode your own beacons with
[Dire Wolf](https://github.com/wb2osz/direwolf) on a computer's sound card, before going
on the air.

---

## How it works (brief)

- **TX:** an AX.25 frame is HDLC-framed (flags, bit-stuffing, CRC-16/X.25 FCS), NRZI-encoded,
  and clocked out at 1200 baud by a phase-accumulator DDS driving PWM on PA1 at the sample
  rate (9600 Hz). An RC filter turns the PWM into clean audio.
- **RX:** PA0 is sampled at 9600 Hz. A delay-line multiply frequency discriminator plus a
  digital PLL bit-sampler recover the bitstream (the classic MicroModem/BeRTOS approach),
  which is then NRZI-decoded, bit-destuffed, and HDLC-assembled with FCS checking.
- **Link layer:** a mod-8 AX.25 v2.2 state machine handles the connected-mode session.

---

## Limitations

- One connected session at a time; **mod-8 only** (no SABME / mod-128).
- Connected mode runs in the terminal (not in KISS — the host owns L2 there).
- Configuration is **not persisted** across reboots (lives in RAM).
- The digipeater is call/alias + WIDEn-N; it does **not** implement `SSn-N`, `TRACEn-N`,
  or preemptive digipeating.
- RX performance depends on your analog front end and levels. For lowest jitter, the ADC can
  be driven directly from the timer (read `ADC1->DR`) instead of `analogRead()` — see the
  note in `sampleISR()`.

---

## Credits

- The AFSK demodulator (delay-line discriminator + digital PLL) is based on the well-known
  design from **MicroModem** by Mark Qvist and the **BeRTOS** AFSK modem.
- Framing and link layer follow the **AX.25 v2.2** specification (TAPR/ARRL).
- Digipeating follows the APRS **New N-Paradigm**.

## License

Suggested: **MIT** (add a `LICENSE` file). If you reuse code derived from GPL-licensed
projects such as MicroModem, review and honor their license terms before publishing.

---

*Author: Your Name / CALLSIGN — replace this line before publishing.*
