# Non-Blocking Protection Relay and Safety System (ESP32 + Wokwi)

A fully non-blocking protection relay for ESP32. There is no `delay()` anywhere:
all timing uses `millis()` timers and a state machine. It detects bad sensor data
(frozen, out-of-range, spikes, noise), tells a genuine load step apart from a sensor
glitch, prevents relay chatter, and recovers from faults with a timed sequence.
Every state change is logged.

▶ **Run it live:** <https://wokwi.com/projects/476951109148680193>
🎥 **Demo video:** 

## Features
- Non-blocking: `millis()` scheduler (50 Hz protection loop) and one state machine
- Anti-chatter: hysteresis (trip above 8.0 A, clear below 7.0 A) plus debounce
  (200 ms to trip, 1000 ms to clear) and a minimum cooldown before re-closing
- Sensor anomaly detection: out-of-range, frozen reading, unrealistic jump, noise
- Genuine load change vs glitch: a jump is only accepted if the new level holds for
  4 samples; noise needs both a large spread and many direction reversals
- Fail-safe: a confirmed sensor fault opens the relay
- Timed recovery: `TRIPPED -> COOLDOWN -> PROBATION -> NORMAL`, with `LOCKOUT` after
  3 failed re-closes
- Every transition is logged with a timestamp, the reason and the current reading
- Built-in fault injection (manual and random "storm" mode) to prove stability

## Files
| File | Purpose |
|---|---|
| `sketch.ino` | Firmware |
| `diagram.json` | Wokwi circuit |
| `libraries.txt` | Wokwi library list (`LiquidCrystal I2C`) |
| `REPORT.md` | Design report |

## Hardware (Wokwi)
| Part | Pin | Role |
|---|---|---|
| Potentiometer | GPIO 34 | Load current sensor, 0 to 10 A |
| I2C LCD 16x2 | SDA 21, SCL 22 | State, current, injected fault |
| Green LED | GPIO 4 | NORMAL |
| Yellow LED | GPIO 5 | Recovery (slow blink = COOLDOWN, fast = PROBATION) |
| Red LED | GPIO 2 | TRIPPED (solid) or LOCKOUT (fast blink) |
| Buzzer | GPIO 18 | Pulses when tripped, faster in lockout |
| Relay (active LOW) | GPIO 19 | LOW = load connected, HIGH = cut off |
| Button: inject | GPIO 25 | Cycles NONE, FREEZE, SPIKE, RANGE, NOISE, STEP |
| Button: reset | GPIO 26 | Clears LOCKOUT |
| Button: storm | GPIO 27 | Toggles continuous random fault injection |

## How to run
1. Open the Wokwi link, or create an ESP32 project and paste in `sketch.ino`,
   `diagram.json` and `libraries.txt`.
2. Press Play and open the Serial Monitor (115200 baud) to watch the event log.

## How it works

### Protection state machine
| State | Relay | Meaning | Leaves when |
|---|---|---|---|
| NORMAL | closed | Healthy | A fault is confirmed |
| TRIPPED | open | Fault active | The fault has cleared (after its debounce) |
| COOLDOWN | open | 3 s anti-short-cycle wait | Timer ends, or the fault returns |
| PROBATION | closed | 3 s watched re-close | Passes, or fails (retry counter +1) |
| LOCKOUT | open | 3 failed re-closes | Reset button pressed |

A fault is either a latched overcurrent or a confirmed sensor fault. All state
changes go through one `setState()` function, so none can be missed or unlogged.

### Sensor validation (per 20 ms sample)
| Anomaly | How it is detected |
|---|---|
| RANGE | Reading outside -0.5 to 10.5 A |
| FROZEN | Max minus min over the last 30 samples is under 0.005 A |
| JUMP | Change over 3.5 A that does not hold within 1 A for 4 samples (a spike) |
| NOISE | Std-dev over 20 samples above 0.6 A and 5 or more direction reversals |

A real load step is large but moves one way and holds, so it is accepted (and logged).
A spike does not hold, and noise reverses direction constantly, so both are rejected.
A leaky health score turns individual anomalies into a debounced sensor fault:
+4 per anomaly, -1 per good sample, fault at 10, cleared at 2 or below for 500 ms.
Isolated glitches are logged but do not trip the relay; repeated ones do.

The sketch adds about 0.03 A of simulated ADC noise to the reading. A real sensor
always has some noise, and its absence is what makes a stuck value detectable.

### Example event log
```
[    4.220] PROTECT  OVERCURRENT asserted: 8.97A > 8.0A for 200ms
[    4.220] RELAY    OPEN (load cut off)
[    4.220] STATE    NORMAL -> TRIPPED | overcurrent | I=8.97A
[    5.240] PROTECT  overcurrent cleared: -0.02A < 7.0A for 1000ms
[    5.240] STATE    TRIPPED -> COOLDOWN | fault cleared, cooldown started | I=-0.02A
[    8.240] RELAY    CLOSED (load connected)
[    8.240] STATE    COOLDOWN -> PROBATION | cooldown done, relay re-closed | I=-0.02A
[    8.460] STATE    PROBATION -> TRIPPED | fault during probation | I=9.01A
```
(When the relay opens, the simulated current drops to 0, as a real load would.)

## Demo script
1. Set the pot to about 5 A: green LED, NORMAL.
2. Raise it above 8 A: after 200 ms the relay trips, then the cooldown and probation
   run. If the pot is still high the re-close fails again, and after 3 failures the
   system enters LOCKOUT. Lower the pot and press Reset to recover.
3. Press the inject button to step through FREEZE, SPIKE, RANGE, NOISE and STEP.
   FREEZE, RANGE and NOISE trip the relay as a sensor fault. A single SPIKE is only
   logged. STEP adds 4.5 A, which is accepted as a genuine load change.
4. Press the storm button for continuous random fault injection. The state machine
   keeps running and every transition appears in the log.

## Stability testing
The logic was run on a PC with stubbed Arduino functions and simulated time:
- 5 minutes at 5 A, then 5 minutes at 7.5 A (inside the hysteresis band): no events
- Genuine steps of 2 to 6 A and back: accepted, no trips
- Pot alternating between 7.9 A and 8.1 A: no chatter
- 30 minutes of random fault injection with periodic resets: 947 transitions, all legal,
  and the logged count matched the firmware's transition counter

This was not run on Wokwi or real hardware. The `STAT` line printed every 5 s shows
`maxLoop` and `stalls`, so you can confirm in Wokwi that the loop never blocks.

## Limitations
- The sensor is a single simulated current channel
- Overcurrent protection is a fixed-threshold curve, not inverse-time
- Retry counts and timings are demo values, not tuned for real equipment
- Real hardware would need a hardware watchdog and a proper current sensor

