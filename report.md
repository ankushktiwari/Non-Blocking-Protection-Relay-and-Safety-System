# Non-Blocking Protection Relay and Safety System: Project Report

## 1. Objective
Design a protection relay for an ESP32 (simulated in Wokwi) that:
- uses no blocking delays, only timers and a state machine;
- avoids relay chatter with hysteresis and debounce timing;
- detects frozen readings, unrealistic jumps and out-of-range values;
- separates genuine rapid load changes from sensor noise using window-based statistics;
- recovers from a fault through a timed sequence, logging every transition;
- stays stable under continuous fault injection, with no resets or missed transitions.

## 2. Hardware (Wokwi)
| Part | Pin | Purpose |
|---|---|---|
| ESP32 DevKit | n/a | Controller |
| Potentiometer | GPIO 34 | Load current sensor, 0 to 10 A |
| I2C LCD 16x2 | SDA 21, SCL 22 | Status display |
| Red / green / yellow LED | GPIO 2 / 4 / 5 | Trip / normal / recovery |
| Buzzer | GPIO 18 | Audible alarm |
| Relay (active LOW) | GPIO 19 | Load switch |
| 3 pushbuttons | GPIO 25 / 26 / 27 | Inject fault / reset lockout / storm mode |

## 3. Non-blocking architecture
`loop()` never waits. It checks `millis()` against per-task deadlines and runs whatever is due:

| Task | Period | Work |
|---|---|---|
| Sample and protect | 20 ms (50 Hz) | Read, validate, compare, run the state machine, update storm injection |
| Buttons | every pass | Debounced with a 30 ms timestamp, no waiting |
| LEDs and buzzer | every pass | Blink patterns computed from `millis()` |
| LCD | 250 ms | Two lines, padded to 16 characters (no `clear()` flicker) |
| Status line | 5 s | Counters and loop health |

Time comparisons use unsigned subtraction, so they survive the 49-day `millis()` wrap.
If the loop is ever delayed, the sample timer re-syncs instead of running a burst of
catch-up samples. The loop also measures its own period: `maxLoop` and `stalls` (any
gap over 50 ms) appear in the status line, so a blocking call would be visible.

Two single points of control keep the system consistent:
- `setRelay()` is the only code that drives the relay pin. It logs every change.
- `setState()` is the only code that changes the state. It logs every change and
  counts it, so a transition cannot happen without a log line.

## 4. Sensor validation pipeline
Each 20 ms sample passes through these stages in order.

1. **Range check.** A reading outside -0.5 to 10.5 A is an out-of-range anomaly and is
   not stored.
2. **Window.** Valid samples go into a 30-sample ring buffer (0.6 s).
3. **Jump check.** A change of more than 3.5 A from the last accepted value is not
   accepted immediately. It becomes a candidate level. If the next samples stay within
   1 A of that level for 4 samples in total (80 ms), it is a genuine load step and is
   accepted and logged. If they do not, it was a spike and is rejected. The last good
   value is held meanwhile, so a glitch never reaches the protection logic.
4. **Frozen check.** If the max minus min over the full window is below 0.005 A, the
   sensor is stuck. A working sensor always has some noise, so this never triggers on a
   healthy one.
5. **Noise check.** Over the last 20 samples, noise needs both a standard deviation
   above 0.6 A and at least 5 direction reversals (consecutive steps over 0.2 A in
   opposite directions).

**Why the noise test uses two measures.** A genuine load step has a large spread too,
but it moves one way and holds, so it produces about one reversal. Noise and rapid
oscillation produce many. Spread alone would flag real load changes as noise, and
reversals alone would miss slow, large oscillations.

### Sensor health score (debounce for anomalies)
A leaky counter turns individual anomalies into one debounced decision:

| Event | Effect |
|---|---|
| Anomalous sample | +4 (capped at 20) |
| Good sample | -1 (floored at 0) |
| Score reaches 10 | Sensor fault asserted |
| Score at 2 or below for 500 ms | Sensor fault cleared |

An isolated spike adds 4 and decays within a few samples, so it is logged but does not
trip the relay. Repeated or persistent anomalies drive the score up and trip it. The
two thresholds (10 to assert, 2 to clear) plus the hold time form hysteresis for the
sensor fault itself.

## 5. Anti-chatter protection
The overcurrent comparator uses both techniques:

| Mechanism | Value | Effect |
|---|---|---|
| Hysteresis | Trip above 8.0 A, clear below 7.0 A | A reading between 7 and 8 A changes nothing |
| Trip debounce | 200 ms continuous | Brief excursions do not trip |
| Clear debounce | 1000 ms continuous | A momentary dip does not release the trip |
| Cooldown | 3 s with the relay open | Prevents rapid re-closing |
| Retry limit | 3 failed re-closes, then LOCKOUT | Prevents endless trip-and-close cycling |

Because the trip and clear limits differ, the comparator picks one threshold depending
on its current state, and a value hovering near 8 A cannot make it flip repeatedly.

## 6. Protection state machine
| State | Relay | LED | Exit condition |
|---|---|---|---|
| NORMAL | closed | Green | Fault confirmed leads to TRIPPED |
| TRIPPED | open | Red, pulsing buzzer | Fault cleared (after debounce) leads to COOLDOWN |
| COOLDOWN | open | Yellow, slow blink | 3 s elapsed leads to PROBATION. Fault returns leads to TRIPPED |
| PROBATION | closed | Yellow, fast blink | 3 s clean leads to NORMAL. Fault leads to TRIPPED, or LOCKOUT at the 3rd failure |
| LOCKOUT | open | Red, fast blink, fast buzzer | Reset button leads to COOLDOWN |

A fault is a latched overcurrent or a confirmed sensor fault. A sensor fault opens the
relay (fail-safe), because an untrusted measurement cannot be allowed to keep the load on.
After 10 s in NORMAL the retry counter is cleared.

### Timed recovery sequence
After a fault clears the relay does not simply switch back on:
1. **Fault-clear debounce:** the cause must stay gone for its clear time.
2. **COOLDOWN (3 s):** the relay stays open. If the fault returns, the sequence restarts.
3. **PROBATION (3 s):** the relay closes, but the system is on trial. Any fault reopens it.
4. **NORMAL:** only after the probation passes cleanly.

## 7. Event logging
Every state change, relay operation, anomaly, comparator change and injection is printed
with a millisecond timestamp:
```
[    8.240] RELAY    CLOSED (load connected)
[    8.240] STATE    COOLDOWN -> PROBATION | cooldown done, relay re-closed | I=-0.02A
[    8.460] PROTECT  OVERCURRENT asserted: 9.01A > 8.0A for 200ms
[    8.460] RELAY    OPEN (load cut off)
[    8.460] STATE    PROBATION -> TRIPPED | fault during probation | I=9.01A
```
Anomaly messages are rate-limited to one per kind per second so a sustained fault does
not flood the serial port, while the counters still record every one. The periodic
`STAT` line reports the state, current, sample count, transitions, relay operations,
anomalies, retries, `maxLoop` and `stalls`.

## 8. Fault injection
The sketch simulates sensor failures between the ADC and the validator:

| Mode | What it does |
|---|---|
| FREEZE | Holds the last reading |
| SPIKE | Adds or subtracts 5 A for 2 samples, repeatedly |
| RANGE | Returns an impossible 14 A |
| NOISE | Adds up to +/-1.5 A random noise |
| STEP | Adds 4.5 A (a real load change, not a fault) |
| Storm | Picks random modes and durations (0.3 to 2.5 s on, 0.2 to 1.5 s off) forever |

When the relay is open the simulated load current is 0 A, so reclosing onto a persistent
overload fails again, as it would on real equipment. That is what exercises the retry
and lockout logic.

## 9. Test results
These tests ran on a PC with stubbed Arduino functions and simulated time. They did not
run on Wokwi or real hardware.

| Test | Result |
|---|---|
| 5 min at 5 A, then 5 min at 7.5 A (inside the hysteresis band) | No events, no false trips |
| Genuine steps 2 to 6 A and back | Accepted and logged, no trips |
| Pot alternating 7.9 A / 8.1 A every 100 ms | No chatter, no trip |
| Overload at 9 A | Trip at 200 ms, 3 failed re-closes, LOCKOUT |
| Lower the load and reset | COOLDOWN, PROBATION, NORMAL |
| Each injection mode in turn | FREEZE, RANGE, NOISE trip as sensor fault. SPIKE is rejected. STEP accepted |
| 30 minutes of random injection with periodic resets | 947 transitions, 0 illegal, logged count matched the firmware counter |

Legal transitions were checked against the table in section 6. Over the storm test the
system went through 19 lockouts and resets without a crash. Loop-stall figures from the
PC test are not meaningful, since time was simulated. On Wokwi, read `stalls` in the
`STAT` line.

## 10. Parameters
All tuning values are constants at the top of the sketch.

| Group | Constants |
|---|---|
| Timing | `SAMPLE_MS`, `LCD_MS`, `STAT_MS` |
| Validation | `LIM_MIN/MAX`, `WIN`, `FROZEN_EPS`, `NOISE_N`, `NOISE_STD`, `NOISE_REV`, `JUMP_LIMIT`, `JUMP_TOL`, `JUMP_CONFIRM` |
| Health score | `SCORE_ADD/MAX/TRIP/CLEAR`, `SENSOR_OK_HOLD_MS` |
| Protection | `OC_TRIP`, `OC_CLEAR`, `OC_TRIP_MS`, `OC_CLEAR_MS` |
| Recovery | `COOLDOWN_MS`, `PROBATION_MS`, `MAX_RETRIES`, `STABLE_RESET_MS` |

## 11. Limitations and future work
- One simulated current channel; real systems cross-check redundant sensors.
- The overcurrent curve is a fixed threshold. An inverse-time curve (I squared t) would
  tolerate short inrush currents better.
- The simulated ADC noise is needed for frozen detection. A real sensor supplies its own.
- Timings are demo values and need tuning for real loads.
- Add a hardware watchdog timer, non-volatile storage of the lockout state across
  power cycles, and persistent logging.

## 12. Conclusion
The system meets the brief. It runs with no blocking calls, switches the relay without
chatter, detects four kinds of sensor anomaly, separates real load steps from glitches
using window statistics, recovers through a timed cooldown and probation, and logs each
transition. In simulation it stayed consistent through 30 minutes of random fault
injection.
