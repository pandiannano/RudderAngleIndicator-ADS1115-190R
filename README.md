# Rudder Angle Indicator — ESP32-C3 + ADS1115 + TJC HMI

Firmware for a marine rudder angle indicator:

- **Angle sensor**: a 0–360° rotary angle sensor, powered from 3.3V with a
  0–3.3V output proportional to angle, on `AIN0`, actually used only over a
  135°–225° window (so its output stays well below 3.3V in normal
  operation). (Originally a 0–190 Ω variable resistor, then a Hall-effect
  sensor — the AIN0 pipeline below is unchanged from that design and works
  the same with any of these, since the 3-point calibration captures
  whatever real voltage range the rudder's actual swing produces, out of
  the sensor's full 360° span.)
- **Level sensor**: a second, floating (float-arm) 0–190 Ω sender with 13
  discrete resistance steps, on `AIN1`, wired through its own voltage
  divider (R1/R2) that keeps its normal reading in the ~1–1.5V range.
- **ADC self-check references**: `AIN2` tied to GND, `AIN3` tied to
  AVDD/3.3V — known-good levels the firmware reads back periodically to
  verify the ADS1115 and 3.3V rail are healthy.
- **ADC**: ADS1115, I2C.
- **Display**: TJC HMI (TJC8048X243, "X2" series, Nextion-protocol
  compatible), driven over UART.
- **MCU**: ESP32-C3, Arduino framework.

The firmware is an Arduino sketch: open
`RudderAngleIndicator/RudderAngleIndicator.ino` in the Arduino IDE (the
other `.h`/`.cpp` files in that folder are compiled automatically as part
of the sketch).

## Libraries required

Install via Library Manager:

- **Adafruit ADS1X15** (and its dependency **Adafruit BusIO**)

`Preferences` (used for calibration storage) and `Wire`/`HardwareSerial`
ship with the ESP32 Arduino core — no extra install needed.

Board: **ESP32C3 Dev Module** (or your specific board) under
`esp32` by Espressif Systems in Boards Manager.

## Required hardware — voltage scaling and noise filtering

The ADS1115 can only accept inputs up to roughly `VDD + 0.3V`. The
sender's 0–12 V swing **must** be scaled down before it reaches `AIN0`, and
a hardware low-pass filter should be added to remove electrical noise
before it ever reaches the ADC:

```
Sender wiper (0-12V) ----[ R_TOP 20k ]----+----[ R_SERIES 1k ]---- AIN0
                                            |                        |
                                       [ R_BOTTOM 10k ]         [ C 1uF ]
                                            |                        |
                                           GND                      GND
```

- `R_TOP` / `R_BOTTOM` = 20 kΩ / 10 kΩ → divider ratio **3.0**, so a 12 V
  sender signal becomes 4.0 V at the ADS1115 pin — safely inside the
  ±4.096 V full-scale range used by the firmware (`GAIN_ONE`).
- `R_SERIES` + `C` form a simple RC low-pass (~160 Hz cutoff) right at the
  ADC pin. A rudder angle signal has no useful content above a few Hz, so
  this removes wiper contact noise and EMI before digitization — this is
  the "noise filter from ADC" stage.
- Use 1% resistors for the divider for best accuracy; the exact ratio
  doesn't have to be 3.0 as long as `DIVIDER_RATIO` in `Config.h` is
  updated to match, and the resulting max voltage stays under the chosen
  PGA's full-scale.
- None of `AIN1`/`AIN2`/`AIN3` are grounded anymore — all four ADS1115
  inputs are now in active use (see below). On the ADS1115 all 4 inputs
  share one physical ADC core through a multiplexer, so an unused, floating
  input would otherwise inject glitches when the mux settles on it; that's
  no longer a concern here since every channel is driven by something.

If you use different resistor values, update `DIVIDER_RATIO`,
`ADS1115_GAIN` and `ADS1115_FULLSCALE_V` in `Config.h` accordingly.

### AIN1 — floating level sender

Build the same style of divider + RC filter for the AIN1 float sender as
for AIN0 above (its own `R_TOP`/`R_BOTTOM`/`R_SERIES`/`C`, tuned to that
sender's actual supply/series-resistor circuit). Update `DIVIDER_RATIO_AIN1`
and `ADS1115_GAIN_AIN1` in `Config.h` to match. Because the ADS1115's PGA
gain register is shared by all 4 inputs, the firmware re-selects the
correct gain immediately before sampling each channel — you don't need to
do anything extra for this, it's just why the code calls `ads.setGain(...)`
at the top of both sampling functions.

### AIN2/AIN3 — ADC reference self-check

No divider is needed here — both are already within the ADS1115's input
range:

- `AIN2` → straight to GND (expected reading: 0V).
- `AIN3` → straight to AVDD/3.3V (expected reading: 3.3V). A small
  decoupling cap (e.g. 100nF) from AIN3 to GND is good practice to filter
  regulator switching noise, but isn't required for this to work.

The ADS1115 must be powered from the same 3.3V rail that AIN3 is checking
(i.e. `AVDD` = the ADS1115's own `VDD`) for this to be a meaningful
self-test — feeding it a rail voltage right at `VDD` is within the
ADS1115's rated input range (`GND-0.3V` to `VDD+0.3V`), just with no
headroom above it, which is fine since AIN3 is only ever expected to sit at
that one fixed level.

## Wiring summary

| Signal              | ESP32-C3 pin (default, edit in `Config.h`) |
|---------------------|---------------------------------------------|
| I2C SDA (to ADS1115)| GPIO8                                        |
| I2C SCL (to ADS1115)| GPIO9                                        |
| HMI UART TX (to HMI RX) | GPIO21                                   |
| HMI UART RX (from HMI TX) | GPIO20                                 |
| USB / debug console | native USB (`Serial`)                        |

ADS1115 `ADDR` pin → GND (I2C address `0x48`).

## Noise-reduction / filtering pipeline

Implemented in `Filtering.h/.cpp`, `Calibration.h/.cpp`, and the main
sketch, tunable from `Config.h`:

1. **ADC-level**: PGA gain (`GAIN_ONE`, ±4.096 V) chosen to match the
   divided-down signal so the full ADC code range is used, and a data rate
   of 64 SPS — a good compromise between the ADS1115's internal
   (sigma-delta) averaging and update speed. Drop to `RATE_ADS1115_8SPS`
   for maximum internal noise rejection if a slower ~1–2 Hz display update
   is acceptable.
2. **Hardware RC filter** at the ADS1115 input (see above) — removes noise
   before it's digitized at all.
3. **Trimmed-mean batch averaging**: each update collects
   `RAW_SAMPLES_PER_BATCH` (15) raw readings, sorts them, and averages the
   middle values after discarding the `TRIM_COUNT` (2) highest and lowest —
   this rejects single-sample spikes that a plain average would let
   through.
4. **Exponential moving average (EMA)**: a software IIR low-pass
   (`EMA_ALPHA`, default 0.25) smooths the batch results across update
   cycles.
5. **Slew-rate limiter**: clamps the displayed angle to a maximum realistic
   rate of change (`SLEW_MAX_DEG_PER_SEC`, default 60°/s) — a rudder can't
   move faster than that, so this transparently rejects any noise spike
   that survived the earlier stages without adding lag to real movement.
6. **Display deadband**: a final ±0.05° deadband (`DISPLAY_DEADBAND_DEG`)
   stops the last digit from flickering at rest without hiding real
   changes.
7. **Fault detection**: if the (divider-corrected) sender voltage falls
   outside a plausible 0–12 V±margin range, the display shows
   `SENSOR FAULT` instead of a bogus angle (catches a disconnected or
   shorted sender).

## AIN1 float-level sender

Implemented in `FloatLevel.h/.cpp`; does not touch any of the AIN0 angle
code or state described above.

Unlike the angle sender, this one only ever rests at
`FLOAT_LEVEL_COUNT` (13) discrete resistance steps — it never sweeps
continuously — so it's decoded differently from the angle channel:

1. Same batch collection + trimmed-mean averaging as AIN0 (smaller batch:
   `LEVEL_RAW_SAMPLES_PER_BATCH` = 10, `LEVEL_TRIM_COUNT` = 1 — a
   discrete signal needs less averaging to resolve, just enough to reject
   glitches).
2. **No EMA.** Averaging across a transition between two real, different
   levels would produce a fake in-between voltage; instead the reading is
   snapped to whichever of the 13 calibrated voltages it's nearest to.
3. **Debounce**: a level is only reported as changed once
   `LEVEL_DEBOUNCE_BATCHES` (3) consecutive batches agree on the new
   nearest level — this is what actually rejects noise/transition chatter
   for a stepped sensor, in place of the EMA/slew-limiter used for the
   continuous angle signal.
4. **Fault detection**: same idea as AIN0 — a sender voltage outside a
   plausible range shows `LEVEL FAULT` instead of a bogus reading.

The displayed value is a 0–100% level (level 0 = 0%, level 12 = 100%,
evenly spaced) sent to `n1.val` and `t2.txt`.

### Calibrating the 13 levels

The factory defaults are just evenly-spaced placeholders — real voltages
depend on your float sender's specific fixed resistor/supply circuit, so
it must be calibrated before use:

**From USB serial** (115200 baud): move the float to each position in
turn and, for each one, type the matching command and press Enter:

- `LVL0`, `LVL1`, … `LVL12` — capture the current AIN1 voltage into that
  slot (do this once per physical float position, lowest to highest).
- `LVLSAVE` — write all 13 slots to flash.
- `LVLRESET` — restore the evenly-spaced factory defaults.
- `STATUS` — now also prints the current level-sender voltage, decoded
  level, and the full 13-slot calibration table.

**From the HMI**: add four buttons (IDs from `Config.h`:
`HMI_BTN_LVL_NEXT_ID` = 20, `_CAPTURE_ID` = 21, `_SAVE_ID` = 22,
`_RESET_ID` = 23):

1. Press **NEXT** repeatedly to select slot 0, then move the float to its
   lowest position and press **CAPTURE**.
2. Press **NEXT** to select slot 1, move the float to its next position,
   **CAPTURE** again — repeat through slot 12.
3. Press **SAVE**.

Status/confirmation messages ("SLOT 3 SET", "LVL SAVED", etc.) are written
to the `t3` text component (`HMI_COMP_LEVEL_FAULT_TXT`) — separate from the
`t1` field used by the angle calibration, so the two don't overwrite each
other's messages.

In your TJC project, also add:

- A **Number** or **Gauge** component named `n1` — receives the level as a
  0–100 integer percentage.
- A **Text** component named `t2` — receives it as text, e.g. `"70%"`.
- A **Text** component named `t3` — level fault/calibration status
  messages.
- The four calibration buttons above (optional, for field calibration).

## AIN2/AIN3 ADC reference self-check

Implemented as `checkAdcReferenceRails()` in the main sketch; does not
touch the AIN0 or AIN1 code, state, or timing (it's not part of either of
their per-batch loops).

`AIN2` (tied to GND) and `AIN3` (tied to AVDD/3.3V) are known, fixed
voltages — reading them back and comparing to the expected values is a
built-in health check that requires no calibration:

- Runs every `REFCHK_INTERVAL_MS` (default 5000 ms), not every `loop()`
  iteration — these levels never change, so there's no reason to spend
  cycles on it any faster; this keeps its overhead on the AIN0/AIN1 update
  rate negligible.
- Each check takes a small trimmed-mean batch on each channel
  (`REFCHK_SAMPLES_PER_BATCH` = 5, `REFCHK_TRIM_COUNT` = 1).
- GND is expected at `REFCHK_GND_EXPECTED_V` (0V) ± `REFCHK_GND_TOLERANCE_V`
  (0.05V); AVDD at `REFCHK_AVDD_EXPECTED_V` (3.30V) ±
  `REFCHK_AVDD_TOLERANCE_V` (0.15V) — adjust in `Config.h` if your actual
  3.3V rail runs consistently outside that window.
- On failure, a message ("ADC REF FAIL GND=...", "...VDD=...", or
  "...GND+VDD" if both are off) is written to the `t4` text component
  (`HMI_COMP_REFCHK_TXT`); it's cleared automatically once readings return
  to normal.
- Runs once at boot (so you get an immediate pass/fail at startup) and then
  on its own schedule from then on.

**From USB serial**: type `ADCCHK` to force an immediate check and print
the measured GND/AVDD voltages right away instead of waiting for the next
scheduled one; `STATUS` also now includes the last measured values and
pass/fail state.

In your TJC project, add one more component:

- A **Text** component named `t4` — ADC self-check fault messages. No
  calibration buttons are needed for this one, since it checks against
  fixed, known-in-advance voltages rather than anything sender-specific.

## Open-wire (disconnected sender) detection

AIN0 (angle sensor) and AIN1 (float sender) fail differently when a signal
wire is cut, so each gets its own detection method, layered on top of the
existing range-based fault checks (`SENDER_V_FAULT_*` / `LEVEL_V_FAULT_*`)
rather than replacing them.

### AIN1 — pinned to the supply rail

The float sender is a passive divider: R1 to Vcc, the sender's own
resistance as R2 to GND, tap in between goes to AIN1. If the wire between
that tap and the sender is cut, R2 drops out of the divider — no more
current flows through R1, so the tap is pulled up to essentially the full
supply rail.

This is caught by comparing the raw AIN1 reading directly against the
*actual measured* AVDD (`g_lastAvddVolts`, from the AIN2/AIN3 self-check
above) minus a margin (`OPEN_CIRCUIT_MARGIN_V`, default 0.15V) — not a
hardcoded 3.3V assumption, and not `senderVolts` (which is scaled by
`DIVIDER_RATIO_AIN1`), so this check works regardless of what that ratio is
set to. The result feeds the shared fault indicator described below as
`LEVEL OPEN FAULT` (vs. plain `LEVEL FAULT` for an out-of-range reading).

### AIN0 — floating input, not a pinned voltage

The angle sensor is an actively-driven (ratiometric) sensor, not a passive
divider — its own internal circuitry drives the output pin. A cut signal
wire here does **not** pin the input to a rail; the ADC input floats, and a
floating input picks up noise/crosstalk from neighboring multiplexed
channels and mains hum, so it reads erratically instead of settling
anywhere predictable. A voltage threshold can't catch this reliably — the
floating reading could even land inside a valid angle range transiently.

Instead, this is caught by watching how much the raw samples *within one
batch* spread out (`batchSpread()` in `Filtering.h/.cpp`, computed from the
same `RAW_SAMPLES_PER_BATCH` samples already collected for the trimmed-mean
average): a real, actively-driven, RC-filtered signal has a small
sample-to-sample spread even while rotating; a floating input's noise
pickup is much larger. If the spread exceeds `ANGLE_NOISE_FAULT_V` (default
0.30V), the result feeds the shared fault indicator described below as
`ANGLE OPEN FAULT` (vs. plain `ANGLE SENSOR FAULT` for an out-of-range
reading).

**Optional hardware improvement**: add a weak pull-up (100k–470kΩ) from
AIN0 to 3.3V. That would make an open angle-sensor wire pin high too, just
like AIN1's failure mode, and you could then rely on a simple threshold
there as well. The software check above works either way and doesn't
require this change, but the pull-up makes the failure mode more certain
and easier to reason about.

### A note on the existing range-based fault thresholds

`SENDER_V_FAULT_HIGH` and `LEVEL_V_FAULT_HIGH` are both still `12.3` —
sized for the original 0–12V sender design. With sensors that now output
0–3.3V, and depending on what you've set `DIVIDER_RATIO` /
`DIVIDER_RATIO_AIN1` to, these limits may never trigger even on a hard
short to the rail. They were intentionally left untouched here rather than
guessed at, since their correct value depends on those ratios; update them
in `Config.h` to match your actual electrical range if you want that
specific failure mode covered too. It doesn't affect the open-wire
detection above, which was deliberately built to be independent of both of
these.

## Unified fault indicator (blinking text + buzzer)

Implemented as `updateSharedFaultDisplay()` and `updateFaultBuzzer()` in the
main sketch, called once per `loop()` after both sampling functions. This
replaced the AIN0 and AIN1 fault checks' previous direct writes to their own
separate fault-text fields with ONE shared field, per request — `t1` and
`t3` now carry only their calibration-confirmation messages ("MIN SET",
"SLOT 3 SET", etc.), unchanged.

- **One shared text field** (`t5`, `HMI_COMP_FAULT_SHARED_TXT`) shows
  whichever fault message applies: `ANGLE SENSOR FAULT` / `ANGLE OPEN FAULT`
  for an AIN0 problem, or `LEVEL SENSOR FAULT` / `LEVEL OPEN FAULT` for an
  AIN1 problem. If both are active at the same time, the angle fault wins
  (steering takes priority over a tank/level reading); switching between
  the two updates the text immediately rather than waiting for the next
  blink.
- **Blinking**: while any fault is active, `t5`'s visibility is toggled
  with the standard `vis` instruction every `FAULT_BLINK_INTERVAL_MS`
  (default 500 ms — 1 Hz blink). It's left visible (but empty) once the
  fault clears, rather than possibly stuck hidden mid-blink.
- **Buzzer**: while any fault is active, the X2's onboard buzzer beeps for
  `FAULT_BUZZER_ON_MS` (2000 ms), then stays silent for `FAULT_BUZZER_OFF_MS`
  (8000 ms), repeating for as long as the fault persists. Uses TJC's
  documented `beep <time_ms>` instruction
  ([wiki.tjc1688.com/commands/beep.html](http://wiki.tjc1688.com/commands/beep.html))
  — a self-timed, fire-and-forget pulse: the firmware fires one `beep 2000`
  at the start of each 10-second cycle and then just waits; the buzzer
  stops on its own after 2 seconds, so there's no separate "off" instruction
  to send (and none exists). If a fault clears mid-beep, the pulse already
  in progress simply finishes on its own within at most 2 seconds.
- **Requires a physical onboard buzzer** — per TJC's docs, this only works
  on models with an actual buzzer component (a black round/square part on
  the back of the board); a display with only a speaker doesn't support
  this instruction. Volume and frequency are both fixed (not adjustable).
- The AIN2/AIN3 ADC self-check keeps its own separate `t4` field rather
  than joining this shared one — it's an internal self-test rather than a
  "sensor is faulty" condition in the sense this was asked for. Say so if
  you'd like it folded in too.

In your TJC project, add one more component for this feature:

- A **Text** component named `t5` — the single shared fault message. Give
  it high-contrast styling (e.g. bright red/yellow text) since it's meant
  to grab attention.

## AIN0 angle calibration

A 3-point calibration (full-port / midships / full-starboard) is stored in
NVS flash (`Preferences`, survives power loss / reflashing) and used with
piecewise-linear interpolation, which corrects for a mechanically
off-center zero as well as tracking the sender's linear resistance curve.

**From the HMI** — add five buttons to your TJC project (page 0) with
"Send Component ID" enabled on their Touch Release Event, and Component
IDs matching `Config.h` (`HMI_BTN_CAL_MIN_ID` = 10, `_CENTER_ID` = 11,
`_MAX_ID` = 12, `_SAVE_ID` = 13, `_RESET_ID` = 14 by default):

1. Move the rudder fully to port, press **SET MIN**.
2. Center the rudder amidships, press **SET CENTER**.
3. Move the rudder fully to starboard, press **SET MAX**.
4. Press **SAVE** to write calibration to flash.
5. **RESET** restores the factory defaults from `Config.h`.

A status/confirmation message ("MIN SET", "CAL SAVED", etc.) is written to
the `t1` text component (`HMI_COMP_FAULT_TXT` in `Config.h`) — this field is
used only for these calibration confirmations now; fault messages go to the
shared `t5` field described above.

**From USB serial** (115200 baud) the same actions are available for bench
calibration without the HMI attached: type `MIN`, `CENTER`, `MAX`, `SAVE`,
`RESET`, or `STATUS` (prints current voltage/angle/calibration) and press
Enter.

## TJC / HMI project setup

The firmware speaks the standard Nextion instruction set (which TJC's X2
series and TJC Editor are compatible with):

- Outgoing commands are plain text terminated with three `0xFF` bytes,
  e.g. `t0.txt="12.3"` + `FF FF FF`.
- Incoming touch events use the standard 7-byte frame
  `0x65 <page> <component> <event> 0xFF 0xFF 0xFF`, emitted automatically
  by a button when "Send Component ID" is enabled for its touch event.

In your TJC project (page 0), create:

- A **Number** or **Gauge** component named `n0` — receives
  `angle * 10` as an integer (`n0.val`), so bind it directly to a gauge, or
  divide by 10 in the widget's display format for one decimal place.
- A **Text** component named `t0` — receives the angle pre-formatted as
  text with one decimal, e.g. `"12.3"`.
- A **Text** component named `t1` — angle calibration confirmation messages
  only (e.g. "MIN SET"); fault display now lives on the shared `t5` field.
- Five buttons for calibration, IDs as listed above (optional, but
  recommended for field calibration without a laptop).
- The `n1`/`t2`/`t3` components and four buttons for the AIN1 float-level
  sender — see "AIN1 float-level sender" above (`t3` likewise now carries
  only level calibration confirmations, not fault display).
- The `t4` component for the AIN2/AIN3 ADC self-check — see "AIN2/AIN3 ADC
  reference self-check" above.
- The `t5` component (and, optionally, a Timer for the buzzer fallback) for
  the shared blinking fault indicator — see "Unified fault indicator" above.

Set the HMI's UART baud rate (via the TJC Editor's device settings, or a
one-time `bauds=115200` command sent from the editor's debug/format
window) to match `HMI_BAUD` in `Config.h` (115200 by default). Note: the
datasheet page linked in the task was not reachable from this environment
to confirm the TJC8048X243's factory-default baud rate, connector pinout,
and supply current — many Nextion-protocol displays default to 9600, so
double-check your unit's documentation/back label and update `HMI_BAUD`
(and the wiring table above) to match if it differs.

## Notes / assumptions

- `ANGLE_MIN_DEG` / `ANGLE_MAX_DEG` default to -50° / +50° — the rudder's
  operating range, not the angle sensor's full mechanical span (0-360° for
  the current sensor). `INVERT_ANGLE` in `Config.h` flips the sign
  convention if increasing sender voltage should read as decreasing angle
  on your installation.
- Update rate is governed by the sampling batches: the AIN0 angle batch
  (~230 ms at 64 SPS / 15 samples) plus the AIN1 level batch (~155 ms at
  64 SPS / 10 samples) run back-to-back each `loop()` iteration, since both
  channels share one ADS1115 — roughly a 385 ms full cycle. The AIN2/AIN3
  self-check adds one more small batch (~80 ms), but only once every
  `REFCHK_INTERVAL_MS` (5 s default), so its effect on the normal cycle
  time is negligible. All of this is well within what the physically
  slow-moving rudder and float mechanisms need; if you need a faster angle
  update independent of everything else, split the angle sensor onto a
  second ADS1115 (different I2C address) instead of sharing one.
