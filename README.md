# Rudder Angle Indicator — ESP32-C3 + ADS1115 + TJC HMI

Firmware for a marine rudder angle indicator:

- **Angle sensor**: Hall-effect angle sensor (-50° … +50°), 0–12 V output,
  on `AIN0`. (Originally a 0–190 Ω variable resistor wired as a bias
  voltage divider — the AIN0 pipeline below is unchanged from that design
  and works the same with either sender.)
- **Level sensor**: a second, floating (float-arm) 0–190 Ω sender with 10
  discrete resistance steps, on `AIN1`.
- **ADC**: ADS1115, I2C; `AIN2`/`AIN3` tied to GND.
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
- `AIN2`/`AIN3` should still be tied to GND — on the ADS1115 all 4 inputs
  share one physical ADC core through a multiplexer, so grounding unused
  channels avoids them injecting glitches when the mux settles on a used
  channel. `AIN1` is now used by the float-level sender (see below), so it
  should no longer be grounded.

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
`FLOAT_LEVEL_COUNT` (10) discrete resistance steps — it never sweeps
continuously — so it's decoded differently from the angle channel:

1. Same batch collection + trimmed-mean averaging as AIN0 (smaller batch:
   `LEVEL_RAW_SAMPLES_PER_BATCH` = 10, `LEVEL_TRIM_COUNT` = 1 — a
   discrete signal needs less averaging to resolve, just enough to reject
   glitches).
2. **No EMA.** Averaging across a transition between two real, different
   levels would produce a fake in-between voltage; instead the reading is
   snapped to whichever of the 10 calibrated voltages it's nearest to.
3. **Debounce**: a level is only reported as changed once
   `LEVEL_DEBOUNCE_BATCHES` (3) consecutive batches agree on the new
   nearest level — this is what actually rejects noise/transition chatter
   for a stepped sensor, in place of the EMA/slew-limiter used for the
   continuous angle signal.
4. **Fault detection**: same idea as AIN0 — a sender voltage outside a
   plausible range shows `LEVEL FAULT` instead of a bogus reading.

The displayed value is a 0–100% level (level 0 = 0%, level 9 = 100%,
evenly spaced) sent to `n1.val` and `t2.txt`.

### Calibrating the 10 levels

The factory defaults are just evenly-spaced placeholders — real voltages
depend on your float sender's specific fixed resistor/supply circuit, so
it must be calibrated before use:

**From USB serial** (115200 baud): move the float to each position in
turn and, for each one, type the matching command and press Enter:

- `LVL0`, `LVL1`, … `LVL9` — capture the current AIN1 voltage into that
  slot (do this once per physical float position, lowest to highest).
- `LVLSAVE` — write all 10 slots to flash.
- `LVLRESET` — restore the evenly-spaced factory defaults.
- `STATUS` — now also prints the current level-sender voltage, decoded
  level, and the full 10-slot calibration table.

**From the HMI**: add four buttons (IDs from `Config.h`:
`HMI_BTN_LVL_NEXT_ID` = 20, `_CAPTURE_ID` = 21, `_SAVE_ID` = 22,
`_RESET_ID` = 23):

1. Press **NEXT** repeatedly to select slot 0, then move the float to its
   lowest position and press **CAPTURE**.
2. Press **NEXT** to select slot 1, move the float to its next position,
   **CAPTURE** again — repeat through slot 9.
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
the `t1` text component (`HMI_COMP_FAULT_TXT` in `Config.h`).

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
- A **Text** component named `t1` — used for fault ("SENSOR FAULT") and
  calibration confirmation messages.
- Five buttons for calibration, IDs as listed above (optional, but
  recommended for field calibration without a laptop).
- The `n1`/`t2`/`t3` components and four buttons for the AIN1 float-level
  sender — see "AIN1 float-level sender" above.

Set the HMI's UART baud rate (via the TJC Editor's device settings, or a
one-time `bauds=115200` command sent from the editor's debug/format
window) to match `HMI_BAUD` in `Config.h` (115200 by default). Note: the
datasheet page linked in the task was not reachable from this environment
to confirm the TJC8048X243's factory-default baud rate, connector pinout,
and supply current — many Nextion-protocol displays default to 9600, so
double-check your unit's documentation/back label and update `HMI_BAUD`
(and the wiring table above) to match if it differs.

## Notes / assumptions

- `ANGLE_MIN_DEG` / `ANGLE_MAX_DEG` default to -50° / +50° as specified;
  `INVERT_ANGLE` in `Config.h` flips the sign convention if increasing
  sender voltage should read as decreasing angle on your installation.
- Update rate is governed by the sampling batches: the AIN0 angle batch
  (~230 ms at 64 SPS / 15 samples) plus the AIN1 level batch (~155 ms at
  64 SPS / 10 samples) run back-to-back each `loop()` iteration, since both
  channels share one ADS1115 — roughly a 385 ms full cycle. Both channels
  update well within what their physically slow-moving mechanisms need;
  if you need a faster angle update independent of the level sensor, split
  them across two separate ADS1115 chips (different I2C addresses) instead
  of one shared one.
