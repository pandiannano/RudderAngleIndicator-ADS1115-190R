# Rudder Angle Indicator — ESP32-C3 + ADS1115 + TJC HMI

Firmware for a marine rudder angle indicator:

- **Sender**: 0–190 Ω variable resistor (-50° … +50°), wired as a bias
  voltage divider producing a 0–12 V signal.
- **ADC**: ADS1115, I2C, sender signal on `AIN0`; `AIN1`/`AIN2`/`AIN3` tied
  to GND.
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
- `AIN1`/`AIN2`/`AIN3` should still be tied to GND as you planned — on the
  ADS1115 all 4 inputs share one physical ADC core through a multiplexer,
  so grounding unused channels avoids them injecting glitches when the mux
  settles on `AIN0`.

If you use different resistor values, update `DIVIDER_RATIO`,
`ADS1115_GAIN` and `ADS1115_FULLSCALE_V` in `Config.h` accordingly.

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

## Calibration

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
- Update rate is governed by the sampling batch: at 64 SPS with 15 samples
  per batch, a full cycle (and HMI update) happens roughly every 230 ms —
  more than fast enough for a mechanism that physically can't move faster
  than tens of degrees per second, while keeping heavy oversampling for
  noise rejection.
