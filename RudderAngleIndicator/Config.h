// Config.h
// All hardware wiring, calibration defaults and filter tuning constants live
// here so the rest of the firmware never contains "magic numbers".
//
// ESP32-C3 target board. Adjust the pin numbers below to match your module.

#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

// ---------------------------------------------------------------------------
// I2C bus (ADS1115)
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN          8      // change to match your ESP32-C3 board
#define I2C_SCL_PIN          9
#define I2C_CLOCK_HZ         400000UL

#define ADS1115_I2C_ADDR     0x48   // ADDR pin tied to GND
#define ADS1115_CHANNEL      0      // AIN0 = sender signal, AIN1..AIN3 = GND

// Full-scale range of the ADS1115 PGA. The external divider (see below) MUST
// bring the 0-12V sender signal below this value with some safety margin.
// GAIN_ONE -> +-4.096V full scale, 125uV/count -> best resolution for a
// signal that has been divided down to a 0-4V swing.
#define ADS1115_GAIN         GAIN_ONE
#define ADS1115_FULLSCALE_V  4.096f

// Lower SPS = more internal (on-chip sigma-delta) averaging = less noise,
// at the cost of slower updates. 64 SPS is a good compromise for a rudder
// indicator; drop to RATE_ADS1115_8SPS for maximum noise rejection if a
// ~1-2 Hz update rate is acceptable.
#define ADS1115_DATA_RATE    RATE_ADS1115_64SPS

// ---------------------------------------------------------------------------
// Sender / voltage-divider scaling
// ---------------------------------------------------------------------------
// The 0-190 ohm sender is wired as a bias/voltage divider that produces a
// 0-12V signal proportional to rudder angle. Because the ADS1115 can only
// accept inputs up to VDD+0.3V, a second resistor divider (external to the
// ADS1115) MUST be built to bring that 0-12V down into the PGA range chosen
// above. Example: R_TOP=20k from sender to AIN0, R_BOTTOM=10k from AIN0 to
// GND -> divider ratio 3.0 -> 12V sender = 4.0V at AIN0 (safely inside the
// 4.096V full-scale of GAIN_ONE).
//
// Also fit a small RC anti-alias/noise filter right at the ADS1115 pin
// (e.g. 1k series resistor + 1uF to GND, ~160Hz cutoff) - a rudder angle
// signal has no useful content above a few Hz, so this removes a large
// amount of electrical noise before it ever reaches the ADC.
#define DIVIDER_RATIO        3.0f     // (R_TOP + R_BOTTOM) / R_BOTTOM

// ---------------------------------------------------------------------------
// Angle range and default (factory) calibration
// ---------------------------------------------------------------------------
#define ANGLE_MIN_DEG        -50.0f   // full port
#define ANGLE_MAX_DEG         50.0f   // full starboard
#define ANGLE_CENTER_DEG      0.0f    // midships

// Sender voltage (AFTER dividing back down by DIVIDER_RATIO, i.e. the real
// 0-12V sender voltage) expected at each reference point. These are only
// used until real calibration is performed and saved; SET_MIN / SET_CENTER
// / SET_MAX (see HmiLink.h) overwrite them in NVS.
#define DEFAULT_V_AT_MIN      0.5f
#define DEFAULT_V_AT_CENTER   6.0f
#define DEFAULT_V_AT_MAX      11.5f

// Set true if increasing sender voltage should correspond to decreasing
// (port-going) angle on your particular installation.
#define INVERT_ANGLE          false

// ---------------------------------------------------------------------------
// Sampling / filtering pipeline
// ---------------------------------------------------------------------------
#define RAW_SAMPLES_PER_BATCH   15    // samples collected per update cycle
#define TRIM_COUNT              2     // # of highest/lowest samples dropped
                                       // before averaging (spike rejection)

#define EMA_ALPHA               0.25f // 0..1, smaller = smoother/slower
#define SLEW_MAX_DEG_PER_SEC     60.0f // clamps unrealistic angle jumps
#define DISPLAY_DEADBAND_DEG     0.05f // suppresses last-digit flicker

// ---------------------------------------------------------------------------
// Fault detection
// ---------------------------------------------------------------------------
// Plausible range of the real sender voltage (post divider-correction).
// Anything outside this (with margin) means an open/shorted sender or wiring
// fault rather than a real angle.
#define SENDER_V_FAULT_LOW     -0.3f
#define SENDER_V_FAULT_HIGH    12.3f

// ---------------------------------------------------------------------------
// Floating (float-type) level sender on AIN1 — added alongside the existing
// AIN0 angle channel. Unlike the AIN0 sender, this one does not move
// continuously: it is a 0-190 ohm sender operated by a float arm that only
// ever rests at 10 discrete resistance steps, so it is decoded as a
// nearest-match against a calibrated table of 10 voltages rather than an
// interpolated curve. Everything in this section is new; nothing above it
// (the AIN0 angle pipeline) was changed to add this.
// ---------------------------------------------------------------------------
#define AIN1_CHANNEL            1      // AIN1 = floating level sender

// The ADS1115 has a single PGA gain register shared by ALL FOUR inputs, so
// the firmware must (re)select the gain appropriate to each channel right
// before sampling it. Default assumes the same style of external divider
// (see DIVIDER_RATIO above) is used for this sender too; change if the
// float sender's divider/supply produces a different voltage swing.
#define ADS1115_GAIN_AIN1       GAIN_ONE
#define DIVIDER_RATIO_AIN1      3.0f   // (R_TOP + R_BOTTOM) / R_BOTTOM, AIN1 divider

#define FLOAT_LEVEL_COUNT       13     // number of discrete float positions

// Fewer samples than the angle channel's batch: a discrete sensor does not
// need heavy averaging to resolve fine steps, only enough to reject noise
// glitches before the nearest-level decision.
#define LEVEL_RAW_SAMPLES_PER_BATCH  10
#define LEVEL_TRIM_COUNT             1

// A level is only reported as changed once this many consecutive sample
// batches agree on the new nearest level — rejects transient chatter while
// the float arm/wiper is physically moving between two resistor steps.
#define LEVEL_DEBOUNCE_BATCHES       3

// Plausible real (post divider-correction) sender-voltage range; outside
// this means an open/shorted float sender or wiring fault.
#define LEVEL_V_FAULT_LOW      -0.3f
#define LEVEL_V_FAULT_HIGH     12.3f

// Factory-default voltage for each of the 10 levels (evenly spaced
// placeholders — the exact values depend on your float sender's fixed
// series resistor and supply voltage, which are not specified here). Use
// the LVL0..LVL9 serial commands, or the HMI capture button, to replace
// these with real measured values and save to flash; see README.md.
#define DEFAULT_LEVEL_V_STEP    (11.5f / (FLOAT_LEVEL_COUNT - 1))

// ---------------------------------------------------------------------------
// HMI (TJC / Nextion-protocol) UART link
// ---------------------------------------------------------------------------
#define HMI_UART_NUM          1       // use HardwareSerial(1)
#define HMI_TX_PIN            21
#define HMI_RX_PIN            20
#define HMI_BAUD              115200  // must match the baud set in the TJC
                                       // project ("bauds=115200" or via the
                                       // Nextion Editor device settings)

// Names of the TJC/Nextion components the firmware writes to. Create these
// in the TJC project (page 0) with matching names, or edit these to match
// names you already used.
#define HMI_COMP_ANGLE_NUM    "n0"    // Number/Gauge component: n0.val
#define HMI_COMP_ANGLE_TXT    "t0"    // Text component: t0.txt
#define HMI_COMP_FAULT_TXT    "t1"    // Text component used for fault text
#define HMI_COMP_FAULT_VIS    "vis0"  // (unused placeholder, see HmiLink.cpp)

// New components for the AIN1 float-level sender (does not touch any of
// the angle components above).
#define HMI_COMP_LEVEL_NUM     "n1"   // Number/Gauge component: n1.val (0-100%)
#define HMI_COMP_LEVEL_TXT     "t2"   // Text component, e.g. "70%"
#define HMI_COMP_LEVEL_FAULT_TXT "t3" // Fault text + calibration status messages

// Touch-event component IDs used for on-screen calibration buttons. These
// are the numeric Component ID assigned in the TJC Editor's widget
// properties (NOT the widget name), on page 0. Enable "Send Component ID"
// for the Touch Release Event of each button.
#define HMI_PAGE_MAIN          0
#define HMI_BTN_CAL_MIN_ID     10
#define HMI_BTN_CAL_CENTER_ID  11
#define HMI_BTN_CAL_MAX_ID     12
#define HMI_BTN_CAL_SAVE_ID    13
#define HMI_BTN_CAL_RESET_ID   14

// AIN1 float-level calibration buttons: NEXT cycles the selected slot
// (0..9), CAPTURE stores the current filtered AIN1 voltage into it.
#define HMI_BTN_LVL_NEXT_ID     20
#define HMI_BTN_LVL_CAPTURE_ID  21
#define HMI_BTN_LVL_SAVE_ID     22
#define HMI_BTN_LVL_RESET_ID    23

// ---------------------------------------------------------------------------
// Update timing / misc
// ---------------------------------------------------------------------------
#define SERIAL_DEBUG_BAUD      115200
#define ENABLE_SERIAL_DEBUG    1       // set 0 to silence USB debug prints
