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

// ---------------------------------------------------------------------------
// Update timing / misc
// ---------------------------------------------------------------------------
#define SERIAL_DEBUG_BAUD      115200
#define ENABLE_SERIAL_DEBUG    1       // set 0 to silence USB debug prints
