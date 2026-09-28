// Config.h
// All hardware wiring, calibration defaults and filter tuning constants live
// here so the rest of the firmware never contains "magic numbers".
//
// ESP32-C3 target board. Adjust the pin numbers below to match your module.

#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

// ---------------------------------------------------------------------------
// Build option — same board, same source file: pick which sensors this
// build includes by changing ACTIVE_BUILD_OPTION below, then reflash.
//   BUILD_OPTION_ANGLE_ONLY      -> only the AIN0 rudder angle indicator
//   BUILD_OPTION_ANGLE_AND_LEVEL -> rudder angle (AIN0) + oil/level (AIN1)
// ---------------------------------------------------------------------------
#define BUILD_OPTION_ANGLE_ONLY        1
#define BUILD_OPTION_ANGLE_AND_LEVEL   2

#define ACTIVE_BUILD_OPTION   BUILD_OPTION_ANGLE_AND_LEVEL   // <-- change this line

// ---------------------------------------------------------------------------
// I2C bus (ADS1115)
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN          8      // change to match your ESP32-C3 board
#define I2C_SCL_PIN          9
#define I2C_CLOCK_HZ         400000UL

#define ADS1115_I2C_ADDR     0x48   // ADDR pin tied to GND
#define ADS1115_CHANNEL      0      // AIN0 = angle sensor signal

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
// fault rather than a real angle. NOTE: these limits date from the original
// 0-12V sender and were left as-is since DIVIDER_RATIO's current value
// (whatever you've since set it to for the 0-3.3V sensor) isn't known here —
// they will effectively never trigger at 3.3V-range signal levels unless you
// update them to match your actual DIVIDER_RATIO-scaled voltage range. This
// isn't a blocker for the angle sensor's open-wire case specifically (see
// ANGLE_NOISE_FAULT_V below, which is the check that actually catches that),
// but it does mean a hard short-to-rail on AIN0 wouldn't be caught by this
// range check alone until updated.
#define SENDER_V_FAULT_LOW     -0.3f
#define SENDER_V_FAULT_HIGH    12.3f

// Root-cause fix for the angle sensor's open-wire fault going quiet after
// one beep: ANGLE_NOISE_FAULT_V only catches noise WHILE the floating input
// is actively bouncing around, but on real hardware a floating input can
// settle to a quiet, stable-but-wrong voltage within a second or so, after
// which that check stops tripping even though the wire is still cut — the
// latch (FAULT_CLEAR_CONFIRM_BATCHES) only delays that false-clear, it
// can't prevent it if the underlying signal genuinely goes quiet for good.
//
// This margin instead checks the reading against your OWN captured
// calibration span (Calibration::isWithinCalibratedRange(), using
// calibration.points()' vAtMin/vAtMax) rather than a fixed voltage — so it
// needs no per-user tuning and works regardless of DIVIDER_RATIO. It's
// aimed at sensors like this one, where the sensor's full mechanical/
// electrical range (0-360 degrees here) is much wider than the narrow
// operating window that's actually calibrated (135-225 degrees here): a
// wire that settles anywhere outside that calibrated window — which is
// likely, since floating CMOS inputs commonly settle toward a supply rail —
// is caught persistently (not just transiently), same as AIN1's pinned-to-
// rail check. 0.20 = allow 20% of the calibrated span as legitimate
// overtravel past each end before calling it a fault.
#define CAL_RANGE_FAULT_MARGIN_FRAC  0.20f

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
// this means an open/shorted float sender or wiring fault. NOTE: same
// caveat as SENDER_V_FAULT_HIGH above — these limits are from the original
// 0-12V design and won't trigger at 3.3V-scale levels unless updated to
// match your current DIVIDER_RATIO_AIN1. The open-wire case you actually
// asked about (sender wire cut -> tap pulled to Vcc) is instead caught by
// OPEN_CIRCUIT_MARGIN_V below, which compares the raw ADC reading directly
// against the measured AVDD rail and so doesn't depend on this value.
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
// (0..FLOAT_LEVEL_COUNT-1), CAPTURE stores the current filtered AIN1
// voltage into it.
#define HMI_BTN_LVL_NEXT_ID     20
#define HMI_BTN_LVL_CAPTURE_ID  21
#define HMI_BTN_LVL_SAVE_ID     22
#define HMI_BTN_LVL_RESET_ID    23

// ---------------------------------------------------------------------------
// AIN2/AIN3 ADC reference self-check — new, additive only. AIN2 is now
// wired straight to GND and AIN3 straight to AVDD (3.3V) as known reference
// levels; periodically reading them back and comparing against the
// expected values is a simple self-test that catches a failed/miswired
// ADS1115, a bad I2C link, or the 3.3V rail sagging — all without touching
// the AIN0/AIN1 sampling functions above (those already re-assert their
// own required PGA gain at the top of each call, so a third channel using
// yet another gain slots in the same way).
// ---------------------------------------------------------------------------
#define AIN2_CHANNEL            2      // AIN2 = tied to GND (expect ~0V)
#define AIN3_CHANNEL            3      // AIN3 = tied to AVDD/3.3V (expect ~3.3V)

// 0-3.3V comfortably fits the same +-4.096V PGA range used elsewhere.
#define ADS1115_GAIN_REFCHK     GAIN_ONE

#define REFCHK_SAMPLES_PER_BATCH  5    // small batch: these are static levels
#define REFCHK_TRIM_COUNT         1

// This is a slow health check on two levels that never change, so it does
// not need to run every loop() iteration like the AIN0/AIN1 channels do —
// it self-paces via millis() to this interval instead, keeping the added
// overhead on the existing loop negligible.
#define REFCHK_INTERVAL_MS        5000UL

#define REFCHK_GND_EXPECTED_V     0.0f
#define REFCHK_GND_TOLERANCE_V    0.05f
#define REFCHK_AVDD_EXPECTED_V    3.30f
#define REFCHK_AVDD_TOLERANCE_V   0.15f

#define HMI_COMP_REFCHK_TXT     "t4"   // ADC self-check fault/status text

// ---------------------------------------------------------------------------
// Open-wire (disconnected sender) detection — additive; does not replace or
// touch the SENDER_V_FAULT_*/LEVEL_V_FAULT_* range checks above.
//
// AIN1 float sender: it's a passive divider (R1 to Vcc, the sender's own
// resistance as R2 to GND). If the wire between the R1/R2 junction and the
// sender is cut, R2 drops out and no current flows through R1 anymore, so
// the tap is pulled up to essentially the supply rail. Checked against the
// ACTUAL measured AVDD (from the AIN2/AIN3 self-check, g_lastAvddVolts in
// the .ino) rather than a hardcoded 3.3, so it tracks the real rail and
// doesn't depend on DIVIDER_RATIO_AIN1's value.
#define OPEN_CIRCUIT_MARGIN_V     0.15f

// AIN0 angle sensor: this is an actively-driven (ratiometric) output, not a
// passive divider, so a cut signal wire does NOT pin the input to a rail —
// it floats, and a floating ADC input picks up noise/crosstalk and reads
// erratically ("oscillates") instead of settling anywhere predictable. A
// voltage threshold can't catch that reliably (the floating reading could
// even land inside a valid range transiently), so instead this looks at how
// much the raw samples within one batch spread out (batchSpread() in
// Filtering.h): a real, actively-driven, RC-filtered signal has a small
// sample-to-sample spread even while rotating; a floating input's noise
// pickup is much larger. Recommended hardware improvement: add a weak
// (100k-470k) pull-up from AIN0 to 3.3V so an open wire pins high instead
// of floating, exactly like AIN1 above — then the same simple threshold
// approach would also apply to AIN0. This software check works either way,
// and is what actually catches it if you don't add that resistor.
#define ANGLE_NOISE_FAULT_V       0.30f

// A floating ADC input often bursts with noise right when it's disconnected
// but then settles toward some quiet, stable-but-meaningless voltage as its
// parasitic capacitance charges up — at which point a single-batch spread
// check like ANGLE_NOISE_FAULT_V can stop tripping even though the wire is
// still cut. To avoid the fault (and its buzzer) silently going quiet in
// that case, an active fault is only cleared after this many CONSECUTIVE
// clean batches — a single good-looking reading is not enough (fast-trip,
// slow-reset). Applied to both AIN0 and AIN1's fault flags for consistency,
// though AIN1's pinned-to-rail open fault doesn't strictly need it (that
// voltage stays put once the wire is cut, so it never needed latching to
// begin with).
#define FAULT_CLEAR_CONFIRM_BATCHES  5

// ---------------------------------------------------------------------------
// Unified fault indicator — ONE shared HMI text field for both the AIN0
// (angle) and AIN1 (float) fault conditions above, instead of writing to
// their own separate fault texts (t1/t3, which now carry ONLY calibration-
// confirmation messages like "MIN SET" — unchanged, still written from the
// doCal*/doLvl* functions). When both are active at once, the angle fault
// (steering) takes priority for display. The AIN2/AIN3 self-check keeps its
// own separate t4 field — it's an internal self-test, not a sensor fault in
// the sense asked for here; say so if you'd like it folded in too.
//
// Blinks (via the standard `vis` show/hide instruction) while any fault is
// active, and drives the X2's onboard buzzer in a repeating on/off pattern.
// ---------------------------------------------------------------------------
#define HMI_COMP_FAULT_SHARED_TXT   "t5"   // single shared fault text component

#define FAULT_BLINK_INTERVAL_MS     500UL  // on/off toggle period while a fault is active

#define FAULT_BUZZER_ON_MS          2000UL // beep duration
#define FAULT_BUZZER_OFF_MS         8000UL // silence after each beep, then repeats

// TJC's documented buzzer instruction (http://wiki.tjc1688.com/commands/beep.html)
// is `beep <time>`, time in milliseconds — a self-timed, fire-and-forget
// pulse: the buzzer sounds for exactly that long and then stops on its own;
// there is no separate "off" instruction, and none is needed. Requires a
// model with a physical onboard buzzer (a black round/square component on
// the back of the board) — screens without one (or with only a speaker)
// don't support this instruction at all.

// ---------------------------------------------------------------------------
// HMI boot-splash workaround: the ESP32 starts sending display updates
// almost immediately, but the TJC screen spends its first few seconds on
// its own loading animation and isn't listening yet — so those early
// updates are missed. Since values are normally only re-sent when they
// change, a sensor that stays put after that window never gets displayed
// until it moves. Fixed by forcing a few "resend everything" refreshes
// during the first several seconds after boot, timed to land after the
// splash screen — see the g_bootRefresh logic in the .ino.
// ---------------------------------------------------------------------------
#define HMI_BOOT_REFRESH_COUNT         3      // how many forced refreshes
#define HMI_BOOT_REFRESH_INTERVAL_MS   2000UL // spacing between them (2s,4s,6s)

// ---------------------------------------------------------------------------
// Update timing / misc
// ---------------------------------------------------------------------------
#define SERIAL_DEBUG_BAUD      115200
#define ENABLE_SERIAL_DEBUG    1       // set 0 to silence USB debug prints
