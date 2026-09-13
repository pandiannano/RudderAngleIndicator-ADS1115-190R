// HmiLink.h
// Minimal driver for talking to a TJC / Nextion-protocol HMI over UART:
//  - outgoing: text/numeric component updates ("t0.txt=...", "n0.val=...")
//    each terminated with the mandatory 0xFF 0xFF 0xFF
//  - incoming: standard touch-event frames
//        0x65 <pageId> <componentId> <eventType> 0xFF 0xFF 0xFF
//    emitted automatically by a button when "Send Component ID" is enabled
//    for its Touch Release Event in the TJC Editor.
//
// The .ino defines onHmiTouchEvent() (declared here as extern) to react to
// calibration button presses without HmiLink needing to know about
// Calibration/filters — keeps this file a dumb, reusable transport.
#pragma once
#include <Arduino.h>

void hmiBegin();
void hmiSendRaw(const String &payload);            // appends 0xFF 0xFF 0xFF
void hmiSendNumber(const char *component, long value);
void hmiSendText(const char *component, const String &text);

// Call every loop() iteration; parses any complete incoming frames and
// dispatches them to onHmiTouchEvent().
void hmiPoll();

// Implemented in the main sketch.
void onHmiTouchEvent(uint8_t pageId, uint8_t componentId, uint8_t eventType);
