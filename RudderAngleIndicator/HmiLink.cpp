#include "HmiLink.h"
#include "Config.h"

static HardwareSerial hmiSerial(HMI_UART_NUM);

// Small ring buffer for reassembling the fixed 7-byte touch-event frame.
static uint8_t frameBuf[7];
static uint8_t frameLen = 0;

void hmiBegin() {
  hmiSerial.begin(HMI_BAUD, SERIAL_8N1, HMI_RX_PIN, HMI_TX_PIN);
}

void hmiSendRaw(const String &payload) {
  hmiSerial.print(payload);
  hmiSerial.write(0xFF);
  hmiSerial.write(0xFF);
  hmiSerial.write(0xFF);
}

void hmiSendNumber(const char *component, long value) {
  String cmd = String(component) + ".val=" + String(value);
  hmiSendRaw(cmd);
}

void hmiSendText(const char *component, const String &text) {
  String escaped = text;
  escaped.replace("\"", "\\\"");
  String cmd = String(component) + ".txt=\"" + escaped + "\"";
  hmiSendRaw(cmd);
}

static void resetFrame() { frameLen = 0; }

void hmiPoll() {
  while (hmiSerial.available()) {
    uint8_t b = hmiSerial.read();

    if (frameLen == 0) {
      if (b != 0x65) continue; // wait for a touch-event frame header
      frameBuf[frameLen++] = b;
      continue;
    }

    frameBuf[frameLen++] = b;

    if (frameLen == 7) {
      // Expect the frame to end with 0xFF 0xFF 0xFF.
      if (frameBuf[4] == 0xFF && frameBuf[5] == 0xFF && frameBuf[6] == 0xFF) {
        onHmiTouchEvent(frameBuf[1], frameBuf[2], frameBuf[3]);
      }
      resetFrame();
    }
  }
}
