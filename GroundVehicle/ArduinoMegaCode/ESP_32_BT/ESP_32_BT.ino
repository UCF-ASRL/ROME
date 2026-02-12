#include "BluetoothSerial.h"
BluetoothSerial BT;

// UART to Mega: TX only on GPIO17 (connect to Mega RX pin 19)
// RX pin = -1 since we don't read back from Mega here.
static constexpr int MEGA_RX_PIN = -1;   // unused
static constexpr int MEGA_TX_PIN = 17;   // ESP32 TX -> Mega RX19
static constexpr uint32_t BAUD = 115200; //match this to Matlab + Mega
static char btBuf[128];
static int btLen = 0;

void setup() {

  // Start Bluetooth
  BT.begin("ESP32_BT_Link");
  // Serial.println("ESP32 ready. Pair and connect via Bluetooth.");

  // Start UART toward the Mega
  Serial1.begin(BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  // Serial.begin(115200);
}

void loop() 
{
  while (BT.available()) {
    char c = BT.read();

    // accumulate into buffer
    if (btLen < sizeof(btBuf) - 1) {
      btBuf[btLen++] = c;
    }

    // if newline, print the full message and forward
    if (c == '\n') {
      btBuf[btLen] = '\0'; // terminate string

      // // Debug print: show the full line
      // Serial.print("BT -> ESP32 full line: ");
      // Serial.println(btBuf);

      // Forward entire line to Mega
      Serial1.write((uint8_t*)btBuf, btLen);

      // reset buffer
      btLen = 0;
    }
  }
}
