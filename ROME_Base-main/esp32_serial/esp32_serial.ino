/* This code should be running on the ESP32 to receive serial data from the MATLAB PC
over Bluetooth and pass it on to the Arduino Mega*/

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  // Serial to computer for debugging
  Serial.begin(115200); 
  
  // Hardware Serial to Arduino (Pins 16/RX, 17/TX are defaults for UART2)
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 
  
  // Bluetooth device name
  SerialBT.begin("ESP32_BT_Link"); 
  Serial.println("Bluetooth started, ready to pair.");
}

void loop() {
  // Receive from Bluetooth and send to Arduino
  if (SerialBT.available()) {
    char data = SerialBT.read();
    Serial2.write(data); // Send to Arduino via TX pin
    // Serial.write(data);  // Optional: monitor on PC
  }
  
  // Optional: Receive from Arduino and send to Bluetooth
  // if (Serial2.available()) {
  //   SerialBT.write(Serial2.read());
  // }
  // delay(10);
}