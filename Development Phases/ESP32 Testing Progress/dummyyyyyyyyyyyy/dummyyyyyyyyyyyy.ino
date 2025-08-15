#include <HardwareSerial.h>

// --- UART Communication with STM32 ---
HardwareSerial SerialPortFromSTM32(2); // Using Serial2 on ESP32 (pins 16, 17 by default)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize the serial port to communicate with the STM32
  SerialPortFromSTM32.begin(115200);
  Serial.println("ESP32 UART Diagnostic Tool: Ready to receive data from STM32...");
}

void loop() {
  // Check if any data is available on the UART port from the STM32
  if (SerialPortFromSTM32.available()) {
    // Read the incoming byte
    char receivedChar = SerialPortFromSTM32.read();
    
    // Print the received byte to the main Serial Monitor
    Serial.print(receivedChar);
  }
  
  // A small delay to prevent potential watchdog timer resets
  delay(10);
}
