#include <Arduino.h>

// Define which UART peripheral to use on ESP32
// UART0 is usually used for USB-to-serial monitor
// UART1 TX: GPIO10, RX: GPIO9 (ESP32 DevKitC)
// UART2 TX: GPIO17, RX: GPIO16 (ESP32 DevKitC)
#define RXD2 16
#define TXD2 17

char receivedChar;
String receivedData = "";
bool newData = false;

void setup() {
  // Initialize Serial Monitor (UART0) for debugging
  Serial.begin(115200);
  Serial.println("ESP32 UART Communication Demo");
  Serial.println("Connecting to STM32F407...");

  // Initialize UART2 for communication with STM32
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Baud rate, Data format, RX pin, TX pin
}

void loop() {
  // Send data to STM32
  if (Serial2.availableForWrite()) {
    Serial2.print("Hello from ESP32!\n");
    Serial.println("Sent 'Hello from ESP32!' to STM32");
  }

  // Receive data from STM32
  while (Serial2.available()) {
    receivedChar = Serial2.read();
    if (receivedChar == '\n') { // Check for newline character as end of message
      newData = true;
      break; // Exit loop after newline, process message
    } else {
      receivedData += receivedChar;
    }
  }

  if (newData) {
    Serial.print("Received from STM32: ");
    Serial.println(receivedData);
    receivedData = ""; // Clear buffer
    newData = false;
  }

  delay(2000); // Send data every 2 seconds
}

