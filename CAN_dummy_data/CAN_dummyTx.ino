#include <SPI.h>
#include <mcp2515.h> // Include the MCP2515 library

// Define the CS pin for MCP2515. This is typically D10 on Arduino Uno.
MCP2515 mcp2515(10); 

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  while (!Serial);      // Wait for serial port to connect (Leonardo/Micro)

  Serial.println("Arduino CAN Sender (Normal Mode)");

  // Initialize MCP2515
  // This resets the MCP2515 chip and prepares it for configuration.
  if (mcp2515.reset() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1); // Halt if initialization fails
  }

  // Set CAN bitrate (e.g., 500 KBPS).
  // IMPORTANT: This MUST match the bitrate configured on your STM32.
  // MCP_8MHZ assumes an 8MHz crystal on your MCP2515 module.
  // If your module has a 16MHz crystal, use MCP_16MHZ.
  // The provided STM32 code (if APB1 is 42MHz) is set for 500KBPS.
  // If your STM32 APB1 is 16MHz, you'd need to adjust STM32's CAN settings.
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    Serial.println("CAN Bitrate Set to 500 KBPS");
  } else {
    Serial.println("Error Setting CAN Bitrate...");
    while (1); // Halt if bitrate setting fails
  }

  // Set MCP2515 to NORMAL mode for actual CAN bus communication.
  // This is the key change from the loopback test.
  if (mcp2515.setNormalMode() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 in Normal Mode");
  } else {
    Serial.println("Error Setting Normal Mode...");
    while (1); // Halt if mode setting fails
  }
}

void loop() {
  struct can_frame txMsg;

  txMsg.can_id = 0x123; // CAN ID (standard 11-bit). This is the ID the STM32 is configured to receive.
  txMsg.can_dlc = 8;    // Data Length Code (number of bytes, max 8)

  // Fill data bytes
  // We'll increment the first byte to easily see new messages on the STM32 side.
  static uint8_t counter = 0;
  txMsg.data[0] = counter++;
  txMsg.data[1] = 0xAA;
  txMsg.data[2] = 0xBB;
  txMsg.data[3] = 0xCC;
  txMsg.data[4] = 0xDD;
  txMsg.data[5] = 0xEE;
  txMsg.data[6] = 0xFF;
  txMsg.data[7] = 0x11; // Example fixed data byte

  // Send the message
  // The sendMessage() function attempts to transmit the frame over the CAN bus.
  // It returns ERROR_OK if the message was successfully put into the transmit buffer
  // AND received an ACK from another node (STM32 in this case).
  if (mcp2515.sendMessage(&txMsg) == MCP2515::ERROR_OK)
  {
    Serial.print("Sent CAN message with ID: 0x");
    Serial.print(txMsg.can_id, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < txMsg.can_dlc; i++)
    {
      Serial.print("0x");
      Serial.print(txMsg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else
  {
    Serial.println("Error sending CAN message.");
    // If you see persistent "Error sending CAN message" here after setting to Normal Mode,
    // it indicates issues on the physical CAN bus:
    // 1. STM32 not powered/running/correctly configured.
    // 2. Incorrect CANH/CANL wiring.
    // 3. Bitrate mismatch between Arduino and STM32.
    // 4. Missing/incorrect 120-ohm termination resistors.
  }

  delay(1000); // Send a message every 1 second
}
