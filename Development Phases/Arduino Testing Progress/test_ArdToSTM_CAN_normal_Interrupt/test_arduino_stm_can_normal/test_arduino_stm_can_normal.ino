#include <SPI.h>
#include <mcp_can.h>

// Set CS pin (usually D10 or D9 on popular shields)
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

void setup() {
  Serial.begin(115200); // For debugging output to Arduino Serial Monitor
  while (!Serial); // Wait for serial port to connect. Needed for native USB port only

  Serial.println("Initializing MCP2515 CAN controller...");

  // Initialize CAN at 500 kbps (match STM32's speed)
  // The first parameter is the CAN speed in kbps.
  // The second parameter is the clock frequency of the crystal on the MCP2515 module.
  // Most modules have an 8MHz crystal.
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("MCP2515 Initialization Failed!");
    Serial.println("Check connections and crystal frequency.");
    while(1); // Halt if initialization fails
  }

  // Set CAN filters to receive messages (optional, but good practice)
  // For simplicity, we can set it to accept all standard IDs
  // CAN.init_Mask(0, 0, 0x00000000); // Set Mask0 to all zeros (accept all IDs)
  // CAN.init_Filt(0, 0, 0x00000000); // Set Filter0 to all zeros (accept all IDs)
  // Note: Most libraries have default filters that accept all, but if you have issues, explicitly setting helps.

  CAN.setMode(MCP_NORMAL); // Set to normal mode
  Serial.println("MCP2515 set to Normal Mode. Waiting for CAN messages...");
}

long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

void loop() {
  // Check if a message has been received
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    // Read the message
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    // Print the received message to Serial Monitor
    Serial.print("Received CAN message: ID = 0x");
    Serial.print(rxId, HEX);
    Serial.print(", DLC = ");
    Serial.print(len);
    Serial.print(", Data = ");

    for (int i = 0; i < len; i++) {
      Serial.print("0x");
      if (rxBuf[i] < 0x10) {
        Serial.print("0"); // Pad with leading zero for single-digit hex
      }
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  // Optional: Add a small delay if needed, though CAN reception is interrupt driven
  // delay(10);
}
