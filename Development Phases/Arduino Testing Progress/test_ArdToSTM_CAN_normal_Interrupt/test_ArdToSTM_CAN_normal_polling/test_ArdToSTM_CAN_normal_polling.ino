#include <SPI.h>
#include <mcp_can.h>

// Set CS pin (usually D10 on popular shields)
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

long unsigned int txId = 0x321; // Standard CAN ID for transmission (from Arduino)
unsigned char txData[8];
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // Send a message every 1000 ms (1 second)

void setup() {
  Serial.begin(115200); // For debugging output to Arduino Serial Monitor
  while (!Serial); // Wait for serial port to connect. Needed for native USB port only

  Serial.println("Initializing MCP2515 CAN controller for TX...");

  // Initialize CAN at 500 kbps (match STM32's speed)
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("MCP2515 Initialization Failed!");
    Serial.println("Check connections and crystal frequency.");
    while(1); // Halt if initialization fails
  }

  CAN.setMode(MCP_NORMAL); // Set to normal mode
  Serial.println("MCP2515 set to Normal Mode. Sending CAN messages...");

  // Initialize data for first transmission
  for (int i = 0; i < 8; i++) {
    txData[i] = i; // Initial data: 0x00, 0x01, ..., 0x07
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Send a message every 'sendInterval' milliseconds
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;

    // Send the CAN message
    // CAN.sendMsgBuf(id, ide, len, buf);
    // id: CAN ID
    // ide: CAN_STDID (standard ID) or CAN_EXTID (extended ID)
    // len: Data Length Code (0-8)
    // buf: Pointer to data array
    byte sndStat = CAN.sendMsgBuf(txId, 0, 8, txData); // 0 for standard ID

    if (sndStat == CAN_OK) {
      Serial.print("CAN TX Success! ID: 0x");
      Serial.print(txId, HEX);
      Serial.print(", Data: ");
      for (int i = 0; i < 8; i++) {
        Serial.print("0x");
        if (txData[i] < 0x10) Serial.print("0");
        Serial.print(txData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.print("CAN TX Failed! Error: ");
      Serial.println(sndStat); // Print error code (e.g., 1 for failed, 2 for timeout)
    }

    // Increment data for next transmission
    for (int i = 0; i < 8; i++) {
      txData[i]++;
    }
  }

  // Remove previous RX logic, as Arduino is primarily a sender now.
  // If you want to keep RX for future full-duplex, you can add it back later.
}
