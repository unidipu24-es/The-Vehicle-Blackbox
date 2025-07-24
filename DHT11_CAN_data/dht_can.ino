#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>       // Include the DHT sensor library
#include <DHT_U.h>     // Include the Adafruit Unified Sensor library (dependency)

// Define the CS pin for MCP2515 (e.g., D10 on Arduino Uno)
MCP2515 mcp2515(10);

// Define DHT sensor pin and type
#define DHTPIN 4       // <-- *** CHANGE THIS TO DIGITAL PIN 4 ***
#define DHTTYPE DHT11  // DHT 11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Arduino CAN Sender with DHT11 Data");

  // Initialize MCP2515
  if (mcp2515.reset() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }

  // Set CAN bitrate (MUST match STM32's 500 KBPS)
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    Serial.println("CAN Bitrate Set to 500 KBPS");
  } else {
    Serial.println("Error Setting CAN Bitrate...");
    while (1);
  }

  // Set MCP2515 to NORMAL mode for actual CAN bus communication
  if (mcp2515.setNormalMode() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 in Normal Mode");
  } else {
    Serial.println("Error Setting Normal Mode...");
    while (1);
  }

  dht.begin(); // Initialize DHT sensor
  Serial.println("DHT11 Sensor Initialized!");
}

void loop() {
  // Wait a few seconds between measurements.
  // DHT11 can only be read every 2 seconds.
  delay(2000);

  // Read humidity
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to retry).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    // You might want to send an error message over CAN or skip sending
    return; // Skip this loop iteration and try again
  }

  // --- Prepare and Send DHT11 Data CAN Message ---
  struct can_frame txMsg;

  txMsg.can_id = 0x124; // New CAN ID for DHT11 data
  txMsg.can_dlc = 4;    // Data Length Code (we'll send 2 bytes for temp, 2 for humidity)

  // Convert float temperature/humidity to integers or scaled integers
  // A common approach is to multiply by 100 to retain two decimal places
  // and then cast to int16_t (signed 16-bit integer).
  int16_t temperature_int = (int16_t)(t * 100);
  int16_t humidity_int = (int16_t)(h * 100);

  // Pack the 16-bit integers into 8-bit bytes for the CAN frame
  txMsg.data[0] = (temperature_int >> 8) & 0xFF; // High byte
  txMsg.data[1] = temperature_int & 0xFF;        // Low byte
  txMsg.data[2] = (humidity_int >> 8) & 0xFF;    // High byte
  txMsg.data[3] = humidity_int & 0xFF;           // Low byte

  // Send the message
  if (mcp2515.sendMessage(&txMsg) == MCP2515::ERROR_OK)
  {
    Serial.print("Sent DHT11 CAN msg ID: 0x");
    Serial.print(txMsg.can_id, HEX);
    Serial.print(", Temp: ");
    Serial.print(t);
    Serial.print(" C, Hum: ");
    Serial.print(h);
    Serial.println(" %");
  }
  else
  {
    Serial.println("Error sending DHT11 CAN message.");
  }
}
