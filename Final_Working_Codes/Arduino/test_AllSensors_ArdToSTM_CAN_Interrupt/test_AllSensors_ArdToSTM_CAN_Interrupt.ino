#include <SPI.h>
#include <mcp_can.h>
#include "DHT.h" // Include DHT sensor library

// Sensor & Buzzer Pin Definitions
#define DHTPIN 7        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
#define IR_SENSOR_PIN 6 // Digital pin connected to IR sensor OUT pin
#define BUZZER_PIN 8    // Digital pin connected to active buzzer
#define SOUND_SENSOR_PIN A0 // Analog pin connected to LM393 Sound sensor A0 pin

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// MCP2515 CAN Controller Definitions
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

// CAN message variables for transmission
long unsigned int txId = 0x457; // Standard CAN ID for combined sensor data
unsigned char txData[8]; // Data buffer: 8 bytes for Temp, Hum, IR, Sound, Sequence
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // Send data every 1000 ms (1 second)

unsigned int sequenceNum = 0; // Sequence number for debugging

// Global variables to store sensor readings
float current_temperature = 0.0f;
float current_humidity = 0.0f;
int current_ir_state = 0; // 0: No obstacle, 1: Obstacle
int current_sound_value = 0; // 0-1023 analog value

void setup() {
  Serial.begin(115200); // For debugging output to Arduino Serial Monitor
  while (!Serial); // Wait for serial port to connect. Needed for native USB port only

  // Configure sensor pins
  pinMode(IR_SENSOR_PIN, INPUT); // IR sensor is an input
  pinMode(BUZZER_PIN, OUTPUT);   // Buzzer is an output

  Serial.println("Initializing DHT11 Sensor...");
  dht.begin(); // Start DHT sensor

  Serial.println("Initializing MCP2515 CAN controller for Multi-Sensor Data TX...");

  // Initialize CAN at 500 kbps (match STM32's speed)
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("MCP2515 Initialization Failed!");
    Serial.println("Check connections and crystal frequency.");
    while(1); // Halt if initialization fails
  }

  CAN.setMode(MCP_NORMAL); // Set to normal mode
  Serial.println("MCP2515 set to Normal Mode. Sending Multi-Sensor data via CAN...");

  // Initial state for buzzer
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
}

void loop() {
  unsigned long currentMillis = millis();

  // Read sensor data and send via CAN periodically
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;

    // --- Read Sensor Data ---
    // Read DHT11
    current_humidity = dht.readHumidity();
    current_temperature = dht.readTemperature(); // Default is Celsius

    // Check if DHT reads failed
    if (isnan(current_humidity) || isnan(current_temperature)) {
      Serial.println(F("Failed to read from DHT sensor! Retrying..."));
      // We'll still send other sensor data if DHT fails for this cycle
      current_humidity = 0.0f; // Set to 0 or last known good value
      current_temperature = 0.0f; // Set to 0 or last known good value
    }

    // Read IR sensor (digital)
    // IR sensor typically outputs LOW when obstacle detected, HIGH when clear
    current_ir_state = !digitalRead(IR_SENSOR_PIN); // Invert logic: 1 if obstacle, 0 if clear

    // Read Sound sensor (analog)
    current_sound_value = analogRead(SOUND_SENSOR_PIN);

    // --- Control Buzzer based on local conditions ---
    bool activateBuzzer = false;

    // Condition 1: IR sensor detects obstacle
    if (current_ir_state == 1) { // 1 means obstacle detected
      activateBuzzer = true;
      Serial.println("LOCAL: IR Obstacle Detected! Buzzer ON.");
    }

    // Condition 2: DHT11 temperature rises > 30C
    if (current_temperature > 30.0f) {
      activateBuzzer = true;
      Serial.println("LOCAL: High Temperature Detected! Buzzer ON.");
    }
    
    // Set buzzer state
    digitalWrite(BUZZER_PIN, activateBuzzer ? HIGH : LOW);


    // --- Prepare CAN data ---
    // Pack temperature and humidity
    int temp_int = (int)current_temperature;
    int temp_frac = (int)((current_temperature - temp_int) * 10);
    int hum_int = (int)current_humidity;
    int hum_frac = (int)((current_humidity - hum_int) * 10);

    // Ensure fractional parts are 0-9
    temp_frac = constrain(temp_frac, 0, 9);
    hum_frac = constrain(hum_frac, 0, 9);

    txData[0] = (uint8_t)temp_int;
    txData[1] = (uint8_t)temp_frac;
    txData[2] = (uint8_t)hum_int;
    txData[3] = (uint8_t)hum_frac;

    // Pack IR sensor state
    txData[4] = (uint8_t)current_ir_state; // 0 or 1

    // Pack Sound sensor value (10-bit value into 2 bytes)
    txData[5] = (uint8_t)(current_sound_value & 0xFF); // Low byte
    txData[6] = (uint8_t)((current_sound_value >> 8) & 0xFF); // High byte

    // Pack Sequence Number
    txData[7] = (uint8_t)(sequenceNum % 256);

    // Send the CAN message
    byte sndStat = CAN.sendMsgBuf(txId, 0, sizeof(txData), txData); // 0 for standard ID

    if (sndStat == CAN_OK) {
      Serial.print("CAN TX Success! ID: 0x"); Serial.print(txId, HEX);
      Serial.print(" T:"); Serial.print(current_temperature, 1);
      Serial.print(" H:"); Serial.print(current_humidity, 1);
      Serial.print(" IR:"); Serial.print(current_ir_state);
      Serial.print(" Sound:"); Serial.print(current_sound_value);
      Serial.print(" Seq:"); Serial.print(sequenceNum); Serial.println();
    } else {
      Serial.print("CAN TX Failed! Error: ");
      Serial.println(sndStat); // Print error code
    }

    sequenceNum++; // Increment sequence number for next message
  }
}
