#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>       // Include the DHT sensor library
#include <DHT_U.h>     // Include the Adafruit Unified Sensor library (dependency)

// --- CAN BUS DEFINITIONS ---
#define CAN_CS_PIN 10  // Chip Select pin for MCP2515 (e.g., D10 on Arduino Uno)
MCP2515 mcp2515(CAN_CS_PIN);

// --- SENSOR PIN DEFINITIONS ---
// DHT11 Sensor
#define DHTPIN 4       // Digital pin for DHT11 sensor
#define DHTTYPE DHT11  // DHT 11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// PIR Motion Sensor
#define PIR_SENSOR_PIN 5 // Signal pin of PIR sensor

// Sound Sensor
#define SOUND_SENSOR_PIN 8 // Signal pin of sound sensor

// Buzzer
#define BUZZER_PIN 3   // Digital pin for buzzer

// LED (from the second code, but not explicitly used in output)
// const int led = 9; // Not used in this merged code for CAN output, kept for completeness if needed.

// --- CAN MESSAGE IDs ---
#define CAN_ID_DHT11   0x124
#define CAN_ID_PIR     0x125
#define CAN_ID_SOUND   0x126

// --- SENSOR STATE VARIABLES ---
int pirState = LOW; // Current state of the PIR sensor
int prevPirState = LOW; // Previous state to detect changes

unsigned long lastSoundEvent = 0; // Timestamp of the last sound detection
const unsigned long soundDebounceTime = 25; // Debounce time for sound sensor (milliseconds)

unsigned long lastDHTReadTime = 0;
const unsigned long dhtReadInterval = 2000; // DHT11 can only be read every 2 seconds

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Arduino Multi-Sensor CAN Sender");

  // --- Initialize MCP2515 ---
  if (mcp2515.reset() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } 
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
  Serial.println("CAN Bitrate Set to 500 KBPS");
}
  else {
    Serial.println("Error Initializing MCP2515...");
    while (1); // Halt if MCP2515 fails to initialize
  }

  // Set CAN bitrate (MUST match receiver's bitrate, e.g., STM32's 500 KBPS)
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    Serial.println("CAN Bitrate Set to 500 KBPS");
  } else {
    Serial.println("Error Setting CAN Bitrate...");
    while (1); // Halt if CAN bitrate fails to set
  }

  // Set MCP2515 to NORMAL mode for actual CAN bus communication
  if (mcp2515.setNormalMode() == MCP2515::ERROR_OK) {
    Serial.println("MCP2515 in Normal Mode");
  } else {
    Serial.println("Error Setting Normal Mode...");
    while (1); // Halt if CAN mode fails to set
  }

  // --- Initialize Sensors ---
  dht.begin(); // Initialize DHT sensor
  Serial.println("DHT11 Sensor Initialized!");

  pinMode(PIR_SENSOR_PIN, INPUT);    // PIR motion sensor is an input
  pinMode(SOUND_SENSOR_PIN, INPUT);  // Sound sensor is an input
  pinMode(BUZZER_PIN, OUTPUT);       // Buzzer is an output
  // pinMode(led, OUTPUT); // If you want to use the LED
  Serial.println("Other Sensors and Buzzer Initialized!");
}

void loop() {
  // --- Read and Send DHT11 Data ---
  if (millis() - lastDHTReadTime >= dhtReadInterval) {
    lastDHTReadTime = millis(); // Update last read time

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      // Optionally, send an error CAN message or skip this data
    } else {
      struct can_frame txMsgDHT;
      txMsgDHT.can_id = CAN_ID_DHT11;
      txMsgDHT.can_dlc = 4;

      // Convert float temperature/humidity to integers (scaled by 100)
      int16_t temperature_int = (int16_t)(t * 100);
      int16_t humidity_int = (int16_t)(h * 100);

      txMsgDHT.data[0] = (temperature_int >> 8) & 0xFF; // High byte
      txMsgDHT.data[1] = temperature_int & 0xFF;        // Low byte
      txMsgDHT.data[2] = (humidity_int >> 8) & 0xFF;    // High byte
      txMsgDHT.data[3] = humidity_int & 0xFF;           // Low byte

      if (mcp2515.sendMessage(&txMsgDHT) == MCP2515::ERROR_OK) {
        Serial.print("Sent DHT11 CAN msg ID: 0x");
        Serial.print(txMsgDHT.can_id, HEX);
        Serial.print(", Temp: ");
        Serial.print(t);
        Serial.print(" C, Hum: ");
        Serial.print(h);
        Serial.println(" %");
      } else {
        Serial.println("Error sending DHT11 CAN message.");
      }
    }
  }

  // --- Read and Send PIR Sensor Data ---
  pirState = digitalRead(PIR_SENSOR_PIN);

  if (pirState == HIGH && prevPirState == LOW) {
    // Motion detected (rising edge)
    Serial.println("Motion detected!");
    digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
    delay(1000); // Buzzer on for 690ms
    digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer

    struct can_frame txMsgPIR;
    txMsgPIR.can_id = CAN_ID_PIR;
    txMsgPIR.can_dlc = 1;
    txMsgPIR.data[0] = 1; // 1 indicates motion detected

    if (mcp2515.sendMessage(&txMsgPIR) == MCP2515::ERROR_OK) {
      Serial.print("Sent PIR CAN msg ID: 0x");
      Serial.print(txMsgPIR.can_id, HEX);
      Serial.println(", Data: Motion Detected");
    } else {
      Serial.println("Error sending PIR CAN message.");
    }
  } else if (pirState == LOW && prevPirState == HIGH) {
    // Motion stopped (falling edge)
    Serial.println("Motion stopped.");
    // No buzzer for motion stopped in the original code, so not adding here.

    struct can_frame txMsgPIR;
    txMsgPIR.can_id = CAN_ID_PIR;
    txMsgPIR.can_dlc = 1;
    txMsgPIR.data[0] = 0; // 0 indicates no motion

    if (mcp2515.sendMessage(&txMsgPIR) == MCP2515::ERROR_OK) {
      Serial.print("Sent PIR CAN msg ID: 0x");
      Serial.print(txMsgPIR.can_id, HEX);
      Serial.println(", Data: No Motion");
    } else {
      Serial.println("Error sending PIR CAN message.");
    }
  }
  prevPirState = pirState; // Update previous state for next loop

  // --- Read and Send Sound Sensor Data ---
  int soundSensorData = digitalRead(SOUND_SENSOR_PIN);

  if (soundSensorData == HIGH) { // If pin goes LOW, sound is detected
    if (millis() - lastSoundEvent > soundDebounceTime) {
      // Sound detected after debounce period
      Serial.println("Sound detected!"); // Retained original message
      digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
      delay(1000); // Buzzer on for 690ms
      digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer

      struct can_frame txMsgSound;
      txMsgSound.can_id = CAN_ID_SOUND;
      txMsgSound.can_dlc = 1;
      txMsgSound.data[0] = 1; // 1 indicates sound detected

      if (mcp2515.sendMessage(&txMsgSound) == MCP2515::ERROR_OK) {
        Serial.print("Sent Sound CAN msg ID: 0x");
        Serial.print(txMsgSound.can_id, HEX);
        Serial.println(", Data: Sound Detected");
      } else {
        Serial.println("Error sending Sound CAN message.");
      }
    }
    lastSoundEvent = millis(); // Remember when last event happened
  }
  
  // A small delay to prevent rapid looping and excessive CAN messages,
  // especially if no motion or sound is continuously detected.
  // This delay can be adjusted or removed if non-blocking polling is strictly required.
  // However, for digital sensors that trigger on state changes, this is less critical.
  delay(10); 
}
