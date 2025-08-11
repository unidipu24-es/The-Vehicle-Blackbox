#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// ====================================================================
//                          CONFIGURATION
// ====================================================================

// LoRa Pin Definitions
#define LORA_CS_PIN    5
#define LORA_RST_PIN   14
#define LORA_IRQ_PIN   2
#define LORA_BAND      433E6

// Wi-Fi credentials (REPLACE WITH YOURS)
const char* ssid = "acts";
const char* password = "";

// ThingsBoard and MQTT Configuration (REPLACE WITH YOURS)
const char* thingsboardServer = "demo.thingsboard.io";
const int mqttPort = 1883;
const char* thingsboardToken = "your token"; // <-- PASTE YOUR TOKEN HERE

// Google Geolocation API Key
const char* googleApiKey = "your key";

// Timer for periodic geolocation updates (in milliseconds)
const long GEO_UPDATE_INTERVAL = 30000;

// Cooldown interval for sending LoRa alerts (to prevent spamming)
const long ALERT_COOLDOWN_INTERVAL = 5000;

// New timer for ThingsBoard publishing interval
const long PUBLISH_INTERVAL_MS = 3000; // 3 seconds

// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================

// LoRa
volatile bool loraReceivedFlag = false;
String receivedLoRaMessage = "";

// STM32 data handling
String serialDataString = "";
bool newDataFromSTM32 = false;

// Variables to hold parsed sensor data from STM32
float temp = 0.0;
float humidity = 0.0;
int irState = 0;
int soundValue = 0;
String speed = "";
String accel = "";
String rpm = "";
String idle = "";
String seq = "";

// Timers
unsigned long lastGeoUpdate = 0;
unsigned long lastAlertTime = 0;
unsigned long lastPublishTime = 0; // New timer variable

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ====================================================================
//                          FUNCTION PROTOTYPES
// ====================================================================

// LoRa related functions
void onReceive(int packetSize);
void sendLoRaAlert(String alertMessage);

// STM32 data handling functions
void receiveSTM32Data();
void checkAndProcessSTM32Data();
void parseStm32Data(String data);
String getValueFromData(String data, char separator, String key);
void checkCriticalStatus();

// WiFi and API functions
void connectToWiFi();
void updateGeolocation();

// MQTT related functions
void setupMqtt();
void publishTelemetry();

// ====================================================================
//                          SETUP
// ====================================================================

void setup() 
{
  Serial.begin(115200); 
  while (!Serial);
  Serial.println("\nESP32 Combined Sketch Starting...");

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  Serial.println("\nInitializing LoRa...");
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);
  while (!LoRa.begin(LORA_BAND))
  {
    Serial.println("LoRa initialization failed! Check wiring and module.");
    delay(500);
  }
  
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x42); 
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa Initialized OK and ready to receive!");

  connectToWiFi();
  setupMqtt();
  
  lastGeoUpdate = millis();
  lastPublishTime = millis(); // Initialize the publishing timer
}

// ====================================================================
//                          MAIN LOOP
// ====================================================================

void loop() 
{
  mqttClient.loop();

  if (loraReceivedFlag)
  {
    Serial.println("-------------------------------------------");
    Serial.println("<<< LoRa Packet Received from Node 2 >>>");
    Serial.print("Message: ");
    Serial.println(receivedLoRaMessage);
    Serial.println("-------------------------------------------");
    
    loraReceivedFlag = false;
    receivedLoRaMessage = "";
    LoRa.receive();
  }

  // Check for data from the STM32 via Serial
  receiveSTM32Data();
  
  // If new data is available, process it and store the values
  if (newDataFromSTM32)
  {
    Serial.println("\n\n--- New Data Packet ---");
    parseStm32Data(serialDataString);
    checkCriticalStatus();
    newDataFromSTM32 = false;
    serialDataString = "";
  }
  
  // Publish the latest stored sensor data to ThingsBoard at a fixed interval
  if (millis() - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    publishTelemetry();
    lastPublishTime = millis(); // Reset the publishing timer
  }

  // Periodic Geolocation Update
  if (millis() - lastGeoUpdate >= GEO_UPDATE_INTERVAL) {
    updateGeolocation();
    lastGeoUpdate = millis();
  }
  
  delay(100);
}

// ====================================================================
//                          MQTT FUNCTIONS
// ====================================================================

void setupMqtt() {
  mqttClient.setServer(thingsboardServer, mqttPort);
  while (!mqttClient.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (mqttClient.connect("ESP32_CLIENT", thingsboardToken, NULL)) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishTelemetry() {
  if (mqttClient.connected()) {
    StaticJsonDocument<256> jsonDoc;

    jsonDoc["temperature"] = temp;
    jsonDoc["humidity"] = humidity;
    jsonDoc["irState"] = irState;
    jsonDoc["soundValue"] = soundValue;
    jsonDoc["speed"] = speed.toInt();
    jsonDoc["accel"] = accel.toInt();
    jsonDoc["rpm"] = rpm.toInt();
    jsonDoc["idle"] = idle.toInt();
    jsonDoc["seq"] = seq.toInt();

    char payload[256];
    serializeJson(jsonDoc, payload);
    
    Serial.print("Publishing to ThingsBoard: ");
    Serial.println(payload);

    mqttClient.publish("v1/devices/me/telemetry", payload);
  } else {
    Serial.println("MQTT client not connected. Not publishing telemetry.");
    setupMqtt();
  }
}

// ====================================================================
//                     INTERRUPT SERVICE ROUTINE (ISR)
// ====================================================================

void onReceive(int packetSize)
{
  if (packetSize > 0)
  {
    receivedLoRaMessage = "";
    while (LoRa.available())
    {
      receivedLoRaMessage += (char)LoRa.read();
    }
    loraReceivedFlag = true;
  }
}

void sendLoRaAlert(String alertMessage) {
  if (millis() - lastAlertTime >= ALERT_COOLDOWN_INTERVAL) {
    Serial.println("[Tx LoRa Alert] : " "Vehicle Status: CRITICAL" + alertMessage);
    LoRa.beginPacket();
    LoRa.print(alertMessage);
    LoRa.endPacket();
    lastAlertTime = millis(); 
    LoRa.receive(); 
  }
}

// ====================================================================
//                       STM32 SERIAL DATA FUNCTIONS
// ====================================================================

void receiveSTM32Data()
{
  while (Serial2.available())
  {
    char inChar = Serial2.read();
    if (inChar == '\n')
    {
      Serial.println("-- End of data packet --\n\n");
      newDataFromSTM32 = true;
    }
    else
    {
      serialDataString += inChar;
    }
  }
}

void parseStm32Data(String data) 
{
  String tempStr = getValueFromData(data, ',', "T:");
  String humidityStr = getValueFromData(data, ',', "H:");
  String irStateStr = getValueFromData(data, ',', "IR:");
  String soundValueStr = getValueFromData(data, ',', "S:");
  
  temp = tempStr.toFloat();
  humidity = humidityStr.toFloat();
  irState = irStateStr.toInt();
  soundValue = soundValueStr.toInt();
  speed = getValueFromData(data, ',', "Speed:");
  accel = getValueFromData(data, ',', "Acc:");
  rpm = getValueFromData(data, ',', "RPM:");
  idle = getValueFromData(data, ',', "Idle:");
  seq = getValueFromData(data, ',', "Seq:");
  
  String irStatus = (irState == 0) ? "CLEAR" : "OBSTACLE";

  Serial.println("DHT11 Temp: " + String(temp, 1) + " C");
  Serial.println("DHT11 Hum: " + String(humidity, 1) + " %");
  Serial.println("IR State: " + irStatus);
  Serial.println("Sound Value: " + String(soundValue));
  Serial.println("Speed: " + speed + " km/h");
  Serial.println("Acceleration: " + accel + " km/h/s");
  Serial.println("Braking: NO"); 
  Serial.println("RPM: " + rpm);
  Serial.println("Idling Time: " + idle + " s");
  Serial.println("Sequence: " + seq);
}

void checkCriticalStatus() {
  if (temp > 30.0) {
    sendLoRaAlert("ALERT: High Temperature! Potential fire risk!");
  }
  if (irState == 1) {
    sendLoRaAlert("ALERT: OBSTACLE DETECTED! Potential crash ahead!");
  }
  if (soundValue > 550) {
    sendLoRaAlert("ALERT: CRASH DETECTED! Loud noise detected!");
  }
}

String getValueFromData(String data, char separator, String key) {
  int keyIndex = data.indexOf(key);
  if (keyIndex == -1) return "N/A";
  
  int startIndex = keyIndex + key.length();
  int endIndex = data.indexOf(separator, startIndex);
  if (endIndex == -1) {
    endIndex = data.length();
  }
  return data.substring(startIndex, endIndex);
}

// ====================================================================
//                          WIFI AND API FUNCTIONS
// ====================================================================

void connectToWiFi()
{
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void updateGeolocation() {
  Serial.println("\n--- Current Geolocation ---");
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://www.googleapis.com/geolocation/v1/geolocate?key=";
    url += googleApiKey;

    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int n = WiFi.scanNetworks();
    Serial.print("Scanned ");
    Serial.print(n);
    Serial.println(" networks.");

    String jsonPayload = "{\"wifiAccessPoints\": [";
    for (int i = 0; i < n; i++) {
      jsonPayload += "{";
      jsonPayload += "\"macAddress\": \"" + WiFi.BSSIDstr(i) + "\",";
      jsonPayload += "\"signalStrength\": " + String(WiFi.RSSI(i)) + ",";
      jsonPayload += "\"channel\": " + String(WiFi.channel(i));
      jsonPayload += "}";
      if (i < n - 1) {
        jsonPayload += ",";
      }
    }
    jsonPayload += "]}";
    
    int httpResponseCode = http.POST(jsonPayload);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, response);
      
      if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
      }
      
      double lat = doc["location"]["lat"];
      double lon = doc["location"]["lng"];
      double acc = doc["accuracy"];

      Serial.print("Latitude: ");
      Serial.println(lat, 6);
      Serial.print("Longitude: ");
      Serial.println(lon, 6);
      Serial.print("Accuracy: ");
      Serial.print(acc);
      Serial.println(" meters");
      
    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected. Cannot get geolocation.");
  }
  Serial.println("-----------------------");
}
