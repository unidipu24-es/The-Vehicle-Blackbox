#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h> // For parsing the JSON response from the API

// ====================================================================
//                          CONFIGURATION
// ====================================================================

// LoRa Pin Definitions
// Ensure these match your physical wiring for the Node 1 ESP32
#define LORA_CS_PIN    5   // Chip Select (NSS)
#define LORA_RST_PIN   14  // Reset (RST)
#define LORA_IRQ_PIN   2   // Interrupt Request (DIO0)

// Define the LoRa frequency (e.g., 433E6, 868E6, 915E6)
#define LORA_BAND      433E6 // 433 MHz band

// Wi-Fi credentials (replace with your own)
const char* ssid = "acts";
const char* password = "";

// Google Geolocation API Key
const char* googleApiKey = "your key";

// Timer for periodic geolocation updates (in milliseconds)
const long GEO_UPDATE_INTERVAL = 30000; // 30 seconds

// Timer for debugging STM32 serial data (in milliseconds)
const long STM32_DEBUG_INTERVAL = 5000; // 5 seconds

// Cooldown interval for sending LoRa alerts (to prevent spamming)
const long ALERT_COOLDOWN_INTERVAL = 5000; // 5 seconds

// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================

// Use 'volatile' for variables modified inside an ISR
volatile bool loraReceivedFlag = false;
String receivedLoRaMessage = "";

// Variables for parsing STM32 data
String serialDataString = "";

// Flag to indicate new data is available from STM32
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

// Timer variable for geolocation updates
unsigned long lastGeoUpdate = 0;

// Timer variable for STM32 serial debug messages
unsigned long lastSTM32Debug = 0;

// Timer variable for LoRa alert cooldown
unsigned long lastAlertTime = 0;

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

// ====================================================================
//                          SETUP
// ====================================================================

void setup() 
{
  // Initialize the main serial port for debugging output to the computer
  Serial.begin(115200); 
  while (!Serial);
  Serial.println("\nESP32 Combined Sketch Starting...");

  // Initialize a separate serial port (UART2) to receive data from the STM32.
  // Make sure to connect the STM32's TX pin to ESP32 GPIO16 (RX2).
  // The baud rate must match the STM32's baud rate.
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  // Initialize LoRa
  Serial.println("\nInitializing LoRa...");
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);
  while (!LoRa.begin(LORA_BAND))
  {
    Serial.println("LoRa initialization failed! Check wiring and module.");
    delay(500);
  }
  
  // Set the LoRa parameters to match Node 2
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x42); 

  // Attach the interrupt handler for LoRa packet reception
  LoRa.onReceive(onReceive);
  LoRa.receive(); // Start receiving packets
  Serial.println("LoRa Initialized OK and ready to receive!");

  // Connect to WiFi
  connectToWiFi();
  
  // Initialize the timers
  lastGeoUpdate = millis();
  lastSTM32Debug = millis();
}

// ====================================================================
//                          MAIN LOOP
// ====================================================================

void loop() 
{
  // -------------------------------------------------------------
  // Section 1: Check for LoRa packets (using the interrupt flag)
  // -------------------------------------------------------------
  if (loraReceivedFlag)
  {
    Serial.println("-------------------------------------------");
    Serial.println("<<< LoRa Packet Received from Node 2 >>>");
    Serial.print("Message: ");
    Serial.println(receivedLoRaMessage);
    Serial.println("-------------------------------------------");
    
    // Process the data here.
    // parseLoRaData(receivedLoRaMessage);
    
    // Reset the flag and clear the message for the next packet
    loraReceivedFlag = false;
    receivedLoRaMessage = "";
    
    // IMPORTANT: Re-enter receive mode after handling a packet
    LoRa.receive();
  }

  // -------------------------------------------------------------
  // Section 2: Check for data from the STM32 via Serial
  // -------------------------------------------------------------
  receiveSTM32Data();
  checkAndProcessSTM32Data();
  
  // -------------------------------------------------------------
  // Section 3: Periodic Geolocation Update
  // -------------------------------------------------------------
  if (millis() - lastGeoUpdate >= GEO_UPDATE_INTERVAL) {
    updateGeolocation();
    lastGeoUpdate = millis();
  }
  
  // -------------------------------------------------------------
  // Section 4: Debug message for STM32 serial data
  // -------------------------------------------------------------
  if (millis() - lastSTM32Debug >= STM32_DEBUG_INTERVAL) {
    if (!newDataFromSTM32) {
      //Serial.println("Debug: No new STM32 data received...");
    }
    lastSTM32Debug = millis();
  }
  
  delay(100);
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
  // Check if enough time has passed since the last alert
  if (millis() - lastAlertTime >= ALERT_COOLDOWN_INTERVAL) {
    Serial.println("[Tx LoRa Alert] : " "Vehicle Status: CRITICAL" + alertMessage);
    LoRa.beginPacket();
    LoRa.print(alertMessage);
    LoRa.endPacket();
    lastAlertTime = millis(); // Reset the alert timer
    
    // IMPORTANT: Put LoRa back in receive mode after sending a packet
    LoRa.receive(); 
  }
}

// ====================================================================
//                       STM32 SERIAL DATA FUNCTIONS
// ====================================================================

void receiveSTM32Data()
{
  // Now reading from Serial2, which is connected to the STM32
  while (Serial2.available())
  {
    char inChar = Serial2.read();
    // This is a crucial check. The STM32 data must end with a newline '\n'
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

void checkAndProcessSTM32Data()
{
  if (newDataFromSTM32)
  {
    Serial.print("Parsing raw string: ");
    Serial.println(serialDataString);
    Serial.println("\n\n--- New Data Packet ---");
    parseStm32Data(serialDataString); // Call the parsing function
    
    // Check for critical status after parsing the data
    checkCriticalStatus();

    // Reset the flag and clear the string
    newDataFromSTM32 = false;
    serialDataString = "";
  }
}

void parseStm32Data(String data) 
{
  String tempStr = getValueFromData(data, ',', "T:");
  String humidityStr = getValueFromData(data, ',', "H:");
  String irStateStr = getValueFromData(data, ',', "IR:");
  String soundValueStr = getValueFromData(data, ',', "S:");
  
  // Update global variables
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
  // Check for high temperature
  if (temp > 30.0) {
    sendLoRaAlert("ALERT: High Temperature! Potential fire risk!");
  }
  
  // Check for obstacle detected by IR sensor
  if (irState == 1) {
    sendLoRaAlert("ALERT: OBSTACLE DETECTED! Potential crash ahead!");
  }

  // Check for high sound level (potential crash)
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

    // Perform a WiFi scan to get real access point data
    int n = WiFi.scanNetworks();
    Serial.print("Scanned ");
    Serial.print(n);
    Serial.println(" networks.");

    // Build the JSON payload dynamically
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
