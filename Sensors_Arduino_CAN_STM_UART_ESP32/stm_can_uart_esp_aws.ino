#include <Arduino.h>
#include <WiFi.h> // Keep WiFi for general connectivity, though not strictly needed for serial output only
#include <ArduinoJson.h> // For JSON serialization

// =============================================================================
//                           WiFi Configuration (Optional)
// =============================================================================
// While WiFi is not strictly necessary for serial output,
// I've kept the WiFi library include and basic connection function
// in case you want to easily re-enable it for other purposes later.
// If you only need serial communication, you can remove these lines.
const char* WIFI_SSID = "vivoV29";
const char* WIFI_PASSWORD = "12345678";

// =============================================================================
//                           ESP32 UART Configuration
// =============================================================================

// Define which UART peripheral to use on ESP32 for STM32 communication
// UART0 is usually used for USB-to-serial monitor (Serial)
// UART1 TX: GPIO10, RX: GPIO9 (ESP32 DevKitC)
// UART2 TX: GPIO17, RX: GPIO16 (ESP32 DevKitC)
#define RXD2 16 // GPIO16 for Serial2 RX (connects to STM32 TX)
#define TXD2 17 // GPIO17 for Serial2 TX (connects to STM32 RX)

// =============================================================================
//                           Global Variables
// =============================================================================

char receivedChar;
String receivedDataBuffer = ""; // Buffer to accumulate received characters
bool newDataAvailable = false;  // Flag to indicate a complete message is received

// =============================================================================
//                           Function Prototypes
// =============================================================================
void connectWiFi(); // Optional: Function to connect to WiFi
void parseAndPrintToSerial(String& dataString);

// =============================================================================
//                               Setup Function
// =============================================================================
void setup() {
    // Initialize Serial Monitor (UART0) for debugging and output
    Serial.begin(115200);
    Serial.println("ESP32: Starting up...");

    // Initialize UART2 for communication with STM32
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Baud rate, Data format, RX pin, TX pin
    Serial.println("ESP32: Initialized Serial2 for STM32 communication.");

    // Optional: Connect to WiFi if needed for other purposes (uncomment if desired)
    // connectWiFi();
}

// =============================================================================
//                               Loop Function
// =============================================================================
void loop() {
    // Receive data from STM32 via Serial2
    while (Serial2.available()) {
        receivedChar = Serial2.read();
        if (receivedChar == '\n') { // Check for newline character as end of message
            newDataAvailable = true;
            break; // Exit loop after newline, process message
        } else if (receivedChar != '\r') { // Ignore carriage return
            receivedDataBuffer += receivedChar;
        }
    }

    // If a complete message is received, parse and print to Serial
    if (newDataAvailable) {
        Serial.print("ESP32: Received from STM32: ");
        Serial.println(receivedDataBuffer);
        parseAndPrintToSerial(receivedDataBuffer); // Process and print the data

        receivedDataBuffer = ""; // Clear buffer for next message
        newDataAvailable = false;
    }

    // Small delay to prevent busy-waiting
    delay(10);
}

// =============================================================================
//                           Helper Functions
// =============================================================================

/**
 * @brief Connects to the configured WiFi network.
 * This function is optional and can be removed if WiFi is not needed.
 */
void connectWiFi() {
    Serial.print("ESP32: Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nESP32: WiFi connected.");
    Serial.print("ESP32: IP Address: ");
    Serial.println(WiFi.localIP());
}

/**
 * @brief Parses the received string from STM32 and prints it to Serial Monitor as JSON.
 * @param dataString The raw string received from STM32.
 */
void parseAndPrintToSerial(String& dataString) 
{
    // Create a StaticJsonDocument to hold the JSON data
    // The size (200) should be adjusted based on the maximum expected JSON size
    StaticJsonDocument<200> doc;

    doc["timestamp"] = millis(); // Add a timestamp

    if (dataString.startsWith("DHT11 ID:")) {
        doc["sensorType"] = "DHT11";
        // Example: "DHT11 ID: 0x124, Temp: 25.50 C, Hum: 60.20 %"
        int tempIndex = dataString.indexOf("Temp: ");
        int humIndex = dataString.indexOf("Hum: ");
        int cIndex = dataString.indexOf(" C,");
        int percentIndex = dataString.indexOf(" %");

        if (tempIndex != -1 && humIndex != -1 && cIndex != -1 && percentIndex != -1) {
            String tempStr = dataString.substring(tempIndex + strlen("Temp: "), cIndex);
            String humStr = dataString.substring(humIndex + strlen("Hum: "), percentIndex);
            doc["temperature"] = tempStr.toFloat();
            doc["humidity"] = humStr.toFloat();
        } else {
            doc["error"] = "DHT11 parsing error";
        }
    } else if (dataString.startsWith("PIR ID:")) {
        doc["sensorType"] = "PIR";
        // Example: "PIR ID: 0x125, Status: Motion Detected"
        int statusIndex = dataString.indexOf("Status: ");
        if (statusIndex != -1) {
            String statusStr = dataString.substring(statusIndex + strlen("Status: "));
            statusStr.trim(); // Remove leading/trailing whitespace
            doc["status"] = statusStr;
            doc["motionDetected"] = (statusStr == "Motion Detected");
        } else {
            doc["error"] = "PIR parsing error";
        }
    } else if (dataString.startsWith("SOUND ID:")) {
        doc["sensorType"] = "SOUND";
        // Example: "SOUND ID: 0x126, Status: Sound Detected"
        int statusIndex = dataString.indexOf("Status: ");
        if (statusIndex != -1) {
            String statusStr = dataString.substring(statusIndex + strlen("Status: "));
            statusStr.trim();
            doc["status"] = statusStr;
            doc["soundDetected"] = (statusStr == "Sound Detected");
        } else {
            doc["error"] = "SOUND parsing error";
        }
    } 
    
    else if (dataString.startsWith("CAN ID:")) {
        doc["sensorType"] = "GenericCAN";
        // Example: "CAN ID: 0x123, DLC: 8, Data: 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08"
        doc["rawData"] = dataString; // Send the raw string if not specifically parsed
        // You could add more sophisticated parsing here if needed for generic CAN data
    } 
    
    else {
        doc["sensorType"] = "Unknown";
        doc["rawData"] = dataString;
    }

    // Serialize the JSON document to a string and print it to Serial
    char jsonBuffer[256]; // Adjust buffer size as needed
    serializeJson(doc, jsonBuffer);

    Serial.print("ESP32: Parsed JSON output: ");
    Serial.println(jsonBuffer);
}
