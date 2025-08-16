#include <WiFi.h> // For ESP32, use WiFi.h
#include <HTTPClient.h> // For ESP32, use HTTPClient.h
#include <WiFiClientSecure.h>
#include <ArduinoJson.h> // Use the modern ArduinoJson library

// WiFi Credentials
const char* ssid = "joker";
const char* password = "joker124";

// Google Geolocation API Credentials
const char* G_API_KEY = "your key"; // Replace with your actual API key
const char* G_API_HOST = "www.googleapis.com";
const String G_API_URL = "/geolocation/v1/geolocate?key=";

// Create a WiFiClientSecure object
WiFiClientSecure client;

void setup() {
  Serial.begin(115200); // Use a faster baud rate for ESP32
  delay(10);

  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Allow insecure connections for simplicity. For production, use certificates.
  // client.setFingerprint(fingerprint); or client.setCACert(root_ca);
  client.setInsecure();
}

void loop() {
  Serial.println("\nScanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  Serial.print(n);
  Serial.println(" networks found.");

  if (n == 0) {
    Serial.println("No networks found. Cannot geolocate.");
  } else {
    //
    // Create JSON request body
    //
    JsonDocument doc; // Using modern ArduinoJson (v6+)

    // Add nearby WiFi access points to the JSON document
    JsonArray wifiAccessPoints = doc["wifiAccessPoints"].to<JsonArray>();
    for (int i = 0; i < n; ++i) {
      JsonObject ap = wifiAccessPoints.add<JsonObject>();
      ap["macAddress"] = WiFi.BSSIDstr(i);
      ap["signalStrength"] = WiFi.RSSI(i);
    }

    // Serialize JSON to a string
    String jsonRequest;
    serializeJson(doc, jsonRequest);
    
    Serial.println("Sending request to Google Geolocation API:");
    Serial.println(jsonRequest);

    //
    // Make the HTTPS POST request
    //
    HTTPClient https;
    // For ESP32, the begin signature for WiFiClientSecure is slightly different if you include the port directly.
    // It's often recommended to use the host and path separately, or let HTTPClient handle the default port 443.
    // However, your current begin(client, G_API_HOST, 443, G_API_URL + String(G_API_KEY), true) should work fine.
    // A more common ESP32 style might be:
    // https.begin(client, String("https://") + G_API_HOST + G_API_URL + String(G_API_KEY));
    // But your original begin call is also valid.
    if (https.begin(client, G_API_HOST, 443, G_API_URL + String(G_API_KEY), true)) {
      https.addHeader("Content-Type", "application/json");
      int httpCode = https.POST(jsonRequest);

      if (httpCode > 0) 
      {
        Serial.printf("[HTTPS] POST... code: %d\n", httpCode);

        if (httpCode == HTTP_CODE_OK) {
          String payload = https.getString();
          Serial.println("Received response:");
          Serial.println(payload);
          
          //
          // Parse the JSON response
          //
          JsonDocument filter;
          filter["location"]["lat"] = true;
          filter["location"]["lng"] = true;
          filter["accuracy"] = true;

          JsonDocument jsonResponse;
          DeserializationError error = deserializeJson(jsonResponse, payload, DeserializationOption::Filter(filter));
          
          if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
          } else {
            double latitude  = jsonResponse["location"]["lat"];
            double longitude = jsonResponse["location"]["lng"];
            double accuracy  = jsonResponse["accuracy"];

            Serial.println("--------------------");
            Serial.print("Latitude: ");
            Serial.println(latitude, 6);
            Serial.print("Longitude: ");
            Serial.println(longitude, 6);
            Serial.print("Accuracy (meters): ");
            Serial.println(accuracy);
            Serial.println("--------------------");
          }

        }
      } 
      
      else {
        Serial.printf("[HTTPS] POST... failed, error: %s\n", https.errorToString(httpCode).c_str());
      }
      https.end();
    } else {
      Serial.printf("[HTTPS] Unable to connect\n");
    }
  }

  // Wait for 5 minutes before the next request to avoid spamming the API
  Serial.println("\nWaiting for 3 seconds...");
  delay(3000); 
}
