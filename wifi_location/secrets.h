// secrets.h
// This file contains all sensitive credentials and certificates.
// DO NOT SHARE THIS FILE PUBLICLY OR COMMIT TO VERSION CONTROL.

#ifndef SECRETS_H
#define SECRETS_H

// --- WiFi Credentials ---
// Replace with your WiFi network SSID (name)
const char* SSID = "joker";
// Replace with your WiFi network password
const char* PASSWORD = "joker124";

// --- Google Geolocation API Credentials ---
// Replace with your Google Geolocation API Key
const char* G_API_KEY = "YOUR_API_KEY";
// Google Geolocation API Host (usually remains constant)
const char* G_API_HOST = "www.googleapis.com";
// Google Geolocation API URL path (usually remains constant)
const String G_API_URL = "/geolocation/v1/geolocate?key=";

// --- AWS IoT Core Configuration ---
// IMPORTANT: Replace with your AWS IoT Core endpoint.
// Example: "a123example456-ats.iot.us-east-1.amazonaws.com"
const char* AWS_IOT_ENDPOINT = "YOUR_AWS_IOT_ENDPOINT.iot.your-region.amazonaws.com";
// IMPORTANT: Replace with the exact name of your AWS IoT Thing.
// Example: "ESP8266_Geolocation_Device"
const char* AWS_IOT_THING_NAME = "YOUR_THING_NAME";
// MQTT Topic to publish geolocation data to. You can change this if needed.
const char* AWS_IOT_PUBLISH_TOPIC = "esp8266/geolocation";

// --- AWS IoT Core Certificates ---
// IMPORTANT: Replace with the entire content of your Amazon Root CA 1 certificate.
// This includes the BEGIN and END CERTIFICATE lines.
const char* AWS_CERT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAqGgAwIBAgITBmyfz5m/jAo54vJ4obndUoQfRwMA
... (PASTE YOUR AMAZON ROOT CA 1 CONTENT HERE) ...
-----END CERTIFICATE-----
)EOF";

// IMPORTANT: Replace with the entire content of your device certificate (.pem.crt file).
// This includes the BEGIN and END CERTIFICATE lines.
const char* AWS_CERT_CRT = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUW6... (PASTE YOUR DEVICE CERTIFICATE CONTENT HERE) ...
-----END CERTIFICATE-----
)EOF";

// IMPORTANT: Replace with the entire content of your private key file (.pem.key file).
// This includes the BEGIN and END RSA PRIVATE KEY lines.
const char* AWS_CERT_PRIVATE = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggS... (PASTE YOUR PRIVATE KEY CONTENT HERE) ...
-----END RSA PRIVATE KEY-----
)EOF";

#endif // SECRETS_H

