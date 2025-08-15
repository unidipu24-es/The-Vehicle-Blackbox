#include <LoRa.h>
#include <SPI.h>

// ====================================================================
//                          CONFIGURATION
// ====================================================================

// LoRa Pin Definitions
// Ensure these match your physical wiring for the Node 2 ESP32
#define LORA_CS_PIN    5   // Chip Select (NSS)
#define LORA_RST_PIN   14  // Reset (RST)
#define LORA_IRQ_PIN   2   // Interrupt Request (DIO0)

// Define the LoRa frequency (e.g., 433E6, 868E6, 915E6)
#define LORA_BAND      433E6 // 433 MHz band

// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================

// Message counter for unique packets
int counter = 0;

// ====================================================================
//                          SETUP
// ====================================================================

void setup() 
{
  Serial.begin(115200); 
  while (!Serial);
  Serial.println("\nESP32 LoRa Proximity Transmitter Starting...");
  
  // Set the LoRa pins
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);
  
  // Initialize LoRa with the specified frequency
  while (!LoRa.begin(LORA_BAND))
  {
    Serial.println("LoRa initialization failed! Check wiring and module.");
    Serial.print(".");
    delay(500);
  }
  
  // =================================================================
  // SYNCHRONIZATION FIX:
  // These settings MUST match the settings in the Node 1 code
  // to ensure successful communication.
  // =================================================================
  LoRa.setSpreadingFactor(12);     // Spreading Factor (6 to 12)
  LoRa.setSignalBandwidth(125E3);  // Bandwidth (7.8 to 500 kHz)
  LoRa.setCodingRate4(5);          // Coding Rate (5 to 8)
  
  // Set a sync word to filter out packets from other LoRa networks
  // The default sync word is 0x12, but it's good practice to set
  // it explicitly on both devices. The library uses 0x34 as default.
  // Setting it explicitly ensures both are using the same value.
  // Let's use 0x42 as a custom sync word for clarity.
  LoRa.setSyncWord(0x42); 
  
  Serial.println("LoRa Initialized OK!");
}

// ====================================================================
//                          MAIN LOOP
// ====================================================================

void loop() 
{
  // The message to be transmitted
  String message = "I am in proximity of your node, packet #";
  message += counter;

  Serial.print("Sending packet: ");
  Serial.println(message);
  
  // Send the LoRa packet
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  
  // Increment the counter for the next packet
  counter++;
  
  // Wait for 10 seconds before sending the next packet
  delay(10000);
}
