#include <SPI.h>
#include <LoRa.h>

// ====================================================================
//                          CONFIGURATION- Polling for reception
// ====================================================================

// LoRa Pin Definitions
// Ensure these match your physical wiring for the Node 2 ESP32
#define LORA_CS_PIN    5   // Chip Select (NSS)
#define LORA_RST_PIN   14  // Reset (RST)
#define LORA_IRQ_PIN   2   // Interrupt Request (DIO0)

// Define the LoRa frequency (e.g., 433E6, 868E6, 915E6)
// This MUST be the same as Node 1
#define LORA_BAND      433E6 // 433 MHz band

// Timer interval for sending a proximity alert (in milliseconds)
const long PROXIMITY_ALERT_INTERVAL = 10000; // 10 seconds

// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================

// Timer variable for the proximity alert
unsigned long lastProximityAlertTime = 0;

// ====================================================================
//                          FUNCTION PROTOTYPES
// ====================================================================

void sendProximityAlert();

// ====================================================================
//                          SETUP
// ====================================================================

void setup() 
{
  // Initialize the serial port for debugging output
  Serial.begin(115200); 
  while (!Serial);
  Serial.println("\nESP32 LoRa Receiver Node 2 Starting...");
  
  // Initialize LoRa
  Serial.println("\nInitializing LoRa...");
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);
  while (!LoRa.begin(LORA_BAND))
  {
    Serial.println("LoRa initialization failed! Check wiring and module.");
    delay(500);
  }

  // Set the LoRa parameters to match Node 1 exactly
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x42); 

  // We are now polling for packets in the loop, so we don't need the interrupt.
  LoRa.receive(); // Start receiving packets initially
  
  Serial.println("LoRa Initialized OK and ready to receive!");
  Serial.println("Waiting for messages...");

  // Initialize the timer for the first proximity alert
  lastProximityAlertTime = millis();
}

// ====================================================================
//                          MAIN LOOP
// ====================================================================

void loop() 
{
  // -------------------------------------------------------------
  // Section 1: Check for LoRa packets by polling LoRa.parsePacket()
  // -------------------------------------------------------------
  int packetSize = LoRa.parsePacket();
  
  if (packetSize)
  {
    Serial.println("-------------------------------------------");
    Serial.println(">>> LoRa Packet Received (polling) <<<");

    // Read the incoming packet
    String receivedLoRaMessage = "";
    while (LoRa.available())
    {
      receivedLoRaMessage += (char)LoRa.read();
    }
    
    // Print the message and other packet information
    Serial.print("Message: ");
    Serial.println(receivedLoRaMessage);
    Serial.print("With RSSI: ");
    Serial.println(LoRa.packetRssi());
    
    Serial.println("-------------------------------------------");
  }

  // -------------------------------------------------------------
  // Section 2: Periodically send a proximity alert to Node 1
  // -------------------------------------------------------------
  if (millis() - lastProximityAlertTime >= PROXIMITY_ALERT_INTERVAL) {
    sendProximityAlert();
    lastProximityAlertTime = millis();
  }
}

// ====================================================================
//                     LORA TRANSMISSION FUNCTIONS
// ====================================================================

void sendProximityAlert() {
  String alertMessage = "I am in proximity of your node! Be Alert";
  Serial.println("--- Starting LoRa Transmission ---");
  LoRa.beginPacket();
  LoRa.print(alertMessage);
  LoRa.endPacket();
  Serial.println("--- LoRa Transmission Complete ---");
  
  Serial.println("[Tx LoRa Alert] : " + alertMessage);
  
  // IMPORTANT: Re-enter receive mode after sending a packet
  LoRa.receive();
  Serial.println("--- LoRa is now in Receive Mode ---");
}
