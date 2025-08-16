#include <SPI.h>
#include <LoRa.h>

// ====================================================================
//                          CONFIGURATION
// ====================================================================

// LoRa Pin Definitions
// Ensure these match your physical wiring for the Node 2 ESP32
#define LORA_CS_PIN    5   // Chip Select (NSS)
#define LORA_RST_PIN   14  // Reset (RST)
#define LORA_IRQ_PIN   2   // Interrupt Request (DIO0)

// Define the LoRa frequency (e.g., 433E6, 868E6, 915E6)
// This MUST be the same as Node 1
#define LORA_BAND      433E6 // 433 MHz band

// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================

// Use 'volatile' for variables modified inside an ISR
volatile bool loraReceivedFlag = false;
String receivedLoRaMessage = "";

// ====================================================================
//                          FUNCTION PROTOTYPES
// ====================================================================

// LoRa related functions
void onReceive(int packetSize);

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

  // Attach the interrupt handler for LoRa packet reception
  LoRa.onReceive(onReceive);
  LoRa.receive(); // Start receiving packets
  
  Serial.println("LoRa Initialized OK and ready to receive!");
  Serial.println("Waiting for messages from Node 1...");
}

// ====================================================================
//                          MAIN LOOP
// ====================================================================

void loop() 
{
  // Check if a packet has been received (the flag is set in the ISR)
  if (loraReceivedFlag)
  {
    Serial.println("-------------------------------------------");
    Serial.println(">>> LoRa Packet Received (from ISR) <<<");
    Serial.print("Message: ");
    Serial.println(receivedLoRaMessage);
    Serial.println("-------------------------------------------");
    
    // Reset the flag and clear the message for the next packet
    loraReceivedFlag = false;
    receivedLoRaMessage = "";
    
    // IMPORTANT: Re-enter receive mode after handling a packet
    LoRa.receive();
  }
}

// ====================================================================
//                     INTERRUPT SERVICE ROUTINE (ISR)
// ====================================================================

void onReceive(int packetSize)
{
  // Debug line to confirm the ISR is being triggered
  Serial.print("onReceive ISR triggered! Packet size: "); 
  Serial.println(packetSize);

  if (packetSize > 0)
  {
    // Clear the message string before reading
    receivedLoRaMessage = "";
    while (LoRa.available())
    {
      receivedLoRaMessage += (char)LoRa.read();
    }
    // Set the flag to indicate a new message is ready to be processed
    loraReceivedFlag = true;
  }
}
