#include <HardwareSerial.h> // For using Serial2

// Define the pins for HardwareSerial2
// We're using GPIO16 for RX (from STM32 TX) and GPIO17 for TX (to STM32 RX)
// Note: ESP32's HardwareSerial instances are 0, 1, 2. Serial0 is USB-UART. Serial1 & Serial2 are configurable.
// GPIO16/GPIO17 are often a good choice for Serial2 as they're not used by default for boot or flash.
HardwareSerial SerialPortFromSTM32(2); // Use UART2 on ESP32

// Buffer to store incoming data from STM32
char receivedChars[100]; // Max length of string we expect + 1 for null terminator
bool newData = false;

// Function prototypes
void recvWithEndMarker();
void parseData();
void showParsedData();

void setup() {
  Serial.begin(115200); // For debugging output to ESP32 Serial Monitor (via USB)
  while (!Serial); // Wait for serial port to connect for debugging

  // Initialize the hardware serial port for communication with STM32
  // Parameters: baud rate, protocol, RX pin, TX pin
  SerialPortFromSTM32.begin(115200, SERIAL_8N1, 16, 17); // Baud rate 115200, 8 data bits, no parity, 1 stop bit
  Serial.println("ESP32: Ready to receive data from STM32...");
}

void loop() {
  recvWithEndMarker(); // Continuously check for new data
  if (newData == true) {
    parseData();
    showParsedData();
    newData = false; // Reset flag
  }
}

// Function to receive data until a newline character
void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;
  const char endMarker = '\n'; // Newline is our delimiter

  while (SerialPortFromSTM32.available() > 0 && newData == false) {
    rc = SerialPortFromSTM32.read();

    if (rc != endMarker) {
      if (ndx < sizeof(receivedChars) - 1) { // Prevent buffer overflow
        receivedChars[ndx] = rc;
        ndx++;
      }
    } else {
      receivedChars[ndx] = '\0'; // Null-terminate the string
      newData = true;
      ndx = 0; // Reset index for next message
    }
  }
}

// Function to parse the received data string
void parseData() {
  // Expected format: T:XX.X,H:YY.Y,IR:Z,S:AAAA,Seq:BBB
  // Using strtok for simple parsing (be mindful of memory and re-entrancy in complex tasks)
  char *token;
  char temp_str[10]; // Small buffer for tokens

  // Temp
  token = strtok(receivedChars, ",");
  if (token != NULL && sscanf(token, "T:%s", temp_str) == 1) {
    // Convert string to float directly if possible or handle 'X.X'
    // For simplicity with sscanf, let's assume T:XX.X
    float temp_val;
    if (sscanf(temp_str, "%f", &temp_val) == 1) {
        Serial.print("Parsed Temp: "); Serial.println(temp_val, 1);
    }
  }

  // Hum
  token = strtok(NULL, ",");
  if (token != NULL && sscanf(token, "H:%s", temp_str) == 1) {
    float hum_val;
    if (sscanf(temp_str, "%f", &hum_val) == 1) {
        Serial.print("Parsed Hum: "); Serial.println(hum_val, 1);
    }
  }

  // IR
  token = strtok(NULL, ",");
  if (token != NULL) {
    int ir_val;
    if (sscanf(token, "IR:%d", &ir_val) == 1) {
      Serial.print("Parsed IR State: "); Serial.println(ir_val == 1 ? "OBSTACLE" : "CLEAR");
    }
  }

  // Sound
  token = strtok(NULL, ",");
  if (token != NULL) {
    int sound_val;
    if (sscanf(token, "S:%d", &sound_val) == 1) {
      Serial.print("Parsed Sound Value: "); Serial.println(sound_val);
    }
  }

  // Sequence
  token = strtok(NULL, ",");
  if (token != NULL) {
    int seq_val;
    if (sscanf(token, "Seq:%d", &seq_val) == 1) {
      Serial.print("Parsed Sequence: "); Serial.println(seq_val);
    }
  }
}


void showParsedData() {
  Serial.println("--- New Data Packet ---");
  // The parsing already prints the individual values.
  Serial.println("-----------------------\n");
}
