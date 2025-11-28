#include <Arduino.h>

// ============================================
// S3 to C5 UART Communication Test
// ============================================
// This firmware sends AT commands from S3 (master) to C5 (slave)
// S3 UART0 (USB) is used for debug output to PC
// S3 UART2 is used to communicate with C5 UART1

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN 35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN 36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN 1   // S3 GPIO1 controls C5 EN pin (power control)

// UART configuration
#define C5_BAUD_RATE 115200
#define UART_BUFFER_SIZE 1024

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2); // UART2

void sendATCommand(const char* command);

void setup() {
  // Initialize USB Serial for debug output (UART0)
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to open
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32-S3 to ESP32-C5 AT Command Test");
  Serial.println("========================================\n");
  
  // Configure C5 power control pin
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH); // Enable C5
  Serial.println("[S3] C5 module powered ON (EN pin HIGH)");
  
  delay(1000); // Wait for C5 to boot
  
  // Initialize UART2 for C5 communication
  C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
  Serial.printf("[S3] UART2 initialized: Baud=%d, TX=GPIO%d, RX=GPIO%d\n", 
                C5_BAUD_RATE, C5_TX_PIN, C5_RX_PIN);
  
  delay(500);
  
  Serial.println("\n[S3] Ready to send AT commands to C5");
  Serial.println("[S3] Commands will be sent every 5 seconds\n");
  Serial.println("----------------------------------------\n");
}

void loop() {
  // Test sequence of AT commands
  static uint32_t lastCommandTime = 0;
  static uint8_t commandIndex = 0;
  
  // Send a command every 5 seconds
  if (millis() - lastCommandTime >= 5000) {
    lastCommandTime = millis();
    
    // Cycle through different AT commands
    switch(commandIndex) {
      case 0:
        sendATCommand("AT"); // Basic test
        break;
      case 1:
        sendATCommand("AT+GMR"); // Get version info
        break;
      case 2:
        sendATCommand("AT+CWMODE?"); // Get WiFi mode
        break;
      case 3:
        sendATCommand("ATE1"); // Enable echo
        break;
      default:
        commandIndex = 0;
        return;
    }
    
    commandIndex++;
    if (commandIndex > 3) commandIndex = 0;
  }
  
  // Check for responses from C5
  if (C5Serial.available()) {
    String response = "";
    unsigned long timeout = millis();
    
    // Read response with timeout
    while (millis() - timeout < 1000) {
      if (C5Serial.available()) {
        char c = C5Serial.read();
        response += c;
        timeout = millis(); // Reset timeout on new data
      }
    }
    
    if (response.length() > 0) {
      Serial.println("[C5 Response]:");
      Serial.println(response);
      Serial.println("----------------------------------------\n");
    }
  }
}

// Function to send AT command to C5
void sendATCommand(const char* command) {
  Serial.printf("[S3] Sending: %s\n", command);
  
  // Send command to C5 with CR+LF termination
  C5Serial.print(command);
  C5Serial.print("\r\n");
  C5Serial.flush(); // Wait for transmission to complete
  
  // Give C5 time to process
  delay(100);
}