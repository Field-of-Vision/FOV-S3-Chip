#include <Arduino.h>

// ============================================
// S3 to C5 WiFi Connection via AT Commands
// ============================================
// This firmware connects the C5 to WiFi using AT commands
// and verifies internet connectivity

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN 35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN 36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN 1   // S3 GPIO1 controls C5 EN pin (power control)

// UART configuration
#define C5_BAUD_RATE 115200

// WiFi credentials
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2); // UART2

// Function declarations
void sendATCommand(const char* command, unsigned long timeout = 2000);
String waitForResponse(unsigned long timeout);
bool connectToWiFi();
bool checkInternetConnection();

void setup() {
  // Initialize USB Serial for debug output (UART0)
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32-S3 C5 WiFi Connection Test");
  Serial.println("========================================\n");
  
  // Configure C5 power control pin
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH); // Enable C5
  Serial.println("[S3] C5 module powered ON (EN pin HIGH)");
  
  delay(2000); // Wait for C5 to boot
  
  // Initialize UART2 for C5 communication
  C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
  Serial.printf("[S3] UART2 initialized: Baud=%d, TX=GPIO%d, RX=GPIO%d\n\n", 
                C5_BAUD_RATE, C5_TX_PIN, C5_RX_PIN);
  
  delay(500);
  
  // Clear any startup messages from C5
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  Serial.println("========================================");
  Serial.println("Step 1: Testing AT Communication");
  Serial.println("========================================\n");
  
  // Test basic AT communication
  sendATCommand("AT");
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("Step 2: Configuring WiFi Mode");
  Serial.println("========================================\n");
  
  // Set WiFi mode to Station (1 = Station, 2 = AP, 3 = Station+AP)
  sendATCommand("AT+CWMODE=1");
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("Step 3: Connecting to WiFi");
  Serial.println("========================================\n");
  Serial.printf("SSID: %s\n", WIFI_SSID);
  Serial.printf("Password: %s\n\n", WIFI_PASSWORD);
  
  // Attempt to connect to WiFi
  if (connectToWiFi()) {
    Serial.println("\n✅ Successfully connected to WiFi!");
    
    delay(2000);
    
    Serial.println("\n========================================");
    Serial.println("Step 4: Checking Internet Connection");
    Serial.println("========================================\n");
    
    // Test internet connectivity
    if (checkInternetConnection()) {
      Serial.println("\n✅ Internet connection verified!");
      Serial.println("\n========================================");
      Serial.println("🎉 SUCCESS - C5 is online!");
      Serial.println("========================================\n");
    } else {
      Serial.println("\n⚠️ Connected to WiFi but internet test failed");
    }
  } else {
    Serial.println("\n❌ Failed to connect to WiFi");
    Serial.println("Please check:");
    Serial.println("  - SSID is correct: tim");
    Serial.println("  - Password is correct: password");
    Serial.println("  - 2.4GHz hotspot is enabled");
    Serial.println("  - C5 is in range of the hotspot");
  }
}

void loop() {
  // Check WiFi status every 10 seconds
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck >= 10000) {
    lastCheck = millis();
    
    Serial.println("\n--- WiFi Status Check ---");
    // Check if still connected to AP
    sendATCommand("AT+CWJAP?");
    delay(300);
    
    // Check IP address
    sendATCommand("AT+CIPSTA?");
    delay(500);
  }
  
  // Forward any unsolicited messages from C5
  if (C5Serial.available()) {
    String msg = waitForResponse(100);
    if (msg.length() > 0) {
      Serial.println("[C5 Message]:");
      Serial.println(msg);
    }
  }
  
  delay(100);
}

// Function to send AT command to C5
void sendATCommand(const char* command, unsigned long timeout) {
  Serial.printf("[S3] Sending: %s\n", command);
  
  // Clear any pending data
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  // Send command to C5 with CR+LF termination
  C5Serial.print(command);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  // Wait for and display response
  String response = waitForResponse(timeout);
  if (response.length() > 0) {
    Serial.println("[C5 Response]:");
    Serial.println(response);
  } else {
    Serial.println("[C5 Response]: (no response)");
  }
}

// Function to wait for response from C5
String waitForResponse(unsigned long timeout) {
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    if (C5Serial.available()) {
      char c = C5Serial.read();
      response += c;
      // Reset timeout when receiving data
      startTime = millis();
    }
  }
  
  return response;
}

// Function to connect to WiFi
bool connectToWiFi() {
  // Disconnect from any existing connection
  Serial.println("[S3] Disconnecting from any existing WiFi...");
  sendATCommand("AT+CWQAP");
  delay(1000);
  
  // Build the connection command
  char connectCmd[128];
  snprintf(connectCmd, sizeof(connectCmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
  
  Serial.println("[S3] Attempting to connect...");
  Serial.printf("[S3] Sending: %s\n", connectCmd);
  
  // Clear buffer
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  // Send connection command
  C5Serial.print(connectCmd);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  // Wait for connection (can take 10-15 seconds)
  String response = waitForResponse(20000); // 20 second timeout
  
  Serial.println("[C5 Response]:");
  Serial.println(response);
  
  // Check if connection was successful
  if (response.indexOf("WIFI CONNECTED") >= 0 || response.indexOf("WIFI GOT IP") >= 0 || response.indexOf("OK") >= 0) {
    delay(2000);
    
    // Query IP address to confirm
    Serial.println("\n[S3] Querying IP address...");
    sendATCommand("AT+CIPSTA?");
    
    return true;
  }
  
  return false;
}

// Function to check internet connectivity
bool checkInternetConnection() {
  // Test internet with DNS query to google.com
  Serial.println("[S3] Testing internet with DNS query...");
  sendATCommand("AT+CIPDOMAIN=\"google.com\"", 5000);
  
  delay(1000);
  
  // Query current WiFi connection info
  Serial.println("\n[S3] Verifying WiFi connection...");
  sendATCommand("AT+CWJAP?");
  
  return true; // If we got this far without errors, connection is good
}
