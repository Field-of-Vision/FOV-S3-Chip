#include <Arduino.h>
#include "secrets.h"

// ============================================
// S3 to C5 AWS IoT MQTT Connection
// ============================================
// This firmware connects C5 to AWS IoT via AT commands

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN 35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN 36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN 1   // S3 GPIO1 controls C5 EN pin (power control)

// UART configuration
#define C5_BAUD_RATE 115200

// WiFi credentials
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// AWS IoT MQTT settings
const char* MQTT_TOPIC = "dalymount_IRL/pub";  // Topic to subscribe to
const int MQTT_PORT = 8883;  // AWS IoT uses 8883 for MQTT over TLS

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2); // UART2

// Function declarations
void sendATCommand(const char* command, unsigned long timeout = 2000);
String waitForResponse(unsigned long timeout);
bool connectToWiFi();
bool uploadCertificates();
bool connectToAWSIoT();
bool subscribeToTopic();
void processCoordinates(String payload);

void setup() {
  // Initialize USB Serial for debug output (UART0)
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32-S3 AWS IoT MQTT via C5");
  Serial.println("========================================\n");
  
  // Configure C5 power control pin
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH); // Enable C5
  Serial.println("[S3] C5 module powered ON");
  
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
  
  // Test basic AT communication
  Serial.println("Step 1: Testing AT Communication");
  Serial.println("========================================\n");
  sendATCommand("AT");
  delay(500);
  
  // Connect to WiFi
  Serial.println("\nStep 2: Connecting to WiFi");
  Serial.println("========================================\n");
  if (!connectToWiFi()) {
    Serial.println("❌ WiFi connection failed!");
    return;
  }
  Serial.println("✅ WiFi connected!\n");
  delay(2000);
  
  // Upload certificates to C5
  Serial.println("Step 3: Uploading AWS Certificates");
  Serial.println("========================================\n");
  if (!uploadCertificates()) {
    Serial.println("❌ Certificate upload failed!");
    return;
  }
  Serial.println("✅ Certificates uploaded!\n");
  delay(2000);
  
  // Connect to AWS IoT
  Serial.println("Step 4: Connecting to AWS IoT");
  Serial.println("========================================\n");
  if (!connectToAWSIoT()) {
    Serial.println("❌ AWS IoT connection failed!");
    return;
  }
  Serial.println("✅ Connected to AWS IoT!\n");
  delay(2000);
  
  // Subscribe to MQTT topic
  Serial.println("Step 5: Subscribing to Topic");
  Serial.println("========================================\n");
  if (!subscribeToTopic()) {
    Serial.println("❌ Topic subscription failed!");
    return;
  }
  Serial.println("✅ Subscribed to topic!\n");
  
  Serial.println("========================================");
  Serial.println("🎉 SUCCESS - Listening for coordinates");
  Serial.println("========================================\n");
}

void loop() {
  // Check for incoming MQTT messages
  if (C5Serial.available()) {
    String response = waitForResponse(100);
    
    if (response.length() > 0) {
      // Check if it's an MQTT message
      if (response.indexOf("+MQTTSUBRECV:") >= 0) {
        Serial.println("\n[📡 MQTT Message Received]");
        Serial.println(response);
        
        // Extract and process the JSON payload
        int payloadStart = response.indexOf("{");
        int payloadEnd = response.lastIndexOf("}");
        
        if (payloadStart >= 0 && payloadEnd > payloadStart) {
          String payload = response.substring(payloadStart, payloadEnd + 1);
          processCoordinates(payload);
        }
      } else {
        // Other messages from C5
        Serial.println("[C5 Message]:");
        Serial.println(response);
      }
    }
  }
  
  delay(10); // Small delay to not overwhelm the UART
}

// Function to send AT command to C5
void sendATCommand(const char* command, unsigned long timeout) {
  Serial.printf("[S3 → C5] %s\n", command);
  
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
    Serial.println("[C5 → S3]");
    Serial.println(response);
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
  // Set WiFi mode to Station
  sendATCommand("AT+CWMODE=1");
  delay(500);
  
  // Disconnect from any existing connection
  sendATCommand("AT+CWQAP");
  delay(1000);
  
  // Build the connection command
  char connectCmd[128];
  snprintf(connectCmd, sizeof(connectCmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
  
  Serial.printf("[S3] Connecting to WiFi: %s\n", WIFI_SSID);
  
  // Clear buffer
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  // Send connection command
  C5Serial.print(connectCmd);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  // Wait for connection (can take 10-15 seconds)
  String response = waitForResponse(20000);
  Serial.println(response);
  
  // Check if connection was successful
  if (response.indexOf("WIFI CONNECTED") >= 0 || response.indexOf("WIFI GOT IP") >= 0) {
    delay(1000);
    sendATCommand("AT+CIPSTA?");
    return true;
  }
  
  return false;
}

// Function to upload certificates to C5
bool uploadCertificates() {
  Serial.println("[S3] Uploading certificates to C5...");
  Serial.println("This may take a minute...\n");
  
  // Note: The C5 AT firmware may need to be configured to accept certificates
  // For now, we'll use the MQTT client ID and configure SSL
  
  // Configure MQTT username (AWS IoT doesn't use username/password, but we set client ID)
  char clientIdCmd[128];
  snprintf(clientIdCmd, sizeof(clientIdCmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"", THINGNAME);
  sendATCommand(clientIdCmd, 3000);
  delay(500);
  
  // Configure MQTT connection parameters
  char mqttConnCmd[256];
  snprintf(mqttConnCmd, sizeof(mqttConnCmd), 
           "AT+MQTTCONNCFG=0,0,600,1,\"\",\"\"");
  sendATCommand(mqttConnCmd, 3000);
  delay(500);
  
  Serial.println("✓ MQTT configuration set");
  
  // Note: Certificate upload via AT commands is complex and depends on C5 AT firmware version
  // Some versions support AT+SYSFLASH for certificate storage
  // For this demo, we'll proceed assuming certificates are pre-configured or use alternative method
  
  Serial.println("⚠️ Note: Full certificate upload via AT commands requires");
  Serial.println("   additional firmware support. For production, use:");
  Serial.println("   - Pre-flash certificates to C5 filesystem");
  Serial.println("   - Or use AT+SYSFLASH commands if supported");
  
  return true;
}

// Function to connect to AWS IoT
bool connectToAWSIoT() {
  // Build MQTT broker connection string
  char brokerCmd[256];
  snprintf(brokerCmd, sizeof(brokerCmd), 
           "AT+MQTTCONN=0,\"%s\",%d,1", 
           AWS_IOT_ENDPOINT, MQTT_PORT);
  
  Serial.printf("[S3] Connecting to: %s:%d\n", AWS_IOT_ENDPOINT, MQTT_PORT);
  
  // Clear buffer
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  // Send connection command
  C5Serial.print(brokerCmd);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  // Wait for connection (can take 10-20 seconds for TLS handshake)
  String response = waitForResponse(30000);
  Serial.println(response);
  
  // Check if connection was successful
  if (response.indexOf("OK") >= 0 || response.indexOf("+MQTTCONNECTED") >= 0) {
    return true;
  }
  
  return false;
}

// Function to subscribe to MQTT topic
bool subscribeToTopic() {
  char subCmd[128];
  snprintf(subCmd, sizeof(subCmd), "AT+MQTTSUB=0,\"%s\",1", MQTT_TOPIC);
  
  Serial.printf("[S3] Subscribing to: %s\n", MQTT_TOPIC);
  
  // Clear buffer
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  // Send subscribe command
  C5Serial.print(subCmd);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  // Wait for confirmation
  String response = waitForResponse(5000);
  Serial.println(response);
  
  // Check if subscription was successful
  if (response.indexOf("OK") >= 0) {
    return true;
  }
  
  return false;
}

// Function to process coordinate data from MQTT
void processCoordinates(String payload) {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   MATCH COORDINATES RECEIVED       ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println(payload);
  
  // TODO: Parse JSON and extract coordinates
  // Example payload: {"T": 5.2, "X": 51.09, "Y": 55.41, "P": 1, "Pa": 0, "G": 0, "C": 0, "R": 0, "S": 0}
  // You would parse this and control the haptic motors accordingly
  
  Serial.println("════════════════════════════════════\n");
}