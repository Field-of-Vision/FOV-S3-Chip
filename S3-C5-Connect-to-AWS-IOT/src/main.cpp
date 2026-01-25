#include <Arduino.h>

// ============================================
// S3 to C5 AWS IoT MQTT Connection
// ============================================
// This firmware connects C5 to AWS IoT via AT commands
// The C5 has AWS certificates pre-flashed in firmware

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN 35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN 36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN 1   // S3 GPIO1 controls C5 EN pin (power control)

#define THINGNAME "kia-tennis-fov-tablet-20"

const char AWS_IOT_ENDPOINT[] = "a3lkzcadhi1yzr-ats.iot.ap-southeast-2.amazonaws.com";

// UART configuration
#define C5_BAUD_RATE 115200

// WiFi credentials
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// AWS IoT MQTT settings
const char* MQTT_TOPIC = "kia_AUS/pub";  // Topic to subscribe to
const int MQTT_PORT = 8883;  // AWS IoT uses 8883 for MQTT over TLS

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2); // UART2

// Function declarations
void sendATCommand(const char* command, unsigned long timeout = 2000);
String waitForResponse(unsigned long timeout);
bool connectToWiFi();
bool connectToWiFiWithRetry(int maxRetries = 3);
bool configureCertificates();
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
  // Serial.println("Step 1: Testing AT Communication");
  // Serial.println("========================================\n");
  // sendATCommand("AT");
  // delay(500);

  // Get firmware version info
  // Serial.println("\nStep 1.1: Getting Firmware Info");
  // Serial.println("========================================\n");
  // sendATCommand("AT+GMR", 3000);
  // delay(500);

  // // Test MQTT command support
  // Serial.println("\nStep 1.5: Verifying MQTT Support");
  // Serial.println("========================================\n");
  // sendATCommand("AT+MQTTUSERCFG=?", 2000);
  // delay(500);
  
  // Connect to WiFi (with retry logic for production boards)
  Serial.println("\nStep 2: Connecting to WiFi");
  Serial.println("========================================\n");
  if (!connectToWiFiWithRetry(3)) {
    Serial.println("WiFi connection failed after all retries!");
    Serial.println("[S3] HARDWARE CHECK NEEDED:");
    Serial.println("  1. Check power supply - add 100-470uF capacitor near C5 VCC");
    Serial.println("  2. Check antenna area - no metal within 10mm");
    Serial.println("  3. Move closer to router for testing");
    return;
  }
  Serial.println("WiFi connected!\n");
  delay(2000);

  // CRITICAL: Sync time via SNTP - TLS will fail without correct time!
  Serial.println("Step 2.5: Syncing Time (CRITICAL for TLS!)");
  Serial.println("========================================\n");
  Serial.println("[S3] Setting up SNTP time sync...");
  sendATCommand("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\",\"time.google.com\"", 5000);
  delay(3000);  // Wait for time sync

  Serial.println("\n[S3] Checking current time:");
  sendATCommand("AT+CIPSNTPTIME?", 3000);
  delay(1000);

  // Test network connectivity and latency
  // Serial.println("\nStep 2.6: Testing Network Connectivity");
  // Serial.println("========================================\n");
  // sendATCommand("AT+PING=\"8.8.8.8\"", 10000);  // Ping Google DNS
  // delay(1000);
  // Serial.println("\nPinging AWS IoT endpoint (shows latency to Sydney):");
  // sendATCommand("AT+PING=\"a3lkzcadhi1yzr-ats.iot.ap-southeast-2.amazonaws.com\"", 15000);
  // delay(2000);

  // Check certificate/SSL status
  Serial.println("\nStep 3: Checking Certificate Configuration");
  Serial.println("========================================\n");

  // List all namespaces in mfg_nvs
  Serial.println("[S3] Listing all PKI namespaces:");
  sendATCommand("AT+SYSMFG?", 3000);
  delay(500);

  // Read MQTT CA certificate info
  Serial.println("\n[S3] Reading MQTT CA certificate (mqtt_ca namespace):");
  sendATCommand("AT+SYSMFG=1,\"mqtt_ca\"", 3000);
  delay(500);

  // Read first 200 bytes of the CA cert to verify it's the right one
  // Key name is "mqtt_ca" (same as namespace), NOT "mqtt_ca.0"
  Serial.println("\n[S3] Reading first 200 bytes of mqtt_ca:");
  sendATCommand("AT+SYSMFG=1,\"mqtt_ca\",\"mqtt_ca\",0,200", 5000);
  delay(500);

  // Read MQTT client certificate info
  Serial.println("\n[S3] Reading MQTT client certificate (mqtt_cert namespace):");
  sendATCommand("AT+SYSMFG=1,\"mqtt_cert\"", 3000);
  delay(500);

  // Read first 200 bytes of the client cert
  // Key name is "mqtt_cert" (same as namespace), NOT "mqtt_cert.0"
  Serial.println("\n[S3] Reading first 200 bytes of mqtt_cert:");
  sendATCommand("AT+SYSMFG=1,\"mqtt_cert\",\"mqtt_cert\",0,200", 5000);
  delay(500);

  // Read MQTT client key info
  Serial.println("\n[S3] Reading MQTT client key (mqtt_key namespace):");
  sendATCommand("AT+SYSMFG=1,\"mqtt_key\"", 3000);
  delay(500);

  // Read first 200 bytes of the client key
  // Key name is "mqtt_key" (same as namespace), NOT "mqtt_key.0"
  Serial.println("\n[S3] Reading first 200 bytes of mqtt_key:");
  sendATCommand("AT+SYSMFG=1,\"mqtt_key\",\"mqtt_key\",0,200", 5000);
  delay(500);

  // Configure MQTT to use pre-flashed certificates
  Serial.println("\nStep 3.5: Configuring MQTT with Pre-Flashed Certificates");
  Serial.println("========================================\n");
  if (!configureCertificates()) {
    Serial.println("Certificate configuration failed!");
    return;
  }
  Serial.println("Certificates configured!\n");
  delay(2000);
  
  // Connect to AWS IoT
  Serial.println("Step 4: Connecting to AWS IoT");
  Serial.println("========================================\n");
  if (!connectToAWSIoT()) {
    Serial.println("AWS IoT connection failed!");
    return;
  }
  Serial.println("Connected to AWS IoT!\n");
  delay(100);
  
  // Subscribe to MQTT topic
  Serial.println("Step 5: Subscribing to Topic");
  Serial.println("========================================\n");
  if (!subscribeToTopic()) {
    Serial.println("Topic subscription failed!");
    return;
  }
  Serial.println("Subscribed to topic!\n");

  Serial.println("========================================");
  Serial.println("SUCCESS - Listening for coordinates");
  Serial.println("========================================\n");
}

void loop() {
  // Check for incoming MQTT messages
  if (C5Serial.available()) {
    String response = waitForResponse(100);
    
    if (response.length() > 0) {
      // Check if it's an MQTT message
      if (response.indexOf("+MQTTSUBRECV:") >= 0) {
        Serial.println("\n[MQTT Message Received]");
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
  } else {
    Serial.println("[C5 → S3] (no response)");
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

// Function to connect to WiFi (single attempt)
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

  // Check for disconnect BEFORE checking connected
  // This catches the case where WiFi connects then immediately disconnects
  // Error codes: +CWJAP:1 = timeout, +CWJAP:2 = wrong password, etc.
  if (response.indexOf("WIFI DISCONNECT") >= 0) {
    Serial.println("[S3] ERROR: WiFi connected but then disconnected!");
    Serial.println("[S3] This usually indicates a power supply issue.");
    return false;
  }

  if (response.indexOf("+CWJAP:") >= 0) {
    // Parse error code
    int errorIdx = response.indexOf("+CWJAP:");
    if (errorIdx >= 0) {
      char errorCode = response.charAt(errorIdx + 7);
      Serial.printf("[S3] ERROR: WiFi connection failed with code: %c\n", errorCode);
      switch (errorCode) {
        case '1':
          Serial.println("[S3]   Code 1 = Connection timeout");
          break;
        case '2':
          Serial.println("[S3]   Code 2 = Wrong password");
          break;
        case '3':
          Serial.println("[S3]   Code 3 = Cannot find AP");
          break;
        case '4':
          Serial.println("[S3]   Code 4 = Connection failed");
          break;
        default:
          Serial.println("[S3]   Unknown error code");
          break;
      }
    }
    return false;
  }

  // Check for successful connection - WIFI GOT IP is more reliable than WIFI CONNECTED
  // WIFI CONNECTED = association started, WIFI GOT IP = DHCP completed successfully
  if (response.indexOf("WIFI GOT IP") >= 0) {
    delay(1000);

    // Verify we got a real IP (not 0.0.0.0)
    sendATCommand("AT+CIPSTA?");

    // Optional: Query RSSI for signal strength diagnosis
    Serial.println("[S3] Checking WiFi signal strength:");
    sendATCommand("AT+CWJAP?", 3000);

    return true;
  }

  // If we only got WIFI CONNECTED but not GOT IP, that's a partial failure
  if (response.indexOf("WIFI CONNECTED") >= 0) {
    Serial.println("[S3] WARNING: WiFi associated but no IP assigned (DHCP failed)");
    return false;
  }

  return false;
}

// Function to connect to WiFi with retry logic
// Handles intermittent connection issues on production boards
bool connectToWiFiWithRetry(int maxRetries) {
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    Serial.printf("\n[S3] ===== WiFi connection attempt %d/%d =====\n", attempt, maxRetries);

    if (connectToWiFi()) {
      Serial.printf("[S3] WiFi connected successfully on attempt %d\n", attempt);
      return true;
    }

    if (attempt < maxRetries) {
      Serial.println("[S3] Connection failed, retrying in 3 seconds...");
      delay(3000);

      // Reset the C5 module between retries for a clean slate
      Serial.println("[S3] Resetting C5 module...");
      digitalWrite(C5_EN_PIN, LOW);
      delay(500);
      digitalWrite(C5_EN_PIN, HIGH);
      delay(2000);  // Wait for C5 to boot

      // Clear any startup messages
      while(C5Serial.available()) {
        C5Serial.read();
      }
    }
  }

  Serial.printf("[S3] WiFi connection failed after %d attempts\n", maxRetries);
  return false;
}

// Function to configure C5 to use pre-flashed certificates
bool configureCertificates() {
  Serial.println("[S3] Configuring MQTT with pre-flashed certificates...\n");
  
  // Configure MQTT user settings
  // Parameters: AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">
  // LinkID: 0
  // scheme values:
  //   1 = MQTT over TCP
  //   2 = MQTT over TLS (no certificate verify) - WRONG FOR AWS!
  //   3 = MQTT over TLS (verify server certificate only)
  //   4 = MQTT over TLS (provide client certificate only)
  //   5 = MQTT over TLS (verify server cert AND provide client cert) - CORRECT FOR AWS IoT!
  // cert_key_ID: 0 = use mqtt_client.crt and mqtt_client.key (pre-flashed)
  // CA_ID: 0 = use mqtt_ca.crt (pre-flashed)

  char clientIdCmd[256];
  snprintf(clientIdCmd, sizeof(clientIdCmd),
           "AT+MQTTUSERCFG=0,5,\"%s\",\"\",\"\",0,0,\"\"",
           THINGNAME);
  sendATCommand(clientIdCmd, 3000);
  delay(500);
  
  // Configure MQTT connection parameters
  // Parameters: AT+MQTTCONNCFG=<LinkID>,<keepalive>,<disable_clean_session>,<"lwt_topic">,<"lwt_msg">,<lwt_qos>,<lwt_retain>
  // keepalive: 120 seconds (AWS IoT supports up to 1200)
  // disable_clean_session: 0 = clean session
  char mqttConnCmd[256];
  snprintf(mqttConnCmd, sizeof(mqttConnCmd), 
           "AT+MQTTCONNCFG=0,120,0,\"\",\"\",0,0");
  sendATCommand(mqttConnCmd, 3000);
  delay(500);
  
  Serial.println("MQTT configuration complete");
  Serial.println("Using pre-flashed certificates from C5 firmware:");
  Serial.println("  - Scheme: 5 (Verify server cert + provide client cert)");
  Serial.println("  - CA: mqtt_ca.crt (Amazon Root CA 1)");
  Serial.println("  - Client Cert: mqtt_cert (mqtt_client.crt)");
  Serial.println("  - Client Key: mqtt_key (mqtt_client.key)\n");
  
  return true;
}

// Function to connect to AWS IoT
bool connectToAWSIoT() {
  // Build MQTT broker connection string
  // Parameters: AT+MQTTCONN=<LinkID>,<"host">,<port>,<reconnect>
  char brokerCmd[256];
  snprintf(brokerCmd, sizeof(brokerCmd),
           "AT+MQTTCONN=0,\"%s\",%d,1",
           AWS_IOT_ENDPOINT, MQTT_PORT);

  Serial.printf("[S3] Connecting to: %s:%d\n", AWS_IOT_ENDPOINT, MQTT_PORT);
  Serial.printf("[S3] Command: %s\n", brokerCmd);
  Serial.println("[S3] Using TLS with client certificate authentication...");

  // Clear buffer
  while(C5Serial.available()) {
    C5Serial.read();
  }

  // Send connection command
  C5Serial.print(brokerCmd);
  C5Serial.print("\r\n");
  C5Serial.flush();

  // Wait for connection - increased timeout for high-latency connections
  Serial.println("[S3] Waiting for TLS handshake... (this may take 60+ seconds)");
  String response = waitForResponse(90000);  // 90 seconds timeout
  Serial.println(response);

  // Check if connection was successful
  if (response.indexOf("OK") >= 0 || response.indexOf("+MQTTCONNECTED") >= 0) {
    return true;
  }

  // If failed, might be a certificate issue
  if (response.indexOf("ERROR") >= 0 || response.indexOf("FAIL") >= 0) {
    Serial.println("\n[DEBUG] Connection failed. Diagnostics:");
    Serial.println("  1. Check if time is synced (AT+CIPSNTPTIME? should show current date)");
    Serial.println("  2. Check if certificates are flashed to C5 partition");
    Serial.println("  3. Verify Thing name matches AWS IoT console");
    Serial.println("  4. Check AWS IoT policy allows this client ID");

    // Try to get more info about the error
    Serial.println("\n[S3] Checking MQTT state:");
    sendATCommand("AT+MQTTCONN?", 3000);

    // Also check SSL error if available
    Serial.println("\n[S3] Checking for SSL/TLS errors:");
    sendATCommand("AT+SYSMSG?", 3000);
  }

  return false;
}

// Function to subscribe to MQTT topic
bool subscribeToTopic() {
  // Parameters: AT+MQTTSUB=<LinkID>,<"topic">,<qos>
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