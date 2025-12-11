#include <Arduino.h>

// ============================================
// S3 WiFi Provisioning via C5 Captive Portal
// ============================================
// This firmware runs on the ESP32-S3 and communicates
// with the ESP32-C5 (running ESP-AT firmware with web server
// and captive portal support enabled) to set up a WiFi
// provisioning captive portal.
//
// Users connect to the C5's SoftAP, get redirected to
// a web page where they enter WiFi credentials, and then
// the C5 connects to that network.
// ============================================

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN   35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN   36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN   1   // S3 GPIO1 controls C5 EN pin (HIGH = enabled)

// UART configuration
#define C5_BAUD_RATE 115200

// RGB LED on S3 for status indication (GPIO 38)
#define RGB_LED_PIN 38

// SoftAP Configuration - Users will see this network name
const char* SOFTAP_SSID = "FOV_Setup";        // Network name shown to users
const char* SOFTAP_PASSWORD = "";              // Empty = open network (easier for setup)
// const char* SOFTAP_PASSWORD = "fovsetup123"; // Or use password for security

// Web server configuration
const int WEBSERVER_PORT = 80;
const int WEBSERVER_TIMEOUT = 60;  // seconds

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2);

// State machine for provisioning process
enum ProvisioningState {
  STATE_INIT,
  STATE_CHECK_C5,
  STATE_RESTORE_DEFAULTS,
  STATE_SET_WIFI_MODE,
  STATE_CONFIGURE_SOFTAP,
  STATE_ENABLE_MUX,
  STATE_START_WEBSERVER,
  STATE_WAITING_FOR_CREDENTIALS,
  STATE_WIFI_CONNECTING,
  STATE_WIFI_CONNECTED,
  STATE_ERROR,
  STATE_PROVISIONING_COMPLETE
};

ProvisioningState currentState = STATE_INIT;
const char* stateNames[] = {
  "INIT", "CHECK_C5", "RESTORE_DEFAULTS", "SET_WIFI_MODE",
  "CONFIGURE_SOFTAP", "ENABLE_MUX", "START_WEBSERVER",
  "WAITING_FOR_CREDENTIALS", "WIFI_CONNECTING", "WIFI_CONNECTED",
  "ERROR", "PROVISIONING_COMPLETE"
};

// Timing variables
unsigned long stateStartTime = 0;
unsigned long lastActivityTime = 0;

// Connection info storage
String connectedSSID = "";
String connectedIP = "";

// Buffer for receiving data
#define RX_BUFFER_SIZE 1024
char rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex = 0;

// Function declarations
bool sendATCommand(const char* command, unsigned long timeout = 2000);
bool sendATCommandAndWait(const char* command, const char* expectedResponse, unsigned long timeout = 5000);
bool sendATCommandAndWaitMultiple(const char* command, const char** expectedResponses, int numResponses, unsigned long timeout = 5000);
void processIncomingData();
void handleWebServerResponse(const char* line);
void printStatus();
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);

// ============================================
// Setup
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║     ESP32-S3 WiFi Provisioning via C5 Captive Portal         ║");
  Serial.println("║                  Field of Vision Project                     ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");
  
  // Initialize RGB LED
  pinMode(RGB_LED_PIN, OUTPUT);
  setLEDColor(255, 0, 0);  // Red = initializing
  
  // Power on C5
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH);
  Serial.println("[S3] C5 module powered ON");
  
  delay(3000);  // Wait for C5 to fully boot
  
  // Initialize UART2 for C5 communication
  C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
  Serial.printf("[S3] UART2 initialized: TX=GPIO%d, RX=GPIO%d @ %d baud\n", 
                C5_TX_PIN, C5_RX_PIN, C5_BAUD_RATE);
  
  delay(500);
  
  // Clear any boot messages from C5
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  Serial.println("[S3] Ready to start provisioning sequence\n");
  
  stateStartTime = millis();
  currentState = STATE_CHECK_C5;
}

// ============================================
// Main Loop
// ============================================
void loop() {
  static unsigned long lastStatusPrint = 0;
  
  // Always process incoming data from C5
  processIncomingData();
  
  // State machine
  switch (currentState) {
    
    case STATE_CHECK_C5:
      Serial.println("\n[STATE] Checking C5 communication...");
      setLEDColor(255, 165, 0);  // Orange
      
      if (sendATCommandAndWait("AT", "OK", 3000)) {
        Serial.println("[OK] C5 is responding!");
        currentState = STATE_RESTORE_DEFAULTS;
      } else {
        Serial.println("[ERROR] C5 not responding. Retrying...");
        delay(2000);
      }
      break;
      
    case STATE_RESTORE_DEFAULTS:
      Serial.println("\n[STATE] Clearing previous WiFi configuration...");
      // Use AT+CWQAP to disconnect from any previous AP instead of full restore
      // This is faster and preserves any certificates
      sendATCommandAndWait("AT+CWQAP", "OK", 3000);
      delay(500);
      currentState = STATE_SET_WIFI_MODE;
      break;
      
    case STATE_SET_WIFI_MODE:
      Serial.println("\n[STATE] Setting WiFi mode to Station+SoftAP...");
      // Mode 3 = Station + SoftAP mode (required for web provisioning)
      if (sendATCommandAndWait("AT+CWMODE=3", "OK", 3000)) {
        Serial.println("[OK] WiFi mode set to Station+SoftAP");
        currentState = STATE_CONFIGURE_SOFTAP;
      } else {
        Serial.println("[ERROR] Failed to set WiFi mode");
        currentState = STATE_ERROR;
      }
      break;
      
    case STATE_CONFIGURE_SOFTAP:
      Serial.println("\n[STATE] Configuring SoftAP...");
      {
        char cmd[256];
        // AT+CWSAP="ssid","password",channel,encryption,max_conn
        // Channel 11 is generally less congested
        // Encryption: 0=OPEN, 2=WPA_PSK, 3=WPA2_PSK, 4=WPA_WPA2_PSK
        
        int encryption = (strlen(SOFTAP_PASSWORD) > 0) ? 3 : 0;  // WPA2 if password, otherwise open
        
        snprintf(cmd, sizeof(cmd), "AT+CWSAP=\"%s\",\"%s\",11,%d,4", 
                 SOFTAP_SSID, SOFTAP_PASSWORD, encryption);
        
        if (sendATCommandAndWait(cmd, "OK", 5000)) {
          Serial.printf("[OK] SoftAP configured: SSID=\"%s\"\n", SOFTAP_SSID);
          if (strlen(SOFTAP_PASSWORD) > 0) {
            Serial.printf("     Password: \"%s\"\n", SOFTAP_PASSWORD);
          } else {
            Serial.println("     (Open network - no password)");
          }
          currentState = STATE_ENABLE_MUX;
        } else {
          Serial.println("[ERROR] Failed to configure SoftAP");
          currentState = STATE_ERROR;
        }
      }
      break;
      
    case STATE_ENABLE_MUX:
      Serial.println("\n[STATE] Enabling multiple connections...");
      if (sendATCommandAndWait("AT+CIPMUX=1", "OK", 3000)) {
        Serial.println("[OK] Multiple connections enabled");
        currentState = STATE_START_WEBSERVER;
      } else {
        Serial.println("[ERROR] Failed to enable multiple connections");
        currentState = STATE_ERROR;
      }
      break;
      
    case STATE_START_WEBSERVER:
      Serial.println("\n[STATE] Starting web server with captive portal...");
      {
        char cmd[64];
        // AT+WEBSERVER=<enable>,<port>,<timeout>
        snprintf(cmd, sizeof(cmd), "AT+WEBSERVER=1,%d,%d", 
                 WEBSERVER_PORT, WEBSERVER_TIMEOUT);
        
        if (sendATCommandAndWait(cmd, "OK", 5000)) {
          Serial.println("[OK] Web server started!");
          setLEDColor(0, 0, 255);  // Blue = waiting for connection
          
          Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
          Serial.println("║              CAPTIVE PORTAL IS NOW ACTIVE                    ║");
          Serial.println("╠══════════════════════════════════════════════════════════════╣");
          Serial.printf("║  Network Name: %-44s ║\n", SOFTAP_SSID);
          if (strlen(SOFTAP_PASSWORD) > 0) {
            Serial.printf("║  Password: %-48s ║\n", SOFTAP_PASSWORD);
          } else {
            Serial.printf("║  Password: %-48s ║\n", "(none - open network)");
          }
          Serial.println("║                                                              ║");
          Serial.println("║  Instructions:                                               ║");
          Serial.println("║  1. Connect your phone/laptop to the WiFi network above      ║");
          Serial.println("║  2. A login page should appear automatically                 ║");
          Serial.println("║  3. If not, open browser and go to: http://192.168.4.1       ║");
          Serial.println("║  4. Select your home WiFi network and enter password         ║");
          Serial.println("║  5. Click 'Connect' - the device will connect to your WiFi   ║");
          Serial.println("╚══════════════════════════════════════════════════════════════╝\n");
          
          currentState = STATE_WAITING_FOR_CREDENTIALS;
          lastActivityTime = millis();
        } else {
          Serial.println("[ERROR] Failed to start web server");
          Serial.println("[HINT] Make sure your C5 firmware has web server support enabled!");
          Serial.println("       Enable via: ./build.py menuconfig > Component config > AT > AT WEB Server command support");
          Serial.println("       Also enable: AT WEB captive portal support");
          currentState = STATE_ERROR;
        }
      }
      break;
      
    case STATE_WAITING_FOR_CREDENTIALS:
      // Just waiting - processIncomingData() handles the responses
      // Blink LED slowly to indicate waiting
      {
        static unsigned long lastBlink = 0;
        static bool ledOn = true;
        if (millis() - lastBlink > 1000) {
          lastBlink = millis();
          ledOn = !ledOn;
          if (ledOn) {
            setLEDColor(0, 0, 255);  // Blue
          } else {
            setLEDColor(0, 0, 50);   // Dim blue
          }
        }
      }
      break;
      
    case STATE_WIFI_CONNECTING:
      // Waiting for connection result
      setLEDColor(255, 255, 0);  // Yellow = connecting
      // Timeout after 30 seconds
      if (millis() - stateStartTime > 30000) {
        Serial.println("[ERROR] WiFi connection timeout");
        currentState = STATE_WAITING_FOR_CREDENTIALS;
      }
      break;
      
    case STATE_WIFI_CONNECTED:
      Serial.println("\n[SUCCESS] WiFi connection established!");
      setLEDColor(0, 255, 0);  // Green = connected
      
      // Get the IP address
      delay(500);
      Serial.println("[INFO] Querying IP address...");
      sendATCommand("AT+CIFSR", 3000);
      
      currentState = STATE_PROVISIONING_COMPLETE;
      break;
      
    case STATE_PROVISIONING_COMPLETE:
      // Provisioning done - now you can proceed with your main application
      Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
      Serial.println("║           WIFI PROVISIONING COMPLETE!                        ║");
      Serial.println("╚══════════════════════════════════════════════════════════════╝\n");
      
      Serial.println("[INFO] The C5 is now connected to WiFi.");
      Serial.println("[INFO] You can now disable the web server and proceed with MQTT, etc.");
      Serial.println("[INFO] To disable web server, send: AT+WEBSERVER=0\n");
      
      // Optionally disable web server to free resources
      // sendATCommandAndWait("AT+WEBSERVER=0", "OK", 3000);
      
      // Keep LED green and stop state machine
      setLEDColor(0, 255, 0);
      
      // Print periodic status
      if (millis() - lastStatusPrint > 10000) {
        lastStatusPrint = millis();
        Serial.println("[STATUS] Provisioning complete. System ready for next steps.");
      }
      break;
      
    case STATE_ERROR:
      setLEDColor(255, 0, 0);  // Red = error
      Serial.println("\n[ERROR] Provisioning failed!");
      Serial.println("[INFO] Will retry in 5 seconds...");
      delay(5000);
      currentState = STATE_CHECK_C5;
      break;
  }
}

// ============================================
// Send AT command and print response (no wait for specific response)
// ============================================
bool sendATCommand(const char* command, unsigned long timeout) {
  // Clear buffers
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  Serial.printf("[TX] %s\n", command);
  C5Serial.print(command);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (C5Serial.available()) {
      char c = C5Serial.read();
      response += c;
      Serial.print(c);  // Echo to serial monitor
    }
  }
  
  return true;
}

// ============================================
// Send AT command and wait for specific response
// ============================================
bool sendATCommandAndWait(const char* command, const char* expectedResponse, unsigned long timeout) {
  // Clear buffers
  while(C5Serial.available()) {
    C5Serial.read();
  }
  rxBufferIndex = 0;
  
  Serial.printf("[TX] %s\n", command);
  C5Serial.print(command);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (C5Serial.available()) {
      char c = C5Serial.read();
      response += c;
      
      // Check for expected response
      if (response.indexOf(expectedResponse) >= 0) {
        Serial.printf("[RX] %s\n", response.c_str());
        return true;
      }
      
      // Check for error
      if (response.indexOf("ERROR") >= 0) {
        Serial.printf("[RX] (error) %s\n", response.c_str());
        return false;
      }
    }
  }
  
  Serial.printf("[RX] (timeout) %s\n", response.c_str());
  return false;
}

// ============================================
// Process incoming data from C5
// This handles unsolicited messages like WiFi events
// ============================================
void processIncomingData() {
  while (C5Serial.available()) {
    char c = C5Serial.read();
    
    // Add to buffer
    if (rxBufferIndex < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxBufferIndex++] = c;
      rxBuffer[rxBufferIndex] = '\0';
    }
    
    // Process complete lines
    if (c == '\n') {
      // Remove trailing CR/LF
      while (rxBufferIndex > 0 && (rxBuffer[rxBufferIndex-1] == '\n' || rxBuffer[rxBufferIndex-1] == '\r')) {
        rxBuffer[--rxBufferIndex] = '\0';
      }
      
      // Skip empty lines
      if (rxBufferIndex > 0) {
        handleWebServerResponse(rxBuffer);
      }
      
      // Reset buffer
      rxBufferIndex = 0;
    }
  }
}

// ============================================
// Handle web server and WiFi responses
// ============================================
void handleWebServerResponse(const char* line) {
  Serial.printf("[C5] %s\n", line);
  
  // +WEBSERVERRSP:1 = Received WiFi credentials from web page
  if (strstr(line, "+WEBSERVERRSP:1") != NULL) {
    Serial.println("\n[EVENT] WiFi credentials received from user!");
    currentState = STATE_WIFI_CONNECTING;
    stateStartTime = millis();
    lastActivityTime = millis();
  }
  
  // +WEBSERVERRSP:2 = WiFi connection result sent to client
  if (strstr(line, "+WEBSERVERRSP:2") != NULL) {
    Serial.println("[EVENT] Connection result sent to user's browser");
    // Note: This comes after successful connection
  }
  
  // WIFI CONNECTED
  if (strstr(line, "WIFI CONNECTED") != NULL) {
    Serial.println("[EVENT] WiFi connecting to AP...");
  }
  
  // WIFI GOT IP
  if (strstr(line, "WIFI GOT IP") != NULL) {
    Serial.println("[EVENT] WiFi connected and got IP!");
    currentState = STATE_WIFI_CONNECTED;
    stateStartTime = millis();
  }
  
  // WIFI DISCONNECT
  if (strstr(line, "WIFI DISCONNECT") != NULL) {
    Serial.println("[EVENT] WiFi disconnected");
    if (currentState == STATE_WIFI_CONNECTING) {
      Serial.println("[ERROR] Failed to connect - wrong password?");
      currentState = STATE_WAITING_FOR_CREDENTIALS;
    }
  }
  
  // +CWJAP: error codes
  if (strstr(line, "+CWJAP:") != NULL) {
    // Parse error code
    int errorCode = atoi(line + 7);
    Serial.printf("[EVENT] CWJAP error code: %d\n", errorCode);
    switch (errorCode) {
      case 1: Serial.println("        Connection timeout"); break;
      case 2: Serial.println("        Wrong password"); break;
      case 3: Serial.println("        Cannot find target AP"); break;
      case 4: Serial.println("        Connection failed"); break;
      default: Serial.println("        Unknown error"); break;
    }
  }
  
  // Client connected to SoftAP
  if (strstr(line, "+STA_CONNECTED:") != NULL) {
    Serial.println("[EVENT] A device connected to our SoftAP!");
  }
  
  // Client disconnected from SoftAP
  if (strstr(line, "+STA_DISCONNECTED:") != NULL) {
    Serial.println("[EVENT] A device disconnected from our SoftAP");
  }
  
  // IP address info from AT+CIFSR
  if (strstr(line, "+CIFSR:STAIP,") != NULL) {
    // Extract IP: +CIFSR:STAIP,"192.168.1.100"
    const char* ipStart = strchr(line, '"');
    if (ipStart) {
      ipStart++;
      const char* ipEnd = strchr(ipStart, '"');
      if (ipEnd) {
        int len = ipEnd - ipStart;
        char ip[32];
        strncpy(ip, ipStart, len);
        ip[len] = '\0';
        connectedIP = String(ip);
        Serial.printf("[INFO] Station IP Address: %s\n", ip);
      }
    }
  }
  
  // SoftAP IP address
  if (strstr(line, "+CIFSR:APIP,") != NULL) {
    const char* ipStart = strchr(line, '"');
    if (ipStart) {
      ipStart++;
      const char* ipEnd = strchr(ipStart, '"');
      if (ipEnd) {
        int len = ipEnd - ipStart;
        char ip[32];
        strncpy(ip, ipStart, len);
        ip[len] = '\0';
        Serial.printf("[INFO] SoftAP IP Address: %s\n", ip);
      }
    }
  }
}

// ============================================
// Set RGB LED color (for S3 onboard LED)
// Note: This is a simplified version - the S3's LED is actually
// a WS2812 addressable LED, so you might need the Adafruit NeoPixel
// library for proper control. This is a placeholder.
// ============================================
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  // For proper WS2812 control, use:
  // #include <Adafruit_NeoPixel.h>
  // Adafruit_NeoPixel led(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
  // led.setPixelColor(0, led.Color(r, g, b));
  // led.show();
  
  // For now, just use the pin as a simple indicator
  // (won't show actual colors without NeoPixel library)
  if (r > 0 || g > 0 || b > 0) {
    digitalWrite(RGB_LED_PIN, HIGH);
  } else {
    digitalWrite(RGB_LED_PIN, LOW);
  }
}