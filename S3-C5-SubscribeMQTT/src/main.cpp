#include <Arduino.h>

// ============================================
// S3 MQTT Relay Test via C5 AT Commands
// SIMPLIFIED - NO FLOW CONTROL
// Sample firmmware for subscribing to MQTT topic via C5 module from firmware running on ESP32-S3
// ============================================

// Pin definitions for C5 communication (UART2)
#define C5_TX_PIN   35  // S3 GPIO35 (U2_TXD) -> C5 GPIO4 (U1_RXD)
#define C5_RX_PIN   36  // S3 GPIO36 (U2_RXD) -> C5 GPIO5 (U1_TXD)
#define C5_EN_PIN   1   // S3 GPIO1 controls C5 EN pin

// UART configuration
#define C5_BAUD_RATE 115200

// WiFi credentials
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";  // UPDATE THIS

// AWS IoT configuration
const char* AWS_IOT_ENDPOINT = "a3lkzcadhi1yzr-ats.iot.eu-west-1.amazonaws.com";
const char* AWS_IOT_CLIENT_ID = "aviva-fov-tablet-1";
const char* MQTT_TOPIC = "dalymount_IRL/pub";

// Create Serial2 object for C5 communication
HardwareSerial C5Serial(2);

// Message tracking
int32_t expectedSequence = -1;
uint32_t messagesReceived = 0;
uint32_t messagesOutOfOrder = 0;
uint32_t messagesMissed = 0;

// Timing
unsigned long lastMessageTime = 0;
float avgInterval = 0;

// State machine
enum SystemState {
  STATE_INIT,
  STATE_WIFI_CONNECTING,
  STATE_SNTP_SYNC,
  STATE_MQTT_CONFIGURING,
  STATE_MQTT_CONNECTING,
  STATE_MQTT_SUBSCRIBING,
  STATE_RUNNING,
  STATE_ERROR
};

SystemState currentState = STATE_INIT;

// Buffer for parsing
#define RX_BUFFER_SIZE 512
char rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex = 0;

// Function declarations
bool sendATCommandAndWait(const char* command, const char* expectedResponse, unsigned long timeout = 5000);
void processIncomingData();
void parseMQTTMessage(const char* line);
void handleMQTTPayload(const char* topic, int length, const char* payload);
void printStatistics();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║  ESP32-S3 MQTT Relay Test via C5           ║");
  Serial.println("║  Simplified - No Flow Control              ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  // Power on C5
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH);
  Serial.println("[S3] C5 module powered ON");
  
  delay(2000);  // Wait for C5 to boot
  
  // Initialize UART2
  C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
  Serial.printf("[S3] UART2: TX=GPIO%d, RX=GPIO%d\n", C5_TX_PIN, C5_RX_PIN);
  
  delay(500);
  
  // Clear boot messages
  while(C5Serial.available()) {
    C5Serial.read();
  }
  
  rxBufferIndex = 0;
}

void loop() {
  static unsigned long lastStatsPrint = 0;
  
  // Process incoming data from C5
  processIncomingData();
  
  // State machine
  switch (currentState) {
    case STATE_INIT:
      Serial.println("[STATE] Initializing...");
      delay(500);
      if (sendATCommandAndWait("AT", "OK", 2000)) {
        Serial.println("[OK] C5 responding!");
        currentState = STATE_WIFI_CONNECTING;
      } else {
        Serial.println("[ERROR] C5 not responding, retrying...");
        delay(1000);
      }
      break;
      
    case STATE_WIFI_CONNECTING:
      Serial.println("[STATE] Connecting to WiFi...");
      sendATCommandAndWait("AT+CWMODE=1", "OK", 2000);
      {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
        if (sendATCommandAndWait(cmd, "WIFI GOT IP", 20000)) {
          Serial.println("[OK] WiFi connected!");
          currentState = STATE_SNTP_SYNC;
        } else {
          Serial.println("[ERROR] WiFi failed, retrying...");
          delay(5000);
        }
      }
      break;
      
    case STATE_SNTP_SYNC:
      Serial.println("[STATE] Syncing time...");
      sendATCommandAndWait("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"", "OK", 3000);
      delay(3000);
      sendATCommandAndWait("AT+CIPSNTPTIME?", "OK", 3000);
      Serial.println("[OK] Time synced!");
      currentState = STATE_MQTT_CONFIGURING;
      break;
      
    case STATE_MQTT_CONFIGURING:
      Serial.println("[STATE] Configuring MQTT...");
      {
        char cmd[256];
        
        // Scheme 5 = MQTT over TLS with mutual auth
        snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,5,\"%s\",\"\",\"\",0,0,\"\"", AWS_IOT_CLIENT_ID);
        sendATCommandAndWait(cmd, "OK", 3000);
        
        // Set SNI
        snprintf(cmd, sizeof(cmd), "AT+MQTTSNI=0,\"%s\"", AWS_IOT_ENDPOINT);
        sendATCommandAndWait(cmd, "OK", 2000);
        
        currentState = STATE_MQTT_CONNECTING;
      }
      break;
      
    case STATE_MQTT_CONNECTING:
      Serial.println("[STATE] Connecting to AWS IoT...");
      {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",8883,1", AWS_IOT_ENDPOINT);
        if (sendATCommandAndWait(cmd, "MQTTCONNECTED", 20000)) {
          Serial.println("[OK] MQTT connected!");
          currentState = STATE_MQTT_SUBSCRIBING;
        } else {
          Serial.println("[ERROR] MQTT failed, retrying...");
          delay(5000);
        }
      }
      break;
      
    case STATE_MQTT_SUBSCRIBING:
      Serial.println("[STATE] Subscribing to topic...");
      {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",0", MQTT_TOPIC);
        if (sendATCommandAndWait(cmd, "OK", 5000)) {
          currentState = STATE_RUNNING;
          Serial.println("\n╔════════════════════════════════════════════╗");
          Serial.println("║  READY - Waiting for messages...           ║");
          Serial.printf("║  Topic: %-34s ║\n", MQTT_TOPIC);
          Serial.println("╚════════════════════════════════════════════╝\n");
          lastMessageTime = millis();
        } else {
          Serial.println("[ERROR] Subscribe failed, retrying...");
          delay(2000);
        }
      }
      break;
      
    case STATE_RUNNING:
      // Print stats every 5 seconds
      if (millis() - lastStatsPrint >= 5000) {
        lastStatsPrint = millis();
        printStatistics();
      }
      break;
      
    case STATE_ERROR:
      Serial.println("[ERROR] Resetting in 5 seconds...");
      delay(5000);
      currentState = STATE_INIT;
      break;
  }
}

bool sendATCommandAndWait(const char* command, const char* expectedResponse, unsigned long timeout) {
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
      
      if (response.indexOf(expectedResponse) >= 0) {
        Serial.printf("[RX] %s\n", response.c_str());
        return true;
      }
      
      if (response.indexOf("ERROR") >= 0) {
        Serial.printf("[RX] %s\n", response.c_str());
        return false;
      }
    }
  }
  
  Serial.printf("[RX] (timeout) %s\n", response.c_str());
  return false;
}

void processIncomingData() {
  while (C5Serial.available()) {
    char c = C5Serial.read();
    
    if (rxBufferIndex < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxBufferIndex++] = c;
      rxBuffer[rxBufferIndex] = '\0';
    }
    
    if (c == '\n') {
      rxBuffer[rxBufferIndex - 1] = '\0';
      if (rxBufferIndex > 1 && rxBuffer[rxBufferIndex - 2] == '\r') {
        rxBuffer[rxBufferIndex - 2] = '\0';
      }
      
      if (strncmp(rxBuffer, "+MQTTSUBRECV:", 13) == 0) {
        parseMQTTMessage(rxBuffer);
      } else if (strlen(rxBuffer) > 0) {
        Serial.printf("[C5] %s\n", rxBuffer);
      }
      
      rxBufferIndex = 0;
    }
  }
}

void parseMQTTMessage(const char* line) {
  char topic[128];
  char payload[256];
  
  const char* p = line + 13;
  
  const char* topicStart = strchr(p, '"');
  if (!topicStart) return;
  topicStart++;
  
  const char* topicEnd = strchr(topicStart, '"');
  if (!topicEnd) return;
  
  int topicLen = topicEnd - topicStart;
  if (topicLen >= (int)sizeof(topic)) topicLen = sizeof(topic) - 1;
  strncpy(topic, topicStart, topicLen);
  topic[topicLen] = '\0';
  
  const char* lengthStart = topicEnd + 2;
  int dataLength = atoi(lengthStart);
  
  const char* payloadStart = strchr(lengthStart, ',');
  if (!payloadStart) return;
  payloadStart++;
  
  strncpy(payload, payloadStart, sizeof(payload) - 1);
  payload[sizeof(payload) - 1] = '\0';
  
  handleMQTTPayload(topic, dataLength, payload);
}

void handleMQTTPayload(const char* topic, int length, const char* payload) {
  unsigned long now = millis();
  unsigned long interval = now - lastMessageTime;
  lastMessageTime = now;
  
  if (messagesReceived > 0) {
    avgInterval = avgInterval * 0.9 + interval * 0.1;
  } else {
    avgInterval = interval;
  }
  
  messagesReceived++;
  
  int32_t sequence = atoi(payload);
  
  bool inOrder = true;
  
  if (expectedSequence == -1) {
    expectedSequence = sequence + 1;
    Serial.printf("[MSG #%lu] First message: %d\n", messagesReceived, sequence);
  } else if (sequence == expectedSequence) {
    expectedSequence = sequence + 1;
  } else if (sequence > expectedSequence) {
    int missed = sequence - expectedSequence;
    messagesMissed += missed;
    messagesOutOfOrder++;
    expectedSequence = sequence + 1;
    inOrder = false;
    Serial.printf("[MSG #%lu] ⚠️ MISSED %d! Got %d, expected %d\n", 
                  messagesReceived, missed, sequence, expectedSequence - missed - 1);
  } else {
    messagesOutOfOrder++;
    inOrder = false;
    Serial.printf("[MSG #%lu] ⚠️ OUT OF ORDER: Got %d, expected %d\n", 
                  messagesReceived, sequence, expectedSequence);
  }
  
  if (inOrder && messagesReceived % 10 == 0) {
    Serial.printf("[MSG #%lu] seq=%d, avg=%.1fms (%.1f FPS)\n", 
                  messagesReceived, sequence, avgInterval, 1000.0/avgInterval);
  }
}

void printStatistics() {
  if (currentState != STATE_RUNNING) return;
  
  Serial.println("\n┌─────────────── Statistics ───────────────┐");
  Serial.printf("│ Messages received:     %-17lu │\n", messagesReceived);
  Serial.printf("│ Messages out of order: %-17lu │\n", messagesOutOfOrder);
  Serial.printf("│ Messages missed:       %-17lu │\n", messagesMissed);
  if (messagesReceived > 0) {
    float successRate = 100.0 * (messagesReceived - messagesOutOfOrder) / messagesReceived;
    Serial.printf("│ Success rate:          %-16.2f%% │\n", successRate);
    Serial.printf("│ Average interval:      %-14.1f ms │\n", avgInterval);
    Serial.printf("│ Effective FPS:         %-17.1f │\n", 1000.0/avgInterval);
  }
  Serial.println("└───────────────────────────────────────────┘\n");
}