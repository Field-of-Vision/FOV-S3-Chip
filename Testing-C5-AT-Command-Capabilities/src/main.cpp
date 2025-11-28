#include <Arduino.h>

// ============================================
// C5 MQTT Capability Check
// ============================================
// This checks what MQTT AT commands your C5 supports

#define C5_TX_PIN 35
#define C5_RX_PIN 36
#define C5_EN_PIN 1
#define C5_BAUD_RATE 115200

HardwareSerial C5Serial(2);

void sendATCommand(const char* command, unsigned long timeout = 3000);
String waitForResponse(unsigned long timeout);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32-C5 MQTT Capability Checker");
  Serial.println("========================================\n");
  
  pinMode(C5_EN_PIN, OUTPUT);
  digitalWrite(C5_EN_PIN, HIGH);
  Serial.println("[S3] C5 powered ON\n");
  
  delay(2000);
  
  C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
  Serial.println("[S3] UART2 initialized\n");
  
  delay(500);
  while(C5Serial.available()) C5Serial.read();
  
  Serial.println("Checking supported MQTT commands...\n");
  Serial.println("========================================\n");
  
  // Test basic MQTT commands
  Serial.println("1. Testing AT+MQTTUSERCFG");
  sendATCommand("AT+MQTTUSERCFG=?");
  delay(1000);
  
  Serial.println("\n2. Testing AT+MQTTCONNCFG");
  sendATCommand("AT+MQTTCONNCFG=?");
  delay(1000);
  
  Serial.println("\n3. Testing AT+MQTTCONN");
  sendATCommand("AT+MQTTCONN=?");
  delay(1000);
  
  Serial.println("\n4. Testing AT+MQTTSUB");
  sendATCommand("AT+MQTTSUB=?");
  delay(1000);
  
  Serial.println("\n5. Checking SSL/TLS support");
  sendATCommand("AT+CIPSSLCCONF=?");
  delay(1000);
  
  Serial.println("\n6. Checking certificate commands");
  sendATCommand("AT+SYSFLASH=?");
  delay(1000);
  
  Serial.println("\n7. Checking MQTT SSL configuration");
  sendATCommand("AT+MQTTSSLCCONF=?");
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("Capability check complete!");
  Serial.println("========================================\n");
  Serial.println("Check the responses above to see which");
  Serial.println("commands are supported by your C5 firmware.");
}

void loop() {
  // Echo any C5 messages
  if (C5Serial.available()) {
    Serial.write(C5Serial.read());
  }
}

void sendATCommand(const char* command, unsigned long timeout) {
  Serial.printf("[→] %s\n", command);
  
  while(C5Serial.available()) C5Serial.read();
  
  C5Serial.print(command);
  C5Serial.print("\r\n");
  C5Serial.flush();
  
  String response = waitForResponse(timeout);
  if (response.length() > 0) {
    Serial.println("[←]");
    Serial.println(response);
  } else {
    Serial.println("[←] (no response)");
  }
}

String waitForResponse(unsigned long timeout) {
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    if (C5Serial.available()) {
      char c = C5Serial.read();
      response += c;
      startTime = millis();
    }
  }
  
  return response;
}