/*
 * ESP32-S3 HTTP Range Request Test
 * 
 * SIMPLER TEST: Before attempting full OTA, this tests whether
 * the C5's AT+HTTPCGET command works with Range headers.
 * 
 * Run this first to verify the approach will work!
 */

#include <Arduino.h>

// Pin definitions
#define C5_TX_PIN   35
#define C5_RX_PIN   36
#define C5_EN_PIN   1

#define C5_BAUD_RATE 115200

// WiFi - UPDATE THESE
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// Test URL - your firmware file
const char* TEST_URL = "https://joinpatch.s3.eu-west-1.amazonaws.com/firmware.bin";

HardwareSerial C5Serial(2);

// Buffer for responses
char responseBuffer[8192];

void clearC5() {
    while (C5Serial.available()) C5Serial.read();
}

String sendAT(const char* cmd, unsigned long timeout = 5000) {
    clearC5();
    Serial.printf("\n>>> %s\n", cmd);
    C5Serial.print(cmd);
    C5Serial.print("\r\n");
    C5Serial.flush();
    
    String response = "";
    unsigned long start = millis();
    
    while (millis() - start < timeout) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            response += c;
            Serial.write(c);  // Echo to monitor
            
            if (response.endsWith("\r\nOK\r\n") || 
                response.endsWith("\r\nERROR\r\n") ||
                response.endsWith("SEND OK\r\n") ||
                response.endsWith("SEND FAIL\r\n")) {
                break;
            }
        }
        yield();
    }
    
    return response;
}

bool waitFor(const char* expected, unsigned long timeout = 10000) {
    String response = "";
    unsigned long start = millis();
    
    while (millis() - start < timeout) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            response += c;
            Serial.write(c);
            
            if (response.indexOf(expected) >= 0) {
                return true;
            }
        }
        yield();
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("  HTTP Range Request Test for C5");
    Serial.println("========================================\n");
    
    // Power on C5
    pinMode(C5_EN_PIN, OUTPUT);
    digitalWrite(C5_EN_PIN, HIGH);
    Serial.println("[INIT] C5 powered ON, waiting for boot...");
    delay(3000);
    
    // Init UART
    C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
    Serial.printf("[INIT] UART2: TX=%d, RX=%d\n", C5_TX_PIN, C5_RX_PIN);
    delay(500);
    clearC5();
    
    // Test basic AT
    Serial.println("\n--- TEST 1: Basic AT ---");
    sendAT("AT", 2000);
    
    // Check AT version
    Serial.println("\n--- TEST 2: AT Version ---");
    sendAT("AT+GMR", 3000);
    
    // Connect to WiFi
    Serial.println("\n--- TEST 3: WiFi Connection ---");
    sendAT("AT+CWMODE=1", 2000);
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
    C5Serial.print(cmd);
    C5Serial.print("\r\n");
    Serial.printf(">>> %s\n", cmd);
    
    if (!waitFor("WIFI GOT IP", 20000)) {
        Serial.println("\n[FAIL] WiFi connection failed!");
        return;
    }
    Serial.println("\n[OK] WiFi connected!");
    
    delay(1000);
    
    // Test SNTP
    Serial.println("\n--- TEST 4: SNTP Time Sync ---");
    sendAT("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"", 3000);
    delay(2000);
    sendAT("AT+CIPSNTPTIME?", 3000);
    
    // Test HTTP GET SIZE (if available)
    Serial.println("\n--- TEST 5: HTTP Get Size ---");
    snprintf(cmd, sizeof(cmd), "AT+HTTPGETSIZE=\"%s\"", TEST_URL);
    sendAT(cmd, 15000);
    
    // Test simple HTTP GET (first 100 bytes without range)
    Serial.println("\n--- TEST 6: Simple HTTP GET ---");
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\",10", TEST_URL);
    sendAT(cmd, 15000);
    
    // Test Range header
    Serial.println("\n--- TEST 7: Set Range Header ---");
    sendAT("AT+HTTPCHEAD=18", 2000);
    delay(100);
    clearC5();
    C5Serial.print("Range: bytes=0-99");
    C5Serial.print("\r\n");
    Serial.println(">>> Range: bytes=0-99");
    delay(500);
    
    String resp = "";
    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            resp += c;
            Serial.write(c);
        }
        yield();
    }
    
    // Test HTTP GET with Range
    Serial.println("\n--- TEST 8: HTTP GET with Range ---");
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\",30", TEST_URL);
    sendAT(cmd, 20000);
    
    // Test another range (bytes 100-199)
    Serial.println("\n--- TEST 9: Different Range (100-199) ---");
    sendAT("AT+HTTPCHEAD=20", 2000);
    delay(100);
    clearC5();
    C5Serial.print("Range: bytes=100-199");
    C5Serial.print("\r\n");
    Serial.println(">>> Range: bytes=100-199");
    waitFor("OK", 3000);
    
    delay(500);
    
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\",30", TEST_URL);
    sendAT(cmd, 20000);
    
    Serial.println("\n\n========================================");
    Serial.println("  TEST COMPLETE");
    Serial.println("========================================");
    Serial.println("\nLook at the responses above to determine:");
    Serial.println("1. Does AT+HTTPGETSIZE work? (returns file size)");
    Serial.println("2. Does AT+HTTPCGET return data?");
    Serial.println("3. Does setting Range header change the response?");
    Serial.println("4. Are bytes 0-99 different from bytes 100-199?");
    Serial.println("\nIf Range headers work, the full OTA should work.");
    Serial.println("If not, we need custom AT commands on C5.");
}

void loop() {
    // Pass through serial for manual testing
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            sendAT(cmd.c_str(), 10000);
        }
    }
    
    if (C5Serial.available()) {
        Serial.write(C5Serial.read());
    }
}
