/*
 * ESP32-S3 HTTP Command Deep Debug
 * 
 * Tests ALL the HTTP AT command variants to find what works on C5
 */

#include <Arduino.h>

#define C5_TX_PIN   35
#define C5_RX_PIN   36
#define C5_EN_PIN   1
#define C5_BAUD_RATE 115200

const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// Test URLs - both HTTP and HTTPS
const char* TEST_URL_HTTP = "http://joinpatch.s3.eu-west-1.amazonaws.com/firmware.bin";
const char* TEST_URL_HTTPS = "https://joinpatch.s3.eu-west-1.amazonaws.com/firmware.bin";

// Simple test URL that definitely works (Espressif's own test server)
const char* TEST_URL_SIMPLE = "http://httpbin.org/bytes/100";

HardwareSerial C5Serial(2);

void clearC5() {
    delay(100);
    while (C5Serial.available()) C5Serial.read();
}

// Send command and show everything
void sendAndShow(const char* cmd, unsigned long timeout = 10000) {
    clearC5();
    Serial.printf("\n─────────────────────────────────────\n");
    Serial.printf(">>> %s\n", cmd);
    Serial.printf("─────────────────────────────────────\n");
    
    C5Serial.print(cmd);
    C5Serial.print("\r\n");
    C5Serial.flush();
    
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            Serial.write(c);
            
            // Reset timeout on activity
            start = millis();
        }
        yield();
    }
    Serial.println();
}

// Send raw data (for multi-line commands)
void sendRaw(const char* data) {
    Serial.printf(">>> (raw) %s\n", data);
    C5Serial.print(data);
    C5Serial.print("\r\n");
    C5Serial.flush();
    delay(500);
    while (C5Serial.available()) {
        Serial.write(C5Serial.read());
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n╔════════════════════════════════════════════╗");
    Serial.println("║  HTTP AT Command Deep Debug for C5         ║");
    Serial.println("╚════════════════════════════════════════════╝\n");
    
    // Power on C5
    pinMode(C5_EN_PIN, OUTPUT);
    digitalWrite(C5_EN_PIN, HIGH);
    Serial.println("[INIT] Waiting for C5 boot...");
    delay(3000);
    
    C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
    delay(500);
    clearC5();
    
    // Basic setup
    sendAndShow("AT", 2000);
    sendAndShow("AT+CWMODE=1", 2000);
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
    sendAndShow(cmd, 20000);
    
    sendAndShow("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"", 3000);
    delay(2000);
    
    // =========================================
    // TEST SUITE: HTTP Commands
    // =========================================
    
    Serial.println("\n\n╔════════════════════════════════════════════╗");
    Serial.println("║  TESTING HTTP COMMANDS                     ║");
    Serial.println("╚════════════════════════════════════════════╝");
    
    // Test 1: List available HTTP commands
    Serial.println("\n[TEST 1] List HTTP commands");
    sendAndShow("AT+HTTP?", 3000);
    sendAndShow("AT+HTTPCLIENT?", 3000);
    
    // Test 2: Check HTTP configuration
    Serial.println("\n[TEST 2] HTTP Configuration");
    sendAndShow("AT+HTTPCCONF?", 3000);
    sendAndShow("AT+HTTPURLCFG?", 3000);
    
    // Test 3: Try different HTTPCGET syntaxes
    Serial.println("\n[TEST 3] HTTPCGET syntax variants");
    
    // Variant A: Basic (what we tried)
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\"", TEST_URL_SIMPLE);
    sendAndShow(cmd, 15000);
    
    // Variant B: With transport type 1 (HTTP)
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\",1", TEST_URL_SIMPLE);
    sendAndShow(cmd, 15000);
    
    // Variant C: With more parameters
    snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\",1,0", TEST_URL_SIMPLE);
    sendAndShow(cmd, 15000);
    
    // Test 4: Try HTTPCLIENT command (different syntax)
    Serial.println("\n[TEST 4] HTTPCLIENT command");
    
    // AT+HTTPCLIENT=<opt>,<content-type>,<url>,[host],[path],[transport_type],[data],[headers]
    // opt: 1=HEAD, 2=GET, 3=POST, 4=PUT, 5=DELETE
    snprintf(cmd, sizeof(cmd), "AT+HTTPCLIENT=2,0,\"%s\",,,1", TEST_URL_SIMPLE);
    sendAndShow(cmd, 15000);
    
    // Try with just URL for httpbin
    sendAndShow("AT+HTTPCLIENT=2,0,\"http://httpbin.org/get\",,,1", 15000);
    
    // Test 5: Try setting URL separately then GET
    Serial.println("\n[TEST 5] Set URL then GET");
    snprintf(cmd, sizeof(cmd), "AT+HTTPURLCFG=%d", strlen(TEST_URL_HTTP));
    sendAndShow(cmd, 3000);
    sendRaw(TEST_URL_HTTP);
    delay(1000);
    sendAndShow("AT+HTTPCGET", 15000);
    
    // Test 6: Try your S3 URL specifically
    Serial.println("\n[TEST 6] Your S3 URL with different approaches");
    
    // HEAD request first
    snprintf(cmd, sizeof(cmd), "AT+HTTPCLIENT=1,0,\"%s\",,,1", TEST_URL_HTTP);
    sendAndShow(cmd, 15000);
    
    // GET request
    snprintf(cmd, sizeof(cmd), "AT+HTTPCLIENT=2,0,\"%s\",,,1", TEST_URL_HTTP);
    sendAndShow(cmd, 15000);
    
    // Test 7: Check if there's a size limit issue
    Serial.println("\n[TEST 7] Request small byte ranges from httpbin");
    sendAndShow("AT+HTTPCLIENT=2,0,\"http://httpbin.org/bytes/10\",,,1", 15000);
    sendAndShow("AT+HTTPCLIENT=2,0,\"http://httpbin.org/bytes/100\",,,1", 15000);
    
    // Test 8: Socket-based approach (alternative to HTTP client)
    Serial.println("\n[TEST 8] Raw TCP socket approach (if HTTP fails)");
    sendAndShow("AT+CIPSTART=\"TCP\",\"httpbin.org\",80", 10000);
    sendAndShow("AT+CIPSEND=47", 3000);
    sendRaw("GET /bytes/100 HTTP/1.1\r\nHost: httpbin.org\r\n\r\n");
    delay(3000);
    while (C5Serial.available()) Serial.write(C5Serial.read());
    sendAndShow("AT+CIPCLOSE", 3000);
    
    Serial.println("\n\n╔════════════════════════════════════════════╗");
    Serial.println("║  DEBUG COMPLETE                            ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println("\nReview output above to find which command syntax works.");
    Serial.println("\nEntering passthrough mode - type AT commands manually:");
}

void loop() {
    // Passthrough for manual testing
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            sendAndShow(cmd.c_str(), 15000);
        }
    }
    
    if (C5Serial.available()) {
        Serial.write(C5Serial.read());
    }
}
