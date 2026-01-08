/*
 * FOV OTA Example - Phase 3
 * 
 * Example ESP32-S3 firmware demonstrating OTA updates via ESP32-C5 WiFi modem.
 * 
 * Hardware Setup:
 *   ESP32-S3 UART1 TX (GPIO35) → ESP32-C5 RX
 *   ESP32-S3 UART1 RX (GPIO36) → ESP32-C5 TX
 *   Both chips share common GND
 * 
 * Usage:
 *   1. Flash this firmware to ESP32-S3
 *   2. Open Serial Monitor at 115200 baud
 *   3. Press 'o' to trigger OTA update
 *   4. Press 'c' to connect to WiFi
 *   5. Press 's' to check status
 */

#include <Arduino.h>
#include "ota_manager.h"

// ============ Configuration ============

// UART pins connected to ESP32-C5
#define C5_UART_TX_PIN  35
#define C5_UART_RX_PIN  36
#define C5_UART_BAUD    115200

// WiFi credentials
#define WIFI_SSID       "tim"
#define WIFI_PASSWORD   "password"

// Firmware URL (HTTP only, no HTTPS)
#define FIRMWARE_URL    "http://joinpatch.s3.eu-west-1.amazonaws.com/firmware.bin"

// ============ Globals ============

OTAManager* ota = nullptr;

// ============ Progress Callback ============

void otaProgressCallback(uint32_t downloaded, uint32_t total, const char* status) {
    if (total > 0) {
        float pct = (float)downloaded * 100.0f / total;
        Serial.printf("[OTA] %s: %lu / %lu bytes (%.1f%%)\n", 
                      status, (unsigned long)downloaded, (unsigned long)total, pct);
    } else {
        Serial.printf("[OTA] %s\n", status);
    }
}

// ============ Setup ============

void setup() {
    // Initialize debug serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n");
    Serial.println("============================================");
    Serial.println("FOV OTA Example - Phase 3");
    Serial.println("============================================");
    Serial.println();
    
    // Initialize UART to ESP32-C5
    Serial1.begin(C5_UART_BAUD, SERIAL_8N1, C5_UART_RX_PIN, C5_UART_TX_PIN);
    Serial.printf("C5 UART initialized: TX=%d, RX=%d, Baud=%d\n", 
                  C5_UART_TX_PIN, C5_UART_RX_PIN, C5_UART_BAUD);
    
    // Create OTA manager
    ota = new OTAManager(Serial1);
    ota->setProgressCallback(otaProgressCallback);
    
    // Wait for C5 to boot
    Serial.println("Waiting for C5 to boot...");
    delay(2000);
    
    // Test AT command
    Serial.println("Testing AT command...");
    Serial1.print("AT\r\n");
    delay(500);
    
    String response = "";
    while (Serial1.available()) {
        response += (char)Serial1.read();
    }
    
    if (response.indexOf("OK") >= 0) {
        Serial.println("C5 responding OK!");
    } else {
        Serial.println("WARNING: No response from C5");
        Serial.println("Response: " + response);
    }
    
    // Print menu
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  c - Connect to WiFi");
    Serial.println("  s - Check status");
    Serial.println("  o - Start OTA update");
    Serial.println("  m - Check memory");
    Serial.println("  r - Restart");
    Serial.println();
}

// ============ Command Handlers ============

void handleConnectWiFi() {
    Serial.println("\nConnecting to WiFi...");
    Serial.printf("SSID: %s\n", WIFI_SSID);
    
    if (ota->connectWiFi(WIFI_SSID, WIFI_PASSWORD, 30000)) {
        Serial.println("WiFi connected!");
    } else {
        Serial.println("WiFi connection failed!");
    }
}

void handleCheckStatus() {
    Serial.println("\n--- Status Check ---");
    
    // Check WiFi
    Serial.print("WiFi: ");
    if (ota->isWiFiConnected()) {
        Serial.println("Connected");
    } else {
        Serial.println("Not connected");
    }
    
    // Check firmware buffer
    uint32_t fwSize, fwCrc;
    Serial.print("Firmware buffer: ");
    if (ota->getFirmwareInfo(&fwSize, &fwCrc)) {
        Serial.printf("READY - %lu bytes, CRC: 0x%08lX\n", 
                      (unsigned long)fwSize, (unsigned long)fwCrc);
    } else {
        Serial.println("Empty or not ready");
    }
    
    // Check current partition
    const esp_partition_t* running = esp_ota_get_running_partition();
    Serial.printf("Running partition: %s\n", running->label);
    
    const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
    if (next) {
        Serial.printf("Next OTA partition: %s (size: %lu bytes)\n", 
                      next->label, (unsigned long)next->size);
    }
    
    Serial.println("--------------------");
}

void handleCheckMemory() {
    Serial.println("\nChecking C5 memory...");
    
    Serial1.print("AT+FWMEMINFO?\r\n");
    delay(500);
    
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
    Serial.println();
}

void handleStartOTA() {
    Serial.println("\n========================================");
    Serial.println("Starting OTA Update");
    Serial.println("========================================");
    Serial.printf("URL: %s\n", FIRMWARE_URL);
    Serial.println();
    
    // Check WiFi first
    if (!ota->isWiFiConnected()) {
        Serial.println("WiFi not connected. Connecting...");
        if (!ota->connectWiFi(WIFI_SSID, WIFI_PASSWORD, 30000)) {
            Serial.println("ERROR: WiFi connection failed!");
            return;
        }
    }
    
    // Perform OTA
    unsigned long startTime = millis();
    ota_error_t result = ota->performOTA(FIRMWARE_URL);
    unsigned long elapsed = millis() - startTime;
    
    Serial.println();
    Serial.println("========================================");
    
    if (result == OTA_OK) {
        Serial.println("OTA UPDATE SUCCESSFUL!");
        Serial.printf("Total time: %.1f seconds\n", elapsed / 1000.0f);
        Serial.println("Restarting in 3 seconds...");
        Serial.println("========================================");
        
        delay(3000);
        ESP.restart();
    } else {
        Serial.printf("OTA FAILED: %s\n", OTAManager::errorToString(result));
        Serial.println("========================================");
    }
}

// ============ Main Loop ============

void loop() {
    // Check for commands from Serial Monitor
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'c':
            case 'C':
                handleConnectWiFi();
                break;
                
            case 's':
            case 'S':
                handleCheckStatus();
                break;
                
            case 'o':
            case 'O':
                handleStartOTA();
                break;
                
            case 'm':
            case 'M':
                handleCheckMemory();
                break;
                
            case 'r':
            case 'R':
                Serial.println("Restarting...");
                delay(500);
                ESP.restart();
                break;
                
            case '\n':
            case '\r':
                // Ignore newlines
                break;
                
            default:
                Serial.println("\nCommands: c=WiFi, s=Status, o=OTA, m=Memory, r=Restart");
                break;
        }
    }
    
    // Forward any data from C5 to debug serial (for debugging)
    while (Serial1.available()) {
        char c = Serial1.read();
        Serial.write(c);
    }
    
    delay(10);
}
