/*
 * ESP32-S3 Streaming OTA Update via C5 WiFi Module
 * 
 * Strategy: 
 *   1. Send AT+HTTPCGET to C5
 *   2. Parse +HTTPCGET:<size>, header
 *   3. Read binary data directly from UART
 *   4. Write to OTA partition as chunks arrive
 *   5. Verify and reboot
 * 
 * No custom C5 firmware needed - uses standard ESP-AT HTTP commands.
 */

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_app_format.h>

// ============================================
// Pin Definitions
// ============================================
#define C5_TX_PIN   35  // S3 GPIO35 -> C5 RX
#define C5_RX_PIN   36  // S3 GPIO36 -> C5 TX  
#define C5_EN_PIN   1   // S3 GPIO1 -> C5 EN
#define LED_PIN     38  // Onboard RGB LED (optional)

// ============================================
// Configuration
// ============================================
#define C5_BAUD_RATE     115200
#define OTA_WRITE_SIZE   4096      // Write to flash in 4KB chunks
#define AT_TIMEOUT       10000     // 10s for normal AT commands
#define HTTP_TIMEOUT     60000     // 60s for HTTP operations
#define WIFI_TIMEOUT     20000     // 20s for WiFi connection

// WiFi credentials - UPDATE THESE
const char* WIFI_SSID = "tim";
const char* WIFI_PASSWORD = "password";

// Firmware URL - UPDATE THIS
const char* FIRMWARE_URL = "http://joinpatch.s3.eu-west-1.amazonaws.com/firmware.bin";

// ============================================
// Global Objects
// ============================================
HardwareSerial C5Serial(2);

// OTA state
esp_ota_handle_t otaHandle = 0;
const esp_partition_t* updatePartition = NULL;
size_t totalFirmwareSize = 0;
size_t bytesReceived = 0;
size_t bytesWritten = 0;

// Write buffer - accumulate data before writing to flash
uint8_t writeBuffer[OTA_WRITE_SIZE];
size_t writeBufferPos = 0;

// ============================================
// State Machine
// ============================================
enum OTAState {
    STATE_INIT,
    STATE_WAIT_C5_BOOT,
    STATE_CHECK_C5,
    STATE_WIFI_CONNECT,
    STATE_SNTP_SYNC,
    STATE_GET_FILE_SIZE,
    STATE_OTA_BEGIN,
    STATE_START_DOWNLOAD,
    STATE_DOWNLOADING,
    STATE_FLUSH_BUFFER,
    STATE_OTA_FINALIZE,
    STATE_OTA_COMPLETE,
    STATE_IDLE,
    STATE_ERROR
};

OTAState currentState = STATE_INIT;
unsigned long stateStartTime = 0;
String lastError = "";

// ============================================
// Utility Functions
// ============================================

void clearC5Buffer() {
    while (C5Serial.available()) {
        C5Serial.read();
    }
}

void printProgress() {
    if (totalFirmwareSize > 0) {
        float percent = (float)bytesReceived / totalFirmwareSize * 100.0;
        Serial.printf("\r[PROGRESS] %u / %u bytes (%.1f%%)    ", 
                      bytesReceived, totalFirmwareSize, percent);
    }
}

void printPartitionInfo() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    
    Serial.println("\n┌─────────────── Partition Info ───────────────┐");
    Serial.printf("│ Running:  %-36s │\n", running ? running->label : "Unknown");
    Serial.printf("│ Next OTA: %-36s │\n", next ? next->label : "None");
    if (running) {
        Serial.printf("│ Address:  0x%08lx  Size: %-15lu │\n", running->address, running->size);
    }
    Serial.println("└───────────────────────────────────────────────┘");
}

// ============================================
// AT Command Functions
// ============================================

// Send AT command and wait for expected response
bool sendATCommand(const char* cmd, const char* expected, unsigned long timeout = AT_TIMEOUT) {
    clearC5Buffer();
    
    Serial.printf("[AT TX] %s\n", cmd);
    C5Serial.print(cmd);
    C5Serial.print("\r\n");
    C5Serial.flush();
    
    String response = "";
    unsigned long start = millis();
    
    while (millis() - start < timeout) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            response += c;
            
            if (response.indexOf(expected) >= 0) {
                Serial.printf("[AT RX] OK - Found '%s'\n", expected);
                return true;
            }
            if (response.indexOf("ERROR") >= 0 && strcmp(expected, "ERROR") != 0) {
                Serial.printf("[AT RX] ERROR in response\n");
                return false;
            }
        }
        yield();
    }
    
    Serial.printf("[AT RX] TIMEOUT waiting for '%s'\n", expected);
    return false;
}

// Send AT command and get full response
String sendATCommandGetResponse(const char* cmd, unsigned long timeout = AT_TIMEOUT) {
    clearC5Buffer();
    
    Serial.printf("[AT TX] %s\n", cmd);
    C5Serial.print(cmd);
    C5Serial.print("\r\n");
    C5Serial.flush();
    
    String response = "";
    unsigned long start = millis();
    
    while (millis() - start < timeout) {
        if (C5Serial.available()) {
            char c = C5Serial.read();
            response += c;
            
            if (response.indexOf("\r\nOK\r\n") >= 0 || 
                response.indexOf("\r\nERROR\r\n") >= 0) {
                break;
            }
        }
        yield();
    }
    
    return response;
}

// ============================================
// OTA Functions  
// ============================================

bool beginOTA() {
    updatePartition = esp_ota_get_next_update_partition(NULL);
    if (!updatePartition) {
        lastError = "No OTA partition available";
        return false;
    }
    
    Serial.printf("[OTA] Target partition: %s at 0x%08lx (%lu bytes)\n",
                  updatePartition->label, updatePartition->address, updatePartition->size);
    
    if (totalFirmwareSize > updatePartition->size) {
        lastError = "Firmware too large for partition";
        return false;
    }
    
    esp_err_t err = esp_ota_begin(updatePartition, totalFirmwareSize, &otaHandle);
    if (err != ESP_OK) {
        lastError = String("esp_ota_begin failed: ") + esp_err_to_name(err);
        return false;
    }
    
    bytesReceived = 0;
    bytesWritten = 0;
    writeBufferPos = 0;
    
    Serial.println("[OTA] OTA begin successful");
    return true;
}

bool writeToOTA(uint8_t* data, size_t len) {
    esp_err_t err = esp_ota_write(otaHandle, data, len);
    if (err != ESP_OK) {
        lastError = String("esp_ota_write failed: ") + esp_err_to_name(err);
        return false;
    }
    bytesWritten += len;
    return true;
}

bool finalizeOTA() {
    esp_err_t err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
        lastError = String("esp_ota_end failed: ") + esp_err_to_name(err);
        return false;
    }
    
    err = esp_ota_set_boot_partition(updatePartition);
    if (err != ESP_OK) {
        lastError = String("esp_ota_set_boot_partition failed: ") + esp_err_to_name(err);
        return false;
    }
    
    Serial.printf("\n[OTA] Success! Wrote %u bytes\n", bytesWritten);
    return true;
}

void abortOTA() {
    if (otaHandle) {
        esp_ota_abort(otaHandle);
        otaHandle = 0;
    }
}

// ============================================
// HTTP Download State Machine
// ============================================

// States for parsing HTTP response
enum DownloadState {
    DL_WAIT_PLUS,           // Waiting for '+' of +HTTPCGET
    DL_PARSE_HEADER,        // Parsing "+HTTPCGET:<size>,"
    DL_READ_DATA,           // Reading binary data
    DL_COMPLETE,            // Download complete
    DL_ERROR                // Error occurred
};

DownloadState dlState = DL_WAIT_PLUS;
String headerBuffer = "";
size_t expectedDataSize = 0;

void resetDownloadState() {
    dlState = DL_WAIT_PLUS;
    headerBuffer = "";
    expectedDataSize = 0;
    bytesReceived = 0;
    writeBufferPos = 0;
}

// Debug buffer to capture what C5 returns
String debugBuffer = "";
bool debugMode = true;

// Process incoming data from C5 during download
// Returns true when download is complete
bool processDownloadData() {
    while (C5Serial.available()) {
        uint8_t c = C5Serial.read();
        
        // Capture first 500 bytes for debugging
        if (debugMode && debugBuffer.length() < 500) {
            if (c >= 32 && c < 127) {
                debugBuffer += (char)c;
            } else {
                debugBuffer += "[0x";
                debugBuffer += String(c, HEX);
                debugBuffer += "]";
            }
        }
        
        switch (dlState) {
            case DL_WAIT_PLUS:
                if (c == '+') {
                    headerBuffer = "+";
                    dlState = DL_PARSE_HEADER;
                }
                // Check for ERROR before we even see +HTTPCGET
                if (c == 'E' || headerBuffer.length() > 0) {
                    if (c == 'E') headerBuffer = "E";
                    else if (headerBuffer.length() > 0 && headerBuffer.length() < 10) {
                        headerBuffer += (char)c;
                        if (headerBuffer.indexOf("ERROR") >= 0) {
                            Serial.printf("\n[DEBUG] Raw response:\n%s\n", debugBuffer.c_str());
                            lastError = "HTTP ERROR received";
                            dlState = DL_ERROR;
                            return true;
                        }
                        if (headerBuffer.length() >= 5 && headerBuffer.indexOf("ERROR") < 0) {
                            headerBuffer = ""; // Reset if not ERROR
                        }
                    }
                }
                break;
                
            case DL_PARSE_HEADER:
                headerBuffer += (char)c;
                
                // Check for error
                if (headerBuffer.indexOf("ERROR") >= 0) {
                    Serial.printf("\n[DEBUG] Raw response:\n%s\n", debugBuffer.c_str());
                    lastError = "HTTP request failed";
                    dlState = DL_ERROR;
                    return true;
                }
                
                // Look for complete header: +HTTPCGET:<size>,
                if (headerBuffer.startsWith("+HTTPCGET:") && c == ',') {
                    // Parse size from header
                    int colonIdx = headerBuffer.indexOf(':');
                    int commaIdx = headerBuffer.indexOf(',');
                    if (colonIdx > 0 && commaIdx > colonIdx) {
                        String sizeStr = headerBuffer.substring(colonIdx + 1, commaIdx);
                        expectedDataSize = sizeStr.toInt();
                        
                        Serial.printf("\n[HTTP] Starting download: %u bytes\n", expectedDataSize);
                        
                        if (expectedDataSize == 0) {
                            lastError = "Invalid data size in response";
                            dlState = DL_ERROR;
                            return true;
                        }
                        
                        // Verify against expected firmware size
                        if (totalFirmwareSize > 0 && expectedDataSize != totalFirmwareSize) {
                            Serial.printf("[WARN] Size mismatch: expected %u, got %u\n", 
                                         totalFirmwareSize, expectedDataSize);
                        }
                        
                        totalFirmwareSize = expectedDataSize;
                        dlState = DL_READ_DATA;
                    }
                }
                
                // Safety: don't let header buffer grow too large
                if (headerBuffer.length() > 50) {
                    headerBuffer = headerBuffer.substring(headerBuffer.length() - 20);
                }
                break;
                
            case DL_READ_DATA:
                // Add byte to write buffer
                writeBuffer[writeBufferPos++] = c;
                bytesReceived++;
                
                // When buffer is full, write to OTA partition
                if (writeBufferPos >= OTA_WRITE_SIZE) {
                    if (!writeToOTA(writeBuffer, writeBufferPos)) {
                        dlState = DL_ERROR;
                        return true;
                    }
                    writeBufferPos = 0;
                    printProgress();
                }
                
                // Check if download is complete
                if (bytesReceived >= expectedDataSize) {
                    Serial.printf("\n[HTTP] Download complete: %u bytes received\n", bytesReceived);
                    dlState = DL_COMPLETE;
                    return true;
                }
                break;
                
            case DL_COMPLETE:
            case DL_ERROR:
                return true;
        }
    }
    
    return false;
}

// ============================================
// Main State Machine
// ============================================

void runStateMachine() {
    switch (currentState) {
        case STATE_INIT:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║  ESP32-S3 Streaming OTA via C5 WiFi        ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            printPartitionInfo();
            
            // Mark current app as valid (for rollback support)
            esp_ota_mark_app_valid_cancel_rollback();
            
            // Power on C5
            pinMode(C5_EN_PIN, OUTPUT);
            digitalWrite(C5_EN_PIN, HIGH);
            Serial.println("[INIT] C5 module powered ON");
            
            stateStartTime = millis();
            currentState = STATE_WAIT_C5_BOOT;
            break;
            
        case STATE_WAIT_C5_BOOT:
            if (millis() - stateStartTime >= 3000) {
                // Initialize UART
                C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
                Serial.printf("[INIT] UART2: TX=%d, RX=%d, Baud=%d\n", 
                              C5_TX_PIN, C5_RX_PIN, C5_BAUD_RATE);
                delay(500);
                clearC5Buffer();
                currentState = STATE_CHECK_C5;
            }
            break;
            
        case STATE_CHECK_C5:
            Serial.println("[STATE] Checking C5 communication...");
            if (sendATCommand("AT", "OK", 2000)) {
                Serial.println("[OK] C5 responding");
                
                // Get firmware version
                String gmr = sendATCommandGetResponse("AT+GMR", 3000);
                Serial.printf("[INFO] C5 Firmware:\n%s\n", gmr.c_str());
                
                currentState = STATE_WIFI_CONNECT;
            } else {
                lastError = "C5 not responding";
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_WIFI_CONNECT:
            Serial.println("[STATE] Connecting to WiFi...");
            
            sendATCommand("AT+CWMODE=1", "OK", 2000);
            
            {
                char cmd[128];
                snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
                if (sendATCommand(cmd, "WIFI GOT IP", WIFI_TIMEOUT)) {
                    Serial.println("[OK] WiFi connected!");
                    currentState = STATE_SNTP_SYNC;
                } else {
                    lastError = "WiFi connection failed";
                    currentState = STATE_ERROR;
                }
            }
            break;
            
        case STATE_SNTP_SYNC:
            Serial.println("[STATE] Syncing time (for HTTPS if needed)...");
            sendATCommand("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"", "OK", 3000);
            delay(2000);  // Give time for sync
            
            {
                String timeResp = sendATCommandGetResponse("AT+CIPSNTPTIME?", 3000);
                Serial.printf("[INFO] Time: %s", timeResp.c_str());
            }
            
            currentState = STATE_GET_FILE_SIZE;
            break;
            
        case STATE_GET_FILE_SIZE:
            Serial.println("[STATE] Getting firmware file size...");
            {
                char cmd[256];
                snprintf(cmd, sizeof(cmd), "AT+HTTPGETSIZE=\"%s\"", FIRMWARE_URL);
                String response = sendATCommandGetResponse(cmd, HTTP_TIMEOUT);
                
                // Parse +HTTPGETSIZE:<size>
                int idx = response.indexOf("+HTTPGETSIZE:");
                if (idx >= 0) {
                    int endIdx = response.indexOf('\r', idx);
                    if (endIdx > idx) {
                        String sizeStr = response.substring(idx + 13, endIdx);
                        totalFirmwareSize = sizeStr.toInt();
                        Serial.printf("[INFO] Firmware size: %u bytes\n", totalFirmwareSize);
                        currentState = STATE_OTA_BEGIN;
                    } else {
                        lastError = "Failed to parse file size";
                        currentState = STATE_ERROR;
                    }
                } else {
                    lastError = "HTTPGETSIZE failed";
                    currentState = STATE_ERROR;
                }
            }
            break;
            
        case STATE_OTA_BEGIN:
            Serial.println("[STATE] Initializing OTA partition...");
            if (beginOTA()) {
                currentState = STATE_START_DOWNLOAD;
            } else {
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_START_DOWNLOAD:
            Serial.println("[STATE] Starting firmware download...");
            resetDownloadState();
            
            {
                // Send the HTTP GET command
                char cmd[256];
                snprintf(cmd, sizeof(cmd), "AT+HTTPCGET=\"%s\"", FIRMWARE_URL);
                
                clearC5Buffer();
                Serial.printf("[AT TX] %s\n", cmd);
                C5Serial.print(cmd);
                C5Serial.print("\r\n");
                C5Serial.flush();
                
                stateStartTime = millis();
                currentState = STATE_DOWNLOADING;
            }
            break;
            
        case STATE_DOWNLOADING:
            // Process incoming data
            if (processDownloadData()) {
                // Download finished (success or error)
                if (dlState == DL_COMPLETE) {
                    currentState = STATE_FLUSH_BUFFER;
                } else {
                    abortOTA();
                    currentState = STATE_ERROR;
                }
            }
            
            // Timeout check
            if (millis() - stateStartTime > HTTP_TIMEOUT + (totalFirmwareSize / 10)) {
                // Dynamic timeout based on file size
                lastError = "Download timeout";
                abortOTA();
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_FLUSH_BUFFER:
            Serial.println("\n[STATE] Flushing remaining data...");
            
            // Write any remaining data in buffer
            if (writeBufferPos > 0) {
                if (!writeToOTA(writeBuffer, writeBufferPos)) {
                    abortOTA();
                    currentState = STATE_ERROR;
                    break;
                }
                Serial.printf("[OTA] Flushed final %u bytes\n", writeBufferPos);
                writeBufferPos = 0;
            }
            
            currentState = STATE_OTA_FINALIZE;
            break;
            
        case STATE_OTA_FINALIZE:
            Serial.println("[STATE] Finalizing OTA update...");
            
            // Consume any trailing data from C5 (like \r\nOK\r\n)
            delay(500);
            clearC5Buffer();
            
            if (finalizeOTA()) {
                currentState = STATE_OTA_COMPLETE;
            } else {
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_OTA_COMPLETE:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║         OTA UPDATE SUCCESSFUL!             ║");
            Serial.println("╠════════════════════════════════════════════╣");
            Serial.printf("║  Downloaded: %-28u bytes ║\n", bytesReceived);
            Serial.printf("║  Written:    %-28u bytes ║\n", bytesWritten);
            Serial.println("║                                            ║");
            Serial.println("║  Rebooting in 3 seconds...                 ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            delay(3000);
            esp_restart();
            break;
            
        case STATE_IDLE:
            // Do nothing - OTA not triggered
            break;
            
        case STATE_ERROR:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║           OTA UPDATE FAILED                ║");
            Serial.println("╠════════════════════════════════════════════╣");
            Serial.printf("║  Error: %-34s ║\n", lastError.c_str());
            Serial.printf("║  Received: %-31u bytes ║\n", bytesReceived);
            Serial.printf("║  Written:  %-31u bytes ║\n", bytesWritten);
            Serial.println("║                                            ║");
            Serial.println("║  Continuing with current firmware...       ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            currentState = STATE_IDLE;
            break;
    }
}

// ============================================
// Setup and Loop
// ============================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Print current firmware info
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_app_desc_t app_desc;
    if (esp_ota_get_partition_description(running, &app_desc) == ESP_OK) {
        Serial.printf("\n[BOOT] App: %s\n", app_desc.project_name);
        Serial.printf("[BOOT] Version: %s\n", app_desc.version);
        Serial.printf("[BOOT] Compiled: %s %s\n", app_desc.date, app_desc.time);
    }
    Serial.printf("[BOOT] Running from: %s\n", running ? running->label : "Unknown");
}

void loop() {
    runStateMachine();
    
    // Small delay to prevent tight loop
    delay(10);
}