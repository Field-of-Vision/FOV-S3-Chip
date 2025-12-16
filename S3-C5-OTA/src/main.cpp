/*
 * ESP32-S3 Streaming OTA Update via C5 WiFi Module
 * 
 * IMPORTANT: AT+HTTPCGET returns data in CHUNKS, not all at once!
 * Response format:
 *   +HTTPCGET:<chunk_size>,<binary data>
 *   +HTTPCGET:<chunk_size>,<binary data>
 *   ... (repeats until complete)
 *   OK
 */

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <esp_app_format.h>

// ============================================
// Pin Definitions
// ============================================
#define C5_TX_PIN   35
#define C5_RX_PIN   36
#define C5_EN_PIN   1

// ============================================
// Configuration
// ============================================
#define C5_BAUD_RATE     115200
#define OTA_WRITE_SIZE   4096
#define AT_TIMEOUT       10000
#define HTTP_TIMEOUT     120000   // 2 minutes for large files
#define WIFI_TIMEOUT     20000

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
size_t totalBytesReceived = 0;
size_t bytesWritten = 0;

// Write buffer
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
// Chunked Download Parser State
// ============================================
enum ChunkParseState {
    CHUNK_WAIT_PLUS,        // Waiting for '+' or 'O' (OK) or 'E' (ERROR)
    CHUNK_PARSE_HEADER,     // Parsing "+HTTPCGET:<size>,"
    CHUNK_READ_DATA,        // Reading <size> bytes of binary
};

ChunkParseState chunkState = CHUNK_WAIT_PLUS;
String headerBuffer = "";
size_t chunkExpectedSize = 0;
size_t chunkBytesRead = 0;
bool downloadComplete = false;
bool downloadError = false;

// ============================================
// Utility Functions
// ============================================

void clearC5Buffer() {
    while (C5Serial.available()) {
        C5Serial.read();
    }
}

void printProgress() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {  // Print every 500ms
        lastPrint = millis();
        if (totalFirmwareSize > 0) {
            float percent = (float)totalBytesReceived / totalFirmwareSize * 100.0;
            Serial.printf("\r[PROGRESS] %u / %u bytes (%.1f%%)     ", 
                          totalBytesReceived, totalFirmwareSize, percent);
        } else {
            Serial.printf("\r[PROGRESS] %u bytes received     ", totalBytesReceived);
        }
    }
}

void printPartitionInfo() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    
    Serial.println("\n┌─────────────── Partition Info ───────────────┐");
    Serial.printf("│ Running:  %-36s │\n", running ? running->label : "Unknown");
    Serial.printf("│ Next OTA: %-36s │\n", next ? next->label : "None");
    Serial.println("└───────────────────────────────────────────────┘");
}

// ============================================
// AT Command Functions
// ============================================

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
    
    Serial.printf("[AT RX] TIMEOUT\n");
    return false;
}

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
    
    esp_err_t err = esp_ota_begin(updatePartition, totalFirmwareSize, &otaHandle);
    if (err != ESP_OK) {
        lastError = String("esp_ota_begin failed: ") + esp_err_to_name(err);
        return false;
    }
    
    totalBytesReceived = 0;
    bytesWritten = 0;
    writeBufferPos = 0;
    
    Serial.println("[OTA] OTA begin successful");
    return true;
}

bool writeToOTA(uint8_t* data, size_t len) {
    if (len == 0) return true;
    
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
    
    Serial.printf("\n[OTA] Success! Written %u bytes to flash\n", bytesWritten);
    return true;
}

void abortOTA() {
    if (otaHandle) {
        esp_ota_abort(otaHandle);
        otaHandle = 0;
    }
}

// ============================================
// Chunked Download Parser
// ============================================

void resetChunkParser() {
    chunkState = CHUNK_WAIT_PLUS;
    headerBuffer = "";
    chunkExpectedSize = 0;
    chunkBytesRead = 0;
    downloadComplete = false;
    downloadError = false;
    totalBytesReceived = 0;
    writeBufferPos = 0;
}

// Process one byte from the UART stream
// Returns true when we need to write buffer to flash
bool processChunkByte(uint8_t c) {
    switch (chunkState) {
        case CHUNK_WAIT_PLUS:
            // Looking for '+' (start of +HTTPCGET), 'O' (OK), or 'E' (ERROR)
            if (c == '+') {
                headerBuffer = "+";
                chunkState = CHUNK_PARSE_HEADER;
            } else if (c == 'O') {
                headerBuffer = "O";
                // Could be "OK" - end of transfer
            } else if (c == 'E') {
                headerBuffer = "E";
                // Could be "ERROR"
            } else if (headerBuffer.length() > 0) {
                headerBuffer += (char)c;
                
                if (headerBuffer == "OK") {
                    Serial.printf("\n[HTTP] Transfer complete - received OK\n");
                    downloadComplete = true;
                    return false;
                } else if (headerBuffer.startsWith("ERROR") || headerBuffer.indexOf("ERROR") >= 0) {
                    Serial.printf("\n[HTTP] Received ERROR\n");
                    downloadError = true;
                    lastError = "HTTP transfer error";
                    return false;
                } else if (headerBuffer.length() > 10) {
                    // Not what we're looking for, reset
                    headerBuffer = "";
                }
            }
            // Ignore \r, \n, and other characters between chunks
            break;
            
        case CHUNK_PARSE_HEADER:
            headerBuffer += (char)c;
            
            // Look for complete header: +HTTPCGET:<size>,
            if (headerBuffer.startsWith("+HTTPCGET:") && c == ',') {
                // Parse size
                int colonIdx = headerBuffer.indexOf(':');
                int commaIdx = headerBuffer.indexOf(',');
                if (colonIdx > 0 && commaIdx > colonIdx) {
                    String sizeStr = headerBuffer.substring(colonIdx + 1, commaIdx);
                    chunkExpectedSize = sizeStr.toInt();
                    chunkBytesRead = 0;
                    
                    if (chunkExpectedSize > 0) {
                        chunkState = CHUNK_READ_DATA;
                    } else {
                        // Invalid size, reset
                        headerBuffer = "";
                        chunkState = CHUNK_WAIT_PLUS;
                    }
                }
            }
            
            // Safety: don't let header grow too large
            if (headerBuffer.length() > 30) {
                headerBuffer = "";
                chunkState = CHUNK_WAIT_PLUS;
            }
            break;
            
        case CHUNK_READ_DATA:
            // Add byte to write buffer
            writeBuffer[writeBufferPos++] = c;
            chunkBytesRead++;
            totalBytesReceived++;
            
            // Check if we've read the entire chunk
            if (chunkBytesRead >= chunkExpectedSize) {
                // Chunk complete, go back to waiting for next chunk
                headerBuffer = "";
                chunkState = CHUNK_WAIT_PLUS;
            }
            
            // Return true if buffer is full and needs flushing
            if (writeBufferPos >= OTA_WRITE_SIZE) {
                return true;
            }
            break;
    }
    
    return false;
}

// Process all available data from C5
// Returns: 0 = continue, 1 = complete, -1 = error
int processDownloadStream() {
    while (C5Serial.available()) {
        uint8_t c = C5Serial.read();
        
        bool needFlush = processChunkByte(c);
        
        if (needFlush) {
            // Write buffer to OTA partition
            if (!writeToOTA(writeBuffer, writeBufferPos)) {
                return -1;  // Error
            }
            writeBufferPos = 0;
            printProgress();
        }
        
        if (downloadComplete) {
            return 1;  // Success
        }
        
        if (downloadError) {
            return -1;  // Error
        }
    }
    
    return 0;  // Continue
}

// ============================================
// Main State Machine
// ============================================

void runStateMachine() {
    switch (currentState) {
        case STATE_INIT:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║  ESP32-S3 Streaming OTA via C5 WiFi        ║");
            Serial.println("║  (Chunked Transfer Support)                ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            printPartitionInfo();
            
            esp_ota_mark_app_valid_cancel_rollback();
            
            pinMode(C5_EN_PIN, OUTPUT);
            digitalWrite(C5_EN_PIN, HIGH);
            Serial.println("[INIT] C5 module powered ON");
            
            stateStartTime = millis();
            currentState = STATE_WAIT_C5_BOOT;
            break;
            
        case STATE_WAIT_C5_BOOT:
            if (millis() - stateStartTime >= 3000) {
                // Use larger RX buffer to prevent data loss during fast transfers
                // Default is only 256 bytes - way too small!
                C5Serial.setRxBufferSize(16384);  // 16KB RX buffer
                C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
                Serial.printf("[INIT] UART2: TX=%d, RX=%d, Baud=%d, RX Buffer=16KB\n", 
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
            Serial.println("[STATE] Syncing time...");
            sendATCommand("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"", "OK", 3000);
            delay(2000);
            currentState = STATE_GET_FILE_SIZE;
            break;
            
        case STATE_GET_FILE_SIZE:
            Serial.println("[STATE] Getting firmware file size...");
            {
                char cmd[256];
                snprintf(cmd, sizeof(cmd), "AT+HTTPGETSIZE=\"%s\"", FIRMWARE_URL);
                String response = sendATCommandGetResponse(cmd, HTTP_TIMEOUT);
                
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
            Serial.println("[INFO] Data arrives in chunks - this is normal");
            resetChunkParser();
            
            {
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
            {
                int result = processDownloadStream();
                
                if (result == 1) {
                    // Download complete
                    currentState = STATE_FLUSH_BUFFER;
                } else if (result == -1) {
                    // Error
                    abortOTA();
                    currentState = STATE_ERROR;
                }
                
                // Timeout check (with generous margin for large files)
                unsigned long elapsed = millis() - stateStartTime;
                unsigned long dynamicTimeout = HTTP_TIMEOUT + (totalFirmwareSize / 5);  // ~5KB/s minimum
                
                if (elapsed > dynamicTimeout) {
                    Serial.printf("\n[ERROR] Download timeout after %lu ms\n", elapsed);
                    lastError = "Download timeout";
                    abortOTA();
                    currentState = STATE_ERROR;
                }
                
                // Print progress periodically
                printProgress();
            }
            break;
            
        case STATE_FLUSH_BUFFER:
            Serial.println("\n[STATE] Flushing remaining data...");
            
            if (writeBufferPos > 0) {
                if (!writeToOTA(writeBuffer, writeBufferPos)) {
                    abortOTA();
                    currentState = STATE_ERROR;
                    break;
                }
                Serial.printf("[OTA] Flushed final %u bytes\n", writeBufferPos);
                writeBufferPos = 0;
            }
            
            // Verify we got all the data
            Serial.printf("[INFO] Total received: %u / %u bytes\n", totalBytesReceived, totalFirmwareSize);
            
            if (totalFirmwareSize > 0 && totalBytesReceived < totalFirmwareSize) {
                size_t missing = totalFirmwareSize - totalBytesReceived;
                Serial.printf("[ERROR] Missing %u bytes (%.1f%%)!\n", 
                              missing, (float)missing / totalFirmwareSize * 100.0);
                Serial.println("[ERROR] Data was lost during transfer - UART buffer overflow?");
                Serial.println("[HINT] Try: slower transfer, larger buffer, or chunked download with Range headers");
                lastError = "Incomplete transfer - data lost";
                abortOTA();
                currentState = STATE_ERROR;
                break;
            }
            
            currentState = STATE_OTA_FINALIZE;
            break;
            
        case STATE_OTA_FINALIZE:
            Serial.println("[STATE] Finalizing OTA update...");
            
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
            Serial.printf("║  Downloaded: %-28u bytes ║\n", totalBytesReceived);
            Serial.printf("║  Written:    %-28u bytes ║\n", bytesWritten);
            Serial.println("║                                            ║");
            Serial.println("║  Rebooting in 3 seconds...                 ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            delay(3000);
            esp_restart();
            break;
            
        case STATE_IDLE:
            break;
            
        case STATE_ERROR:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║           OTA UPDATE FAILED                ║");
            Serial.println("╠════════════════════════════════════════════╣");
            Serial.printf("║  Error: %-34s ║\n", lastError.c_str());
            Serial.printf("║  Received: %-31u bytes ║\n", totalBytesReceived);
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
    
    // Only delay when not actively downloading
    // During download, we need to process data as fast as possible
    if (currentState != STATE_DOWNLOADING) {
        delay(1);
    }
}