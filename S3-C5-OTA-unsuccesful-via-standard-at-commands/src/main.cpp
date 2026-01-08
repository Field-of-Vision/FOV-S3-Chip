/*
 * ESP32-S3 Streaming OTA Update via C5 WiFi Module
 * 
 * WITH IMPROVED FLOW CONTROL DEBUGGING
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
#include <driver/uart.h>

// ============================================
// Pin Definitions
// ============================================
#define C5_TX_PIN   35
#define C5_RX_PIN   36
#define C5_RTS_PIN  21    // S3 RTS -> C5 CTS (S3 tells C5 to pause)
#define C5_CTS_PIN  37    // S3 CTS <- C5 RTS (C5 tells S3 to pause)
#define C5_EN_PIN   1

// ============================================
// Configuration
// ============================================
#define C5_BAUD_RATE     115200
#define OTA_WRITE_SIZE   4096
#define AT_TIMEOUT       10000
#define HTTP_TIMEOUT     120000
#define WIFI_TIMEOUT     20000

// Set to true to enable flow control attempts
#define ENABLE_FLOW_CONTROL  true

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
bool flowControlEnabled = false;

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
    STATE_SETUP_FLOW_CONTROL,  // New state for FC setup
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
    CHUNK_WAIT_PLUS,
    CHUNK_PARSE_HEADER,
    CHUNK_READ_DATA,
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
    if (millis() - lastPrint > 500) {
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

void printFlowControlStatus() {
    Serial.println("\n┌─────────────── Flow Control Status ──────────┐");
    Serial.printf("│ S3 RTS Pin (GPIO%d): %s                    │\n", 
                  C5_RTS_PIN, 
                  digitalRead(C5_RTS_PIN) ? "HIGH (C5 can send)" : "LOW (C5 paused)");
    Serial.printf("│ S3 CTS Pin (GPIO%d): %s                    │\n",
                  C5_CTS_PIN,
                  digitalRead(C5_CTS_PIN) ? "HIGH" : "LOW");
    Serial.printf("│ Flow Control Active: %-24s │\n", flowControlEnabled ? "YES" : "NO");
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
    
    Serial.printf("[AT RX] TIMEOUT waiting for '%s'\n", expected);
    Serial.printf("[AT RX] Got: %s\n", response.c_str());
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
// Flow Control Setup
// ============================================

bool setupFlowControl() {
    Serial.println("\n╔═══════════════════════════════════════════════╗");
    Serial.println("║       FLOW CONTROL SETUP                      ║");
    Serial.println("╚═══════════════════════════════════════════════╝");
    
    // Step 1: Query current C5 UART settings
    Serial.println("\n[FC] Step 1: Query C5 current UART settings...");
    String uartInfo = sendATCommandGetResponse("AT+UART_CUR?", 2000);
    Serial.printf("[FC] C5 reports: %s\n", uartInfo.c_str());
    
    // Step 2: Check RTS pin state
    // When using hardware flow control on S3, the RTS line should be:
    // - HIGH when buffer has space (C5 can send)
    // - LOW when buffer is full (C5 should pause)
    Serial.println("\n[FC] Step 2: Check S3 RTS pin state...");
    pinMode(C5_RTS_PIN, INPUT);  // Temporarily read it
    int rtsState = digitalRead(C5_RTS_PIN);
    Serial.printf("[FC] S3 RTS pin (GPIO%d) = %d\n", C5_RTS_PIN, rtsState);
    
    // Step 3: Manually ensure RTS is HIGH before enabling flow control on C5
    // If C5 starts in CTS mode and sees CTS LOW, it won't send anything!
    Serial.println("\n[FC] Step 3: Force RTS HIGH before enabling C5 flow control...");
    pinMode(C5_RTS_PIN, OUTPUT);
    digitalWrite(C5_RTS_PIN, HIGH);  // Clear to send
    delay(50);
    Serial.printf("[FC] S3 RTS pin forced HIGH\n");
    
    // Step 4: Now reconfigure S3 UART with proper flow control
    Serial.println("\n[FC] Step 4: Reconfigure S3 UART with RTS flow control...");
    
    // We need to re-setup UART to properly claim the RTS pin
    C5Serial.end();
    delay(100);
    
    C5Serial.setRxBufferSize(16384);
    C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
    
    // Set pins including RTS
    C5Serial.setPins(C5_RX_PIN, C5_TX_PIN, C5_CTS_PIN, C5_RTS_PIN);
    
    // Enable RTS mode - S3 controls its RTS line based on RX buffer fill
    // Threshold 64 means: pull RTS LOW when buffer has < 64 bytes free
    // FIFO is 128 bytes, so this gives us headroom
    C5Serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_RTS, 64);
    
    delay(100);
    clearC5Buffer();
    
    Serial.printf("[FC] S3 UART reconfigured with RTS flow control (threshold=64)\n");
    
    // Step 5: Verify S3 can still talk to C5
    Serial.println("\n[FC] Step 5: Verify basic communication...");
    if (!sendATCommand("AT", "OK", 2000)) {
        Serial.println("[FC] ERROR: Lost communication after S3 UART reconfig!");
        return false;
    }
    Serial.println("[FC] Basic communication OK");
    
    // Step 6: Now enable CTS mode on C5
    // This tells C5 to watch its CTS pin (which is connected to S3's RTS)
    Serial.println("\n[FC] Step 6: Enable CTS mode on C5...");
    Serial.println("[FC] Command: AT+UART_CUR=115200,8,1,0,2");
    Serial.println("[FC] (baud=115200, 8N1, flow_control=2=CTS)");
    
    if (!sendATCommand("AT+UART_CUR=115200,8,1,0,2", "OK", 3000)) {
        Serial.println("[FC] WARNING: Failed to enable C5 CTS mode");
        Serial.println("[FC] Proceeding without flow control...");
        return false;
    }
    
    Serial.println("[FC] C5 CTS mode enabled");
    delay(100);
    
    // Step 7: Verify communication still works
    Serial.println("\n[FC] Step 7: Verify communication after C5 flow control...");
    if (!sendATCommand("AT", "OK", 3000)) {
        Serial.println("[FC] ERROR: Communication failed after enabling C5 flow control!");
        Serial.println("[FC] This likely means RTS/CTS handshake is stuck");
        
        // Try to recover by disabling C5 flow control
        Serial.println("[FC] Attempting recovery...");
        
        // Force RTS high in case it's stuck low
        pinMode(C5_RTS_PIN, OUTPUT);
        digitalWrite(C5_RTS_PIN, HIGH);
        delay(100);
        
        // Try to disable C5 flow control
        C5Serial.print("AT+UART_CUR=115200,8,1,0,0\r\n");
        delay(500);
        clearC5Buffer();
        
        if (sendATCommand("AT", "OK", 2000)) {
            Serial.println("[FC] Recovery successful - proceeding without flow control");
        } else {
            Serial.println("[FC] Recovery failed - C5 may need power cycle");
        }
        return false;
    }
    
    Serial.println("[FC] Communication verified!");
    
    // Step 8: Query C5 to confirm settings
    Serial.println("\n[FC] Step 8: Confirm C5 UART settings...");
    uartInfo = sendATCommandGetResponse("AT+UART_CUR?", 2000);
    Serial.printf("[FC] C5 UART now: %s\n", uartInfo.c_str());
    
    Serial.println("\n╔═══════════════════════════════════════════════╗");
    Serial.println("║       FLOW CONTROL SETUP COMPLETE             ║");
    Serial.println("╚═══════════════════════════════════════════════╝\n");
    
    return true;
}

// Disable flow control on C5 (for recovery)
void disableC5FlowControl() {
    Serial.println("[FC] Disabling C5 flow control...");
    
    // Force RTS high first
    pinMode(C5_RTS_PIN, OUTPUT);
    digitalWrite(C5_RTS_PIN, HIGH);
    delay(50);
    
    // Send command to disable flow control
    C5Serial.print("AT+UART_CUR=115200,8,1,0,0\r\n");
    delay(500);
    clearC5Buffer();
    
    // Verify
    if (sendATCommand("AT", "OK", 2000)) {
        Serial.println("[FC] Flow control disabled, communication restored");
    }
    flowControlEnabled = false;
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

bool processChunkByte(uint8_t c) {
    switch (chunkState) {
        case CHUNK_WAIT_PLUS:
            if (c == '+') {
                headerBuffer = "+";
                chunkState = CHUNK_PARSE_HEADER;
            } else if (c == 'O') {
                headerBuffer = "O";
            } else if (c == 'E') {
                headerBuffer = "E";
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
                    headerBuffer = "";
                }
            }
            break;
            
        case CHUNK_PARSE_HEADER:
            headerBuffer += (char)c;
            
            if (headerBuffer.startsWith("+HTTPCGET:") && c == ',') {
                int colonIdx = headerBuffer.indexOf(':');
                int commaIdx = headerBuffer.indexOf(',');
                if (colonIdx > 0 && commaIdx > colonIdx) {
                    String sizeStr = headerBuffer.substring(colonIdx + 1, commaIdx);
                    chunkExpectedSize = sizeStr.toInt();
                    chunkBytesRead = 0;
                    
                    if (chunkExpectedSize > 0) {
                        chunkState = CHUNK_READ_DATA;
                    } else {
                        headerBuffer = "";
                        chunkState = CHUNK_WAIT_PLUS;
                    }
                }
            }
            
            if (headerBuffer.length() > 30) {
                headerBuffer = "";
                chunkState = CHUNK_WAIT_PLUS;
            }
            break;
            
        case CHUNK_READ_DATA:
            writeBuffer[writeBufferPos++] = c;
            chunkBytesRead++;
            totalBytesReceived++;
            
            if (chunkBytesRead >= chunkExpectedSize) {
                headerBuffer = "";
                chunkState = CHUNK_WAIT_PLUS;
            }
            
            if (writeBufferPos >= OTA_WRITE_SIZE) {
                return true;
            }
            break;
    }
    
    return false;
}

int processDownloadStream() {
    while (C5Serial.available()) {
        uint8_t c = C5Serial.read();
        
        bool needFlush = processChunkByte(c);
        
        if (needFlush) {
            if (!writeToOTA(writeBuffer, writeBufferPos)) {
                return -1;
            }
            writeBufferPos = 0;
            printProgress();
        }
        
        if (downloadComplete) {
            return 1;
        }
        
        if (downloadError) {
            return -1;
        }
    }
    
    return 0;
}

// ============================================
// Main State Machine
// ============================================

void runStateMachine() {
    switch (currentState) {
        case STATE_INIT:
            Serial.println("\n╔════════════════════════════════════════════╗");
            Serial.println("║  ESP32-S3 Streaming OTA via C5 WiFi        ║");
            Serial.println("║  WITH FLOW CONTROL DEBUGGING               ║");
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
                // Initial UART setup WITHOUT flow control
                C5Serial.setRxBufferSize(16384);
                C5Serial.begin(C5_BAUD_RATE, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
                
                Serial.printf("[INIT] UART2: TX=%d, RX=%d, Baud=%d, RX Buffer=16KB\n", 
                              C5_TX_PIN, C5_RX_PIN, C5_BAUD_RATE);
                Serial.println("[INIT] Flow control will be configured after C5 check");
                
                delay(500);
                clearC5Buffer();
                currentState = STATE_CHECK_C5;
            }
            break;
            
        case STATE_CHECK_C5:
            Serial.println("[STATE] Checking C5 communication...");
            if (sendATCommand("AT", "OK", 2000)) {
                Serial.println("[OK] C5 responding");
                
                // Print firmware version
                String version = sendATCommandGetResponse("AT+GMR", 3000);
                Serial.printf("[INFO] C5 Firmware:\n%s\n", version.c_str());
                
                if (ENABLE_FLOW_CONTROL) {
                    currentState = STATE_SETUP_FLOW_CONTROL;
                } else {
                    Serial.println("[INFO] Flow control disabled by config");
                    currentState = STATE_WIFI_CONNECT;
                }
            } else {
                lastError = "C5 not responding";
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_SETUP_FLOW_CONTROL:
            Serial.println("[STATE] Setting up hardware flow control...");
            
            flowControlEnabled = setupFlowControl();
            
            if (flowControlEnabled) {
                Serial.println("[OK] Flow control active - should prevent data loss");
            } else {
                Serial.println("[WARN] Flow control NOT active - data loss may occur");
            }
            
            printFlowControlStatus();
            currentState = STATE_WIFI_CONNECT;
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
                // Print flow control status before download
                if (flowControlEnabled) {
                    printFlowControlStatus();
                }
                
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
                        
                        // Estimate transfer time
                        float estimatedSeconds = (float)totalFirmwareSize / 11520.0;  // ~11.5 KB/s
                        Serial.printf("[INFO] Estimated transfer time: %.1f seconds at 115200 baud\n", estimatedSeconds);
                        
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
            
            if (flowControlEnabled) {
                Serial.println("[INFO] Flow control ENABLED - C5 will pause when S3 buffer fills");
            } else {
                Serial.println("[WARN] Flow control DISABLED - may lose data if C5 sends too fast");
            }
            
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
                static unsigned long lastDataTime = 0;
                static size_t lastBytesReceived = 0;
                static unsigned long lastFCPrint = 0;
                
                int result = processDownloadStream();
                
                if (result == 1) {
                    currentState = STATE_FLUSH_BUFFER;
                } else if (result == -1) {
                    abortOTA();
                    currentState = STATE_ERROR;
                }
                
                if (totalBytesReceived > lastBytesReceived) {
                    lastDataTime = millis();
                    lastBytesReceived = totalBytesReceived;
                }
                
                // Periodically print flow control status during download
                if (flowControlEnabled && millis() - lastFCPrint > 5000) {
                    lastFCPrint = millis();
                    Serial.printf("\n[FC] RTS=%d during download\n", digitalRead(C5_RTS_PIN));
                }
                
                // Stall detection
                if (lastDataTime > 0 && millis() - lastDataTime > 10000) {
                    Serial.printf("\n[ERROR] Transfer stalled - no data for 10 seconds\n");
                    Serial.printf("[INFO] Got %u / %u bytes (%.1f%%)\n", 
                                  totalBytesReceived, totalFirmwareSize,
                                  (float)totalBytesReceived / totalFirmwareSize * 100.0);
                    
                    if (flowControlEnabled) {
                        Serial.printf("[DEBUG] RTS pin state: %d\n", digitalRead(C5_RTS_PIN));
                        Serial.println("[DEBUG] If RTS is LOW, S3 is telling C5 to pause");
                        Serial.println("[DEBUG] This could mean a deadlock in flow control");
                    }
                    
                    lastError = "Transfer stalled";
                    abortOTA();
                    currentState = STATE_ERROR;
                    
                    lastDataTime = 0;
                    lastBytesReceived = 0;
                }
                
                unsigned long elapsed = millis() - stateStartTime;
                unsigned long dynamicTimeout = HTTP_TIMEOUT + (totalFirmwareSize / 5);
                
                if (elapsed > dynamicTimeout) {
                    Serial.printf("\n[ERROR] Download timeout after %lu ms\n", elapsed);
                    lastError = "Download timeout";
                    abortOTA();
                    currentState = STATE_ERROR;
                    
                    lastDataTime = 0;
                    lastBytesReceived = 0;
                }
                
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
            
            Serial.printf("[INFO] Total received: %u / %u bytes\n", totalBytesReceived, totalFirmwareSize);
            
            if (totalFirmwareSize > 0 && totalBytesReceived < totalFirmwareSize) {
                size_t missing = totalFirmwareSize - totalBytesReceived;
                float percentMissing = (float)missing / totalFirmwareSize * 100.0;
                Serial.printf("[ERROR] Missing %u bytes (%.1f%%)!\n", missing, percentMissing);
                Serial.println("[ERROR] Data was lost during transfer");
                
                if (flowControlEnabled) {
                    Serial.println("[DEBUG] Flow control was enabled but data still lost!");
                    Serial.println("[DEBUG] Possible causes:");
                    Serial.println("  - C5 not respecting CTS signal");
                    Serial.println("  - RTS threshold too high");
                    Serial.println("  - HTTP streaming doesn't integrate with UART FC");
                } else {
                    Serial.println("[HINT] Enable flow control or use buffered download");
                }
                
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
            Serial.printf("║  Flow Ctrl:  %-28s ║\n", flowControlEnabled ? "ENABLED" : "DISABLED");
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
            Serial.printf("║  Flow Ctrl: %-30s ║\n", flowControlEnabled ? "ENABLED" : "DISABLED");
            Serial.println("║                                            ║");
            Serial.println("║  Continuing with current firmware...       ║");
            Serial.println("╚════════════════════════════════════════════╝\n");
            
            // If flow control might be stuck, try to disable it
            if (flowControlEnabled) {
                disableC5FlowControl();
            }
            
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
    
    if (currentState != STATE_DOWNLOADING) {
        delay(1);
    }
}