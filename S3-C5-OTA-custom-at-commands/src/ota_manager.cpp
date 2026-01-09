/*
 * FOV OTA Manager - Phase 3 Implementation
 * 
 * ESP32-S3 side implementation for OTA updates via ESP32-C5 WiFi modem.
 */

#include "ota_manager.h"
#include <esp_log.h>

static const char* TAG = "OTA_MGR";

// ============ Constructor ============

OTAManager::OTAManager(HardwareSerial& serial) 
    : _serial(serial)
    , _progressCallback(nullptr)
    , _lastError(OTA_OK)
{
    memset(_responseBuffer, 0, sizeof(_responseBuffer));
}

// ============ Public Methods ============

void OTAManager::setProgressCallback(ota_progress_callback_t callback) {
    _progressCallback = callback;
}

const char* OTAManager::errorToString(ota_error_t error) {
    switch (error) {
        case OTA_OK:                     return "Success";
        case OTA_ERR_WIFI_NOT_CONNECTED: return "WiFi not connected";
        case OTA_ERR_DOWNLOAD_FAILED:    return "Download failed";
        case OTA_ERR_DOWNLOAD_TIMEOUT:   return "Download timeout";
        case OTA_ERR_NO_FIRMWARE:        return "No firmware in buffer";
        case OTA_ERR_PARTITION_NOT_FOUND: return "OTA partition not found";
        case OTA_ERR_PARTITION_TOO_SMALL: return "OTA partition too small";
        case OTA_ERR_OTA_BEGIN_FAILED:   return "OTA begin failed";
        case OTA_ERR_CHUNK_READ_FAILED:  return "Chunk read failed";
        case OTA_ERR_CHUNK_WRITE_FAILED: return "Chunk write failed";
        case OTA_ERR_CRC_MISMATCH:       return "CRC mismatch";
        case OTA_ERR_OTA_END_FAILED:     return "OTA end failed";
        case OTA_ERR_SET_BOOT_FAILED:    return "Set boot partition failed";
        case OTA_ERR_AT_COMMAND_FAILED:  return "AT command failed";
        case OTA_ERR_INVALID_RESPONSE:   return "Invalid response";
        case OTA_ERR_TIMEOUT:            return "Timeout";
        default:                         return "Unknown error";
    }
}

bool OTAManager::isWiFiConnected() {
    clearSerialBuffer();
    
    if (!sendATCommand("AT+CWJAP?")) {
        return false;
    }
    
    // Check if response contains a valid AP (not "No AP")
    return (strstr(_responseBuffer, "+CWJAP:") != nullptr) && 
           (strstr(_responseBuffer, "No AP") == nullptr);
}

bool OTAManager::connectWiFi(const char* ssid, const char* password, uint32_t timeout_ms) {
    clearSerialBuffer();
    
    // Set station mode
    if (!sendATCommand("AT+CWMODE=1")) {
        ESP_LOGW(TAG, "Failed to set WiFi mode");
    }
    
    // Check if already connected to this network
    if (sendATCommand("AT+CWJAP?")) {
        if (strstr(_responseBuffer, ssid) != nullptr) {
            ESP_LOGI(TAG, "Already connected to %s", ssid);
            return true;
        }
    }
    
    // Build connect command
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    
    // Send command (don't wait for normal response)
    _serial.print(cmd);
    _serial.print("\r\n");
    
    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
    reportProgress(0, 100, "Connecting to WiFi...");
    
    // Wait for "WIFI GOT IP" or error
    uint32_t start = millis();
    String response = "";
    
    while (millis() - start < timeout_ms) {
        while (_serial.available()) {
            char c = _serial.read();
            response += c;
            
            if (response.indexOf("WIFI GOT IP") >= 0) {
                // Wait a bit more for OK
                delay(500);
                while (_serial.available()) {
                    _serial.read();
                }
                ESP_LOGI(TAG, "WiFi connected");
                return true;
            }
            
            if (response.indexOf("FAIL") >= 0 || 
                response.indexOf("+CWJAP:") >= 0) {  // Error code
                ESP_LOGE(TAG, "WiFi connection failed");
                return false;
            }
        }
        delay(100);
    }
    
    ESP_LOGE(TAG, "WiFi connection timeout");
    return false;
}

bool OTAManager::getFirmwareInfo(uint32_t* size, uint32_t* crc) {
    clearSerialBuffer();
    
    if (!sendATCommand("AT+FWBUFSTATUS?")) {
        return false;
    }
    
    // Parse response: +FWBUFSTATUS:1,READY,<size>,0x<crc>
    char* statusLine = strstr(_responseBuffer, "+FWBUFSTATUS:");
    if (!statusLine) {
        return false;
    }
    
    // Check state is READY
    if (strstr(statusLine, "READY") == nullptr) {
        ESP_LOGW(TAG, "Buffer not ready");
        return false;
    }
    
    // Parse size and CRC
    // Format: +FWBUFSTATUS:1,READY,275072,0x20269654
    int version;
    char state[16];
    unsigned long parsedSize, parsedCrc;
    
    if (sscanf(statusLine, "+FWBUFSTATUS:%d,%[^,],%lu,0x%lx", 
               &version, state, &parsedSize, &parsedCrc) >= 4) {
        *size = parsedSize;
        *crc = parsedCrc;
        return true;
    }
    
    // Try without 0x prefix
    if (sscanf(statusLine, "+FWBUFSTATUS:%d,%[^,],%lu,%lu", 
               &version, state, &parsedSize, &parsedCrc) >= 4) {
        *size = parsedSize;
        *crc = parsedCrc;
        return true;
    }
    
    return false;
}

bool OTAManager::clearBuffer() {
    return sendATCommand("AT+FWBUFCLEAR");
}

ota_error_t OTAManager::performOTA(const char* url) {
    ESP_LOGI(TAG, "Starting OTA from: %s", url);
    _lastError = OTA_OK;
    
    // ========== Step 1: Check WiFi ==========
    reportProgress(0, 100, "Checking WiFi...");
    
    if (!isWiFiConnected()) {
        ESP_LOGE(TAG, "WiFi not connected");
        _lastError = OTA_ERR_WIFI_NOT_CONNECTED;
        return _lastError;
    }
    
    // ========== Step 2: Clear any existing buffer ==========
    reportProgress(0, 100, "Preparing...");
    clearBuffer();
    
    // ========== Step 3: Download firmware to C5 ==========
    reportProgress(0, 100, "Downloading firmware...");
    
    char cmd[300];
    snprintf(cmd, sizeof(cmd), "AT+FWBUFDOWNLOAD=\"%s\"", url);
    
    // Send download command
    clearSerialBuffer();
    _serial.print(cmd);
    _serial.print("\r\n");
    
    ESP_LOGI(TAG, "Sent download command");
    
    // Wait for STARTED
    if (!waitForResponse("+FWBUFDOWNLOAD:STARTED", 10000)) {
        ESP_LOGE(TAG, "No STARTED response");
        _lastError = OTA_ERR_DOWNLOAD_FAILED;
        return _lastError;
    }
    
    // Wait for DONE or ERROR
    uint32_t start = millis();
    String response = "";
    uint32_t firmwareSize = 0;
    uint32_t firmwareCrc = 0;
    
    while (millis() - start < OTA_DOWNLOAD_TIMEOUT_MS) {
        while (_serial.available()) {
            char c = _serial.read();
            response += c;
            
            // Check for completion
            int doneIdx = response.indexOf("+FWBUFDOWNLOAD:DONE,");
            if (doneIdx >= 0) {
                // Parse size and CRC: +FWBUFDOWNLOAD:DONE,275072,0x20269654
                String doneLine = response.substring(doneIdx);
                int comma1 = doneLine.indexOf(',');
                int comma2 = doneLine.indexOf(',', comma1 + 1);
                
                if (comma1 > 0 && comma2 > comma1) {
                    firmwareSize = doneLine.substring(comma1 + 1, comma2).toInt();
                    String crcStr = doneLine.substring(comma2 + 1);
                    crcStr.trim();
                    if (crcStr.startsWith("0x") || crcStr.startsWith("0X")) {
                        firmwareCrc = strtoul(crcStr.c_str() + 2, nullptr, 16);
                    } else {
                        firmwareCrc = strtoul(crcStr.c_str(), nullptr, 10);
                    }
                }
                
                ESP_LOGI(TAG, "Download complete: %lu bytes, CRC: 0x%08lX", 
                         (unsigned long)firmwareSize, (unsigned long)firmwareCrc);
                goto download_done;
            }
            
            // Check for error
            if (response.indexOf("+FWBUFDOWNLOAD:ERROR") >= 0) {
                ESP_LOGE(TAG, "Download error: %s", response.c_str());
                _lastError = OTA_ERR_DOWNLOAD_FAILED;
                return _lastError;
            }
        }
        delay(100);
    }
    
    ESP_LOGE(TAG, "Download timeout");
    _lastError = OTA_ERR_DOWNLOAD_TIMEOUT;
    return _lastError;

download_done:
    if (firmwareSize == 0) {
        // Try to get info from status
        if (!getFirmwareInfo(&firmwareSize, &firmwareCrc)) {
            ESP_LOGE(TAG, "Could not get firmware info");
            _lastError = OTA_ERR_NO_FIRMWARE;
            return _lastError;
        }
    }
    
    // ========== Step 4: Find OTA partition ==========
    reportProgress(0, firmwareSize, "Preparing OTA partition...");
    
    const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (!updatePartition) {
        ESP_LOGE(TAG, "No OTA partition found");
        _lastError = OTA_ERR_PARTITION_NOT_FOUND;
        return _lastError;
    }
    
    ESP_LOGI(TAG, "Writing to partition: %s (size: %lu)", 
             updatePartition->label, (unsigned long)updatePartition->size);
    
    if (updatePartition->size < firmwareSize) {
        ESP_LOGE(TAG, "Partition too small: %lu < %lu", 
                 (unsigned long)updatePartition->size, (unsigned long)firmwareSize);
        _lastError = OTA_ERR_PARTITION_TOO_SMALL;
        return _lastError;
    }
    
    // ========== Step 5: Begin OTA ==========
    esp_ota_handle_t otaHandle;
    esp_err_t err = esp_ota_begin(updatePartition, firmwareSize, &otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        _lastError = OTA_ERR_OTA_BEGIN_FAILED;
        return _lastError;
    }
    
    // ========== Step 6: Read chunks and write to flash ==========
    uint8_t* chunkBuffer = (uint8_t*)malloc(OTA_CHUNK_SIZE);
    if (!chunkBuffer) {
        ESP_LOGE(TAG, "Failed to allocate chunk buffer");
        esp_ota_abort(otaHandle);
        _lastError = OTA_ERR_CHUNK_READ_FAILED;
        return _lastError;
    }
    
    uint32_t offset = 0;
    uint32_t retries = 0;
    
    while (offset < firmwareSize) {
        uint32_t remaining = firmwareSize - offset;
        uint32_t chunkSize = (remaining < OTA_CHUNK_SIZE) ? remaining : OTA_CHUNK_SIZE;
        
        reportProgress(offset, firmwareSize, "Writing firmware...");
        
        // Read chunk from C5
        int32_t bytesRead = readChunk(offset, chunkBuffer, chunkSize);
        
        if (bytesRead < 0) {
            ESP_LOGW(TAG, "Chunk read failed at offset %lu, retry %lu", 
                     (unsigned long)offset, (unsigned long)retries);
            retries++;
            if (retries >= OTA_MAX_RETRIES) {
                ESP_LOGE(TAG, "Max retries exceeded");
                free(chunkBuffer);
                esp_ota_abort(otaHandle);
                _lastError = OTA_ERR_CHUNK_READ_FAILED;
                return _lastError;
            }
            delay(100);
            continue;
        }
        
        if (bytesRead != (int32_t)chunkSize) {
            ESP_LOGW(TAG, "Short read: expected %lu, got %ld", 
                     (unsigned long)chunkSize, (long)bytesRead);
        }
        
        // Write to flash
        err = esp_ota_write(otaHandle, chunkBuffer, bytesRead);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(chunkBuffer);
            esp_ota_abort(otaHandle);
            _lastError = OTA_ERR_CHUNK_WRITE_FAILED;
            return _lastError;
        }
        
        offset += bytesRead;
        retries = 0;  // Reset retry counter on success
        
        // Progress log every 10%
        if ((offset * 10 / firmwareSize) != ((offset - bytesRead) * 10 / firmwareSize)) {
            ESP_LOGI(TAG, "Progress: %lu / %lu bytes (%.0f%%)", 
                     (unsigned long)offset, (unsigned long)firmwareSize,
                     (float)offset * 100.0f / firmwareSize);
        }
    }
    
    free(chunkBuffer);
    
    // ========== Step 7: Verify CRC ==========
    reportProgress(firmwareSize, firmwareSize, "Verifying...");
    
    // Verify with C5's stored CRC
    char verifyCmd[32];
    snprintf(verifyCmd, sizeof(verifyCmd), "AT+FWBUFVERIFY=0x%08lX", (unsigned long)firmwareCrc);
    
    if (!sendATCommand(verifyCmd)) {
        ESP_LOGW(TAG, "CRC verification command failed");
        // Continue anyway - we'll check OTA end
    } else if (strstr(_responseBuffer, "MISMATCH") != nullptr) {
        ESP_LOGE(TAG, "CRC mismatch!");
        esp_ota_abort(otaHandle);
        _lastError = OTA_ERR_CRC_MISMATCH;
        return _lastError;
    }
    
    // ========== Step 8: Finalize OTA ==========
    reportProgress(firmwareSize, firmwareSize, "Finalizing...");
    
    err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        _lastError = OTA_ERR_OTA_END_FAILED;
        return _lastError;
    }
    
    // ========== Step 9: Set boot partition ==========
    err = esp_ota_set_boot_partition(updatePartition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        _lastError = OTA_ERR_SET_BOOT_FAILED;
        return _lastError;
    }
    
    // ========== Step 10: Clear C5 buffer ==========
    clearBuffer();
    
    ESP_LOGI(TAG, "OTA complete! Restart to boot new firmware.");
    reportProgress(firmwareSize, firmwareSize, "OTA complete!");
    
    _lastError = OTA_OK;
    return OTA_OK;
}

// ============ Private Methods ============

void OTAManager::clearSerialBuffer() {
    while (_serial.available()) {
        _serial.read();
    }
}

bool OTAManager::sendATCommand(const char* cmd, uint32_t timeout_ms) {
    clearSerialBuffer();
    
    _serial.print(cmd);
    _serial.print("\r\n");
    
    return readResponse(timeout_ms);
}

bool OTAManager::readResponse(uint32_t timeout_ms) {
    memset(_responseBuffer, 0, sizeof(_responseBuffer));
    
    uint32_t start = millis();
    size_t idx = 0;
    
    while (millis() - start < timeout_ms) {
        while (_serial.available() && idx < sizeof(_responseBuffer) - 1) {
            char c = _serial.read();
            _responseBuffer[idx++] = c;
            
            // Check for response terminators
            if (idx >= 4) {
                if (strstr(_responseBuffer, "OK\r\n") != nullptr ||
                    strstr(_responseBuffer, "ERROR\r\n") != nullptr) {
                    return strstr(_responseBuffer, "OK") != nullptr;
                }
            }
        }
        delay(10);
    }
    
    return false;  // Timeout
}

bool OTAManager::waitForResponse(const char* expected, uint32_t timeout_ms) {
    uint32_t start = millis();
    String response = "";
    
    while (millis() - start < timeout_ms) {
        while (_serial.available()) {
            char c = _serial.read();
            response += c;
            
            if (response.indexOf(expected) >= 0) {
                return true;
            }
        }
        delay(10);
    }
    
    return false;
}

int32_t OTAManager::readChunk(uint32_t offset, uint8_t* buffer, uint32_t length) {
    clearSerialBuffer();
    
    // Send read command
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+FWBUFREAD=%lu,%lu", 
             (unsigned long)offset, (unsigned long)length);
    
    _serial.print(cmd);
    _serial.print("\r\n");
    
    // Wait for header: +FWBUFREAD:<length>,
    uint32_t start = millis();
    String header = "";
    int32_t actualLength = -1;
    
    while (millis() - start < OTA_UART_TIMEOUT_MS) {
        while (_serial.available()) {
            char c = _serial.read();
            header += c;
            
            // Check for error
            if (header.indexOf("ERROR") >= 0) {
                ESP_LOGE(TAG, "Read error at offset %lu", (unsigned long)offset);
                return -1;
            }
            
            // Look for header format: +FWBUFREAD:<len>,
            int colonIdx = header.indexOf("+FWBUFREAD:");
            if (colonIdx >= 0) {
                int commaIdx = header.indexOf(',', colonIdx);
                if (commaIdx > colonIdx) {
                    // Parse length
                    String lenStr = header.substring(colonIdx + 11, commaIdx);
                    actualLength = lenStr.toInt();
                    goto read_binary;
                }
            }
        }
        delay(1);
    }
    
    ESP_LOGE(TAG, "Timeout waiting for header");
    return -1;

read_binary:
    if (actualLength <= 0 || actualLength > (int32_t)length) {
        ESP_LOGE(TAG, "Invalid length in response: %ld", (long)actualLength);
        return -1;
    }
    
    // Read binary data
    uint32_t bytesRead = 0;
    start = millis();
    
    while (bytesRead < (uint32_t)actualLength && millis() - start < OTA_UART_TIMEOUT_MS) {
        while (_serial.available() && bytesRead < (uint32_t)actualLength) {
            buffer[bytesRead++] = _serial.read();
        }
        if (bytesRead < (uint32_t)actualLength) {
            delay(1);
        }
    }
    
    if (bytesRead != (uint32_t)actualLength) {
        ESP_LOGW(TAG, "Short read: expected %ld, got %lu", 
                 (long)actualLength, (unsigned long)bytesRead);
    }
    
    // Consume trailer (\r\n and OK\r\n)
    delay(50);
    while (_serial.available()) {
        _serial.read();
    }
    
    return bytesRead;
}

void OTAManager::reportProgress(uint32_t downloaded, uint32_t total, const char* status) {
    if (_progressCallback) {
        _progressCallback(downloaded, total, status);
    }
}
