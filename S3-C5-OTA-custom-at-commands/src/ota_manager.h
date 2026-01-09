/*
 * FOV OTA Manager - Phase 3
 * 
 * ESP32-S3 side implementation for OTA updates via ESP32-C5 WiFi modem.
 * 
 * This module:
 *   1. Sends AT commands to the C5 to download firmware
 *   2. Reads firmware chunks from C5's PSRAM buffer
 *   3. Writes chunks to S3's OTA partition
 *   4. Verifies CRC and triggers reboot
 * 
 * Usage:
 *   #include "ota_manager.h"
 *   
 *   OTAManager ota(Serial1);  // UART connected to C5
 *   
 *   if (ota.performOTA("http://server/firmware.bin")) {
 *       ESP.restart();  // Boot into new firmware
 *   }
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// ============ Configuration ============

#define OTA_CHUNK_SIZE          4096    // Bytes per read (matches C5 max)
#define OTA_UART_TIMEOUT_MS     5000    // Timeout for AT responses
#define OTA_DOWNLOAD_TIMEOUT_MS 120000  // Timeout for firmware download (2 min)
#define OTA_MAX_RETRIES         3       // Retries per chunk on error
#define OTA_RESPONSE_BUFFER_SIZE 256    // Buffer for AT responses

// ============ Error Codes ============

typedef enum {
    OTA_OK = 0,
    OTA_ERR_WIFI_NOT_CONNECTED,
    OTA_ERR_DOWNLOAD_FAILED,
    OTA_ERR_DOWNLOAD_TIMEOUT,
    OTA_ERR_NO_FIRMWARE,
    OTA_ERR_PARTITION_NOT_FOUND,
    OTA_ERR_PARTITION_TOO_SMALL,
    OTA_ERR_OTA_BEGIN_FAILED,
    OTA_ERR_CHUNK_READ_FAILED,
    OTA_ERR_CHUNK_WRITE_FAILED,
    OTA_ERR_CRC_MISMATCH,
    OTA_ERR_OTA_END_FAILED,
    OTA_ERR_SET_BOOT_FAILED,
    OTA_ERR_AT_COMMAND_FAILED,
    OTA_ERR_INVALID_RESPONSE,
    OTA_ERR_TIMEOUT
} ota_error_t;

// ============ Status Callback ============

typedef void (*ota_progress_callback_t)(uint32_t downloaded, uint32_t total, const char* status);

// ============ OTA Manager Class ============

class OTAManager {
public:
    /**
     * Constructor
     * @param serial Reference to HardwareSerial connected to C5 (e.g., Serial1)
     */
    OTAManager(HardwareSerial& serial);
    
    /**
     * Set progress callback for status updates
     * @param callback Function to call with progress updates
     */
    void setProgressCallback(ota_progress_callback_t callback);
    
    /**
     * Check if C5 is connected to WiFi
     * @return true if connected
     */
    bool isWiFiConnected();
    
    /**
     * Connect C5 to WiFi network
     * @param ssid WiFi network name
     * @param password WiFi password
     * @param timeout_ms Connection timeout in milliseconds
     * @return true if connected successfully
     */
    bool connectWiFi(const char* ssid, const char* password, uint32_t timeout_ms = 30000);
    
    /**
     * Perform complete OTA update
     * @param url HTTP URL to firmware binary
     * @return OTA_OK on success, error code on failure
     */
    ota_error_t performOTA(const char* url);
    
    /**
     * Get last error code
     * @return Last error code
     */
    ota_error_t getLastError() { return _lastError; }
    
    /**
     * Get human-readable error string
     * @param error Error code
     * @return Error description string
     */
    static const char* errorToString(ota_error_t error);
    
    /**
     * Get firmware info from C5 buffer (after download)
     * @param size Output: firmware size in bytes
     * @param crc Output: firmware CRC32
     * @return true if info retrieved successfully
     */
    bool getFirmwareInfo(uint32_t* size, uint32_t* crc);
    
    /**
     * Clear the C5's firmware buffer
     * @return true on success
     */
    bool clearBuffer();

private:
    HardwareSerial& _serial;
    ota_progress_callback_t _progressCallback;
    ota_error_t _lastError;
    
    char _responseBuffer[OTA_RESPONSE_BUFFER_SIZE];
    
    // AT command helpers
    bool sendATCommand(const char* cmd, uint32_t timeout_ms = OTA_UART_TIMEOUT_MS);
    bool waitForResponse(const char* expected, uint32_t timeout_ms);
    bool readResponse(uint32_t timeout_ms);
    void clearSerialBuffer();
    
    // Binary chunk reading
    int32_t readChunk(uint32_t offset, uint8_t* buffer, uint32_t length);
    
    // Response parsing
    bool parseDownloadResponse(uint32_t* size, uint32_t* crc);
    bool parseStatusResponse(char* state, uint32_t* param1, uint32_t* param2);
    
    // Progress reporting
    void reportProgress(uint32_t downloaded, uint32_t total, const char* status);
};

#endif // OTA_MANAGER_H
