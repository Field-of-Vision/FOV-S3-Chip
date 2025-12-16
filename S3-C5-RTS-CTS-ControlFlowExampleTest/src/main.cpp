/*
 * RTS/CTS Hardware Connection Test
 * 
 * This test verifies the physical wiring between:
 *   S3 GPIO21 (RTS) -> C5 GPIO2 (CTS)
 *   S3 GPIO37 (CTS) <- C5 GPIO3 (RTS)
 * 
 * Run this BEFORE trying flow control to confirm wires are connected.
 */

#include <Arduino.h>

// Pin definitions - same as OTA code
#define C5_TX_PIN   35
#define C5_RX_PIN   36
#define C5_RTS_PIN  21    // S3 RTS -> C5 CTS (S3 tells C5 to pause)
#define C5_CTS_PIN  37    // S3 CTS <- C5 RTS (C5 tells S3 to pause)
#define C5_EN_PIN   1

HardwareSerial C5Serial(2);

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║     RTS/CTS HARDWARE CONNECTION TEST           ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    // Power on C5
    pinMode(C5_EN_PIN, OUTPUT);
    digitalWrite(C5_EN_PIN, HIGH);
    Serial.println("[INIT] C5 powered on, waiting for boot...");
    delay(3000);
    
    // Basic UART - NO flow control
    C5Serial.begin(115200, SERIAL_8N1, C5_RX_PIN, C5_TX_PIN);
    delay(500);
    
    // Clear buffer
    while (C5Serial.available()) C5Serial.read();
    
    // Verify C5 is responding
    Serial.println("\n[TEST 0] Basic UART communication...");
    C5Serial.print("AT\r\n");
    delay(500);
    String resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    
    if (resp.indexOf("OK") >= 0) {
        Serial.println("[PASS] C5 responding to AT commands");
    } else {
        Serial.println("[FAIL] C5 not responding!");
        Serial.printf("       Got: %s\n", resp.c_str());
        Serial.println("\n*** Fix basic UART first before testing flow control ***");
        return;
    }
    
    // Query C5's current UART settings
    Serial.println("\n[INFO] Querying C5 UART settings...");
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT+UART_CUR?\r\n");
    delay(500);
    resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    Serial.printf("[INFO] C5 UART: %s\n", resp.c_str());
    
    // =========================================
    // TEST 1: S3 RTS -> C5 CTS connection
    // =========================================
    Serial.println("\n" "═══════════════════════════════════════════════════");
    Serial.println("TEST 1: S3 RTS (GPIO21) -> C5 CTS (GPIO2) connection");
    Serial.println("═══════════════════════════════════════════════════\n");
    
    Serial.println("We'll toggle S3's RTS pin and see if C5 notices.\n");
    Serial.println("For this to work, C5 must be in CTS mode (flow_control=2).");
    Serial.println("But first, let's just see if the wire is connected by");
    Serial.println("checking if C5's behavior changes when we toggle RTS.\n");
    
    // Configure S3 RTS as output
    pinMode(C5_RTS_PIN, OUTPUT);
    
    // Test with RTS HIGH (C5 should be able to send)
    Serial.println("[TEST 1a] Setting S3 RTS HIGH (should allow C5 to send)...");
    digitalWrite(C5_RTS_PIN, HIGH);
    delay(100);
    
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT\r\n");
    delay(500);
    resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    bool highWorked = (resp.indexOf("OK") >= 0);
    Serial.printf("         Response: %s\n", highWorked ? "OK (good)" : "TIMEOUT or garbled");
    
    // Test with RTS LOW (if C5 were in CTS mode, it would pause)
    // But since C5 is NOT in CTS mode yet, this shouldn't affect anything
    Serial.println("\n[TEST 1b] Setting S3 RTS LOW...");
    Serial.println("         (C5 is not in CTS mode, so this shouldn't change anything yet)");
    digitalWrite(C5_RTS_PIN, LOW);
    delay(100);
    
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT\r\n");
    delay(500);
    resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    bool lowWorked = (resp.indexOf("OK") >= 0);
    Serial.printf("         Response: %s\n", lowWorked ? "OK (expected - C5 ignoring CTS)" : "TIMEOUT");
    
    // Put RTS back HIGH
    digitalWrite(C5_RTS_PIN, HIGH);
    
    // =========================================
    // TEST 2: C5 RTS -> S3 CTS connection
    // =========================================
    Serial.println("\n" "═══════════════════════════════════════════════════");
    Serial.println("TEST 2: C5 RTS (GPIO3) -> S3 CTS (GPIO37) connection");
    Serial.println("═══════════════════════════════════════════════════\n");
    
    // Configure S3 CTS as input
    pinMode(C5_CTS_PIN, INPUT);
    
    Serial.println("Reading S3 CTS pin (should reflect C5's RTS output)...");
    Serial.println("C5 is in RTS mode (flow_control=1), so it controls its RTS pin.\n");
    
    int ctsState = digitalRead(C5_CTS_PIN);
    Serial.printf("[INFO] S3 CTS (GPIO37) reads: %d (%s)\n", 
                  ctsState, ctsState ? "HIGH" : "LOW");
    
    if (ctsState == HIGH) {
        Serial.println("       C5's RTS is HIGH, meaning C5 is ready to receive");
        Serial.println("       (This suggests the wire IS connected)");
    } else {
        Serial.println("       C5's RTS is LOW, meaning C5 buffer might be full");
        Serial.println("       OR the wire is not connected");
    }
    
    // =========================================
    // TEST 3: Enable CTS mode on C5 and test
    // =========================================
    Serial.println("\n" "═══════════════════════════════════════════════════");
    Serial.println("TEST 3: Enable CTS mode on C5 and verify handshake");
    Serial.println("═══════════════════════════════════════════════════\n");
    
    Serial.println("This is the critical test. We will:");
    Serial.println("1. Ensure S3 RTS is HIGH (telling C5 'you can send')");
    Serial.println("2. Enable CTS mode on C5 (AT+UART_CUR=115200,8,1,0,2)");
    Serial.println("3. Try to communicate");
    Serial.println("4. Set S3 RTS LOW and see if C5 stops responding");
    Serial.println("5. Set S3 RTS HIGH and see if C5 resumes\n");
    
    // Step 1: Ensure RTS is HIGH
    Serial.println("[STEP 1] Setting S3 RTS HIGH...");
    digitalWrite(C5_RTS_PIN, HIGH);
    delay(100);
    Serial.printf("         GPIO21 state: %d\n", digitalRead(C5_RTS_PIN));
    
    // Verify communication still works
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT\r\n");
    delay(500);
    resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    if (resp.indexOf("OK") < 0) {
        Serial.println("[FAIL] Lost communication before enabling CTS mode!");
        return;
    }
    Serial.println("         Communication OK before CTS enable");
    
    // Step 2: Enable CTS mode on C5
    Serial.println("\n[STEP 2] Enabling CTS mode on C5...");
    Serial.println("         Sending: AT+UART_CUR=115200,8,1,0,2");
    
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT+UART_CUR=115200,8,1,0,2\r\n");
    delay(1000);
    resp = "";
    while (C5Serial.available()) resp += (char)C5Serial.read();
    Serial.printf("         Response: %s\n", resp.c_str());
    
    if (resp.indexOf("OK") < 0) {
        Serial.println("[WARN] Didn't get OK, but let's try anyway...");
    }
    
    // Step 3: Try to communicate with RTS HIGH
    Serial.println("\n[STEP 3] Testing communication with CTS mode enabled, RTS HIGH...");
    delay(200);
    
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT\r\n");
    
    unsigned long start = millis();
    resp = "";
    while (millis() - start < 2000) {
        if (C5Serial.available()) {
            resp += (char)C5Serial.read();
            if (resp.indexOf("OK") >= 0) break;
        }
    }
    
    if (resp.indexOf("OK") >= 0) {
        Serial.println("[PASS] C5 responds with RTS HIGH - CTS mode working!");
    } else {
        Serial.println("[FAIL] C5 not responding even with RTS HIGH!");
        Serial.printf("       Got: '%s'\n", resp.c_str());
        Serial.println("\n       DIAGNOSIS: The S3 RTS -> C5 CTS wire may not be connected,");
        Serial.println("       or C5 is seeing CTS as LOW despite our HIGH output.");
        Serial.println("\n       Try swapping HIGH/LOW - maybe C5 expects active-low CTS?");
        
        // Try active-low
        Serial.println("\n[RETRY] Trying with RTS LOW instead (active-low CTS?)...");
        digitalWrite(C5_RTS_PIN, LOW);
        delay(200);
        
        while (C5Serial.available()) C5Serial.read();
        C5Serial.print("AT\r\n");
        start = millis();
        resp = "";
        while (millis() - start < 2000) {
            if (C5Serial.available()) {
                resp += (char)C5Serial.read();
                if (resp.indexOf("OK") >= 0) break;
            }
        }
        
        if (resp.indexOf("OK") >= 0) {
            Serial.println("[!!!] C5 responds with RTS LOW!");
            Serial.println("      This means C5 uses ACTIVE-LOW CTS logic!");
            Serial.println("      You need to invert the RTS signal or use a different approach.");
        } else {
            Serial.println("[FAIL] Still no response. Wire is likely not connected.");
        }
        
        // Attempt recovery
        Serial.println("\n[RECOVERY] Trying to restore C5 to no flow control...");
        digitalWrite(C5_RTS_PIN, HIGH);
        delay(100);
        digitalWrite(C5_RTS_PIN, LOW);
        delay(100);
        digitalWrite(C5_RTS_PIN, HIGH);
        delay(100);
        
        C5Serial.print("AT+UART_CUR=115200,8,1,0,0\r\n");
        delay(1000);
        
        while (C5Serial.available()) C5Serial.read();
        C5Serial.print("AT\r\n");
        delay(500);
        resp = "";
        while (C5Serial.available()) resp += (char)C5Serial.read();
        
        if (resp.indexOf("OK") >= 0) {
            Serial.println("[OK] Recovery successful - C5 back to normal");
        } else {
            Serial.println("[!!!] Recovery failed - power cycle C5 to restore");
        }
        
        return;
    }
    
    // Step 4: Set RTS LOW - C5 should stop responding
    Serial.println("\n[STEP 4] Setting S3 RTS LOW - C5 should PAUSE...");
    digitalWrite(C5_RTS_PIN, LOW);
    delay(100);
    
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT\r\n");
    
    start = millis();
    resp = "";
    while (millis() - start < 1500) {  // Shorter timeout - we expect no response
        if (C5Serial.available()) {
            resp += (char)C5Serial.read();
        }
    }
    
    if (resp.indexOf("OK") >= 0) {
        Serial.println("[FAIL] C5 still responding with RTS LOW!");
        Serial.println("       Flow control is NOT working.");
        Serial.println("       C5 is ignoring the CTS signal.");
    } else if (resp.length() == 0) {
        Serial.println("[PASS] C5 paused (no response) - flow control working!");
    } else {
        Serial.printf("[PARTIAL] Got partial response: '%s'\n", resp.c_str());
        Serial.println("          Flow control might be partially working.");
    }
    
    // Step 5: Set RTS HIGH - C5 should resume
    Serial.println("\n[STEP 5] Setting S3 RTS HIGH - C5 should RESUME...");
    digitalWrite(C5_RTS_PIN, HIGH);
    delay(100);
    
    // The previous AT command might now complete, or we send a new one
    start = millis();
    while (millis() - start < 1500) {
        if (C5Serial.available()) {
            resp += (char)C5Serial.read();
            if (resp.indexOf("OK") >= 0) break;
        }
    }
    
    if (resp.indexOf("OK") >= 0) {
        Serial.println("[PASS] C5 resumed and responded!");
        Serial.println("\n*** FLOW CONTROL IS WORKING! ***");
    } else {
        // Send fresh command
        while (C5Serial.available()) C5Serial.read();
        C5Serial.print("AT\r\n");
        delay(500);
        resp = "";
        while (C5Serial.available()) resp += (char)C5Serial.read();
        
        if (resp.indexOf("OK") >= 0) {
            Serial.println("[PASS] C5 responding again after RTS HIGH");
        } else {
            Serial.println("[FAIL] C5 not responding after RTS HIGH");
            Serial.println("       Something is stuck.");
        }
    }
    
    // =========================================
    // SUMMARY
    // =========================================
    Serial.println("\n" "═══════════════════════════════════════════════════");
    Serial.println("                    SUMMARY");
    Serial.println("═══════════════════════════════════════════════════\n");
    
    Serial.println("If TEST 3 passed (C5 paused with RTS LOW, resumed with RTS HIGH),");
    Serial.println("then hardware flow control CAN work between S3 and C5.\n");
    
    Serial.println("If TEST 3 failed, check:");
    Serial.println("  1. Wire between S3 GPIO21 and C5 GPIO2 (CTS)");
    Serial.println("  2. Voltage levels (both should be 3.3V logic)");
    Serial.println("  3. Whether C5's CTS pin is actually GPIO2 in your build\n");
    
    Serial.println("To restore C5 to normal after this test, power cycle it");
    Serial.println("or send: AT+UART_CUR=115200,8,1,0,0");
    
    // Restore C5 to no flow control for clean exit
    Serial.println("\n[CLEANUP] Restoring C5 to no flow control...");
    while (C5Serial.available()) C5Serial.read();
    C5Serial.print("AT+UART_CUR=115200,8,1,0,0\r\n");
    delay(500);
}

void loop() {
    // Interactive mode - forward between Serial and C5Serial
    if (Serial.available()) {
        C5Serial.write(Serial.read());
    }
    if (C5Serial.available()) {
        Serial.write(C5Serial.read());
    }
}