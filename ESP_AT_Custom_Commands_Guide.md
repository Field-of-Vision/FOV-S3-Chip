# Adding Custom AT Commands to ESP-AT: A Complete Guide

This guide documents the process of adding custom AT commands to the ESP-AT firmware, based on implementing `AT+FWMEMINFO?` for the ESP32-C5. It includes common pitfalls and their solutions.

## Overview

**Goal:** Add a custom AT command (`AT+FWMEMINFO?`) to query PSRAM and internal RAM availability.

**Final Result:**
```
AT+FWMEMINFO?
+FWMEMINFO:PSRAM,4179640,4128768
+FWMEMINFO:INTERNAL,118575,65536
OK
```

---

## Step-by-Step Implementation

### Step 1: Create the Source File

Create `at_fw_buffer_cmd.c` in `components/at/src/`:

```c
/*
 * Custom AT Commands for Firmware Buffer OTA
 */
#include <stdio.h>
#include <string.h>
#include "esp_at.h"
#include "esp_at_cmd_register.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "AT_FWBUF";

// ============ AT+FWMEMINFO? ============
static uint8_t at_query_cmd_fwmeminfo(uint8_t *cmd_name)
{
    char resp[128];
    
    // Query PSRAM (external SPI RAM)
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    
    // Query internal RAM
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
    // Build response string
    // NOTE: Use %u with (unsigned int) cast, NOT %zu - see "Pitfall #4"
    int len = snprintf(resp, sizeof(resp),
             "+FWMEMINFO:PSRAM,%u,%u\r\n"
             "+FWMEMINFO:INTERNAL,%u,%u\r\n",
             (unsigned int)psram_free, (unsigned int)psram_largest, 
             (unsigned int)internal_free, (unsigned int)internal_largest);
    
    // Send response
    esp_at_port_write_data((uint8_t*)resp, len);
    
    return ESP_AT_RESULT_CODE_OK;
}

// ============ Command Table ============
// CRITICAL: Order is {name, TEST, QUERY, SETUP, EXECUTE}
//           See "Pitfall #3" for details
static const esp_at_cmd_struct at_fw_buffer_cmd[] = {
    {"+FWMEMINFO", NULL, at_query_cmd_fwmeminfo, NULL, NULL},
    //             ^TEST ^QUERY                  ^SETUP ^EXEC
};

// ============ Registration Function ============
bool esp_at_fw_buffer_cmd_regist(void)
{
    ESP_LOGI(TAG, "Registering firmware buffer AT commands");
    
    return esp_at_custom_cmd_array_regist(
        at_fw_buffer_cmd, 
        sizeof(at_fw_buffer_cmd) / sizeof(at_fw_buffer_cmd[0])
    );
}
```

### Step 2: Add Source to CMakeLists.txt

Edit `components/at/CMakeLists.txt` and add your file to the sources list:

```cmake
set(srcs "")
list(APPEND srcs "src/at_default_config.c")
list(APPEND srcs "src/at_init.c")
list(APPEND srcs "src/at_workaround.c")
list(APPEND srcs "src/at_cmd_register.c")
list(APPEND srcs "src/at_fw_buffer_cmd.c")  # <-- ADD THIS LINE
```

### Step 3: Register the Command in at_cmd_register.c

Edit `components/at/src/at_cmd_register.c` and add the registration macro:

```c
#ifdef CONFIG_AT_ETHERNET_SUPPORT
ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_eth_cmd_regist, 20);
#endif

// ADD THIS LINE - Custom firmware buffer commands for OTA
ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_fw_buffer_cmd_regist, 25);

void esp_at_cmd_set_register(void)
{
    // ... rest of function
```

**Key points:**
- Place it AFTER the last `#endif` of the existing commands
- Use a priority number (25) that doesn't conflict with existing ones (1-24 are used)
- The macro must be in `at_cmd_register.c`, NOT in your own source file

### Step 4: Add Linker Flag to force_symbol_ref.cmake

Edit `components/at/force_symbol_ref.cmake` and add at the END of the file:

```cmake
# Custom firmware buffer commands for OTA
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u esp_at_fw_buffer_cmd_regist")
```

### Step 5: Build and Flash

```bash
cd ~/esp-at
./build.py build
./build.py flash -p /dev/ttyUSB0
```

### Step 6: Test

```bash
# Simple test
echo -e "AT+FWMEMINFO?\r\n" > /dev/ttyUSB0
cat /dev/ttyUSB0

# Or use Python
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.5)
ser.write(b'AT+FWMEMINFO?\r\n')
time.sleep(0.5)
print(ser.read_all().decode())
ser.close()
"
```

---

## Common Pitfalls and Solutions

### Pitfall #1: Registration Macro in Wrong File

❌ **Wrong:** Putting `ESP_AT_CMD_SET_FIRST_INIT_FN()` in your own source file
```c
// at_fw_buffer_cmd.c
ESP_AT_CMD_SET_FIRST_INIT_FN(esp_at_fw_buffer_cmd_regist, 25);  // WON'T WORK
```

✅ **Correct:** Put the macro in `at_cmd_register.c` where all the other registrations are

**Why:** The linker section ordering is critical. When the macro is in a separate file, the struct might not be placed correctly in the init array, even if it appears to be there in the binary.

### Pitfall #2: Function Naming Convention

❌ **Wrong:** Using non-standard function name
```c
bool at_fw_buffer_cmd_register(void)  // Wrong - doesn't follow convention
```

✅ **Correct:** Follow the ESP-AT naming convention
```c
bool esp_at_fw_buffer_cmd_regist(void)  // Correct - esp_at_<module>_cmd_regist
```

**Why:** The linker flags and macros expect this naming pattern.

### Pitfall #3: Wrong Order in Command Struct

❌ **Wrong:** Query handler in wrong position
```c
{"+FWMEMINFO", at_query_cmd_fwmeminfo, NULL, NULL, NULL},
//             ^-- This is TEST position, not QUERY!
```

✅ **Correct:** Query handler in second position
```c
{"+FWMEMINFO", NULL, at_query_cmd_fwmeminfo, NULL, NULL},
//             ^TEST ^QUERY                  ^SETUP ^EXEC
```

**The struct order is:**
```c
{
    "command_name",
    test_handler,    // AT+CMD=?  (position 1)
    query_handler,   // AT+CMD?   (position 2)
    setup_handler,   // AT+CMD=x  (position 3)
    execute_handler  // AT+CMD    (position 4)
}
```

### Pitfall #4: Format Specifier for size_t

❌ **Wrong:** Using `%zu` for size_t
```c
snprintf(resp, sizeof(resp), "+FWMEMINFO:PSRAM,%zu,%zu\r\n", psram_free, psram_largest);
// Output: +FWMEMINFO:PSRAM,zu,zu  (literal "zu" instead of numbers!)
```

✅ **Correct:** Use `%u` with explicit cast
```c
snprintf(resp, sizeof(resp), "+FWMEMINFO:PSRAM,%u,%u\r\n", 
         (unsigned int)psram_free, (unsigned int)psram_largest);
```

**Why:** The `%zu` format specifier may not be fully supported on all embedded platforms.

### Pitfall #5: PSRAM Not Enabled

If `AT+FWMEMINFO?` returns `PSRAM,0,0`:

1. Check sdkconfig:
```bash
grep "CONFIG_SPIRAM" ~/esp-at/sdkconfig
# Should show: CONFIG_SPIRAM=y
# If it shows: # CONFIG_SPIRAM is not set  <-- PROBLEM!
```

2. Enable PSRAM:
```bash
./build.py menuconfig
# Navigate to: Component config → ESP PSRAM → Support for external, SPI-connected RAM
# Press Y to enable, save and exit
```

3. Rebuild and flash

### Pitfall #6: Testing Wrong Command Format

❌ **Wrong:** Sending execute format when you defined query handler
```
AT+FWMEMINFO    <-- Execute format (calls execute_handler)
ERROR
```

✅ **Correct:** Use query format for query handler
```
AT+FWMEMINFO?   <-- Query format (calls query_handler)
+FWMEMINFO:PSRAM,4179640,4128768
OK
```

### Pitfall #7: Missing Linker Flag

If the command compiles but never registers (doesn't appear in `AT+CMD?`):

Check that `force_symbol_ref.cmake` has:
```cmake
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u esp_at_fw_buffer_cmd_regist")
```

**Why:** Without the `-u` flag, the linker may optimize away the registration struct since nothing explicitly references it.

---

## Debugging Checklist

If your command isn't working, check these in order:

### 1. Is the file being compiled?
```bash
grep "at_fw_buffer_cmd" ~/esp-at/build/compile_commands.json
```

### 2. Is the symbol in the binary?
```bash
nm ~/esp-at/build/esp-at.elf | grep fw_buffer
```

### 3. Is the command in the registered list?
Send `AT+CMD?` and look for your command in the output.

### 4. Is the registration function being called?
Add a `printf()` or `ESP_LOGE()` in your registration function and check boot logs.

### 5. Clean rebuild if in doubt
```bash
cd ~/esp-at
rm -rf build
./build.py set-target esp32c5
./build.py build
./build.py flash -p /dev/ttyUSB0
```

---

## File Checklist

After implementation, verify these files are modified:

| File | Change |
|------|--------|
| `components/at/src/at_fw_buffer_cmd.c` | New file - command implementation |
| `components/at/CMakeLists.txt` | Added source file to `srcs` list |
| `components/at/src/at_cmd_register.c` | Added `ESP_AT_CMD_SET_FIRST_INIT_FN` macro |
| `components/at/force_symbol_ref.cmake` | Added `-u` linker flag |
| `sdkconfig` | `CONFIG_SPIRAM=y` (if using PSRAM) |

---

## Command Types Reference

| Type | Format | Handler Signature | Position in Struct |
|------|--------|-------------------|-------------------|
| Test | `AT+CMD=?` | `uint8_t func(uint8_t *cmd_name)` | 1 (test) |
| Query | `AT+CMD?` | `uint8_t func(uint8_t *cmd_name)` | 2 (query) |
| Setup | `AT+CMD=params` | `uint8_t func(uint8_t para_num)` | 3 (setup) |
| Execute | `AT+CMD` | `uint8_t func(uint8_t *cmd_name)` | 4 (execute) |

---

## Summary

The key lessons learned:

1. **Put the registration macro in `at_cmd_register.c`** - not in your own file
2. **Follow the exact naming convention** - `esp_at_<module>_cmd_regist`
3. **Get the struct order right** - {name, TEST, QUERY, SETUP, EXECUTE}
4. **Use `%u` not `%zu`** for size_t in snprintf
5. **Enable PSRAM in menuconfig** if you need external RAM
6. **Test with the correct command format** - `?` for query, `=` for setup, etc.
7. **Add the linker flag** in `force_symbol_ref.cmake`

When in doubt, look at `at_user_cmd.c` as a reference - it's a working example of custom AT commands.
