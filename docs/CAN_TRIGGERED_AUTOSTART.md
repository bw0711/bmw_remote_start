# CAN-Triggered Auto-Start Feature

**Document Version:** 1.0  
**Date:** January 4, 2026  
**Branch:** `feature/can-triggered-autostart`

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Codebase Analysis](#codebase-analysis)
3. [Implementation Details](#implementation-details)
4. [Configuration Guide](#configuration-guide)
5. [Testing Instructions](#testing-instructions)
6. [Risk Assessment](#risk-assessment)
7. [Appendix: Code Diff](#appendix-code-diff)

---

## Executive Summary

This document describes the implementation of a new feature that allows the BMW F30 remote start system to be triggered via an external CAN frame. This enables integration with aftermarket alarm systems, GSM modules, or any device capable of sending CAN bus messages.

### Key Feature

When the system detects CAN frame `0x3B3` with payload `0F 00 00 00 00 F0 F8`, it immediately initiates the auto-start sequence (same as triple-lock trigger).

### Changes Made

| Metric | Value |
|--------|-------|
| Files modified | 1 (`bmw_remote_start.ino`) |
| Lines added | 63 |
| Lines removed | 0 |
| New functions | 1 (`can_payload_matches`) |

---

## Codebase Analysis

### Architecture Overview

The BMW Remote Start system is built on an Arduino Nano with an MCP2515 CAN bus interface. The architecture follows a simple polling loop pattern:

```
┌─────────────────────────────────────────────────────────────┐
│                        Main Loop                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   CAN RX    │───▶│   Status    │───▶│   Action    │     │
│  │  (MCP2515)  │    │   Update    │    │  Execution  │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                  │              │
│         ▼                  ▼                  ▼              │
│   Hardware         Boolean flags        Relay sequences     │
│   Filters          (status_*)           (engine_do_*)       │
└─────────────────────────────────────────────────────────────┘
```

### CAN Frame Processing

#### Data Model

The system uses the `can_frame` struct from the MCP2515 library:

```cpp
struct can_frame {
    uint32_t can_id;    // CAN arbitration ID
    uint8_t  can_dlc;   // Data Length Code (0-8)
    uint8_t  data[8];   // Payload bytes
};
```

#### Frame Ingestion

Frames are read in `can_updateStatus()` via:

```cpp
if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Process frame based on can_id
}
```

#### Hardware Filtering

The MCP2515 chip performs hardware-level filtering to reduce CPU load. Only frames matching configured filters reach the Arduino:

| Filter | CAN ID | Purpose |
|--------|--------|---------|
| RXF0 | 0x21A | Brake light status |
| RXF1 | 0x23A | Key fob buttons |
| RXF2 | 0x0A5 | Engine running status |
| **RXF3** | **0x3B3** | **External auto-start trigger (NEW)** |

### Existing Trigger Patterns

The codebase uses three trigger patterns:

#### Pattern 1: Bit Mask (Brake Light)
```cpp
if (canMsg.data[0] & 0x80)  // Check bit 7 of byte 0
```

#### Pattern 2: Multi-Byte Match (Key Fob)
```cpp
if ((canMsg.data[1] == 0xF3) &&
    (canMsg.data[2] == 0x04) &&
    (canMsg.data[3] == 0x3F))
```

#### Pattern 3: Zero Check (Engine Status)
```cpp
if ((canMsg.data[5] == 0x00) &&
    (canMsg.data[6] == 0x00))
```

### Observations

1. **No DLC validation** - Existing code assumes frames have sufficient bytes
2. **No debouncing** - Relies on CAN message timing/frequency
3. **Simple state machine** - Uses boolean flags to coordinate actions

---

## Implementation Details

### Design Decision

**Chosen approach:** Strict matching (CAN ID + DLC + Full Payload)

**Rationale:**
- Auto-start is a safety-critical action; false positives must be avoided
- The specific payload (`0F 00 00 00 00 F0 F8`) appears to be a fixed command
- The trailing bytes (`F0 F8`) may be a checksum/signature, adding validation

### Code Changes

#### 1. Configuration Constants (Lines 5-16)

```cpp
// ===== EXTERNAL AUTO-START TRIGGER CONFIGURATION =====
#define CAN_ID_AUTOSTART_TRIGGER 0x3B3
#define CAN_DLC_AUTOSTART_TRIGGER 7

const uint8_t AUTOSTART_TRIGGER_PAYLOAD[] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8};
const uint8_t AUTOSTART_TRIGGER_PAYLOAD_LEN = 7;

#define AUTOSTART_TRIGGER_DEBOUNCE_MS 5000
// ======================================================
```

**Design notes:**
- Constants at top of file for easy customization
- Debounce time (5 seconds) prevents accidental repeated triggers
- Payload stored as byte array for efficient comparison

#### 2. State Variable (Line 56)

```cpp
unsigned long last_autostart_trigger_time = 0;
```

Tracks the last trigger time for debouncing.

#### 3. Helper Function (Lines 82-90)

```cpp
bool can_payload_matches(const uint8_t* received, const uint8_t* expected, uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    if(received[i] != expected[i])
      return false;
  }
  return true;
}
```

**Design notes:**
- Reusable for future trigger patterns
- Early-exit optimization (fails fast on first mismatch)
- Takes length parameter for flexibility

#### 4. Hardware Filter (Line 215)

```cpp
mcp2515.setFilter(MCP2515::RXF3, 0, CAN_ID_AUTOSTART_TRIGGER);
```

**Critical:** Without this filter, frames with ID `0x3B3` would be discarded by the MCP2515 chip and never reach the Arduino.

#### 5. Frame Handler (Lines 166-197)

```cpp
else if(canMsg.can_id == CAN_ID_AUTOSTART_TRIGGER)
{
  // Check DLC matches expected length
  if(canMsg.can_dlc == CAN_DLC_AUTOSTART_TRIGGER)
  {
    // Check full payload match
    if(can_payload_matches(canMsg.data, AUTOSTART_TRIGGER_PAYLOAD, AUTOSTART_TRIGGER_PAYLOAD_LEN))
    {
      // Debounce check
      if(millis() - last_autostart_trigger_time > AUTOSTART_TRIGGER_DEBOUNCE_MS)
      {
        Serial.println("External auto-start trigger received!");
        last_autostart_trigger_time = millis();
        
        // Trigger engine start only if not already running/starting/stopping
        if(!engine_start && !engine_stop && !remote_started && !status_engine_running)
        {
          Serial.println("-> Initiating auto-start");
          engine_start = true;
        }
        else
        {
          Serial.println("-> Ignored (engine already running or in transition)");
        }
      }
      else
      {
        Serial.println("External trigger ignored (debounce)");
      }
    }
  }
}
```

**Validation layers:**
1. CAN ID match (hardware + software)
2. DLC check (must be exactly 7)
3. Full payload comparison (all 7 bytes)
4. Debounce check (5-second cooldown)
5. State check (not already starting/stopping)

---

## Configuration Guide

### Changing the Trigger Frame

To use a different CAN frame as the trigger, modify these constants:

```cpp
// CAN ID (11-bit, 0x000 - 0x7FF)
#define CAN_ID_AUTOSTART_TRIGGER 0x3B3

// Expected DLC
#define CAN_DLC_AUTOSTART_TRIGGER 7

// Expected payload bytes
const uint8_t AUTOSTART_TRIGGER_PAYLOAD[] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8};
const uint8_t AUTOSTART_TRIGGER_PAYLOAD_LEN = 7;
```

### Adjusting Debounce Time

```cpp
// Time in milliseconds (default: 5000 = 5 seconds)
#define AUTOSTART_TRIGGER_DEBOUNCE_MS 5000
```

### Partial Payload Matching

If only the first N bytes should be matched (e.g., last 2 bytes are variable):

```cpp
// Match only first 5 bytes
const uint8_t AUTOSTART_TRIGGER_PAYLOAD[] = {0x0F, 0x00, 0x00, 0x00, 0x00};
const uint8_t AUTOSTART_TRIGGER_PAYLOAD_LEN = 5;
```

---

## Testing Instructions

### Prerequisites

- Arduino IDE with MCP2515 library installed
- CAN bus analyzer/injector (e.g., Peak PCAN, CANalyzer, or second MCP2515)
- Serial monitor access (9600 baud)

### Test Cases

#### Test 1: Correct Frame Triggers Start

1. Power on Arduino
2. Open Serial Monitor (9600 baud)
3. Verify "BMW REMOTE START v1" appears
4. Inject CAN frame: `ID=0x3B3, DLC=7, Data=0F 00 00 00 00 F0 F8`
5. **Expected:** Serial shows:
   ```
   External auto-start trigger received!
   -> Initiating auto-start
   *** Engine starting ***
   ```

#### Test 2: Wrong CAN ID Ignored

1. Inject frame: `ID=0x3B4, DLC=7, Data=0F 00 00 00 00 F0 F8`
2. **Expected:** No output (filtered by MCP2515 hardware)

#### Test 3: Wrong DLC Ignored

1. Inject frame: `ID=0x3B3, DLC=8, Data=0F 00 00 00 00 F0 F8 00`
2. **Expected:** No "auto-start" output

#### Test 4: Wrong Payload Ignored

1. Inject frame: `ID=0x3B3, DLC=7, Data=0F 00 00 00 01 F0 F8`
2. **Expected:** No "auto-start" output

#### Test 5: Debounce Works

1. Inject correct frame twice within 5 seconds
2. **Expected:** 
   - First: "Initiating auto-start"
   - Second: "External trigger ignored (debounce)"

#### Test 6: Engine Running Blocks Trigger

1. Start engine normally (triple-lock or ignition)
2. Inject correct trigger frame
3. **Expected:** "Ignored (engine already running or in transition)"

### Debug Mode

To see all received CAN frames, temporarily add this at the start of `can_updateStatus()`:

```cpp
Serial.print("RX: ");
Serial.print(canMsg.can_id, HEX);
Serial.print(" DLC:");
Serial.print(canMsg.can_dlc, DEC);
Serial.print(" Data:");
for(int i = 0; i < canMsg.can_dlc; i++) {
  if(canMsg.data[i] < 0x10) Serial.print("0");
  Serial.print(canMsg.data[i], HEX);
  Serial.print(" ");
}
Serial.println();
```

---

## Risk Assessment

| Risk | Severity | Likelihood | Mitigation |
|------|----------|------------|------------|
| **False positive trigger** | High | Low | Strict 7-byte payload match + DLC check |
| **CAN ID collision with OEM** | High | Low | Verify 0x3B3 not used by BMW via CAN logs |
| **Repeated trigger spam** | Medium | Medium | 5-second debounce implemented |
| **Millis overflow** | Low | Low | Unsigned arithmetic handles wrap correctly |
| **Filter slot exhaustion** | Low | None | 3 slots remaining (RXF4, RXF5) |
| **DLC=8 with padding** | Medium | Medium | If observed, adjust DLC check or ignore |

### Recommendations

1. **Before deployment:** Capture CAN traffic to verify `0x3B3` is not used by any BMW module
2. **Security consideration:** Ensure the external trigger source (alarm/GSM) is secured
3. **Field testing:** Test with actual external device before relying on feature

---

## Appendix: Code Diff

```diff
diff --git a/bmw_remote_start.ino b/bmw_remote_start.ino
index 1008122..d9afb66 100644
--- a/bmw_remote_start.ino
+++ b/bmw_remote_start.ino
@@ -2,6 +2,19 @@
 #include <mcp2515.h>
 #include <SPI.h>
 
+// ===== EXTERNAL AUTO-START TRIGGER CONFIGURATION =====
+// CAN ID for external auto-start trigger (e.g., aftermarket alarm/GSM module)
+#define CAN_ID_AUTOSTART_TRIGGER 0x3B3
+#define CAN_DLC_AUTOSTART_TRIGGER 7
+
+// Expected payload for auto-start trigger: "3B3 7 0F 00 00 00 00 F0 F8"
+const uint8_t AUTOSTART_TRIGGER_PAYLOAD[] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8};
+const uint8_t AUTOSTART_TRIGGER_PAYLOAD_LEN = 7;
+
+// Debounce: minimum time between trigger activations (ms)
+#define AUTOSTART_TRIGGER_DEBOUNCE_MS 5000
+// ======================================================
+
 //relays use inverted logic
 enum RELAY_STATUS {
     RELAY_HIGH = 0,
@@ -40,6 +53,9 @@ bool wait_for_lock_release = false;
 unsigned long last_lock_detected_time = 0;
 unsigned long cur_lock_detected_time = 0;
 
+//external auto-start trigger data
+unsigned long last_autostart_trigger_time = 0;
+
 //can library data
 struct can_frame canMsg;
 MCP2515 mcp2515(10);
@@ -56,8 +72,22 @@ MCP2515 mcp2515(10);
  * 
  * ENGINE STATUS
  * 0x0A5 : data: Byte[5] = 0x00 and Byte[5] = 0x00 -> if engine is NOT running
+ * 
+ * EXTERNAL AUTO-START TRIGGER
+ * 0x3B3 : data: 0F 00 00 00 00 F0 F8 -> external auto-start command
  */
 
+// Helper function to compare CAN payload bytes
+bool can_payload_matches(const uint8_t* received, const uint8_t* expected, uint8_t len)
+{
+  for(uint8_t i = 0; i < len; i++)
+  {
+    if(received[i] != expected[i])
+      return false;
+  }
+  return true;
+}
+
 void can_updateStatus()
 {
@@ -133,6 +163,38 @@ void can_updateStatus()
          status_engine_running = true;
        }
     }
+    else if(canMsg.can_id == CAN_ID_AUTOSTART_TRIGGER)
+    {
+      if(canMsg.can_dlc == CAN_DLC_AUTOSTART_TRIGGER)
+      {
+        if(can_payload_matches(canMsg.data, AUTOSTART_TRIGGER_PAYLOAD, AUTOSTART_TRIGGER_PAYLOAD_LEN))
+        {
+          if(millis() - last_autostart_trigger_time > AUTOSTART_TRIGGER_DEBOUNCE_MS)
+          {
+            Serial.println("External auto-start trigger received!");
+            last_autostart_trigger_time = millis();
+            
+            if(!engine_start && !engine_stop && !remote_started && !status_engine_running)
+            {
+              Serial.println("-> Initiating auto-start");
+              engine_start = true;
+            }
+            else
+            {
+              Serial.println("-> Ignored (engine already running or in transition)");
+            }
+          }
+          else
+          {
+            Serial.println("External trigger ignored (debounce)");
+          }
+        }
+      }
+    }
   }
 }
 
@@ -150,6 +212,7 @@ void can_setup()
   mcp2515.setFilter(MCP2515::RXF0, 0, 0x21A); //break light
   mcp2515.setFilter(MCP2515::RXF1, 0, 0x23A); //keyfob
   mcp2515.setFilter(MCP2515::RXF2, 0, 0x0A5); //rpm
+  mcp2515.setFilter(MCP2515::RXF3, 0, CAN_ID_AUTOSTART_TRIGGER); //external trigger
 
   //start the sniffing
   mcp2515.setNormalMode();
```

---

*Document generated for BMW Remote Start project*  
*Feature branch: `feature/can-triggered-autostart`*


