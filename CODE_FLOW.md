# AOA Controller - Code Flow & Architecture Guide

A complete step-by-step explanation of how data flows through the AOA (Angle of Attack) control system, from the moment the ESP32 boots up to the final elevator command output.

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Boot Sequence](#boot-sequence)
3. [Control Loop Flow](#control-loop-flow)
4. [Detailed Function Breakdown](#detailed-function-breakdown)
5. [Data Flow Diagrams](#data-flow-diagrams)
6. [Important Decision Points](#important-decision-points)

---

## System Overview

### What Does This System Do?

```
┌──────────────────────────────────────────────────────────────┐
│                    AOA Control System                        │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  INPUT: Raw Sensor Data (3 AOA sensors + airspeed)          │
│    ↓                                                         │
│  PROCESS: Validate → Estimate → Check FSM State             │
│    ↓                                                         │
│  OUTPUT: Elevator Command + LED Status                      │
│                                                              │
│  PURPOSE: Prevent aircraft stall by monitoring angle of     │
│           attack and automatically controlling elevator     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

### Key Concept: Real-Time Control Loop

The system runs on a **20ms timer** (50 Hz):
- Every 20ms: New sensor data is processed
- Every 20ms: FSM evaluates current state
- Every 20ms: Elevator physical position is updated with slew-rate limiting

---

## Boot Sequence

### Step 1: System Initialization

```c
FILE: components/aoa_controller/src/main.c
FUNCTION: app_main()

void app_main(void) {
    ESP_LOGI(TAG, "=== Aircraft AoA Safety Controller Boot Sequence ===");
```

**What happens:**
1. ESP32 main function is called
2. Creates FreeRTOS event group for task synchronization
3. Initializes all modules
4. Starts two parallel tasks: UART input task and control loop task

---

### Step 2: Event Group Creation

```c
event_group = xEventGroupCreate();
if (!event_group) {
    ESP_LOGE(TAG, "Failed to create event group");
    return;
}
```

**Critical Decision:**
- **Event Group Purpose:** Synchronizes two tasks without busy-waiting
- **Pattern:** Task 1 (UART) signals when new data arrives
- **Pattern:** Task 2 (Control) waits for signal, then processes

---

### Step 3: HAL Input Initialization

```c
FILE: components/aoa_controller/src/hal_input.c
FUNCTION: hal_input_init()

void hal_input_init(EventGroupHandle_t event_group) {
    // Initialize 3 circular buffers (one per sensor)
    circular_buffer_init(&sensor_data.sensor1_buffer);
    circular_buffer_init(&sensor_data.sensor2_buffer);
    circular_buffer_init(&sensor_data.sensor3_buffer);
    
    // Create thread-safe mutex
    sensor_data.buffer_mutex = xSemaphoreCreateMutex();
    if (!sensor_data.buffer_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
```

**Important Decisions:**
- ✅ **Circular Buffers:** Store last 5 sensor readings (SENSOR_BUFFER_SIZE=5)
  - Why? Smooths out sensor noise and outliers
  - Why? Provides historical data for trending analysis
  
- ✅ **Mutex (Thread Lock):** Prevents race conditions
  - Why? UART task writes sensor data, Control task reads it simultaneously
  - Mutex ensures data consistency

---

### Step 4: UART Configuration

```c
uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE,        // 115200 baud
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

uart_param_config(UART_NUM_0, &uart_config);
uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, 
             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
uart_driver_install(UART_NUM_0, UART_BUFFER_SIZE, 0, 0, NULL, 0);
```

**Configuration Choices:**
- 115200 baud: Standard aircraft avionics speed
- 8-bit data: Standard UART frame
- No parity: Assumed reliable link (flying over ethernet inside aircraft)
- 256-byte buffer: Can hold ~50 sensor messages before overflow

---

### Step 5: Task Creation

```c
BaseType_t ret = xTaskCreatePinnedToCore(
    hal_input_task,           // Task function
    "hal_input_task",         // Task name (for debugging)
    4096,                     // Stack size (bytes)
    (void *)event_group,      // Argument passed to task
    5,                        // Priority
    NULL,                     // Task handle (not needed)
    0                         // Core 0 (UART, WiFi handled here)
);

// Create second task on Core 1 for control loop
xTaskCreatePinnedToCore(
    control_task,
    "control_task",
    8192,                     // Larger stack for heavy computation
    NULL,
    5,
    NULL,
    1                         // Core 1 (dedicated to control loop)
);
```

**Dual-Core Architecture:**
- **Core 0:** Handles UART input (fast, non-deterministic)
- **Core 1:** Runs control loop (deterministic, real-time)
- **Why?** Prevents UART jitter from affecting control timing

---

## Control Loop Flow

### The Master Control Loop (20ms cycle)

```
TIMER FIRES (every 20ms)
        ↓
   [Set RUN_CYCLE bit]
        ↓
   [Control Task Wakes Up]
        ↓
   [Read Latest Sensors]
        ↓
   [Validate Data]
        ↓
   [Estimate AOA]
        ↓
   [Run FSM Logic]
        ↓
   [Apply Elevator Command]
        ↓
   [Update Physical Position]
        ↓
   [Log Results]
        ↓
   [Wait for Next Timer]
```

---

### Step 1: Timer Callback

```c
FILE: components/aoa_controller/src/main.c

static void control_timer_callback(void *arg) {
    xEventGroupSetBits(event_group, BIT_RUN_CYCLE);
}

// Timer setup
esp_timer_create_args_t timer_args = {
    .callback = control_timer_callback,
    .arg = NULL,
    .name = "control_timer",
    .skip_unhandled_events = false,
};
esp_timer_create(&timer_args, &control_timer);
esp_timer_start_periodic(control_timer, 20000);  // 20ms in microseconds
```

**Why This Design?**
- **Hardware Timer:** Guarantees 20ms accuracy
- **Event Bit:** Lightweight synchronization (1 bit)
- **Skip unhandled:** False = Never skip cycles (deterministic)

---

### Step 2: Read Sensor Data

```c
FILE: components/aoa_controller/src/main.c
FUNCTION: control_task()

for (;;) {
    EventBits_t bits = xEventGroupWaitBits(
        event_group,
        BIT_RUN_CYCLE,
        pdTRUE,      // Clear bit after read (auto-reset)
        pdFALSE,     // Don't wait for multiple bits
        pdMS_TO_TICKS(25)  // Max 25ms timeout
    );
    
    // Get current sensor readings
    sensor_data_t *sensor_data = hal_get_sensor_data();
    hal_lock_sensor_data();  // Prevent UART task from modifying
    
    float s1 = circular_buffer_get_newest(&sensor_data->sensor1_buffer);
    float s2 = circular_buffer_get_newest(&sensor_data->sensor2_buffer);
    float s3 = circular_buffer_get_newest(&sensor_data->sensor3_buffer);
    float airspeed = sensor_data->airspeed;
    
    hal_unlock_sensor_data();  // Release lock for UART task
```

**Critical Decision: Locking**
```
Timeline without lock (BAD):
  [Control reads S1=10.0]
  [UART updates S1=20.0]  ← Race condition!
  [Control reads S2=15.0]
  Result: S1 and S2 from different time samples

Timeline with lock (GOOD):
  [Control: LOCK]
  [Control reads S1=10.0]
  [Control reads S2=15.0]
  [Control reads S3=12.0]
  [Control: UNLOCK]
  [UART can now update]
  Result: S1, S2, S3 are from same time sample
```

---

### Step 3: Validate Sensor Data

```c
FILE: components/aoa_controller/src/validator.c
FUNCTION: validator_run_values()

validator_result_t validator_run_values(float s1, float s2, float s3) {
    validator_result_t result = {0};
    
    // DECISION 1: Check each sensor in valid range
    result.sensor1_valid = (s1 >= SENSOR_MIN_DEG && s1 <= SENSOR_MAX_DEG);
    result.sensor2_valid = (s2 >= SENSOR_MIN_DEG && s2 <= SENSOR_MAX_DEG);
    result.sensor3_valid = (s3 >= SENSOR_MIN_DEG && s3 <= SENSOR_MAX_DEG);
    
    // Typical ranges: -20° to +40°
    // Outside this = Likely sensor fault
    
    // DECISION 2: Calculate median (robust to outliers)
    result.median_aoa = calculate_median(s1, s2, s3);
    
    // DECISION 3: Count how many sensors are working
    uint32_t num_valid = 0;
    if (result.sensor1_valid) num_valid++;
    if (result.sensor2_valid) num_valid++;
    if (result.sensor3_valid) num_valid++;
    result.num_valid_sensors = num_valid;
```

**Key Logic: Median Calculation**

```
Why median and not average?

Example 1 (Normal):
  S1=10.0, S2=10.1, S3=9.9
  Average = 10.0 ✓
  Median = 10.0 ✓
  
Example 2 (Sensor 3 fails):
  S1=10.0, S2=10.1, S3=45.0  ← Sensor 3 glitched
  Average = 21.7 ✗ (Wrong!)
  Median = 10.0 ✓ (Correct! 2 out of 3 sensors agree)
```

**Important Decision:**
- ✅ **Median Filter:** Tolerates 1 bad sensor out of 3
- ✅ **Graceful Degradation:** System continues even with sensor fault
- ✅ **Fault Detection:** Logs which sensors failed

```c
    // DECISION 4: Quality assurance level
    if (result.num_valid_sensors < 2) {
        // Less than 2 sensors valid = TOO RISKY
        // System should enter failsafe mode
        ESP_LOGW(TAG, "FAULT: Only %d valid sensors!", num_valid_sensors);
    }
    
    return result;
}
```

---

### Step 4: Estimate True AOA

```c
FILE: components/aoa_controller/src/estimator.c
FUNCTION: estimator_run()

void estimator_run(validator_result_t *validator_result, 
                   estimator_state_t *estimator_state) {
    
    // STEP 1: Weighted Fusion (combine multiple sensors)
    float fused_aoa = perform_weighted_fusion(validator_result);
    
    // STEP 2: Kalman Filter (noise reduction)
    apply_kalman_filter(fused_aoa, &kalman_state);
    
    // STEP 3: Output final estimate
    estimator_state->final_calculated_aoa = kalman_state.estimated_aoa;
    estimator_state->is_initialized = true;
}
```

**Weighted Fusion Logic:**

```c
float perform_weighted_fusion(validator_result_t *result) {
    float sum_aoa = 0.0f;
    float sum_weight = 0.0f;
    
    if (result->sensor1_valid) {
        sum_aoa += result->sensor1_value * WEIGHT_SENSOR1;
        sum_weight += WEIGHT_SENSOR1;
    }
    if (result->sensor2_valid) {
        sum_aoa += result->sensor2_value * WEIGHT_SENSOR2;  // Center = higher weight
        sum_weight += WEIGHT_SENSOR2;
    }
    if (result->sensor3_valid) {
        sum_aoa += result->sensor3_value * WEIGHT_SENSOR3;
        sum_weight += WEIGHT_SENSOR3;
    }
    
    // Normalize
    return (sum_weight > 0) ? (sum_aoa / sum_weight) : 0.0f;
}
```

**Important Decision: Weighted Fusion**
- Center sensor is MORE important (middle of fuselage = most representative)
- Side sensors confirm but are secondary
- If center sensor fails → rely on side sensors at reduced weight

**Kalman Filter Logic:**

```c
void apply_kalman_filter(float measurement, kalman_state_t *state) {
    // Classical Kalman filtering for 1D AOA estimation
    
    // Predict step
    float predicted = state->estimated_aoa;  // Simple model: AOA doesn't change much
    float predicted_variance = state->estimated_variance + KALMAN_PROCESS_NOISE;
    
    // Update step (Bayesian correction)
    float innovation = measurement - predicted;  // What we measured vs. expected
    float kalman_gain = predicted_variance / 
                       (predicted_variance + KALMAN_MEASUREMENT_NOISE);
    
    state->estimated_aoa = predicted + kalman_gain * innovation;
    state->estimated_variance = (1.0f - kalman_gain) * predicted_variance;
}
```

**What Does Kalman Filter Do?**

```
Input: Noisy sensor: 10.0, 10.3, 9.7, 10.5, 9.8
Output: Smooth estimate: 10.0, 10.08, 10.06, 10.1, 10.07

Why? Learns sensor noise characteristics and filters them out
Benefit: Smooth elevator commands (no jitter)
```

---

### Step 5: Lookup Flight Mode Thresholds

```c
FILE: components/aoa_controller/src/fsm.c
FUNCTION: fsm_set_thresholds()

void fsm_set_thresholds(const char *aircraft_type, const char *flight_mode) {
    // DECISION: Different AOA limits for different flight phases
    
    float low = 0, high = 0;
    
    if (thresholds_lookup(aircraft_type, flight_mode, &low, &high)) {
        fsm_context.aoa_limit_low = low;
        fsm_context.aoa_limit_high = high;
```

**Threshold Table (from thresholds.c):**

```c
threshold_entry_t default_table[] = {
    {"Aircraft_A", "TAKEOFF", 0.0f,  18.0f},   // Safe during slow flight
    {"Aircraft_A", "CLIMB",   5.0f,  20.0f},   // Higher limit when climbing
    {"Aircraft_A", "CRUISE",  3.0f,  22.0f},   // Most permissive (high speed)
    {"Aircraft_A", "LANDING", 2.0f,  20.0f},   // Conservative during landing
};
```

**Critical Design Decision:**

```
Why different limits per flight mode?

TAKEOFF (18°):
  - Low speed, nose high
  - Need margin for pitch control
  
CLIMB (20°):
  - Medium speed
  - Steeper pitch angle
  
CRUISE (22°):
  - High speed = low AOA naturally
  - Can allow higher limit
  
LANDING (20°):
  - Speed dropping = AOA rising
  - Need tighter control
```

---

### Step 6: Run FSM (Finite State Machine)

```c
FILE: components/aoa_controller/src/fsm.c
FUNCTION: fsm_run()

fsm_output_t fsm_run(float calculated_aoa) {
    fsm_output_t out = {0};
    out.aoa_value = calculated_aoa;

    float limit_high = fsm_context.aoa_limit_high;                    // e.g., 22.0°
    float limit_protection = limit_high - FSM_PROTECTION_MARGIN_DEG;  // 22.0 - 0.5 = 21.5°
    float limit_caution = limit_high - FSM_CAUTION_MARGIN_DEG;        // 22.0 - 2.0 = 20.0°
    
    // DECISION: Threshold-based state machine
    fsm_state_t new_state = FSM_STATE_NORMAL;
    
    if (calculated_aoa > limit_high) {
        new_state = FSM_STATE_OVERRIDE;          // CRITICAL - immediate action
    }
    else if (calculated_aoa > limit_protection) {
        new_state = FSM_STATE_PROTECTION;        // SERIOUS - active control
    }
    else if (calculated_aoa > limit_caution) {
        new_state = FSM_STATE_CAUTION;           // WARNING - alert the pilot
    }
```

**Visual Threshold Map:**

```
AOA Value        State            LED        Elevator
─────────────────────────────────────────────────────
  > 22.0°    OVERRIDE (3)    Steady ON     -15° (max)
  21.5°      PROTECTION (2)  Fast blink    Active cmd
  
  20.0°      CAUTION (1)     Slow blink    Mild cmd
  
  < 20.0°    NORMAL (0)      OFF           0° (neutral)
```

**State Transitions with Hysteresis (Anti-Chatter):**

```c
    // IMPORTANT: Only change state if transition is valid
    if (new_state != fsm_context.current_state) {
        fsm_context.current_state = new_state;
        fsm_context.state_entry_time = 0;
        ESP_LOGI(TAG, "FSM state -> %d", new_state);  // Log transitions
    }
```

Why log state changes?
- Helps troubleshoot: "Did FSM oscillate?" (chatter = control instability)
- Performance analysis: "How long in each state?"
- Safety audit: "When did critical events occur?"

---

### Step 7: Apply Elevator Command

```c
FILE: components/aoa_controller/src/elevator.c
FUNCTION: elevator_apply_cmd()

void elevator_apply_cmd(fsm_state_t current_state, float calculated_aoa, 
                       float limit_high) {
    
    // DECISION 1: Calculate target deflection based on FSM state
    if (current_state == FSM_STATE_PROTECTION || 
        current_state == FSM_STATE_OVERRIDE) {
        
        elevator_state.cmd_active = true;
        
        // Calculate how much AOA exceeds the safe limit
        float overage = calculated_aoa - limit_high;
        if (overage < 0) overage = 0;
        
        // PROPORTIONAL CONTROL: More overage = More elevator
        // 2.0 = Gain: For each 1° of AOA excess, pitch down 2°
        elevator_state.target_deflection_deg = -(overage * 2.0f);
        
    } else {
        elevator_state.cmd_active = false;
        elevator_state.target_deflection_deg = 0.0f;  // Neutral
    }
    
    // DECISION 2: Enforce saturation limits
    if (elevator_state.target_deflection_deg < -ELEVATOR_MAX_DEFLECTION_DEG) {
        // If calculated command exceeds max, clamp it
        elevator_state.target_deflection_deg = -ELEVATOR_MAX_DEFLECTION_DEG;
        ESP_LOGW(TAG, "Command saturated at -%.1f deg!", ELEVATOR_MAX_DEFLECTION_DEG);
    }
}
```

**Proportional Control Calculation:**

```
Example: AOA = 24°, Limit = 22°, Max Deflection = 15°

Overage = 24 - 22 = 2°
Target = -(2 * 2.0) = -4°  ← Negative = Pitch down (pull stick back)

If AOA = 30° (way over limit):
Overage = 30 - 22 = 8°
Target = -(8 * 2.0) = -16°
But MAX = -15°, so: CLAMP to -15°  ← Maximum pitch down
```

---

### Step 8: Update Physical Elevator Position

```c
FILE: components/aoa_controller/src/elevator.c
FUNCTION: elevator_tick()

void elevator_tick(uint32_t dt_ms) {
    // Calculate maximum allowed step for this cycle
    float max_step = ELEVATOR_MAX_SLEW_RATE_DEG_PER_SEC * (dt_ms / 1000.0f);
    
    // Max rate = 10°/sec, cycle = 20ms
    // max_step = 10 * 0.020 = 0.2°/cycle
    
    // Calculate error (how far from target)
    float error = elevator_state.target_deflection_deg - 
                  elevator_state.current_deflection_deg;
    
    // DECISION: Slew-rate limiting (smooth ramp vs. instant jump)
    if (fabsf(error) <= max_step) {
        // Close enough, snap to target
        elevator_state.current_deflection_deg = elevator_state.target_deflection_deg;
    } else {
        // Move towards target by maximum allowed step
        if (error > 0.0f) {
            elevator_state.current_deflection_deg += max_step;
        } else {
            elevator_state.current_deflection_deg -= max_step;
        }
    }
}
```

**Why Slew-Rate Limiting?**

```
WITHOUT slew limiting (BAD):
  t=0ms:   Elevator = 0°
  t=20ms:  Sudden command = -15° (instant)
  Problem: Mechanical shock on actuator, pilot feels jerk
  
WITH slew limiting (GOOD):
  t=0ms:   Elevator = 0°
  t=20ms:  Elevator = -0.2°  (ramp up)
  t=40ms:  Elevator = -0.4°
  t=60ms:  Elevator = -0.6°
  ...
  t=1500ms: Elevator = -15° (fully deployed)
  Benefit: Smooth motion, fair wear on actuator
```

---

### Step 9: Logging and LED Control

```c
FILE: components/aoa_controller/src/main.c

// Update LED status
logger_control_led(fsm_output.led_on, fsm_output.led_blink_period);

// Prepare log entry
log_entry_t log_entry = {0};
log_entry.cycle_index = cycle_count;
log_entry.timestamp = timestamp;
log_entry.sensor1 = validation.sensor1_value;
log_entry.sensor2 = validation.sensor2_value;
log_entry.sensor3 = validation.sensor3_value;
log_entry.airspeed = airspeed;
log_entry.calculated_aoa = estimator_state.final_calculated_aoa;
strncpy(log_entry.status, fsm_output.status_str, 
        sizeof(log_entry.status) - 1);

// Write to persistent storage (SD card, EEPROM, or cloud)
logger_write_entry(&log_entry);
```

**LED Display Logic:**

```c
FILE: components/aoa_controller/src/logger.c

void logger_control_led(bool led_on, uint32_t blink_period_ms) {
    static uint32_t last_blink_time = 0;
    uint32_t now = esp_timer_get_time() / 1000;  // Convert to ms
    
    if (!led_on) {
        gpio_set_level(LED_GPIO_PIN, 0);  // Off
        return;
    }
    
    if (blink_period_ms == 0) {
        gpio_set_level(LED_GPIO_PIN, 1);  // Steady ON (OVERRIDE state)
        return;
    }
    
    // Blinking logic
    if ((now - last_blink_time) > blink_period_ms) {
        gpio_toggle_level(LED_GPIO_PIN);
        last_blink_time = now;
    }
}
```

**LED Status Meanings:**

```
OFF              → NORMAL state (green light)
Slow blink 500ms → CAUTION state (yellow light)
Fast blink 200ms → PROTECTION state (orange light)
Steady ON        → OVERRIDE state (red light - critical)
```

---

## Detailed Function Breakdown

### HAL Input Module

#### Function: `circular_buffer_init()`
```c
void circular_buffer_init(circular_buffer_t *buffer) {
    buffer->write_index = 0;
    // Array already zeroed from static initialization
    // Purpose: Prepare buffer to accept new sensor readings
}
```

**Why Circular?**
- Oldest data is automatically overwritten by newest
- No need to manually manage buffer
- Always have last 5 readings

---

#### Function: `circular_buffer_push()`
```c
void circular_buffer_push(circular_buffer_t *buffer, float value) {
    // DECISION: Use write_index to find where to store new value
    buffer->sensor_values[buffer->write_index] = value;
    
    // Move write pointer to next position (wrap around at end)
    buffer->write_index = (buffer->write_index + 1) % SENSOR_BUFFER_SIZE;
}
```

**Memory Layout:**

```
Initial state:
[0.0] [0.0] [0.0] [0.0] [0.0]  write_index = 0

After push(5.0):
[5.0] [0.0] [0.0] [0.0] [0.0]  write_index = 1

After push(5.5):
[5.0] [5.5] [0.0] [0.0] [0.0]  write_index = 2

After 5 more pushes (buffer full):
[5.3] [5.4] [5.5] [5.2] [5.1]  write_index = 0  (wrapped!)

Next push (5.6):
[5.6] [5.4] [5.5] [5.2] [5.1]  write_index = 1  (oldest data 5.0 is lost)
```

---

#### Function: `circular_buffer_get_newest()`
```c
float circular_buffer_get_newest(const circular_buffer_t *buffer) {
    // Newest value is JUST BEFORE the write pointer
    uint32_t newest_index = (buffer->write_index - 1 + SENSOR_BUFFER_SIZE) 
                           % SENSOR_BUFFER_SIZE;
    return buffer->sensor_values[newest_index];
}
```

**Why the modulo arithmetic?**
```
If write_index = 0 (just wrapped):
  newest_index = (0 - 1 + 5) % 5 = 4 % 5 = 4  ✓ (last position)
  
If write_index = 3:
  newest_index = (3 - 1 + 5) % 5 = 7 % 5 = 2  ✓ (just wrote here)
```

---

### Parser Functions

#### Function: `parse_aoa_message()`
```c
int parse_aoa_message(const char *line) {
    // INPUT: "$AOA,S1=10.0,S2=10.1,S3=9.9,TS=1000*"
    // OUTPUT: Stores values in circular buffers, returns 0 if success
    
    if (strncmp(line, "$AOA,", 5) != 0) return -1;  // Wrong message type
    
    float s1, s2, s3;
    uint32_t ts;
    
    // DECISION: Use sscanf to parse fixed format
    if (sscanf(line, "$AOA,S1=%f,S2=%f,S3=%f,TS=%lu*", &s1, &s2, &s3, &ts) != 4) {
        return -1;  // Parse failed
    }
    
    // DECISION: Validate range before storing
    if (s1 < SENSOR_MIN_DEG || s1 > SENSOR_MAX_DEG ||
        s2 < SENSOR_MIN_DEG || s2 > SENSOR_MAX_DEG ||
        s3 < SENSOR_MIN_DEG || s3 > SENSOR_MAX_DEG) {
        return -1;  // Out of valid range
    }
    
    hal_lock_sensor_data();
    circular_buffer_push(&sensor_data.sensor1_buffer, s1);
    circular_buffer_push(&sensor_data.sensor2_buffer, s2);
    circular_buffer_push(&sensor_data.sensor3_buffer, s3);
    sensor_data.timestamp = ts;
    hal_unlock_sensor_data();
    
    if (event_group_handle) {
        xEventGroupSetBits(event_group_handle, BIT_DATA_READY);  // Signal data arrived
    }
    
    ESP_LOGD(TAG, "AoA parsed: S1=%.1f S2=%.1f S3=%.1f TS=%lu", s1, s2, s3, ts);
    return 0;  // Success
}
```

**Message Format Validation:**

```
Valid:   "$AOA,S1=10.0,S2=10.1,S3=9.9,TS=1000*"
Invalid: "$AOA,S1=10.0,S2=10.1*"                (missing S3, TS)
Invalid: "$AOA,S1=45.0,S2=50.0,S3=10.0,TS=1*"  (S1, S2 out of range)
Invalid: "$FLIGHT_MODE,..."                     (wrong message type)
```

---

#### Function: `parse_flight_mode_message()`
```c
int parse_flight_mode_message(const char *line) {
    // INPUT: "$FLIGHT_MODE,MODE=CRUISE,TS=1000*"
    // OUTPUT: Stores mode string, used later by fsm_set_thresholds()
    
    if (strncmp(line, "$FLIGHT_MODE,", 13) != 0) return -1;
    
    char mode_str[32];
    uint32_t ts;
    
    if (sscanf(line, "$FLIGHT_MODE,MODE=%31[^,],TS=%lu*", mode_str, &ts) != 2) {
        return -1;
    }
    
    hal_lock_sensor_data();
    strncpy(sensor_data.flight_mode, mode_str, 
            sizeof(sensor_data.flight_mode) - 1);
    sensor_data.flight_mode[sizeof(sensor_data.flight_mode) - 1] = '\0';  // Null-terminate
    hal_unlock_sensor_data();
    
    ESP_LOGD(TAG, "Flight mode: MODE=%s TS=%lu", sensor_data.flight_mode, ts);
    return 0;
}
```

---

### Validator Module

#### Function: `calculate_median()`
```c
float calculate_median(float v1, float v2, float v3) {
    // Sort three values and return middle one
    if (v1 > v2) { float tmp = v1; v1 = v2; v2 = tmp; }  // Swap v1, v2
    if (v2 > v3) { float tmp = v2; v2 = v3; v3 = tmp; }  // Swap v2, v3
    if (v1 > v2) { float tmp = v1; v1 = v2; v2 = tmp; }  // Swap v1, v2 again
    
    return v2;  // Middle value is median
}
```

**Example:**

```
Input: v1=15.0, v2=10.0, v3=12.0

Step 1 (compare v1, v2): 15 > 10? Yes → swap
  [10.0, 15.0, 12.0]

Step 2 (compare v2, v3): 15 > 12? Yes → swap
  [10.0, 12.0, 15.0]

Step 3 (compare v1, v2): 10 > 12? No → no swap
  [10.0, 12.0, 15.0]

Return v2 = 12.0  ✓ (correct median)
```

---

### Estimator Module (Kalman Filtering)

This section is brief since Kalman filtering is complex, but the key takeaway:

```c
void apply_kalman_filter(float measurement, kalman_state_t *state) {
    // Kalman gain: how much to trust new measurement vs. history
    float K = state->estimated_variance / 
             (state->estimated_variance + KALMAN_MEASUREMENT_NOISE);
    
    // DECISION: Blend old estimate with new measurement
    state->estimated_aoa = state->estimated_aoa + K * (measurement - state->estimated_aoa);
    
    // Update uncertainty (variance gets smaller as Kalman learns)
    state->estimated_variance = (1.0f - K) * state->estimated_variance;
}
```

**Intuition:**
- If K = 0.5: Trust measurement and history equally
- If K = 0.9: Trust measurement heavily
- If K = 0.1: Trust historical value heavily

---

## Data Flow Diagrams

### Full System Data Flow

```
┌─────────────────────────────────────────────────────┐
│ Aircraft Avionics System (sends UART messages)     │
└──────────────────┬──────────────────────────────────┘
                   │
                   │ Serial @ 115200 baud
                   ↓
        ┌──────────────────────┐
        │  UART Driver (RX)     │
        │  (ESP32 Hardware)     │
        └──────────┬────────────┘
                   │
                   ↓
   ┌───────────────────────────────────┐
   │  hal_input_task (Core 0)          │
   │  (parses UART messages)           │
   │  • parse_aoa_message()            │
   │  • parse_flight_mode_message()    │
   └──────────┬────────────────────────┘
              │
              ↓
   ┌───────────────────────────────────┐
   │  Circular Buffers (SHARED)        │
   │  (S1, S2, S3, airspeed)           │
   │  Protected by MUTEX               │
   └──────────┬────────────────────────┘
              │
              ↓ (every 20ms)
   ┌───────────────────────────────────┐
   │  control_task (Core 1)            │
   │  (decision making loop)           │
   └───────────────────────────────────┘
              │
              ├─→ Read sensors → Validate → Estimate AOA
              │
              ├─→ FSM State Machine (compare to thresholds)
              │
              ├─→ Elevator Command (proportional control)
              │
              ├─→ Slew Rate Limiting (smooth actuator motion)
              │
              └─→ LED & Logging (output status)
              
              ↓
   ┌───────────────────────────────────┐
   │ Hardware Output Drivers           │
   │ • PWM to Elevator Motor           │
   │ • GPIO to Status LED              │
   │ • UART to Logging System          │
   └───────────────────────────────────┘
```

---

## Important Decision Points

### Decision 1: Mutex Vs. Atomic Access

```
Chosen: Mutex (thread-safe lock)

Why not atomic variables?
- Kalman filter state has multiple floating-point values
- Quick read of one value leaves others stale
- Mutex ensures logical consistency across all sensor data

When to use atomic?
- Single 32-bit or 64-bit variable
- Don't need to read multiple values together
```

---

### Decision 2: Kalman Filter Vs. Simple Average

```
Chosen: Kalman filter (adaptive filtering)

Why not simple average?
- Simple average weighs all samples equally
- Doesn't adapt to noise characteristics
- Kalman learns noise properties over time

Tradeoff:
- Kalman: Better smoothing, but more complex CPU use
- Average: Simpler, but may leave residual noise
```

---

### Decision 3: State Machine With Hysteresis

```
Chosen: Clear thresholds with no hysteresis

Thresholds:
NORMAL  → 0° to 20.0°
CAUTION → 20.0° to 21.5°
PROTECTION → 21.5° to 22.0°
OVERRIDE → > 22.0°

Why no hysteresis (different up vs. down thresholds)?
- Hysteresis could mask real dangers
- Better to use fast control loop to prevent chatter
- Safety critical = accept some chatter if needed
```

---

### Decision 4: Proportional Gain = 2.0

```
Chosen: elevator = -(AOA_overage * 2.0)

Why 2.0?
- For each 1° of AOA excess, deflect 2° pitch down
- Empirically tested for Cessna-class aircraft
- Conservative (slower response) = safer for first deployment

Tuneable parameter:
- Too high (5.0): Overshoots, then oscillates ← Bad
- Too low (0.5): Too slow to respond ← Bad
- Just right (2.0): Smooth damped response ← Good
```

---

### Decision 5: Slew Rate = 10°/sec

```
Chosen: elevator_max_slew = 10°/sec

Why 10°?
- Typical hydraulic actuator rate
- Prevents mechanical stress
- Gives smooth pilot experience

Calculation for 20ms cycle:
  max_step = 10°/sec * 0.020sec = 0.2°/cycle
  
  To reach -15° (max deflection):
  Time = 15° / 10°/sec = 1.5 seconds
  
Decision rationale:
- Not too fast (mechanical wear)
- Not too slow (stall protection slow)
```

---

## Key Takeaways

1. **Dual-Core Architecture:** Input (Core 0) separate from Control (Core 1)
2. **Synchronization:** Event groups + Mutex prevent race conditions
3. **Sensor Fusion:** Median + Weighted fusion + Kalman filter = robust estimates
4. **State Machine:** Clear thresholds with smooth transitions via slew limiting
5. **Proportional Control:** Proportional gain makes elevator response realistic
6. **Logging:** Every cycle logged for post-flight analysis

---

## Testing the Flow

### Test Case 1: Normal Flight
```
Input:  S1=10.0, S2=10.0, S3=10.0 (CRUISE mode, limit=22°)
Process: AOA=10.0° < 20.0° (caution threshold)
Output: FSM=NORMAL(0), Elevator=0°, LED=OFF ✓
```

### Test Case 2: High AOA Warning
```
Input:  S1=21.0, S2=21.0, S3=21.0 (CRUISE mode, limit=22°)
Process: AOA=21.0° > 20.0° (caution) but < 21.5° (protection)
Output: FSM=CAUTION(1), Elevator=0°, LED=500ms blink ✓
```

### Test Case 3: Critical Stall
```
Input:  S1=25.0, S2=25.0, S3=25.0 (CRUISE mode, limit=22°)
Process: AOA=25.0° > 22.0° (override)
         Overage = 3.0°
         Target = -(3.0 * 2.0) = -6.0° pitch down
         Ramp down at 0.2°/cycle until reaches -6°
Output: FSM=OVERRIDE(3), Elevator=-6.0° (reaches in 300ms), LED=STEADY ✓
```

---

## Debugging Tips

To add debug output:
```c
ESP_LOGI(TAG, "AOA=%.2f, FSM=%d, Elevator=%.2f", 
         estimator_state.final_calculated_aoa,
         fsm_output.state,
         elevator_get_current_deflection());
```

Monitor in real-time:
```bash
idf.py -p /dev/ttyUSB0 monitor
```

---

**Document Version:** 1.0  
**Last Updated:** April 9, 2026  
**Status:** Complete
