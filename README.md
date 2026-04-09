# AOA Controller (Angle of Attack Control System)

Real-time angle of attack monitoring and elevator control system for aircraft, with ESP32 firmware and PC-based simulation tools.

## Quick Start

### Option 1: Build for ESP32 (With Docker - Recommended)

Docker ensures a clean, reproducible build environment.

```bash
cd /workspaces/AOA_Controller_ty_1
docker run --rm -v $(pwd):/project -w /project espressif/idf idf.py build
docker run --rm -v $(pwd):/project -w /project espressif/idf idf.py flash
```

**Advantages:**
- ✅ No local ESP-IDF installation needed
- ✅ Guaranteed clean build environment
- ✅ Works on Linux, macOS, Windows with WSL
- ✅ Reproducible across different machines

### Option 2: Build for ESP32 (Without Docker - Native)

Prerequisites:
- ESP-IDF v6.1 installed: https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/get-started/linux-setup.html
- xtensa-esp32-elf toolchain
- esptool.py for flashing

```bash
cd /workspaces/AOA_Controller_ty_1

# Source ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Build
idf.py build

# Flash to ESP32 (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

**Configuration:**
- Edit `sdkconfig` or run `idf.py menuconfig` to adjust:
  - UART baud rate (default: 115200)
  - Sensor buffer size (default: 5 samples)
  - FSM thresholds (mode-specific AOA limits)

---

## PC-Based Simulation Tools

Test AOA control logic without ESP32 hardware using flight test data.

### 1. CSV Batch Simulator (sim_csv_safe.c)

Process flight test data from CSV file. Simulates all 4 flight phases (TAKEOFF, CLIMB, CRUISE, LANDING) with realistic FSM state transitions and elevator commands.

#### Compile:
```bash
cd /workspaces/AOA_Controller_ty_1
gcc -I . -I components/aoa_controller/include \
    sim_csv_safe.c components/aoa_controller/src/elevator.c \
    -lm -o sim_csv_safe
```

#### Run:
```bash
# Using included test data
./sim_csv_safe components/aoa_controller/data/sim_input.csv

# Output to file
./sim_csv_safe components/aoa_controller/data/sim_input.csv > results.csv

# View results (first 30 rows)
head -30 results.csv
```

#### Output Format:
```
idx,ts,mode,s1,s2,s3,aoa,fsm_state,elevator
1,1000,-,5.38,5.32,5.01,5.24,0,0.00
192,4820,LANDING,16.89,16.96,17.26,17.04,3,-0.20
```

| Column | Description |
|--------|-------------|
| idx | Row index |
| ts | Timestamp (milliseconds) |
| mode | Flight phase: TAKEOFF, CLIMB, CRUISE, LANDING |
| s1, s2, s3 | Raw sensor readings (degrees) |
| aoa | Computed angle of attack (average of s1/s2/s3) |
| fsm_state | 0=NORMAL, 1=CAUTION, 2=PROTECTION, 3=OVERRIDE |
| elevator | Elevator deflection command (degrees, -15 to +15) |

#### FSM State Thresholds (by Phase):
| Phase | AOA Threshold | CAUTION | PROTECTION | OVERRIDE |
|-------|---------------|---------|------------|----------|
| TAKEOFF | 15.0° | >13.0° | >14.5° | >15.0° |
| CLIMB | 18.0° | >16.0° | >17.5° | >18.0° |
| CRUISE | 22.0° | >20.0° | >21.5° | >22.0° |
| LANDING | 16.0° | >14.0° | >15.5° | >16.0° |

#### Key Features:
- ✅ No ESP-IDF dependencies (uses mock headers)
- ✅ Processes 200-row test flight
- ✅ Realistic scenario: normal cruise, sudden spikes during landing
- ✅ Slew-rate limited elevator (10°/sec max)
- ✅ Handles missing sensor data (0.00 values)

#### Example Output:
```
=== AOA Controller CSV Simulation (Safe Mode) ===
idx,ts,mode,s1,s2,s3,aoa,fsm_state,elevator
...
[TAKEOFF] 34,1660,TAKEOFF,15.20,14.90,15.28,15.13,3,-0.20  ← AOA exceeds limit
...
[CRUISE] 100,2980,CRUISE,3.44,3.60,3.11,3.38,0,0.00        ← Safe zone
...
[LANDING] 192,4820,LANDING,16.89,16.96,17.26,17.04,3,-0.20 ← Critical event
...
=== Complete (200 rows) ===
```

---

### 2. Interactive UART Simulator (simulator.c)

Real-time interactive testing with manual AOA message input. Uses actual project FSM and elevator control logic (links real source files).

#### Compile:
```bash
cd /workspaces/AOA_Controller_ty_1
gcc -I . -I components/aoa_controller/include \
    simulator.c \
    components/aoa_controller/src/hal_input.c \
    components/aoa_controller/src/elevator.c \
    components/aoa_controller/src/fsm.c \
    components/aoa_controller/src/thresholds.c \
    -lm -o simulator
```

#### Run:
```bash
./simulator
```

#### Usage:
```
Enter AOA message (e.g. $AOA,S1=10.0,S2=10.0,S3=10.0,TS=12345*): $AOA,S1=10.0,S2=10.0,S3=10.0,TS=1000*
[HAL_INPUT] AoA parsed: S1=10.0 S2=10.0 S3=10.0 TS=1000
AOA: 10.0, State: NORMAL, Elevator: 0.0, LED: OFF

Enter AOA message: $AOA,S1=20.0,S2=20.0,S3=20.0,TS=1020*
[HAL_INPUT] AoA parsed: S1=20.0 S2=20.0 S3=20.0 TS=1020
[ELEVATOR] Command saturated at -0.2 deg!
AOA: 20.0, State: PROTECTION, Elevator: -0.2, LED: ON

Enter AOA message: ^C
```

#### Message Format:
```
$AOA,S1=<value>,S2=<value>,S3=<value>,TS=<timestamp>*
```

| Field | Range | Unit | Description |
|-------|-------|------|-------------|
| S1 | 0-30 | degrees | Left sensor |
| S2 | 0-30 | degrees | Center sensor |
| S3 | 0-30 | degrees | Right sensor |
| TS | 0-999999999 | ms | Timestamp |

#### Test Scenarios:
```bash
# Test 1: Normal flight
$AOA,S1=5.0,S2=5.0,S3=5.0,TS=1000*

# Test 2: Moderate AOA (CAUTION state)
$AOA,S1=8.0,S2=8.0,S3=8.0,TS=1020*

# Test 3: High AOA (PROTECTION state)
$AOA,S1=15.0,S2=15.0,S3=15.0,TS=1040*

# Test 4: Critical AOA (OVERRIDE state with elevator action)
$AOA,S1=22.0,S2=22.0,S3=22.0,TS=1060*

# Test 5: Sensor fault (missing value)
$AOA,S1=10.0,S2=0.0,S3=10.0,TS=1080*
```

#### Features:
- ✅ Links real project FSM and elevator code (not reimplemented)
- ✅ Uses actual `parse_aoa_message()` function
- ✅ Real elevator slew-rate limiting
- ✅ LED status indicator
- ✅ Interactive debugging

---

## Project Structure

```
AOA_Controller_ty_1/
├── CMakeLists.txt                    # Build config
├── README.md                          # This file
├── sdkconfig                          # ESP32 configuration
├── components/aoa_controller/
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── config.h                  # System thresholds
│   │   ├── hal_input.h               # Sensor input (UART parser)
│   │   ├── elevator.h                # Elevator control
│   │   ├── fsm.h                     # FSM state machine
│   │   ├── thresholds.h              # Mode-based thresholds
│   │   └── ...
│   └── src/
│       ├── hal_input.c
│       ├── elevator.c
│       ├── fsm.c
│       ├── thresholds.c
│       └── ...
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                        # ESP32 firmware entry
│   └── Kconfig.projbuild
├── build/                            # Build artifacts (Docker)
├── simulator.c                       # Interactive UART simulator
├── sim_csv_safe.c                    # CSV batch simulator
├── sim_output.csv                    # Example output
├── components/aoa_controller/data/
│   └── sim_input.csv                 # Test flight data (200 rows)
└── ...
```

---

## Configuration

### ESP32 Build Options (sdkconfig)
```bash
# View/edit configuration
idf.py menuconfig

# Key settings:
# - UART_BAUD_RATE = 115200
# - SENSOR_BUFFER_SIZE = 5
# - ELEVATOR_MAX_DEFLECTION_DEG = 15
```

### Thresholds by Aircraft Mode
Edit `components/aoa_controller/include/thresholds.h`:

```c
// Example: Aircraft_A thresholds
#define AIRCRAFT_A_TAKEOFF_HIGH    15.0f
#define AIRCRAFT_A_CLIMB_HIGH      18.0f
#define AIRCRAFT_A_CRUISE_HIGH     22.0f
#define AIRCRAFT_A_LANDING_HIGH    16.0f
```

---

## Operation Modes

### FSM State Machine

**NORMAL (0)** - Green
- AOA below caution threshold
- Elevator neutral (0°)
- LED off

**CAUTION (1)** - Yellow  
- AOA 2° below protection threshold
- Elevator begins mild correction
- LED slow blink

**PROTECTION (2)** - Orange
- AOA 0.5° below override threshold
- Elevator fully active with slew limiting
- LED fast blink

**OVERRIDE (3)** - Red
- AOA exceeds mode threshold
- Maximum elevator deflection
- LED steady on (stall protection)

### Elevator Control Logic

```
If FSM_PROTECTION or FSM_OVERRIDE:
  command = -(AOA_excess * 2.0)
  if command < -15.0: saturate at -15.0
  
Apply slew-rate limiting: max 10°/sec
Final deflection: smooth ramp from current to command
```

---

## Data Formats

### UART Input (from aircraft system)
```
$AOA,S1=<val>,S2=<val>,S3=<val>,TS=<ms>*<checksum>
```

### CSV Test Data Format
```csv
idx,ts,mode,s1,s2,s3,airspeed
1,1000,-,5.38,5.32,5.01,45.2
2,1020,TAKEOFF,5.20,5.48,5.33,46.1
...
```

### Simulation Output
```csv
idx,ts,mode,s1,s2,s3,aoa,fsm_state,elevator
1,1000,-,5.38,5.32,5.01,5.24,0,0.00
2,1020,TAKEOFF,5.20,5.48,5.33,5.34,0,0.00
```

---

## Testing & Validation

### Automated CSV Test
```bash
# Full 200-row test flight
./sim_csv_safe components/aoa_controller/data/sim_input.csv > test_results.csv

# Check critical events (OVERRIDE state)
grep ",3," test_results.csv | head -5
```

Expected output shows FSM=3 (OVERRIDE) during landing phase when AOA spikes (rows 192-200).

### Manual Simulation
```bash
./simulator << EOF
$AOA,S1=5.0,S2=5.0,S3=5.0,TS=1000*
$AOA,S1=20.0,S2=20.0,S3=20.0,TS=1020*
$AOA,S1=25.0,S2=25.0,S3=25.0,TS=1040*
EOF
```

### ESP32 Serial Monitor
```bash
idf.py -p /dev/ttyUSB0 monitor

# Expected output:
# [FSM] Thresholds set: Aircraft_A/CRUISE → Low=3.0 High=22.0
# [UART] Connected at 115200 baud
# [HAL_INPUT] AoA parsed: S1=10.0 S2=10.0 S3=10.0 TS=1000
# [FSM] State: NORMAL
```

---

## Troubleshooting

### Docker Build Fails
```bash
# Full clean rebuild
docker run --rm -v $(pwd):/project -w /project espressif/idf idf.py fullclean
docker run --rm -v $(pwd):/project -w /project espressif/idf idf.py build
```

### Native Build (No Docker)
```bash
# Verify ESP-IDF is sourced
echo $IDF_PATH

# If empty:
source ~/esp/esp-idf/export.sh

# Then rebuild
idf.py fullclean
idf.py build
```

### Simulator Won't Compile
```bash
# Ensure all dependencies are available
gcc --version
which gcc

# Check include paths
ls -l components/aoa_controller/include/
```

### Serial Port Not Found
```bash
# List connected USB devices
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Check permissions
sudo usermod -a -G dialout $USER

# Use correct port in flash command
idf.py -p /dev/ttyUSB0 flash
```

---

## Performance Metrics

- **Build Time (Docker):** ~2-3 minutes (first run), ~30s (incremental)
- **Build Time (Native):** ~30s-1 min (with warm cache)
- **Firmware Size:** ~200 KB (program), ~50 KB (data)
- **Simulation Speed (CSV):** 200 rows in <100ms
- **Latency (ESP32):** FSM response <20ms (real-time capable)

---

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- Project thresholds: `components/aoa_controller/include/thresholds.h`
- FSM logic: `components/aoa_controller/src/fsm.c`
- Elevator control: `components/aoa_controller/src/elevator.c`

---

## License

[Your license here]

---

## Support

For issues or questions:
1. Check test flight data: `components/aoa_controller/data/sim_input.csv`
2. Run simulator: `./sim_csv_safe components/aoa_controller/data/sim_input.csv`
3. Review output: `cat sim_output.csv`
4. Check ESP32 logs: `idf.py monitor`