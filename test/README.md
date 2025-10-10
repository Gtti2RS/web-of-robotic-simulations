# WOS Test Suite

This directory contains test scripts for the Web of Simulators (WOS) platform, including latency tests, load tests, bandwidth tests, and CPU monitoring utilities.

## 📋 Table of Contents

- [Test Scripts Overview](#test-scripts-overview)
- [Python Latency Tests](#python-latency-tests)
- [Bash Test Scripts](#bash-test-scripts)
- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Test Results](#test-results)

---

## Test Scripts Overview

| Script | Type | Purpose | Duration |
|--------|------|---------|----------|
| `gazebo_test.py` | Python | WoT operations latency testing for Gazebo | 5-7 min |
| `coppeliasim_test.py` | Python | WoT operations latency testing for CoppeliaSim | 5-7 min |
| `fileUploader_test.py` | Python | File upload latency testing | 1-2 min |
| `load_test.sh` | Bash | Orchestrated load test with CPU monitoring | 135 sec |
| `ur10_test.sh` | Bash | UR10 robot pick-and-place test sequence | 30 sec |
| `bandwidth_test.sh` | Bash | Network bandwidth capture using tshark | configurable |
| `cpu_logger.bash` | Bash | CPU and memory monitoring using pidstat | configurable |
| `cleanup_test_uploads.sh` | Bash | Cleanup test upload files | instant |

---

## Python Latency Tests

### 1. Gazebo WoT Operations Test (`gazebo_test.py`)

Tests WoT operations on `gz_controller` (port 8080) and measures response times.

**Target Services:**
- gz_controller: `http://localhost:8080/gz_controller`
- ur10_server: `http://localhost:8083/ur10_server` (when UR10 loaded)

**Test Sequence (7 tests, ~145 operations):**
```
1. SimStats_Read          - Read simStats 10x with TD baseline (2s)
2. Visualize_Toggle       - Toggle visualize 20x, set to true (2s)
3. SimControl_RunPause    - Toggle run/pause 20x (2s)
4. SimControl_Speed       - Toggle faster/slower 20x (2s)
5. Box_SpawnDelete        - Spawn-Delete box.urdf 10 cycles (2s+2s)
6. Box_SetPose            - Load box, set 10 poses, delete (2s)
7. UR10_Full_Test         - Load UR10, test all actions (~35s)
                            - jointPositions 10x (with UR10 TD baseline)
                            - moveJoint 10x
                            - gripper open/close 5x
                            - moveToCartesian 10x
```

**Usage:**
```bash
# Run all tests
./gazebo_test.py

# The script will:
# - Measure baseline latency before each operation
# - Export results to: wot_latency_YYYYMMDD_HHMMSS.csv
# - Show normalized latency (operation time - baseline)
```

**Configuration:**
Edit `TEST_SUITE` in the script to enable/disable individual tests:
```python
TEST_SUITE = [
    {"name": "SimStats_Read", "enabled": True},
    {"name": "Visualize_Toggle", "enabled": True},
    # ... etc
]
```

**CSV Output Columns:**
- Test_Name
- Operation
- Iteration
- Baseline_Latency_ms
- Response_Time_ms
- Normalized_Latency_ms
- Status_Code
- Timestamp

---

### 2. CoppeliaSim WoT Operations Test (`coppeliasim_test.py`)

Tests WoT operations on `cs_controller` (port 8081).

**Target Services:**
- cs_controller: `http://localhost:8081/cs_controller`
- ur10_server: Same as Gazebo (port 8083/8084)

**Test Sequence (6 tests, ~125 operations):**
```
1. SimStats_Read          - Read simStats 10x with TD baseline (2s)
2. SimControl_RunPause    - Toggle run/pause 20x (2s)
3. SimControl_Speed       - Toggle faster/slower 20x (2s)
4. Box_SpawnDelete        - Spawn-Delete box.urdf 10 cycles (2s+2s)
5. Box_SetPose            - Load box, set 10 poses, delete (2s)
6. UR10_Full_Test         - Same as Gazebo (~35s)
```

**Usage:**
```bash
./coppeliasim_test.py
# Output: cs_latency_YYYYMMDD_HHMMSS.csv
```

**Note:** CoppeliaSim doesn't have a `visualize` property, so that test is excluded.

---

### 3. File Upload Latency Test (`fileUploader_test.py`)

Tests file upload performance to the fileUploader service (port 8082).

**Target:** `http://localhost:8082/upload`

**Test Sequence (4 tests, ~40 uploads):**
```
1. URDF_Upload     - Upload ur10_rg2.urdf to Gazebo (14 KB, 10x)
                     → Assets/urdf/uploaded/{modelName}/
2. SDF_Upload      - Upload diff_drive.sdf to Gazebo (10x)
                     → Assets/gazebo/worlds/uploaded/
3. Launch_Upload   - Upload diff_drive.launch.py (10x)
                     → Assets/gazebo/launch/uploaded/
4. TTT_Upload      - Upload IoTRemoteLab.ttt to CoppeliaSim (10x)
                     → Assets/coppeliasim/scenes/uploaded/
```

**Usage:**
```bash
./fileUploader_test.py
# Output: upload_latency_YYYYMMDD_HHMMSS.csv
# Automatic cleanup after completion
```

**Timeout Configuration:**
- Auto-calculated: 60s base + 10s per MB
- Small files (<1MB): 60s
- Large files (11MB .ttt): 170s

---

## Bash Test Scripts

### 1. Load Test (`load_test.sh`)

Orchestrated test sequence with CPU monitoring for performance testing.

**Test Timeline (Total: 135 seconds):**
```
Phase 1 [0-30s]:    Initial wait
Phase 2 [30-60s]:   Enable visualization + set camera pose (Gazebo only)
Phase 3 [60-90s]:   Load UR10 robot (ur10_rg2.urdf)
Phase 4 [90-120s]:  Execute UR10 test sequence
Phase 5 [120-135s]: Remove UR10 robot
```

**Usage:**
```bash
# Schedule a load test
./load_test.sh "HH:MM:SS" [simulator_type]

# Examples:
./load_test.sh "14:30:00" gazebo      # Gazebo test at 2:30 PM
./load_test.sh "22:00:00" coppelia    # CoppeliaSim test at 10:00 PM

# With custom host (for Docker)
WOS_HOST=10.157.150.3 ./load_test.sh "14:30:00" gazebo
```

**Configuration:**
- **Gazebo:**
  - gz_controller: Port 8080
  - ur10_server: Port 8083
  - URDF: `ur10_rg2.urdf`
- **CoppeliaSim:**
  - cs_controller: Port 8081
  - ur10_server: Port 8084
  - URDF: `ur10_rg2_coppelia.urdf`

**Output:**
- CPU logs: `./logs/cpu_YYYYMMDD_HHMMSS.csv`
- Includes per-process CPU, memory usage, and timestamps

---

### 2. UR10 Test Sequence (`ur10_test.sh`)

Performs a 2-cycle pick-and-place operation using WoT actions.

**Test Sequence (per cycle, 12 steps):**
1. Move to initial position
2. Open gripper
3. Move down to pickup
4. Close gripper (grasp)
5. Move up
6. Move to place position
7. Open gripper (release)
8. Move down
9. Close gripper
10. Move up
11. Move to return position
12. Open gripper

**Usage:**
```bash
# Default (port 8082)
./ur10_test.sh

# Specify port
./ur10_test.sh 8083  # Gazebo
./ur10_test.sh 8084  # CoppeliaSim

# Specify port and host
./ur10_test.sh 8083 10.157.150.3
```

---

### 3. Bandwidth Test (`bandwidth_test.sh`)

Captures network traffic using tshark for bandwidth analysis.

**Usage:**
```bash
# Schedule a capture
./bandwidth_test.sh "HH:MM:SS" <interface> "<filter>" [duration] [outdir]

# Example: Capture gz_controller traffic for 60s
./bandwidth_test.sh "14:30:00" lo "tcp port 8080" 60 ./captures

# Example: Capture ur10_server traffic
./bandwidth_test.sh "14:30:00" lo "tcp port 8083" 60 ./captures
```

**Output:**
- PCAP file: `./captures/capture_YYYYMMDD_HHMMSS.pcap`
- Can be analyzed with Wireshark

**Prerequisites:**
```bash
sudo apt install tshark
# May require root permissions
```

---

### 4. CPU Logger (`cpu_logger.bash`)

Monitors CPU and memory usage per process using pidstat.

**Usage:**
```bash
# Schedule CPU monitoring
./cpu_logger.bash "HH:MM:SS" [duration_s] [outdir]

# Example: Monitor for 300 seconds starting at 2:30 PM
./cpu_logger.bash "14:30:00" 300 ./logs
```

**Output CSV Columns:**
- time
- pid
- usr% (user CPU)
- system% (system CPU)
- guest%
- CPU% (total)
- VSZ(kB) (virtual memory)
- RSS(kB) (resident memory)
- mem%
- command

**Prerequisites:**
```bash
sudo apt install sysstat  # for pidstat
```

---

### 5. Cleanup Script (`cleanup_test_uploads.sh`)

Removes all test upload files created during testing.

**Usage:**
```bash
./cleanup_test_uploads.sh
```

**Cleans:**
- Gazebo uploaded models: `Assets/gazebo/models/uploaded/`
- Gazebo uploaded worlds: `Assets/gazebo/worlds/uploaded/`
- Gazebo uploaded launch files: `Assets/gazebo/launch/uploaded/`
- CoppeliaSim uploaded scenes: `Assets/coppeliasim/scenes/uploaded/`
- CoppeliaSim uploaded models: `Assets/coppeliasim/models/uploaded/`

---

## Quick Start

### Run All Latency Tests

```bash
# 1. Start Gazebo controller
cd ~/wos/main
node gz_controller.js &

# 2. Run Gazebo latency test
cd ~/wos/test
./gazebo_test.py

# 3. Start CoppeliaSim controller
cd ~/wos/main
node cs_controller.js &

# 4. Run CoppeliaSim latency test
cd ~/wos/test
./coppeliasim_test.py

# 5. Run file upload test
./fileUploader_test.py

# 6. Cleanup test files
./cleanup_test_uploads.sh
```

### Run Scheduled Load Test

```bash
# Schedule a load test with CPU monitoring
./load_test.sh "15:00:00" gazebo

# The script will:
# - Wait until 15:00:00
# - Start CPU logger
# - Execute 135-second test sequence
# - Save CPU logs to ./logs/
```

---

## Prerequisites

### Required Packages

**Python Tests:**
```bash
python3 --version  # Requires Python 3.6+
# Uses standard library only (http.client, json, csv, time)
```

**Bash Tests:**
```bash
# CPU monitoring
sudo apt install sysstat

# Bandwidth capture
sudo apt install tshark

# Other tools (usually pre-installed)
curl, date, sleep, timeout
```

### Required Services

Before running tests, ensure the appropriate services are running:

**For Gazebo tests:**
- gz_controller on port 8080
- fileUploader on port 8082
- Gazebo simulation running

**For CoppeliaSim tests:**
- cs_controller on port 8081
- fileUploader on port 8082
- CoppeliaSim running

**For UR10 tests:**
- UR10 model loaded in simulator
- ur10_server on port 8083 (Gazebo) or 8084 (CoppeliaSim)

---

## Test Results

### Output Files

All test scripts generate timestamped output files:

**Python Tests:**
- `wot_latency_YYYYMMDD_HHMMSS.csv` - Gazebo operations
- `cs_latency_YYYYMMDD_HHMMSS.csv` - CoppeliaSim operations
- `upload_latency_YYYYMMDD_HHMMSS.csv` - File uploads

**Bash Tests:**
- `./logs/cpu_YYYYMMDD_HHMMSS.csv` - CPU/memory monitoring
- `./captures/capture_YYYYMMDD_HHMMSS.pcap` - Network traffic

### CSV Analysis

Use standard tools to analyze CSV results:

```bash
# View results
cat wot_latency_*.csv | column -t -s,

# Calculate average latency
awk -F, 'NR>1 {sum+=$6; count++} END {print sum/count}' wot_latency_*.csv

# Find slowest operations
sort -t, -k6 -n wot_latency_*.csv | tail -10

# Import to spreadsheet for visualization
# Excel, Google Sheets, pandas, etc.
```

---

## Notes

- **Baseline Normalization**: Python tests measure a baseline (TD read) before each operation to isolate operation latency from network/server overhead
- **Fixed Phase Durations**: Load test phases have fixed durations regardless of operation response times
- **Docker Compatibility**: Use `WOS_HOST` environment variable when running in Docker containers
- **Timeout Protection**: All operations have timeouts to prevent hanging
- **Automatic Cleanup**: File upload test automatically cleans up after completion

---

## Troubleshooting

**"Connection refused" errors:**
- Ensure the target service (gz_controller, cs_controller) is running
- Check that the simulator is running
- Verify port numbers match your configuration

**"Permission denied" for bandwidth_test.sh:**
- tshark may require root permissions
- Run with sudo or add user to wireshark group

**"pidstat not found" for cpu_logger.bash:**
```bash
sudo apt install sysstat
```

**Python tests hang:**
- Check if simulator is frozen
- Increase timeout values in the script
- Verify operations complete successfully in simulator

---

## License

Part of the Web of Simulators (WOS) project.

