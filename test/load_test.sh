#!/bin/bash
# Load Test Script for WOS Server
# This script orchestrates a timed test sequence with CPU logging
#
# Usage:
#   ./load_test.sh "HH:MM:SS" [simulator_type]
# Example:
#   ./load_test.sh "22:30:00" coppelia
#   ./load_test.sh "22:30:00" gazebo
#
# Environment Variables:
#   WOS_HOST - Override the hostname/IP for WOS controllers (useful for Docker)
#   Example: WOS_HOST=10.157.150.3 ./load_test.sh "22:30:00" gazebo

set -euo pipefail

START_TIME="${1:-}"
SIMULATOR_TYPE="${2:-coppelia}"

if [[ -z "$START_TIME" ]]; then
  echo "Usage: $0 \"HH:MM:SS\" [simulator_type]"
  echo "Example: $0 \"22:30:00\" coppelia"
  echo "         $0 \"22:30:00\" gazebo"
  echo ""
  echo "Supported simulator types:"
  echo "  - coppelia (default)"
  echo "  - gazebo"
  echo ""
  echo "Environment Variables:"
  echo "  WOS_HOST - Override hostname (e.g., WOS_HOST=10.157.150.3)"
  exit 1
fi

# Configure based on simulator type
case "${SIMULATOR_TYPE,,}" in
  coppelia|coppeliasim|cs)
    URL="10.157.150.3"
    PORT="8081"              # cs_controller port
    UR10_PORT="8084"         # ur10_server port for CoppeliaSim
    SIM_CONTROLLER="cs_controller"
    URDF_FILE="ur10_rg2_coppelia.urdf"
    ;;
  gazebo|gz)
    URL="10.157.150.3"
    PORT="8080"              # gz_controller port
    UR10_PORT="8083"         # ur10_server port for Gazebo
    SIM_CONTROLLER="gz_controller"
    URDF_FILE="ur10_rg2.urdf"
    ;;
  *)
    echo "ERROR: Unknown simulator type: $SIMULATOR_TYPE"
    echo "Supported types: coppelia, gazebo"
    exit 1
    ;;
esac

# Build base URL
BASE_URL="http://${URL}:${PORT}/${SIM_CONTROLLER}"
ACTIONS_URL="${BASE_URL}/actions"

# Test configuration
TOTAL_DURATION=135  # Total test duration in seconds
LOG_DIR="./logs"

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║              WOS Load Test Configuration                      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo "Start time:       $START_TIME"
echo "Simulator type:   $SIMULATOR_TYPE"
echo "Controller:       $SIM_CONTROLLER (port: $PORT)"
echo "UR10 Server:      ur10_server (port: $UR10_PORT)"
echo "URDF File:        $URDF_FILE"
echo "Base URL:         $BASE_URL"
echo "Actions URL:      $ACTIONS_URL"
echo "Total duration:   ${TOTAL_DURATION}s"
echo "Log directory:    $LOG_DIR"
echo ""
echo "Test sequence:"
echo "  [0s]    Start CPU logger"
case "${SIMULATOR_TYPE,,}" in
  gazebo|gz)
    echo "  [30s]   Enable visualization (30s)"
    ;;
  coppelia|coppeliasim|cs)
    echo "  [30s]   (Skip enabling visualization for CoppeliaSim)"
    ;;
esac
echo "  [60s]   Load $URDF_FILE (30s)"
echo "  [90s]   Execute ur10_test.sh (30s)"
echo "  [120s]  Remove ur10 (15s)"
echo "  [135s]  Exit"
echo ""

# Start CPU logger in background
echo ">>> Starting CPU logger at $START_TIME for ${TOTAL_DURATION}s..."
./cpu_logger.bash "$START_TIME" "$TOTAL_DURATION" "$LOG_DIR" &
CPU_LOGGER_PID=$!

# Wait for the scheduled start time (cpu_logger handles this internally)
# We need to calculate the same wait time
NOW_EPOCH=$(date +%s)
TARGET_EPOCH=$(date -d "$(date +%Y-%m-%d) $START_TIME" +%s) || {
  echo "Invalid time format: $START_TIME"; exit 1;
}
if (( TARGET_EPOCH < NOW_EPOCH )); then
  TARGET_EPOCH=$(( TARGET_EPOCH + 86400 ))
fi
WAIT=$(( TARGET_EPOCH - NOW_EPOCH ))

echo "Waiting ${WAIT}s until $START_TIME..."
sleep "$WAIT"

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                  Starting Load Test Sequence                  ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Helper function to send HTTP requests
send_request() {
    local endpoint="$1"
    local data="$2"
    local description="$3"
    
    echo "[$(date +%H:%M:%S)] $description"
    
    # Temporarily disable exit on error to capture curl failures
    set +e
    response=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/json" \
        -d "$data" \
        "$endpoint" 2>&1)
    curl_exit=$?
    set -e
    
    # Extract HTTP code (last line) and body (everything else)
    http_code=$(echo "$response" | tail -n1)
    body=$(echo "$response" | sed '$d')
    
    # Check if curl failed
    if [[ $curl_exit -ne 0 ]]; then
        echo "Response: CURL ERROR (exit code: $curl_exit, connection failed)"
        [[ -n "$body" ]] && echo "Details: $body"
    # Check if HTTP code indicates success (2xx or 3xx)
    elif [[ "$http_code" =~ ^[23][0-9][0-9]$ ]]; then
        echo "Response (HTTP $http_code): ${body:-Success}"
    else
        echo "Response: ERROR (HTTP $http_code): ${body}"
    fi
    echo ""
}

# Phase 1: Wait 30s (0-30s)
echo "═══ Phase 1: Initial Wait (0-30s) ═══"
sleep 60

# # Phase 2: Enable visualization (30-60s)
# echo "═══ Phase 2: Enable Visualization (30-60s) ═══"
# phase2_start=$(date +%s)
# case "${SIMULATOR_TYPE,,}" in
#   coppelia|coppeliasim|cs)
#     echo "[$(date +%H:%M:%S)] Skipping enabling visualization for CoppeliaSim"
#     ;;
#   gazebo|gz)
#     echo "[$(date +%H:%M:%S)] Writing visualize property to true for Gazebo"
#     set +e  # Temporarily disable exit on error
#     http_code=$(curl -s -w "%{http_code}" -o /dev/null -X PUT \
#         -H "Content-Type: application/json" \
#         -d 'true' \
#         "${BASE_URL}/properties/visualize")
#     curl_exit=$?
#     set -e  # Re-enable exit on error
    
#     if [[ $curl_exit -ne 0 ]]; then
#         echo "Response: CURL ERROR (exit code: $curl_exit, connection failed)"
#     elif [[ "$http_code" =~ ^(200|204)$ ]]; then
#         echo "Response: Success (HTTP $http_code)"
#     else
#         echo "Response: ERROR (HTTP $http_code)"
#     fi
    
#     # Set camera pose
#     sleep 1
#     echo "set camera pose"
#     send_request \
#         "${ACTIONS_URL}/manageModel" \
#         '{
#             "mode": "setPose",
#             "modelName": "wot_camera",
#             "position": {
#                 "x": -1.8,
#                 "y": 0.0,
#                 "z": 0
#             },
#             "orientation": {
#                 "roll": 0,
#                 "pitch": 0,
#                 "yaw": 0
#             }
#         }' \
#         "Setting camera pose"
#     ;;
# esac
# # Ensure phase lasts exactly 30s
# phase2_elapsed=$(($(date +%s) - phase2_start))
# phase2_remaining=$((30 - phase2_elapsed))
# if [[ $phase2_remaining -gt 0 ]]; then
#     echo "Waiting ${phase2_remaining}s to complete 30s phase..."
#     sleep $phase2_remaining
# fi
# echo ""

# Phase 3: Load ur10_rg2.urdf (60-90s)
echo "═══ Phase 3: Load UR10_RG2 URDF (60-90s) ═══"
phase3_start=$(date +%s)
send_request \
    "${ACTIONS_URL}/manageModel" \
    "{
        \"mode\": \"load\",
        \"modelName\": \"ur10_rg2\",
        \"fileName\": \"${URDF_FILE}\"
    }" \
    "Loading ${URDF_FILE}"
# Ensure phase lasts exactly 30s
phase3_elapsed=$(($(date +%s) - phase3_start))
phase3_remaining=$((30 - phase3_elapsed))
if [[ $phase3_remaining -gt 0 ]]; then
    echo "Waiting ${phase3_remaining}s to complete 30s phase..."
    sleep $phase3_remaining
fi

# Phase 4: Execute ur10_test.sh (90-120s)
echo "═══ Phase 4: Execute UR10 Test (90-120s) ═══"
phase4_start=$(date +%s)
if [[ -f "./ur10_test.sh" ]]; then
    echo "[$(date +%H:%M:%S)] Running ur10_test.sh with port $UR10_PORT..."
    timeout 30 bash ./ur10_test.sh "$UR10_PORT" || echo "Test execution completed or timed out"
else
    echo "WARNING: ./ur10_test.sh not found"
fi
# Ensure phase lasts exactly 30s
phase4_elapsed=$(($(date +%s) - phase4_start))
phase4_remaining=$((30 - phase4_elapsed))
if [[ $phase4_remaining -gt 0 ]]; then
    echo "Waiting ${phase4_remaining}s to complete 30s phase..."
    sleep $phase4_remaining
fi
echo ""

# Phase 5: Remove ur10 (120-135s)
echo "═══ Phase 5: Remove UR10 (120-135s) ═══"
phase5_start=$(date +%s)
send_request \
    "${ACTIONS_URL}/manageModel" \
    '{
        "mode": "remove",
        "modelName": "ur10_rg2"
    }' \
    "Removing ur10_rg2"
# Ensure phase lasts exactly 30s
phase5_elapsed=$(($(date +%s) - phase5_start))
phase5_remaining=$((30 - phase5_elapsed))
if [[ $phase5_remaining -gt 0 ]]; then
    echo "Waiting ${phase5_remaining}s to complete 30s phase..."
    sleep $phase5_remaining
fi

# Test complete
echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                  Load Test Completed                          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo "Test finished at: $(date)"
echo "CPU logger PID: $CPU_LOGGER_PID"
echo ""
echo "Waiting for CPU logger to finish..."
wait $CPU_LOGGER_PID 2>/dev/null || true

echo "All processes completed. Check logs in: $LOG_DIR"

