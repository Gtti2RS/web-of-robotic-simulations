#!/bin/bash
# Schedule a tshark capture for a fixed duration
# Usage:
#   ./scheduled_capture.sh "HH:MM:SS" <interface> "<capture filter>" [duration_s] [outdir]
# Example:
#   ./scheduled_capture.sh "22:30:00" lo "tcp port 8081" 30 ~/captures
# dependency: sudo apt install wireshark tshark, root permission

set -euo pipefail

START_TIME="${1:-}"
INTERFACE="${2:-}"
FILTER="${3:-}"
DURATION="${4:-30}"
OUTDIR="${5:-./captures}"

if [[ -z "$START_TIME" || -z "$INTERFACE" || -z "$FILTER" ]]; then
  echo "Usage: $0 \"HH:MM:SS\" <interface> '<capture filter>' [duration_s] [outdir]"
  exit 1
fi

# tools check
command -v tshark >/dev/null || { echo "tshark not found. Install Wireshark or tshark."; exit 1; }

# absolute output dir
OUTDIR="$(realpath -m "$OUTDIR")"
mkdir -p "$OUTDIR"

# compute start epoch (today, or tomorrow if time has passed)
NOW_EPOCH=$(date +%s)
TARGET_EPOCH=$(date -d "$(date +%Y-%m-%d) $START_TIME" +%s) || { echo "Invalid start time: $START_TIME"; exit 1; }
if (( TARGET_EPOCH < NOW_EPOCH )); then TARGET_EPOCH=$(( TARGET_EPOCH + 86400 )); fi
WAIT=$(( TARGET_EPOCH - NOW_EPOCH ))

STAMP=$(date -d "@$TARGET_EPOCH" +%Y%m%d_%H%M%S)
OUT_FILE="$OUTDIR/capture_${STAMP}_${INTERFACE}.pcapng"
TMP_FILE="/tmp/$(basename "$OUT_FILE")"
REAL_USER="$(logname 2>/dev/null || echo "$USER")"

echo "Now:        $(date)"
echo "Start at:   $START_TIME  (epoch $TARGET_EPOCH)"
echo "Wait secs:  $WAIT"
echo "Interface:  $INTERFACE"
echo "Filter:     $FILTER"
echo "Duration:   ${DURATION}s"
echo "Output dir: $OUTDIR"
echo "Output:     $OUT_FILE"

sleep "$WAIT"

echo ">>> Starting capture at $(date)"
# capture as root to a safe temp path, then move and chown
sudo tshark -i "$INTERFACE" -f "$FILTER" -a duration:"$DURATION" -w "$TMP_FILE"
echo ">>> Moving capture to $OUT_FILE"
sudo mv "$TMP_FILE" "$OUT_FILE"
sudo chown "$REAL_USER:$REAL_USER" "$OUT_FILE"
echo ">>> Done at $(date). Saved: $OUT_FILE"
