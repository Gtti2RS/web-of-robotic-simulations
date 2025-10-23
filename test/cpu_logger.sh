#!/bin/bash
# Schedule a per-process CPU and memory logger.
# Usage:
#   ./cpu_logger.sh "HH:MM:SS" [duration_s] [outdir]
# Example:
#   ./cpu_logger.sh "22:30:00" 60 /project-root/test/logs
# dependency: sudo apt install sysstat

set -euo pipefail

START_TIME="${1:-}"
DURATION="${2:-60}"
OUTDIR="${3:-./cpu_logs}"
INTERVAL=1

if [[ -z "$START_TIME" ]]; then
  echo "Usage: $0 \"HH:MM:SS\" [duration_s] [outdir]"
  exit 1
fi

# Make sure pidstat is available
command -v pidstat >/dev/null || {
  echo "pidstat not found. Install with: sudo apt install sysstat"
  exit 1
}

# Create absolute path
OUTDIR="$(realpath -m "$OUTDIR")"
mkdir -p "$OUTDIR"

# Compute start time (today or tomorrow)
NOW_EPOCH=$(date +%s)
TARGET_EPOCH=$(date -d "$(date +%Y-%m-%d) $START_TIME" +%s) || {
  echo "Invalid time format: $START_TIME"; exit 1;
}
if (( TARGET_EPOCH < NOW_EPOCH )); then
  TARGET_EPOCH=$(( TARGET_EPOCH + 86400 ))
fi
WAIT=$(( TARGET_EPOCH - NOW_EPOCH ))

STAMP=$(date -d "@$TARGET_EPOCH" +%Y%m%d_%H%M%S)
OUTFILE="$OUTDIR/cpu_${STAMP}.csv"

echo "Current time: $(date)"
echo "Will start at: $START_TIME (in $WAIT seconds)"
echo "Duration: $DURATION s"
echo "Output: $OUTFILE"

sleep "$WAIT"

echo ">>> Starting CPU logging at $(date)"

# Write header
echo "time,uid,pid,usr%,system%,guest%,wait%,CPU%,cpu_core,minflt/s,majflt/s,VSZ(kB),RSS(kB),mem%,command" > "$OUTFILE"

# Run pidstat for duration
timeout --preserve-status "$DURATION" \
  pidstat -u -r -h -p ALL "$INTERVAL" |
  awk 'NR>1 && $1 ~ /^[0-9]/ {
        time=$1; uid=$2; pid=$3; usr=$4; sys=$5; guest=$6; wait=$7; cpu=$8; cpu_core=$9;
        minflt=$10; majflt=$11; vsz=$12; rss=$13; mem=$14;
        cmd=$15; for (i=16;i<=NF;i++) cmd=cmd" "$i;
        print time","uid","pid","usr","sys","guest","wait","cpu","cpu_core","minflt","majflt","vsz","rss","mem","cmd; fflush();
      }' >> "$OUTFILE"

echo ">>> Done at $(date)"
echo "Saved to: $OUTFILE"
