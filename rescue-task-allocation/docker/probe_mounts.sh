#!/usr/bin/env bash
set -euo pipefail

research_root=/root/workspace/rescue-task-allocation
masar_root=/root/workspace/masar_ws
ros2swarm_root=/root/workspace/ROS2swarm_w

probe_file="${research_root}/.mount-probe.$$"
trap 'rm -f "${probe_file}"' EXIT

printf 'research mother project writable: '
if touch "${probe_file}"; then
  echo OK
else
  echo FAILED
  exit 1
fi

for protected_root in "${masar_root}" "${ros2swarm_root}"; do
  printf '%s read-only: ' "${protected_root}"
  protected_probe="${protected_root}/.write-probe.$$"
  if touch "${protected_probe}" 2>/dev/null; then
    rm -f "${protected_probe}"
    echo FAILED
    exit 1
  fi
  echo OK
done

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
else
  echo "nvidia-smi unavailable; GPU smoke test is not confirmed"
fi

echo "mount/GPU policy probe passed"
