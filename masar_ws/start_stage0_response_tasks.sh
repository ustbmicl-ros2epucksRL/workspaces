#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/start_stage0_response_tasks.sh" "$@"
