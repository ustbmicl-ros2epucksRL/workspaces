#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
platform_root="$(cd -- "$script_dir/.." && pwd -P)"
workspace_root_candidate="${MASAR_WORKSPACE_ROOT:-$(cd -- "$platform_root/.." && pwd -P)}"
workspace_root=""
runtime_dir="${MASAR_RUNTIME_DIR:-$platform_root/.runtime}"
logs_dir="$runtime_dir/logs"
secret_path="${MASAR_PLATFORM_SECRET_PATH:-$runtime_dir/agent.secret}"
compose_file="$platform_root/deploy/compose.yaml"
gateway_url="http://127.0.0.1:18090"

export MASAR_XAUTHORITY="${MASAR_XAUTHORITY:-${XAUTHORITY:-${HOME}/.Xauthority}}"
export MASAR_HOST_DISPLAY="${MASAR_HOST_DISPLAY:-${DISPLAY:-:0}}"
export MASAR_ROS2_DOMAIN_ID="${MASAR_ROS2_DOMAIN_ID:-42}"

readonly ROS1_IMAGE="talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59"
readonly ROS2_IMAGE="talent18/masar_ros2_bridge-w:v2@sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c"
readonly NODE_IMAGE="node:22-alpine@sha256:c610fcdfb1d5b4740dd70c284ed3cb16bb857e0f7166196e36a5501df7a3aa32"
readonly -a SERVICES=(
  api
  ros1-runtime
  ros2-runtime
  ros1-web
  ros2-web
  webots-guard
  display
  caddy
  public-tunnel
)
readonly -a PLATFORM_PORTS=(
  18090 18100 18101 18102 18111 18112 18121 18122 18131 18132 18133 18141
)

usage() {
  cat <<'EOF'
Usage: deploy/platform.sh COMMAND [SERVICE]

Commands:
  init            Check prerequisites, prepare runtime images and create the shared secret
  start           Build missing assets/overlays and start the complete platform
  stop            Stop the active Run, then stop all Compose-managed platform services
  status          Show Compose, gateway and active Run status
  logs [SERVICE]  Show recent logs for all services or one of the nine platform services
  doctor          Run read-only host and port prerequisite checks
EOF
}

log() {
  printf '[platform] %s\n' "$*"
}

print_access_hint() {
  local public_url
  log "browser: $gateway_url"
  log "remote SSH: LocalForward 18090 127.0.0.1:18090"
  public_url="$(public_tunnel_url)"
  if [[ -n "$public_url" ]]; then
    log "public URL (anonymous, expires within 60 minutes): $public_url"
  else
    warn "public tunnel URL is not available; check: make docker-logs SERVICE=public-tunnel"
  fi
}

public_tunnel_url() {
  compose logs --no-color public-tunnel 2>/dev/null \
    | sed -nE 's#.*(https://[[:alnum:]-]+\.free\.pinggy\.net).*#\1#p' \
    | tail -n 1 \
    || true
}

warn() {
  printf '[platform][warning] %s\n' "$*" >&2
}

die() {
  printf '[platform][error] %s\n' "$*" >&2
  exit 1
}

resolve_workspace_root() {
  [[ -d "$workspace_root_candidate" ]] \
    || die "workspace root is not a directory: $workspace_root_candidate"
  workspace_root="$(cd -- "$workspace_root_candidate" && pwd -P)"
  export MASAR_WORKSPACE_ROOT="$workspace_root"
}

resolve_workspace_root

compose() {
  docker compose -f "$compose_file" "$@"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command is unavailable: $1"
}

init_runtime_dirs() {
  mkdir -p "$runtime_dir" "$logs_dir"
  chmod 0700 "$runtime_dir"
}

compose_service_running() {
  local service="$1"
  local output
  output="$(compose ps --status running --services "$service" 2>/dev/null)" || return 1
  grep -Fq "$service" <<<"$output"
}

legacy_runtime_running() {
  local name="$1"
  [[ "$(docker inspect --format '{{.State.Running}}' "$name" 2>/dev/null || true)" == "true" ]]
}

expected_port_owner_exists() {
  local port="$1"
  case "$port" in
    18090) compose_service_running caddy ;;
    18100) compose_service_running api ;;
    18101) compose_service_running ros1-runtime || legacy_runtime_running masar_ros1_srv-g ;;
    18102) compose_service_running ros2-runtime || legacy_runtime_running masar_ros2_bridge-w ;;
    18111|18121) compose_service_running ros1-web ;;
    18112|18122) compose_service_running ros2-web ;;
    18131|18132|18133) compose_service_running display ;;
    18141) compose_service_running webots-guard ;;
    *) return 1 ;;
  esac
}

check_platform_ports() {
  local listeners port line
  listeners="$(ss -H -ltnp 2>/dev/null || true)"
  for port in "${PLATFORM_PORTS[@]}"; do
    line="$(grep -E ":${port}([[:space:]]|$)" <<<"$listeners" | head -n 1 || true)"
    [[ -z "$line" ]] && continue
    if expected_port_owner_exists "$port"; then
      log "port $port is owned by a managed or migratable platform service"
      continue
    fi
    die "unrelated process owns platform port $port: $line"
  done
}

doctor_impl() {
  local display_number
  log "workspace root: $workspace_root"
  for command in docker curl python3 ss sha256sum; do
    require_command "$command"
  done
  docker info >/dev/null 2>&1 || die "Docker daemon is unavailable"
  docker compose version >/dev/null 2>&1 || die "Docker Compose v2 is unavailable"
  [[ "$(uname -m)" == "x86_64" ]] || die "only linux/amd64 hosts are supported"

  for path in \
    "$workspace_root/masar_ws" \
    "$workspace_root/ROS2swarm_w" \
    "$workspace_root/masar-simulation-platform" \
    "$platform_root/backend" \
    "$platform_root/frontend" \
    "$platform_root/scenarios"; do
    [[ -d "$path" ]] || die "required workspace directory is missing: $path"
  done
  [[ -f "$compose_file" ]] || die "Compose file is missing: $compose_file"

  if [[ "${MASAR_PLATFORM_SKIP_DEVICE_CHECKS:-0}" != "1" ]]; then
    require_command nvidia-smi
    require_command xauth
    nvidia-smi -L >/dev/null 2>&1 || die "NVIDIA driver is unavailable"
    docker info --format '{{json .Runtimes}}' | grep -qi nvidia \
      || die "Docker NVIDIA runtime is unavailable"
    [[ -d /dev/dri ]] || die "/dev/dri is unavailable"
    [[ -r "$MASAR_XAUTHORITY" ]] || die "Xauthority file is not readable: $MASAR_XAUTHORITY"
    display_number="${MASAR_HOST_DISPLAY##*:}"
    display_number="${display_number%%.*}"
    [[ "$display_number" =~ ^[0-9]+$ ]] || die "unsupported DISPLAY value: $MASAR_HOST_DISPLAY"
    [[ -S "/tmp/.X11-unix/X${display_number}" ]] \
      || die "X11 socket is unavailable for DISPLAY=$MASAR_HOST_DISPLAY"
  fi

  check_platform_ports
  log "doctor checks passed"
}

prepare_runtime_images() {
  local label image warned=0
  while read -r label image; do
    if docker image inspect "$image" >/dev/null 2>&1; then
      log "$label image digest matched; skipping pull"
      continue
    fi
    if (( warned == 0 )); then
      warn "missing ROS runtime images are large; the first Docker Hub download can take a long time"
      warned=1
    fi
    log "pulling missing $label image"
    docker pull "$image" || die "failed to pull $label image"
  done <<EOF
ROS1 $ROS1_IMAGE
ROS2 $ROS2_IMAGE
EOF
}

ensure_secret() {
  local mode size
  if [[ ! -e "$secret_path" ]]; then
    umask 077
    mkdir -p "$(dirname -- "$secret_path")"
    dd if=/dev/urandom of="$secret_path" bs=32 count=1 status=none
    chmod 0600 "$secret_path"
    log "created shared Agent secret"
  fi
  [[ -f "$secret_path" ]] || die "Agent secret is not a regular file: $secret_path"
  mode="$(stat -c '%a' "$secret_path")"
  size="$(stat -c '%s' "$secret_path")"
  [[ "$mode" == "600" ]] || die "Agent secret must have mode 0600: $secret_path"
  [[ "$size" == "32" ]] || die "Agent secret must contain exactly 32 bytes: $secret_path"
}

normalize_secret_owner() {
  local owner parent name
  owner="$(stat -c '%u' "$secret_path")"
  [[ "$owner" == "65532" ]] && return 0
  if [[ "$(id -u)" == "0" ]]; then
    chown 65532:65532 "$secret_path"
    return 0
  fi
  parent="$(dirname -- "$secret_path")"
  name="$(basename -- "$secret_path")"
  docker run --rm \
    --entrypoint /bin/chown \
    --volume "$parent:/runtime" \
    "$ROS1_IMAGE" \
    65532:65532 "/runtime/$name" >/dev/null \
    || die "failed to make the Agent secret readable by the API container"
}

frontend_fingerprint() {
  (
    cd -- "$platform_root/frontend"
    {
      find src -type f -print0
      find . -maxdepth 1 -type f \
        ! -name '*.tsbuildinfo' \
        ! -name 'playwright.config.ts' \
        -print0
    } | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
  )
}

prepare_frontend_assets() {
  local fingerprint previous="" started="$SECONDS"
  local stamp="$runtime_dir/frontend.sha256"
  local build_log="$logs_dir/frontend.log"
  fingerprint="$(frontend_fingerprint)"
  [[ -f "$stamp" ]] && previous="$(<"$stamp")"
  if [[ "${MASAR_PLATFORM_FORCE_FRONTEND_BUILD:-0}" != "1" \
      && -f "$platform_root/frontend/dist/index.html" \
      && "$fingerprint" == "$previous" ]]; then
    log "frontend assets are current; skipping build"
    return 0
  fi

  mkdir -p "$platform_root/frontend/dist"
  log "building frontend assets in the pinned Node container"
  if ! docker run --rm --pull=missing \
      --user "$(id -u):$(id -g)" \
      --env HOME=/tmp \
      --env npm_config_cache=/tmp/npm-cache \
      --volume "$platform_root/frontend:/app" \
      --workdir /app \
      "$NODE_IMAGE" \
      sh -lc 'npm ci --no-audit --no-fund && npm run build' >"$build_log" 2>&1; then
    tail -n 30 "$build_log" >&2 || true
    die "frontend build failed; full log: $build_log"
  fi
  [[ -f "$platform_root/frontend/dist/index.html" ]] \
    || die "frontend build did not create dist/index.html; full log: $build_log"
  printf '%s\n' "$fingerprint" >"$stamp"
  log "frontend assets ready in $((SECONDS - started))s"
}

restore_generated_owner() {
  local path="$1"
  local image="$2"
  local owner
  [[ -e "$path" ]] || return 0
  owner="$(stat -c '%u:%g' "$path")"
  [[ "$owner" == "$(id -u):$(id -g)" ]] && return 0
  if [[ "$(id -u)" == "0" ]]; then
    chown -R "$(id -u):$(id -g)" "$path"
    return 0
  fi
  docker run --rm \
    --entrypoint /bin/chown \
    --volume "$path:/generated" \
    "$image" \
    -R "$(id -u):$(id -g)" /generated >/dev/null \
    || die "failed to restore generated directory ownership: $path"
}

build_ros1_overlay() {
  local build_log="$logs_dir/bootstrap-ros1.log"
  local started="$SECONDS"
  log "ROS1 overlay is missing; running catkin_make"
  if ! compose run --rm --no-deps --entrypoint /bin/bash ros1-runtime -lc \
      'source /opt/ros/noetic/setup.bash; cd /root/workspace/masar_ws; catkin_make' \
      >"$build_log" 2>&1; then
    tail -n 30 "$build_log" >&2 || true
    die "ROS1 overlay build failed; full log: $build_log"
  fi
  [[ -f "$workspace_root/masar_ws/devel/setup.bash" ]] \
    || die "ROS1 build completed without devel/setup.bash; full log: $build_log"
  restore_generated_owner "$workspace_root/masar_ws/build" "$ROS1_IMAGE"
  restore_generated_owner "$workspace_root/masar_ws/devel" "$ROS1_IMAGE"
  log "ROS1 overlay ready in $((SECONDS - started))s"
}

build_ros2_overlay() {
  local build_log="$logs_dir/bootstrap-ros2.log"
  local started="$SECONDS"
  log "ROS2 overlay is missing; running colcon build"
  if ! compose run --rm --no-deps --entrypoint /bin/bash ros2-runtime -lc \
      'source /opt/ros/humble/setup.bash; cd /root/workspace/ROS2swarm_w; colcon build' \
      >"$build_log" 2>&1; then
    tail -n 30 "$build_log" >&2 || true
    die "ROS2 overlay build failed; full log: $build_log"
  fi
  [[ -f "$workspace_root/ROS2swarm_w/install/setup.bash" ]] \
    || die "ROS2 build completed without install/setup.bash; full log: $build_log"
  restore_generated_owner "$workspace_root/ROS2swarm_w/build" "$ROS2_IMAGE"
  restore_generated_owner "$workspace_root/ROS2swarm_w/install" "$ROS2_IMAGE"
  restore_generated_owner "$workspace_root/ROS2swarm_w/log" "$ROS2_IMAGE"
  log "ROS2 overlay ready in $((SECONDS - started))s"
}

prepare_overlays() {
  if [[ -f "$workspace_root/masar_ws/devel/setup.bash" ]]; then
    log "ROS1 overlay already available; skipping build"
  else
    build_ros1_overlay
  fi
  if [[ -f "$workspace_root/ROS2swarm_w/install/setup.bash" ]]; then
    log "ROS2 overlay already available; skipping build"
  else
    build_ros2_overlay
  fi
}

compose_services_healthy() {
  local output
  output="$(compose ps --format json 2>/dev/null)" || return 1
  COMPOSE_PS_OUTPUT="$output" python3 - <<'PY'
import json
import os

expected = {
    "api",
    "ros1-runtime",
    "ros2-runtime",
    "ros1-web",
    "ros2-web",
    "webots-guard",
    "display",
    "caddy",
    "public-tunnel",
}
raw = os.environ.get("COMPOSE_PS_OUTPUT", "").strip()
if not raw:
    raise SystemExit(1)
try:
    parsed = json.loads(raw)
except json.JSONDecodeError:
    parsed = [json.loads(line) for line in raw.splitlines()]
if isinstance(parsed, dict):
    parsed = [parsed]
services = {item.get("Service"): item for item in parsed if isinstance(item, dict)}
ok = set(services) == expected
ok = ok and all(item.get("State") == "running" for item in services.values())
ok = ok and all(item.get("Health") == "healthy" for item in services.values())
raise SystemExit(0 if ok else 1)
PY
}

agent_health_matches() {
  local url="$1"
  local expected_agent="$2"
  local capability="$3"
  local payload
  payload="$(curl --noproxy '*' -fsS "$url" 2>/dev/null)" || return 1
  AGENT_HEALTH_PAYLOAD="$payload" EXPECTED_AGENT="$expected_agent" REQUIRED_CAPABILITY="$capability" \
    python3 - <<'PY'
import json
import os

try:
    data = json.loads(os.environ["AGENT_HEALTH_PAYLOAD"])["data"]
except (KeyError, TypeError, json.JSONDecodeError):
    raise SystemExit(1)
ok = data.get("agent") == os.environ["EXPECTED_AGENT"] and data.get("status") == "ok"
required = os.environ["REQUIRED_CAPABILITY"]
if required:
    ok = ok and required in data.get("capabilities", [])
raise SystemExit(0 if ok else 1)
PY
}

gateway_healthy() {
  curl --noproxy '*' -fsS "$gateway_url/api/v1/capabilities" >/dev/null 2>&1
}

platform_healthy() {
  compose_services_healthy \
    && gateway_healthy \
    && agent_health_matches http://127.0.0.1:18101/healthz ros1 "" \
    && agent_health_matches http://127.0.0.1:18102/healthz ros2 webots_preflight
}

current_run_payload() {
  curl --noproxy '*' -fsS "$gateway_url/api/v1/runs/current" 2>/dev/null
}

run_id_from_payload() {
  RUN_PAYLOAD="$1" python3 - <<'PY'
import json
import os

payload = json.loads(os.environ["RUN_PAYLOAD"])
run = payload.get("run")
if run is not None:
    print(run["run_id"])
PY
}

ensure_no_active_run() {
  local payload run_id
  if ! payload="$(current_run_payload)"; then
    warn "platform API is unavailable; no active Run could be queried"
    return 0
  fi
  run_id="$(run_id_from_payload "$payload")" || die "platform API returned invalid current Run data"
  [[ -z "$run_id" ]] || die "active Run must be stopped before platform migration: $run_id"
}

stop_active_run() {
  local payload run_id attempt
  if ! payload="$(current_run_payload)"; then
    warn "platform API is unavailable; continuing with whole-platform stop"
    return 0
  fi
  run_id="$(run_id_from_payload "$payload")" || die "platform API returned invalid current Run data"
  [[ -n "$run_id" ]] || return 0
  log "stopping active Run: $run_id"
  curl --noproxy '*' -fsS -X POST "$gateway_url/api/v1/runs/$run_id/stop" >/dev/null \
    || die "failed to request active Run stop: $run_id"
  for attempt in $(seq 1 120); do
    if payload="$(current_run_payload)" \
        && [[ -z "$(run_id_from_payload "$payload")" ]]; then
      log "active Run stopped"
      return 0
    fi
    sleep 0.5
  done
  die "active Run did not reach a terminal state: $run_id"
}

migrate_legacy_runtimes() {
  local name project
  for name in masar_ros1_srv-g masar_ros2_bridge-w; do
    docker inspect "$name" >/dev/null 2>&1 || continue
    project="$(docker inspect --format '{{ index .Config.Labels "com.docker.compose.project" }}' "$name" 2>/dev/null || true)"
    if [[ -n "$project" ]]; then
      continue
    fi
    log "migrating legacy container: $name"
    docker stop "$name" >/dev/null
    docker rm "$name" >/dev/null
  done
}

ensure_compose_images() {
  local image missing=0
  while IFS= read -r image; do
    [[ -n "$image" ]] || continue
    docker image inspect "$image" >/dev/null 2>&1 || missing=1
  done < <(compose config --images)
  if (( missing )); then
    log "building missing platform dependency images"
    compose build || die "failed to build platform dependency images"
  fi
}

wait_for_platform() {
  local attempt
  local attempts="${MASAR_PLATFORM_READY_ATTEMPTS:-300}"
  local interval="${MASAR_PLATFORM_READY_INTERVAL_SECONDS:-2}"
  for attempt in $(seq 1 "$attempts"); do
    if platform_healthy; then
      log "all nine services are healthy"
      return 0
    fi
    sleep "$interval"
  done
  compose ps >&2 || true
  die "platform did not become healthy within $((attempts * interval)) seconds"
}

command_init() {
  doctor_impl
  init_runtime_dirs
  prepare_runtime_images
  ensure_secret
  normalize_secret_owner
  log "initialization complete"
}

command_start() {
  doctor_impl
  init_runtime_dirs
  if platform_healthy; then
    prepare_frontend_assets
    log "platform is already healthy: $gateway_url"
    print_access_hint
    return 0
  fi

  ensure_no_active_run
  prepare_runtime_images
  ensure_secret
  normalize_secret_owner
  prepare_frontend_assets
  prepare_overlays
  ensure_compose_images

  compose down --remove-orphans >/dev/null 2>&1 || true
  migrate_legacy_runtimes
  log "starting nine Compose-managed services"
  compose up -d --remove-orphans
  wait_for_platform
  log "ready: $gateway_url"
  print_access_hint
}

command_stop() {
  stop_active_run
  compose down --remove-orphans
  log "platform stopped"
}

command_status() {
  compose ps || true
  if gateway_healthy; then
    log "gateway healthy: $gateway_url"
    print_access_hint
  else
    warn "gateway unavailable: $gateway_url"
  fi
  local payload run_id
  if payload="$(current_run_payload)"; then
    run_id="$(run_id_from_payload "$payload")" || die "platform API returned invalid current Run data"
    if [[ -n "$run_id" ]]; then
      log "active Run: $run_id"
    else
      log "active Run: none"
    fi
  else
    warn "active Run status unavailable"
  fi
}

service_is_valid() {
  local requested="$1"
  local service
  for service in "${SERVICES[@]}"; do
    [[ "$requested" == "$service" ]] && return 0
  done
  return 1
}

command_logs() {
  local service="${1:-}"
  [[ $# -le 1 ]] || die "logs accepts at most one service name"
  if [[ -n "$service" ]]; then
    service_is_valid "$service" || die "unknown platform service: $service"
    compose logs --tail=200 "$service"
  else
    compose logs --tail=200
  fi
}

command="${1:-}"
[[ -n "$command" ]] || { usage >&2; exit 2; }
shift
case "$command" in
  init) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; command_init ;;
  start) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; command_start ;;
  stop) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; command_stop ;;
  status) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; command_status ;;
  logs) command_logs "$@" ;;
  doctor) [[ $# -eq 0 ]] || { usage >&2; exit 2; }; doctor_impl ;;
  -h|--help|help) usage ;;
  *) usage >&2; exit 2 ;;
esac
