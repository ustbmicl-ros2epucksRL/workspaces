#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace_root=$(cd "${project_root}/.." && pwd)
image_name=${RESCUE_TASK_ALLOCATION_IMAGE:-rescue-task-allocation:local}
container_name=${RESCUE_TASK_ALLOCATION_CONTAINER:-rescue-task-allocation}
host_xauth=${HOST_XAUTHORITY:-${XAUTHORITY:-${HOME}/.Xauthority}}
display_value=${DISPLAY:-${MASAR_HOST_DISPLAY:-:0}}
mpl_backend=${MPLBACKEND:-Agg}

if [[ ! -r "${host_xauth}" ]]; then
  echo "Xauthority file is not readable: ${host_xauth}" >&2
  echo "Set HOST_XAUTHORITY to the host-side xauth file (for example ~/.Xauthority or /tmp/.docker.xauth)." >&2
  exit 1
fi

if [[ ! -d /tmp/.X11-unix ]]; then
  echo "X11 socket directory is unavailable: /tmp/.X11-unix" >&2
  exit 1
fi

docker build \
  --build-arg UID="$(id -u)" \
  --build-arg GID="$(id -g)" \
  -t "${image_name}" \
  -f "${project_root}/docker/Dockerfile" \
  "${project_root}"

if docker container inspect "${container_name}" >/dev/null 2>&1; then
  echo "container ${container_name} already exists; stop/remove it explicitly before rerunning" >&2
  exit 2
fi

exec docker run --rm -it \
  --name "${container_name}" \
  --gpus all \
  --ipc=host \
  --network=host \
  --group-add video \
  --device=/dev/dri \
  --read-only \
  --tmpfs /tmp \
  --tmpfs /home/research/.cache \
  -e DISPLAY="${display_value}" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e MPLBACKEND="${mpl_backend}" \
  -v "${workspace_root}:/root/workspace:ro" \
  -v "${project_root}:/root/workspace/rescue-task-allocation:rw" \
  -v "${host_xauth}:/tmp/.docker.xauth:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -w /root/workspace/rescue-task-allocation \
  "${image_name}" \
  bash
