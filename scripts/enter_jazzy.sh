#!/usr/bin/env bash
# Enter the persistent ROS2 Jazzy container for Revo3 teleop.
#
# Usage:
#   ./scripts/enter_jazzy.sh              # interactive shell (as admin user)
#   ./scripts/enter_jazzy.sh -- cmd ...    # run command (as admin)
#
# Environment variables:
#   ISAAC_ROS_WS     - host workspace dir (default: $HOME/workspaces/isaac_ros-dev)
#   BC_STARK_SDK_SRC - path to bc-stark-sdk site-packages on host (required for first run)
#   HTTP_PROXY       - proxy for pip install inside container (optional)

set -euo pipefail

CONTAINER_NAME="revo3_jazzy"
IMAGE="nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_69313772f0f318d6d4ecbf18d77b3dfe-amd64"
ISAAC_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
REVO_WS="/workspaces/isaac_ros-dev/Revo-Retargeting"
HOST_UID=$(id -u)
HOST_GID=$(id -g)
BC_STARK_SRC="${BC_STARK_SDK_SRC:-}"

# ── Kill stale processes holding the serial port ────────────────────────────
free_serial_port() {
    docker exec "${CONTAINER_NAME}" bash -c '
        # Method 1: scan /proc for any fd pointing to ttyUSB
        for pdir in /proc/*/fd; do
            pid=$(echo "$pdir" | cut -d/ -f3)
            [ "$pid" = "1" ] && continue
            [ "$pid" = "$$" ] && continue
            if ls -l "$pdir" 2>/dev/null | grep -q "ttyUSB"; then
                echo "[jazzy] Killing stale PID $pid holding serial port"
                kill -9 "$pid" 2>/dev/null || true
            fi
        done
        # Method 2: fuser (more reliable)
        fuser -k /dev/ttyUSB0 2>/dev/null || true
        fuser -k /dev/ttyUSB1 2>/dev/null || true
        sleep 0.5
    ' 2>/dev/null || true
}

# ── Ensure container is running ─────────────────────────────────────────────
ensure_container() {
    if docker ps --filter "name=${CONTAINER_NAME}" --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER_NAME}$"; then
        free_serial_port
        return 0
    fi

    docker rm "${CONTAINER_NAME}" 2>/dev/null || true

    echo "[jazzy] Creating container ${CONTAINER_NAME}..."
    docker run -d --name "${CONTAINER_NAME}" \
        --init \
        --privileged \
        --network host \
        --runtime nvidia \
        -v "${ISAAC_WS}:/workspaces/isaac_ros-dev" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /etc/localtime:/etc/localtime:ro \
        -e DISPLAY="${DISPLAY:-}" \
        -e TERM=xterm-256color \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e HOST_USER_UID="${HOST_UID}" \
        -e HOST_USER_GID="${HOST_GID}" \
        "${IMAGE}" \
        sleep infinity

    # ── Create admin user (same as isaac-ros activate) ─────────────────
    echo "[jazzy] Creating admin user (uid=${HOST_UID} gid=${HOST_GID})..."
    docker exec "${CONTAINER_NAME}" bash -c "
        export USERNAME=admin
        export HOST_USER_UID=${HOST_UID}
        export HOST_USER_GID=${HOST_GID}
        if [ ! \$(getent group \${HOST_USER_GID}) ]; then
            groupadd --gid \${HOST_USER_GID} \${USERNAME}
        fi
        if [ ! \$(getent passwd \${HOST_USER_UID}) ]; then
            useradd --no-log-init --uid \${HOST_USER_UID} --gid \${HOST_USER_GID} -m \${USERNAME}
        fi
        echo \${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/\${USERNAME}
        chmod 0440 /etc/sudoers.d/\${USERNAME}
        usermod -aG video,plugdev,sudo,dialout \${USERNAME} 2>/dev/null || true
    "

    # ── Copy bc-stark-sdk ──────────────────────────────────────────────
    if [ -n "${BC_STARK_SRC}" ] && [ -d "${BC_STARK_SRC}" ]; then
        echo "[jazzy] Copying bc-stark-sdk from ${BC_STARK_SRC}..."
        docker cp "${BC_STARK_SRC}/bc_stark_sdk" \
            "${CONTAINER_NAME}:/usr/local/lib/python3.12/dist-packages/bc_stark_sdk" 2>/dev/null || true
        docker cp "${BC_STARK_SRC}/bc_stark_sdk-1.4.5.dist-info" \
            "${CONTAINER_NAME}:/usr/local/lib/python3.12/dist-packages/bc_stark_sdk-1.4.5.dist-info" 2>/dev/null || true
        docker cp "${BC_STARK_SRC}/bc_stark_sdk.libs" \
            "${CONTAINER_NAME}:/usr/local/lib/python3.12/dist-packages/bc_stark_sdk.libs" 2>/dev/null || true
    fi

    # ── Install mujoco ─────────────────────────────────────────────────
    local proxy_opts=""
    [ -n "${HTTP_PROXY:-}" ] && proxy_opts="-e HTTP_PROXY=${HTTP_PROXY} -e HTTPS_PROXY=${HTTPS_PROXY:-${HTTP_PROXY}}"
    docker exec ${proxy_opts} "${CONTAINER_NAME}" \
        pip3 install --break-system-packages mujoco \
        -i https://pypi.tuna.tsinghua.edu.cn/simple/ 2>/dev/null || true

    # ── Install Pinocchio (for C++ retarget) ────────────────────────────
    docker exec "${CONTAINER_NAME}" bash -c '
        if ! dpkg -l ros-jazzy-pinocchio 2>/dev/null | grep -q "^ii"; then
            echo "[jazzy] Installing Pinocchio..."
            echo "deb [trusted=yes] https://mirrors.aliyun.com/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2-aliyun.list
            apt-get update -qq 2>/dev/null
            apt-get install -y -qq ros-jazzy-pinocchio 2>/dev/null
            rm /etc/apt/sources.list.d/ros2-aliyun.list 2>/dev/null || true
            echo "[jazzy] Pinocchio installed."
        fi
    ' 2>/dev/null || true

    echo "[jazzy] Container ready."
}

# ── Source ROS2 + workspace ─────────────────────────────────────────────────
SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source ${REVO_WS}/install/setup.bash"

# ── Main ────────────────────────────────────────────────────────────────────
ensure_container

if [[ $# -eq 0 ]]; then
    echo "[jazzy] Entering container as admin (${CONTAINER_NAME})..."
    exec docker exec -it -u admin -w "${REVO_WS}" \
        -e TERM=xterm-256color \
        -e DISPLAY="${DISPLAY:-}" \
        "${CONTAINER_NAME}" \
        bash -c "${SOURCE_CMD}; echo 'Ready.'; exec bash"
else
    exec docker exec -it -u admin -w "${REVO_WS}" \
        -e TERM=xterm-256color \
        -e DISPLAY="${DISPLAY:-}" \
        "${CONTAINER_NAME}" \
        bash -c "${SOURCE_CMD}; $*"
fi
