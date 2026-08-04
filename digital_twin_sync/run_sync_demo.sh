#!/usr/bin/env bash
set -Eeuo pipefail

# ============================================================
# Jackson Digital Twin synchronized demonstration
# - Does not modify YAML files
# - Checks physical lidar before navigation
# - Applies temporary virtual speed parameters
# ============================================================

: "${JETSON_IP:?Set JETSON_IP to the Jetson IPv4 address}"
JETSON_USER="${JETSON_USER:-$USER}"

GOAL_X="${1:-1.80}"
GOAL_Y="${2:--0.04}"
GOAL_YAW="${3:-0.0}"

VIRTUAL_MAX_X="0.15"
VIRTUAL_MAX_THETA="0.60"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAIN_SCRIPT="$SCRIPT_DIR/send_goal_both.sh"
export PHYSICAL_HOST="${JETSON_USER}@${JETSON_IP}"

RED=$'\033[1;31m'
GREEN=$'\033[1;32m'
YELLOW=$'\033[1;33m'
BLUE=$'\033[1;34m'
RESET=$'\033[0m'

alert_error()
{
    local message="$1"

    echo
    echo "${RED}ERROR: ${message}${RESET}"

    if command -v notify-send >/dev/null 2>&1; then
        notify-send \
          --urgency=critical \
          "Jackson: comprobación fallida" \
          "$message" || true
    fi
}

alert_ok()
{
    local message="$1"
    echo "${GREEN}OK: ${message}${RESET}"
}

source_virtual()
{
    export ROS_DOMAIN_ID=10
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset CYCLONEDDS_URI || true

    set +u
    source /opt/ros/humble/setup.bash
    set -u
}

check_virtual_nav2()
{
    echo
    echo "${BLUE}===== NAV2 VIRTUAL =====${RESET}"

    source_virtual

    if ! timeout 6s ros2 action list 2>/dev/null |
         grep -qx '/navigate_to_pose'; then
        alert_error \
          "El servidor /navigate_to_pose virtual no está disponible."
        return 1
    fi

    alert_ok "/navigate_to_pose virtual disponible"
}

check_physical_nav2()
{
    echo
    echo "${BLUE}===== NAV2 FÍSICO =====${RESET}"

    if ! ssh \
      -o BatchMode=yes \
      -o ConnectTimeout=5 \
      "${JETSON_USER}@${JETSON_IP}" '
        export ROS_DOMAIN_ID=0
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI || true

        set +u
        source /opt/ros/humble/setup.bash
        source ~/jackson_dt_ws/install/setup.bash
        set -u

        timeout 6s ros2 action list 2>/dev/null |
        grep -qx "/navigate_to_pose"
      '; then
        alert_error \
          "El servidor /navigate_to_pose físico no está disponible."
        return 1
    fi

    alert_ok "/navigate_to_pose físico disponible"
}

check_physical_lidar()
{
    echo
    echo "${BLUE}===== LIDAR FÍSICO =====${RESET}"
    echo "Esperando un mensaje LaserScan válido..."

    if ! ssh \
      -o BatchMode=yes \
      -o ConnectTimeout=5 \
      "${JETSON_USER}@${JETSON_IP}" '
        export ROS_DOMAIN_ID=0
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI || true

        set +u
        source /opt/ros/humble/setup.bash
        source ~/jackson_dt_ws/install/setup.bash
        set -u

        # Debe existir al menos un publicador.
        ros2 topic info /scan 2>/dev/null |
        grep -Eq "Publisher count: [1-9][0-9]*" || exit 10

        # No basta con que exista el tópico:
        # debe recibirse un mensaje LaserScan real.
        timeout 10s ros2 topic echo \
          /scan \
          sensor_msgs/msg/LaserScan \
          --once \
          --qos-reliability best_effort \
          >/dev/null 2>&1 || exit 11
      '; then
        alert_error \
          "El lidar no está publicando mensajes válidos en /scan. Revisa su conexión antes de mover el robot."
        return 1
    fi

    alert_ok "Lidar físico publicando correctamente en /scan"
}

set_virtual_demo_speed()
{
    echo
    echo "${BLUE}===== AJUSTE TEMPORAL VIRTUAL =====${RESET}"

    source_virtual

    ros2 param set \
      /controller_server \
      FollowPath.max_vel_x \
      "$VIRTUAL_MAX_X" >/dev/null

    ros2 param set \
      /controller_server \
      FollowPath.max_vel_theta \
      "$VIRTUAL_MAX_THETA" >/dev/null

    ros2 param set \
      /velocity_smoother \
      max_velocity \
      "[$VIRTUAL_MAX_X, 0.0, $VIRTUAL_MAX_THETA]" >/dev/null

    ros2 param set \
      /velocity_smoother \
      min_velocity \
      "[-$VIRTUAL_MAX_X, 0.0, -$VIRTUAL_MAX_THETA]" >/dev/null

    CURRENT_X=$(
      ros2 param get \
        /controller_server \
        FollowPath.max_vel_x |
      awk '{print $NF}'
    )

    CURRENT_THETA=$(
      ros2 param get \
        /controller_server \
        FollowPath.max_vel_theta |
      awk '{print $NF}'
    )

    echo "max_vel_x:     $CURRENT_X m/s"
    echo "max_vel_theta: $CURRENT_THETA rad/s"

    alert_ok "Parámetros temporales aplicados al virtual"
}

main()
{
    echo
    echo "${YELLOW}========================================${RESET}"
    echo "${YELLOW} JACKSON — DEMOSTRACIÓN SINCRONIZADA${RESET}"
    echo "${YELLOW}========================================${RESET}"
    echo
    echo "Meta:"
    echo "  x:   $GOAL_X"
    echo "  y:   $GOAL_Y"
    echo "  yaw: $GOAL_YAW"

    if [[ ! -x "$MAIN_SCRIPT" ]]; then
        alert_error \
          "No se encontró el ejecutable $MAIN_SCRIPT"
        exit 1
    fi

    check_virtual_nav2
    check_physical_nav2
    check_physical_lidar
    set_virtual_demo_speed

    echo
    echo "${BLUE}===== EJECUCIÓN =====${RESET}"

    "$MAIN_SCRIPT" \
      "$GOAL_X" \
      "$GOAL_Y" \
      "$GOAL_YAW"
}

main "$@"
