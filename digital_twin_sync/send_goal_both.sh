#!/usr/bin/env bash
set -Eeuo pipefail

: "${PHYSICAL_HOST:?Set PHYSICAL_HOST as user@host}"
VIRTUAL_DOMAIN="${VIRTUAL_DOMAIN:-10}"

usage() {
  echo "Uso:"
  echo "  $0 check"
  echo "  $0 X Y [YAW_RAD]"
  echo
  echo "Ejemplo:"
  echo "  $0 1.80 -0.04 0.0"
}

virtual_setup() {
  export ROS_DOMAIN_ID="$VIRTUAL_DOMAIN"
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset CYCLONEDDS_URI || true

  set +u
  source /opt/ros/humble/setup.bash
  set -u
}

check_virtual() {
  virtual_setup
  ros2 action list 2>/dev/null |
    grep -qx '/navigate_to_pose'
}

check_physical() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 \
    "$PHYSICAL_HOST" '
      export ROS_DOMAIN_ID=0
      export ROS_LOCALHOST_ONLY=0
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      unset CYCLONEDDS_URI || true

      source /opt/ros/humble/setup.bash
      source ~/jackson_dt_ws/install/setup.bash

      ros2 action list 2>/dev/null |
        grep -qx "/navigate_to_pose"
    '
}

if [[ $# -eq 1 && "$1" == "check" ]]; then
  echo "===== JACKSON VIRTUAL ====="

  if check_virtual; then
    echo "OK: /navigate_to_pose disponible en dominio 10"
  else
    echo "ERROR: Nav2 virtual no está disponible"
    exit 1
  fi

  echo
  echo "===== JACKSON FÍSICO ====="

  if check_physical; then
    echo "OK: /navigate_to_pose disponible en dominio 0"
  else
    echo "ERROR: Nav2 físico o SSH no están disponibles"
    exit 1
  fi

  echo
  echo "OK: ambos sistemas están listos"
  exit 0
fi

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
  exit 2
fi

X="$1"
Y="$2"
YAW="${3:-0.0}"

read -r QZ QW < <(
  python3 - "$YAW" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

echo "===== COMPROBACIÓN PREVIA ====="

if ! check_virtual; then
  echo "ERROR: Nav2 virtual no ofrece /navigate_to_pose"
  exit 1
fi
echo "OK: Jackson virtual listo"

if ! check_physical; then
  echo "ERROR: Nav2 físico no ofrece /navigate_to_pose"
  exit 1
fi
echo "OK: Jackson físico listo"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$HOME/jackson_dt_sync/logs/$STAMP"
mkdir -p "$LOG_DIR"

echo
echo "===== META COMÚN ====="
echo "x=$X"
echo "y=$Y"
echo "yaw=$YAW rad"
echo "qz=$QZ"
echo "qw=$QW"
echo "Logs: $LOG_DIR"
echo

# ------------------------------------------------------------
# META VIRTUAL: dominio ROS 10, ejecución local en la Legion
# ------------------------------------------------------------
(
  virtual_setup

  ros2 action send_goal \
    /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{
      pose: {
        header: {
          frame_id: map
        },
        pose: {
          position: {
            x: ${X},
            y: ${Y},
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: ${QZ},
            w: ${QW}
          }
        }
      }
    }" \
    --feedback
) >"$LOG_DIR/virtual.log" 2>&1 &

PID_VIRTUAL=$!

# ------------------------------------------------------------
# META FÍSICA: dominio ROS 0, ejecución remota en la Jetson
# Solo se transportan cuatro números simples por SSH.
# ------------------------------------------------------------
(
  ssh -o BatchMode=yes \
    "$PHYSICAL_HOST" \
    bash -s -- "$X" "$Y" "$QZ" "$QW" <<'REMOTE'
set -Eeuo pipefail

X="$1"
Y="$2"
QZ="$3"
QW="$4"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI || true

set +u
source /opt/ros/humble/setup.bash
source ~/jackson_dt_ws/install/setup.bash
set -u

ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{
    pose: {
      header: {
        frame_id: map
      },
      pose: {
        position: {
          x: ${X},
          y: ${Y},
          z: 0.0
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: ${QZ},
          w: ${QW}
        }
      }
    }
  }" \
  --feedback
REMOTE
) >"$LOG_DIR/physical.log" 2>&1 &

PID_PHYSICAL=$!

echo "Meta virtual enviada, PID=$PID_VIRTUAL"
echo "Meta física enviada, PID=$PID_PHYSICAL"
echo
echo "Esperando resultados..."

set +e
wait "$PID_VIRTUAL"
RC_VIRTUAL=$?

wait "$PID_PHYSICAL"
RC_PHYSICAL=$?
set -e

echo
echo "===== RESULTADO VIRTUAL ====="
tail -n 12 "$LOG_DIR/virtual.log"

echo
echo "===== RESULTADO FÍSICO ====="
tail -n 12 "$LOG_DIR/physical.log"

echo
echo "===== RESUMEN ====="

if grep -q "Goal finished with status: SUCCEEDED" \
  "$LOG_DIR/virtual.log"; then
  echo "Virtual: SUCCEEDED"
else
  echo "Virtual: revisar $LOG_DIR/virtual.log"
fi

if grep -q "Goal finished with status: SUCCEEDED" \
  "$LOG_DIR/physical.log"; then
  echo "Físico: SUCCEEDED"
else
  echo "Físico: revisar $LOG_DIR/physical.log"
fi

echo "Código virtual: $RC_VIRTUAL"
echo "Código físico:  $RC_PHYSICAL"

if [[ $RC_VIRTUAL -ne 0 || $RC_PHYSICAL -ne 0 ]]; then
  exit 1
fi
