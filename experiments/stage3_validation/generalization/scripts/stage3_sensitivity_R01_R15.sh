#!/usr/bin/env bash
set -Eeuo pipefail

# Jackson Stage 3 — Reviewer 1.5 generalization campaign
# 15 independent virtual runs per condition.
#
# Natural workflow:
#   1) Select condition:  condition G01_CYL_PLUS_Y
#   2) Isaac Stop, reset robot/cylinder, rearm Python trigger
#   3) pre
#   4) Press Play in Isaac
#   5) post
#   6) run R01
#   7) Stop Isaac and repeat for R02 ... R15
#
# Other commands: show, status, stop

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI || true

ROS_SETUP=/opt/ros/humble/setup.bash
SENSOR_SETUP=/home/carlos/jackson_waypoint_ws/install/setup.bash
DWB_CFG_BASE=/home/carlos/jackson_stage3_virtual/config/nav2_params_stage3_dwb_infl025.yaml
DWB_CFG_G04=/home/carlos/jackson_dt_ws/revision_r1_5_generalization/config/nav2_params_stage3_dwb_infl025_g04_lidar.yaml
DWB_CFG=$DWB_CFG_BASE
EKF_CFG=/home/carlos/jackson_dt_ws/config/ekf_stage3_virtual.yaml
DYNAMIC_SCRIPT=/home/carlos/jackson_dt_ws/revision_r1_5_generalization/scripts/stage3_dynamic_cylinder_sensitivity.py
LIDAR_DEGRADER=/home/carlos/jackson_dt_ws/revision_r1_5_generalization/scripts/lidar_degrader_g04.py
CAMPAIGN_ROOT=/home/carlos/jackson_dt_ws/revision_r1_5_generalization
RUNS_ROOT=$CAMPAIGN_ROOT/runs
CONDITION_FILE=$CAMPAIGN_ROOT/ACTIVE_CONDITION.txt

if [[ -f "$CONDITION_FILE" ]] && [[ "$(cat "$CONDITION_FILE")" == "G04_LIDAR_DEGRADED" ]]; then
  DWB_CFG=$DWB_CFG_G04
fi
STATE=/tmp/jackson_stage3_generalization
BAG_BASE=
PIDS=$STATE/pids
LOGS=$STATE/logs

START_X=-0.6909
START_Y=-0.0364
GOAL_X=1.2091
GOAL_Y=-0.0364

CONDITION=
WHEEL_RADIUS=
WHEEL_SEPARATION=
PHYSX_DAMPING=

load_condition(){
  [[ -f "$CONDITION_FILE" ]] || red "No hay condición seleccionada. Ejecuta: $0 condition G01_CYL_PLUS_Y"

  CONDITION=$(cat "$CONDITION_FILE")

  case "$CONDITION" in
    G01_CYL_PLUS_Y)
      WHEEL_RADIUS=0.03300
      WHEEL_SEPARATION=0.19800
      PHYSX_DAMPING=10000
      ;;
    G02_CYL_MINUS_Y)
      WHEEL_RADIUS=0.03300
      WHEEL_SEPARATION=0.19800
      PHYSX_DAMPING=10000
      ;;
    G03_LOW_FRICTION)
      WHEEL_RADIUS=0.03300
      WHEEL_SEPARATION=0.19800
      PHYSX_DAMPING=10000
      ;;
    G04_LIDAR_DEGRADED)
      WHEEL_RADIUS=0.03300
      WHEEL_SEPARATION=0.19800
      PHYSX_DAMPING=10000
      ;;
    *)
      red "Condición desconocida: $CONDITION"
      ;;
  esac

  BAG_BASE="$RUNS_ROOT/$CONDITION"
  mkdir -p "$BAG_BASE"
}

show_condition(){
  load_condition

  cat <<TXT

============================================================
 ACTIVE GENERALIZATION CONDITION
============================================================
Condition:          $CONDITION
Wheel radius:       $WHEEL_RADIUS m
Wheel separation:   $WHEEL_SEPARATION m
Expected damping:   $PHYSX_DAMPING
Runs:               R01-R15
Output directory:   $BAG_BASE

IMPORTANT:
  The PhysX damping value shown above is the EXPECTED value.
  It will be verified separately in Isaac Sim before running.
============================================================

TXT
}

cmd_condition(){
  [[ $# -eq 1 ]] || red "Uso: $0 condition G01_CYL_PLUS_Y"

  case "$1" in
    G01_CYL_PLUS_Y|G02_CYL_MINUS_Y|G03_LOW_FRICTION|G04_LIDAR_DEGRADED)
      ;;
    *)
      red "Condición inválida: $1"
      ;;
  esac

  printf '%s\n' "$1" > "$CONDITION_FILE"
  green "Condición seleccionada: $1"
  show_condition
}

if [[ -f /home/carlos/jackson_maps/jackson_map_02.yaml ]]; then
  MAP_FILE=/home/carlos/jackson_maps/jackson_map_02.yaml
else
  MAP_FILE=/home/carlos/jackson_maps/jackson_map_02/jackson_map_02.yaml
fi

mkdir -p "$PIDS" "$LOGS" "$RUNS_ROOT"
# shellcheck disable=SC1090
set +u
source "$ROS_SETUP"
set -u

blue(){ printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
green(){ printf '\033[1;32mOK:\033[0m %s\n' "$*"; }
yellow(){ printf '\033[1;33mADVERTENCIA:\033[0m %s\n' "$*" >&2; }
red(){ printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }
need(){ [[ -f "$1" ]] || red "No existe: $1"; }

check_files(){
  need "$ROS_SETUP"; need "$SENSOR_SETUP"; need "$DWB_CFG"; need "$EKF_CFG"
  need "$DYNAMIC_SCRIPT"; need "$MAP_FILE"
}

start_bg(){
  local name=$1; shift
  local pidfile=$PIDS/$name.pid logfile=$LOGS/$name.log
  [[ ! -f $pidfile ]] || red "$name ya tiene PID file. Ejecuta stop o pre."
  : > "$logfile"
  setsid bash -lc "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset CYCLONEDDS_URI || true; set +u; source '$ROS_SETUP'; set -u; $*" >"$logfile" 2>&1 &
  local pid=$!
  echo "$pid" > "$pidfile"
  sleep 1
  kill -0 "$pid" 2>/dev/null || { tail -n 50 "$logfile" >&2; red "No arrancó $name"; }
  green "$name iniciado (PID $pid); log: $logfile"
}

stop_one(){
  local pidfile=$1
  [[ -f $pidfile ]] || return 0
  local pid name
  pid=$(cat "$pidfile" 2>/dev/null || true)
  name=$(basename "$pidfile" .pid)
  if [[ -n ${pid:-} ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in {1..40}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
    kill -0 "$pid" 2>/dev/null && kill -TERM -- "-$pid" 2>/dev/null || true
  fi
  rm -f "$pidfile"
  green "$name detenido"
}

stop_managed(){
  shopt -s nullglob
  local f
  for f in "$PIDS"/*.pid; do stop_one "$f"; done
  shopt -u nullglob
}

cleanup_orphans(){
  # Remove detached Nav2 lifecycle managers left by previous launches.
  # This is operational cleanup only; it does not alter experiment parameters.
  pkill -TERM -f '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null || true
  sleep 1

  local p
  for p in \
    'ros2 bag record' rviz2 controller_server planner_server behavior_server \
    bt_navigator waypoint_follower velocity_smoother nav2_lifecycle_manager \
    map_server nav2_amcl component_container 'robot_localization.*ekf_node' \
    joint_state_odometry imu_preprocessor static_transform_publisher \
    localization_launch.py navigation_launch.py rviz_launch.py
  do pkill -INT -f "$p" 2>/dev/null || true; done
  sleep 1
  for p in \
    'ros2 bag record' rviz2 controller_server planner_server behavior_server \
    bt_navigator waypoint_follower velocity_smoother nav2_lifecycle_manager \
    map_server nav2_amcl component_container 'robot_localization.*ekf_node' \
    joint_state_odometry imu_preprocessor static_transform_publisher \
    localization_launch.py navigation_launch.py rviz_launch.py
  do pkill -TERM -f "$p" 2>/dev/null || true; done
}

wait_topic(){
  local topic=$1 timeout_s=${2:-30} start=$SECONDS
  while (( SECONDS-start < timeout_s )); do
    timeout 2s ros2 topic echo "$topic" --once >/dev/null 2>&1 && { green "Tópico $topic"; return; }
    sleep 1
  done
  red "No se recibió $topic en ${timeout_s}s"
}

wait_active(){
  local node=$1 timeout_s=${2:-45} start=$SECONDS out
  while (( SECONDS-start < timeout_s )); do
    out=$(timeout 3s ros2 lifecycle get "$node" 2>/dev/null || true)
    if grep -Eq '(^|[[:space:]])active([[:space:]]|\[|$)' <<<"$out"; then
      green "$node active"
      return
    fi

    out=$(timeout 3s ros2 service call "$node/get_state" lifecycle_msgs/srv/GetState '{}' 2>/dev/null || true)
    if grep -Eq "label:[[:space:]]*'?active'?|label='active'" <<<"$out"; then
      green "$node active"
      return
    fi
    sleep 1
  done
  red "$node no llegó a active"
}

wait_tf(){
  local a=$1 b=$2 timeout_s=${3:-30} start=$SECONDS out
  while (( SECONDS-start < timeout_s )); do
    out=$(timeout 3s ros2 run tf2_ros tf2_echo "$a" "$b" 2>&1 || true)
    if grep -q 'Translation:' <<<"$out"; then
      green "TF $a -> $b"
      return
    fi
    sleep 1
  done
  red "No apareció TF $a -> $b"
}

data_value(){ timeout 4s ros2 topic echo "$1" --once 2>/dev/null | awk '/data:/{print $2; exit}'; }
param(){
  local node="$1"
  local key="$2"
  local out=""
  local attempt

  for attempt in 1 2 3; do
    if out=$(timeout 5s ros2 param get "$node" "$key" 2>&1); then
      printf '%s\n' "$out"
      return 0
    fi

    echo "WARN: param get intento $attempt/3 falló: $node $key" >&2
    [[ $attempt -lt 3 ]] && sleep 1
  done

  echo "$out" >&2
  return 1
}

wait_active_soft(){
  local node="$1"
  local timeout_s="${2:-75}"
  local deadline=$((SECONDS + timeout_s))
  local out=""

  while (( SECONDS < deadline )); do
    out=$(timeout 4s ros2 lifecycle get "$node" 2>/dev/null || true)

    if [[ "$out" == active* ]]; then
      green "OK: $node active"
      return 0
    fi

    sleep 1
  done

  yellow "WARN: $node no llegó a active en ${timeout_s}s"
  return 1
}

stop_navigation_only(){
  # Detiene únicamente Nav2 navigation.
  # NO toca EKF, AMCL, map_server, sensores ni TF.
  stop_one "$PIDS/navigation.pid" || true

  local pattern

  for pattern in \
    '/opt/ros/humble/lib/nav2_controller/controller_server' \
    '/opt/ros/humble/lib/nav2_smoother/smoother_server' \
    '/opt/ros/humble/lib/nav2_planner/planner_server' \
    '/opt/ros/humble/lib/nav2_behaviors/behavior_server' \
    '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator' \
    '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower' \
    '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother'
  do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done

  pkill -TERM -f \
    '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager.*lifecycle_manager_navigation' \
    2>/dev/null || true

  sleep 2

  for pattern in \
    '/opt/ros/humble/lib/nav2_controller/controller_server' \
    '/opt/ros/humble/lib/nav2_smoother/smoother_server' \
    '/opt/ros/humble/lib/nav2_planner/planner_server' \
    '/opt/ros/humble/lib/nav2_behaviors/behavior_server' \
    '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator' \
    '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower' \
    '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother'
  do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done

  pkill -KILL -f \
    '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager.*lifecycle_manager_navigation' \
    2>/dev/null || true

  rm -f "$PIDS/navigation.pid"
  sleep 2
}

start_navigation_with_retry(){
  local attempt
  local node
  local ok

  for attempt in 1 2; do
    blue "Nav2 — intento $attempt/2"

    start_bg navigation \
      "ros2 launch nav2_bringup navigation_launch.py params_file:='$DWB_CFG' use_sim_time:=true autostart:=true use_composition:=False use_respawn:=False"

    ok=1

    for node in \
      /controller_server \
      /planner_server \
      /behavior_server \
      /bt_navigator \
      /velocity_smoother
    do
      if ! wait_active_soft "$node" 75; then
        ok=0
        break
      fi
    done

    if (( ok == 1 )); then
      green "OK: Nav2 completo active en intento $attempt/2"
      return 0
    fi

    yellow "WARN: activación Nav2 incompleta; reiniciando SOLO Nav2"

    stop_navigation_only

    if (( attempt < 2 )); then
      sleep 4
    fi
  done

  red "Nav2 no pudo activarse después de 2 intentos completos"
}

verify_controller(){
  local a b c d
  a=$(param /controller_server FollowPath.plugin)
  b=$(param /controller_server FollowPath.min_speed_theta)
  c=$(param /controller_server general_goal_checker.xy_goal_tolerance)
  d=$(param /controller_server general_goal_checker.yaw_goal_tolerance)
  printf '%s\n%s\n%s\n%s\n' "$a" "$b" "$c" "$d"
  grep -q 'dwb_core::DWBLocalPlanner' <<<"$a" || red 'Plugin activo distinto de DWB'
  grep -Eq '0\.3([0-9]*)?' <<<"$b" || red 'min_speed_theta distinto de 0.30'
  grep -Eq '0\.25([0-9]*)?' <<<"$c" || red 'xy_goal_tolerance distinto de 0.25'
  grep -Eq '0\.4([0-9]*)?' <<<"$d" || red 'yaw_goal_tolerance distinto de 0.40'
  green 'Controlador y tolerancias correctos'
}

verify_costmaps(){
  local node r i s
  for node in /local_costmap/local_costmap /global_costmap/global_costmap; do
    blue "$node"
    r=$(param "$node" robot_radius); i=$(param "$node" inflation_layer.inflation_radius); s=$(param "$node" inflation_layer.cost_scaling_factor)
    printf '%s\n%s\n%s\n' "$r" "$i" "$s"
    grep -Eq '0\.16([0-9]*)?' <<<"$r" || red "$node robot_radius incorrecto"
    grep -Eq '0\.25([0-9]*)?' <<<"$i" || red "$node inflation_radius incorrecto"
    grep -Eq '10(\.0+)?' <<<"$s" || red "$node cost_scaling_factor incorrecto"
  done
}

verify_trigger_initial(){
  wait_topic /stage3/cylinder_inserted 10
  wait_topic /stage3/forward_displacement 10
  wait_topic /stage3/trigger_displacement 10
  local ins disp trig
  ins=$(data_value /stage3/cylinder_inserted)
  disp=$(data_value /stage3/forward_displacement)
  trig=$(data_value /stage3/trigger_displacement)
  printf 'cylinder_inserted=%s\nforward_displacement=%s\ntrigger_displacement=%s\n' "$ins" "$disp" "$trig"
  [[ $ins == false ]] || red 'El cilindro ya está insertado. Reinicia Isaac y rearma el script.'
  python3 - "$disp" "$trig" <<'PY'
import sys
x=float(sys.argv[1]); t=float(sys.argv[2])
if abs(x)>0.02: raise SystemExit(f'forward_displacement={x:.6f}, no está cerca de cero')
if abs(t+1.0)>1e-6: raise SystemExit(f'trigger_displacement={t:.6f}, debería ser -1.0')
PY
  green 'Disparador dinámico armado'
}

cmd_pre(){
  load_condition
  show_condition
  check_files
  blue 'Limpiando la ejecución anterior'
  rm -f "$STATE/READY"
  stop_managed
  cleanup_orphans
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 2
  ros2 daemon start >/dev/null
  sleep 2

  blue 'TF estáticos'
  start_bg tf_base_link "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.034 --roll 0 --pitch 0 --yaw 0 --frame-id base_footprint --child-frame-id base_link"
  start_bg tf_imu "ros2 run tf2_ros static_transform_publisher --x -0.05 --y -0.045 --z 0.134 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id imu_link"

  blue 'Sensores derivados'
  start_bg wheel_odom "set +u; source '$SENSOR_SETUP'; set -u; ros2 run jackson_virtual_sensors joint_state_odometry --ros-args -p use_sim_time:=true -p wheel_radius:=$WHEEL_RADIUS -p wheel_separation:=$WHEEL_SEPARATION -p left_joint:=left_wheel_joint -p right_joint:=right_wheel_joint -p left_sign:=1.0 -p right_sign:=1.0 -p odom_frame:=odom -p base_frame:=base_footprint"
  start_bg imu_filter "set +u; source '$SENSOR_SETUP'; set -u; ros2 run jackson_virtual_sensors imu_preprocessor --ros-args -p use_sim_time:=true -p input_topic:=/imu/data -p output_topic:=/imu/data_filtered -p output_frame:=imu_link -p angular_velocity_z_scale:=0.6953"

  if [[ "$CONDITION" == "G04_LIDAR_DEGRADED" ]]; then
    blue 'LiDAR degradado — G04'
    pkill -f "$LIDAR_DEGRADER" >/dev/null 2>&1 || true
    start_bg lidar_degrader "set +u; source '$ROS_SETUP'; set -u; python3 '$LIDAR_DEGRADER'"
  fi

  cat <<'TXT'

PRE completado.
Ahora, en Isaac Sim:
  1. Confirma Jackson (-0.6909, -0.0364, yaw 0°).
  2. Confirma cilindro (0.6091, -0.0364, z=-1.0000).
  3. Rearma en Script Editor:
     exec(open("/home/carlos/jackson_dt_ws/revision_r1_5_generalization/scripts/stage3_dynamic_cylinder_sensitivity.py", encoding="utf-8").read())
  4. Pulsa Play y espera 5 s.
  5. Ejecuta en terminal: stage3_sensitivity_R01_R15.sh post
TXT
}

cmd_post(){
  check_files
  blue 'Esperando Isaac Sim'
  for t in /clock /ground_truth/odom /joint_states /scan /imu/data /wheel/odom /imu/data_filtered; do wait_topic "$t" 30; done

  if [[ -f "$CONDITION_FILE" ]] && [[ "$(cat "$CONDITION_FILE")" == "G04_LIDAR_DEGRADED" ]]; then
    wait_topic /scan_degraded 30
    green "G04: /scan_degraded disponible"
  fi

  blue 'EKF'
  start_bg ekf "ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node --params-file '$EKF_CFG'"
  wait_topic /odometry/filtered 30
  wait_tf odom base_footprint 30

  blue 'Localización'
  start_bg localization "ros2 launch nav2_bringup localization_launch.py map:='$MAP_FILE' params_file:='$DWB_CFG' use_sim_time:=true autostart:=true use_composition:=False"
  wait_active /map_server 50
  wait_active /amcl 50

  blue 'Pose inicial'
  ros2 topic pub -r 2 --times 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0076]}}" >/dev/null
  sleep 4
  wait_topic /amcl_pose 20
  wait_tf map base_footprint 60

  blue 'Nav2'
  start_navigation_with_retry
  verify_controller
  verify_costmaps
  verify_trigger_initial

  blue 'Limpieza de costmaps'
  ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}' >/dev/null
  ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}' >/dev/null
  sleep 3

  start_bg rviz "ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true"
  date -Iseconds > "$STATE/READY"

  cat <<'TXT'

POST completado: sistema listo.
Confirma en RViz que el mapa y el scan estén alineados y que el cilindro aún no sea visible.
Ejecuta, por ejemplo:
  stage3_sensitivity_R01_R15.sh run R01
TXT
}

valid_run(){ [[ $1 =~ ^R(0[1-9]|1[0-5])$ ]] || red 'Usa R01, R02, ..., R15'; }

save_artifacts(){
  local bag=$1 run=$2 result=$3 contact=$4 action_log=$5
  timeout 6s ros2 run tf2_ros tf2_echo map base_footprint >"$bag/final_tf_map_base_footprint.txt" 2>&1 || true
  timeout 6s ros2 topic echo /amcl_pose --once >"$bag/final_amcl_pose.txt" 2>&1 || true
  timeout 6s ros2 topic echo /ground_truth/odom --once >"$bag/final_ground_truth_odom.txt" 2>&1 || true
  timeout 6s ros2 topic echo /stage3/forward_displacement --once >"$bag/final_forward_displacement.txt" 2>&1 || true
  timeout 6s ros2 topic echo /stage3/trigger_displacement --once >"$bag/trigger_displacement.txt" 2>&1 || true
  timeout 10s ros2 param dump /controller_server >"$bag/controller_server_active_params.yaml" 2>&1 || true
  timeout 10s ros2 param dump /local_costmap/local_costmap >"$bag/local_costmap_active_params.yaml" 2>&1 || true
  timeout 10s ros2 param dump /global_costmap/global_costmap >"$bag/global_costmap_active_params.yaml" 2>&1 || true
  timeout 10s ros2 param dump /velocity_smoother >"$bag/velocity_smoother_active_params.yaml" 2>&1 || true

  # Runtime parameters actually used by the wheel-odometry node.
  timeout 10s ros2 param dump /joint_state_odometry >"$bag/wheel_odom_active_params.yaml" 2>&1 || true

  # Condition-level PhysX readback performed directly in Isaac Sim.
  if [[ -f "$BAG_BASE/PHYSX_CONDITION_SETUP.txt" ]]; then
    cp -a "$BAG_BASE/PHYSX_CONDITION_SETUP.txt" "$bag/"
  else
    echo "WARNING: PHYSX_CONDITION_SETUP.txt was not available at artifact-save time."       >"$bag/PHYSX_CONDITION_SETUP_MISSING.txt"
  fi

  # Exact campaign definition used for this run.
  [[ -f "$CAMPAIGN_ROOT/CONDITIONS.txt" ]] &&     cp -a "$CAMPAIGN_ROOT/CONDITIONS.txt" "$bag/" || true

  [[ -f "$CONDITION_FILE" ]] &&     cp -a "$CONDITION_FILE" "$bag/ACTIVE_CONDITION.txt" || true

  [[ -f "$CAMPAIGN_ROOT/scripts/apply_physx_condition.py" ]] &&     cp -a "$CAMPAIGN_ROOT/scripts/apply_physx_condition.py" "$bag/" || true

  cp -a "$(readlink -f "$0")" "$bag/stage3_sensitivity_R01_R15.sh" || true

  cp -a "$DWB_CFG" "$EKF_CFG" "$DYNAMIC_SCRIPT" "$MAP_FILE" "$bag/"
  [[ -f ${MAP_FILE%.yaml}.pgm ]] && cp -a "${MAP_FILE%.yaml}.pgm" "$bag/" || true
  cp -a "$action_log" "$bag/navigate_to_pose_action.txt"
  cat >"$bag/RUN_NOTES.txt" <<TXT
Jackson Stage 3 Virtual Dynamic Cylinder Official Run — INFL025 COST10
Condition: $CONDITION
Run: $run
Result: $result
Obstacle contact: $contact
Wheel radius: $WHEEL_RADIUS m
Wheel separation: $WHEEL_SEPARATION m
Expected PhysX damping: $PHYSX_DAMPING
PhysX verification: PHYSX_CONDITION_SETUP.txt
Wheel-odometry runtime parameters: wheel_odom_active_params.yaml
Manual cancellation: no
Operator intervention during navigation: no
Controller: DWBLocalPlanner
min_speed_theta: 0.30 rad/s
xy_goal_tolerance: 0.25 m
yaw_goal_tolerance: 0.40 rad
robot_radius: 0.16 m
inflation_radius: 0.25 m
cost_scaling_factor: 10.0
Start: x=$START_X m, y=$START_Y m, yaw=0 rad
Goal: x=$GOAL_X m, y=$GOAL_Y m, yaw=0 rad
Cylinder: x=0.6091 m, y=-0.0364 m, parked z=-1.0000 m, inserted z=0.1900 m
Nominal trigger: 0.120 m ground-truth forward displacement
Completion: $(date -Iseconds)
TXT
  ros2 bag info "$bag" | tee "$bag/bag_info.txt"
  sha256sum "$bag"/*.db3 > "$bag/db3_sha256.txt"

  (
    cd "$bag" || exit 0
    files=(
      RUN_NOTES.txt
      wheel_odom_active_params.yaml
      PHYSX_CONDITION_SETUP.txt
      ACTIVE_CONDITION.txt
      CONDITIONS.txt
      stage3_sensitivity_R01_R15.sh
      stage3_dynamic_cylinder_sensitivity.py
      apply_physx_condition.py
      nav2_params_stage3_dwb_infl025.yaml
      ekf_stage3_virtual.yaml
      jackson_map_02.yaml
      jackson_map_02.pgm
    )

    existing=()
    for f in "${files[@]}"; do
      [[ -f "$f" ]] && existing+=("$f")
    done

    if (( ${#existing[@]} > 0 )); then
      sha256sum "${existing[@]}" > PROVENANCE_SHA256.txt
    fi
  )
}

cmd_run(){
  [[ $# -eq 1 ]] || red "Uso: $0 run R01"
  load_condition
  local run=$1; valid_run "$run"; check_files
  [[ -f $STATE/READY ]] || red 'Ejecuta primero pre, Play y post.'
  wait_active /controller_server 10
  wait_active /bt_navigator 10
  verify_trigger_initial

  local old
  old=$(find "$BAG_BASE" -mindepth 1 -maxdepth 1 -type d -name "${CONDITION}_${run}_*" -print -quit 2>/dev/null || true)
  [[ -z $old ]] || red "Ya existe $run: $old"

  local id bag baglog actionlog bagpid result inserted trigger contact answer
  id="${CONDITION}_${run}_$(date +%Y%m%d_%H%M%S)"
  bag="$BAG_BASE/$id"; baglog="$LOGS/${id}_rosbag.log"; actionlog="$LOGS/${id}_action.log"
  echo "$bag" >/tmp/jackson_dynamic_official_bag.txt
  echo "$run" >/tmp/jackson_dynamic_official_run.txt

  local topics=(/clock /cmd_vel /cmd_vel_nav /scan /scan_degraded /joint_states /wheel/odom /imu/data /imu/data_filtered /odometry/filtered /ground_truth/odom /amcl_pose /plan /local_plan /map /map_metadata /tf /tf_static /parameter_events /stage3/forward_displacement /stage3/cylinder_inserted /stage3/trigger_displacement /navigate_to_pose/_action/goal /navigate_to_pose/_action/result /navigate_to_pose/_action/cancel /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback /behavior_tree_log /rosout)

  blue "Grabando $run"
  setsid ros2 bag record --include-hidden-topics -o "$bag" "${topics[@]}" >"$baglog" 2>&1 &
  bagpid=$!; echo "$bagpid" >"$PIDS/official_bag.pid"
  trap 'yellow "Interrupción: cerrando bag"; stop_one "$PIDS/official_bag.pid" || true; rm -f "$STATE/READY"' INT TERM
  sleep 4
  kill -0 "$bagpid" 2>/dev/null || { tail -n 80 "$baglog" >&2; red 'No arrancó rosbag'; }
  [[ -d $bag ]] || red 'No se creó el directorio del bag'
  green "Bag activo: $bag"

  ros2 topic echo /stage3/cylinder_inserted --once >"$bag/initial_cylinder_inserted.txt"
  ros2 topic echo /stage3/forward_displacement --once >"$bag/initial_forward_displacement.txt"
  ros2 topic echo /stage3/trigger_displacement --once >"$bag/initial_trigger_displacement.txt"

  blue 'Enviando meta'
  set +e
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback 2>&1 | tee "$actionlog"
  local action_rc=${PIPESTATUS[0]}
  set -e

  sleep 2
  stop_one "$PIDS/official_bag.pid"
  trap - INT TERM

  if grep -q 'Goal finished with status: SUCCEEDED' "$actionlog"; then result=SUCCEEDED
  elif grep -q 'Goal finished with status: ABORTED' "$actionlog"; then result=ABORTED
  elif grep -q 'Goal finished with status: CANCELED' "$actionlog"; then result=CANCELED
  elif (( action_rc != 0 )); then result=ACTION_CLI_ERROR
  else result=UNKNOWN; fi

  inserted=$(data_value /stage3/cylinder_inserted || true)
  trigger=$(data_value /stage3/trigger_displacement || true)
  printf '\nResultado: %s\nCilindro insertado: %s\nTrigger: %s m\n' "$result" "${inserted:-SIN_DATO}" "${trigger:-SIN_DATO}"
  printf '¿Hubo contacto con el cilindro? [s/N]: '
  read -r answer || true
  contact=no; [[ ${answer:-} =~ ^[sSyY]$ ]] && contact=yes

  save_artifacts "$bag" "$run" "$result" "$contact" "$actionlog"
  rm -f "$STATE/READY"

  cat <<TXT

Ejecución terminada.
Run: $run
Resultado: $result
Bag: $bag
Contacto: $contact

El bag ya se cerró correctamente. Ahora pulsa Stop en Isaac Sim.
Para la siguiente prueba de ESTA MISMA condición:
  Stop Isaac -> reset robot/cylinder -> rearmar trigger -> pre -> Play -> post -> run RXX.

Cuando completes R01-R15, selecciona la siguiente condición con:
  $0 condition CXX_...
TXT
}

cmd_status(){
  blue 'Procesos administrados'
  shopt -s nullglob
  local f pid
  for f in "$PIDS"/*.pid; do pid=$(cat "$f" 2>/dev/null || true); printf '%-24s PID=%s %s\n' "$(basename "$f" .pid)" "$pid" "$(kill -0 "$pid" 2>/dev/null && echo RUNNING || echo STOPPED)"; done
  shopt -u nullglob
  blue 'Nodos ROS'; ros2 node list 2>/dev/null | sort || true
  blue 'Estado dinámico'
  printf 'inserted=%s\ndisplacement=%s\ntrigger=%s\n' "$(data_value /stage3/cylinder_inserted 2>/dev/null || echo SIN_DATO)" "$(data_value /stage3/forward_displacement 2>/dev/null || echo SIN_DATO)" "$(data_value /stage3/trigger_displacement 2>/dev/null || echo SIN_DATO)"
  [[ -f $STATE/READY ]] && green "READY desde $(cat "$STATE/READY")" || yellow 'No READY'
}

cmd_stop(){
  rm -f "$STATE/READY"
  stop_managed
  cleanup_orphans
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  green 'Todo detenido'
}

usage(){ cat <<TXT
Uso:
  $0 condition G01_CYL_PLUS_Y
  $0 show
  $0 pre
  $0 post
  $0 run R01
  $0 status
  $0 stop

Flujo humano por run:
  condition (solo al cambiar de condición)
  -> Isaac Stop + reset + rearmar Python
  -> pre
  -> Play
  -> post
  -> run RXX
  -> Stop Isaac
TXT
}

case ${1:-} in
  condition) shift; cmd_condition "$@" ;;
  show) [[ $# -eq 1 ]] || red 'Uso: show'; show_condition ;;
  pre) [[ $# -eq 1 ]] || red 'Uso: pre'; cmd_pre ;;
  post) [[ $# -eq 1 ]] || red 'Uso: post'; cmd_post ;;
  run) shift; cmd_run "$@" ;;
  status) cmd_status ;;
  stop) cmd_stop ;;
  -h|--help|help|'') usage ;;
  *) usage; red "Comando desconocido: $1" ;;
esac
