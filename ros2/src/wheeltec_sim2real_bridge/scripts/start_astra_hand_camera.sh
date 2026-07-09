#!/bin/bash
# =============================================================================
# start_astra_hand_camera.sh
# 启动 Orbbec Astra S 手眼相机: ROS1 astra_camera 驱动 + ros1_bridge,
# 把 Astra 的 RGB 压缩流桥接到 ROS2, 供前端"手眼相机"槽位实时显示。
#
# 链路:
#   Astra S --(OpenNI2)--> astra_camera(ROS1) --> /camera/color/image_raw + /camera/depth/image_raw
#        --> image_transport republish --> /camera/color/image_raw/compressed (ROS1)
#        --> image_transport republish --> /camera/depth/image_raw/compressed  (ROS1, PNG 16-bit)
#        --> ros1_bridge dynamic_bridge --bridge-all-1to2-topics
#        --> /camera/color/image_raw/compressed + /camera/depth/image_raw/compressed + camera_info (ROS2)
#        --> rosbridge_websocket --> 浏览器前端 (CompressedImage, jpeg)
#        --> rgbd_tool_detector (YOLO + RGB-D, 输出世界坐标标注 + 调试图)
#
# 前端订阅话题: /camera/color/image_raw/compressed  (消息类型 CompressedImage)
# 检测器订阅:   /camera/color/image_raw/compressed + /camera/depth/image_raw/compressed + /camera/color/camera_info
#
# 用法:
#   ./start_astra_hand_camera.sh start     # 后台启动 (默认, 手动运行)
#   ./start_astra_hand_camera.sh supervise # 前台阻塞 (供 ROS2 launch ExecuteProcess 调用)
#   ./start_astra_hand_camera.sh stop      # 停止
#   ./start_astra_hand_camera.sh restart   # 重启
#   ./start_astra_hand_camera.sh status    # 查看状态
#
# 依赖:
#   - ROS1 noetic + ~/wheeltec_robot (astra_camera 驱动, OpenNI2/libPS1080)
#   - ros-foxy-ros1-bridge (sudo apt install ros-foxy-ros1-bridge)
#   - ROS2 bringup (rosbridge_server) 已在运行 (本脚本只负责手眼相机链路)
#
# 注意 (Astra S 已知问题):
#   Astra S 在 Jetson USB2.0 上偶尔会卡死 (astra 日志报 "USB transfer timeout"),
#   此时软件复位也无法恢复, 需要物理拔插 Astra 的 USB 线或重启小车。
#   本脚本启动时会先做一次 USB 软件复位 + 彩色流健康检查; 若失败会明确提示。
#   注意: color+depth 同时开启时 USB 功耗增大, 电池电量低时可能导致机械臂串口
#   干扰 (臂抽搐)。请确保电池电量充足后再启用 depth。
# =============================================================================


# 注意: 不使用 set -u, 因为 ROS 的 setup.bash (1.ros_distro.sh 等) 会引用未定义变量
LOGDIR=/tmp
PIDFILE="$LOGDIR/astra_hand_bridge.pid"
NOETIC=/opt/ros/noetic/setup.bash
FOXY=/opt/ros/foxy/setup.bash
WS=/home/wheeltec/wheeltec_robot/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

log() { echo "[astra_hand] $*"; }

is_running() {
  [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE" 2>/dev/null)" 2>/dev/null
}

# 清理残留进程 (bridge 挂了但 astra/republish 还活着的情况)
_cleanup() {
  pkill -f 'dynamic_bridge'              2>/dev/null
  pkill -f 'republish raw in:=/camera/color'  2>/dev/null
  pkill -f 'republish raw in:=/camera/depth'  2>/dev/null
  pkill -f 'astra_camera_node'           2>/dev/null
  pkill -f 'roslaunch astra_camera astra.launch' 2>/dev/null
  pkill -x rosmaster                     2>/dev/null
  pkill -x roscore                       2>/dev/null
  rm -f "$PIDFILE"
  rm -f /dev/shm/astra_device.lock       2>/dev/null
  sleep 1
}

# 软件复位 Astra 的 USB 设备 (authorized 开关)。需要 sudo (密码 dongguan)。
# 注意: 若 Astra 已彻底卡死 (USB transfer timeout), 软件复位无法恢复,
#       需要物理拔插 Astra 的 USB 线或重启小车。
_usb_reset_astra() {
  local d
  d=$(for x in /sys/bus/usb/devices/*; do
        [ -f "$x/idVendor" ] && grep -q 2bc5 "$x/idVendor" 2>/dev/null && { echo "$x"; break; }
      done)
  if [ -z "$d" ]; then log "  (未找到 Astra USB 设备, 跳过 USB 复位)"; return 1; fi
  if [ -w "$d/authorized" ]; then
    echo 0 > "$d/authorized" 2>/dev/null; sleep 1; echo 1 > "$d/authorized" 2>/dev/null; sleep 2
  else
    echo 'dongguan' | sudo -S -p '' sh -c "echo 0 > $d/authorized 2>/dev/null; sleep 1; echo 1 > $d/authorized 2>/dev/null" 2>/dev/null
    sleep 2
  fi
  log "  Astra USB 复位完成 ($d)"
  return 0
}

start() {
  if is_running; then log "已在运行 (bridge pid $(cat "$PIDFILE"))"; exit 0; fi
  log "清理残留..."
  _cleanup
  log "软件复位 Astra USB (清除可能的 OpenNI2 卡死状态)..."
  _usb_reset_astra

  # --- ROS1 环境 ---
  # shellcheck disable=SC1090,SC1091
  source "$NOETIC"
  # shellcheck disable=SC1090,SC1091
  source "$WS"
  export ROS_MASTER_URI=http://localhost:11311

  # 1. roscore (若未运行)
  if ! pgrep -x rosmaster >/dev/null 2>&1; then
    nohup roscore >"$LOGDIR/astra_hand_roscore.log" 2>&1 &
    log "等待 roscore 启动..."
    for _ in $(seq 1 10); do pgrep -x rosmaster >/dev/null 2>&1 && break; sleep 1; done
  fi

  # 2. astra 相机 (彩色 + 深度, 关闭 ir/点云/TF)
  nohup roslaunch astra_camera astra.launch \
    enable_color:=true enable_ir:=false enable_depth:=true \
    enable_point_cloud:=false publish_tf:=true \
    >"$LOGDIR/astra_hand_astra.log" 2>&1 &
  ASTRA_PID=$!
  log "astra 已启动 (pid $ASTRA_PID), 等待彩色流出图 (最多 25s)..."
  COLOR_OK=0
  for i in $(seq 1 25); do
    if rostopic list 2>/dev/null | grep -q '/camera/color/image_raw'; then
      log "彩色流就绪 (等待 ${i}s)"; COLOR_OK=1; break
    fi
    sleep 1
  done

  # 健康检查: 若彩色流没出, 大概率是 Astra USB 卡死 (USB transfer timeout)
  if [ "$COLOR_OK" = "0" ]; then
    log "!! 错误: Astra 彩色流未启动。"
    log "   常见原因: Astra USB 卡死 (日志里会看到 'USB transfer timeout')。"
    log "   恢复方法: 物理拔插 Astra 的 USB 线, 或重启小车, 然后重新运行: $0 start"
    log "   astra 日志: $LOGDIR/astra_hand_astra.log"
    log "   (已启动的 roslaunch/roscore 仍在后台, 可用 $0 stop 清理)"
    exit 1
  fi

  # 3. image_transport republish: color raw -> compressed
  nohup rosrun image_transport republish \
    raw in:=/camera/color/image_raw compressed out:=/camera/color/image_raw \
    >"$LOGDIR/astra_hand_republish.log" 2>&1 &
  log "republish 已启动 (color raw -> compressed)"

  # 3b. image_transport republish: depth raw -> compressed (PNG 16-bit)
  nohup rosrun image_transport republish \
    raw in:=/camera/depth/image_raw compressed out:=/camera/depth/image_raw \
    >"$LOGDIR/astra_hand_republish_depth.log" 2>&1 &
  log "republish 已启动 (depth raw -> compressed)"
  sleep 2

  # 4. ros1_bridge: ROS1 -> ROS2 (桥接所有 ROS1 话题, 仅在有 ROS2 订阅者时才实际传数据)
  # shellcheck disable=SC1090,SC1091
  source "$FOXY"
  export ROS_MASTER_URI=http://localhost:11311
  nohup ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics \
    >"$LOGDIR/astra_hand_bridge.log" 2>&1 &
  BRIDGE_PID=$!
  echo "$BRIDGE_PID" > "$PIDFILE"
  log "ros1_bridge 已启动 (pid $BRIDGE_PID)"

  log "完成。前端手眼相机槽位订阅: /camera/color/image_raw/compressed"
  log "RGB-D 检测器订阅: /camera/color/image_raw/compressed + /camera/depth/image_raw/compressed"
  log "日志: $LOGDIR/astra_hand_{roscore,astra,republish,republish_depth,bridge}.log"
}

stop() {
  log "停止..."
  _cleanup
  log "已停止"
}

# 前台阻塞模式: 供 ROS2 launch (ExecuteProcess) 调用。
# 启动全链路并阻塞, 收到 SIGINT/SIGTERM (launch 退出/Ctrl+C) 时清理全部子进程。
supervise() {
  log "(supervise) 清理残留..."
  _cleanup
  log "(supervise) 软件复位 Astra USB..."
  _usb_reset_astra
  # shellcheck disable=SC1090,SC1091
  source "$NOETIC"
  # shellcheck disable=SC1090,SC1091
  source "$WS"
  export ROS_MASTER_URI=http://localhost:11311
  ROSCORE_PID=""; ASTRA_PID=""; REPUB_PID=""; DEPTH_REPUB_PID=""; BRIDGE_PID=""

  _kill_supervise_children() {
    [ -n "$BRIDGE_PID" ]  && kill "$BRIDGE_PID"  2>/dev/null
    [ -n "$DEPTH_REPUB_PID" ] && kill "$DEPTH_REPUB_PID" 2>/dev/null
    [ -n "$REPUB_PID" ]   && kill "$REPUB_PID"   2>/dev/null
    [ -n "$ASTRA_PID" ]   && kill "$ASTRA_PID"   2>/dev/null
    [ -n "$ROSCORE_PID" ] && kill "$ROSCORE_PID" 2>/dev/null
    pkill -9 -f dynamic_bridge 2>/dev/null
    pkill -9 -f 'republish raw in:=/camera/color' 2>/dev/null
    pkill -9 -f 'republish raw in:=/camera/depth' 2>/dev/null
    pkill -9 -f astra_camera_node 2>/dev/null
    pkill -9 -f 'roslaunch astra' 2>/dev/null
    pkill -x rosmaster 2>/dev/null; pkill -x roscore 2>/dev/null
    rm -f "$PIDFILE"
    rm -f /dev/shm/astra_device.lock 2>/dev/null
  }
  trap '_kill_supervise_children; exit 0' INT TERM

  # 1. roscore
  if ! pgrep -x rosmaster >/dev/null 2>&1; then
    nohup roscore >"$LOGDIR/astra_hand_roscore.log" 2>&1 &
    ROSCORE_PID=$!
    for _ in $(seq 1 10); do pgrep -x rosmaster >/dev/null 2>&1 && break; sleep 1; done
  fi

  # 2. astra (最多重试 2 次, 应对偶发的 OpenNI2 初始化失败)
  COLOR_OK=0
  for attempt in 1 2; do
    if [ "$attempt" = "2" ]; then
      log "(supervise) 首次未出图, 重试 astra..."
      pkill -9 -f astra_camera_node 2>/dev/null; pkill -9 -f 'roslaunch astra' 2>/dev/null; sleep 2
      rm -f /dev/shm/astra_device.lock 2>/dev/null
    fi
    nohup roslaunch astra_camera astra.launch \
      enable_color:=true enable_ir:=false enable_depth:=true \
      enable_point_cloud:=false publish_tf:=false \
      color_width:=320 color_height:=240 \
      >"$LOGDIR/astra_hand_astra.log" 2>&1 &
    ASTRA_PID=$!
    log "(supervise) astra 启动 (pid $ASTRA_PID), 等待彩色流 (第 $attempt 次, 最多 25s)..."
    for i in $(seq 1 25); do
      rostopic list 2>/dev/null | grep -q '/camera/color/image_raw' && { COLOR_OK=1; log "(supervise) 彩色流就绪 (${i}s)"; break; }
      sleep 1
    done
    [ "$COLOR_OK" = "1" ] && break
  done

  if [ "$COLOR_OK" = "0" ]; then
    log "!! Astra 彩色流未启动 (USB transfer timeout?)。"
    log "   请物理拔插 Astra USB 线或重启小车, 再重启 launch (astra_hand:=true)。"
    log "   astra 日志: $LOGDIR/astra_hand_astra.log"
    _kill_supervise_children
    trap - INT TERM
    exit 1
  fi

  # 3. republish (color raw -> compressed)
  nohup rosrun image_transport republish \
    raw in:=/camera/color/image_raw compressed out:=/camera/color/image_raw \
    >"$LOGDIR/astra_hand_republish.log" 2>&1 &
  REPUB_PID=$!

  # 3b. republish (depth raw -> compressed, PNG 16-bit)
  nohup rosrun image_transport republish \
    raw in:=/camera/depth/image_raw compressed out:=/camera/depth/image_raw \
    >"$LOGDIR/astra_hand_republish_depth.log" 2>&1 &
  DEPTH_REPUB_PID=$!
  sleep 2

  # 3c. 触发 republish 开始发布 compressed (republish 懒订阅: 无人订阅则不发布)
  #     dynamic_bridge 启动时需发现 compressed topic 才能桥接到 ROS2
  timeout 5 rostopic echo /camera/color/image_raw/compressed --noarr >/dev/null 2>&1 &
  timeout 5 rostopic echo /camera/depth/image_raw/compressed --noarr >/dev/null 2>&1 &
  sleep 2

  # 4. ros1_bridge
  # shellcheck disable=SC1090,SC1091
  source "$FOXY"
  export ROS_MASTER_URI=http://localhost:11311
  nohup ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics \
    >"$LOGDIR/astra_hand_bridge.log" 2>&1 &
  BRIDGE_PID=$!
  echo "$BRIDGE_PID" > "$PIDFILE"
  log "(supervise) 链路就绪, 前台阻塞运行 (bridge pid $BRIDGE_PID)。"
  log "   前端手眼相机槽位订阅: /camera/color/image_raw/compressed"
  log "   RGB-D 检测器订阅: /camera/color/image_raw/compressed + /camera/depth/image_raw/compressed"
  log "   日志: $LOGDIR/astra_hand_{roscore,astra,republish,republish_depth,bridge}.log"

  # 阻塞, 直到 bridge 退出
  wait "$BRIDGE_PID" 2>/dev/null
  log "(supervise) bridge 已退出, 清理子进程..."
  _kill_supervise_children
  trap - INT TERM
}

status() {
  if is_running; then log "运行中 (bridge pid $(cat "$PIDFILE"))"; else log "未运行"; fi
  if [ -f "$FOXY" ]; then
    # shellcheck disable=SC1090,SC1091
    source "$FOXY" 2>/dev/null
    log "ROS2 /camera/color/image_raw/compressed:"
    timeout 3 ros2 topic info /camera/color/image_raw/compressed 2>&1 | head -4
  fi
}

case "${1:-start}" in
  start)  start  ;;
  supervise) supervise ;;
  stop)   stop   ;;
  restart) stop; sleep 2; start ;;
  status|info) status ;;
  *) echo "用法: $0 {start|supervise|stop|restart|status}"; exit 1 ;;
esac
