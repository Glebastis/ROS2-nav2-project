#!/usr/bin/env bash
# record_demo.sh — запускает bringup + запись rosbag + запись видео.
# Никак не меняет конфиги. Всё пишет в data/demo_output/<timestamp>/.
#
# Использование:
#   ./scripts/record_demo.sh
#   WAYPOINTS="[0.0,0.0,2.5,-1.5,5.0,1.5,10.0,0.0]" ./scripts/record_demo.sh
#   IMPL=cpp ./scripts/record_demo.sh
#   DISPLAY=:0 VIDEO_SIZE=3440x1440 ./scripts/record_demo.sh
#
# Останов: Ctrl+C — скрипт погасит ros2 launch, ros2 bag и ffmpeg.

set -euo pipefail

# ---------- параметры (переопределяются через env) ----------
IMPL="${IMPL:-py}"                                   # py | cpp
WAYPOINTS="${WAYPOINTS:-[0.0,0.0,2.5,-1.5,5.0,1.5,10.0,0.0]}"
PARAMS_FILE="${PARAMS_FILE:-}"                       # пусто = дефолт из launch
LAUNCH_PKG="${LAUNCH_PKG:-my_robot_bringup}"
LAUNCH_FILE="${LAUNCH_FILE:-bringup.launch.xml}"
DISPLAY="${DISPLAY:-:0}"                             # для ffmpeg x11grab
VIDEO_SIZE="${VIDEO_SIZE:-3440x1440}"
VIDEO_FPS="${VIDEO_FPS:-30}"
BAG_TOPICS="${BAG_TOPICS:-/scan_lidar /odom /tf /tf_static /map /cmd_vel /plan /local_plan /amcl_pose /gps/fix /imu/data /front_ultrasonic/range /left_ultrasonic/range /right_ultrasonic/range}"
OUTPUT_ROOT="${OUTPUT_ROOT:-data/demo_output}"
NAV2_WAIT_TIMEOUT="${NAV2_WAIT_TIMEOUT:-60}"         # сколько секунд ждать /bt_navigator

# ---------- окружение ROS ----------
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[record_demo] ros2 не найден в PATH. Сначала: source /opt/ros/humble/setup.bash && source install/setup.bash" >&2
    exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "[record_demo] ffmpeg не найден. Установите: sudo apt install ffmpeg" >&2
    exit 1
fi

# ---------- папка прогона ----------
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${OUTPUT_ROOT}/${STAMP}"
mkdir -p "${RUN_DIR}"
echo "[record_demo] output: ${RUN_DIR}"

# ---------- PID-ы процессов ----------
LAUNCH_PID=""
BAG_PID=""
FFMPEG_PID=""

# ---------- корректная остановка одного процесса ----------
stop_proc() {
    local pid="$1"
    local name="$2"
    [[ -z "${pid}" ]] && return 0
    kill -0 "${pid}" 2>/dev/null || { echo "[record_demo] ${name} уже остановлен"; return 0; }

    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do        # до 3 с
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
        sleep 0.5
    fi
    if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
    echo "[record_demo] ${name} остановлен"
}

# ---------- cleanup с защитой от повторного входа ----------
CLEANING_UP=0
cleanup() {
    if [[ "${CLEANING_UP}" -eq 1 ]]; then
        return
    fi
    CLEANING_UP=1
    trap - INT TERM
    echo ""
    echo "[record_demo] остановка..."
    # Порядок: bag → ffmpeg → launch.
    # Сначала bag, чтобы rosbag2 успел закрыть БД и записать metadata.yaml.
    stop_proc "${BAG_PID}"    "rosbag"
    stop_proc "${FFMPEG_PID}" "ffmpeg"
    stop_proc "${LAUNCH_PID}" "launch"
    echo "[record_demo] готово. Файлы: ${RUN_DIR}"
    exit 0
}
trap cleanup INT TERM

# ---------- 1. bringup ----------
LAUNCH_ARGS=("impl:=${IMPL}" "waypoints:=${WAYPOINTS}")
if [[ -n "${PARAMS_FILE}" ]]; then
    LAUNCH_ARGS+=("params_file:=${PARAMS_FILE}")
fi

echo "[record_demo] запуск: ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE} ${LAUNCH_ARGS[*]}"
ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" \
    > "${RUN_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

# ---------- 2. ожидание Nav2 ----------
echo "[record_demo] ожидание Nav2 (узел /bt_navigator, до ${NAV2_WAIT_TIMEOUT} с)..."
NAV2_READY=0
for _ in $(seq 1 $((NAV2_WAIT_TIMEOUT * 2))); do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[record_demo] launch упал на старте. См. ${RUN_DIR}/launch.log" >&2
        exit 1
    fi
    if ros2 node list 2>/dev/null | grep -q '^/bt_navigator$'; then
        NAV2_READY=1
        break
    fi
    sleep 0.5
done
if [[ "${NAV2_READY}" -ne 1 ]]; then
    echo "[record_demo] Nav2 не поднялся за ${NAV2_WAIT_TIMEOUT} с. Продолжаем всё равно." >&2
fi
# небольшая пауза, чтобы costmap'ы и TF успели стабилизироваться
sleep 3

# ---------- 3. rosbag ----------
echo "[record_demo] запись rosbag..."
ros2 bag record -o "${RUN_DIR}/demo_bag" ${BAG_TOPICS} \
    > "${RUN_DIR}/bag.log" 2>&1 &
BAG_PID=$!
sleep 2
if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "[record_demo] rosbag не стартовал. См. ${RUN_DIR}/bag.log" >&2
    cleanup
fi

# ---------- 4. видео ----------
echo "[record_demo] запись видео (${VIDEO_SIZE}@${VIDEO_FPS}, display=${DISPLAY})..."
ffmpeg -y -loglevel warning \
    -f x11grab -video_size "${VIDEO_SIZE}" -framerate "${VIDEO_FPS}" -i "${DISPLAY}" \
    -c:v libx264 -preset ultrafast -pix_fmt yuv420p \
    "${RUN_DIR}/demo.mp4" \
    > "${RUN_DIR}/ffmpeg.log" 2>&1 &
FFMPEG_PID=$!
sleep 1
if ! kill -0 "${FFMPEG_PID}" 2>/dev/null; then
    echo "[record_demo] ffmpeg не стартовал. См. ${RUN_DIR}/ffmpeg.log" >&2
    cleanup
fi

# ---------- ожидание ----------
echo "[record_demo] идёт запись. Ctrl+C — остановить и сохранить."
# || true — чтобы set -e не убил скрипт, если wait вернёт非-ноль после сигнала
wait "${LAUNCH_PID}" || true

# Если launch завершился сам (робот доехал, что-то упало) — тоже корректно всё гасим
cleanup