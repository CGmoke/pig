#!/usr/bin/env bash
# Run from anywhere. Official models are mounted read-only; only the result directory is writable.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project=$(cd -- "$here/../.." && pwd)
scene=$(realpath "${BAJIE_SIM_DIR:?Set BAJIE_SIM_DIR to the complete official model directory}")
urdf=$(realpath "${BAJIE_URDF:?Set BAJIE_URDF to the expanded official URDF}")
out=$(realpath "${Q3_RESULT_DIR:-$project/answers}")
[[ -d $out && -f $urdf && -f $scene/scene/abr_scene.usd && -d $scene/models ]] || { echo 'Missing output directory, official USD/URDF, or models directory' >&2; exit 2; }
name=bajie-q3-safe
available() { awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo; }
gpu_free() { timeout 3s nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | awk 'NR==1 {v=$1} END {print v}'; }
# Keep at least 4.5 GiB of host headroom even if the container reaches its full budget.
[[ $(available) -ge 12800 ]] || { echo '未启动：系统可用内存不足 12800 MiB，请先保存并关闭不必要的程序。' >&2; exit 1; }
gpu=$(gpu_free) || gpu=0
[[ $gpu =~ ^[0-9]+$ && $gpu -ge 6000 ]] || { echo '未启动：显存余量不足 6000 MiB，或无法读取显卡状态。' >&2; exit 1; }
args=(--output /result/q3_demo_output.jsonl)
if (($#)); then
    [[ $# == 2 && $1 == --case && $2 =~ ^(rubik_cube|dinosaur|soft_toy)$ ]] || { echo 'Usage: run_laptop.sh [--case rubik_cube|dinosaur|soft_toy]' >&2; exit 2; }
    args=("$@")
fi
if [[ ${Q3_PREVIEW:-0} == 1 ]]; then
    args+=(--preview /result/frame.json)
fi
cid=$(docker create --name "$name" --gpus all --network none --memory 8g --memory-swap 8g --cpus 4 --pids-limit 768 \
    --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=53 \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e NVIDIA_DRIVER_CAPABILITIES=all -e OMP_NUM_THREADS=2 \
    -e OPENBLAS_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
    --mount "type=bind,src=$project,dst=/project,readonly" \
    --mount "type=bind,src=$scene,dst=/scene,readonly" \
    --mount "type=bind,src=$urdf,dst=/model.urdf,readonly" \
    --mount "type=bind,src=$out,dst=/result" \
    "${Q3_IMAGE:-bajie/sim:humble45}" bash -c 'source /opt/ros/humble/setup.bash && exec /isaac-sim/python.sh -u "$@"' _ \
    /project/src/question3_project/run_touch.py --scene /scene/scene/abr_scene.usd --urdf /model.urdf \
    --input /project/inputs/q3_demo_input.jsonl "${args[@]}")
watcher=
cleanup() {
    [[ -z $watcher ]] || kill "$watcher" 2>/dev/null || true
    # Only this script's container, never any unrelated job.
    docker stop -t 2 "$cid" >/dev/null 2>&1 || true
    docker rm "$cid" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
(
    start=$SECONDS; low=0
    while sleep 2; do
        status=$(docker inspect -f '{{.State.Status}}' "$cid" 2>/dev/null) || break
        [[ $status == running || $status == created ]] || break
        avail=$(available)
        gpu=$(gpu_free) || gpu=0
        [[ $gpu =~ ^[0-9]+$ ]] || gpu=0
        if [[ $avail -lt 4500 || $gpu -lt 500 ]]; then ((low+=1)); else low=0; fi
        printf '[watchdog] host_available=%s MiB gpu_free=%s MiB\n' "$avail" "$gpu"
        if [[ $low -ge 2 || $((SECONDS-start)) -ge 360 ]]; then
            echo '[watchdog] 内存、显存或运行时间达到停止线，正在终止仿真。'
            docker stop -t 2 "$cid" >/dev/null || true
            break
        fi
    done
) & watcher=$!
rc=0
docker start -a "$cid" || rc=$?
docker inspect -f '[container] exit={{.State.ExitCode}} oom={{.State.OOMKilled}}' "$cid"
exit "$rc"
