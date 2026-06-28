#!/bin/bash
set -euo pipefail

INTERVAL_SECONDS="${1:-60}"
COUNT="${2:-0}"
PREFIX="${3:-slam_map}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_MAP_DIR="${ROSMASTER_HOST_MAP_DIR:-/home/pi/rosmaster_maps}"
SLICE_DIR="$HOST_MAP_DIR/slices"
OVERALL_NAME="overall_map"

if ! [[ "$INTERVAL_SECONDS" =~ ^[0-9]+$ ]] || [ "$INTERVAL_SECONDS" -lt 1 ]; then
    echo "Usage: $0 [interval_seconds] [count_0_means_forever] [prefix]" >&2
    exit 1
fi

if ! [[ "$COUNT" =~ ^[0-9]+$ ]]; then
    echo "Usage: $0 [interval_seconds] [count_0_means_forever] [prefix]" >&2
    exit 1
fi

mkdir -p "$HOST_MAP_DIR" "$SLICE_DIR"

echo "Starting SLAM stack if needed..."
"$TOOLS_DIR/rosmaster_slam_start.sh"

echo "Autosave settings:"
echo "  interval: ${INTERVAL_SECONDS}s"
echo "  count: $COUNT (0=forever)"
echo "  overall: $HOST_MAP_DIR/$OVERALL_NAME.*"
echo "  slices:  $SLICE_DIR/"

# 获取 PCD 文件的点数（用于防覆盖判断）
get_point_count() {
    local f="$1"
    if [ -f "$f" ]; then
        # PCD header 中 POINTS 行
        grep -m1 '^POINTS' "$f" 2>/dev/null | awk '{print $2}' || echo "0"
    else
        echo "0"
    fi
}

i=1
while true; do
    stamp="$(date +%Y%m%d_%H%M%S)"
    slice_name="${PREFIX}_${stamp}"

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Saving snapshot ${i}..."

    # 1. 先保存到临时名字
    tmp_name="_tmp_save_$$"
    "$TOOLS_DIR/rosmaster_map_save.sh" "$tmp_name" 2>&1 | tail -3

    # 2. 检查新地图点数
    new_pcd="$HOST_MAP_DIR/${tmp_name}.pcd"
    new_points=$(get_point_count "$new_pcd")
    old_points=$(get_point_count "$HOST_MAP_DIR/${OVERALL_NAME}.pcd")

    echo "  新地图: ${new_points} 点, 旧整体: ${old_points} 点"

    # 3. 防覆盖：如果新地图比旧的小50%以上，跳过覆盖 overall
    if [ "$old_points" -gt 0 ] && [ "$new_points" -gt 0 ]; then
        threshold=$((old_points / 2))
        if [ "$new_points" -lt "$threshold" ]; then
            echo "  ⚠ 新地图(${new_points})远小于旧地图(${old_points})，跳过覆盖 overall（可能 gmapping 已重启）"
        else
            # 更新 overall
            for ext in pgm yaml pcd; do
                if [ -f "$HOST_MAP_DIR/${tmp_name}.${ext}" ]; then
                    cp "$HOST_MAP_DIR/${tmp_name}.${ext}" "$HOST_MAP_DIR/${OVERALL_NAME}.${ext}"
                fi
            done
            echo "  overall 已更新"
        fi
    else
        # 首次保存或旧文件不存在，直接覆盖
        for ext in pgm yaml pcd; do
            if [ -f "$HOST_MAP_DIR/${tmp_name}.${ext}" ]; then
                cp "$HOST_MAP_DIR/${tmp_name}.${ext}" "$HOST_MAP_DIR/${OVERALL_NAME}.${ext}"
            fi
        done
        echo "  overall 已更新（首次）"
    fi

    # 4. 切片始终保存（带时间戳）
    for ext in pgm yaml pcd; do
        if [ -f "$HOST_MAP_DIR/${tmp_name}.${ext}" ]; then
            mv "$HOST_MAP_DIR/${tmp_name}.${ext}" "$SLICE_DIR/${slice_name}.${ext}"
        fi
    done
    echo "  slice: $SLICE_DIR/${slice_name}.*"

    # 5. 文件大小
    ls -lh "$HOST_MAP_DIR/${OVERALL_NAME}.pcd" "$SLICE_DIR/${slice_name}.pcd" 2>/dev/null | awk '{print "  " $5 " " $NF}'

    if [ "$COUNT" -ne 0 ] && [ "$i" -ge "$COUNT" ]; then
        echo "Autosave finished after $COUNT snapshots."
        break
    fi

    i=$((i + 1))
    sleep "$INTERVAL_SECONDS"
done
