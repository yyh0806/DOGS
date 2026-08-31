#!/usr/bin/env bash
# 获取 ros2_livox_simulation 扫描模式表 (git 已排除, 见 src/VENDORED.md)
# 用法:
#   fetch_livox_scan_modes.sh            # 只取本项目用到的 mid360.csv (16MB)
#   fetch_livox_scan_modes.sh --all      # 全部 7 个模式 (164MB)
set -euo pipefail

BASE_URL="https://raw.githubusercontent.com/stm32f303ret6/livox_laser_simulation_RO2/master/scan_mode"
DEST="$(cd "$(dirname "$0")/.." && pwd)/src/ros2_livox_simulation/scan_mode"

ALL_MODES=(avia.csv HAP.csv horizon.csv mid360.csv mid40.csv mid70.csv tele.csv)
if [[ "${1:-}" == "--all" ]]; then
  FILES=("${ALL_MODES[@]}")
else
  FILES=(mid360.csv)
fi

mkdir -p "$DEST"
for f in "${FILES[@]}"; do
  if [[ -s "$DEST/$f" ]]; then
    echo "已存在, 跳过: $f"
    continue
  fi
  echo "下载: $f"
  curl -fL --retry 3 --progress-bar -o "$DEST/$f.tmp" "$BASE_URL/$f"
  mv "$DEST/$f.tmp" "$DEST/$f"
done
echo "完成 -> $DEST"
ls -lh "$DEST"
