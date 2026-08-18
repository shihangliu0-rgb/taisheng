#!/usr/bin/env bash
# 编译并运行全路线物理闭环仿真。
# 链接工程真实的 path.c / path_map.c / path_localization.c /
# path_safety.c / path_line_imu.c；不把真值位姿写回控制层。
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
out_dir="${1:-$repo_root/user/path/sim/results}"
binary="${TMPDIR:-/tmp}/taisheng_path_sim"

mkdir -p "$out_dir"

cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined \
  -fno-omit-frame-pointer -g \
  -I"$repo_root/user/path/sim/stubs" \
  -I"$repo_root/user/path" \
  -I"$repo_root/user/path/imu" \
  "$repo_root/user/path/path.c" \
  "$repo_root/user/path/path_map.c" \
  "$repo_root/user/path/path_localization.c" \
  "$repo_root/user/path/path_safety.c" \
  "$repo_root/user/path/imu/path_line_imu.c" \
  "$repo_root/user/path/sim/sim_route.c" \
  -lm -o "$binary"

status=0
run_one() {
  local title="$1"
  local log="$2"
  shift 2
  echo "================ $title ================"
  if ! "$@" | tee "$log"; then
    echo "**** $title 失败 ****"
    status=1
  fi
  echo
}

run_one "常规侧 · 手动全速（去程）" "$out_dir/normal_manual.txt" \
  "$binary" --csv="$out_dir/normal_manual.csv"

run_one "常规侧 · 全自动（无遥控，上电5s自启）" "$out_dir/normal_auto.txt" \
  "$binary" --auto --csv="$out_dir/normal_auto.csv"

exit "$status"
