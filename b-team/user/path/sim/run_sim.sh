#!/usr/bin/env bash
# 编译并运行全路线物理闭环仿真（复用 tests/stubs 的存根头文件，
# 链接工程真实的 path.c / path_map.c / path_localization.c / path_safety.c）。
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
out_dir="${1:-$repo_root/user/path/sim/results}"
binary="${TMPDIR:-/tmp}/taisheng_path_sim"

mkdir -p "$out_dir"

cc -std=c11 -Wall -Wextra -fsanitize=address,undefined \
  -fno-omit-frame-pointer -g \
  -I"$repo_root/user/path/tests/stubs" \
  -I"$repo_root/user/path" \
  "$repo_root/user/path/path.c" \
  "$repo_root/user/path/path_map.c" \
  "$repo_root/user/path/path_localization.c" \
  "$repo_root/user/path/path_safety.c" \
  "$repo_root/user/path/sim/sim_route.c" \
  -lm -o "$binary"

echo "================ 常规侧 · 手动全速（去程） ================"
"$binary" --csv="$out_dir/normal_manual.csv" | tee "$out_dir/normal_manual.txt"
echo
echo "================ 常规侧 · 全自动（无遥控，上电5s自启） ================"
"$binary" --auto --csv="$out_dir/normal_auto.csv" | tee "$out_dir/normal_auto.txt"
echo
echo "================ 镜像侧 · 手动谨慎（去程） ================"
"$binary" --mirrored --careful --csv="$out_dir/mirrored_manual.csv" | tee "$out_dir/mirrored_manual.txt"
echo
echo "================ 镜像侧 · 全自动（无遥控，上电5s自启） ================"
"$binary" --auto --mirrored --csv="$out_dir/mirrored_auto.csv" | tee "$out_dir/mirrored_auto.txt"
