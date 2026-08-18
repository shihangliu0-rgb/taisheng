#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../../.." && pwd)"
pure_binary="${TMPDIR:-/tmp}/taisheng_path_host_tests"
runtime_binary="${TMPDIR:-/tmp}/taisheng_path_runtime_host_tests"

common_flags=(
  -std=c11 -Wall -Wextra -Werror -pedantic
  -fsanitize=address,undefined -fno-omit-frame-pointer
)

cc "${common_flags[@]}" \
  -I"$repo_root/user/path" \
  "$repo_root/user/path/path_map.c" \
  "$repo_root/user/path/path_localization.c" \
  "$repo_root/user/path/path_safety.c" \
  "$repo_root/user/path/tests/test_path.c" \
  -lm -o "$pure_binary"

cc "${common_flags[@]}" \
  -I"$repo_root/user/path/tests/stubs" \
  -I"$repo_root/user/path" \
  "$repo_root/user/path/path.c" \
  "$repo_root/user/path/path_map.c" \
  "$repo_root/user/path/path_localization.c" \
  "$repo_root/user/path/path_safety.c" \
  "$repo_root/user/path/tests/test_path_runtime.c" \
  -lm -o "$runtime_binary"

"$pure_binary"
"$runtime_binary"
rm -f "$pure_binary" "$runtime_binary"
