#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../../../.." && pwd)
BUILD_DIR=$(mktemp -d "${TMPDIR:-/tmp}/path-line-imu-test.XXXXXX")
trap 'rm -rf "$BUILD_DIR"' EXIT HUP INT TERM

CC=${CC:-cc}
"$CC" -std=c11 -Wall -Wextra -Wpedantic -Werror \
    -I"$SCRIPT_DIR/stubs" \
    -I"$REPO_ROOT/user/path/imu" \
    "$REPO_ROOT/user/path/imu/path_line_imu.c" \
    "$SCRIPT_DIR/test_path_line_imu.c" \
    -lm -o "$BUILD_DIR/test_path_line_imu"

"$BUILD_DIR/test_path_line_imu"
