#!/usr/bin/env bash

set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly BUILD_PRESET="Release"

export PATH="/usr/bin:/bin"

for tool in cmake ninja arm-none-eabi-gcc openocd lsusb; do
    if [[ ! -x "/usr/bin/${tool}" ]]; then
        echo "错误：缺少 Debian 工具 /usr/bin/${tool}" >&2
        exit 1
    fi
done

if ! /usr/bin/lsusb -d 0416:5051 >/dev/null; then
    echo "错误：未检测到 DAPLink（USB VID:PID 0416:5051）" >&2
    exit 1
fi

cd "${PROJECT_DIR}"

echo "==> 配置 ${BUILD_PRESET} 工程"
/usr/bin/cmake --preset "${BUILD_PRESET}"

echo "==> 编译并烧录 ${BUILD_PRESET} 固件"
/usr/bin/cmake --build --preset "${BUILD_PRESET}" --parallel --target flash

echo "==> ${BUILD_PRESET} 编译、烧录、校验和复位全部完成"
