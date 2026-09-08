#!/usr/bin/env bash

set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly BUILD_PRESET="Debug"

export PATH="/usr/bin:/bin"

for tool in cmake ninja arm-none-eabi-gcc arm-none-eabi-g++; do
    if [[ ! -x "/usr/bin/${tool}" ]]; then
        echo "错误：缺少 Debian 工具 /usr/bin/${tool}" >&2
        exit 1
    fi
done

cd "${PROJECT_DIR}"

echo "==> 配置 ${BUILD_PRESET} 工程"
/usr/bin/cmake --preset "${BUILD_PRESET}"

echo "==> 仅编译 ${BUILD_PRESET} 固件（不烧录）"
/usr/bin/cmake --build --preset "${BUILD_PRESET}" --parallel

echo "==> ${BUILD_PRESET} 编译完成，未执行烧录"
