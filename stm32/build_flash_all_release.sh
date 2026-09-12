#!/usr/bin/env bash

set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly BOOT_BUILD="${PROJECT_DIR}/bootloader/build/Release"
readonly APP_BUILD="${PROJECT_DIR}/build/Release"
readonly METADATA_BIN="${APP_BUILD}/app_metadata.bin"

export PATH="/usr/bin:/bin"

flash=false
if [[ "${1:-}" == "--flash" ]]; then
    flash=true
elif [[ $# -ne 0 ]]; then
    echo "用法：$0 [--flash]" >&2
    exit 2
fi

for tool in cmake ninja arm-none-eabi-gcc arm-none-eabi-objcopy python3; do
    if [[ ! -x "/usr/bin/${tool}" ]]; then
        echo "错误：缺少 Debian 工具 /usr/bin/${tool}" >&2
        exit 1
    fi
done

echo "==> Release 构建 CAN Bootloader（链接区上限固定 8 KB）"
/usr/bin/cmake --fresh --preset Release -S "${PROJECT_DIR}/bootloader"
/usr/bin/cmake --build "${BOOT_BUILD}" --parallel

echo "==> Release 构建 Application（链接区上限固定 23 KB）"
/usr/bin/cmake --preset Release -S "${PROJECT_DIR}"
/usr/bin/cmake --build --preset Release

echo "==> 生成 1 KB Application metadata"
/usr/bin/python3 "${PROJECT_DIR}/tools/pack_stm32_app.py" \
    "${APP_BUILD}/project.bin" "${METADATA_BIN}" --version 1

if [[ "${flash}" != true ]]; then
    echo "==> 构建和打包完成，未访问烧录器；手动烧录时重新运行：$0 --flash"
    exit 0
fi

if [[ ! -x /usr/bin/openocd ]]; then
    echo "错误：缺少 Debian 工具 /usr/bin/openocd" >&2
    exit 1
fi

echo "==> 烧录 Bootloader、Application 和 metadata，并逐项 verify"
/usr/bin/openocd \
    -f interface/cmsis-dap.cfg \
    -c "transport select swd" \
    -f target/stm32f1x.cfg \
    -c "adapter speed 1000" \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase ${BOOT_BUILD}/stm32_can_bootloader.hex" \
    -c "verify_image ${BOOT_BUILD}/stm32_can_bootloader.hex" \
    -c "flash write_image erase ${APP_BUILD}/project.hex" \
    -c "verify_image ${APP_BUILD}/project.hex" \
    -c "flash write_image erase ${METADATA_BIN} 0x08007C00 bin" \
    -c "verify_image ${METADATA_BIN} 0x08007C00 bin" \
    -c "reset run" \
    -c "shutdown"
