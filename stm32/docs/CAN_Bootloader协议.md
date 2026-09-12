# STM32F103C6 CAN Bootloader 协议

Flash 固定为 Bootloader `0x08000000..0x08001FFF`（8 KB）、Application
`0x08002000..0x08007BFF`（23 KB）、metadata
`0x08007C00..0x08007FFF`（1 KB）。Flash page 为 1 KB。

29-bit 扩展帧 ID 的低 26 bit 是现有 FNV-1a `device_id`。消息类型为：

- `0x0C000000 | device_id`：BOOT_CTRL
- `0x10000000 | device_id`：BOOT_DATA
- `0x14000000 | device_id`：BOOT_RESP

多字节字段均为 little-endian。协议版本为 1。

| 帧 | payload |
|---|---|
| ENTER | `01 01 07 B0 AD 10` |
| BEGIN | `02 01 size:u16 crc32:u32` |
| DATA | `sequence:u16 data:1..6`；非末帧必须为 6 字节 |
| END | `03 version:u32` |
| ABORT | `04` |
| READY | `80 01` |
| ACK | `81 sequence:u16` |
| NACK | `82 error expected_sequence:u16` |
| SUCCESS | `83 00` |
| CRC_ERROR | `84 actual_crc32:u32` |
| ERROR | `85 error` |

CRC 使用 CRC-32/IEEE reflected：poly `0xEDB88320`、init/xorout 均为
`0xFFFFFFFF`，结果与 Python `zlib.crc32()` 一致。BEGIN 会先擦除 metadata，
再擦除本次镜像覆盖的 Application pages。END 重新读取完整镜像校验 CRC，随后先写
size/CRC/version，最后写 metadata magic `0x31505041`。

Linux 参考升级命令：

```bash
python3 -m pip install python-can
sudo ip link set can0 up type can bitrate 1000000
python3 tools/can_flash_stm32.py --channel can0 \
  --device-id 0x123456 --bin build/Release/project.bin --version 1
```

首次 SWD 生产烧录运行 `./build_flash_all_release.sh --flash`。脚本使用 HEX 烧录
Bootloader/Application，避免某些 ELF loader 按向下对齐的 LOAD segment 写入分区外；
metadata 使用 BIN 并显式指定 `0x08007C00`。不带 `--flash` 时只构建和打包。
