# STM32 单电机 FOC 通信协议说明书

> 文档版本：STM32-CAN-1.0
>
> 源码快照：`main` / `a935eb4`，2026-09-12
>
> 适用对象：STM32F103C6T6A 单电机 FOC 固件及其 CAN 控制端

## 0. 先读结论

本固件对外定义的是一套基于 CAN 的最小通信协议，包含三类业务：

1. 控制端用标准帧 `0x700` 广播设备发现。
2. STM32 用扩展帧返回自己的动态 `device_id`。
3. 控制端向指定扩展 ID 发送扭矩目标，STM32 周期性返回编码器反馈。

现行帧定义如下：

| 业务 | 方向 | ID 类型 | CAN ID | DLC | 数据 |
|---|---|---:|---|---:|---|
| 设备发现请求 | 控制端 → STM32 | 标准 | `0x700` | 0 | 无 |
| 设备发现响应 | STM32 → 控制端 | 扩展 | `0x08000000 OR device_id` | 0 | 无 |
| 扭矩目标 | 控制端 → STM32 | 扩展 | `0x00000000 OR device_id` | 4 | 小端 IEEE-754 `float32`，单位 N·m |
| 编码器反馈 | STM32 → 控制端 | 扩展 | `0x04000000 OR device_id` | 8 | `uint16 + uint16 + int32`，小端 |

本文描述的是当前代码已经实现的行为，不把规划中的状态帧、命令序号、CRC、速度字段或健康心跳写成现行协议。

---

## 1. 协议范围与术语

### 1.1 范围

本书覆盖：

- STM32F103C6T6A 的 CAN1 链路参数和收发方向。
- 设备发现、设备 ID 分配、扭矩命令和编码器反馈。
- 扭矩命令从 CAN 线到 FOC 的数据语义。
- 编码器反馈字段的来源、单位、端序和回绕规则。
- AS5600 I2C 采集作为内部数据源时的接口约定。
- 控制端的解析示例、调试方法和当前实现限制。

协议实现入口是 [`user_lib/devices/io/can_comm.cpp`](../user_lib/devices/io/can_comm.cpp)，公开接口见 [`can_comm.h`](../user_lib/devices/io/can_comm.h)。

### 1.2 术语

| 名称 | 含义 |
|---|---|
| 控制端 | ESP32、上位机或其他负责发现设备、下发扭矩的 CAN 节点 |
| STM32 节点 | 本书描述的 STM32F103 单电机控制器 |
| `D` | 26 位非零 `device_id` |
| 标准帧 | 11 位 CAN ID，代码中为 `CAN_ID_STD` |
| 扩展帧 | 29 位 CAN ID，代码中为 `CAN_ID_EXT` |
| DLC | CAN Data Length Code，当前只使用 0、4、8 |
| `full_count` | AS5600 计数展开后的累计机械计数 |
| `sequence` | 编码器反馈发送尝试序号，不是编码器采样序号 |

### 1.3 线上的通用规则

- 所有业务帧都是数据帧，`RTR = CAN_RTR_DATA`。
- 多字节整数按小端序排列。
- 扭矩输入是 IEEE-754 binary32 的原始 4 字节，不是 ASCII 字符串，也不是定点整数。
- CAN 应使用自身的数据帧校验机制；当前业务层没有额外校验和或 CRC 字段。
- 当前协议没有版本字段。协议发生不兼容变更时，应通过固件/控制端版本管理，而不是从现有帧中猜版本。

---

## 2. CAN 链路与硬件边界

### 2.1 CAN 参数

当前编译进固件的 CAN1 初始化位于 [`Core/Src/can.c`](../Core/Src/can.c#L40)：

| 参数 | 当前值 |
|---|---:|
| 外设 | CAN1 |
| 工作模式 | `CAN_MODE_NORMAL` |
| 波特率 | 1 Mbit/s |
| Prescaler | 2 |
| SJW | 1 TQ |
| BS1 | 11 TQ |
| BS2 | 4 TQ |
| 总 TQ | 16 TQ/bit |
| 自动 Bus-Off | 开启 |
| 自动唤醒 | 开启 |
| 自动重发 | 开启 |
| 接收 FIFO | FIFO0 |

系统时钟为 64 MHz，APB1 为 32 MHz，因此 CAN 位时间为：

```text
CAN 时钟 / Prescaler = 32 MHz / 2 = 16 MHz
每位 TQ = 1 + 11 + 4 = 16
波特率 = 16 MHz / 16 = 1 Mbit/s
```

控制端必须使用相同的 1 Mbit/s 配置。链路处于正常模式，物理总线仍需要正确的 CAN 收发器、供电、共地和终端匹配；这些不属于应用帧格式。

### 2.2 引脚与中断

| 功能 | STM32 引脚/中断 |
|---|---|
| CAN_RX | PA11 |
| CAN_TX | PA12 |
| CAN 接收中断 | `USB_LP_CAN1_RX0_IRQn` |
| 接收中断优先级 | 2 |
| 接收处理入口 | `HAL_CAN_RxFifo0MsgPendingCallback()` |

引脚配置见 [`Core/Src/can.c`](../Core/Src/can.c#L62)，中断转发见 [`Core/Src/stm32f1xx_it.c`](../Core/Src/stm32f1xx_it.c#L217)。CAN 接收不是由主循环轮询，而是由 FIFO0 消息挂起中断触发。

### 2.3 STM32 过滤器

`can_comm::init()` 配置两个精确匹配过滤器，均送入 FIFO0：

| Filter Bank | 接受内容 | 匹配条件 |
|---:|---|---|
| 0 | 设备发现请求 | 标准帧、ID `0x700`、数据帧、DLC 0 |
| 1 | 本节点扭矩命令 | 扩展帧、ID `command_id`、数据帧 |

过滤器同时约束 ID 类型和 RTR 位。因此，发送到其他节点的命令、使用错误 ID 类型的帧、远程帧不会作为本协议消息交给应用层。扩展命令即使通过 ID 过滤，软件仍会再次检查 `IDE`、`RTR`、ID 和 DLC。

---

## 3. 扩展 ID 规划

### 3.1 位布局

扩展 ID 使用 29 位，布局如下：

```text
bit 28                         bit 26 bit 25                 bit 0
┌──────────── 消息类型 ────────────┬──────── device_id ──────────┐
│              3 bit              │            26 bit            │
└─────────────────────────────────┴──────────────────────────────┘
```

解析公式：

```text
message_type = (extended_id >> 26) & 0x07
device_id    =  extended_id        & 0x03FFFFFF
```

当前已定义的类型：

| `message_type` | 十六进制基址 | 业务 | 方向 |
|---:|---:|---|---|
| `0b000` | `0x00000000` | 扭矩命令 | 控制端 → STM32 |
| `0b001` | `0x04000000` | 编码器反馈 | STM32 → 控制端 |
| `0b010` | `0x08000000` | 设备发现响应 | STM32 → 控制端 |
| `0b011`～`0b111` | 未定义 | 保留 | 不应使用 |

因此，给定设备 ID `D` 时：

```text
command_id  = 0x00000000 | D
feedback_id = 0x04000000 | D
response_id = 0x08000000 | D
```

例如 `D = 0x00123456` 时：

```text
扭矩命令 ID = 0x00123456
反馈 ID     = 0x04123456
响应 ID     = 0x08123456
```

### 3.2 `device_id` 生成

STM32 不把芯片 UID 原值直接放入 CAN ID，而是把 STM32F103 的 96 位 Unique Device ID 哈希为 26 位：

1. 从地址 `0x1FFFF7E8` 连续读取 12 个字节。
2. 使用 FNV-1a：

   ```text
   hash = 2166136261
   for byte in uid[0..11]:
       hash = hash XOR byte
       hash = hash * 16777619  (mod 2^32)
   ```

3. `D = hash & 0x03FFFFFF`。
4. 若结果为 0，则改为 1；0 不作为有效设备 ID。

这段逻辑位于 [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L51)。控制端通常不需要自行计算 `D`，直接发送发现请求并从响应 ID 提取即可。

该算法在单个芯片上是确定性的，但哈希理论上存在碰撞可能。当前协议没有碰撞检测、重新分配或配置持久化机制；如果系统部署多个节点，应在装配和调试阶段确认响应 ID 不重复。

---

## 4. 帧格式定义

### 4.1 设备发现请求

控制端发送：

| 字段 | 值 |
|---|---|
| ID 类型 | 标准帧 |
| CAN ID | `0x700` |
| RTR | 数据帧 |
| DLC | `0` |
| Data | 无 |

STM32 只有在 `IDE = CAN_ID_STD`、`StdId = 0x700`、`RTR = CAN_RTR_DATA`、`DLC = 0` 同时成立时才执行发现响应。

发现请求是广播请求，不携带目标设备 ID。总线上所有已成功初始化 CAN 且匹配过滤器的 STM32 节点都可能响应。

### 4.2 设备发现响应

每个节点响应：

| 字段 | 值 |
|---|---|
| ID 类型 | 扩展帧 |
| CAN ID | `0x08000000 OR D` |
| RTR | 数据帧 |
| DLC | `0` |
| Data | 无 |

设备身份全部编码在扩展 ID 中，因此响应没有 payload。控制端收到扩展帧且 `message_type == 2`、DLC 为 0 时，可按下式得到设备 ID：

```text
D = response_id & 0x03FFFFFF
```

STM32 在接收中断回调内直接尝试发送响应。重复发送发现请求会产生重复响应；控制端应按需限频并按 ID 去重。

### 4.3 扭矩目标命令

控制端向某个已发现节点发送：

| 字段 | 值 |
|---|---|
| ID 类型 | 扩展帧 |
| CAN ID | `D`，即 `0x00000000 OR D` |
| RTR | 数据帧 |
| DLC | `4` |
| Data[0..3] | IEEE-754 binary32，小端，单位 N·m |

payload 定义：

| 字节偏移 | 长度 | 类型 | 含义 |
|---:|---:|---|---|
| 0 | 4 | `float32` | 目标扭矩 `torque_Nm` |

STM32 的接收转换为：

```text
target_mNm = (int32_t)(torque_Nm * 1000.0f)
```

因此，控制端发送 `0.050` 表示 `0.050 N·m = 50 mN·m`，不能把 `50` 直接当作 mN·m 发送。转换结果是浮点转 `int32_t` 的截断值。

例如 `D = 0x00123456`、扭矩为 `0.050 N·m` 时，完整命令为：

```text
扩展 ID：0x00123456
DLC：    4
Data：   CD CC 4C 3D
```

当前接收器只检查帧类型、目标 ID、RTR 和 DLC，不显式检查 `NaN`、无穷大或数值范围。控制端必须只发送有限的、处于系统设计范围内的扭矩值；异常浮点值不属于协议的有效输入。

该命令没有命令序号、时间戳、应答帧或独立的“使能”字段。同一节点的多个有效命令到达后，后到的值覆盖先到的值。

### 4.4 编码器反馈

STM32 使用扩展反馈 ID 发送：

| 字段 | 值 |
|---|---|
| ID 类型 | 扩展帧 |
| CAN ID | `0x04000000 OR D` |
| RTR | 数据帧 |
| DLC | `8` |
| Data | 见下表 |

payload 布局：

| 字节偏移 | 长度 | 类型 | 端序 | 含义 |
|---:|---:|---|---|---|
| 0 | 2 | `uint16_t` | 小端 | `sequence` |
| 2 | 2 | `uint16_t` | 小端 | `timestamp_us` |
| 4 | 4 | `int32_t` | 小端 | `full_count` |

对应的 C 结构语义为：

```cpp
uint16_t sequence;
uint16_t timestamp_us;
int32_t  full_count;
```

这里使用字段逐段拷贝，而不是直接发送 C 结构体，因此协议不依赖结构体填充布局；端序仍由当前实现约定为小端。

#### `sequence`

- 类型为 `uint16_t`，从第一次调用发送函数时的 `1` 开始。
- 每次调用 `send_feedback()` 都先自增，再放入 payload。
- 到 `65535` 后按 16 位无符号数回绕到 `0`。
- 序号统计的是发送尝试，不是 AS5600 采样次数。
- 如果发送邮箱不可用导致本次发送失败，序号仍可能被消耗，控制端看到跳号不能简单等同于编码器丢采样。

#### `timestamp_us`

- 来源是 AS5600 I2C DMA 完成时的 TIM1 微秒计数。
- 线上的字段只有低 16 位，约每 `65536 us` 回绕一次。
- 它是传感器 DMA 完成时间，不是 CAN 帧发送时间，也不是控制端接收时间。
- 控制端计算相邻时间差时应使用 16 位模减法。

#### `full_count`

- AS5600 原始角度为 12 位，每机械圈 4096 个计数。
- 第一个有效样本直接作为初始累计值，通常为 `0..4095`；它不是固定从 0 开始的绝对零点。
- 后续样本按最短方向展开跨圈变化，再累计到 `int32_t`。
- 机械圈数差可按 `delta_count / 4096.0` 计算。
- 该字段是传感器机械计数，未编码电机极对数、方向校准值或 FOC 电角零点。

当前 `encoder_package` 内部还有 `speed_mrad_s`，但 `send_feedback()` 没有把速度放入 CAN payload。不要按旧的“双 float 角度/速度”格式解析现行 STM32 反馈。

例如 `sequence = 17`、`timestamp_us = 54321`、`full_count = -2048` 时，8 字节 payload 为：

```text
11 00 31 D4 00 F8 FF FF
```

### 4.5 发送成功的含义

`send_feedback()` 和设备发现响应最终调用 `HAL_CAN_AddTxMessage()`。返回 `true` 只表示消息成功加入 CAN 发送邮箱，不表示控制端已经收到，也不提供业务层 ACK。

当前没有应用层发送队列、重试计数、丢帧统计或发送失败状态上报。控制端应把反馈看作周期性状态流，通过 `sequence` 和接收超时自行判断连续性。

---

## 5. 运行时数据链路

### 5.1 启动顺序

主函数先初始化 HAL、时钟和外设，再进入应用：

```text
HAL_Init
  → SystemClock_Config
  → MX_GPIO_Init
  → MX_DMA_Init
  → MX_CAN_Init
  → MX_I2C1_Init
  → MX_TIM2_Init
  → MX_TIM1_Init
  → app_init
```

`app_init()` 的应用初始化顺序为：

```text
AS5600 init → CAN init → motor init → 创建任务
```

实际入口见 [`main.c`](../Core/Src/main.c#L69) 和 [`app.cpp`](../user_lib/devices/app.cpp#L33)。`can_comm::init()` 成功后，CAN RX FIFO0 中断开始处理发现和扭矩命令。

### 5.2 正常交互时序

```text
控制端                         STM32 节点
   │                               │
   │-- STD 0x700, DLC=0 ---------->│  设备发现广播
   │<-- EXT 0x08000000|D, DLC=0 ----│  返回 device_id
   │                               │
   │-- EXT D, DLC=4, torque -------->│  接收并覆盖最新扭矩目标
   │                               │
   │<-- EXT 0x04000000|D, DLC=8 -----│  周期编码器反馈
   │<-- EXT 0x04000000|D, DLC=8 -----│
   │              ...              │
```

### 5.3 任务和中断关系

| 路径 | 调度方式 | 当前周期/行为 |
|---|---|---|
| CAN 接收 | CAN RX FIFO0 中断 | 收到消息立即处理 |
| AS5600 更新 | `task::loop()` 查询任务 | `period_ms = 0`，主循环尽可能快地轮询 DMA 状态 |
| 编码器反馈 | `task::loop()` 查询任务 | `period_ms = 2`，名义约 500 Hz |
| FOC 更新 | TIM2 CH4 中断 | CH4 事件交替跳过一次，名义约为 PWM 频率的一半 |

反馈任务位于 [`app.cpp`](../user_lib/devices/app.cpp#L21)，任务调度规则位于 [`task.cpp`](../user_lib/devices/task.cpp#L127)。2 ms 是调度目标，不是严格的总线发送周期保证；主循环负载、CAN 邮箱和总线仲裁都会影响实际反馈到达时间。

### 5.4 扭矩命令如何影响 FOC

收到命令后，CAN ISR 只保存最新 `target_mNm` 和接收时刻。FOC 更新时读取它，并执行：

```text
CAN float32 torque_Nm
    → int32 torque_mNm
    → torque_uq
    → 加上速度相关的 BEMF 补偿
    → SVPWM
    → TIM2 CCR1 / CCR2 / CCR3
```

当前路径是直接的电压型 `Uq` 控制输入，不是带扭矩传感器的闭环扭矩控制，也不是速度环或位置环。协议字段名使用“目标扭矩”是系统语义，不能据此宣称已经提供精确的机械扭矩闭环。

### 5.5 命令超时

`can_comm::get_target_mNm()` 规定：距最近一次有效扭矩帧超过 100 ms 时，返回 `0`。这是一项目标值看门狗：

```text
if (now_ms - latest_target_time > 100 ms):
    target_mNm = 0
```

必须注意：

- 超时只把扭矩目标清零，不是 CAN 层 ACK，也不是完整的驱动器失能协议。
- 当前 `motor::update()` 仍会计算速度相关的 BEMF 补偿，因此“目标扭矩为零”不能被解释为“所有 PWM 已停止”。
- 当前编码器包只有“是否曾经有效”的判断，没有基于包时间戳的运行时新鲜度超时；I2C DMA 停止后，缓存数据可能继续被读取。
- 协议没有传感器故障、驱动器状态、过温或 PWM 停止原因的反馈字段。

这些行为是当前实现的边界。控制端如需要更强的急停语义，应增加并实现独立的驱动禁用/故障协议，不能仅依赖现有 100 ms 目标超时。

---

## 6. AS5600 I2C 内部接口

AS5600 不是对外 CAN 协议的一部分，但它是反馈字段的直接数据源，调试协议时需要知道其边界。

| 参数 | 当前值 |
|---|---|
| I2C 外设 | I2C1 |
| 7 位设备地址 | `0x36` |
| HAL 地址参数 | `0x36 << 1` |
| 原始角度寄存器 | `0x0C`（RAW ANGLE） |
| 读取长度 | 2 字节 |
| I2C 速率 | 300 kHz |
| 读取方式 | DMA，DMA1 Channel 7 |
| SCL / SDA | PB6 / PB7 |

读取后的 12 位角度按下式拼接：

```text
raw = ((data[0] & 0x0F) << 8) | data[1]
```

跨圈处理规则：

```text
delta = raw - last_raw
if delta >  2048: delta -= 4096
if delta < -2048: delta += 4096
full_count += delta
```

内部速度估计使用毫弧度每秒 `mrad/s`，一阶滤波时间参数为 3000 us：

```text
speed_mrad_s =
    (old_speed_mrad_s * 3000 + delta * 1533981)
    / (3000 + dt_us)
```

该速度值只供 STM32 内部 FOC 使用，当前 8 字节 CAN 反馈不发送它。

---

## 7. 控制端实现示例

下面示例使用 Python 伪代码和 `python-can` 风格 API，重点是帧字段和端序；具体 CAN 驱动初始化按控制端平台调整。

### 7.1 发现节点并发送扭矩

```python
import struct
import can
import time

bus = can.Bus(
    interface="socketcan",
    channel="can0",
    bitrate=1_000_000,
)

# 1. 广播发现：标准帧、DLC=0
bus.send(can.Message(
    arbitration_id=0x700,
    is_extended_id=False,
    data=b"",
))

# 2. 收集扩展响应，提取 26 位 device_id
devices = {}
deadline = time.monotonic() + 0.2
while time.monotonic() < deadline:
    msg = bus.recv(timeout=0.02)
    if msg is None:
        continue
    if not msg.is_extended_id or len(msg.data) != 0:
        continue
    if ((msg.arbitration_id >> 26) & 0x07) != 0b010:
        continue

    device_id = msg.arbitration_id & 0x03FFFFFF
    devices[device_id] = msg.arbitration_id

# 3. 向指定节点发送 0.050 N·m
device_id = next(iter(devices))
command_id = device_id
bus.send(can.Message(
    arbitration_id=command_id,
    is_extended_id=True,
    data=struct.pack("<f", 0.050),
))
```

控制端应持续刷新目标，且刷新间隔应显著小于 100 ms。工程上可使用 10～20 ms 的发送周期，再结合总线负载和应用调度验证实际效果；这属于系统集成建议，不是帧格式中的额外字段。

### 7.2 解析反馈

```python
def decode_feedback(msg):
    if not msg.is_extended_id:
        return None
    if len(msg.data) != 8:
        return None
    if ((msg.arbitration_id >> 26) & 0x07) != 0b001:
        return None

    device_id = msg.arbitration_id & 0x03FFFFFF
    sequence, timestamp_us, full_count = struct.unpack("<HHi", msg.data)
    return {
        "device_id": device_id,
        "sequence": sequence,
        "timestamp_us": timestamp_us,
        "full_count": full_count,
    }
```

序号和时间戳都应使用模运算比较：

```python
sequence_delta = (current_sequence - previous_sequence) & 0xFFFF
time_delta_us = (current_timestamp_us - previous_timestamp_us) & 0xFFFF
```

`full_count` 是有符号 32 位累计值。若只关心相对转动，可保存上一帧并计算差值；若长时间运行，应考虑 `int32` 累计值最终回绕的处理策略。

### 7.3 C/C++ 打包规则

控制端不要直接把带填充的结构体作为 wire payload。应显式逐字段写入，或者使用明确的小端序列化函数：

```cpp
float torque_Nm = 0.050f;
uint8_t data[4];
memcpy(data, &torque_Nm, sizeof(torque_Nm));
```

这段写法只有在控制端也明确采用 IEEE-754 binary32 且为小端时才与当前 STM32 实现兼容。跨架构控制端应使用 `float` 位模式转换和显式小端写入。

---

## 8. 调试与故障定位

### 8.1 发现不到节点

按以下顺序检查：

1. 控制端和 STM32 都为 1 Mbit/s。
2. 发现帧是标准 ID `0x700`，不是扩展 ID。
3. `DLC = 0`，是数据帧而不是远程帧。
4. CAN 收发器、CANH/CANL、共地和终端电阻正确。
5. STM32 已完成 `can_comm::init()`，没有在 HAL 错误处理处停住。
6. 总线监视器能看到控制端实际发出的帧和 ACK/错误状态。

发现响应 ID 是动态的，不能预先假定为某个固定扩展 ID。

### 8.2 有响应但没有期望的反馈

发现响应至少说明 STM32 收到了符合条件的 `0x700` 帧并尝试加入发送邮箱，但它不代表编码器已有效、FOC 已启动或反馈帧已经被控制端接收。继续检查：

- 反馈 ID 的消息类型是否为 `0b001`。
- DLC 是否为 8。
- 控制端是否使用扩展帧解析。
- CAN 发送邮箱是否因总线仲裁或总线错误而失败。
- 主循环是否持续运行。

当前没有状态查询帧，因此无法仅凭 CAN 协议区分 CAN 初始化失败、编码器无数据、motor 初始化失败和控制端过滤错误。

### 8.3 反馈全为 0

当前应用层反馈任务忽略 `as5600::get_package()` 的布尔返回值，仍会调用 `send_feedback()`。因此在编码器尚无有效包时，控制端可能收到初始全零反馈；已有有效包但本轮读取失败时，则可能重复上次反馈。

这不等价于“机械角度明确为 0”，协议中也没有 `valid` 标志。控制端应结合启动阶段、反馈连续性和实际硬件状态判断。

### 8.4 `sequence` 跳号

跳号可能来自：

- CAN 发送邮箱临时没有可用位置。
- 总线仲裁、错误或控制端丢帧。
- 反馈函数被调用但 `HAL_CAN_AddTxMessage()` 没有成功。
- 16 位序号正常回绕。

它不能直接证明 AS5600 丢了相同数量的采样，因为反馈序号和采样没有一一对应关系。

### 8.5 扭矩命令没有效果

检查：

- 命令是否为扩展帧。
- ID 是否为发现得到的 `D`，而不是反馈 ID 或响应 ID。
- DLC 是否为 4。
- payload 是否为小端 `struct.pack("<f", torque_Nm)`。
- 扭矩是否在 100 ms 内持续刷新。
- 是否把 N·m 错传成 mN·m 数值。
- 电机是否完成初始化和编码器校准。

现行协议没有命令回显，因此不能通过 CAN 反馈直接确认某一个命令已被应用；只能通过反馈、总线抓包和硬件状态联合判断。

### 8.6 发热、锁死和传感器异常

协议层排查的优先级是停止风险源，而不是提高扭矩或电压：

- 立即停止发送非零扭矩命令。
- 确认命令超时只能将目标值变为 0，不能当作完整硬件急停。
- 检查 AS5600 I2C DMA 是否完成、SCL/SDA 是否被拉住，以及 `full_count` 是否持续变化。
- 检查 FOC 使用的编码器缓存是否已经陈旧。
- 在低能量、可快速断电的条件下重新验证方向、零点和 PWM。

代码构建通过只能证明主机编译链路正确，不能证明 CAN 物理链路、编码器连续采样、PWM 时序或电机平稳转动。

---

## 9. 兼容性与明确不属于现行协议的内容

### 9.1 旧的 `0x100` 双浮点调试帧

仓库中的旧 ESP32 调试代码仍保留过这样的解析路径：

```text
标准 ID 0x100，DLC=8，前 4 字节 float 角度，后 4 字节 float 速度
```

这不是当前 STM32 `can_comm.cpp` 的反馈格式。当前 STM32 反馈是：

```text
扩展 ID 0x04000000|D，DLC=8，uint16 sequence + uint16 timestamp_us + int32 full_count
```

如果控制端仍按 `0x100` 或两个 `float` 解析，会得到错误的大数或无意义的角度速度。

### 9.2 当前 ESP32 工程的状态

本书以 STM32 实现为协议源。仓库同一项目中的当前 ESP32 `lib/system/start.cpp` 仍是调试路径：它监听旧标准帧 `0x100`，并没有完整实现本书的 `0x700` 发现、动态扩展 ID 管理和扭矩下发流程。

因此，“STM32 代码已经具备本书定义的 CAN 端点”不等于“仓库内当前 ESP32 程序已经与其完全互通”。要联调，控制端必须先按本书实现现行协议，或另行编写兼容层；不能通过修改 STM32 文档来掩盖两端格式不一致。

### 9.3 UART

STM32 工程没有配置 UART/USART 应用通信协议。本书不定义串口命令、串口日志格式或 CAN-to-UART 透传格式；ESP32 的调试串口输出也不是 STM32 对外协议。

---

## 10. 联调验收清单

### 10.1 软件和静态检查

```bash
./build_debug.sh
/usr/bin/cmake --build --preset Release --parallel
git diff --check
```

构建检查只覆盖固件编译和文档变更格式，不代替烧录、总线抓包或电机测试。

### 10.2 CAN 抓包验收

至少应观察到：

1. 控制端发送标准 `0x700`、DLC 0。
2. 每个 STM32 节点返回唯一的扩展 `0x08000000 OR D`、DLC 0。
3. 控制端按返回的 `D` 向扩展 ID `D` 发送 4 字节小端 float。
4. STM32 返回扩展 `0x04000000 OR D`、DLC 8。
5. 反馈 `sequence` 持续变化，`timestamp_us` 按 16 位回绕，`full_count` 在转动时变化。
6. 停止下发后超过 100 ms，内部目标值读取应归零；不能仅以此断言 PWM 已关闭。

### 10.3 硬件验收边界

以下结论必须由实机证据支持，不能由本书或一次成功编译推导：

- CAN 收发器在目标板上正常 ACK。
- AS5600 DMA 事务持续完成且没有总线卡死。
- 反馈数据与实际机械转动方向、圈数一致。
- FOC 中断按预期运行且满足实时性。
- 电机在安全电压、电流和机械条件下连续平稳运行。

---

## 11. 源码对照表

| 协议/行为 | 实现位置 |
|---|---|
| 发现 ID、消息类型、`device_id` | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L14) |
| UID FNV-1a 哈希 | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L51) |
| bxCAN 精确过滤器 | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L81) |
| CAN 发送帧头和临界区 | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L147) |
| 扭矩接收与 100 ms 超时 | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L210) |
| 反馈 8 字节打包 | [`can_comm.cpp`](../user_lib/devices/io/can_comm.cpp#L331) |
| CAN1 位时序和引脚 | [`Core/Src/can.c`](../Core/Src/can.c#L29) |
| 应用初始化和反馈任务 | [`app.cpp`](../user_lib/devices/app.cpp#L7) |
| 编码器包定义 | [`encoder.h`](../user_lib/devices/hw/encoder.h#L6) |
| AS5600 I2C DMA 与累计计数 | [`encoder.cpp`](../user_lib/devices/hw/encoder.cpp#L24) |
| 扭矩到 FOC/SVPWM 数据链 | [`motor.cpp`](../user_lib/devices/hw/motor.cpp#L302) |
| 主函数外设启动顺序 | [`main.c`](../Core/Src/main.c#L69) |

---

## 12. 协议演进建议

下列能力当前没有实现，如未来需要，建议作为明确的新协议版本或新增消息类型设计，不要复用现有字段的隐含含义：

- 协议版本和能力查询。
- 节点在线/健康状态与故障码。
- 命令序号、命令回显和应用层 ACK。
- 驱动使能、急停和明确的 PWM 停止语义。
- 编码器数据有效标志、采样序号和新鲜度时间。
- 速度、电角度、目标值和实际状态的独立反馈字段。
- 浮点输入的范围、NaN/Inf 拒绝和量化规则。
- 发现响应限频、节点冲突检测和可配置节点 ID。

在这些能力加入之前，控制端应严格按本书的三类现行帧解析，并把未定义消息类型当作未知消息忽略。
