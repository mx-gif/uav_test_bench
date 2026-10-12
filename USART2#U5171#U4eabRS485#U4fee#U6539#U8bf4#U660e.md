# USART2 共享 RS485 修改说明

## 已完成的修改

- CubeMX 中已禁用 UART5，工程代码不再引用 `huart5` 或 `MX_UART5_Init()`。
- 编码器与力传感器统一使用 USART2：
  - USART2_TX：PA2
  - USART2_RX：PA3
  - RS485 方向控制：PB2，发送时置高，接收时置低
- 新增 `Library/RS485Bus/RS485Bus.c/.h`，统一管理：
  - RS485 收发方向
  - UART 波特率切换
  - 阻塞式发送和接收完整事务
  - 超时和串口错误状态
- 编码器不再使用 `HAL_UART_Receive_IT()` 单字节中断接收，改为一次发送 8 字节查询、一次接收 7 字节响应。
- 力传感器不再使用 UART5，改为通过共享总线一次发送 8 字节查询、一次接收 9 字节响应。
- Keil 工程文件已加入 `RS485Bus.c/.h` 和对应头文件路径。
- 已删除旧编译产物，打开 Keil 后需要执行一次 Rebuild。

## 当前设备参数

| 设备 | 波特率 | 地址 | 查询内容 |
|---|---:|---:|---|
| 编码器 1～3 | 9600 | 1、2、3 | 读 `0x0000`，1 个寄存器 |
| 力传感器 | 19200 | 1 | 读 `0x3000`，2 个寄存器 |

编码器和力传感器虽然可能都存在地址 `1`，但当前使用不同波特率。程序会在每笔事务开始前自动切换 USART2：

1. 查询编码器前切换到 9600；
2. 查询力传感器前切换到 19200；
3. 每笔查询必须完成或超时后，才开始下一笔查询。

因此不需要立即修改力传感器地址。

## 硬件连接

只使用一个与 USART2 相连的 RS485 收发器：

```text
STM32 PA2/PA3 + PB2
          │
      RS485收发器
          │ A、B
          ├── 编码器1
          ├── 编码器2
          ├── 编码器3
          └── 力传感器
```

注意：

- 所有设备 A 接 A、B 接 B；没有响应时可以交换 A/B 排查。
- 所有设备和主控应有可靠的信号地参考。
- 力传感器仍按原方式使用 12 V 供电，不能用 STM32 的 3.3 V 给传感器供电。
- PB2 必须连接共享 RS485 收发器的 DE/RE 控制端。
- 总线两端按需要放置 120 Ω 终端电阻，中间节点不要重复放置终端电阻。

## 主要调试变量

可在 Keil Watch 中观察：

- `Encoder_LastStatus`：最近一次编码器通信状态
- `Encoder_LastPolledId`：最近查询的编码器 ID
- `Encorder_Txpacket`：最近编码器查询帧
- `Encorder_Rxpacket`：最近编码器响应帧
- `Encoder_Position[0..2]`：三个编码器角度
- `g_force_status`：最近一次力传感器读取状态
- `ForceSensor_LastRx`：最近力传感器响应帧
- `g_force_raw`：最近正确的原始力值
- `RS485Bus_GetBaudRate()`：当前 USART2 波特率
- `RS485Bus_GetLastStatus()`：最近共享总线状态

## 编译和测试顺序

1. 打开 `MDK-ARM/quadrotor_test_bench.uvprojx`。
2. 执行 `Project -> Rebuild all target files`。
3. 先只连接一个编码器，观察 `Encoder_LastStatus` 和接收帧。
4. 再只连接力传感器，观察 `g_force_status` 和 `ForceSensor_LastRx`。
5. 两类设备分别正常后，再把它们并联到同一条 A/B 总线上测试。

## 参数调整位置

- 编码器波特率：`Library/EncoderLib/EncoderSimple.h` 中的 `ENCODER_BAUD_RATE`
- 力传感器波特率：`Library/ForceSensorLib/ForceSensor.h` 中的 `FORCE_SENSOR_BAUD_RATE`
- 力传感器地址：`Library/ForceSensorLib/ForceSensor.h` 中的 `FORCE_SENSOR_ADDRESS`
