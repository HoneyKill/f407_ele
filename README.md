# f407-ele 二维云台（Emm_V5 + USB CDC）

本项目基于 STM32F407，实现了通过 USB CDC 文本命令控制双轴云台（PAN/TILT），底层驱动使用 `Emm_V5_Vel_Control()` 速度模式。

## 1. 功能概览

- 双轴电机控制：PAN + TILT
- 控制方式：USB CDC（虚拟串口）
- 控制模式：速度模式（RPM）
- 已实现命令：`HELP`、`EN`、`VEL`、`STOP`
- 已提供上位机测试脚本：`tools/gimbal_wave_test.py`（可让云台画波浪线）

## 2. 关键文件

- 云台应用层：`App/Emm_V5_App.c`、`App/Emm_V5_App.h`
- 电机底层接口：`App/Emm_V5.c`、`App/Emm_V5.h`
- USB CDC 接收入口：`USB_DEVICE/App/usbd_cdc_if.c`
- 主程序初始化：`Core/Src/main.c`
- 构建配置：`CMakeLists.txt`
- Python 测试脚本：`tools/gimbal_wave_test.py`

## 3. 宏参数配置（全部在 `App/Emm_V5_App.h`）

可根据你的接线和机械方向修改：

- 电机地址
  - `EMM_GIMBAL_PAN_MOTOR_ADDR`
  - `EMM_GIMBAL_TILT_MOTOR_ADDR`
- 方向定义
  - `EMM_GIMBAL_DIR_CW`
  - `EMM_GIMBAL_DIR_CCW`
- 方向反向
  - `EMM_GIMBAL_PAN_DIR_INVERT`
  - `EMM_GIMBAL_TILT_DIR_INVERT`
- 速度与加速度
  - `EMM_GIMBAL_MAX_RPM`（默认 5000）
  - `EMM_GIMBAL_DEFAULT_ACC`
- 同步与上电行为
  - `EMM_GIMBAL_SYNC_FLAG`
  - `EMM_GIMBAL_ENABLE_ON_BOOT`
- CDC 缓冲
  - `EMM_GIMBAL_USB_CMD_BUF_SIZE`
  - `EMM_GIMBAL_USB_TX_BUF_SIZE`

## 4. USB CDC 命令协议

命令以 `\n` 或 `\r\n` 结尾，大小写不敏感。

1. `HELP`
- 返回命令说明。

2. `EN <0|1>`
- `EN 1`：使能双轴
- `EN 0`：失能双轴

3. `VEL <PAN_RPM> <TILT_RPM>`
- 设置双轴速度，单位 RPM。
- 支持正负值：正值表示正方向，负值表示反方向。
- 内部会做限幅，范围 `[-EMM_GIMBAL_MAX_RPM, +EMM_GIMBAL_MAX_RPM]`。

4. `STOP`
- 双轴立即停止。

返回格式示例：

- `OK EN 1`
- `OK VEL 300 -120`
- `OK STOP`
- `ERR UNKNOWN CMD`

## 5. 固件构建

在项目根目录执行：

```bash
cmake --preset Debug
cmake --build --preset Debug -j
```

输出 ELF：`build/Debug/f407-fish.elf`

## 6. 上位机波浪线测试

### 6.1 安装依赖

```bash
pip install pyserial
```

### 6.2 运行脚本

```bash
python3 tools/gimbal_wave_test.py \
  --port /dev/ttyACM0 \
  --duration 15 \
  --pan-rpm 300 \
  --tilt-amp-rpm 500 \
  --freq 0.25 \
  --read-reply
```

参数说明：

- `--port`：CDC 设备节点（例如 `/dev/ttyACM0`）
- `--duration`：运行总时长（秒）
- `--dt`：命令刷新周期（秒，默认 0.05）
- `--pan-rpm`：PAN 轴恒定速度
- `--tilt-amp-rpm`：TILT 轴正弦速度振幅
- `--freq`：正弦频率（Hz）
- `--max-rpm`：上位机限幅（默认 5000）

脚本动作流程：

1. 发送 `EN 1`
2. 循环发送 `VEL <pan> <tilt_sin>`
3. 结束时发送 `STOP`
4. 最后发送 `EN 0`

## 7. 常见问题

1. 云台方向反了
- 修改 `EMM_GIMBAL_PAN_DIR_INVERT` 或 `EMM_GIMBAL_TILT_DIR_INVERT` 为 `1`。

2. 电机无响应
- 检查电机地址宏是否与驱动器地址一致。
- 检查 `USART1` 接线和波特率配置。
- 用串口工具先发 `HELP`，确认 CDC 收发正常。

3. 速度过冲或启动太猛
- 适当调小 `EMM_GIMBAL_DEFAULT_ACC` 和测试脚本中的速度参数。

## 8. 安全建议

- 初次调试先低速（如 `100~300 RPM`）并空载。
- 上电后先 `EN 1`，停止后及时 `STOP` + `EN 0`。
- 避免超过机构可承受的极限角度/速度。
