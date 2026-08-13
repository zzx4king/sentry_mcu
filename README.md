# sentry_mcu — RoboMaster 哨兵底盘控制固件

STM32F407IGH6 @168MHz + FreeRTOS 的麦轮哨兵机器人底盘控制固件。通过 USART6 接收上位机的差速驱动指令（线速度 + 角速度）控制底盘运动，CAN 总线驱动 4 路 C620 电调 + M3508 电机。

---

## 硬件平台

| 项目 | 配置 |
|---|---|
| MCU | STM32F407IGH6 @168MHz (大疆开发板 C 型) |
| RTOS | FreeRTOS V10.3.1 (CMSIS-OS2) |
| 构建 | Keil MDK-ARM, 工程名 `pid_serialPlot` |
| 时钟 | HSE+PLL, SYSCLK=168MHz, APB1=42MHz, APB2=84MHz |
| CAN1 | 1Mbps, 4×C620+M3508, 控制帧 ID `0x200`, 反馈 ID `0x201~0x204` |
| USART6 | 460800 8N1, PG14(TX)/PG9(RX), DMA2_Stream1(RX)/Stream6(TX) Channel5 |

### 底盘物理参数
- 轮半径 `wheel_radius` = 0.0765 m
- 半轮距 `half_wheelbase` = 0.218 m (对称布局)
- 减速比 `reduction_ratio` = 15.76
- 限幅: 电机 ±8000 RPM, 电流 ±16384

---

## 控制协议

帧格式（变长，小端）:

```
+--------+--------+--------+----------------+--------+
| Header | Type   | Len    | Data[Len]      | CRC    |
| 1 字节 | 1 字节 | 1 字节 | Len 字节       | 1 字节 |
+--------+--------+--------+----------------+--------+
```

- **Header** = `0xA5`（帧头，字节流同步）
- **Type** = 指令类型
- **Len** = Data 字节数 (0 ~ 64)
- **CRC** = CRC-8/MAXIM, 多项式 0x31, 初值 0x00, 输入/输出反射, 校验 `"123456789"` = `0xA1`
  - CRC 计算范围: Header + Type + Len + Data（不含 CRC 自身）

### 指令类型

| Type | 名称 | Len | Data | 说明 |
|---|---|---|---|---|
| `0x00` | stop 状态帧 | 0 | 无 | 切换 STOP 状态, 底盘停转, 清零指令 |
| `0x01` | run 状态帧 | 0 | 无 | 切换 RUN 状态, 允许接收差速指令 |
| `0x10` | 差速指令 | 8 | linear(f32 LE) + angular(f32 LE) | 驱动底盘 (仅 RUN 状态生效) |

差速指令映射到 `Chassis_SetCmd`: `vx = linear`, `vw = angular`, **`vy = 0`（强制无横移）**。

### 坐标系
右手系: x 向前, y 向左, yaw 逆时针为正。
- `linear` > 0 前进, < 0 后退 (m/s)
- `angular` > 0 逆时针(左转), < 0 顺时针(右转) (rad/s)

---

## 总体状态机

### 系统运行状态 (g_state)

```
        ┌─────────────────────────────┐
        │   上电默认  STOP            │
        │   底盘强制停转              │
        └──────────────┬──────────────┘
                       │ 收到 run 帧 (Type 0x01)
                       ▼
        ┌─────────────────────────────┐
        │           RUN               │◄──┐
        │  接收差速指令驱动底盘       │   │
        │  500ms 无帧 → 自动回零      │   │
        └──────────────┬──────────────┘   │
                       │ 收到 stop 帧     │ 收到新差速帧
                       │ (Type 0x00)      │ (g_active 恢复 1)
                       ▼                  │
        ┌─────────────────────────────┐   │
        │           STOP              │   │
        │   底盘停转 + 清零指令       │───┘ (需先发 run 切回 RUN)
        └─────────────────────────────┘
```

| 事件 | g_state | g_active | g_cmd | 新差速帧能否立即驱动 |
|---|---|---|---|---|
| 上电 | STOP | 0 | 0 | 否（需先 run） |
| 收到 stop 帧 | **STOP** | 0 | 归零 | 否（需先 run） |
| 收到 run 帧 | **RUN** | 0 | 0 | 是（新帧置 active=1） |
| RUN + 差速帧 | RUN | 1 | 新值 | 是 |
| RUN + 500ms 无帧 | RUN(不变) | 0 | 归零 | 是（新帧置 active=1） |

**关键设计**: 超时回零（`g_active=0`）与 stop 帧停车（`g_state=STOP`）分离——超时只是"暂时失去通信"，恢复通信即恢复运动；stop 是"主动停车"，需显式 run 才能恢复。

### 帧解析状态机

```
WAIT_HDR ──(0xA5)──► WAIT_TYPE ──► WAIT_LEN ──(Len>0)──► WAIT_DATA(n) ──► WAIT_CRC
                       │              │                                    │
                       │              │(Len=0)                              │
                       │              └──────────────► WAIT_CRC ◄──────────┘
                       │                                   │
                       │                             CRC 校验 ──► handle_frame
                       │                             失败/超长 ──► WAIT_HDR
```

CRC 边收边算（每收到一字节更新累加值），收到 CRC 字节直接比较，无需存整帧重算。

---

## 安全性操作

1. **上电默认 STOP** — MCU 上电即 `g_state=STOP`，底盘强制停转，避免上电瞬间失控
2. **stop/run 状态机** — 仅 RUN 状态下差速指令才生效；STOP 下收到的差速帧被忽略
3. **500ms 超时回零** — RUN 状态下超过 500ms 未收到有效差速帧，自动归零停车（防通信断线失控）
4. **stop 帧清零指令** — 收到 stop 帧不仅切状态，还清零 `g_cmd`，防止切回 RUN 时使用旧指令
5. **NaN 防护** — 差速指令的 float 值做 NaN 检查（`x != x`），防止 PID 被毒化
6. **Len 上限** — `Len` 字段上限 64 字节（`SERIAL_CMD_MAX_DATA`），超长直接丢弃，防缓冲区溢出
7. **vy 强制 0** — 差速模型无横移，`vy` 永远置 0，避免上位机误发横移值
8. **电机/电流限幅** — 电机 ±8000 RPM，电流 ±16384，底层硬限幅
9. **CRC 校验** — 每帧 CRC-8/MAXIM 校验，校验失败丢弃，防噪声帧误执行

---

## 数据流向

### 指令通路（上位机 → 底盘）

```
上位机
  │ USART6 (460800 8N1)
  ▼
USART6 RX ── DMA2_Stream1 (循环) ──► g_uart6_rx_buf[]
  │
  ├── DMA HalfCplt 中断 ─┐
  ├── DMA Cplt 中断     ├─► SerialCmd_FeedFromISR ──► StreamBuffer (256B)
  └── IDLE 中断         ─┘                              │
                                                        ▼
                                          SerialCmd_Task (独立解析任务)
                                          状态机解析 + CRC 校验
                                                        │
                                          ┌─────────────┴─────────────┐
                                          ▼                           ▼
                                   g_state (STOP/RUN)        g_cmd (vx,vy,vw)
                                          │                           │
                                          └───────────┬───────────────┘
                                                      ▼
                                    CAN_TaskFunction (500Hz, osDelay 2ms)
                                    ┌─────────────────────────────────┐
                                    │ STOP → Chassis_SetCmd(0,0,0)    │
                                    │ RUN  → Chassis_SetCmd(vx,0,vw)  │
                                    └────────────────┬────────────────┘
                                                     ▼
                              Kinematics_Inverse (cmd → 4 电机目标 RPM)
                                                     │
                              MotorControl_GetCurrentByRPM ×4 (前馈+PID 速度环)
                                                     │
                              C620_PackChassisCmd (4×int16 电流 → 8 字节)
                                                     │
                              pdev_can1->Send (CAN ID 0x200)
                                                     ▼
                                            4×C620 电调 → M3508 电机
```

### 反馈通路（底盘 → 里程计）

```
CAN 反馈 0x201~0x204 ──► C620_DecodeFeedback ──► chassis.motor_fb[] (转速/电流/温度)
                                                    │
                              ┌─────────────────────┴─────────────────────┐
                              ▼                                           ▼
                    Kinematics_Forward (实际 RPM → 底盘速度)    SAFE_PRINTF 遥测 (500ms)
                              │
                    Odometry_Update (一阶欧拉积分 → pose)
```

---

## 文件结构

```
sentry/
├─ Core/                            # HAL 配置 + 任务定义
│  ├─ Inc/                          #   FreeRTOSConfig.h, main.h, can.h, dma.h, usart.h, ...
│  └─ Src/
│     ├─ freertos.c                 #   ★ CAN_TaskFunction (500Hz 控制环) + defaultTask (LED)
│     ├─ main.c                     #   外设初始化顺序 (GPIO→DMA→USART1→USART6→CAN1)
│     ├─ usart.c                    #   USART1/6 HAL 配置 + MspInit + fputc
│     ├─ stm32f4xx_it.c            #   中断入口: USART1/6, DMA2_Stream1/2/6/7
│     └─ can.c dma.c gpio.c ...
│
├─ bsp_drivers/                     # 设备抽象层 (HAL 之上, 协议无关)
│  ├─ can_device.c/.h              #   CAN1 收发 + 接收队列 + TX 三邮箱信号量
│  ├─ usart_device.c/.h            #   USART1/6 设备表 + DMA/IDLE 回调 + UART6_Rx_Start
│  └─ sys_lock.h                   #   SAFE_PRINTF 宏 (printf 互斥锁)
│
├─ protocol/                        # 通信协议层
│  ├─ serial_cmd.c/.h              #   ★串口指令: StreamBuffer + 解析任务 + 状态机
│  │                                #     + CRC8/MAXIM + stop/run 状态 + 500ms 超时回零
│  ├─ dbus_c620.c/.h               #   C620 电调协议: 打包电流帧 / 解析反馈帧
│  └─ frame.md                     #   协议帧定义文档
│
├─ math/                            # 控制算法
│  ├─ pid.c/.h                     #   PID 控制器
│  └─ motor_control.c/.h           #   前馈 + PID 速度环 (4 电机独立)
│
├─ chassis/                         # 底盘运动学 / 运动控制
│  ├─ chassis.c/.h                 #   总装: Chassis_SetCmd / Chassis_Control
│  ├─ kinematics.c/.h              #   麦轮逆解/正解 (含 CAN ID 重排)
│  └─ odometry.c/.h                #   一阶欧拉积分里程计
│
├─ py_debug/                        # 上位机调试脚本 (pyserial)
│  ├─ debug_controller.py          #   主控: stop→run→差速指令演示序列 (双工)
│  ├─ test.py                      #   USART6 TX 通路测试 (接收 MCU 测试串)
│  ├─ test_straight.py             #   直行5秒后顺时针旋转
│  ├─ test_timeout.py              #   500ms 超时回零测试
│  └─ test_recover.py              #   超时停车后恢复运动测试
│
├─ MDK-ARM/                         # Keil 工程
│  ├─ pid_serialPlot.uvprojx       #   工程文件 (新增 .c 须在此登记)
│  └─ startup_stm32f407xx.s        #   启动文件
│
├─ pid_serialPlot.ioc              # CubeMX 配置
├─ Drivers/                         # STM32F4xx HAL + CMSIS (第三方, 不改)
└─ Middlewares/Third_Party/FreeRTOS/# FreeRTOS V10.3.1 (第三方, 不改)
```

### 分层调用关系（上层依赖下层）
```
freertos.c (任务/接线) → chassis → math(motor_control→pid) → protocol(dbus_c620) → bsp_drivers(can_device/usart_device) → Core(HAL)
                       ↘ protocol(serial_cmd) ↗
```

### 电机/CAN ID 映射（易错）
- 数组下标 = CAN ID 顺序: `[0]=0x201 M1(右下)`, `[1]=0x202 M2(左下)`, `[2]=0x203 M3(左上)`, `[3]=0x204 M4(右上)`
- 运动学公式索引顺序: `0=左下 1=右下 2=右上 3=左上`（与数组下标不同）
- `Kinematics_Inverse` 内部用公式索引计算后，重排为 CAN ID 顺序输出

---

## 编译与烧录

1. Keil MDK 打开 `MDK-ARM/pid_serialPlot.uvprojx`
2. 编译（新增源文件需手动加入工程并配置 Include 路径）
3. 烧录到 STM32F407IGH6（J-Link/ST-Link）

> 注: `.ioc` 未同步配置 USART6（CubeMX 中未配），若用 CubeMX 重新生成代码会覆盖手改的 USART6 配置。

---

## 上位机调试脚本

依赖: `pip install pyserial`，串口默认 `COM10` @ 460800。

| 脚本 | 功能 |
|---|---|
| `debug_controller.py` | 主控: stop→run→差速指令演示序列循环（无参数）或固定指令持续发送（带参数 `linear angular`） |
| `test.py` | USART6 TX 通路测试，接收 MCU 复位后发送的测试字符串 |
| `test_straight.py` | 直行 5 秒后顺时针旋转 |
| `test_timeout.py` | 验证 500ms 无指令自动回零停车 |
| `test_recover.py` | 验证超时停车后新指令能否立即恢复运动 |

所有脚本退出时自动发送 stop 帧安全停止底盘。
