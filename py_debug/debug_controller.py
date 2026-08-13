"""
串口虚拟调试脚本 - 控制 COM10 (USART6 指令口)

协议帧: Header(0xA5) + Type + Len + Data[Len] + CRC8/MAXIM
  Type 0x00 stop  状态帧 (Len=0) -> 切换 STOP 状态, 底盘停转
  Type 0x01 run   状态帧 (Len=0) -> 切换 RUN  状态, 允许驱动
  Type 0x10 差速  指令帧 (Len=8) -> linear(f32 LE) + angular(f32 LE)

自动流程: stop(安全起步) -> run(进入运行) -> 循环演示丰富动作序列
动作序列: 停止/前进/后退/原地左转/原地右转/前进+左转/前进+右转, 循环往复
Ctrl+C 退出时自动发送 stop 帧安全停止。

双工调试: 发送指令的同时, 接收并打印 MCU 每 500ms 回传的调试信息:
  st=状态(0STOP/1RUN) vx/vw=当前指令 act=通信正常 ok/crc/len/type=解析统计

坐标系: x 向前, y 向左, yaw 逆时针为正
  linear  > 0 前进, < 0 后退  (m/s)
  angular > 0 逆时针(左转), < 0 顺时针(右转)  (rad/s)

依赖: pyserial  (pip install pyserial)
用法:
  python debug_controller.py              # 自动循环演示动作序列
  python debug_controller.py 0.3 0.0      # 持续发送固定指令 (linear angular)
"""

import serial
import struct
import sys
import time

# ==================== 协议常量 ====================
PORT = "COM10"
BAUD = 460800
HEADER = 0xA5
TYPE_STOP = 0x00
TYPE_RUN = 0x01
TYPE_DIFFERENTIAL = 0x10

# 控制指令持续发送间隔 (秒). 20ms = 50Hz (原 100ms; 须 < MCU 端 500ms 超时)
SEND_INTERVAL = 0.02

# ==================== 演示动作序列 ====================
# (名称, linear m/s, angular rad/s, 持续秒)
# 每个动作在其持续时间内以 SEND_INTERVAL 间隔持续发送该差速指令
ACTIONS = [
    ("停止",         0.0,  0.0, 1.5),
    ("前进",         0.4,  0.0, 3.0),
    ("原地左转",     0.0,  0.6, 2.0),
    ("前进+左转",    0.4,  0.3, 3.0),
    ("停止",         0.0,  0.0, 1.0),
    ("后退",        -0.3,  0.0, 2.5),
    ("原地右转",     0.0, -0.6, 2.0),
    ("前进+右转",    0.4, -0.3, 3.0),
    ("停止",         0.0,  0.0, 1.0),
]


# ==================== CRC-8/MAXIM ====================
# 多项式 0x31, 初值 0x00, 输入/输出反射 (reversed poly = 0x8C)
# 校验 "123456789" = 0xA1 (与 MCU 端 serial_cmd.c 一致)
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


def pack_frame(frame_type: int, data: bytes = b"") -> bytes:
    """打包一帧: Header + Type + Len + Data + CRC"""
    frame = bytes([HEADER, frame_type, len(data)]) + data
    crc = crc8_maxim(frame)
    return frame + bytes([crc])


def pack_stop() -> bytes:
    return pack_frame(TYPE_STOP)


def pack_run() -> bytes:
    return pack_frame(TYPE_RUN)


def pack_diff(linear: float, angular: float) -> bytes:
    """差速指令: linear(f32 LE) + angular(f32 LE) = 8 字节"""
    data = struct.pack("<ff", linear, angular)
    return pack_frame(TYPE_DIFFERENTIAL, data)


def hex_str(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# ==================== 接收 MCU 回传 ====================
def recv_print(ser, label="MCU"):
    """读取并打印 MCU 回传的调试信息 (ASCII 文本, 每 500ms 一行)
    回传格式: st=X vx=X.XX vw=X.XX act=X ok=N crc=N len=N type=N
    """
    n = ser.in_waiting
    if n:
        data = ser.read(n)
        txt = data.decode('ascii', errors='replace')
        for line in txt.splitlines():
            if line.strip():
                print(f"  [{label}] {line}")


# ==================== 自检 ====================
def self_test():
    assert crc8_maxim(b"123456789") == 0xA1, "CRC-8/MAXIM 自检失败!"
    print(f"[自检] CRC-8/MAXIM OK (123456789 -> 0xA1)")
    print(f"[自检] stop 帧: {hex_str(pack_stop())}")
    print(f"[自检] run  帧: {hex_str(pack_run())}")
    print(f"[自检] diff 帧(0.4,0.3): {hex_str(pack_diff(0.4, 0.3))}")


# ==================== 动作执行 ====================
def run_action(ser, name: str, linear: float, angular: float, duration: float):
    """在 duration 秒内持续发送同一差速指令 (防 MCU 500ms 超时回零)"""
    frame = pack_diff(linear, angular)
    n = max(1, int(duration / SEND_INTERVAL))
    print(f"[动作] {name:<10} vx={linear:+.2f} vw={angular:+.2f} "
          f"持续 {duration:.1f}s ({n} 帧)  {hex_str(frame)}")
    for _ in range(n):
        ser.write(frame)
        recv_print(ser)
        time.sleep(SEND_INTERVAL)


def run_fixed(ser, linear: float, angular: float):
    """持续发送固定指令, 每 50 帧(~1s) 打印一次状态"""
    frame = pack_diff(linear, angular)
    print(f"[发送] 持续发送固定指令: vx={linear} vw={angular}")
    print(f"       {hex_str(frame)}  (Ctrl+C 停止)")
    print("-" * 56)
    count = 0
    t0 = time.time()
    while True:
        ser.write(frame)
        count += 1
        recv_print(ser)
        if count % 50 == 0:
            print(f"[运行] 已发送 {count} 帧, 累计 {time.time()-t0:.1f}s, "
                  f"vx={linear} vw={angular}")
        time.sleep(SEND_INTERVAL)


# ==================== 主流程 ====================
def main():
    fixed = len(sys.argv) > 1
    linear = float(sys.argv[1]) if fixed else 0.0
    angular = float(sys.argv[2]) if fixed else 0.0

    self_test()
    print()
    print(f"端口: {PORT}  波特率: {BAUD}  8N1  发送间隔: {SEND_INTERVAL*1000:.0f}ms")
    print("双工: 发送指令 + 接收 MCU 调试回传 (st/vx/vw/act/ok/crc/len/type)")
    if fixed:
        print(f"模式: 固定指令 (linear={linear}, angular={angular})")
    else:
        print(f"模式: 演示动作序列 ({len(ACTIONS)} 个动作循环)")
    print("-" * 56)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[错误] 打开 {PORT} 失败: {e}")
        print("提示: 确认串口未被占用, COM10+ 在某些环境需用 \\\\.\\COM10 语法")
        sys.exit(1)

    print(f"[OK] {PORT} 已打开")

    try:
        # 1. 自动 stop (安全起步, 确保从 STOP 状态开始)
        frame = pack_stop()
        ser.write(frame)
        print(f"[发送] stop  : {hex_str(frame)}  <- 切换 STOP 状态")
        time.sleep(0.2)
        recv_print(ser)

        # 2. 自动 run (切换到运行状态, 允许接收差速指令)
        frame = pack_run()
        ser.write(frame)
        print(f"[发送] run   : {hex_str(frame)}  <- 切换 RUN 状态")
        time.sleep(0.2)
        recv_print(ser)

        # 3. 自动发送控制指令
        if fixed:
            run_fixed(ser, linear, angular)
        else:
            loop = 0
            while True:
                loop += 1
                print(f"\n===== 演示序列 第 {loop} 轮 =====")
                for name, vx, vw, dur in ACTIONS:
                    run_action(ser, name, vx, vw, dur)

    except KeyboardInterrupt:
        print("\n[中断] 收到 Ctrl+C, 安全停止...")

    finally:
        # 退出前自动发送 stop 帧安全停止底盘
        try:
            if 'ser' in dir() and ser.is_open:
                frame = pack_stop()
                ser.write(frame)
                print(f"[发送] stop  : {hex_str(frame)}  <- 安全停止")
                time.sleep(0.1)
                ser.close()
                print(f"[OK] {PORT} 已关闭")
        except Exception as e:
            print(f"[警告] 退出清理异常: {e}")


if __name__ == "__main__":
    main()
