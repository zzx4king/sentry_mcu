"""
直行测试 - 直行5秒后顺时针旋转

流程: stop(安全起步) -> run(进入运行) -> 直行 5 秒 -> 顺时针旋转(持续, Ctrl+C 停)
退出时自动发送 stop 帧安全停止。

坐标系: x 向前, yaw 逆时针为正
  linear  > 0 前进 (m/s)
  angular < 0 顺时针 (rad/s)

依赖: pyserial  (pip install pyserial)
用法: python test_straight.py
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
TYPE_DIFF = 0x10

SEND_INTERVAL = 0.02   # 20ms = 50Hz (须 < MCU 端 500ms 超时)

# ==================== 测试参数 ====================
LINEAR = 0.4           # 直行速度 (m/s)
STRAIGHT_SEC = 5.0     # 直行时长 (秒)
ANGULAR = 0.6         # 顺时针旋转角速度 (rad/s, 负=顺时针)


# ==================== CRC-8/MAXIM ====================
# 多项式 0x31, 初值 0x00, 反射 (reversed poly 0x8C), 校验 "123456789" = 0xA1
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 0x01 else crc >> 1
    return crc & 0xFF


def pack(frame_type: int, data: bytes = b"") -> bytes:
    """打包一帧: Header + Type + Len + Data + CRC"""
    f = bytes([HEADER, frame_type, len(data)]) + data
    return f + bytes([crc8_maxim(f)])


def hex_str(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def send_for(ser, frame: bytes, seconds: float, label: str):
    """在 seconds 秒内持续发送同一帧"""
    n = max(1, int(seconds / SEND_INTERVAL))
    print(f"[{label}] 持续 {seconds:.1f}s ({n} 帧)  {hex_str(frame)}")
    for _ in range(n):
        ser.write(frame)
        time.sleep(SEND_INTERVAL)


# ==================== 主流程 ====================
def main():
    # 自检 CRC
    assert crc8_maxim(b"123456789") == 0xA1, "CRC 自检失败"

    print(f"直行测试: 前进 {LINEAR} m/s × {STRAIGHT_SEC}s -> 顺时针旋转 {ANGULAR} rad/s")
    print(f"端口: {PORT} @ {BAUD}  发送间隔: {SEND_INTERVAL*1000:.0f}ms")
    print("-" * 50)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[错误] 打开 {PORT} 失败: {e}")
        sys.exit(1)
    print(f"[OK] {PORT} 已打开")

    try:
        # 1. stop (安全起步)
        f = pack(TYPE_STOP)
        ser.write(f)
        print(f"[发送] stop : {hex_str(f)}  <- 切换 STOP")
        time.sleep(0.2)

        # 2. run (进入运行)
        f = pack(TYPE_RUN)
        ser.write(f)
        print(f"[发送] run  : {hex_str(f)}  <- 切换 RUN")
        time.sleep(0.2)

        # 3. 直行 5 秒
        send_for(ser, pack(TYPE_DIFF, struct.pack("<ff", LINEAR, 0.0)),
                 STRAIGHT_SEC, "直行")

        # 4. 顺时针旋转 (持续, 直到 Ctrl+C)
        f = pack(TYPE_DIFF, struct.pack("<ff", 0.0, ANGULAR))
        print(f"[旋转] 顺时针 {ANGULAR} rad/s (持续, Ctrl+C 停止)  {hex_str(f)}")
        while True:
            ser.write(f)
            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\n[中断] 收到 Ctrl+C, 安全停止...")

    finally:
        try:
            if ser.is_open:
                f = pack(TYPE_STOP)
                ser.write(f)
                print(f"[发送] stop : {hex_str(f)}  <- 安全停止")
                time.sleep(0.1)
                ser.close()
                print(f"[OK] {PORT} 已关闭")
        except Exception as e:
            print(f"[警告] 退出清理异常: {e}")


if __name__ == "__main__":
    main()
