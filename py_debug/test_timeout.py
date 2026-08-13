"""
500ms 超时回零测试 - 验证 MCU 端无指令自动停车保护是否生效

MCU 端 serial_cmd.c 逻辑: RUN 状态下, 距上一帧差速指令超过 500ms
则自动把指令归零停车 (防通信断线失控)。

测试流程:
  1. stop -> run (进入运行状态)
  2. 发 1 帧前进指令 (让车动起来)
  3. 完全停止发送, 观察 OBSERVE_SEC 秒
     - 若 MCU 超时回零生效: 车在 ~500ms 自动停下并保持静止
     - 若未生效: 车持续运动到测试结束 (由 stop 帧停)
  4. 退出时发 stop 帧安全停止

关键: 观察期间绝不发任何帧 (否则会刷新 MCU 超时计时, 测试无效)。

依赖: pyserial  (pip install pyserial)
用法: python test_timeout.py
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

# ==================== 测试参数 ====================
LINEAR = 0.4           # 测试用前进速度 (m/s)
OBSERVE_SEC = 3.0      # 停发后观察时长 (秒)
TIMEOUT_MS = 500       # MCU 端超时阈值 (毫秒, 与 serial_cmd.h 一致)


# ==================== CRC-8/MAXIM ====================
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 0x01 else crc >> 1
    return crc & 0xFF


def pack(frame_type: int, data: bytes = b"") -> bytes:
    f = bytes([HEADER, frame_type, len(data)]) + data
    return f + bytes([crc8_maxim(f)])


def hex_str(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# ==================== 主流程 ====================
def main():
    assert crc8_maxim(b"123456789") == 0xA1, "CRC 自检失败"

    print(f"500ms 超时回零测试")
    print(f"流程: stop -> run -> 发1帧前进({LINEAR}m/s) -> 停发{OBSERVE_SEC}s 观察")
    print(f"判定: 车在 ~{TIMEOUT_MS}ms 停且保持 = 生效; 持续运动到结束 = 未生效")
    print("-" * 56)

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

        # 3. 发 1 帧前进指令 (仅 1 帧, 让车动起来)
        f = pack(TYPE_DIFF, struct.pack("<ff", LINEAR, 0.0))
        ser.write(f)
        print(f"[发送] diff : {hex_str(f)}  <- 1 帧前进 {LINEAR} m/s")

        # 4. 完全停止发送, 观察超时回零
        print(f"[停发] 现在停止发送, 观察 {OBSERVE_SEC}s "
              f"(MCU 应在 ~{TIMEOUT_MS}ms 自动回零)")
        print("-" * 56)
        t0 = time.time()
        last_marker = -1
        while True:
            elapsed = time.time() - t0
            ms = int(elapsed * 1000)
            marker = ms // 500
            if marker != last_marker:
                last_marker = marker
                tag = "  <- MCU 应已回零停车" if ms >= TIMEOUT_MS else ""
                print(f"  已停发 {ms:4d}ms{tag}")
            if elapsed >= OBSERVE_SEC:
                break
            time.sleep(0.05)
        print("-" * 56)
        print(f"[结束] 观察完成")
        print(f"判定: 车在 ~{TIMEOUT_MS}ms 停下并保持静止  -> 超时回零生效")
        print(f"      车持续运动到此刻才停         -> 超时回零未生效")

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
