"""
超时停车后恢复运动测试 - 验证超时回零后新指令能否立即恢复运动

MCU 端逻辑 (serial_cmd.c):
  - RUN 状态下 500ms 无差速帧 -> 超时回零 (g_active=0, g_cmd=0, g_state 仍 RUN)
  - 收到新差速帧 -> g_active=1, g_cmd=新值 -> 车立即恢复运动

测试流程:
  阶段1: 发 1 帧前进指令 (车动), 然后停发 >500ms 让超时回零 (车停)
  阶段2: 持续发后退指令 (与前进反向, 便于观察), 验证车能否立即恢复运动
  退出: 发 stop 帧安全停止

判定:
  阶段2开始后车立即后退运动   -> 恢复成功 (g_state 仍 RUN, 新帧生效)
  车不动                      -> 恢复失败 (可能 g_state=STOP, 需先发 run)

依赖: pyserial  (pip install pyserial)
用法: python test_recover.py
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

SEND_INTERVAL = 0.02   # 20ms = 50Hz (阶段2持续发送间隔)

# ==================== 测试参数 ====================
LINEAR_A = 0.4         # 阶段1: 前进速度 (m/s)
LINEAR_B = -0.3        # 阶段2: 后退速度 (m/s, 与A反向便于观察)
TIMEOUT_MS = 500       # MCU 端超时阈值
HOLD_A_SEC = 1.2       # 阶段1后停发时长 (>500ms 确保超时回零生效)
HOLD_B_SEC = 2.0       # 阶段2持续发B时长


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

    print(f"超时停车后恢复运动测试")
    print(f"阶段1: 发1帧前进({LINEAR_A}m/s) -> 停发{HOLD_A_SEC}s 超时回零(车停)")
    print(f"阶段2: 持续发后退({LINEAR_B}m/s) {HOLD_B_SEC}s -> 验证立即恢复运动")
    print(f"判定: 阶段2车立即后退 = 成功; 不动 = 失败(需先发run)")
    print("-" * 56)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[错误] 打开 {PORT} 失败: {e}")
        sys.exit(1)
    print(f"[OK] {PORT} 已打开")

    try:
        # stop (安全起步)
        f = pack(TYPE_STOP)
        ser.write(f)
        print(f"[发送] stop : {hex_str(f)}  <- 切换 STOP")
        time.sleep(0.2)

        # run (进入运行)
        f = pack(TYPE_RUN)
        ser.write(f)
        print(f"[发送] run  : {hex_str(f)}  <- 切换 RUN")
        time.sleep(0.2)

        # ===== 阶段1: 发 1 帧前进, 然后停发让超时回零 =====
        f = pack(TYPE_DIFF, struct.pack("<ff", LINEAR_A, 0.0))
        ser.write(f)
        print(f"\n[阶段1] 发1帧前进 {LINEAR_A}m/s: {hex_str(f)}")
        print(f"[阶段1] 停发 {HOLD_A_SEC}s 让超时回零 (MCU ~{TIMEOUT_MS}ms 自动停车)")

        t0 = time.time()
        last_marker = -1
        while time.time() - t0 < HOLD_A_SEC:
            ms = int((time.time() - t0) * 1000)
            marker = ms // 400
            if marker != last_marker:
                last_marker = marker
                tag = "  <- MCU 应已回零停车" if ms >= TIMEOUT_MS else ""
                print(f"  停发 {ms:4d}ms{tag}")
            time.sleep(0.05)
        print(f"[阶段1] 完成, 车应已停。现在测试新指令能否恢复运动")

        # ===== 阶段2: 持续发后退指令, 验证立即恢复 =====
        f = pack(TYPE_DIFF, struct.pack("<ff", LINEAR_B, 0.0))
        print(f"\n[阶段2] 持续发后退 {LINEAR_B}m/s {HOLD_B_SEC}s: {hex_str(f)}")
        print(f"[阶段2] 预期: 车立即恢复运动 (后退)")
        t0 = time.time()
        last_marker = -1
        while time.time() - t0 < HOLD_B_SEC:
            ser.write(f)
            ms = int((time.time() - t0) * 1000)
            marker = ms // 500
            if marker != last_marker:
                last_marker = marker
                print(f"  持续发新指令 {ms:4d}ms")
            time.sleep(SEND_INTERVAL)

        print("-" * 56)
        print(f"[判定] 阶段2开始后车立即后退运动  -> 恢复成功 (g_state 仍 RUN, 新帧生效)")
        print(f"       车不动                     -> 恢复失败 (需先发 run 切回 RUN)")

    except KeyboardInterrupt:
        print("\n[中断] 收到 Ctrl+C, 安全停止...")

    finally:
        try:
            if ser.is_open:
                f = pack(TYPE_STOP)
                ser.write(f)
                print(f"\n[发送] stop : {hex_str(f)}  <- 安全停止")
                time.sleep(0.1)
                ser.close()
                print(f"[OK] {PORT} 已关闭")
        except Exception as e:
            print(f"[警告] 退出清理异常: {e}")


if __name__ == "__main__":
    main()
