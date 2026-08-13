"""
USART6 TX 通路测试 - PC 端接收脚本

配合 MCU 端 freertos.c 的测试代码:
  MCU 复位后, CAN_TaskFunction 激活 USART6 并用 USART6 TX (DMA) 向 COM10
  发送字符串 "USART6 TX test from MCU\r\n"

用法:
  1. 烧录含 TX 测试代码的固件
  2. 运行本脚本: python test.py
  3. 按 MCU 复位键, 观察是否收到测试字符串

判定:
  收到 "USART6 TX test from MCU"      -> TX 通路/波特率/接线均正常
  收到乱码                           -> 波特率不匹配 (检查 460800)
  收不到任何数据                      -> TX 接线错 / USART6 TX 未发 / COM 口选错
"""

import serial
import sys
import time

PORT = "COM10"
BAUD = 460800


def main():
    print(f"USART6 TX 通路测试 - 监听 {PORT} @ {BAUD}")
    print("打开串口后请按 MCU 复位键, MCU 将发送测试字符串")
    print("Ctrl+C 退出\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[错误] 打开 {PORT} 失败: {e}")
        print("提示: 确认串口未被占用, COM10+ 在某些环境需用 \\\\.\\COM10 语法")
        sys.exit(1)

    print(f"[OK] {PORT} 已打开, 等待 MCU 数据... (按 MCU 复位键)\n")

    total = 0
    try:
        while True:
            n = ser.in_waiting
            if n:
                data = ser.read(n)
                total += len(data)
                print(f"[收到 {len(data)}B] HEX  : {data.hex(' ')}")
                print(f"          ASCII: {data.decode('ascii', errors='replace')}")
                if b"USART6 TX test" in data:
                    print("\n[结论] 收到测试字符串, USART6 TX 通路正常!")
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print(f"\n[退出] 共收到 {total} 字节")
    finally:
        ser.close()
        print(f"[OK] {PORT} 已关闭")


if __name__ == "__main__":
    main()
