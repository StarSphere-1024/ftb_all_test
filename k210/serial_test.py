from machine import UART
import time

# 配置 UART1，波特率 115200，8 位数据，无奇偶校验，1 位停止位，超时时间 1000ms
try:
    uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1, timeout=1000)
    print("UART1 initialized successfully")
except Exception as e:
    print("UART init error:", e)
    while True:
        pass  # 停止执行，等待重启

# 主循环：持续检查串口数据并处理
while True:
    try:
        if uart.any():  # 检查是否有数据可读
            data = uart.readline()  # 读取一行数据（以换行符结束）
            if data:
                data = data.strip()  # 去除首尾空白字符
                print("Received:", data)
                if data == b'PingK210':  # 如果收到 "PingK210" 命令
                    uart.write(b'PongK210\n')  # 回复 "PongK210"
                    print("Sent: PongK210")
                else:
                    uart.write(data + b'\n')  # 否则回显接收到的数据
                    print("Sent:", data)
        time.sleep(0.1)  # 短暂延时，避免 CPU 占用过高
    except Exception as e:
        print("Error:", e)
        time.sleep(1)  # 出错时等待 1 秒后重试
