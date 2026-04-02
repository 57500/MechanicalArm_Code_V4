import pygame
import serial
import time
import socket
import threading

# ======== 配置参数 ========
PORT = 'COM5'  # 替换为实际串口号
BAUD = 115200
FPS = 100  # 目标发送频率 100Hz
DEADZONE = 0.15  # 摇杆死区

TCP_PORT = 8888  # VOFA+ 连接的本地网络端口

# ======== 全局变量 ========
vofa_client = None
serial_lock = threading.Lock()


# ======== 辅助运算函数 ========
def dz(val):
    return 0.0 if abs(val) < DEADZONE else val


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def read_stick(js, axis_id, axis_init):
    raw = js.get_axis(axis_id)
    corrected = raw - axis_init[axis_id]
    corrected = clamp(corrected, -1.0, 1.0)
    return clamp(int(dz(corrected) * 100), -100, 100)


def read_trigger(js, axis_id, axis_init):
    raw = js.get_axis(axis_id)
    base = axis_init[axis_id]
    if raw <= base:
        return 0
    val = (raw - base) / (1.0 - base) * 100.0 if base < 1.0 else 0
    return clamp(int(val), 0, 100)


# ======== 网络桥接线程 ========
def tcp_server_task(ser):
    global vofa_client
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', TCP_PORT))
    server.listen(1)
    print(f"\n[网络桥接] TCP 服务器已启动！请在 VOFA+ 连接 127.0.0.1:{TCP_PORT}")

    while True:
        client, addr = server.accept()
        print(f"\n[网络桥接] VOFA+ 已成功连接: {addr}")
        vofa_client = client
        try:
            while True:
                data = client.recv(1024)
                if not data:
                    break
                with serial_lock:
                    ser.write(data)
                print(f"[VOFA+ -> 单片机] 转发指令: {data.decode('utf-8', 'ignore').strip()}")
        except Exception as e:
            print(f"\n[网络桥接] VOFA+ 连接异常: {e}")
        finally:
            vofa_client = None
            client.close()
            print("[网络桥接] VOFA+ 已断开，等待重新连接...")


def serial_rx_task(ser):
    global vofa_client
    rx_buffer = b''  # 新增：字节缓冲区，用于解决打印截断问题

    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)

                # 1. 收到数据第一时间原封不动发给 VOFA+ (保证VOFA+波形实时性)
                if vofa_client:
                    try:
                        vofa_client.sendall(data)
                    except Exception as e:
                        print(f"[网络桥接] 发给VOFA+失败: {e}")
                        vofa_client = None

                # 2. 将数据加入本地缓冲区，用于控制台完整打印
                rx_buffer += data

                # 3. 只要缓冲区里有换行符，就提取出一整行
                while b'\n' in rx_buffer:
                    # 以第一个换行符为界，分割成 "完整的一行" 和 "剩下的碎片"
                    line, rx_buffer = rx_buffer.split(b'\n', 1)

                    try:
                        # 解码并去除两端的空白字符（包括 \r）
                        debug_text = line.decode('utf-8', 'ignore').strip()
                        if debug_text:
                            print(f"[单片机 -> 电脑] {debug_text}")
                    except:
                        pass

        except Exception as e:
            print(f"[串口读取报错] {e}")
            time.sleep(1)
        time.sleep(0.002)


# ======== 主程序 ========
def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未检测到手柄！")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"已连接手柄: {js.get_name()}")
    num_buttons = js.get_numbuttons()
    print(f"检测到手柄共有 {num_buttons} 个按键")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.01)
        print(f"串口 {PORT} 已打开，波特率 {BAUD}")
    except Exception as e:
        print(f"串口打开失败: {e}")
        return

    threading.Thread(target=tcp_server_task, args=(ser,), daemon=True).start()
    threading.Thread(target=serial_rx_task, args=(ser,), daemon=True).start()

    print(">>> 手柄放桌上别动，校准中...")
    time.sleep(1)
    for _ in range(100):
        pygame.event.pump()
        time.sleep(0.01)

    axis_init = [js.get_axis(i) for i in range(js.get_numaxes())]
    print(f"校准完成！初始偏移量: {[round(x, 2) for x in axis_init]}")
    print(f"开始以 {FPS}Hz 频率实时控制机械臂...\n")

    clock = pygame.time.Clock()
    running = True
    last_msg = ""

    try:
        while running:
            clock.tick(FPS)
            pygame.event.pump()

            # === 读取摇杆和扳机 ===
            lx = read_stick(js, 0, axis_init)
            ly = read_stick(js, 1, axis_init)
            rx = read_stick(js, 2, axis_init)
            ry = read_stick(js, 3, axis_init)
            lt = read_trigger(js, 4, axis_init)
            rt = read_trigger(js, 5, axis_init)

            # === 读取常规按键 ===
            btn_a = js.get_button(0)
            btn_b = js.get_button(1)
            btn_x = js.get_button(2)
            btn_y = js.get_button(3)
            btn_lb = js.get_button(4)
            btn_rb = js.get_button(5)

            # === 读取 M1/M2 (映射为 L3/R3) ===
            # 通常 L3 是索引 8，R3 是索引 9。加入越界保护。
            btn_m1 = js.get_button(8) if num_buttons > 8 else 0
            btn_m2 = js.get_button(9) if num_buttons > 9 else 0

            # === 读取十字键 ===
            hat = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)
            dpad_x, dpad_y = hat[0], hat[1]

            # === 组装数据 (在末尾追加了 btn_m1 和 btn_m2) ===
            msg = (f"PAD:{lx},{ly},{rx},{ry},"
                   f"{lt},{rt},"
                   f"{btn_a},{btn_b},{btn_x},{btn_y},{btn_lb},{btn_rb},"
                   f"{dpad_x},{dpad_y},"
                   f"{btn_m1},{btn_m2}\n")

            # === 发送数据 ===
            try:
                with serial_lock:
                    ser.write(msg.encode('ascii'))

                if msg != last_msg:
                    print(f"-> 实时下发: {msg.strip()}")
                    last_msg = msg

            except Exception as e:
                print(f"发送断开: {e}")
                break

    except KeyboardInterrupt:
        print("\n用户手动停止...")
    except Exception as e:
        print(f"\n运行出错: {e}")
    finally:
        ser.close()
        pygame.quit()
        print("程序已退出，串口已释放。")


if __name__ == "__main__":
    main()
