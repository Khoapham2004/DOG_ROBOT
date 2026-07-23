#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge trung gian trên Raspberry Pi.

Nhận lệnh từ tay cầm và GUI, chuyển xuống ESP; đồng thời gửi camera, IMU và 12 góc khớp về laptop.
"""

import cv2
import socket
import struct
import time
import threading
import serial
from serial.tools import list_ports
import re
import sys


# ============================================================
# CẤU HÌNH CHÍNH
# ============================================================

# IP của laptop Ubuntu nhận camera, IMU và IK12
LAPTOP_IP = "192.168.0.102"

# Cổng serial kết nối với ESP điều khiển robot
SERIAL_PORT = "AUTO"  # Tự động tìm cổng serial
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.01

# Camera từ Raspberry Pi gửi về GUI laptop
CAMERA_ID = 0
CAMERA_WIDTH = 640             
CAMERA_HEIGHT = 480            
CAMERA_FPS = 30
JPEG_QUALITY = 85  # Mức 80-90 cho ảnh rõ hơn 
CAMERA_SEND_PORT = 5005

# Tay cầm ESP32 gửi UDP về Raspberry Pi
RC_LISTEN_IP = "0.0.0.0"
RC_UDP_PORT = 5000

# GUI laptop gửi lệnh qua Raspberry Pi rồi chuyển xuống ESP robot
# GUI gửi nút điều khiển tay hoặc lệnh PID camera vào port này; Pi chuyển nguyên lệnh xuống ESP.
GUI_OFFSET_LISTEN_IP = "0.0.0.0"
GUI_OFFSET_LISTEN_PORT = 6006

# Raspberry Pi gửi trạng thái về GUI, ví dụ khi nhấn B11 trên tay cầm.
# GUI dựa vào trạng thái này để tự bật/tắt PID camera và bắt đầu/dừng gửi offset.
GUI_STATUS_SEND_PORT = 6007

# Chỉnh chiều cao/gait theo trục X từ GUI hoặc tay cầm. ESP bản mới nhận trực tiếp:
#   X+ / X- / X_RESET
# Mỗi lần nhấn sẽ đổi trục X của các pose FWD/BWD/LEFT/RIGHT/STOP/SL/SR một bước 10 đơn vị.
# Các lệnh HEIGHT_* chỉ giữ để tương thích GUI cũ; Pi sẽ map sang X+/X-/X_RESET.
HEIGHT_X_DEFAULT = -190.0
HEIGHT_X_STEP = 10.0
HEIGHT_X_MIN = -230.0
HEIGHT_X_MAX = -150.0
HEIGHT_LEG_YZ = {
    "FL": (-44.2, 30.0),
    "FR": ( 44.2, 40.0),
    "RL": (-44.2, 40.0),
    "RR": ( 44.2, 40.0),
}

# Raspberry Pi gửi dữ liệu IMU về laptop
IMU_SEND_PORT = 7007

# Raspberry Pi gửi IK12 về laptop/Gazebo tab
IK12_SEND_PORT = 6001
IK12_SEND_ENABLE = True
IK12_SEND_HZ_LIMIT = 80.0  # Giới hạn tần số để tránh spam UDP; ESP in mỗi 30 ms vẫn ổn.

# Cờ debug
PRINT_SERIAL_LINES = False  # Bật True sẽ in nhiều dòng serial, dễ làm chậm quá trình đọc/gửi IMU.
PRINT_RC_DEBUG = False
PRINT_GUI_RX = False  # Giữ False để không spam terminal khi GUI gửi liên tục.
PRINT_CAMERA_FPS = True
PRINT_IMU_SEND = False
PRINT_IK12_SEND = False  # Bật True sẽ in liên tục gói IK12 UDP.


# ============================================================
# CẤU HÌNH GÓI UDP TỪ TAY CẦM
# Theo định dạng gói tay cầm gửi: <BBBBB??????????????B>
# header, ly, rx, ry, pot, b1..b12, js1, js2, footer
# ============================================================

HEADER = 0x4E
FOOTER = 0x42

CENTER = 128
DEADZONE = 28
LOW_TH = 50
HIGH_TH = 150
SIGNAL_TIMEOUT = 1.0

PACKET_FORMAT = "<BBBBB??????????????B"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# ============================================================
# ĐIỀU KHIỂN CÂN BẰNG SBAL TỪ TAY CẦM
# ============================================================
# Packet có 14 nút boolean: b1..b12 + b13/js1 + b14/js2.
# B14 dùng để bật/tắt PID cân bằng SBAL.
# Khi SBAL_ON, joystick phải rx/ry gửi setpoint Roll/Pitch.
# Setpoint giới hạn trong ±15 độ và chỉ gửi lại khi thay đổi hơn 1 độ.
SBAL_SP_LIMIT_DEG = 15.0
SBAL_SP_SEND_DELTA_DEG = 1.0
SBAL_SP_SEND_MIN_INTERVAL = 0.08
SBAL_SP_SEND_ORDER = "PITCH_ROLL"  # ESP hiện nhận SET_SBAL_SP theo thứ tự pitch trước, roll sau.


# ============================================================
# BIẾN TOÀN CỤC
# ============================================================

running = True
ser = None
ser_lock = threading.Lock()

imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ik12_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gui_status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

last_ik12_send_time = 0.0

# Bộ đếm tốc độ đọc/gửi để kiểm tra sample rate.
stats_lock = threading.Lock()
serial_line_count = 0
imu_rx_count = 0
imu_udp_count = 0
ik12_rx_count = 0
ik12_udp_count = 0

height_x_current = HEIGHT_X_DEFAULT

# Khi SBAL/BALANCE đang bật, bridge khóa các lệnh di chuyển.
# Khóa FWD/BWD/LEFT/RIGHT/SL/SR từ cả tay cầm và GUI để tránh đè PID cân bằng.
balance_mode_enabled = False
balance_mode_lock = threading.Lock()
MOTION_COMMANDS_LOCKED_WHEN_BAL = {"FWD", "BWD", "LEFT", "RIGHT", "SL", "SR"}

def set_balance_mode_enabled(enabled: bool):
    global balance_mode_enabled
    with balance_mode_lock:
        balance_mode_enabled = bool(enabled)

def is_balance_mode_enabled() -> bool:
    with balance_mode_lock:
        return bool(balance_mode_enabled)


# ============================================================
# SERIAL
# ============================================================

def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("[SERIAL] KhÃï¿½ÃÂ´ng thÃÂ¡ÃÂºÃÂ¥y cÃÂ¡ÃÂ»Ã¯Â¿Â½ng serial nÃï¿½ÃÂ o")
        return []
    print("[SERIAL] Danh sÃï¿½ÃÂ¡ch cÃÂ¡ÃÂ»Ã¯Â¿Â½ng:")
    for p in ports:
        print(f"  {p.device} | {p.description} | {p.hwid}")
    return ports


def find_serial_port():
    ports = list_serial_ports()
    if not ports:
        return None

    keywords = ["ttyusb", "ttyacm", "ch340", "cp210", "silicon", "usb", "uart", "arduino"]

    for p in ports:
        dev = (p.device or "").lower()
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in dev for k in keywords) or any(k in desc for k in keywords) or any(k in hwid for k in keywords):
            return p.device

    return ports[0].device


def open_robot_serial():
    port = SERIAL_PORT
    if port == "AUTO":
        port = find_serial_port()

    if port is None:
        print("[SERIAL] KhÃï¿½ÃÂ´ng tÃï¿½ÃÂ¬m Ãï¿½Ã¯Â¿Â½Ãï¿½ÃÂ°ÃÂ¡ÃÂ»ÃÂ£c ESP robot")
        return None

    try:
        s = serial.Serial(
            port=port,
            baudrate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
            write_timeout=1,
        )
        time.sleep(2.0)
        s.reset_input_buffer()
        s.reset_output_buffer()
        print(f"[SERIAL] OK {port} @ {SERIAL_BAUD}")
        return s
    except Exception as e:
        print(f"[SERIAL] LÃÂ¡ÃÂ»Ã¯Â¿Â½i mÃÂ¡ÃÂ»Ã¯Â¿Â½ {port}: {e}")
        print("NÃÂ¡ÃÂºÃÂ¿u Permission denied:")
        print("  sudo usermod -a -G dialout pi")
        print("  sudo reboot")
        return None




def reconnect_serial(reason=""):
    """Tự động mở lại serial khi ESP reset, rút USB hoặc bridge gặp lỗi."""
    global ser
    try:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    except Exception:
        pass

    if reason:
        print(f"[SERIAL] Reconnect do: {reason}")
    else:
        print("[SERIAL] Reconnect...")

    ser = open_robot_serial()
    if ser is None:
        print("[SERIAL] Reconnect thÃ¡ÂºÂ¥t bÃ¡ÂºÂ¡i, sÃ¡ÂºÂ½ thÃ¡Â»Â­ lÃ¡ÂºÂ¡i sau")
        return False
    print("[SERIAL] Reconnect OK")
    return True


def send_gui_status(msg: str):
    """Gửi trạng thái từ Raspberry Pi lên GUI trên laptop."""
    try:
        gui_status_sock.sendto(str(msg).strip().encode("utf-8"), (LAPTOP_IP, GUI_STATUS_SEND_PORT))
        print(f"[GUI STATUS] {msg}")
        return True
    except Exception as e:
        print("[GUI STATUS] Lá»i gá»­i:", e)
        return False


def send_uart_once(cmd: str, tag: str = "UART") -> bool:
    global ser
    cmd = str(cmd).strip()
    if not cmd:
        return False

    if ser is None or not ser.is_open:
        print(f"[SERIAL OFF] {cmd}")
        return False

    try:
        with ser_lock:
            ser.write((cmd + "\n").encode("utf-8"))
            ser.flush()
        print(f"[{tag}] {cmd}")
        return True
    except Exception as e:
        print(f"[SERIAL] LÃ¡Â»ï¿½i gÃ¡Â»Â­i '{cmd}': {e}")
        reconnect_serial(f"write error: {e}")
        return False


# ============================================================
# ĐỌC SERIAL TỪ ESP: IMU VÀ IK12
# ============================================================

def parse_imu_line(line: str):
    """
    Tách dữ liệu IMU từ một dòng serial.
    Hỗ trợ dạng: roll:-0.39 pitch:1.04 yaw:-54.97 hoặc [MPU] roll=... pitch=... yaw=...
    """
    m = re.search(
        r"roll\s*[:=]\s*(-?\d+(?:\.\d+)?)\s+pitch\s*[:=]\s*(-?\d+(?:\.\d+)?)\s+yaw\s*[:=]\s*(-?\d+(?:\.\d+)?)",
        line,
        flags=re.IGNORECASE,
    )
    if not m:
        return None
    try:
        return float(m.group(1)), float(m.group(2)), float(m.group(3))
    except Exception:
        return None


def parse_ik12_line(line: str):
    """
    Tách 12 góc khớp từ các dòng chuẩn IK12, RAWIK12 hoặc JOINT12.

    Chỉ nhận đúng định dạng để tránh lấy nhầm 12 số từ các dòng debug khác.
    """
    raw = line.strip()
    upper = raw.upper()

    valid_heads = ("IK12", "RAWIK12", "JOINT12")
    if not upper.startswith(valid_heads):
        return None

    # Bỏ phần prefix trước dấu phẩy hoặc trước khoảng trắng đầu tiên.
    if "," in raw:
        body = raw.split(",", 1)[1]
    else:
        parts = raw.split(maxsplit=1)
        body = parts[1] if len(parts) > 1 else ""

    nums = re.findall(r"[-+]?\d+(?:\.\d+)?", body)
    if len(nums) != 12:
        print(f"[IK12 WARN] DÃï¿½ÃÂ²ng IK12 nhÃï¿½ÃÂ°ng khÃï¿½ÃÂ´ng Ãï¿½Ã¯Â¿Â½ÃÂ¡ÃÂ»ÃÂ§ 12 sÃÂ¡ÃÂ»Ã¯Â¿Â½: {line}")
        return None

    try:
        vals = [float(x) for x in nums]
    except Exception:
        return None

    return vals


def send_imu_udp(roll, pitch, yaw):
    global imu_udp_count
    msg = f"{roll:.2f},{pitch:.2f},{yaw:.2f}"
    try:
        imu_sock.sendto(msg.encode("utf-8"), (LAPTOP_IP, IMU_SEND_PORT))
        with stats_lock:
            imu_udp_count += 1
        if PRINT_IMU_SEND:
            print("[IMU UDP]", msg)
    except Exception as e:
        print("[IMU UDP] LÃÂ¡ÃÂ»Ã¯Â¿Â½i gÃÂ¡ÃÂ»ÃÂ­i:", e)


def send_ik12_udp(vals):
    global last_ik12_send_time

    if not IK12_SEND_ENABLE:
        return

    now = time.time()
    min_dt = 1.0 / max(1.0, IK12_SEND_HZ_LIMIT)
    if now - last_ik12_send_time < min_dt:
        return

    # Gửi đúng định dạng mà GUI laptop đang đọc:
    # IK12,FL1,FL2,FL3,FR1,FR2,FR3,RL1,RL2,RL3,RR1,RR2,RR3
    msg = "IK12," + ",".join(f"{v:.3f}" for v in vals)

    try:
        ik12_sock.sendto(msg.encode("utf-8"), (LAPTOP_IP, IK12_SEND_PORT))
        last_ik12_send_time = now
        global ik12_udp_count
        with stats_lock:
            ik12_udp_count += 1
        if PRINT_IK12_SEND:
            print("[IK12 UDP]", msg)
    except Exception as e:
        print("[IK12 UDP] LÃÂ¡ÃÂ»Ã¯Â¿Â½i gÃÂ¡ÃÂ»ÃÂ­i:", e)


def serial_read_loop():
    global running, ser
    rx_buffer = ""

    print("[SERIAL RX] Thread started")

    while running:
        if ser is None:
            time.sleep(0.2)
            continue

        try:
            with ser_lock:
                n = ser.in_waiting
                data = ser.read(n).decode("utf-8", errors="ignore") if n > 0 else ""

            if data:
                rx_buffer += data

                while "\n" in rx_buffer:
                    line, rx_buffer = rx_buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    global serial_line_count, imu_rx_count, ik12_rx_count
                    with stats_lock:
                        serial_line_count += 1

                    if PRINT_SERIAL_LINES:
                        print("[ESP]", line)

                    imu = parse_imu_line(line)
                    if imu is not None:
                        with stats_lock:
                            imu_rx_count += 1
                        send_imu_udp(*imu)

                    ik12 = parse_ik12_line(line)
                    if ik12 is not None:
                        with stats_lock:
                            ik12_rx_count += 1
                        send_ik12_udp(ik12)

        except Exception as e:
            print("[SERIAL RX] LÃ¡Â»ï¿½i:", e)
            reconnect_serial(f"read error: {e}")
            time.sleep(0.5)

        time.sleep(0.002)


# ============================================================
# ĐIỀU KHIỂN CHIỀU CAO / GAIT-X QUA SERIAL
# ============================================================

def clamp_height_x(x: float) -> float:
    return max(HEIGHT_X_MIN, min(HEIGHT_X_MAX, float(x)))


def send_x_command_to_esp(cmd: str, tag: str = "XCTRL") -> bool:
    """
    Gửi trực tiếp lệnh X+/X-/X_RESET xuống ESP.

    ESP tự xử lý trục X của các pose FWD/BWD/LEFT/RIGHT/STOP/SL/SR,
    nên Raspberry Pi không cần đổi sang từng lệnh LEG riêng.
    """
    cmd = str(cmd).strip().upper()
    ok = send_uart_once(cmd, tag=tag)
    # Hỏi lại ESP để terminal hiển thị rõ trạng thái X và góc IK12 hiện tại.
    if cmd in ("X+", "X-", "X_RESET"):
        time.sleep(0.02)
        send_uart_once("SHOWX", tag=tag)
        time.sleep(0.02)
        send_uart_once("IK12_NOW", tag=tag)
    return ok


def handle_gui_command(msg: str):
    """
    Xử lý lệnh từ GUI trước khi chuyển xuống ESP.
    Lệnh thường được forward nguyên văn; riêng HEIGHT_* cũ được map sang X+/X-/X_RESET.
    """
    global height_x_current
    raw = str(msg).strip()
    if not raw:
        return

    up = raw.upper().strip()

    # Đồng bộ trạng thái balance nếu GUI cũng gửi SBAL_ON/OFF.
    if up == "SBAL_ON":
        set_balance_mode_enabled(True)
    elif up == "SBAL_OFF":
        set_balance_mode_enabled(False)

    # Khi Balance/SBAL đang bật, khóa lệnh di chuyển từ GUI để không đè PID cân bằng.
    # Vẫn cho phép STOP, SBAL_OFF, SET_SBAL_SP và các lệnh cấu hình SBAL đi qua bình thường.
    if is_balance_mode_enabled() and up in MOTION_COMMANDS_LOCKED_WHEN_BAL:
        print(f"[GUI BLOCKED BY SBAL] {up}")
        return

    # Hỗ trợ nhiều tên lệnh từ GUI/RC để thao tác nhanh hơn.
    if up in ("X+", "X +", "HEIGHT+", "HEIGHT_UP", "UP_HEIGHT", "CAO+", "CAO_UP"):
        height_x_current = clamp_height_x(height_x_current + HEIGHT_X_STEP)
        print(f"[GUI->X] X+ | X dÃÂ¡ÃÂ»ÃÂ± kiÃÂ¡ÃÂºÃÂ¿n {height_x_current:.1f} mm")
        send_x_command_to_esp("X+", tag="GUI->X")
        return

    if up in ("X-", "X -", "HEIGHT-", "HEIGHT_DOWN", "DOWN_HEIGHT", "CAO-", "CAO_DOWN"):
        height_x_current = clamp_height_x(height_x_current - HEIGHT_X_STEP)
        print(f"[GUI->X] X- | X dÃÂ¡ÃÂ»ÃÂ± kiÃÂ¡ÃÂºÃÂ¿n {height_x_current:.1f} mm")
        send_x_command_to_esp("X-", tag="GUI->X")
        return

    if up in ("HEIGHT_RESET", "X_RESET", "HEIGHT_HOME", "X_HOME"):
        height_x_current = HEIGHT_X_DEFAULT
        print(f"[GUI->X] X_RESET | X dÃÂ¡ÃÂ»ÃÂ± kiÃÂ¡ÃÂºÃÂ¿n {height_x_current:.1f} mm")
        send_x_command_to_esp("X_RESET", tag="GUI->X")
        return

    # Tương thích lệnh đặt X tuyệt đối từ GUI cũ: Pi đổi thành nhiều bước X+/X-.
    m = re.match(r"^(?:HEIGHT_X|SET_HEIGHT_X|SET_X|X)\s+(-?\d+(?:\.\d+)?)$", raw, flags=re.IGNORECASE)
    if m:
        try:
            target = clamp_height_x(float(m.group(1)))
            steps = int(round((target - height_x_current) / HEIGHT_X_STEP))
            if steps > 0:
                for _ in range(steps):
                    height_x_current = clamp_height_x(height_x_current + HEIGHT_X_STEP)
                    send_x_command_to_esp("X+", tag="GUI->X")
                    time.sleep(0.03)
            elif steps < 0:
                for _ in range(abs(steps)):
                    height_x_current = clamp_height_x(height_x_current - HEIGHT_X_STEP)
                    send_x_command_to_esp("X-", tag="GUI->X")
                    time.sleep(0.03)
            print(f"[GUI->X] SET target={target:.1f} | now={height_x_current:.1f} | steps={steps}")
        except Exception as e:
            print("[XCTRL] parse error:", e)
        return

    # Nếu GUI gửi STAND thì đồng bộ biến chiều cao của Pi về đúng standPose trong ESP.
    if up == "STAND":
        height_x_current = HEIGHT_X_DEFAULT

    # Tab tọa độ: chuyển nguyên lệnh xuống ESP.
    if up.startswith(("SET_POSE ", "SET_PLEG ", "SET_POSE_LEG ", "GET_POSE ", "SHOW_POSE ", "APPLY_POSE ")) or up in ("SHOWPOSES", "GET_POSES", "DUMP_POSES", "RESET_POSES", "POSES_RESET"):
        send_uart_once(raw, tag="GUI->POSE")
        return

    # Các lệnh còn lại như offset camera, FWD/BWD, SET_SBAL, SBAL_ON... được forward nguyên văn.
    send_uart_once(raw, tag="GUI->ESP")

# ============================================================
# UDP TỪ GUI: OFFSET / COMMAND -> SERIAL
# ============================================================

def gui_offset_loop():
    global running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((GUI_OFFSET_LISTEN_IP, GUI_OFFSET_LISTEN_PORT))
    sock.settimeout(0.1)

    print(f"[GUI RX] Listen {GUI_OFFSET_LISTEN_IP}:{GUI_OFFSET_LISTEN_PORT}")

    while running:
        try:
            packet, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception as e:
            print("[GUI RX] LÃÂ¡ÃÂ»Ã¯Â¿Â½i:", e)
            time.sleep(0.1)
            continue

        msg = packet.decode("utf-8", errors="ignore").strip()
        if not msg:
            continue

        if PRINT_GUI_RX:
            print(f"[GUI RX] {addr}: {msg}")

        # Nhận offset camera hoặc các lệnh như FWD/STOP/CAMGAIT_ON/SBAL_ON/SET_SBAL...
        # rồi xử lý trong handle_gui_command trước khi gửi xuống ESP.
        handle_gui_command(msg)

    sock.close()


# ============================================================
# CAMERA RASPBERRY PI -> LAPTOP QUA UDP
# ============================================================

def open_camera_device():
    """Mở camera với cấu hình nhẹ cho Raspberry Pi 3B+; trả về cap hoặc None nếu lỗi."""
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[CAM] KhÃÂ´ng mÃ¡Â»ï¿½ Ãï¿½ÃÂ°Ã¡Â»Â£c camera /dev/video{CAMERA_ID}")
        try:
            cap.release()
        except Exception:
            pass
        return None

    # Ưu tiên MJPG để giảm tải CPU; nếu camera không hỗ trợ thì FOURCC thực tế sẽ được in ra.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))

    print(f"[CAM] Open /dev/video{CAMERA_ID}")
    print(f"[CAM] Request {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, JPEG Q={JPEG_QUALITY}")
    print(f"[CAM] Real    {real_w}x{real_h}@{real_fps:.1f}, FOURCC={fourcc_str}")
    print(f"[CAM] Send to {LAPTOP_IP}:{CAMERA_SEND_PORT}")
    return cap


def camera_send_loop():
    global running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cap = None

    frame_id = 0
    fps_count = 0
    fps_last = time.time()
    last_ok_frame = time.time()

    # Gói 1400 byte giúp tránh phân mảnh IP qua WiFi; gói quá lớn dễ làm rớt frame.
    max_packet_size = 1400

    while running:
        if cap is None or not cap.isOpened():
            cap = open_camera_device()
            if cap is None:
                time.sleep(1.0)
                continue
            last_ok_frame = time.time()

        try:
            # Xả bớt buffer cũ để giảm độ trễ tích lũy.
            try:
                cap.grab()
            except Exception:
                pass

            ret, frame = cap.read()
            if not ret or frame is None:
                print("[CAM] KhÃÂ´ng Ãï¿½Ã¡Â»ï¿½c Ãï¿½ÃÂ°Ã¡Â»Â£c frame -> reconnect camera")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.5)
                continue

            last_ok_frame = time.time()

            if frame.shape[1] != CAMERA_WIDTH or frame.shape[0] != CAMERA_HEIGHT:
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT), interpolation=cv2.INTER_AREA)

            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                continue

            data = jpg.tobytes()
            total_chunks = (len(data) + max_packet_size - 1) // max_packet_size

            for chunk_id in range(total_chunks):
                start = chunk_id * max_packet_size
                chunk = data[start:start + max_packet_size]
                header = struct.pack("!IHH", frame_id, total_chunks, chunk_id)
                try:
                    sock.sendto(header + chunk, (LAPTOP_IP, CAMERA_SEND_PORT))
                except Exception as e:
                    print("[CAM UDP] LÃ¡Â»ï¿½i gÃ¡Â»Â­i:", e)
                    time.sleep(0.05)
                    break

            frame_id = (frame_id + 1) % 1000000
            fps_count += 1

            now = time.time()
            if PRINT_CAMERA_FPS and now - fps_last >= 1.0:
                print(f"[CAM] Send FPS: {fps_count} | jpg={len(data)}B | chunks={total_chunks}")
                fps_count = 0
                fps_last = now

            # Watchdog: nếu camera treo quá 3 giây thì mở lại camera.
            if time.time() - last_ok_frame > 3.0:
                print("[CAM] Watchdog reconnect camera")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.5)

        except Exception as e:
            print("[CAM LOOP] LÃ¡Â»ï¿½i:", e)
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
            cap = None
            time.sleep(0.8)

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass
    sock.close()


# ============================================================
# TAY CẦM UDP -> SERIAL ESP
# ============================================================

def decode_packet(data):
    if len(data) != PACKET_SIZE:
        print(f"[RC WARN] Sai kÃï¿½ÃÂ­ch thÃï¿½ÃÂ°ÃÂ¡ÃÂ»Ã¯Â¿Â½c packet: {len(data)} bytes, cÃÂ¡ÃÂºÃÂ§n {PACKET_SIZE}")
        return None

    unpacked = struct.unpack(PACKET_FORMAT, data)

    header = unpacked[0]
    footer = unpacked[-1]
    if header != HEADER or footer != FOOTER:
        print(f"[RC WARN] Sai header/footer: header=0x{header:02X}, footer=0x{footer:02X}")
        return None

    return {
        "ly": unpacked[1],
        "rx": unpacked[2],
        "ry": unpacked[3],
        "pot": unpacked[4],
        "b1": unpacked[5],
        "b2": unpacked[6],
        "b3": unpacked[7],
        "b4": unpacked[8],
        "b5": unpacked[9],
        "b6": unpacked[10],
        "b7": unpacked[11],
        "b8": unpacked[12],
        "b9": unpacked[13],
        "b10": unpacked[14],
        "b11": unpacked[15],
        "b12": unpacked[16],

        # Hai bit cuối trong packet gốc thường là công tắc joystick JS1/JS2.
        # Trên tay cầm đang gọi là B13/B14 nên tạo thêm alias b13/b14.
        "b13": unpacked[17],
        "b14": unpacked[18],
        "js1": unpacked[17],
        "js2": unpacked[18],
    }


def joystick_to_cmd(ly, pot, rx, ry):
    """
    Chuyển giá trị joystick thành lệnh di chuyển.

    Joystick 1: ly điều khiển FWD/BWD, pot điều khiển LEFT/RIGHT.
    Joystick 2: rx điều khiển SL/SR; chiều đã được đảo để khớp với code tay cầm.
    """
    dy = ly - CENTER
    dx = pot - CENTER
    rdx = rx - CENTER
    rdy = ry - CENTER

    # Ưu tiên joystick 2 cho lệnh đi ngang.
    joy2_active = abs(rdx) >= DEADZONE or abs(rdy) >= DEADZONE
    if joy2_active:
        if abs(rdx) >= abs(rdy):
            if rx < LOW_TH:
                return "SR", dx, dy, rdx, rdy
            elif rx > HIGH_TH:
                return "SL", dx, dy, rdx, rdy
            return "STOP", dx, dy, rdx, rdy
        return "STOP", dx, dy, rdx, rdy

    # Xử lý joystick 1.
    joy1_active = abs(dx) >= DEADZONE or abs(dy) >= DEADZONE
    if not joy1_active:
        return "STOP", dx, dy, rdx, rdy

    if abs(dx) > abs(dy):
        if pot < LOW_TH:
            return "LEFT", dx, dy, rdx, rdy
        elif pot > HIGH_TH:
            return "RIGHT", dx, dy, rdx, rdy
        return "STOP", dx, dy, rdx, rdy

    if ly > HIGH_TH:
        return "FWD", dx, dy, rdx, rdy
    elif ly < LOW_TH:
        return "BWD", dx, dy, rdx, rdy
    return "STOP", dx, dy, rdx, rdy




def joystick1_to_cmd_only(ly, pot):
    """
    Chỉ lấy joystick trái để điều khiển tiến/lùi/trái/phải.
    Dùng khi SBAL đang bật để joystick phải dành riêng cho setpoint cân bằng.
    """
    dy = ly - CENTER
    dx = pot - CENTER

    joy1_active = abs(dx) >= DEADZONE or abs(dy) >= DEADZONE
    if not joy1_active:
        return "STOP", dx, dy

    if abs(dx) > abs(dy):
        if pot < LOW_TH:
            return "LEFT", dx, dy
        elif pot > HIGH_TH:
            return "RIGHT", dx, dy
        return "STOP", dx, dy

    if ly > HIGH_TH:
        return "FWD", dx, dy
    elif ly < LOW_TH:
        return "BWD", dx, dy
    return "STOP", dx, dy


def rc_axis_to_sbal_setpoint(rx, ry):
    """
    Map joystick phải sang setpoint SBAL.
    rx trái/phải điều khiển Roll, ry lên/xuống điều khiển Pitch.
    Kéo joystick lên tạo Pitch âm, kéo xuống tạo Pitch dương.
    """
    span_pos = max(1.0, 255.0 - CENTER)
    span_neg = max(1.0, CENTER)

    def map_axis(v):
        d = float(v) - CENTER
        if d >= 0:
            out = (d / span_pos) * SBAL_SP_LIMIT_DEG
        else:
            out = (d / span_neg) * SBAL_SP_LIMIT_DEG
        return max(-SBAL_SP_LIMIT_DEG, min(SBAL_SP_LIMIT_DEG, out))

    roll_sp = map_axis(rx)
    pitch_sp = map_axis(ry)

    if abs(rx - CENTER) < DEADZONE:
        roll_sp = 0.0
    if abs(ry - CENTER) < DEADZONE:
        pitch_sp = 0.0

    return roll_sp, pitch_sp


def send_sbal_setpoint_from_rc(roll_sp, pitch_sp, tag="RC->SBAL"):
    """
    Gửi setpoint cân bằng xuống ESP.
    ESP hiện nhận SET_SBAL_SP theo thứ tự pitch trước, roll sau.
    """
    if SBAL_SP_SEND_ORDER == "PITCH_ROLL":
        cmd = f"SET_SBAL_SP {pitch_sp:.2f} {roll_sp:.2f}"
    else:
        cmd = f"SET_SBAL_SP {roll_sp:.2f} {pitch_sp:.2f}"
    return send_uart_once(cmd, tag=tag)

def send_motion_cmd(cmd, last_cmd, moving_active):
    motion_cmds = ["FWD", "BWD", "LEFT", "RIGHT", "SL", "SR"]

    if cmd in motion_cmds:
        if cmd != last_cmd:
            send_uart_once(cmd, tag="RC->ESP")
            last_cmd = cmd
        moving_active = True
        return last_cmd, moving_active

    if cmd == "STOP":
        if moving_active:
            send_uart_once("STOP", tag="RC->ESP")
            last_cmd = "STOP"
            moving_active = False
        return last_cmd, moving_active

    return last_cmd, moving_active


def rc_udp_loop():
    global running, height_x_current

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((RC_LISTEN_IP, RC_UDP_PORT))
    sock.settimeout(0.2)

    print(f"[RC] Listen UDP tay cáº§m on {RC_LISTEN_IP}:{RC_UDP_PORT}")
    print("[RC] B1 STAND | B2 SIT | B3 LIE | B4 SHAKE | B5 X+ | B6 X- | B8 CAPTURE FWD/BWD BAL POSE | B11 CAM PID toggle | B14 SBAL toggle")
    print("[RC] Joy1 ly/pot -> FWD/BWD/LEFT/RIGHT")
    print("[RC] Joy2 rx -> SL/SR khi SBAL OFF | Joy2 rx/ry -> Roll/Pitch SP khi SBAL ON")
    print("[RC] Khi SBAL ON: khÃ³a FWD/BWD/LEFT/RIGHT/SL/SR, chá» gá»­i setpoint balance")
    print("[RC] Khi CAM PID ON báº±ng B11: joystick motion bá» bá» qua Äá» khÃ´ng ÄÃ¨ CAMGAIT")

    last_cmd = None
    moving_active = False
    sbal_enabled = False
    cam_pid_enabled = False
    last_packet_time = time.time()

    last_b1 = last_b2 = last_b3 = last_b4 = False
    last_b5 = last_b6 = last_b8 = False
    last_b11 = False
    last_b14 = False

    last_sbal_roll_sp = None
    last_sbal_pitch_sp = None
    last_sbal_sp_send_time = 0.0

    while running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            if time.time() - last_packet_time > SIGNAL_TIMEOUT:
                last_cmd, moving_active = send_motion_cmd("STOP", last_cmd, moving_active)
            continue
        except Exception as e:
            print("[RC] UDP lá»i:", e)
            time.sleep(0.1)
            continue

        packet = decode_packet(data)
        if packet is None:
            continue

        last_packet_time = time.time()

        ly = packet["ly"]
        rx = packet["rx"]
        ry = packet["ry"]
        pot = packet["pot"]

        b1 = packet["b1"]
        b2 = packet["b2"]
        b3 = packet["b3"]
        b4 = packet["b4"]
        b5 = packet["b5"]
        b6 = packet["b6"]
        b8 = packet["b8"]
        b11 = packet.get("b11", False)
        b14 = packet.get("b14", packet.get("js2", False))

        b1_rising = b1 and not last_b1
        b2_rising = b2 and not last_b2
        b3_rising = b3 and not last_b3
        b4_rising = b4 and not last_b4
        b5_rising = b5 and not last_b5
        b6_rising = b6 and not last_b6
        b8_rising = b8 and not last_b8
        b11_rising = b11 and not last_b11
        b14_rising = b14 and not last_b14

        last_b1, last_b2, last_b3, last_b4 = b1, b2, b3, b4
        last_b5, last_b6, last_b8, last_b11, last_b14 = b5, b6, b8, b11, b14

        # Ưu tiên xử lý nút bấm trước joystick.
        if b1_rising:
            cam_pid_enabled = False
            send_gui_status("CAM_PID_OFF")
            send_uart_once("STAND", tag="RC->ESP")
            last_cmd = "STAND"
            moving_active = False
            print("[RC] B1 -> STAND")
            continue

        if b2_rising:
            cam_pid_enabled = False
            send_gui_status("CAM_PID_OFF")
            send_uart_once("SIT", tag="RC->ESP")
            last_cmd = "SIT"
            moving_active = False
            print("[RC] B2 -> SIT")
            continue

        if b3_rising:
            cam_pid_enabled = False
            send_gui_status("CAM_PID_OFF")
            send_uart_once("LIE", tag="RC->ESP")
            last_cmd = "LIE"
            moving_active = False
            print("[RC] B3 -> LIE")
            continue

        if b4_rising:
            cam_pid_enabled = False
            send_gui_status("CAM_PID_OFF")
            send_uart_once("SHAKE", tag="RC->ESP")
            last_cmd = "SHAKE"
            moving_active = False
            print("[RC] B4 -> SHAKE")
            continue

        if b5_rising:
            height_x_current = clamp_height_x(height_x_current + HEIGHT_X_STEP)
            send_x_command_to_esp("X+", tag="RC->X")
            last_cmd = "X+"
            moving_active = False
            print(f"[RC] B5 -> X+ | X dá»± kiáº¿n {height_x_current:.1f} mm")
            continue

        if b6_rising:
            height_x_current = clamp_height_x(height_x_current - HEIGHT_X_STEP)
            send_x_command_to_esp("X-", tag="RC->X")
            last_cmd = "X-"
            moving_active = False
            print(f"[RC] B6 -> X- | X dá»± kiáº¿n {height_x_current:.1f} mm")
            continue

        if b8_rising:
            # B8 lấy tọa độ chân hiện tại sau khi cân bằng bằng SBAL,
            # rồi chỉ ghi vào tâm gait FWD/BWD trên ESP.
            # Sau khi capture thì tự tắt SBAL để mở khóa lệnh tiến/lùi.
            cam_pid_enabled = False
            send_gui_status("CAM_PID_OFF")
            send_uart_once("CAMGAIT_OFF", tag="RC->CAP")
            time.sleep(0.03)
            send_uart_once("CAPTURE_FB_BALANCE_POSE", tag="RC->CAP")
            time.sleep(0.05)

            if sbal_enabled or is_balance_mode_enabled():
                send_uart_once("SBAL_OFF", tag="RC->CAP")
                sbal_enabled = False
                set_balance_mode_enabled(False)
                last_sbal_roll_sp = None
                last_sbal_pitch_sp = None
                print("[RC] B8 -> CAPTURE FWD/BWD balance pose + SBAL_OFF | ÄÃ£ má» khÃ³a FWD/BWD")
            else:
                print("[RC] B8 -> CAPTURE FWD/BWD balance pose | SBAL Äang OFF")

            last_cmd = "CAPTURE_FB_BALANCE_POSE"
            moving_active = False
            continue

        if b11_rising:
            if sbal_enabled or is_balance_mode_enabled():
                print("[RC] B11 bá» bá» qua vÃ¬ SBAL/BAL Äang ON")
                send_gui_status("CAM_PID_OFF")
                cam_pid_enabled = False
                continue

            cam_pid_enabled = not cam_pid_enabled

            if cam_pid_enabled:
                # Giống nút "BẬT BÁM NGƯỜI" trên GUI: bật CAMGAIT rồi cho robot đi FWD.
                send_uart_once("CAMGAIT_ON", tag="RC->CAM")
                time.sleep(0.03)
                send_uart_once("FWD", tag="RC->CAM")
                send_gui_status("CAM_PID_ON")
                last_cmd = "CAM_PID_ON"
                print("[RC] B11 -> CAM PID ON + FWD | ÄÃ£ cáº­p nháº­t GUI")
            else:
                # Giống nút "TẮT BÁM NGƯỜI" trên GUI: STOP trước rồi tắt CAMGAIT.
                send_uart_once("STOP", tag="RC->CAM")
                time.sleep(0.03)
                send_uart_once("CAMGAIT_OFF", tag="RC->CAM")
                send_gui_status("CAM_PID_OFF")
                last_cmd = "CAM_PID_OFF"
                print("[RC] B11 -> CAM PID OFF + STOP | ÄÃ£ cáº­p nháº­t GUI")

            moving_active = False
            continue

        if b14_rising:
            sbal_enabled = not sbal_enabled
            set_balance_mode_enabled(sbal_enabled)
            if sbal_enabled:
                # Khi vào balance, dừng gait/camera gait trước để tránh lệnh di chuyển cũ còn chạy.
                cam_pid_enabled = False
                send_gui_status("CAM_PID_OFF")
                send_uart_once("STOP", tag="RC->BAL")
                time.sleep(0.03)
                send_uart_once("CAMGAIT_OFF", tag="RC->BAL")
                time.sleep(0.03)
                send_uart_once("SBAL_ON", tag="RC->ESP")
                time.sleep(0.02)
                roll_sp, pitch_sp = rc_axis_to_sbal_setpoint(rx, ry)
                send_sbal_setpoint_from_rc(roll_sp, pitch_sp)
                last_sbal_roll_sp = roll_sp
                last_sbal_pitch_sp = pitch_sp
                last_sbal_sp_send_time = time.time()
                last_cmd = "SBAL_ON"
                print(f"[RC] B14 -> SBAL_ON | RollSP={roll_sp:+.1f} PitchSP={pitch_sp:+.1f}")
            else:
                send_uart_once("SBAL_OFF", tag="RC->ESP")
                last_cmd = "SBAL_OFF"
                print("[RC] B14 -> SBAL_OFF | má» khÃ³a lá»nh di chuyá»n")
            moving_active = False
            continue

        if cam_pid_enabled:
            # Đang bám người bằng camera nên không cho joystick thường gửi STOP/FWD/SL/SR đè lên CAMGAIT.
            # B11 vẫn dùng để tắt; các nút tư thế phía trên vẫn được ưu tiên xử lý trước.
            if PRINT_RC_DEBUG:
                print(f"[RC] CAM PID ON - ignore joystick motion | b11={int(b11)} rx={rx} ry={ry}")
            continue

        if sbal_enabled:
            # Khi SBAL ON, khóa toàn bộ lệnh tiến/lùi/trái/phải/SL/SR từ joystick.
            # Joystick phải chỉ dùng để gửi setpoint Roll/Pitch cho PID cân bằng.
            roll_sp, pitch_sp = rc_axis_to_sbal_setpoint(rx, ry)
            now = time.time()

            need_send = False
            if last_sbal_roll_sp is None or last_sbal_pitch_sp is None:
                need_send = True
            elif abs(roll_sp - last_sbal_roll_sp) > SBAL_SP_SEND_DELTA_DEG:
                need_send = True
            elif abs(pitch_sp - last_sbal_pitch_sp) > SBAL_SP_SEND_DELTA_DEG:
                need_send = True

            if need_send and (now - last_sbal_sp_send_time) >= SBAL_SP_SEND_MIN_INTERVAL:
                if send_sbal_setpoint_from_rc(roll_sp, pitch_sp):
                    last_sbal_roll_sp = roll_sp
                    last_sbal_pitch_sp = pitch_sp
                    last_sbal_sp_send_time = now

            # Không gửi lệnh di chuyển khi SBAL đang bật.
            cmd = "BAL_LOCK"
            dx = pot - CENTER
            dy = ly - CENTER
            rdx = rx - CENTER
            rdy = ry - CENTER
            moving_active = False
        else:
            cmd, dx, dy, rdx, rdy = joystick_to_cmd(ly, pot, rx, ry)
            last_cmd, moving_active = send_motion_cmd(cmd, last_cmd, moving_active)

        if PRINT_RC_DEBUG:
            print(
                f"[RC] ly={ly:3d} pot={pot:3d} rx={rx:3d} ry={ry:3d} "
                f"b1={int(b1)} b2={int(b2)} b3={int(b3)} b4={int(b4)} "
                f"b5={int(b5)} b6={int(b6)} b8={int(b8)} b11={int(b11)} b14={int(b14)} "
                f"cam={int(cam_pid_enabled)} sbal={int(sbal_enabled)} "
                f"dx={dx:4d} dy={dy:4d} rdx={rdx:4d} rdy={rdy:4d} "
                f"cmd={cmd} last={last_cmd} moving={int(moving_active)}"
            )

    sock.close()


def stats_loop():
    global serial_line_count, imu_rx_count, imu_udp_count, ik12_rx_count, ik12_udp_count
    last_t = time.time()
    last_serial = last_imu_rx = last_imu_udp = last_ik12_rx = last_ik12_udp = 0

    while running:
        time.sleep(1.0)
        now = time.time()
        dt = max(now - last_t, 1e-6)
        with stats_lock:
            s = serial_line_count
            irx = imu_rx_count
            iudp = imu_udp_count
            krx = ik12_rx_count
            kudp = ik12_udp_count

        print(
            f"[RATE] serial={((s-last_serial)/dt):5.1f} lines/s | "
            f"IMU rx={((irx-last_imu_rx)/dt):5.1f}/s udp={((iudp-last_imu_udp)/dt):5.1f}/s | "
            f"IK12 rx={((krx-last_ik12_rx)/dt):5.1f}/s udp={((kudp-last_ik12_udp)/dt):5.1f}/s"
        )

        last_t = now
        last_serial, last_imu_rx, last_imu_udp, last_ik12_rx, last_ik12_udp = s, irx, iudp, krx, kudp

# ============================================================
# MAIN
# ============================================================

def main():
    global ser, running

    print("=" * 70)
    print("RASPBERRY FULL BRIDGE: RC + GUI OFFSET + CAMERA + IMU + IK12")
    print("=" * 70)
    print(f"Laptop IP              : {LAPTOP_IP}")
    print(f"Serial robot           : {SERIAL_PORT} @ {SERIAL_BAUD}")
    print(f"RC UDP listen          : {RC_UDP_PORT}")
    print(f"GUI offset listen      : {GUI_OFFSET_LISTEN_PORT}")
    print("GUI manual cmds        : FWD/BWD/LEFT/RIGHT/SL/SR/STOP/STAND/SIT/LIE/SHAKE/X+/X-/X_RESET")
    print("GUI balance cmds       : SBAL_ON/SBAL_OFF/SHOWSBAL/SET_SBAL/SET_SBAL_IMAX/SET_SBAL_SP")
    print("RC camera tracking     : B11 toggle CAM PID/BÃ¡m ngÆ°á»i, cáº­p nháº­t GUI port 6007")
    print("RC balance             : B14 toggle SBAL, Joy2 -> Roll/Pitch SP Â±15deg")
    print(f"Camera send            : {CAMERA_SEND_PORT}")
    print(f"IMU send               : {IMU_SEND_PORT}")
    print(f"IK12 send              : {IK12_SEND_PORT}")
    print(f"Camera config          : {CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, Q={JPEG_QUALITY}")
    print("=" * 70)

    ser = open_robot_serial()
    if ser is None:
        print("[WARN] ChÃï¿½ÃÂ°a cÃï¿½ÃÂ³ Serial ESP. Camera/UDP vÃÂ¡ÃÂºÃÂ«n chÃÂ¡ÃÂºÃÂ¡y nhÃï¿½ÃÂ°ng khÃï¿½ÃÂ´ng Ãï¿½Ã¯Â¿Â½iÃÂ¡ÃÂ»Ã¯Â¿Â½u khiÃÂ¡ÃÂ»Ã¯Â¿Â½n Ãï¿½Ã¯Â¿Â½Ãï¿½ÃÂ°ÃÂ¡ÃÂ»ÃÂ£c robot.")

    threads = [
        threading.Thread(target=serial_read_loop, daemon=True),
        threading.Thread(target=gui_offset_loop, daemon=True),
        threading.Thread(target=camera_send_loop, daemon=True),
        threading.Thread(target=rc_udp_loop, daemon=True),
        threading.Thread(target=stats_loop, daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        while running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C -> gÃÂ¡ÃÂ»ÃÂ­i STOP vÃï¿½ÃÂ  thoÃï¿½ÃÂ¡t")
        send_uart_once("STOP", tag="EXIT")
    finally:
        running = False
        time.sleep(0.5)
        try:
            if ser:
                ser.close()
        except Exception:
            pass
        try:
            imu_sock.close()
        except Exception:
            pass
        try:
            ik12_sock.close()
        except Exception:
            pass
        try:
            gui_status_sock.close()
        except Exception:
            pass
        print("[MAIN] Ãï¿½Ã¯Â¿Â½Ãï¿½ÃÂ£ Ãï¿½Ã¯Â¿Â½Ãï¿½ÃÂ³ng")


if __name__ == "__main__":
    main()
