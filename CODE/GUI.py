#!/usr/bin/env python3

import time
import socket
import struct
import threading
import json
import os
import math
import csv
from datetime import datetime
from collections import deque
from typing import List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import cv2
import numpy as np

from PIL import Image, ImageTk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import mediapipe as mp

# ============================================================
# Thứ tự gửi setpoint SBAL
# ============================================================
# Quy ước joystick: ngang trái/phải điều chỉnh Roll, dọc lên/xuống điều chỉnh Pitch.
# ESP đang nhận lệnh SET_SBAL_SP theo thứ tự Pitch trước, Roll sau,
# vì vậy GUI gửi theo thứ tự PITCH_ROLL để trục dọc điều khiển đúng Pitch.
SBAL_SP_SEND_ORDER = "PITCH_ROLL"

# ============================================================
# ROS 2 tùy chọn - publish góc khớp sang Gazebo
# ============================================================
ROS_OK = False
ROS_ERR = ""
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import Imu
    from builtin_interfaces.msg import Duration
    ROS_OK = True
except Exception as e:
    ROS_ERR = str(e)
    rclpy = None
    Node = object
    Imu = object


# ============================================================
# Camera từ Raspberry
# ============================================================
UDP_IP = "0.0.0.0"
CAMERA_UDP_PORT = 5005

# ============================================================
# Offset từ GUI gửi về Raspberry
# ============================================================
RASPBERRY_IP = "192.168.0.107"
RASPBERRY_OFFSET_PORT = 6006

# ============================================================
# Điều chỉnh chiều cao: GUI gửi trực tiếp X+ / X- xuống ESP thông qua Raspberry.
# ESP bản mới xử lý X+ / X- để thay đổi trục X cho các pose/gait FWD, BWD, LEFT, RIGHT, STOP, SL và SR.
# ============================================================
HEIGHT_DEFAULT_X = -190.0
HEIGHT_STEP_X = 10.0
HEIGHT_MIN_X = -230.0
HEIGHT_MAX_X = -150.0

# ============================================================
# Thông số gait mặc định
# GUI gửi thông số xuống ESP qua Raspberry bằng lệnh:
#   SET_GAIT MODE hStep tCycle phase swingRatio
# Ý nghĩa tham số trong ESP:
#   hStep      : độ nhấc chân theo trục X trong pha vung.
#   tCycle     : thời gian hoàn thành một chu kỳ bước, đơn vị giây.
#   phase      : độ lệch pha giữa hai nhóm chân FL+RR và FR+RL, trong khoảng 0..1.
#   swingRatio : tỉ lệ thời gian chân ở pha vung, trong khoảng 0.05..0.49.
# ============================================================
GAIT_CONFIG_PATH = os.path.expanduser("~/.quadruped_gait_config.json")
GAIT_MODES = ["FWD", "BWD", "LEFT", "RIGHT", "SL", "SR"]
GAIT_DEFAULTS = {
    "FWD":   {"hStep": 100.0, "tCycle": 0.70, "phase": 0.50, "swing": 0.35},
    "BWD":   {"hStep": 100.0, "tCycle": 0.70, "phase": 0.50, "swing": 0.35},
    "LEFT":  {"hStep":  50.0, "tCycle": 0.40, "phase": 0.50, "swing": 0.35},
    "RIGHT": {"hStep":  50.0, "tCycle": 0.40, "phase": 0.50, "swing": 0.35},
    "SL":    {"hStep":  50.0, "tCycle": 0.70, "phase": 0.50, "swing": 0.35},
    "SR":    {"hStep":  50.0, "tCycle": 0.70, "phase": 0.50, "swing": 0.35},
}

# ============================================================
# Tọa độ pose mặc định - đồng bộ với pose trong code ESP.
# Mỗi pose gồm tọa độ của bốn chân: FL(x,y,z), FR(x,y,z), RL(x,y,z), RR(x,y,z).
# ============================================================
POSE_LEGS = ["FL", "FR", "RL", "RR"]
POSE_AXES = ["x", "y", "z"]
POSE_DEFAULTS = {
    "STAND": {"FL": [-190.0, -44.2, 30.0], "FR": [-190.0, 44.2, 40.0], "RL": [-190.0, -44.2, 40.0], "RR": [-190.0, 44.2, 40.0]},
    "SIT":   {"FL": [-230.0, -50.0, 40.0], "FR": [-230.0, 50.0, 40.0], "RL": [-120.0, -100.0, -10.0], "RR": [-120.0, 100.0, -10.0]},
    "LIE":   {"FL": [-80.0, -100.0, -10.0], "FR": [-80.0, 100.0, -10.0], "RL": [-80.0, -100.0, -10.0], "RR": [-80.0, 100.0, -10.0]},
    "SHAKE": {"FL": [-225.0, -55.0, 35.0], "FR": [-230.0, 0.0, 40.0], "RL": [-120.0, -100.0, -10.0], "RR": [-120.0, 100.0, -10.0]},
    "STAMP": {"FL": [-210.0, -55.0, 0.0], "FR": [-210.0, 55.0, 0.0], "RL": [-210.0, -55.0, 20.0], "RR": [-210.0, 55.0, 20.0]},
    "FWD":   {"FL": [-197.0, -55.0, 0.0], "FR": [-210.0, 55.0, 0.0], "RL": [-197.0, -55.0, 20.0], "RR": [-210.0, 55.0, 20.0]},
    "BWD":   {"FL": [-201.0, -55.0, 0.0], "FR": [-210.0, 55.0, 0.0], "RL": [-201.0, -55.0, 20.0], "RR": [-210.0, 55.0, 20.0]},
    "LEFT":  {"FL": [-210.0, 0.0, 0.0], "FR": [-210.0, 100.0, 0.0], "RL": [-210.0, 0.0, 20.0], "RR": [-210.0, 100.0, 20.0]},
    "RIGHT": {"FL": [-210.0, -100.0, 0.0], "FR": [-210.0, 0.0, 0.0], "RL": [-210.0, -100.0, 20.0], "RR": [-210.0, 0.0, 20.0]},
    "SIDE":  {"FL": [-202.0, -55.0, 0.0], "FR": [-210.0, 55.0, 0.0], "RL": [-202.0, -55.0, 20.0], "RR": [-210.0, 55.0, 20.0]},
}
POSE_CONFIG_PATH = os.path.expanduser("~/.quadruped_pose_coords.json")

# ============================================================
# GUI nhận dữ liệu IMU từ Raspberry
# ============================================================
IMU_LISTEN_IP = "0.0.0.0"
IMU_LISTEN_PORT = 7007

# ============================================================
# Camera và MediaPipe
# ============================================================
WIDTH = 640
HEIGHT = 480

GAUSS_KERNEL = (5, 5)
GAUSS_SIGMA = 0

CENTER_DEADBAND_PX = 25

MIN_DETECTION_CONF = 0.5
MIN_TRACKING_CONF = 0.5
FULL_BODY_VISIBILITY_TH = 0.45

SEND_INTERVAL = 0.05
LPF_ALPHA = 0.85

BOX_H_LIMIT = 360

# ============================================================
# Giảm offset theo bề rộng khung người
# ============================================================
# Khi box_width_lpf <= OFFSET_SCALE_REF_BOX_W, offset gửi xuống bridge được giữ nguyên.
# Khi box_width_lpf lớn hơn giá trị tham chiếu, offset_send = offset_lpf * (REF / box_width_lpf).
# OFFSET_SCALE_MIN_RATIO giới hạn mức giảm nhỏ nhất để offset không bị thu nhỏ về 0.
# Ví dụ với REF=260, box_width_lpf=520 và min_ratio=0.25:
#   scale = max(260/520, 0.25) = 0.50, nghĩa là offset gửi đi còn 50%.
OFFSET_SCALE_BY_BOX_W_ENABLE = 1
OFFSET_SCALE_REF_BOX_W = 260
OFFSET_SCALE_MIN_RATIO = 0.25

# Lệnh từ GUI gửi về Pi, sau đó Pi chuyển tiếp xuống ESP.
AUTO_STAND_BEFORE_FWD = False

SEND_PID_SETTING_ON_START = False
CAM_PID_CMD = "SET_CAMGAIT 0.18 0.00 0.04 35 25 6 120"

SEND_STOP_SETTING_ON_START = False
CAM_STOP_CMD = "SET_CAMSTOP 360 800"

# ============================================================
# Cấu hình đồ thị
# ============================================================
PLOT_SECONDS_WINDOW = 5
PLOT_MAX_POINTS = 400

ROLL_YLIM = (-20, 20)
PITCH_YLIM = (-20, 20)
YAW_YLIM = (-180, 180)

# ============================================================
# Làm mượt đồ thị Balance theo thời gian thực
# ============================================================
# Các thông số này chỉ dùng cho đường vẽ live ở tab Balance, giúp hạn chế giật do spike hoặc rung UDP.
# Dữ liệu ghi vào file CSV vẫn là dữ liệu raw, không bị lọc.
BALANCE_PLOT_FILTER_ENABLE = 0       # Không lọc ở GUI. ESP đã lọc rồi, GUI chỉ vẽ raw mới nhất.
BALANCE_PLOT_LPF_ALPHA = 0.0         # Giữ lại để tương thích file config cũ, không dùng cho Balance plot.
BALANCE_PLOT_MAX_STEP_DEG = 999.0    # Giữ lại để tương thích file config cũ, không dùng cho Balance plot.
BALANCE_PLOT_FIXED_Y = 1             # 1 = giữ trục Y cố định -20..20 để hình không bị giật do autoscale.
BALANCE_PLOT_UPDATE_MS = 30          # Chu kỳ vẽ Balance. 30 ms tương đương khoảng 33 Hz.

# Giới hạn kích thước hiển thị camera để khung giao diện gọn hơn.
CAMERA_DISPLAY_MAX_W = 760
CAMERA_DISPLAY_MAX_H = 570

# ============================================================
# Thông tin hiển thị trên GUI
# ============================================================
UNIVERSITY_NAME = "TRƯỜNG ĐẠI HỌC CÔNG NGHỆ KỸ THUẬT TP.HCM"
PROJECT_TITLE = "ĐỀ TÀI: THIẾT KẾ, CHẾ TẠO VÀ ĐIỀU KHIỂN ROBOT 4 CHÂN"
SUPERVISOR_NAME = "GVHD: PGS.TS NGUYỄN MINH TÂM"
STUDENT_NAMES = "SVTH: ĐẶNG MẠNH TRƯỜNG - HUỲNH HUY HOÀNG"

LEFT_LOGO_FILES = [
    "logo FEEE.jpg", "logo FEEE.jpeg", "logo FEEE.png",
    "logo FEEE.JPG", "logo FEEE.JPEG", "logo FEEE.PNG"
]
RIGHT_LOGO_FILES = [
    "LOGO UTE.png", "LOGO UTE.PNG", "LOGO UTE.jpg", "LOGO UTE.jpeg",
    "HCMUTE.png", "hcmute.png"
]
UTE_LOGO_TARGET_HEIGHT = 110
FEEE_LOGO_HEIGHT_RATIO = 0.8
LOGO_TEXT_GAP_X = 28

# ============================================================
# Lưu và tải cấu hình
# ============================================================
CONFIG_PATH = os.path.expanduser("~/.cam_udp_pose_gui_config.json")
REC_DEFAULT_DIR = os.path.expanduser("~/Desktop")

MODEL_COMPLEXITY = 0
CAMERA_DISPLAY_SCALE = 0.96
PLOT_UPDATE_MS = 80

# ============================================================
# Tinh chỉnh hiệu năng và FPS, có thể thay đổi trong tab Cài đặt thông số.
# ============================================================
# 1 = xử lý MediaPipe ở mọi frame; 2/3/4 = xử lý cách frame để GUI nhẹ và mượt hơn.
CAM_PROCESS_EVERY_N = 3
# 1 = chỉ chạy MediaPipe khi bật chế độ bám người; 0 = luôn chạy MediaPipe.
CAM_MEDIAPIPE_ONLY_WHEN_PID = 1
# 1 = vẽ skeleton MediaPipe; 0 = không vẽ skeleton để giảm tải CPU.
CAM_DRAW_LANDMARKS = 0
# Chu kỳ gọi update_frame của Tkinter, đơn vị ms; giá trị nhỏ giúp mượt hơn nhưng tốn CPU hơn.
CAM_UPDATE_MS = 10
# 1 = bật publish JointTrajectory sang Gazebo; 0 = tắt mặc định để GUI nhẹ hơn.
GAZEBO_PUBLISH_ENABLE = 0

# ============================================================
# Chế độ ưu tiên Balance
# Khi chuyển sang tab Balance, GUI sẽ tạm dừng các tác vụ nặng
# như decode/hiển thị camera, publish Gazebo và vẽ plot Gazebo để giảm lag.
# Luồng IMU UDP và chức năng ghi CSV vẫn hoạt động bình thường.
# ============================================================
BALANCE_FOCUS_MODE_ENABLE = 1
BALANCE_PAUSE_CAMERA = 1
BALANCE_PAUSE_GAZEBO_PUBLISH = 1
BALANCE_PAUSE_GAZEBO_IMU_PLOT = 1
BALANCE_PAUSE_JOINT_TABLE_UI = 1


# ============================================================
# Cấu hình nhận Joint UDP và mapping sang Gazebo
# ============================================================
JOINT_UDP_LISTEN_IP = "0.0.0.0"
JOINT_UDP_LISTEN_PORT = 6001
JOINT_CONFIG_PATH = os.path.expanduser("~/.quadruped_gazebo_joint_mapper.json")
JOINT_DEFAULT_TOPIC = "/joint_trajectory_controller/joint_trajectory"
JOINT_PUBLISH_HZ_LIMIT = 10.0
JOINT_POINT_TIME_SEC = 0.02
JOINT_UI_UPDATE_MS = 50

# Topic IMU của Gazebo dùng để so sánh với IMU thực tế nhận từ Raspberry qua UDP 7007.
GAZEBO_IMU_TOPIC = "/imu"
GAZEBO_IMU_PLOT_MAX_POINTS = 500
GAZEBO_IMU_PLOT_WINDOW_SEC = 10.0
GAZEBO_IMU_PLOT_UPDATE_MS = 80


RAW_JOINT_ORDER = [
    "FL_j1", "FL_j2", "FL_j3",
    "FR_j1", "FR_j2", "FR_j3",
    "RL_j1", "RL_j2", "RL_j3",
    "RR_j1", "RR_j2", "RR_j3",
]

# Thứ tự publish được giữ giống cấu hình controller trong Gazebo:
# JFL -> JFR -> JBL -> JBR
GAZEBO_PUBLISH_ORDER = [
    "JFL1", "JFL2", "JFL3",
    "JFR1", "JFR2", "JFR3",
    "JBL1", "JBL2", "JBL3",
    "JBR1", "JBR2", "JBR3",
]

# Dữ liệu đầu vào là góc IK raw sau tính toán động học nghịch, không phải góc servo đã quy đổi.
# Các joint thật trong Gazebo gồm: JBL1..3, JBR1..3, JFL1..3 và JFR1..3.
# ESP/Raspberry gửi dữ liệu theo thứ tự FL, FR, RL, RR nên cần mapping như dưới.
# FL -> JFL, FR -> JFR, RL -> JBL, RR -> JBR.
DEFAULT_JOINT_MAPPING = [
    {"src": "FL_j1", "joint": "JFL1", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "FL_j2", "joint": "JFL2", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "FL_j3", "joint": "JFL3", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "FR_j1", "joint": "JFR1", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "FR_j2", "joint": "JFR2", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "FR_j3", "joint": "JFR3", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RL_j1", "joint": "JBL1", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RL_j2", "joint": "JBL2", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RL_j3", "joint": "JBL3", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RR_j1", "joint": "JBR1", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RR_j2", "joint": "JBR2", "sign":  1.0, "offset_deg": 0.0, "enable": True},
    {"src": "RR_j3", "joint": "JBR3", "sign":  1.0, "offset_deg": 0.0, "enable": True},
]

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles


class UdpCameraReceiver:
    def __init__(self, ip="0.0.0.0", port=5005):
        self.ip = ip
        self.port = port

        self.sock = None
        self.running = False
        self.thread = None

        self.frames = {}
        self.latest_frame = None
        self.lock = threading.Lock()

        self.last_packet_time = 0.0
        self.recv_fps = 0.0
        self._fps_count = 0
        self._fps_last_time = time.time()

        # Khi vào tab Balance, có thể tạm dừng decode JPEG để giảm tải CPU.
        # Socket vẫn được đọc để tránh đầy buffer, nhưng frame sẽ được bỏ qua.
        self.paused = False

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.1)

        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

        print(f"[UDP CAM] Dang nhan camera tai {self.ip}:{self.port}")

    def stop(self):
        self.running = False
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def set_paused(self, paused: bool):
        self.paused = bool(paused)
        if self.paused:
            # Xóa các chunk/frame cũ để khi bật lại không hiển thị frame tồn đọng.
            with self.lock:
                self.frames.clear()
                self.latest_frame = None
            self.recv_fps = 0.0
            self._fps_count = 0
            self._fps_last_time = time.time()

    def get_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def _recv_loop(self):
        while self.running:
            try:
                packet, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                print("[UDP CAM ERR]", e)
                continue

            if self.paused:
                # Ở chế độ ưu tiên Balance, camera được bỏ qua ngay từ mức gói UDP,
                # nhờ đó tránh ghép chunk và decode JPEG gây lag GUI.
                continue

            if len(packet) < 8:
                continue

            header = packet[:8]
            chunk = packet[8:]

            try:
                frame_id, total_chunks, chunk_id = struct.unpack("!IHH", header)
            except Exception:
                continue

            if frame_id not in self.frames:
                self.frames[frame_id] = {
                    "total": total_chunks,
                    "chunks": {}
                }

            self.frames[frame_id]["chunks"][chunk_id] = chunk

            if len(self.frames[frame_id]["chunks"]) == total_chunks:
                try:
                    data = b"".join(
                        self.frames[frame_id]["chunks"][i]
                        for i in range(total_chunks)
                    )
                except KeyError:
                    self.frames.pop(frame_id, None)
                    continue

                self.frames.pop(frame_id, None)

                jpg_array = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                with self.lock:
                    self.latest_frame = frame

                self.last_packet_time = time.time()

                self._fps_count += 1
                now = time.time()
                if now - self._fps_last_time >= 1.0:
                    self.recv_fps = self._fps_count / (now - self._fps_last_time)
                    self._fps_count = 0
                    self._fps_last_time = now

                if len(self.frames) > 40:
                    old_keys = sorted(self.frames.keys())[:-10]
                    for k in old_keys:
                        self.frames.pop(k, None)


class UdpImuReceiver:
    def __init__(self, ip="0.0.0.0", port=7007):
        self.ip = ip
        self.port = port

        self.sock = None
        self.running = False
        self.thread = None

        self.lock = threading.Lock()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.last_packet_time = 0.0
        self.recv_count = 0
        self.recv_hz = 0.0
        self._count_t0 = time.time()
        self._count_n = 0
        self.sample_seq = 0
        self.sample_buffer = deque(maxlen=10000)  # lưu tất cả mẫu UDP nhận được để REC không bị giới hạn bởi GUI update

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.1)

        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

        print(f"[UDP IMU] Dang nhan IMU tai {self.ip}:{self.port}")

    def stop(self):
        self.running = False

        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def get_imu(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.last_packet_time

    def get_recv_hz(self):
        # Tần số thực tế GUI nhận gói IMU từ Raspberry/ESP, tính trong cửa sổ khoảng 1 giây.
        with self.lock:
            return self.recv_hz

    def get_latest_sample(self):
        # Dùng cho live plot: lấy đúng mẫu mới nhất, không lọc và không vẽ dồn buffer.
        with self.lock:
            return (
                self.sample_seq,
                self.last_packet_time,
                self.roll,
                self.pitch,
                self.yaw,
                self.recv_hz,
            )

    def get_latest_seq(self):
        with self.lock:
            return self.sample_seq

    def get_samples_since(self, last_seq):
        # Trả về toàn bộ mẫu mới GUI nhận từ Raspberry, không phụ thuộc tốc độ cập nhật giao diện.
        with self.lock:
            samples = [s for s in self.sample_buffer if s[0] > last_seq]
            latest_seq = self.sample_seq
        return samples, latest_seq

    def _recv_loop(self):
        while self.running:
            try:
                packet, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                print("[UDP IMU ERR]", e)
                continue

            msg = packet.decode("utf-8", errors="ignore").strip()
            parts = msg.split(",")

            if len(parts) < 3:
                continue

            try:
                roll = float(parts[0])
                pitch = float(parts[1])
                yaw = float(parts[2])
            except Exception:
                continue

            now = time.time()
            with self.lock:
                self.roll = roll
                self.pitch = pitch
                self.yaw = yaw
                self.last_packet_time = now
                self.recv_count += 1
                self._count_n += 1
                dt = now - self._count_t0
                if dt >= 1.0:
                    self.recv_hz = self._count_n / dt
                    self._count_n = 0
                    self._count_t0 = now
                self.sample_seq += 1
                self.sample_buffer.append((self.sample_seq, now, roll, pitch, yaw))


# ============================================================
# Panel mapping góc khớp sang Gazebo
# ============================================================
class GazeboJointPublisher(Node):
    def __init__(self, topic: str):
        super().__init__("quadruped_joint_mapper_gui")
        self.topic = topic

        # Controller đang subscribe ở chế độ BEST_EFFORT, nên publisher cũng dùng BEST_EFFORT để tương thích.
        try:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.pub = self.create_publisher(JointTrajectory, topic, qos)
        except Exception:
            self.pub = self.create_publisher(JointTrajectory, topic, 10)

    def publish_positions(self, joint_names: List[str], positions_rad: List[float], point_time_sec: float):
        # Format publish giống cấu hình mẫu của Gazebo:
        # msg.joint_names = ['JFL1','JFL2','JFL3','JFR1','JFR2','JFR3','JBL1','JBL2','JBL3','JBR1','JBR2','JBR3']
        # point.positions dùng đơn vị radian và cùng thứ tự với danh sách joint.
        msg = JointTrajectory()
        msg.joint_names = list(joint_names)

        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions_rad]
        sec = int(point_time_sec)
        p.time_from_start = Duration(
            sec=sec,
            nanosec=int((point_time_sec - sec) * 1e9),
        )
        msg.points.append(p)
        self.pub.publish(msg)


class UdpJointReceiver:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.latest_deg = [0.0] * 12
        self.latest_order = list(RAW_JOINT_ORDER)
        self.seq = 0
        self.last_packet_time = 0.0
        self.recv_hz = 0.0
        self._count_t0 = time.time()
        self._count_n = 0

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.1)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[JOINT UDP] Listen on {self.ip}:{self.port}")

    def stop(self):
        self.running = False
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def get_latest(self):
        with self.lock:
            return list(self.latest_deg), list(self.latest_order), self.seq, self.last_packet_time, self.recv_hz

    def _parse_packet(self, data: bytes):
        s = data.decode("utf-8", errors="ignore").strip()
        if not s:
            return None

        # Định dạng mới từ Raspberry: JSON type=ik12_deg, trường deg chứa đủ 12 góc IK raw sau tính toán.
        if s.startswith("{"):
            obj = json.loads(s)
            msg_type = str(obj.get("type", ""))
            if msg_type and msg_type not in ("ik12_deg", "joint_angles_deg"):
                return None
            deg = [float(x) for x in obj.get("deg", [])]
            if len(deg) != 12:
                return None
            order = obj.get("order", RAW_JOINT_ORDER)
            if len(order) != 12:
                order = RAW_JOINT_ORDER
            seq = int(obj.get("seq", 0))
            return deg, list(order), seq

        # Định dạng dự phòng khi gửi dạng text: IK12,<12 số>.
        upper = s.upper()
        for prefix in ("IK12", "RAWIK12", "IK12DEG", "RAW_IK12"):
            if upper.startswith(prefix):
                body = s[len(prefix):].strip(" :;,	")
                parts = body.replace(";", ",").replace(" ", ",").split(",")
                nums = []
                for p in parts:
                    p = p.strip()
                    if not p:
                        continue
                    try:
                        nums.append(float(p))
                    except Exception:
                        pass
                if len(nums) == 12:
                    return nums, list(RAW_JOINT_ORDER), 0
        return None

    def _loop(self):
        while self.running:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                print("[JOINT UDP] recv error:", e)
                continue
            try:
                parsed = self._parse_packet(data)
            except Exception as e:
                print("[JOINT UDP] parse error:", e)
                continue
            if parsed is None:
                continue
            deg, order, seq = parsed
            now = time.time()
            with self.lock:
                self.latest_deg = deg
                self.latest_order = order
                self.seq = seq
                self.last_packet_time = now
                self._count_n += 1
                dt = now - self._count_t0
                if dt >= 1.0:
                    self.recv_hz = self._count_n / dt
                    self._count_n = 0
                    self._count_t0 = now


class JointMapperPanel:
    def __init__(self, parent):
        self.parent = parent
        self.mapping = [dict(x) for x in DEFAULT_JOINT_MAPPING]
        self.publish_enabled = tk.BooleanVar(value=bool(GAZEBO_PUBLISH_ENABLE))
        self.deg_to_rad_enabled = tk.BooleanVar(value=True)
        self.point_time_sec = tk.DoubleVar(value=JOINT_POINT_TIME_SEC)
        self.udp_port_var = tk.IntVar(value=JOINT_UDP_LISTEN_PORT)
        self.topic_var = tk.StringVar(value=JOINT_DEFAULT_TOPIC)
        self.receiver = None
        self.ros_node = None
        self.ros_thread = None
        self.ros_running = False
        self.ros_ready = False
        self.last_pub_time = 0.0
        self.pub_count = 0
        self.pub_hz = 0.0
        self._pub_count_t0 = time.time()
        self._pub_count_n = 0
        self.raw_deg = [0.0] * 12
        self.mapped_rad = [0.0] * 12
        self.ui_paused = False

        # Cache dữ liệu để publish liên tục cho Gazebo; việc cập nhật UDP/UI chạy riêng,
        # còn thread này giữ lệnh mới nhất và publish đều theo tần số đặt trước.
        self.pub_lock = threading.Lock()
        self.latest_joint_names = []
        self.latest_positions = []
        self.latest_seq_for_publish = -1
        self.publish_thread_running = True
        self.publish_thread = None

        self.load_config()
        self.build_ui()
        self.start_udp()
        self.start_ros()
        self.start_continuous_publish_thread()
        self.update_loop()

    def is_mapping_for_this_gazebo(self, mapping):
        wanted = {"JBL1", "JBL2", "JBL3", "JBR1", "JBR2", "JBR3", "JFL1", "JFL2", "JFL3", "JFR1", "JFR2", "JFR3"}
        try:
            joints = {str(m.get("joint", "")).strip() for m in mapping}
        except Exception:
            return False
        return wanted.issubset(joints)

    def force_default_mapping(self, save=False):
        self.mapping = [dict(x) for x in DEFAULT_JOINT_MAPPING]
        if hasattr(self, "tree"):
            self.refresh_tree_static()
        if save:
            cfg = {
                "mapping": self.mapping,
                "topic": self.topic_var.get().strip(),
                "point_time_sec": self.point_time_sec.get(),
                "deg_to_rad": self.deg_to_rad_enabled.get(),
                "publish_enabled": self.publish_enabled.get(),
                "udp_port": self.udp_port_var.get(),
            }
            try:
                with open(JOINT_CONFIG_PATH, "w", encoding="utf-8") as f:
                    json.dump(cfg, f, ensure_ascii=False, indent=2)
                print("[JOINT CONFIG] Saved default JBL/JBR/JFL/JFR mapping")
            except Exception as e:
                print("[JOINT CONFIG] save default error:", e)

    def load_config(self):
        if not os.path.exists(JOINT_CONFIG_PATH):
            self.force_default_mapping(save=False)
            return
        try:
            with open(JOINT_CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            loaded_mapping = cfg.get("mapping", self.mapping)

            # Nếu file cấu hình cũ còn các tên joint kiểu FL_hip_joint hoặc FR_thigh_joint thì bỏ qua và đặt lại mapping đúng cho Gazebo hiện tại.
            if not self.is_mapping_for_this_gazebo(loaded_mapping):
                print("[JOINT CONFIG] Old/invalid mapping detected -> reset to JBL/JBR/JFL/JFR")
                self.force_default_mapping(save=True)
            else:
                self.mapping = loaded_mapping

            self.topic_var.set(cfg.get("topic", self.topic_var.get()))
            self.point_time_sec.set(float(cfg.get("point_time_sec", self.point_time_sec.get())))
            self.deg_to_rad_enabled.set(bool(cfg.get("deg_to_rad", True)))
            self.publish_enabled.set(bool(cfg.get("publish_enabled", True)))
            self.udp_port_var.set(int(cfg.get("udp_port", JOINT_UDP_LISTEN_PORT)))
            print("[JOINT CONFIG] Loaded", JOINT_CONFIG_PATH)
        except Exception as e:
            print("[JOINT CONFIG] load error:", e)
            self.force_default_mapping(save=False)

    def save_config(self):
        self.read_mapping_from_table()
        cfg = {
            "mapping": self.mapping,
            "topic": self.topic_var.get().strip(),
            "point_time_sec": self.point_time_sec.get(),
            "deg_to_rad": self.deg_to_rad_enabled.get(),
            "publish_enabled": self.publish_enabled.get(),
            "udp_port": self.udp_port_var.get(),
        }
        try:
            with open(JOINT_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
            messagebox.showinfo("Lưu mapping", f"Đã lưu:\n{JOINT_CONFIG_PATH}")
        except Exception as e:
            messagebox.showerror("Lỗi lưu mapping", str(e))

    def build_ui(self):
        main = ttk.Frame(self.parent, style="TFrame", padding=16)
        main.pack(fill=tk.BOTH, expand=True)
        ttk.Label(main, text="MAPPING 12 GÓC CHÂN → GAZEBO", style="Title.TLabel").pack(anchor="w", pady=(0, 10))

        top = ttk.Frame(main, style="Panel.TFrame", padding=12)
        top.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(top, text="UDP Port nhận 12 góc:", style="Info.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.udp_port_var, width=8, font=("Consolas", 13)).grid(row=0, column=1, padx=(6, 14))
        ttk.Button(top, text="Restart UDP", style="Gray.TButton", command=self.restart_udp).grid(row=0, column=2, padx=(0, 16))
        ttk.Label(top, text="ROS Topic Gazebo:", style="Info.TLabel").grid(row=0, column=3, sticky="w")
        ttk.Entry(top, textvariable=self.topic_var, width=46, font=("Consolas", 13)).grid(row=0, column=4, padx=(6, 14))
        ttk.Button(top, text="Reconnect ROS", style="Blue.TButton", command=self.restart_ros).grid(row=0, column=5, padx=(0, 12))
        ttk.Checkbutton(top, text="Publish Gazebo", variable=self.publish_enabled).grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(top, text="deg → rad", variable=self.deg_to_rad_enabled).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(top, text="Point time:", style="Info.TLabel").grid(row=1, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(top, textvariable=self.point_time_sec, width=8, font=("Consolas", 13)).grid(row=1, column=4, sticky="w", padx=(6, 14), pady=(8, 0))
        ttk.Button(top, text="LƯU MAPPING", style="Green.TButton", command=self.save_config).grid(row=1, column=5, sticky="w", pady=(8, 0))
        ttk.Button(top, text="RESET JBL/JBR/JFL/JFR", style="Red.TButton", command=lambda: self.force_default_mapping(save=True)).grid(row=2, column=5, sticky="w", pady=(8, 0))

        self.status_label = ttk.Label(main, text="Status", style="BigImu.TLabel", padding=8)
        self.status_label.pack(fill=tk.X, pady=(0, 10))

        body = ttk.Frame(main, style="TFrame")
        body.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(body, style="Panel.TFrame", padding=10)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        cols = ("src", "raw_deg", "joint", "sign", "offset", "mapped_rad", "enable")
        self.tree = ttk.Treeview(left, columns=cols, show="headings", height=15)
        headings = {"src":"Nguồn", "raw_deg":"Raw deg", "joint":"Gazebo joint name", "sign":"Sign", "offset":"Offset deg", "mapped_rad":"Mapped rad", "enable":"Enable"}
        widths = {"src":80, "raw_deg":100, "joint":260, "sign":70, "offset":105, "mapped_rad":120, "enable":80}
        for c in cols:
            self.tree.heading(c, text=headings[c])
            self.tree.column(c, width=widths[c], anchor="center")
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<Double-1>", self.on_tree_double_click)
        ttk.Label(left, text="Double click vào joint/sign/offset/enable để sửa. Enable nhập 1/0.", style="Info.TLabel").pack(anchor="w", pady=(8, 0))

        right = ttk.Frame(body, style="Panel.TFrame", padding=10, width=390)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)
        ttk.Label(right, text="Raw 12 góc từ Raspberry", style="SubTitle.TLabel").pack(anchor="w")
        self.raw_text = tk.Text(right, height=10, font=("Consolas", 12), bg="#f1f5f9")
        self.raw_text.pack(fill=tk.X, pady=(6, 12))
        ttk.Label(right, text="Mapped publish rad", style="SubTitle.TLabel").pack(anchor="w")
        self.map_text = tk.Text(right, height=10, font=("Consolas", 12), bg="#f1f5f9")
        self.map_text.pack(fill=tk.X, pady=(6, 12))
        hint = "Công thức:\n  mapped_rad = radians(raw_deg * sign + offset_deg)\n\nNếu Gazebo ngược chiều: sign = -1.\nNếu lệch zero: chỉnh offset_deg.\nTên joint phải đúng controller Gazebo."
        ttk.Label(right, text=hint, style="Info.TLabel", justify="left", wraplength=360).pack(anchor="w")
        self.refresh_tree_static()

    def refresh_tree_static(self):
        self.tree.delete(*self.tree.get_children())
        for i, m in enumerate(self.mapping):
            self.tree.insert("", "end", iid=str(i), values=(m.get("src", RAW_JOINT_ORDER[i]), "0.00", m.get("joint", ""), f"{float(m.get('sign', 1.0)):.1f}", f"{float(m.get('offset_deg', 0.0)):.2f}", "0.0000", "1" if m.get("enable", True) else "0"))

    def on_tree_double_click(self, event):
        region = self.tree.identify("region", event.x, event.y)
        if region != "cell":
            return
        row_id = self.tree.identify_row(event.y)
        col_id = self.tree.identify_column(event.x)
        if not row_id or not col_id:
            return
        col_index = int(col_id.replace("#", "")) - 1
        cols = ("src", "raw_deg", "joint", "sign", "offset", "mapped_rad", "enable")
        col = cols[col_index]
        if col not in ("joint", "sign", "offset", "enable"):
            return
        x, y, w, h = self.tree.bbox(row_id, col_id)
        old = self.tree.set(row_id, col)
        entry = ttk.Entry(self.tree)
        entry.place(x=x, y=y, width=w, height=h)
        entry.insert(0, old)
        entry.focus()
        def commit(_evt=None):
            val = entry.get().strip()
            self.tree.set(row_id, col, val)
            entry.destroy()
            self.read_mapping_from_table()
        entry.bind("<Return>", commit)
        entry.bind("<FocusOut>", commit)

    def read_mapping_from_table(self):
        new_map = []
        for i in range(12):
            vals = self.tree.item(str(i), "values")
            try:
                sign = float(vals[3])
            except Exception:
                sign = 1.0
            try:
                offset = float(vals[4])
            except Exception:
                offset = 0.0
            enable_txt = str(vals[6]).strip().lower()
            enable = enable_txt not in ("0", "false", "no", "off", "")
            new_map.append({"src": vals[0], "joint": vals[2], "sign": sign, "offset_deg": offset, "enable": enable})
        self.mapping = new_map

    def start_udp(self):
        if self.receiver:
            self.receiver.stop()
        self.receiver = UdpJointReceiver(JOINT_UDP_LISTEN_IP, int(self.udp_port_var.get()))
        self.receiver.start()

    def restart_udp(self):
        self.start_udp()

    def start_ros(self):
        if not ROS_OK:
            self.ros_ready = False
            print("[ROS] Not available:", ROS_ERR)
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.ros_node = GazeboJointPublisher(self.topic_var.get().strip())
            self.ros_running = True
            self.ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self.ros_thread.start()
            self.ros_ready = True
            print("[ROS] Publishing to", self.topic_var.get().strip())
        except Exception as e:
            self.ros_ready = False
            print("[ROS] init error:", e)

    def restart_ros(self):
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
        except Exception:
            pass
        self.ros_node = None
        self.ros_ready = False
        self.start_ros()

    def _ros_spin_loop(self):
        while self.ros_running and self.ros_node is not None:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.02)
            except Exception:
                time.sleep(0.05)

    def compute_mapping(self, raw_deg: List[float]):
        self.read_mapping_from_table()

        mapped_all = []
        joint_to_val = {}
        joint_enable = {}

        for i, m in enumerate(self.mapping):
            raw = raw_deg[i]
            deg = raw * float(m.get("sign", 1.0)) + float(m.get("offset_deg", 0.0))
            val = math.radians(deg) if self.deg_to_rad_enabled.get() else deg
            mapped_all.append(val)

            joint = str(m.get("joint", "")).strip()
            if joint:
                joint_to_val[joint] = val
                joint_enable[joint] = bool(m.get("enable", True))

        # Publish theo đúng thứ tự controller Gazebo yêu cầu: JFL, JFR, JBL, JBR.
        # Thứ tự publish không phụ thuộc vào thứ tự hiển thị trong bảng.
        joint_names = []
        positions = []
        for joint in GAZEBO_PUBLISH_ORDER:
            if joint_enable.get(joint, False) and joint in joint_to_val:
                joint_names.append(joint)
                positions.append(joint_to_val[joint])

        return joint_names, positions, mapped_all

    def update_publish_cache(self, joint_names: List[str], positions: List[float], seq: int):
        if not joint_names or not positions:
            return
        if len(joint_names) != len(positions):
            return
        with self.pub_lock:
            self.latest_joint_names = list(joint_names)
            self.latest_positions = [float(x) for x in positions]
            self.latest_seq_for_publish = int(seq)

    def start_continuous_publish_thread(self):
        if self.publish_thread is not None and self.publish_thread.is_alive():
            return
        self.publish_thread_running = True
        self.publish_thread = threading.Thread(target=self._continuous_publish_loop, daemon=True)
        self.publish_thread.start()

    def _continuous_publish_loop(self):
        # Publish liên tục theo cấu hình Gazebo: tần số mục tiêu và time_from_start do GUI thiết lập.
        # Không phụ thuộc tốc độ refresh GUI, giúp Gazebo nhận lệnh đều hơn khi robot di chuyển.
        next_t = time.time()
        while self.publish_thread_running:
            hz = max(1.0, float(JOINT_PUBLISH_HZ_LIMIT))
            period = 1.0 / hz
            next_t += period

            if self.publish_enabled.get() and self.ros_ready and self.ros_node is not None:
                with self.pub_lock:
                    joint_names = list(self.latest_joint_names)
                    positions = list(self.latest_positions)

                if joint_names and positions and len(joint_names) == len(positions):
                    try:
                        self.ros_node.publish_positions(joint_names, positions, float(self.point_time_sec.get()))
                        now_pub = time.time()
                        self.last_pub_time = now_pub
                        self.pub_count += 1
                        self._pub_count_n += 1
                        dt_pub = now_pub - self._pub_count_t0
                        if dt_pub >= 1.0:
                            self.pub_hz = self._pub_count_n / dt_pub
                            self._pub_count_n = 0
                            self._pub_count_t0 = now_pub
                    except Exception as e:
                        print("[ROS] continuous publish error:", e)
                        time.sleep(0.05)

            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # Nếu GUI hoặc ROS bị nghẽn, reset nhịp để không dồn các lệnh cũ.
                next_t = time.time()

    def set_ui_paused(self, paused: bool):
        # Chỉ dừng cập nhật bảng, tree và text trong tab mapping ẩn.
        # Bộ nhận UDP IK12 vẫn chạy; publish sẽ được bật hoặc tắt riêng.
        self.ui_paused = bool(paused)

    def get_runtime_stats(self):
        """Trả về tần số nhận IK12 và tần số publish sang Gazebo để hiển thị trên tab Gazebo."""
        raw_deg, order, seq, last_t, recv_hz = self.receiver.get_latest() if self.receiver else ([0.0]*12, RAW_JOINT_ORDER, 0, 0, 0)
        now = time.time()
        rx_age = now - last_t if last_t > 0 else 999.0
        pub_age = now - self.last_pub_time if self.last_pub_time > 0 else 999.0
        return {
            "seq": seq,
            "rx_hz": float(recv_hz),
            "rx_age": float(rx_age),
            "pub_hz": float(self.pub_hz),
            "pub_age": float(pub_age),
            "pub_count": int(self.pub_count),
            "target_hz": float(JOINT_PUBLISH_HZ_LIMIT),
            "publish_enabled": bool(self.publish_enabled.get()),
        }

    def update_loop(self):
        if not hasattr(self.parent, "winfo_exists") or not self.parent.winfo_exists():
            return
        if getattr(self, "ui_paused", False):
            self.parent.after(max(200, JOINT_UI_UPDATE_MS), self.update_loop)
            return
        raw_deg, order, seq, last_t, recv_hz = self.receiver.get_latest() if self.receiver else ([0.0]*12, RAW_JOINT_ORDER, 0, 0, 0)
        joint_names, positions, mapped_all = self.compute_mapping(raw_deg)
        self.update_publish_cache(joint_names, positions, seq)
        age = time.time() - last_t if last_t > 0 else 999
        rx_state = "OK" if age < 1.0 else "WAIT"
        ros_state = "OK" if self.ros_ready else f"NO ROS ({ROS_ERR[:55]})"
        pub_age = time.time() - self.last_pub_time if self.last_pub_time > 0 else 999
        pub_state = "OK" if pub_age < 1.0 else "WAIT"
        self.status_label.config(
            text=(
                f"UDP {rx_state} | seq={seq} | IK12 rx={recv_hz:.1f} Hz | "
                f"ROS={ros_state} | GZB pub={self.pub_hz:.1f} Hz/{JOINT_PUBLISH_HZ_LIMIT:.0f} Hz | "
                f"pub_state={pub_state} | count={self.pub_count}"
            )
        )
        for i in range(12):
            iid = str(i)
            if self.tree.exists(iid):
                self.tree.set(iid, "raw_deg", f"{raw_deg[i]:.2f}")
                self.tree.set(iid, "mapped_rad", f"{mapped_all[i]:.4f}")
        self.raw_text.delete("1.0", tk.END)
        for i, name in enumerate(RAW_JOINT_ORDER):
            self.raw_text.insert(tk.END, f"{name:6s}: {raw_deg[i]:8.2f} deg\n")
        self.map_text.delete("1.0", tk.END)
        for i, m in enumerate(self.mapping):
            self.map_text.insert(tk.END, f"{m.get('joint','')[:18]:18s}: {mapped_all[i]:+8.4f} rad\n")
        self.parent.after(JOINT_UI_UPDATE_MS, self.update_loop)

    def close(self):
        self.publish_thread_running = False
        try:
            if self.receiver:
                self.receiver.stop()
        except Exception:
            pass
        self.ros_running = False
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
        except Exception:
            pass


class GazeboImuNode(Node):
    def __init__(self, topic: str):
        super().__init__("gazebo_imu_compare_gui")
        self.topic = topic
        self.lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = 0.0
        self.recv_count = 0
        self.create_subscription(Imu, topic, self.imu_callback, 10)

    @staticmethod
    def quat_to_rpy_deg(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return roll, pitch, yaw

    def imu_callback(self, msg):
        roll, pitch, yaw = self.quat_to_rpy_deg(msg.orientation)
        with self.lock:
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.last_time = time.time()
            self.recv_count += 1

    def get_imu(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.last_time, self.recv_count


class GazeboImuComparePanel:
    """Tab Gazebo chỉ dùng để so sánh IMU mô phỏng và IMU thực tế.

    JointMapperPanel vẫn chạy ẩn phía sau để nhận IK12 và publish JointTrajectory cho Gazebo.
    """
    def __init__(self, parent, app, hidden_joint_panel=None):
        self.parent = parent
        self.app = app
        self.hidden_joint_panel = hidden_joint_panel
        self.gazebo_node = None
        self.ros_thread = None
        self.ros_running = False
        self.ros_ready = False
        self.ros_error = ""

        self.t0 = time.time()
        self.t_data = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.real_roll = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.real_pitch = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.real_yaw = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.gz_roll = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.gz_pitch = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)
        self.gz_yaw = deque(maxlen=GAZEBO_IMU_PLOT_MAX_POINTS)

        self.build_ui()
        self.start_ros_imu()
        self.update_loop()

    def build_ui(self):
        main = ttk.Frame(self.parent, style="TFrame", padding=16)
        main.pack(fill=tk.BOTH, expand=True)

        title_row = ttk.Frame(main, style="TFrame")
        title_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(title_row, text="GAZEBO - SO SÁNH IMU MÔ PHỎNG VÀ THỰC TẾ", style="Title.TLabel").pack(side=tk.LEFT)

        info = ttk.Frame(main, style="Panel.TFrame", padding=12)
        info.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(info, text="Gazebo IMU topic:", style="Info.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.topic_var = tk.StringVar(value=GAZEBO_IMU_TOPIC)
        ttk.Entry(info, textvariable=self.topic_var, width=28, font=("Consolas", 14)).grid(row=0, column=1, sticky="w", padx=(0, 8))
        ttk.Button(info, text="Reconnect IMU Gazebo", style="Blue.TButton", command=self.reconnect_ros_imu).grid(row=0, column=2, padx=(0, 12))

        self.status_label = ttk.Label(info, text="Gazebo IMU: waiting... | Real IMU: waiting...", style="BigImu.TLabel", padding=8)
        self.status_label.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        info.columnconfigure(3, weight=1)

        values = ttk.Frame(main, style="Panel.TFrame", padding=12)
        values.pack(fill=tk.X, pady=(0, 10))
        self.real_value_label = ttk.Label(values, text="IMU thực tế: Roll 0.00° | Pitch 0.00° | Yaw 0.00°", style="Info.TLabel")
        self.gz_value_label = ttk.Label(values, text="IMU Gazebo: Roll 0.00° | Pitch 0.00° | Yaw 0.00°", style="Info.TLabel")
        self.freq_value_label = ttk.Label(
            values,
            text="Tần số: IK12 đọc 0.0 Hz | Publish GZB 0.0 Hz | Target 0.0 Hz",
            style="BigImu.TLabel",
            padding=6
        )
        self.real_value_label.pack(anchor="w")
        self.gz_value_label.pack(anchor="w", pady=(4, 0))
        self.freq_value_label.pack(anchor="w", fill=tk.X, pady=(8, 0))

        self.fig = Figure(figsize=(15, 7.5), dpi=100, facecolor="#ffffff")
        self.ax_roll = self.fig.add_subplot(311)
        self.ax_pitch = self.fig.add_subplot(312)
        self.ax_yaw = self.fig.add_subplot(313)

        self.line_real_roll, = self.ax_roll.plot([], [], linewidth=2.0, label="Real Roll", color="tab:blue")
        self.line_gz_roll, = self.ax_roll.plot([], [], linewidth=2.0, linestyle="--", label="Gazebo Roll", color="tab:cyan")
        self.line_real_pitch, = self.ax_pitch.plot([], [], linewidth=2.0, label="Real Pitch", color="tab:orange")
        self.line_gz_pitch, = self.ax_pitch.plot([], [], linewidth=2.0, linestyle="--", label="Gazebo Pitch", color="tab:red")
        self.line_real_yaw, = self.ax_yaw.plot([], [], linewidth=2.0, label="Real Yaw", color="tab:green")
        self.line_gz_yaw, = self.ax_yaw.plot([], [], linewidth=2.0, linestyle="--", label="Gazebo Yaw", color="tab:purple")

        for ax, title, ylim in [
            (self.ax_roll, "ROLL: thực tế vs Gazebo", ROLL_YLIM),
            (self.ax_pitch, "PITCH: thực tế vs Gazebo", PITCH_YLIM),
            (self.ax_yaw, "YAW: thực tế vs Gazebo", YAW_YLIM),
        ]:
            ax.set_facecolor("#ffffff")
            ax.set_title(title, fontsize=13, fontweight="bold", color="#0f172a")
            ax.set_ylabel("deg", fontsize=11)
            ax.set_ylim(*ylim)
            ax.grid(True, color="#cbd5e1", linewidth=0.8)
            ax.legend(loc="upper right", fontsize=9, facecolor="#ffffff", edgecolor="#cbd5e1")
            for spine in ax.spines.values():
                spine.set_color("#94a3b8")
        self.ax_yaw.set_xlabel("Time (s)", fontsize=11)
        self.fig.tight_layout(pad=1.5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=main)
        self.canvas.get_tk_widget().configure(bg="#ffffff", highlightthickness=0)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.help_label = ttk.Label(
            main,
            text="Tab này hiển thị IMU + tần số IK12 đọc được + tần số publish JointTrajectory sang Gazebo. Phần mapping IK12 vẫn chạy ẩn phía sau.",
            style="Small.TLabel"
        )
        self.help_label.pack(anchor="w", pady=(8, 0))

    def start_ros_imu(self):
        if not ROS_OK:
            self.ros_error = ROS_ERR
            self.ros_ready = False
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.gazebo_node = GazeboImuNode(self.topic_var.get().strip())
            self.ros_running = True
            self.ros_ready = True
            self.ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self.ros_thread.start()
        except Exception as e:
            self.ros_error = str(e)
            self.ros_ready = False

    def reconnect_ros_imu(self):
        self.close_ros_only()
        self.start_ros_imu()

    def _ros_spin_loop(self):
        while self.ros_running and self.gazebo_node is not None:
            try:
                rclpy.spin_once(self.gazebo_node, timeout_sec=0.02)
            except Exception as e:
                self.ros_error = str(e)
                self.ros_ready = False
                time.sleep(0.05)

    def close_ros_only(self):
        self.ros_running = False
        try:
            if self.gazebo_node is not None:
                self.gazebo_node.destroy_node()
        except Exception:
            pass
        self.gazebo_node = None
        self.ros_ready = False

    def update_loop(self):
        if not hasattr(self.parent, "winfo_exists") or not self.parent.winfo_exists():
            return

        # Khi đang ở tab Balance, tạm dừng vẽ plot Gazebo để nhường CPU cho đồ thị Balance.
        if (bool(BALANCE_PAUSE_GAZEBO_IMU_PLOT)
                and getattr(self.app, "balance_focus_mode", False)):
            self.parent.after(max(250, GAZEBO_IMU_PLOT_UPDATE_MS), self.update_loop)
            return

        # IMU thực tế nhận từ Raspberry qua UDP 7007.
        real_r = real_p = real_y = 0.0
        real_t = 0.0
        if getattr(self.app, "udp_imu", None) is not None:
            real_r, real_p, real_y, real_t = self.app.udp_imu.get_imu()

        # IMU mô phỏng nhận từ ROS topic /imu.
        gz_r = gz_p = gz_y = 0.0
        gz_t = 0.0
        gz_count = 0
        if self.gazebo_node is not None:
            gz_r, gz_p, gz_y, gz_t, gz_count = self.gazebo_node.get_imu()

        now = time.time()
        t = now - self.t0
        self.t_data.append(t)
        self.real_roll.append(real_r)
        self.real_pitch.append(real_p)
        self.real_yaw.append(real_y)
        self.gz_roll.append(gz_r)
        self.gz_pitch.append(gz_p)
        self.gz_yaw.append(gz_y)

        real_ok = (now - real_t) < 2.0 if real_t > 0 else False
        gz_ok = (now - gz_t) < 2.0 if gz_t > 0 else False

        hz_info = {
            "seq": 0,
            "rx_hz": 0.0,
            "rx_age": 999.0,
            "pub_hz": 0.0,
            "pub_age": 999.0,
            "pub_count": 0,
            "target_hz": float(JOINT_PUBLISH_HZ_LIMIT),
            "publish_enabled": False,
        }
        if self.hidden_joint_panel is not None:
            try:
                hz_info.update(self.hidden_joint_panel.get_runtime_stats())
            except Exception:
                hz_info["pub_count"] = getattr(self.hidden_joint_panel, "pub_count", 0)
                hz_info["pub_hz"] = getattr(self.hidden_joint_panel, "pub_hz", 0.0)

        ik12_ok = hz_info["rx_age"] < 1.0
        pub_ok = hz_info["pub_age"] < 1.0 and hz_info["publish_enabled"]
        hidden_status = (
            f" | IK12 rx={hz_info['rx_hz']:.1f}Hz ({'OK' if ik12_ok else 'WAIT'})"
            f" | GZB pub={hz_info['pub_hz']:.1f}Hz ({'OK' if pub_ok else 'WAIT/OFF'})"
        )

        self.status_label.config(
            text=f"Gazebo IMU: {'OK' if gz_ok else 'WAIT'} | Real IMU: {'OK' if real_ok else 'WAIT'} | ROS: {'OK' if self.ros_ready else self.ros_error[:45]}{hidden_status}"
        )
        self.real_value_label.config(text=f"IMU thực tế: Roll {real_r:+7.2f}° | Pitch {real_p:+7.2f}° | Yaw {real_y:+7.2f}°")
        self.gz_value_label.config(text=f"IMU Gazebo: Roll {gz_r:+7.2f}° | Pitch {gz_p:+7.2f}° | Yaw {gz_y:+7.2f}°")
        self.freq_value_label.config(
            text=(
                f"Tần số: IK12 đọc {hz_info['rx_hz']:.1f} Hz | "
                f"Publish GZB {hz_info['pub_hz']:.1f} Hz / Target {hz_info['target_hz']:.1f} Hz | "
                f"seq={hz_info['seq']} | count={hz_info['pub_count']} | "
                f"{'PUB ON' if hz_info['publish_enabled'] else 'PUB OFF'}"
            )
        )

        xs = list(self.t_data)
        self.line_real_roll.set_data(xs, list(self.real_roll))
        self.line_gz_roll.set_data(xs, list(self.gz_roll))
        self.line_real_pitch.set_data(xs, list(self.real_pitch))
        self.line_gz_pitch.set_data(xs, list(self.gz_pitch))
        self.line_real_yaw.set_data(xs, list(self.real_yaw))
        self.line_gz_yaw.set_data(xs, list(self.gz_yaw))

        if len(xs) >= 2:
            xmin = max(0.0, xs[-1] - GAZEBO_IMU_PLOT_WINDOW_SEC)
            xmax = xs[-1] + 0.5
            for ax in (self.ax_roll, self.ax_pitch, self.ax_yaw):
                ax.set_xlim(xmin, xmax)

        self.canvas.draw_idle()
        self.parent.after(GAZEBO_IMU_PLOT_UPDATE_MS, self.update_loop)

    def close(self):
        self.close_ros_only()



class CamPidApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera PID + IMU Plot - UDP Raspberry - FULLSCREEN")
        self.root.geometry("1920x1080")
        self.root.minsize(1350, 760)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.configure(bg="#f8fafc")

        # Mặc định mở giao diện toàn màn hình.
        self.fullscreen = True
        self.root.attributes("-fullscreen", True)

        self.pose = None
        self.udp_cam = None
        self.udp_imu = None

        self.offset_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = True
        self.send_camera_data = True
        self.cam_pid_enabled = False

        # Chế độ ưu tiên Balance: khi vào tab Balance, tạm dừng các tác vụ nặng để đồ thị IMU mượt hơn.
        self.balance_focus_mode = False
        self._balance_saved_send_camera_data = True
        self._balance_saved_gazebo_publish = None
        self._balance_last_tab_text = ""

        self.last_send_time = 0.0
        self.prev_time = time.time()
        self.fps = 0.0

        # Cache xử lý camera: dùng để giảm tải MediaPipe bằng cách xử lý cách frame.
        self.cam_frame_count = 0
        self.last_processed_output = None

        self.detected_val = 0
        self.full_body_ok = False

        self.offset_x = 0
        self.direction = "NO_PERSON"
        self.box_width = 0
        self.box_height = 0
        self.person_center_x = 0
        self.person_center_y = 0
        self.last_status_msg = "Ready"

        self.lpf_ready = False
        self.f_offset_x = 0.0
        self.f_box_width = 0.0
        self.f_box_height = 0.0
        self.f_center_x = 0.0
        self.f_center_y = 0.0

        # Snapshot offset camera để file REC CSV lưu được cả raw, LPF và offset thực tế gửi xuống bridge.
        self.offset_record_seq = 0
        self.offset_last_update_time = 0.0
        self.offset_record_snapshot = {
            "seq": 0,
            "unix_time": 0.0,
            "cam_pid_enabled": 0,
            "mediapipe_enabled": 0,
            "detected": 0,
            "full_body_ok": 0,
            "direction": "INIT",
            "offset_x_raw_px": 0.0,
            "offset_x_lpf_px": 0.0,
            "offset_x_send_px": 0.0,
            "offset_box_width_scale": 1.0,
            "offset_scale_ref_box_w_px": float(OFFSET_SCALE_REF_BOX_W),
            "offset_scale_enabled": int(OFFSET_SCALE_BY_BOX_W_ENABLE),
            "box_width_raw_px": 0.0,
            "box_width_lpf_px": 0.0,
            "box_height_raw_px": 0.0,
            "box_height_lpf_px": 0.0,
            "center_x_raw_px": 0.0,
            "center_x_lpf_px": 0.0,
            "center_y_raw_px": 0.0,
            "center_y_lpf_px": 0.0,
        }

        self.esp_box_status = "BOX H: waiting..."
        self.last_udp_msg = "None"

        # Dữ liệu đồ thị IMU.
        self.t0 = time.time()
        self.t_data = deque(maxlen=PLOT_MAX_POINTS)
        self.roll_data = deque(maxlen=PLOT_MAX_POINTS)
        self.pitch_data = deque(maxlen=PLOT_MAX_POINTS)
        self.yaw_data = deque(maxlen=PLOT_MAX_POINTS)

        # Ghi dữ liệu IMU thực tế nhận từ Raspberry qua UDP 7007.
        self.rec_enabled = False
        self.rec_folder = REC_DEFAULT_DIR
        self.rec_file_path = None
        self.rec_file = None
        self.rec_writer = None
        self.rec_start_time = 0.0
        self.rec_samples = 0
        self.rec_last_t_last = 0.0
        self.rec_last_seq = 0
        self.rec_status_labels = []

        # Tab Balance và PID SBAL.
        self.sbal_kp_var = tk.StringVar(value="0.1")
        self.sbal_ki_var = tk.StringVar(value="1.5")
        self.sbal_kd_var = tk.StringVar(value="0.0")
        self.sbal_maxx_var = tk.StringVar(value="80")
        self.sbal_deadband_var = tk.StringVar(value="0.1")
        self.sbal_hold_var = tk.StringVar(value="0.25")
        self.sbal_imax_var = tk.StringVar(value="40")
        self.sbal_roll_sp_var = tk.StringVar(value="0")
        self.sbal_pitch_sp_var = tk.StringVar(value="0")
        self.sbal_sp_limit_deg = 15.0
        self.sbal_joy_size = 210
        self.sbal_joy_center = self.sbal_joy_size / 2
        self.sbal_joy_radius = 82
        self.sbal_joy_knob_radius = 13
        self.sbal_last_sp_send_time = 0.0
        self.sbal_sp_send_interval = 0.10
        self.sbal_plot_window_sec = 20.0
        self.sbal_plot_max_points = 600
        self.sbal_plot_t0 = time.time()
        self.sbal_plot_t = deque(maxlen=self.sbal_plot_max_points)
        self.sbal_plot_roll = deque(maxlen=self.sbal_plot_max_points)
        self.sbal_plot_pitch = deque(maxlen=self.sbal_plot_max_points)
        self.sbal_plot_last_seq = 0
        self.sbal_plot_hz = 0.0
        self.sbal_plot_count_n = 0
        self.sbal_plot_count_t0 = time.time()
        # Bộ lọc hiển thị Balance chỉ làm mượt đường vẽ, không ảnh hưởng dữ liệu ghi CSV.
        self.sbal_plot_filter_ready = False
        self.sbal_plot_roll_f = 0.0
        self.sbal_plot_pitch_f = 0.0
        self.sbal_plot_spike_count = 0
        self.sbal_plot_spike_count_window = 0

        # Điều chỉnh trục X/chiều cao gait thông qua ESP bằng các lệnh X+, X- và X_RESET.
        # Cần khởi tạo trước build_ui() vì label X sử dụng self.height_x.
        self.height_x = HEIGHT_DEFAULT_X
        self.height_step_x = HEIGHT_STEP_X
        self.height_min_x = HEIGHT_MIN_X
        self.height_max_x = HEIGHT_MAX_X

        # Tab tinh chỉnh gait.
        self.gait_vars = {}
        self.gait_status_label = None
        self.init_gait_config_vars()

        self.setting_vars = {}

        self.pose_vars = {}
        self.pose_status_label = None
        for pose_name, legs in POSE_DEFAULTS.items():
            self.pose_vars[pose_name] = {}
            for leg in POSE_LEGS:
                self.pose_vars[pose_name][leg] = {}
                for i, axis in enumerate(POSE_AXES):
                    self.pose_vars[pose_name][leg][axis] = tk.StringVar(value=f"{legs[leg][i]:.1f}")
        self.load_saved_config()

        self.setup_style()
        self.build_ui()

        self.init_mediapipe()
        self.init_udp_camera()
        self.init_udp_imu()

        self.update_frame()
        self.update_imu_plot()

    # ================== GIAO DIỆN ==================
    def setup_style(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure(
            "TNotebook",
            background="#f8fafc",
            borderwidth=0,
            tabmargins=(4, 4, 4, 0)
        )

        # Tab đang chọn được hiển thị lớn hơn, sáng hơn và nổi bật hơn các tab còn lại.
        style.configure(
            "TNotebook.Tab",
            font=("Segoe UI", 14, "bold"),
            padding=(16, 9),
            background="#d1d5db",
            foreground="#475569",
            borderwidth=1
        )

        style.map(
            "TNotebook.Tab",
            background=[
                ("selected", "#ffffff"),
                ("!selected", "#d1d5db")
            ],
            foreground=[
                ("selected", "#0f172a"),
                ("!selected", "#475569")
            ],
            padding=[
                ("selected", (26, 15)),
                ("!selected", (16, 9))
            ],
            expand=[
                ("selected", (0, 0, 0, 4)),
                ("!selected", (0, 0, 0, 0))
            ]
        )

        style.configure("TFrame", background="#f8fafc")
        style.configure("Panel.TFrame", background="#ffffff")

        style.configure(
            "Title.TLabel",
            background="#f8fafc",
            foreground="#0f172a",
            font=("Segoe UI", 28, "bold")
        )

        style.configure(
            "School.TLabel",
            background="#f8fafc",
            foreground="#0369a1",
            font=("Segoe UI", 20, "bold")
        )

        style.configure(
            "Project.TLabel",
            background="#f8fafc",
            foreground="#92400e",
            font=("Segoe UI", 19, "bold")
        )

        style.configure(
            "Author.TLabel",
            background="#f8fafc",
            foreground="#334155",
            font=("Segoe UI", 16, "bold")
        )

        style.configure(
            "SubTitle.TLabel",
            background="#ffffff",
            foreground="#0f172a",
            font=("Segoe UI", 19, "bold")
        )

        style.configure(
            "Info.TLabel",
            background="#ffffff",
            foreground="#1e293b",
            font=("Consolas", 15)
        )

        style.configure(
            "BigImu.TLabel",
            background="#f1f5f9",
            foreground="#166534",
            font=("Consolas", 20, "bold")
        )

        style.configure(
            "Small.TLabel",
            background="#ffffff",
            foreground="#475569",
            font=("Segoe UI", 13)
        )

        style.configure(
            "Status.TLabel",
            background="#ffffff",
            foreground="#166534",
            font=("Consolas", 14)
        )

        style.configure(
            "DateTime.TLabel",
            background="#f8fafc",
            foreground="#0f172a",
            font=("Consolas", 15, "bold")
        )

        style.configure(
            "Debug.TLabel",
            background="#f1f5f9",
            foreground="#92400e",
            font=("Consolas", 13)
        )

        style.configure("TButton", font=("Segoe UI", 15, "bold"), padding=12)
        style.configure("Compact.TButton", font=("Segoe UI", 12, "bold"), padding=(8, 7))
        style.configure("CompactGreen.TButton", font=("Segoe UI", 12, "bold"), padding=(8, 7), background="#15803d", foreground="#ffffff")
        style.configure("CompactRed.TButton", font=("Segoe UI", 12, "bold"), padding=(8, 7), background="#b91c1c", foreground="#ffffff")
        style.configure("CompactBlue.TButton", font=("Segoe UI", 12, "bold"), padding=(8, 7), background="#2563eb", foreground="#ffffff")
        style.configure("CompactGray.TButton", font=("Segoe UI", 12, "bold"), padding=(8, 7), background="#475569", foreground="#ffffff")
        style.configure("Green.TButton", background="#15803d", foreground="#ffffff")
        style.configure("Red.TButton", background="#b91c1c", foreground="#ffffff")
        style.configure("Blue.TButton", background="#2563eb", foreground="#ffffff")
        style.configure("Gray.TButton", background="#475569", foreground="#ffffff")
        style.map("Green.TButton", background=[("active", "#16a34a")])
        style.map("Red.TButton", background=[("active", "#dc2626")])
        style.map("Blue.TButton", background=[("active", "#3b82f6")])
        style.map("Gray.TButton", background=[("active", "#64748b")])

    def find_logo_path(self, candidates):
        search_dirs = [
            os.path.dirname(os.path.abspath(__file__)),
            os.path.expanduser("~/Desktop"),
            os.path.expanduser("~/Downloads"),
            os.path.expanduser("~/Pictures"),
            os.path.expanduser("~/logo"),
        ]

        for folder in search_dirs:
            for name in candidates:
                path = os.path.join(folder, name)
                if os.path.exists(path):
                    return path
        return None

    def load_logo_image(self, candidates, target_height):
        path = self.find_logo_path(candidates)
        if not path:
            return None

        try:
            img = Image.open(path).convert("RGBA")
            scale = target_height / max(img.height, 1)
            new_w = max(1, int(img.width * scale))
            img = img.resize((new_w, target_height), Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(img)
        except Exception as e:
            print(f"[LOGO] Không load được {path}: {e}")
            return None

    def build_header_with_logos(self, parent):
        header_info = ttk.Frame(parent, style="TFrame")
        header_info.pack(fill=tk.X, pady=(0, 8))

        group_wrap = ttk.Frame(header_info, style="TFrame")
        group_wrap.pack(anchor="center")
        group_wrap.columnconfigure(1, weight=0)

        left_logo_h = int(UTE_LOGO_TARGET_HEIGHT * FEEE_LOGO_HEIGHT_RATIO)
        right_logo_h = UTE_LOGO_TARGET_HEIGHT

        self.left_logo_imgtk = self.load_logo_image(LEFT_LOGO_FILES, left_logo_h)
        self.right_logo_imgtk = self.load_logo_image(RIGHT_LOGO_FILES, right_logo_h)

        left_wrap = ttk.Frame(group_wrap, style="TFrame")
        left_wrap.grid(row=0, column=0, padx=(0, LOGO_TEXT_GAP_X), pady=(4, 0))

        center_wrap = ttk.Frame(group_wrap, style="TFrame")
        center_wrap.grid(row=0, column=1, padx=(0, 0), pady=(0, 0))

        right_wrap = ttk.Frame(group_wrap, style="TFrame")
        right_wrap.grid(row=0, column=2, padx=(LOGO_TEXT_GAP_X, 0), pady=(4, 0))

        if self.left_logo_imgtk is not None:
            ttk.Label(left_wrap, image=self.left_logo_imgtk, background="#f8fafc").pack()
        else:
            ttk.Label(left_wrap, text="", style="Author.TLabel").pack()

        if self.right_logo_imgtk is not None:
            ttk.Label(right_wrap, image=self.right_logo_imgtk, background="#f8fafc").pack()
        else:
            ttk.Label(right_wrap, text="", style="Author.TLabel").pack()

        ttk.Label(
            center_wrap,
            text=UNIVERSITY_NAME,
            style="School.TLabel",
            anchor="center",
            justify="center"
        ).pack(anchor="center")

        ttk.Label(
            center_wrap,
            text=PROJECT_TITLE,
            style="Project.TLabel",
            anchor="center",
            justify="center"
        ).pack(anchor="center", pady=(2, 0))

        ttk.Label(
            center_wrap,
            text=SUPERVISOR_NAME,
            style="Author.TLabel",
            anchor="center",
            justify="center"
        ).pack(anchor="center", pady=(2, 0))

        ttk.Label(
            center_wrap,
            text=STUDENT_NAMES,
            style="Author.TLabel",
            anchor="center",
            justify="center"
        ).pack(anchor="center", pady=(2, 0))

    def build_ui(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Hiển thị ngày giờ ở góc phải phía trên.
        self.datetime_label = ttk.Label(self.root, text="", style="DateTime.TLabel")
        self.datetime_label.place(relx=1.0, x=-18, y=12, anchor="ne")
        self.update_datetime_label()

        monitor_tab = ttk.Frame(self.notebook, style="TFrame")
        settings_tab = ttk.Frame(self.notebook, style="TFrame")
        balance_tab = ttk.Frame(self.notebook, style="TFrame")
        gait_tab = ttk.Frame(self.notebook, style="TFrame")
        pose_tab = ttk.Frame(self.notebook, style="TFrame")
        gazebo_tab = ttk.Frame(self.notebook, style="TFrame")

        self.notebook.add(monitor_tab, text="Giám sát")
        self.notebook.add(settings_tab, text="Cài đặt thông số")
        self.notebook.add(balance_tab, text="Balance")
        self.notebook.add(gait_tab, text="Gait")
        self.notebook.add(pose_tab, text="Tọa độ")
        self.notebook.add(gazebo_tab, text="Gazebo")
        self.notebook.bind("<<NotebookTabChanged>>", self.on_notebook_tab_changed)

        main = ttk.Frame(monitor_tab, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # Thông tin đề tài và logo hai bên.
        self.build_header_with_logos(main)

        content = ttk.Frame(main)
        content.pack(fill=tk.BOTH, expand=True)

        # Khung camera bên trái.
        left = ttk.Frame(content, style="Panel.TFrame", padding=8)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self.video_label = ttk.Label(left, background="#000000", anchor="center")
        self.video_label.pack(fill=tk.BOTH, expand=True)

        # Bảng điều khiển bên phải.
        right = ttk.Frame(content, style="Panel.TFrame", padding=16, width=1080)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)
        right.pack_propagate(False)

        ttk.Label(right, text="UDP / PID", style="SubTitle.TLabel").pack(anchor="w")

        top_info = ttk.Frame(right, style="Panel.TFrame")
        top_info.pack(fill=tk.X, pady=(2, 5))

        self.info_pi_ip = ttk.Label(
            top_info,
            text=f"Pi IP: {RASPBERRY_IP}:{RASPBERRY_OFFSET_PORT}",
            style="Info.TLabel"
        )
        self.info_cam_port = ttk.Label(
            top_info,
            text=f"Cam UDP: {CAMERA_UDP_PORT}",
            style="Info.TLabel"
        )
        self.info_imu_port = ttk.Label(
            top_info,
            text=f"IMU UDP: {IMU_LISTEN_PORT}",
            style="Info.TLabel"
        )

        self.info_pi_ip.grid(row=0, column=0, sticky="w", padx=(0, 20))
        self.info_cam_port.grid(row=0, column=1, sticky="w", padx=(0, 20))
        self.info_imu_port.grid(row=0, column=2, sticky="w")

        self.info_udp = ttk.Label(right, text="UDP CAM: waiting...", style="Info.TLabel")
        self.info_imu = ttk.Label(right, text="UDP IMU: waiting...", style="Info.TLabel")
        self.info_gui_fps = ttk.Label(right, text="GUI FPS: 0.0", style="Info.TLabel")
        self.info_udp.pack(anchor="w", pady=1)
        self.info_imu.pack(anchor="w", pady=1)
        self.info_gui_fps.pack(anchor="w", pady=1)

        pid_box = ttk.Frame(right, style="Panel.TFrame")
        pid_box.pack(fill=tk.X, pady=(8, 8))

        self.btn_pid_on = ttk.Button(
            pid_box,
            text="BẬT BÁM NGƯỜI",
            style="Green.TButton",
            command=self.start_cam_pid
        )
        self.btn_pid_on.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))

        self.btn_pid_off = ttk.Button(
            pid_box,
            text="TẮT BÁM NGƯỜI",
            style="Red.TButton",
            command=self.stop_cam_pid
        )
        self.btn_pid_off.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.pid_state_label = ttk.Label(right, text="PID cam: OFF", style="Info.TLabel")
        self.pid_state_label.pack(anchor="w", pady=(0, 5))

        # Cụm REC IMU hiển thị ở cả tab Giám sát và tab Balance.
        self.build_rec_controls(right)

        # Các nút điều khiển thủ công.
        manual_box = ttk.LabelFrame(right, text="Điều khiển robot từ GUI", padding=8)
        manual_box.pack(fill=tk.X, pady=(4, 8))

        # Hàng nút chọn tư thế.
        pose_row = ttk.Frame(manual_box, style="Panel.TFrame")
        pose_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(pose_row, text="ĐỨNG", style="Blue.TButton", command=lambda: self.send_manual_cmd("STAND")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(pose_row, text="NGỒI", style="Gray.TButton", command=lambda: self.send_manual_cmd("SIT")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(pose_row, text="NẰM", style="Gray.TButton", command=lambda: self.send_manual_cmd("LIE")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(pose_row, text="BẮT TAY", style="Blue.TButton", command=lambda: self.send_manual_cmd("SHAKE")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        # Hàng nút điều khiển di chuyển chính.
        move_grid = ttk.Frame(manual_box, style="Panel.TFrame")
        move_grid.pack(fill=tk.X)
        for c in range(5):
            move_grid.columnconfigure(c, weight=1)

        ttk.Button(move_grid, text="TIẾN", style="Green.TButton", command=lambda: self.send_manual_motion("FWD")).grid(row=0, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="LÙI", style="Green.TButton", command=lambda: self.send_manual_motion("BWD")).grid(row=2, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="TRÁI", style="Blue.TButton", command=lambda: self.send_manual_motion("LEFT")).grid(row=1, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="PHẢI", style="Blue.TButton", command=lambda: self.send_manual_motion("RIGHT")).grid(row=1, column=2, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="STOP", style="Red.TButton", command=self.send_manual_stop).grid(row=1, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="SL", style="Blue.TButton", command=lambda: self.send_manual_motion("SL")).grid(row=1, column=3, sticky="ew", padx=3, pady=3)
        ttk.Button(move_grid, text="SR", style="Blue.TButton", command=lambda: self.send_manual_motion("SR")).grid(row=1, column=4, sticky="ew", padx=3, pady=3)

        # Hàng chỉnh chiều cao trục X; Pi chuyển lệnh về ESP mà không gửi STOP trước.
        height_row = ttk.Frame(manual_box, style="Panel.TFrame")
        height_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(height_row, text="Chiều cao X:", style="Info.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(height_row, text="X+ / CAO+", style="CompactGreen.TButton", command=self.send_height_up).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(height_row, text="X- / CAO-", style="CompactBlue.TButton", command=self.send_height_down).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(height_row, text="X = -190", style="CompactGray.TButton", command=self.send_height_reset).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        self.height_label = ttk.Label(height_row, text=f"X={self.height_x:.0f} mm", style="Info.TLabel")
        self.height_label.pack(side=tk.LEFT, padx=(10, 0))


        ttk.Separator(right).pack(fill=tk.X, pady=6)

        # Hiển thị giá trị IMU lớn.
        ttk.Label(right, text="IMU Roll / Pitch / Yaw", style="SubTitle.TLabel").pack(anchor="w")

        self.imu_value_label = ttk.Label(
            right,
            text="Roll: 0.00°    Pitch: 0.00°    Yaw: 0.00°",
            style="BigImu.TLabel",
            padding=8
        )
        self.imu_value_label.pack(fill=tk.X, pady=(2, 4))

        # Tab Giám sát không hiển thị đồ thị IMU.
        # IMU vẫn được nhận để hiển thị số và ghi CSV; có thể chọn file CSV để xem lại đồ thị.

        self.debug_label = ttk.Label(
            right,
            text="Last UDP: None",
            style="Debug.TLabel",
            wraplength=900,
            justify=tk.LEFT,
            padding=5
        )
        self.debug_label.pack(fill=tk.X, pady=(2, 3))

        self.box_debug_label = ttk.Label(
            right,
            text="BOX H: waiting...",
            style="Debug.TLabel",
            wraplength=900,
            justify=tk.LEFT,
            padding=5
        )
        self.box_debug_label.pack(fill=tk.X, pady=(0, 3))

        self.status_label = ttk.Label(
            right,
            text="Ready",
            style="Status.TLabel",
            wraplength=900
        )
        self.status_label.pack(anchor="w", fill=tk.X, pady=(2, 0))

        self.build_settings_tab(settings_tab)
        self.build_balance_tab(balance_tab)
        self.build_gait_tab(gait_tab)
        self.build_pose_tab(pose_tab)

        # Chạy luồng IK12 → JointTrajectory ẩn phía sau để Gazebo vẫn nhận lệnh.
        # Tab Gazebo chỉ hiển thị đồ thị so sánh IMU Gazebo và IMU thực tế.
        hidden_joint_frame = ttk.Frame(gazebo_tab, style="TFrame")
        self.joint_mapper_panel = JointMapperPanel(hidden_joint_frame)
        self.gazebo_imu_panel = GazeboImuComparePanel(gazebo_tab, self, self.joint_mapper_panel)
        self.apply_runtime_performance_settings()
        self.root.after(100, self.on_notebook_tab_changed)

        self.root.bind("<b>", lambda e: self.start_cam_pid())
        self.root.bind("<B>", lambda e: self.start_cam_pid())
        self.root.bind("<n>", lambda e: self.stop_cam_pid())
        self.root.bind("<N>", lambda e: self.stop_cam_pid())
        self.root.bind("<q>", lambda e: self.on_close())
        self.root.bind("<Q>", lambda e: self.on_close())
        self.root.bind("<Escape>", lambda e: self.end_fullscreen())
        self.root.bind("<F11>", lambda e: self.toggle_fullscreen())
        self.root.bind("<r>", lambda e: self.start_imu_record())
        self.root.bind("<R>", lambda e: self.start_imu_record())

        # Phím tắt điều khiển robot từ GUI.
        self.root.bind("<w>", lambda e: self.send_manual_motion("FWD"))
        self.root.bind("<W>", lambda e: self.send_manual_motion("FWD"))
        self.root.bind("<s>", lambda e: self.send_manual_motion("BWD"))
        self.root.bind("<S>", lambda e: self.send_manual_motion("BWD"))
        self.root.bind("<a>", lambda e: self.send_manual_motion("LEFT"))
        self.root.bind("<A>", lambda e: self.send_manual_motion("LEFT"))
        self.root.bind("<d>", lambda e: self.send_manual_motion("RIGHT"))
        self.root.bind("<D>", lambda e: self.send_manual_motion("RIGHT"))
        self.root.bind("<z>", lambda e: self.send_manual_motion("SL"))
        self.root.bind("<Z>", lambda e: self.send_manual_motion("SL"))
        self.root.bind("<x>", lambda e: self.send_manual_motion("SR"))
        self.root.bind("<X>", lambda e: self.send_manual_motion("SR"))
        self.root.bind("<Prior>", lambda e: self.send_height_up())      # PageUp
        self.root.bind("<Next>", lambda e: self.send_height_down())     # PageDown
        self.root.bind("<space>", lambda e: self.send_manual_stop())
        self.root.bind("<KeyPress-1>", lambda e: self.send_manual_cmd("STAND"))
        self.root.bind("<KeyPress-2>", lambda e: self.send_manual_cmd("SIT"))
        self.root.bind("<KeyPress-3>", lambda e: self.send_manual_cmd("LIE"))
        self.root.bind("<KeyPress-4>", lambda e: self.send_manual_cmd("SHAKE"))


    # ================== CHẾ ĐỘ ƯU TIÊN BALANCE ==================
    def get_current_tab_text(self):
        try:
            tab_id = self.notebook.select()
            if not tab_id:
                return ""
            return str(self.notebook.tab(tab_id, "text"))
        except Exception:
            return ""

    def is_balance_tab_active(self):
        return self.get_current_tab_text().strip().lower() == "balance"

    def on_notebook_tab_changed(self, event=None):
        if not bool(BALANCE_FOCUS_MODE_ENABLE):
            return
        tab_text = self.get_current_tab_text()
        self._balance_last_tab_text = tab_text
        if tab_text.strip().lower() == "balance":
            self.enter_balance_focus_mode()
        else:
            self.exit_balance_focus_mode()

    def enter_balance_focus_mode(self):
        if self.balance_focus_mode:
            return
        self.balance_focus_mode = True

        # 1) Dừng gửi gói camera PID và tạm dừng decode camera.
        self._balance_saved_send_camera_data = self.send_camera_data
        self.send_camera_data = False
        if bool(BALANCE_PAUSE_CAMERA):
            try:
                if self.udp_cam is not None:
                    self.udp_cam.set_paused(True)
            except Exception as e:
                print("[BALANCE FOCUS] pause camera error:", e)

        # 2) Tạm dừng publish sang Gazebo.
        if bool(BALANCE_PAUSE_GAZEBO_PUBLISH):
            try:
                if hasattr(self, "joint_mapper_panel") and self.joint_mapper_panel is not None:
                    self._balance_saved_gazebo_publish = bool(self.joint_mapper_panel.publish_enabled.get())
                    self.joint_mapper_panel.publish_enabled.set(False)
            except Exception as e:
                print("[BALANCE FOCUS] pause gazebo publish error:", e)

        # 3) Tạm dừng cập nhật bảng mapping ẩn.
        if bool(BALANCE_PAUSE_JOINT_TABLE_UI):
            try:
                if hasattr(self, "joint_mapper_panel") and self.joint_mapper_panel is not None:
                    self.joint_mapper_panel.set_ui_paused(True)
            except Exception as e:
                print("[BALANCE FOCUS] pause joint table error:", e)

        try:
            self.last_status_msg = "Balance focus ON: tạm dừng camera/Gazebo để giảm lag plot IMU"
            self.update_status()
        except Exception:
            pass
        try:
            if hasattr(self, "performance_status_label"):
                self.performance_status_label.config(
                    text="Balance focus ON: camera decode OFF | Gazebo publish OFF | Gazebo plot OFF | IMU/REC vẫn chạy"
                )
        except Exception:
            pass

    def exit_balance_focus_mode(self):
        if not self.balance_focus_mode:
            return
        self.balance_focus_mode = False

        # Khôi phục decode camera và trạng thái gửi camera như trước.
        self.send_camera_data = self._balance_saved_send_camera_data
        try:
            if self.udp_cam is not None:
                self.udp_cam.set_paused(False)
        except Exception as e:
            print("[BALANCE FOCUS] resume camera error:", e)

        # Khôi phục trạng thái publish Gazebo như trước khi vào tab Balance.
        try:
            if hasattr(self, "joint_mapper_panel") and self.joint_mapper_panel is not None:
                if self._balance_saved_gazebo_publish is not None:
                    self.joint_mapper_panel.publish_enabled.set(bool(self._balance_saved_gazebo_publish))
                self.joint_mapper_panel.set_ui_paused(False)
        except Exception as e:
            print("[BALANCE FOCUS] resume gazebo error:", e)

        try:
            self.apply_runtime_performance_settings()
            self.last_status_msg = "Balance focus OFF: đã khôi phục camera/Gazebo"
            self.update_status()
        except Exception:
            pass


    # ================== TAB BALANCE / SBAL ==================
    def build_rec_controls(self, parent):
        """Tạo cụm nút REC IMU dùng chung cho tab Giám sát và tab Balance."""
        rec_box = ttk.LabelFrame(parent, text="", padding=10)
        rec_box.pack(fill=tk.X, pady=(4, 10))

        rec_btn_row = ttk.Frame(rec_box, style="Panel.TFrame")
        rec_btn_row.pack(fill=tk.X)

        ttk.Button(
            rec_btn_row,
            text="CHỌN FOLDER",
            style="CompactGray.TButton",
            command=self.choose_rec_folder
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))

        ttk.Button(
            rec_btn_row,
            text="BẮT ĐẦU REC IMU+OFFSET",
            style="CompactGreen.TButton",
            command=self.start_imu_record
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        ttk.Button(
            rec_btn_row,
            text="DỪNG REC",
            style="CompactRed.TButton",
            command=self.stop_imu_record
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        ttk.Button(
            rec_btn_row,
            text="CHỌN FILE CSV ĐỂ PLOT",
            style="CompactBlue.TButton",
            command=self.choose_csv_and_plot
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))

        label = ttk.Label(
            rec_box,
            text=f"REC: OFF | Folder: {self.rec_folder}",
            style="Small.TLabel",
            wraplength=900
        )
        label.pack(anchor="w", fill=tk.X, pady=(6, 0))
        self.rec_status_labels.append(label)
        return rec_box

    def set_rec_status_text(self, text):
        for label in getattr(self, "rec_status_labels", []):
            try:
                label.config(text=text)
            except Exception:
                pass

    def build_balance_tab(self, parent):
        outer = ttk.Frame(parent, style="TFrame", padding=16)
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(outer, text="BALANCE SBAL - PID + SETPOINT + LIVE PLOT", style="Title.TLabel").pack(anchor="w", pady=(0, 10))

        body = ttk.Frame(outer, style="TFrame")
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, style="Panel.TFrame", padding=14)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        cmd_box = ttk.LabelFrame(left, text="Lệnh balance", padding=10)
        cmd_box.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(cmd_box, text="STAND", style="Blue.TButton", command=lambda: self.send_balance_command_sequence(["STAND"], "Balance gửi STAND")).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(cmd_box, text="SBAL ON", style="Green.TButton", command=lambda: self.send_balance_command_sequence(["STAND", "SBAL_ON"], "Balance STAND + SBAL_ON")).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ttk.Button(cmd_box, text="SBAL OFF", style="Red.TButton", command=lambda: self.send_balance_command_sequence(["SBAL_OFF"], "Balance SBAL_OFF")).grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        ttk.Button(cmd_box, text="SHOWSBAL", style="Gray.TButton", command=lambda: self.send_balance_command_sequence(["SHOWSBAL"], "Balance SHOWSBAL")).grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        for c in range(4):
            cmd_box.columnconfigure(c, weight=1)

        # Cụm REC IMU hiển thị ở cả tab Giám sát và tab Balance.
        self.build_rec_controls(left)

        pid_box = ttk.LabelFrame(left, text="Chỉnh PID SBAL", padding=10)
        pid_box.pack(fill=tk.X, pady=(0, 10))
        labels = ["Kp", "Ki", "Kd", "MaxX", "Deadband", "Hold", "IMax"]
        vars_ = [self.sbal_kp_var, self.sbal_ki_var, self.sbal_kd_var, self.sbal_maxx_var, self.sbal_deadband_var, self.sbal_hold_var, self.sbal_imax_var]
        for i, (lb, var) in enumerate(zip(labels, vars_)):
            ttk.Label(pid_box, text=lb, style="Info.TLabel").grid(row=0, column=i, padx=3, pady=(0, 4))
            ttk.Entry(pid_box, textvariable=var, width=8, font=("Consolas", 13)).grid(row=1, column=i, padx=3, pady=4)

        btnrow = ttk.Frame(pid_box, style="Panel.TFrame")
        btnrow.grid(row=2, column=0, columnspan=7, sticky="ew", pady=(8, 2))
        ttk.Button(btnrow, text="GỬI SET_SBAL", style="Green.TButton", command=self.send_sbal_pid).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(btnrow, text="GỬI IMax", style="Blue.TButton", command=self.send_sbal_imax).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(btnrow, text="Ki = 0", style="Gray.TButton", command=self.zero_sbal_ki).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        # Chỉ sử dụng một bộ thông số PID mặc định, không dùng các preset nhẹ/vừa/mạnh.
        default_note = ttk.Label(
            pid_box,
            text="Mặc định: Kp=0.1 | Ki=1.5 | Kd=0.0 | MaxX=80 | Deadband=0.1 | Hold=0.25 | IMax=40",
            style="Small.TLabel"
        )
        default_note.grid(row=3, column=0, columnspan=7, sticky="w", pady=(8, 0))

        sp_box = ttk.LabelFrame(left, text="Setpoint Roll/Pitch - Joystick ±15°", padding=10)
        sp_box.pack(fill=tk.X, pady=(0, 10))

        self.sbal_joy_canvas = tk.Canvas(sp_box, width=self.sbal_joy_size, height=self.sbal_joy_size, bg="white", highlightthickness=1, highlightbackground="#94a3b8")
        self.sbal_joy_canvas.grid(row=0, column=0, rowspan=5, padx=(0, 12), pady=4)
        self.draw_sbal_joystick_base()
        self.sbal_joy_canvas.bind("<Button-1>", self.on_sbal_joystick_event)
        self.sbal_joy_canvas.bind("<B1-Motion>", self.on_sbal_joystick_event)
        self.sbal_joy_canvas.bind("<ButtonRelease-1>", lambda e: self.send_sbal_setpoint(log_cmd=True))

        ttk.Label(sp_box, text="Roll SP (deg)", style="Info.TLabel").grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(sp_box, text="Pitch SP (deg)", style="Info.TLabel").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(sp_box, width=12, textvariable=self.sbal_roll_sp_var, font=("Consolas", 14)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Entry(sp_box, width=12, textvariable=self.sbal_pitch_sp_var, font=("Consolas", 14)).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(sp_box, text="GỬI SET_SBAL_SP", style="Green.TButton", command=self.send_sbal_setpoint).grid(row=1, column=3, padx=5, pady=5, sticky="ew")
        ttk.Button(sp_box, text="SP = 0", style="Gray.TButton", command=self.zero_sbal_setpoint).grid(row=2, column=3, padx=5, pady=5, sticky="ew")

        self.sbal_sp_value_label = ttk.Label(sp_box, text="Roll SP = 0.00° | Pitch SP = 0.00°", style="BigImu.TLabel", padding=6)
        self.sbal_sp_value_label.grid(row=3, column=1, columnspan=3, sticky="ew", padx=5, pady=(8, 0))
        self.update_sbal_joystick_knob_from_sp()

        right = ttk.Frame(body, style="Panel.TFrame", padding=14)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        ttk.Label(right, text="LIVE Roll/Pitch thực tế + setpoint", style="SubTitle.TLabel").pack(anchor="w")
        self.sbal_status_label = ttk.Label(right, text="SBAL tab ready", style="Info.TLabel")
        self.sbal_status_label.pack(anchor="w", pady=(0, 6))

        self.sbal_imu_hz_label = ttk.Label(
            right,
            text="Tần số IMU ESP: RX 0.0 Hz | Plot Balance 0.0 Hz | RAW không lọc | seq=0",
            style="BigImu.TLabel",
            padding=6
        )
        self.sbal_imu_hz_label.pack(anchor="w", fill=tk.X, pady=(0, 8))

        self.sbal_fig = Figure(figsize=(10.5, 6.2), dpi=100, facecolor="#ffffff")
        self.sbal_ax = self.sbal_fig.add_subplot(111)
        self.sbal_line_roll, = self.sbal_ax.plot([], [], label="Roll thực tế", color="tab:blue", linewidth=2.0)
        self.sbal_line_pitch, = self.sbal_ax.plot([], [], label="Pitch thực tế", color="tab:orange", linewidth=2.0)
        self.sbal_roll_sp_line = self.sbal_ax.axhline(0.0, color="tab:blue", linestyle="--", linewidth=1.4, label="Roll SP")
        self.sbal_pitch_sp_line = self.sbal_ax.axhline(0.0, color="tab:orange", linestyle="--", linewidth=1.4, label="Pitch SP")
        self.sbal_ax.set_title("LIVE Roll / Pitch SBAL", fontsize=15, fontweight="bold")
        self.sbal_ax.set_xlabel("Time (s)")
        self.sbal_ax.set_ylabel("Angle (deg)")
        self.sbal_ax.set_ylim(-25, 25)
        self.sbal_ax.grid(True, color="#cbd5e1", linewidth=0.8)
        self.sbal_ax.legend(loc="upper right")
        self.sbal_canvas = FigureCanvasTkAgg(self.sbal_fig, master=right)
        self.sbal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.update_sbal_live_plot()

    def send_balance_command_sequence(self, commands, msg="Balance command sent"):
        self.cam_pid_enabled = False

        seq = ["CAMGAIT_OFF"] + [str(c).strip().upper() for c in commands]
        self.send_command_sequence_to_pi(seq, msg)

    def clamp_float(self, value, lo, hi):
        return max(lo, min(hi, value))

    def draw_sbal_joystick_base(self):
        c = self.sbal_joy_center
        r = self.sbal_joy_radius
        self.sbal_joy_canvas.create_oval(c-r, c-r, c+r, c+r, outline="#334155", width=2)
        self.sbal_joy_canvas.create_line(c-r, c, c+r, c, fill="#cbd5e1", width=1)
        self.sbal_joy_canvas.create_line(c, c-r, c, c+r, fill="#cbd5e1", width=1)
        self.sbal_joy_canvas.create_text(c, 16, text="-Pitch", fill="#64748b", font=("Arial", 9))
        self.sbal_joy_canvas.create_text(c, self.sbal_joy_size-16, text="+Pitch", fill="#64748b", font=("Arial", 9))
        self.sbal_joy_canvas.create_text(32, c, text="-Roll", fill="#64748b", font=("Arial", 9))
        self.sbal_joy_canvas.create_text(self.sbal_joy_size-32, c, text="+Roll", fill="#64748b", font=("Arial", 9))
        rr = self.sbal_joy_knob_radius
        self.sbal_joy_knob = self.sbal_joy_canvas.create_oval(c-rr, c-rr, c+rr, c+rr, fill="#ef4444", outline="#111827", width=1)

    def sbal_joystick_pos_to_sp(self, x, y):
        dx = x - self.sbal_joy_center
        dy = y - self.sbal_joy_center
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self.sbal_joy_radius:
            k = self.sbal_joy_radius / max(dist, 1e-6)
            dx *= k
            dy *= k
        roll_sp = (dx / self.sbal_joy_radius) * self.sbal_sp_limit_deg
        pitch_sp = (dy / self.sbal_joy_radius) * self.sbal_sp_limit_deg
        return self.clamp_float(roll_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg), self.clamp_float(pitch_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg)

    def sbal_sp_to_joystick_pos(self, roll_sp, pitch_sp):
        roll_sp = self.clamp_float(roll_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg)
        pitch_sp = self.clamp_float(pitch_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg)
        x = self.sbal_joy_center + (roll_sp / self.sbal_sp_limit_deg) * self.sbal_joy_radius
        y = self.sbal_joy_center + (pitch_sp / self.sbal_sp_limit_deg) * self.sbal_joy_radius
        return x, y

    def set_sbal_joystick_knob_xy(self, x, y):
        rr = self.sbal_joy_knob_radius
        self.sbal_joy_canvas.coords(self.sbal_joy_knob, x-rr, y-rr, x+rr, y+rr)

    def get_sbal_setpoints(self):
        try:
            roll_sp = float(self.sbal_roll_sp_var.get())
        except Exception:
            roll_sp = 0.0
        try:
            pitch_sp = float(self.sbal_pitch_sp_var.get())
        except Exception:
            pitch_sp = 0.0
        roll_sp = self.clamp_float(roll_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg)
        pitch_sp = self.clamp_float(pitch_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg)
        return roll_sp, pitch_sp

    def update_sbal_sp_value_label(self):
        if not hasattr(self, "sbal_sp_value_label"):
            return
        roll_sp, pitch_sp = self.get_sbal_setpoints()
        self.sbal_sp_value_label.config(text=f"Roll SP = {roll_sp:.2f}° | Pitch SP = {pitch_sp:.2f}°")

    def update_sbal_joystick_knob_from_sp(self):
        if not hasattr(self, "sbal_joy_canvas"):
            return
        roll_sp, pitch_sp = self.get_sbal_setpoints()
        x, y = self.sbal_sp_to_joystick_pos(roll_sp, pitch_sp)
        self.set_sbal_joystick_knob_xy(x, y)
        self.update_sbal_sp_value_label()

    def on_sbal_joystick_event(self, event):
        roll_sp, pitch_sp = self.sbal_joystick_pos_to_sp(event.x, event.y)
        self.sbal_roll_sp_var.set(f"{roll_sp:.2f}")
        self.sbal_pitch_sp_var.set(f"{pitch_sp:.2f}")
        x, y = self.sbal_sp_to_joystick_pos(roll_sp, pitch_sp)
        self.set_sbal_joystick_knob_xy(x, y)
        self.update_sbal_sp_value_label()
        now = time.time()
        if now - self.sbal_last_sp_send_time >= self.sbal_sp_send_interval:
            self.sbal_last_sp_send_time = now
            self.send_sbal_setpoint(log_cmd=False)

    def send_sbal_pid(self):
        try:
            kp = float(self.sbal_kp_var.get())
            ki = float(self.sbal_ki_var.get())
            kd = float(self.sbal_kd_var.get())
            maxx = float(self.sbal_maxx_var.get())
            deadband = float(self.sbal_deadband_var.get())
            hold = float(self.sbal_hold_var.get())
        except ValueError:
            messagebox.showerror("Lỗi nhập PID", "Kp Ki Kd MaxX Deadband Hold phải là số")
            return
        self.send_balance_command_sequence([f"SET_SBAL {kp:.4f} {ki:.4f} {kd:.4f} {maxx:.2f} {deadband:.2f} {hold:.2f}"], "Đã gửi SET_SBAL")

    def send_sbal_imax(self):
        try:
            imax = float(self.sbal_imax_var.get())
        except ValueError:
            messagebox.showerror("Lỗi nhập IMax", "IMax phải là số")
            return
        self.send_balance_command_sequence([f"SET_SBAL_IMAX {imax:.2f}"], "Đã gửi SET_SBAL_IMAX")

    def set_sbal_light(self):
        self.sbal_kp_var.set("0.5"); self.sbal_ki_var.set("0.0"); self.sbal_kd_var.set("0.25")
        self.sbal_maxx_var.set("60"); self.sbal_deadband_var.set("0.8"); self.sbal_hold_var.set("0.25"); self.sbal_imax_var.set("40")
        self.send_sbal_pid(); self.send_sbal_imax()

    def set_sbal_medium(self):
        self.sbal_kp_var.set("0.5"); self.sbal_ki_var.set("2.0"); self.sbal_kd_var.set("0.4")
        self.sbal_maxx_var.set("80"); self.sbal_deadband_var.set("0.5"); self.sbal_hold_var.set("0.25"); self.sbal_imax_var.set("40")
        self.send_sbal_pid(); self.send_sbal_imax()

    def set_sbal_strong(self):
        self.sbal_kp_var.set("0.8"); self.sbal_ki_var.set("2.5"); self.sbal_kd_var.set("0.5")
        self.sbal_maxx_var.set("80"); self.sbal_deadband_var.set("0.5"); self.sbal_hold_var.set("0.25"); self.sbal_imax_var.set("40")
        self.send_sbal_pid(); self.send_sbal_imax()

    def zero_sbal_ki(self):
        self.sbal_ki_var.set("0.0")
        self.send_sbal_pid()

    def send_sbal_setpoint(self, log_cmd=True):
        roll_sp, pitch_sp = self.get_sbal_setpoints()
        self.sbal_roll_sp_var.set(f"{roll_sp:.2f}")
        self.sbal_pitch_sp_var.set(f"{pitch_sp:.2f}")
        self.update_sbal_joystick_knob_from_sp()

        # Quy ước joystick trên GUI:
        #   - Ngang trái/phải  -> Roll SP.
        #   - Trên/dưới       -> Pitch SP.
        # ESP đang hiểu SET_SBAL_SP theo thứ tự Pitch, Roll,
        # nên cần gửi pitch_sp trước để thao tác lên/xuống điều khiển đúng Pitch.
        if SBAL_SP_SEND_ORDER == "PITCH_ROLL":
            cmd = f"SET_SBAL_SP {pitch_sp:.3f} {roll_sp:.3f}"
        else:
            cmd = f"SET_SBAL_SP {roll_sp:.3f} {pitch_sp:.3f}"

        self.send_balance_command_sequence([cmd], "Đã gửi SET_SBAL_SP")

    def zero_sbal_setpoint(self):
        self.sbal_roll_sp_var.set("0")
        self.sbal_pitch_sp_var.set("0")
        self.update_sbal_joystick_knob_from_sp()
        self.send_balance_command_sequence(["SBAL_SP_ZERO"], "Đã gửi SBAL_SP_ZERO")

    def reset_balance_plot_filter(self):
        # GUI không lọc dữ liệu; hàm này được giữ lại để tương thích với code cũ.
        self.sbal_plot_filter_ready = False
        self.sbal_plot_roll_f = 0.0
        self.sbal_plot_pitch_f = 0.0

    def filter_balance_plot_sample(self, raw_roll, raw_pitch):
        # ESP đã lọc IMU, nên đồ thị Balance vẽ trực tiếp dữ liệu raw nhận được.
        # File REC CSV vẫn lấy dữ liệu raw từ buffer như trước.
        return float(raw_roll), float(raw_pitch)

    def update_sbal_live_plot(self):
        if not self.running:
            return

        # Chỉ cập nhật đồ thị Balance khi tab Balance đang được mở.
        # Khi ở tab khác, không vẽ canvas ẩn để giảm tải cho mainloop.
        if hasattr(self, "notebook") and not self.is_balance_tab_active():
            self.root.after(250, self.update_sbal_live_plot)
            return

        roll = pitch = yaw = 0.0
        t_last = 0.0
        imu_rx_hz = 0.0
        latest_seq = self.sbal_plot_last_seq
        added_samples = 0

        if self.udp_imu is not None:
            # Chỉ lấy mẫu IMU mới nhất để hiển thị live.
            # Không lấy toàn bộ buffer để tránh GUI vẽ dồn quá nhiều điểm trong một lần cập nhật,
            # vì điều đó có thể tạo cảm giác bị giật dù ESP/Serial vẫn đang chạy mượt.
            latest_seq, t_last, roll, pitch, yaw, imu_rx_hz = self.udp_imu.get_latest_sample()

            if t_last > 0 and latest_seq > self.sbal_plot_last_seq:
                t = t_last - self.sbal_plot_t0
                if t < 0:
                    t = time.time() - self.sbal_plot_t0

                # GUI không lọc và không chặn spike; ESP đã xử lý dữ liệu trước khi gửi.
                self.sbal_plot_t.append(t)
                self.sbal_plot_roll.append(float(roll))
                self.sbal_plot_pitch.append(float(pitch))
                self.sbal_plot_last_seq = int(latest_seq)
                added_samples = 1

        self.sbal_plot_count_n += added_samples
        now = time.time()
        dt_hz = now - self.sbal_plot_count_t0
        if dt_hz >= 1.0:
            self.sbal_plot_hz = self.sbal_plot_count_n / dt_hz
            self.sbal_plot_count_n = 0
            self.sbal_plot_count_t0 = now

        imu_age = now - t_last if t_last > 0 else 999.0
        imu_state = "OK" if imu_age < 2.0 else "WAIT"

        if hasattr(self, "sbal_imu_hz_label"):
            self.sbal_imu_hz_label.config(
                text=(
                    f"Tần số IMU ESP: RX {imu_rx_hz:.1f} Hz | "
                    f"Plot Balance {self.sbal_plot_hz:.1f} Hz | "
                    f"RAW không lọc | seq={self.sbal_plot_last_seq} | {imu_state} | FOCUS={'ON' if self.balance_focus_mode else 'OFF'}"
                )
            )

        if hasattr(self, "sbal_line_roll"):
            xs = list(self.sbal_plot_t)
            self.sbal_line_roll.set_data(xs, list(self.sbal_plot_roll))
            self.sbal_line_pitch.set_data(xs, list(self.sbal_plot_pitch))

            roll_sp, pitch_sp = self.get_sbal_setpoints()
            self.sbal_roll_sp_line.set_ydata([roll_sp, roll_sp])
            self.sbal_pitch_sp_line.set_ydata([pitch_sp, pitch_sp])

            if len(xs) >= 2:
                self.sbal_ax.set_xlim(max(0, xs[-1] - self.sbal_plot_window_sec), xs[-1] + 0.5)

            if bool(BALANCE_PLOT_FIXED_Y):
                self.sbal_ax.set_ylim(ROLL_YLIM[0], ROLL_YLIM[1])
            else:
                all_y = list(self.sbal_plot_roll) + list(self.sbal_plot_pitch) + [
                    roll_sp, pitch_sp, -self.sbal_sp_limit_deg, self.sbal_sp_limit_deg
                ]
                if all_y:
                    ymin, ymax = min(all_y), max(all_y)
                    if abs(ymax-ymin) < 1:
                        ymin -= 1
                        ymax += 1
                    margin = max(2, (ymax-ymin)*0.15)
                    self.sbal_ax.set_ylim(ymin-margin, ymax+margin)

            if t_last > 0:
                self.sbal_status_label.config(
                    text=(
                        f"IMU thực tế | Roll={roll:+.2f}° Pitch={pitch:+.2f}° Yaw={yaw:+.2f}° | "
                        f"SP Roll={roll_sp:+.2f}° Pitch={pitch_sp:+.2f}° | "
                        f"RX={imu_rx_hz:.1f}Hz Plot={self.sbal_plot_hz:.1f}Hz | RAW không lọc | FOCUS={'ON' if self.balance_focus_mode else 'OFF'}"
                    )
                )
            else:
                self.sbal_status_label.config(
                    text=(
                        f"IMU thực tế: waiting... | RX={imu_rx_hz:.1f}Hz "
                        f"Plot={self.sbal_plot_hz:.1f}Hz"
                    )
                )

            # Chỉ vẽ lại khi có mẫu mới để giảm tải CPU và hạn chế lag do Matplotlib.
            if added_samples > 0:
                self.sbal_canvas.draw_idle()

        self.root.after(BALANCE_PLOT_UPDATE_MS, self.update_sbal_live_plot)

    def update_datetime_label(self):
        try:
            now_txt = datetime.now().strftime("%d/%m/%Y  %H:%M:%S")
            if hasattr(self, "datetime_label"):
                self.datetime_label.config(text=now_txt)
                self.datetime_label.lift()
            if getattr(self, "running", True):
                self.root.after(1000, self.update_datetime_label)
        except Exception:
            pass

    # ================== TAB CÀI ĐẶT ==================
    def load_saved_config(self):
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.apply_config_dict(data, recreate_pose=False, update_widgets=False, show_message=False)
            print(f"[CONFIG] Loaded: {CONFIG_PATH}")
        except Exception as e:
            print("[CONFIG] Loi load config:", e)


    # ================== TAB TỌA ĐỘ POSE ==================
    # ================== TAB TINH CHỈNH GAIT ==================
    def init_gait_config_vars(self):
        self.gait_vars = {}
        for mode in GAIT_MODES:
            defaults = GAIT_DEFAULTS[mode]
            self.gait_vars[mode] = {
                "hStep": tk.StringVar(value=f"{defaults['hStep']:.1f}"),
                "tCycle": tk.StringVar(value=f"{defaults['tCycle']:.2f}"),
                "phase": tk.StringVar(value=f"{defaults['phase']:.2f}"),
                "swing": tk.StringVar(value=f"{defaults['swing']:.2f}"),
            }
        self.load_gait_config_from_file(silent=True)

    def _safe_float_var(self, var, default, lo=None, hi=None):
        try:
            v = float(var.get())
        except Exception:
            v = float(default)
        if lo is not None:
            v = max(float(lo), v)
        if hi is not None:
            v = min(float(hi), v)
        return v

    def _get_gait_values(self, mode):
        defaults = GAIT_DEFAULTS[mode]
        gv = self.gait_vars[mode]
        h = self._safe_float_var(gv["hStep"], defaults["hStep"], -200.0, 200.0)
        t = self._safe_float_var(gv["tCycle"], defaults["tCycle"], 0.05, 10.0)
        phase = self._safe_float_var(gv["phase"], defaults["phase"], -100.0, 100.0)
        phase = phase - math.floor(phase)
        swing = self._safe_float_var(gv["swing"], defaults["swing"], 0.05, 0.49)
        gv["hStep"].set(f"{h:.2f}")
        gv["tCycle"].set(f"{t:.3f}")
        gv["phase"].set(f"{phase:.3f}")
        gv["swing"].set(f"{swing:.3f}")
        return h, t, phase, swing

    def build_gait_tab(self, parent):
        outer = ttk.Frame(parent, style="TFrame", padding=16)
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(outer, text="CHỈNH THÔNG SỐ GAIT DI CHUYỂN", style="Title.TLabel").pack(anchor="w", pady=(0, 10))


        table = ttk.Frame(outer, style="Panel.TFrame", padding=12)
        table.pack(fill=tk.X, pady=(0, 10))

        headers = ["Gait", "hStep", "tCycle (s)", "Phase", "Swing ratio", "Gửi", "Chạy thử"]
        for c, text in enumerate(headers):
            ttk.Label(table, text=text, style="SubTitle.TLabel").grid(row=0, column=c, padx=6, pady=(0, 8), sticky="w")
            table.columnconfigure(c, weight=1 if c in (1, 2, 3, 4) else 0)

        mode_labels = {
            "FWD": "FWD - Tiến",
            "BWD": "BWD - Lùi",
            "LEFT": "LEFT - Xoay trái",
            "RIGHT": "RIGHT - Xoay phải",
            "SL": "SL - Đi ngang trái",
            "SR": "SR - Đi ngang phải",
        }

        for r, mode in enumerate(GAIT_MODES, start=1):
            ttk.Label(table, text=mode_labels.get(mode, mode), style="Info.TLabel").grid(row=r, column=0, padx=6, pady=5, sticky="w")
            ttk.Entry(table, textvariable=self.gait_vars[mode]["hStep"], width=10, font=("Consolas", 13)).grid(row=r, column=1, padx=6, pady=5, sticky="ew")
            ttk.Entry(table, textvariable=self.gait_vars[mode]["tCycle"], width=10, font=("Consolas", 13)).grid(row=r, column=2, padx=6, pady=5, sticky="ew")
            ttk.Entry(table, textvariable=self.gait_vars[mode]["phase"], width=10, font=("Consolas", 13)).grid(row=r, column=3, padx=6, pady=5, sticky="ew")
            ttk.Entry(table, textvariable=self.gait_vars[mode]["swing"], width=10, font=("Consolas", 13)).grid(row=r, column=4, padx=6, pady=5, sticky="ew")
            ttk.Button(table, text="Gửi", style="CompactGreen.TButton", command=lambda m=mode: self.send_single_gait_config(m)).grid(row=r, column=5, padx=6, pady=5, sticky="ew")
            ttk.Button(table, text="Test", style="CompactBlue.TButton", command=lambda m=mode: self.test_gait_mode(m)).grid(row=r, column=6, padx=6, pady=5, sticky="ew")

        btn_bar = ttk.Frame(outer, style="Panel.TFrame", padding=12)
        btn_bar.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(btn_bar, text="GỬI TẤT CẢ GAIT", style="Green.TButton", command=self.send_all_gait_configs).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="LƯU FILE GUI", style="Blue.TButton", command=self.save_gait_config_to_file).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="LOAD FILE GUI", style="Gray.TButton", command=lambda: self.load_gait_config_from_file(silent=False)).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="RESET GUI DEFAULT", style="Gray.TButton", command=self.reset_gait_config_vars).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="RESET ESP GAITS", style="Red.TButton", command=self.send_reset_gaits_to_esp).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="SHOW ESP GAITS", style="Gray.TButton", command=lambda: self.send_udp_to_pi("SHOW_GAITS")).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="STOP", style="Red.TButton", command=self.send_manual_stop).pack(side=tk.RIGHT, padx=(8, 0))

        self.gait_status_label = ttk.Label(outer, text="Gait tab ready", style="Status.TLabel", padding=8)
        self.gait_status_label.pack(fill=tk.X, pady=(0, 10))

        guide = ttk.LabelFrame(outer, text="Lệnh ESP nhận được", padding=10)
        guide.pack(fill=tk.X)
        txt = (
            "SET_GAIT FWD 100 0.70 0.50 0.35\n"
            "SET_GAIT_HSTEP FWD 90\n"
            "SET_GAIT_TCYCLE SL 0.60\n"
            "SET_GAIT_PHASE RIGHT 0.50\n"
            "SET_GAIT_SWING BWD 0.32\n"
            "SHOW_GAITS / RESET_GAITS"
        )
        ttk.Label(guide, text=txt, style="Info.TLabel", justify=tk.LEFT).pack(anchor="w")

    def send_single_gait_config(self, mode):
        mode = str(mode).strip().upper()
        if mode not in GAIT_MODES:
            return False
        h, t, phase, swing = self._get_gait_values(mode)
        cmd = f"SET_GAIT {mode} {h:.3f} {t:.3f} {phase:.3f} {swing:.3f}"
        ok = self.send_udp_to_pi(cmd)
        if ok:
            self.last_status_msg = f"Đã gửi {cmd}"
            if self.gait_status_label:
                self.gait_status_label.config(text=self.last_status_msg)
            self.update_status()
        return ok

    def send_all_gait_configs(self):
        cmds = []
        for mode in GAIT_MODES:
            h, t, phase, swing = self._get_gait_values(mode)
            cmds.append(f"SET_GAIT {mode} {h:.3f} {t:.3f} {phase:.3f} {swing:.3f}")
        cmds.append("SHOW_GAITS")
        self.send_command_sequence_to_pi(cmds, "Đã gửi toàn bộ cấu hình gait")
        if self.gait_status_label:
            self.gait_status_label.config(text="Đã gửi toàn bộ cấu hình gait xuống ESP")

    def test_gait_mode(self, mode):
        mode = str(mode).strip().upper()
        self.send_single_gait_config(mode)
        self.cam_pid_enabled = False
        # Tắt CAMGAIT để kiểm tra gait thủ công.
        self.send_command_sequence_to_pi(["CAMGAIT_OFF", mode], f"Test gait {mode}")

    def reset_gait_config_vars(self):
        for mode in GAIT_MODES:
            for key, value in GAIT_DEFAULTS[mode].items():
                self.gait_vars[mode][key].set(f"{float(value):.3f}" if key != "hStep" else f"{float(value):.2f}")
        if self.gait_status_label:
            self.gait_status_label.config(text="Đã reset giá trị trên GUI về mặc định")

    def send_reset_gaits_to_esp(self):
        self.reset_gait_config_vars()
        self.send_command_sequence_to_pi(["RESET_GAITS", "SHOW_GAITS"], "Đã reset gait trong ESP")
        if self.gait_status_label:
            self.gait_status_label.config(text="Đã gửi RESET_GAITS xuống ESP")

    def save_gait_config_to_file(self):
        data = {}
        for mode in GAIT_MODES:
            h, t, phase, swing = self._get_gait_values(mode)
            data[mode] = {"hStep": h, "tCycle": t, "phase": phase, "swing": swing}
        try:
            with open(GAIT_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            if self.gait_status_label:
                self.gait_status_label.config(text=f"Đã lưu gait config: {GAIT_CONFIG_PATH}")
            messagebox.showinfo("Lưu gait", f"Đã lưu:\n{GAIT_CONFIG_PATH}")
        except Exception as e:
            messagebox.showerror("Lỗi lưu gait", str(e))

    def load_gait_config_from_file(self, silent=False):
        if not os.path.exists(GAIT_CONFIG_PATH):
            return
        try:
            with open(GAIT_CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            for mode in GAIT_MODES:
                if mode not in data:
                    continue
                for key in ("hStep", "tCycle", "phase", "swing"):
                    if key in data[mode]:
                        self.gait_vars[mode][key].set(str(data[mode][key]))
            if not silent:
                if self.gait_status_label:
                    self.gait_status_label.config(text=f"Đã load gait config: {GAIT_CONFIG_PATH}")
                messagebox.showinfo("Load gait", f"Đã load:\n{GAIT_CONFIG_PATH}")
        except Exception as e:
            if not silent:
                messagebox.showerror("Lỗi load gait", str(e))

    def build_pose_tab(self, parent):
        outer = ttk.Frame(parent, padding=10, style="TFrame")
        outer.pack(fill="both", expand=True)

        title_row = ttk.Frame(outer, style="TFrame")
        title_row.pack(fill="x", pady=(0, 8))
        ttk.Label(title_row, text="Cài đặt tọa độ các pose / tâm gait trong ESP", style="Title.TLabel").pack(side=tk.LEFT)
        self.pose_status_label = ttk.Label(title_row, text="Chưa gửi", style="Info.TLabel")
        self.pose_status_label.pack(side=tk.RIGHT)


        btn_bar = ttk.Frame(outer, style="TFrame")
        btn_bar.pack(fill="x", pady=(0, 8))
        ttk.Button(btn_bar, text="GỬI TẤT CẢ POSE", style="Green.TButton", command=self.send_all_pose_coords).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="LƯU THÔNG SỐ", style="Blue.TButton", command=self.save_pose_coords_to_file).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="LOAD THÔNG SỐ", style="Gray.TButton", command=self.load_pose_coords_from_file).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="MẶC ĐỊNH", style="Orange.TButton", command=self.reset_pose_vars_default).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="GỬI RESET_POSES ESP", style="Red.TButton", command=lambda: self.send_udp_to_pi("RESET_POSES")).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_bar, text="ĐỌC POSE TỪ ESP", style="Gray.TButton", command=lambda: self.send_udp_to_pi("SHOWPOSES")).pack(side=tk.LEFT, padx=(0, 8))

        canvas = tk.Canvas(outer, bg="#f8fafc", highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        content = ttk.Frame(canvas, style="TFrame")
        content.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=content, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        header = ttk.Frame(content, style="TFrame")
        header.pack(fill="x", pady=(0, 4))
        ttk.Label(header, text="Pose", width=9, style="Info.TLabel").grid(row=0, column=0, padx=3)
        col = 1
        for leg in POSE_LEGS:
            for axis in POSE_AXES:
                ttk.Label(header, text=f"{leg}_{axis}", width=8, style="Info.TLabel").grid(row=0, column=col, padx=2)
                col += 1
        ttk.Label(header, text="Thao tác", width=28, style="Info.TLabel").grid(row=0, column=col, padx=3)

        for pose_name in POSE_DEFAULTS.keys():
            row = ttk.Frame(content, style="TFrame")
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=pose_name, width=9, style="Header.TLabel").grid(row=0, column=0, padx=3, sticky="w")
            col = 1
            for leg in POSE_LEGS:
                for axis in POSE_AXES:
                    ent = ttk.Entry(row, width=8, textvariable=self.pose_vars[pose_name][leg][axis])
                    ent.grid(row=0, column=col, padx=2)
                    col += 1
            ttk.Button(row, text="Gửi", width=6, command=lambda n=pose_name: self.send_pose_coord(n)).grid(row=0, column=col, padx=(8, 2))
            ttk.Button(row, text="Áp dụng", width=8, command=lambda n=pose_name: self.apply_pose_coord(n)).grid(row=0, column=col+1, padx=2)
            ttk.Button(row, text="Default", width=8, command=lambda n=pose_name: self.reset_one_pose_default(n)).grid(row=0, column=col+2, padx=2)

    def collect_pose_coord(self, pose_name):
        vals = []
        for leg in POSE_LEGS:
            for axis in POSE_AXES:
                vals.append(float(self.pose_vars[pose_name][leg][axis].get()))
        return vals

    def pose_set_cmd(self, pose_name):
        vals = self.collect_pose_coord(pose_name)
        return "SET_POSE " + pose_name + " " + " ".join(f"{v:.2f}" for v in vals)

    def send_pose_coord(self, pose_name):
        try:
            cmd = self.pose_set_cmd(pose_name)
            ok = self.send_udp_to_pi(cmd)
            if ok and self.pose_status_label:
                self.pose_status_label.config(text=f"Đã gửi {pose_name}")
        except Exception as e:
            messagebox.showerror("Lỗi tọa độ", f"Pose {pose_name} có giá trị không hợp lệ:\n{e}")

    def apply_pose_coord(self, pose_name):
        self.send_pose_coord(pose_name)
        time.sleep(0.04)
        self.send_udp_to_pi(f"APPLY_POSE {pose_name}")
        if self.pose_status_label:
            self.pose_status_label.config(text=f"Đã áp dụng {pose_name}")

    def send_all_pose_coords(self):
        count = 0
        for pose_name in POSE_DEFAULTS.keys():
            try:
                self.send_udp_to_pi(self.pose_set_cmd(pose_name))
                count += 1
                time.sleep(0.025)
            except Exception as e:
                messagebox.showerror("Lỗi tọa độ", f"Pose {pose_name} lỗi:\n{e}")
                return
        if self.pose_status_label:
            self.pose_status_label.config(text=f"Đã gửi {count} pose")

    def reset_one_pose_default(self, pose_name):
        for leg in POSE_LEGS:
            for i, axis in enumerate(POSE_AXES):
                self.pose_vars[pose_name][leg][axis].set(f"{POSE_DEFAULTS[pose_name][leg][i]:.1f}")
        if self.pose_status_label:
            self.pose_status_label.config(text=f"Đã reset {pose_name} về mặc định GUI")

    def reset_pose_vars_default(self):
        for pose_name in POSE_DEFAULTS.keys():
            self.reset_one_pose_default(pose_name)
        if self.pose_status_label:
            self.pose_status_label.config(text="Đã reset tất cả về mặc định GUI")

    def save_pose_coords_to_file(self):
        data = {}
        try:
            for pose_name in POSE_DEFAULTS.keys():
                data[pose_name] = {}
                for leg in POSE_LEGS:
                    data[pose_name][leg] = [float(self.pose_vars[pose_name][leg][axis].get()) for axis in POSE_AXES]
            file_path = filedialog.asksaveasfilename(
                title="Lưu thông số tọa độ pose",
                initialfile="quadruped_pose_coords.json",
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            if not file_path:
                return
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            with open(POSE_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            if self.pose_status_label:
                self.pose_status_label.config(text=f"Đã lưu {os.path.basename(file_path)}")
        except Exception as e:
            messagebox.showerror("Lỗi lưu tọa độ", str(e))

    def load_pose_coords_from_file(self):
        file_path = filedialog.askopenfilename(
            title="Load thông số tọa độ pose",
            initialfile="quadruped_pose_coords.json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if not file_path:
            if os.path.exists(POSE_CONFIG_PATH):
                file_path = POSE_CONFIG_PATH
            else:
                return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            for pose_name, legs in data.items():
                if pose_name not in self.pose_vars:
                    continue
                for leg, vals in legs.items():
                    if leg not in POSE_LEGS or leg not in self.pose_vars[pose_name]:
                        continue
                    for i, axis in enumerate(POSE_AXES):
                        if i < len(vals):
                            self.pose_vars[pose_name][leg][axis].set(f"{float(vals[i]):.2f}")
            if self.pose_status_label:
                self.pose_status_label.config(text=f"Đã load {os.path.basename(file_path)}")
        except Exception as e:
            messagebox.showerror("Lỗi load tọa độ", str(e))

    def build_settings_tab(self, parent):
        outer = ttk.Frame(parent, style="TFrame", padding=16)
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(outer, text="CÀI ĐẶT THÔNG SỐ", style="Title.TLabel").pack(anchor="w", pady=(0, 10))

        form = ttk.Frame(outer, style="Panel.TFrame", padding=16)
        form.pack(fill=tk.BOTH, expand=True)

        self.setting_defs = [
            ("RASPBERRY_IP", "IP Raspberry nhận offset", "str", RASPBERRY_IP),
            ("RASPBERRY_OFFSET_PORT", "Port gửi offset về Raspberry", "int", RASPBERRY_OFFSET_PORT),
            ("CAMERA_UDP_PORT", "Port nhận camera từ Raspberry", "int", CAMERA_UDP_PORT),
            ("IMU_LISTEN_PORT", "Port nhận IMU từ Raspberry", "int", IMU_LISTEN_PORT),
            ("WIDTH", "Width xử lý MediaPipe", "int", WIDTH),
            ("HEIGHT", "Height xử lý MediaPipe", "int", HEIGHT),
            ("GAUSS_KERNEL_SIZE", "Gauss kernel size, số lẻ 1/3/5/7", "int", GAUSS_KERNEL[0]),
            ("GAUSS_SIGMA", "Gauss sigma", "float", GAUSS_SIGMA),
            ("MODEL_COMPLEXITY", "MediaPipe model complexity 0/1/2", "int", MODEL_COMPLEXITY),
            ("MIN_DETECTION_CONF", "Ngưỡng detection 0.1 - 0.9", "float", MIN_DETECTION_CONF),
            ("MIN_TRACKING_CONF", "Ngưỡng tracking 0.1 - 0.9", "float", MIN_TRACKING_CONF),
            ("FULL_BODY_VISIBILITY_TH", "Ngưỡng đủ đầu + 2 chân", "float", FULL_BODY_VISIBILITY_TH),
            ("CENTER_DEADBAND_PX", "Deadband offset pixel", "int", CENTER_DEADBAND_PX),
            ("SEND_INTERVAL", "Chu kỳ gửi offset, giây", "float", SEND_INTERVAL),
            ("LPF_ALPHA", "Lọc offset LPF alpha", "float", LPF_ALPHA),
            ("BOX_H_LIMIT", "Giới hạn box height", "int", BOX_H_LIMIT),
            ("OFFSET_SCALE_BY_BOX_W_ENABLE", "Bật giảm offset theo box width 0/1", "int", OFFSET_SCALE_BY_BOX_W_ENABLE),
            ("OFFSET_SCALE_REF_BOX_W", "Box width gốc - dưới mức này offset giữ nguyên", "int", OFFSET_SCALE_REF_BOX_W),
            ("OFFSET_SCALE_MIN_RATIO", "Tỉ lệ offset nhỏ nhất khi box width tăng", "float", OFFSET_SCALE_MIN_RATIO),
            ("PLOT_SECONDS_WINDOW", "Độ dài trục thời gian plot, giây", "float", PLOT_SECONDS_WINDOW),
            ("PLOT_MAX_POINTS", "Số điểm lưu plot", "int", PLOT_MAX_POINTS),
            ("PLOT_UPDATE_MS", "Chu kỳ cập nhật plot, ms", "int", PLOT_UPDATE_MS),
            ("ROLL_Y_MIN", "Roll Y min", "float", ROLL_YLIM[0]),
            ("ROLL_Y_MAX", "Roll Y max", "float", ROLL_YLIM[1]),
            ("PITCH_Y_MIN", "Pitch Y min", "float", PITCH_YLIM[0]),
            ("PITCH_Y_MAX", "Pitch Y max", "float", PITCH_YLIM[1]),
            ("YAW_Y_MIN", "Yaw Y min", "float", YAW_YLIM[0]),
            ("YAW_Y_MAX", "Yaw Y max", "float", YAW_YLIM[1]),
            ("CAMERA_DISPLAY_SCALE", "Tỉ lệ hiển thị camera 0.5 - 1.0", "float", CAMERA_DISPLAY_SCALE),
            ("CAM_PROCESS_EVERY_N", "MediaPipe xử lý mỗi N frame", "int", CAM_PROCESS_EVERY_N),
            ("CAM_MEDIAPIPE_ONLY_WHEN_PID", "Chỉ chạy MediaPipe khi PID ON 0/1", "int", CAM_MEDIAPIPE_ONLY_WHEN_PID),
            ("CAM_DRAW_LANDMARKS", "Vẽ skeleton MediaPipe 0/1", "int", CAM_DRAW_LANDMARKS),
            ("CAM_UPDATE_MS", "Chu kỳ update camera GUI, ms", "int", CAM_UPDATE_MS),
            ("GAZEBO_PUBLISH_ENABLE", "Bật publish Gazebo 0/1", "int", GAZEBO_PUBLISH_ENABLE),
            ("JOINT_PUBLISH_HZ_LIMIT", "Tần số publish Gazebo, Hz", "float", JOINT_PUBLISH_HZ_LIMIT),
            ("BALANCE_FOCUS_MODE_ENABLE", "Auto tạm dừng khi vào Balance 0/1", "int", BALANCE_FOCUS_MODE_ENABLE),
            ("BALANCE_PAUSE_CAMERA", "Balance dừng camera 0/1", "int", BALANCE_PAUSE_CAMERA),
            ("BALANCE_PAUSE_GAZEBO_PUBLISH", "Balance dừng publish GZB 0/1", "int", BALANCE_PAUSE_GAZEBO_PUBLISH),
            ("BALANCE_PLOT_FIXED_Y", "Balance plot khóa trục Y 0/1", "int", BALANCE_PLOT_FIXED_Y),
            ("BALANCE_PLOT_UPDATE_MS", "Chu kỳ vẽ Balance, ms", "int", BALANCE_PLOT_UPDATE_MS),
        ]

        split_row = (len(self.setting_defs) + 1) // 2
        for i, (key, label, typ, default) in enumerate(self.setting_defs):
            col = 0 if i < split_row else 3
            row = i if i < split_row else i - split_row
            ttk.Label(form, text=label, style="Info.TLabel").grid(row=row, column=col, sticky="w", padx=(0, 10), pady=5)
            var = tk.StringVar(value=str(default))
            self.setting_vars[key] = var
            ent = ttk.Entry(form, textvariable=var, width=18, font=("Consolas", 14))
            ent.grid(row=row, column=col + 1, sticky="w", padx=(0, 28), pady=5)

        btn_box = ttk.Frame(outer, style="TFrame")
        btn_box.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(btn_box, text="ÁP DỤNG THÔNG SỐ", style="Green.TButton", command=lambda: self.apply_settings_from_tab(save=False)).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(btn_box, text="LƯU CẤU HÌNH", style="Blue.TButton", command=lambda: self.apply_settings_from_tab(save=True)).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(btn_box, text="LOAD LẠI FILE", style="Gray.TButton", command=self.reload_config_to_tab).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(btn_box, text="GAZEBO PUB ON", style="Blue.TButton", command=lambda: self.set_gazebo_publish_from_gui(True)).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(btn_box, text="GAZEBO PUB OFF", style="Red.TButton", command=lambda: self.set_gazebo_publish_from_gui(False)).pack(side=tk.LEFT, padx=(0, 10))

        self.performance_status_label = ttk.Label(outer, text="Performance: MediaPipe N-frame / Gazebo publish có thể chỉnh trực tiếp ở đây.", style="Status.TLabel")
        self.performance_status_label.pack(anchor="w", fill=tk.X, pady=(10, 0))

        self.config_status_label = ttk.Label(outer, text=f"Config file: {CONFIG_PATH}", style="Status.TLabel")
        self.config_status_label.pack(anchor="w", fill=tk.X, pady=(10, 0))

    def parse_setting_value(self, key, typ):
        raw = self.setting_vars[key].get().strip()
        if typ == "int":
            return int(float(raw))
        if typ == "float":
            return float(raw)
        return raw

    def collect_settings_from_tab(self):
        data = {}
        for key, label, typ, default in self.setting_defs:
            data[key] = self.parse_setting_value(key, typ)
        k = int(data.get("GAUSS_KERNEL_SIZE", 5))
        if k < 1:
            k = 1
        if k % 2 == 0:
            k += 1
        data["GAUSS_KERNEL_SIZE"] = k
        data["CAMERA_DISPLAY_SCALE"] = max(0.3, min(1.0, float(data.get("CAMERA_DISPLAY_SCALE", 0.96))))
        data["LPF_ALPHA"] = max(0.0, min(0.99, float(data.get("LPF_ALPHA", 0.85))))
        data["OFFSET_SCALE_BY_BOX_W_ENABLE"] = 1 if int(float(data.get("OFFSET_SCALE_BY_BOX_W_ENABLE", OFFSET_SCALE_BY_BOX_W_ENABLE))) != 0 else 0
        data["OFFSET_SCALE_REF_BOX_W"] = max(1, min(10000, int(data.get("OFFSET_SCALE_REF_BOX_W", OFFSET_SCALE_REF_BOX_W))))
        data["OFFSET_SCALE_MIN_RATIO"] = max(0.0, min(1.0, float(data.get("OFFSET_SCALE_MIN_RATIO", OFFSET_SCALE_MIN_RATIO))))
        data["MIN_DETECTION_CONF"] = max(0.1, min(0.95, float(data.get("MIN_DETECTION_CONF", 0.5))))
        data["MIN_TRACKING_CONF"] = max(0.1, min(0.95, float(data.get("MIN_TRACKING_CONF", 0.5))))
        data["FULL_BODY_VISIBILITY_TH"] = max(0.1, min(0.95, float(data.get("FULL_BODY_VISIBILITY_TH", 0.45))))
        data["MODEL_COMPLEXITY"] = max(0, min(2, int(data.get("MODEL_COMPLEXITY", 0))))
        data["WIDTH"] = max(160, min(1280, int(data.get("WIDTH", WIDTH))))
        data["HEIGHT"] = max(120, min(720, int(data.get("HEIGHT", HEIGHT))))
        data["CAM_PROCESS_EVERY_N"] = max(1, min(30, int(data.get("CAM_PROCESS_EVERY_N", 3))))
        data["CAM_MEDIAPIPE_ONLY_WHEN_PID"] = 1 if int(float(data.get("CAM_MEDIAPIPE_ONLY_WHEN_PID", 1))) != 0 else 0
        data["CAM_DRAW_LANDMARKS"] = 1 if int(float(data.get("CAM_DRAW_LANDMARKS", 0))) != 0 else 0
        data["CAM_UPDATE_MS"] = max(1, min(100, int(data.get("CAM_UPDATE_MS", 10))))
        data["GAZEBO_PUBLISH_ENABLE"] = 1 if int(float(data.get("GAZEBO_PUBLISH_ENABLE", 0))) != 0 else 0
        data["JOINT_PUBLISH_HZ_LIMIT"] = max(1.0, min(100.0, float(data.get("JOINT_PUBLISH_HZ_LIMIT", 10.0))))
        data["BALANCE_FOCUS_MODE_ENABLE"] = 1 if int(float(data.get("BALANCE_FOCUS_MODE_ENABLE", BALANCE_FOCUS_MODE_ENABLE))) != 0 else 0
        data["BALANCE_PAUSE_CAMERA"] = 1 if int(float(data.get("BALANCE_PAUSE_CAMERA", BALANCE_PAUSE_CAMERA))) != 0 else 0
        data["BALANCE_PAUSE_GAZEBO_PUBLISH"] = 1 if int(float(data.get("BALANCE_PAUSE_GAZEBO_PUBLISH", BALANCE_PAUSE_GAZEBO_PUBLISH))) != 0 else 0
        data["BALANCE_PLOT_FILTER_ENABLE"] = 0
        data["BALANCE_PLOT_LPF_ALPHA"] = 0.0
        data["BALANCE_PLOT_MAX_STEP_DEG"] = 999.0
        data["BALANCE_PLOT_FIXED_Y"] = 1 if int(float(data.get("BALANCE_PLOT_FIXED_Y", 1))) != 0 else 0
        data["BALANCE_PLOT_UPDATE_MS"] = max(10, min(200, int(data.get("BALANCE_PLOT_UPDATE_MS", 30))))
        return data

    def apply_settings_from_tab(self, save=False):
        try:
            data = self.collect_settings_from_tab()
            self.apply_config_dict(data, recreate_pose=True, update_widgets=True, show_message=True)
            if save:
                with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                self.config_status_label.config(text=f"Đã lưu cấu hình: {CONFIG_PATH}")
                messagebox.showinfo("Lưu cấu hình", "Đã lưu thông số. Lần sau mở GUI sẽ tự load lại.")
            else:
                self.config_status_label.config(text="Đã áp dụng thông số hiện tại")
        except Exception as e:
            messagebox.showerror("Lỗi thông số", str(e))

    def apply_config_dict(self, data, recreate_pose=False, update_widgets=False, show_message=False):
        global RASPBERRY_IP, RASPBERRY_OFFSET_PORT, CAMERA_UDP_PORT, IMU_LISTEN_PORT
        global WIDTH, HEIGHT, GAUSS_KERNEL, GAUSS_SIGMA, CENTER_DEADBAND_PX
        global MIN_DETECTION_CONF, MIN_TRACKING_CONF, FULL_BODY_VISIBILITY_TH
        global SEND_INTERVAL, LPF_ALPHA, BOX_H_LIMIT, MODEL_COMPLEXITY
        global OFFSET_SCALE_BY_BOX_W_ENABLE, OFFSET_SCALE_REF_BOX_W, OFFSET_SCALE_MIN_RATIO
        global PLOT_SECONDS_WINDOW, PLOT_MAX_POINTS, PLOT_UPDATE_MS
        global ROLL_YLIM, PITCH_YLIM, YAW_YLIM, CAMERA_DISPLAY_SCALE
        global CAM_PROCESS_EVERY_N, CAM_MEDIAPIPE_ONLY_WHEN_PID, CAM_DRAW_LANDMARKS, CAM_UPDATE_MS
        global GAZEBO_PUBLISH_ENABLE, JOINT_PUBLISH_HZ_LIMIT
        global BALANCE_FOCUS_MODE_ENABLE, BALANCE_PAUSE_CAMERA, BALANCE_PAUSE_GAZEBO_PUBLISH
        global BALANCE_PLOT_FILTER_ENABLE, BALANCE_PLOT_LPF_ALPHA, BALANCE_PLOT_MAX_STEP_DEG, BALANCE_PLOT_FIXED_Y, BALANCE_PLOT_UPDATE_MS

        RASPBERRY_IP = str(data.get("RASPBERRY_IP", RASPBERRY_IP))
        RASPBERRY_OFFSET_PORT = int(data.get("RASPBERRY_OFFSET_PORT", RASPBERRY_OFFSET_PORT))
        CAMERA_UDP_PORT = int(data.get("CAMERA_UDP_PORT", CAMERA_UDP_PORT))
        IMU_LISTEN_PORT = int(data.get("IMU_LISTEN_PORT", IMU_LISTEN_PORT))
        WIDTH = int(data.get("WIDTH", WIDTH))
        HEIGHT = int(data.get("HEIGHT", HEIGHT))
        k = int(data.get("GAUSS_KERNEL_SIZE", GAUSS_KERNEL[0]))
        if k < 1:
            k = 1
        if k % 2 == 0:
            k += 1
        GAUSS_KERNEL = (k, k)
        GAUSS_SIGMA = float(data.get("GAUSS_SIGMA", GAUSS_SIGMA))
        MODEL_COMPLEXITY = int(data.get("MODEL_COMPLEXITY", MODEL_COMPLEXITY))
        MIN_DETECTION_CONF = float(data.get("MIN_DETECTION_CONF", MIN_DETECTION_CONF))
        MIN_TRACKING_CONF = float(data.get("MIN_TRACKING_CONF", MIN_TRACKING_CONF))
        FULL_BODY_VISIBILITY_TH = float(data.get("FULL_BODY_VISIBILITY_TH", FULL_BODY_VISIBILITY_TH))
        CENTER_DEADBAND_PX = int(data.get("CENTER_DEADBAND_PX", CENTER_DEADBAND_PX))
        SEND_INTERVAL = float(data.get("SEND_INTERVAL", SEND_INTERVAL))
        LPF_ALPHA = float(data.get("LPF_ALPHA", LPF_ALPHA))
        BOX_H_LIMIT = int(data.get("BOX_H_LIMIT", BOX_H_LIMIT))
        OFFSET_SCALE_BY_BOX_W_ENABLE = 1 if int(float(data.get("OFFSET_SCALE_BY_BOX_W_ENABLE", OFFSET_SCALE_BY_BOX_W_ENABLE))) != 0 else 0
        OFFSET_SCALE_REF_BOX_W = max(1, int(data.get("OFFSET_SCALE_REF_BOX_W", OFFSET_SCALE_REF_BOX_W)))
        OFFSET_SCALE_MIN_RATIO = max(0.0, min(1.0, float(data.get("OFFSET_SCALE_MIN_RATIO", OFFSET_SCALE_MIN_RATIO))))
        PLOT_SECONDS_WINDOW = float(data.get("PLOT_SECONDS_WINDOW", PLOT_SECONDS_WINDOW))
        PLOT_MAX_POINTS = int(data.get("PLOT_MAX_POINTS", PLOT_MAX_POINTS))
        PLOT_UPDATE_MS = int(data.get("PLOT_UPDATE_MS", PLOT_UPDATE_MS))
        ROLL_YLIM = (float(data.get("ROLL_Y_MIN", ROLL_YLIM[0])), float(data.get("ROLL_Y_MAX", ROLL_YLIM[1])))
        PITCH_YLIM = (float(data.get("PITCH_Y_MIN", PITCH_YLIM[0])), float(data.get("PITCH_Y_MAX", PITCH_YLIM[1])))
        YAW_YLIM = (float(data.get("YAW_Y_MIN", YAW_YLIM[0])), float(data.get("YAW_Y_MAX", YAW_YLIM[1])))
        CAMERA_DISPLAY_SCALE = float(data.get("CAMERA_DISPLAY_SCALE", CAMERA_DISPLAY_SCALE))

        CAM_PROCESS_EVERY_N = max(1, int(data.get("CAM_PROCESS_EVERY_N", CAM_PROCESS_EVERY_N)))
        CAM_MEDIAPIPE_ONLY_WHEN_PID = 1 if int(float(data.get("CAM_MEDIAPIPE_ONLY_WHEN_PID", CAM_MEDIAPIPE_ONLY_WHEN_PID))) != 0 else 0
        CAM_DRAW_LANDMARKS = 1 if int(float(data.get("CAM_DRAW_LANDMARKS", CAM_DRAW_LANDMARKS))) != 0 else 0
        CAM_UPDATE_MS = max(1, int(data.get("CAM_UPDATE_MS", CAM_UPDATE_MS)))
        GAZEBO_PUBLISH_ENABLE = 1 if int(float(data.get("GAZEBO_PUBLISH_ENABLE", GAZEBO_PUBLISH_ENABLE))) != 0 else 0
        JOINT_PUBLISH_HZ_LIMIT = max(1.0, float(data.get("JOINT_PUBLISH_HZ_LIMIT", JOINT_PUBLISH_HZ_LIMIT)))
        BALANCE_FOCUS_MODE_ENABLE = 1 if int(float(data.get("BALANCE_FOCUS_MODE_ENABLE", BALANCE_FOCUS_MODE_ENABLE))) != 0 else 0
        BALANCE_PAUSE_CAMERA = 1 if int(float(data.get("BALANCE_PAUSE_CAMERA", BALANCE_PAUSE_CAMERA))) != 0 else 0
        BALANCE_PAUSE_GAZEBO_PUBLISH = 1 if int(float(data.get("BALANCE_PAUSE_GAZEBO_PUBLISH", BALANCE_PAUSE_GAZEBO_PUBLISH))) != 0 else 0

        BALANCE_PLOT_FILTER_ENABLE = 0
        BALANCE_PLOT_LPF_ALPHA = 0.0
        BALANCE_PLOT_MAX_STEP_DEG = 999.0
        BALANCE_PLOT_FIXED_Y = 1 if int(float(data.get("BALANCE_PLOT_FIXED_Y", BALANCE_PLOT_FIXED_Y))) != 0 else 0
        BALANCE_PLOT_UPDATE_MS = max(10, min(200, int(data.get("BALANCE_PLOT_UPDATE_MS", BALANCE_PLOT_UPDATE_MS))))

        # Khi đổi thông số camera/MediaPipe, xóa cache frame cũ để tránh lệch kích thước.
        if hasattr(self, "last_processed_output"):
            self.last_processed_output = None
            self.cam_frame_count = 0

        if hasattr(self, "t_data") and self.t_data.maxlen != PLOT_MAX_POINTS:
            self.t_data = deque(self.t_data, maxlen=PLOT_MAX_POINTS)
            self.roll_data = deque(self.roll_data, maxlen=PLOT_MAX_POINTS)
            self.pitch_data = deque(self.pitch_data, maxlen=PLOT_MAX_POINTS)
            self.yaw_data = deque(self.yaw_data, maxlen=PLOT_MAX_POINTS)

        if update_widgets and hasattr(self, "setting_vars"):
            for key, var in self.setting_vars.items():
                if key == "GAUSS_KERNEL_SIZE":
                    var.set(str(GAUSS_KERNEL[0]))
                elif key == "ROLL_Y_MIN":
                    var.set(str(ROLL_YLIM[0]))
                elif key == "ROLL_Y_MAX":
                    var.set(str(ROLL_YLIM[1]))
                elif key == "PITCH_Y_MIN":
                    var.set(str(PITCH_YLIM[0]))
                elif key == "PITCH_Y_MAX":
                    var.set(str(PITCH_YLIM[1]))
                elif key == "YAW_Y_MIN":
                    var.set(str(YAW_YLIM[0]))
                elif key == "YAW_Y_MAX":
                    var.set(str(YAW_YLIM[1]))
                elif key in globals():
                    var.set(str(globals()[key]))
        if recreate_pose and hasattr(self, "pose") and self.pose is not None:
            try:
                self.pose.close()
            except Exception:
                pass
            self.init_mediapipe()
        if hasattr(self, "setup_plot_axes") and hasattr(self, "canvas"):
            try:
                self.setup_plot_axes()
                self.canvas.draw_idle()
            except Exception:
                pass
        if hasattr(self, "info_pi_ip"):
            self.info_pi_ip.config(text=f"Pi IP: {RASPBERRY_IP}:{RASPBERRY_OFFSET_PORT}")
            self.info_cam_port.config(text=f"Cam UDP: {CAMERA_UDP_PORT}")
            self.info_imu_port.config(text=f"IMU UDP: {IMU_LISTEN_PORT}")

        self.apply_runtime_performance_settings()

        if bool(BALANCE_FOCUS_MODE_ENABLE):
            try:
                self.on_notebook_tab_changed()
            except Exception:
                pass
        else:
            try:
                self.exit_balance_focus_mode()
            except Exception:
                pass

        if show_message:
            print("[CONFIG] Applied", data)

    def apply_runtime_performance_settings(self):
        """Áp dụng các thông số hiệu năng cho các thread đang chạy mà không cần tắt GUI."""
        try:
            if hasattr(self, "joint_mapper_panel") and self.joint_mapper_panel is not None:
                if not getattr(self, "balance_focus_mode", False):
                    self.joint_mapper_panel.publish_enabled.set(bool(GAZEBO_PUBLISH_ENABLE))
        except Exception as e:
            print("[PERF] Không áp dụng được Gazebo publish setting:", e)

        try:
            if hasattr(self, "performance_status_label"):
                self.performance_status_label.config(
                    text=(
                        f"Performance: MediaPipe mỗi {CAM_PROCESS_EVERY_N} frame | "
                        f"Only PID={CAM_MEDIAPIPE_ONLY_WHEN_PID} | Skeleton={CAM_DRAW_LANDMARKS} | "
                        f"Gazebo publish={'ON' if GAZEBO_PUBLISH_ENABLE else 'OFF'} @ {JOINT_PUBLISH_HZ_LIMIT:.1f} Hz | "
                        f"Balance focus={'ON' if BALANCE_FOCUS_MODE_ENABLE else 'OFF'} | "
                        f"Balance plot RAW, update={BALANCE_PLOT_UPDATE_MS} ms | "
                        f"Offset scale={'ON' if OFFSET_SCALE_BY_BOX_W_ENABLE else 'OFF'} "
                        f"refW={OFFSET_SCALE_REF_BOX_W}px min={OFFSET_SCALE_MIN_RATIO:.2f}"
                    )
                )
        except Exception:
            pass

    def set_gazebo_publish_from_gui(self, enabled):
        global GAZEBO_PUBLISH_ENABLE
        GAZEBO_PUBLISH_ENABLE = 1 if enabled else 0
        if hasattr(self, "setting_vars") and "GAZEBO_PUBLISH_ENABLE" in self.setting_vars:
            self.setting_vars["GAZEBO_PUBLISH_ENABLE"].set(str(GAZEBO_PUBLISH_ENABLE))
        self.apply_runtime_performance_settings()
        self.last_status_msg = "Gazebo publish ON" if enabled else "Gazebo publish OFF"
        try:
            self.update_status()
        except Exception:
            pass

    def reload_config_to_tab(self):
        self.load_saved_config()
        self.apply_config_dict({}, recreate_pose=True, update_widgets=True, show_message=False)
        if hasattr(self, "config_status_label"):
            self.config_status_label.config(text=f"Đã load lại file: {CONFIG_PATH}")

    def setup_plot_axes(self):
        axes = [
            (self.ax_roll, "ROLL (deg)", ROLL_YLIM, "tab:blue"),
            (self.ax_pitch, "PITCH (deg)", PITCH_YLIM, "tab:orange"),
            (self.ax_yaw, "YAW (deg)", YAW_YLIM, "tab:green"),
        ]

        for ax, title, ylim, color in axes:
            ax.set_facecolor("#ffffff")
            ax.set_title(title, fontsize=12, fontweight="bold", color=color)
            ax.set_ylabel("deg", fontsize=10, color="#0f172a")
            ax.set_ylim(*ylim)
            ax.grid(True, color="#cbd5e1", linewidth=0.8)
            ax.tick_params(axis="both", labelsize=9, colors="#0f172a")
            for spine in ax.spines.values():
                spine.set_color("#94a3b8")
            ax.legend(loc="upper right", fontsize=8, facecolor="#ffffff", edgecolor="#cbd5e1")

        self.ax_yaw.set_xlabel("Time (s)", fontsize=10, color="#0f172a")

    # ================== ĐIỀU KHIỂN TOÀN MÀN HÌNH ==================
    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def end_fullscreen(self):
        self.fullscreen = False
        self.root.attributes("-fullscreen", False)
        self.root.geometry("1500x850")

    # ================== KHỞI TẠO ==================
    def init_mediapipe(self):
        self.pose = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=MODEL_COMPLEXITY,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=MIN_DETECTION_CONF,
            min_tracking_confidence=MIN_TRACKING_CONF
        )

    def init_udp_camera(self):
        try:
            self.udp_cam = UdpCameraReceiver(UDP_IP, CAMERA_UDP_PORT)
            self.udp_cam.start()
            self.last_status_msg = f"Đang nhận camera UDP port {CAMERA_UDP_PORT}"
        except Exception as e:
            messagebox.showerror("UDP Camera error", str(e))
            self.last_status_msg = "Không mở được UDP camera"
        self.update_status()

    def init_udp_imu(self):
        try:
            self.udp_imu = UdpImuReceiver(IMU_LISTEN_IP, IMU_LISTEN_PORT)
            self.udp_imu.start()
        except Exception as e:
            messagebox.showerror("UDP IMU error", str(e))

    # ================== GỬI UDP ĐẾN PI ==================
    def send_udp_to_pi(self, msg):
        try:
            self.offset_sock.sendto(
                msg.encode("utf-8"),
                (RASPBERRY_IP, RASPBERRY_OFFSET_PORT)
            )
            self.last_udp_msg = msg
            print("[UDP TO PI]", msg)
            return True
        except Exception as e:
            print("[ERR] Loi gui UDP ve Raspberry:", e)
            self.last_status_msg = "Lỗi gửi UDP về Raspberry"
            self.update_status()
            return False

    def send_command_sequence_to_pi(self, commands, final_message):
        old_send_state = self.send_camera_data
        self.send_camera_data = False

        try:
            time.sleep(0.05)

            for cmd in commands:
                self.send_udp_to_pi(cmd)
                time.sleep(0.12)

            self.last_status_msg = final_message
            print("[SEQ UDP]", " -> ".join(commands))

        finally:
            self.send_camera_data = old_send_state
            self.last_send_time = time.time()
            self.update_status()

    # ================== NÚT ĐIỀU KHIỂN THỦ CÔNG ==================
    def send_manual_cmd(self, cmd):
        """Gửi lệnh tư thế hoặc lệnh đơn từ GUI xuống Raspberry, sau đó chuyển tiếp đến ESP."""
        self.cam_pid_enabled = False

        cmd = str(cmd).strip().upper()

        # Với các lệnh tư thế, nên tắt camera gait để ESP không tiếp tục PID camera sau đó.
        if cmd in ("STAND", "SIT", "LIE", "SHAKE"):
            seq = ["CAMGAIT_OFF", cmd]
        else:
            seq = [cmd]

        self.send_command_sequence_to_pi(seq, f"GUI gửi lệnh {cmd} về Pi")
        self.update_status()

    def send_manual_motion(self, cmd):
        """Gửi lệnh di chuyển thủ công từ GUI và tự tắt CAMGAIT để robot nhận lệnh tay bình thường."""
        self.cam_pid_enabled = False

        cmd = str(cmd).strip().upper()
        self.send_command_sequence_to_pi(["CAMGAIT_OFF", cmd], f"Manual {cmd} gửi về Pi")
        self.update_status()

    def send_manual_stop(self):
        self.cam_pid_enabled = False

        self.send_command_sequence_to_pi(["STOP", "CAMGAIT_OFF"], "Manual STOP + CAMGAIT_OFF gửi về Pi")
        self.update_status()

    # ================== ĐIỀU CHỈNH CHIỀU CAO ==================
    def _clamp_height_x(self, value):
        return max(HEIGHT_MIN_X, min(HEIGHT_MAX_X, float(value)))

    def send_height_value(self, x_value):
        """Giữ tương thích với cách điều khiển cũ bằng cách quy đổi X mong muốn thành số lần X+/X-."""
        self.cam_pid_enabled = False
        target = self._clamp_height_x(x_value)
        delta = target - self.height_x
        steps = int(round(delta / self.height_step_x))
        if steps > 0:
            cmds = ["X+"] * steps
        elif steps < 0:
            cmds = ["X-"] * abs(steps)
        else:
            cmds = []
        self.height_x = self._clamp_height_x(self.height_x + steps * self.height_step_x)
        if hasattr(self, "height_label"):
            self.height_label.config(text=f"X={self.height_x:.0f} mm")
        if cmds:
            self.send_command_sequence_to_pi(cmds, f"Đã gửi đổi X bằng {len(cmds)} bước, X dự kiến={self.height_x:.1f} mm")
            self.last_udp_msg = cmds[-1]
        self.update_status()

    def send_height_up(self):
        # ESP nhận trực tiếp X+ và tự tăng trục X của các pose/gait FWD, BWD, LEFT, RIGHT, STOP, SL, SR thêm 10 đơn vị.
        self.height_x = self._clamp_height_x(self.height_x + self.height_step_x)
        if hasattr(self, "height_label"):
            self.height_label.config(text=f"X={self.height_x:.0f} mm")
        self.send_command_sequence_to_pi(["X+"], f"Đã gửi X+ / CAO+, X dự kiến={self.height_x:.1f} mm")
        self.last_udp_msg = "X+"
        self.update_status()

    def send_height_down(self):
        # ESP nhận trực tiếp X- và tự giảm trục X của các pose/gait FWD, BWD, LEFT, RIGHT, STOP, SL, SR đi 10 đơn vị.
        self.height_x = self._clamp_height_x(self.height_x - self.height_step_x)
        if hasattr(self, "height_label"):
            self.height_label.config(text=f"X={self.height_x:.0f} mm")
        self.send_command_sequence_to_pi(["X-"], f"Đã gửi X- / CAO-, X dự kiến={self.height_x:.1f} mm")
        self.last_udp_msg = "X-"
        self.update_status()

    def send_height_reset(self):
        self.height_x = HEIGHT_DEFAULT_X
        if hasattr(self, "height_label"):
            self.height_label.config(text=f"X={self.height_x:.0f} mm")
        self.send_command_sequence_to_pi(["X_RESET"], f"Đã gửi X_RESET, X dự kiến={self.height_x:.1f} mm")
        self.last_udp_msg = "X_RESET"
        self.update_status()

    # ================== NÚT PID CAMERA ==================
    def start_cam_pid(self):
        self.cam_pid_enabled = True

        cmds = []

        if SEND_PID_SETTING_ON_START:
            cmds.append(CAM_PID_CMD)

        if SEND_STOP_SETTING_ON_START:
            cmds.append(CAM_STOP_CMD)

        if AUTO_STAND_BEFORE_FWD:
            cmds.append("STAND")

        cmds.append("CAMGAIT_ON")
        cmds.append("FWD")

        self.send_command_sequence_to_pi(cmds, "PID cam ON + FWD gửi về Pi")
        self.update_status()

    def stop_cam_pid(self):
        self.cam_pid_enabled = False


        self.send_command_sequence_to_pi(
            ["STOP", "CAMGAIT_OFF"],
            "STOP + PID cam OFF gửi về Pi"
        )
        self.update_status()

    # ================== THÔNG TIN TRẠNG THÁI ==================
    def update_status(self):
        self.pid_state_label.config(
            text="PID cam: ON" if self.cam_pid_enabled else "PID cam: OFF"
        )
        self.status_label.config(text=self.last_status_msg)

    def update_info_labels(self):
        if self.udp_cam is not None:
            dt = time.time() - self.udp_cam.last_packet_time if self.udp_cam.last_packet_time > 0 else 999
            if dt < 2.0:
                self.info_udp.config(text=f"UDP CAM: OK {self.udp_cam.recv_fps:.1f} fps")
            else:
                self.info_udp.config(text="UDP CAM: waiting...")

        if self.udp_imu is not None:
            roll, pitch, yaw, t_last = self.udp_imu.get_imu()
            dt = time.time() - t_last if t_last > 0 else 999
            if dt < 2.0:
                self.info_imu.config(text="UDP IMU: OK")
            else:
                self.info_imu.config(text="UDP IMU: waiting...")

        self.info_gui_fps.config(text=f"GUI FPS: {self.fps:.1f}")

        self.update_box_h_status()
        self.debug_label.config(text=f"Last UDP: {self.last_udp_msg}")
        self.box_debug_label.config(text=self.esp_box_status)

    def update_box_h_status(self):
        if self.detected_val == 0:
            self.esp_box_status = f"BOX H: NO FULL BODY | limit={BOX_H_LIMIT}"
            return

        if self.box_height < BOX_H_LIMIT:
            self.esp_box_status = f"BOX H: {self.box_height} < {BOX_H_LIMIT}  | RUN"
        else:
            self.esp_box_status = f"BOX H: {self.box_height} >= {BOX_H_LIMIT} | STOP"

    # ================== KIỂM TRA TOÀN THÂN ==================
    def is_full_body_visible(self, landmarks, visibility_th=FULL_BODY_VISIBILITY_TH):
        lm = landmarks.landmark

        def visible(point):
            return lm[point].visibility >= visibility_th

        head_ok = visible(mp_pose.PoseLandmark.NOSE)

        left_leg_ok = (
            visible(mp_pose.PoseLandmark.LEFT_ANKLE) or
            visible(mp_pose.PoseLandmark.LEFT_FOOT_INDEX)
        )

        right_leg_ok = (
            visible(mp_pose.PoseLandmark.RIGHT_ANKLE) or
            visible(mp_pose.PoseLandmark.RIGHT_FOOT_INDEX)
        )

        return head_ok and left_leg_ok and right_leg_ok

    # ================== VẼ HIỂN THỊ ==================
    def draw_text_with_bg(self, img, text, org, scale=0.5,
                          text_color=(255, 255, 255),
                          bg_color=(0, 0, 0),
                          thickness=1):
        x, y = org
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        pad = 4
        cv2.rectangle(
            img,
            (x - pad, y - th - pad),
            (x + tw + pad, y + baseline + pad),
            bg_color,
            -1
        )
        cv2.putText(img, text, (x, y), font, scale, text_color, thickness, cv2.LINE_AA)

    def draw_esp_debug_on_frame(self, output):
        self.update_box_h_status()

        h, w = output.shape[:2]
        x = 10
        y = h - 18

        line = self.esp_box_status

        if "RUN" in line:
            color = (0, 255, 0)
        elif "STOP" in line:
            color = (0, 0, 255)
        else:
            color = (0, 255, 255)

        self.draw_text_with_bg(
            output,
            line,
            (x, y),
            scale=0.65,
            text_color=color,
            bg_color=(0, 0, 0),
            thickness=2
        )

    # ================== XỬ LÝ CAMERA ==================
    def process_frame(self, frame):
        frame = cv2.resize(frame, (WIDTH, HEIGHT))

        blur = cv2.GaussianBlur(frame, GAUSS_KERNEL, GAUSS_SIGMA)

        rgb = cv2.cvtColor(blur, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.pose.process(rgb)
        rgb.flags.writeable = True

        output = blur.copy()
        h, w, _ = output.shape

        self.detected_val = 0
        self.full_body_ok = False
        self.offset_x = 0
        self.direction = "NO_PERSON"
        self.box_width = 0
        self.box_height = 0
        self.person_center_x = 0
        self.person_center_y = 0

        screen_center_x = w // 2

        cv2.line(output, (screen_center_x, 0), (screen_center_x, h), (255, 0, 0), 2)

        if result.pose_landmarks:
            full_ok = self.is_full_body_visible(result.pose_landmarks)

            if full_ok:
                self.detected_val = 1
                self.full_body_ok = True
            else:
                self.detected_val = 0
                self.full_body_ok = False
                self.direction = "NOT_FULL_BODY"

            if CAM_DRAW_LANDMARKS:
                mp_drawing.draw_landmarks(
                    output,
                    result.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style()
                )

            xs = []
            ys = []

            for lm in result.pose_landmarks.landmark:
                if lm.visibility > 0.4:
                    xs.append(int(lm.x * w))
                    ys.append(int(lm.y * h))

            if xs and ys:
                x_min = max(min(xs) - 20, 0)
                y_min = max(min(ys) - 20, 0)
                x_max = min(max(xs) + 20, w)
                y_max = min(max(ys) + 20, h)

                self.box_width = x_max - x_min
                self.box_height = y_max - y_min

                self.person_center_x = (x_min + x_max) // 2
                self.person_center_y = (y_min + y_max) // 2

                if self.full_body_ok:
                    self.offset_x = self.person_center_x - screen_center_x

                    if self.offset_x < -CENTER_DEADBAND_PX:
                        self.direction = "LEFT"
                    elif self.offset_x > CENTER_DEADBAND_PX:
                        self.direction = "RIGHT"
                    else:
                        self.direction = "CENTER"
                else:
                    self.offset_x = 0
                    self.direction = "NOT_FULL_BODY"

                box_color = (0, 255, 0) if self.full_body_ok else (0, 0, 255)

                cv2.rectangle(output, (x_min, y_min), (x_max, y_max), box_color, 2)
                cv2.circle(output, (self.person_center_x, self.person_center_y), 6, (0, 0, 255), -1)

                cv2.line(
                    output,
                    (screen_center_x, self.person_center_y),
                    (self.person_center_x, self.person_center_y),
                    (0, 255, 255),
                    2
                )

                label = "FULL BODY - SEND TO PI" if self.full_body_ok else "NEED HEAD + BOTH FEET"

        status_text = "FULL BODY - SEND" if self.full_body_ok else "NO SEND"
        status_color = (0, 255, 0) if self.full_body_ok else (0, 0, 255)

        # Cập nhật LPF và snapshot sau mỗi lần MediaPipe xử lý frame.
        # File CSV sẽ lưu đủ offset raw, offset sau LPF và offset đã scale để gửi xuống bridge.
        self.update_lpf_values()
        self.update_offset_record_snapshot()

        return output

    # ================== BỘ LỌC LPF ==================
    def reset_lpf_values(self):
        self.lpf_ready = False
        self.f_offset_x = 0.0
        self.f_box_width = 0.0
        self.f_box_height = 0.0
        self.f_center_x = 0.0
        self.f_center_y = 0.0

    def update_lpf_values(self):
        if not self.full_body_ok:
            self.reset_lpf_values()
            return

        if not self.lpf_ready:
            self.f_offset_x = float(self.offset_x)
            self.f_box_width = float(self.box_width)
            self.f_box_height = float(self.box_height)
            self.f_center_x = float(self.person_center_x)
            self.f_center_y = float(self.person_center_y)
            self.lpf_ready = True
            return

        a = LPF_ALPHA
        b = 1.0 - LPF_ALPHA

        self.f_offset_x = a * self.f_offset_x + b * float(self.offset_x)
        self.f_box_width = a * self.f_box_width + b * float(self.box_width)
        self.f_box_height = a * self.f_box_height + b * float(self.box_height)
        self.f_center_x = a * self.f_center_x + b * float(self.person_center_x)
        self.f_center_y = a * self.f_center_y + b * float(self.person_center_y)

    # ================== SCALE OFFSET VÀ SNAPSHOT REC ==================
    def get_offset_box_width_scale(self):
        """Tính hệ số giảm offset theo bề rộng khung người.

        Quy tắc tính:
          - Nếu box_width_lpf <= OFFSET_SCALE_REF_BOX_W thì scale = 1.0.
          - Nếu box_width_lpf > OFFSET_SCALE_REF_BOX_W thì scale = REF / box_width_lpf.
          - scale không nhỏ hơn OFFSET_SCALE_MIN_RATIO.
        """
        try:
            if not int(OFFSET_SCALE_BY_BOX_W_ENABLE):
                return 1.0

            ref_w = max(1.0, float(OFFSET_SCALE_REF_BOX_W))
            min_ratio = max(0.0, min(1.0, float(OFFSET_SCALE_MIN_RATIO)))
            box_w = float(self.f_box_width) if self.lpf_ready else float(self.box_width)

            if box_w <= ref_w:
                return 1.0

            scale = ref_w / max(box_w, 1.0)
            return max(min_ratio, min(1.0, scale))
        except Exception:
            return 1.0

    def get_scaled_offset_for_bridge(self):
        """Trả về đúng giá trị offset sẽ gửi xuống bridge.

        offset_send_px = offset_lpf_px * offset_box_width_scale.
        """
        base_offset = float(self.f_offset_x) if self.lpf_ready else float(self.offset_x)
        scale = self.get_offset_box_width_scale()
        offset_send = base_offset * scale
        return base_offset, scale, offset_send

    def update_offset_record_snapshot(self):
        now = time.time()
        _offset_lpf_for_send, offset_scale, offset_send = self.get_scaled_offset_for_bridge()
        self.offset_record_seq += 1
        self.offset_last_update_time = now
        self.offset_record_snapshot = {
            "seq": int(self.offset_record_seq),
            "unix_time": float(now),
            "cam_pid_enabled": 1 if self.cam_pid_enabled else 0,
            "mediapipe_enabled": 1 if (self.pose is not None and (not bool(CAM_MEDIAPIPE_ONLY_WHEN_PID) or self.cam_pid_enabled)) else 0,
            "detected": int(self.detected_val),
            "full_body_ok": 1 if self.full_body_ok else 0,
            "direction": str(self.direction),
            "offset_x_raw_px": float(self.offset_x),
            "offset_x_lpf_px": float(self.f_offset_x) if self.lpf_ready else 0.0,
            "offset_x_send_px": float(offset_send) if self.full_body_ok else 0.0,
            "offset_box_width_scale": float(offset_scale) if self.full_body_ok else 1.0,
            "offset_scale_ref_box_w_px": float(OFFSET_SCALE_REF_BOX_W),
            "offset_scale_enabled": int(OFFSET_SCALE_BY_BOX_W_ENABLE),
            "box_width_raw_px": float(self.box_width),
            "box_width_lpf_px": float(self.f_box_width) if self.lpf_ready else 0.0,
            "box_height_raw_px": float(self.box_height),
            "box_height_lpf_px": float(self.f_box_height) if self.lpf_ready else 0.0,
            "center_x_raw_px": float(self.person_center_x),
            "center_x_lpf_px": float(self.f_center_x) if self.lpf_ready else 0.0,
            "center_y_raw_px": float(self.person_center_y),
            "center_y_lpf_px": float(self.f_center_y) if self.lpf_ready else 0.0,
        }

    def get_offset_record_snapshot(self, ref_time=None):
        snap = dict(getattr(self, "offset_record_snapshot", {}))
        if not snap:
            self.update_offset_record_snapshot()
            snap = dict(self.offset_record_snapshot)
        if ref_time is None:
            ref_time = time.time()
        snap_time = float(snap.get("unix_time", 0.0) or 0.0)
        snap["age_s"] = float(ref_time - snap_time) if snap_time > 0 else 999.0
        return snap

    # ================== GỬI OFFSET ==================
    def send_camera_packet(self):
        now = time.time()

        if not self.send_camera_data:
            return

        if now - self.last_send_time < SEND_INTERVAL:
            return

        self.last_send_time = now

        if not self.full_body_ok:
            self.reset_lpf_values()
            self.update_offset_record_snapshot()
            return

        # LPF đã được cập nhật trong process_frame(), nên không lọc thêm lần nữa tại đây.
        # Offset gửi xuống bridge được giảm theo tỉ lệ bề rộng khung người.
        _base_offset, _offset_scale, _offset_send = self.get_scaled_offset_for_bridge()

        send_offset_x = int(round(_offset_send))
        send_box_width = int(round(self.f_box_width))
        send_box_height = int(round(self.f_box_height))
        send_center_x = int(round(self.f_center_x))
        send_center_y = int(round(self.f_center_y))

        msg = (
            f"{self.detected_val},"
            f"{send_offset_x},"
            f"{self.direction},"
            f"{send_box_width},"
            f"{send_box_height},"
            f"{send_center_x},"
            f"{send_center_y}"
        )

        # Cập nhật snapshot ngay trước khi gửi để REC lưu đúng offset thực tế gửi xuống bridge.
        self.update_offset_record_snapshot()
        self.send_udp_to_pi(msg)

    def make_tk_image_fit_label(self, output_bgr):
        """
        Camera vẫn được xử lý ở kích thước WIDTH x HEIGHT.
        Riêng phần hiển thị sẽ phóng ảnh theo khung GUI trước,
        sau đó mới vẽ chữ overlay để chữ sắc nét hơn và không bị mờ do resize.
        """
        label_w = max(self.video_label.winfo_width(), 1)
        label_h = max(self.video_label.winfo_height(), 1)

        max_w = int(label_w * CAMERA_DISPLAY_SCALE)
        max_h = int(label_h * CAMERA_DISPLAY_SCALE)

        if max_w <= 1 or max_h <= 1:
            max_w, max_h = 960, 720

        h, w = output_bgr.shape[:2]
        scale = min(max_w / w, max_h / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))

        # Phóng ảnh trước.
        display_bgr = cv2.resize(output_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Vẽ chữ sau khi phóng để chữ hiển thị rõ hơn.
        self.draw_camera_overlay_text(display_bgr)

        output_rgb = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(output_rgb)
        return ImageTk.PhotoImage(image=img)

    def draw_camera_overlay_text(self, img):
        """Vẽ chữ lên ảnh sau khi phóng lớn để nội dung hiển thị rõ hơn."""
        h, w = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Font tự thay đổi theo kích thước hiển thị.
        scale_big = max(0.65, min(1.15, w / 850.0))
        scale_small = max(0.55, min(0.9, w / 1050.0))
        thick_big = max(2, int(round(scale_big * 2)))
        thick_small = max(1, int(round(scale_small * 2)))

        def put_bg(text, x, y, scale, color, thickness):
            (tw, th), base = cv2.getTextSize(text, font, scale, thickness)
            pad = 6
            cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + base + pad), (0, 0, 0), -1)
            cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

        if bool(CAM_MEDIAPIPE_ONLY_WHEN_PID) and not self.cam_pid_enabled:
            status_text = "PID OFF - CAMERA ONLY"
            status_color = (0, 255, 255)
        else:
            status_text = "FULL BODY - SEND" if self.full_body_ok else "NO SEND"
            status_color = (0, 255, 0) if self.full_body_ok else (0, 0, 255)

        put_bg(status_text, 14, 34, scale_big, status_color, thick_big)
        put_bg(f"Offset X: {self.offset_x}px", 14, int(34 + 40 * scale_big), scale_small, (0, 255, 255), thick_small)

        if self.lpf_ready and self.full_body_ok:
            put_bg(
                f"LPF a={LPF_ALPHA}: off={int(round(self.f_offset_x))} | w={int(round(self.f_box_width))}",
                14,
                int(34 + 78 * scale_big),
                scale_small,
                (255, 200, 0),
                thick_small
            )
            _base_off, _scale, _send_off = self.get_scaled_offset_for_bridge()
            put_bg(
                f"Send offset={int(round(_send_off))} | scale={_scale:.2f} | refW={OFFSET_SCALE_REF_BOX_W}",
                14,
                int(34 + 116 * scale_big),
                scale_small,
                (255, 255, 0),
                thick_small
            )

        if not self.full_body_ok and not (bool(CAM_MEDIAPIPE_ONLY_WHEN_PID) and not self.cam_pid_enabled):
            put_bg(
                "Need head + left foot + right foot",
                14,
                int(34 + 78 * scale_big),
                scale_small,
                (0, 0, 255),
                thick_small
            )

    # ================== CẬP NHẬT CAMERA ==================
    def update_frame(self):
        if not self.running:
            return

        # Khi đang ở tab Balance, bỏ qua toàn bộ xử lý camera, MediaPipe, resize và hiển thị
        # để mainloop ưu tiên đồ thị IMU. Bộ nhận camera cũng đã tạm dừng decode.
        if getattr(self, "balance_focus_mode", False) and bool(BALANCE_PAUSE_CAMERA):
            self.root.after(max(120, CAM_UPDATE_MS), self.update_frame)
            return

        frame = None

        if self.udp_cam is not None:
            frame = self.udp_cam.get_frame()

        if frame is None:
            blank = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

            cv2.putText(
                blank,
                "WAITING UDP CAMERA FROM RASPBERRY...",
                (40, HEIGHT // 2 - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                blank,
                f"Listen camera port: {CAMERA_UDP_PORT}",
                (40, HEIGHT // 2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                blank,
                f"Listen IMU port: {IMU_LISTEN_PORT}",
                (40, HEIGHT // 2 + 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            output = blank

        else:
            now = time.time()
            self.fps = 1.0 / max(now - self.prev_time, 0.0001)
            self.prev_time = now

            # Nếu chỉ giám sát camera, không chạy MediaPipe để tránh làm GUI giảm xuống khoảng 10 fps.
            run_mediapipe = (not bool(CAM_MEDIAPIPE_ONLY_WHEN_PID)) or bool(self.cam_pid_enabled)

            if run_mediapipe:
                self.cam_frame_count += 1
                n = max(1, int(CAM_PROCESS_EVERY_N))
                should_process = (self.cam_frame_count % n == 0) or (self.last_processed_output is None)

                if should_process:
                    output = self.process_frame(frame)
                    self.last_processed_output = output.copy()
                    self.send_camera_packet()
                else:
                    # Với frame trung gian, chỉ resize để hiển thị mượt và giữ trạng thái phát hiện gần nhất.
                    output = cv2.resize(frame, (WIDTH, HEIGHT))
                    cv2.line(output, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), (255, 0, 0), 2)
            else:
                # Khi PID OFF, chỉ xem camera thường, không chạy MediaPipe và không gửi offset.
                output = cv2.resize(frame, (WIDTH, HEIGHT))
                cv2.line(output, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), (255, 0, 0), 2)
                self.detected_val = 0
                self.full_body_ok = False
                self.offset_x = 0
                self.direction = "PID_OFF"
                self.box_width = 0
                self.box_height = 0
                self.person_center_x = 0
                self.person_center_y = 0
                self.last_processed_output = None
                self.reset_lpf_values()
                self.update_offset_record_snapshot()

        # Cập nhật trạng thái UDP CAM, UDP IMU và GUI FPS trong mọi trường hợp.
        # Bản cũ chỉ cập nhật khi có frame camera, nên nếu camera đang chờ
        # thì nhãn UDP IMU cũng bị kẹt ở trạng thái waiting dù tab Balance vẫn nhận IMU bình thường.
        self.update_info_labels()

        imgtk = self.make_tk_image_fit_label(output)

        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

        self.root.after(CAM_UPDATE_MS, self.update_frame)

    # ================== GHI REC IMU ROLL/PITCH/YAW ==================
    def choose_rec_folder(self):
        folder = filedialog.askdirectory(
            title="Chọn thư mục lưu CSV IMU + offset",
            initialdir=self.rec_folder if os.path.isdir(self.rec_folder) else REC_DEFAULT_DIR
        )
        if not folder:
            return
        self.rec_folder = folder
        state = "ON" if self.rec_enabled else "OFF"
        self.set_rec_status_text(f"REC IMU+OFFSET: {state} | Folder: {self.rec_folder}")

    def make_rec_filename(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.rec_folder, f"imu_rpy_offset_{ts}.csv")

    def start_imu_record(self):
        if self.rec_enabled:
            messagebox.showinfo("REC IMU", "Đang ghi dữ liệu rồi.")
            return

        try:
            os.makedirs(self.rec_folder, exist_ok=True)
            self.rec_file_path = self.make_rec_filename()
            self.rec_file = open(self.rec_file_path, "w", newline="", encoding="utf-8")
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow([
                "time_s",
                "unix_time",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",

                # Offset camera: raw, sau LPF và giá trị gửi đi.
                "cam_pid_enabled",
                "mediapipe_enabled",
                "full_body_ok",
                "detected",
                "direction",
                "offset_x_raw_px",
                "offset_x_lpf_px",
                "offset_x_send_px",
                "offset_box_width_scale",
                "offset_scale_ref_box_w_px",
                "offset_scale_enabled",
                "box_width_raw_px",
                "box_width_lpf_px",
                "box_height_raw_px",
                "box_height_lpf_px",
                "center_x_raw_px",
                "center_x_lpf_px",
                "center_y_raw_px",
                "center_y_lpf_px",
                "offset_sample_seq",
                "offset_sample_age_s",
            ])
            self.rec_start_time = time.time()
            self.rec_samples = 0
            self.rec_last_t_last = 0.0
            self.rec_last_seq = self.udp_imu.get_latest_seq() if self.udp_imu is not None else 0
            self.rec_enabled = True
            self.set_rec_status_text(f"REC IMU+OFFSET: ON | 0 mẫu | File: {self.rec_file_path}")
            self.last_status_msg = "Đang REC Roll/Pitch/Yaw + offset gửi bridge"
            self.update_status()
        except Exception as e:
            self.rec_enabled = False
            self.rec_writer = None
            try:
                if self.rec_file:
                    self.rec_file.close()
            except Exception:
                pass
            self.rec_file = None
            messagebox.showerror("Lỗi REC IMU", str(e))

    def stop_imu_record(self):
        if not self.rec_enabled and self.rec_file is None:
            self.set_rec_status_text(f"REC IMU+OFFSET: OFF | Folder: {self.rec_folder}")
            return

        self.rec_enabled = False
        try:
            if self.rec_file:
                self.rec_file.flush()
                self.rec_file.close()
        except Exception:
            pass

        saved_path = self.rec_file_path
        samples = self.rec_samples
        self.rec_file = None
        self.rec_writer = None

        self.set_rec_status_text(f"REC IMU+OFFSET: OFF | Đã lưu {samples} mẫu | File: {saved_path}")
        self.last_status_msg = f"Đã lưu REC IMU: {saved_path}"
        try:
            self.update_status()
        except Exception:
            pass

    def record_imu_samples_if_needed(self):
        """Ghi toàn bộ mẫu IMU mới nhận qua UDP 7007 kèm offset camera.

        Offset camera được ghi theo snapshot mới nhất:
          - offset_x_raw_px: offset đo trực tiếp từ MediaPipe.
          - offset_x_lpf_px: offset sau lọc LPF.
          - offset_x_send_px: offset đã giảm theo bề rộng box và đúng với giá trị gửi xuống bridge.
        """
        if not self.rec_enabled or self.rec_writer is None or self.udp_imu is None:
            return

        try:
            samples, latest_seq = self.udp_imu.get_samples_since(self.rec_last_seq)
            if not samples:
                return

            for seq, unix_t, roll, pitch, yaw in samples:
                if seq <= self.rec_last_seq:
                    continue
                t_s = unix_t - self.rec_start_time
                if t_s < 0:
                    t_s = 0.0
                off = self.get_offset_record_snapshot(ref_time=unix_t)

                self.rec_writer.writerow([
                    f"{t_s:.4f}",
                    f"{unix_t:.6f}",
                    f"{roll:.4f}",
                    f"{pitch:.4f}",
                    f"{yaw:.4f}",

                    int(off.get("cam_pid_enabled", 0)),
                    int(off.get("mediapipe_enabled", 0)),
                    int(off.get("full_body_ok", 0)),
                    int(off.get("detected", 0)),
                    str(off.get("direction", "")),
                    f"{float(off.get('offset_x_raw_px', 0.0)):.4f}",
                    f"{float(off.get('offset_x_lpf_px', 0.0)):.4f}",
                    f"{float(off.get('offset_x_send_px', 0.0)):.4f}",
                    f"{float(off.get('offset_box_width_scale', 1.0)):.4f}",
                    f"{float(off.get('offset_scale_ref_box_w_px', OFFSET_SCALE_REF_BOX_W)):.4f}",
                    int(off.get("offset_scale_enabled", OFFSET_SCALE_BY_BOX_W_ENABLE)),
                    f"{float(off.get('box_width_raw_px', 0.0)):.4f}",
                    f"{float(off.get('box_width_lpf_px', 0.0)):.4f}",
                    f"{float(off.get('box_height_raw_px', 0.0)):.4f}",
                    f"{float(off.get('box_height_lpf_px', 0.0)):.4f}",
                    f"{float(off.get('center_x_raw_px', 0.0)):.4f}",
                    f"{float(off.get('center_x_lpf_px', 0.0)):.4f}",
                    f"{float(off.get('center_y_raw_px', 0.0)):.4f}",
                    f"{float(off.get('center_y_lpf_px', 0.0)):.4f}",
                    int(off.get("seq", 0)),
                    f"{float(off.get('age_s', 999.0)):.4f}",
                ])
                self.rec_samples += 1
                self.rec_last_seq = seq

            if latest_seq > self.rec_last_seq:
                self.rec_last_seq = latest_seq

            if self.rec_samples % 20 == 0:
                self.rec_file.flush()
            self.set_rec_status_text(f"REC IMU+OFFSET: ON | {self.rec_samples} mẫu | File: {self.rec_file_path}")
        except Exception as e:
            print("[REC IMU/OFFSET ERR]", e)
            self.stop_imu_record()

    def choose_csv_and_plot(self):
        file_path = filedialog.askopenfilename(
            title="Chọn file CSV IMU để plot",
            initialdir=self.rec_folder if os.path.isdir(self.rec_folder) else REC_DEFAULT_DIR,
            filetypes=[("CSV files", "*.csv"), ("All files", "*")]
        )
        if not file_path:
            return
        self.plot_imu_csv_file(file_path)

    def _read_imu_csv_for_plot(self, file_path):
        times, rolls, pitches, yaws = [], [], [], []
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.DictReader(f)
            for idx, row in enumerate(reader):
                try:
                    # Hỗ trợ header mới như time_s, roll_deg... và một số tên cũ nếu có.
                    t_raw = row.get("time_s", row.get("time", row.get("t", idx)))
                    r_raw = row.get("roll_deg", row.get("roll", row.get("Roll", 0)))
                    p_raw = row.get("pitch_deg", row.get("pitch", row.get("Pitch", 0)))
                    y_raw = row.get("yaw_deg", row.get("yaw", row.get("Yaw", 0)))
                    times.append(float(t_raw))
                    rolls.append(float(r_raw))
                    pitches.append(float(p_raw))
                    yaws.append(float(y_raw))
                except Exception:
                    continue
        return times, rolls, pitches, yaws

    def calc_auto_ylim_for_plot(self, values, min_span=2.0, margin_ratio=0.15):
        vals = []
        for v in values:
            try:
                fv = float(v)
                if math.isfinite(fv):
                    vals.append(fv)
            except Exception:
                pass

        if not vals:
            return (-10.0, 10.0)

        ymin = min(vals)
        ymax = max(vals)

        if abs(ymax - ymin) < min_span:
            mid = 0.5 * (ymin + ymax)
            ymin = mid - min_span * 0.5
            ymax = mid + min_span * 0.5

        span = ymax - ymin
        margin = max(min_span * 0.25, span * margin_ratio)
        return (ymin - margin, ymax + margin)

    def plot_imu_csv_file(self, file_path):
        try:
            times, rolls, pitches, yaws = self._read_imu_csv_for_plot(file_path)
            if len(times) < 2:
                messagebox.showwarning("Plot CSV", "File CSV không đủ dữ liệu để plot.")
                return

            win = tk.Toplevel(self.root)
            win.title(f"Plot IMU CSV - {os.path.basename(file_path)}")
            win.geometry("1200x800")
            win.configure(bg="#f8fafc")

            top = ttk.Frame(win, style="TFrame", padding=10)
            top.pack(fill=tk.X)
            ttk.Label(
                top,
                text=f"File: {file_path} | Samples: {len(times)}",
                style="Info.TLabel"
            ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Button(top, text="ĐÓNG", style="Red.TButton", command=win.destroy).pack(side=tk.RIGHT)

            fig = Figure(figsize=(12, 7), dpi=100, facecolor="#ffffff")
            ax1 = fig.add_subplot(311)
            ax2 = fig.add_subplot(312)
            ax3 = fig.add_subplot(313)

            ax1.plot(times, rolls, linewidth=1.8, label="Roll", color="tab:blue")
            ax2.plot(times, pitches, linewidth=1.8, label="Pitch", color="tab:orange")
            ax3.plot(times, yaws, linewidth=1.8, label="Yaw", color="tab:green")

            roll_ylim = self.calc_auto_ylim_for_plot(rolls)
            pitch_ylim = self.calc_auto_ylim_for_plot(pitches)
            yaw_ylim = self.calc_auto_ylim_for_plot(yaws, min_span=5.0, margin_ratio=0.15)

            for ax, title, ylim in [
                (ax1, "ROLL (deg)", roll_ylim),
                (ax2, "PITCH (deg)", pitch_ylim),
                (ax3, "YAW (deg)", yaw_ylim),
            ]:
                ax.set_title(title, fontsize=13, fontweight="bold")
                ax.set_ylabel("deg")
                ax.set_ylim(*ylim)
                ax.grid(True, color="#cbd5e1", linewidth=0.8)
                ax.legend(loc="upper right")

            ax3.set_xlabel("Time (s)")
            fig.tight_layout(pad=1.4)

            canvas = FigureCanvasTkAgg(fig, master=win)
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
            canvas.draw()

        except Exception as e:
            messagebox.showerror("Lỗi plot CSV", str(e))

    # ================== CẬP NHẬT GIÁ TRỊ IMU VÀ REC ==================
    def update_imu_plot(self):
        # Giữ tên hàm cũ để không phải thay đổi nhiều vị trí gọi.
        # Tab Giám sát đã bỏ đồ thị IMU; hàm này chỉ cập nhật số Roll/Pitch/Yaw và ghi CSV khi REC đang bật.
        if not self.running:
            return

        if self.udp_imu is not None:
            roll, pitch, yaw, t_last = self.udp_imu.get_imu()

            if t_last > 0:
                self.record_imu_samples_if_needed()

                self.imu_value_label.config(
                    text=f"Roll: {roll:7.2f}°    Pitch: {pitch:7.2f}°    Yaw: {yaw:7.2f}°"
                )

        self.root.after(PLOT_UPDATE_MS, self.update_imu_plot)

    # ================== ĐÓNG CHƯƠNG TRÌNH ==================
    def on_close(self):
        self.running = False

        try:
            self.stop_imu_record()
        except Exception:
            pass

        try:
            if self.udp_cam:
                self.udp_cam.stop()
        except Exception:
            pass

        try:
            if self.udp_imu:
                self.udp_imu.stop()
        except Exception:
            pass

        try:
            if self.offset_sock:
                self.offset_sock.close()
        except Exception:
            pass

        try:
            if hasattr(self, "gazebo_imu_panel") and self.gazebo_imu_panel:
                self.gazebo_imu_panel.close()
        except Exception:
            pass

        try:
            if hasattr(self, "joint_mapper_panel") and self.joint_mapper_panel:
                self.joint_mapper_panel.close()
        except Exception:
            pass

        try:
            if self.pose:
                self.pose.close()
        except Exception:
            pass

        try:
            if ROS_OK and rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CamPidApp(root)
    root.mainloop()