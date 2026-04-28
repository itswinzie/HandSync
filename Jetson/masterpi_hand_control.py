#!/usr/bin/env python3
# ============================================================
# MasterPi Hand Gesture Controller — LOCK MODE + SMOOTH
#
# CARA GUNA:
#   1. Tunjuk gesture dan TAHAN sehingga bar penuh
#   2. LOCKED! — servo ikut posisi tangan
#   3. Fist = Unlock + Gripper tutup
#
# Gesture Kanan:
#   👍 Thumb Up  → LOCK → Shoulder (atas/bawah)
#   🤘 Rock Sign → LOCK → Elbow    (atas/bawah)
#   🤙 Shaka     → LOCK → Wrist    (atas/bawah)
#   ☝️  Point     → LOCK → Base     (kiri/kanan)
#   🖐️  Open Palm → Gripper BUKA   (terus)
#   👊 Fist      → Gripper TUTUP + UNLOCK
#
# Gesture Kiri:
#   ☝️  Point → Mecanum ikut arah jari
# ============================================================

import cv2
import json
import math
import time
import threading
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from collections import deque

# ── ROS2 ─────────────────────────────────────────────────────
# Global untuk simpan frame camera MasterPi
masterpi_frame = [None]
masterpi_frame_lock = threading.Lock()

class MasterPiController(Node):
    def __init__(self):
        super().__init__('masterpi_hand_controller')
        self.publisher_ = self.create_publisher(String, 'masterpi_command', 10)

        # Subscribe camera dari MasterPi
        self.cam_sub = self.create_subscription(
            CompressedImage,
            'masterpi/camera/compressed',
            self.camera_callback,
            10
        )
        self.get_logger().info('Subscribed to /masterpi/camera/compressed')

    def camera_callback(self, msg):
        import numpy as np
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None:
            with masterpi_frame_lock:
                masterpi_frame[0] = frame

    def send(self, command: dict):
        msg = String()
        msg.data = json.dumps(command)
        self.publisher_.publish(msg)

# ── MediaPipe ─────────────────────────────────────────────────
mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.7
)

# ── Config ────────────────────────────────────────────────────
SERVO_MIN    = 700
SERVO_MAX    = 2300
SERVO_MID    = 1500
GRIPPER_OPEN = 2000
GRIPPER_SHUT = 700
MOTOR_SPEED  = 55
LOCK_TIME    = 1.0   # Saat untuk lock

servo_state = {
    "gripper":  SERVO_MID,
    "wrist":    SERVO_MID,
    "elbow":    SERVO_MID,
    "shoulder": SERVO_MID,
    "base":     SERVO_MID,
}

# ── Smoothing ─────────────────────────────────────────────────
SMOOTH_SIZE     = 6
smooth_shoulder = deque(maxlen=SMOOTH_SIZE)
smooth_elbow    = deque(maxlen=SMOOTH_SIZE)
smooth_base     = deque(maxlen=SMOOTH_SIZE)
smooth_wrist    = deque(maxlen=SMOOTH_SIZE)

def smooth_avg(buf, val):
    buf.append(val)
    return int(sum(buf) / len(buf))

# ── Lock State ────────────────────────────────────────────────
class GestureLock:
    def __init__(self):
        self.locked_gesture  = None
        self.candidate       = None
        self.candidate_start = None
        self.is_locked       = False

    def update(self, gesture):
        now = time.time()

        # Instant gesture — tak perlu lock
        if gesture in {"open_palm", "fist", "unknown", None}:
            self.candidate       = None
            self.candidate_start = None
            if gesture == "fist":
                self.is_locked      = False
                self.locked_gesture = None
            return self.is_locked, self.locked_gesture, 0.0

        # Dah locked dengan gesture sama — kekal
        if self.is_locked and gesture == self.locked_gesture:
            return True, self.locked_gesture, 1.0

        # Tukar gesture semasa locked — unlock dulu
        if self.is_locked and gesture != self.locked_gesture:
            self.is_locked      = False
            self.locked_gesture = None
            self.candidate       = gesture
            self.candidate_start = now
            return False, None, 0.0

        # Mula hold gesture baru
        if gesture != self.candidate:
            self.candidate       = gesture
            self.candidate_start = now

        elapsed  = now - self.candidate_start
        progress = min(elapsed / LOCK_TIME, 1.0)

        if elapsed >= LOCK_TIME:
            self.is_locked      = True
            self.locked_gesture = gesture
            return True, gesture, 1.0

        return False, None, progress

    def reset(self):
        self.locked_gesture  = None
        self.candidate       = None
        self.candidate_start = None
        self.is_locked       = False

lock_right = GestureLock()

# ── Landmark helpers ──────────────────────────────────────────
TIP = [4, 8, 12, 16, 20]
PIP = [3, 6, 10, 14, 18]
MCP = [2, 5,  9, 13, 17]

def lm(hand, idx):
    return hand.landmark[idx].x, hand.landmark[idx].y

def clamp(v):
    return max(SERVO_MIN, min(SERVO_MAX, int(v)))

def map_val(val, in_min, in_max, out_min, out_max):
    val = max(in_min, min(in_max, float(val)))
    ratio = (val - in_min) / (in_max - in_min)
    return clamp(out_min + ratio * (out_max - out_min))

# ── Detect jari ───────────────────────────────────────────────
def fingers_up(hand):
    up = []
    _, tip_y = lm(hand, TIP[0])
    _, ip_y  = lm(hand, PIP[0])
    up.append(tip_y < ip_y)
    for i in range(1, 5):
        up.append(hand.landmark[TIP[i]].y < hand.landmark[PIP[i]].y)
    return up

# ── Detect Gesture ────────────────────────────────────────────
def detect_gesture(hand):
    t, i, m, r, p = fingers_up(hand)
    if t and i and m and r and p:                 return "open_palm"
    if not any([t, i, m, r, p]):                  return "fist"
    if t and not i and not m and not r and not p: return "thumb_up"
    if i and not m and not r and p:               return "rock"    # thumb tak kisah
    if t and not i and not m and not r and p:     return "shaka"
    if not t and i and not m and not r and not p: return "point"
    return "unknown"

def point_direction(hand):
    mx, my = lm(hand, MCP[1])
    tx, ty = lm(hand, TIP[1])
    angle = math.degrees(math.atan2(ty - my, tx - mx))
    if -135 <= angle < -45:          return "up"
    if   45 <= angle <  135:         return "down"
    if angle < -135 or angle >= 135: return "left"
    return "right"

# ── Tilt Joystick ────────────────────────────────────────────
# Detect condong tangan menggunakan vektor wrist → middle finger
# Tangan condong depan  = maju
# Tangan condong belakang = undur
# Tangan condong kiri   = slide kiri
# Tangan condong kanan  = slide kanan
# Tangan flat (tegak)   = STOP

TILT_THRESHOLD = 0.08   # Minimum tilt untuk detect (elak noise)
current_tilt   = [0.0, 0.0]   # [tilt_x, tilt_y] untuk UI

def get_tilt(hand):
    """
    Kira tilt tangan guna vektor wrist(0) → middle_mcp(9)
    Return (tilt_forward, tilt_right) dalam range -1.0 ~ 1.0
    tilt_forward: positif=condong depan(maju), negatif=condong belakang(undur)
    tilt_right:   positif=condong kanan, negatif=condong kiri
    """
    # Wrist landmark
    wx, wy = lm(hand, 0)
    # Middle finger MCP (landmark 9) — pangkal jari tengah
    mx, my = lm(hand, 9)

    # Vektor dari wrist ke middle MCP
    # dx: positif = tangan condong kanan
    # dy: positif = tangan condong ke bawah skrin (undur)
    dx = mx - wx   # horizontal tilt
    dy = my - wy   # vertical tilt (y besar = bawah skrin)

    # dy negatif = tangan condong ke atas skrin = maju
    tilt_forward = -dy * 3.0   # flip: atas=maju
    tilt_right   =  dx * 3.0

    # Clamp ke -1.0 ~ 1.0
    tilt_forward = max(-1.0, min(1.0, tilt_forward))
    tilt_right   = max(-1.0, min(1.0, tilt_right))

    return tilt_forward, tilt_right

def process_left(hand):
    global current_tilt
    gesture = detect_gesture(hand)

    # Open Palm = STOP paksa
    if gesture == "open_palm":
        current_tilt = [0.0, 0.0]
        return {"type":"wheels","linear":0,"strafe":0,"angular":0}, gesture, "stop"

    tilt_f, tilt_r = get_tilt(hand)
    current_tilt[0] = tilt_r
    current_tilt[1] = tilt_f

    # Apply threshold — elak noise bila tangan flat
    if abs(tilt_f) < TILT_THRESHOLD: tilt_f = 0.0
    if abs(tilt_r) < TILT_THRESHOLD: tilt_r = 0.0

    if tilt_f == 0.0 and tilt_r == 0.0:
        return {"type":"wheels","linear":0,"strafe":0,"angular":0}, gesture, "stop"

    linear = int(tilt_f * MOTOR_SPEED)
    strafe = int(tilt_r * MOTOR_SPEED)

    linear = max(-MOTOR_SPEED, min(MOTOR_SPEED, linear))
    strafe = max(-MOTOR_SPEED, min(MOTOR_SPEED, strafe))

    # Direction untuk display
    if abs(tilt_f) > abs(tilt_r):
        direction = "maju" if linear > 0 else "undur"
    elif abs(tilt_r) > abs(tilt_f):
        direction = "kanan" if strafe > 0 else "kiri"
    else:
        # Diagonal
        fb = "maju" if linear > 0 else "undur"
        lr = "kanan" if strafe > 0 else "kiri"
        direction = f"{lr}-{fb}"

    return {"type":"wheels","linear":linear,"strafe":strafe,"angular":0}, gesture, direction

# ── Process Kanan — Arm (LOCK MODE) ──────────────────────────
def process_right(hand):
    global servo_state
    raw_gesture             = detect_gesture(hand)
    is_locked, locked, prog = lock_right.update(raw_gesture)
    active = None

    # Instant gestures — terus tanpa lock
    if raw_gesture == "open_palm":
        servo_state["gripper"] = GRIPPER_OPEN
        active = "gripper"
    elif raw_gesture == "fist":
        servo_state["gripper"] = GRIPPER_SHUT
        active = "gripper"

    # Servo control — hanya bila locked
    elif is_locked:
        if locked == "thumb_up":
            _, tip_y = lm(hand, TIP[0])
            pulse = map_val(tip_y, 0.05, 0.85, SERVO_MIN, SERVO_MAX)
            servo_state["shoulder"] = smooth_avg(smooth_shoulder, pulse)
            active = "shoulder"

        elif locked == "rock":
            _, iy = lm(hand, TIP[1])
            _, py = lm(hand, TIP[4])
            avg_y = (iy + py) / 2.0
            pulse = map_val(avg_y, 0.05, 0.85, SERVO_MIN, SERVO_MAX)
            servo_state["elbow"] = smooth_avg(smooth_elbow, pulse)
            active = "elbow"

        elif locked == "shaka":
            _, tip_y = lm(hand, TIP[4])
            pulse = map_val(tip_y, 0.05, 0.85, SERVO_MAX, SERVO_MIN)
            servo_state["wrist"] = smooth_avg(smooth_wrist, pulse)
            active = "wrist"

        elif locked == "point":
            tx, _ = lm(hand, TIP[1])
            pulse = map_val(tx, 0.05, 0.95, SERVO_MIN, SERVO_MAX)
            servo_state["base"] = smooth_avg(smooth_base, pulse)
            active = "base"

    return {
        "type":           "arm",
        "servo_gripper":  servo_state["gripper"],
        "servo_wrist":    servo_state["wrist"],
        "servo_elbow":    servo_state["elbow"],
        "servo_shoulder": servo_state["shoulder"],
        "servo_base":     servo_state["base"],
        "active_servo":   active,
    }, raw_gesture, is_locked, prog

# ── Draw UI ───────────────────────────────────────────────────
GESTURE_DESC = {
    "open_palm": "Open Palm — Gripper Buka",
    "fist":      "Fist — Tutup+Unlock",
    "thumb_up":  "Thumb Up — Shoulder",
    "rock":      "Rock — Elbow",
    "shaka":     "Shaka — Wrist",
    "point":     "Point — Base",
    "unknown":   "...",
}

def draw_panel_left(panel, left_data):
    """Draw tilt info untuk tangan kiri"""
    h, w = panel.shape[:2]
    F = cv2.FONT_HERSHEY_SIMPLEX

    # Header
    cv2.rectangle(panel, (0,0), (w,28), (0,60,0), -1)
    cv2.putText(panel, "TANGAN KIRI — Tilt Control",
                (8,20), F, 0.45, (100,255,100), 1)

    cx2 = w//2
    tx = current_tilt[0]
    ty = current_tilt[1]

    # Forward/backward bar
    cv2.putText(panel, "Maju/Undur:", (8,50), F, 0.38, (150,150,150), 1)
    bar_w = int(abs(ty) * (w//2 - 20))
    bar_col = (50,255,50) if ty > 0 else (50,150,255)
    if ty > 0:
        cv2.rectangle(panel, (cx2,38), (cx2+bar_w,52), bar_col, -1)
    else:
        cv2.rectangle(panel, (cx2-bar_w,38), (cx2,52), bar_col, -1)
    cv2.line(panel, (cx2,35), (cx2,55), (100,100,100), 1)

    # Left/right bar
    cv2.putText(panel, "Kiri/Kanan:", (8,75), F, 0.38, (150,150,150), 1)
    bar_w2 = int(abs(tx) * (w//2 - 20))
    bar_col2 = (255,200,50) if tx > 0 else (200,100,255)
    if tx > 0:
        cv2.rectangle(panel, (cx2,63), (cx2+bar_w2,77), bar_col2, -1)
    else:
        cv2.rectangle(panel, (cx2-bar_w2,63), (cx2,77), bar_col2, -1)
    cv2.line(panel, (cx2,60), (cx2,80), (100,100,100), 1)

    # Status
    if left_data:
        gesture, direction = left_data
        moving = direction != "stop"
        color  = (50,255,50) if moving else (150,150,150)
        status = direction.upper().replace("-"," ") if moving else "STOP"
        cv2.putText(panel, status, (w//2-35, 115), F, 0.65, color, 2)
        cv2.putText(panel, f"F:{ty:+.2f}  R:{tx:+.2f}",
                    (8,140), F, 0.38, (100,100,100), 1)
        if gesture == "open_palm":
            cv2.putText(panel, "PALM = STOP paksa",
                        (8,160), F, 0.4, (200,200,50), 1)
    else:
        cv2.putText(panel, "Tiada tangan kiri",
                    (w//2-60, 115), F, 0.45, (80,80,80), 1)

    cv2.putText(panel, "Condong tangan = kawalan arah",
                (8,180), F, 0.33, (80,80,80), 1)

def draw_panel_right(panel, right_data):
    """Draw info tangan kanan pada panel kanan"""
    h, w = panel.shape[:2]
    F = cv2.FONT_HERSHEY_SIMPLEX

    # Header
    cv2.rectangle(panel, (0,0), (w,28), (0,0,80), -1)
    cv2.putText(panel, "TANGAN KANAN — Arm",
                (8,20), F, 0.5, (100,150,255), 1)

    if right_data:
        gesture, is_locked, progress, locked_gesture = right_data
        lock_color = (0,220,0) if is_locked else (0,160,255)

        # Gesture semasa
        cv2.putText(panel, f"Gesture: {GESTURE_DESC.get(gesture,gesture)}",
                    (10,50), F, 0.42, (200,200,200), 1)

        # Lock status
        if is_locked:
            cv2.rectangle(panel, (8,58), (w-8,82), (0,60,0), -1)
            cv2.putText(panel, f"LOCKED: {locked_gesture.upper()}",
                        (12,75), F, 0.45, (0,255,0), 1)
            cv2.putText(panel, "Gerak tangan untuk kawal servo",
                        (10,98), F, 0.36, (150,255,150), 1)
            cv2.putText(panel, "Fist = Unlock",
                        (10,115), F, 0.36, (180,180,180), 1)
        else:
            msg = "Tahan gesture..." if progress > 0 else "Tahan 1 saat untuk LOCK"
            cv2.putText(panel, msg, (10,75), F, 0.4, lock_color, 1)

        # Progress bar
        bar_w = int(progress * (w-20))
        cv2.rectangle(panel, (10,125), (w-10,140), (40,40,40), -1)
        cv2.rectangle(panel, (10,125), (10+bar_w,140), lock_color, -1)
        cv2.putText(panel, f"Lock: {int(progress*100)}%",
                    (10,155), F, 0.38, lock_color, 1)

        # Divider
        cv2.line(panel, (10,165), (w-10,165), (60,60,60), 1)

        # Servo state
        cv2.putText(panel, "Servo State:",
                    (10,182), F, 0.4, (100,150,255), 1)
        entries = [
            ("Gripper (ID1)",  servo_state["gripper"],  (100,200,255)),
            ("Wrist   (ID3)",  servo_state["wrist"],    (150,180,255)),
            ("Elbow   (ID4)",  servo_state["elbow"],    (100,255,200)),
            ("Shoulder(ID5)",  servo_state["shoulder"], (100,255,150)),
            ("Base    (ID6)",  servo_state["base"],     (200,255,100)),
        ]
        for i, (name, val, col) in enumerate(entries):
            ey = 200 + i*28
            # Highlight servo yang locked
            is_active = (is_locked and locked_gesture and name.lower().startswith(locked_gesture[:3]))
            txt_col = col if is_active else (150,150,150)
            cv2.putText(panel, f"{name}: {val}",
                        (10,ey), F, 0.38, txt_col, 1)
            bar = int(((val-500)/2000.0)*(w-30))
            cv2.rectangle(panel, (10,ey+2), (w-10,ey+10), (40,40,40), -1)
            cv2.rectangle(panel, (10,ey+2), (10+bar,ey+10), col if is_active else (70,70,70), -1)
    else:
        cv2.putText(panel, "Tiada tangan dikesan",
                    (10, h//2), F, 0.45, (80,80,80), 1)

def draw_tilt_overlay(frame):
    """Overlay tilt indicator pada camera frame kiri"""
    h, w = frame.shape[:2]
    F = cv2.FONT_HERSHEY_SIMPLEX

    # Lukis crosshair di tengah
    cx, cy = w//2, h//2
    R = 60   # radius crosshair

    # Background bulatan
    cv2.circle(frame, (cx,cy), R+5, (0,0,0), 2)
    cv2.circle(frame, (cx,cy), R+5, (60,60,60), 1)

    # Cross lines
    cv2.line(frame, (cx-R,cy), (cx+R,cy), (70,70,70), 1)
    cv2.line(frame, (cx,cy-R), (cx,cy+R), (70,70,70), 1)

    # Label arah
    cv2.putText(frame, "MAJU",  (cx-18, cy-R-5), F, 0.4, (100,100,100), 1)
    cv2.putText(frame, "UNDUR", (cx-22, cy+R+14),F, 0.4, (100,100,100), 1)
    cv2.putText(frame, "KIRI",  (cx-R-38,cy+4),  F, 0.4, (100,100,100), 1)
    cv2.putText(frame, "KANAN", (cx+R+5, cy+4),  F, 0.4, (100,100,100), 1)

    # Tilt dot — ikut current_tilt
    tx = current_tilt[0]   # kiri/kanan
    ty = current_tilt[1]   # maju/undur (positif=maju)
    dot_x = int(cx + tx * R)
    dot_y = int(cy - ty * R)   # flip Y: maju = atas skrin
    dot_x = max(cx-R, min(cx+R, dot_x))
    dot_y = max(cy-R, min(cy+R, dot_y))

    moving = abs(tx) > TILT_THRESHOLD or abs(ty) > TILT_THRESHOLD
    dot_col = (50,255,50) if moving else (150,150,150)

    # Line dari tengah ke dot
    cv2.line(frame, (cx,cy), (dot_x,dot_y), dot_col, 2)
    cv2.circle(frame, (dot_x,dot_y), 10, dot_col, -1)
    cv2.circle(frame, (cx,cy), 4, (200,200,200), -1)

    # Direction label bawah
    if left_data_global[0]:
        _, direction = left_data_global[0]
        moving2 = direction != "stop"
        color   = (50,255,50) if moving2 else (150,150,150)
        label   = direction.upper().replace("-"," ") if moving2 else "STOP"
        cv2.putText(frame, label, (cx-30, h-15), F, 0.55, color, 2)

    # Instruction
    cv2.putText(frame, "Condong tangan untuk gerak",
                (5, h-35), F, 0.32, (100,100,100), 1)
    cv2.putText(frame, "Palm terbuka = STOP",
                (5, h-20), F, 0.32, (120,120,80), 1)

    return frame

# Global untuk simpan left_data supaya boleh diakses dalam draw_tilt_overlay
left_data_global = [None]

def draw_ui(frame, left_data, right_data):
    """Split view — kiri untuk tangan kiri, kanan untuk tangan kanan"""
    import numpy as np
    h, fw = frame.shape[:2]
    half  = fw // 2

    # Update global left_data
    left_data_global[0] = left_data

    # Split frame
    left_frame  = frame[:, :half].copy()
    right_frame = frame[:, half:].copy()

    # Overlay grid pada camera kiri
    left_frame = draw_tilt_overlay(left_frame)

    # Buat panel info di bawah setiap frame
    panel_h = 280
    left_panel  = np.zeros((panel_h, half, 3), dtype="uint8")
    right_panel = np.zeros((panel_h, half, 3), dtype="uint8")

    draw_panel_left(left_panel, left_data)
    draw_panel_right(right_panel, right_data)

    # Stack: camera atas + panel bawah
    left_combined  = np.vstack([left_frame,  left_panel])
    right_combined = np.vstack([right_frame, right_panel])

    # Gabung kiri + kanan
    combined = np.hstack([left_combined, right_combined])

    # Divider tengah
    cv2.line(combined, (half,0), (half,combined.shape[0]), (100,100,100), 2)

    # Header atas
    cv2.rectangle(combined, (0,0), (combined.shape[1],22), (20,20,20), -1)
    cv2.putText(combined, "MasterPi Lock Mode  |  ESC=Keluar",
                (combined.shape[1]//2-130, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180,180,180), 1)

    # Tambah camera MasterPi di bahagian bawah
    with masterpi_frame_lock:
        mp_cam = masterpi_frame[0]

    if mp_cam is not None:
        # Resize camera MasterPi supaya lebar sama dengan combined
        cam_w = combined.shape[1]
        cam_h = int(mp_cam.shape[0] * cam_w / mp_cam.shape[1])
        cam_h = min(cam_h, 200)   # Max height 200px
        cam_w_actual = int(mp_cam.shape[1] * cam_h / mp_cam.shape[0])
        mp_resized = cv2.resize(mp_cam, (cam_w_actual, cam_h))

        # Buat bar camera
        cam_bar = np.zeros((cam_h + 24, combined.shape[1], 3), dtype='uint8')
        cv2.rectangle(cam_bar, (0,0), (combined.shape[1],22), (30,30,80), -1)
        cv2.putText(cam_bar, "Camera MasterPi (Robot View)",
                    (10,16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150,180,255), 1)

        # Letak camera di tengah bar
        x_offset = (combined.shape[1] - cam_w_actual) // 2
        cam_bar[24:24+cam_h, x_offset:x_offset+cam_w_actual] = mp_resized

        # Stack bawah combined
        combined = np.vstack([combined, cam_bar])
    else:
        # Placeholder kalau camera tak connect
        no_cam = np.zeros((50, combined.shape[1], 3), dtype='uint8')
        cv2.rectangle(no_cam, (0,0), (combined.shape[1],50), (30,30,50), -1)
        cv2.putText(no_cam, "Camera MasterPi — Menunggu stream... (pastikan masterpi_camera_pub.py running)",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100,100,150), 1)
        combined = np.vstack([combined, no_cam])

    # Resize untuk display
    display = cv2.resize(combined, (1280, 900))
    return display

# ── Main ──────────────────────────────────────────────────────
def main():
    rclpy.init()
    controller = MasterPiController()
    threading.Thread(target=rclpy.spin, args=(controller,), daemon=True).start()

    cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR: Tiada kamera!")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    print("Camera berjaya dibuka!")

    latest_frame = [None]
    frame_lock   = threading.Lock()

    def camera_thread():
        while cap.isOpened():
            ok, f = cap.read()
            if ok:
                with frame_lock:
                    latest_frame[0] = f

    threading.Thread(target=camera_thread, daemon=True).start()

    print("=" * 55)
    print("  LOCK MODE:")
    print("  Tahan gesture 1 saat → LOCKED")
    print("  Gerak tangan → servo ikut")
    print("  Fist = Unlock + Gripper tutup")
    print("  Open Palm = Gripper buka (terus)")
    print("=" * 55)

    time.sleep(0.5)

    while True:
        with frame_lock:
            frame = latest_frame[0]
        if frame is None:
            continue

        frame   = frame.copy()
        frame   = cv2.flip(frame, 1)
        results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        left_data  = None
        right_data = None

        fw = frame.shape[1]
        half = fw // 2

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_lm, handedness in zip(
                results.multi_hand_landmarks, results.multi_handedness
            ):
                # Guna posisi wrist untuk tentukan tangan ada di section mana
                wrist_x = hand_lm.landmark[0].x  # 0.0=kiri, 1.0=kanan

                mp_drawing.draw_landmarks(
                    frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                    mp_styles.get_default_hand_landmarks_style(),
                    mp_styles.get_default_hand_connections_style()
                )

                # Tangan dalam section kiri (x < 0.5) = kawalan roda
                # Tangan dalam section kanan (x >= 0.5) = kawalan arm
                if wrist_x < 0.5:
                    cmd, gesture, direction = process_left(hand_lm)
                    controller.send(cmd)
                    left_data = (gesture, direction)
                else:
                    cmd, gesture, is_locked, progress = process_right(hand_lm)
                    controller.send(cmd)
                    right_data = (gesture, is_locked, progress, lock_right.locked_gesture)
        else:
            lock_right.reset()
            # Kalau tiada tangan kiri — stop motor
            controller.send({"type":"wheels","linear":0,"strafe":0,"angular":0})

        display = draw_ui(frame, left_data, right_data)
        cv2.imshow("MasterPi Lock Mode", display)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
