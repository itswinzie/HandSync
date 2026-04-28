#!/usr/bin/env python3
# ============================================================
# Gesture Debug Tool
# Tunjuk nilai landmark setiap jari secara realtime
# Guna ini untuk fine-tune gesture detection
# ============================================================

import cv2
import math
import mediapipe as mp
import numpy as np

mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.6
)

TIP = [4, 8, 12, 16, 20]
PIP = [3, 6, 10, 14, 18]
MCP = [2, 5,  9, 13, 17]
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

def get_finger_data(hand):
    """Kira nilai setiap jari"""
    data = []
    for i in range(5):
        tip = hand.landmark[TIP[i]]
        pip = hand.landmark[PIP[i]]
        mcp = hand.landmark[MCP[i]]

        if i == 0:
            # Ibu jari — guna x
            extended = tip.x < mcp.x
            diff = round(mcp.x - tip.x, 3)
        else:
            # 4 jari — guna y (kecil = atas = naik)
            extended = tip.y < pip.y
            diff = round(pip.y - tip.y, 3)

        data.append({
            "name": FINGER_NAMES[i],
            "extended": extended,
            "diff": diff,
            "tip_y": round(tip.y, 3),
            "pip_y": round(pip.y, 3),
        })
    return data

def detect_gesture(data):
    """
    Detect gesture berdasarkan pattern jari
    """
    up = [d["extended"] for d in data]
    t, i, m, r, p = up

    # Kira berapa jari naik
    count = sum(up)

    if t and i and m and r and p:                 return "open_palm ✋"
    if not t and not i and not m and not r and not p: return "fist ✊"
    if t and not i and not m and not r and not p: return "thumb_up 👍"
    if not t and i and m and not r and not p:     return "peace ✌️"
    if not t and i and not m and not r and not p: return "point ☝️"
    if not t and not i and not m and not r and p: return "pinky 🤙"
    return f"unknown ({count} fingers up)"

# Warna untuk setiap status
COLOR_UP   = (50, 255, 100)   # Hijau = jari naik
COLOR_DOWN = (50, 100, 255)   # Biru = jari turun
COLOR_INFO = (200, 200, 200)

cap = None
for idx in [0, 1, 2]:
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        print(f"Camera: index {idx}")
        break

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("=" * 50)
print("  GESTURE DEBUG TOOL")
print("  Tunjuk tangan di depan kamera")
print("  Tengok nilai jari di sebelah kiri")
print("  ESC untuk keluar")
print("=" * 50)

while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        continue

    frame   = cv2.flip(frame, 1)
    h, w    = frame.shape[:2]
    rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    # Panel kiri
    cv2.rectangle(frame, (0, 0), (320, h), (20, 20, 20), -1)
    cv2.putText(frame, "GESTURE DEBUG",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.putText(frame, "ESC = Keluar",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150,150,150), 1)

    if results.multi_hand_landmarks:
        for hand_lm in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                mp_styles.get_default_hand_landmarks_style(),
                mp_styles.get_default_hand_connections_style()
            )

            finger_data = get_finger_data(hand_lm)
            gesture     = detect_gesture(finger_data)

            # Tunjuk gesture
            cv2.putText(frame, "Gesture:", (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150,150,150), 1)
            cv2.putText(frame, gesture, (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)

            # Divider
            cv2.line(frame, (10, 125), (310, 125), (60,60,60), 1)
            cv2.putText(frame, "Finger Status:", (10, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 1)

            # Tunjuk setiap jari
            for i, d in enumerate(finger_data):
                y = 170 + i * 75
                color = COLOR_UP if d["extended"] else COLOR_DOWN
                status = "UP ↑" if d["extended"] else "DOWN ↓"

                # Nama jari
                cv2.putText(frame, f"{d['name']}: {status}",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

                # Nilai diff
                bar_val = min(abs(d["diff"]) * 500, 200)
                bar_col = COLOR_UP if d["diff"] > 0 else COLOR_DOWN
                cv2.rectangle(frame, (10, y+8), (int(10+bar_val), y+20), bar_col, -1)
                cv2.putText(frame, f"diff={d['diff']:+.3f}",
                            (10, y+35), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180,180,180), 1)
                cv2.putText(frame, f"tip_y={d['tip_y']:.3f} pip_y={d['pip_y']:.3f}",
                            (10, y+52), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130,130,130), 1)

            # Divider bawah
            cv2.line(frame, (10, h-80), (310, h-80), (60,60,60), 1)

            # Pattern summary
            up_pattern = [d["extended"] for d in finger_data]
            pattern_str = " ".join(["↑" if u else "↓" for u in up_pattern])
            labels = "T  I  M  R  P"
            cv2.putText(frame, labels,
                        (10, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 1)
            cv2.putText(frame, pattern_str,
                        (10, h-35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)
            cv2.putText(frame, "(T=Thumb I=Index M=Middle R=Ring P=Pinky)",
                        (10, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100,100,100), 1)

    else:
        cv2.putText(frame, "Tiada tangan dikesan...",
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100,100,100), 1)

    cv2.imshow("Gesture Debug Tool", frame)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
