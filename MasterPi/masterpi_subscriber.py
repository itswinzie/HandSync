#!/usr/bin/env python3
# ============================================================
# MasterPi ROS2 Subscriber (SERVO ID BETUL)
#
# PWM Servo ID yang betul:
#   ID 1 = Gripper  (capit)       pulse 500~2500
#   ID 3 = Wrist    (pusing)      pulse 500~2500
#   ID 4 = Elbow    (bengkok)     pulse 500~2500
#   ID 5 = Shoulder (naik/turun)  pulse 500~2500
#   ID 6 = Base     (putar)       pulse 500~2500
#
# Motor Mecanum:
#   ID 1 = Depan Kiri
#   ID 2 = Depan Kanan
#   ID 3 = Belakang Kiri
#   ID 4 = Belakang Kanan
# ============================================================

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ros_robot_controller_sdk as rrc

# ── Board Setup ───────────────────────────────────────────────
board = rrc.Board()
board.enable_reception()
print("Board connected!")

SERVO_MIN = 500
SERVO_MAX = 2500

def clamp_pwm(v):
    return max(SERVO_MIN, min(SERVO_MAX, int(v)))

def reset_arm():
    board.pwm_servo_set_position(1.0, [
        [1, 1500],  # Gripper  — tengah
        [3, 1500],  # Wrist    — tengah
        [4, 1500],  # Elbow    — tengah
        [5, 1500],  # Shoulder — tengah
        [6, 1500],  # Base     — tengah
    ])
    print("  Arm reset ke posisi tengah (1500)")

def stop_motors():
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

# ── Mecanum Wheel ─────────────────────────────────────────────
def mecanum_move(linear=0, strafe=0, angular=0):
    fl =  linear + strafe + angular
    fr =  linear - strafe - angular
    bl =  linear - strafe + angular
    br =  linear + strafe - angular

    max_val = max(abs(fl), abs(fr), abs(bl), abs(br), 1)
    if max_val > 100:
        fl = fl / max_val * 100
        fr = fr / max_val * 100
        bl = bl / max_val * 100
        br = br / max_val * 100

    board.set_motor_duty([
        [1,  int(fl)],
        [2, -int(fr)],
        [3,  int(bl)],
        [4, -int(br)],
    ])
    print(f"  Mecanum → FL:{int(fl)} FR:{int(fr)} BL:{int(bl)} BR:{int(br)}")

# ── Arm Control ───────────────────────────────────────────────
def move_arm(gripper, wrist, elbow, shoulder, base, active=None):
    """
    active = servo yang perlu bergerak sahaja
    None   = gerak semua (untuk reset)
    """
    servo_map = {
        "gripper":  [1, clamp_pwm(gripper)],
        "wrist":    [3, clamp_pwm(wrist)],
        "elbow":    [4, clamp_pwm(elbow)],
        "shoulder": [5, clamp_pwm(shoulder)],
        "base":     [6, clamp_pwm(base)],
    }

    if active and active in servo_map:
        # Hanya gerak servo yang aktif sahaja
        data = [servo_map[active]]
        print(f"  Arm [{active}] → ID{data[0][0]}:{data[0][1]}")
    else:
        # Gerak semua (reset/init)
        data = list(servo_map.values())
        print(f"  Arm (all) → Gr:{clamp_pwm(gripper)} Sh:{clamp_pwm(shoulder)} El:{clamp_pwm(elbow)} Ba:{clamp_pwm(base)}")

    board.pwm_servo_set_position(0.05, data)

# ── ROS2 Subscriber ───────────────────────────────────────────
class MasterPiSubscriber(Node):
    def __init__(self):
        super().__init__('masterpi_executor')
        self.subscription = self.create_subscription(
            String, 'masterpi_command', self.callback, 10)
        self.get_logger().info('MasterPi Executor ready!')
        print("=" * 55)
        print("  MasterPi Subscriber Started")
        print("  ID1=Gripper  ID3=Wrist  ID4=Elbow")
        print("  ID5=Shoulder ID6=Base")
        print("  Pulse: 500~2500  |  Tengah: 1500")
        print("  Menunggu command dari Jetson...")
        print("=" * 55)

    def callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            t   = cmd.get("type", "")

            if t == "wheels":
                linear  = cmd.get("linear",  0)
                strafe  = cmd.get("strafe",  0)
                angular = cmd.get("angular", 0)

                if linear == 0 and strafe == 0 and angular == 0:
                    stop_motors()
                    print("  Motor → STOP")
                else:
                    print(f"[RODA] Linear:{linear} Strafe:{strafe} Angular:{angular}")
                    mecanum_move(linear, strafe, angular)

            elif t == "arm":
                gripper  = cmd.get("servo_gripper",  1500)
                wrist    = cmd.get("servo_wrist",    1500)
                elbow    = cmd.get("servo_elbow",    1500)
                shoulder = cmd.get("servo_shoulder", 1500)
                base     = cmd.get("servo_base",     1500)
                active   = cmd.get("active_servo",   None)  # Servo yang aktif
                move_arm(gripper, wrist, elbow, shoulder, base, active)

        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON: {e}")
        except Exception as e:
            print(f"[ERROR] {e}")

# ── Main ──────────────────────────────────────────────────────
def main():
    stop_motors()
    reset_arm()

    rclpy.init()
    node = MasterPiSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nBerhenti...")
    finally:
        stop_motors()
        node.destroy_node()
        rclpy.shutdown()
        print("Subscriber ditutup.")

if __name__ == '__main__':
    main()
