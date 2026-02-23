# firebase_bridge.py  (내 컴퓨터에서 실행)
# ROS2 토픽 subscribe → Firebase Realtime DB 업데이트
# Firebase robot_command 감지 → /robot/command, /robot/design 토픽 publish
#
# 구독 토픽 (로봇 → Firebase):
#   /robot/step                     (std_msgs/Int32)
#   /robot/state                    (std_msgs/String)
#   /robot/tcp                      (std_msgs/Float32MultiArray)  [x,y,z,rx,ry,rz]
#   /robot/completed_jobs           (std_msgs/Int32)
#   /robot/speed                    (std_msgs/Int32)
#   /robot/collision_sensitivity    (std_msgs/Int32)
#   ※ /robot/design 은 subscribe 안 함 (bridge가 publish 전용)
#
# 발행 토픽 (Firebase → 로봇):
#   /robot/command   (std_msgs/String)  "start" | "pause" | "reset"
#   /robot/design    (std_msgs/Int32)   1 | 2 | 3  ← 웹에서 선택한 디자인 번호

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray

import firebase_admin
from firebase_admin import credentials, db
import time

# ── Firebase 설정 ──────────────────────────────────────────
import os
SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
    "~/Team_tilemate/src/tilemate_web/config/co1-tiling-firebase-adminsdk-fbsvc-f4f88c3832.json"
)
DATABASE_URL = "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__("firebase_bridge")

        # Firebase 초기화
        if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
            raise FileNotFoundError(f"Firebase key not found: {SERVICE_ACCOUNT_KEY_PATH}")

        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
        self.ref     = db.reference("/robot_status")
        self.cmd_ref = db.reference("/robot_command")  # 전체 객체 읽기 (action + design)
        self.get_logger().info("Firebase Connected!")

        # Firebase 초기 상태
        self.ref.update({
            "current_step":   0,
            "state":          "대기",
            "pos_x":          0.0,
            "pos_y":          0.0,
            "pos_z":          0.0,
            "completed_jobs": 0,
        })

        # ── Publisher: 웹 명령 → 로봇 ──────────────────────
        self._pub_cmd    = self.create_publisher(String, '/robot/command', 10)
        self._pub_design = self.create_publisher(Int32,  '/robot/design',  10)
        self._pub_design_ab = self.create_publisher(String,  '/robot/design_ab',  10)

        # ── Subscriber: 로봇 상태 → Firebase ──────────────
        # /robot/design 은 여기서 subscribe 안 함 (bridge가 publish 전용)
        self.create_subscription(Int32,             "/robot/step",                  self._cb_step,                 10)
        self.create_subscription(String,            "/robot/state",                 self._cb_state,                10)
        self.create_subscription(Float32MultiArray, "/robot/tcp",                   self._cb_tcp,                  10)
        self.create_subscription(Int32,             "/robot/completed_jobs",         self._cb_completed_jobs,        10)
        self.create_subscription(Int32,             "/robot/speed",                  self._cb_speed,                 10)
        self.create_subscription(Int32,             "/robot/collision_sensitivity",   self._cb_collision_sensitivity, 10)

        self.get_logger().info("Subscribed: /robot/step, /robot/state, /robot/tcp, "
                               "/robot/completed_jobs, /robot/speed, /robot/collision_sensitivity")
        self.get_logger().info("Publishing: /robot/command, /robot/design,/robot/design_ab")

        # TCP throttle
        self._last_tcp_update = 0.0

        # 시작 시 현재 Firebase 값으로 _last_command 초기화 (묵은 명령 무시)
        
        self._last_command = "idle"
        self.get_logger().info(f"Firebase 현재 명령 상태: '{self._last_command}' (무시하고 시작)")

        # Firebase를 idle로 리셋해서 새 명령만 받도록
        self.cmd_ref.update({"action": "idle"})

        self._cmd_thread = threading.Thread(target=self._watch_firebase_command, daemon=True)
        self._cmd_thread.start()

    # ── Firebase 명령 감지 루프 ────────────────────────────
    def _watch_firebase_command(self):
        """Firebase robot_command 변화 감지 → /robot/command, /robot/design publish"""
        self.get_logger().info("Firebase command watcher started...")
        while rclpy.ok():
            try:
                cmd = self.cmd_ref.get()  # { action, design, timestamp }
                if isinstance(cmd, dict):
                    action = cmd.get("action", "idle")
                    design = cmd.get("design", None)
                else:
                    action = cmd if cmd else "idle"
                    design = None

                if action and action != self._last_command:
                    self._last_command = action
                    if action in ("start", "pause", "reset"):
                        # /robot/command publish
                        msg = String()
                        msg.data = action
                        self._pub_cmd.publish(msg)
                        self.get_logger().info(f"[CMD] Firebase '{action}' → /robot/command publish")

                        # start 명령이면:
                        if action == "start" and design is not None:
                            design_int = int(design)

                            # 1) /robot/design publish (항상 숫자 그대로)
                            d_msg = Int32()
                            d_msg.data = design_int
                            self._pub_design.publish(d_msg)
                            self.get_logger().info(
                                f"[DESIGN] design={design_int} -> /robot/design publish"
                            )
                            self.ref.update({"design": design_int})

                            # 2) /robot/design_ab publish (design=1 지그재그 전용)
                            # B 흰 A 검
                            if design_int == 1:
                                ZIGZAG_PATTERN = "B,A,B,A,B,A,B,A,B" # 흰검흰검흰검흰검흰 (흰5검4)
                                ab_msg = String()
                                ab_msg.data = ZIGZAG_PATTERN
                                self._pub_design_ab.publish(ab_msg)
                                self.get_logger().info(
                                    f"[DESIGN_AB] design=1 -> /robot/design_ab publish: '{ZIGZAG_PATTERN}'"
                                )
                                self.ref.update({"design_ab": ZIGZAG_PATTERN})
                            elif design_int == 2:
                                STRAIGHT_PATTERN = "B,B,B,A,A,A,B,B,B" # 흰흰흰검검검흰흰흰 (흰6검3)
                                ab_msg = String()
                                ab_msg.data = STRAIGHT_PATTERN
                                self._pub_design_ab.publish(ab_msg)
                                self.get_logger().info(
                                    f"[DESIGN_AB] design=2 -> /robot/design_ab publish: '{STRAIGHT_PATTERN}'"
                                )
                                self.ref.update({"design_ab": STRAIGHT_PATTERN})
                            else:
                                # 웹에서 직접 입력한 커스텀 패턴 사용
                                custom_pattern = cmd.get("custom_pattern", None)
                                if custom_pattern:
                                    ab_msg = String()
                                    ab_msg.data = custom_pattern
                                    self._pub_design_ab.publish(ab_msg)
                                    self.get_logger().info(
                                        f"[DESIGN_AB] design=3 (custom) -> /robot/design_ab publish: '{custom_pattern}'"
                                    )
                                    self.ref.update({"design_ab": custom_pattern})
                                else:
                                    self.get_logger().warn(
                                        "[DESIGN_AB] design=3 but no custom_pattern in Firebase! /robot/design_ab NOT published."
                                    )

                        # reset이면 Firebase robot_status 전체 초기화
                        if action == "reset":
                            self.ref.set({
                                "current_step":   0,
                                "state":          "대기",
                                "pos_x":          0.0,
                                "pos_y":          0.0,
                                "pos_z":          0.0,
                                "completed_jobs": 0,
                                "working_tile":   0,
                                "speed":          0,
                            })
                            self.get_logger().info("[RESET] Firebase robot_status 전체 초기화 완료")

            except Exception as e:
                self.get_logger().error(f"Firebase watch error: {e}")
            time.sleep(0.3)

    # ── 콜백: 로봇 상태 → Firebase ────────────────────────
    def _cb_step(self, msg: Int32):
        self.ref.update({"current_step": msg.data})
        self.get_logger().info(f"[STEP] → Firebase: {msg.data}")

    def _cb_state(self, msg: String):
        self.ref.update({"state": msg.data})
        self.get_logger().info(f"[STATE] → Firebase: {msg.data}")

    def _cb_completed_jobs(self, msg: Int32):
        self.ref.update({"completed_jobs": msg.data})
        self.get_logger().info(f"[COMPLETED] → Firebase: {msg.data}")

    def _cb_speed(self, msg: Int32):
        self.ref.update({"speed": msg.data})
        self.get_logger().info(f"[SPEED] → Firebase: {msg.data}")

    def _cb_collision_sensitivity(self, msg: Int32):
        self.ref.update({"collision_sensitivity": msg.data})
        self.get_logger().info(f"[COLLISION] → Firebase: {msg.data}")

    def _cb_tcp(self, msg: Float32MultiArray):
        now = time.time()
        if now - self._last_tcp_update < 0.2:  # 0.2초 throttle
            return
        self._last_tcp_update = now
        data = msg.data
        if len(data) >= 3:
            self.ref.update({
                "pos_x": round(float(data[0]), 2),
                "pos_y": round(float(data[1]), 2),
                "pos_z": round(float(data[2]), 2),
            })


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    print("Firebase Bridge running... (Ctrl+C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            db.reference("/robot_status").update({"state": "대기", "current_step": 0})
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()