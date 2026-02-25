# firebase_bridge.py  (내 컴퓨터에서 실행)
# ROS2 토픽 subscribe → Firebase Realtime DB 업데이트
# Firebase robot_command 감지 → /robot/command, /robot/design 토픽 publish
#
# 구독 토픽 (로봇 → Firebase):
#   /robot/step                     (std_msgs/Int32)
#   /robot/state                    (std_msgs/String)
#   /robot/tcp                      (std_msgs/Float32MultiArray)  [x,y,z,rx,ry,rz]
#   /robot/completed_jobs           (std_msgs/Int32)
#   /dsr01/joint_states             (sensor_msgs/JointState) → joint_speed
#   ※ /robot/design 은 subscribe 안 함 (bridge가 publish 전용)
#
# 서비스 콜 (주기적):
#   /dsr01/aux_control/get_tool_force      → tool_force, force_z  (툴 끝 외력)
#   /dsr01/aux_control/get_external_torque → ext_torque           (로봇 각 관절 외부 토크)
#
# 발행 토픽 (Firebase → 로봇):
#   /robot/command   (std_msgs/String)  "start" | "stop" | "reset"
#   /robot/design    (std_msgs/Int32)   1 | 2 | 3  ← 웹에서 선택한 디자인 번호

import threading
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetToolForce, GetExternalTorque

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
        self.cmd_ref = db.reference("/robot_command")
        self.get_logger().info("Firebase Connected!")

        # Firebase 초기 상태
        self.ref.update({
            "current_step":   0,
            "state":          "대기",
            "pos_x":          0.0,
            "pos_y":          0.0,
            "pos_z":          0.0,
            "completed_jobs": 0,
            "tool_force":     {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
            "force_z":        0.0,
            "ext_torque":     {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
        })

        # ── Publisher: 웹 명령 → 로봇 ──────────────────────
        self._pub_cmd       = self.create_publisher(String, '/robot/command',        10)
        self._pub_design    = self.create_publisher(Int32,  '/robot/design',         10)
        self._pub_design_ab = self.create_publisher(String, '/robot/design_ab',      10)
        self._pub_completed = self.create_publisher(Int32,  '/robot/completed_jobs', 10)
        self._pub_step      = self.create_publisher(Int32,  '/robot/step',           10)

        # ── Subscriber: 로봇 상태 → Firebase ──────────────
        self.create_subscription(Int32,             "/robot/step",                  self._cb_step,                  10)
        self.create_subscription(String,            "/robot/state",                 self._cb_state,                 10)
        self.create_subscription(Float32MultiArray, "/robot/tcp",                   self._cb_tcp,                   10)
        self.create_subscription(Int32,             "/robot/completed_jobs",         self._cb_completed_jobs,        10)
        self.create_subscription(JointState,        "/dsr01/joint_states",           self._cb_joint_states,          10)
        self.create_subscription(Int32,             "/robot/press",                  self._cb_press,                 10)

        self.get_logger().info("Subscribed: /robot/step, /robot/state, /robot/tcp, "
                               "/robot/completed_jobs, /dsr01/joint_states, /robot/press")
        self.get_logger().info("Publishing: /robot/command, /robot/design, /robot/design_ab")

        # ── 서비스 클라이언트 ──────────────────────────────
        self._tool_force_client = self.create_client(GetToolForce,     '/dsr01/aux_control/get_tool_force')
        self._ext_torque_client = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        self.create_timer(0.3, self._timer_get_tool_force)
        self.create_timer(0.3, self._timer_get_ext_torque)
        self.get_logger().info("Service clients: get_tool_force, get_external_torque")

        # ── 충돌 감지 설정 ────────────────────────────────
        self.COLLISION_THRESHOLD = 250.0   # 관절 외부토크 임계값 (Nm)
        self.FORCE_THRESHOLD     = 250.0   # TCP 합력 임계값 (N)
        self.FORCE_Z_THRESHOLD   = 218.0   # TCP Fz 임계값 (N)
        self._collision_detected = False  # 중복 stop 방지

        # throttle 타임스탬프
        self._last_tcp_update   = 0.0
        self._last_joint_update = 0.0

        # 묵은 명령 무시
        self._last_command = "idle"
        self.get_logger().info(f"Firebase 현재 명령 상태: '{self._last_command}' (무시하고 시작)")
        self.cmd_ref.update({"action": "idle"})

        self._cmd_thread = threading.Thread(target=self._watch_firebase_command, daemon=True)
        self._cmd_thread.start()

    # ── tool_force 서비스 콜 ──────────────────────────────
    def _timer_get_tool_force(self):
        if not self._tool_force_client.service_is_ready():
            return
        req = GetToolForce.Request()
        req.ref = 0
        future = self._tool_force_client.call_async(req)
        future.add_done_callback(self._cb_tool_force_response)

    def _cb_tool_force_response(self, future):
        try:
            res = future.result()
            if res.success:
                forces_dict = {
                    str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                    for i, v in enumerate(res.tool_force)
                }
                force_z = forces_dict.get("2", 0.0)

                # TCP 합력 계산 (Fx² + Fy² + Fz² 의 루트, 선형력 3축만)
                fx = forces_dict.get("0", 0.0)
                fy = forces_dict.get("1", 0.0)
                fz = forces_dict.get("2", 0.0)
                force_total = float(round(math.sqrt(fx**2 + fy**2 + fz**2), 2))

                self.ref.update({
                    "tool_force":  forces_dict,
                    "force_z":     force_z,
                    "force_total": force_total,
                })

                # ── TCP 합력 임계값 초과 시 자동 비상정지 ──
                if not self._collision_detected and force_total > self.FORCE_THRESHOLD:
                    self._collision_detected = True
                    self.get_logger().warn(
                        f"[FORCE DETECTED] TCP 합력={force_total} N "
                        f"(threshold={self.FORCE_THRESHOLD}) → 자동 비상정지!"
                    )
                    self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                    self.ref.update({
                        "state":            "충돌 감지 - 비상정지",
                        "collision_joint":  0,
                        "collision_torque": force_total,
                    })
                    msg = String()
                    msg.data = "stop"
                    self._pub_cmd.publish(msg)
                    self._last_command = "stop"

                # ── TCP Fz 임계값 초과 시 자동 비상정지 ──
                elif not self._collision_detected and abs(fz) > self.FORCE_Z_THRESHOLD:
                    self._collision_detected = True
                    self.get_logger().warn(
                        f"[FORCE_Z DETECTED] TCP Fz={fz} N "
                        f"(threshold={self.FORCE_Z_THRESHOLD}) → 자동 비상정지!"
                    )
                    self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                    self.ref.update({
                        "state":            "충돌 감지 - 비상정지",
                        "collision_joint":  -1,   # -1 = Fz 감지
                        "collision_torque": fz,
                    })
                    msg = String()
                    msg.data = "stop"
                    self._pub_cmd.publish(msg)
                    self._last_command = "stop"
        except Exception as e:
            self.get_logger().error(f"[FORCE] error: {e}")

    # ── ext_torque 서비스 콜 ──────────────────────────────
    def _timer_get_ext_torque(self):
        if not self._ext_torque_client.service_is_ready():
            return
        req = GetExternalTorque.Request()
        future = self._ext_torque_client.call_async(req)
        future.add_done_callback(self._cb_ext_torque_response)

    def _cb_ext_torque_response(self, future):
        try:
            res = future.result()
            if res.success:
                torque_dict = {
                    str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                    for i, v in enumerate(res.ext_torque)
                }
                self.ref.update({"ext_torque": torque_dict})

                # ── 충돌 감지: 어느 관절이든 임계값 초과 시 자동 비상정지 ──
                if not self._collision_detected:
                    for i, v in torque_dict.items():
                        if abs(v) > self.COLLISION_THRESHOLD:
                            self._collision_detected = True
                            self.get_logger().warn(
                                f"[COLLISION DETECTED] J{int(i)+1} ext_torque={v} Nm "
                                f"(threshold={self.COLLISION_THRESHOLD}) → 자동 비상정지!"
                            )
                            # Firebase에 stop 명령 + 충돌 정보 기록
                            self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
                            self.ref.update({
                                "state":              "충돌 감지 - 비상정지",
                                "collision_joint":    int(i) + 1,
                                "collision_torque":   v,
                            })
                            # /robot/command 토픽에도 즉시 publish
                            msg = String()
                            msg.data = "stop"
                            self._pub_cmd.publish(msg)
                            self._last_command = "stop"
                            break

                # reset 명령이 오면 충돌 감지 플래그 해제 (watch_firebase_command에서 처리)
        except Exception as e:
            self.get_logger().error(f"[EXT_TORQUE] error: {e}")

    # ── 중요 명령 재시도 publish ───────────────────────────
    def _publish_reliable(self, publisher, msg, retries=3, interval=0.05):
        """
        중요 명령을 retries회 반복 publish해서 씹힘 방지.
        subscriber가 아직 연결 안 됐거나 타이밍 문제를 커버.
        interval: 각 publish 사이 간격 (초)
        """
        for i in range(retries):
            publisher.publish(msg)
            if i < retries - 1:
                time.sleep(interval)

    # ── Firebase 명령 감지 루프 ────────────────────────────
    def _watch_firebase_command(self):
        self.get_logger().info("Firebase command watcher started...")
        while rclpy.ok():
            try:
                cmd = self.cmd_ref.get()
                if isinstance(cmd, dict):
                    action = cmd.get("action", "idle")
                    design = cmd.get("design", None)
                else:
                    action = cmd if cmd else "idle"
                    design = None

                if action and action != self._last_command:
                    self._last_command = action
                    if action in ("start", "stop", "reset", "resume"):
                        msg = String()
                        msg.data = action
                        self._publish_reliable(self._pub_cmd, msg)
                        self.get_logger().info(f"[CMD] Firebase '{action}' → /robot/command publish (x3)")

                        if action == "start":
                            is_resume = cmd.get("is_resume", False) if isinstance(cmd, dict) else False
                            if is_resume:
                                completed_jobs = int(cmd.get("completed_jobs", 0))
                                current_step   = int(cmd.get("current_step",   0))

                                cj_msg = Int32()
                                cj_msg.data = completed_jobs
                                self._pub_completed.publish(cj_msg)
                                self.get_logger().info(f"[RESUME] completed_jobs={completed_jobs} → /robot/completed_jobs publish")

                                st_msg = Int32()
                                st_msg.data = current_step
                                self._pub_step.publish(st_msg)
                                self.get_logger().info(f"[RESUME] current_step={current_step} → /robot/step publish")

                        if action == "start" and design is not None:
                            design_int = int(design)

                            d_msg = Int32()
                            d_msg.data = design_int
                            self._publish_reliable(self._pub_design, d_msg)
                            self.get_logger().info(f"[DESIGN] design={design_int} -> /robot/design publish (x3)")
                            self.ref.update({"design": design_int})

                            if design_int == 1:
                                ZIGZAG_PATTERN = "B,A,B,A,B,A,B,A,B"
                                ab_msg = String()
                                ab_msg.data = ZIGZAG_PATTERN
                                self._publish_reliable(self._pub_design_ab, ab_msg)
                                self.get_logger().info(f"[DESIGN_AB] design=1 -> /robot/design_ab publish (x3): '{ZIGZAG_PATTERN}'")
                                self.ref.update({"design_ab": ZIGZAG_PATTERN})
                            elif design_int == 2:
                                STRAIGHT_PATTERN = "B,B,B,A,A,A,B,B,B"
                                ab_msg = String()
                                ab_msg.data = STRAIGHT_PATTERN
                                self._publish_reliable(self._pub_design_ab, ab_msg)
                                self.get_logger().info(f"[DESIGN_AB] design=2 -> /robot/design_ab publish (x3): '{STRAIGHT_PATTERN}'")
                                self.ref.update({"design_ab": STRAIGHT_PATTERN})
                            else:
                                custom_pattern = cmd.get("custom_pattern", None)
                                if custom_pattern:
                                    ab_msg = String()
                                    ab_msg.data = custom_pattern
                                    self._publish_reliable(self._pub_design_ab, ab_msg)
                                    self.get_logger().info(f"[DESIGN_AB] design=3 (custom) -> /robot/design_ab publish (x3): '{custom_pattern}'")
                                    self.ref.update({"design_ab": custom_pattern})
                                else:
                                    self.get_logger().warn("[DESIGN_AB] design=3 but no custom_pattern in Firebase!")

                        if action == "reset":
                            self._collision_detected = False  # 충돌 감지 플래그 해제
                            self.ref.set({
                                "current_step":   0,
                                "state":          "대기",
                                "pos_x":          0.0,
                                "pos_y":          0.0,
                                "pos_z":          0.0,
                                "completed_jobs": 0,
                                "working_tile":   0,
                                "speed":          0,
                                "design":         0,
                                "design_ab":      "",
                                "press_tile":     0,
                                "tool_force":     {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
                                "force_z":        0.0,
                                "ext_torque":     {"0": 0.0, "1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0},
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
        
    # ── 두산 M0609 Jacobian 기반 TCP 선속도 계산 ────────────
    # DH 파라미터 (a, d, alpha) — 단위: m, rad
    # 출처: 두산로보틱스 M0609 매뉴얼
    M0609_DH = [
        # (a,      d,      alpha)
        (0.0,    0.1555,  math.pi/2),   # Joint 1
        (0.409,  0.0,     0.0),         # Joint 2
        (0.367,  0.0,     0.0),         # Joint 3
        (0.0,    0.1335,  math.pi/2),   # Joint 4
        (0.0,    0.0995, -math.pi/2),   # Joint 5
        (0.0,    0.0996,  0.0),         # Joint 6
    ]

    def _compute_tcp_velocity(self, q, qdot):
        """
        q    : 관절 각도 리스트 (rad), 6개
        qdot : 관절 각속도 리스트 (rad/s), 6개
        반환  : TCP 선속도 크기 (m/s)

        Jacobian 위치 열(J_v) 계산:
          T_0_i = A1 * A2 * ... * Ai  (i번 관절까지의 변환행렬)
          z_i   = T_0_i 의 3번째 열 (z축 방향)
          p_i   = T_0_i 의 위치 벡터
          J_v_i = z_{i-1} × (p_n - p_{i-1})   (회전 관절)
          v_tcp = J_v · qdot
        """
        dh = self.M0609_DH

        # 각 관절까지의 변환행렬 누적
        T = np.eye(4)
        transforms = [T.copy()]  # T_0_0 = I
        for i, (a, d, alpha) in enumerate(dh):
            ct = math.cos(q[i]); st = math.sin(q[i])
            ca = math.cos(alpha); sa = math.sin(alpha)
            A = np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d   ],
                [0,   0,      0,     1   ]
            ])
            T = T @ A
            transforms.append(T.copy())

        p_n = transforms[6][:3, 3]  # TCP 위치

        # Jacobian 위치 열 (3×6)
        J_v = np.zeros((3, 6))
        for i in range(6):
            z_i = transforms[i][:3, 2]   # i번 프레임 z축
            p_i = transforms[i][:3, 3]   # i번 프레임 위치
            J_v[:, i] = np.cross(z_i, p_n - p_i)

        v_tcp = J_v @ np.array(qdot)     # TCP 선속도 벡터 (m/s)
        return float(round(np.linalg.norm(v_tcp), 4))

    def _cb_joint_states(self, msg: JointState):
        now = time.time()
        if now - self._last_joint_update < 0.5:
            return
        self._last_joint_update = now

        pos = list(msg.position)   # 관절 각도 (rad)
        vel = list(msg.velocity)   # 관절 각속도 (rad/s)

        if len(pos) >= 6 and len(vel) >= 6:
            tcp_speed = self._compute_tcp_velocity(pos[:6], vel[:6])
            self.ref.update({"joint_speed": tcp_speed})
            self.get_logger().debug(f"[TCP_SPEED] {tcp_speed} m/s")

    def _cb_tcp(self, msg: Float32MultiArray):
        now = time.time()
        if now - self._last_tcp_update < 0.2:
            return
        self._last_tcp_update = now
        data = msg.data
        if len(data) >= 3:
            self.ref.update({
                "pos_x": round(float(data[0]), 2),
                "pos_y": round(float(data[1]), 2),
                "pos_z": round(float(data[2]), 2),
            })

    def _cb_press(self, msg: Int32):
        # /robot/press: 현재 압착 중인 타일 번호 (1~9)
        press_tile = msg.data
        self.ref.update({"press_tile": press_tile})
        self.get_logger().info(f"[PRESS] tile={press_tile} → Firebase")


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