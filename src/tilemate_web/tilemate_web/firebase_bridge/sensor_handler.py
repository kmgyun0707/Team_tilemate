"""
firebase_bridge/sensor_handler.py
센서 데이터 수신 및 충돌 감지:
  - Tool Force (GetToolForce 서비스)
  - External Torque (GetExternalTorque 서비스)
  - TCP 속도 계산 (Jacobian)
  - JointState / TCP position 콜백
"""

import math
import time

import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray


# M0609 DH 파라미터 (a, d, alpha)
M0609_DH = [
    (0.0,   0.1555,  math.pi / 2),
    (0.409, 0.0,     0.0),
    (0.367, 0.0,     0.0),
    (0.0,   0.1335,  math.pi / 2),
    (0.0,   0.0995, -math.pi / 2),
    (0.0,   0.0996,  0.0),
]


def compute_tcp_velocity(q, qdot) -> float:
    """관절 위치·속도로 TCP 합속도(m/s)를 계산해 반환."""
    T = np.eye(4)
    transforms = [T.copy()]

    for i, (a, d, alpha) in enumerate(M0609_DH):
        ct, st = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(alpha), math.sin(alpha)
        A = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1],
        ])
        T = T @ A
        transforms.append(T.copy())

    p_n = transforms[6][:3, 3]
    J_v = np.zeros((3, 6))
    for i in range(6):
        z_i = transforms[i][:3, 2]
        p_i = transforms[i][:3, 3]
        J_v[:, i] = np.cross(z_i, p_n - p_i)

    v_tcp = J_v @ np.array(qdot)
    return float(round(np.linalg.norm(v_tcp), 4))


class SensorHandlerMixin:
    """
    FirebaseBridgeNode에 믹스인으로 사용.
    self.ref, self.cmd_ref, self._collision_detected, self._reset_time,
    self.COLLISION_THRESHOLD, self.FORCE_THRESHOLD, self.FORCE_Z_THRESHOLD,
    self._cancel_active_goal(), self._publish_reliable(), self.get_logger() 에 접근 가정.
    """

    # ── Tool Force ──────────────────────────────────────
    def _timer_get_tool_force(self):
        if not self._tool_force_client.service_is_ready():
            return
        from dsr_msgs2.srv import GetToolForce
        req = GetToolForce.Request()
        req.ref = 0
        self._tool_force_client.call_async(req).add_done_callback(
            self._cb_tool_force_response
        )

    def _cb_tool_force_response(self, future):
        try:
            res = future.result()
            if not res.success:
                return

            forces = {
                str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                for i, v in enumerate(res.tool_force)
            }
            fx, fy, fz = forces["0"], forces["1"], forces["2"]
            force_total = float(round(math.sqrt(fx**2 + fy**2 + fz**2), 2))

            self.ref.update({"tool_force": forces, "force_z": fz, "force_total": force_total})

            if self._collision_detected or (time.time() - self._reset_time <= 3.0):
                return

            if force_total > self.FORCE_THRESHOLD:
                self._trigger_emergency_stop(
                    joint=0, torque=force_total,
                    reason=f"force_total={force_total} > {self.FORCE_THRESHOLD}"
                )
            elif abs(fz) > self.FORCE_Z_THRESHOLD:
                self._trigger_emergency_stop(
                    joint=-1, torque=fz,
                    reason=f"fz={fz} > {self.FORCE_Z_THRESHOLD}"
                )

        except Exception as e:
            self.get_logger().error(f"[FORCE] error: {e}")

    # ── External Torque ──────────────────────────────────
    def _timer_get_ext_torque(self):
        if not self._ext_torque_client.service_is_ready():
            return
        from dsr_msgs2.srv import GetExternalTorque
        req = GetExternalTorque.Request()
        self._ext_torque_client.call_async(req).add_done_callback(
            self._cb_ext_torque_response
        )

    def _cb_ext_torque_response(self, future):
        try:
            res = future.result()
            if not res.success:
                return

            torque = {
                str(i): 0.0 if math.isnan(v) else float(round(v, 2))
                for i, v in enumerate(res.ext_torque)
            }
            self.ref.update({"ext_torque": torque})

            if self._collision_detected or (time.time() - self._reset_time <= 3.0):
                return

            for i, v in torque.items():
                if abs(v) > self.COLLISION_THRESHOLD:
                    self._trigger_emergency_stop(
                        joint=int(i) + 1, torque=v,
                        reason=f"J{int(i)+1} torque={v} > {self.COLLISION_THRESHOLD}"
                    )
                    break

        except Exception as e:
            self.get_logger().error(f"[EXT_TORQUE] error: {e}")

    # ── 공통 비상정지 ────────────────────────────────────
    def _trigger_emergency_stop(self, joint: int, torque: float, reason: str):
        """충돌/과부하 감지 시 비상정지 처리."""
        self._collision_detected = True
        self.get_logger().warn(f"[COLLISION] {reason}")

        self.cmd_ref.update({"action": "stop", "timestamp": int(time.time() * 1000)})
        self.ref.update({
            "state": "충돌 감지 - 비상정지",
            "collision_joint": joint,
            "collision_torque": torque,
        })
        self._cancel_active_goal()
        msg = String()
        msg.data = "stop"
        self._publish_reliable(self._pub_cmd, msg, retries=3)

    # ── ROS 상태 콜백 ────────────────────────────────────
    def _cb_joint_states(self, msg: JointState):
        now = time.time()
        if now - self._last_joint_update < 0.5:
            return
        self._last_joint_update = now

        pos, vel = list(msg.position), list(msg.velocity)
        if len(pos) >= 6 and len(vel) >= 6:
            tcp_speed = compute_tcp_velocity(pos[:6], vel[:6])
            self.ref.update({"joint_speed": tcp_speed})

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
