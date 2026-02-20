#!/usr/bin/env python3
"""
fake_publisher.py - 웹 기능 검증용 페이크 토픽 퍼블리셔

[s] 시작  → step 1→2→3→4 (0.2초 간격) 반복, 4 끝나면 completed_jobs +1
[p] 일시정지
[r] 초기화
[q] 종료
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray
import threading
import time

class FakePublisher(Node):
    def __init__(self):
        super().__init__("fake_publisher")

        self._pub_jobs  = self.create_publisher(Int32,             "/robot/completed_jobs",        10)
        self._pub_step  = self.create_publisher(Int32,             "/robot/step",                  10)
        self._pub_state = self.create_publisher(String,            "/robot/state",                 10)
        self._pub_speed = self.create_publisher(Int32,             "/robot/speed",                 10)
        self._pub_col   = self.create_publisher(Int32,             "/robot/collision_sensitivity",  10)

        self._completed_jobs = 0
        self._running = False
        self._sim_thread = None

    def pub_jobs(self, val):
        msg = Int32(); msg.data = val
        self._pub_jobs.publish(msg)

    def pub_step(self, val):
        msg = Int32(); msg.data = val
        self._pub_step.publish(msg)

    def pub_state(self, val):
        msg = String(); msg.data = val
        self._pub_state.publish(msg)

    def pub_speed(self, val):
        msg = Int32(); msg.data = val
        self._pub_speed.publish(msg)

    def start_sim(self):
        """step 1→2→3→4 반복, 4 완료 시 completed_jobs +1"""
        self._running = True
        step_names = {1: "접착제 파지", 2: "접착제 도포", 3: "타일 파지", 4: "타일 배치"}
        while self._running and rclpy.ok():
            for step in [1, 2, 3, 4]:
                if not self._running:
                    break
                self.pub_step(step)
                self.pub_state(step_names[step])
                print(f"  >> step={step} ({step_names[step]})")
                time.sleep(0.2)

            if not self._running:
                break

            # step 4 완료 → completed_jobs +1
            self._completed_jobs += 1
            self.pub_jobs(self._completed_jobs)
            self.pub_step(0)
            print(f"  >> ✅ completed_jobs={self._completed_jobs}")

            # 45개 완료 시 자동 종료
            if self._completed_jobs >= 45:
                print("  >> 모든 타일 완료!")
                self._running = False
                break

    def stop_sim(self):
        self._running = False

    def reset(self):
        self._running = False
        time.sleep(0.3)
        self._completed_jobs = 0
        self.pub_jobs(0)
        self.pub_step(0)
        self.pub_state("대기")
        self.pub_speed(0)
        print("  >> 전체 초기화 완료")


def menu(node: FakePublisher):
    print("\n" + "="*45)
    print("  Tilemate 페이크 토픽 퍼블리셔")
    print("="*45)
    print("  [s] 시작  (step 1→2→3→4 반복 + completed_jobs +1)")
    print("  [p] 일시정지")
    print("  [r] 초기화")
    print("  [q] 종료")
    print("="*45)

    while rclpy.ok():
        cmd = input(f"\n[jobs={node._completed_jobs}] 선택 > ").strip().lower()

        if cmd == "s":
            if node._running:
                print("  이미 실행 중입니다")
            else:
                print("  >> 시뮬레이션 시작")
                node._sim_thread = threading.Thread(target=node.start_sim, daemon=True)
                node._sim_thread.start()

        elif cmd == "p":
            node.stop_sim()
            node.pub_step(0)
            node.pub_state("일시정지")
            print("  >> 일시정지")

        elif cmd == "r":
            node.reset()

        elif cmd == "q":
            node.stop_sim()
            print("종료합니다.")
            break

        else:
            print("  s/p/r/q 중 선택하세요")


def main():
    rclpy.init()
    node = FakePublisher()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        menu(node)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        node.stop_sim()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()