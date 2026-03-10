#!/usr/bin/env python3
# testno.py (test_action_server.py)

import time
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Int32, Float32
from tilemate_msgs.action import ExecuteJob

class TestJobServer(Node):
    # ----------------------------
    # overall step enum
    # ----------------------------
    OVERALL_READY = 0
    OVERALL_TILE_WORK = 1
    OVERALL_FINISHED = 2

    # ----------------------------
    # tile step enum
    # ----------------------------
    TILE_STEP_IDLE = 0
    TILE_STEP_PICK = 1
    TILE_STEP_PLACE = 2
    TILE_STEP_INSPECT = 3
    TILE_STEP_COMPACT = 4
    TILE_STEP_DONE = 5

    def __init__(self):
        super().__init__('test_job_server')
        
        self._action_server = ActionServer(
            self,
            ExecuteJob,
            '/task/run_job',
            self.execute_callback
        )
        
        # 검수(Step 3) 및 압착(Step 4) UI 연동을 위한 토픽 퍼블리셔
        self.pub_inspect_no = self.create_publisher(Int32, '/robot/tile_inspect_no', 10)
        self.pub_tile_level = self.create_publisher(Float32, '/robot/tile_level', 10)
        self.pub_pressing_no = self.create_publisher(Int32, '/robot/pressing_no', 10)
        
        self.get_logger().info('테스트 액션 서버가 켜졌습니다. (Enum 반영 버전)')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Goal 수신 완료! 타일 9개 시뮬레이션을 시작합니다.')
        feedback_msg = ExecuteJob.Feedback()
        
        req = goal_handle.request
        start_tile = req.completed_jobs if hasattr(req, 'completed_jobs') and req.is_resume else 0
        layout = req.design_layout if hasattr(req, 'design_layout') and req.design_layout else [1] * 9
        total_tiles = 9
        
        # 한 타일 내에서 수행될 4가지 핵심 Enum 상태
        active_steps = [
            (self.TILE_STEP_PICK, "타일 파지 중"),
            (self.TILE_STEP_PLACE, "타일 부착 중"),
            (self.TILE_STEP_INSPECT, "단차 검수 중"),
            (self.TILE_STEP_COMPACT, "압착 보정 중")
        ]
        
        for i in range(start_tile, total_tiles):
            if hasattr(feedback_msg, 'tile_index'): feedback_msg.tile_index = i
            if hasattr(feedback_msg, 'tile_type'): feedback_msg.tile_type = layout[i] if i < len(layout) else 1
            
            # 각 타일마다 PICK -> PLACE -> INSPECT -> COMPACT 순차 진행
            for step_idx, (step_enum, step_str) in enumerate(active_steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal이 취소되었습니다.')
                    return ExecuteJob.Result()

                if hasattr(feedback_msg, 'overall_step'): feedback_msg.overall_step = self.OVERALL_TILE_WORK
                if hasattr(feedback_msg, 'detail_step'): feedback_msg.detail_step = step_enum
                if hasattr(feedback_msg, 'state'): feedback_msg.state = f"타일 {i+1} - {step_str}"
                
                # 로딩바가 부드럽게 차오르도록 잘게 쪼개서 피드백 발행 (10틱)
                for p in range(1, 11):
                    time.sleep(0.1)
                    
                    # 4개 단계를 거치며 타일 하나의 진행도(detail_progress)가 0.0 -> 1.0으로 참
                    current_detail_progress = (step_idx * 10 + p) / 40.0
                    
                    # 9개 타일 전체의 진행도(overall_progress)
                    completed_fraction = i / total_tiles
                    current_fraction = current_detail_progress / total_tiles
                    overall_progress = completed_fraction + current_fraction
                    
                    if hasattr(feedback_msg, 'detail_progress'): feedback_msg.detail_progress = float(current_detail_progress)
                    if hasattr(feedback_msg, 'overall_progress'): feedback_msg.overall_progress = float(overall_progress)
                        
                    goal_handle.publish_feedback(feedback_msg)
                    
                # ── 단차 검수(INSPECT) 시뮬레이션 토픽 발행 ──
                if step_enum == self.TILE_STEP_INSPECT:
                    msg_inspect = Int32()
                    msg_inspect.data = i + 1
                    self.pub_inspect_no.publish(msg_inspect)
                    
                    time.sleep(0.1)
                    msg_level = Float32()
                    msg_level.data = round(random.uniform(0.5, 1.2), 2)  # 0.5 ~ 1.2mm 사이 정상범위 난수
                    self.pub_tile_level.publish(msg_level)
                    
                # ── 압착 보정(COMPACT) 시뮬레이션 토픽 발행 ──
                elif step_enum == self.TILE_STEP_COMPACT:
                    msg_pressing = Int32()
                    msg_pressing.data = i + 1
                    self.pub_pressing_no.publish(msg_pressing)

            # 한 타일의 모든 작업이 끝나면 DONE(5) 상태 1회 발행
            if hasattr(feedback_msg, 'detail_step'): feedback_msg.detail_step = self.TILE_STEP_DONE
            if hasattr(feedback_msg, 'state'): feedback_msg.state = f"타일 {i+1} 배치 완료"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.3)

        # 9개 타일 모두 완료
        if hasattr(feedback_msg, 'overall_step'): feedback_msg.overall_step = self.OVERALL_FINISHED
        if hasattr(feedback_msg, 'detail_step'): feedback_msg.detail_step = self.TILE_STEP_IDLE
        if hasattr(feedback_msg, 'overall_progress'): feedback_msg.overall_progress = 1.0
        if hasattr(feedback_msg, 'detail_progress'): feedback_msg.detail_progress = 1.0
        if hasattr(feedback_msg, 'state'): feedback_msg.state = "모든 작업 완료"
        goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        
        result = ExecuteJob.Result()
        if hasattr(result, 'success'): result.success = True
        if hasattr(result, 'message'): result.message = "모든 타일 작업(파지/부착/검수/압착) 완료!"
            
        self.get_logger().info('Goal 성공적으로 완료됨!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TestJobServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()