"""
로봇 제어 유틸리티 모듈
- 일시정지, 재개, 정지 등의 기능 제공
"""
import rclpy
import DR_init

def call_pause(robot_id="dsr01"):
    """
    [핵심 기능] 로봇 이동 일시 정지 (Service Call)
    - ROS 2 Service를 직접 호출하여 즉각적인 Pause를 요청합니다.
    
    Args:
        robot_id: 로봇 ID (기본값: "dsr01")
    
    Returns:
        bool: 성공 여부
    """
    from dsr_msgs2.srv import MovePause
    
    try:
        # 서비스 클라이언트 생성
        cli = DR_init.__dsr__node.create_client(MovePause, f'/{robot_id}/motion/move_pause')
        
        # 서비스 서버가 준비될 때까지 대기
        if not cli.wait_for_service(timeout_sec=1.0):
            print("MovePause service not available")
            return False
        
        # 요청 전송 및 완료 대기
        req = MovePause.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(DR_init.__dsr__node, future)
        
        print(">>> 로봇 이동이 Pause 되었습니다.")
        return True
    except Exception as e:
        print(f"Pause 호출 실패: {e}")
        return False


def call_resume(robot_id="dsr01"):
    """
    [핵심 기능] 로봇 이동 재개 (Service Call)
    - Pause 된 로봇의 남은 모션을 재개합니다.
    
    Args:
        robot_id: 로봇 ID (기본값: "dsr01")
    
    Returns:
        bool: 성공 여부
    """
    from dsr_msgs2.srv import MoveResume
    
    try:
        # 서비스 클라이언트 생성
        cli = DR_init.__dsr__node.create_client(MoveResume, f'/{robot_id}/motion/move_resume')
        
        if not cli.wait_for_service(timeout_sec=1.0):
            print("MoveResume service not available")
            return False
        
        # 요청 전송 및 완료 대기
        req = MoveResume.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(DR_init.__dsr__node, future)
        
        print(">>> 로봇 이동이 Resume 되었습니다.")
        return True
    except Exception as e:
        print(f"Resume 호출 실패: {e}")
        return False


def call_stop(robot_id="dsr01", stop_mode=0):
    """
    [핵심 기능] 로봇 정지 (Service Call)
    - 로봇의 모든 동작을 정지시킵니다.
    
    Args:
        robot_id: 로봇 ID (기본값: "dsr01")
        stop_mode: 정지 모드 (0: 급정지, 1: 감속 정지)
    
    Returns:
        bool: 성공 여부
    """
    from dsr_msgs2.srv import MoveStop
    
    try:
        # 서비스 클라이언트 생성
        cli = DR_init.__dsr__node.create_client(MoveStop, f'/{robot_id}/motion/move_stop')
        
        if not cli.wait_for_service(timeout_sec=1.0):
            print("MoveStop service not available")
            return False
        
        # 요청 전송 및 완료 대기
        req = MoveStop.Request()
        req.stop_mode = stop_mode
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(DR_init.__dsr__node, future)
        
        print(f">>> 로봇이 정지되었습니다 (stop_mode={stop_mode})")
        return True
    except Exception as e:
        print(f"Stop 호출 실패: {e}")
        return False