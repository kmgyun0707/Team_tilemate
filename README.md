# Team_tilemate
<img width="1244" height="632" alt="image (1)" src="https://github.com/user-attachments/assets/930d1c35-4146-41d1-9124-43e49bd45ff6" />
## Project Tree
```text
├── tilemate_main
│   ├── launch
│   │   └── tilemate.launch.py # ROS LAUNCH CONFIG FILE
│   ├── package.xml #ros2 package configuration
│   ├── resource    #ros2 package configuration
│   │   └── tilemate_main
│   ├── setup.cfg   #ros2 package configuration
│   ├── setup.py    #ros2 package configuration
│   └── tilemate_main
│       ├── Boilerplate.py  # Code BoilerPlate
│       ├── gripper_node.py 
│       ├── __init__.py
│       ├── interrupt_node.py
│       ├── robot_config.py     # Doosan Robot Config Parameters 
│       ├── scraper_motion_node.py
│       ├── task_manager_node.py
│       ├── tile_compact_motion_node.py
│       ├── tile_inspect_motion_node.py
│       └── tile_motion_node.py
└── tilemate_web
    ├── package.xml #ros2 package configuration
    ├── resource    #ros2 package configuration
    │   └── tilemate_web
    ├── setup.cfg   #ros2 package configuration
    ├── setup.py    #ros2 package configuration
    └── tilemate_web
        ├── firebase_bridge.py  # ROS-FIREBASE Bridge
        ├── index.html  # WEB PAGE included script,css
        ├── __init__.py
        └── testno.py   # TEST NODE

```
## How To BringUP & Launch 
### 1. 두산 로봇 팔 브링업

- 환경설정 + ROS DOMAIN확인

```bash
export PYTHONPATH=$PYTHONPATH:~/cobot_ws/install/dsr_common2/lib/dsr_common2/imp
```

- 로봇 연결

```bash
#virtual
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

#real
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

---

### 2. 그리퍼 드라이버 실행

- 드라이버 설치
    
    https://github.com/tonydle/OnRobot_ROS2_Driver/tree/main


    
    0 . Navigate to your ROS 2 workspace 
    
    ```bash
    cd ~/cobot_ws/src
    ```
    
    1.  **clone the repository** into the `src` directory:
        
        ```bash
        git clone --recurse-submodules https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
        ```
        
    2. Install git dependencies using `vcs`:
        
        ```bash
        vcs import src --input src/onrobot_driver/required.repos
        ```
        
    3. Install libnet:
        
        ```bash
        sudo apt install libnet1-dev
        ```
        
    4. Build using colcon with symlink install:
        
        ```bash
        colcon build --symlink-install
        ```
        
    5. Source the workspace:
        
        `source install/setup.bash`
        
- 연결테스트
    
    ```bash
    jeonguk@jeonguk:~/cobot_ws/src$ ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
    publisher: beginning loop
    publishing #1: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.05])
    
    ```
    
     "{data: [0.05]}" → 단위 m , 그리퍼가 벌리는 폭
    

```bash
ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=tcp ip_address:=192.168.1.1 port:=502 launch_rviz:=false

```

- **드라이버 설치필수**
- **해당 런치 실행중일 경우 192.168.1.1 제어 불가능. 필요시 해당 런치 종료**
- 192.168.1.1 → Id: admin pw: 12345678

---

### 3. Tilemate launch

```bash
cd ~/Team_tilemate && source install/setup.bash
```
```bash
ros2 launch tilemate_main tilemate.launch.py
```
