## Installation
### mujoco
```
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.5/mujoco-3.2.5-linux-x86_64.tar.gz
```

```
pip3 install mujoco
```

### example package
```
cd ~/ros2_ws/src
git clone https://github.com/JAEWOOK6488/mujoco_bootcamp.git
```

```
cd ~/ros2_ws
colcon build --symlink-install
```

## Usage
### 2D Manipulator (2 DoF)
- 2자유도 로봇팔
```
ros2 run mujoco_bootcamp run_mujoco_2d_manipulator
```

### Diff Drive Drive
- 디퍼런셜 드라이브 로봇
```
ros2 run mujoco_bootcamp run_mujoco_diff_drive
```

### Ball in Drag
- 저항 받는 공
```
ros2 run mujoco_bootcamp run_mujoco_ball
```

### Interactive mujoco
- 데이터 오버레이 시키기
```
ros2 run mujoco_bootcamp run_mujoco_overlay_disp
```

```
ros2 run mujoco_bootcamp run_mujoco_glfw
```
