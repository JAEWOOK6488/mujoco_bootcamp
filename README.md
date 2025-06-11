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
```
ros2 run mujoco_bootcamp run_mujoco_2d_manipulator
```

### Diff Drive Drive
```
ros2 run mujoco_bootcamp run_mujoco_diff_drive
```