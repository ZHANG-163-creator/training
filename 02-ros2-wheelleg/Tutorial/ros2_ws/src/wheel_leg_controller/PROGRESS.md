# wheel_leg_controller 进度记录

> 第 09 章大作业：轮腿机器人 LQR/VMC 平衡控制器
> 本文件随做随更，可以直接在 VS Code 里打开查看（与代码同一目录）。
> 最后更新：2026-11-06

---

## 一、当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 | 读懂仿真器（话题、dt=0.002s、指令驱动、心跳） | ✅ 完成 |
| 1 | 打通链路（能收 /imu、/joint_states，能发 /actuator_cmd） | ✅ 已跑通（帧数 27000+，车翻倒=预期） |
| 2 | 轮速 PI（轮子跟随 /cmd_vel） | 🔁 跳过，直接并入阶段 4 的 LQR（v_set） |
| 3 | VMC 接入（腿长 PID + 五连杆正运动学） | ✅ 代码已写（vmc_cal.py 已复制进包） |
| 4 | **LQR 平衡（必做）**：原地平衡 ≥5 s，pitch <10° | ✅ 代码已写（lqr_update.py 已复制进包），**待跑通验证** |
| 5 | 加分项（平衡下走、加速度限幅、抗扰、手柄） | 🔁 爬坡限幅已内置；手柄待做 |
| 6 | 提交物（README、录屏/bag、git push） | ⬜ 待做 |

### 当前代码结构（2026-11-06 更新为阶段 3+4 版本）

控制器主文件 358 行，每个控制周期依次做 6 件事：
① 四元数→pitch、角速度→gyro_pitch/gyro_yaw（原工程取负号）
② 左右轮共模 → 整车位移/速度
③ 五连杆正运动学 → L_0、theta
④ 腿长 PID + 重力前馈 → F_0
⑤ LQR：奇数 K → 髋关节 T_p，偶数 K → 轮力矩 wheel_T
⑥ 打包 6 个力矩发布

可调参数（无需改代码）：
```bash
ros2 run wheel_leg_controller wheel_leg_controller \
  --ros-args -p target_length:=0.10 -p wheel_radius:=0.06
```


---

## 二、文件说明

```
src/wheel_leg_controller/
├── package.xml              # 依赖：rclpy / sensor_msgs / geometry_msgs / rm_msgs（ros2 pkg create 生成，未改）
├── setup.py                 # 入口注册：wheel_leg_controller = wheel_leg_controller.wheel_leg_controller:main
├── setup.cfg
├── PROGRESS.md              # 本文件
└── wheel_leg_controller/
    ├── __init__.py
    └── wheel_leg_controller.py    # ← 唯一需要改的文件（当前 = 阶段 1）
```

---

## 三、阶段 1 已完成的内容（代码要点）

- 常量：腿力矩 ±6.0、轮力矩 ±2.0（对应 `mjcf/robot.xml` 的 `<actuator ctrlrange>`）
- `JOINT_NAMES = ['right1','right2','left1','left2','rightwheel','leftwheel']`（与 `wheel_leg_sim.py` 第 83 行一致）
- `clamp()`：出口统一限幅，避免算错一个数就把模型打飞
- 订阅：`/imu`（Imu）、`/joint_states`（JointState）、`/cmd_vel`（Twist）
- 发布：`/actuator_cmd`（`rm_msgs/msg/WheelLegCmd`，6 个字段）
- 控制循环写在 `on_joints()` 里：收到一帧状态 → 算一次 → 立刻发下一条指令
- 阶段 1 输出固定力矩（`STAGE1_RIGHT_WHEEL = -0.3`、`STAGE1_LEFT_WHEEL = 0.3`），只为验证链路

### 关键机制备忘

- **指令驱动**：仿真器每收到一条 `/actuator_cmd` 才 `mj_step` 一个 `dt = 0.002 s`，然后回发一次状态。没人发指令 → 时间不流逝 → 车定在原地（不是 bug）。
- **心跳**：超过 0.5 s 没收到指令就补发一次状态，所以空跑时 `/joint_states` 约 2 Hz。
- **闭环自激**：阶段 1 一收一发，`/actuator_cmd` 频率会很高（几百～上千 Hz），属正常。
- **6 字段顺序**：`right_phi1→ctrl[0]`、`right_phi4→ctrl[1]`、`left_phi1→ctrl[2]`、`left_phi4→ctrl[3]`、`right_wheel→ctrl[4]`、`left_wheel→ctrl[5]`，错位会打到别的电机。

---

## 四、怎么跑（每个终端都要先 `ros2on`）

```bash
# 终端 A：仿真器（会弹 MuJoCo 窗口）
ros2on
ros2 run wheel_leg_sim wheel_leg_sim

# 终端 B：控制器
ros2on
ros2 run wheel_leg_controller wheel_leg_controller

# 终端 C：观察
ros2on
ros2 node list                  # 应只有 /wheel_leg_sim 和 /wheel_leg_controller
ros2 topic hz /actuator_cmd     # 应有稳定高频
ros2 topic echo /joint_states --once
```

**阶段 1 的验收现象**：终端 B 打印关节顺序与帧计数；查看器里两轮转动、车前进后摔倒（预期）；`/actuator_cmd` 有频率。

---

## 五、阶段 2 起的计划与参考位置

| 阶段 | 要做的事 | 参考 |
|---|---|---|
| 2 轮速 PI | 用 `/joint_states` 轮速对 `/cmd_vel.linear.x` 做 PI，输出两个轮力矩 | `Wheel_Leg/LQR_VMC/robot_control_main.py` 第 16–20 行 `PIDController` |
| 3 VMC | 复制 `vmc_cal.py` 进本包；`forward_kinematics_cal()` 算 `L_0/theta`，腿长 PID → `F_0` → `VMC_torque_cal()` 出力矩 | `vmc_cal.py`、`robot_control_main.py` 的 `control_left_leg/control_right_leg` |
| 4 LQR | 复制 `lqr_update.py`；`states_update()` 更新状态，`calc_k_matrix_from_poly(L_0)` 取 K；轮力矩 = K[0],K[2],K[4],K[6],K[8],K[10] 六项加权 | `lqr_update.py`、`robot_control_main.py` 第 68–73 行 |
| 5 加分 | 平衡下跟踪 `/cmd_vel`；`VelocityRampFilter` 加速度限幅；抗扰；`/joy` 手柄 | `util/climb_traj.py`、教程 5.10 节 |
| 6 提交 | 包内 README（控制结构、K 来源、平衡时长、pitch 曲线）+ 录屏或 bag + 推自己的 fork | 教程 5.8 节 |

---

## 六、踩过的坑 / 必须警惕的点

1. **`x_b_set` 不能每步都更新**：只在有目标速度时跟随位移，站桩必须冻结。写错的现象是"车站得稳但匀速滑走"（实测能滑 8 m）。
2. **别同时跑 `wheel_leg_bridge`**：与 `wheel_leg_sim` 话题同名，会串台，表现为"代码没错但一直摔"。
3. **`vmc_cal.py` 里 `d_L_0`/`d_phi_0`/`d_alpha` 恒为 0** 是原版既有行为，照原样移植，不要"顺手修好"。
4. **pitch / gyro_pitch / roll 都要取负号**（`robot_control_main.py` 第 185–189 行），漏一个就会越控越倒。
5. **左右轮力矩异号**：模型两个轮子转轴方向相反，同向滚动需要异号力矩。
6. **改 .py 不用重新 build**（`--symlink-install`）；只有新增文件或改 `setup.py`/`package.xml` 后才需要 `colcon build --symlink-install --packages-up-to wheel_leg_controller`。
7. 项目路径不能含中文（`rosidl` 处理不了非 ASCII）。

---

## 七、环境速查

```bash
ros2on    # = conda activate rm_ros2
          #   + source /opt/ros/jazzy/setup.bash
          #   + cd ~/training/02-ros2-wheelleg/Tutorial/ros2_ws
          #   + source install/setup.bash
```

本机：Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12（教程写的是 22.04 / Humble / 3.10，命令里的发行版名要替换）。
