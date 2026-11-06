个人声明：这个readme，code主体部分由AI生成，我只能基于我对作业的理解给出自己对于AI代码大框架的理解。
AI为workbuddy，在之前和它一起完成前几章时完成skill的蒸馏。
skill蒸馏方面关于AI思考：给出问题概述，检查缺失部分，提供大方向思路，拆分细节步骤，给出验收标准，有问题先求指示的习惯。
三步骤：
1.初步代码：基于飞书要求，要求AI写一个控制程序（参考提示，阅读readme)，来解决小车运行左右轮协同问题。
2.改进：小车重心偏高，开车时会摔倒，让AI写程序：小车先降低重心，再开车
3.小车无法爬坡，AI解决方案是自己做了一个坡（因为原来代码是台阶，小车爬不上去）

个人收获：主要是如何与AI沟通
1.特别是作为一个面对全新领域的学生，如何利用AI提高效率，快速抓住重点，对问题进行切割与圈定，把一个一个node抛给AI完成。
2.重点在于如何把问题定义清楚，让AI看懂，抓住重点，简要描述

通过网盘分享的文件：ros小车爬坡.mp4
链接: https://pan.baidu.com/s/1YlyripiCexewzPEAf6GpXw 提取码: ujyq


# 轮腿机器人平衡控制器（wheel_leg_controller）

第 09 章大作业的交付包：把 LQR / VMC 的解算放进一个**独立的 ROS2 节点**，
通过话题驱动 `wheel_leg_sim` 里的 MuJoCo 仿真，让轮腿小车原地站住、还能边走边上坡。

---

## 一、这个包到底干了啥（一句话版）

> 仿真器负责"当身体"，我写的这个节点负责"当小脑"：
> 它听身体汇报姿态和关节角度，每 2 毫秒算一次该给 6 个电机多大的力，再发回去。

身体和小脑之间**只靠话题说话**，没有任何进程内共享变量，两个节点各自 `ros2 run` 独立启动。

---

## 二、怎么跑（三个终端，每个都先 `ros2on`）

`ros2on` 是本机配好的一个函数，等价于"激活 conda 环境 + 加载 ROS2 + 进工作区 + 登记包"。

**终端 A：先起仿真器**
```bash
ros2on
ros2 run wheel_leg_sim wheel_leg_sim
```
会弹出 MuJoCo 窗口。注意：**没人给它指令它就不动**，这是设计（见下面第五节第 1 条）。

**终端 B：再起控制器**（这个终端全程别关）
```bash
ros2on
ros2 run wheel_leg_controller wheel_leg_controller
```
看到下面两行说明起来了，接着小车会自己站起来：
```
平衡控制器已启动：target_length=0.1 m, wheel_radius=0.06 m
订阅: /imu, /joint_states, /cmd_vel  |  发布: /actuator_cmd
```

**终端 C：想让它走，就给它目标速度**
```bash
ros2on
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'
```
一直刷不要停，车就会边走边平衡；按 `Ctrl+C` 它会自己减速停下。

**可调参数（不改代码）**
```bash
# 想让车站高一点：腿长 0.10~0.14 之间
ros2 run wheel_leg_controller wheel_leg_controller --ros-args -p target_length:=0.12
```

---

## 三、它跟谁说话（接口）

| 方向 | 话题 | 消息类型 | 内容 |
|---|---|---|---|
| 听 | `/imu` | `sensor_msgs/Imu` | 机身姿态四元数、三轴角速度 |
| 听 | `/joint_states` | `sensor_msgs/JointState` | 6 个关节的角度和速度（顺序：right1, right2, left1, left2, rightwheel, leftwheel） |
| 听 | `/cmd_vel` | `geometry_msgs/Twist` | 目标前进速度，只用 `linear.x` |
| 说 | `/actuator_cmd` | `rm_msgs/WheelLegCmd` | 6 个执行器力矩（腿 ±6 N·m，轮 ±2 N·m） |

`WheelLegCmd` 的 6 个字段必须按这个顺序填，错一格就会打到别的电机：

```
right_phi1 → ctrl[0]（右腿近端）    right_phi4 → ctrl[1]（右腿远端）
left_phi1  → ctrl[2]（左腿近端）    left_phi4  → ctrl[3]（左腿远端）
right_wheel→ ctrl[4]（右轮）        left_wheel → ctrl[5]（左轮）
```

---

## 四、它每 2 毫秒在想什么（六个步骤）

**前提**：仿真器是"指令驱动"的——收到一条 `/actuator_cmd` 才往前走一个 `dt = 0.002 s`，
然后回发一次状态。所以我的控制循环写在"收到状态"的回调里：**收到状态 = 上一步刚结束**，
此刻算完就发下一条，一收一发正好咬合。

每个周期依次做六件事：

1. **算姿态**：四元数换算成俯仰角 pitch；再取角速度。注意原工程里 pitch 和它的角速度都**取了负号**，漏掉就会"越控越倒"。
2. **算整车运动**：左右轮的位置/速度取平均（左减右再除 2），得到整车前进距离和速度。左右轮转轴方向相反，减号正好把方向对齐。
3. **算腿长**：用两个髋关节角做五连杆正运动学，得到"虚拟腿长 L_0"和"摆杆相对机身的夹角 theta"。
4. **管腿长**：把 L_0 跟目标腿长做 PID，再加一个重力前馈，得到腿要伸多大力 F_0。
5. **算平衡**：LQR 用 12 个增益（K 矩阵）把"摆杆角、摆杆角速度、位置误差、速度误差、机身俯仰、俯仰角速度"加权成两组量——奇数下标给腿（T_p），偶数下标给轮（wheel_T）。
6. **发指令**：把 VMC 映射出的腿力矩、LQR 算出的轮力矩，打包成一条消息发出去。

补偿/保护：
- **K 矩阵随腿长变**：用 12 组多项式系数按当前 L_0 现算（腿伸长后动力学变了，增益也得跟着变）。
- **站桩冻结位置环**：只有当目标速度不为 0 时，才把"目标位置"更新成当前位置。否则位置误差恒为 0，位置环失效——现象是"车站得稳但匀速滑走"。
- **速度爬坡**：目标速度不是一步跳到 0.3，而是按 0.8 m/s²（减速 1.4）慢慢变，避免一下把车顶翻。
- **倒地保护**：机身倾斜超过 60° 就认为已经翻了，直接输出零力矩并提示去 Reset，免得躺在地上乱使劲。
- **异常隔离**：控制计算里任何异常都不许把节点打死，捕获后打日志、发零力矩、继续跑。

---

## 五、参数都是从哪来的

| 参数 | 取值 | 来源 |
|---|---|---|
| `dt = 0.002 s` | 固定 | `robot.xml` 的 `<option timestep>`，LQR 的 K 就是按这个整定的 |
| 腿力矩限幅 ±6.0、轮 ±2.0 | 固定 | `robot.xml` 里 `<motor ctrlrange>` |
| 轮半径 0.06 m | 固定 | `robot.xml` 里轮子几何 |
| 腿长 PID 700 / 0 / 5，输出限幅 150 N | 固定 | 原工程 `robot_control_main.py` |
| LQR 的 12 组多项式系数 | 固定 | 原工程 `lqr_update.py`，MATLAB 离线整定后拟合的 |
| 目标腿长 0.10 m | **可调**（0.10~0.14） | 原工程手柄调腿长的范围 |
| 加速度 0.8 / 减速 1.4 m/s² | **可调** | 原工程 `util/climb_traj.py` 的爬坡滤波器 |

算法文件 `vmc_cal.py`、`lqr_update.py` 是**从 `Wheel_Leg/LQR_VMC/` 原样复制**进来的，一个字符没改。
本次作业考的是"把控制回路拆成两个 ROS2 节点"，控制算法本身直接复用。

---

## 六、我的实测结果

| 项目 | 结果 |
|---|---|
| 原地平衡时长 | 连续 **84 s**（42000 步 × 0.002 s）未倒，之后仍在继续（要求 ≥5 s） |
| 平衡时 pitch 偏差 | **-0.11°**（稳定值时日志读数，要求 <10°） |
| 腿长 L_0 稳定值 | 0.0985 m（目标 0.10，被车重压下去 1.5 mm，正常） |
| 带速度前进 | 【待填】m/s，走了【待填】m 不摔 |
| 上坡（自制 3° 缓坡） | 【待填：能/不能】 |

**录屏**：通过网盘分享的文件：ros小车爬坡.mp4
链接: https://pan.baidu.com/s/1YlyripiCexewzPEAf6GpXw 提取码: ujyq

> 测法：终端 B 每 2000 步（= 4 s 仿真时间）打印一行
> `steps=... pitch=...° L0_R=... L0_L=... v_set=... wheel_T(R/L)=...`
> 也可以用 `ros2 bag record /imu /joint_states /actuator_cmd` 录下来事后画 pitch 曲线。

---

## 七、踩过的坑（都在这了）

1. **节点会"悄悄死掉"**：车翻倒后角度超出正常范围，数学函数抛异常把节点打死 → `/cmd_vel` 订阅数变成 0、车突然不动。已在回调整体套 `try/except`，异常只打日志不退出。
2. **`x_b_set` 不能每步都写**：写了位置误差恒为 0，车会站得很稳但匀速往前滑走（实测能滑 8 米）。只能"运动时跟随、站桩冻结"。
3. **左右轮力矩必须异号**：模型两个轮子转轴方向相反，同号会让车原地打转。
4. **pitch 和 gyro_pitch 都要取负号**（原工程如此），漏一个就越控越倒。
5. **别同时开 `wheel_leg_bridge`**：它和 `wheel_leg_sim` 用的是同名话题，两个发布者会让控制器收到交错的状态，必然失稳。用 `ros2 node list` 确认只有 sim 和控制器。
6. **官方地形其实是台阶不是坡**：`obstacles.xml` 里 5 块 9° 板各自绕自身中心旋转，接缝落差约 15.6 cm，而轮子直径只有 12 cm，靠滚动上不去。练爬坡我用的是自制的连续 3° 缓坡（见下）。

---

## 八、文件清单

```
wheel_leg_controller/
├── package.xml            # 依赖：rclpy / sensor_msgs / geometry_msgs / rm_msgs
├── setup.py               # 入口：wheel_leg_controller = wheel_leg_controller.wheel_leg_controller:main
├── setup.cfg
├── README.md              # 本文件
├── PROGRESS.md            # 做作业的过程记录（阶段、调试日志）
└── wheel_leg_controller/
    ├── __init__.py
    ├── wheel_leg_controller.py   # 控制节点本体（收状态、算力矩、发指令）
    ├── vmc_cal.py                # 五连杆正运动学 + VMC 力矩映射（原样复制）
    └── lqr_update.py             # LQR 状态更新 + K 矩阵多项式（原样复制）
```

### 附：自制缓坡（练爬坡用，不属于仓库）

```
~/my_sim/
├── robot.xml          # 从 wheel_leg_sim/robots/ 复制的模型
└── obstacles.xml      # 换成了连续 3° 缓坡：长 2 m，高差约 10.5 cm，无台阶
```

用法：
```bash
ros2 run wheel_leg_sim wheel_leg_sim --ros-args -p robot_xml:=$HOME/my_sim/robot.xml
```
> 两个 xml 必须在同一目录（`robot.xml` 里用相对路径 `<include file="obstacles.xml"/>`）。
> 仓库里的 `wheel_leg_sim` 一个文件都没改。
