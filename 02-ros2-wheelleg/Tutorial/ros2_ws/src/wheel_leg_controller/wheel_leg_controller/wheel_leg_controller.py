#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 09 章大作业：轮腿机器人平衡控制器（VMC + LQR）

数据流（只有一个闭环，所有计算都由 /joint_states 回调驱动）：

    wheel_leg_sim ──/imu──────────▶ 本节点
                  ──/joint_states─▶ （收一帧 → 算一次 → 发一条指令）
                  ◀─/actuator_cmd─
    外部          ──/cmd_vel──────▶ 目标速度（可选）

为什么控制循环写在 on_joints 里：
    仿真器是指令驱动的（wheel_leg_sim.py 第 208 行 on_cmd）：收到一条 /actuator_cmd
    才 mj_step 一个 dt=0.002 s，然后回发一次状态。所以"收到状态 = 上一步刚结束"，
    此刻算力矩并发下一条，一收一发严格咬合，dt 恒等于 0.002，与控制律整定一致。

每个控制周期做 6 件事（对应下面 control_leg 的 6 个编号）：
    ① 姿态：四元数 → pitch；角速度 → gyro_pitch / gyro_yaw（注意原工程取负号）
    ② 共模：左右轮位置/速度求平均 → 整车位移与速度
    ③ 五连杆正运动学：腿关节角 → 虚拟腿长 L_0、摆杆角 theta
    ④ 腿长 PID → 虚拟力 F_0；LQR 的 T_p → 髋关节力矩（VMC 雅可比映射）
    ⑤ LQR 的 wheel_T → 轮力矩（左右腿各算一次，符号相反）
    ⑥ 6 个力矩打包成 WheelLegCmd 发出

算法来源（未修改，原样复制进本包的 vmc_cal.py / lqr_update.py）：
    Wheel_Leg/LQR_VMC/vmc_cal.py        五连杆正运动学 + VMC 力矩映射
    Wheel_Leg/LQR_VMC/lqr_update.py     LQR 状态更新 + 按腿长插值的 K 矩阵
    公式组织方式参考 Wheel_Leg/LQR_VMC/robot_control_main.py 的
        control_left_leg / control_right_leg 两个函数

与参考版相比，这里**去掉**了不适用于仿真的部分（都在原文件里，需要时自己加回来）：
    · 离地检测 ground_detection（原版要按 Enter 才启用，是实机保护）
    · 手柄遥控 / roll 平衡（原版用 RollLegController 调左右腿长，这里左右同目标）
        → 想做加分项时，在 __init__ 里加 left/right 两个 target_length 即可
"""

# ---------------------------------------------------------------------------
# 导入
#   math / numpy   数学运算（vmc_cal、lqr_update 内部也用 numpy）
#   rclpy, Node    ROS2 客户端库与节点基类
#   Imu/JointState/Twist  标准消息
#   WheelLegCmd    自定义消息（rm_msgs）
#   five_links_param / lqr_controller  复制进来的算法类（相对导入，前面那个点别丢）
# ---------------------------------------------------------------------------
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from rm_msgs.msg import WheelLegCmd

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller

# ---------------------------------------------------------------------------
# 常量（与 mjcf/robot.xml、原工程保持一致，别随手改）
# ---------------------------------------------------------------------------
LEG_TORQUE_LIMIT = 6.0      # 消息层面的腿关节力矩上限 (N·m)
WHEEL_TORQUE_LIMIT = 2.0    # 消息层面的轮电机力矩上限 (N·m)
WHEEL_T_WHEEL_CLIP = 1.5    # LQR 输出的轮力矩限幅（原版就是 ±1.5，比消息上限更紧）
DT = 0.002                  # 仿真步长，LQR 的 K 矩阵按它整定，不能改

# 加速度爬坡：摇杆/指令从 0 猛跳到 0.5 m/s 会瞬间把车顶翻，
# 所以目标速度要按斜率慢慢变（原版 VelocityRampFilter 用的是 0.8 / 1.4）
ACCEL_RATE = 0.8            # m/s²
DECEL_RATE = 1.4            # m/s²


def clamp(value, limit):
    """把 value 限制在 [-limit, +limit]。出口统一限幅，别在每个公式里各写一遍。"""
    return max(-limit, min(limit, value))


class PIDController:
    """原工程 robot_control_main.py 第 16-25 行的同一个 PID（照搬，含它的特点）。

    注意两点原版特征，别"优化"掉：
      1. 微分项用 (当前误差 - 上次误差)，**没有再除以 dt**，所以 kd 的量纲是"每步"；
         分母里也没有 dt，整定值是按 0.002 s 调出来的。
      2. 积分限幅 max_i，防止长时间累积把输出顶死。
    """

    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.feedforward = feedforward
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral += error
        self.integral = clamp(self.integral, self.max_i)
        derivative = error - self.prev_error
        self.prev_error = error
        output = (self.kp * error + self.ki * self.integral
                  + self.kd * derivative + self.feedforward)
        return clamp(output, self.max_out)


class WheelLegController(Node):

    def __init__(self):
        super().__init__('wheel_leg_controller')

        # ---------------- 可调参数（不用改代码，--ros-args -p 就能覆盖）----------
        self.declare_parameter('target_length', 0.10)   # 目标虚拟腿长 (m)，可用 0.10~0.14
        self.declare_parameter('wheel_radius', 0.06)    # 轮半径 (m)，来自 robot.xml
        self.target_length = float(self.get_parameter('target_length').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)

        # ---------------- 输入缓存 ----------------
        self.latest_imu = None          # sensor_msgs/Imu
        self.latest_joints = None       # sensor_msgs/JointState
        self.latest_cmd_vel = Twist()   # 目标速度，默认 0
        self.name_to_index = {}         # 关节名 -> 数组下标
        self.step_count = 0
        self.error_count = 0            # 控制计算抛异常的累计次数（异常不许打死节点）

        # ---------------- 控制状态 ----------------
        self.current_v = 0.0            # 爬坡后的当前目标速度（喂给 LQR 的 v_set）
        self.robot_is_airborne = False  # 预留：接离地检测时用

        # 左右各一套算法实例：它们内部都存历史量（last_phi_0、K...），共用会串
        self.five_right = five_links_param(dt=DT)
        self.five_left = five_links_param(dt=DT)
        self.lqr_right = lqr_controller(dt=DT)
        self.lqr_left = lqr_controller(dt=DT)

        # 腿长 PID：kp=700, kd=5，输出限幅 150 N（原版参数）
        self.l0_pid_right = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        self.l0_pid_left = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        # 偏航 PID：把 yaw 角速度拉回 0（不接手柄时就是"别自己转"）
        self.yaw_pid_right = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.yaw_pid_left = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)

        # ---------------- 话题接口 ----------------
        self.cmd_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info(
            f'平衡控制器已启动：target_length={self.target_length} m, '
            f'wheel_radius={self.wheel_radius} m')
        # 把名单打出来，方便和 ros2 topic info 对照排查"订阅数为 0"
        self.get_logger().info('订阅: /imu, /joint_states, /cmd_vel  |  发布: /actuator_cmd')

    # ===================================================================
    # 输入回调
    # ===================================================================
    def on_imu(self, msg):
        self.latest_imu = msg

    def on_cmd_vel(self, msg):
        self.latest_cmd_vel = msg

    def on_joints(self, msg):
        """主控制回路入口：异常防护 + 倒地保护；真正的计算在 _control_step 里。

        为什么要包这一层（这次踩的坑）：
          1. 车翻倒后 pitch / 腿长会跑到正常范围之外，数学函数一旦抛异常，
             回调里的异常会直接把整个节点打死 —— 现象就是"车突然不动，ros2 node list 里
             也没有它了，/cmd_vel 的订阅数变成 0"。
          2. 所以这里捕获异常 → 打日志 → 发零力矩 → 节点继续活着，你还能 Reset 重来。
          3. 倒地（|pitch| > 60°）时不再计算，直接发零力矩：躺在地上继续使劲只会乱跳。
        """
        self.latest_joints = msg
        self.step_count += 1
        try:
            self._control_step(msg)
        except Exception as exc:          # noqa: BLE001 —— 任何异常都不许打死节点
            self.error_count += 1
            if self.error_count <= 5 or self.error_count % 500 == 0:
                self.get_logger().error(
                    f'控制计算异常（第 {self.error_count} 次）: {exc!r} —— 已发零力矩，节点继续运行')
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def _control_step(self, msg):
        """真正的控制计算（原 on_joints 的内容）。"""
        # 第一帧建立"关节名 -> 下标"映射（顺序由 sim 决定，不硬编码下标）
        if not self.name_to_index:
            self.name_to_index = {name: i for i, name in enumerate(msg.name)}
            self.get_logger().info(f'关节顺序: {list(msg.name)}')

        # 姿态还没来时不能控制：先发全 0，等 /imu 到位
        if self.latest_imu is None:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # 倒地保护：|pitch| > 60° 认为已经翻了，算了也是垃圾值，直接零力矩并提示
        pitch_now, _, _ = self.get_attitude(self.latest_imu)
        if abs(pitch_now) > 1.05:                      # rad，≈60°
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if self.step_count % 2000 == 0:
                self.get_logger().warn(
                    f'机器人已倒地（pitch={math.degrees(pitch_now):+.1f}°），输出零力矩；'
                    f'请在 MuJoCo 查看器 Simulation 面板点 Reset 后重新开始')
            return

        # ---------------- ① 姿态：四元数 -> pitch；角速度 -> gyro_pitch / gyro_yaw ----
        pitch, gyro_pitch, gyro_yaw = self.get_attitude(self.latest_imu)

        # ---------------- ② 共模：左右轮取平均，代表"整车"----------------
        wheel_pos_right = self.joint_value(msg, 'rightwheel', 'position')
        wheel_pos_left = self.joint_value(msg, 'leftwheel', 'position')
        wheel_vel_right = self.joint_value(msg, 'rightwheel', 'velocity')
        wheel_vel_left = self.joint_value(msg, 'leftwheel', 'velocity')

        # 左减右再除以 2：模型里左右轮转轴方向相反，减号正好把方向对齐
        common_motor_pos = (wheel_pos_left - wheel_pos_right) / 2.0
        common_motor_speed = (wheel_vel_left - wheel_vel_right) / 2.0

        # ---------------- 目标速度：爬坡 + x_b_set 的更新规则 ----------------
        target_v = float(self.latest_cmd_vel.linear.x)
        self.current_v = self.ramp(self.current_v, target_v)

        # 【关键】x_b_set 只在"要运动"时跟随当前位移；站桩时必须冻结。
        # 每步都更新 → (x_b - x_b_set) 恒为 0 → 位置环失效 → 车站得稳但匀速滑走。
        if not (self.current_v == 0.0 and target_v == 0.0):
            self.lqr_left.x_b_set = common_motor_pos * self.wheel_radius
            self.lqr_right.x_b_set = common_motor_pos * self.wheel_radius

        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v

        # ---------------- ③④⑤ 左右腿各算一套 ----------------
        # 返回值顺序：髋关节 1 力矩、髋关节 2 力矩、轮力矩
        right_p1, right_p4, right_wheel = self.control_leg(
            leg_flag=0,                       # 0 = 右腿
            motor1=self.joint_value(msg, 'right1', 'position'),
            motor2=self.joint_value(msg, 'right2', 'position'),
            five=self.five_right, lqr=self.lqr_right,
            l0_pid=self.l0_pid_right, yaw_pid=self.yaw_pid_right,
            pitch=pitch, gyro_pitch=gyro_pitch, gyro_yaw=gyro_yaw,
            common_motor_pos=-common_motor_pos,      # 右腿传负值（原版如此）
            common_motor_speed=-common_motor_speed,
        )
        left_p1, left_p4, left_wheel = self.control_leg(
            leg_flag=1,                       # 1 = 左腿
            motor1=self.joint_value(msg, 'left1', 'position'),
            motor2=self.joint_value(msg, 'left2', 'position'),
            five=self.five_left, lqr=self.lqr_left,
            l0_pid=self.l0_pid_left, yaw_pid=self.yaw_pid_left,
            pitch=pitch, gyro_pitch=gyro_pitch, gyro_yaw=gyro_yaw,
            common_motor_pos=common_motor_pos,
            common_motor_speed=common_motor_speed,
        )

        # ---------------- ⑥ 打包发布 ----------------
        self.publish_cmd(
            right_phi1=right_p1, right_phi4=right_p4,
            left_phi1=left_p1, left_phi4=left_p4,
            right_wheel=right_wheel, left_wheel=left_wheel,
        )

        # 低频日志：每 2000 步（= 4 s 仿真时间）打一次，看平衡质量
        if self.step_count % 2000 == 0:
            self.get_logger().info(
                f'steps={self.step_count} pitch={math.degrees(pitch):+.2f}° '
                f'L0_R={self.five_right.L_0:.4f} L0_L={self.five_left.L_0:.4f} '
                f'v_set={self.current_v:.2f} wheel_T(R/L)={right_wheel:+.2f}/{left_wheel:+.2f} '
                f'cmd_vel_pubs={self.count_publishers("cmd_vel")}')

    # ===================================================================
    # 单腿控制：③ 正运动学 → ④ 腿长力 → ⑤ LQR 力矩
    # ===================================================================
    def control_leg(self, leg_flag, motor1, motor2, five, lqr, l0_pid, yaw_pid,
                    pitch, gyro_pitch, gyro_yaw, common_motor_pos, common_motor_speed):
        """返回 (髋关节1力矩, 髋关节2力矩, 轮力矩)。"""

        # ③ 五连杆正运动学：把两个髋关节角 + 机体姿态换算成
        #    L_0（虚拟腿长）、theta（摆杆相对机体夹角）、d_theta（角速度）
        five.forward_kinematics_cal(leg_flag, motor1, motor2, pitch, gyro_pitch)

        # LQR 状态更新（phi/d_phi = 机体 pitch 及其角速度，theta/d_theta 来自运动学）
        lqr.states_update(leg_flag, five, pitch, gyro_pitch,
                          common_motor_pos, common_motor_speed, self.wheel_radius)

        # K 矩阵随腿长变化：多项式拟合，每次用当前 L_0 重算 12 个增益
        lqr.calc_k_matrix_from_poly(five.L_0)
        K = lqr.K

        # ④ 虚拟力 F_0 = 重力前馈 + 腿长 PID
        #    4 / cos(theta) 是原版的经验前馈（机体重量的等效项），照抄别改
        cos_theta = math.cos(five.theta)
        feedforward = (4.0 / cos_theta) if abs(cos_theta) > 1e-6 else 0.0
        F_0 = feedforward + l0_pid.compute(five.L_0, self.target_length)
        F_0 = clamp(F_0, 100.0)

        # ⑤ LQR 力矩：奇数下标 K[1,3,5,7,9,11] → 髋关节（T_p）
        #               偶数下标 K[0,2,4,6,8,10] → 轮子（wheel_T）
        #    六项依次是：摆杆角、摆杆角速度、位置误差、速度误差、机体俯仰、俯仰角速度
        T_p = (K[1] * five.theta
               + K[3] * five.d_theta
               + K[5] * (lqr.x_b - lqr.x_b_set)
               + K[7] * (lqr.v_b_whole - lqr.v_set)
               + K[9] * lqr.phi
               + K[11] * lqr.d_phi)

        wheel_T = (K[0] * five.theta
                   + K[2] * five.d_theta
                   + K[4] * (lqr.x_b - lqr.x_b_set)
                   + K[6] * (lqr.v_b_whole - lqr.v_set)
                   + K[8] * lqr.phi
                   + K[10] * lqr.d_phi)

        # 偏航：不给目标转速时就是把 yaw 角速度拉回 0
        yaw_ctrl = yaw_pid.compute(gyro_yaw, 0.0)

        # VMC：把极坐标的 (F_0, T_p) 通过五连杆雅可比映射成两个髋关节力矩
        torque_0, torque_1 = five.VMC_torque_cal(F_0, T_p)

        if leg_flag == 1:      # 左腿：轮力矩取负、髋关节力矩取负（原版符号）
            wheel = clamp(-wheel_T - yaw_ctrl, WHEEL_T_WHEEL_CLIP)
            return -torque_0, -torque_1, wheel
        else:                  # 右腿：轮力矩取正、髋关节力矩取正
            wheel = clamp(wheel_T - yaw_ctrl, WHEEL_T_WHEEL_CLIP)
            return torque_0, torque_1, wheel

    # ===================================================================
    # 工具函数
    # ===================================================================
    def get_attitude(self, imu):
        """从 IMU 消息算 (pitch, gyro_pitch, gyro_yaw)。

        · 四元数：ROS 里是 (x, y, z, w)，而 robot_info_update.py 用的是
          MuJoCo 的 (w, x, y, z)，所以这里按分量名取，别按顺序拆。
        · 欧拉角公式与 robot_info_update.py 第 32-34 行完全一致。
        · 符号：原版 pitch、gyro_pitch 都取了负号（robot_control_main.py 第 189/193 行），
          漏掉会"越控越倒"。gyro_yaw 不取负（第 194 行）。
        """
        q = imu.orientation
        w, x, y, z = q.w, q.x, q.y, q.z

        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))          # 防 asin 出界
        pitch = -math.asin(t2)

        gyro_pitch = -imu.angular_velocity.y
        gyro_yaw = imu.angular_velocity.x
        return pitch, gyro_pitch, gyro_yaw

    def joint_value(self, msg, name, field='position'):
        """按关节名取 position / velocity；名字不存在返回 0.0。"""
        index = self.name_to_index.get(name)
        if index is None:
            return 0.0
        return getattr(msg, field)[index]

    @staticmethod
    def ramp(current, target):
        """速度爬坡：按 ACCEL_RATE / DECEL_RATE 逐步逼近目标，避免指令突跳。"""
        if target > current:
            return min(target, current + ACCEL_RATE * DT)
        return max(target, current - DECEL_RATE * DT)

    def publish_cmd(self, right_phi1, right_phi4, left_phi1, left_phi4,
                    right_wheel, left_wheel):
        """组装 WheelLegCmd 并发布（6 个字段顺序 = sim 的 ctrl[0..5]）。"""
        msg = WheelLegCmd()
        msg.right_phi1 = clamp(right_phi1, LEG_TORQUE_LIMIT)
        msg.right_phi4 = clamp(right_phi4, LEG_TORQUE_LIMIT)
        msg.left_phi1 = clamp(left_phi1, LEG_TORQUE_LIMIT)
        msg.left_phi4 = clamp(left_phi4, LEG_TORQUE_LIMIT)
        msg.right_wheel = clamp(right_wheel, WHEEL_TORQUE_LIMIT)
        msg.left_wheel = clamp(left_wheel, WHEEL_TORQUE_LIMIT)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C 时 rclpy 已经关过 context，再关一次会抛 RCLError，所以先判断
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
