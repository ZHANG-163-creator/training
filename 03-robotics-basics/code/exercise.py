"""基础作业完成前两个函数，进阶作业完成第三个函数。距离 m，角度 rad。"""
import math
from robotics_demo import rotate2d


def optical_to_link(point_optical):
    """理想共原点安装：光学 X右Y下Z前 → link X前Y左Z上。"""
    # 基础 1：根据前、左、上对应的光学分量，返回三个坐标。
    x_o=point_optical[0]
    y_o=point_optical[1]
    z_o=point_optical[2]
    return(z_o,-x_o,-y_o)

def aim_angles(point_reference):
    """输入固定瞄准参考系中的三维点，返回 (yaw, elevation)，均为 rad。

    参考系 X前Y左Z上，理想转轴共点。elevation 向上为正。
    如果输入有非有限值，或水平距离 <= 1e-12 m，应抛出 ValueError。
    """
    # 基础 2：使用 isfinite、hypot 和 atan2，实现上述约定。
    x=point_reference[0]
    y=point_reference[1]
    z=point_reference[2]

    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        raise ValueError("请输入含有非有限值")

    h=math.hypot(x,y);
    if h <= 1e-12:
        raise ValueError("水平距离过小")

    yaw=math.atan2(y,x)
    elevation=math.atan2(z,h)

    return(yaw,elevation)

def small_yaw_to_base(point_small, big_yaw, small_yaw, small_offset, big_offset):
    """将小 yaw 中的三维点换到底盘中，返回 (x, y, z)。

    两根 yaw 轴均竖直，角度均为相对父坐标系的角度，向左为正。
    small_offset：小 yaw 原点在大 yaw 平台 L 中的位置，用 L 的轴表达。
    big_offset：大 yaw 原点在底盘 B 中的位置，用 B 的轴表达。
    p_L = Rz(small_yaw) p_S + small_offset
    p_B = Rz(big_yaw) p_L + big_offset
    """
    # 进阶：调用两次 rotate2d，分别得到 L、B 中的水平分量。
    # 每次旋转后加该段偏移，Z 只累加对应的高度偏移。
    
    #分别输入
    x_s=point_small[0]
    y_s=point_small[1]
    z_s=point_small[2]

    #s转l
    x_l,y_l=rotate2d(x_s,y_s,small_yaw)
    x_l = x_l + small_offset[0]
    y_l = y_l + small_offset[1]
    z_l = z_s + small_offset[2]

    #l转b
    x_b,y_b=rotate2d(x_l,y_l,big_yaw)
    x_b = x_b + big_offset[0]
    y_b = y_b + big_offset[1]
    z_b = z_l + big_offset[2]

    return (x_b, y_b, z_b)


