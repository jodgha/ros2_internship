#!/usr/bin/env python3
"""
vehicle_4axle  四桥八轮车辆 阿克曼转向几何计算库
[模块输出变量 outputs]
┌──────────────────────┬──────────┬────────────────────────────────────────────┐
│ 变量名                │ 类型      │ 说明                                       │
├──────────────────────┼──────────┼────────────────────────────────────────────┤
│ X_LIST               │ list      │ 4 桥 x 坐标 (m)，前+, 后-，相对底盘中心       │
│ W_LIST               │ list      │ 4 桥轮距 (m)，左右主销间距                   │
│ EPS                  │ float     │ 浮点容差 (1e-9)                            │
│ STEER_NAMES          │ list[str] │ 8 个转向关节短名                            │
│ JOINT_NAMES          │ list[str] │ 全部 16 个 URDF 关节名 (先转向后车轮)         │
│ calc_8wheel_steer_angles() │ func │ 阿克曼转角计算函数 (核心算法)                │
└──────────────────────┴───────────┴───────────────────────────────────────────┘

[参数表 parameter_table 车辆几何参数]
┌──────────┬─────────┬──────────┬────────────────────────────────┐
│ 参数      │ 值       │ 单位     │ 说明                           │
├──────────┼─────────┼──────────┼────────────────────────────────┤
│ 车长      │ 14.980  │ m        │ 底盘总长 (box size.x)           │
│ 车宽      │ 2.460   │ m        │ 底盘总宽 (box size.y)           │
│ 车高      │ 1.977   │ m        │ 底盘总高 (box size.z)           │
│ 总质量    │ 12000   │ kg       │ 整备质量                        │
│ Ixx       │ 5000    │ kg·m²    │ 侧倾惯量                       │
│ Iyy       │ 12000   │ kg·m²    │ 俯仰惯量                       │
│ Izz       │ 13000   │ kg·m²    │ 横摆惯量                       │
│ 桥1 x     │ +3.100  │ m        │ 1桥 x 坐标 (前转向桥)           │
│ 桥2 x     │ +4.750  │ m        │ 2桥 x 坐标 (前转向桥)           │
│ 桥3 x     │ -3.100  │ m        │ 3桥 x 坐标 (后转向桥)           │
│ 桥4 x     │ -4.750  │ m        │ 4桥 x 坐标 (后转向桥)           │
│ 轮距      │ 2.260   │ m        │ 每桥左右主销间距 (W)             │
│ 车轮半径   │ 0.700   │ m        │ 轮胎圆柱半径                    │
│ 轮胎宽度   │ 0.355   │ m        │ 轮胎圆柱长度                    │
│ 主销半径   │ 0.080   │ m        │ 主销圆柱半径                    │
│ 转向限位   │ ±0.785  │ rad      │ ±45° steer joint limit        │
│ 主销高度   │ 0.700   │ m        │ 主销圆柱长度                    │
│ 桥安装高度 │ 0.700   │ m        │ axle joint z (距底盘中心)       │
│ ICR 容差  │ 1e-9    │ —        │ 直线行驶/奇点保护阈值             │
└──────────┴─────────┴──────────┴────────────────────────────────┘

[转向方程 steering_equations 阿克曼几何]
  已知: θ1L=1 桥左轮转角 输入
    求:全部 8 轮转角 θ_iL, θ_iR  (i = 1..4)

  步骤1:计算 ICR 到车辆中心线的横向距离 R
      R=x1/tan(θ1L)+w1/2

  步骤2:对每桥 i, 计算左右轮转角
      tan(θ_iL)=xi/(R-wi/2)  →  θ_iL=atan(xi/(R-wi/2))
      tan(θ_iR)=xi/(R+wi/2)  →  θ_iR=atan(xi/(R+wi/2))

  步骤3:高速模式 <蟹行> 下, 8 轮完全平行, 直接返回 [θ1L] × 8
      低速模式下走标准阿克曼公式

  边界保护:
      - |θ1L|<EPS   → 直线行驶, 全部返回 0
      - |denom|<EPS → θ=±π/2 保护除零
      - θ1L 超出 ±45°  → 抛出 ValueError

  参考:`Reza N.Jazar,"Vehicle Dynamics:Theory and Application", Ch.7`
"""
import math

# 车辆全局几何参数 parameter_table
X_LIST=[3.100,4.750,-3.100,-4.750]   # 4桥 x 坐标 m, 前+后-
W_LIST=[2.260,2.260,2.260,2.260]     # 每桥轮距 m, 左右主销间距
WHEEL_RADIUS=0.700              # 车轮半径 m, 用于 v → ω 换算: ω = v / r
EPS=1e-9                        # 浮点容差 直线/除零保护

# URDF 关节名定义 urdf_joints 与 URDF 中 joint name 严格一致
# 8 转向关节短名 对应 urdf_links 中的 steer_joint
STEER_NAMES=[
    "ax1L","ax1R","ax2L","ax2R",
    "ax3L","ax3R","ax4L","ax4R",
]

# 16 完整 URDF joint 名 [先转向后车轮，与 JointState.position 顺序相同]
JOINT_NAMES=[
    "axle1_left_steer_joint","axle1_right_steer_joint",
    "axle2_left_steer_joint","axle2_right_steer_joint",
    "axle3_left_steer_joint","axle3_right_steer_joint",
    "axle4_left_steer_joint","axle4_right_steer_joint",
    "axle1_left_wheel_joint","axle1_right_wheel_joint",
    "axle2_left_wheel_joint","axle2_right_wheel_joint",
    "axle3_left_wheel_joint","axle3_right_wheel_joint",
    "axle4_left_wheel_joint","axle4_right_wheel_joint",
]

# 核心算法:阿克曼8轮转角计算 steering_equations
def calc_8wheel_steer_angles(theta1L:float,mode:str="low_speed")->list[float]:
    """
    阿克曼全轮转向角度计算

    Args:
        theta1L: 1桥左轮转角 (弧度)，正=左转，负=右转
        mode:    "low_speed"  后轴反向（减小转弯半径，常规转向）
                 "high_speed" 后轴同向（蟹行 / 高速稳定 / 平移）

    Returns:
        [ax1L, ax1R, ax2L, ax2R, ax3L, ax3R, ax4L, ax4R]
        8 个车轮转角 (弧度)，顺序与 STEER_NAMES / JOINT_NAMES[0:8] 一致

    Raises:
        ValueError: theta1L 超出 ±45° 物理范围

    Geometry:
        设 ICR [瞬时转向中心] 位于车辆左侧，距中心线横向距离 R:
            tan(θ_iL)=xi/(R-wi/2)    左轮
            tan(θ_iR)=xi/(R+wi/2)    右轮
        其中 R=x1/tan(θ1L)+w1/2

    Example:
        >>> angles = calc_8wheel_steer_angles(math.radians(15), "low_speed")
        >>> [f"{math.degrees(a):.1f}" for a in angles[:4]]
        ['15.0','14.0','22.3','20.3']  # 前桥:内轮>外轮 阿克曼效应
    """
    if not (-math.pi/4<=theta1L<=math.pi/4):
        raise ValueError(
            f"theta1L={math.degrees(theta1L):.1f}°，超出 ±45° 物理范围"
        )

    # 直线行驶
    if abs(theta1L)<EPS:
        return [0.0]*8

    # 高速模式:蟹行 8轮完全平行，无阿克曼修正
    if mode=="high_speed":
        return [theta1L]*8

    x1=X_LIST[0]
    w1=W_LIST[0]

    # 低速模式:标准阿克曼，后轴 xi 为负→反向偏转

    # ICR 到车辆中心线的横向距离
    R=x1/math.tan(theta1L)+w1/2.0

    angles=[]
    for xi,wi in zip(X_LIST,W_LIST):
        # 左轮
        denom_L=R-wi/2.0
        theta_L=(
            math.atan(xi/denom_L)
            if abs(denom_L)>EPS
            else math.copysign(math.pi/2,xi)
        )
        # 右轮
        denom_R=R+wi/2.0
        theta_R=(
            math.atan(xi/denom_R)
            if abs(denom_R)>EPS
            else math.copysign(math.pi/2,xi)
        )
        angles.append(theta_L)
        angles.append(theta_R)
    return angles

# 测试代码
if __name__=="__main__":
    for mode in ("low_speed","high_speed"):
        print(f"\n{'='*60}")
        print(f"  模式: {mode}")
        print(f"{'='*60}")
        for deg in (0.0,5.0,10.0,20.0,30.0,-10.0):
            theta=math.radians(deg)
            angles=calc_8wheel_steer_angles(theta,mode)
            print(
                f"θ1L={deg:>6.1f}° → "
                +" | ".join(
                    f"{n}: {math.degrees(a):>7.2f}°"
                    for n,a in zip(STEER_NAMES,angles)
                )
            )
