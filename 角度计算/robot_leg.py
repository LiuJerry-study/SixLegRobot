"""
六足机器人 单腿 2自由度 运动学可视化（腿长同步固件 include/Config.h）
坐标原点 = 大腿根部舵机（髋关节）；y+ 竖直向上，x+ 为水平方向。
腿坐标系与固件/gait-sim 一致：x+ = 指向机体中心的内侧，x− = 外侧；
默认站姿脚在 −x 侧（外侧），小腿竖直、膝盖为整条腿最高点，侧看呈 Λ 拱。

角度约定（本工具用与 角度计算/try.cpp::ccl()、固件 src/LegKinematics.cpp::ik() 同一式的
旧口径 acos+atan2 解：入参 cm、结果弧度，只在显示时转度）：
  ag1 膝内角（两连杆夹角，恒为 0~180 正数）：180°=完全伸直、0°=完全折叠；
      几何弯曲角 θ = 180° − ag1（也就是小腿舵机角：0°=完全伸直、180°=完全折叠）
  ag2 大腿数学角（**外倾为正**，2026-09-10 起与固件统一）：0°=大腿指向 +y（竖直向上）；
      向 −x（髋外侧/外拱）为正、向 +x（体内）为负 —— 默认站姿大腿上拱外倾，
      故 ag2 = +67.38°（拱起角 22.62°；换算 3D 模拟口径 α = 90° − ag2 = +22.62°，
      相对水平外轴、向上为正）。
  IK 原始 ag2 允许超出 ±180°（acos 落在 0~180°，再加 atan2(x,y) 的 (−180°,180°]）：
  例如脚 (0.04, −18.93) 解出 ≈186.7°，与 −173.3° 是同一方向；本工具显示处一律经
  norm_deg() 折到 [−180°,180°)（+180° 归 −180°），与所画弧一致。

舵机角映射（**与固件 src/SingleLeg.cpp::writeServosFromRad() 完全一致**）：
  大腿舵机角 = ag2（度）  ← 【2026-09 实物实测】大腿舵机 0° = 大腿竖直向上、180° = 竖直向下，
                            中间扫过"朝外"（−x）那半边 ⇒ 逻辑角本身就是舵机角，不加任何常量
  小腿舵机角 = 180° − ag1（度）
  （某条腿整体朝向真的反了 = 机械安装/镜像装错，应改该腿的安装或单独做镜像，
    不要在映射里再加减 90° —— 实测那样会让大腿直接转到最下方。）
  默认站姿 (−6, −10) → 逻辑角 (67.38°, 112.62°)；Config.h::ANGLE_OFFSET 当前【整表为 0】
  （旧值是在错误的大腿映射下标定的，已作废、待按实物重新标定）⇒ 本工具显示的逻辑角
  = 实机最终写入角。将来偏移非 0 时，"先加偏移、再限幅 0~180"由固件 ServoManager::write
  完成，本工具不模拟、也不做 0~180 限幅。
  大腿舵机 0~180° 行程 ↔ ag2 ∈ [0°, 180°]（可用外拱区间 ag2 ∈ [0°, 90°]：
  站姿 67.38°、伸出脚 ≈ 83.06°）；ag2 < 0（大腿越过竖直向上、倒向体内侧，例如脚抬到
  髋关节高度以上）会被固件限幅到 0°（大腿竖直向上）→ 实际到不了，本工具对这种脚位
  标红"超行程"；绿扇区就是行程内的可行区域，落在扇区外的红脚即会被限幅截断的脚位。

⚠ 与固件当前代码的关系（2026-09-10 核对）：
  1) 固件 src/LegKinematics.cpp::ik() 返回的是【弧度】，与本文件 leg_ik() 完全一致
     （都是 −(acos + atan2(fx,fy))、cos 先夹逼 [−1,1]；d < 1e-9 时固件把两角置 0、
      本工具返回 None，含义都是"无法构角"）—— 两边是同一个解；
     src/SingleLeg.cpp::writeServosFromRad() 把弧度结果 ×180/π（只换算一次），
     不存在"重复换算"（旧注释里"返回度 / 重复换算"的说法已过时）。
  2) 口径 / 映射变更记录（越靠后越新，只有最后一条是现状）：
     - 2026-09-10 之前："体内侧为正"旧口径（站姿 −67.38°），固件映射写成 180 − ag2
       （站姿 247.38° → 被限幅到 180° 端点）—— 已作废。
     - 2026-09-10：口径统一为"外倾为正"（站姿 +67.38°），但大腿映射按"水平朝外为 0 点"
       多加了一个 90° 的常量（等于写成"大腿已转过水平、快指到最下面"的值）
       —— 【2026-09 实物实测证伪：上电后大腿直接转到最下方，站姿根本摆不出来】，
       配套标定出来的 Config.h 偏移同样作废。
     - 现状（实物实测确认）：大腿舵机角 = ag2（站姿 67.38°）、小腿 = 180 − ag1
       （站姿 112.62°），ANGLE_OFFSET 整表清零 ⇒ 最终写入角 = 逻辑角。
     ⚠ 只翻 atan2 的符号（acos + |atan2|、acos − atan2）是错的：acos 项只依赖距离 d，
       镜像下不变，翻 atan2 等于把解换成镜像脚位 (−fx, fy) —— 站姿会解到 (+6, −10)。
  3) 编译阻塞已解除（2026-09-10 实测 pio run 通过）：src/ServoManager.cpp 里那行
     const double PI = 3.14159265358979323846; 曾与 Arduino.h 的 #define PI 冲突，被宏展开成非法
     声明（expected unqualified-id before numeric constant），现已删除；别再往 include 了
     <Arduino.h> 的文件作用域写 const double PI。

画布：
- 灰色圆环 = 几何可达范围（内圆 |A-B| ~ 外圆 A+B）
- 绿色扇区 = 当前舵机行程（面板可调 0~180°）约束下的真实可行脚位：
  IK 算出但超出该区域的脚位，固件会被 ServoManager 限幅截断
- 紫色虚线 = 步态的两个固定足端目标（站姿脚 ↔ 伸出脚 2/3 分点）
- 腿长与固件 include/Config.h 同步：LENA=6.5cm（大腿）、LENB=12.5cm（小腿）
"""

import math
import tkinter as tk

# ============ 默认参数（腿长与 include/Config.h 同步；舵机行程是本工具面板默认值） ============
LENA = 6.5   # 大腿长 (cm)
LENB = 12.5  # 小腿长 (cm)
SERVO_MIN_DEG = 0.0    # 舵机行程下限（面板可改，模拟真实舵机如 30/150）
SERVO_MAX_DEG = 180.0  # 舵机行程上限
W = 960
H = 680

MIN_SCALE = 6.0
MAX_SCALE = 500.0

# 配色
C_THIGH = "#1971c2"   # 大腿
C_SHANK = "#2f9e44"   # 小腿
C_HIP   = "#e03131"   # 髋
C_KNEE  = "#f08c00"   # 膝
C_FOOT  = "#2b8a3e"   # 脚（可达）
C_FOOT_BAD = "#e03131"  # 脚（不可达/超舵机行程）
C_RING  = "#f1f3f5"   # 几何可达环底色
C_FAN   = "#d8f3dc"   # 舵机可行扇区
C_FAN_L = "#40c057"   # 扇区描边
C_GRID  = "#e9ecef"
C_AXIS  = "#ced4da"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------- 运动学（旧口径 acos+atan2：入参 cm、角度结果弧度；与 try.cpp/固件 ik() 同式，
#                  舵机映射见 servo_angles*()） ----------------

def leg_ik(a, b, fx, fy):
    """足端坐标(cm) -> (膝角 ag1, 大腿角 ag2, 髋足距 d)；ag1/ag2 单位为弧度。
    ag1 = acos((a²+b²−d²)/(2ab))（两连杆夹角，恒为正）：d = a+b（外圆、完全伸直）→ 180°、
          d = |a−b|（内圆、完全折叠）→ 0°；几何弯曲角 θ = 180° − ag1（= 小腿舵机角，
          默认站姿 ag1 = 67.38° → θ = 112.62°）
    ag2 = −( acos((a²+d²−b²)/(2ad)) + atan2(x, y) )，即大腿方向相对 +y 轴的角度，
          偏向 −x（髋外侧/外拱）为正 —— "外倾为正"口径，与固件 src/LegKinematics.cpp::ik() 同一式
          （= 角度计算/try.cpp::ccl() 的结果取反；原始值可超出 ±180°，显示处经 norm_deg() 折算）
    cos 已夹逼到 [−1,1]；d < 1e-9（脚落在髋原点）时返回 (None, None, d)"""
    d = math.sqrt(fx * fx + fy * fy)
    if d < 1e-9:
        return None, None, d
    cos1 = (a * a + b * b - d * d) / (2 * a * b)
    cos1 = clamp(cos1, -1, 1)
    ag1 = math.acos(cos1)
    cos2 = (a * a + d * d - b * b) / (2 * a * d)
    cos2 = clamp(cos2, -1, 1)
    ag2 = -(math.acos(cos2) + math.atan2(fx, fy))   # 取反 = "外倾为正"口径
    return ag1, ag2, d


def servo_angles(ag1, ag2):
    """数学角(弧度) -> 逻辑舵机角(度)：大腿 = ag2、小腿 = 180 − ag1
    （与固件 src/SingleLeg.cpp::writeServosFromRad() 完全一致的口径：
      2026-09 实物实测——大腿舵机 0° = 大腿竖直向上、180° = 竖直向下，中间扫过"朝外"
      那半边，所以大腿舵机角就是 ag2 本身，不加任何常量；站姿 = 67.38° / 112.62°）。
    本函数不含 ANGLE_OFFSET 零位偏移与 0~180 限幅（那两步在 ServoManager::write，
    且是"先加偏移、再限幅"；该表当前整表为 0 ⇒ 最终写入角 = 逻辑角）。
    ag2 未规范化，本函数只作对照，界面显示走 servo_angles_disp()。"""
    thigh_deg = ag2 * 180.0 / math.pi
    knee_deg = ag1 * 180.0 / math.pi
    servo_thigh = thigh_deg
    servo_knee = 180.0 - knee_deg
    return servo_thigh, servo_knee


def servo_angles_disp(ag1, ag2):
    """同 servo_angles，但大腿角先用规范化角(≡同方向, [−180°,180°))再映射，
    避免 ag2 原始值超范围时映射出无意义的舵机数。
    行程正区间 ag2 ∈ [0°, 180°] 内两者完全相同，映射结果不受影响。
    ⚠ 规范化会把"反折解"（IK 原始 ag2 < −180°、大腿-足端夹角接近 180° 的折叠姿态）也
      折进 [0°,180°) ⇒ 这类脚位在本工具里会显示成"行程内"，而固件是拿【原始角】直接限幅
      到端点的；绿扇区（feasible_d_range）按"原始角在行程内"画，两者在体内侧/反折区域
      可能不一致（该区域不是步态工作区，抬腿/迈步都用不到）。"""
    d2 = ((ag2 * 180.0 / math.pi) + 180.0) % 360.0 - 180.0
    return d2, 180.0 - ag1 * 180.0 / math.pi


def default_standing_foot(a, b):
    """默认站立足端：小腿竖直并与内圆（半径 |a-b|）负侧相切 ——
    膝 = (−r, √(a²−r²))，脚 = 膝正下方 b（小腿竖直）。
    LENA/LENB = 6.5/12.5 时：膝 (−6, 2.5)、脚 (−6, −10)（髋外侧 6cm、下方 10cm），
    此时 ag1 = ag2 = 67.38°（外倾为正口径），脚恰在内圆切线上。"""
    r = abs(a - b)
    kx = -r
    ky = math.sqrt(a * a - r * r)
    return kx, ky - b


def extended_foot(a, b):
    """伸出足端：站立高度上，站姿脚与外圆(A+B)同侧交点所成线段的 2/3 分点
    （取 2/3 而非 1/2：更靠外，迈步行程更大）。
    LENA/LENB = 6.5/12.5 时 → (−12.77, −10)。"""
    sx, sy = default_standing_foot(a, b)
    R = a + b
    xi = -math.sqrt(R * R - sy * sy)
    return sx + (xi - sx) * (2.0 / 3.0), sy


def d_of_beta(a, b, beta):
    """给定大腿-足端方向夹角 beta(rad, 0~pi)，解出髋足距 d（单调递减）。
    d = a·cosβ + sqrt(b² - a²·sin²β)；无解返回 None"""
    t = b * b - a * a * math.sin(beta) ** 2
    if t < 0:
        return None
    return a * math.cos(beta) + math.sqrt(t)


def feasible_d_range(a, b, psi_deg, s_min, s_max):
    """在舵机行程 [s_min, s_max]（度）约束下，求方向 psi_deg（足端方向，
    与 IK 相同从 +y 轴起算，度）上的可行髋足距区间 (d_lo, d_hi)
    （已与内圆 |a−b|、外圆 a+b 求交）；无解返回 None。

    映射（与固件一致，2026-09 实物实测）：舵机大腿 = ag2、舵机小腿 = 180 - ag1
      => 行程内要求 ag2 ∈ [s_min, s_max]，ag1 ∈ [180-s_max, 180-s_min]
    注：本函数内部的几何推导沿用"体内侧为正"的等效角 ag2_old = psi + beta（= −ag2），
        所以下面 ag2_lo/ag2_hi 取 [-s_max, -s_min]（即 −ag2 的行程区间）；
        绿扇区就是按这套行程约束画出的可行脚位 —— ag2 < 0（体内侧/反折）不在扇区内，
        与"面板按【原始角】判超行程"的固件口径一致。"""
    if s_min > s_max:
        return None
    deg2rad = math.pi / 180.0
    ag2_lo = -s_max * deg2rad
    ag2_hi = -s_min * deg2rad
    ag1_lo = clamp(180.0 - s_max, 0.0, 180.0) * deg2rad
    ag1_hi = clamp(180.0 - s_min, 0.0, 180.0) * deg2rad

    inner = abs(a - b)
    outer = a + b

    # 膝角 ag1 约束 -> d ∈ [d(ag1_lo), d(ag1_hi)]（d 随 ag1 单调增）
    d1_lo = math.sqrt(a * a + b * b - 2 * a * b * math.cos(ag1_lo))
    d1_hi = math.sqrt(a * a + b * b - 2 * a * b * math.cos(ag1_hi))

    # 大腿角约束：ag2 = psi + beta，beta ∈ [ag2_lo-psi, ag2_hi-psi] ∩ [0, pi]
    psi = psi_deg * deg2rad
    beta_lo = max(0.0, ag2_lo - psi)
    beta_hi = min(math.pi, ag2_hi - psi)
    if beta_lo > beta_hi:
        return None
    d_hi = d_of_beta(a, b, beta_lo)   # d 随 beta 单调递减
    d_lo = d_of_beta(a, b, beta_hi)
    if d_lo is None or d_hi is None:
        return None

    lo = max(inner, d1_lo, d_lo)
    hi = min(outer, d1_hi, d_hi)
    if lo > hi + 1e-9:   # 容差：容忍外圆切线等零宽度边界上的浮点误差
        return None
    return lo, hi


# ---------------- 界面 ----------------

class LegApp:
    def __init__(self, root):
        self.root = root
        root.title("六足机器人单腿运动学可视化（含舵机角度 / 可行范围）")
        self.a = LENA
        self.b = LENB
        self.s_min = SERVO_MIN_DEG
        self.s_max = SERVO_MAX_DEG
        self.scale = 40.0
        self.fx, self.fy = default_standing_foot(self.a, self.b)
        self.dragging = False
        self.first_fit = True

        # ===== 顶部：参数行 =====
        top = tk.Frame(root)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)
        tk.Label(top, text="大腿长A(cm):").pack(side=tk.LEFT)
        self.e_lena = tk.Entry(top, width=5)
        self.e_lena.insert(0, str(LENA))
        self.e_lena.pack(side=tk.LEFT)
        tk.Label(top, text="  小腿长B(cm):").pack(side=tk.LEFT)
        self.e_lenb = tk.Entry(top, width=5)
        self.e_lenb.insert(0, str(LENB))
        self.e_lenb.pack(side=tk.LEFT)
        tk.Label(top, text="  舵机行程 min(°):").pack(side=tk.LEFT)
        self.e_smin = tk.Entry(top, width=4)
        self.e_smin.insert(0, str(int(SERVO_MIN_DEG)))
        self.e_smin.pack(side=tk.LEFT)
        tk.Label(top, text="max(°):").pack(side=tk.LEFT)
        self.e_smax = tk.Entry(top, width=4)
        self.e_smax.insert(0, str(int(SERVO_MAX_DEG)))
        self.e_smax.pack(side=tk.LEFT)
        tk.Button(top, text="更新参数", command=self.update_params).pack(side=tk.LEFT, padx=8)

        # ===== 顶部：缩放行 =====
        zoom = tk.Frame(root)
        zoom.pack(side=tk.TOP, fill=tk.X, padx=8)
        tk.Button(zoom, text="放大 +", command=lambda: self.zoom(1.25)).pack(side=tk.LEFT)
        tk.Button(zoom, text="缩小 -", command=lambda: self.zoom(0.8)).pack(side=tk.LEFT)
        tk.Button(zoom, text="适配窗口", command=self.fit).pack(side=tk.LEFT, padx=2)
        self.lb_zoom = tk.Label(zoom, text="缩放: 100%", fg="darkblue", width=14, anchor="w")
        self.lb_zoom.pack(side=tk.LEFT, padx=6)
        self.lb_hint = tk.Label(zoom, text="拖拽=移动脚  滚轮=缩放", fg="#868e96")
        self.lb_hint.pack(side=tk.LEFT, padx=10)

        # ===== 画布 =====
        self.cw = W
        self.ch = H
        self.canvas = tk.Canvas(root, width=W, height=H, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # ===== 底部：角度信息行 =====
        info1 = tk.Frame(root)
        info1.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(6, 0))
        self.lb_ag2 = tk.Label(info1, text="大腿数学角 ag2: --", font=("", 12))
        self.lb_ag2.pack(side=tk.LEFT, padx=10)
        self.lb_ag1 = tk.Label(info1, text="膝角 ag1: --", font=("", 12))
        self.lb_ag1.pack(side=tk.LEFT, padx=10)
        self.lb_feet = tk.Label(info1, text="脚坐标: --", font=("", 12))
        self.lb_feet.pack(side=tk.LEFT, padx=10)
        self.lb_d = tk.Label(info1, text="髋足距: --", font=("", 12))
        self.lb_d.pack(side=tk.LEFT, padx=10)

        info2 = tk.Frame(root)
        info2.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(2, 4))
        self.lb_sv_t = tk.Label(info2, text="大腿舵机角: --", font=("", 13, "bold"))
        self.lb_sv_t.pack(side=tk.LEFT, padx=10)
        self.lb_sv_k = tk.Label(info2, text="小腿舵机角: --", font=("", 13, "bold"))
        self.lb_sv_k.pack(side=tk.LEFT, padx=10)
        self.lb_status = tk.Label(info2, text="", font=("", 13, "bold"))
        self.lb_status.pack(side=tk.LEFT, padx=14)

        # 事件
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        self.canvas.bind("<Button-4>", lambda e: self.zoom(1.1))
        self.canvas.bind("<Button-5>", lambda e: self.zoom(0.8))
        self.canvas.bind("<Configure>", self.on_resize)
        for e in (self.e_lena, self.e_lenb, self.e_smin, self.e_smax):
            e.bind("<Return>", lambda ev: self.update_params())

    # ---------- 坐标/缩放 ----------

    def ip(self, x, y):
        """数学坐标(cm) -> 画布像素"""
        ox, oy = self.cw / 2, self.ch / 2
        return ox + x * self.scale, oy - y * self.scale

    def zoom(self, factor):
        self.scale = clamp(self.scale * factor, MIN_SCALE, MAX_SCALE)
        self.lb_zoom.config(text="缩放: {0:.0f}%".format(self.scale / 40.0 * 100))
        self.draw()

    def fit(self):
        """让外圆整体可见"""
        R = self.a + self.b
        self.scale = clamp(0.46 * min(self.cw, self.ch) / max(R, 1.0),
                           MIN_SCALE, MAX_SCALE)
        self.lb_zoom.config(text="缩放: {0:.0f}%".format(self.scale / 40.0 * 100))
        self.draw()

    def on_resize(self, event):
        self.cw = max(event.width, 50)
        self.ch = max(event.height, 50)
        if self.first_fit:
            self.first_fit = False
            self.fit()
        else:
            self.draw()

    def on_wheel(self, event):
        self.zoom(1.12 if event.delta > 0 else 1 / 1.12)
        return "break"

    def on_press(self, event):
        self.dragging = True
        self.set_foot_from_event(event)

    def on_drag(self, event):
        if self.dragging:
            self.set_foot_from_event(event)

    def on_release(self, event):
        self.dragging = False

    def set_foot_from_event(self, event):
        ox, oy = self.cw / 2, self.ch / 2
        self.fx = (event.x - ox) / self.scale
        self.fy = -(event.y - oy) / self.scale
        self.draw()

    def update_params(self):
        try:
            a = float(self.e_lena.get())
            b = float(self.e_lenb.get())
            s_min = float(self.e_smin.get())
            s_max = float(self.e_smax.get())
            if a <= 0 or b <= 0:
                raise ValueError
            if s_min > s_max:
                s_min, s_max = s_max, s_min  # 自动纠正
            self.a, self.b = a, b
            self.s_min, self.s_max = s_min, s_max
            # 当前脚若落在新范围外，回到新站姿脚
            d = math.sqrt(self.fx * self.fx + self.fy * self.fy)
            if d < abs(a - b) - 1e-6 or d > a + b + 1e-6 or d < 1e-9:
                self.fx, self.fy = default_standing_foot(a, b)
            self.lb_status.config(text="", fg="black")
            self.fit()
        except ValueError:
            self.lb_status.config(text="参数错误：请输入有效数字", fg="red")

    # ---------- 绘制辅助 ----------

    @staticmethod
    def tk_ang(mx, my):
        """数学向量(mx,my) -> Tk 画布角度(0°=3点钟, 逆时针为正, 0~360)"""
        return math.degrees(math.atan2(my, mx)) % 360.0

    @staticmethod
    def norm_deg(deg):
        """同方向角规范化到 [−180°, 180°)（+180° 会被折成 −180°，仍是同一方向）：
        如 251° ≡ −109°、190° ≡ −170°、180° ≡ −180°。
        只影响显示口径，不影响 IK/舵机内部计算。"""
        return ((deg + 180.0) % 360.0) - 180.0

    def arc_between(self, cx, cy, v1x, v1y, v2x, v2y, r_px, color, width=2):
        """在数学点(cx,cy)处画 v1->v2 的劣弧，返回弧中点角度(画布角)"""
        a1 = self.tk_ang(v1x, v1y)
        a2 = self.tk_ang(v2x, v2y)
        sweep = (a2 - a1) % 360.0
        if sweep > 180.0:
            start, ext = a2, 360.0 - sweep
        else:
            start, ext = a1, sweep
        if abs(ext) < 0.5:
            return None
        px, py = self.ip(cx, cy)
        self.canvas.create_arc(px - r_px, py - r_px, px + r_px, py + r_px,
                               start=start, extent=ext, style=tk.ARC,
                               outline=color, width=width)
        return start + ext / 2.0

    def arc_label(self, cx, cy, mid_ang, r_px, text, color):
        """沿弧外沿放角度文本（mid_ang 为画布角），白底描边防与线段重叠。
        注意：画布角 a 的像素方向是 (cos a, −sin a)，纵轴符号写反会把文字镜像到关节另一侧。"""
        if mid_ang is None:
            return
        # 弧中点即角区的外沿方向；画布角 a 的像素方向 = (cos a, -sin a)
        k = (r_px + 16) / self.scale
        rad = math.radians(mid_ang)
        mx = cx + math.cos(rad) * k
        my = cy + math.sin(rad) * k   # ip() 会翻转 y，最终像素偏移为 (cos, -sin)
        px, py = self.ip(mx, my)
        c = self.canvas
        t = c.create_text(px, py, text=text, fill=color, font=("", 9))
        x1, y1, x2, y2 = c.bbox(t)
        c.create_rectangle(x1 - 3, y1 - 2, x2 + 3, y2 + 2,
                           fill="#ffffff", outline="#ced4da")
        c.tag_raise(t)

    # ---------- 主绘制 ----------

    def draw(self):
        c = self.canvas
        a, b = self.a, self.b
        smin, smax = self.s_min, self.s_max
        inner = abs(a - b)
        outer = a + b
        c.delete("all")
        ox, oy = self.cw / 2, self.ch / 2

        # ---- 参考网格（每 5cm 一圈 + 中心十字）----
        for r_cm in range(5, int(outer) + 1, 5):
            rp = r_cm * self.scale
            c.create_oval(ox - rp, oy - rp, ox + rp, oy + rp,
                          outline=C_GRID, width=1)
        c.create_line(0, oy, self.cw, oy, fill=C_AXIS, width=1)
        c.create_line(ox, 0, ox, self.ch, fill=C_AXIS, width=1)
        # 轴方向标注（帮助理解角度符号：x+ = 体内侧，x− = 外侧）
        c.create_text(self.cw - 8, oy - 8, text="x+ (体内侧)",
                      anchor="e", fill="#adb5bd", font=("", 8))
        c.create_text(8, oy - 8, text="x− (外侧)",
                      anchor="w", fill="#adb5bd", font=("", 8))
        c.create_text(ox + 6, 10, text="y+ (上)", anchor="w",
                      fill="#adb5bd", font=("", 8))
        c.create_text(ox + 6, self.ch - 10, text="y− (地面/下)", anchor="w",
                      fill="#adb5bd", font=("", 8))

        # ---- 几何可达范围：灰色圆环（内圆 ~ 外圆）----
        for r_cm, w, dash in ((outer, 2, ()), (inner, 1, (4, 4))):
            rp = r_cm * self.scale
            c.create_oval(ox - rp, oy - rp, ox + rp, oy + rp,
                          outline="#adb5bd", width=w, dash=dash)
        # 内圆内部填白（环带之外不可达）
        rp = inner * self.scale
        c.create_oval(ox - rp, oy - rp, ox + rp, oy + rp,
                      fill="white", outline="")
        # 环带浅灰底
        rp = outer * self.scale
        c.create_oval(ox - rp, oy - rp, ox + rp, oy + rp,
                      fill=C_RING, outline="")
        # 内圆内部重新填白（覆盖环带填充）
        rp = inner * self.scale
        c.create_oval(ox - rp, oy - rp, ox + rp, oy + rp,
                      fill="white", outline="")

        # ---- 舵机行程约束下的可行扇区（绿色）----
        fan_polys = []
        run = []
        n = 720  # 0.5°/步
        for i in range(n):
            psi = -180.0 + i * (360.0 / n)
            res = feasible_d_range(a, b, psi, smin, smax)
            if res is None:
                if run:
                    fan_polys.append(run)
                    run = []
                continue
            lo, hi = res
            run.append((psi, lo, hi))
        if run:
            fan_polys.append(run)

        for seg in fan_polys:
            pts = []
            for psi, lo, hi in seg:
                ra = math.radians(psi)
                for d in (lo, hi):
                    px = ox + d * math.sin(ra) * self.scale
                    py = oy - d * math.cos(ra) * self.scale
                    pts.append((px, py))
            for psi, lo, hi in reversed(seg):
                ra = math.radians(psi)
                px = ox + lo * math.sin(ra) * self.scale
                py = oy - lo * math.cos(ra) * self.scale
                pts.append((px, py))
            if len(pts) >= 6:
                flat = [v for p in pts for v in p]
                c.create_polygon(flat, fill=C_FAN, outline=C_FAN_L, width=1)

        # ---- 步态两个固定足端目标（站姿脚 ↔ 伸出脚）----
        sx, sy = default_standing_foot(a, b)
        ex, ey = extended_foot(a, b)
        p1 = self.ip(sx, sy)
        p2 = self.ip(ex, ey)
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#9c36b5",
                      width=2, dash=(4, 3))
        for (px, py), label in ((p1, "站姿脚"), (p2, "伸出脚")):
            c.create_rectangle(px - 3, py - 3, px + 3, py + 3,
                               outline="#9c36b5", width=2)
            c.create_text(px + 8, py + 12, text=label, fill="#9c36b5", font=("", 8))

        # ---- 当前脚 & 腿形 ----
        fx, fy = self.fx, self.fy
        d = math.sqrt(fx * fx + fy * fy)
        ag1, ag2, d = leg_ik(a, b, fx, fy)
        ok_geo = (inner - 1e-6 <= d <= outer + 1e-6) and d > 1e-9
        # 超行程判定用【做对照的原始角】servo_angles()，而不是显示用的规范化角：
        # 固件 src/SingleLeg.cpp 是拿 IK 原始角（不规范化）直接交 ServoManager 限幅 0~180 的，
        # 所以"原始角在行程内"才是"固件真能摆出来"的判据，也才和绿扇区（feasible_d_range）
        # 的口径一致。规范化角只用于显示（它会把反折解折进 [0°,180°) 而显得"可达"）。
        sv_t = sv_k = None
        if ag1 is not None:
            sv_t, sv_k = servo_angles(ag1, ag2)
        ok_servo = (ok_geo and sv_t is not None
                    and smin - 1e-6 <= sv_t <= smax + 1e-6
                    and smin - 1e-6 <= sv_k <= smax + 1e-6)

        hip_px = self.ip(0, 0)
        c.create_oval(hip_px[0] - 5, hip_px[1] - 5, hip_px[0] + 5, hip_px[1] + 5,
                      fill=C_HIP, outline="black")
        c.create_text(hip_px[0] - 14, hip_px[1] + 18, text="髋", fill=C_HIP,
                      font=("", 9, "bold"))

        if ag1 is not None and ok_geo:
            # 膝坐标（ag2 = 外倾为正口径 → 方向单位向量 = (−sin, cos)）
            kx = -a * math.sin(ag2)
            ky = a * math.cos(ag2)
            knee_px = self.ip(kx, ky)
            foot_px = self.ip(fx, fy)
            # 大腿 / 小腿
            c.create_line(hip_px[0], hip_px[1], knee_px[0], knee_px[1],
                          fill=C_THIGH, width=6, capstyle=tk.ROUND)
            c.create_line(knee_px[0], knee_px[1], foot_px[0], foot_px[1],
                          fill=C_SHANK, width=6, capstyle=tk.ROUND)
            # 关节角度弧（标注值用规范化角，保证与所画劣弧一致、不出现 186.7°/−173.3° 式的原始值）
            mid_t = self.arc_between(0, 0, 0, 1, kx, ky, 20, C_THIGH)
            self.arc_label(0, 0, mid_t, 20,
                           "{0:.1f}°".format(self.norm_deg(math.degrees(ag2))),
                           C_THIGH)
            mid_k = self.arc_between(kx, ky, -kx, -ky, fx - kx, fy - ky, 22, C_KNEE)
            self.arc_label(kx, ky, mid_k, 22,
                           "{0:.1f}°".format(math.degrees(ag1)), C_KNEE)
            # 关节点
            c.create_oval(knee_px[0] - 4, knee_px[1] - 4,
                          knee_px[0] + 4, knee_px[1] + 4,
                          fill=C_KNEE, outline="black")
            c.create_text(knee_px[0] - 24, knee_px[1] - 12, text="膝", fill=C_KNEE,
                          font=("", 9, "bold"))
            # 脚
            fcol = C_FOOT if ok_servo else C_FOOT_BAD
            c.create_oval(foot_px[0] - 5, foot_px[1] - 5,
                          foot_px[0] + 5, foot_px[1] + 5,
                          fill=fcol, outline="black")
            c.create_text(foot_px[0] - 10, foot_px[1] - 14, text="脚", fill=fcol,
                          font=("", 9, "bold"))
        else:
            c.create_text(ox, oy - 40, text="该位置无法构成腿部三角形（远离髋关节中心）",
                          fill=C_FOOT_BAD, font=("", 11, "bold"))

        # ---- 图例 ----
        self.draw_legend()

        # ---- 底部信息文本 ----
        self.update_labels(ag1, ag2, d, ok_geo, ok_servo, sv_t, sv_k)

    def draw_legend(self):
        c = self.canvas
        items = [
            ("rect", C_HIP, "髋（坐标原点）"),
            ("rect", C_KNEE, "膝关节"),
            ("rect", C_FOOT, "脚（红=超限）"),
            ("line", C_THIGH, "大腿"),
            ("line", C_SHANK, "小腿"),
            ("ring", C_RING, "灰环 = 几何可达范围（A-B ~ A+B）"),
            ("fan", C_FAN, "绿扇区 = 舵机行程 {0:.0f}~{1:.0f}° 可行范围".format(self.s_min, self.s_max)),
            ("dash", "#9c36b5", "紫虚线 = 步态：站姿脚 ↔ 伸出脚"),
        ]
        # 卡片底
        w_card = 430
        h_card = len(items) * 18 + 12
        c.create_rectangle(8, 8, 8 + w_card, 8 + h_card,
                           fill="#ffffff", outline="#dee2e6", width=1)
        y = 18
        for kind, color, label in items:
            x0 = 20
            if kind == "rect":
                c.create_rectangle(x0, y - 6, x0 + 10, y + 4,
                                   fill=color, outline="black")
            elif kind == "line":
                c.create_line(x0, y - 1, x0 + 10, y - 1, fill=color, width=4,
                              capstyle=tk.ROUND)
            elif kind == "ring":
                c.create_oval(x0, y - 6, x0 + 10, y + 4,
                              fill=C_RING, outline="#adb5bd")
            elif kind == "fan":
                c.create_rectangle(x0, y - 6, x0 + 10, y + 4,
                                   fill=C_FAN, outline=C_FAN_L)
            elif kind == "dash":
                c.create_line(x0, y - 1, x0 + 10, y - 1, fill=color, width=2,
                              dash=(3, 2))
            c.create_text(x0 + 16, y, text=label, anchor="w",
                          fill="#343a40", font=("", 9))
            y += 18
        c.create_text(8 + w_card - 12, 8 + h_card + 10,
                      text="角度口径：ag2 相对 +y（竖直），偏向 −x（髋外侧）为正 → 站姿外拱为 +67.38°",
                      anchor="e", fill="#868e96", font=("", 8))
        c.create_text(8 + w_card - 12, 8 + h_card + 22,
                      text="换算 3D 模拟 α = 90° − ag2（相对水平外轴、向上为正，站姿 = +22.6°）",
                      anchor="e", fill="#868e96", font=("", 8))
        c.create_text(8 + w_card - 12, 8 + h_card + 34,
                      text="舵机映射：大腿 = ag2（0°=竖直向上、180°=竖直向下）、小腿 = 180° − ag1（与固件一致）",
                      anchor="e", fill="#868e96", font=("", 8))
        c.create_text(8 + w_card - 12, 8 + h_card + 46,
                      text="零位偏移 ANGLE_OFFSET 当前整表为 0 ⇒ 最终写入角 = 上面的逻辑角",
                      anchor="e", fill="#868e96", font=("", 8))

    def update_labels(self, ag1, ag2, d, ok_geo, ok_servo, sv_t, sv_k):
        rng = "范围 {0:.0f}~{1:.0f}°".format(self.s_min, self.s_max)
        if ag1 is None:
            self.lb_ag1.config(text="膝角 ag1: --", fg="black")
            self.lb_ag2.config(text="大腿角 ag2: --", fg="black")
            self.lb_sv_t.config(text="大腿舵机角: --", fg="black")
            self.lb_sv_k.config(text="小腿舵机角: --", fg="black")
            self.lb_status.config(text="在髋原点附近，无法构角", fg=C_FOOT_BAD)
            return
        d1 = math.degrees(ag1)
        d2_raw = math.degrees(ag2)
        d2 = self.norm_deg(d2_raw)   # 同方向折到 [−180,180)，和画弧一致
        # 显示用的规范化映射（大腿 = 规范化后的 ag2）：sv_t/sv_k 是固件真正收到的原始角，
        # 两者只在"反折解"（IK 原始 ag2 < −180°）处不同，下面会单独提示。
        sv_t_disp, sv_k_disp = servo_angles_disp(ag1, ag2)
        self.lb_feet.config(text="脚: ({0:.2f}, {1:.2f})".format(self.fx, self.fy))
        self.lb_d.config(text="髋足距: {0:.2f}cm".format(d))
        self.lb_ag1.config(text="膝角 ag1: {0:.2f}°".format(d1), fg=C_KNEE)
        if abs(d2_raw - d2) > 0.05:
            ag2_txt = "大腿数学角 ag2: {0:.2f}°（IK 原始值 {1:.1f}°，同方向）".format(d2, d2_raw)
        else:
            ag2_txt = "大腿数学角 ag2: {0:.2f}°".format(d2)
        self.lb_ag2.config(text=ag2_txt, fg=C_THIGH)

        def servo_label(text, deg, base_color):
            if deg is None:
                return text + ": --", "black"
            if deg < self.s_min - 1e-6 or deg > self.s_max + 1e-6:
                over = max(self.s_min - deg, deg - self.s_max)
                return "{0}: {1:.2f}°（超行程 {2:.1f}°，固件会限幅截断）".format(
                    text, deg, over), C_FOOT_BAD
            margin = min(deg - self.s_min, self.s_max - deg)
            return "{0}: {1:.2f}°（余量 {2:.1f}°）".format(text, deg, margin), base_color

        t_txt, t_col = servo_label("大腿舵机", sv_t, C_THIGH)
        k_txt, k_col = servo_label("小腿舵机", sv_k, C_KNEE)
        # 大腿：原始角超行程、但"同方向的规范化角"落在行程内时说明清楚
        # （固件不做规范化 ⇒ 仍会被限幅到端点，摆不出这个脚位；绿扇区同样已排除）
        if (sv_t is not None
                and not (self.s_min - 1e-6 <= sv_t <= self.s_max + 1e-6)
                and self.s_min - 1e-6 <= sv_t_disp <= self.s_max + 1e-6):
            t_txt += "；同方向 {0:.1f}° 本在行程内，但固件用原始角不规范化 → 仍摆不出".format(
                sv_t_disp)
        self.lb_sv_t.config(text=t_txt + "  " + rng, fg=t_col)
        self.lb_sv_k.config(text=k_txt, fg=k_col)

        if not ok_geo:
            self.lb_status.config(text="✗ 超出几何可达范围", fg=C_FOOT_BAD)
        elif not ok_servo:
            self.lb_status.config(text="✗ 舵机角度超行程（固件会限幅，实际到不了）",
                                  fg=C_FOOT_BAD)
        else:
            self.lb_status.config(text="✓ 可达", fg=C_FOOT)


def main():
    root = tk.Tk()
    LegApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
