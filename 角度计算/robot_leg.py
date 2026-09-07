"""
六足机器人 单腿 2自由度 运动学可视化（同步固件 Config.h 参数）
坐标原点 = 大腿根部舵机（髋关节），y+ 竖直向上，x+ 为水平方向

功能：
- 鼠标点击/拖动画布 = 移动脚的位置；滚轮 / 按钮 = 缩放
- 实时显示：大腿数学角 ag2、膝角 ag1、两个舵机角及限位余量
- 舵机角映射与固件 SingleLeg.cpp 一致：
      大腿舵机角 = 90° - ag2（装反则取反为 90° + ag2）
      小腿舵机角 = 180° - ag1（装反则取反为 ag1）
- 灰色圆环 = 几何可达范围（内圆 |A-B| ~ 外圆 A+B）
- 绿色扇区 = 当前舵机行程（可在面板调 0~180°）约束下的真实可行范围：
  IK 算出但超出该区域的脚位，固件会被 ServoManager 限幅截断，
  实际到不了 —— 即之前讨论的"大腿舵机逼近 180° 上限"问题所在
- 紫色虚线 = 步态的两个固定足端目标（站姿脚 ↔ 伸出脚 2/3 分点）
- 腿长与固件 Config.h 同步：LENA=6.5cm（大腿）、LENB=12.6cm（小腿）
"""

import math
import tkinter as tk

# ============ 默认参数（与 include/Config.h 同步） ============
LENA = 6.5   # 大腿长 (cm)
LENB = 12.6  # 小腿长 (cm)
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


# ---------------- 运动学（与固件 LegKinematics.cpp / SingleLeg.cpp 同构） ----------------

def leg_ik(a, b, fx, fy):
    """足端坐标 -> (膝角 ag1, 大腿角 ag2, 髋足距 d)，单位弧度/厘米。
    ag1：膝内角（0=完全伸直，180=完全折叠）
    ag2：大腿方向相对 +y 轴的角度（+x 方向为正）"""
    d = math.sqrt(fx * fx + fy * fy)
    if d < 1e-9:
        return None, None, d
    cos1 = (a * a + b * b - d * d) / (2 * a * b)
    cos1 = clamp(cos1, -1, 1)
    ag1 = math.acos(cos1)
    cos2 = (a * a + d * d - b * b) / (2 * a * d)
    cos2 = clamp(cos2, -1, 1)
    ag2 = math.acos(cos2) + math.atan2(fx, fy)
    return ag1, ag2, d


def servo_angles(ag1, ag2):
    """数学角(弧度) -> 舵机角(度)，映射与固件 writeServosFromRad 一致"""
    thigh_deg = ag2 * 180.0 / math.pi
    knee_deg = ag1 * 180.0 / math.pi
    servo_thigh = 90.0 - thigh_deg
    servo_knee = 180.0 - knee_deg
    return servo_thigh, servo_knee


def default_standing_foot(a, b):
    """默认站立足端：小腿竖直并与内圆（半径 |a-b|）负侧相切"""
    r = abs(a - b)
    kx = -r
    ky = math.sqrt(a * a - r * r)
    return kx, ky - b


def extended_foot(a, b):
    """伸出足端：站立高度上，站姿脚与外圆同侧交点所成线段 2/3 分点"""
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
    与 IK 相同从 +y 轴起算，度）上的可行髋足距区间 (d_lo, d_hi)；无解返回 None。

    固件映射：舵机大腿 = 90 - ag2，舵机小腿 = 180 - ag1
      => ag2 ∈ [90-s_max, 90-s_min]，ag1 ∈ [180-s_max, 180-s_min]"""
    if s_min > s_max:
        return None
    deg2rad = math.pi / 180.0
    ag2_lo = (90.0 - s_max) * deg2rad
    ag2_hi = (90.0 - s_min) * deg2rad
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
        """沿弧中点方向放角度文本（画布角 mid_ang）"""
        if mid_ang is None:
            return
        rad = math.radians(mid_ang)
        mx, my = cx + math.cos(rad) * (r_px + 14) / self.scale, \
                 cy - math.sin(rad) * (r_px + 14) / self.scale
        px, py = self.ip(mx, my)
        self.canvas.create_text(px, py, text=text, fill=color, font=("", 9))

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
            # 膝坐标
            kx = a * math.sin(ag2)
            ky = a * math.cos(ag2)
            knee_px = self.ip(kx, ky)
            foot_px = self.ip(fx, fy)
            # 大腿 / 小腿
            c.create_line(hip_px[0], hip_px[1], knee_px[0], knee_px[1],
                          fill=C_THIGH, width=6, capstyle=tk.ROUND)
            c.create_line(knee_px[0], knee_px[1], foot_px[0], foot_px[1],
                          fill=C_SHANK, width=6, capstyle=tk.ROUND)
            # 关节角度弧
            mid_t = self.arc_between(0, 0, 0, 1, kx, ky, 20, C_THIGH)
            self.arc_label(0, 0, mid_t, 20, "{0:.1f}°".format(math.degrees(ag2)), C_THIGH)
            mid_k = self.arc_between(kx, ky, -kx, -ky, fx - kx, fy - ky, 22, C_KNEE)
            self.arc_label(kx, ky, mid_k, 22, "{0:.1f}°".format(math.degrees(ag1)), C_KNEE)
            # 关节点
            c.create_oval(knee_px[0] - 4, knee_px[1] - 4,
                          knee_px[0] + 4, knee_px[1] + 4,
                          fill=C_KNEE, outline="black")
            c.create_text(knee_px[0] - 24, knee_px[1] - 12, text="膝", fill=C_KNEE,
                          font=("", 9, "bold"))
            # 舵机角提示
            c.create_text(hip_px[0] + 34, hip_px[1] + 6,
                          text="大腿舵机 {0:.1f}°".format(sv_t),
                          fill=C_THIGH, font=("", 10, "bold"))
            c.create_text(knee_px[0] + 26, knee_px[1] + 30,
                          text="小腿舵机 {0:.1f}°".format(sv_k),
                          fill=C_KNEE, font=("", 10, "bold"))
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
                      text="舵机映射：大腿=90°−ag2，小腿=180°−ag1（固件约定，装反需取反）",
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
        d2 = math.degrees(ag2)
        self.lb_feet.config(text="脚: ({0:.2f}, {1:.2f})".format(self.fx, self.fy))
        self.lb_d.config(text="髋足距: {0:.2f}cm".format(d))
        self.lb_ag1.config(text="膝角 ag1: {0:.2f}°".format(d1), fg=C_KNEE)
        self.lb_ag2.config(text="大腿数学角 ag2: {0:.2f}°".format(d2), fg=C_THIGH)

        def servo_label(text, deg, base_color):
            if deg is None:
                return text + ": --", "black"
            if deg < self.s_min - 1e-6 or deg > self.s_max + 1e-6:
                over = max(self.s_min - deg, deg - self.s_max)
                return "{0}: {1:.2f}°（超限 {2:.1f}°!）".format(text, deg, over), C_FOOT_BAD
            margin = min(deg - self.s_min, self.s_max - deg)
            return "{0}: {1:.2f}°（余量 {2:.1f}°）".format(text, deg, margin), base_color

        t_txt, t_col = servo_label("大腿舵机", sv_t, C_THIGH)
        k_txt, k_col = servo_label("小腿舵机", sv_k, C_KNEE)
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
