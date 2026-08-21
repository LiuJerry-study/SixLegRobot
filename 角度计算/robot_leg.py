"""
六足机器人 单腿 2自由度 逆运动学可视化
坐标原点 = 大腿根部舵机（髋关节）
- 鼠标点击/拖动画布 = 移动脚的位置
- 鼠标滚轮 / 按钮 = 缩放
- 实时计算并显示：大腿角(ag2/机腿角)、膝盖角(ag1/两腿角)
- 画出可达范围（内径 |lena-lenb| 到外径 lena+lenb 的环形区域）
"""
import tkinter as tk
import math

LENA = 4.0   # 大腿长度
LENB = 3.0   # 小腿长度
W = 900
H = 640
SCALE = 60   # 初始缩放（每单位像素数）
MIN_SCALE = 15
MAX_SCALE = 250


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def leg_ik(fx, fy):
    d = math.sqrt(fx * fx + fy * fy)
    if d < 1e-9:
        return None, None, d
    cos1 = (LENA * LENA + LENB * LENB - d * d) / (2 * LENA * LENB)
    cos1 = clamp(cos1, -1, 1)
    ag1 = math.acos(cos1)
    cos2 = (LENA * LENA + d * d - LENB * LENB) / (2 * LENA * d)
    cos2 = clamp(cos2, -1, 1)
    ag2 = math.acos(cos2) + math.atan2(fx, fy)
    return ag1, ag2, d


class LegApp:
    def __init__(self, root):
        self.root = root
        root.title("六足机器人单腿逆运动学可视化")
        self.scale = SCALE
        self.cw = W
        self.ch = H

        top = tk.Frame(root)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)
        tk.Label(top, text="大腿长(LENA):").pack(side=tk.LEFT)
        self.e_lena = tk.Entry(top, width=6)
        self.e_lena.insert(0, str(LENA))
        self.e_lena.pack(side=tk.LEFT)
        tk.Label(top, text="  小腿长(LENB):").pack(side=tk.LEFT)
        self.e_lenb = tk.Entry(top, width=6)
        self.e_lenb.insert(0, str(LENB))
        self.e_lenb.pack(side=tk.LEFT)
        tk.Button(top, text="更新参数", command=self.update_params).pack(side=tk.LEFT, padx=8)

        # ===== 缩放控制按钮 =====
        tk.Button(top, text="放大 +", command=lambda: self.zoom(1.25)).pack(side=tk.LEFT, padx=2)
        tk.Button(top, text="缩小 -", command=lambda: self.zoom(0.8)).pack(side=tk.LEFT, padx=2)
        tk.Button(top, text="重置", command=self.reset_zoom).pack(side=tk.LEFT, padx=2)
        self.lb_zoom = tk.Label(top, text="缩放: 100%", fg="darkblue")
        self.lb_zoom.pack(side=tk.LEFT, padx=8)

        self.lb_status = tk.Label(top, text="", fg="blue")
        self.lb_status.pack(side=tk.LEFT, padx=12)
        # 画布随窗口伸缩
        self.canvas = tk.Canvas(root, width=W, height=H, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        info = tk.Frame(root)
        info.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)
        self.lb_ag1 = tk.Label(info, text="膝盖角(ag1): --", font=("", 12))
        self.lb_ag1.pack(side=tk.LEFT, padx=15)
        self.lb_ag2 = tk.Label(info, text="大腿角(ag2): --", font=("", 12))
        self.lb_ag2.pack(side=tk.LEFT, padx=15)
        self.lb_feet = tk.Label(info, text="脚坐标: --", font=("", 12))
        self.lb_feet.pack(side=tk.LEFT, padx=15)

        self.fx, self.fy = 5.0, 0.0
        self.dragging = False
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        # 鼠标滚轮缩放（Windows: <MouseWheel>）
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        # Linux 支持
        self.canvas.bind("<Button-4>", lambda e: self.zoom(1.1))
        self.canvas.bind("<Button-5>", lambda e: self.zoom(0.9))
        # 窗口尺寸变化时自动重绘
        self.canvas.bind("<Configure>", lambda e: self.draw())
        self.draw()

    def zoom(self, factor):
        self.scale = clamp(self.scale * factor, MIN_SCALE, MAX_SCALE)
        self.lb_zoom.config(text="缩放: {0}%".format(int(self.scale / SCALE * 100)))
        self.draw()

    def reset_zoom(self):
        self.scale = SCALE
        self.lb_zoom.config(text="缩放: 100%")
        self.draw()

    def on_wheel(self, event):
        # 滚轮向上放大，向下缩小
        direction = 1 if event.delta > 0 else -1
        self.zoom(1.1 if direction > 0 else 0.9)
        return "break"

    def update_params(self):
        global LENA, LENB
        try:
            LENA = float(self.e_lena.get())
            LENB = float(self.e_lenb.get())
            self.draw()
        except ValueError:
            self.lb_status.config(text="请输入有效数字", fg="red")

    def on_press(self, event):
        self.dragging = True
        ox, oy = self.cw // 2, self.ch // 2
        self.fx = (event.x - ox) / self.scale
        self.fy = -(event.y - oy) / self.scale
        self.draw()

    def on_drag(self, event):
        if not self.dragging:
            return
        ox, oy = self.cw // 2, self.ch // 2
        self.fx = (event.x - ox) / self.scale
        self.fy = -(event.y - oy) / self.scale
        self.draw()

    def on_release(self, event):
        self.dragging = False

    def ip(self, x, y):
        ox, oy = self.cw // 2, self.ch // 2
        return ox + x * self.scale, oy - y * self.scale

    def draw(self):
        c = self.canvas
        cw = self.cw = max(c.winfo_width(), 1)
        ch = self.ch = max(c.winfo_height(), 1)
        c.delete("all")
        d = LENA + LENB
        outer_r = int(d * self.scale)
        inner_r = int(abs(LENA - LENB) * self.scale)
        OX, OY = cw // 2, ch // 2
        c.create_oval(OX - outer_r, OY - outer_r, OX + outer_r, OY + outer_r, outline="lightgray", width=2)
        c.create_oval(OX - inner_r, OY - inner_r, OX + inner_r, OY + inner_r, outline="lightgray", width=2)
        c.create_line(0, OY, cw, OY, fill="gray", dash=(2, 2))
        c.create_line(OX, 0, OX, ch, fill="gray", dash=(2, 2))
        px, py = self.ip(0, 0)
        c.create_oval(px - 4, py - 4, px + 4, py + 4, fill="red", outline="black")

        ag1, ag2, dist = leg_ik(self.fx, self.fy)
        if ag1 is None:
            c.create_text(OX, OY - 30, text="原点位置无法构成角度", fill="red")
            return

        a2_deg = math.degrees(ag2)
        a1_deg = math.degrees(ag1)
        ax = LENA * math.sin(ag2)
        ay = LENA * math.cos(ag2)
        kx_px, ky_py = self.ip(ax, ay)
        f_px, f_py = self.ip(self.fx, self.fy)
        c.create_line(px, py, kx_px, ky_py, fill="blue", width=6)
        c.create_line(kx_px, ky_py, f_px, f_py, fill="green", width=6)
        c.create_oval(kx_px - 4, ky_py - 4, kx_px + 4, ky_py + 4, fill="orange", outline="black")
        c.create_oval(f_px - 5, f_py - 5, f_px + 5, f_py + 5, fill="darkgreen", outline="black")

        # ===== 水平线：脚同一高度与可达外圆交点 -> 脚，标记分点 =====
        R = LENA + LENB
        feet_y = self.fy
        # 该高度下外圆的水平半宽：x^2 + y^2 = R^2 -> x = +/-sqrt(R^2 - y^2)
        yy2 = (R ** 2 - feet_y ** 2)
        if yy2 >= 0:
            dx = math.sqrt(yy2)
            # 圆交点取在脚的同一侧水平方向
            ix = dx if self.fx >= 0 else -dx
            iy = feet_y
            i_px = OX + ix * self.scale
            i_py = OY - iy * self.scale
            # 水平交点->脚 的水平线段(品红)
            f_px2, f_py2 = self.ip(self.fx, self.fy)
            if abs(self.fx) <= abs(ix):
                sx, sy, ex, ey = ix, iy, self.fx, self.fy
            else:
                sx, sy, ex, ey = self.fx, self.fy, ix, iy
            s_px, s_py = self.ip(sx, sy)
            e_px, e_py = self.ip(ex, ey)
            c.create_line(s_px, s_py, e_px, e_py, fill="magenta", width=3)
            # 标记水平圆交点
            c.create_oval(i_px - 4, i_py - 4, i_px + 4, i_py + 4, fill="white", outline="purple", width=2)
            c.create_text(i_px, i_py + 15, text="水平圆交点", fill="purple", font=("", 9))

            def mark_point(t, color, label):
                mx = sx + (ex - sx) * t
                my = sy + (ey - sy) * t
                mx_px, my_py = self.ip(mx, my)
                c.create_oval(mx_px - 5, my_py - 5, mx_px + 5, my_py + 5, fill=color, outline="black")
                c.create_text(mx_px + 10, my_py - 6, text=label, fill=color, font=("", 9))

            # 只保留三分点 (1/3, 2/3)
            mark_point(1 / 3, "red", "3分(1/3)")
            mark_point(2 / 3, "red", "3分(2/3)")

        inner_r = abs(LENA - LENB)
        reachable = (inner_r <= dist <= d)
        status = "可达" if reachable else "超出范围"
        color = "green" if reachable else "red"
        self.lb_status.config(text=status, fg=color)
        self.lb_ag1.config(text="膝盖角(ag1): {0:6.2f}°".format(a1_deg))
        self.lb_ag2.config(text="大腿角(ag2): {0:6.2f}°".format(a2_deg))
        self.lb_feet.config(text="脚坐标: ({0:.2f}, {1:.2f})".format(self.fx, self.fy))
        c.create_text(px + 50, py + 55, text="大腿 {0:.1f}°".format(a2_deg), fill="blue")
        c.create_text(kx_px + 55, ky_py - 25, text="膝 {0:.1f}°".format(a1_deg), fill="orange")


def main():
    root = tk.Tk()
    LegApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
