/* gaits.js — 步态规划器（三角 / 波纹 / 对腿）
 * 行程系数 c_i = cos(φ − α_i): c>0 支撑相"收"(拉地), c<0 "伸"(蹬地);
 * 足端扫掠用轻度 S 曲线, 机身速度由 main.js 的摩擦动力学决定。
 * 全局挂载: window.GaitPlanner */
(function (g) {
  'use strict';

  const TRIPOD_A = [0, 2, 4];        // 0°,120°,240°
  const TRIPOD_B = [1, 3, 5];        // 60°,180°,300°
  const PAIRS = [[0, 3], [1, 4], [2, 5]];  // 对腿组合 (0/180, 60/240, 120/300)

  function createGaitPlanner(CONFIG) {
    const { azimuths, R_LO, R_HI } = CONFIG;
    const R_MID = (R_LO + R_HI) / 2;
    const n = 6;

    // 每腿行程系数 c_i = cos(φ − α_i)
    function coeffs(phiDeg) {
      const c = new Array(n);
      for (let i = 0; i < n; i++) {
        const d = (phiDeg - azimuths[i]) * Math.PI / 180;
        c[i] = Math.cos(d);
      }
      return c;
    }

    // 支撑相足端扫掠的起止径向位置: c>0 从外扫到内(收), c<0 从内扫到外(伸), |c| 决定幅度
    function sweepTargets(c, sweepMag) {
      const from = new Array(n), to = new Array(n);
      for (let i = 0; i < n; i++) {
        const mag = Math.max(Math.abs(c[i]), 0.06) * sweepMag; // 几乎垂直的腿也保留最小行程
        const hi = R_MID + mag / 2, lo = R_MID - mag / 2;
        if (c[i] > 0) { from[i] = hi; to[i] = lo; }   // 收
        else          { from[i] = lo; to[i] = hi; }   // 伸
      }
      return { from, to };
    }

    const lerp = (a, b, u) => a + (b - a) * u;
    const mod1 = x => ((x % 1) + 1) % 1; // JS 的 % 保留符号, 转成 [0,1)
    // 缓动相位: 轻度 S 曲线 (斜率 0.5~1.5 倍均速), 换腿瞬间不刹停
    const ease = u => u - 0.5 * Math.sin(2 * Math.PI * u) / (2 * Math.PI);
    const easeDot = u => 1 - 0.5 * Math.cos(2 * Math.PI * u);   // d(ease)/du

    /* compute(mode, phiDeg, t, opts) -> { targets[6]: {r, y, rdot, state, coeff}, dir, vScale }
     * rdot = 足端指令径向速度 (cm/s, 解析解), 供摩擦动力学使用
     * ⚠ main.js 额外传的 bodyHeight / 返回的 vScale 均无人使用 */
    function compute(mode, phiDeg, t, opts) {
      const { sweepMag, cycleT, lift, pairIndex } = opts;
      const c = coeffs(phiDeg);
      const { from, to } = sweepTargets(c, sweepMag);
      const targets = new Array(n);
      const stanceIdx = [];

      const setStance = (i, u, winT) => {
        targets[i] = {
          r: lerp(from[i], to[i], ease(u)), y: 0, state: 'stance', coeff: c[i],
          rdot: (to[i] - from[i]) * easeDot(u) / winT
        };
        stanceIdx.push(i);
      };
      const setSwing = (i, u, winT) => {
        targets[i] = {
          r: lerp(to[i], from[i], ease(u)), y: lift * Math.sin(Math.PI * u), state: 'swing', coeff: c[i],
          rdot: (from[i] - to[i]) * easeDot(u) / winT
        };
      };
      const setHold = (i) => {
        // 悬停: 微微离地, 不触地、不参与摩擦驱动 (对腿步态保持"干净平移")
        targets[i] = { r: R_MID, y: lift * 0.5, state: 'hold', coeff: c[i], rdot: 0 };
      };

      if (mode === 'tripod') {
        const frac = (t / cycleT) % 1;
        for (const i of TRIPOD_A) {
          if (frac < 0.5) setStance(i, frac / 0.5, 0.5 * cycleT);
          else setSwing(i, (frac - 0.5) / 0.5, 0.5 * cycleT);
        }
        for (const i of TRIPOD_B) {
          if (frac >= 0.5) setStance(i, (frac - 0.5) / 0.5, 0.5 * cycleT);
          else setSwing(i, frac / 0.5, 0.5 * cycleT);
        }
      } else if (mode === 'ripple') {
        const SW = 1 / 6; // 摆动窗 = 1/6 周期, 与 6 腿错相恰好铺满: 任意时刻恰有一条腿摆动
        for (let i = 0; i < n; i++) {
          const d = mod1(t / cycleT - i / 6); // 距本腿摆动窗起点的相位
          if (d < SW) setSwing(i, d / SW, SW * cycleT);
          else setStance(i, (d - SW) / (1 - SW), (1 - SW) * cycleT);
        }
      } else if (mode === 'bipod') {
        const pair = PAIRS[pairIndex];
        const frac = (t / cycleT) % 1;
        for (let i = 0; i < n; i++) {
          if (i === pair[0]) {
            if (frac < 0.5) setStance(i, frac / 0.5, 0.5 * cycleT); else setSwing(i, (frac - 0.5) / 0.5, 0.5 * cycleT);
          } else if (i === pair[1]) {
            if (frac >= 0.5) setStance(i, (frac - 0.5) / 0.5, 0.5 * cycleT); else setSwing(i, frac / 0.5, 0.5 * cycleT);
          } else {
            setHold(i);
          }
        }
      }

      // 净推力方向 = Σ(支撑腿 c_i·ê_i), 归一化
      let dx = 0, dz = 0;
      for (const i of stanceIdx) {
        const az = azimuths[i] * Math.PI / 180;
        dx += c[i] * Math.cos(az);
        dz += c[i] * Math.sin(az);
      }
      const len = Math.hypot(dx, dz);
      let dir = null;
      if (len > 1e-4) {
        if (mode === 'bipod') {
          // 对腿模式: 只能沿所选对轴平移 (方向 ± 由 φ 的对轴分量符号决定)
          const az = azimuths[PAIRS[pairIndex][0]] * Math.PI / 180;
          const sign = (dx * Math.cos(az) + dz * Math.sin(az)) >= 0 ? 1 : -1;
          dir = { x: sign * Math.cos(az), z: sign * Math.sin(az) };
        } else {
          dir = { x: dx / len, z: dz / len };
        }
      }

      // 速度系数: 旧"净推力匀速"模型遗留, 现无人使用
      const sf = mode === 'ripple' ? (1 - 1 / 6) : 0.5;
      const vScale = (sweepMag / (sf * cycleT)) * (mode === 'tripod' ? 2 / 3 : mode === 'ripple' ? 0.5 : 0.5);

      return { targets, dir, vScale };
    }

    return { compute, coeffs, TRIPOD_A, TRIPOD_B, PAIRS };
  }

  g.GaitPlanner = { createGaitPlanner };
})(typeof window !== 'undefined' ? window : globalThis);
