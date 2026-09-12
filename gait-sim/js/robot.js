/* ============================================================
 * robot.js — 径向 2 舵机腿（蜘蛛式折腿）三维模型 + 逆运动学
 * 构型: 每条腿只在自身径向竖直平面内运动, 无转向关节; 大腿从髋向外上拱、
 *   小腿从膝盖向下折回着地, 膝盖是最高/最外点, 侧看呈 "Λ" 拱。
 *   尺寸同硬件: 大腿 L1=6.5cm、小腿 L2=12.5cm（固件 Config.h 的 LENA/LENB）。
 *
 * 逆运动学(余弦定理, 与固件 LegKinematics.cpp 同构):
 *   a = 足端相对髋的径向偏移, drop = 足端在髋下方的落差, d = √(a²+drop²)
 *   β = acos((L1²+L2²−d²)/(2·L1·L2))   膝部内角: π=直腿, 0=完全折回
 *   γ = acos((L1²+d²−L2²)/(2·L1·d))
 *   α = atan2(−drop, a) + γ            相对"水平径向向外"轴, 向上为正
 *   足端 = L1·(cosα,sinα) + L2·(cos(α+π+β), sin(α+π+β)); 可达环 6 ≤ d ≤ 19
 *   默认站姿同固件 defaultStandingFoot: 髋外 6.0cm、下方 10.0cm, 小腿竖直
 *   → 本页 IK 解出 α ≈ +22.62°（上拱外倾的那条大腿, 仍是 "Λ" 拱）
 *
 * 与固件的角度口径（2026-09-10「外倾为正」方案 B + 大腿舵机映射按实物实测改版, 详见 README「固件 ↔ 仿真角度口径」）:
 *   固件 LegKinematics::ik() 返回【弧度】（调用方 src/SingleLeg.cpp 只换算一次, 本页不必再换）:
 *     knee(ag1)  = acos((LENA²+LENB²−d²)/(2·LENA·LENB))   // 两连杆夹角: 180° = 完全伸直、0° = 完全折叠, 恒为正
 *                                                          // （几何弯曲角 θ = 180° − ag1; 站姿 ag1 = 67.38° → θ = 112.62°）
 *     thigh(ag2) = −( acos((LENA²+d²−LENB²)/(2·LENA·d)) + atan2(fx,fy) )   // 外倾为正
 *   ⚠ 口径对应: 本页 α（相对"水平径向向外"轴、向上为正）= 90° − ag2;
 *     站姿 ag1 = ag2 = +67.38° → α = 90 − 67.38 = +22.62°。
 *     （旧记法 ag2 = −67.38° 及配套的负号口径已作废 —— 姿态没变, 只是符号口径翻成正的）
 *   舵机映射（2026-09-10 实物实测改版）: servoThigh = ag2（站姿 67.38°）、servoKnee = 180 − ag1（站姿 112.62°）,
 *     再由 ServoManager::write() 叠 Config.h::ANGLE_OFFSET 并限幅 0~180 —— 该表整表已清零,
 *     故【最终写入角 = 逻辑角】: 大腿 0~180° ↔ ag2 ∈ [0°, 180°]（可用外拱区间 ag2 ∈ [0°, 90°];
 *     ag2 < 0 即脚抬到髋高以上, 会被限幅到大腿竖直向上的 0°）。
 *
 * 舵机角 → 渲染姿态（孪生反向映射, 走 applyServoAngles）: α = 90°−S, knee = −K
 *   ⚠ 与固件口径自洽: 回传的 S = ag2 → α = 90 − ag2; K = 180 − ag1 → knee = ag1 − 180。
 *     ANGLE_OFFSET 已清零 → 回传的 S/K 就是最终写入角(= 逻辑角), 本页只 1:1 镜像, 不做补偿、也不反向换算。
 */
(function (g) {
  'use strict';

  const CONFIG = {
    bodyRadius: 6.8,     // 六边形机体外接圆半径 (cm)
    bodyThickness: 2.0,  // 机身厚度 (cm)
    hipRadius: 7.4,      // 髋关节离机心距离 (cm): 略大于机体半径, 大腿根部不穿模
    hipDrop: 1.0,        // 髋关节低于机体中心的量 (cm)
    L1: 6.5,             // 大腿长 (cm, 真实硬件/固件 LENA)
    L2: 12.5,            // 小腿长 (cm, 真实硬件/固件 LENB)
    azimuths: [0, 60, 120, 180, 240, 300], // 各腿方位角 (度, 0°=前)
    R_LO: 13.4,          // 足端径向范围下界 (cm, 离机心) = hip+6.0 (固件默认站立足)
    R_HI: 20.2           // 足端径向范围上界 (cm, 离机心) = hip+12.8 (固件伸出足 2/3 分点)
  };
  const R_MID = (CONFIG.R_LO + CONFIG.R_HI) / 2;

  function clamp1(v) { return Math.max(-1, Math.min(1, v)); }

  /* 蜘蛛式折腿逆运动学 */
  function solveIK(a, drop) {
    const L1 = CONFIG.L1, L2 = CONFIG.L2;
    const d = Math.sqrt(a * a + drop * drop);
    if (d < 1e-9) return { th1: 0, th2: Math.PI };
    const beta = Math.acos(clamp1((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)));
    const gamma = Math.acos(clamp1((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)));
    const alpha = Math.atan2(-drop, a) + gamma;   // 相对径向向外轴, 向上为正
    return { th1: isFinite(alpha) ? alpha : 0, th2: isFinite(beta) ? beta : Math.PI };
  }

  /* 舵机原始角(0~180) → 孪生渲染角 */
  function servoToVisual(S, K) {
    // 固件映射(实物实测): 大腿 S = ag2、小腿 K = 180 − ag1 → α = 90°−S、β = 180 − K、膝转 = −K
    return { alphaDeg: 90 - S, betaDeg: 180 - K, kneeTurnDeg: -K };
  }

  // 建立三维模型, 返回 { root, legs, config, setPose }
  function createRobot(THREE) {
    const root = new THREE.Group();
    const legs = [];

    // ---- 机体 (六棱柱) ----
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x39405c, roughness: 0.55, metalness: 0.4 });
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(CONFIG.bodyRadius, CONFIG.bodyRadius, CONFIG.bodyThickness, 6),
      bodyMat
    );
    root.add(body);

    // 前向指示器 (红尖, 0° 方向; 本构型无法偏航, 它永远朝 +x)
    const noseMat = new THREE.MeshStandardMaterial({ color: 0xe5484d, roughness: 0.4 });
    const nose = new THREE.Mesh(new THREE.ConeGeometry(1.0, 3.0, 4), noseMat);
    nose.rotation.z = -Math.PI / 2;   // 锥尖指向 +x
    nose.position.set(CONFIG.bodyRadius + 1.7, CONFIG.bodyThickness / 2 + 0.3, 0);
    root.add(nose);

    const STANCE_COLOR = 0x4f9d69;   // 支撑腿 绿
    const SWING_COLOR  = 0xe0913d;   // 摆动腿 橙
    const HOLD_COLOR   = 0x8a93a8;   // 悬停/非驱动 灰

    for (let i = 0; i < 6; i++) {
      const azRad = CONFIG.azimuths[i] * Math.PI / 180;

      // 腿组: 局部 +x = 径向向外, +y = 向上; 髋位于机身边缘、略低于机体中心
      const legGroup = new THREE.Group();
      legGroup.position.set(
        Math.cos(azRad) * CONFIG.hipRadius, -CONFIG.hipDrop, Math.sin(azRad) * CONFIG.hipRadius
      );
      legGroup.rotation.y = -azRad;  // 负号：Three.js绕Y轴正旋转是从X转向Z，需要反转
      root.add(legGroup);

      // 髋关节 (绕局部 z 旋转: + = 大腿上抬, - = 大腿下垂)
      const hip = new THREE.Group();
      legGroup.add(hip);

      // 髋球 (腿部根部, 不随髋旋转)
      const hipMat = new THREE.MeshStandardMaterial({ color: 0x565d73, roughness: 0.6 });
      const hipJoint = new THREE.Mesh(new THREE.SphereGeometry(0.7, 10, 10), hipMat);
      legGroup.add(hipJoint);

      // 大腿: 从髋向外伸的锥台 (根部粗、膝端细), 侧看呈 "Λ" 拱
      const thighMat = new THREE.MeshStandardMaterial({ color: HOLD_COLOR, roughness: 0.6 });
      const thigh = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.75, CONFIG.L1, 8), thighMat);
      thigh.rotation.z = -Math.PI / 2;   // 圆柱轴对齐局部 +x (径向向外)
      thigh.position.x = CONFIG.L1 / 2;
      hip.add(thigh);

      // 膝关节 (大腿外端): 小腿在此向下折回
      const knee = new THREE.Group();
      knee.position.x = CONFIG.L1;
      hip.add(knee);
      const kneeCapMat = new THREE.MeshStandardMaterial({ color: 0x565d73, roughness: 0.6 });
      const kneeCap = new THREE.Mesh(new THREE.SphereGeometry(0.6, 8, 8), kneeCapMat);
      knee.add(kneeCap);

      // 小腿: 锥台 (膝端粗、足端细), 绕膝旋转 (π+β 时向下折回)
      const shinMat = new THREE.MeshStandardMaterial({ color: HOLD_COLOR, roughness: 0.6 });
      const shin = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.55, CONFIG.L2, 8), shinMat);
      shin.rotation.z = -Math.PI / 2;
      shin.position.x = CONFIG.L2 / 2;
      knee.add(shin);

      const footMat = new THREE.MeshStandardMaterial({ color: 0x22252e, roughness: 0.8 });
      const foot = new THREE.Mesh(new THREE.SphereGeometry(0.45, 8, 8), footMat);
      foot.position.x = CONFIG.L2;
      knee.add(foot);

      legs.push({
        index: i,
        azimuthDeg: CONFIG.azimuths[i],
        group: legGroup, hip, knee, foot,
        thighMat, shinMat, footMat,
        // state/coeff/r/y/th1Deg/th2Deg 由 setPose() 写入; slipping 由 main.js 写;
        // ⚠ force 字段已废弃: 推力矢量读的是 main.js 的 foot[i].force
        state: 'hold', coeff: 0, r: R_MID, y: 0, th1Deg: 0, th2Deg: 0,
        force: 0, slipping: false
      });
    }

    // 设置全部腿的关节角与配色; targets[i] = {r, y, state, coeff}, y = 足端高度(地面=0)
    function setPose(bodyHeight, targets) {
      for (let i = 0; i < 6; i++) {
        const leg = legs[i];
        const t = targets[i];
        const a = t.r - CONFIG.hipRadius;          // 足端相对髋的径向偏移
        const drop = Math.max(0.5, bodyHeight - CONFIG.hipDrop - t.y); // 髋到足端竖直落差 (髋低于机心 hipDrop)
        const { th1, th2 } = solveIK(a, drop);

        leg.hip.rotation.z = th1;                  // 大腿角
        leg.knee.rotation.z = th2 - Math.PI;       // 小腿向下折回 (β-π，负值使小腿向下弯)
        leg.r = t.r; leg.y = t.y; leg.state = t.state; leg.coeff = t.coeff;
        leg.th1Deg = th1 * 180 / Math.PI;
        leg.th2Deg = th2 * 180 / Math.PI;

        const col = t.state === 'stance' ? STANCE_COLOR : t.state === 'swing' ? SWING_COLOR : HOLD_COLOR;
        leg.thighMat.color.setHex(col);
        leg.shinMat.color.setHex(col);
      }
    }

    // 数字孪生: 直接用 12 路舵机角摆姿态, 不经 IK —— 真机发什么角就摆什么姿态。
    // arr12[ch], ch = leg*2+joint (0=大腿 1=小腿), 角度 0~180; null = 未知(跳过)
    function applyServoAngles(arr12) {
      for (let i = 0; i < 6; i++) {
        const S = arr12[i * 2], K = arr12[i * 2 + 1];
        if (S === null || S === undefined || K === null || K === undefined) continue;
        const leg = legs[i];
        leg.hip.rotation.z = (90 - S) * Math.PI / 180;    // 大腿 α = 90°−S = 90°−ag2（固件 S = ag2, 零位偏移已清零）
        leg.knee.rotation.z = -K * Math.PI / 180;         // 小腿 β−180° = −K（固件 K = 180 − ag1）
        leg.state = 'hold'; leg.coeff = 0;
        leg.th1Deg = 90 - S;
        leg.th2Deg = 180 - K;
        leg.thighMat.color.setHex(HOLD_COLOR);
        leg.shinMat.color.setHex(HOLD_COLOR);
      }
    }

    return { root, legs, config: CONFIG, setPose, applyServoAngles };
  }

  g.Robot = { CONFIG, solveIK, servoToVisual, createRobot };
})(typeof window !== 'undefined' ? window : globalThis);
