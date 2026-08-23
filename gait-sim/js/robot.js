/* ============================================================
 * robot.js — 径向 2 舵机腿（蜘蛛式折腿）机器人三维模型
 *
 * 构型（与真实硬件一致）:
 *   - 每条腿只在自身径向竖直平面内运动, 无转向关节
 *   - 蜘蛛式折腿: 大腿从髋关节向外伸(可上拱), 小腿从膝盖向下折回着地,
 *     膝盖是整条腿的最高/最外点 —— 侧看像 "Λ" 形拱
 *   - 尺寸与真实硬件一致: 大腿 L1=8.0cm, 小腿 L2=13.0cm
 *     (固件 Config.h 的 LENA/LENB 同步为 8.0/13.0)
 *   - 默认站立姿态与固件 SingleLeg::defaultStandingFoot 一致:
 *     足端在髋外侧 5.0cm、下方 6.755cm, 小腿竖直, 脚直接落在膝正下方
 *     → R_LO = hipRadius + 5.0, R_HI ≈ hipRadius + 14.9 (= extendedFoot 2/3 分点)
 *
 * 逆运动学（与固件 LegKinematics.cpp 同构, 余弦定理）:
 *   输入: a    = 足端相对髋的径向偏移 (cm, 向外为正)
 *         drop = 足端在髋下方的竖直距离 (cm, 向下为正)
 *   输出: th1 髋角 α (rad, 相对"水平径向向外"轴, 向上为正)
 *         th2 膝角 β (rad, 膝部内角: π=直腿, 0=完全折回)
 *   d = √(a²+drop²)
 *   β = acos((L1²+L2²−d²) / (2·L1·L2))
 *   γ = acos((L1²+d²−L2²) / (2·L1·d))
 *   α = atan2(−drop, a) + γ
 *   足端 = L1·(cosα, sinα) + L2·(cos(α+π+β), sin(α+π+β))
 *   可达环: |L1−L2|=5 ≤ d ≤ L1+L2=21
 * ============================================================ */
(function (g) {
  'use strict';

  const CONFIG = {
    bodyRadius: 6.8,     // 六边形机体外接圆半径 (cm) —— 蜘蛛式: 机体小、腿长
    bodyThickness: 2.0,  // 机身厚度 (cm)
    hipRadius: 7.4,      // 髋关节离机体中心距离 (cm) —— 略大于机体半径,
                         //   腿从机身边缘(顶点)伸出, 大腿根部不再穿过机身(不穿模)
    hipDrop: 1.0,        // 髋关节低于机身中心的高度 (cm) —— 蜘蛛腿从机体下方伸出
    L1: 8.0,             // 大腿长 (cm, 真实硬件/固件 LENA)
    L2: 13.0,            // 小腿长 (cm, 真实硬件/固件 LENB)
    azimuths: [0, 60, 120, 180, 240, 300], // 各腿方位角 (度, 0°=前)
    R_LO: 12.4,          // 足端径向范围下界 (cm, 离机心) = hip+5.0 (固件默认站立足)
    R_HI: 22.3           // 足端径向范围上界 (cm, 离机心) = hip+14.9 (固件伸出足 2/3 分点)
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
        // 每帧由步态规划器 + 摩擦动力学 + setPose 更新
        state: 'hold', coeff: 0, r: R_MID, y: 0, th1Deg: 0, th2Deg: 0,
        force: 0, slipping: false
      });
    }

    // 设置全部腿的关节角与配色
    // targets[i] = { r, y, state, coeff }   r=足端离机心径向距离, y=足端世界高度(地面=0)
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

    return { root, legs, config: CONFIG, setPose };
  }

  g.Robot = { CONFIG, solveIK, createRobot };
})(typeof window !== 'undefined' ? window : globalThis);
