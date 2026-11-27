/* 径向双关节三维模型。机头为 +X，上方为 +Y，左侧为 −Z。 */
(function (g) {
  'use strict';

  const model = g.RobotModel;
  const motion = g.MotionFrame;
  const CONFIG = {
    bodyRadius: 6.8,     // 六边形机体外接圆半径 (cm)
    bodyThickness: 2.0,  // 机身厚度 (cm)
    hipRadius: 7.4,      // 髋关节离机心距离 (cm): 略大于机体半径, 大腿根部不穿模
    hipDrop: 1.0,        // 髋关节低于机体中心的量 (cm)
    L1: model.thigh,             // 大腿长 (cm, 真实硬件/固件 LENA)
    L2: model.shin,            // 小腿长 (cm, 真实硬件/固件 LENB)
    azimuths: model.pins.map((_, i) => motion.legAzimuth(i)), // 各腿方位角 (度, 0°=前)
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
    let lastMirror = 1;

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
        Math.cos(azRad) * CONFIG.hipRadius, -CONFIG.hipDrop, -Math.sin(azRad) * CONFIG.hipRadius
      );
      legGroup.rotation.y = azRad;
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
    function setMirror(mirror) {
      if (mirror === lastMirror) return;
      lastMirror = mirror;
      for (const leg of legs) {
        leg.azimuthDeg = motion.legAzimuth(leg.index, mirror);
        const heading = motion.direction(leg.azimuthDeg);
        leg.group.position.x = CONFIG.hipRadius * heading.x;
        leg.group.position.z = CONFIG.hipRadius * heading.z;
        leg.group.rotation.y = leg.azimuthDeg * Math.PI / 180;
      }
    }

    function setPose(bodyHeight, targets) {
      setMirror(1);
      for (let i = 0; i < 6; i++) {
        const leg = legs[i];
        leg.group.visible = true;
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

    function applyServoAngles(applied, mirror = 1, support = null) {
      setMirror(mirror);
      for (const leg of legs) {
        const pose = motion.joints(leg.index, applied);
        leg.group.visible = !!pose;
        if (!pose) continue;
        leg.hip.rotation.z = pose.alpha;
        leg.knee.rotation.z = pose.turn;
        leg.state = support === null ? 'hold' : (support & (1 << leg.index)) ? 'stance' : 'swing';
        leg.coeff = 0;
        leg.th1Deg = pose.alpha * 180 / Math.PI;
        leg.th2Deg = 180 + pose.turn * 180 / Math.PI;
        const color = leg.state === 'stance' ? STANCE_COLOR : leg.state === 'swing' ? SWING_COLOR : HOLD_COLOR;
        leg.thighMat.color.setHex(color);
        leg.shinMat.color.setHex(color);
      }
    }

    return { root, legs, config: CONFIG, setPose, applyServoAngles };
  }

  g.Robot = { CONFIG, solveIK, servoToVisual, createRobot };
})(typeof window !== 'undefined' ? window : globalThis);
