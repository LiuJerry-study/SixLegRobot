/* ============================================================
 * main.js — 场景搭建、相机控制、UI 绑定、主循环
 * 依赖: three.min.js, OrbitControls.js, robot.js, gaits.js
 * 单位: 厘米 (与固件 Config.h 一致)
 * ============================================================ */
(async function () {
  'use strict';
  if (typeof THREE === 'undefined') {
    document.getElementById('bootError').textContent = 'THREE 库加载失败：请检查网络或 lib/ 目录';
    return;
  }

  // ---------- 初始化 Rapier 物理引擎（失败不阻塞场景渲染） ----------
  let physicsOK = false;
  if (typeof Physics !== 'undefined') {
    var physicsReady = await Physics.init();
    if (physicsReady) {
      physicsOK = true;
    } else {
      console.warn('[Physics] Rapier 加载失败，将以无物理模式运行');
      document.getElementById('bootError').textContent =
        'Rapier 物理引擎加载失败（无物理模式运行）';
      document.getElementById('bootError').style.display = 'block';
    }
  }

  // ---------- 全局状态 ----------
  const S = {
    mode: 'tripod',
    phiDeg: 0,
    speed: 0.6,
    stride: 0.8,
    lift: 3.5,
    bodyHeight: 7.8,   // 默认≈固件站立高: 足端在髋下 6.755cm + hipDrop 1.0
    mu: 1.0,
    pairIndex: 0,
    running: true,
    showFootTrails: true,
    showBodyTrail: true,
    showForces: true,
    showAzimuths: true,
    follow: false,
    autoRotate: false,
    t: 0
  };

  // ---------- 渲染器 / 场景 / 相机 ----------
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x14161d);
  scene.fog = new THREE.Fog(0x14161d, 120, 300);

  const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 600);
  camera.position.set(85, 62, 85);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.getElementById('app').appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 8, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 12;
  controls.maxDistance = 400;

  // 灯光
  scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2c3a, 0.95));
  const sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(30, 60, 25);
  scene.add(sun);
  scene.add(new THREE.AmbientLight(0xffffff, 0.25));

  // 地面网格 (200cm × 80 = 2.5cm/格)
  const grid = new THREE.GridHelper(200, 80, 0x4a5066, 0x2a2e40);
  grid.position.y = -0.01;
  scene.add(grid);

  // ---------- 机器人 ----------
  const robot = window.Robot.createRobot(THREE);
  robot.root.position.set(0, S.bodyHeight, 0);
  scene.add(robot.root);

  const planner = window.GaitPlanner.createGaitPlanner(robot.config);
  const CONFIG = robot.config;

  // ---------- 摩擦动力学 (由 Rapier 物理引擎驱动) ----------
  // 机身是 Rapier 动态刚体, 支撑腿以弹簧-阻尼驱动扫掠, 推力经库仑摩擦封顶
  const PHYS = {
    k: 200,        // 腿驱动刚度 (kg/s²)
    c: 28,         // 腿阻尼 (kg/s)
    hyst: 0.85,    // 再粘住迟滞
    tauSlip: 0.06  // 滑移松弛时间常数 (s)
  };
  const foot = [];
  for (let i = 0; i < 6; i++) {
    foot.push({ r: null, pinned: false, fx: 0, fz: 0, force: 0, slipping: false });
  }
  let muS = S.mu, muK = 0.85 * S.mu, footNormal = 0;
  var bodyPos = { x: 0, y: S.bodyHeight, z: 0 };
  var bodyVel = { x: 0, y: 0, z: 0 };

  // 初始化 Rapier 机器人位置
  if (physicsOK) Physics.setPosition(0, S.bodyHeight, 0);

  // Rapier 物理步进: 每帧对每条支撑腿施加弹簧-阻尼力到刚体
  function stepPhysics(res, dt) {
    if (!physicsOK || !Physics.ready()) return;
    muS = S.mu; muK = 0.85 * S.mu;
    const nStance = res.targets.reduce((k, t) => k + (t.state === 'stance' ? 1 : 0), 0);
    if (nStance === 0) { Physics.step(); return; }
    footNormal = PHYS_M * PHYS_G / Math.max(1, nStance);

    const bodyPos = Physics.getPosition();
    const bodyVel = Physics.getLinvel();

    for (let i = 0; i < 6; i++) {
      const t = res.targets[i], fp = foot[i];
      const az = CONFIG.azimuths[i] * Math.PI / 180;
      const ex = Math.cos(az), ez = Math.sin(az);

      if (t.state !== 'stance') {
        fp.pinned = false; fp.r = t.r; fp.force = 0; fp.slipping = false;
        continue;
      }

      if (!fp.pinned) {
        fp.pinned = true;
        fp.fx = bodyPos.x + fp.r * ex;
        fp.fz = bodyPos.z + fp.r * ez;
      }

      fp.r = (fp.fx - bodyPos.x) * ex + (fp.fz - bodyPos.z) * ez;
      const e = fp.r - t.r;
      const rDot = -(bodyVel.x * ex + bodyVel.z * ez);
      const A = PHYS.k * e + PHYS.c * (rDot - t.rdot);
      fp.force = A;

      const limit = fp.slipping ? PHYS.hyst * muS * footNormal : muS * footNormal;
      if (Math.abs(A) <= limit) {
        fp.slipping = false;
        Physics.addForceAtPoint(A * ex, 0, A * ez, fp.fx, 0, fp.fz);
      } else {
        const vfx = bodyVel.x + t.rdot * ex;
        const vfz = bodyVel.z + t.rdot * ez;
        const vfl = Math.hypot(vfx, vfz);
        const fReg = vfl / (vfl + 1.0);
        if (vfl > 1e-3) {
          Physics.addForceAtPoint(
            -muK * footNormal * fReg * (vfx / vfl), 0,
            -muK * footNormal * fReg * (vfz / vfl),
            fp.fx, 0, fp.fz
          );
        }
        fp.pinned = false;
        fp.r += (t.r - fp.r) * Math.min(1, dt / PHYS.tauSlip);
        fp.slipping = true;
      }
    }
    Physics.step();
  }

  const PHYS_M = 2.0, PHYS_G = 980; // 机身质量kg, 重力cm/s²

  // ---------- 方位参考环 (挂在机器人上, 随机器人移动) ----------
  const azGroup = new THREE.Group();
  azGroup.position.y = 0.05;
  (function buildAzimuthRing() {
    const pts = [];
    for (let i = 0; i <= 128; i++) {
      const a = (i / 128) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(a) * 32, 0, Math.sin(a) * 32));
    }
    const ring = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0x4a5066, transparent: true, opacity: 0.55 })
    );
    azGroup.add(ring);

    const labels = { 0: '前 0°', 60: '60°', 120: '120°', 180: '后 180°', 240: '240°', 300: '300°' };
    for (let d = 0; d < 360; d += 60) {
      const a = d * Math.PI / 180;
      const tick = new THREE.Mesh(
        new THREE.BoxGeometry(3.4, 0.18, 0.18),
        new THREE.MeshBasicMaterial({ color: 0x66708c })
      );
      tick.position.set(Math.cos(a) * 31, 0.05, Math.sin(a) * 31);
      tick.rotation.y = -a + Math.PI / 2;   // 刻度沿径向
      azGroup.add(tick);

      const spr = makeTextSprite(labels[d], '#8fa2c8');
      spr.position.set(Math.cos(a) * 36, 0, Math.sin(a) * 36);
      azGroup.add(spr);
    }
  })();
  robot.root.add(azGroup);

  // ---------- 目标方向箭头 (φ) ----------
  const phiArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0.6, 0), 30, 0x4fc3f7, 6.5, 3.5
  );
  robot.root.add(phiArrow);

  // ---------- 推力矢量 (每腿) ----------
  const forceArrows = [];
  for (let i = 0; i < 6; i++) {
    const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 3, 0x6fd08c, 2.0, 1.2);
    a.visible = false;
    scene.add(a);
    forceArrows.push(a);
  }

  // ---------- 轨迹 ----------
  const TRAIL_CAP = 600;
  const TRAIL_COLORS = [0x4fc3f7, 0x81c784, 0xffd54f, 0xba68c8, 0xff8a65, 0x4dd0e1];
  function makeTrail(color) {
    const pos = new Float32Array(TRAIL_CAP * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setDrawRange(0, 0);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.7 }));
    line.frustumCulled = false;
    return { line, pos, count: 0 };
  }
  function trailPush(tr, x, y, z) {
    if (tr.count >= TRAIL_CAP) { tr.pos.copyWithin(0, 3, tr.count * 3); tr.count = TRAIL_CAP - 1; }
    tr.pos[tr.count * 3] = x;
    tr.pos[tr.count * 3 + 1] = y;
    tr.pos[tr.count * 3 + 2] = z;
    tr.count++;
    tr.line.geometry.attributes.position.needsUpdate = true;
    tr.line.geometry.setDrawRange(0, tr.count);
  }
  function trailClear(tr) { tr.count = 0; tr.line.geometry.setDrawRange(0, 0); }

  const footTrails = [];
  for (let i = 0; i < 6; i++) {
    footTrails.push(makeTrail(TRAIL_COLORS[i]));
    scene.add(footTrails[i].line);
  }
  const bodyTrail = makeTrail(0xffffff);
  scene.add(bodyTrail.line);

  // ---------- 工具 ----------
  function makeTextSprite(text, color) {
    const cv = document.createElement('canvas');
    cv.width = 256; cv.height = 128;
    const ctx = cv.getContext('2d');
    ctx.clearRect(0, 0, 256, 128);
    ctx.font = 'bold 54px "Segoe UI", "Microsoft YaHei", Arial, sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    ctx.fillText(text, 128, 64);
    const tex = new THREE.CanvasTexture(cv);
    const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
    spr.scale.set(3.6, 1.8, 1);
    return spr;
  }

  const MODE_LABEL = { tripod: '三角步态', ripple: '波纹步态', bipod: '对腿步态' };

  // ---------- HUD ----------
  const hudBody = document.getElementById('hudBody');
  const hudRows = [];
  for (let i = 0; i < 6; i++) {
    const tr = document.createElement('tr');
    for (let k = 0; k < 6; k++) tr.appendChild(document.createElement('td'));
    hudBody.appendChild(tr);
    hudRows.push(tr.children);
  }
  const statusEl = document.getElementById('status');
  const STATE_LABEL = { stance: '支撑', swing: '摆动', hold: '悬停' };
  const STATE_COLOR = { stance: '#6fd08c', swing: '#f0a04b', hold: '#8b93a8' };
  function updateHUD() {
    for (let i = 0; i < 6; i++) {
      const leg = robot.legs[i];
      const cells = hudRows[i];
      const slip = leg.state === 'stance' && leg.slipping;
      cells[0].textContent = 'L' + (i + 1) + '(' + leg.azimuthDeg + '°)';
      cells[1].textContent = leg.coeff.toFixed(2);
      cells[2].textContent = slip ? '支撑·滑移' : (STATE_LABEL[leg.state] || '—');
      cells[2].style.color = slip ? '#ff6b6b' : STATE_COLOR[leg.state] || '#fff';
      cells[3].textContent = Math.round(leg.r * 10);
      cells[4].textContent = leg.th1Deg.toFixed(1);
      cells[5].textContent = leg.th2Deg.toFixed(1);
    }
    let axisTxt = '';
    if (S.mode === 'bipod') {
      axisTxt = ' · 对轴 ' + CONFIG.azimuths[planner.PAIRS[S.pairIndex][0]] + '°/' +
        CONFIG.azimuths[planner.PAIRS[S.pairIndex][1]] + '°';
    }
    const speed = Math.hypot(bodyVel.x, bodyVel.z);
    statusEl.innerHTML = MODE_LABEL[S.mode] + ' · 方向 φ=' + S.phiDeg + '° · 摩擦 μ=' + S.mu.toFixed(2) +
      ' · 机身速度 ' + speed.toFixed(1) + ' cm/s' + axisTxt;
  }
  updateHUD();

  // ---------- UI 绑定 ----------
  const $ = id => document.getElementById(id);
  const panel = {
    phi: $('phi'), phiVal: $('phiVal'), speed: $('speed'), stride: $('stride'),
    lift: $('lift'), bh: $('bh'), mu: $('mu'), muVal: $('muVal'),
    follow: $('follow'), auto: $('autorotate'),
    trails: $('showTrails'), body: $('showBody'), forces: $('showForces'), az: $('showAz'),
    playPause: $('playPause'), reset: $('reset'), pairRow: $('pairRow')
  };

  document.querySelectorAll('input[name="mode"]').forEach(r => {
    r.addEventListener('change', () => {
      S.mode = r.value;
      panel.pairRow.style.display = S.mode === 'bipod' ? 'block' : 'none';
    });
  });
  panel.pairRow.querySelectorAll('.pairBtn').forEach(btn => {
    btn.addEventListener('click', () => {
      S.pairIndex = parseInt(btn.dataset.pair, 10);
      panel.pairRow.querySelectorAll('.pairBtn').forEach(b => b.classList.toggle('active', b === btn));
    });
  });
  panel.phi.addEventListener('input', () => { S.phiDeg = parseFloat(panel.phi.value); panel.phiVal.textContent = S.phiDeg + '°'; });
  panel.speed.addEventListener('input', () => { S.speed = parseFloat(panel.speed.value); });
  panel.stride.addEventListener('input', () => { S.stride = parseFloat(panel.stride.value); });
  panel.lift.addEventListener('input', () => { S.lift = parseFloat(panel.lift.value); });
  panel.bh.addEventListener('input', () => { S.bodyHeight = parseFloat(panel.bh.value); });
  panel.mu.addEventListener('input', () => {
    S.mu = parseFloat(panel.mu.value);
    panel.muVal.textContent = S.mu.toFixed(2);
  });
  panel.follow.addEventListener('change', () => {
    S.follow = panel.follow.checked;
    if (S.follow) camOffset.copy(camera.position).sub(controls.target);
  });
  panel.auto.addEventListener('change', () => { S.autoRotate = panel.auto.checked; controls.autoRotate = S.autoRotate; });
  panel.trails.addEventListener('change', () => {
    S.showFootTrails = panel.trails.checked;
    footTrails.forEach(tr => { tr.line.visible = S.showFootTrails; });
  });
  panel.body.addEventListener('change', () => {
    S.showBodyTrail = panel.body.checked;
    bodyTrail.line.visible = S.showBodyTrail;
  });
  panel.forces.addEventListener('change', () => { S.showForces = panel.forces.checked; });
  panel.az.addEventListener('change', () => { S.showAzimuths = panel.az.checked; azGroup.visible = S.showAzimuths; });
  panel.playPause.addEventListener('click', () => {
    S.running = !S.running;
    panel.playPause.textContent = S.running ? '⏸ 暂停' : '▶ 继续';
  });
  panel.reset.addEventListener('click', () => {
    if (physicsOK && Physics.ready()) {
      Physics.setPosition(0, S.bodyHeight, 0);
    }
    for (const fp of foot) {
      fp.r = null; fp.pinned = false; fp.fx = 0; fp.fz = 0; fp.force = 0; fp.slipping = false;
    }
    trailClear(bodyTrail);
    footTrails.forEach(trailClear);
  });

  document.querySelectorAll('.q').forEach(btn => {
    btn.addEventListener('click', () => {
      S.phiDeg = parseFloat(btn.dataset.phi);
      panel.phi.value = S.phiDeg;
      panel.phiVal.textContent = S.phiDeg + '°';
    });
  });

  // ---------- 视角预设 / 跟随 ----------
  const camOffset = new THREE.Vector3(0, 0, 0);
  function setCam(pos, target) {
    if (S.follow) camOffset.copy(pos).sub(target);
    camera.position.copy(pos);
    controls.target.copy(target);
  }
  const camTarget = () => robot.root.position.clone().setY(8);
  $('camDefault').addEventListener('click', () => setCam(new THREE.Vector3(85, 62, 85), camTarget()));
  $('camTop').addEventListener('click', () => setCam(new THREE.Vector3(0, 95, 0.1), camTarget()));
  $('camSide').addEventListener('click', () => setCam(new THREE.Vector3(0.1, 30, 78), camTarget()));
  $('camFront').addEventListener('click', () => setCam(new THREE.Vector3(78, 30, 0.1), camTarget()));

  // 键盘: ←/→ 方向, ↑/↓ 速度
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    if (e.key === 'ArrowLeft')  { S.phiDeg = (S.phiDeg + 355) % 360; syncPhiUI(); }
    else if (e.key === 'ArrowRight') { S.phiDeg = (S.phiDeg + 5) % 360; syncPhiUI(); }
    else if (e.key === 'ArrowUp')   { S.speed = Math.min(1, S.speed + 0.05); panel.speed.value = S.speed; }
    else if (e.key === 'ArrowDown') { S.speed = Math.max(0.05, S.speed - 0.05); panel.speed.value = S.speed; }
  });
  function syncPhiUI() { panel.phi.value = S.phiDeg; panel.phiVal.textContent = S.phiDeg + '°'; }

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // ---------- 主循环 ----------
  const clock = new THREE.Clock();
  let hudTimer = 0;
  const tmp = new THREE.Vector3();

  function animate() {
    requestAnimationFrame(animate);
    const dt = Math.min(clock.getDelta(), 0.05);
    if (S.running) S.t += dt;

    const sweepMag = (CONFIG.R_HI - CONFIG.R_LO) * S.stride;
    const cycleT = Math.min(4.0, Math.max(0.6, 1.8 / (0.3 + S.speed * 1.2)));
    const res = planner.compute(S.mode, S.phiDeg, S.t, {
      sweepMag, cycleT, lift: S.lift, pairIndex: S.pairIndex, bodyHeight: S.bodyHeight
    });

    // 摩擦动力学: Rapier 刚体 + 足端弹簧-阻尼力
    if (S.running) {
      if (physicsOK) {
        stepPhysics(res, dt);
      } else {
        // 无物理模式: 直接用步态规划器的足端位置
        for (let i = 0; i < 6; i++) {
          foot[i].r = res.targets[i].r;
          foot[i].force = 0;
          foot[i].slipping = false;
        }
      }
    }

    // 从 Rapier 读取机身实际状态, 同步到 Three.js
    if (physicsOK && Physics.ready()) {
      var bp = Physics.getPosition();
      var bv = Physics.getLinvel();
      bodyPos.x = bp.x; bodyPos.y = bp.y; bodyPos.z = bp.z;
      bodyVel.x = bv.x; bodyVel.y = bv.y; bodyVel.z = bv.z;
    }

    // 用"实际"足端位置 (含滑移量) 渲染腿
    for (let i = 0; i < 6; i++) {
      res.targets[i].r = foot[i].r;
      res.targets[i].slipping = foot[i].slipping;
    }
    robot.setPose(S.bodyHeight, res.targets);
    for (let i = 0; i < 6; i++) robot.legs[i].slipping = foot[i].slipping;
    robot.root.position.set(bodyPos.x, S.bodyHeight, bodyPos.z);

    // 目标方向箭头
    const ph = S.phiDeg * Math.PI / 180;
    phiArrow.setDirection(new THREE.Vector3(Math.cos(ph), 0, Math.sin(ph)));

    // 轨迹
    if (S.showBodyTrail) trailPush(bodyTrail, bodyPos.x, 0.1, bodyPos.z);
    if (S.showFootTrails) {
      for (let i = 0; i < 6; i++) {
        const leg = robot.legs[i];
        const a = leg.azimuthDeg * Math.PI / 180;
        trailPush(footTrails[i], bodyPos.x + leg.r * Math.cos(a), Math.max(0, leg.y), bodyPos.z + leg.r * Math.sin(a));
      }
    }

    // 推力矢量 (支撑腿: 方向 = 实际推力方向, 长度 ∝ |F|/静摩擦极限; 滑移时红色)
    for (let i = 0; i < 6; i++) {
      const arrow = forceArrows[i];
      const leg = robot.legs[i];
      const fp = foot[i];
      const cap = muS * footNormal;
      if (S.showForces && leg.state === 'stance' && Math.abs(fp.force) > 0.5) {
        const a = leg.azimuthDeg * Math.PI / 180;
        const s = Math.sign(fp.force);
        arrow.setDirection(new THREE.Vector3(Math.cos(a) * s, 0, Math.sin(a) * s));
        arrow.setLength(1.5 + 13 * Math.min(Math.abs(fp.force), cap) / Math.max(cap, 1e-6), 2.5, 1.5);
        arrow.setColor(fp.slipping ? 0xff5555 : 0x6fd08c);
        arrow.position.set(bodyPos.x + leg.r * Math.cos(a), 0.3, bodyPos.z + leg.r * Math.sin(a));
        arrow.visible = true;
      } else {
        arrow.visible = false;
      }
    }

    // 跟随相机
    if (S.follow) {
      controls.target.lerp(tmp.set(bodyPos.x, 8, bodyPos.z), 0.08);
      camera.position.copy(controls.target).add(camOffset);
    }

    // HUD (节流)
    hudTimer -= dt;
    if (hudTimer <= 0) { hudTimer = 0.12; updateHUD(); }

    controls.update();
    renderer.render(scene, camera);
  }

  animate();
})();
