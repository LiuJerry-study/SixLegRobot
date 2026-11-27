/* ============================================================
 * main.js — 场景、相机、UI 绑定、主循环 + 三种模式切换: 触地(ground)/悬空(hang)/真机遥控(rc)
 * 依赖: three.min.js, OrbitControls.js, robot.js, gaits.js, physics.js
 * 面板: control-panel.js(真机遥控 rc 模式) / twin-panel.js(悬空孪生 hang 模式) 各自轮询 window.SimCore 就绪后自举,
 *       分别挂到 window.RcPanel / window.TwinPanel (孪生面板还会注册 SimCore.onTwinFrame)
 * 单位: 厘米 (与固件 Config.h 一致) */
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
    bodyHeight: 11.0,  // 默认≈固件站立高 (髋下 10.0cm + hipDrop 1.0)
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

  // ============================================================
  const HANG_Y = 46;
  let appMode = 'ground';
  const channelCount = RobotModel.channelCount;
  const twin = {
    angles: new Array(channelCount).fill(null),
    cur: new Array(channelCount).fill(null),
    lastSeenAt: 0,
    demo: false,
    frame: null,
    groundPreview: false
  };
  twin.setChannel = function (ch, deg) {
    if (!MotionFrame.channels.has(ch) || !Number.isFinite(deg)) return;
    twin.angles[ch] = twin.cur[ch] = deg;
    twin.lastSeenAt = performance.now();
  };
  twin.reset = function () {
    twin.angles.fill(null);
    twin.cur.fill(null);
    twin.frame = null;
    twin.lastSeenAt = 0;
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

  // 悬空吊架（仅悬空孪生模式显示：上方锚点 + 3 根吊索 → 机体顶面）
  const hangStand = new THREE.Group();
  hangStand.visible = false;
  scene.add(hangStand);
  (function buildHangStand() {
    const topY = HANG_Y + 30;
    const ball = new THREE.Mesh(
      new THREE.SphereGeometry(1.5, 10, 10),
      new THREE.MeshStandardMaterial({ color: 0x7c849c, roughness: 0.5 })
    );
    ball.position.set(0, topY, 0);
    hangStand.add(ball);
    const cableMat = new THREE.LineBasicMaterial({ color: 0x8a93a8, transparent: true, opacity: 0.9 });
    const att = HANG_Y + 1.0;                 // 机体顶面(中心+半厚)
    for (let i = 0; i < 3; i++) {
      const a = (90 + i * 120) * Math.PI / 180;   // 避开机头(0°)
      const r = 3.4;
      const pts = [
        new THREE.Vector3(0, topY, 0),
        new THREE.Vector3(Math.cos(a) * r, att, -Math.sin(a) * r)
      ];
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts), cableMat
      );
      hangStand.add(line);
    }
  })();

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
  // μ静 = S.mu, μ动 = 0.85·S.mu (同 PHYS.hyst); footNormal = 单腿法向力 (每帧在 stepPhysics 里算)
  let muS = S.mu, muK = 0.85 * S.mu, footNormal = 0;
  var bodyPos = { x: 0, y: S.bodyHeight, z: 0 };
  var bodyVel = { x: 0, y: 0, z: 0 };

  const PHYS_M = 2.0, PHYS_G = 980; // 机身质量kg, 重力cm/s²

  // 初始化 Rapier 机器人位置
  if (physicsOK) Physics.setPosition(0, S.bodyHeight, 0);

  // 垂直支撑 / 姿态稳定增益 —— 按刚体实际质量推导:
  //   高度弹簧-阻尼: 支撑腿均分重量 + 悬浮于 S.bodyHeight
  //   水平恢复 + 偏航锁定: 机身水平、机头恒朝 0° (本构型无法偏航)
  let MASS = PHYS_M;
  if (physicsOK && Physics.ready()) {
    const m = Physics.getMass();
    if (isFinite(m) && m > 0) MASS = m;
  }
  const KV = MASS * Math.pow(2 * Math.PI * 1.8, 2);   // 高度弹簧刚度 (≈1.8Hz)
  const CV = 1.7 * Math.sqrt(KV * MASS);              // 高度阻尼 (ζ≈0.85)

  // Rapier 物理步进: 每帧对每条支撑腿施加弹簧-阻尼力到刚体
  function stepPhysics(res, dt) {
    if (!physicsOK || !Physics.ready()) return;
    muS = S.mu; muK = 0.85 * S.mu;
    const nStance = res.targets.reduce((k, t) => k + (t.state === 'stance' ? 1 : 0), 0);
    if (nStance === 0) { Physics.step(); return; }
    // ⚠ 按固定 2.0kg(PHYS_M) 估算单腿法向力; 垂直支撑/增益用的是实测 MASS = Physics.getMass()
    footNormal = PHYS_M * PHYS_G / Math.max(1, nStance);

    const bodyPos = Physics.getPosition();
    const bodyVel = Physics.getLinvel();
    const q = Physics.getRotation();
    const attitude = new THREE.Quaternion(q.x, q.y, q.z, q.w);
    const omega = Physics.getAngvel();

    for (let i = 0; i < 6; i++) {
      const t = res.targets[i], fp = foot[i];
      const heading = MotionFrame.direction(CONFIG.azimuths[i]);
      const axis = new THREE.Vector3(heading.x, 0, heading.z).applyQuaternion(attitude);
      const radialLength = Math.hypot(axis.x, axis.z) || 1;
      const ex = axis.x / radialLength, ez = axis.z / radialLength;
      const offset = new THREE.Vector3(heading.x * t.r, t.y - S.bodyHeight, heading.z * t.r).applyQuaternion(attitude);
      fp.groundY = bodyPos.y + offset.y;
      fp.vy = bodyVel.y + omega.z * offset.x - omega.x * offset.z;

      if (t.state !== 'stance' || fp.groundY > 0.5) {
        fp.pinned = false; fp.r = t.r; fp.force = 0; fp.slipping = false;
        continue;
      }

      if (!fp.pinned) {
        fp.pinned = true;
        if (fp.r === null) fp.r = t.r;
        fp.fx = bodyPos.x + offset.x;
        fp.fz = bodyPos.z + offset.z;
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

    // ---- 垂直支撑: 支撑腿均分重量 + 高度弹簧-阻尼, 力作用于足端点(天然提供倾斜恢复力矩) ----
    for (let i = 0; i < 6; i++) {
      const fp = foot[i];
      if (res.targets[i].state === 'stance' && fp.groundY <= 0) {
        const force = Math.max(0, -KV * fp.groundY - CV * fp.vy) / nStance;
        Physics.addForceAtPoint(0, force, 0, fp.fx, 0, fp.fz);
      }
    }

    Physics.step();
  }

  // ---------- 方位参考环 (挂在机器人上, 随机器人移动) ----------
  const azGroup = new THREE.Group();
  azGroup.position.y = 0.05;
  (function buildAzimuthRing() {
    const pts = [];
    for (let i = 0; i <= 128; i++) {
      const a = (i / 128) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(a) * 32, 0, -Math.sin(a) * 32));
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
      tick.position.set(Math.cos(a) * 31, 0.05, -Math.sin(a) * 31);
      tick.rotation.y = -a + Math.PI / 2;   // 刻度沿径向
      azGroup.add(tick);

      const spr = makeTextSprite(labels[d], '#8fa2c8');
      spr.position.set(Math.cos(a) * 36, 0, -Math.sin(a) * 36);
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
    statusEl.innerHTML = (physicsOK ? '接触估算 · ' : '理想方向预览 · ') + MODE_LABEL[S.mode] + ' · 方向 φ=' + S.phiDeg + '° · 摩擦 μ=' + S.mu.toFixed(2) +
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
  panel.reset.addEventListener('click', resetSim);

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
  // 相机预设瞄准机身当前位置(悬空时为吊架高度), 而非写死地面 y=8
  const camTarget = () => robot.root.position.clone();
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

  // ============================================================
  // 应用模式切换(两模式互斥): hang = 悬空孪生 | ground = 触地模拟 + 真机遥控
  // ============================================================
  let twinHudTimer = 0;

  // 地面相关视觉显隐(悬空时关掉 HUD/轨迹/推力矢量/方位环/方向箭头)
  function applyGroundFx(show) {
    const hudEl = document.getElementById('hud');
    if (hudEl) hudEl.style.display = show ? '' : 'none';
    azGroup.visible = show && S.showAzimuths;
    phiArrow.visible = show;
    if (!show) forceArrows.forEach(a => { a.visible = false; });
    for (let i = 0; i < footTrails.length; i++) footTrails[i].line.visible = show && S.showFootTrails;
    bodyTrail.line.visible = show && S.showBodyTrail;
  }

  // 触地复位：机身回原点、清轨迹与足端钉点（#reset 与切回触地模式共用）
  let physicsTime = 0;
  const physicsStep = 1 / 60;
  function resetSim() {
    physicsTime = 0;
    if (physicsOK && Physics.ready()) {
      Physics.setPosition(0, S.bodyHeight, 0);
    }
    bodyPos.x = 0; bodyPos.y = S.bodyHeight; bodyPos.z = 0;
    bodyVel.x = 0; bodyVel.y = 0; bodyVel.z = 0;
    for (const fp of foot) {
      fp.r = null; fp.pinned = false; fp.fx = 0; fp.fz = 0; fp.force = 0; fp.slipping = false;
    }
    trailClear(bodyTrail);
    footTrails.forEach(trailClear);
  }

  function setAppMode(mode) {
    if ((mode !== 'hang' && mode !== 'ground' && mode !== 'rc') || mode === appMode) return;
    const prev = appMode;
    appMode = mode;
    const hangMode = mode === 'hang';
    const rcMode = mode === 'rc';

    // 三模式互斥（串口独占）：离开谁就断开谁的串口；离开遥控模式额外复位圆形移动键
    // ⚠ 顺序: RcPanel.disconnect() 内部会先补发一条 stop（真机行走中不能直接关串口）
    if (!hangMode && window.TwinPanel && typeof window.TwinPanel.disconnect === 'function') window.TwinPanel.disconnect();
    if (!rcMode && window.RcPanel && typeof window.RcPanel.disconnect === 'function') window.RcPanel.disconnect();
    if (prev === 'rc' && !rcMode && window.RcPanel && typeof window.RcPanel.deactivate === 'function') window.RcPanel.deactivate();

    // 面板显隐
    const go = document.getElementById('groundOnly');   // 步态模式/行进 = 纯仿真参数(遥控模式收起)
    const ps = document.getElementById('playSection');
    const tp = document.getElementById('twinPanel');
    const rcs = document.getElementById('rcSection');    // control-panel.js 建的真机遥控面板
    const ds = document.getElementById('dispSection');   // 显示区：轨迹/推力/方位环 = 地面专用
    const nt = document.getElementById('notes');         // 长说明 = 地面为主
    const fr = document.getElementById('followRow');     // 跟随机器人 = 地面专用(悬空不动)
    if (go) go.classList.toggle('hidden', hangMode || rcMode);
    if (ps) ps.classList.toggle('hidden', hangMode);
    if (tp) tp.classList.toggle('hidden', !hangMode);
    if (rcs) rcs.style.display = rcMode ? '' : 'none';   // 遥控面板只在真机遥控模式显示(且排在最上面)
    if (ds) ds.classList.toggle('hidden', hangMode);
    if (nt) nt.classList.toggle('hidden', hangMode || rcMode);
    if (fr) fr.classList.toggle('hidden', hangMode);
    grid.visible = !hangMode;            // 悬空时去掉地面网格，画面干净
    const mg = document.getElementById('modeGroundBtn');
    const mh = document.getElementById('modeHangBtn');
    const mr = document.getElementById('modeRcBtn');
    if (mg) mg.classList.toggle('active', !hangMode && !rcMode);
    if (mh) mh.classList.toggle('active', hangMode);
    if (mr) mr.classList.toggle('active', rcMode);

    if (hangMode) {
      // 悬空：机身固定在空中 + 吊架 + 关闭地面特效
      hangStand.visible = true;
      applyGroundFx(false);
      if (physicsOK && Physics.ready()) Physics.setPosition(0, HANG_Y, 0);
      robot.root.position.set(0, HANG_Y, 0);
      robot.root.quaternion.identity();
      robot.applyServoAngles(twin.cur);   // 初始即默认双 90° 占位(非站姿), 数据到了自然过渡
      controls.target.set(0, HANG_Y, 0);
    } else if (rcMode && prev === 'ground') {
      // 触地 → 真机遥控：同一套地面场景，保留机身位置与相机，只把右栏换成遥控面板
      hangStand.visible = false;
      applyGroundFx(true);
    } else {
      // 悬空 → 触地 / 悬空 → 真机遥控：回地面并从干净起点重新开始
      hangStand.visible = false;
      applyGroundFx(true);
      resetSim();
      S.running = true;
      panel.playPause.textContent = '⏸ 暂停';
      controls.target.set(0, 8, 0);
    }
  }

  // 悬空孪生逐帧推进：离线演示 / 平滑追尾 / 渲染 / 机身固定 / 面板刷新
  function stepHangTwin(dt) {
    if (window.ReplayPanel) window.ReplayPanel.update(performance.now());
    if (twin.demo) {
      const now = performance.now();
      const [x, y] = RobotModel.stand;
      const stand = Robot.solveIK(-x, -y);
      const degrees = 180 / Math.PI;
      const logical = [90 - stand.th1 * degrees, 180 - stand.th2 * degrees];
      RobotModel.pins.forEach((pair, leg) => pair.forEach((ch, joint) => {
        twin.setChannel(ch, logical[joint] + RobotModel.offsets[leg][joint]);
      }));
      const phase = (now % 3200) / 3200;
      const index = Math.floor(now / 3200) % MotionFrame.channels.size;
      const channel = Array.from(MotionFrame.channels.keys())[index];
      twin.setChannel(channel, 50 + 100 * (1 - Math.abs(2 * phase - 1)));
    }
    const frame = twin.frame;
    robot.applyServoAngles(twin.cur, frame ? frame.mirror : 1, frame && frame.support ? frame.support : null);
    const supportFoot = frame?.feet.find((foot, index) => foot && frame.support & (1 << index));
    const groundHeight = CONFIG.hipDrop - (supportFoot?.y ?? RobotModel.stand[1]);
    robot.root.position.set(0, twin.groundPreview ? groundHeight : HANG_Y, 0);
    robot.root.quaternion.identity();
    grid.visible = twin.groundPreview;
    hangStand.visible = !twin.groundPreview;
    // 保险：悬空时地面视觉物(方位环/方向箭头/推力)一律不显示
    azGroup.visible = false;
    phiArrow.visible = false;
    forceArrows.forEach(a => { a.visible = false; });

    twinHudTimer -= dt;
    if (twinHudTimer <= 0) {
      twinHudTimer = 0.1;
      if (window.SimCore && typeof SimCore.onTwinFrame === 'function') SimCore.onTwinFrame();
    }
  }

  window.SimCore = {
    scene, camera, renderer, controls, CONFIG, S, robot,
    getCenter() {
      return { x: bodyPos.x, y: robot.root.position.y, z: bodyPos.z };
    },
    // 目标方向 φ（真机遥控的圆形移动键用）：只改方向箭头/滑块读数, 不动机身
    // （遥控模式只发串口、不驱动 3D 场景, 所以这里仅"指个方向"）
    setPhi(deg) {
      if (typeof deg !== 'number' || !isFinite(deg)) return;
      S.phiDeg = ((deg % 360) + 360) % 360;
      syncPhiUI();
    },

    // ===== 应用模式：'ground' 触地(无孪生) | 'hang' 悬空(有孪生) | 'rc' 真机遥控 =====
    getAppMode() { return appMode; },
    setAppMode,
    isHang() { return appMode === 'hang'; },
    isRc() { return appMode === 'rc'; },
    HANG_Y,

    // ===== 悬空数字孪生（镜像数据由 twin-panel.js 喂入） =====
    twin: {
      setChannel: (ch, deg) => twin.setChannel(ch, deg),
      resetVisual: () => twin.reset(),
      setDemo: (on) => { twin.demo = !!on; twin.reset(); },
      groundPreview(on) {
        twin.groundPreview = on;
        controls.target.set(0, on ? 8 : HANG_Y - 4, 0);
      },
      setFrame(frame) {
        twin.demo = false;
        twin.frame = frame;
        twin.angles = frame.outputs.map(command => command ? command.applied : null);
        twin.cur = twin.angles.slice();
        twin.lastSeenAt = performance.now();
      },
      frame: () => twin.frame,
      lastSeen: () => twin.lastSeenAt,
      knownCount: () => twin.angles.reduce((k, v) => k + (v === null ? 0 : 1), 0),
      labelOf(ch) {
        const {leg, joint} = MotionFrame.channels.get(ch);
        return 'ch' + ch + ' 腿' + leg + (joint ? '小腿' : '大腿');
      },
      targetOf: (ch) => twin.angles[ch],
      currentOf: (ch) => twin.cur[ch]
    },
    // 孪生面板每帧(节流)刷新的回调, 由 twin-panel.js 注册
    onTwinFrame: null,
    resetSim
  };

  // 模式开关按钮（顶部三个并列互斥：悬空 / 触地 / 真机遥控）
  document.getElementById('modeHangBtn').addEventListener('click', () => setAppMode('hang'));
  document.getElementById('modeGroundBtn').addEventListener('click', () => setAppMode('ground'));
  const modeRcBtnEl = document.getElementById('modeRcBtn');
  if (modeRcBtnEl) modeRcBtnEl.addEventListener('click', () => setAppMode('rc'));

  // ---------- 主循环 ----------
  const clock = new THREE.Clock();
  let hudTimer = 0;
  const tmp = new THREE.Vector3();

  function animate() {
    requestAnimationFrame(animate);
    try {
      const dt = Math.min(clock.getDelta(), 0.1);

      // ===== 悬空孪生（有孪生）：只镜像舵机, 不跑步态/物理/轨迹 =====
      if (appMode === 'hang') {
        stepHangTwin(dt);
        controls.update();
        renderer.render(scene, camera);
        return;
      }

      // 遥控姿态模式已下线（真机遥控改为纯串口摇杆）→ 场景永远按自动步态规划器推进
      if (S.running && !physicsOK) S.t += dt;

      const sweepMag = (CONFIG.R_HI - CONFIG.R_LO) * S.stride;
      const cycleT = Math.min(4.0, Math.max(0.6, 1.8 / (0.3 + S.speed * 1.2)));
      const plan = time => planner.compute(S.mode, S.phiDeg, time, {
        sweepMag, cycleT, lift: S.lift, pairIndex: S.pairIndex, bodyHeight: S.bodyHeight
      });
      let res = plan(S.t);

      // 摩擦动力学: Rapier 刚体 + 足端弹簧-阻尼力
      if (S.running) {
        if (physicsOK) {
          physicsTime += dt;
          while (physicsTime + 1e-9 >= physicsStep) {
            physicsTime -= physicsStep;
            S.t += physicsStep;
            res = plan(S.t);
            stepPhysics(res, physicsStep);
          }
        } else {
          // 无物理模式: 直接用步态规划器的足端位置 + 简单位移估算
          for (let i = 0; i < 6; i++) {
            foot[i].r = res.targets[i].r;
            foot[i].force = 0;
            foot[i].slipping = false;
          }
          if (res.dir) {
            const sf = S.mode === 'ripple' ? (1 - 1 / 6) : 0.5;
            const vScale = (sweepMag / (sf * cycleT)) * (S.mode === 'tripod' ? 2 / 3 : 0.5);
            const px = bodyPos.x, pz = bodyPos.z;
            bodyPos.x += res.dir.x * vScale * S.speed * dt;
            bodyPos.z -= res.dir.z * vScale * S.speed * dt;
            if (dt > 1e-6) {   // 由位移估算机身速度, 供 HUD 显示 (离线模式)
              bodyVel.x = (bodyPos.x - px) / dt;
              bodyVel.z = (bodyPos.z - pz) / dt;
              bodyVel.y = 0;
            }
          }
        }
      }

      // 从 Rapier 读机身实际状态(高度/姿态用物理值)同步到 Three.js
      let rootY = S.bodyHeight;
      if (physicsOK && Physics.ready()) {
        var bp = Physics.getPosition();
        var bv = Physics.getLinvel();
        var bq = Physics.getRotation();
        if (isFinite(bp.x) && isFinite(bp.y) && isFinite(bp.z)) {
          bodyPos.x = bp.x; bodyPos.y = bp.y; bodyPos.z = bp.z;
          rootY = bp.y;
        }
        if (isFinite(bv.x) && isFinite(bv.y) && isFinite(bv.z)) {
          bodyVel.x = bv.x; bodyVel.y = bv.y; bodyVel.z = bv.z;
        }
        if (isFinite(bq.x) && isFinite(bq.y) && isFinite(bq.z) && isFinite(bq.w)) {
          robot.root.quaternion.set(bq.x, bq.y, bq.z, bq.w);
        }
      } else {
        robot.root.quaternion.identity();
      }

      // NaN 防护: bodyPos 的 x/z 出现 NaN 时归零（y 不兜, 沿用物理值 rootY）
      if (!isFinite(bodyPos.x)) bodyPos.x = 0;
      if (!isFinite(bodyPos.z)) bodyPos.z = 0;

      // 用"实际"足端位置 (含滑移量) 渲染腿
      for (let i = 0; i < 6; i++) {
        res.targets[i].r = foot[i].r;
        res.targets[i].slipping = foot[i].slipping;
      }
      robot.setPose(rootY, res.targets);
      for (let i = 0; i < 6; i++) robot.legs[i].slipping = foot[i].slipping;
      robot.root.position.set(bodyPos.x, rootY, bodyPos.z);

      // 目标方向箭头
      const ph = S.phiDeg * Math.PI / 180;
      phiArrow.setDirection(new THREE.Vector3(Math.cos(ph), 0, -Math.sin(ph)));

      // 轨迹
      if (S.showBodyTrail) trailPush(bodyTrail, bodyPos.x, 0.1, bodyPos.z);
      if (S.showFootTrails) {
        for (let i = 0; i < 6; i++) {
          const leg = robot.legs[i];
          leg.foot.getWorldPosition(tmp);
          trailPush(footTrails[i], tmp.x, Math.max(0, tmp.y), tmp.z);
        }
      }

      // 推力矢量 (支撑腿: 方向 = 实际推力方向, 长度 ∝ |F|/静摩擦极限; 滑移时红色)
      // ⚠ 只有 Rapier 物理模式才有推力: 无物理模式 foot[i].force 恒为 0（footNormal 也是 0）, 箭头不会显示
      for (let i = 0; i < 6; i++) {
        const arrow = forceArrows[i];
        const leg = robot.legs[i];
        const fp = foot[i];
        const cap = muS * footNormal;
        if (S.showForces && leg.state === 'stance' && Math.abs(fp.force) > 0.5) {
          const heading = MotionFrame.direction(leg.azimuthDeg);
          const sign = Math.sign(fp.force);
          arrow.setDirection(new THREE.Vector3(heading.x * sign, 0, heading.z * sign));
          arrow.setLength(1.5 + 13 * Math.min(Math.abs(fp.force), cap) / Math.max(cap, 1e-6), 2.5, 1.5);
          arrow.setColor(fp.slipping ? 0xff5555 : 0x6fd08c);
          arrow.position.set(fp.fx, 0.3, fp.fz);
          arrow.visible = true;
        } else {
          arrow.visible = false;
        }
      }

      // 跟随相机: ⚠ 目标高度写死 y=8(旧口径, 当前默认站高 11.0), 跟随时视线偏低
      if (S.follow && isFinite(bodyPos.x) && isFinite(bodyPos.z)) {
        controls.target.lerp(tmp.set(bodyPos.x, 8, bodyPos.z), 0.08);
        camera.position.copy(controls.target).add(camOffset);
      }

      // HUD (节流)
      hudTimer -= dt;
      if (hudTimer <= 0) { hudTimer = 0.12; updateHUD(); }

      controls.update();
    } catch (e) {
      console.error('[animate] 渲染循环异常:', e);
    }
    renderer.render(scene, camera);
  }

  animate();
})();
