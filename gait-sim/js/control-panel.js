/* ============================================================
 * control-panel.js — 触地模式 · 真机遥控面板（Web Serial → ESP32）
 *
 * 勾选"🎯 目标点遥控"后点击/拖拽地面: 只取目标点到机心的径向距离 r(限幅 [R_LO, R_HI]),
 *   6 条腿足端同步落到半径 r 的目标环上, 同时发 goto <x> <y>
 *   (x = −(r−hipRadius), y = −drop; 与固件 SingleLeg 足端坐标约定一致)。
 * 另有: 站立/循环行走/停止/读状态、动作时长、手动命令、收发日志。
 * 固件须为 MODE_SIX_LEGS_SYNC; 串口独占(连接前关掉串口监视器), 须 http://localhost + Chrome/Edge。
 * 与悬空孪生互斥: 切到悬空时 main.js 调 window.RcPanel.disconnect()。
 * @SERVO 遥测行由 rx() 丢弃, 不进本日志。依赖 main.js 暴露的 window.SimCore。
 */
(function () {
  'use strict';

  // ================= 状态 =================
  const S = {
    port: null, reader: null, writer: null, connected: false,
    // ⚠ 死字段: manualOn 不写不读; targetR/drop 只写不读 (遥控开关状态看 SimCore.manual.on 与 #rcTargetChk)
    manualOn: false, targetR: 0, drop: 0,
    lastSentAt: 0, pendingSend: null,
    drag: false, downX: 0, downY: 0,
    opening: false   // 正在建立连接的标志（防重复点击 / 重复 open）
  };
  const LOG_MAX = 220;
  let ui = null, markers = null;

  function boot() {
    if (window.SimCore && window.SimCore.robot) { build(); return; }
    setTimeout(boot, 150);
  }

  // ================= UI 构建 =================
  function build() {
    const C = window.SimCore.CONFIG;
    const panel = document.getElementById('panel');
    const notes = document.getElementById('notes');

    const sec = document.createElement('section');
    sec.id = 'rcSection';
    const mk = (tag, cls, txt) => {
      const el = document.createElement(tag);
      if (cls) el.className = cls;
      if (txt !== undefined) el.textContent = txt;
      return el;
    };

    sec.appendChild(mk('h2', null, '🤖 真机遥控（串口）'));

    // ---- 连接区 ----
    const stRow = mk('div', 'row');
    const dot = mk('span', 'rcDot off');
    dot.id = 'rcDot';
    const stTxt = mk('span', null, '未连接');
    stTxt.id = 'rcConnTxt';
    stRow.appendChild(dot); stRow.appendChild(stTxt);
    sec.appendChild(stRow);

    const btnRow = mk('div', 'rcBtnRow');
    const btnConnect = mk('button', 'rcPrimary', '🔌 连接串口');
    btnConnect.id = 'rcConnect';
    btnRow.appendChild(btnConnect);
    sec.appendChild(btnRow);
    const connHint = mk('div', 'dim rcHint');
    connHint.id = 'rcConnHint';
    connHint.textContent = '提示：需先关闭 PlatformIO 串口监视器；固件须为 MODE_SIX_LEGS_SYNC；Chrome/Edge 打开';
    sec.appendChild(connHint);

    // ---- 快捷动作 ----
    sec.appendChild(mk('h2', null, '快捷动作'));
    const quickRow = mk('div', 'rcBtnRow');
    ['站立|stand', '循环行走|cycle', '停止|stop', '读状态|pos'].forEach(function (pair) {
      const p = pair.split('|');
      const b = mk('button', null, p[0]);
      b.addEventListener('click', function () { preset(p[1]); });
      quickRow.appendChild(b);
    });
    sec.appendChild(quickRow);

    // ---- 动作时长 ----
    const durRow = mk('div', 'row');
    durRow.appendChild(mk('span', null, '动作时长'));
    const durVal = mk('span', 'rcNum', '300 ms');
    const durIn = document.createElement('input');
    durIn.type = 'range'; durIn.min = 120; durIn.max = 900; durIn.step = 20; durIn.value = 300;
    durIn.addEventListener('input', function () {
      durVal.textContent = durIn.value + ' ms';
      clearTimeout(durIn._t);
      durIn._t = setTimeout(function () { send('dur ' + durIn.value); }, 250);
    });
    durRow.appendChild(durIn); durRow.appendChild(durVal);
    sec.appendChild(durRow);

    // ---- 手动命令 ----
    const manualRow = mk('div', 'row');
    const cmdIn = document.createElement('input');
    cmdIn.id = 'rcCmdIn';
    cmdIn.placeholder = '手动命令，如 pos / home / 0 90';
    cmdIn.type = 'text';
    const btnSend = mk('button', null, '发送');
    btnSend.addEventListener('click', function () {
      const v = cmdIn.value.trim();
      if (v) send(v);
      cmdIn.value = '';
    });
    cmdIn.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); btnSend.click(); }
    });
    manualRow.appendChild(cmdIn); manualRow.appendChild(btnSend);
    sec.appendChild(manualRow);

    // ---- 目标点遥控 ----
    sec.appendChild(mk('h2', null, '🎯 目标点遥控（点击地面 = 全体足端目标）'));
    const tgtRow = mk('div', 'row');
    const tgtCheck = document.createElement('input');
    tgtCheck.type = 'checkbox';
    tgtCheck.id = 'rcTargetChk';
    const tgtLbl = mk('label', 'rcInline');
    tgtLbl.appendChild(tgtCheck);
    tgtLbl.appendChild(mk('span', null, '启用（点击/拖拽地面设定目标环）'));
    tgtRow.appendChild(tgtLbl);
    sec.appendChild(tgtRow);
    const meta = mk('div', 'rcMeta');
    meta.id = 'rcMeta';
    meta.innerHTML = '目标半径 r = — cm　髋外 a = — cm　固件 goto —';
    sec.appendChild(meta);
    const lastCmd = mk('div', 'rcMeta');
    lastCmd.id = 'rcLastCmd';
    sec.appendChild(lastCmd);

    // ---- 日志 ----
    sec.appendChild(mk('h2', null, '串口日志'));
    const logBox = mk('pre', 'rcLog');
    logBox.id = 'rcLog';
    sec.appendChild(logBox);
    const logBtn = mk('button', null, '清空日志');
    logBtn.addEventListener('click', function () { logBox.textContent = ''; logs.length = 0; });
    sec.appendChild(logBtn);

    panel.insertBefore(sec, notes);

    ui = { dot, stTxt, btnConnect, durVal, durIn, cmdIn, tgtCheck, meta, lastCmd, logBox };
    log('遥控面板就绪（需 http://localhost 打开 + Chrome/Edge 才能用串口）');

    markers = createMarkers();

    // ---- 事件 ----
    btnConnect.addEventListener('click', onConnectClick);
    tgtCheck.addEventListener('change', onTargetToggle);
    const canvas = window.SimCore.renderer.domElement;
    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    window.addEventListener('beforeunload', function () { if (S.connected) disconnect(); });

    // 每帧让目标环跟随机心（机心可能因之前的步态/复位而移动）
    (function followLoop() {
      if (markers && markers.group.visible) {
        const c = window.SimCore.getCenter();
        markers.group.position.set(c.x, 0.15, c.z);
      }
      requestAnimationFrame(followLoop);
    })();
  }

  // ================= 3D 目标环标记（机心相对坐标） =================
  function createMarkers() {
    const THREE = window.THREE;
    const group = new THREE.Group();
    group.visible = false;

    // 目标环: 单位圆 + 缩放 = 半径 r (XZ 平面)
    const ringPts = [];
    for (let i = 0; i <= 96; i++) {
      const a = (i / 96) * Math.PI * 2;
      ringPts.push(new THREE.Vector3(Math.cos(a), Math.sin(a), 0));
    }
    const ring = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(ringPts),
      new THREE.LineBasicMaterial({ color: 0xffb347, transparent: true, opacity: 0.9 })
    );
    ring.rotation.x = -Math.PI / 2;   // 平躺到地面
    group.add(ring);

    // 6 个足端落点（方位角与各腿一致）
    const dotMat = new THREE.MeshBasicMaterial({ color: 0xffb347 });
    const dots = [];
    for (let i = 0; i < 6; i++) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.55, 10, 10), dotMat);
      group.add(m);
      dots.push(m);
    }

    // 点击位置竖针（机心相对）
    const pin = new THREE.Mesh(
      new THREE.CylinderGeometry(0.18, 0.18, 2.6, 8),
      new THREE.MeshBasicMaterial({ color: 0x7fd0ff })
    );
    pin.position.y = 1.3;
    pin.visible = false;
    group.add(pin);

    window.SimCore.scene.add(group);
    return { group, ring, dots, pin };
  }

  // relX/relZ = 点击点相对机心的地面坐标; 传 null 只画环不画针
  function updateMarkers(r, relX, relZ) {
    if (!markers) return;
    const C = window.SimCore.CONFIG;
    markers.ring.scale.set(r, r, 1);
    for (let i = 0; i < 6; i++) {
      const a = C.azimuths[i] * Math.PI / 180;
      markers.dots[i].position.set(Math.cos(a) * r, 0.4, Math.sin(a) * r);
    }
    if (typeof relX === 'number' && typeof relZ === 'number') {
      markers.pin.position.x = relX;
      markers.pin.position.z = relZ;
      markers.pin.visible = true;
    } else {
      markers.pin.visible = false;
    }
  }

  // ================= 地面点击/拖拽拾取 =================
  function onDown(e) {
    if (!ui || !ui.tgtCheck.checked || e.button !== 0) return;
    S.drag = false;
    S.downX = e.clientX; S.downY = e.clientY;
    pickFromPointer(e);
  }
  function onMove(e) {
    if (!ui || !ui.tgtCheck.checked || e.buttons !== 1) return;
    if (!S.drag && Math.hypot(e.clientX - S.downX, e.clientY - S.downY) > 4) S.drag = true;
    if (S.drag) pickFromPointer(e);
  }
  function onUp() { S.drag = false; }

  function pickFromPointer(e) {
    const THREE = window.THREE;
    const C = window.SimCore.CONFIG;
    const core = window.SimCore;
    const canvas = core.renderer.domElement;
    const rect = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, core.camera);
    const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const hit = new THREE.Vector3();
    if (!ray.ray.intersectPlane(ground, hit)) return;

    const c = core.getCenter();
    const d = Math.hypot(hit.x - c.x, hit.z - c.z);
    if (d < 0.5) return;                 // 点在机心正下方：忽略
    const r = Math.max(C.R_LO, Math.min(C.R_HI, d));
    setTarget(r, hit.x - c.x, hit.z - c.z);
  }

  // 设定全体足端目标环半径并发 goto (节流: ≥120ms 直发, 否则 130ms 后补发最后一条)
  function setTarget(r, relX, relZ) {
    const C = window.SimCore.CONFIG;
    const core = window.SimCore;

    core.setManual(true, r);        // 进入遥控姿态（自动暂停自动步态）
    if (ui) ui.tgtCheck.checked = true;
    markers.group.visible = true;
    updateMarkers(r, relX, relZ);

    const a = r - C.hipRadius;                       // 足端相对髋的径向偏移 (cm)
    const drop = core.getCenter().y - C.hipDrop;     // 髋到地落差
    S.drop = drop;
    S.targetR = r;

    if (ui) {
      ui.meta.innerHTML =
        '目标半径 r = ' + r.toFixed(1) + ' cm　髋外 a = ' + a.toFixed(1) + ' cm' +
        '<br>固件 goto ' + (-a).toFixed(1) + ' ' + (-drop).toFixed(1) +
        '（6腿同步执行：大腿/小腿舵机同转到位）';
    }

    if (S.connected) {
      throttleSend('goto ' + (-a).toFixed(1) + ' ' + (-drop).toFixed(1));
    } else {
      log('! 未连接串口：当前仅预览，机器人不动（先点“连接串口”）');
    }
  }

  // ================= 快捷动作 =================
  function preset(cmd) {
    const C = window.SimCore.CONFIG;
    switch (cmd) {
      case 'stand': {
        if (!S.connected) { log('! 未连接串口'); return; }
        send('home');
        log('→ home：真机回默认站立位');
        if (ui && ui.tgtCheck.checked) {
          window.SimCore.setManual(true, C.R_LO);   // 环同步缩到站立足半径（R_LO = 髋外 6.0cm, 对应固件站姿脚 (−6,−10)）
          updateMarkers(C.R_LO, null);
        }
        break;
      }
      case 'cycle': {
        if (!S.connected) { log('! 未连接串口'); return; }
        send('cycle');
        log('→ cycle：真机 6 腿同步连续行走；发“停止”停下');
        break;
      }
      case 'stop': {
        if (!S.connected) { log('! 未连接串口'); return; }
        send('stop');
        log('→ stop：真机停止并回站立位');
        break;
      }
      case 'pos': {
        if (!S.connected) { log('! 未连接串口'); return; }
        send('pos');
        break;
      }
      default: send(cmd);
    }
  }

  // ================= 目标遥控开关 =================
  function onTargetToggle() {
    const on = ui.tgtCheck.checked;
    const core = window.SimCore;
    const C = core.CONFIG;
    if (on) {
      core.setManual(true);
      markers.group.visible = true;
      markers.group.position.set(core.getCenter().x, 0.15, core.getCenter().z);
      const r = core.manual.r;
      updateMarkers(r, null);
      ui.meta.innerHTML =
        '目标半径 r = ' + r.toFixed(1) + ' cm —— 点击/拖拽地面任意点设定';
      log('🎯 目标点遥控已开启：点击/拖拽地面 = 全体足端目标（视角拖拽已暂停）');
    } else {
      core.setManual(false);
      markers.group.visible = false;
      ui.meta.innerHTML = '目标半径 r = — cm　髋外 a = — cm　固件 goto —';
      ui.lastCmd.textContent = '';
      log('🎯 目标点遥控已关闭（模拟可按 ▶ 继续自动步态）');
    }
    core.renderer.domElement.style.cursor = on ? 'crosshair' : 'default';
    core.controls.enabled = !on;     // 遥控时禁用视角拖拽，把地面留给点选
  }

  // ================= Web Serial =================
  function onConnectClick() {
    if (S.connected) disconnect();
    else if (!S.opening) connect();
  }

  // 彻底释放串口连接并等 close 完成, 防止对没关干净的端口再次 open 报 "already open"
  async function releasePort() {
    if (S.reader) { try { await S.reader.cancel(); } catch (e) { /* ignore */ } }
    S.reader = null;
    if (S.writer) { try { S.writer.releaseLock(); } catch (e) { /* ignore */ } S.writer = null; }
    const p = S.port;
    S.port = null;
    if (p) { try { await p.close(); } catch (e) { /* ignore */ } }
    S.connected = false;
  }

  async function connect() {
    if (!navigator.serial) {
      log('! 浏览器不支持 Web Serial：请用 Chrome/Edge 通过 http://localhost:8000 打开本页');
      return;
    }
    if (S.opening) return;
    S.opening = true;
    try {
      if (S.port || S.reader || S.writer || S.connected) {   // 残留旧连接 → 先彻底关掉
        await releasePort();
        setConnUI(false);
      }
      const port = await navigator.serial.requestPort();
      try {
        await port.open({ baudRate: 115200 });
      } catch (e) {
        if (/already open|already in use|busy/i.test(e && e.message)) {
          log('! ' + e.message);
          log('  → 该串口仍被占用：请关掉其它连着它的网页标签/旧页面后刷新本页再连；');
          log('    并确认 PlatformIO / Arduino 串口监视器已关闭。');
        } else {
          log('! 连接失败：' + e.message);
          log('  → 请先关闭 PlatformIO 串口监视器或其他占用串口的程序，再重试');
        }
        try { await port.close(); } catch (e2) { /* ignore */ }
        return;
      }
      S.port = port;
      S.connected = true;
      try {
        S.writer = port.writable.getWriter();
      } catch (e) {
        log('! 获取写入通道失败：' + e.message);
        S.connected = false; S.port = null;
        try { await port.close(); } catch (e2) { /* ignore */ }
        return;
      }
      setConnUI(true);
      log('✓ 已连接串口 115200（固件须为 MODE_SIX_LEGS_SYNC）');
      send('dur ' + ui.durIn.value);       // 同步动作时长
      readLoop(port);
    } catch (e) {
      if (e && e.message && /no port|cancel/i.test(e.message)) {
        log('已取消选择串口');
      } else {
        log('! 连接失败：' + (e && e.message ? e.message : e));
      }
    } finally {
      S.opening = false;
    }
  }

  function setConnUI(on) {
    if (!ui) return;
    ui.dot.className = 'rcDot ' + (on ? 'on' : 'off');
    ui.stTxt.textContent = on ? '已连接 · 115200' : '未连接';
    ui.btnConnect.textContent = on ? '🔌 断开串口' : '🔌 连接串口';
  }

  async function disconnect() {
    if (!S.connected && !S.port && !S.reader && !S.writer) return;
    const wasConn = S.connected;
    S.connected = false;
    try { if (S.reader) S.reader.cancel(); } catch (e) { /* ignore */ }
    releasePort();
    setConnUI(false);
    if (wasConn) log('串口已断开');
  }

  async function readLoop(port) {
    const decoder = new TextDecoder();
    let buf = '';
    let reader = null;
    try {
      reader = port.readable.getReader();
      S.reader = reader;
      while (S.connected) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        let idx;
        while ((idx = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, idx).replace(/\r$/, '').trim();
          buf = buf.slice(idx + 1);
          if (line) rx(line);
        }
      }
    } catch (e) {
      if (S.connected) log('! 串口读取中断：' + e.message);
    } finally {
      try { if (reader) reader.releaseLock(); } catch (e) { /* ignore */ }
      if (S.reader === reader) S.reader = null;
      if (S.port === port) {               // 没被 disconnect 抢先接管时，由本循环负责关端口
        S.port = null;
        try { await port.close(); } catch (e) { /* ignore */ }
      }
      S.writer = null;
      if (S.connected) {
        S.connected = false;
        setConnUI(false);
        log('串口已断开');
      }
    }
  }

  function throttleSend(cmd) {
    const now = Date.now();
    if (now - S.lastSentAt >= 120) {
      S.lastSentAt = now;
      send(cmd);
      return;
    }
    S.pendingSend = cmd;
    clearTimeout(throttleSend._t);
    throttleSend._t = setTimeout(function () {
      if (S.pendingSend && S.connected) {
        S.lastSentAt = Date.now();
        send(S.pendingSend);
        S.pendingSend = null;
      }
    }, 130);
  }

  async function send(cmd) {
    if (!S.connected || !S.writer) return false;
    try {
      await S.writer.write(new TextEncoder().encode(cmd + '\n'));
      tx(cmd);
      return true;
    } catch (e) {
      log('! 发送失败：' + e.message);
      return false;
    }
  }

  // ================= 日志 =================
  const logs = [];
  function log(msg) {
    push('[' + new Date().toLocaleTimeString('zh-CN', { hour12: false }) + '] ' + msg);
  }
  function rx(line) {
    if (/^@SERVO/.test(line)) return;   // @SERVO 为孪生遥测行（遥测现已通用回传），不进触地遥控日志
    push(line);
  }
  function tx(cmd) {
    if (ui) ui.lastCmd.textContent = '▶ 最近发送: ' + cmd;
    push('→ ' + cmd);
  }
  function push(txt) {
    logs.push(txt);
    while (logs.length > LOG_MAX) logs.shift();
    if (ui) {
      ui.logBox.textContent = logs.join('\n');
      ui.logBox.scrollTop = ui.logBox.scrollHeight;
    }
  }

  // 暴露给 main.js：切到悬空模式时关闭本面板(触地·真机遥控)的串口连接
  window.RcPanel = {
    disconnect: disconnect,
    isConnected: function () { return S.connected; }
  };

  boot();
})();
