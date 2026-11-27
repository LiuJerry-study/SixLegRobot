/* ============================================================
 * control-panel.js — 真机遥控模式(rc) · 串口遥控面板（Web Serial → ESP32）
 *
 * 顶部「🤖 真机遥控」按钮切到本面板（触地场景 + 串口遥控）: 面板插在 #groundOnly 之前,
 *   紧贴模式开关, 不用往下翻; 悬空/触地模式下面板整体隐藏(main.js setAppMode 切 style.display)。
 *
 * 🕹 圆形移动键（本面板主控件, 取代旧的「输入式/点地面设目标环」方向控制）:
 *   · 口径: 圆盘正上方 = leg0 正前方 = 0°, 逆时针 60° 一档（本构型没有髋部偏转关节,
 *     固件方向只有 sqo 的 6 档 60°, 见 test/MasterControl.cpp）
 *       上=前 0°(f) 左上=左前 60°(fl) 左下=左后 120°(bl)
 *       下=后 180°(b) 右下=右后 240°(br) 右上=右前 300°(fr)
 *   · 按住往外拖 = 发该档方向键（固件: 设方向 + 直接开走）; 拖到别的档 = 发新方向键
 *     （固件换向 = 停 → 6 腿回站姿 → 按新方向重新起步, 所以换档 400ms 节流、只补发最后一档）
 *   · 松手 / 指针取消 / 窗口失焦 / 页面切后台 = 发 stop（真机停一切并平滑回站立位）
 *   · 拖到中心死区（< 0.22R）= 没给方向 = 停
 *   · 未连接串口 = 纯预览: 圆盘照样跟手 + 日志提示, 真机不动
 *   · 拖动摇杆【只发串口】, 3D 触地场景不动; 只把方向 φ 箭头指到该档（便于确认朝哪边走）
 *   · 安全: 离开 rc 模式 / 断开串口 / 关页面 之前都会先补一条 stop（否则真机会一直走下去）
 * 另有: 站立(home) / 连续走(walk) / 停止(stop) / 读状态(pos)、每拍时长(dur)、手动命令、收发日志。
 * 固件须为 -DMODE_MASTER_CONTROL（test/MasterControl.cpp）; 串口独占(连接前关掉串口监视器),
 *   须 http://localhost + Chrome/Edge。
 * 与悬空孪生互斥: 切走时 main.js 调 window.RcPanel.disconnect()（先补 stop 再关串口）+ deactivate()。
 * @SERVO / @STATE 遥测行由 rx() 消化, 不进本日志（@STATE 用来回写圆盘状态行与 dur 滑条）。
 * 依赖 main.js 暴露的 window.SimCore。
 */
(function () {
  'use strict';

  // ================= 状态 =================
  const S = {
    port: null, reader: null, writer: null, connected: false,
    opening: false,          // 正在建立连接的标志（防重复点击 / 重复 open）
    // ---- 摇杆 / 行走 ----
    joy: { active: false, pointerId: -1, k: -1, dx: 0, dy: 0 },
    walking: false,          // 本地认为"已让真机开走" → 松手 / 断开 必须补 stop
    lastDirAt: -Infinity,    // 上一次真正发出方向命令的时间 → 首次拖动必须立刻发, 不能吃节流
    pendingDir: null,        // 节流窗口内被压住的最后一档, 窗口到点补发
    pendTimer: null,
    warnedNoConn: false,     // 一次拖动只提示一次"未连接"
    fw: { dir: null, walk: null }   // 固件 @STATE 回传（只用于显示, 不驱动逻辑）
  };
  const LOG_MAX = 220;
  const DIR_THROTTLE_MS = 400;   // 连续换档的最小间隔（固件换向要 停→回站姿→重新起步）
  let ui = null;

  // ================= 摇杆几何（纯函数, 无 DOM: node 单测按标记原文抽取）=================
  const JOY = {
    KEYS:   ['f', 'fl', 'bl', 'b', 'br', 'fr'],
    ANGLES: [0, 60, 120, 180, 240, 300],
    LABELS: ['前', '左前', '左后', '后', '右后', '右前'],
    DEAD:   0.22,    // 中心死区: 拖动距离 < DEAD·R = 没给方向（→ 停）
    KNOB:   0.80,    // 旋钮最多距圆心 KNOB·R（再远只记角度, 不再往外画）
    // 屏幕坐标(相对圆心, y 向下) → 方向档 0..5
    //   正上方 = 0° = leg0 正前；逆时针为正（屏幕左侧 = +60° = 左前）
    //   正落在两档边界(30°/90°…)时判给角度较小的那一侧（确定、可预测, 避免边界抖动）
    sectorOf: function (dx, dy) {
      const a = Math.atan2(-dx, -dy) * 180 / Math.PI;
      return ((Math.round(a / 60 - 1e-9) % 6) + 6) % 6;
    },
    // 屏幕坐标 → 0~360° 方位角（同口径, 仅供显示/调试）
    angleOf: function (dx, dy) {
      const a = Math.atan2(-dx, -dy) * 180 / Math.PI;
      return ((a % 360) + 360) % 360;
    },
    distOf: function (dx, dy) { return Math.sqrt(dx * dx + dy * dy); }
  };

  // 摇杆圆盘尺寸（css px）: 面板内容宽 ≈356, 居中放得下
  const PAD = 216;
  const PAD_R = PAD / 2;
  const PAD_RIN = PAD_R - 16;    // 圆盘有效半径（外圈 16px 留给"正前方"标记）

  // 本页角度口径(度, 正上方=0, 逆时针为正) → canvas 弧度（canvas: +x 为 0, y 向下, 角度顺时针增大）
  function cAng(deg) { return -(Math.PI / 2) - deg * Math.PI / 180; }
  function cPt(cx, cy, deg, r) {
    return [cx + r * Math.cos(cAng(deg)), cy + r * Math.sin(cAng(deg))];
  }

  let leaseTimer = null;
  let lastAngle = null;

  function boot() {
    if (window.SimCore && window.SimCore.robot) { build(); return; }
    setTimeout(boot, 150);
  }

  // ================= UI 构建 =================
  function build() {
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

    sec.appendChild(mk('h2', null, '🤖 真机遥控（串口 · 主控台固件）'));

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
    connHint.textContent = '提示：固件须为 -DMODE_MASTER_CONTROL（test/MasterControl.cpp）；先关闭 PlatformIO 串口监视器；Chrome/Edge 打开';
    sec.appendChild(connHint);

    // ---- 🕹 圆形移动键 ----
    sec.appendChild(mk('h2', null, '🕹 圆形移动键（拖 = 走 · 松 = 停）'));
    const padWrap = mk('div', 'joyWrap');
    const pad = document.createElement('canvas');
    pad.className = 'joyPad';
    pad.id = 'rcJoyPad';
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    pad.width = Math.round(PAD * dpr);
    pad.height = Math.round(PAD * dpr);
    pad.style.width = PAD + 'px';
    pad.style.height = PAD + 'px';
    const padCtx = pad.getContext('2d');
    padCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    padWrap.appendChild(pad);
    sec.appendChild(padWrap);

    const joyStat = mk('div', 'rcMeta joyStat');
    joyStat.id = 'rcJoyStat';
    sec.appendChild(joyStat);
    const joyFw = mk('div', 'dim rcHint');
    joyFw.id = 'rcJoyFw';
    sec.appendChild(joyFw);
    sec.appendChild(mk('div', 'dim rcHint',
      '正上方 = 0 号腿正前方(0°)，逆时针 60° 一档：左上=左前 60°、左下=左后 120°、下=后 180°、' +
      '右下=右后 240°、右上=右前 300°。换档时固件会「停 → 6 腿回站姿 → 按新方向重新起步」，' +
      '所以快速划圈只认最后一档（400ms 节流）；拖到中间死区或松手都会停。'));

    // ---- 快捷动作 ----
    sec.appendChild(mk('h2', null, '快捷动作'));
    const quickRow = mk('div', 'rcBtnRow');
    ['站立 home|stand', '连续走 walk|walk', '停止 stop|stop', '读状态 pos|pos', '↔ 镜像 mirror|mirror'].forEach(function (pair) {
      const p = pair.split('|');
      const b = mk('button', null, p[0]);
      b.addEventListener('click', function () { preset(p[1]); });
      quickRow.appendChild(b);
    });
    sec.appendChild(quickRow);
    sec.appendChild(mk('div', 'dim rcHint',
      '站立 = home（上电默认选腿 all，6 条腿平滑回站姿）；连续走 = walk（按固件当前方向一直走，' +
      '松手不会停，点「停止」或用圆盘换向）；停止 = stop（停步态并回站姿）；读状态 = pos；' +
      '镜像 = mirror（实物把方向走反了/腿序与假设相反时发一次，圆盘方向随之整体镜像）。'));

    // ---- 每拍时长 ----
    const durRow = mk('div', 'row');
    durRow.appendChild(mk('span', null, '每拍时长'));
    const durVal = mk('span', 'rcNum', '600 ms');
    const durIn = document.createElement('input');
    durIn.type = 'range'; durIn.min = 150; durIn.max = 1200; durIn.step = 50; durIn.value = 600;
    durIn.addEventListener('input', function () {
      durVal.textContent = durIn.value + ' ms';
      clearTimeout(durIn._t);
      durIn._t = setTimeout(function () { send('dur ' + durIn.value); }, 250);
    });
    durRow.appendChild(durIn); durRow.appendChild(durVal);
    sec.appendChild(durRow);
    sec.appendChild(mk('div', 'dim rcHint',
      'dur = 主控台固件的每拍时长（默认 600，太小会丢步/堵转）；换向/改时长后下一次起步生效。'));

    // ---- 手动命令 ----
    const manualRow = mk('div', 'row');
    const cmdIn = document.createElement('input');
    cmdIn.id = 'rcCmdIn';
    cmdIn.placeholder = '手动命令，如 dir 120 / mirror / state / pos';
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

    // ---- 日志 ----
    sec.appendChild(mk('h2', null, '串口日志'));
    const logBox = mk('pre', 'rcLog');
    logBox.id = 'rcLog';
    sec.appendChild(logBox);
    const logBtn = mk('button', null, '清空日志');
    logBtn.addEventListener('click', function () { logBox.textContent = ''; logs.length = 0; });
    sec.appendChild(logBtn);

    // 插到「步态模式/行进」区之前（紧贴模式开关下方；真机遥控模式下 groundOnly 隐藏, 本面板直接上位）
    const old = document.getElementById('rcSection');
    if (old && old !== sec) old.remove();
    const anchor = document.getElementById('groundOnly') || notes;
    panel.insertBefore(sec, anchor);
    // 初始显隐跟随当前模式（页面默认进触地模式 → 本面板先收起，之后由 main.js setAppMode 接管）
    sec.style.display = (window.SimCore.isRc && window.SimCore.isRc()) ? '' : 'none';

    ui = { dot, stTxt, btnConnect, durVal, durIn, cmdIn, logBox, pad, padCtx, joyStat, joyFw };
    log('遥控面板就绪（圆盘：拖 = 走 · 松 = 停；需 http://localhost + Chrome/Edge 才能用串口）');

    drawPad();
    setJoyStat();

    // ---- 事件 ----
    btnConnect.addEventListener('click', onConnectClick);
    pad.addEventListener('pointerdown', onPadDown);
    pad.addEventListener('pointermove', onPadMove);
    pad.addEventListener('pointerup', onPadUp);
    pad.addEventListener('pointercancel', onPadUp);
    pad.addEventListener('lostpointercapture', onPadUp);
    // 安全: 窗口失焦 / 页面切后台 = 当作松手（真机不能因为切走标签页一直走）
    window.addEventListener('blur', function () { if (S.joy.active) endDrag('窗口失焦'); });
    // 兜底: 指针捕获失败时指针弹到圆盘外松开也收不到 pointerup → 窗口再兜一层（已松手时是空操作）
    window.addEventListener('pointerup', function (e) { if (S.joy.active) onPadUp(e); });
    document.addEventListener('visibilitychange', function () {
      if (document.hidden && S.joy.active) endDrag('页面切后台');
    });
    window.addEventListener('beforeunload', function () { if (S.connected) disconnect(); });
  }

  // ================= 摇杆绘制 =================
  function drawPad() {
    if (!ui || !ui.padCtx) return;
    const ctx = ui.padCtx, cx = PAD_R, cy = PAD_R, rIn = PAD_RIN;
    const activeK = (S.joy.active && S.joy.k >= 0) ? S.joy.k : -1;
    ctx.clearRect(0, 0, PAD, PAD);

    // 底盘
    ctx.beginPath(); ctx.arc(cx, cy, rIn, 0, Math.PI * 2);
    ctx.fillStyle = '#0d1017'; ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = S.joy.active ? '#4fc3f7' : '#3a4158';
    ctx.stroke();

    // 6 个 60° 扇区（中心角 0/60/…/300°，边界在 30°/90°/…）
    for (let k = 0; k < 6; k++) {
      const a = 60 * k;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, rIn, cAng(a + 30), cAng(a - 30));
      ctx.closePath();
      ctx.fillStyle = (k === activeK)
        ? 'rgba(255, 179, 71, 0.24)'
        : (k % 2 ? 'rgba(255, 255, 255, 0.02)' : 'rgba(255, 255, 255, 0.05)');
      ctx.fill();
    }

    // 扇区分界线
    ctx.strokeStyle = '#2a2f42'; ctx.lineWidth = 1;
    for (let k = 0; k < 6; k++) {
      const p = cPt(cx, cy, 60 * k + 30, rIn);
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(p[0], p[1]); ctx.stroke();
    }

    // 档位标签（方位名 + 角度/命令键）
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    for (let k = 0; k < 6; k++) {
      const a = 60 * k, rr = rIn * 0.66;
      const p = cPt(cx, cy, a, rr);
      const on = (k === activeK);
      ctx.font = 'bold 12.5px "Microsoft YaHei", "Segoe UI", sans-serif';
      ctx.fillStyle = on ? '#ffd9a3' : '#8b93a8';
      ctx.fillText(JOY.LABELS[k], p[0], p[1] - 7);
      ctx.font = '10px Consolas, "Courier New", monospace';
      ctx.fillStyle = on ? '#ffb347' : '#5f687e';
      ctx.fillText(JOY.ANGLES[k] + '° ' + JOY.KEYS[k], p[0], p[1] + 8);
    }

    // 中心死区
    ctx.setLineDash([3, 4]); ctx.strokeStyle = '#2f3648';
    ctx.beginPath(); ctx.arc(cx, cy, JOY.DEAD * rIn, 0, Math.PI * 2); ctx.stroke();
    ctx.setLineDash([]);

    // 正前方标记（圆盘外圈正上方小三角 = leg0）
    ctx.beginPath();
    ctx.moveTo(cx, cy - rIn - 3);
    ctx.lineTo(cx - 6, cy - rIn - 13);
    ctx.lineTo(cx + 6, cy - rIn - 13);
    ctx.closePath();
    ctx.fillStyle = S.joy.active && S.joy.k === 0 ? '#ffb347' : '#4fc3f7';
    ctx.fill();

    // 指针线 + 旋钮（一直画：没拖动时旋钮停在圆心）
    let kx = cx, ky = cy;
    if (S.joy.active) {
      const d = JOY.distOf(S.joy.dx, S.joy.dy);
      const maxR = JOY.KNOB * rIn;
      const s = (d > maxR && d > 0) ? (maxR / d) : 1;
      kx = cx + S.joy.dx * s;
      ky = cy + S.joy.dy * s;
      ctx.strokeStyle = S.walking ? 'rgba(255, 179, 71, 0.85)' : 'rgba(79, 195, 247, 0.7)';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(kx, ky); ctx.stroke();
    }
    ctx.beginPath();
    ctx.arc(kx, ky, 15, 0, Math.PI * 2);
    ctx.fillStyle = S.joy.active ? (S.walking ? '#ffb347' : '#4fc3f7') : '#3a4158';
    ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = '#0d1017'; ctx.stroke();
  }

  // ================= 摇杆交互 =================
  // 指针位置 → 圆盘坐标（相对圆心, css px, y 向下）
  function padPoint(e) {
    const r = ui.pad.getBoundingClientRect();
    const R = r.width / 2;
    return { dx: e.clientX - r.left - R, dy: e.clientY - r.top - R, R: R };
  }

  function onPadDown(e) {
    if (e.pointerType === 'mouse' && e.button !== 0) return;   // 只认左键（右键/中键留给浏览器）
    e.preventDefault();
    S.joy.active = true;
    S.joy.pointerId = e.pointerId;
    S.warnedNoConn = false;
    try { ui.pad.setPointerCapture(e.pointerId); } catch (err) { /* 忽略: 拿不到捕获也能靠 move 跟随 */ }
    ui.pad.classList.add('active');
    updateJoy(padPoint(e));
  }

  function onPadMove(e) {
    if (!S.joy.active || e.pointerId !== S.joy.pointerId) return;
    e.preventDefault();
    updateJoy(padPoint(e));
  }

  function onPadUp(e) {
    if (!S.joy.active) return;
    if (e && e.pointerId !== undefined && S.joy.pointerId !== undefined && e.pointerId !== S.joy.pointerId) return;
    endDrag('松手');
  }

  // 拖动推进: 死区 = 停; 换档 = 发方向键（同档不重复发）
  function updateJoy(p) {
    const R = p.R || PAD_R;
    S.joy.dx = p.dx; S.joy.dy = p.dy;
    const dist = JOY.distOf(p.dx, p.dy);

    if (dist < JOY.DEAD * R) {                    // 中心死区 → 没给方向
      if (S.joy.k !== -1) {
        S.joy.k = -1;
        stopRobot('拖回中心死区', false);
      }
      drawPad(); setJoyStat();
      return;
    }

    const k = JOY.sectorOf(p.dx, p.dy);
    const continuous = S.fw.lease && S.fw.gait !== 'legacy';
    const angle = continuous ? Math.round(JOY.angleOf(p.dx, p.dy) / 5) * 5 % 360 : null;
    if (k !== S.joy.k || angle !== lastAngle) {
      lastAngle = angle;                          // 换档（含首次进入）
      S.joy.k = k;
      commandDir(k, angle);
    }
    drawPad(); setJoyStat();
  }

  // 结束拖动: 圆盘复位 + 停车
  function endDrag(reason) {
    S.joy.active = false;
    S.joy.pointerId = -1;
    S.joy.k = -1;
    S.joy.dx = 0; S.joy.dy = 0;
    if (ui && ui.pad) ui.pad.classList.remove('active');
    stopRobot(reason, false);
    drawPad(); setJoyStat();
  }

  // 发某个方向档: 固件里 f/fl/bl/b/br/fr = 设方向 + 直接开走
  function commandDir(k, angle) {
    if (angle === null) setPhiArrow(k);
    else window.SimCore.setPhi(angle);                               // 3D 场景不动, 只把方向 φ 箭头指到该档
    if (!S.connected) {
      if (!S.warnedNoConn) {
        S.warnedNoConn = true;
        log('! 未连接串口：圆盘只做预览，真机不动（先点「🔌 连接串口」）');
      }
      return;
    }
    S.walking = true;                             // 发出去了 → 松手/断开一定补 stop
    const cmd = S.fw.lease ? 'drive ' + (angle ?? JOY.ANGLES[k]) : JOY.KEYS[k];
    sendDirThrottled(cmd);
  }

  // 换档节流: 固件换向 = 停→回站姿→重新起步, 划圈时只认最后一档
  function sendDirThrottled(cmd) {
    const now = performance.now();
    if (S.pendTimer) { clearTimeout(S.pendTimer); S.pendTimer = null; }
    if (now - S.lastDirAt >= DIR_THROTTLE_MS) {
      S.lastDirAt = now;
      S.pendingDir = null;
      send(cmd);
      return;
    }
    S.pendingDir = cmd;
    S.pendTimer = setTimeout(function () {
      S.pendTimer = null;
      const kk = S.pendingDir;
      S.pendingDir = null;
      if (kk === null || !S.walking) return;       // 期间松手了 → 不再补发
      S.lastDirAt = performance.now();
      send(kk);
    }, DIR_THROTTLE_MS - (now - S.lastDirAt));
  }

  // 停车: 发 stop（固件 = 停一切 + 6 腿平滑回站立位）。force = 本地认为没在走也照发
  function stopRobot(reason, force) {
    clearPending();
    const need = S.walking || force;
    S.walking = false;
    lastAngle = null;
    if (!need) { setJoyStat(); return; }
    if (!S.connected) {
      if (force) log('! 未连接串口：没法发 stop');
      setJoyStat();
      return;
    }
    send('stop');
    log('■ 停止（' + reason + '）→ stop：真机停一切并回站立位');
    setJoyStat();
  }

  function clearPending() {
    if (S.pendTimer) { clearTimeout(S.pendTimer); S.pendTimer = null; }
    S.pendingDir = null;
  }

  // 3D 侧只改方向箭头（main.js 的 SimCore.setPhi）: 场景/机身不动, 便于确认朝哪边走
  function setPhiArrow(k) {
    const core = window.SimCore;
    if (core && typeof core.setPhi === 'function') core.setPhi(JOY.ANGLES[k]);
  }

  function setJoyStat() {
    if (!ui) return;
    const k = (S.joy.active && S.joy.k >= 0) ? S.joy.k : -1;
    let txt;
    if (S.walking) {
      txt = '状态：真机行走中' + (k >= 0
        ? ' → 方向 ' + JOY.ANGLES[k] + '° ' + JOY.LABELS[k] + '（' + JOY.KEYS[k] + '）'
        : '（方向 = 固件当前档）');
    } else if (S.joy.active) {
      txt = '状态：拖动中 —— 拖出中心死区才会走';
    } else {
      txt = '状态：待机 —— 按住圆盘往外拖 = 走，松手 = 停';
    }
    ui.joyStat.textContent = txt;
    const fw = S.fw;
    ui.joyFw.textContent = '固件 @STATE：' + ((fw.dir === null && fw.walk === null)
      ? '—（连上主控台固件后回传）'
      : 'dir=' + (fw.dir === null ? '?' : fw.dir) + '° walk=' + (fw.walk === null ? '?' : (fw.walk ? 1 : 0)));
  }

  // ================= 快捷动作 =================
  function preset(cmd) {
    switch (cmd) {
      case 'stand': {
        if (!requireConn('站立')) return;
        S.walking = false; clearPending();
        send('home');
        log('→ home：回默认站立位（上电默认选腿 all = 6 条腿）');
        break;
      }
      case 'walk': {
        if (!requireConn('连续走')) return;
        S.walking = true;
        send(S.fw.lease ? 'drive' : 'walk');
        log('→ walk：按固件当前方向连续走（用圆盘换向 / 点「停止 stop」才停）');
        break;
      }
      case 'stop':
        stopRobot('快捷动作', true);
        return;
      case 'pos': {
        if (!requireConn('读状态')) return;
        send('pos');
        break;
      }
      default: send(cmd);
    }
    setJoyStat();
  }

  function requireConn(what) {
    if (S.connected) return true;
    log('! 未连接串口：' + what + '只做预览，真机不动（先点「🔌 连接串口」）');
    return false;
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
      log('✓ 已连接串口 115200（固件须为 -DMODE_MASTER_CONTROL；圆盘拖 = 走 · 松 = 停）');
      S.fw.lease = false;
      S.fw.gait = null;
      readLoop(port);
      send('state');
      if (leaseTimer !== null) clearInterval(leaseTimer);
      leaseTimer = setInterval(() => {
        if (S.connected && S.walking && S.fw.lease && !document.hidden) send('keepalive');
      }, 250);
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
    if (leaseTimer !== null) clearInterval(leaseTimer);
    leaseTimer = null;
    if (!S.connected && !S.port && !S.reader && !S.writer) return;
    const wasConn = S.connected;
    // 安全: 真机还在走的时候关串口 = 它会一直走下去 → 趁串口还活着先补一条 stop
    if (wasConn && S.walking) {
      await send('stop');                                   // send() 内部同步发起写入, 关页面前也发得出去
      log('■ 断开串口前已补发 stop（真机停在原地并回站立位）');
      await new Promise(function (r) { setTimeout(r, 80); });   // 给串口一点把字节推出去的时间
    }
    S.walking = false;
    clearPending();
    if (S.joy.active) endDrag('断开串口');      // 圆盘复位（walking 已清 → 不会再补发第二条 stop）
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

  async function send(cmd) {
    if (!S.connected || !S.writer) return false;
    try {
      await S.writer.write(new TextEncoder().encode(cmd + '\n'));
      if (cmd !== 'keepalive') tx(cmd);
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
    if (/^@(SERVO|FRAME)/.test(line)) return;        // 12 路舵机遥测（孪生用），本面板不展示
    if (/^@STATE/.test(line)) { onState(line.slice(6)); return; }   // 主控台固件状态行 → 回写圆盘/时长
    push(line);
  }
  // @STATE dir=60 sqo=1 walk=1 cont=1 steps=3 sel=all raw=0 dur=600 lift=3.0
  //   只用于显示（真机实际档位/是否在走）；行走状态以本地 S.walking 为准,
  //   否则「先发的旧状态行后到」会误清行走标志, 松手就漏发 stop。
  function onState(raw) {
    const kv = {};
    String(raw || '').trim().split(/\s+/).forEach(function (s) {
      const i = s.indexOf('=');
      if (i > 0) kv[s.slice(0, i)] = s.slice(i + 1);
    });
    if (kv.lease !== undefined) S.fw.lease = Number(kv.lease) > 0;
    if (kv.gait !== undefined) S.fw.gait = kv.gait;
    if (kv.stage === 'hold') S.walking = false;
    if (kv.dir !== undefined) S.fw.dir = parseInt(kv.dir, 10);
    if (kv.walk !== undefined) S.fw.walk = (kv.walk === '1');
    if (ui && kv.dur !== undefined && document.activeElement !== ui.durIn) {
      const d = parseInt(kv.dur, 10);
      if (isFinite(d)) { ui.durIn.value = String(d); ui.durVal.textContent = d + ' ms'; }
    }
    setJoyStat();
  }
  function tx(cmd) {
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

  // 暴露给 main.js：切走真机遥控模式时关闭串口（内部会先补 stop）+ 复位圆盘
  window.RcPanel = {
    disconnect: disconnect,
    isConnected: function () { return S.connected; },
    isWalking: function () { return S.walking; },
    // 离开 rc 模式时调用：圆盘复位 + 取消待发的换档（串口由 disconnect 负责）
    deactivate: function () {
      clearPending();
      if (S.joy.active) endDrag('离开遥控模式');
      else { S.joy.k = -1; drawPad(); setJoyStat(); }
    }
  };

  boot();
})();
