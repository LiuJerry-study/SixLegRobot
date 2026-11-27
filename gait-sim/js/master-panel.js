/* ============================================================
 * master-panel.js — 悬空模式 · 主控台（方向行走 / 选腿控制 / 单舵机控制）
 *
 * 与固件 -DMODE_MASTER_CONTROL（test/MasterControl.cpp）配套，一个面板管三件事：
 *   ① 方向行走    f / fl / bl / b / br / fr / dir / walk [n] / step / stop / mirror
 *   选腿控制    legs all|none|<n...>|<a-b> / cycle / one / each / home / goto <x> <y>
 *   单舵机直控  sv <leg> <joint> <角度> / ch <通道> <角度> / lsv / asv / center
 *   另有 dur / lift / pos / map / state / verify / phase。
 *
 * 串口由 twin-panel.js 独占：本面板**不自己开串口**，只
 *   · 发命令 → window.TwinPanel.send(cmd)
 *   · 收遥测 → twin-panel.js 转发进来的
 *        @SERVO:<ch>:<角度>   （ServoManager::write 统一回传，用来回写 12 路滑条）
 *        @STATE dir=.. sqo=.. walk=.. cont=.. steps=.. sel=.. raw=.. dur=.. lift=..
 *                             （固件每条命令后回传，用来同步按钮/选腿/参数状态）
 *
 * 显隐规则：悬空模式 + 固件模式选「主控台」或「自动识别」时显示（勾选别的固件时自动隐藏）。
 * 依赖 twin-panel.js 暴露的 window.TwinPanel（发送/连接状态）与 main.js 的 window.SimCore。
 * ============================================================ */
(function () {
  'use strict';

  const S = {
    built: false,
    fw: 'auto',                                  // 当前固件模式（twin-panel.js 的 #twinFwSel）
    sel: [false, false, false, false, false, false],
    conn: null                                   // 上一次的"已连接"状态（用于切换控件可用性）
  };

  // 6 个行走方向（0° = 机头方向，逆时针为正；快捷键就是固件命令）
  const DIRS = [
    ['f',  '前 0°',     0],
    ['fl', '左前 60°',  60],
    ['bl', '左后 120°', 120],
    ['b',  '后 180°',   180],
    ['br', '右后 240°', 240],
    ['fr', '右前 300°', 300]
  ];
  const JOINT_LABEL = ['大腿', '小腿'];
  let ui = null;

  // ================= 发送（带节流，供滑条连续拖动用） =================
  let lastSendAt = 0, pendingCmd = null, pendTimer = null;
  const THROTTLE_MS = 120;

  function send(cmd) {
    if (window.TwinPanel && typeof window.TwinPanel.send === 'function') return window.TwinPanel.send(cmd);
    return false;
  }

  // 拖动滑条时按 120ms 节流下发（立即发第一条，其余合并到尾部）
  function sendThrottled(cmd) {
    const now = performance.now();
    if (now - lastSendAt >= THROTTLE_MS) {
      lastSendAt = now;
      send(cmd);
      return;
    }
    pendingCmd = cmd;
    if (pendTimer) return;
    pendTimer = setTimeout(function () {
      pendTimer = null;
      lastSendAt = performance.now();
      const c = pendingCmd;
      pendingCmd = null;
      if (c) send(c);
    }, THROTTLE_MS - (now - lastSendAt));
  }

  function isConnected() {
    return !!(window.TwinPanel && window.TwinPanel.isConnected && window.TwinPanel.isConnected());
  }

  // ================= 自举 =================
  function boot() {
    if (!(window.SimCore && window.SimCore.robot && window.TwinPanel)) { setTimeout(boot, 150); return; }
    if (S.built) return;
    build();
  }

  function mk(tag, cls, txt) {
    const el = document.createElement(tag);
    if (cls) el.className = cls;
    if (txt !== undefined) el.textContent = txt;
    return el;
  }

  function btn(label, cmd, cls) {
    const b = mk('button', cls || null, label);
    b.addEventListener('click', function () { send(cmd); });
    return b;
  }

  // ================= UI 构建 =================
  function build() {
    S.built = true;
    const panel = document.getElementById('panel');
    if (!panel) return;

    const sec = mk('section');
    sec.id = 'masterPanel';
    sec.appendChild(mk('h2', null, '🕹 主控台 · 方向行走 / 选腿控制 / 单舵机'));
    sec.appendChild(mk('div', 'dim rcHint',
      '固件：-DMODE_MASTER_CONTROL（test/MasterControl.cpp）。串口由上方「🔌 连接串口」统一管理，' +
      '本面板只发命令 + 显示回传；写舵机都会回传 @SERVO 遥测，所以 3D 孪生会跟着动。'));

    // ---- 状态行 ----
    const stat = mk('div', 'rcMeta');
    stat.id = 'mpStat';
    stat.textContent = '状态：未连接';
    sec.appendChild(stat);

    // ---- ① 方向行走 ----
    const d1 = mk('details', 'twinDetails');
    d1.open = true;
    d1.appendChild(mk('summary', null, '行走方向'));
    const dirGrid = mk('div', 'mpGrid3');
    DIRS.forEach(function (d) {
      const b = mk('button', 'mpDirBtn', d[1]);
      b.dataset.dir = String(d[2]);
      b.addEventListener('click', function () {
        S.dir = d[2];                                  // 打方向快捷键 = 设方向 + 直接开跑
        send(d[0]);
        paintDir();
      });
      dirGrid.appendChild(b);
    });
    d1.appendChild(dirGrid);
    const walkRow = mk('div', 'rcBtnRow');
    walkRow.appendChild(btn('▶ 连续走', 'walk'));
    walkRow.appendChild(btn('走一拍', 'step'));
    walkRow.appendChild(btn('走一个周期', 'walk 2'));
    walkRow.appendChild(btn('■ 停 stop', 'stop'));
    walkRow.appendChild(btn('↔ 镜像 mirror', 'mirror'));
    d1.appendChild(walkRow);
    const dirNow = mk('div', 'dim rcHint');
    dirNow.id = 'mpDirNow';
    dirNow.textContent = '当前方向：0° 前';
    d1.appendChild(dirNow);
    d1.appendChild(mk('div', 'dim rcHint',
      '本构型没有髋部偏转关节，"往哪走" = 整张相位表绕机体旋转（sqo），所以方向只有 6 档 60°；' +
      '换方向 = 停→6 腿回站姿→按新方向重新起步。实物走反了发一次 mirror。'));
    sec.appendChild(d1);

    // ---- 选腿控制 ----
    const d2 = mk('details', 'twinDetails');
    d2.open = true;
    d2.appendChild(mk('summary', null, '选腿控制（下面这些只作用于选中的腿）'));
    const legRow = mk('div', 'rcBtnRow');
    legRow.id = 'mpLegRow';
    ui = ui || {};
    ui.legBtns = [];
    for (let i = 0; i < 6; i++) {
      (function (id) {
        const b = mk('button', 'mpLegBtn', 'leg' + id);
        b.addEventListener('click', function () {
          S.sel[id] = !S.sel[id];
          paintSel();
          send('legs ' + selCmdSpec());
        });
        legRow.appendChild(b);
        ui.legBtns.push(b);
      })(i);
    }
    d2.appendChild(legRow);

    const selRow = mk('div', 'rcBtnRow');
    selRow.appendChild(presetBtn('全选', [0, 1, 2, 3, 4, 5]));
    selRow.appendChild(presetBtn('清空', []));
    selRow.appendChild(presetBtn('单侧 1 3 5', [1, 3, 5]));
    selRow.appendChild(presetBtn('单侧 0 2 4', [0, 2, 4]));
    selRow.appendChild(presetBtn('只 leg0', [0]));
    d2.appendChild(selRow);

    const actRow = mk('div', 'rcBtnRow');
    actRow.appendChild(btn('cycle 连续走', 'cycle'));
    actRow.appendChild(btn('one 走一步', 'one'));
    actRow.appendChild(btn('each 逐腿巡检', 'each'));
    actRow.appendChild(btn('home 回站立位', 'home'));
    d2.appendChild(actRow);

    // goto：选中腿足端到绝对坐标（cm，原点 = 髋关节）
    const gRow = mk('div', 'row');
    gRow.appendChild(mk('span', null, 'goto'));
    const gx = mk('input'); gx.type = 'number'; gx.id = 'mpGx'; gx.value = '-6'; gx.step = '0.5'; gx.className = 'mpNum';
    const gy = mk('input'); gy.type = 'number'; gy.id = 'mpGy'; gy.value = '-10'; gy.step = '0.5'; gy.className = 'mpNum';
    gRow.appendChild(gx); gRow.appendChild(gy);
    const gBtn = mk('button', null, '发送');
    gBtn.addEventListener('click', function () {
      send('goto ' + (parseFloat(gx.value) || 0) + ' ' + (parseFloat(gy.value) || 0));
    });
    gRow.appendChild(gBtn);
    d2.appendChild(gRow);
    d2.appendChild(mk('div', 'dim rcHint',
      'goto 坐标原点 = 各腿自己的髋关节：默认站姿 (−6, −10)、伸出足 (−12.77, −10)；' +
      '可达环 6cm ~ 19cm，超出会被 IK 夹逼。cycle/one/each 走的都是"摆动抬腿前迈 + 支撑贴地收回"。'));
    sec.appendChild(d2);

    // ---- 单舵机直控 ----
    const d3 = mk('details', 'twinDetails');
    d3.open = true;
    d3.appendChild(mk('summary', null, '单个舵机直控（想动哪一路就动哪一路）'));

    // 单条腿的单个舵机：腿 + 关节 + 角度
    const svRow = mk('div', 'row');
    svRow.appendChild(mk('span', null, '单舵机'));
    const legSel = mk('select', 'mpSel');
    for (let i = 0; i < 6; i++) {
      const o = mk('option', null, 'leg' + i); o.value = String(i);
      legSel.appendChild(o);
    }
    const jSel = mk('select', 'mpSel');
    [['0', '大腿'], ['1', '小腿']].forEach(function (p) {
      const o = mk('option', null, p[1]); o.value = p[0];
      jSel.appendChild(o);
    });
    svRow.appendChild(legSel); svRow.appendChild(jSel);
    d3.appendChild(svRow);

    const svRow2 = mk('div', 'row');
    const svRange = mk('input'); svRange.type = 'range'; svRange.min = '0'; svRange.max = '180'; svRange.step = '0.5'; svRange.value = '90';
    svRange.id = 'mpSvRange';
    const svVal = mk('span', 'rcNum', '90.0°');
    const svSend = mk('button', null, '发送');
    function svCmd() {
      return 'sv ' + legSel.value + ' ' + jSel.value + ' ' + (parseFloat(svRange.value) || 0).toFixed(1);
    }
    svRange.addEventListener('input', function () {
      svVal.textContent = (parseFloat(svRange.value) || 0).toFixed(1) + '°';
      sendThrottled(svCmd());
    });
    svRange.addEventListener('change', function () { send(svCmd()); });
    svSend.addEventListener('click', function () { send(svCmd()); });
    svRow2.appendChild(svRange); svRow2.appendChild(svVal); svRow2.appendChild(svSend);
    d3.appendChild(svRow2);

    const cRow = mk('div', 'rcBtnRow');
    cRow.appendChild(btn('center 全回 90°', 'center'));
    cRow.appendChild(btn('map 通道表', 'map'));
    cRow.appendChild(btn('pos 读坐标', 'pos'));
    // ⚠ 这一条必须在点击时现算（腿/关节/角度都是当前选择），不能像上面那样把命令字符串写死
    const asvBtn = mk('button', null, 'asv 选中腿同关节');
    asvBtn.addEventListener('click', function () {
      send('asv ' + jSel.value + ' ' + (parseFloat(svRange.value) || 0).toFixed(1));
    });
    cRow.appendChild(asvBtn);
    d3.appendChild(cRow);
    d3.appendChild(mk('div', 'dim rcHint', '滑条发送逻辑角。每行显示已回传的逻辑角、最终角与限幅。'));

    // 12 路滑条：拖哪一路就发哪一路（@SERVO 遥测回来会回写）
    const chWrap = mk('div', 'mpChWrap');
    chWrap.id = 'mpChWrap';
    ui.chRows = {};
    for (const [ch, {leg, joint}] of MotionFrame.channels) {
      const row = mk('div', 'mpChRow');
      const lb = mk('span', 'mpChLabel' + (joint ? ' tib' : ''), window.SimCore.twin.labelOf(ch));
      const range = mk('input', 'mpChRange');
      range.type = 'range'; range.min = '0'; range.max = '180'; range.step = '0.1'; range.value = '90';
      range.setAttribute('aria-label', lb.textContent + '逻辑角');
      const val = mk('span', 'twVal', '待同步');
      let dragging = false;
      range.addEventListener('pointerdown', event => { dragging = true; range.setPointerCapture?.(event.pointerId); });
      range.addEventListener('lostpointercapture', () => { dragging = false; });
      range.addEventListener('blur', () => { dragging = false; });
      const cmd = () => 'sv ' + leg + ' ' + joint + ' ' + Number(range.value).toFixed(1);
      range.addEventListener('input', () => sendThrottled(cmd()));
      range.addEventListener('change', () => {
        dragging = false;
        if (pendTimer) clearTimeout(pendTimer);
        pendTimer = pendingCmd = null;
        send(cmd());
      });
      row.append(lb, range, val);
      chWrap.appendChild(row);
      ui.chRows[ch] = {range, val, dragging: () => dragging};
    }
    d3.appendChild(chWrap);
    sec.appendChild(d3);

    // ---- 参数 / 诊断 ----
    const d4 = mk('details', 'twinDetails');
    d4.appendChild(mk('summary', null, '参数 / 诊断'));
    const pRow = mk('div', 'row');
    pRow.appendChild(mk('span', null, 'dur'));
    const durIn = mk('input'); durIn.type = 'number'; durIn.id = 'mpDur'; durIn.value = '600'; durIn.step = '50'; durIn.className = 'mpNum';
    pRow.appendChild(durIn);
    pRow.appendChild(mk('span', null, 'ms'));
    const durBtn = mk('button', null, '发送');
    durBtn.addEventListener('click', function () { send('dur ' + (parseInt(durIn.value, 10) || 600)); });
    pRow.appendChild(durBtn);
    d4.appendChild(pRow);

    const lRow = mk('div', 'row');
    lRow.appendChild(mk('span', null, 'lift'));
    const liftIn = mk('input'); liftIn.type = 'number'; liftIn.id = 'mpLift'; liftIn.value = '3'; liftIn.step = '0.5'; liftIn.className = 'mpNum';
    lRow.appendChild(liftIn);
    lRow.appendChild(mk('span', null, 'cm'));
    const liftBtn = mk('button', null, '发送');
    liftBtn.addEventListener('click', function () { send('lift ' + Number(liftIn.value)); });
    lRow.appendChild(liftBtn);
    d4.appendChild(lRow);

    const diagRow = mk('div', 'rcBtnRow');
    diagRow.appendChild(btn('verify 方向自检', 'verify'));
    diagRow.appendChild(btn('phase 相位表', 'phase'));
    diagRow.appendChild(btn('state 状态', 'state'));
    diagRow.appendChild(btn('home 全部回位', 'home'));
    d4.appendChild(diagRow);
    const motionSettings = mk('div');
    motionSettings.hidden = true;
    const gait = mk('select');
    gait.setAttribute('aria-label', '真机步态');
    for (const [value, text] of [['legacy', '旧三角'], ['five', '五足支撑'], ['four', '四足支撑'], ['three', '三足支撑']]) {
      const option = mk('option', null, text);
      option.value = value;
      gait.append(option);
    }
    gait.addEventListener('change', () => send('gait ' + gait.value));
    const used = mk('p', 'dim');
    motionSettings.append(gait, used);
    const numeric = {};
    for (const [command, label, min, max, step] of [
      ['dir', '连续方向 °', 0, 360, 1], ['stride', '步幅 cm', 0.1, 6.8, 0.1],
      ['height', '站高 cm', 9.5, 12, 0.1], ['duty', '支撑占空比（0 用预设）', 0, 1, 0.001]
    ]) {
      const row = mk('label', 'row', label);
      const input = mk('input', 'mpNum');
      input.type = 'number'; input.min = min; input.max = max; input.step = step;
      const action = mk('button', null, '发送');
      action.addEventListener('click', () => {
        if (input.value !== '' && input.checkValidity()) send(command + ' ' + input.value);
      });
      row.append(input, action);
      motionSettings.append(row);
      numeric[command] = input;
    }
    d4.append(motionSettings);
    Object.assign(ui, {motionSettings, gait, used, numeric});

    sec.appendChild(d4);

    // 插到"地面参数区"之前（与 twin-panel 同一个锚点 ⇒ 正好排在孪生控制台下面）
    const old = document.getElementById('masterPanel');
    if (old && old !== sec) old.remove();
    const anchor = document.getElementById('groundOnly') || document.getElementById('notes');
    panel.insertBefore(sec, anchor);

    ui.legSel = legSel; ui.jSel = jSel; ui.svRange = svRange; ui.svVal = svVal;
    ui.dirBtns = Array.prototype.slice.call(dirGrid.children);
    ui.dirNow = dirNow; ui.stat = stat; ui.durIn = durIn; ui.liftIn = liftIn;

    paintSel();
    paintDir();
    applyVisibility();
    refreshEnabled();

    // 每帧（悬空模式 0.1s 一次）：刷状态行 / 同步滑条 / 切换控件可用性
    const prevFrame = window.SimCore.onTwinFrame;
    window.SimCore.onTwinFrame = function () {
      if (typeof prevFrame === 'function') prevFrame();
      onFrame();
    };
    // 模式切换按钮：触地/真机遥控时立刻隐藏本面板（悬空模式才由 onTwinFrame 兜底）
    const mh = document.getElementById('modeHangBtn'), mg = document.getElementById('modeGroundBtn');
    if (mh) mh.addEventListener('click', applyVisibility);
    if (mg) mg.addEventListener('click', applyVisibility);
    const mr = document.getElementById('modeRcBtn');
    if (mr) mr.addEventListener('click', applyVisibility);

    window.MasterPanel = {
      onFw: onFw,           // twin-panel.js 的 setFw() 调用：固件模式变了（决定显隐）
      onState: onState,     // twin-panel.js 收到 @STATE 转发进来
      onServo, onFrameData, resetTelemetry,
      send: send,
      isVisible: isVisible
    };
  }

  // 选腿预设按钮
  function presetBtn(label, ids) {
    const b = mk('button', null, label);
    b.addEventListener('click', function () {
      for (let i = 0; i < 6; i++) S.sel[i] = false;
      ids.forEach(function (i) { S.sel[i] = true; });
      paintSel();
      send('legs ' + selCmdSpec());
    });
    return b;
  }

  // ================= 显隐 / 可用性 =================
  function isVisible() {
    if (!window.SimCore || typeof window.SimCore.isHang !== 'function') return false;
    if (!window.SimCore.isHang()) return false;
    return S.fw === 'master' || S.fw === 'auto';   // 自动识别（未探测出结果）时也给出来
  }

  function applyVisibility() {
    const sec = document.getElementById('masterPanel');
    if (sec) sec.classList.toggle('hidden', !isVisible());
  }

  function onFw(k) {
    S.fw = k;
    applyVisibility();
  }

  function refreshEnabled() {
    const sec = document.getElementById('masterPanel');
    if (!sec) return;
    const on = isConnected();
    if (on === S.conn) return;
    S.conn = on;
    const els = sec.querySelectorAll('button, input, select');
    for (let i = 0; i < els.length; i++) els[i].disabled = !on;
    if (ui && ui.stat) {
      ui.stat.textContent = on ? '状态：已连接（命令可用）' : '状态：未连接 → 请先在上方点「🔌 连接串口」';
    }
  }

  // ================= 绘制 =================
  function paintSel() {
    if (!ui || !ui.legBtns) return;
    for (let i = 0; i < 6; i++) ui.legBtns[i].classList.toggle('active', S.sel[i]);
  }

  function paintDir() {
    if (!ui || !ui.dirBtns) return;
    for (let i = 0; i < ui.dirBtns.length; i++) {
      ui.dirBtns[i].classList.toggle('active', parseInt(ui.dirBtns[i].dataset.dir, 10) === S.dir);
    }
  }

  function selCmdSpec() {
    const ids = [];
    for (let i = 0; i < 6; i++) if (S.sel[i]) ids.push(i);
    if (ids.length === 0) return 'none';
    if (ids.length === 6) return 'all';
    return ids.join(' ');
  }

  // ================= 遥测 =================
  // 固件每条命令后回传：@STATE dir=0 sqo=0 walk=1 cont=1 steps=3 sel=0,1 raw=0 dur=600 lift=3.0
  function onState(raw) {
    const kv = {};
    String(raw || '').trim().split(/\s+/).forEach(function (s) {
      const i = s.indexOf('=');
      if (i > 0) kv[s.slice(0, i)] = s.slice(i + 1);
    });
    if (kv.dir !== undefined) { S.dir = Number(kv.dir) || 0; paintDir(); }
    if (kv.sel !== undefined) {
      const spec = kv.sel;
      for (let i = 0; i < 6; i++) S.sel[i] = false;
      if (spec === 'all') { for (let i = 0; i < 6; i++) S.sel[i] = true; }
      else if (spec !== 'none' && spec !== '') {
        spec.split(',').forEach(function (t) {
          const id = parseInt(t, 10);
          if (id >= 0 && id < 6) S.sel[id] = true;
        });
      }
      paintSel();
    }
    if (ui && ui.dirNow) {
      const named = DIRS.filter(function (d) { return d[2] === S.dir; })[0];
      const walking = kv.walk === '1';
      ui.dirNow.textContent = '当前方向：' + S.dir + '°' + (named ? ' ' + named[1].split(' ')[0] : '') +
        ' · sqo=' + (kv.sqo !== undefined ? kv.sqo : '?') +
        ' · ' + (kv.gait || '旧三角') + (walking ? (' · 已落足 ' + (kv.steps || 0) + '/' + (kv.issued || kv.steps || 0) + ' 拍') : ' · 停') +
        ' · 直控 ' + (kv.raw === '1' ? '开' : '关');
    }
    if (ui && kv.model !== undefined) {
      ui.motionSettings.hidden = Number(kv.model) !== RobotModel.version;
      ui.gait.value = kv.gait;
      for (const [command, input] of Object.entries(ui.numeric))
        if (kv[command] !== undefined && document.activeElement !== input) input.value = kv[command];
      ui.used.textContent = '采用参数：步幅 ' + kv.usedStride + ' cm，抬脚 ' + kv.usedLift +
        ' cm，每拍 ' + kv.usedMs + ' ms · 遥测丢帧 ' + kv.dropped + ' · 控制最大延迟 ' + kv.late + ' ms';
    }
    // dur / lift 回写（用户正在输入时不覆盖）
    if (ui) {
      if (ui.durIn && document.activeElement !== ui.durIn && kv.dur !== undefined) ui.durIn.value = kv.dur;
      if (ui.liftIn && document.activeElement !== ui.liftIn && kv.lift !== undefined) ui.liftIn.value = kv.lift;
    }
  }

  function showCommand(ch, command) {
    const row = ui?.chRows[ch];
    if (!row) return;
    row.val.textContent = command
      ? command.logical.toFixed(1) + '° → ' + command.applied.toFixed(1) + '°' + (command.limited ? ' 限幅' : '')
      : '待同步';
    if (command && !row.dragging() && document.activeElement !== row.range) row.range.value = command.logical;
  }

  function onServo(ch, applied) {
    const logical = MotionFrame.logicalFromApplied(ch, applied);
    if (logical !== null) showCommand(ch, { logical, applied });
  }

  function onFrameData(frame) {
    for (const ch of MotionFrame.channels.keys()) showCommand(ch, frame.outputs[ch]);
  }

  function resetTelemetry() {
    if (!ui) return;
    ui.motionSettings.hidden = true;
    for (const ch of MotionFrame.channels.keys()) showCommand(ch, null);
  }

  // ================= 每帧（0.1s） =================
  function onFrame() {
    applyVisibility();
    refreshEnabled();
  }

  boot();
})();
