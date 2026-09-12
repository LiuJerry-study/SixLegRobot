/* ============================================================
 * twin-panel.js — 悬空模式 · 数字孪生控制台（串口控制 + 镜像）
 *
 * 遥测通用: @SERVO:<ch>:<角度>（ch = leg*2 + joint = Config.h SERVO_PINS[leg][joint],
 *   0=大腿 1=小腿; 角度 = 逻辑角 + ANGLE_OFFSET 再限幅 0~180 的最终舵机角）由固件
 *   ServoManager::write() 节流回传(变化 ≥0.5° 且每通道 ≥50ms), 任意经它写舵机的模式都能镜像。
 *   ⚠ 逻辑角口径（2026-09-10「外倾为正」方案 B + 大腿映射按实物实测改版）: 大腿 = ag2、小腿 = 180 − ag1
 *     （ag1/ag2 为固件 ik() 返回的弧度换算成的度; 站姿 ag1 = ag2 = +67.38° → 大腿 67.38° / 小腿 112.62°）;
 *     Config.h::ANGLE_OFFSET 整表已清零 → 最终写入角 = 逻辑角, 站姿直接就是 67.4° / 112.6°, 不会顶 180° 端点。
 *   ⚠ MODE_SERVO_TEST 直写 PCA9685、不走 ServoManager → 选它时 12 路镜像不动。
 *
 * 控制台顶部可选/自动识别固件模式(连接后发 help), 快捷键、命令提示、参考表随之联动。
 * 默认双 90° 只是未收到遥测时的显示占位(不代表站姿); 固件 center 也发 90°(安全位, 偏移已清零)。
 * 3D 映射见 robot.js::applyServoAngles(): α = 90°−S = 90°−ag2、knee = −K = ag1 − 180, 与固件口径自洽
 * （回传角即最终写入角, 是"发什么显示什么"的 1:1 镜像, 不反向补偿）。
 * 串口独占: 连接前关掉串口监视器; 须 http://localhost + Chrome/Edge。
 * 与触地遥控互斥: 切到触地时 main.js 调 window.TwinPanel.disconnect()。
 * 依赖 main.js 暴露的 window.SimCore（轮询等待就绪后自举）。
 */
(function () {
  'use strict';

  const S = {
    port: null, reader: null, writer: null, connected: false,
    rxCount: 0,      // 收到 @SERVO 遥测帧数
    txCount: 0,      // 发往真机的命令数
    fw: 'auto',      // 当前固件模式: auto|single|leg|sync|servoTest
    probed: false,   // auto 模式下是否已发过 help 探测
    opening: false   // 正在建立连接的标志（防重复点击）
  };
  const LOG_MAX = 180;
  let ui = null;
  let built = false;

  // ================= 固件模式档案 =================
  // 与固件各 MODE_* 的命令集一一对应（见 src/main.cpp 与 test/*.cpp）
  const FW = {
    auto: {
      label: '自动识别（连接后自动探测）',
      quick: [],
      placeholder: '选择固件模式后这里会给出示例命令',
      ref: [
        ['auto', '连接后自动发一条 help 探测固件；识别失败就手动选下方模式'],
        ['说明', '命令集随固件模式而不同: 先选对模式，按钮和提示才对得上']
      ]
    },
    single: {   // MODE_SINGLE_SERVO_TEST  test/SingleServoTest.cpp
      // ⚠ ref 里"回 90°"指命令逻辑角; ANGLE_OFFSET 整表已清零 → 实际到达角就是 90°(安全位, 非站姿), 发 map 可查
      label: '单舵机测试 · MODE_SINGLE_SERVO_TEST',
      keys: ['平台单舵机测试', '单舵机测试命令'],
      quick: [
        ['center 全回90°', 'center'],
        ['map 通道表', 'map'],
        ['停止扫动', 'stop'],
        ['示例 sweep 3', 'sweep 3']
      ],
      placeholder: '如 leg 0 0 45 / sweep 3 / center / map / <ch> <角度>',
      ref: [
        ['map', '通道↔(腿,关节)对照表（含零位偏移）'],
        ['center', '全部 12 路舵机回 90°（安全位）'],
        ['sweep <ch>', '通道 ch 在 30°~150° 来回扫动（ch=0~11）'],
        ['sweep leg <l> <j>', '(腿 l, 关节 j) 来回扫动'],
        ['stop', '停止扫动，保持当前角度'],
        ['<ch> <角度>', '单通道直转（如 3 90）'],
        ['leg <l> <j> <角度>', '(腿 l, 关节 j) 直转（如 leg 0 0 45）']
      ]
    },
    leg: {      // MODE_SINGLE_LEG_TEST    test/SingleLegTest.cpp
      // ⚠ 与固件有两处对不上(不改下面字符串): 本模式 dur 实际默认 600(表里的 300 是六腿同步默认值);
      //   固件另有 legs(选被测腿) 与 each(逐腿巡检) 两条命令, 本表未列。
      label: '单腿测试 · MODE_SINGLE_LEG_TEST',
      keys: ['单腿测试', '单腿测试命令'],
      quick: [
        ['home 回站立', 'home'],
        ['one 走一步', 'one'],
        ['cycle 连续走', 'cycle'],
        ['pos 读坐标', 'pos'],
        ['stop', 'stop']
      ],
      placeholder: '如 cycle / one / home / goto 0 -8 / lift 3 / dur 300',
      ref: [
        ['home', '平滑回默认站立位'],
        ['one', '只走一个周期（摆动相抬腿前迈 + 支撑相贴地收回）'],
        ['cycle', '连续行走循环（无限）'],
        ['goto <x> <y>', '足端移到绝对坐标 (cm)'],
        ['lift <cm>', '设置弧线抬腿高度（默认 3.0）'],
        ['dur <ms>', '动作时长（默认 300，越小越快）'],
        ['pos', '打印足端坐标/状态'],
        ['stop', '停止循环，回站立位']
      ]
    },
    sync: {     // MODE_SIX_LEGS_SYNC     test/SixLegsSyncTest.cpp
      // 本表命令与默认值(lift 3.0 / dur 300)与 test/SixLegsSyncTest.cpp 一致
      label: '六腿同步 · MODE_SIX_LEGS_SYNC',
      keys: ['六腿同步测试', '六腿同步测试命令'],
      quick: [
        ['home 回站立', 'home'],
        ['one 走一步', 'one'],
        ['cycle 连续走', 'cycle'],
        ['pos 读坐标', 'pos'],
        ['stop', 'stop']
      ],
      placeholder: '如 0 67.4（6条大腿站姿角）/ 1 112.6（6条小腿站姿角）/ goto 0 -8 / cycle',
      ref: [
        ['home', '6 腿平滑回默认站立位'],
        ['one / cycle', '只走一个周期 / 连续行走循环'],
        ['goto <x> <y>', '6 腿足端同步移到绝对坐标 (cm)'],
        ['lift <cm>', '设置弧线抬腿高度（默认 3.0）'],
        ['dur <ms>', '动作时长（默认 300，越小越快）'],
        ['0 <角度>', '6 条大腿(FEMUR)舵机同步转到角度 (0~180)'],
        ['1 <角度>', '6 条小腿(TIBIA)舵机同步转到角度 (0~180)'],
        ['注', '0/1 直写的是最终舵机角(偏移已清零); 默认站姿 = 大腿 67.4° / 小腿 112.6°(发 home 让固件自己算)'],
        ['pos', '打印 6 腿足端坐标/状态'],
        ['stop', '停止循环，回站立位']
      ]
    },
    servoTest: { // MODE_SERVO_TEST       src/main.cpp 内联
      // ⚠ 本模式在 src/main.cpp 内联实现, 直写 PCA9685 不走 ServoManager → 不吃零位偏移、也不回传 @SERVO
      label: '通用舵机测试 · MODE_SERVO_TEST',
      keys: ['Mode=SERVO_TEST'],
      quick: [
        ['sweep 逐个扫', 'sweep'],
        ['all 同步扫', 'all'],
        ['mode 3 串口直控', 'mode 3']
      ],
      placeholder: '如 sweep / all / mode 3 / <ch> <角度>',
      ref: [
        ['sweep', '逐通道 30°↔150° 扫描（上电默认）'],
        ['all', '12 路同步来回扫描'],
        ['mode 3', '切换到串口直控（手动 <通道> <角度>）'],
        ['<ch> <角度>', '串口直控模式下单通道直转（如 3 90）'],
        ['注', '此模式 sweep 一直在动，想停就发 mode 3 转手动']
      ]
    }
  };

  function boot() {
    if (window.SimCore && window.SimCore.robot) { if (!built) build(); return; }
    setTimeout(boot, 150);
  }

  // ================= UI 构建 =================
  function build() {
    built = true;
    const panel = document.getElementById('panel');
    const notes = document.getElementById('notes');

    const sec = document.createElement('section');
    sec.id = 'twinPanel';
    const mk = (tag, cls, txt) => {
      const el = document.createElement(tag);
      if (cls) el.className = cls;
      if (txt !== undefined) el.textContent = txt;
      return el;
    };

    sec.appendChild(mk('h2', null, '🪝 悬空孪生控制台（网页控制 + 镜像）'));

    // ---- 连接行：状态点 + 文字 + 连接按钮 ----
    const connRow = mk('div', 'row');
    const dot = mk('span', 'rcDot off');
    dot.id = 'twinDot';
    const stTxt = mk('span', null, '未连接');
    stTxt.id = 'twinConnTxt';
    const btnConnect = mk('button', 'rcPrimary', '🔌 连接串口');
    btnConnect.id = 'twinConnect';
    connRow.appendChild(dot); connRow.appendChild(stTxt);
    const pad = mk('span', null, ''); pad.style.flex = '1'; connRow.appendChild(pad);
    connRow.appendChild(btnConnect);
    sec.appendChild(connRow);

    // ---- 固件模式行 ----
    const fwRow = mk('div', 'row');
    fwRow.appendChild(mk('span', null, '固件模式'));
    const fwSel = document.createElement('select');
    fwSel.id = 'twinFwSel';
    ['auto', 'single', 'leg', 'sync', 'servoTest'].forEach(function (k) {
      const o = document.createElement('option');
      o.value = k;
      o.textContent = FW[k].label;
      fwSel.appendChild(o);
    });
    fwRow.appendChild(fwSel);
    sec.appendChild(fwRow);

    // ---- 命令参考（随固件模式变化，折叠） ----
    const refDet = document.createElement('details');
    refDet.className = 'twinDetails cmdRef';
    const refSum = mk('summary', null, '命令集与说明（随固件模式）');
    refDet.appendChild(refSum);
    const refBody = mk('div', 'cmdRefBody');
    const refNote = mk('div', 'dim rcHint');
    refNote.innerHTML =
      '串口是<b>独占</b>的：连接前请关闭 PlatformIO / Arduino 串口监视器；' +
      '悬空孪生对固件模式<b>无限制</b>——只要写舵机走 <code>ServoManager::write()</code> 就会回传 ' +
      '<code>@SERVO</code> 遥测镜像（命令集才随固件模式而不同）。';
    refBody.appendChild(refNote);
    const refList = mk('ul', 'cmdRefList');
    refBody.appendChild(refList);
    refDet.appendChild(refBody);
    sec.appendChild(refDet);

    // ---- 命令输入 ----
    const cmdRow = mk('div', 'row');
    const cmdIn = document.createElement('input');
    cmdIn.type = 'text';
    cmdIn.id = 'twinCmdIn';
    cmdIn.placeholder = '选择固件模式后自动给出示例';
    const btnSend = mk('button', null, '发送');
    btnSend.id = 'twinSend';
    cmdRow.appendChild(cmdIn); cmdRow.appendChild(btnSend);
    sec.appendChild(cmdRow);

    // ---- 快捷动作（随固件模式变化） ----
    const quickRow = mk('div', 'rcBtnRow');
    quickRow.id = 'twinQuick';
    sec.appendChild(quickRow);

    // ---- 状态行 ----
    const stat = mk('div', 'rcMeta');
    stat.id = 'twinStat';
    sec.appendChild(stat);

    // ---- 12 通道实时表（可折叠） ----
    const chDet = document.createElement('details');
    chDet.className = 'twinDetails';
    chDet.open = true;
    chDet.appendChild(mk('summary', null, '12 通道实时角度（ch = 腿×2 + 关节，0=大腿 1=小腿）'));
    const chWrap = mk('div', 'twChWrap');
    chWrap.id = 'twinChWrap';
    chDet.appendChild(chWrap);
    sec.appendChild(chDet);

    // ---- 演示（不连真机） ----
    const actRow = mk('div', 'row');
    const demoChk = document.createElement('input');
    demoChk.type = 'checkbox';
    demoChk.id = 'twinDemoChk';
    const demoLbl = mk('label', 'rcInline');
    demoLbl.appendChild(demoChk);
    demoLbl.appendChild(mk('span', null, '离线扫动演示（不连真机，预览每路镜像方向）'));
    actRow.appendChild(demoLbl);
    sec.appendChild(actRow);

    // ---- 日志 ----
    const logRow = mk('div', 'rcBtnRow');
    const btnClearLog = mk('button', null, '清空日志');
    btnClearLog.id = 'twinClearLog';
    logRow.appendChild(btnClearLog);
    sec.appendChild(logRow);
    const logBox = mk('pre', 'rcLog');
    logBox.id = 'twinLog';
    sec.appendChild(logBox);

    // 插到"地面参数区"之前（紧贴模式开关下方；悬空时 groundOnly 隐藏、本面板自然上位）
    const old = document.getElementById('twinPanel');
    if (old && old !== sec) old.remove();
    const anchor = document.getElementById('groundOnly') || notes;
    panel.insertBefore(sec, anchor);

    ui = { dot, stTxt, btnConnect, fwSel, refList, cmdIn, btnSend, quickRow, stat, chWrap, demoChk, logBox };

    // 初始按当前应用模式显隐
    applyModeVisibility();
    setFw(S.fw);                  // 渲染当前固件模式的快捷键/参考/提示

    // 每行：标签 + 进度条 + 数值
    function buildRows() {
      ui.chWrap.innerHTML = '';
      ui.rows = [];
      const T = window.SimCore.twin;
      for (let c = 0; c < 12; c++) {
        const row = document.createElement('div');
        row.className = 'twRow';
        const lb = document.createElement('span');
        lb.className = 'twLabel' + (c % 2 === 1 ? ' tib' : '');
        lb.textContent = T.labelOf(c);
        const bar = document.createElement('div');
        bar.className = 'twBar';
        const fill = document.createElement('div');
        fill.className = 'twFill';
        bar.appendChild(fill);
        const val = document.createElement('span');
        val.className = 'twVal';
        row.appendChild(lb); row.appendChild(bar); row.appendChild(val);
        ui.chWrap.appendChild(row);
        ui.rows.push({ lb, fill, val });
      }
    }
    buildRows();

    // ---- 事件 ----
    btnConnect.addEventListener('click', onConnectClick);
    fwSel.addEventListener('change', function () {
      S.fw = fwSel.value;
      setFw(S.fw);
      log('固件模式：' + FW[S.fw].label);
      // auto 模式下重连/当前已连接 → 再探测一次
      if (S.fw === 'auto') S.probed = false;
      if (S.fw === 'auto' && S.connected) probeFirmware();
    });
    btnSend.addEventListener('click', function () {
      const v = cmdIn.value.trim();
      if (v) send(v);
      cmdIn.value = '';
    });
    cmdIn.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); btnSend.click(); }
    });
    demoChk.addEventListener('change', function () {
      window.SimCore.twin.setDemo(demoChk.checked);
      log(demoChk.checked ? '▶ 离线扫动演示开：ch0~ch11 逐个 30°↔150° 扫动（仅本地预览，真机不动）'
                          : '■ 离线扫动演示关');
    });
    btnClearLog.addEventListener('click', function () { ui.logBox.textContent = ''; logs.length = 0; });

    log('孪生控制台就绪：先选/识别固件模式 → 连接固件 → 发命令控制真机并同步镜像（连接前先关串口监视器）');

    // 每帧(节流 0.1s)刷新表格 —— 由 main.js 在悬空模式逐帧调用
    window.SimCore.onTwinFrame = function () { refreshTable(); refreshStat(); };

    window.addEventListener('beforeunload', function () { if (S.connected) disconnect(); });

    // 暴露给 main.js 模式切换用
    window.TwinPanel = {
      disconnect: disconnect,
      isConnected: function () { return S.connected; },
      send: send
    };
    applyModeVisibility();
  }

  // ================= 固件模式渲染 =================
  // 快捷键按钮、命令框占位提示、命令参考表都跟随固件模式
  function setFw(k) {
    if (!ui) return;
    const f = FW[k] || FW.auto;
    ui.fwSel.value = k;

    // 命令参考表
    ui.refList.innerHTML = '';
    (f.ref || []).forEach(function (pair) {
      const li = document.createElement('li');
      const code = document.createElement('code');
      code.textContent = pair[0];
      const desc = document.createElement('span');
      desc.textContent = pair[1];
      li.appendChild(code); li.appendChild(desc);
      ui.refList.appendChild(li);
    });

    // 快捷按钮
    ui.quickRow.innerHTML = '';
    (f.quick || []).forEach(function (pair) {
      const b = document.createElement('button');
      b.textContent = pair[0];
      b.addEventListener('click', function () { send(pair[1]); });
      ui.quickRow.appendChild(b);
    });
    if (!f.quick || !f.quick.length) {
      const hint = document.createElement('span');
      hint.className = 'dim';
      hint.textContent = '(自动识别模式没有固定快捷键)';
      ui.quickRow.appendChild(hint);
    }

    // 命令输入占位
    ui.cmdIn.placeholder = f.placeholder || '手动输入命令后回车';
  }

  // 悬空时显示本面板, 触地时隐藏
  function applyModeVisibility() {
    if (!window.SimCore) return;
    const tp = document.getElementById('twinPanel');
    if (!tp) return;
    tp.classList.toggle('hidden', !window.SimCore.isHang());
  }

  // ================= 表格 / 状态刷新 =================
  function refreshTable() {
    if (!ui || !ui.rows) return;
    const T = window.SimCore.twin;
    for (let c = 0; c < 12; c++) {
      const target = T.targetOf(c);       // 最近目标（真机遥测）
      const shown = T.currentOf(c);       // 平滑后的显示值
      const row = ui.rows[c];
      const deg = (target !== null && shown !== null) ? shown : null;
      const pct = deg === null ? 0 : Math.max(0, Math.min(100, deg / 180 * 100));
      row.fill.style.width = pct.toFixed(1) + '%';
      row.val.textContent = deg === null ? '—' : deg.toFixed(1) + '°';
    }
  }

  function refreshStat() {
    if (!ui) return;
    const T = window.SimCore.twin;
    const ls = T.lastSeen();
    const age = ls ? ((performance.now() - ls) / 1000).toFixed(1) : '—';
    const live = ls && (performance.now() - ls) < 1500;
    ui.dot.className = 'rcDot ' + (S.connected ? (live ? 'on' : 'off') : 'off');
    ui.stat.textContent =
      '固件 ' + (S.fw === 'auto' ? '自动识别中' : FW[S.fw].label.replace(/ · MODE_.*/, '')) +
      ' · 发 ' + S.txCount + ' · 遥测 ' + S.rxCount + ' · 已知通道 ' + T.knownCount() + '/12' +
      ' · 最后更新 ' + age + 's 前' +
      (S.connected ? (live ? '（数据流正常）' : '（已连接，暂无新遥测…）') : '（未连接）');
  }

  // ================= Web Serial（控制 + 镜像） =================
  function onConnectClick() {
    if (S.connected) disconnect();
    else if (!S.opening) connect();
  }

  // 彻底释放串口(reader/writer/port 全释放并等 close 完成): port 没 close 时再次 open 会报 "already open"
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
      log('! 浏览器不支持 Web Serial：请用 Chrome/Edge 通过 http://localhost 打开本页');
      return;
    }
    if (S.opening) return;
    S.opening = true;
    try {
      // 若还残留着旧连接（上次中断/断开没关干净），先彻底关掉再开，避免 "already open"
      if (S.port || S.reader || S.writer || S.connected) {
        await releasePort();
        log('↻ 已自动清理上次残留的串口连接');
        if (ui) setConnUI(false);
      }

      const port = await navigator.serial.requestPort();
      try {
        await port.open({ baudRate: 115200 });
      } catch (e) {
        if (/already open|already in use|busy/i.test(e && e.message)) {
          log('! ' + e.message);
          log('  → 这个串口此刻仍被占用，占用它的可能是：');
          log('    1) 另一个网页标签/旧页面还连着 —— 把它们全部关掉，只留本页刷新后再连；');
          log('    2) 本页上次连接没断开干净 —— 点“断开”再连，或直接刷新页面；');
          log('    3) PlatformIO / Arduino 串口监视器 —— 确认已关闭后再试。');
        } else {
          log('! 打开串口失败：' + e.message);
          log('  → 请确认已关闭 PlatformIO/Arduino 等占用串口的程序后重试。');
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
        S.connected = false;
        try { await port.close(); } catch (e2) { /* ignore */ }
        S.port = null;
        return;
      }
      setConnUI(true);
      if (ui && ui.demoChk) { ui.demoChk.checked = false; ui.demoChk.disabled = true; }
      window.SimCore.twin.setDemo(false);
      log('✓ 已连接：网页独占串口');
      readLoop(port);
      // auto 模式 → 发 help 探测固件，让命令集与真机对得上
      if (S.fw === 'auto' && !S.probed) probeFirmware();
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
    ui.stTxt.textContent = on ? '已连接 · 独占串口中' : '未连接';
    ui.btnConnect.textContent = on ? '🔌 断开串口' : '🔌 连接串口';
    if (!on && ui.demoChk) ui.demoChk.disabled = false;
  }

  async function disconnect() {
    if (!S.connected && !S.port && !S.reader && !S.writer) return;
    const wasConn = S.connected;
    S.connected = false;                       // 先置假：读循环 finally 不再重复打日志
    try { if (S.reader) S.reader.cancel(); } catch (e) { /* ignore */ }
    releasePort();                             // 异步收尾（readLoop finally 也会兜底关 port）
    setConnUI(false);
    if (wasConn) log('串口已断开');
    if (ui) refreshStat();
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
      if (S.port === port) {                   // 若没有被 disconnect 抢先接管，就由本循环负责关掉端口
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
    cmd = String(cmd || '').trim();
    if (!cmd) return false;
    if (!S.connected || !S.writer) {
      log('! 未连接串口：请先点“🔌 连接串口”');
      return false;
    }
    try {
      await S.writer.write(new TextEncoder().encode(cmd + '\n'));
      S.txCount++;
      tx(cmd);
      return true;
    } catch (e) {
      log('! 发送失败：' + e.message);
      return false;
    }
  }

  // ================= 行解析 =================
  function rx(line) {
    // 孪生遥测: @SERVO:<ch>:<角度>（分隔符 ':' 或空格都认, ch 只取 0~11, 角度=叠加零位偏移后的最终角）
    // 越界(ch>11)/非数值的 @SERVO 行不镜像, 会落到函数末尾进日志
    const m = /^@SERVO[:\s]+(\d{1,2})[:\s]+(\d+(?:\.\d+)?)/.exec(line);
    if (m) {
      const ch = parseInt(m[1], 10);
      const deg = parseFloat(m[2]);
      if (ch >= 0 && ch <= 11 && isFinite(deg)) {
        S.rxCount++;
        window.SimCore.twin.setChannel(ch, deg);
        return;                       // 遥测不回显到日志（防刷屏）
      }
    }
    // auto 模式下按回显文本识别固件模式
    if (S.connected && S.fw === 'auto') detectFirmware(line);
    push(line);                       // 其余行（help/ack/状态等）进日志
  }

  // auto 模式：发一条 help，按"命令帮助标题"识别固件
  function probeFirmware() {
    if (S.probed) return;
    S.probed = true;
    log('↻ 自动识别固件：发送 help 探测…');
    send('help');
  }

  function detectFirmware(line) {
    if (!ui || S.fw !== 'auto') return;
    for (const k of ['single', 'leg', 'sync', 'servoTest']) {
      const f = FW[k];
      if ((f.keys || []).some(function (key) { return line.indexOf(key) >= 0; })) {
        S.fw = k;
        setFw(k);
        log('✓ 识别到固件：' + FW[k].label + '（快捷按钮/命令参考已切换）');
        return;
      }
    }
  }

  // ================= 日志 =================
  const logs = [];
  function log(msg) {
    push('[' + new Date().toLocaleTimeString('zh-CN', { hour12: false }) + '] ' + msg);
  }
  function tx(cmd) {
    log('→ ' + cmd);
  }
  function push(txt) {
    logs.push(txt);
    while (logs.length > LOG_MAX) logs.shift();
    if (ui) {
      ui.logBox.textContent = logs.join('\n');
      ui.logBox.scrollTop = ui.logBox.scrollHeight;
    }
  }

  boot();
})();
