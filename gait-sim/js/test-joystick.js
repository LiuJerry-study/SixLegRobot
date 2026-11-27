/* ============================================================
 * test-joystick.js — 单元测试（node 运行）: 真机遥控「🕹 圆形移动键」
 *   ① 从 control-panel.js 原文抽出纯几何 const JOY（不依赖 DOM）→ 断言"屏幕坐标 → 方向档"映射:
 *        正上方 = 0 档(leg0 正前)、左 = 60° 档(左前)…; 边界(30°/90°…)归属确定; 死区/旋钮半径常量
 *   ② 与固件 test/MasterControl.cpp 的 6 档方向表逐项对齐（命令键 / 方位名 / 角度）—— 防止只改一边
 *   ③ 静态检查: 旧的「🎯 目标点遥控（点地面 = 全体足端 goto / setManual 遥控姿态）」已彻底移除;
 *        拖动 → 发方向键、松手 → stop、断开串口前补 stop、失焦/切后台当松手, 这些安全线都在
 *   ④ main.js: manual 遥控姿态分支已下线; SimCore.setPhi 已导出（摇杆只指方向、不驱动场景）
 *   不覆盖: 真实 Web Serial、真实鼠标事件、canvas 绘制像素
 * 运行: node js/test-joystick.js
 * ============================================================ */
'use strict';
const fs = require('fs');
const path = require('path');
const cpSrc = fs.readFileSync(path.join(__dirname, 'control-panel.js'), 'utf8');
const mainSrc = fs.readFileSync(path.join(__dirname, 'main.js'), 'utf8');
const fwSrc = fs.readFileSync(path.join(__dirname, '..', '..', 'test', 'MasterControl.cpp'), 'utf8');

// ---- 从 control-panel.js 抽出纯几何 const JOY = { … };（花括号配对）----
const jStart = cpSrc.indexOf('const JOY = {');
if (jStart < 0) throw new Error('control-panel.js 里找不到 const JOY = {...}（常量被改名了？）');
let i = cpSrc.indexOf('{', jStart), depth = 0, jEnd = -1;
for (; i < cpSrc.length; i++) {
  const c = cpSrc[i];
  if (c === '{') depth++;
  else if (c === '}') { depth--; if (depth === 0) { jEnd = i + 1; break; } }
}
const JOY = new Function(cpSrc.slice(jStart, jEnd) + '; return JOY;')();

// ---- 从固件抽出 6 档方向表 ----
function grab(re, what) {
  const m = fwSrc.match(re);
  if (!m) throw new Error('MasterControl.cpp 里找不到 ' + what + '（固件改过方向表？）');
  return m[1];
}
const fwKeys = grab(/static const char\* K\[6\] = \{([^}]*)\};/, '方向键表 K[6]')
  .split(',').map(s => s.trim().replace(/"/g, '')).filter(Boolean);
const fwNames = grab(/static const char\* N\[6\] = \{([^}]*)\};/, '方位名表 N[6]')
  .split(',').map(s => s.trim().replace(/"/g, '')).filter(Boolean);

// ---- 断言工具 ----
let pass = 0, fail = 0;
function eq(what, got, want) {
  const g = JSON.stringify(got), w = JSON.stringify(want);
  if (g === w) { pass++; console.log('  \u2713 ' + what + ' = ' + g); }
  else { fail++; console.log('  \u2717 ' + what + ' = ' + g + '  (期望 ' + w + ')'); }
}

const R = 100;
const at = deg => {                      // 本页口径的方位角(上=0, 逆时针为正) → 圆盘上的屏幕坐标
  const a = deg * Math.PI / 180;
  return { dx: -Math.sin(a) * R, dy: -Math.cos(a) * R };
};
const secAt = deg => { const p = at(deg); return JOY.sectorOf(p.dx, p.dy); };

console.log('— ① 几何: 圆盘角度 → 方向档（正上方 = leg0 正前）—');
eq('常量 DEAD / KNOB', [JOY.DEAD, JOY.KNOB], [0.22, 0.80]);
eq('档位数', JOY.KEYS.length, 6);
[0, 60, 120, 180, 240, 300].forEach((deg, k) => eq('正 ' + deg + '° → 档 ' + k + ' ' + JOY.KEYS[k], secAt(deg), k));
eq('档位角度', JOY.ANGLES, [0, 60, 120, 180, 240, 300]);

console.log('\n— 边界(30°/90°…)归属确定: 判给角度较小的一侧 —');
[[30, 0], [90, 1], [150, 2], [210, 3], [270, 4], [330, 5], [-30, 5]].forEach(([deg, k]) =>
  eq('边界 ' + deg + '° → 档 ' + k, secAt(deg), k));
eq('刚进一档(31°)', secAt(31), 1);
eq('差一点(29°)', secAt(29), 0);
eq('正上方(0,-R)', JOY.sectorOf(0, -R), 0);
eq('正左(-R,0) = 左前/左后边界', JOY.sectorOf(-R, 0), 1);
eq('正下方(0,R)', JOY.sectorOf(0, R), 3);
eq('正右(R,0)', JOY.sectorOf(R, 0), 4);
eq('极近中心的坐标也能算(不 NaN)', Number.isFinite(JOY.sectorOf(0.001, -0.001)), true);

console.log('\n— ② 与固件 test/MasterControl.cpp 的方向表逐项对齐 —');
eq('命令键 f/fl/bl/b/br/fr', JOY.KEYS, fwKeys);
eq('方位名 前/左前/左后/后/右后/右前', JOY.LABELS, fwNames);
eq('角度 = 60°×档（固件 dirKey(i) → 60×i）', JOY.ANGLES, fwKeys.map((_, k) => 60 * k));
eq('固件方向键 = 设方向 + 直接开走', /if \(first == dirKey\(i\)\) \{ setDirIndex\(i, true\); return; \}/.test(fwSrc), true);
eq('固件 stop 委托给受控收步', /first == "stop" \|\| first == "halt"\) \{ requestStop\(\)/.test(fwSrc)
  && /if \(gWalk\) gGait.stop\(\);/.test(fwSrc), true);
eq('固件 @STATE 回传 dir/walk', /@STATE dir=%\.1f sqo=%d walk=%d/.test(fwSrc), true);

console.log('\n— ③ control-panel.js: 拖动/松手/安全停 都接上了 —');
eq('旧目标点遥控已移除(勾选框/拾取/标记环)', /rcTargetChk|pickFromPointer|createMarkers|updateMarkers|onTargetToggle/.test(cpSrc), false);
eq('不再调用 SimCore.setManual', /setManual/.test(cpSrc), false);
eq('圆盘 canvas + pointerdown 绑定', /addEventListener\('pointerdown', onPadDown\)/.test(cpSrc) &&
  /class = 'joyPad'|className = 'joyPad'/.test(cpSrc), true);
eq('拖动换向 → 发送合并后的命令', /commandDir\(k, angle\)/.test(cpSrc) && /sendDirThrottled\(cmd\)/.test(cpSrc), true);
eq('松手 → endDrag → stopRobot', /endDrag\('松手'\)/.test(cpSrc) &&
  /function endDrag\(reason\)[\s\S]{0,420}?stopRobot\(reason, false\)/.test(cpSrc), true);
eq('stopRobot 发 stop', /function stopRobot\(reason, force\)[\s\S]{0,520}?send\('stop'\)/.test(cpSrc), true);
eq('断开串口前补 stop（行走中不裸关串口）', /S\.walking\) \{[\s\S]{0,260}?await send\('stop'\)/.test(cpSrc), true);
eq('窗口失焦 / 页面切后台 = 松手', /addEventListener\('blur'[\s\S]{0,80}?endDrag\('窗口失焦'\)/.test(cpSrc) &&
  /visibilitychange[\s\S]{0,120}?endDrag\('页面切后台'\)/.test(cpSrc), true);
eq('换档节流 400ms（固件换向要停→回站姿→重新起步）', /DIR_THROTTLE_MS = 400/.test(cpSrc), true);
eq('@STATE 只用于显示, 不清本地行走标志', /S\.fw\.walk = \(kv\.walk === '1'\)/.test(cpSrc) && !/S\.walking = \(kv\.walk/.test(cpSrc), true);
eq('导出 RcPanel.deactivate / isWalking', /deactivate: function/.test(cpSrc) && /isWalking: function/.test(cpSrc), true);

console.log('\n— ④ main.js: 遥控姿态分支下线, 只留 setPhi 指方向 —');
eq('manual 遥控姿态已移除', /setManual|isManualOn|manualTargets|const manual = /.test(mainSrc), false);
eq('导出 SimCore.setPhi（只改方向箭头/滑块）', /setPhi\(deg\) \{[\s\S]{0,220}?syncPhiUI\(\);/.test(mainSrc), true);
eq('主循环不再有 manualOn 分支', /manualOn/.test(mainSrc), false);

console.log('\n— ⑤ 面板冒烟（最小 DOM + canvas 桩, 跑真代码 + 模拟拖动）—');
// 够 control-panel.js 建面板 + 模拟指针事件的极简 DOM
const ctxStub = new Proxy({}, { get: (t, k) => (k in t ? t[k] : () => {}), set: (t, k, v) => (t[k] = v, true) });
function mkEl(tag) {
  return {
    tagName: String(tag || '').toUpperCase(), id: '', className: '', textContent: '', value: '',
    type: '', placeholder: '', style: {}, children: [], _h: {},
    appendChild(c) { this.children.push(c); return c; },
    insertBefore(c, anchor) {
      const i = this.children.indexOf(anchor);
      if (i < 0) this.children.push(c); else this.children.splice(i, 0, c);
      return c;
    },
    remove() {},
    addEventListener(t, f) { (this._h[t] = this._h[t] || []).push(f); },
    classList: {
      _s: new Set(), add(c) { this._s.add(c); }, remove(c) { this._s.delete(c); },
      contains(c) { return this._s.has(c); }
    },
    getBoundingClientRect() { return { left: 0, top: 0, width: PAD_SMOKE, height: PAD_SMOKE }; },
    getContext() { return ctxStub; },
    fire(t, ev) {
      (this._h[t] || []).forEach(f => f(Object.assign(
        { preventDefault() {}, pointerType: 'mouse', button: 0, pointerId: 1 }, ev)));
    }
  };
}
function findEl(root, pred) {
  if (pred(root)) return root;
  for (const c of root.children || []) { const r = findEl(c, pred); if (r) return r; }
  return null;
}
const PAD_SMOKE = 216;                 // 与 control-panel.js 里的 PAD 一致
const phiCalls = [];
const panelEl = mkEl('div'), notesEl = mkEl('details');
const docStub = {
  _els: { panel: panelEl, notes: notesEl, rcSection: null },
  getElementById(id) { return this._els[id] || null; },
  createElement(t) { return mkEl(t); },
  addEventListener() {}, hidden: false, activeElement: null
};
const winStub = {
  SimCore: { robot: {}, isRc: () => true, setPhi: d => phiCalls.push(d) },
  devicePixelRatio: 1, _h: {},
  addEventListener(t, f) { (this._h[t] = this._h[t] || []).push(f); }
};
new Function('window', 'document', cpSrc)(winStub, docStub);   // 跑真代码（boot() 会同步建面板）★

const padEl = findEl(panelEl, e => e.className === 'joyPad');
const statEl = findEl(panelEl, e => e.id === 'rcJoyStat');
const logEl = findEl(panelEl, e => e.id === 'rcLog');
const fwEl = findEl(panelEl, e => e.id === 'rcJoyFw');
eq('建出圆盘 canvas', !!padEl, true);
eq('建出状态行 / 固件回传行 / 日志', !!statEl && !!fwEl && !!logEl, true);
eq('初始状态 = 待机', /待机/.test(statEl.textContent), true);

padEl.fire('pointerdown', { clientX: PAD_SMOKE / 2, clientY: 20, pointerId: 7 });   // 正上方
eq('拖到正上方 → 方向 φ=0（leg0 正前）', phiCalls.slice(), [0]);
eq('未连接 = 预览提示（真机不动）', /未连接串口：圆盘只做预览/.test(logEl.textContent), true);
eq('未连接拖动 → 状态"拖动中", 但没在走（不发方向命令）',
  /拖动中/.test(statEl.textContent) && !/行走中/.test(statEl.textContent) &&
  winStub.RcPanel.isWalking() === false, true);

padEl.fire('pointermove', { clientX: 25, clientY: PAD_SMOKE / 2, pointerId: 7 });   // 正左 → 边界 90°
eq('拖到正左 → 方向 φ=60（左前档，边界判给角度较小侧）', phiCalls.slice(), [0, 60]);

padEl.fire('pointermove', { clientX: PAD_SMOKE / 2, clientY: PAD_SMOKE / 2 + 6, pointerId: 7 });   // 回中心死区
eq('拖回中心死区 → 不再指新方向', phiCalls.slice(), [0, 60]);

padEl.fire('pointerup', { pointerId: 7 });                                          // 松手
eq('松手 → 圆盘复位（状态回待机）', /待机/.test(statEl.textContent), true);
eq('松手 → 指针线归零（未连接时无需 stop）', padEl.classList.contains('active'), false);

padEl.fire('pointerdown', { clientX: PAD_SMOKE / 2 + 4, clientY: PAD_SMOKE / 2 + 4, pointerId: 8 });  // 死区起步
eq('中心死区起步 → 一个方向都不发', phiCalls.slice(), [0, 60]);
eq('窗口失焦 = 松手（安全线已挂）', (winStub._h.blur || []).length, 1);
eq('窗口 pointerup 兜底 = 松手（安全线已挂）', (winStub._h.pointerup || []).length, 1);

console.log('\n— ⑥ 已连接端到端（假串口）: 拖 = 走 · 松 = 停 · 断开前补 stop —');
const sleep = ms => new Promise(r => setTimeout(r, ms));

// 假端口可返回新版固件能力，也可模拟旧版。
const written = [];
let firmwareReply = '';
function fakePort() {
  let replied = false;
  return {
    open: async () => {},
    close: async () => {},
    readable: { getReader: () => ({
      read: () => {
        if (!replied && firmwareReply) {
          replied = true;
          return Promise.resolve({value: new TextEncoder().encode(firmwareReply), done: false});
        }
        return new Promise(() => {});
      },
      releaseLock() {}, cancel() {}
    }) },
    writable: {
      getWriter: () => ({
        write: d => { written.push(new TextDecoder().decode(d).trim()); return Promise.resolve(); },
        releaseLock() {}
      })
    }
  };
}

(async function () {
  const panel2 = mkEl('div');
  const doc2 = {
    _els: { panel: panel2, notes: mkEl('details'), rcSection: null },
    getElementById(id) { return this._els[id] || null; },
    createElement(t) { return mkEl(t); },
    addEventListener() {}, hidden: false, activeElement: null
  };
  const win2 = {
    SimCore: { robot: {}, isRc: () => true, setPhi: () => {} },
    devicePixelRatio: 1, _h: {},
    addEventListener(t, f) { (this._h[t] = this._h[t] || []).push(f); }
  };
  const nav2 = { serial: { requestPort: async () => fakePort() } };
  new Function('window', 'document', 'navigator', cpSrc)(win2, doc2, nav2);   // ★ 带 navigator 桩再跑一遍

  const pad2 = findEl(panel2, e => e.className === 'joyPad');
  const stat2 = findEl(panel2, e => e.id === 'rcJoyStat');
  const btn2 = findEl(panel2, e => e.textContent === '🔌 连接串口');
  const at2 = (fx, fy) => ({ clientX: PAD_SMOKE * fx, clientY: PAD_SMOKE * fy, pointerId: 3 });

  btn2.fire('click', {});
  await sleep(10);
  eq('连上后请求固件状态', written.slice(), ['state']);
  eq('连接状态 = 已连接', win2.RcPanel.isConnected(), true);

  pad2.fire('pointerdown', at2(0.5, 0.10));            // 拖到正上方 = 前 0°
  await sleep(10);
  eq('拖动 → 立即发方向键 f（固件 = 设方向 + 开走）', written.slice(), ['state', 'f']);
  eq('状态 = 行走中', /行走中/.test(stat2.textContent) && win2.RcPanel.isWalking(), true);

  pad2.fire('pointermove', at2(0.14, 0.42));           // 迅速划到左前 60°，落在 400ms 节流窗口内
  await sleep(10);
  eq('换档被节流（不立刻连发）', written.slice(), ['state', 'f']);
  await sleep(500);
  eq('节流窗口到点 → 补发最后一档 fl', written.slice(), ['state', 'f', 'fl']);

  pad2.fire('pointerup', { pointerId: 3 });            // 松手
  await sleep(10);
  eq('松手 → stop', written.slice(), ['state', 'f', 'fl', 'stop']);
  eq('松手后不再认为在走', win2.RcPanel.isWalking(), false);
  eq('松手后状态回待机', /待机/.test(stat2.textContent), true);

  pad2.fire('pointerdown', at2(0.5, 0.10));            // 再次开走, 然后直接断开串口
  await sleep(420);
  const before = written.length;
  await win2.RcPanel.disconnect();
  eq('行走中断开串口 → 先补 stop 再关', written.slice(before).indexOf('stop') >= 0 &&
    written[written.length - 1] === 'stop', true);
  eq('断开后 = 未连接', win2.RcPanel.isConnected(), false);

  firmwareReply = '@STATE dir=0 walk=0 gait=three lease=750\n';
  written.length = 0;
  btn2.fire('click', {});
  await sleep(10);
  pad2.fire('pointerdown', at2(0.5, 0.10));
  await sleep(300);
  eq('新版固件使用 drive 和续期', written.slice(0, 3), ['state', 'drive 0', 'keepalive']);
  pad2.fire('pointerup', { pointerId: 3 });
  await sleep(300);
  eq('松手后不再续期', written.at(-1), 'stop');
  await win2.RcPanel.disconnect();

  console.log('\n— 平台信息（不判定成败）—');
  const ini = fs.readFileSync(path.join(__dirname, '..', '..', 'platformio.ini'), 'utf8');
  const active = (ini.match(/^\s*(-DMODE_[A-Z_]+)/m) || [])[1];
  console.log('  当前 platformio.ini 生效固件模式: ' + (active || '未找到') +
    (active === '-DMODE_MASTER_CONTROL' ? ' ✓ 与圆形移动键的方向命令一致' : '  ⚠ 圆形移动键的方向命令需要 -DMODE_MASTER_CONTROL'));

  console.log('\n==== 通过 ' + pass + ' / 失败 ' + fail + ' ====');
  process.exit(fail ? 1 : 0);
})();
