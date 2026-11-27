/* ============================================================
 * test-mode-switch.js — 单元测试（node 运行）: 顶部三模式（悬空/触地/真机遥控）切换规则
 *   把 main.js 里的 setAppMode() 原文抽出来，喂一个最小 DOM + SimCore 桩，逐次切模式后断言：
 *     面板显隐(rcSection/groundOnly/twinPanel/notes/playSection) · 模式按钮 .active ·
 *     地面网格/吊架显隐 · 串口互斥断开(TwinPanel/RcPanel) · 离开遥控模式撤销目标点遥控 ·
 *     触地↔遥控保留机身位置（不 resetSim）、悬空↔其它回地面重来
 *   另静态检查 control-panel.js 的初始显隐、插入锚点、RcPanel.deactivate 导出
 *   不覆盖: 真实 Web Serial、Rapier 物理、three.js 渲染
 * 运行: node js/test-mode-switch.js
 * ============================================================ */
'use strict';
const fs = require('fs');
const path = require('path');
const mainSrc = fs.readFileSync(path.join(__dirname, 'main.js'), 'utf8');
const cpSrc = fs.readFileSync(path.join(__dirname, 'control-panel.js'), 'utf8');

// ---- 抽出 setAppMode(mode) 函数原文（花括号配对）----
const start = mainSrc.indexOf('function setAppMode(mode) {');
if (start < 0) throw new Error('main.js 里找不到 setAppMode(mode)（函数被改名了？）');
let i = mainSrc.indexOf('{', start), depth = 0, end = -1;
for (; i < mainSrc.length; i++) {
  const c = mainSrc[i];
  if (c === '{') depth++;
  else if (c === '}') { depth--; if (depth === 0) { end = i + 1; break; } }
}
const fnSrc = mainSrc.slice(start, end);

// ---- 最小 DOM ----
const els = {};
function mkEl(id) {
  const set = new Set();
  const el = {
    id, style: {}, textContent: '', children: [],
    classList: {
      add: c => set.add(c), remove: c => set.delete(c),
      contains: c => set.has(c),
      toggle: (c, on) => { const v = (on === undefined) ? !set.has(c) : !!on; v ? set.add(c) : set.delete(c); return v; }
    },
    _classes: set
  };
  els[id] = el;
  return el;
}
['groundOnly', 'playSection', 'twinPanel', 'rcSection', 'dispSection', 'notes', 'followRow',
 'modeGroundBtn', 'modeHangBtn', 'modeRcBtn'].forEach(mkEl);
els.modeGroundBtn.classList.add('active');     // index.html 里触地是默认模式
els.rcSection.style.display = 'none';          // control-panel.js 建面板时的初始显隐

const shimDoc = { getElementById: id => els[id] || null };

// ---- 最小 SimCore / 场景桩, 暴露 setAppMode 原文 ----
const factory = new Function('document', 'shim', `
  let appMode = 'ground';
  let hangStand = { visible: false };
  let grid = { visible: true };
  let physicsOK = false;
  let HANG_Y = 46;
  const S = { running: false };
  const twin = { cur: [] };
  const Physics = { ready: () => false, setPosition: () => {} };
  const robot = { root: { position: { set: () => {} }, quaternion: { identity: () => {} } }, applyServoAngles: () => {} };
  const controls = { target: { set: () => {} } };
  const panel = { playPause: { textContent: '' } };
  const window = {
    TwinPanel: { disconnect: () => shim.calls.push('twin.disconnect') },
    RcPanel:  { disconnect: () => shim.calls.push('rc.disconnect'),
                deactivate: () => shim.calls.push('rc.deactivate') }
  };
  const applyGroundFx = v => shim.calls.push('applyGroundFx(' + v + ')');
  const resetSim = () => shim.calls.push('resetSim');
  ${fnSrc}
  return {
    setAppMode,
    state: () => ({ appMode, hangStand: hangStand.visible, grid: grid.visible, running: S.running })
  };
`);
const shim = { calls: [] };
const app = factory(shimDoc, shim);

// ---- 断言工具 ----
let pass = 0, fail = 0;
function shown(id) {
  const el = els[id];
  if (!el) return 'MISSING';
  return (el._classes.has('hidden') || el.style.display === 'none') ? 'hidden' : 'shown';
}
function eq(what, got, want) {
  if (got === want) { pass++; console.log('  \u2713 ' + what + ' = ' + got); }
  else { fail++; console.log('  \u2717 ' + what + ' = ' + got + '  (期望 ' + want + ')'); }
}
function activeModes() {
  return ['modeHangBtn', 'modeGroundBtn', 'modeRcBtn'].filter(id => els[id]._classes.has('active')).join(',');
}
function calls() { return shim.calls.join(' | '); }
function reset() { shim.calls.length = 0; }

console.log('— 初始(触地为默认模式, 遥控面板建好后应自动收起) —');
eq('rcSection', shown('rcSection'), 'hidden');
eq('模式按钮 active', activeModes(), 'modeGroundBtn');

console.log('\n— ground → rc(遥控面板上位, 纯仿真参数区收起) —');
reset();
app.setAppMode('rc');
eq('rcSection', shown('rcSection'), 'shown');
eq('groundOnly', shown('groundOnly'), 'hidden');
eq('twinPanel', shown('twinPanel'), 'hidden');
eq('notes', shown('notes'), 'hidden');
eq('playSection', shown('playSection'), 'shown');
eq('模式按钮 active', activeModes(), 'modeRcBtn');
eq('地面网格', app.state().grid, true);
eq('appMode', app.state().appMode, 'rc');
eq('断开孪生串口', calls(), 'twin.disconnect | applyGroundFx(true)');

console.log('\n— rc → ground(断开遥控串口 + 撤销目标点遥控) —');
reset();
app.setAppMode('ground');
eq('rcSection', shown('rcSection'), 'hidden');
eq('groundOnly', shown('groundOnly'), 'shown');
eq('模式按钮 active', activeModes(), 'modeGroundBtn');
eq('断开+撤销+复位', calls(), 'twin.disconnect | rc.disconnect | rc.deactivate | applyGroundFx(true) | resetSim');
eq('S.running 恢复', app.state().running, true);

console.log('\n— ground → hang(孪生面板上位) —');
reset();
app.setAppMode('hang');
eq('twinPanel', shown('twinPanel'), 'shown');
eq('rcSection', shown('rcSection'), 'hidden');
eq('groundOnly', shown('groundOnly'), 'hidden');
eq('notes', shown('notes'), 'hidden');
eq('playSection', shown('playSection'), 'hidden');
eq('地面网格', app.state().grid, false);
eq('吊架', app.state().hangStand, true);
eq('模式按钮 active', activeModes(), 'modeHangBtn');
eq('断开遥控串口', calls(), 'rc.disconnect | applyGroundFx(false)');

console.log('\n— hang → rc(悬空进遥控: 回地面并重来) —');
reset();
app.setAppMode('rc');
eq('rcSection', shown('rcSection'), 'shown');
eq('groundOnly', shown('groundOnly'), 'hidden');
eq('吊架收起', app.state().hangStand, false);
eq('模式按钮 active', activeModes(), 'modeRcBtn');
eq('断开孪生串口 + 回地面', calls(), 'twin.disconnect | applyGroundFx(true) | resetSim');
eq('S.running', app.state().running, true);

console.log('\n— rc → ground → rc(第二次切不重置机身位置/相机) —');
reset();
app.setAppMode('ground');
reset();
app.setAppMode('rc');
eq('rcSection', shown('rcSection'), 'shown');
eq('无 resetSim', calls(), 'twin.disconnect | applyGroundFx(true)');

console.log('\n— 重复点同一模式 / 非法模式名 = 无操作 —');
reset();
app.setAppMode('rc');
eq('调用', calls(), '');
app.setAppMode('bogus');
eq('appMode 未变', app.state().appMode, 'rc');

console.log('\n— control-panel.js 静态检查 —');
eq('建面板时按 isRc 设初始显隐', /sec\.style\.display\s*=\s*\(window\.SimCore\.isRc[\s\S]{0,60}?'none'/.test(cpSrc), true);
eq('插入锚点 = #groundOnly(紧贴模式开关)', /getElementById\('groundOnly'\)\s*\|\|\s*notes/.test(cpSrc) &&
  /panel\.insertBefore\(sec,\s*anchor\)/.test(cpSrc), true);
eq('导出 RcPanel.deactivate', /deactivate:\s*function/.test(cpSrc), true);

console.log('\n==== 通过 ' + pass + ' / 失败 ' + fail + ' ====');
process.exit(fail ? 1 : 0);
