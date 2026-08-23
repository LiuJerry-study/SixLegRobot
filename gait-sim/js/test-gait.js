/* ============================================================
 * test-gait.js — 单元测试 (node 运行)
 *   验证: IK 正解回代 / 三角步态符号与方向 / 波纹一次一腿 / 对腿限制
 * 运行: node js/test-gait.js
 * ============================================================ */
'use strict';
require('./robot.js');
require('./gaits.js');

const { CONFIG, solveIK } = globalThis.Robot;
const { createGaitPlanner } = globalThis.GaitPlanner;
const planner = createGaitPlanner(CONFIG);

let fail = 0;
function check(name, cond) {
  console.log((cond ? '  \u2713 ' : '  \u2717 ') + name);
  if (!cond) fail++;
}
const opts = { sweepMag: 2.0, cycleT: 1, lift: 2.0, pairIndex: 0, bodyHeight: 7.8 };
const R_MID = (CONFIG.R_LO + CONFIG.R_HI) / 2;

console.log('— 蜘蛛折腿 IK 正解验证 (足端位置回代) —');
// 可达环: |L1−L2|=5 ≤ d ≤ L1+L2=21 (取固件站立足/伸出足及中间若干点)
for (const [a, drop] of [[5.0, 6.755], [6.0, 8.0], [12.0, 6.0], [14.9, 6.755], [8.0, 10.0]]) {
  const { th1, th2 } = solveIK(a, drop);
  // 正解: 足端 = L1·(cosα,sinα) + L2·(cos(α+π+β), sin(α+π+β)) = (a, −drop)
  const x = CONFIG.L1 * Math.cos(th1) + CONFIG.L2 * Math.cos(th1 + Math.PI + th2);
  const y = CONFIG.L1 * Math.sin(th1) + CONFIG.L2 * Math.sin(th1 + Math.PI + th2);
  check(`IK(a=${a}, drop=${drop}) 回代 (${x.toFixed(4)}, ${y.toFixed(4)})`,
    Math.abs(x - a) < 1e-6 && Math.abs(y + drop) < 1e-6);
}
{
  // 蜘蛛拱形特征: 固件默认站立足 (a=5, drop=6.755) 大腿上抬、膝盖高于髋、小腿竖直向下
  const { th1, th2 } = solveIK(5.0, 6.755);
  const kneeY = CONFIG.L1 * Math.sin(th1);                       // 膝盖相对髋的高度
  const shinDown = Math.sin(th1 + Math.PI + th2);                // 小腿方向 y 分量
  check('默认站姿: 大腿上抬 (α>0)', th1 > 0);
  check('默认站姿: 膝盖高于髋', kneeY > 0);
  check('默认站姿: 小腿向下折回', shinDown < 0);
  check('默认站姿: 小腿竖直 (脚在膝正下方)', Math.abs(shinDown + 1) < 1e-6);
  // 极限: 直腿 d=L1+L2=21 → β=π; 完全折回 d=|L1−L2|=5 → β=0
  const straight = solveIK(Math.sqrt(441 - 9), 3);               // d=21
  const folded = solveIK(0, 5);                                  // d=5
  check('直腿时膝角 β=180°', Math.abs(straight.th2 - Math.PI) < 1e-6);
  check('完全折回时膝角 β=0°', Math.abs(folded.th2) < 1e-6);
}

console.log('— 三角步态: 分组与符号 —');
const stanceOf = res => res.targets.map((tg, i) => tg.state === 'stance' ? i : -1).filter(i => i >= 0).join(',');
{
  const resA = planner.compute('tripod', 0, 0.05, opts);
  check('t=0.05 时 A 组(0,120,240) 支撑', stanceOf(resA) === '0,2,4');
  const resB = planner.compute('tripod', 0, 0.55, opts);
  check('t=0.55 时 B 组(60,180,300) 支撑', stanceOf(resB) === '1,3,5');
  check('0° 腿 c=+1 (收)', Math.abs(resA.targets[0].coeff - 1) < 1e-9);
  check('180° 腿 c=−1 (伸)', Math.abs(resB.targets[3].coeff + 1) < 1e-9);
  // 前腿支撑相 r 递减(收), 后腿 r 递增(伸)
  const r1 = planner.compute('tripod', 0, 0.05, opts).targets[0].r;   // u=0.1
  const r2 = planner.compute('tripod', 0, 0.35, opts).targets[0].r;   // u=0.7
  check('0° 腿支撑相 r 递减 (收)', r2 < r1);
  const b1 = planner.compute('tripod', 0, 0.55, opts).targets[3].r;   // u=0.1
  const b2 = planner.compute('tripod', 0, 0.85, opts).targets[3].r;   // u=0.7
  check('180° 腿支撑相 r 递增 (伸)', b2 > b1);
}

console.log('— 三角步态: 任意方向净推力 = 目标方向 —');
for (const phi of [0, 30, 90, 135, 210, 300]) {
  const res = planner.compute('tripod', phi, 0.25, opts);
  const ex = Math.cos(phi * Math.PI / 180), ez = Math.sin(phi * Math.PI / 180);
  check(`φ=${phi}° 净方向 (${ex.toFixed(3)},${ez.toFixed(3)})`,
    res.dir && Math.abs(res.dir.x - ex) < 1e-6 && Math.abs(res.dir.z - ez) < 1e-6);
}

console.log('— 波纹步态: 任意时刻只动一条腿 —');
for (const t of [0.01, 0.1666, 0.5, 0.98, 0.8334]) {
  const res = planner.compute('ripple', 45, t, opts);
  const swing = res.targets.filter(tg => tg.state === 'swing').length;
  check(`t=${t} 摆动腿数=${swing} (应为1)`, swing === 1);
}

console.log('— 对腿步态: 沿对轴平移, 垂直轴不动 —');
{
  const res = planner.compute('bipod', 0, 0.05, opts);
  check('φ=0 对轴(0/180) 净方向 +x', res.dir && Math.abs(res.dir.x - 1) < 1e-6);
  check('4 条非驱动腿悬停', res.targets.filter(tg => tg.state === 'hold').length === 4);
  const res2 = planner.compute('bipod', 90, 0.05, opts);
  check('φ=90° (垂直对轴) 不移动', res2.dir === null);
}

console.log('— 随机采样无 NaN / 无异常 —');
{
  let bad = 0;
  for (let i = 0; i < 500; i++) {
    const res = planner.compute('tripod', Math.random() * 360, Math.random() * 10, { ...opts, cycleT: 1.2 });
    for (const tg of res.targets) {
      if (!isFinite(tg.r) || !isFinite(tg.y) || !isFinite(tg.coeff)) bad++;
    }
    if (res.dir && (!isFinite(res.dir.x) || !isFinite(res.dir.z))) bad++;
  }
  check('500 次随机采样无 NaN', bad === 0);
}

console.log(fail === 0 ? '\n全部通过 \u2714' : '\n' + fail + ' 项失败 \u2718');
process.exit(fail === 0 ? 0 : 1);
