(function () {
  'use strict';
  const model = window.RobotModel;
  const protocol = window.MotionFrame;
  let ui, twin, frames = [], recording = [], liveHistory = [];
  let origin = null, cursor = 0, painted = null, connected = false;

  function node(tag, label) {
    const element = document.createElement(tag);
    if (label) element.textContent = label;
    return element;
  }

  function create(parent, simulation) {
    twin = simulation;
    const section = node('details');
    section.open = true;
    section.append(node('summary', '命令回放与足端诊断'));
    const controls = node('div');
    controls.className = 'rcBtnRow';
    const file = node('input');
    file.type = 'file';
    file.accept = '.txt,.log,.jsonl';
    file.setAttribute('aria-label', '打开固件命令日志');
    const play = node('button', '播放');
    const save = node('button', '下载串口帧');
    const ground = node('button', '固定机身接地预览');
    const seek = node('input');
    seek.type = 'range';
    seek.min = seek.max = seek.value = '0';
    seek.setAttribute('aria-label', '回放时间');
    const leg = node('select');
    leg.setAttribute('aria-label', '侧视腿号');
    model.pins.forEach((_, index) => {
      const option = node('option', '腿 ' + index);
      option.value = index;
      leg.append(option);
    });
    const status = node('p', '打开命令日志，或连接主控台接收完整帧。');
    status.className = 'dim';
    const canvas = node('canvas');
    canvas.width = 480;
    canvas.height = 214;
    canvas.style.width = '100%';
    canvas.setAttribute('aria-label', '单腿侧视和六腿俯视');
    controls.append(play, save, ground);
    section.append(file, controls, seek, leg, status, canvas);
    parent.append(section);
    ui = { file, play, save, ground, seek, leg, status, canvas, preview: false };
    state();

    file.addEventListener('change', async () => {
      if (!file.files.length || connected) return;
      try {
        const log = await file.files[0].text();
        if (connected) return;
        const next = protocol.readLog(log);
        if (!next.length) throw new Error('文件没有完整命令帧');
        frames = next;
        seek.max = frames[frames.length - 1].elapsed;
        showAt(0);
      } catch (error) {
        ui.status.textContent = error.message;
      }
      file.value = '';
      state();
    });
    play.addEventListener('click', () => {
      if (origin !== null) origin = null;
      else {
        if (cursor >= Number(seek.max)) showAt(0);
        origin = performance.now() - cursor;
      }
      state();
    });
    seek.addEventListener('input', () => { showAt(Number(seek.value)); state(); });
    leg.addEventListener('change', () => { painted = null; draw(); });
    ground.addEventListener('click', () => {
      ui.preview = !ui.preview;
      twin.groundPreview(ui.preview);
      ground.textContent = ui.preview ? '悬空预览' : '固定机身接地预览';
    });
    save.addEventListener('click', () => {
      const url = URL.createObjectURL(new Blob([recording.join('\n') + '\n'], { type: 'text/plain' }));
      const link = node('a');
      link.href = url;
      link.download = 'six-leg-motion.log';
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
  }

  function state() {
    ui.file.disabled = connected;
    ui.play.disabled = ui.seek.disabled = connected || !frames.length;
    ui.save.disabled = !recording.length;
    ui.play.textContent = origin === null ? '播放' : '暂停';
  }

  function showAt(ms) {
    origin = null;
    cursor = ms;
    ui.seek.value = ms;
    twin.setFrame(protocol.atTime(frames, ms));
    draw();
  }

  function connection(open) {
    if (!ui || open === connected) return;
    connected = open;
    origin = null;
    painted = null;
    frames = [];
    liveHistory = [];
    if (open) recording = [];
    ui.status.textContent = open ? '等待主控台完整帧。' : '串口已断开，可打开日志回放。';
    ui.canvas.getContext('2d').clearRect(0, 0, ui.canvas.width, ui.canvas.height);
    state();
  }

  function live(line, frame) {
    recording.push(line);
    if (recording.length > 30000) recording = recording.slice(-29000);
    liveHistory.push(frame);
    if (liveHistory.length > 40) liveHistory.shift();
    ui.save.disabled = false;
  }

  function update(now) {
    if (!ui) return;
    if (origin !== null) {
      cursor = Math.min(Number(ui.seek.max), now - origin);
      const frame = protocol.atTime(frames, cursor);
      if (frame !== twin.frame()) twin.setFrame(frame);
      ui.seek.value = cursor;
      if (cursor >= Number(ui.seek.max)) { origin = null; state(); }
    }
    draw();
  }

  const stageLabel = { idle: '待机', align: '预摆', walk: '行走', recover: '收步', hold: '保持' };
  function draw() {
    const frame = twin.frame();
    if (!frame || painted === frame) return;
    painted = frame;
    const ctx = ui.canvas.getContext('2d');
    const leg = Number(ui.leg.value);
    const angles = frame.outputs.map(output => output ? output.applied : null);
    const poses = model.pins.map((_, index) => protocol.joints(index, angles));
    const stance = poses.flatMap((pose, index) => {
      if (!pose || !(frame.support & (1 << index))) return [];
      const heading = protocol.direction(protocol.legAzimuth(index, frame.mirror));
      const r = window.SimCore.CONFIG.hipRadius - pose.foot.x;
      return [{x: heading.x * r, z: heading.z * r}];
    });
    const hull = protocol.supportHull(stance);
    const limited = frame.outputs.flatMap((output, ch) => output && output.limited ? ['ch' + ch] : []);
    const support = hull.margin === null ? '—' : hull.margin.toFixed(2) + ' cm';
    ui.status.textContent = (connected ? '串口' : '日志') + ' · 校准 v' + model.version +
      ' · 帧 ' + frame.sequence + ' · ' + stageLabel[frame.stage] +
      ' · 已落足 ' + frame.completed + '/' + frame.issued + ' 拍 · 计划支撑裕量 ' + support +
      (limited.length ? ' · 限幅 ' + limited.join(', ') : '');

    ctx.clearRect(0, 0, 480, 214);
    ctx.fillStyle = '#14161d';
    ctx.fillRect(0, 0, 480, 214);
    ctx.fillStyle = '#e2e4ef';
    ctx.font = '13px sans-serif';
    ctx.fillText('侧视 · 腿 ' + leg, 12, 19);
    ctx.fillText('俯视 · 机头朝右', 252, 19);
    ctx.fillText('灰 基准　绿 命令推算　橙 目标', 12, 203);
    function line(points, color, dot = false) {
      ctx.strokeStyle = ctx.fillStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      points.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
      ctx.stroke();
      if (dot) for (const [x, y] of points) {
        ctx.beginPath(); ctx.arc(x, y, 3, 0, 2 * Math.PI); ctx.fill();
      }
    }
    const side = ({ x, y }) => [192 + 8 * x, 74 - 8 * y];
    const stand = { x: model.stand[0], y: model.stand[1] };
    const standingKnee = { x: stand.x, y: stand.y + model.shin };
    const index = connected ? -1 : frames.indexOf(frame);
    const history = connected ? liveHistory : frames.slice(Math.max(0, index - 39), index + 1);
    const path = history.flatMap(sample => {
      const pose = protocol.joints(leg, sample.outputs.map(output => output ? output.applied : null));
      return pose ? [pose.foot] : [];
    });
    line(path.map(side), '#3f795e');
    line(history.flatMap(sample => sample.feet[leg] ? [side(sample.feet[leg])] : []), '#946936');
    line([[192, 74], side(standingKnee), side(stand)], '#788397', true);
    if (poses[leg]) line([[192, 74], side(poses[leg].knee), side(poses[leg].foot)], '#6fd08c', true);
    if (frame.feet[leg]) line([side(frame.feet[leg])], '#e3a04d', true);
    const top = (x, z) => [356 + 4.2 * x, 108 + 4.2 * z];
    if (hull.points.length >= 3) line([...hull.points, hull.points[0]].map(p => top(p.x, p.z)), '#476c8f');
    line([top(0, 0), top(8, 0)], '#db5866', true);
    const hips = window.SimCore.CONFIG.hipRadius;
    const axis = protocol.direction(protocol.legAzimuth(leg, frame.mirror));
    line(path.map(p => top((hips - p.x) * axis.x, (hips - p.x) * axis.z)), '#3f795e');
    poses.forEach((pose, index) => {
      const heading = protocol.direction(protocol.legAzimuth(index, frame.mirror));
      const point = x => top((hips - x) * heading.x, (hips - x) * heading.z);
      if (pose) line([point(0), point(pose.foot.x)], '#6fd08c', true);
      if (frame.feet[index]) line([point(frame.feet[index].x)], '#e3a04d', true);
    });
  }

  window.ReplayPanel = { create, update, connection, live };
})();
