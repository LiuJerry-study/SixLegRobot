(function (g) {
  'use strict';
  const model = g.RobotModel;
  const radians = Math.PI / 180;
  const channels = new Map();
  model.pins.forEach((pair, leg) => pair.forEach((channel, joint) => channels.set(channel, { leg, joint })));

  function legAzimuth(leg, mirror = 1) { return mirror * leg * model.spacing; }
  function direction(degrees) { return { x: Math.cos(degrees * radians), z: -Math.sin(degrees * radians) }; }

  function logicalFromApplied(channel, applied) {
    const id = channels.get(channel);
    return id ? applied - model.offsets[id.leg][id.joint] : null;
  }

  function joints(leg, applied) {
    const [femur, tibia] = model.pins[leg];
    if (!Number.isFinite(applied[femur]) || !Number.isFinite(applied[tibia])) return null;
    const alpha = (model.thighZero - logicalFromApplied(femur, applied[femur])) * radians;
    const turn = -logicalFromApplied(tibia, applied[tibia]) * radians;
    const knee = { x: -model.thigh * Math.cos(alpha), y: model.thigh * Math.sin(alpha) };
    const foot = {
      x: knee.x - model.shin * Math.cos(alpha + turn),
      y: knee.y + model.shin * Math.sin(alpha + turn)
    };
    return { alpha, turn, knee, foot };
  }

  function parse(line) {
    if (!line.startsWith('@FRAME ')) return null;
    const fields = line.trim().split(/\s+/);
    if (fields[1] !== String(model.version)) throw new Error('校准版本不匹配：' + fields[1]);
    if (fields.length !== 9 + model.channelCount + model.pins.length) return null;
    const [sequence, ms, issued, completed, support, mirror] = [2, 3, 5, 6, 7, 8].map(i => Number(fields[i]));
    const stage = fields[4];
    if (![sequence, ms, issued, completed, support].every(n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff) ||
        support > 63 || issued < completed || ![-1, 1].includes(mirror) ||
        !['idle', 'align', 'walk', 'recover', 'hold'].includes(stage)) return null;
    const start = 9 + model.channelCount;
    const outputs = fields.slice(9, start).map(token => {
      if (token === '-') return null;
      const values = token.split(',');
      if (values.length !== 3 || !values.every(Boolean)) return false;
      const [logical, applied, limited] = values.map(Number);
      if (!Number.isFinite(logical) || !Number.isFinite(applied) || applied < model.limit[0] ||
          applied > model.limit[1] || ![0, 1].includes(limited)) return false;
      return { logical, applied, limited: limited === 1 };
    });
    const feet = fields.slice(start).map(token => {
      if (token === '-') return null;
      const parts = token.split(',');
      const [x, y] = parts.map(Number);
      return parts.length === 2 && parts.every(Boolean) && Number.isFinite(x) && Number.isFinite(y) ? { x, y } : false;
    });
    if (outputs.includes(false) || feet.includes(false)) return null;
    return { sequence, ms, stage, issued, completed, support, mirror, outputs, feet };
  }

  function readLog(source) {
    const frames = [];
    for (const line of source.split(/\r?\n/)) {
      const frame = parse(line.trim());
      if (!frame) continue;
      if (frames.length) {
        const previous = frames[frames.length - 1];
        const step = (frame.ms - previous.ms) >>> 0;
        const sequenceStep = (frame.sequence - previous.sequence) >>> 0;
        if (sequenceStep === 0 || sequenceStep > 0x7fffffff || step > 0x7fffffff)
          throw new Error('日志含多个会话或帧顺序错误');
        frame.elapsed = previous.elapsed + step;
      } else frame.elapsed = 0;
      frames.push(frame);
    }
    return frames;
  }

  function atTime(frames, ms) {
    let lo = 0, hi = frames.length;
    while (lo < hi) {
      const mid = Math.floor((lo + hi) / 2);
      if (frames[mid].elapsed <= ms) lo = mid + 1;
      else hi = mid;
    }
    return frames[Math.max(0, lo - 1)];
  }

  // Ground-plane convex hull and signed margin for the modeled center at (0, 0).
  function supportHull(feet) {
    if (feet.length < 3) return { points: feet, margin: null };
    const sorted = feet.slice().sort((a, b) => a.x - b.x || a.z - b.z);
    const cross = (a, b, c) => (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
    function half(points) {
      const result = [];
      for (const p of points) {
        while (result.length > 1 && cross(result[result.length - 2], result[result.length - 1], p) <= 0)
          result.pop();
        result.push(p);
      }
      return result.slice(0, -1);
    }
    const points = [...half(sorted), ...half(sorted.reverse())];
    if (points.length < 3) return { points, margin: null };
    const center = { x: 0, z: 0 };
    const margin = Math.min(...points.map((a, i) => {
      const b = points[(i + 1) % points.length];
      return cross(a, b, center) / Math.hypot(b.x - a.x, b.z - a.z);
    }));
    return {points, margin};
  }

  g.MotionFrame = { channels, direction, legAzimuth, logicalFromApplied, joints, parse, readLog, atTime, supportHull };
})(typeof window !== 'undefined' ? window : globalThis);
