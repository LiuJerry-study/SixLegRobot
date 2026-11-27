'use strict';
const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {join} = require('node:path');
require('./robot-model.js');
require('./motion-frame.js');
const { RobotModel: model, MotionFrame: motion } = globalThis;
const sample = name => motion.readLog(readFileSync(join(__dirname, '../samples', name + '.txt'), 'utf8'));
const legacy = sample('legacy');
const run = sample('three');
const stand = legacy[0];
const angles = frame => frame.outputs.map(command => command ? command.applied : null);
const expected = [[83.4, 162.6], [67.4, 140.6], [67.4, 162.6], [83.4, 162.6], [67.4, 157.6], [83.4, 162.6]];

for (let leg = 0; leg < 6; leg++) {
  const pose = motion.joints(leg, angles(stand));
  assert(Math.hypot(pose.foot.x + 6, pose.foot.y + 10) < 0.1);
  assert(Math.abs(pose.knee.y - 2.5) < 0.01);
  model.pins[leg].forEach((ch, j) => {
    assert.equal(stand.outputs[ch].applied, expected[leg][j]);
    const logical = motion.logicalFromApplied(ch, stand.outputs[ch].applied);
    assert(Math.abs(logical - stand.outputs[ch].logical) < 0.01);
    assert.equal(logical + model.offsets[leg][j], expected[leg][j]);
  });
}
const clipped = legacy.find(frame => frame.feet[3]?.x === -6 && frame.feet[3]?.y === -7);
assert(clipped);
assert(clipped.outputs[7].limited);
assert.equal(clipped.outputs[7].applied, 180);
assert(Math.abs(clipped.outputs[7].logical - 134.3) < 0.2);
assert.notDeepEqual(clipped.feet[3], motion.joints(3, angles(clipped)).foot);
const partial = angles(stand);
partial[7] = null;
assert.equal(motion.joints(3, partial), null);
const line = readFileSync(join(__dirname, '../samples/legacy.txt'), 'utf8').split('\n')[0];
assert.throws(() => motion.parse(line.replace('@FRAME 1', '@FRAME 2')), /版本/);
assert.equal(motion.parse(line.replace(/ - /, ' abc ')), null);
assert(motion.direction(0).x > 0 && motion.direction(60).z < 0 && motion.direction(300).z > 0);
assert.equal(motion.legAzimuth(1, -1), -60);

function landings(hz) {
  const transitions = [];
  let previous = 0;
  const end = run[run.length - 1].elapsed;
  for (let i = 0; i <= Math.ceil(end / 1000 * hz); i++) {
    const frame = motion.atTime(run, Math.min(i * 1000 / hz, end));
    if (frame.completed !== previous) transitions.push(frame.completed);
    previous = frame.completed;
  }
  return transitions;
}
for (const hz of [30, 60, 120]) assert.deepEqual(landings(hz), [1, 2]);
const last = motion.atTime(run, Infinity);
assert.deepEqual(angles(last), angles(stand));
assert(run.every(frame => frame.outputs.every(output => !output || !output.limited)));
assert(motion.supportHull([{x: 1, z: 0}, {x: -1, z: -1}, {x: -1, z: 1}]).margin > 0);
assert(motion.supportHull([{x: 1, z: 0}, {x: 2, z: -1}, {x: 2, z: 1}]).margin < 0);
console.log('Twelve-channel calibration, legacy clipping, partial frames, left/right and timed playback passed.');
