import { DEFAULTS as P, makeBall, step } from './public/physics.js';
import { RB, makeRobot, run, robotStep, reach } from './public/robot-sim.js';
const dt = 1 / 480, ball = makeBall(P), R = makeRobot(-4, 0);
let t = 0, maxA = 0, maxV = 0, prevV = 0, launched = null;
R.onLaunch = (i) => (launched = i);
const tick = () => { robotStep(R, ball, P, dt); step(ball, P, dt); t += dt; const v = Math.hypot(R.vx, R.vz); maxV = Math.max(maxV, v); maxA = Math.max(maxA, Math.abs(v - prevV) / dt); prevV = v; };
const idle = () => !R.task && !R.queue.length;
run(R, { type: 'walk', x: -2, z: 1.5 });
tick();
while (!idle() && t < 30) tick();
console.log(`walk done t=${t.toFixed(2)}s pos=(${R.x.toFixed(2)},${R.z.toFixed(2)}) maxV=${maxV.toFixed(2)} maxA=${maxA.toFixed(2)}`);
if (Math.hypot(R.x + 2, R.z - 1.5) > 0.1 || maxV > RB.vmax + 0.05 || maxA > RB.amax * 1.5 + 3) { console.error('FAIL walk'); process.exit(1); }
for (const power of [0.25, 0.5, 1]) {
  Object.assign(ball, makeBall(P)); Object.assign(R, makeRobot(-4, 1)); R.onLaunch = (i) => (launched = i); launched = null;
  run(R, { type: 'kick', power, loft: 25, azimuth: 0, backspin: 0, sidespin: 0 });
  tick();
  while (!idle() && t < 120) tick();
  const t0 = t; while (!ball.landing && t < t0 + 10) tick();
  console.log(`power ${power}: foot ${launched?.foot_speed_mps.toFixed(1)} m/s -> ball ${launched?.speed.toFixed(1)} m/s, range ${ball.landing?.range.toFixed(1)} m, robot at (${R.x.toFixed(2)},${R.z.toFixed(2)})`);
  if (!launched) { console.error('FAIL kick missed'); process.exit(1); }
}
console.log('ok');
