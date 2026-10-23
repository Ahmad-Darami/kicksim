import RAPIER from '@dimforge/rapier3d-compat';
import { createSim } from './public/sim.js';
import { DT } from './public/biped.js';
const sim = await createSim(RAPIER);
const run = (secs, every = 0, f = null) => { for (let i = 0; i < secs * 480; i++) { sim.step(); if (every && i % (every * 480) === 0) f?.(); } };
const show = (label) => { const s = sim.state(); console.log(label.padEnd(14), 'robot', s.robot.position_m.join(','), s.robot.mode, 'fallen', s.robot.fallen, 'speed', s.robot.speed_mps, '| ball', s.ball.position_m.join(','), 'at_rest', s.ball.at_rest); };
const wait = async (p, maxSec = 120) => { let done = null; p.then((r) => (done = r)); for (let i = 0; i < maxSec * 480 && !done; i++) { sim.step(); await null; if (i % 4800 === 0) {} } return done; };
export { sim, run, show, wait };
if (process.argv[1].endsWith('test-sim.mjs')) {
  run(1.5); show('settled');
  const r1 = await wait(sim.run({ type: 'walk_to', x: -1.5, z: 0.5 })); console.log('walk_to', JSON.stringify(r1)); show('after walk');
  const r2 = await wait(sim.run({ type: 'turn_to', degrees: 90 })); console.log('turn_to', JSON.stringify(r2)); show('after turn');
  const r3 = await wait(sim.run({ type: 'turn_to', degrees: 0 })); console.log('turn_to', JSON.stringify(r3));
  sim.push(250, 0); run(3); show('after push 250N'); console.log('fallen?', sim.state().robot.fallen);
}
