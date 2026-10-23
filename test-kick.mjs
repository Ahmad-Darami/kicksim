import { sim, run, show, wait } from './test-sim.mjs';
run(1.0);
const power = +process.argv[2] || 0.6, loft = +process.argv[3] || 25, azimuth = +process.argv[4] || 0;
show('start');
const r = await wait(sim.run({ type: 'kick', power, loft, azimuth, foot: 'r' }), 150);
console.log('kick', JSON.stringify(r));
show('after'); console.log('fallen?', sim.state().robot.fallen);
