import { DEFAULTS, makeBall, kick, step, vacuumPrediction } from './public/physics.js';
const run = (over) => {
  const P = { ...DEFAULTS, ...over }, b = makeBall(P);
  kick(b, P, { speed: 20, angle: 45 });
  while (!b.landing && b.t < 20) step(b, P, 1 / 480);
  return b.landing;
};
const vac = run({ drag: false, magnus: false, r: 1e-4 }), pred = vacuumPrediction(20, 45, 9.81);
console.log('vacuum sim', vac.range.toFixed(2), 'analytic', pred.range.toFixed(2));
console.log('with drag  ', run({}).range.toFixed(2));
if (Math.abs(vac.range - pred.range) > 0.3) { console.error('FAIL'); process.exit(1); }
console.log('ok');
