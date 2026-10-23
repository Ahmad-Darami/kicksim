import RAPIER from '@dimforge/rapier3d-compat';
import { buildBiped, DT, groups } from './public/biped.js';
await RAPIER.init();
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
world.integrationParameters.dt = DT; world.integrationParameters.numSolverIterations = 16;
const gb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.5, 0));
world.createCollider(RAPIER.ColliderDesc.cuboid(300, 0.5, 300).setFriction(1).setRestitution(0.6).setCollisionGroups(groups.ground), gb);
const R = buildBiped(RAPIER, world, 0, 0);
const push = parseFloat(process.argv[2] ?? '0'), pushAt = parseFloat(process.argv[3] ?? '4'), dir = process.argv[4] ?? 'x';
for (let i = 0; i < 480 * 1.5; i++) { R.update(DT); world.step(); }
R.speedCmd = 0.5; R.startWalk('r');
let log = (t) => { const c = R.com(); console.log(t.toFixed(1).padStart(5), (R.isStanding() ? 'stand' : R.mode).padEnd(5), R.wk ? R.wk.ph : '-  ', 'com', c.p.map((n) => n.toFixed(2).padStart(6)).join(' '), 'v', c.v[0].toFixed(2), 'tilt', (R.tilt * 57.3).toFixed(1), R.fallen ? 'FALLEN' : ''); };
for (let i = 0; i < 480 * 14; i++) {
  const t = i / 480;
  if (Math.abs(t - 5) < DT / 2) { R.speedCmd = 0; R.wantStand = true; }
  if (push && Math.abs(t - pushAt) < DT / 2) R.bodies.torso.applyImpulse({ x: dir === 'x' ? push : 0, y: 0, z: dir === 'z' ? push : 0 }, true);
  R.update(DT); world.step();
  if (i % 240 === 0) log(t);
  if (R.fallen) { log(t); console.log('FELL', R.fallReason); break; }
}
