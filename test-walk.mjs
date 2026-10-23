import RAPIER from '@dimforge/rapier3d-compat';
import { buildBiped, DT, groups } from './public/biped.js';
await RAPIER.init();
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
world.integrationParameters.dt = DT; world.integrationParameters.numSolverIterations = 16;
const gb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.5, 0));
world.createCollider(RAPIER.ColliderDesc.cuboid(300, 0.5, 300).setFriction(1).setRestitution(0.6).setCollisionGroups(groups.ground), gb);
const R = buildBiped(RAPIER, world, 0, 0);
const speed = parseFloat(process.argv[2] ?? '0.4'), T = parseFloat(process.argv[3] ?? '10'), turn = parseFloat(process.argv[4] ?? '0');
for (let i = 0; i < 480 * 1.5; i++) { R.update(DT); world.step(); }
R.speedCmd = speed; R.turnCmd = turn; R.startWalk('r');
const rollOf = (q) => { const up = [2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z + q.w * q.x)]; return Math.atan2(up[2], up[1]) * 57.3; };
const pitchOf = (q) => { const up = [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z)]; return Math.atan2(-up[0], up[1]) * 57.3; };
let ok = true;
for (let i = 0; i < 480 * T; i++) {
  R.update(DT); world.step();
  if (i % 240 === 0) { const c = R.com(), p = R.s('pelvis'); console.log((i / 480).toFixed(1).padStart(5), R.dbg.ph.padEnd(5), 'step', String(R.wk.n).padStart(2), 'com', c.p.map((n) => n.toFixed(2).padStart(6)).join(' '), 'vx', c.v[0].toFixed(2).padStart(5), 'pelvis roll/pitch', rollOf(p.q).toFixed(1).padStart(5), pitchOf(p.q).toFixed(1).padStart(5), 'tilt(torso)', (R.tilt * 57.3).toFixed(1)); }
  if (R.fallen) { console.log('FELL at', (i / 480).toFixed(2), R.fallReason); ok = false; break; }
}
console.log(ok ? 'STILL STANDING' : 'FAILED');
