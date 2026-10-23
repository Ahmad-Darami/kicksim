import RAPIER from '@dimforge/rapier3d-compat';
import { buildBiped, DT, groups } from './public/biped.js';
await RAPIER.init();
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
world.integrationParameters.dt = DT;
world.integrationParameters.numSolverIterations = 16;
const gb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.5, 0));
world.createCollider(RAPIER.ColliderDesc.cuboid(300, 0.5, 300).setFriction(1).setRestitution(0.6).setCollisionGroups(groups.ground), gb);
const R = buildBiped(RAPIER, world, 0, 0); if (process.env.HIPS) R.hipS = +process.env.HIPS; if (process.env.REFLEX) R.stepReflex = true;
console.log('mass', R.total.toFixed(1), 'joints', Object.keys(R.joints).length);
const push = process.argv[2] ? parseFloat(process.argv[2]) : 0, dir = process.argv[3] ?? 'x';
for (let i = 0; i < 480 * 12; i++) {
  if (i === 480 * 4 && push) R.bodies.torso.applyImpulse({ x: dir === 'x' ? push : 0, y: 0, z: dir === 'z' ? push : 0 }, true);
  R.update(DT); world.step();
  if (i % 960 === 0 || i === 480 * 12 - 1) { const c = R.com(); console.log((i / 480).toFixed(1), 'com', c.p.map((n) => n.toFixed(3)).join(','), 'pelvisY', R.s('pelvis').p[1].toFixed(3), 'tilt', R.tilt.toFixed(3), 'fallen', R.fallen, R.mode); }
}
