import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { DEFAULTS, makeBall, step, vacuumPrediction } from './physics.js';
import { RB, makeRobot, run, robotStep } from './robot-sim.js';

const P = { ...DEFAULTS };
const ball = makeBall(P);
const R = makeRobot();
let lastLaunch = null, reported = true, pending = null;

// --- scene
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x9ad0f5);
const cam = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 1000);
cam.position.set(-9, 3.5, 6);
const controls = new OrbitControls(cam, renderer.domElement);
controls.target.set(-1.5, 1, 0);
scene.add(new THREE.HemisphereLight(0xffffff, 0x446644, 1.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(10, 30, 10); scene.add(sun);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ color: 0x3a8a3a }));
ground.rotation.x = -Math.PI / 2; scene.add(ground);
const grid = new THREE.GridHelper(400, 400, 0xffffff, 0x2f6f2f); grid.position.y = 0.01; scene.add(grid);
const mesh = new THREE.Mesh(new THREE.SphereGeometry(P.r, 24, 16), new THREE.MeshStandardMaterial({ map: checker() }));
scene.add(mesh);
const trailPts = []; const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff0000 })); scene.add(trail);
function checker() {
  const c = document.createElement('canvas'); c.width = c.height = 64; const x = c.getContext('2d');
  for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) { x.fillStyle = (i + j) % 2 ? '#fff' : '#d22'; x.fillRect(i * 8, j * 8, 8, 8); }
  return new THREE.CanvasTexture(c);
}
for (let d = 10; d <= 100; d += 10) { const m = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 6), new THREE.MeshBasicMaterial({ color: 0xffffff })); m.position.set(d, 0.05, 0); scene.add(m); }

// --- robot body: ~1.5 m tall humanoid. Hip at 0.8 m; legs 0.8 m (matches the physics model).
const orange = new THREE.MeshStandardMaterial({ color: 0xff7a1a, metalness: 0.3, roughness: 0.5 });
const white = new THREE.MeshStandardMaterial({ color: 0xf2f2f2, metalness: 0.2, roughness: 0.4 });
const dark = new THREE.MeshStandardMaterial({ color: 0x222233, metalness: 0.5, roughness: 0.3 });
const glow = new THREE.MeshStandardMaterial({ color: 0x33ddff, emissive: 0x33ddff, emissiveIntensity: 1 });
const robot = new THREE.Group(); scene.add(robot);
const add = (geo, mat, parent, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); parent.add(m); return m; };
const upper = new THREE.Group(); upper.position.y = RB.hipH; robot.add(upper);       // pitches (leans) about the hips
add(new THREE.BoxGeometry(0.3, 0.16, 0.34), dark, upper, 0, 0.05, 0);                  // pelvis
add(new THREE.CapsuleGeometry(0.2, 0.35, 6, 12), orange, upper, 0, 0.42, 0);            // torso
add(new THREE.BoxGeometry(0.02, 0.18, 0.2), glow, upper, 0.19, 0.45, 0);                // chest light
add(new THREE.SphereGeometry(0.17, 20, 16), white, upper, 0, 0.92, 0);                  // head
add(new THREE.BoxGeometry(0.1, 0.08, 0.26), glow, upper, 0.12, 0.94, 0);                // visor
add(new THREE.CylinderGeometry(0.01, 0.01, 0.2), dark, upper, 0, 1.18, 0); add(new THREE.SphereGeometry(0.03), glow, upper, 0, 1.29, 0);
const limb = (parent, y, z, len, rad, mat) => { const piv = new THREE.Group(); piv.position.set(0, y, z); parent.add(piv); add(new THREE.CapsuleGeometry(rad, len - 2 * rad, 4, 8), mat, piv, 0, -len / 2, 0); return piv; };
const armL = limb(upper, 0.68, -0.3, 0.6, 0.06, white), armR = limb(upper, 0.68, 0.3, 0.6, 0.06, white);
const legL = limb(robot, RB.hipH, -RB.ell, RB.L, 0.08, dark), legR = limb(robot, RB.hipH, RB.ell, RB.L, 0.08, dark);
for (const l of [legL, legR]) add(new THREE.BoxGeometry(0.28, 0.1, 0.16), orange, l, 0.07, -RB.L, 0);
let lean = 0;
function renderRobot() {
  robot.position.set(R.x, 0, R.z); robot.rotation.y = -R.yaw;
  lean += (clamp(-R.ax / RB.g, -0.4, 0.4) - lean) * 0.15; upper.rotation.z = lean;
  const swing = R.amp * Math.sin(2 * Math.PI * R.cycle);
  legL.rotation.z = swing; legR.rotation.z = R.kickLeg ? R.legTh : -swing;
  armL.rotation.z = -swing * 0.9; armR.rotation.z = R.waveT >= 0 ? 2.7 + Math.sin(R.waveT * 12) * 0.4 : (R.kickLeg ? 0.5 : swing * 0.9);
}
const clamp = (x, l, h) => Math.max(l, Math.min(h, x));

// --- controls
const ids = ['power', 'loft', 'azimuth', 'backspin', 'sidespin', 'g'];
const val = (id) => parseFloat(document.getElementById(id).value);
const sync = () => ids.forEach((id) => (document.getElementById('v_' + id).textContent = val(id)));
ids.forEach((id) => document.getElementById(id).addEventListener('input', sync)); sync();
const dragBox = document.getElementById('drag');
function setSliders(o) { for (const k of ids) if (o[k] !== undefined) document.getElementById(k).value = o[k]; sync(); }

function doKick(o = {}) {
  setSliders({ ...o, power: o.power !== undefined ? o.power * 100 : undefined });
  if (o.drag !== undefined) dragBox.checked = o.drag;
  P.g = val('g'); P.drag = dragBox.checked;
  return run(R, { type: 'kick', power: val('power') / 100, loft: val('loft'), azimuth: val('azimuth'), backspin: val('backspin'), sidespin: val('sidespin') });
}
function doReset() {
  Object.assign(ball, makeBall(P));
  const cb = R.onLaunch; Object.assign(R, makeRobot()); R.onLaunch = cb;
  trailPts.length = 0; lastLaunch = null; reported = true;
}
R.onLaunch = (info) => { lastLaunch = info; trailPts.length = 0; reported = false; };
document.getElementById('kick').onclick = () => doKick();
document.getElementById('reset').onclick = doReset;

// --- websocket to the agent server
const ws = new WebSocket(`ws://${location.host}/ws`);
const send = (o) => ws.readyState === 1 && ws.send(JSON.stringify(o));
const log = document.getElementById('log');
const say = (who, t) => { const p = document.createElement('p'); p.innerHTML = `<b>${who}:</b> `; p.append(t); log.append(p); log.scrollTop = 1e9; };
document.getElementById('f').onsubmit = (e) => { e.preventDefault(); const i = document.getElementById('msg'); if (!i.value) return; say('You', i.value); send({ type: 'chat', text: i.value }); i.value = ''; };
const r2 = (n) => +n.toFixed(2);
const snapshot = () => ({
  time_s: r2(ball.t),
  ball: { position_m: ball.p.map(r2), velocity_mps: ball.v.map(r2), speed_mps: r2(Math.hypot(...ball.v)), height_m: r2(ball.p[1] - P.r), at_rest: ball.stopped },
  robot: { x: r2(R.x), z: r2(R.z), facing_deg: Math.round((R.yaw * 180) / Math.PI), speed_mps: r2(Math.hypot(R.vx, R.vz)) },
  settings: { gravity: P.g, air_resistance: P.drag, bounciness: P.e, magnus_spin_effect: P.magnus },
  robot_limits: { max_walk_speed_mps: RB.vmax, max_acceleration_mps2: RB.amax, leg_length_m: RB.L, leg_motor_torque_Nm: RB.tauMax, leg_inertia_kgm2: RB.legI, ball_mass_kg: P.m },
  last_kick: lastLaunch,
});

ws.onmessage = (ev) => {
  const m = JSON.parse(ev.data);
  if (m.type === 'say') { say('Robot', m.text); if (m.speak) { speechSynthesis.cancel(); speechSynthesis.speak(new SpeechSynthesisUtterance(m.text)); } }
  else if (m.type === 'tool') {
    const { id, name, input } = m;
    const done = (extra = {}) => send({ type: 'result', id, result: { ...snapshot(), ...extra } });
    if (name === 'kick') { pending = id; doKick(input).then((r) => { if (!r.hit) { pending = null; done({ missed: true }); } }); }
    else if (name === 'walk_to') run(R, { type: 'walk', x: input.x, z: input.z }).then(() => done());
    else if (name === 'turn_to') run(R, { type: 'turn', degrees: input.degrees }).then(() => done());
    else if (name === 'wave') run(R, { type: 'wave' }).then(() => done());
    else {
      if (name === 'reset') doReset();
      if (name === 'set_physics') {
        if (input.gravity !== undefined) { P.g = input.gravity; setSliders({ g: input.gravity }); }
        if (input.air_resistance !== undefined) { P.drag = input.air_resistance; dragBox.checked = P.drag; }
        if (input.bounciness !== undefined) P.e = input.bounciness;
        if (input.magnus_spin_effect !== undefined) P.magnus = input.magnus_spin_effect;
      }
      done();
    }
  }
};

// --- main loop: one fixed physics step for robot + ball, rendered at display rate
const DT = 1 / 480; let acc = 0, prev = performance.now();
const hud = document.getElementById('hud');
const goal = new THREE.Vector3();
function frame(now) {
  acc += Math.min((now - prev) / 1000, 0.1); prev = now;
  while (acc >= DT) { robotStep(R, ball, P, DT); step(ball, P, DT); acc -= DT; }
  mesh.position.set(...ball.p);
  mesh.rotation.x += ball.w[0] / 60; mesh.rotation.y += ball.w[1] / 60; mesh.rotation.z += ball.w[2] / 60;
  renderRobot();
  if (!ball.stopped && lastLaunch && (trailPts.length === 0 || mesh.position.distanceTo(trailPts.at(-1)) > 0.2)) { trailPts.push(mesh.position.clone()); trail.geometry.setFromPoints(trailPts); }
  const L = ball.landing;
  hud.textContent = `ball height = ${(ball.p[1] - P.r).toFixed(2)} m   speed = ${Math.hypot(...ball.v).toFixed(2)} m/s\n` +
    `robot speed = ${Math.hypot(R.vx, R.vz).toFixed(2)} m/s   task: ${R.task ? R.task.type + (R.task.phase ? ' / ' + R.task.phase : '') : 'idle'}\n` +
    (lastLaunch ? `kick: foot ${lastLaunch.foot_speed_mps.toFixed(1)} m/s -> ball ${lastLaunch.speed.toFixed(1)} m/s\n` : '') +
    (L ? `LANDED: range ${L.range.toFixed(1)} m, hang ${L.time.toFixed(2)} s, peak ${L.maxHeight.toFixed(1)} m` : '');
  if (L && !reported && lastLaunch) {
    reported = true;
    const vac = vacuumPrediction(lastLaunch.speed, lastLaunch.angle_deg, P.g);
    const result = { ...snapshot(), landing: { range_m: r2(L.range), sideways_m: r2(L.z), hang_time_s: r2(L.time), peak_height_m: r2(L.maxHeight), impact_speed_mps: r2(L.impactSpeed) }, ideal_no_air_prediction: { range_m: r2(vac.range), peak_height_m: r2(vac.maxHeight), hang_time_s: r2(vac.time) } };
    if (pending) { send({ type: 'result', id: pending, result }); pending = null; }
  }
  // camera follows the action
  const ballMoving = lastLaunch && !ball.stopped;
  goal.set(ballMoving ? ball.p[0] : (R.x + ball.p[0]) / 2, 1, ballMoving ? ball.p[2] : (R.z + ball.p[2]) / 2);
  const old = controls.target.clone(); controls.target.lerp(goal, 0.04); cam.position.add(controls.target.clone().sub(old));
  controls.update(); renderer.render(scene, cam); requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
addEventListener('resize', () => { renderer.setSize(innerWidth, innerHeight); cam.aspect = innerWidth / innerHeight; cam.updateProjectionMatrix(); });
