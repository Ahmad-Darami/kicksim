import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { DEFAULTS, makeBall, kick, step, vacuumPrediction } from './physics.js';

const P = { ...DEFAULTS };
const ball = makeBall(P);
let lastKick = null, reported = false, pending = null;

// --- three.js scene
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x9ad0f5);
const cam = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 1000);
cam.position.set(-8, 6, 15);
const controls = new OrbitControls(cam, renderer.domElement);
controls.target.set(15, 2, 0);
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
// distance markers every 10 m
for (let d = 10; d <= 100; d += 10) { const m = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 6), new THREE.MeshBasicMaterial({ color: 0xffffff })); m.position.set(d, 0.05, 0); scene.add(m); }

// --- robot (simulated body; mirrors commands sent to a real robot)
const bodyMat = new THREE.MeshStandardMaterial({ color: 0x4477ff }), headMat = new THREE.MeshStandardMaterial({ color: 0xdddddd });
const robot = new THREE.Group(); scene.add(robot);
const part = (w, h, d, m, y, z = 0, parent = robot) => { const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m); b.position.set(0, y, z); parent.add(b); return b; };
part(0.25, 0.6, 0.45, bodyMat, 1.0); part(0.25, 0.25, 0.25, headMat, 1.5);
const limb = (y, z, len) => { const piv = new THREE.Group(); piv.position.set(0, y, z); robot.add(piv); part(0.12, len, 0.12, headMat, -len / 2, 0, piv); return piv; };
const armL = limb(1.25, -0.3, 0.55), armR = limb(1.25, 0.3, 0.55), legL = limb(0.7, -0.1, 0.7), legR = limb(0.7, 0.1, 0.7);
const pose = { x: -3, z: 0, az: 0 };
const applyPose = () => { robot.position.set(pose.x, 0, pose.z); robot.rotation.y = -pose.az * Math.PI / 180; };
applyPose();
const queue = []; let current = null;
const act = (update) => new Promise((res) => queue.push({ update, res }));
const rest = () => { armL.rotation.z = armR.rotation.z = legL.rotation.z = legR.rotation.z = 0; };
const angDiff = (a, b) => ((b - a + 540) % 360) - 180;
function walkTo(x, z) { let t = 0; return act((dt) => {
  const dx = x - pose.x, dz = z - pose.z, dist = Math.hypot(dx, dz), step = 2.5 * dt; t += dt;
  if (dist > 0.05) pose.az = Math.atan2(dz, dx) * 180 / Math.PI;
  if (dist <= step) { pose.x = x; pose.z = z; rest(); applyPose(); return true; }
  pose.x += (dx / dist) * step; pose.z += (dz / dist) * step;
  legL.rotation.z = Math.sin(t * 9) * 0.6; legR.rotation.z = -legL.rotation.z; armL.rotation.z = legR.rotation.z * 0.7; armR.rotation.z = legL.rotation.z * 0.7;
  applyPose(); return false; }); }
function turnTo(az) { return act((dt) => {
  const d = angDiff(pose.az, az), step = 240 * dt;
  if (Math.abs(d) <= step) { pose.az = az; applyPose(); return true; }
  pose.az += Math.sign(d) * step; applyPose(); return false; }); }
function wave() { let t = 0; return act((dt) => { t += dt; armR.rotation.z = 2.7 + Math.sin(t * 12) * 0.4;
  if (t > 2) { rest(); return true; } return false; }); }
function robotKick(spec) {
  doReset();
  const az = (spec.azimuth || 0) * Math.PI / 180;
  walkTo(-0.7 * Math.cos(az), -0.7 * Math.sin(az)); turnTo(spec.azimuth || 0);
  let t = 0, launched = false;
  return act((dt) => { t += dt;
    legR.rotation.z = t < 0.2 ? -0.9 * (t / 0.2) : -0.9 + 2.4 * Math.min(1, (t - 0.2) / 0.15); armL.rotation.z = 0.6;
    if (t >= 0.33 && !launched) { launched = true; doKick(spec); }
    if (t > 0.8) { rest(); return true; } return false; });
}

// --- controls
const ids = ['speed', 'angle', 'azimuth', 'backspin', 'sidespin', 'g'];
const val = (id) => parseFloat(document.getElementById(id).value);
const sync = () => ids.forEach((id) => (document.getElementById('v_' + id).textContent = val(id)));
ids.forEach((id) => document.getElementById(id).addEventListener('input', sync)); sync();
const dragBox = document.getElementById('drag');

function setSliders(o) { for (const k of ids) if (o[k] !== undefined) document.getElementById(k).value = o[k]; sync(); }
function doKick(o = {}) {
  setSliders(o);
  if (o.drag !== undefined) dragBox.checked = o.drag;
  P.g = val('g'); P.drag = dragBox.checked;
  lastKick = { speed: val('speed'), angle: val('angle'), azimuth: val('azimuth'), backspin: val('backspin'), sidespin: val('sidespin') };
  kick(ball, P, lastKick);
  trailPts.length = 0; reported = false;
}
function doReset() { Object.assign(ball, makeBall(P)); trailPts.length = 0; lastKick = null; reported = true; }
document.getElementById('kick').onclick = () => doKick();
document.getElementById('reset').onclick = doReset;
doReset();

// --- websocket to the agent server
const ws = new WebSocket(`ws://${location.host}/ws`);
const send = (o) => ws.readyState === 1 && ws.send(JSON.stringify(o));
const log = document.getElementById('log');
const say = (who, t) => { const p = document.createElement('p'); p.innerHTML = `<b>${who}:</b> `; p.append(t); log.append(p); log.scrollTop = 1e9; };
document.getElementById('f').onsubmit = (e) => { e.preventDefault(); const i = document.getElementById('msg'); if (!i.value) return; say('You', i.value); send({ type: 'chat', text: i.value }); i.value = ''; };

const snapshot = () => ({ time_s: +ball.t.toFixed(3), position_m: ball.p.map((n) => +n.toFixed(2)), velocity_mps: ball.v.map((n) => +n.toFixed(2)), speed_mps: +Math.hypot(...ball.v).toFixed(2), height_m: +(ball.p[1] - P.r).toFixed(2), stopped: ball.stopped, settings: { gravity: P.g, air_resistance: P.drag, bounciness: P.e, magnus_spin_effect: P.magnus }, last_kick: lastKick, robot: { x: +pose.x.toFixed(2), z: +pose.z.toFixed(2), facing_deg: Math.round(pose.az) } });

ws.onmessage = (ev) => {
  const m = JSON.parse(ev.data);
  if (m.type === 'say') { say('Robot', m.text); if (m.speak) { speechSynthesis.cancel(); speechSynthesis.speak(new SpeechSynthesisUtterance(m.text)); } }
  else if (m.type === 'tool') {
    const { id, name, input } = m;
    const done = () => send({ type: 'result', id, result: snapshot() });
    if (name === 'kick') { pending = id; robotKick(input); }
    else if (name === 'walk_to') walkTo(input.x, input.z).then(done);
    else if (name === 'turn_to') turnTo(input.degrees).then(done);
    else if (name === 'wave') wave().then(done);
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

// --- main loop: fixed physics step, rendered at display rate
const DT = 1 / 480; let acc = 0, prev = performance.now();
const hud = document.getElementById('hud');
function frame(now) {
  const real = Math.min((now - prev) / 1000, 0.1); acc += real; prev = now;
  if (!current && queue.length) current = queue.shift();
  if (current && current.update(real)) { current.res(); current = null; }
  while (acc >= DT) { step(ball, P, DT); acc -= DT; }
  mesh.position.set(...ball.p);
  mesh.rotation.x += ball.w[0] / 60; mesh.rotation.y += ball.w[1] / 60; mesh.rotation.z += ball.w[2] / 60;
  if (!ball.stopped && lastKick && (trailPts.length === 0 || mesh.position.distanceTo(trailPts.at(-1)) > 0.2)) {
    trailPts.push(mesh.position.clone()); trail.geometry.setFromPoints(trailPts);
  }
  const L = ball.landing;
  hud.textContent = `t = ${ball.t.toFixed(2)} s\nheight = ${(ball.p[1] - P.r).toFixed(2)} m\nspeed = ${Math.hypot(...ball.v).toFixed(2)} m/s\n` +
    `vx,vy,vz = ${ball.v.map((n) => n.toFixed(1)).join(', ')}\n` +
    (L ? `\nLANDED: range ${L.range.toFixed(1)} m, hang ${L.time.toFixed(2)} s, peak ${L.maxHeight.toFixed(1)} m` : '');
  if (L && !reported && lastKick) {
    reported = true;
    const vac = vacuumPrediction(lastKick.speed, lastKick.angle, P.g);
    const result = { ...snapshot(), landing: { range_m: +L.range.toFixed(2), sideways_m: +L.z.toFixed(2), hang_time_s: +L.time.toFixed(2), peak_height_m: +L.maxHeight.toFixed(2), impact_speed_mps: +L.impactSpeed.toFixed(2) }, ideal_no_air_prediction: { range_m: +vac.range.toFixed(2), peak_height_m: +vac.maxHeight.toFixed(2), hang_time_s: +vac.time.toFixed(2) } };
    if (pending) { send({ type: 'result', id: pending, result }); pending = null; }
  }
  controls.update(); renderer.render(scene, cam); requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
addEventListener('resize', () => { renderer.setSize(innerWidth, innerHeight); cam.aspect = innerWidth / innerHeight; cam.updateProjectionMatrix(); });
