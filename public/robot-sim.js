// Rigid-body robot on flat ground: mass, momentum, traction-limited acceleration,
// torque-limited turning, a gait driven by distance walked, and a motorised kicking leg
// whose swing speed comes from motor torque / leg inertia (so kick speed is not a slider).
import { launch, spinVector } from './physics.js';

export const RB = {
  mass: 35, vmax: 2.5, amax: 3, alphaMax: 6, wmax: 3,        // walk speed m/s, accel m/s², turn accel rad/s²
  mu: 0.8, g: 9.81,                                          // foot traction
  L: 0.8, hipH: 0.8, legI: 0.8, tauMax: 250, footR: 0.07,    // leg length m, inertia kg·m², motor N·m
  ell: 0.1, bodyR: 0.22, bodyH: 1.5, eFoot: 0.5, eBody: 0.4,
};
const clamp = (x, l, h) => Math.max(l, Math.min(h, x));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const rad = (d) => (d * Math.PI) / 180;

export function makeRobot(x = -4, z = 0) {
  return { x, z, vx: 0, vz: 0, yaw: 0, w: 0, cycle: 0, amp: 0, ax: 0, legTh: 0, legW: 0, kickLeg: false, waveT: -1,
           task: null, queue: [], cmd: { vdes: 0, wdes: 0 }, onLaunch: null };
}
// Queue a task; the promise resolves with its result when it finishes.
export function run(R, task) { return new Promise((resolve) => R.queue.push({ ...task, resolve, t: 0, phase: 'start' })); }

const speedAlong = (R) => R.vx * Math.cos(R.yaw) + R.vz * Math.sin(R.yaw);
function drive(R, tx, tz) {
  const dx = tx - R.x, dz = tz - R.z, dist = Math.hypot(dx, dz);
  if (dist > 0.1) {
    const err = wrap(Math.atan2(dz, dx) - R.yaw);
    // close in, keep the heading steady (the bearing to the target flips wildly up close)
    R.cmd.wdes = dist > 0.3 || Math.abs(err) > 0.35 ? clamp(4 * err, -RB.wmax, RB.wmax) : 0;
    R.cmd.vdes = Math.abs(err) > 0.35 ? 0 : Math.min(RB.vmax, Math.max(0.25, Math.sqrt(2 * 0.4 * RB.amax * dist)));
    return false;
  }
  return Math.hypot(R.vx, R.vz) < 0.1;
}
function face(R, yaw) {
  const err = wrap(yaw - R.yaw);
  R.cmd.wdes = clamp(4 * err, -RB.wmax, RB.wmax);
  return Math.abs(err) < 0.02 && Math.abs(R.w) < 0.05;
}
const legPD = (R, target, dt) => { R.legW += ((300 * (target - R.legTh) - 40 * R.legW) / RB.legI) * dt; R.legTh += R.legW * dt; };
// horizontal distance from hip to ball centre at the moment the foot passes through it
export const reach = (r) => Math.sqrt(RB.L * RB.L - (RB.hipH - r) ** 2);

export function robotStep(R, ball, P, dt) {
  R.cmd = { vdes: 0, wdes: 0 };
  if (!R.task && R.queue.length) R.task = R.queue.shift();
  const tk = R.task; let done = false, result = {};
  if (tk) {
    tk.t += dt;
    if (tk.type === 'walk') done = drive(R, tk.x, tk.z);
    else if (tk.type === 'turn') done = face(R, rad(tk.degrees));
    else if (tk.type === 'wave') { R.waveT = tk.t; done = tk.t > 2; if (done) R.waveT = -1; }
    else if (tk.type === 'kick') {
      const az = rad(tk.azimuth), ballStill = Math.hypot(...ball.v) < 0.05;
      const bx = ball.p[0], bz = ball.p[2];
      if (tk.phase === 'start') tk.phase = 'wait';
      if (tk.phase === 'wait' && ballStill) tk.phase = 'walk';
      if (tk.phase === 'walk') {
        const d = reach(P.r);
        if (drive(R, bx - d * Math.cos(az) + RB.ell * Math.sin(az), bz - d * Math.sin(az) - RB.ell * Math.cos(az))) tk.phase = 'align';
      } else if (tk.phase === 'align') {
        if (face(R, az)) { tk.phase = 'windup'; R.kickLeg = true; R.legTh = 0; R.legW = 0; }
      } else if (tk.phase === 'windup') {
        legPD(R, -0.9, dt);
        if (Math.abs(R.legTh + 0.9) < 0.03 && Math.abs(R.legW) < 0.2) { tk.phase = 'swing'; tk.ts = 0; }
      } else if (tk.phase === 'swing') {
        tk.ts += dt;
        R.legW += ((tk.power * RB.tauMax) / RB.legI - 2 * R.legW) * dt;
        R.legTh += R.legW * dt;
        const h = [Math.cos(R.yaw), Math.sin(R.yaw)], lat = [-h[1], h[0]];
        const fx = R.x + RB.ell * lat[0] + RB.L * Math.sin(R.legTh) * h[0], fz = R.z + RB.ell * lat[1] + RB.L * Math.sin(R.legTh) * h[1];
        const fy = RB.hipH - RB.L * Math.cos(R.legTh);
        if (!tk.hit && Math.hypot(fx - ball.p[0], fy - ball.p[1], fz - ball.p[2]) < P.r + RB.footR) {
          const lo = rad(tk.loft), n = [Math.cos(lo) * h[0], Math.sin(lo), Math.cos(lo) * h[1]];
          const tan = [Math.cos(R.legTh) * h[0], Math.sin(R.legTh), Math.cos(R.legTh) * h[1]];
          const vf = R.legW * RB.L, vn = vf * (tan[0] * n[0] + tan[1] * n[1] + tan[2] * n[2]);
          if (vn > 0) {
            const meff = RB.legI / (RB.L * RB.L), u = ((1 + RB.eFoot) * meff / (meff + P.m)) * vn;
            R.legW -= (P.m * u * RB.L * (tan[0] * n[0] + tan[1] * n[1] + tan[2] * n[2])) / RB.legI;
            launch(ball, [u * n[0], u * n[1], u * n[2]], spinVector(tk.azimuth, tk.backspin, tk.sidespin));
            tk.hit = true;
            R.onLaunch?.({ speed: u, angle_deg: tk.loft, azimuth_deg: tk.azimuth, power: tk.power, foot_speed_mps: vf });
          }
        }
        if (R.legTh > 1.4 || tk.ts > 0.8) tk.phase = 'recover';
      } else if (tk.phase === 'recover') {
        legPD(R, 0, dt);
        if (Math.abs(R.legTh) < 0.05 && Math.abs(R.legW) < 0.3) { R.kickLeg = false; done = true; result = { hit: !!tk.hit }; }
      }
    }
  }

  // --- rigid-body dynamics ---
  const hx = Math.cos(R.yaw), hz = Math.sin(R.yaw);
  const alpha = clamp((R.cmd.wdes - R.w) / 0.1, -RB.alphaMax, RB.alphaMax);
  R.w += alpha * dt; R.yaw += R.w * dt;
  const s = R.vx * hx + R.vz * hz, vl = -R.vx * hz + R.vz * hx;
  R.ax = clamp((R.cmd.vdes - s) / 0.25, -RB.amax, RB.amax);
  const al = clamp(-vl / 0.05, -RB.mu * RB.g, RB.mu * RB.g);
  R.vx += (R.ax * hx - al * hz) * dt; R.vz += (R.ax * hz + al * hx) * dt;
  R.x += R.vx * dt; R.z += R.vz * dt;

  // gait follows distance walked
  const spd = Math.hypot(R.vx, R.vz);
  R.cycle += (spd * dt) / 1.4;
  R.amp += (0.45 * Math.min(1, spd / 1.5) - R.amp) * Math.min(1, 10 * dt);

  // ball vs torso (inelastic-ish impulse between ball and a 35 kg body)
  if (ball.p[1] < RB.bodyH) {
    const dx = ball.p[0] - R.x, dz = ball.p[2] - R.z, d = Math.hypot(dx, dz), min = RB.bodyR + P.r;
    if (d < min && d > 1e-6) {
      const nx = dx / d, nz = dz / d;
      ball.p[0] = R.x + nx * min; ball.p[2] = R.z + nz * min;
      const vrel = (ball.v[0] - R.vx) * nx + (ball.v[2] - R.vz) * nz;
      if (vrel < 0) {
        const j = (-(1 + RB.eBody) * vrel) / (1 / P.m + 1 / RB.mass);
        ball.v[0] += (j / P.m) * nx; ball.v[2] += (j / P.m) * nz; R.vx -= (j / RB.mass) * nx; R.vz -= (j / RB.mass) * nz;
        ball.stopped = false;
      }
    }
  }
  if (done) { R.task = null; tk.resolve(result); }
}
