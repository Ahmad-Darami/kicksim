// Shared by browser and Node tests. SI units, y is up, +x is "forward" at azimuth 0.
export const DEFAULTS = { g: 9.81, m: 0.43, r: 0.11, cd: 0.25, rho: 1.2, e: 0.7, mu: 0.4, drag: true, magnus: true };

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const rad = (d) => (d * Math.PI) / 180;

export function makeBall(P = DEFAULTS) {
  return { p: [0, P.r, 0], v: [0, 0, 0], w: [0, 0, 0], t: 0, maxH: P.r, landing: null, stopped: false };
}

// speed m/s, elevation & azimuth degrees, backspin/sidespin rev/s
export function kick(ball, P, { speed, angle, azimuth = 0, backspin = 0, sidespin = 0 }) {
  const el = rad(angle), az = rad(azimuth);
  const h = [Math.cos(az), 0, Math.sin(az)];
  ball.p = [0, P.r, 0];
  ball.v = [speed * Math.cos(el) * h[0], speed * Math.sin(el), speed * Math.cos(el) * h[2]];
  const lat = [-h[2], 0, h[0]];
  ball.w = add(mul(lat, backspin * 2 * Math.PI), [0, sidespin * 2 * Math.PI, 0]);
  ball.t = 0; ball.maxH = P.r; ball.landing = null; ball.stopped = false;
}

export function step(b, P, dt) {
  if (b.stopped) return;
  const A = Math.PI * P.r * P.r;
  let a = [0, -P.g, 0];
  const sp = len(b.v);
  if (P.drag && sp > 1e-6) a = add(a, mul(b.v, (-0.5 * P.rho * P.cd * A * sp) / P.m));
  if (P.magnus && sp > 1e-6 && b.p[1] > P.r + 1e-3) {
    const wv = cross(b.w, b.v), n = len(wv);
    if (n > 1e-9) {
      const cl = Math.min(0.4, (P.r * len(b.w)) / sp * 0.5);
      a = add(a, mul(wv, (0.5 * P.rho * A * cl * sp * sp) / n / P.m));
    }
  }
  b.v = add(b.v, mul(a, dt));
  b.p = add(b.p, mul(b.v, dt));
  b.t += dt;
  if (b.p[1] > b.maxH) b.maxH = b.p[1];

  if (b.p[1] <= P.r) {
    b.p[1] = P.r;
    const vn = b.v[1];
    if (vn < 0) {
      if (!b.landing) b.landing = { time: b.t, x: b.p[0], z: b.p[2], range: Math.hypot(b.p[0], b.p[2]), maxHeight: b.maxH - P.r, impactSpeed: len(b.v) };
      const jn = vn < -0.6 ? -(1 + P.e) * vn : -vn; // slow impacts just rest
      b.v[1] = vn + jn;
      // friction at the contact point (rc = -r*y): couples spin and sliding
      const rc = [0, -P.r, 0];
      const s = add([b.v[0], 0, b.v[2]], cross(b.w, rc));
      const sl = len(s);
      if (sl > 1e-9) {
        const I = 0.4 * P.m * P.r * P.r;
        const full = sl * P.m / (1 + P.m * P.r * P.r / I); // impulse that cancels slip
        const jt = Math.min(full, P.mu * P.m * jn);
        const J = mul(s, -jt / sl);
        b.v = add(b.v, mul(J, 1 / P.m));
        b.w = add(b.w, mul(cross(rc, J), 1 / I));
      }
    }
    if (Math.abs(b.v[1]) < 0.05) {
      b.v[1] = 0;
      const hs = Math.hypot(b.v[0], b.v[2]);
      if (hs > 0) { const d = Math.min(hs, 0.06 * P.g * dt) / hs; b.v[0] *= 1 - d; b.v[2] *= 1 - d; }
      if (hs < 0.02) { b.v = [0, 0, 0]; b.w = [0, 0, 0]; b.stopped = true; }
    }
  }
}

// What a perfect no-air-resistance parabola predicts (flat ground, launch from ground)
export function vacuumPrediction(speed, angleDeg, g) {
  const el = rad(angleDeg), vy = speed * Math.sin(el);
  return { range: (speed * speed * Math.sin(2 * el)) / g, maxHeight: (vy * vy) / (2 * g), time: (2 * vy) / g };
}
