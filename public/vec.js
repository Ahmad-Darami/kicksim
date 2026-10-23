// tiny vector / quaternion helpers ({x,y,z,w} quats, [x,y,z] vectors)
export const V = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
};
export const Q = {
  id: () => ({ x: 0, y: 0, z: 0, w: 1 }),
  mul: (a, b) => ({
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  }),
  conj: (a) => ({ x: -a.x, y: -a.y, z: -a.z, w: a.w }),
  axis: (ax, ang) => { const s = Math.sin(ang / 2); return { x: ax[0] * s, y: ax[1] * s, z: ax[2] * s, w: Math.cos(ang / 2) }; },
  rot: (q, v) => { // rotate vector v by q
    const t = [2 * (q.y * v[2] - q.z * v[1]), 2 * (q.z * v[0] - q.x * v[2]), 2 * (q.x * v[1] - q.y * v[0])];
    return [v[0] + q.w * t[0] + (q.y * t[2] - q.z * t[1]), v[1] + q.w * t[1] + (q.z * t[0] - q.x * t[2]), v[2] + q.w * t[2] + (q.x * t[1] - q.y * t[0])];
  },
  slerp: (a, b, t) => {
    let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w, s = 1; if (d < 0) { d = -d; s = -1; }
    if (d > 0.9995) { const r = { x: a.x + (s * b.x - a.x) * t, y: a.y + (s * b.y - a.y) * t, z: a.z + (s * b.z - a.z) * t, w: a.w + (s * b.w - a.w) * t }; const n = Math.hypot(r.x, r.y, r.z, r.w); return { x: r.x / n, y: r.y / n, z: r.z / n, w: r.w / n }; }
    const th = Math.acos(d), sn = Math.sin(th), wa = Math.sin((1 - t) * th) / sn, wb = (s * Math.sin(t * th)) / sn;
    return { x: a.x * wa + b.x * wb, y: a.y * wa + b.y * wb, z: a.z * wa + b.z * wb, w: a.w * wa + b.w * wb };
  },
  // quaternion from the columns (x, y, z) of a rotation matrix
  fromBasis: (x, y, z) => {
    const m00 = x[0], m10 = x[1], m20 = x[2], m01 = y[0], m11 = y[1], m21 = y[2], m02 = z[0], m12 = z[1], m22 = z[2];
    const tr = m00 + m11 + m22; let s;
    if (tr > 0) { s = 0.5 / Math.sqrt(tr + 1); return { w: 0.25 / s, x: (m21 - m12) * s, y: (m02 - m20) * s, z: (m10 - m01) * s }; }
    if (m00 > m11 && m00 > m22) { s = 2 * Math.sqrt(1 + m00 - m11 - m22); return { w: (m21 - m12) / s, x: 0.25 * s, y: (m01 + m10) / s, z: (m02 + m20) / s }; }
    if (m11 > m22) { s = 2 * Math.sqrt(1 + m11 - m00 - m22); return { w: (m02 - m20) / s, x: (m01 + m10) / s, y: 0.25 * s, z: (m12 + m21) / s }; }
    s = 2 * Math.sqrt(1 + m22 - m00 - m11); return { w: (m10 - m01) / s, x: (m02 + m20) / s, y: (m12 + m21) / s, z: 0.25 * s };
  },
  // rotation vector (axis * angle) of the rotation q, shortest way round
  vec: (q) => {
    let { x, y, z, w } = q; if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
    const s = Math.hypot(x, y, z); if (s < 1e-9) return [2 * x, 2 * y, 2 * z];
    const k = (2 * Math.atan2(s, w)) / s; return [x * k, y * k, z * k];
  },
};
export const clamp = (x, l, h) => Math.max(l, Math.min(h, x));
