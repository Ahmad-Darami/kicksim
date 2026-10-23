// Articulated humanoid in Rapier: rigid segments joined by joints, driven by torque-limited PD
// "muscles" computed here, plus balance (ankle strategy), walking (SIMBICON-style) and agent-set poses.
import { V, Q, clamp } from './vec.js';

export const DT = 1 / 480;
export const G = 9.81;
const rad = (d) => (d * Math.PI) / 180;
const deg = (r) => (r * 180) / Math.PI;

// ---------- body plan (metres, kg). x forward, y up, z = robot's right ----------
const HZ = 0.09, HIP_Y = 0.94, KNEE_Y = 0.51, ANKLE_Y = 0.08, SH_Y = 1.44, SH_Z = 0.22, ELB_Y = 1.16;
const BODIES = (s) => ({ // s = +1 right, -1 left
});
function plan() {
  const b = {};
  b.pelvis = { c: [0, 0.98, 0], parts: [{ h: [0.09, 0.075, 0.15], m: 10 }] };
  b.torso = { c: [0, 1.27, 0], parts: [{ h: [0.11, 0.22, 0.17], m: 24 }, { ball: 0.1, o: [0, 0.34, 0], m: 4 }] };
  for (const [side, s] of [['r', 1], ['l', -1]]) {
    b['thigh_' + side] = { c: [0, HIP_Y - 0.215, s * HZ], parts: [{ h: [0.06, 0.215, 0.06], m: 8 }] };
    b['shank_' + side] = { c: [0, KNEE_Y - 0.215, s * HZ], parts: [{ h: [0.05, 0.215, 0.05], m: 4 }] };
    b['foot_' + side] = { c: [0.05, 0.03, s * HZ], parts: [{ h: [0.13, 0.03, 0.05], m: 1.2 }], foot: true };
    b['uarm_' + side] = { c: [0, SH_Y - 0.14, s * SH_Z], parts: [{ h: [0.04, 0.14, 0.04], m: 2 }] };
    b['farm_' + side] = { c: [0, ELB_Y - 0.13, s * SH_Z], parts: [{ h: [0.035, 0.13, 0.035], m: 1.5 }, { ball: 0.06, o: [0, -0.16, 0], m: 0.4 }], hand: true };
  }
  return b;
}
// joints: parent, child, world anchor, type. dofs map agent-facing angle -> (axis, sign) of rotation about local axes
const JOINTS = () => {
  const j = [];
  j.push({ name: 'waist', parent: 'pelvis', child: 'torso', at: [0, 1.05, 0], type: 'sph', kp: 3000, kd: 150, tmax: 500,
    dofs: { pitch: ['z', -1], roll: ['x', 1], yaw: ['y', -1] } });
  for (const [side, s] of [['r', 1], ['l', -1]]) {
    const rollSign = s === 1 ? -1 : 1; // abduction (leg away from midline)
    j.push({ name: 'hip_' + side, parent: 'pelvis', child: 'thigh_' + side, at: [0, HIP_Y, s * HZ], type: 'sph', kp: 6000, kd: 300, tmax: 600,
      dofs: { pitch: ['z', 1], roll: ['x', rollSign], yaw: ['y', -1] } });
    j.push({ name: 'knee_' + side, parent: 'thigh_' + side, child: 'shank_' + side, at: [0, KNEE_Y, s * HZ], type: 'rev', limits: [-2.5, 0.0], kp: 6000, kd: 300, tmax: 600,
      dofs: { flex: ['z', -1] } });
    j.push({ name: 'ankle_' + side, parent: 'shank_' + side, child: 'foot_' + side, at: [0, ANKLE_Y, s * HZ], type: 'sph', kp: 3000, kd: 120, tmax: 200,
      dofs: { pitch: ['z', 1], roll: ['x', 1], yaw: ['y', -1] } });
    j.push({ name: 'shoulder_' + side, parent: 'torso', child: 'uarm_' + side, at: [0, SH_Y, s * SH_Z], type: 'sph', kp: 200, kd: 6, tmax: 150,
      dofs: { pitch: ['z', 1], roll: ['x', rollSign], yaw: ['y', -1] } });
    j.push({ name: 'elbow_' + side, parent: 'uarm_' + side, child: 'farm_' + side, at: [0, ELB_Y, s * SH_Z], type: 'rev', limits: [-0.05, 2.6], kp: 150, kd: 4, tmax: 100,
      dofs: { flex: ['z', 1] } });
  }
  return j;
};
const AXIS = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
const GROUND = 1, ROBOT = 2, BALL = 4;
export const groups = { ground: (0x0001 << 16) | 0xffff, ball: (BALL << 16) | (GROUND | ROBOT | BALL), robot: (ROBOT << 16) | (GROUND | BALL) };

// Every agent-facing joint name, e.g. "right_hip_pitch", "left_knee_flex", "waist_yaw"
export function agentJointNames() {
  const out = [];
  for (const j of JOINTS()) for (const d of Object.keys(j.dofs)) out.push(nameFor(j.name, d));
  return out;
}
function nameFor(jn, dof) {
  const m = jn.match(/^(\w+)_(r|l)$/);
  return (m ? (m[2] === 'r' ? 'right_' : 'left_') + m[1] : jn) + '_' + dof;
}
export const JOINT_LIMITS_DEG = {}; // filled by buildBiped

export function buildBiped(RAPIER, world, x0 = -4, z0 = 0) {
  const P = plan(), bodies = {}, mass = {}, off = [x0, 0, z0];
  let total = 0;
  for (const [name, spec] of Object.entries(P)) {
    const rb = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(spec.c[0] + off[0], spec.c[1], spec.c[2] + off[2]).setCanSleep(false).setAngularDamping(0.02));
    let m = 0;
    for (const part of spec.parts) {
      const cd = part.ball ? RAPIER.ColliderDesc.ball(part.ball) : RAPIER.ColliderDesc.cuboid(...part.h);
      if (part.o) cd.setTranslation(...part.o);
      cd.setMass(part.m).setFriction(spec.foot ? 1.2 : 0.6).setRestitution(0.1).setCollisionGroups(groups.robot);
      world.createCollider(cd, rb); m += part.m;
    }
    bodies[name] = rb; mass[name] = m; total += m;
  }
  const joints = {};
  for (const J of JOINTS()) {
    const pc = P[J.parent].c, cc = P[J.child].c;
    const a1 = { x: J.at[0] - pc[0], y: J.at[1] - pc[1], z: J.at[2] - pc[2] }, a2 = { x: J.at[0] - cc[0], y: J.at[1] - cc[1], z: J.at[2] - cc[2] };
    let data;
    if (J.type === 'sph') data = RAPIER.JointData.generic(a1, a2, { x: 1, y: 0, z: 0 }, RAPIER.JointAxesMask.LinX | RAPIER.JointAxesMask.LinY | RAPIER.JointAxesMask.LinZ);
    else { data = RAPIER.JointData.revolute(a1, a2, { x: 0, y: 0, z: 1 }); data.limitsEnabled = true; data.limits = J.limits; }
    const joint = world.createImpulseJoint(data, bodies[J.parent], bodies[J.child], true);
    const raw = world.impulseJoints.raw;
    if (J.type === 'sph') for (let k = 3; k < 6; k++) { raw.jointConfigureMotorModel(joint.handle, k, 1); raw.jointSetMotorMaxForce(joint.handle, k, J.tmax); }
    else { joint.configureMotorModel(1); joint.setMotorMaxForce(J.tmax); }
    joints[J.name] = { ...J, joint, raw, pb: bodies[J.parent], cb: bodies[J.child], ang: {}, goal: Object.fromEntries(Object.keys(J.dofs).map((d) => [d, 0])) };
    for (const [d, [ax, sg]] of Object.entries(J.dofs)) JOINT_LIMITS_DEG[nameFor(J.name, d)] = J.type === 'rev' ? J.limits.map((l) => deg(l * sg)).sort((x, y) => x - y) : [-120, 120];
  }
  return new Biped(RAPIER, world, bodies, mass, total, joints, off);
}

export class Biped {
  constructor(RAPIER, world, bodies, mass, total, joints, off) {
    Object.assign(this, { RAPIER, world, bodies, mass, total, joints });
    this.mode = 'limp'; this.heading = 0; this.t = 0;
    this.fallen = false; this.fallReason = null; this.motion = null; this.balanceOn = true;
    this.cmd = {}; this.dbg = {}; this.latThr = 0.09; this.kickPlan = null; this.kickDone = false;
    this.setMode('stand');
  }
  // ----- state helpers -----
  s(name) { const b = this.bodies[name]; const t = b.translation(), r = b.rotation(), v = b.linvel(), w = b.angvel(); return { p: [t.x, t.y, t.z], q: r, v: [v.x, v.y, v.z], w: [w.x, w.y, w.z] }; }
  com() {
    let p = [0, 0, 0], v = [0, 0, 0];
    for (const [n, b] of Object.entries(this.bodies)) { const c = b.worldCom(), l = b.linvel(), m = this.mass[n]; p = V.add(p, V.mul([c.x, c.y, c.z], m)); v = V.add(v, V.mul([l.x, l.y, l.z], m)); }
    return { p: V.mul(p, 1 / this.total), v: V.mul(v, 1 / this.total) };
  }
  frame() { const c = Math.cos(this.heading), s = Math.sin(this.heading); return { f: [c, 0, s], r: [-s, 0, c] }; }
  ankle(side) { const sh = this.s('shank_' + side); return V.add(sh.p, Q.rot(sh.q, [0, -0.215, 0])); }
  sole(side) {
    const f = this.s('foot_' + side); let lo = 9;
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) lo = Math.min(lo, V.add(f.p, Q.rot(f.q, [0.13 * sx, -0.03, 0.05 * sz]))[1]);
    return { low: lo, p: f.p };
  }
  setMode(m) { this.mode = m; if (m === 'limp') this.motion = null; }
  rel(J) { return Q.mul(Q.conj(this.s(J.parent).q), this.s(J.child).q); }
  // ----- agent-facing joint access (degrees) -----
  getAngles() {
    const out = {};
    for (const J of Object.values(this.joints)) {
      const v = Q.vec(this.rel(J));
      if (J.type === 'rev') out[nameFor(J.name, 'flex')] = +deg(v[2] * J.dofs.flex[1]).toFixed(1);
      else for (const [d, [ax, sg]] of Object.entries(J.dofs)) out[nameFor(J.name, d)] = +deg(v[{ x: 0, y: 1, z: 2 }[ax]] * sg).toFixed(1);
    }
    return out;
  }
  setGoal(map) { // degrees, by agent joint name
    for (const J of Object.values(this.joints)) for (const d of Object.keys(J.dofs)) { const n = nameFor(J.name, d); if (map[n] !== undefined) J.goal[d] = rad(map[n]); }
  }
  goalQuat(J) {
    if (J.type === 'rev') return Q.axis(AXIS.z, J.goal.flex * J.dofs.flex[1]);
    const e = (d) => { const [ax, sg] = J.dofs[d]; return Q.axis(AXIS[ax], J.goal[d] * sg); };
    return Q.mul(Q.mul(e('pitch'), e('roll')), e('yaw'));
  }
  // ----- motor commands (applied inside Rapier's solver, so they stay stable at high gain) -----
  // Aim joint J at relative rotation `qRel` (child relative to parent), with gain scale k.
  aim(J, qRel, k = 1) { this.cmd[J.name] = { q: qRel, k }; }
  // Command a *torque* about J's parent-frame axes (vector tau): aim at (current + tau / kp)
  push(J, tauParentFrame, k = 0.1) {
    const cur = Q.vec(this.rel(J));
    this.cmd[J.name] = { rv: V.add(cur, V.mul(tauParentFrame, 1 / (J.kp * k))), k };
  }
  hold(J, k = 1) { this.aim(J, this.goalQuat(J), k); }
  flush() {
    for (const J of Object.values(this.joints)) {
      const c = this.cmd[J.name] || { k: 0 };
      if (this.debugErr) { const cur = Q.vec(this.rel(J)); const tg = c.rv || (c.q ? Q.vec(c.q) : [0, 0, 0]); (this.errs ||= {})[J.name] = Math.hypot(cur[0] - tg[0], cur[1] - tg[1], cur[2] - tg[2]) * 57.3; }
      const kp = J.kp * c.k, kd = c.k > 0 ? J.kd * Math.sqrt(c.k) : 2;
      if (J.type === 'rev') {
        const a = c.q ? Q.vec(c.q)[2] : c.rv ? c.rv[2] : 0;
        J.joint.configureMotorPosition(clamp(a, J.limits[0], J.limits[1]), kp, kd);
      } else {
        const rv = c.rv || (c.q ? Q.vec(c.q) : [0, 0, 0]);
        for (let i = 0; i < 3; i++) J.raw.jointConfigureMotorPosition(J.joint.handle, 3 + i, rv[i], kp, kd);
      }
    }
    this.cmd = {};
  }
  // ----- control -----
  update(dt) {
    this.t += dt; this.dtNow = dt;
    const c = this.com(), pel = this.s('pelvis'), tor = this.s('torso');
    this.comState = c;
    this.tilt = Math.acos(clamp(Q.rot(tor.q, [0, 1, 0])[1], -1, 1));
    if (!this.fallen && (pel.p[1] < 0.6 || this.tilt > 1.0 || c.p[1] < 0.55)) { this.fallen = true; this.fallReason = pel.p[1] < 0.6 ? 'hips hit low' : 'tipped over'; this.savedMode = this.mode; this.mode = 'limp'; }
    this.runMotion(dt);
    if (this.mode === 'pose') for (const J of Object.values(this.joints)) this.hold(J);
    else if (this.mode === 'stand') this.stand(dt, c);
    else if (this.mode === 'walk') this.walk(dt, c);
    this.flush();
  }
  yawQ(psi) { return Q.axis([0, 1, 0], -psi); }
  isStanding() { return this.mode === 'stand' || (this.mode === 'walk' && this.wk && this.wk.ph === 'hold'); }
  beginStand() { // hand over from walking: remember where the legs are and ease them to the standing pose
    this.standBlend = { t: 0, from: {} };
    for (const n of ['hip_r', 'hip_l', 'knee_r', 'knee_l']) this.standBlend.from[n] = this.rel(this.joints[n]);
    this.wk = null; this.mode = 'stand'; this.wantStand = false; this.lastStep = this.t;
  }
  upperBody(armSwing = null) {
    const J = this.joints;
    // torso stays upright in the world (plus any lean the agent asked for on the waist joint)
    this.aim(J.waist, Q.mul(Q.conj(this.s('pelvis').q), Q.mul(this.yawQ(this.heading), this.goalQuat(J.waist))));
    for (const [s, sg] of [['r', 1], ['l', -1]]) {
      const rc = this.reach && this.reach[s];
      if (rc) {
        // glide the hand toward the target (a sudden arm swing would knock the robot over)
        const d = V.sub(rc.target, rc.cur), n = V.len(d), step = Math.min(n, (rc.speed ?? 0.6) * this.dtNow);
        if (n > 1e-6) rc.cur = V.add(rc.cur, V.mul(d, step / n));
        const ik = this.armIK(s, rc.cur); this.aim(J['shoulder_' + s], ik.q); this.aim(J['elbow_' + s], Q.axis(AXIS.z, ik.kappa)); continue;
      }
      if (armSwing !== null && !this.armsFree) {
        this.aim(J['shoulder_' + s], Q.axis(AXIS.z, armSwing * sg));
        this.aim(J['elbow_' + s], Q.axis(AXIS.z, 0.25 + 0.25 * Math.abs(armSwing)));
      } else { this.hold(J['shoulder_' + s]); this.hold(J['elbow_' + s]); }
    }
  }
  // two-link arm IK: place the hold point (0.43 m beyond the elbow) at a world position
  armIK(side, T) {
    const s = side === 'r' ? 1 : -1, tor = this.s('torso'), L1 = 0.28, L2 = 0.43;
    const S = V.add(tor.p, Q.rot(tor.q, [0, 0.17, s * 0.22]));
    let d = V.sub(T, S), dist = V.len(d);
    dist = clamp(dist, Math.abs(L1 - L2) + 0.03, L1 + L2 - 0.005);
    const u = V.mul(d, 1 / (V.len(d) || 1));
    const beta = Math.acos(clamp((L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist), -1, 1));
    let w = V.sub([0, -1, 0], V.mul(u, -u[1])); const wl = V.len(w); w = wl > 1e-6 ? V.mul(w, 1 / wl) : [1, 0, 0];
    const a = V.add(V.mul(u, Math.cos(beta)), V.mul(w, Math.sin(beta)));
    const E = V.mul(a, L1), f0 = V.sub(V.mul(u, dist), E), f = V.mul(f0, 1 / (V.len(f0) || 1));
    const kappa = Math.acos(clamp(V.dot(a, f), -1, 1));
    let z = V.cross(a, f); const zl = V.len(z); z = zl > 1e-6 ? V.mul(z, 1 / zl) : [0, 0, 1];
    const y = V.mul(a, -1), x = V.cross(y, z);
    return { q: Q.mul(Q.conj(tor.q), Q.fromBasis(x, y, z)), kappa };
  }
  reachHand(side, target) {
    this.reach = this.reach || {};
    if (!target && this.reach[side]) { // hand the arm back to joint goals without a jump
      const a = this.getAngles(), pre = (side === 'r' ? 'right_' : 'left_');
      this.setGoal({ [pre + 'shoulder_pitch']: a[pre + 'shoulder_pitch'], [pre + 'shoulder_roll']: a[pre + 'shoulder_roll'], [pre + 'shoulder_yaw']: a[pre + 'shoulder_yaw'], [pre + 'elbow_flex']: a[pre + 'elbow_flex'] });
    }
    if (target) {
      const f = this.s('farm_' + side), prev = this.reach[side];
      this.reach[side] = { target, cur: prev ? prev.cur : V.add(f.p, Q.rot(f.q, [0, -0.30, 0])), speed: this.reachSpeed ?? 0.6 };
      this.armsFree = true;
    } else this.reach[side] = null;
  }
  // ----- keyframe motions: [{t, joints:{name: degrees}, event?}] (times in seconds from now) -----
  goalsDeg() {
    const out = {};
    for (const J of Object.values(this.joints)) for (const d of Object.keys(J.dofs)) out[nameFor(J.name, d)] = deg(J.goal[d]);
    return out;
  }
  playMotion(frames, onEvent) {
    const start = this.goalsDeg(), knots = {};
    for (const fr of [...frames].sort((a, b) => a.t - b.t)) for (const [n, v] of Object.entries(fr.joints || {})) (knots[n] ||= [{ t: 0, v: start[n] ?? 0 }]).push({ t: Math.max(fr.t, 0.001), v });
    const events = frames.filter((f) => f.event).map((f) => ({ t: f.t, name: f.event, done: false }));
    const total = Math.max(0, ...frames.map((f) => f.t));
    if (Object.keys(knots).some((n) => /shoulder|elbow/.test(n))) this.armsFree = true;
    this.motion = { knots, events, t: 0, total, onEvent };
  }
  runMotion(dt) {
    const m = this.motion; if (!m) return;
    m.t += dt; const out = {};
    for (const [n, k] of Object.entries(m.knots)) {
      let v = k[k.length - 1].v;
      for (let i = 0; i < k.length - 1; i++) if (m.t < k[i + 1].t) { const u = clamp((m.t - k[i].t) / (k[i + 1].t - k[i].t), 0, 1), e = u * u * (3 - 2 * u); v = k[i].v + (k[i + 1].v - k[i].v) * e; break; }
      out[n] = v;
    }
    this.setGoal(out);
    for (const ev of m.events) if (!ev.done && m.t >= ev.t) { ev.done = true; m.onEvent?.(ev.name); }
    if (m.t >= m.total) this.motion = null;
  }
  stand(dt, c) {
    const { f, r } = this.frame(), J = this.joints;
    if (this.standBlend) { this.standBlend.t += dt; if (this.standBlend.t > 0.8) this.standBlend = null; }
    const aR = this.ankle('r'), aL = this.ankle('l'), mid = V.mul(V.add(aR, aL), 0.5);
    const rel = V.sub(c.p, mid), dcom = V.dot(rel, f), v = V.dot(c.v, f), dl = V.dot(rel, r), vl = V.dot(c.v, r);
    // hip strategy: shift the pelvis over the feet with the hip joints when the ankles alone cannot cope
    let hipPitch = 0, hipRoll = 0;
    if (this.balanceOn) {
      const dEx = dcom - 0.04, over = dEx - clamp(dEx, -0.03, 0.03);
      hipPitch = (this.hipS ?? 1) * clamp(2.0 * over + 0.25 * v, -0.25, 0.25);   // both thighs forward -> pelvis moves back
      hipRoll = (this.hipS ?? 1) * clamp(1.6 * dl + 0.45 * vl, -0.2, 0.2);        // right leg abducts / left adducts -> pelvis moves left
    }
    for (const s of ['r', 'l']) {
      const Jh = J['hip_' + s], rs = Jh.dofs.roll[1], sg = s === 'r' ? 1 : -1;
      let qh = Q.mul(Q.mul(this.goalQuat(Jh), Q.axis(AXIS.z, hipPitch)), Q.axis(AXIS.x, hipRoll * sg * rs)), qk = this.goalQuat(J['knee_' + s]);
      const sb = this.standBlend;
      if (sb) { const u = clamp(sb.t / 0.7, 0, 1), e = u * u * (3 - 2 * u); qh = Q.slerp(sb.from['hip_' + s], qh, e); qk = Q.slerp(sb.from['knee_' + s], qk, e); }
      this.aim(Jh, qh); this.aim(J['knee_' + s], qk);
    }
    this.upperBody();
    const W = this.total * G, d = dcom - 0.04;
    // ankle torque = cancel gravity's tipping torque + spring/damper toward "COM 4 cm ahead of the ankles".
    // A foot can only push where it touches the ground: heel is 8 cm behind the ankle, toe 18 cm ahead.
    const Tp = this.balanceOn ? clamp((W * dcom + 900 * d + 480 * v) / 2, -0.08 * W / 2 * 0.9, 0.18 * W / 2 * 0.9) : 0;
    const Tr = this.balanceOn ? (this.trS ?? 1) * clamp((250 * dl + 120 * vl) / 2, -25, 25) : 0;
    for (const s of ['r', 'l']) {
      const shq = this.s('shank_' + s).q;
      const tauWorld = V.add(V.mul(r, -Tp), V.mul(f, Tr)); // torque on the foot
      this.push(J['ankle_' + s], Q.rot(Q.conj(shq), tauWorld));
    }
    this.dbg = { d, v, dl, vl, Tp, Tr };
    // stepping reflex: if the capture point (COM + v/omega) leaves the region the ankles/hips can handle, take a step
    if (this.balanceOn && this.stepReflex !== false && !this.fallen && this.t - (this.lastStep ?? -9) > 2.0) {
      const om = Math.sqrt(G / 0.93), xi = V.add(c.p, V.mul(c.v, 1 / om)), rx = V.sub(xi, mid);
      const df = V.dot(rx, f) - 0.04, dlat = V.dot(rx, r);
      if (df > 0.12 || df < -0.09 || Math.abs(dlat) > 0.1) {
        this.startWalk(dlat >= 0 ? 'r' : 'l', true); this.speedCmd = 0; this.turnCmd = 0; this.wantStand = true; this.lastStep = this.t;
      }
    }
  }

  // ================= walking: capture-point foot placement + leg IK =================
  // The centre of mass is modelled as a linear inverted pendulum (LIPM). Each step, the swing foot is placed at the
  // "divergent component of motion" (COM + vel/omega) minus a desired offset, using the *measured* COM, so pushes and
  // errors change where the next foot goes. Joint angles come from analytic leg IK; the motors do the rest.
  legIK(side, qPel, pelPos, ankleW, footYaw) {
    const s = side === 'r' ? 1 : -1, A = 0.43, B = 0.43;
    const hip = V.add(pelPos, Q.rot(qPel, [0, -0.04, s * HZ]));
    let d = V.sub(ankleW, hip), l = V.len(d);
    l = Math.min(l, A + B - 1e-3); d = V.mul(d, 1 / V.len(d));
    const kappa = Math.PI - Math.acos(clamp((A * A + B * B - l * l) / (2 * A * B), -1, 1));
    const beta = Math.acos(clamp((A * A + l * l - B * B) / (2 * A * l), -1, 1));
    const f = [Math.cos(footYaw), 0, Math.sin(footYaw)];
    let w = V.sub(f, V.mul(d, V.dot(f, d))); w = V.mul(w, 1 / V.len(w));
    const t = V.add(V.mul(d, Math.cos(beta)), V.mul(w, Math.sin(beta)));
    const yT = V.mul(t, -1);
    let xT = V.sub(f, V.mul(yT, V.dot(f, yT))); xT = V.mul(xT, 1 / V.len(xT));
    const zT = V.cross(xT, yT);
    const qThigh = Q.fromBasis(xT, yT, zT);
    const qShank = Q.mul(qThigh, Q.axis(AXIS.z, -kappa));
    return { qThigh, qShank, kappa };
  }
  startWalk(first = 'r', quick = false) {
    const c = this.com(), pelvis = this.s('pelvis');
    const w = this.wk = { ph: 'shift', t: 0, swing: first, n: 0, hc: 0.93, plant: {}, ref: { p: [c.p[0], c.p[2]], v: [c.v[0], c.v[2]] }, off: [c.p[0] - pelvis.p[0], c.p[2] - pelvis.p[2]], vcmd: 0, p0: null, p1: null, armPhase: 0, quick };
    for (const s of ['r', 'l']) { const a = this.ankle(s); w.plant[s] = { p: [a[0], a[2]], yaw: this.heading }; }
    this.mode = 'walk';
  }
  walk(dt, c) {
    if (!this.wk) this.startWalk();
    const w = this.wk, T_SS = this.tss ?? 0.32, T_DS = this.tds ?? 0.08, T_STEP = T_SS + T_DS, STEP_W = 0.19, ZOFF = 0.03;
    const om = Math.sqrt(G / w.hc), { f, r } = this.frame();
    const f2 = [f[0], f[2]], r2 = [r[0], r[2]];
    const cp = [c.p[0], c.p[2]], cv = [c.v[0], c.v[2]];
    const dot2 = (a, b) => a[0] * b[0] + a[1] * b[1], add2 = (a, b) => [a[0] + b[0], a[1] + b[1]], sub2 = (a, b) => [a[0] - b[0], a[1] - b[1]], mul2 = (a, k) => [a[0] * k, a[1] * k];
    // heading & speed commands (rate limited)
    this.heading += clamp(this.turnCmd || 0, -0.6, 0.6) * dt;
    w.vcmd += clamp((this.speedCmd || 0) - w.vcmd, -(this.decCmd ?? 0.6) * dt, (this.accCmd ?? 0.3) * dt);
    w.t += dt;
    const sw = w.swing, st = sw === 'r' ? 'l' : 'r', sgnR = sw === 'r' ? 1 : -1;
    const foot = (s) => w.plant[s];
    const zmpOf = (s) => add2(foot(s).p, mul2([Math.cos(foot(s).yaw), Math.sin(foot(s).yaw)], ZOFF));
    const xi = add2(cp, mul2(cv, 1 / om));
    // periodic-orbit DCM offsets (relative to the stance ZMP point) at the start of a single-support phase,
    // per axis, accounting for the double-support ZMP ramp.  u = foot step along the axis, sig = +1 (same offset next step) or -1 (flips)
    const orbitAxis = (u, sig) => {
      const E = Math.exp(om * T_SS);
      const g = (b) => { let q = b * E; const n = 20; for (let i = 0; i < n; i++) { const z = ((i + 0.5) / n) * u; q += om * (q - z) * (T_DS / n); } return q - u; };
      const g0 = g(0), m = g(1) - g0;
      return g0 / (sig - m);
    };
    const startSS = (swNew) => { // re-anchor the reference DCM on the periodic orbit relative to the (new) stance foot
      const stNew = swNew === 'r' ? 'l' : 'r', sg = swNew === 'r' ? 1 : -1;
      const L = clamp(w.vcmd, -0.6, 1.2) * T_STEP;
      const bx = orbitAxis(L, 1), by = orbitAxis(STEP_W, -1);
      const zs = zmpOf(stNew);
      const xiDes = add2(zs, add2(mul2(f2, bx), mul2(r2, sg * by)));
      w.ref.p = add2(cp, mul2(sub2(w.ref.p, cp), 0.5)); // stay close to where the body really is
      w.ref.v = mul2(sub2(xiDes, w.ref.p), om);
      w.ierr = [0, 0]; w.orbit = { bx, by };
    };
    let zmp;
    if (w.ph === 'hold') {
      // both feet planted, posture held by IK; start moving when asked, or take a recovery step if the capture point strays
      const mid = mul2(add2(foot('r').p, foot('l').p), 0.5);
      const rx = sub2(xi, mid), df = dot2(rx, f2) - 0.03, dlat = dot2(rx, r2);
      const settled = Math.abs(df) < 0.035 && Math.abs(dlat) < 0.04;
      if (this.kickPlan && settled) {
        const K = this.kickPlan; this.kickPlan = null;
        const pw = clamp(K.power, 0.05, 1), Ts = 0.22 - 0.12 * pw;
        w.kick = { ...K, Tb: 0.3, Ts, Tf: 0.25, T: 0.3 + Ts + 0.25, yc: 0.075 + 0.1 * Math.pow(clamp((45 - K.loft) / 45, 0, 1), 1.6) };
        w.ph = 'shift'; w.t = 0; w.quick = false; w.swing = K.swing; w.ref = { p: [...cp], v: [...cv] }; w.ierr = [0, 0]; w.n = 0;
        return this.walk(dt, c);
      }
      if ((!this.wantStand && settled) || (this.stepReflex !== false && (df > 0.12 || df < -0.09 || Math.abs(dlat) > this.latThr))) {
        w.ph = 'shift'; w.t = 0; w.quick = this.wantStand !== false ? true : false;
        w.swing = this.wantStand ? (dlat >= 0 ? 'r' : 'l') : (w.lastSwing === 'r' ? 'l' : 'r');
        w.ref = { p: [...cp], v: [...cv] }; w.ierr = [0, 0]; w.n = 0;
        return this.walk(dt, c);
      }
      // a short walk-style quick step is not needed: hold
    }
    const ankleY = 0.08;
    let tgt = { r: null, l: null }, holdRef = null;
    if (w.ph === 'hold') {
      const mid = mul2(add2(foot('r').p, foot('l').p), 0.5);
      zmp = add2(mid, mul2(f2, ZOFF)); holdRef = add2(mid, mul2(f2, 0.03));
      tgt.r = { p: foot('r').p, y: ankleY, yaw: foot('r').yaw }; tgt.l = { p: foot('l').p, y: ankleY, yaw: foot('l').yaw };
    }
    if (w.ph === 'shift') {
      // shift weight sideways only: ZMP at the swing foot laterally, but level with the COM forward (no forward push)
      const zs = zmpOf(sw); zmp = add2(zs, mul2(f2, dot2(sub2(cp, zs), f2)));
      const b = dot2(sub2(xi, foot(st).p), r2) * sgnR; // DCM offset toward the swing side
      if (w.quick || (b <= (w.kick ? 0.02 : 0.04) && w.t > (w.kick ? 0.25 : 0.15)) || w.t > 1.2) { w.ph = 'ss'; w.t = 0; w.p0 = [...foot(sw).p]; w.p1 = [...foot(sw).p]; w.ref = { p: [...cp], v: [...cv] }; if (!w.kick) startSS(sw); }
    }
    if (w.ph === 'ss' && w.kick) {
      // ---- kick: balance on the stance foot (COM held just inside it) while the other leg winds back, strikes and follows through
      const K = w.kick, fk = f2, pc = sub2(K.ball, mul2(fk, 0.29)), B = sub2(pc, mul2(fk, 0.55));
      const pLand = add2(foot(st).p, add2(mul2(f2, 0.2), mul2(r2, sgnR * 0.19)));
      zmp = zmpOf(st);
      const t = w.t, mj = (u) => u * u * u * (10 - 15 * u + 6 * u * u);
      let pos, y;
      if (t < K.Tb) { const s = mj(t / K.Tb); pos = add2(w.p0, mul2(sub2(B, w.p0), s)); y = 0.08 + 0.12 * s; }
      else if (t < K.Tb + K.Ts) { const u = (t - K.Tb) / K.Ts; pos = add2(B, mul2(sub2(pc, B), u * u)); y = 0.2 + (K.yc - 0.2) * u; }
      else {
        const u = clamp((t - K.Tb - K.Ts) / K.Tf, 0, 1), vc = (2 * Math.hypot(...sub2(pc, B))) / K.Ts, h00 = 2 * u ** 3 - 3 * u ** 2 + 1, h10 = u ** 3 - 2 * u ** 2 + u, h01 = -2 * u ** 3 + 3 * u ** 2;
        pos = add2(add2(mul2(pc, h00), mul2(mul2(fk, vc * K.Tf), h10)), mul2(pLand, h01));
        y = K.yc + (0.08 - K.yc) * (u * u * (3 - 2 * u)) + 0.03 * Math.sin(Math.PI * u);
      }
      w.p1 = pLand;
      tgt[sw] = { p: pos, y, yaw: this.heading }; tgt[st] = { p: foot(st).p, y: ankleY, yaw: foot(st).yaw };
      { // keep the (reference) DCM parked just inside the stance foot: ZMP = DCM + k (DCM - target)
        const xiTarget = add2(zmpOf(st), mul2(r2, sgnR * 0.01)), xiR = add2(w.ref.p, mul2(w.ref.v, 1 / om));
        zmp = add2(xiR, mul2(sub2(xiR, xiTarget), 2.0)); }
      if (t >= K.T) { w.kick = null; w.ph = 'ds'; w.t = 0; w.plant[sw] = { p: [...pLand], yaw: this.heading }; this.kickDone = true; }
    } else if (w.ph === 'ss') {
      zmp = zmpOf(st);
      const tau = clamp(w.t / T_SS, 0, 1);
      // deterministic step: fixed width, length = speed * step time
      const L = clamp(w.vcmd, -0.6, 1.2) * T_STEP;
      // nominal landing spot, nudged by the DCM tracking error (measured vs reference) amplified over the time left: capture-step feedback
      const xiRef = add2(w.ref.p, mul2(w.ref.v, 1 / om)), dErr = sub2(xi, xiRef);
      const gain = (this.kstep ?? 0.8) * Math.exp(om * Math.max(T_SS - w.t, 0));
      const fAdj = clamp(dot2(dErr, f2) * gain, -0.45, 0.45), lAdj = clamp(dot2(dErr, r2) * gain, -0.12, 0.12);
      w.adj = add2(mul2(w.adj || [0, 0], 0.9), mul2([fAdj, lAdj], 0.1));
      const lat = clamp(STEP_W + sgnR * w.adj[1], 0.13, 0.32);
      const p1 = add2(foot(st).p, add2(mul2(f2, L + w.adj[0]), mul2(r2, sgnR * lat)));
      w.p1 = p1;
      const s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
      const pos = add2(w.p0, mul2(sub2(w.p1, w.p0), s));
      tgt[sw] = { p: pos, y: ankleY + 0.07 * Math.sin(Math.PI * tau) ** 2, yaw: this.heading };
      tgt[st] = { p: foot(st).p, y: ankleY, yaw: foot(st).yaw };
      w.armPhase = (sgnR) * (1 - 2 * tau);
      if (w.t >= T_SS) { w.ph = 'ds'; w.t = 0; w.plant[sw] = { p: [...w.p1], yaw: this.heading }; }
    } else if (w.ph === 'ds') {
      const k = clamp(w.t / T_DS, 0, 1);
      zmp = add2(mul2(zmpOf(st), 1 - k), mul2(zmpOf(sw), k));
      tgt[sw] = { p: foot(sw).p, y: ankleY, yaw: foot(sw).yaw }; tgt[st] = { p: foot(st).p, y: ankleY, yaw: foot(st).yaw };
      // asked to stop: once nearly at rest with both feet down, hand over to the standing balance controller
      if (w.t >= T_DS && this.wantStand && Math.abs(w.vcmd) < 0.02 && Math.abs(dot2(sub2(xi, mul2(add2(foot('r').p, foot('l').p), 0.5)), r2)) < 0.055 && Math.abs(dot2(sub2(xi, mul2(add2(foot('r').p, foot('l').p), 0.5)), f2) - 0.03) < 0.06) {
        w.ph = 'hold'; w.t = 0; w.quick = false; w.holdFrom = [...cp]; this.wantStand = true; this.lastStep = this.t; return;
      }
      if (w.t >= T_DS) { w.lastSwing = sw; w.ph = 'ss'; w.t = 0; w.swing = st; w.n++; w.p0 = [...foot(st).p]; w.p1 = [...foot(st).p]; startSS(st); }
    } else if (w.ph === 'shift') {
      tgt.r = { p: foot('r').p, y: ankleY, yaw: foot('r').yaw }; tgt.l = { p: foot('l').p, y: ankleY, yaw: foot('l').yaw };
    }
    if (w.ph === 'shift') { tgt.r = { p: foot('r').p, y: ankleY, yaw: foot('r').yaw }; tgt.l = { p: foot('l').p, y: ankleY, yaw: foot('l').yaw }; }
    // reference COM follows LIPM about the ZMP, gently pulled toward the measurement
    if (holdRef) { const u = clamp(w.t / 0.8, 0, 1), e = u * u * (3 - 2 * u), hf = w.holdFrom || holdRef; w.ref = { p: add2(hf, mul2(sub2(holdRef, hf), e)), v: [0, 0] }; }
    else {
    const acc = mul2(sub2(w.ref.p, zmp), om * om);
    w.ref.v = add2(w.ref.v, mul2(acc, dt)); w.ref.p = add2(w.ref.p, mul2(w.ref.v, dt));
    }
    const KFB = this.kfb ?? 0; w.ref.p = add2(w.ref.p, mul2(sub2(cp, w.ref.p), KFB * dt)); w.ref.v = add2(w.ref.v, mul2(sub2(cv, w.ref.v), KFB * dt));
    // pelvis reference: ref COM minus the (measured) COM-to-pelvis offset
    const pel = this.s('pelvis');
    w.off = add2(mul2(w.off, 0.98), mul2([c.p[0] - pel.p[0], c.p[2] - pel.p[2]], 0.02));
    // close the loop on the COM: shift the pelvis target by (P + I) of the tracking error so leg sag / compliance is cancelled
    const err = sub2(w.ref.p, cp);
    w.ierr = add2(w.ierr || [0, 0], mul2(err, dt)); w.ierr = [clamp(w.ierr[0], -0.08, 0.08), clamp(w.ierr[1], -0.08, 0.08)];
    const KP = this.kcp ?? 2, KI = this.kci ?? 4;
    let corr = add2(mul2(err, KP), mul2(w.ierr, KI)); const cn = Math.hypot(corr[0], corr[1]); if (cn > 0.1) corr = mul2(corr, 0.1 / cn);
    const pelPos = [w.ref.p[0] - w.off[0] + corr[0], 0.955, w.ref.p[1] - w.off[1] + corr[1]];
    // pelvis attitude loop: rotate the commanded pelvis frame against the measured tilt (P + I) so the stance hips level the pelvis
    const qRef = this.yawQ(this.heading), eRot = Q.vec(Q.mul(qRef, Q.conj(this.s('pelvis').q)));
    eRot[1] = 0; // yaw is handled by the feet
    w.irot = V.add(w.irot || [0, 0, 0], V.mul(eRot, dt)); w.irot = w.irot.map((x) => clamp(x, -0.15, 0.15));
    const KPR = this.kpr ?? 1.0, KIR = this.kir ?? 3.0;
    const qPel = Q.mul(Q.mul(Q.axis([1, 0, 0], 0), (() => { const v = V.add(V.mul(eRot, KPR), V.mul(w.irot, KIR)); const n = V.len(v); return n > 1e-9 ? Q.axis(V.mul(v, 1 / n), n) : Q.id(); })()), qRef);
    for (const s of ['r', 'l']) {
      const T = tgt[s], J = this.joints;
      const swingNow = w.ph === 'ss' && s === w.swing;
      const qP = swingNow ? pel.q : qPel, pP = swingNow ? pel.p : pelPos; // swing foot aims at its world target from the real pelvis
      const ik = this.legIK(s, qP, pP, [T.p[0], T.y, T.p[1]], T.yaw);
      this.aim(J['hip_' + s], Q.mul(Q.conj(qP), ik.qThigh));
      this.aim(J['knee_' + s], Q.axis(AXIS.z, -ik.kappa));
      this.aim(J['ankle_' + s], Q.mul(Q.conj(ik.qShank), this.yawQ(T.yaw)));
    }
    this.upperBody(0.55 * clamp(Math.abs(w.vcmd) / 0.8, 0.2, 1) * (w.ph === 'ss' ? w.armPhase : 0));
    this.dbg = { ph: w.ph, n: w.n, sw: w.swing, vcmd: w.vcmd, cv: Math.hypot(...cv), xi: xi.map((n) => +n.toFixed(3)), p1: w.p1 && w.p1.map((n) => +n.toFixed(3)) };
  }
}
