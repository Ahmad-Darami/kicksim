// The whole world: ground, pedestal, ball (rigid body + air drag + Magnus force) and the biped, plus a task layer
// (walk, turn, motions, kick, grab, throw...) that the agent drives. Runs in the browser and in Node.
import { V, Q, clamp } from './vec.js';
import { buildBiped, DT, groups, G } from './biped.js';

export const PARAMS = { g: 9.81, drag: true, magnus: true, bounce: 0.7 };
const BALL_R = 0.11, BALL_M = 0.43, PEDESTAL = { x: 3, z: 0, hx: 0.2, hy: 0.45, hz: 0.3 };
const wrapA = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export async function createSim(RAPIER, opts = {}) {
  await RAPIER.init?.();
  return new Sim(RAPIER, opts);
}

export class Sim {
  constructor(RAPIER, opts = {}) {
    this.RAPIER = RAPIER; this.P = { ...PARAMS }; this.t = 0; this.events = []; this.queue = []; this.task = null;
    const world = this.world = new RAPIER.World({ x: 0, y: -this.P.g, z: 0 });
    world.integrationParameters.dt = DT; world.integrationParameters.numSolverIterations = 16;
    const gb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.5, 0));
    world.createCollider(RAPIER.ColliderDesc.cuboid(500, 0.5, 500).setFriction(1).setRestitution(this.P.bounce).setCollisionGroups(groups.ground), gb);
    const pb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(PEDESTAL.x, PEDESTAL.hy, PEDESTAL.z));
    world.createCollider(RAPIER.ColliderDesc.cuboid(PEDESTAL.hx, PEDESTAL.hy, PEDESTAL.hz).setFriction(0.8).setCollisionGroups(groups.ground), pb);
    this.pedestalTop = PEDESTAL.hy * 2;
    this.makeBall(0, BALL_R, 0);
    this.robotStart = { x: opts.x ?? -4, z: opts.z ?? 0 };
    this.biped = buildBiped(RAPIER, world, this.robotStart.x, this.robotStart.z);
    this.held = null; this.flight = { state: 'ground', launch: null, maxH: 0 };
  }

  // ---------------- ball ----------------
  makeBall(x, y, z) {
    const R = this.RAPIER;
    if (this.ball) this.world.removeRigidBody(this.ball);
    const b = this.ball = this.world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(x, y, z).setCanSleep(false).setCcdEnabled(true));
    this.ballCollider = this.world.createCollider(R.ColliderDesc.ball(BALL_R).setMass(BALL_M).setFriction(0.7).setRestitution(this.P.bounce).setRestitutionCombineRule(R.CoefficientCombineRule.Max).setCollisionGroups(groups.ball), b);
    this.flight = { state: 'ground', launch: null, maxH: 0 };
    return b;
  }
  setBall(x, y, z) { this.release(); this.makeBall(x, y, z); this.ball.setLinvel({ x: 0, y: 0, z: 0 }, true); }
  ballState() {
    const p = this.ball.translation(), v = this.ball.linvel(), w = this.ball.angvel();
    return { p: [p.x, p.y, p.z], v: [v.x, v.y, v.z], w: [w.x, w.y, w.z], speed: Math.hypot(v.x, v.y, v.z), h: p.y - BALL_R };
  }
  setParams(o) {
    Object.assign(this.P, o);
    if (o.g !== undefined) this.world.gravity = { x: 0, y: -o.g, z: 0 };
    if (o.bounce !== undefined) { this.ballCollider.setRestitution(o.bounce); }
  }
  aero(dt) {
    const b = this.ball, bs = this.ballState(), P = this.P, A = Math.PI * BALL_R * BALL_R;
    if (this.held) return;
    const onGround = bs.h < 0.02;
    let F = [0, 0, 0];
    if (P.drag && bs.speed > 1e-6) F = V.add(F, V.mul(bs.v, -0.5 * 1.2 * 0.25 * A * bs.speed));
    if (P.magnus && !onGround && bs.speed > 1e-6) {
      const wv = V.cross(bs.w, bs.v), n = V.len(wv);
      if (n > 1e-9) F = V.add(F, V.mul(wv, (0.5 * 1.2 * A * Math.min(0.4, 0.5 * BALL_R * V.len(bs.w) / bs.speed) * bs.speed * bs.speed) / n));
    }
    if (onGround) { // rolling resistance
      const hs = Math.hypot(bs.v[0], bs.v[2]);
      if (hs > 1e-4) { const d = Math.min(hs / dt, 0.06 * this.P.g); F[0] -= BALL_M * d * (bs.v[0] / hs); F[2] -= BALL_M * d * (bs.v[2] / hs); }
      b.setAngularDamping(0.8);
    } else b.setAngularDamping(0);
    b.applyImpulse({ x: F[0] * dt, y: F[1] * dt, z: F[2] * dt }, true);
  }
  track() { // detect kicks/throws: takeoff -> landing
    const bs = this.ballState(), f = this.flight;
    if (this.held) { f.state = 'ground'; return; }
    const air = bs.h > 0.05;
    if (f.state === 'ground' && air && bs.speed > 1.5) {
      const h = Math.hypot(bs.v[0], bs.v[2]);
      f.state = 'air'; f.maxH = bs.h; f.t0 = this.t;
      f.launch = { pos: bs.p, vel: bs.v, speed: bs.speed, angle_deg: (Math.atan2(bs.v[1], h) * 180) / Math.PI, azimuth_deg: (Math.atan2(bs.v[2], bs.v[0]) * 180) / Math.PI, h0: bs.h };
    } else if (f.state === 'air') {
      f.maxH = Math.max(f.maxH, bs.h);
      if (bs.h < 0.02 && bs.v[1] < 0) {
        const L = f.launch, d = [bs.p[0] - L.pos[0], bs.p[2] - L.pos[2]];
        const ev = { type: 'landed', launch: L, range_m: Math.hypot(d[0], d[1]), sideways_m: d[0] * -Math.sin((L.azimuth_deg * Math.PI) / 180) + d[1] * Math.cos((L.azimuth_deg * Math.PI) / 180), hang_time_s: this.t - f.t0, peak_height_m: f.maxH - L.h0, impact_speed_mps: bs.speed, spot: [bs.p[0], bs.p[2]] };
        f.state = 'ground'; this.events.push(ev); this.onEvent?.(ev);
      }
    }
  }
  // ideal frictionless-air parabola for the measured launch (what a physics book predicts)
  static ideal(launch, g) {
    const vy = launch.vel[1], vh = Math.hypot(launch.vel[0], launch.vel[2]), h0 = launch.h0;
    const tUp = vy / g, peak = h0 + (vy > 0 ? (vy * vy) / (2 * g) : 0), tFall = Math.sqrt((2 * peak) / g), time = tUp + tFall;
    return { range_m: vh * time, peak_height_m: peak - h0, hang_time_s: time };
  }

  // ---------------- robot ----------------
  resetRobot(x = this.robotStart.x, z = this.robotStart.z, heading = 0) {
    this.release(); this.queue = []; this.task = null;
    for (const b of Object.values(this.biped.bodies)) this.world.removeRigidBody(b);
    this.biped = buildBiped(this.RAPIER, this.world, x, z);
    this.biped.heading = heading;
    if (heading) { for (const b of Object.values(this.biped.bodies)) { const p = b.translation(), q = b.rotation(); const c = Math.cos(heading), s = Math.sin(heading); const dx = p.x - x, dz = p.z - z; b.setTranslation({ x: x + dx * c + dz * s, y: p.y, z: z - dx * s + dz * c }, true); const nq = Q.mul(Q.axis([0, 1, 0], -heading), q); b.setRotation(nq, true); } }
  }
  push(newtons, directionDeg, seconds = 0.15) { // shove the torso: direction in degrees from +x toward +z (robot's right)
    const a = (directionDeg * Math.PI) / 180, J = newtons * seconds;
    this.biped.bodies.torso.applyImpulse({ x: J * Math.cos(a), y: 0, z: J * Math.sin(a) }, true);
  }
  heldBy() { return this.held ? this.held.side : null; }
  handTip(side) { const f = this.biped.s('farm_' + side); return V.add(f.p, Q.rot(f.q, [0, -0.30, 0])); }
  grab(side) {
    if (this.held) return { ok: false, reason: 'already holding the ball' };
    const R = this.RAPIER, f = this.biped.s('farm_' + side), b = this.ballState();
    const tip = this.handTip(side), d = V.len(V.sub(b.p, tip));
    if (d > 0.28) return { ok: false, reason: `ball is ${d.toFixed(2)} m from the ${side === 'r' ? 'right' : 'left'} hand (needs < 0.28 m)`, distance: +d.toFixed(2) };
    const qb = this.ball.rotation();
    const rel = Q.rot(Q.conj(f.q), V.sub(b.p, f.p));
    const data = R.JointData.fixed({ x: rel[0], y: rel[1], z: rel[2] }, Q.id(), { x: 0, y: 0, z: 0 }, Q.mul(Q.conj(qb), f.q));
    const joint = this.world.createImpulseJoint(data, this.biped.bodies['farm_' + side], this.ball, true);
    joint.setContactsEnabled(false);
    this.held = { side, joint, prevV: this.ballState().v };
    return { ok: true, distance: +d.toFixed(2) };
  }
  release() {
    if (!this.held) return false;
    this.world.removeImpulseJoint(this.held.joint, true); this.held = null; return true;
  }
  monitorGrip(dt) { // the grip only holds so much: estimate the force on the ball and let go if it slips
    if (!this.held) return;
    const v = this.ballState().v, a = V.sub(V.mul(V.sub(v, this.held.prevV), 1 / dt), [0, -this.P.g, 0]);
    const Fraw = BALL_M * V.len(a); this.held.prevV = v;
    this.held.force = (this.held.force ?? Fraw) + (Fraw - (this.held.force ?? Fraw)) * Math.min(1, dt / 0.03); // smooth out solver spikes
    if (this.held.force > 500) { const F = this.held.force; this.release(); this.events.push({ type: 'slipped', force_N: Math.round(F) }); }
  }

  // ---------------- tasks (each returns a promise) ----------------
  run(task) { return new Promise((resolve) => { this.queue.push({ ...task, resolve, t: 0, phase: 'start' }); }); }
  cancelAll(reason = 'cancelled') { for (const t of [this.task, ...this.queue]) t?.resolve({ ok: false, reason }); this.task = null; this.queue = []; this.biped.motion = null; }
  taskStep(dt) {
    const B = this.biped;
    if (!this.task && this.queue.length) { this.task = this.queue.shift(); }
    const tk = this.task; if (!tk) return;
    tk.t += dt;
    const fin = (r = { ok: true }) => {
      if (tk.then && r.ok !== false) { this.task = null; this.queue.unshift({ ...tk.then, resolve: tk.resolve, t: 0, phase: tk.then.phase ?? 'start' }); return; }
      this.task = null; tk.resolve({ ...r, t_s: +tk.t.toFixed(1) });
    };
    if (B.fallen) return fin({ ok: false, reason: 'the robot fell over' });
    const pel = B.s('pelvis').p;
    if (tk.type === 'approach_ball') { // walk to where the chosen hand can reach the ball, facing it
      const b = this.ballState().p, dx = b[0] - pel[0], dz = b[2] - pel[2], dl = Math.hypot(dx, dz) || 1, f = [dx / dl, dz / dl], r = [-f[1], f[0]];
      const side = tk.side === 'l' ? -1 : 1, standoff = tk.standoff ?? 0.33;
      const T = [b[0] - f[0] * standoff - r[0] * 0.22 * side, b[2] - f[1] * standoff - r[1] * 0.22 * side];
      this.task = { type: 'walk_to', x: T[0], z: T[1], tol: 0.12, resolve: tk.resolve, t: 0, phase: 'start', then: { type: 'turn_to', degrees: (Math.atan2(f[1], f[0]) * 180) / Math.PI } };
      return;
    }
    if (tk.type === 'wait') { if (tk.t >= tk.seconds) fin(); return; }
    if (tk.type === 'motion') {
      if (tk.phase === 'start') { B.playMotion(tk.frames, (name) => this.motionEvent(name)); tk.phase = 'run'; }
      else if (!B.motion) fin();
      return;
    }
    if (tk.type === 'walk_to' || tk.type === 'turn_to') {
      const wantWalk = tk.type === 'walk_to';
      if (tk.phase === 'start') { tk.phase = 'go'; tk.tries = 0; }
      const dx = tk.x - pel[0], dz = tk.z - pel[2], dist = wantWalk ? Math.hypot(dx, dz) : 0;
      if (tk.backward && tk.keep === undefined) tk.keep = B.heading;
      const target = tk.backward ? tk.keep : wantWalk ? (dist > 0.12 ? Math.atan2(dz, dx) : B.heading) : (tk.degrees * Math.PI) / 180;
      const err = wrapA(target - B.heading), tol = tk.tol ?? 0.25;
      const arrived = wantWalk ? dist < tol : Math.abs(err) < 0.05;
      if (tk.phase === 'go') {
        if (arrived) { B.speedCmd = 0; B.turnCmd = 0; B.wantStand = true; tk.phase = 'stop'; }
        else {
          if (B.mode !== 'walk') B.startWalk(Math.random() < 0.5 ? 'r' : 'l');
          B.wantStand = false;
          B.turnCmd = clamp(2.2 * err, -0.7, 0.7);
          B.speedCmd = wantWalk ? (Math.abs(err) > 0.5 ? 0 : (tk.backward ? -1 : 1) * clamp(Math.sqrt(Math.max(dist - 0.15, 0)), tk.backward ? 0.2 : 0.25, tk.backward ? 0.4 : (tk.speed ?? 0.8))) : 0;
        }
      } else if (tk.phase === 'stop') {
        if (B.isStanding()) {
          if (wantWalk && dist > tol * 1.8 && tk.tries < 2) { tk.tries++; tk.phase = 'go'; }
          else fin({ ok: true, position: [+pel[0].toFixed(2), +pel[2].toFixed(2)], missed_by_m: wantWalk ? +dist.toFixed(2) : undefined });
        }
      }
      if (tk.t > (tk.timeout ?? 90)) { B.speedCmd = 0; B.wantStand = true; fin({ ok: false, reason: 'timed out' }); }
      return;
    }
    if (tk.type === 'reach') {
      if (tk.phase === 'start') {
        const tor = B.s('torso'), sh = V.add(tor.p, Q.rot(tor.q, [0, 0.17, tk.side === 'r' ? 0.22 : -0.22])), dist = V.len(V.sub(tk.target, sh));
        if (dist > 0.70) return fin({ ok: false, reason: `target is ${dist.toFixed(2)} m from the shoulder but the arm only reaches 0.70 m - walk closer first`, distance_m: +dist.toFixed(2) });
        B.reachHand(tk.side, tk.target); tk.phase = 'run';
      }
      const d = V.len(V.sub(this.handTip(tk.side), tk.target));
      if (d < 0.05 || tk.t > (tk.timeout ?? 4)) fin({ ok: d < 0.12, hand_to_target_m: +d.toFixed(2) });
      return;
    }
    if (tk.type === 'relax_arms') { B.reachHand('r', null); B.reachHand('l', null); B.armsFree = !!tk.keep; fin(); return; }
    if (tk.type === 'throw') {
      if (tk.phase === 'start') {
        if (!this.held) return fin({ ok: false, reason: 'not holding the ball' });
        tk.side = this.held.side; B.reachHand(tk.side, null);
        tk.angle = tk.angle ?? 40; const frames = Sim.throwFrames(tk.side, tk.power ?? 0.6, tk.angle);
        B.playMotion(frames.frames, (name) => this.motionEvent(name)); tk.phase = 'run';
      } else {
        // release when the arm has *actually* swung up to the release angle (the motors lag the keyframes)
        const J = B.joints['shoulder_' + tk.side], pitch = (Q.vec(B.rel(J))[2] * 180 / Math.PI) * J.dofs.pitch[1];
        if (pitch < -30) tk.sawBack = true;
        if (this.held && tk.sawBack && pitch >= (tk.angle ?? 40)) this.release();
        if (!B.motion) { B.armsFree = true; fin({ ok: true, released: !this.held }); }
      }
      return;
    }
    if (tk.type === 'grab') { fin(this.grab(tk.side)); return; }
    if (tk.type === 'release') { fin({ ok: this.release() }); return; }
    if (tk.type === 'kick') { this.kickStep(tk, fin); return; }
    fin({ ok: false, reason: 'unknown task ' + tk.type });
  }
  // underhand/sidearm throw: arm swings from behind up to the release angle (= launch angle), then lets go
  static throwFrames(side, power, angle) {
    const Ts = 0.36 - 0.2 * clamp(power, 0.05, 1), pre = 0.5, a0 = -70, a1 = angle + 30;
    const uStar = (() => { const target = (angle - a0) / (a1 - a0); let lo = 0, hi = 1; for (let i = 0; i < 30; i++) { const m = (lo + hi) / 2; (m * m * (3 - 2 * m) < target ? (lo = m) : (hi = m)); } return (lo + hi) / 2; })();
    const sh = side + '', P = (n) => (side === 'r' ? 'right_' : 'left_') + n;
    return { frames: [
      { t: pre, joints: { [P('shoulder_pitch')]: a0, [P('elbow_flex')]: 70, [P('shoulder_roll')]: 0, [P('shoulder_yaw')]: 0 } },
      { t: pre + Ts, joints: { [P('shoulder_pitch')]: a1, [P('elbow_flex')]: 15 } },
      { t: pre + Ts + 0.5, joints: { [P('shoulder_pitch')]: 10, [P('elbow_flex')]: 10 } },
    ] };
  }
  motionEvent(name) { if (name === 'release') this.release(); }
  kickStep(tk, fin) {
    const B = this.biped;
    if (tk.phase === 'start') {
      const bs = this.ballState();
      if (this.held) return fin({ ok: false, reason: 'drop the ball first (release it)' });
      if (bs.speed > 0.3) { if (tk.t > 10) fin({ ok: false, reason: 'the ball keeps moving' }); return; }
      const az = ((tk.azimuth ?? 0) * Math.PI) / 180, f = [Math.cos(az), Math.sin(az)], r = [-f[1], f[0]], sg = tk.foot === 'l' ? -1 : 1;
      tk.az = az; tk.swing = tk.foot === 'l' ? 'l' : 'r';
      // stand so the ball is 0.45 m ahead of the planted (other) foot and in line with the kicking foot
      const T = [bs.p[0] - f[0] * 0.45 - r[0] * sg * 0.09, bs.p[2] - f[1] * 0.45 - r[1] * sg * 0.09];
      this.task = { type: 'walk_to', x: T[0], z: T[1], tol: 0.08, resolve: tk.resolve, t: 0, phase: 'start', then: { type: 'turn_to', degrees: tk.azimuth ?? 0, then: { ...tk, type: 'kick', phase: 'aim' } } };
      return;
    }
    if (tk.phase === 'aim') {
      const bs = this.ballState(); B.kickDone = false;
      if (!B.isStanding()) return;
      const dx = bs.p[0] - B.s('pelvis').p[0], dz = bs.p[2] - B.s('pelvis').p[2], dist = Math.hypot(dx, dz);
      B.heading = tk.az;
      B.kickPlan = { swing: tk.swing, ball: [bs.p[0], bs.p[2]], az: tk.az, power: tk.power ?? 0.6, loft: tk.loft ?? 25 };
      tk.phase = 'strike'; tk.t = 0; this.events.length = 0; tk.dist0 = dist;
      return;
    }
    if (tk.phase === 'strike') {
      const ev = this.events.find((e) => e.type === 'landed');
      if (B.kickDone && !B.kickPlan && ev) {
        const ideal = Sim.ideal(ev.launch, this.P.g);
        return fin({ ok: true, launch: { speed_mps: +ev.launch.speed.toFixed(2), angle_deg: +ev.launch.angle_deg.toFixed(1), azimuth_deg: +ev.launch.azimuth_deg.toFixed(1) }, landing: { range_m: +ev.range_m.toFixed(2), sideways_m: +ev.sideways_m.toFixed(2), hang_time_s: +ev.hang_time_s.toFixed(2), peak_height_m: +ev.peak_height_m.toFixed(2), impact_speed_mps: +ev.impact_speed_mps.toFixed(2) }, ideal_no_air_prediction: { range_m: +ideal.range_m.toFixed(2), peak_height_m: +ideal.peak_height_m.toFixed(2), hang_time_s: +ideal.hang_time_s.toFixed(2) } });
      }
      if (B.kickDone && !B.kickPlan && tk.t > 3 && !ev) return fin({ ok: false, reason: 'the foot missed the ball or only nudged it', ball: this.state().ball });
      if (tk.t > 25) return fin({ ok: false, reason: 'kick timed out' });
    }
  }

  // ---------------- main step ----------------
  step() {
    this.taskStep(DT);
    this.biped.update(DT);
    this.aero(DT);
    this.world.step();
    this.t += DT;
    this.monitorGrip(DT);
    this.track();
  }
  // JSON-friendly snapshot for the agent
  state() {
    const B = this.biped, c = B.com(), pel = B.s('pelvis'), r2 = (n) => +n.toFixed(2);
    const bs = this.ballState();
    return {
      time_s: r2(this.t),
      robot: { position_m: [r2(pel.p[0]), r2(pel.p[2])], pelvis_height_m: r2(pel.p[1]), heading_deg: Math.round((B.heading * 180) / Math.PI), mode: B.mode, fallen: B.fallen, fall_reason: B.fallReason, speed_mps: r2(Math.hypot(c.v[0], c.v[2])), com_m: c.p.map(r2), holding_ball_with: this.heldBy(), busy: !!(this.task || this.queue.length), tilt_deg: Math.round(((B.tilt || 0) * 180) / Math.PI),
        hands_m: { right: this.handTip('r').map(r2), left: this.handTip('l').map(r2) } },
      ball: { position_m: bs.p.map(r2), speed_mps: r2(bs.speed), height_m: r2(bs.h), at_rest: bs.speed < 0.05 && bs.h < 0.03, held: !!this.held },
      pedestal: { x: PEDESTAL.x, z: PEDESTAL.z, top_height_m: this.pedestalTop },
      world: { gravity: this.P.g, air_resistance: this.P.drag, spin_curves_ball: this.P.magnus, ball_bounciness: this.P.bounce },
    };
  }
}
export { BALL_R, PEDESTAL };
