/* The app's own little simulator, for "Try offline" (no PC needed).
 *
 * Not a physics engine - just enough to practise with: the arm moves at the real
 * speed limits, the gripper really grabs a puck when the fingers are around it and
 * close, a released puck drops onto the table / belt / bin below, and the conveyor
 * cell runs by itself (sorting into 4 bins, or command mode with one bin).
 * The geometry, joint chain and pick heights are the ones of the real package
 * (they come from scene.js, generated from scripts/scara_scene.py).
 */
(() => {
'use strict';

const ARM = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint'];
const COLORS = ['red', 'green', 'blue', 'yellow'];
const SPEEDS = {                       // same table as scripts/scara_safety.py (rad/s, m/s)
  safe: { column_joint: 0.6, shoulder_joint: 0.04, forearm_joint: 0.6, wrist_joint: 1.0, grip: 0.025 },
  normal: { column_joint: 1.2, shoulder_joint: 0.08, forearm_joint: 1.2, wrist_joint: 2.0, grip: 0.06 },
  off: { column_joint: 3, shoulder_joint: 0.2, forearm_joint: 3, wrist_joint: 5, grip: 0.12 },
};
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, a, b) => Math.min(Math.max(v, a), b);

class LocalSim {
  constructor(scene, log) {
    this.sc = scene;
    this.k = scene.kin;
    this.z = scene.z;
    this.cell = scene.cell;
    this.log = log || (() => {});
    this.q = { column_joint: 0, shoulder_joint: 0, forearm_joint: 0, wrist_joint: 0, left_finger_joint: 0 };
    this.target = { ...this.q };
    this.mode = 'safe';
    this.scale = 0.6;
    this.assist = true;
    this.held = null;
    this.note = '';
    this.world = 'manual';
    this.pucks = [];
    this.belt = false;
    this.auto = null;           // running plan / cell state
    this.running = false;
    this.active = new Set();    // command mode: colours being picked
    this.queue = [];            // command mode: sequence still to come
    this.countLeft = null;
    this.sorted = { red: 0, green: 0, blue: 0, yellow: 0 };
    this.t = 0;
    this.nextSpawn = 0;
    this.n = 0;
    this.setWorld('manual');
  }

  // ---------------------------------------------------------------- kinematics
  fk(q) {
    const [cx, cy] = this.k.column;
    const t1 = q.column_joint + this.k.base_yaw;
    const ex = cx + this.k.L1 * Math.cos(t1), ey = cy + this.k.L1 * Math.sin(t1);
    const wx = ex + this.k.L2 * Math.cos(t1 + q.forearm_joint), wy = ey + this.k.L2 * Math.sin(t1 + q.forearm_joint);
    const a = t1 + q.forearm_joint + q.wrist_joint;
    return { x: wx + this.k.grip_offset * Math.cos(a), y: wy + this.k.grip_offset * Math.sin(a), yaw: a };
  }

  padZ(q) { return this.sc.table_top + this.k.pad_bottom_z0 + q.shoulder_joint; }

  within(j, v, m = 0.02) { const [lo, hi] = this.sc.limits[j]; return v >= lo + m && v <= hi - m; }

  /* pad centre at (x, y), gripper along yaw (any multiple of `sym`); null if out of reach */
  ik(x, y, yaw, prefer, sym = Math.PI) {
    let best = null;
    for (let k = -4; k <= 4; k++) {
      const psi = yaw + k * sym;
      const wx = x - this.k.grip_offset * Math.cos(psi), wy = y - this.k.grip_offset * Math.sin(psi);
      const dx = wx - this.k.column[0], dy = wy - this.k.column[1];
      const a = this.k.L1, b = this.k.L2;
      const c2 = (dx * dx + dy * dy - a * a - b * b) / (2 * a * b);
      if (Math.abs(c2) > 1) continue;
      for (const sgn of [1, -1]) {
        const t2 = sgn * Math.acos(c2);
        const t1 = wrap(Math.atan2(dy, dx) - Math.atan2(b * Math.sin(t2), a + b * Math.cos(t2)) - this.k.base_yaw);
        const t3 = wrap(psi - this.k.base_yaw - t1 - t2);
        if (!this.within('column_joint', t1) || !this.within('forearm_joint', t2) || !this.within('wrist_joint', t3)) continue;
        let cost = Math.abs(t3) * 0.5;
        if (prefer) cost += Math.abs(t1 - prefer.column_joint) + Math.abs(t2 - prefer.forearm_joint);
        if (!best || cost < best.cost) best = { cost, column_joint: t1, forearm_joint: t2, wrist_joint: t3 };
      }
    }
    return best;
  }

  /* where a puck at (x, y) is in the gripper frame */
  local(q, p) {
    const f = this.fk(q);
    const dx = p.x - f.x, dy = p.y - f.y;
    return { lx: dx * Math.cos(f.yaw) + dy * Math.sin(f.yaw), ly: -dx * Math.sin(f.yaw) + dy * Math.cos(f.yaw), d: Math.hypot(dx, dy) };
  }

  between(q, p) {                     // puck body between the open pads
    const l = this.local(q, p);
    return Math.abs(l.lx) < this.k.pad_half_len && Math.abs(l.ly) < 0.012;
  }

  fingerOver(q, p) {                  // a finger pad (not the gap) above this puck - same test as scara_safety.py
    const l = this.local(q, p);
    const inner = this.sc.gap_open / 2 + Math.min(0, q.left_finger_joint);
    const hl = this.k.pad_half_len, thick = 0.027;
    const rect = (x0, x1, y0, y1) => Math.hypot(Math.max(x0 - l.lx, 0, l.lx - x1), Math.max(y0 - l.ly, 0, l.ly - y1));
    return rect(-hl, hl, inner, inner + thick) < this.sc.puck_radius || rect(-hl, hl, -inner - thick, -inner) < this.sc.puck_radius;
  }

  // ---------------------------------------------------------------- world
  surfaceAt(x, y) {
    const top = this.sc.table_top;
    for (const b of this.bins()) {
      const c = Math.cos(-b.yaw), s = Math.sin(-b.yaw);
      const dx = x - b.xy[0], dy = y - b.xy[1];
      if (Math.abs(dx * c - dy * s) < this.sc.bin_size / 2 && Math.abs(dx * s + dy * c) < this.sc.bin_size / 2) {
        return { z: top + this.cell.bin_floor, bin: b };
      }
    }
    if (this.world !== 'manual') {
      const s = this.beltS(x, y), lat = this.beltLat(x, y);
      if (Math.abs(lat) < this.sc.belt.width / 2 && s > -0.6 && s < 0.6) return { z: this.sc.belt_top, belt: true };
    }
    return { z: top };
  }

  beltS(x, y) { return (x - this.cell.p0[0]) * this.cell.dir[0] + (y - this.cell.p0[1]) * this.cell.dir[1]; }
  beltLat(x, y) { return -(x - this.cell.p0[0]) * this.cell.dir[1] + (y - this.cell.p0[1]) * this.cell.dir[0]; }
  beltPoint(s) { return [this.cell.p0[0] + this.cell.dir[0] * s, this.cell.p0[1] + this.cell.dir[1] * s]; }

  bins() {
    if (this.world === 'sort') return this.sc.bins;
    if (this.world === 'command') return [{ color: null, xy: this.sc.command_bin.xy, yaw: this.sc.command_bin.yaw }];
    return [];
  }

  setWorld(w) {
    this.stop();
    this.world = w;
    this.held = null;
    this.belt = false;
    this.sorted = { red: 0, green: 0, blue: 0, yellow: 0 };
    this.active = new Set(); this.queue = []; this.countLeft = null;
    this.pucks = w === 'manual'
      ? this.sc.manual_pucks.map((p) => ({ c: p.name.split('_')[1], x: p.xy[0], y: p.xy[1], z: this.sc.table_top, id: ++this.n }))
      : [];
    this.nextSpawn = this.t + 0.5;
  }

  // ---------------------------------------------------------------- commands
  request(text) {
    if (this.world !== 'command') { this.log('Colours are for command mode - choose Command first', 'warning'); return; }
    const w = String(text).toLowerCase().replace(/,/g, ' ').split(/\s+/).filter(Boolean);
    if (w[0] === 'sequence' || w.includes('then')) {
      const seq = []; let rep = 1;
      for (const x of w) {
        if (/^\d+$/.test(x)) rep = Math.max(1, +x);
        else if (COLORS.includes(x)) { for (let i = 0; i < rep; i++) seq.push(x); rep = 1; }
      }
      if (!seq.length) return;
      this.queue = seq.slice(1); this.active = new Set([seq[0]]); this.countLeft = 1;
      this.log('Sequence: ' + seq.join(' → '));
    } else if (w.includes('none')) {
      this.active = new Set(); this.queue = []; this.countLeft = null;
      this.log('Picking stopped - the belt keeps running');
    } else {
      const cs = w.filter((x) => COLORS.includes(x));
      const num = w.find((x) => /^\d+$/.test(x));
      this.queue = [];
      this.active = new Set(w.includes('any') || w.includes('all') ? ['any'] : cs);
      this.countLeft = num ? +num : null;
      if (this.active.size) this.log('Now picking ' + (this.active.has('any') ? 'every colour' : [...this.active].join(' + ')) + ' until the next colour');
    }
    if (!this.running) this.start();
  }


  wants(c) {
    if (this.world === 'sort') return true;
    return this.active.has('any') || this.active.has(c);
  }

  fulfil() {
    if (this.world !== 'command' || !this.countLeft) return;
    this.countLeft -= 1;
    if (this.countLeft > 0) return;
    if (this.queue.length) {
      const nxt = this.queue.shift();
      this.active = new Set([nxt]); this.countLeft = 1;
      this.log('Sequence: next ' + nxt);
    } else {
      this.active = new Set(); this.countLeft = null;
      this.log('Done - waiting for the next colour');
    }
  }

  start() {
    if (this.world === 'manual') return false;
    this.running = true;
    this.auto = { phase: 'watch', plan: [], wait: 0 };
    if (this.world === 'sort') this.log('Sorting: every colour into its own bin');
    else if (!this.active.size) this.log('Command mode: choose a colour');
    return true;
  }

  stop() {
    if (this.running) this.log('Cell stopped');
    this.running = false;
    this.belt = false;
    this.auto = null;
    this.target = { ...this.q };
  }

  /* tap-to-move on the table, like the PC version */
  pickPlace(src, dst) {
    if (this.running) { this.log('Stop the cell first', 'warning'); return; }
    const p = this.pucks.find((u) => Math.hypot(u.x - src.x, u.y - src.y) < 0.03 && !u.held);
    if (!p) return;
    const plan = this.planPick(p, dst, this.surfaceAt(dst.x, dst.y).z);
    if (!plan) { this.log('Out of reach', 'warning'); return; }
    this.auto = { phase: 'plan', plan, wait: 0, manualPlan: true, label: 'Pick & place done' };
  }

  // ---------------------------------------------------------------- planning
  planPick(p, dst, dstSurface, dstYaw) {
    const cur = this.target;
    const yaw = this.fk(cur).yaw;
    const at = (x, y, z, yw, sym) => {
      const s = this.ik(x, y, yw, cur, sym);
      return s ? { column_joint: s.column_joint, shoulder_joint: z, forearm_joint: s.forearm_joint, wrist_joint: s.wrist_joint } : null;
    };
    const surf = p.z;
    const zPick = Math.max(surf + 0.006 - this.sc.table_top - this.k.pad_bottom_z0, this.sc.limits.shoulder_joint[0] + 0.002);
    const zPre = surf + this.cell.handle_top + 0.02 - this.sc.table_top - this.k.pad_bottom_z0;
    const zPut = Math.max(dstSurface + 0.006 + 0.012 - this.sc.table_top - this.k.pad_bottom_z0, this.sc.limits.shoulder_joint[0] + 0.002);
    const T = this.z.travel;
    const a1 = at(p.x, p.y, T, yaw, Math.PI);
    if (!a1) return null;
    const a2 = { ...a1, shoulder_joint: zPre }, a3 = { ...a1, shoulder_joint: zPick }, a4 = { ...a1, shoulder_joint: T };
    const b1 = at(dst.x, dst.y, T, dstYaw === undefined ? yaw : dstYaw, dstYaw === undefined ? Math.PI : Math.PI / 2);
    if (!b1) return null;
    const b2 = { ...b1, shoulder_joint: zPut }, b3 = { ...b1, shoulder_joint: T };
    return [
      { grip: 0 }, { q: { ...this.target, shoulder_joint: T } },
      { q: a1 }, { q: a2 }, { q: a3 }, { grip: this.sc.grip_close, wait: 0.5 }, { q: a4 },
      { q: b1 }, { q: b2 }, { grip: 0, wait: 0.35 }, { q: b3 },
    ];
  }

  planReady() {
    const s = this.ik(this.sc.ready_xy[0], this.sc.ready_xy[1], this.fk(this.target).yaw, this.target, Math.PI);
    return s ? [{ q: { column_joint: s.column_joint, shoulder_joint: this.z.travel, forearm_joint: s.forearm_joint, wrist_joint: s.wrist_joint } }] : [];
  }

  // ---------------------------------------------------------------- stepping
  reached(q) { return ARM.every((j) => Math.abs(this.q[j] - q[j]) < (j === 'shoulder_joint' ? 0.002 : 0.01)); }

  runPlan(dt) {
    const a = this.auto;
    if (a.wait > 0) { a.wait -= dt; return false; }
    const st = a.plan[0];
    if (!st) return true;
    if (st.q) {
      if (!st.sent) { Object.assign(this.target, st.q); st.sent = true; st.t0 = this.t; }
      if (this.reached(st.q) || this.t - st.t0 > 8) a.plan.shift();   // a guard may stop Z a little short
    } else if ('grip' in st) {
      if (!st.sent) { this.target.left_finger_joint = st.grip; st.sent = true; a.wait = st.wait || 0.4; return false; }
      a.plan.shift();
    }
    return a.plan.length === 0;
  }

  cellStep(dt) {
    const a = this.auto;
    if (!a) return;
    if (a.phase === 'plan') {                                  // tap-to-move
      if (this.runPlan(dt)) {
        if (a.loop) { a.plan = a.loop.map((x) => ({ ...x })); return; }     // teach & replay, loop
        this.auto = null;
        this.log(a.label || 'Done');
      }
      return;
    }
    if (a.phase === 'watch') {
      this.belt = true;
      const [lo, hi] = this.cell.pick_window;
      const hit = this.pucks.find((p) => !p.held && Math.abs(p.z - this.sc.belt_top) < 0.005
        && this.beltS(p.x, p.y) >= lo && this.beltS(p.x, p.y) <= hi + 0.02 && this.wants(p.c));
      if (hit) {
        this.belt = false;
        a.phase = 'settle'; a.wait = 0.4; a.puck = hit;
      }
    } else if (a.phase === 'settle') {
      a.wait -= dt;
      if (a.wait > 0) return;
      const p = a.puck;
      const bin = this.world === 'sort' ? this.sc.bins.find((b) => b.color === p.c) : { xy: this.sc.command_bin.xy, yaw: this.sc.command_bin.yaw };
      const plan = this.planPick(p, { x: bin.xy[0], y: bin.xy[1] }, this.sc.table_top + this.cell.bin_floor, bin.yaw);
      if (!plan) { a.phase = 'watch'; return; }
      a.plan = plan.concat(this.planReady());
      a.phase = 'pick';
      this.log(`Picking ${p.c} puck`);
    } else if (a.phase === 'pick') {
      if (this.runPlan(dt)) {
        const p = a.puck;
        if (this.surfaceAt(p.x, p.y).bin) {
          this.sorted[p.c] += 1;
          this.log(`Placed ${p.c}  (red ${this.sorted.red} · green ${this.sorted.green} · blue ${this.sorted.blue} · yellow ${this.sorted.yellow})`);
          this.fulfil();
        }
        a.phase = 'watch';
      }
    }
  }

  spawn() {
    if (this.world === 'manual' || !this.belt || this.t < this.nextSpawn) return;
    const s0 = this.cell.spawn_s;
    const onBelt = this.pucks.filter((p) => Math.abs(p.z - this.sc.belt_top) < 0.005);
    if (onBelt.length >= 6 || onBelt.some((p) => this.beltS(p.x, p.y) < s0 + 0.15)) { this.nextSpawn = this.t + 0.5; return; }
    const [x, y] = this.beltPoint(s0);
    this.pucks.push({ c: COLORS[Math.floor(Math.random() * 4)], x, y, z: this.sc.belt_top, id: ++this.n });
    this.nextSpawn = this.t + 2.2 + Math.random() * 1.5;
  }

  step(dt) {
    dt = Math.min(dt, 0.05);
    this.t += dt;
    if (this.running || (this.auto && this.auto.manualPlan)) this.cellStep(dt);
    const sp = SPEEDS[this.running || (this.auto && this.auto.manualPlan) ? 'normal' : this.mode];
    const scale = this.running ? 1 : this.scale;
    this.note = '';
    const tgt = { ...this.target };
    // pick assist (manual): open gripper coming down close to a puck -> centre over it
    const open = this.q.left_finger_joint > -0.003 && !this.held;
    if (open && this.assist && this.mode !== 'off' && !this.running && tgt.shoulder_joint < this.q.shoulder_joint - 1e-4) {
      for (const p of this.pucks) {
        const l = this.local(this.q, p);
        const above = this.padZ(this.q) - (p.z + this.sc.puck_height);
        if (l.d < 0.045 && l.d > 0.002 && above < 0.045) {
          const s = this.ik(p.x, p.y, this.fk(this.q).yaw, this.q, Math.PI);
          if (s) { tgt.column_joint = s.column_joint; tgt.forearm_joint = s.forearm_joint; tgt.wrist_joint = s.wrist_joint; this.note = 'assist: centring over the puck'; }
          Object.assign(this.target, { column_joint: tgt.column_joint, forearm_joint: tgt.forearm_joint, wrist_joint: tgt.wrist_joint });
          break;
        }
      }
    }
    // move every joint towards its target at the speed limit
    for (const j of ARM) {
      const st = sp[j] * scale * dt;
      this.q[j] += clamp(tgt[j] - this.q[j], -st, st);
    }
    const gst = sp.grip * Math.max(scale, 0.5) * dt;
    let g = this.q.left_finger_joint + clamp(tgt.left_finger_joint - this.q.left_finger_joint, -gst, gst);
    // the fingers stop on a puck between them; closing on it grabs it
    if (!this.held) {
      const touch = -(this.sc.gap_open - 2 * this.sc.puck_radius) / 2;           // -0.0105
      for (const p of this.pucks) {
        const padLow = this.padZ(this.q) < p.z + this.sc.puck_height - 0.008;
        if (padLow && this.between(this.q, p) && g <= touch) {
          this.held = p; p.held = true; g = touch;
          this.note = 'holding the puck';
          break;
        }
      }
    } else {
      const touch = -(this.sc.gap_open - 2 * this.sc.puck_radius) / 2;
      if (tgt.left_finger_joint < touch) g = Math.max(g, touch);
      if (g > touch + 0.003) { this.drop(); }
    }
    this.q.left_finger_joint = g;
    // guards: pads never below the table / belt / bin floor, a finger never pushes a puck down
    const f = this.fk(this.q);
    let floor = this.surfaceAt(f.x, f.y).z + 0.006;
    if (this.mode !== 'off' || this.running) {
      for (const p of this.pucks) {
        if (p === this.held || p.held) continue;
        if (this.fingerOver(this.q, p)) floor = Math.max(floor, p.z + this.sc.puck_height + 0.002);
      }
    }
    const zMin = Math.max(floor - this.sc.table_top - this.k.pad_bottom_z0, this.sc.limits.shoulder_joint[0]);
    if (this.q.shoulder_joint < zMin) {
      this.q.shoulder_joint = zMin;
      if (this.target.shoulder_joint < zMin) this.target.shoulder_joint = zMin;
      if (!this.running && floor > this.surfaceAt(f.x, f.y).z + 0.007) this.note = 'guard: a finger is on a puck - centre the gripper';
    }
    // carried puck follows the pads
    if (this.held) {
      const fk = this.fk(this.q);
      Object.assign(this.held, { x: fk.x, y: fk.y, z: this.padZ(this.q) - 0.006 });
    }
    // belt, falling pucks, bins emptying
    const v = this.belt ? this.cell.speed * dt : 0;
    for (const p of this.pucks) {
      if (p.held) continue;
      const s = this.surfaceAt(p.x, p.y);
      if (p.z > s.z + 1e-4) p.z = Math.max(s.z, p.z - 0.6 * dt);
      if (v && s.belt && Math.abs(p.z - s.z) < 1e-3) { p.x += this.cell.dir[0] * v; p.y += this.cell.dir[1] * v; }
      if (s.bin && this.world !== 'manual') p.binT = (p.binT || 0) + dt;
    }
    this.pucks = this.pucks.filter((p) => p.held || ((p.binT || 0) < 2.5 && this.beltS(p.x, p.y) < this.cell.end_s - 0.02 || this.world === 'manual'));
    this.spawn();
  }

  drop() {
    if (!this.held) return;
    this.held.held = false;
    this.held = null;
  }

  state() {
    return {
      joints: { ...this.q }, pucks: this.pucks.map((p) => ({ c: p.c, x: p.x, y: p.y, z: p.z })), running: this.running,
      active: [...this.active], sequence: this.countLeft === 1 && this.active.size === 1 && !this.active.has('any') ? [...this.active, ...this.queue] : [],
      sorted: this.sorted, world: this.world, note: this.note,
    };
  }
}

window.LocalSim = LocalSim;
})();
