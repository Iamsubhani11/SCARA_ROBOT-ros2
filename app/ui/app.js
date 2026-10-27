/* SCARA Digital Twin 3.2 - phone app and website.
 *
 * Talks to scripts/scara_twin_server.py on the PC (port 9090). That server runs the
 * MuJoCo table / cell, follows a running Gazebo cell, and talks to the real robot
 * over USB - so this one app drives the simulation, the real arm, or both.
 *
 * If the other end is a plain rosbridge_server (digital_twin.launch.py rosbridge:=true)
 * the app falls back to publishing ROS topics: sticks, gripper, conveyor and colours
 * still work, the twin-only parts (teach, robot tab, safety) are greyed out.
 *
 * "Try offline" runs everything on the phone (sim.js): manual picking with the joysticks,
 * tap-to-move, and the autonomous cell (sorting or command mode) - no PC needed.
 */
(() => {
'use strict';

const SC = window.SCARA_SCENE;
const JOINTS = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint', 'left_finger_joint'];
const ARM = JOINTS.slice(0, 4);
const RATE = { column_joint: 0.7, shoulder_joint: 0.045, forearm_joint: 0.7, wrist_joint: 1.4 };   // per s at 100 %
const PUCK_RGB = { red: 0xd63a32, green: 0x2f9e44, blue: 0x1c6ed6, yellow: 0xe6b400 };
const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { const v = localStorage.getItem('scara.' + k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem('scara.' + k, JSON.stringify(v)); } catch (e) { /* private mode */ } },
};

let scene = SC;
const app = {
  link: 'off',          // off | wait | twin | rosbridge | demo
  st: null,             // last state from the PC
  q: { column_joint: 0, shoulder_joint: 0, forearm_joint: 0, wrist_joint: 0, left_finger_joint: 0 },
  real: null,           // real robot joints (twin)
  target: null,         // rosbridge / demo: our own targets
  pucks: [],
  speed: store.get('speed', 60) / 100,
  tapMode: false, picked: null,
  seqBuild: false, seq: [],
  lastLogId: 0,
  sim: null,            // offline simulator (sim.js)
  drive: store.get('drive', 'joints'),   // joints | xy
  cart: null,           // XY drive: gripper target {x, y, z, yaw} while a stick is held
  cartQ: null,          // XY drive: joint target we last sent
  tilt: null,           // tilt drive: {beta0, gamma0, x, y}
  follow: false,
  teachLocal: store.get('teach', []),
  firstPick: null,
};
const KIN = { get k() { return scene.kin || SC.kin; }, get sc() { return scene.limits ? scene : SC; } };
const kfk = (q) => window.LocalSim.prototype.fk.call(KIN, q);
const kik = (x, y, yaw, prefer) => window.LocalSim.prototype.ik.call(
  Object.assign(Object.create(window.LocalSim.prototype), { k: KIN.k, sc: KIN.sc }), x, y, yaw, prefer, 2 * Math.PI);
function buzz(ms) { try { if (navigator.vibrate && store.get('haptics', true)) navigator.vibrate(ms); } catch (e) { /* no vibrator */ } }

/* ------------------------------------------------------------------ helpers */
let toastTimer = 0;
function toast(text) {
  const t = $('toast');
  t.textContent = text; t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, 3200);
}
const fix = (v, d) => { const t = v.toFixed(d); return /^-0(\.0+)?$/.test(t) ? t.slice(1) : t; };   // no "-0.0"
const deg = (r) => fix(r * 180 / Math.PI, 1) + '°';
const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
function limits(j) {
  // with the real robot in the loop (Real robot / Twin) its own soft limits apply too - the PC sends them
  const r = app.st && app.link === 'twin' && app.st.robot && app.st.robot.limits;
  if (r && r[j]) return r[j];
  return (scene.limits && scene.limits[j]) || [-10, 10];
}

/* ------------------------------------------------------------------ 3-D view */
const canvas = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'low-power' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x26313b);
const world = new THREE.Scene();
world.background = new THREE.Color(0x26313b);
const cam = new THREE.PerspectiveCamera(40, 1, 0.02, 20);
cam.up.set(0, 0, 1);
const controls = new THREE.OrbitControls(cam, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.maxPolarAngle = Math.PI * 0.49;
controls.minDistance = 0.35;
controls.maxDistance = 4;

world.add(new THREE.HemisphereLight(0xf2f5f8, 0x4a4038, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(-1.2, -0.8, 2.4);
world.add(sun);
const fill = new THREE.DirectionalLight(0xdfe8ff, 0.6);
fill.position.set(1.5, 1.2, 1.0);
world.add(fill);

const mat = {
  blue: new THREE.MeshStandardMaterial({ color: 0x0a66cc, metalness: 0.15, roughness: 0.5 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x0d2a66, metalness: 0.2, roughness: 0.5 }),
  grey: new THREE.MeshStandardMaterial({ color: 0xb3b3b3, metalness: 0.4, roughness: 0.4 }),
  camera: new THREE.MeshStandardMaterial({ color: 0x232327, roughness: 0.6 }),
  name: new THREE.MeshStandardMaterial({ color: 0xf5f5f5, roughness: 0.35 }),
  ghost: new THREE.MeshBasicMaterial({ color: 0xf2b705, transparent: true, opacity: 0.28, depthWrite: false,
    polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -4 }),   // drawn over the sim arm, no flicker
  wood: new THREE.MeshStandardMaterial({ color: 0xc98a4b, roughness: 0.8 }),
  woodTop: new THREE.MeshStandardMaterial({ color: 0xe6a564, roughness: 0.75 }),
  belt: new THREE.MeshStandardMaterial({ color: 0x3b3e44, roughness: 0.9 }),
  rail: new THREE.MeshStandardMaterial({ color: 0xd9dde0, metalness: 0.3, roughness: 0.4 }),
  bin: new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.6 }),
};

function meshGeometry(name) {
  const m = window.SCARA_MESHES && window.SCARA_MESHES[name];
  if (!m) return null;
  const bin = atob(m.data);
  const u16 = new Uint16Array(bin.length / 2);
  for (let i = 0; i < u16.length; i++) u16[i] = bin.charCodeAt(2 * i) | (bin.charCodeAt(2 * i + 1) << 8);
  const pos = new Float32Array(u16.length);
  for (let i = 0; i < u16.length; i += 3) {
    for (let k = 0; k < 3; k++) pos[i + k] = (m.min[k] + u16[i + k] / 65535 * m.span[k]) * 0.001;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.computeVertexNormals();
  return g;
}

function part(parent, meshName, material, offset) {
  const g = meshGeometry(meshName);
  if (!g) return null;
  const m = new THREE.Mesh(g, material);
  m.position.set(offset[0], offset[1], offset[2]);
  parent.add(m);
  return m;
}

/* robot: same joint chain and mesh offsets as urdf/arm.xacro */
function buildRobot(ghost) {
  const M = (m) => (ghost ? mat.ghost : m);
  const r = {};
  r.base = new THREE.Group();
  const b = scene.base_xyz || [0.2, 0, 0.4];
  r.base.position.set(b[0], b[1], b[2]);
  r.base.rotation.z = scene.base_yaw;
  part(r.base, 'base_link', M(mat.blue), [0, 0, 0]);
  r.column = new THREE.Group(); r.column.position.set(0.045, 0, 0.089); r.base.add(r.column);
  part(r.column, 'column_link', M(mat.blue), [-0.045, 0, -0.089]);
  r.shoulder = new THREE.Group(); r.shoulder.position.set(0, 0, 0.28); r.column.add(r.shoulder);
  const so = [-0.059954333, -0.059974857, -0.39];
  part(r.shoulder, 'shoulder_link', M(mat.blue), so);
  if (!ghost) part(r.shoulder, 'shoulder_name', mat.name, so);
  r.forearm = new THREE.Group(); r.forearm.position.set(0.228, 0, -0.04); r.shoulder.add(r.forearm);
  part(r.forearm, 'forearm_link', M(mat.blue), [-0.273, 0, -0.351]);
  r.wrist = new THREE.Group(); r.wrist.position.set(0.1365, 0, -0.06); r.forearm.add(r.wrist);
  part(r.wrist, 'wrist_link', M(mat.dark), [-0.4095, 0, -0.291]);
  // arm camera on the forearm, beside the wrist (does not turn with the gripper) - scara_scene.py ARM_CAM_*
  const ac = scene.arm_cam || { xyz: [0.205, 0, 0], pitch: 1.71681 };
  r.camLink = new THREE.Group(); r.camLink.position.set(ac.xyz[0], ac.xyz[1], ac.xyz[2]); r.camLink.rotation.y = ac.pitch; r.forearm.add(r.camLink);
  if (!ghost) {
    part(r.camLink, 'camera_link', mat.camera, [-0.1405, 0, -0.115]);
    const br = new THREE.Mesh(new THREE.BoxGeometry(0.011, 0.032, 0.024), mat.dark);
    br.position.set(0.1875, 0, -0.006); r.forearm.add(br);
  }
  r.left = new THREE.Group(); r.wrist.add(r.left);
  part(r.left, 'left_finger_link', M(mat.grey), [-0.409, -0.0626, -0.2775]);
  r.right = new THREE.Group(); r.wrist.add(r.right);
  part(r.right, 'right_finger_link', M(mat.grey), [-0.409, 0.0626, -0.2775]);
  world.add(r.base);
  r.set = (q) => {
    r.column.rotation.z = q.column_joint || 0;
    r.shoulder.position.z = 0.28 + (q.shoulder_joint || 0);
    r.forearm.rotation.z = q.forearm_joint || 0;
    r.wrist.rotation.z = q.wrist_joint || 0;
    const g = q.left_finger_joint || 0;
    r.left.position.set(0, 0.064 + g, -0.0135);
    r.right.position.set(0, -0.064 - g, -0.0135);
  };
  r.set({});
  return r;
}

/* the cell around the robot */
const cell = new THREE.Group();
world.add(cell);
function box(sx, sy, sz, x, y, z, m, yaw) {
  const o = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m);
  o.position.set(x, y, z);
  if (yaw) o.rotation.z = yaw;
  cell.add(o);
  return o;
}
let cellKind = '';
function buildCell(kind) {
  if (kind === cellKind) return;
  cellKind = kind;
  while (cell.children.length) cell.remove(cell.children[0]);
  const [tx, ty] = scene.table || [0.9, 1.2];
  const [cx, cy] = scene.table_center || [0, 0];
  const top = scene.table_top || 0.4;
  box(tx, ty, 0.04, cx, cy, top - 0.02, mat.woodTop);
  box(tx, 0.04, top - 0.04, cx, cy - ty / 2 + 0.02, (top - 0.04) / 2, mat.wood);
  box(tx, 0.04, top - 0.04, cx, cy + ty / 2 - 0.02, (top - 0.04) / 2, mat.wood);
  box(0.04, ty, top - 0.04, cx + tx / 2 - 0.02, cy, (top - 0.04) / 2, mat.wood);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: 0x3a4651, roughness: 1 }));
  cell.add(floor);
  const grid = new THREE.GridHelper(6, 30, 0x52606c, 0x46525d);
  grid.rotation.x = Math.PI / 2; grid.position.z = 0.001;
  cell.add(grid);
  const kin = scene.kin || SC.kin;
  if (kin) {                                    // how far the gripper can reach, on the table
    const r1 = kin.L1 + kin.L2 + kin.grip_offset, r0 = Math.abs(kin.L1 - kin.L2 - kin.grip_offset);
    const ring = new THREE.Mesh(new THREE.RingGeometry(r0, r1, 96),
      new THREE.MeshBasicMaterial({ color: 0x0a5cc2, transparent: true, opacity: 0.08, depthWrite: false }));
    ring.position.set(kin.column[0], kin.column[1], top + 0.0015);
    cell.add(ring);
    const edge = new THREE.Mesh(new THREE.RingGeometry(r1 - 0.002, r1, 128),
      new THREE.MeshBasicMaterial({ color: 0x0a5cc2, transparent: true, opacity: 0.45, depthWrite: false }));
    edge.position.copy(ring.position);
    cell.add(edge);
  }
  if (kind.startsWith('hardware')) buildHwCell(app.hwLayout || {}, top);
  if (kind === 'sort' || kind === 'command') {
    const a = scene.belt.a, b = scene.belt.b, w = scene.belt.width;
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
    const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
    const bt = scene.belt_top || top + 0.035;
    box(len, w, bt - top, mx, my, top + (bt - top) / 2, mat.belt, yaw);
    const nx = -Math.sin(yaw), ny = Math.cos(yaw);
    for (const s of [-1, 1]) box(len, 0.012, 0.03, mx + s * nx * (w / 2 + 0.008), my + s * ny * (w / 2 + 0.008), bt + 0.005, mat.rail, yaw);
    const bins = kind === 'sort' ? scene.bins.map((x) => ({ ...x }))
      : [{ color: null, xy: scene.command_bin.xy, yaw: scene.command_bin.yaw }];
    const s = scene.bin_size, h = scene.bin_height || 0.03;
    for (const bn of bins) {
      const g = new THREE.Group();
      g.position.set(bn.xy[0], bn.xy[1], top);
      g.rotation.z = bn.yaw;
      const m = bn.color ? new THREE.MeshStandardMaterial({ color: PUCK_RGB[bn.color], roughness: 0.6 }) : mat.bin;
      const add = (sx, sy, sz, x, y, z) => { const o = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), m); o.position.set(x, y, z); g.add(o); };
      add(s, s, 0.008, 0, 0, 0.004);
      add(s, 0.005, h, 0, s / 2, h / 2); add(s, 0.005, h, 0, -s / 2, h / 2);
      add(0.005, s, h, s / 2, 0, h / 2); add(0.005, s, h, -s / 2, 0, h / 2);
      cell.add(g);
    }
  }
}

/* the real camera cell, laid out like the hardware (Part 24/25 videos): a short red conveyor
   in front of the robot, the webcam on a black stand beside the pick point, cubes on the belt.
   Where the belt is comes from the poses taught on the real robot (pick + upstream). */
function buildHwCell(lay, top) {
  const k = scene.kin || SC.kin;
  let pick = lay.pick, dir = lay.dir;
  if (!pick || !dir) {                          // not taught yet: the belt of the sim cell, nearest the robot
    const a = scene.belt.a, b = scene.belt.b, L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const u = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
    const t = (k.column[0] - a[0]) * u[0] + (k.column[1] - a[1]) * u[1];
    pick = [a[0] + u[0] * t, a[1] + u[1] * t];
    dir = [-u[0], -u[1]];
  }
  const yaw = Math.atan2(dir[1], dir[0]), nx = -dir[1], ny = dir[0];
  const back = 0.18, fwd = 0.18, len = back + fwd, w = 0.065;              // the pick window in the middle of the belt
  const mx = pick[0] + dir[0] * (fwd - back) / 2, my = pick[1] + dir[1] * (fwd - back) / 2;
  const bt = scene.belt_top || top + 0.035, red = new THREE.MeshStandardMaterial({ color: 0xc62828, roughness: 0.55 });
  const black = new THREE.MeshStandardMaterial({ color: 0x1b1d20, roughness: 0.9 });
  box(len, w, 0.004, mx, my, bt - 0.002, black, yaw);                          // belt
  for (const sd of [-1, 1]) {                                                   // red side frames
    box(len + 0.03, 0.008, bt - top + 0.006, mx + sd * nx * (w / 2 + 0.004), my + sd * ny * (w / 2 + 0.004), top + (bt - top + 0.006) / 2, red, yaw);
  }
  for (const e of [-1, 1]) {                                                    // end rollers
    const r = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, w + 0.022, 24), red);
    r.rotation.z = yaw;
    r.position.set(mx + e * dir[0] * len / 2, my + e * dir[1] * len / 2, top + 0.018);
    cell.add(r);
  }
  // pick zone on the belt (the camera window)
  const win = lay.window_mm || [-12, 18];
  const zc = (win[0] + win[1]) / 2000, zl = (win[1] - win[0]) / 1000;
  const zone = new THREE.Mesh(new THREE.PlaneGeometry(Math.max(zl, 0.01), w),
    new THREE.MeshBasicMaterial({ color: 0xf2b705, transparent: true, opacity: 0.35, depthWrite: false }));
  zone.position.set(pick[0] + dir[0] * zc, pick[1] + dir[1] * zc, bt + 0.0005); zone.rotation.z = yaw;
  cell.add(zone);
  // the webcam (as in the reference videos): its stand beside the belt on the far side from the robot, the arm reaching
  // over the belt so the camera hangs above the middle of the belt, looking down at the pick window
  const toCol = [k.column[0] - pick[0], k.column[1] - pick[1]];
  const side = toCol[0] * nx + toCol[1] * ny > 0 ? -1 : 1;
  // the head sits 6 cm to the downstream side of the pick window, so the gripper can come down past it
  const up = 0.06, hx = pick[0] - dir[0] * up, hy = pick[1] - dir[1] * up;
  const off = w / 2 + 0.07, sx = hx + side * nx * off, sy = hy + side * ny * off, H = 0.26;
  box(0.05, 0.07, 0.006, sx, sy, top + 0.003, black, yaw);                     // foot
  box(0.012, 0.012, H, sx, sy, top + H / 2, black);                            // pole
  const look = Math.atan2(hy - sy, hx - sx);
  box(off + 0.01, 0.012, 0.012, (sx + hx) / 2, (sy + hy) / 2, top + H, black, look);   // arm over the belt
  const camG = new THREE.Group();
  camG.position.set(hx, hy, top + H - 0.02);
  camG.rotation.z = yaw;
  camG.rotation.y = Math.atan2(up, H - 0.02 - (bt - top));                    // tipped towards the pick window
  const cb = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.07, 0.025), black); camG.add(cb);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.006, 20), new THREE.MeshStandardMaterial({ color: 0x0b0c0e, roughness: 0.2, metalness: 0.4 }));
  lens.rotation.x = Math.PI / 2; lens.position.z = -0.0145; camG.add(lens);    // pointing down at the belt
  cell.add(camG);
  // where the robot drops each colour (taught drop poses)
  const pad = 0.05;
  for (const [c, xy] of Object.entries(lay.drops || {})) {
    const m = new THREE.MeshStandardMaterial({ color: PUCK_RGB[c] || 0x9aa4ad, roughness: 0.7, transparent: true, opacity: 0.55 });
    const o = new THREE.Mesh(new THREE.BoxGeometry(pad, pad, 0.003), m);
    o.position.set(xy[0], xy[1], top + 0.0015);
    cell.add(o);
  }
}

/* pucks (from the PC, or the manual table layout offline) */
const puckGroup = new THREE.Group();
world.add(puckGroup);
const puckGeo = new THREE.CylinderGeometry(1, 1, 1, 40);
puckGeo.rotateX(Math.PI / 2);
const cubeGeo = new THREE.BoxGeometry(0.025, 0.025, 0.025);
const ringGeo = new THREE.TorusGeometry(0.011, 0.0025, 10, 28);
const puckMats = {};
function puckMat(c) {
  if (!puckMats[c]) puckMats[c] = new THREE.MeshStandardMaterial({ color: PUCK_RGB[c] || 0x888888, roughness: 0.45 });
  return puckMats[c];
}
function drawPucks(list) {
  const r = scene.puck_radius || 0.035, h = scene.puck_height || 0.035;
  while (puckGroup.children.length < list.length) {
    const g = new THREE.Group();
    const body = new THREE.Mesh(puckGeo); body.scale.set(r, r, h); body.position.z = h / 2; g.add(body);
    const ring = new THREE.Mesh(ringGeo); ring.rotation.x = Math.PI / 2; ring.position.z = h + 0.011; g.add(ring);
    puckGroup.add(g);
  }
  puckGroup.children.forEach((g, i) => {
    const p = list[i];
    g.visible = !!p;
    if (!p) return;
    g.position.set(p.x, p.y, p.z);
    const cube = !!p.cube;                       // the real cell: 25 mm cubes seen by the webcam
    if (g.children[0].userData.isCube !== cube) {
      g.children[0].userData.isCube = cube;
      g.children[0].geometry = cube ? cubeGeo : puckGeo;
      g.children[0].scale.set(cube ? 1 : r, cube ? 1 : r, cube ? 1 : h);
      g.children[0].position.z = cube ? 0.0125 : h / 2;
      g.children[1].visible = !cube;
    }
    const m = puckMat(p.c);
    g.children[0].material = m; g.children[1].material = m;
    g.userData = p;
    g.children[0].userData.puck = p;
  });
}

const robot = buildRobot(false);
const ghost = buildRobot(true);
ghost.base.visible = false;
buildCell('manual');

/* arm camera: looks along +x of camera_link (Gazebo convention), image up = +z */
const armCam = new THREE.PerspectiveCamera(55, 4 / 3, 0.01, 5);
armCam.matrixAutoUpdate = false;
const camMount = new THREE.Object3D();
camMount.matrix.makeBasis(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0));
camMount.matrix.setPosition(0.012, 0, 0);                  // the lens, on the front face of the camera body
camMount.matrixAutoUpdate = false;
robot.camLink.add(camMount);
let showInset = store.get('inset', true);

const views = {
  front: { pos: [-1.25, -0.55, 1.15], at: [0.08, 0.0, 0.5] },
  top: { pos: [0.02, 0.0, 2.1], at: [0.02, 0.0, 0.4] },
};
function setView(v) {
  const s = views[v];
  cam.position.set(...s.pos);
  controls.target.set(...s.at);
  controls.update();
}
setView('front');

/* arm camera window: full size by default (the maximum), smaller with a pinch or the corner handle */
const INSET_MIN = 0.45;
let insetScale = clamp(+store.get('insetScale', 1) || 1, INSET_MIN, 1);
function insetSize(w) {
  const iw = Math.round(clamp(w * 0.36, 110, 260) * insetScale);
  return [iw, Math.round(iw * 3 / 4)];
}
function setInsetScale(v, save) {
  insetScale = clamp(v, INSET_MIN, 1);
  resize();
  if (save) store.set('insetScale', +insetScale.toFixed(3));
}
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  cam.aspect = w / h;
  cam.updateProjectionMatrix();
  const [iw, ih] = insetSize(w);
  const f = $('insetFrame');
  f.style.width = iw + 'px'; f.style.height = ih + 'px';
  f.hidden = !showInset;
  const cs = getComputedStyle(f);                // the picture goes exactly where the CSS puts its frame
  insetOff = { top: parseFloat(cs.top) || 8, right: parseFloat(cs.right) || 8 };
  document.documentElement.style.setProperty('--inset-w', showInset ? iw + 'px' : '0px');
}
let insetOff = { top: 8, right: 8 };
new ResizeObserver(resize).observe(canvas);

function render() {
  controls.update();
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, w, h);
  renderer.render(world, cam);
  if (showInset) {
    world.updateMatrixWorld();
    armCam.matrixWorld.copy(camMount.matrixWorld);
    armCam.matrixWorldInverse.copy(armCam.matrixWorld).invert();
    const [iw, ih] = insetSize(w);
    const x = w - iw - insetOff.right, y = h - ih - insetOff.top;   // gl: origin bottom-left -> top-right corner
    renderer.setScissorTest(true);
    renderer.setScissor(x, y, iw, ih);
    renderer.setViewport(x, y, iw, ih);
    renderer.render(world, armCam);
    renderer.setScissorTest(false);
  }
}

/* ------------------------------------------------------------------ connection */
let ws = null, msgId = 0, helloTimer = 0, retryTimer = 0, wantConnected = false;

function setLink(state, text) {
  app.link = state;
  const c = $('conn');
  c.dataset.state = { twin: 'on', rosbridge: 'on', demo: 'demo', wait: 'wait' }[state] || 'off';
  $('connText').textContent = text;
  document.body.dataset.link = state;
  refreshEnabled();
}

function connect(ip, port) {
  disconnect(false);
  wantConnected = true;
  store.set('ip', ip); store.set('port', port);
  setLink('wait', 'Connecting…');
  $('subtitle').textContent = `${ip}:${port}`;
  try {
    ws = new WebSocket(`ws://${ip}:${port}`);
  } catch (e) {
    setLink('off', 'Connect'); toast('That address does not look right'); return;
  }
  ws.onopen = () => {
    send({ op: 'scara', id: 'hello', cmd: 'hello', args: { device: window.ScaraNative ? 'phone app' : 'website' } });
    helloTimer = setTimeout(startRosbridge, 1500);        // no answer: plain rosbridge_server
  };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch (e) { return; }
    onMessage(m);
  };
  ws.onclose = () => {
    clearTimeout(helloTimer);
    showBusy(''); app.pendingWorld = null;
    if (app.link !== 'demo') setLink('off', 'Connect');
    if (wantConnected) {
      $('subtitle').textContent = 'Lost the PC - retrying…';
      retryTimer = setTimeout(() => connect(store.get('ip'), store.get('port', 9090)), 3000);
    }
  };
  ws.onerror = () => { $('subtitle').textContent = `No answer from ${ip}:${port}`; };
}

function disconnect(user = true) {
  if (user) wantConnected = false;
  clearTimeout(retryTimer); clearTimeout(helloTimer);
  if (ws) { ws.onclose = null; try { ws.close(); } catch (e) { /* closed */ } }
  ws = null;
  if (user) { setLink('off', 'Connect'); $('subtitle').textContent = 'Not connected'; }
}

function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }
function cmd(name, args = {}) {
  if (app.link === 'twin') { send({ op: 'scara', id: ++msgId, cmd: name, args }); return true; }
  if (app.link === 'rosbridge' || app.link === 'demo') return localCmd(name, args);
  toast('Connect to the PC first (tap the status at the top)');
  return false;
}

function onMessage(m) {
  if (m.op === 'scara_reply') {
    if (m.cmd === 'hello') {
      clearTimeout(helloTimer);
      if (m.scene) { scene = Object.assign({}, SC, m.scene); cellKind = ''; }
      ghost.base.position.set(...(scene.base_xyz || [0.2, 0, 0.4]));
      setLink('twin', 'Connected');
      fillPorts(m.ports || []);
      toast('Connected to SCARA Twin Control');
    } else if (m.cmd === 'ports') {
      fillPorts(m.ports || []);
    } else if (!m.ok && m.error) {
      toast(m.error);
    }
    return;
  }
  if (m.op === 'scara_state') { onState(m.state); return; }
  if (m.op === 'publish' && m.topic === '/joint_states') {
    const msg = m.msg;
    msg.name.forEach((n, i) => { if (n in app.q) app.q[n] = msg.position[i]; });
    if (!app.target) app.target = { ...app.q };
  }
}

/* plain rosbridge: publish trajectories / topics ourselves */
const ROS_TOPICS = [
  ['/arm_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory'],
  ['/gripper_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory'],
  ['/scara/pick_color', 'std_msgs/String'],
];
function startRosbridge() {
  if (app.link !== 'wait') return;
  for (const [topic, type] of ROS_TOPICS) send({ op: 'advertise', topic, type });
  send({ op: 'subscribe', topic: '/joint_states', type: 'sensor_msgs/JointState', throttle_rate: 100 });
  app.target = null;
  setLink('rosbridge', 'ROS link');
  $('subtitle').textContent = 'rosbridge - sticks, gripper and colours only';
}
function rosTraj(topic, names, pos) {
  send({ op: 'publish', topic, msg: { joint_names: names, points: [{ positions: pos, time_from_start: { sec: 0, nanosec: 200000000 } }] } });
}

/* offline / rosbridge: we keep the targets ourselves */
function simCmd(name, args) {
  const S = app.sim, t = S.target;
  switch (name) {
    case 'jog_by':
      if (S.running) { toast('The cell is running - press Stop on the Auto tab to drive the arm yourself'); return false; }
      if (S.auto && S.auto.manualPlan) S.auto = null;
      for (const [j, d] of Object.entries(args.deltas || {})) t[j] = clamp(t[j] + d, ...limits(j));
      return true;
    case 'gripper':
      t.left_finger_joint = 'value' in args ? args.value : (args.close ? (scene.grip_close || -0.025) : 0); return true;
    case 'move':
      if (S.running) { toast('The cell is running - press Stop on the Auto tab to drive the arm yourself'); return false; }
      if (S.auto && S.auto.manualPlan) S.auto = null;
      Object.assign(t, args.q); return true;
    case 'teach_add': {
      const p = {}; for (const j of JOINTS) p[j] = +S.target[j].toFixed(4);
      app.teachLocal.push(p); store.set('teach', app.teachLocal); showTeach(app.teachLocal, false);
      toast(`Saved pose ${app.teachLocal.length}`); return true;
    }
    case 'teach_delete': app.teachLocal.splice(args.index, 1); store.set('teach', app.teachLocal); showTeach(app.teachLocal, false); return true;
    case 'teach_clear': app.teachLocal = []; store.set('teach', []); showTeach([], false); return true;
    case 'teach_goto': if (app.teachLocal[args.index]) Object.assign(t, app.teachLocal[args.index]); return true;
    case 'teach_play': {
      if (S.running) { toast('Stop the cell first'); return false; }
      if (!app.teachLocal.length) { toast('Save some poses first'); return false; }
      const one = [];
      for (const p of app.teachLocal) { one.push({ grip: p.left_finger_joint, wait: 0.3 }); one.push({ q: { column_joint: p.column_joint, shoulder_joint: p.shoulder_joint, forearm_joint: p.forearm_joint, wrist_joint: p.wrist_joint } }); }
      S.auto = { phase: 'plan', plan: one.map((x) => ({ ...x })), wait: 0, manualPlan: true, loop: args.loop ? one : null, label: 'Replay finished' };
      return true;
    }
    case 'teach_stop': if (S.auto && S.auto.manualPlan) S.auto = null; return true;
    case 'home': if (!S.running) S.auto = { phase: 'plan', plan: S.planReady(), wait: 0, manualPlan: true, label: 'Home' }; return true;
    case 'pick': {
      const txt = String(args.color).trim().toLowerCase();
      if (txt === 'start') return simCmd('conveyor', { run: true });
      if (txt === 'stop') return simCmd('conveyor', { run: false });
      if (S.world !== 'command') { setWorld('command'); }
      S.request(txt); return true;
    }
    case 'conveyor':
      if (args.run) {
        if (S.world === 'manual') setWorld('sort');
        S.start();
      } else S.stop();
      return true;
    case 'estop': S.stop(); Object.assign(t, S.q); toast('Stopped'); return true;
    case 'release': return true;
    case 'pick_place': S.pickPlace({ x: args.x, y: args.y }, { x: args.to_x, y: args.to_y }); return true;
    case 'safety': S.mode = args.mode; document.querySelectorAll('#safety button').forEach((b) => b.classList.toggle('on', b.dataset.v === S.mode)); return true;
    case 'speed': S.scale = args.scale; return true;
    default:
      toast('Offline mode: teaching and the real robot need the PC');
      return false;
  }
}

function setWorld(w) {
  if (app.link === 'twin') {
    const sim = (app.st && app.st.sim) || {};
    if (!sim.worlds) { toast(sim.kind === 'hardware' ? 'The real camera cell has no worlds' : 'This simulation has only one world'); return; }
    if (sim.switching) { toast(`Still starting the ${worldName(sim.switching)} world - one moment`); return; }
    if ((sim.world || 'manual') === w) return;
    app.pucks = [];                       // nothing of the old world stays on the screen
    app.pendingWorld = { w, t: performance.now() };
    showBusy(`Starting the ${worldName(w)} world…` + (sim.kind === 'gazebo' ? ' (Gazebo restarts, about 30 s)' : ''));
    cmd('world', { world: w });
    return;
  }
  if (app.link !== 'demo') return;
  app.sim.setWorld(w);
  app.pucks = app.sim.pucks;
  buildCell(w);
  showBusy('');
  document.querySelectorAll('#world button').forEach((b) => b.classList.toggle('on', b.dataset.w === w));
}

function localCmd(name, args) {
  if (app.link === 'demo') return simCmd(name, args);
  const t = app.target || (app.target = { ...app.q });
  if (name === 'jog_by') {
    for (const [j, d] of Object.entries(args.deltas || {})) t[j] = clamp(t[j] + d, ...limits(j));
  } else if (name === 'move') {
    Object.assign(t, args.q);
  } else if (name === 'gripper') {
    t.left_finger_joint = args.close ? (scene.grip_close || -0.025) : 0;
  } else if (name === 'home') {
    Object.assign(t, { column_joint: 1.9, shoulder_joint: scene.z_travel || 0.015, forearm_joint: 1.8, wrist_joint: 0, left_finger_joint: 0 });
  } else if (name === 'pick') {
    if (app.link === 'rosbridge') send({ op: 'publish', topic: '/scara/pick_color', msg: { data: args.color } });
    else toast('Colours need the conveyor cell on the PC');
    return true;
  } else if (name === 'conveyor') {
    if (app.link === 'rosbridge') send({ op: 'publish', topic: '/scara/pick_color', msg: { data: args.run ? 'start' : 'stop' } });
    return true;
  } else if (name === 'estop') {
    Object.assign(t, app.q);
    toast('Stopped');
    return true;
  } else if (name === 'release') {
    return true;
  } else {
    toast(app.link === 'demo' ? 'Offline mode: connect to the PC for this' : 'Needs SCARA Twin Control on the PC (not plain rosbridge)');
    return false;
  }
  if (app.link === 'rosbridge') {
    rosTraj('/arm_controller/joint_trajectory', ARM, ARM.map((j) => t[j]));
    rosTraj('/gripper_controller/joint_trajectory', ['left_finger_joint', 'right_finger_joint'], [t.left_finger_joint, t.left_finger_joint]);
  }
  return true;
}

function startDemo() {
  disconnect(true);
  scene = SC;
  app.sim = new window.LocalSim(SC, (text, level) => pushLog({ text, level: level || 'info' }));
  app.sim.scale = app.speed;
  app.target = app.sim.target;
  setLink('demo', 'Offline');
  $('subtitle').textContent = 'Offline - everything runs on this phone';
  setWorld('manual');
  document.querySelectorAll('#safety button').forEach((b) => b.classList.toggle('on', b.dataset.v === app.sim.mode));
}

/* the newest 2 messages over the 3-D view (they fade after 7 s), and the full list in the Robot tab */
function pushLog(e) {
  e = { ...e, at: performance.now() };
  logLines.push(e);
  if (logLines.length > 2) logLines.shift();
  drawTicker();
  const box = $('log');
  const d = document.createElement('div');
  d.className = e.level || 'info';
  d.innerHTML = `<time>${escapeHtml(e.t || new Date().toTimeString().slice(0, 8))}</time>${escapeHtml(e.text)}`;
  box.appendChild(d);
  while (box.children.length > 120) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}
function drawTicker() {
  const now = performance.now();
  while (logLines.length && now - logLines[0].at > 9000) logLines.shift();
  $('ticker').innerHTML = logLines.map((x) => `<div class="${x.level}${now - x.at > 7000 ? ' old' : ''}">${escapeHtml(x.text)}</div>`).join('');
}
setInterval(drawTicker, 1000);

const worldName = (w) => ({ manual: 'table', sort: 'sorting', command: 'command' }[w] || w);
function showBusy(text) { $('busy').textContent = text || ''; $('busy').hidden = !text; }

/* the Auto tab, from the PC's state or the offline simulator */
function showCell(c) {
  const active = c.active || [];
  $('active').innerHTML = active.length ? active.map(chip).join('') : '<span class="note">nothing - pick a colour</span>';
  document.querySelectorAll('#colors button').forEach((b) => b.classList.toggle('active', active.includes(b.dataset.c)));
  const seq = c.sequence || [];
  if (!app.seqBuild) $('seq').innerHTML = seq.length ? seq.map(chip).join('') : '<span class="note">none</span>';
  $('totals').textContent = c.sorted ? Object.entries(c.sorted).map(([k, n]) => `${k[0].toUpperCase()}${n}`).join('  ') : '';
  $('cellNote').textContent = c.note;
  document.querySelectorAll('#world button').forEach((b) => b.classList.toggle('on', b.dataset.w === c.world));
  $('beltRun').classList.toggle('on', !!c.running);
  $('beltRun').textContent = c.running ? 'Running' : 'Start';
}

/* ------------------------------------------------------------------ state from the PC */
function onState(st) {
  app.st = st;
  const sim = st.sim || {}, rob = st.robot || {};
  const ctl = st.control;
  const simJ = sim.joints, realJ = rob.joints;
  const shown = (ctl === 'robot' && realJ) ? realJ : (simJ || realJ || st.cmd);
  if (shown) Object.assign(app.q, shown);
  app.real = realJ;
  const twin = rob.connected && rob.zeroed && simJ && realJ && ctl !== 'sim' && sim.kind !== 'hardware';   // hardware: sim IS the robot
  // the real robot as a yellow see-through arm - only where it is not where the sim is (when they
  // agree there is nothing to show, and two arms in the same place would just flicker)
  const apart = twin && ARM.some((j) => Math.abs((realJ[j] || 0) - (simJ[j] || 0)) > (j === 'shoulder_joint' ? 0.002 : 0.015));
  ghost.base.visible = !!apart;
  app.twinOn = !!twin;
  if (apart) ghost.set(realJ);
  const curWorld = sim.world || sim.cell_mode || (sim.kind === 'gazebo' && sim.has_belt ? 'sort' : 'manual');
  if (sim.switching) {
    app.pucks = [];
    showBusy(`Starting the ${worldName(sim.switching)} world…` + (sim.kind === 'gazebo' ? ' (Gazebo restarts, about 30 s)' : ''));
  } else if (app.pendingWorld) {
    if (curWorld === app.pendingWorld.w || performance.now() - app.pendingWorld.t > 8000) { app.pendingWorld = null; showBusy(''); }
    app.pucks = [];
  } else {
    showBusy('');
    app.pucks = sim.pucks || [];
  }
  if (sim.kind === 'hardware') {
    app.hwLayout = sim.layout || {};
    buildCell('hardware:' + JSON.stringify([app.hwLayout.pick, app.hwLayout.dir, app.hwLayout.drops]));
  } else buildCell(sim.switching || (app.pendingWorld && app.pendingWorld.w) || curWorld);

  // header + target switch
  $('simLabel').textContent = { none: 'none running', mujoco: 'MuJoCo', gazebo: 'Gazebo', hardware: 'camera cell (real)' }[sim.kind] || sim.kind || '';
  $('robotLabel').textContent = rob.connected ? (rob.answering === false ? 'NOT ANSWERING' : rob.zeroed ? 'ready' : 'home it first') : 'not connected';
  document.querySelector('#target [data-target="robot"]').classList.toggle('alarm', rob.connected && rob.answering === false);
  document.querySelectorAll('#target button').forEach((b) => b.classList.toggle('on', b.dataset.target === ctl));
  $('subtitle').textContent = sim.autonomous ? 'Cell is running by itself' : ({ sim: 'Moving the simulation', robot: 'Moving the real robot', both: 'Digital twin: robot copies the sim' }[ctl] || '');

  // stop button
  const es = st.safety && st.safety.estop;
  $('estop').classList.toggle('latched', !!es);
  $('estop').textContent = es ? 'RELEASE' : 'STOP';

  // safety + speed
  if (st.safety) {
    document.querySelectorAll('#safety button').forEach((b) => b.classList.toggle('on', b.dataset.v === st.safety.mode));
  }

  // cell
  const world = sim.switching || curWorld;
  app.pcWorlds = !!sim.worlds && !sim.switching;
  showCell({ active: sim.active, sequence: sim.sequence, sorted: sim.sorted, world, running: sim.autonomous || (st.belt && st.belt.on),
    note: sim.commands ? 'Command mode on the PC: a colour stays active until you pick another one.'
      : world === 'sort' ? 'Sorting mode on the PC: every colour goes to its own bin. Start / Stop runs the belt.'
        : sim.worlds ? 'MuJoCo on the PC: choose Sorting or Command above to run the conveyor cell.'
          : 'The PC runs the table (manual). Choose Sorting or Command above to start that cell.' });

  // teach
  showTeach((st.teach && st.teach.points) || [], st.teach && st.teach.playing);

  // robot tab
  $('rNote').textContent = !rob.connected ? 'Plug the robot into the PC running SCARA Twin Control, pick its port and connect.'
    : `${rob.port}: ${rob.zeroed ? 'zeroed' : 'not homed - press Home switches (or Zero here with the arm at the ready pose)'}`
      + `${rob.motors_on === false ? ', motors free' : ''}${(rob.out_of_range || []).length ? ', target out of range: ' + rob.out_of_range.join(', ') : ''}`;

  // real camera cell: which poses are taught
  const cp = st.cell_poses;
  $('cellTeach').hidden = !cp;
  if (cp) document.querySelectorAll('#teachBtns button').forEach((b) => b.classList.toggle('done', cp.includes(b.dataset.what)));

  // log ticker: the last few lines
  for (const e of st.log || []) {
    app.lastLogId = Math.max(app.lastLogId, e.id);
    pushLog(e);
    if (e.level === 'error') toast(e.text);
  }
  refreshEnabled();
}
const logLines = [];
let teachKey = '';
function showTeach(pts, playing) {
  const key = JSON.stringify(pts) + playing;
  if (key === teachKey) return;
  teachKey = key;
  $('tList').innerHTML = pts.map((p, i) => `<div class="item"><span>${i + 1}. ${deg(p.column_joint || 0)} · ${((p.shoulder_joint || 0) * 1000).toFixed(0)} mm · ${deg(p.forearm_joint || 0)}${(p.left_finger_joint || 0) < -0.005 ? ' · closed' : ''}</span>`
    + `<button data-go="${i}">Go</button><button data-del="${i}">Delete</button></div>`).join('');
  $('tNote').textContent = playing ? 'Playing…' : (pts.length ? `${pts.length} saved poses` : 'Move the arm, save poses, then play them back in order.');
}
function escapeHtml(s) { return String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]); }
function chip(c) {
  const bg = { red: '#d63a32', green: '#2f9e44', blue: '#1c6ed6', yellow: '#e6b400', any: '#1d2730' }[c] || '#55616c';
  return `<span class="chip" style="background:${bg}">${c === 'any' ? 'every colour' : c}</span>`;
}

function refreshEnabled() {
  const twin = app.link === 'twin';
  const st = app.st || {};
  const rob = st.robot || {};
  const demo = app.link === 'demo';
  document.querySelectorAll('[data-tab="robot"] button, #target button').forEach((b) => { b.disabled = !twin; });
  document.querySelectorAll('[data-tab="teach"] button').forEach((b) => { b.disabled = !(twin || demo); });
  $('offlineCell').hidden = !demo;
  document.querySelectorAll('#safety button, #tapPick').forEach((b) => { b.disabled = !(twin || demo); });
  document.querySelectorAll('#world button').forEach((b) => { b.disabled = !(demo || (twin && app.pcWorlds)); });
  if (twin) {
    $('rZero').disabled = $('rFree').disabled = $('rLock').disabled = $('rHome').disabled = $('rDisc').disabled = !rob.connected;
    document.querySelector('#target [data-target="robot"]').disabled = !rob.connected;
    document.querySelector('#target [data-target="both"]').disabled = !rob.connected || (st.sim && st.sim.kind === 'none');
    document.querySelector('#target [data-target="sim"]').disabled = st.sim && (st.sim.kind === 'none' || st.sim.kind === 'hardware');
    if (st.sim && st.sim.kind === 'hardware') document.querySelector('#target [data-target="both"]').disabled = true;
  }
  $('seqRun').disabled = !app.seq.length;
}

/* ------------------------------------------------------------------ joysticks */
function stick(el) {
  const knob = el.querySelector('.knob');
  const s = { x: 0, y: 0, id: null };
  const move = (e) => {
    const r = el.getBoundingClientRect();
    let dx = (e.clientX - r.left - r.width / 2) / (r.width / 2 * 0.72);
    let dy = (e.clientY - r.top - r.height / 2) / (r.height / 2 * 0.72);
    const n = Math.hypot(dx, dy);
    if (n > 1) { dx /= n; dy /= n; }
    s.x = dx; s.y = -dy;
    knob.style.transform = `translate(calc(-50% + ${dx * r.width * 0.36}px), calc(-50% + ${dy * r.height * 0.36}px))`;
  };
  el.addEventListener('pointerdown', (e) => { s.id = e.pointerId; el.setPointerCapture(e.pointerId); move(e); });
  el.addEventListener('pointermove', (e) => { if (e.pointerId === s.id) move(e); });
  const up = (e) => { if (e.pointerId !== s.id) return; s.id = null; s.x = s.y = 0; knob.style.transform = 'translate(-50%, -50%)'; };
  el.addEventListener('pointerup', up);
  el.addEventListener('pointercancel', up);
  return s;
}
const L = stick($('stickL')), R = stick($('stickR'));
const dead = (v) => (Math.abs(v) < 0.12 ? 0 : (v - Math.sign(v) * 0.12) / 0.88);
let lastTick = performance.now(), idleSince = 0;

function currentTarget() {
  if (app.link === 'demo' && app.sim) return app.sim.target;
  if (app.link === 'twin' && app.st && app.st.targets) {
    // the real robot alone and not homed yet: nothing is sent to it, so show where it really is
    const r = app.st.robot || {};
    if (app.st.control === 'robot' && r.connected && !r.zeroed && r.joints) return { ...app.st.targets, ...r.joints };
    return app.st.targets;
  }
  return app.target || app.q;
}

/* the left stick as XY on the table, relative to how you look at the 3-D view */
function viewAxes() {
  const f = new THREE.Vector3().subVectors(controls.target, cam.position);
  f.z = 0;
  if (f.lengthSq() < 1e-6) f.set(1, 0, 0);
  f.normalize();
  return { fwd: [f.x, f.y], right: [f.y, -f.x] };
}

function tiltVec() {
  const t = app.tilt;
  if (!t || t.x === undefined) return { x: 0, y: 0 };
  return { x: clamp(t.x / 25, -1, 1), y: clamp(t.y / 25, -1, 1) };
}

setInterval(() => {
  const now = performance.now(), dt = Math.min((now - lastTick) / 1000, 0.25);   // slow phones: keep the speed
  lastTick = now;
  let lx = dead(L.x), ly = dead(L.y);
  if (app.tilt && !lx && !ly) { const tv = tiltVec(); lx = dead(tv.x); ly = dead(tv.y); }
  if (app.hand && !lx && !ly) { const h = window.ScaraHand.out; if (h.seen) { lx = handDead(h.x); ly = handDead(h.y); } }
  const kb = keyVec();
  if (kb.active) { lx = lx || kb.lx; ly = ly || kb.ly; }
  if (pad.active) { lx = lx || pad.lx; ly = ly || pad.ly; }
  const rx = dead(R.x) || kb.rx || pad.rx, ry = dead(R.y) || kb.ry || pad.ry;
  const active = lx || ly || rx || ry;
  if (!active) {
    if (app.cart && now - idleSince > 400) { app.cart = null; app.cartQ = null; }
    return;
  }
  idleSince = now;
  if (app.drive === 'xy') {
    const base = app.cartQ || currentTarget();
    if (!app.cart) {
      const f = kfk(base);
      app.cart = { x: f.x, y: f.y, z: base.shoulder_joint, yaw: f.yaw };
      app.cartQ = { column_joint: base.column_joint, shoulder_joint: base.shoulder_joint, forearm_joint: base.forearm_joint, wrist_joint: base.wrist_joint };
    }
    const c = app.cart, ax = viewAxes(), v = 0.09 * app.speed * dt;
    const nx = c.x + (lx * Math.abs(lx) * ax.right[0] + ly * Math.abs(ly) * ax.fwd[0]) * v;
    const ny = c.y + (lx * Math.abs(lx) * ax.right[1] + ly * Math.abs(ly) * ax.fwd[1]) * v;
    const nz = clamp(c.z + ry * Math.abs(ry) * RATE.shoulder_joint * app.speed * dt, ...limits('shoulder_joint'));
    const nyaw = c.yaw - rx * Math.abs(rx) * 1.2 * app.speed * dt;
    const sol = kik(nx, ny, nyaw, app.cartQ);
    if (!sol) { buzz(20); return; }                      // edge of the workspace: stay
    Object.assign(c, { x: nx, y: ny, z: nz, yaw: nyaw });
    app.cartQ = { column_joint: sol.column_joint, shoulder_joint: nz, forearm_joint: sol.forearm_joint, wrist_joint: sol.wrist_joint };
    cmd('move', { q: app.cartQ });
    return;
  }
  const v = { column_joint: lx, shoulder_joint: ly, forearm_joint: rx, wrist_joint: ry };
  const deltas = {};
  for (const j of ARM) if (v[j]) deltas[j] = v[j] * Math.abs(v[j]) * RATE[j] * app.speed * dt;
  cmd('jog_by', { deltas });
}, 50);

/* ------------------------------------------------------------------ buttons */
const on = (id, f) => $(id).addEventListener('click', f);
on('conn', () => {
  if (app.link === 'twin' || app.link === 'rosbridge') {
    if (confirm('Disconnect from the PC?')) disconnect(true);
    return;
  }
  $('ip').value = store.get('ip', '');
  $('portNo').value = store.get('port', 9090);
  $('sheet').hidden = false;
  setTimeout(() => $('ip').focus(), 50);
});
$('connForm').addEventListener('submit', (e) => {
  e.preventDefault();
  const ip = $('ip').value.trim(), port = parseInt($('portNo').value, 10) || 9090;
  if (!/^[\w.-]+$/.test(ip)) { toast('Type the PC address, e.g. 192.168.1.50'); return; }
  $('sheet').hidden = true;
  connect(ip, port);
});
on('demo', () => { $('sheet').hidden = true; startDemo(); });
on('closeSheet', () => { $('sheet').hidden = true; });

on('estop', () => {
  const latched = $('estop').classList.contains('latched');
  if (app.link === 'demo' || app.link === 'rosbridge') {
    localCmd(latched ? 'release' : 'estop', {});
    $('estop').classList.toggle('latched', !latched);
    $('estop').textContent = latched ? 'STOP' : 'RELEASE';
    return;
  }
  cmd(latched ? 'release' : 'estop');
});
document.querySelectorAll('#target button').forEach((b) => b.addEventListener('click', () => cmd('control', { mode: b.dataset.target })));
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('on', t.dataset.tab === b.dataset.tab));
  store.set('tab', b.dataset.tab);
}));
on('gOpen', () => cmd('gripper', { close: false }));
on('gClose', () => cmd('gripper', { close: true }));
on('home', () => cmd('home'));
$('speed').value = Math.round(app.speed * 100);
$('speedVal').textContent = `${Math.round(app.speed * 100)} %`;
$('speed').addEventListener('input', () => {
  app.speed = $('speed').value / 100;
  $('speedVal').textContent = `${$('speed').value} %`;
  store.set('speed', +$('speed').value);
  if (app.sim && app.link === 'demo') app.sim.scale = app.speed;
});
$('speed').addEventListener('change', () => { if (app.link === 'twin') cmd('speed', { scale: app.speed }); });
document.querySelectorAll('#safety button').forEach((b) => b.addEventListener('click', () => {
  if (b.dataset.v === 'off' && !confirm('Turn off speed limits and the pick assist? Only do this if you know the arm is clear.')) return;
  cmd('safety', { mode: b.dataset.v });
}));

// tap to move a puck: tap the puck, then tap where it should go
on('tapPick', () => {
  if (app.kinTap) { app.kinTap = false; $('kTap').classList.remove('on'); }
  app.tapMode = !app.tapMode;
  app.picked = null;
  $('tapPick').classList.toggle('on', app.tapMode);
  hint(app.tapMode ? 'Tap a puck' : '');
});
function hint(t) { $('hint').textContent = t; $('hint').hidden = !t; }
const ray = new THREE.Raycaster();
let downAt = null;
canvas.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', (e) => {
  if (app.kinTap && downAt && Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) <= 8) {
    const r0 = canvas.getBoundingClientRect(), p = new THREE.Vector3();
    ray.setFromCamera(new THREE.Vector2((e.clientX - r0.left) / r0.width * 2 - 1, -(e.clientY - r0.top) / r0.height * 2 + 1), cam);
    if (ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -(scene.table_top || 0.4)), p)) kinTapAt(p);
    return;
  }
  if (!app.tapMode || !downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 8) return;
  const r = canvas.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), cam);
  if (!app.picked) {
    const hit = ray.intersectObjects(puckGroup.children, true)[0];
    if (!hit) { hint('Tap a puck (it turns bright when picked)'); return; }
    let o = hit.object; while (o.parent !== puckGroup) o = o.parent;
    app.picked = o.userData;
    hint('Now tap where it should go');
  } else {
    const p = new THREE.Vector3();
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -(scene.table_top || 0.4));
    if (!ray.ray.intersectPlane(plane, p)) return;
    cmd('pick_place', { x: app.picked.x, y: app.picked.y, to_x: p.x, to_y: p.y });
    app.picked = null; app.tapMode = false;
    $('tapPick').classList.remove('on');
    hint('');
  }
});

// cell: colours, sequences, belt
document.querySelectorAll('#colors button').forEach((b) => b.addEventListener('click', () => {
  const c = b.dataset.c;
  if (app.seqBuild) {
    if (c === 'any' || c === 'none') return;
    app.seq.push(c);
    $('seq').innerHTML = app.seq.map(chip).join('');
    refreshEnabled();
    return;
  }
  cmd('pick', { color: c });
}));
on('seqMode', () => {
  app.seqBuild = !app.seqBuild;
  if (app.seqBuild) { app.seq = []; $('seq').innerHTML = '<span class="note">tap colours in order</span>'; }
  $('seqMode').classList.toggle('on', app.seqBuild);
  $('seqMode').textContent = app.seqBuild ? 'Cancel' : 'Build a sequence';
  refreshEnabled();
});
on('seqRun', () => {
  if (!app.seq.length) return;
  cmd('pick', { color: 'sequence ' + app.seq.join(' ') });
  app.seqBuild = false; app.seq = [];
  $('seqMode').classList.remove('on'); $('seqMode').textContent = 'Build a sequence';
  refreshEnabled();
});
on('beltRun', () => cmd('conveyor', { run: true }));
on('beltStop', () => cmd('conveyor', { run: false }));
function setDrive(d) {
  app.drive = d; store.set('drive', d); app.cart = null; app.cartQ = null;
  document.querySelectorAll('#drive button').forEach((b) => b.classList.toggle('on', b.dataset.d === d));
  document.querySelectorAll('.stick .cap')[0].innerHTML = d === 'xy' ? 'Move over the table' : 'Column ↔ &nbsp; Height ↕';
  document.querySelectorAll('.stick .cap')[1].innerHTML = d === 'xy' ? 'Wrist ↔ &nbsp; Height ↕' : 'Elbow ↔ &nbsp; Wrist ↕';
}
document.querySelectorAll('#drive button').forEach((b) => b.addEventListener('click', () => setDrive(b.dataset.d)));
setDrive(app.drive);
function onTilt(e) {
  if (!app.tilt || e.beta === null) return;
  if (app.tilt.beta0 === undefined) { app.tilt.beta0 = e.beta; app.tilt.gamma0 = e.gamma; }
  app.tilt.x = e.gamma - app.tilt.gamma0;
  app.tilt.y = -(e.beta - app.tilt.beta0);
}
on('tilt', () => {
  if (app.tilt) { app.tilt = null; window.removeEventListener('deviceorientation', onTilt); $('tilt').classList.remove('on'); return; }
  const go = () => { app.tilt = {}; window.addEventListener('deviceorientation', onTilt); $('tilt').classList.add('on'); toast('Tilt drive: hold the phone the way it is now - tilting it moves the arm'); };
  if (window.DeviceOrientationEvent && typeof DeviceOrientationEvent.requestPermission === 'function') {
    DeviceOrientationEvent.requestPermission().then((r) => (r === 'granted' ? go() : toast('Tilt needs motion access'))).catch(() => toast('Tilt needs motion access'));
  } else go();
});
$('gripFine').addEventListener('input', () => {
  const pct = +$('gripFine').value;
  $('gripVal').textContent = pct ? `${pct} %` : 'open';
  cmd('gripper', { value: (scene.grip_close || -0.025) * pct / 100 });
});
$('cmdForm').addEventListener('submit', (e) => {
  e.preventDefault();
  const t = $('cmdText').value.trim();
  if (!t) return;
  const low = t.toLowerCase();
  if (low === 'start' || low === 'run') cmd('conveyor', { run: true });
  else if (low === 'stop') cmd('conveyor', { run: false });
  else cmd('pick', { color: t });
  $('cmdText').value = '';
  $('cmdText').blur();
});
on('addPuck', () => {
  const S = app.sim; if (!S) return;
  const cs = ['red', 'green', 'blue', 'yellow'], c = cs[Math.floor(Math.random() * 4)];
  if (S.world === 'manual') {
    for (let k = 0; k < 40; k++) {
      const a = Math.random() * Math.PI * 2, r = 0.2 + Math.random() * 0.12;
      const x = SC.kin.column[0] + r * Math.cos(a), y = SC.kin.column[1] + r * Math.sin(a);
      if (x > -0.3 && x < 0.38 && Math.abs(y) < 0.5 && S.pucks.every((p) => Math.hypot(p.x - x, p.y - y) > 0.12)) {
        S.pucks.push({ c, x, y, z: SC.table_top, id: ++S.n }); return;
      }
    }
    toast('No free spot on the table');
  } else { S.nextSpawn = 0; }
});
on('resetWorld', () => { if (app.sim) { setWorld(app.sim.world); app.firstPick = null; } });
on('help', () => { $('helpSheet').hidden = false; });
on('closeHelp', () => {
  $('helpSheet').hidden = true;
  store.set('helpSeen', true);
  if (app.link === 'off' && !store.get('ip', '')) $('conn').click();
});
on('vFollow', () => { app.follow = !app.follow; $('vFollow').classList.toggle('on', app.follow); });
document.addEventListener('click', (e) => { if (e.target.closest('button')) buzz(e.target.closest('#estop') ? 80 : 12); }, true);
document.querySelectorAll('#world button').forEach((b) => b.addEventListener('click', () => setWorld(b.dataset.w)));

// teach
on('tSave', () => cmd('teach_add'));
on('tPlay', () => cmd('teach_play', { loop: false }));
on('tLoop', () => cmd('teach_play', { loop: true }));
on('tStop', () => cmd('teach_stop'));
on('tClear', () => { if (confirm('Delete all saved poses?')) cmd('teach_clear'); });
$('tList').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  if (b.dataset.go) cmd('teach_goto', { index: +b.dataset.go });
  if (b.dataset.del) cmd('teach_delete', { index: +b.dataset.del });
});

// robot
function fillPorts(ports) {
  const sel = $('port'), keep = sel.value;
  sel.innerHTML = '<option value="">Serial port…</option>' + ports.map((p) => `<option>${escapeHtml(p)}</option>`).join('');
  if (ports.includes(keep)) sel.value = keep;
  else if (ports.length === 1) sel.value = ports[0];
}
on('ports', () => send({ op: 'scara', id: ++msgId, cmd: 'ports' }));
on('rConnect', () => {
  if (!$('port').value) { toast('Pick the robot\'s serial port first (Refresh after plugging it in)'); return; }
  cmd('connect', { port: $('port').value });
});
on('rFake', () => cmd('connect', { fake: true }));
document.querySelectorAll('#dcheck [data-nudge]').forEach((b) => b.addEventListener('click', () => cmd('dir_nudge', { joint: b.dataset.nudge })));
document.querySelectorAll('#dcheck [data-flip]').forEach((b) => b.addEventListener('click', () => {
  if (confirm('Reverse the direction of this joint? Only how its position is read changes - the arm does not move.')) cmd('dir_flip', { joint: b.dataset.flip });
}));
$('teachBtns').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b) cmd('cell_teach', { what: b.dataset.what });
});
on('rDisc', () => cmd('disconnect'));
on('rZero', () => cmd('zero_here'));
on('rHome', () => cmd('home_switches'));
on('rFree', () => cmd('free'));
on('rLock', () => cmd('lock'));

// view buttons
on('vHome', () => setView('front'));
on('vTop', () => setView('top'));
on('vCam', () => {
  showInset = !showInset;
  store.set('inset', showInset);
  $('vCam').classList.toggle('on', showInset);
  resize();
});
$('vCam').classList.toggle('on', showInset);

const lastTab = store.get('tab', 'manual');
const tb = document.querySelector(`#tabs [data-tab="${lastTab}"]`);
if (tb) tb.click();


/* ------------------------------------------------------------------ v3: voice */
const VOICE_HELP = 'Say e.g. "red", "red then blue", "3 green", "any", "start", "stop", "open", "close", "home", "sorting"';
function micState(st) {
  const on = st === 'listening';
  $('mic').classList.toggle('on', on); $('mic2').classList.toggle('on', on);
  $('mic2').textContent = on ? 'Listening…' : 'Speak';
  const msg = { 'no-recognizer': 'No speech recogniser on this phone - install or enable "Speech Recognition & Synthesis" by Google',
    unsupported: 'Voice needs the Android app, or Chrome / Edge on a computer - you can type the command instead',
    denied: 'The browser blocked the microphone - allow it for this page',
    network: 'Browser voice needs internet - or use the Android app (offline speech)',
    nothing: 'Did not catch that. ' + VOICE_HELP, error: 'Voice did not start - try again',
    listening: 'Listening… ' + VOICE_HELP }[st];
  if (msg) toast(msg);
}
function voice() {
  if (!window.ScaraVoice) return;
  window.ScaraVoice.listen((text) => runSpoken(text), micState);
}
function runSpoken(text) {
  const acts = window.ScaraVoice.parse(text);
  if (!acts.length) { toast(`Heard "${text}" - ${VOICE_HELP}`); return; }
  pushLog({ text: `Voice: "${text}"`, level: 'info' });
  for (const a of acts) {
    if (a.kind === 'pick') {
      if (app.link === 'demo' && app.sim && app.sim.world !== 'command') setWorld('command');
      cmd('pick', { color: a.text });
      toast(a.text === 'none' ? 'Stop picking' : `Picking: ${a.text}`);
    } else if (a.kind === 'belt') { cmd('conveyor', { run: a.run }); toast(a.run ? 'Belt on' : 'Belt off'); }
    else if (a.kind === 'gripper') { cmd('gripper', { close: a.close }); toast(a.close ? 'Gripper closing' : 'Gripper opening'); }
    else if (a.kind === 'home') { cmd('home'); toast('Going home'); }
    else if (a.kind === 'world') { setWorld(a.world); toast(`World: ${worldName(a.world)}`); }
    else if (a.kind === 'estop') { if (!$('estop').classList.contains('latched')) $('estop').click(); }
    else if (a.kind === 'release') { if ($('estop').classList.contains('latched')) $('estop').click(); }
  }
}
on('mic', voice);
on('mic2', voice);

/* ------------------------------------------------------------------ v3: hand control */
const handDead = (v) => (Math.abs(v) < 0.18 ? 0 : Math.sign(v) * Math.min(1, (Math.abs(v) - 0.18) / 0.62));
let handFist = null;
function handStatus(t) { $('handText').textContent = t; }
async function handOn() {
  // Android: the app needs the camera permission first (asked once)
  if (window.ScaraNative && window.ScaraNative.hasCamera && !window.ScaraNative.hasCamera()) {
    window.onNativePermission = (ok) => { if (ok) handOn(); else toast('Gesture control needs the camera - allow it in Android Settings > Apps > SCARA Digital Twin > Permissions'); };
    window.ScaraNative.askCamera();
    return;
  }
  try {
    camShow(true);
    handStatus('Starting the camera…');
    $('hand').classList.add('on'); $('vHand').classList.add('on');
    await Promise.race([window.ScaraHand.start($('handVideo'), $('handView'), handStatus),
      new Promise((_, no) => setTimeout(() => no(new Error('the camera did not start - close other apps that use it and try again')), 12000))]);
    app.hand = true; handFist = null;
    toast('Gesture control: hold your open hand in the yellow square for 2 s');
    if (app.tilt) $('tilt').click();
  } catch (e) {
    window.ScaraHand.stop();
    camShow(false);
    $('hand').classList.remove('on'); $('vHand').classList.remove('on');
    toast(location.protocol === 'http:' && !['localhost', '127.0.0.1'].includes(location.hostname)
      ? 'The browser only allows the camera on http://localhost (or https) - use the PC itself or the Android app'
      : 'Camera not available: ' + (e.message || e.name || e));
  }
}
function handOff() {
  window.ScaraHand.stop();
  app.hand = false;
  $('hand').classList.remove('on'); $('vHand').classList.remove('on');
  camShow(false);
}
const handToggle = () => (app.hand || $('hand').classList.contains('on') ? handOff() : handOn());
on('hand', handToggle);
on('vHand', handToggle);
on('handCal', () => window.ScaraHand.calibrate());
on('camClose', () => handOff());

/* The gesture camera, like the camera window of Gazebo / MuJoCo - it never squeezes the robot view:
   Docked - its own place: above the robot on a phone, top of the right column on a tablet or PC
            (drag the bar under the picture to make it taller or shorter)
   Window - floats over the page; drag the title bar, resize from the corner
   Small  - a preview in the robot view's bottom-right corner; tap it to dock it again
   x      - closes the camera and turns gesture control off. Mode, size and window place are remembered. */
const screenEl = $('screen'), camEl = $('handCam'), appEl = document.querySelector('.app');
let camMode = store.get('camMode2', 'dock');
if (!['dock', 'float', 'mini'].includes(camMode)) camMode = 'dock';
let camH = +store.get('camH', 0) || 0;                      // docked height in px (0 = the layout's default)
let camRect = Object.assign({ x: 0.55, y: 0.12, w: 0.36, h: 0.42 }, store.get('camRect2', {}) || {});
function camApply() {
  const shown = !camEl.hidden;
  camEl.dataset.cm = camMode;
  appEl.classList.toggle('cam-dock', shown && camMode === 'dock');
  document.body.classList.toggle('cam-dock', shown && camMode === 'dock');
  if (camH) appEl.style.setProperty('--camh', camH + 'px'); else appEl.style.removeProperty('--camh');
  const vw = innerWidth, vh = innerHeight;
  camEl.style.left = camEl.style.top = camEl.style.width = camEl.style.height = '';
  if (shown && camMode === 'float') {
    const w = clamp(camRect.w * vw, 180, vw), h = clamp(camRect.h * vh, 150, vh);
    camEl.style.width = w + 'px'; camEl.style.height = h + 'px';
    camEl.style.left = clamp(camRect.x * vw, 0, vw - w) + 'px'; camEl.style.top = clamp(camRect.y * vh, 0, vh - h) + 'px';
  } else if (shown && camMode === 'mini') {                    // bottom-right of the robot view, clear of its buttons
    const r = screenEl.getBoundingClientRect(), w = clamp(vw * 0.22, 130, 210), h = w * 0.9;
    camEl.style.left = Math.max(0, r.right - w - 10) + 'px'; camEl.style.top = Math.max(0, r.bottom - h - 10) + 'px';
  }
  document.querySelectorAll('#camMode button').forEach((b) => b.classList.toggle('on', b.dataset.cm === camMode));
  requestAnimationFrame(resize);
}
function camShow(v) { camEl.hidden = !v; camApply(); }
function setCamMode(m) { camMode = m; store.set('camMode2', m); camApply(); }
document.querySelectorAll('#camMode button').forEach((b) => b.addEventListener('click', () => setCamMode(b.dataset.cm)));
$('camView').addEventListener('click', () => { if (camMode === 'mini') setCamMode('dock'); });
$('camView').addEventListener('dblclick', () => { if (camMode !== 'mini') window.ScaraHand.calibrate(); });
window.addEventListener('resize', () => { if (!camEl.hidden) camApply(); });
new ResizeObserver(() => { if (!camEl.hidden && camMode === 'mini') camApply(); }).observe(screenEl);
function camDrag(el, onMove, onEnd) {        // pointer drag (mouse, finger, pen)
  el.addEventListener('pointerdown', (e) => {
    if (e.target.closest('button')) return;
    e.preventDefault(); el.setPointerCapture(e.pointerId); el.classList.add('drag');
    const x0 = e.clientX, y0 = e.clientY, r0 = camEl.getBoundingClientRect();
    const mv = (ev) => onMove(ev.clientX - x0, ev.clientY - y0, r0);
    const up = () => { el.classList.remove('drag'); el.removeEventListener('pointermove', mv); el.removeEventListener('pointerup', up);
      el.removeEventListener('pointercancel', up); onEnd(); };
    el.addEventListener('pointermove', mv); el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up);
  });
}
const camHMax = () => Math.max(160, innerHeight * 0.7);
camDrag($('camSplit'), (dx, dy, r0) => { camH = Math.round(clamp(r0.height + dy, 140, camHMax())); camApply(); }, () => store.set('camH', camH));
$('camSplit').addEventListener('keydown', (e) => {
  const d = { ArrowUp: -20, ArrowDown: 20 }[e.key];
  if (!d) return;
  e.preventDefault(); camH = Math.round(clamp((camH || camEl.getBoundingClientRect().height) + d, 140, camHMax())); camApply(); store.set('camH', camH);
});
camDrag($('camBar'), (dx, dy, r0) => {
  if (camMode !== 'float') return;
  camRect.x = (r0.left + dx) / innerWidth; camRect.y = (r0.top + dy) / innerHeight; camApply();
}, () => store.set('camRect2', camRect));
camDrag($('camGrip'), (dx, dy, r0) => {
  camRect.w = (r0.width + dx) / innerWidth; camRect.h = (r0.height + dy) / innerHeight; camApply();
}, () => store.set('camRect2', camRect));
camApply();

/* what the hand tracker sees, drawn over the (mirrored) camera picture */
function camOverlay() {
  requestAnimationFrame(camOverlay);
  if ($('handCam').hidden || !window.ScaraHand.overlay) return;
  const cv = $('handOverlay'), vid = $('handVideo'), box = cv.getBoundingClientRect();
  const dpr = Math.min(2, window.devicePixelRatio || 1), W = Math.round(box.width * dpr), H = Math.round(box.height * dpr);
  if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
  const g = cv.getContext('2d'); g.clearRect(0, 0, W, H);
  const vw = vid.videoWidth || 4, vh = vid.videoHeight || 3, k = Math.min(W / vw, H / vh);   // object-fit: contain
  const pw = vw * k, ph = vh * k, ox = (W - pw) / 2, oy = (H - ph) / 2;
  const X = (u) => ox + (1 - u) * pw, Y = (v) => oy + v * ph;                            // mirrored like the picture
  const v = window.ScaraHand.overlay();
  g.lineWidth = 2 * dpr; g.font = `${12 * dpr}px sans-serif`;
  if (v.state === 'calibrating' || v.state === 'waiting') {
    const b = v.box || [0.35, 0.3, 0.65, 0.7];
    g.strokeStyle = '#ffc61a'; g.setLineDash([8 * dpr, 6 * dpr]);
    g.strokeRect(X(b[2]), Y(b[1]), X(b[0]) - X(b[2]), Y(b[3]) - Y(b[1])); g.setLineDash([]);
    if (v.calLeft > 0) { g.fillStyle = '#ffc61a'; g.fillText(`hold still ${v.calLeft.toFixed(1)} s`, X(b[2]), Y(b[1]) - 6 * dpr); }
  }
  if (v.home) {
    g.strokeStyle = '#ffffffaa';
    g.beginPath(); g.arc(X(v.home.x), Y(v.home.y), v.deadR * pw, 0, 7); g.stroke();             // still zone: the arm does not move
    g.strokeStyle = '#ffffff44';
    g.beginPath(); g.arc(X(v.home.x), Y(v.home.y), v.travel * pw, 0, 7); g.stroke();            // full speed at this ring
  }
  if (v.blob && v.home) {
    g.strokeStyle = '#ffc61a'; g.beginPath(); g.moveTo(X(v.home.x), Y(v.home.y)); g.lineTo(X(v.blob.x), Y(v.blob.y)); g.stroke();
  }
  if (v.blob) {
    const fist = v.out && v.out.fist;
    g.fillStyle = fist ? '#e8463a' : '#2fbf71';
    g.beginPath(); g.arc(X(v.blob.x), Y(v.blob.y), 9 * dpr, 0, 7); g.fill();
    g.fillStyle = '#fff'; g.fillText(fist ? 'fist: gripper closes' : 'open hand', X(v.blob.x) + 12 * dpr, Y(v.blob.y) + 4 * dpr);
  }
  $('handGrip').textContent = v.state === 'tracking' && v.out && v.out.fist ? 'GRIP' : '';
}
requestAnimationFrame(camOverlay);

/* External gamepads (USB / Bluetooth: Zebronics, Xbox, PlayStation, generic) on the website and the Android app.
   Left stick and right stick drive like the on-screen joysticks. Buttons:
   A / cross close the gripper, B / circle open it, X / square home, Y / triangle add a teach point,
   LB / RB slower / faster, Start emergency stop (press again to release), Select belt on / off, d-pad the same as the left stick. */
const pad = { lx: 0, ly: 0, rx: 0, ry: 0, active: false, name: '', prev: [], dev: {} };
/* Safety rules (bug 32: a phone reported a device whose triggers rest at -1; they were read as a
   right stick held over and the wrist kept turning):
   - a pad does nothing until a person uses it: a button is pressed or a stick is moved well away
     from where it rested when it was first seen;
   - every axis is measured from its resting value, and an axis that does not rest near 0
     (a trigger) is never used as a stick. */
const near0 = (v) => Math.abs(v) < 0.3;
function padState(gp) {
  let st = pad.dev[gp.index];
  if (!st || st.id !== gp.id) {
    st = pad.dev[gp.index] = { id: gp.id, rest: gp.axes.map((v) => v || 0), maxDev: gp.axes.map(() => 0),
      prevBtn: gp.buttons.map((x) => !!x.pressed), engaged: false };
  }
  return st;
}
function padAxes(gp, st) {
  const a = gp.axes, std = gp.mapping === 'standard';
  const ok = (i) => i < a.length && near0(st.rest[i]);
  const ax = (i) => (ok(i) ? (a[i] || 0) - st.rest[i] : 0);
  for (let i = 0; i < a.length; i++) st.maxDev[i] = Math.max(st.maxDev[i], Math.abs(ax(i)));
  let rX, rY;
  if (std) { rX = 2; rY = 3; } else {
    // generic pads: right stick X on axis 2 (or 3); Y is whichever of 5 / 3 / 4 the person actually moves
    rX = ok(2) ? 2 : 3;
    const cand = [5, 3, 4].filter((i) => i !== rX && ok(i));
    rY = cand.sort((p, q) => st.maxDev[q] - st.maxDev[p])[0];
    if (rY === undefined) rY = -1;
  }
  let lx = ax(0), ly = -ax(1), rx = ax(rX), ry = rY >= 0 ? -ax(rY) : 0;
  const b = (i) => !!(gp.buttons[i] && gp.buttons[i].pressed);
  if (std) { lx = lx || (b(15) ? 1 : 0) - (b(14) ? 1 : 0); ly = ly || (b(12) ? 1 : 0) - (b(13) ? 1 : 0); }
  else if (a.length >= 10 && Math.abs(a[9]) <= 1.01 && Math.abs(st.rest[9]) > 1.05) {
    // hat switch on axis 9 (DirectInput pads): -1 up, -1/7 right, 3/7 down, 5/7 left; about 1.29 = not pressed
    const h = Math.round((a[9] + 1) * 3.5);
    if (h === 0) ly = 1; else if (h === 2) lx = 1; else if (h === 4) ly = -1; else if (h === 6) lx = -1;
  }
  return { lx, ly, rx, ry };
}
function padPoll() {
  const list = navigator.getGamepads ? [...navigator.getGamepads()].filter((g) => g && g.connected) : [];
  pad.lx = pad.ly = pad.rx = pad.ry = 0; pad.active = false;
  if (!list.length) return;
  const d = (x) => (Math.abs(x) < 0.15 ? 0 : Math.sign(x) * Math.min(1, (Math.abs(x) - 0.15) / 0.8));
  for (const gp of list) {
    const st = padState(gp);
    const v = padAxes(gp, st);
    const now = gp.buttons.map((x) => !!x.pressed), was = st.prevBtn, hit = (i) => now[i] && !was[i];
    st.prevBtn = now;
    if (!st.engaged) {                          // wait for a person: a new button press or a real stick push
      if (now.some((x, i) => x && !was[i]) || [v.lx, v.ly, v.rx, v.ry].some((x) => Math.abs(x) > 0.5)) {
        st.engaged = true;
        $('extPad').hidden = false; $('padName').textContent = 'Gamepad: ' + (gp.id.replace(/\s*\(.*$/, '').slice(0, 40) || 'connected');
        toast(`Joystick ready: ${gp.id.replace(/\s*\(.*$/, '').slice(0, 40) || 'gamepad'}`);
      } else continue;
    }
    pad.name = gp.id;
    pad.lx = pad.lx || d(v.lx); pad.ly = pad.ly || d(v.ly); pad.rx = pad.rx || d(v.rx); pad.ry = pad.ry || d(v.ry);
    if (hit(0)) { cmd('gripper', { close: true }); buzz(25); }
    if (hit(1)) cmd('gripper', { close: false });
    if (hit(2)) cmd('home');
    if (hit(3)) cmd('teach_add');
    if (hit(4) || hit(5)) {
      const sp = $('speed');
      sp.value = clamp(+sp.value + (hit(5) ? 10 : -10), +sp.min, +sp.max);
      sp.dispatchEvent(new Event('input')); sp.dispatchEvent(new Event('change')); toast(`Speed ${sp.value} %`);
    }
    if (hit(9)) $('estop').click();
    if (hit(8)) { app.padBelt = !app.padBelt; cmd('conveyor', { run: app.padBelt }); toast(app.padBelt ? 'Belt on' : 'Belt off'); }
  }
  pad.active = !!(pad.lx || pad.ly || pad.rx || pad.ry);
}
window.addEventListener('gamepadconnected', (e) => { delete pad.dev[e.gamepad.index]; });   // shown once it is actually used
window.addEventListener('gamepaddisconnected', (e) => {
  const was = pad.dev[e.gamepad.index] && pad.dev[e.gamepad.index].engaged;
  delete pad.dev[e.gamepad.index]; pad.active = false;
  if (was) { $('extPad').hidden = true; toast('Joystick disconnected'); }
});
setInterval(padPoll, 30);
window.onNativePause = () => { if (app.hand) handOff(); };
setInterval(() => {                         // fist closes, open hand opens (only on a change)
  if (!app.hand) return;
  const h = window.ScaraHand.out;
  if (!h.seen || h.fist === handFist) return;
  if (handFist !== null) { cmd('gripper', { close: h.fist }); buzz(25); }
  handFist = h.fist;
}, 100);

/* ------------------------------------------------------------------ v3: exact joint sliders */
const SL_FMT = { shoulder_joint: (v) => `${fix(v * 1000, 0)} mm` };
let slDrag = null;
document.querySelectorAll('.sliders input[data-j]').forEach((el) => {
  const j = el.dataset.j;
  el.addEventListener('pointerdown', () => { slDrag = j; });
  el.addEventListener('input', () => {
    slDrag = j;
    el.nextElementSibling.textContent = (SL_FMT[j] || deg)(+el.value);
    const base = currentTarget();
    const q = {}; for (const k of ARM) q[k] = k === j ? +el.value : base[k];
    cmd('move', { q });
  });
  el.addEventListener('change', () => { slDrag = null; });
});
function updateSliders() {
  const t = currentTarget();
  document.querySelectorAll('.sliders input[data-j]').forEach((el) => {
    const j = el.dataset.j, [lo, hi] = limits(j);
    if (+el.min !== lo) { el.min = lo; el.max = hi; }
    if (slDrag === j) return;
    const v = t[j] || 0;
    if (Math.abs(+el.value - v) > 1e-4) el.value = v;
    el.nextElementSibling.textContent = (SL_FMT[j] || deg)(v);
  });
}
setInterval(updateSliders, 200);
/* Manual tab: joysticks or sliders (phone); the PC page shows both */
let mmode = store.get('mmode', 'sticks');
function setMmode(m) {
  mmode = m; store.set('mmode', m);
  document.querySelectorAll('#mmode button').forEach((b) => b.classList.toggle('on', b.dataset.m === m));
  document.querySelector('.tab[data-tab="manual"]').dataset.mode = m;
}
document.querySelectorAll('#mmode button').forEach((b) => b.addEventListener('click', () => setMmode(b.dataset.m)));
setMmode(mmode);
const gSl = document.querySelector('#mSliders input[data-g]');
gSl.addEventListener('input', () => { $('gripFine').value = gSl.value; $('gripFine').dispatchEvent(new Event('input')); });
setInterval(() => {                          // gripper slider follows the gripper
  if (document.activeElement === gSl) return;
  const g = clamp((app.q.left_finger_joint || 0) / (scene.grip_close || -0.025), 0, 1);
  gSl.value = Math.round(g * 100);
  gSl.nextElementSibling.textContent = g < 0.08 ? 'open' : `${Math.round(g * 100)} %`;
}, 200);

/* ------------------------------------------------------------------ v3: keyboard (computer) */
const keys = new Set();
function keyVec() {
  const k = (a, b) => (keys.has(a) ? 1 : 0) - (keys.has(b) ? 1 : 0);
  const v = { lx: k('d', 'a') || k('arrowright', 'arrowleft'), ly: k('w', 's') || k('arrowup', 'arrowdown'), rx: k('l', 'j'), ry: k('i', 'k') };
  v.active = !!(v.lx || v.ly || v.rx || v.ry);
  return v;
}
const ARROWS = ['arrowleft', 'arrowright', 'arrowup', 'arrowdown'];
function showKeys() {                         // a real keyboard was used (PC, or one plugged into the phone)
  if (!$('extKeys').hidden) return;
  $('extKeys').hidden = false; store.set('kbSeen', 1);
}
if (store.get('kbSeen', 0)) $('extKeys').hidden = false;
document.addEventListener('keydown', (e) => {
  // typing in a text box is not driving; sliders keep their own arrow keys but still let WASD drive
  const t = e.target, typing = t.closest('select, textarea') || (t.closest('input') && t.type !== 'range' && t.type !== 'checkbox');
  if (typing || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (e.isTrusted) showKeys();
  if (ARROWS.includes(k) && t.type === 'range') return;
  if (('adwsjlik'.includes(k) && k.length === 1) || ARROWS.includes(k)) { keys.add(k); e.preventDefault(); return; }
  if (e.repeat) return;
  if (k === '+' || k === '=' || k === '-' || k === '_') {
    const sp = $('speed'); sp.value = clamp(+sp.value + (k === '+' || k === '=' ? 10 : -10), +sp.min, +sp.max);
    sp.dispatchEvent(new Event('input')); sp.dispatchEvent(new Event('change')); toast(`Speed ${sp.value} %`); return;
  }
  if (k === 'x') { setDrive(app.drive === 'xy' ? 'joints' : 'xy'); toast(app.drive === 'xy' ? 'Sticks: XY + height' : 'Sticks: joints'); return; }
  if (k === ' ') { e.preventDefault(); $('estop').click(); }
  else if (k === 'g') cmd('gripper', { close: true });
  else if (k === 'o') cmd('gripper', { close: false });
  else if (k === 'h') cmd('home');
  else if (k === 'p') cmd('teach_add');
  else if (k === 'v') voice();
  else if (k === 't') toggleTheme();
});
document.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => keys.clear());

/* ------------------------------------------------------------------ v3: forward / inverse kinematics */
const wrapPi = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const kinK = () => scene.kin || SC.kin;
let kFrame = store.get('kframe', 'robot');
/* robot frame: origin on the column axis at table height, x along the arm at column = 0 */
function toRobot(x, y) {
  const k = kinK(), c = Math.cos(k.base_yaw), s = Math.sin(k.base_yaw), dx = x - k.column[0], dy = y - k.column[1];
  return [c * dx + s * dy, -s * dx + c * dy];
}
function fromRobot(x, y) {
  const k = kinK(), c = Math.cos(k.base_yaw), s = Math.sin(k.base_yaw);
  return [k.column[0] + c * x - s * y, k.column[1] + s * x + c * y];
}
function fkFull(q) {
  const k = kinK(), t1 = q.column_joint, t2 = q.forearm_joint, t3 = q.wrist_joint;
  const xr = k.L1 * Math.cos(t1) + k.L2 * Math.cos(t1 + t2) + k.grip_offset * Math.cos(t1 + t2 + t3);
  const yr = k.L1 * Math.sin(t1) + k.L2 * Math.sin(t1 + t2) + k.grip_offset * Math.sin(t1 + t2 + t3);
  const [xw, yw] = fromRobot(xr, yr);
  return { xr, yr, xw, yw, yawR: wrapPi(t1 + t2 + t3), yawW: wrapPi(t1 + t2 + t3 + k.base_yaw),
    h: k.pad_bottom_z0 + q.shoulder_joint, reach: Math.hypot(xr, yr), m: Math.abs(Math.sin(t2)) };
}
/* both elbow answers for gripper centre (xr, yr) in the robot frame, yaw psi, pad height h */
function ikBoth(xr, yr, psi, h) {
  const k = kinK(), a = k.L1, b = k.L2;
  const wx = xr - k.grip_offset * Math.cos(psi), wy = yr - k.grip_offset * Math.sin(psi);
  const c2 = (wx * wx + wy * wy - a * a - b * b) / (2 * a * b);
  const d = h - k.pad_bottom_z0;
  if (Math.abs(c2) > 1) return { reach: false, why: Math.hypot(wx, wy) > a + b ? 'too far from the column' : 'too close to the column' };
  const out = [];
  for (const sgn of [-1, 1]) {
    const t2 = sgn * Math.acos(c2);
    const t1 = wrapPi(Math.atan2(wy, wx) - Math.atan2(b * Math.sin(t2), a + b * Math.cos(t2)));
    let t3 = wrapPi(psi - t1 - t2);
    const [wlo, whi] = limits('wrist_joint');
    for (const kk of [0, 2 * Math.PI, -2 * Math.PI]) if (t3 + kk >= wlo && t3 + kk <= whi && Math.abs(t3 + kk) < Math.abs(t3) + 1e-9) t3 += kk;
    const q = { column_joint: t1, shoulder_joint: d, forearm_joint: t2, wrist_joint: t3 };
    const bad = ARM.filter((j) => { const [lo, hi] = limits(j); return q[j] < lo - 1e-6 || q[j] > hi + 1e-6; });
    out.push({ elbow: t2 >= 0 ? 'left' : 'right', q, bad });
  }
  return { reach: true, sols: out };
}
const JN = { column_joint: 'θ1 column', shoulder_joint: 'd  height', forearm_joint: 'θ2 elbow', wrist_joint: 'θ3 wrist' };
const jtxt = (j, v) => (j === 'shoulder_joint' ? `${fix(v * 1000, 1)} mm` : deg(v));
function kinInputs() {
  const v = (id) => parseFloat($(id).value);
  return { x: v('kX') / 1000, y: v('kY') / 1000, h: v('kZ') / 1000, yaw: v('kA') * Math.PI / 180 };
}
function setKinInputs(x, y, h, yaw) {          // x, y, yaw in the chosen frame
  $('kX').value = fix(x * 1000, 1); $('kY').value = fix(y * 1000, 1);
  $('kZ').value = fix(h * 1000, 1); $('kA').value = fix(yaw * 180 / Math.PI, 1);
  solveKin();
}
function useHere() {
  const f = fkFull(currentTarget());
  if (kFrame === 'robot') setKinInputs(f.xr, f.yr, f.h, f.yawR); else setKinInputs(f.xw, f.yw, f.h, f.yawW);
}
let kinSols = [];
function solveKin() {
  const i = kinInputs(), el = $('kSol');
  if ([i.x, i.y, i.h, i.yaw].some((v) => !Number.isFinite(v))) { el.innerHTML = '<p class="note none">Type X, Y, height and yaw.</p>'; kinMarker(null); return; }
  const k = kinK();
  let [xr, yr] = [i.x, i.y], psi = i.yaw;
  if (kFrame === 'world') { [xr, yr] = toRobot(i.x, i.y); psi = i.yaw - k.base_yaw; }
  const r = ikBoth(xr, yr, psi, i.h);
  const [mx, my] = fromRobot(xr, yr);
  if (!r.reach) {
    kinSols = [];
    el.innerHTML = `<p class="note none">Out of reach: the point is ${r.why} (gripper reach ${fix((k.L1 + k.L2 + k.grip_offset) * 1000, 0)} mm).</p>`;
    kinMarker([mx, my], false);
    return;
  }
  const cur = currentTarget();
  const cost = (s) => (s.bad.length ? 100 : 0) + Math.abs(s.q.column_joint - cur.column_joint) + Math.abs(s.q.forearm_joint - cur.forearm_joint);
  kinSols = r.sols;
  const best = r.sols.filter((s) => !s.bad.length).sort((a, b) => cost(a) - cost(b))[0];
  el.innerHTML = r.sols.map((s, n) => `<div class="sol${s === best ? ' best' : ''}"><h4>Elbow ${s.elbow}${s === best ? ' · nearest' : ''}</h4>`
    + ARM.map((j) => `<div class="j${s.bad.includes(j) ? ' bad' : ''}"><span>${JN[j]}</span><b>${jtxt(j, s.q[j])}</b></div>`).join('')
    + (s.bad.length ? `<div class="why">Past the joint limit: ${s.bad.map((j) => JN[j].split(' ').pop()).join(', ')}</div>` : '')
    + `<button data-sol="${n}" class="${s === best ? 'primary' : ''}" ${s.bad.length ? 'disabled' : ''}>Go</button></div>`).join('');
  el.querySelectorAll('button[data-sol]').forEach((b) => {
    b.disabled = b.disabled || !(app.link === 'demo' || app.link === 'twin' || app.link === 'rosbridge');
    b.addEventListener('click', () => {
      const s = kinSols[+b.dataset.sol];
      if (!s) return;
      cmd('move', { q: { ...s.q } });
      pushLog({ text: `IK: going to X ${$('kX').value}, Y ${$('kY').value} mm (${kFrame} frame), elbow ${s.elbow}`, level: 'info' });
    });
  });
  kinMarker([mx, my], !!best, i.h);
}
function updateFk() {
  if (!$('fkX').offsetParent) return;           // tab not visible
  const f = fkFull(app.q), w = kFrame === 'world';
  $('fkX').textContent = `${fix((w ? f.xw : f.xr) * 1000, 1)} mm`;
  $('fkY').textContent = `${fix((w ? f.yw : f.yr) * 1000, 1)} mm`;
  $('fkZ').textContent = `${fix(f.h * 1000, 1)} mm`;
  $('fkA').textContent = deg(w ? f.yawW : f.yawR);
  $('fkR').textContent = `${fix(f.reach * 1000, 1)} mm`;
  $('fkE').textContent = Math.abs(app.q.forearm_joint) < 0.02 ? 'straight' : (app.q.forearm_joint > 0 ? 'left' : 'right');
  $('fkM').style.width = `${Math.round(f.m * 100)}%`;
  $('fkM').classList.toggle('low', f.m < 0.15);
  $('fkMt').textContent = f.m < 0.15 ? `${Math.round(f.m * 100)} % near singular` : `${Math.round(f.m * 100)} %`;
}
setInterval(updateFk, 150);
document.querySelectorAll('#kFrame button').forEach((b) => {
  b.classList.toggle('on', b.dataset.f === kFrame);
  b.addEventListener('click', () => {
    if (b.dataset.f === kFrame) return;
    // keep the same physical point: convert what is typed
    const i = kinInputs(), k = kinK();
    kFrame = b.dataset.f; store.set('kframe', kFrame);
    document.querySelectorAll('#kFrame button').forEach((x) => x.classList.toggle('on', x === b));
    if ([i.x, i.y, i.h, i.yaw].every(Number.isFinite)) {
      if (kFrame === 'world') { const [x, y] = fromRobot(i.x, i.y); setKinInputs(x, y, i.h, wrapPi(i.yaw + k.base_yaw)); }
      else { const [x, y] = toRobot(i.x, i.y); setKinInputs(x, y, i.h, wrapPi(i.yaw - k.base_yaw)); }
    }
    updateFk();
  });
});
['kX', 'kY', 'kZ', 'kA'].forEach((id) => $(id).addEventListener('input', solveKin));
on('kHere', useHere);
on('kTap', () => {
  app.kinTap = !app.kinTap;
  if (app.kinTap && app.tapMode) $('tapPick').click();
  $('kTap').classList.toggle('on', app.kinTap);
  hint(app.kinTap ? 'Tap a point on the table' : '');
});
function kinTapAt(p) {
  const i = kinInputs(), k = kinK();
  const f = fkFull(currentTarget());
  const h = Number.isFinite(i.h) ? i.h : f.h, yaw = Number.isFinite(i.yaw) ? i.yaw : (kFrame === 'world' ? f.yawW : f.yawR);
  if (kFrame === 'world') setKinInputs(p.x, p.y, h, yaw);
  else { const [x, y] = toRobot(p.x, p.y); setKinInputs(x, y, h, yaw); }
  app.kinTap = false; $('kTap').classList.remove('on'); hint('');
  void k;
}

/* 3-D: the IK target and the area the gripper can really reach (joint limits included) */
const kinGroup = new THREE.Group();
world.add(kinGroup);
const kinMark = new THREE.Group();
{
  const m = new THREE.MeshBasicMaterial({ color: 0x16a34a, depthTest: false, transparent: true, opacity: 0.9 });
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.011, 0.015, 40), m);
  const bar1 = new THREE.Mesh(new THREE.PlaneGeometry(0.044, 0.003), m), bar2 = bar1.clone(); bar2.rotation.z = Math.PI / 2;
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.0015, 0.0015, 1, 8), m); pole.rotation.x = Math.PI / 2;
  kinMark.add(ring, bar1, bar2, pole);
  kinMark.userData = { m, pole };
  kinMark.renderOrder = 5;
  kinMark.visible = false;
  kinGroup.add(kinMark);
}
function kinMarker(xy, ok, h) {
  if (!xy) { kinMark.visible = false; return; }
  const top = scene.table_top || 0.4;
  kinMark.visible = true;
  kinMark.position.set(xy[0], xy[1], top + 0.002);
  kinMark.userData.m.color.set(ok ? 0x16a34a : 0xdc2626);
  const hh = Math.max(0.002, h || 0.002);
  kinMark.userData.pole.scale.set(1, hh, 1); kinMark.userData.pole.position.z = hh / 2;
}
let reachMesh = null;
function buildReach() {
  const k = kinK(), R = k.L1 + k.L2 + k.grip_offset + 0.01, N = 220, cv = document.createElement('canvas');
  cv.width = cv.height = N;
  const g = cv.getContext('2d'), img = g.createImageData(N, N);
  const [lo1, hi1] = limits('column_joint'), [lo2, hi2] = limits('forearm_joint');
  const okWrist = (wx, wy) => {                 // a wrist centre the column + elbow can reach
    const c2 = (wx * wx + wy * wy - k.L1 * k.L1 - k.L2 * k.L2) / (2 * k.L1 * k.L2);
    if (Math.abs(c2) > 1) return false;
    for (const sgn of [-1, 1]) {
      const t2 = sgn * Math.acos(c2), t1 = wrapPi(Math.atan2(wy, wx) - Math.atan2(k.L2 * Math.sin(t2), k.L1 + k.L2 * Math.cos(t2)));
      if (t1 >= lo1 && t1 <= hi1 && t2 >= lo2 && t2 <= hi2) return true;
    }
    return false;
  };
  for (let iy = 0; iy < N; iy++) for (let ix = 0; ix < N; ix++) {
    const xr = (ix + 0.5) / N * 2 * R - R, yr = R - (iy + 0.5) / N * 2 * R;
    let n = 0;
    for (let a = 0; a < 24 && !n; a++) {
      const ps = a / 24 * 2 * Math.PI;
      if (okWrist(xr - k.grip_offset * Math.cos(ps), yr - k.grip_offset * Math.sin(ps))) n = 1;
    }
    const o = 4 * (iy * N + ix);
    img.data[o] = 10; img.data[o + 1] = 92; img.data[o + 2] = 194; img.data[o + 3] = n ? 46 : 0;
  }
  g.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(cv);
  reachMesh = new THREE.Mesh(new THREE.PlaneGeometry(2 * R, 2 * R),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
  reachMesh.position.set(k.column[0], k.column[1], (scene.table_top || 0.4) + 0.0025);
  reachMesh.rotation.z = k.base_yaw;           // texture is drawn in the robot frame
  kinGroup.add(reachMesh);
}
function showKinOverlay() {
  const onTab = !!$('fkX').offsetParent;
  if (onTab && $('kReach').checked && !reachMesh) buildReach();
  if (reachMesh) reachMesh.visible = onTab && $('kReach').checked;
  kinGroup.visible = onTab;
}
$('kReach').addEventListener('change', showKinOverlay);
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => setTimeout(() => {
  showKinOverlay();
  if (b.dataset.tab === 'kin' && !$('kX').value) useHere();
}, 0)));
setTimeout(() => { showKinOverlay(); if ($('fkX').offsetParent && !$('kX').value) useHere(); }, 300);

/* ------------------------------------------------------------------ v3.1: name, start screen, joint bars */
(function brand() {
  const b = window.SCARA_BRAND || {};
  const ver = b.version || '3.2';
  if (b.name) {
    document.documentElement.classList.add('named');
    $('nameplate').textContent = b.name; $('nameplate').hidden = false;
    $('bootName').textContent = b.name; $('bootName').hidden = false;
    document.title = `SCARA Digital Twin by ${b.name}`;
    $('about').innerHTML = `SCARA Digital Twin ${ver} - designed and built by <b>${b.name}</b>. MuJoCo, Gazebo and the real robot in one app.`;
  } else {
    $('about').textContent = `SCARA Digital Twin ${ver} - MuJoCo, Gazebo and the real robot in one app.`;
  }
})();
let bootDone = false;
function hideBoot() {
  if (bootDone) return;
  bootDone = true;
  $('boot').classList.add('gone');
  setTimeout(() => { $('boot').hidden = true; }, 600);
}
setTimeout(hideBoot, 4000);                 // never keep it up, whatever happens
const readoutBoxes = [...document.querySelectorAll('#readout > div')];
function updateBars() {
  ARM.forEach((j, i) => {
    const [lo, hi] = limits(j), v = app.q[j] || 0;
    const p = clamp((v - lo) / ((hi - lo) || 1), 0, 1);
    readoutBoxes[i].style.setProperty('--p', p.toFixed(3));
    readoutBoxes[i].classList.toggle('near', p < 0.04 || p > 0.96);
  });
  const g = clamp((app.q.left_finger_joint || 0) / (scene.grip_close || -0.025), 0, 1);
  readoutBoxes[4].style.setProperty('--p', g.toFixed(3));
  if ($('tcp').offsetParent) {
    const f = fkFull(app.q);
    $('tcp').innerHTML = `<span>Gripper X<b>${fix(f.xr * 1000, 1)} mm</b></span><span>Y<b>${fix(f.yr * 1000, 1)} mm</b></span>`
      + `<span>height<b>${fix(f.h * 1000, 0)} mm</b></span><span>yaw<b>${deg(f.yawR)}</b></span>`;
  }
}
setInterval(updateBars, 120);
function markMore() {                         // fade the bottom of a tab that has more below
  document.querySelectorAll('.tab').forEach((t) => {
    t.classList.toggle('more', t.classList.contains('on') && t.scrollHeight - t.scrollTop - t.clientHeight > 6);
  });
}
document.querySelectorAll('.tab').forEach((t) => t.addEventListener('scroll', markMore, { passive: true }));
setInterval(markMore, 500);
window.addEventListener('resize', markMore);

/* ------------------------------------------------------------------ v3.1: dark / light, arm camera size */
const pcMQ = window.matchMedia('(min-width: 1100px) and (min-height: 640px)');
function applyTheme() {
  const pref = store.get('theme', 'auto');
  document.documentElement.dataset.theme = pref === 'auto' ? (pcMQ.matches ? 'dark' : 'light') : pref;
}
if (pcMQ.addEventListener) pcMQ.addEventListener('change', applyTheme);
applyTheme();
function toggleTheme() {
  const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
  store.set('theme', next);
  applyTheme();
  toast(next === 'dark' ? 'Dark mode' : 'Light mode');
}
on('theme', toggleTheme);
{
  const fr = $('insetFrame'), grip = $('insetGrip');
  let pinch = null;
  const dist = (t) => Math.hypot(t[0].clientX - t[1].clientX, t[0].clientY - t[1].clientY);
  fr.addEventListener('touchstart', (e) => {
    if (e.touches.length >= 2) { pinch = { d: dist(e.touches) || 1, s: insetScale }; e.preventDefault(); }
  }, { passive: false });
  fr.addEventListener('touchmove', (e) => {
    if (!pinch || e.touches.length < 2) return;
    e.preventDefault();
    setInsetScale(pinch.s * dist(e.touches) / pinch.d, false);
  }, { passive: false });
  fr.addEventListener('touchend', (e) => {
    if (pinch && e.touches.length < 2) { pinch = null; store.set('insetScale', +insetScale.toFixed(3)); }
  });
  let drag = null;
  grip.addEventListener('pointerdown', (e) => {
    e.preventDefault(); e.stopPropagation();
    grip.setPointerCapture(e.pointerId);
    drag = { x: e.clientX, y: e.clientY, w: fr.getBoundingClientRect().width, s: insetScale };
    fr.classList.add('sizing');
  });
  grip.addEventListener('pointermove', (e) => {
    if (!drag) return;
    // the window hangs from its top-right corner: dragging the handle left / down makes it bigger
    const grow = Math.max(drag.x - e.clientX, (e.clientY - drag.y) * 4 / 3);
    setInsetScale(drag.s * (drag.w + grow) / drag.w, false);
  });
  const end = () => { if (drag) { drag = null; fr.classList.remove('sizing'); store.set('insetScale', +insetScale.toFixed(3)); } };
  grip.addEventListener('pointerup', end);
  grip.addEventListener('pointercancel', end);
  fr.addEventListener('dblclick', () => setInsetScale(1, true));
}

/* ------------------------------------------------------------------ v3.1: motion graph, recording, gripper trail */
const MG = { buf: [], rec: null, t0: performance.now() };
const MG_COLORS = ['#3d8bfd', '#f2b705', '#2f9e44', '#d63a32'];
setInterval(() => {                          // 20 Hz samples of where the arm is
  const t = (performance.now() - MG.t0) / 1000, q = app.q;
  MG.buf.push([t, ...ARM.map((j) => q[j] || 0)]);
  while (MG.buf.length && t - MG.buf[0][0] > 10.5) MG.buf.shift();
  if (MG.rec) {
    const f = fkFull(q);
    MG.rec.push([t - MG.rec.t0, ...ARM.map((j) => q[j] || 0), q.left_finger_joint || 0, f.xr, f.yr, f.h, f.yawR]);
    if (MG.rec.length >= 20 * 300) stopRecording('5 minutes - recording stopped');
  }
  if ($('mgraph').offsetParent) drawGraph();
  trailAdd();
}, 50);
function drawGraph() {
  const cv = $('mgraph'), r = cv.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
  const W = Math.max(1, Math.round(r.width * dpr)), H = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
  const g = cv.getContext('2d'), cs = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, W, H);
  g.strokeStyle = cs.getPropertyValue('--line2').trim() || '#ccc'; g.lineWidth = 1;
  for (let k = 0; k <= 4; k++) { const y = Math.round(6 * dpr + (H - 12 * dpr) * k / 4) + 0.5; g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke(); }
  if (MG.buf.length < 2) return;
  const tEnd = MG.buf[MG.buf.length - 1][0];
  ARM.forEach((j, i) => {
    const [lo, hi] = limits(j);
    g.strokeStyle = MG_COLORS[i]; g.lineWidth = 2 * dpr; g.beginPath();
    MG.buf.forEach((p, k) => {
      const x = W - (tEnd - p[0]) / 10 * W, v = clamp((p[i + 1] - lo) / ((hi - lo) || 1), 0, 1);
      const y = 6 * dpr + (1 - v) * (H - 12 * dpr);
      if (k) g.lineTo(x, y); else g.moveTo(x, y);
    });
    g.stroke();
  });
}
function stopRecording(why) {
  if (!MG.rec) return;
  MG.last = MG.rec; MG.rec = null;
  $('mRec').classList.remove('on'); $('mRec').textContent = 'Record';
  $('mSave').disabled = !MG.last.length;
  $('mNote').textContent = `${why || 'Recorded'}: ${MG.last.length} samples, ${(MG.last.length / 20).toFixed(1)} s. Save CSV to keep it.`;
}
on('mRec', () => {
  if (MG.rec) { stopRecording(); return; }
  MG.rec = []; MG.rec.t0 = (performance.now() - MG.t0) / 1000;
  $('mRec').classList.add('on'); $('mRec').textContent = 'Stop recording';
  $('mNote').textContent = 'Recording… move the arm (joysticks, sliders, IK, the cell).';
});
on('mSave', () => {
  const rows = MG.last || [];
  if (!rows.length) return;
  const head = 'time_s,column_rad,height_m,elbow_rad,wrist_rad,gripper_m,x_robot_m,y_robot_m,pad_height_m,yaw_rad';
  const csv = head + '\n' + rows.map((r) => r.map((v, k) => (k ? v.toFixed(5) : v.toFixed(3))).join(',')).join('\n') + '\n';
  const name = `scara_motion_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;
  if (window.ScaraNative) {                  // the Android app: the PC keeps the file
    if (app.link === 'twin') { cmd('save_recording', { name, csv }); toast('Saved on the PC (see the log for where)'); }
    else toast('Connect to the PC to save the recording - or record in the website');
    return;
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
  a.download = name; document.body.appendChild(a); a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  toast('Saved ' + name);
});
// the gripper's path in the 3-D view
const TRAIL_N = 400;
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_N * 3), 3));
trailGeo.setDrawRange(0, 0);
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xf2b705, transparent: true, opacity: 0.9, depthTest: false }));
trail.renderOrder = 6; trail.frustumCulled = false; trail.visible = false;
world.add(trail);
let trailPts = [];
function trailAdd() {
  if (!trail.visible) return;
  const f = fkFull(app.q), z = (scene.table_top || 0.4) + f.h;
  const last = trailPts[trailPts.length - 1];
  if (last && Math.hypot(last[0] - f.xw, last[1] - f.yw, last[2] - z) < 0.002) return;
  trailPts.push([f.xw, f.yw, z]);
  if (trailPts.length > TRAIL_N) trailPts.shift();
  const a = trailGeo.attributes.position.array;
  trailPts.forEach((p, i) => { a[3 * i] = p[0]; a[3 * i + 1] = p[1]; a[3 * i + 2] = p[2]; });
  trailGeo.attributes.position.needsUpdate = true;
  trailGeo.setDrawRange(0, trailPts.length);
}
$('kTrail').addEventListener('change', () => {
  trail.visible = $('kTrail').checked; trailPts = []; trailGeo.setDrawRange(0, 0);
  store.set('trail', trail.visible);
});
if (store.get('trail', false)) { $('kTrail').checked = true; trail.visible = true; }

/* ------------------------------------------------------------------ main loop */
let lastFrame = performance.now(), lastNote = '';
function frame() {
  const now = performance.now(), dt = (now - lastFrame) / 1000;
  lastFrame = now;
  if (app.link === 'demo' && app.sim) {
    const S = app.sim;
    for (let left = Math.min(dt, 0.25); left > 1e-4; left -= 0.05) S.step(Math.min(left, 0.05));   // slow phones: substeps
    Object.assign(app.q, S.q);
    app.pucks = S.pucks;
    const c = S.state();
    showCell({ ...c, note: S.world === 'manual'
      ? 'Offline. Table: drive the arm with the joysticks and grab pucks. Choose Sorting or Command for the conveyor cell.'
      : S.world === 'sort' ? 'Offline sorting cell: Start runs the belt, the robot puts every colour in its own bin.'
        : 'Offline command cell: tap a colour - it stays active until the next one. Start / Stop runs the belt.' });
    showTeach(app.teachLocal, !!(S.auto && S.auto.manualPlan && S.auto.loop !== undefined));
    if (c.note && c.note !== lastNote && !S.running) pushLog({ text: c.note, level: c.note.startsWith('guard') ? 'warning' : 'info' });
    lastNote = c.note;
  }
  if (app.follow) {
    const f = kfk(app.q);
    const z = (scene.table_top || 0.4) + 0.08;
    const dx = f.x - controls.target.x, dy = f.y - controls.target.y, dz = z - controls.target.z;
    controls.target.x += dx * 0.12; controls.target.y += dy * 0.12; controls.target.z += dz * 0.12;
    cam.position.x += dx * 0.12; cam.position.y += dy * 0.12; cam.position.z += dz * 0.12;
  }
  if (app.link === 'demo' && app.sim) {
    const held = !!app.sim.held;
    if (held !== app.wasHeld) buzz(held ? 40 : 15);
    app.wasHeld = held;
    const tot = Object.values(app.sim.sorted).reduce((a, b) => a + b, 0);
    if (tot && !app.firstPick) app.firstPick = { t: app.sim.t, n: tot };
    $('rate').textContent = app.firstPick && tot > app.firstPick.n
      ? `${(60 * (tot - app.firstPick.n) / (app.sim.t - app.firstPick.t)).toFixed(1)} picks/min` : '';
  }
  robot.set(app.q);
  const r = app.q;
  $('r0').textContent = deg(r.column_joint);
  $('r1').textContent = `${fix(r.shoulder_joint * 1000, 0)} mm`;
  $('r2').textContent = deg(r.forearm_joint);
  $('r3').textContent = deg(r.wrist_joint);
  $('r4').textContent = r.left_finger_joint > -0.002 ? 'open' : `${Math.min(100, Math.round(100 * r.left_finger_joint / (scene.grip_close || -0.025)))} %`;
  const real = app.twinOn ? app.real : null;
  $('g0').textContent = real ? 'robot ' + deg(real.column_joint || 0) : '';
  $('g1').textContent = real ? `robot ${((real.shoulder_joint || 0) * 1000).toFixed(0)} mm` : '';
  $('g2').textContent = real ? 'robot ' + deg(real.forearm_joint || 0) : '';
  $('g3').textContent = real ? 'robot ' + deg(real.wrist_joint || 0) : '';
  $('g4').textContent = '';
  drawPucks(app.pucks);
  trailAdd();                                    // the gripper's path: every frame, smooth on any device
  render();
  if (!bootDone && performance.now() > 600) hideBoot();
  requestAnimationFrame(frame);
}
resize();
refreshEnabled();
requestAnimationFrame(frame);

window.scaraDebug = { app, cmd, setWorld, setDrive, runSpoken, handOn, handOff, keyVec, fkFull, ikBoth, solveKin, setKinInputs, kinTapAt, MG, trailCount: () => trailPts.length,
  insetOff: () => insetOff, pad, setCamMode, camState: () => ({ mode: camMode, h: camH, rect: camRect }), ghostVisible: () => ghost.base.visible };
const firstRun = !store.get('helpSeen', false);
if (firstRun) $('helpSheet').hidden = false;          // first start: the short guide, then the connect sheet   // for tools/test_app.py

// served by scara_twin_server.py (the PC control page): connect to that server by itself
const fromServer = /^https?:$/.test(location.protocol) && !/[?&]standalone/.test(location.search);
if (fromServer) {
  fetch('/api/config').then((r) => r.json()).then((c) => {
    if (c && c.server === 'scara_twin') { $('helpSheet').hidden = true; connect(location.hostname, c.app_port || 9090); }
  }).catch(() => { /* a plain web server (testing): behave like the phone */ startPhone(); });
} else startPhone();
function startPhone() {
  // reconnect to the last PC on start
  const ip0 = store.get('ip', '');
  if (ip0) connect(ip0, store.get('port', 9090));
  else if (!firstRun) setTimeout(() => { if (app.link === 'off') $('conn').click(); }, 400);
}
})();
