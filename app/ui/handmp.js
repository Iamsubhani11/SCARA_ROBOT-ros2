/* Hand control with MediaPipe - 21 points on each hand, no internet needed.
 *
 * MediaPipe "Hand Landmarker" (Google, Apache 2.0) finds up to two hands in the camera picture and
 * gives 21 points on each one (wrist, knuckles, finger joints, finger tips). Everything it needs is
 * inside the app: lib/mediapipe/ has the JavaScript, the WebAssembly program and the model file,
 * so it works in the browser, on the PC page and inside the Android app without internet.
 *
 * How your hands drive the robot (the same as the joysticks):
 *   one hand        the middle of your palm is the LEFT joystick: inside the small circle the arm
 *                   stays still, move away from it and the arm moves that way, faster the further
 *   fist            closes the gripper, open hand opens it (it has to stay that way for 0.3 s)
 *   pinch           thumb and index together: now your hand is the RIGHT joystick, measured from
 *                   where you pinched (in "XY + height" drive: pinch and move up / down = height).
 *                   The gripper does not change while you pinch.
 *   two hands       the hand on the left of the picture is the left joystick, the other one the
 *                   right joystick (like the PC program gesture_control/)
 * A hand only starts to drive after it has been inside its circle once - so a hand that comes into
 * the picture, or a second hand that appears, never makes the arm jump.
 *
 * If MediaPipe cannot start (very old phone, the page opened straight from a file in a normal
 * browser ...) the old skin-colour tracker of hand.js takes over by itself, and the camera window
 * says which one is running and why.
 */
(() => {
'use strict';

const LIB = 'lib/mediapipe/';
const STILL = 0.18 * 0.32, TRAVEL = 0.32;          // same still zone and full-speed distance as hand.js (part of the picture width)
const WRIST = 0, THUMB_TIP = 4, INDEX_MCP = 5, INDEX_TIP = 8, MIDDLE_MCP = 9, PINKY_MCP = 17;
const FINGERS = [[5, 8], [9, 12], [13, 16], [17, 20]];     // (knuckle, tip) of index .. little finger
const BONES = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8], [5, 9], [9, 10], [10, 11], [11, 12],
  [9, 13], [13, 14], [14, 15], [15, 16], [13, 17], [17, 18], [18, 19], [19, 20], [0, 17]];
const PINCH_CLOSE = 0.30, PINCH_OPEN = 0.50;       // thumb-index distance / palm size (same as gesture_core.py)
const HOLD = 300;                                   // ms a fist / open hand / pinch must last
const LOST = 300;                                   // ms without a hand -> that stick lets go
const SMOOTH = 0.45;                                // low-pass on the palm centre (0..1, higher = snappier)

/* ---------------------------------------------------------------- the hand, from 21 points */
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const palmSize = (lm) => dist(lm[MIDDLE_MCP], lm[WRIST]) + 1e-6;
const palmCentre = (lm) => ({ x: (lm[WRIST].x + lm[INDEX_MCP].x + lm[MIDDLE_MCP].x + lm[PINKY_MCP].x) / 4,
  y: (lm[WRIST].y + lm[INDEX_MCP].y + lm[MIDDLE_MCP].y + lm[PINKY_MCP].y) / 4 });
/* all four fingers curled: each tip closer to the wrist than its knuckle (works at any hand angle) */
function curled(lm) {
  let n = 0;
  for (const [k, t] of FINGERS) if (dist(lm[t], lm[WRIST]) < 1.05 * dist(lm[k], lm[WRIST])) n += 1;
  return n;
}
const pinchRatio = (lm) => dist(lm[THUMB_TIP], lm[INDEX_TIP]) / palmSize(lm);
/* landmarks come from the camera picture as it is; we show it mirrored, so x is flipped here once */
const mirror = (lm) => lm.map((p) => ({ x: 1 - p.x, y: p.y }));

/* ---------------------------------------------------------------- loading MediaPipe */
function xhr(url, type) {                 // XHR, not fetch(): fetch() cannot read file:// (the Android app)
  return new Promise((ok, no) => {
    const r = new XMLHttpRequest();
    r.open('GET', url); r.responseType = type || 'arraybuffer';
    r.onload = () => ((r.status === 200 || r.status === 0) && r.response ? ok(r.response) : no(new Error(`${url}: ${r.status}`)));
    r.onerror = () => no(new Error(`cannot read ${url}`));
    r.send();
  });
}
function addScript(src) {
  return new Promise((ok, no) => {
    const s = document.createElement('script');
    s.src = src; s.onload = ok; s.onerror = () => no(new Error(`cannot load ${src}`));
    document.head.appendChild(s);
  });
}
let landmarker = null, loading = null;
function loadMediaPipe() {
  if (landmarker) return Promise.resolve(landmarker);
  if (loading) return loading;
  loading = (async () => {
    if (window.SCARA_NO_MEDIAPIPE) throw new Error('switched off (simple tracker chosen)');
    if (typeof WebAssembly !== 'object') throw new Error('this browser has no WebAssembly');
    if (!window.Vision) await addScript(LIB + 'vision_bundle.js');
    const [loader, wasm, model] = await Promise.all([xhr(LIB + 'vision_wasm_internal.js', 'text'),
      xhr(LIB + 'vision_wasm_internal.wasm'), xhr(LIB + 'hand_landmarker.task')]);
    const blobUrl = (data, type) => URL.createObjectURL(new Blob([data], { type }));
    const files = { wasmLoaderPath: blobUrl(loader, 'text/javascript'), wasmBinaryPath: blobUrl(wasm, 'application/wasm') };
    const opts = (delegate) => ({ baseOptions: { modelAssetBuffer: new Uint8Array(model), delegate }, runningMode: 'VIDEO',
      numHands: 2, minHandDetectionConfidence: 0.5, minHandPresenceConfidence: 0.5, minTrackingConfidence: 0.5 });
    try { landmarker = await window.Vision.HandLandmarker.createFromOptions(files, opts('GPU')); }
    catch (e) { landmarker = await window.Vision.HandLandmarker.createFromOptions(files, opts('CPU')); }   // no WebGL2 on this phone
    return landmarker;
  })();
  loading.catch(() => { loading = null; });
  return loading;
}

/* ---------------------------------------------------------------- opening the camera, with reasons */
function cameraProblem(e) {
  const n = (e && e.name) || '';
  if (n === 'NotAllowedError' || n === 'SecurityError') {
    return window.ScaraNative ? 'Camera permission is off - allow it in Android Settings > Apps > SCARA Digital Twin > Permissions'
      : 'Camera permission was refused - click the camera icon in the address bar and allow it, then try again';
  }
  if (n === 'NotFoundError' || n === 'OverconstrainedError') return 'No camera found on this device';
  if (n === 'NotSupportedError') return 'This browser or WebView does not let pages use the camera';
  if (n === 'NotReadableError' || n === 'AbortError') return 'The camera is busy - close other apps or tabs that use it and try again';
  return 'Camera not available: ' + ((e && e.message) || n || e);
}
async function openCamera(video) {
  if (!window.isSecureContext || !navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    const remote = location.protocol === 'http:' && !['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
    throw new Error(remote ? 'The browser only allows the camera on http://localhost, on https or in the Android app - '
      + 'this page came over plain http from another computer. Open it on the PC itself, or use the phone app'
      : 'This browser cannot use the camera');
  }
  let stream;
  try {
    stream = await navigator.mediaDevices.getUserMedia({ audio: false,
      video: { facingMode: 'user', width: { ideal: 640 }, height: { ideal: 480 } } });
  } catch (e) {
    if (e.name !== 'OverconstrainedError' && e.name !== 'NotFoundError') throw new Error(cameraProblem(e));
    try { stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false }); }   // no front camera: any camera
    catch (e2) { throw new Error(cameraProblem(e2)); }
  }
  video.srcObject = stream; video.muted = true; video.playsInline = true;
  try {
    await Promise.race([video.play(), new Promise((_, no) => setTimeout(() => no(new Error('timeout')), 6000))]);
  } catch (e) {
    stream.getTracks().forEach((t) => t.stop());
    throw new Error('The camera started but its picture does not play - tap the page once and try again');
  }
  return stream;
}

/* ---------------------------------------------------------------- one "stick" made by a hand */
class HandStick {
  constructor() { this.reset(); }
  reset() { this.home = null; this.armed = false; this.p = null; this.x = 0; this.y = 0; this.seenT = 0; }
  /* p: smoothed palm centre (mirrored picture, 0..1); home: centre of its still circle */
  update(p, home, now) {
    if (!this.home || Math.hypot(home.x - this.home.x, home.y - this.home.y) > 1e-6) {
      this.home = home; this.armed = false; this.p = null;   // new circle: start again, do not slide in from the old place
    }
    this.p = this.p ? { x: this.p.x + SMOOTH * (p.x - this.p.x), y: this.p.y + SMOOTH * (p.y - this.p.y) } : p;
    this.seenT = now;
    const dx = this.p.x - this.home.x, dy = this.p.y - this.home.y;
    if (Math.hypot(dx, dy) < STILL) this.armed = true;        // only drive after the hand was in its circle once
    const lim = (v) => Math.max(-1, Math.min(1, v));
    this.x = this.armed ? lim(dx / TRAVEL) : 0;
    this.y = this.armed ? lim(-dy / TRAVEL) : 0;
  }
  lost(now) { if (now - this.seenT > LOST) { this.p = null; this.x = this.y = 0; this.armed = false; this.home = null; } }
}

/* ---------------------------------------------------------------- the MediaPipe tracker */
class MPTracker {
  constructor() {
    this.state = 'off'; this.hands = []; this.L = new HandStick(); this.R = new HandStick();
    this.out = { x: 0, y: 0, rx: 0, ry: 0, fist: false, seen: false, seen2: false, pinch: false, hands: 0 };
    this.fist = false; this.fistT = 0; this.openT = 0; this.pinched = false; this.pinchT = 0; this.pinchHome = null;
    this.lastTs = -1; this.lastHandT = 0; this.startT = 0;
  }
  attach(video, onStatus) {
    this.video = video; this.onStatus = onStatus || (() => {});
    this.state = 'tracking'; this.startT = performance.now(); this.lastHandT = 0;
    this.onStatus('MediaPipe: show your hand - palm to the camera');
    const loop = () => { if (this.state === 'off') return; this.tick(); this.raf = requestAnimationFrame(loop); };
    this.raf = requestAnimationFrame(loop);
    this.timer = setInterval(() => { if (document.hidden) this.tick(); }, 100);   // keeps going when frames are throttled
  }
  detach() {
    this.state = 'off'; cancelAnimationFrame(this.raf); clearInterval(this.timer);
    this.out = { x: 0, y: 0, rx: 0, ry: 0, fist: false, seen: false, seen2: false, pinch: false, hands: 0 };
    this.hands = []; this.L.reset(); this.R.reset();
  }
  calibrate() { this.L.reset(); this.R.reset(); this.pinched = false; this.pinchHome = null; this.onStatus('MediaPipe: bring your hand to the circle'); }

  tick() {
    const v = this.video;
    if (!v || v.readyState < 2 || !landmarker) return;
    const ts = performance.now();
    if (v.currentTime === this.lastVideoTime) return;   // no new camera frame yet
    this.lastVideoTime = v.currentTime;
    let res;
    try { res = landmarker.detectForVideo(v, Math.max(ts, this.lastTs + 1)); } catch (e) { this.onStatus('MediaPipe error: ' + (e.message || e)); return; }
    this.lastTs = Math.max(ts, this.lastTs + 1);
    const now = performance.now();
    const hands = (res.landmarks || []).map((lm) => mirror(lm)).map((lm) => ({ lm, c: palmCentre(lm), curled: curled(lm), pinch: pinchRatio(lm) }));
    hands.sort((a, b) => a.c.x - b.c.x);                 // left of the picture first
    this.hands = hands;
    if (hands.length) this.lastHandT = now;
    const two = hands.length >= 2;
    const left = hands[0], right = two ? hands[hands.length - 1] : null;
    // pinch (one hand only): the hand becomes the right stick, measured from where you pinched
    if (!two && left) {
      const want = this.pinched ? !(left.pinch > PINCH_OPEN) : left.pinch < PINCH_CLOSE && left.curled < 3;
      if (want !== this.pinched) { if (!this.pinchT) this.pinchT = now; else if (now - this.pinchT > HOLD * 0.7) { this.pinched = want; this.pinchT = 0; this.pinchHome = want ? { ...left.c } : null; this.L.reset(); this.R.reset(); } }
      else this.pinchT = 0;
    } else if (this.pinched) { this.pinched = false; this.pinchHome = null; this.R.reset(); }
    if (this.pinched) this.L.reset();                 // a pinching hand is only the right stick
    else if (left) this.L.update(left.c, two ? { x: 0.27, y: 0.5 } : { x: 0.5, y: 0.5 }, now);
    else this.L.lost(now);
    if (this.pinched && left) this.R.update(left.c, this.pinchHome, now);
    else if (right) this.R.update(right.c, { x: 0.73, y: 0.5 }, now);
    else this.R.lost(now);
    // fist / open on the left-stick hand (never while pinching)
    if (left && !this.pinched) {
      if (left.curled >= 4) { this.openT = 0; if (!this.fistT) this.fistT = now; if (now - this.fistT > HOLD) this.fist = true; }
      else if (left.curled <= 1 && left.pinch > PINCH_OPEN) { this.fistT = 0; if (!this.openT) this.openT = now; if (now - this.openT > HOLD) this.fist = false; }
    }
    this.out = { x: this.L.x, y: this.L.y, rx: this.R.x, ry: this.R.y, fist: this.fist, seen: !!(left && !this.pinched && this.L.armed),
      seen2: !!((right || this.pinched) && this.R.armed), pinch: this.pinched, hands: hands.length };
    this.status(now, two);
  }

  status(now, two) {
    let t;
    if (!this.hands.length) {
      const since = now - (this.lastHandT || this.startT);
      t = since > 3000 ? this.darkHint() : 'MediaPipe: show your hand - palm to the camera';
    } else if (this.pinched) t = 'Pinch: your hand is the right joystick' + (this.R.armed ? '' : ' (hold still a moment)');
    else if (!this.L.armed) t = two ? 'Two hands: bring each hand to its circle' : 'Bring your hand to the middle circle to start';
    else t = two ? 'Two hands: left = left joystick, right = right joystick' : (this.fist ? 'Fist: gripper closed' : 'Hand control on: move your hand to move the arm');
    if (t !== this.lastText) { this.lastText = t; this.onStatus(t); }
  }

  /* no hand for a while: say why if we can see it (too dark, too bright) */
  darkHint() {
    const v = this.video;
    if (!this.probe) { this.probe = document.createElement('canvas'); this.probe.width = 16; this.probe.height = 12; }
    let y = 128;
    try {
      const g = this.probe.getContext('2d', { willReadFrequently: true });
      g.drawImage(v, 0, 0, 16, 12);
      const d = g.getImageData(0, 0, 16, 12).data; let s = 0;
      for (let i = 0; i < d.length; i += 4) s += 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2];
      y = s / (d.length / 4);
    } catch (e) { /* picture not readable */ }
    if (y < 40) return 'No hand seen - the picture is too dark, turn on a light';
    if (y > 225) return 'No hand seen - too much light behind you, turn the other way';
    return 'No hand seen - hold your whole hand in the picture, 30-60 cm from the camera';
  }

  overlay() {
    const two = this.hands.length >= 2;
    const circ = [];
    if (this.pinched && this.pinchHome) circ.push({ home: this.pinchHome, stick: this.R, tag: 'R' });
    else {
      circ.push({ home: two ? { x: 0.27, y: 0.5 } : { x: 0.5, y: 0.5 }, stick: this.L, tag: 'L' });
      if (two) circ.push({ home: { x: 0.73, y: 0.5 }, stick: this.R, tag: 'R' });
    }
    return { engine: 'mediapipe', state: this.state, hands: this.hands, bones: BONES, circles: circ, deadR: STILL, travel: TRAVEL,
      out: this.out, pinch: this.pinched };
  }
}

/* ---------------------------------------------------------------- what the app talks to */
class HandControl {
  constructor() {
    this.skin = window.ScaraHandSkin; this.mp = new MPTracker();
    this.engine = 'off'; this.why = ''; this.stream = null;
  }
  get out() {
    if (this.engine === 'mediapipe') return this.mp.out;
    const o = this.skin.out;
    return { x: o.x, y: o.y, rx: 0, ry: 0, fist: o.fist, seen: o.seen, seen2: false, pinch: false, hands: o.seen ? 1 : 0 };
  }
  get state() { return this.engine === 'mediapipe' ? this.mp.state : this.skin.state; }

  /* prefer: 'auto' (MediaPipe, else the colour tracker) or 'skin' */
  async start(video, view, onStatus, prefer) {
    this.video = video; this.onStatus = onStatus || (() => {});
    this.onStatus('Starting the camera…');
    this.stream = await openCamera(video);
    if (prefer !== 'skin') {
      this.onStatus('Loading MediaPipe hand tracking…');
      try {
        await Promise.race([loadMediaPipe(), new Promise((_, no) => setTimeout(() => no(new Error('it took too long to load')), 20000))]);
        if (!this.stream) return;                         // switched off while loading
        this.engine = 'mediapipe'; this.why = '';
        this.mp.attach(video, this.onStatus);
        return;
      } catch (e) {
        this.why = (e && e.message) || String(e);
        console.warn('MediaPipe could not start, using the colour tracker:', this.why);
      }
    } else this.why = 'chosen in the camera window';
    if (!this.stream) return;
    this.engine = 'skin';
    this.skin.attach(video, view, this.onStatus);
  }
  stop() {
    this.mp.detach(); this.skin.detach();
    if (this.stream) this.stream.getTracks().forEach((t) => t.stop());
    this.stream = null;
    if (this.video) this.video.srcObject = null;
    this.engine = 'off';
  }
  calibrate() { if (this.engine === 'mediapipe') this.mp.calibrate(); else this.skin.calibrate(); }
  overlay() { return this.engine === 'mediapipe' ? this.mp.overlay() : Object.assign({ engine: this.engine }, this.skin.overlay()); }
  label() {
    if (this.engine === 'mediapipe') return 'MediaPipe hand tracking';
    if (this.engine === 'skin') return 'Simple colour tracking' + (this.why ? ` (MediaPipe: ${this.why})` : '');
    return '';
  }
}

window.ScaraHand = new HandControl();
window.ScaraHandMP = { loadMediaPipe, curled, pinchRatio, palmCentre };   // for the tests
})();
