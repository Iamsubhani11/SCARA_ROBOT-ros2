/* Hand control with the phone's front camera (or a laptop webcam) - no internet, no model files.
 *
 * Your hand works like the left joystick: hold it in the middle of the picture and the arm
 * stays still; move it left / right / up / down and the arm moves that way, faster the further
 * you go.  Make a fist to close the gripper, open your hand to open it.
 *
 * How it sees the hand: skin colour.  At the start you hold your open palm in the square for
 * two seconds; the app learns the colour of your skin under your light and the size of your
 * open hand.  After that it follows the skin-coloured patch that is nearest to where your hand
 * was (so your face, if it is in the picture, is ignored), and compares its size with the open
 * hand to tell a fist from an open palm.  Plain background and even light work best.
 */
(() => {
'use strict';

const W = 80, H = 60;                 // analysis size (pixels)
const BOX = 0.36;                     // calibration square (fraction of the picture height)

class HandTracker {
  constructor() {
    this.video = null; this.view = null; this.stream = null;
    this.work = document.createElement('canvas');
    this.work.width = W; this.work.height = H;
    this.wctx = this.work.getContext('2d', { willReadFrequently: true });
    this.state = 'off';               // off | starting | calibrating | tracking | lost
    this.out = { x: 0, y: 0, fist: false, seen: false };
    this.onStatus = () => {};
    this.timer = 0;
  }

  async start(video, view, onStatus) {
    this.video = video; this.view = view; this.onStatus = onStatus || (() => {});
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) throw new Error('This browser cannot use the camera');
    this.state = 'starting';
    this.stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: 'user', width: { ideal: 320 }, height: { ideal: 240 } }, audio: false });
    video.srcObject = this.stream;
    video.muted = true; video.playsInline = true;
    await video.play();
    this.calibrate();
    clearInterval(this.timer);
    this.timer = setInterval(() => this.tick(), 66);
  }

  stop() {
    clearInterval(this.timer); this.timer = 0;
    if (this.stream) this.stream.getTracks().forEach((t) => t.stop());
    this.stream = null;
    if (this.video) this.video.srcObject = null;
    this.state = 'off';
    this.out = { x: 0, y: 0, fist: false, seen: false };
  }

  /* where to draw things in the big camera view (fractions of the mirrored picture) */
  overlay() {
    const deadR = 0.18 * 0.32;                              // the still zone: |x| < 0.18 of the 0.32-wide travel
    return { state: this.state, box: this.box, home: this.home ? { x: this.home.x / W, y: this.home.y / H } : null,
      deadR, travel: 0.32, blob: this.state === 'tracking' ? this.blob : null, out: this.out,
      calLeft: this.state === 'calibrating' && this.cal ? Math.max(0, 2.0 - (performance.now() - this.cal.t0) / 1000) : 0 };
  }

  calibrate() {
    this.state = 'calibrating';
    this.cal = { t0: performance.now(), cr: [], cb: [], area: [] };
    this.skin = null; this.last = null; this.fistSince = 0; this.openSince = 0;
    this.onStatus('Hold your open hand in the square…');
  }

  /* one frame: mirror (so the picture is like a mirror), YCrCb skin mask, blobs */
  tick() {
    const v = this.video;
    if (!v || v.readyState < 2) return;
    const c = this.wctx;
    c.save(); c.translate(W, 0); c.scale(-1, 1); c.drawImage(v, 0, 0, W, H); c.restore();
    const img = c.getImageData(0, 0, W, H), px = img.data;
    const n = W * H, Y = new Float32Array(n), Cr = new Float32Array(n), Cb = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const r = px[4 * i], g = px[4 * i + 1], b = px[4 * i + 2];
      Y[i] = 0.299 * r + 0.587 * g + 0.114 * b;
      Cr[i] = 128 + 0.5 * r - 0.4187 * g - 0.0813 * b;
      Cb[i] = 128 - 0.1687 * r - 0.3313 * g + 0.5 * b;
    }
    const bx0 = Math.round(W / 2 - BOX * H / 2), bx1 = Math.round(W / 2 + BOX * H / 2);
    const by0 = Math.round(H / 2 - BOX * H / 2), by1 = Math.round(H / 2 + BOX * H / 2);
    this.box = [bx0 / W, by0 / H, bx1 / W, by1 / H];        // for the big camera view (0..1, mirrored image)
    if (this.state === 'calibrating') {
      // the middle of the square is the palm: learn its colour
      for (let y = by0 + 3; y < by1 - 3; y++) for (let x = bx0 + 3; x < bx1 - 3; x++) {
        const i = y * W + x; if (Y[i] > 35) { this.cal.cr.push(Cr[i]); this.cal.cb.push(Cb[i]); }
      }
      const left = 2.0 - (performance.now() - this.cal.t0) / 1000;
      this.onStatus(left > 0 ? `Hold your open hand in the square… ${left.toFixed(1)} s` : 'Learning…');
      if (left <= 0) {
        const med = (a) => { const s = a.slice().sort((p, q) => p - q); return s[Math.floor(s.length / 2)] || 0; };
        const mad = (a, m) => med(a.map((v2) => Math.abs(v2 - m)));
        const cr = med(this.cal.cr), cb = med(this.cal.cb);
        this.skin = { cr, cb, dcr: Math.max(7, 2.6 * mad(this.cal.cr, cr) + 4), dcb: Math.max(7, 2.6 * mad(this.cal.cb, cb) + 4) };
        // plausible skin? (otherwise the square was empty or the light is very coloured)
        if (cr < 128 || cr > 185 || cb < 70 || cb > 135) {
          this.onStatus('No hand in the square - try again'); this.calibrate(); return;
        }
        const mask = this.mask(Y, Cr, Cb);
        const blob = this.blobs(mask).find((bl) => bl.cx >= bx0 && bl.cx <= bx1 && bl.cy >= by0 && bl.cy <= by1);
        if (!blob) { this.onStatus('No hand in the square - try again'); this.calibrate(); return; }
        this.openArea = blob.area;
        this.home = { x: W / 2, y: H / 2 };
        this.last = { x: blob.cx, y: blob.cy };
        this.state = 'tracking';
        this.onStatus('Hand control on: move your hand to move the arm, fist = close');
      }
      this.draw(img, null, [bx0, by0, bx1, by1]);
      return;
    }
    const mask = this.mask(Y, Cr, Cb);
    const blobs = this.blobs(mask);
    let best = null, bd = 1e9;
    for (const bl of blobs) {
      if (bl.area < Math.max(25, 0.25 * this.openArea)) continue;
      const d = this.last ? Math.hypot(bl.cx - this.last.x, bl.cy - this.last.y) : 0;
      if (d < bd) { bd = d; best = bl; }
    }
    this.blob = best ? { x: best.cx / W, y: best.cy / H, r: Math.sqrt(best.area / Math.PI) / W } : null;
    if (!best || bd > W * 0.45) {
      this.out = { x: 0, y: 0, fist: this.out.fist, seen: false };
      if (this.state !== 'lost') this.onStatus('Hand lost - show your hand again');
      this.state = 'lost';
      this.draw(img, mask, null);
      return;
    }
    if (this.state === 'lost') this.onStatus('Hand control on');
    this.state = 'tracking';
    this.last = { x: best.cx, y: best.cy }; this._lastArea = best.area;
    const ratio = best.area / this.openArea, now = performance.now();
    // a fist is about half the open hand; hysteresis + a short hold so it does not flicker
    if (ratio < 0.6) { this.openSince = 0; if (!this.fistSince) this.fistSince = now; }
    else if (ratio > 0.78) { this.fistSince = 0; if (!this.openSince) this.openSince = now; }
    let fist = this.out.fist;
    if (this.fistSince && now - this.fistSince > 300) fist = true;
    if (this.openSince && now - this.openSince > 300) fist = false;
    const nx = (best.cx - this.home.x) / (W * 0.32), ny = -(best.cy - this.home.y) / (H * 0.32);
    this.out = { x: Math.max(-1, Math.min(1, nx)), y: Math.max(-1, Math.min(1, ny)), fist, seen: true };
    this.draw(img, mask, null, best);
  }

  mask(Y, Cr, Cb) {
    const s = this.skin, m = new Uint8Array(W * H);
    for (let i = 0; i < m.length; i++) {
      m[i] = Y[i] > 30 && Math.abs(Cr[i] - s.cr) < s.dcr && Math.abs(Cb[i] - s.cb) < s.dcb ? 1 : 0;
    }
    // close small gaps: a pixel with 5+ skin neighbours counts as skin
    const o = new Uint8Array(m.length);
    for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
      const i = y * W + x;
      const k = m[i - W - 1] + m[i - W] + m[i - W + 1] + m[i - 1] + m[i + 1] + m[i + W - 1] + m[i + W] + m[i + W + 1];
      o[i] = (m[i] && k >= 2) || k >= 5 ? 1 : 0;
    }
    return o;
  }

  blobs(m) {
    const lab = new Int32Array(m.length), out = [], st = [];
    let id = 0;
    for (let i = 0; i < m.length; i++) {
      if (!m[i] || lab[i]) continue;
      id += 1; let area = 0, sx = 0, sy = 0;
      st.push(i); lab[i] = id;
      while (st.length) {
        const j = st.pop(), x = j % W, y = (j / W) | 0;
        area += 1; sx += x; sy += y;
        if (x > 0 && m[j - 1] && !lab[j - 1]) { lab[j - 1] = id; st.push(j - 1); }
        if (x < W - 1 && m[j + 1] && !lab[j + 1]) { lab[j + 1] = id; st.push(j + 1); }
        if (y > 0 && m[j - W] && !lab[j - W]) { lab[j - W] = id; st.push(j - W); }
        if (y < H - 1 && m[j + W] && !lab[j + W]) { lab[j + W] = id; st.push(j + W); }
      }
      if (area >= 12) out.push({ area, cx: sx / area, cy: sy / area });
    }
    return out.sort((a, b) => b.area - a.area);
  }

  /* small preview: the camera picture, the hand in blue, the still zone and the square */
  draw(img, mask, box, blob) {
    const cv = this.view;
    if (!cv) return;
    if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
    const g = cv.getContext('2d');
    const d = img.data;
    for (let i = 0; i < W * H; i++) {
      const on = mask && mask[i];
      d[4 * i] = on ? 40 : d[4 * i] * 0.8; d[4 * i + 1] = on ? 140 : d[4 * i + 1] * 0.8; d[4 * i + 2] = on ? 255 : d[4 * i + 2] * 0.8;
    }
    g.putImageData(img, 0, 0);
    g.lineWidth = 1;
    if (box) { g.strokeStyle = '#f2b705'; g.strokeRect(box[0] + 0.5, box[1] + 0.5, box[2] - box[0], box[3] - box[1]); return; }
    if (this.home) {
      g.strokeStyle = '#ffffffaa';
      g.beginPath(); g.arc(this.home.x, this.home.y, 0.18 * W * 0.32 * 2, 0, Math.PI * 2); g.stroke();
    }
    if (blob) {
      g.fillStyle = this.out.fist ? '#f2b705' : '#ffffff';
      g.beginPath(); g.arc(blob.cx, blob.cy, 3, 0, Math.PI * 2); g.fill();
    }
  }
}

window.ScaraHand = new HandTracker();
})();
