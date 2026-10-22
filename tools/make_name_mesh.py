#!/usr/bin/env python3
"""
Make the raised name plate that sits on top of the SCARA shoulder link.

    python3 tools/make_name_mesh.py                       # SUBHANI, default size
    python3 tools/make_name_mesh.py --text "MY NAME" --cap 18 --depth 1.5

The raised letters are merged INTO meshes/shoulder_link.stl, so the name is part of
the arm itself - deleting or forgetting another file does not remove it.  A second,
thin mesh, meshes/shoulder_name.stl, paints the tops of the letters white (same frame
and scale as the shoulder link, so it cannot drift).  Without that file the name is
still there, raised, in the arm's blue.
    python3 tools/make_name_mesh.py --remove     # back to the plain arm

The letters are real font outlines (fontTools), triangulated (ear clipping) and
extruded: they start 0.5 mm inside the top face (no gap, no z-fighting) and stand
--depth mm above it. The text is centred on the flat, straight part of the arm's
top face and reads along the arm, from the column towards the elbow.

To change it in Blender instead: File > Import > STL (meshes/shoulder_link.stl and
meshes/shoulder_name.stl, scale 1), edit the name object, File > Export > STL
(selection only, scale 1) back to meshes/shoulder_name.stl.  tools/blender_name.py
does the same from Blender's Python console (uses Blender's own Text object).

Needs: numpy and fontTools (pip install fonttools).
"""
import argparse
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if os.path.isdir('/opt/pysrc/fonttools/Lib'):
    sys.path.append('/opt/pysrc/fonttools/Lib')
from fontTools.pens.basePen import BasePen   # noqa: E402
from fontTools.ttLib import TTFont           # noqa: E402


# ── polygon triangulation (ear clipping with hole bridging, no extra packages) ──
def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_cross(p, q, a, b):
    """Proper intersection of segments pq and ab (touching at end points does not count)."""
    d1, d2 = _cross(a, b, p), _cross(a, b, q)
    d3, d4 = _cross(p, q, a), _cross(p, q, b)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _bridge(outer, hole, others):
    """Join a hole into the outer ring with a two-way cut from its right-most vertex."""
    m = int(np.argmax(hole[:, 0]))
    M = hole[m]
    edges = [(r[i], r[(i + 1) % len(r)]) for r in [outer, hole] + others for i in range(len(r))]
    for j in np.argsort(np.hypot(*(outer - M).T)):
        P = outer[j]
        if not any(_seg_cross(M, P, a, b) for a, b in edges):
            hole_rot = np.vstack([hole[m:], hole[:m + 1]])
            return np.vstack([outer[:j + 1], hole_rot, outer[j:]])
    raise RuntimeError('could not bridge a hole')


def _in_tri(p, a, b, c):
    return _cross(a, b, p) >= 0 and _cross(b, c, p) >= 0 and _cross(c, a, p) >= 0


def triangulate(outer, holes):
    """outer: (N,2) counter-clockwise, holes: list of clockwise rings -> list of 2-D triangles (ccw)."""
    poly = outer
    for k, h in enumerate(sorted(holes, key=lambda r: -r[:, 0].max())):
        poly = _bridge(poly, h, [x for x in holes if x is not h])
    pts = [tuple(p) for p in poly]
    idx = list(range(len(pts)))
    tris, guard = [], 0
    while len(idx) > 3 and guard < 10 * len(pts) ** 2:
        guard += 1
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if _cross(a, b, c) <= 1e-12:
                continue
            ok = True
            for j in idx:
                p = pts[j]
                if j in (i0, i1, i2) or p in (a, b, c):
                    continue
                if _in_tri(p, a, b, c):
                    ok = False
                    break
            if ok:
                tris.append((a, b, c))
                idx.pop(k)
                break
        else:                         # no ear found (degenerate) - drop the flattest vertex
            k = min(range(n), key=lambda k: abs(_cross(pts[idx[k - 1]], pts[idx[k]], pts[idx[(k + 1) % n]])))
            idx.pop(k)
    if len(idx) == 3 and _cross(pts[idx[0]], pts[idx[1]], pts[idx[2]]) > 0:
        tris.append(tuple(pts[i] for i in idx))
    return tris

# shoulder_link.stl -> shoulder_link frame:  p_link = p_mesh * 0.001 + MESH_OFFSET
MESH_OFFSET = np.array([-0.059954333, -0.059974857, -0.390000])
TOP_FACE_Z = 0.027          # link frame: flat top face of the arm (measured from the mesh)
FLAT_X = (0.066, 0.228)     # link frame: straight part of the arm (before the rounded elbow end)
FACE_HALF_WIDTH = 0.039     # link frame: half width of the arm
CENTER_X = 0.160            # link frame: visual centre of the arm, from the neck (0.055) to the tip (0.267)

FONTS = ['/usr/share/fonts/opentype/inter/InterDisplay-Bold.otf',
         '/usr/share/fonts/opentype/inter/Inter-Bold.otf',
         '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
         '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf']


class FlatPen(BasePen):
    """Collects glyph contours as polylines (curves flattened)."""

    def __init__(self, glyphset, steps=10):
        super().__init__(glyphset)
        self.contours, self.cur, self.steps = [], [], steps

    def _moveTo(self, p):
        self.cur = [p]

    def _lineTo(self, p):
        self.cur.append(p)

    def _curveToOne(self, p1, p2, p3):
        p0 = np.array(self.cur[-1], float)
        for t in np.linspace(0, 1, self.steps + 1)[1:]:
            q = ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * np.array(p1) + 3 * (1 - t) * t * t * np.array(p2) \
                + t ** 3 * np.array(p3)
            self.cur.append(tuple(q))

    def _qCurveToOne(self, p1, p2):
        p0 = np.array(self.cur[-1], float)
        for t in np.linspace(0, 1, self.steps + 1)[1:]:
            q = ((1 - t) ** 2) * p0 + 2 * (1 - t) * t * np.array(p1) + t * t * np.array(p2)
            self.cur.append(tuple(q))

    def _closePath(self):
        c = np.array(self.cur, float)
        if len(c) > 1 and np.allclose(c[0], c[-1]):
            c = c[:-1]
        if len(c) >= 3:
            self.contours.append(c)
        self.cur = []

    _endPath = _closePath


def area(c):
    x, y = c[:, 0], c[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def inside(pt, c):
    x, y = pt
    xs, ys = c[:, 0], c[:, 1]
    xj, yj = np.roll(xs, 1), np.roll(ys, 1)
    m = ((ys > y) != (yj > y)) & (x < (xj - xs) * (y - ys) / (yj - ys + 1e-30) + xs)
    return bool(np.count_nonzero(m) % 2)


def text_contours(text, font_path, spacing=0.06):
    font = TTFont(font_path)
    gs = font.getGlyphSet()
    cmap = font.getBestCmap()
    upm = font['head'].unitsPerEm
    cap = getattr(font['OS/2'], 'sCapHeight', 0) or 0.7 * upm
    x0, out = 0.0, []
    for ch in text:
        gname = cmap.get(ord(ch))
        if gname is None:
            continue
        pen = FlatPen(gs)
        gs[gname].draw(pen)
        for c in pen.contours:
            out.append(c + [x0, 0.0])
        x0 += gs[gname].width + spacing * upm
    return out, cap


def build(text, font_path, cap_mm, depth_mm, sink_mm=0.5, z_from=None):
    contours, cap_units = text_contours(text, font_path)
    if not contours:
        raise SystemExit('no glyphs for %r in %s' % (text, font_path))
    s = cap_mm / cap_units                       # font units -> mm
    contours = [c * s for c in contours]
    allp = np.vstack(contours)
    centre = (allp.min(0) + allp.max(0)) / 2
    contours = [c - centre for c in contours]
    size = allp.max(0) - allp.min(0)

    # outer contours vs holes: a contour is a hole if it lies inside an odd number of others
    depth = [sum(inside(c[0], o) for j, o in enumerate(contours) if j != i) for i, c in enumerate(contours)]
    outers = [i for i, d in enumerate(depth) if d % 2 == 0]
    holes = {i: [] for i in outers}
    for i, d in enumerate(depth):
        if d % 2 == 1:
            owners = [o for o in outers if inside(contours[i][0], contours[o])]
            owner = min(owners, key=lambda o: abs(area(contours[o])))
            holes[owner].append(i)

    tris = []                                    # list of (3,3) arrays, mm, text frame (z up)
    z0, z1 = (-sink_mm if z_from is None else z_from), depth_mm

    def ring(c, ccw):
        return c if (area(c) > 0) == ccw else c[::-1]

    for o in outers:
        rings = [ring(contours[o], True)] + [ring(contours[h], False) for h in holes[o]]
        for pa, pb, pc in triangulate(rings[0], rings[1:]):
            tris.append([[*pa, z1], [*pb, z1], [*pc, z1]])      # top, normal +z
            tris.append([[*pa, z0], [*pc, z0], [*pb, z0]])      # bottom, normal -z
        for r in rings:                                          # walls (outer ccw, holes cw -> outward)
            for i in range(len(r)):
                p, q = r[i], r[(i + 1) % len(r)]
                tris.append([[*p, z0], [*q, z0], [*q, z1]])
                tris.append([[*p, z0], [*q, z1], [*p, z1]])
    return np.array(tris, float), size


def place(tris_mm, reverse=False):
    """Text frame (mm, x along text, y up the letters, z out of the face) -> shoulder mesh frame (mm)."""
    t = tris_mm.copy()
    if reverse:                                   # read from the elbow towards the column
        t[..., 0] *= -1
        t[..., 1] *= -1
    cx = CENTER_X
    link = np.empty_like(t)
    link[..., 0] = cx + t[..., 0] * 0.001         # along the arm
    link[..., 1] = t[..., 1] * 0.001              # across the arm
    link[..., 2] = TOP_FACE_Z + t[..., 2] * 0.001
    return (link - MESH_OFFSET) / 0.001


MERGED_TAG = b'SCARA shoulder_link with name, plain triangles='


def read_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return b[:80], a['v'].astype(np.float64)


def plain_shoulder(path):
    """The shoulder mesh without any name (a merged mesh remembers how many triangles were the arm)."""
    header, v = read_stl(path)
    if header.startswith(MERGED_TAG):
        n = int(header[len(MERGED_TAG):].split()[0])
        return v[:n]
    return v


def write_stl(path, tris, header=b'SCARA shoulder name plate'):
    tris = np.asarray(tris, np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
    with open(path, 'wb') as f:
        f.write(header[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        rec = np.zeros(len(tris), np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        rec['n'], rec['v'] = n, tris
        f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--text', default='SUBHANI')
    ap.add_argument('--font', default=None, help='TTF/OTF file (default: Inter Display Bold or DejaVu Sans Bold)')
    ap.add_argument('--cap', type=float, default=19.0, help='letter height in mm (default 19)')
    ap.add_argument('--depth', type=float, default=1.6, help='how far the letters stand out, mm (default 1.6)')
    ap.add_argument('--reverse', action='store_true', help='read from the elbow towards the column')
    ap.add_argument('--overlay-only', action='store_true',
                    help='old style: only write the white letters as a separate mesh, do not touch the arm mesh')
    ap.add_argument('--remove', action='store_true', help='take the name off again (plain shoulder mesh)')
    a = ap.parse_args()
    arm = os.path.join(ROOT, 'meshes', 'shoulder_link.stl')
    paint = os.path.join(ROOT, 'meshes', 'shoulder_name.stl')
    if a.remove:
        write_stl(arm, plain_shoulder(arm).reshape(-1, 3, 3), header=b'shoulder_link')
        if os.path.exists(paint):
            os.remove(paint)
        print('name removed: plain meshes/shoulder_link.stl, no shoulder_name.stl')
        return
    font = a.font or next((f for f in FONTS if os.path.exists(f)), None)
    if not font:
        raise SystemExit('no font found - pass --font /path/to/font.ttf')
    letters, size = build(a.text, font, a.cap, a.depth)
    flat_len = 2 * min(CENTER_X - FLAT_X[0], FLAT_X[1] - CENTER_X) * 1000
    if size[0] > flat_len - 4 or size[1] > 2 * FACE_HALF_WIDTH * 1000 - 12:
        raise SystemExit('text %.0f x %.0f mm does not fit on the %.0f x %.0f mm face - use a smaller --cap'
                         % (size[0], size[1], flat_len, 2 * FACE_HALF_WIDTH * 1000))
    # white "paint" on the top of the letters (the colour; the letters themselves are in the arm mesh)
    cap, _ = build(a.text, font, a.cap, a.depth + 0.15, z_from=a.depth - 0.25)
    write_stl(paint, place(cap, a.reverse))
    if not a.overlay_only:
        # the raised letters become part of the shoulder mesh itself: deleting shoulder_name.stl
        # only takes the white paint away, the name stays on the arm
        plain = plain_shoulder(arm).reshape(-1, 3, 3)
        merged = np.concatenate([plain, place(letters, a.reverse)])
        write_stl(arm, merged, header=MERGED_TAG + b'%d' % len(plain))
    print('"%s" %.1f x %.1f mm, %.1f mm high (font %s): %s'
          % (a.text, size[0], size[1], a.depth, os.path.basename(font),
             'white letters only (shoulder_name.stl)' if a.overlay_only else
             'letters merged into shoulder_link.stl + white paint in shoulder_name.stl'))


if __name__ == '__main__':
    main()
