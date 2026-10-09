"""DEMO ONLY (branch demo-on-grid, never merge): warp the off-grid parts onto the 8 mm grid in the web app.

The real fix is in the CAD (then `npm run catalog` / convert_meshes). This mocks it up: one warp per part moves the
sockets, bbox, footprint and tower sockets in src/data/parts.json AND the preview meshes (src/meshes, src/meshes_j),
so data and picture agree. Print meshes (3MF export) and thumbnails are not touched.

Target: every part moves the open end by whole grid steps (8 mm), except the deliberate one-third shifters
(Gerade80/100, Distanz65, SchieneDistanz95, cross tunnel 95), which stay as they are.

  Distanz46 + adapter        24.533 -> 24.000 mm (3 steps)              uniform stretch along x
  Gerade88 + adapter         46.933 -> 48.000 mm (6 steps)              uniform stretch along x
  Spirale_100-60 + adapter   run 55.467 -> 56.000 (7 steps)             outlet end face +0.533 in x, helix unchanged
  Kippwippe (all) + adapter  outlets (28.6 | +-36) -> (32 | +-40)       block wall x 10.6..15.27 stretched,
                                                                        outlet channels 4 mm longer along y
  Trichter_100-60 + adapter  exit along 101.333 -> 104 (13 steps),      exit channel (x > 38.2) stretched
                             sideways 27.783 -> 24 (3 steps)            and bent: +3.783 in y at the end face

Run once on the files as they are on main:  python3 tools/demo_on_grid.py
"""
import gzip, json, os, struct

ROOT = os.path.join(os.path.dirname(__file__), '..', 'src')
PARTS = os.path.join(ROOT, 'data', 'parts.json')


def pw(*knots):
    """Monotonic piecewise-linear map through (in, out) knots, slope 1 outside (a plain shift)."""
    def f(v):
        if v <= knots[0][0]: return v - knots[0][0] + knots[0][1]
        for (a, fa), (b, fb) in zip(knots, knots[1:]):
            if v <= b: return fa + (v - a) * (fb - fa) / (b - a)
        return v - knots[-1][0] + knots[-1][1]
    return f


ident = lambda v: v
FUNNEL_X0, FUNNEL_X1, FUNNEL_DY = 38.2, 63.2, 27.783 - 24.0
WARPS = {}   # part id / mesh file -> (x, y) -> (x', y')
for ids, fx, fy in [
    (['Distanz46-0_40-40_16mm', 'AdapterDistanz46_16mm'], pw((0, 0), (24.533, 24.0)), ident),
    (['Gerade88_60-40_16mm', 'AdapterGerade88_16mm'], pw((0, 0), (46.933, 48.0)), ident),
    (['Spirale_100-60_16mm', 'AdapterSpirale_100-60_16mm'], pw((0, 0), (12.8, 13.333), (15.467, 15.467)), ident),  # helix / column rim stay
    (['Kippwippe_120-60_16mm', 'Kippwippe_120-60_Links_16mm', 'Kippwippe_120-60_Rechts_16mm', 'AdapterKippwippe_120-60_16mm',
      'Kippwippe_Ansicht_Wippe_Links_16mm', 'Kippwippe_Ansicht_Wippe_Rechts_16mm'],
     pw((10.6, 10.6), (15.267, 18.667)), pw((-36, -40), (-13.333, -13.333), (13.333, 13.333), (36, 40))),
]:
    for i in ids: WARPS[i] = (lambda fx, fy: lambda x, y: (fx(x), fy(y)))(fx, fy)


def funnel(x, y):
    t = min(1.0, max(0.0, (x - FUNNEL_X0) / (FUNNEL_X1 - FUNNEL_X0)))
    return pw((FUNNEL_X0, FUNNEL_X0), (FUNNEL_X1, 65.867))(x), y + FUNNEL_DY * t
for i in ['Trichter_100-60_16mm', 'AdapterTrichter_100-60_16mm']: WARPS[i] = funnel


def r3(v): return round(v + 0.0, 3)


def warp_part(p, w):
    for port in p['ports']:
        port['p'][0], port['p'][1] = map(r3, w(port['p'][0], port['p'][1]))
    for side in ('bottom', 'top'):
        for s in p['vsock'][side]: s['c'][0], s['c'][1] = map(r3, w(*s['c']))
    (x0, y0, _), (x1, y1, _) = p['bbox']
    cx = [w(x, y)[0] for x in (x0, x1) for y in (y0, y1)]
    cy = [w(x, y)[1] for x in (x0, x1) for y in (y0, y1)]
    if p['id'] in ('Trichter_100-60_16mm', 'AdapterTrichter_100-60_16mm'):
        cy = [y0, y1]                       # only the exit channel moves, well inside the basin/chute y range
    p['bbox'][0][0], p['bbox'][0][1], p['bbox'][1][0], p['bbox'][1][1] = r3(min(cx)), r3(min(cy)), r3(max(cx)), r3(max(cy))
    p['foot'] = [p['bbox'][0][0], p['bbox'][0][1], p['bbox'][1][0], p['bbox'][1][1]]
    if p['family'] in ('spacer', 'straight'):
        p['length'] = p['phys']['L'] = r3(p['ports'][1]['p'][0] - p['ports'][0]['p'][0])


def warp_kbm3(path, w):
    data = open(path, 'rb').read()
    assert data[:4] == b'KBM3', path
    nV, nT, scale = struct.unpack('<IIf', data[4:16])
    b = gzip.decompress(data[16:])
    q = [[0] * nV for _ in range(3)]
    for a in range(3):
        acc = 0
        for i in range(nV):
            d = b[3 * nV + a * nV + i] << 8 | b[a * nV + i]
            acc = (acc + (d - 65536 if d >= 32768 else d) + 32768) % 65536 - 32768
            q[a][i] = acc
    for i in range(nV):
        x, y = w(q[0][i] * scale, q[1][i] * scale)
        q[0][i], q[1][i] = round(x / scale), round(y / scale)
        assert abs(q[0][i]) < 32767 and abs(q[1][i]) < 32767, path
    lo, hi = bytearray(), bytearray()
    for a in range(3):
        prev = 0
        for v in q[a]:
            d = (v - prev) & 0xffff; prev = v
            lo.append(d & 0xff); hi.append(d >> 8)
    body = bytes(lo) + bytes(hi) + b[6 * nV:]          # index planes unchanged (same vertex order)
    open(path, 'wb').write(data[:16] + gzip.compress(body, compresslevel=9, mtime=0))


d = json.load(open(PARTS, encoding='utf-8'))
assert next(p for p in d['parts'] if p['id'] == 'Distanz46-0_40-40_16mm')['length'] == 24.53, 'already applied - run on main files'
for p in d['parts']:
    if p['id'] in WARPS: warp_part(p, WARPS[p['id']])
with open(PARTS, 'w', encoding='utf-8') as f:
    f.write(json.dumps(d, separators=(',', ':'), ensure_ascii=False))

for name, w in WARPS.items():
    for folder in ('meshes', 'meshes_j'):
        path = os.path.join(ROOT, folder, name + '.kbm')
        if os.path.exists(path): warp_kbm3(path, w); print('warped', folder, name)
