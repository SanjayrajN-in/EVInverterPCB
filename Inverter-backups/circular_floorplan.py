"""Re-floorplan Inverter.kicad_pcb onto a circular board with a central shaft hole.

usage: python circ.py SRC.kicad_pcb DST.kicad_pcb OD_mm
"""
import math, sys
import pcbnew
from shapely.geometry import Point, Polygon, box
from shapely.affinity import rotate, translate
from shapely.strtree import STRtree

SRC, DST, OD = sys.argv[1], sys.argv[2], float(sys.argv[3])
BAR_Y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0   # barrier offset from centre (- = up)
R = OD / 2
CX, CY = 100.0, 100.0
CYB = CY + BAR_Y
SHAFT_D = 14.0
HOLE_R = 8.0          # Ø16 shaft clearance hole
BAR = 3.5             # isolation strip half-width (7 mm copper-free)
EDGE = 1.5            # courtyard to board-edge
MAG_R = 12.5          # assumed ring magnet OD 24 mm -> no parts inside this radius
ENC_R = 15.0          # MA732 centre radius (just outside magnet rim)
MH_R = R - 5.0        # mounting-hole pitch radius
MH_ANG = (-135, -45, 45, 135)
BIG_NETS = {'GND', 'GND1', '+3.3V', '+5V', '+1V2', '+5VA', '+15V', '+12V', 'AD_REF'}
ISO = ['IC1', 'IC2', 'IC3', 'IC4', 'IC6', 'U2', 'PS3', 'PS4']

mm, tomm = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(SRC)
fps = {f.GetReference(): f for f in b.GetFootprints()}


def nets_of(f):
    return [p.GetNetname() for p in f.Pads() if p.GetNetname() and not p.GetNetname().startswith('unconnected')]


# ---------------------------------------------------------------- domains
parent = {r: r for r in fps}
def find(x):
    while parent[x] != x:
        parent[x] = parent[parent[x]]; x = parent[x]
    return x
net_parts = {}
for r, f in fps.items():
    for n in set(nets_of(f)):
        net_parts.setdefault(n, set()).add(r)
for n, rs in net_parts.items():
    rs = [r for r in rs if r not in ISO]
    for r in rs[1:]:
        parent[find(r)] = find(rs[0])
dom = {}
for r in fps:
    if r in ISO:
        continue
    root = find(r)
    dom[r] = 'HV' if root == find('U10') else 'LV' if root == find('IC8') else '?'
dom['P2'] = 'HV'   # phase outputs: only touch the ACS724 primaries
net_dom = {}
for n, rs in net_parts.items():
    ds = {dom[r] for r in rs if r in dom}
    net_dom[n] = ds.pop() if len(ds) == 1 else '?'
assert '?' not in dom.values(), [r for r, d in dom.items() if d == '?']


# ---------------------------------------------------------------- geometry
def local_rect(f):
    """Courtyard (or body) bbox at orientation 0, relative to the footprint origin."""
    f.SetOrientationDegrees(0)
    o = f.GetPosition()
    cy = f.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else f.GetBoundingBox(False, False)
    if not cy.OutlineCount():
        bb.Inflate(mm(0.25))
    for p in f.Pads():   # some courtyards are smaller than their own pads (e.g. SW1)
        pb = p.GetBoundingBox()
        pb.Inflate(mm(0.25))
        bb.Merge(pb)
    return box(tomm(bb.GetLeft() - o.x), tomm(bb.GetTop() - o.y),
               tomm(bb.GetRight() - o.x), tomm(bb.GetBottom() - o.y))

RECT = {r: local_rect(f) for r, f in fps.items()}

def shape(ref, x, y, th):
    # KiCad orientation is CCW on a y-down screen == shapely rotate by -th
    return translate(rotate(RECT[ref], -th, origin=(0, 0)), x, y)

def put(ref, x, y, th):
    f = fps[ref]
    f.SetOrientationDegrees(th)
    f.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))

def pad_xy(ref):
    return [(tomm(p.GetPosition().x), tomm(p.GetPosition().y), p.GetNetname()) for p in fps[ref].Pads()]

def polar(r, phi):
    return CX + r * math.cos(math.radians(phi)), CY + r * math.sin(math.radians(phi))

# sanity check of the rotation convention
put('IC1', 0, 0, 90)
p1 = [p for p in fps['IC1'].Pads() if p.GetNumber() == '1'][0].GetPosition()
assert abs(tomm(p1.x) - (-4.45)) < 0.05 and abs(tomm(p1.y) - 4.72) < 0.05, (tomm(p1.x), tomm(p1.y))

BOARD_OK = Point(CX, CY).buffer(R - EDGE, 256)
NOGO = Point(CX, CY).buffer(MAG_R).union(Point(CX, CY).buffer(HOLE_R + 1.0))
HOLES = [Point(*polar(MH_R, a)).buffer(3.6) for a in MH_ANG]
placed = {}   # ref -> polygon (un-inflated)
placed_buf = {}  # ref -> polygon inflated by half its gap
fixed_obstacles = list(HOLES)

GAP_IC = float(__import__('os').environ.get('GAP_IC', 2.0))
GAP_SMALL = float(__import__('os').environ.get('GAP_SMALL', 1.0))

def gap_for(ref):
    # room for fan-out and 2-layer routing channels, not just assembly clearance
    return GAP_IC if len(fps[ref].Pads()) > 8 else GAP_SMALL

def commit(ref, x, y, th):
    put(ref, x, y, th)
    placed[ref] = shape(ref, x, y, th)
    placed_buf[ref] = placed[ref].buffer(gap_for(ref) / 2, join_style=2)


# ---------------------------------------------------------------- anchors
def straddle(ref, x, edge=None):
    """Place an isolator on the barrier: LV pads above the strip, HV pads below, gap centred."""
    for th in (0, 90, 180, 270):
        put(ref, 0, 0, th)
        pads = [(py, net_dom.get(n, '?')) for _, py, n in pad_xy(ref) if n in net_dom]
        lv = [py for py, d in pads if d == 'LV']
        hv = [py for py, d in pads if d == 'HV']
        if lv and hv and max(lv) < min(hv):
            y = CYB - (max(lv) + min(hv)) / 2
            if edge is not None:   # x is where the near edge of the courtyard goes
                x0, _, x1, _ = shape(ref, 0, 0, th).bounds
                x = edge - x0 if edge > 0 else edge - x1
            commit(ref, CX + x, y, th)
            return
    raise RuntimeError(f'cannot orient {ref} across barrier')

def rim_pose(ref, phi):
    th = -(phi + 90)
    for r in [R - 2 - i * 0.25 for i in range(200)]:
        x, y = polar(r, phi)
        if BOARD_OK.contains(shape(ref, x, y, th)):
            return x, y, th
    return None

LOCAL_PADS = {}
def pad_at(ref, num, x, y, th):
    """Board position of pad `num` if `ref` were placed at (x, y, th)."""
    if ref not in LOCAL_PADS:
        put(ref, 0, 0, 0)
        LOCAL_PADS[ref] = {p.GetNumber(): (tomm(p.GetPosition().x), tomm(p.GetPosition().y)) for p in fps[ref].Pads()}
    px, py = LOCAL_PADS[ref][num]
    c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
    return x + px * c + py * s_, y - px * s_ + py * c

def place_dc_link():
    """P4 on the rim with the series film caps C54/C55 right behind it, then on to the IPM.

    Chain: P4.1(+DC) - C54.1 | C54.2 - mid - C55.1 | C55.2 - P4.3(-DC).
    Cost is dominated by terminal->cap distance; IPM P pin / shunt distance is a secondary pull.
    """
    from shapely.ops import unary_union
    from shapely.prepared import prep
    u = placed['U10'].bounds
    reserve = box(u[0], u[3], u[2], u[3] + float(ENV.get('RESERVE', 14)))         # bootstrap + gate RC under the IPM control row
    obst = unary_union([p.buffer(1.5) for p in placed.values()] + HOLES + [reserve])
    hv_ok = prep(BOARD_OK.intersection(box(CX - R, CYB + BAR + 0.6, CX + R, CY + R)).difference(NOGO))
    obst_p = prep(obst)
    U10P = (tomm([p for p in fps['U10'].Pads() if p.GetNumber() == '23'][0].GetPosition().x),
            tomm([p for p in fps['U10'].Pads() if p.GetNumber() == '23'][0].GetPosition().y))
    U10N = (tomm([p for p in fps['U10'].Pads() if p.GetNumber() == '18'][0].GetPosition().x),
            tomm([p for p in fps['U10'].Pads() if p.GetNumber() == '18'][0].GetPosition().y))
    d = lambda a, b_: math.hypot(a[0] - b_[0], a[1] - b_[1])
    grid = [(CX + gx, CY + gy) for gx in range(-int(R), int(R), 2) for gy in range(0, int(R), 2)]
    best = None
    # the terminal body is symmetric about its pin row, so both pin orders are valid
    poses = [(phi, p) for phi in range(20, 165, 5) for p in [rim_pose('P4', phi)] if p]
    poses += [(phi, (x, y, th + 180)) for phi, (x, y, th) in list(poses)]
    for phi, pose in poses:
        s4 = shape('P4', *pose)
        if not hv_ok.contains(s4) or obst_p.intersects(s4):
            if ENV.get('DBG'): print('phi', phi, 'P4 blocked', [k for k in placed if placed[k].buffer(1.5).intersects(s4)], hv_ok.contains(s4))
            continue
        t_pos, t_neg = pad_at('P4', '1', *pose), pad_at('P4', '3', *pose)
        DBG = ENV.get('DBG')
        o2 = prep(obst.union(s4.buffer(1.5)))
        c54 = []
        for gx, gy in grid:
            for th in (0, 90, 180, 270):
                p1 = pad_at('C54', '1', gx, gy, th)
                if d(p1, t_pos) > 45:
                    continue
                sc = shape('C54', gx, gy, th)
                if hv_ok.contains(sc) and not o2.intersects(sc):
                    c54.append((1.5 * d(p1, t_pos) + 0.2 * d(p1, U10P), gx, gy, th, sc))
        if DBG: print('phi', phi, 'C54 candidates', len(c54))
        phi_best = None
        for c1, gx, gy, th, sc in sorted(c54, key=lambda t: t[0])[:6]:
            mid = pad_at('C54', '2', gx, gy, th)
            o3 = prep(obst.union(s4.buffer(1.5)).union(sc.buffer(1.5)))
            for hx, hy in grid:
                if math.hypot(hx - gx, hy - gy) > 40:
                    continue
                for th2 in (0, 90, 180, 270):
                    n2 = pad_at('C55', '2', hx, hy, th2)
                    m1 = pad_at('C55', '1', hx, hy, th2)
                    cost = c1 + 1.5 * d(n2, t_neg) + 0.7 * d(mid, m1)   # both cap terminals hug P4
                    if best and cost >= best[0]:
                        continue
                    s5 = shape('C55', hx, hy, th2)
                    if hv_ok.contains(s5) and not o3.intersects(s5):
                        best = (cost, pose, (gx, gy, th), (hx, hy, th2))
                        phi_best = (round(cost, 1), round(d(pad_at('C54', '1', gx, gy, th), t_pos), 1), round(d(n2, t_neg), 1))
        if DBG: print('phi', phi, 'best-so-far', phi_best)
    assert best, 'no room for DC link'
    print(f'DC link: P4 at rim pose {tuple(round(v, 1) for v in best[1])}, cost {best[0]:.1f}')
    commit('P4', *best[1])
    commit('C54', *best[2])
    commit('C55', *best[3])

def at_rim(ref, phi):
    """Connector tangential to the rim, as far out as the edge clearance allows."""
    th = -(phi + 90)
    for r in [R - 2 - i * 0.25 for i in range(200)]:
        x, y = polar(r, phi)
        s = shape(ref, x, y, th)
        if Point(CX, CY).buffer(R - EDGE).contains(s):
            commit(ref, x, y, th)
            return
    raise RuntimeError(ref)

# encoder: just outside the magnet rim, LV side
commit('U6', *polar(ENC_R, -90), 0)
commit('C9', CX + 3.8, CY - ENC_R, 90)   # MA732 1 uF decoupling, datasheet: close to the sensor
commit('IC8', CX, CY - 32, 0)
# power stage: IPM with its power row (NU NU NU W V U P) facing the ACS724 row on the barrier
ENV = __import__('os').environ
U10_X = float(ENV.get('U10_X', -25))
commit('U10', CX + U10_X, CYB + 25.5, 0)
def pack(refs, start, step):
    """Line isolators up along the barrier from the shaft outwards, `step` mm apart."""
    e = start
    for ref in refs:
        straddle(ref, 0, edge=e)
        x0, _, x1, _ = placed[ref].bounds
        e = (x1 - CX + step) if start > 0 else (x0 - CX - step)

pack(['IC1', 'IC2', 'IC3'], -(HOLE_R + 3.5), 2.5)                  # U V W, mirrors U10 pin order
pack(['IC4', 'U2', 'IC6', 'PS3', 'PS4'], HOLE_R + 2.0, 1.5)
at_rim('P1', -112)   # CAN
at_rim('P3', -68)    # 12 V in
at_rim('P2', float(ENV.get('P2_PHI', 155)))    # U V W out
place_dc_link()

if '-v' in sys.argv:
    for a in sorted(placed):
        x0, y0, x1, y1 = placed[a].bounds
        print(f'{a:5} x {x0-CX:7.1f} {x1-CX:7.1f}  y {y0-CY:7.1f} {y1-CY:7.1f}  r_max {max(math.hypot(x-CX, y-CY) for x, y in placed[a].exterior.coords):5.1f}')
for a in placed:
    assert BOARD_OK.contains(placed[a]), f'{a} off board'
    for c in placed:
        if a < c and placed[a].intersects(placed[c]):
            raise RuntimeError(f'anchor overlap {a} {c}')

if '--anchors-only' in sys.argv:
    print('anchors ok'); sys.exit(0)

# mounting holes (board-only footprints)
MH_LIB = '/usr/share/kicad/footprints/MountingHole.pretty'
for i, a in enumerate(MH_ANG):
    h = pcbnew.FootprintLoad(MH_LIB, 'MountingHole_3.2mm_M3')
    h.SetReference(f'H{i+1}')
    h.SetBoardOnly(True)
    h.SetExcludedFromBOM(True)
    h.SetExcludedFromPosFiles(True)
    b.Add(h)
    x, y = polar(MH_R, a)
    h.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))

# ---------------------------------------------------------------- greedy placement of the rest
ANCHORS = set(placed)
rest = [r for r in fps if r not in placed]
# parts that only touch rails: pin them to the IC they decouple
OWNER = {}
for r in rest:
    small = [n for n in nets_of(fps[r]) if n not in BIG_NETS]
    if not small:
        rails = set(nets_of(fps[r]))
        sheet = fps[r].GetSheetfile()
        cands = [a for a in fps if a != r and len(fps[a].Pads()) > 4 and fps[a].GetSheetfile() == sheet
                 and rails & set(nets_of(fps[a]))]
        OWNER[r] = max(cands, key=lambda a: len(rails & set(nets_of(fps[a]))) * 100 + len(fps[a].Pads())) if cands else 'IC8'


def feasible(ref, s):
    if not BOARD_OK.contains(s) or s.intersects(NOGO):
        return False
    minx, miny, maxx, maxy = s.bounds
    if dom[ref] == 'LV' and maxy > CYB - BAR - 0.4:
        return False
    if dom[ref] == 'HV' and miny < CYB + BAR + 0.4:
        return False
    g = gap_for(ref)
    sb = s.buffer(g / 2, join_style=2)
    for o in fixed_obstacles:
        if sb.intersects(o):
            return False
    bx0, by0, bx1, by1 = sb.bounds
    for ps in placed_buf.values():
        px0, py0, px1, py1 = ps.bounds
        if px0 > bx1 or px1 < bx0 or py0 > by1 or py1 < by0:
            continue
        if sb.intersects(ps):
            return False
    return True

def net_targets(ref):
    """Weighted points this part wants to be near."""
    pts = []
    for n in set(nets_of(fps[ref])):
        if n in BIG_NETS:
            continue
        others = [(x, y) for k in placed for x, y, nn in pad_xy(k) if nn == n]
        if others:
            w = 1.0 / max(1, len(net_parts[n]) - 1)
            pts.append((sum(p[0] for p in others) / len(others), sum(p[1] for p in others) / len(others), w))
    if ref in OWNER and OWNER[ref] in placed:
        own = [(x, y) for x, y, n in pad_xy(OWNER[ref]) if n in set(nets_of(fps[ref]))]
        if own:
            pts.append((sum(p[0] for p in own) / len(own), sum(p[1] for p in own) / len(own), 1.0))
    return pts

def place(ref):
    pts = net_targets(ref)
    if not pts:   # nothing placed it talks to yet: sit with its own sheet on its own side
        same = [placed[k].centroid for k in placed
                if dom.get(k) == dom[ref] and fps[k].GetSheetfile() == fps[ref].GetSheetfile()]
        same = same or [placed[k].centroid for k in placed if dom.get(k) == dom[ref]]
        pts = [(sum(p.x for p in same) / len(same), sum(p.y for p in same) / len(same), 1.0)]
    W = sum(w for _, _, w in pts)
    tx, ty = sum(x * w for x, y, w in pts) / W, sum(y * w for x, y, w in pts) / W
    best = None
    for i in range(0, 160):
        rr = i * 0.5
        n = max(1, int(2 * math.pi * rr / 0.6))
        for k in range(n):
            a = 2 * math.pi * k / n
            x, y = tx + rr * math.cos(a), ty + rr * math.sin(a)
            for th in (0, 90):
                s = shape(ref, x, y, th)
                if feasible(ref, s):
                    put(ref, x, y, th)
                    cost = sum(w * min(math.hypot(px - qx, py - qy) for px, py, _ in pad_xy(ref))
                               for qx, qy, w in pts)
                    if best is None or cost < best[0]:
                        best = (cost, x, y, th)
        if best and rr > 3:
            break
    if not best:
        raise RuntimeError(f'no room for {ref}')
    commit(ref, best[1], best[2], best[3])

def attach(ref):
    nets = set(nets_of(fps[ref])) - BIG_NETS
    s = sum(1.0 / max(1, len(net_parts[n]) - 1) for n in nets
            if any(k in placed for k in net_parts[n] if k != ref))
    if ref in OWNER and OWNER[ref] in placed:
        s += 0.5
    return s

todo = set(rest)
order = []
while todo:
    # most-attached first; ties -> bigger part first
    ref = max(todo, key=lambda r: (attach(r), RECT[r].area))
    place(ref)
    todo.discard(ref)
    order.append(ref)

# ---------------------------------------------------------------- outline, keepouts, zones
for d in list(b.GetDrawings()):
    if d.GetLayer() == pcbnew.Edge_Cuts:
        b.Remove(d)

def circle(r, layer, width=0.05):
    c = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_CIRCLE)
    c.SetCenter(pcbnew.VECTOR2I(mm(CX), mm(CY)))
    c.SetEnd(pcbnew.VECTOR2I(mm(CX + r), mm(CY)))
    c.SetLayer(layer)
    c.SetWidth(mm(width))
    b.Add(c)

circle(R, pcbnew.Edge_Cuts)
circle(HOLE_R, pcbnew.Edge_Cuts)
circle(SHAFT_D / 2, pcbnew.F_Fab, 0.1)
circle(MAG_R - 0.5, pcbnew.F_Fab, 0.1)   # assumed ring-magnet OD 24

def text(s, x, y, layer=pcbnew.F_Fab, size=1.0):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(s)
    t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    b.Add(t)

text('SHAFT Ø14 / HOLE Ø16', CX, CY + HOLE_R + 2)
text('RING MAGNET Ø24 (ASSUMED)', CX, CY - MAG_R - 0.6, size=0.8)

def poly_pts(geom, n=None):
    return [pcbnew.VECTOR2I(mm(x), mm(y)) for x, y in list(geom.exterior.coords)[:-1]]

def zone(geom, layers, net=None, keepout=None, name='', priority=0):
    z = pcbnew.ZONE(b)
    ls = pcbnew.LSET()
    for l in layers:
        ls.AddLayer(l)
    z.SetLayerSet(ls)
    ol = z.Outline()
    ol.NewOutline()
    for p in poly_pts(geom):
        ol.Append(p.x, p.y)
    if keepout:
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks('tracks' in keepout)
        z.SetDoNotAllowVias('vias' in keepout)
        z.SetDoNotAllowZoneFills('pour' in keepout)
        z.SetDoNotAllowPads('pads' in keepout)
        z.SetDoNotAllowFootprints('footprints' in keepout)
    else:
        z.SetNet(b.FindNet(net))
        z.SetAssignedPriority(priority)
        z.SetLocalClearance(mm(0.3))
        z.SetMinThickness(mm(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    if name:
        z.SetZoneName(name)
    b.Add(z)
    return z

CU = (pcbnew.F_Cu, pcbnew.B_Cu)
disk = Point(CX, CY).buffer(R + 1, 128)
zone(box(CX - R - 1, CYB - BAR, CX + R + 1, CYB + BAR).intersection(disk),
     CU, keepout={'tracks', 'vias', 'pour'}, name='ISOLATION_BARRIER')
zone(Point(CX, CY).buffer(MAG_R, 64), [pcbnew.F_Cu], keepout={'footprints'}, name='MAGNET_KEEPOUT')
inner = Point(CX, CY).buffer(R - 0.5, 128).difference(Point(CX, CY).buffer(HOLE_R + 0.5, 64))
zone(inner.intersection(box(CX - R, CY - R, CX + R, CYB - BAR)), [pcbnew.B_Cu], net='GND', name='GND_LV')
zone(inner.intersection(box(CX - R, CYB + BAR, CX + R, CY + R)), [pcbnew.B_Cu], net='GND1', name='GND1_HV')

b.Save(DST)

# ---------------------------------------------------------------- report
area = math.pi * (R ** 2 - HOLE_R ** 2)
used = sum(p.area for p in placed.values())
print(f'OD {OD} mm: placed {len(placed)} parts, courtyard area {used:.0f} / {area:.0f} mm^2 = {100*used/area:.0f}%')
for side in ('LV', 'HV'):
    a = sum(placed[r].area for r in placed if dom.get(r) == side)
    print(f'  {side}: {a:.0f} mm^2')
