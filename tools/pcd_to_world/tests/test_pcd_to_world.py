import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pcd_to_world as p  # noqa: E402


# ---------------------------------------------------------------- synthetic scene
SLOPE = 0.04


def plane_z(x, y):
    return SLOPE * x + np.where(y > 10.0, 0.10, 0.0)


def make_scene():
    rng = np.random.default_rng(1)
    g = np.arange(-20, 20, 0.05)
    X, Y = np.meshgrid(g, g)
    X = X.ravel()
    Y = Y.ravel()
    Z = plane_z(X, Y) + rng.normal(0, 0.003, X.shape)
    # box1 (4x4x3 at [-5,-1]^2) and box2 (12x12x6 at x[2,14], y[-18,-6]) : remove ground under, add roof+walls
    boxes = [((-5, -1, -5, -1), 3.0), ((2, 14, -18, -6), 6.0)]
    pts = []
    keep = np.ones(X.shape, bool)
    for (x0, x1, y0, y1), hh in boxes:
        inside = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        keep &= ~inside
        gx, gy = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05))
        roof_z = SLOPE * (x0 + x1) / 2 + hh
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, roof_z)])
        for t in np.arange(0, hh, 0.05):
            for xs in np.arange(x0, x1, 0.05):
                pts.append(np.array([[xs, y0, SLOPE * xs + t], [xs, y1, SLOPE * xs + t]]))
            for ys in np.arange(y0, y1, 0.05):
                pts.append(np.array([[x0, ys, SLOPE * x0 + t], [x1, ys, SLOPE * x1 + t]]))
    ground = np.c_[X[keep], Y[keep], Z[keep]]
    # pole radius 0.025, height 2 at (-12, 5)
    th = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    pole = []
    for t in np.arange(0, 2.0, 0.02):
        pole.append(np.c_[-12 + 0.025 * np.cos(th), 5 + 0.025 * np.sin(th),
                          np.full(8, SLOPE * -12 + t)])
    allp = np.vstack([ground] + pts + pole).astype(np.float32)
    return allp


@pytest.fixture(scope="module")
def scene():
    return make_scene()


@pytest.fixture(scope="module")
def result(scene, tmp_path_factory):
    out = tmp_path_factory.mktemp("out")
    cfg = p.Config()

    def chunks():
        for i in range(0, len(scene), 400_000):
            yield scene[i:i + 400_000]

    stats = p.run_pipeline(chunks, str(out), cfg, log=lambda *a: None)
    return out, stats


def read_obj(path):
    v, f = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                v.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("f "):
                f.append([int(t.split("/")[0]) for t in line.split()[1:4]])
    return np.array(v).reshape(-1, 3), np.array(f).reshape(-1, 3)


def test_ground_recovers_slope_and_step(scene):
    cfg = p.Config()
    origin, shape = p.grid_for_bounds(-20, 20, -20, 20, cfg.cell, pad=0.0)
    cellmin = p.compute_cellmin(lambda: iter([scene]), origin, shape, cfg.cell)
    gr = p.estimate_ground(cellmin, cfg)
    cx = origin[0] + (np.arange(shape[1]) + 0.5) * cfg.cell
    cy = origin[1] + (np.arange(shape[0]) + 0.5) * cfg.cell
    CX, CY = np.meshgrid(cx, cy)
    true = plane_z(CX, CY)
    err = np.abs(gr.ground - true)
    # away from the step line (+-0.3m) everything within 2 cm, including under boxes
    far = np.abs(CY - 10.0) > 0.3
    assert err[far].max() < 0.02
    # step is kept
    iy_lo = int((9.5 - origin[1]) / cfg.cell)
    iy_hi = int((10.5 - origin[1]) / cfg.cell)
    ix = int((0.0 - origin[0]) / cfg.cell)
    assert gr.ground[iy_hi, ix] - gr.ground[iy_lo, ix] == pytest.approx(0.10, abs=0.02)
    # boxes are non-ground (roof cells)
    def cell(x, y):
        return int((y - origin[1]) / cfg.cell), int((x - origin[0]) / cfg.cell)
    assert not gr.is_ground[cell(-3, -3)]
    assert not gr.is_ground[cell(8, -12)]
    assert gr.is_ground[cell(0, 0)]
    assert gr.is_ground[cell(0, 15)]


def test_structures_contain_pole_and_boxes_not_ground(result):
    out, stats = result
    v, f = read_obj(os.path.join(out, "meshes", "structures.obj"))
    near_pole = (np.hypot(v[:, 0] + 12, v[:, 1] - 5) < 0.25)
    assert near_pole.sum() >= 8
    zt = v[near_pole, 2]
    assert zt.max() > SLOPE * -12 + 1.7
    # box1 and box2 walls present
    in_b2 = (v[:, 0] > 1.8) & (v[:, 0] < 14.2) & (v[:, 1] > -18.2) & (v[:, 1] < -5.8)
    assert in_b2.sum() > 100
    # ground plane must not become structure: vertices far from every object
    def near(vv):
        b1 = (vv[:, 0] > -5.3) & (vv[:, 0] < -0.7) & (vv[:, 1] > -5.3) & (vv[:, 1] < -0.7)
        b2 = (vv[:, 0] > 1.7) & (vv[:, 0] < 14.3) & (vv[:, 1] > -18.3) & (vv[:, 1] < -5.7)
        pl = np.hypot(vv[:, 0] + 12, vv[:, 1] - 5) < 0.4
        return b1 | b2 | pl
    assert (~near(v)).sum() <= 0.01 * len(v)


def test_terrain_obj_follows_slope(result):
    out, stats = result
    v, f = read_obj(os.path.join(out, "meshes", "terrain.obj"))
    sel = (v[:, 1] < 9.0) & (v[:, 1] > 6.0) & (v[:, 0] > 5.0) & (v[:, 0] < 15.0)
    assert sel.sum() > 100
    assert np.abs(v[sel, 2] - SLOPE * v[sel, 0]).max() < 0.02
    assert stats["terrain_tris"] == len(f)


def test_obj_valid_and_budgets(result):
    out, stats = result
    cfg = p.Config()
    budgets = {"terrain.obj": cfg.terrain_tris, "structures.obj": cfg.structures_tris,
               "structures_collision.obj": cfg.collision_tris}
    for name, b in budgets.items():
        v, f = read_obj(os.path.join(out, "meshes", name))
        assert len(f) > 0 and len(f) <= b
        assert f.min() >= 1 and f.max() <= len(v)


def test_collision_has_low_structures_only(result):
    out, stats = result
    v, f = read_obj(os.path.join(out, "meshes", "structures_collision.obj"))
    # box1 walls up to 2 m above ground, not the roof (3 m)
    sel = (v[:, 0] > -5.3) & (v[:, 0] < -0.7) & (v[:, 1] > -5.3) & (v[:, 1] < -0.7)
    assert sel.sum() > 0
    top = v[sel, 2] - SLOPE * v[sel, 0]
    assert top.max() < 2.0 + 0.45


def test_winding_outward_for_voxel_cube():
    keys = np.array([0], dtype=np.int64)
    verts, faces = p.voxel_surface(np.array([[5, 5, 5]]), 1, 1.0, np.array([0.0, 0.0, 0.0]))
    # signed volume positive => outward normals
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0
    assert vol == pytest.approx(1.0, abs=1e-6)
    assert len(faces) == 12


def test_decimate_budget():
    n = 100
    g = np.arange(n, dtype=np.float32)
    X, Y = np.meshgrid(g, g)
    verts = np.c_[X.ravel(), Y.ravel(), np.sin(X.ravel() / 5.0)]
    idx = np.arange(n * n).reshape(n, n)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    faces = np.r_[np.c_[a, b, c], np.c_[a, c, d]]
    v2, f2 = p.decimate_mesh(verts, faces, 2000)
    assert len(f2) <= 2000
    assert f2.max() < len(v2)


def test_model_files(result):
    out, stats = result
    sdf = open(os.path.join(out, "model.sdf")).read()
    assert "<static>true</static>" in sdf and 'name="tsukuba"' in sdf
    assert os.path.exists(os.path.join(out, "model.config"))


# ================================================================ 追加: 間引き禁止・greedy meshing
def test_greedy_merges_flat_slab():
    g = np.arange(10)
    I, J = np.meshgrid(g, g)
    ijk = np.c_[I.ravel(), J.ravel(), np.zeros(100, int)]
    v, f = p.voxel_surface(ijk, 0, 0.1, np.zeros(3))
    assert len(f) == 12 and len(v) == 8
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0
    assert vol == pytest.approx(10 * 10 * 0.1 ** 3, rel=1e-6)


def test_voxel_surface_vertices_are_voxel_corners_and_closed():
    rng = np.random.default_rng(3)
    ijk = np.unique(rng.integers(0, 12, size=(400, 3)), axis=0)
    vox = np.array([0.2, 0.2, 0.1])
    v, f = p.voxel_surface(ijk, 0, vox, np.array([1.0, 2.0, 3.0]))
    lat = np.rint((v - np.array([1.0, 2.0, 3.0])) / vox).astype(int)
    occ = set(map(tuple, ijk))
    for l in lat[::7]:
        ok = any((l[0] - dx, l[1] - dy, l[2] - dz) in occ for dx in (0, 1) for dy in (0, 1) for dz in (0, 1))
        assert ok
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0
    assert vol == pytest.approx(len(ijk) * 0.2 * 0.2 * 0.1, rel=1e-6)


def test_columns_to_voxels():
    ij = np.array([[0, 0], [3, 4]])
    ijk = p.columns_to_voxels(ij, np.array([2, -1]), np.array([5, 1]))
    assert len(ijk) == 3 + 2
    assert set(map(tuple, ijk)) == {(0, 0, 2), (0, 0, 3), (0, 0, 4), (3, 4, -1), (3, 4, 0)}


def make_gap_scene():
    """flat ground z=0, two 1m-high boxes x[-2,0] and x[1,3] (y[0,4]) with a 1m gap."""
    rng = np.random.default_rng(5)
    g = np.arange(-8, 8, 0.05)
    X, Y = np.meshgrid(g, g)
    X, Y = X.ravel(), Y.ravel()
    keep = np.ones(X.shape, bool)
    pts = []
    for x0, x1 in ((-2, 0), (1, 3)):
        keep &= ~((X >= x0) & (X <= x1) & (Y >= 0) & (Y <= 4))
        gx, gy = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(0, 4, 0.05))
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, 1.0)])
        for t in np.arange(0, 1.0, 0.05):
            for xs in np.arange(x0, x1, 0.05):
                pts.append(np.array([[xs, 0, t], [xs, 4, t]]))
            for ys in np.arange(0, 4, 0.05):
                pts.append(np.array([[x0, ys, t], [x1, ys, t]]))
    gnd = np.c_[X[keep], Y[keep], rng.normal(0, 0.003, keep.sum())]
    return np.vstack([gnd] + pts).astype(np.float32)


@pytest.fixture(scope="module")
def gap(tmp_path_factory):
    sc = make_gap_scene()
    out = tmp_path_factory.mktemp("gap")
    p.run_pipeline(lambda: iter([sc]), str(out), p.Config(), log=lambda *a: None)
    return sc, out


def tri_samples(v, f, n=6):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    pts = []
    for i in range(n + 1):
        for j in range(n + 1 - i):
            u, w = i / n, j / n
            pts.append(a * (1 - u - w)[None, None][0][:, None] + b * u + c * w) if False else \
                pts.append(a * (1 - u - w) + b * u + c * w)
    return np.vstack(pts)


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_no_triangle_crosses_gap(gap, name):
    sc, out = gap
    v, f = read_obj(os.path.join(out, "meshes", name))
    f = f - 1
    s = tri_samples(v, f)
    # empty gap volume between the boxes (margins keep clear of voxel-rounded walls)
    bad = (s[:, 0] > 0.25) & (s[:, 0] < 0.75) & (s[:, 1] > 0.3) & (s[:, 1] < 3.7) \
        & (s[:, 2] > 0.2) & (s[:, 2] < 0.9)
    assert bad.sum() == 0
    assert len(f) > 0


def test_vertices_close_to_input_points(gap):
    from scipy.spatial import cKDTree
    sc, out = gap
    tree = cKDTree(sc.astype(np.float64))
    v, _ = read_obj(os.path.join(out, "meshes", "structures.obj"))
    d, _ = tree.query(v)
    assert d.max() <= 0.1 * np.sqrt(3) + 1e-3
    vc, _ = read_obj(os.path.join(out, "meshes", "structures_collision.obj"))
    top = vc[vc[:, 2] > 0.2]
    d, _ = tree.query(top)
    assert d.max() <= np.sqrt(0.2 ** 2 + 0.2 ** 2 + 0.1 ** 2) + 1e-3


# ================================================================ 追加: OBJ 法線 (Gazebo dartsim/ODE 用)
def read_obj_full(path):
    v, vn, f = [], [], []
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                v.append([float(a) for a in t[1:4]])
            elif t[0] == "vn":
                vn.append([float(a) for a in t[1:4]])
            elif t[0] == "f":
                assert all("//" in a for a in t[1:4]), line
                f.append([[int(b) for b in a.split("//")] for a in t[1:4]])
    return np.array(v), np.array(vn), np.array(f)


@pytest.mark.parametrize("name", ["terrain.obj", "structures.obj", "structures_collision.obj"])
def test_obj_has_consistent_normals(result, name):
    out, stats = result
    v, vn, f = read_obj_full(os.path.join(out, "meshes", name))
    assert len(vn) > 0 and len(f) > 0
    vi, ni = f[:, :, 0], f[:, :, 1]
    assert vi.min() >= 1 and vi.max() <= len(v)
    assert ni.min() >= 1 and ni.max() <= len(vn)
    a, b, c = v[vi[:, 0] - 1], v[vi[:, 1] - 1], v[vi[:, 2] - 1]
    fn = np.cross(b - a, c - a)
    for k in range(3):
        d = np.einsum("ij,ij->i", fn, vn[ni[:, k] - 1])
        assert (d > 0).all()
    assert np.allclose(np.linalg.norm(vn, axis=1), 1.0, atol=1e-3)


def test_structures_use_six_face_normals(result):
    out, stats = result
    _, vn, f = read_obj_full(os.path.join(out, "meshes", "structures.obj"))
    assert len(vn) == 6
    assert (f[:, 0, 1] == f[:, 1, 1]).all() and (f[:, 1, 1] == f[:, 2, 1]).all()


def test_write_obj_vertex_normals_unit(tmp_path):
    verts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], float)
    faces = np.array([[0, 1, 3], [0, 3, 2]])
    p.write_obj(str(tmp_path / "x.obj"), verts, faces, normals="vertex")
    v, vn, f = read_obj_full(str(tmp_path / "x.obj"))
    assert np.allclose(vn, [[0, 0, 1]] * 4)


# ================================================================ 追加: 経路近傍の低いブロック除去
def test_path_cell_mask_boundary_uses_cell_center():
    poly = np.array([[0.0, 0.0], [10.0, 0.0]])
    ny, nx = 12, 60
    m = p.path_cell_mask(poly, (-1.0, -1.0), (ny, nx), 0.2, 1.0)
    # セル中心 y = -1 + (iy+0.5)*0.2 ; 中心 y=0.9 は内側, 1.1 は外側
    iy_in = int(round((0.9 + 1.0) / 0.2 - 0.5))
    iy_out = iy_in + 1
    ix = int((5.0 + 1.0) / 0.2)
    assert m[iy_in, ix] and not m[iy_out, ix]
    # 端点の外側は円で判定: 中心 (10.5, 0.1) は内側、(11.1, 0.1) は外側
    ix_a = int(round((10.5 + 1.0) / 0.2 - 0.5))
    ix_b = int(round((11.1 + 1.0) / 0.2 - 0.5))
    iy0 = int(round((0.1 + 1.0) / 0.2 - 0.5))
    assert m[iy0, ix_a] and (ix_b >= nx or not m[iy0, ix_b])


def make_clear_scene():
    rng = np.random.default_rng(7)
    g = np.arange(-15, 15, 0.05)
    X, Y = np.meshgrid(g, g)
    X, Y = X.ravel(), Y.ravel()
    boxes = {"A": (0, 1, 0.3, 0.9, 0.2), "B": (3, 3.4, 0.3, 0.7, 1.0), "C": (0, 1, 5.0, 5.6, 0.2)}
    keep = np.ones(X.shape, bool)
    pts = []
    for x0, x1, y0, y1, hh in boxes.values():
        keep &= ~((X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1))
        gx, gy = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05))
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, hh)])
        for t in np.arange(0, hh, 0.05):
            for xs in np.arange(x0, x1, 0.05):
                pts.append(np.array([[xs, y0, t], [xs, y1, t]]))
            for ys in np.arange(y0, y1, 0.05):
                pts.append(np.array([[x0, ys, t], [x1, ys, t]]))
    gnd = np.c_[X[keep], Y[keep], rng.normal(0, 0.003, keep.sum())]
    return np.vstack([gnd] + pts).astype(np.float32)


@pytest.fixture(scope="module")
def cleared(tmp_path_factory):
    sc = make_clear_scene()
    out = tmp_path_factory.mktemp("clr")
    path = np.array([[-10.0, 0.0], [10.0, 0.0]])
    p.run_pipeline(lambda: iter([sc]), str(out), p.Config(), log=lambda *a: None, clear_path=path)
    return out


def _count(out, name, x0, x1, y0, y1, zmin, zmax=9):
    v, _ = read_obj(os.path.join(out, "meshes", name))
    s = (v[:, 0] > x0 - 0.25) & (v[:, 0] < x1 + 0.25) & (v[:, 1] > y0 - 0.25) & (v[:, 1] < y1 + 0.25) \
        & (v[:, 2] > zmin) & (v[:, 2] < zmax)
    return int(s.sum())


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_clear_removes_low_box_near_path_keeps_others(cleared, name):
    out = cleared
    # A: 経路近傍の h=0.2 の低い箱 -> 消える
    assert _count(out, name, 0, 1, 0.3, 0.9, 0.08) == 0
    # B: 経路近傍の h=1.0 の柱 -> 残る(下端ボクセルも含め地面付近の頂点がある)
    assert _count(out, name, 3, 3.4, 0.3, 0.7, 0.4) > 0
    assert _count(out, name, 3, 3.4, 0.3, 0.7, -0.2, 0.25) > 0
    # C: 経路から遠い h=0.2 の箱 -> 残る
    assert _count(out, name, 0, 1, 5.0, 5.6, 0.08) > 0


# ================================================================ 追加: 区間クリアゾーン
def test_zone_parse_and_yaml(tmp_path):
    assert p.parse_zone("70.571,71.831,104.7,99.559,3.0,0.5") == (70.571, 71.831, 104.7, 99.559, 3.0, 0.5)
    y = tmp_path / "z.yaml"
    y.write_text("zones:\n- {x1: 1.0, y1: 2.0, x2: 3.0, y2: 4.0, radius: 3.0, max_h: 0.5}\n"
                 "- {x1: 0, y1: 0, x2: 5, y2: 0, radius: 2, max_h: 0.4}\n")
    z = p.load_zones(str(y))
    assert z == [(1.0, 2.0, 3.0, 4.0, 3.0, 0.5), (0.0, 0.0, 5.0, 0.0, 2.0, 0.4)]


def make_zone_scene():
    rng = np.random.default_rng(11)
    g = np.arange(-15, 15, 0.05)
    X, Y = np.meshgrid(g, g)
    X, Y = X.ravel(), Y.ravel()
    boxes = [(0, 1, 1.5, 2.1, 0.45), (3, 3.4, 1.5, 1.9, 0.8), (0, 1, 5.5, 6.1, 0.45)]
    keep = np.ones(X.shape, bool)
    pts = []
    for x0, x1, y0, y1, hh in boxes:
        keep &= ~((X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1))
        gx, gy = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05))
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, hh)])
        for t in np.arange(0, hh, 0.05):
            for xs in np.arange(x0, x1, 0.05):
                pts.append(np.array([[xs, y0, t], [xs, y1, t]]))
            for ys in np.arange(y0, y1, 0.05):
                pts.append(np.array([[x0, ys, t], [x1, ys, t]]))
    gnd = np.c_[X[keep], Y[keep], rng.normal(0, 0.003, keep.sum())]
    return np.vstack([gnd] + pts).astype(np.float32)


@pytest.fixture(scope="module")
def zoned(tmp_path_factory):
    sc = make_zone_scene()
    out = tmp_path_factory.mktemp("zone")
    p.run_pipeline(lambda: iter([sc]), str(out), p.Config(), log=lambda *a: None,
                   clear_zones=[(-10.0, 0.0, 10.0, 0.0, 3.0, 0.5)])
    return out


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_zone_clears_low_keeps_tall_and_outside(zoned, name):
    out = zoned
    assert _count(out, name, 0, 1, 1.5, 2.1, 0.08) == 0          # ゾーン内 h=0.45 -> 消える
    assert _count(out, name, 3, 3.4, 1.5, 1.9, 0.5) > 0          # ゾーン内 h=0.8 -> 残る
    assert _count(out, name, 3, 3.4, 1.5, 1.9, -0.2, 0.25) > 0   # 下端も残る
    assert _count(out, name, 0, 1, 5.5, 6.1, 0.08) > 0           # ゾーン外 h=0.45 -> 残る


# ================================================================ 追加: 判定を最終ボクセル/柱で行う
def make_sparse_scene():
    """平坦地面 + 疎な低い点列。0.15m ボクセルには 3 点入るが、0.2m セル/5cm ビンでは点数不足になる。"""
    rng = np.random.default_rng(13)
    g = np.arange(-15, 15, 0.05)
    X, Y = np.meshgrid(g, g)
    X, Y = X.ravel(), Y.ravel()
    gnd = np.c_[X, Y, rng.normal(0, 0.002, X.shape)]
    blobs = [[[x0 + dx, y, 0.18] for dx in (0.01, 0.07, 0.13)]
             for y in (0.5, 5.0) for x0 in (0.15, 1.35, 2.55)]
    return np.vstack([gnd] + [np.array(b) for b in blobs]).astype(np.float32)


@pytest.fixture(scope="module")
def sparse_runs(tmp_path_factory):
    sc = make_sparse_scene()
    cfg = p.Config(voxel=0.15, min_count=3)
    outs = {}
    for tag, path in (("clear", np.array([[-10.0, 0.0], [10.0, 0.0]])), ("none", None)):
        out = tmp_path_factory.mktemp("sp_" + tag)
        p.run_pipeline(lambda: iter([sc]), str(out), cfg, log=lambda *a: None, clear_path=path)
        outs[tag] = out
    return outs


def test_sparse_low_voxels_near_path_are_cleared_by_final_voxels(sparse_runs):
    # クリア無しでは経路近傍(y=0.5)にも遠方(y=5)にも visual ボクセルが存在する
    n_near = _count(sparse_runs["none"], "structures.obj", 0.0, 3.0, 0.4, 0.65, 0.05)
    assert n_near > 0
    # 経路クリア有りでは、経路上の柱は消え、遠方は残る
    assert _count(sparse_runs["clear"], "structures.obj", 0.0, 3.0, 0.4, 0.65, 0.05) == 0
    assert _count(sparse_runs["clear"], "structures.obj", 0.0, 3.0, 4.9, 5.1, 0.05) > 0


# ================================================================ 追加: 鉛直 run 判定
def _box_pts(x0, x1, y0, y1, z0, z1, top=True, bottom=False, step=0.05):
    pts = []
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step))
    if top:
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z1)])
    if bottom:
        pts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z0)])
    for t in np.arange(z0, z1 + 1e-9, step):
        for xs in np.arange(x0, x1 + 1e-9, step):
            pts.append(np.array([[xs, y0, t], [xs, y1, t]]))
        for ys in np.arange(y0, y1 + 1e-9, step):
            pts.append(np.array([[x0, ys, t], [x1, ys, t]]))
    return np.vstack(pts)


def make_run_scene():
    rng = np.random.default_rng(21)
    g = np.arange(-9, 9, 0.05)
    X, Y = np.meshgrid(g, g)
    gnd = np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]
    parts = [gnd]
    # 屋根 (h 3.0-3.3) x[-0.5,4.5] y[-2,2] : 上面と下面
    parts.append(_box_pts(-0.5, 4.5, -2, 2, 3.0, 3.3, top=True, bottom=True)[:0])
    for z in (3.0, 3.3):
        gx, gy = np.meshgrid(np.arange(-0.5, 4.5, 0.05), np.arange(-2, 2, 0.05))
        parts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z)])
    parts.append(_box_pts(1.0, 2.0, 0.2, 0.8, 0.0, 0.3))       # 屋根の下の低い箱 (h 0.3)
    parts.append(_box_pts(3.0, 3.4, 0.2, 0.6, 0.0, 3.0))       # 地面から屋根まで連続する柱
    # 樹冠(h 2.5-2.8) x[-5.5,-2.5] y[-0.5,1.5] + 幹 + 幹と接しない低木
    for z in (2.5, 2.8):
        gx, gy = np.meshgrid(np.arange(-5.5, -2.5, 0.05), np.arange(-0.5, 1.5, 0.05))
        parts.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z)])
    parts.append(_box_pts(-4.0, -3.7, 0.3, 0.6, 0.0, 2.5))     # 幹
    parts.append(_box_pts(-5.0, -4.5, 0.8, 1.2, 0.0, 0.3))     # 低木 (h 0.3)
    return np.vstack(parts).astype(np.float32)


@pytest.fixture(scope="module")
def runs_out(tmp_path_factory):
    sc = make_run_scene()
    out = tmp_path_factory.mktemp("runs")
    p.run_pipeline(lambda: iter([sc]), str(out), p.Config(), log=lambda *a: None,
                   clear_zones=[(-10.0, 0.0, 10.0, 0.0, 3.0, 0.5)])
    return out


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_runs_low_blocks_under_roof_and_canopy_cleared(runs_out, name):
    out = runs_out
    assert _count(out, name, 1.0, 2.0, 0.2, 0.8, 0.06, 1.0) == 0            # 屋根の下の低い箱は消える
    assert _count(out, name, -5.0, -4.5, 0.8, 1.2, 0.06, 1.0) == 0          # 樹冠下の低木は消える
    assert _count(out, name, 3.0, 3.4, 0.2, 0.6, 0.4, 1.4) > 0              # 連続柱は残る(collision は上端 h≈1.2)
    assert _count(out, name, 3.0, 3.4, 0.2, 0.6, -0.2, 0.25) > 0
    assert _count(out, name, -4.0, -3.7, 0.3, 0.6, 0.4, 1.4) > 0            # 幹は残る


def test_runs_roof_and_canopy_kept_in_visual(runs_out):
    assert _count(runs_out, "structures.obj", 0.5, 3.5, -1.5, 1.5, 2.9, 3.5) > 0
    assert _count(runs_out, "structures.obj", -5.0, -3.0, 0.0, 1.0, 2.4, 2.9) > 0


# ================================================================ 追加: 実寸直方体への置き換え
def _rot(u, v, yaw_deg, cx, cy):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return cx + c * u - s * v, cy + s * u + c * v


def make_override_scene():
    rng = np.random.default_rng(31)
    g = np.arange(-10, 10, 0.05)
    X, Y = np.meshgrid(g, g)
    parts = [np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]]
    # 太った柱 0.9x0.7 h=3 を yaw 30 で (0,5) に置く。実寸は 0.6x0.4 に置換する想定
    local = _box_pts(-0.45, 0.45, -0.35, 0.35, 0.0, 3.0)
    x, y = _rot(local[:, 0], local[:, 1], 30.0, 0.0, 5.0)
    parts.append(np.c_[x, y, local[:, 2]])
    # 範囲外の柱 (影響を受けない)
    parts.append(_box_pts(6.0, 6.6, -6.0, -5.4, 0.0, 3.0))
    return np.vstack(parts).astype(np.float32)


@pytest.fixture(scope="module")
def override_out(tmp_path_factory):
    sc = make_override_scene()
    out = tmp_path_factory.mktemp("ovr")
    ov = [dict(center_x=0.0, center_y=5.0, yaw_deg=30.0, size_x=0.6, size_y=0.4,
               z_bottom_h=0.0, z_top_h=3.0, margin=0.25)]
    p.run_pipeline(lambda: iter([sc]), str(out), p.Config(), log=lambda *a: None, box_overrides=ov)
    return out


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_override_replaces_fat_pillar_with_exact_box(override_out, name):
    v, _ = read_obj(os.path.join(override_out, "meshes", name))
    d = np.hypot(v[:, 0] - 0.0, v[:, 1] - 5.0) < 1.5
    assert d.sum() >= 8
    c, s = np.cos(np.radians(30)), np.sin(np.radians(30))
    u = c * (v[d, 0]) + s * (v[d, 1] - 5.0)
    w = -s * (v[d, 0]) + c * (v[d, 1] - 5.0)
    assert np.abs(u).max() <= 0.3 + 1e-3 and np.abs(w).max() <= 0.2 + 1e-3
    assert np.abs(u).max() >= 0.3 - 1e-3 and np.abs(w).max() >= 0.2 - 1e-3
    ztop = 3.0 if name == "structures.obj" else 1.2
    assert v[d, 2].max() == pytest.approx(ztop, abs=0.02)
    assert v[d, 2].min() == pytest.approx(-0.05, abs=0.02)


@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_override_leaves_other_pillar(override_out, name):
    assert _count(override_out, name, 6.0, 6.6, -6.0, -5.4, 0.4, 1.4) > 0


def test_obj_normals_exact_for_rotated_box(override_out):
    v, vn, f = read_obj_full(os.path.join(override_out, "meshes", "structures.obj"))
    vi, ni = f[:, :, 0], f[:, :, 1]
    a, b, c = v[vi[:, 0] - 1], v[vi[:, 1] - 1], v[vi[:, 2] - 1]
    fn = np.cross(b - a, c - a)
    fn /= np.linalg.norm(fn, axis=1)[:, None]
    assert (np.einsum("ij,ij->i", fn, vn[ni[:, 0] - 1]) > 0.999).all()


# ================================================================ 追加: collision を経路周辺に絞る
ROUTE = np.array([[-10.0, 0.0], [10.0, 0.0]])


def make_route_scene():
    rng = np.random.default_rng(41)
    g = np.arange(-20, 20, 0.05)
    X, Y = np.meshgrid(g, g)
    parts = [np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]]
    parts.append(_box_pts(2.0, 2.6, 2.7, 3.3, 0.0, 1.0))     # A: 経路から 3m (半径 5 以内)
    parts.append(_box_pts(2.0, 2.6, 8.7, 9.3, 0.0, 1.0))     # B: 経路から 9m (半径 5 の外)
    return np.vstack(parts).astype(np.float32)


def _route_dist(P, poly):
    d = np.full(len(P), np.inf)
    for a, b in zip(poly[:-1], poly[1:]):
        v = b - a
        t = np.clip(((P - a) @ v) / (v @ v), 0, 1)
        d = np.minimum(d, np.hypot(*(P - (a + t[:, None] * v)).T))
    return d


def _read_mesh(path):
    v, f = read_obj(path)
    return v, f - 1


@pytest.fixture(scope="module")
def route_runs(tmp_path_factory):
    sc = make_route_scene()
    outs = {}
    for tag, cfg, route in (
            ("near", p.Config(collision_route_radius=5.0, terrain_collision_route_radius=8.0), ROUTE),
            ("none", p.Config(collision_route_radius=None, terrain_collision_route_radius=None), ROUTE),
            ("noroute", p.Config(), None)):
        out = tmp_path_factory.mktemp("rt_" + tag)
        p.run_pipeline(lambda: iter([sc]), str(out), cfg, log=lambda *a: None, route_path=route)
        outs[tag] = out
    return outs


def test_collision_limited_to_route_radius_visual_unchanged(route_runs):
    near, full = route_runs["near"], route_runs["none"]
    assert _count(near, "structures_collision.obj", 2.0, 2.6, 2.7, 3.3, 0.4, 1.4) > 0
    assert _count(near, "structures_collision.obj", 2.0, 2.6, 8.7, 9.3, 0.4, 1.4) == 0
    assert _count(full, "structures_collision.obj", 2.0, 2.6, 8.7, 9.3, 0.4, 1.4) > 0   # none -> 全域
    # visual は不変
    assert open(os.path.join(near, "meshes", "structures.obj")).read() == \
        open(os.path.join(full, "meshes", "structures.obj")).read()
    assert _count(near, "structures.obj", 2.0, 2.6, 8.7, 9.3, 0.4, 1.4) > 0


def test_terrain_collision_triangles_within_radius_and_complete(route_runs):
    out = route_runs["near"]
    v, f = _read_mesh(os.path.join(out, "meshes", "terrain.obj"))
    vc, fc = _read_mesh(os.path.join(out, "meshes", "terrain_collision.obj"))
    cen = v[f].mean(axis=1)[:, :2]
    cenc = vc[fc].mean(axis=1)[:, :2]
    assert (_route_dist(cenc, ROUTE) <= 8.0 + 1e-6).all()
    expect = cen[_route_dist(cen, ROUTE) <= 8.0]
    assert len(fc) == len(expect) > 0
    a = np.round(np.sort(expect[:, 0] * 1000 + expect[:, 1]), 3)
    b = np.round(np.sort(cenc[:, 0] * 1000 + cenc[:, 1]), 3)
    assert np.allclose(a, b, atol=2e-3)
    assert len(fc) < len(f)
    assert len(np.unique(fc)) == len(vc)           # 未使用頂点は詰める
    _, vn, ff = read_obj_full(os.path.join(out, "meshes", "terrain_collision.obj"))
    assert len(vn) == len(vc) and len(ff) == len(fc)


def test_route_none_means_everywhere(route_runs):
    for tag in ("none", "noroute"):
        out = route_runs[tag]
        _, f = _read_mesh(os.path.join(out, "meshes", "terrain.obj"))
        _, fc = _read_mesh(os.path.join(out, "meshes", "terrain_collision.obj"))
        assert len(fc) == len(f)
    assert _count(route_runs["noroute"], "structures_collision.obj", 2.0, 2.6, 8.7, 9.3, 0.4, 1.4) > 0


def test_model_sdf_references_terrain_collision(route_runs):
    sdf = open(os.path.join(route_runs["near"], "model.sdf")).read()
    vis = sdf[sdf.index('<visual name="terrain_visual">'):sdf.index('</visual>')]
    col = sdf[sdf.index('<collision name="terrain_collision">'):sdf.index('</collision>')]
    assert "meshes/terrain.obj" in vis and "terrain_collision.obj" not in vis
    assert "meshes/terrain_collision.obj" in col


# ================================================================ 追加: 柱の置換 yaml (点群座標)


def test_box_override_center_is_pillar_cloud_coords():
    here = os.path.join(os.path.dirname(__file__), "..", "box_overrides.yaml")
    b = p.load_boxes(here)[0]
    assert (b["center_x"], b["center_y"]) == pytest.approx((446.482, 185.027), abs=1e-3)
    assert b["size_x"] == 1.10 and b["size_y"] == 0.45 and b["z_top_h"] == 3.86


# ================================================================ 追加: 経路沿い地形の平滑化
def _plane(x, y):
    return 0.05 * x + 0.02 * y


def make_bumpy_grid():
    cell = 0.2
    n = 200
    origin = (-20.0, -20.0)
    cx = origin[0] + (np.arange(n) + 0.5) * cell
    cy = origin[1] + (np.arange(n) + 0.5) * cell
    X, Y = np.meshgrid(cx, cy)
    z = _plane(X, Y) + 0.075 * np.sin(2 * np.pi * X / 2.0)       # 凸凹 (振幅 7.5cm = 15cm p-p)
    z = z + np.where((np.abs(X - 3.0) < 0.2) & (np.abs(Y - 0.5) < 0.2), 0.3, 0.0)   # 30cm スパイク
    z = z + np.where((np.abs(X + 6.0) < 0.2) & (np.abs(Y + 1.0) < 0.2), 0.3, 0.0)
    return z.astype(np.float32), origin, cell, X, Y


ROUTE2 = np.array([[-15.0, 0.0], [15.0, 0.0]])


def test_route_smoothing_flat_slope_kept_far_unchanged():
    z, origin, cell, X, Y = make_bumpy_grid()
    out = p.smooth_route_ground(z, origin, cell, ROUTE2, radius=3.0, blend=2.0, median=1.5, sigma=1.0)
    d = np.abs(Y)                                  # 経路(y=0)までの距離 (x∈[-15,15])
    inner = (d <= 2.5) & (np.abs(X) <= 10)
    assert np.abs(out - _plane(X, Y))[inner].max() <= 0.01
    # 傾斜(roll/pitch)が保たれる: 平面フィットの傾斜角 ±0.2°
    A = np.c_[X[inner], Y[inner], np.ones(inner.sum())]
    cf, *_ = np.linalg.lstsq(A, out[inner].astype(float), rcond=None)
    ang = np.degrees(np.arctan(np.hypot(cf[0], cf[1])))
    ang0 = np.degrees(np.arctan(np.hypot(0.05, 0.02)))
    assert abs(ang - ang0) <= 0.2
    # 経路から 6m 以上離れた場所は不変
    far = (d >= 6.0)
    assert np.array_equal(out[far], z[far])


def test_route_smoothing_disabled_when_radius_none():
    z, origin, cell, X, Y = make_bumpy_grid()
    out = p.smooth_route_ground(z, origin, cell, ROUTE2, radius=None, blend=2.0, median=1.5, sigma=1.0)
    assert np.array_equal(out, z)


def test_pipeline_route_smoothing_terrain(tmp_path):
    rng = np.random.default_rng(51)
    g = np.arange(-20, 20, 0.05)
    X, Y = np.meshgrid(g, g)
    Z = _plane(X, Y) + 0.075 * np.sin(2 * np.pi * X / 2.0) + rng.normal(0, 0.002, X.shape)
    pts = np.c_[X.ravel(), Y.ravel(), Z.ravel()].astype(np.float32)
    outs = {}
    for tag, cfg in (("smooth", p.Config(route_smooth_radius=3.0)), ("raw", p.Config(route_smooth_radius=None))):
        o = tmp_path / tag
        p.run_pipeline(lambda: iter([pts]), str(o), cfg, log=lambda *a: None, route_path=ROUTE2)
        outs[tag] = read_obj(str(o / "meshes" / "terrain.obj"))[0]
    v = outs["smooth"]
    inner = (np.abs(v[:, 1]) <= 2.5) & (np.abs(v[:, 0]) <= 10)
    assert inner.sum() > 100
    A = np.c_[v[inner, 0], v[inner, 1], np.ones(inner.sum())]
    cf, *_ = np.linalg.lstsq(A, v[inner, 2], rcond=None)
    assert np.abs(A @ cf - v[inner, 2]).max() <= 0.01          # 平面からの残差 1cm 以下 (絶対高さのバイアスは地面推定器の性質)
    assert abs(np.degrees(np.arctan(np.hypot(cf[0], cf[1]))) - np.degrees(np.arctan(np.hypot(0.05, 0.02)))) <= 0.2
    r = outs["raw"]
    far = np.abs(r[:, 1]) >= 6.5
    key = lambda a: {(round(x, 3), round(y, 3)): z for x, y, z in a}
    ks, kr = key(v), key(r)
    com = [k for k in kr if abs(k[1]) >= 6.5 and k in ks]
    assert len(com) > 100
    assert max(abs(ks[k] - kr[k]) for k in com) < 1e-3


# ================================================================ 追加: 不変条件 collision ⊂ visual
NBOX = 25


def make_threshold_scene():
    rng = np.random.default_rng(61)
    g = np.arange(-12, 12, 0.05)
    X, Y = np.meshgrid(g, g)
    parts = [np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]]
    for i in range(NBOX):                    # h = 0.20 .. 0.32 (しきい値 0.25 付近)、格子とずれた位置、経路(y=0)から 0.3-0.7m
        x0 = -9.0 + 0.77 * i + 0.013 * i
        parts.append(_box_pts(x0, x0 + 0.4, 0.3 + 0.01 * (i % 3), 0.7 + 0.01 * (i % 3), 0.0, 0.20 + 0.005 * i))
    return np.vstack(parts).astype(np.float32)


@pytest.fixture(scope="module", params=[(0.1, 3), (0.15, 3)])
def thr_out(request, tmp_path_factory):
    vox, mc = request.param
    out = tmp_path_factory.mktemp("thr")
    cfg = p.Config(voxel=vox, min_count=mc)
    p.run_pipeline(lambda: iter([make_threshold_scene()]), str(out), cfg, log=lambda *a: None,
                   clear_path=np.array([[-10.0, 0.0], [10.0, 0.0]]),
                   route_path=np.array([[-10.0, 0.0], [10.0, 0.0]]))
    return out


def test_threshold_boxes_visual_and_collision_agree(thr_out):
    res = []
    for i in range(NBOX):
        x0 = -9.0 + 0.77 * i + 0.013 * i
        v = _count(thr_out, "structures.obj", x0 + 0.1, x0 + 0.3, 0.4, 0.6, 0.1) > 0
        c = _count(thr_out, "structures_collision.obj", x0 + 0.1, x0 + 0.3, 0.4, 0.6, 0.1) > 0
        res.append((v, c))
    assert all(v == c for v, c in res), res


def test_collision_cells_have_visual_nearby(thr_out):
    from scipy.spatial import cKDTree
    vv, _ = read_obj(os.path.join(thr_out, "meshes", "structures.obj"))
    cv, _ = read_obj(os.path.join(thr_out, "meshes", "structures_collision.obj"))
    cv = cv[cv[:, 2] > 0.12]
    if len(cv) == 0:
        return
    tree = cKDTree(vv[:, :2])
    for q in cv:
        idx = tree.query_ball_point(q[:2], 0.4)
        assert idx and np.abs(vv[idx, 2] - q[2]).min() <= 0.35


def test_noise_filtered_in_visual_not_in_collision(tmp_path):
    rng = np.random.default_rng(62)
    g = np.arange(-6, 6, 0.05)
    X, Y = np.meshgrid(g, g)
    base = np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]
    pts = []
    for fx in (0.02, 0.12):                 # 0.2m セル 1 つに 4 つの 0.1m ボクセル x 3 点 = 12 点 (min_count=5 未満)
        for fy in (0.02, 0.12):
            for _ in range(3):
                pts.append([2.0 + fx, 2.0 + fy, 0.30])
    sc = np.vstack([base, np.array(pts)]).astype(np.float32)
    p.run_pipeline(lambda: iter([sc]), str(tmp_path), p.Config(min_count=5), log=lambda *a: None)
    assert _count(tmp_path, "structures.obj", 1.9, 2.3, 1.9, 2.3, 0.1) == 0
    assert _count(tmp_path, "structures_collision.obj", 1.9, 2.3, 1.9, 2.3, 0.1) == 0


# ================================================================ 追加: 経路沿い細い柱状物の自動実寸置換
POLE_A = (2.03, 2.15, 1.02, 1.32)     # 0.12 x 0.30
POLE_B = (3.25, 3.55, 1.05, 1.25)     # 0.30 x 0.20  (A との隙間 1.10)
POLE_FAR = (2.03, 2.15, 8.02, 8.32)   # 経路から 8m


def make_pole_scene(extra=()):
    rng = np.random.default_rng(71)
    g = np.arange(-12, 12, 0.05)
    X, Y = np.meshgrid(g, g)
    parts = [np.c_[X.ravel(), Y.ravel(), rng.normal(0, 0.002, X.size)]]
    for (x0, x1, y0, y1) in (POLE_A, POLE_B, POLE_FAR) + tuple(extra):
        parts.append(_box_pts(x0, x1, y0, y1, 0.0, 1.5))
    return np.vstack(parts).astype(np.float32)


def run_poles(tmp, vox, extra=(), radius=5.0):
    cfg = p.Config(voxel=vox, pole_route_radius=radius)
    st = p.run_pipeline(lambda: iter([make_pole_scene(extra)]), str(tmp), cfg, log=lambda *a: None,
                        route_path=ROUTE2)
    return st


@pytest.mark.parametrize("vox", [0.1, 0.15])
@pytest.mark.parametrize("name", ["structures.obj", "structures_collision.obj"])
def test_poles_replaced_with_real_size_and_gap(tmp_path, vox, name):
    st = run_poles(tmp_path, vox)
    assert st["poles_replaced"] == 2                      # 隣接 2 本が 1 本にまとまらない / 遠方は置換しない
    v, _ = read_obj(os.path.join(tmp_path, "meshes", name))
    def ext(x0, x1, y0, y1):
        s = (v[:, 0] > x0) & (v[:, 0] < x1) & (v[:, 1] > y0) & (v[:, 1] < y1) & (v[:, 2] > 0.3)
        return v[s, 0].min(), v[s, 0].max(), v[s, 1].min(), v[s, 1].max()
    ax0, ax1, ay0, ay1 = ext(1.5, 2.7, 0.7, 1.7)
    bx0, bx1, by0, by1 = ext(2.9, 4.0, 0.7, 1.7)
    assert abs((ax1 - ax0) - 0.12) <= 0.03 and abs((ay1 - ay0) - 0.30) <= 0.03
    assert abs((bx1 - bx0) - 0.30) <= 0.03 and abs((by1 - by0) - 0.20) <= 0.03
    assert abs((bx0 - ax1) - 1.10) <= 0.05


def test_pole_outside_route_radius_not_replaced(tmp_path):
    st = run_poles(tmp_path, 0.1, radius=None)
    assert st["poles_replaced"] == 0
    st2 = run_poles(tmp_path / "r", 0.1, radius=0.5)       # A,B は経路から 1m 以上
    assert st2["poles_replaced"] == 0


def test_wall_not_replaced(tmp_path):
    st = run_poles(tmp_path, 0.1, extra=((-6.0, -4.0, 1.0, 1.3),))   # 長軸 2m の壁
    assert st["poles_replaced"] == 2
