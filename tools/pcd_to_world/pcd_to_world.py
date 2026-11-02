#!/usr/bin/env python3
"""点群地図(PCD) -> Gazebo 用静的モデル(tsukuba) 生成ツール。

出力: <out>/model.config, model.sdf, meshes/{terrain,structures,structures_collision}.obj
座標は map(=Gazebo world) 座標系でそのまま書き出す(モデル姿勢は原点)。

実行例は README 相当として末尾の main() / --help を参照。
"""
import argparse
import os
import resource
import time
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------- 座標変換
@dataclass
class GeoParams:
    gth: float = -2.3500858378103646   # rsf_slam/config/tsukuba_georef.yaml enu_to_map
    gtx: float = 156.89610202453204
    gty: float = 64.64302275894745
    lat0: float = 36.083470500000004
    lon0: float = 140.07630341666666
    oe: float = 21903.53               # 点群ローカル -> 平面直角(EPSG:6677) オフセット (spherical で nav 地図と照合して推定)
    on: float = 9223.57
    zoff: float = -24.775              # map_z = z + zoff
    epsg: str = "EPSG:6677"
    model: str = "enu_origin_spherical"  # enu_origin_spherical(既定): 点群=原点中心の局所ENU / spherical: 平面直角+オフセット+球近似 / ellipsoid: 平面直角+オフセット+ortho
    origin_lat: float = 36.082880      # 点群の水平原点(JGD2024) 出典: raucha/tc-datasets add-map (TC2026 ISE Robotics GNSS map data)
    origin_lon: float = 140.076545
    earth_radius: float = 6378137.0


_TR_CACHE = {}


def _transformers(g):
    key = (g.lat0, g.lon0, g.epsg)
    if key not in _TR_CACHE:
        from pyproj import Transformer
        inv = Transformer.from_crs(g.epsg, "EPSG:4326", always_xy=True)
        enu = Transformer.from_crs(
            "EPSG:4326", f"+proj=ortho +lat_0={g.lat0} +lon_0={g.lon0} +ellps=GRS80",
            always_xy=True)
        _TR_CACHE[key] = (inv, enu)
    return _TR_CACHE[key]


_ORIGIN_CACHE = {}


def _origin_inv(g):
    key = (g.origin_lat, g.origin_lon)
    if key not in _ORIGIN_CACHE:
        from pyproj import Transformer
        _ORIGIN_CACHE[key] = Transformer.from_crs(
            f"+proj=ortho +lat_0={g.origin_lat} +lon_0={g.origin_lon} +ellps=GRS80", "EPSG:4326",
            always_xy=True)
    return _ORIGIN_CACHE[key]


def cloud2latlon(x, y, g=None):
    """点群ローカル座標 -> 緯度経度 (pyproj, EPSG:6677 -> 4326)。"""
    g = g or GeoParams()
    if g.model == "enu_origin_spherical":
        lon, lat = _origin_inv(g).transform(np.asarray(x, float), np.asarray(y, float))
        return np.asarray(lat), np.asarray(lon)
    inv, _ = _transformers(g)
    lon, lat = inv.transform(np.asarray(x, float) + g.oe, np.asarray(y, float) + g.on)
    return np.asarray(lat), np.asarray(lon)


def _enu_to_map(e, n, g):
    c, s = np.cos(g.gth), np.sin(g.gth)
    return c * e - s * n + g.gtx, s * e + c * n + g.gty


def latlon_to_map(lat, lon, g=None):
    """GNSS 緯度経度 -> map。プロジェクトの球近似式 (enu_from_latlon) + georef の enu_to_map。"""
    g = g or GeoParams()
    lat, lon = np.asarray(lat, float), np.asarray(lon, float)
    e = np.radians(lon - g.lon0) * np.cos(np.radians(g.lat0)) * g.earth_radius
    n = np.radians(lat - g.lat0) * g.earth_radius
    return _enu_to_map(e, n, g)


def map_to_latlon(mx, my, g=None):
    """map -> 緯度経度。rsf_device_emulator.latlon() と同じ式(ノイズ無し)。"""
    g = g or GeoParams()
    c, s = np.cos(g.gth), np.sin(g.gth)
    dx, dy = np.asarray(mx, float) - g.gtx, np.asarray(my, float) - g.gty
    e, n = c * dx + s * dy, -s * dx + c * dy
    lat = g.lat0 + np.degrees(n / g.earth_radius)
    lon = g.lon0 + np.degrees(e / (g.earth_radius * np.cos(np.radians(g.lat0))))
    return lat, lon


def cloud2map_exact(x, y, g=None):
    """点群ローカル -> map の厳密変換 (遅いので検証/アフィンフィット用)。
    model=enu_origin_spherical(既定): 点群=(origin_lat, origin_lon) 中心の局所 ENU -> ortho 逆で緯度経度 -> 球近似 ENU -> enu_to_map。
    model=spherical: 6677 -> 緯度経度(pyproj) -> 球近似 ENU -> enu_to_map (プロジェクト定義)。
    model=ellipsoid: 6677 -> 緯度経度 -> +proj=ortho(楕円体) ENU -> enu_to_map。"""
    g = g or GeoParams()
    lat, lon = cloud2latlon(x, y, g)
    if g.model in ("spherical", "enu_origin_spherical"):
        return latlon_to_map(lat, lon, g)
    _, enu = _transformers(g)
    e, n = enu.transform(lon, lat)
    return _enu_to_map(e, n, g)


@dataclass
class Affine:
    A: np.ndarray
    t: np.ndarray
    max_residual: float = 0.0

    def apply(self, x, y):
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        return (self.A[0, 0] * x + self.A[0, 1] * y + self.t[0],
                self.A[1, 0] * x + self.A[1, 1] * y + self.t[1])


def fit_affine(xmin, xmax, ymin, ymax, step=10.0, g=None):
    """範囲内の格子点で cloud2map_exact を 2D アフィンに最小二乗フィット。"""
    g = g or GeoParams()
    xs = np.arange(xmin, xmax + step, step)
    ys = np.arange(ymin, ymax + step, step)
    X, Y = np.meshgrid(xs, ys)
    X, Y = X.ravel(), Y.ravel()
    MX, MY = cloud2map_exact(X, Y, g)
    D = np.c_[X, Y, np.ones_like(X)]
    cx, *_ = np.linalg.lstsq(D, MX, rcond=None)
    cy, *_ = np.linalg.lstsq(D, MY, rcond=None)
    A = np.array([[cx[0], cx[1]], [cy[0], cy[1]]])
    t = np.array([cx[2], cy[2]])
    res = np.hypot(D @ cx - MX, D @ cy - MY).max()
    return Affine(A, t, float(res))


# ---------------------------------------------------------------- 設定
@dataclass
class Config:
    cell: float = 0.2              # 地面推定グリッド[m]
    open_window: float = 8.0       # グレーオープニング窓[m]
    smooth_window: float = 1.0     # 平滑化窓[m]
    ground_tol: float = 0.25       # セル最小z - 地表面 < tol -> 地面
    smooth_dz: float = 0.04        # 地面値の範囲制限付き3x3平均の閾値[m] (最小値ノイズ低減)
    link_dz: float = 0.2           # 隣接セル間の連結とみなす最大段差[m]
    robust_bin: float = 0.025      # ロバスト地面推定のz-ビン幅[m]
    robust_nbins: int = 80         # セル最小zから上へのビン数(幅 = bin*nbins)
    robust_k: int = 5              # 地面とみなす最初のビンの最小点数
    min_comp_area: float = 300.0   # 地面とみなす連結成分の最小面積[m^2]
    valid_margin: float = 3.0      # 有効範囲: 点のあるセルの膨張量[m]
    terrain_tris: int = 500_000
    structures_tris: int = 4_000_000   # 上限目安: 超えたらボクセルを voxel_coarse に上げる(間引きはしない)
    collision_tris: int = 1_000_000    # 目安(超過してもエラーにはしない)
    voxel_coarse: float = 0.15
    clear_radius: float = 1.0      # 経路折れ線からの水平距離[m]
    clear_max_h: float = 0.25      # セルの構造物最大hがこれ以下なら除去
    clear_run_bottom: float = 0.25 # 地面に接する run とみなす下端 h の上限[m]
    clear_tol: float = 0.02        # 最上ボクセル中心の h 判定の許容(地面ノイズ吸収)[m]
    min_count_coarse: int = 10     # 0.15m ボクセル時のノイズ除去しきい値(三角形数を4M以内にするため)
    hcol_min: float = 0.15
    col_bottom: float = 0.05       # collision 柱の下端(地形より下)
    vz_col: float = 0.1            # collision 柱の高さ量子化
    col_isolated_count: int = 10   # 孤立セルをノイズとして除く点数しきい値
    hmin: float = 0.10
    hmax: float = 15.0
    hcol_max: float = 1.2
    voxel: float = 0.1
    min_count: int = 3
    min_count_col: int = 3
    chunk: int = 4_000_000


# ---------------------------------------------------------------- グリッド
def grid_for_bounds(xmin, xmax, ymin, ymax, cell, pad=0.0):
    x0 = np.floor((xmin - pad) / cell) * cell
    y0 = np.floor((ymin - pad) / cell) * cell
    nx = int(np.ceil((xmax + pad - x0) / cell)) + 1
    ny = int(np.ceil((ymax + pad - y0) / cell)) + 1
    return (float(x0), float(y0)), (ny, nx)


def compute_cellmin(factory, origin, shape, cell):
    ny, nx = shape
    cm = np.full(ny * nx, np.inf, np.float32)
    for pts in factory():
        ix = np.floor((pts[:, 0] - origin[0]) / cell).astype(np.int64)
        iy = np.floor((pts[:, 1] - origin[1]) / cell).astype(np.int64)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        key = iy[ok] * nx + ix[ok]
        np.minimum.at(cm, key, pts[ok, 2])
    cm = cm.reshape(ny, nx)
    cm[np.isinf(cm)] = np.nan
    return cm


def compute_cellground(factory, origin, shape, cell, cellmin, cfg):
    """セル最小z(下方外れ値に弱い)の代わりに、セル内 z ヒストグラムで
    「点数>=robust_k の最初のビン」から上方3ビンの重み付き平均を地面高さとする。
    該当ビンが無いセルは cellmin を使う。"""
    ny, nx = shape
    occ = ~np.isnan(cellmin)
    cid = np.full(ny * nx, -1, np.int64)
    n = int(occ.sum())
    cid[occ.ravel()] = np.arange(n)
    nb, bw = cfg.robust_nbins, cfg.robust_bin
    zlow = cellmin.ravel()[occ.ravel()].astype(np.float64)
    hist = np.zeros(n * nb, np.uint16)
    for pts in factory():
        ix = np.floor((pts[:, 0] - origin[0]) / cell).astype(np.int64)
        iy = np.floor((pts[:, 1] - origin[1]) / cell).astype(np.int64)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        c = cid[iy[ok] * nx + ix[ok]]
        b = np.floor((pts[ok, 2] - zlow[c]) / bw).astype(np.int64)
        good = (b >= 0) & (b < nb)
        uk, cnt = np.unique(c[good] * nb + b[good], return_counts=True)
        hist[uk] = np.minimum(hist[uk].astype(np.int64) + cnt, 65535).astype(np.uint16)
    hist = hist.reshape(n, nb)
    out = zlow.copy()
    blk = 200_000
    for a in range(0, n, blk):
        h = hist[a:a + blk].astype(np.float64)
        m = h >= cfg.robust_k
        has = m.any(axis=1)
        b0 = m.argmax(axis=1)
        rows = np.arange(len(h))
        num = np.zeros(len(h))
        den = np.zeros(len(h))
        for j in range(4):
            bj = np.minimum(b0 + j, nb - 1)
            w = h[rows, bj] * ((b0 + j) < nb)
            num += w * (bj + 0.5) * bw
            den += w
        est = zlow[a:a + blk] + num / np.maximum(den, 1)
        out[a:a + blk] = np.where(has, est, zlow[a:a + blk])
    res = np.full(ny * nx, np.nan, np.float32)
    res[occ.ravel()] = out
    return res.reshape(ny, nx)


def clear_decision(xc, yc, htop, regions, origin, shape, cfg, log=None):
    """柱(中心 xc,yc / 最上ボクセル中心の h = htop)ごとの除去判定。
    いずれかの領域で「中心が折れ線から radius 以内」かつ htop <= max_h + clear_tol なら True。"""
    rm = np.zeros(len(xc), bool)
    ny, nx = shape
    for poly, rad, mh in regions:
        cand = htop <= mh + cfg.clear_tol
        if not cand.any():
            continue
        pm = path_cell_mask(poly, origin, shape, cfg.cell, rad + cfg.cell)
        ix = np.clip(np.floor((xc - origin[0]) / cfg.cell).astype(np.int64), 0, nx - 1)
        iy = np.clip(np.floor((yc - origin[1]) / cfg.cell).astype(np.int64), 0, ny - 1)
        idx = np.flatnonzero(cand & pm[iy, ix])
        if len(idx) == 0:
            continue
        P = np.c_[xc[idx], yc[idx]]
        dmin = np.full(len(idx), np.inf)
        segs = list(zip(poly[:-1], poly[1:])) if len(poly) > 1 else [(poly[0], poly[0])]
        for a, b in segs:
            d = b - a
            L2 = float(d @ d)
            t = np.zeros(len(P)) if L2 < 1e-12 else np.clip(((P[:, 0] - a[0]) * d[0] + (P[:, 1] - a[1]) * d[1]) / L2, 0, 1)
            dmin = np.minimum(dmin, np.hypot(P[:, 0] - (a[0] + t * d[0]), P[:, 1] - (a[1] + t * d[1])))
        hit = idx[dmin <= rad]
        if log:
            log(f"region ({poly[0][0]:.1f},{poly[0][1]:.1f})-({poly[-1][0]:.1f},{poly[-1][1]:.1f}) r={rad} max_h={mh}: "
                f"removed {int((htop[hit] <= mh + cfg.clear_tol).sum())} columns")
        rm[hit] = True
    return rm


def vertical_runs(cid, k):
    """(柱ID, 鉛直インデックス) から、鉛直に連続する run を求める(空き1つで分割)。
    戻り値: run_id(各ボクセル), first(各 run の先頭ボクセル index), kstart, kend"""
    o = np.lexsort((k, cid))
    c, kk = cid[o], k[o]
    new = np.ones(len(o), bool)
    new[1:] = (c[1:] != c[:-1]) | (kk[1:] != kk[:-1] + 1)
    st = np.flatnonzero(new)
    en = np.r_[st[1:], len(o)] - 1
    run_id = np.empty(len(o), np.int64)
    run_id[o] = np.cumsum(new) - 1
    return run_id, o[st], kk[st], kk[en]


def in_box_footprint(x, y, box, extra=0.0):
    cx, cy = box["center_x"], box["center_y"]
    c, s = np.cos(np.radians(box["yaw_deg"])), np.sin(np.radians(box["yaw_deg"]))
    u = c * (x - cx) + s * (y - cy)
    w = -s * (x - cx) + c * (y - cy)
    return (np.abs(u) <= box["size_x"] / 2 + extra) & (np.abs(w) <= box["size_y"] / 2 + extra)


def oriented_box_mesh(cx, cy, yaw_deg, sx, sy, zb, zt):
    """向き付き直方体(外向き巻き)。頂点 8, 三角形 12。"""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    uv = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
    v = []
    for z in (zb, zt):
        for u, w in uv:
            v.append((cx + c * u - s * w, cy + s * u + c * w, z))
    f = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        f += [(i, j, j + 4), (i, j + 4, i + 4)]
    return np.array(v, float), np.array(f, np.int64)


def load_boxes(path):
    import yaml
    d = yaml.safe_load(open(path)) or {}
    keys = ("center_x", "center_y", "yaw_deg", "size_x", "size_y", "z_bottom_h", "z_top_h", "margin")
    return [{k: float(b[k]) for k in keys} for b in d.get("boxes", [])]


def add_override_boxes(sv, sf, boxes, gr, origin, cfg, collision):
    for bx in boxes:
        ny, nx = gr.ground.shape
        half = max(bx["size_x"], bx["size_y"])
        ix0 = max(int((bx["center_x"] - half - origin[0]) / cfg.cell), 0)
        ix1 = min(int((bx["center_x"] + half - origin[0]) / cfg.cell) + 1, nx)
        iy0 = max(int((bx["center_y"] - half - origin[1]) / cfg.cell), 0)
        iy1 = min(int((bx["center_y"] + half - origin[1]) / cfg.cell) + 1, ny)
        X, Y = np.meshgrid(origin[0] + (np.arange(ix0, ix1) + 0.5) * cfg.cell,
                           origin[1] + (np.arange(iy0, iy1) + 0.5) * cfg.cell)
        m = in_box_footprint(X, Y, bx, cfg.cell * 0.5)
        G = gr.ground[iy0:iy1, ix0:ix1]
        gc = gr.ground[min(max(int((bx["center_y"] - origin[1]) / cfg.cell), 0), ny - 1),
                       min(max(int((bx["center_x"] - origin[0]) / cfg.cell), 0), nx - 1)]
        zb = float(G[m].min() if m.any() else gc) - cfg.col_bottom
        top = min(bx["z_top_h"], cfg.hcol_max) if collision else bx["z_top_h"]
        v, f = oriented_box_mesh(bx["center_x"], bx["center_y"], bx["yaw_deg"], bx["size_x"], bx["size_y"],
                                 zb, float(gc) + top)
        sf = np.concatenate([sf, f + len(sv)]) if len(sf) else f
        sv = np.concatenate([sv, v]) if len(sv) else v
    return sv, sf


def parse_zone(txt):
    v = tuple(float(t) for t in txt.split(","))
    if len(v) != 6:
        raise ValueError("--clear-zone は x1,y1,x2,y2,radius,max_h")
    return v


def load_zones(path):
    import yaml
    d = yaml.safe_load(open(path)) or {}
    return [tuple(float(z[k]) for k in ("x1", "y1", "x2", "y2", "radius", "max_h"))
            for z in d.get("zones", [])]


def path_cell_mask(poly, origin, shape, cell, radius):
    """折れ線(M,2)から radius 以内にセル中心を持つセルの mask (ny,nx)。判定はセル中心-線分間距離。"""
    ny, nx = shape
    poly = np.asarray(poly, np.float64)
    mask = np.zeros(shape, bool)
    segs = list(zip(poly[:-1], poly[1:])) if len(poly) > 1 else [(poly[0], poly[0])]
    for a, b in segs:
        lo = np.minimum(a, b) - radius
        hi = np.maximum(a, b) + radius
        ix0 = max(int(np.floor((lo[0] - origin[0]) / cell)) - 1, 0)
        ix1 = min(int(np.ceil((hi[0] - origin[0]) / cell)) + 1, nx)
        iy0 = max(int(np.floor((lo[1] - origin[1]) / cell)) - 1, 0)
        iy1 = min(int(np.ceil((hi[1] - origin[1]) / cell)) + 1, ny)
        if ix0 >= ix1 or iy0 >= iy1:
            continue
        cx = origin[0] + (np.arange(ix0, ix1) + 0.5) * cell
        cy = origin[1] + (np.arange(iy0, iy1) + 0.5) * cell
        CX, CY = np.meshgrid(cx, cy)
        d = b - a
        L2 = float(d @ d)
        t = np.zeros_like(CX) if L2 < 1e-12 else np.clip(((CX - a[0]) * d[0] + (CY - a[1]) * d[1]) / L2, 0, 1)
        dist = np.hypot(CX - (a[0] + t * d[0]), CY - (a[1] + t * d[1]))
        mask[iy0:iy1, ix0:ix1] |= dist <= radius
    return mask


@dataclass
class Ground:
    ground: np.ndarray      # 全セルの地面高さ(補間含む)
    is_ground: np.ndarray   # 実測由来の地面セル
    valid: np.ndarray       # メッシュ化する有効範囲
    occupied: np.ndarray


def _nearest_fill(arr, mask_known):
    from scipy import ndimage as ndi
    idx = ndi.distance_transform_edt(~mask_known, return_distances=False, return_indices=True)
    return arr[idx[0], idx[1]]


def _interp_fill(z, known):
    """known セルの値から他セルを線形補間(凸包外は最近傍)。"""
    from scipy.interpolate import LinearNDInterpolator
    ky, kx = np.nonzero(known)
    pts = np.c_[kx, ky].astype(np.float64)
    vals = z[ky, kx].astype(np.float64)
    out = _nearest_fill(z, known).astype(np.float64)
    uy, ux = np.nonzero(~known)
    if len(uy) == 0:
        return out
    try:
        lin = LinearNDInterpolator(pts, vals)
        v = lin(np.c_[ux, uy].astype(np.float64))
        good = ~np.isnan(v)
        out[uy[good], ux[good]] = v[good]
    except Exception:
        pass
    return out


def _bilateral3(z, mask, dz):
    """3x3 範囲制限付き平均。|差|<dz の地面セルだけ平均する(縁石などの段差は保存)。"""
    ny, nx = z.shape
    zp = np.pad(np.where(mask, z, 0.0), 1)
    mp = np.pad(mask, 1)
    acc = np.zeros_like(z, dtype=np.float64)
    cnt = np.zeros_like(z, dtype=np.float64)
    for dy in range(3):
        for dx in range(3):
            zn = zp[dy:dy + ny, dx:dx + nx]
            ok = mp[dy:dy + ny, dx:dx + nx] & (np.abs(zn - z) < dz)
            acc += np.where(ok, zn, 0.0)
            cnt += ok
    return np.where(mask, acc / np.maximum(cnt, 1), z)


def estimate_ground(cellmin, cfg):
    from scipy import ndimage as ndi
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    occ = ~np.isnan(cellmin)
    if not occ.any():
        raise ValueError("no points")
    ny, nx = cellmin.shape
    filled = _nearest_fill(np.nan_to_num(cellmin), occ).astype(np.float32)

    w = max(3, int(round(cfg.open_window / cfg.cell)))
    op = ndi.minimum_filter(filled, size=w, mode="nearest")
    op = ndi.maximum_filter(op, size=w, mode="nearest")
    sw = max(1, int(round(cfg.smooth_window / cfg.cell)))
    surf0 = ndi.uniform_filter(op, size=sw * 5, mode="nearest")
    cand = occ & (np.nan_to_num(cellmin) - surf0 < cfg.ground_tol)

    # 段差の小さい隣接セルで連結成分化し、小さい島(屋根など)を除外
    zz = np.nan_to_num(cellmin)
    zraw = zz
    idx = np.arange(ny * nx).reshape(ny, nx)
    rows, cols = [], []
    for sl_a, sl_b in (((slice(None), slice(0, -1)), (slice(None), slice(1, None))),
                       ((slice(0, -1), slice(None)), (slice(1, None), slice(None)))):
        ok = cand[sl_a] & cand[sl_b] & (np.abs(zz[sl_a] - zz[sl_b]) < cfg.link_dz)
        rows.append(idx[sl_a][ok])
        cols.append(idx[sl_b][ok])
    rows = np.concatenate(rows)
    cols = np.concatenate(cols)
    g = coo_matrix((np.ones(len(rows), np.int8), (rows, cols)), shape=(ny * nx, ny * nx))
    _, lab = connected_components(g, directed=False)
    lab = lab.reshape(ny, nx)
    lab_c = np.where(cand, lab, -1)
    cnt = np.bincount(lab_c[cand])
    min_cells = cfg.min_comp_area / cfg.cell ** 2
    keep_lab = cnt >= min_cells
    if not keep_lab.any():
        keep_lab = cnt == cnt.max()
    is_g0 = cand & keep_lab[np.where(cand, lab, 0)]

    # 補間 -> 再判定
    # セル最小値は斜面で下り側の縁を拾うため、地面補間面の勾配でセル中心値に補正する(2段階)
    interp_a = _interp_fill(np.where(is_g0, zraw, 0).astype(np.float32), is_g0)
    gy, gx = np.gradient(ndi.uniform_filter(interp_a, size=5, mode="nearest"), cfg.cell)
    zz = zraw + 0.5 * cfg.cell * (np.abs(gx) + np.abs(gy))
    zs0 = _bilateral3(zz, is_g0, cfg.smooth_dz)
    interp0 = _interp_fill(np.where(is_g0, zs0, 0).astype(np.float32), is_g0).astype(np.float32)
    is_g = occ & ((zraw - interp0 < cfg.ground_tol) | is_g0)
    # 屋根のように is_g0 でないが interp より高いセルは上の式で非地面になる
    zs = _bilateral3(zz, is_g, cfg.smooth_dz)
    ground = np.where(is_g, zs, interp0).astype(np.float32)

    r = cfg.valid_margin / cfg.cell
    dist = ndi.distance_transform_edt(~occ)
    valid = ndi.binary_fill_holes(dist <= r)
    return Ground(ground, is_g, valid, occ)


# ---------------------------------------------------------------- メッシュ
def terrain_mesh(gr, origin, cell):
    ny, nx = gr.ground.shape
    v = gr.valid
    quad = v[:-1, :-1] & v[:-1, 1:] & v[1:, 1:] & v[1:, :-1]
    idx = np.arange(ny * nx).reshape(ny, nx)
    a = idx[:-1, :-1][quad]
    b = idx[:-1, 1:][quad]
    c = idx[1:, 1:][quad]
    d = idx[1:, :-1][quad]
    faces = np.r_[np.c_[a, b, c], np.c_[a, c, d]]
    used = np.unique(faces)
    remap = np.full(ny * nx, -1, np.int64)
    remap[used] = np.arange(len(used))
    faces = remap[faces]
    iy, ix = np.divmod(used, nx)
    verts = np.c_[origin[0] + (ix + 0.5) * cell, origin[1] + (iy + 0.5) * cell,
                  gr.ground.ravel()[used]]
    return verts.astype(np.float64), faces.astype(np.int64)


def decimate_mesh(verts, faces, target, agg=5, method="fast"):
    """QEM による間引き。fast_simplification を優先(高速・確実に目標へ到達)、無ければ open3d。
    open3d は非多様体部で止まり予算に届かないことがあったため構造物では fast を使う。
    ただし fast は開境界を保存せず地形(境界あり)で大きく崩れるため、地形は method="open3d"。"""
    if len(faces) <= target:
        return np.asarray(verts), np.asarray(faces)
    try:
        if method != "fast":
            raise ImportError
        import fast_simplification as fs
        v = np.asarray(verts, np.float32)
        f = np.asarray(faces, np.int32)
        for _ in range(4):
            v, f = fs.simplify(v, f, target_count=int(target), agg=agg)
            if len(f) <= target:
                break
        return np.asarray(v, np.float64), np.asarray(f, np.int64)
    except ImportError:
        import open3d as o3d
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(np.asarray(verts, np.float64))
        m.triangles = o3d.utility.Vector3iVector(np.asarray(faces, np.int32))
        m = m.simplify_quadric_decimation(target_number_of_triangles=int(target))
        m.remove_unreferenced_vertices()
        return np.asarray(m.vertices), np.asarray(m.triangles)


def mesh_heights(verts, faces, x, y, z0=1000.0):
    """鉛直レイキャストでメッシュ高さを返す(ミス=nan)。"""
    import open3d as o3d
    import open3d.core as o3c
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3c.Tensor(np.asarray(verts, np.float32)),
                     o3c.Tensor(np.asarray(faces, np.uint32)))
    out = np.full(len(x), np.nan)
    for i in range(0, len(x), 500_000):
        xs, ys = x[i:i + 500_000], y[i:i + 500_000]
        rays = np.c_[xs, ys, np.full(len(xs), z0), np.zeros(len(xs)), np.zeros(len(xs)),
                     -np.ones(len(xs))].astype(np.float32)
        t = sc.cast_rays(o3c.Tensor(rays))["t_hit"].numpy()
        out[i:i + 500_000] = np.where(np.isfinite(t), z0 - t, np.nan)
    return out


def voxel_surface(ijk, _reserved, voxel, origin, extra=None):
    """占有ボクセルの露出面を greedy meshing(同一平面・同一向きの隣接面を長方形に結合)し、
    外向き巻きの三角形メッシュにする。間引きはしない(頂点は全てボクセル角)。
    ijk: (N,3) int, voxel: スカラまたは(3,)。extra: 占有判定のみに使う隣接ボクセル。"""
    vox = np.broadcast_to(np.asarray(voxel, np.float64), (3,))
    ijk = np.asarray(ijk, np.int64)
    has_extra = extra is not None and len(extra) > 0
    allv = np.concatenate([ijk, np.asarray(extra, np.int64)]) if has_extra else ijk
    lo = allv.min(axis=0) - 1
    D = allv.max(axis=0) - lo + 3
    ijk = ijk - lo
    key = (ijk[:, 0] * D[1] + ijk[:, 1]) * D[2] + ijk[:, 2]
    if has_extra:
        e = np.asarray(extra, np.int64) - lo
        skey = np.sort(np.concatenate([key, (e[:, 0] * D[1] + e[:, 1]) * D[2] + e[:, 2]]))
    else:
        skey = np.sort(key)
    strides = np.array([D[1] * D[2], D[2], 1], np.int64)
    Dc = D + 1
    cstr = np.array([Dc[1] * Dc[2], Dc[2], 1], np.int64)
    ckeys = []
    for a in range(3):
        b, c = (a + 1) % 3, (a + 2) % 3
        for sgn in (+1, -1):
            nk = key + sgn * strides[a]
            pos = np.searchsorted(skey, nk)
            pos[pos >= len(skey)] = len(skey) - 1
            sel = ijk[skey[pos] != nk]
            if len(sel) == 0:
                continue
            P = sel[:, a] + (1 if sgn > 0 else 0)
            U = sel[:, b]
            W = sel[:, c]
            del sel
            # 段階1: u 方向の連続を run にする
            o = np.lexsort((U, W, P))
            P, W, U = P[o], W[o], U[o]
            n = len(P)
            new = np.ones(n, bool)
            new[1:] = (P[1:] != P[:-1]) | (W[1:] != W[:-1]) | (U[1:] != U[:-1] + 1)
            st = np.flatnonzero(new)
            ulen = np.diff(np.r_[st, n])
            rp, rw, ru = P[st], W[st], U[st]
            del P, W, U, o, new
            # 段階2: 同じ (p,u0,ulen) で w が連続する run を結合
            o = np.lexsort((rw, ulen, ru, rp))
            rp, rw, ru, ulen = rp[o], rw[o], ru[o], ulen[o]
            m = len(rp)
            new = np.ones(m, bool)
            new[1:] = ((rp[1:] != rp[:-1]) | (ru[1:] != ru[:-1]) | (ulen[1:] != ulen[:-1])
                       | (rw[1:] != rw[:-1] + 1))
            st = np.flatnonzero(new)
            wlen = np.diff(np.r_[st, m])
            rp, rw, ru, ulen = rp[st], rw[st], ru[st], ulen[st]
            if sgn > 0:
                offs = [(0, 0), (1, 0), (1, 1), (0, 1)]
            else:
                offs = [(0, 0), (0, 1), (1, 1), (1, 0)]
            cs = []
            for ob, oc in offs:
                lat = np.empty((len(rp), 3), np.int64)
                lat[:, a] = rp
                lat[:, b] = ru + ob * ulen
                lat[:, c] = rw + oc * wlen
                cs.append(lat @ cstr)
            ckeys.append(np.stack(cs, axis=1))
    if not ckeys:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    ck = np.concatenate(ckeys)
    uniq, inv = np.unique(ck.ravel(), return_inverse=True)
    inv = inv.reshape(-1, 4).astype(np.int64)
    faces = np.concatenate([inv[:, [0, 1, 2]], inv[:, [0, 2, 3]]])
    i0 = uniq // cstr[0]
    rem = uniq - i0 * cstr[0]
    i1 = rem // cstr[1]
    i2 = rem - i1 * cstr[1]
    lat = np.c_[i0, i1, i2] + lo
    verts = lat * vox + np.asarray(origin, np.float64)
    return verts, faces


def columns_to_voxels(ij, kbot, ktop):
    """セル(i,j)ごとに k∈[kbot,ktop) の柱をボクセル列挙する。"""
    ij = np.asarray(ij, np.int64)
    n = np.maximum(np.asarray(ktop, np.int64) - np.asarray(kbot, np.int64), 0)
    rep = np.repeat(np.arange(len(ij)), n)
    start = np.cumsum(n) - n
    k = np.arange(n.sum()) - np.repeat(start, n) + np.asarray(kbot, np.int64)[rep]
    return np.c_[ij[rep], k]


class VoxelAccumulator:
    def __init__(self, flush=10_000_000):
        self.k, self.c, self.n, self.flush = [], [], 0, flush
        self.keys = np.zeros(0, np.int64)
        self.counts = np.zeros(0, np.int64)

    def add(self, keys):
        u, c = np.unique(keys, return_counts=True)
        self.k.append(u)
        self.c.append(c)
        self.n += len(u)
        if self.n > self.flush:
            self._merge()

    def _merge(self):
        if not self.k:
            return
        k = np.concatenate([self.keys] + self.k)
        c = np.concatenate([self.counts] + self.c)
        u, inv = np.unique(k, return_inverse=True)
        self.keys = u
        self.counts = np.bincount(inv, weights=c).astype(np.int64)
        self.k, self.c, self.n = [], [], 0

    def result(self):
        self._merge()
        return self.keys, self.counts


def tiled_voxel_mesh(ijk, voxel, origin, max_voxels=3_500_000, log=print):
    """ボクセルを i 方向のスラブに分割して greedy meshing し連結する(メモリ節約、間引きなし)。
    露出判定は隣接スラブのボクセルも参照するので、境界で面の欠け/重複は無い
    (長方形の結合がスラブ境界で切れるだけ)。"""
    ijk = ijk[np.argsort(ijk[:, 0], kind="stable")]
    ii = ijk[:, 0]
    cuts = [0]
    while cuts[-1] < len(ijk):
        e = min(cuts[-1] + max_voxels, len(ijk))
        if e < len(ijk):
            e = int(np.searchsorted(ii, ii[e - 1], side="right"))
        cuts.append(e)
    vs, fs, off = [], [], 0
    for a, b in zip(cuts[:-1], cuts[1:]):
        l0 = int(np.searchsorted(ii, ii[a] - 1, side="left"))
        l1 = int(np.searchsorted(ii, ii[b - 1] + 1, side="right"))
        extra = np.concatenate([ijk[l0:a], ijk[b:l1]])
        v, f = voxel_surface(ijk[a:b], 0, voxel, origin, extra=extra)
        log(f"  tile voxels={b - a} tris={len(f)}")
        vs.append(v)
        fs.append(f + off)
        off += len(v)
    return np.concatenate(vs), np.concatenate(fs)


def write_obj(path, verts, faces, normals="vertex", block=500_000):
    """OBJ 出力。法線は必ず出す(Gazebo dartsim/ODE は法線無しメッシュで落ちる)。
    normals="vertex": 頂点法線(面積加重平均)、f a//a ...
    normals="face6" : 軸整列メッシュ用。向きごとの6本の vn を共有、f a//n ...
    法線は巻き順(右手系)と一致=外向き。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    verts = np.asarray(verts, np.float64)
    faces = np.asarray(faces, np.int64)
    with open(path, "w") as fh:
        for i in range(0, len(verts), block):
            np.savetxt(fh, verts[i:i + block], fmt="v %.3f %.3f %.3f")
        if normals == "vertex":
            vn = np.zeros_like(verts)
            for i in range(0, len(faces), block):
                f = faces[i:i + block]
                cr = np.cross(verts[f[:, 1]] - verts[f[:, 0]], verts[f[:, 2]] - verts[f[:, 0]])
                for k in range(3):
                    np.add.at(vn, f[:, k], cr)
            L = np.linalg.norm(vn, axis=1)
            bad = L < 1e-12
            vn[bad] = (0.0, 0.0, 1.0)
            L[bad] = 1.0
            vn /= L[:, None]
            for i in range(0, len(vn), block):
                np.savetxt(fh, vn[i:i + block], fmt="vn %.4f %.4f %.4f")
            for i in range(0, len(faces), block):
                f = faces[i:i + block] + 1
                np.savetxt(fh, np.c_[f[:, 0], f[:, 0], f[:, 1], f[:, 1], f[:, 2], f[:, 2]],
                           fmt="f %d//%d %d//%d %d//%d")
        else:
            nn = np.zeros((len(faces), 3), np.int32)
            for i in range(0, len(faces), block):
                f = faces[i:i + block]
                cr = np.cross(verts[f[:, 1]] - verts[f[:, 0]], verts[f[:, 2]] - verts[f[:, 0]])
                L = np.linalg.norm(cr, axis=1)
                bad = L < 1e-12
                cr[bad] = (0.0, 0.0, 1.0)
                L[bad] = 1.0
                nn[i:i + block] = np.rint(cr / L[:, None] * 10000).astype(np.int32)
            key = ((nn[:, 0].astype(np.int64) + 10001) * 20003 + (nn[:, 1] + 10001)) * 20003 + (nn[:, 2] + 10001)
            uk, inv = np.unique(key, return_inverse=True)
            z = uk % 20003 - 10001
            y = (uk // 20003) % 20003 - 10001
            x = uk // (20003 * 20003) - 10001
            vn = np.c_[x, y, z] / 10000.0 + 0.0
            vn /= np.linalg.norm(vn, axis=1)[:, None]
            np.savetxt(fh, vn, fmt="vn %.4f %.4f %.4f")
            n = inv.astype(np.int64) + 1
            for i in range(0, len(faces), block):
                f = faces[i:i + block] + 1
                nb = n[i:i + block]
                np.savetxt(fh, np.c_[f[:, 0], nb, f[:, 1], nb, f[:, 2], nb], fmt="f %d//%d %d//%d %d//%d")


def fix_winding_report(verts, faces):
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


# ---------------------------------------------------------------- パイプライン
def _fmt_mem():
    return f"{resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1048576:.1f}GB"


def run_pipeline(factory, outdir, cfg=None, log=print, probes=(), cache=None, dump_ground=None, clear_path=None, clear_zones=(), box_overrides=()):
    cfg = cfg or Config()
    t0 = time.time()
    stats = {}

    def L(msg):
        log(f"[{time.time() - t0:7.1f}s peak={_fmt_mem()}] {msg}")

    # pass0/1: 範囲とセル最小z (cache があれば再利用)
    if cache and os.path.exists(cache):
        z = np.load(cache)
        mn, mx, npts, origin, shape = z["mn"], z["mx"], int(z["npts"]), tuple(z["origin"]), tuple(z["shape"])
        cellmin = z["cellmin"]
        cellrob = z["cellrob"] if "cellrob" in z.files else None
        L(f"cache loaded {cache}")
    else:
        mn = np.full(3, np.inf)
        mx = np.full(3, -np.inf)
        npts = 0
        for pts in factory():
            mn = np.minimum(mn, pts.min(axis=0))
            mx = np.maximum(mx, pts.max(axis=0))
            npts += len(pts)
        origin, shape = grid_for_bounds(mn[0], mx[0], mn[1], mx[1], cfg.cell, pad=0.0)
        L(f"points={npts} bounds min={mn} max={mx}")
        cellmin = compute_cellmin(factory, origin, shape, cfg.cell)
        cellrob = None
    if cellrob is None:
        cellrob = compute_cellground(factory, origin, shape, cfg.cell, cellmin, cfg)
        L("robust cell ground done")
        if cache:
            np.savez(cache, mn=mn, mx=mx, npts=npts, origin=np.array(origin), shape=np.array(shape),
                     cellmin=cellmin, cellrob=cellrob)
    cellmin = cellrob
    stats["points"] = npts
    gr = estimate_ground(cellmin, cfg)
    L(f"ground cells={int(gr.is_ground.sum())} valid={int(gr.valid.sum())}")
    stats["ground_cells"] = int(gr.is_ground.sum())
    if dump_ground:
        np.savez(dump_ground, ground=gr.ground, is_ground=gr.is_ground, valid=gr.valid, origin=np.array(origin))
    del cellmin

    # 地形メッシュ
    verts, faces = terrain_mesh(gr, origin, cfg.cell)
    L(f"terrain full: verts={len(verts)} tris={len(faces)}")
    dv, df = decimate_mesh(verts, faces, cfg.terrain_tris, method="open3d")
    L(f"terrain decimated tris={len(df)}")
    hz = mesh_heights(dv, df, verts[:, 0], verts[:, 1])
    err = np.abs(hz - verts[:, 2])
    fin = np.isfinite(err)
    stats["terrain_max_err"] = float(err[fin].max())
    stats["terrain_p99_err"] = float(np.percentile(err[fin], 99))
    stats["terrain_p50_err"] = float(np.percentile(err[fin], 50))
    stats["terrain_miss"] = int((~fin).sum())
    L(f"terrain vertical err max={stats['terrain_max_err']:.3f} p99={stats['terrain_p99_err']:.3f} "
      f"p50={stats['terrain_p50_err']:.4f} miss={stats['terrain_miss']}")
    stats["terrain_tris"] = len(df)
    mdir = os.path.join(outdir, "meshes")
    write_obj(os.path.join(mdir, "terrain.obj"), dv, df)
    stats["probes"] = {}
    for (px, py) in probes:
        h = mesh_heights(dv, df, np.array([px]), np.array([py]))[0]
        stats["probes"][(px, py)] = float(h)
        L(f"terrain height at ({px}, {py}) = {h:.3f}")
    del verts, faces, dv, df, hz, err

    # pass2: 構造物
    ny, nx = shape
    zref = float(np.floor(gr.ground.min())) - 1.0
    zmax = float(max(mx[2], gr.ground.max()) + 1.0)
    vz = cfg.vz_col
    Dc = (nx + 2, ny + 2, int(np.ceil((zmax - zref) / vz)) + 2)
    acc_col = VoxelAccumulator()
    regions = []   # (polyline, radius, max_h): 最終ボクセル/柱で判定する除去領域
    if clear_path is not None:
        regions.append((np.asarray(clear_path, float), cfg.clear_radius, cfg.clear_max_h))
    for (x1, y1, x2, y2, rad, mh) in clear_zones:
        regions.append((np.array([[x1, y1], [x2, y2]]), rad, mh))
    fine_sizes = [cfg.voxel, cfg.voxel_coarse]
    fine_idx = 0
    mdir = os.path.join(outdir, "meshes")

    def accumulate(vfine, do_col):
        Df = (int(np.ceil(nx * cfg.cell / vfine)) + 2, int(np.ceil(ny * cfg.cell / vfine)) + 2,
              int(np.ceil((zmax - zref) / vfine)) + 2)
        acc = VoxelAccumulator()
        nst = 0
        for pts in factory():
            ix = np.floor((pts[:, 0] - origin[0]) / cfg.cell).astype(np.int64)
            iy = np.floor((pts[:, 1] - origin[1]) / cfg.cell).astype(np.int64)
            ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            pts, ix, iy = pts[ok], ix[ok], iy[ok]
            ok = gr.valid[iy, ix]
            pts, ix, iy = pts[ok], ix[ok], iy[ok]
            h = pts[:, 2] - gr.ground[iy, ix]
            sel = (h >= cfg.hmin) & (h <= cfg.hmax)
            nst += int(sel.sum())
            q = pts[sel]
            ii = np.floor((q[:, 0] - origin[0]) / vfine).astype(np.int64)
            jj = np.floor((q[:, 1] - origin[1]) / vfine).astype(np.int64)
            kk = np.floor((q[:, 2] - zref) / vfine).astype(np.int64)
            acc.add((ii * Df[1] + jj) * Df[2] + kk)
            if do_col:
                s2 = (h >= cfg.hcol_min) & (h <= cfg.hcol_max)
                ii, jj = ix[s2], iy[s2]
                kk = np.floor((pts[s2, 2] - zref) / vz).astype(np.int64)
                acc_col.add((ii * Dc[1] + jj) * Dc[2] + kk)
        return acc, Df, nst

    # --- visual (0.1m ボクセルの greedy meshing。三角形数が多すぎれば 0.15m へ)
    do_col = True
    while True:
        vfine = fine_sizes[fine_idx]
        acc, Df, nstruct = accumulate(vfine, do_col)
        do_col = False
        L(f"structure points={nstruct} (voxel {vfine})")
        keys, counts = acc.result()
        keys = keys[counts >= (cfg.min_count if fine_idx == 0 else cfg.min_count_coarse)]
        k = keys // Df[2]
        ijk = np.c_[k // Df[1], k % Df[1], keys % Df[2]]
        L(f"fine: voxels(after noise filter)={len(ijk)}")
        del acc, keys, counts, k
        if (regions or box_overrides) and len(ijk):
            xv = origin[0] + (ijk[:, 0] + 0.5) * vfine
            yv = origin[1] + (ijk[:, 1] + 0.5) * vfine
            ixv = np.clip(np.floor((xv - origin[0]) / cfg.cell).astype(np.int64), 0, nx - 1)
            iyv = np.clip(np.floor((yv - origin[1]) / cfg.cell).astype(np.int64), 0, ny - 1)
            gv = gr.ground[iyv, ixv]
            hv = zref + (ijk[:, 2] + 0.5) * vfine - gv
            keep = np.ones(len(ijk), bool)
            for bx in box_overrides:   # 置換対象: footprint+margin 内の h 範囲のボクセル
                keep &= ~(in_box_footprint(xv, yv, bx, bx["margin"] + 0.7071 * vfine)
                          & (hv + vfine / 2 >= bx["z_bottom_h"]) & (hv - vfine / 2 <= bx["z_top_h"]))
            stats["override_visual_voxels"] = int((~keep).sum())
            ijk, gv = ijk[keep], gv[keep]
        if regions and len(ijk):
            # 柱(2D)を鉛直 run に分割し、run 上端 h <= max_h の run を除去。
            # 上端が高い run(屋根・樹冠・柱・幹)は残る。上面だけ観測された中空の低いブロック(宙に浮いた低い run)は除去対象
            cid = ijk[:, 0] * (ijk[:, 1].max() + 2) + ijk[:, 1]
            run_id, first, ks, ke = vertical_runs(cid, ijk[:, 2])
            xc = origin[0] + (ijk[first, 0] + 0.5) * vfine
            yc = origin[1] + (ijk[first, 1] + 0.5) * vfine
            gr_ = gv[first]
            htop = zref + (ke + 0.5) * vfine - gr_            # run 上端ボクセル中心の h
            hbot = zref + ks * vfine - gr_                    # run 下端の h
            rmc = clear_decision(xc, yc, htop, regions, origin, shape, cfg, log=lambda m: L('visual ' + m))
            rm = rmc[run_id]
            stats["clear_visual_columns"] = int(rmc.sum())
            stats["clear_visual_voxels"] = int(rm.sum())
            L(f"clear: visual removed runs={int(rmc.sum())} voxels={int(rm.sum())}")
            ijk = ijk[~rm]
        sv, sf = tiled_voxel_mesh(ijk, vfine, (origin[0], origin[1], zref),
                                  log=lambda m: L(f"fine: {m}"))
        L(f"fine: tris={len(sf)}")
        if len(sf) > cfg.structures_tris and fine_idx + 1 < len(fine_sizes):
            L(f"fine: tris > {cfg.structures_tris}, retry with voxel {fine_sizes[fine_idx + 1]}")
            fine_idx += 1
            del sv, sf, ijk
            continue
        break
    stats["structure_points"] = nstruct
    stats["structures_voxel"] = vfine
    stats["structures_voxels"] = len(ijk)
    stats["structures_tris"] = len(sf)
    sv, sf = add_override_boxes(sv, sf, box_overrides, gr, origin, cfg, collision=False)
    stats["structures_tris"] = len(sf)
    write_obj(os.path.join(mdir, "structures.obj"), sv, sf, normals="face6")
    stats["structures_vol"] = fix_winding_report(sv, sf) if len(sf) else 0.0
    del sv, sf, ijk

    # --- collision: 0.2m 2D 占有セルの柱(地形-0.05m から 最大 h まで)を greedy meshing
    keys, counts = acc_col.result()
    del acc_col
    good = counts >= cfg.min_count_col
    keys, counts = keys[good], counts[good]
    kk = keys % Dc[2]
    r = keys // Dc[2]
    jj = r % Dc[1]
    ii = r // Dc[1]
    cid = jj * nx + ii
    cnt2d = np.bincount(cid, weights=counts, minlength=ny * nx).reshape(ny, nx)
    from scipy import ndimage as ndi
    occ2 = cnt2d > 0
    nb = ndi.convolve(occ2.astype(np.int32), np.ones((3, 3), np.int32), mode="constant") - occ2
    occ2 &= ~((nb == 0) & (cnt2d < cfg.col_isolated_count))
    kv = occ2[jj, ii]
    ii, jj, kk = ii[kv], jj[kv], kk[kv]
    ijk = np.zeros((0, 3), np.int64)
    nruns = 0
    if len(ii):
        xv = origin[0] + (ii + 0.5) * cfg.cell
        yv = origin[1] + (jj + 0.5) * cfg.cell
        gv = gr.ground[jj, ii]
        if box_overrides:
            hv = zref + (kk + 0.5) * vz - gv
            kb = np.ones(len(ii), bool)
            for bx in box_overrides:
                kb &= ~(in_box_footprint(xv, yv, bx, bx["margin"] + 0.7071 * cfg.cell)
                        & (hv + vz / 2 >= bx["z_bottom_h"]) & (hv - vz / 2 <= bx["z_top_h"]))
            stats["override_collision_voxels"] = int((~kb).sum())
            ii, jj, kk, gv = ii[kb], jj[kb], kk[kb], gv[kb]
    if len(ii):
        # 鉛直 run に分割。地面に接する run で上端 h <= max_h のものを除去。残りを柱化
        run_id, first, ks, ke = vertical_runs(jj * nx + ii, kk)
        cxr, cyr, g_r = ii[first], jj[first], gv[first]
        hbot = zref + ks * vz - g_r
        htop = zref + (ke + 0.5) * vz - g_r
        grounded = hbot <= cfg.clear_run_bottom
        rmc = np.zeros(len(first), bool)
        if regions:
            xc = origin[0] + (cxr + 0.5) * cfg.cell
            yc = origin[1] + (cyr + 0.5) * cfg.cell
            rmc = clear_decision(xc, yc, htop, regions, origin, shape, cfg,
                                 log=lambda m: L('collision ' + m))
            stats["clear_collision_cells"] = int(rmc.sum())
            L(f"clear: collision removed runs={int(rmc.sum())}")
        kbot = np.where(grounded, np.floor((g_r - cfg.col_bottom - zref) / vz).astype(np.int64), ks)
        ktop = np.maximum(ke + 1, kbot + 1)
        sel = ~rmc
        ijk = columns_to_voxels(np.c_[cxr, cyr][sel], kbot[sel], ktop[sel])
        ijk = np.unique(ijk, axis=0)
        nruns = int(sel.sum())
    L(f"col: runs={nruns} voxels={len(ijk)}")
    stats["col_cells"] = nruns
    if len(ijk):
        sv, sf = tiled_voxel_mesh(ijk, (cfg.cell, cfg.cell, vz), (origin[0], origin[1], zref),
                                  log=lambda m: L(f"col: {m}"))
    else:
        sv, sf = np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    L(f"col: tris={len(sf)}")
    stats["structures_collision_tris"] = len(sf)
    sv, sf = add_override_boxes(sv, sf, box_overrides, gr, origin, cfg, collision=True)
    stats["structures_collision_tris"] = len(sf)
    write_obj(os.path.join(mdir, "structures_collision.obj"), sv, sf, normals="face6")
    stats["structures_collision_vol"] = fix_winding_report(sv, sf) if len(sf) else 0.0
    del sv, sf, ijk, keys, counts
    write_model_files(outdir)
    L("done")
    return stats


# ---------------------------------------------------------------- モデルファイル
SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="tsukuba">
    <static>true</static>
    <link name="link">
      <visual name="terrain_visual">
        <geometry><mesh><uri>model://tsukuba/meshes/terrain.obj</uri></mesh></geometry>
        <material>
          <ambient>0.62 0.62 0.60 1</ambient>
          <diffuse>0.62 0.62 0.60 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
      <collision name="terrain_collision">
        <geometry><mesh><uri>model://tsukuba/meshes/terrain.obj</uri></mesh></geometry>
      </collision>
      <visual name="structures_visual">
        <geometry><mesh><uri>model://tsukuba/meshes/structures.obj</uri></mesh></geometry>
        <material>
          <ambient>0.55 0.60 0.68 1</ambient>
          <diffuse>0.55 0.60 0.68 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
      <collision name="structures_collision">
        <geometry><mesh><uri>model://tsukuba/meshes/structures_collision.obj</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

CONFIG = """<?xml version="1.0"?>
<model>
  <name>tsukuba</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    つくば高精度点群地図から生成した地形+構造物メッシュ (map 座標そのまま、モデル姿勢は原点)。
    tools/pcd_to_world/pcd_to_world.py で生成。手で編集しないこと。
  </description>
</model>
"""


def write_model_files(outdir):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "model.sdf"), "w") as f:
        f.write(SDF)
    with open(os.path.join(outdir, "model.config"), "w") as f:
        f.write(CONFIG)


# ---------------------------------------------------------------- PCD 入力
def pcd_chunks_factory(path, aff, zoff, chunk, stride=1):
    with open(path, "rb") as f:
        off = 0
        npts = 0
        while True:
            line = f.readline()
            off += len(line)
            if line.startswith(b"POINTS"):
                npts = int(line.split()[1])
            if line.startswith(b"DATA"):
                break
    mm = np.memmap(path, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")]),
                   mode="r", offset=off, shape=(npts,))

    def factory():
        for i in range(0, npts, chunk):
            blk = mm[i:i + chunk:stride]
            x = np.asarray(blk["x"], np.float64)
            y = np.asarray(blk["y"], np.float64)
            z = np.asarray(blk["z"], np.float64)
            ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            mx, my = aff.apply(x[ok], y[ok])
            yield np.c_[mx, my, z[ok] + zoff].astype(np.float32)
    return factory, npts, mm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--pcd", default="/home/kyo/Downloads/tsukuba_map/cleaned_dense_2p5cm_tsukuba_jgd2024_intensity.pcd")
    ap.add_argument("--out", default=os.path.normpath(os.path.join(here, "..", "..", "models", "tsukuba")))
    ap.add_argument("--stride", type=int, default=1, help="点の間引き(開発用)")
    ap.add_argument("--geo-model", choices=["enu_origin_spherical", "spherical", "ellipsoid"], default="enu_origin_spherical")
    ap.add_argument("--origin-lat", type=float, default=GeoParams.origin_lat)
    ap.add_argument("--origin-lon", type=float, default=GeoParams.origin_lon)
    ap.add_argument("--oe", type=float, default=GeoParams.oe)
    ap.add_argument("--on", type=float, default=GeoParams.on)
    ap.add_argument("--zoff", type=float, default=GeoParams.zoff)
    ap.add_argument("--cache", default=None, help="セル最小zのキャッシュ(.npz)。開発用")
    ap.add_argument("--dump-ground", default=None)
    ap.add_argument("--clear-waypoints", default="/home/kyo/rsf_ws/src/rsf_waypoint_manager/waypoints/tsukuba_wp.yaml",
                    help="経路 yaml。この折れ線近傍の低いブロックを除去。'none' で無効")
    ap.add_argument("--clear-zone", action="append", default=[],
                    help="x1,y1,x2,y2,radius,max_h (複数可)。線分から radius 以内で最大h<=max_h のセルを除去")
    ap.add_argument("--clear-zones-file", default=os.path.join(here, "clear_zones.yaml"),
                    help="ゾーン yaml (存在すれば読む。'none' で無効)")
    ap.add_argument("--box-overrides-file", default=os.path.join(here, "box_overrides.yaml"),
                    help="実寸直方体への置換 yaml (存在すれば読む。'none' で無効)")
    ap.add_argument("--probe", action="append", default=[], help="x,y の地形高さを表示 (複数可)")
    for k, v in Config().__dict__.items():
        ap.add_argument("--" + k.replace("_", "-"), type=type(v), default=v)
    a = ap.parse_args()
    cfg = Config(**{k: getattr(a, k) for k in Config().__dict__})
    g = GeoParams(oe=a.oe, on=a.on, zoff=a.zoff, model=a.geo_model, origin_lat=a.origin_lat,
                  origin_lon=a.origin_lon)

    # 点群ローカル範囲 (先頭ヘッダ後を memmap で走査)
    factory0, npts, mm = pcd_chunks_factory(a.pcd, Affine(np.eye(2), np.zeros(2)), 0.0, cfg.chunk, 1)
    xmn = xmx = ymn = ymx = None
    xs = np.asarray(mm["x"][::50]); ys = np.asarray(mm["y"][::50])
    xmn, xmx, ymn, ymx = float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())
    del mm
    aff = fit_affine(xmn - 20, xmx + 20, ymn - 20, ymx + 20, 10.0, g)
    print(f"local range x[{xmn:.1f},{xmx:.1f}] y[{ymn:.1f},{ymx:.1f}]  affine A={aff.A.tolist()} "
          f"t={aff.t.tolist()} max_residual={aff.max_residual:.2e} m")
    factory, npts, mm = pcd_chunks_factory(a.pcd, aff, g.zoff, cfg.chunk, a.stride)
    probes = [tuple(float(t) for t in s.split(",")) for s in a.probe]
    clear_path = None
    if a.clear_waypoints.lower() != "none":
        import yaml
        wp = yaml.safe_load(open(a.clear_waypoints))["waypoints"]
        clear_path = np.array([[70.571, 71.831]] + [[w["x"], w["y"]] for w in wp], float)
        print(f"clear path: {len(clear_path)} points, radius={cfg.clear_radius} max_h={cfg.clear_max_h}")
    zones = []
    if a.clear_zones_file.lower() != "none" and os.path.exists(a.clear_zones_file):
        zones += load_zones(a.clear_zones_file)
    zones += [parse_zone(z) for z in a.clear_zone]
    print(f"clear zones: {zones}")
    boxes = []
    if a.box_overrides_file.lower() != "none" and os.path.exists(a.box_overrides_file):
        boxes = load_boxes(a.box_overrides_file)
    print(f"box overrides: {boxes}")
    stats = run_pipeline(factory, a.out, cfg, probes=probes, cache=a.cache, dump_ground=a.dump_ground,
                         clear_path=clear_path, clear_zones=zones, box_overrides=boxes)
    print(stats)


if __name__ == "__main__":
    main()
