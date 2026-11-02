#!/usr/bin/env python3
"""点群地図(PCD) -> Gazebo 用静的モデル(tsukuba) 生成ツール。

出力: <out>/model.config, model.sdf, meshes/{terrain,structures,structures_collision}.obj
座標は map(=Gazebo world) 座標系でそのまま書き出す(モデル姿勢は原点)。
map = 点群座標(公式原点中心の局所 ENU、z は標高)そのもの(恒等)。georef は
rsf_navigation_executor/config/tsukuba_georef.yaml。

実行例は README 相当として末尾の main() / --help を参照。
"""
import argparse
import os
import resource
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

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
    pole_route_radius: Optional[float] = 5.0   # 経路から何 m 以内の細い柱状物を実寸直方体に置換するか(None=しない)
    pole_max_size: float = 0.6                 # 柱状物とみなす水平長軸の上限[m]
    pole_min_size: float = 0.05                # 直方体の最小サイズ[m]
    pole_hmin: float = 0.2                     # 検出に使う点の h 下限[m]
    pole_hmax: float = 1.5                     # 水平連結成分に使う h 上限[m]
    pole_min_points: int = 100                 # 成分の最小点数
    pole_cont_frac: float = 0.7                # h 0.2〜1.0 の 0.1m ビン(各 3 点以上)のうち点のある割合の下限
    pole_min_top: float = 0.6                  # 上端 h の下限[m](実データの歩行者ポールは高さ約 0.9m)
    pole_margin: float = 0.05                  # 置換時のボクセル除去マージン[m](+ボクセル対角の半分)
    route_smooth_radius: Optional[float] = 3.0   # 経路から何 m 以内の地面グリッドを平滑化するか(None=しない)
    route_smooth_blend: float = 2.0              # 外側へ元の高さに線形ブレンドする幅[m]
    route_smooth_median: float = 2.0             # 外れ値除去のメディアン窓[m] (1.5 では経路上の p99 残差が 3cm 残るため 2.0)
    route_smooth_sigma: float = 2.0              # ガウシアン σ[m] (1.0 では 3cm 残るため 2.0)
    collision_route_radius: Optional[float] = 10.0          # structures_collision を経路から何 m 以内の柱に絞るか(None=全域)
    terrain_collision_route_radius: Optional[float] = 20.0  # terrain_collision.obj を経路から何 m 以内の三角形に絞るか(None=全域)
    clear_radius: float = 1.0      # 経路折れ線からの水平距離[m]
    clear_max_h: float = 0.25      # セルの構造物最大hがこれ以下なら除去
    clear_run_bottom: float = 0.25 # 地面に接する run とみなす下端 h の上限[m]
    clear_tol: float = 0.02        # 最上ボクセル中心の h 判定の許容(地面ノイズ吸収)[m]
    min_count_coarse: int = 10     # 0.15m ボクセル時のノイズ除去しきい値(三角形数を4M以内にするため)
    hcol_min: float = 0.15
    col_bottom: float = 0.05       # collision 柱の下端(地形より下)
    vz_col: float = 0.1            # collision 柱の高さ量子化
    hmin: float = 0.10
    hmax: float = 15.0
    hcol_max: float = 1.2
    voxel: float = 0.1
    min_count: int = 3
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


def polyline_distance(P, poly):
    """点 P(N,2) から折れ線 poly(M,2) までの水平距離。"""
    P = np.asarray(P, np.float64)
    poly = np.asarray(poly, np.float64)
    d = np.full(len(P), np.inf)
    segs = list(zip(poly[:-1], poly[1:])) if len(poly) > 1 else [(poly[0], poly[0])]
    for a, b in segs:
        v = b - a
        L2 = float(v @ v)
        t = np.zeros(len(P)) if L2 < 1e-12 else np.clip(((P - a) @ v) / L2, 0, 1)
        d = np.minimum(d, np.hypot(P[:, 0] - (a[0] + t * v[0]), P[:, 1] - (a[1] + t * v[1])))
    return d


def near_route(xy, route, radius, origin, shape, cell):
    """xy(N,2) の各点が経路から radius 以内か。セルマスクで粗く絞ってから厳密距離。"""
    ny, nx = shape
    pm = path_cell_mask(route, origin, shape, cell, radius + 2 * cell)
    ix = np.clip(np.floor((xy[:, 0] - origin[0]) / cell).astype(np.int64), 0, nx - 1)
    iy = np.clip(np.floor((xy[:, 1] - origin[1]) / cell).astype(np.int64), 0, ny - 1)
    idx = np.flatnonzero(pm[iy, ix])
    out = np.zeros(len(xy), bool)
    if len(idx):
        out[idx] = polyline_distance(xy[idx], route) <= radius
    return out


def smooth_route_ground(ground, origin, cell, route, radius, blend, median, sigma):
    """経路から radius 以内の地面グリッドを平滑化し、外側 blend[m] は元の高さへ線形にブレンドする。
    外れ値(植生・物体の誤認)をメディアンで除いてからガウシアンで滑らかな面にする(平面成分=傾斜は保たれる)。"""
    if radius is None or route is None:
        return ground
    from scipy import ndimage as ndi
    ny, nx = ground.shape
    outer = radius + blend
    pm = path_cell_mask(route, origin, (ny, nx), cell, outer + cell)
    if not pm.any():
        return ground
    k = int(median / cell)
    k = max(k, 1) | 1
    sg = sigma / cell
    marg = k // 2 + int(np.ceil(4 * sg)) + 2
    out = ground.copy()
    B = 128
    for by in range(0, ny, B):
        for bx in range(0, nx, B):
            blk = pm[by:by + B, bx:bx + B]
            if not blk.any():
                continue
            y0, y1 = max(by - marg, 0), min(by + B + marg, ny)
            x0, x1 = max(bx - marg, 0), min(bx + B + marg, nx)
            crop = ground[y0:y1, x0:x1].astype(np.float64)
            sm = ndi.gaussian_filter(ndi.median_filter(crop, size=k, mode="nearest"), sg, mode="nearest")
            sl = (slice(by - y0, by - y0 + blk.shape[0]), slice(bx - x0, bx - x0 + blk.shape[1]))
            cy, cx = np.nonzero(blk)
            gy, gx = cy + by, cx + bx
            xy = np.c_[origin[0] + (gx + 0.5) * cell, origin[1] + (gy + 0.5) * cell]
            d = polyline_distance(xy, route)
            w = np.clip((outer - d) / blend, 0.0, 1.0) if blend > 0 else (d <= outer).astype(float)
            base = ground[gy, gx].astype(np.float64)
            val = sm[sl][cy, cx]
            new = np.where(w > 0, base + w * (val - base), base)
            out[gy, gx] = new.astype(ground.dtype)
    return out


def detect_poles(factory, gr, origin, shape, cfg, route_path, log=print):
    """経路から pole_route_radius 以内の細い柱状物(水平長軸 <= pole_max_size、鉛直に連続)を点群から検出し、
    PCA と 2〜98 パーセンタイル幅で実寸の向き付き直方体(box_overrides 形式の dict)を返す。
    樹冠・壁・横に広がる頭部を持つものは対象外(迷うものは除外)。"""
    from scipy import ndimage as ndi
    ny, nx = shape
    cell = cfg.cell
    pm = path_cell_mask(route_path, origin, shape, cell, cfg.pole_route_radius)
    chunks = []
    for pts in factory():
        ix = np.floor((pts[:, 0] - origin[0]) / cell).astype(np.int64)
        iy = np.floor((pts[:, 1] - origin[1]) / cell).astype(np.int64)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        pts, ix, iy = pts[ok], ix[ok], iy[ok]
        ok = pm[iy, ix] & gr.valid[iy, ix]
        pts, ix, iy = pts[ok], ix[ok], iy[ok]
        h = pts[:, 2] - gr.ground[iy, ix]
        k = (h >= cfg.pole_hmin) & (h <= cfg.hmax)
        if k.any():
            chunks.append(np.c_[pts[k, 0], pts[k, 1], h[k]].astype(np.float32))
    if not chunks:
        return []
    P = np.concatenate(chunks)
    del chunks
    lowm = P[:, 2] <= cfg.pole_hmax
    Q = P[lowm]
    g = 0.05
    x0, y0 = float(Q[:, 0].min()) - 0.2, float(Q[:, 1].min()) - 0.2
    gi = np.floor((Q[:, 0] - x0) / g).astype(np.int64)
    gj = np.floor((Q[:, 1] - y0) / g).astype(np.int64)
    occ = np.zeros((gj.max() + 3, gi.max() + 3), bool)
    occ[gj + 1, gi + 1] = True
    occ = ndi.binary_dilation(occ, structure=np.ones((3, 3), bool))
    lab, nl = ndi.label(occ, structure=np.ones((3, 3), bool))
    del occ
    ql = lab[gj + 1, gi + 1]
    del lab
    order = np.argsort(ql, kind="stable")
    ql = ql[order]
    Q = Q[order]
    starts = np.flatnonzero(np.r_[True, ql[1:] != ql[:-1]])
    ends = np.r_[starts[1:], len(ql)]
    Pord = np.argsort(P[:, 0], kind="stable")
    Px = P[Pord, 0]
    boxes = []
    rej = {}

    def reject(k):
        rej[k] = rej.get(k, 0) + 1
    for a, b in zip(starts, ends):
        if b - a < cfg.pole_min_points:
            reject('few_points')
            continue
        c = Q[a:b]
        xy = c[:, :2].astype(np.float64)
        if np.ptp(xy[:, 0]) > cfg.pole_max_size * 1.6 + 0.2 or np.ptp(xy[:, 1]) > cfg.pole_max_size * 1.6 + 0.2:
            reject('bbox')
            continue
        mu = xy.mean(axis=0)
        ev, evec = np.linalg.eigh(np.cov((xy - mu).T))
        ax = evec[:, 1]
        u = (xy - mu) @ ax
        w = (xy - mu) @ evec[:, 0]
        ul, uh = np.percentile(u, [2, 98])
        wl, wh = np.percentile(w, [2, 98])
        L_, W_ = uh - ul, wh - wl
        if L_ > cfg.pole_max_size:
            reject('long_axis')
            continue
        hh = c[:, 2]
        if hh.max() < cfg.pole_min_top:
            reject('low')
            continue
        cnt, _ = np.histogram(hh, bins=np.arange(cfg.pole_hmin, 1.0 + 1e-9, 0.1))
        if (cnt >= 3).mean() < cfg.pole_cont_frac:
            reject('not_continuous')
            continue
        cen = mu + ax * (ul + uh) / 2 + evec[:, 0] * (wl + wh) / 2
        yaw = float(np.degrees(np.arctan2(ax[1], ax[0])))
        sx, sy = max(L_, cfg.pole_min_size), max(W_, cfg.pole_min_size)
        # 真上で連続する上端と、樹冠・壁・頭部の有無(上部で横に広がるものは対象外)
        r = max(sx, sy) / 2 + 0.6
        i0, i1 = np.searchsorted(Px, [cen[0] - r, cen[0] + r])
        nb = P[Pord[i0:i1]]
        nb = nb[(np.abs(nb[:, 1] - cen[1]) <= r)]
        cth, sth = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        du, dv = nb[:, 0] - cen[0], nb[:, 1] - cen[1]
        lu, lv = cth * du + sth * dv, -sth * du + cth * dv
        inside = (np.abs(lu) <= sx / 2 + 0.05) & (np.abs(lv) <= sy / 2 + 0.05)
        ring = (np.abs(lu) <= sx / 2 + 0.4) & (np.abs(lv) <= sy / 2 + 0.4) & ~inside
        hin = np.sort(nb[inside, 2])
        bins = np.floor((hin - cfg.pole_hmin) / 0.1).astype(np.int64)
        occb = np.unique(bins)
        gaps = np.flatnonzero(np.diff(occb) > 2)
        last = occb[gaps[0]] if len(gaps) else occb[-1]
        top_pts = hin[bins <= last]
        top = float(np.percentile(top_pts, 99.5))
        if top < cfg.pole_min_top:
            reject('top')
            continue
        hr = nb[ring, 2]
        if ((hr > cfg.pole_hmax) & (hr <= top)).sum() > 0.5 * ((top_pts > cfg.pole_hmax).sum()) + 50:
            reject('widening')
            continue
        boxes.append(dict(center_x=float(cen[0]), center_y=float(cen[1]), yaw_deg=yaw, size_x=float(sx),
                          size_y=float(sy), z_bottom_h=0.0, z_top_h=top, margin=cfg.pole_margin, auto=True))
    log(f"poles: {len(boxes)} detected within {cfg.pole_route_radius}m of route; rejected {rej}")
    return boxes


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


def decimate_mesh(verts, faces, target):
    """open3d の quadric decimation で target 三角形以下へ間引く(地形用。開境界が保たれる)。"""
    if len(faces) <= target:
        return np.asarray(verts), np.asarray(faces)
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
    if len(ijk) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
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
            uk, inv, cnts = np.unique(key, return_inverse=True, return_counts=True)
            ordr = np.argsort(-cnts, kind="stable")        # 頻出の法線を小さい番号にして f 行を短くする
            rank = np.empty(len(uk), np.int64)
            rank[ordr] = np.arange(len(uk))
            uk, inv = uk[ordr], rank[inv]
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


def run_pipeline(factory, outdir, cfg=None, log=print, probes=(), cache=None, dump_ground=None, clear_path=None, clear_zones=(), box_overrides=(), route_path=None):
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
    if route_path is not None and cfg.route_smooth_radius is not None:
        g0 = gr.ground
        gr.ground = smooth_route_ground(g0, origin, cfg.cell, route_path, cfg.route_smooth_radius,
                                        cfg.route_smooth_blend, cfg.route_smooth_median, cfg.route_smooth_sigma)
        L(f"route smoothing: radius {cfg.route_smooth_radius}m blend {cfg.route_smooth_blend}m, max change {float(np.abs(gr.ground - g0).max()):.3f}m")
        del g0
    L(f"ground cells={int(gr.is_ground.sum())} valid={int(gr.valid.sum())}")
    stats["ground_cells"] = int(gr.is_ground.sum())
    if dump_ground:
        np.savez(dump_ground, ground=gr.ground, is_ground=gr.is_ground, valid=gr.valid, origin=np.array(origin))
    del cellmin

    # 地形メッシュ
    verts, faces = terrain_mesh(gr, origin, cfg.cell)
    L(f"terrain full: verts={len(verts)} tris={len(faces)}")
    dv, df = decimate_mesh(verts, faces, cfg.terrain_tris)
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
    if route_path is not None:
        nr = near_route(verts[:, :2], route_path, cfg.route_smooth_radius or 3.0, origin, shape, cfg.cell)
        ef = err[nr & fin]
        if len(ef):
            stats["terrain_route_err_max"] = float(ef.max())
            stats["terrain_route_err_p99"] = float(np.percentile(ef, 99))
            L(f"terrain vertical err within route: max={ef.max():.4f} p99={np.percentile(ef, 99):.4f} n={len(ef)}")
    mdir = os.path.join(outdir, "meshes")
    write_obj(os.path.join(mdir, "terrain.obj"), dv, df)
    # terrain collision: デシメート済み三角形のうち、重心が経路から半径以内のものだけ(同一三角形、再デシメートなし)
    if route_path is not None and cfg.terrain_collision_route_radius is not None:
        cen = dv[df].mean(axis=1)[:, :2]
        tsel = polyline_distance(cen, route_path) <= cfg.terrain_collision_route_radius
        cf = df[tsel]
        used = np.unique(cf)
        remap = np.full(len(dv), -1, np.int64)
        remap[used] = np.arange(len(used))
        tcv, tcf = dv[used], remap[cf]
    else:
        tcv, tcf = dv, df
    stats["terrain_collision_tris"] = len(tcf)
    L(f"terrain_collision tris={len(tcf)} (terrain {len(df)})")
    write_obj(os.path.join(mdir, "terrain_collision.obj"), tcv, tcf)
    del tcv, tcf
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
    regions = []   # (polyline, radius, max_h): 最終ボクセル/柱で判定する除去領域
    if clear_path is not None:
        regions.append((np.asarray(clear_path, float), cfg.clear_radius, cfg.clear_max_h))
    for (x1, y1, x2, y2, rad, mh) in clear_zones:
        regions.append((np.array([[x1, y1], [x2, y2]]), rad, mh))
    stats["poles_replaced"] = 0
    if route_path is not None and cfg.pole_route_radius is not None:
        auto = detect_poles(factory, gr, origin, shape, cfg, route_path, log=L)
        auto = [b for b in auto
                if not any(in_box_footprint(np.array([b["center_x"]]), np.array([b["center_y"]]), m, m["margin"])[0]
                           for m in box_overrides)]
        stats["poles_replaced"] = len(auto)
        stats["pole_boxes"] = auto
        if auto:
            sz = np.array([[b["size_x"], b["size_y"], b["z_top_h"]] for b in auto])
            L(f"poles replaced={len(auto)} (manual overrides take priority); size_x p50={np.median(sz[:,0]):.2f} size_y p50={np.median(sz[:,1]):.2f} top p50={np.median(sz[:,2]):.2f}")
        box_overrides = list(box_overrides) + auto
    fine_sizes = [cfg.voxel, cfg.voxel_coarse]
    fine_idx = 0
    mdir = os.path.join(outdir, "meshes")

    def accumulate(vfine):
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
        return acc, Df, nst

    # --- visual (0.1m ボクセルの greedy meshing。三角形数が多すぎれば 0.15m へ)
    while True:
        vfine = fine_sizes[fine_idx]
        acc, Df, nstruct = accumulate(vfine)
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
    vis_ijk = ijk.copy()   # 最終 visual ボクセル(ノイズ除去・クリア・box 置換 適用後)。collision はここから導出する
    stats["structure_points"] = nstruct
    stats["structures_voxel"] = vfine
    stats["structures_voxels"] = len(ijk)
    stats["structures_tris"] = len(sf)
    sv, sf = add_override_boxes(sv, sf, box_overrides, gr, origin, cfg, collision=False)
    stats["structures_tris"] = len(sf)
    write_obj(os.path.join(mdir, "structures.obj"), sv, sf, normals="face6")
    stats["structures_vol"] = fix_winding_report(sv, sf) if len(sf) else 0.0
    del sv, sf, ijk

    # --- collision: 最終 visual ボクセルから導出(不変条件: collision ⊂ visual)。
    # h∈[hcol_min, hcol_max] の visual ボクセルを 0.2m セル x 0.1m 高さビンに割り当て(ボクセルの全高をカバー)、
    # セルごとに鉛直 run 化して、地面に接する run は地形-0.05m から上端まで柱化する。
    ijk = np.zeros((0, 3), np.int64)
    nruns = 0
    if len(vis_ijk):
        xv = origin[0] + (vis_ijk[:, 0] + 0.5) * vfine
        yv = origin[1] + (vis_ijk[:, 1] + 0.5) * vfine
        ci = np.clip(np.floor((xv - origin[0]) / cfg.cell).astype(np.int64), 0, nx - 1)
        cj = np.clip(np.floor((yv - origin[1]) / cfg.cell).astype(np.int64), 0, ny - 1)
        gvc = gr.ground[cj, ci]
        zc = zref + (vis_ijk[:, 2] + 0.5) * vfine
        hc = zc - gvc
        m = (hc >= cfg.hcol_min) & (hc <= cfg.hcol_max)
        ci, cj, zc, gvc = ci[m], cj[m], zc[m], gvc[m]
        if len(ci):
            k0 = np.floor((zc - vfine / 2 - zref) / vz + 1e-6).astype(np.int64)
            k1 = np.floor((zc + vfine / 2 - zref) / vz - 1e-6).astype(np.int64)
            ijk_b = columns_to_voxels(np.c_[ci, cj], k0, k1 + 1)
            rep = np.repeat(np.arange(len(ci)), np.maximum(k1 + 1 - k0, 0))
            ii, jj, kk, gv = ijk_b[:, 0], ijk_b[:, 1], ijk_b[:, 2], gvc[rep]
            hv = zref + (kk + 0.5) * vz - gv
            kp = hv <= cfg.hcol_max
            ii, jj, kk, gv = ii[kp], jj[kp], kk[kp], gv[kp]
            u = np.unique(np.c_[jj * nx + ii, kk], axis=0, return_index=True)[1]
            ii, jj, kk, gv = ii[u], jj[u], kk[u], gv[u]
    if len(vis_ijk) and len(ci) and len(ii):
        run_id, first, ks, ke = vertical_runs(jj * nx + ii, kk)
        cxr, cyr, g_r = ii[first], jj[first], gv[first]
        hbot = zref + ks * vz - g_r
        grounded = hbot <= cfg.clear_run_bottom
        kbot = np.where(grounded, np.floor((g_r - cfg.col_bottom - zref) / vz).astype(np.int64), ks)
        ktop = np.maximum(ke + 1, kbot + 1)
        sel = np.ones(len(first), bool)
        if route_path is not None and cfg.collision_route_radius is not None:
            xr = origin[0] + (cxr + 0.5) * cfg.cell
            yr = origin[1] + (cyr + 0.5) * cfg.cell
            near = polyline_distance(np.c_[xr, yr], route_path) <= cfg.collision_route_radius
            L(f"col: route radius {cfg.collision_route_radius}m keeps {int((sel & near).sum())} of {int(sel.sum())} runs")
            sel &= near
        ijk = np.unique(columns_to_voxels(np.c_[cxr, cyr][sel], kbot[sel], ktop[sel]), axis=0)
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
    cboxes = box_overrides
    if route_path is not None and cfg.collision_route_radius is not None:
        cboxes = [b for b in box_overrides
                  if polyline_distance(np.array([[b["center_x"], b["center_y"]]]), route_path)[0] <= cfg.collision_route_radius]
    sv, sf = add_override_boxes(sv, sf, cboxes, gr, origin, cfg, collision=True)
    stats["structures_collision_tris"] = len(sf)
    write_obj(os.path.join(mdir, "structures_collision.obj"), sv, sf, normals="face6")
    stats["structures_collision_vol"] = fix_winding_report(sv, sf) if len(sf) else 0.0
    del sv, sf, ijk, vis_ijk
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
        <geometry><mesh><uri>model://tsukuba/meshes/terrain_collision.obj</uri></mesh></geometry>
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
def pcd_chunks_factory(path, chunk, stride=1):
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
            yield np.c_[x[ok], y[ok], z[ok]].astype(np.float32)
    return factory, npts, mm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--pcd", default="/home/kyo/Downloads/tsukuba_map/cleaned_dense_2p5cm_tsukuba_jgd2024_intensity.pcd")
    ap.add_argument("--out", default=os.path.normpath(os.path.join(here, "..", "..", "models", "tsukuba")))
    ap.add_argument("--stride", type=int, default=1, help="点の間引き(開発用)")
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
    NONE_OK = ("collision_route_radius", "terrain_collision_route_radius", "route_smooth_radius", "pole_route_radius")

    def float_or_none(t):
        return None if str(t).lower() == "none" else float(t)
    for k, v in Config().__dict__.items():
        ap.add_argument("--" + k.replace("_", "-"), type=float_or_none if k in NONE_OK else type(v),
                        default=v, help="none で無効" if k in NONE_OK else None)
    a = ap.parse_args()
    cfg = Config(**{k: getattr(a, k) for k in Config().__dict__})
    factory, npts, mm = pcd_chunks_factory(a.pcd, cfg.chunk, a.stride)
    probes = [tuple(float(t) for t in s.split(",")) for s in a.probe]
    clear_path = None
    if a.clear_waypoints.lower() != "none":
        import yaml
        wp = yaml.safe_load(open(a.clear_waypoints))["waypoints"]
        clear_path = np.array([[w["x"], w["y"]] for w in wp], float)   # 先頭点 = スポーン(新しい始点)
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
                         clear_path=clear_path, clear_zones=zones, box_overrides=boxes, route_path=clear_path)
    print(stats)


if __name__ == "__main__":
    main()
