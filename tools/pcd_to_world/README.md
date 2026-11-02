# pcd_to_world

点群地図(PCD)から Gazebo 用モデル `models/tsukuba` を生成する。

## 再生成コマンド
```
cd /home/kyo/rsf_ws/src/rsf_simulator/tools/pcd_to_world
env -u PYTHONPATH /home/kyo/pcd2pgm_ws/venv/bin/python -m pytest -q tests
# 区間ゾーンは clear_zones.yaml から自動で読む(--clear-zone の重複指定は不要)
env -u PYTHONPATH /home/kyo/pcd2pgm_ws/venv/bin/python -u pcd_to_world.py \
  --probe 70.571,71.831 --probe 78.543,77.5 --probe 123.851,96.149
```
- 経路クリア: `--clear-waypoints`(既定 tsukuba_wp.yaml, `none` で無効), `--clear-radius 1.0`, `--clear-max-h 0.25`
- 区間ゾーン: `--clear-zone x1,y1,x2,y2,radius,max_h`(複数可、既定値には含めない)。
  `clear_zones.yaml` があれば自動で読む(`--clear-zones-file`、`none` で無効)。ゾーン追加はこの yaml に行を足す。
- 開発用: `--cache X.npz`(セル統計キャッシュ)
- 柱の置換: `box_overrides.yaml`(`--box-overrides-file`、`none` で無効)。指定 footprint+margin 内の h 範囲のボクセル/柱を除去し、実寸の向き付き直方体を追加する。
- 除去の判定は鉛直方向に連続するボクセルの run 単位(run の上端 h <= max_h なら除去。屋根・樹冠・柱は上端が高いので残る)。

## georeference(座標変換)
- 出典: https://github.com/raucha/tc-datasets/tree/add-map#tc2026-ise-robotics-gnss-map-data
  「The map is referenced to JGD2024, with its horizontal origin at latitude 36.082880°, longitude 140.076545°.
  Heights are orthometric heights in metres.」→ 点群の x,y は平面直角座標ではなく、この原点を中心とする局所 ENU(東・北)。
- 既定 `--geo-model enu_origin_spherical`:
  1. 点群 (x,y) → `+proj=ortho +lat_0=36.082880 +lon_0=140.076545 +ellps=GRS80` の逆変換 → 緯度経度
  2. → プロジェクトの球近似式 `e=radians(lon-lon0)*cos(radians(lat0))*6378137, n=radians(lat-lat0)*6378137`
     (lat0/lon0 は tsukuba_georef.yaml: 36.0834705, 140.0763034)
  3. → georef の enu_to_map (theta=-2.3500858, tx=156.896102, ty=64.643023) → map
  4. z は `map_z = z + zoff`(`--zoff -24.775`)。全点には 2D アフィン近似を使う(最大残差は fit 時にログ表示)。
- 旧方式は `--geo-model spherical|ellipsoid`(平面直角 EPSG:6677 + `--oe/--on` オフセット)で残してある。新方式では oe/on は使わない。

## collision を経路周辺に絞る(起動時の負荷削減)
- `--collision-route-radius 10.0`: 経路(`--clear-waypoints` の yaml + 先頭にスポーン)から 2D 柱の中心までの水平距離が
  この値以内の柱だけを `structures_collision.obj` に残す。`none` で全域。visual(`structures.obj`)は変わらない。
- `--terrain-collision-route-radius 20.0`: デシメート済み terrain の三角形のうち、重心が経路から半径以内のものだけを
  `meshes/terrain_collision.obj` に出力(同一三角形・再デシメートなし・未使用頂点は詰める・頂点法線つき)。`none` で全域。
  `model.sdf` は terrain の visual が `terrain.obj`、collision が `terrain_collision.obj` を参照する。
- `--clear-waypoints none` で経路を無効にした場合は、経路が無いため上記の絞り込みも行われない(全域)。
