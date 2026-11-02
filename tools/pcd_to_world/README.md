# pcd_to_world

点群地図(PCD)から Gazebo 用モデル `models/tsukuba` を生成する。

## 再生成コマンド
```
cd /home/kyo/rsf_ws/src/rsf_simulator/tools/pcd_to_world
env -u PYTHONPATH /home/kyo/pcd2pgm_ws/venv/bin/python -m pytest -q tests
# 区間ゾーンは clear_zones.yaml から自動で読む(--clear-zone の重複指定は不要)
env -u PYTHONPATH /home/kyo/pcd2pgm_ws/venv/bin/python -u pcd_to_world.py \
  --probe 24.808,3.303
```
- 経路クリア: `--clear-waypoints`(既定 tsukuba_wp.yaml, `none` で無効), `--clear-radius 1.0`, `--clear-max-h 0.25`
- 区間ゾーン: `--clear-zone x1,y1,x2,y2,radius,max_h`(複数可、既定値には含めない)。
  `clear_zones.yaml` があれば自動で読む(`--clear-zones-file`、`none` で無効)。ゾーン追加はこの yaml に行を足す。
- 開発用: `--cache X.npz`(セル統計キャッシュ)
- 柱の置換: `box_overrides.yaml`(`--box-overrides-file`、`none` で無効)。指定 footprint+margin 内の h 範囲のボクセル/柱を除去し、実寸の向き付き直方体を追加する。
- 除去の判定は鉛直方向に連続するボクセルの run 単位(run の上端 h <= max_h なら除去。屋根・樹冠・柱は上端が高いので残る)。

## 座標系
- **点群座標 = map 座標(恒等)**。点群の x,y は公式原点(36.082880°N, 140.076545°E, JGD2024)中心の局所 ENU、z は標高で、
  座標変換は行わない(ツールは点群の x,y,z をそのままワールドに書く)。
  出典: https://github.com/raucha/tc-datasets/tree/add-map#tc2026-ise-robotics-gnss-map-data
- georef(GNSS との関係)は `rsf_navigation_executor/config/tsukuba_georef.yaml`(map_origin)。

## 経路沿い地形の平滑化
- 新 waypoint 折れ線から `--route-smooth-radius`(既定 3.0m、`none` で無効)以内の地面グリッドを平滑化し、外側
  `--route-smooth-blend`(既定 2.0m)は元の高さへ線形ブレンドする。メディアン(`--route-smooth-median` 既定 2.0m、外れ値除去)
  → ガウシアン(`--route-smooth-sigma` 既定 2.0m)。平面成分(傾斜)は保たれる。
  平滑化した地面から terrain メッシュ(visual/collision)を作り、構造物の h も平滑化後の地面基準。

## collision を経路周辺に絞る(起動時の負荷削減)
- `--collision-route-radius 10.0`: 経路(`--clear-waypoints` の yaml + 先頭にスポーン)から 2D 柱の中心までの水平距離が
  この値以内の柱だけを `structures_collision.obj` に残す。`none` で全域。visual(`structures.obj`)は変わらない。
- `--terrain-collision-route-radius 20.0`: デシメート済み terrain の三角形のうち、重心が経路から半径以内のものだけを
  `meshes/terrain_collision.obj` に出力(同一三角形・再デシメートなし・未使用頂点は詰める・頂点法線つき)。`none` で全域。
  `model.sdf` は terrain の visual が `terrain.obj`、collision が `terrain_collision.obj` を参照する。
- `--clear-waypoints none` で経路を無効にした場合は、経路が無いため上記の絞り込みも行われない(全域)。
