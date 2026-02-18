# hino_jt128 データでの MapCleaner 実行

## 入力データ

- **データ元**: GLIM による SLAM 出力
- **LiDAR**: Hesai JT128
- **フレーム数**: 4,626（`frames/00000000` ～ `frames/00004625`、`traj_lidar.txt` 行数に準拠）
- **1フレームあたりの点数**: 230,400 点（`points_compact.bin` = 2,764,800 bytes / 12 bytes/点）
- **データパス**: `/home/user/map/hino_jt128/`

| ファイル | 説明 |
|----------|------|
| `frames/NNNNNNNN/points_compact.bin` | 各フレームの点群（float32 × 3 = 12 bytes/点） |
| `frames/NNNNNNNN/intensities_compact.bin` | 各フレームの intensity |
| `traj_lidar.txt` | 軌跡（`timestamp tx ty tz qx qy qz qw` 形式、4,626行） |

---

## 初回実行: OOM クラッシュ

### 症状

- PatchWorkpp が全 4,626 フレームを処理完了後、`Finished: GroundSegmentation` 出力前にプロセスが kill された
- **終了コード**: -9（SIGKILL、カーネル OOM Killer による強制終了）

### 原因

`GroundSegmentation.hpp` の `compute()` が全フレームの点群を1つの `pcl::PointCloud` に集約する際、メモリ消費が爆発する。

```
全点数 = 230,400 点/フレーム × 4,626 フレーム ≈ 10.66 億点
```

| メモリ消費の内訳 | サイズ |
|-----------------|--------|
| `cloud.reserve()` (PointXYZI × 10.66億) | ~18.75 GB |
| `ground_indices` + `nonground_indices` (int × 10.66億 × 2) | ~4 GB |
| `shrink_to_fit()` による一時的なメモリ二重確保 | ピーク ~36 GB |
| **合計ピーク** | **~40 GB** |

`shrink_to_fit()` は内部で新しい連続メモリ領域を確保→コピー→旧領域を解放するため、一時的にクラウドのメモリが2倍必要になる。これが OOM の直接的原因である。

---

## 対策: 設定変更

`config/config_glim.yaml` の `ground_segmentation` セクションを変更した。

| パラメータ | 変更前 | 変更後 | 効果 |
|-----------|--------|--------|------|
| `use_voxel_grid` | `false` | `true` | フレームごとにボクセルグリッドフィルタで点数を削減 |
| `voxel_leaf_size` | `0.1` | `0.3` | ダウンサンプリングの解像度（大きいほど点数削減） |
| `frame_skip` | `0` | `3` | 3フレームごとにスキップ（処理フレーム数を1/4に削減） |

この設定変更によりメモリ使用量が大幅に削減され、OOM を回避できた。

---

## 2回目実行: 正常完了

全パイプラインが正常に完了した。

| ステップ | 所要時間 |
|---------|----------|
| GroundSegmentation (PatchWorkpp 4,626フレーム) | ~2分 |
| GridMapBuilder | ~5秒 |
| VarianceFilter | ~1秒 |
| 1st BGKFilter | ~10秒 |
| TrajectoryFilter | ~5秒 |
| 2nd BGKFilter | ~20秒 |
| MedianFilter | ~1秒 |
| DivideByTerrain | ~1秒 |
| MovingPointIdentification | ~30秒 |
| Save | ~10秒 |
| **合計** | **~4分** |

---

## 出力ファイル

保存先: `/home/user/map/hino_jt128_output/`

| ファイル | サイズ | 説明 |
|---------|--------|------|
| `static.pcd` | 187 MB | 静的物体の点群（建物、壁、植生等） |
| `ground.pcd` | 14 MB | 地面の点群 |
| `dynamic.pcd` | 11 MB | 動的物体の点群（車両、歩行者等） |
| `terrain.pcd` | 4.2 MB | 地形（BGKフィルタで推定した標高マップ） |
| `ground_below.pcd` | 1.9 MB | 地面より下の点群 |
| `other.pcd` | 180 B | その他（該当点なし） |
| **合計** | **217 MB** | |

---

## RViz2 設定修正

`rviz/rviz.rviz` が RViz1（ROS1）形式のままであったため、RViz2（ROS2）形式に書き換えた。

| 項目 | RViz1 (旧) | RViz2 (新) |
|------|-----------|-----------|
| パネルクラス | `rviz/Displays` | `rviz_common/Displays` |
| ディスプレイクラス | `rviz/PointCloud2` | `rviz_default_plugins/PointCloud2` |
| ツールクラス | `rviz/MoveCamera` | `rviz_default_plugins/MoveCamera` |
| ビュークラス | `rviz/Orbit` | `rviz_default_plugins/Orbit` |
| Topic 指定 | 文字列 (`/topic`) | QoS マップ (`Depth`, `Durability Policy`, `Value` 等) |

結果トピックには `Durability Policy: Transient Local` を設定し、RViz2 を後から起動しても latched メッセージを受信できるようにした。

---

## RViz での可視化に関する補足

### `/in_process` トピックの2回再生

`in_process_visualization: true` の場合、RViz 上で点群が2回再生されるように見える。これは以下の2ステップが同じ `/in_process` トピックに全フレームを順次 publish するためである。

| ステップ | 処理内容 | フレームループ |
|---------|----------|--------------|
| Step 1: GroundSegmentation | PatchWorkpp で地面分離 | **全フレーム再生（1回目）** |
| Step 2: GridMapBuilder | 標高マップ構築 | なし |
| Step 3: VarianceFilter | 分散フィルタ | なし |
| Step 4: 1st BGKFilter | BGK補間 | なし |
| Step 5: TrajectoryFilter | 軌跡上の法線フィルタ | なし |
| Step 6: 2nd BGKFilter | BGK補間（2回目） | なし |
| Step 7: MedianFilter | メディアンフィルタ | なし |
| Step 8: DivideByTerrain | 地形による分類 | なし |
| Step 9: MovingPointIdentification | 動的物体検出 | **全フレーム再生（2回目）** |

Step 2～8 は grid_map 上の演算のためフレーム再生は発生しない。

### `/grid_map` トピックの有効化

`/grid_map` トピック（`grid_map_msgs/GridMap`）は元コードでデフォルト無効であった。

- **原因**: `src/map_cleaner.cpp` で `#define PUBLISH_GRID_MAP` がコメントアウトされていた
- **修正**: コメントを外して有効化し、リビルド

```cpp
// 変更前
// #define PUBLISH_GRID_MAP

// 変更後
#define PUBLISH_GRID_MAP
```

有効化後、RViz2 の GridMap ディスプレイ（`grid_map_rviz_plugin/GridMap`、トピック `/grid_map`）で標高マップを確認できる。rviz.rviz にはデフォルト無効の状態で設定済みであるため、RViz2 上でチェックを入れるだけで表示される。

---

## 実行コマンド

```bash
cd /home/user/workspace_ros2/map_cleaner_ws
source install/setup.bash
ros2 launch map_cleaner run.launch.py config:=$(pwd)/src/MapCleaner_Unofficial/config/config_glim.yaml
```

別ターミナルで RViz2 を起動:

```bash
cd /home/user/workspace_ros2/map_cleaner_ws
source install/setup.bash
rviz2 -d src/MapCleaner_Unofficial/rviz/rviz.rviz
```

---

## 使用設定 (config_glim.yaml)

```yaml
/**:
  ros__parameters:
    loader:
      pcds_dir: "/home/user/map/hino_jt128/frames/"
      pose_file: "/home/user/map/hino_jt128/traj_lidar.txt"
      kitti_calibration_file: ""
      start: 0
      end: -1
      format: "glim"

    map_cleaner:
      save_dir: "/home/user/map/hino_jt128_output"
      frame_id: "map"
      in_process_visualization: true
      visualization_vg_res: 0.4

    ground_segmentation:
      use_voxel_grid: true
      voxel_leaf_size: 0.3
      frame_skip: 3

    patchworkpp:
      verbose: false
      sensor_height: 0.0
      th_seeds: 0.125
      th_dist: 0.125
      max_range: 80.0
      min_range: 0.3

    grid_map_builder:
      grid_res: 0.2

    variance_filter:
      variance_th: 0.01
      min_num: 1

    first_bgk:
      kernel_radius: 1.0
      sigma: 1.0
      min_num: 3

    trajectory_filter:
      normal_th_deg: 15.0

    second_bgk:
      kernel_radius: 2.0
      sigma: 0.5
      min_num: 3

    median:
      enable: true
      kernel_size: 3

    divide_by_terrain:
      threshold: 0.1
      on_terrain_only: false

    moving_point_identification:
      voxel_leaf_size: 0.2
      fov_h_deg: 360.0
      fov_v_deg: 90.0
      lidar_range: 50.0
      threshold: 0.1
      frame_skip: 0
      submap_update_dist: 5.0
```
