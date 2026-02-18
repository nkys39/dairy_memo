# glim + glim_save_frame_plugin 詳細

## ビルド修正
- **livox_ros_driver2**: CMakeLists.txt 先頭に `ROS_EDITION`, `HUMBLE_ROS`, `CMAKE_BUILD_TYPE` 変数を追加（ROS2ビルド対応）
- **livox_ros_driver2**: `package_ROS2.xml` を削除（package.xml に統合）
- **glim_save_frame_plugin**: `modules/glim_save_frame_plugin/CMakeLists.txt` の `cmake_minimum_required` を `3.0.2` → `3.5` に変更（CMake 4.2 互換性修正）

## glim 設定変更（config_ros.json）
- `keep_raw_points`: `false` → `true`（save_frame_plugin が生点群データを保存するために必要）
- `extension_modules`: `libglim_save_frame_plugin.so` を追加
- トピック設定を Livox センサ用に変更
  - `imu_topic`: `/lidar_imu`
  - `points_topic`: `/lidar_points`

## glim パフォーマンスチューニング（GPU設定）
- **config_sub_mapping_gpu.json**:
  - `keyframe_randomsampling_rate`: `1.0` → `0.6`
  - `keyframe_voxel_resolution`: `0.25` → `0.5`
  - `keyframe_voxelmap_levels`: `2` → `1`
  - `submap_downsample_resolution`: `0.1` → `0.3`
- **config_global_mapping_gpu.json**:
  - `randomsampling_rate`: `1.0` → `0.6`
  - `submap_voxelmap_levels`: `2` → `1`

## rosbag 処理実行
- 対象: `~/rosbag/12_05_tsukuba_all/`（約125.7 GiB、約56分間）
- `ros2 run glim_ros glim_rosbag` で処理
- 結果: 33,883 フレーム（約 11 GB）が `/tmp/dump/frames/` に保存
  - 各フレーム: `points_compact.bin`（Vector3f）+ `intensities_compact.bin`（float）
  - rosbag の点群メッセージ数 33,894 に対し 33,883 フレーム保存（差分 11 は初期化で消費）
