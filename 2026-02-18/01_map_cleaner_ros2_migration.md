# MapCleaner ROS1 → ROS2 Jazzy 移行作業ログ

## 概要

`MapCleaner_Unofficial` を ROS1 (Catkin / roscpp) から ROS2 Jazzy (ament_cmake / rclcpp) で動作するように移行した。

- ワークスペース: `/home/user/workspace_ros2/map_cleaner_ws/src/MapCleaner_Unofficial/`
- ビルド結果: 成功 (warning のみ、error なし)

---

## 設計方針

| 項目 | 方針 |
|------|------|
| アーキテクチャ | `MapCleaner` クラスはノードを継承せず `rclcpp::Node::SharedPtr` をコンストラクタで受け取る |
| ロギング | ノードを持たないフィルタクラスは `rclcpp::get_logger("ClassName")` を使用 |
| tf2 Broadcaster | ROS2 では `StaticTransformBroadcaster` のコンストラクタにノードが必要なため `shared_ptr` で保持 |
| パラメータ | `nh_.param<T>("/ns/key", var, default)` → `node_->declare_parameter<T>("ns.key", default)` |
| Latched Publisher | `advertise(..., 1, true)` → `rclcpp::QoS(1).transient_local()` |
| float パラメータ | ROS2 では `double` で declare して `static_cast<float>` |
| vector\<int\> パラメータ | ROS2 では `vector<int64_t>` で declare して変換 |

---

## 変更ファイル一覧

### ビルドシステム

| ファイル | 変更内容 |
|----------|----------|
| `package.xml` | format 2→3, catkin→ament_cmake, roscpp→rclcpp, pcl_ros→pcl_conversions, `<build_type>ament_cmake</build_type>` 追加 |
| `CMakeLists.txt` | catkin→ament_cmake 全面書き換え。`find_package` 個別化、`ament_target_dependencies`、`install()` ルール追加 |
| `thirdparty/mod_pcl/CMakeLists.txt` | `cmake_minimum_required` を 3.8 に更新 |

### ヘッダファイル

| ファイル | 変更内容 |
|----------|----------|
| `include/map_cleaner/utils.hpp` | `PublisherPtr` 型を `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr` に変更 |
| `include/map_cleaner/GridMapBuilder.hpp` | include 変更のみ |
| `include/map_cleaner/VarianceFilter.hpp` | include + `RCLCPP_ERROR_STREAM` に変更 |
| `include/map_cleaner/BGKFilter.hpp` | 同上 |
| `include/map_cleaner/TrajectoryFilter.hpp` | 同上 |
| `include/map_cleaner/MedianFilter.hpp` | 同上 |
| `include/map_cleaner/DivideByTerrain.hpp` | 同上 |
| `include/map_cleaner/DataLoader.hpp` | `RCLCPP_INFO_STREAM` に変更 |
| `include/map_cleaner/GroundSegmentation.hpp` | 全面ROS2化 (node_, tf2 broadcaster, メッセージ型, タイムスタンプ, ロギング) |
| `include/map_cleaner/MovingPointIdentification.hpp` | 同上 |

### ソースファイル

| ファイル | 変更内容 |
|----------|----------|
| `src/map_cleaner.cpp` | 全面ROS2化。NodeHandle→Node::SharedPtr、Publisher、パラメータ(~40箇所)、spinループ、main関数 |

### サードパーティ

| ファイル | 変更内容 |
|----------|----------|
| `thirdparty/mod_pcl/include/voxel_grid_large.h` | `#include <pcl/filters/boost.h>` → `#include <cfloat>` (PCL 1.14 対応) |
| `thirdparty/mod_pcl/include/impl/voxel_grid_large.hpp` | `#include <pcl/common/impl/common.hpp>` と `#include <pcl/filters/impl/voxel_grid.hpp>` 追加 (リンクエラー修正) |

### 設定ファイル

| ファイル | 変更内容 |
|----------|----------|
| `config/config.yaml` | `/**:` > `ros__parameters:` ラッパー追加 |
| `config/config_kitti.yaml` | 同上 |
| `config/config_glim.yaml` | 同上 + パスを hino_jt128 データに更新 |

### ランチファイル

| ファイル | 変更内容 |
|----------|----------|
| `launch/run.launch.py` | 新規作成 (Python形式)。config 引数で設定ファイル指定可能 |

---

## ビルド中に遭遇した問題と修正

### 1. grid_map パッケージ未インストール

- **症状**: `find_package(grid_map_core)` が失敗
- **修正**: `sudo apt install ros-jazzy-grid-map-*` で手動インストール

### 2. grid_map_core の Eigen プラグイン定義がサードパーティに波及

- **症状**: `grid_map_core/eigen_plugins/FunctorsPlugin.hpp` not found (patchworkpp, voxel_grid_large のコンパイル時)
- **原因**: `grid_map_core-extras.cmake` が `add_definitions(-DEIGEN_FUNCTORS_PLUGIN=...)` をグローバルに追加。これにより全ターゲットで Eigen ヘッダ解決時に grid_map_core のインクルードパスが必要になる
- **試行1 (失敗)**: CMakeLists.txt でサードパーティを `find_package(grid_map_core)` より前にビルド → CMake のグローバルフラグはビルド時に適用されるため順序に関係なく影響
- **試行2 (失敗)**: `${grid_map_core_INCLUDE_DIRS}` をサードパーティに追加 → モダンCMakeターゲットのため変数が空
- **修正**: `get_target_property(_grid_map_core_incs grid_map_core::grid_map_core INTERFACE_INCLUDE_DIRECTORIES)` でインクルードパスを取得し、patchworkpp と voxel_grid_large の `target_include_directories` に追加

### 3. PCL 1.14 での `pcl/filters/boost.h` 廃止

- **症状**: `FLT_MAX` undeclared (voxel_grid_large.h)
- **原因**: PCL 1.14 (Ubuntu 24.04 / Jazzy) で `pcl/filters/boost.h` が削除されている
- **修正**: `#include <pcl/filters/boost.h>` → `#include <cfloat>`

### 4. `pcl::getMinMax3D` リンクエラー

- **症状**: `undefined reference to void pcl::getMinMax3D<pcl::PointXYZI>(...)` 等、多数の点型で発生
- **原因**: `voxel_grid_large` テンプレートが呼ぶ `getMinMax3D` のフィルタ付きオーバーロードは `pcl/filters/voxel_grid.h` に宣言されているが、テンプレート実装ヘッダ (`pcl/filters/impl/voxel_grid.hpp`) がインクルードされていなかった。PCL のプリコンパイル済みインスタンスではこのオーバーロードが提供されないため、リンク時に未解決シンボルとなる
- **修正**: `impl/voxel_grid_large.hpp` に `#include <pcl/filters/impl/voxel_grid.hpp>` と `#include <pcl/common/impl/common.hpp>` を追加

---

## 実行方法

```bash
cd /home/user/workspace_ros2/map_cleaner_ws
source install/setup.bash

# デフォルト設定で実行
ros2 launch map_cleaner run.launch.py

# GLIM (hino_jt128) データで実行
ros2 launch map_cleaner run.launch.py config:=$(pwd)/src/MapCleaner_Unofficial/config/config_glim.yaml
```

### hino_jt128 データ設定 (config_glim.yaml)

```yaml
loader:
  pcds_dir: "/home/user/map/hino_jt128/frames/"
  pose_file: "/home/user/map/hino_jt128/traj_lidar.txt"
  format: "glim"

map_cleaner:
  save_dir: "/home/user/map/hino_jt128_output"
```

- フレーム数: 4626 (frames/00000000 ~ frames/00004625)
- 出力先: `/home/user/map/hino_jt128_output/`

---

## 残課題

- ランタイム動作確認（実データでの実行テスト未実施）
- `publishGridMap` 関数の `GridMapRosConverter::toMessage` API がROS2で正しいか検証 (`#ifdef PUBLISH_GRID_MAP` で囲まれているため通常は無効)
- コンパイラ warning (符号比較、未使用変数等) は元コード由来。動作には影響しない
