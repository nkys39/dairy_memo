# atc_ws ROS2 Humble → Jazzy ポーティング

## 概要

`atc_ws` ワークスペース（`livox_ros_driver2`, `fast_lio`, `cae_percep` の3パッケージ）を ROS2 Humble から Jazzy に移行した。

- **環境**: Ubuntu 24.04 / ROS2 Jazzy / GCC 13.3 / CMake 4.2.1
- **Livox SDK**: インストール済み（`/usr/local/lib/liblivox_lidar_sdk_shared.so`）

---

## 設計方針

| 項目 | 方針 |
|------|------|
| C++標準 | C++14 → C++17（Jazzy の最低要件） |
| CMake Python検索 | `FindPythonLibs` → `FindPython3`（CMake 3.27で前者削除） |
| ヘッダファイル | `.h` → `.hpp`（Jazzy でのヘッダ名変更に追従） |
| package.xml | 不足依存の追加、存在しないパッケージの削除 |
| ビルド順序 | `livox_ros_driver2` → `fast_lio` → `cae_percep`（依存チェーン順） |

---

## 事前準備: apt パッケージ

ROS2 Jazzy 本体と主要パッケージはインストール済みだった。不足していたのは3つのみ:

```bash
sudo apt install -y libf2c2-dev libapr1-dev liblapacke-dev
```

| パッケージ | 用途 | 必要パッケージ |
|---|---|---|
| `libf2c2-dev` | Fortran→C変換（PCA計算） | `cae_percep` |
| `libapr1-dev` | Apache Portable Runtime（通信層） | `livox_ros_driver2` |
| `liblapacke-dev` | LAPACK C API（線形代数） | `cae_percep` |

---

## 変更ファイル一覧

### livox_ros_driver2

| ファイル | 変更内容 |
|----------|----------|
| `CMakeLists.txt` | `set(CMAKE_CXX_STANDARD 14)` → `17` |
| `package.xml` | `libpcl-all-dev` → `libpcl-dev`（Ubuntu 24.04 でメタパッケージ名変更） |
| `package_ROS2.xml` | 同上（`build.sh` がコピー元として使用） |

### fast_lio (FAST_LIO)

| ファイル | 変更内容 |
|----------|----------|
| `CMakeLists.txt` | C++14→17（4箇所）、重複 `ADD_COMPILE_OPTIONS` 削除、冗長フラグ `-std=c++0x` 削除 |
| `CMakeLists.txt` | `find_package(PythonLibs)` → `find_package(Python3 COMPONENTS Development)` |
| `CMakeLists.txt` | `${PYTHON_LIBRARIES}` → `Python3::Python`、`${PYTHON_INCLUDE_DIRS}` → `${Python3_INCLUDE_DIRS}` |
| `package.xml` | `rospy` 依存削除（ROS1パッケージ、Jazzy に存在しない） |
| `package.xml` | `std_srvs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `rclcpp_components` 追加 |
| `src/laserMapping.cpp` | `tf2_geometry_msgs/tf2_geometry_msgs.h` → `.hpp` |

### cae_percep

| ファイル | 変更内容 |
|----------|----------|
| `CMakeLists.txt` | `set(CMAKE_CXX_STANDARD 17)` + `set(CMAKE_CXX_STANDARD_REQUIRED ON)` 追加 |
| `CMakeLists.txt` | 重複 `find_package(ament_cmake REQUIRED)` 削除 |
| `package.xml` | 9個の依存追加（`rclcpp`, `std_msgs`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`） |
| `src/cae_percep.cpp` | `cv_bridge/cv_bridge.h` → `cv_bridge/cv_bridge.hpp` |
| `.vscode/c_cpp_properties.json` | `/opt/ros/humble/` → `/opt/ros/jazzy/`、`c++14` → `c++17` |

---

## 問題と修正

### 1. cv_bridge ヘッダ不存在

- **症状**: `fatal error: cv_bridge/cv_bridge.h: そのようなファイルやディレクトリはありません`
- **原因**: Jazzy では `cv_bridge/cv_bridge.hpp` に変更されている
- **修正**: `cae_percep/src/cae_percep.cpp` のインクルードを `.hpp` に変更

### 2. tf2_geometry_msgs ヘッダ不存在

- **症状**: `fatal error: tf2_geometry_msgs/tf2_geometry_msgs.h: そのようなファイルやディレクトリはありません`
- **原因**: Jazzy では `tf2_geometry_msgs/tf2_geometry_msgs.hpp` に変更されている
- **修正**: `FAST_LIO/src/laserMapping.cpp` のインクルードを `.hpp` に変更

### 3. libpcl-all-dev 不存在

- **症状**: `rosdep` で `libpcl-all-dev` が解決できない
- **原因**: Ubuntu 24.04 ではメタパッケージ名が `libpcl-dev` に変更
- **修正**: `livox_ros_driver2/package.xml` と `package_ROS2.xml` を `libpcl-dev` に変更

---

## 変更不要だった箇所

- **EOL_LIST チェック** (`FAST_LIO/CMakeLists.txt` 行112-122): `jazzy` は EOL リストに含まれないため、正しく `rosidl_get_typesupport_target` パスを通る
- **launch ファイル**: 全て Python launch API を使用しており、Humble/Jazzy 間で互換性あり
- **YAML config**: `ros__parameters` 形式で統一されており変更不要
- **カスタムメッセージ**: `Pose6D.msg`, `CustomMsg.msg`, `CustomPoint.msg` はそのまま動作

---

## 実行方法

```bash
source /opt/ros/jazzy/setup.bash
cd /home/user/workspace_ros2/atc_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 残課題

- Boost の deprecated 警告（`BOOST_BIND_GLOBAL_PLACEHOLDERS`）が `fast_lio` ビルド時に出力される（動作には影響なし）
- `cae_percep/src/cae.cpp` に多数のコンパイラ警告（未使用変数、VLA使用等）が残存（既存コードの問題、動作には影響なし）
- `CMakeLists.txt` の `cmake_minimum_required(VERSION 3.8)` に対する deprecation 警告（CMake 4.2 では 3.10 未満が非推奨）
