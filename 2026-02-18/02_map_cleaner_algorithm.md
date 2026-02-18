# Map Cleaner アルゴリズム処理フロー

## 概要

MapCleanerは、複数フレームのLiDAR点群データを入力として、**地面分離→標高マップ構築→フィルタリング→地形分類→動的物体検出**のパイプラインを通じて、クリーンな静的マップを生成するシステムである。

### 入力データ

- **複数フレームの点群** (KITTI / ERASOR / GLIM形式)
- **各フレームのポーズ情報** (位置 + 姿勢)

### 出力

| 出力ファイル | 内容 |
|---|---|
| `terrain.pcd` | クリーンな地形標高モデル |
| `ground.pcd` | 地面上の点群 |
| `ground_below.pcd` | 地面下の点群 |
| `static.pcd` | 静止障害物 |
| `dynamic.pcd` | 動的物体（車両・歩行者等） |
| `other.pcd` | グリッド範囲外の点 |

---

## パイプライン全体図

```
複数フレーム (点群 + ポーズ)
    │
    ▼
[1] Ground Segmentation (PatchWorkpp) ── 地面/非地面の分離
    │ (地面点群)
    ▼
[2] Grid Map Builder ── 2D標高グリッドマップ構築
    │
    ▼
[3] Variance Filter ── 高分散セルの除去
    │
    ▼
[4] 1st BGK Filter (kernel=1.0m) ── 軽い平滑化
    │
    ▼
[5] Trajectory Filter ── 走行経路外のアーティファクト除去
    │
    ▼
[6] 2nd BGK Filter (kernel=2.0m) ── 強い平滑化
    │
    ▼
[7] Median Filter ── 最終的なノイズ除去
    │ (完成した地形モデル)
    ▼
[8] Divide By Terrain ── 地形に対する点の高さ分類
    ├── ground.pcd (地面上)
    ├── ground_below.pcd (地面下)
    │
    └── 非地面点 (地面上方)
         │
         ▼
[9] Moving Point Identification ── レンジ画像比較による動的物体検出
         ├── static.pcd (静止物体)
         ├── dynamic.pcd (動的物体)
         └── other.pcd (その他)
```

---

## 各ステップ詳細

---

### Step 1: Ground Segmentation (PatchWorkpp)

- **ファイル**: `include/map_cleaner/GroundSegmentation.hpp`, `thirdparty/patchworkpp/`
- **出力**: 結合済み点群 + 地面インデックス + 非地面インデックス

#### 1.1 呼び出しフロー

1. 各フレームの点群を `DataLoader` から読み込み
2. ボクセルグリッドでダウンサンプリング（オプション）
3. フレームスキップ（設定可能）
4. 各フレームをポーズの回転で変換
5. `PatchWorkpp::estimateGround()` を呼び出し
6. ポーズの並進で世界座標系に変換
7. 全フレームの結果を結合

#### 1.2 PatchWorkpp アルゴリズム全体フロー

```
入力点群
  │
  ▼
[RNR] 反射ノイズ除去
  │
  ▼
[CZM] 同心円ゾーンモデルへの点割り当て
  │
  ▼
各パッチ(zone/ring/sector)について:
  ├── Z高さでソート
  ├── [R-VPF] 垂直面除去 (Zone 0のみ)
  ├── [R-GPF] 反復的地面平面フィッティング (PCA)
  └── [GLE] 地面尤度推定
        ├── uprightness（法線の鉛直度）チェック
        ├── elevation（標高）チェック（適応的閾値）
        ├── flatness（平坦度）チェック（適応的閾値）
        └── 分類: GROUND / NONGROUND / CANDIDATE
  │
  ▼
[TGR] 時間的地面復帰（CANDIDATEの再判定）
  │
  ▼
適応的閾値の更新
  │
  ▼
出力: 地面/非地面インデックス
```

#### 1.3 CZM（Concentric Zone Model: 同心円ゾーンモデル）

点群をセンサからの距離と角度に基づき、ゾーン→リング→セクターの階層構造に分割する。

| ゾーン | 距離範囲 | リング数 | セクター数 |
|--------|---------|----------|-----------|
| Zone 0 | min_range ～ min_range_z2 | 2 | 16 |
| Zone 1 | min_range_z2 ～ min_range_z3 | 4 | 32 |
| Zone 2 | min_range_z3 ～ min_range_z4 | 4 | 54 |
| Zone 3 | min_range_z4 ～ max_range | 4 | 32 |

各点の割り当て:
```
r = sqrt(x² + y²)        // 水平距離
θ = atan2(y, x)           // 方位角
zone ← r に基づくゾーン選択
ring_idx = (r - zone_min_range) / ring_size
sector_idx = θ / sector_size
→ czm[zone][ring][sector] に格納
```

#### 1.4 RNR（Reflected Noise Removal: 反射ノイズ除去）

マルチパス反射によるノイズ点を除去する。

```
各点 (x, y, z, intensity) について:
  ver_angle = atan2(z, r) × 180/π        // 垂直角度

  以下の3条件すべてを満たす場合 → ノイズとして除去:
    1. ver_angle < RNR_ver_angle_thr (-15°)   // 下向きの反射
    2. z < -sensor_height - 0.8                // センサより十分下方
    3. intensity < RNR_intensity_thr (0.2)     // 低輝度（弱い反射）
```

#### 1.5 R-VPF（Region-wise Vertical Plane Fitting: 領域別垂直面フィッティング）

Zone 0で壁やポールなどの垂直構造物を検出・除去する。

```
Zone 0の各パッチについて（最大 num_iter 回反復）:
  1. 最低点群からシード抽出（th_seeds_v 以内）
  2. PCAで平面推定
  3. 法線の鉛直成分 normal.z < 0.707 の場合（=垂直面）:
     - 平面からの距離 < th_dist_v (0.1m) の点 → 非地面へ
     - その他の点 → 残留し次の反復へ
  4. normal.z ≥ 0.707 になれば終了（地面を発見）
```

#### 1.6 R-GPF（Region-wise Ground Plane Fitting: 領域別地面平面フィッティング）

PCA（主成分分析）による反復的な地面平面推定。

```
1. シード抽出: 最低 num_lpr 点から th_seeds 以内の点を選択

2. PCA平面推定:
   - 点群を Eigen 行列 (N×3) に変換
   - 重心を計算し中心化
   - 共分散行列 cov = (centered^T × centered) / (N-1)
   - SVD分解: cov = U × Σ × V^T
   - 法線 = 最小特異値に対応する特異ベクトル（U の第3列）
   - 法線が下向きの場合は反転（normal.z < 0 → normal *= -1）

3. 反復精錬（num_iter = 3回）:
   各点について:
     dist = 点から平面への距離
     dist < th_dist (0.125m) → 地面候補に追加
     dist ≥ th_dist → 非地面に追加
   地面候補から再度PCA推定 → 繰り返し
```

#### 1.7 GLE（Ground Likelihood Estimation: 地面尤度推定）

PCA結果からパッチの地面らしさを判定する。

```
PCAから抽出するメトリクス:
  - uprightness = normal.z（法線の鉛直成分、1.0で完全水平面）
  - elevation = mean.z（パッチ中心の標高）
  - flatness = singular_values.min()（最小特異値、平面からの分散）
  - line_variable = σ[0]/σ[1]（アスペクト比、>8 でライン状）

分類ロジック:
  if (!upright):          → NONGROUND（傾いた平面）
  else if (遠方ゾーン):    → GROUND（遠方は水平なら地面と仮定）
  else if (heading不正):   → NONGROUND（法線方向が異常）
  else if (低標高 or 平坦): → GROUND（品質チェック通過）
  else:                    → CANDIDATE（不確定、TGRへ）
```

#### 1.8 TGR（Temporal Ground Revert: 時間的地面復帰）

CANDIDATEパッチを確率的に再判定する。

```
各リングの確認済み地面パッチから flatness の統計量 (μ, σ) を計算

各CANDIDATEパッチについて:
  1. 平坦度の確率:
     mu_flatness = μ + 1.5σ
     prob_flatness = 1 / (1 + exp((flatness - mu_flatness) / (mu_flatness/10)))
     （シグモイド関数による滑らかな遷移）

     特殊ケース: サイズ > 1500 かつ flatness < th_dist² → prob = 1.0

  2. ライン構造の確率:
     line_variable > 8.0 → prob_line = 0.0（ライン状、地面でない）
     それ以外            → prob_line = 1.0

  3. 復帰判定:
     prob_line × prob_flatness > 0.5 かつ 近距離ゾーン → GROUND に復帰
     それ以外 → NONGROUND
```

#### 1.9 適応的閾値の更新

確認済み地面パッチの統計量から閾値を動的に更新する。

```
標高閾値の更新:
  各リングについて:
    mean = 確認済み地面の標高平均
    stdev = 標高の標準偏差
    Zone 0: elevation_thr = mean + 3σ（緩い閾値、センサ高さも更新）
    その他: elevation_thr = mean + 2σ

平坦度閾値の更新:
  各リングについて:
    flatness_thr = mean + σ

※ 最新1000サンプルのみ保持（時間的安定性の確保）
```

#### 1.10 PatchWorkpp 主要パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| num_zones | 4 | ゾーン数 |
| num_iter | 3 | PCA反復回数 |
| num_lpr | 20 | 最低点代表数 |
| num_min_pts | 10 | 最小点数 |
| sensor_height | 1.723m | センサ高さ |
| th_seeds | 0.125m | 地面シード閾値 |
| th_dist | 0.125m | 地面層厚さ |
| th_seeds_v | 0.25m | 垂直構造シード閾値 |
| th_dist_v | 0.1m | 垂直構造厚さ |
| uprightness_thr | 0.707 | 法線鉛直成分閾値（cos 45°）|
| RNR_ver_angle_thr | -15.0° | RNR垂直角度閾値 |
| RNR_intensity_thr | 0.2 | RNR輝度閾値 |
| min_range | 2.7m | 最小距離 |
| max_range | 80.0m | 最大距離 |

---

### Step 2: Grid Map Builder

- **ファイル**: `include/map_cleaner/GridMapBuilder.hpp`

#### 2.1 グリッド構築手順

1. **AABB計算**: 地面点群の x, y 最小/最大値を算出
2. **GridMap作成**: `grid_map::GridMap` を6レイヤーで初期化
3. **セル更新**: 各地面点について対応するグリッドセルの統計量を逐次更新

#### 2.2 セル統計量のオンライン更新（Welfordの公式）

```
初回の点（セルが空の場合）:
  count = 1, average = z, variance = 0
  min = z, max = z, intensity = point.intensity

N番目の点の追加:
  last_avg = average（N-1時点の平均）

  average = (average × (N-1) + z) / N          // オンライン平均

  variance = ((N-1) × (old_var + last_avg²) + z²) / N - average²
             // 平行軸の定理: Var = E[X²] - (E[X])²

  min = min(min, z)
  max = max(max, z)
  intensity = max(intensity, point.intensity)   // 最大輝度
  count += 1
```

#### 2.3 出力レイヤー

| レイヤー名 | 内容 |
|-----------|------|
| `count` | セル内の点数 |
| `average` | 平均Z標高 |
| `variance` | Z値の分散 |
| `min` | 最小Z標高 |
| `max` | 最大Z標高 |
| `intensity` | 最大輝度値 |

---

### Step 3: Variance Filter

- **ファイル**: `include/map_cleaner/VarianceFilter.hpp`
- **入力レイヤー**: `count`, `variance`, `average`
- **出力レイヤー**: `variance_filtered`

#### 3.1 フィルタリングロジック

```
各グリッドセルについて:
  if (count >= min_num AND variance < variance_th):
    output = average（平均標高値を保持）
  else:
    output = NaN（無効化）
```

#### 3.2 パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| variance_th | 0.01 | 分散閾値（これ以上は除去） |
| min_num | 3 | 最小点数（これ未満は除去） |

#### 3.3 目的

- 高分散セル（ノイズ・不安定な地形）を除去
- 点数が少なく信頼性の低いセルを除去

---

### Step 4: 1st BGK Filter

- **ファイル**: `include/map_cleaner/BGKFilter.hpp`
- **入力レイヤー**: `variance_filtered`
- **出力レイヤー**: `first_bgk_filtered`

#### 4.1 BGKカーネル関数

```
k(pos, center) = ((2 + cos(θ)) / 3) × (1 - d_r) + sin(θ) / (2π)

ここで:
  d_r = ||pos - center|| / kernel_radius    // 正規化距離 [0, 1]
  θ = 2π × d_r                              // 角度パラメータ
```

ガボールカーネルに類似した関数で、放射基底成分 `(1 - d_r)` と振動成分 `cos(θ)`, `sin(θ)` を組み合わせている。

#### 4.2 2パス方式

**Pass 1（通常BGK）**:
```
各セルについて（OpenMPで並列処理）:
  円形近傍（半径 = kernel_radius）内の有効セルを収集

  sum_k = 0, sum_k_z = 0, n = 0
  各近傍セルについて:
    z が NaN → スキップ
    k = kernel(pos, center)
    sum_k += k
    sum_k_z += k × z
    n++

  if n < min_num: 出力 = NaN
  else:
    prior_z = λ × center_z（λ = 0 なら prior 無効）
    出力 = (prior_z + sum_k_z) / (λ + sum_k)
```

**Pass 2（バイラテラルBGK）**:
```
各セルについて:
  if 元の入力値が有効 → 元の値をそのまま保持
  else if Pass 1の結果が有効:
    Pass 1の結果を参照値としてバイラテラル重み付け:

    c = exp(-(z_in - z_ref)² / (2σ²))    // バイラテラル重み

    sum_c_k = Σ(c × k)
    sum_c_k_z = Σ(c × k × z_in)

    出力 = (prior_z + sum_c_k_z) / (λ + sum_c_k)
  else: 出力 = NaN
```

#### 4.3 パラメータ（1st BGK）

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| kernel_radius | 1.0m | カーネル半径 |
| lambda | 0.0 | 事前分布の重み |
| sigma | 1.0 | バイラテラルの範囲パラメータ |
| min_num | 3 | 最小有効近傍数 |

#### 4.4 目的

- 分散フィルタで除去されたセル（NaN）を近傍から補間
- エッジ保存平滑化（バイラテラル重みにより急な段差を保持）

---

### Step 5: Trajectory Filter

- **ファイル**: `include/map_cleaner/TrajectoryFilter.hpp`
- **入力レイヤー**: `first_bgk_filtered`
- **出力レイヤー**: `trajectory_filtered`

#### 5.1 表面法線の計算

境界セル（端から1セル内）を除く全セルについて:

```
上下方向の勾配:
  z_u, z_d = 上下隣接セルの標高
  ケース1: 両方有効 → z_diff_ud = z_d - z_u, distance = 2 × resolution
  ケース2: 中央と下のみ → z_diff_ud = z_d - z_center, distance = resolution
  ケース3: 上と中央のみ → z_diff_ud = z_center - z_u, distance = resolution
  ケース4: どちらもNaN → スキップ

左右方向の勾配: （同様の3ケース処理）

法線ベクトル:
  n = normalize(z_diff_ud / distance_ud, z_diff_lr / distance_lr, 1.0)
  angle = acos(|n · (0, 0, 1)|)    // 鉛直方向からの角度
```

#### 5.2 Flood-fill マスク生成

```
1. 初期化: 全走行軌跡点（ポーズ位置）のうち、
   法線が有効なセルを open_list に追加

2. Flood-fill:
   while open_list が空でない:
     idx = open_list.pop_back()
     if 法線が有効 AND マスク未設定:
       if 法線角度 < normal_th:
         mask[idx] = 1.0（有効マーク）
         8近傍（上下左右＋斜め）を open_list に追加
```

#### 5.3 マスク適用

```
各セルについて:
  if mask が有効: 出力 = 入力値
  else: 出力 = NaN
```

#### 5.4 パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| normal_th_deg | 15.0° | 法線角度閾値（これ以下の平坦なセルのみ接続） |

#### 5.5 目的

- 走行経路から到達可能な平坦な地形のみを保持
- 遠方の浮遊物体やアーティファクトを除去
- 法線角度による勾配制限で崖や壁を境界として切断

---

### Step 6: 2nd BGK Filter

- **ファイル**: `include/map_cleaner/BGKFilter.hpp`（Step 4と同一アルゴリズム）
- **入力レイヤー**: `trajectory_filtered`
- **出力レイヤー**: `second_bgk_filtered`

#### 6.1 パラメータ（2nd BGK）

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| kernel_radius | 2.0m | カーネル半径（1st より大きい） |
| lambda | 0.0 | 事前分布の重み |
| sigma | 0.5 | バイラテラルの範囲パラメータ（1st より小さい） |
| min_num | 3 | 最小有効近傍数 |

#### 6.2 目的

- Trajectory Filterで生じたエッジを平滑化
- より広いカーネルでマクロな地形を滑らかに
- sigma が小さいため、値の差が大きい近傍は重みが低くなりエッジ保存

---

### Step 7: Median Filter

- **ファイル**: `include/map_cleaner/MedianFilter.hpp`
- **入力レイヤー**: `second_bgk_filtered`
- **出力レイヤー**: `elevation`（最終地形モデル）

#### 7.1 アルゴリズム

```
各セルについて:
  if 入力値が NaN → スキップ

  kernel_size × kernel_size の正方近傍（例: 3×3）から有効値を収集

  if 有効値なし → スキップ

  std::nth_element で中央値を O(n) で算出
  出力 = 中央値
```

#### 7.2 パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| kernel_size | 3 | カーネルサイズ（3×3） |

#### 7.3 目的

- BGKフィルタ後に残留するゴマ塩ノイズの除去
- 非線形フィルタのため外れ値に対してロバスト

---

### Step 8: Divide By Terrain

- **ファイル**: `include/map_cleaner/DivideByTerrain.hpp`
- **入力**: 全点群 + 地面/非地面インデックス + 最終標高グリッド

#### 8.1 分類ロジック

```
各点について:
  1. グリッドセルの取得: grid.getIndex(Position(p.x, p.y), idx)
     → 失敗（グリッド外）→ other に分類

  2. 地形標高の取得: terrain_z = elevation_layer(idx)
     → NaN（無効セル）→ other に分類

  3. 高さ差分: diff_z = p.z - terrain_z

  4. 分類:
     diff_z < -threshold  → ground_below（地面下）
     diff_z > +threshold  → ground_above（地面上方）
     それ以外             → ground（地面上）
```

#### 8.2 動作モード

- **on_terrain_only = true**: 地面点のみを分類、非地面点はそのまま above へ
- **on_terrain_only = false**: 地面点・非地面点の両方を上記ロジックで分類

#### 8.3 パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| threshold | 0.172m | 高さ差分の閾値（±17.2cm） |
| on_terrain_only | false | 地面点のみ分類するか |

#### 8.4 出力インデックス

| 出力 | 内容 |
|------|------|
| `ground_indices` | 地形上の点（±threshold以内） |
| `ground_above_indices` | 地形より上の点（障害物等） |
| `ground_below_indices` | 地形より下の点（影・エラー） |
| `other_indices` | グリッド外・無効セルの点 |

---

### Step 9: Moving Point Identification

- **ファイル**: `include/map_cleaner/MovingPointIdentification.hpp`
- **入力**: 走行軌跡（ポーズ）+ ground_above_indices（地面上方の点のみ）

#### 9.1 前処理

```
1. ground_above の点群をボクセルグリッドでダウンサンプリング（0.2m）
   → cloud_ds（ダウンサンプリング済み点群）
   → vg_indices[i] に各ボクセルに含まれる元のインデックスを保持

2. KD-tree 構築（nanoflann、3D空間）
   cloud_ds に対して空間インデックスを構築

3. 投票リスト初期化:
   vote_list_static[i] = 0（cloud_ds の各点）
   vote_list_dynamic[i] = 0
```

#### 9.2 レンジ画像構築（球面投影）

各フレームのスキャンデータから2Dレンジ画像を生成する。

```
各点 (x, y, z) について:
  range = sqrt(x² + y² + z²)                // 3Dユークリッド距離
  pos_h = atan2(y, x) × res_h_scale         // 方位角 → グリッド座標
  pos_v = asin(z / range) × res_v_scale     // 仰角 → グリッド座標

グリッドサイズ:
  水平: fov_h × res_h_scale
  垂直: fov_v × res_v_scale
  解像度: range_im_res（水平・垂直解像度の小さい方）

スケール係数（解像度の正規化）:
  if res_h < res_v:
    res_h_scale = 1.0, res_v_scale = res_h/res_v
  else:
    res_h_scale = res_v/res_h, res_v_scale = 1.0
```

#### 9.3 サブマップ抽出

```
KD-tree半径検索:
  center = 現在のLiDARポーズ位置
  radius = lidar_range（50m）
  → 半径内の全ダウンサンプリング済み点をサブマップとして抽出

サブマップ更新戦略:
  前回更新位置からの移動距離 > submap_update_dist (3.0m) の場合のみ再構築
  → 冗長な再構築を防止
```

#### 9.4 レンジ比較の4ケース

サブマップの各点（target）と現在フレームのレンジ画像（scan）を比較:

```
compareRange(scan_range, target_range):

  CASE_A: |target - scan| ≤ threshold
    → レンジが一致 → 静止物体の証拠

  CASE_B: target > scan + threshold
    → ターゲットがスキャンより遠方 → オクルージョン（無視）

  CASE_C: target < scan - threshold
    → ターゲットがスキャンより手前 → 動的物体の証拠

  CASE_D: scan または target が NaN/無限大
    → データ不足（無視）
```

#### 9.5 近傍投票（getFusedResult）

単一セルではなく、近傍セルも含めて多数決を行う。

```
(2×delta_h+1) × (2×delta_v+1) の近傍について:
  各セルのレンジ比較結果をカウント

水平方向ラッピング:
  spinning_lidar（360° LiDAR）の場合、方位角方向で循環

投票の融合（優先順位）:
  CASE_A が1つでもあれば → CASE_A（静止）を返す
  CASE_B があれば → OTHERWISE（無視）
  CASE_C があれば → CASE_C（動的）を返す
  それ以外 → OTHERWISE（無視）
```

#### 9.6 投票集約と最終分類

```
全フレームについてループ:
  1. 現在フレームのレンジ画像を構築
  2. サブマップを抽出（必要に応じて更新）
  3. サブマップを現在LiDAR座標系に変換
  4. 各サブマップ点について近傍投票:
     CASE_A → vote_list_static[point_idx]++
     CASE_C → vote_list_dynamic[point_idx]++

最終分類:
  各ダウンサンプリング済み点について:
    if vote_dynamic[i] ≤ vote_static[i]:
      → 静止物体（vg_indices[i] の全元点を static_indices に追加）
    else:
      → 動的物体（vg_indices[i] の全元点を dynamic_indices に追加）
```

#### 9.7 パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| voxel_leaf_size | 0.2m | ダウンサンプリング解像度 |
| fov_h_deg | 360.0° | 水平視野角 |
| fov_v_deg | 40.0° | 垂直視野角 |
| res_h_deg | 0.2° | 水平解像度 |
| res_v_deg | 2.0° | 垂直解像度 |
| lidar_range | 50.0m | サブマップ抽出半径 |
| delta_h | 5 | 水平近傍投票幅 |
| delta_v | 1 | 垂直近傍投票幅 |
| threshold | 0.15m | レンジ差分閾値 |
| submap_update_dist | 5.0m | サブマップ更新距離 |
| frame_skip | 0 | フレームスキップ数（0=全フレーム処理） |

---

## DataLoader（データ読み込み）

- **ファイル**: `include/map_cleaner/DataLoader.hpp`

### KITTI形式

- **点群**: バイナリファイル（4 float/点: x, y, z, intensity）
- **ポーズ**: テキストファイル（3×4変換行列）
- **キャリブレーション**: `lidar_pose = calib_inv × camera_pose × calib`
- **ファイル名**: `{pcd_dir}/{frame_id:06d}.bin`

### ERASOR形式

- **点群**: PCD形式（`pcl::io::loadPCDFile`）
- **ポーズ**: CSV（`idx, timestamp, tx, ty, tz, qx, qy, qz, qw`）
- **四元数正規化**: `quat.normalize()`
- **ファイル名**: `{pcd_dir}/{frame_idx:06d}.pcd`

### GLIM形式（詳細）

GLIMのSLAM出力（`traj_lidar.txt` + フレームディレクトリ）を読み込む。
フレームデータの保存には [glim_save_frame_plugin](https://github.com/kamibukuro5656/glim_save_frame_plugin) が必要（GLIMデフォルトでは個別フレームは保存されない）。

#### クラス構造

```
DataLoaderBase（基底クラス）
  │
  ├── Frame 構造体:
  │   ├── idx: size_t（フレーム番号）
  │   ├── frame: CloudType::Ptr（点群）
  │   ├── t: Eigen::Vector3f（並進）
  │   └── r: Eigen::Quaternionf（回転）
  │
  ├── FrameInfo 構造体:
  │   ├── idx: size_t（フレーム番号）
  │   ├── pcd_filename: std::string（フレームディレクトリパス）
  │   ├── t: Eigen::Vector3f（並進）
  │   └── r: Eigen::Quaternionf（回転）
  │
  └── GLIMFormatLoader（派生クラス）
      ├── loadFrameInfo()  ── ポーズファイル解析・フレーム情報構築
      ├── loadFrame()      ── 個別フレームの読み込み+NaN除去
      ├── loadGLIMCloud()  ── バイナリ点群・輝度ファイルの読み込み
      └── parseLine()      ── ポーズファイル1行の解析
```

#### 期待されるディレクトリ構造

```
pcds_dir/                          （例: /tmp/dump/frames）
├── 00000000/
│   ├── points_compact.bin         （点群バイナリ）
│   └── intensities_compact.bin    （輝度バイナリ）
├── 00000001/
│   ├── points_compact.bin
│   └── intensities_compact.bin
├── 00000002/
│   ├── ...
└── XXXXXXXX/
    └── ...

pose_file                          （例: /tmp/dump/traj_lidar.txt）
├── 行0: timestamp tx ty tz qx qy qz qw  ← フレーム 00000000 に対応
├── 行1: timestamp tx ty tz qx qy qz qw  ← フレーム 00000001 に対応
└── ...
```

- フレームディレクトリ名: **8桁ゼロ埋め**（`std::setw(8)`）
- ポーズファイルの行番号がフレームインデックスに直接対応

#### loadFrameInfo(): ポーズファイル解析

```
入力: pcds_dir, pose_file, start, end

1. pose_file をテキストとして開く
2. 各行について（行番号 = line_num、0始まり）:
   a. line_num < start → スキップ
   b. start ≤ line_num < end (end=-1なら全行):
      - parseLine() でポーズを解析
      - フレームディレクトリパスを生成:
        pcd_filename = pcds_dir + "{line_num:08d}/"
      - frame_info_buf_ に追加
   c. line_num ≥ end → 終了
```

**parseLine(): 1行のパース処理**

```
入力: スペース区切り文字列
形式: "timestamp tx ty tz qx qy qz qw"

各トークン:
  [0] timestamp（double） → 使用しない（破棄）
  [1] tx → frame_info.t[0]（X並進）
  [2] ty → frame_info.t[1]（Y並進）
  [3] tz → frame_info.t[2]（Z並進）
  [4] qx → tmp_quat[0]
  [5] qy → tmp_quat[1]
  [6] qz → tmp_quat[2]
  [7] qw → tmp_quat[3]

後処理: frame_info.r.normalize()（四元数の正規化）
```

#### loadGLIMCloud(): バイナリ点群読み込み

**Part 1: points_compact.bin（点座標）**

```
ファイル形式:
  [Eigen::Vector3f][Eigen::Vector3f][Eigen::Vector3f]...
  = [x,y,z][x,y,z][x,y,z]...（各要素 float = 4バイト）

1点あたり: sizeof(Eigen::Vector3f) = 12バイト（3 × 4バイト）

読み込み手順:
  1. バイナリモードで開く（std::ios::binary | std::ios::ate）
  2. ファイルサイズから点数を算出: num_points = bytes / 12
  3. cloud.reserve(num_points) でメモリ確保
  4. 先頭にシーク後、1点ずつ読み込み:
     in.read((char*)&eigen_p, sizeof(Eigen::Vector3f))
  5. PointXYZI に変換: p.x = eigen_p[0], p.y = eigen_p[1], p.z = eigen_p[2]
     p.intensity = 0（初期値、後でPart 2で上書き）
  6. NaNチェック: std::isfinite() で各座標を検査
     → NaN検出時は cloud.is_dense = false に設定
  7. shrink_to_fit() でメモリ最適化
```

**Part 2: intensities_compact.bin（輝度値）**

```
ファイル形式:
  [float][float][float]...（各 4バイト）

1値あたり: sizeof(float) = 4バイト

読み込み手順:
  1. バイナリモードで開く
  2. ファイルサイズから値数を算出: num_points = bytes / 4
  3. 点数の一致確認: num_points == cloud.size()
     → 不一致の場合はスキップ（輝度は0のまま）
  4. 1値ずつ読み込み:
     in.read((char*)&intensity, sizeof(float))
     cloud[i].intensity = intensity
```

#### loadFrame(): フレーム読み込み

```
入力: idx（frame_info_buf_ 内のインデックス）

1. 範囲チェック: idx ≥ frame_info_buf_.size() → 空の Frame を返す
2. loadGLIMCloud(pcd_filename, tmp_frame) で点群読み込み
3. メタデータコピー: idx, t（並進）, r（回転）
4. NaN除去: pcl::removeNaNFromPointCloud(tmp_frame, *frame.frame, dummy)
   → NaN座標を含む点を完全に除去
5. Frame を返す
```

#### エラーハンドリング

| 状況 | 挙動 |
|------|------|
| points_compact.bin が存在しない | `loadGLIMCloud()` が早期リターン（空の点群） |
| intensities_compact.bin が存在しない | 輝度は全て0のまま（サイレント） |
| 輝度ファイルの点数が一致しない | 輝度読み込みをスキップ（サイレント） |
| フレームインデックスが範囲外 | 空の Frame を返す |
| ポーズファイルが開けない | frame_info_buf_ が空のまま |
| 点座標にNaNが含まれる | `removeNaNFromPointCloud()` で除去 |

※ エラー時に例外をスローせず、サイレントに処理を継続する設計。

#### 設定パラメータ（config_glim.yaml）

```yaml
loader:
  pcds_dir: "/tmp/dump/frames"           # フレームディレクトリの親パス
  pose_file: "/tmp/dump/traj_lidar.txt"  # GLIMのポーズ出力ファイル
  format: "glim"                          # フォーマット指定
  start: 0                                # 開始フレーム
  end: -1                                 # 終了フレーム（-1 = 全フレーム）
```

### 共通処理

- 全形式で `pcl::removeNaNFromPointCloud()` によるNaN除去を実施
- ポーズは位置（並進: Eigen::Vector3f）+ 姿勢（四元数: Eigen::Quaternionf）として保持
- 四元数は読み込み後に正規化

---

## メインエントリポイント

処理は `src/map_cleaner.cpp` の `MapCleaner::exec()` メソッドで上記パイプラインが順次実行される。

```
launch/run.launch → map_cleaner ノード起動 → MapCleaner::exec() → パイプライン実行
```

### フィルタリングパイプラインのレイヤー遷移

| Step | フィルタ | 入力レイヤー | 出力レイヤー |
|------|---------|-------------|-------------|
| 3 | VarianceFilter | count, variance, average | variance_filtered |
| 4 | 1st BGKFilter | variance_filtered | first_bgk_filtered |
| 5 | TrajectoryFilter | first_bgk_filtered | trajectory_filtered |
| 6 | 2nd BGKFilter | trajectory_filtered | second_bgk_filtered |
| 7 | MedianFilter | second_bgk_filtered | elevation |

### メモリ管理

Step 9（MovingPointIdentification）の前に、不要になった `initial_ground_indices` と `nonground_indices` を明示的に解放する。

---

## 設定ファイル

- `config/config.yaml`: デフォルト設定
- `config/config_kitti.yaml`: KITTIデータセット用
- `config/config_glim.yaml`: GLIMデータセット用

## 依存ライブラリ

| ライブラリ | 用途 |
|---|---|
| PatchWorkpp (`thirdparty/patchworkpp/`) | 地面セグメンテーション |
| nanoflann (`thirdparty/nanoflann/`) | KD-tree高速空間検索 |
| Modified PCL (`thirdparty/mod_pcl/`) | 大規模点群のボクセルダウンサンプリング、インデックス付きPCD書き出し |
| grid_map | 2D標高グリッドマップ |
| Eigen3 | 線形代数演算 |
| OpenMP | 並列処理（BGKFilter等） |
