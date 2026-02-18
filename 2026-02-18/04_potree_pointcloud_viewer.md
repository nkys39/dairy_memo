# MapCleaner 出力の Potree Web ビューア構築

## 概要

MapCleaner の PCD 出力を Web ブラウザで閲覧できるよう、PCD → LAS 1.2 変換 → PotreeConverter によるオクツリー生成 → Potree ビューアのカスタマイズを行った。GitHub Pages での複数データセットホスティングを想定したファイル構成も設計した。

---

## PCD → LAS 1.2 変換

MapCleaner が出力する PCD v0.7 binary（float32×4 = 16 bytes/点、x, y, z, intensity）を LAS 1.2 Format 0 に変換した。

### 分類コードのマッピング

LAS 1.2 Format 0 の classification フィールドは 5-bit（0〜31）のため、MapCleaner の動的物体コード 64 をリマッピングした。

| PCD ファイル | LAS 分類コード | 説明 |
|-------------|--------------|------|
| `ground.pcd` | 2 | 地面 |
| `terrain.pcd` | 4 | 地形 |
| `static.pcd` | 6 | 静的物体 |
| `ground_below.pcd` | 7 | 地面より下 |
| `dynamic.pcd` | 20 | 動的物体（64 → 20 にリマッピング） |

### 変換スクリプト

- **ファイル**: `convert_pcd_to_las_v12.py`
- **依存**: `numpy`, `laspy`
- **LAS バージョン**: 1.2 Format 0（最大互換性）
- **スケール**: 0.001（ミリメートル精度）
- **出力**: `merged_v12.las`（217 MB、14.2M 点）

```bash
pip install numpy laspy
python3 convert_pcd_to_las_v12.py
```

---

## PotreeConverter によるオクツリー生成

LAS ファイルを Potree 2.0 用のオクツリー構造に変換した。

```bash
PotreeConverter merged_v12.las -o potree_output --generate-page index -m BROTLI
```

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `-o` | `potree_output` | 出力ディレクトリ |
| `--generate-page` | `index` | 初期 HTML ページの自動生成 |
| `-m` | `BROTLI` | 圧縮方式（最高圧縮率） |

### 圧縮効果

| 項目 | サイズ |
|------|--------|
| 入力 LAS | 217 MB |
| octree.bin（BROTLI） | 64 MB |
| libs/ | 34 MB（最適化後） |
| **合計** | **98 MB** |

---

## Potree ビューアのカスタマイズ

PotreeConverter が生成した `index.html` を大幅にカスタマイズした。

### MapCleaner 分類色

RViz 設定ファイル `rviz/rviz.rviz` から各分類の表示色を抽出し、Potree の classification scheme に適用した。

| 分類 | 色（RGB） | 視覚的な色 |
|------|----------|-----------|
| Ground (2) | `0.94, 0.50, 0.17` | オレンジ |
| Terrain (4) | `0.49, 0.84, 0.24` | ライトグリーン |
| Static (6) | `0.94, 0.94, 0.94` | ほぼ白 |
| Ground Below (7) | `0.44, 0.77, 0.87` | 水色 |
| Dynamic (20) | `0.83, 0.14, 0.24` | 赤 |

### UI コントロール

1. **色モード切替ボタン**: Classification / Height / Intensity の3モード
2. **分類トグル**: 各分類の表示/非表示を個別に制御
3. **点サイズスライダー**: 0.01〜2.0 の範囲でリアルタイム変更

点サイズスライダーの実装:

```html
<div id="size_slider_container">
    <label>Size</label>
    <input type="range" id="point_size_slider" min="0.01" max="2.0" step="0.01" value="0.1">
    <span id="point_size_value">0.10</span>
</div>
```

```javascript
var slider = document.getElementById('point_size_slider');
slider.addEventListener('input', function() {
    var val = parseFloat(slider.value);
    document.getElementById('point_size_value').textContent = val.toFixed(2);
    var pc = viewer.scene.pointclouds[0];
    if (pc) pc.material.size = val;
});
```

---

## GitHub Pages 向けファイル構成

複数データセット（最大10個）を1つのリポジトリでホスティングする構成を設計した。

### ディレクトリ構造

```
potree_output/
├── index.html              # トップページ（データセットカード一覧）
├── viewer.html             # 共通ビューア（URLパラメータで切替）
├── libs/                   # Potree ライブラリ（34 MB、1コピー共有）
└── data/
    ├── hino_jt128/         # データセット1（64 MB）
    │   ├── metadata.json
    │   ├── hierarchy.bin
    │   └── octree.bin
    └── dataset_2/          # データセット2（追加時）
```

### URL パラメータ方式

`viewer.html?cloud=hino_jt128` のように URL パラメータでデータセットを切り替える。

```javascript
var params = new URLSearchParams(window.location.search);
var cloudName = params.get('cloud') || 'hino_jt128';
Potree.loadPointCloud("./data/" + cloudName + "/metadata.json", cloudName, callback);
```

### サイズ見積もり

| 構成要素 | サイズ |
|---------|--------|
| libs/（共有） | 34 MB |
| data/（1データセット） | ~64 MB |
| data/（10データセット） | ~640 MB |
| **合計（10データセット）** | **~674 MB** |

GitHub Pages の 1 GB 制限内に収まる。

---

## libs/ の最適化

Potree ライブラリから不要ファイルを削除し、49 MB → 34 MB に軽量化した。

| 削除対象 | サイズ | 理由 |
|---------|--------|------|
| `libs/potree/lazylibs/` | 11 MB | GeoPackage/SQL.js（未使用） |
| `libs/potree/potree.js.map` | 5 MB | ソースマップ（本番不要） |

### CDN 化の検討結果

CDN 化は以下の理由で見送った:

- Potree は `potree.js` のロード元パスから Workers/WASM を相対パスで動的読み込みする
- CDN 上の Potree は古いバージョン（2017年）のみで、現行 v2.0 と互換性がない

---

## 不要ファイルの整理

作業過程で生成されたテスト用ディレクトリ・ファイルを削除し、約 1.4 GB を削減した。

| 削除対象 | サイズ | 理由 |
|---------|--------|------|
| `potree_output_test/` | 415 MB | 初期テスト出力 |
| `potree_v12_uncomp/` | 333 MB | 非圧縮テスト |
| `potree_output_v211/` | 113 MB | v2.1.1 テスト |
| `potree_v12_brotli/` | 113 MB | BROTLI テスト |
| `merged.las` | 406 MB | LAS 1.4 版（v1.2 版で代替） |
| スクリーンショット類 | 13 MB | 作業用画像 |
| **合計** | **~1,393 MB** | |

---

## トラブルシューティング

### 1. ブラウザキャッシュによる初回読み込み失敗

- **症状**: ファイル構成変更後、ビューアが点群を読み込めない
- **原因**: ブラウザが旧パス（`./pointclouds/index/metadata.json`）をキャッシュしていた
- **解決**: Ctrl+Shift+R（キャッシュ無効リロード）で新パス（`./data/hino_jt128/metadata.json`）が正常に読み込まれた

### 2. LAS バージョン選定

- **問題**: LAS 1.4 Format 6 は PotreeConverter 2.1 で正常に処理されるが、ツール互換性に不安がある
- **解決**: LAS 1.2 Format 0（最大互換性）を採用。classification の 5-bit 制約に対応するため動的物体コード 64 → 20 にリマッピング

---

## データ追加手順

新しいデータセットを追加する際のワークフロー:

```bash
# 1. PCD → LAS 変換（convert_pcd_to_las_v12.py の INPUT_DIR を変更）
python3 convert_pcd_to_las_v12.py

# 2. PotreeConverter でオクツリー生成
PotreeConverter merged_v12.las -o temp_output -m BROTLI

# 3. data/ 配下にコピー
mkdir -p potree_output/data/<dataset_name>
cp temp_output/metadata.json temp_output/hierarchy.bin temp_output/octree.bin \
   potree_output/data/<dataset_name>/

# 4. index.html の DATASETS 配列にエントリ追加
```

---

## 最終成果物

| ファイル | 役割 |
|---------|------|
| `potree_output/index.html` | データセット一覧トップページ |
| `potree_output/viewer.html` | Potree ビューア（MapCleaner色・スライダー付き） |
| `potree_output/data/hino_jt128/` | hino_jt128 の点群データ（64 MB） |
| `potree_output/libs/` | Potree ライブラリ（34 MB） |
| `convert_pcd_to_las_v12.py` | PCD → LAS 変換スクリプト |
| `PROCEDURE.md` | 全体手順書 |
