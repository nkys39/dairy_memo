# Potree ワークフローのディレクトリ整理

## 概要

MapCleaner 出力から Potree ビューアへの変換ワークフローにおいて、データセット固有ディレクトリ（`hino_jt128_output/`）に共有ツール・ドキュメント・ビューアが混在していた問題を解消した。今後の新規データセット追加に対応できるよう、役割ごとにディレクトリを分離した。

---

## 整理前の問題

```
hino_jt128_output/           ← データセット固有名に依存
├── *.pcd (218MB)            ← MapCleaner生出力
├── merged_v12.las (271MB)   ← 中間ファイル（放置）
├── convert_pcd_to_las_v12.py ← 共有ツール（パスハードコード）
├── PROCEDURE.md             ← 共有ドキュメント
├── .venv/                   ← Python仮想環境
└── potree_output/ (98MB)    ← デプロイ対象
```

- 共有ツール・ドキュメントがデータセット固有ディレクトリに閉じ込められていた
- `convert_pcd_to_las_v12.py` の `INPUT_DIR` がハードコードされていた
- 中間 LAS ファイル（271MB）が放置されていた
- 新データセット追加時のファイル配置が不明確だった

---

## 整理後の構成

```
/home/user/map/
├── hino_jt128_output/       # MapCleaner出力（PCDのみ）
├── tools/                   # 共有ツール
│   ├── convert_pcd_to_las.py   # 引数化済み変換スクリプト
│   ├── run_pipeline.sh         # 一括変換パイプライン（新規）
│   ├── PROCEDURE.md            # 手順書
│   └── .venv/                  # Python仮想環境
├── staging/                 # 中間LASファイル置き場（使い捨て）
└── potree_viewer/           # GitHub Pagesデプロイ対象（98MB）
    ├── index.html / viewer.html / libs/ / data/
```

---

## 実施内容

### 1. ディレクトリ移動

| 移動元 | 移動先 | 理由 |
|--------|--------|------|
| `hino_jt128_output/potree_output/` | `potree_viewer/` | デプロイ対象をトップレベルに |
| `hino_jt128_output/convert_pcd_to_las_v12.py` | `tools/convert_pcd_to_las.py` | 共有ツールとして独立+リネーム |
| `hino_jt128_output/PROCEDURE.md` | `tools/PROCEDURE.md` | 共有ドキュメントとして独立 |

### 2. 変換スクリプトの引数化

- **ファイル**: `tools/convert_pcd_to_las.py`
- `INPUT_DIR` ハードコード → `argparse` で引数化
- 出力先をオプション `-o` で指定可能に（デフォルト: `staging/<dir_name>.las`）

```bash
# 使用例
python convert_pcd_to_las.py /home/user/map/hino_jt128_output
# → /home/user/map/staging/hino_jt128_output.las
```

### 3. パイプライン自動化スクリプト作成

- **ファイル**: `tools/run_pipeline.sh`（新規作成）
- PCD→LAS→PotreeConverter→data/配置→中間ファイル削除を1コマンドで実行

```bash
./run_pipeline.sh <dataset_name> /home/user/map/<xxx_output>
```

処理フロー:
1. `convert_pcd_to_las.py` で PCD → LAS（staging/ に出力）
2. `PotreeConverter` で LAS → オクツリー（/tmp に一時出力）
3. `potree_viewer/data/<name>/` に metadata.json, hierarchy.bin, octree.bin をコピー
4. 中間ファイル（LAS、一時ディレクトリ）を自動削除
5. `index.html` の DATASETS 配列編集を案内表示

### 4. Python 仮想環境の再作成

venv はパスがハードコードされるため、移動ではなく `tools/.venv` に再作成し、旧 `.venv` を削除。

### 5. 不要ファイルの削除

| 対象 | サイズ | 理由 |
|------|--------|------|
| `hino_jt128_output/merged_v12.las` | 271 MB | Potree変換済みで不要 |
| `hino_jt128_output/.venv/` | 82 MB | tools/ に再作成済み |
| `hino_jt128_output/.claude/` | 微小 | セッション設定（不要） |

### 6. PROCEDURE.md のパス更新

手順書内のすべてのパス参照を新構成に合わせて更新:
- スクリプト・venv 関連: `hino_jt128_output/` → `tools/`
- ビューア関連: `potree_output/` → `potree_viewer/`
- 新データセット追加手順に `run_pipeline.sh` の使い方を追記

---

## 新データセット追加ワークフロー

```bash
# 1. MapCleaner 実行（出力先は config で指定）
# 2. 一括変換（1コマンド）
cd /home/user/map/tools
./run_pipeline.sh tsukuba /home/user/map/tsukuba_output

# 3. index.html の DATASETS 配列にエントリ追加（手動）
# 4. ローカル確認
cd /home/user/map/potree_viewer && python3 -m http.server 8080
```

---

## 検証結果

- `potree_viewer/` でローカルサーバー起動 → `viewer.html?cloud=hino_jt128` が正常表示
- `du -sh potree_viewer/` → 98MB（想定通り）
- `hino_jt128_output/` に PCD ファイルのみ残存（217MB）
- `convert_pcd_to_las.py --help` が正常動作
