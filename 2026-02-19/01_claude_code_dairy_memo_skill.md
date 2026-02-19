# Claude Code カスタムスキル `/dairy-memo` の作成

## 概要

Claude Code のセッション作業内容を dairy_memo リポジトリに自動記録するカスタムスキル `/dairy-memo` を作成した。セッション終了時にスラッシュコマンドを実行すると、STYLE_GUIDE.md に従った日記メモが生成される。

---

## スキル設計

### ディレクトリ構造

```
~/.claude/skills/dairy-memo/
├── SKILL.md                    # スキル定義（YAML frontmatter + 実行手順）
└── references/
    └── STYLE_GUIDE.md          # dairy_memo/STYLE_GUIDE.md のコピー
```

### SKILL.md の構成

- **YAML Frontmatter**: `name: dairy-memo`、`description` でスキルの用途を記述
- **Markdown Body**: 6ステップの実行手順
  1. セッション分析（トピック列挙・タイプA〜D分類・ファイル名決定）
  2. ユーザ確認（作成計画の提示と承認）
  3. 日付ディレクトリ確認（新規 or 既存追加の判定）
  4. ファイル作成（詳細ファイル → サマリー → ルートREADME の順序厳守）
  5. コンテンツ生成ルール（日本語・である調・スタイルガイド準拠）
  6. Git コミット（オプション）

### スキル名の規約

- Claude Code のスキル名は **lowercase + hyphens** が規約
- `dairy_memo`（アンダースコア）ではなく `dairy-memo`（ハイフン）を採用
- 呼び出し: `/dairy-memo`

---

## 作成手順

1. ディレクトリ作成: `mkdir -p ~/.claude/skills/dairy-memo/references/`
2. `SKILL.md` を作成（YAML frontmatter + 実行手順の Markdown）
3. `STYLE_GUIDE.md` を `references/` にコピー

---

## トラブルシューティング

### 「Unknown skill: dairy-memo」エラー

- **症状**: スキル作成直後に `/dairy-memo` を実行すると `Unknown skill` と表示される
- **原因**: VSCode 拡張がスキルディレクトリの変更を検出していなかった
- **解決**: `Ctrl+Shift+P` → `Developer: Reload Window` で VSCode 拡張をリロード

### 参考: スキルの配置場所

| レベル | パス | スコープ |
|--------|------|----------|
| ユーザレベル | `~/.claude/skills/<name>/SKILL.md` | 全プロジェクト共通 |
| プロジェクトレベル | `.claude/skills/<name>/SKILL.md` | 該当プロジェクトのみ |

今回はどのプロジェクトからでも呼び出せるようユーザレベル（`~/.claude/skills/`）に配置した。

---

## references/ の運用

- `references/STYLE_GUIDE.md` は `/home/user/Documents/dairy_memo/STYLE_GUIDE.md` のコピー
- SKILL.md 内で「元ファイルがアクセス可能ならそちらを優先参照」と指示しているため、dairy_memo リポジトリ内での作業時は最新版が自動的に使われる
- STYLE_GUIDE.md を更新した場合は `references/` のコピーも再同期が必要
