# iceoryx共有メモリによるPointCloud配信パフォーマンス改善

## 問題

- **症状**: Hesai JT128のPointCloudトピックにsubscriberが2個以上接続すると、出力周期が10Hz → 2Hzに低下
- **環境**: ROS2 Jazzy + CycloneDDS + Hesai JT128 (3D LiDAR)
- **データ量**: 1フレーム 230,400点 × 26B/点 = **約5.7MB/フレーム**

---

## 根本原因

ROS2のデフォルト通信（CycloneDDS）では、publish時に以下の処理が発生する:

1. **CDRシリアライゼーション**: PointCloud2メッセージを各subscriber向けにシリアライズ（subscriberの数だけコピー発生）
2. **UDPフラグメンテーション**: 5.7MBのメッセージをUDPの最大ペイロード（~65KB）に分割して送信
3. **再組立て**: subscriber側で数十〜数百のUDPフラグメントを再組立て

subscriberが増えるほどシリアライゼーションとUDP送信のコストが線形に増加し、10Hz（100ms周期）の制約を超過する。

---

## 試した対策と結果

| 対策 | 設定 | 結果 |
|------|------|------|
| UDPソケットバッファ拡大 | `sysctl rmem_default 64MB` | 効果なし |
| CycloneDDS書き込みキャッシュ拡大 | `WhcHigh 64MB` | 効果なし |
| QoSをBEST_EFFORTに変更 | `ReliabilityPolicy::BEST_EFFORT` | 効果なし |

いずれもUDP転送のオーバーヘッド自体は解消できないため、根本的な解決にならなかった。

---

## 解決策: iceoryx共有メモリ (zero-copy IPC)

### 仕組み

iceoryxはプロセス間で**共有メモリ**を使ってメッセージを受け渡すミドルウェアである。CycloneDDSと統合することで、同一マシン内の通信を自動的にzero-copy IPCに切り替える。

```
通常のDDS通信:
  Publisher → CDRシリアライズ → UDPフラグメント → ネットワーク → 再組立て → デシリアライズ → Subscriber

iceoryx共有メモリ:
  Publisher → 共有メモリに書き込み → Subscriber（ポインタ参照のみ、コピーなし）
```

- subscriberが何個接続してもデータコピーが発生しない
- シリアライゼーション・UDPフラグメンテーションを完全にバイパス
- **ドライバコードの変更不要**（CycloneDDSが自動的にiceoryxを使用）

### 結果

- **iceoryx有効時**: subscriber複数接続でも **10Hz維持**
- **RELIABLE QoS + iceoryx**: 10Hz維持（BEST_EFFORTに変更する必要なし）
- **rosbag record/play**: iceoryx環境でも正常動作（16秒間: PointCloud 161msg ~10Hz + IMU 6,787msg ~424Hz）

---

## 設定ファイル

3ファイルを `~/cyclonedds_config/` に配置する。

### `cyclonedds_config_shm.xml`（iceoryx ON版）

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <SocketReceiveBufferSize min="64MB"/>
            <Watermarks>
                <WhcHigh>64MB</WhcHigh>
            </Watermarks>
        </Internal>
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```

- **`<SharedMemory><Enable>true</Enable>`** がiceoryx有効化のキー設定

### `cyclonedds_config.xml`（iceoryx OFF版・通常版）

上記と同一だが `<Enable>false</Enable>` のみ異なる。iceoryxを使わない通常のDDS通信用。

### `roudi_config.toml`（RouDi mempool設定）

```toml
[general]
version = 1

[[segment]]

# --- 小メッセージ用（DDS制御、ディスカバリ、tf等） ---
[[segment.mempool]]
size = 128          # 128 B × 10,000個
count = 10000

[[segment.mempool]]
size = 1024         # 1 KB × 5,000個
count = 5000

# --- 中メッセージ用（IMU sensor_msgs/Imu ~500B, 小さいトピック） ---
[[segment.mempool]]
size = 16384        # 16 KB × 1,000個
count = 1000

[[segment.mempool]]
size = 131072       # 128 KB × 200個
count = 200

# --- 大メッセージ用（小〜中規模の点群） ---
[[segment.mempool]]
size = 524288       # 512 KB × 50個
count = 50

[[segment.mempool]]
size = 1048576      # 1 MB × 30個
count = 30

[[segment.mempool]]
size = 4194304      # 4 MB × 10個
count = 10

# --- JT128 PointCloud2用（~5.7MB/フレーム → 8MBチャンクが必要） ---
[[segment.mempool]]
size = 8388608      # 8 MB × 20個
count = 20
```

---

## RouDi mempool設計

iceoryxではRouDi（ルーティングデーモン）が共有メモリプールを事前確保して管理する。publisherがメッセージを送信する際、メッセージサイズに合うプール（`size` が足りる最小のチャンク）からメモリブロックを1つ借り、subscriber側はそのポインタを参照する。

### mempoolの仕組み

- **`size`**: 1チャンクのバイト数。メッセージがこのサイズ以下であれば、このプールから割り当てられる
- **`count`**: そのサイズのチャンクを何個事前確保するか。同時に使用可能な上限数に相当する
- iceoryxはメッセージサイズに対して **`size` が足りる最小のプール** を自動選択する
- どのプールにも収まらないメッセージ → **RouDiクラッシュ**

### チャンクサイズ階層の設計意図

| プール (size) | 用途例 | count の根拠 |
|--------------|--------|-------------|
| 128 B | DDS内部制御メッセージ、ディスカバリ | 大量に飛ぶため多めに確保 |
| 1 KB | 小型トピック（tf, パラメータ通知等） | 同上 |
| 16 KB | sensor_msgs/Imu (~500B)、診断メッセージ | IMUが200Hz+で発行されるため |
| 128 KB | 中規模トピック | 汎用バッファ |
| 512 KB〜4 MB | 小〜中規模の点群、画像 | 汎用バッファ |
| **8 MB** | **JT128 PointCloud2 (~5.7MB)** | **この用途のために追加** |

### JT128で8MBが必要な理由

- **PointCloud2のシリアライズ後サイズ**: 230,400点 × (3×float + その他フィールド) + ヘッダ → **約4.4〜5.7MB**
- **iceoryxデフォルトの最大チャンク**: 4MB → メッセージが収まらず **クラッシュ**
- **対処**: 8MB × 20個のプールを追加。`count = 20` は同時にpublisher/subscriberが保持するバッファ数に余裕を持たせた値

---

## 使い方

### 1. RouDi起動

iceoryx使用時はRouDiデーモンを先に起動する必要がある:

```bash
iox-roudi -c ~/cyclonedds_config/roudi_config.toml
```

### 2. CYCLONEDDS_URI切替

`.bashrc` で環境変数を切り替える:

```bash
# CycloneDDS設定ファイル (通常版: iceoryx OFF / 共有メモリ版: iceoryx ON)
# 共有メモリ版を使う場合は先にRouDiを起動: iox-roudi -c ~/cyclonedds_config/roudi_config.toml
export CYCLONEDDS_URI=$HOME/cyclonedds_config/cyclonedds_config.xml
# export CYCLONEDDS_URI=$HOME/cyclonedds_config/cyclonedds_config_shm.xml
```

iceoryxを有効にする場合は `cyclonedds_config_shm.xml` の行のコメントを外し、通常版の行をコメントアウトする。

---

## 変更ファイル一覧

| ファイル | 変更内容 |
|----------|----------|
| `~/cyclonedds_config/cyclonedds_config.xml` | 新規作成 (iceoryx OFF) |
| `~/cyclonedds_config/cyclonedds_config_shm.xml` | 新規作成 (iceoryx ON) |
| `~/cyclonedds_config/roudi_config.toml` | 新規作成 (8MB mempool) |
| `~/.bashrc` | sysctl, CYCLONEDDS_URI切替追加 |
| `src/manager/source_driver_ros2.hpp` | 変更なし（一時変更→元に戻した） |
