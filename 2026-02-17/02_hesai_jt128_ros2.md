# Hesai JT128 + ROS2 Jazzy 詳細

## 初期セットアップ
- HesaiLidar_ROS_2.0 ドライバのビルド・起動確認
- config.yaml の全パラメータ解説
- device_ip_address はLiDAR本体のIP（192.168.1.201）
- IPアドレス変更方法（LiDAR側 Web UI + PC側ネットワーク設定）

## IMUデータ解析
- IMU周期: ~200Hz（ジャイロ/加速度が~0.112msオフセットでインターリーブ）
- 静止データ・振動データの両方を解析
- FastLIO互換性: sensor_msgs/Imu 200Hz出力、座標系確認とcovariance=0に注意

## PointCloud パフォーマンス問題
- subscriber複数接続時に10Hz → 2Hzに低下 → iceoryx共有メモリで解決 → [詳細](03_iceoryx_shared_memory.md)

## config.yaml Q&A
- 距離フィルタ: config.yamlには存在しない（FOV/チャンネルフィルタのみ）
- echo_mode_filter: デュアルリターン時のエコー選択（0=全取得）
- デュアルリターン: LiDAR本体設定（Web UI / PTC SetReturnMode）
- multicast: multicast_ip_address を空文字で無効化
- PC時刻: use_timestamp_type を1に設定

## 変更ファイル
- `config/config.yaml`: multicast OFF, PC時刻に変更
- iceoryx / CycloneDDS関連 → [03_iceoryx_shared_memory.md](03_iceoryx_shared_memory.md) を参照
