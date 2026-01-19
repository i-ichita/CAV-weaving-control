# CAV Weaving Zone Control System

Connected Autonomous Vehicle (CAV) のための織り込み区間（Weaving Section）における階層型制御システム。Apollo準拠のFrenet QP制御とUrgency-based MPCを統合。

## 主な特徴

- **分散自律型制御**: 各車両が独立してQPを解く（中央サーバーなし）
- **V2V協調制御**: 軌道共有 + LC協調リクエスト + コストベース仲裁
- **階層型アーキテクチャ**: Level 1 (戦略: Urgency-MPC) + Level 2 (戦術: Frenet QP)
- **Apollo準拠**: ST-Boundary制約、Piecewise Jerk QP、RSS安全距離
- **混合交通対応**: CAV + HDVの共存環境をサポート
- **ベイズ最適化**: パラメータ自動チューニング

## ディレクトリ構造

```
CAV-weaving-control/
├── weaving_v11/           # メインパッケージ
│   ├── main.py            # エントリーポイント
│   ├── controllers.py     # 階層型制御・V2V協調
│   ├── frenet_qp_apollo.py # QP最適化・ST-Boundary
│   ├── mpc_controller.py  # Urgency計算
│   ├── vehicle.py         # 車両状態・軌道共有
│   └── ...
├── optimization/          # ベイズ最適化
│   ├── bayesian_opt_v11.py
│   └── best_params_v11_*.json
├── references/            # Apollo参考コード
└── docs/                  # ドキュメント（下記参照）
```

## ドキュメント

| ファイル | 内容 | 対象読者 |
|---------|------|---------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | システム全体構造、V2V協調の概要 | 開発者 |
| [docs/ALGORITHMS.md](docs/ALGORITHMS.md) | 数式・アルゴリズムの詳細 | 研究者 |
| [docs/API_REFERENCE.md](docs/API_REFERENCE.md) | クラス・メソッド一覧 | 開発者 |
| [docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md) | 実装ガイド・タスク例 | 開発者 |
| [docs/APOLLO_ALIGNMENT.md](docs/APOLLO_ALIGNMENT.md) | Apollo準拠の詳細 | 研究者 |

## クイックスタート

```bash
# 依存関係のインストール
pip install -r requirements.txt

# テスト実行
python -m weaving_v11.main --mode test

# シミュレーション実行
python -m weaving_v11.main --mode sim --load medium --tmax 600
```

## ライセンス

MIT License

## 連絡先

- IIZAKA ICHITA
- 横浜国立大学 交通と都市研究室
