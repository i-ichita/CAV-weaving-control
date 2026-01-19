# CAV Weaving Zone Control System

Connected Autonomous Vehicle (CAV) のための織り込み区間（Weaving Section）における階層型制御システム。Apollo準拠のFrenet QP制御とUrgency-based MPCを統合。

## 主な特徴

- 階層型制御アーキテクチャ: Level 1 (戦略) + Level 2 (戦術)
- Apollo準拠のQP制御: ST-Boundary制約による衝突回避
- Urgency-based MPC: 動的安全マージン緩和
- ベイズ最適化対応: パラメータ自動チューニング

## ディレクトリ構造

```
CAV-weaving-control/
├── weaving_v11/           # メインパッケージ
│   ├── main.py            # エントリーポイント
│   ├── frenet_qp_apollo.py
│   ├── dp_speed_optimizer.py
│   ├── mpc_controller.py
│   └── ...
├── optimization/          # ベイズ最適化
│   ├── bayesian_opt_v11.py
│   └── best_params_v11_*.json
├── references/            # Apollo参考コード
└── docs/                  # ドキュメント
```

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
