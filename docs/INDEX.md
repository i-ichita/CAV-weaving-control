# ドキュメント索引

> **CAV-Weaving-Control プロジェクトドキュメント一覧**
>
> **最終更新**: 2026-01-23
> **プロジェクト**: Urgency誘導型分散協調制御システム

---

## 📚 ドキュメント構成

本プロジェクトのドキュメントは以下の構成になっています：

```
docs/
├── INDEX.md                          # このファイル（索引）
├── BACKGROUND_AND_OBJECTIVES.md      # 研究背景と目的 ⭐ 最初に読む
├── LITERATURE_REVIEW.md              # 文献レビュー
├── ARCHITECTURE.md                   # システムアーキテクチャ
├── ALGORITHMS.md                     # アルゴリズム詳細
├── APOLLO_ALIGNMENT.md               # Apolloとの対応関係
├── IMPLEMENTATION.md                 # 実装詳細
├── API_REFERENCE.md                  # API仕様
└── references/                       # 参考文献PDF
```

---

## 📖 ドキュメント詳細

### 1. [BACKGROUND_AND_OBJECTIVES.md](BACKGROUND_AND_OBJECTIVES.md) ⭐ **最初に読む**

**対象読者**: 研究者・論文執筆者・プロジェクト関係者

**内容**:
- **研究の正確な定義**: 分散協調制御（分散最適化ではない）
- **3つの核心要素**: Urgency関数、V2V軌道共有、ST-Boundary QP
- **研究背景**: 織り込み区間の問題（Front-loading、Capacity Drop）
- **研究目的**: 3つの検証課題（空間分散、スケーラビリティ、安全性）
- **研究の独自性**: 先行研究との差異
- **Apolloとの関係**: 参考にした要素とオリジナル要素
- **用語の統一**: 論文執筆用の用語定義
- **研究の進め方**: Phase 1-6のワークフロー

**主要トピック**:
```
✅ 研究の正確な定義
✅ Front-loading問題
✅ Urgency関数の設計
✅ V2V協調制御
✅ 階層型制御アーキテクチャ
✅ Apolloとの関係（参考 vs 拡張）
```

---

### 2. [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md)

**対象読者**: 研究者・論文執筆者

**内容**:
- **A. 織り込み区間の交通流特性**: Tilg2018, Tanaka2017
- **B. 制御アルゴリズムと最適化手法**: 中央集権型、学習型、最適化型
- **C. CAVの意義**: V2V通信による協調制御
- **D. Apolloアーキテクチャ**: ST-Boundary、QP最適化
- **E. 安全性保証**: RSS、ISO規格

**主要トピック**:
```
✅ Capacity Drop現象
✅ Front-loading問題の文献
✅ 中央集権型の限界（Zhang, Peng）
✅ 学習ベースの課題（Yan, xi）
✅ Apollo参考手法
```

**参考文献数**: 20+ 論文

---

### 3. [ARCHITECTURE.md](ARCHITECTURE.md)

**対象読者**: 実装者・システム設計者

**内容**:
- **システム概要**: 階層型制御アーキテクチャ
- **Level 1 (戦略層)**: UrgencyPlanner（0.5秒周期）
- **Level 2 (戦術層)**: FrenetQPController（0.1秒周期）
- **V2V協調フロー**: 軌道共有のメカニズム
- **制御フロー図**: 各コンポーネントの関係

**主要トピック**:
```
✅ 階層型制御の設計思想
✅ Level 1 vs Level 2の役割分担
✅ V2V通信プロトコル
✅ QP最適化のフロー
```

---

### 4. [ALGORITHMS.md](ALGORITHMS.md) 🔬 **アルゴリズム詳細**

**対象読者**: 研究者・実装者

**内容**:
- **Urgency関数**: 数式、パラメータ設計、実装
- **V2V軌道共有**: プロトコル、データ形式
- **ST-Boundary構築**: 時空間制約の計算
- **Piecewise-Jerk QP**: 定式化、ソルバー設定
- **RSS安全距離**: 計算式、CAV対応
- **Gap Acceptance**: 車線変更判定ロジック

**主要トピック**:
```
✅ U(s, ρ) = min(1.0, (s_norm)^γ + α·ρ_target)
✅ γ=3.0の設計根拠
✅ ST-Boundaryの数理的定義
✅ QP目的関数と制約
✅ RSSパラメータ（0.15s vs 0.5s）
```

**数式・疑似コード**: 豊富に含まれる

---

### 5. [APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md)

**対象読者**: 実装者・Apollo経験者

**内容**:
- **Apolloとの対応表**: C++ファイルとPythonファイルのマッピング
- **コンポーネント詳細**: PlanningComponent、PathDecider、SpeedBoundsDecider等
- **パラメータ対応**: Apollo標準値との比較
- **検証マトリクス**: 各機能の実装状況確認

**主要トピック**:
```
✅ PlanningComponent → IntegratedZoneController
✅ SpeedBoundsDecider → ST-Boundary構築
✅ PiecewiseJerkSpeedOptimizer → FrenetQPController
✅ Apollo C++ファイルの参照先
```

**Apollo準拠度**: 95%+（意図的な差異を除く）

---

### 6. [IMPLEMENTATION.md](IMPLEMENTATION.MD)

**対象読者**: 実装者・デバッグ担当

**内容**:
- **ファイル構成**: weaving_v11/の各ファイルの役割
- **クラス設計**: 主要クラスの責務
- **データフロー**: シミュレーション1ステップの流れ
- **デバッグガイド**: よくある問題と解決法

**主要トピック**:
```
✅ main.py: エントリーポイント
✅ simulator.py: シミュレーションループ
✅ controllers.py: 統合制御ロジック
✅ frenet_qp_apollo.py: QP最適化
✅ mpc_controller.py: Urgency計算
```

---

### 7. [API_REFERENCE.md](API_REFERENCE.md)

**対象読者**: 実装者・拡張開発者

**内容**:
- **主要クラスのAPI**: コンストラクタ、メソッド、パラメータ
- **データクラス**: VehicleState、ObstacleInfo、STBoundary等
- **関数リファレンス**: 各モジュールの公開関数

**主要トピック**:
```
✅ IntegratedZoneController API
✅ FrenetQPController API
✅ UrgencyPlanner API
✅ Vehicle クラス
```

---

## 🎯 目的別の読み方

### 🎓 研究全体を理解したい

| 順序 | ドキュメント | 読む内容 | 所要時間 |
|-----|------------|---------|---------|
| 1 | [BACKGROUND_AND_OBJECTIVES.md](BACKGROUND_AND_OBJECTIVES.md) | 研究の定義、背景、3つの核心要素 | 30分 |
| 2 | [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) | 先行研究との差異、Research Gap | 20分 |
| 3 | [ALGORITHMS.md](ALGORITHMS.md) | Urgency関数、QP定式化、数式 | 40分 |

**合計**: 約1.5時間

---

### 💻 実装を理解したい

| 順序 | ドキュメント | 読む内容 | 所要時間 |
|-----|------------|---------|---------|
| 1 | [ARCHITECTURE.md](ARCHITECTURE.md) | システム設計、階層型制御 | 25分 |
| 2 | [IMPLEMENTATION.md](IMPLEMENTATION.md) | ファイル構成、データフロー | 30分 |
| 3 | [APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md) | Apolloとの対応、C++↔Python | 20分 |
| 4 | [API_REFERENCE.md](API_REFERENCE.md) | クラス・メソッド仕様 | 15分 |

**合計**: 約1.5時間

**推奨**: コードと並行して読む

---

### 📝 論文を書きたい

| 順序 | ドキュメント | 使用箇所 | 注意点 |
|-----|------------|---------|-------|
| 1 | [BACKGROUND_AND_OBJECTIVES.md](BACKGROUND_AND_OBJECTIVES.md) | Introduction, Background, Objectives | 用語の統一を確認 |
| 2 | [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) | Related Work | 引用文献リスト |
| 3 | [ALGORITHMS.md](ALGORITHMS.md) | Proposed Method | 数式をそのまま使用可 |
| 4 | [BACKGROUND_AND_OBJECTIVES.md - Phase 4](BACKGROUND_AND_OBJECTIVES.md#phase-4-効果検証) | Evaluation | 評価指標の定義 |

**重要**:
- [用語の統一](BACKGROUND_AND_OBJECTIVES.md#6-用語の統一)を必ず確認
- 「分散最適化」ではなく「分散協調制御」を使用

---

### 🔍 Apolloとの関係を知りたい

| ドキュメント | セクション | 内容 |
|------------|-----------|------|
| [BACKGROUND_AND_OBJECTIVES.md](BACKGROUND_AND_OBJECTIVES.md#5-apolloとの関係) | セクション5 | 概要、借用要素、オリジナル要素 |
| [APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md) | 全体 | 詳細な対応表、パラメータ比較 |

**要約**:
- **参考にした**: Piecewise-Jerk QP、ST-Boundary、RSS距離
- **オリジナル**: Urgency関数、V2V軌道共有、協調LC要求

---

### 🐛 トラブルシューティング

| 問題 | 確認箇所 | ドキュメント |
|-----|---------|------------|
| QP最適化失敗 | ST-Boundary制約、車両密度 | [IMPLEMENTATION.md](IMPLEMENTATION.md) |
| 衝突発生 | RSS安全距離、V2V有効性 | [ALGORITHMS.md](ALGORITHMS.md) |
| パラメータ調整 | γ, α, qp_w_j, qp_w_a | [ALGORITHMS.md](ALGORITHMS.md)<br>[API_REFERENCE.md](API_REFERENCE.md) |
| ログ分析 | `outputs/simulation_log_*.txt` | [IMPLEMENTATION.md](IMPLEMENTATION.md) |

---

### ⚡ クイックスタート（5分）

```bash
# 1. 依存関係インストール
pip install -r requirements.txt

# 2. テスト実行
python -m weaving_v11.main --mode test

# 3. デモ実行（60秒）
python -m weaving_v11.main --mode demo --tmax 60

# 4. 本格シミュレーション（HIGH負荷、600秒）
python -m weaving_v11.main --mode sim --load high --tmax 600
```

詳細: [IMPLEMENTATION.md](IMPLEMENTATION.md)

---

## 📊 ドキュメント統計

| ファイル | 行数 | サイズ | 主要内容 | 更新頻度 |
|---------|------|--------|---------|---------|
| BACKGROUND_AND_OBJECTIVES.md | 500+ | 25KB | 研究の定義・目的 | 🟢 安定 |
| ALGORITHMS.md | 2096 | 94KB | アルゴリズム詳細 | 🟢 安定 |
| APOLLO_ALIGNMENT.md | 1013 | 35KB | Apollo対応 | 🟢 安定 |
| ARCHITECTURE.md | 691 | 28KB | システム設計 | 🟢 安定 |
| IMPLEMENTATION.md | 1300 | 39KB | 実装詳細 | 🟡 更新中 |
| LITERATURE_REVIEW.md | 481 | 35KB | 文献レビュー | 🟢 安定 |
| API_REFERENCE.md | 623 | 16KB | API仕様 | 🟡 更新中 |

**合計**: 約 7000行、270KB のドキュメント

**凡例**:
- 🟢 安定: 大きな変更なし
- 🟡 更新中: 実装に伴い更新される可能性あり

---

## 🔗 ドキュメント間の相互参照マップ

```
BACKGROUND_AND_OBJECTIVES.md (中心ドキュメント)
├─→ LITERATURE_REVIEW.md (先行研究の詳細)
├─→ ARCHITECTURE.md (システム設計の詳細)
├─→ ALGORITHMS.md (数式・アルゴリズム詳細)
└─→ APOLLO_ALIGNMENT.md (Apollo対応の詳細)

LITERATURE_REVIEW.md
└─→ BACKGROUND_AND_OBJECTIVES.md (研究の位置づけ)

ARCHITECTURE.md
├─→ ALGORITHMS.md (アルゴリズムの実装)
├─→ IMPLEMENTATION.md (実装詳細)
└─→ API_REFERENCE.md (APIドキュメント)

ALGORITHMS.md
├─→ APOLLO_ALIGNMENT.md (Apolloとの対応)
└─→ IMPLEMENTATION.md (実装例)

APOLLO_ALIGNMENT.md
├─→ ALGORITHMS.md (アルゴリズム詳細)
└─→ IMPLEMENTATION.md (実装詳細)

IMPLEMENTATION.md
├─→ API_REFERENCE.md (関数リファレンス)
└─→ ARCHITECTURE.md (システム全体像)

API_REFERENCE.md
└─→ IMPLEMENTATION.md (使用例)
```

### 各ドキュメントの参照関係

| ドキュメント | 参照先 | 被参照元 |
|------------|-------|---------|
| BACKGROUND_AND_OBJECTIVES.md | 4件 | 2件 |
| LITERATURE_REVIEW.md | 1件 | 1件 |
| ARCHITECTURE.md | 3件 | 2件 |
| ALGORITHMS.md | 2件 | 3件 |
| APOLLO_ALIGNMENT.md | 2件 | 2件 |
| IMPLEMENTATION.md | 2件 | 3件 |
| API_REFERENCE.md | 1件 | 2件 |

---

## 🔍 キーワード索引

### A

**Apollo**
- [BACKGROUND_AND_OBJECTIVES.md - セクション5「Apolloとの関係」](BACKGROUND_AND_OBJECTIVES.md#5-apolloとの関係)
- [APOLLO_ALIGNMENT.md - 完全な対応表](APOLLO_ALIGNMENT.md)
- [ARCHITECTURE.md - Apollo準拠の設計](ARCHITECTURE.md)

### B

**Bayesian Optimization（ベイズ最適化）**
- [BACKGROUND_AND_OBJECTIVES.md - Phase 3](BACKGROUND_AND_OBJECTIVES.md#phase-3-パラメータ設定)
- 実装: `optimization/bayesian_opt_v11.py`

### C

**CAV（Connected Autonomous Vehicle）**
- [LITERATURE_REVIEW.md - CAVを用いる意義](LITERATURE_REVIEW.md#3-cavを用いる意義)
- [BACKGROUND_AND_OBJECTIVES.md - 研究目的](BACKGROUND_AND_OBJECTIVES.md#2-研究目的-objective)

**Capacity Drop（容量低下）**
- [BACKGROUND_AND_OBJECTIVES.md - 織り込み区間の問題](BACKGROUND_AND_OBJECTIVES.md#1-織り込み区間における交通課題)
- [LITERATURE_REVIEW.md - Tilg2018](LITERATURE_REVIEW.md)

### F

**Front-loading（前方集中）**
- [BACKGROUND_AND_OBJECTIVES.md - 研究背景](BACKGROUND_AND_OBJECTIVES.md#1-研究背景-background)
- [LITERATURE_REVIEW.md - Tanaka2017](LITERATURE_REVIEW.md)
- [ALGORITHMS.md - Urgency関数による解消](ALGORITHMS.md)

**Frenet座標系**
- [ALGORITHMS.md - 座標変換](ALGORITHMS.md)
- [APOLLO_ALIGNMENT.md - PathDecider](APOLLO_ALIGNMENT.md#2-path-decider)
- 実装: `weaving_v11/coordinate_transform.py`

### G

**Gap Acceptance（ギャップ受容）**
- [ALGORITHMS.md - LC判定ロジック](ALGORITHMS.md)
- [APOLLO_ALIGNMENT.md - LaneChangeDecider](APOLLO_ALIGNMENT.md#6-lane-change-decider)
- 実装: `weaving_v11/controllers.py:296-395`

**Gini係数（空間分散の評価）**
- [BACKGROUND_AND_OBJECTIVES.md - 評価指標](BACKGROUND_AND_OBJECTIVES.md#phase-4-効果検証)

### L

**Level 1 / Level 2（階層型制御）**
- [BACKGROUND_AND_OBJECTIVES.md - 階層型アーキテクチャ](BACKGROUND_AND_OBJECTIVES.md#2-分散自律型階層型制御アーキテクチャの構築)
- [ARCHITECTURE.md - システム概要](ARCHITECTURE.md)
- Level 1実装: `weaving_v11/mpc_controller.py`
- Level 2実装: `weaving_v11/frenet_qp_apollo.py`

### P

**Piecewise-Jerk QP**
- [BACKGROUND_AND_OBJECTIVES.md - QP定式化](BACKGROUND_AND_OBJECTIVES.md#3-数学的な安全性保証と実用性の両立)
- [ALGORITHMS.md - 数式詳細](ALGORITHMS.md)
- [APOLLO_ALIGNMENT.md - Apollo対応](APOLLO_ALIGNMENT.md#4-piecewise-jerk-speed-optimizer)
- 実装: `weaving_v11/frenet_qp_apollo.py:558-750`

### Q

**QP（二次計画法）**
- [ALGORITHMS.md - 定式化](ALGORITHMS.md)
- ソルバー: OSQP

### R

**RSS（Responsibility-Sensitive Safety）**
- [BACKGROUND_AND_OBJECTIVES.md - 安全性保証](BACKGROUND_AND_OBJECTIVES.md#3-数学的な安全性保証と実用性の両立)
- [ALGORITHMS.md - 安全距離計算](ALGORITHMS.md)
- [APOLLO_ALIGNMENT.md - Safety Manager](APOLLO_ALIGNMENT.md#8-safety-manager)
- 実装: `weaving_v11/apollo_safety.py:45-51`

### S

**ST-Boundary（時空間境界）**
- [BACKGROUND_AND_OBJECTIVES.md - ST-Boundary制約](BACKGROUND_AND_OBJECTIVES.md#3-数学的な安全性保証と実用性の両立)
- [ALGORITHMS.md - 構築方法](ALGORITHMS.md)
- [APOLLO_ALIGNMENT.md - SpeedBoundsDecider](APOLLO_ALIGNMENT.md#3-speed-bounds-decider)
- 実装: `weaving_v11/frenet_qp_apollo.py:867-975`

### U

**Urgency関数（切迫度関数）**
- [BACKGROUND_AND_OBJECTIVES.md - 核心要素](BACKGROUND_AND_OBJECTIVES.md#3つの核心要素)
- [ALGORITHMS.md - 数式とパラメータ](ALGORITHMS.md)
- 数式: `U(s, ρ) = min(1.0, (s_norm)^γ + α·ρ_target)`
- 実装: `weaving_v11/mpc_controller.py:114-170`

### V

**V2V軌道共有**
- [BACKGROUND_AND_OBJECTIVES.md - 3つの核心要素](BACKGROUND_AND_OBJECTIVES.md#3つの核心要素)
- [ARCHITECTURE.md - V2V協調フロー](ARCHITECTURE.md)
- [ALGORITHMS.md - プロトコル](ALGORITHMS.md)
- 実装: `weaving_v11/vehicle.py:662`

### その他

**分散協調制御 vs 分散最適化**
- [BACKGROUND_AND_OBJECTIVES.md - 研究の正確な定義](BACKGROUND_AND_OBJECTIVES.md#研究の正確な定義)
- **重要**: 本研究は分散協調制御（ADMM等の反復合意は行わない）

**織り込み区間（Weaving Section）**
- [BACKGROUND_AND_OBJECTIVES.md - 研究背景](BACKGROUND_AND_OBJECTIVES.md#1-研究背景-background)
- [LITERATURE_REVIEW.md - 交通流特性](LITERATURE_REVIEW.md)

---

## 💻 実装ファイル クイックリンク

### 主要Pythonファイル

| ファイル | 役割 | 行数 | ドキュメント |
|---------|------|------|-------------|
| [main.py](../weaving_v11/main.py) | エントリーポイント | 543 | [IMPLEMENTATION.md](IMPLEMENTATION.md) |
| [controllers.py](../weaving_v11/controllers.py) | 統合制御ロジック | 2,645 | [ARCHITECTURE.md](ARCHITECTURE.md) |
| [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py) | QP最適化・ST-Boundary | 3,567 | [ALGORITHMS.md](ALGORITHMS.md)<br>[APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md#4-piecewise-jerk-speed-optimizer) |
| [mpc_controller.py](../weaving_v11/mpc_controller.py) | Urgency計算 | 551 | [ALGORITHMS.md](ALGORITHMS.md) |
| [apollo_safety.py](../weaving_v11/apollo_safety.py) | 安全管理 | 931 | [APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md#8-safety-manager) |
| [vehicle.py](../weaving_v11/vehicle.py) | 車両状態・軌道共有 | 840 | [API_REFERENCE.md](API_REFERENCE.md) |
| [simulator.py](../weaving_v11/simulator.py) | シミュレーションループ | 2,879 | [ARCHITECTURE.md](ARCHITECTURE.md) |
| [parameters.py](../weaving_v11/parameters.py) | パラメータ設定 | 603 | [API_REFERENCE.md](API_REFERENCE.md) |

### パラメータ・設定ファイル

| ファイル | 内容 | 用途 |
|---------|------|------|
| [best_params_v11_high.json](../optimization/best_params_v11_high.json) | HIGH負荷用最適パラメータ | シミュレーション実行 |
| [best_params_v11_medium.json](../optimization/best_params_v11_medium.json) | MEDIUM負荷用最適パラメータ | シミュレーション実行 |
| [bayesian_opt_v11.py](../optimization/bayesian_opt_v11.py) | ベイズ最適化スクリプト | パラメータチューニング |

### 出力ファイル

| ディレクトリ/ファイル | 内容 |
|-------------------|------|
| `outputs/simulation_log_v11_*.txt` | シミュレーションログ |
| `weaving_v11_opt_high.db` | Optunaデータベース（最適化結果） |

---

## ❓ よくある質問（FAQ）

### 研究について

**Q1: この研究は「分散最適化」ですか？**
- A: いいえ。**分散協調制御**です。ADMM等の反復合意は行っていません。
- 詳細: [BACKGROUND_AND_OBJECTIVES.md - 研究の正確な定義](BACKGROUND_AND_OBJECTIVES.md#研究の正確な定義)

**Q2: Apolloベースのシステムですか？**
- A: Apolloの軌道生成手法（QP）を**参考**にしていますが、Apolloの拡張ではありません。Urgency関数とV2V協調はオリジナルです。
- 詳細: [BACKGROUND_AND_OBJECTIVES.md - Apolloとの関係](BACKGROUND_AND_OBJECTIVES.md#5-apolloとの関係)

**Q3: 本研究の独自性は何ですか？**
- A: 3つあります：
  1. Urgency関数による空間分散誘導（完全オリジナル）
  2. V2V軌道共有のST-Boundary統合（オリジナル実装）
  3. 自律分散でありながらLC分散を実現
- 詳細: [BACKGROUND_AND_OBJECTIVES.md - 研究の独自性](BACKGROUND_AND_OBJECTIVES.md#4-研究の独自性と貢献)

**Q4: Front-loadingとは何ですか？**
- A: 織り込み区間の入口付近に車線変更が集中し、区間全体を有効活用できない問題です。
- 詳細: [BACKGROUND_AND_OBJECTIVES.md - Front-loading問題](BACKGROUND_AND_OBJECTIVES.md#1-織り込み区間における交通課題)

### 実装について

**Q5: どのファイルから読めばいいですか？**
- A: 以下の順序を推奨：
  1. `README.md` - プロジェクト概要
  2. `docs/BACKGROUND_AND_OBJECTIVES.md` - 研究の定義と目的
  3. `docs/ARCHITECTURE.md` - システム設計
  4. `weaving_v11/main.py` - エントリーポイント

**Q6: シミュレーションを実行するには？**
```bash
# HIGH負荷でシミュレーション（600秒）
python -m weaving_v11.main --mode sim --load high --tmax 600

# 最適化済みパラメータを使用
python -m weaving_v11.main --mode sim --load high --config optimization/best_params_v11_high.json --tmax 600
```
- 詳細: [IMPLEMENTATION.md](IMPLEMENTATION.md)

**Q7: パラメータを変更するには？**
- A: `weaving_v11/parameters.py` を編集するか、JSONファイルで上書きします。
- 詳細: [API_REFERENCE.md](API_REFERENCE.md)

**Q8: V2V効果を検証するには？**
```bash
# V2V有効（デフォルト）
python -m weaving_v11.main --mode sim --load high --tmax 600

# V2V無効（比較用）
python -m weaving_v11.main --mode sim --load high --tmax 600 --disable-v2v
```

### パラメータについて

**Q9: Urgencyパラメータ γ の推奨値は？**
- A: γ = 3.0 - 3.7（ベイズ最適化で調整済み）
- γ < 2.0: Front-loading発生
- γ > 4.0: 出口付近で集中
- 詳細: [ALGORITHMS.md](ALGORITHMS.md)

**Q10: QP重みパラメータの意味は？**
- `qp_w_j`: ジャークペナルティ（推奨: 5279）→ 軌道の滑らかさ
- `qp_w_a`: 加速度ペナルティ（推奨: 26）→ エネルギー効率
- 詳細: [ALGORITHMS.md](ALGORITHMS.md)、[APOLLO_ALIGNMENT.md - パラメータ対応](APOLLO_ALIGNMENT.md#parameter-alignment)

### トラブルシューティング

**Q11: QP最適化が失敗します**
- A: 以下を確認：
  1. ST-Boundary制約が厳しすぎないか
  2. 車両密度が高すぎないか
  3. ログで `[QP_FAIL]` を検索
- 詳細: [IMPLEMENTATION.md](IMPLEMENTATION.md)

**Q12: 衝突が発生します**
- A: ログで `[COLLISION]` を検索し、以下を確認：
  1. RSS安全距離パラメータ
  2. V2V軌道共有の有効性
  3. AEB発動率
- 詳細: [ALGORITHMS.md - RSS安全距離](ALGORITHMS.md)

**Q13: ログファイルはどこにありますか？**
- A: `outputs/simulation_log_v11_*.txt`
- フォーマット: `simulation_log_v11_<LOAD>_<TIMESTAMP>.txt`

---

## 📝 ドキュメント更新履歴

| 日付 | 更新内容 |
|------|---------|
| 2026-01-23 | **INDEX.md v2.0完成**: キーワード索引（A-Z）、FAQ、実装ファイルリンク、相互参照マップを追加 |
| 2026-01-23 | BACKGROUND_AND_OBJECTIVES.mdに「研究の正確な定義」「Apolloとの関係」「用語の統一」を追加 |
| 2026-01-22 | BACKGROUND_AND_OBJECTIVES.md v1.1 更新 |
| 2026-01-20 | LITERATURE_REVIEW.md追加 |
| 2025-12-26 | APOLLO_ALIGNMENT.md v1.0 作成 |
| 2025-12-24 | ALGORITHMS.md更新 |

---

## 🚀 次のステップ

### Phase 1: データ収集と検証（1-2週間）

```bash
# シミュレーションログ確認
outputs/simulation_log_v11_HIGH_*.txt を分析

# V2V効果の比較実験
python -m weaving_v11.main --mode sim --load high --tmax 600  # V2V有効
python -m weaving_v11.main --mode sim --load high --tmax 600 --disable-v2v  # V2V無効

# 空間分散の定量化
- LC位置のGini係数計算
- ヒートマップ作成
```

### Phase 2: 論文執筆（2-3週間）

- Introduction & Related Work
- Proposed Method (Urgency + V2V + QP)
- Evaluation (V2V効果、空間分散、安全性)

---

**End of Documentation Index**
