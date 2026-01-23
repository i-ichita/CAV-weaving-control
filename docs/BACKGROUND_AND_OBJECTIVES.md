# 研究背景と目的

> **関連ドキュメント**: [README](../README.md) | [LITERATURE_REVIEW](LITERATURE_REVIEW.md) | [ARCHITECTURE](ARCHITECTURE.md) | [ALGORITHMS](ALGORITHMS.md)

**対象読者**: 研究者・論文執筆者・プロジェクト関係者
**目的**: 本研究の背景にある交通課題と、それに対する研究目的を明確化する

---

## 目次

1. [研究背景](#1-研究背景-background)
2. [研究目的](#2-研究目的-objective)
3. [研究目的の統合的記述](#3-研究目的の統合的記述)
4. [研究の独自性と貢献](#4-研究の独自性と貢献)
5. [研究の進め方](#5-研究の進め方research-workflow)

---

## 1. 研究背景 (Background)

本研究は、都市高速道路における交通渋滞の主要因である**「織り込み区間（Weaving Section）」**の効率化と安全性向上を背景としています。

### (1) 織り込み区間における交通課題

織り込み区間は、合流と分岐が近接しており、車線変更（Lane Changing: LC）が頻繁に発生するため、交通流が不安定になりやすいボトルネックです。

#### **Capacity Drop（容量低下）**
[G_Tilg2018](LITERATURE_REVIEW.md#1-g_tilg2018pdf) が示すように、渋滞発生後に交通容量が低下する現象が確認されており、これは車線変更による車両間の干渉や空隙（Void）の発生が主因です。

#### **Front-loading（前方集中）問題**
[S_Tanaka2017](LITERATURE_REVIEW.md#2-s_tanaka2017pdf) が指摘するように、多くの車両が織り込み区間の手前（上流）で早めに車線変更を完了しようとするため、入口付近に混雑が集中し、区間全体を有効活用できていないという課題があります。

**これらの課題は、LC位置の空間的分散が交通効率向上の鍵であることを示しています。**（詳細は [LITERATURE_REVIEW.md - A. 織り込み区間の交通流特性と課題提起](LITERATURE_REVIEW.md#a-織り込み区間の交通流特性と課題提起) を参照）

---

### (2) 既存技術（先行研究）の限界

自動運転車（CAV）を用いた制御研究は多く存在しますが、実用化に向けて以下の課題が残されています。

#### **中央集権型のスケーラビリティ問題**
[C_Zhang2019、T_Peng2025、X_Xu_2024](LITERATURE_REVIEW.md#6-c_zhang2019pdf-t_peng2025pdf-x_xu_2024pdf-ramezani2019pdf) などの路側機（RSU）や中央サーバーが全車両を制御する手法は、通信負荷や計算コストが高く、**単一障害点（サーバーダウン時の全停止）のリスク**があります。

#### **計算コストとリアルタイム性**
[E_Amini2021](LITERATURE_REVIEW.md#4-e_amini2021pdf) の遺伝的アルゴリズム（GA）や [R_Rudolf2024](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf) の混合整数計画法（MIQP）を用いた最適化は計算負荷が高く、10Hzなどの高速な制御周期での実装が困難です。

#### **安全性の証明**
[L_Yan2024、xi2020](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf) などの深層強化学習（DRL）を用いた学習ベースの手法は、**ブラックボックス性が高く、事故責任や安全性の数学的保証（Safety Assurance）が困難**です。

#### **実用との乖離**
多くの研究が独自の簡易シミュレーションに留まり、[Baidu Apollo (F_Zhu2018)](LITERATURE_REVIEW.md#10-f_zhu2018pdf) のような産業界標準のアーキテクチャとの整合性が考慮されていません。

**詳細な先行研究の限界については、[LITERATURE_REVIEW.md - B. 制御アルゴリズムと最適化手法](LITERATURE_REVIEW.md#b-制御アルゴリズムと最適化手法) を参照してください。**

---

## 2. 研究目的 (Objective)

本研究の目的は、CAVの通信機能（V2V）を活用し、**「分散自律型」**かつ**「実用的な計算コストで安全性を保証する」**織り込み区間の制御システムを構築することです。

具体的には以下の3点を達成することを目指しています。

---

### (1) Urgency関数による空間的・時間的分散の実現

特定の場所（入口付近）への車線変更集中（Front-loading）を防ぐため、**「Urgency関数（切迫度関数）」**を導入します。

#### **アプローチ**
車両の現在位置と周辺密度に基づき、車線変更の実行確率を動的に変化させることで、中央からの指示なしに、各車両が自律的に車線変更位置を区間全体に分散（奥寄り配置）させ、交通流の効率化を図ります。

**Urgency関数の数学的定式化**:

```
U(s, ρ) = min(1.0, (s_norm)^γ + α · ρ_target)

where:
  s_norm = (s - s_entry) / (s_exit - s_entry)  ∈ [0, 1]
  γ = 3.0  (Urgency curve steepness)
  α = 0.2  (Density influence coefficient)
  ρ_target: Density in target lane [veh/m]
```

実装箇所: [weaving_v11/mpc_controller.py:70-76](../weaving_v11/mpc_controller.py)

**設計意図**:
- γ < 2.0: 緩やかな曲線 → 早期LC → Front-loading ❌
- γ > 4.0: 急峻な曲線 → 出口付近での集中 → Exit deadlock ❌
- γ = 3.0: 自然な分布 → 空間的に分散 ✓

#### **先行研究との差異**
[Tilg2018やTanaka2017](LITERATURE_REVIEW.md#34-先行研究との差異自律分散とlc分散の両立) が提案する「固定的なエリア規制」や「集中制御」ではなく、**「各車両の自律的な確率判断」**によって空間分散を実現する軽量なアルゴリズムです。

詳細は [ALGORITHMS.md - Urgency Function](ALGORITHMS.md) を参照してください。

---

### (2) 分散自律型・階層型制御アーキテクチャの構築

中央サーバーに依存しないスケーラブルなシステムを実現するため、**Level 1（戦略）**と**Level 2（戦術）**の階層型制御を提案します。

#### **Level 1 (MPC/Urgency)**: 戦略的意思決定層
- **制御周期**: 0.5秒（実装: [mpc_controller.py:88](../weaving_v11/mpc_controller.py)）
- **役割**: 車線変更の意思決定と大まかなスケジュールを決定
- **特徴**: Urgency関数による確率的判断で、中央管理なしにLC分散を実現
- **計画ホライズン**: 5.0秒（[parameters.py:252](../weaving_v11/parameters.py)）

#### **Level 2 (QP/Apollo)**: 戦術的軌道生成層
- **制御周期**: 0.1秒（10Hz、Apollo標準）（実装: [parameters.py:364](../weaving_v11/parameters.py)）
- **物理シミュレーション**: 0.01秒（100Hz）で高精度な衝突検出（[parameters.py:363](../weaving_v11/parameters.py)）
- **役割**: 衝突回避制約を満たす具体的な加速度・操舵プロファイルを生成
- **特徴**: 各車両が独立して二次計画法（QP）を解くことで、計算負荷を分散
- **計画ホライズン**: 8.0秒（80ステップ × 0.1秒）（[parameters.py:284](../weaving_v11/parameters.py)）

#### **アーキテクチャの優位性**
[LITERATURE_REVIEW.md - 3. CAVを用いる意義](LITERATURE_REVIEW.md#3-cavを用いる意義) で詳述されているように、本研究は**「自律分散型（Decentralized）」でありながら「LC分散（Spatial Distribution）」を実現**しています。これは、CAVの計画軌道共有という特性を活かすことで、中央管理者なしに車両間で暗黙的に協調し、結果的にLCが時空間的に分散されるという、**従来手法では不可能だった両立を達成**しています。

詳細なアーキテクチャは [ARCHITECTURE.md](ARCHITECTURE.md) を参照してください。

---

### (3) 数学的な安全性保証と実用性の両立

理論的な安全性と産業界標準の実装を統合します。

#### **RSS (Responsibility-Sensitive Safety)**
[Shalev-Shwartz2017](LITERATURE_REVIEW.md#8-shai-shalev-shwartz2017pdf-mobileye-rss) が提唱するMobileyeの責任感知型安全モデルを採用し、V2V通信による反応時間短縮（0.5s → 0.15s）を反映させた安全距離基準を実装します。

**本研究の拡張**: RSS理論を、V2V通信を前提とした環境に適応させ、**実用的な安全距離計算式として実装**しています。

**RSSパラメータの実装** ([apollo_safety.py:45-51](../weaving_v11/apollo_safety.py)):
```python
RSS_REACTION_TIME_CAV = 0.15   # [s] V2V通信環境での反応時間
RSS_REACTION_TIME = 0.5        # [s] HDV（人間運転）の反応時間
A_MAX_BRAKE = 6.0              # [m/s²] 最大制動（UN R157準拠）
A_COMFORT_BRAKE = 3.5          # [m/s²] 快適制動
L_VEHICLE = 5.0                # [m] 車両長
```

**RSS安全距離の計算式**:
```
d_safe = v_rear · t_reaction + (v_rear² - v_front²) / (2 · a_brake) + d_min

where:
  t_reaction = 0.15s (CAV-CAV), 0.5s (CAV-HDV or HDV-HDV)
  a_brake = 6.0 m/s² (緊急制動)
```

#### **ST-Boundary制約**
[Baidu Apollo (F_Zhu2018)](LITERATURE_REVIEW.md#10-f_zhu2018pdf) アーキテクチャに準拠し、時間(t)と位置(s)のグラフ上で障害物領域を定義することで、数学的に追突が不可能な軌道生成（Collision-free trajectory generation）を保証します。

**ST-Boundary実装** ([apollo_safety.py:106-112](../weaving_v11/apollo_safety.py)):
```python
@dataclass
class STBoundary:
    s_lower: np.ndarray  # 各時刻の最小安全位置
    s_upper: np.ndarray  # 各時刻の最大安全位置
    boundary_type: BoundaryType  # FOLLOW/YIELD/OVERTAKE
    vehicle_id: int
```

各障害物に対してST-Boundaryを計算し、QPの制約として統合することで、複数車両との同時衝突回避を保証します。

#### **QP定式化による高速計算**
[Werling2010](LITERATURE_REVIEW.md#7-werling2010pdf) のFrenetフレーム理論を継承しつつ、サンプリングベースではなくApollo型のPiecewise-Jerk QP定式化を採用することで、[MIQP手法](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf) や [GA手法](LITERATURE_REVIEW.md#4-e_amini2021pdf) よりも**高速かつ決定論的な安全性**を持って解を導出します。

**Piecewise-Jerk QP定式化** ([frenet_qp_apollo.py:13-15](../weaving_v11/frenet_qp_apollo.py)):

制御入力はジャーク(j)とし、以下の運動方程式で軌道を生成：
```
s(t+1) = s(t) + v(t)·dt + 0.5·a(t)·dt² + (1/6)·j(t)·dt³
v(t+1) = v(t) + a(t)·dt + 0.5·j(t)·dt²
a(t+1) = a(t) + j(t)·dt
```

**目的関数**:
```
minimize: Σ [w_ref · (s - s_ref)² + w_a · a² + w_j · j²]

subject to:
  s_lower - slack ≤ s(t) ≤ s_upper + slack  (ST-Boundary制約)
  v_min ≤ v(t) ≤ v_max
  a_min ≤ a(t) ≤ a_max  (a_min = -6.0 m/s²)
  slack ≥ 0  (ソフト制約変数)
```

**ソルバー**: OSQP（Apollo/Autoware標準、[parameters.py:265](../weaving_v11/parameters.py)）

スラック変数の導入により、極端な状況でもInfeasibleにならず、「最小違反軌道」を計算できるロバストな定式化を実現しています。

詳細な実装フローは [APOLLO_ALIGNMENT.md](APOLLO_ALIGNMENT.md) を参照してください。

---

## 3. 研究目的の統合的記述

本研究の目的は、**高速道路の織り込み区間における交通流の効率化と安全性の両立**です。これを3つの具体的な検証課題として定式化します。

### 検証課題1: 空間的分散の実証

**目的**: Urgency関数を用いた自律的な意思決定ロジックにより、既存研究（[Tilg et al., 2018](LITERATURE_REVIEW.md#1-g_tilg2018pdf)）で課題とされた**車線変更の上流集中（Front-loading）を解消**し、LC位置を空間的に分散させる効果を実験的に実証する。

**具体的アプローチ**:
- Urgency関数 `U(s, ρ) = min(1.0, (s_norm)^γ + α·ρ_target)` による確率的LC判断
- γ=3.0により、入口付近での早期LCを抑制し、区間全体に自然分散
- 実装: [mpc_controller.py:70-76](../weaving_v11/mpc_controller.py)

**先行研究との差異**: [Tilg2018/Tanaka2017](LITERATURE_REVIEW.md#a-織り込み区間の交通流特性と課題提起)は「固定的なエリア規制」や「集中制御」を提案したが、本研究は各車両の自律的な確率判断により分散を実現。

---

### 検証課題2: スケーラビリティの実現

**目的**: この制御を**中央サーバーに依存しない分散自律型アーキテクチャ**で実現し、従来の中央集権型手法（[Zhang et al., 2019](LITERATURE_REVIEW.md#6-c_zhang2019pdf-t_peng2025pdf-x_xu_2024pdf-ramezani2019pdf)）が抱える**スケーラビリティの課題を解決**する。

**具体的アプローチ**:
- 各車両が独立してQPを解く（Level 2: 10Hz周期）
- V2V通信で計画軌道を共有し、他車の計画をST-Boundary制約として取り込む
- 中央サーバー不要 → 単一障害点の排除、無限のスケーラビリティ
- 実装: [controllers.py:180-222](../weaving_v11/controllers.py)

**先行研究との差異**: [Zhang2019/Peng2025/Xu2024](LITERATURE_REVIEW.md#6-c_zhang2019pdf-t_peng2025pdf-x_xu_2024pdf-ramezani2019pdf)のRSU主導型は、中央サーバーの計算能力に依存し、サーバーダウン時に全停止するリスクがある。

**本研究の優位性**: [LITERATURE_REVIEW.md - 3.2](LITERATURE_REVIEW.md#32-自律分散型でありながらlc分散を実現するメカニズム)で詳述されているように、「制御の分散」と「LC位置の分散」という2つの分散を同時達成。

---

### 検証課題3: 数理的安全性とリアルタイム性の両立

**目的**: Apollo準拠の**ST-Boundary制約付きQP**を用いることで、ブラックボックスな学習手法（[DRL（Yan2024/xi2020）](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf)）とは異なり、**数理的に衝突安全性が保証された軌道をリアルタイム（10Hz）に生成**できることを示す。

**具体的アプローチ**:

1. **RSS安全距離**（[Shalev-Shwartz2017](LITERATURE_REVIEW.md#8-shai-shalev-shwartz2017pdf-mobileye-rss)）
   - V2V環境での反応時間短縮（0.5s → 0.15s）を反映
   - 実装: [apollo_safety.py:45-51](../weaving_v11/apollo_safety.py)

2. **ST-Boundary制約**（[Apollo (F_Zhu2018)](LITERATURE_REVIEW.md#10-f_zhu2018pdf)）
   - 時間-空間グラフ上で衝突不可能領域を定義
   - 実装: [frenet_qp_apollo.py:867-975](../weaving_v11/frenet_qp_apollo.py)

3. **Piecewise-Jerk QP**（[Werling2010](LITERATURE_REVIEW.md#7-werling2010pdf)）
   - OSQPソルバーによる高速計算（10Hz動作）
   - 実装: [frenet_qp_apollo.py:558-750](../weaving_v11/frenet_qp_apollo.py)

**先行研究との差異**:
- [DRL（Yan/xi等）](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf): ブラックボックスで安全性の数学的証明が困難
- [GA（Amini2021）](LITERATURE_REVIEW.md#4-e_amini2021pdf): 計算負荷が高くリアルタイム制御に不向き
- [MIQP（Rudolf2024）](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf): 混合整数計画法は10Hz制御には遅すぎる

**本研究の優位性**: 説明可能（White-box）かつ高速（10Hz）な安全軌道生成を実現。

---

## 4. 研究の独自性と貢献

上記3つの検証課題を通じて、本研究は以下の独自性と学術的貢献を提供します：

| 独自性 | 内容 | 先行研究との差異 |
|-------|------|---------------|
| **1. 自律分散型でありながらLC分散** | V2V計画軌道共有により、中央管理なしに暗黙的協調でLC分散を実現（検証課題1+2） | [集中型（Zhang, Peng等）](LITERATURE_REVIEW.md#34-先行研究との差異自律分散とlc分散の両立)：LC分散可能だがスケーラビリティに課題<br>[従来の分散型](LITERATURE_REVIEW.md#32-自律分散型でありながらlc分散を実現するメカニズム)：自律的だがLCが集中 |
| **2. 説明可能な安全性保証** | RSSベースの数学的安全保証 + ST-Boundary制約による衝突回避（検証課題3） | [DRL/NN（Yan, xi等）](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf)：ブラックボックスで安全性証明が困難 |
| **3. 実用性とリアルタイム性** | Apollo準拠のQP定式化により10Hz制御を実現（検証課題3） | [GA（Amini）](LITERATURE_REVIEW.md#4-e_amini2021pdf)/[MIQP（Rudolf）](LITERATURE_REVIEW.md#5-l_yan2024pdf-xi2020pdf-r_rudolf2024pdf)：計算コストが高くリアルタイム制御困難 |

### 研究の統合的意義

つまり、本研究は**「分散自律型制御」**というスケーラブルなアプローチで挑みつつ、**「Apollo準拠のQP」**と**「RSS」**を用いることで、アカデミックな理想論に留まらない**実機搭載可能なレベルの安全性とリアルタイム性**を提供します。

これは、以下の3つの検証を通じて実証されます：
1. **検証課題1**: Front-loading問題の解消（効率化）
2. **検証課題2**: 中央サーバー不要の分散型制御（スケーラビリティ）
3. **検証課題3**: 数理的安全保証とリアルタイム制御の両立（実用性）

---

**より詳細な先行研究との比較と理論的裏付けについては、[LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) を参照してください。**

---

## 5. 研究の進め方（Research Workflow）

本研究はシミュレーションベースの研究であり、以下のフェーズで進行します。各フェーズは**反復的**に実施され、必要に応じて前のフェーズに戻ることがあります。

### 全体フロー

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│   Phase 1: 理論構築                                    │
│        ↓                                               │
│   Phase 2: シミュレータ実装 ←─────────┐               │
│        ↓                              │               │
│   Phase 3: パラメータ設定 ←───┐       │               │
│        ↓                     │       │ バグ発見      │
│   Phase 4: 効果検証 ─────────┘       │ or           │
│        │                             │ 理論の修正   │
│        │ 問題発見 ───────────────────┘               │
│        ↓                                               │
│   Phase 5: 考察・確定                                  │
│        ↓                                               │
│   Phase 6: 論文化                                      │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

### Phase 1: 理論構築

**目的**: 「何を」「なぜ」やるかを明確にする

**内容**:
- 問題の特定（Front-loading、Capacity Drop）
- 解決策の設計（Urgency関数、階層制御、ST境界+QP）
- 先行研究との差異の明確化

**成果物**:
- [BACKGROUND_AND_OBJECTIVES.md](BACKGROUND_AND_OBJECTIVES.md)（本文書）
- [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md)
- [ALGORITHMS.md](ALGORITHMS.md)

**状況**: ✅ 完了

---

### Phase 2: シミュレータ実装

**目的**: 理論を検証可能な形にする

**内容**:
- 車両モデル、交通流モデルの実装
- Level 1（戦略層）/ Level 2（戦術層）制御の実装
- V2V通信モデルの実装
- 評価指標の計測機能

**成果物**:
- [weaving_v11/](../weaving_v11/)（シミュレータ本体）
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [IMPLEMENTATION.md](IMPLEMENTATION.md)

**状況**: ✅ 基本実装完了（Phase 3, 4と並行して改良の可能性あり）

---

### Phase 3: パラメータ設定

**目的**: 「公平な比較」ができる設定を決める

**パラメータの種類**:

| 種類 | 例 | 決め方 |
|-----|-----|-------|
| **環境パラメータ** | 道路長、車両密度、速度分布 | 先行研究や実測値を参考に固定 |
| **制御パラメータ** | γ, α, w_j, w_a | 最適化 or 感度分析 |

**制御パラメータの決定手順**:
1. **ベースライン設定**: 手動設定で動作確認
2. **粗探索**: Bayesian Optimizationで有望な領域を特定
3. **感度分析**: 各パラメータの影響度を定量化（「なぜその値か」を説明可能にする）

**主要な制御パラメータ**:

| パラメータ | 記号 | 役割 | 探索範囲 |
|-----------|------|------|---------|
| Urgency急峻度 | γ | LC位置の分散度合い | 2.0 - 4.0 |
| 密度影響係数 | α | 混雑時のUrgency底上げ | 0.1 - 0.3 |
| ジャークコスト | w_j | 軌道の滑らかさ | 2000 - 8000 |
| 加速度コスト | w_a | エネルギー効率 | 10 - 50 |

**成果物**:
- 最適化スクリプト（`bayesian_optimization.py`等）
- パラメータ設定の根拠文書

**状況**: 🔄 進行中（Bayesian Optimization実行中）

---

### Phase 4: 効果検証

**目的**: 提案手法の有効性を定量的に示す

**比較実験の設計**:

提案手法の有効性を示すため、以下の比較軸で実験を行う。

| 比較軸 | 条件 | 検証内容 |
|-------|------|---------|
| **制御方式** | HDVのみ / 提案手法 | 提案手法の効果 |
| **CAV普及率** | 0% / 50% / 100% | HDV混在環境での有効性 |
| **交通密度** | Low / Medium / High | 適用条件の範囲 |
| **Urgencyパラメータ** | γ=2.0, 3.0, 4.0 | パラメータ感度 |

**CAV普及率と混在シナリオ**:

| 普及率 | 構成 | 検証目的 |
|-------|------|---------|
| 0% | 全車HDV | ベースライン（制御なし） |
| 50% | CAV + HDV混在 | 現実的な過渡期シナリオ |
| 100% | 全車CAV | 提案手法の理想的効果 |

※ 現在の実装は全車CAV（100%）。HDV混在シナリオ（0%, 50%）は今後実装予定。

**HDVモデルの想定**:
- 車線変更判断: 固定閾値（位置ベース）または確率的（人間の判断ばらつき）
- 軌道生成: IDMベースの追従モデル（QP最適化なし）
- V2V通信: なし（CAVの計画軌道を受信できない）

**評価指標**:

| 指標 | 意味 | 対応する検証課題 |
|-----|------|-----------------|
| LC位置のGini係数 | 空間分散の度合い（低いほど均等） | 検証課題1 |
| 衝突率 | 安全性 | 検証課題3 |
| スループット | 単位時間あたりの通過車両数 | 検証課題1 |
| 平均速度 | 交通効率 | 検証課題1 |
| 計算時間 | リアルタイム性（10Hz達成可否） | 検証課題3 |

**実験条件**:
- 複数の交通密度（Low / Medium / High）
- 複数のCAV普及率（0% / 50% / 100%）
- 統計的有意性のための複数回試行（各条件5回以上）

**成果物**:
- 実験結果データ（`outputs/`）
- グラフ・可視化

**状況**: 🔄 進行中

---

### Phase 5: 考察・確定

**目的**: 結果から「何が言えるか」を整理する

**内容**:
- なぜ効果があったか / なかったかの分析
- パラメータ感度の考察（どれが重要か）
- 提案手法の限界と適用条件
- 今後の課題の整理

**成果物**:
- 考察文書
- 最終パラメータ設定

**状況**: ⏳ 未着手

---

### Phase 6: 論文化

**目的**: 研究成果を論文としてまとめる

**内容**:
- 論文構成の決定
- 図表の作成
- 執筆・推敲

**状況**: ⏳ 未着手

---

### 注意事項：シミュレータ変更時の対応

Phase 3, 4の実行中にシミュレータ（weaving_v11）を変更する場合、変更の種類によって対応が異なります。

| 変更の種類 | 例 | パラメータ最適化への影響 |
|-----------|-----|------------------------|
| **バグ修正** | 衝突判定の誤り修正 | やり直し必要 |
| **機能追加（出力のみ）** | 新しいログ出力追加 | 影響なし |
| **理論変更** | Urgency関数の形を変える | **全てやり直し** |
| **パラメータ追加** | 新しい調整項の追加 | 部分的にやり直し |

**推奨手順**（大きな変更を入れる場合）:
1. 現在の最適化を最後まで実行（ベースライン結果として保存）
2. 変更を実施
3. 変更後に再度最適化を実行
4. 変更前後の結果を比較可能な状態で保持

---

### 現在の状況サマリ

| Phase | 状況 | 備考 |
|-------|------|------|
| 1. 理論構築 | ✅ 完了 | |
| 2. シミュレータ実装 | ✅ 完了 | 改良の可能性あり |
| 3. パラメータ設定 | 🔄 進行中 | Bayesian Opt実行中 |
| 4. 効果検証 | 🔄 進行中 | Phase 3と並行 |
| 5. 考察・確定 | ⏳ 未着手 | |
| 6. 論文化 | ⏳ 未着手 | |

---

**文書バージョン**: 1.1
**最終更新**: 2026-01-22

---

**End of Background and Objectives**
