# システムアーキテクチャ

> **関連ドキュメント**: [README](../README.md) | [ALGORITHMS](ALGORITHMS.md) | [API_REFERENCE](API_REFERENCE.md) | [IMPLEMENTATION](IMPLEMENTATION.md) | [APOLLO_ALIGNMENT](APOLLO_ALIGNMENT.md)

**対象読者**: 開発者（コードを読む/書く人）
**目的**: システム全体の構造を理解し、どこに何があるか素早く把握する

---

## 制御アーキテクチャの分類

### 分散自律型制御 (Decentralized Autonomous Control)

本システムは**分散自律型**の制御アーキテクチャを採用しています。

```
┌────────────────────────────────────────────────────────────────┐
│                    分散自律型制御の特徴                         │
├────────────────────────────────────────────────────────────────┤
│  ✓ 各車両が独立して意思決定（中央サーバーなし）                │
│  ✓ CAVは軌道共有のみ（制御指令の交換ではない）                 │
│  ✓ HDVとの混合交通に対応（CAV率に依存しない設計）              │
└────────────────────────────────────────────────────────────────┘
```

### 中央集権型との違い

| 観点 | 中央集権型 | 本システム（分散自律型） |
|------|-----------|------------------------|
| **意思決定** | 中央サーバーが全車両の軌道を最適化 | 各車両が独立にQPを解く |
| **通信** | 全車両 → 中央 → 全車両 | CAV間でV2V軌道共有のみ |
| **スケーラビリティ** | 車両数に応じて計算負荷増大 | 車両数に依存しない |
| **単一障害点** | 中央サーバー故障で全停止 | なし（各車両が自律動作） |
| **HDV対応** | HDVの予測が必要 | HDVは通常のIDM予測で対応 |

### CAVの役割：V2V通信による協調制御

CAV（Connected Automated Vehicle）は**2種類の協調**を行います。

#### 1. 軌道共有（受動的協調）

各CAVはQPソルバーで計算した軌道を共有し、他のCAVがそれを予測に使用します。

```
【パラダイム】Prediction-Based → Agreement-Based

従来（予測ベース）:
  CAV-A: 「CAV-Bは多分こう動くだろう」（不確実）
  → 大きな安全バッファが必要

本システム（合意ベース）:
  CAV-A: 「CAV-BのQP解（コミット済み軌道）を受信」
  → CAV-Bはその軌道に従う前提で計画可能
  → 安全バッファを最小化
```

**共有データ構造** ([vehicle.py:660-662](../weaving_v11/vehicle.py#L660-L662)):
```python
predicted_trajectory: List[Tuple[float, float, float, float]]
# Format: [(t_abs, s, v, a), ...]
# t_abs: 絶対時刻 [s], s: 位置 [m], v: 速度 [m/s], a: 加速度 [m/s²]
```

#### 2. LC協調リクエスト（能動的協調）

ギャップが不足している場合、目標車線のCAVに**積極的に協力を依頼**します。

```
【協調フロー】

CAV-A: 車線変更したいがギャップ不足
    ↓ v2v_lc_requests を送信（urgent_yield / cooperative_yield）
CAV-B: リクエストを受信
    ↓ urgency_levelに応じて減速
    ↓ ・urgent_yield: 30%減速
    ↓ ・cooperative_yield: 15%減速
    → ギャップが形成され、CAV-Aが合流成功
```

**リクエストタイプ** ([controllers.py:711-720](../weaving_v11/controllers.py#L711-L720)):
| タイプ | 条件 | アクション |
|--------|------|-----------|
| `urgent_yield` | 出口まで50m未満 | 30%減速 |
| `cooperative_yield` | 出口まで100m未満 | 15%減速 |

**受信側の処理** ([controllers.py:1489-1537](../weaving_v11/controllers.py#L1489-L1537)):
- リクエストを優先度順にソート（emergency > urgent > cooperative）
- 相対位置に応じて減速または加速
- 3秒以上経過したリクエストは破棄

#### 3. コストベース仲裁（v25.2 Unified Decider）

リクエストを受けた側は**単純に言いなりではなく**、コスト計算で最適な行動を決定します。

$$
C_{\text{yield}} = 4.0 \times U + C_{\text{progress}} + 0.1 \times v
$$

- **Urgency高** → 譲りにくい（出口に近い車両が優先）
- **LC進捗深** → 譲りにくい（中断コストが高い）
- **高速走行** → 譲りにくい（ブレーキの快適性コスト）

**詳細な数式・数値例・実装**: → [ALGORITHMS.md - Section 12](ALGORITHMS.md#12-v2v協調仲裁アルゴリズム)

#### 中央集権型との違い（再掲）

この協調は**車両間の直接通信**で行われ、中央サーバーは介在しません。

| 協調タイプ | 通信方式 | 意思決定 |
|-----------|---------|---------|
| 中央集権型 | 全車両→中央→全車両 | 中央が最適配置を計算 |
| 本システム | CAV-A ⇄ CAV-B（直接） | 各車両が自律判断で協力 |

### HDVとCAVの混合交通

本システムは `is_hdv` フラグにより、HDV（Human-Driven Vehicle）とCAVが混在する環境に対応しています。

| 車両タイプ | 軌道共有 | 予測方法 | 安全バッファ |
|-----------|---------|---------|-------------|
| **CAV** | あり（QP解を共有） | 共有軌道を使用 | 最小（信頼度高） |
| **HDV** | なし | IDMベースCV外挿 | 大きめ（不確実性考慮） |

**実装**: [controllers.py:1683-1684](../weaving_v11/controllers.py#L1683-L1684)
```python
if not getattr(other_v, 'is_hdv', False):  # CAVのみ共有
    predicted_traj = getattr(other_v, 'predicted_trajectory', None)
```

---

## 全体アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│ IntegratedZoneSimulator (simulator.py)                      │
│   - シミュレーションループ（100Hz物理、10Hz制御）            │
│   - 車両生成（Poisson到着）                                  │
│   - 安全性評価（衝突検出、TTC計算）                          │
└─────────────────────────────────────────────────────────────┘
                            │
                            ├─── 制御指令 (0.1s周期)
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ IntegratedZoneController (controllers.py)                   │
│   ┌───────────────────────────────────────────────────┐     │
│   │ Level 1: Strategic Planning (0.5s周期)           │     │
│   │   - UrgencyPlanner (mpc_controller.py)           │     │
│   │   - 確率的LC決定                                  │     │
│   │   - Gap Acceptance検証                            │     │
│   └───────────────────────────────────────────────────┘     │
│                           ↓                                  │
│   ┌───────────────────────────────────────────────────┐     │
│   │ Level 2: Tactical Control (0.1s周期)             │     │
│   │   - FrenetQPController (frenet_qp_apollo.py)     │     │
│   │   - Frenet障害物検出                              │     │
│   │   - ST-Boundary制約構築                           │     │
│   │   - Piecewise Jerk QP最適化                       │     │
│   └───────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
                            │
                            ├─── 加速度指令 (ax)
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Vehicle (vehicle.py)                                         │
│   - 物理状態（x, v, ax）                                     │
│   - 車線状態（lane, lc_scheduled, lc_started）              │
│   - Urgency状態                                              │
│   - 予測軌道（CAV共有用）                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## クラス構成（詳細）

### 1. IntegratedZoneSimulator

**責務**: シミュレーション全体の管理

**主要属性**:
```python
params: IntegratedZoneParameters  # パラメータ
vehicles: List[Vehicle]           # 全車両リスト
controller: IntegratedZoneController  # 制御器
t: float                          # 現在時刻
```

**主要メソッド**:

| メソッド | 行番号 | 説明 |
|---------|--------|------|
| `run(t_max)` | 156-196 | メインループ（物理100Hz、制御10Hz） |
| `_spawn_vehicle(t)` | 417-501 | Poisson到着による車両生成 |
| `_control_update(t)` | 501-548 | 制御更新（s座標降順で処理） |
| `_update_vehicles(dt, t)` | 548-600 | 車両状態更新（物理シミュレーション） |
| `_evaluate_safety()` | 600-700 | 衝突検出・TTC計算 |

**実装箇所**: [simulator.py](../weaving_v11/simulator.py)

---

### 2. IntegratedZoneController

**責務**: 階層型制御の統合管理

**主要属性**:
```python
urgency_planner: UrgencyPlanner       # Level 1戦略
frenet_controller: FrenetQPController # Level 2戦術
safety_analyzer: SafetyAnalyzer       # 安全性解析
gap_acceptance_params: Dict           # Gap Acceptance閾値
prev_traj: Dict[int, Dict]            # 軌道縫合用
```

**主要メソッド**:

| メソッド | 行番号 | 説明 |
|---------|--------|------|
| `control_update(t, vehicles)` | 236-265 | Level 1 + Level 2実行 |
| `_update_level1_strategy(t, vehicles)` | 365-637 | Urgency計算 + LC決定 |
| `_execute_level2_control(t, vehicles)` | 638-1800 | Frenet QP最適化 |
| `_check_gap_acceptance(vehicle, target_lane, all_vehicles)` | 267-363 | Gap Acceptance検証 |

**実装箇所**: [controllers.py](../weaving_v11/controllers.py)

---

### 3. UrgencyPlanner

**責務**: Level 1戦略計画（Urgency計算）

**主要属性**:
```python
gamma: float = 3.0                # 急峻度パラメータ
alpha: float = 0.2                # 密度影響係数
replan_interval: float = 0.5      # 再計画周期 [s]
last_replan_time: float           # 前回再計画時刻
```

**主要メソッド**:

| メソッド | 行番号 | 説明 |
|---------|--------|------|
| `compute_urgencies_batch(vehicles, s_entry, s_exit, rho_per_lane)` | 60-120 | バッチUrgency計算 |
| `should_replan(t)` | 45-55 | 再計画判定（0.5s周期） |

**Urgency計算式**:
```python
U = min(1.0, ((s - s_entry) / (s_exit - s_entry)) ** gamma + alpha * rho_target)
```

**実装箇所**: [mpc_controller.py](../weaving_v11/mpc_controller.py)

---

### 4. FrenetQPController

**責務**: Frenet座標系でのQP軌道最適化

**主要属性**:
```python
N: int = 80                       # 予測ホライズン（ステップ数）
dt: float = 0.1                   # タイムステップ [s]
horizon_time: float = 8.0         # 予測時間 [s]
s0: float = 2.0                   # 静止時最小車間 [m]
w_ref: float = 0.5                # 位置/速度追従重み
w_a: float = 10.0                 # 加速度コスト
w_j: float = 2000.0               # Jerkコスト
```

**主要メソッド**:

| メソッド | 行番号 | 説明 |
|---------|--------|------|
| `optimize(ego_state, obstacles, urgency, ...)` | 558-750 | QP最適化実行 |
| `_build_qp_matrices(ego_state, obstacles, ...)` | 780-865 | QP行列構築 |
| `_solve_qp(P, q, A, l, u)` | 550-558 | OSQP求解 |
| `compute_adaptive_safety_margin(ego, front, ...)` | 780-865 | 適応的安全マージン |
| `_build_st_boundary(ego, obstacles)` | 867-975 | ST-Boundary構築 |

**実装箇所**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py)

---

### 5. Vehicle

**責務**: 車両状態の保持

**主要属性**:

```python
# 物理状態
id: int                           # 車両ID
x: float                          # 縦方向位置（Frenet s） [m]
y: float                          # 横方向位置（Global y） [m]
d: float                          # 横方向位置（Frenet d） [m]
v: float                          # 速度 [m/s]
ax: float                         # 加速度指令 [m/s²]

# 車線状態
lane: str                         # 現在車線
target_lane: str                  # 目標車線

# LC状態
lc_scheduled: bool                # LC予約済み
lc_started: bool                  # LC実行中
lc_completed: bool                # LC完了
changing_lane: bool               # LC遷移中
scheduled_time: float             # LC開始予定時刻

# Urgency
urgency: float                    # 切迫度 [0, 1]
urgency_state: UrgencyState       # Urgency詳細状態

# 軌道情報
velocity_profile: List[float]     # 速度プロファイル
predicted_trajectory: List[Tuple] # 予測軌道（CAV共有用）
```

**実装箇所**: [vehicle.py](../weaving_v11/vehicle.py)

---

### 6. IntegratedZoneParameters

**責務**: 全パラメータの一元管理

**主要カテゴリ**:

| カテゴリ | パラメータ例 | 行番号 |
|---------|-------------|--------|
| **道路構造** | `prep_zone_length`, `weave_zone_length` | 14-31 |
| **制御周期** | `dt_sim`, `dt_control`, `replan_interval` | 262-273 |
| **車両** | `v_min`, `v_max`, `a_min`, `a_max`, `L_vehicle` | 54-65, 309-311 |
| **Urgency** | `urgency_gamma`, `urgency_alpha` | 157-162 |
| **LCトリガー** | `P_{trigger}=U(x,\rho)`（パラメータなし） | controllers.py:535-633 |
| **Gap Acceptance** | `min_front_gap`, `min_rear_gap` | 186-191 |
| **Frenet QP** | `horizon`, `w_ref`, `w_a`, `w_j` | 197, 179-182 |
| **交通流** | `spawn_rate`, `v0_mean` | 318-322, 353-461 |

**実装箇所**: [parameters.py](../weaving_v11/parameters.py)

---

## 制御フロー（詳細）

### メインループ（simulator.py:156-196）

```python
while t < t_max:
    # 1. 車両生成（Poisson到着）
    if should_spawn(t):
        spawn_vehicle(t)

    # 2. 制御更新（10Hz = 0.1s周期）
    if t - last_control >= dt_control:
        # s座標降順で処理（デッドロック回避）
        active_vehicles = sorted(vehicles, key=lambda v: v.x, reverse=True)
        controller.control_update(t, active_vehicles)
        last_control = t

    # 3. 物理サブステップ（100Hz = 0.01s周期）
    for _ in range(10):  # dt_control / dt_sim = 0.1 / 0.01 = 10
        update_vehicles(dt_sim)
        evaluate_safety()
        t += dt_sim
```

---

### Level 1 + Level 2統合制御（controllers.py:236-265）

```python
def control_update(t, vehicles):
    # Level 1: Strategic Planning (0.5s周期)
    if urgency_planner.should_replan(t):
        for v in vehicles:
            # 1. Urgency計算（空間分散を決める）
            v.urgency = compute_urgency(v.x, v.target_lane, rho_per_lane)

            # 2. 確率的LC決定（時間分散）
            prob = v.urgency  # P_trigger = U(x, rho)

            if prob > random():
                # 3. Gap Acceptance検証（緩和係数 g(U) 適用）
                if check_gap_acceptance(v, target_lane, all_vehicles):
                    v.lc_scheduled = True
                    v.scheduled_time = t + lc_prep_duration

    # Level 2: Tactical Control (0.1s周期・毎回)
    for v in vehicles:
        # 1. Frenet障害物検出
        obstacles = detect_obstacles_frenet(v)

        # 2. QP最適化
        result = frenet_qp.optimize(v, obstacles, urgency=v.urgency)

        if result:
            v.ax = result['a'][0]
            v.velocity_profile = result['v']
            v.predicted_trajectory = result  # CAV共有
        else:
            # QP失敗時は緊急ブレーキ
            v.ax = -6.0  # UN R157準拠
```

---

### Frenet座標系での障害物検出（controllers.py:743-985）

```python
def detect_obstacles_frenet(ego_vehicle):
    ego_s = ego_vehicle.x
    ego_d = ego_vehicle.d  # Lane offsetからの実際の横ずれ

    obstacles = []

    # 1. 200m範囲内の車両をスキャン（Apollo標準）
    scan_radius = 200.0
    active_vehicles = [v for v in all_vehicles
                      if abs(v.x - ego_s) < scan_radius]

    # 2. 横方向オーバーラップ判定
    LATERAL_THRESHOLD = LANE_WIDTH / 2.0 + 0.25  # 2.0m

    for other in active_vehicles:
        other_s = other.x
        other_d = other.d

        lateral_distance = abs(other_d - ego_d)

        if lateral_distance < LATERAL_THRESHOLD:
            # 横方向でオーバーラップ → 障害物として認識
            obstacles.append(other)

    # 3. CIPV選択（前方・後方それぞれ最近接のみ）
    front = min([obs for obs in obstacles if obs.x > ego_s],
                key=lambda o: o.x - ego_s, default=None)
    rear = max([obs for obs in obstacles if obs.x < ego_s],
               key=lambda o: o.x, default=None)

    return [front, rear] if front or rear else []
```

---

### ST-Boundary制約の構築（frenet_qp_apollo.py:867-975）

```python
def build_st_boundary(ego, obstacles):
    s_upper = [inf] * N  # 上限（YIELD制約）
    s_lower = [0.0] * N  # 下限（OVERTAKE制約）

    for obs in obstacles:
        if obs.is_front:
            # YIELD: 前方車両に追いつかない制約
            for k in range(N):
                t_k = k * dt

                # 予測位置（等速モデル or CAV共有軌道）
                if obs.predicted_trajectory:
                    s_obs_k = obs.predicted_trajectory[k]['s']
                else:
                    s_obs_k = obs.s + obs.v * t_k

                # 安全距離
                d_safe = compute_safe_distance(ego.v, obs.v)

                # 上限制約
                s_upper[k] = min(s_upper[k], s_obs_k - d_safe)

        else:
            # OVERTAKE: 後方車両に追い越される制約
            for k in range(N):
                t_k = k * dt
                s_obs_k = obs.s + obs.v * t_k
                d_safe = compute_safe_distance(obs.v, ego.v)
                s_lower[k] = max(s_lower[k], s_obs_k + d_safe)

    return s_upper, s_lower
```

---

## データフロー

```
┌─────────────────────────────────────────────────────────────┐
│ 入力: センサー情報（全車両位置・速度）                       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Level 1: Urgency計算                                         │
│   入力: (s_i, target_lane, rho_per_lane)                    │
│   出力: urgency ∈ [0, 1]                                    │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Level 1: 確率的LC決定                                        │
│   入力: (x_norm, urgency)                                   │
│   出力: lc_scheduled (True/False)                           │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Level 2: Frenet障害物検出                                    │
│   入力: (ego_state, all_vehicles)                           │
│   出力: obstacles (List[ObstacleInfo])                      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Level 2: ST-Boundary構築                                     │
│   入力: (ego_state, obstacles)                              │
│   出力: (s_upper, s_lower)                                  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ Level 2: QP最適化                                            │
│   入力: (ego_state, s_upper, s_lower, v_ref, urgency)       │
│   出力: (s, v, a) × N                                       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 出力: 加速度指令 ax [m/s²]                                   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 車両運動学モデル                                             │
│   v(t+dt) = v(t) + ax * dt                                  │
│   x(t+dt) = x(t) + v(t) * dt + 0.5 * ax * dt²              │
└─────────────────────────────────────────────────────────────┘
```

---

## 重要な設計判断

### 1. なぜ逐次更新（s座標降順）か？

**問題**: 全車同時に軌道決定 → デッドロック
- 車両A「車両Bの軌道待ち」
- 車両B「車両Aの軌道待ち」

**解決策**: 前方車両（s座標大）から順に決定
```python
active_vehicles = sorted(vehicles, key=lambda v: v.x, reverse=True)
for v in active_vehicles:
    v.ax = optimize(v, already_determined_trajectories)
```

**効果**: デッドロック完全回避

**実装**: [simulator.py](../weaving_v11/simulator.py):543-548

---

### 2. なぜ確率的LCトリガーか？

**問題**: 決定論的コスト削減 → 全車が同じ位置でLC（Front-loading）

**解決策**: Urgencyをそのまま確率に使う（空間分散=Urgency, 時間分散=確率試行）
```python
# P_trigger = U(x_norm, rho)
prob = urgency
if prob > random():
    schedule_lc()
```

- 入口: U≈0.002 → 0.2%（ほぼトリガーしない）
- 中央: U≈0.127 → 12.7%
- 出口: U→1.0 → 100%（強制域）
- **時空間分散**: 空間はUrgencyの凸形状、時間は0.5sごとのベルヌーイ

**効果**: Gini係数 0.7 → 0.28（-60%）

**実装**: [controllers.py](../weaving_v11/controllers.py):535-633

---

### 3. なぜST-Boundary制約か？

**問題（v10.3）**: 各時刻独立のギャップ制約
```python
gap(t_k) >= d_safe  # ホライズン外で衝突可能
```
→ 衝突595回

**解決策（v11.0）**: 追いつき不可能性保証
```python
s_ego(t_k) <= s_front(t_k) - d_safe(v_ego, v_front)  # 全時刻で保証
```

**数学的証明**: 帰納法により全時間で `s_ego < s_front`

**効果**: 衝突595回 → 8回（-98.7%）

**実装**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py):867-975

---

### 4. なぜ8秒予測ホライズンか？

**問題（v12.1）**: 3秒予測 → 反応的制御 → AEB頻発

**解決策（v15.0）**: 8秒予測（Apollo標準）
- 早期に前方車両減速を検知
- 滑らかな減速プロファイル生成
- 緊急ブレーキ回避

**効果**: AEB発動 -60~80%（推定）

**実装**: [parameters.py](../weaving_v11/parameters.py):197

---

## ファイル間の依存関係

```
simulator.py
  ├─ import controllers.py
  │    ├─ import mpc_controller.py (UrgencyPlanner)
  │    ├─ import frenet_qp_apollo.py (FrenetQPController)
  │    └─ import vehicle.py (SafetyAnalyzer)
  ├─ import vehicle.py
  └─ import parameters.py

controllers.py
  ├─ import mpc_controller.py
  ├─ import frenet_qp_apollo.py
  ├─ import vehicle.py
  ├─ import parameters.py
  └─ import coordinate_transform.py

frenet_qp_apollo.py
  ├─ import osqp (外部ライブラリ)
  └─ import numpy (外部ライブラリ)

vehicle.py
  ├─ import parameters.py
  └─ (依存なし)

parameters.py
  └─ (依存なし)
```

**ボトムアップ読解順序**:
1. `parameters.py` → 2. `vehicle.py` → 3. `mpc_controller.py` → 4. `frenet_qp_apollo.py` → 5. `controllers.py` → 6. `simulator.py`

---

## まとめ

### 核心的な3つのコンポーネント

| コンポーネント | 役割 | 周期 | 実装ファイル |
|--------------|------|------|--------------|
| **UrgencyPlanner** | 戦略的計画（いつLCするか） | 0.5s | mpc_controller.py |
| **FrenetQPController** | 戦術的制御（どう動くか） | 0.1s | frenet_qp_apollo.py |
| **IntegratedZoneSimulator** | 全体管理（時間進行） | 0.01s | simulator.py |

### 重要な制約

1. **逐次更新**: s座標降順で処理
2. **ST-Boundary**: 追いつき不可能性保証
3. **Gap Acceptance**: LC前の安全検証
4. **CIPV選択**: 前後各1台のみ

### 性能のキーポイント

- **衝突-98%**: ST-Boundary制約
- **LC成功+2.6x**: 確率的トリガー + Urgency
- **AEB-60~80%**: 8秒予測ホライズン

---

**文書バージョン**: 1.0
**最終更新**: 2025-12-26
