# API完全リファレンス

> **関連ドキュメント**: [README](../README.md) | [ARCHITECTURE](ARCHITECTURE.md) | [ALGORITHMS](ALGORITHMS.md) | [IMPLEMENTATION](IMPLEMENTATION.md) | [APOLLO_ALIGNMENT](APOLLO_ALIGNMENT.md)

**対象読者**: 開発者（実装・デバッグする人）
**目的**: クラス・メソッド・パラメータを素早く検索・参照する

---

## クイック索引

| クラス | 説明 | ファイル |
|--------|------|----------|
| [IntegratedZoneSimulator](#integratedzones

imulator) | シミュレーター本体 | simulator.py |
| [IntegratedZoneController](#integratedzonecontroller) | 階層型制御器 | controllers.py |
| [UrgencyPlanner](#urgencyplanner) | Level 1戦略 | mpc_controller.py |
| [FrenetQPController](#frenetqpcontroller) | Level 2戦術 | frenet_qp_apollo.py |
| [Vehicle](#vehicle) | 車両状態 | vehicle.py |
| [IntegratedZoneParameters](#integratedzoneparameters) | パラメータ | parameters.py |
| [SafetyAnalyzer](#safetyanalyzer) | 安全性解析 | vehicle.py |

---

## IntegratedZoneSimulator

**ファイル**: [simulator.py](../weaving_v11/simulator.py)

### コンストラクタ

```python
def __init__(self, params: IntegratedZoneParameters)
```

**パラメータ**:
- `params`: システムパラメータ

**初期化内容**:
- `controller`: IntegratedZoneController
- `vehicles`: List[Vehicle] (空)
- `collision_count`: int = 0
- `near_miss_count`: int = 0

### 主要メソッド

#### run()

```python
def run(self, t_max: float = 600.0) -> Dict[str, Any]
```

**説明**: メインシミュレーションループ

**パラメータ**:
- `t_max`: シミュレーション時間 [s] (default: 600.0)

**戻り値**: 統計情報辞書
```python
{
    'exit_success_rate': float,
    'lc_success_rate': float,
    'collision_count': int,
    'near_miss_count': int,
    'gini_coefficient': float
}
```

**実装**: 156-196行

**ループ構造**:
```python
while t < t_max:
    # 車両生成
    _spawn_vehicle(t)

    # 制御更新（10Hz）
    if t - last_control >= dt_control:
        _control_update(t)

    # 物理更新（100Hz）
    for _ in range(10):
        _update_vehicles(dt_sim)
```

---

#### _spawn_vehicle()

```python
def _spawn_vehicle(self, t: float) -> Optional[Vehicle]
```

**説明**: Poisson到着による車両生成

**パラメータ**:
- `t`: 現在時刻 [s]

**戻り値**: 生成された車両 or None

**実装**: 417-501行

**生成条件**:
1. Poisson確率判定: `random() < spawn_rate * dt_sim`
2. 車線選択: LEFT側 vs RIGHT側（50%ずつ）
3. 初期位置: `x = -warmup_length`
4. 初速度: `v ~ N(v0_mean, v0_std)`

---

#### _control_update()

```python
def _control_update(self, t: float) -> None
```

**説明**: 制御更新（10Hz周期）

**パラメータ**:
- `t`: 現在時刻 [s]

**実装**: 501-548行

**処理内容**:
```python
# s座標降順でソート（デッドロック回避）
active = sorted(vehicles, key=lambda v: v.x, reverse=True)

# 制御実行
controller.control_update(t, active)
```

---

## IntegratedZoneController

**ファイル**: [controllers.py](../weaving_v11/controllers.py)

### コンストラクタ

```python
def __init__(self, params: IntegratedZoneParameters, simulator: IntegratedZoneSimulator)
```

**パラメータ**:
- `params`: システムパラメータ
- `simulator`: シミュレーター参照（車両リストアクセス用）

**初期化内容**:
```python
urgency_planner = UrgencyPlanner(gamma=3.0, alpha=0.2, replan_interval=0.5)
frenet_controller = FrenetQPController(horizon=80, dt=0.1)
safety_analyzer = SafetyAnalyzer(...)
gap_acceptance_params = {...}
prev_traj = {}  # 軌道縫合用
```

### 主要メソッド

#### control_update()

```python
def control_update(self, t: float, vehicles: List[Vehicle]) -> None
```

**説明**: Level 1 + Level 2実行

**パラメータ**:
- `t`: 現在時刻 [s]
- `vehicles`: 制御対象車両リスト

**実装**: 236-265行

**処理フロー**:
```python
# Level 1 (0.5s周期)
if urgency_planner.should_replan(t):
    _update_level1_strategy(t, vehicles)

# Level 2 (0.1s周期・毎回)
_execute_level2_control(t, vehicles)
```

---

#### _check_gap_acceptance()

```python
def _check_gap_acceptance(self, vehicle: Vehicle, target_lane: str,
                          all_vehicles: List[Vehicle]) -> bool
```

**説明**: Gap Acceptance検証

**パラメータ**:
- `vehicle`: LC希望車両
- `target_lane`: 目標車線
- `all_vehicles`: 全車両リスト

**戻り値**: True (安全) / False (危険)

**実装**: 267-363行

**検証項目**:
```python
min_front_gap = 15.0  # [m]
min_rear_gap = 20.0   # [m]
min_front_ttc = 3.0   # [s]
min_rear_ttc = 4.0    # [s]

# 予測ギャップ（3秒後）
LC_DURATION = 3.0
predicted_gap = current_gap - (rel_v * LC_DURATION)
```

---

## UrgencyPlanner

**ファイル**: [mpc_controller.py](../weaving_v11/mpc_controller.py)

### コンストラクタ

```python
def __init__(self, gamma: float = 3.0, alpha: float = 0.2,
             replan_interval: float = 0.5, urgency_min: float = 0.0,
             urgency_max: float = 1.0)
```

**パラメータ**:
- `gamma`: 急峻度（べき乗） (default: 3.0)
- `alpha`: 密度影響係数 (default: 0.2)
- `replan_interval`: 再計画周期 [s] (default: 0.5)
- `urgency_min`: Urgency最小値 (default: 0.0)
- `urgency_max`: Urgency最大値 (default: 1.0)

### 主要メソッド

#### compute_urgencies_batch()

```python
def compute_urgencies_batch(self, vehicles: List[Dict], s_entry: float,
                            s_exit: float, rho_per_lane: Dict[str, float]
                           ) -> List[UrgencyState]
```

**説明**: バッチUrgency計算

**パラメータ**:
- `vehicles`: 車両情報リスト `[{'id': int, 'x': float, 'target_lane': str}, ...]`
- `s_entry`: 入口位置 [m] (通常0.0)
- `s_exit`: 出口位置 [m] (通常900.0)
- `rho_per_lane`: 車線別密度 `{'left': float, 'lcenter': float, ...}`

**戻り値**: UrgencyStateリスト

**実装**: 60-120行

**計算式**:
```python
s_norm = (s - s_entry) / (s_exit - s_entry)
U = min(1.0, s_norm ** gamma + alpha * rho_target)
```

---

## FrenetQPController

**ファイル**: [frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py)

### コンストラクタ

```python
def __init__(self, horizon: int = 80, dt: float = 0.1)
```

**パラメータ**:
- `horizon`: 予測ステップ数 (default: 80)
- `dt`: タイムステップ [s] (default: 0.1)

**導出属性**:
- `horizon_time`: 予測時間 [s] = `horizon * dt` = 8.0

**QP重み**:
```python
w_ref = 0.5      # 位置/速度追従
w_a = 10.0       # 加速度コスト
w_j = 2000.0     # Jerkコスト
```

### 主要メソッド

#### optimize()

```python
def optimize(self, ego_state: VehicleState, obstacles: List[ObstacleInfo],
             s_ref: Optional[np.ndarray] = None, v_ref: float = 20.0,
             urgency: float = 0.0, current_time: float = 0.0,
             previous_trajectory: Optional[Dict] = None
            ) -> Optional[Dict[str, np.ndarray]]
```

**説明**: QP軌道最適化

**パラメータ**:
- `ego_state`: 自車状態 (VehicleState)
- `obstacles`: 障害物リスト (List[ObstacleInfo])
- `s_ref`: 参照位置軌道 (Optional, default: None)
- `v_ref`: 参照速度 [m/s] (default: 20.0)
- `urgency`: 切迫度 [0, 1] (default: 0.0)
- `current_time`: 現在時刻 [s] (default: 0.0)
- `previous_trajectory`: 前回軌道（縫合用） (Optional, default: None)

**戻り値**: 最適軌道 or None
```python
{
    's': np.ndarray,  # 位置軌道 [N+1]
    'v': np.ndarray,  # 速度軌道 [N+1]
    'a': np.ndarray   # 加速度軌道 [N]
}
```

**実装**: 558-750行

**QP問題**:
```python
min  Σ[ w_ref*(s_k - s_ref)² + w_v*(v_k - v_ref)² + w_a*a_k² + w_j*(a_k+1 - a_k)² ]
s.t. s_lower[k] <= s_k <= s_upper[k]
     v_min <= v_k <= v_max
     a_min <= a_k <= a_max
```

---

#### compute_adaptive_safety_margin()

```python
def compute_adaptive_safety_margin(self, ego_state: VehicleState,
                                   front_vehicle: VehicleState,
                                   traffic_density: float, urgency: float
                                  ) -> float
```

**説明**: 適応的安全マージン計算

**パラメータ**:
- `ego_state`: 自車状態
- `front_vehicle`: 前方車両状態
- `traffic_density`: 交通密度 [veh/m]
- `urgency`: 切迫度 [0, 1]

**戻り値**: 時間ヘッドウェイ [s]

**実装**: 780-865行

**計算式**:
```python
T_base = 1.5  # ベース [s]

# Urgency-based緩和
T_urgency = T_base * (1.0 - 0.4 * urgency)

# 交通密度調整
density_factor = 0.85 if traffic_density > 0.05 else 1.0

# 相対速度調整
v_rel = max(0.0, ego.v - front.v)
velocity_factor = 1.2 if v_rel > 5.0 else 1.0

# 統合
T_final = max(0.9, T_urgency * density_factor * velocity_factor)
```

---

## Vehicle

**ファイル**: [vehicle.py](../weaving_v11/vehicle.py)

### 属性一覧

```python
@dataclass
class Vehicle:
    # 識別
    id: int

    # 物理状態
    x: float              # 縦方向位置（Frenet s） [m]
    y: float              # 横方向位置（Global y） [m]
    d: float              # 横方向位置（Frenet d） [m]
    v: float              # 速度 [m/s]
    ax: float             # 加速度指令 [m/s²]
    vy: float = 0.0       # 横速度（LC時） [m/s]

    # 車線状態
    lane: str             # 現在車線 ('left', 'lcenter', 'rcenter', 'right')
    target_lane: Optional[str] = None        # 目標車線
    target_lane_prep: Optional[str] = None   # 準備区間目標車線
    target_lane_weave: Optional[str] = None  # 織込区間目標車線

    # LC状態
    lc_scheduled: bool = False       # LC予約済み
    lc_started: bool = False         # LC実行中
    lc_completed: bool = False       # LC完了
    changing_lane: bool = False      # LC遷移中
    scheduled_time: Optional[float] = None   # LC開始予定時刻
    lc_start_time: Optional[float] = None    # LC開始時刻
    lc_duration: float = 3.0         # LC所要時間 [s]
    lc_start_x: float = 0.0          # LC開始位置 [m]
    lane_from: Optional[str] = None  # LC元車線
    lane_to: Optional[str] = None    # LC先車線

    # Urgency
    urgency: float = 0.0             # 切迫度 [0, 1]
    urgency_state: Optional[UrgencyState] = None  # Urgency詳細

    # 軌道情報
    velocity_profile: Optional[List[float]] = None           # 速度プロファイル
    predicted_trajectory: Optional[List[Tuple]] = None       # 予測軌道（CAV共有）
    previous_trajectory: Optional[Dict[str, np.ndarray]] = None  # 前回軌道（縫合用）

    # フラグ
    exited: bool = False             # 退出済み
    needs_initial_lc: bool = False   # 初期LC必要性
    is_hdv: bool = False             # HDVフラグ（デフォルトはCAV）

    # AEB状態
    aeb_active: bool = False         # AEB発動中

    # 協調制御
    lc_intent: Optional[Dict] = None          # LC意図ブロードキャスト
    cooperative_accel_request: float = 0.0    # 協調加速要求
    yield_start_time: Optional[float] = None  # Yield開始時刻
```

---

## IntegratedZoneParameters

**ファイル**: [parameters.py](../weaving_v11/parameters.py)

### 主要パラメータ

#### 道路構造

```python
prep_zone_length: float = 400.0      # 準備区間長 [m]
weave_zone_length: float = 500.0     # 織込区間長 [m]
warmup_length: float = 400.0         # ウォームアップ区間長 [m]
total_length: float = 900.0          # 総区間長 [m] (自動計算)
```

#### 制御周期

```python
dt_sim: float = 0.01                 # 物理シミュレーション [s] (100Hz)
dt_control: float = 0.1              # 制御更新 [s] (10Hz)
replan_interval: float = 0.5         # Level 1再計画 [s]
```

#### 車両パラメータ

```python
v_min: float = 5.0                   # 最低速度 [m/s]
v_max: float = 20.0                  # 最高速度 [m/s]
a_min: float = -6.0                  # 緊急ブレーキ [m/s²] (UN R157)
a_max: float = 2.0                   # 最大加速度 [m/s²]
L_vehicle: float = 5.0               # 車両長 [m]
```

#### Urgency関数

```python
urgency_gamma: float = 3.0           # 急峻度
urgency_alpha: float = 0.2           # 密度影響係数
```

#### LCトリガモデル（確率 = Urgency）

```python
# 現行実装: P_trigger = U(x_norm, rho)
# パラメータなし（Urgencyパラメータを共有）
lc_prep_duration: float = 5.0        # LC準備期間 [s]
```

#### Gap Acceptance

```python
min_front_gap: float = 15.0          # 最小前方ギャップ [m]
min_rear_gap: float = 20.0           # 最小後方ギャップ [m]
min_front_ttc: float = 3.0           # 最小前方TTC [s]
min_rear_ttc: float = 4.0            # 最小後方TTC [s]
```

#### Frenet QP

```python
horizon: int = 80                    # 予測ステップ数
dt: float = 0.1                      # タイムステップ [s]
w_ref: float = 0.5                   # 位置/速度追従重み
w_a: float = 10.0                    # 加速度コスト
w_j: float = 2000.0                  # Jerkコスト
```

#### 交通流パラメータ

```python
spawn_rate_left: float = 0.15        # LEFT側spawn率 [veh/s]
spawn_rate_right: float = 0.15       # RIGHT側spawn率 [veh/s]
v0_mean_normal: float = 20.0         # 通常時平均速度 [m/s]
v0_std_normal: float = 3.0           # 通常時速度標準偏差 [m/s]
```

### 負荷レベル設定

```python
def set_load_level(self, level: str) -> None
```

**パラメータ**:
- `level`: 'low', 'medium', 'high', 'congestion'

**設定例**:
```python
params = IntegratedZoneParameters()
params.set_load_level('medium')  # spawn_rate=0.60, v0_mean=14.0
```

---

## SafetyAnalyzer

**ファイル**: [vehicle.py](../weaving_v11/vehicle.py)

### 主要メソッド

#### compute_ttc()

```python
@staticmethod
def compute_ttc(s_rear: float, s_front: float, v_rear: float,
                v_front: float, L: float = 5.0) -> Optional[float]
```

**説明**: TTC（Time-to-Collision）計算

**パラメータ**:
- `s_rear`: 後方車両位置 [m]
- `s_front`: 前方車両位置 [m]
- `v_rear`: 後方車両速度 [m/s]
- `v_front`: 前方車両速度 [m/s]
- `L`: 車両長 [m] (default: 5.0)

**戻り値**: TTC [s] or None（追いつかない場合）

**計算式**:
```python
gap = s_front - s_rear - L
rel_v = v_rear - v_front

if rel_v > 0:  # 追いつく場合のみ
    ttc = gap / rel_v
else:
    ttc = None  # 追いつかない
```

---

## データ型

### VehicleState

```python
@dataclass
class VehicleState:
    id: int                    # 車両ID
    s: float                   # 縦方向位置（Frenet） [m]
    v: float                   # 速度 [m/s]
    a: float                   # 加速度 [m/s²]
    lane: str                  # 車線
```

### ObstacleInfo

```python
@dataclass
class ObstacleInfo:
    vehicle_state: VehicleState                # 障害物状態
    is_front: bool                             # 前方/後方フラグ
    predicted_trajectory: Optional[List]       # 予測軌道（CAV共有）
    is_changing_lane: bool = False             # LC中フラグ
    lateral_velocity: float = 0.0              # 横速度 [m/s]
    lane_relevance: str = "current"            # 車線関連性
```

### UrgencyState

```python
@dataclass
class UrgencyState:
    vehicle_id: int            # 車両ID
    urgency: float             # 切迫度 [0, 1]
    s_current: float           # 現在位置 [m]
    target_lane: str           # 目標車線
```

---

## まとめ：頻繁に参照するAPI

| 用途 | メソッド | ファイル:行 |
|------|---------|------------|
| **Urgency計算** | `UrgencyPlanner.compute_urgencies_batch()` | mpc_controller.py:60-120 |
| **QP最適化** | `FrenetQPController.optimize()` | frenet_qp_apollo.py:558-750 |
| **Gap検証** | `IntegratedZoneController._check_gap_acceptance()` | controllers.py:267-363 |
| **TTC計算** | `SafetyAnalyzer.compute_ttc()` | vehicle.py |
| **安全マージン** | `FrenetQPController.compute_adaptive_safety_margin()` | frenet_qp_apollo.py:780-865 |

---

**文書バージョン**: 1.0
**最終更新**: 2025-12-26
