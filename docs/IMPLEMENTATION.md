# Implementation Guide

> **関連ドキュメント**: [README](../README.md) | [ARCHITECTURE](ARCHITECTURE.md) | [ALGORITHMS](ALGORITHMS.md) | [API_REFERENCE](API_REFERENCE.md) | [APOLLO_ALIGNMENT](APOLLO_ALIGNMENT.md)

**Document Version**: 1.0
**Last Updated**: 2025-12-26
**System Version**: v15.0

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Code Structure](#code-structure)
3. [Common Tasks](#common-tasks)
4. [Best Practices](#best-practices)
5. [Advanced Topics](#advanced-topics)
6. [Testing](#testing)

---

## Getting Started

### Prerequisites

**Python Version**: 3.8+

**Required Packages**:
```bash
pip install numpy scipy matplotlib osqp
```

**Optional Packages**:
```bash
pip install pandas  # For data analysis
pip install seaborn  # For advanced visualization
```

### Directory Structure

```
CAV-weaving-control/
├── SYSTEM_SPECIFICATION.md       # Main documentation entry
├── docs/                          # Detailed documentation
│   ├── ARCHITECTURE.md
│   ├── API_REFERENCE.md
│   ├── ALGORITHMS.md
│   ├── HISTORY.md
│   ├── APOLLO_ALIGNMENT.md
│   ├── TROUBLESHOOTING.md
│   └── IMPLEMENTATION.md (this file)
├── weaving_v11/                   # Main package
│   ├── __init__.py
│   ├── main.py                    # Entry point
│   ├── simulator.py               # Simulation engine
│   ├── controllers.py             # Hierarchical control
│   ├── frenet_qp_apollo.py        # QP optimization
│   ├── mpc_controller.py          # Urgency planning
│   ├── dp_speed_optimizer.py      # DP optimizer
│   ├── vehicle.py                 # Vehicle class
│   ├── parameters.py              # Parameter definitions
│   ├── frenet_tools.py            # Frenet utilities
│   └── visualization.py           # Plotting utilities
└── outputs/                       # Simulation results
    ├── simulation_log_*.txt
    └── plots_*.png
```

### First Run

```bash
# Quick test (60 seconds)
python -m weaving_v11.main --load-level medium --mode l2 --debug

# Full simulation (600 seconds)
python -m weaving_v11.main --load-level medium --mode l2 --tmax 600

# High load stress test
python -m weaving_v11.main --load-level high --mode l2 --tmax 600
```

**Expected Output**:
```
=== Simulation Configuration ===
Version: v15.0 - Apollo Intelligent Control
...
=== Final Statistics (600.0s) ===
[Lane Change Metrics]
  LC Success Rate: 68-72%
...
```

---

## Code Structure

### 1. Main Entry Point

**File**: [weaving_v11/main.py](../weaving_v11/main.py)

**Structure**:
```python
def main():
    # 1. Parse command-line arguments
    args = parse_args()

    # 2. Create parameters
    params = create_parameters(args.load_level)

    # 3. Create simulator
    sim = IntegratedZoneSimulator(params)

    # 4. Run simulation
    results = sim.run(t_max=args.tmax)

    # 5. Print statistics
    sim.safety_analyzer.print_statistics(sim.t)
```

**Adding Custom Arguments**:
```python
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--custom-param', type=float, default=1.0,
                        help='Your custom parameter')
    # ... existing arguments
    args = parser.parse_args()
    return args
```

---

### 2. Simulation Loop

**File**: [weaving_v11/simulator.py](../weaving_v11/simulator.py)

**Core Loop**:
```python
def run(self, t_max: float = 600.0) -> Dict:
    """
    Main simulation loop

    Control frequencies:
    - Physics update: 100 Hz (dt_sim = 0.01s)
    - Control update: 10 Hz (dt_control = 0.1s)
    - Spawn check: Variable (Poisson process)
    """
    while self.t < t_max:
        # 1. Spawn vehicles (Poisson process)
        if self._should_spawn_vehicle(self.t):
            self._spawn_vehicle(self.t)

        # 2. Control update (10 Hz)
        if self.t - self.last_control_update >= self.dt_control:
            self._control_update(self.t)
            self.last_control_update = self.t

        # 3. Physics update (100 Hz, 10 substeps per control cycle)
        for _ in range(int(self.dt_control / self.dt_sim)):
            self._update_vehicles(self.dt_sim)
            self.t += self.dt_sim

    return self._get_results()
```

**Key Design Principles**:
1. **Separation of control (10 Hz) and physics (100 Hz)**: Apollo standard
2. **Sequential vehicle update**: Front vehicles plan first (line 427-461)
3. **Lazy spawning**: Poisson process for realistic arrivals

---

### 3. Hierarchical Control

**File**: [weaving_v11/controllers.py](../weaving_v11/controllers.py)

**Two-Level Architecture**:

```python
class IntegratedZoneController:
    """
    Level 1: Strategic Planning (0.5s cycle)
        └─ UrgencyPlanner: Urgency + Probabilistic LC trigger

    Level 2: Tactical Control (0.1s cycle)
        └─ FrenetQPController: ST-Boundary + QP optimization
    """

    def control_update(self, t: float, active_vehicles: List[Vehicle]) -> None:
        # Sort vehicles by s-coordinate (front first)
        sorted_vehicles = sorted(active_vehicles, key=lambda v: v.x, reverse=True)

        for v in sorted_vehicles:
            # === LEVEL 1: STRATEGIC PLANNING ===
            if t - v.last_replan >= self.params.replan_interval:
                urgency = self.urgency_planner.compute_urgency(v, active_vehicles)
                v.last_replan = t

                # Probabilistic LC trigger
                if self._should_attempt_lane_change(v, urgency, t):
                    if self._check_gap_acceptance(v, active_vehicles):
                        v.lane_change_scheduled = True

            # === LEVEL 2: TACTICAL CONTROL ===
            ego_state = self._create_vehicle_state(v)
            obstacles = self._detect_obstacles_in_frenet(v, active_vehicles)

            result = self.frenet_controller.optimize(
                ego_state=ego_state,
                obstacles=obstacles,
                urgency=urgency,
                current_time=t,
                previous_trajectory=getattr(v, 'previous_trajectory', None)
            )

            if result:
                v.ax = result['a'][0]
                v.previous_trajectory = result  # Store for next cycle
            else:
                # Fallback: Emergency braking
                v.ax = -6.0
```

**Implementation Notes**:
- Line 1154-1424: Main control loop
- Line 267-363: Gap acceptance logic
- Line 743-985: Frenet obstacle detection
- Line 1425-1606: Safety mechanisms (AEB, RSS)

---

### 4. QP Optimization

**File**: [weaving_v11/frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py)

**QP Formulation**:

```python
def optimize(self, ego_state, obstacles, s_ref, v_ref, urgency,
             current_time=0.0, previous_trajectory=None):
    """
    Piecewise Jerk Speed Optimization (Apollo-aligned)

    Decision variables: [s_0, ..., s_N-1, v_0, ..., v_N-1, a_0, ..., a_N-1, j_0, ..., j_N-1]
    Total: 4N variables (N=80 for 8-second horizon)

    Cost function:
        J = Σ [w_s(s-s_ref)² + w_v(v-v_ref)² + w_a·a² + w_j·j²]

    Constraints:
        - Dynamics: s[k+1] = s[k] + v[k]·dt + 0.5·a[k]·dt²
        - Kinematics: v[k+1] = v[k] + a[k]·dt
        - Jerk: a[k+1] = a[k] + j[k]·dt
        - Bounds: v_min ≤ v[k] ≤ v_max, a_min ≤ a[k] ≤ a_max, j_min ≤ j[k] ≤ j_max
        - ST-Boundary: s_lower[k] ≤ s[k] ≤ s_upper[k]
    """
    # 1. Trajectory stitching (Apollo TrajectoryStitcher)
    if previous_trajectory:
        ego_state = self._stitch_trajectory(ego_state, previous_trajectory)

    # 2. Decision making (Apollo DecisionMaker)
    decisions = self._make_interaction_decisions(ego_state, obstacles, current_time)

    # 3. Pre-DP gap check (Apollo LaneChangeDecider::IsPerceptionBlocked)
    if not self._pre_dp_gap_check(ego_state, obstacles):
        # Path blocked, force all YIELD
        decisions = {obs.id: InteractionDecision.YIELD for obs in obstacles}

    # 4. DP optimizer (Apollo DpStSpeedOptimizer)
    if urgency > 0.5:  # High urgency
        s_ref_dp = self._run_dp_optimizer(ego_state, obstacles, decisions)
        if s_ref_dp is not None:
            s_ref = s_ref_dp

    # 5. ST-Boundary construction (Apollo SpeedBoundsDecider)
    st_boundary = self._build_st_boundary(obstacles, decisions)

    # 6. QP solver (Apollo PiecewiseJerkSpeedOptimizer)
    P, q = self._build_cost_matrices(s_ref, v_ref)
    G, h = self._build_inequality_constraints(st_boundary)
    A, b = self._build_equality_constraints(ego_state)

    solution = self.qp_solver.solve(P, q, G, h, A, b)

    if solution.info.status == 'solved':
        return self._extract_trajectory(solution.x)
    else:
        # Fallback: Emergency braking
        return self._fallback_emergency_braking(ego_state)
```

**Key Methods**:
- Line 558-750: Main `optimize()` method
- Line 867-975: ST-Boundary construction
- Line 780-865: Adaptive safety margin (v15.0)
- Line 1291-1355: Trajectory stitching
- Line 1343-1401: Pre-DP gap check

---

## Architecture Roadmap

本プロジェクトは仕様上はモジュール化されていますが、`weaving_v11/controllers.py` が肥大化（~2.4k行）しやすく、ロジックの重複（TTC/RSS/AEB計算）と責務混在（検知/計画/安全/最適化/実行）が「スパゲティ化」の主因です。ここでは段階的にリスクを抑えつつ整理する計画を示します。

### 階層構造（目標像）
- Domain Models: `vehicle.py`, `vehicle_state.py`, `parameters.py`
- Perception/Env: `frenet_tools.py`（障害物抽出・座標変換）
- Planning
  - Strategic (L1): `mpc_controller.py`（Urgency・LC意思決定）
  - Tactical (L2): `frenet_qp_apollo.py`（ST境界・QP）
- Safety Manager: AEB/RSS/TTC（新規: `safety.py` または `safety_manager.py`）
- Orchestrator: `controllers.py`（薄い統合レイヤ）
- Diagnostics: `visualization.py`, ログ出力、統計集計

### 現状ホットスポット
- `controllers.py` が長大で条件分岐が深い
- TTC/AEB計算が複数箇所で重複・微妙に不一致（`v_front` 参照の揺れ等）
- パラメータが散在（快適/緊急減速度、RSS係数、反応時間など）
- 状態フラグ（`aeb_active`, `lc_scheduled` 等）の更新が多層で行われる

### フェーズ分割リファクタ
1) Safety抽出（影響最小・効果大）
- 新規 `weaving_v11/safety.py` を追加し、以下の関数を集約:
  - `compute_ttc(v_ego, v_front, gap)`
  - `required_decel(rel_v, gap, caps)`
  - `rss_safe_distance(v_ego, v_front, params)`
  - `evaluate_aeb(ego, front, gap, context) -> AebCmd(ax, reason, severity)`
- 既存のAEB/RSS判定箇所を上記APIに置換（ふるまい同等をゴール）。
- 受け渡しは `VehicleState`/`Obstacle` など軽量DTOで統一。

2) Perceptionの境界明確化
- Frenet障害物抽出を `frenet_tools.py` へ集約し、
  `detect_front_obstacle(ego, world) -> Optional[Obstacle]` 提供。
- `controllers.py` から生配列アクセスを排し、型安全な構造体に統一。

3) Controllersの薄型化
- `IntegratedZoneController` は「オーケストレーション専用」へ縮退。
- L1（戦略: `mpc_controller.py`）とL2（戦術: `frenet_qp_apollo.py`）の
  インターフェースを明示（入出力DTO, 例外, ログ方針）。

4) パラメータ集中管理
- 安全/快適/RSS/反応時間などを `parameters.py` で名前空間化。
- マジックナンバーの排除とテスト時の上書き手段提供。

5) 型とテスト
- 公開APIに型注釈を付与（`Vehicle`, `VehicleState`, `Obstacle`, `AebCmd`）。
- 単体テスト: TTC, RSS, AEB境界、ギャップ算定（バンパー基準）。
- 統合テスト: 代表シナリオ（接近/合流/渋滞）で回帰ログ比較。

### 完了の定義（DoD）
- `controllers.py` 行数: 2400→<800（Phase1-3）、最終<400
- 重複TTC/AEBロジック: 0箇所（`safety.py` のみ）
- 静的検査: flake8/mypy で重大警告0（既知除外を明示管理）
- テスト: safety周りのユニットテスト10件以上、簡易回帰テスト1件

### クイックウィン（即日対応可能）
- TTC/減速度の共通ユーティリティ導入（呼び出し置換のみ）
- `v_front` 命名統一（`front_vehicle.v`/`v_front` の混在解消）
- ログ出力の共通prefix（`[AEB-*]`, `[RSS-*]`）& 重要情報の標準化

---

## Common Tasks

### Task 1: Modify Parameters

**File**: [weaving_v11/parameters.py](../weaving_v11/parameters.py)

**Example: Change gap acceptance thresholds**

```python
@dataclass
class IntegratedZoneParameters:
    # ... existing parameters ...

    # Gap Acceptance (MODIFY HERE)
    min_front_gap: float = 12.0  # Original: 15.0m
    min_rear_gap: float = 15.0   # Original: 20.0m
    min_front_ttc: float = 2.5   # Original: 3.0s
    min_rear_ttc: float = 3.5    # Original: 4.0s
```

**After modification**:
```bash
python -m weaving_v11.main --load-level medium --mode l2 --tmax 600
# Check if LC success rate improved
```

---

### Task 2: Add New Vehicle Behavior

**File**: [weaving_v11/vehicle.py](../weaving_v11/vehicle.py)

**Example: Add fuel consumption tracking**

```python
@dataclass
class Vehicle:
    # ... existing fields ...

    # New field
    fuel_consumed: float = 0.0  # [liters]

    def update_fuel_consumption(self, dt: float):
        """
        Simple fuel model: consumption proportional to acceleration

        Fuel rate = base_rate + k * |acceleration|
        """
        base_rate = 0.1  # [L/s] idle consumption
        k = 0.05  # [L/s per m/s²] acceleration penalty

        fuel_rate = base_rate + k * abs(self.ax)
        self.fuel_consumed += fuel_rate * dt
```

**Integrate in simulator**:

```python
# In simulator.py, _update_vehicles()
def _update_vehicles(self, dt: float):
    for v in self.vehicles:
        # ... existing update logic ...

        # Add fuel tracking
        v.update_fuel_consumption(dt)
```

**Log results**:

```python
# In safety_analyzer.py, print_statistics()
total_fuel = sum(v.fuel_consumed for v in vehicles)
print(f"Total Fuel Consumed: {total_fuel:.2f} L")
print(f"Fuel per Vehicle: {total_fuel / len(vehicles):.2f} L/vehicle")
```

---

### Task 3: Implement Custom Cost Function

**File**: [weaving_v11/frenet_qp_apollo.py](../weaving_v11/frenet_qp_apollo.py)

**Example: Add energy efficiency cost**

```python
def _build_cost_matrices(self, s_ref, v_ref):
    """
    Build QP cost matrices P and q

    Original cost:
        J = Σ [w_s(s-s_ref)² + w_v(v-v_ref)² + w_a·a² + w_j·j²]

    New cost (add energy term):
        J = Σ [... + w_e·(a·v)²]
        where a·v is power (energy rate)
    """
    N = self.N
    n_vars = 4 * N

    # ... existing cost terms ...

    # NEW: Energy efficiency cost
    w_e = 10.0  # Energy weight

    for k in range(N):
        v_idx = N + k  # Velocity index
        a_idx = 2 * N + k  # Acceleration index

        # Approximate power cost: (a·v)² ≈ a²·v_ref²
        # (Linearization for quadratic form)
        P[a_idx, a_idx] += w_e * (v_ref ** 2)

    return P, q
```

---

### Task 4: Add Custom Logging

**File**: [weaving_v11/controllers.py](../weaving_v11/controllers.py)

**Example: Log detailed decision-making**

```python
def _make_interaction_decisions(self, ego_state, obstacles, current_time):
    """
    Enhanced with detailed logging
    """
    decisions = {}

    for obs in obstacles:
        # Compute decision
        decision = self._compute_single_decision(ego_state, obs)

        # LOG DETAILS
        logger.info(f"[DECISION-DETAIL] Ego s={ego_state.s:.1f}, v={ego_state.v:.1f}")
        logger.info(f"[DECISION-DETAIL] Obs {obs.id}: s={obs.s:.1f}, v={obs.v:.1f}, lane={obs.lane_relevance}")
        logger.info(f"[DECISION-DETAIL] Distance: {abs(obs.s - ego_state.s):.1f}m")
        logger.info(f"[DECISION-DETAIL] Decision: {decision}")

        decisions[obs.id] = decision

    return decisions
```

**Extract logs**:
```bash
grep "\[DECISION-DETAIL\]" simulation_log.txt > decision_log.txt
```

---

### Task 5: Implement New Safety Feature

**File**: [weaving_v11/controllers.py](../weaving_v11/controllers.py)

**Example: Add lane departure warning**

```python
class IntegratedZoneController:
    def __init__(self, params):
        # ... existing initialization ...
        self.ldw_threshold = 0.5  # [m] lateral offset threshold

    def _check_lane_departure_warning(self, v: Vehicle) -> bool:
        """
        Lane Departure Warning (LDW)

        Returns True if vehicle is drifting out of lane
        """
        # Assuming lane width = 4.0m, center at d = 0.0
        lane_center_d = 0.0
        lane_width = 4.0

        # Compute lateral offset from lane center
        lateral_offset = abs(v.y - (v.lane * lane_width))

        if lateral_offset > self.ldw_threshold:
            logger.warning(f"[LDW] Vehicle {v.id}: lateral offset {lateral_offset:.2f}m")
            return True

        return False

    def control_update(self, t, active_vehicles):
        for v in active_vehicles:
            # ... existing control logic ...

            # Add LDW check
            if self._check_lane_departure_warning(v):
                # Take corrective action (e.g., apply lateral correction)
                pass
```

---

### Task 6: Create Custom Scenario

**File**: Create new file `weaving_v11/scenarios.py`

```python
"""
Custom scenario definitions
"""

from dataclasses import dataclass
from weaving_v11.parameters import IntegratedZoneParameters

@dataclass
class CustomScenario:
    """Highway merging scenario with bottleneck"""
    name: str = "highway_bottleneck"

    def create_parameters(self) -> IntegratedZoneParameters:
        params = IntegratedZoneParameters()

        # Geometry
        params.total_length = 800.0  # Longer section
        params.num_lanes = 3  # 3 lanes instead of 2

        # Traffic load (HIGH)
        params.spawn_interval_mean = 4.0  # Spawn every 4s (very dense)
        params.spawn_interval_std = 1.0

        # Tighter gap acceptance (challenging)
        params.min_front_gap = 10.0  # Aggressive
        params.min_rear_gap = 15.0

        return params

# Usage
from weaving_v11.scenarios import CustomScenario
from weaving_v11.simulator import IntegratedZoneSimulator

scenario = CustomScenario()
params = scenario.create_parameters()
sim = IntegratedZoneSimulator(params)
sim.run(t_max=600)
```

---

## Best Practices

### 1. Code Organization

**DO**:
```python
# Clear separation of concerns
class FrenetQPController:
    def optimize(self, ...):
        # High-level orchestration
        decisions = self._make_decisions(...)
        st_boundary = self._build_st_boundary(...)
        solution = self._solve_qp(...)
        return self._extract_trajectory(solution)

    def _make_decisions(self, ...):
        # Single responsibility
        pass
```

**DON'T**:
```python
# Monolithic method with 500+ lines
def optimize(self, ...):
    # Decision making
    # ST-Boundary construction
    # QP setup
    # Solving
    # Extraction
    # All in one method
```

---

### 2. Parameter Management

**DO**:
```python
# Centralized in parameters.py
@dataclass
class IntegratedZoneParameters:
    min_front_gap: float = 15.0  # [m] Documented with units
    min_rear_gap: float = 20.0   # [m]
```

**DON'T**:
```python
# Magic numbers scattered throughout code
if gap < 15.0:  # What is 15.0? Why 15.0?
    return False
```

---

### 3. Error Handling

**DO**:
```python
def optimize(self, ...):
    try:
        solution = self.qp_solver.solve(P, q, G, h, A, b)
        if solution.info.status != 'solved':
            logger.warning(f"QP status: {solution.info.status}")
            return None
    except Exception as e:
        logger.error(f"QP solver error: {e}")
        return None

    return self._extract_trajectory(solution.x)
```

**DON'T**:
```python
def optimize(self, ...):
    solution = self.qp_solver.solve(P, q, G, h, A, b)
    # No error handling, will crash on failure
    return self._extract_trajectory(solution.x)
```

---

### 4. Logging

**DO**:
```python
import logging
logger = logging.getLogger(__name__)

# Structured logging with levels
logger.debug(f"[QP-SETUP] Building cost matrices: N={self.N}")
logger.info(f"[QP-RESULT] Status: {solution.info.status}")
logger.warning(f"[QP-WARN] Infeasible: using fallback")
logger.error(f"[QP-ERROR] Solver crashed: {e}")
```

**DON'T**:
```python
# Unstructured print statements
print("Building cost matrices")
print(f"Status: {solution.info.status}")
```

---

### 5. Testing

**DO**:
```python
# Unit tests for individual components
def test_urgency_calculation():
    params = IntegratedZoneParameters()
    planner = UrgencyPlanner(params)

    # Test at entry (x=0)
    vehicle = Vehicle(x=0.0, ...)
    urgency = planner.compute_urgency(vehicle, [])
    assert 0.0 <= urgency <= 0.1  # Should be low

    # Test at exit (x=L)
    vehicle.x = params.total_length
    urgency = planner.compute_urgency(vehicle, [])
    assert 0.9 <= urgency <= 1.0  # Should be high
```

**DON'T**:
```python
# Only test full simulation (hard to debug)
def test_everything():
    sim = IntegratedZoneSimulator(params)
    results = sim.run(t_max=600)
    assert results['collision_count'] == 0
```

---

### 6. Documentation

**DO**:
```python
def compute_safe_distance_rss(self, v_ego: float, v_front: float,
                               time_headway: float = 1.5) -> float:
    """
    Compute RSS-compliant safe distance

    Formula (ISO 21934):
        d_safe = s0 + v_ego * T + (v_ego * (v_ego - v_front)) / (2 * sqrt(a_max * b_max))

    Args:
        v_ego: Ego vehicle velocity [m/s]
        v_front: Front vehicle velocity [m/s]
        time_headway: Time headway [s], typically 0.9-1.5s

    Returns:
        Safe distance [m]

    References:
        - ISO 21934 (RSS standard)
        - Apollo safety_manager.cc
    """
    # Implementation
```

**DON'T**:
```python
def compute_safe_distance_rss(self, v_ego, v_front, time_headway=1.5):
    # Compute safe distance
    # (No explanation of formula, units, or references)
```

---

## Advanced Topics

### 1. Warm-Starting QP Solver

Warm-starting can significantly speed up QP solving by using previous solution as initial guess.

**Implementation**:

```python
class FrenetQPController:
    def __init__(self, params):
        # ... existing initialization ...
        self.previous_solution = None  # Store previous solution

    def optimize(self, ...):
        # ... build P, q, G, h, A, b ...

        # Warm-start if previous solution exists
        if self.previous_solution is not None:
            x_init = self.previous_solution
            solution = self.qp_solver.solve(
                P, q, G, h, A, b,
                x_init=x_init  # Warm-start
            )
        else:
            solution = self.qp_solver.solve(P, q, G, h, A, b)

        # Store for next iteration
        if solution.info.status == 'solved':
            self.previous_solution = solution.x

        return solution
```

**Expected Speedup**: 2-5x faster convergence

---

### 2. Multi-Threading for Large-Scale Simulation

For simulations with 50+ vehicles, parallelize vehicle control updates.

**Implementation**:

```python
from concurrent.futures import ThreadPoolExecutor

class IntegratedZoneController:
    def __init__(self, params):
        # ... existing initialization ...
        self.executor = ThreadPoolExecutor(max_workers=4)

    def control_update_parallel(self, t: float, active_vehicles: List[Vehicle]):
        """
        Parallel control update (experimental)

        WARNING: Requires careful handling of shared state
        """
        # Sort vehicles (sequential dependency)
        sorted_vehicles = sorted(active_vehicles, key=lambda v: v.x, reverse=True)

        # Group vehicles by lane (independent groups can be parallelized)
        lane_groups = {}
        for v in sorted_vehicles:
            if v.lane not in lane_groups:
                lane_groups[v.lane] = []
            lane_groups[v.lane].append(v)

        # Process each lane in parallel
        futures = []
        for lane, vehicles in lane_groups.items():
            future = self.executor.submit(self._control_update_lane, t, vehicles)
            futures.append(future)

        # Wait for all to complete
        for future in futures:
            future.result()
```

**Caution**: Ensure no race conditions on shared data structures.

---

### 3. Adaptive Prediction Horizon

Dynamically adjust prediction horizon based on traffic density.

**Implementation**:

```python
def _compute_adaptive_horizon(self, ego_state, obstacles, traffic_density):
    """
    Adaptive horizon: longer in sparse traffic, shorter in dense

    Rationale:
    - Dense traffic: Short horizon (3s) for reactivity
    - Sparse traffic: Long horizon (8s) for smoothness
    """
    if traffic_density > 0.3:
        # Dense: short horizon
        horizon_steps = 30  # 3.0s
    elif traffic_density > 0.15:
        # Medium: standard horizon
        horizon_steps = 50  # 5.0s
    else:
        # Sparse: long horizon
        horizon_steps = 80  # 8.0s (v15.0 default)

    return horizon_steps
```

**Trade-off**: Complexity vs performance gain (may not be worth it)

---

### 4. Learning-Based Parameter Tuning

Use Bayesian optimization to tune parameters automatically.

**Example**:

```python
from skopt import gp_minimize
from skopt.space import Real

def objective_function(params_tuple):
    """
    Objective: Maximize LC success rate, minimize collisions

    Returns: -score (minimize negative score = maximize score)
    """
    min_front_gap, urgency_gamma = params_tuple

    # Create parameters
    params = IntegratedZoneParameters()
    params.min_front_gap = min_front_gap
    params.urgency_gamma = urgency_gamma  # P_trigger = U(x)^gamma

    # Run simulation
    sim = IntegratedZoneSimulator(params)
    results = sim.run(t_max=300)  # Shorter for tuning

    # Compute score
    lc_success_rate = results['lc_success_rate']
    collision_count = results['collision_count']

    score = lc_success_rate - 0.1 * collision_count
    return -score  # Minimize negative

# Define search space
space = [
    Real(10.0, 20.0, name='min_front_gap'),
    Real(2.5, 3.8, name='urgency_gamma')
]

# Optimize
result = gp_minimize(objective_function, space, n_calls=50)

print(f"Optimal min_front_gap: {result.x[0]:.2f}")
print(f"Optimal urgency_gamma: {result.x[1]:.2f}")
```

---

### 5. Integration with External Controllers

Interface with ROS or other control systems.

**Example: ROS Publisher**

```python
import rospy
from geometry_msgs.msg import Twist

class ROSIntegratedController(IntegratedZoneController):
    def __init__(self, params):
        super().__init__(params)

        # ROS setup
        rospy.init_node('cav_weaving_controller')
        self.publishers = {}

    def control_update(self, t, active_vehicles):
        # Standard control update
        super().control_update(t, active_vehicles)

        # Publish control commands to ROS
        for v in active_vehicles:
            if v.id not in self.publishers:
                self.publishers[v.id] = rospy.Publisher(
                    f'/vehicle_{v.id}/cmd_vel', Twist, queue_size=1
                )

            # Create Twist message
            msg = Twist()
            msg.linear.x = v.vx
            msg.linear.y = 0.0
            msg.angular.z = 0.0  # No steering in 1D model

            # Publish
            self.publishers[v.id].publish(msg)
```

---

## Testing

### Unit Tests

**File**: Create `tests/test_urgency.py`

```python
import unittest
import numpy as np
from weaving_v11.mpc_controller import UrgencyPlanner
from weaving_v11.parameters import IntegratedZoneParameters
from weaving_v11.vehicle import Vehicle

class TestUrgencyPlanner(unittest.TestCase):
    def setUp(self):
        self.params = IntegratedZoneParameters()
        self.planner = UrgencyPlanner(self.params)

    def test_urgency_at_entry(self):
        """Urgency should be low at entry"""
        v = Vehicle(id=1, x=0.0, y=0.0, vx=15.0, lane=0)
        urgency = self.planner.compute_urgency(v, [])

        self.assertGreaterEqual(urgency, 0.0)
        self.assertLess(urgency, 0.15)  # Should be low

    def test_urgency_at_exit(self):
        """Urgency should be high near exit"""
        v = Vehicle(id=1, x=self.params.total_length * 0.95, y=0.0, vx=15.0, lane=0)
        urgency = self.planner.compute_urgency(v, [])

        self.assertGreater(urgency, 0.8)  # Should be high
        self.assertLessEqual(urgency, 1.0)

    def test_urgency_exponential_growth(self):
        """Urgency should grow exponentially (γ=3.0)"""
        positions = [0.0, 0.25, 0.5, 0.75, 1.0]
        urgencies = []

        for x_norm in positions:
            v = Vehicle(id=1, x=x_norm * self.params.total_length, y=0.0, vx=15.0, lane=0)
            urgency = self.planner.compute_urgency(v, [])
            urgencies.append(urgency)

        # Check exponential growth: U(0.5) << U(0.75) << U(1.0)
        self.assertLess(urgencies[2], 0.3)  # U(0.5) < 0.3
        self.assertGreater(urgencies[3], 0.5)  # U(0.75) > 0.5
        self.assertGreater(urgencies[4], 0.9)  # U(1.0) > 0.9

if __name__ == '__main__':
    unittest.main()
```

**Run tests**:
```bash
python -m pytest tests/test_urgency.py -v
```

---

### Integration Tests

**File**: Create `tests/test_integration.py`

```python
import unittest
from weaving_v11.simulator import IntegratedZoneSimulator
from weaving_v11.parameters import IntegratedZoneParameters

class TestIntegration(unittest.TestCase):
    def test_full_simulation_medium_load(self):
        """Full simulation should complete without crashes"""
        params = IntegratedZoneParameters()
        params.spawn_interval_mean = 8.0  # MEDIUM load

        sim = IntegratedZoneSimulator(params)
        results = sim.run(t_max=60)  # Short test

        # Basic sanity checks
        self.assertGreater(results['total_spawned'], 0)
        self.assertGreaterEqual(results['collision_count'], 0)

    def test_collision_free_sparse_traffic(self):
        """Sparse traffic should have zero collisions"""
        params = IntegratedZoneParameters()
        params.spawn_interval_mean = 20.0  # Very sparse

        sim = IntegratedZoneSimulator(params)
        results = sim.run(t_max=300)

        self.assertEqual(results['collision_count'], 0)

    def test_lc_success_rate_acceptable(self):
        """LC success rate should be > 60%"""
        params = IntegratedZoneParameters()

        sim = IntegratedZoneSimulator(params)
        results = sim.run(t_max=600)

        lc_success_rate = results['lc_success_rate']
        self.assertGreater(lc_success_rate, 0.6)

if __name__ == '__main__':
    unittest.main()
```

---

### Performance Benchmarking

**File**: Create `benchmark/benchmark.py`

```python
import time
from weaving_v11.simulator import IntegratedZoneSimulator
from weaving_v11.parameters import IntegratedZoneParameters

def benchmark_simulation():
    params = IntegratedZoneParameters()

    print("=== Performance Benchmark ===")

    # Test different loads
    loads = {
        'LOW': 15.0,
        'MEDIUM': 8.0,
        'HIGH': 5.0
    }

    for load_name, spawn_interval in loads.items():
        params.spawn_interval_mean = spawn_interval

        sim = IntegratedZoneSimulator(params)

        start_time = time.time()
        results = sim.run(t_max=600)
        elapsed_time = time.time() - start_time

        print(f"\n{load_name} Load:")
        print(f"  Real time: {elapsed_time:.1f}s")
        print(f"  Sim time: 600.0s")
        print(f"  Speed ratio: {600.0 / elapsed_time:.2f}x")
        print(f"  Vehicles spawned: {results['total_spawned']}")
        print(f"  Collisions: {results['collision_count']}")
        print(f"  LC success rate: {results['lc_success_rate']:.1%}")

if __name__ == '__main__':
    benchmark_simulation()
```

**Expected Output**:
```
=== Performance Benchmark ===

LOW Load:
  Real time: 45.2s
  Sim time: 600.0s
  Speed ratio: 13.27x
  Vehicles spawned: 42
  Collisions: 0
  LC success rate: 75.3%

MEDIUM Load:
  Real time: 89.6s
  Sim time: 600.0s
  Speed ratio: 6.70x
  Vehicles spawned: 78
  Collisions: 5
  LC success rate: 68.4%

HIGH Load:
  Real time: 156.3s
  Sim time: 600.0s
  Speed ratio: 3.84x
  Vehicles spawned: 125
  Collisions: 18
  LC success rate: 62.1%
```

---

## Debugging Workflow

### 1. Identify Problem

```bash
# Run simulation
python -m weaving_v11.main --load-level medium --mode l2 --tmax 600

# Check final statistics
grep "Final Statistics" simulation_log.txt -A 30
```

### 2. Isolate Issue

```bash
# Focus on specific problem
grep "QP status: primal_infeasible" simulation_log.txt | wc -l
# High count → QP problem

grep "\[COLLISION-DETECTED\]" simulation_log.txt | wc -l
# High count → Safety problem
```

### 3. Enable Debug Logging

```python
# In relevant module
import logging
logging.basicConfig(level=logging.DEBUG)  # Enable all logs
```

### 4. Add Targeted Logging

```python
# In frenet_qp_apollo.py
if solution.info.status != 'solved':
    logger.debug(f"[QP-DEBUG] P matrix condition number: {np.linalg.cond(P)}")
    logger.debug(f"[QP-DEBUG] Constraint violations: {np.max(G @ x - h)}")
```

### 5. Visualize Problem

```python
# Enable ST-Graph visualization
sim.controller.frenet_controller.enable_st_visualization = True
```

### 6. Test Fix

```bash
# Re-run with fix
python -m weaving_v11.main --load-level medium --mode l2 --tmax 600

# Verify improvement
grep "QP Success Rate" simulation_log.txt
```

---

## Conclusion

This implementation guide provides practical examples and best practices for working with the CAV Weaving Control system. For theoretical details, see:

- [ARCHITECTURE.md](ARCHITECTURE.md) - System design
- [ALGORITHMS.md](ALGORITHMS.md) - Mathematical formulations
- [API_REFERENCE.md](API_REFERENCE.md) - Complete API documentation
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Common issues and solutions

---

**Document Maintained By**: CAV Weaving Control Team
**Last Updated**: 2025-12-26

---

**End of Implementation Guide**
