# -*- coding: utf-8 -*-
"""
weaving_v11/parameters.py

IntegratedZoneParameters: 統合ゾーン制御システムのメインパラメータデータクラス

区間構成:
- Spawn区間: [-400, 0] (warmup_length)
- 準備LC区間: [0, 500] (prep_zone_length)
- 織り込み区間: [500, 1000] (weave_zone_length)
- LCは x >= 0 のみ許可

パラメータカテゴリ:
- 区間設定: ゾーン長、セルサイズ
- 車両力学: 加速度・速度制限
- 安全パラメータ: RSS係数、AEB閾値
- LCパラメータ: ギャップ要件、実行時間
- QP重み: 加速度、ジャーク
- spawn設定: 負荷レベル別の生成率
"""

from dataclasses import dataclass

@dataclass
class IntegratedZoneParameters:

    # --- modified (v18.21): 区間設定変更 ---
    # Spawn区間: [-400, 0] (warmup_length)
    # 準備LC区間: [0, 500] (prep_zone_length)
    # 織り込み区間: [500, 1000] (weave_zone_length)
    # LCは x >= 0 のみ許可
    # 基本区間設定(ここだけ変更すればOK)
    prep_zone_length: float = 500.0   # v18.21: 400→500 (準備LC区間[0,500])
    weave_zone_length: float = 500.0  # 織り込み区間[500,1000]
    warmup_length: float = 400.0
    cell_length: float = 10.0  # セルの長さ(固定)

    # 以下は__post_init__で自動計算される(手動変更不要)
    total_length: float = 0.0  # prep + weave (自動計算)
    num_cells_prep: int = 0  # prep_zone_length / cell_length (自動計算)
    num_cells_weave: int = 0  # weave_zone_length / cell_length (自動計算)
    num_cells_total: int = 0  # num_cells_prep + num_cells_weave (自動計算)
    # ---

    def __post_init__(self):
        self.total_length = self.prep_zone_length + self.weave_zone_length
        self.num_cells_prep = int(self.prep_zone_length / self.cell_length)
        self.num_cells_weave = int(self.weave_zone_length / self.cell_length)
        self.num_cells_total = self.num_cells_prep + self.num_cells_weave
        # v11.0: Gurobi is no longer used (replaced by OSQP), skip thread setup
        if self.debug_mode:
            print("DEBUG MODE ACTIVATED")
            print(f"  Simulation time: 600s -> {self.debug_tmax}s")
            print(f"  Spawn rate factor: x {self.debug_spawn_rate_factor}")
            print(f"  Early stop at: {self.debug_early_stop_collisions} collisions")
            print(f"  Max time: {self.debug_early_stop_time}s")
            print(f"  Level2 debug: {self.enable_level2_unified_debug}")
            if hasattr(self, 'spawn_rate'):
                original_spawn = self.spawn_rate
                self.spawn_rate *= self.debug_spawn_rate_factor
                self.spawn_rate_left *= self.debug_spawn_rate_factor
                self.spawn_rate_right *= self.debug_spawn_rate_factor
                print(f"[DEBUG] Spawn rate: {original_spawn:.3f} -> {self.spawn_rate:.3f} veh/s")
            if not self.enable_level2_unified_debug:
                self.enable_level2_unified_debug = True
                print("[DEBUG] Auto-enabled: enable_level2_unified_debug=True")

    @property
    def total_sim_length(self) -> float:
        return float(self.warmup_length + self.prep_zone_length + self.weave_zone_length)
    
    v_min: float = 5.0
    v_max: float = 20.0

    # --- modified (Final Fix v11.3 / 2025-12-17): Emergency braking capability ---
    # Rationale: -3.0 m/s² is too weak for AEB (Automatic Emergency Braking)
    # - UN R157 (ALKS): Emergency deceleration up to -6.0 m/s² is allowed
    # - Apollo AEB: Uses -5.0 to -6.0 m/s² for collision avoidance
    # - This allows the controller to prevent imminent collisions
    a_min: float = -6.0  # Emergency braking capability (UN R157 compliant)
    # -------------------------------------------------------------------------------

    a_max: float = 2.0
    S_min: float = 5.0
    #   - 空間分散が相対的に重視される(寄与度を1.5% → 50%に改善)
    #   - LC成功も保証される(lambda_soft=50.0は十分大きい)

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    alpha_prep: float = 93.3868046174258         # v7.27 ベスト解 (Best)
    beta_prep: float = 5.737959630044404          # v7.27 ベスト解
    lambda_soft_prep: float = 3633.896882042066   # v7.27 ベスト解 (大幅削減: 7756→1103)
    # --- 

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    alpha_weave: float = 5.655124029721978           # v7.27 ベスト解
    beta_weave: float = 0.5949249279633206           # v7.27 ベスト解
    lambda_soft_weave: float = 1357.8990752536592     # v7.27 ベスト解
    # ---

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    spatial_penalty_strength: float = 6.266615710608102  # v7.27 ベスト解
    # ---

    density_threshold_high: float = 0.0   # 高密度閾値(固定)
    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    density_threshold_medium: float = 0.001274502249451445  # v7.27 ベスト解
    # ---
    
    target_lane_capacity_high: int = 2
    target_lane_capacity_med: int = 4
    target_lane_capacity_low: int = 6
    
    kappa_lower_init: int = 1
    kappa_upper_init: int = 10  # Weave区間用(固定範囲)
    
    # -------------------------------------------------------------------------------
    # v29.0: Phase 0-3 Comprehensive Safety & Control Improvements
    # -------------------------------------------------------------------------------
    # Phase 0: Progress Jump Root Fix
    #   - LC completion: lc_progress reset to None
    #   - LC start: Explicit lc_progress = 0.0 initialization
    #   - Progress clamp: ±15% per control cycle
    #   - Pause/resume: Time adjustment with 50% limit + progress continuity
    # 
    # Phase 1: Parameter Optimization (from overnight sweep results)
    #   - lc_min_front_gap: 15.0m (was 10.0m) - RSS × 2.0 safety margin
    #   - lc_min_rear_gap: 18.0m (was 12.0m) - Enhanced rear safety
    #   - urgency_gap_relax_coeff: 0.3 (proven optimal, 62.4% success rate)
    #
    # Phase 2: Advanced Control Logic
    #   - QP Fallback: Dynamic deceleration with velocity scaling & CAV coordination
    #   - Mid-LC Gap Monitor: Pause at progress 0.3-0.7 if front_gap < 15m
    #   - TTC Prediction: Consider target lane front vehicle during LC
    #   - LC Prep Conflict: Detect simultaneous LC schedules, delay by 2s
    #
    # Phase 3: Collision Prevention & Recovery
    #   - Collision Recovery: Proper statistics + gradual 10-step position recovery
    #   - Acceleration-limited velocity adjustment (respects physical limits)
    #   - Gap-based fallback triggers (5m proximity override)
    # -------------------------------------------------------------------------------
    qp_w_a: float = 25.0       # Acceleration weight
    qp_w_j: float = 6000.0     # Jerk weight (v28.2: 4000→6000, reduce accel violations)

    # v28.0: Increased LC gap requirements for safer lane changes
    lc_min_front_gap: float = 15.0   # Lane Change Front Gap Floor (was 10.0, +50%)
    lc_min_rear_gap: float = 18.0    # Lane Change Rear Gap Floor (was 12.0, +50%)

    # v28.1: Further relaxed AEB thresholds (trust QP, AEB as last resort)
    aeb_rss_factor: float = 0.98      # AEB RSS Distance Factor (v28.1: 0.95→0.98, closer to Apollo 1.0)
    critical_rss_factor: float = 1.3  # CRITICAL RSS Distance Factor (was 1.5)
    
    proactive_brake_threshold: float = -1.0  # Deceleration threshold to trigger proactive braking
    
    # v27.14: Urgency & LC Parameters for Optimization
    urgency_gamma: float = 3.0      # Urgency curve shape
    urgency_alpha: float = 0.2      # Minimum urgency floor
    # -------------------------------------------------------------------------------
    # --- modified (v7.11_3 / 2025-11-30): Prep区間のkappa上限を固定化 ---
    kappa_upper_prep: int = 15  # Prep区間用(固定範囲、Weave区間と同じ設計思想)
    # --- modified (v8.1_2 / 2025-12-07): Solver thread count (deprecated for v11) ---
    # Previously used for Gurobi thread limiting; kept as generic solver_threads for compatibility
    solver_threads: int = 0  # 0=システムのCPU数を自動決定, kept for backward compatibility
    # ---
    
    # --- modified (v9.4 / 2025-12-13): デバッグフラグ追加 ---
    enable_level2a_debug: bool = False   # Level2A失敗時のIIS出力（v9.4まで使用）
    enable_gap_debug: bool = True        # ギャップ判定の詳細ログ (v18.20: DEBUG ON)
    enable_spawn_debug: bool = False     # Spawn調整ログ
    # ---

    # --- modified (v9.5 / 2025-12-13): 統合Level2デバッグフラグ ---
    enable_level2_unified_debug: bool = False   # 統合Level2失敗時のIIS出力
    # ---
    
    # --- modified (v9.6 / 2025-12-14): デバッグモード追加 ---
    debug_mode: bool = False             # デバッグモード（高速実行）
    debug_tmax: float = 60.0             # デバッグ時の短縮時間（600s → 60s）
    debug_spawn_rate_factor: float = 0.5 # spawn_rate削減係数（50%）
    debug_early_stop_collisions: int = 10  # この衝突数で早期終了
    debug_early_stop_time: float = 120.0   # この時刻で早期終了（安全策）
    # ---
    
    # 理論的根拠:
    #   - 動的範囲(kappa_dynamic_range=35)では予測不確実性により
    #     安全ギャップ制約を通過できず、フィージブルペア激減
    #   - 固定範囲により全車両が均等な選択肢を持つ
    #   - 15秒 × 14m/s = 210m → 約21セル到達可能(十分な空間分散)
    # ---
    kappa_dynamic_range: int = 35  # 廃止予定(v7.11_3でkappa_upper_prepに置き換え)
    dt: float = 1.0
    
    # --- modified (v9.4 / 2025-12-13): 固定G_minを削除、動的計算に変更 ---
    # G_min: float = 13.564  # 削除：速度依存の動的計算に置き換え

    # --- modified (v9.5 / 2025-12-13): Level1の安全バッファを緩和、統合Level2で安全担保 ---
    # Level1の安全バッファ（統合Level2が安全を保証するため緩く設定）
    safety_buffer_level1: float = 0.0  # ベイズ最適化範囲: -2.0 ~ 2.0
    # 負の値も許可 → Level1を緩くし、統合Level2で安全担保
    # ---
    
    # --- modified (v10.3 / 2025-12-14): ベイズ最適化パラメータ ---
    # v10.0-v10.2: LC scheduled 0 → ギャップ判定パラメータを最適化
    constraint_tightening_margin: float = 0.05  # Range: 0.0-0.10 (Bayesian opt)
    # ---
    tightening_increment: float = 0.10  # +10% on infeasibility
    max_tightening: float = 0.40  # Abort if > 40%
    
    # CAV time headway (Shladover et al., 2012; CACC experiments)
    time_headway_CAV: float = 0.7  # seconds (with V2V communication)
    time_headway_HDV: float = 1.5  # seconds (human driver baseline)
    use_CAV_parameters: bool = True  # Enable CAV-specific parameters
    
    # TTC thresholds (ISO 21202)
    ttc_threshold_base: float = 2.5  # seconds (base threshold)
    ttc_threshold_high_speed: float = 3.5  # seconds (Δv > 15 m/s)
    
    # Cooperative deceleration (VISSIM standard)
    cooperative_decel_limit: float = -3.0  # m/s² (VISSIM default)
    emergency_decel_limit: float = -6.0  # m/s² (UN R157)
    
    # Lane change duration (empirical)
    lane_change_duration: float = 4.0  # seconds (typical LC duration)

    # --- modified (v27.4 / 2026-01-06): Dynamic Aggressiveness Tuning Parameters ---
    # Used by ApolloSafetyManager to scale thresholds in weaving zone
    lc_aggressive_base_front: float = 10.0  # Base front gap [m]
    lc_aggressive_base_rear: float = 12.0   # Base rear gap [m]
    lc_aggressive_base_front_ttc: float = 2.0  # Base front TTC [s]
    lc_aggressive_base_rear_ttc: float = 3.0   # Base rear TTC [s]
    # ---
    
    # --- modified (v27.29 / 2026-01-19): Exit-Near Urgency Parameters (Bayesian Optimization) ---
    # Dynamic threshold computation in Apollo Safety Manager based on distance to exit
    exit_urgent_dist: float = 100.0         # [m] Distance threshold for 'urgent' mode
    exit_emergency_dist: float = 50.0       # [m] Distance threshold for 'emergency' mode
    exit_long_relax_urgent: float = 0.7     # Longitudinal relaxation factor for urgent mode
    exit_lat_relax_urgent: float = 0.75     # Lateral relaxation factor for urgent mode
    exit_long_relax_emerg: float = 0.5      # Longitudinal relaxation factor for emergency mode
    exit_lat_relax_emerg: float = 0.6       # Lateral relaxation factor for emergency mode
    exit_scale_floor: float = 0.3           # Minimum safety scaling factor near exit
    # ---
    
    # VISSIM Wiedemann 99 parameters
    CC0: float = 1.5  # standstill distance (m)
    CC1: float = 0.9  # time headway (s)
    safety_distance_reduction_LC: float = 0.6  # 40% reduction during LC
    
    # Reference governor
    enable_reference_governor: bool = True  # Enable feasibility pre-check
    
    # Level1 horizon (Xi et al., arXiv:1904.08784)
    level1_horizon_time: float = 5.0  # seconds (validated value)
    level1_horizon_steps: int = 50  # discretization points
    
    # Level2 horizon (tracking MPC typical)
    level2_horizon_time: float = 1.0  # seconds
    level2_horizon_steps: int = 10  # shorter for tracking
    
    # Apollo piecewise-jerk QP weights (validated)
    w_ref_deviation: float = 0.5  # position/velocity tracking
    w_acceleration: float = 1.0  # acceleration cost
    w_jerk: float = 100.0  # jerk cost (smoothness)
    
    # Level2 solver
    use_osqp: bool = True  # Use OSQP instead of Gurobi (Apollo/Autoware standard)
    # ---
    
    gamma: float = 0.3
    # --- modified (ver.7.7 / 2025-11-29): soft-consistency penalty強化 ---
    lambda_soft: float = 200.0  # 旧値: 334.902
    # ---
    # --- modified (v19.3 / 2025-12-29): Ultra-aggressive horizon reduction to Apollo minimum ---
    # Rationale: Even horizon=40 caused hang at 37.4s with ~18 vehicles
    # - v19.0 (80 steps): stopped at 51s
    # - v19.1 (60 steps): stopped at 25s
    # - v19.2 (40 steps): stopped at 37.4s
    # - v19.3 (30 steps): 3s prediction horizon = Apollo absolute minimum
    # - QP complexity ∝ O(horizon²); 30 steps = 56% of v19.2 computation
    # - Must achieve 60s completion even at minimal horizon
    # - Trade-off: Reduced foresight BUT stable execution is prerequisite
    # - v19.3 (30 steps): 3s prediction horizon = Apollo absolute minimum
    # - v26.0 (50 steps): Increased to 5s... resulted in -5.04 m/s^2 (better but not zero)
    # - v26.0 (80 steps): Restore Apollo standard (8.0s) for max smoothness
    horizon: int = 80  # Apollo standard: 80 steps (8s) for optimal safety planning
    # ---------------------------------------------------------------------------------

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    lookahead_horizon: float = 28.500946702466198  # v7.27 ベスト解(v7.24_4: 41.06 → 15.46 大幅短縮)
    # ---

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    eta_lambda_prep: float = 0.4108300762413586      # v7.27 ベスト解(準備区間の勾配倍率: 1.76 → 4.38 大幅増加)
    eta_lambda_weave: float = 3.290402242638633     # v7.27 ベスト解(織り込み区間の勾配倍率: 3.08 → 3.85 増加)
    # ---

    # --- modified (v8.2 / 2025-12-08): v8.2 ベイズ最適化結果を反映 (Target=360.71) ---
    # λ_i = λ_base * ((1 + η * progress) ^ p)
    # p > 1: 奥で急激に増加(凸関数)、p = 1: 線形(v7.4と同じ)、p < 1: 緩やかな増加(凹関数)
    power_lambda_prep: float = 3.9649709889437488    # v7.27 ベスト解(2.10 → 2.80 凸性強化)
    power_lambda_weave: float = 1.0395525687421079  # v7.27 ベスト解(1.26 維持)
    # ---

    # --- modified (v7.24_3 / 2025-12-05): v7.24_4 ベイズ最適化結果を反映 ---
    # 空間コストの位置依存重み(準備区間のみ適用)
    # w_c = 1.0 + gamma_spatial * ((c_max - c) / c_max) ** q_spatial
    gamma_spatial: float = 0.7512922820748574  # v7.24_4 ベスト解(手前ペナルティの強度)
    q_spatial: float = 2.5309541989570024      # v7.24_4 ベスト解(勾配の形状)
    # ---

    # --- modified (v8.4 / 2025-12-09): Level 2パラメータの追加 ---
    # Level 2-A 目的関数のパラメータ化（v9.4まで使用、v9.5では統合Level2に置き換え）
    # 目的: v_refを最適化可能にし、旅行時間短縮を実現
    v_ref_level2: float = 20.0        # 参照速度（m/s）- ベイズ最適化対象
    alpha_level2: float = 1.0         # 速度維持の重み - ベイズ最適化対象
    beta_level2: float = 1.0          # 速度変化抑制の重み - ベイズ最適化対象
    # ---

    # --- modified (v9.5 / 2025-12-13): 統合Level2パラメータ ---
    # 統合Level2（安全制約 + ギャップ生成を単一のQPで最適化）
    alpha_level2_unified: float = 2.0    # 速度平滑化の重み（Level2A目的）
    beta_level2_unified: float = 1.0     # 速度変化の重み（Level2A目的）
    gamma_jerk_unified: float = 0.5      # jerk最小化の重み（Level2B目的）
    G_min_level2b: float = 15.0          # ギャップ生成の目標値 [m]
    # ---

    # --- modified (ver.7.9 / 2025-11-29): consistency_weightを無効化 ---
    consistency_weight: float = 0.1  # 500.0 → 0.0
    # 理論的根拠:
    #   - バイナリ変数y_{i,c,κ}への適用は数理的に不適切
    #   - MPCの時間進行により(c*,κ*)が常にシフトするため、ペナルティが逆効果
    #   - 連続的なLC位置調整を妨げ、手前への集中を誘発
    #   - 代わりに空間履歴H_cとw_baseによる分散を強化
    # ---

    gap_threshold_high: float = 35.0
    gap_threshold_low: float = 25.0
    
    delta_rho_min: float = 0.015  # v6.19
    
    # v30.0: LC Cooldown (基本値は実験によって決定)
    # 根拠: 従前のハードコード値 (0.5/1.0/3.0/5.0秒) を統一
    # 
    # 注意: 5.0秒は「Pause×3後に観測された値」だが、根拠は不完全
    # ユーザー課題: 「5秒固定cooldown が失敗原因」
    # → より短い値、または動的調整が必要の可能性あり
    # 
    # Current value = 5.0秒 (暫定値)
    # TODO: LC成功率vs.cooldown時間の関係を実験で検証し、最適値を決定
    lc_cooldown_time: float = 5.0  # v30.0: Experimental value, needs validation
    
    # v30.0: Emergency LC Distance (出口手前の強制urgency boost)
    # 役割: urgency計算内で、この距離以内なら urgency=1.0 に強制
    # 従前: if-then文での不連続な確率ジャンプ → 現在: urgency計算内の連続関数
    # 効果: 出口50m以内でのLC成功率向上 + スムーズなdistribution
    lc_min_distance_from_exit: float = 50.0  # v30.0: Emergency LC zone

    # --- modified (v19.0 / 2025-12-29): High-precision physics for collision-free control ---
    # Rationale: Prioritize control performance over computation speed
    # - dt_sim: 0.01s (100Hz) for precise collision detection and trajectory following
    # - dt_control: 0.1s (10Hz) Apollo standard for real-time planning
    # - Sub-step integration: 10 physics steps per control update
    # - Enables accurate ST-Boundary constraint enforcement
    dt_sim: float = 0.01  # Physics simulation timestep (100Hz for precision)
    dt_control: float = 0.1  # Control update cycle (10Hz, Apollo standard)
    dt: float = 0.1  # Backward compatibility (same as dt_control)
    # ---------------------------------------------------------------------------------

    # --- modified (Final Fix v11.3 / 2025-12-17): Apollo 10Hz high-frequency control ---
    # Rationale: 1.0s control interval is too slow for collision avoidance
    # - At relative velocity 10 m/s, vehicles can close 10m in 1 second
    # - Apollo standard: 10Hz control (0.1s interval) for real-time safety
    # - Enables immediate response to sudden lane changes and emergency scenarios
    control_interval: float = 0.1  # Apollo standard (10Hz control loop) - DEPRECATED, use dt_control
    # ---------------------------------------------------------------------------------

    lc_duration: float = 3.0        # Lane change duration [s] (v2 optimal for HIGH load)

    # 速度の正規分布
    v_mean: float = 14.0  # μ_v (m/s)
    v_std: float = 1.0    # σ_v (m/s)
    
    # 車間距離の正規分布
    spacing_reaction_time: float = 1.0  # T (s)
    spacing_min: float = 5.0            # s_min (m)
    spacing_std: float = 5.0            # σ_s (m)
    
    # 車頭時間(headway)のパラメータ
    headway_mean: float = 1.5          # 平均車頭時間 [s]
    headway_std: float = 0.5           # 車頭時間の標準偏差 [s]
    # --- modified (v9.2 / 2025-12-11): 安全性向上のため最小車頭時間を延長 ---
    headway_min: float = 1.0           # 最小車頭時間 [s]（旧: 0.5s）
    # ---
    headway_mean_congestion: float = 2.5   # 渋滞時平均車頭時間 [s]
    headway_std_congestion: float = 0.8    # 渋滞時標準偏差 [s]
    
    # --- modified (v6.28 / 2025-11-18): 渋滞環境用パラメータ追加 ---
    # --- modified (v9.2 / 2025-12-11): 渋滞時速度を首都高速想定に修正 (12.0 → 8.0 m/s) ---
    congestion_mode: bool = False  # 渋滞モードフラグ
    v0_mean_congestion: float = 8.0  # 渋滞時平均速度 [m/s] (29 km/h、首都高速渋滞時)
    v0_std_congestion: float = 2.0    # 渋滞時速度標準偏差 [m/s](通常3.0→2.0)
    headway_time_congestion: float = 2.5  # 渋滞時車頭時間 [s](通常1.5→2.5)
    # ---

    # --- modified (v9.2 / 2025-12-11): 通常時速度パラメータを追加（負荷レベル別に設定） ---
    v0_mean_normal: float = 20.0  # 通常時平均速度 [m/s]（set_load_levelで設定される）
    v0_std_normal: float = 3.0    # 通常時速度標準偏差 [m/s]
    # ---

    # --- modified (v10.3 / 2025-12-14): ベイズ最適化パラメータ ---
    L_vehicle: float = 5.0    # 車両長 [m]（固定）
    c0_vissim: float = 0.9    # VISSIM速度係数 (Range: 0.5-1.2, Bayesian opt)
    d0_vissim: float = 1.5    # 静止時最小距離 [m] (Range: 0.0-3.0, Bayesian opt)
    # Required gap = (L + c0*v + d0) * (1 + tightening)
    # 衝突回避とLC成功のバランスを最適化
    # ---
    xi_miqp: float = 0.6      # Level 1（MIQP）緩和係数
    xi_qp_gap: float = 0.6    # Level 2-B（ギャップ生成）緩和係数

    # LEFT/RIGHT車線群で分離
    spawn_rate_left: float = 0.15   # LEFT/LCENTER車線のspawn率 [veh/s]
    spawn_rate_right: float = 0.15  # RCENTER/RIGHT車線のspawn率 [veh/s]
    # spawn_rate: float = 0.30  # 非推奨（後方互換性のため残す）← 既存のspawn_rateを使用

    # headway_min: float = 1.0  # 最小車頭時間 [s]（旧: 0.5s）← 既存のheadway_minを上書き
    # ---

    # --- modified (v11.10 / 2025-12-19): HDV混在モードパラメータ ---
    # Human-Driven Vehicle (HDV) ratio for mixed traffic simulation
    # 0.0: All CAVs (default), 1.0: All HDVs
    # HDVs use IDM only and cannot share trajectory information
    hdv_ratio: float = 0.0
    # ---

    # --- NEW (v29.1 / 2026-01-07): Spawn scheduler mode ---
    # "attempt": 既存方式（試行ごとにPoisson間隔で挿入し、安全性でスキップされ得る）
    # "queue":   サイド毎Poisson到着（レーン配分）を需要として発生→安全に挿入できるまで待機
    #              （需要はPoissonのまま、実受入は安全成立時。自由流では目標流量に収束）
    spawn_mode: str = "attempt"

    # --- modified (v18.29 / 2026-01-01): LC確率モデル + Gap Acceptance緩和 ---
    # Logistic Probability Model: P(LC|x) = sigmoid(β₀ + β₁·x_norm)
    # 
    # v18.29: Position-Based LC Spatial Distribution
    # ========================================================
    # 目標: LC成功率50%以上 + LC位置の時空間的分散化
    # 
    # 設計原理:
    # - 位置ベースの確率: 入口で低確率、出口で高確率
    # - 0.5秒ごとの連続判定で自然な分布が創発
    # - Gap Acceptance緩和(Urgency高時)でLC成功率向上
    # 
    # 確率計算例 (β₀=-0.5, β₁=4.0):
    # | x_norm | logit | P(LC)  | 解釈           |
    # |--------|-------|--------|----------------|
    # | 0.0    | -0.5  | 37.8%  | 入口: 低確率   |
    # | 0.25   | 0.5   | 62.2%  | 1/4地点: 中    |
    # | 0.5    | 1.5   | 81.8%  | 中央: 高       |
    # | 0.75   | 2.5   | 92.4%  | 3/4地点: 非常高|
    # | 1.0    | 3.5   | 97.1%  | 出口: 確実     |
    # ========================================================
    lc_beta_0: float = -0.5  # v18.29: Baseline (low at entry)
    lc_beta_1: float = 11.0  # v2 OPTIMAL: 11.0 (was 4.0, overnight v2 best HIGH/queue LC=0.896)
    lc_beta_2: float = 0.0   # Exponential term: Disabled
    lc_beta_k: float = 5.0   # Exponential decay rate (if lc_beta_2 > 0)
    # 
    # v18.29: Gap Acceptance Urgency Relaxation
    # - 高Urgency時にGap閾値を緩和（LC成功率向上）
    # - gap_relax_factor(U) = 1.0 - urgency_gap_relax_coeff * U
    # - 例: U=0.8, coeff=0.5 → gap閾値は0.60倍に緩和
    urgency_gap_relax_coeff: float = 0.5  # v2 OPTIMAL: 0.5 (was 0.3, overnight v2 best HIGH/queue LC=0.896)
    # ---

    # --- modified (v11.12 / 2025-12-19): LC実行パラメータ ---
    # Lane change preparation duration (booking to execution)
    # Allows cooperative vehicles to create space and adjust speeds
    # v18.20 FIX: Reduced from 5.0s to 2.0s for faster LC execution
    lc_prep_duration: float = 2.0  # seconds (shorter prep for faster LC response)
    # ---
    
    def set_load_level(self, level: str):
        level = level.lower()

        if level == "low":
            # --- modified (v9.2 / 2025-12-11): LEFT/RIGHT車線群分離とSpawn設定修正 ---
            self.spawn_rate_left = 0.20   # LEFT全体: 0.4 veh/s
            self.spawn_rate_right = 0.20  # RIGHT全体: 0.4 veh/s
            self.spawn_rate = 0.40
            self.headway_min = 1.0
            self.congestion_mode = False
            # 首都高速想定の速度設定
            self.v0_mean_normal = 16.0  # 58 km/h（首都高速通常時）
            # ---

            # --- modified (v9.2.1 / 2025-12-13): ベイズ最適化結果（Trial 95, Value=203.33）を反映 ---
            self.alpha_prep = 8.357272615249649
            self.beta_prep = 18.682797100835174
            self.lambda_soft_prep = 738.8530342586615
            self.alpha_weave = 7.124800596373161
            self.beta_weave = 0.13291045321936457
            self.lambda_soft_weave = 3282.0016095611436
            self.eta_lambda_prep = 0.4036095515920459
            self.eta_lambda_weave = 2.3626108168505757
            self.power_lambda_prep = 1.4839678135454883
            self.power_lambda_weave = 2.3153437592720767
            self.spatial_penalty_strength = 5.08279291718398
            self.lookahead_horizon = 38.39786021774582
            self.v_ref_level2 = 21.065313232342227
            self.alpha_level2 = 3.1708829205489524
            self.beta_level2 = 1.0832951122482135
            # ---

        elif level == "medium":
            # --- modified (v9.2 / 2025-12-11): LEFT/RIGHT車線群分離とSpawn設定修正 ---
            self.spawn_rate_left = 0.30   # LEFT全体: 0.6 veh/s
            self.spawn_rate_right = 0.30  # RIGHT全体: 0.6 veh/s
            self.spawn_rate = 0.60
            self.headway_min = 1.0
            self.congestion_mode = False
            # 首都高速想定の速度設定
            self.v0_mean_normal = 14.0  # 50 km/h（首都高速やや混雑）
            # ---

            # --- modified (v9.2.1 / 2025-12-13): ベイズ最適化結果（Trial 35, Value=167.84）を反映 ---
            self.alpha_prep = 18.1110523333329
            self.beta_prep = 3.0982906166171547
            self.lambda_soft_prep = 791.2222046425934
            self.alpha_weave = 7.951205690677975
            self.beta_weave = 0.08825517332967195
            self.lambda_soft_weave = 349.9826956032912
            self.eta_lambda_prep = 0.6508617827062106
            self.eta_lambda_weave = 1.731908657883833
            self.power_lambda_prep = 2.1877930902873444
            self.power_lambda_weave = 1.9630169684758778
            self.spatial_penalty_strength = 9.715544269619333
            self.lookahead_horizon = 26.89481465259616
            self.v_ref_level2 = 18.251150956815664
            self.alpha_level2 = 1.3654574119154055
            self.beta_level2 = 5.581203753217281
            # ---

        elif level == "high":
            # --- modified (v9.2 / 2025-12-11): LEFT/RIGHT車線群分離とSpawn設定修正 ---
            self.spawn_rate_left = 0.35   # LEFT全体: 0.7 veh/s
            self.spawn_rate_right = 0.35  # RIGHT全体: 0.7 veh/s
            self.spawn_rate = 0.70
            self.headway_min = 1.0
            self.congestion_mode = False
            # 首都高速想定の速度設定
            self.v0_mean_normal = 12.0  # 43 km/h（首都高速混雑）
            # ---

            # --- modified (v9.2.1 / 2025-12-13): ベイズ最適化結果（Trial 1, Value=146.90）を反映 ---
            self.alpha_prep = 78.03532317523569
            self.beta_prep = 19.01921469755733
            self.lambda_soft_prep = 2910.635913133072
            self.alpha_weave = 6.588597115674811
            self.beta_weave = 0.19821770842031466
            self.lambda_soft_weave = 205.11104188433976
            self.eta_lambda_prep = 0.6389197338501941
            self.eta_lambda_weave = 3.4647045830997407
            self.power_lambda_prep = 3.20501755284444
            self.power_lambda_weave = 2.2701814444901136
            self.spatial_penalty_strength = 0.3037864935284442
            self.lookahead_horizon = 48.645943347289744
            self.v_ref_level2 = 23.324426408004218
            self.alpha_level2 = 2.202157195714934
            self.beta_level2 = 1.9000671753502962
            # ---
        elif level == "congestion":
            # --- modified (v6.28 / 2025-11-18): 渋滞環境設定 ---
            # --- modified (v9.2 / 2025-12-11): LEFT/RIGHT車線群分離とSpawn設定修正 ---
            self.spawn_rate_left = 0.25   # LEFT全体: 0.5 veh/s
            self.spawn_rate_right = 0.25  # RIGHT全体: 0.5 veh/s
            self.spawn_rate = 0.50
            self.headway_min = 1.5
            self.congestion_mode = True
            self.headway_mean = self.headway_mean_congestion  # 渋滞時車頭時間を適用
            # 首都高速想定の速度設定（v0_mean_congestionを上書き）
            # v0_mean = 8.0  # 29 km/h（首都高速渋滞）← v0_mean_congestionを使用
            # ---
            print("[IntegratedZoneParameters] Load level set to: CONGESTION")
            print(f"  Spawn rate: {self.spawn_rate:.2f} veh/s")
            print(f"  Initial velocity: mean={self.v0_mean_congestion:.1f} m/s, std={self.v0_std_congestion:.1f} m/s")
            print(f"  Headway time: {self.headway_time_congestion:.2f} s")
            # ---
        else:
            raise ValueError(f"Unknown load level: {level}")
        
        if not self.congestion_mode:
            print(f"[IntegratedZoneParameters] Load level set to: {level.upper()}")
            print(f"  Total spawn_rate: {self.spawn_rate:.2f} veh/s (All 4 lanes)\n")
    
    # --- modified (v9.4 / 2025-12-13): 動的ギャップ計算メソッド追加 ---
    def compute_required_gap_level1(self, v_rear: float) -> float:
        # VISSIM基準の安全車間（Level2Aと同じ）
        s_req_vissim = self.L_vehicle + self.c0_vissim * v_rear + self.d0_vissim
        
        # Level1はさらに余裕を持たせる
        required_gap = s_req_vissim + self.safety_buffer_level1
        
        return required_gap
    # ---
# ---
