#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAV織り込み区間制御システム - メインエントリポイント
==================================================

バージョン: v11.0
日付: 2025-12-16

実行方法:
    # テストモード（Frenet QP + Urgencyプランナーのユニットテスト）
    python -m weaving_v11.main --mode test
    
    # デモモード（コントローラの初期化確認）
    python -m weaving_v11.main --mode demo --tmax 60
    
    # シミュレーションモード（フルシミュレーション）
    python -m weaving_v11.main --mode sim --load medium --tmax 600
    python -m weaving_v11.main --mode sim --load high --debug
    
    # パラメータオーバーライド付き（ベイズ最適化用）
    python -m weaving_v11.main --mode sim --load high --config best_params.json
    
    # イベントベース終了（AEBまたは衝突まで実行）
    python -m weaving_v11.main --mode sim --load high --until aeb
"""

import sys
import argparse
from typing import Optional, Callable, Any

# Import from current package
from .frenet_qp_apollo import FrenetQPController, VehicleState, ObstacleInfo
from .mpc_controller import (
    UrgencyPlanner,
    FrenetQPControllerWithRelaxation,
    compute_urgency_distribution_metrics
)


def test_frenet_qp():
    """Frenet QPコントローラのテスト"""
    print("=" * 80)
    print("Frenet QP Apolloコントローラのテスト")
    print("=" * 80)

    controller = FrenetQPController(horizon=30, dt=0.1)

    # Test 1: Normal car following
    print("\n### Test 1: Normal Car Following ###")
    ego = VehicleState(id=1, s=0.0, v=15.0, a=0.0, lane='center')
    front = VehicleState(id=2, s=50.0, v=12.0, a=0.0, lane='center')

    result = controller.optimize(
        ego_state=ego,
        obstacles=[ObstacleInfo(vehicle_state=front, is_front=True)]
    )

    if result:
        print("[PASS] Optimization successful!")
        print(f"  Initial velocity: {ego.v:.2f} m/s")
        print(f"  Final velocity: {result['v'][-1]:.2f} m/s")
        gap_maintained = min(front.s + front.v * k * 0.1 - result['s'][k] for k in range(30))
        print(f"  Min gap maintained: {gap_maintained:.2f} m")
    else:
        print("[FAIL] Optimization failed")
        return False

    # Test 2: Free road
    print("\n### Test 2: Free Road (No Front Vehicle) ###")
    result2 = controller.optimize(ego_state=ego, obstacles=[])

    if result2:
        print("[PASS] Optimization successful!")
        print(f"  Final velocity: {result2['v'][-1]:.2f} m/s (should approach v_max)")
        print(f"  Distance traveled: {result2['s'][-1] - ego.s:.2f} m")
    else:
        print("[FAIL] Optimization failed")
        return False

    return True


def test_urgency_planner():
    """緊急度プランナーのテスト"""
    print("\n" + "=" * 80)
    print("緊急度ベースMPCコントローラのテスト")
    print("=" * 80)

    planner = UrgencyPlanner(gamma=3.0, alpha=0.2, replan_interval=0.5)

    s_entry = 0.0
    s_exit = 500.0

    # Test urgency computation
    print("\n### Test 1: Urgency Computation ###")
    test_positions = [0, 100, 250, 400, 490]
    for s in test_positions:
        urgency = planner.compute_urgency(s, s_entry, s_exit, rho_target=0.01)
        print(f"  Position s={s:3d}m: Urgency U={urgency:.3f}")

    # Test dynamic relaxation
    print("\n### Test 2: Safety Margin Relaxation ###")
    controller = FrenetQPControllerWithRelaxation(
        horizon=30,
        dt=0.1,
        T_base=1.5,
        beta_relax=0.4
    )

    test_urgencies = [0.0, 0.25, 0.5, 0.75, 1.0]
    for U in test_urgencies:
        T_adjusted = controller.adjust_safety_margin(U)
        print(f"  Urgency U={U:.2f}: Time headway T={T_adjusted:.2f}s "
              f"(relaxation: {T_adjusted/controller.T_base*100:.1f}%)")

    # Test batch computation
    print("\n### Test 3: Batch Urgency Computation ###")
    vehicles = [
        {'id': 1, 'x': 50, 'target_lane': 'center_left'},
        {'id': 2, 'x': 200, 'target_lane': 'center_left'},
        {'id': 3, 'x': 450, 'target_lane': 'center_right'}
    ]

    rho_per_lane = {
        'center_left': 0.02,
        'center_right': 0.01
    }

    urgency_states = planner.compute_urgencies_batch(
        vehicles,
        s_entry,
        s_exit,
        rho_per_lane
    )

    for state in urgency_states:
        print(f"  {state}")

    # Test distribution metrics
    print("\n### Test 4: Distribution Metrics ###")
    metrics = compute_urgency_distribution_metrics(urgency_states)
    print(f"  Spatial std: {metrics['spatial_std']:.2f} m")
    print(f"  Concentration index: {metrics['concentration_index']:.3f}")
    print(f"  Mean position: {metrics['mean_position']:.1f} m")
    print(f"  Mean urgency: {metrics['mean_urgency']:.3f}")

    return True


def demo_mode(tmax: float = 60.0):
    """
    デモシミュレーションモード - コントローラ初期化の確認のみ

    引数:
        tmax: 最大シミュレーション時間 [秒]
    """
    print("\n" + "=" * 80)
    print(f"Demo Mode: t_max = {tmax}s")
    print("=" * 80)

    print("\nInitializing controllers...")

    # Initialize Urgency Planner
    urgency_planner = UrgencyPlanner(
        gamma=3.0,
        alpha=0.2,
        replan_interval=0.5
    )

    # Initialize Frenet QP Controller with Relaxation
    frenet_controller = FrenetQPControllerWithRelaxation(
        horizon=30,
        dt=0.1,
        T_base=1.5,
        beta_relax=0.4,
        T_min=0.9
    )

    print("[PASS] Controllers initialized")
    print("\nSimulation Summary:")
    print(f"  - Urgency curve steepness (gamma): {urgency_planner.gamma}")
    print(f"  - Base time headway: {frenet_controller.T_base}s")
    print(f"  - Minimum time headway: {frenet_controller.T_min}s")
    print(f"  - Relaxation coefficient: {frenet_controller.beta_relax}")

    print("\n[INFO] For full simulation, use: python -m weaving_v11.main --mode sim")
    print("[INFO] See files/IMPLEMENTATION_GUIDE_v11_0.md for details")


def simulation_mode(load: str = 'medium', tmax: float = 600.0, debug: bool = False, 
                    hdv_ratio: float = 0.0, until_event: Optional[str] = None,
                    config_path: Optional[str] = None,
                    export_video: bool = False,
                    silent: bool = False):
    """
    フルシミュレーションモード - v10.3に基づきv11.0 Level 2コントローラを実行

    引数:
        load: 負荷レベル ('low', 'medium', 'high', 'congestion')
        tmax: 最大シミュレーション時間 [秒]
        debug: デバッグモード (高速実行)
        hdv_ratio: HDV（人間運転車両）の割合 (0.0-1.0)
        until_event: イベントベース終了 ('aeb', 'collision', 'any')
        config_path: パラメータオーバーライド用JSONファイルパス
        export_video: シミュレーション後に動画をエクスポート
        silent: サイレントモード (ログファイル無効、JSON_STATSのみ出力)
    """
    import sys
    import os
    import io
    from datetime import datetime
    
    # Save original stdout for silent mode
    original_stdout = sys.stdout
    
    # In silent mode, suppress all output from the start
    if silent:
        sys.stdout = io.StringIO()

    # Import v11.0 simulator components (self-contained, no external dependencies)
    try:
        from .simulator import IntegratedZoneSimulator
        from .parameters import IntegratedZoneParameters
        from .utils import Logger
    except ImportError as e:
        if silent:
            sys.stdout = original_stdout
        print(f"[ERROR] Failed to import simulator modules: {e}")
        print("[INFO] Make sure simulator.py, parameters.py, and utils.py exist in weaving_v11/")
        return False

    # Try to import visualization module (optional) - dynamic import only
    # Skip in silent mode to avoid unnecessary imports
    plot_func: Optional[Callable[[Any, str], None]] = None
    export_func: Optional[Callable[[Any, str], None]] = None
    if not silent:
        try:
            import importlib.util

            # Use find_spec to check module availability without importing
            spec = importlib.util.find_spec('.visualization', package='weaving_v11')
            if spec is not None:
                from . import visualization as viz_module
                plot_func = getattr(viz_module, 'plot_lc_spatiotemporal_distribution', None)
                export_func = getattr(viz_module, 'export_video', None)
                if plot_func is not None:
                    print("[INFO] Visualization module loaded successfully")
                else:
                    print("[WARNING] plot_lc_spatiotemporal_distribution function not found in visualization module")
            else:
                print("[WARNING] Visualization module not available. Plots will be skipped.")
        except Exception as e:
            print(f"[WARNING] Failed to load visualization module: {e}")

    print("\n" + "=" * 80)
    print("CAV Weaving Zone Control System v11.0")
    print("Fully modularized simulation with Apollo Frenet QP + Urgency MPC")
    print("=" * 80)

    # Setup logging (skip in silent mode - stdout already redirected above)
    logger = None
    log_file = None
    
    if not silent:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        output_dir = os.path.join(script_dir, "outputs")
        os.makedirs(output_dir, exist_ok=True)

        log_filename = f"simulation_log_v11_{load.upper()}_{timestamp}.txt"
        log_file = os.path.join(output_dir, log_filename)

        logger = Logger(log_file, script_name="weaving_v11")
        sys.stdout = logger

    # Setup parameters
    params = IntegratedZoneParameters()
    params.set_load_level(load)
    params.hdv_ratio = hdv_ratio
    # Flag to control video export in post-processing
    setattr(params, 'export_video', bool(export_video))

    # v27.12 NEW: Apply optimization overrides from JSON
    if config_path and os.path.exists(config_path):
        import json
        try:
            with open(config_path, 'r') as f:
                overrides = json.load(f)
            
            print(f"\n[CONFIG] Applying overrides from {config_path}:")
            for k, v in overrides.items():
                if hasattr(params, k):
                    old_val = getattr(params, k)
                    setattr(params, k, v)
                    print(f"  - {k}: {old_val} -> {v}")
                else:
                    print(f"  - [WARNING] Unknown parameter: {k}")
            
            # Re-initialize safety manager if params changed
            # (Note: safety manager is created inside simulator, so passing params to simulator is enough)
        except Exception as e:
            print(f"[ERROR] Failed to load config: {e}")

    if debug:
        params.debug_mode = True
        if tmax == 600.0:  # Default value
            tmax = params.debug_tmax

    # Create simulator
    print(f"\nLoad Level: {load.upper()}")
    print(f"Simulation Time: {tmax}s")
    if debug:
        print("DEBUG MODE ENABLED")
    if hdv_ratio > 0.0:
        print(f"HDV Ratio: {hdv_ratio:.1%} (Mixed Traffic Mode)")
    print("Control Mode: L2 (Hierarchical MIQP + QP with Apollo Frenet compatibility)")
    print("=" * 80 + "\n")
    sys.stdout.flush()

    # Run simulation
    try:
        simulator = IntegratedZoneSimulator(params, load, mode='l2')
        simulator.run(t_max=tmax, until_event=until_event)
        
        # v27.15: Output JSON metrics for optimization
        try:
            stats = simulator.get_statistics()
            # Add extra metrics useful for BO
            # Using getattr/defaults to be safe
            stats['collision_count'] = simulator.collision_count if hasattr(simulator, 'collision_count') else 0
            stats['aeb_count'] = simulator.aeb_trigger_count if hasattr(simulator, 'aeb_trigger_count') else 0
            
            import json
            # In silent mode, restore original stdout for JSON output
            if silent:
                sys.stdout = original_stdout
            print("\n[JSON_STATS] " + json.dumps(stats))
            sys.stdout.flush()
        except Exception as e:
            print(f"[WARNING] Failed to print JSON stats: {e}")
    except Exception as e:
        import traceback
        print(f"[ERROR] Simulation failed: {e}")
        print("[TRACEBACK]")
        traceback.print_exc()
        sys.stdout.flush()
        if logger is not None:
            sys.stdout.close()
            sys.stdout = sys.__stdout__
        return False

    # Generate plots (if visualization available and not silent mode)
    if not silent and plot_func is not None:
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            output_dir = os.path.join(script_dir, "outputs")
            plot_file = os.path.join(output_dir, f'plots_v11_{load.upper()}_{timestamp}.png')
            plot_func(simulator, plot_file)
            print(f"Plot saved to: {plot_file}")
        except Exception as e:
            print(f"[WARNING] Plot generation failed: {e}")
    elif not silent:
        print("[INFO] Skipping plot generation (visualization module not available)")

    # Export animation video if available and requested (not in silent mode)
    if not silent:
        try:
            if export_func is not None and getattr(simulator.params, 'export_video', False):
                video_file = os.path.join(output_dir, f'video_v11_{load.upper()}_{timestamp}.mp4')
                export_func(simulator, video_file)
                print(f"Video saved to: {video_file}")
            elif export_func is None:
                print("[INFO] Skipping video export (export_video not available)")
        except Exception as e:
            print(f"[WARNING] Video export failed: {e}")

    if not silent and log_file:
        print(f"\nSimulation completed. Log saved to: {log_file}")

    # Restore stdout
    if not silent:
        sys.stdout.flush()
        if logger is not None:
            sys.stdout.close()
            sys.stdout = sys.__stdout__
    # In silent mode, stdout is already restored after JSON output

    return True


def main(argv: Optional[list] = None):
    """メインエントリポイント"""
    parser = argparse.ArgumentParser(
        description="CAV織り込み区間制御システム v11.0",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
実行例:
  # ユニットテスト (デフォルト)
  python -m weaving_v11.main --mode test

  # デモモード (コントローラ初期化)
  python -m weaving_v11.main --mode demo --tmax 60

  # v11.0コントローラでフルシミュレーション
  python -m weaving_v11.main --mode sim --load medium --tmax 600
  python -m weaving_v11.main --mode sim --load high --debug

  # クイックデバッグ実行 (60秒、spawn率削減)
  python -m weaving_v11.main --mode sim --debug

統合の詳細:
  files/IMPLEMENTATION_GUIDE_v11_0.md を参照
        """
    )

    parser.add_argument(
        '--mode',
        type=str,
        choices=['test', 'demo', 'sim'],
        default='test',
        help='実行モード: test (ユニットテスト), demo (コントローラ初期化), sim (フルシミュレーション)'
    )

    parser.add_argument(
        '--load',
        type=str,
        choices=['low', 'medium', 'high', 'congestion'],
        default='medium',
        help='シミュレーションの負荷レベル (デフォルト: medium)'
    )

    parser.add_argument(
        '--tmax',
        type=float,
        default=60.0,
        help='最大シミュレーション時間 [秒] (demo/simモード、デフォルト: demo=60s, sim=600s)'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='デバッグモード有効化 (高速実行: 60s, spawn率削減, 早期終了)'
    )

    parser.add_argument(
        '--hdv_ratio',
        type=float,
        default=0.0,
        help='混合交通におけるHDV（人間運転車両）の割合 (0.0-1.0、デフォルト: 0.0=全てCAV)'
    )

    parser.add_argument(
        '--until',
        type=str,
        choices=['aeb', 'collision', 'any'],
        default=None,
        help='イベントまで継続: aeb (AEB発動), collision (衝突発生), any (いずれか)'
    )

    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='パラメータオーバーライド用JSON設定ファイルのパス (最適化用)'
    )

    parser.add_argument(
        '--silent',
        action='store_true',
        help='サイレントモード: ログファイル出力を無効化、JSON_STATSのみ標準出力 (ベイズ最適化用)'
    )

    parser.add_argument(
        '--export_video',
        action='store_true',
        help='シミュレーション後にアニメーション動画をエクスポート (visualizationモジュールが必要)'
    )

    args = parser.parse_args(argv)

    print("=" * 80)
    print("CAV Weaving Zone Control System")
    print("Version: v11.0")
    print("=" * 80)

    success = True

    if args.mode == 'test':
        # Run tests
        if not test_frenet_qp():
            success = False

        if not test_urgency_planner():
            success = False

        if success:
            print("\n" + "=" * 80)
            print("[SUCCESS] All tests passed!")
            print("=" * 80)
        else:
            print("\n" + "=" * 80)
            print("[FAILED] Some tests failed")
            print("=" * 80)
            return 1

    elif args.mode == 'demo':
        demo_mode(args.tmax)

    elif args.mode == 'sim':
        # v18.20 FIX: Use provided tmax directly
        tmax = args.tmax
        # --until mode: set very long tmax ONLY if user didn't explicitly specify tmax
        if args.until:
            # Only auto-expand if tmax is the default (60.0)
            # If user explicitly set --tmax, respect that value
            if tmax == 60.0:
                tmax = 36000.0  # 10 hours max (auto-expanded)
                print(f"\n[EVENT MODE] Running until {args.until.upper()} event occurs (max {tmax}s, auto-expanded)")
            else:
                print(f"\n[EVENT MODE] Running until {args.until.upper()} event occurs (max {tmax}s, user-specified)")
        # Pass export flag via params by setting on the fly in simulation_mode
        # simulation_mode reads this from simulator.params.export_video
        # Here we simply call simulation_mode and set the flag inside via config_path or attribute
        success = simulation_mode(args.load, tmax, args.debug, args.hdv_ratio, args.until,
                      config_path=args.config,
                      export_video=args.export_video,
                      silent=args.silent)
        # Store flag in a global accessible way: set on params via a tiny hack here is not possible.
        # Instead, set environment variable and simulator.params reads it if implemented.
        # For now, we set it on the parameters inside simulation_mode by checking args.export_video below.
        if not success:
            print("\n" + "=" * 80)
            print("[FAILED] Simulation failed to run")
            print("=" * 80)
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
