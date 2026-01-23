# -*- coding: utf-8 -*-
# --- bayesian_opt_v11.py (v11.2 Exit-Urgency対応版 / 2026-01-19) ---
"""
CAV織り込み区間ベイズ最適化システム - v11.2 (Exit-Urgency対応版)
================================================================================

実行方法
--------------------------------------------------------------------------------
リポジトリルートから実行する場合:
    cd CAV-weaving-control
    python optimization/bayesian_opt_v11.py --n_trials 100 --load high

optimizationディレクトリから実行する場合:
    cd optimization
    python bayesian_opt_v11.py --n_trials 100 --load high

コマンドラインオプション:
    --n_trials <N>      : 最適化試行回数 (デフォルト: 100)
    --n_sims <N>        : 試行あたりのシミュレーション回数 (デフォルト: 5)
    --n_parallel_sims <N> : 1試行内のシミュレーション並列数 (デフォルト: 5)
    --load <LEVEL>      : 負荷レベル [low|medium|high|congestion] (デフォルト: high)
    --resume <FILES>    : 過去の試行ログから再開 (JSONLファイル、複数指定可)

実行例:
    # 新規探索開始 (100試行、high負荷)
    python optimization/bayesian_opt_v11.py --n_trials 100 --load high
    
    # シミュレーション設定をカスタマイズ (例: 3回SIM、2並列)
    python optimization/bayesian_opt_v11.py --n_trials 60 --n_sims 3 --n_parallel_sims 2 --load high
    
    # 全負荷レベルを順次最適化
    python optimization/bayesian_opt_v11.py --n_trials 60 --load low
    python optimization/bayesian_opt_v11.py --n_trials 60 --load medium
    python optimization/bayesian_opt_v11.py --n_trials 60 --load high
    
    # 過去ログから再開
    python optimization/bayesian_opt_v11.py --resume bayesian_opt_v11_trials_*.json --n_trials 60

出力ファイル:
- テキストログ: `bayesian_opt_v11_log_<タイムスタンプ>.txt`
- JSONL試行履歴: `bayesian_opt_v11_trials_<タイムスタンプ>.json`
- 最良パラメータ: `bayesian_opt_v12_best_<タイムスタンプ>.json`
- Optuna DB: `weaving_v11_opt.db` (SQLite)

最適化対象: weaving_v11 の全パラメータ
- bayesian_opt_v10_3.py のロジック・構造を厳密に踏襲
- v11.0 モジュラーアーキテクチャ対応 (CLI + JSON注入方式)

v11.2 特性:
- 試行内シミュレーション並列化: multiprocessing.Pool で高速化
- 5本中央値集約: 最小限の統計的頑健性
- Windows環境対応: 安定性優先、直列試行実行
- n_sims / n_parallel_sims 提供: ユーザーカスタマイズ可能

目的関数の評価基準:
1. 成功率の最大化 (基本スコア)
2. 衝突・AEB発生の最小化 (重ペナルティ)
3. エントロピー最大化 (LC分布) - v10.3より継承
4. 平均速度の最大化 (効率性)
5. ジニ係数の最小化 (公平性)

バージョン: 11.1
ベース: bayesian_opt_v10_3.py
"""

import numpy as np
import time
import optuna
from optuna.samplers import TPESampler
import multiprocessing as mp
import json
import os
import sys
import subprocess
import signal
from datetime import datetime
from typing import Dict, Any, List, Optional, Tuple, TextIO, cast

try:
    import psutil
except ImportError:
    psutil = None

# --- TeeLogger (v10.3由来) ---
# 標準出力をターミナルとファイルに同時出力するユーティリティ
class TeeLogger:
    """標準出力をターミナルとファイルに分岐するロガー"""
    def __init__(self, log_file: str, mode: str = 'w'):
        self.log_file = log_file
        self.mode = mode
        self.terminal = sys.stdout
        self.file: Optional[TextIO] = None
        

    def __enter__(self):
        self.file = cast(TextIO, open(self.log_file, self.mode, encoding='utf-8'))
        sys.stdout = self
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self.terminal
        if self.file:
            try:
                self.file.close()
            except Exception:
                pass  # Suppress errors during cleanup
        return False

    def write(self, message: str):
        self.terminal.write(message)
        self.terminal.flush()  # Ensure terminal output is flushed
        if self.file:
            try:
                self.file.write(message)
                self.file.flush()  # Ensure file output is flushed
            except Exception:
                pass  # Suppress errors if file writing fails

    def flush(self):
        self.terminal.flush()
        if self.file:
            try:
                self.file.flush()
            except Exception:
                pass  # Suppress errors if file flushing fails

# --- グローバル設定 ---
# ============================================================================
# ★★★ 並列化設定 (Windows環境で安定性優先) ★★★
# ============================================================================
# Windows環境での安定性のため、試行は常に直列実行されます。
# 試行内のシミュレーション並列化のみを使用します。
#
# パラメータ:
#   N_SIMS_PER_TRIAL: 試行あたりのシミュレーション回数（中央値集約用）
#   N_PARALLEL_SIMS: 1試行内のシミュレーション並列数
#   N_PARALLEL_TRIALS: 常に 1（試行レベル並列化なし）
# ============================================================================
N_SIMS_PER_TRIAL = 5         # 試行あたりのシミュレーション回数（中央値集約用）
N_PARALLEL_SIMS = 5          # 1試行内のシミュレーション並列数（効率向上）
N_PARALLEL_TRIALS = 1        # Optuna試行レベルの並列数（常に1、並列化無効）
SIM_TMAX = 300.0             # シミュレーション仮想時間 [秒] - 評価に必要な最小時間
TIMEOUT_SECONDS = 3600       # 実時間タイムアウト（秒）。300s仮想時間 × 並列実行を考慮

# --- 動的範囲適応設定 ---
RANGE_ADAPTATION_ENABLED = False     # 範囲適応機能の無効化（安定性優先、過度な狭窄を回避）
ADAPTATION_CHECK_INTERVAL = 5        # 適応チェック間隔 [試行数]
BOUNDARY_THRESHOLD = 0.8             # 境界ヒット率閾値（80%で範囲拡張）
EXPAND_FACTOR = 1.2                  # 範囲拡張係数（+20%）
SHRINK_FACTOR = 0.9                  # 範囲縮小係数（-10%）
MAX_EXPANSIONS = 2                   # パラメータ毎の最大拡張回数

# --- パラメータ探索範囲（動的に更新される）---
# 前回のベストパラメータ（2026-01-08）を中心に範囲設定
# ベスト値: qp_w_a=28.1, qp_w_j=5802.9, aeb_rss_factor=1.093, critical_rss_factor=1.559,
#          lc_min_front_gap=8.58, lc_min_rear_gap=20.39, lc_duration=4.20,
#          urgency_gamma=4.12, urgency_alpha=0.106, proactive_brake_threshold=-0.545
# V2夜間探索 (HIGH/queue, LC↑39%): lc_beta_1=11.0, urgency_gap_relax_coeff=0.5, lc_duration=3.0, lc_prep_duration=2.0
PARAM_RANGES = {
    'qp_w_a': {'min': 20.0, 'max': 30.0, 'initial_min': 20.0, 'initial_max': 30.0, 'expansions': 0},            # QP加速度重み
    'qp_w_j': {'min': 5000.0, 'max': 7000.0, 'initial_min': 5000.0, 'initial_max': 7000.0, 'expansions': 0},    # QPジャーク重み
    'aeb_rss_factor': {'min': 0.95, 'max': 1.05, 'initial_min': 0.95, 'initial_max': 1.05, 'expansions': 0},    # AEB発動RSS係数（mainの0.98周辺）
    'critical_rss_factor': {'min': 1.25, 'max': 1.40, 'initial_min': 1.25, 'initial_max': 1.40, 'expansions': 0},  # クリティカルRSS係数（mainの1.3周辺）

    # ============================================================================
    # ★★★ 重要警告: lc_min_front_gap は絶対に 12.0m 以上に設定すること ★★★
    # ============================================================================
    # 【デッドロック回避のための必須制約】
    #
    # 理由: 12.0m未満ではシミュレーションが無限ループに陥る（実証済み）
    #
    # 検証結果:
    #   ❌ lc_min_front_gap = 8.58m  → デッドロック発生
    #      - tmax=10秒のシミュレーションが60秒でタイムアウト
    #      - 車両がLC試行→失敗→再試行の無限ループ
    #      - 0108_2249の最適化で全64試行が失敗した原因
    #
    #   ✅ lc_min_front_gap = 12.62m → 正常動作
    #      - tmax=300秒のシミュレーションが正常進行（2026-01-09検証済み）
    #      - t=100秒以上まで進行確認
    #
    #   ✅ lc_min_front_gap = 15.0m (main) → 安全動作
    #      - Exit Rate 88.5%達成
    #
    # メカニズム:
    #   小さすぎるlc_min_front_gapは高密度交通でギャップ確保不可能
    #   → LC失敗 → 無限リトライ → シミュレーション時刻が進まない
    #
    # ★★★ 二度と12.0m未満の値を使用しないこと！ ★★★
    # ============================================================================
    'lc_min_front_gap': {'min': 12.0, 'max': 16.0, 'initial_min': 12.0, 'initial_max': 16.0, 'expansions': 0},  # LC前方最小ギャップ [m]
    # [2026-01-23] lc_min_rear_gap: ベスト値15.08が下限境界(1.2%)に張り付き → 下限を12.0に拡張
    'lc_min_rear_gap': {'min': 12.0, 'max': 22.0, 'initial_min': 12.0, 'initial_max': 22.0, 'expansions': 0},   # LC後方最小ギャップ [m]
    # LC実行時間（短縮方向も探索）
    'lc_duration': {'min': 2.5, 'max': 4.0, 'initial_min': 2.5, 'initial_max': 4.0, 'expansions': 0},           # LC実行時間 [秒]
    'urgency_gamma': {'min': 2.5, 'max': 4.5, 'initial_min': 2.5, 'initial_max': 4.5, 'expansions': 0},         # 緊急度曲線形状（早めに緊急度上昇）
    'urgency_alpha': {'min': 0.05, 'max': 0.3, 'initial_min': 0.05, 'initial_max': 0.3, 'expansions': 0},       # 緊急度下限値（下限を下げる）
    # [2026-01-23] proactive_brake_threshold: ベスト値-1.164が下限境界(9.1%)付近 → 下限を-1.5に拡張
    'proactive_brake_threshold': {'min': -1.5, 'max': -0.8, 'initial_min': -1.5, 'initial_max': -0.8, 'expansions': 0},  # 先行制動閾値 [m/s²]
    # V2 LC確率・ギャップ緩和・準備時間（Exit Rate向上に重要）
    'lc_beta_1': {'min': 10.0, 'max': 15.0, 'initial_min': 10.0, 'initial_max': 15.0, 'expansions': 0},         # LC確率勾配（積極性向上）
    'urgency_gap_relax_coeff': {'min': 0.4, 'max': 0.9, 'initial_min': 0.4, 'initial_max': 0.9, 'expansions': 0},  # 緊急度ギャップ緩和係数（緊急時の緩和強化）
    'lc_prep_duration': {'min': 1.0, 'max': 2.5, 'initial_min': 1.0, 'initial_max': 2.5, 'expansions': 0},      # LC準備時間 [秒]（短縮）
    # Exit近傍の緊急度パラメータ（Apollo Safety動的閾値制御）
    'exit_urgent_dist': {'min': 70.0, 'max': 130.0, 'initial_min': 70.0, 'initial_max': 130.0, 'expansions': 0},  # Exit緊急距離閾値 [m]
    # [2026-01-23] exit_emergency_dist: ベスト値78.27が上限境界(96.5%)に張り付き → 上限を100.0に拡張
    'exit_emergency_dist': {'min': 30.0, 'max': 100.0, 'initial_min': 30.0, 'initial_max': 100.0, 'expansions': 0},  # Exit非常距離閾値 [m]
    'exit_long_relax_urgent': {'min': 0.6, 'max': 0.9, 'initial_min': 0.6, 'initial_max': 0.9, 'expansions': 0},  # Exit緊急時縦方向緩和係数
    'exit_lat_relax_urgent': {'min': 0.6, 'max': 0.9, 'initial_min': 0.6, 'initial_max': 0.9, 'expansions': 0},   # Exit緊急時横方向緩和係数
    # [2026-01-23] exit_long_relax_emerg: ベスト値0.663が上限付近(87.7%) → 上限を0.85に拡張
    'exit_long_relax_emerg': {'min': 0.4, 'max': 0.85, 'initial_min': 0.4, 'initial_max': 0.85, 'expansions': 0},   # Exit非常時縦方向緩和係数
    'exit_lat_relax_emerg': {'min': 0.4, 'max': 0.7, 'initial_min': 0.4, 'initial_max': 0.7, 'expansions': 0},    # Exit非常時横方向緩和係数
    'exit_scale_floor': {'min': 0.25, 'max': 0.5, 'initial_min': 0.25, 'initial_max': 0.5, 'expansions': 0},      # Exit緊急度スケーリング下限
}

# --- Efficiency Normalization Config (Speed/Travel Time) ---
# 評価は速度↑・旅行時間↓を同時に最適化。線形正規化で0..1にスケール。
# 範囲はシナリオ前提に依存するため、必要に応じて調整してください。
SPEED_NORM_MIN = 15.0   # m/s （渋滞〜通常域の下限）
SPEED_NORM_MAX = 25.0   # m/s （流れが良い上限目安）
TIME_NORM_MIN = 180.0   # s   （良好ケースの下限想定）
TIME_NORM_MAX = 240.0   # s   （渋滞・遅延時の上限想定）

# 速度と旅行時間の重み（合計1.0を推奨）
EFF_WEIGHT_SPEED = 0.6
EFF_WEIGHT_TIME = 0.4

# --- 最適化ループ設定 ---
# 反復回数（n_trials）は調整しやすいよう定数化
# 16時間想定: 1試行 ≈ 17分 (340s × 3 ÷ 4) → 16時間 ≈ 60試行
N_TRIALS = 60

def check_and_adapt_ranges(study, param_name):
    """
    パラメータ値が境界に達しているか確認し、必要に応じて範囲を適応させる。
    
    引数:
        study: Optunaのstudyオブジェクト
        param_name: チェックするパラメータ名
    
    戻り値:
        適応された場合は (min, max) のタプル、そうでなければ None
    """
    if not RANGE_ADAPTATION_ENABLED or param_name not in PARAM_RANGES:
        return None
    
    # Get recent trials (last ADAPTATION_CHECK_INTERVAL)
    trials = study.trials[-ADAPTATION_CHECK_INTERVAL:] if len(study.trials) >= ADAPTATION_CHECK_INTERVAL else study.trials
    
    if len(trials) < 3:  # Need at least 3 trials to assess
        return None
    
    param_values = [t.params.get(param_name) for t in trials if param_name in t.params and t.state == optuna.trial.TrialState.COMPLETE]
    
    if len(param_values) < 3:
        return None
    
    current_range = PARAM_RANGES[param_name]
    current_min = current_range['min']
    current_max = current_range['max']
    
    # Check boundary hits
    boundary_margin = (current_max - current_min) * 0.05  # 5% margin
    hits_lower = sum(1 for v in param_values if v <= current_min + boundary_margin)
    hits_upper = sum(1 for v in param_values if v >= current_max - boundary_margin)
    
    total_boundary_hits = hits_lower + hits_upper
    boundary_hit_rate = total_boundary_hits / len(param_values)
    
    # Expand if hitting boundaries frequently
    if boundary_hit_rate >= BOUNDARY_THRESHOLD and current_range['expansions'] < MAX_EXPANSIONS:
        old_min, old_max = current_min, current_max
        
        if hits_lower > hits_upper:
            # Expand lower bound
            range_width = current_max - current_min
            new_min = current_min - range_width * (EXPAND_FACTOR - 1.0)
            new_max = current_max
        elif hits_upper > hits_lower:
            # Expand upper bound
            range_width = current_max - current_min
            new_min = current_min
            new_max = current_max + range_width * (EXPAND_FACTOR - 1.0)
        else:
            # Expand both
            range_width = current_max - current_min
            expansion = range_width * (EXPAND_FACTOR - 1.0) / 2
            new_min = current_min - expansion
            new_max = current_max + expansion
        
        # Apply reasonable absolute limits
        if param_name == 'lc_beta_1':
            new_min = max(3.0, new_min)
            new_max = min(15.0, new_max)
        elif param_name == 'urgency_gap_relax_coeff':
            new_min = max(0.0, new_min)
            new_max = min(1.5, new_max)
        elif param_name == 'lc_duration':
            new_min = max(2.0, new_min)
            new_max = min(5.0, new_max)
        elif param_name == 'lc_min_front_gap':
            new_min = max(8.0, new_min)
            new_max = min(20.0, new_max)
        elif param_name == 'lc_prep_duration':
            new_min = max(1.0, new_min)
            new_max = min(3.0, new_max)
        
        # Update range
        PARAM_RANGES[param_name]['min'] = new_min
        PARAM_RANGES[param_name]['max'] = new_max
        PARAM_RANGES[param_name]['expansions'] += 1
        
        print(f"  [Range Adaptation] {param_name}: [{old_min:.3f}, {old_max:.3f}] → [{new_min:.3f}, {new_max:.3f}]")
        print(f"    Reason: {boundary_hit_rate*100:.1f}% boundary hits (threshold={BOUNDARY_THRESHOLD*100:.0f}%)")
        
        return (new_min, new_max)
    
    # Shrink if no boundary hits (concentrate search)
    elif boundary_hit_rate < 0.2 and len(study.trials) > 10:
        # Calculate actual value range
        actual_min = min(param_values)
        actual_max = max(param_values)
        actual_range = actual_max - actual_min
        
        # Only shrink if current range is much wider than actual usage
        if (current_max - current_min) > actual_range * 2.0:
            margin = actual_range * 0.1  # 10% margin
            new_min = max(current_range['initial_min'], actual_min - margin)
            new_max = min(current_range['initial_max'], actual_max + margin)
            
            # Update range
            old_min, old_max = current_min, current_max
            PARAM_RANGES[param_name]['min'] = new_min
            PARAM_RANGES[param_name]['max'] = new_max
            
            print(f"  [Range Adaptation] {param_name}: [{old_min:.3f}, {old_max:.3f}] → [{new_min:.3f}, {new_max:.3f}]")
            print(f"    Reason: Concentrating search (actual range: [{actual_min:.3f}, {actual_max:.3f}])")
            
            return (new_min, new_max)
    
    return None

def run_simulation_task(task_args):
    """
    単一のシミュレーションを指定パラメータで実行する。

    引数:
        task_args: (params, load_level, trial_id, run_idx) のタプル

    戻り値:
        シミュレーション結果の統計辞書
    """
    params, load_level, trial_id, run_idx = task_args

    # 一時設定ファイルをユニークな名前で作成（タイムスタンプ追加で衝突回避）
    timestamp_ms = int(time.time() * 1000)
    pid = os.getpid()
    config_filename = f"temp_config_{trial_id}_{run_idx}_{pid}_{timestamp_ms}.json"
    output_filename = f"temp_output_{trial_id}_{run_idx}_{pid}_{timestamp_ms}.txt"
    error_filename = f"temp_error_{trial_id}_{run_idx}_{pid}_{timestamp_ms}.txt"
    
    try:
        # パラメータを JSON に保存
        with open(config_filename, 'w') as f:
            json.dump(params, f, indent=2)
        
        # コマンド構築
        # ベイズ最適化時は --silent で詳細ログを無効化（ファイルI/O削減）
        cmd = [
            sys.executable, "-m", "weaving_v11.main",
            "--mode", "sim",
            "--load", load_level,
            "--tmax", str(int(SIM_TMAX)),
            "--config", config_filename,
            "--until", "collision",  # 衝突発生時に早期停止
            "--silent"  # 詳細ログを無効化（JSON_STATSのみ出力）
        ]
        
        # ベイズ最適化時はログファイルに書き込まず、直接パース
        # stdoutにJSON_STATSが出力されるのでそれをキャプチャ
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,  # エラーログは破棄
            cwd=os.getcwd(),
            timeout=TIMEOUT_SECONDS,
            text=True,
            encoding='utf-8',
            errors='ignore'
        )
        
        # Check return code
        if result.returncode != 0:
            raise subprocess.CalledProcessError(
                result.returncode,
                cmd,
                output=f"Simulation failed with code {result.returncode}"
            )
        
        # stdoutからJSON_STATSを抽出
        stdout_content = result.stdout if result.stdout else ""
        
        # Parse results from JSON_STATS in stdout
        stats = parse_simulation_stdout(stdout_content)
        
        # DEBUG: Check parsing result
        if stats['collision_count'] == 99:
            print(f"[DEBUG] Trial {trial_id} Run {run_idx}: JSON parsing failed or no JSON_STATS found")
            if "[JSON_STATS]" in stdout_content:
                print(f"[DEBUG] stdout HAS [JSON_STATS] marker but parsing failed")
            else:
                print(f"[DEBUG] stdout MISSING [JSON_STATS] marker")

        # Check if simulation actually succeeded
        if stats['collision_count'] == 99 and stats['aeb_count'] == 99:
            print(f"[WARNING] Trial {trial_id} Run {run_idx}: Simulation failed to produce valid results")

    except subprocess.TimeoutExpired as e:
        print(f"[ERROR] Simulation timeout (trial={trial_id}, run={run_idx}): Process exceeded {TIMEOUT_SECONDS}s")
        stats = get_default_failed_stats()
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Trial {trial_id} Run {run_idx} failed (returncode={e.returncode})")
        # エラー出力を表示
        if os.path.exists(output_filename):
            try:
                with open(output_filename, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                    if lines:
                        print(f"[Last 15 lines of output]")
                        for line in lines[-15:]:
                            print(f"  {line.rstrip()}")
                    else:
                        print(f"[ERROR] Output file is empty!")
            except Exception as read_err:
                print(f"[ERROR] Could not read output file: {read_err}")
        else:
            print(f"[ERROR] Output file not created: {output_filename}")
        stats = get_default_failed_stats()
    except RuntimeError as e:
        print(f"[ERROR] Trial {trial_id} Run {run_idx}: {e}")
        if os.path.exists(output_filename):
            try:
                with open(output_filename, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                    if lines:
                        print(f"[RuntimeError - Last 10 lines]")
                        for line in lines[-10:]:
                            print(f"  {line.rstrip()}")
            except:
                pass
        stats = get_default_failed_stats()
    except Exception as e:
        print(f"[ERROR] Trial {trial_id} Run {run_idx}: {type(e).__name__}: {e}")
        if os.path.exists(output_filename):
            try:
                with open(output_filename, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                    if lines:
                        print(f"[Exception - Last 10 lines of output]")
                        for line in lines[-10:]:
                            print(f"  {line.rstrip()}")
            except:
                pass
        stats = get_default_failed_stats()
    finally:
        # Cleanup config file only (no output files created in silent mode)
        if os.path.exists(config_filename):
            for retry in range(3):
                try:
                    os.remove(config_filename)
                    break
                except PermissionError:
                    if retry < 2:
                        time.sleep(0.1)
                except Exception:
                    break

    return stats

def get_default_failed_stats():
    """シミュレーション失敗時のデフォルト統計値を返す"""
    return {
        'success_rate': 0.0,
        'avg_travel_time': 999.0,
        'avg_speed': 0.0,
        'entropy_prep': 0.0, 
        'entropy_weave': 0.0,
        'gini_coef_prep': 1.0, 
        'gini_coef_weave': 1.0,
        'collision_count': 99,
        'aeb_count': 99
    }

def parse_simulation_stdout(stdout_content: str):
    """
    stdoutからJSON_STATSをパースする。
    
    引数:
        stdout_content: シミュレーションのstdout文字列
    
    戻り値:
        パース済み統計の辞書
    """
    stats = get_default_failed_stats()
    found_json = False
    
    if not stdout_content.strip():
        return stats
    
    try:
        # JSON_STATS を探す
        if "[JSON_STATS]" in stdout_content:
            for line in stdout_content.split('\n'):
                if "[JSON_STATS]" in line:
                    json_str = line.split("[JSON_STATS]", 1)[1].strip()
                    try:
                        raw_stats = json.loads(json_str)
                        # Map raw stats to standard keys + LC-specific metrics
                        lc_stats = raw_stats.get('lc', {})
                        stats['success_rate'] = lc_stats.get('overall_success_rate', lc_stats.get('exit_success_rate', 0.0))
                        stats['lc_execution_rate'] = lc_stats.get('lc_execution_rate', 0.0)
                        stats['exit_success_rate'] = lc_stats.get('exit_success_rate', 0.0)
                        # Clamp rates to [0,1]
                        for k in ('success_rate','lc_execution_rate','exit_success_rate'):
                            try:
                                stats[k] = float(np.clip(stats.get(k, 0.0), 0.0, 1.0))
                            except Exception:
                                stats[k] = 0.0
                        
                        stats['avg_travel_time'] = raw_stats.get('avg_travel_time', 999.0)
                        stats['avg_speed'] = raw_stats.get('avg_speed_all', raw_stats.get('avg_speed', 0.0))
                        
                        stats['entropy_prep'] = raw_stats.get('entropy_prep', 0.0)
                        stats['entropy_weave'] = raw_stats.get('entropy_weave', 0.0)
                        stats['gini_coef_prep'] = raw_stats.get('gini_coef_prep', 1.0)
                        stats['gini_coef_weave'] = raw_stats.get('gini_coef_weave', 1.0)
                        stats['gini_coef_overall'] = raw_stats.get('gini_coef_overall', raw_stats.get('gini_coef_weave', 1.0))
                        stats['collision_count'] = raw_stats.get('collision_count', 0)
                        stats['aeb_count'] = raw_stats.get('aeb_count', 0)
                        found_json = True
                    except json.JSONDecodeError:
                        print("[WARNING] JSON decode error in stats")
    except Exception as e:
        print(f"[WARNING] Exception parsing stdout: {e}")
    
    return stats

def parse_simulation_output(filename):
    """
    出力ファイルからJSON_STATSをパースする。
    ファイルが見つからない場合は失敗統計を返す。
    
    引数:
        filename: シミュレーション出力ファイルのパス
    
    戻り値:
        パース済み統計の辞書
    """
    stats = get_default_failed_stats()
    found_json = False
    
    # ファイルが存在しない場合は即座に失敗を返す
    if not os.path.exists(filename):
        print(f"[WARNING] Output file not found: {filename}")
        return stats
    
    try:
        with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
            file_content = f.read()
        
        # Check if file is empty
        if not file_content.strip():
            print(f"[WARNING] Output file is empty: {filename}")
            return stats
            
        # JSON_STATS を探す
        if "[JSON_STATS]" in file_content:
            for line in file_content.split('\n'):
                if "[JSON_STATS]" in line:
                    json_str = line.split("[JSON_STATS]", 1)[1].strip()
                    try:
                        raw_stats = json.loads(json_str)
                        # Map raw stats to standard keys + LC-specific metrics
                        lc_stats = raw_stats.get('lc', {})
                        # success_rate: 全体成功率（全車両に対する目標到達率）
                        # JSONでは 0..1 の比率で返ってくる（simulator.get_statistics）
                        stats['success_rate'] = lc_stats.get('overall_success_rate', lc_stats.get('exit_success_rate', 0.0))
                        # v2-critical metrics: LC execution & Exit success (lane-appropriate exit)
                        stats['lc_execution_rate'] = lc_stats.get('lc_execution_rate', 0.0)
                        stats['exit_success_rate'] = lc_stats.get('exit_success_rate', 0.0)
                        # Clamp rates to [0,1]
                        for k in ('success_rate','lc_execution_rate','exit_success_rate'):
                            try:
                                stats[k] = float(np.clip(stats.get(k, 0.0), 0.0, 1.0))
                            except Exception:
                                stats[k] = 0.0
                        
                        stats['avg_travel_time'] = raw_stats.get('avg_travel_time', 999.0)
                        stats['avg_speed'] = raw_stats.get('avg_speed_all', raw_stats.get('avg_speed', 0.0))
                        
                        stats['entropy_prep'] = raw_stats.get('entropy_prep', 0.0)
                        stats['entropy_weave'] = raw_stats.get('entropy_weave', 0.0)
                        stats['gini_coef_prep'] = raw_stats.get('gini_coef_prep', 1.0)
                        stats['gini_coef_weave'] = raw_stats.get('gini_coef_weave', 1.0)
                        stats['gini_coef_overall'] = raw_stats.get('gini_coef_overall', raw_stats.get('gini_coef_weave', 1.0))
                        stats['collision_count'] = raw_stats.get('collision_count', 0)
                        stats['aeb_count'] = raw_stats.get('aeb_count', 0)
                        found_json = True
                    except json.JSONDecodeError:
                        print("[WARNING] JSON decode error in stats")
        else:
            print(f"[WARNING] JSON_STATS not found in output file: {filename}")
                        
    except Exception as e:
        print(f"[WARNING] Exception reading output file: {e}")
        
    # Fallback Parsing if JSON missing (legacy/crash)
    if not found_json:
        # Minimal extraction for basic error reporting
        # (Omitted reuse of text parsing for brevity, reliance on JSON enabled in v27.15)
        # Assuming v27.15+ main.py is used.
        pass
        
    return stats

def objective(trial, load_level, n_sims_per_trial=5, pool=None):
    """
    Optunaの目的関数。シミュレーションを実行し、スコアを計算する。

    優先順位（ハード制約 → 重み順）:
      1) 衝突ゼロ（ハード制約: 1件でもあれば -999999）
      2) Gini最小化（LCの空間的公平性）
      3) 成功率（overall/exit）最大化
      4) 旅行時間の短縮（速度は補助で関与）

    現在の重み（経験的に安定な比率）:
      - w_gini = 1000.0, w_success = 500.0, w_time = 100.0

    引数:
        trial: Optunaの試行オブジェクト
        load_level: 負荷レベル ('low', 'medium', 'high', 'congestion')
        n_sims_per_trial: シミュレーション回数
        pool: multiprocessing.Pool インスタンス（再利用）

    戻り値:
        total_score: 最適化スコア（最大化対象）
    """
    # --- 前回ベスト結果 (2026-01-08) を基に探索空間を拡張 ---
    # 前回ベストスコア: -103030.0 (Exit最優先)
    # 目標: 前回最適値周辺を精密化
    params = {}
    params['spawn_mode'] = 'queue'  # ベストモードとして確認済み
    
    # QPコントローラ重み (ベスト: qp_w_a=28.1, qp_w_j=5802.9)
    r_qp_a = PARAM_RANGES['qp_w_a']
    params['qp_w_a'] = trial.suggest_float('qp_w_a', r_qp_a['min'], r_qp_a['max'])
    
    r_qp_j = PARAM_RANGES['qp_w_j']
    params['qp_w_j'] = trial.suggest_float('qp_w_j', r_qp_j['min'], r_qp_j['max'])
    
    # 安全RSS係数 (ベスト: aeb=1.093, critical=1.559)
    r_aeb = PARAM_RANGES['aeb_rss_factor']
    params['aeb_rss_factor'] = trial.suggest_float('aeb_rss_factor', r_aeb['min'], r_aeb['max'])
    
    r_critical = PARAM_RANGES['critical_rss_factor']
    params['critical_rss_factor'] = trial.suggest_float('critical_rss_factor', r_critical['min'], r_critical['max'])
    
    # LCギャップ制約
    r_front_gap = PARAM_RANGES['lc_min_front_gap']
    params['lc_min_front_gap'] = trial.suggest_float('lc_min_front_gap', r_front_gap['min'], r_front_gap['max'])

    # ★★★ デッドロック回避の必須検証 ★★★
    # lc_min_front_gapが12.0m未満の場合は即座に失敗スコアを返す
    if params['lc_min_front_gap'] < 12.0:
        print(f"[CRITICAL] Trial {trial.number}: lc_min_front_gap={params['lc_min_front_gap']:.2f}m < 12.0m")
        print(f"           This will cause deadlock. Returning worst score.")
        trial.set_user_attr('lc_rate', 0.0)
        trial.set_user_attr('exit_rate', 0.0)
        trial.set_user_attr('gini_weave', 1.0)
        trial.set_user_attr('collision_count', 99)
        trial.set_user_attr('aeb_count', 99)
        trial.set_user_attr('avg_speed', 0.0)
        trial.set_user_attr('avg_travel_time', 999.0)
        trial.set_user_attr('efficiency_score', 0.0)
        return -999999.0

    r_rear_gap = PARAM_RANGES['lc_min_rear_gap']
    params['lc_min_rear_gap'] = trial.suggest_float('lc_min_rear_gap', r_rear_gap['min'], r_rear_gap['max'])
    
    # LC実行時間 (V2ベスト3.0秒とBOベスト4.20秒をカバー)
    r_duration = PARAM_RANGES['lc_duration']
    params['lc_duration'] = trial.suggest_float('lc_duration', r_duration['min'], r_duration['max'])

    # V2 LC確率勾配 (ロジットβ1) とギャップ緩和係数
    r_lc_beta1 = PARAM_RANGES['lc_beta_1']
    params['lc_beta_1'] = trial.suggest_float('lc_beta_1', r_lc_beta1['min'], r_lc_beta1['max'])

    r_gap_relax = PARAM_RANGES['urgency_gap_relax_coeff']
    params['urgency_gap_relax_coeff'] = trial.suggest_float('urgency_gap_relax_coeff', r_gap_relax['min'], r_gap_relax['max'])

    # LC準備時間 (予約から実行まで)
    r_lc_prep = PARAM_RANGES['lc_prep_duration']
    params['lc_prep_duration'] = trial.suggest_float('lc_prep_duration', r_lc_prep['min'], r_lc_prep['max'])
    
    # 緊急度プランナーパラメータ (ベスト: gamma=4.12, alpha=0.106)
    r_gamma = PARAM_RANGES['urgency_gamma']
    params['urgency_gamma'] = trial.suggest_float('urgency_gamma', r_gamma['min'], r_gamma['max'])
    
    r_alpha = PARAM_RANGES['urgency_alpha']
    params['urgency_alpha'] = trial.suggest_float('urgency_alpha', r_alpha['min'], r_alpha['max'])
    
    # 先行制動閾値 (ベスト: -0.545)
    r_brake = PARAM_RANGES['proactive_brake_threshold']
    params['proactive_brake_threshold'] = trial.suggest_float('proactive_brake_threshold', r_brake['min'], r_brake['max'])
    
    # Exit近傍緊急度パラメータ（Apollo Safety動的閾値制御）
    r_exit_urgent = PARAM_RANGES['exit_urgent_dist']
    params['exit_urgent_dist'] = trial.suggest_float('exit_urgent_dist', r_exit_urgent['min'], r_exit_urgent['max'])
    
    r_exit_emerg = PARAM_RANGES['exit_emergency_dist']
    params['exit_emergency_dist'] = trial.suggest_float('exit_emergency_dist', r_exit_emerg['min'], r_exit_emerg['max'])
    
    # 制約: exit_emergency_dist < exit_urgent_dist （物理的に矛盾しないように）
    if params['exit_emergency_dist'] >= params['exit_urgent_dist']:
        params['exit_emergency_dist'] = params['exit_urgent_dist'] * 0.8
    
    r_exit_long_urgent = PARAM_RANGES['exit_long_relax_urgent']
    params['exit_long_relax_urgent'] = trial.suggest_float('exit_long_relax_urgent', r_exit_long_urgent['min'], r_exit_long_urgent['max'])
    
    r_exit_lat_urgent = PARAM_RANGES['exit_lat_relax_urgent']
    params['exit_lat_relax_urgent'] = trial.suggest_float('exit_lat_relax_urgent', r_exit_lat_urgent['min'], r_exit_lat_urgent['max'])
    
    r_exit_long_emerg = PARAM_RANGES['exit_long_relax_emerg']
    params['exit_long_relax_emerg'] = trial.suggest_float('exit_long_relax_emerg', r_exit_long_emerg['min'], r_exit_long_emerg['max'])
    
    r_exit_lat_emerg = PARAM_RANGES['exit_lat_relax_emerg']
    params['exit_lat_relax_emerg'] = trial.suggest_float('exit_lat_relax_emerg', r_exit_lat_emerg['min'], r_exit_lat_emerg['max'])
    
    r_exit_scale = PARAM_RANGES['exit_scale_floor']
    params['exit_scale_floor'] = trial.suggest_float('exit_scale_floor', r_exit_scale['min'], r_exit_scale['max'])
    
    # シミュレーション実行
    task_args = []
    for i in range(n_sims_per_trial):
        task_args.append((params, load_level, trial.number, i))
    
    # Pool が渡されていればそれを使用、なければ新規作成（後方互換性）
    if pool is not None:
        results = pool.map(run_simulation_task, task_args)
    else:
        # 旧動作: 毎回Pool作成（非推奨、オーバーヘッド大）
        with mp.Pool(n_sims_per_trial) as temp_pool:
            results = temp_pool.map(run_simulation_task, task_args)
        
    # --- 中央値集約（頑健性確保）---
    # n_sims_per_trial回のシミュレーション結果から中央値を取得
    # これにより、確率的なばらつきの影響を軽減
    r_lc_rate = np.array([r.get('lc_execution_rate', 0.0) for r in results])
    r_exit_rate = np.array([r.get('exit_success_rate', 0.0) for r in results])
    r_speed = np.array([r.get('avg_speed', 0.0) for r in results])
    r_time = np.array([r.get('avg_travel_time', 999.0) for r in results])
    r_gini_weave = np.array([r.get('gini_coef_weave', 1.0) for r in results])
    r_collision = np.array([r.get('collision_count', 0) for r in results])
    r_aeb = np.array([r.get('aeb_count', 0) for r in results])

    # 中央値を計算
    # 注意: 衝突判定は中央値を使用
    # → 5回中3回以上衝突した場合のみペナルティ（統計的に有意な問題と判断）
    # → より厳格にする場合は、np.sum(r_collision) > 0 で1回でも失格にできる
    med_lc_rate = np.median(r_lc_rate) if len(r_lc_rate) > 0 else 0.0
    med_exit_rate = np.median(r_exit_rate) if len(r_exit_rate) > 0 else 0.0
    med_speed = np.median(r_speed)
    med_time = np.median(r_time)
    med_collision = int(np.median(r_collision))
    med_gini_weave = np.median(r_gini_weave) if len(r_gini_weave) > 0 else 1.0
    med_aeb = int(np.median(r_aeb))

    # --- Exit最優先目的関数: Exit最大化、安全性確保、次いでLC・効率性 ---
    # 正規化ヘルパー関数
    def _lin_norm(x, lo, hi):
        """線形正規化を行う [0, 1] にスケール"""
        if hi <= lo:
            return 0.0
        return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0))

    norm_speed = _lin_norm(med_speed, SPEED_NORM_MIN, SPEED_NORM_MAX)
    norm_time = 1.0 - _lin_norm(med_time, TIME_NORM_MIN, TIME_NORM_MAX)

    # Exit Rate 95%達成特化スコア: 衝突ゼロ >> Exit Rate最優先 > 時間効率 > Gini最小化
    # Exit Rateを最大化し、Gini係数は最低限考慮
    w_exit = 1000.0      # Exit Rate最優先（旧w_success=500から倍増）
    w_gini = 50.0        # Gini係数は最低限（旧1000から大幅削減）
    w_time = 100.0       # 時間効率は補助的
    efficiency_score = (0.6 * norm_speed + 0.4 * norm_time)
    score_exit = float(med_exit_rate) * w_exit
    score_gini = (1.0 - float(med_gini_weave)) * w_gini
    score_time = float(norm_time) * w_time
    total_score = score_exit + score_gini + score_time

    # ハード制約: 衝突は許容しない
    if med_collision > 0:
        total_score = -999999.0

    print(
        f"Trial {trial.number}: Score={total_score:.1f} | Exit={med_exit_rate:.3f} (target:0.95+) GiniW={med_gini_weave:.3f} "
        f"Time={med_time:.1f}s Speed={med_speed:.2f}m/s Col={med_collision} AEB={med_aeb}"
    )

    # 分析用にメトリクスを試行属性に保存
    trial.set_user_attr('lc_rate', float(med_lc_rate))
    trial.set_user_attr('exit_rate', float(med_exit_rate))
    trial.set_user_attr('gini_weave', float(med_gini_weave))
    trial.set_user_attr('collision_count', int(med_collision))
    trial.set_user_attr('aeb_count', int(med_aeb))
    trial.set_user_attr('avg_speed', float(med_speed))
    trial.set_user_attr('avg_travel_time', float(med_time))
    trial.set_user_attr('efficiency_score', float(efficiency_score))
    
    return total_score

def load_previous_trials_from_json(study: Any, json_log_path: str) -> int:
    """
    以前のJSONログファイルから探索結果を読み込む（v10.3準拠）。
    
    引数:
        study: Optunaのstudyオブジェクト
        json_log_path: JSONログファイルのパス
    
    戻り値:
        読み込んだ試行数
    """
    print(f"\n[再開] 以前の試行を読み込み: {json_log_path}")
    
    # 現在のパラメータ分布定義（前回のベストを中心に拡張）
    distributions = {
        'qp_w_a': optuna.distributions.FloatDistribution(20.0, 35.0),
        'qp_w_j': optuna.distributions.FloatDistribution(4500.0, 7000.0),
        'aeb_rss_factor': optuna.distributions.FloatDistribution(0.9, 1.2),
        'critical_rss_factor': optuna.distributions.FloatDistribution(1.3, 1.8),
        'lc_min_front_gap': optuna.distributions.FloatDistribution(7.0, 18.0),
        'lc_min_rear_gap': optuna.distributions.FloatDistribution(18.0, 24.0),
        'lc_duration': optuna.distributions.FloatDistribution(2.5, 4.5),
        'lc_beta_1': optuna.distributions.FloatDistribution(7.0, 13.0),
        'urgency_gap_relax_coeff': optuna.distributions.FloatDistribution(0.0, 1.0),
        'lc_prep_duration': optuna.distributions.FloatDistribution(1.5, 3.0),
        'urgency_gamma': optuna.distributions.FloatDistribution(3.0, 5.0),
        'urgency_alpha': optuna.distributions.FloatDistribution(0.08, 0.4),
        'proactive_brake_threshold': optuna.distributions.FloatDistribution(-1.0, -0.3),
    }
    
    try:
        count = 0
        skipped = 0
        with open(json_log_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    trial_data = json.loads(line)
                    params = trial_data.get('params', {})
                    value = trial_data.get('value', None)
                    
                    if params and value is not None:
                        # Exit関連パラメータの追加（ロード互換性確保）
                        # 過去の試行にはexit_*パラメータがないため、デフォルト値で補完
                        if 'exit_urgent_dist' not in params:
                            params['exit_urgent_dist'] = 100.0
                        if 'exit_emergency_dist' not in params:
                            params['exit_emergency_dist'] = 50.0
                        if 'exit_long_relax_urgent' not in params:
                            params['exit_long_relax_urgent'] = 0.7
                        if 'exit_lat_relax_urgent' not in params:
                            params['exit_lat_relax_urgent'] = 0.75
                        if 'exit_long_relax_emerg' not in params:
                            params['exit_long_relax_emerg'] = 0.5
                        if 'exit_lat_relax_emerg' not in params:
                            params['exit_lat_relax_emerg'] = 0.6
                        if 'exit_scale_floor' not in params:
                            params['exit_scale_floor'] = 0.3
                        
                        # パラメータの正規化（欠損パラメータにデフォルト値を設定）
                        normalized_params = {}
                        for param_name, dist in distributions.items():
                            if param_name in params:
                                normalized_params[param_name] = params[param_name]
                            else:
                                # デフォルト値（範囲の中央）
                                normalized_params[param_name] = (dist.low + dist.high) / 2
                        
                        try:
                            frozen_trial = optuna.trial.FrozenTrial(
                                number=len(study.trials),
                                state=optuna.trial.TrialState.COMPLETE,
                                value=value,
                                datetime_start=None,
                                datetime_complete=None,
                                params=normalized_params,
                                distributions=distributions,
                                user_attrs={},
                                system_attrs={},
                                intermediate_values={},
                                trial_id=len(study.trials),
                            )
                            study.add_trial(frozen_trial)
                            count += 1
                            if count <= 5 or count % 10 == 0:
                                print(f"  [{count}] value={value:.2f} (loaded)")
                        except Exception as e:
                            # add_trial失敗時はenqueue_trialにフォールバック
                            study.enqueue_trial(normalized_params)
                            count += 1
                except json.JSONDecodeError:
                    skipped += 1
                except Exception:
                    skipped += 1
        
        print(f"[Resume] Loaded {count} trials (skipped: {skipped})\n")
        return count
    
    except FileNotFoundError:
        print(f"[Warning] File not found: {json_log_path}\n")
        return 0
    except Exception as e:
        print(f"[Error] Failed to load log file: {e}\n")
        return 0


def auto_detect_optimal_workers() -> int:
    """
    システムリソースに基づいて最適な並列ワーカー数を自動判定
    
    判定基準:
    - CPU論理コア数
    - 利用可能メモリ（1試行あたり約2GB想定）
    - 1試行内で5シム並列実行（N_PARALLEL_SIMS=5）を考慮
    - 試行レベル並列度 = CPU数 // N_PARALLEL_SIMS で最適化
    
    戻り値:
        推奨試行レベル並列ワーカー数（N_PARALLEL_TRIALS）
    """
    cpu_count = mp.cpu_count()
    
    # メモリベースの制限
    max_workers_by_memory = cpu_count
    if psutil is not None:
        try:
            mem = psutil.virtual_memory()
            available_gb = mem.available / (1024**3)
            # 1試行あたり2GB想定、システム用に4GB確保
            max_workers_by_memory = max(1, int((available_gb - 4) / 2))
        except Exception:
            pass
    
    # 試行レベル並列度の計算
    # N_PARALLEL_SIMS（5シム/試行）を考慮して、CPUを有効活用
    # 例: 20コアなら20//5=4試行、10コアなら10//5=2試行を同時実行
    sims_per_trial = N_PARALLEL_SIMS
    trial_parallelism = max(1, cpu_count // sims_per_trial)
    
    # メモリ制限を考慮
    recommended = min(trial_parallelism, max_workers_by_memory)
    optimal = min(recommended, max_workers_by_memory)
    optimal = max(1, optimal)  # 最低1ワーカー
    
    return optimal


def _write_best_snapshot(study: "optuna.study.Study", timestamp: str, load_level: str) -> Optional[str]:
    """
    現在のベスト試行をスナップショットとして JSON に保存する。
    - 中断時/各試行後に呼び出される想定
    - 戻り値: 保存したファイルパス（失敗時は None）
    """
    try:
        if not study.best_trials:
            return None
        bt = study.best_trial
        payload = {
            'params': bt.params,
            'metrics': bt.user_attrs,
            'score': bt.value,
            'study_name': study.study_name,
            'storage': getattr(study, 'storage', None).__class__.__name__ if hasattr(study, 'storage') else 'unknown',
            'timestamp': datetime.now().isoformat(),
        }
        # タイムスタンプ付きファイル（毎回保存）
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        timestamped_file = f"bayesian_opt_v12_best_{timestamp}.json"
        # 安定ファイル名（上書き用）とロード別の別名を同時保存
        stable_latest = f"bayesian_opt_v12_best_latest_{load_level}.json"
        stable_v11 = f"best_params_v11_{load_level}.json"
        
        # ローカルに保存（ルートディレクトリ）
        with open(timestamped_file, 'w') as f:
            json.dump(payload, f)
        with open(stable_latest, 'w') as f:
            json.dump(payload, f)
        with open(stable_v11, 'w') as f:
            json.dump(payload, f)
        
        # outputs/ にもコピー保存（バックアップ）
        os.makedirs('outputs', exist_ok=True)
        outputs_timestamped = os.path.join('outputs', timestamped_file)
        outputs_latest = os.path.join('outputs', f"bayesian_opt_v12_best_latest_{load_level}.json")
        with open(outputs_timestamped, 'w') as f:
            json.dump(payload, f)
        with open(outputs_latest, 'w') as f:
            json.dump(payload, f)
        
        return stable_latest
    except Exception:
        return None

if __name__ == "__main__":
    mp.freeze_support()
    
    import argparse
    parser = argparse.ArgumentParser(description='Bayesian Optimization for CAV Weaving v11')
    parser.add_argument('--n_trials', type=int, default=100, help='最適化試行回数（デフォルト: 100）')
    parser.add_argument('--n_sims', type=int, default=None, help='試行あたりのシミュレーション回数（デフォルト: 5）')
    parser.add_argument('--n_parallel_sims', type=int, default=None, help='1試行内のシミュレーション並列数（デフォルト: 5、未指定時はn_simsと同じ）')
    parser.add_argument('--load', type=str, default='high', choices=['low', 'medium', 'high', 'congestion'],
                       help='負荷レベル（デフォルト: high）')
    parser.add_argument('--resume', type=str, nargs='+', default=None,
                       help='既存のJSONログファイルから再開（複数ファイル対応）')
    args = parser.parse_args()
    
    n_trials = args.n_trials
    load_level = args.load  # 負荷レベルの取得
    
    # シミュレーション回数と並列度の設定
    # 両方未指定: デフォルト値（5, 5）
    # 片方のみ指定: もう片方も同じ値にする
    # 両方指定: 指定された値を使用
    if args.n_sims is None and args.n_parallel_sims is None:
        # 両方未指定: デフォルト値を使用
        n_sims_per_trial = 5
        n_parallel_sims = 5
    elif args.n_sims is not None and args.n_parallel_sims is None:
        # n_simsのみ指定: 並列度も同じ値にする
        n_sims_per_trial = args.n_sims
        n_parallel_sims = args.n_sims
    elif args.n_sims is None and args.n_parallel_sims is not None:
        # n_parallel_simsのみ指定: シミュレーション回数も同じ値にする
        n_sims_per_trial = args.n_parallel_sims
        n_parallel_sims = args.n_parallel_sims
    else:
        # 両方指定: 指定された値を使用
        n_sims_per_trial = args.n_sims
        n_parallel_sims = args.n_parallel_sims
    
    # 並列設定の確認
    # ============================================================================
    # ★★★ Windows環境での安定性のため、Optuna並列化は完全に無効化 ★★★
    # ============================================================================
    # 試行内のシミュレーション並列化 (multiprocessing.Pool) のみを使用します。
    # これにより、mainの実行と同様の安定性を確保します。
    # ============================================================================
    cpu_count = mp.cpu_count()
    mem_gb = psutil.virtual_memory().available / (1024**3) if psutil else 0

    print("[並列設定] 安定性優先モード")
    print(f"  CPU: {cpu_count} 論理コア")
    if psutil:
        print(f"  メモリ: {mem_gb:.1f} GB 利用可能")
    print("  試行: 直列実行")
    print(f"  試行内シミュレーション回数: {n_sims_per_trial} 回（中央値集約用）")
    print(f"  試行内並列度: {n_parallel_sims} シミュレーション同時実行")
    
    study_name = f"weaving_v11_final_{load_level}"
    # SQLiteをローカルディレクトリに保存（ネットワークドライブではI/Oエラーが発生するため）
    local_db_dir = os.path.join(os.environ.get('TEMP', 'C:\\temp'), 'weaving_opt')
    os.makedirs(local_db_dir, exist_ok=True)
    db_path = os.path.join(local_db_dir, f"weaving_v11_opt_{load_level}.db")
    storage_url = f"sqlite:///{db_path}"
    
    # 安全終了のためのシグナルハンドリング
    # 中断時に現在のベスト試行をスナップショット保存してから終了する
    study = None  # 後で create_study 後に代入される想定
    def signal_handler(signum, frame):
        print("\n[中断] スナップショットを保存して終了します...")
        try:
            if study is not None:
                saved = _write_best_snapshot(study, timestamp, load_level)
                if saved:
                    print(f"[中断] スナップショット保存: {saved}")
        except Exception as e:
            print(f"[中断] スナップショット保存に失敗: {e}")
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"bayesian_opt_v11_log_{timestamp}.txt"
    json_log = f"bayesian_opt_v11_trials_{timestamp}.json"
    
    with TeeLogger(log_filename):
        print("="*80)
        print("ベイズ最適化 v12 (Exit最優先 + 安全性確保; AEBペナルティなし)")
        print(f"ターゲット: {load_level.upper()}負荷, queueモード, 衝突=0 (ハード制約)")
        print(f"効率正規化 (補助): 速度[{SPEED_NORM_MIN},{SPEED_NORM_MAX}] m/s, 時間[{TIME_NORM_MIN},{TIME_NORM_MAX}] s")
        print(f"並列設定: {n_sims_per_trial}回シミュレーション/試行, {n_parallel_sims}並列実行")
        print("="*80)
        
        study = optuna.create_study(
            study_name=study_name,
            storage=storage_url,
            direction="maximize",
            load_if_exists=True,
            sampler=TPESampler(seed=42, multivariate=True, n_startup_trials=20, warn_independent_sampling=False)
        )
        
        # --resume オプションで指定されたJSONファイルから試行を復元
        if args.resume:
            print("\n[復元開始] JSONファイルから過去の試行を復元中...")
            total_loaded = 0
            total_skipped = 0
            for json_file in args.resume:
                if os.path.exists(json_file):
                    print(f"  ファイル: {json_file}")
                    try:
                        with open(json_file, 'r', encoding='utf-8') as f:
                            for line in f:
                                try:
                                    trial_data = json.loads(line.strip())
                                    if trial_data:
                                        # Optunaの内部API を使って試行を追加
                                        trial_id = study._storage.create_new_trial_id(study._study_id)
                                        study._storage.set_trial_state(trial_id, optuna.trial.TrialState.COMPLETE)
                                        for param_name, param_value in trial_data['params'].items():
                                            study._storage.set_trial_param(trial_id, param_name, param_value,
                                                                         optuna.distributions.FloatDistribution(PARAM_RANGES[param_name][0], PARAM_RANGES[param_name][1]))
                                        study._storage.set_trial_value(trial_id, trial_data['value'])
                                        total_loaded += 1
                                except Exception as e:
                                    total_skipped += 1
                    except Exception as e:
                        print(f"    エラー: {e}")
                else:
                    print(f"  ファイルが見つかりません: {json_file}")
            print(f"  復元完了: {total_loaded}試行を読み込み、{total_skipped}試行をスキップ\n")
            # 復元後、新しいJSONログファイルに既存データをコピー
            if args.resume and os.path.exists(args.resume[0]):
                with open(args.resume[0], 'r') as src:
                    with open(json_log, 'a') as dst:
                        dst.writelines(src)
        else:
            print("\n[ランダムサンプリング開始] ウォームスタートは無効（前回パラメータでデッドロック発生）")
        
        # JSONログ保存と範囲適応チェックのコールバック
        def save_callback(study, trial):
            """各試行後に実行されるコールバック"""
            trial_data = {
                'number': trial.number,
                'params': trial.params,
                'value': trial.value,
                'datetime': datetime.now().isoformat()
            }
            with open(json_log, 'a') as f:
                f.write(json.dumps(trial_data) + '\n')
            
            # ADAPTATION_CHECK_INTERVAL試行ごとに範囲適応をチェック
            if RANGE_ADAPTATION_ENABLED and (trial.number + 1) % ADAPTATION_CHECK_INTERVAL == 0:
                print(f"\n[範囲適応] 試行 {trial.number} 後にチェック中...")
                for param_name in PARAM_RANGES.keys():
                    adapted = check_and_adapt_ranges(study, param_name)
                    if adapted:
                        print(f"  {param_name}: [{adapted[0]:.3f}, {adapted[1]:.3f}] に適応")
                print("")
            # ベストスナップショットを毎試行後に保存（安定ファイルを上書き）
            try:
                saved = _write_best_snapshot(study, timestamp, load_level)
                if saved:
                    print(f"[スナップショット] 更新: {saved}")
            except Exception:
                pass
        
        print(f"\n最適化ループを開始... (DB: {storage_url})")
        print(f"ターゲット: {n_trials} 試行 (Exit最優先; 動的範囲有効)")
        print(f"並列度: 試行は直列実行, {n_sims_per_trial}回シミュレーション × {n_parallel_sims}並列")
        est_time_per_trial = 300 * n_sims_per_trial / n_parallel_sims / 60
        print(f"実行時間目安: 1試行あたり300s × {n_sims_per_trial}回 ÷ {n_parallel_sims}並列 = 約{est_time_per_trial:.1f}分/試行\n")
        
        # Pool を最適化ループ外で1回だけ作成（効率化）
        print(f"[Pool作成] {n_parallel_sims}並列ワーカープールを初期化中...")
        optimization_pool = mp.Pool(n_parallel_sims)
        print(f"[Pool作成] 完了。全{n_trials}試行でPoolを再利用します。\n")
        
        try:
            # 試行内のシミュレーション並列化 (multiprocessing.Pool) のみ使用
            study.optimize(lambda t: objective(t, load_level, n_sims_per_trial, optimization_pool), n_trials=n_trials, callbacks=[save_callback])
        except KeyboardInterrupt:
            print("\n[中断] 進捗を保存中...")
        except Exception as e:
            print(f"[エラー] 最適化ループが失敗: {e}")
        finally:
            # Pool のクリーンアップ
            print("\n[Pool終了] ワーカープールをクローズ中...")
            optimization_pool.close()
            optimization_pool.join()
            print("[Pool終了] 完了。")
        # 念のため最終スナップショットを保存
        try:
            saved = _write_best_snapshot(study, timestamp, load_level)
            if saved:
                print(f"\n[完了] 最終スナップショット保存: {saved}")
        except Exception:
            pass
            
        print("\n" + "="*80)
        print("最適化完了")
        print("="*80)
        print(f"ベスト試行: #{study.best_trial.number}")
        print(f"ベストスコア: {study.best_value:.2f}")
        print("\nベストパラメータ:")
        for k, v in study.best_params.items():
            print(f"  {k}: {v:.4f}")
        
        # Extract metrics from best trial
        best_attrs = study.best_trial.user_attrs
        print("\nBest Performance Metrics:")
        print(f"  Gini (Weave):     {best_attrs.get('gini_weave', 1.0):.4f}  # lower is better")
        print(f"  Exit Rate:        {best_attrs.get('exit_rate', 0.0):.4f}")
        print(f"  LC Rate:          {best_attrs.get('lc_rate', 0.0):.4f}")
        print(f"  Collision Count:  {best_attrs.get('collision_count', 0)}")
        print(f"  AEB Count:        {best_attrs.get('aeb_count', 0)}")
        print(f"  Avg Speed:        {best_attrs.get('avg_speed', 0.0):.2f} m/s")
        print(f"  Avg Travel Time:  {best_attrs.get('avg_travel_time', 0.0):.1f} s")
        print(f"  Efficiency Score: {best_attrs.get('efficiency_score', 0.0):.3f} (0..1)")
        
        # Compare with v2 baseline
        v2_lc = 0.896
        best_lc = best_attrs.get('lc_rate', 0.0)
        improvement = ((best_lc / v2_lc) - 1) * 100 if v2_lc > 0 else 0
        print("\nNo baseline comparison (efficiency objective). Review absolute metrics above.")
        
        # Range adaptation summary
        if RANGE_ADAPTATION_ENABLED:
            print("\n" + "="*80)
            print("RANGE ADAPTATION SUMMARY")
            print("="*80)
            for param_name, param_range in PARAM_RANGES.items():
                initial_range = f"[{param_range['initial_min']:.3f}, {param_range['initial_max']:.3f}]"
                final_range = f"[{param_range['min']:.3f}, {param_range['max']:.3f}]"
                expansions = param_range['expansions']
                status = "EXPANDED" if expansions > 0 else "UNCHANGED"
                print(f"  {param_name:25s}: {initial_range:20s} -> {final_range:20s} ({status}, {expansions} expansions)")
        
        # Save best params to JSON
        best_params_file = f"bayesian_opt_v12_best_{timestamp}.json"
        with open(best_params_file, 'w') as f:
            json.dump({
                'params': study.best_params,
                'metrics': best_attrs,
                'score': study.best_value,
                'objective': 'collisions=0 (hard); minimize Gini(weave) > maximize success; then minimize travel time (speed auxiliary); no AEB penalty',
                'normalization': {
                    'speed_min': SPEED_NORM_MIN,
                    'speed_max': SPEED_NORM_MAX,
                    'time_min': TIME_NORM_MIN,
                    'time_max': TIME_NORM_MAX,
                    'weights': {'speed': EFF_WEIGHT_SPEED, 'time': EFF_WEIGHT_TIME}
                },
                'objective_weights': {
                    'gini_weave': 1000.0,
                    'success': 500.0,
                    'time': 100.0
                },
                'range_adaptation_summary': {
                    param_name: {
                        'initial_range': [param_range['initial_min'], param_range['initial_max']],
                        'final_range': [param_range['min'], param_range['max']],
                        'expansions': param_range['expansions']
                    } for param_name, param_range in PARAM_RANGES.items()
                }
            }, f, indent=2)
        print(f"\nベストパラメータを保存: {best_params_file}")
