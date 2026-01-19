# -*- coding: utf-8 -*-
"""
weaving_v11/simulator.py (ver.11.0 / 2025-12-17)

統合ゾーンシミュレータ:
- IntegratedZoneSimulator: 準備区間と織り込み区間のメインシミュレーションクラス
- v11.0 緊急度ベースローリングホライズン制御のための完全書き換え
- Gurobi MIQP依存なし

主要機能:
- 車両スポーン管理 (Poisson到着 + queue/attemptモード)
- V2V協調LCインテントレジストリ
- CAV軌道共有バッファ (3秒ホライズン)
- レーン密度計算と空間分布分析
"""

from typing import List, Optional
import numpy as np
import pandas as pd
import sys
import gc

from .vehicle import (
    Vehicle, SafetyAnalyzer, IDMModel, ObservedDistributionModel
)
from .safety import SAFETY  # v22.1 FIX: Import SAFETY constant
from .parameters import IntegratedZoneParameters
from .controllers import IntegratedZoneController
from .utils import SCRIPT_VERSION, get_next_adjacent_lane
from .coordinate_transform import LANE_OFFSETS
# v27.0: Unified Apollo Safety Manager for mid-LC safety checks
from .apollo_safety import get_safety_manager

# Debug output flag
ENABLE_DEBUG_OUTPUT = True  # v14.0: Enable for spawn velocity matching debugging


class IntegratedZoneSimulator:
    """
    v11.0 Integrated Zone Simulator

    Main simulation class managing vehicle spawning, control updates, and state evolution.
    Uses IntegratedZoneController for unified control logic.
    """

    def __init__(self,
                 params: IntegratedZoneParameters,
                 load_level: str = 'medium',
                 mode: str = 'l2'):
        """
        Initialize simulator

        Args:
            params: System parameters
            load_level: Load level ("low", "medium", "high", "congestion")
            mode: Control mode ("l2", "l1", "idm", "observed")
        """
        self.params = params
        self.load_level = load_level
        self.mode = mode

        # Vehicle lists
        self.vehicles: List[Vehicle] = []
        self.completed_vehicles: List[Vehicle] = []
        self.vehicle_id_counter = 0

        # Statistics
        self.stats = {
            "left": {
                "total_vehicles": 0,
                "lc_needed": 0,
                "lc_scheduled": 0,
                "lc_completed": 0,
                "reached_target": 0,
                "exited_vehicles": 0,
                "lc_started_at_exit": 0
            },
            "right": {
                "total_vehicles": 0,
                "lc_needed": 0,
                "lc_scheduled": 0,
                "lc_completed": 0,
                "reached_target": 0,
                "exited_vehicles": 0,
                "lc_started_at_exit": 0
            }
        }

        # Lane change history
        self.lc_history = []

        # Cell usage history (for compatibility)
        self.cell_usage_history = {c: [] for c in range(params.num_cells_total)}
        self.time_window = 15.0

        # History arrays for spatial distribution analysis
        self.history_prep_left = np.zeros(params.num_cells_prep)
        self.history_prep_right = np.zeros(params.num_cells_prep)
        self.history_weave = np.zeros(params.num_cells_weave)

        # Observed distribution model (for 'observed' mode)
        if self.mode == 'observed':
            self.obs_dist = ObservedDistributionModel()

        # IDM model (for warmup and 'idm' mode)
        T_idm_val = getattr(params, 'tau', getattr(params, 'T', getattr(params, 'T_idm', 1.5)))
        self.idm = IDMModel(v0=params.v_max, T=T_idm_val, s0=params.S_min)

        # Safety analyzer
        self.safety_analyzer = SafetyAnalyzer(
            collision_threshold=0.1,
            near_miss_threshold=2.0,
            critical_ttc_threshold=1.5
        )

        # Alias metrics for external consumers (main.py JSON stats)
        self.collision_count = 0

        # ========================================================================
        # v11.0 NEW: Initialize Integrated Controller
        # ========================================================================
        # IntegratedZoneController handles both Level 1 (Urgency) and Level 2 (QP)
        # No separate prep/weave controllers needed
        self.controller = IntegratedZoneController(params, self)
        # ========================================================================
        
        # ========================================================================
        # v22.0 NEW: V2V Cooperative LC Intent Registry
        # ========================================================================
        # Tracks active LC intents for V2V coordination between CAVs.
        # When a CAV intends to LC, it broadcasts intent and nearby CAVs adjust.
        # Format: {vehicle_id: LCIntent object or dict}
        # ========================================================================
        self.lc_intent_registry = {}  # Active LC intents
        self.v2v_gap_reservations = {}  # {target_lane: [(vehicle_id, x_pos, priority), ...]}
        # ========================================================================
        
        # ========================================================================
        # v23.0 NEW: Apollo CAV-CAV Trajectory Sharing Buffer
        # ========================================================================
        # Each CAV shares its planned trajectory for the next 3 seconds.
        # Format: {vehicle_id: {
        #     't_base': float,  # Base timestamp
        #     'x': np.array,    # x positions at t_base + [0, 0.1, ..., 3.0]
        #     'd': np.array,    # d (lateral) positions
        #     'v': np.array,    # velocities
        #     'lane_changing': bool,
        #     'target_lane': str or None
        # }}
        # ========================================================================
        self.cav_trajectories = {}
        self.trajectory_horizon = 3.0  # seconds
        self.trajectory_dt = 0.1  # 10Hz sampling
        # ========================================================================

        # ========================================================================
        # v28.3 NEW: Debug log aggregation for anomaly tracing
        # ========================================================================
        # Counters for anomaly logs and LC completions; per-vehicle breakdown
        self.log_counters = {
            'v28_3_lateral_anomaly': 0,
            'v28_3_progress_anomaly': 0,
            'guardian_lateral_anomaly': 0,
            'lc_completed': 0
        }
        self._per_vehicle_log = {
            'v28_3_lateral_anomaly': {},
            'v28_3_progress_anomaly': {},
            'guardian_lateral_anomaly': {},
            'lc_completed': {}
        }
        # Store a few recent anomaly samples for quick inspection
        self._recent_events = []  # list of dicts with keys: type, vid, t, details
        
        # ------------------------------------------------------------------------
        # v27.16: Track AEB trigger count for JSON statistics
        # ------------------------------------------------------------------------
        self.aeb_trigger_count = 0

        # v27.17: Capture AEB event snapshots for post-mortem analysis
        self._aeb_event_log = []  # list of dicts: {t, vid, front_id, gap, rel_v, lateral, ttc, lane_from, lane_to, progress}
        
        # v27.18: Track which vehicles have already triggered AEB (per-vehicle)
        self._aeb_triggered_vehicles = set()  # Set of vehicle IDs that have triggered AEB

        # ------------------------------------------------------------------------
        # v29.1: Spawn scheduler (Poisson with queue mode)
        # ------------------------------------------------------------------------
        self._spawn_lanes = ["left", "lcenter", "rcenter", "right"]
        # Per-lane spawn rates (lane groups split equally)
        self._lane_spawn_rate = {
            "left":  max(0.0, float(getattr(self.params, 'spawn_rate_left', 0.0))) / 2.0,
            "lcenter": max(0.0, float(getattr(self.params, 'spawn_rate_left', 0.0))) / 2.0,
            "rcenter": max(0.0, float(getattr(self.params, 'spawn_rate_right', 0.0))) / 2.0,
            "right": max(0.0, float(getattr(self.params, 'spawn_rate_right', 0.0))) / 2.0,
        }
        # Next arrival absolute time per lane (initialized at t=0 draw)
        self._next_arrival_per_lane = {ln: (np.random.exponential(1.0 / r) if r > 0 else float('inf'))
                                       for ln, r in self._lane_spawn_rate.items()}
        # Pending demand (arrivals waiting to be inserted safely)
        self._pending_spawns = {ln: 0 for ln in self._spawn_lanes}
        # Last accepted spawn time per lane (for headway_min enforcement if desired)
        self._last_spawn_time_per_lane = {ln: -1e12 for ln in self._spawn_lanes}

    def _capture_front_snapshot(self, v, t: float):
        """Capture nearest-front context for AEB/collision logging.

        Returns a dict with front vehicle info (gap, rel_v, lateral, ttc) or None fields if not found.
        """
        front = None
        min_gap = float('inf')
        for u in self.vehicles:
            if u.id == v.id or u.exited:
                continue
            if u.x <= v.x:
                continue
            gap = u.x - v.x - self.params.L_vehicle
            if gap < min_gap:
                min_gap = gap
                front = u

        if front is not None:
            lateral = abs(getattr(v, 'd', 0.0) - getattr(front, 'd', 0.0))
            rel_v = v.v - front.v
            ttc = SafetyAnalyzer.compute_ttc(v.x, front.x, v.v, front.v, self.params.L_vehicle)
            return {
                't': t,
                'vid': v.id,
                'front_id': front.id,
                'gap': min_gap,
                'rel_v': rel_v,
                'lateral': lateral,
                'ttc': ttc,
                'lane_from': getattr(v, 'lane_from', None),
                'lane_to': getattr(v, 'lane_to', None),
                'progress': getattr(v, 'lc_progress', None),
                'changing': getattr(v, 'changing_lane', False)
            }
        else:
            return {
                't': t,
                'vid': v.id,
                'front_id': None,
                'gap': None,
                'rel_v': None,
                'lateral': None,
                'ttc': None,
                'lane_from': getattr(v, 'lane_from', None),
                'lane_to': getattr(v, 'lane_to', None),
                'progress': getattr(v, 'lc_progress', None),
                'changing': getattr(v, 'changing_lane', False)
            }

    def run(self, t_max: Optional[float] = None, until_event: Optional[str] = None):
        """
        Run simulation

        Args:
            t_max: Maximum simulation time [s]
            until_event: Event-based termination ('aeb', 'collision', 'any')
                - 'aeb': Stop when AEB is triggered
                - 'collision': Stop when a collision occurs
                - 'any': Stop when either event occurs
        """
        if t_max is None:
            t_max = self.params.debug_tmax if self.params.debug_mode else 600.0
        
        # Initialize event tracking for --until mode
        self._until_event = until_event
        self._aeb_triggered = False  # Global flag for --until=aeb termination
        self._collision_triggered = False
        self._event_stop_reason = None
        self._prev_collision_count = 0  # Track collision count changes

        print(f"\n{'='*80}")
        print(f"[Simulation Start - {SCRIPT_VERSION} - Load Level: {self.load_level.upper()}]")
        if until_event:
            event_desc = {'aeb': 'AEB triggered', 'collision': 'Collision occurs', 'any': 'AEB or Collision'}
            print(f"  🎯 EVENT MODE: Running until {event_desc.get(until_event, until_event)}")
        if self.params.debug_mode:
            print(f"  🔧 DEBUG MODE: t_max={t_max}s (normal: 600s)")
        print(f"  Total spawn_rate: {self.params.spawn_rate:.2f} veh/s")
        print(f"  Prep zone: {self.params.prep_zone_length}m, Weave zone: {self.params.weave_zone_length}m")
        print(f"  Density (H/M): {self.params.density_threshold_high:.2f} / {self.params.density_threshold_medium:.2f}")
        print(f"  LC Density Threshold: {self.params.delta_rho_min:.3f} veh/m")
        print(f"  Headway Stats: Mean={self.params.headway_mean}s, Min={self.params.headway_min}s (Stochastic)")
        print(f"  Control Mode: {self.mode.upper()}")
        print(f"  Physics Timestep: {self.params.dt_sim*1000:.1f}ms ({1.0/self.params.dt_sim:.0f}Hz)")
        print(f"  Control Cycle: {self.params.dt_control*1000:.1f}ms ({1.0/self.params.dt_control:.0f}Hz)")
        print(f"{'='*80}\n")

        t = 0.0
        dt_sim = self.params.dt_sim
        dt_control = self.params.dt_control
        control_interval = self.params.control_interval
        last_control_time = -control_interval
        last_spawn_time = 0.0

        # ====================================================================
        # v12.3 Modified: Explicit sub-stepping for physics precision
        # ====================================================================
        # Calculate number of physics steps per control cycle
        # steps_per_control = dt_control / dt_sim = 0.1 / 0.01 = 10
        steps_per_control = int(dt_control / dt_sim + 0.5)
        if abs(steps_per_control * dt_sim - dt_control) > 1e-6:
            print(f"[WARNING] dt_control ({dt_control:.4f}s) is not exact multiple of dt_sim ({dt_sim:.4f}s)")
            print(f"  Adjusted: {steps_per_control} steps x {dt_sim:.4f}s = {steps_per_control * dt_sim:.4f}s")
        # ====================================================================

        while t < t_max:
            # Spawn scheduling
            if getattr(self.params, 'spawn_mode', 'attempt') == 'queue':
                # 1) Generate Poisson demand per lane (insertion queue)
                for ln in self._spawn_lanes:
                    rate = self._lane_spawn_rate.get(ln, 0.0)
                    while t >= self._next_arrival_per_lane.get(ln, float('inf')):
                        # One demand arrival for this lane
                        self._pending_spawns[ln] = self._pending_spawns.get(ln, 0) + 1
                        # Schedule next arrival for this lane
                        if rate > 0:
                            self._next_arrival_per_lane[ln] += np.random.exponential(1.0 / rate)
                        else:
                            self._next_arrival_per_lane[ln] = float('inf')

                # 2) Try to insert at most one pending vehicle per lane per control step
                for ln in self._spawn_lanes:
                    if self._pending_spawns.get(ln, 0) <= 0:
                        continue
                    # Optional: enforce minimal headway between accepted spawns on the same lane
                    if (t - self._last_spawn_time_per_lane.get(ln, -1e12)) < self.params.headway_min:
                        continue
                    if self._attempt_spawn_on_lane(t, ln):
                        self._pending_spawns[ln] -= 1
                        self._last_spawn_time_per_lane[ln] = t
            else:
                # Legacy attempt mode: spawn by global rate and immediate safety checks
                if t - last_spawn_time >= 1.0 / self.params.spawn_rate:
                    self._spawn_vehicle(t)
                    last_spawn_time = t

            # Control update at control_interval
            if t - last_control_time >= control_interval:
                self._control_update(t)
                last_control_time = t

            # ====================================================================
            # v12.3 Modified: Sub-step loop for physics accuracy
            # ====================================================================
            # Apply same control input over multiple physics steps
            # This ensures control signal is maintained during the finer physics simulation
            for _ in range(steps_per_control):
                # Update vehicle states with fine-grained physics
                self._update_vehicles(dt_sim, t)

                # Safety evaluation at each physics step (improved collision detection)
                if hasattr(self.safety_analyzer, 'set_current_time'):
                    self.safety_analyzer.set_current_time(t)
                self._evaluate_safety()
                
                # ================================================================
                # v16.4: Event-based termination (--until mode)
                # ================================================================
                if self._until_event:
                    # Check for NEW AEB activations (per-vehicle tracking)
                    for v in self.vehicles:
                        if getattr(v, 'aeb_active', False) and v.id not in self._aeb_triggered_vehicles:
                            # Mark this vehicle as having triggered AEB
                            self._aeb_triggered_vehicles.add(v.id)
                            self._aeb_triggered = True  # Global flag for termination
                            
                            # Increment AEB counter for stats
                            self.aeb_trigger_count += 1
                            
                            # Snapshot nearest-front context for diagnostics
                            snap = self._capture_front_snapshot(v, t)
                            self._aeb_event_log.append(snap)
                            
                            # Print detailed AEB event log
                            print(f"\n{'='*80}")
                            print(f"[AEB-EVENT #{self.aeb_trigger_count}] 🚨 t={t:.2f}s - Vehicle V{v.id}")
                            print(f"  Position: x={v.x:.1f}m, Lane: {v.lane}")
                            print(f"  Velocity: v={v.v:.1f}m/s, Accel: ax={v.ax:.2f}m/s²")
                            print(f"  Reason: {getattr(v, 'aeb_reason', 'N/A')}")
                            if snap['gap'] is not None:
                                print(f"  Front Context: V{snap['front_id']}, gap={snap['gap']:.2f}m, "
                                      f"rel_v={snap['rel_v']:.2f}m/s, lateral={snap['lateral']:.2f}m, ttc={snap['ttc']}")
                            else:
                                print(f"  Front Context: No front vehicle detected")
                            print(f"  LC Status: changing={snap['changing']}, progress={snap['progress']}")
                            print(f"{'='*80}")
                    
                    # Check for collision (look at metrics increment)
                    prev_collision_count = getattr(self, '_prev_collision_count', 0)
                    current_collision_count = self.safety_analyzer.metrics.collision_count
                    if current_collision_count > prev_collision_count:
                        self._collision_triggered = True
                        new_collisions = current_collision_count - prev_collision_count
                        print(f"\n{'='*80}")
                        print(f"[EVENT] 💥 COLLISION DETECTED at t={t:.2f}s ({new_collisions} new)")
                        if self.safety_analyzer.metrics.collision_details:
                            latest = self.safety_analyzer.metrics.collision_details[-1]
                            print(f"  Vehicles: V{latest['rear_vehicle_id']} -> V{latest['front_vehicle_id']}")
                            print(f"  Headway: {latest['headway']:.2f}m")
                        print(f"{'='*80}")
                    self._prev_collision_count = current_collision_count
                    
                    # Check termination condition
                    should_stop = False
                    if self._until_event == 'aeb' and self._aeb_triggered:
                        self._event_stop_reason = 'AEB triggered'
                        should_stop = True
                    elif self._until_event == 'collision' and self._collision_triggered:
                        self._event_stop_reason = 'Collision detected'
                        should_stop = True
                    elif self._until_event == 'any' and (self._aeb_triggered or self._collision_triggered):
                        self._event_stop_reason = 'AEB triggered' if self._aeb_triggered else 'Collision detected'
                        should_stop = True
                    
                    if should_stop:
                        print(f"\n[EVENT-STOP] Simulation stopped: {self._event_stop_reason} at t={t:.2f}s")
                        # Jump to finalization
                        self.safety_analyzer.finalize_metrics()
                        self._print_final_statistics()
                        self._print_event_summary(t)
                        return
                # ================================================================

                # Debug early stopping
                if self.params.debug_mode:
                    collision_count = self.safety_analyzer.metrics.collision_count
                    if collision_count >= self.params.debug_early_stop_collisions:
                        print(f"\n[DEBUG] Early stop: {collision_count} collisions at t={t:.1f}s")
                        break
                    if t >= self.params.debug_early_stop_time:
                        print(f"\n[DEBUG] Time limit reached: t={t:.1f}s")
                        break

                t += dt_sim
                
                # [HEARTBEAT & GC] Every 10.0s simulation time
                if abs(t % 10.0) < dt_sim * 0.5:
                    sys.stdout.flush()
                    
                    # [PERF] Get QP Stats
                    qp_stats = {}
                    try:
                        if hasattr(self.controller, 'frenet_controller'):
                            qp_stats = self.controller.frenet_controller.get_qp_stats()
                    except:
                        pass
                    
                    qp_msg = ""
                    if qp_stats:
                        total = qp_stats.get('total_attempts', 0)
                        relaxed = qp_stats.get('relaxed_trigger', 0)
                        fallback = qp_stats.get('fallback_trigger', 0)
                        avg_time = qp_stats.get('total_time_ms', 0) / max(1, total)
                        qp_msg = f" | QP: {total} calls, {relaxed} relaxed ({relaxed/max(1,total)*100:.1f}%), {fallback} fallback, {avg_time:.1f}ms/call"

                    if self.vehicles:
                        # v27.5: Add speed statistics for congestion detection
                        speeds = [v.v for v in self.vehicles]
                        avg_speed = sum(speeds) / len(speeds) if speeds else 0.0
                        min_speed = min(speeds) if speeds else 0.0
                        print(f"[HEARTBEAT] t={t:.1f}s | Vehicles: {len(self.vehicles)} | Exited: {len(self.completed_vehicles)} | Avg v={avg_speed:.1f}m/s | Min v={min_speed:.1f}m/s{qp_msg}", flush=True)
                    # Force garbage collection to prevent memory buildup in long runs
                    gc.collect()

            # Check for early exit after sub-steps
            if self.params.debug_mode and t >= self.params.debug_early_stop_time:
                break
            # ====================================================================

        # Finalize metrics
        self.safety_analyzer.finalize_metrics()
        self._print_final_statistics()
    
    def _print_event_summary(self, stop_time: float):
        """Print summary when simulation stops due to event"""
        print(f"\n{'='*80}")
        print(f"[EVENT MODE SUMMARY]")
        print(f"{'='*80}")
        print(f"  Stop Reason: {self._event_stop_reason}")
        print(f"  Simulation Time: {stop_time:.2f}s")
        print(f"  AEB Triggered: {'Yes' if self._aeb_triggered else 'No'}")
        print(f"  Collision Occurred: {'Yes' if self._collision_triggered else 'No'}")
        print(f"  Total Collisions: {self.safety_analyzer.metrics.collision_count}")
        print(f"  Total Vehicles: {len(self.vehicles)}")
        print(f"{'='*80}")

    def _log_inc(self, key: str, vid: int, t: float, **details):
        """Increment log counter and per-vehicle breakdown, capture a small sample."""
        self.log_counters[key] = self.log_counters.get(key, 0) + 1
        per = self._per_vehicle_log.setdefault(key, {})
        per[vid] = per.get(vid, 0) + 1
        if len(self._recent_events) < 20:
            self._recent_events.append({'type': key, 'vid': vid, 't': t, **details})

    def _spawn_vehicle(self, t: float):
        """
        Spawn a new vehicle

        Args:
            t: Current time [s]
        """
        # Lane selection (uniform distribution)
        lane = np.random.choice(["left", "lcenter", "rcenter", "right"])
        side = "left" if lane in ["left", "lcenter"] else "right"

        # Velocity parameters (congestion vs normal)
        if self.params.congestion_mode:
            v0_mean = self.params.v0_mean_congestion
            v0_std = self.params.v0_std_congestion
        else:
            v0_mean = self.params.v0_mean_normal
            v0_std = self.params.v0_std_normal

        # Side-specific spawn rate
        spawn_rate = (self.params.spawn_rate_left if side == "left"
                      else self.params.spawn_rate_right)

        # Exponential headway time (Poisson arrival)
        headway_time = np.random.exponential(1.0 / spawn_rate)
        headway_time = max(headway_time, self.params.headway_min)

        # Initial velocity
        v0 = np.clip(np.random.normal(v0_mean, v0_std), 5.0, 30.0)

        # ====================================================================
        # v13.6: Dynamic RSS Check Before Spawning (Root Cause 1 Fix)
        # ====================================================================
        # Problem (Root Cause 1): Spawn vehicle at x=-400m without checking if
        #          rear vehicle approaching at high speed. Gap can be 18m->0.92m
        #          in 5 seconds (V8->V7 collision evidence).
        #
        # Solution: Check relative approach speed before spawn. If rear vehicle
        #          approaching too fast, delay spawn or move spawn position forward.
        #          Apollo traffic_decider.cc uses pre-spawn RSS validation.
        #
        # Reference: Apollo traffic_decider.cc - CheckSpawnRSSCompliance()
        #           Validates: gap >= RSS(v_rear, v_spawn)
        #           RSS distance: d = (v^2 / (2a)) + v*T + s₀
        # ====================================================================
        x0 = -self.params.warmup_length
        spawn_adjusted = False
        
        # Find rear vehicle in same lane approaching spawn position
        lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]
        
        # Filter vehicles coming from behind (x < x0)
        approaching_vehicles = [u for u in lane_vehicles if u.x < x0 and u.v > 0]
        
        if approaching_vehicles:
            # Find closest approaching vehicle
            rear_vehicle = min(approaching_vehicles, key=lambda u: x0 - u.x)
            
            # Calculate RSS distance if rear vehicle approaching
            gap_initial = x0 - rear_vehicle.x  # Distance from rear vehicle to spawn point
            v_rear = rear_vehicle.v
            L_vehicle = 5.0  # Vehicle length [m]
            
            # Dynamic spawn velocity matching (enhanced)
            v_spawn = v0
            if rear_vehicle.x - (-self.params.warmup_length) < 100.0:
                v_spawn = min(v_spawn, rear_vehicle.v)
                
            # RSS computation: d_rss = v^2 / (2a) + v*T + s₀
            # where a = 6.0 m/s^2 (maximum deceleration, UN R157)
            #       T = 0.1s (reaction time, CAV)
            #       s₀ = 2.0m (minimum static spacing)
            a_brake = 6.0
            t_react = 0.1
            s0_min = 2.0
            v_rel = max(0.0, v_rear - v_spawn)
            rss_distance = (v_rel**2) / (2 * a_brake) + v_rel * t_react + s0_min
            required_gap = rss_distance + L_vehicle
            
            # Check if spawn is safe at current position
            if gap_initial < required_gap:
                # Unsafe to spawn here. Move spawn position backward (earlier)
                # Alternative: delay spawn until rear vehicle passes
                # Use backoff distance approach: x0_new = rear_vehicle.x - required_gap
                x0_new = rear_vehicle.x - required_gap
                
                if x0_new < -self.params.warmup_length - 100.0:
                    # Would spawn too far back, reject spawn entirely
                    return
                
                x0 = x0_new  # Adjust spawn position
                spawn_adjusted = True
                
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[SPAWN-RSS-CHECK] Adjusted spawn position for V{rear_vehicle.id}: "
                          f"gap_initial={gap_initial:.1f}m < RSS_req={required_gap:.1f}m -> "
                          f"x0_adjusted={x0:.1f}m (delay spawn)")
        
        # ====================================================================
        # v14.0: Enhanced Velocity Matching (Front + Rear)
        # ====================================================================
        # Match velocity with BOTH rear and front vehicles to prevent
        # excessive relative speeds that trigger cascade AEB
        # ====================================================================
        lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]
        rear_vehicles = [u for u in lane_vehicles if u.x < -self.params.warmup_length + 100.0]
        # v14.1: CRITICAL FIX - Remove 200m upper limit to find ALL front vehicles
        # Previous: u.x < x0 + 200.0 was too restrictive, missed vehicles beyond 200m
        front_vehicles = [u for u in lane_vehicles if u.x > x0]

        v_spawn = v0  # Default
        v_spawn_original = v0  # Track original for logging

        # 1. Match with rear vehicle (collision prevention)
        if rear_vehicles:
            leader = max(rear_vehicles, key=lambda u: u.x)
            if leader.x - x0 < 50.0:
                v_spawn = min(v_spawn, leader.v)
                if ENABLE_DEBUG_OUTPUT and not spawn_adjusted:
                    print(f"[SPAWN-REAR-MATCH] V{leader.id}: "
                          f"v_spawn={v_spawn:.1f}m/s (dist={leader.x - x0:.1f}m)")

        # 2. Match with front vehicle (prevent high relative speed -> cascade AEB)
        # v18.26: Also check vehicles that spawned just before in same position
        all_front_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited and u.x >= x0 - 5.0]
        if all_front_vehicles:
            # v14.1: Find nearest front vehicle (regardless of distance)
            nearest_front = min(all_front_vehicles, key=lambda u: abs(u.x - x0))
            dist_to_front = nearest_front.x - x0
            
            # v18.26: For vehicles behind or at same position, use absolute distance
            if dist_to_front <= 0:
                # Vehicle is at same position or slightly behind - very dangerous
                dist_to_front = max(1.0, abs(dist_to_front))  # At least 1m to prevent division issues

            # v18.25: Dynamic relative speed limit based on front vehicle speed
            # If front vehicle is slow (< 10 m/s), reduce allowed relative speed
            # This prevents AEB trigger when catching up to slow-moving vehicles
            BASE_MAX_RELATIVE_SPEED = 10.0  # [m/s] Apollo traffic_decider standard
            
            # Scale down max relative speed when front vehicle is slow
            # At front_v=0: max_rel=5m/s, at front_v>=10: max_rel=10m/s
            if nearest_front.v < 10.0:
                MAX_RELATIVE_SPEED = 5.0 + (nearest_front.v / 10.0) * 5.0
            else:
                MAX_RELATIVE_SPEED = BASE_MAX_RELATIVE_SPEED
            
            # v18.26: More aggressive limit when gap is small
            # gap < 20m: max_rel = 3.0m/s (very close)
            # gap < 30m: max_rel = 5.0m/s
            if dist_to_front < 20.0:
                MAX_RELATIVE_SPEED = min(MAX_RELATIVE_SPEED, 3.0)
            elif dist_to_front < 30.0:
                MAX_RELATIVE_SPEED = min(MAX_RELATIVE_SPEED, 5.0)
            
            # ====================================================================
            # v18.28: Critical Safety - Minimum Gap and TTC Check
            # ====================================================================
            # Problem: V6 spawned at gap=14.1m, v_rel=3.0m/s and immediately
            #          triggered AEB-LATCH at t=15.10s (just 1.5s after spawn)
            # Root cause: Gap shrinks rapidly when approaching front vehicle
            #             TTC = gap / v_rel = 14.1 / 3.0 = 4.7s (very short!)
            #
            # Solution: 
            # 1. Minimum gap enforcement: If gap < MIN_SAFE_GAP, match front speed
            # 2. TTC check: If approaching and TTC < MIN_TTC, reject spawn
            # ====================================================================
            MIN_SAFE_GAP = 20.0  # [m] v28.3: Reverted from 25.0 (v28.1 balance was better)
            MIN_TTC_SPAWN = 5.0  # [s] v28.0: Increased from 4.0 (more conservative)
            
            # Calculate potential v_rel with max allowed spawn velocity
            potential_v_spawn = nearest_front.v + MAX_RELATIVE_SPEED
            potential_v_rel = max(0.0, potential_v_spawn - nearest_front.v)
            
            # Check 1: Minimum gap enforcement
            if dist_to_front < 2.0:
                # v18.30: CRITICAL SAFETY - Abort spawn if gap is too small
                # 0.6m gap observed in high load -> physical risk
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[SPAWN-ABORT] Gap {dist_to_front:.1f}m too small (< 2.0m)")
                return

            if dist_to_front < MIN_SAFE_GAP:
                # Very small gap - match front vehicle speed exactly
                MAX_RELATIVE_SPEED = 0.0
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[SPAWN-MIN-GAP] gap={dist_to_front:.1f}m < {MIN_SAFE_GAP}m: "
                          f"forcing v_spawn = front_v ({nearest_front.v:.1f}m/s)")
            
            # Check 2: TTC check for approaching scenarios
            elif potential_v_rel > 0.1:  # Approaching front vehicle
                ttc_spawn = dist_to_front / potential_v_rel
                if ttc_spawn < MIN_TTC_SPAWN:
                    # Reduce v_rel to achieve minimum TTC
                    # TTC = gap / v_rel >= MIN_TTC => v_rel <= gap / MIN_TTC
                    safe_v_rel = dist_to_front / MIN_TTC_SPAWN
                    MAX_RELATIVE_SPEED = max(0.0, min(MAX_RELATIVE_SPEED, safe_v_rel))
                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[SPAWN-TTC-CHECK] TTC={ttc_spawn:.1f}s < {MIN_TTC_SPAWN}s: "
                              f"reducing max_v_rel to {MAX_RELATIVE_SPEED:.1f}m/s")
            
            max_safe_spawn_v = nearest_front.v + MAX_RELATIVE_SPEED

            if v_spawn > max_safe_spawn_v:
                v_spawn_before = v_spawn
                v_spawn = max_safe_spawn_v
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[SPAWN-FRONT-MATCH] V{nearest_front.id} at x={nearest_front.x:.1f}m: "
                          f"v_spawn: {v_spawn_before:.1f} -> {v_spawn:.1f}m/s "
                          f"(front_v={nearest_front.v:.1f}m/s, dist={dist_to_front:.1f}m, "
                          f"v_rel={v_spawn - nearest_front.v:.1f}m/s <= {MAX_RELATIVE_SPEED:.1f}m/s)")
            else:
                if ENABLE_DEBUG_OUTPUT and len(front_vehicles) > 0:
                    print(f"[SPAWN-FRONT-OK] V{nearest_front.id} at x={nearest_front.x:.1f}m: "
                          f"v_spawn={v_spawn:.1f}m/s OK (front_v={nearest_front.v:.1f}m/s, "
                          f"dist={dist_to_front:.1f}m, v_rel={v_spawn - nearest_front.v:.1f}m/s)")

        # Apply matched velocity
        v0 = v_spawn

        # v14.1: Log spawn summary
        if ENABLE_DEBUG_OUTPUT:
            if v_spawn != v_spawn_original:
                print(f"[SPAWN-SUMMARY] lane={lane}, x0={x0:.1f}m: "
                      f"v_original={v_spawn_original:.1f} -> v_final={v_spawn:.1f}m/s "
                      f"({len(front_vehicles)} front, {len(rear_vehicles)} rear)")
        # ====================================================================

        # Check headway constraint
        side_vehicles = [v for v in self.vehicles
                         if v.side == side and v.lane == lane and not v.exited]
        if side_vehicles:
            last_spawn_time = max(v.spawn_time for v in side_vehicles)
            if t - last_spawn_time < headway_time:
                return  # Skip spawning

        # Destination area (50/50 split)
        destination_area = np.random.choice(["LEFT", "RIGHT"], p=[0.5, 0.5])

        in_dest_area = (
            (destination_area == "LEFT" and lane in ["left", "lcenter"]) or
            (destination_area == "RIGHT" and lane in ["rcenter", "right"])
        )

        needs_initial_lc = not in_dest_area
        can_density_adjust = in_dest_area

        # IDM parameters
        v_desired = np.clip(np.random.normal(25.0, 5.0), 15.0, 35.0)
        T_idm = np.clip(np.random.normal(1.5, 0.3), 1.0, 2.5)

        # --- modified (v11.10 / 2025-12-19): HDV determination ---
        # Randomly assign vehicle type based on hdv_ratio parameter
        # HDVs cannot use Apollo control and rely on IDM only
        is_hdv = np.random.random() < self.params.hdv_ratio
        # ---

        # Create vehicle
        # Note: Pylance may show false positives for dataclass keyword arguments
        y0 = LANE_OFFSETS.get(lane, 0.0)
        vehicle = Vehicle(
            vehicle_id=self.vehicle_id_counter,
            x=x0,
            y=y0,
            v=v0,
            lane=lane,
            spawn_time=t,
            destination_area=destination_area,
            in_destination_area=in_dest_area,
            needs_initial_lc=needs_initial_lc,
            can_density_adjust=can_density_adjust,
            v_desired=v_desired,
            T_idm=T_idm,
            is_hdv=is_hdv,
            params=self.params
        )

        # Set side attribute after instantiation
        vehicle.side = side

        # Initialize Frenet coordinates based on spawn lane
        vehicle.d = y0  # Already set from LANE_OFFSETS

        # ====================================================================
        # v19.5: CRITICAL FIX - Initialize target_lane_prep/weave at Spawn
        # ====================================================================
        # BUG: target_lane_prep and target_lane_weave were initialized to None
        #      with comment "will be set by controller logic", but controller
        #      expects them to already be set! This caused 0/16 LC scheduled.
        #
        # FIX: Set target lanes during spawn based on destination_area and lane
        #      - Vehicles needing LC get immediate target lane assignment
        #      - Controller can now properly check target_lane validity
        #
        # Logic:
        #      LEFT spawn -> RIGHT dest: left/lcenter -> rcenter/right (1-2 LCs)
        #      RIGHT spawn -> LEFT dest: right/rcenter -> lcenter/left (1-2 LCs)
        # ====================================================================
        if needs_initial_lc:
            # Determine first target lane based on current position and destination
            if destination_area == "RIGHT":
                # Moving from left side to right side
                if lane == "left":
                    vehicle.target_lane_prep = "lcenter"  # First LC: left -> lcenter
                    vehicle.target_lane_weave = "rcenter"  # Second LC: lcenter -> rcenter (or -> right)
                elif lane == "lcenter":
                    vehicle.target_lane_prep = "rcenter"  # First LC: lcenter -> rcenter
                    vehicle.target_lane_weave = "right"   # Second LC: rcenter -> right
                else:
                    # Shouldn't happen (left/lcenter are the only lanes needing RIGHT LC)
                    vehicle.target_lane_prep = "rcenter"
                    vehicle.target_lane_weave = "right"
            else:  # destination_area == "LEFT"
                # Moving from right side to left side
                if lane == "right":
                    vehicle.target_lane_prep = "rcenter"  # First LC: right -> rcenter
                    vehicle.target_lane_weave = "lcenter"  # Second LC: rcenter -> lcenter (or -> left)
                elif lane == "rcenter":
                    vehicle.target_lane_prep = "lcenter"  # First LC: rcenter -> lcenter
                    vehicle.target_lane_weave = "left"    # Second LC: lcenter -> left
                else:
                    # Shouldn't happen
                    vehicle.target_lane_prep = "lcenter"
                    vehicle.target_lane_weave = "left"

            # Set initial target_lane to first step (prep zone target)
            vehicle.target_lane = vehicle.target_lane_prep
        else:
            # Already in destination area: no LC needed
            vehicle.target_lane_prep = None
            vehicle.target_lane_weave = None
            vehicle.target_lane = None
        # ====================================================================

        # ====================================================================
        # v11.11: Poisson + Safety Spawn (Traffic Engineering)
        # ====================================================================
        # Maintain Poisson arrival process (random timing) while ensuring
        # physical safety constraints (IDM equilibrium spacing + margin)
        # - Try spawning at requested position (x0)
        # - If conflict: shift backward in 15m increments (max 3 attempts)
        # - If all attempts fail: skip this spawn (capacity limit reached)
        # Reference: Traffic flow theory - safe spacing = s0 + v*T
        # ====================================================================
        lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]

        spawn_x = x0
        safe_to_spawn = False
        retry_count = 0
        MAX_RETRIES = 3
        RETRY_OFFSET = 15.0  # meters (shift backward per retry)

        while retry_count < MAX_RETRIES:
            conflict = False

            for u in lane_vehicles:
                # Required safe spacing: IDM equilibrium distance
                # s_safe = s0 + v * T (T=1.5s for safety margin)
                # Use existing vehicle's velocity for calculation
                safe_dist = self.params.S_min + u.v * 1.5

                # Check if spawn position conflicts with existing vehicle
                if abs(u.x - spawn_x) < safe_dist:
                    conflict = True
                    break

            if not conflict:
                # Safe position found
                safe_to_spawn = True
                break

            # Conflict detected: shift backward and retry
            spawn_x -= RETRY_OFFSET
            retry_count += 1

        if safe_to_spawn:
            # Update spawn position if adjusted
            vehicle.x = spawn_x

            if spawn_x != x0 and ENABLE_DEBUG_OUTPUT and self.params.enable_spawn_debug:
                print(f"[SPAWN ADJUSTED] V{vehicle.id} lane={lane}: "
                      f"Poisson position {x0:.1f}m -> Safe position {spawn_x:.1f}m "
                      f"(shifted {x0-spawn_x:.1f}m after {retry_count} retries)")

            self.vehicles.append(vehicle)

            # v14.1: Comprehensive spawn logging
            if ENABLE_DEBUG_OUTPUT:
                vehicle_type = "HDV" if is_hdv else "CAV"
                print(f"[SPAWN-SUCCESS] V{vehicle.id} ({vehicle_type}): "
                      f"lane={lane}, x={spawn_x:.1f}m, v={v0:.1f}m/s, "
                      f"dest={destination_area}, t={t:.1f}s")
                # Show front vehicle info for context
                if front_vehicles:
                    nearest = min(front_vehicles, key=lambda u: u.x - spawn_x)
                    gap_to_front = nearest.x - spawn_x
                    v_rel_front = v0 - nearest.v
                    print(f"  -> Front: V{nearest.id} at {nearest.x:.1f}m, "
                          f"gap={gap_to_front:.1f}m, v={nearest.v:.1f}m/s, "
                          f"v_rel={v_rel_front:.1f}m/s")
        else:
            # All retry attempts failed - traffic capacity limit reached
            # Skip this spawn (vehicle arrival delayed to next cycle)
            if ENABLE_DEBUG_OUTPUT and self.params.enable_spawn_debug:
                print(f"[SPAWN SKIPPED] lane={lane}: No safe position found "
                      f"within {MAX_RETRIES} retries (capacity limit)")
            return  # Exit without spawning
        # ====================================================================
        self.vehicle_id_counter += 1
        self.stats[side]["total_vehicles"] += 1

        if needs_initial_lc:
            self.stats[side]["lc_needed"] += 1

    def _attempt_spawn_on_lane(self, t: float, lane: str) -> bool:
        """Attempt to spawn a vehicle on the specified lane at time t.

        This follows the same safety-adjusted logic as _spawn_vehicle but
        without drawing a new headway or randomly choosing a lane.
        Returns True if a vehicle was spawned, else False.
        """
        try:
            side = "left" if lane in ["left", "lcenter"] else "right"

            # Velocity parameters
            if self.params.congestion_mode:
                v0_mean = self.params.v0_mean_congestion
                v0_std = self.params.v0_std_congestion
            else:
                v0_mean = self.params.v0_mean_normal
                v0_std = self.params.v0_std_normal

            # Initial desired velocity sample
            v0 = np.clip(np.random.normal(v0_mean, v0_std), 5.0, 30.0)

            # Base spawn position
            x0 = -self.params.warmup_length
            spawn_adjusted = False

            # RSS rear-approach safety before spawning
            lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]
            approaching_vehicles = [u for u in lane_vehicles if u.x < x0 and u.v > 0]
            if approaching_vehicles:
                rear_vehicle = min(approaching_vehicles, key=lambda u: x0 - u.x)
                gap_initial = x0 - rear_vehicle.x
                v_rear = rear_vehicle.v
                v_spawn = v0
                if rear_vehicle.x - (-self.params.warmup_length) < 100.0:
                    v_spawn = min(v_spawn, rear_vehicle.v)
                a_brake = 6.0
                t_react = 0.1
                s0_min = 2.0
                v_rel = max(0.0, v_rear - v_spawn)
                rss_distance = (v_rel**2) / (2 * a_brake) + v_rel * t_react + s0_min
                required_gap = rss_distance + self.params.L_vehicle
                if gap_initial < required_gap:
                    x0_new = rear_vehicle.x - required_gap
                    if x0_new < -self.params.warmup_length - 100.0:
                        return False
                    x0 = x0_new
                    spawn_adjusted = True

            # Velocity matching (rear + front)
            lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]
            rear_vehicles = [u for u in lane_vehicles if u.x < -self.params.warmup_length + 100.0]
            front_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited and u.x >= x0 - 5.0]
            v_spawn = v0
            v_spawn_original = v0
            if rear_vehicles:
                leader = max(rear_vehicles, key=lambda u: u.x)
                if leader.x - x0 < 50.0:
                    v_spawn = min(v_spawn, leader.v)
            if front_vehicles:
                nearest_front = min(front_vehicles, key=lambda u: abs(u.x - x0))
                dist_to_front = nearest_front.x - x0
                BASE_MAX_RELATIVE_SPEED = 10.0
                if nearest_front.v < 10.0:
                    MAX_RELATIVE_SPEED = 5.0 + (nearest_front.v / 10.0) * 5.0
                else:
                    MAX_RELATIVE_SPEED = BASE_MAX_RELATIVE_SPEED
                if dist_to_front < 20.0:
                    MAX_RELATIVE_SPEED = min(MAX_RELATIVE_SPEED, 3.0)
                elif dist_to_front < 30.0:
                    MAX_RELATIVE_SPEED = min(MAX_RELATIVE_SPEED, 5.0)
                MIN_SAFE_GAP = 20.0
                MIN_TTC_SPAWN = 5.0
                potential_v_spawn = nearest_front.v + MAX_RELATIVE_SPEED
                potential_v_rel = max(0.0, potential_v_spawn - nearest_front.v)
                if dist_to_front < 2.0:
                    return False
                if dist_to_front < MIN_SAFE_GAP:
                    MAX_RELATIVE_SPEED = 0.0
                elif potential_v_rel > 0.1:
                    ttc_spawn = dist_to_front / potential_v_rel
                    if ttc_spawn < MIN_TTC_SPAWN:
                        safe_v_rel = dist_to_front / MIN_TTC_SPAWN
                        MAX_RELATIVE_SPEED = max(0.0, min(MAX_RELATIVE_SPEED, safe_v_rel))
                max_safe_spawn_v = nearest_front.v + MAX_RELATIVE_SPEED
                if v_spawn > max_safe_spawn_v:
                    v_spawn = max_safe_spawn_v
            v0 = v_spawn

            # Destination area (50/50 split)
            destination_area = np.random.choice(["LEFT", "RIGHT"], p=[0.5, 0.5])
            in_dest_area = (
                (destination_area == "LEFT" and lane in ["left", "lcenter"]) or
                (destination_area == "RIGHT" and lane in ["rcenter", "right"]))
            needs_initial_lc = not in_dest_area
            can_density_adjust = in_dest_area

            # IDM parameters
            v_desired = np.clip(np.random.normal(25.0, 5.0), 15.0, 35.0)
            T_idm = np.clip(np.random.normal(1.5, 0.3), 1.0, 2.5)
            is_hdv = np.random.random() < self.params.hdv_ratio

            # Create vehicle
            y0 = LANE_OFFSETS.get(lane, 0.0)
            vehicle = Vehicle(
                vehicle_id=self.vehicle_id_counter,
                x=x0,
                y=y0,
                v=v0,
                lane=lane,
                spawn_time=t,
                destination_area=destination_area,
                in_destination_area=in_dest_area,
                needs_initial_lc=needs_initial_lc,
                can_density_adjust=can_density_adjust,
                v_desired=v_desired,
                T_idm=T_idm,
                is_hdv=is_hdv,
                params=self.params
            )
            vehicle.side = side
            vehicle.d = y0

            # Initialize target lanes at spawn (same as _spawn_vehicle)
            if needs_initial_lc:
                if destination_area == "RIGHT":
                    if lane == "left":
                        vehicle.target_lane_prep = "lcenter"
                        vehicle.target_lane_weave = "rcenter"
                    elif lane == "lcenter":
                        vehicle.target_lane_prep = "rcenter"
                        vehicle.target_lane_weave = "right"
                    else:
                        vehicle.target_lane_prep = "rcenter"
                        vehicle.target_lane_weave = "right"
                else:  # LEFT destination
                    if lane == "right":
                        vehicle.target_lane_prep = "rcenter"
                        vehicle.target_lane_weave = "lcenter"
                    elif lane == "rcenter":
                        vehicle.target_lane_prep = "lcenter"
                        vehicle.target_lane_weave = "left"
                    else:
                        vehicle.target_lane_prep = "lcenter"
                        vehicle.target_lane_weave = "left"
                vehicle.target_lane = vehicle.target_lane_prep
            else:
                vehicle.target_lane_prep = None
                vehicle.target_lane_weave = None
                vehicle.target_lane = None

            # Safety spawn positioning (IDM equilibrium spacing checks)
            lane_vehicles = [u for u in self.vehicles if u.lane == lane and not u.exited]
            spawn_x = x0
            safe_to_spawn = False
            retry_count = 0
            MAX_RETRIES = 3
            RETRY_OFFSET = 15.0
            while retry_count < MAX_RETRIES:
                conflict = False
                for u in lane_vehicles:
                    safe_dist = self.params.S_min + u.v * 1.5
                    if abs(u.x - spawn_x) < safe_dist:
                        conflict = True
                        break
                if not conflict:
                    safe_to_spawn = True
                    break
                spawn_x -= RETRY_OFFSET
                retry_count += 1

            if not safe_to_spawn:
                return False

            vehicle.x = spawn_x
            self.vehicles.append(vehicle)
            self.vehicle_id_counter += 1

            # Stats
            self.stats[side]["total_vehicles"] += 1
            if needs_initial_lc:
                self.stats[side]["lc_needed"] += 1

            # Debug log (optional)
            if ENABLE_DEBUG_OUTPUT:
                vehicle_type = "HDV" if is_hdv else "CAV"
                print(f"[SPAWN-SUCCESS] V{vehicle.id} ({vehicle_type}): lane={lane}, x={spawn_x:.1f}m, v={v0:.1f}m/s, dest={destination_area}, t={t:.1f}s")
                if front_vehicles:
                    nearest = min(front_vehicles, key=lambda u: u.x - spawn_x)
                    gap_to_front = nearest.x - spawn_x
                    v_rel_front = v0 - nearest.v
                    print(f"  -> Front: V{nearest.id} at {nearest.x:.1f}m, gap={gap_to_front:.1f}m, v={nearest.v:.1f}m/s, v_rel={v_rel_front:.1f}m/s")
            return True
        except Exception:
            return False

    def _control_update(self, t: float):
        """
        Control update (v11.0 Simplified)

        Reference:
        - Apollo: PlanningComponent calls Frame::Plan() every control cycle
        - Autoware: MotionVelocityOptimizer updates trajectories at fixed rate

        Design:
        - All vehicles get controlled by IntegratedZoneController.control_update()
        - Controller applies acceleration/steering directly to vehicle.ax and vehicle.steering
        - Simulator only needs to call controller and update vehicle states
        """

        # Special handling for 'observed' mode
        if self.mode == 'observed':
            self._control_update_observed(t)
            return

        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_control_update_debug', False):
            print(f"\n[CONTROL UPDATE START t={t:.2f}] ===================================")

        # Collect all active vehicles for control
        # --- modified (Final Fix v11.14): Remove Warmup Zone Filter ---
        # x < -warmup_length (安全確保で後ろにずらされた車両) も含めて
        # 全ての車両をApolloコントローラーの制御下に入れる。
        active_vehicles = [v for v in self.vehicles
                          if not v.exited
                          # and v.x >= -self.params.warmup_length  # ★削除: warmup zone制限撤廃
                          and v.x < self.params.total_length]

        if not active_vehicles:
            if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_control_update_debug', False):
                print(f"[CONTROL UPDATE END t={t:.2f}] No active vehicles\n")
            return

        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_control_update_debug', False):
            print(f"  Active vehicles: {len(active_vehicles)}")
            print(f"  Preparation zone: {len([v for v in active_vehicles if 0 <= v.x < self.params.prep_zone_length])}")
            print(f"  Weaving zone: {len([v for v in active_vehicles if v.x >= self.params.prep_zone_length])}")

        # ========================================================================
        # v11.0 SIMPLIFIED: Call unified controller
        # ========================================================================
        # IntegratedZoneController handles Level 1 (Urgency) and Level 2 (QP)
        # No need for vehicle grouping or zone-specific logic
        self.controller.control_update(t, active_vehicles)
        # ========================================================================
        
        # ========================================================================
        # v23.0 NEW: Update CAV Trajectory Buffer (Apollo Trajectory Sharing)
        # ========================================================================
        # Each CAV shares its predicted trajectory for the next 3 seconds.
        # This enables predictive collision detection between CAVs.
        # ========================================================================
        for v in active_vehicles:
            is_cav = not getattr(v, 'is_hdv', False)
            if is_cav:
                # Generate predicted trajectory for next 3 seconds
                horizon = self.trajectory_horizon  # 3.0s
                dt_traj = self.trajectory_dt  # 0.1s
                n_steps = int(horizon / dt_traj) + 1  # 31 points
                
                # Check for QP trajectory (Shared Plan)
                qp_traj = getattr(v, 'predicted_trajectory', None)

                if qp_traj and len(qp_traj) > 0:
                    # Use QP trajectory
                    # qp_traj format: [(t, s, v, a), ...]
                    # We need to resample/interpolate to dt_traj (0.1s)
                    x_pred = []
                    v_pred = []
                    
                    # Convert list to array for easier access
                    traj_arr = np.array(qp_traj)
                    ts = traj_arr[:, 0]
                    xs = traj_arr[:, 1]
                    vs = traj_arr[:, 2]
                    
                    for i in range(n_steps):
                        t_target = t + i * dt_traj
                        if t_target <= ts[0]:
                            x_pred.append(xs[0])
                            v_pred.append(vs[0])
                        elif t_target >= ts[-1]:
                            # Extrapolate last state
                            dt_ex = t_target - ts[-1]
                            x_pred.append(xs[-1] + vs[-1] * dt_ex)
                            v_pred.append(vs[-1])
                        else:
                            # Interpolate
                            idx = np.searchsorted(ts, t_target)
                            t1, t2 = ts[idx-1], ts[idx]
                            x1, x2 = xs[idx-1], xs[idx]
                            v1, v2 = vs[idx-1], vs[idx]
                            alpha = (t_target - t1) / (t2 - t1)
                            x_pred.append(x1 + alpha * (x2 - x1))
                            v_pred.append(v1 + alpha * (v2 - v1))
                            
                    x_pred = np.array(x_pred)
                    v_pred = np.array(v_pred)
                else:
                    # Fallback: Simple kinematic prediction (constant velocity)
                    x_pred = np.array([v.x + v.v * (i * dt_traj) for i in range(n_steps)])
                    v_pred = np.full(n_steps, v.v)
                
                # Lateral prediction based on LC state
                if v.changing_lane and v.lane_from and v.lane_to:
                    elapsed = t - v.lc_start_time
                    duration = self.params.lc_duration  # v27.14: Use optimized duration
                    d_from = LANE_OFFSETS.get(v.lane_from, 0.0)
                    d_to = LANE_OFFSETS.get(v.lane_to, 0.0)
                    
                    d_pred = []
                    for i in range(n_steps):
                        future_elapsed = elapsed + i * dt_traj
                        progress = np.clip(future_elapsed / duration, 0.0, 1.0)
                        ease = 0.5 * (1.0 - np.cos(np.pi * progress))
                        d_pred.append(d_from + (d_to - d_from) * ease)
                    d_pred = np.array(d_pred)
                else:
                    # Not changing lanes - stay at current d
                    d_pred = np.full(n_steps, v.d)
                
                # Store in shared buffer
                self.cav_trajectories[v.id] = {
                    't_base': t,
                    'x': x_pred,
                    'd': d_pred,
                    'v': v_pred,
                    'v_ref': getattr(v, 'v_ref', v.v),  # v27.11: Share target speed
                    'lane_changing': getattr(v, 'changing_lane', False),
                    'target_lane': getattr(v, 'target_lane', None),
                    'current_lane': v.lane
                }
        # ========================================================================

        # v18.29: Track L1 LC scheduling in statistics
        # L1 controller sets v.lc_scheduled = True when booking LC
        # We need to count these in stats to track L1 scheduling rate
        for v in active_vehicles:
            if getattr(v, 'lc_scheduled', False) and not getattr(v, 'lc_counted_in_stats', False):
                if hasattr(v, 'side') and v.side in self.stats:
                    self.stats[v.side]["lc_scheduled"] += 1
                    v.lc_counted_in_stats = True

        # Note: The new controller directly modifies vehicle.ax and vehicle.steering
        # No schedule dictionaries are returned (no MIQP schedules)
        # Vehicle state updates happen in _update_vehicles() as before

        if ENABLE_DEBUG_OUTPUT and getattr(self.params, 'enable_control_update_debug', False):
            print(f"[CONTROL UPDATE END t={t:.2f}] ===================================\n")

    def _control_update_observed(self, t: float):
        """
        Observed distribution mode control update (compatibility mode)

        This mode uses observed LC distribution from real traffic data
        instead of urgency-based planning.
        """
        if ENABLE_DEBUG_OUTPUT:
            print(f"\n[CONTROL UPDATE START t={t:.2f} (OBSERVED MODE)] =======================")

        lc_needed_vehicles = [v for v in self.vehicles
                              if v.needs_lane_change and not v.exited
                              and v.x < self.params.total_length]

        for v in lc_needed_vehicles:
            lane_is_target = v.target_lane and v.lane != v.target_lane

            if not lane_is_target:
                # Determine target lane based on zone
                if v.x >= self.params.prep_zone_length:
                    # Weaving zone: move towards target area
                    if v.target_lane_weave:
                        v.target_lane = get_next_adjacent_lane(v.lane, v.target_lane_weave)
                    else:
                        v.target_lane = None

            # Schedule LC if target lane is set
            if v.target_lane and v.lane != v.target_lane and not v.lc_scheduled:
                if v.x < self.params.prep_zone_length:
                    x_start = max(v.x, 0.0)
                    x_end = self.params.prep_zone_length
                else:
                    x_start = max(v.x, self.params.prep_zone_length)
                    x_end = self.params.total_length

                # Sample LC position from observed distribution
                lc_position = self.obs_dist.sample_lc_position(x_start=x_start, x_end=x_end)

                # Calculate time
                travel_distance = lc_position - v.x
                if travel_distance > 0 and v.v > 0:
                    travel_time_mean = travel_distance / v.v
                    lc_time_mean = t + travel_time_mean

                    # Add time noise
                    time_noise = np.random.normal(0, self.obs_dist.get_time_distribution_std())
                    lc_time = lc_time_mean + time_noise
                    lc_time = max(lc_time, t + self.params.dt_sim)

                    # Schedule
                    v.lc_scheduled = True
                    v.target_position = lc_position
                    v.scheduled_time = lc_time

                    # Statistics
                    if v.side in self.stats and not v.lc_counted_in_stats:
                        self.stats[v.side]["lc_scheduled"] += 1
                        v.lc_counted_in_stats = True

                    if ENABLE_DEBUG_OUTPUT:
                        print(f"  Veh#{v.id} SCHEDULED (Observed): {v.lane}->{v.target_lane} "
                              f"at x={lc_position:.1f}m, t={lc_time:.1f}s")

        if ENABLE_DEBUG_OUTPUT:
            scheduled_count = len([v for v in self.vehicles if v.lc_scheduled and not v.exited])
            print(f"[CONTROL UPDATE END t={t:.2f} (OBSERVED MODE)] ========================")
            print(f"  Scheduled vehicles: {scheduled_count}\n")

    def _update_vehicles(self, dt: float, t: float):
        """
        Update vehicle states

        Args:
            dt: Time step [s]
            t: Current time [s]
        """
        for v in self.vehicles:
            if v.exited:
                continue

            # --- modified: Enable Apollo Control Everywhere (v11.10 / 2025-12-19) ---
            # Previous: x < 0 forced IDM usage, causing warmup zone collisions
            # Current: Allow controller commands even in warmup zone
            # - Vehicles just spawned (no command yet): Use IDM as buffer
            # - Vehicles with controller commands: Use Apollo control immediately
            # Rationale: Prevents "too late to react" scenario at zone boundary
            # -----------------------------------------------------------------------

            # Check if controller has already issued commands
            has_control_command = (hasattr(v, 'ax') and v.ax != 0.0) or (hasattr(v, 'velocity_profile') and v.velocity_profile)

            # Use IDM only for freshly spawned vehicles (no command yet) deep in warmup zone
            if not has_control_command and v.x < -300.0:
                v_acc = self._compute_idm_acceleration(v)
                v.v_prev = v.v
                v.v += v_acc * dt
                v.v = np.clip(v.v, self.params.v_min, self.params.v_max)
                v.x += v.v * dt
                v.update_history(t)
                continue
            # -----------------------------------------------------------------------

            # ====================================================================
            # v11.10: HDV Control Branch (v11.10 / 2025-12-19)
            # ====================================================================
            # HDVs (Human-Driven Vehicles) always use IDM, never Apollo control
            # - Cannot communicate trajectory information
            # - Simulator treats them as "obstacles" for CAVs
            # - Controller recognizes them by is_hdv flag
            # ====================================================================
            if getattr(v, 'is_hdv', False):
                v_acc = self._compute_idm_acceleration(v)
                v.v_prev = v.v
                v.v += v_acc * dt
                v.v = np.clip(v.v, self.params.v_min, self.params.v_max)
                v.x += v.v * dt
                v.update_history(t)
                continue
            # ====================================================================

            # ====================================================================
            # v11.8: Control Command Priority with AEB Latching (CAVs only)
            # ====================================================================
            # Priority: 1. Latched AEB 2. Critical AEB Command 3. QP Profile 4. IDM
            # AEB Latch: Once AEB triggers, maintain max braking until explicitly cleared
            # Reference: ISO 22179 AEBS - Emergency brake must persist until safe
            # ====================================================================
            if self.mode == 'idm':
                v_acc = self._compute_idm_acceleration(v)
            elif getattr(v, 'aeb_active', False):
                # --- AEB Latch: Maintain emergency braking until controller clears flag ---
                v_acc = -6.0  # Force maximum emergency braking
                # Controller is responsible for clearing aeb_active when safe distance restored
                if ENABLE_DEBUG_OUTPUT and not getattr(v, '_aeb_latch_logged', False):
                    reason = getattr(v, 'aeb_reason', 'Unknown reason')
                    print(f"[AEB-LATCH] V{v.id} Emergency brake latched at t={t:.2f}s | Reason: {reason}")
                    setattr(v, '_aeb_latch_logged', True)
            elif hasattr(v, 'ax') and v.ax <= -5.8:
                # --- Critical AEB command from controller ---
                v_acc = v.ax
                if ENABLE_DEBUG_OUTPUT:
                    print(f"[AEB-CMD] V{v.id} Emergency brake command: {v.ax:.2f} m/s^2 at t={t:.2f}s")
            elif hasattr(v, 'ax') and v.velocity_profile:
                # ================================================================
                # v13.8: Trajectory Stitching Timestamp Fix (Root Cause 7)
                # ================================================================
                # Problem (Root Cause 7): Velocity profile computed at t=21.50s for
                #          next 50 steps, but 10 physics substeps (dt_sim=0.01s) occur
                #          between t=21.50 and t=21.60. Interpolation index calculated as
                #          int((t - t_profile)/dt_sim) causes wrong velocity applied.
                #          Vehicles diverge from computed trajectory.
                #
                # Solution: Recompute interpolation index relative to current physics
                #          timestep within control cycle. Ensures velocity profile
                #          stays synchronized to physics simulation.
                #          Apollo: State resampling every control cycle (10Hz).
                #
                # Reference: Apollo MotionPlanner::GenerateTrajectory()
                #           All trajectory points indexed relative to LATEST reference_line
                #           sampling time, not original planning time.
                # ================================================================
                # time_elapsed_since_solve: How long since QP solved trajectory
                time_elapsed_since_solve = t - v.velocity_profile_time
                
                # NEW: Compute interpolation using consistent time reference
                # Instead of: idx = int(time_elapsed_since_solve / dt_sim)
                # Use: Index relative to control cycle completion time
                profile_duration = len(v.velocity_profile) * self.params.dt_sim
                
                if time_elapsed_since_solve < profile_duration:
                    # Profile still valid
                    # Interpolate using kinematic equation for robustness
                    # v(t) = v0 + a*Deltat (rather than table lookup)
                    # This handles timestamp misalignment gracefully
                    
                    idx = int(time_elapsed_since_solve / self.params.dt_sim)
                    
                    if 0 <= idx < len(v.velocity_profile) - 1:
                        # Linear interpolation between profile points
                        v_idx = v.velocity_profile[idx]
                        v_next = v.velocity_profile[idx + 1]
                        frac = (time_elapsed_since_solve % self.params.dt_sim) / self.params.dt_sim
                        target_v = v_idx + frac * (v_next - v_idx)
                        v_acc = (target_v - v.v) / dt
                    elif idx == len(v.velocity_profile) - 1:
                        target_v = v.velocity_profile[-1]
                        v_acc = (target_v - v.v) / dt
                    else:
                        # Profile expired: use IDM fallback
                        v_acc = self._compute_idm_acceleration(v)
                else:
                    # Profile expired: use IDM fallback
                    v_acc = self._compute_idm_acceleration(v)
            else:
                # No profile: use IDM
                v_acc = self._compute_idm_acceleration(v)

            v_acc = np.clip(v_acc, self.params.a_min, self.params.a_max)
            # ====================================================================

            # Save previous velocity for acceleration checking
            v.v_prev = v.v

            # Update velocity
            v.v += v_acc * dt
            
            # ================================================================
            # v13.7: v_min Relaxation During Emergency Braking (Root Cause 4)
            # ================================================================
            # Problem (Root Cause 4): After AEB triggers, v_min=5.0m/s prevents
            #          further deceleration. Gap stays at 0.69-3.0m permanently,
            #          creating deadlock (V15↔V14 collision pattern).
            #          Vehicles cannot decelerate below 5m/s despite needing to.
            #
            # Solution: Allow v < v_min temporarily when aeb_active=True.
            #          Once gap recovers (gap > release_threshold), restore v_min.
            #          Apollo SafetyManager: Emergency control overrides normal limits.
            #
            # Reference: Apollo emergency_motion_manager.cc
            #           When AEB activated, allows minimum velocity to reduce to 0.0m/s
            #           to ensure collision avoidance capability.
            # ================================================================
            if v.aeb_active:
                # Emergency mode: allow deceleration below v_min
                # Only clip to v_max (prevent illegal acceleration)
                v.v = np.clip(v.v, 0.0, self.params.v_max)
            else:
                # Normal mode: enforce v_min constraint
                v.v = np.clip(v.v, self.params.v_min, self.params.v_max)

            # Update longitudinal position
            v.x += v.v * dt

            # ====================================================================
            # v11.5 FINAL FIX: Lateral Guardian (Physics-Based Collision Guard)
            # ====================================================================
            # Smooth lateral movement during lane changes using sinusoidal easing
            # This enables true Frenet-based obstacle detection
            # --- modified (Final Fix): Added lateral collision check before moving ---
            # ====================================================================
            if v.changing_lane and v.lane_from and v.lane_to:
                elapsed = t - getattr(v, 'lc_start_time', t)
                progress = getattr(v, 'lc_progress', 0.0)
                # ====================================================================
                # v25.0: Check if LC is paused (waiting for cooperation)
                # ====================================================================
                # ====================================================================
                # ====================================================================
                # v29.0: Phase 2 - Mid-LC Gap Monitoring (prevent gap intrusion)
                # ====================================================================
                # During critical LC phase (30-70%), monitor front gap
                # If gap < 15m, pause LC to prevent AEB trigger
                if 0.3 <= getattr(v, 'lc_progress', 0.0) <= 0.7 and not getattr(v, 'lc_paused', False):
                    # Find front vehicle in target lane
                    target_lane = getattr(v, 'lane_to', None) or v.lane
                    front_v = None
                    min_gap = float('inf')
                    for u in self.vehicles:
                        if not u.exited and u.lane == target_lane and u.x > v.x:
                            gap = u.x - v.x - self.params.L_vehicle
                            if gap < min_gap:
                                min_gap = gap
                                front_v = u
                    
                    if front_v and min_gap < 15.0 and min_gap > 0:
                        v.lc_paused = True
                        v.lc_pause_start_time = t
                        v.lc_pause_until = t + 1.0  # Short pause
                        if not hasattr(v, 'mid_lc_gap_pause_logged'):
                            print(f"[V29.0-MID-LC-GAP-PAUSE] V{v.id} paused: front_gap={min_gap:.1f}m < 15m at progress={v.lc_progress*100:.1f}%")
                            v.mid_lc_gap_pause_logged = True

                # v25.0: Check if LC is paused (waiting for cooperation)
                # ====================================================================
                if getattr(v, 'lc_paused', False):
                    if t < getattr(v, 'lc_pause_until', 0):
                        # Still paused - don't update lateral position, just wait
                        # Keep proposed_d as current d to maintain lateral position
                        proposed_d = v.d
                    else:
                        # v29.0: Improved pause/resume - preserve progress continuity
                        pause_start_time = getattr(v, 'lc_pause_start_time', t)
                        pause_duration = t - pause_start_time

                        if pause_duration > 0.05:  # Real pause detected
                            # v29.0: Instead of time adjustment, preserve pause_progress
                            if not hasattr(v, 'lc_pause_progress'):
                                v.lc_pause_progress = v.lc_progress if hasattr(v, 'lc_progress') else 0.0
                            # Adjust time with limit (max 50% of duration)
                            time_adjustment = min(pause_duration, self.params.lc_duration * 0.5)
                            v.lc_start_time += time_adjustment
                            v.lc_paused = False
                            print(f"[V29.0-LC-RESUME] V{v.id} after {pause_duration:.2f}s pause (time+{time_adjustment:.2f}s)")
                        else:
                            # False trigger - don't adjust time
                            v.lc_paused = False

                        # Resume: continue from pause progress
                        elapsed = t - v.lc_start_time
                        duration = self.params.lc_duration
                        progress = np.clip(elapsed / duration, 0.0, 1.0)

                        # v29.0: Progress continuity check with clamp
                        if hasattr(v, 'lc_progress') and v.lc_progress is not None:
                            progress_delta = progress - v.lc_progress
                            if abs(progress_delta) > 0.50:
                                print(f"[V28.3-PROGRESS-ANOMALY-RESUME] V{v.id} t={t:.2f}s: "
                                      f"Δprogress={progress_delta:.3f} (old={v.lc_progress:.3f}, new={progress:.3f})")
                                print(f"  lc_start_time={v.lc_start_time:.2f}, elapsed={elapsed:.2f}, duration={duration:.2f}")
                                self._log_inc('v28_3_progress_anomaly', v.id, t,
                                              phase='resume', delta=progress_delta,
                                              old=v.lc_progress, new=progress,
                                              lc_start_time=v.lc_start_time,
                                              elapsed=elapsed, duration=duration)
                            
                            # v29.0: Clamp resume jump
                            if abs(progress_delta) > 0.15:
                                max_step = 0.15
                                if progress_delta > 0:
                                    progress = v.lc_progress + max_step
                                else:
                                    progress = v.lc_progress - max_step
                                print(f"[V29.0-RESUME-CLAMP] V{v.id}: Clamped Δ={progress_delta:.3f} to ±{max_step:.3f}")
                        
                        # Clear pause progress marker (reset to 0 instead of delete)
                        v.lc_pause_progress = 0.0
                        v.lc_progress = progress

                        d_from = LANE_OFFSETS.get(v.lane_from, 0.0)
                        d_to = LANE_OFFSETS.get(v.lane_to, 0.0)
                        ease_factor = 0.5 * (1.0 - np.cos(np.pi * progress))
                        proposed_d = d_from + (d_to - d_from) * ease_factor
                else:
                    # Normal LC
                    elapsed = t - v.lc_start_time
                    duration = self.params.lc_duration
                    progress = np.clip(elapsed / duration, 0.0, 1.0)

                    # v28.3: LC progress differential check (relaxed, log-only)
                    # v29.0: Added clamp to prevent jumps
                    if hasattr(v, 'lc_progress') and v.lc_progress is not None:
                        progress_delta = progress - v.lc_progress
                        if abs(progress_delta) > 0.50:  # >50% jump
                            print(f"[V28.3-PROGRESS-ANOMALY] V{v.id} t={t:.2f}s: "
                                  f"Δprogress={progress_delta:.3f} (old={v.lc_progress:.3f}, new={progress:.3f})")
                            print(f"  lc_start_time={v.lc_start_time:.2f}, elapsed={elapsed:.2f}, duration={duration:.2f}")
                            self._log_inc('v28_3_progress_anomaly', v.id, t,
                                          phase='normal', delta=progress_delta,
                                          old=v.lc_progress, new=progress,
                                          lc_start_time=v.lc_start_time,
                                          elapsed=elapsed, duration=duration)
                        
                        # v29.0: Clamp to prevent >15% jump per cycle
                        if abs(progress_delta) > 0.15:
                            max_step = 0.15
                            if progress_delta > max_step:
                                progress = v.lc_progress + max_step
                                print(f"[V29.0-PROGRESS-CLAMP] V{v.id}: Clamped +{progress_delta:.3f} to +{max_step:.3f}")
                            elif progress_delta < -max_step:
                                progress = v.lc_progress - max_step
                                print(f"[V29.0-PROGRESS-CLAMP] V{v.id}: Clamped {progress_delta:.3f} to -{max_step:.3f}")
                    v.lc_progress = progress  # Store for next iteration

                    d_from = LANE_OFFSETS.get(v.lane_from, 0.0)
                    d_to = LANE_OFFSETS.get(v.lane_to, 0.0)
                    ease_factor = 0.5 * (1.0 - np.cos(np.pi * progress))
                    proposed_d = d_from + (d_to - d_from) * ease_factor

                # ====================================================================
                # v28.3: LC completion check and cleanup
                # ====================================================================
                # Fix: v28.2 issue where LC completed but changing_lane=True remained
                # causing sudden progress jumps when new LC started
                if progress >= 0.99:
                    # LC is complete, finalize and cleanup
                    v.changing_lane = False
                    v.lane = v.lane_to
                    proposed_d = LANE_OFFSETS.get(v.lane_to, 0.0)  # Snap to target lane
                    
                    # v29.0: Clean up LC state attributes to prevent jump on next LC
                    # Note: These are dataclass fields, so we reset to None/0 instead of delattr
                    v.lc_progress = 0.0
                    v.lane_from = None
                    v.lane_to = None
                    v.lc_pause_progress = 0.0  # Also reset pause progress
                    # Keep lc_start_time for statistics (lc_history/temporal stats)
                    print(f"[V28.3-LC-COMPLETE] V{v.id} t={t:.2f}s: LC completed, "
                          f"finalized in lane={v.lane}, d={proposed_d:.2f}m")
                    self._log_inc('lc_completed', v.id, t,
                                  lane=v.lane, d=proposed_d, progress=progress)

                # ====================================================================
                # v28.3: Anomaly detection (log-only, no correction)
                # ====================================================================
                # v28.3: Detect but don't correct lateral anomalies
                # v28.2: 0.05m threshold was too strict, causing 700+ clamping events
                # Physics: @ 100Hz update, 0.20m/step = 20m/s lateral velocity (very large)
                lateral_delta = proposed_d - v.d
                if abs(lateral_delta) > 0.20:  # v28.3: relaxed from 0.05m to 0.20m
                    print(f"[V28.3-LATERAL-ANOMALY] V{v.id} t={t:.2f}s: "
                          f"Large lateral jump: {v.d:.2f}m -> {proposed_d:.2f}m (Δ={lateral_delta:.2f}m)")
                    if v.changing_lane and hasattr(v, 'lane_from'):
                        print(f"  LC: {v.lane_from} -> {v.lane_to} ({progress*100:.1f}%), "
                              f"elapsed={elapsed:.2f}s")
                    # v28.3: No clamping, let natural dynamics handle it
                    # (v28.3 LC completion check should prevent most issues)
                    self._log_inc('v28_3_lateral_anomaly', v.id, t,
                                  d_before=v.d, d_after=proposed_d,
                                  delta=lateral_delta,
                                  lane_from=getattr(v, 'lane_from', None),
                                  lane_to=getattr(v, 'lane_to', None),
                                  progress=progress, elapsed=elapsed)

                # ====================================================================
                # v27.0: Unified Mid-LC Safety Check (ApolloSafetyManager)
                # ====================================================================
                # Replaces scattered v21.0-v23.0 Guardian logic.
                # Uses 3-second trajectory prediction with stricter thresholds:
                # - Longitudinal: 15m (was 10m)
                # - Lateral: 2.0m (was 2.5m)
                # Single source of truth for mid-LC collision detection.
                # ====================================================================
                safety_manager = get_safety_manager()
                is_safe_lateral, collision_risk_vehicle, collision_time = safety_manager.check_mid_lc_safety(
                    ego=v,
                    vehicles=self.vehicles,
                    cav_trajectories=self.cav_trajectories,
                    t=t,
                    prep_zone_len=self.params.prep_zone_length,
                    total_len=self.params.total_length
                )
                
                if not is_safe_lateral and collision_risk_vehicle is not None:
                    print(f"[V27-GUARDIAN] V{v.id} mid-LC collision risk with V{collision_risk_vehicle.id} "
                          f"predicted at t+{collision_time:.1f}s")
                # ====================================================================

                if is_safe_lateral:
                    old_d_before_move = v.d
                    v.d = proposed_d  # Safe to move laterally

                    # --- Lateral Movement Anomaly Detection ---
                    lateral_movement = abs(v.d - old_d_before_move)
                    max_expected_lateral_speed = 2.5  # m/s (reasonable LC speed)
                    max_expected_movement = max_expected_lateral_speed * dt

                    if lateral_movement > max_expected_movement * 2.5: # 2.5x tolerance
                        print(f"\n⚠️  [LATERAL ANOMALY] t={t:.2f}s")
                        print(f"   V{v.id}: Abnormal lateral jump!")
                        print(f"   d: {old_d_before_move:+.2f}m -> {v.d:+.2f}m (Deltad={lateral_movement:.2f}m)")
                        print(f"   Expected max: {max_expected_movement:.2f}m in {dt:.2f}s")
                        print(f"   LC: {v.lane_from} -> {v.lane_to} ({progress*100:.1f}%)")
                        self._log_inc('guardian_lateral_anomaly', v.id, t,
                                      d_before=old_d_before_move, d_after=v.d,
                                      delta=lateral_movement,
                                      expected=max_expected_movement,
                                      lane_from=v.lane_from, lane_to=v.lane_to,
                                      progress=progress)
                        sys.stdout.flush()
                    # -----------------------------------------------------------------------
                else:
                    # ====================================================================
                    # v25.0: Mid-LC Cooperative Resolution (Apollo-style)
                    # ====================================================================
                    # Instead of just aborting, request blocking vehicle to cooperate.
                    # This enables LC completion instead of repeated abort/retry cycles.
                    #
                    # 1. If blocker is CAV: Request yield (decel) or accelerate
                    # 2. Pause LC (don't fully abort) for 2s
                    # 3. If blocker cooperates, LC resumes
                    # 4. If no cooperation after 3 attempts, then abort
                    # ====================================================================
                    # ====================================================================
                    # Re-calculate progress for cooperative logic (was removed in v27.0 update)
                    elapsed = t - getattr(v, 'lc_start_time', t)
                    duration = self.params.lc_duration
                    progress = np.clip(elapsed / duration, 0.0, 1.0)
                    
                    blocker_is_cav = (collision_risk_vehicle is not None and
                                      not getattr(collision_risk_vehicle, 'is_hdv', False))
                    
                    if blocker_is_cav:
                        blocker_id = collision_risk_vehicle.id if collision_risk_vehicle is not None else 'UNKNOWN'
                        # Request cooperation from blocking CAV
                        if collision_risk_vehicle is not None:
                            req = getattr(collision_risk_vehicle, 'cooperative_yield_request', None)
                            last_req_t = req.get('t_request', 0) if isinstance(req, dict) else 0
                            if not isinstance(req, dict) or last_req_t < t - 1.0:
                                # Fresh request
                                collision_risk_vehicle.cooperative_yield_request = {
                                    'requester_id': v.id,
                                    'action': 'yield',  # Slow down to create gap
                                    't_request': t,
                                    'requester_x': v.x,
                                    'requester_lane_to': v.lane_to
                                }
                                print(f"[V25-COOP-REQUEST] V{v.id} requests V{blocker_id} to yield "
                                      f"(LC progress={progress*100:.1f}%)")
                        
                        # Pause LC instead of aborting - keep changing_lane=True but stop lateral movement
                        v.lc_paused = True
                        v.lc_pause_start_time = t  # v28.0: Record pause start for time adjustment
                        v.lc_pause_until = t + 2.0  # Wait 2s for cooperation
                        v.lc_pause_count = getattr(v, 'lc_pause_count', 0) + 1
                        
                        # If paused too many times, fall back to abort
                        if v.lc_pause_count >= 3:
                            print(f"[V25-COOP-FAIL] V{v.id} LC aborted after {v.lc_pause_count} cooperation attempts")
                            v.changing_lane = False
                            v.lc_started = False
                            v.lc_scheduled = False
                            v.target_lane = None
                            if v.lane_from is not None:
                                v.lane = v.lane_from
                            v.lc_pause_count = 0
                            # v30.0: Use params.lc_cooldown_time (base cooldown after Pause×3)
                            # Note: Current value 5.0s is empirical, not scientifically validated
                            v.scheduled_time = t + self.params.lc_cooldown_time
                        else:
                            print(f"[V25-LC-PAUSE] V{v.id} LC paused at {progress*100:.1f}%, waiting for V{blocker_id}")
                    else:
                        # Blocker is HDV or unknown - abort (no cooperation possible)
                        blocker_id = collision_risk_vehicle.id if collision_risk_vehicle is not None else 'UNKNOWN'
                        print(f"[GUARDIAN-LC-ABORT] V{v.id} LC aborted (HDV/unknown blocker V{blocker_id})")
                        v.changing_lane = False
                        v.lc_started = False
                        v.lc_scheduled = False
                        v.target_lane = None
                        if v.lane_from is not None:
                            v.lane = v.lane_from
                            # Gradual return to original lane
                            d_original = LANE_OFFSETS.get(v.lane_from, 0.0)
                            v.d = v.d + (d_original - v.d) * 0.15
                        # v30.0: HDV blocker case uses reduced cooldown (3.0s heuristic)
                        # Note: Shorter than base 5.0s because no cooperation is possible
                        v.scheduled_time = t + 3.0
                    # ====================================================================
                # -------------------------------------------------------
            else:
                # Not changing lanes: stay at lane center
                # (Could add lateral noise/oscillation here for realism)
                v.d = LANE_OFFSETS.get(v.lane, 0.0)

            # Map Frenet d to Global y (for straight road: y = d)
            v.y = v.d
            # ====================================================================

            v.update_history(t)

            # Zone transition
            if not v.entered_weave_zone and v.x >= self.params.prep_zone_length:
                v.entered_weave_zone = True
                v.weave_zone_entry_time = t

                # Don't reset LC if already scheduled or in progress
                if not (v.lc_scheduled or v.lc_started):
                    v.lc_scheduled = False

                    if not v.in_destination_area:
                        if v.destination_area == "LEFT" and v.lane in ["left", "lcenter"]:
                            v.in_destination_area = True
                            v.target_lane = None
                        elif v.destination_area == "RIGHT" and v.lane in ["rcenter", "right"]:
                            v.in_destination_area = True
                            v.target_lane = None
                        else:
                            if v.target_lane_weave:
                                v.target_lane = get_next_adjacent_lane(v.lane, v.target_lane_weave)
                            else:
                                v.target_lane = None
                    else:
                        v.target_lane = None

            # LC start condition
            if v.lc_scheduled and not v.lc_started:
                # --- modified: Clean Mode Separation ---
                if self.mode == 'l2':
                    # L2モード: 時間のみでトリガー (位置制約・タイムアウト判定なし)
                    start_lc = (v.scheduled_time > 0 and t >= v.scheduled_time)
                else:
                    # Observedモード: 既存ロジック (時間+位置+タイムアウト)
                    time_cond = (v.scheduled_time > 0 and t >= v.scheduled_time)
                    pos_cond = (v.target_position > 0 and abs(v.x - v.target_position) < 2.0 * self.params.cell_length)
                    near_exit = (self.params.total_length - v.x) < 50.0
                    pos_cond_relaxed = (v.target_position > 0 and abs(v.x - v.target_position) < 30.0)

                    if near_exit:
                        start_lc = time_cond and pos_cond_relaxed
                    else:
                        start_lc = time_cond and pos_cond
                # ----------------------------------------------------

                # ====================================================================
                # RSS-BASED LANE CHANGE SAFETY CHECK (Apollo Safety Manager)
                # ====================================================================
                # Before allowing lane change, verify RSS (Responsibility-Sensitive Safety)
                # distance constraints are satisfied for all vehicles in target lane.
                # This prevents dangerous cut-ins that would force others to brake hard.
                # Reference: Apollo RSS Manager + ISO 21934 (ALKS)
                # ====================================================================
                if start_lc and v.target_lane:
                    target_lane_vehicles = [u for u in self.vehicles
                                           if u.lane == v.target_lane and not u.exited and u.id != v.id]

                    unsafe_to_change = False

                    # --- modified (Final Fix v11.9 / 2025-12-18): RSS-Based LC Gate ---
                    # Check rear vehicle RSS distance (cut-in safety)
                    rear_vehicles = [u for u in target_lane_vehicles if u.x < v.x]
                    if rear_vehicles:
                        rear_v = max(rear_vehicles, key=lambda u: u.x)  # Closest rear

                        # RSS distance: How much space does rear need to safely stop?
                        # Formula: d_RSS = v_rear * t_reaction + (v_rear^2 - v_ego^2) / (2 * a_brake)
                        rho = 1.0  # Reaction time [s] (ISO 21934: 1.0s for LC)
                        a_brake = 6.0  # Maximum braking [m/s^2] (UN R157)
                        rss_dist = rear_v.v * rho + (rear_v.v**2 - v.v**2) / (2 * a_brake)
                        rss_dist = max(rss_dist, 5.0)  # Minimum 5m safety margin

                        actual_gap = v.x - rear_v.x - self.params.L_vehicle

                        # Reject LC if actual gap < RSS distance
                        if actual_gap < rss_dist:
                            unsafe_to_change = True
                            # Reschedule
                            v.lc_scheduled = False
                            v.scheduled_time = t + 1.0  # Retry in 1 second

                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[RSS-LC-REJECT] V{v.id} LC blocked: rear V{rear_v.id} gap={actual_gap:.1f}m < RSS={rss_dist:.1f}m")

                    # Also check front vehicle (rare, but possible in dense traffic)
                    front_vehicles = [u for u in target_lane_vehicles if u.x > v.x]
                    if front_vehicles and not unsafe_to_change:
                        front_v = min(front_vehicles, key=lambda u: u.x)  # Closest front

                        # RSS distance from ego to front
                        rho = 0.5  # Reaction time [s]
                        a_brake = 6.0  # Maximum braking [m/s^2]
                        rss_dist = v.v * rho + (v.v**2 - front_v.v**2) / (2 * a_brake)
                        rss_dist = max(rss_dist, 5.0)

                        actual_gap = front_v.x - v.x - self.params.L_vehicle

                        if actual_gap < rss_dist:
                            unsafe_to_change = True
                            v.lc_scheduled = False
                            v.scheduled_time = t + 1.0

                            if ENABLE_DEBUG_OUTPUT:
                                print(f"[RSS-LC-REJECT] V{v.id} LC blocked: front V{front_v.id} gap={actual_gap:.1f}m < RSS={rss_dist:.1f}m")

                    if unsafe_to_change:
                        continue  # Skip LC start
                # ====================================================================

                if start_lc and v.target_lane:
                    # ====================================================================
                    # CRITICAL: Enhanced Safety Check at LC Start (v11.2)
                    # ====================================================================
                    # Double-check safety before initiating lane change
                    # Reference: Apollo DecisionMaker secondary validation
                    #
                    # Checks:
                    # 1. Gap distance (front and rear)
                    # 2. TTC (Time-to-Collision) with rear vehicle
                    # 3. Emergency abort if unsafe
                    # ====================================================================

                    target_lane_vehicles = [u for u in self.vehicles
                                           if u.lane == v.target_lane and not u.exited and u.id != v.id]
                    target_lane_vehicles.sort(key=lambda u: u.x)

                    # ====================================================================
                    # v22.0 NEW: V2V Cooperative LC Intent & Response
                    # ====================================================================
                    # CAV-CAV: Broadcast LC intent, receiving CAVs respond smartly:
                    #   - Accelerate (if faster than requester) → create rear gap
                    #   - Yield (if slower) → soft decel to open front gap
                    #   - Reject (if gap impossible) → requester delayed
                    # CAV-HDV: Fall back to Apollo-style reactive safety
                    # ====================================================================
                    is_cav = not getattr(v, 'is_hdv', False)
                    
                    if is_cav and v.target_lane:
                        # Register/update LC intent
                        dist_to_exit = self.params.total_length - v.x
                        v_priority = dist_to_exit  # Lower = higher priority
                        
                        # v22.1: Exit urgency factor - more aggressive near exit
                        # Urgency: 1.0 (normal) to 2.0 (very urgent) based on distance
                        urgency = 1.0
                        if dist_to_exit < 50.0:  # Very close to exit
                            urgency = 2.0
                        elif dist_to_exit < 100.0:  # Approaching exit
                            urgency = 1.5
                        elif dist_to_exit < 150.0:  # Getting close
                            urgency = 1.2
                        
                        self.lc_intent_registry[v.id] = {
                            'target_lane': v.target_lane,
                            'current_lane': v.lane,
                            'x': v.x,
                            'v': v.v,
                            'priority': v_priority,
                            'urgency': urgency,
                            't': t
                        }
                        
                        # Check for CAV neighbors in target lane - ask them to cooperate
                        for other in target_lane_vehicles:
                            other_is_cav = not getattr(other, 'is_hdv', False)
                            if not other_is_cav:
                                continue  # HDV - no V2V, use reactive safety
                            
                            gap = abs(other.x - v.x)
                            # v22.1: Extend V2V range based on urgency
                            effective_range = SAFETY.V2V_RANGE * urgency
                            
                            if gap < effective_range:  # Within V2V range (scaled by urgency)
                                # Determine best cooperative action for 'other'
                                if other.x > v.x:
                                    # Other is AHEAD of ego
                                    # v22.1: Lower speed threshold when urgent
                                    speed_threshold = 2.0 / urgency  # 2.0 → 1.0 when very urgent
                                    if other.v > v.v + speed_threshold:
                                        # Other is faster → accelerate to pull away → create gap
                                        if not hasattr(other, 'v2v_action') or urgency > 1.5:
                                            other.v2v_action = 'accelerate'
                                            other.v2v_requester = v.id
                                            other.v2v_urgency = urgency
                                            if ENABLE_DEBUG_OUTPUT:
                                                print(f"[V2V-ACCEL] V{other.id} accelerating for V{v.id} LC "
                                                      f"(urgency={urgency:.1f}, other_v={other.v:.1f} > ego_v={v.v:.1f})")
                                    else:
                                        # Other is slower or similar → normal following, no action
                                        pass
                                else:
                                    # Other is BEHIND ego
                                    if other.v > v.v:
                                        # Other is faster and behind → needs to yield (soft decel)
                                        if not hasattr(other, 'v2v_action') or urgency > 1.5:
                                            other.v2v_action = 'yield'
                                            other.v2v_requester = v.id
                                            other.v2v_urgency = urgency
                                            if ENABLE_DEBUG_OUTPUT:
                                                print(f"[V2V-YIELD] V{other.id} yielding for V{v.id} LC "
                                                      f"(urgency={urgency:.1f}, rear vehicle)")
                                    # else: other is slower and behind → no action needed
                    # ====================================================================

                    # ====================================================================
                    # v20.0 NEW: Concurrent LC Conflict Detection (Apollo-inspired)
                    # ====================================================================
                    # Problem: Multiple vehicles LC-ing to the same lane simultaneously
                    # cause overlap collisions because only the initiator is blocked.
                    # 
                    # Solution: Check if ANY other vehicle is already LC-ing to the 
                    # same target lane. If so, lower-priority vehicle yields.
                    # Priority: Closer to destination (weaving exit) wins.
                    # ====================================================================
                    lc_conflict = False
                    for other in self.vehicles:
                        if other.id == v.id or other.exited:
                            continue
                        # Check if other is currently in a lane change
                        other_changing = getattr(other, 'changing_lane', False)
                        other_target = getattr(other, 'target_lane', None)
                        
                        if other_changing and other_target == v.target_lane:
                            # Conflict: Another vehicle is already LC-ing to same target
                            # Priority: Closer to weaving exit wins
                            v_dist_to_exit = self.params.total_length - v.x
                            other_dist_to_exit = self.params.total_length - other.x
                            
                            if other_dist_to_exit < v_dist_to_exit:
                                # Other vehicle is closer to exit -> it has priority -> v yields
                                lc_conflict = True
                                print(f"[LC-CONFLICT] V{v.id} yields to V{other.id} "
                                      f"(both LC to {v.target_lane}, V{other.id} closer to exit)")
                                break
                            elif other_dist_to_exit == v_dist_to_exit:
                                # Tiebreaker: Lower ID wins
                                if other.id < v.id:
                                    lc_conflict = True
                                    print(f"[LC-CONFLICT] V{v.id} yields to V{other.id} "
                                          f"(same distance, lower ID wins)")
                                    break
                        
                        # Also check counter-flow: other in target lane LC-ing to ego's lane
                        if other_changing and other.lane == v.target_lane and other_target == v.lane:
                            # Counter-flow conflict: meeting in the middle
                            lc_conflict = True
                            print(f"[LC-CONFLICT] V{v.id} vs V{other.id} counter-flow "
                                  f"(V{v.id}: {v.lane}->{v.target_lane}, V{other.id}: {other.lane}->{other_target})")
                            break
                    
                    if lc_conflict:
                        v.lc_scheduled = False
                        # v30.0: Quick retry for exit proximity conflict
                        # Note: 0.5s is heuristic - may need tuning based on LC success rate
                        v.scheduled_time = t + 0.5
                        continue  # Skip LC start
                    # ====================================================================

                    # ====================================================================
                    # CRITICAL: Physical Overlap Guard (v11.4 / 2025-12-17)
                    # ====================================================================
                    # Check for ANY vehicle physically overlapping in the target lane
                    # This prevents initiating LC when vehicles are too close regardless
                    # of front/rear classification logic
                    # ====================================================================
                    overlap_unsafe = False
                    for u in target_lane_vehicles:
                        gap_abs = abs(v.x - u.x)
                        # If vehicle centers are closer than length + safety margin
                        # v27.7 FIX: Raised from 5.0m to 10.0m for LC overlap prevention
                        if gap_abs < (self.params.L_vehicle + 10.0):
                            overlap_unsafe = True
                            print(f"[SAFETY ABORT] Veh#{v.id} CRITICAL OVERLAP with Veh#{u.id} "
                                  f"(gap={gap_abs:.1f}m < {self.params.L_vehicle + 10.0:.1f}m)")
                            print(f"  From {v.lane} to {v.target_lane} at t={t:.2f}s, x={v.x:.1f}m")
                            break

                    if overlap_unsafe:
                        v.lc_scheduled = False
                        # v30.0: Mid-range retry for physical overlap prevention
                        # Note: 1.0s is heuristic - may need tuning
                        v.scheduled_time = t + 1.0
                        continue  # Skip LC start
                    # ====================================================================

                    rear_vehicle = None
                    front_gap = float('inf')
                    rear_gap = float('inf')

                    for u in target_lane_vehicles:
                        if u.x > v.x:
                            front_gap = u.x - v.x - self.params.L_vehicle
                            break
                    for u in reversed(target_lane_vehicles):
                        if u.x < v.x:
                            rear_vehicle = u
                            rear_gap = v.x - u.x - self.params.L_vehicle
                            break

                    # Safety threshold checks
                    # v18.27: Dynamic threshold based on relative velocity
                    critical_front_gap = 5.0  # [m]
                    rel_v_rear = rear_vehicle.v - v.v if rear_vehicle else 0.0
                    is_rear_cav = rear_vehicle and not getattr(rear_vehicle, 'is_hdv', False)
                    
                    # RSS-based critical rear gap
                    if rel_v_rear > 0:
                        # Closing: need more gap
                        t_react = 0.5 if is_rear_cav else 1.5
                        a_brake = 4.0 if is_rear_cav else 3.0
                        critical_rear_gap = rel_v_rear * t_react + (rel_v_rear ** 2) / (2 * a_brake) + 5.0
                        critical_rear_gap = max(10.0, min(20.0, critical_rear_gap))  # Clamp [10, 20]
                    else:
                        # Opening or stable
                        critical_rear_gap = 8.0 if is_rear_cav else 12.0
                    
                    critical_rear_ttc = 2.5 if is_rear_cav else 3.0  # v18.27: stricter TTC

                    # Check 1: Gap distance
                    gap_safe = (front_gap >= critical_front_gap and rear_gap >= critical_rear_gap)

                    # Check 2: TTC with rear vehicle
                    ttc_safe = True
                    if rear_vehicle is not None:
                        rear_ttc = SafetyAnalyzer.compute_ttc(
                            rear_vehicle.x, v.x, rear_vehicle.v, v.v, self.params.L_vehicle
                        )
                        if rear_ttc is not None and rear_ttc < critical_rear_ttc:
                            ttc_safe = False
                            print(f"[SAFETY ABORT] Veh#{v.id} LC aborted: rear TTC={rear_ttc:.2f}s < {critical_rear_ttc}s")

                    if not gap_safe:
                        print(f"[SAFETY ABORT] Veh#{v.id} LC aborted: unsafe gap "
                              f"(front={front_gap:.1f}m, rear={rear_gap:.1f}m)")
                        print(f"  From {v.lane} to {v.target_lane} at t={t:.2f}s, x={v.x:.1f}m")
                        # Abort lane change: reschedule
                        v.lc_scheduled = False
                        v.scheduled_time = t + 1.0  # Retry in 1 second
                        continue  # Skip LC start

                    if not ttc_safe:
                        print(f"  From {v.lane} to {v.target_lane} at t={t:.2f}s, x={v.x:.1f}m")
                        # Abort lane change: reschedule
                        v.lc_scheduled = False
                        v.scheduled_time = t + 1.0  # Retry in 1 second
                        continue  # Skip LC start

                    # All safety checks passed: proceed with lane change
                    # v18.27: Log cooperative gap info instead of warning
                    coop_active = rear_vehicle is not None and getattr(rear_vehicle, 'cooperative_decel_request', 0.0) > 0.1
                    if coop_active and rear_vehicle is not None:
                        coop_decel = getattr(rear_vehicle, 'cooperative_decel_request', 0.0)
                        print(f"[LC START] Veh#{v.id} with CAV cooperation: "
                              f"rear V{rear_vehicle.id} decel_req={coop_decel:.1f}m/s²")
                        print(f"  Gaps: front={front_gap:.1f}m, rear={rear_gap:.1f}m (dynamic_min={critical_rear_gap:.1f}m)")

                    v.lc_started = True
                    v.lc_start_time = t
                    v.lc_start_position = v.x
                    v.changing_lane = True
                    v.lc_progress = 0.0  # v29.0: Explicit init to prevent jump
                    v.lc_pause_count = 0  # Reset pause counter
                    v.lc_paused = False
                    v.lane_from = v.lane
                    v.lane_to = v.target_lane
                else:
                    # --- modified (Critical Fix): Correct Mode Separation ---
                    # L2モードとObservedモードで完全に分離した判定ロジック
                    if self.mode == 'l2':
                        # L2モード: 時間のみでトリガー（位置制約なし、タイムアウトなし）
                        # Apollo制御は動的なので、「決まった地点」という概念がない
                        # scheduled_time時刻に達したら、その時点で安全かどうかを再チェックして開始
                        pass  # start_lcはすでに上で設定済み（時間条件のみ）

                    else:
                        # Observedモード: 統計分布に基づく位置制約あり
                        # タイムアウト判定もObservedモードのみで実行
                        time_margin = 3.0
                        time_overdue = (v.scheduled_time > 0 and t > v.scheduled_time + time_margin)

                        if time_overdue and not start_lc:
                            # 位置に到達できずタイムアウト: リスケジュール
                            if ENABLE_DEBUG_OUTPUT or True:
                                pos_diff = abs(v.x - v.target_position) if v.target_position > 0 else 0
                                print(f"[LC RESCHEDULE] Veh#{v.id}: Time overdue "
                                      f"(scheduled={v.scheduled_time:.2f}s, current={t:.2f}s), "
                                      f"pos_diff={pos_diff:.1f}m")
                                print(f"  Current: lane={v.lane}, x={v.x:.1f}m, v={v.v:.1f}m/s")
                                print(f"  Target: lane={v.target_lane}, target_x={v.target_position:.1f}m")
                                print("  Reason: Unable to reach target position")

                            v.lc_scheduled = False
                            v.scheduled_cell = -1
                            v.scheduled_kappa = -1
                            v.scheduled_time = -1.0
                            v.target_position = -1.0
                            v.initial_schedule = None
                    # ----------------------------------------------------

            # LC completion
            if v.lc_started and not v.lc_completed:
                if t - v.lc_start_time >= self.params.lc_duration:
                    old_lane = v.lane
                    old_in_area = v.in_destination_area

                    v.lc_completed = True
                    v.lc_end_time = t
                    v.lc_end_position = v.x
                    v.lane = v.target_lane if v.target_lane else v.lane
                    v.changing_lane = False

                    # ================================================================
                    # v13.5: Frenet d-coordinate Update on LC Completion
                    # ================================================================
                    # Problem: When LC completes, v.lane is updated but v.d (Frenet
                    #          lateral offset) was not reset to the new lane centerline.
                    #          This caused collision detection (lateral_dist < 1.8m)
                    #          to use stale d-coordinates from the old lane.
                    # Solution: Reset v.d to new lane center (d=0.0 for target lane)
                    # Reference: Apollo Frenet Frame Consistency - Ensure all
                    #            coordinate systems align after state transitions
                    #            (Reference: Apollo Planning/PathData coordinate system)
                    # ================================================================
                    # Snap to the actual lane center to keep collision checks honest
                    v.d = LANE_OFFSETS.get(v.lane, 0.0)
                    # ================================================================

                    side = v.side
                    if not v.lc_completed_prep and not v.lc_completed_weave:
                        if v.needs_initial_lc:
                            self.stats[side]["lc_completed"] += 1

                    if v.x < self.params.prep_zone_length:
                        v.lc_completed_prep = True
                    else:
                        v.lc_completed_weave = True

                    v.lc_scheduled = False
                    v.lc_started = False

                    # Record LC history
                    if v.lc_start_position >= 0.0:
                        self.lc_history.append({
                            'vehicle_id': v.id,
                            'x': v.lc_start_position,
                            't': v.lc_start_time,
                            'lane_from': old_lane,
                            'lane_to': v.lane
                        })

                        # Update history arrays
                        lc_x = v.lc_start_position
                        if lc_x < self.params.prep_zone_length:
                            cell_idx = int(lc_x / self.params.cell_length)
                            if 0 <= cell_idx < self.params.num_cells_prep:
                                if v.lane in ["lcenter", "left"] or v.destination_area == "LEFT":
                                    self.history_prep_left[cell_idx] += 1.0
                                elif v.lane in ["rcenter", "right"] or v.destination_area == "RIGHT":
                                    self.history_prep_right[cell_idx] += 1.0
                        else:
                            lc_x_local = lc_x - self.params.prep_zone_length
                            cell_idx = int(lc_x_local / self.params.cell_length)
                            if 0 <= cell_idx < self.params.num_cells_weave:
                                self.history_weave[cell_idx] += 1.0

                    is_in_area = False
                    if v.destination_area == "LEFT" and v.lane in ["left", "lcenter"]:
                        is_in_area = True
                    elif v.destination_area == "RIGHT" and v.lane in ["rcenter", "right"]:
                        is_in_area = True

                    if v.in_destination_area:
                        is_in_area = True

                    v.in_destination_area = is_in_area

                    if ENABLE_DEBUG_OUTPUT:
                        print(f"[LC COMPLETED t={t:.2f}] Veh#{v.id}: {old_lane}->{v.lane} | "
                              f"x={v.x:.1f}, dest={v.destination_area}, "
                              f"in_area: {old_in_area}->{is_in_area}")

                    if is_in_area:
                        v.target_lane = None
                    else:
                        v.lc_completed = False
                        if v.target_lane_weave:
                            v.target_lane = v.target_lane_weave
                        else:
                            if v.target_lane_weave:
                                next_lane = get_next_adjacent_lane(v.lane, v.target_lane_weave)
                                v.target_lane = next_lane if next_lane else v.lane
                            else:
                                v.target_lane = v.lane

            # Exit condition
            if v.x >= self.params.total_length and not v.exited:
                v.exited = True
                v.x_exit = v.x
                self.completed_vehicles.append(v)

                if v.destination_area:
                    judgment_lane = v.lane
                    if v.lc_started and not v.lc_completed:
                        judgment_lane = v.target_lane if v.target_lane else v.lane
                        if v.needs_initial_lc:
                            self.stats[v.side]["lc_started_at_exit"] += 1

                    if v.destination_area == "LEFT":
                        success = (judgment_lane in ["left", "lcenter"])
                    elif v.destination_area == "RIGHT":
                        success = (judgment_lane in ["rcenter", "right"])
                    else:
                        success = False

                    if success:
                        self.stats[v.side]["reached_target"] += 1
                    else:
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[FAILURE] Veh#{v.id} (Side: {v.side}) "
                                  f"Exited in {judgment_lane}, "
                                  f"Needed {v.destination_area}")

                    self.stats[v.side]["exited_vehicles"] += 1

        self.vehicles = [v for v in self.vehicles if not v.exited]

    def _evaluate_safety(self):
        """
        Evaluate safety for all lane pairs

        Evaluates:
        - Headway (inter-vehicle spacing)
        - Collisions (headway < 0.1m)
        - Near-miss events (headway < 2.0m)
        - TTC (Time-to-Collision)
        - Constraint violations
        
        v13.4: Now includes collision recovery to separate overlapping vehicles
        """
        L_vehicle = 5.0  # Vehicle length for collision detection
        
        # Evaluate each lane
        for lane_idx in [0, 1, 2, 3]:
            lane_vehicles = self._get_lane_vehicles(lane_idx)

            # Analyze adjacent pairs
            for i in range(len(lane_vehicles) - 1):
                v_rear = lane_vehicles[i]
                v_front = lane_vehicles[i + 1]
                is_collision = self.safety_analyzer.analyze_vehicle_pair(v_rear, v_front, L_vehicle=L_vehicle)
                
                # ================================================================
                # v13.9: Collision Recovery with v_min Bypass (Anti-Deadlock)
                # ================================================================
                # Problem (Root Cause 6 Remaining): After collision, vehicles
                #          remained overlapped because v_min=5.0m/s was applied
                #          even during collision recovery. With both vehicles at
                #          same speed, gap never opens.
                #
                # Solution: Don't force v_min during collision recovery.
                #          Allow rear vehicle to slow below v_min if needed.
                #          Use cooperative deceleration: rear slows down, front
                #          accelerates (if possible) to open gap faster.
                #
                # Reference: Apollo SafetyManager - Emergency bypass of limits
                #            CARLA collision response - Physics-based separation
                # ================================================================
                if is_collision:
                    # Calculate actual overlap
                    headway = v_front.x - v_rear.x - L_vehicle
                    
                    if headway < 0.0:
                        # ====================================================================
                        # v29.0: Phase 3 - Improved Collision Recovery
                        # - Add proper collision statistics (was missing!)
                        # - Gradual position recovery (10 steps instead of instant)
                        # - Acceleration-limited velocity adjustment
                        # ====================================================================
                        
                        # 1. Record collision (THIS WAS MISSING!)
                        # Use analyze_vehicle_pair() which properly counts collision
                        # Use configured vehicle length from parameters
                        is_collision = self.safety_analyzer.analyze_vehicle_pair(
                            v_rear, v_front, self.params.L_vehicle
                        )
                        
                        # Vehicles are physically overlapping
                        overlap = -headway + 0.5  # Add 0.5m safety buffer
                        
                        # 2. Gradual position recovery (not instant teleport)
                        if not hasattr(v_rear, 'collision_recovery_steps'):
                            v_rear.collision_recovery_steps = 10
                            v_rear.collision_recovery_dx = -overlap * 0.6 / 10  # per step
                            v_front.collision_recovery_steps = 10
                            v_front.collision_recovery_dx = overlap * 0.4 / 10
                        
                        # Apply one step of gradual recovery
                        if v_rear.collision_recovery_steps > 0:
                            v_rear.x += v_rear.collision_recovery_dx
                            v_rear.collision_recovery_steps -= 1
                        if v_front.collision_recovery_steps > 0:
                            v_front.x += v_front.collision_recovery_dx
                            v_front.collision_recovery_steps -= 1
                        
                        # 3. Acceleration-limited velocity adjustment
                        avg_speed = (v_rear.v + v_front.v) / 2.0
                        target_v_rear = max(0.0, avg_speed * 0.7)
                        target_v_front = min(self.params.v_max, avg_speed * 1.1)
                        
                        # Apply with accel limit
                        max_decel = -6.0 * self.params.dt_sim
                        max_accel = 2.0 * self.params.dt_sim
                        v_rear.v = max(v_rear.v + max_decel, target_v_rear)
                        v_front.v = min(v_front.v + max_accel, target_v_front)
                        
                        # Mark to skip accel constraint check
                        setattr(v_rear, 'skip_next_accel_check', True)
                        setattr(v_front, 'skip_next_accel_check', True)
                        
                        # Force AEB on rear vehicle
                        v_rear.aeb_active = True
                        v_rear.ax = -6.0
                        
                        # Signal front vehicle to accelerate
                        if not getattr(v_front, 'aeb_active', False):
                            v_front.ax = min(2.0, self.params.a_max)
                        
                        if ENABLE_DEBUG_OUTPUT:
                            print(f"[V29.0-COLLISION-RECOVERY] V{v_rear.id}↔V{v_front.id}: "
                                  f"overlap={overlap:.2f}m (headway={headway:.2f}m), "
                                  f"v_rear={v_rear.v:.1f}m/s, v_front={v_front.v:.1f}m/s, "
                                  f"recovery_steps={v_rear.collision_recovery_steps}")
                # ================================================================

        # Check constraint violations for all vehicles
        for v in self.vehicles:
            if not v.exited:
                self.safety_analyzer.check_constraint_violations(
                    v,
                    v_min=self.params.v_min,
                    v_max=self.params.v_max,
                    a_min=self.params.a_min,
                    a_max=self.params.a_max,
                    dt=self.params.dt_sim
                )

    def _get_lane_vehicles(self, lane_idx: int) -> List[Vehicle]:
        """
        Get vehicles in a specific lane (including LC vehicles)

        Args:
            lane_idx: Lane index (0: left, 1: lcenter, 2: rcenter, 3: right)

        Returns:
            List of vehicles sorted by x position
        """
        lane_map = {0: "left", 1: "lcenter", 2: "rcenter", 3: "right"}
        target_lane = lane_map[lane_idx]

        vehicles = []
        for v in self.vehicles:
            if v.exited:
                continue

            # Normal driving
            if not v.changing_lane and v.lane == target_lane:
                vehicles.append(v)

            # LC in progress: include in both source and target lanes
            elif v.changing_lane:
                lane_from = getattr(v, 'lane_from', None)
                lane_to = getattr(v, 'lane_to', None)
                if lane_from is not None and lane_to is not None:
                    if lane_from == target_lane or lane_to == target_lane:
                        vehicles.append(v)
                elif v.lane == target_lane:
                    vehicles.append(v)

        vehicles.sort(key=lambda v: v.x)
        return vehicles

    def _compute_idm_acceleration(self, v: 'Vehicle') -> float:
        """
        Compute IDM acceleration

        Args:
            v: Vehicle

        Returns:
            Acceleration [m/s^2]
        """
        lane_vehicles = [veh for veh in self.vehicles
                        if veh.lane == v.lane and not veh.exited and veh.x > v.x]

        if not lane_vehicles:
            return self.idm.acceleration(v.v, float('inf'), v.v_desired)

        leader = min(lane_vehicles, key=lambda veh: veh.x - v.x)
        gap = leader.x - v.x

        return self.idm.acceleration(v.v, gap, leader.v)

    def _print_final_statistics(self):
        """Print final statistics"""
        print(f"\n{'='*80}")
        print(f"[Final Statistics - v11.0 ({self.mode.upper()})]")
        print(f"{'='*80}\n")
        sys.stdout.flush()

        for side in ["left", "right"]:
            total_vehicles = self.stats[side].get("total_vehicles", 0)
            exited = self.stats[side].get("exited_vehicles", 0)
            needed = self.stats[side]["lc_needed"]
            scheduled = self.stats[side]["lc_scheduled"]
            completed = self.stats[side]["lc_completed"]
            reached = self.stats[side]["reached_target"]
            started_at_exit = self.stats[side]["lc_started_at_exit"]

            # --- modified: Per-side Success Rate based on Schedule ---
            sched_pct = (scheduled / needed * 100) if needed > 0 else 0.0
            # 予約した車のうち、完了できた割合 (Execution Rate)
            comp_pct = (completed / scheduled * 100) if scheduled > 0 else 0.0
            exit_success_pct = (reached / exited * 100) if exited > 0 else 0.0
            exit_comp_pct = (exited / total_vehicles * 100) if total_vehicles > 0 else 0.0
            overall_pct = (reached / total_vehicles * 100) if total_vehicles > 0 else 0.0

            print(f"[{side.upper()} side spawn]")
            print(f"  Total vehicles spawned:  {total_vehicles}")
            print(f"  Exited vehicles:         {exited} ({exit_comp_pct:.1f}% of spawned)")
            print(f"  LC needed (Initial):     {needed}")
            print(f"  LC scheduled (L1):       {scheduled} ({sched_pct:.1f}% of needed)")
            print(f"  LC completed:            {completed} ({comp_pct:.1f}% of scheduled)")  # ★変更
            print(f"  Target reached:          {reached} ({exit_success_pct:.1f}% of exited)")
            print(f"  LC in progress at exit:  {started_at_exit}")
            print("  ───────────────────────────────────────────────")
            print(f"  Exit success rate:       {exit_success_pct:.1f}%  # Control performance")
            print(f"  Exit completion rate:    {exit_comp_pct:.1f}%  # Sim time adequacy")
            print(f"  Overall success rate:    {overall_pct:.1f}%  # (Reached / Spawned)")
            print()

        total_needed = self.stats["left"]["lc_needed"] + self.stats["right"]["lc_needed"]
        total_scheduled = self.stats["left"]["lc_scheduled"] + self.stats["right"]["lc_scheduled"]
        total_completed = self.stats["left"]["lc_completed"] + self.stats["right"]["lc_completed"]
        total_reached = self.stats["left"]["reached_target"] + self.stats["right"]["reached_target"]
        total_exited = self.stats["left"].get("exited_vehicles", 0) + self.stats["right"].get("exited_vehicles", 0)
        total_started_at_exit = self.stats["left"]["lc_started_at_exit"] + self.stats["right"]["lc_started_at_exit"]
        total_vehicles_overall = self.stats["left"].get("total_vehicles", 0) + self.stats["right"].get("total_vehicles", 0)

        # --- modified: Overall Success Rate based on Schedule ---
        # 定義変更: LC execution rate = Completed / Scheduled
        lc_execution_rate = (total_completed / total_scheduled * 100) if total_scheduled > 0 else 0.0
        exit_success_rate = (total_reached / total_exited * 100) if total_exited > 0 else 0.0
        exit_comp_rate = (total_exited / total_vehicles_overall * 100) if total_vehicles_overall > 0 else 0.0
        overall_success_rate = (total_reached / total_vehicles_overall * 100) if total_vehicles_overall > 0 else 0.0

        print("[OVERALL - v11.0]")
        print(f"  Total vehicles spawned:      {total_vehicles_overall}")
        print(f"  Exited vehicles:             {total_exited} ({exit_comp_rate:.1f}% of spawned)")
        print(f"  Total LC needed (Initial):   {total_needed}")
        print(f"  Total LC scheduled (L1):     {total_scheduled}")
        print(f"  Total LC completed:          {total_completed}")
        print(f"  Target reached:              {total_reached} ({exit_success_rate:.1f}% of exited)")
        print(f"  LC in progress at exit:      {total_started_at_exit}")
        sys.stdout.flush()
        print("  ═══════════════════════════════════════════════════════")
        sys.stdout.flush()
        print(f"  LC execution rate:           {lc_execution_rate:.1f}%  # (LC completed / Scheduled) ★")
        sys.stdout.flush()
        print(f"  Exit success rate:           {exit_success_rate:.1f}%  # CONTROL PERFORMANCE ★")
        sys.stdout.flush()
        print(f"  Exit completion rate:        {exit_comp_rate:.1f}%  # Sim time adequacy")
        sys.stdout.flush()
        print(f"  Overall success rate:        {overall_success_rate:.1f}%  # (Reached / Spawned)")
        sys.stdout.flush()
        # Note: Exit completion rate is time-bound and not a primary KPI per user request
        print()

        # Debug Log Summary (v28.3)
        print("\n" + "="*80)
        print("[Debug Log Summary - v28.3]")
        print("="*80)
        print(f"  V28.3 Lateral anomalies:    {self.log_counters.get('v28_3_lateral_anomaly', 0)}")
        print(f"  V28.3 Progress anomalies:   {self.log_counters.get('v28_3_progress_anomaly', 0)}")
        print(f"  Guardian lateral anomalies: {self.log_counters.get('guardian_lateral_anomaly', 0)}")
        print(f"  LC completions (counted):   {self.log_counters.get('lc_completed', 0)}")
        sys.stdout.flush()
        # Brief per-vehicle top offenders for anomalies
        if hasattr(self, '_per_vehicle_log') and self._per_vehicle_log:
            for key in ('v28_3_lateral_anomaly', 'v28_3_progress_anomaly', 'guardian_lateral_anomaly'):
                per = self._per_vehicle_log.get(key, {})
                if per:
                    top = sorted(per.items(), key=lambda kv: kv[1], reverse=True)[:5]
                    summary = ", ".join([f"V{vid}:{cnt}" for vid, cnt in top])
                    print(f"  Top {key}: {summary}")
            sys.stdout.flush()

        # Safety metrics
        print("\n" + "="*80)
        print("[Safety Metrics]")
        print("="*80)
        metrics = self.safety_analyzer.get_metrics_dict()
        print(f"  Collisions:              {metrics['collision_count']}")
        print(f"  Near-miss events:        {metrics['near_miss_count']}")
        print(f"  Critical TTC events:     {metrics['critical_ttc_count']}")
        print(f"  Min headway:             {metrics['min_headway']:.2f} m")
        print(f"  Mean headway:            {metrics['mean_headway']:.2f} m")
        print(f"  Velocity violations:     {metrics['velocity_violations']}")
        print(f"  Acceleration violations: {metrics['acceleration_violations']}")
        print(f"  Extreme risk events:     {metrics['extreme_risk_count']}")
        print(f"  High risk events:        {metrics['high_risk_count']}")
        print(f"  Moderate risk events:    {metrics['moderate_risk_count']}")
        sys.stdout.flush()

        # Optional detail: Top offenders for acceleration violations
        per_v = metrics.get('accel_violations_by_vehicle', {})
        if per_v:
            top5 = sorted(per_v.items(), key=lambda kv: kv[1], reverse=True)[:5]
            top_str = ", ".join([f"V{vid}:{cnt}" for vid, cnt in top5])
            print(f"  Accel violation top-5:   {top_str}")
            sys.stdout.flush()

        # Compare sampling effects: recompute accel violations at control rate (10Hz)
        try:
            a_min, a_max = self.params.a_min, self.params.a_max
            sample = self.params.dt_control
            total = 0
            per_v10 = {}
            def count_resampled(v):
                t_hist = getattr(v, 'history_time', [])
                v_hist = getattr(v, 'history_v', [])
                if not t_hist or len(t_hist) < 2:
                    return 0
                i0 = 0
                cnt = 0
                for i in range(1, len(t_hist)):
                    if t_hist[i] - t_hist[i0] + 1e-12 >= sample:
                        dt_local = t_hist[i] - t_hist[i0]
                        if dt_local > 0:
                            a_est = (v_hist[i] - v_hist[i0]) / dt_local
                            if a_est < a_min or a_est > a_max:
                                cnt += 1
                        i0 = i
                return cnt
            for v in list(self.vehicles) + list(self.completed_vehicles):
                c = count_resampled(v)
                if c:
                    total += c
                    per_v10[v.id] = c
            if total:
                print(f"  Accel violations (10Hz): {total}")
                top5_10 = sorted(per_v10.items(), key=lambda kv: kv[1], reverse=True)[:5]
                top_str10 = ", ".join([f"V{vid}:{cnt}" for vid, cnt in top5_10])
                print(f"  └ Top-5 @10Hz:          {top_str10}")
                sys.stdout.flush()
        except Exception as _:
            pass

        # v27.5: Traffic Flow Metrics (Speed Statistics)
        print("\n" + "="*80)
        print("[Traffic Flow Metrics]")
        print("="*80)
        # Calculate speeds from exited vehicles
        exit_speeds = [getattr(v, 'exit_speed', v.v) for v in self.completed_vehicles]
        current_speeds = [v.v for v in self.vehicles]
        all_speeds = exit_speeds + current_speeds
        
        if all_speeds:
            avg_speed = sum(all_speeds) / len(all_speeds)
            min_speed = min(all_speeds)
            max_speed = max(all_speeds)
            congestion_indicator = "YES" if avg_speed < 10.0 else "NO"
        else:
            avg_speed = min_speed = max_speed = 0.0
            congestion_indicator = "N/A"
        
        print(f"  Average speed (all):     {avg_speed:.1f} m/s")
        print(f"  Minimum speed:           {min_speed:.1f} m/s")
        print(f"  Maximum speed:           {max_speed:.1f} m/s")
        print(f"  Congestion detected:     {congestion_indicator}  # (avg < 10 m/s)")
        sys.stdout.flush()

        # Spatiotemporal distribution
        print("\n" + "="*80)
        print(f"[Lane Change Spatiotemporal Distribution - Mode: {self.mode.upper()}]")
        print("="*80)
        sys.stdout.flush()

        if self.lc_history:
            lc_df = pd.DataFrame(self.lc_history)

            x_values = np.array(lc_df['x'].tolist())
            print("\nSpatial Distribution:")
            print(f"  Mean position: {np.mean(x_values):.1f} m")
            print(f"  Std Dev:       {np.std(x_values):.1f} m")
            print(f"  Min position:  {np.amin(x_values):.1f} m")
            print(f"  Max position:  {np.amax(x_values):.1f} m")

            t_values = np.array(lc_df['t'].tolist())
            print("\nTemporal Distribution:")
            print(f"  Mean time: {np.mean(t_values):.1f} s")
            print(f"  Std Dev:   {np.std(t_values):.1f} s")
            print(f"  Min time:  {np.amin(t_values):.1f} s")
            print(f"  Max time:  {np.amax(t_values):.1f} s")

            # Gini coefficient
            x_sorted = np.sort(x_values)
            n = len(x_sorted)
            if np.sum(x_sorted) > 0 and n > 0:
                index = np.arange(1, n + 1)
                gini = (2 * np.sum(index * x_sorted)) / (n * np.sum(x_sorted)) - (n + 1) / n
                print(f"\nGini Coefficient (Spatial): {gini:.3f}")
                print("  (0: Perfect equality, 1: Perfect inequality)")

                # Zone-specific Gini
                x_prep = x_values[(x_values >= 0) & (x_values < self.params.prep_zone_length)]
                if len(x_prep) > 1:
                    x_prep_sorted = np.sort(x_prep)
                    n_prep = len(x_prep_sorted)
                    if np.sum(x_prep_sorted) > 0:
                        index_prep = np.arange(1, n_prep + 1)
                        gini_prep = (2 * np.sum(index_prep * x_prep_sorted)) / (n_prep * np.sum(x_prep_sorted)) - (n_prep + 1) / n_prep
                    else:
                        gini_prep = 0.0
                else:
                    gini_prep = 0.0

                x_weave = x_values[(x_values >= self.params.prep_zone_length) & (x_values < self.params.prep_zone_length + self.params.weave_zone_length)]
                if len(x_weave) > 1:
                    x_weave_sorted = np.sort(x_weave)
                    n_weave = len(x_weave_sorted)
                    if np.sum(x_weave_sorted) > 0:
                        index_weave = np.arange(1, n_weave + 1)
                        gini_weave = (2 * np.sum(index_weave * x_weave_sorted)) / (n_weave * np.sum(x_weave_sorted)) - (n_weave + 1) / n_weave
                    else:
                        gini_weave = 0.0
                else:
                    gini_weave = 0.0

                print(f"  Gini Coefficient (Prep):  {gini_prep:.3f}")
                print(f"  Gini Coefficient (Weave): {gini_weave:.3f}")
            else:
                print("\nGini Coefficient (Spatial): N/A (No LC)")

            cell_counts = {}
            for x in x_values:
                cell = int(x / self.params.cell_length)
                cell_counts[cell] = cell_counts.get(cell, 0) + 1

            if cell_counts:
                print("\nCell-wise LC Distribution:")
                print(f"  Total cells with LC: {len(cell_counts)}")
                print(f"  Max LC per cell: {max(cell_counts.values())}")
                print(f"  Min LC per cell: {min(cell_counts.values())}")
        else:
            print("  No lane change recorded.")

        # Update public counters for external access (e.g., main.py JSON stats)
        self.collision_count = getattr(self.safety_analyzer.metrics, 'collision_count', 0)

        print("\n" + "="*80)
        sys.stdout.flush()

    def get_statistics(self) -> dict:
        """Return a dictionary of key simulation statistics and debug counters."""
        # Aggregate overall LC stats
        total_needed = self.stats["left"]["lc_needed"] + self.stats["right"]["lc_needed"]
        total_scheduled = self.stats["left"]["lc_scheduled"] + self.stats["right"]["lc_scheduled"]
        total_completed = self.stats["left"]["lc_completed"] + self.stats["right"]["lc_completed"]
        total_reached = self.stats["left"]["reached_target"] + self.stats["right"]["reached_target"]
        total_exited = self.stats["left"].get("exited_vehicles", 0) + self.stats["right"].get("exited_vehicles", 0)
        total_vehicles_overall = self.stats["left"].get("total_vehicles", 0) + self.stats["right"].get("total_vehicles", 0)

        lc_execution_rate = (total_completed / total_scheduled) if total_scheduled > 0 else 0.0
        exit_success_rate = (total_reached / total_exited) if total_exited > 0 else 0.0
        overall_success_rate = (total_reached / total_vehicles_overall) if total_vehicles_overall > 0 else 0.0

        # v27.17+: Surface flow/efficiency metrics for BO
        exit_speeds = [getattr(v, 'exit_speed', getattr(v, 'v', 0.0)) for v in self.completed_vehicles]
        current_speeds = [getattr(v, 'v', 0.0) for v in self.vehicles]
        all_speeds = exit_speeds + current_speeds
        if all_speeds:
            avg_speed_all = float(np.mean(all_speeds))
        else:
            avg_speed_all = 0.0

        # Approximate travel times from per-vehicle history timestamps
        travel_times = []
        for v in self.completed_vehicles:
            t_hist = getattr(v, 'history_time', [])
            if t_hist and len(t_hist) >= 2:
                duration = t_hist[-1] - t_hist[0]
                if duration > 0:
                    travel_times.append(duration)
        avg_travel_time = float(np.mean(travel_times)) if travel_times else 999.0

        # v28.1: Gini coefficients for spatial distribution fairness
        # Note on definition:
        # - We follow the same computation as the on-screen report below:
        #   Treat each LC event position x (>=0) as a nonnegative value,
        #   and compute G = (2 Σ_i i·x_(i)) / (n Σ_i x_(i)) - (n+1)/n for sorted x.
        # - This captures inequality of LC positions along x (where LCs concentrate).
        # - Alternative (count-based) Gini using per-cell LC counts can be used,
        #   but we keep this definition for consistency with existing logs.
        gini_overall = 0.0
        gini_prep = 0.0
        gini_weave = 0.0
        try:
            if self.lc_history:
                x_values = np.array([rec['x'] for rec in self.lc_history], dtype=float)
                x_sorted = np.sort(x_values)
                n = len(x_sorted)
                denom = np.sum(x_sorted)
                if n > 0 and denom > 0:
                    index = np.arange(1, n + 1)
                    gini_overall = float((2 * np.sum(index * x_sorted)) / (n * denom) - (n + 1) / n)

                # Zone-specific
                prep_start = 0.0
                prep_end = float(self.params.prep_zone_length)
                weave_end = float(self.params.prep_zone_length + self.params.weave_zone_length)

                x_prep = x_values[(x_values >= prep_start) & (x_values < prep_end)]
                if len(x_prep) > 1 and np.sum(x_prep) > 0:
                    xs = np.sort(x_prep)
                    n_p = len(xs)
                    idx = np.arange(1, n_p + 1)
                    gini_prep = float((2 * np.sum(idx * xs)) / (n_p * np.sum(xs)) - (n_p + 1) / n_p)

                x_weave = x_values[(x_values >= prep_end) & (x_values < weave_end)]
                if len(x_weave) > 1 and np.sum(x_weave) > 0:
                    xs = np.sort(x_weave)
                    n_w = len(xs)
                    idx = np.arange(1, n_w + 1)
                    gini_weave = float((2 * np.sum(idx * xs)) / (n_w * np.sum(xs)) - (n_w + 1) / n_w)
        except Exception:
            # Be robust in case of any numerical issues
            gini_overall = gini_overall or 0.0
            gini_prep = gini_prep or 0.0
            gini_weave = gini_weave or 0.0

        return {
            'lc': {
                'total_needed': total_needed,
                'total_scheduled': total_scheduled,
                'total_completed': total_completed,
                'lc_execution_rate': lc_execution_rate,
                'exit_success_rate': exit_success_rate,
                'overall_success_rate': overall_success_rate
            },
            'spawn': {
                'left': self.stats.get('left', {}),
                'right': self.stats.get('right', {})
            },
            'debug_logs': {
                'counters': dict(self.log_counters),
                'per_vehicle': {k: dict(v) for k, v in self._per_vehicle_log.items()},
                'recent_events': list(self._recent_events)
            },
            # v27.16: Include safety summary counts for convenience
            'collision_count': getattr(self.safety_analyzer.metrics, 'collision_count', 0),
            'aeb_count': getattr(self, 'aeb_trigger_count', 0),
            # v27.17: Surface recent AEB snapshots (last 5) for quick inspection
            'aeb_events': list(self._aeb_event_log[-5:]),
            # v28.0: Flow/efficiency metrics for BO consumers
            'avg_speed_all': avg_speed_all,
            'avg_speed': avg_speed_all,
            'avg_travel_time': avg_travel_time,
            # v28.1: Fairness metrics (lower is better)
            'gini_coef_overall': gini_overall,
            'gini_coef_prep': gini_prep,
            'gini_coef_weave': gini_weave
        }
