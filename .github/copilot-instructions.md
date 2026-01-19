# Copilot instructions for CAV Weaving Control

- **Language/runtime**: Python 3.8+; optimization via OSQP (default) and optional Gurobi (legacy v10.3). Dependencies pinned in `requirements.txt`.
- **Main entrypoints**:
  - Smoke tests: `python -m weaving_v11.main --mode test` (Frenet QP + Urgency planner unit checks).
  - Full sim: `python -m weaving_v11.main --mode sim --load [low|medium|high|congestion] --tmax 600 [--debug] [--hdv_ratio x] [--until aeb|collision|any] [--config overrides.json] [--export_video]`.
  - DP integration: `python test_dp_integration.py`; DP-only: `python -m weaving_v11.dp_speed_optimizer`.
- **Control architecture (Apollo-style)**:
  - `weaving_v11/controllers.py` `IntegratedZoneController`: Level 1 UrgencyPlanner (0.5s), Level 2 Frenet QP (0.1s, horizon≈80 steps/8s) with RSS safety, gap acceptance, slot-based planning.
  - Trajectory stitching via `prev_traj` cache; fallback IDM when QP fails.
  - Safety gates in `apollo_safety.py`/`safety.py` (RSS, TTC, AEB latch) and lane geometry in `coordinate_transform.py` (lanes: left/lcenter/rcenter/right).
- **Trajectory/Safety solvers**:
  - `frenet_qp_apollo.py`: Piecewise jerk QP; enable DP coarse decision via `use_dp_optimizer=True`; input types `VehicleState`, `ObstacleInfo`.
  - `dp_speed_optimizer.py`: ST-graph DP to choose YIELD/OVERTAKE; called from Frenet controller when flagged.
  - `mpc_controller.py`: Urgency-based safety margin relaxation (`FrenetQPControllerWithRelaxation.adjust_safety_margin`).
- **Simulator pipeline**:
  - `main.py` → `simulation_mode` builds `IntegratedZoneParameters` and `IntegratedZoneSimulator` (`simulator.py`), then drives `IntegratedZoneController` each 0.1s.
  - Parameters live in `parameters.py` (horizon, dt_control, spawn rates, AEB/LC thresholds). Override at runtime with `--config my.json` (applied before sim start); debug mode shortens `tmax`.
  - Logging via `utils.Logger` redirects stdout to `outputs/simulation_log_v11_<LOAD>_<TS>.txt`; JSON metrics emitted as `[JSON_STATS] {...}` for BO scripts.
- **Visualization/exports**:
  - Optional `visualization.py` (plots) auto-detected; set `--export_video` to request MP4 via `export_video` when available.
- **Tuning/automation**:
  - BO scripts (`bayesian_opt_v11.py`, `bayesian_opt_v12.py`) and sweep runners (`run_diagnostic_sweep*.py`, `run_overnight_*`) parse JSON stats; keep `[JSON_STATS]` output stable.
  - Temp configs (`temp_config_13_*.json`) illustrate override structure; `run_sim.py`/`run_batch_simulations.py` wrap repeated sim runs.
- **Common conventions**:
  - Vehicle fields: `Vehicle` (`vehicle.py`) uses x/lane/target_lane; Frenet solvers use `s`, `v`, `a`, `lane` (string) and `ObstacleInfo(is_front: bool)`.
  - Safety gaps use RSS; do not relax front safety limits. LC decisions rely on gap acceptance params (`gap_acceptance_params` in controller).
  - Control loop frequency fixed at 10Hz; physics dt in simulator is finer (see `parameters.py`).
- **Debug tips**:
  - AEB triggers or LC starvation often point to `parameters.py` thresholds or `controllers.py` gap logic; check recent change logs (`CHANGES_v28_*.md`, `VALIDATION_v28_*.md`).
  - For lane-change issues, inspect spawn initialization in `simulator.py` (target_lane_prep/weave) and lane density calc `compute_lane_density`.
- **Style/structure**:
  - Maintain docstrings + inline logs mirroring existing tone (`[INFO]`, `[WARNING]`, `[ERROR]`, `[JSON_STATS]`).
  - Keep backward-compat shims (`PreparationZoneController`, `WeavingZoneController`, dummy attrs) intact for legacy scripts.
- **Outputs**: Plots/videos drop in `outputs/`; heavy artifacts are gitignored—do not commit.

When updating this repo, prefer small, testable changes; rerun `python -m weaving_v11.main --mode test` at minimum. Unsure about a workflow or parameter? Point to the file/section above and request clarification.
