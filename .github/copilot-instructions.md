# Copilot instructions for CAV Weaving Control

## Overview
CAV (Connected Autonomous Vehicle) weaving zone control system with Apollo-compliant Frenet QP control.

## Language/Runtime
- Python 3.8+
- Optimization: OSQP (default)
- Dependencies: `requirements.txt`

## Main Entrypoints
```bash
# Smoke tests
python -m weaving_v11.main --mode test

# Full simulation
python -m weaving_v11.main --mode sim --load [low|medium|high] --tmax 600 [--debug]

# DP optimizer test
python -m weaving_v11.dp_speed_optimizer
```

## Project Structure
```
CAV-weaving-control/
├── weaving_v11/              # Main package
│   ├── main.py               # Entry point
│   ├── controllers.py        # IntegratedZoneController (Level 1 + Level 2)
│   ├── frenet_qp_apollo.py   # Piecewise jerk QP solver
│   ├── dp_speed_optimizer.py # ST-graph DP for YIELD/OVERTAKE decision
│   ├── mpc_controller.py     # Urgency-based safety margin relaxation
│   ├── apollo_safety.py      # RSS, TTC, AEB safety gates
│   ├── simulator.py          # Simulation environment
│   ├── parameters.py         # All tunable parameters
│   └── visualization.py      # Plotting utilities
├── optimization/             # Bayesian optimization
│   ├── bayesian_opt_v11.py   # Parameter tuning script
│   └── best_params_v11_*.json
├── docs/                     # Documentation (ALGORITHMS, ARCHITECTURE, etc.)
└── references/               # Apollo reference code (.cc files)
```

## Control Architecture (Apollo-style)
- **Level 1**: UrgencyPlanner (0.5s interval)
- **Level 2**: Frenet QP (0.1s, horizon ~30 steps)
- RSS safety, gap acceptance, slot-based planning
- Fallback IDM when QP fails

## Key Types
- `VehicleState`: s, v, a, lane (string)
- `ObstacleInfo`: vehicle_state, is_front (bool)
- Lanes: left / lcenter / rcenter / right

## Conventions
- Control loop: 10Hz fixed
- Logging: `[INFO]`, `[WARNING]`, `[ERROR]`, `[JSON_STATS]`
- Outputs go to `outputs/` (gitignored)

## Debug Tips
- AEB triggers → check `parameters.py` thresholds
- LC issues → check `simulator.py` spawn initialization (target_lane_prep/weave)
- QP failures → check `frenet_qp_apollo.py` constraints

## Style
- Maintain docstrings and inline logs
- Small, testable changes preferred
- Always run `python -m weaving_v11.main --mode test` before committing
