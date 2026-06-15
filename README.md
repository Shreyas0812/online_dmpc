# Online Multi-Robot Motion Planning 
This code accompanies the RA-L/ICRA 2020 paper

>  C. E. Luis, M. Vukosavljev, and A. P. Schoellig, “Online trajectory generation with distributed model predictive control for multi-robot motion
planning,” IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 604–611, Jan. 2020.

---

> **This is a fork.** See [What This Fork Adds](#what-this-fork-adds) for the additions built on top of the original library.

---

## Citation
If you use this library for your own work, consider citing:
```
@article{luis2020online,
  title={Online trajectory generation with distributed model predictive control for multi-robot motion planning},
  author={Luis, Carlos E and Vukosavljev, Marijan and Schoellig, Angela P},
  journal={IEEE Robotics and Automation Letters},
  volume={5},
  number={2},
  pages={604--611},
  year={2020},
  publisher={IEEE}
}
```

## What's included
- Standalone C++ library implementing the algorithm.
- MATLAB code for running the benchmark and visualize data.

## Usage
Below you will find instructions on how to use the two main pieces of software included in this repo.

## C++ Library

Dependencies:
- C++14
- Cmake >= 3.0
- Eigen >= 3.0

### Installation

1. Initialize qpOASES submodule
```
cd <path-to-repo>
git submodule init && git submodule update
```

2. Build the project
```
mkdir build
cd build
cmake ..
make
```
3. Test installation by running example scenario 
```
cd ../bin
./run
```

4. You should see a stream of data in the console. The expected last lines of the console output are:
```
No collisions found!
All the vehicles reached their goals!
Writing solution to text file...
```

5. The generated simulation data is in `cpp/results/trajectories.txt`. You can run the MATLAB script `plot_results.m` for a 3D visualization of the generated trajectories.

### Running your own scenarios
The entry point of the code is `src/main.cpp`. If you want to run your own transition scenarios, or play around with the (many) hyperparameters of the algorithm, the main configuration file is in `cpp/config/config.json`. You can find an explanation of each hyperparameter in `cpp/config/help.txt`. 

## MATLAB
The code was written and executed with **MATLAB2018a**. There's no guarantees it will work in other versions.

### Running benchmark
To run the benchmark presented in the paper against the Buffered Voronoi Cells method, execute `matlab/tests/comp_dmpc_bvc.m`. At the top of the file there's several parameters to change the test characteristics.

### Plotting
- `plot_2agent_video.m`: dynamic 3D visualization of experiment with 2 drones exchanging positions
- `plot_comp_allCA.m`: summarize results from benchmark against BVC method
- `plot_disturbance_exp.m`: plots in paper for continuous vs event-based replanning
- `plot_hoop_test_paper.m`: static 3D visualization of 10 drones passing through a hula-hoop
- `plot_hoop_test_video.m`: dynamic 3D visualization of 10 drones passing through a hula-hoop
- `plot_obstaclefree_video.m`: dynamic 3D visualization of 20 drones randomly transitioning between goal points

---

## What This Fork Adds

This fork extends the original library with:

- **Dynamic goal motion** — goals can move during simulation (circular, translating, random-jump, or combined)
- **Online task reallocation** — Hungarian algorithm periodically re-solves the robot-to-goal assignment to minimize total travel distance
- **9 curated test scenarios** — covering reallocation benefit cases, BVC vs on-demand collision avoidance, and moving goal tracking
- **Scalability experiments** — automated sweep from 4 to 64 agents
- **Python analysis pipeline** — metrics extraction, comparison figures, scalability plots

### Key Changes to Existing Files

- `cpp/src/simulator.cpp` — Added reallocation trigger loop, dynamic goal motion evaluation, configurable output paths, and `saveGoalDataToFile()`
- `cpp/src/generator.cpp` — Added `setGoalPoint()` for mid-run goal updates, `computeGoalPosition()` for moving goals, `getNewHorizon()` for predictive reallocation

### Files Added

| File | Description |
|---|---|
| `cpp/src/task_reallocation.cpp` | Hungarian algorithm manager |
| `cpp/include/task_reallocation.h` | TaskReallocationManager class |
| `cpp/src/bvc_avoidance.cpp` | Explicit BVC collision avoidance |
| `cpp/config/scenario_1-9.json` | 9 pre-configured test scenarios |
| `cpp/config/scenario_scale_*.json` | Scalability configs (4–64 agents) |
| `extract_metrics.py` | Aggregates experiment results |
| `visualize_results.py` | Generates comparison figures |
| `analyze_scalability.py` | Scalability analysis and plots |
| `run_comprehensive_experiments.sh` | Batch runner for all 9 scenarios |
| `run_scalability_experiments.sh` | Batch runner for scalability sweep |

---

## Scenarios

Run any scenario by passing its config file:

```bash
cd cpp/build
./bin/run ../config/scenario_1.json
```

### Reallocation vs Static Assignment

| Config | Agents | Description |
|---|---|---|
| `scenario_1.json` | 4 | Diagonal cross swap — clearest reallocation benefit |
| `scenario_2.json` | 4 | Dense cross — static already optimal, reallocation is a no-op |
| `scenario_3.json` | 4 | Circle 90° rotation — maximum path crossing |

### BVC vs On-Demand Collision Avoidance

| Config | Agents | Description |
|---|---|---|
| `scenario_4.json` | 4 | Cross pattern with BVC (conservative, wider paths) |
| `scenario_5.json` | 4 | Cross pattern with on-demand (tighter coordination) |
| `scenario_6.json` | 6 | Dense 6-agent pattern stress-testing both methods |

### Dynamic / Moving Goals

| Config | Agents | Description |
|---|---|---|
| `scenario_7.json` | 4 | Translating goals — constant-velocity linear motion |
| `scenario_8.json` | 4 | Circular goals — goals orbit fixed centers |
| `scenario_9.json` | 4 | Combined — goals translate and rotate simultaneously |

---

## Task Reallocation Config

```json
{
  "reallocation_enabled": true,
  "reallocation_period": 2.0,
  "_use_predictive": false,
  "_prediction_horizon": 1.0
}
```

When enabled, every `reallocation_period` seconds the Hungarian algorithm re-solves the assignment problem. If a better assignment is found, `Generator::setGoalPoint()` updates each agent's target in place.

---

## Goal Motion Types

Set `"motion_type"` in config:

| Value | Description |
|---|---|
| `"static"` | Goals fixed (original behavior) |
| `"circular"` | Goals orbit a center at radius `goal_circular_radius`, angular velocity `goal_circular_omega` |
| `"translating"` | Goals move linearly at `goal_translation_velocity` |
| `"circular_translating"` | Orbit + linear drift combined |
| `"random_jump"` | Goals teleport randomly every 5 seconds |

---

## Running Experiments

```bash
# From online_dmpc/

# All 9 scenarios, static vs reallocation, 3 runs each
bash run_comprehensive_experiments.sh

# Scalability sweep: 4, 8, 16, 32, 64 agents
bash run_scalability_experiments.sh

# Aggregate and visualize
python extract_metrics.py
python visualize_results.py
python analyze_scalability.py
```

Figures are saved to `cpp/results/experiments/figures/`.