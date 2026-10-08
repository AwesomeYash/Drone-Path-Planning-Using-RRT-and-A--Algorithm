# Drone Racing Path Planning: RRT, Trajectory Optimization and A*

Planning a simulated drone's path through 4 racing hoops and around obstacles in 3-D, from a sampling-based planner to a smooth optimized trajectory. Built for the Robot Science and Systems course (CS 5335, Northeastern University, Programming Assignment 2) in Google Colab with Python and GTSAM.

## What is implemented

| Stage | Description | Where |
|---|---|---|
| 3-D RRT | Random sampling with at least 20 % goal bias, vectorised nearest-neighbour search, naive then dynamics-based steering | main notebook, Parts 1, 3, 4 |
| Drone dynamics | ZYX Euler attitude, thrust and gravity forces, drag and terminal velocity (k_d = 0.0425), used for steering | Part 2 to 4 |
| Racing | Sequential RRT through hoops in free space and with sphere/box obstacles | Parts 5, 6 |
| Trajectory optimization | Direct transcription: states and controls as decision variables (10N + 6), costs for thrust, angular rate, smoothness (jerk) and gimbal lock, constraints for dynamics, boundary, hoops and collisions, solved with SciPy SLSQP warm-started from the RRT path | Part 7, 8 |
| A* planner (extra credit) | 3-D voxel grid, 26-connected moves, sphere and box obstacles with safety margin, hoop-aware path smoothing | extra-credit notebook |

The mathematical formulation (state and control spaces, dynamics, costs, constraints, solver settings) is written up in [`MATHEMATICAL_FORMULATION.pdf`](MATHEMATICAL_FORMULATION.pdf).

## Repository contents

| File | Purpose |
|---|---|
| `Programming_Assignment_2_Trajectory_Optimization.ipynb` | Main notebook: RRT, dynamics, optimization, racing (outputs cleared) |
| `Programming_Assignment_2_Extra_Credits.ipynb` | A* planner and path smoothing on the 8-obstacle "hard" course (outputs saved) |
| `helpers_obstacles.py` | Obstacle classes, scenes, plotting, collision checks and the provided optimizer wrappers |
| `MATHEMATICAL_FORMULATION.pdf` | 9-page formulation of the optimization problem |

## Requirements

Designed for Google Colab. The notebooks install their dependencies with `%pip install numpy==1.25 gtsam==4.2`; they also use `pandas`, `plotly` and `scipy`. The main notebook downloads `helpers_obstacles.py` with `gdown` from the course Drive; you can instead upload the copy in this repository. It also mounts Google Drive (`drive.mount`), which you can remove when running elsewhere.

## How to run

1. Open a notebook in Google Colab (File → Upload notebook).
2. Upload `helpers_obstacles.py` to the Colab session (or keep the `gdown` cell).
3. Run the cells in order. Parts 1 to 8 build up from RRT to the optimized racing path; the extra-credit notebook runs A* on the hard course.

## Results saved in the repository

Only the extra-credit notebook keeps its outputs:

| Experiment | Result |
|---|---|
| A* on an obstacle-free straight line | 20 nodes expanded, 11.31 m, 0.022 s |
| A* with obstacles (test 2) | 1,248 nodes expanded, 9.81 m, 1.035 s, collision-free |
| A* hard course, first segment | 750 nodes expanded, 8.45 m, 0.522 s |
| Hoop-aware smoothing of the full racing path | 70 → 13 waypoints (81.4 % removed), 49.53 m → 38.64 m (22.0 % shorter), collision-free, all 4 hoops preserved |

The main notebook's outputs are cleared, so RRT versus optimized path-length comparisons are not recorded; re-run it to regenerate them.

## Known limitations

- The notebooks are Colab-oriented (Drive mount, `gdown`, pinned `numpy`/`gtsam` versions).
- The optimization step uses SciPy SLSQP with numerical gradients, which is slow on the hard course (the notebook warns of minutes per attempt).
- Dynamics are quasi-static (terminal-velocity model), not full rigid-body dynamics.
- A metrics cell near the end of the main notebook prints the earlier run's `length_rrt`/`length_opt` instead of the final-course values.

## Credits

- Course: CS 5335, Robot Science and Systems (Northeastern University). The notebook structure, the helper code, and the optimizer wrapper functions (`optimize_trajectory`, `optimize_racing_path_sequential`) were provided by the course; the RRT, dynamics, steering, cost, constraint and initialization functions and the whole A* extra-credit notebook are the student's work.
- Drone kinematics and visualisation helpers are adapted from the open-source *Introduction to Robotics* book code (`gtbook`, https://www.roboticsbook.org), as noted in `helpers_obstacles.py`.
<!-- TODO: `MATHEMATICAL_FORMULATION.pdf` carries the name "Srijan Dokania" on its first page. Confirm authorship and credit accordingly before publishing. -->

## License

MIT License (see `LICENSE`).
