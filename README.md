# RO47005 PDM: Motion Planning for an Indoor Firefighter Drone

Course project for RO47005 Planning and Decision Making at TU Delft (2024–2025). The task is to plan and fly a quadrotor through a multi-storey building to a target floor. The final code builds a small grid of buildings in PyBullet, computes a global path with either A* on a grid or RRT, and follows that path with a convex model predictive controller (MPC, solved with cvxpy) that constrains the drone to a cylinder around the current path segment.

Team: Oscar Devos, Clara Espirito Santo, Leander Le Ba, Nitya Nanvani

## What is in the repository

- `Submission/`: the final code. `path_motion_sim.py` is a single script that builds the environment, plans, controls and visualizes. `assets/` holds the room URDF and the `cf2x` drone URDF and mesh.
- `project_root/`: an earlier, modular version of the project (environment setup, RRT and RRT* planners, a PID controller and trajectory follower). Kept for reference; the submission script does not depend on it.
- `dev_test_urdf/`: development notebooks and URDF experiments (collision checking, RRT and A* prototypes, an MPC notebook).
- `report/`: LaTeX source of the report.
- `PDM_Group_8_Project_preliminary_report.pdf`: the preliminary report.
- `PDM_Project.pdf`: the course assignment brief.
- `POA.txt`: the original plan of action. `README_old.md`: earlier notes on running `project_root`.

## Requirements

There is no requirements file. `path_motion_sim.py` imports `pybullet`, `numpy`, `cvxpy`, `scipy` and `matplotlib`, and uses `tkinter` for the GUI. Install these into a Python 3 environment before running.

## Run

1. Clone the repository and go to the submission folder:
   ```bash
   git clone https://github.com/oscardvs/RO47005_PDM.git
   cd RO47005_PDM/Submission
   ```

2. Start the planner and the simulation:
   ```bash
   python3 path_motion_sim.py
   ```
   On the first run the script creates a `graphs/` directory next to it. A* grids are cached there, so later runs with the same parameters load the grid instead of rebuilding it.

   Four windows open:
   - the PyBullet simulation, with the buildings built and the drone hovering
   - a log window with the time taken to generate or load the A* grid
   - a plot of the drone's current thrust values
   - a small GUI listing the available key commands

3. Control the run from the GUI:
   - `F`: select a random floor in the building as the goal
   - `A`: plan with A* and start the flight
   - `R`: plan with RRT and start the flight
   - `Z`: re-run A* with the previous goal
   - `T`: re-run RRT with the previous goal
   - `Q`: quit

   Both planners can take a while. When planning finishes, the simulation shows the drone flying from start to goal and then opens a pop-up with performance metrics.

4. Tune the simulation. The top of `path_motion_sim.py` defines global variables with default values: drone mass and thrust limits, obstacle buffers, MPC horizon, RRT iterations, A* cell size, number and height of buildings, random seed, and camera options. Edit them to change the run.
