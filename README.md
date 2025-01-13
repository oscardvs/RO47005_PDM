# PDM - Motion Planning for Indoor Firefighter Drone - README

## Student Information
- **Name 1**: Oscar Devos, 5245982
- **Name 2**: Clara Espirito Santo, 5557917
- **Name 3**: Leander Le Ba, 6291325
- **Name 4**: Nitya Nanvani, 6140289

## Overview
This repository contains all the files required to execute the developed path and motion planning algorithms and to run the simulation for reproducing and visualizing the results. It includes detailed instructions, shell commands, and necessary files to ensure reproducibility. Note that some additional packages may need to be installed, which are not explicitly described in this README.

---

## Instructions

1. **Create a new local directory for the repository (the name and location can vary):**
   ```bash
   mkdir -p ~/PDM_project

2. **Navigate to the newly created directory and clone the repository:**
   ```bash
   cd ~/PDM_project
   git clone git@gitlab.ro47003.me.tudelft.nl:students-2425/ro47003_mirte_simulator.git

3. **Verify the repository content after cloning:**
   Your local repository should contain the following files:
   - `path_motion_sim.py`
   - `assets` directory containing the drone and room URDF files
   - `graph` directory (initially empty; it will contain generated grid-based graphs used for A*)

4. **Run the path and motion planner along with the simulation:** 
   Execute the following command:
   ```bash
   python3 path_motion_sim.py
   ```
   Upon execution, the following windows should open:
   - A simulation environment in `PyBullet` where the environment is built, and the drone hovers
   - A log window showing the time taken to generate or load the grid for A* (if previously saved)
   - A visualization of the drone's current thrust values
   - A `GUI` displaying possible user actions

5. **Control the motion planning execution using the `GUI` options:**
   - Press **`F`** to select a random floor in the building as the goal
   - Press **`A`** to use the A* algorithm as the path planner (this starts execution)
   - Press **`R`** to use the RRT algorithm as the path planner (this starts execution)
   - After the first execution, without changing the goal:
     - Press **`Z`** to re-execute A* with the previous goal
     - Press **`T`** to re-execute RRT with the previous goal
   - When finished, press **`Q`** to quit the simulation

   **Note**: Both A* and RRT algorithms may take some time to execute. Once complete, the simulation will display the drone's movement from start to goal, followed by a pop-up window showing the performance metrics.

6. **Modify global variables:**
   The file `path_motion_sim.py` contains multiple global variables with default values. These can be adjusted as needed to customize the simulation behavior.

