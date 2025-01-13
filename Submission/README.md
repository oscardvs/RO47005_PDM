# PDM - Motion Planning for Indoor Firefighter Drone

## Student Information
- **Name 1**: Oscar Devos, 5245982  
- **Name 2**: Clara Espirito Santo, 5557917  
- **Name 3**: Leander Le Ba, 6291325  
- **Name 4**: Nitya Nanvani, 6140289  

---

## Overview
This repository contains all the necessary files and instructions to execute the developed path and motion planning algorithms for an indoor firefighter drone. It also enables running simulations to reproduce and visualize the results. While detailed setup steps are provided, note that some additional packages may need to be installed, which are not explicitly covered in this README.

---

## Repository Contents
Ensure your local repository contains the following files and directories:

- `path_motion_sim.py`: The main script for running the simulation.
- `assets` directory: Contains drone and room URDF files.
- `graph` directory: Stores precomputed grid-based graphs for A* to save computation time on repeated runs with the same parameters.

---

## Instructions

### 1. **Run the Simulation**
To execute the path and motion planner along with the simulation, run the following command in your terminal:

```bash
python3 ./path_motion_sim.py
```

Upon successful execution, the following components will appear:
- **Simulation Environment**: A PyBullet window displaying the environment, with the drone initialized and hovering.
- **Log Window**: Displays details such as the time taken to generate or load the grid for A*.
- **Visualization**: Shows the drone's current thrust values.
- **GUI**: A graphical user interface providing options for controlling the simulation.

### 2. **Control the Simulation via GUI**
Use the following keyboard commands to interact with the simulation:

- **`F`**: Select a random floor in the building as the goal.
- **`A`**: Use the A* algorithm for path planning and execute the simulation.
- **`R`**: Use the RRT algorithm for path planning and execute the simulation.
- After the first execution (without changing the goal):
  - **`Z`**: Re-run A* with the previously selected goal.
  - **`T`**: Re-run RRT with the previously selected goal.
- **`Q`**: Quit the simulation.

> **Note**: Both A* and RRT algorithms may take some time to execute. Once the simulation completes, it will display the drone’s movement from start to goal and a pop-up window showing performance metrics.

### 3. **Adjust Global Variables**
The script `path_motion_sim.py` contains several global variables with default values. You can modify these variables to customize the simulation parameters.

---

## Additional Notes
- Ensure all dependencies for the simulation and visualization are installed. Use a Python environment with the required packages.

---

## Troubleshooting
If you encounter any issues:
- Verify that all required files and directories are present in your local repository.
- Check the terminal output for error messages and resolve them as needed.
- Ensure all dependencies are properly installed and up-to-date.

---

Thank you for using this repository. Happy simulating!
