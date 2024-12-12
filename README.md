# RO47005_PDM
Motion Planning for Indoor Fire Fighter Drone

The **clean** project version is inside `project_root`

**RRT** and **RRT_star** are both implemented and working.

#RUN
```
cd ~/RO47005_PDM/project_root
conda activate drones
python visualize_test.py
```

* In order to switch between RRT and RRT_star. Uncomment and comment the imports and the function call in `visualize_test.py` (line 180 - 200 and start of code)

#DEV MODE
If you want to work locally. 
- Change NUM_BUILDINGS to 1 (line 30 of visualize_test.py) to avoid building an entire city each time
- In line 46 of visualize_test.py, change num_floors = random.randint(2, 2) to only generate 2 floors.

#NEXT TO IMPLEMENT

- Spawn the drone instead of a ball (takes literally 5 min its 3 lines of code)
- Understand how drone naviguates
- Develop path follower/controller
- Apply the controller to the drone
- Fine tune paramters to generate path quicker and have smallest offset path/drone_pos
- ...
- Apply other path making algos
