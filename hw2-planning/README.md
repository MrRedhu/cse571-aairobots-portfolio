# HW2 — Planning (PDDL + Refinement)

This project implements a planning pipeline in the AAIRobots environment:

- **High-level planning** using PDDL (domain + problem definition)
- **Downward refinement** that converts a high-level plan into executable robot actions
- **Replayable results** saved to a CSV for simulation playback

## What I implemented
- Completed the **PDDL domain actions** and **problem goal** (see `pddl/`)
- Implemented **downward refinement** logic (`scripts/refinement.py`) that maps symbolic actions into low-level motion/action sequences

## Key idea: Downward refinement
The symbolic planner produces high-level actions (e.g., move / pick / place).
My refinement layer expands these into primitive robot commands (turn / move forward, and action-server calls) so that the plan can be executed in simulation.

## Artifacts
- `pddl/`  
  - `domain_bookWorld.pddl`
  - `domain_cafeWorld.pddl`
  - `problem.pddl`
- `plots/hw2_results.csv`  
  Replay file containing completed runs (used by the Gazebo replay script).

## How to run
From your ROS workspace:

1) Generate a run (writes/updates `results.csv`)
```bash
rosrun hw2 refinement.py --objtypes 1 --objcount 1 --seed 100 --env cafeWorld --clean
Replay in Gazebo using the generated CSV

bash
Copy code
rosrun hw2 gazebo.py --input-file ~/catkin_ws/src/hw2/results.csv
(If a particular seed fails to refine, try another seed.)
