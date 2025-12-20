# CSE 571 — AAIRobots Portfolio (Search • Planning • Reinforcement Learning)

This repo is a cleaned portfolio version of my CSE 571 AAIRobots assignments (originally from GitHub Classroom, then forked and reorganized).

## Quick navigation
- HW1 — Classical Search: `hw1-search/`
- HW2 — Planning + Refinement (PDDL): `hw2-planning/`
- HW3 — Reinforcement Learning (Q-Learning): `hw3-rl/`

---

## Visual showcase

### HW1 — A* search replay (Gazebo → GIF)
![HW1 Search Demo](hw1-search/demo/hw1_demo.gif)

### HW3 — Reward vs Episodes (learned policy improving)
![HW3 cafeWorld rewards](hw3-rl/plots/qlearning_cafeWorld.png)
![HW3 bookWorld rewards](hw3-rl/plots/qlearning_bookWorld.png)

---

## What each folder contains

### HW1 — Classical Search
- Code: `hw1-search/scripts/`
- Demo: `hw1-search/demo/`
- Run logs/results: `hw1-search/plots/`

Reference commands (from the assignment):
- Generate CSV output:
  `rosrun hw1 search.py --output-file /tmp/test.csv`
- Replay a specific run:
  `rosrun hw1 gazebo.py --input-file /tmp/test.csv --line 3`

### HW2 — Planning + Refinement
- PDDL: `hw2-planning/pddl/`
- Code: `hw2-planning/scripts/`
- Run logs/results: `hw2-planning/plots/` (portfolio copy of the results CSV)

Reference commands (from the assignment):
- Run refinement:
  `rosrun hw2 refinement.py --objtypes <N> --objcount <N> --seed <seed> --env <cafeWorld|bookWorld>`
- Replay results:
  `rosrun hw2 gazebo.py --input-file ~/catkin_ws/src/hw2/results.csv --line <line>`

### HW3 — Reinforcement Learning (Q-Learning)
- Code: `hw3-rl/scripts/qlearning.py`
- Output CSV + plots: `hw3-rl/plots/`  
  (plots are named `qlearning_{cafeWorld|bookWorld}.png`)

Reference command (from the assignment):
- Generate submission outputs:
  `rosrun hw3 qlearning.py --submit`

Note: HW3 does not include Gazebo playback like HW1/HW2; you can monitor learning via ROS topics (e.g., `rostopic echo /status` while it runs).

