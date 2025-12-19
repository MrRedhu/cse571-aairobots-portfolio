# Autonomous AI Agents: Search, Planning, and Reinforcement Learning (CSE 571)

This repo showcases three projects demonstrating:
- Classical Search (BFS / UCS / Greedy Best-First / A*)
- Task & Motion-style Planning with PDDL + refinement
- Reinforcement Learning (Q-Learning / Approx Q-Learning) in a stochastic environment

> Note: Original environments/scaffolding were provided via a course GitHub Classroom setup.
> This portfolio repo focuses on *my implementation + demos* and avoids redistributing large third-party code or private identifiers.

---

## Demos (watch first)

- HW3 RL agent demo: `assets/gifs/hw3_rl_demo.gif`
- Reward vs Episodes: `assets/plots/hw3_reward_curve.png`

---

## Homework 1 — Search (BFS, UCS, GBFS, A*)

**What I implemented**
- Graph search with explored/frontier control
- Heuristic design for A*
- Performance profiling: nodes expanded, plan length

**Code**
- `hw1-search/src/`

**Results**
- `hw1-search/plots/` and `assets/plots/`

---

## Homework 2 — Planning (PDDL + Refinement)

**What I implemented**
- High-level planning with PDDL (domain/problem)
- Refinement of actions into executable low-level plans
- Debugging and validation via testcases + simulation runs

**Code**
- `hw2-planning/src/`
- `hw2-planning/pddl/`

---

## Homework 3 — Reinforcement Learning (Q-Learning)

**What I implemented**
- Q-value update rules + epsilon decay
- Epsilon-greedy learning
- Reward tracking + convergence plots
- Handling stochastic transitions

**Code**
- `hw3-rl/src/`

---

## How to run (high level)
Each HW folder will include its own `README.md` with exact run commands and expected outputs.
