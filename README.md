# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Masriah091/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Masriah091/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A deterministic push-optimal Sokoban solver. It searches in push space
with IDA\* (iterative deepening on pushes + heuristic), guided by static
push-distances to the nearest goal precomputed with reverse pull-BFS;
squares that can never reach a goal are pruned as dead. A transposition
table keyed by the sorted box cells and the canonical player position
(the smallest cell of the player's reachable region) cuts revisits, and
pushes are tried in a fixed best-first order: onto a goal first, then by
heuristic decrease, then by resulting player-region size. Levels are
decomposed into the rooms off the hallway — each room is solved
push-optimally on its own and the solutions are concatenated with
hallway walks, so the push counts add up to the level optimum. No clock
reads and no randomness: two runs produce byte-identical output.

