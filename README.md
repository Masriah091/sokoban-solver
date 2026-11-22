# Sokoban Solver

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

[![badge](https://make.fun/api/badge/Masriah091/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![badge](https://make.fun/api/badge/Masriah091/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![badge](https://make.fun/api/badge/Masriah091/sokoban-solver)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
