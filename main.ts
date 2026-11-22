// Sokoban solver: IDA* in push space with a transposition table and
// deterministic move ordering, exploiting room decomposition.
//
// The search runs on pushes, not steps: a node is (sorted box cells,
// player cell). From a node, a player-reachability BFS finds every stance
// the player can walk to, and each reachable stance that can push a box
// onto a non-dead square generates one child. The heuristic is the sum
// over boxes of a precomputed static push-distance to the nearest goal
// (reverse pull-BFS from each goal over the static grid). Squares that
// cannot reach any goal are dead and pushes landing there are pruned.
//
// IDA* iterates on bound = pushes + heuristic. A transposition table
// keyed by (sorted boxes, canonical player position = smallest cell of
// the player's reachable region) stores the smallest g (pushes so far)
// at which the state was visited in the current iteration; a revisit
// with g >= stored is cut. The table is cleared between iterations, so
// its behaviour is fully deterministic. Pushes are ordered: landing on
// a goal first, then by heuristic decrease, then by larger resulting
// player region, then by (box, direction) for a fixed tie-break.
//
// Levels from the generator are a hallway with rooms off it, one door
// per room, and a box pushed into the hallway can never return. So each
// room is solved separately (player entering through its door, boxes
// confined to the room, final player position able to walk back to the
// door) and the per-room push-optimal solutions are concatenated with
// hallway walks. Push counts add, so the concatenation is push-optimal
// for the whole level. If a level does not decompose this way, the same
// IDA* runs on the whole board.
//
// No clock reads, no randomness: budgets are node counts only, so two
// runs produce byte-identical output.

import { readdirSync, readFileSync, mkdirSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 1, dy: 0, ch: "r" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
];
const INF = 1e9;

// ---------- level parsing ----------

function parseLevel(text) {
  const rows = text.replace(/\r/g, "").split("\n");
  while (rows.length && rows[rows.length - 1] === "") rows.pop();
  const H = rows.length;
  let W = 0;
  for (const r of rows) W = Math.max(W, r.length);
  const floor = new Uint8Array(W * H);
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const ch = x < rows[y].length ? rows[y][x] : "#";
      if (ch === "#") continue;
      const i = y * W + x;
      floor[i] = 1;
      if (ch === "." || ch === "*" || ch === "+") goals.push(i);
      if (ch === "$" || ch === "*") boxes.push(i);
      if (ch === "@" || ch === "+") player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { W, H, floor, goals, boxes, player };
}

// ---------- static push distances ----------

// For the given allowed cell set, compute for every cell the minimum
// number of pushes that moves a lone box from that cell to the nearest
// goal (player reachability ignored, walls only). Returns an array
// minDist indexed by cell, INF when unreachable (a dead square unless
// the cell is itself a goal).
function computeMinDist(W, H, allowed, goals) {
  const N = W * H;
  const minDist = new Int32Array(N).fill(INF);
  const goalSet = new Set(goals);
  // Per-goal reverse pull-BFS, then take the min over goals.
  for (const g of goals) {
    const dist = new Int32Array(N).fill(INF);
    dist[g] = 0;
    const queue = [g];
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      const cx = cur % W, cy = (cur / W) | 0;
      // A box at p = cur - dir was pushed in direction dir to reach cur;
      // the player stood at p - dir = cur - 2*dir.
      for (let d = 0; d < 4; d++) {
        const px = cx - DIRS[d].dx, py = cy - DIRS[d].dy;
        const sx = cx - 2 * DIRS[d].dx, sy = cy - 2 * DIRS[d].dy;
        if (px < 0 || py < 0 || px >= W || py >= H) continue;
        if (sx < 0 || sy < 0 || sx >= W || sy >= H) continue;
        const p = py * W + px, s = sy * W + sx;
        if (!allowed[p] || !allowed[s]) continue;
        if (dist[p] !== INF) continue;
        dist[p] = dist[cur] + 1;
        queue.push(p);
      }
    }
    for (let i = 0; i < N; i++) if (dist[i] < minDist[i]) minDist[i] = dist[i];
  }
  // Goal cells have distance 0 by construction.
  for (const g of goalSet) minDist[g] = 0;
  return { minDist, goalSet };
}

// ---------- player reachability ----------

// BFS over allowed cells avoiding boxSet, starting at start. Returns
// parent map (cell -> [prevCell, dirIndex]) covering the region.
function reachable(W, allowed, boxSet, start) {
  const parent = new Map();
  parent.set(start, null);
  const queue = [start];
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cx = cur % W, cy = (cur / W) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= W || ny >= ((allowed.length / W) | 0)) continue;
      const n = ny * W + nx;
      if (!allowed[n] || boxSet.has(n) || parent.has(n)) continue;
      parent.set(n, [cur, d]);
      queue.push(n);
    }
  }
  return parent;
}

function walkString(parent, from, to) {
  if (from === to) return "";
  const chars = [];
  let cur = to;
  while (cur !== from) {
    const step = parent.get(cur);
    if (!step) return null;
    chars.push(DIRS[step[1]].ch);
    cur = step[0];
  }
  chars.reverse();
  return chars.join("");
}

// ---------- IDA* push-space solver for one (sub)problem ----------

// allowed: Uint8Array over the whole board; movement and pushes stay
// inside it. startPlayer: cell the player stands on initially.
// exitCell: if >= 0, a solved state only counts when the player can
// walk back to exitCell without moving a box. nodeCap: expansion cap.
// Returns { moves, pushes, finalPlayer } or null.
function solvePushIDA(W, H, allowed, boxes0, goals, startPlayer, exitCell, nodeCap) {
  const { minDist, goalSet } = computeMinDist(W, H, allowed, goals);
  const nBoxes = boxes0.length;

  function heuristic(boxes) {
    let h = 0;
    for (let i = 0; i < nBoxes; i++) {
      const d = minDist[boxes[i]];
      if (d >= INF) return INF; // a box on a dead square
      h += d;
    }
    return h;
  }

  const h0 = heuristic(boxes0);
  if (h0 >= INF) return null;

  let nodes = 0;
  let aborted = false;
  const tt = new Map(); // key -> smallest g visited this iteration
  const pathSegs = []; // walk+push string per push on the current path
  let solution = null;

  // Returns: -1 when solved, otherwise the smallest f that exceeded
  // bound below this node (Infinity when the subtree was fully pruned).
  function dfs(boxes, player, g, bound) {
    if (aborted) return Infinity;
    const h = heuristic(boxes);
    if (h >= INF) return Infinity;
    const f = g + h;
    if (f > bound) return f;

    const boxSet = new Set(boxes);
    const reach = reachable(W, allowed, boxSet, player);

    if (h === 0) {
      // Every box on a goal.
      if (exitCell < 0 || reach.has(exitCell)) {
        solution = { moves: pathSegs.join(""), pushes: g, finalPlayer: player };
        return -1;
      }
      // Solved boxes but the player is trapped: keep searching.
    }

    // Transposition table: canonical player = smallest reachable cell.
    let canon = Infinity;
    for (const c of reach.keys()) if (c < canon) canon = c;
    const key = boxes.join(",") + "|" + canon;
    const seen = tt.get(key);
    if (seen !== undefined && seen <= g) return Infinity;
    tt.set(key, g);

    if (++nodes > nodeCap) { aborted = true; return Infinity; }

    // Generate pushes.
    const cand = [];
    for (let bi = 0; bi < nBoxes; bi++) {
      const b = boxes[bi];
      const bx = b % W, by = (b / W) | 0;
      for (let d = 0; d < 4; d++) {
        const sx = bx - DIRS[d].dx, sy = by - DIRS[d].dy; // stance
        const tx = bx + DIRS[d].dx, ty = by + DIRS[d].dy; // box target
        if (sx < 0 || sy < 0 || sx >= W || sy >= H) continue;
        if (tx < 0 || ty < 0 || tx >= W || ty >= H) continue;
        const stance = sy * W + sx, target = ty * W + tx;
        if (!reach.has(stance)) continue;
        if (!allowed[target] || boxSet.has(target)) continue;
        if (minDist[target] >= INF) continue; // dead square prune
        const nb = boxes.slice();
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        const nh = heuristic(nb);
        if (nh >= INF) continue;
        cand.push({ bi, d, stance, target, nb, nh });
      }
    }
    // Move ordering: goal landing first, then heuristic decrease
    // (lower resulting h first), then larger player region after the
    // push, then fixed (box, direction) tie-break.
    for (const c of cand) {
      const nbs = new Set(c.nb);
      // Player stands on the pushed box's old cell after the push.
      const oldCell = boxes[c.bi];
      let region = 0;
      const seenCells = new Set([oldCell]);
      const q = [oldCell];
      let qh = 0;
      while (qh < q.length) {
        const cur = q[qh++];
        region++;
        const cx = cur % W, cy = (cur / W) | 0;
        for (let d2 = 0; d2 < 4; d2++) {
          const nx = cx + DIRS[d2].dx, ny = cy + DIRS[d2].dy;
          if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
          const n = ny * W + nx;
          if (!allowed[n] || nbs.has(n) || seenCells.has(n)) continue;
          seenCells.add(n);
          q.push(n);
        }
      }
      c.onGoal = goalSet.has(c.target) ? 0 : 1;
      c.region = region;
    }
    cand.sort((a, b) =>
      a.onGoal - b.onGoal ||
      a.nh - b.nh ||
      b.region - a.region ||
      a.bi - b.bi ||
      a.d - b.d
    );

    let nextBound = Infinity;
    for (const c of cand) {
      const walk = walkString(reach, player, c.stance);
      if (walk === null) continue;
      pathSegs.push(walk + DIRS[c.d].ch);
      const r = dfs(c.nb, boxes[c.bi], g + 1, bound);
      pathSegs.pop();
      if (solution) return -1;
      if (aborted) return Infinity;
      if (r > 0 && r < nextBound) nextBound = r;
    }
    return nextBound;
  }

  let bound = h0;
  // Deterministic iteration cap: bounds are push counts, bounded by a
  // generous fixed ceiling; the node cap is the real budget.
  for (let iter = 0; iter < 1000; iter++) {
    tt.clear();
    const r = dfs(boxes0.slice(), startPlayer, 0, bound);
    if (solution) return solution;
    if (aborted) return null;
    if (!isFinite(r) || r <= bound) return null;
    bound = r;
  }
  return null;
}

// ---------- whole-level solving with room decomposition ----------

function solveLevel(level) {
  const { W, H, floor, goals, boxes, player } = level;
  const goalSetAll = new Set(goals);

  // Hallway = the player's row (all its floor cells).
  const hy = (player / W) | 0;
  const hallway = new Set();
  for (let x = 0; x < W; x++) {
    const i = hy * W + x;
    if (floor[i]) hallway.add(i);
  }

  // Rooms = connected components of the remaining floor cells.
  const rest = new Set();
  for (let i = 0; i < W * H; i++) if (floor[i] && !hallway.has(i)) rest.add(i);
  const rooms = [];
  let decompOk = hallway.size > 0 && !boxes.some((b) => hallway.has(b)) &&
    !goals.some((g) => hallway.has(g));
  while (rest.size) {
    const seed = Math.min(...rest);
    rest.delete(seed);
    const comp = new Set([seed]);
    const q = [seed];
    let qh = 0;
    while (qh < q.length) {
      const cur = q[qh++];
      const cx = cur % W, cy = (cur / W) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DIRS[d].dx, ny = cy + DIRS[d].dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        const n = ny * W + nx;
        if (rest.has(n)) { rest.delete(n); comp.add(n); q.push(n); }
      }
    }
    const rBoxes = boxes.filter((b) => comp.has(b));
    const rGoals = goals.filter((g) => comp.has(g));
    const entries = [];
    for (const hCell of hallway) {
      const hx = hCell % W, hyy = (hCell / W) | 0;
      for (let d = 0; d < 4; d++) {
        const n = (hyy + DIRS[d].dy) * W + (hx + DIRS[d].dx);
        if (comp.has(n)) { entries.push(hCell); break; }
      }
    }
    rooms.push({ comp, boxes: rBoxes, goals: rGoals, entries });
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length || entries.length !== 1) {
      decompOk = false;
    }
  }
  if (rooms.length === 0) decompOk = false;

  if (decompOk) {
    rooms.sort((a, b) => a.entries[0] - b.entries[0]);
    // Global state for walk assembly.
    const curBoxes = new Set(boxes);
    let curPlayer = player;
    let out = "";
    let totalPushes = 0;
    let failed = false;

    const walkTo = (target) => {
      const reach = reachable(W, floor, curBoxes, curPlayer);
      const s = walkString(reach, curPlayer, target);
      if (s === null) return false;
      out += s;
      curPlayer = target;
      return true;
    };

    for (const room of rooms) {
      const entry = room.entries[0];
      if (!walkTo(entry)) { failed = true; break; }
      const allowed = new Uint8Array(W * H);
      for (const c of room.comp) allowed[c] = 1;
      allowed[entry] = 1;
      const sol = solvePushIDA(W, H, allowed, room.boxes.slice(), room.goals.slice(), entry, entry, 4000000);
      if (!sol) { failed = true; break; }
      out += sol.moves;
      totalPushes += sol.pushes;
      for (const b of room.boxes) curBoxes.delete(b);
      for (const g of room.goals) curBoxes.add(g);
      curPlayer = sol.finalPlayer;
      if (!walkTo(entry)) { failed = true; break; }
    }
    if (!failed) {
      // Sanity: every box on a goal.
      let ok = curBoxes.size === goalSetAll.size;
      if (ok) for (const b of curBoxes) if (!goalSetAll.has(b)) { ok = false; break; }
      if (ok) return out;
    }
  }

  // Fallback: one global IDA* over the whole board.
  const sol = solvePushIDA(W, H, floor, boxes.slice(), goals.slice(), player, -1, 8000000);
  return sol ? sol.moves : "";
}

// ---------- main ----------

function main() {
  const inputDir = process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outputDir = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  let ids = [];
  try {
    ids = readdirSync(inputDir, { withFileTypes: true })
      .filter((e) => e.isDirectory())
      .map((e) => e.name)
      .sort();
  } catch {
    ids = [];
  }
  mkdirSync(outputDir, { recursive: true });
  for (const id of ids) {
    let moves = "";
    try {
      const text = readFileSync(join(inputDir, id, "level.txt"), "utf8");
      moves = solveLevel(parseLevel(text));
    } catch {
      moves = "";
    }
    const dir = join(outputDir, id);
    mkdirSync(dir, { recursive: true });
    writeFileSync(join(dir, "solution.txt"), moves + "\n");
  }
}

main();
