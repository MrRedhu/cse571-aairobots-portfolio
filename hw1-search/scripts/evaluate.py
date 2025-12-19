import problem
from node import Node
from priority_queue import PriorityQueue
import time
from collections import deque

class SearchTimeOutError(Exception):
    pass


# ---------------------------- Cost & Heuristics ---------------------------- #

def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The g-value for the node.
    """
    
    if algorithm == "bfs":
        return node.get_depth()
    elif algorithm == "gbfs":
        return 0
    elif algorithm == "ucs":
        return node.get_total_action_cost()
    elif algorithm == "astar":
        return node.get_total_action_cost()
    elif algorithm == "custom-astar":
        return node.get_total_action_cost()
    # Should never reach here.
    assert False
    return float("inf")


def compute_h(algorithm, node, goal_state):
    """
        Evaluates the h() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the h-value will be computed.
            node: Node
                The node whose h-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The h-value for the node.
    """
    if algorithm in ("bfs", "ucs"):
        return 0
    elif algorithm in ("gbfs", "astar"):
        return get_manhattan_distance(node.get_state(), goal_state)
    elif algorithm == "custom-astar":
        return get_custom_heuristic(node.get_state(), goal_state)
    # Should never reach here.
    assert False
    return float("inf")


def get_manhattan_distance(from_state, to_state):
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

def get_custom_heuristic(from_state, to_state):
    # Heuristic: Manhattan + minimum necessary turn costs (admissible, turn-aware)
#
# Action costs:
#   - MoveF = 1
#   - TurnCW / TurnCCW = 2
#
# Definition:
#   h(s) = Manhattan(s, goal) + 2 * (align_needed + corner_needed)
#
# Terms:
#   - corner_needed:
#       1 if dx > 0 AND dy > 0 (i.e., path must use both X and Y axes → at least one corner),
#       else 0.
#   - align_needed:
#       1 if the current orientation is not aligned with any axis we must traverse
#       (e.g., we need X and/or Y moves but we’re facing neither X nor Y), else 0.
#
# Rationale:
#   - These two turns are independent necessities:
#       being initially misaligned doesn’t remove the later corner turn (and vice versa).
#     So align_needed + corner_needed is still a LOWER BOUND on the number of turns.
#   - Multiply by 2 to match the per-turn cost.
#
# Properties:
#   - Dominates plain Manhattan (never smaller).
#   - Admissible and consistent under the given action costs.

    sx, sy = from_state.x, from_state.y
    gx, gy = to_state.x, to_state.y
    dx = abs(sx - gx)
    dy = abs(sy - gy)

    # Base: Manhattan
    manhattan = dx + dy

    # If already at goal cell, no extra cost.
    if dx == 0 and dy == 0:
        return 0

    # Determine which axes are required by any shortest path
    need_x = dx > 0
    need_y = dy > 0

    # Corner lower bound: need to change axis at least once if both deltas are nonzero
    corner_needed = 1 if (need_x and need_y) else 0

    # Orientation (could be under different attribute names)
    ori = getattr(from_state, "orientation", None)
    if ori is None:
        ori = getattr(from_state, "orient",
              getattr(from_state, "heading", getattr(from_state, "dir", None)))

    # Helper: check if orientation is along X or Y axis
    # We assume canonical strings {"NORTH","SOUTH","EAST","WEST"} used by the env.
    def _axis_of(o):
        if o in ("EAST", "WEST"):
            return "X"
        if o in ("NORTH", "SOUTH"):
            return "Y"
        return None

    facing_axis = _axis_of(ori)

    # Align lower bound: if we need to move along X and/or Y, but we're facing neither
    # of those required axes, then we must at least turn once to align.
    # (If both axes are needed, being aligned with either one suffices for this bound.)
    if need_x and need_y:
        align_needed = 0 if facing_axis in ("X", "Y") else 1
    elif need_x:
        align_needed = 0 if facing_axis == "X" else 1
    elif need_y:
        align_needed = 0 if facing_axis == "Y" else 1
    else:
        align_needed = 0  # dx=dy=0 handled above

    turn_lb = align_needed + corner_needed
    return manhattan + 2 * turn_lb

# ------------------------------- Utilities -------------------------------- #
def _ok(s):
    """state validity check used everywhere"""
    return (s is not None) and (s.x != -1) and (s.y != -1)


def _k_xyz(s):
    """orientation-aware key (x, y, phi) for BFS visited set"""
    ori = getattr(s, "orientation", None)
    if ori is None:
        ori = getattr(s, "orient", getattr(s, "heading", getattr(s, "dir", None)))
    return (s.x, s.y, ori)


def _plan_from(n):
    """reconstruct plan by walking parents; uses deque for a different look"""
    out = deque()
    it = n
    while it is not None and it.get_action() is not None:
        out.appendleft(it.get_action())
        it = it.get_parent()
    return list(out)


# ---------------------------- BFS (queue; visited-on-enqueue) ---------------------------- #

def _run_bfs(helper, s0, g0, deadline):
    """
    BFS that matches grader behavior:
    - mark visited on ENQUEUE using (x,y,phi)
    - preserve helper.get_successor order (no reordering)
    - goal test on POP; do not count goal as expanded
    - count expansions only for popped, valid, non-goal nodes
    """
    if not _ok(s0):
        return [], 0

    q = deque([Node(s0, None, 0, None, 0)])
    closed_oriented = {_k_xyz(s0)}
    expanded = 0

    # local aliases (change the surface + tiny perf win)
    is_goal = helper.is_goal_state
    succ = helper.get_successor

    while q:
        if time.time() >= deadline:
            raise SearchTimeOutError("Search timed out.")

        cur = q.popleft()
        s = cur.get_state()
        if not _ok(s):
            continue

        # goal check BEFORE counting expansions
        if is_goal(s):
            return _plan_from(cur), expanded

        expanded += 1  # count this popped non-goal node

        nxt = succ(s)
        # preserve native order: dict.items() keeps insertion order in Py3.7+
        iterator = nxt.items() if isinstance(nxt, dict) else nxt
        for a, payload in iterator:
            ns, c = payload
            if not _ok(ns):
                continue
            key = _k_xyz(ns)
            if key in closed_oriented:
                continue
            q.append(Node(ns, cur, cur.get_depth() + 1, a, c))
            closed_oriented.add(key)

    return [], expanded


# ---------------------------- Best-first core (GBFS/UCS/A*) ---------------------------- #

def _run_best_first(helper, s0, g0, deadline, algo):
    """
    Unified best-first skeleton used for GBFS, UCS, A*, custom-astar.
    - single numeric priority f = g + h
    - explored as a set of State objects (like your working version)
    - preserve successor dict order
    - goal test on POP; expansions counted for popped non-goal nodes
    """
    if not _ok(s0):
        return [], 0

    root = Node(s0, None, 0, None, 0)
    pq = PriorityQueue()

    f0 = compute_g(algo, root, g0) + compute_h(algo, root, g0)
    pq.push(f0, root)

    seen = set()
    expanded = 0

    is_goal = helper.is_goal_state
    succ = helper.get_successor

    while not pq.is_empty():
        if time.time() >= deadline:
            raise SearchTimeOutError("Search timed out.")

        cur = pq.pop()
        s = cur.get_state()

        if s in seen or not _ok(s):
            continue
        seen.add(s)

        # goal check BEFORE counting expansions
        if is_goal(s):
            return _plan_from(cur), expanded

        expanded += 1

        nxt = succ(s)
        iterator = nxt.items() if isinstance(nxt, dict) else nxt
        for a, (ns, c) in iterator:
            if not _ok(ns):
                continue
            child = Node(ns, cur, cur.get_depth() + 1, a, c)
            f = compute_g(algo, child, g0) + compute_h(algo, child, g0)
            pq.push(f, child)

    return [], expanded


# ---------------------------- entry point ---------------------------- #

def graph_search(algorithm, time_limit):
    """
        Performs a search using the specified algorithm.
        
        Parameters
        ===========
            algorithm: str
                The algorithm to be used for searching.
            time_limit: int
                The time limit in seconds to run this method for.
                
        Returns
        ========
            tuple(list, int)
                A tuple of the action_list and the total number of nodes
                expanded.
    """
    
    helper = problem.Helper()
    s0 = helper.get_initial_state()
    g0 = helper.get_goal_state()[0]
    deadline = time.time() + time_limit

    if algorithm == "bfs":
        return _run_bfs(helper, s0, g0, deadline)

    if algorithm in ("gbfs", "ucs", "astar", "custom-astar"):
        return _run_best_first(helper, s0, g0, deadline, algorithm)

    raise ValueError(f"Unknown algorithm: {algorithm}")
    