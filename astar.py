import heapq
import math
import cv2
from grid_generator import generate_grid

# ── A* ALGORITHM ───────────────────────────────────────────────────────────
def heuristic(a, b):
    # Octile distance for 8-directional grid
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)

def astar(grid, start, goal, dist_transform=None, wall_weight=3.0):
    """
    8-directional A*.  Diagonal moves are blocked if either adjacent cardinal
    cell is a wall (no corner-cutting).  When dist_transform is supplied,
    a wall_weight/distance penalty is added so the planner prefers corridor
    centres over wall-hugging routes.
    """
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    SQRT2 = math.sqrt(2)

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            r, c = current[0] + dr, current[1] + dc
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            if grid[r][c] == 1:
                continue
            # Prevent corner-cutting through diagonal wall gaps
            if dr != 0 and dc != 0:
                if grid[current[0] + dr][current[1]] == 1 or \
                   grid[current[0]][current[1] + dc] == 1:
                    continue

            is_diag = (dr != 0 and dc != 0)
            step_cost = SQRT2 if is_diag else 1.0
            if dist_transform is not None:
                d = float(dist_transform[r, c])
                if d > 0:
                    step_cost += wall_weight / d

            tentative_g = g_score[current] + step_cost
            neighbor = (r, c)
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
    return []

# ── PATH SMOOTHING ─────────────────────────────────────────────────────────
def smooth_path(path, grid):
    """
    Remove unnecessary waypoints.
    Only keeps points where direction changes — cleaner commands.
    """
    if len(path) < 3:
        return path

    def line_of_sight(grid, p1, p2):
        r1, c1 = p1
        r2, c2 = p2
        steps = max(abs(r2-r1), abs(c2-c1))
        if steps == 0:
            return True
        for i in range(steps + 1):
            t = i / steps
            r = int(round(r1 + t * (r2 - r1)))
            c = int(round(c1 + t * (c2 - c1)))
            if grid[r][c] == 1:
                return False
        return True

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(grid, path[i], path[j]):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

# ── PATH TO COMMANDS ───────────────────────────────────────────────────────
def path_to_commands(path, cell_size_cm=10, initial_heading=0,
                     real_cm_per_cell=None):
    """
    Convert a path (raw A* or smoothed waypoints) to robot motion commands.
    Handles both unit steps (dr,dc ∈ {-1,0,1}) and multi-cell segments from
    smooth_path by snapping direction to the nearest 45° and computing actual
    Euclidean distance.  Heading in degrees CW from North (0=N, 90=E, …).
    Returns e.g. ['R90', 'F190', 'R45', 'F85'].
    """
    if len(path) < 2:
        return []

    cm_per_cell = real_cm_per_cell if real_cm_per_cell is not None \
                  else float(cell_size_cm)
    SQRT2 = math.sqrt(2)

    UNIT_DIR = {
        (-1, 0): 0,   (-1, 1): 45,  (0,  1): 90,  (1,  1): 135,
        (1,  0): 180, (1, -1): 225, (0, -1): 270, (-1,-1): 315,
    }

    def _deg_and_dist(dr, dc):
        exact = UNIT_DIR.get((dr, dc))
        if exact is not None:
            is_diag = (dr != 0 and dc != 0)
            return exact, cm_per_cell * (SQRT2 if is_diag else 1.0)
        # Multi-cell segment: compute heading via atan2 then snap to 45°
        # heading = (90 - atan2_math) % 360  where math uses +x=E, +y=N
        angle_math = math.degrees(math.atan2(-dr, dc))
        deg = round(((90 - angle_math) % 360) / 45) * 45 % 360
        dist = math.sqrt(dr * dr + dc * dc) * cm_per_cell
        return int(deg), dist

    commands = []
    current_heading = int(initial_heading) % 360
    forward_dist = 0.0

    for i in range(1, len(path)):
        dr = path[i][0] - path[i-1][0]
        dc = path[i][1] - path[i-1][1]
        if dr == 0 and dc == 0:
            continue

        move_deg, step_dist = _deg_and_dist(dr, dc)

        if move_deg != current_heading:
            if forward_dist > 0:
                commands.append(f"F{round(forward_dist * 2) / 2:g}")
                forward_dist = 0.0
            diff = (move_deg - current_heading) % 360
            if diff == 180:
                commands.append("R180")
            elif diff < 180:
                commands.append(f"R{diff}")
            else:
                commands.append(f"L{360 - diff}")
            current_heading = move_deg

        forward_dist += step_dist

    if forward_dist > 0:
        commands.append(f"F{round(forward_dist * 2) / 2:g}")

    return commands

# ── VISUALISE PATH ─────────────────────────────────────────────────────────
def visualise_path(grid, path, smoothed_path,
                   start, goal, cell_size=10,
                   original_path="test_blueprint.jpg",
                   output_path="path_result.jpg"):
    img = cv2.imread(original_path)
    overlay = img.copy()

    # Wall cells in red
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r][c] == 1:
                x1, y1 = c*cell_size, r*cell_size
                cv2.rectangle(overlay, (x1,y1),
                              (x1+cell_size, y1+cell_size),
                              (0, 0, 180), -1)
    img = cv2.addWeighted(overlay, 0.25, img, 0.75, 0)

    # Raw path — blue dots
    for (r, c) in path:
        cx = c*cell_size + cell_size//2
        cy = r*cell_size + cell_size//2
        cv2.circle(img, (cx, cy), 2, (255, 100, 0), -1)

    # Smoothed path — green line
    for i in range(len(smoothed_path) - 1):
        r1, c1 = smoothed_path[i]
        r2, c2 = smoothed_path[i+1]
        p1 = (c1*cell_size + cell_size//2, r1*cell_size + cell_size//2)
        p2 = (c2*cell_size + cell_size//2, r2*cell_size + cell_size//2)
        cv2.line(img, p1, p2, (0, 200, 0), 2)

    # Waypoints — cyan dots
    for (r, c) in smoothed_path:
        cx = c*cell_size + cell_size//2
        cy = r*cell_size + cell_size//2
        cv2.circle(img, (cx, cy), 5, (0, 220, 220), -1)

    # Start — green, Goal — red
    sr, sc = start
    gr, gc = goal
    cv2.circle(img,
               (sc*cell_size+cell_size//2, sr*cell_size+cell_size//2),
               8, (0, 255, 0), -1)
    cv2.circle(img,
               (gc*cell_size+cell_size//2, gr*cell_size+cell_size//2),
               8, (0, 0, 255), -1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "START",
                (sc*cell_size, sr*cell_size - 5),
                font, 0.5, (0,200,0), 1)
    cv2.putText(img, "GOAL",
                (gc*cell_size, gr*cell_size - 5),
                font, 0.5, (0,0,255), 1)

    cv2.imwrite(output_path, img)
    print(f"   Path saved -> {output_path}")

# ── RUN ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    grid, gw, gh, real_cm_per_cell, cell_size_px, _ = generate_grid("test_blueprint.jpg", cell_size=10)

    # Find doorway columns — where wall rows are free
    print("\nFinding doorway columns...")
    top_cols = [c for c in range(5, 95)
                if grid[28][c]==0 and grid[29][c]==0 and grid[30][c]==0]
    bot_cols = [c for c in range(5, 95)
                if grid[38][c]==0 and grid[39][c]==0 and grid[40][c]==0]
    print(f"Top doorway cols: {top_cols[:10]}")
    print(f"Bot doorway cols: {bot_cols[:10]}")

    if not top_cols or not bot_cols:
        print("❌ No doorway columns found — check threshold in grid_generator.py")
        exit()

    # Pick centre doorway column for each room pair
    # Reception ↔ ICU: left third of blueprint (cols ~10-33)
    # Ward 1 ↔ OT:     middle third (cols ~34-65)
    # Ward 2 ↔ Pharmacy: right third (cols ~66-95)

    room_pairs = [
        ("Reception -> ICU",   (25, 17), (44, 17)),
        ("Ward 1 -> OT",       (25, 50), (44, 50)),
        ("Ward 2 -> Pharmacy", (25, 80), (44, 80)),
    ]

    for label, start, goal in room_pairs:
        print(f"\n{'='*45}")
        print(f"🔍 {label}: {start} -> {goal}")

        # Snap start/goal to nearest doorway column
        # Find closest top doorway col to start col
        best_top = min(top_cols, key=lambda c: abs(c - start[1]))
        best_bot = min(bot_cols, key=lambda c: abs(c - goal[1]))
        start = (start[0], best_top)
        goal  = (goal[0], best_bot)
        print(f"   Snapped to doorway: {start} -> {goal}")

        path = astar(grid, start, goal)

        if not path:
            print(f"   ❌ No path found")
            continue

        smoothed = smooth_path(path, grid)
        print(f"   ✅ Raw path: {len(path)} cells")
        print(f"   ✅ Smoothed: {len(smoothed)} waypoints")

        # Use RAW path for commands — more reliable step-by-step
        commands = path_to_commands(path, cell_size_cm=10, initial_heading='N',
                                    real_cm_per_cell=real_cm_per_cell)
        print(f"\n   🤖 Motion commands:")
        if commands:
            for cmd in commands:
                print(f"      {cmd}")
        else:
            print("      (no commands generated)")

        # Save path image for first pair only
        if label == "Reception -> ICU":
            visualise_path(grid, path, smoothed, start, goal,
                           cell_size=10,
                           original_path="test_blueprint.jpg",
                           output_path="path_result.jpg")

    print("\n✅ A* complete! Open path_result.jpg to see the path.")