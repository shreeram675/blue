from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from blueprint_parser import parse_blueprint, estimate_scale_from_room_positions
from grid_generator import generate_grid, find_narrow_passages
from astar import astar, path_to_commands
from flask import send_file
from io import BytesIO
from firebase_queue import firebase_queue
import os
import uuid
import numpy as np
import cv2
import threading
import time

ROBOT_ID = "wheelchair_01"

app = Flask(__name__)
CORS(app, origins="*")
UPLOAD_FOLDER = "uploads"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# In-memory store
state = {
    "raw_grid": None,
    "inflated_grid": None,
    "rooms": [],
    "orientation": None,
    "parse_status": None,
    "robot_pose": {"row": 0, "col": 0, "heading": 0},
    "command_queue": [],
    "all_commands": [],
    "nav_start_pose": None,
    "blueprint_path": None,
    "grid_size": None,
    "real_cm_per_cell": 10.0,
    "cell_size_px": 10,
    "robot_params": {"robot_width_cm": 60.0, "safety_margin_cm": 10.0},
    "robot_id": ROBOT_ID,
    "last_synced_seq": 0,   # highest command sequence confirmed DONE via Firebase
}

# ── FIREBASE POSE SYNC (background thread) ────────────────────────────────
# Watches /robots/<id>/queue every 1.5 s for DONE commands and replays them
# to keep state["robot_pose"] current without needing the ESP32 on the same LAN.
def _sync_pose_from_firebase():
    if not firebase_queue.enabled:
        return
    if not state.get("nav_start_pose") or not state.get("all_commands"):
        return
    try:
        queue_data = firebase_queue._db.reference(
            f"/robots/{ROBOT_ID}/queue"
        ).get()
    except Exception:
        return
    if not queue_data:
        return

    # Walk cmd_1, cmd_2, … in order; stop at first non-DONE
    done_seq = 0
    for i in range(1, len(state["all_commands"]) + 1):
        item = queue_data.get(f"cmd_{i}")
        if item and item.get("status") == "DONE":
            done_seq = i
        else:
            break

    if done_seq <= state["last_synced_seq"]:
        return

    sp = state["nav_start_pose"]
    new_r, new_c, new_h = _replay_commands(
        sp["row"], sp["col"], sp["heading"],
        state["all_commands"][:done_seq],
        state["real_cm_per_cell"],
    )
    state["robot_pose"]    = {"row": new_r, "col": new_c, "heading": new_h}
    state["last_synced_seq"] = done_seq
    print(f"   Firebase sync: cmd_{done_seq} DONE → pose ({new_r},{new_c}) h={new_h}°")
    try:
        firebase_queue.publish_status(ROBOT_ID, state["robot_pose"])
    except Exception:
        pass


def _firebase_sync_loop():
    while True:
        try:
            _sync_pose_from_firebase()
        except Exception as e:
            print(f"   Firebase sync error: {e}")
        time.sleep(1.5)


@app.route("/")
def index():
    return render_template("index.html")

@app.route("/favicon.ico")
def favicon():
    return "", 204

# ── ROUTE 1: Upload blueprint ──────────────────────────────────────────────
@app.route("/upload", methods=["POST"])
def upload():
    try:
        if "blueprint" not in request.files:
            return jsonify({"error": "No file uploaded"}), 400

        file = request.files["blueprint"]
        filename = f"{uuid.uuid4().hex}.jpg"
        filepath = os.path.join(UPLOAD_FOLDER, filename)
        file.save(filepath)
        print(f" File saved: {filepath}")

        print(" Starting OCR parse...")
        result = parse_blueprint(filepath)
        print(f" Parse done: {result['parse_status']}")

        unit            = request.form.get("unit", "cm")
        wall_thickness  = round(float(request.form.get("wall_thickness", 4)))
        robot_width_cm  = float(request.form.get("robot_width", 60.0))
        robot_length_cm = float(request.form.get("robot_length", robot_width_cm))
        safety_margin   = float(request.form.get("safety_margin", 10.0))

        # Auto-estimate scale from room label positions × real dimensions
        rooms_parsed = result.get("rooms", [])
        cm_per_pixel = estimate_scale_from_room_positions(rooms_parsed, unit)
        if cm_per_pixel:
            print(f"   Auto-scale: {cm_per_pixel:.4f} cm/px (from room positions)")
        else:
            print("   Auto-scale: not enough room dimension data, using default cell size")

        print("Generating grid...")
        raw_grid, inflated_grid, gw, gh, real_cm_per_cell, cell_size_px, scale_applied = generate_grid(
            filepath, cell_size=10,
            wall_thickness=wall_thickness,
            unit=unit,
            robot_width_cm=robot_width_cm,
            robot_length_cm=robot_length_cm,
            safety_margin_cm=safety_margin,
            cm_per_pixel=cm_per_pixel,
        )
        print(f"Grid done: {gw}x{gh}, cell={cell_size_px}px, {real_cm_per_cell:.2f} cm/cell")

        # Detect doorways too narrow for the robot (uses raw_grid)
        narrow_passages = []
        if scale_applied:
            narrow_passages = find_narrow_passages(
                raw_grid, real_cm_per_cell, robot_width_cm, safety_margin
            )
            if narrow_passages:
                print(f"   {len(narrow_passages)} narrow passage(s) detected")

        # Compute inflation stats for UI (mirrors inflate_obstacles exactly)
        inflation_cells = max(0, round(safety_margin / real_cm_per_cell)) \
                          if real_cm_per_cell > 0 else 0

        state["raw_grid"]      = raw_grid.tolist()
        state["inflated_grid"] = inflated_grid.tolist()
        state["rooms"]         = result.get("rooms", [])
        state["orientation"]   = result.get("orientation")
        state["parse_status"]  = result.get("parse_status")
        state["blueprint_path"] = filepath
        state["grid_size"]     = {"rows": gh, "cols": gw}
        state["real_cm_per_cell"] = real_cm_per_cell
        state["cell_size_px"]  = cell_size_px
        state["robot_params"]  = {
            "robot_width_cm": robot_width_cm,
            "robot_length_cm": robot_length_cm,
            "safety_margin_cm": safety_margin,
            "inflation_cells": inflation_cells,
        }
        # Clear any stale commands from a previous navigation
        state["command_queue"] = []
        state["all_commands"]  = []

        # ── Firebase: reset robot state on new blueprint ──────────────────
        try:
            firebase_queue.cancel_queue(ROBOT_ID)
            firebase_queue.publish_status(ROBOT_ID, {
                "row": None, "col": None, "heading": None
            })
        except Exception as fb_err:
            print(f"     Firebase reset failed: {fb_err}")

        return jsonify({
            "parse_status":           result["parse_status"],
            "orientation":            result.get("orientation"),
            "rooms":                  result.get("rooms", []),
            "missing_fields":         result.get("missing_fields", []),
            "grid_size":              {"rows": gh, "cols": gw},
            "real_cm_per_cell":       real_cm_per_cell,
            "cell_size_px":           cell_size_px,
            "inflation_cells":        inflation_cells,
            "scale_cm_per_pixel":     cm_per_pixel,
            "narrow_passages":        narrow_passages,
            "robot_width_cm":         robot_width_cm,
            "robot_length_cm":        robot_length_cm,
            "has_grid":               True,
            "raw_grid":               state["raw_grid"],
            "inflated_grid":          state["inflated_grid"],
        })

    except Exception as e:
        import traceback
        print(" ERROR in /upload:")
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

# ── ROUTE 2: Check if cell is free (for robot placement) ──────────────────
def _snap_to_free(grid, row, col, max_radius=20):
    """Return nearest free cell (row, col) on grid, or None if not found."""
    rows, cols = grid.shape
    if 0 <= row < rows and 0 <= col < cols and grid[row, col] == 0:
        return row, col
    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                    return nr, nc
    return None


def _blocked_doorway_hint(raw_grid, start, goal):
    """
    Walk the straight line from start to goal in grid space and find the
    midpoint of the longest continuous wall run — that is the most likely
    blocked doorway.  Returns (row, col) or None.
    """
    r1, c1 = start
    r2, c2 = goal
    steps = max(abs(r2 - r1), abs(c2 - c1))
    if steps == 0:
        return None

    runs, in_wall, run_start = [], False, None
    for i in range(steps + 1):
        t = i / steps
        r = int(round(r1 + t * (r2 - r1)))
        c = int(round(c1 + t * (c2 - c1)))
        if 0 <= r < raw_grid.shape[0] and 0 <= c < raw_grid.shape[1]:
            if raw_grid[r, c] == 1:
                if not in_wall:
                    in_wall, run_start = True, (r, c)
            else:
                if in_wall:
                    runs.append((run_start, (r, c)))
                    in_wall = False
    if in_wall and run_start:
        runs.append((run_start, (r2, c2)))
    if not runs:
        return None
    r0, c0 = max(runs, key=lambda x: abs(x[1][0]-x[0][0]) + abs(x[1][1]-x[0][1]))[0]
    r1_, c1_ = max(runs, key=lambda x: abs(x[1][0]-x[0][0]) + abs(x[1][1]-x[0][1]))[1]
    return (r0 + r1_) // 2, (c0 + c1_) // 2


@app.route("/is-free")
def is_free():
    try:
        row = int(request.args.get("row", 0))
        col = int(request.args.get("col", 0))
        
        if not state["inflated_grid"]:
            return jsonify({"free": False, "reason": "No grid loaded"})

        rows = len(state["inflated_grid"])
        cols = len(state["inflated_grid"][0]) if rows > 0 else 0

        if not (0 <= row < rows and 0 <= col < cols):
            return jsonify({"free": False, "reason": "Out of bounds"})

        is_free_cell = state["inflated_grid"][row][col] == 0
        return jsonify({
            "free": is_free_cell,
            "reason": "Obstacle" if not is_free_cell else None
        })
    except Exception as e:
        return jsonify({"free": False, "reason": str(e)}), 500

# ── ROUTE 3: Set robot start pose (with validation) ───────────────────────
@app.route("/set-pose", methods=["POST"])
def set_pose():
    try:
        data = request.json
        row     = int(data.get("row", 0))
        col     = int(data.get("col", 0))
        heading = int(data.get("heading", 0)) % 360
        
        # Validate against inflated_grid — path planning runs on it, so the
        # start must be free there too, not just in the raw grid.
        if not state["inflated_grid"]:
            return jsonify({"ok": False, "error": "No grid loaded"}), 400

        rows = len(state["inflated_grid"])
        cols = len(state["inflated_grid"][0]) if rows > 0 else 0

        if not (0 <= row < rows and 0 <= col < cols):
            return jsonify({"ok": False, "error": "Position out of grid bounds"}), 400

        if state["inflated_grid"][row][col] == 1:
            # Snap to nearest free cell (3-cell radius)
            found = False
            for radius in range(1, 4):
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr, nc = row + dr, col + dc
                        if 0 <= nr < rows and 0 <= nc < cols and state["inflated_grid"][nr][nc] == 0:
                            row, col = nr, nc
                            found = True
                            break
                    if found: break
                if found: break
            
            if not found:
                return jsonify({"ok": False, "error": "No free cell nearby - too close to walls"}), 400
            
            print(f"Snapped pose from obstacle ({data['row']},{data['col']}) to free ({row},{col})")
        
        state["robot_pose"] = {"row": row, "col": col, "heading": heading}

        # ── Firebase: sync pose ───────────────────────────────────────────
        try:
            firebase_queue.publish_status(ROBOT_ID, {
                "row": row, "col": col, "heading": heading
            })
        except Exception as fb_err:
            print(f"     Firebase pose sync failed: {fb_err}")

        return jsonify({"ok": True, "pose": state["robot_pose"], "snapped": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

# ── ROUTE 4: Navigate to a room ───────────────────────────────────────────
@app.route("/navigate", methods=["POST"])
def navigate():
    try:
        if state["inflated_grid"] is None:
            return jsonify({"error": "No blueprint loaded. Call /upload first."}), 400

        data        = request.json
        target_room = data.get("room", "").strip().lower()

        grid_np = np.array(state["inflated_grid"], dtype=np.uint8)

        # ── Room matching: exact first, then partial ──────────────────────
        matched = None
        for room in state["rooms"]:
            if room["name"].lower() == target_room:
                matched = room
                break
        if not matched:
            for room in state["rooms"]:
                if target_room in room["name"].lower() or \
                   room["name"].lower() in target_room:
                    matched = room
                    break

        if not matched:
            available = [r["name"] for r in state["rooms"]]
            return jsonify({
                "error": f"Room '{target_room}' not found",
                "available_rooms": available
            }), 404

        # ── Pixel -> grid cell ─────────────────────────────────────────────
        cell_size = state.get("cell_size_px", 10)
        rx, ry    = matched["center"]
        goal_col  = rx // cell_size
        goal_row  = ry // cell_size

        # Snap goal to nearest free cell
        if grid_np[goal_row][goal_col] == 1:
            found = False
            for radius in range(1, 20):
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr, nc = goal_row + dr, goal_col + dc
                        if 0 <= nr < grid_np.shape[0] and \
                           0 <= nc < grid_np.shape[1] and \
                           grid_np[nr][nc] == 0:
                            goal_row, goal_col = nr, nc
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            print(f"   Snapped goal to free cell: ({goal_row}, {goal_col})")

        start_row = state["robot_pose"]["row"]
        start_col = state["robot_pose"]["col"]
        heading   = state["robot_pose"]["heading"]

        # Snap start to nearest free cell
        if grid_np[start_row][start_col] == 1:
            found = False
            for radius in range(1, 20):
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nr, nc = start_row + dr, start_col + dc
                        if 0 <= nr < grid_np.shape[0] and \
                           0 <= nc < grid_np.shape[1] and \
                           grid_np[nr][nc] == 0:
                            start_row, start_col = nr, nc
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            print(f"   Snapped start to free cell: ({start_row}, {start_col})")

        start = (start_row, start_col)
        goal  = (goal_row, goal_col)

        print(f" Navigating to {matched['name']}: {start} -> {goal}")

        raw_np = np.array(state["raw_grid"], dtype=np.uint8)

        def _dist(g):
            return cv2.distanceTransform((g == 0).astype(np.uint8), cv2.DIST_L2, 5)

        # ── Level 1: inflated grid (full safety margins) ──────────────────
        path = astar(grid_np, start, goal, dist_transform=_dist(grid_np), wall_weight=1.0)
        fallback_used = None

        # ── Level 2: raw grid (margins stripped, doorways maximally open) ─
        if not path:
            print("   L1 failed -> trying raw grid (no inflation)")
            raw_start = _snap_to_free(raw_np, start[0], start[1]) or start
            raw_goal  = _snap_to_free(raw_np, goal[0],  goal[1])  or goal
            path = astar(raw_np, raw_start, raw_goal, dist_transform=_dist(raw_np), wall_weight=1.0)
            if path:
                fallback_used = "raw_grid"
                grid_np = raw_np          # use raw grid for smooth/commands
                start, goal = raw_start, raw_goal
                print("   L2 succeeded on raw grid")

        # ── Level 3: eroded walls (bridges 1-cell doorway gaps from noise) ─
        if not path:
            print("   L2 failed -> trying wall-eroded grid (opening narrow doorways)")
            opened = cv2.erode(raw_np, np.ones((3, 3), np.uint8))
            open_start = _snap_to_free(opened, start[0], start[1]) or start
            open_goal  = _snap_to_free(opened, goal[0],  goal[1])  or goal
            path = astar(opened, open_start, open_goal, dist_transform=_dist(opened), wall_weight=1.0)
            if path:
                fallback_used = "opened_doorways"
                grid_np = opened
                start, goal = open_start, open_goal
                print("   L3 succeeded on wall-eroded grid")

        # ── Level 4: report approximate blocked location ──────────────────
        if not path:
            hint = _blocked_doorway_hint(raw_np, start, goal)
            return jsonify({
                "error": (
                    f"No path to '{matched['name']}'. "
                    "All doorways on this route appear blocked — check wall "
                    "detection or reduce safety margin."
                ),
                "blocked_near": list(hint) if hint else None,
            }), 400

        commands = path_to_commands(
            path,
            cell_size_cm=10,
            initial_heading=heading,
            real_cm_per_cell=state["real_cm_per_cell"],
        )

        state["command_queue"]   = list(commands)
        state["all_commands"]    = list(commands)
        state["current_path"]    = path
        state["nav_start_pose"]  = dict(state["robot_pose"])
        state["last_synced_seq"] = 0   # reset so sync thread re-scans from cmd_1

        # ── Firebase: publish command queue ───────────────────────────────
        try:
            firebase_queue.publish_queue(ROBOT_ID, commands)
        except Exception as fb_err:
            print(f"     Firebase publish failed: {fb_err}")

        print(f" Path found: {len(path)} cells, {len(commands)} commands")

        return jsonify({
            "target_room":  matched["name"],
            "start":        list(start),
            "goal":         list(goal),
            "path_cells":   len(path),
            "path":         [list(p) for p in path],
            "commands":     commands,
            "has_path":     True,
            "fallback_used": fallback_used,
        })

    except Exception as e:
        import traceback
        print(" ERROR in /navigate:")
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

# ── ROUTE 5: Get current status ───────────────────────────────────────────
@app.route("/status", methods=["GET"])
def status():
    raw_free_pct = 100 * np.sum(np.array(state.get("raw_grid", [])) == 0) / (state.get("grid_size", {}).get("rows", 1) * state.get("grid_size", {}).get("cols", 1)) if state.get("raw_grid") else 0
    
    return jsonify({
        "parse_status":     state["parse_status"],
        "robot_pose":       state["robot_pose"],
        "rooms_detected":   len(state["rooms"]),
        "command_queue":    state["command_queue"],
        "grid_loaded":      state["raw_grid"] is not None,
        "real_cm_per_cell": state["real_cm_per_cell"],
        "inflation_cells":  state["robot_params"].get("inflation_cells", 0),
        "raw_free_pct":     round(raw_free_pct, 1),
        "robot_params":     state["robot_params"],
        "firebase_enabled": firebase_queue.enabled,
    })

# ── ROUTE 6: ESP32 polls for next command ─────────────────────────────────
@app.route("/get-command", methods=["GET"])
def get_command():
    if state["command_queue"]:
        cmd = state["command_queue"].pop(0)
        seq = len(state["all_commands"]) - len(state["command_queue"])
        return jsonify({
            "command":   cmd,
            "sequence":  seq,
            "remaining": len(state["command_queue"]),
        })
    return jsonify({"command": "DONE", "sequence": 0, "remaining": 0})

# ── ROUTE 7: Grid overlay with highlighted inflated obstacles ─────────────
@app.route("/grid.png")
def grid_overlay():
    if not state["raw_grid"] or not state["blueprint_path"]:
        return "", 204
    
    blueprint = cv2.imread(state["blueprint_path"])
    if blueprint is None:
        return "", 204
    
    raw_grid_np = np.array(state["raw_grid"], dtype=np.uint8)
    inflated_grid_np = np.array(state["inflated_grid"], dtype=np.uint8)
    h, w = raw_grid_np.shape
    cell_px = state["cell_size_px"]
    
    overlay = blueprint.copy()
    
    for r in range(h):
        for c in range(w):
            y1, x1 = r * cell_px, c * cell_px
            y2, x2 = y1 + cell_px, x1 + cell_px
            
            if raw_grid_np[r, c] == 1:
                # Raw walls: dark red
                overlay[y1:y2, x1:x2] = overlay[y1:y2, x1:x2] * 0.6 + np.array([0, 0, 100]) * 0.4
            elif inflated_grid_np[r, c] == 1:
                # Inflated halo: bright yellow/orange warning (robot no-go)
                overlay[y1:y2, x1:x2] = overlay[y1:y2, x1:x2] * 0.3 + np.array([0, 180, 255]) * 0.7
    
    _, buffer = cv2.imencode('.png', overlay)
    return send_file(BytesIO(buffer.tobytes()), mimetype='image/png')

def _replay_commands(start_row, start_col, start_heading, commands, real_cm_per_cell):
    """Replay commands from a start pose; return final (row, col, heading)."""
    import math as _m
    DEG_DIR = {
        0: (-1, 0), 45: (-1, 1), 90: (0, 1), 135: (1, 1),
        180: (1, 0), 225: (1, -1), 270: (0, -1), 315: (-1, -1),
    }
    SQRT2 = _m.sqrt(2)
    row, col, d = float(start_row), float(start_col), int(start_heading) % 360
    for cmd in commands:
        if cmd.startswith('R'):
            d = (d + int(cmd[1:])) % 360
        elif cmd.startswith('L'):
            d = (d - int(cmd[1:])) % 360
        elif cmd.startswith('F'):
            dist_cm = float(cmd[1:])
            dr, dc = DEG_DIR.get(d, (-1, 0))
            is_diag = (dr != 0 and dc != 0)
            cm_per_step = real_cm_per_cell * (SQRT2 if is_diag else 1.0)
            steps = round(dist_cm / cm_per_step)
            row += dr * steps
            col += dc * steps
        elif cmd.startswith('B'):
            dist_cm = float(cmd[1:])
            dr, dc = DEG_DIR.get(d, (-1, 0))
            is_diag = (dr != 0 and dc != 0)
            cm_per_step = real_cm_per_cell * (SQRT2 if is_diag else 1.0)
            steps = round(dist_cm / cm_per_step)
            row -= dr * steps
            col -= dc * steps
    return round(row), round(col), d % 360




# ── ROUTE 8: Path overlay ─────────────────────────────────────────────────
@app.route("/path.png")
def path_overlay():
    if not state["blueprint_path"]:
        return "", 204

    blueprint = cv2.imread(state["blueprint_path"])
    if blueprint is None:
        return "", 204

    img = blueprint.copy()
    cell_px = state.get("cell_size_px", 10)

    # Use the validated A* path for display — it ran on the inflated grid so
    # every cell is guaranteed to respect the robot's safety margins.
    # With 4-directional A*, the path is already axis-aligned (no diagonals).
    astar_path = state.get("current_path") or []

    if astar_path:
        robot_w_cm = state["robot_params"]["robot_width_cm"]
        real_cpc   = state["real_cm_per_cell"]
        inflated_np = np.array(state["inflated_grid"], dtype=np.uint8)
        h_cells, w_cells = inflated_np.shape

        # Mark every path cell in a cell-resolution binary mask
        path_cell_mask = np.zeros((h_cells, w_cells), dtype=np.uint8)
        for r, c in astar_path:
            if 0 <= r < h_cells and 0 <= c < w_cells:
                path_cell_mask[r, c] = 1

        # Dilate by robot half-width (in cells) to show the footprint sweep
        robot_half_cells = max(1, round((robot_w_cm / 2.0) / real_cpc))
        kern = np.ones((robot_half_cells * 2 + 1, robot_half_cells * 2 + 1), np.uint8)
        path_band = cv2.dilate(path_cell_mask, kern)

        # Shrink free mask by 1 cell so the band has a visible gap from every wall boundary
        free_mask   = (inflated_np == 0).astype(np.uint8)
        free_shrunk = cv2.erode(free_mask, np.ones((3, 3), np.uint8))
        path_band   = np.minimum(path_band, free_shrunk)
        # Always keep actual A* path cells visible even in 1-cell-wide corridors
        path_band   = np.maximum(path_band, np.minimum(path_cell_mask, free_mask))

        # Expand cell mask to pixel space and apply semi-transparent colour
        band_px = np.kron(path_band, np.ones((cell_px, cell_px), dtype=np.uint8))
        h_img, w_img = img.shape[:2]
        band_px = band_px[:h_img, :w_img]

        path_overlay_img = img.copy()
        path_overlay_img[band_px > 0] = (200, 100, 0)
        cv2.addWeighted(path_overlay_img, 0.45, img, 0.55, 0, img)

        # Dots at each A* path cell — radius ≤ cell_px//3 keeps them inside the cell,
        # so they can never bleed into adjacent wall pixels (unlike antialiased lines)
        dot_r = max(1, cell_px // 3)
        for idx, (r, c) in enumerate(astar_path):
            pt    = (c * cell_px + cell_px // 2, r * cell_px + cell_px // 2)
            color = (0, 220, 255) if idx % 10 == 0 else (255, 190, 60)
            rad   = dot_r + 1   if idx % 10 == 0 else dot_r
            cv2.circle(img, pt, rad, color, -1)

    # Robot at current pose (green)
    if "robot_pose" in state:
        row, col = state["robot_pose"]["row"], state["robot_pose"]["col"]
        draw_rotated_rect(img, row, col, state["robot_params"], cell_px, (0, 255, 0),
                          state["robot_pose"].get("heading", 0))

    # Goal marker (red) — end of the validated A* path
    if astar_path:
        goal_r, goal_c = astar_path[-1]
        draw_rotated_rect(img, goal_r, goal_c, state["robot_params"], cell_px, (0, 0, 255), 0)
        cv2.circle(img, (int(goal_c * cell_px + cell_px / 2),
                         int(goal_r * cell_px + cell_px / 2)), 8, (0, 0, 255), -1)
    
    _, buffer = cv2.imencode('.png', img)
    return send_file(BytesIO(buffer.tobytes()), mimetype='image/png')

def draw_rotated_rect(img, row, col, robot_params, cell_px, color, heading=0):
    """Draw rotated robot rect. heading = degrees CW from North (0=N, 90=E)."""
    w_cm = robot_params["robot_width_cm"]
    l_cm = robot_params.get("robot_length_cm", w_cm)
    real_cm_per_cell = state["real_cm_per_cell"]

    w_px = max(8, (w_cm / real_cm_per_cell) * cell_px)
    l_px = max(8, (l_cm / real_cm_per_cell) * cell_px)

    cx = int(col * cell_px + cell_px / 2)
    cy = int(row * cell_px + cell_px / 2)

    angle = int(heading) % 360
    if angle > 180:
        angle = angle - 360  # normalize to [-180, 180] for OpenCV boxPoints

    rect = ((cx, cy), (w_px, l_px), angle)
    box = cv2.boxPoints(rect)
    box = np.array(box, dtype=np.int32)
    cv2.drawContours(img, [box], 0, color, -1)
    cv2.drawContours(img, [box], 0, (255, 255, 255), 2)

def _make_mask_png(grid_np, cell_px):
    """RGBA PNG helper: free cells (==0) → alpha 255, wall cells → alpha 0."""
    h, w    = grid_np.shape
    free    = (grid_np == 0).astype(np.uint8)
    free_px = np.kron(free, np.ones((cell_px, cell_px), dtype=np.uint8))
    img     = np.zeros((h * cell_px, w * cell_px, 4), dtype=np.uint8)
    val     = free_px * 255
    img[:, :, 0] = val
    img[:, :, 1] = val
    img[:, :, 2] = val
    img[:, :, 3] = val
    _, buf = cv2.imencode('.png', img)
    return send_file(BytesIO(buf.tobytes()), mimetype='image/png')

@app.route("/free-mask.png")
def free_mask():
    """Inflated-grid mask — used as belt-and-suspenders for path-centre clip."""
    if not state["inflated_grid"]:
        return "", 204
    return _make_mask_png(np.array(state["inflated_grid"], dtype=np.uint8),
                          state["cell_size_px"])

@app.route("/raw-mask.png")
def raw_mask():
    """Raw-grid mask — actual structural walls only (no safety-margin inflation).
    Used to clip the robot-width corridor so it stops at real walls but is
    allowed to occupy the safety-margin buffer the robot body passes through."""
    if not state["raw_grid"]:
        return "", 204
    return _make_mask_png(np.array(state["raw_grid"], dtype=np.uint8),
                          state["cell_size_px"])


@app.route("/debug_grid.png")
def debug_grid():
    if not state["inflated_grid"] or not state["blueprint_path"]:
        return "", 204
    
    grid_np = np.array(state["inflated_grid"], dtype=np.uint8)
    h, w = grid_np.shape
    cell_px = state["cell_size_px"]
    
    img = np.zeros((h * cell_px, w * cell_px, 3), dtype=np.uint8)
    
    for r in range(h):
        for c in range(w):
            if grid_np[r, c]:
                color = [0, 0, 200]
            else:
                color = [240, 240, 240]
            y1, x1 = r * cell_px, c * cell_px
            img[y1:y1+cell_px, x1:x1+cell_px] = color
    
    _, buffer = cv2.imencode('.png', img)
    return send_file(BytesIO(buffer.tobytes()), mimetype='image/png')

@app.route("/command-done", methods=["POST"])
def command_done():
    """
    ESP32 POSTs {"sequence": N} when it finishes executing command N.
    Marks that command DONE in Firebase and advances active_sequence.
    In-memory queue is already popped by /get-command, so nothing to do there.
    """
    data = request.get_json(silent=True) or {}
    seq  = data.get("sequence")

    if firebase_queue.enabled and seq is not None:
        try:
            firebase_queue._db.reference(
                f"/robots/{ROBOT_ID}/queue/{seq}/status"
            ).set("DONE")
            firebase_queue._db.reference(
                f"/robots/{ROBOT_ID}/meta/active_sequence"
            ).set(seq + 1)
            print(f"    Command {seq} marked DONE in Firebase")
        except Exception as fb_err:
            print(f"     Firebase command-done failed: {fb_err}")

    # Advance robot_pose by replaying all commands up to this sequence
    if seq is not None and state.get("nav_start_pose") and state.get("all_commands"):
        executed = state["all_commands"][:seq]
        sp = state["nav_start_pose"]
        new_r, new_c, new_h = _replay_commands(
            sp["row"], sp["col"], sp["heading"],
            executed, state["real_cm_per_cell"]
        )
        state["robot_pose"] = {"row": new_r, "col": new_c, "heading": new_h}
        print(f"    Pose -> ({new_r},{new_c}) h={new_h}deg after cmd {seq}")
        try:
            firebase_queue.publish_status(ROBOT_ID, state["robot_pose"])
        except Exception:
            pass

    return jsonify({"ok": True, "sequence": seq, "pose": state["robot_pose"]})

# ── ROUTE: Manual d-pad movement ─────────────────────────────────────────────
@app.route("/manual-move", methods=["POST"])
def manual_move():
    try:
        import math as _math
        data      = request.get_json(silent=True) or {}
        direction = data.get("dir", "S")
        step_cm   = float(data.get("step_cm", 50.0))
        turn_deg  = int(data.get("turn_deg", 90))

        row     = state["robot_pose"]["row"]
        col     = state["robot_pose"]["col"]
        heading = state["robot_pose"]["heading"]

        DEG_DIR = {
            0: (-1, 0), 45: (-1, 1), 90: (0, 1), 135: (1, 1),
            180: (1, 0), 225: (1, -1), 270: (0, -1), 315: (-1, -1),
        }

        # STOP: clear queue immediately
        if direction == "S":
            state["command_queue"] = []
            try:
                firebase_queue.cancel_queue(ROBOT_ID)
            except Exception:
                pass
            return jsonify({"ok": True, "pose": state["robot_pose"]})

        obstacle_cell = None
        cmd = None
        new_row, new_col, new_heading = row, col, heading

        if direction in ("F", "B"):
            dr, dc      = DEG_DIR.get(heading % 360, (-1, 0))
            sign        = 1 if direction == "F" else -1
            total_steps = max(1, round(step_cm / state["real_cm_per_cell"]))
            grid        = state["inflated_grid"]
            rows_g      = len(grid) if grid else 0
            cols_g      = len(grid[0]) if rows_g > 0 else 0

            last_r, last_c = row, col
            for s in range(1, total_steps + 1):
                nr = row + sign * dr * s
                nc = col + sign * dc * s
                if not (0 <= nr < rows_g and 0 <= nc < cols_g):
                    break
                if grid and grid[nr][nc] == 1:
                    obstacle_cell = [nr, nc]
                    break
                last_r, last_c = nr, nc

            if obstacle_cell and last_r == row and last_c == col:
                return jsonify({
                    "ok":            False,
                    "error":         "Obstacle immediately ahead — cannot move",
                    "obstacle_cell": obstacle_cell,
                    "pose":          state["robot_pose"],
                })

            steps_moved = max(abs(last_r - row), abs(last_c - col))
            is_diag     = (dr != 0 and dc != 0)
            safe_cm     = round(steps_moved * state["real_cm_per_cell"] * (_math.sqrt(2) if is_diag else 1.0))
            if safe_cm <= 0:
                return jsonify({"ok": False, "error": "No movement possible", "pose": state["robot_pose"]})

            cmd     = f"{direction}{safe_cm}"
            new_row, new_col = last_r, last_c

        elif direction == "L":
            cmd        = f"L{turn_deg}"
            new_heading = (heading - turn_deg) % 360

        elif direction == "R":
            cmd        = f"R{turn_deg}"
            new_heading = (heading + turn_deg) % 360

        else:
            return jsonify({"ok": False, "error": f"Unknown direction: {direction}"}), 400

        # Push to Firebase queue — pose updates only when /command-done is received
        state["nav_start_pose"] = dict(state["robot_pose"])
        state["all_commands"]   = [cmd]
        state["command_queue"]  = [cmd]

        try:
            firebase_queue.publish_queue(ROBOT_ID, [cmd])
        except Exception as fb_err:
            print(f"     Firebase manual queue failed: {fb_err}")

        # Fallback when Firebase is disabled: apply pose immediately
        if not firebase_queue.enabled:
            state["robot_pose"] = {"row": new_row, "col": new_col, "heading": new_heading}
            resp = {"ok": True, "queued": False, "pose": state["robot_pose"]}
            if obstacle_cell:
                resp["warning"]       = "Partial move: obstacle ahead"
                resp["obstacle_cell"] = obstacle_cell
            return jsonify(resp)

        resp = {"ok": True, "queued": True, "command": cmd, "pose": state["robot_pose"]}
        if obstacle_cell:
            resp["warning"]       = "Partial move: obstacle ahead"
            resp["obstacle_cell"] = obstacle_cell
        return jsonify(resp)

    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


if __name__ == "__main__":
    # Start Firebase pose-sync watcher (daemon so it exits when Flask exits)
    _t = threading.Thread(target=_firebase_sync_loop, daemon=True)
    _t.start()

    print(f"   Firebase: {'enabled' if firebase_queue.enabled else 'disabled -- set FIREBASE_CREDENTIALS and FIREBASE_DATABASE_URL'}")
    print("Routes:")
    print("  POST /upload         upload blueprint image")
    print("  GET  /is-free        check robot placement validity")
    print("  POST /set-pose       set robot start position (validated)")
    print("  POST /navigate       navigate to a room")
    print("  GET  /status         get system status")
    print("  GET  /get-command    ESP32 polls for next command")
    print("  POST /command-done   ESP32 confirms command executed")
    print("  GET  /grid.png       grid overlay (raw walls dark red + inflated yellow)")
    print("  GET  /path.png       path overlay")
    print("  GET  /debug_grid.png debug grid")
    print("  POST /manual-move    d-pad manual movement")
    
    app.run(debug=True, host='0.0.0.0', port=5000, use_reloader=False)
