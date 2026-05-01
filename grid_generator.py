import cv2
import numpy as np

# ── UNIT CONVERSION ────────────────────────────────────────────────────────
UNIT_TO_CM = {"cm": 1.0, "m": 100.0, "ft": 30.48}


def extract_wall_mask(gray, wall_thickness=4):
    """
    Extract structural walls while suppressing text and symbols.
    Uses directional morphological kernels then filters small components.
    """
    binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)[1]

    h_kern = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (max(12, wall_thickness * 6), max(1, wall_thickness))
    )
    v_kern = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (max(1, wall_thickness), max(12, wall_thickness * 6))
    )
    horizontal = cv2.morphologyEx(binary, cv2.MORPH_OPEN, h_kern)
    vertical   = cv2.morphologyEx(binary, cv2.MORPH_OPEN, v_kern)
    walls = cv2.bitwise_or(horizontal, vertical)

    join_kern = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (max(3, wall_thickness * 2 + 1), max(3, wall_thickness * 2 + 1))
    )
    walls = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, join_kern)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(walls, connectivity=8)
    filtered = np.zeros_like(walls)
    min_span = max(gray.shape) * 0.12
    min_area = gray.shape[0] * gray.shape[1] * 0.002
    for idx in range(1, count):
        _, _, w, h, area = stats[idx]
        if area >= min_area or w >= min_span or h >= min_span:
            filtered[labels == idx] = 255
    return filtered


def compute_cell_size(image_width_px, building_width_units, unit,
                      robot_narrow_cm=60.0):
    """
    Derive a grid cell size so each cell ≈ robot_narrow_cm / 4.
    Returns: cell_size_px, real_cm_per_cell
    """
    unit_factor       = UNIT_TO_CM.get(str(unit).lower(), 1.0)
    building_width_cm = building_width_units * unit_factor
    real_cm_per_pixel = building_width_cm / image_width_px
    cell_size_px      = max(2, round(robot_narrow_cm / 4.0 / real_cm_per_pixel))
    real_cm_per_cell  = cell_size_px * real_cm_per_pixel
    return cell_size_px, real_cm_per_cell


def compute_cell_size_from_scale(cm_per_pixel, robot_narrow_cm=60.0):
    """
    Same as compute_cell_size but takes cm_per_pixel directly
    (estimated from room label positions × room dimensions).
    robot_narrow_cm = min(robot_width, robot_length).
    """
    cell_size_px     = max(2, round(robot_narrow_cm / 4.0 / cm_per_pixel))
    real_cm_per_cell = cell_size_px * cm_per_pixel
    return cell_size_px, real_cm_per_cell


def find_narrow_passages(raw_grid, real_cm_per_cell,
                         robot_width_cm, safety_margin_cm=10.0):
    """
    Scan raw (un-inflated) grid for doorway-sized gaps in wall rows/columns.
    Reports gaps that are passable but tighter than robot_width + 2*margin.

    Returns list of dicts: {row, col, width_cm, direction}
    """
    required_cm = robot_width_cm + 2 * safety_margin_cm
    narrow = []
    rows, cols = raw_grid.shape

    def _scan_line(line, axis_label, fixed_idx):
        in_gap, gap_start = False, 0
        for i, cell in enumerate(line):
            if cell == 0:
                if not in_gap:
                    in_gap, gap_start = True, i
            else:
                if in_gap:
                    gap_cm = (i - gap_start) * real_cm_per_cell
                    # Only flag gaps that are surrounded by walls (real passages)
                    if 0 < gap_cm < required_cm * 2:
                        entry = {"width_cm": round(gap_cm, 1), "direction": axis_label}
                        if axis_label == "horizontal":
                            entry["row"] = fixed_idx
                            entry["col"] = gap_start
                        else:
                            entry["row"] = gap_start
                            entry["col"] = fixed_idx
                        narrow.append(entry)
                    in_gap = False

    for r in range(rows):
        _scan_line(raw_grid[r], "horizontal", r)
    for c in range(cols):
        _scan_line(raw_grid[:, c], "vertical", c)

    return narrow


def verify_doorway_clearance(doorway_width_px, robot_width_cm,
                              real_cm_per_pixel, safety_margin_cm=10.0):
    """Return (ok, doorway_cm)."""
    doorway_cm  = doorway_width_px * real_cm_per_pixel
    required_cm = robot_width_cm + 2 * safety_margin_cm
    if doorway_cm >= required_cm:
        return True, doorway_cm
    print(f"   Doorway too narrow: {doorway_cm:.1f}cm "
          f"< {required_cm:.1f}cm required")
    return False, doorway_cm


def generate_grid(image_path, cell_size=10, wall_thickness=4,
                  unit="cm", robot_width_cm=60.0, robot_length_cm=None,
                  safety_margin_cm=10.0,
                  building_width_units=None,
                  cm_per_pixel=None):
    """
    Convert a blueprint image to a binary occupancy grid.
      0 = free space, 1 = wall/obstacle

    Scale priority:
      1. cm_per_pixel  -- estimated from room label positions + dimensions (best)
      2. building_width_units -- user-provided fallback
      3. default cell_size   -- last resort

    robot_length_cm defaults to robot_width_cm when omitted.
    Returns: grid, grid_w, grid_h, real_cm_per_cell, cell_size_px, scale_applied
    """
    if robot_length_cm is None:
        robot_length_cm = robot_width_cm

    robot_narrow_cm = min(robot_width_cm, robot_length_cm)

    print(f"\nGenerating grid from: {image_path}")
    img  = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    print(f"   Image size: {w}x{h} px")
    print(f"   Robot footprint: {robot_width_cm}x{robot_length_cm} cm "
          f"(narrow={robot_narrow_cm} cm)")

    real_cm_per_cell = float(cell_size)
    scale_applied    = False

    # ── 1. Scale from room-position estimation (best) ──────────────────────
    if cm_per_pixel is not None and cm_per_pixel > 0:
        cs_auto, cpc_auto = compute_cell_size_from_scale(cm_per_pixel, robot_narrow_cm)
        MAX_GRID_CELLS = 200
        cs_clamped = max(cs_auto, max(w, h) // MAX_GRID_CELLS + 1, 5)
        if cs_clamped != cs_auto:
            real_cm_per_cell = cs_clamped * cm_per_pixel
            print(f"   Auto cell size {cs_auto}px clamped to {cs_clamped}px")
            cs_auto, cpc_auto = cs_clamped, real_cm_per_cell

        if w // cs_auto >= 20 and h // cs_auto >= 20:
            cell_size        = cs_auto
            real_cm_per_cell = cpc_auto
            scale_applied    = True
            print(f"   Scale (room positions): {cm_per_pixel:.4f} cm/px  "
                  f"-> cell={cell_size}px, {real_cm_per_cell:.2f} cm/cell")
        else:
            print(f"   Scale gave grid too small ({w//cs_auto}x{h//cs_auto}) -- keeping default")

    # ── 2. Fallback: user-supplied building width ──────────────────────────
    elif building_width_units is not None:
        cs_auto, cpc_auto = compute_cell_size(
            w, building_width_units, unit, robot_narrow_cm
        )
        MAX_GRID_CELLS = 200
        cs_clamped = max(cs_auto, max(w, h) // MAX_GRID_CELLS + 1, 5)
        if cs_clamped != cs_auto:
            unit_factor      = UNIT_TO_CM.get(str(unit).lower(), 1.0)
            real_cm_per_pixel = building_width_units * unit_factor / w
            cpc_auto          = cs_clamped * real_cm_per_pixel
            print(f"   Auto cell size {cs_auto}px clamped to {cs_clamped}px")
            cs_auto = cs_clamped
        if w // cs_auto >= 20 and h // cs_auto >= 20:
            cell_size        = cs_auto
            real_cm_per_cell = cpc_auto
            scale_applied    = True
            print(f"   Scale (building width): cell={cell_size}px, {real_cm_per_cell:.2f} cm/cell")
        else:
            print(f"   Building-width cell size too small  keeping {cell_size}px")

    # ── Wall extraction ────────────────────────────────────────────────────
    cleaned = extract_wall_mask(gray, wall_thickness=wall_thickness)

    # ── Build occupancy grid ───────────────────────────────────────────────
    grid_w = w // cell_size
    grid_h = h // cell_size
    print(f"   Grid size: {grid_w}x{grid_h} cells "
          f"(cell={cell_size}px, {real_cm_per_cell:.1f} cm/cell)")

    grid = np.zeros((grid_h, grid_w), dtype=np.uint8)
    for row in range(grid_h):
        for col in range(grid_w):
            px1, py1 = col * cell_size, row * cell_size
            block    = cleaned[py1:py1 + cell_size, px1:px1 + cell_size]
            if np.sum(block > 0) / block.size > 0.12:
                grid[row][col] = 1

    print(f"   Raw wall cells: {np.sum(grid == 1)}")
    print(f"   Raw free cells: {np.sum(grid == 0)}")

    # ── Seal exterior: flood-fill from image edges to block margin/title area ─
    # Prevents A* from routing outside the floor plan through free margin cells
    from collections import deque as _deque
    _gh, _gw = grid.shape
    _outside = np.zeros_like(grid)
    _queue = _deque()
    for _c in range(_gw):
        for _r in [0, _gh - 1]:
            if grid[_r, _c] == 0 and _outside[_r, _c] == 0:
                _outside[_r, _c] = 1
                _queue.append((_r, _c))
    for _r in range(_gh):
        for _c in [0, _gw - 1]:
            if grid[_r, _c] == 0 and _outside[_r, _c] == 0:
                _outside[_r, _c] = 1
                _queue.append((_r, _c))
    while _queue:
        _r, _c = _queue.popleft()
        for _dr, _dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            _nr, _nc = _r + _dr, _c + _dc
            if 0 <= _nr < _gh and 0 <= _nc < _gw and \
               grid[_nr, _nc] == 0 and _outside[_nr, _nc] == 0:
                _outside[_nr, _nc] = 1
                _queue.append((_nr, _nc))
    grid[_outside == 1] = 1
    print(f"   Exterior sealed: {int(np.sum(_outside))} margin cells blocked")

    # Inflate for path planning/obstacle avoidance
    inflated = inflate_obstacles(
        grid,
        robot_width_cm=robot_width_cm,
        robot_length_cm=robot_length_cm,
        real_cm_per_cell=real_cm_per_cell,
        safety_margin_cm=safety_margin_cm,
    )
    print(f"   Inflated obstacle cells: {np.sum(inflated == 1)}")
    
    return grid, inflated, grid_w, grid_h, real_cm_per_cell, cell_size, scale_applied


def inflate_obstacles(grid, robot_width_cm, robot_length_cm,
                      real_cm_per_cell, safety_margin_cm=10.0):
    """
    Inflate wall cells by exactly the user-specified safety_margin.
    safety_margin=0 -> no inflation, raw walls used as-is.
    """
    inflate_cells = max(0, round(safety_margin_cm / real_cm_per_cell)) \
                    if real_cm_per_cell > 0 else 0

    if inflate_cells == 0:
        print(f"   No inflation (safety_margin={safety_margin_cm}cm)")
        print(f"   Free cells after inflation: {int(np.sum(grid == 0))}")
        return grid.astype(np.uint8).copy()

    kernel   = np.ones((inflate_cells * 2 + 1, inflate_cells * 2 + 1), np.uint8)
    inflated = cv2.dilate(grid.astype(np.uint8), kernel)

    print(f"   Safety margin {safety_margin_cm}cm = {inflate_cells} cells inflation")
    print(f"   Free cells after inflation: {int(np.sum(inflated == 0))}")
    return inflated


def visualise_grid(grid, output_path="grid_overlay.jpg",
                   original_path="test_blueprint.jpg", cell_size=10):
    """Draw grid overlay on the original blueprint. Red = wall cell."""
    img     = cv2.imread(original_path)
    overlay = img.copy()
    gh, gw  = grid.shape
    for row in range(gh):
        for col in range(gw):
            if grid[row][col] == 1:
                x1, y1 = col * cell_size, row * cell_size
                cv2.rectangle(overlay, (x1, y1),
                              (x1 + cell_size, y1 + cell_size),
                              (0, 0, 200), -1)
    result = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
    for col in range(gw):
        cv2.line(result, (col * cell_size, 0),
                 (col * cell_size, gh * cell_size), (200, 200, 200), 1)
    for row in range(gh):
        cv2.line(result, (0, row * cell_size),
                 (gw * cell_size, row * cell_size), (200, 200, 200), 1)
    cv2.imwrite(output_path, result)
    print(f"   Grid overlay saved -> {output_path}")
    return result


if __name__ == "__main__":
    raw_grid, inflated_grid, gw, gh, real_cm_per_cell, cell_size_px, _ = generate_grid(
        "test_blueprint.jpg",
        cell_size=10,
        wall_thickness=4,
        robot_width_cm=60.0,
        robot_length_cm=90.0,
    )
    print(f"   real_cm_per_cell: {real_cm_per_cell:.1f} cm  cell_size: {cell_size_px}px")

    visualise_grid(raw_grid, "grid_raw.jpg", "test_blueprint.jpg", cell_size=cell_size_px)
    visualise_grid(inflated_grid, "grid_inflated.jpg", "test_blueprint.jpg", cell_size=cell_size_px)

    print("\nGrid generation complete!")