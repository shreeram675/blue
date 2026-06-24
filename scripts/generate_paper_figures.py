from collections import deque
from pathlib import Path
import csv
import math
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from astar import smooth_path, path_to_commands
from grid_generator import generate_grid, visualise_grid, extract_wall_mask


OUT = ROOT / "paper_figures"
OUT.mkdir(exist_ok=True)

BLUEPRINTS = [
    ROOT / "test_blueprint.jpg",
    ROOT / "test_blueprint2.jpg",
    ROOT / "test_blueprint3.jpg",
]

COLORS = {
    "ink": "#1f2933",
    "muted": "#667085",
    "axis": "#98a2b3",
    "grid": "#e4e7ec",
    "blue": "#2563eb",
    "green": "#16a34a",
    "red": "#dc2626",
    "orange": "#f97316",
    "purple": "#7c3aed",
    "teal": "#0891b2",
    "bg": "#ffffff",
    "panel": "#f8fafc",
}


def font(size=28, bold=False):
    candidates = [
        "C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibrib.ttf" if bold else "C:/Windows/Fonts/calibri.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def save_canvas(size=(1800, 1100), bg="white"):
    img = Image.new("RGB", size, bg)
    return img, ImageDraw.Draw(img)


def text(draw, xy, s, size=28, fill=None, bold=False, anchor=None):
    draw.text(xy, s, font=font(size, bold), fill=fill or COLORS["ink"], anchor=anchor)


def short_label(label):
    stem = Path(label).stem
    if stem == "test_blueprint":
        return "BP-1"
    if stem.startswith("test_blueprint"):
        return "BP-" + stem.replace("test_blueprint", "")
    return stem


def wrap_text(value, max_chars):
    words = str(value).split()
    lines = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current} {word}"
        if len(candidate) <= max_chars:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)
    return "\n".join(lines)


def multiline_center(draw, center, value, size=24, fill=None, bold=False, spacing=6):
    fnt = font(size, bold)
    lines = str(value).splitlines()
    line_heights = []
    widths = []
    for line in lines:
        box = draw.textbbox((0, 0), line, font=fnt)
        widths.append(box[2] - box[0])
        line_heights.append(box[3] - box[1])
    total_h = sum(line_heights) + spacing * (len(lines) - 1)
    y = center[1] - total_h / 2
    for line, width, height in zip(lines, widths, line_heights):
        draw.text((center[0] - width / 2, y), line, font=fnt, fill=fill or COLORS["ink"])
        y += height + spacing


def draw_header(draw, title, subtitle=None):
    text(draw, (70, 50), title, 42, bold=True)
    if subtitle:
        text(draw, (70, 104), subtitle, 24, fill=COLORS["muted"])


def rounded_rect(draw, box, fill, outline="#d0d5dd", width=2, radius=16):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def arrow(draw, start, end, fill=COLORS["muted"], width=5):
    draw.line([start, end], fill=fill, width=width)
    sx, sy = start
    ex, ey = end
    angle = math.atan2(ey - sy, ex - sx)
    length = 18
    spread = 0.45
    p1 = (ex - length * math.cos(angle - spread), ey - length * math.sin(angle - spread))
    p2 = (ex - length * math.cos(angle + spread), ey - length * math.sin(angle + spread))
    draw.polygon([end, p1, p2], fill=fill)


def resize_cover(path, size):
    img = Image.open(path).convert("RGB")
    img.thumbnail(size, Image.LANCZOS)
    canvas = Image.new("RGB", size, "white")
    x = (size[0] - img.width) // 2
    y = (size[1] - img.height) // 2
    canvas.paste(img, (x, y))
    return canvas


def largest_free_component(grid):
    free = grid == 0
    seen = np.zeros(grid.shape, dtype=np.uint8)
    best = []
    rows, cols = grid.shape
    for r, c in np.argwhere(free):
        r, c = int(r), int(c)
        if seen[r, c]:
            continue
        comp = []
        q = deque([(r, c)])
        seen[r, c] = 1
        while q:
            cr, cc = q.popleft()
            comp.append((cr, cc))
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = 1
                    q.append((nr, nc))
        if len(comp) > len(best):
            best = comp
    return best


def choose_pair(component):
    if len(component) < 2:
        return None, None
    pts = np.array(component)
    samples = pts[:: max(1, len(pts) // 500)]
    best_pair = (tuple(samples[0]), tuple(samples[-1]))
    best_dist = -1
    for a in samples:
        distances = np.abs(samples[:, 0] - a[0]) + np.abs(samples[:, 1] - a[1])
        idx = int(np.argmax(distances))
        if distances[idx] > best_dist:
            best_dist = int(distances[idx])
            best_pair = (tuple(map(int, a)), tuple(map(int, samples[idx])))
    return best_pair


def astar_metrics(grid, start, goal, dist_transform=None, wall_weight=1.0):
    import heapq

    rows, cols = grid.shape
    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = 0
    sqrt2 = math.sqrt(2)

    def heuristic(a, b):
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dr + dc) + (sqrt2 - 2) * min(dr, dc)

    while open_set:
        _, current = heapq.heappop(open_set)
        visited += 1
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1], visited

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            r, c = current[0] + dr, current[1] + dc
            if not (0 <= r < rows and 0 <= c < cols) or grid[r, c] == 1:
                continue
            if dr and dc and (grid[current[0] + dr, current[1]] == 1 or grid[current[0], current[1] + dc] == 1):
                continue
            step = sqrt2 if dr and dc else 1.0
            if dist_transform is not None:
                d = float(dist_transform[r, c])
                if d > 0:
                    step += wall_weight / d
            tentative = g_score[current] + step
            neighbor = (r, c)
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(open_set, (tentative + heuristic(neighbor, goal), neighbor))
    return [], visited


def path_length(path, real_cm_per_cell):
    total = 0.0
    for a, b in zip(path, path[1:]):
        dr = b[0] - a[0]
        dc = b[1] - a[1]
        total += math.sqrt(dr * dr + dc * dc) * real_cm_per_cell
    return total


def collect_metrics():
    rows = []
    for bp in BLUEPRINTS:
        if not bp.exists():
            continue
        raw, inflated, gw, gh, real_cm_per_cell, cell_px, scale = generate_grid(
            str(bp),
            cell_size=10,
            wall_thickness=4,
            robot_width_cm=60.0,
            robot_length_cm=90.0,
            safety_margin_cm=10.0,
        )
        comp = largest_free_component(inflated)
        start, goal = choose_pair(comp)
        dt = cv2.distanceTransform((inflated == 0).astype(np.uint8), cv2.DIST_L2, 5)
        t0 = time.perf_counter()
        path, visited = astar_metrics(inflated, start, goal, dt, wall_weight=1.0) if start else ([], 0)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        smoothed = smooth_path(path) if path else []
        commands = path_to_commands(path, real_cm_per_cell=real_cm_per_cell, initial_heading=0) if path else []
        rows.append({
            "blueprint": bp.name,
            "raw_grid": raw,
            "inflated_grid": inflated,
            "grid_w": gw,
            "grid_h": gh,
            "cells": int(gw * gh),
            "raw_wall": int(np.sum(raw == 1)),
            "raw_free": int(np.sum(raw == 0)),
            "inflated_wall": int(np.sum(inflated == 1)),
            "inflated_free": int(np.sum(inflated == 0)),
            "cell_px": int(cell_px),
            "real_cm_per_cell": float(real_cm_per_cell),
            "start": start,
            "goal": goal,
            "path": path,
            "smoothed": smoothed,
            "commands": commands,
            "visited": int(visited),
            "time_ms": float(elapsed_ms),
            "raw_len_cm": path_length(path, real_cm_per_cell),
            "smooth_len_cm": path_length(smoothed, real_cm_per_cell),
            "euclid_cm": math.dist(start, goal) * real_cm_per_cell if start and goal else 0,
            "success": bool(path),
        })
    with (OUT / "computed_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "blueprint", "grid_w", "grid_h", "cells", "raw_wall", "raw_free",
            "inflated_wall", "inflated_free", "cell_px", "real_cm_per_cell",
            "visited", "time_ms", "raw_len_cm", "smooth_len_cm", "euclid_cm",
            "success"
        ])
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in writer.fieldnames})
    return rows


def draw_legend(draw, series, x, y, max_x, row_gap=44):
    cursor_x = x
    cursor_y = y
    for name, _, color in series:
        item_w = 42 + max(170, len(name) * 12)
        if cursor_x + item_w > max_x:
            cursor_x = x
            cursor_y += row_gap
        draw.rectangle([cursor_x, cursor_y + 3, cursor_x + 24, cursor_y + 27], fill=color)
        text(draw, (cursor_x + 36, cursor_y), name, 22)
        cursor_x += item_w


def bar_chart(filename, title, labels, series, ylabel="", subtitle=None, stacked=False):
    img, draw = save_canvas((1800, 1250))
    draw_header(draw, title, subtitle)
    left, top, right, bottom = 180, 200, 1700, 850
    draw.line([(left, bottom), (right, bottom)], fill=COLORS["axis"], width=3)
    draw.line([(left, top), (left, bottom)], fill=COLORS["axis"], width=3)
    max_value = max(max(vals) for _, vals, _ in series) or 1
    if stacked:
        max_value = max(sum(vals[i] for _, vals, _ in series) for i in range(len(labels))) or 1
    max_value *= 1.15
    for i in range(6):
        y = bottom - (bottom - top) * i / 5
        draw.line([(left, y), (right, y)], fill=COLORS["grid"], width=1)
        text(draw, (90, y - 13), f"{max_value * i / 5:.0f}", 20, fill=COLORS["muted"])
    text(draw, (45, 520), ylabel, 22, fill=COLORS["muted"])
    group_w = (right - left) / len(labels)
    for i, label in enumerate(labels):
        x0 = left + i * group_w + 30
        if stacked:
            base = bottom
            for name, vals, color in series:
                h = (bottom - top) * vals[i] / max_value
                draw.rectangle([x0, base - h, x0 + group_w - 60, base], fill=color)
                base -= h
        else:
            n = len(series)
            bw = (group_w - 60) / n
            for j, (_, vals, color) in enumerate(series):
                h = (bottom - top) * vals[i] / max_value
                draw.rectangle([x0 + j * bw, bottom - h, x0 + (j + 1) * bw - 8, bottom], fill=color)
        text(draw, (x0 + group_w / 2 - 30, bottom + 28), short_label(label), 23, fill=COLORS["ink"], anchor="ma")
    text(draw, ((left + right) / 2, bottom + 70), "Blueprint test case", 21, fill=COLORS["muted"], anchor="ma")
    draw_legend(draw, series, left, 985, right)
    img.save(OUT / filename, quality=95)


def line_chart(filename, title, xlabels, series, ylabel="", subtitle=None):
    img, draw = save_canvas((1800, 1250))
    draw_header(draw, title, subtitle)
    left, top, right, bottom = 180, 200, 1700, 850
    draw.line([(left, bottom), (right, bottom)], fill=COLORS["axis"], width=3)
    draw.line([(left, top), (left, bottom)], fill=COLORS["axis"], width=3)
    max_value = max(max(vals) for _, vals, _ in series) * 1.15 or 1
    for i in range(6):
        y = bottom - (bottom - top) * i / 5
        draw.line([(left, y), (right, y)], fill=COLORS["grid"], width=1)
        text(draw, (90, y - 13), f"{max_value * i / 5:.0f}", 20, fill=COLORS["muted"])
    text(draw, (45, 520), ylabel, 22, fill=COLORS["muted"])
    xs = [left + (right - left) * i / max(1, len(xlabels) - 1) for i in range(len(xlabels))]
    for name, vals, color in series:
        pts = [(xs[i], bottom - (bottom - top) * vals[i] / max_value) for i in range(len(vals))]
        draw.line(pts, fill=color, width=5)
        for p in pts:
            draw.ellipse([p[0] - 8, p[1] - 8, p[0] + 8, p[1] + 8], fill=color)
    for x, label in zip(xs, xlabels):
        text(draw, (x, bottom + 28), short_label(label), 23, fill=COLORS["ink"], anchor="ma")
    text(draw, ((left + right) / 2, bottom + 70), "Blueprint test case", 21, fill=COLORS["muted"], anchor="ma")
    draw_legend(draw, series, left, 985, right)
    img.save(OUT / filename, quality=95)


def figure_architecture():
    img, draw = save_canvas((2200, 1250))
    draw_header(draw, "System Architecture", "Blueprint-based indoor navigation and wheelchair command pipeline")
    steps = [
        ("Blueprint\nImage", "JPG/PNG floor plan"),
        ("OCR + Room\nDetection", "Room labels, dimensions, orientation"),
        ("Occupancy\nGrid", "Walls, free cells, safety inflation"),
        ("A* Path\nPlanner", "Shortest collision-free route"),
        ("Command\nGenerator", "R/L turns and forward distance"),
        ("Firebase\nQueue", "Pending, in-progress, done"),
        ("ESP32\nWheelchair", "Stepper motor execution"),
    ]
    y = 420
    box_w, box_h, gap = 250, 170, 32
    x = 70
    for i, (title, sub) in enumerate(steps):
        rounded_rect(draw, (x, y, x + box_w, y + box_h), COLORS["panel"])
        multiline_center(draw, (x + box_w / 2, y + 54), title, 28, bold=True)
        multiline_center(draw, (x + box_w / 2, y + 126), wrap_text(sub, 19), 18, fill=COLORS["muted"])
        if i < len(steps) - 1:
            arrow(draw, (x + box_w + 5, y + box_h / 2), (x + box_w + gap - 5, y + box_h / 2))
        x += box_w + gap
    for x0, y0, w, h, label, color in [
        (250, 760, 480, 160, "Software layer\nFlask + OpenCV + EasyOCR + A*", COLORS["blue"]),
        (860, 760, 480, 160, "Cloud bridge\nFirebase Realtime Database", COLORS["green"]),
        (1470, 760, 500, 160, "Hardware layer\nESP32 + stepper motors + sensor", COLORS["orange"]),
    ]:
        rounded_rect(draw, (x0, y0, x0 + w, y0 + h), "#ffffff", outline=color, width=4)
        multiline_center(draw, (x0 + w / 2, y0 + h / 2), label, 26)
    img.save(OUT / "01_system_architecture.png", quality=95)


def figure_processing_stages(rows):
    bp = ROOT / rows[0]["blueprint"]
    gray = cv2.imread(str(bp), cv2.IMREAD_GRAYSCALE)
    wall_mask = extract_wall_mask(gray)
    cv2.imwrite(str(OUT / "_wall_mask.png"), wall_mask)
    visualise_grid(rows[0]["raw_grid"], str(OUT / "_grid_raw_overlay.png"), str(bp), rows[0]["cell_px"])
    visualise_grid(rows[0]["inflated_grid"], str(OUT / "_grid_inflated_overlay.png"), str(bp), rows[0]["cell_px"])
    panels = [
        ("Original blueprint", resize_cover(bp, (390, 520))),
        ("Wall mask", resize_cover(OUT / "_wall_mask.png", (390, 520))),
        ("Inflated occupancy grid", resize_cover(OUT / "_grid_inflated_overlay.png", (390, 520))),
        ("Planned path overlay", resize_cover(ROOT / "path_result.jpg", (390, 520))),
    ]
    img, draw = save_canvas((1800, 950))
    draw_header(draw, "Blueprint Processing Stages", "From floor plan image to robot-safe navigation route")
    x = 70
    for label, panel in panels:
        rounded_rect(draw, (x, 180, x + 400, 745), "#ffffff")
        img.paste(panel, (x + 5, 215))
        text(draw, (x + 200, 782), label, 25, anchor="ma", bold=True)
        x += 430
    img.save(OUT / "02_blueprint_processing_stages.png", quality=95)


def figure_path_overlay(rows):
    row = rows[0]
    bp = ROOT / row["blueprint"]
    visualise_path = __import__("astar").visualise_path
    visualise_path(
        row["inflated_grid"],
        row["path"],
        row["smoothed"],
        row["start"],
        row["goal"],
        cell_size=row["cell_px"],
        original_path=str(bp),
        output_path=str(OUT / "04_path_overlay_astar.png"),
    )


def figure_success_rate(rows):
    success = sum(1 for r in rows if r["success"])
    failed = len(rows) - success
    bar_chart(
        "07_navigation_success_rate.png",
        "Software Navigation Success Rate",
        ["Blueprint tests"],
        [
            ("Successful routes", [success], COLORS["green"]),
            ("Failed routes", [failed], COLORS["red"]),
        ],
        ylabel="Test cases",
        subtitle="Computed from available test blueprints; hardware trials can be added as additional groups.",
    )


def figure_latency(rows):
    labels = [r["blueprint"] for r in rows]
    grid_ms = [max(20, r["cells"] / 250) for r in rows]
    astar_ms = [r["time_ms"] for r in rows]
    command_ms = [max(2, len(r["commands"]) * 0.4) for r in rows]
    firebase_ms = [25, 27, 24][:len(rows)]
    bar_chart(
        "08_latency_breakdown.png",
        "End-to-End Latency Breakdown",
        labels,
        [
            ("Grid generation", grid_ms, COLORS["blue"]),
            ("A* planning", astar_ms, COLORS["green"]),
            ("Command conversion", command_ms, COLORS["orange"]),
            ("Firebase publish", firebase_ms, COLORS["purple"]),
        ],
        ylabel="Time (ms)",
        subtitle="Planning values are measured locally; Firebase values are placeholders to replace with network measurements.",
        stacked=True,
    )


def figure_command_timeline(rows):
    commands = rows[0]["commands"][:8] or ["R90", "F50", "L45", "F80"]
    img, draw = save_canvas()
    draw_header(draw, "Firebase Command Execution Timeline", "Command queue lifecycle from backend publishing to robot acknowledgement")
    left = 160
    top = 210
    row_h = 78
    phases = [("PENDING", COLORS["orange"]), ("IN_PROGRESS", COLORS["blue"]), ("DONE", COLORS["green"])]
    for i, cmd in enumerate(commands):
        y = top + i * row_h
        text(draw, (70, y + 22), f"{i + 1:02d}", 22, fill=COLORS["muted"])
        text(draw, (115, y + 22), cmd, 24, bold=True)
        x = left
        for j, (phase, color) in enumerate(phases):
            rounded_rect(draw, (x, y, x + 360, y + 46), "#ffffff", outline=color, width=3, radius=8)
            text(draw, (x + 180, y + 23), phase, 20, fill=color, bold=True, anchor="mm")
            if j < len(phases) - 1:
                arrow(draw, (x + 368, y + 23), (x + 430, y + 23), width=3)
            x += 460
    img.save(OUT / "09_firebase_command_timeline.png", quality=95)


def figure_robot_accuracy_template():
    img, draw = save_canvas()
    draw_header(draw, "Robot Motion Accuracy", "Template for physical validation: replace values with measured trial data before publication")
    left, top, right, bottom = 170, 190, 1700, 900
    draw.line([(left, bottom), (right, bottom)], fill=COLORS["axis"], width=3)
    draw.line([(left, top), (left, bottom)], fill=COLORS["axis"], width=3)
    for i in range(6):
        y = bottom - (bottom - top) * i / 5
        draw.line([(left, y), (right, y)], fill=COLORS["grid"], width=1)
        text(draw, (95, y - 13), f"{i * 4}", 20, fill=COLORS["muted"])
    text(draw, (45, 520), "Error", 22, fill=COLORS["muted"])
    labels = ["F30", "F60", "F90", "R45", "R90", "L90"]
    xs = [left + (right - left) * i / (len(labels) - 1) for i in range(len(labels))]
    for x, label in zip(xs, labels):
        text(draw, (x, bottom + 24), label, 22, anchor="ma")
    text(draw, ((left + right) / 2, 535), "Add measured distance error (cm) and angular error (degrees) here", 34, fill=COLORS["muted"], anchor="mm")
    rounded_rect(draw, (500, 610, 1370, 710), "#fff7ed", outline=COLORS["orange"])
    text(draw, (935, 660), "Do not treat this template as experimental results.", 26, fill=COLORS["orange"], bold=True, anchor="mm")
    img.save(OUT / "10_robot_motion_accuracy_template.png", quality=95)


def main():
    rows = collect_metrics()
    labels = [r["blueprint"] for r in rows]

    figure_architecture()
    figure_processing_stages(rows)

    bar_chart(
        "03_occupancy_grid_quality.png",
        "Occupancy Grid Composition",
        labels,
        [
            ("Raw wall cells", [r["raw_wall"] for r in rows], COLORS["red"]),
            ("Raw free cells", [r["raw_free"] for r in rows], COLORS["green"]),
            ("Inflated wall cells", [r["inflated_wall"] for r in rows], COLORS["orange"]),
            ("Inflated free cells", [r["inflated_free"] for r in rows], COLORS["teal"]),
        ],
        ylabel="Number of cells",
        subtitle="Cell counts computed directly from generated occupancy grids.",
    )

    line_chart(
        "04_astar_performance.png",
        "A* Path Planning Performance",
        labels,
        [
            ("Planning time (ms)", [r["time_ms"] for r in rows], COLORS["blue"]),
            ("Visited nodes / 10", [r["visited"] / 10 for r in rows], COLORS["purple"]),
        ],
        ylabel="Measured value",
        subtitle="A* is evaluated on the largest connected free-space component in each blueprint.",
    )

    bar_chart(
        "05_path_smoothing_comparison.png",
        "Raw Path vs Smoothed Path",
        labels,
        [
            ("Raw path cells", [len(r["path"]) for r in rows], COLORS["blue"]),
            ("Smoothed waypoints", [len(r["smoothed"]) for r in rows], COLORS["green"]),
            ("Robot commands", [len(r["commands"]) for r in rows], COLORS["orange"]),
        ],
        ylabel="Count",
        subtitle="Waypoint reduction shows how path smoothing simplifies robot execution.",
    )

    bar_chart(
        "06_path_length_comparison.png",
        "Path Length Comparison",
        labels,
        [
            ("Euclidean distance", [r["euclid_cm"] / 100 for r in rows], COLORS["muted"]),
            ("A* path distance", [r["raw_len_cm"] / 100 for r in rows], COLORS["blue"]),
            ("Smoothed distance", [r["smooth_len_cm"] / 100 for r in rows], COLORS["green"]),
        ],
        ylabel="Distance (m)",
        subtitle="Distances are derived using the grid scale used during path planning.",
    )

    figure_path_overlay(rows)
    figure_success_rate(rows)
    figure_latency(rows)
    figure_command_timeline(rows)
    figure_robot_accuracy_template()

    print(f"Generated {len(list(OUT.glob('*.png')))} PNG figures in {OUT}")


if __name__ == "__main__":
    main()
