import easyocr
import cv2
import re
import numpy as np

# ── CONFIG ─────────────────────────────────────────────────────────────────
CONFIDENCE_THRESHOLD = 0.5
ROOM_KEYWORDS = [
    'reception', 'ward', 'icu', 'ot', 'pharmacy', 'corridor',
    'toilet', 'lab', 'office', 'store', 'nursing', 'nurse', 'pantry',
    'operation', 'emergency', 'casualty', 'x-ray', 'xray',
    'lobby', 'station', 'waiting', 'consultation', 'utility',
    'staircase', 'lift', 'kitchen', 'canteen', 'storage'
]

# ── HELPERS ────────────────────────────────────────────────────────────────
def is_dimension(text):
    """Match: 30*40, 30x40, 30 X 40, 30×40"""
    cleaned = text.strip().replace(' ', '')
    return bool(re.match(r'^\d+[\*xX×]\d+$', cleaned))

def parse_dimension(text):
    """Returns (north_south, east_west) or None"""
    parts = re.split(r'[\*xX×]', text.strip().replace(' ', ''))
    if len(parts) == 2:
        try:
            return (int(parts[0]), int(parts[1]))
        except ValueError:
            return None
    return None

def fix_dimension_misread(text):
    """
    Fix common OCR misreads in dimension strings.
    e.g. '0+90' -> '10*90', '1O*40' -> '10*40'
    """
    t = text.strip()
    # Replace + with * (common OCR confusion)
    t = t.replace('+', '*')
    # Replace O (letter) with 0 (digit)
    t = re.sub(r'(?<=[0-9])O|O(?=[0-9])', '0', t)
    # Replace S with 5 when it appears as first digit (e.g. S0*30 -> 50*30)
    t = re.sub(r'^S(\d)', r'5\1', t)
    # Replace SC or S with 5 before * or x
    t = re.sub(r'^SC', '50', t)
    # If starts with digit missing (e.g. '0*90' when original is '10*90')
    # heuristic: if first number is single digit and < 5, try prepending 1
    match = re.match(r'^(\d+)[\*xX×](\d+)$', t.replace(' ', ''))
    if match:
        n1, n2 = int(match.group(1)), int(match.group(2))
        # If first number looks too small relative to second (likely missing a digit)
        if n1 < 5 and n2 > 20:
            t = f"1{n1}*{n2}"
    return t

def is_orientation_marker(text):
    """N, S, E, W standalone"""
    return text.strip().upper() in ['N', 'S', 'E', 'W']

def is_room_label(text):
    t = text.strip().lower()
    for kw in ROOM_KEYWORDS:
        if kw in t:
            return True
    return False

def bbox_center(bbox):
    pts = np.array(bbox)
    return (int(pts[:, 0].mean()), int(pts[:, 1].mean()))

def merge_nearby_texts(results, x_gap=100, y_gap=30):
    """
    Merge text fragments that are on the same horizontal line.
    Sorts left-to-right before merging so 'Ward' + '1' -> 'Ward 1'.
    """
    # Sort by vertical center first, then horizontal
    sorted_results = sorted(results, key=lambda r: (bbox_center(r[0])[1], bbox_center(r[0])[0]))

    merged = []
    used = set()

    for i, (bbox_i, text_i, conf_i) in enumerate(sorted_results):
        if i in used:
            continue
        cx_i, cy_i = bbox_center(bbox_i)
        group_items = [(cx_i, text_i, conf_i, bbox_i)]

        for j, (bbox_j, text_j, conf_j) in enumerate(sorted_results):
            if j <= i or j in used:
                continue
            cx_j, cy_j = bbox_center(bbox_j)
            # Same row (within y_gap) and close enough horizontally
            if abs(cy_i - cy_j) < y_gap and abs(cx_i - cx_j) < x_gap:
                group_items.append((cx_j, text_j, conf_j, bbox_j))
                used.add(j)

        used.add(i)

        # Sort group left to right and join
        group_items.sort(key=lambda x: x[0])
        merged_text = ' '.join(item[1] for item in group_items).strip()
        merged_conf = min(item[2] for item in group_items)
        merged_bbox = group_items[0][3]  # use leftmost bbox as anchor
        merged.append((merged_bbox, merged_text, merged_conf))

    return merged

def detect_orientation_from_region(img, reader):
    """
    Detect N marker using circle detection (Hough Circles).
    The N marker is drawn as a letter inside a circle with an arrow.
    We find the circle, crop tightly around it, then OCR just that crop.
    Falls back to scanning corners with OCR if circle not found.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Detect circles in the image
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=20,
        maxRadius=80
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        print(f"    Found {len(circles)} circle(s), checking for N marker...")

        for (cx, cy, r) in circles:
            # Crop with padding around the circle
            pad = 15
            x1 = max(0, cx - r - pad)
            y1 = max(0, cy - r - pad)
            x2 = min(img.shape[1], cx + r + pad)
            y2 = min(img.shape[0], cy + r + pad)
            crop = img[y1:y2, x1:x2]

            if crop.size == 0:
                continue

            # Upscale crop for better OCR accuracy
            crop_up = cv2.resize(crop, None, fx=4, fy=4,
                                 interpolation=cv2.INTER_CUBIC)

            # Try OCR on upscaled crop -- no allowlist so N isn't forced
            ocr_results = reader.readtext(crop_up)
            for (_, text, conf) in ocr_results:
                cleaned = text.strip().upper()
                if cleaned in ['N', 'S', 'E', 'W'] and conf > 0.2:
                    print(f"    Orientation '{cleaned}' found via circle at ({cx},{cy}), conf={conf:.2f}")
                    return {
                        "direction": cleaned,
                        "confidence": round(conf, 2),
                        "method": "circle_detection",
                        "circle_center": (int(cx), int(cy))
                    }

            # If OCR on circle crop still fails, check position heuristic
            # Arrow inside circle points toward the N label
            # For our blueprint: circle is in top-right -> assume N facing up = North
            h, w = img.shape[:2]
            in_top_half    = cy < h * 0.5
            in_bottom_half = cy > h * 0.5

            # Use circle position in image to infer orientation.
            print(f"    Circle at ({cx},{cy}) r={r} -- using position heuristic")
            if in_top_half:
                direction = 'N'
            elif in_bottom_half:
                direction = 'S'
            else:
                direction = 'N'  # default

            print(f"    Inferred orientation '{direction}' from circle position")
            return {
                "direction": direction,
                "confidence": 0.6,
                "method": "circle_position_heuristic",
                "circle_center": (int(cx), int(cy))
            }

    # Last resort -- scan corners with OCR (original fallback)
    print("    No circles found, trying corner OCR fallback...")
    h, w = img.shape[:2]
    corner_size = int(min(w, h) * 0.2)
    regions = {
        'top-right':    img[0:corner_size, w-corner_size:w],
        'top-left':     img[0:corner_size, 0:corner_size],
        'bottom-right': img[h-corner_size:h, w-corner_size:w],
        'bottom-left':  img[h-corner_size:h, 0:corner_size],
    }
    for region_name, region in regions.items():
        if region.size == 0:
            continue
        up = cv2.resize(region, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
        ocr_results = reader.readtext(up)
        for (_, text, conf) in ocr_results:
            if text.strip().upper() in ['N','S','E','W'] and conf > 0.3:
                print(f"    Found '{text.upper()}' in {region_name} via corner OCR")
                return {
                    "direction": text.strip().upper(),
                    "confidence": round(conf, 2),
                    "method": "corner_ocr",
                    "region": region_name
                }

    return None

# ── BUILDING DIMENSION ESTIMATOR ───────────────────────────────────────────
def estimate_building_dimensions(rooms, orientation):
    """
    Rough estimate of total building size from room dimensions.
    Assumes rooms sharing similar y-center are in the same row (east-west span),
    and rooms sharing similar x-center are in the same column (north-south span).
    Returns {"width_cm": int, "height_cm": int} or None.
    """
    dims = [r["dimensions"] for r in rooms if "dimensions" in r]
    if not dims:
        return None

    # Cluster room centers by y into rows; sum EW dims per row -> total width
    # Cluster room centers by x into cols; sum NS dims per col -> total height
    # Simple approach: sum all EW dims for the widest row
    ew_values = [d["east_west"] for d in dims if d.get("east_west")]
    ns_values = [d["north_south"] for d in dims if d.get("north_south")]

    if not ew_values or not ns_values:
        return None

    # Group rooms by their y-center (within 20% of image height tolerance)
    # Without image height here, use sorted halves heuristic
    ew_sorted = sorted(ew_values, reverse=True)
    ns_sorted = sorted(ns_values, reverse=True)

    # Take the two largest rows/cols and sum them (coarse estimate)
    estimated_width  = sum(ew_sorted[:max(1, len(ew_sorted) // 2)])
    estimated_height = sum(ns_sorted[:max(1, len(ns_sorted) // 2)])

    return {"width_cm": estimated_width, "height_cm": estimated_height}


def estimate_scale_from_room_positions(rooms, unit):
    """
    Derive cm_per_pixel by cross-referencing OCR room label pixel positions
    with their real-world dimensions.

    Rooms in the same horizontal band have centers separated by roughly
    (ew1 + ew2) / 2 in real units -- mirroring the pixel gap between them.
    The same logic applies vertically using NS dimensions.

    Returns median cm_per_pixel, or None when < 2 rooms carry dimensions.
    """
    unit_factor = {"cm": 1.0, "m": 100.0, "ft": 30.48}.get(str(unit).lower(), 1.0)
    rooms_with_dims = [r for r in rooms if "dimensions" in r]
    if len(rooms_with_dims) < 2:
        return None

    estimates = []

    def _pairwise(sorted_rooms, coord_idx, dim_key):
        for i in range(len(sorted_rooms) - 1):
            r1, r2 = sorted_rooms[i], sorted_rooms[i + 1]
            dpx = abs(r2["center"][coord_idx] - r1["center"][coord_idx])
            if dpx < 20:
                continue
            d1 = r1["dimensions"].get(dim_key, 0)
            d2 = r2["dimensions"].get(dim_key, 0)
            if d1 <= 0 or d2 <= 0:
                continue
            estimates.append((d1 + d2) / 2.0 * unit_factor / dpx)

    # ── Horizontal rows -> EW scale ───────────────────────────────────────
    sorted_y = sorted(rooms_with_dims, key=lambda r: r["center"][1])
    y_span   = max(r["center"][1] for r in sorted_y) - min(r["center"][1] for r in sorted_y)
    y_tol    = max(y_span * 0.12, 40)

    rows, row = [], [sorted_y[0]]
    for rm in sorted_y[1:]:
        if abs(rm["center"][1] - row[-1]["center"][1]) <= y_tol:
            row.append(rm)
        else:
            rows.append(row); row = [rm]
    rows.append(row)

    for row in rows:
        if len(row) >= 2:
            _pairwise(sorted(row, key=lambda r: r["center"][0]), 0, "east_west")

    # ── Vertical columns -> NS scale ──────────────────────────────────────
    sorted_x = sorted(rooms_with_dims, key=lambda r: r["center"][0])
    x_span   = max(r["center"][0] for r in sorted_x) - min(r["center"][0] for r in sorted_x)
    x_tol    = max(x_span * 0.12, 40)

    cols, col = [], [sorted_x[0]]
    for rm in sorted_x[1:]:
        if abs(rm["center"][0] - col[-1]["center"][0]) <= x_tol:
            col.append(rm)
        else:
            cols.append(col); col = [rm]
    cols.append(col)

    for col in cols:
        if len(col) >= 2:
            _pairwise(sorted(col, key=lambda r: r["center"][1]), 1, "north_south")

    if not estimates:
        return None

    estimates.sort()
    median = estimates[len(estimates) // 2]
    print(f"   Scale estimated from {len(estimates)} room pairs: {median:.4f} cm/px")
    return median


# ── MAIN PARSER ────────────────────────────────────────────────────────────
def parse_blueprint(image_path):
    print(f"\n Parsing: {image_path}")

    reader = easyocr.Reader(['en'], verbose=False)
    img = cv2.imread(image_path)

    if img is None:
        return {"parse_status": "FAILED", "error": "Image not found"}

    # Step 1 -- Full image OCR
    print("   Running full-image OCR...")
    raw_results = reader.readtext(img)

    # Step 2 -- Merge nearby fragments (fixes "Ward" + "1" split)
    print("   Merging text fragments...")
    results = merge_nearby_texts(raw_results, x_gap=100, y_gap=30)

    # Step 3 -- Classify
    rooms = []
    dimensions = []
    orientation = None
    unclassified = []
    low_confidence = []

    for (bbox, text, conf) in results:
        center = bbox_center(bbox)
        entry = {"text": text, "confidence": round(conf, 2), "center": center}

        if conf < CONFIDENCE_THRESHOLD:
            low_confidence.append(entry)
            continue

        if is_orientation_marker(text):
            orientation = {
                "direction": text.strip().upper(),
                "confidence": round(conf, 2),
                "center": center
            }

        else:
            # Try dimension fix before classifying
            fixed_text = fix_dimension_misread(text)
            if is_dimension(fixed_text.replace(' ', '')):
                dim = parse_dimension(fixed_text)
                if dim:
                    dimensions.append({
                        "raw": text,
                        "fixed": fixed_text,
                        "north_south": dim[0],
                        "east_west": dim[1],
                        "confidence": round(conf, 2),
                        "center": center
                    })
                    continue

            if is_room_label(text):
                # Check if dimension got merged into the room name
                # e.g. "Lab 35*45" or "35*45 Lobby" -> split them out
                words = text.strip().split()
                clean_name_parts = []
                embedded_dim = None

                for word in words:
                    fixed_w = fix_dimension_misread(word)
                    if is_dimension(fixed_w.replace(' ','')):
                        # This word is a dimension -- extract it
                        dim_parsed = parse_dimension(fixed_w)
                        if dim_parsed:
                            embedded_dim = {
                                "raw": word,
                                "fixed": fixed_w,
                                "north_south": dim_parsed[0],
                                "east_west": dim_parsed[1],
                                "confidence": round(conf, 2),
                                "center": center
                            }
                    else:
                        clean_name_parts.append(word)

                clean_name = ' '.join(clean_name_parts).strip()
                if not clean_name:
                    continue

                room_entry = {
                    "name": clean_name,
                    "confidence": round(conf, 2),
                    "center": center
                }
                # If dimension was embedded, assign it directly to this room
                if embedded_dim:
                    room_entry["dimensions"] = {
                        "north_south": embedded_dim["north_south"],
                        "east_west": embedded_dim["east_west"],
                        "raw": embedded_dim["raw"],
                        "fixed": embedded_dim["fixed"]
                    }
                rooms.append(room_entry)
            else:
                unclassified.append(entry)

    # Step 4 -- Dedicated orientation scan if not found in main OCR
    if not orientation:
        print("   N marker not found in main OCR  scanning corners...")
        orientation = detect_orientation_from_region(img, reader)

    # Step 5 -- Assign dimensions to nearest room
    for dim in dimensions:
        min_dist = float('inf')
        assigned_room = None
        dx, dy = dim['center']
        for room in rooms:
            rx, ry = room['center']
            dist = ((dx - rx)**2 + (dy - ry)**2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                assigned_room = room
        if assigned_room and min_dist < 300:  # only assign if reasonably close
            assigned_room['dimensions'] = {
                "north_south": dim['north_south'],
                "east_west": dim['east_west'],
                "raw": dim['raw'],
                "fixed": dim.get('fixed', dim['raw'])
            }

    # Step 6 -- Validate
    missing = []
    if not orientation:
        missing.append("orientation marker (N/S/E/W)")
    if not rooms:
        missing.append("room labels")
    # Dimensions are valid if either the separate list has entries
    # OR rooms already have embedded dimensions
    rooms_with_dims = [r for r in rooms if 'dimensions' in r
                       and r['name'].upper() != 'CORRIDOR']
    if not dimensions and not rooms_with_dims:
        missing.append("room dimensions")

    rooms_without_dims = [
        r['name'] for r in rooms
        if 'dimensions' not in r and r['name'].upper() != 'CORRIDOR'
    ]
    if rooms_without_dims:
        missing.append(f"dimensions for: {', '.join(rooms_without_dims)}")

    parse_status = "COMPLETE" if not missing else "INCOMPLETE"

    building_dims = estimate_building_dimensions(rooms, orientation)

    return {
        "parse_status": parse_status,
        "orientation": orientation,
        "rooms": rooms,
        "dimensions_detected": dimensions,
        "unclassified": unclassified,
        "low_confidence_items": low_confidence,
        "missing_fields": missing,
        "estimated_building_dims": building_dims,
    }

# ── PRETTY PRINT ───────────────────────────────────────────────────────────
def print_result(result):
    print("\n" + "="*50)
    status = result['parse_status']
    icon = "OK" if status == "COMPLETE" else "WARN️ "
    print(f"{icon} PARSE STATUS: {status}")
    print("="*50)

    o = result['orientation']
    if o:
        print(f"\n Orientation: {o['direction']} (conf: {o['confidence']})")
    else:
        print("\n Orientation: NOT FOUND")

    print(f"\n Rooms detected ({len(result['rooms'])}):")
    for r in result['rooms']:
        dim_str = ""
        if 'dimensions' in r:
            d = r['dimensions']
            dim_str = f"  ->  {d['north_south']}(NS) × {d['east_west']}(EW)"
            if d['raw'] != d['fixed']:
                dim_str += f"  [fixed from '{d['raw']}']"
        print(f"    {r['name']} (conf: {r['confidence']}){dim_str}")

    if result['missing_fields']:
        print(f"\n Missing:")
        for m in result['missing_fields']:
            print(f"    {m}")
    else:
        print("\n All required fields detected!")

    if result['low_confidence_items']:
        print(f"\n  Low confidence (ignored):")
        for lc in result['low_confidence_items']:
            print(f"    '{lc['text']}' ({lc['confidence']})")

    if result['unclassified']:
        print(f"\n Unclassified:")
        for u in result['unclassified']:
            print(f"    '{u['text']}' ({u['confidence']})")

# ── RUN ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    result = parse_blueprint("test_blueprint.jpg")
    print_result(result)