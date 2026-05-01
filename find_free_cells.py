import numpy as np
from grid_generator import generate_grid, inflate_obstacles

grid, gw, gh = generate_grid("test_blueprint.jpg", cell_size=10)
inflated = inflate_obstacles(grid, robot_width_cells=2, safety_margin_cells=1)

print(f"Grid shape: {gh} rows x {gw} cols")
print()

# Print a section of the grid around row 25-45, col 10-25
# to see what's free near the doorways
print("=== Grid slice (rows 25-45, cols 10-25) ===")
print("     ", "".join(f"{c:<2}" for c in range(10, 26)))
for r in range(25, 46):
    row_str = "".join("░░" if inflated[r][c] == 0 else "██" for c in range(10, 26))
    print(f"r{r:02d}: {row_str}")

# Also find ALL free cells in corridor area (rows 28-40)
print("\n=== Free cells in corridor rows 28-40 ===")
free = []
for r in range(28, 41):
    for c in range(5, 95):
        if inflated[r][c] == 0:
            free.append((r, c))

if free:
    print(f"Found {len(free)} free cells")
    print(f"First few: {free[:10]}")
    print(f"Last few:  {free[-10:]}")
    # Show some in different column ranges
    left   = [(r,c) for r,c in free if c < 35]
    middle = [(r,c) for r,c in free if 35 <= c < 65]
    right  = [(r,c) for r,c in free if c >= 65]
    print(f"\nLeft area (col<35):    {left[:5]}")
    print(f"Middle area (col35-65): {middle[:5]}")
    print(f"Right area (col>65):   {right[:5]}")
else:
    print("No free cells found in corridor!")