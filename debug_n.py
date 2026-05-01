from grid_generator import generate_grid, inflate_obstacles

grid, gw, gh = generate_grid("test_blueprint.jpg", cell_size=10)
inflated = inflate_obstacles(grid, robot_width_cells=1, safety_margin_cells=0)

# Check the exact column 17 from row 20 to 50
print("=== Column 17, rows 20-50 ===")
for r in range(20, 51):
    status = "FREE" if inflated[r][17] == 0 else "WALL"
    print(f"  row {r}: {status}")

# Also check if start and goal are free
print(f"\nStart (25,17): {'FREE' if inflated[25][17]==0 else 'WALL'}")
print(f"Goal  (43,17): {'FREE' if inflated[43][17]==0 else 'WALL'}")

# Find nearest free cell to each
print("\n=== Nearest free cells to start (25,17) ===")
for r in range(20, 35):
    for c in range(10, 30):
        if inflated[r][c] == 0:
            print(f"  FREE at ({r},{c})")
            break

print("\n=== Nearest free cells to goal (43,17) ===")
for r in range(35, 50):
    for c in range(10, 30):
        if inflated[r][c] == 0:
            print(f"  FREE at ({r},{c})")
            break