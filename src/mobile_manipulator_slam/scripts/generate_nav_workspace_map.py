#!/usr/bin/env python3
"""
Generate a pixel-perfect nav_workspace.pgm map directly from SDF geometry.
No SLAM drift. Map is guaranteed to match the world.

World bounds : X ∈ [-8, 8],  Y ∈ [-6, 6]  (16 m × 12 m)
Resolution   : 0.05 m / pixel
Map size     : 320 × 240 pixels
Origin (YAML): (-8, -6, 0) — bottom-left corner of world
PGM row 0    : top of image  = world Y = +6
PGM col 0    : left of image = world X = -8

Pixel values : 254 = free, 0 = occupied (wall/obstacle)
"""

import os, struct, math

# ── World parameters ──────────────────────────────────────────────────────────
RES       = 0.05          # m / pixel
WORLD_MIN_X, WORLD_MAX_X = -8.0, 8.0
WORLD_MIN_Y, WORLD_MAX_Y = -6.0, 6.0
WIDTH  = int(round((WORLD_MAX_X - WORLD_MIN_X) / RES))   # 320
HEIGHT = int(round((WORLD_MAX_Y - WORLD_MIN_Y) / RES))   # 240

FREE     = 254
OCCUPIED =   0

# ── World geometry (cx, cy, sx, sy) ──────────────────────────────────────────
# All sizes are the FULL side length of the box in world units.
WALLS = [
    # Outer walls
    ( 0.0,  6.0, 16.4, 0.2),   # north
    ( 0.0, -6.0, 16.4, 0.2),   # south
    ( 8.0,  0.0,  0.2, 12.4),  # east
    (-8.0,  0.0,  0.2, 12.4),  # west
    # Central divider (X = 0)
    # South segment Y ∈ [-6.0, -2.5]
    ( 0.0, -4.25, 0.2, 3.5),
    # Middle segment Y ∈ [-1.0, 2.5]
    ( 0.0,  0.75, 0.2, 3.5),
    # North stub Y ∈ [5.5, 6.0]
    ( 0.0,  5.75, 0.2, 0.5),
    # Zone-A pillars
    (-3.5,  3.0,  0.5, 0.5),
    (-3.5, -3.0,  0.5, 0.5),
    # Tables (solid obstacles in map)
    ( 5.0,  4.0,  1.2, 0.8),   # pick table
    ( 5.0, -4.0,  1.2, 0.8),   # place table
]

# ── Helper: world coords → pixel (col, row) ───────────────────────────────────
def w2p(wx, wy):
    col = (wx - WORLD_MIN_X) / RES
    row = (WORLD_MAX_Y - wy) / RES
    return col, row

# ── Build map array (row-major, row 0 = top) ─────────────────────────────────
data = bytearray([FREE] * (WIDTH * HEIGHT))

def fill_rect(cx, cy, sx, sy):
    """Fill an axis-aligned rectangle as occupied."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    # pixel col range
    c0 = max(0, int(math.floor((x0 - WORLD_MIN_X) / RES)))
    c1 = min(WIDTH  - 1, int(math.ceil ((x1 - WORLD_MIN_X) / RES)))
    # pixel row range (Y is flipped)
    r0 = max(0, int(math.floor((WORLD_MAX_Y - y1) / RES)))
    r1 = min(HEIGHT - 1, int(math.ceil ((WORLD_MAX_Y - y0) / RES)))
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            data[r * WIDTH + c] = OCCUPIED

for wall in WALLS:
    fill_rect(*wall)

# ── Determine output directory ────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Try project maps directory first
candidate = os.path.join(
    SCRIPT_DIR,
    '../../../../src/mobile_manipulator_slam/maps')
if os.path.isdir(candidate):
    OUT_DIR = os.path.realpath(candidate)
else:
    # fallback: same directory as script
    OUT_DIR = SCRIPT_DIR

PGM_FILE  = os.path.join(OUT_DIR, 'nav_workspace.pgm')
YAML_FILE = os.path.join(OUT_DIR, 'nav_workspace.yaml')

# ── Write PGM (P5 binary) ─────────────────────────────────────────────────────
with open(PGM_FILE, 'wb') as f:
    header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
    f.write(header.encode('ascii'))
    f.write(bytes(data))

print(f"Wrote PGM : {PGM_FILE}  ({WIDTH}×{HEIGHT} px)")

# ── Write YAML ────────────────────────────────────────────────────────────────
yaml_content = f"""image: nav_workspace.pgm
mode: trinary
resolution: {RES}
origin: [{WORLD_MIN_X:.3f}, {WORLD_MIN_Y:.3f}, 0.000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(YAML_FILE, 'w') as f:
    f.write(yaml_content)

print(f"Wrote YAML: {YAML_FILE}")
print()
print("Map summary:")
print(f"  World     : X∈[{WORLD_MIN_X},{WORLD_MAX_X}]  Y∈[{WORLD_MIN_Y},{WORLD_MAX_Y}]")
print(f"  Size      : {WIDTH} × {HEIGHT} pixels  @  {RES} m/px")
print(f"  Wide door : Y ∈ [ 2.5,  5.5]  → 3.0 m opening")
print(f"  Narrow gap: Y ∈ [-2.5, -1.0]  → 1.5 m opening")
print(f"  Spawn     : (-6.0, 0.0)")
print(f"  Pick table: ( 5.0, 4.0)  — via wide door")
print(f"  Place tbl : ( 5.0,-4.0)  — via narrow passage (high cost) or wide door (longer)")
