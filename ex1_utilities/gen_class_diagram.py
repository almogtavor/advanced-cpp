"""Generate UML class diagram as PDF using matplotlib."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

FIG_W, FIG_H = 14, 18
FONT = {"family": "monospace", "size": 9}
TITLE_FONT = {"family": "sans-serif", "size": 10.5, "weight": "bold"}
LH = 0.30  # line height

C_IFACE = "#dceaff"
C_CLASS = "#ffffcc"
C_MOCK  = "#d9ffd9"
C_DATA  = "#ffe6e6"
C_UTIL  = "#ece0ff"
C_SIM   = "#fff0dd"

fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
ax.set_xlim(0, FIG_W)
ax.set_ylim(0, FIG_H)
ax.set_aspect("equal")
ax.axis("off")
fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)


def draw_box(x, y, w, h, title, attrs, methods, color, stereo=None):
    rect = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                          facecolor=color, edgecolor="black", linewidth=0.9)
    ax.add_patch(rect)
    ty = y + h - 0.35
    if stereo:
        ax.text(x + w/2, ty + 0.14, f"\u00AB{stereo}\u00BB",
                ha="center", va="center", fontsize=8, fontstyle="italic")
        ty -= 0.16
    ax.text(x + w/2, ty, title, ha="center", va="center", **TITLE_FONT)
    sep_y = y + h - (0.65 if stereo else 0.50)
    ax.plot([x + 0.08, x + w - 0.08], [sep_y, sep_y], "k-", lw=0.5)
    cy = sep_y - 0.25
    for a in attrs:
        ax.text(x + 0.15, cy, a, va="center", **FONT)
        cy -= LH
    if attrs and methods:
        cy -= 0.05
        ax.plot([x + 0.08, x + w - 0.08], [cy, cy], "k-", lw=0.5)
        cy -= 0.20
    for m in methods:
        ax.text(x + 0.15, cy, m, va="center", **FONT)
        cy -= LH


def arrow_line(x1, y1, x2, y2, dashed=False, style="-|>"):
    ls = "--" if dashed else "-"
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color="black", lw=1.0,
                                linestyle=ls))


def diamond(x1, y1, x2, y2, filled=True):
    ax.plot([x1, x2], [y1, y2], "k-", lw=1.0)
    dx, dy = x2 - x1, y2 - y1
    length = (dx**2 + dy**2) ** 0.5
    if length == 0:
        return
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    s = 0.18
    pts = [(x1, y1),
           (x1 + ux*s + px*s*0.5, y1 + uy*s + py*s*0.5),
           (x1 + ux*s*2, y1 + uy*s*2),
           (x1 + ux*s - px*s*0.5, y1 + uy*s - py*s*0.5)]
    ax.add_patch(plt.Polygon(pts, closed=True,
                             fc="black" if filled else "white",
                             ec="black", lw=0.7))


# =================== ROW 1: Interfaces ===================
iy = 15.9
ih = 1.0
draw_box(0.2, iy, 3.8, ih, "ILidarSensor", [],
         ["+ scan(xy, pitch): LidarFrame"],
         C_IFACE, "interface")
draw_box(4.5, iy, 4.3, ih, "IPositionSensor", [],
         ["+ get_position(): Position", "+ get_yaw(): Angle"],
         C_IFACE, "interface")
draw_box(9.3, iy, 4.5, ih, "IMovementDriver", [],
         ["+ rotate(dir, angle): MoveResult",
          "+ advance(len): MoveResult",
          "+ elevate(len): MoveResult"],
         C_IFACE, "interface")

# =================== ROW 2: Mocks ===================
my = 13.5
mh = 1.55
draw_box(0.1, my, 3.9, mh, "LidarMock",
         ["- world_: MockWorld&"],
         ["+ scan() override", "- cast_ray()",
          "- compute_grid_side()"],
         C_MOCK)
draw_box(4.5, my, 4.0, mh, "PositionMock",
         ["- world_: MockWorld&"],
         ["+ get_position() override",
          "+ get_yaw() override"],
         C_MOCK)
draw_box(9.1, my, 4.7, mh, "MovementMock",
         ["- world_: MockWorld&"],
         ["+ rotate() override", "+ advance() override",
          "+ elevate() override", "- try_translate()"],
         C_MOCK)

# inheritance: mocks -> interfaces
for mx in [2.1, 6.5, 11.45]:
    arrow_line(mx, my + mh, mx, iy, dashed=True, style="-|>")

# =================== ROW 3: MockWorld ===================
mw_x, mw_y, mw_w, mw_h = 3.5, 11.0, 5.8, 1.9
draw_box(mw_x, mw_y, mw_w, mw_h, "MockWorld",
         ["+ truth: BuildingTruth",
          "+ drone_config: DroneConfig",
          "+ position: Position",
          "+ yaw: Angle",
          "+ collided: bool"],
         [], C_DATA)

# aggregation: mocks -> MockWorld
for mx in [2.1, 6.5, 11.45]:
    diamond(mx, my, 6.4, mw_y + mw_h, filled=False)

# =================== ROW 4: Simulator (left) + Drone (right) ===================
sx, sy, sw, sh = 0.5, 7.6, 6.0, 2.5
draw_box(sx, sy, sw, sh, "Simulator",
         ["- mission_: MissionConfig",
          "- world_: MockWorld",
          "- known_map_: BuildingMap",
          "- drone_: Drone",
          "- pos/lidar/move mocks"],
         ["+ run(max_cmds): SimulationReport",
          "- score_against_truth()"],
         C_SIM)

# Simulator owns MockWorld
diamond(3.5, sy + sh, 5.0, mw_y, filled=True)

dx_, dy_, dw, dh = 7.5, 7.0, 5.8, 4.0
draw_box(dx_, dy_, dw, dh, "Drone",
         ["- map_: BuildingMap&",
          "- cfg_: DroneConfig&",
          "- state_: State (enum)",
          "- last_position_: Position",
          "- yaw_known_: Angle",
          "- pending_moves_: deque"],
         ["+ next_command(): DroneCommand",
          "+ on_location(pos, yaw)",
          "+ on_scan(LidarFrame)",
          "+ on_move_result(MoveResult)",
          "- apply_full_scan_to_map()",
          "- bfs_to_frontier()",
          "- plan_next_target()"],
         C_CLASS)

# Simulator owns Drone
diamond(sx + sw, sy + sh * 0.5, dx_, dy_ + dh * 0.5, filled=True)

# =================== ROW 5: VoxelGrid + BuildingTruth + BuildingMap ===================
vx, vy, vw, vh = 0.2, 3.2, 4.0, 2.6
draw_box(vx, vy, vw, vh, "VoxelGrid",
         ["- cell_size_: Length",
          "- origin_: Position",
          "- nx_, ny_, nz_: int",
          "- data_: vector<int8_t>"],
         ["+ cell_at(Pos): Cell",
          "+ center_of(Cell): Position",
          "+ get(Cell) / set(Cell, v)",
          "+ in_bounds(Cell): bool"],
         C_CLASS)

btx, bty, btw, bth = 0.2, 1.3, 4.0, 1.3
draw_box(btx, bty, btw, bth, "BuildingTruth",
         ["- grid_: VoxelGrid"],
         ["+ at(Cell) / at(Position): int8_t",
          "+ is_occupied() / is_empty()"],
         C_CLASS)
diamond(2.2, bty + bth, 2.2, vy, filled=True)

bmx, bmy, bmw, bmh = 4.8, 3.2, 4.5, 1.8
draw_box(bmx, bmy, bmw, bmh, "BuildingMap",
         ["- grid_: VoxelGrid"],
         ["+ get(Pos) / set(Pos, v)",
          "+ get_cell(Cell) / set_cell(Cell, v)",
          "- point_in_polygon()"],
         C_CLASS)
# BuildingMap has VoxelGrid
diamond(bmx, bmy + bmh * 0.5, vx + vw, vy + vh * 0.3, filled=True)

# Simulator owns BuildingTruth
diamond(sx + 0.5, sy, 2.2, bty + bth, filled=True)
# Simulator owns BuildingMap
diamond(sx + sw * 0.6, sy, bmx + bmw * 0.5, bmy + bmh, filled=True)

# Drone uses BuildingMap
arrow_line(dx_ + dw * 0.3, dy_, bmx + bmw, bmy + bmh * 0.4,
           dashed=True, style="-|>")
ax.text(9.6, 6.1, "uses", fontsize=9, fontstyle="italic", color="gray",
        rotation=45)

# =================== ROW 6: Utilities ===================
draw_box(0.2, -0.8, 4.3, 1.3, "ConfigParser / MapIO", [],
         ["+ load_drone_config(path, out)",
          "+ load_mission_config(path, out)",
          "+ load_truth() / save_map()"],
         C_UTIL)

draw_box(5.0, -0.8, 3.8, 1.3, "Logger",
         ["- level_: LogLevel", "- file_: ofstream"],
         ["+ instance(): Logger&",
          "+ info() / warning() / error()"],
         C_UTIL)

draw_box(9.3, -0.8, 4.5, 1.5, "Data Types",
         ["Position {x, y, z : Length}",
          "Cell {x, y, z : int}",
          "LidarFrame {side, fov, dist[]}",
          "DroneCommand {kind, angle, len}",
          "SimulationReport {score, ...}"],
         [], C_DATA)

# =================== Legend (top-left corner, small) ===================
# Keep it compact and out of the way
ax.text(0.3, 17.65, "Legend", fontsize=10, fontweight="bold")
for i, (c, label) in enumerate([
    (C_IFACE, "Interface"), (C_CLASS, "Core class"),
    (C_MOCK, "Mock impl."), (C_DATA, "Data type"),
    (C_UTIL, "Utility"), (C_SIM, "Orchestrator"),
]):
    lx = 0.3 + (i % 3) * 2.8
    ly = 17.35 - (i // 3) * 0.35
    ax.add_patch(FancyBboxPatch((lx, ly - 0.1), 0.25, 0.2,
                                boxstyle="round,pad=0.02",
                                facecolor=c, edgecolor="black", lw=0.5))
    ax.text(lx + 0.35, ly, label, va="center", fontsize=8.5)

# Arrow legend - horizontal row
aly = 16.7
ax.text(0.3, aly + 0.1, "Arrows:", fontsize=9, fontweight="bold")
items = [
    (True, "-|>", "implements", False),
    (False, "-", "composition", True),
    (False, "-", "aggregation", False),
    (True, "-|>", "uses (ref)", False),
]
alx = 2.2
for dashed, style, label, fill_d in items:
    if "comp" in label or "agg" in label:
        ax.plot([alx, alx + 0.6], [aly, aly], "k-", lw=0.9)
        diamond(alx, aly, alx + 0.6, aly, filled=fill_d)
    else:
        arrow_line(alx, aly, alx + 0.6, aly, dashed=dashed, style=style)
    ax.text(alx + 0.75, aly, label, va="center", fontsize=8)
    alx += 3.0

fig.savefig("/Users/almogtavor/Documents/code/un/advanced_cpp/ex1/class_diagram.pdf",
            bbox_inches="tight", dpi=200)
print("class_diagram.pdf written")
