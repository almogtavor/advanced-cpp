"""Generate UML sequence diagram as PDF using matplotlib."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

FIG_W, FIG_H = 11, 14
FONT = {"family": "sans-serif", "size": 8}
BOLD = {"family": "sans-serif", "size": 9, "weight": "bold"}
SMALL = {"family": "sans-serif", "size": 7}
ITALIC = {"family": "sans-serif", "size": 7, "style": "italic"}

fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
ax.set_xlim(0, FIG_W)
ax.set_ylim(0, FIG_H)
ax.set_aspect("equal")
ax.axis("off")
fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)

actors = [
    ("main", 0.9),
    ("Simulator", 2.8),
    ("Drone", 5.0),
    ("PosMock", 6.9),
    ("LidarMock", 8.5),
    ("MoveMock", 10.2),
]

TOP_Y = 13.5
BOT_Y = 0.3

for name, x in actors:
    ax.add_patch(FancyBboxPatch((x - 0.6, TOP_Y), 1.2, 0.4,
                                boxstyle="round,pad=0.04",
                                facecolor="white", edgecolor="black", lw=0.9))
    ax.text(x, TOP_Y + 0.2, name, ha="center", va="center", **BOLD)
    ax.plot([x, x], [TOP_Y, BOT_Y], color="gray", lw=0.5, ls="--")

X = {name: x for name, x in actors}


def msg(x1, x2, y, label, ret=False):
    ls = "--" if ret else "-"
    color = "#555" if ret else "black"
    ax.annotate("", xy=(x2, y), xytext=(x1, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=0.8,
                                linestyle=ls))
    ax.text((x1+x2)/2, y + 0.1, label, ha="center", va="bottom", **FONT)


def self_msg(x, y, label):
    ax.annotate("", xy=(x + 0.03, y - 0.25), xytext=(x + 0.03, y),
                arrowprops=dict(arrowstyle="-|>", color="black", lw=0.7,
                                connectionstyle="arc3,rad=-0.5"))
    ax.text(x + 0.6, y - 0.12, label, va="center", **ITALIC)


def phase(y, label, detail=""):
    ax.add_patch(FancyBboxPatch((10.55, y - 0.15), 0.35, 0.3,
                                boxstyle="round,pad=0.03",
                                facecolor="#e0ecff", edgecolor="#7799cc",
                                lw=0.5))
    ax.text(10.72, y, label, ha="center", va="center",
            fontsize=7, fontweight="bold", color="#335588")
    if detail:
        ax.text(10.72, y - 0.3, detail, ha="center", va="top",
                fontsize=6.5, color="#335588")


def bracket(y1, y2, label):
    ax.plot([1.4, 1.4], [y1, y2], color="gray", lw=0.6, ls=":")
    ax.plot([3.8, 3.8], [y1, y2], color="gray", lw=0.6, ls=":")
    ax.plot([1.4, 3.8], [y2, y2], color="gray", lw=0.6, ls=":")
    ax.text(2.6, y2 - 0.1, label, ha="center", va="top",
            fontsize=7, fontstyle="italic", color="gray")


y = 13.0

# Setup
msg(X["main"], X["Simulator"], y, "load configs + truth")
y -= 0.42
msg(X["main"], X["Simulator"], y, "Simulator(truth,cfg,mission)")
y -= 0.42
msg(X["main"], X["Simulator"], y, "run()")

# Phase 1
y -= 0.55
phase(y, "P1", "Get location")
msg(X["Simulator"], X["Drone"], y, "next_command()")
y -= 0.35
msg(X["Drone"], X["Simulator"], y, "GetLocation", ret=True)
y -= 0.4
msg(X["Simulator"], X["PosMock"], y, "get_position()")
y -= 0.32
msg(X["PosMock"], X["Simulator"], y, "Position, Yaw", ret=True)
y -= 0.38
msg(X["Simulator"], X["Drone"], y, "on_location(pos, yaw)")
y -= 0.28
self_msg(X["Drone"], y, "mark cell empty\nstate -> Scanning")

# Phase 2
y -= 0.6
phase(y, "P2", "6 cardinal scans")
scan_top = y
msg(X["Simulator"], X["Drone"], y, "next_command()")
y -= 0.35
msg(X["Drone"], X["Simulator"], y, "Scan(xy, pitch)", ret=True)
y -= 0.38
msg(X["Simulator"], X["LidarMock"], y, "scan(xy, pitch)")
y -= 0.3
self_msg(X["LidarMock"], y, "ray-cast vs truth")
y -= 0.35
msg(X["LidarMock"], X["Simulator"], y, "LidarFrame", ret=True)
y -= 0.38
msg(X["Simulator"], X["Drone"], y, "on_scan(frame)")
y -= 0.28
self_msg(X["Drone"], y, "apply_full_scan_to_map()\nupdate BuildingMap")
bracket(scan_top + 0.1, y - 0.2, "repeat x6")

# Phase 3
y -= 0.65
phase(y, "P3", "BFS + movement")
move_top = y
msg(X["Simulator"], X["Drone"], y, "next_command()")
y -= 0.28
self_msg(X["Drone"], y, "bfs_to_frontier()\nplan_next_target()")
y -= 0.5
msg(X["Drone"], X["Simulator"], y, "Rotate(dir, angle)", ret=True)
y -= 0.38
msg(X["Simulator"], X["MoveMock"], y, "rotate(dir, angle)")
y -= 0.32
msg(X["MoveMock"], X["Simulator"], y, "Ok", ret=True)
y -= 0.35
msg(X["Simulator"], X["Drone"], y, "on_move_result(Ok)")

y -= 0.45
msg(X["Simulator"], X["Drone"], y, "next_command()")
y -= 0.35
msg(X["Drone"], X["Simulator"], y, "Advance(dist)", ret=True)
y -= 0.38
msg(X["Simulator"], X["MoveMock"], y, "advance(dist)")
y -= 0.32
msg(X["MoveMock"], X["Simulator"], y, "Ok", ret=True)
y -= 0.35
msg(X["Simulator"], X["Drone"], y, "on_move_result(Ok)")
bracket(move_top + 0.1, y - 0.15, "repeat until path done -> back to P2")

# Phase 4
y -= 0.6
phase(y, "P4", "Score + output")
msg(X["Simulator"], X["Drone"], y, "next_command()")
y -= 0.35
msg(X["Drone"], X["Simulator"], y, "Finished", ret=True)
y -= 0.32
self_msg(X["Simulator"], y, "score_against_truth()")
y -= 0.42
msg(X["Simulator"], X["main"], y, "SimulationReport", ret=True)
y -= 0.35
self_msg(X["main"], y, "save map_output.txt\nprint results")

fig.savefig("/Users/almogtavor/Documents/code/un/advanced_cpp/ex1/sequence_diagram.pdf",
            bbox_inches="tight", dpi=200)
print("sequence_diagram.pdf written")
