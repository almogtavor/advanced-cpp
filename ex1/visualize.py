#!/usr/bin/env python3
"""Drone Mapper visualization utility.

Reads map_input.txt and map_output.txt from a given directory and generates
a self-contained HTML file (visualization.html) comparing the truth map with
the drone's reconstructed output.

Usage:
    python3 visualize.py [<input_output_files_path>]

If no path is given, the current directory is used.
Requires only the Python 3 standard library (no external packages).
"""

import os
import sys
import json


def parse_map(filepath):
    """Parse a map file (map_input.txt or map_output.txt).

    Returns a dict with keys:
        cell_size (float), origin (list[float]),
        size (list[int]: [nx, ny, nz]),
        layers (dict[int, list[str]]): z_index -> list of row strings
    """
    result = {"cell_size": 10.0, "origin": [0, 0, 0], "size": [0, 0, 0], "layers": {}}
    current_layer = None
    rows_remaining = 0

    with open(filepath) as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            # When reading grid rows, don't skip '#' lines (they are walls)
            if current_layer is not None and rows_remaining > 0:
                result["layers"][current_layer].append(line)
                rows_remaining -= 1
                if rows_remaining == 0:
                    current_layer = None
                continue

            if line.startswith("#"):
                continue

            if line.startswith("cell_size"):
                result["cell_size"] = float(line.split()[1])
            elif line.startswith("origin"):
                parts = line.split()
                result["origin"] = [float(parts[1]), float(parts[2]), float(parts[3])]
            elif line.startswith("size"):
                parts = line.split()
                result["size"] = [int(parts[1]), int(parts[2]), int(parts[3])]
            elif line.startswith("layer"):
                current_layer = int(line.split()[1])
                result["layers"][current_layer] = []
                rows_remaining = result["size"][1]

    return result


CHAR_TO_NAME = {".": "empty", "#": "wall", "?": "unmapped", "_": "oob"}


def compute_score(truth, output):
    """Compute the mapping score (0-100) matching the C++ formula."""
    total = 0
    correct = 0
    incorrect = 0
    unmapped = 0

    nx, ny, nz = truth["size"]
    for z in range(nz):
        t_rows = truth["layers"].get(z, [])
        o_rows = output["layers"].get(z, [])
        for y in range(ny):
            t_row = t_rows[y] if y < len(t_rows) else ""
            o_row = o_rows[y] if y < len(o_rows) else ""
            for x in range(nx):
                oc = o_row[x] if x < len(o_row) else "?"
                tc = t_row[x] if x < len(t_row) else "."
                if oc == "_":
                    continue
                total += 1
                if oc == "?":
                    unmapped += 1
                else:
                    tc_eff = "." if tc == "_" else tc
                    if oc == tc_eff:
                        correct += 1
                    else:
                        incorrect += 1

    score = (100.0 * correct / total) if total > 0 else 0.0
    return {
        "score": round(score, 2),
        "total": total,
        "correct": correct,
        "incorrect": incorrect,
        "unmapped": unmapped,
    }


def build_html(truth, output, stats):
    """Generate a self-contained HTML string."""
    nx, ny, nz = truth["size"]

    # Prepare layer data as JSON for JavaScript
    layers_data = {}
    for z in range(nz):
        t_rows = truth["layers"].get(z, [])
        o_rows = output["layers"].get(z, [])
        truth_grid = []
        output_grid = []
        diff_grid = []
        for y in range(ny):
            t_row = t_rows[y] if y < len(t_rows) else "." * nx
            o_row = o_rows[y] if y < len(o_rows) else "?" * nx
            tr = []
            orow = []
            dr = []
            for x in range(nx):
                tc = t_row[x] if x < len(t_row) else "."
                oc = o_row[x] if x < len(o_row) else "?"
                tr.append(tc)
                orow.append(oc)
                if oc == "_":
                    dr.append("oob")
                elif oc == "?":
                    dr.append("unmapped")
                else:
                    tc_eff = "." if tc == "_" else tc
                    dr.append("correct" if oc == tc_eff else "incorrect")
            truth_grid.append(tr)
            output_grid.append(orow)
            diff_grid.append(dr)
        layers_data[z] = {
            "truth": truth_grid,
            "output": output_grid,
            "diff": diff_grid,
        }

    data_json = json.dumps(
        {
            "nx": nx,
            "ny": ny,
            "nz": nz,
            "cell_size": truth["cell_size"],
            "origin": truth["origin"],
            "stats": stats,
            "layers": layers_data,
        }
    )

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Drone Mapper Visualization</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, monospace;
       background: #1a1a2e; color: #e0e0e0; padding: 20px; }}
h1 {{ text-align: center; margin-bottom: 8px; color: #e94560; }}
.stats {{ text-align: center; margin-bottom: 16px; font-size: 14px; }}
.stats .score {{ font-size: 28px; font-weight: bold;
                color: {_score_color(stats["score"])}; }}
.stats span {{ margin: 0 12px; }}
.layer-tabs {{ text-align: center; margin-bottom: 12px; }}
.layer-tabs button {{ background: #16213e; color: #e0e0e0; border: 1px solid #0f3460;
    padding: 6px 16px; margin: 2px; cursor: pointer; border-radius: 4px; font-size: 13px; }}
.layer-tabs button.active {{ background: #e94560; color: #fff; border-color: #e94560; }}
.grids {{ display: flex; justify-content: center; gap: 24px; flex-wrap: wrap; }}
.grid-panel {{ text-align: center; }}
.grid-panel h3 {{ margin-bottom: 6px; font-size: 14px; }}
table {{ border-collapse: collapse; }}
td {{ width: 18px; height: 18px; border: 1px solid #111; font-size: 0; }}
td:hover {{ outline: 2px solid #fff; z-index: 1; position: relative; }}
.c-empty {{ background: #2d2d44; }}
.c-wall  {{ background: #e0e0e0; }}
.c-unmapped {{ background: #f5a623; }}
.c-oob   {{ background: #0f3460; }}
.d-correct   {{ background: #27ae60; }}
.d-incorrect {{ background: #e74c3c; }}
.d-unmapped  {{ background: #f5a623; }}
.d-oob       {{ background: #0f3460; }}
.legend {{ text-align: center; margin-top: 16px; font-size: 13px; }}
.legend span {{ display: inline-block; margin: 0 8px; }}
.legend .box {{ display: inline-block; width: 14px; height: 14px;
               vertical-align: middle; margin-right: 3px; border: 1px solid #333; }}
#tooltip {{ position: fixed; background: #16213e; color: #e0e0e0; padding: 6px 10px;
           border: 1px solid #0f3460; border-radius: 4px; font-size: 12px;
           pointer-events: none; display: none; z-index: 100; }}
</style>
</head>
<body>
<h1>Drone Mapper - Map Visualization</h1>
<div class="stats">
  <span class="score">{stats["score"]}%</span><br>
  <span>Correct: {stats["correct"]}</span>
  <span>Incorrect: {stats["incorrect"]}</span>
  <span>Unmapped: {stats["unmapped"]}</span>
  <span>Total in-bounds: {stats["total"]}</span>
</div>
<div class="layer-tabs" id="tabs"></div>
<div class="grids" id="grids"></div>
<div class="legend">
  <b>Map:</b>
  <span><span class="box c-empty"></span>Empty</span>
  <span><span class="box c-wall"></span>Wall</span>
  <span><span class="box c-unmapped"></span>Unmapped</span>
  <span><span class="box c-oob"></span>Out of bounds</span>
  &nbsp;&nbsp;<b>Diff:</b>
  <span><span class="box d-correct"></span>Correct</span>
  <span><span class="box d-incorrect"></span>Incorrect</span>
  <span><span class="box d-unmapped"></span>Unmapped</span>
  <span><span class="box d-oob"></span>Out of bounds</span>
</div>
<div id="tooltip"></div>
<script>
const DATA = {data_json};
const CELL_CLASS = {{'.':'c-empty','#':'c-wall','?':'c-unmapped','_':'c-oob'}};
const CELL_LABEL = {{'.':'empty','#':'wall','?':'unmapped','_':'out of bounds'}};
const DIFF_CLASS = {{'correct':'d-correct','incorrect':'d-incorrect','unmapped':'d-unmapped','oob':'d-oob'}};
const tooltip = document.getElementById('tooltip');

function makeTable(grid, classMap, labelMap, prefix) {{
  const t = document.createElement('table');
  for (let y = 0; y < DATA.ny; y++) {{
    const tr = document.createElement('tr');
    for (let x = 0; x < DATA.nx; x++) {{
      const td = document.createElement('td');
      const v = grid[y][x];
      td.className = classMap[v] || '';
      td.dataset.x = x; td.dataset.y = y; td.dataset.v = labelMap[v] || v;
      td.dataset.prefix = prefix;
      tr.appendChild(td);
    }}
    t.appendChild(tr);
  }}
  return t;
}}

function showLayer(z) {{
  document.querySelectorAll('.layer-tabs button').forEach(b =>
    b.classList.toggle('active', parseInt(b.dataset.z) === z));
  const g = document.getElementById('grids');
  g.innerHTML = '';
  const L = DATA.layers[z];

  const panels = [
    ['Truth (input)', makeTable(L.truth, CELL_CLASS, CELL_LABEL, 'Truth')],
    ['Drone Output', makeTable(L.output, CELL_CLASS, CELL_LABEL, 'Output')],
    ['Diff', makeTable(L.diff, DIFF_CLASS, {{}}, 'Diff')]
  ];
  for (const [title, tbl] of panels) {{
    const d = document.createElement('div');
    d.className = 'grid-panel';
    const h = document.createElement('h3');
    h.textContent = title;
    d.appendChild(h); d.appendChild(tbl);
    g.appendChild(d);
  }}
}}

// Tabs
const tabs = document.getElementById('tabs');
for (let z = 0; z < DATA.nz; z++) {{
  const b = document.createElement('button');
  b.textContent = 'Layer ' + z;
  b.dataset.z = z;
  b.onclick = () => showLayer(z);
  tabs.appendChild(b);
}}
showLayer(0);

// Tooltip
document.getElementById('grids').addEventListener('mousemove', e => {{
  const td = e.target.closest('td');
  if (!td) {{ tooltip.style.display = 'none'; return; }}
  const x = td.dataset.x, y = td.dataset.y;
  const cs = DATA.cell_size;
  const wx = (DATA.origin[0] + parseInt(x) * cs).toFixed(1);
  const wy = (DATA.origin[1] + parseInt(y) * cs).toFixed(1);
  tooltip.innerHTML = `<b>${{td.dataset.prefix}}</b> cell (${{x}}, ${{y}})<br>` +
    `World: (${{wx}}, ${{wy}}) cm<br>Value: ${{td.dataset.v}}`;
  tooltip.style.display = 'block';
  tooltip.style.left = (e.clientX + 14) + 'px';
  tooltip.style.top = (e.clientY + 14) + 'px';
}});
document.getElementById('grids').addEventListener('mouseleave', () =>
  tooltip.style.display = 'none');
</script>
</body>
</html>"""


def _score_color(score):
    if score >= 90:
        return "#27ae60"
    if score >= 70:
        return "#f5a623"
    return "#e74c3c"


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    input_path = os.path.join(directory, "map_input.txt")
    output_path = os.path.join(directory, "map_output.txt")
    html_path = os.path.join(directory, "visualization.html")

    if not os.path.isfile(input_path):
        print(f"ERROR: {input_path} not found", file=sys.stderr)
        return 1
    if not os.path.isfile(output_path):
        print(f"ERROR: {output_path} not found", file=sys.stderr)
        return 1

    truth = parse_map(input_path)
    output = parse_map(output_path)
    stats = compute_score(truth, output)
    html = build_html(truth, output, stats)

    with open(html_path, "w") as f:
        f.write(html)

    print(f"Visualization written to {html_path}")
    print(f"Score: {stats['score']}% "
          f"(correct={stats['correct']}, incorrect={stats['incorrect']}, "
          f"unmapped={stats['unmapped']}, total={stats['total']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
