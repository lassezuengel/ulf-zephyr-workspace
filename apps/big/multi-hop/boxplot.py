#!/usr/bin/env python3
"""
boxplot.py – Generate a box plot from a CSV file with optional comments.

CONFIGURATION
=============
Edit the CONFIG dict below to customise the plot to your needs.
All settings have sensible defaults and are documented inline.
"""

import csv
import sys
import pathlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ─── CONFIGURATION ────────────────────────────────────────────────────────────

CONFIG = {
    # --- Input / Output ---
    "input_file": "data.csv",          # path to CSV (override with CLI arg)
    "output_file": None,               # None → show interactively; "plot.png" → save

    # --- Labels ---
    "x_label":     "Protocol", # "Antenna / Device",
    "y_label":     "NIT",               # unit on the y-axis
    "column_labels": {                  # rename CSV column headers for display
        "nitNrf": "nRF52840DK (Saclay)",
        "nitDWM": "DWM3001 (Toulouse)",
        "clockSynNrf": "nRF52840DK (Saclay)",
        "clockSynDWM": "DWM3001 (Toulouse)",
        "nitRUDP": "RUDP",
        "nitTCP": "TCP"
    },

    # --- Layout ---
    "figure_size":   (6, 4),            # inches (width, height)
    "dpi":           150,
    "show_points":   False,             # overlay individual data points (jittered)
    "show_means":    False,             # draw a marker for the mean value
    "show_grid":     True,
    "y_axis_zero":   True,             # force y-axis to start at 0
    "font_family":   "Times New Roman", # font family for all text
    "y_axis_top_margin": 1.3,          # add extra space above max value (None to disable)
    "y_axis_as_percent": True,         # if True, multiply all y values by 100 and add % to y_label

    # --- Colours (any matplotlib colour string or hex) ---
    # Per-box color schemes: each entry is a dict with box_color, median_color, mean_color, point_color
    # If not specified, falls back to the single-color scheme below
    "color_schemes": [
        {  # First box (blue)
            "box_color":     "#4C9BE8",
            "median_color":  "#1E5A96",
            "mean_color":    "#2E7DB6",
            "point_color":   "#4C9BE8",
        },
        {  # Second box (red)
            "box_color":     "#E84C4C",
            "median_color":  "#C41E3A",
            "mean_color":    "#E24C4C",
            "point_color":   "#E84C4C",
        },
        {  # Third box (green)
            "box_color":     "#2ECC71",
            "median_color":  "#1A7C4F",
            "mean_color":    "#27AE60",
            "point_color":   "#2ECC71",
        },
        {  # Fourth box (orange)
            "box_color":     "#F39C12",
            "median_color":  "#BA4A00",
            "mean_color":    "#E67E22",
            "point_color":   "#F39C12",
        },
    ],
    # Fallback colors if color_schemes is not used
    "box_color":     "#4C9BE8",         # fill colour of boxes
    "median_color":  "#E84C4C",         # median line colour
    "mean_color":    "#2ECC71",         # mean marker colour
    "point_color":   "#333333",         # jittered point colour
    "point_alpha":   0.45,              # transparency of individual points

    # --- Statistics annotation ---
    "annotate_n":    False,             # print sample size (n=…) below each box
    "annotate_median": False,           # print median value above each box
}

# ─── END OF CONFIGURATION ─────────────────────────────────────────────────────


def load_csv(filepath: str) -> dict[str, list[float]]:
    """Read a CSV file, skip comment lines (#), and return column data."""
    path = pathlib.Path(filepath)
    if not path.exists():
        sys.exit(f"Error: file not found – {filepath}")

    columns: dict[str, list[float]] = {}
    headers: list[str] = []

    with path.open(newline="", encoding="utf-8") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            reader = csv.reader([line])
            row = next(reader)

            if not headers:
                headers = [h.strip() for h in row]
                for h in headers:
                    columns[h] = []
                continue

            for i, cell in enumerate(row):
                if i >= len(headers):
                    break
                cell = cell.strip()
                if cell:
                    try:
                        columns[headers[i]].append(float(cell))
                    except ValueError:
                        print(f"Warning: could not parse '{cell}' as float – skipped.")

    if not columns:
        sys.exit("Error: no data columns found in the CSV.")

    return columns


def apply_font_settings(cfg: dict) -> None:
    requested_family = cfg.get("font_family", "sans-serif")

    if requested_family == "Times New Roman":
        plt.rcParams["font.family"] = "serif"
        plt.rcParams["font.serif"] = ["Times New Roman", "Times", "DejaVu Serif"]
    else:
        plt.rcParams["font.family"] = requested_family


def make_boxplot(data: dict[str, list[float]], cfg: dict) -> None:
    apply_font_settings(cfg)

    column_labels = cfg.get("column_labels", {})
    labels  = [column_labels.get(k, k) for k in data]
    series  = [np.array(v) for v in data.values()]
    y_axis_as_percent = cfg.get("y_axis_as_percent", cfg.get("y_axis_as_percentage", False))
    if y_axis_as_percent:
        series = [s * 100 for s in series]

    fig, ax = plt.subplots(figsize=cfg["figure_size"], dpi=cfg["dpi"])

    bp = ax.boxplot(
        series,
        labels=labels,
        patch_artist=True,
        medianprops=dict(color=cfg["median_color"], linewidth=2),
        whiskerprops=dict(linewidth=1),
        capprops=dict(linewidth=1),
        flierprops=dict(marker="x", markersize=4, alpha=0.5),
    )

    # Get color schemes if defined, otherwise use fallback colors
    color_schemes = cfg.get("color_schemes", None)

    for i, patch in enumerate(bp["boxes"]):
        if color_schemes and i < len(color_schemes):
            scheme = color_schemes[i]
            patch.set_facecolor(scheme["box_color"])
            # Update median line color for this box
            bp["medians"][i].set_color(scheme["median_color"])
            bp["medians"][i].set_linewidth(1)
        else:
            patch.set_facecolor(cfg["box_color"])
        patch.set_alpha(0.7)

    if cfg["show_points"]:
        for i, s in enumerate(series, start=1):
            x_jitter = np.random.uniform(-0.12, 0.12, size=len(s)) + i
            point_color = cfg["point_color"]
            if color_schemes and (i - 1) < len(color_schemes):
                point_color = color_schemes[i - 1]["point_color"]
            ax.scatter(x_jitter, s,
                       color=point_color,
                       alpha=cfg["point_alpha"],
                       s=18, zorder=3)

    if cfg["show_means"]:
        for i, s in enumerate(series, start=1):
            mean_color = cfg["mean_color"]
            if color_schemes and (i - 1) < len(color_schemes):
                mean_color = color_schemes[i - 1]["mean_color"]
            ax.scatter(i, np.mean(s),
                       marker="D", s=40,
                       color=mean_color,
                       zorder=4, label="mean" if i == 1 else "")
        ax.legend(fontsize=9, framealpha=0.6)

    if cfg["annotate_n"]:
        y_bottom = ax.get_ylim()[0]
        for i, s in enumerate(series, start=1):
            ax.text(i, y_bottom, f"n={len(s)}",
                    ha="center", va="bottom",
                    fontsize=8, color="#555555")

    if cfg["annotate_median"]:
        for i, s in enumerate(series, start=1):
            med = np.median(s)
            ax.text(i, med, f" {med:.4g}",
                    va="center", fontsize=8, color=cfg["median_color"])

    y_label = cfg["y_label"]

    ax.set_xlabel(cfg["x_label"], fontsize=11)
    ax.set_ylabel(y_label, fontsize=11)

    if cfg["show_grid"]:
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.5)
        ax.grid(axis="y", which="minor", linestyle=":", alpha=0.3)

    if y_axis_as_percent:
        ax.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:g}%"))

    if cfg["y_axis_zero"]:
        ax.set_ylim(bottom=0)

    if cfg["y_axis_top_margin"]:
        y_max = max(max(s) for s in series)
        ax.set_ylim(top=y_max * cfg["y_axis_top_margin"])

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()

    if cfg["output_file"]:
        fig.savefig(cfg["output_file"])
        print(f"Plot saved to {cfg['output_file']}")
    else:
        plt.show()


def main() -> None:
    cfg = CONFIG.copy()

    # Allow passing the CSV path as a command-line argument
    if len(sys.argv) > 1:
        cfg["input_file"] = sys.argv[1]
    if len(sys.argv) > 2:
        cfg["output_file"] = sys.argv[2]

    data = load_csv(cfg["input_file"])

    print("Loaded columns:")
    for col, vals in data.items():
        print(f"  {col}: {len(vals)} values  "
              f"(min={min(vals):.4g}, max={max(vals):.4g}, mean={np.mean(vals):.4g})")

    make_boxplot(data, cfg)


if __name__ == "__main__":
    main()