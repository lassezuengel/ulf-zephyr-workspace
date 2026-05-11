#!/usr/bin/env python3
"""
boxplot.py – Generate a box plot or bar plot from a CSV file with optional comments.

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

    # --- Value scaling ---
    # Multiply every data value by this factor before plotting.
    # Useful for unit conversions, e.g.:
    #   1000  → ms  → µs
    #   0.001 → ms  → s
    #   1     → no change (default)
    "value_scale": 0.5,

    # --- Plot type ---
    # "boxplot" → classic box-and-whisker plot (original behaviour)
    # "barplot" → bar chart showing mean ± error bars
    "plot_type": "barplot",

    # --- Bar plot error bar options (only used when plot_type = "barplot") ---
    # "sd"  → ± one standard deviation
    # "sem" → ± standard error of the mean
    # "ci95"→ ± 95 % confidence interval (assumes normality, uses 1.96 × SEM)
    "barplot_error": "sd",

    # Width of the bars (0–1, fraction of available space)
    "barplot_bar_width": 0.5,

    # Whether to draw individual data points as a swarm/jitter overlay on bars
    # (re-uses the same show_points / point_alpha settings as the boxplot)
    "barplot_show_points": False,

    # --- Labels ---
    "x_label":     "Antenna / Device",
    "y_label":     "OWD (ms)",
    "column_labels": {                  # rename CSV column headers for display
        "nitNrf": "nRF52840DK (Saclay)",
        "nitDWM": "DWM3001 (Toulouse)",
        "clockSynNrf": "nRF52840DK (Saclay)",
        "clockSynDWM": "DWM3001 (Toulouse)",
        "nitRUDP": "RUDP",
        "nitTCP": "TCP",
        "n2nrf": "nRF52840DK (Saclay)",
        "n2dwm": "DWM3001 (Toulouse)",
    },

    # --- Layout ---
    "figure_size":   (6, 4),
    "dpi":           150,
    "show_points":   False,             # overlay individual data points (jittered) on boxplot
    "show_means":    False,             # draw a mean marker on boxplot
    "show_grid":     True,
    "show_outlier_fliers": False,       # show outliers as 'x' markers on boxplot
    "y_axis_zero":   True,
    "font_family":   "Times New Roman",
    "y_axis_top_margin": 1.3,           # extra space above max value (None to disable)
    "y_axis_as_percent": False,

    # --- Colours ---
    "color_schemes": [
        {
            "box_color":     "#4C9BE8",
            "median_color":  "#1E5A96",
            "mean_color":    "#2E7DB6",
            "point_color":   "#4C9BE8",
        },
        {
            "box_color":     "#E84C4C",
            "median_color":  "#C41E3A",
            "mean_color":    "#E24C4C",
            "point_color":   "#E84C4C",
        },
        {
            "box_color":     "#2ECC71",
            "median_color":  "#1A7C4F",
            "mean_color":    "#27AE60",
            "point_color":   "#2ECC71",
        },
        {
            "box_color":     "#F39C12",
            "median_color":  "#BA4A00",
            "mean_color":    "#E67E22",
            "point_color":   "#F39C12",
        },
    ],
    # Fallback colours (used when color_schemes runs out of entries)
    "box_color":     "#4C9BE8",
    "median_color":  "#E84C4C",
    "mean_color":    "#2ECC71",
    "point_color":   "#333333",
    "point_alpha":   0.45,

    # --- Statistics annotation ---
    "annotate_n":      False,           # print sample size (n=…) below each box/bar
    "annotate_median": False,           # print median value on boxplot
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


def _get_color(cfg: dict, index: int, key: str) -> str:
    """Return a per-series colour from color_schemes, falling back to cfg[key]."""
    schemes = cfg.get("color_schemes")
    if schemes and index < len(schemes):
        return schemes[index][key]
    return cfg[key]


def _compute_error(s: np.ndarray, method: str) -> float:
    """Compute error bar half-width for array s using the chosen method."""
    if method == "sd":
        return float(np.std(s, ddof=1))
    elif method == "sem":
        return float(np.std(s, ddof=1) / np.sqrt(len(s)))
    elif method == "ci95":
        return float(1.96 * np.std(s, ddof=1) / np.sqrt(len(s)))
    else:
        raise ValueError(f"Unknown barplot_error method: '{method}'. "
                         "Choose 'sd', 'sem', or 'ci95'.")


def _apply_transforms(series: list[np.ndarray], cfg: dict) -> list[np.ndarray]:
    """Apply value_scale and y_axis_as_percent transforms to all series."""
    scale = cfg.get("value_scale", 1)
    if scale != 1:
        series = [s * scale for s in series]
    if cfg.get("y_axis_as_percent", cfg.get("y_axis_as_percentage", False)):
        series = [s * 100 for s in series]
    return series


# ─── BOXPLOT ──────────────────────────────────────────────────────────────────

def make_boxplot(data: dict[str, list[float]], cfg: dict) -> None:
    column_labels = cfg.get("column_labels", {})
    labels  = [column_labels.get(k, k) for k in data]
    series  = _apply_transforms([np.array(v) for v in data.values()], cfg)

    fig, ax = plt.subplots(figsize=cfg["figure_size"], dpi=cfg["dpi"])

    bp = ax.boxplot(
        series,
        labels=labels,
        patch_artist=True,
        medianprops=dict(color=cfg["median_color"], linewidth=2),
        whiskerprops=dict(linewidth=1),
        capprops=dict(linewidth=1),
        flierprops=dict(marker="x", markersize=4, alpha=0.5),
        showfliers=cfg.get("show_outlier_fliers", True),
    )

    for i, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(_get_color(cfg, i, "box_color"))
        bp["medians"][i].set_color(_get_color(cfg, i, "median_color"))
        bp["medians"][i].set_linewidth(1)
        patch.set_alpha(0.7)

    if cfg["show_points"]:
        for i, s in enumerate(series, start=1):
            x_jitter = np.random.uniform(-0.12, 0.12, size=len(s)) + i
            ax.scatter(x_jitter, s,
                       color=_get_color(cfg, i - 1, "point_color"),
                       alpha=cfg["point_alpha"], s=18, zorder=3)

    if cfg["show_means"]:
        for i, s in enumerate(series, start=1):
            ax.scatter(i, np.mean(s), marker="D", s=40,
                       color=_get_color(cfg, i - 1, "mean_color"),
                       zorder=4, label="mean" if i == 1 else "")
        ax.legend(fontsize=9, framealpha=0.6)

    if cfg["annotate_n"]:
        y_bottom = ax.get_ylim()[0]
        for i, s in enumerate(series, start=1):
            ax.text(i, y_bottom, f"n={len(s)}",
                    ha="center", va="bottom", fontsize=8, color="#555555")

    if cfg["annotate_median"]:
        for i, s in enumerate(series, start=1):
            med = np.median(s)
            ax.text(i, med, f" {med:.4g}",
                    va="center", fontsize=8, color=cfg["median_color"])

    _finalise_axes(ax, cfg, series)
    _save_or_show(fig, cfg)


# ─── BARPLOT ──────────────────────────────────────────────────────────────────

def make_barplot(data: dict[str, list[float]], cfg: dict) -> None:
    column_labels = cfg.get("column_labels", {})
    labels  = [column_labels.get(k, k) for k in data]
    series  = _apply_transforms([np.array(v) for v in data.values()], cfg)

    error_method = cfg.get("barplot_error", "sd")
    means  = [float(np.mean(s)) for s in series]
    errors = [_compute_error(s, error_method) for s in series]

    x_pos     = np.arange(len(labels))
    bar_width = cfg.get("barplot_bar_width", 0.5)

    fig, ax = plt.subplots(figsize=cfg["figure_size"], dpi=cfg["dpi"])

    for i, (x, mean, err) in enumerate(zip(x_pos, means, errors)):
        bar_color = _get_color(cfg, i, "box_color")

        ax.bar(
            x, mean,
            width=bar_width,
            color=bar_color,
            alpha=0.75,
            zorder=2,
        )
        ax.errorbar(
            x, mean,
            yerr=err,
            fmt="none",
            color=_get_color(cfg, i, "median_color"),
            capsize=5,
            capthick=1.5,
            elinewidth=1.5,
            zorder=3,
        )

    # Optional jitter overlay
    if cfg.get("barplot_show_points") or cfg.get("show_points"):
        for i, (x, s) in enumerate(zip(x_pos, series)):
            x_jitter = np.random.uniform(-bar_width * 0.3, bar_width * 0.3, size=len(s)) + x
            ax.scatter(x_jitter, s,
                       color=_get_color(cfg, i, "point_color"),
                       alpha=cfg["point_alpha"], s=18, zorder=4)

    if cfg["annotate_n"]:
        y_bottom = ax.get_ylim()[0]
        for x, s in zip(x_pos, series):
            ax.text(x, y_bottom, f"n={len(s)}",
                    ha="center", va="bottom", fontsize=8, color="#555555")

    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)

    # Add error-bar type note to y-label if desired
    error_labels = {"sd": "SD", "sem": "SEM", "ci95": "95 % CI"}
    y_label = cfg["y_label"]
    note = error_labels.get(error_method, error_method)
    ax.set_ylabel(f"{y_label}", fontsize=11)

    _finalise_axes(ax, cfg, series, set_ylabel=False)
    _save_or_show(fig, cfg)


# ─── SHARED HELPERS ───────────────────────────────────────────────────────────

def _finalise_axes(ax, cfg: dict, series: list[np.ndarray], set_ylabel: bool = True) -> None:
    y_axis_as_percent = cfg.get("y_axis_as_percent", cfg.get("y_axis_as_percentage", False))

    ax.set_xlabel(cfg["x_label"], fontsize=11)
    if set_ylabel:
        ax.set_ylabel(cfg["y_label"], fontsize=11)

    if cfg["show_grid"]:
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.5)
        ax.grid(axis="y", which="minor", linestyle=":", alpha=0.3)

    if y_axis_as_percent:
        ax.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:g}%"))

    if cfg["y_axis_zero"]:
        ax.set_ylim(bottom=0)

    if cfg.get("y_axis_top_margin"):
        y_max = max(max(s) for s in series)
        ax.set_ylim(top=y_max * cfg["y_axis_top_margin"])

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def _save_or_show(fig, cfg: dict) -> None:
    fig.tight_layout()
    if cfg["output_file"]:
        fig.savefig(cfg["output_file"])
        print(f"Plot saved to {cfg['output_file']}")
    else:
        plt.show()


# ─── ENTRY POINT ──────────────────────────────────────────────────────────────

def main() -> None:
    cfg = CONFIG.copy()

    if len(sys.argv) > 1:
        cfg["input_file"] = sys.argv[1]
    if len(sys.argv) > 2:
        cfg["output_file"] = sys.argv[2]

    apply_font_settings(cfg)

    data = load_csv(cfg["input_file"])

    print("Loaded columns:")
    for col, vals in data.items():
        print(f"  {col}: {len(vals)} values  "
              f"(min={min(vals):.4g}, max={max(vals):.4g}, mean={np.mean(vals):.4g})")

    plot_type = cfg.get("plot_type", "boxplot").lower()

    if plot_type == "barplot":
        make_barplot(data, cfg)
    elif plot_type == "boxplot":
        make_boxplot(data, cfg)
    else:
        sys.exit(f"Error: unknown plot_type '{plot_type}'. Choose 'boxplot' or 'barplot'.")


if __name__ == "__main__":
    main()