#!/usr/bin/env python3
"""
delay_chain_analysis.py – Analyse wireless-transmission latency for federated
Lingua Franca delay chains across different federate counts.

Each CSV file corresponds to one chain length (n=2, 4, 6, 8 …) and has the
format:

    delayNoClockSyn,delayClockSyn
    ,62.00          ← only clock-sync column has a value
    66.00,          ← only no-clock-sync column has a value
    # comment lines and blank lines are ignored

CONFIGURATION
=============
Edit the CONFIG dict below to customise the plot to your needs.
"""

import csv
import sys
import pathlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from itertools import zip_longest

# ─── CONFIGURATION ────────────────────────────────────────────────────────────

CONFIG = {
    # --- Input files ---
    # Each entry: (path, federate_count_label)
    # The script infers N from the filename (e.g. "n4.csv" → N=4) when
    # auto_discover is True; otherwise the list below is used as-is.
    "input_files": [
        ("measurements/dwm3001/n2.csv", "N=2"),
        ("measurements/dwm3001/n4.csv", "N=4"),
        ("measurements/dwm3001/n6.csv", "N=6"),
        ("measurements/dwm3001/n8.csv", "N=8"),
    ],

    # If True, look for files matching the pattern below in the current dir
    # and ignore input_files above.
    "auto_discover": False,
    "auto_discover_glob": "n*.csv",

    # --- Output ---
    "output_file": None,          # None → interactive; "plot.png" → save
    "dpi":         150,
    "font_family": "Times New Roman",

    # --- Column names in the CSV (case-insensitive match) ---
    "col_no_sync": "delayNoClockSyn",
    "col_sync":    "delayClockSyn",

    # --- Plot selection ---
    # Set any to False to skip that subplot
    "show_boxplot":    True,   # grouped box plot per chain length
    "show_mean_line":  True,   # line chart of mean latency vs. N
    "show_iqr_line":   True,   # line chart of IQR (spread) vs. N

    # --- Titles ---
    # Set to False to suppress all subplot titles and the figure suptitle
    # (recommended when embedding in LaTeX – use \caption{} instead)
    "show_titles":     False,

    # --- Per-plot export (independent of the combined output_file) ---
    # Each can be None (skip), or a file path string.
    # Supported extensions: .png .pdf .svg .eps (anything matplotlib accepts).
    # .pdf and .svg are recommended for LaTeX (\includegraphics{}).
    # When set, that single panel is saved as its own figure at the same dpi.
    "output_boxplot":   None,   # e.g. "fig_boxplot.pdf"
    "output_mean_line": None,   # e.g. "fig_mean.pdf"
    "output_iqr_line":  None,   # e.g. "fig_iqr.pdf"

    # --- SVG export of each individual panel ---
    # Each can be None (skip) or a file path string.
    # Useful for importing into LaTeX or Inkscape.
    # The panel is rendered as a standalone figure at the same dpi.
    "svg_boxplot":   "measurements/dwm3001/fig_boxplot.svg",   # e.g. "fig_boxplot.svg"
    "svg_mean_line": "measurements/dwm3001/fig_mean.svg",   # e.g. "fig_mean.svg"
    "svg_iqr_line":  "measurements/dwm3001/fig_iqr.svg",   # e.g. "fig_iqr.svg"

    # --- Labels ---
    "y_label":         "Latency (ms)",
    "x_label":         "Federate count",
    "label_no_sync":   "Clock sync off",
    "label_sync":      "Clock sync on",

    # --- Colours ---
    "color_no_sync":   "#4C9BE8",   # blue family
    "color_sync":      "#E84C4C",   # red family
    "median_alpha":    0.85,
    "box_alpha":       0.65,
    "line_alpha":      0.90,

    # --- Misc ---
    "show_points":     True,    # overlay jittered points on box plot
    "point_alpha":     0.15,
    "show_means":      False,    # diamond mean marker on box plot
    "annotate_n":      False,    # print sample count below each group
    "show_grid":       True,
    "y_axis_zero":     True,
    "y_axis_top_margin": 1.15,  # multiply max value for top headroom

    # Figure size is computed automatically from the number of subplots;
    # override here if you want a fixed size (set to None for auto).
    "figure_size":     None,
}

# ─── END OF CONFIGURATION ─────────────────────────────────────────────────────


# ── helpers ───────────────────────────────────────────────────────────────────

def apply_font(cfg: dict) -> None:
    fam = cfg.get("font_family", "sans-serif")
    if fam == "Times New Roman":
        plt.rcParams["font.family"] = "serif"
        plt.rcParams["font.serif"]  = ["Times New Roman", "Times", "DejaVu Serif"]
    else:
        plt.rcParams["font.family"] = fam


def load_two_column_csv(filepath: str,
                        col_no_sync: str,
                        col_sync: str) -> tuple[list[float], list[float]]:
    """
    Parse the 'abused' CSV format:
      - Lines starting with '#' or blank lines are skipped.
      - Header row is detected by the presence of the expected column names.
      - Each data row contributes to whichever column is non-empty.

    Returns (no_sync_values, sync_values).
    """
    path = pathlib.Path(filepath)
    if not path.exists():
        sys.exit(f"Error: file not found – {filepath}")

    no_sync: list[float] = []
    sync:    list[float] = []

    idx_no_sync: int | None = None
    idx_sync:    int | None = None
    header_seen = False

    with path.open(newline="", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            reader = csv.reader([line])
            row = next(reader)

            if not header_seen:
                # Try to detect header
                low = [c.strip().lower() for c in row]
                target_no  = col_no_sync.lower()
                target_syn = col_sync.lower()
                if target_no in low or target_syn in low:
                    idx_no_sync = low.index(target_no) if target_no in low else None
                    idx_sync    = low.index(target_syn) if target_syn in low else None
                    header_seen = True
                    continue
                else:
                    # No recognisable header → assume col 0 = no_sync, col 1 = sync
                    idx_no_sync, idx_sync = 0, 1
                    header_seen = True
                    # Fall through to parse this row as data

            def try_float(s: str) -> float | None:
                s = s.strip()
                if not s:
                    return None
                try:
                    return float(s)
                except ValueError:
                    print(f"  Warning: cannot parse '{s}' as float – skipped.")
                    return None

            if idx_no_sync is not None and idx_no_sync < len(row):
                v = try_float(row[idx_no_sync])
                if v is not None:
                    no_sync.append(v)

            if idx_sync is not None and idx_sync < len(row):
                v = try_float(row[idx_sync])
                if v is not None:
                    sync.append(v)

    return no_sync, sync


def collect_data(cfg: dict) -> list[tuple[str, list[float], list[float]]]:
    """
    Returns a list of (label, no_sync_values, sync_values) sorted by label.
    """
    if cfg.get("auto_discover"):
        files = sorted(pathlib.Path(".").glob(cfg["auto_discover_glob"]))
        sources = []
        for f in files:
            # Try to extract a number from the filename for the label
            stem = f.stem  # e.g. "n4"
            num  = "".join(c for c in stem if c.isdigit())
            label = f"N={num}" if num else stem
            sources.append((str(f), label))
    else:
        sources = cfg["input_files"]

    results = []
    for fpath, label in sources:
        print(f"Loading {fpath}  ({label}) …")
        no_sync, sync = load_two_column_csv(
            fpath,
            cfg["col_no_sync"],
            cfg["col_sync"],
        )
        print(f"  No-sync: {len(no_sync)} values   Sync: {len(sync)} values")
        results.append((label, no_sync, sync))

    if not results:
        sys.exit("Error: no data loaded. Check input_files / auto_discover settings.")

    return results


# ── plot functions ─────────────────────────────────────────────────────────────

def _style_ax(ax: plt.Axes, cfg: dict, title: str = "") -> None:
    if title and cfg.get("show_titles", True):
        ax.set_title(title, fontsize=11, pad=6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if cfg["show_grid"]:
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.45)
        ax.grid(axis="y", which="minor", linestyle=":", alpha=0.25)


def plot_grouped_boxplot(ax: plt.Axes,
                         data: list[tuple[str, list[float], list[float]]],
                         cfg: dict) -> None:
    """
    Side-by-side box plots for each chain length: no-sync (blue) vs sync (red).
    """
    n_groups = len(data)
    positions_no  = []
    positions_syn = []
    series_no     = []
    series_syn    = []

    group_gap    = 3.0   # gap between chain-length groups
    within_offset = 0.7  # half-gap between the two boxes in a group

    centres = [i * group_gap for i in range(n_groups)]
    xtick_pos   = centres
    xtick_labels = [d[0] for d in data]

    for centre, (_, ns, sy) in zip(centres, data):
        positions_no.append(centre - within_offset)
        positions_syn.append(centre + within_offset)
        series_no.append(np.array(ns))
        series_syn.append(np.array(sy))

    def draw_boxes(ax, series, positions, color, median_color, label):
        if not any(len(s) > 0 for s in series):
            return
        bp = ax.boxplot(
            [s for s in series],
            positions=positions,
            widths=0.9,
            patch_artist=True,
            medianprops=dict(color=median_color, linewidth=2),
            whiskerprops=dict(linewidth=1.2),
            capprops=dict(linewidth=1.2),
            flierprops=dict(marker="x", markersize=4, alpha=0.5, markeredgecolor=color),
            manage_ticks=False,
        )
        for patch in bp["boxes"]:
            patch.set_facecolor(color)
            patch.set_alpha(cfg["box_alpha"])
        # Invisible proxy for legend
        ax.plot([], [], color=color, linewidth=6, alpha=cfg["box_alpha"], label=label)

        if cfg.get("show_points"):
            for pos, s in zip(positions, series):
                if len(s) == 0:
                    continue
                jit = np.random.uniform(-0.25, 0.25, size=len(s)) + pos
                ax.scatter(jit, s, color=color, alpha=cfg["point_alpha"],
                           s=14, zorder=3)

        if cfg.get("show_means"):
            mean_color = color  # slightly lighter would be nice but keep it simple
            for pos, s in zip(positions, series):
                if len(s) == 0:
                    continue
                ax.scatter(pos, np.mean(s),
                           marker="D", s=45, color=mean_color,
                           zorder=5, edgecolors="white", linewidths=0.8)

        return bp

    draw_boxes(ax, series_no,  positions_no,  cfg["color_no_sync"],
               _darken(cfg["color_no_sync"]),  cfg["label_no_sync"])
    draw_boxes(ax, series_syn, positions_syn, cfg["color_sync"],
               _darken(cfg["color_sync"]),     cfg["label_sync"])

    ax.set_xticks(xtick_pos)
    ax.set_xticklabels(xtick_labels, fontsize=10)
    ax.set_xlabel(cfg["x_label"], fontsize=11)
    ax.set_ylabel(cfg["y_label"], fontsize=11)

    if cfg["y_axis_zero"]:
        ax.set_ylim(bottom=0)
    if cfg["y_axis_top_margin"]:
        all_vals = [v for _, ns, sy in data for v in list(ns) + list(sy) if v]
        if all_vals:
            ax.set_ylim(top=max(all_vals) * cfg["y_axis_top_margin"])

    if cfg["annotate_n"]:
        y_bot = ax.get_ylim()[0]
        for (_, ns, sy), c_ns, c_sy in zip(data, positions_no, positions_syn):
            if ns:
                ax.text(c_ns, y_bot, f"n={len(ns)}", ha="center", va="bottom",
                        fontsize=7, color="#666666")
            if sy:
                ax.text(c_sy, y_bot, f"n={len(sy)}", ha="center", va="bottom",
                        fontsize=7, color="#666666")

    ax.legend(fontsize=9, framealpha=0.7, loc="upper left")
    _style_ax(ax, cfg, "Latency distribution per chain length")


def plot_mean_line(ax: plt.Axes,
                   data: list[tuple[str, list[float], list[float]]],
                   cfg: dict) -> None:
    """
    Line chart: mean latency vs. chain length, with ±1 std error bars.
    """
    labels   = [d[0] for d in data]
    x        = np.arange(len(labels))

    def stats(vals):
        if not vals:
            return np.nan, np.nan
        a = np.array(vals)
        return np.mean(a), np.std(a) / np.sqrt(len(a))

    means_ns, errs_ns = zip(*[stats(ns) for _, ns, _ in data])
    means_sy, errs_sy = zip(*[stats(sy) for _, _, sy in data])

    ax.errorbar(x, means_ns, yerr=errs_ns,
                marker="o", markersize=7, linewidth=2,
                color=cfg["color_no_sync"], alpha=cfg["line_alpha"],
                capsize=4, label=cfg["label_no_sync"])
    ax.errorbar(x, means_sy, yerr=errs_sy,
                marker="s", markersize=7, linewidth=2,
                color=cfg["color_sync"], alpha=cfg["line_alpha"],
                capsize=4, label=cfg["label_sync"])

    # Shade the area between them
    ax.fill_between(x,
                    [m - e for m, e in zip(means_ns, errs_ns)],
                    [m + e for m, e in zip(means_sy, errs_sy)],
                    alpha=0.10, color="#888888")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_xlabel(cfg["x_label"], fontsize=11)
    ax.set_ylabel(cfg["y_label"], fontsize=11)

    if cfg["y_axis_zero"]:
        ax.set_ylim(bottom=0)
    if cfg["y_axis_top_margin"]:
        top = max(
            [m + e for m, e in zip(means_ns, errs_ns) if not np.isnan(m)] +
            [m + e for m, e in zip(means_sy, errs_sy) if not np.isnan(m)],
            default=1,
        )
        ax.set_ylim(top=top * cfg["y_axis_top_margin"])

    ax.legend(fontsize=9, framealpha=0.7)
    _style_ax(ax, cfg, "Mean latency ± SEM vs. chain length")


def plot_iqr_line(ax: plt.Axes,
                  data: list[tuple[str, list[float], list[float]]],
                  cfg: dict) -> None:
    """
    Line chart: IQR (p75−p25) as a measure of spread / jitter vs. chain length.
    Also plots p95 as a dotted line to show worst-case tail behaviour.
    """
    labels = [d[0] for d in data]
    x      = np.arange(len(labels))

    def iqr(vals):
        if not vals:
            return np.nan
        a = np.array(vals)
        return np.percentile(a, 75) - np.percentile(a, 25)

    def p95(vals):
        if not vals:
            return np.nan
        return np.percentile(np.array(vals), 95)

    iqr_ns = [iqr(ns) for _, ns, _ in data]
    iqr_sy = [iqr(sy) for _, _, sy in data]
    p95_ns = [p95(ns) for _, ns, _ in data]
    p95_sy = [p95(sy) for _, _, sy in data]

    ax.plot(x, iqr_ns, marker="o", linewidth=2, markersize=7,
            color=cfg["color_no_sync"], alpha=cfg["line_alpha"],
            label=f"{cfg['label_no_sync']} – IQR")
    ax.plot(x, iqr_sy, marker="s", linewidth=2, markersize=7,
            color=cfg["color_sync"], alpha=cfg["line_alpha"],
            label=f"{cfg['label_sync']} – IQR")
    ax.plot(x, p95_ns, marker="^", linewidth=1.4, markersize=6,
            linestyle="--", color=cfg["color_no_sync"], alpha=0.55,
            label=f"{cfg['label_no_sync']} – p95")
    ax.plot(x, p95_sy, marker="v", linewidth=1.4, markersize=6,
            linestyle="--", color=cfg["color_sync"], alpha=0.55,
            label=f"{cfg['label_sync']} – p95")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_xlabel(cfg["x_label"], fontsize=11)
    ax.set_ylabel("Latency (ms)", fontsize=11)

    if cfg["y_axis_zero"]:
        ax.set_ylim(bottom=0)
    if cfg["y_axis_top_margin"]:
        finite = [v for v in iqr_ns + iqr_sy + p95_ns + p95_sy if not np.isnan(v)]
        if finite:
            ax.set_ylim(top=max(finite) * cfg["y_axis_top_margin"])

    ax.legend(fontsize=8, framealpha=0.7, ncol=2)
    _style_ax(ax, cfg, "Latency spread (IQR) and 95th percentile vs. chain length")


# ── CSV export ────────────────────────────────────────────────────────────────
def export_mean_csv(path: str,
                    data: list[tuple[str, list[float], list[float]]],
                    cfg: dict) -> None:
    """
    Mean ± SEM per chain length.
    Columns: chain_label, n_no_sync, mean_no_sync_ms, sem_no_sync_ms,
             n_sync, mean_sync_ms, sem_sync_ms
    """
    def stats(vals):
        if not vals:
            return 0, float("nan"), float("nan")
        a = np.array(vals)
        return len(a), float(np.mean(a)), float(np.std(a) / np.sqrt(len(a)))

    p = pathlib.Path(path)
    with p.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["chain_label",
                    "n_no_sync", "mean_no_sync_ms", "sem_no_sync_ms",
                    "n_sync",    "mean_sync_ms",    "sem_sync_ms"])
        for label, ns, sy in data:
            n_ns, m_ns, e_ns = stats(ns)
            n_sy, m_sy, e_sy = stats(sy)
            w.writerow([label,
                        n_ns, f"{m_ns:.6g}", f"{e_ns:.6g}",
                        n_sy, f"{m_sy:.6g}", f"{e_sy:.6g}"])
    print(f"  CSV (mean line data)   → {path}")

def _save_single_panel(plot_fn, data, cfg, outpath: str) -> None:
    """Render one panel into its own figure and save it."""
    if cfg["figure_size"] is None:
        fig_w = max(5, 2.2 * len(data))
        size = (fig_w, 4.5)
    else:
        # Use half the combined width for a single panel
        w, h = cfg["figure_size"]
        size = (w / max(1, cfg.get("_n_panels", 1)), h)

    fig, ax = plt.subplots(figsize=size, dpi=cfg["dpi"])
    plot_fn(ax, data, cfg)
    fig.tight_layout()
    fig.savefig(outpath, bbox_inches="tight")
    plt.close(fig)
    print(f"  Plot (single panel)    → {outpath}")


def _darken(hex_color: str, factor: float = 0.65) -> str:
    """Return a darker version of a hex colour for median lines."""
    hex_color = hex_color.lstrip("#")
    r, g, b = [int(hex_color[i:i+2], 16) for i in (0, 2, 4)]
    return "#{:02x}{:02x}{:02x}".format(
        int(r * factor), int(g * factor), int(b * factor))


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    cfg = CONFIG.copy()

    if len(sys.argv) > 1:
        cfg["output_file"] = sys.argv[1]

    apply_font(cfg)

    data = collect_data(cfg)

    # Print a quick summary table
    print("\n{:<8}  {:>12}  {:>12}  {:>12}  {:>12}  {:>12}  {:>12}".format(
        "Label", "n(NoSync)", "mean(NS)", "iqr(NS)", "n(Sync)", "mean(S)", "iqr(S)"))
    print("-" * 80)
    for label, ns, sy in data:
        def fmt(v): return f"{v:.2f}" if not np.isnan(v) else "—"
        ns_a = np.array(ns) if ns else np.array([np.nan])
        sy_a = np.array(sy) if sy else np.array([np.nan])
        iqr_ns = fmt(np.percentile(ns_a, 75) - np.percentile(ns_a, 25)) if ns else "—"
        iqr_sy = fmt(np.percentile(sy_a, 75) - np.percentile(sy_a, 25)) if sy else "—"
        print("{:<8}  {:>12}  {:>12}  {:>12}  {:>12}  {:>12}  {:>12}".format(
            label,
            len(ns), fmt(np.mean(ns_a)), iqr_ns,
            len(sy), fmt(np.mean(sy_a)), iqr_sy,
        ))
    print()

    # Build the figure
    panels = [p for p in ["show_boxplot", "show_mean_line", "show_iqr_line"]
              if cfg.get(p, True)]
    n_panels = len(panels)
    if n_panels == 0:
        sys.exit("All panels disabled – nothing to plot.")

    if cfg["figure_size"] is None:
        fig_w = max(7, 2.5 * len(data))
        cfg["figure_size"] = (fig_w * n_panels * 0.6, 4.5)

    fig, axes = plt.subplots(1, n_panels,
                             figsize=cfg["figure_size"],
                             dpi=cfg["dpi"])
    if n_panels == 1:
        axes = [axes]

    panel_idx = 0
    if cfg.get("show_boxplot"):
        plot_grouped_boxplot(axes[panel_idx], data, cfg)
        panel_idx += 1
    if cfg.get("show_mean_line"):
        plot_mean_line(axes[panel_idx], data, cfg)
        panel_idx += 1
    if cfg.get("show_iqr_line"):
        plot_iqr_line(axes[panel_idx], data, cfg)
        panel_idx += 1

    if cfg.get("show_titles", True):
        fig.suptitle(
            "LF Delay-Chain Transmission Latency: Clock Sync Impact",
            fontsize=13, fontweight="bold", y=1.01,
        )
    fig.tight_layout()

    if cfg["output_file"]:
        fig.savefig(cfg["output_file"], bbox_inches="tight")
        print(f"  Plot (combined)        -> {cfg['output_file']}")
    else:
        plt.show()

    # --- Per-panel exports ---
    cfg["_n_panels"] = n_panels   # sizing hint for _save_single_panel
    panel_map = [
        ("show_boxplot",   "svg_boxplot",   plot_grouped_boxplot),
        ("show_mean_line", "svg_mean_line",  plot_mean_line),
        ("show_iqr_line",  "svg_iqr_line",   plot_iqr_line),
    ]
    for show_key, svg_key, fn in panel_map:
        outpath = cfg.get(svg_key)
        if outpath and cfg.get(show_key, True):
            _save_single_panel(fn, data, cfg, outpath)


if __name__ == "__main__":
    main()
