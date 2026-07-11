#!/usr/bin/env python3
"""
n4_clock_sync_channel_plot.py - Compare n=4 DWM3001 delay-chain latency for
different clock synchronization channels.

The n4.csv file contributes:
  - delayNoClockSyn: RUDP data channel, no clock synchronization
  - delayClockSyn:   RUDP data channel, UDP/dedicated clock synchronization

The n4_nodedicated.log file contributes:
  - RUDP data channel, RUDP/reliable clock synchronization

CONFIGURATION
=============
Edit CONFIG below to customize input files, labels, statistics, and the
thesis-friendly figure size. By default the plot focuses on the 0-60 ms range.
"""

from __future__ import annotations

import csv
import os
import pathlib
import re
import sys
import tempfile
from dataclasses import dataclass

os.environ.setdefault(
    "MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "matplotlib")
)

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


# --- CONFIGURATION -----------------------------------------------------------

CONFIG = {
    # --- Input / output ---
    "csv_file": "measurements/dwm3001/n4.csv",
    "log_file": "measurements/dwm3001/n4_nodedicated.log",
    "output_file": "measurements/dwm3001/fig_n4_clock_sync_channel.pdf",
    "summary_csv": "measurements/dwm3001/n4_clock_sync_channel_summary.csv",
    "script_dir": pathlib.Path(__file__).resolve().parent,
    "dpi": 300,
    "bbox_inches": None,

    # --- CSV column names ---
    "col_no_sync": "delayNoClockSyn",
    "col_udp_sync": "delayClockSyn",

    # --- Thesis-friendly sizing ---
    # 426.79135 pt is the same text width used by delay-chain-plot.py.
    "latex_text_width_pt": 426.79135,
    "width_fraction": 0.6,
    "aspect_ratio": 0.83,
    "fallback_width_in": 2.85,
    "figure_size": None,

    # --- Font and rendering ---
    "font_family": "serif",
    "font_serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font_size_pt": 8.5,
    "axis_label_size_pt": 8.5,
    "tick_label_size_pt": 7.5,
    "legend_size_pt": 7.2,
    "annotation_size_pt": 6.8,
    "use_latex_text": False,
    "latex_preamble": r"\usepackage{fontspec}\setmainfont{Times New Roman}",
    "pgf_texsystem": "xelatex",
    "pdf_fonttype": 42,
    "svg_fonttype": "none",

    # --- Labels ---
    "y_label": "Latency (ms)",
    "series": [
        {
            "key": "no_sync",
            "label": "No sync",
            "detail": "RUDP data, no clock sync",
            "color": "#4C9BE8",
        },
        {
            "key": "udp_sync",
            "label": "UDP sync",
            "detail": "RUDP data, UDP clock sync",
            "color": "#E84C4C",
        },
        {
            "key": "rudp_sync",
            "label": "RUDP sync",
            "detail": "RUDP data, RUDP clock sync",
            "color": "#F39C12",
        },
    ],

    # --- Plot detail ---
    "plot_kind": "violin",
    "show_points": False,
    "point_alpha": 0.12,
    "point_size": 9,
    "jitter_width": 0.16,
    "show_mean": False,
    "show_iqr": False,
    "show_median": False,
    "show_p95": False,
    "show_sample_count": False,
    "show_clipped_count": False,
    "clipped_marker_label": "values >45 ms",
    "show_grid": True,
    "show_titles": False,
    "legend_loc": "lower left",
    "show_outlier_fliers": False,
    "violin_width": 0.78,
    "violin_alpha": 0.62,
    "violin_clip_to_focus": True,
    "violin_bw_method": 0.18,
    "box_width": 0.56,
    "box_alpha": 0.62,
    "median_line_width": 1.25,
    "whisker_line_width": 0.9,

    # Focus the main thesis figure on the dense low-latency region. Values
    # above this range are counted and marked near the top of the plot.
    "focus_ylim": (0, 350),

    # Optional broken y-axis mode for inspecting the long tail.
    "use_broken_y_axis": False,
    "lower_ylim": (0, 350),
    "upper_ylim": None,      # None -> derived from all values above lower_ylim[1]
    "upper_axis_height_ratio": 0.42,
    "broken_axis_gap": 0.06,
    "broken_axis_diagonal_size": 0.012,
    "y_axis_top_margin": 1.08,

    # Set to a number to make the jitter deterministic in generated figures.
    "random_seed": 7,
}

# --- END CONFIGURATION -------------------------------------------------------


@dataclass(frozen=True)
class Series:
    key: str
    label: str
    detail: str
    color: str
    values: np.ndarray


def latex_pt_to_in(pt: float) -> float:
    return pt / 72.27


def resolve_path(path: str | pathlib.Path, cfg: dict) -> pathlib.Path:
    p = pathlib.Path(path)
    if p.is_absolute():
        return p
    return pathlib.Path(cfg["script_dir"]) / p


def figure_size(cfg: dict) -> tuple[float, float]:
    if cfg.get("figure_size") is not None:
        return tuple(cfg["figure_size"])
    text_width_pt = cfg.get("latex_text_width_pt")
    if text_width_pt:
        width_in = latex_pt_to_in(float(text_width_pt)) * float(cfg["width_fraction"])
    else:
        width_in = float(cfg["fallback_width_in"])
    return width_in, width_in * float(cfg["aspect_ratio"])


def apply_plot_style(cfg: dict) -> None:
    font_size = cfg.get("font_size_pt", 8.5)
    mpl.rcParams.update(
        {
            "font.family": cfg.get("font_family", "serif"),
            "font.serif": cfg.get("font_serif", ["DejaVu Serif"]),
            "font.size": font_size,
            "axes.labelsize": cfg.get("axis_label_size_pt", font_size),
            "xtick.labelsize": cfg.get("tick_label_size_pt", font_size),
            "ytick.labelsize": cfg.get("tick_label_size_pt", font_size),
            "legend.fontsize": cfg.get("legend_size_pt", max(font_size - 1, 6)),
            "lines.linewidth": 1.0,
            "pdf.fonttype": cfg.get("pdf_fonttype", 42),
            "ps.fonttype": cfg.get("pdf_fonttype", 42),
            "svg.fonttype": cfg.get("svg_fonttype", "none"),
            "text.usetex": bool(cfg.get("use_latex_text", False)),
            "text.latex.preamble": cfg.get("latex_preamble", ""),
            "pgf.texsystem": cfg.get("pgf_texsystem", "pdflatex"),
            "pgf.rcfonts": False,
            "pgf.preamble": cfg.get("latex_preamble", ""),
        }
    )


def _try_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def load_two_column_csv(
    filepath: pathlib.Path,
    col_no_sync: str,
    col_udp_sync: str,
) -> tuple[list[float], list[float]]:
    if not filepath.exists():
        sys.exit(f"Error: file not found: {filepath}")

    no_sync: list[float] = []
    udp_sync: list[float] = []
    idx_no_sync: int | None = None
    idx_udp_sync: int | None = None
    header_seen = False

    with filepath.open(newline="", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            row = next(csv.reader([line]))

            if not header_seen:
                lower = [cell.strip().lower() for cell in row]
                target_no = col_no_sync.lower()
                target_udp = col_udp_sync.lower()
                if target_no in lower or target_udp in lower:
                    idx_no_sync = lower.index(target_no) if target_no in lower else None
                    idx_udp_sync = lower.index(target_udp) if target_udp in lower else None
                    header_seen = True
                    continue

                idx_no_sync, idx_udp_sync = 0, 1
                header_seen = True

            if idx_no_sync is not None and idx_no_sync < len(row):
                value = _try_float(row[idx_no_sync])
                if value is not None:
                    no_sync.append(value)

            if idx_udp_sync is not None and idx_udp_sync < len(row):
                value = _try_float(row[idx_udp_sync])
                if value is not None:
                    udp_sync.append(value)

    return no_sync, udp_sync


def load_latency_log(filepath: pathlib.Path) -> list[float]:
    if not filepath.exists():
        sys.exit(f"Error: file not found: {filepath}")

    pattern = re.compile(r"Latency:\s*([0-9]+(?:\.[0-9]+)?)\s*ms")
    values: list[float] = []
    with filepath.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            match = pattern.search(line)
            if match:
                values.append(float(match.group(1)))
    return values


def collect_data(cfg: dict) -> list[Series]:
    csv_path = resolve_path(cfg["csv_file"], cfg)
    log_path = resolve_path(cfg["log_file"], cfg)
    no_sync, udp_sync = load_two_column_csv(
        csv_path,
        cfg["col_no_sync"],
        cfg["col_udp_sync"],
    )
    rudp_sync = load_latency_log(log_path)

    values_by_key = {
        "no_sync": no_sync,
        "udp_sync": udp_sync,
        "rudp_sync": rudp_sync,
    }

    series = []
    for item in cfg["series"]:
        values = np.asarray(values_by_key[item["key"]], dtype=float)
        if values.size == 0:
            sys.exit(f"Error: no values loaded for series '{item['key']}'.")
        series.append(
            Series(
                key=item["key"],
                label=item["label"],
                detail=item["detail"],
                color=item["color"],
                values=values,
            )
        )
    return series


def describe(values: np.ndarray) -> dict[str, float]:
    return {
        "n": float(values.size),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        "sem": (
            float(np.std(values, ddof=1) / np.sqrt(values.size))
            if values.size > 1
            else 0.0
        ),
        "iqr": float(np.percentile(values, 75) - np.percentile(values, 25)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "over_50_ms": float(np.mean(values > 50.0) * 100.0),
        "over_100_ms": float(np.mean(values > 100.0) * 100.0),
    }


def print_summary(series: list[Series]) -> None:
    print("\nN=4 DWM3001 delay-chain latency")
    print("--------------------------------")
    print(
        "{:<10} {:>5} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}".format(
            "Series", "n", "mean", "median", "IQR", "p95", "p99", ">50ms", ">100ms"
        )
    )
    for s in series:
        d = describe(s.values)
        print(
            "{:<10} {:>5.0f} {:>8.2f} {:>8.2f} {:>8.2f} {:>8.2f} "
            "{:>8.2f} {:>7.1f}% {:>7.1f}%".format(
                s.label,
                d["n"],
                d["mean"],
                d["median"],
                d["iqr"],
                d["p95"],
                d["p99"],
                d["over_50_ms"],
                d["over_100_ms"],
            )
        )
    print()


def export_summary_csv(series: list[Series], cfg: dict) -> None:
    out = cfg.get("summary_csv")
    if not out:
        return

    path = resolve_path(out, cfg)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "key",
        "label",
        "detail",
        "n",
        "mean_ms",
        "median_ms",
        "std_ms",
        "sem_ms",
        "iqr_ms",
        "p90_ms",
        "p95_ms",
        "p99_ms",
        "min_ms",
        "max_ms",
        "over_50_ms_percent",
        "over_100_ms_percent",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for s in series:
            d = describe(s.values)
            writer.writerow(
                {
                    "key": s.key,
                    "label": s.label,
                    "detail": s.detail,
                    "n": int(d["n"]),
                    "mean_ms": f"{d['mean']:.6g}",
                    "median_ms": f"{d['median']:.6g}",
                    "std_ms": f"{d['std']:.6g}",
                    "sem_ms": f"{d['sem']:.6g}",
                    "iqr_ms": f"{d['iqr']:.6g}",
                    "p90_ms": f"{d['p90']:.6g}",
                    "p95_ms": f"{d['p95']:.6g}",
                    "p99_ms": f"{d['p99']:.6g}",
                    "min_ms": f"{d['min']:.6g}",
                    "max_ms": f"{d['max']:.6g}",
                    "over_50_ms_percent": f"{d['over_50_ms']:.6g}",
                    "over_100_ms_percent": f"{d['over_100_ms']:.6g}",
                }
            )
    print(f"Summary CSV -> {path}")


def _darken(hex_color: str, factor: float = 0.62) -> str:
    h = hex_color.lstrip("#")
    r, g, b = [int(h[i:i + 2], 16) for i in (0, 2, 4)]
    return "#{:02x}{:02x}{:02x}".format(
        int(r * factor), int(g * factor), int(b * factor)
    )


def _style_axis(ax: plt.Axes, cfg: dict) -> None:
    ax.spines["right"].set_visible(False)
    if cfg.get("show_grid", True):
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", linewidth=0.45, alpha=0.42)
        ax.grid(axis="y", which="minor", linestyle=":", linewidth=0.35, alpha=0.22)


def _visible_values(values: np.ndarray, cfg: dict) -> np.ndarray:
    if not cfg.get("violin_clip_to_focus", False) or not cfg.get("focus_ylim"):
        return values

    y_min, y_max = cfg["focus_ylim"]
    visible = values[(values >= y_min) & (values <= y_max)]
    return visible if visible.size else values


def _draw_violin_distribution(
    ax: plt.Axes,
    series: list[Series],
    cfg: dict,
    rng: np.random.Generator,
) -> None:
    data = [_visible_values(s.values, cfg) for s in series]
    positions = np.arange(1, len(series) + 1)

    violins = ax.violinplot(
        data,
        positions=positions,
        widths=cfg["violin_width"],
        showmeans=False,
        showmedians=False,
        showextrema=False,
        bw_method=cfg.get("violin_bw_method"),
    )

    for body, s in zip(violins["bodies"], series):
        body.set_facecolor(s.color)
        body.set_edgecolor(_darken(s.color))
        body.set_alpha(cfg["violin_alpha"])
        body.set_linewidth(0.9)

    if cfg.get("show_iqr", True):
        for pos, s, values in zip(positions, series, data):
            q1, median, q3 = np.percentile(values, [25, 50, 75])
            dark = _darken(s.color, 0.5)
            ax.plot(
                [pos, pos],
                [q1, q3],
                color=dark,
                linewidth=3.0,
                solid_capstyle="round",
                zorder=5,
                label="IQR" if pos == 1 else None,
            )
            if cfg.get("show_median", True):
                ax.plot(
                    [pos - 0.16, pos + 0.16],
                    [median, median],
                    color="#222222",
                    linewidth=1.35,
                    solid_capstyle="round",
                    zorder=6,
                    label="median" if pos == 1 else None,
                )

    if cfg.get("show_points", True):
        for pos, s, values in zip(positions, series, data):
            jitter = rng.uniform(
                -cfg["jitter_width"],
                cfg["jitter_width"],
                size=values.size,
            )
            ax.scatter(
                np.full(values.size, pos) + jitter,
                values,
                s=cfg["point_size"],
                color=s.color,
                alpha=cfg["point_alpha"],
                linewidths=0,
                zorder=2,
            )

    if cfg.get("show_mean", True):
        means = [np.mean(s.values) for s in series]
        ax.scatter(
            positions,
            means,
            marker="D",
            s=20,
            facecolor="white",
            edgecolor="#222222",
            linewidth=0.75,
            zorder=7,
            label="mean",
        )

    if cfg.get("show_p95", True):
        p95 = [np.percentile(s.values, 95) for s in series]
        ax.scatter(
            positions,
            p95,
            marker="^",
            s=24,
            facecolor="#222222",
            edgecolor="white",
            linewidth=0.45,
            zorder=8,
            label="p95",
        )

    _style_axis(ax, cfg)


def _draw_boxplot_distribution(
    ax: plt.Axes,
    series: list[Series],
    cfg: dict,
    rng: np.random.Generator,
) -> None:
    data = [s.values for s in series]
    positions = np.arange(1, len(series) + 1)

    bp = ax.boxplot(
        data,
        positions=positions,
        widths=cfg["box_width"],
        patch_artist=True,
        showfliers=cfg.get("show_outlier_fliers", True),
        medianprops={"linewidth": cfg["median_line_width"]},
        whiskerprops={"linewidth": cfg["whisker_line_width"]},
        capprops={"linewidth": cfg["whisker_line_width"]},
        flierprops={
            "marker": "x",
            "markersize": 3.2,
            "markeredgewidth": 0.7,
            "alpha": 0.55,
        },
        manage_ticks=False,
    )

    for i, s in enumerate(series):
        bp["boxes"][i].set_facecolor(s.color)
        bp["boxes"][i].set_alpha(cfg["box_alpha"])
        bp["boxes"][i].set_edgecolor(_darken(s.color))
        bp["medians"][i].set_color(_darken(s.color, 0.45))
        for j in (2 * i, 2 * i + 1):
            bp["whiskers"][j].set_color(_darken(s.color))
            bp["caps"][j].set_color(_darken(s.color))
        if i < len(bp["fliers"]):
            bp["fliers"][i].set_markeredgecolor(_darken(s.color))

    if cfg.get("show_points", True):
        for pos, s in zip(positions, series):
            jitter = rng.uniform(
                -cfg["jitter_width"],
                cfg["jitter_width"],
                size=s.values.size,
            )
            ax.scatter(
                np.full(s.values.size, pos) + jitter,
                s.values,
                s=cfg["point_size"],
                color=s.color,
                alpha=cfg["point_alpha"],
                linewidths=0,
                zorder=2,
            )

    if cfg.get("show_mean", True):
        means = [np.mean(s.values) for s in series]
        ax.scatter(
            positions,
            means,
            marker="D",
            s=20,
            facecolor="white",
            edgecolor="#222222",
            linewidth=0.75,
            zorder=5,
            label="mean",
        )

    if cfg.get("show_p95", True):
        p95 = [np.percentile(s.values, 95) for s in series]
        ax.scatter(
            positions,
            p95,
            marker="^",
            s=24,
            facecolor="#222222",
            edgecolor="white",
            linewidth=0.45,
            zorder=6,
            label="p95",
        )

    _style_axis(ax, cfg)


def _draw_distribution(
    ax: plt.Axes,
    series: list[Series],
    cfg: dict,
    rng: np.random.Generator,
) -> None:
    if cfg.get("plot_kind") == "violin":
        _draw_violin_distribution(ax, series, cfg, rng)
    elif cfg.get("plot_kind") == "boxplot":
        _draw_boxplot_distribution(ax, series, cfg, rng)
    else:
        raise ValueError("Unknown plot_kind. Use 'violin' or 'boxplot'.")


def _add_broken_axis_marks(
    ax_top: plt.Axes,
    ax_bottom: plt.Axes,
    cfg: dict,
) -> None:
    d = cfg["broken_axis_diagonal_size"]
    kwargs = dict(color="#222222", clip_on=False, linewidth=0.7)
    ax_top.plot((-d, +d), (-d, +d), transform=ax_top.transAxes, **kwargs)
    ax_top.plot((1 - d, 1 + d), (-d, +d), transform=ax_top.transAxes, **kwargs)
    ax_bottom.plot((-d, +d), (1 - d, 1 + d), transform=ax_bottom.transAxes, **kwargs)
    ax_bottom.plot((1 - d, 1 + d), (1 - d, 1 + d), transform=ax_bottom.transAxes, **kwargs)


def _upper_ylim(series: list[Series], cfg: dict) -> tuple[float, float] | None:
    configured = cfg.get("upper_ylim")
    if configured is not None:
        return tuple(configured)

    lower_top = cfg["lower_ylim"][1]
    all_values = np.concatenate([s.values for s in series])
    tail_values = all_values[all_values > lower_top]
    if tail_values.size == 0:
        return None

    upper_bottom = max(lower_top + 2.0, float(np.min(tail_values)) - 3.0)
    upper_top = float(np.max(tail_values)) * float(cfg["y_axis_top_margin"])
    return upper_bottom, upper_top


def _add_clipped_markers(
    ax: plt.Axes,
    series: list[Series],
    positions: np.ndarray,
    cfg: dict,
) -> None:
    if not cfg.get("show_clipped_count", False) or not cfg.get("focus_ylim"):
        return

    _, y_top = cfg["focus_ylim"]
    y_bottom, _ = ax.get_ylim()
    y_span = y_top - y_bottom
    marker_y = y_top - 0.045 * y_span
    label_y = y_top - 0.095 * y_span
    label_used = False

    for pos, s in zip(positions, series):
        clipped_count = int(np.sum(s.values > y_top))
        if clipped_count == 0:
            continue

        ax.scatter(
            pos,
            marker_y,
            marker="^",
            s=18,
            facecolor="white",
            edgecolor=_darken(s.color),
            linewidth=0.8,
            zorder=7,
            label=cfg["clipped_marker_label"] if not label_used else None,
        )
        ax.text(
            pos,
            label_y,
            str(clipped_count),
            ha="center",
            va="top",
            fontsize=cfg["annotation_size_pt"],
            color=_darken(s.color),
        )
        label_used = True


def make_plot(series: list[Series], cfg: dict) -> plt.Figure:
    rng = np.random.default_rng(cfg.get("random_seed"))
    fig_size = figure_size(cfg)
    if cfg.get("show_sample_count", True):
        labels = [f"{s.label}\n(n={s.values.size})" for s in series]
    else:
        labels = [s.label for s in series]
    positions = np.arange(1, len(series) + 1)

    if cfg.get("use_broken_y_axis", True):
        upper_ylim = _upper_ylim(series, cfg)
    else:
        upper_ylim = None

    if upper_ylim:
        fig, (ax_top, ax_bottom) = plt.subplots(
            2,
            1,
            sharex=True,
            figsize=fig_size,
            dpi=cfg["dpi"],
            gridspec_kw={
                "height_ratios": [cfg["upper_axis_height_ratio"], 1.0],
                "hspace": cfg["broken_axis_gap"],
            },
        )
        _draw_distribution(ax_top, series, cfg, rng)
        _draw_distribution(ax_bottom, series, cfg, rng)

        ax_bottom.set_ylim(*cfg["lower_ylim"])
        ax_top.set_ylim(*upper_ylim)
        ax_top.spines["bottom"].set_visible(False)
        ax_bottom.spines["top"].set_visible(False)
        ax_top.tick_params(labelbottom=False, bottom=False)
        ax_bottom.set_xticks(positions)
        ax_bottom.set_xticklabels(labels)
        ax_bottom.set_ylabel(cfg["y_label"])
        ax_bottom.yaxis.set_label_coords(-0.12, 0.78)
        _add_broken_axis_marks(ax_top, ax_bottom, cfg)

        legend_handles, legend_labels = ax_bottom.get_legend_handles_labels()
        if legend_handles:
            ax_top.legend(
                legend_handles,
                legend_labels,
                loc=cfg["legend_loc"],
                framealpha=0.72,
                borderpad=0.25,
                handletextpad=0.4,
            )
        axes = [ax_top, ax_bottom]
    else:
        fig, ax = plt.subplots(figsize=fig_size, dpi=cfg["dpi"])
        _draw_distribution(ax, series, cfg, rng)
        ax.set_xticks(positions)
        ax.set_xticklabels(labels)
        ax.set_ylabel(cfg["y_label"])
        if cfg.get("focus_ylim"):
            ax.set_ylim(*cfg["focus_ylim"])
            _add_clipped_markers(ax, series, positions, cfg)
        else:
            ax.set_ylim(bottom=0)
            all_values = np.concatenate([s.values for s in series])
            ax.set_ylim(top=float(np.max(all_values)) * float(cfg["y_axis_top_margin"]))
        legend_handles, legend_labels = ax.get_legend_handles_labels()
        if legend_handles:
            ax.legend(
                legend_handles,
                legend_labels,
                loc=cfg["legend_loc"],
                framealpha=0.72,
            )
        axes = [ax]

    if cfg.get("show_titles", False):
        fig.suptitle("N=4 delay-chain latency by clock-sync channel", y=0.995)

    for ax in axes:
        ax.set_xlim(0.45, len(series) + 0.55)

    fig.subplots_adjust(left=0.18, right=0.99, bottom=0.17, top=0.98)
    return fig


def save_figure(fig: plt.Figure, cfg: dict) -> None:
    output_path = resolve_path(cfg["output_file"], cfg)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=cfg["dpi"], bbox_inches=cfg.get("bbox_inches"))
    width_in, height_in = figure_size(cfg)
    print(
        f"Plot -> {output_path} "
        f"({cfg['width_fraction']:.2f}\\linewidth, {width_in:.2f} x {height_in:.2f} in)"
    )


def main() -> None:
    cfg = CONFIG.copy()
    if len(sys.argv) > 1:
        cfg["output_file"] = sys.argv[1]

    apply_plot_style(cfg)
    series = collect_data(cfg)
    print_summary(series)
    export_summary_csv(series, cfg)
    fig = make_plot(series, cfg)
    save_figure(fig, cfg)
    plt.close(fig)


if __name__ == "__main__":
    main()
