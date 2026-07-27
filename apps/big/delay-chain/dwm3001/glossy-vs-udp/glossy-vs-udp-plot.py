#!/usr/bin/env python3
"""
glossy-vs-udp-plot.py - Extract and plot delay-chain latency and RUDP
retransmission rates from paired Glossy and UDP experiment logs.

The script expects log lines like:

    dwm3001-1 > Latency: 137.00 ms
    dwm3001-2 > Forwarding message (offset 5.00 ms)
    dwm3001-7 > (...) [INFO] [NET] RUdpIpChannel: [stat] 5001 packets sent, 701 retransmissions

CONFIGURATION
=============
Edit the CONFIG dict below to customise inputs and output.
"""

import pathlib
import re
import sys
import os
import tempfile
from dataclasses import dataclass

os.environ.setdefault(
    "MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "matplotlib")
)

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


# CONFIGURATION ---------------------------------------------------------------

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent

CONFIG = {
    # --- Input files ---
    # Each entry: (path, plot_label)
    "input_files": [
        (SCRIPT_DIR / "glossyrun.txt", "Glossy"),
        (SCRIPT_DIR / "udprun.txt", "LF (UDP)"),
    ],

    # --- Output ---
    # None -> interactive; otherwise save combined figure to this path.
    # PDF is the most convenient vector format for \includegraphics in LaTeX.
    "output_file": SCRIPT_DIR / "glossy-vs-udp.pdf",
    "dpi": 300,
    # Keep the declared figure size in the PDF. Tight-cropped bounding boxes can
    # make LaTeX rescale text unexpectedly when using \includegraphics[width=...].
    "bbox_inches": None,

    # LaTeX/thesis-friendly sizing.
    # If you know your thesis \textwidth, set "latex_text_width_pt" to the
    # value printed by \the\textwidth and keep the fractions below. Otherwise
    # the explicit *_width_in values are used.
    "latex_text_width_pt": 426.79135,
    # Width fractions are the intended LaTeX include widths for each PDF.
    # Include a generated PDF at the same width fraction to preserve font sizes.
    "combined_width_fraction": 1.0,
    "latency_width_fraction": 0.6,
    "retransmission_width_fraction": 0.6,
    "hop_offset_width_fraction": 1.0,
    "hop_offset_linear_trend_width_fraction": 1.0,

    # Aspect ratios are height / width. The panel defaults mirror the original
    # plot proportions, just scaled to the intended LaTeX width.
    "combined_aspect_ratio": 0.46,
    "latency_aspect_ratio": 1,
    "retransmission_aspect_ratio": 1,
    "hop_offset_aspect_ratio": 3.2 / 5.2,
    "hop_offset_linear_trend_aspect_ratio": 3.2 / 5.2,

    # Fallback absolute widths, used only when latex_text_width_pt is None.
    "combined_width_in": 6.6,
    "latency_width_in": 3.2,
    "retransmission_width_in": 3.2,
    "hop_offset_width_in": 6.6,
    "hop_offset_linear_trend_width_in": 6.6,

    # Matplotlib PDF output embeds these fonts directly. Keep figures at their
    # intended LaTeX size; scaling them later also scales these font sizes.
    "font_family": "serif",
    "font_size_pt": 10,
    "axis_label_size_pt": 10,
    "title_size_pt": 10,
    "tick_label_size_pt": 8.5,
    "legend_size_pt": 8.5,
    "annotation_size_pt": 8,
    "font_serif": ["Times New Roman", "Times", "DejaVu Serif"],

    # Optional PGF export for \input{...}. This matches a XeLaTeX/LuaLaTeX
    # thesis that selects Times New Roman via fontspec.
    "use_latex_text": False,
    "pgf_texsystem": "xelatex",
    "latex_preamble": r"\usepackage{fontspec}\setmainfont{Times New Roman}",
    "output_pgf": False,

    # Per-plot export. Each can be None or a path string/pathlib.Path.
    "output_latency_boxplot": SCRIPT_DIR / "latency-boxplot.pdf",
    "output_retransmission_boxplot": SCRIPT_DIR / "retransmission-boxplot.pdf",
    "output_hop_offset_plot": SCRIPT_DIR / "hop-offsets.pdf",
    "output_hop_offset_linear_trend_plot": SCRIPT_DIR / "hop-offsets-linear-trend.pdf",

    # --- Plot selection ---
    "show_latency_boxplot": True,
    "show_retransmission_boxplot": True,
    "show_hop_offset_plot": True,
    "show_titles": False,

    # --- Labels ---
    "clock_sync_x_label": "clock sync",
    "latency_y_label": "Latency (ms)",
    "retransmission_y_label": "Retransmissions (%)",
    "hop_offset_y_label": "Forwarding offset (ms)",
    "label_glossy": "Glossy",
    "label_udp": "UDP",

    # --- Colours ---
    "color_glossy": "#E84C4C",
    "color_udp": "#4C9BE8",
    "median_alpha": 0.85,
    "box_alpha": 0.65,
    "point_alpha": 0.18,
    "line_alpha": 0.90,

    # --- Misc ---
    "show_points": True,
    "show_means": True,
    "annotate_n": False,
    "show_grid": True,
    "y_axis_zero": True,
    "y_axis_top_margin": 1.12,

    # --- Forwarding offset plot ---
    # Use symlog for signed values with occasional large excursions.
    # Valid values: "linear", "symlog", "log".
    "hop_offset_y_scale": "symlog",
    "hop_offset_symlog_linthresh": 10,
    "hop_offset_nodes": list(range(1, 9)),
    "hop_offset_include_origin_zero": True,
    "hop_offset_show_iqr": True,
    "hop_offset_show_points": False,
    "hop_offset_point_alpha": 0.08,
    "hop_offset_trend_stat": "median",  # "median" or "mean"

    # Figure size is computed automatically unless this is set.
    "figure_size": None,
}

# END OF CONFIGURATION --------------------------------------------------------


LATENCY_RE = re.compile(r"Latency:\s*([-+]?\d+(?:\.\d+)?)\s*ms")
FORWARD_RE = re.compile(
    r"^dwm3001-(\d+)\s*>.*?Forwarding message\s*\(offset\s+([^\s]+)\s*ms\)"
)
STAT_RE = re.compile(
    r"\[stat\].*?(\d+)\s+packets\s+sent,\s*(\d+)\s+retransmissions"
)
NODE_RE = re.compile(r"^(dwm3001-\d+)\s*>")


@dataclass
class RetransmissionSample:
    node: str | None
    packets_sent: int
    retransmissions: int

    @property
    def percent(self) -> float:
        if self.packets_sent <= 0:
            return 0.0
        return (self.retransmissions / self.packets_sent) * 100.0


@dataclass
class RunData:
    label: str
    latencies_ms: list[float]
    retransmissions: list[RetransmissionSample]
    forwarding_offsets_ms_by_node: dict[int, list[float]]

    @property
    def retransmission_percentages(self) -> list[float]:
        return [sample.percent for sample in self.retransmissions]


def latex_pt_to_in(pt: float) -> float:
    return pt / 72.27


def thesis_width_in(cfg: dict, fraction_key: str, fallback_key: str) -> float:
    text_width_pt = cfg.get("latex_text_width_pt")
    if text_width_pt:
        return latex_pt_to_in(float(text_width_pt)) * float(cfg[fraction_key])
    return float(cfg[fallback_key])


def figure_size_from_width_fraction(
    cfg: dict, fraction_key: str, fallback_width_key: str, aspect_ratio_key: str
) -> tuple[float, float]:
    width_in = thesis_width_in(cfg, fraction_key, fallback_width_key)
    return (width_in, width_in * float(cfg[aspect_ratio_key]))


def apply_plot_style(cfg: dict) -> None:
    font_size = cfg.get("font_size_pt", 10)
    mpl.rcParams.update(
        {
            "font.family": cfg.get("font_family", "serif"),
            "font.serif": cfg.get("font_serif", ["DejaVu Serif"]),
            "font.size": font_size,
            "axes.labelsize": cfg.get("axis_label_size_pt", font_size),
            "axes.titlesize": cfg.get("title_size_pt", font_size),
            "legend.fontsize": cfg.get("legend_size_pt", max(font_size - 1, 6)),
            "xtick.labelsize": cfg.get("tick_label_size_pt", max(font_size - 1, 6)),
            "ytick.labelsize": cfg.get("tick_label_size_pt", max(font_size - 1, 6)),
            "lines.linewidth": 1.2,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "text.usetex": bool(cfg.get("use_latex_text", False)),
            "text.latex.preamble": cfg.get("latex_preamble", ""),
            "pgf.texsystem": cfg.get("pgf_texsystem", "pdflatex"),
            "pgf.rcfonts": False,
            "pgf.preamble": cfg.get("latex_preamble", ""),
        }
    )


def parse_offset_ms(raw: str) -> float | None:
    """
    Parse normal offsets and the broken negative format seen in logs, e.g.
    '-5510.-691627' -> -5510.691627 and '0.-196480' -> -0.196480.
    """
    text = raw.strip()
    if ".-" in text:
        text = text.replace(".-", ".", 1)
        if not text.startswith("-"):
            text = "-" + text

    try:
        return float(text)
    except ValueError:
        print(f"  Warning: cannot parse forwarding offset '{raw}' - skipped.")
        return None


def extract_log_data(filepath: pathlib.Path, label: str) -> RunData:
    if not filepath.exists():
        sys.exit(f"Error: file not found - {filepath}")

    latencies_ms: list[float] = []
    retransmissions: list[RetransmissionSample] = []
    forwarding_offsets_ms_by_node: dict[int, list[float]] = {}

    with filepath.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            for match in LATENCY_RE.finditer(line):
                latencies_ms.append(float(match.group(1)))

            forward_match = FORWARD_RE.search(line)
            if forward_match:
                node = int(forward_match.group(1))
                offset_ms = parse_offset_ms(forward_match.group(2))
                if offset_ms is not None:
                    forwarding_offsets_ms_by_node.setdefault(node, []).append(offset_ms)

            stat_match = STAT_RE.search(line)
            if stat_match:
                node_match = NODE_RE.search(line)
                retransmissions.append(
                    RetransmissionSample(
                        node=node_match.group(1) if node_match else None,
                        packets_sent=int(stat_match.group(1)),
                        retransmissions=int(stat_match.group(2)),
                    )
                )

    return RunData(label, latencies_ms, retransmissions, forwarding_offsets_ms_by_node)


def collect_data(cfg: dict) -> list[RunData]:
    results: list[RunData] = []

    for raw_path, label in cfg["input_files"]:
        path = pathlib.Path(raw_path)
        print(f"Loading {path}  ({label}) ...")
        data = extract_log_data(path, label)
        configured_nodes = cfg.get("hop_offset_nodes", list(range(1, 9)))
        configured_forwarding_offsets = sum(
            len(data.forwarding_offsets_ms_by_node.get(node, []))
            for node in configured_nodes
        )
        print(
            f"  Latency: {len(data.latencies_ms)} values   "
            f"Retransmission stats: {len(data.retransmissions)} values   "
            f"Forwarding offsets: "
            f"{configured_forwarding_offsets} selected / "
            f"{sum(len(v) for v in data.forwarding_offsets_ms_by_node.values())} total"
        )
        results.append(data)

    if not results:
        sys.exit("Error: no data loaded. Check CONFIG['input_files'].")

    return results


def _style_ax(ax: plt.Axes, cfg: dict, title: str = "") -> None:
    if title and cfg.get("show_titles", True):
        ax.set_title(title, pad=6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if cfg["show_grid"]:
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.45)
        ax.grid(axis="y", which="minor", linestyle=":", alpha=0.25)


def _set_y_limits(ax: plt.Axes, series: list[list[float]], cfg: dict) -> None:
    values = [value for values in series for value in values]
    if not values:
        return

    bottom, top = ax.get_ylim()
    if cfg.get("y_axis_zero", True):
        bottom = 0
    top = max(top, max(values) * cfg.get("y_axis_top_margin", 1.12))
    ax.set_ylim(bottom, top)


def _plot_boxplot(
    ax: plt.Axes,
    series: list[list[float]],
    labels: list[str],
    colors: list[str],
    cfg: dict,
    title: str,
    y_label: str,
    value_format: str,
) -> None:
    non_empty = [(idx, values) for idx, values in enumerate(series) if values]
    if not non_empty:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
        ax.set_axis_off()
        return

    positions = [idx + 1 for idx, _ in non_empty]
    plot_series = [values for _, values in non_empty]

    bp = ax.boxplot(
        plot_series,
        positions=positions,
        widths=0.55,
        patch_artist=True,
        showmeans=cfg.get("show_means", True),
        meanprops={
            "marker": "D",
            "markerfacecolor": "#222222",
            "markeredgecolor": "#222222",
            "markersize": 4,
        },
        medianprops={
            "color": "#111111",
            "linewidth": 1.8,
            "alpha": cfg["median_alpha"],
        },
        whiskerprops={"color": "#333333", "alpha": cfg["line_alpha"]},
        capprops={"color": "#333333", "alpha": cfg["line_alpha"]},
        flierprops={
            "marker": "o",
            "markerfacecolor": "#777777",
            "markeredgecolor": "none",
            "alpha": 0.25,
            "markersize": 3,
        },
    )

    for patch, (idx, _) in zip(bp["boxes"], non_empty):
        patch.set_facecolor(colors[idx])
        patch.set_alpha(cfg["box_alpha"])
        patch.set_edgecolor("#333333")

    if cfg.get("show_points", True):
        rng = np.random.default_rng(42)
        for idx, values in non_empty:
            jitter = rng.normal(0, 0.045, len(values))
            x = np.full(len(values), idx + 1) + jitter
            ax.scatter(
                x,
                values,
                s=10,
                color=colors[idx],
                alpha=cfg["point_alpha"],
                linewidths=0,
                zorder=2,
            )

    if cfg.get("annotate_n", True):
        ymin, ymax = ax.get_ylim()
        y_text = ymin + (ymax - ymin) * 0.02
        font_size = cfg.get(
            "annotation_size_pt", max(cfg.get("font_size_pt", 10) - 2, 6)
        )
        for idx, values in non_empty:
            ax.text(
                idx + 1,
                y_text,
                f"n={len(values)}",
                ha="center",
                va="bottom",
                fontsize=font_size,
                alpha=0.75,
            )

    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_xlabel(cfg.get("clock_sync_x_label", ""))
    ax.set_ylabel(y_label)
    ax.yaxis.set_major_formatter(ticker.StrMethodFormatter(value_format))
    _set_y_limits(ax, series, cfg)
    _style_ax(ax, cfg, title)


def plot_latency_boxplot(ax: plt.Axes, data: list[RunData], cfg: dict) -> None:
    labels = [run.label for run in data]
    colors = [cfg["color_glossy"], cfg["color_udp"]][: len(data)]
    series = [run.latencies_ms for run in data]
    _plot_boxplot(
        ax,
        series,
        labels,
        colors,
        cfg,
        title="Latency",
        y_label=cfg["latency_y_label"],
        value_format="{x:.0f}",
    )


def plot_retransmission_boxplot(
    ax: plt.Axes, data: list[RunData], cfg: dict
) -> None:
    labels = [run.label for run in data]
    colors = [cfg["color_glossy"], cfg["color_udp"]][: len(data)]
    series = [run.retransmission_percentages for run in data]
    _plot_boxplot(
        ax,
        series,
        labels,
        colors,
        cfg,
        title="RUDP Retransmissions",
        y_label=cfg["retransmission_y_label"],
        value_format="{x:.1f}",
    )


def hop_offset_values_for_node(run: RunData, node: int, cfg: dict) -> list[float]:
    values = run.forwarding_offsets_ms_by_node.get(node, [])
    if values or node != 1 or not cfg.get("hop_offset_include_origin_zero", True):
        return values

    configured_nodes = cfg.get("hop_offset_nodes", list(range(1, 9)))
    sample_count = max(
        (len(run.forwarding_offsets_ms_by_node.get(n, [])) for n in configured_nodes if n != 1),
        default=0,
    )
    return [0.0] * sample_count


def plot_hop_offset_plot(ax: plt.Axes, data: list[RunData], cfg: dict) -> None:
    nodes = cfg.get("hop_offset_nodes", list(range(1, 9)))
    colors = [cfg["color_glossy"], cfg["color_udp"]][: len(data)]

    plotted = False
    rng = np.random.default_rng(7)

    for run_idx, run in enumerate(data):
        series = [hop_offset_values_for_node(run, node, cfg) for node in nodes]
        medians = [np.median(values) if values else np.nan for values in series]
        q1 = [np.percentile(values, 25) if values else np.nan for values in series]
        q3 = [np.percentile(values, 75) if values else np.nan for values in series]

        if not any(values for values in series):
            continue

        plotted = True
        color = colors[run_idx]
        x = np.array(nodes, dtype=float)

        if cfg.get("hop_offset_show_iqr", True):
            ax.fill_between(
                x,
                q1,
                q3,
                color=color,
                alpha=0.16,
                linewidth=0,
            )

        ax.plot(
            x,
            medians,
            marker="o",
            linewidth=1.8,
            markersize=4,
            color=color,
            label=run.label,
        )

        if cfg.get("hop_offset_show_points", False):
            for node, values in zip(nodes, series):
                if not values:
                    continue
                jitter = rng.normal(0, 0.04, len(values))
                ax.scatter(
                    np.full(len(values), node) + jitter,
                    values,
                    s=8,
                    color=color,
                    alpha=cfg["hop_offset_point_alpha"],
                    linewidths=0,
                    zorder=1,
                )

    if not plotted:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
        ax.set_axis_off()
        return

    y_scale = cfg.get("hop_offset_y_scale", "symlog")
    if y_scale == "symlog":
        ax.set_yscale(
            "symlog",
            linthresh=cfg.get("hop_offset_symlog_linthresh", 10),
        )
    elif y_scale == "log":
        ax.set_yscale("log")
    elif y_scale != "linear":
        sys.exit(f"Error: unsupported hop_offset_y_scale '{y_scale}'")

    ax.axhline(0, color="#222222", linewidth=0.8, alpha=0.6)
    ax.set_xticks(nodes)
    ax.set_xticklabels([f"{node}" for node in nodes])
    ax.set_xlabel("dwm3001 node")
    ax.set_ylabel(cfg["hop_offset_y_label"])
    ax.legend(frameon=False)
    _style_ax(ax, cfg, "Hop Offsets")


def hop_offset_trend(values: list[float], cfg: dict) -> float:
    if not values:
        return np.nan

    trend_stat = cfg.get("hop_offset_trend_stat", "median")
    if trend_stat == "median":
        return float(np.median(values))
    if trend_stat == "mean":
        return float(np.mean(values))

    sys.exit(f"Error: unsupported hop_offset_trend_stat '{trend_stat}'")


def plot_hop_offset_linear_trend_plot(
    ax: plt.Axes, data: list[RunData], cfg: dict
) -> None:
    nodes = cfg.get("hop_offset_nodes", list(range(1, 9)))
    colors = [cfg["color_glossy"], cfg["color_udp"]][: len(data)]
    trend_label = cfg.get("hop_offset_trend_stat", "median").capitalize()

    plotted = False
    for run_idx, run in enumerate(data):
        series = [hop_offset_values_for_node(run, node, cfg) for node in nodes]
        trend = [hop_offset_trend(values, cfg) for values in series]

        if not any(values for values in series):
            continue

        plotted = True
        ax.plot(
            nodes,
            trend,
            marker="o",
            linewidth=2.5,
            markersize=5,
            color=colors[run_idx],
            label=run.label,
        )

    if not plotted:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
        ax.set_axis_off()
        return

    ax.axhline(0, color="#222222", linewidth=0.8, alpha=0.6)
    ax.set_xticks(nodes)
    ax.set_xticklabels([f"{node}" for node in nodes])
    ax.set_xlabel("dwm3001 node")
    ax.set_ylabel(cfg["hop_offset_y_label"])
    ax.legend(frameon=False)
    _style_ax(ax, cfg, f"Hop Offset {trend_label}")


def save_figure(fig: plt.Figure, output_path: pathlib.Path, cfg: dict) -> None:
    fig.savefig(output_path, dpi=cfg["dpi"], bbox_inches=cfg.get("bbox_inches"))
    print(f"Saved {output_path}")

    if cfg.get("output_pgf", False) and output_path.suffix.lower() != ".pgf":
        pgf_path = output_path.with_suffix(".pgf")
        fig.savefig(pgf_path, bbox_inches=cfg.get("bbox_inches"))
        print(f"Saved {pgf_path}")


def save_panel(
    plot_func,
    data: list[RunData],
    cfg: dict,
    output_path: pathlib.Path | None,
    figure_size: tuple[float, float] = (4.2, 3.1),
) -> None:
    if output_path is None:
        return

    fig, ax = plt.subplots(figsize=figure_size)
    plot_func(ax, data, cfg)
    fig.tight_layout()
    save_figure(fig, output_path, cfg)
    plt.close(fig)


def print_figure_size_summary(
    cfg: dict, figure_sizes: dict[str, tuple[float, float]]
) -> None:
    print("\nFigure sizes")
    print("------------")
    rows = [
        ("output_file", "combined_width_fraction", "combined"),
        ("output_latency_boxplot", "latency_width_fraction", "latency"),
        (
            "output_retransmission_boxplot",
            "retransmission_width_fraction",
            "retransmission",
        ),
        ("output_hop_offset_plot", "hop_offset_width_fraction", "hop offsets"),
        (
            "output_hop_offset_linear_trend_plot",
            "hop_offset_linear_trend_width_fraction",
            "hop offset trend",
        ),
    ]
    for output_key, fraction_key, size_key in rows:
        output_file = cfg.get(output_key)
        if not output_file:
            continue
        width_in, height_in = figure_sizes[size_key]
        print(
            f"{pathlib.Path(output_file).name}: "
            f"{cfg[fraction_key]:.2f}\\textwidth, "
            f"{width_in:.2f} x {height_in:.2f} in"
        )


def print_summary(data: list[RunData], cfg: dict) -> None:
    print("\nSummary")
    print("-------")
    for run in data:
        lat = np.array(run.latencies_ms, dtype=float)
        rt = np.array(run.retransmission_percentages, dtype=float)

        if len(lat):
            print(
                f"{run.label}: latency n={len(lat)}, "
                f"median={np.median(lat):.2f} ms, mean={np.mean(lat):.2f} ms, "
                f"p95={np.percentile(lat, 95):.2f} ms"
            )
        else:
            print(f"{run.label}: latency n=0")

        if len(rt):
            print(
                f"{run.label}: retransmission stats n={len(rt)}, "
                f"median={np.median(rt):.2f}%, mean={np.mean(rt):.2f}%, "
                f"max={np.max(rt):.2f}%"
            )
        else:
            print(f"{run.label}: retransmission stats n=0")

        configured_nodes = cfg.get("hop_offset_nodes", list(range(1, 9)))
        hop_counts = [
            len(hop_offset_values_for_node(run, node, cfg))
            for node in configured_nodes
        ]
        if any(hop_counts):
            medians = []
            for node in configured_nodes:
                values = hop_offset_values_for_node(run, node, cfg)
                medians.append(np.median(values) if values else np.nan)
            median_text = ", ".join(
                f"{node}:{median:.2f}"
                for node, median in zip(configured_nodes, medians)
                if not np.isnan(median)
            )
            print(f"{run.label}: hop offset medians ms [{median_text}]")
        else:
            print(f"{run.label}: hop offsets n=0")


def main() -> None:
    cfg = CONFIG
    apply_plot_style(cfg)
    data = collect_data(cfg)
    print_summary(data, cfg)

    panels = []
    if cfg.get("show_latency_boxplot", True):
        panels.append(("latency", plot_latency_boxplot))
    if cfg.get("show_retransmission_boxplot", True):
        panels.append(("retransmission", plot_retransmission_boxplot))
    if cfg.get("show_hop_offset_plot", True):
        panels.append(("hop_offset", plot_hop_offset_plot))

    if not panels:
        sys.exit("Error: all plots are disabled in CONFIG.")

    fig_size = cfg.get("figure_size")
    if fig_size is None:
        fig_size = figure_size_from_width_fraction(
            cfg,
            "combined_width_fraction",
            "combined_width_in",
            "combined_aspect_ratio",
        )

    latency_figure_size = figure_size_from_width_fraction(
        cfg,
        "latency_width_fraction",
        "latency_width_in",
        "latency_aspect_ratio",
    )
    retransmission_figure_size = figure_size_from_width_fraction(
        cfg,
        "retransmission_width_fraction",
        "retransmission_width_in",
        "retransmission_aspect_ratio",
    )
    hop_offset_figure_size = figure_size_from_width_fraction(
        cfg,
        "hop_offset_width_fraction",
        "hop_offset_width_in",
        "hop_offset_aspect_ratio",
    )
    hop_offset_linear_trend_figure_size = figure_size_from_width_fraction(
        cfg,
        "hop_offset_linear_trend_width_fraction",
        "hop_offset_linear_trend_width_in",
        "hop_offset_linear_trend_aspect_ratio",
    )
    figure_sizes = {
        "combined": fig_size,
        "latency": latency_figure_size,
        "retransmission": retransmission_figure_size,
        "hop offsets": hop_offset_figure_size,
        "hop offset trend": hop_offset_linear_trend_figure_size,
    }
    print_figure_size_summary(cfg, figure_sizes)

    fig, axes = plt.subplots(1, len(panels), figsize=fig_size)
    if len(panels) == 1:
        axes = [axes]

    for ax, (_, plot_func) in zip(axes, panels):
        plot_func(ax, data, cfg)

    fig.tight_layout()

    save_panel(
        plot_latency_boxplot,
        data,
        cfg,
        pathlib.Path(cfg["output_latency_boxplot"])
        if cfg.get("output_latency_boxplot")
        else None,
        figure_size=latency_figure_size,
    )
    save_panel(
        plot_retransmission_boxplot,
        data,
        cfg,
        pathlib.Path(cfg["output_retransmission_boxplot"])
        if cfg.get("output_retransmission_boxplot")
        else None,
        figure_size=retransmission_figure_size,
    )
    save_panel(
        plot_hop_offset_plot,
        data,
        cfg,
        pathlib.Path(cfg["output_hop_offset_plot"])
        if cfg.get("output_hop_offset_plot")
        else None,
        figure_size=hop_offset_figure_size,
    )
    save_panel(
        plot_hop_offset_linear_trend_plot,
        data,
        cfg,
        pathlib.Path(cfg["output_hop_offset_linear_trend_plot"])
        if cfg.get("output_hop_offset_linear_trend_plot")
        else None,
        figure_size=hop_offset_linear_trend_figure_size,
    )

    output_file = cfg.get("output_file")
    if output_file:
        output_path = pathlib.Path(output_file)
        save_figure(fig, output_path, cfg)
    else:
        plt.show()


if __name__ == "__main__":
    main()
