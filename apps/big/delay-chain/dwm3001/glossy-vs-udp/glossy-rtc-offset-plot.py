#!/usr/bin/env python3
"""
glossy-rtc-offset-plot.py - Plot Glossy RTC clock drift estimates over time.

The script expects Glossy sync log lines like:

    dwm3001-3 > [00:17:54.011,932] <inf> glossy_lf: [glossy] sync ok: rtc_offset=3 ms  dwt_offset=4 ms

The reported rtc_offset is interpreted as the local device clock offset relative
to the Glossy root/initiator, which is dwm3001-1 in this experiment. By default,
each device's first observed offset is subtracted, so the plot shows oscillator
drift rather than the absolute startup offset.

CONFIGURATION
=============
Edit the CONFIG dict below to customise inputs and output.
"""

import os
import pathlib
import re
import sys
import tempfile
from dataclasses import dataclass

os.environ.setdefault(
    "MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "matplotlib")
)

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


# CONFIGURATION ---------------------------------------------------------------

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent

CONFIG = {
    # --- Input ---
    "input_file": SCRIPT_DIR / "glossyrun_clockdrift.txt",

    # --- Output ---
    # None -> interactive; otherwise save the figure to this path.
    "output_file": SCRIPT_DIR / "glossy-rtc-drift.svg",
    "dpi": 150,
    "font_family": "Times New Roman",

    # --- Device selection ---
    "reference_node": 1,
    "include_reference_node": True,
    # None -> auto-discover all devices in the input file.
    # Example: list(range(1, 9)) to show only dwm3001-1 through dwm3001-8.
    "nodes": None,

    # --- Plot options ---
    # "time" uses elapsed minutes from the first parsed sync line.
    # "sample" uses each device's sample index.
    "x_axis": "time",
    "normalize_to_first_sample": True,
    "y_scale": "linear",  # "linear" or "symlog"
    "symlog_linthresh": 1,
    "show_points": False,
    "point_size": 10,
    "point_alpha": 0.45,
    # How to draw the line over the quantized measurements:
    #   "linear_fit"          least-squares fit; best for oscillator drift
    #   "step"                discrete sample-and-hold measurements
    #   "connect_measurements" connect raw measurements directly
    #   "lerp_dense"          dense linear interpolation between raw measurements
    "line_mode": "linear_fit",
    "fit_degree": 1,
    "fit_points": 300,
    # Legacy switch: used only when line_mode is None.
    "interpolate_line": False,
    "interpolation_points_per_segment": 10,
    "non_interpolated_drawstyle": "steps-post",
    "line_alpha": 0.85,
    "line_width": 1.4,
    "show_grid": True,
    "show_titles": False,
    "figure_size": (8.0, 4.2),

    # --- Labels ---
    "x_label_time": "Elapsed time (min)",
    "x_label_sample": "Sync sample",
    "y_label": "RTC drift from first sync (ms)",
}

# END OF CONFIGURATION --------------------------------------------------------


SYNC_RE = re.compile(
    r"^dwm3001-(\d+)\s*>.*"
    r"\[(\d{2}):(\d{2}):(\d{2})\.(\d{3}),(\d{3})\]\s*"
    r"<inf>\s+glossy_lf:.*?"
    r"rtc_offset=([-+]?\d+(?:\.\d+)?)\s*ms"
)


@dataclass
class OffsetSample:
    time_s: float
    offset_ms: float


def apply_font(cfg: dict) -> None:
    fam = cfg.get("font_family", "sans-serif")
    if fam == "Times New Roman":
        plt.rcParams["font.family"] = "serif"
        plt.rcParams["font.serif"] = ["Times New Roman", "Times", "DejaVu Serif"]
    else:
        plt.rcParams["font.family"] = fam


def parse_log_time_s(match: re.Match) -> float:
    hours = int(match.group(2))
    minutes = int(match.group(3))
    seconds = int(match.group(4))
    millis = int(match.group(5))
    micros = int(match.group(6))

    return (
        hours * 3600
        + minutes * 60
        + seconds
        + millis / 1000.0
        + micros / 1000000.0
    )


def extract_offsets(filepath: pathlib.Path) -> dict[int, list[OffsetSample]]:
    if not filepath.exists():
        sys.exit(f"Error: file not found - {filepath}")

    samples_by_node: dict[int, list[OffsetSample]] = {}
    day_offset_s = 0.0
    last_time_s: float | None = None

    with filepath.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            match = SYNC_RE.search(line)
            if not match:
                continue

            raw_time_s = parse_log_time_s(match)
            candidate_time_s = raw_time_s + day_offset_s
            if (
                last_time_s is not None
                and candidate_time_s < last_time_s - 12 * 3600
            ):
                day_offset_s += 24 * 3600
                candidate_time_s = raw_time_s + day_offset_s

            time_s = candidate_time_s
            last_time_s = max(last_time_s, time_s) if last_time_s is not None else time_s

            node = int(match.group(1))
            offset_ms = float(match.group(7))
            samples_by_node.setdefault(node, []).append(OffsetSample(time_s, offset_ms))

    if not samples_by_node:
        sys.exit(f"Error: no Glossy RTC offset samples found in {filepath}")

    return samples_by_node


def selected_nodes(samples_by_node: dict[int, list[OffsetSample]], cfg: dict) -> list[int]:
    nodes = cfg.get("nodes")
    if nodes is None:
        nodes = sorted(samples_by_node)
    else:
        nodes = [int(node) for node in nodes]

    reference_node = cfg.get("reference_node", 1)
    if not cfg.get("include_reference_node", True):
        nodes = [node for node in nodes if node != reference_node]

    return [node for node in nodes if node in samples_by_node]


def maybe_normalize_offsets(samples: list[OffsetSample], cfg: dict) -> list[float]:
    values = [sample.offset_ms for sample in samples]
    if cfg.get("normalize_to_first_sample", False) and values:
        first = values[0]
        return [value - first for value in values]
    return values


def drift_slope_ms_per_min(samples: list[OffsetSample], cfg: dict) -> float | None:
    if len(samples) < 2:
        return None

    t0 = samples[0].time_s
    x_min = np.array([(sample.time_s - t0) / 60.0 for sample in samples], dtype=float)
    y_ms = np.array(maybe_normalize_offsets(samples, cfg), dtype=float)

    if np.allclose(x_min, x_min[0]):
        return None

    slope, _ = np.polyfit(x_min, y_ms, 1)
    return float(slope)


def interpolate_series(
    x: list[float], y: list[float], points_per_segment: int
) -> tuple[list[float], list[float]]:
    if len(x) < 2 or points_per_segment <= 0:
        return x, y

    x_interp: list[float] = []
    y_interp: list[float] = []

    for idx in range(len(x) - 1):
        segment_x = np.linspace(x[idx], x[idx + 1], points_per_segment + 2)
        segment_y = np.linspace(y[idx], y[idx + 1], points_per_segment + 2)

        if idx > 0:
            segment_x = segment_x[1:]
            segment_y = segment_y[1:]

        x_interp.extend(segment_x.tolist())
        y_interp.extend(segment_y.tolist())

    return x_interp, y_interp


def fitted_series(
    x: list[float], y: list[float], degree: int, points: int
) -> tuple[list[float], list[float]]:
    if len(x) < 2:
        return x, y

    fit_degree = max(1, min(degree, len(x) - 1))
    coeffs = np.polyfit(np.array(x, dtype=float), np.array(y, dtype=float), fit_degree)
    x_fit = np.linspace(min(x), max(x), max(points, 2))
    y_fit = np.polyval(coeffs, x_fit)

    return x_fit.tolist(), y_fit.tolist()


def line_mode(cfg: dict) -> str:
    mode = cfg.get("line_mode")
    if mode:
        return mode

    return "lerp_dense" if cfg.get("interpolate_line", False) else "step"


def print_summary(samples_by_node: dict[int, list[OffsetSample]], nodes: list[int], cfg: dict) -> None:
    print("\nSummary")
    print("-------")
    for node in nodes:
        samples = samples_by_node[node]
        values = np.array(maybe_normalize_offsets(samples, cfg), dtype=float)
        slope = drift_slope_ms_per_min(samples, cfg)
        slope_text = "n/a" if slope is None else f"{slope:.4f} ms/min"
        first_offset = samples[0].offset_ms
        print(
            f"dwm3001-{node}: n={len(samples)}, "
            f"initial={first_offset:.2f} ms, "
            f"min={np.min(values):.2f} ms, median={np.median(values):.2f} ms, "
            f"max={np.max(values):.2f} ms, slope={slope_text}"
        )


def style_axis(ax: plt.Axes, cfg: dict) -> None:
    if cfg.get("show_titles", True):
        ax.set_title("Glossy RTC Drift Over Time", fontsize=11, pad=6)

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    if cfg.get("show_grid", True):
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(axis="y", which="major", linestyle="--", alpha=0.45)
        ax.grid(axis="y", which="minor", linestyle=":", alpha=0.25)
        ax.grid(axis="x", which="major", linestyle=":", alpha=0.20)


def plot_offsets(
    ax: plt.Axes,
    samples_by_node: dict[int, list[OffsetSample]],
    nodes: list[int],
    cfg: dict,
) -> None:
    first_time_s = min(sample.time_s for samples in samples_by_node.values() for sample in samples)
    cmap = plt.get_cmap("tab20")

    for idx, node in enumerate(nodes):
        samples = samples_by_node[node]
        offsets_ms = maybe_normalize_offsets(samples, cfg)

        if cfg.get("x_axis", "time") == "time":
            x = [(sample.time_s - first_time_s) / 60.0 for sample in samples]
            ax.set_xlabel(cfg["x_label_time"])
        elif cfg.get("x_axis") == "sample":
            x = list(range(len(samples)))
            ax.set_xlabel(cfg["x_label_sample"])
        else:
            sys.exit(f"Error: unsupported x_axis '{cfg.get('x_axis')}'")

        color = cmap(idx % cmap.N)
        label = f"dwm3001-{node}"
        if node == cfg.get("reference_node", 1):
            label += " (root)"

        mode = line_mode(cfg)
        drawstyle = "default"

        if mode == "linear_fit":
            line_x, line_y = fitted_series(
                x,
                offsets_ms,
                cfg.get("fit_degree", 1),
                cfg.get("fit_points", 300),
            )
        elif mode == "lerp_dense":
            line_x, line_y = interpolate_series(
                x,
                offsets_ms,
                cfg.get("interpolation_points_per_segment", 10),
            )
        elif mode == "connect_measurements":
            line_x = x
            line_y = offsets_ms
        elif mode == "step":
            line_x = x
            line_y = offsets_ms
            drawstyle = cfg.get("non_interpolated_drawstyle", "steps-post")
        else:
            sys.exit(f"Error: unsupported line_mode '{mode}'")

        ax.plot(
            line_x,
            line_y,
            color=color,
            alpha=cfg["line_alpha"],
            linewidth=cfg["line_width"],
            drawstyle=drawstyle,
            label=label,
        )

        if cfg.get("show_points", True):
            ax.scatter(
                x,
                offsets_ms,
                color=color,
                alpha=cfg["point_alpha"],
                s=cfg["point_size"],
                linewidths=0,
            )

    y_scale = cfg.get("y_scale", "linear")
    if y_scale == "symlog":
        ax.set_yscale("symlog", linthresh=cfg.get("symlog_linthresh", 1))
    elif y_scale != "linear":
        sys.exit(f"Error: unsupported y_scale '{y_scale}'")

    ax.axhline(0, color="#222222", linewidth=0.8, alpha=0.65)
    ax.set_ylabel(cfg["y_label"])
    ax.legend(frameon=False, fontsize=8, ncol=2)
    style_axis(ax, cfg)


def main() -> None:
    cfg = CONFIG
    apply_font(cfg)

    input_file = pathlib.Path(cfg["input_file"])
    print(f"Loading {input_file} ...")
    samples_by_node = extract_offsets(input_file)
    nodes = selected_nodes(samples_by_node, cfg)
    if not nodes:
        sys.exit("Error: no selected nodes have RTC offset samples.")

    total_samples = sum(len(samples_by_node[node]) for node in nodes)
    print(f"  Nodes: {', '.join(f'dwm3001-{node}' for node in nodes)}")
    print(f"  Selected samples: {total_samples}")
    print_summary(samples_by_node, nodes, cfg)

    fig, ax = plt.subplots(figsize=cfg["figure_size"])
    plot_offsets(ax, samples_by_node, nodes, cfg)
    fig.tight_layout()

    output_file = cfg.get("output_file")
    if output_file:
        output_path = pathlib.Path(output_file)
        fig.savefig(output_path, dpi=cfg["dpi"], bbox_inches="tight")
        print(f"Saved {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
