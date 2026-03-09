#!/usr/bin/env python3

"""
Siglent SDS CSV edge-delay analyzer and plotter.

Expected input data
-------------------
- CSV export from a Siglent SDS oscilloscope (e.g. SDS1104X-E style export).
- Two analog channels are expected: CH1 and CH2.
- Metadata/header section followed by data header:
    - "Source,CH1,CH2"
    - "Second,Value,Value"
- Data rows contain three numeric columns:
    - time (typically in seconds)
    - CH1 voltage
    - CH2 voltage

Purpose
-------
This tool helps estimate transmission + processing latency between two MCUs where
MCU A toggles a GPIO and MCU B toggles another GPIO upon receiving/processing a
command. The CH1->CH2 edge delay approximates the end-to-end reaction time.

When a known logical delay is configured in Lingua Franca port connections, you
can pass it via --delay-offset-ms to subtract it from each measured edge delay.
If that logical delay exceeds the actual transmission delay, it absorbs
transmission effects; the remaining value after subtraction reflects clock
synchronization error.
"""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import mean


@dataclass
class SiglentData:
    metadata: dict[str, list[str]]
    channel_names: tuple[str, str]
    time_column_label: str
    time_values: list[float]
    ch1_values: list[float]
    ch2_values: list[float]


@dataclass
class EdgeEvent:
    kind: str
    time: float


@dataclass
class MatchedDelay:
    kind: str
    ch1_time: float
    ch2_time: float
    delay_ms: float


def _clean_row(row: list[str]) -> list[str]:
    return [item.strip() for item in row]


def read_siglent_csv(csv_path: Path) -> SiglentData:
    metadata: dict[str, list[str]] = {}
    channel_names = ("CH1", "CH2")
    time_column_label = "Second"
    time_values: list[float] = []
    ch1_values: list[float] = []
    ch2_values: list[float] = []

    with csv_path.open("r", newline="") as file:
        reader = csv.reader(file)
        in_data_section = False

        for raw_row in reader:
            row = _clean_row(raw_row)
            if not row or not row[0]:
                continue

            first_col = row[0]

            if not in_data_section:
                if first_col == "Source" and len(row) >= 3:
                    channel_names = (row[1] or "CH1", row[2] or "CH2")
                    metadata[first_col] = [row[1], row[2]]
                    continue

                if first_col == "Second":
                    time_column_label = row[0]
                    in_data_section = True
                    continue

                metadata[first_col] = [value for value in row[1:] if value]
                continue

            if len(row) < 3:
                continue

            try:
                time_values.append(float(row[0]))
                ch1_values.append(float(row[1]))
                ch2_values.append(float(row[2]))
            except ValueError:
                continue

    if not time_values:
        raise ValueError("No signal samples found in CSV file.")

    return SiglentData(
        metadata=metadata,
        channel_names=channel_names,
        time_column_label=time_column_label,
        time_values=time_values,
        ch1_values=ch1_values,
        ch2_values=ch2_values,
    )


def print_summary(data: SiglentData) -> None:
    def first_value(key: str, default: str = "-") -> str:
        values = data.metadata.get(key, [])
        return values[0] if values else default

    print("=== Siglent CSV Summary ===")
    print(f"Model: {first_value('Model Number')}")
    print(f"Serial: {first_value('Serial Number')}")
    print(f"Software: {first_value('Software Version')}")
    print(f"Record Length: {first_value('Record Length')}")
    print(f"Sample Interval: {first_value('Sample Interval')}")
    print(f"Time Column: {data.time_column_label}")
    print(f"Horizontal Units: {first_value('Horizontal Units')}")
    print(f"Channels: {data.channel_names[0]}, {data.channel_names[1]}")
    print(f"Samples loaded: {len(data.time_values)}")
    print(f"Time range: {data.time_values[0]} to {data.time_values[-1]}")
    print(
        f"{data.channel_names[0]} min/max: "
        f"{min(data.ch1_values):.6g} / {max(data.ch1_values):.6g}"
    )
    print(
        f"{data.channel_names[1]} min/max: "
        f"{min(data.ch2_values):.6g} / {max(data.ch2_values):.6g}"
    )


def _unit_to_ms_factor(horizontal_unit: str) -> float:
    unit = horizontal_unit.strip().lower()
    if unit in {"s", "sec", "second", "seconds"}:
        return 1000.0
    if unit in {"ms", "millisecond", "milliseconds"}:
        return 1.0
    if unit in {"us", "µs", "microsecond", "microseconds"}:
        return 0.001
    if unit in {"ns", "nanosecond", "nanoseconds"}:
        return 0.000001
    return 1.0


def _normalize_time_unit(label: str) -> str:
    unit = label.strip().lower()
    if unit in {"second", "seconds", "s", "sec"}:
        return "s"
    if unit in {"millisecond", "milliseconds", "ms"}:
        return "ms"
    if unit in {"microsecond", "microseconds", "us", "µs"}:
        return "us"
    if unit in {"nanosecond", "nanoseconds", "ns"}:
        return "ns"
    return label.strip() or "s"


def _invert_levels(values: list[float]) -> list[float]:
    if not values:
        return values
    low = min(values)
    high = max(values)
    return [low + high - value for value in values]


def _detect_edges(time_values: list[float], signal_values: list[float], threshold: float) -> list[EdgeEvent]:
    edges: list[EdgeEvent] = []
    for idx in range(1, len(signal_values)):
        prev_v = signal_values[idx - 1]
        curr_v = signal_values[idx]
        prev_t = time_values[idx - 1]
        curr_t = time_values[idx]

        is_rising = prev_v < threshold <= curr_v
        is_falling = prev_v > threshold >= curr_v
        if not (is_rising or is_falling):
            continue

        if curr_v == prev_v:
            crossing_time = curr_t
        else:
            alpha = (threshold - prev_v) / (curr_v - prev_v)
            crossing_time = prev_t + alpha * (curr_t - prev_t)

        edges.append(EdgeEvent(kind="rising" if is_rising else "falling", time=crossing_time))

    return edges


def _pair_edges(ch1_edges: list[EdgeEvent], ch2_edges: list[EdgeEvent], factor_to_ms: float) -> list[MatchedDelay]:
    matched: list[MatchedDelay] = []
    ch2_index = 0

    for edge in ch1_edges:
        while ch2_index < len(ch2_edges) and ch2_edges[ch2_index].time < edge.time:
            ch2_index += 1

        if ch2_index >= len(ch2_edges):
            break

        ch2_edge = ch2_edges[ch2_index]
        matched.append(
            MatchedDelay(
                kind=edge.kind,
                ch1_time=edge.time,
                ch2_time=ch2_edge.time,
                delay_ms=(ch2_edge.time - edge.time) * factor_to_ms,
            )
        )
        ch2_index += 1

    return matched


def analyze_phase_delay(
    data: SiglentData,
    separate_edge_kinds: bool = True,
) -> tuple[list[MatchedDelay], list[MatchedDelay], float, str]:
    time_unit = _normalize_time_unit(data.time_column_label)
    factor_to_ms = _unit_to_ms_factor(time_unit)

    ch1_threshold = 0.5 * (min(data.ch1_values) + max(data.ch1_values))
    ch2_threshold = 0.5 * (min(data.ch2_values) + max(data.ch2_values))

    ch1_edges = _detect_edges(data.time_values, data.ch1_values, ch1_threshold)
    ch2_edges = _detect_edges(data.time_values, data.ch2_values, ch2_threshold)

    if separate_edge_kinds:
        ch1_rising = [edge for edge in ch1_edges if edge.kind == "rising"]
        ch1_falling = [edge for edge in ch1_edges if edge.kind == "falling"]
        ch2_rising = [edge for edge in ch2_edges if edge.kind == "rising"]
        ch2_falling = [edge for edge in ch2_edges if edge.kind == "falling"]

        rising_delays = _pair_edges(ch1_rising, ch2_rising, factor_to_ms)
        falling_delays = _pair_edges(ch1_falling, ch2_falling, factor_to_ms)
        return rising_delays, falling_delays, factor_to_ms, time_unit

    all_delays = _pair_edges(ch1_edges, ch2_edges, factor_to_ms)
    rising_delays = [item for item in all_delays if item.kind == "rising"]
    falling_delays = [item for item in all_delays if item.kind == "falling"]
    return rising_delays, falling_delays, factor_to_ms, time_unit


def print_delay_report(rising_delays: list[MatchedDelay], falling_delays: list[MatchedDelay]) -> None:
    def print_stats(name: str, values: list[float]) -> None:
        if not values:
            print(f"{name}: no matched edges")
            return
        print(
            f"{name}: count={len(values)}, min={min(values):.6f} ms, "
            f"max={max(values):.6f} ms, avg={mean(values):.6f} ms"
        )

    print("=== Phase/Delay Analysis (CH1 -> CH2) ===")

    for idx, delay in enumerate(rising_delays, start=1):
        print(
            f"Rising #{idx:03d}: CH1={delay.ch1_time:.9g}, "
            f"CH2={delay.ch2_time:.9g}, delay={delay.delay_ms:.6f} ms"
        )

    for idx, delay in enumerate(falling_delays, start=1):
        print(
            f"Falling #{idx:03d}: CH1={delay.ch1_time:.9g}, "
            f"CH2={delay.ch2_time:.9g}, delay={delay.delay_ms:.6f} ms"
        )

    print_stats("Rising stats", [item.delay_ms for item in rising_delays])
    print_stats("Falling stats", [item.delay_ms for item in falling_delays])


def _build_ping_pong_levels(count: int, y_min: float, y_max: float, levels: int = 9) -> list[float]:
    if count <= 0:
        return []

    if y_max <= y_min:
        return [y_min] * count

    levels = max(2, levels)
    up = [y_min + (y_max - y_min) * idx / (levels - 1) for idx in range(levels)]
    down = up[-2:0:-1]
    pattern = up + down

    result: list[float] = []
    while len(result) < count:
        result.extend(pattern)
    return result[:count]


def _draw_delay_overlays(ax, delays: list[MatchedDelay], y_levels: list[float]) -> None:
    for item, y_level in zip(delays, y_levels):
        ax.hlines(
            y=y_level,
            xmin=item.ch1_time,
            xmax=item.ch2_time,
            colors="tab:purple",
            linewidth=1.4,
            alpha=0.9,
            linestyles="--",
        )
        ax.plot(item.ch1_time, y_level, marker="o", color="tab:purple", markersize=3, alpha=0.9)
        ax.plot(item.ch2_time, y_level, marker="o", color="tab:purple", markersize=3, alpha=0.9)


def _compute_midpoint_level(data: SiglentData) -> float:
    ch1_low = min(data.ch1_values)
    ch2_low = min(data.ch2_values)
    ch1_high = max(data.ch1_values)
    ch2_high = max(data.ch2_values)

    common_low = max(ch1_low, ch2_low)
    common_high = min(ch1_high, ch2_high)

    if common_high >= common_low:
        return 0.5 * (common_low + common_high)

    overall_low = min(ch1_low, ch2_low)
    overall_high = max(ch1_high, ch2_high)
    return 0.5 * (overall_low + overall_high)


def _add_middle_zoom_inset(
    ax_signal,
    data: SiglentData,
    all_delays: list[MatchedDelay],
    connector_y_levels: list[float],
    y_min: float,
    y_max: float,
) -> None:
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

    all_x = data.time_values
    if not all_x:
        return

    x_min, x_max = min(all_x), max(all_x)
    x_span = x_max - x_min
    if x_span <= 0:
        return

    y_span = y_max - y_min
    if y_span <= 0:
        return

    x_center = x_min + 0.5 * x_span
    y_center = y_min + 0.5 * y_span

    x_half = 0.04 * x_span
    y_half = 0.46 * y_span

    inset = inset_axes(
        ax_signal,
        width="39%",
        height="39%",
        loc="center",
        borderpad=0.8,
    )
    inset.patch.set_facecolor("white")
    inset.patch.set_alpha(0.9)
    inset.plot(data.time_values, data.ch1_values, linewidth=1)
    inset.plot(data.time_values, data.ch2_values, linewidth=1)

    if connector_y_levels:
        _draw_delay_overlays(inset, all_delays, y_levels=connector_y_levels)

    inset.set_xlim(x_center - x_half, x_center + x_half)
    inset.set_ylim(y_center - y_half, y_center + y_half)
    inset.grid(True, linestyle=":", alpha=0.4)
    inset.set_title("Zoom", fontsize=9, bbox={"facecolor": "white", "alpha": 1, "edgecolor": "none", "pad": 1.8})
    inset.tick_params(labelsize=8)
    for label in inset.get_xticklabels() + inset.get_yticklabels():
        label.set_bbox({"facecolor": "white", "alpha": 1, "edgecolor": "none", "pad": 2.9})

    mark_inset(ax_signal, inset, loc1=1, loc2=3, fc="none", ec="0.35", lw=0.9)


def plot_signals(
    data: SiglentData,
    rising_delays: list[MatchedDelay],
    falling_delays: list[MatchedDelay],
    show_connectors: bool,
    connector_mode: str,
    show_zoom: bool,
    lower_plot_mode: str,
    delay_offset_ms: float,
    save_path: str | None,
    format: str,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "matplotlib is required for plotting. Install it with: pip install matplotlib"
        ) from exc

    horizontal_unit = _normalize_time_unit(data.time_column_label)
    vertical_units = data.metadata.get("Vertical Units", [])
    y_unit = "V"
    if vertical_units:
        y_unit = vertical_units[0].split(":")[-1]

    all_delays = sorted([*rising_delays, *falling_delays], key=lambda item: item.ch1_time)

    fig, (ax_signal, ax_delay) = plt.subplots(2, 1, figsize=(13, 8), sharex=False)

    ax_signal.plot(data.time_values, data.ch1_values, label=data.channel_names[0], linewidth=1)
    ax_signal.plot(data.time_values, data.ch2_values, label=data.channel_names[1], linewidth=1)

    lowest_val = min(min(data.ch1_values), min(data.ch2_values))
    highest_val = max(max(data.ch1_values), max(data.ch2_values))

    y_min = lowest_val - 0.5
    y_max = highest_val + 0.5
    y_span = y_max - y_min if y_max > y_min else 1.0

    connector_y_levels: list[float] = []

    if show_connectors:
        if connector_mode == "midpoint":
            midpoint = _compute_midpoint_level(data)
            connector_y_levels = [midpoint] * len(all_delays)
        else:
            connector_margin = 0.06 * y_span
            connector_y_levels = _build_ping_pong_levels(
                count=len(all_delays),
                y_min=y_min + connector_margin,
                y_max=y_max - connector_margin,
            )
        _draw_delay_overlays(ax_signal, all_delays, y_levels=connector_y_levels)

    if show_zoom:
        _add_middle_zoom_inset(
            ax_signal=ax_signal,
            data=data,
            all_delays=all_delays,
            connector_y_levels=connector_y_levels,
            y_min=y_min,
            y_max=y_max,
        )

    ax_signal.set_xlabel(f"Time ({horizontal_unit})")
    ax_signal.set_ylabel(f"Voltage ({y_unit})")
    ax_signal.set_title("Siglent SDS CSV Plot + CH1→CH2 Edge Delays")
    ax_signal.grid(True, alpha=0.3)
    ax_signal.legend()
    ax_signal.set_ylim(y_min, y_max)

    # Subtract configured logical delay from measured edge delay.
    # In LF setups where logical delay exceeds transmission delay, this yields
    # the residual clock synchronization error.
    merged_y = [item.delay_ms - delay_offset_ms for item in all_delays]
    lower_y_label = "Clock Sync Error (ms)" if delay_offset_ms != 0.0 else "Delay (ms)"
    if lower_plot_mode == "chronological":
        merged_x = list(range(1, len(all_delays) + 1))
        lower_series_label = "Clock sync error (time-ordered)" if delay_offset_ms != 0.0 else "Edge delay (time-ordered)"
        ax_delay.plot(merged_x, merged_y, "o-", color="tab:purple", label=lower_series_label)
        ax_delay.set_xlabel("Edge index")
        ax_delay.set_ylabel(lower_y_label)
        if delay_offset_ms != 0.0:
            ax_delay.set_title("Per-edge clock sync error (rising+falling merged, chronological)")
        else:
            ax_delay.set_title("Per-edge delay (rising+falling merged, chronological)")
        ax_delay.grid(True, alpha=0.3)
        ax_delay.legend()

        if all_delays:
            max_delay = max(merged_y)
            min_delay = min(merged_y)
            if max_delay == min_delay:
                ax_delay.set_ylim(min_delay - 0.1, max_delay + 0.1)
    else:
        if merged_y:
            bins = min(30, max(8, int(len(merged_y) ** 0.5)))
            ax_delay.hist(merged_y, bins=bins, color="tab:purple", alpha=0.75, edgecolor="black")
            avg_delay = mean(merged_y)
            avg_label = "avg error" if delay_offset_ms != 0.0 else "avg"
            ax_delay.axvline(avg_delay, color="black", linestyle="--", linewidth=1.2, label=f"{avg_label}={avg_delay:.3f} ms")
            ax_delay.legend()
        ax_delay.set_xlabel(lower_y_label)
        ax_delay.set_ylabel("Count")
        if delay_offset_ms != 0.0:
            ax_delay.set_title("Clock Sync Error")
        else:
            ax_delay.set_title("Delay distribution (all edges)")
        ax_delay.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        if format == "svg":
            fig.savefig(save_path, format="svg")
        else:
            fig.savefig(save_path, dpi=160)
        print(f"Saved plot: {save_path}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Read a Siglent SDS CSV log and plot CH1/CH2 signals. "
            "Optionally subtract a known logical delay to inspect clock sync error."
        )
    )
    parser.add_argument(
        "csv_file",
        help="Path to Siglent CSV file.",
    )
    parser.add_argument(
        "--no-connectors",
        action="store_true",
        help="Disable dashed CH1->CH2 delay connectors in the upper plot.",
    )
    parser.add_argument(
        "--connector-mode",
        choices=["midpoint", "ping-pong"],
        default="midpoint",
        help=(
            "Layout of CH1->CH2 dashed delay connectors in the upper plot. "
            "'midpoint' draws all connectors at one shared y-level (default). "
            "'ping-pong' staggers connector y-levels to reduce overlap."
        ),
    )
    parser.add_argument(
        "--lower-plot",
        choices=["chronological", "distribution"],
        default="chronological",
        help="Lower subplot style: chronological (default) or distribution.",
    )
    parser.add_argument(
        "--zoom",
        action="store_true",
        help="Enable a small zoom inset centered on the middle part of the upper signal plot.",
    )
    parser.add_argument(
        "--invert-ch2",
        action="store_true",
        help="Invert CH2 signal levels (swap high/low around CH2 mid-level) before analysis and plotting.",
    )
    parser.add_argument(
        "--chronological-edge-pairing",
        action="store_true",
        help=(
            "Pair CH1 and CH2 edges in chronological order regardless of rising/falling kind. "
            "Default behavior pairs rising with rising and falling with falling."
        ),
    )
    parser.add_argument(
        "--delay-offset-ms",
        type=float,
        default=0.0,
        metavar="MS",
        help=(
            "Delay offset in milliseconds subtracted from each measured CH1->CH2 delay "
            "for the lower plot. Set this to the Lingua Franca logical delay used in "
            "port connections. If this logical delay exceeds transmission delay, it "
            "absorbs transmission effects; the residual after subtraction represents "
            "clock synchronization error. Default: 0.0"
        ),
    )
    parser.add_argument(
        "--save",
        nargs="?",
        const="__AUTO__",
        metavar="OUTPUT",
        default=None,
        help=(
            "Save plot image. Use '--save out.png' for explicit path, or '--save' "
            "to save next to CSV with the appropriate file type (see --format)."
        ),
    )
    parser.add_argument(
      "--format",
      choices=["png", "svg"],
      default="svg",
      help="Output image format when using --save. Default: svg (scalable vector graphics)."
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed phase/delay edge matches and delay statistics in console.",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    save_path: str | None = None
    if args.save is not None:
        if args.save == "__AUTO__":
            suffix = "-zoom" if args.zoom else ""
            save_path = str(csv_path.with_name(f"{csv_path.stem}{suffix}.{args.format}"))
        else:
            save_path = args.save

    data = read_siglent_csv(csv_path)
    if args.invert_ch2:
        data.ch2_values = _invert_levels(data.ch2_values)
    print_summary(data)
    rising_delays, falling_delays, _, _ = analyze_phase_delay(
        data,
        separate_edge_kinds=not args.chronological_edge_pairing,
    )
    if args.verbose:
        print_delay_report(rising_delays, falling_delays)
    plot_signals(
        data,
        rising_delays,
        falling_delays,
        show_connectors=not args.no_connectors,
        connector_mode=args.connector_mode,
        show_zoom=args.zoom,
        lower_plot_mode=args.lower_plot,
        delay_offset_ms=args.delay_offset_ms,
        save_path=save_path,
        format=args.format,
    )


if __name__ == "__main__":
    main()
