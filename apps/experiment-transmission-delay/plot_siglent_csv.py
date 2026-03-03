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


def analyze_phase_delay(data: SiglentData) -> tuple[list[MatchedDelay], list[MatchedDelay], float, str]:
    time_unit = _normalize_time_unit(data.time_column_label)
    factor_to_ms = _unit_to_ms_factor(time_unit)

    ch1_threshold = 0.5 * (min(data.ch1_values) + max(data.ch1_values))
    ch2_threshold = 0.5 * (min(data.ch2_values) + max(data.ch2_values))

    ch1_edges = _detect_edges(data.time_values, data.ch1_values, ch1_threshold)
    ch2_edges = _detect_edges(data.time_values, data.ch2_values, ch2_threshold)

    ch1_rising = [edge for edge in ch1_edges if edge.kind == "rising"]
    ch1_falling = [edge for edge in ch1_edges if edge.kind == "falling"]
    ch2_rising = [edge for edge in ch2_edges if edge.kind == "rising"]
    ch2_falling = [edge for edge in ch2_edges if edge.kind == "falling"]

    rising_delays = _pair_edges(ch1_rising, ch2_rising, factor_to_ms)
    falling_delays = _pair_edges(ch1_falling, ch2_falling, factor_to_ms)
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


def _draw_delay_overlays(ax, delays: list[MatchedDelay], y_level: float) -> None:
    for item in delays:
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


def plot_signals(
    data: SiglentData,
    rising_delays: list[MatchedDelay],
    falling_delays: list[MatchedDelay],
    show_connectors: bool,
    lower_plot_mode: str,
    save_path: str | None,
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

    y_min = min(min(data.ch1_values), min(data.ch2_values))
    y_max = max(max(data.ch1_values), max(data.ch2_values))
    y_span = y_max - y_min if y_max > y_min else 1.0
    ch1_mid = 0.5 * (min(data.ch1_values) + max(data.ch1_values))
    ch2_mid = 0.5 * (min(data.ch2_values) + max(data.ch2_values))
    connector_y = 0.5 * (ch1_mid + ch2_mid)

    if show_connectors:
        _draw_delay_overlays(ax_signal, all_delays, y_level=connector_y)

    ax_signal.set_xlabel(f"Time ({horizontal_unit})")
    ax_signal.set_ylabel(f"Voltage ({y_unit})")
    ax_signal.set_title("Siglent SDS CSV Plot + CH1→CH2 Edge Delays")
    ax_signal.grid(True, alpha=0.3)
    ax_signal.legend()
    ax_signal.set_ylim(y_min - 0.05 * y_span, y_max + 0.05 * y_span)

    merged_y = [item.delay_ms for item in all_delays]
    if lower_plot_mode == "chronological":
        merged_x = list(range(1, len(all_delays) + 1))
        ax_delay.plot(merged_x, merged_y, "o-", color="tab:purple", label="Edge delay (time-ordered)")
        ax_delay.set_xlabel("Edge index")
        ax_delay.set_ylabel("Delay (ms)")
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
            ax_delay.axvline(avg_delay, color="black", linestyle="--", linewidth=1.2, label=f"avg={avg_delay:.3f} ms")
            ax_delay.legend()
        ax_delay.set_xlabel("Delay (ms)")
        ax_delay.set_ylabel("Count")
        ax_delay.set_title("Delay distribution (all edges)")
        ax_delay.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=160)
        print(f"Saved plot: {save_path}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read a Siglent SDS CSV log and plot CH1/CH2 signals."
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
        "--lower-plot",
        choices=["chronological", "distribution"],
        default="chronological",
        help="Lower subplot style: chronological (default) or distribution.",
    )
    parser.add_argument(
        "--save",
        nargs="?",
        const="__AUTO__",
        metavar="OUTPUT",
        default=None,
        help=(
            "Save plot image. Use '--save out.png' for explicit path, or '--save' "
            "to save next to CSV with .png extension."
        ),
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    save_path: str | None = None
    if args.save is not None:
        if args.save == "__AUTO__":
            save_path = str(csv_path.with_suffix(".png"))
        else:
            save_path = args.save

    data = read_siglent_csv(csv_path)
    print_summary(data)
    rising_delays, falling_delays, _, _ = analyze_phase_delay(data)
    print_delay_report(rising_delays, falling_delays)
    plot_signals(
        data,
        rising_delays,
        falling_delays,
        show_connectors=not args.no_connectors,
        lower_plot_mode=args.lower_plot,
        save_path=save_path,
    )


if __name__ == "__main__":
    main()
