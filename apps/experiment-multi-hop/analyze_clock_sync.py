import argparse
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ClockSyncSample:
  line_no: int
  wall_time_s: float | None
  device_name: str | None
  federate_id: int
  offset_ms: float


CLOCK_SYNC_PATTERN = re.compile(
  r"\[CLOCK_SYNC\].*?offset:\s*(?P<offset>-?\d+(?:\.\d+)?)\s*ms"
)

VALUE_LINE_PATTERN = re.compile(
  r"\[(?P<federate_id>-?\d+)\]\s+Value\s+"
)


@dataclass(frozen=True)
class ParsedPrefix:
  wall_time_s: float | None
  device_name: str | None
  log_segment: str


def parse_prefixed_line(raw_line: str) -> ParsedPrefix:
  stripped = raw_line.strip()
  if not stripped:
    return ParsedPrefix(wall_time_s=None, device_name=None, log_segment="")

  # iot-lab serial aggregation format can be:
  # <wall-time>;<device>;...actual-log-line...
  parts = [part.strip() for part in stripped.split(";")]

  if len(parts) >= 3:
    wall_time_s: float | None
    try:
      wall_time_s = float(parts[0])
    except ValueError:
      wall_time_s = None
    return ParsedPrefix(
      wall_time_s=wall_time_s,
      device_name=parts[1] or None,
      log_segment=parts[-1],
    )

  return ParsedPrefix(wall_time_s=None, device_name=None, log_segment=stripped)


def load_clock_sync_samples(log_file: str | Path) -> tuple[list[ClockSyncSample], int, int]:
  pending_samples: list[tuple[int, float | None, str | None, float]] = []
  device_to_federate: dict[str, int] = {}
  ignored_lines = 0

  with Path(log_file).open("r", encoding="utf-8") as f:
    for line_no, raw_line in enumerate(f, start=1):
      parsed = parse_prefixed_line(raw_line)
      log_segment = parsed.log_segment
      if not log_segment:
        ignored_lines += 1
        continue

      value_match = VALUE_LINE_PATTERN.search(log_segment)
      if value_match and parsed.device_name is not None:
        federate_id = int(value_match.group("federate_id"))
        known = device_to_federate.get(parsed.device_name)
        if known is None:
          device_to_federate[parsed.device_name] = federate_id

      clock_sync_match = CLOCK_SYNC_PATTERN.search(log_segment)
      if clock_sync_match is None:
        ignored_lines += 1
        continue

      pending_samples.append(
        (
          line_no,
          parsed.wall_time_s,
          parsed.device_name,
          float(clock_sync_match.group("offset")),
        )
      )

  resolved_samples: list[ClockSyncSample] = []
  unmapped_clock_sync_lines = 0

  for line_no, wall_time_s, device_name, offset_ms in pending_samples:
    if device_name is None:
      unmapped_clock_sync_lines += 1
      continue

    federate_id = device_to_federate.get(device_name)
    if federate_id is None:
      unmapped_clock_sync_lines += 1
      continue

    resolved_samples.append(
      ClockSyncSample(
        line_no=line_no,
        wall_time_s=wall_time_s,
        device_name=device_name,
        federate_id=federate_id,
        offset_ms=offset_ms,
      )
    )

  return resolved_samples, ignored_lines, unmapped_clock_sync_lines


def collect_pair_offsets(
  samples: list[ClockSyncSample],
) -> dict[tuple[int, int], list[ClockSyncSample]]:
  pair_samples: dict[tuple[int, int], list[ClockSyncSample]] = {}

  for sample in samples:
    # Convention: offset reported by federate b belongs to pair (b-1) -> b.
    if sample.federate_id <= 0:
      continue

    pair = (sample.federate_id - 1, sample.federate_id)
    pair_samples.setdefault(pair, []).append(sample)

  for pair in pair_samples:
    pair_samples[pair].sort(
      key=lambda s: (s.wall_time_s if s.wall_time_s is not None else float("inf"), s.line_no)
    )

  return pair_samples


def x_value(sample: ClockSyncSample, x_axis: str, first_wall_time: float | None) -> float:
  if x_axis == "line":
    return float(sample.line_no)
  if x_axis == "time":
    if sample.wall_time_s is None or first_wall_time is None:
      return float(sample.line_no)
    return sample.wall_time_s - first_wall_time
  raise ValueError(f"Unsupported x-axis: {x_axis}")


def plot_pair_offsets(
  pair_samples: dict[tuple[int, int], list[ClockSyncSample]],
  x_axis: str,
  show_zoom: bool,
  svg_output_path: Path | None,
) -> None:
  try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
  except ImportError:
    print("matplotlib is not installed. Skipping plot.")
    return

  if not pair_samples:
    print("No pair-wise CLOCK_SYNC offsets available to plot.")
    return

  all_samples = [sample for samples in pair_samples.values() for sample in samples]
  first_wall_time = next((s.wall_time_s for s in all_samples if s.wall_time_s is not None), None)

  fig, axis = plt.subplots(figsize=(11, 5))

  sorted_pairs = sorted(pair_samples.keys())
  plotted_series: list[tuple[list[float], list[float], str, str | None]] = []

  for src_fed_id, dst_fed_id in sorted_pairs:
    samples = pair_samples[(src_fed_id, dst_fed_id)]
    xs = [x_value(sample, x_axis, first_wall_time) for sample in samples]
    ys = [sample.offset_ms for sample in samples]
    line_label = f"{src_fed_id}->{dst_fed_id}"
    axis.plot(xs, ys, linewidth=1.5, label=line_label)
    plotted_series.append((xs, ys, line_label, None))

  # Total cumulative error: pointwise sum across all neighboring pair series.
  min_len = min(len(pair_samples[pair]) for pair in sorted_pairs)
  if min_len > 0:
    first_pair = sorted_pairs[0]
    last_pair = sorted_pairs[-1]

    reference_samples = pair_samples[first_pair][:min_len]
    total_x = [x_value(sample, x_axis, first_wall_time) for sample in reference_samples]
    total_y = [
      sum(pair_samples[pair][idx].offset_ms for pair in sorted_pairs)
      for idx in range(min_len)
    ]

    axis.plot(
      total_x,
      total_y,
      color="black",
      linewidth=1.0,
      label=f"{first_pair[0]}->{last_pair[1]} (total)",
      linestyle="dashed",
    )
    plotted_series.append(
      (total_x, total_y, f"{first_pair[0]}->{last_pair[1]} (total)", "black")
    )

  if show_zoom:
    # Add a zoom inset focused on the middle region of the data.
    all_x = [x for xs, _, _, _ in plotted_series for x in xs]
    all_y = [y for _, ys, _, _ in plotted_series for y in ys]
    if all_x and all_y:
      x_min, x_max = min(all_x), max(all_x)
      y_min, y_max = min(all_y), max(all_y)

      x_span = x_max - x_min
      y_span = y_max - y_min

      if x_span > 0 and y_span > 0:
        x_center = x_min + 0.5 * x_span
        y_center = y_min + 0.5 * y_span

        x_half = 0.06 * x_span
        y_half = 0.036 * y_span

        inset = inset_axes(
          axis,
          width="29%",
          height="29%",
          loc="center",
          bbox_to_anchor=(+0.20, -0.25, 1.0, 1.0),
          bbox_transform=axis.transAxes,
          borderpad=0.8,
        )
        for xs, ys, _, color in plotted_series:
          kwargs = {"linewidth": 1.0}
          if color is not None:
            kwargs["color"] = color
            kwargs["linestyle"] = "dashed"
          inset.plot(xs, ys, **kwargs)

        inset.set_xlim(x_center - x_half, x_center + x_half)
        inset.set_ylim(y_center - y_half, y_center + y_half)
        inset.grid(True, linestyle=":", alpha=0.4)
        inset.set_title("Zoom", fontsize=9)
        inset.tick_params(labelsize=8)

        # Draw connectors and the source rectangle on the main axis.
        mark_inset(axis, inset, loc1=1, loc2=3, fc="none", ec="0.35", lw=0.9)

  axis.axhline(0.0, linestyle="--", linewidth=1.0, color="black")
  axis.grid(True, linestyle=":", alpha=0.5)
  axis.set_title("Clock sync error per neighboring pair + total")
  axis.set_ylabel("Offset (ms)")

  if x_axis == "time":
    axis.set_xlabel("Wall time since first sample (s)")
  else:
    axis.set_xlabel("Log line")

  axis.legend(title="Series")
  fig.tight_layout()

  if svg_output_path is not None:
    fig.savefig(svg_output_path, format="svg")
    print(f"Saved SVG plot to {svg_output_path}")
    plt.close(fig)
    return

  plt.show()


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Plot pair-wise clock sync error from [CLOCK_SYNC] offset output"
  )
  parser.add_argument(
    "log_file",
    type=str,
    help="Path to a log file containing CLOCK_SYNC output.",
  )
  parser.add_argument(
    "--x_axis",
    choices=["time", "line"],
    default="time",
    help="X-axis type: wall-time delta in seconds (time) or log line number (line).",
  )
  parser.add_argument(
    "--svg",
    action="store_true",
    help="Save plot as SVG next to the input log file and do not open an interactive window.",
  )
  parser.add_argument(
    "--zoom",
    action="store_true",
    help="Enable a small zoom inset (disabled by default).",
  )
  args = parser.parse_args()

  samples, ignored_lines, unmapped_lines = load_clock_sync_samples(args.log_file)
  print(
    f"Parsed {len(samples)} CLOCK_SYNC offset samples "
    f"({ignored_lines} non-matching lines ignored, {unmapped_lines} CLOCK_SYNC lines without federate mapping)."
  )

  pair_samples = collect_pair_offsets(samples)
  if not pair_samples:
    print("No neighboring federate pair offsets found.")
    return

  for pair in sorted(pair_samples.keys()):
    print(f"Pair {pair[0]}->{pair[1]}: {len(pair_samples[pair])} samples")

  plot_pair_offsets(
    pair_samples=pair_samples,
    x_axis=args.x_axis,
    show_zoom=args.zoom,
    svg_output_path=Path(args.log_file).with_suffix(".clock-sync.svg") if args.svg else None,
  )


if __name__ == "__main__":
  main()
