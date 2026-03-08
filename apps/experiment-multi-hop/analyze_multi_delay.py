import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path

# ------------------------- Data retrieval and parsing -------------------------

@dataclass(frozen=True)
class LogEvent:
  device_name: str | None
  federate_id: int
  value: int
  physical_time_ms: float
  logical_time_ms: int


@dataclass(frozen=True)
class ParsedLog:
  events: list[LogEvent]
  ignored_line_count: int


LOG_LINE_PATTERN = re.compile(
  r"^(?:\[(?P<device_name>[^\]]+)\]\s+)?"
  r"\[(?P<federate_id>-?\d+)\]\s+"
  r"Value\s+(?P<value>-?\d+)\s+"
  r"at physical time\s+(?P<physical_time_ms>-?\d+(?:\.\d+)?)\s+ms\s+"
  r"and logical time\s+(?P<logical_time_ms>-?\d+)\s+ms\s*$"
)


def normalize_log_line(raw_line: str) -> str:
  # iot-lab serial aggregation can prepend metadata ending in ';'.
  # Keep only the trailing segment that contains the actual log line.
  stripped = raw_line.strip()
  if ";" in stripped:
    stripped = stripped.rsplit(";", maxsplit=1)[-1].lstrip()
  return stripped


def load_log_lines(log_file: str | Path) -> list[str]:
  path = Path(log_file)
  with path.open("r", encoding="utf-8") as f:
    return f.readlines()


def parse_log_line(line: str) -> LogEvent | None:
  normalized_line = normalize_log_line(line)
  match = LOG_LINE_PATTERN.match(normalized_line)
  if not match:
    return None

  groups = match.groupdict()
  return LogEvent(
    device_name=groups["device_name"],
    federate_id=int(groups["federate_id"]),
    value=int(groups["value"]),
    physical_time_ms=float(groups["physical_time_ms"]),
    logical_time_ms=int(groups["logical_time_ms"]),
  )


def parse_log_file(log_file: str | Path) -> ParsedLog:
  events: list[LogEvent] = []
  ignored_line_count = 0

  for line in load_log_lines(log_file):
    event = parse_log_line(line)
    if event is None:
      ignored_line_count += 1
      continue
    events.append(event)

  return ParsedLog(events=events, ignored_line_count=ignored_line_count)


# ---------------------------- Data interpretation -----------------------------

@dataclass
class EventTimeMap:
  _by_federate: dict[int, dict[int, float]]

  @classmethod
  def empty(cls) -> "EventTimeMap":
    return cls(_by_federate={})

  def record(self, event: LogEvent) -> None:
    if event.federate_id not in self._by_federate:
      self._by_federate[event.federate_id] = {}
    self._by_federate[event.federate_id][event.value] = event.physical_time_ms

  def get_time(self, federate_id: int, value: int) -> float | None:
    fed_map = self._by_federate.get(federate_id)
    if fed_map is None:
      return None
    return fed_map.get(value)

  def require_time(self, federate_id: int, value: int) -> float:
    physical_time = self.get_time(federate_id, value)
    if physical_time is None:
      raise KeyError(f"No event for federate_id={federate_id}, value={value}")
    return physical_time

  def has_value(self, federate_id: int, value: int) -> bool:
    return self.get_time(federate_id, value) is not None

  def federate_ids(self) -> list[int]:
    return sorted(self._by_federate.keys())

  def values_for_federate(self, federate_id: int) -> list[int]:
    fed_map = self._by_federate.get(federate_id)
    if fed_map is None:
      return []
    return sorted(fed_map.keys())

  def items_for_federate(self, federate_id: int) -> list[tuple[int, float]]:
    fed_map = self._by_federate.get(federate_id)
    if fed_map is None:
      return []
    return sorted(fed_map.items())

  def as_nested_dict(self) -> dict[int, dict[int, float]]:
    return {fed_id: dict(value_map) for fed_id, value_map in self._by_federate.items()}


# Builds a map from federate ID and event value to physical time,
# while hiding the nested-dict storage details.
def build_mapping(parsed_log: ParsedLog) -> EventTimeMap:
  mapping = EventTimeMap.empty()
  for event in parsed_log.events:
    mapping.record(event)
  return mapping

def calculate_physical_delay(mapping: EventTimeMap, value: int, federate_id1: int, federate_id2: int) -> float | None:
  time_fed1 = mapping.get_time(federate_id=federate_id1, value=value)
  time_fed2 = mapping.get_time(federate_id=federate_id2, value=value)
  if time_fed1 is None or time_fed2 is None:
    return None
  return time_fed2 - time_fed1


def collect_neighbor_pair_delays(
  mapping: EventTimeMap,
  max_fed_id: int,
  logical_delay: int,
) -> dict[tuple[int, int], list[tuple[int, float]]]:
  pair_delays: dict[tuple[int, int], list[tuple[int, float]]] = {}

  for src_fed_id in range(max_fed_id):
    dst_fed_id = src_fed_id + 1
    pair = (src_fed_id, dst_fed_id)
    pair_delays[pair] = []

    for value in mapping.values_for_federate(src_fed_id):
      delay = calculate_physical_delay(mapping, value, src_fed_id, dst_fed_id)
      if delay is None:
        continue
      pair_delays[pair].append((value, delay - logical_delay))

  return pair_delays


def collect_total_accumulated_delays(
  pair_delays: dict[tuple[int, int], list[tuple[int, float]]],
) -> list[tuple[int, float]]:
  if not pair_delays:
    return []

  delay_maps = [dict(samples) for _, samples in sorted(pair_delays.items())]
  if not delay_maps:
    return []

  common_values = set(delay_maps[0].keys())
  for delay_map in delay_maps[1:]:
    common_values &= set(delay_map.keys())

  total_delays: list[tuple[int, float]] = []
  for value in sorted(common_values):
    total_delay = sum(delay_map[value] for delay_map in delay_maps)
    total_delays.append((value, total_delay))

  return total_delays


def collect_cumulative_delay_statistics_per_node(
  pair_delays: dict[tuple[int, int], list[tuple[int, float]]],
) -> tuple[list[int], list[float], list[float], list[float]]:
  if not pair_delays:
    return [], [], [], []

  first_fed_id = min(src_fed_id for src_fed_id, _ in pair_delays.keys())
  last_fed_id = max(dst_fed_id for _, dst_fed_id in pair_delays.keys())

  pair_delay_maps = {pair: dict(samples) for pair, samples in pair_delays.items()}

  node_ids = list(range(first_fed_id, last_fed_id + 1))
  avg_cumulative_delays: list[float] = [0.0]
  min_cumulative_delays: list[float] = [0.0]
  max_cumulative_delays: list[float] = [0.0]

  traversed_pair_delay_maps: list[dict[int, float]] = []
  common_values_so_far: set[int] | None = None

  for dst_fed_id in range(first_fed_id + 1, last_fed_id + 1):
    pair = (dst_fed_id - 1, dst_fed_id)
    pair_delay_map = pair_delay_maps.get(pair, {})
    traversed_pair_delay_maps.append(pair_delay_map)

    current_values = set(pair_delay_map.keys())
    if common_values_so_far is None:
      common_values_so_far = current_values
    else:
      common_values_so_far &= current_values

    if not common_values_so_far:
      avg_cumulative_delays.append(float("nan"))
      min_cumulative_delays.append(float("nan"))
      max_cumulative_delays.append(float("nan"))
      continue

    cumulative_samples = [
      sum(delay_map[value] for delay_map in traversed_pair_delay_maps)
      for value in sorted(common_values_so_far)
    ]

    avg_cumulative_delays.append(sum(cumulative_samples) / len(cumulative_samples))
    min_cumulative_delays.append(min(cumulative_samples))
    max_cumulative_delays.append(max(cumulative_samples))

  return node_ids, avg_cumulative_delays, min_cumulative_delays, max_cumulative_delays

# ------------------------- Plotting and Visualization -------------------------

def plot_neighbor_pair_delays(
  pair_delays: dict[tuple[int, int], list[tuple[int, float]]],
  logical_delay: int,
  total_only: bool = False,
  svg_output_path: Path | None = None,
) -> None:
  try:
    import matplotlib.pyplot as plt
  except ImportError:
    print("matplotlib is not installed. Skipping plot.")
    return

  if not pair_delays:
    print("No neighboring pairs available to plot.")
    return

  fig, (axis, summary_axis) = plt.subplots(
    2,
    1,
    figsize=(11, 9),
    gridspec_kw={"height_ratios": [2, 1]},
  )

  plotted_series_count = 0
  total_delays = collect_total_accumulated_delays(pair_delays)
  first_fed_id = min(src for src, _ in pair_delays.keys())
  last_fed_id = max(dst for _, dst in pair_delays.keys())

  if total_only:
    if total_delays:
      values = [sample[0] for sample in total_delays]
      delays = [sample[1] for sample in total_delays]
      axis.plot(
        values,
        delays,
        marker="o",
        linewidth=3.0,
        color="black",
        label=f"{first_fed_id}->{last_fed_id} (total)",
      )
      plotted_series_count = 1
    else:
      print("No common values across all neighboring pairs for total accumulated plot.")
  else:
    pair_items = sorted(pair_delays.items())
    for pair, samples in pair_items:
      src_fed_id, dst_fed_id = pair

      if samples:
        values = [sample[0] for sample in samples]
        delays = [sample[1] for sample in samples]
        axis.plot(
          values,
          delays,
          marker="o",
          linewidth=1.5,
          label=f"{src_fed_id}->{dst_fed_id}",
        )
        plotted_series_count += 1
      else:
        print(f"Skipping pair {src_fed_id}->{dst_fed_id} in plot (no shared values).")

    if total_delays:
      total_values = [sample[0] for sample in total_delays]
      total_delay_values = [sample[1] for sample in total_delays]
      axis.plot(
        total_values,
        total_delay_values,
        marker="o",
        linewidth=3.0,
        color="black",
        label=f"{first_fed_id}->{last_fed_id} (total)",
      )
      plotted_series_count += 1
    else:
      print("No common values across all neighboring pairs for total accumulated overlay.")

  axis.axhline(0.0, linestyle="--", linewidth=1.0, color="black")
  axis.set_title(
    "Estimated total error" if total_only else "Estimated error per neighboring pair + total"
  )
  axis.set_xlabel("Message #")
  axis.set_ylabel("Delta (ms)")
  axis.grid(True, linestyle=":", alpha=0.5)

  if plotted_series_count > 0:
    axis.legend(title="Series")

  node_ids, avg_cumulative, min_cumulative, max_cumulative = collect_cumulative_delay_statistics_per_node(pair_delays)
  if node_ids:
    summary_axis.plot(
      node_ids,
      avg_cumulative,
      marker="o",
      linewidth=1.8,
      color="tab:blue",
      label="Average clock difference",
    )
    summary_axis.fill_between(
      node_ids,
      min_cumulative,
      max_cumulative,
      color="gray",
      alpha=0.25,
      label="Min-max range",
    )

    valid_node_count = sum(1 for value in avg_cumulative if math.isfinite(value))
    if valid_node_count <= 1:
      print("Not enough shared values across hops to compute cumulative delay statistics beyond the first node.")

    first_node = min(node_ids)
    summary_axis.set_xlim(first_node, max(node_ids))
    summary_axis.set_xticks(node_ids)
    summary_axis.set_title(f"Approximate reaction timing error (delta) between node {first_node} and node b")
    summary_axis.set_xlabel("Node b")
    summary_axis.set_ylabel("Delta (ms)")
    summary_axis.grid(True, linestyle=":", alpha=0.5)
    summary_axis.axhline(0.0, linestyle="--", linewidth=1.0, color="black")
    summary_axis.legend()
  else:
    summary_axis.set_title("Reaction timing error summary")
    summary_axis.set_xlabel("Node b")
    summary_axis.set_ylabel("Delta (ms)")
    summary_axis.grid(True, linestyle=":", alpha=0.5)

  fig.suptitle(f"Approx. reaction timing error (logical delay: {logical_delay} ms)")
  fig.tight_layout()
  if svg_output_path is not None:
    fig.savefig(svg_output_path, format="svg")
    print(f"Saved SVG plot to {svg_output_path}")
    plt.close(fig)
    return
  plt.show()


# ------------------------------------ Main ------------------------------------

def main():
  parser = argparse.ArgumentParser(
    description="Analyze multi-hop delay experiment results"
  )
  parser.add_argument(
    "log_file",
    type=str,
    help="Path to the log file containing the experiment results."
  )
  parser.add_argument(
    "--logical_delay",
    type=int,
    default=50,
    help="Logical delay of connection. Used to calculate the clock sync error."
  )
  parser.add_argument(
    "--total_only",
    action="store_true",
    help="Only show accumulated delay from node 0 to the last node (pointwise sum across neighboring pairs).",
  )
  parser.add_argument(
    "--svg",
    action="store_true",
    help="Save plot as SVG next to the input log file and do not open an interactive window.",
  )
  args = parser.parse_args()

  parsed_log = parse_log_file(args.log_file)

  mapping = build_mapping(parsed_log)

  federate_ids = mapping.federate_ids()
  if not federate_ids:
    print("No federate events found in the log.")
    return
  max_fed_id = max(federate_ids)

  # Assume federate IDs are contiguous starting from 0. Abort if any expected IDs are missing.
  for fed_id in range(max_fed_id + 1):
    if fed_id not in federate_ids:
      print(f"Error: No events found for federate ID {fed_id}.")
      return

  pair_delays = collect_neighbor_pair_delays(
    mapping=mapping,
    max_fed_id=max_fed_id,
    logical_delay=args.logical_delay,
  )

  if args.total_only:
    total_delays = collect_total_accumulated_delays(pair_delays)
    first_fed_id = 0
    last_fed_id = max_fed_id
    if not total_delays:
      print("No common values across all neighboring pairs for clock error output.")
    for value, delay in total_delays:
      print(f"Total delay for {first_fed_id}->{last_fed_id} with value {value}: {round(delay, 2)} ms")
  else:
    for (src_fed_id, dst_fed_id), samples in sorted(pair_delays.items()):
      if not samples:
        print(f"No shared values between federates {src_fed_id} and {dst_fed_id}.")
        continue
      for value, delay in samples:
        print(f"Delay for {src_fed_id}->{dst_fed_id} with value {value}: {round(delay, 2)} ms")

  plot_neighbor_pair_delays(
    pair_delays,
    logical_delay=args.logical_delay,
    total_only=args.total_only,
    svg_output_path=Path(args.log_file).with_suffix(".svg") if args.svg else None,
  )

if __name__ == "__main__":
  main()