import argparse
import re
from dataclasses import dataclass
from pathlib import Path

# ------------------------- Data retrieval and parsing -------------------------

@dataclass(frozen=True)
class LogEvent:
  device_name: str
  federate_id: int
  value: int
  physical_time_ms: float
  logical_time_ms: int


@dataclass(frozen=True)
class ParsedLog:
  events: list[LogEvent]
  ignored_line_count: int


LOG_LINE_PATTERN = re.compile(
  r"^\[(?P<device_name>[^\]]+)\]\s+"
  r"\[(?P<federate_id>-?\d+)\]\s+"
  r"Value\s+(?P<value>-?\d+)\s+"
  r"at physical time\s+(?P<physical_time_ms>-?\d+(?:\.\d+)?)\s+ms\s+"
  r"and logical time\s+(?P<logical_time_ms>-?\d+)\s+ms\s*$"
)


def load_log_lines(log_file: str | Path) -> list[str]:
  path = Path(log_file)
  with path.open("r", encoding="utf-8") as f:
    return f.readlines()


def parse_log_line(line: str) -> LogEvent | None:
  match = LOG_LINE_PATTERN.match(line.strip())
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
  args = parser.parse_args()

  parsed_log = parse_log_file(args.log_file)

  mapping = build_mapping(parsed_log)

  delays = []
  for value in mapping.values_for_federate(0):
    delay = calculate_physical_delay(mapping, value, 0, 1) - args.logical_delay
    print(f"Delay for {0}->{1} with value {value}: {round(delay, 2)} ms")
    if delay is not None:
      delays.append(delay)

  print(f"Calculated delays: {delays}")

if __name__ == "__main__":
  main()