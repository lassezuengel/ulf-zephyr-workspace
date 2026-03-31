#!/usr/bin/env python3
"""
Reflash active IoT-LAB nodes using the deployment mapping created by
`deploy_experiment.py`.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Dict, List


DEFAULT_MAPPING_FILE = Path(__file__).with_name("last_deployment.json")


def run(args: List[str]) -> str:
    """Run a command and return stdout."""
    print("Running:", " ".join(args))
    result = subprocess.run(args, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def get_node_id(network_address: str, archi: str) -> str:
    match = re.search(rf"{re.escape(archi)}-(\d+)", network_address)
    if match is None:
        raise ValueError(f"Cannot parse network_address '{network_address}' for archi '{archi}'")
    return match.group(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reflash active IoT-LAB nodes from a saved deployment mapping."
    )
    parser.add_argument(
        "--mapping-file",
        type=Path,
        default=DEFAULT_MAPPING_FILE,
        help=f"Path to mapping JSON file (default: {DEFAULT_MAPPING_FILE}).",
    )
    parser.add_argument(
        "--site",
        default=None,
        help="Override site from mapping file.",
    )
    parser.add_argument(
        "--archi",
        default=None,
        help="Override architecture from mapping file.",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not reset nodes after reflashing.",
    )
    return parser.parse_args()


def load_mapping(mapping_file: Path) -> Dict[str, object]:
    mapping_path = mapping_file.expanduser()
    if not mapping_path.exists():
        print(f"Mapping file not found: {mapping_path}")
        print("Run deploy_experiment.py first to generate a mapping.")
        sys.exit(1)

    try:
        data = json.loads(mapping_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        print(f"Invalid JSON in mapping file {mapping_path}: {exc}")
        sys.exit(1)

    node_programs = data.get("node_programs")
    if not isinstance(node_programs, list) or not node_programs:
        print(f"Mapping file has no valid 'node_programs': {mapping_path}")
        sys.exit(1)

    return data


def main() -> None:
    args = parse_args()
    mapping = load_mapping(args.mapping_file)

    site = args.site or str(mapping.get("site", ""))
    archi = args.archi or str(mapping.get("archi", ""))
    if not site or not archi:
        print("Site or architecture missing from mapping. Use --site/--archi.")
        sys.exit(1)

    mapped_programs = mapping["node_programs"]
    assert isinstance(mapped_programs, list)

    firmware_by_node_id: Dict[str, str] = {}
    for entry in mapped_programs:
        if not isinstance(entry, dict):
            continue
        node_id = str(entry.get("node_id", "")).strip()
        firmware = str(entry.get("firmware", "")).strip()
        if not node_id or not firmware:
            continue
        firmware_by_node_id[node_id] = firmware

    if not firmware_by_node_id:
        print("No valid node/firmware entries in mapping file.")
        sys.exit(1)

    # Query currently active nodes in the current experiment context.
    get_out = run(["iotlab-experiment", "get", "-n"])
    nodes = json.loads(get_out).get("items", [])

    active_node_ids: List[str] = []
    for node in nodes:
        network_address = node.get("network_address", "")
        if not network_address:
            continue
        try:
            node_id = get_node_id(network_address, archi)
        except ValueError:
            continue
        active_node_ids.append(node_id)

    active_node_ids = sorted(set(active_node_ids))
    if not active_node_ids:
        print("No active nodes found for the selected architecture.")
        sys.exit(1)

    reflashed_node_ids: List[str] = []
    for node_id in active_node_ids:
        firmware = firmware_by_node_id.get(node_id)
        if firmware is None:
            continue

        firmware_path = Path(firmware).expanduser()
        if not firmware_path.exists():
            print(f"Firmware not found for node {node_id}: {firmware_path}")
            sys.exit(1)

        print(f"Reflashing node {node_id} with {firmware_path}")
        run(
            [
                "iotlab-node",
                "--flash",
                str(firmware_path),
                "-l",
                f"{site},{archi},{node_id}",
            ]
        )
        reflashed_node_ids.append(node_id)

    if not reflashed_node_ids:
        print("No active nodes matched entries in the mapping file.")
        sys.exit(1)

    print(f"Reflashed {len(reflashed_node_ids)} node(s): {reflashed_node_ids}")

    if not args.no_reset:
        run(
            [
                "iotlab-node",
                "--reset"
            ]
        )


if __name__ == "__main__":
    main()
