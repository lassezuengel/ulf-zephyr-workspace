#!/usr/bin/env python3
"""
Deploy an IoT-LAB experiment and flash one firmware per reserved node.

This script stores the node-to-firmware mapping in a JSON file so that
active nodes can be reflashed later with reflash_active_nodes.py.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Dict, List


DEFAULT_MAPPING_FILE = Path(__file__).with_name("last_deployment.json")


def run(args: List[str]) -> str:
    """Run a command and return stdout."""
    print("Running:", " ".join(args))
    result = subprocess.run(args, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reserve IoT-LAB nodes and flash one ELF file per node."
    )
    parser.add_argument(
        "program_inputs",
        nargs="+",
        help=(
            "Program inputs: each item can be an ELF file or a directory. "
            "Directory inputs are expanded to all files contained in them."
        ),
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=20,
        help="Experiment duration in minutes (default: 20).",
    )
    parser.add_argument(
        "--site",
        default="saclay",
        help="IoT-LAB site to use (default: saclay).",
    )
    parser.add_argument(
        "--archi",
        default="nrf52840dk",
        help="Node architecture to reserve (default: nrf52840dk).",
    )
    parser.add_argument(
        "--name",
        default="multi_elf_experiment",
        help="Experiment name (default: multi_elf_experiment).",
    )
    parser.add_argument(
        "--mapping-file",
        type=Path,
        default=DEFAULT_MAPPING_FILE,
        help=(
            "Path where node-to-firmware mapping is written "
            f"(default: {DEFAULT_MAPPING_FILE})."
        ),
    )
    return parser.parse_args()


def validate_elf_paths(raw_paths: List[str]) -> List[Path]:
    elf_paths: List[Path] = []

    for raw_path in raw_paths:
        path = Path(raw_path).expanduser()
        if not path.exists():
            print(f"Input path does not exist: {path}")
            sys.exit(1)

        if path.is_dir():
            # Keep expansion deterministic for stable node/program mapping.
            directory_files = sorted(child for child in path.iterdir() if child.is_file())
            if not directory_files:
                print(f"Directory has no files: {path}")
                sys.exit(1)
            elf_paths.extend(directory_files)
            continue

        if not path.is_file():
            print(f"Not a file or directory: {path}")
            sys.exit(1)

        elf_paths.append(path)

    if not elf_paths:
        print("No program files found in the provided inputs.")
        sys.exit(1)

    for elf_path in elf_paths:
        if elf_path.suffix.lower() != ".elf":
            print(f"Expected a .elf file, got: {elf_path}")
            sys.exit(1)
    return elf_paths


def get_node_id(network_address: str, archi: str) -> str:
    match = re.search(rf"{re.escape(archi)}-(\d+)", network_address)
    if match is None:
        raise ValueError(f"Cannot parse network_address '{network_address}' for archi '{archi}'")
    return match.group(1)


def build_mapping_data(
    exp_id: int,
    exp_name: str,
    site: str,
    archi: str,
    selected_nodes: List[Dict[str, Any]],
    elf_paths: List[Path],
    node_ids: List[str],
) -> Dict[str, Any]:
    node_programs: List[Dict[str, str]] = []
    for elf_path, node_id, node in zip(elf_paths, node_ids, selected_nodes):
        node_programs.append(
            {
                "node_id": node_id,
                "network_address": node.get("network_address", ""),
                "firmware": str(elf_path.resolve()),
            }
        )

    return {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "experiment_id": exp_id,
        "experiment_name": exp_name,
        "site": site,
        "archi": archi,
        "node_programs": node_programs,
    }


def write_mapping(mapping_file: Path, mapping_data: Dict[str, Any]) -> None:
    mapping_file.parent.mkdir(parents=True, exist_ok=True)
    mapping_file.write_text(json.dumps(mapping_data, indent=2), encoding="utf-8")
    print(f"Saved deployment mapping to: {mapping_file}")


def main() -> None:
    args = parse_args()
    elf_paths = validate_elf_paths(args.program_inputs)
    node_count = len(elf_paths)

    # 1. Submit experiment sized to the number of provided ELF files.
    submit_out = run(
        [
            "iotlab-experiment",
            "submit",
            "-n",
            args.name,
            "-d",
            str(args.duration),
            "-l",
            f"{node_count},archi={args.archi}:multi+site={args.site}",
        ]
    )

    # JSON parse: submit output is { "id": #### }
    exp_id = json.loads(submit_out)["id"]
    print("Experiment ID:", exp_id)

    # 2. Wait until experiment is running.
    run(["iotlab-experiment", "wait"])

    # 3. Get node list in JSON.
    get_out = run(["iotlab-experiment", "get", "-n"])
    nodes = json.loads(get_out)["items"]

    if len(nodes) < node_count:
        print(f"Expected at least {node_count} nodes, got: {len(nodes)}")
        sys.exit(1)

    # Keep deterministic order so ELF-to-node mapping is reproducible.
    nodes_sorted = sorted(nodes, key=lambda node: node.get("network_address", ""))
    selected_nodes = nodes_sorted[:node_count]

    # 4. Extract proper node IDs.
    node_ids: List[str] = []
    for node in selected_nodes:
        network_address = node.get("network_address", "")
        try:
            node_ids.append(get_node_id(network_address, args.archi))
        except ValueError as exc:
            print(exc)
            sys.exit(1)

    print("Nodes identified:", node_ids)

    mapping_data = build_mapping_data(
        exp_id=exp_id,
        exp_name=args.name,
        site=args.site,
        archi=args.archi,
        selected_nodes=selected_nodes,
        elf_paths=elf_paths,
        node_ids=node_ids,
    )
    write_mapping(args.mapping_file.expanduser(), mapping_data)

    # 5. Flash each node with corresponding ELF file.
    for index, (elf_path, node_id) in enumerate(zip(elf_paths, node_ids), start=1):
        flash_cmd = [
            "iotlab-node",
            "--flash",
            str(elf_path),
            "-l",
            f"{args.site},{args.archi},{node_id}",
        ]
        print(f"Flashing node {index}/{node_count}: {elf_path} -> {node_id}")
        run(flash_cmd)

    print("All nodes flashed successfully!")

    # 6. Reset nodes used in this experiment.
    for node_id in node_ids:
        reset_cmd = [
            "iotlab-node",
            "--reset",
            "-l",
            f"{args.site},{args.archi},{node_id}",
        ]
        run(reset_cmd)


if __name__ == "__main__":
    main()
