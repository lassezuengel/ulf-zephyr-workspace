#!/usr/bin/env python3
"""Upload generated federate ELF binaries to IoT-LAB frontend via scp.

Expected input layout:
  <input_dir>/
	<federate_name_1>/build/zephyr/zephyr.elf
	<federate_name_2>/build/zephyr/zephyr.elf
	...

Each discovered ELF is uploaded as:
  <remote_host>:~/<remote_dir>/<federate_name>.elf
"""

import argparse
from pathlib import Path
import subprocess
import sys
from typing import List, Tuple


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description=(
			"SCP each federate build/zephyr/zephyr.elf from an input directory "
			"to a remote directory, renamed to <federate-name>.elf."
		)
	)
	parser.add_argument(
		"input_dir",
		help="Directory containing one subdirectory per federate.",
	)
	parser.add_argument(
		"remote_dir",
		help="Remote directory under home, e.g. 'mydir' -> ~/mydir.",
	)
	parser.add_argument(
		"--host",
		default="saclay.iot-lab.info",
		help="Remote host (default: saclay.iot-lab.info).",
	)
	parser.add_argument(
		"--user",
		default="",
		help="Optional SSH username; if set, destination becomes user@host.",
	)
	parser.add_argument(
		"--dry-run",
		action="store_true",
		help="Print commands without executing them.",
	)
	return parser.parse_args()


def discover_federates(input_dir: Path) -> List[Tuple[str, Path]]:
	"""Return (federate_name, elf_path) pairs for valid subdirectories."""
	discovered: List[Tuple[str, Path]] = []

	for child in sorted(input_dir.iterdir()):
		if not child.is_dir():
			continue

		federate_name = child.name
		elf_path = child / "build" / "zephyr" / "zephyr.elf"
		if elf_path.is_file():
			discovered.append((federate_name, elf_path))

	return discovered


def run(cmd: List[str], dry_run: bool) -> None:
	print("$", " ".join(cmd))
	if dry_run:
		return
	subprocess.run(cmd, check=True)


def make_target_host(host: str, user: str) -> str:
	return f"{user}@{host}" if user else host


def main() -> None:
	args = parse_args()

	input_dir = Path(args.input_dir).expanduser().resolve()
	if not input_dir.exists() or not input_dir.is_dir():
		print(f"Input directory not found or not a directory: {input_dir}")
		sys.exit(1)

	federates = discover_federates(input_dir)
	if not federates:
		print(
			"No federates found. Expected entries like "
			"<input_dir>/<federate>/build/zephyr/zephyr.elf"
		)
		sys.exit(1)

	remote_dir = args.remote_dir.strip().strip("/")
	if not remote_dir:
		print("remote_dir must not be empty")
		sys.exit(1)

	target_host = make_target_host(args.host, args.user)

	# Ensure remote destination directory exists before uploading.
	# run(["ssh", target_host, "mkdir", "-p", f"~/{remote_dir}"], args.dry_run)

	for federate_name, elf_path in federates:
		remote_target = f"{target_host}:~/{remote_dir}/{federate_name}.elf"
		run(["scp", str(elf_path), remote_target], args.dry_run)

	print(f"Uploaded {len(federates)} federate ELF file(s).")


if __name__ == "__main__":
	main()
