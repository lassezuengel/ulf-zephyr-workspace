#!/usr/bin/env python3
"""Upload generated federate ELF binaries to IoT-LAB frontend via scp.

Default input layout:
  <input_dir>/
	<federate_name_1>/build/zephyr/zephyr.elf
	<federate_name_2>/build/zephyr/zephyr.elf
	...

Artifacts mode layout (--artifacts <federation>):
	src-gen/<federation>/artifacts/
		<federate_name_1>/zephyr.elf
		<federate_name_2>/zephyr.elf
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
			"SCP each discovered federate zephyr.elf to a remote directory, "
			"renamed to <federate-name>.elf."
		)
	)
	parser.add_argument(
		"input_dir",
		nargs="?",
		help="Directory containing one subdirectory per federate.",
	)
	parser.add_argument(
		"--artifacts",
		default="",
		help=(
			"Use lfc-mono artifacts mode. Value can be a federation name "
			"(resolved as ./src-gen/<name>/artifacts) or a direct path to "
			"a federation dir or artifacts dir."
		),
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


def discover_artifact_federates(artifacts_dir: Path) -> List[Tuple[str, Path]]:
	"""Return (federate_name, elf_path) pairs from artifacts/<federate>/zephyr.elf."""
	discovered: List[Tuple[str, Path]] = []

	for child in sorted(artifacts_dir.iterdir()):
		if not child.is_dir():
			continue

		federate_name = child.name
		elf_path = child / "zephyr.elf"
		if elf_path.is_file():
			discovered.append((federate_name, elf_path))

	return discovered


def resolve_artifacts_dir(artifacts_arg: str, workspace_root: Path) -> Path:
	"""Resolve artifacts arg to an artifacts directory path."""
	given = Path(artifacts_arg).expanduser()
	candidates: List[Path] = []

	if given.is_absolute():
		candidates.extend([given, given / "artifacts"])
	else:
		cwd_given = (Path.cwd() / given).resolve()
		workspace_given = (workspace_root / given).resolve()
		workspace_src_gen = (workspace_root / "src-gen" / given / "artifacts").resolve()
		candidates.extend(
			[
				cwd_given,
				cwd_given / "artifacts",
				workspace_given,
				workspace_given / "artifacts",
				workspace_src_gen,
			]
		)

	for candidate in candidates:
		if not candidate.exists() or not candidate.is_dir():
			continue
		if candidate.name == "artifacts":
			return candidate
		artifacts_child = candidate / "artifacts"
		if artifacts_child.is_dir():
			return artifacts_child

	return (workspace_root / "src-gen" / artifacts_arg / "artifacts").resolve()


def run(cmd: List[str], dry_run: bool) -> None:
	print("$", " ".join(cmd))
	if dry_run:
		return
	subprocess.run(cmd, check=True)


def make_target_host(host: str, user: str) -> str:
	return f"{user}@{host}" if user else host


def main() -> None:
	args = parse_args()
	workspace_root = Path(__file__).resolve().parent.parent

	if args.artifacts:
		artifacts_dir = resolve_artifacts_dir(args.artifacts, workspace_root)
		if not artifacts_dir.exists() or not artifacts_dir.is_dir():
			print(f"Artifacts directory not found or not a directory: {artifacts_dir}")
			sys.exit(1)
		federates = discover_artifact_federates(artifacts_dir)
	else:
		if not args.input_dir:
			print("input_dir is required unless --artifacts is set")
			sys.exit(1)

		input_dir = Path(args.input_dir).expanduser().resolve()
		if not input_dir.exists() or not input_dir.is_dir():
			print(f"Input directory not found or not a directory: {input_dir}")
			sys.exit(1)

		federates = discover_federates(input_dir)
	if not federates:
		if args.artifacts:
			print(
				"No artifact federates found. Expected entries like "
				"<artifacts_dir>/<federate>/zephyr.elf"
			)
		else:
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
