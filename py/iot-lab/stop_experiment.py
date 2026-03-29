#!/usr/bin/env python3
"""
Stop an IoT-LAB experiment cleanly by:
1) flashing idle firmware globally,
2) stopping the running experiment,
3) deleting the deployment mapping file.
"""
from pathlib import Path
import re
import subprocess
import sys
from typing import List, Optional


DEFAULT_MAPPING_FILE = Path(__file__).with_name("last_deployment.json")
USE_COLOR = sys.stdout.isatty()

# Subtle palette: keep regular output neutral and errors in dark red.
RESET = "\033[0m" if USE_COLOR else ""
DIM = "\033[2m" if USE_COLOR else ""
DARK_RED = "\033[31m" if USE_COLOR else ""


def info(message: str) -> None:
    print(f"{DIM}{message}{RESET}")


def error(message: str) -> None:
    print(f"{DARK_RED}{message}{RESET}")


def extract_cli_error(stderr: str, stdout: str) -> Optional[str]:
    """Extract the most relevant CLI error message, if present."""
    for stream in (stderr, stdout):
        if not stream:
            continue
        matches = re.findall(r"error:\s*(.+)", stream, flags=re.IGNORECASE)
        if matches:
            return matches[-1].strip()
    return None


def run(args: List[str], allow_failure: bool = False) -> bool:
    """Run a command and return whether it succeeded."""
    info("Running: " + " ".join(args))
    try:
        result = subprocess.run(args, check=False, capture_output=True, text=True)
    except FileNotFoundError:
        error(f"Command not found: {args[0]}")
        return False

    if result.returncode == 0:
        return True

    info(f"Command failed with exit code {result.returncode}: {' '.join(args)}")
    stderr = result.stderr.strip()
    stdout = result.stdout.strip()
    cli_error = extract_cli_error(stderr, stdout)
    if cli_error:
        error(cli_error)
    else:
        error("Command failed (no detailed error message available).")

    if not allow_failure:
        return False
    return False


def main() -> None:
    mapping_path = DEFAULT_MAPPING_FILE.expanduser()
    had_failure = False

    # 1) Flash idle firmware on currently reserved nodes.
    flash_ok = run(["iotlab-node", "--flash-idle"], allow_failure=True)
    if not flash_ok:
        had_failure = True
        error("Continuing despite flash-idle failure.")

    # 2) Stop current experiment.
    stop_ok = run(["iotlab-experiment", "stop"], allow_failure=True)
    if not stop_ok:
        had_failure = True
        error("Continuing despite experiment stop failure.")

    # 3) Remove mapping file so stale mappings are not reused accidentally.
    if mapping_path.exists():
        mapping_path.unlink()
        info(f"Deleted mapping file: {mapping_path}")
    else:
        info(f"Mapping file not found, nothing to delete: {mapping_path}")

    if had_failure:
        sys.exit(1)


if __name__ == "__main__":
    main()
