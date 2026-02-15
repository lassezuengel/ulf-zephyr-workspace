#!/usr/bin/env python3

"""Build helper that compiles a Lingua Franca program and deploys binaries."""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


REMOTE_DEFAULT = "zungel@saclay.iot-lab.info:"
DEST_DIR = Path("build") / "programs"


class Colors:
    INFO = "\033[36m"
    SUCCESS = "\033[32m"
    WARN = "\033[33m"
    ERROR = "\033[31m"
    RESET = "\033[0m"


def log(color, message):
    print(f"{color}{message}{Colors.RESET}")


def run(command, **kwargs):
    """Run a command, surfacing stderr/stdout on failure."""
    cwd = kwargs.get("cwd")
    cwd_note = f" (cwd: {cwd})" if cwd else ""
    log(Colors.INFO, f"Running: {' '.join(map(str, command))}{cwd_note}")
    subprocess.run(command, check=True, **kwargs)


def clear_src_gen(root):
    src_gen = root / "src-gen"
    if src_gen.exists():
        log(Colors.INFO, f"Removing {src_gen}")
        shutil.rmtree(src_gen)
    else:
        log(Colors.WARN, f"No src-gen directory at {src_gen}, skipping")


def run_lfc(lf_path, root):
    lfc = Path.home() / "reactor-uc/lfc/bin/lfc-dev"
    if not lfc.is_file():
        raise FileNotFoundError(f"Missing compiler binary: {lfc}")
    log(Colors.INFO, f"Using compiler {lfc}")
    command = [str(lfc), str(lf_path)]
    run(command, cwd=root)


def find_federates(root, main_name):
    fed_root = root / "src-gen" / main_name
    if not fed_root.is_dir():
        raise FileNotFoundError(f"No generated output in {fed_root}")
    federates = sorted(p for p in fed_root.iterdir() if p.is_dir())
    if len(federates) != 2:
        raise RuntimeError(f"Expected 2 federates, found {len(federates)} in {fed_root}")
    log(Colors.INFO, f"Found federates: {', '.join(p.name for p in federates)}")
    return federates


def resolve_elf(federate_dir):
    zephyr_candidate = federate_dir / "build" / "zephyr" / "zephyr.elf"
    log(Colors.INFO, f"Looking for ELF in {zephyr_candidate}")
    if zephyr_candidate.is_file():
        return zephyr_candidate
    matches = sorted(federate_dir.glob("**/zephyr.elf"))
    if matches:
        log(Colors.INFO, f"Resolved ELF at {matches[0]}")
        return matches[0]
    raise FileNotFoundError(f"Missing zephyr.elf under {federate_dir / 'build'}")


def classify_federates(federates):
    mapping = {}
    for path in federates:
        name = path.name.lower()
        if any(tag in name for tag in ("client", "src", "send")) and "client" not in mapping:
            mapping["client"] = path
        elif any(tag in name for tag in ("server", "snk", "recv")) and "server" not in mapping:
            mapping["server"] = path
    remaining = [p for p in federates if p not in mapping.values()]
    if "client" not in mapping and remaining:
        mapping["client"] = remaining.pop(0)
    if "server" not in mapping and remaining:
        mapping["server"] = remaining.pop(0)
    if len(mapping) != 2:
        raise RuntimeError("Unable to classify federates as client/server")
    log(Colors.INFO, f"Client federate: {mapping['client'].name}, Server federate: {mapping['server'].name}")
    return mapping["client"], mapping["server"]


def stage_binaries(client_elf, server_elf, root):
    dest_dir = root / DEST_DIR
    dest_dir.mkdir(parents=True, exist_ok=True)
    client_dest = dest_dir / "echo_client.elf"
    server_dest = dest_dir / "echo_server.elf"
    log(Colors.INFO, f"Cloning stored ELF {client_elf} -> {client_dest}")
    shutil.copy2(client_elf, client_dest)
    log(Colors.INFO, f"Cloning stored ELF {server_elf} -> {server_dest}")
    shutil.copy2(server_elf, server_dest)
    log(Colors.SUCCESS, f"Staged client ELF at {client_dest}")
    log(Colors.SUCCESS, f"Staged server ELF at {server_dest}")
    return client_dest, server_dest


def scp_binaries(binaries, remote):
    command = ["scp", *map(str, binaries), remote]
    run(command)
    log(Colors.SUCCESS, f"Uploaded to {remote}")


def main():
    parser = argparse.ArgumentParser(description="Build and deploy LF federates")
    parser.add_argument("lf_file", type=Path, help="Path to .lf source file")
    parser.add_argument("--remote", default=REMOTE_DEFAULT, help="scp target (default: %(default)s)")
    args = parser.parse_args()

    lf_path = args.lf_file.resolve()
    if lf_path.suffix != ".lf" or not lf_path.is_file():
        parser.error("lf_file must point to an existing .lf file")

    root = Path(__file__).resolve().parent.parent
    log(Colors.INFO, f"Project root: {root}")

    try:
        clear_src_gen(root)
        run_lfc(lf_path, root)
        federates = find_federates(root, lf_path.stem)
        client_dir, server_dir = classify_federates(federates)
        client_elf = resolve_elf(client_dir)
        server_elf = resolve_elf(server_dir)
        staged = stage_binaries(client_elf, server_elf, root)
        scp_binaries(staged, args.remote)
        log(Colors.SUCCESS, "Done")
    except Exception as exc:
        log(Colors.ERROR, f"Error: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
