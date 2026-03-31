#!/usr/bin/env python3
"""
Generates, then builds one Lingua Franca federation.

Flow:
1) Runs $REACTOR_UC_PATH/lfc/bin/lfc-dev -n <lf-file>
2) Runs tools/lf-federation-build.py <federation>

This little python script is functionaly equivalent to running
`REACTOR_UC_PATH/lfc/bin/lfc-dev <lf-file>`, but builds the federation
in a way that allows for incremental builds of the federates, by leaving
sharing the generated C code and build artifacts for reactor-uc and Zephyr
dependencies in place, and only rebuilding the federate-specific code for
each federate. This should improve build times for large federations with
many federates.
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


def log(msg: str) -> None:
    print(msg)


def run_cmd(cmd: list[str], dry_run: bool, cwd: Path) -> int:
    log(f"$ {' '.join(shlex.quote(c) for c in cmd)}")
    return subprocess.run(cmd, cwd=str(cwd)).returncode


def find_generated_federation_dir(lf_file: Path, workspace_root: Path) -> Path | None:
    federation_name = lf_file.stem

    if lf_file.parent.name == "src":
        lf_project_root = lf_file.parent.parent
    else:
        lf_project_root = lf_file.parent

    candidates = [
        lf_project_root / "src-gen" / federation_name,
        workspace_root / "src-gen" / federation_name,
    ]

    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate
    return None


def find_lf_project_root(lf_file: Path) -> Path:
    if lf_file.parent.name == "src":
        return lf_file.parent.parent
    return lf_file.parent


def clean_build_roots(lf_project_root: Path) -> None:
    # Always start from a clean generated tree and monobuild staging area.
    for path in (lf_project_root / "src-gen", lf_project_root / ".lf-monobuild"):
        if path.exists():
            log(f"Cleaning: {path}")
            shutil.rmtree(path)


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        log("Usage: lfc-mono.py <path/to/Federation.lf>")
        return 2

    script_dir = Path(__file__).resolve().parent
    workspace_root = script_dir.parent

    lf_arg = Path(argv[0])
    if lf_arg.is_absolute():
        lf_file = lf_arg.resolve()
    else:
        # Primary behavior: treat relative paths as local to current shell cwd.
        lf_from_cwd = (Path.cwd() / lf_arg).resolve()
        # Fallback: allow workspace-root relative paths for convenience.
        lf_from_workspace = (workspace_root / lf_arg).resolve()
        lf_file = lf_from_cwd if lf_from_cwd.exists() else lf_from_workspace

    if not lf_file.exists() or not lf_file.is_file():
        log(f"ERROR: LF file not found: {lf_file}")
        return 2

    reactor_uc_path = os.environ.get("REACTOR_UC_PATH")
    if not reactor_uc_path:
        log("ERROR: REACTOR_UC_PATH is not set")
        return 2

    lfc_dev = Path(reactor_uc_path).resolve() / "lfc" / "bin" / "lfc-dev"
    if not lfc_dev.exists() or not lfc_dev.is_file():
        log(f"ERROR: lfc-dev not found at: {lfc_dev}")
        return 2

    federation_name = lf_file.stem
    lf_project_root = find_lf_project_root(lf_file)

    build_script = script_dir / "lf-federation-build.py"
    if not build_script.exists() or not build_script.is_file():
        log(f"ERROR: build script not found: {build_script}")
        return 2

    log(f"LF file: {lf_file}")
    log(f"Federation: {federation_name}")

    clean_build_roots(lf_project_root)

    rc = run_cmd([str(lfc_dev), "-n", str(lf_file)], dry_run=False, cwd=workspace_root)
    if rc != 0:
        log(f"ERROR: lfc-dev failed (exit code {rc})")
        return 1

    federation_dir = find_generated_federation_dir(lf_file=lf_file, workspace_root=workspace_root)
    if federation_dir is None:
        log("ERROR: generated federation dir not found in expected locations:")
        log(f"  - {(lf_file.parent.parent if lf_file.parent.name == 'src' else lf_file.parent) / 'src-gen' / federation_name}")
        log(f"  - {workspace_root / 'src-gen' / federation_name}")
        return 1

    build_cmd: list[str] = [sys.executable, str(build_script), str(federation_dir)]

    rc = run_cmd(build_cmd, dry_run=False, cwd=workspace_root)
    if rc != 0:
        log(f"ERROR: federation build failed (exit code {rc})")
        return 1

    log("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
