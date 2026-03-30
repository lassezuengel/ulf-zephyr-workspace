#!/usr/bin/env python3
"""Build a single LF Zephyr federation quickly using one shared build directory.

- Input is one federation directory under src-gen (or an absolute path).
- Reuse one staging source/build dir for all federates.
- Build first federate with pristine configure, then incremental for the rest.

This gives monobuild semantics: reactor-uc and Zephyr dependencies compile once,
then each federate is rebuilt with minimal incremental work.

This approach assumes that all federates in the federation share the same reactor-uc
and the same Zephyr configuration (prj.conf and prj_lf.conf).
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Sequence


EXCLUDE_TOP = {"build"}
SHARED_INPUTS = ("reactor-uc", "prj.conf", "prj_lf.conf")
ARTIFACT_FILES = ("zephyr.elf", "zephyr.hex", "zephyr.bin", "zephyr.uf2")
GRAPH_INPUT_FILES = ("Include.cmake", "Kconfig")


def log(msg: str) -> None:
    print(msg)


def run_cmd(cmd: list[str], dry_run: bool) -> int:
    log(f"$ {' '.join(shlex.quote(c) for c in cmd)}")
    if dry_run:
        return 0
    return subprocess.run(cmd).returncode


def parse_module_paths(raw: str | None) -> list[Path]:
    if not raw:
        return []
    # Accept both CMake list ';' and shell path separators.
    normalized = raw.replace(";", os.pathsep)
    out: list[Path] = []
    for part in normalized.split(os.pathsep):
        part = part.strip()
        if not part:
            continue
        p = Path(part).expanduser().resolve()
        if p not in out:
            out.append(p)
    return out


def resolve_extra_zephyr_modules(workspace_root: Path) -> list[Path]:
    merged: list[Path] = []
    for var in ("EXTRA_ZEPHYR_MODULES", "ZEPHYR_EXTRA_MODULES"):
        for path in parse_module_paths(os.environ.get(var)):
            if path not in merged:
                merged.append(path)

    local_drivers = (workspace_root / "drivers").resolve()
    if local_drivers.exists() and local_drivers.is_dir() and local_drivers not in merged:
        merged.append(local_drivers)

    # Also include the repository-level drivers module where this script lives,
    # so app-local src-gen paths still pick up local Zephyr drivers/Kconfig.
    repo_root = Path(__file__).resolve().parent.parent
    repo_drivers = (repo_root / "drivers").resolve()
    if repo_drivers.exists() and repo_drivers.is_dir() and repo_drivers not in merged:
        merged.append(repo_drivers)

    return merged


def resolve_west_cmd() -> list[str] | None:
    west = shutil.which("west")
    if west:
        return [west]

    probe = subprocess.run(
        [sys.executable, "-m", "west", "--version"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode == 0:
        return [sys.executable, "-m", "west"]
    return None


def detect_federates(federation_dir: Path) -> list[Path]:
    out: list[Path] = []
    for child in sorted(federation_dir.iterdir()):
        if not child.is_dir():
            continue
        if child.name in {"build", "artifacts", "shared"}:
            continue
        if (child / "CMakeLists.txt").exists() and (child / "Include.cmake").exists():
            out.append(child)
    return out


def remove_path(path: Path, dry_run: bool) -> None:
    if not path.exists() and not path.is_symlink():
        return
    if dry_run:
        log(f"[dry-run] remove {path}")
        return
    if path.is_symlink() or path.is_file():
        path.unlink()
    else:
        shutil.rmtree(path)


def path_is_symlink_to(path: Path, expected_target: str) -> bool:
    return path.is_symlink() and os.readlink(path) == expected_target


def copy_file_if_changed(src: Path, dst: Path, dry_run: bool) -> None:
    if dst.exists() and dst.is_file():
        if src.stat().st_size == dst.stat().st_size and src.read_bytes() == dst.read_bytes():
            return
    if dry_run:
        log(f"[dry-run] copy file {src} -> {dst}")
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    # Use non-metadata-preserving copy so updated staged sources get a fresh
    # mtime and are reliably rebuilt by Ninja in incremental monobuild mode.
    shutil.copyfile(src, dst)


def sync_tree(src: Path, dst: Path, dry_run: bool, exclude_top: set[str] | None = None) -> None:
    if not src.exists() or not src.is_dir():
        raise ValueError(f"sync source does not exist: {src}")

    effective_exclude = EXCLUDE_TOP if exclude_top is None else exclude_top

    if not dry_run:
        dst.mkdir(parents=True, exist_ok=True)

    seen_rel: set[Path] = set()

    for entry in sorted(src.rglob("*")):
        rel = entry.relative_to(src)
        if rel.parts and rel.parts[0] in effective_exclude:
            continue

        seen_rel.add(rel)
        dst_entry = dst / rel

        if entry.is_dir() and not entry.is_symlink():
            if not dry_run:
                dst_entry.mkdir(parents=True, exist_ok=True)
            continue

        if entry.is_symlink():
            target = os.readlink(entry)
            if path_is_symlink_to(dst_entry, target):
                continue
            if dst_entry.exists() or dst_entry.is_symlink():
                remove_path(dst_entry, dry_run=dry_run)
            if dry_run:
                log(f"[dry-run] symlink {dst_entry} -> {target}")
                continue
            dst_entry.parent.mkdir(parents=True, exist_ok=True)
            dst_entry.symlink_to(target)
            continue

        if entry.is_file():
            copy_file_if_changed(entry, dst_entry, dry_run=dry_run)

    for existing in sorted(dst.rglob("*"), reverse=True):
        rel = existing.relative_to(dst)
        if rel.parts and rel.parts[0] in effective_exclude:
            continue
        if rel in seen_rel:
            continue
        remove_path(existing, dry_run=dry_run)


def stage_shared_inputs(seed_federate: Path, staging_src: Path, dry_run: bool) -> int:
    errors = 0
    for name in SHARED_INPUTS:
        src = seed_federate / name
        dst = staging_src / name

        if not src.exists() and not src.is_symlink():
            log(f"ERROR: missing shared input in seed federate: {src}")
            errors += 1
            continue

        if dst.exists() or dst.is_symlink():
            remove_path(dst, dry_run=dry_run)

        if dry_run:
            log(f"[dry-run] stage shared input {src} -> {dst}")
            continue

        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.is_dir():
            shutil.copytree(src, dst, dirs_exist_ok=False)
        else:
            shutil.copy2(src, dst)

    return errors


def copy_artifacts(staging_build_dir: Path, federation_dir: Path, federate_name: str, dry_run: bool) -> None:
    zephyr_out = staging_build_dir / "zephyr"
    dst_root = federation_dir / "artifacts" / federate_name
    for name in ARTIFACT_FILES:
        src = zephyr_out / name
        if not src.exists() or not src.is_file():
            continue
        dst = dst_root / name
        if dry_run:
            log(f"[dry-run] artifact copy {src} -> {dst}")
            continue
        dst_root.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def file_digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def normalized_include_cmake_digest(path: Path) -> str:
    # Ignore generated comments and whitespace-only noise so equivalent
    # federates (e.g., src_0/src_1/src_2) can reuse one CMake configure.
    normalized_lines: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        normalized_lines.append(line)
    payload = "\n".join(normalized_lines).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def compute_graph_signature(federate: Path) -> tuple[tuple[str, str], ...]:
    # Capture the subset of inputs that alter CMake/Ninja graph generation.
    parts: list[tuple[str, str]] = []
    for name in GRAPH_INPUT_FILES:
        p = federate / name
        if p.exists() and p.is_file():
            digest = normalized_include_cmake_digest(p) if name == "Include.cmake" else file_digest(p)
            parts.append((name, digest))
    return tuple(parts)


def resolve_federation_path(src_gen: Path, federation: str) -> Path:
    given = Path(federation)
    if given.is_absolute():
        return given.resolve()
    return (src_gen / federation).resolve()


def build_monodir(
    federation_dir: Path,
    src_gen: Path,
    board: str | None,
    first_pristine: str,
    use_ccache: bool,
    dry_run: bool,
) -> int:
    west_cmd: list[str] | None
    if dry_run:
        west_cmd = ["west"]
    else:
        west_cmd = resolve_west_cmd()
        if west_cmd is None:
            log("ERROR: west not found (neither 'west' nor 'python -m west').")
            return 1

    if use_ccache and not dry_run and shutil.which("ccache") is None:
        log("WARNING: ccache not found; continuing without compiler cache")

    extra_modules = resolve_extra_zephyr_modules(src_gen.parent)
    if extra_modules:
        log(f"Extra Zephyr modules: {', '.join(str(p) for p in extra_modules)}")

    federates = detect_federates(federation_dir)
    if len(federates) < 1:
        log(f"ERROR: no federates found in {federation_dir}")
        return 1

    log(f"Federation: {federation_dir}")
    log(f"Federates: {', '.join(f.name for f in federates)}")

    staging_root = src_gen.parent / ".lf-monobuild" / federation_dir.name
    staging_src = staging_root / "app"
    staging_build = staging_root / "build"

    if dry_run:
        log(f"[dry-run] staging source: {staging_src}")
        log(f"[dry-run] staging build:  {staging_build}")
    else:
        staging_src.mkdir(parents=True, exist_ok=True)
        staging_build.mkdir(parents=True, exist_ok=True)

    errors = 0
    # Pin shared Zephyr/runtime inputs from the first federate so expensive
    # dependency graphs are configured once and reused across federates.
    errors += stage_shared_inputs(
        seed_federate=federates[0],
        staging_src=staging_src,
        dry_run=dry_run,
    )
    if errors:
        return errors

    sync_exclude = set(EXCLUDE_TOP)
    sync_exclude.update(SHARED_INPUTS)
    previous_graph_signature: tuple[tuple[str, str], ...] | None = None

    for i, federate in enumerate(federates):
        log(f"\n[monobuild] sync {federate.name}")
        sync_tree(federate, staging_src, dry_run=dry_run, exclude_top=sync_exclude)

        pristine = first_pristine if i == 0 else "never"
        graph_signature = compute_graph_signature(federate)
        run_cmake = i == 0 or graph_signature != previous_graph_signature
        previous_graph_signature = graph_signature

        cmd: list[str] = [
            *west_cmd,
            "build",
            "-d",
            str(staging_build),
            "-s",
            str(staging_src),
            "-p",
            pristine,
        ]
        if run_cmake:
            cmd.append("-c")
        if board:
            cmd.extend(["-b", board])

        cmake_cache = staging_build / "CMakeCache.txt"
        cmake_args: list[str] = []
        if run_cmake and extra_modules:
            cmake_args.append(
                "-DEXTRA_ZEPHYR_MODULES=" + ";".join(str(p) for p in extra_modules)
            )
        if run_cmake and use_ccache and (dry_run or not cmake_cache.exists()):
            cmake_args.extend(
                [
                    "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
                    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
                ]
            )
        if cmake_args:
            cmd.extend(["--", *cmake_args])

        cmake_mode = "on" if run_cmake else "off"
        log(f"[monobuild] build {federate.name} (cmake: {cmake_mode})")
        rc = run_cmd(cmd, dry_run=dry_run)
        if rc != 0:
            log(f"ERROR: build failed for {federate.name} (exit code {rc})")
            errors += 1
            continue

        copy_artifacts(
            staging_build_dir=staging_build,
            federation_dir=federation_dir,
            federate_name=federate.name,
            dry_run=dry_run,
        )

    return errors


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fast monobuild for one LF Zephyr federation"
    )
    parser.add_argument(
        "federation",
        help="Federation name under src-gen (or absolute path to federation dir)",
    )
    parser.add_argument(
        "--src-gen",
        default="src-gen",
        help="Path to src-gen root (default: src-gen)",
    )
    parser.add_argument(
        "--board",
        default=None,
        help="Optional board passed to west build",
    )
    parser.add_argument(
        "--first-pristine",
        default="always",
        choices=["auto", "always", "never"],
        help="Pristine mode for first federate build (default: always)",
    )
    parser.add_argument(
        "--no-ccache",
        action="store_true",
        help="Disable ccache launcher flags",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print actions without building",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)

    given_federation = Path(args.federation)
    if given_federation.is_absolute():
        federation_dir = given_federation.resolve()
        # When federation is provided as an absolute path, infer src-gen from it
        # instead of requiring --src-gen in the current working directory.
        src_gen = federation_dir.parent
    else:
        src_gen = Path(args.src_gen).resolve()
        if not src_gen.exists() or not src_gen.is_dir():
            log(f"ERROR: src-gen not found: {src_gen}")
            return 2
        federation_dir = resolve_federation_path(src_gen, args.federation)

    if not federation_dir.exists() or not federation_dir.is_dir():
        log(f"ERROR: federation directory not found: {federation_dir}")
        return 2

    if not src_gen.exists() or not src_gen.is_dir():
        log(f"ERROR: src-gen not found: {src_gen}")
        return 2

    errors = build_monodir(
        federation_dir=federation_dir,
        src_gen=src_gen,
        board=args.board,
        first_pristine=args.first_pristine,
        use_ccache=not args.no_ccache,
        dry_run=args.dry_run,
    )

    if errors:
        log(f"\nCompleted with {errors} error(s).")
        return 1

    log("\nMonobuild completed successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
