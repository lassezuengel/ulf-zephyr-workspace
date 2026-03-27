#!/usr/bin/env python3
"""Post-process Lingua Franca Zephyr federation output to de-duplicate shared assets.

This script is intended to run *after* LF code generation (src-gen update). It centralizes
runtime and config artifacts that are duplicated across federates, then wires each federate
back to the shared copies.

Default behavior:
- Discover federation directories under src-gen/.
- In each federation, detect federate directories by presence of CMakeLists.txt + Include.cmake.
- Share these artifacts from the first federate to <federation>/<shared-dir-name>/:
  - reactor-uc/
  - prj.conf
  - prj_lf.conf
- Replace each federate-local artifact with a symlink to the shared artifact.

Notes:
- This reduces duplicated source/config trees and keeps post-gen edits centralized.
- Zephyr will still create one build per federate image; this script does not merge images.
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
from typing import List, Sequence


SHARED_ARTIFACTS = ("reactor-uc", "prj.conf", "prj_lf.conf")
MONODIR_EXCLUDE_TOP = {"build"}
MONODIR_ARTIFACT_FILES = ("zephyr.elf", "zephyr.hex", "zephyr.bin", "zephyr.uf2")


def log(msg: str) -> None:
    print(msg)


def hash_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def list_files_recursive(root: Path) -> List[Path]:
    return sorted(p for p in root.rglob("*") if p.is_file())


def dirs_identical(a: Path, b: Path) -> bool:
    if not a.exists() or not b.exists() or not a.is_dir() or not b.is_dir():
        return False

    files_a = [p.relative_to(a) for p in list_files_recursive(a)]
    files_b = [p.relative_to(b) for p in list_files_recursive(b)]
    if files_a != files_b:
        return False

    for rel in files_a:
        if hash_file(a / rel) != hash_file(b / rel):
            return False
    return True


def files_identical(a: Path, b: Path) -> bool:
    if not a.exists() or not b.exists() or not a.is_file() or not b.is_file():
        return False
    return hash_file(a) == hash_file(b)


def ensure_parent(path: Path, dry_run: bool) -> None:
    if dry_run:
        return
    path.parent.mkdir(parents=True, exist_ok=True)


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


def copy_path(src: Path, dst: Path, dry_run: bool) -> None:
    if dry_run:
        log(f"[dry-run] copy {src} -> {dst}")
        return

    ensure_parent(dst, dry_run=False)
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=False)
    else:
        shutil.copy2(src, dst)


def ensure_symlink(link_path: Path, target_path: Path, dry_run: bool) -> None:
    rel_target = os.path.relpath(target_path, start=link_path.parent)

    if link_path.is_symlink():
        current = os.readlink(link_path)
        if current == rel_target:
            log(f"skip symlink (already correct): {link_path} -> {rel_target}")
            return

    if link_path.exists() or link_path.is_symlink():
        remove_path(link_path, dry_run=dry_run)

    if dry_run:
        log(f"[dry-run] symlink {link_path} -> {rel_target}")
        return

    ensure_parent(link_path, dry_run=False)
    link_path.symlink_to(rel_target)
    log(f"symlinked {link_path} -> {rel_target}")


def detect_federate_dirs(federation_dir: Path, shared_dir_name: str) -> List[Path]:
    federates: List[Path] = []
    for child in sorted(federation_dir.iterdir()):
        if not child.is_dir():
            continue
        if child.name in {shared_dir_name, "build"}:
            continue
        if (child / "CMakeLists.txt").exists() and (child / "Include.cmake").exists():
            federates.append(child)
    return federates


def compare_shared_artifact(artifact: str, first: Path, other: Path) -> bool:
    path_a = first / artifact
    path_b = other / artifact

    if artifact == "reactor-uc":
        return dirs_identical(path_a, path_b)
    return files_identical(path_a, path_b)


def compare_artifact_paths(artifact: str, a: Path, b: Path) -> bool:
    if artifact == "reactor-uc":
        return dirs_identical(a, b)
    return files_identical(a, b)


def prepare_shared_artifact(artifact: str, src: Path, shared_dst: Path, dry_run: bool) -> None:
    src_path = src / artifact
    if shared_dst.exists() or shared_dst.is_symlink():
        if compare_artifact_paths(artifact, src_path, shared_dst):
            return
        log(f"shared artifact out of date, refreshing: {shared_dst}")
        remove_path(shared_dst, dry_run=dry_run)
    copy_path(src_path, shared_dst, dry_run=dry_run)


def pick_runtime_seed(federates: Sequence[Path]) -> Path | None:
    candidates: List[Path] = []

    env_runtime = os.environ.get("REACTOR_UC_PATH")
    if env_runtime:
        candidates.append(Path(env_runtime))

    for federate in federates:
        local_runtime = federate / "reactor-uc"
        try:
            resolved = local_runtime.resolve(strict=True)
            candidates.append(resolved)
        except (FileNotFoundError, RuntimeError):
            continue

    for candidate in candidates:
        if candidate.exists() and candidate.is_dir() and (candidate / "CMakeLists.txt").exists():
            return candidate
    return None


def resolve_federate_dirs(
    federation_dir: Path,
    shared_dir_name: str,
    explicit_federates: Sequence[str] | None,
) -> List[Path]:
    if explicit_federates:
        return [federation_dir / name for name in explicit_federates]
    return detect_federate_dirs(federation_dir, shared_dir_name)


def path_is_symlink_to(path: Path, expected_target: str) -> bool:
    return path.is_symlink() and os.readlink(path) == expected_target


def copy_file_if_changed(src: Path, dst: Path, dry_run: bool) -> None:
    if dst.exists() and dst.is_file() and hash_file(src) == hash_file(dst):
        return
    if dry_run:
        log(f"[dry-run] copy file {src} -> {dst}")
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def sync_tree(src: Path, dst: Path, dry_run: bool, exclude_top: set[str]) -> None:
    if not src.exists() or not src.is_dir():
        raise ValueError(f"sync source directory does not exist: {src}")

    if not dry_run:
        dst.mkdir(parents=True, exist_ok=True)

    seen_rel: set[Path] = set()

    for entry in sorted(src.rglob("*")):
        rel = entry.relative_to(src)
        parts = rel.parts
        if parts and parts[0] in exclude_top:
            continue

        seen_rel.add(rel)
        dst_entry = dst / rel

        if entry.is_dir() and not entry.is_symlink():
            if dry_run:
                continue
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
        parts = rel.parts
        if parts and parts[0] in exclude_top:
            continue
        if rel in seen_rel:
            continue
        remove_path(existing, dry_run=dry_run)


def process_federation(
    federation_dir: Path,
    shared_dir_name: str,
    dry_run: bool,
    assume_equal_prj_conf: bool,
    assume_equal_prj_lf_conf: bool,
    explicit_federates: Sequence[str] | None,
) -> int:
    errors = 0
    shared_dir = federation_dir / shared_dir_name

    federates = resolve_federate_dirs(
        federation_dir=federation_dir,
        shared_dir_name=shared_dir_name,
        explicit_federates=explicit_federates,
    )

    missing = [f for f in federates if not f.exists()]
    for miss in missing:
        log(f"ERROR: federate directory does not exist: {miss}")
    if missing:
        return len(missing)

    if len(federates) < 2:
        log(f"skip {federation_dir}: need at least 2 federates, found {len(federates)}")
        return 0

    log(f"\n== Federation: {federation_dir}")
    log(f"Federates: {', '.join(f.name for f in federates)}")
    log(f"Shared dir: {shared_dir}")

    first = federates[0]

    # Recover from a stale/partial shared runtime by reseeding from a known-good runtime path.
    shared_runtime = shared_dir / "reactor-uc"
    if shared_runtime.exists() and not (shared_runtime / "CMakeLists.txt").exists():
        seed = pick_runtime_seed(federates)
        if seed is not None:
            log(f"shared runtime invalid, reseeding from: {seed}")
            remove_path(shared_runtime, dry_run=dry_run)
            copy_path(seed, shared_runtime, dry_run=dry_run)

    for artifact in SHARED_ARTIFACTS:
        for f in federates:
            candidate = f / artifact
            if not candidate.exists() and not candidate.is_symlink():
                log(f"ERROR: missing required artifact: {candidate}")
                errors += 1

    if errors:
        return errors

    for other in federates[1:]:
        for artifact in SHARED_ARTIFACTS:
            if artifact == "prj.conf" and assume_equal_prj_conf:
                continue
            if artifact == "prj_lf.conf" and assume_equal_prj_lf_conf:
                continue

            if not compare_shared_artifact(artifact, first, other):
                log(
                    "ERROR: artifact differs across federates: "
                    f"{artifact} ({first.name} vs {other.name})"
                )
                errors += 1

    if errors:
        log("aborting this federation due to mismatches")
        return errors

    if dry_run:
        log("[dry-run] would create shared directory and rewrite artifacts")
    else:
        shared_dir.mkdir(parents=True, exist_ok=True)

    for artifact in SHARED_ARTIFACTS:
        shared_dst = shared_dir / artifact
        prepare_shared_artifact(artifact, src=first, shared_dst=shared_dst, dry_run=dry_run)

    for federate in federates:
        for artifact in SHARED_ARTIFACTS:
            local_artifact = federate / artifact
            shared_artifact = shared_dir / artifact
            ensure_symlink(local_artifact, target_path=shared_artifact, dry_run=dry_run)

    return 0


def discover_federations(src_gen_dir: Path, shared_dir_name: str) -> List[Path]:
    out: List[Path] = []
    for child in sorted(src_gen_dir.iterdir()):
        if not child.is_dir():
            continue
        federates = detect_federate_dirs(child, shared_dir_name=shared_dir_name)
        if len(federates) >= 2:
            out.append(child)
    return out


def run_cmd(cmd: List[str], dry_run: bool, env: dict[str, str] | None = None) -> int:
    log(f"$ {' '.join(shlex.quote(c) for c in cmd)}")
    if dry_run:
        return 0
    completed = subprocess.run(cmd, env=env)
    return completed.returncode


def resolve_west_cmd() -> List[str] | None:
    west_path = shutil.which("west")
    if west_path is not None:
        return [west_path]

    probe = subprocess.run(
        [sys.executable, "-m", "west", "--version"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode == 0:
        return [sys.executable, "-m", "west"]

    return None


def build_federate(
    federate_dir: Path,
    west_cmd: Sequence[str],
    board: str | None,
    pristine: str,
    use_ccache: bool,
    dry_run: bool,
) -> int:
    build_dir = federate_dir / "build"
    cmake_cache = build_dir / "CMakeCache.txt"

    cmd: List[str] = [
        *west_cmd,
        "build",
        "-d",
        str(build_dir),
        "-s",
        str(federate_dir),
        "-p",
        pristine,
    ]

    if board:
        cmd.extend(["-b", board])

    cmake_args: List[str] = []
    if use_ccache and (dry_run or not cmake_cache.exists()):
        cmake_args.extend(
            [
                "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
                "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
            ]
        )

    if cmake_args:
        cmd.append("--")
        cmd.extend(cmake_args)

    log(f"\nBuilding federate: {federate_dir.name}")
    return run_cmd(cmd, dry_run=dry_run)


def copy_monodir_artifacts(
    staging_build_dir: Path,
    federation_dir: Path,
    federate_name: str,
    dry_run: bool,
) -> None:
    zephyr_out = staging_build_dir / "zephyr"
    artifact_dst = federation_dir / "artifacts" / federate_name

    for name in MONODIR_ARTIFACT_FILES:
        src = zephyr_out / name
        if not src.exists() or not src.is_file():
            continue
        dst = artifact_dst / name
        if dry_run:
            log(f"[dry-run] artifact copy {src} -> {dst}")
            continue
        artifact_dst.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def build_federation_monodir(
    federation_dir: Path,
    federates: Sequence[Path],
    staging_root: Path,
    west_cmd: Sequence[str],
    board: str | None,
    pristine: str,
    use_ccache: bool,
    dry_run: bool,
) -> int:
    errors = 0
    staging_src = staging_root / "app"
    staging_build = staging_root / "build"
    shared_dir = federation_dir / "shared"
    monodir_exclude_top = set(MONODIR_EXCLUDE_TOP)
    monodir_exclude_top.update(SHARED_ARTIFACTS)

    if dry_run:
        log(f"[dry-run] staging source dir: {staging_src}")
        log(f"[dry-run] staging build dir: {staging_build}")
    else:
        staging_root.mkdir(parents=True, exist_ok=True)
        staging_src.mkdir(parents=True, exist_ok=True)

    for artifact in SHARED_ARTIFACTS:
        staging_link = staging_src / artifact
        shared_target = shared_dir / artifact
        if dry_run and not shared_target.exists():
            log(f"[dry-run] shared artifact expected at {shared_target}")
        ensure_symlink(staging_link, target_path=shared_target, dry_run=dry_run)

    for index, federate in enumerate(federates):
        if not federate.exists() or not federate.is_dir():
            log(f"ERROR: federate directory does not exist: {federate}")
            errors += 1
            continue

        log(f"\n[monodir] Sync federate into staging app: {federate.name}")
        sync_tree(federate, staging_src, dry_run=dry_run, exclude_top=monodir_exclude_top)

        cmake_cache = staging_build / "CMakeCache.txt"
        build_ninja = staging_build / "build.ninja"
        if index == 0:
            first_pristine = pristine
            if not dry_run and not build_ninja.exists():
                first_pristine = "always"
        else:
            first_pristine = "never"

        cmd: List[str] = [
            *west_cmd,
            "build",
            "-d",
            str(staging_build),
            "-s",
            str(staging_src),
            "-p",
            first_pristine,
            "-c",
        ]

        if board:
            cmd.extend(["-b", board])

        cmake_args: List[str] = []
        if use_ccache and (dry_run or not cmake_cache.exists() or not build_ninja.exists()):
            cmake_args.extend(
                [
                    "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
                    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
                ]
            )
        if cmake_args:
            cmd.append("--")
            cmd.extend(cmake_args)

        log(f"\n[monodir] Build federate in shared build dir: {federate.name}")
        rc = run_cmd(cmd, dry_run=dry_run)
        if rc != 0:
            log(f"ERROR: monodir build failed for {federate.name} (exit code {rc})")
            errors += 1
            continue

        copy_monodir_artifacts(
            staging_build_dir=staging_build,
            federation_dir=federation_dir,
            federate_name=federate.name,
            dry_run=dry_run,
        )

    return errors


def build_federations(
    federation_dirs: Sequence[Path],
    src_gen_dir: Path,
    shared_dir_name: str,
    explicit_federates: Sequence[str] | None,
    board: str | None,
    pristine: str,
    use_ccache: bool,
    build_strategy: str,
    dry_run: bool,
) -> int:
    errors = 0
    west_cmd = resolve_west_cmd()
    if west_cmd is None:
        log("ERROR: west is not available (neither 'west' in PATH nor 'python -m west').")
        return 1

    if use_ccache and shutil.which("ccache") is None:
        log("WARNING: ccache not found in PATH; builds will proceed without compiler cache")

    for federation_dir in federation_dirs:
        federates = resolve_federate_dirs(
            federation_dir=federation_dir,
            shared_dir_name=shared_dir_name,
            explicit_federates=explicit_federates,
        )
        if not federates:
            log(f"skip build for {federation_dir}: no federates found")
            continue

        log(f"\n== Build federation: {federation_dir.name}")

        if build_strategy == "monodir":
            staging_root = src_gen_dir.parent / ".lf-monobuild" / federation_dir.name
            errors += build_federation_monodir(
                federation_dir=federation_dir,
                federates=federates,
                staging_root=staging_root,
                west_cmd=west_cmd,
                board=board,
                pristine=pristine,
                use_ccache=use_ccache,
                dry_run=dry_run,
            )
            continue

        for federate in federates:
            if not federate.exists() or not federate.is_dir():
                log(f"ERROR: federate directory does not exist: {federate}")
                errors += 1
                continue
            rc = build_federate(
                federate_dir=federate,
                west_cmd=west_cmd,
                board=board,
                pristine=pristine,
                use_ccache=use_ccache,
                dry_run=dry_run,
            )
            if rc != 0:
                log(f"ERROR: build failed for {federate} (exit code {rc})")
                errors += 1
    return errors


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Post-process LF Zephyr federations to share duplicated runtime/config artifacts"
    )
    parser.add_argument(
        "--src-gen",
        default="src-gen",
        help="Path to src-gen root (default: src-gen)",
    )
    parser.add_argument(
        "--federation",
        action="append",
        dest="federations",
        help="Federation directory name under src-gen; may be passed multiple times",
    )
    parser.add_argument(
        "--federate",
        action="append",
        dest="federates",
        help="Federate directory name(s) inside selected federation(s); default: auto-detect",
    )
    parser.add_argument(
        "--shared-dir-name",
        default="shared",
        help="Shared directory name inside each federation (default: shared)",
    )
    parser.add_argument(
        "--assume-equal-prj-conf",
        action="store_true",
        help="Skip equality checks for prj.conf across federates",
    )
    parser.add_argument(
        "--assume-equal-prj-lf-conf",
        action="store_true",
        help="Skip equality checks for prj_lf.conf across federates",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print actions without modifying files",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="After post-processing, build detected federates with west",
    )
    parser.add_argument(
        "--board",
        default=None,
        help="Optional board to pass to west build (default: omit and use app defaults)",
    )
    parser.add_argument(
        "--pristine",
        default="never",
        choices=["auto", "always", "never"],
        help="Pristine mode for west build (default: never)",
    )
    parser.add_argument(
        "--no-ccache",
        action="store_true",
        help="Disable ccache launchers when configuring build directories",
    )
    parser.add_argument(
        "--build-strategy",
        default="monodir",
        choices=["monodir", "separate"],
        help="Build strategy: monodir reuses one source/build dir per federation (default: monodir)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)

    src_gen_dir = Path(args.src_gen).resolve()
    if not src_gen_dir.exists() or not src_gen_dir.is_dir():
        log(f"ERROR: src-gen directory not found: {src_gen_dir}")
        return 2

    if args.federations:
        federation_dirs = [src_gen_dir / name for name in args.federations]
    else:
        federation_dirs = discover_federations(src_gen_dir, args.shared_dir_name)

    if not federation_dirs:
        log("No federation directories detected.")
        return 0

    total_errors = 0
    for federation_dir in federation_dirs:
        if not federation_dir.exists() or not federation_dir.is_dir():
            log(f"ERROR: federation directory not found: {federation_dir}")
            total_errors += 1
            continue

        total_errors += process_federation(
            federation_dir=federation_dir,
            shared_dir_name=args.shared_dir_name,
            dry_run=args.dry_run,
            assume_equal_prj_conf=args.assume_equal_prj_conf,
            assume_equal_prj_lf_conf=args.assume_equal_prj_lf_conf,
            explicit_federates=args.federates,
        )

    if total_errors == 0 and args.build:
        total_errors += build_federations(
            federation_dirs=federation_dirs,
            src_gen_dir=src_gen_dir,
            shared_dir_name=args.shared_dir_name,
            explicit_federates=args.federates,
            board=args.board,
            pristine=args.pristine,
            use_ccache=not args.no_ccache,
            build_strategy=args.build_strategy,
            dry_run=args.dry_run,
        )

    if total_errors:
        log(f"\nCompleted with {total_errors} error(s).")
        return 1

    log("\nPost-processing completed successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
