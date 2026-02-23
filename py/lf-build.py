#!/usr/bin/env python3
"""
Lingua Franca Federate Build and Deploy Tool

This script compiles, builds, and transfers Lingua Franca federate programs.
It replaces makefile functionality with more flexible Python-based automation.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional
import re

class Colors:
    """ANSI color codes for terminal output"""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'

    # Foreground colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # Bright foreground colors
    BRIGHT_BLACK = '\033[90m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'

    @staticmethod
    def strip_if_no_tty():
        """Disable colors if output is not a TTY"""
        if not sys.stdout.isatty():
            for attr in dir(Colors):
                if not attr.startswith('_') and attr.isupper() and attr != 'strip_if_no_tty':
                    setattr(Colors, attr, '')

# Initialize color support
Colors.strip_if_no_tty()

class LFBuildError(Exception):
    """Custom exception for build errors"""
    pass


class LFBuilder:
    """Handles building and deploying Lingua Franca federate programs"""

    def __init__(self, source_file: str, remote_url: str = "hailo@hailo-desktop:~/lf", skip_ssh: bool = False):
        """
        Initialize the builder

        Args:
            source_file: Path to the .lf source file (relative to current directory)
            remote_url: SSH destination for file transfer (format: user@host:path)
            skip_ssh: If True, skip SSH operations
        """
        self.source_file = Path(source_file)
        self.remote_url = remote_url
        self.skip_ssh = skip_ssh
        self.cwd = Path.cwd()
        self.lfc_path = Path("~/reactor-uc/lfc/bin/lfc-dev").expanduser()

        # Get base name without extension for dynamic directory naming
        self.base_name = self.source_file.stem

        # Build directories (all relative to current directory)
        # Generated sources are in src-gen/<source_stem>, build output is <source_stem>_build
        self.src_gen_base_dir = self.cwd / "src-gen"
        self.src_gen_project_dir = self.src_gen_base_dir / self.base_name
        self.fed_build_dir = self.cwd / f"{self.base_name}_build"

        # Validate inputs
        self._validate_source_file()
        if not skip_ssh:
            self._validate_remote_url()

    def _validate_source_file(self) -> None:
        """Validate that the source file exists and has .lf extension"""
        if not self.source_file.suffix == '.lf':
            raise LFBuildError(
                f"Invalid file extension: {self.source_file.suffix}. "
                f"Expected .lf extension."
            )

        if not self.source_file.exists():
            raise LFBuildError(
                f"Source file not found: {self.source_file}"
            )

        if not self.source_file.is_file():
            raise LFBuildError(
                f"Source path is not a file: {self.source_file}"
            )

    def _validate_remote_url(self) -> None:
        """Validate the remote URL format"""
        # Basic validation for SSH URL format: user@host:path
        pattern = r'^[^@]+@[^:]+:.+$'
        if not re.match(pattern, self.remote_url):
            raise LFBuildError(
                f"Invalid remote URL format: {self.remote_url}. "
                f"Expected format: user@host:path"
            )

    def clean(self) -> None:
        """Remove previous build artifacts"""
        print(f"{Colors.CYAN}Cleaning previous builds for {Colors.BOLD}{self.base_name}{Colors.RESET}{Colors.CYAN}...{Colors.RESET}")

        dirs_to_clean = [self.fed_build_dir, self.src_gen_project_dir]

        for dir_path in dirs_to_clean:
            if dir_path.exists():
                print(f"{Colors.DIM}  Removing {dir_path.relative_to(self.cwd)}{Colors.RESET}")
                shutil.rmtree(dir_path)

        print(f"{Colors.GREEN}Clean complete{Colors.RESET}")

    def build(self) -> None:
        """Build the federate programs"""
        print(f"{Colors.CYAN}Building federate programs from {Colors.BOLD}{self.source_file}{Colors.RESET}{Colors.CYAN}...{Colors.RESET}")

        # Run lfc compiler
        try:
            result = subprocess.run(
                [str(self.lfc_path), str(self.source_file)],
                cwd=self.cwd,
                check=True,
                capture_output=True,
                text=True
            )
            print(f"{Colors.DIM}  LFC compilation successful{Colors.RESET}")
            if result.stdout:
                print(result.stdout)
        except subprocess.CalledProcessError as e:
            raise LFBuildError(
                f"LFC compilation failed:\n{e.stderr}"
            )
        except FileNotFoundError:
            raise LFBuildError(
                f"Compiler not found: {self.lfc_path}. Please ensure lfc-dev is installed."
            )

        # Find and copy all federate .elf files
        self._copy_federate_elfs()

        print(f"{Colors.GREEN}Build complete{Colors.RESET}")

    def _find_federate_elfs(self) -> List[Path]:
        """
        Find all federate zephyr.elf files in the generated directory

        Returns:
            List of paths to zephyr.elf files
        """
        if not self.src_gen_project_dir.exists():
            raise LFBuildError(
                f"Generated directory not found: {self.src_gen_project_dir}. "
                f"Build may have failed."
            )

        # Search for federate directories (federate__*)
        federate_elfs = []

        # Pattern: src-gen/<program>/<federate>/build/zephyr/zephyr.elf
        for federate_dir in self.src_gen_project_dir.iterdir():
            if not federate_dir.is_dir():
                continue

            elf_path = federate_dir / "build" / "zephyr" / "zephyr.elf"
            if elf_path.exists():
                federate_elfs.append((federate_dir.name, elf_path))

        if not federate_elfs:
            raise LFBuildError(
                "No federate .elf files found. Build may have failed or "
                "no federates were generated."
            )

        return federate_elfs

    def _copy_federate_elfs(self) -> None:
        """Copy all federate .elf files to the build directory"""
        # Create build directory
        self.fed_build_dir.mkdir(exist_ok=True)

        federate_elfs = self._find_federate_elfs()

        print(f"{Colors.DIM}  Found {len(federate_elfs)} federate(s):{Colors.RESET}")

        for federate_name, elf_path in federate_elfs:
            dest_path = self.fed_build_dir / f"{federate_name}.elf"
            shutil.copy2(elf_path, dest_path)
            print(f"{Colors.DIM}    {federate_name}.elf{Colors.RESET}")

    def ssh(self) -> None:
        """Transfer built files to remote server via SSH"""
        if self.skip_ssh:
            print(f"{Colors.YELLOW}SSH transfer skipped (--no-ssh mode){Colors.RESET}")
            return

        print(f"{Colors.CYAN}Transferring files to {Colors.BOLD}{self.remote_url}{Colors.RESET}{Colors.CYAN}...{Colors.RESET}")

        if not self.fed_build_dir.exists():
            raise LFBuildError(
                f"Build directory not found: {self.fed_build_dir}. "
                f"Run 'build' first."
            )

        # Find all .elf files in build directory
        elf_files = list(self.fed_build_dir.glob("*.elf"))

        if not elf_files:
            raise LFBuildError(
                f"No .elf files found in {self.fed_build_dir}"
            )

        # Transfer each file
        for elf_file in elf_files:
            print(f"{Colors.DIM}  Transferring {elf_file.name}...{Colors.RESET}", end=' ')
            try:
                subprocess.run(
                    ["scp", str(elf_file), self.remote_url],
                    check=True,
                    capture_output=True
                )
                print(f"{Colors.GREEN}done{Colors.RESET}")
            except subprocess.CalledProcessError as e:
                print(f"{Colors.RED}failed{Colors.RESET}")
                raise LFBuildError(
                    f"SCP transfer failed for {elf_file.name}:\n{e.stderr.decode()}"
                )
            except FileNotFoundError:
                print(f"{Colors.RED}failed{Colors.RESET}")
                raise LFBuildError(
                    "scp command not found. Please ensure SSH client is installed."
                )

        print(f"{Colors.GREEN}Transfer complete{Colors.RESET}")

    def all(self) -> None:
        """Run the complete pipeline: clean, build, and optionally ssh"""
        self.clean()
        self.build()
        self.ssh()


def main():
    parser = argparse.ArgumentParser(
        description="Build and deploy Lingua Franca federate programs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Build only (no SSH)
  %(prog)s build -s src/NrfDistribute.lf

  # Build and transfer with SSH
  %(prog)s all -s src/NrfDistribute.lf -r user@server:~/destination

  # Build without SSH transfer
  %(prog)s all -s src/NrfDistribute.lf --no-ssh

  # Just transfer existing build
  %(prog)s ssh -s src/NrfDistribute.lf -r dev@remote.server:~/lfprograms

  # Clean specific experiment
  %(prog)s clean -s experiments/MyExperiment.lf

Note:
  Build output directories are named based on the source file:
    src/NrfDistribute.lf → NrfDistribute_build/
    experiments/Test.lf  → Test_build/

    Generated sources are in src-gen/<program_name>/
        """
    )

    parser.add_argument(
        "command",
        choices=["all", "clean", "build", "ssh"],
        help="Command to execute"
    )

    parser.add_argument(
        "-s", "--source",
        default="src/NrfDistribute.lf",
        help="Path to the .lf source file (default: src/NrfDistribute.lf)"
    )

    parser.add_argument(
        "-r", "--remote-url",
        default="hailo@hailo-desktop:~/lf",
        help="SSH destination URL (default: hailo@hailo-desktop:~/lf)"
    )

    parser.add_argument(
        "--no-ssh",
        action="store_true",
        help="Skip SSH transfer (useful for 'all' command to do clean+build only)"
    )

    args = parser.parse_args()

    try:
        builder = LFBuilder(args.source, args.remote_url, skip_ssh=args.no_ssh)

        # Execute the requested command
        command_method = getattr(builder, args.command)
        command_method()

        print(f"\n{Colors.BOLD}{Colors.GREEN}Command '{args.command}' completed successfully{Colors.RESET}")
        return 0

    except LFBuildError as e:
        print(f"\n{Colors.RED}Error: {e}{Colors.RESET}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}Interrupted by user{Colors.RESET}", file=sys.stderr)
        return 130
    except Exception as e:
        print(f"\n{Colors.RED}Unexpected error: {e}{Colors.RESET}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())