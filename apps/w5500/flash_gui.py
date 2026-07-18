#!/usr/bin/env python3
"""Windows GUI for mapping UF2 images to LF W5500 federates and flashing them."""

from __future__ import annotations

import ctypes
import json
import os
import queue
import shutil
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    serial = None
    list_ports = None


BOOTSEL_VOLUME_LABEL = "RPI-RP2"
WINDOWS_BOOTSEL_TIMEOUT_SECONDS = 20.0
LINUX_BOOTSEL_TIMEOUT_SECONDS = 20.0


@dataclass(frozen=True)
class FlashJob:
    port: str
    firmware: Path


def settings_file() -> Path:
    if os.name == "nt":
        local_app_data = os.environ.get("LOCALAPPDATA")
        base = Path(local_app_data) if local_app_data else Path.home() / "AppData" / "Local"
        return base / "LF W5500 Flasher" / "settings.json"

    xdg_config_home = os.environ.get("XDG_CONFIG_HOME")
    base = Path(xdg_config_home) if xdg_config_home else Path.home() / ".config"
    return base / "lf-w5500-flasher" / "settings.json"


def port_identity(port_info: object) -> str:
    serial_number = getattr(port_info, "serial_number", None)
    vendor_id = getattr(port_info, "vid", None)
    product_id = getattr(port_info, "pid", None)
    if serial_number:
        vid = f"{vendor_id:04x}" if vendor_id is not None else "unknown"
        pid = f"{product_id:04x}" if product_id is not None else "unknown"
        return f"{vid}:{pid}:{serial_number}"
    return str(getattr(port_info, "device"))


def natural_port_key(port: str) -> tuple[str, int]:
    prefix = port.rstrip("0123456789")
    suffix = port[len(prefix) :]
    return prefix, int(suffix) if suffix else -1


def windows_bootsel_drives() -> set[str]:
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    get_logical_drives = kernel32.GetLogicalDrives
    get_logical_drives.restype = ctypes.c_uint32
    get_drive_type = kernel32.GetDriveTypeW
    get_drive_type.argtypes = [ctypes.c_wchar_p]
    get_drive_type.restype = ctypes.c_uint32
    get_volume_information = kernel32.GetVolumeInformationW
    get_volume_information.argtypes = [
        ctypes.c_wchar_p,
        ctypes.c_wchar_p,
        ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_wchar_p,
        ctypes.c_uint32,
    ]
    get_volume_information.restype = ctypes.c_int

    mask = get_logical_drives()
    result: set[str] = set()
    for index in range(26):
        if not mask & (1 << index):
            continue
        root = f"{chr(ord('A') + index)}:\\"
        if get_drive_type(root) != 2:  # DRIVE_REMOVABLE
            continue
        volume_name = ctypes.create_unicode_buffer(261)
        if get_volume_information(
            root, volume_name, len(volume_name), None, None, None, None, 0
        ) and volume_name.value.upper() == BOOTSEL_VOLUME_LABEL:
            result.add(root)
    return result


def unescape_linux_mount(value: str) -> str:
    return (
        value.replace("\\040", " ")
        .replace("\\011", "\t")
        .replace("\\012", "\n")
        .replace("\\134", "\\")
    )


def linux_mount_points() -> set[Path]:
    result: set[Path] = set()
    try:
        for line in Path("/proc/self/mounts").read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if len(fields) >= 3 and (
                fields[2]
                in {"vfat", "msdos", "exfat", "ntfs", "ntfs3", "9p", "drvfs"}
                or fields[2].startswith("fuse")
            ):
                result.add(Path(unescape_linux_mount(fields[1])))
    except OSError:
        pass
    return result


def linux_bootsel_mounts() -> set[str]:
    result: set[str] = set()
    for mount_point in linux_mount_points():
        try:
            if (mount_point / "INFO_UF2.TXT").is_file():
                result.add(str(mount_point))
        except OSError:
            # Disconnected USB filesystems can remain briefly in the mount table.
            continue
    return result


def bootsel_locations() -> set[str]:
    if os.name == "nt":
        return windows_bootsel_drives()
    if os.name == "posix":
        return linux_bootsel_mounts()
    return set()


def bootsel_timeout() -> float:
    return (
        WINDOWS_BOOTSEL_TIMEOUT_SECONDS
        if os.name == "nt"
        else LINUX_BOOTSEL_TIMEOUT_SECONDS
    )


def wait_for_new_bootsel_location(existing: set[str], timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        new_locations = bootsel_locations() - existing
        if len(new_locations) == 1:
            return new_locations.pop()
        if len(new_locations) > 1:
            raise RuntimeError(
                "multiple new RP2040 BOOTSEL volumes appeared; "
                "reconnect and flash one batch at a time"
            )
        time.sleep(0.2)
    mount_hint = (
        ""
        if os.name == "nt"
        else " Mount the BOOTSEL filesystem so INFO_UF2.TXT is visible to Linux."
    )
    raise TimeoutError(
        "RP2040 BOOTSEL volume did not appear; close serial monitors and try manual "
        f"BOOTSEL once.{mount_hint}"
    )


def wait_for_bootsel_to_disappear(location: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if location not in bootsel_locations():
            return
        time.sleep(0.2)
    raise TimeoutError(
        f"{location} did not reboot after the copy; the UF2 may not be valid for RP2040"
    )


class FlashGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("LF W5500 RP2040 Flasher")
        self.root.minsize(850, 480)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.messages: queue.Queue[tuple[str, str]] = queue.Queue()
        self.rows: dict[str, tuple[tk.BooleanVar, tk.StringVar, str]] = {}
        self.saved_rows: dict[str, tuple[bool, str]] = {}
        self.busy = False
        self.load_settings()

        outer = ttk.Frame(root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            outer,
            text="Map each running federate's serial port to a UF2 image. "
            "Devices are rebooted and flashed sequentially.",
        ).pack(anchor=tk.W, pady=(0, 10))

        toolbar = ttk.Frame(outer)
        toolbar.pack(fill=tk.X, pady=(0, 8))
        self.refresh_button = ttk.Button(
            toolbar, text="Refresh serial ports", command=self.refresh
        )
        self.refresh_button.pack(side=tk.LEFT)
        self.flash_button = ttk.Button(toolbar, text="Flash selected", command=self.start_flash)
        self.flash_button.pack(side=tk.RIGHT)

        self.device_frame = ttk.LabelFrame(outer, text="Connected serial devices", padding=8)
        self.device_frame.pack(fill=tk.X)
        self.device_frame.columnconfigure(3, weight=1)

        ttk.Label(self.device_frame, text="Flash").grid(row=0, column=0, padx=4, sticky=tk.W)
        ttk.Label(self.device_frame, text="Serial port").grid(
            row=0, column=1, padx=4, sticky=tk.W
        )
        ttk.Label(self.device_frame, text="Description").grid(
            row=0, column=2, padx=4, sticky=tk.W
        )
        ttk.Label(self.device_frame, text="UF2 firmware").grid(
            row=0, column=3, padx=4, sticky=tk.W
        )

        ttk.Label(outer, text="Progress").pack(anchor=tk.W, pady=(12, 3))
        self.log = tk.Text(outer, height=12, wrap=tk.WORD, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True)

        self.root.after(100, self.process_messages)
        self.refresh()

    def append_log(self, message: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, message + "\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def close(self) -> None:
        if self.busy:
            messagebox.showwarning(
                "Flashing in progress", "Wait for the current flashing operation to finish."
            )
            return
        self.save_row_state()
        self.root.destroy()

    def load_settings(self) -> None:
        path = settings_file()
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
            devices = document.get("devices", {})
            self.saved_rows = {
                identity: (bool(values.get("selected", False)), str(values.get("firmware", "")))
                for identity, values in devices.items()
                if isinstance(values, dict)
            }
        except (FileNotFoundError, OSError, ValueError, TypeError):
            self.saved_rows = {}

    def save_settings(self) -> None:
        path = settings_file()
        document = {
            "version": 1,
            "devices": {
                identity: {"selected": selected, "firmware": firmware}
                for identity, (selected, firmware) in self.saved_rows.items()
            },
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
            temporary.replace(path)
        except OSError as error:
            self.append_log(f"WARNING: could not save firmware mappings: {error}")

    def save_row_state(self) -> None:
        for selected, firmware, identity in self.rows.values():
            self.saved_rows[identity] = selected.get(), firmware.get()
        self.save_settings()

    def refresh(self) -> None:
        if self.busy:
            return
        self.save_row_state()
        for widget in self.device_frame.grid_slaves():
            if int(widget.grid_info()["row"]) > 0:
                widget.destroy()
        self.rows.clear()

        ports = sorted(list_ports.comports(), key=lambda item: natural_port_key(item.device))
        if not ports:
            ttk.Label(self.device_frame, text="No serial ports found").grid(
                row=1, column=0, columnspan=5, padx=4, pady=8, sticky=tk.W
            )
            return

        for row_number, port_info in enumerate(ports, start=1):
            identity = port_identity(port_info)
            old_selected, old_firmware = self.saved_rows.get(identity, (False, ""))
            selected = tk.BooleanVar(value=old_selected)
            firmware = tk.StringVar(value=old_firmware)
            self.rows[port_info.device] = selected, firmware, identity

            ttk.Checkbutton(self.device_frame, variable=selected).grid(
                row=row_number, column=0, padx=4
            )
            ttk.Label(self.device_frame, text=port_info.device).grid(
                row=row_number, column=1, padx=4, sticky=tk.W
            )
            ttk.Label(self.device_frame, text=port_info.description).grid(
                row=row_number, column=2, padx=4, sticky=tk.W
            )
            ttk.Entry(self.device_frame, textvariable=firmware).grid(
                row=row_number, column=3, padx=4, pady=2, sticky=tk.EW
            )
            ttk.Button(
                self.device_frame,
                text="Browse...",
                command=lambda value=firmware: self.choose_firmware(value),
            ).grid(row=row_number, column=4, padx=4)

    def choose_firmware(self, target: tk.StringVar) -> None:
        selected = filedialog.askopenfilename(
            title="Select RP2040 UF2 firmware",
            filetypes=(("UF2 firmware", "*.uf2"), ("All files", "*.*")),
        )
        if selected:
            target.set(selected)
            self.save_row_state()

    def start_flash(self) -> None:
        jobs: list[FlashJob] = []
        self.save_row_state()
        for port, (selected, firmware_value, _identity) in self.rows.items():
            if not selected.get():
                continue
            firmware = Path(firmware_value.get()).expanduser()
            if not firmware.is_file() or firmware.suffix.lower() != ".uf2":
                messagebox.showerror("Invalid firmware", f"Select a valid UF2 for {port}.")
                return
            jobs.append(FlashJob(port, firmware.resolve()))

        if not jobs:
            messagebox.showinfo("Nothing selected", "Select at least one serial port to flash.")
            return

        self.busy = True
        self.refresh_button.configure(state=tk.DISABLED)
        self.flash_button.configure(state=tk.DISABLED)
        self.append_log(f"Starting {len(jobs)} device(s). Close any serial monitors first.")
        threading.Thread(target=self.flash_jobs, args=(jobs,), daemon=True).start()

    def flash_jobs(self, jobs: list[FlashJob]) -> None:
        try:
            for position, job in enumerate(jobs, start=1):
                self.messages.put(
                    ("log", f"[{position}/{len(jobs)}] {job.port} -> {job.firmware.name}")
                )
                existing = bootsel_locations()
                self.messages.put(("log", f"Opening {job.port} and requesting BOOTSEL..."))

                try:
                    with serial.Serial(
                        job.port,
                        baudrate=115200,
                        timeout=1,
                        write_timeout=2,
                    ) as connection:
                        # Force a DTR transition and keep the handle open until
                        # the device has actually rebooted. Closing immediately
                        # can leave the command pending in the host's CDC path.
                        connection.dtr = False
                        time.sleep(0.1)
                        connection.dtr = True
                        time.sleep(0.75)
                        connection.reset_input_buffer()
                        connection.write(b"\r\n")
                        connection.flush()
                        time.sleep(0.15)
                        connection.write(b"bootsel\r\n")
                        connection.flush()
                        if os.name == "posix":
                            self.messages.put(
                                (
                                    "log",
                                    "BOOTSEL requested. Mount its filesystem now; "
                                    "waiting for INFO_UF2.TXT...",
                                )
                            )
                        location = wait_for_new_bootsel_location(
                            existing, bootsel_timeout()
                        )
                except serial.SerialException as error:
                    raise RuntimeError(
                        f"cannot open {job.port}: {error}. Close every serial monitor using it"
                    ) from error

                destination = Path(location) / job.firmware.name
                self.messages.put(
                    ("log", f"BOOTSEL mounted at {location}; copying firmware...")
                )
                shutil.copyfile(job.firmware, destination)
                wait_for_bootsel_to_disappear(location, bootsel_timeout())
                self.messages.put(("log", f"{job.port}: flash complete and board rebooted."))
                time.sleep(1.0)

            self.messages.put(("log", "All selected devices flashed successfully."))
        except Exception as error:
            self.messages.put(("error", str(error)))
        finally:
            self.messages.put(("done", ""))

    def process_messages(self) -> None:
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "log":
                    self.append_log(value)
                elif kind == "error":
                    self.append_log("ERROR: " + value)
                    messagebox.showerror("Flashing failed", value)
                elif kind == "done":
                    self.busy = False
                    self.refresh_button.configure(state=tk.NORMAL)
                    self.flash_button.configure(state=tk.NORMAL)
                    self.refresh()
        except queue.Empty:
            pass
        self.root.after(100, self.process_messages)


def main() -> int:
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"Cannot open the graphical interface: {error}")
        return 1
    if os.name not in ("nt", "posix"):
        root.withdraw()
        messagebox.showerror("Unsupported platform", "Use this flasher on Windows or Linux.")
        return 1
    if serial is None or list_ports is None:
        root.withdraw()
        install_command = (
            "py -m pip install pyserial"
            if os.name == "nt"
            else "python3 -m pip install pyserial"
        )
        messagebox.showerror(
            "pyserial required",
            f"Install pyserial first:\n\n{install_command}",
        )
        return 1

    FlashGui(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
