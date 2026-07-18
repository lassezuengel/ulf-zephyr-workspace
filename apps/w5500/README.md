# W5500 Federations

This example runs two Lingua Franca federates on W5500-EVB-Pico boards. The
federates communicate over Zephyr's native IPv4 stack using TCP; LF clock
synchronization currently uses UDP AFAIK.

## Build

Set `REACTOR_UC_PATH` to the reactor-uc checkout and make sure `west` is
available. The script also finds this workspace's `venv/bin/west` when `west`
is not on `PATH`.

From this directory, run:

```sh
./build.py
```

The resulting firmware images are:

- `src-gen/PingPong/ping/build/zephyr/zephyr.uf2`
- `src-gen/PingPong/pong/build/zephyr/zephyr.uf2`

`PingPong.lf` remains the default. Select another top-level LF file in `src/`
with `--source` (the `.lf` suffix is optional):

```sh
./build.py --source DelayChain
```

Both `@interface_tcp(address="...")` and
`@interface_rudp(address="...")` federates are supported.

The default network identities are:

| Federate | IPv4 address | MAC address |
| --- | --- | --- |
| `ping` | `192.168.50.10/24` | `02:00:00:00:00:01` |
| `pong` | `192.168.50.11/24` | `02:00:00:00:00:02` |

The boards use static addresses and no gateway or DHCP, so they can be
connected directly or through an isolated Ethernet switch.

Application output and reactor-uc logs use USB CDC ACM. After flashing and
resetting a board, its USB connector appears on the host as `/dev/ttyACM*` (or
the platform-equivalent serial device); no external USB-to-UART adapter is
needed. The serial baud-rate selection is nominal for USB CDC. Ethernet driver
and network-configuration errors are also sent to this console; early driver
messages are buffered briefly while USB initializes. Because USB CDC normally
drops output written before a host opens the port, the wrapper also adds a tiny
non-blocking notifier thread. Whenever a serial monitor asserts DTR, it prints:

```text
LF W5500 firmware alive; CDC console connected.
```

The notifier waits independently and does not delay LF or Ethernet startup.

The CDC device is also an interactive Zephyr shell. Open `/dev/ttyACM*` in a
serial terminal and press Enter to get the `w5500:~$` prompt. The baud-rate
setting is ignored by USB CDC. Useful diagnostics include:

```text
net iface
net iface show 1
net events on
net arp
net ping 192.168.50.11
net tcp
net udp
```

`net iface show 1` distinguishes the administrative state from physical
carrier. Run `net events on`, then unplug and reconnect Ethernet to observe
interface/link events. Zephyr logs and the shell share the CDC port; internal
USB control-transfer logging is disabled so it does not flood the terminal.

Useful options are:

```sh
# Reuse src-gen and only customize/build the existing generated projects.
./build.py --skip-generate

# Force clean Zephyr builds for both federates.
./build.py --pristine always
```

Run `./build.py --help` for all options.

## Flash from Windows 11

The generated firmware provides a `bootsel` shell command that calls the
RP2040 ROM USB-boot function. This allows a Windows host to move a running
federate from its CDC COM port into BOOTSEL mode without pressing the physical
button. Install the one Python dependency and start the GUI from PowerShell:

```powershell
py -m pip install pyserial
py flash_gui.py
```

Select the desired COM ports, map a `.uf2` file to each one, and click **Flash
selected**. The GUI handles devices sequentially: it sends `bootsel`, waits for
the newly mounted `RPI-RP2` drive, copies that device's UF2, waits for it to
reboot, and only then proceeds to the next mapping. Serial monitors must be
closed while flashing because Windows permits only one program to own a COM
port at a time.

Firmware selections are retained in
`%LOCALAPPDATA%\LF W5500 Flasher\settings.json`. When the CDC device exposes a
USB serial number, the mapping uses that stable identity rather than the COM
number, so it survives Windows assigning another COM port. The stored value is
the UF2 path, not a stale copy; rebuilding the firmware at that path makes the
new image available on the next flash automatically.

The first installation of firmware containing the `bootsel` command still
requires the physical BOOTSEL button. The button is also the recovery path if
the application is damaged, CDC does not enumerate, or the COM port is busy.
Automatic flashing requires Windows to assign a drive letter to the RP2040
`RPI-RP2` mass-storage volume.

### Linux and WSL2

The same GUI runs on native Linux and under WSL2/WSLg. On Ubuntu, install Tk
and pyserial, then start it from the application directory:

```sh
sudo apt install python3-tk python3-venv
python3 -m venv .venv
. .venv/bin/activate
python -m pip install pyserial
python flash_gui.py
```

Linux mappings are stored in
`${XDG_CONFIG_HOME:-$HOME/.config}/lf-w5500-flasher/settings.json`. The user
running the GUI must have access to `/dev/ttyACM*`; on a regular Linux system
this normally means membership in the `dialout` group.

On WSL2, attach the running CDC device using `usbipd-win` so it appears as
`/dev/ttyACM*`. When the GUI sends `bootsel`, mount the newly appearing
mass-storage filesystem while the GUI waits. For example, when it is attached
to Linux as a block device:

```powershell
usbipd list
usbipd bind --busid <busid>       # Administrator PowerShell, once
usbipd attach --wsl --busid <busid>
```

The RP2040 changes USB identity when it enters BOOTSEL, so some `usbipd-win`
setups require running `usbipd attach` again after that transition. Then, in
WSL:

```sh
lsblk -f
sudo mkdir -p /mnt/rpi-rp2
sudo mount /dev/sdX1 /mnt/rpi-rp2
```

If Windows owns the BOOTSEL drive as, for example, `E:`, expose that existing
drive to WSL instead:

```sh
sudo mkdir -p /mnt/rpi-rp2
sudo mount -t drvfs E: /mnt/rpi-rp2
```

The GUI scans mounted filesystems for the RP2040 `INFO_UF2.TXT` marker, so the
mountpoint name is arbitrary. It waits 90 seconds on Linux to allow time for
manual mounting. It does not invoke `sudo`, modify `/etc/fstab`, or choose a
block device automatically.

## What the wrapper does

For a typical 6LoWPAN federation using `net-interface: sicslowpan`, `lfc-dev`
can generate and build the complete configuration directly. For this W5500
example, `net-interface: ethernet` does not currently provide all of the
board-specific static IPv4 and MAC configuration required by the two physical
Ethernet interfaces. In particular, each board needs its own address and must
use Zephyr's native TCP/IP stack instead of the socket-offload settings emitted
by the compiler.

Rather than extend or otherwise change the LF compiler architecture for this
experiment, `build.py` adds a small post-generation step:

1. Run `lfc-dev src/<selected-file>.lf -n` to generate without building.
2. Read each federate name and IPv4 address from its `@interface_tcp` or
   `@interface_rudp` annotation.
3. Copy the shared `prj.conf` into each generated Zephyr project and append
   that federate's IPv4 address. This user configuration is applied after
   `prj_lf.conf`, so it can override compiler-generated Kconfig values.
4. Generate an `app.overlay` with a distinct, locally administered MAC address
   for each federate and select USB CDC ACM as Zephyr's console.
5. Add a small `cdc_console.c` notifier to each project so opening the USB
   serial monitor always produces a visible sign of life, even if all boot
   messages have already passed.
6. Invoke `west build` separately for every generated project, explicitly
   passing the Kconfig and devicetree overlays.

Set a distinct, non-loopback IPv4 address on each `@interface_tcp` or
`@interface_rudp` annotation. Change addresses in the selected LF source; do
not edit generated copies in `src-gen`, because they are replaced by the next
build.

## Source parsing limitation

The wrapper currently extracts explicitly addressed `@interface_tcp` and
`@interface_rudp` annotations with a deliberately small regular expression.
This is adequate for these examples, but it is not a general or especially
scalable LF parser: changes to annotation syntax or more complex ways of
expressing configuration may require updates to the wrapper.

A cleaner long-term interface would let `lfc-dev` emit a machine-readable
description, such as JSON, containing the resolved federation, target
configuration, federates, and relevant AST annotations. External build tools
could then consume compiler-owned semantic data instead of reparsing LF source.
Adding such a compiler interface is intentionally outside the scope of this
example.

# Results

Regarding the `DelayChain` example, we see that we have basically no packet
loss with a 20 ms period, and the average forwarding delay is roughly 9ms.
No retransmissions, thus no latency spikes. The Works much more reliably than
the 6LoWPAN version, which is not surprising given that the W5500 is a dedicated
Ethernet controller. However,
 1. Using a reliable clock sync channel, wee see much higher offset fluctuations
    (4ms - 16ms) vs unreliable UDP (6ms - 10ms). This is likely due to the fact
    that the clock sync channel is also used for data, and overall TCP usage is
    higher, and also less symmetric and predictable network performance. Shows
    that UDP-based clock sync is feasible not only for 6LoWPAN, but also for Ethernet.
 2. RUDP ping-pong delay is much lower at <15ms, compared to 18ms-19ms with TCP.

I.e., TCP works okay with W5500, but reliable (and - for clock sync - unreliable)
UDP is better.