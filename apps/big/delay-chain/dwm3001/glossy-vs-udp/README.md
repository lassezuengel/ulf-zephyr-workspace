# Glossy vs. UDP Clock Sync Delay-Chain Experiment

This folder contains one comparison run for the DWM3001 delay-chain example.
The experiment sends timestamp messages through the same chain of federates and
compares two clock-synchronization approaches:

- `glossyrun.txt`: delay chain using Glossy-based clock synchronization.
- `udprun.txt`: delay chain using UDP/IP-based PTP-ish clock synchronization.

The chain logs end-to-end latency at `dwm3001-1`, per-hop forwarding offsets
from the intermediate federates, and RUDP retransmission statistics. The helper
script `glossy-offset-plot.py` extracts those measurements and generates the
figures below.

Regenerate the plots with:

```sh
python3 glossy-offset-plot.py glossyrun.txt udprun.txt
```

Passing only a Glossy log generates its symmetric-log and linear-trend
forwarding-offset plots:

```sh
python3 glossy-offset-plot.py glossy-multihop/n25/glossyrun.txt
```

Generated PDFs are written to the directory containing the Glossy log.
Forwarding-offset plots show the first eight detected nodes by default. The
x-axis maps their device IDs to contiguous hop counts starting at zero. Use
`--node-count N` (or `-n N`) to show the first N nodes, and use `0` to include
all nodes detected in the input logs:

```sh
python3 glossy-offset-plot.py --node-count 0 \
    glossy-multihop/n25/glossyrun.txt
```

First-seen node order is preserved by default. Pass `--sort` to order the
selected nodes by ascending average forwarding offset from the Glossy log:

```sh
python3 glossy-offset-plot.py --node-count 0 --sort \
    glossy-multihop/n25/glossyrun.txt
```

For plots with many hops, label only every second hop to keep the x axis
readable (use a larger step if needed):

```bash
python3 glossy-offset-plot.py --node-count 0 --hop-label-step 2 \
    glossy-multihop/n33/glossyrun.txt
```

## Combined Overview

`glossy-vs-udp.svg` combines the main latency, retransmission, and hop-offset
views in one figure.

![Combined overview](glossy-vs-udp.svg)

## End-to-End Latency

The latency box plot uses all lines of the form `Latency: ... ms`. In this run,
Glossy clock synchronization produces lower median latency and a tighter spread.

![Latency box plot](latency-boxplot.svg)

## RUDP Retransmissions

Retransmissions are extracted from RUDP `[stat] ... packets sent, ...
retransmissions` lines and plotted as a percentage of packets sent. The UDP
clock-sync run shows substantially higher retransmission pressure.

![Retransmission box plot](retransmission-boxplot.svg)

## Hop Offsets

Forwarding offsets are extracted from lines such as `Forwarding message (offset
5.00 ms)`. Device IDs are mapped to contiguous hop counts on the x-axis.
Each Glossy marker is colored by the device's average `sync ok: hops=...`
value. Hop 1 is blue and hop 2 is yellow; fractional averages interpolate
between those colors. The initiator is shown as hop 0. Lines between devices
use a continuous gradient between the neighboring marker colors.

The symmetric-log plot keeps both negative offsets and large UDP excursions
visible.

![Hop offsets symlog](hop-offsets.svg)

The linear trend plot shows only the central trend line, currently the median
per node. This makes the nearly linear Glossy hop-to-hop behavior easier to
inspect.

![Hop offsets linear trend](hop-offsets-linear-trend.svg)

## Glossy RTC Drift

`glossy-rtc-offset-plot.py` extracts the Glossy `rtc_offset=... ms` reports and
plots their development over elapsed log time. `dwm3001-1` is the Glossy root
and reference device, so all other offsets are relative to it. The first
reported offset for each device is treated as the constant startup offset and
subtracted from that device's later measurements, so the plot shows drift from
the first sync rather than absolute clock offset.

Regenerate this plot with:

```sh
python3 glossy-rtc-offset-plot.py
```

![Glossy RTC drift](glossy-rtc-drift.svg)
