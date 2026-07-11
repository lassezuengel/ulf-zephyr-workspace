# Delay-chain experiments

This directory contains small federated Lingua Franca experiments for measuring
end-to-end message latency over a wireless delay chain on Zephyr targets.

The experiment uses one source/destination federate and a configurable bank of
forwarder federates. The source/destination federate injects a timestamped
message periodically, each forwarder relays the timestamp unchanged, and the
source/destination federate prints the measured latency when the message returns
at the end of the chain. Varying the number of forwarders changes the chain
length and gives a simple way to compare how latency and jitter scale with the
number of federates.

## Variants

- `dwm3001/`: DWM3001CDK / DW3000 UWB variant using `sicslowpan`, RUDP, and
  clock synchronization.
- `nrf5-saclay/`: nRF52840DK IEEE 802.15.4 variant for the Saclay FIT IoT-LAB
  site.
- `measurements/`: collected latency data and generated plots.
- `delay-chain-plot.py` and `boxplot.py`: plotting helpers for the collected
  CSV data.

## Running and measuring

Set the federated reactor parameter `n` in the relevant `FedDelayChain.lf` file
to choose the number of forwarders in the chain. The latency output is printed
by the source/destination federate in milliseconds:

```text
Latency: <ms>.<fraction> ms
```

The measurement CSV files in `measurements/` separate runs with clock
synchronization enabled and disabled where applicable. The plotting scripts
consume those CSV files and generate the PDF/SVG figures stored next to the
data.

## Logging caution on FIT IoT-LAB

Keep UART output as quiet as possible during measurements. On FIT IoT-LAB,
UART logging can interfere with Zephyr scheduling and processing, which in turn
distorts thread timing and can cause hard-to-debug runtime issues.

In particular, disable or suppress as much colored UART logging as possible,
including Lingua Franca info messages, warnings, and other non-essential Zephyr
or driver logs. Prefer only the latency lines needed for data collection, and
keep the LF target logging level at `error` or quieter for measurement runs.
