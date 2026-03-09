# Setup

- `nRF54L15DK`: Source
- `nRF52840DK`: Sink
- Power supply: USB connection (Laptop)

If not stated otherwise, all traces have a duration of exactly 14 seconds.

A memory depth of 70k is usually reasonable for a trace of 14s, but keep in mind that if data is lost and packets are received too late (TCP retransmission), the packets might "dam up", resuling in the LED toggling reaction being invoked in quick succession. These events cannot usually be captured with 70k memory depth (see `25Hz-VeryFastSuccession.csv`). In order to capture these events, unfortunately, 700k memory depth must be chosen.

<p align="center">
  <img src="measurements/setupA.png" alt="Closeup of experiment setup using oscilloscope" title="Closeup of experiment setup" width="49%" />
  <img src="measurements/setupB.png" alt="Overview of experiment setup using oscilloscope" title="Overview of experiment setup" width="49%" />
</p>

Additional info: Almost no logging (`logging: warn`), only inspecting the `CLOCK_SYNC` offset of one device (logging on both).

## Experiment: FedDelay

The federation `src/FedDelay` consists of a physical connectinon between two federates. One federate (source) emits an untagged event and toggles his LED, and the other federate receives the event and processes it as soon as possible, toggling his LED. The measured delay between these LED toggles indicates the complete processing and transmission delay, including scheduling overhead etc.

The program is as follows:
<p align="center">
	<img src="src/FedDelay.svg" alt="Program using physical connection" title="Program with physical connection" width="700" />
</p>

### 5Hz vs 25Hz communication rate

The left view uses a `200ms` period (5Hz), the right view uses a `40ms` period (25Hz).

<p align="center">
  <img src="measurements/FedDelay/5Hz-CalibNoLog-zoom.svg" alt="Measurements using 5Hz communication rate" title="5Hz communication rate" width="49%" />
  <img src="measurements/FedDelay/25Hz-CalibNoLogWrongEdges-zoom.svg" alt="Measurements using 25Hz communication rate" title="25Hz communication rate" width="49%" />
</p>

also consider

<p align="center">
  <img src="measurements/FedDelay/25Hz-CalibNoLogShortFail.svg" alt="Measurements using 25Hz communication rate with TCP retransmission" title="25Hz communication rate" width="49%" />
  <img src="measurements/FedDelay/25Hz-CalibNoLogShortFail-zoom.svg" alt="Measurements using 25Hz communication rate with TCP retransmission" title="25Hz communication rate" width="49%" />
</p>

## Experiment: FedClockSync

In this experiment, we use a logical delay that exceeds the maximum transmission delay determined in experiment `FedDelay`. The logical delay will "absorb" the transmission delay and after evaluating the end-to-end delay and subtracting the known logical delay, the remaining value reflects clock
synchronization error.

The program is as follows:
<p align="center">
	<img src="src/FedClockSync.svg" alt="Program using logical connection" title="Program with logical connection" width="700" />
</p>

The resulting clock sync error distribution is:
<p align="center">
	<img src="measurements/FedClockSync/5Hz.png" alt="Clock Sync error estimation using logical connection" title="Clock Sync Estimation" width="49%" />
</p>

Scope view:
<p align="center">
	<img src="measurements/FedClockSync/ScopeView.png" alt="Scope view of the experiment using logical connection" title="Scope view of the experiment using logical connection" width="700" />
</p>