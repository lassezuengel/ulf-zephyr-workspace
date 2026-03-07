Setup:
- `nRF54L15DK`: Source
- `nRF52840DK`: Sink
- Power supply: USB connection (Laptop)

If not stated otherwise, all traces have a duration of exactly 14 seconds.

<p align="center">
  <img src="measurements/setupA.png" alt="Closeup of experiment setup using oscilloscope" title="Closeup of experiment setup" width="49%" />
  <img src="measurements/setupB.png" alt="Overview of experiment setup using oscilloscope" title="Overview of experiment setup" width="49%" />
</p>

## Experiment: FedDelay

The federation `src/FedDelay` consists of a physical connectinon between two federates. One federate (source) emits an untagged event and toggles his LED, and the other federate receives the event and processes it as soon as possible, toggling his LED. The measured delay between these LED toggles indicates the complete processing and transmission delay, including scheduling overhead etc.

The program is as follows:
<p align="center">
	<img src="src/FedDelay.svg" alt="Program using physical connection" title="Program with physical connection" width="700" />
</p>

### 5Hz vs 25Hz communication rate

The left view uses a `200ms` period (5Hz), the right view uses a `40ms` period (25Hz).

<p align="center">
  <img src="measurements/FedDelay/5Hz.png" alt="Measurements using 5Hz communication rate" title="5Hz communication rate" width="49%" />
  <img src="measurements/FedDelay/25Hz.png" alt="Measurements using 25Hz communication rate" title="25Hz communication rate" width="49%" />
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