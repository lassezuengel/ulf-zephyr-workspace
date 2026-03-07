Evaluates multi-hop transmission delays and clock synchronization errors. The programs are an extension of the `FedDelay` and `FedClockSync` experiments in that we have one sequence of federates (source, intermediate, sink) instead of two federates (source, sink).

Setup:
- `3001CDK` for all federates (`hailo-desktop`)
- Power supply: USB connection (Raspberry PI)

For evaluation, instead of using the oscilloscope setup, we just look at the physical timestamps of the events in the log files.

The logging format is expected to be
```
[<device-name>] [<FEDERATE_ID>] Value <value> at physical time <timestamp> ms and logical time <logical_time> ms
```
where
  - `<device-name> :: String` is the name of the device (e.g., `hailo-desktop:dwm3001cdk-9`) (optional!),
  - `<FEDERATE_ID> :: Int` is the respective federate id,
  - `<value> :: Int` is the value of the event,
  - `<timestamp> :: Float` is the physical time when the event was processed (in ms), and
  - `<logical_time> :: Int` is the logical time of the event (in ms). By comparing the physical timestamps of events across the source, intermediate, and sink federates, we can estimate transmission delays and clock synchronization errors.
Furthermore, in order to support the `iot-lab` serial aggregation, we ignore and drop any log line prefixes that end in `;`.