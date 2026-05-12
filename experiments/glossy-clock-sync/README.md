# Glossy Clock Sync (Proof of Concept)

This small Zephyr application runs a Glossy-based clock synchronization round
using the workspace UWB driver API. It is intended as a minimal proof-of-concept
for two DWM3001CDK boards.

Build (on host):

```bash
west build -b decawave_dwm3001cdk experiments/glossy-clock-sync
```

Flash (example):

```bash
west flash
```

Set role / node id before building by editing `experiments/glossy-clock-sync/prj.conf`:

- Uncomment `CONFIG_GLOSSY_IS_INITIATOR=y` on the initiator board.
- Set `CONFIG_GLOSSY_NODE_ID` to `1` and `2` respectively for the two boards.

Logs are printed over the console (UART). The app prints received glossy results
including root id, hop count and the paired timestamps (RTC and deca timestamp).
