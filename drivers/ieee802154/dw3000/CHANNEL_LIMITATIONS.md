# DW3000 IEEE802154 Channel Limitations

## Status
- Channel 5: supported
- Channel 9: disabled in `ieee802154_dw3000`

## Why channel 9 is disabled
Validation runs with two DWM3001CDK nodes showed the following on channel 9:
- TX counters increase (`tx_ok`), so frames are being sent.
- RX enable succeeds continuously.
- RX preamble detect never fires (`PRD=0`), and no SFD/PHY/frame events are observed.

This indicates no usable on-air reception on channel 9 in this driver+board setup, even though channel/rf registers report channel-9 values.

To avoid silent failure, the driver now rejects channel 9 explicitly:
- Kconfig selection with `CONFIG_NET_CONFIG_IEEE802154_CHANNEL=9` returns init error.
- Runtime `set_channel(9)` returns `-ENOTSUP`.
- Advertised supported channel set is channel 5 only.

## Practical guidance
Use channel 5 for DWM3001 IEEE802154 communication in this repository until a full channel-9 RF bring-up is completed and validated.
