# Source-Sink (2 nodes)

## Retransmission rate (25Hz)

```
- With DW3000 IEEE802.15.4 driver (CH05):         0.073  %
                                                  0.0021 % (after RX fix)
- With DW3000 IEEE802.15.4 driver (CH05, Aloha): 13.000  %
- With nRF5   IEEE802.15.4 driver (CH26):         0.460  %
```

HACK delay DW3000: TODO
HACK delay nRF5: probably around 30ms, but not measured (yet).

## Retransmission rate (40Hz) - after RX restart frame discard fix(?)

```
- With DW3000 IEEE802.15.4 driver (CH05):         1.220 %
                                                  0.053 % (after RX fix)
- With nRF5   IEEE802.15.4 driver (CH26):         0.590 %
```

DW3000: On rare occasions, we seem to get a lot of unnecessary retransmissions (lots of duplicates on receiver side). Perhaps the sender is not able to process received ACKs fast enough, and thus retransmits unnecessarily. Only happens rarely with high traffic load (40Hz), and not at all with low traffic load (25Hz).
nRF5: Not sure if this happens here, too.