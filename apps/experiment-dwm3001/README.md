# Retransmission rate (25Hz)

```
- With DW3000 IEEE802.15.4 driver (CH05):         0.073  %
                                                  0.0021 % (after RX fix)
- With DW3000 IEEE802.15.4 driver (CH05, Aloha): 13.000  %
- With nRF5   IEEE802.15.4 driver (CH26):         0.460  %
```

HACK delay DW3000: TODO
HACK delay nRF5: probably around 30ms, but not measured (yet).

# Retransmission rate (40Hz) - after RX restart frame discard fix(?)

```
- With DW3000 IEEE802.15.4 driver (CH05):         0.032 %
- With nRF5   IEEE802.15.4 driver (CH26):         0.590 %
```

HACK delay DW3000: Noticed HACK delays of up to 110ms (30ish ms expected) in the DW3000 driver; may be related to the RX fix. Seems like the driver is trottling the RX path; TX works fine, RX is slow?
HACK delay nRF5: Normal 30ms HACK delay for nRF5 driver.