# Tardy packets at 10Hz

DW3000 driver: 0.3%
nRF5 driver: >95%

# Lateness of packets

Occasionally, using the DW3000 driver, we receive packets up to 50ms late (both RUDP acknowledgements and data packets, also on TCP), although no retransmission is triggered.

On the nRF5 driver, we can even see lateness of up to 450ms.

**TODO**: Compare! Also check using logic analyzer. Pretty sure that clock sync is not the issue here.