# Multi-Hop ("Pipeline")

This example demonstrates a multi-hop ("pipeline") network of N nodes. The first node (Node 1) is the initiator, and the last node (Node N) is the final recipient. Each node in between forwards the message to the next node in the chain.

## 4 nodes

### 50ms logical delay per node

Not in time (NIT, tardy / not arriving):

Setup (Node IDs)  |    Rate / Delay    | DW3000             |  nRF5               |
----------------- | ------------------ | ------------------ | ------------------- |
97-100 (aneochic) |     1Hz / 50ms     |                    |        3.6%         |

Setup (Node IDs)  |    Rate / Delay    | DW3000             |  nRF5               | nRF5 (saclay)       |
----------------- | ------------------ | ------------------ | ------------------- | ------------------- |
1-4               |     1Hz / 50ms     |         1%         |        6.7%         |     3.8%     (TBA)  |

-> nRF5: `retransmissions - duplicates` correlates with NIT (between node 3 and 4), see `1hz-rudp-n4.txt`. Average retransmissions is 12% (+-2% usually).

Not in time (NIT, tardy / not arriving) with `75ms` deadline:

   Transport      |    Rate / Delay    | DW3000             |  nRF5
----------------- | ------------------ | ------------------ | -------------------
      TCP         |     1Hz / 50ms     |                    |        14.6%
      RUDP        |     1Hz / 50ms     |                    |          7%


fe80::1-fe80::12

-> RUDP: Basically no deadline violations causing NITs. TCP: Some deadline violations in case of retransmission delays.

### 100ms logical delay per node

Setup (Node IDs)  |    Rate / Delay    | DW3000             |  nRF5               |
----------------- | ------------------ | ------------------ | ------------------- |
1-4               |     1Hz / 50ms     |                    |        6.7%         |

### Physical connection

Tests the retransmission count / channel congestion without any periodic clock synchronization.

Retransmission count:

Setup (Node IDs)  |    Rate / Delay    | DW3000             |  nRF5               |
----------------- | ------------------ | ------------------ | ------------------- |
1-4               |   1Hz / 40ms+10ms  |                    |         7%          |

## 8 nodes

### 50ms logical delay per node

Not in time (NIT, tardy / not arriving):

|    Rate / Delay    | DW3000             |  nRF5               |
| ------------------ | ------------------ | ------------------- |
|     1Hz / 50ms     |  35/1055=3.3%      |        > 70%        |

    320/3000 = 10.7% retransmissions (nRF5)
      (peak node: 36% retransmissions)

      16/1000 =  1.6% retransmissions (DW3000)

### 100ms logical delay per node

Not in time (NIT, tardy / not arriving):

|    Rate / Delay    | DW3000             |  nRF5               |
| ------------------ | ------------------ | ------------------- |
|     1Hz / 100ms    |       <1.1%        |        > 70%        |

### 50ms logical delay per node + RTT outlier filtering

Not in time (NIT, tardy / not arriving):

|    Rate / Delay    | DW3000             |  nRF5               |
| ------------------ | ------------------ | ------------------- |
|     1Hz / 50ms     |  44/2050=2.15%     |                     |