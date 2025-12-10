# Fast DDS Monitor Guide

Technical guide for interpreting metrics exposed by eProsima Fast DDS Monitor for ROS 2 developers.

---

## Installation

For installation and environment variable configuration, follow:

**[https://github.com/leonardonels/zero_copy/tree/main/fast_dds_monitor](https://github.com/leonardonels/zero_copy/tree/main/fast_dds_monitor)**

---

## 1. Fundamental Concepts

### Transport Layer: UDP vs Shared Memory

| Mode | Scenario | Monitor Behavior |
|------|----------|-----------------|
| **UDP / Network** | Robot-to-PC, robot-to-robot (WLAN/LAN) | Complete: packet loss, retransmissions, overhead visible |
| **Shared Memory** | Nodes on same PC (localhost) | Partial: Resent Data, Heartbeat, Acknack always 0 |

### DDS Entities

| Entity | Description |
|--------|-------------|
| DomainParticipant | Main ROS 2 node |
| DataWriter | Publisher (sends data) |
| DataReader | Subscriber (receives data) |
| Locator | Physical address (IP:Port) |
| Topic | Logical channel (e.g., /cmd_vel) |

---

## 2. Metrics Dictionary

### Performance Metrics

| Metric | Description |
|--------|-------------|
| FASTDDS_LATENCY | Time between Writer write and Reader notification |
| PUBLICATION_THROUGHPUT | Data send rate (MB/s) |
| SUBSCRIPTION_THROUGHPUT | Data receive rate (MB/s) |
| DATA_COUNT | Total samples sent |

### Reliability Metrics

| Metric | Description |
|--------|-------------|
| HEARTBEAT_COUNT | Control messages from Writer announcing new data |
| ACKNACK_COUNT | Reader requests for missing packets |
| NACKFRAG_COUNT | Requests for lost fragments (large messages) |

### Diagnostic Metrics

| Metric | Description |
|--------|-------------|
| RESENT_DATA | Packets resent by Writer |
| GAP_COUNT | Data overwritten in buffer (bottleneck indicator) |
| PDP/EDP PACKETS | Discovery traffic (active at startup) |

---

## 3. Graph Configuration

### Status Legend

| Symbol | Meaning |
|--------|---------|
| [OK] | Always works |
| [SHM] | Often 0 in Shared Memory (normal) |
| [ERR] | Must be 0 (values > 0 = error) |

---

### A: Latency

| Data Kind | Source -> Target | Statistic | Status |
|-----------|-----------------|-----------|--------|
| FASTDDS_LATENCY | DataWriter -> DataReader | MEAN | [OK] |
| FASTDDS_LATENCY | DataWriter -> DataReader | MAX | [OK] |
| FASTDDS_LATENCY | DataWriter -> DataReader | MIN | [OK] |
| FASTDDS_LATENCY | DataWriter -> DataReader | MEDIAN | [OK] |
| FASTDDS_LATENCY | DataWriter -> DataReader | STANDARD_DEVIATION | [OK] |
| FASTDDS_LATENCY | Topic -> Topic | MEAN | [OK] |

### B: Throughput

| Data Kind | Source | Statistic | Status |
|-----------|--------|-----------|--------|
| PUBLICATION_THROUGHPUT | DataWriter | MEAN | [SHM] |
| PUBLICATION_THROUGHPUT | DataWriter | MAX | [SHM] |
| SUBSCRIPTION_THROUGHPUT | DataReader | MEAN | [OK] |
| SUBSCRIPTION_THROUGHPUT | DataReader | MAX | [OK] |
| DATA_COUNT | DataWriter | SUM | [SHM] |

### C: Writer Reliability

| Data Kind | Source | Statistic | Status |
|-----------|--------|-----------|--------|
| RESENT_DATA | DataWriter | SUM | [SHM] |
| HEARTBEAT_COUNT | DataWriter | SUM | [SHM] |
| GAP_COUNT | DataWriter | SUM | [ERR] |

### D: Reader Reliability

| Data Kind | Source | Statistic | Status |
|-----------|--------|-----------|--------|
| ACKNACK_COUNT | DataReader | SUM | [SHM] |
| NACKFRAG_COUNT | DataReader | SUM | [SHM] |

### E: Discovery

| Data Kind | Source | Statistic |
|-----------|--------|-----------|
| PDP_PACKETS | DomainParticipant | SUM |
| EDP_PACKETS | DomainParticipant | SUM |

### F: Per-IP Analysis (network only)

| Data Kind | Source | Statistic | Status |
|-----------|--------|-----------|--------|
| PUBLICATION_THROUGHPUT | Locator | MEAN | [SHM] |
| RESENT_DATA | Locator | SUM | [SHM] |

### G: Per-Process Analysis

| Data Kind | Source | Statistic |
|-----------|--------|-----------|
| PUBLICATION_THROUGHPUT | Process | MEAN |
| SUBSCRIPTION_THROUGHPUT | Process | MEAN |
| RESENT_DATA | Process | SUM |

---

## 4. Troubleshooting

### All Reliability Graphs at ZERO

- **Context:** Nodes on same PC
- **Diagnosis:** Normal (Shared Memory active)
- **Action:** Monitor only Latency and Throughput

### High Latency + Increasing Acknack

- **Context:** WiFi/Ethernet
- **Diagnosis:** Packet Loss
- **Action:** Check WiFi signal or reduce data bandwidth

### Increasing Gap Count

- **Context:** Any
- **Diagnosis:** Writer Overload (buffer overflow)
- **Action:** Increase History QoS depth or optimize Publisher

### Nackfrag Count > 0

- **Context:** Images or PointCloud
- **Diagnosis:** Packet fragmentation
- **Action:** Check network MTU or reduce resolution

---

## 5. SIM Transport Note

Fast DDS Monitor **cannot observe SIM traffic** because SIM bypasses DDS entirely.

For SIM monitoring:
- Check application logs for FPS and latency
- Monitor `/dev/shm/sim_*` files
- Use built-in dropped frame counters
