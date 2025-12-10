# Performance Analysis: High-Bandwidth Video Transport in ROS 2

Technical evaluation of 1080p video streaming (6 MB frames) at 30 Hz (180 MB/s throughput).

---

## Executive Summary

This analysis compares three transport architectures for high-bandwidth video streaming in ROS 2:

| Architecture | End-to-End Latency | Middleware Latency | Jitter (σ) |
|--------------|-------------------|-------------------|------------|
| **Standard** | 7.19 ms | 3.97 ms | ≈1.81 ms |
| **Loaned (Zero-Copy)** | 3.10 ms | 0.28 ms | ≈0.38 ms |
| **SIM with Fallback** | 1.03 ms | N/A | ≈0.15 ms |
| **SIM Pure** | 0.68 ms | N/A | ≈0.02 ms |

**Key Finding:** While Fast DDS achieves near-instantaneous transfers (0.28 ms), application-level overhead in ROS 2 significantly penalizes Loaned nodes (3.10 ms total). SIM Pure, bypassing the messaging stack for payload, achieves 0.68 ms average latency—the only viable solution for hard real-time HD video.

---

## Methodology

### Measurement Points

1. **Application-Level (End-to-End):** Timestamp at publish → timestamp at receive
2. **Middleware-Level (Fast DDS Monitor):** DDS transport time only

### Test Configuration

- **Frame Size:** 1920×1080×3 = 6,220,800 bytes (~6 MB)
- **Frame Rate:** 30 Hz
- **Throughput:** ~180 MB/s
- **Platform:** Ubuntu 22.04, ROS 2 Humble, Fast DDS

---

## Detailed Results

### Latency Statistics

| Metric | Standard | Loaned (ZC) | SIM Fallback | SIM Pure |
|--------|----------|-------------|--------------|----------|
| Mean | 7.19 ms | 3.10 ms | 1.03 ms | 0.68 ms |
| Min | 6.07 ms | 2.51 ms | 0.67 ms | 0.64 ms |
| Max | 14.90 ms | 5.25 ms | 1.74 ms | 0.99 ms |
| Jitter | ≈1.81 ms | ≈0.38 ms | ≈0.15 ms | ≈0.02 ms |

### Overhead Analysis

The discrepancy between middleware and application latency reveals significant overhead:

| Architecture | Middleware | App Latency | Overhead | Overhead % |
|--------------|-----------|-------------|----------|------------|
| Standard | 3.97 ms | 7.19 ms | 3.22 ms | 45% |
| Loaned | 0.28 ms | 3.10 ms | 2.82 ms | **91%** |
| SIM Pure | N/A | 0.68 ms | ~0.40 ms | ~59% |

**Critical Insight:** For Loaned nodes, 91% of latency is overhead—the middleware transport (0.28 ms) is nearly instantaneous, but memory copy from video buffer to loaned buffer dominates.

---

## Architecture Analysis

### Standard (Baseline)

**Data Path:**
```
Sensor → ROS Msg Conversion → Serialization → DDS → 
      → Deserialization → Msg Conversion → Application
```

- **Performance:** 7.19 ms mean, 14.9 ms peaks (high jitter)
- **Bottleneck:** 6 MB serialization/deserialization saturates system resources
- **Verdict:** Inadequate for real-time HD video

### Loaned (Zero-Copy DDS)

**Data Path:**
```
Sensor → Buffer Copy → Loaned Memory → DDS SHM → Application
```

- **Performance:** 3.10 ms mean (50% improvement over standard)
- **Bottleneck:** Application must copy frame to loaned buffer (memcpy of 6 MB)
- **DDS Transport:** Only 0.28 ms (shared memory is fast)
- **Verdict:** Good but limited by API constraints

### SIM with Fallback

**Data Path:**
```
Sensor → POSIX SHM (primary) 
      → Zero-Copy (fallback 1) 
      → Standard (fallback 2)
```

- **Performance:** 1.03 ms mean
- **Feature:** Runtime fallback for robustness
- **Overhead:** Fallback logic adds ~0.35 ms
- **Verdict:** Robust with good performance

### SIM Pure (Maximum Performance)

**Data Path:**
```
Sensor → POSIX SHM → Application
```

- **Performance:** 0.68 ms mean, 0.02 ms jitter
- **Feature:** No fallback, no ROS message abstraction for payload
- **Verdict:** Optimal for controlled environments requiring minimum latency

---

## Comparison with Literature

Reference: arXiv:2510.11448 - "A Faster and More Reliable Middleware for Autonomous Driving Systems"

| Aspect | Paper (SIM Hybrid) | Our SIM Pure |
|--------|-------------------|--------------|
| Fallback | 3-tier runtime | None |
| Latency | ~0.2 ms (LiDAR) | 0.68 ms (1080p) |
| Robustness | High | Moderate |
| Determinism | Moderate | High |

**Note:** Our higher latency is expected due to larger payload (6 MB vs ~50 KB for LiDAR).

---

## Recommendations

### For Hard Real-Time Video (< 1 ms requirement)

**Use SIM Pure.** Sub-millisecond latency (0.68 ms) on 1080p frames is industrial-grade performance, maximizing time budget for vision algorithms.

### For Robust Systems

**Use SIM with Fallback.** The 3-tier fallback (SIM → Zero-Copy → Standard) ensures communication never fails, at the cost of ~0.35 ms additional latency.

### For Standard ROS 2 Integration

**Use Loaned Nodes.** 50% improvement over standard with no custom code required, but limited by memcpy overhead.

### Future Optimization

To achieve SIM-level performance with Loaned API, develop a custom camera driver that writes directly to the loaned buffer pointer, eliminating the 2.82 ms memcpy overhead.

---

## Monitoring Note

Fast DDS Monitor cannot observe SIM traffic (it bypasses DDS). For SIM monitoring, use:
- Application-level statistics (built into SIM nodes)
- `/dev/shm/sim_*` file monitoring
- Custom health topic (lightweight)
