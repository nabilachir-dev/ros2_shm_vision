# SIM Pure Nodes Guide (Maximum Performance)

SIM transport without fallback for minimum possible latency.

---

## Overview

SIM Pure uses direct POSIX shared memory with no fallback logic:
- Transport: POSIX shared memory only
- No ROS 2 message passing for payload
- Latency: ~0.68 ms (91% reduction from standard)

This is the fastest transport option available.

---

## Performance

| Metric | Value |
|--------|-------|
| Mean Latency | 0.68 ms |
| Min Latency | 0.64 ms |
| Max Latency | 0.99 ms |
| Jitter | ~0.02 ms |
| Improvement | 91% vs Standard |

---

## Comparison

| Mode | Latency | Improvement |
|------|---------|-------------|
| Standard | 7.19 ms | Baseline |
| Loaned | 3.10 ms | 57% |
| SIM Fallback | 1.03 ms | 86% |
| **SIM Pure** | **0.68 ms** | **91%** |

---

## When to Use

- Hard real-time requirements (< 1 ms latency)
- Controlled environment (same computer guaranteed)
- Vision pipeline where every millisecond matters
- Development/testing of SIM library

---

## When NOT to Use

- Network communication required
- Shared memory might be unavailable
- System robustness is priority over performance

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

---

## Run

### Terminal 1 - Subscriber

```bash
ros2 run ros2_shm_vision sim_subscriber_nofb_1080p
```

### Terminal 2 - Publisher

```bash
ros2 run ros2_shm_vision sim_publisher_nofb_1080p
```

---

## Parameters

### Subscriber

| Parameter | Default | Description |
|-----------|---------|-------------|
| `show_video` | true | Display video window |

---

## Video Display

The subscriber displays video with a **yellow** overlay showing:
- Transport mode: "SIM-PURE"
- Latency (sub-millisecond)

---

## Source Files

| File | Description |
|------|-------------|
| `src/sim_publisher_nofb_1080p.cpp` | Minimal SIM writer |
| `src/sim_subscriber_nofb_1080p.cpp` | Minimal SIM reader with polling |
| `include/sim_transport.hpp` | SIM library header |
| `src/sim_transport.cpp` | SIM library implementation |

---

## Key Differences from SIM with Fallback

| Aspect | SIM Fallback | SIM Pure |
|--------|--------------|----------|
| Fallback | Yes (ZC -> Std) | None |
| Overhead | ~0.35 ms added | Minimal |
| Robustness | High | Moderate |
| Dependencies | ROS 2 msgs | SIM library only |
| Use Case | Production | Performance testing |

---

## Verify SIM Active

Check for SIM Pure shared memory:

```bash
ls -la /dev/shm/sim_pure_*
```

Expected output:
```
-rw-rw-r-- 1 user user 12441728 Dec 10 12:00 /dev/shm/sim_pure_1080p
```

---

## Architecture

```
┌────────────┐     direct write     ┌─────────────────────┐
│  Publisher │─────────────────────>│  POSIX Shared Memory│
│            │                      │  /dev/shm/sim_pure  │
└────────────┘                      │  +─────────────────+│
                                    │  │ Double Buffer   ││
                                    │  │ Lock-free       ││
                                    │  +─────────────────+│
                                    └──────────┬──────────┘
                                               │ polling read
                                               v
                                    ┌─────────────────────┐
                                    │     Subscriber      │
                                    │  (high-speed loop)  │
                                    └─────────────────────┘
```

---

## Notes

- No checksum by default (maximum performance)
- Subscriber uses high-speed polling with `std::this_thread::yield()`
- If SIM is unavailable, the node will not start (no fallback)
- Ideal for benchmarking and latency-critical applications
