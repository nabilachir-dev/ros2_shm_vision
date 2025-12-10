# SIM Nodes Guide (with Fallback)

SIM transport with automatic fallback chain: SIM -> Zero-Copy -> Standard.

---

## Overview

SIM (Sensor-in-Memory) with fallback provides robust high-performance transport:
- Primary: POSIX shared memory (SIM)
- Fallback 1: ROS 2 loaned messages (Zero-Copy)
- Fallback 2: Standard sensor_msgs/Image
- Latency: ~1 ms (86% reduction from standard)

---

## Performance

| Metric | Value |
|--------|-------|
| Mean Latency | 1.03 ms |
| Min Latency | 0.67 ms |
| Max Latency | 1.74 ms |
| Jitter | ~0.15 ms |
| Improvement | 86% vs Standard |

---

## Fallback Chain

```
┌─────────────┐     fail     ┌─────────────┐     fail     ┌─────────────┐
│     SIM     │─────────────>│  Zero-Copy  │─────────────>│  Standard   │
│   ~1.0 ms   │              │   ~3.1 ms   │              │   ~7.2 ms   │
└─────────────┘              └─────────────┘              └─────────────┘
```

---

## When to Use

- Production systems requiring robustness
- Environments where SHM might be unavailable
- Need automatic recovery from transport failures
- Acceptable ~0.35 ms fallback overhead

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

---

## Environment Setup (for fallback modes)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros2_shm_vision)/share/ros2_shm_vision/config/fastdds_sim_fallback.xml
```

---

## Run

### Terminal 1 - Subscriber (starts first, waits for SIM)

```bash
ros2 run ros2_shm_vision sim_subscriber_1080p
```

### Terminal 2 - Publisher

```bash
ros2 run ros2_shm_vision sim_publisher_1080p
```

---

## Parameters

### Publisher

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_checksum` | false | Enable CRC32 checksum |
| `sim_shm_name` | /sim_image_1080p | Shared memory name |

Example with checksum:
```bash
ros2 run ros2_shm_vision sim_publisher_1080p --ros-args -p enable_checksum:=true
```

### Subscriber

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_shm_name` | /sim_image_1080p | Shared memory name |
| `show_video` | true | Display video window |
| `sim_timeout_ms` | 5000 | Timeout before fallback |

---

## Video Display

The subscriber displays video with colored overlay:
- **Magenta** = SIM active (~1 ms)
- **Green** = Zero-Copy fallback (~3 ms)
- **Red** = Standard fallback (~7 ms)

---

## Source Files

| File | Description |
|------|-------------|
| `src/sim_publisher_1080p.cpp` | SIM writer with fallback logic |
| `src/sim_subscriber_1080p.cpp` | SIM reader with fallback logic |
| `include/sim_transport.hpp` | SIM library header |
| `src/sim_transport.cpp` | SIM library implementation |
| `config/fastdds_sim_fallback.xml` | DDS config with AUTOMATIC |

---

## Verify SIM Active

Check for SIM shared memory:

```bash
ls -la /dev/shm/sim_*
```

Expected output:
```
-rw-rw-r-- 1 user user 12441728 Dec 10 12:00 /dev/shm/sim_image_1080p
```

---

## Log Output Examples

### SIM Mode (Success)

```
[SIM] SIM inizializzato! Checksum: OFF
[SIM] FPS: 30.0 | Latenza: 1.023 ms | Dropped: 0
```

### Fallback to Zero-Copy

```
[SIM] SIM fallito, provo Zero-Copy...
[SIM] Zero-Copy abilitato!
[ZC] FPS: 30.0 | Latenza: 3.10 ms
```

### Fallback to Standard

```
[SIM] Zero-Copy fallito, uso Standard...
[STD] FPS: 30.0 | Latenza: 7.19 ms
```

---

## Notes

- Fallback logic adds ~0.35 ms overhead compared to SIM Pure
- Recommended for production systems
- Checksum is optional (disabled by default for performance)
