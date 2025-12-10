# Architecture Overview

System architecture for high-bandwidth video transport in ROS 2.

---

## Transport Modes

This package provides four transport architectures:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Transport Latency Comparison                     │
├──────────────────┬──────────────┬──────────────────┬────────────────────┤
│     Standard     │    Loaned    │   SIM Fallback   │     SIM Pure       │
│     7.19 ms      │    3.10 ms   │     1.03 ms      │     0.68 ms        │
│   ████████████   │  ██████      │    ██            │     █              │
└──────────────────┴──────────────┴──────────────────┴────────────────────┘
```

---

## Data Flow Diagrams

### Standard Transport

```
┌────────────┐    ┌─────────────┐    ┌───────────────┐    ┌────────────┐
│   Camera   │───>│ ROS Message │───>│ Serialization │───>│    DDS     │
│   Driver   │    │  Conversion │    │   (CDR)       │    │  Transport │
└────────────┘    └─────────────┘    └───────────────┘    └─────┬──────┘
                                                                │
                                                                v
┌────────────┐    ┌─────────────┐    ┌───────────────┐    ┌────────────┐
│Application │<───│ ROS Message │<───│Deserialization│<───│    DDS     │
│            │    │  Conversion │    │               │    │   Receive  │
└────────────┘    └─────────────┘    └───────────────┘    └────────────┘

Copies: 2-3 per frame
Latency: ~7 ms
```

### Loaned (Zero-Copy DDS)

```
┌────────────┐    ┌─────────────┐    ┌───────────────────────────────────┐
│   Camera   │───>│   memcpy    │───>│     DDS Loaned Memory (SHM)       │
│   Driver   │    │  (6 MB)     │    │         (shared memory)           │
└────────────┘    └─────────────┘    └─────────────────┬─────────────────┘
                                                       │ pointer only
                                                       v
┌────────────┐                       ┌───────────────────────────────────┐
│Application │<──────────────────────│     DDS Loaned Memory (SHM)       │
│            │      direct read      │                                   │
└────────────┘                       └───────────────────────────────────┘

Copies: 1 (into loaned buffer)
Latency: ~3 ms
```

### SIM Transport

```
┌────────────┐         ┌──────────────────────────────────────────────────┐
│   Camera   │────────>│            POSIX Shared Memory                   │
│   Driver   │  write  │  ┌────────┐  ┌─────────┐  ┌─────────┐            │
└────────────┘         │  │ Header │  │Buffer 0 │  │Buffer 1 │            │
                       │  │front_idx│  │ (6 MB)  │  │ (6 MB)  │           │
                       │  └────────┘  └─────────┘  └─────────┘            │
                       └───────────────────────────┬──────────────────────┘
                                                   │ direct read
                                                   v
                       ┌────────────────────────────────────────────────┐
                       │                 Application                    │
                       │     (reads from active buffer via front_idx)   │
                       └────────────────────────────────────────────────┘

Copies: 1 (into SHM, reader copies out)
Latency: ~0.7 ms
```

---

## SIM Fallback Chain

```
┌────────────────────────────────────────────────────────────────────────┐
│                         SIM Fallback Chain                             │
│                                                                        │
│   ┌─────────┐         ┌─────────────┐         ┌──────────────┐         │
│   │   SIM   │──fail──>│  Zero-Copy  │──fail──>│   Standard   │         │
│   │ 0.7 ms  │         │   3.1 ms    │         │    7.2 ms    │         │
│   └─────────┘         └─────────────┘         └──────────────┘         │
│       │                     │                        │                 │
│       │                     │                        │                 │
│   Best perf            Good perf              Always works             │
│   SHM required         SHM required           Network OK               │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Node Types Summary

| Node Type | Publisher | Subscriber | Transport |
|-----------|-----------|------------|-----------|
| Standard | `standard_publisher_1080p` | `standard_subscriber_1080p` | sensor_msgs/Image |
| Loaned | `loaned_publisher_1080p` | `loaned_subscriber_1080p` | Image1080p (POD) |
| SIM Fallback | `sim_publisher_1080p` | `sim_subscriber_1080p` | SIM + ZC + Std |
| SIM Pure | `sim_publisher_nofb_1080p` | `sim_subscriber_nofb_1080p` | SIM only |

---

## Message Types

### sensor_msgs/Image (Standard)

```
std_msgs/Header header
uint32 height
uint32 width  
string encoding
uint8 is_bigendian
uint32 step
uint8[] data          # Variable size, requires serialization
```

### Image1080p.msg (Zero-Copy POD)

```
int32 timestamp_sec
uint32 timestamp_nanosec
uint32 frame_id
uint32 width
uint32 height
uint8[6220800] data   # Fixed size: 1920*1080*3 = 6,220,800 bytes
```

### SIM Header (Custom Shared Memory)

```cpp
struct Header {
    uint32_t magic;                    // Validation
    uint32_t version;                  // Protocol version
    size_t capacity;                   // Max buffer size
    std::atomic<uint32_t> front_idx;   // Active buffer (0 or 1)
    std::atomic<uint64_t> frame[2];    // Frame numbers
    std::atomic<int64_t> timestamp_ns[2]; // Timestamps
    std::atomic<size_t> published_length; // Current frame size
    std::atomic<int64_t> writer_heartbeat_ns; // Liveness
    std::atomic<uint32_t> checksum[2]; // Optional integrity
    std::atomic<bool> checksum_enabled;
};
```

---

## Environment Variables

Required for Fast DDS shared memory:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros2_shm_vision)/share/ros2_shm_vision/config/fastdds_setup.xml
```

---

## Quick Reference

| What to Use | When to Use It |
|-------------|----------------|
| **SIM Pure** | Minimum latency required, controlled environment |
| **SIM Fallback** | Production systems needing robustness |
| **Loaned** | Standard ROS 2 compatibility, 50% improvement OK |
| **Standard** | Network transport, variable message sizes |
