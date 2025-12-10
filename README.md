# ROS2 SHARED MEMORY VISION

High-Bandwidth Video Transport for ROS 2

> **Note:** This is an independent implementation by Nabil Achir based on published research. It is NOT officially affiliated with or endorsed by the original paper authors (He & Shi, University of Delaware). See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) for full details.

---

## Overview

Ultra-low-latency video streaming (1080p @ 30 FPS) using multiple transport architectures.

---

## Performance Results

| Transport | Mean Latency | Improvement | Use Case |
|-----------|-------------|-------------|----------|
| **Standard** | 7.19 ms | Baseline | Network transport |
| **Loaned (Zero-Copy DDS)** | 3.10 ms | 57% faster | ROS 2 compatibility |
| **SIM with Fallback** | 1.03 ms | 86% faster | Production systems |
| **SIM Pure** | 0.68 ms | **91% faster** | Hard real-time |

---

## Demo

### Standard Transport (~7 ms)

![Standard Transport Demo](ros2_shm_vision/assets/Standard.gif)

### Loaned Zero-Copy (~3 ms)

![Loaned Zero-Copy Demo](ros2_shm_vision/assets/Loaned.gif)

### SIM with Fallback (~1 ms)

![SIM with Fallback Demo](ros2_shm_vision/assets/SIM.gif)

### SIM Pure (~0.7 ms)

![SIM Pure Demo](ros2_shm_vision/assets/SIM-nofb.gif)

---

## Quick Start

### Prerequisites

```bash
sudo apt install libopencv-dev ros-humble-sensor-msgs
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

### Add Video

Place a 1080p video file in:
```
ros2_shm_vision/video/sample.mp4
```

### Environment Setup (for Zero-Copy modes)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros2_shm_vision)/share/ros2_shm_vision/config/fastdds_setup.xml
```

### Run (choose one pair)

| Mode | Subscriber | Publisher |
|------|------------|-----------|
| Standard | `ros2 run ros2_shm_vision standard_subscriber_1080p` | `ros2 run ros2_shm_vision standard_publisher_1080p` |
| Loaned | `ros2 run ros2_shm_vision loaned_subscriber_1080p` | `ros2 run ros2_shm_vision loaned_publisher_1080p` |
| SIM | `ros2 run ros2_shm_vision sim_subscriber_1080p` | `ros2 run ros2_shm_vision sim_publisher_1080p` |
| SIM Pure | `ros2 run ros2_shm_vision sim_subscriber_nofb_1080p` | `ros2 run ros2_shm_vision sim_publisher_nofb_1080p` |

---

## Video Overlay Colors

| Color | Mode | Latency |
|-------|------|---------|
| Red | Standard | ~7 ms |
| Green | Zero-Copy | ~3 ms |
| Magenta | SIM Fallback | ~1 ms |
| Yellow | SIM Pure | ~0.7 ms |

---

## Documentation

### Technical Docs

| Document | Description |
|----------|-------------|
| [Performance Analysis](ros2_shm_vision/docs/performance_analysis.md) | Detailed latency measurements and analysis |
| [Architecture Overview](ros2_shm_vision/docs/architecture_overview.md) | System design and data flow diagrams |
| [FastDDS Monitor Guide](ros2_shm_vision/docs/fastdds_monitor_guide.md) | Monitoring and debugging guide |
| [Troubleshooting](ros2_shm_vision/docs/troubleshooting.md) | Common issues and solutions |

### Node Guides

| Guide | Description |
|-------|-------------|
| [Standard Nodes](ros2_shm_vision/guides/standard_nodes/README.md) | sensor_msgs/Image transport |
| [Loaned Nodes](ros2_shm_vision/guides/loaned_nodes/README.md) | Zero-copy DDS with loaned messages |
| [SIM Nodes](ros2_shm_vision/guides/sim_nodes/README.md) | SIM with fallback chain |
| [SIM Pure Nodes](ros2_shm_vision/guides/sim_pure_nodes/README.md) | Maximum performance SIM |

### Reusable Libraries

| Library | Description |
|---------|-------------|
| [SIM Library](ros2_shm_vision/sim_library/README.md) | Standalone shared memory transport library |

---

## Package Structure

```
ros2_shm_vision/
├── README.md                 # Package docs
├── ACKNOWLEDGMENTS.md        # Credits and citations
├── assets/                   # Demo GIFs
├── docs/                     # Technical documentation
├── guides/                   # Per-node-type guides
├── sim_library/              # Reusable SIM library (copy-able)
├── config/                   # FastDDS configuration files
├── msg/                      # ROS message definitions
├── include/                  # Package headers
├── src/                      # Node implementations
└── video/                    # Test video files
```

---

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| Resolution | 1920 x 1080 |
| Color format | RGB (3 channels) |
| Frame size | 6,220,800 bytes (~6 MB) |
| Frame rate | 30 FPS |
| Throughput | ~180 MB/s |

---

## Architecture Comparison

```
Standard:   Sensor → Serialize → DDS → Deserialize → App     (7.19 ms)
Loaned:     Sensor → memcpy → DDS SHM → App                  (3.10 ms)
SIM:        Sensor → POSIX SHM (double buffer) → App         (0.68 ms)
```

### Why SIM is Faster

- No serialization/deserialization
- No DDS discovery overhead
- Lock-free double buffering
- Direct memory access

---

## Acknowledgments

This is an independent implementation based on the research paper:

**"A Faster and More Reliable Middleware for Autonomous Driving Systems"**
- Authors: Yuankai He, Weisong Shi
- Institution: University of Delaware
- arXiv: [2510.11448](https://arxiv.org/abs/2510.11448)

See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) for full citation and disclaimer.

---

## Author

**Nabil Achir**
- GitHub: https://github.com/nabilachir-dev

---

## License

MIT License - Copyright (c) 2025 Nabil Achir
