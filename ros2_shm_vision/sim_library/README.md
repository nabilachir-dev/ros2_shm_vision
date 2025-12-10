# SIM Library - Sensor-in-Memory Transport

A lightweight, lock-free shared memory transport library for high-bandwidth real-time applications.

> **Note:** This is an independent implementation by Nabil Achir based on published research. It is NOT officially affiliated with or endorsed by the original paper authors (He & Shi, University of Delaware).

---

## Overview

SIM (Sensor-in-Memory) is a C++ library that provides ultra-low-latency data transport using POSIX shared memory. Designed for autonomous vehicles and robotics, it achieves sub-millisecond latency for 1080p video frames.

### Key Features

- **Lock-free double buffering** - No mutex contention
- **Native memory layouts** - No serialization overhead
- **POSIX shared memory** - Standard Linux API
- **Optional checksum** - CRC32 integrity verification
- **Simple API** - 5 function calls to integrate

### Performance

| Payload | Latency | Jitter |
|---------|---------|--------|
| 6 MB (1080p) | 0.68 ms | 0.02 ms |
| 50 KB (LiDAR) | ~0.2 ms | ~0.01 ms |

---

## Quick Start

### Writer

```cpp
#include "sim_transport.hpp"

int main() {
    SIM::Writer writer("/my_sensor", 1920*1080*3);
    if (!writer.init()) return 1;
    
    while (running) {
        uint8_t* data = captureFrame();
        writer.write(data, frameSize);
    }
    
    writer.destroy();
    return 0;
}
```

### Reader

```cpp
#include "sim_transport.hpp"

int main() {
    SIM::Reader reader("/my_sensor", 1920*1080*3);
    if (!reader.init()) return 1;
    
    std::vector<uint8_t> buffer(1920*1080*3);
    while (running) {
        size_t size;
        if (reader.read(buffer.data(), size)) {
            processFrame(buffer.data(), size);
        }
    }
    return 0;
}
```

---

## API Reference

### SIM::Writer

| Method | Description |
|--------|-------------|
| `Writer(shm_name, max_size, enable_checksum)` | Constructor |
| `bool init()` | Initialize shared memory |
| `bool write(data, size)` | Write data to buffer |
| `bool isReady()` | Check if initialized |
| `void destroy()` | Close and unlink shm |
| `void setChecksumEnabled(bool)` | Enable/disable checksum |
| `uint64_t getFrameCount()` | Get frames written |

### SIM::Reader

| Method | Description |
|--------|-------------|
| `Reader(shm_name, max_size)` | Constructor |
| `bool init()` | Connect to shared memory |
| `bool read(data, size)` | Read new frame if available |
| `bool readWithTimeout(data, size, timeout_ms)` | Read with timeout |
| `bool isReady()` | Check if connected |
| `bool isWriterAlive(timeout_ms)` | Check writer liveness |
| `uint64_t getDroppedFrames()` | Get skipped frame count |
| `bool verifyLastChecksum()` | Verify last frame checksum |

---

## CMake Integration

```cmake
add_library(sim_transport SHARED sim_transport.cpp)
target_include_directories(sim_transport PUBLIC include)
target_link_libraries(sim_transport rt pthread)
target_link_libraries(my_target sim_transport)
```

---

## Author & License

**Author:** Nabil Achir
- GitHub: https://github.com/nabilachir-dev

**License:** MIT - Copyright (c) 2025 Nabil Achir

---

## CITATION REQUIREMENT

**If you use this library, you MUST cite:**

### 1. Library Author

```bibtex
@software{achir2025sim,
  author = {Achir, Nabil},
  title = {SIM Library - Sensor-in-Memory Transport},
  year = {2025},
  url = {https://github.com/nabilachir-dev/sim_library}
}
```

### 2. Original Research Paper

```bibtex
@article{he2025sim,
  title={A Faster and More Reliable Middleware for Autonomous Driving Systems},
  author={He, Yuankai and Shi, Weisong},
  journal={arXiv preprint arXiv:2510.11448},
  year={2025}
}
```

> **DISCLAIMER:** This is an independent implementation, NOT affiliated with the original researchers.
