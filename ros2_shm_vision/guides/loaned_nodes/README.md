# Loaned Nodes Guide (Zero-Copy DDS)

Zero-copy transport using ROS 2 loaned messages with Fast DDS shared memory.

---

## Overview

Loaned nodes use the ROS 2 zero-copy API:
- Message type: `Image1080p.msg` (fixed-size POD)
- Transport: Fast DDS shared memory
- Latency: ~3 ms (57% reduction from standard)

---

## Performance

| Metric | Value |
|--------|-------|
| Mean Latency | 3.10 ms |
| Min Latency | 2.51 ms |
| Max Latency | 5.25 ms |
| Jitter | ~0.38 ms |
| Improvement | 57% vs Standard |

---

## Prerequisites

### Fixed-Size Message

Zero-copy requires a fixed-size message. The `Image1080p.msg` is:

```
int32 timestamp_sec
uint32 timestamp_nanosec
uint32 frame_id
uint32 width
uint32 height
uint8[6220800] data   # Fixed: 1920*1080*3
```

### FastDDS Configuration

File: `config/fastdds_setup.xml`

```xml
<data_sharing>
    <kind>AUTOMATIC</kind>
</data_sharing>
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
```

---

## When to Use

- Intra-host communication (same computer)
- Standard ROS 2 API compatibility needed
- 50% latency reduction acceptable
- No custom shared memory code desired

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_shm_vision
source install/setup.bash
```

---

## Environment Setup

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGE=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros2_shm_vision)/share/ros2_shm_vision/config/fastdds_setup.xml
```

---

## Run

### Terminal 1 - Subscriber

```bash
ros2 run ros2_shm_vision loaned_subscriber_1080p
```

### Terminal 2 - Publisher

```bash
ros2 run ros2_shm_vision loaned_publisher_1080p
```

---

## Video Display

The subscriber displays video with a **green** overlay showing:
- Transport mode: "ZERO-COPY"
- Current FPS
- Latency in milliseconds

---

## Source Files

| File | Description |
|------|-------------|
| `src/loaned_publisher_1080p.cpp` | Uses `borrow_loaned_message()` API |
| `src/loaned_subscriber_1080p.cpp` | Receives zero-copy messages |
| `msg/Image1080p.msg` | Fixed-size POD message |
| `config/fastdds_setup.xml` | DDS data sharing config |

---

## Key Code Snippets

### Publisher (Zero-Copy)

```cpp
if (publisher_->can_loan_messages()) {
    auto loaned_msg = publisher_->borrow_loaned_message();
    
    // Fill message data
    loaned_msg.get().frame_id = frame_count;
    std::memcpy(loaned_msg.get().data.data(), frame.data, IMAGE_SIZE);
    
    publisher_->publish(std::move(loaned_msg));
}
```

### Subscriber

```cpp
void callback(const Image1080p::SharedPtr msg) {
    // Direct access to shared memory
    cv::Mat frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3,
                  const_cast<uint8_t*>(msg->data.data()));
    cv::imshow("Zero-Copy Stream", frame);
}
```

---

## Verify Zero-Copy Active

Check for Fast DDS shared memory files:

```bash
ls -la /dev/shm/fastrtps*
```

Expected output:
```
-rw-r--r-- 1 user user 6553600 Dec 10 12:00 fastrtps_segment_xxx
```

---

## Limitations

- Requires fixed-size messages
- Application must still copy data into loaned buffer (~2.8 ms overhead)
- Both nodes must be on same host
- Shared memory availability depends on system configuration
