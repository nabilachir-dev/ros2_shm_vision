# Standard Nodes Guide

Standard transport using `sensor_msgs/Image` with full serialization.

---

## Overview

Standard nodes use the traditional ROS 2 message transport:
- Message type: `sensor_msgs/Image`
- Transport: DDS with serialization/deserialization
- Latency: ~7 ms (highest)

This is the baseline for comparison with zero-copy transports.

---

## Performance

| Metric | Value |
|--------|-------|
| Mean Latency | 7.19 ms |
| Min Latency | 6.07 ms |
| Max Latency | 14.90 ms |
| Jitter | ~1.81 ms |
| Throughput | ~180 MB/s |

---

## When to Use

- Network transport between different computers
- Variable message sizes
- Maximum compatibility required
- Learning/development purposes

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
ros2 run ros2_shm_vision standard_subscriber_1080p
```

### Terminal 2 - Publisher

```bash
ros2 run ros2_shm_vision standard_publisher_1080p
```

---

## Video Display

The subscriber displays video with a **red** overlay showing:
- Transport mode: "STANDARD"
- Current FPS
- Latency in milliseconds

---

## Source Files

| File | Description |
|------|-------------|
| `src/standard_publisher_1080p.cpp` | Reads video, publishes sensor_msgs/Image |
| `src/standard_subscriber_1080p.cpp` | Receives images, displays with OpenCV |

---

## Key Code Snippets

### Publisher

```cpp
auto msg = sensor_msgs::msg::Image();
msg.header.stamp = this->now();
msg.width = 1920;
msg.height = 1080;
msg.encoding = "rgb8";
msg.step = 1920 * 3;
msg.data.assign(frame.data, frame.data + IMAGE_SIZE);

publisher_->publish(msg);
```

### Subscriber

```cpp
void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame(msg->height, msg->width, CV_8UC3, 
                  const_cast<uint8_t*>(msg->data.data()));
    cv::imshow("Standard 1080p Stream", frame);
}
```

---

## Notes

- Serialization overhead is significant for large images
- Each frame is copied multiple times through DDS
- Use for compatibility, not for minimum latency
