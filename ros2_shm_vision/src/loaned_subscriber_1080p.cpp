/**
 * @file loaned_subscriber_1080p.cpp
 * @brief Subscriber Zero-Copy per immagini 1080p a 30 FPS
 * 
 * IT: Riceve frame 1080p via shared memory e li visualizza.
 * EN: Receives 1080p frames via shared memory and displays them.
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ros2_shm_vision/msg/image1080p.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;

/**
 * @class LoanedSubscriber1080p
 * @brief Subscriber zero-copy per streaming video 1080p
 */
class LoanedSubscriber1080p : public rclcpp::Node
{
public:
    LoanedSubscriber1080p()
    : Node("loaned_subscriber_1080p"), 
      frame_count_(0),
      show_video_(true)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        
        subscription_ = this->create_subscription<ros2_shm_vision::msg::Image1080p>(
            "image_1080p",
            qos,
            std::bind(&LoanedSubscriber1080p::frame_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber LOANED 1080p avviato. In ascolto su 'image_1080p'.");
        
        if (show_video_) {
            cv::namedWindow("Zero-Copy 1080p Stream", cv::WINDOW_AUTOSIZE);
        }
        
        last_time_ = this->now();
    }
    
    ~LoanedSubscriber1080p() {
        cv::destroyAllWindows();
    }

private:
    void frame_callback(const ros2_shm_vision::msg::Image1080p::SharedPtr msg)
    {
        auto now = this->now();
        ++frame_count_;
        
        // Calcola latenza
        int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL + 
                         static_cast<int64_t>(msg->timestamp_nanosec);
        int64_t now_ns = now.nanoseconds();
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        // Calcola FPS ogni secondo
        double elapsed = (now - last_time_).seconds();
        if (elapsed >= 1.0) {
            double fps = static_cast<double>(frames_in_second_) / elapsed;
            RCLCPP_INFO(this->get_logger(),
                "[IT] FPS: %.1f | Latenza: %.2f ms | Frame: %u",
                fps, latency_ms, frame_count_);
            frames_in_second_ = 0;
            last_time_ = now;
        }
        ++frames_in_second_;
        
        // Visualizza frame
        if (show_video_) {
            cv::Mat frame_rgb(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, 
                             const_cast<uint8_t*>(msg->data.data()));
            cv::Mat frame_bgr;
            cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
            
            // Info su frame
            std::string info = "Frame: " + std::to_string(frame_count_) + 
                              " | Latency: " + std::to_string(latency_ms).substr(0,5) + " ms";
            cv::putText(frame_bgr, info, cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow("Zero-Copy 1080p Stream", frame_bgr);
            
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                rclcpp::shutdown();
            }
        }
    }
    
    rclcpp::Subscription<ros2_shm_vision::msg::Image1080p>::SharedPtr subscription_;
    uint32_t frame_count_;
    uint32_t frames_in_second_ = 0;
    rclcpp::Time last_time_;
    bool show_video_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanedSubscriber1080p>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
