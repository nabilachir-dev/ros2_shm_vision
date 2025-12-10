/**
 * @file standard_subscriber_1080p.cpp
 * @brief Subscriber Standard per immagini 1080p (CON COPIE)
 * 
 * IT: Riceve frame 1080p via sensor_msgs/Image e li visualizza.
 * EN: Receives 1080p frames via sensor_msgs/Image and displays them.
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;

/**
 * @class StandardSubscriber1080p
 * @brief Subscriber standard con sensor_msgs/Image (CON COPIE)
 */
class StandardSubscriber1080p : public rclcpp::Node
{
public:
    StandardSubscriber1080p()
    : Node("standard_subscriber_1080p"), 
      frame_count_(0),
      show_video_(true)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_1080p_standard",
            10,
            std::bind(&StandardSubscriber1080p::frame_callback, this, std::placeholders::_1));
        
        RCLCPP_WARN(this->get_logger(),
            "[IT] Questo subscriber USA COPIE - NON è zero-copy!");
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Subscriber STANDARD 1080p avviato. In ascolto su 'image_1080p_standard'.");
        
        if (show_video_) {
            cv::namedWindow("Standard 1080p Stream", cv::WINDOW_AUTOSIZE);
        }
        
        last_time_ = this->now();
    }
    
    ~StandardSubscriber1080p() {
        cv::destroyAllWindows();
    }

private:
    void frame_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto now = this->now();
        ++frame_count_;
        
        // Calcola latenza
        int64_t msg_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
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
        if (show_video_ && !msg->data.empty()) {
            cv::Mat frame_rgb(msg->height, msg->width, CV_8UC3, 
                             const_cast<uint8_t*>(msg->data.data()));
            cv::Mat frame_bgr;
            cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
            
            // Info su frame (con latenza come nei loaned)
            std::string info = "Frame: " + std::to_string(frame_count_) + 
                              " | Latency: " + std::to_string(latency_ms).substr(0,5) + " ms | STANDARD";
            cv::putText(frame_bgr, info, cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            
            cv::imshow("Standard 1080p Stream", frame_bgr);
            
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                rclcpp::shutdown();
            }
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    uint32_t frame_count_;
    uint32_t frames_in_second_ = 0;
    rclcpp::Time last_time_;
    bool show_video_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandardSubscriber1080p>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
