/**
 * @file standard_publisher_1080p.cpp
 * @brief Publisher Standard per immagini 1080p a 30 FPS (CON COPIE)
 * 
 * IT: Legge frame da file video e li pubblica usando sensor_msgs/Image.
 *     Questo metodo COPIA i dati (non è zero-copy).
 * 
 * EN: Reads frames from video file and publishes using sensor_msgs/Image.
 *     This method COPIES data (not zero-copy).
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

/**
 * @class StandardPublisher1080p
 * @brief Publisher standard con sensor_msgs/Image (CON COPIE)
 */
class StandardPublisher1080p : public rclcpp::Node
{
public:
    StandardPublisher1080p()
    : Node("standard_publisher_1080p"), frame_count_(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Cerca il file video
        video_path_ = find_video_file();
        
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                "[IT] Nessun file video trovato! Metti un video .mp4 in:");
            RCLCPP_ERROR(this->get_logger(), 
                "     ~/ros2_ws/src/ros2_shm_vision/video/");
            return;
        }
        
        // Apri il video
        video_capture_.open(video_path_);
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), 
                "[IT] Impossibile aprire il video: %s", video_path_.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Video caricato: %s", video_path_.c_str());
        
        // Crea publisher con sensor_msgs/Image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_1080p_standard", 10);
        
        RCLCPP_WARN(this->get_logger(),
            "[IT] Questo publisher USA COPIE - NON è zero-copy!");
        RCLCPP_WARN(this->get_logger(),
            "[EN] This publisher USES COPIES - NOT zero-copy!");
        
        // Timer a ~30 fps
        timer_ = this->create_wall_timer(
            33ms, std::bind(&StandardPublisher1080p::publish_frame, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Publisher STANDARD 1080p avviato. 30 FPS su 'image_1080p_standard'.");
    }

private:
    std::string find_video_file()
    {
        std::string video_dir = "/home/oe/ros2_ws/src/ros2_shm_vision/video/";
        cv::String pattern = video_dir + "*.mp4";
        std::vector<cv::String> files;
        cv::glob(pattern, files, false);
        
        if (!files.empty()) {
            return files[0];
        }
        return "";
    }
    
    void publish_frame()
    {
        if (!video_capture_.isOpened()) {
            return;
        }
        
        cv::Mat frame;
        
        if (!video_capture_.read(frame)) {
            video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!video_capture_.read(frame)) {
                return;
            }
        }
        
        // Ridimensiona a 1080p se necessario
        if (frame.cols != IMAGE_WIDTH || frame.rows != IMAGE_HEIGHT) {
            cv::resize(frame, frame, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
        }
        
        // Converti BGR → RGB
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        // Crea messaggio sensor_msgs/Image (NON POD - ha stringhe!)
        auto msg = sensor_msgs::msg::Image();
        
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera";  // STRINGA! Non POD!
        
        msg.width = IMAGE_WIDTH;
        msg.height = IMAGE_HEIGHT;
        msg.encoding = "rgb8";           // STRINGA! Non POD!
        msg.is_bigendian = false;
        msg.step = IMAGE_WIDTH * 3;
        
        // COPIA 1: da cv::Mat a vector
        msg.data.assign(frame.data, frame.data + IMAGE_SIZE);
        
        RCLCPP_INFO(this->get_logger(),
            "[IT] Frame #%u (COPIA) | %dx%d | ~6 MB copiati",
            frame_count_, IMAGE_WIDTH, IMAGE_HEIGHT);
        
        // COPIA 2: publish copia nel middleware
        publisher_->publish(msg);
        
        ++frame_count_;
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    std::string video_path_;
    uint32_t frame_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandardPublisher1080p>());
    rclcpp::shutdown();
    return 0;
}
