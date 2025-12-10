/**
 * @file sim_publisher_nofb_1080p.cpp
 * @brief Publisher SIM PURO - Nessun fallback, massima velocita
 * 
 * Questo publisher usa SOLO SIM, senza fallback a Zero-Copy o Standard.
 * E' la modalita piu veloce possibile (~0.2ms latenza).
 */

#include <chrono>
#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sim_transport.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

// Nome shared memory
const std::string SIM_SHM_NAME = "/sim_pure_1080p";

/**
 * @class SIMPurePublisher
 * @brief Publisher SIM puro - massima velocita, nessun fallback
 */
class SIMPurePublisher : public rclcpp::Node
{
public:
    SIMPurePublisher()
    : Node("sim_publisher_nofb"), 
      frame_count_(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Cerca video
        std::string video_dir = "/home/oe/ros2_ws/src/ros2_shm_vision/video/";
        cv::String pattern = video_dir + "*.mp4";
        std::vector<cv::String> files;
        cv::glob(pattern, files, false);
        
        if (files.empty()) {
            RCLCPP_ERROR(get_logger(), "[SIM-PURE] Nessun video trovato!");
            return;
        }
        
        video_capture_.open(files[0]);
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "[SIM-PURE] Impossibile aprire video!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "[SIM-PURE] Video: %s", files[0].c_str());
        
        // Inizializza SIM (NO checksum per massima velocita)
        sim_writer_ = std::make_unique<SIM::Writer>(SIM_SHM_NAME, IMAGE_SIZE, false);
        
        if (!sim_writer_->init()) {
            RCLCPP_ERROR(get_logger(), "[SIM-PURE] Errore init SIM!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "[SIM-PURE] SIM inizializzato: %s", SIM_SHM_NAME.c_str());
        
        // Timer 30 FPS
        timer_ = this->create_wall_timer(33ms, 
            std::bind(&SIMPurePublisher::publishFrame, this));
        
        last_log_time_ = this->now();
        
        RCLCPP_INFO(get_logger(), 
            "[SIM-PURE] Publisher avviato - MASSIMA VELOCITA (no fallback, no checksum)");
    }
    
    ~SIMPurePublisher() {
        if (sim_writer_) {
            sim_writer_->destroy();
        }
    }

private:
    void publishFrame() {
        if (!video_capture_.isOpened() || !sim_writer_ || !sim_writer_->isReady()) {
            return;
        }
        
        cv::Mat frame;
        if (!video_capture_.read(frame)) {
            video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!video_capture_.read(frame)) {
                return;
            }
        }
        
        // Ridimensiona e converti
        if (frame.cols != IMAGE_WIDTH || frame.rows != IMAGE_HEIGHT) {
            cv::resize(frame, frame, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
        }
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        if (!frame.isContinuous()) {
            frame = frame.clone();
        }
        
        // Scrivi direttamente in SIM
        sim_writer_->write(frame.data, IMAGE_SIZE);
        ++frame_count_;
        ++frames_in_second_;
        
        // Log ogni secondo
        auto now = this->now();
        double elapsed = (now - last_log_time_).seconds();
        if (elapsed >= 1.0) {
            double fps = static_cast<double>(frames_in_second_) / elapsed;
            RCLCPP_INFO(get_logger(), 
                "[SIM-PURE] FPS: %.1f | Frame: %u | Scritti: %lu",
                fps, frame_count_, sim_writer_->getFrameCount());
            frames_in_second_ = 0;
            last_log_time_ = now;
        }
    }
    
    std::unique_ptr<SIM::Writer> sim_writer_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    
    uint32_t frame_count_;
    uint32_t frames_in_second_ = 0;
    rclcpp::Time last_log_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SIMPurePublisher>());
    rclcpp::shutdown();
    return 0;
}
