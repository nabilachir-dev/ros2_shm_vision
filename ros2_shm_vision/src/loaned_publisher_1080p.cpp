/**
 * @file loaned_publisher_1080p.cpp
 * @brief Publisher Zero-Copy per immagini 1080p a 30 FPS
 * 
 * IT: Legge frame da file video e li pubblica usando borrow_loaned_message()
 *     per trasferimento zero-copy via shared memory.
 * 
 * EN: Reads frames from video file and publishes using borrow_loaned_message()
 *     for zero-copy transfer via shared memory.
 */

#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <cstring>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_shm_vision/msg/image1080p.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;  // RGB

/**
 * @class LoanedPublisher1080p
 * @brief Publisher zero-copy per streaming video 1080p
 */
class LoanedPublisher1080p : public rclcpp::Node
{
public:
    LoanedPublisher1080p()
    : Node("loaned_publisher_1080p"), frame_count_(0)
    {
        // Disabilita buffering stdout
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Cerca il file video
        video_path_ = find_video_file();
        
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                "[IT] Nessun file video trovato! Metti un video .mp4 in:");
            RCLCPP_ERROR(this->get_logger(), 
                "     ~/ros2_ws/src/ros2_shm_vision/video/");
            RCLCPP_ERROR(this->get_logger(), 
                "[EN] No video file found! Put a .mp4 video in:");
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
        
        // Info video
        double fps = video_capture_.get(cv::CAP_PROP_FPS);
        int width = static_cast<int>(video_capture_.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = static_cast<int>(video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
        int total_frames = static_cast<int>(video_capture_.get(cv::CAP_PROP_FRAME_COUNT));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Video caricato: %s", video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Risoluzione: %dx%d, FPS: %.1f, Frame totali: %d",
            width, height, fps, total_frames);
        
        // QoS per streaming video
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        
        // Crea publisher
        publisher_ = this->create_publisher<ros2_shm_vision::msg::Image1080p>(
            "image_1080p", qos);
        
        // Verifica zero-copy
        if (publisher_->can_loan_messages()) {
            RCLCPP_INFO(this->get_logger(), 
                "[IT] Zero-copy ABILITATO per immagini 1080p (~6 MB/frame).");
            RCLCPP_INFO(this->get_logger(), 
                "[EN] Zero-copy ENABLED for 1080p images (~6 MB/frame).");
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[IT] Zero-copy NON DISPONIBILE - Usando pubblicazione standard.");
            RCLCPP_WARN(this->get_logger(),
                "[EN] Zero-copy NOT AVAILABLE - Using standard publishing.");
        }
        
        // Timer a ~30 fps (33.33 ms)
        timer_ = this->create_wall_timer(
            33ms, std::bind(&LoanedPublisher1080p::publish_frame, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "[IT] Publisher LOANED 1080p avviato. 30 FPS su 'image_1080p'.");
    }

private:
    std::string find_video_file()
    {
        // Cerca in vari percorsi possibili
        std::vector<std::string> paths = {
            "/home/oe/ros2_ws/src/ros2_shm_vision/video/sample_1080p.mp4",
            "/home/oe/ros2_ws/src/ros2_shm_vision/video/video.mp4",
            "/home/oe/ros2_ws/src/ros2_shm_vision/video/input.mp4"
        };
        
        // Cerca anche qualsiasi .mp4 nella cartella video
        std::string video_dir = "/home/oe/ros2_ws/src/ros2_shm_vision/video/";
        cv::String pattern = video_dir + "*.mp4";
        std::vector<cv::String> files;
        cv::glob(pattern, files, false);
        
        for (const auto& file : files) {
            return file;
        }
        
        // Prova i percorsi predefiniti
        for (const auto& path : paths) {
            if (std::ifstream(path).good()) {
                return path;
            }
        }
        
        return "";
    }
    
    void publish_frame()
    {
        if (!video_capture_.isOpened()) {
            return;
        }
        
        cv::Mat frame;
        
        // Leggi frame
        if (!video_capture_.read(frame)) {
            // Fine video - riparti dall'inizio (loop)
            video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!video_capture_.read(frame)) {
                RCLCPP_ERROR(this->get_logger(), "[IT] Errore lettura video!");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "[IT] Video in loop - ricominciato.");
        }
        
        // Ridimensiona a 1080p se necessario
        if (frame.cols != IMAGE_WIDTH || frame.rows != IMAGE_HEIGHT) {
            cv::resize(frame, frame, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
        }
        
        // Converti BGR (OpenCV) → RGB
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        // Assicurati che sia continuo in memoria
        if (!frame.isContinuous()) {
            frame = frame.clone();
        }
        
        // Timestamp
        auto now = this->now();
        
        if (!publisher_->can_loan_messages()) {
            // FALLBACK: pubblicazione standard
            auto msg = ros2_shm_vision::msg::Image1080p();
            msg.timestamp_sec = static_cast<int32_t>(now.seconds());
            msg.timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            msg.frame_id = frame_count_;
            msg.width = IMAGE_WIDTH;
            msg.height = IMAGE_HEIGHT;
            
            std::memcpy(msg.data.data(), frame.data, IMAGE_SIZE);
            
            RCLCPP_INFO(this->get_logger(),
                "[IT] Frame #%u (std) | %dx%d | ~6 MB",
                frame_count_, IMAGE_WIDTH, IMAGE_HEIGHT);
            
            publisher_->publish(msg);
        } else {
            // ZERO-COPY: scrivi direttamente in shared memory
            auto loaned_msg = publisher_->borrow_loaned_message();
            
            loaned_msg.get().timestamp_sec = static_cast<int32_t>(now.seconds());
            loaned_msg.get().timestamp_nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            loaned_msg.get().frame_id = frame_count_;
            loaned_msg.get().width = IMAGE_WIDTH;
            loaned_msg.get().height = IMAGE_HEIGHT;
            
            // Copia pixel direttamente in shared memory
            std::memcpy(loaned_msg.get().data.data(), frame.data, IMAGE_SIZE);
            
            RCLCPP_INFO(this->get_logger(),
                "[IT] Frame #%u (zc) | %dx%d | ~6 MB | Zero-Copy",
                frame_count_, IMAGE_WIDTH, IMAGE_HEIGHT);
            
            publisher_->publish(std::move(loaned_msg));
        }
        
        ++frame_count_;
    }
    
    rclcpp::Publisher<ros2_shm_vision::msg::Image1080p>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    std::string video_path_;
    uint32_t frame_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanedPublisher1080p>());
    rclcpp::shutdown();
    return 0;
}
