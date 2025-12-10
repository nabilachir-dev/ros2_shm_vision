/**
 * @file sim_publisher_1080p.cpp
 * @brief Publisher SIM con fallback chain: SIM -> Zero-Copy -> Standard
 * 
 * Questo publisher tenta prima SIM (latenza minima ~0.2ms),
 * poi fallback a zero-copy (~2-3ms), poi standard (~5-6ms).
 */

#include <chrono>
#include <memory>
#include <string>
#include <cstring>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ros2_shm_vision/msg/image1080p.hpp"
#include "sim_transport.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

// Nome shared memory SIM
const std::string SIM_SHM_NAME = "/sim_image_1080p";

enum class TransportMode {
    SIM,         // Sensor-in-Memory (latenza minima)
    ZERO_COPY,   // ROS2 loaned messages
    STANDARD     // sensor_msgs/Image standard
};

std::string modeToString(TransportMode mode) {
    switch (mode) {
        case TransportMode::SIM: return "SIM";
        case TransportMode::ZERO_COPY: return "ZERO_COPY";
        case TransportMode::STANDARD: return "STANDARD";
        default: return "UNKNOWN";
    }
}

/**
 * @class SIMPublisher1080p
 * @brief Publisher con fallback automatico SIM -> ZeroCopy -> Standard
 */
class SIMPublisher1080p : public rclcpp::Node
{
public:
    SIMPublisher1080p()
    : Node("sim_publisher_1080p"), 
      frame_count_(0),
      current_mode_(TransportMode::SIM),
      sim_enabled_(true),
      enable_checksum_(false)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Parametri
        this->declare_parameter("enable_checksum", false);
        this->declare_parameter("sim_shm_name", SIM_SHM_NAME);
        
        enable_checksum_ = this->get_parameter("enable_checksum").as_bool();
        sim_shm_name_ = this->get_parameter("sim_shm_name").as_string();
        
        // Cerca video
        video_path_ = findVideoFile();
        if (video_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "[SIM] Nessun video trovato!");
            return;
        }
        
        video_capture_.open(video_path_);
        if (!video_capture_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "[SIM] Impossibile aprire: %s", video_path_.c_str());
            return;
        }
        
        RCLCPP_INFO(get_logger(), "[SIM] Video: %s", video_path_.c_str());
        
        // Inizializza trasporto con fallback
        initializeTransport();
        
        // Timer 30 FPS
        timer_ = this->create_wall_timer(33ms, 
            std::bind(&SIMPublisher1080p::publishFrame, this));
        
        RCLCPP_INFO(get_logger(), "[SIM] Publisher avviato in modalita: %s", 
            modeToString(current_mode_).c_str());
    }
    
    ~SIMPublisher1080p() {
        if (sim_writer_) {
            sim_writer_->destroy();
        }
    }

private:
    void initializeTransport() {
        // STEP 1: Prova SIM
        RCLCPP_INFO(get_logger(), "[SIM] Tentativo inizializzazione SIM...");
        
        sim_writer_ = std::make_unique<SIM::Writer>(
            sim_shm_name_, IMAGE_SIZE, enable_checksum_);
        
        if (sim_writer_->init()) {
            current_mode_ = TransportMode::SIM;
            RCLCPP_INFO(get_logger(), 
                "[SIM] SIM inizializzato! Checksum: %s", 
                enable_checksum_ ? "ON" : "OFF");
            return;
        }
        
        RCLCPP_WARN(get_logger(), "[SIM] SIM fallito, provo Zero-Copy...");
        sim_writer_.reset();
        sim_enabled_ = false;
        
        // STEP 2: Prova Zero-Copy
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        
        zc_publisher_ = this->create_publisher<ros2_shm_vision::msg::Image1080p>(
            "image_1080p", qos);
        
        if (zc_publisher_->can_loan_messages()) {
            current_mode_ = TransportMode::ZERO_COPY;
            RCLCPP_INFO(get_logger(), "[SIM] Zero-Copy abilitato!");
            return;
        }
        
        RCLCPP_WARN(get_logger(), "[SIM] Zero-Copy fallito, uso Standard...");
        
        // STEP 3: Fallback a Standard
        std_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_1080p_standard", 10);
        current_mode_ = TransportMode::STANDARD;
        
        RCLCPP_WARN(get_logger(), "[SIM] Modalita STANDARD (con copie)");
    }
    
    std::string findVideoFile() {
        std::string video_dir = "/home/oe/ros2_ws/src/ros2_shm_vision/video/";
        cv::String pattern = video_dir + "*.mp4";
        std::vector<cv::String> files;
        cv::glob(pattern, files, false);
        
        if (!files.empty()) {
            return files[0];
        }
        return "";
    }
    
    void publishFrame() {
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
        
        // Ridimensiona e converti
        if (frame.cols != IMAGE_WIDTH || frame.rows != IMAGE_HEIGHT) {
            cv::resize(frame, frame, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
        }
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        
        if (!frame.isContinuous()) {
            frame = frame.clone();
        }
        
        ++frame_count_;
        auto now = this->now();
        
        // Pubblica in base alla modalita
        switch (current_mode_) {
            case TransportMode::SIM:
                publishSIM(frame, now);
                break;
            case TransportMode::ZERO_COPY:
                publishZeroCopy(frame, now);
                break;
            case TransportMode::STANDARD:
                publishStandard(frame, now);
                break;
        }
        
        // Log ogni secondo
        if (frame_count_ % 30 == 0) {
            RCLCPP_INFO(get_logger(), 
                "[%s] Frame #%u | %dx%d", 
                modeToString(current_mode_).c_str(),
                frame_count_, IMAGE_WIDTH, IMAGE_HEIGHT);
        }
    }
    
    void publishSIM(const cv::Mat& frame, const rclcpp::Time& /*now*/) {
        if (!sim_writer_ || !sim_writer_->isReady()) {
            // Fallback
            RCLCPP_WARN(get_logger(), "[SIM] Writer non pronto, fallback...");
            initializeTransport();
            return;
        }
        
        if (!sim_writer_->write(frame.data, IMAGE_SIZE)) {
            RCLCPP_ERROR(get_logger(), "[SIM] Errore scrittura!");
        }
    }
    
    void publishZeroCopy(const cv::Mat& frame, const rclcpp::Time& now) {
        if (!zc_publisher_->can_loan_messages()) {
            // Fallback a standard
            current_mode_ = TransportMode::STANDARD;
            std_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "image_1080p_standard", 10);
            publishStandard(frame, now);
            return;
        }
        
        auto loaned_msg = zc_publisher_->borrow_loaned_message();
        loaned_msg.get().timestamp_sec = static_cast<int32_t>(now.seconds());
        loaned_msg.get().timestamp_nanosec = static_cast<uint32_t>(
            now.nanoseconds() % 1000000000);
        loaned_msg.get().frame_id = frame_count_;
        loaned_msg.get().width = IMAGE_WIDTH;
        loaned_msg.get().height = IMAGE_HEIGHT;
        std::memcpy(loaned_msg.get().data.data(), frame.data, IMAGE_SIZE);
        
        zc_publisher_->publish(std::move(loaned_msg));
    }
    
    void publishStandard(const cv::Mat& frame, const rclcpp::Time& now) {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = now;
        msg.header.frame_id = "camera";
        msg.width = IMAGE_WIDTH;
        msg.height = IMAGE_HEIGHT;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = IMAGE_WIDTH * 3;
        msg.data.assign(frame.data, frame.data + IMAGE_SIZE);
        
        std_publisher_->publish(msg);
    }
    
    // Publishers
    rclcpp::Publisher<ros2_shm_vision::msg::Image1080p>::SharedPtr zc_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr std_publisher_;
    std::unique_ptr<SIM::Writer> sim_writer_;
    
    // Timer e video
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
    std::string video_path_;
    
    // Stato
    uint32_t frame_count_;
    TransportMode current_mode_;
    bool sim_enabled_;
    bool enable_checksum_;
    std::string sim_shm_name_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SIMPublisher1080p>());
    rclcpp::shutdown();
    return 0;
}
