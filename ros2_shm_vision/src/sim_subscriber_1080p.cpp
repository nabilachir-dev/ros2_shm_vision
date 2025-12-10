/**
 * @file sim_subscriber_1080p.cpp
 * @brief Subscriber SIM con fallback chain: SIM -> Zero-Copy -> Standard
 * 
 * Questo subscriber tenta prima SIM (latenza minima ~0.2ms),
 * poi fallback a zero-copy (~2-3ms), poi standard (~5-6ms).
 */

#include <chrono>
#include <memory>
#include <string>
#include <cstring>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ros2_shm_vision/msg/image1080p.hpp"
#include "sim_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

// Nome shared memory SIM
const std::string SIM_SHM_NAME = "/sim_image_1080p";

enum class TransportMode {
    SIM,
    ZERO_COPY,
    STANDARD
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
 * @class SIMSubscriber1080p
 * @brief Subscriber con fallback automatico SIM -> ZeroCopy -> Standard
 */
class SIMSubscriber1080p : public rclcpp::Node
{
public:
    SIMSubscriber1080p()
    : Node("sim_subscriber_1080p"), 
      frame_count_(0),
      current_mode_(TransportMode::SIM),
      show_video_(true),
      sim_retry_count_(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Parametri
        this->declare_parameter("sim_shm_name", SIM_SHM_NAME);
        this->declare_parameter("show_video", true);
        this->declare_parameter("sim_timeout_ms", 5000);
        
        sim_shm_name_ = this->get_parameter("sim_shm_name").as_string();
        show_video_ = this->get_parameter("show_video").as_bool();
        sim_timeout_ms_ = this->get_parameter("sim_timeout_ms").as_int();
        
        // Alloca buffer per dati
        data_buffer_.resize(IMAGE_SIZE);
        
        // Crea finestra se abilitato
        if (show_video_) {
            cv::namedWindow("SIM 1080p Stream", cv::WINDOW_AUTOSIZE);
        }
        
        // Inizializza trasporto
        initializeTransport();
        
        RCLCPP_INFO(get_logger(), "[SIM] Subscriber avviato in modalita: %s", 
            modeToString(current_mode_).c_str());
        
        last_stats_time_ = this->now();
    }
    
    ~SIMSubscriber1080p() {
        running_ = false;
        if (sim_thread_.joinable()) {
            sim_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void initializeTransport() {
        // STEP 1: Prova SIM
        RCLCPP_INFO(get_logger(), "[SIM] Tentativo connessione SIM...");
        
        sim_reader_ = std::make_unique<SIM::Reader>(sim_shm_name_, IMAGE_SIZE);
        
        // Attendi che SIM sia disponibile (con timeout)
        auto start = std::chrono::steady_clock::now();
        while (!sim_reader_->init()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            
            if (elapsed > sim_timeout_ms_) {
                RCLCPP_WARN(get_logger(), 
                    "[SIM] Timeout attesa SIM (%d ms), provo Zero-Copy...", 
                    sim_timeout_ms_);
                sim_reader_.reset();
                break;
            }
            
            std::this_thread::sleep_for(100ms);
            ++sim_retry_count_;
            
            if (sim_retry_count_ % 10 == 0) {
                RCLCPP_INFO(get_logger(), 
                    "[SIM] Attendo publisher SIM... (%d ms)", 
                    static_cast<int>(elapsed));
            }
        }
        
        if (sim_reader_ && sim_reader_->isReady()) {
            current_mode_ = TransportMode::SIM;
            RCLCPP_INFO(get_logger(), "[SIM] Connesso a SIM!");
            
            // Avvia thread lettura SIM
            running_ = true;
            sim_thread_ = std::thread(&SIMSubscriber1080p::simReadLoop, this);
            return;
        }
        
        // STEP 2: Prova Zero-Copy
        RCLCPP_INFO(get_logger(), "[SIM] Provo Zero-Copy...");
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        zc_subscription_ = this->create_subscription<ros2_shm_vision::msg::Image1080p>(
            "image_1080p", qos,
            std::bind(&SIMSubscriber1080p::zcCallback, this, std::placeholders::_1));
        
        current_mode_ = TransportMode::ZERO_COPY;
        RCLCPP_INFO(get_logger(), "[SIM] Sottoscritto a Zero-Copy topic");
        
        // Se non riceve nulla entro timeout, fallback a standard
        fallback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(sim_timeout_ms_),
            std::bind(&SIMSubscriber1080p::checkFallbackToStandard, this));
    }
    
    void simReadLoop() {
        RCLCPP_INFO(get_logger(), "[SIM] Thread lettura SIM avviato");
        
        while (running_ && rclcpp::ok()) {
            size_t size = 0;
            
            if (sim_reader_->read(data_buffer_.data(), size)) {
                ++frame_count_;
                ++frames_in_second_;
                
                // Calcola latenza
                int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                int64_t msg_ns = sim_reader_->getLastTimestampNs();
                double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
                
                // Verifica checksum
                if (!sim_reader_->verifyLastChecksum()) {
                    RCLCPP_WARN(get_logger(), "[SIM] Checksum FALLITO frame #%u!", 
                        frame_count_);
                }
                
                // Statistiche ogni secondo
                auto now = this->now();
                double elapsed = (now - last_stats_time_).seconds();
                if (elapsed >= 1.0) {
                    double fps = static_cast<double>(frames_in_second_) / elapsed;
                    RCLCPP_INFO(get_logger(),
                        "[SIM] FPS: %.1f | Latenza: %.3f ms | Dropped: %lu",
                        fps, latency_ms, sim_reader_->getDroppedFrames());
                    frames_in_second_ = 0;
                    last_stats_time_ = now;
                }
                
                // Visualizza
                if (show_video_) {
                    displayFrame(data_buffer_.data(), size, latency_ms, "SIM");
                }
            }
            
            // Verifica se writer e ancora vivo
            if (!sim_reader_->isWriterAlive(2000)) {
                RCLCPP_WARN(get_logger(), "[SIM] Writer non risponde, fallback...");
                running_ = false;
                
                // Schedule fallback nel thread principale
                // (non possiamo creare subscription da altro thread)
            }
            
            // Breve sleep
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        
        RCLCPP_INFO(get_logger(), "[SIM] Thread lettura SIM terminato");
    }
    
    void zcCallback(const ros2_shm_vision::msg::Image1080p::SharedPtr msg) {
        // Disabilita timer fallback se riceviamo dati
        if (fallback_timer_) {
            fallback_timer_->cancel();
            fallback_timer_.reset();
        }
        
        ++frame_count_;
        ++frames_in_second_;
        
        // Calcola latenza
        int64_t msg_ns = static_cast<int64_t>(msg->timestamp_sec) * 1000000000LL +
                         static_cast<int64_t>(msg->timestamp_nanosec);
        int64_t now_ns = this->now().nanoseconds();
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        // Statistiche
        auto now = this->now();
        double elapsed = (now - last_stats_time_).seconds();
        if (elapsed >= 1.0) {
            double fps = static_cast<double>(frames_in_second_) / elapsed;
            RCLCPP_INFO(get_logger(),
                "[ZC] FPS: %.1f | Latenza: %.2f ms | Frame: %u",
                fps, latency_ms, frame_count_);
            frames_in_second_ = 0;
            last_stats_time_ = now;
        }
        
        // Visualizza
        if (show_video_) {
            displayFrame(msg->data.data(), IMAGE_SIZE, latency_ms, "ZERO_COPY");
        }
    }
    
    void stdCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        ++frame_count_;
        ++frames_in_second_;
        
        // Calcola latenza
        int64_t msg_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        int64_t now_ns = this->now().nanoseconds();
        double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
        
        // Statistiche
        auto now = this->now();
        double elapsed = (now - last_stats_time_).seconds();
        if (elapsed >= 1.0) {
            double fps = static_cast<double>(frames_in_second_) / elapsed;
            RCLCPP_INFO(get_logger(),
                "[STD] FPS: %.1f | Latenza: %.2f ms | Frame: %u",
                fps, latency_ms, frame_count_);
            frames_in_second_ = 0;
            last_stats_time_ = now;
        }
        
        // Visualizza
        if (show_video_ && !msg->data.empty()) {
            displayFrame(msg->data.data(), msg->data.size(), latency_ms, "STANDARD");
        }
    }
    
    void checkFallbackToStandard() {
        if (frame_count_ == 0) {
            RCLCPP_WARN(get_logger(), 
                "[SIM] Nessun frame Zero-Copy ricevuto, fallback a Standard...");
            
            fallback_timer_->cancel();
            fallback_timer_.reset();
            
            // Sottoscrivi a standard
            std_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "image_1080p_standard", 10,
                std::bind(&SIMSubscriber1080p::stdCallback, this, std::placeholders::_1));
            
            current_mode_ = TransportMode::STANDARD;
            RCLCPP_INFO(get_logger(), "[SIM] Modalita STANDARD attiva");
        } else {
            // Abbiamo ricevuto frame, cancella timer
            fallback_timer_->cancel();
            fallback_timer_.reset();
        }
    }
    
    void displayFrame(const uint8_t* data, size_t /*size*/, 
                      double latency_ms, const std::string& mode) {
        cv::Mat frame_rgb(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, 
                         const_cast<uint8_t*>(data));
        cv::Mat frame_bgr;
        cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
        
        // Overlay info
        std::string info = mode + " | Frame: " + std::to_string(frame_count_) +
                          " | Lat: " + std::to_string(latency_ms).substr(0, 5) + " ms";
        
        cv::Scalar color;
        if (mode == "SIM") {
            color = cv::Scalar(255, 0, 255);  // Magenta per SIM
        } else if (mode == "ZERO_COPY") {
            color = cv::Scalar(0, 255, 0);    // Verde per ZC
        } else {
            color = cv::Scalar(0, 0, 255);    // Rosso per Standard
        }
        
        cv::putText(frame_bgr, info, cv::Point(20, 40),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
        
        cv::imshow("SIM 1080p Stream", frame_bgr);
        
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            rclcpp::shutdown();
        }
    }
    
    // Subscriptions
    rclcpp::Subscription<ros2_shm_vision::msg::Image1080p>::SharedPtr zc_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr std_subscription_;
    std::unique_ptr<SIM::Reader> sim_reader_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr fallback_timer_;
    
    // Thread SIM
    std::thread sim_thread_;
    std::atomic<bool> running_{false};
    
    // Buffer dati
    std::vector<uint8_t> data_buffer_;
    
    // Stato
    uint32_t frame_count_;
    uint32_t frames_in_second_ = 0;
    TransportMode current_mode_;
    bool show_video_;
    int sim_timeout_ms_;
    int sim_retry_count_;
    std::string sim_shm_name_;
    rclcpp::Time last_stats_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SIMSubscriber1080p>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
