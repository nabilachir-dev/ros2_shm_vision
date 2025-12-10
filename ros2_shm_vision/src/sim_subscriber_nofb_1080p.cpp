/**
 * @file sim_subscriber_nofb_1080p.cpp
 * @brief Subscriber SIM PURO - Nessun fallback, massima velocita
 * 
 * Questo subscriber usa SOLO SIM, senza fallback a Zero-Copy o Standard.
 * E' la modalita piu veloce possibile (~0.2ms latenza).
 */

#include <chrono>
#include <memory>
#include <string>
#include <cstring>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sim_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

// Dimensioni immagine 1080p
constexpr uint32_t IMAGE_WIDTH = 1920;
constexpr uint32_t IMAGE_HEIGHT = 1080;
constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * 3;

// Nome shared memory
const std::string SIM_SHM_NAME = "/sim_pure_1080p";

/**
 * @class SIMPureSubscriber
 * @brief Subscriber SIM puro - massima velocita, nessun fallback
 */
class SIMPureSubscriber : public rclcpp::Node
{
public:
    SIMPureSubscriber()
    : Node("sim_subscriber_nofb"), 
      frame_count_(0),
      show_video_(true)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        
        // Parametro visualizzazione
        this->declare_parameter("show_video", true);
        show_video_ = this->get_parameter("show_video").as_bool();
        
        // Alloca buffer
        data_buffer_.resize(IMAGE_SIZE);
        
        // Crea finestra
        if (show_video_) {
            cv::namedWindow("SIM Pure 1080p", cv::WINDOW_AUTOSIZE);
        }
        
        // Inizializza reader
        sim_reader_ = std::make_unique<SIM::Reader>(SIM_SHM_NAME, IMAGE_SIZE);
        
        RCLCPP_INFO(get_logger(), "[SIM-PURE] Attendo publisher su: %s", SIM_SHM_NAME.c_str());
        
        // Attesa connessione (polling veloce)
        while (!sim_reader_->init() && rclcpp::ok()) {
            std::this_thread::sleep_for(50ms);
        }
        
        if (!sim_reader_->isReady()) {
            RCLCPP_ERROR(get_logger(), "[SIM-PURE] Impossibile connettersi a SIM!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "[SIM-PURE] Connesso a SIM!");
        
        // Avvia thread lettura ad alta velocita
        running_ = true;
        read_thread_ = std::thread(&SIMPureSubscriber::readLoop, this);
        
        RCLCPP_INFO(get_logger(), 
            "[SIM-PURE] Subscriber avviato - MASSIMA VELOCITA (no fallback, no checksum)");
        
        last_log_time_ = this->now();
    }
    
    ~SIMPureSubscriber() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void readLoop() {
        while (running_ && rclcpp::ok()) {
            size_t size = 0;
            
            if (sim_reader_->read(data_buffer_.data(), size)) {
                ++frame_count_;
                ++frames_in_second_;
                
                // Calcola latenza (timestamp SIM)
                int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                int64_t msg_ns = sim_reader_->getLastTimestampNs();
                double latency_ms = static_cast<double>(now_ns - msg_ns) / 1e6;
                
                // Accumula per media
                latency_sum_ += latency_ms;
                ++latency_count_;
                
                // Log ogni secondo  
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_log_time_steady_).count();
                
                if (elapsed >= 1000) {
                    double fps = static_cast<double>(frames_in_second_) * 1000.0 / elapsed;
                    double avg_latency = latency_sum_ / latency_count_;
                    
                    RCLCPP_INFO(this->get_logger(),
                        "[SIM-PURE] FPS: %.1f | Latenza: %.3f ms | Dropped: %lu",
                        fps, avg_latency, sim_reader_->getDroppedFrames());
                    
                    frames_in_second_ = 0;
                    latency_sum_ = 0;
                    latency_count_ = 0;
                    last_log_time_steady_ = now;
                }
                
                // Visualizza (opzionale)
                if (show_video_) {
                    cv::Mat frame_rgb(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, data_buffer_.data());
                    cv::Mat frame_bgr;
                    cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
                    
                    // Overlay giallo per SIM PURE
                    std::string info = "SIM-PURE | Lat: " + 
                        std::to_string(latency_ms).substr(0, 5) + " ms";
                    cv::putText(frame_bgr, info, cv::Point(20, 40),
                               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
                    
                    cv::imshow("SIM Pure 1080p", frame_bgr);
                    
                    int key = cv::waitKey(1);
                    if (key == 27 || key == 'q') {
                        rclcpp::shutdown();
                    }
                }
            }
            
            // Polling velocissimo (no sleep per latenza minima)
            // Ma per evitare 100% CPU, breve yield
            std::this_thread::yield();
        }
    }
    
    std::unique_ptr<SIM::Reader> sim_reader_;
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    
    std::vector<uint8_t> data_buffer_;
    
    uint32_t frame_count_;
    uint32_t frames_in_second_ = 0;
    double latency_sum_ = 0;
    uint64_t latency_count_ = 0;
    
    bool show_video_;
    rclcpp::Time last_log_time_;
    std::chrono::steady_clock::time_point last_log_time_steady_ = 
        std::chrono::steady_clock::now();
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SIMPureSubscriber>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
