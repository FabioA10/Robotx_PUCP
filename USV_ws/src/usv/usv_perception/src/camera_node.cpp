#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

namespace usv_perception
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_node", options), running_(false)
  {
    this->declare_parameter("cam_index", 0);
    int cam_index = this->get_parameter("cam_index").as_int();

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/robotx/camera/image_raw", 10);

    cap_.open(cam_index);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Error: Could not open USB camera at index %d", cam_index);
      return;
    }

    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    RCLCPP_INFO(this->get_logger(), "[+] OPTICAL SENSOR INITIALIZED: %dx%d pixels", 
                (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    running_ = true;
    capture_thread_ = std::thread(&CameraNode::capture_loop, this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraNode::timer_callback, this));
  }

  ~CameraNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    cap_.release();
  }

private:
  void capture_loop()
  {
    while (running_) {
      cv::Mat frame;
      if (cap_.read(frame) && !frame.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = frame.clone();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  void timer_callback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_frame_.empty()) {
        return;
      }
      frame = latest_frame_.clone();
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_link";

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*msg);
    pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::thread capture_thread_;
  std::mutex mutex_;
  cv::Mat latest_frame_;
  std::atomic<bool> running_;
};

}  // namespace usv_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usv_perception::CameraNode)
