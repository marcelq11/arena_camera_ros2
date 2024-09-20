#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger
#include "camera_msg/msg/camera_settings.hpp"
// arena sdk
#include "ArenaApi.h"
#include "SaveApi.h"  

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node"), previous_time_(std::chrono::steady_clock::now())//for FPS calculation
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

  bool is_device_created = false;

 private:
 std::chrono::steady_clock::time_point previous_time_;//for FPS calculation
//Recording variables
  bool is_recording_;
  std::unique_ptr<Save::VideoRecorder> video_recorder_;
  Save::VideoParams video_params_;
  void start_recording_();
  void stop_recording_();
  int get_video_name_(std::string& folder_name);
  int video_num_;
  std::string video_name_;
  std::string folder_path_;

  rclcpp::TimerBase::SharedPtr m_publish_timer;
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

// Subscriber for parameter updates
  rclcpp::Subscription<camera_msg::msg::CameraSettings>::SharedPtr m_params_subscriber_;
  bool is_stream_started_ = false;

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  double gamma_;
  bool is_passed_gamma_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  //Parameters for adjusting the exposure time and gain
  double exposure_time_lower_limit_;
  double exposure_time_upper_limit_;
  double gain_lower_limit_;
  double gain_upper_limit_;
  int64_t brightness_;
  int64_t width_aoi_exposure_;
  int64_t height_aoi_exposure_;
  int64_t offset_x_aoi_exposure_;
  int64_t offset_y_aoi_exposure_;
  int64_t width_aoi_awb_;
  int64_t height_aoi_awb_;
  int64_t offset_x_aoi_awb_;
  int64_t offset_y_aoi_awb_;

  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_gamma_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_trigger_mode_();
  void publish_images_();
  void set_exposure_node_limits_();
  void set_exposure_aoi_node_();
  void set_awb_aoi_node_();
  void set_gain_node_limits_();
  void set_brightness_node_();

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
// Callback function to handle parameter updates
  void params_callback_(const camera_msg::msg::CameraSettings::SharedPtr msg);
};

