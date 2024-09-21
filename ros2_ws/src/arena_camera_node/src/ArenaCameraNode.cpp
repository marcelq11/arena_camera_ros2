#include <cstring>  // memcopy
#include <filesystem>
#include <iostream>
#include <regex>
#include <stdexcept>  // std::runtime_err
#include <string>
namespace fs = std::filesystem;

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "camera_msg/msg/camera_settings.hpp"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

#define FRAMES_PER_SECOND 28.0

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    if (this->has_parameter("serial")) {
      int serial_integer;
      this->get_parameter<int>("serial", serial_integer);
      serial_ = std::to_string(serial_integer);
      is_passed_serial_ = true;
    } else {
      serial_ = "";  // Set it to an empty string to indicate it's not passed.
      is_passed_serial_ = false;
    }

    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );


  int64_t depth_1 = 100;
  pub_qos_.keep_last(depth_1);
  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());

  // Create a subscriber for the /params topic
  m_params_subscriber_ =
      this->create_subscription<camera_msg::msg::CameraSettings>(
          "/params", 10,
          std::bind(&ArenaCameraNode::params_callback_, this,
                    std::placeholders::_1));

  // subciber loop
  auto timer_callback = [this]() -> void { this->publish_images_(); };
  auto publish_interval =
      std::chrono::milliseconds(25);  // Frequency of publishing images
  m_publish_timer = this->create_wall_timer(publish_interval, timer_callback);
  // Recording
  is_recording_ = false;
  folder_path_ = "videos";
  int highest_number = get_video_name_(folder_path_);
  video_num_ = highest_number + 1;
}

void ArenaCameraNode::params_callback_(
    const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (is_device_created == true) {

    auto nodemap = m_pDevice->GetNodeMap();

    if (msg->target_brightness != brightness_) {
      set_brightness_node_(msg);
    }
    if (msg->exposure_time_upper_limit != exposure_time_upper_limit_ ||
        msg->exposure_time_lower_limit != exposure_time_lower_limit_) {
      set_exposure_node_limits_(msg);
    }
    if (msg->gain_upper_limit != gain_upper_limit_ ||
        msg->gain_lower_limit != gain_lower_limit_) {
      set_gain_node_limits_(msg);
    }
    if (msg->roi_width != width_aoi_exposure_ ||
        msg->roi_height != height_aoi_exposure_ ||
        msg->roi_offset_x != offset_x_aoi_exposure_ ||
        msg->roi_offset_y != offset_y_aoi_exposure_) {
      set_exposure_aoi_node_(msg);
    }
    if (msg->roi_width != width_aoi_awb_ || 
        msg->roi_height != height_aoi_awb_ ||
        msg->roi_offset_x != offset_x_aoi_awb_ || 
        msg->roi_offset_y != offset_y_aoi_awb_) {
      set_awb_aoi_node_(msg);
    }
    if (msg->width != width_ || msg->height != height_) {
      width_ = msg->width;
      height_ = msg->height;
      set_nodes_resolution_(msg);
    }
    if (msg->recording == 1 && !is_recording_) {
      start_recording_();
    } else if (msg->recording == 0 && is_recording_) {
      stop_recording_();
    }
  }
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  is_device_created = true;

  set_nodes_();
  get_nodes_values_();

  m_pDevice->StartStream();
  is_stream_started_ = true;
  //
  //  if (!trigger_mode_activated_) {
  //    publish_images_();
  //  } else {
  //    // else ros::spin will
  //  }
}

void ArenaCameraNode::publish_images_()
{

  Arena::IImage* pImage = nullptr;
  if (!is_stream_started_) {
    return;
  }
  try {
    //log_info("publish_images_");
    // auto nodemap = m_pDevice->GetNodeMap();
    // auto Width_check = Arena::GetNodeValue<int64_t>(nodemap, "Width");
    // auto Height_check = Arena::GetNodeValue<int64_t>(nodemap, "Height");
    // RCLCPP_INFO(this->get_logger(), "Width ARENA CAMERA: %ld", Width_check);
    // RCLCPP_INFO(this->get_logger(), "Height ARENA CAMERA: %ld", Height_check);
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
    pImage = m_pDevice->GetImage(1000);
    msg_form_image_(pImage, *p_image_msg);
    if (is_recording_) {
      Arena::IImage* convertedImage =
          Arena::ImageFactory::Convert(pImage, BGR8);
      video_recorder_->AppendImage(convertedImage->GetData());
      Arena::ImageFactory::Destroy(convertedImage);
    }

    m_pub_->publish(std::move(p_image_msg));

    this->m_pDevice->RequeueBuffer(pImage);

    // Calculate and log FPS
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - previous_time_;
    if (elapsed_seconds.count() > 0) {
        double fps = 1.0 / elapsed_seconds.count();
        //RCLCPP_INFO(this->get_logger(), "FPS ARENA CAMERA: %f", fps);
    }

    previous_time_ = current_time;

  } catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
      log_warn(std::string("Exception occurred while publishing an image\n") +
               e.what());
    }
  }
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::get_nodes_values_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  exposure_time_lower_limit_ = Arena::GetNodeValue<double>(nodemap, "ExposureAutoLowerLimit");
  exposure_time_upper_limit_ = Arena::GetNodeValue<double>(nodemap, "ExposureAutoUpperLimit");
  gain_lower_limit_ = Arena::GetNodeValue<double>(nodemap, "GainAutoLowerLimit");
  gain_upper_limit_ = Arena::GetNodeValue<double>(nodemap, "GainAutoUpperLimit");
  brightness_ = Arena::GetNodeValue<int64_t>(nodemap, "TargetBrightness");
  width_aoi_exposure_ = Arena::GetNodeValue<int64_t>(nodemap, "AutoExposureAOIWidth");
  height_aoi_exposure_ = Arena::GetNodeValue<int64_t>(nodemap, "AutoExposureAOIHeight");
  offset_x_aoi_exposure_ = Arena::GetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetX");
  offset_y_aoi_exposure_ = Arena::GetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetY");
  width_aoi_awb_ = Arena::GetNodeValue<int64_t>(nodemap, "AwbAOIWidth");
  height_aoi_awb_ = Arena::GetNodeValue<int64_t>(nodemap, "AwbAOIHeight");
  offset_x_aoi_awb_ = Arena::GetNodeValue<int64_t>(nodemap, "AwbAOIOffsetX");
  offset_y_aoi_awb_ = Arena::GetNodeValue<int64_t>(nodemap, "AwbAOIOffsetY");
}

void ArenaCameraNode::set_nodes_()
{
  Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "PixelFormat", "BayerRG8");

  Arena::FeatureStream featureStreamDst(m_pDevice->GetNodeMap());
  featureStreamDst.Read("features.txt");
  // Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureAutoLimitAuto", "Off");
//   Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "TargetBrightness", 140);
  // Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "AutoExposureAOIWidth", 900);
  // Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "ExposureAutoLowerLimit", 55.288);
//   Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "ExposureAutoUpperLimit", 30000);
  // Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "GainAuto", "Continuous");
//   Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "GainAutoUpperLimit", 40);
  Arena::FeatureStream featureStreamout(m_pDevice->GetNodeMap());
  featureStreamout.Write("featuresout.txt");
  // set_nodes_load_profile_();
  // set_nodes_resolution_();
  Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(), "AcquisitionFrameRateEnable", true);

  double desired_fps = 24.0;
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "AcquisitionFrameRate", desired_fps);

  auto pNodeMap = m_pDevice->GetNodeMap();
  GenApi::CFloatPtr pFloat = pNodeMap->GetNode("AcquisitionFrameRate");
  pFloat->SetValue(desired_fps);

  // int64_t width_new = 2048;
  // int64_t height_new = 2048;
  // auto nodemap = m_pDevice->GetNodeMap();
  // Arena::SetNodeValue<int64_t>(nodemap, "Width", width_new);
  // Arena::SetNodeValue<int64_t>(nodemap, "Height", height_new);
}

void ArenaCameraNode::set_brightness_node_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (msg->target_brightness < 0 && msg->target_brightness > 120) {
    log_info(std::string("\tBrightness value is out of range."));
    return;
  }
  brightness_ = msg->target_brightness;
  Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", brightness_);
  log_info(std::string("\tBrightness set to ") + std::to_string(brightness_));
  
}

void ArenaCameraNode::set_exposure_node_limits_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto") != "Continuous") {
    log_info(std::string("\tExposureAuto is not in Continuous mode. Setting it to Continuous"));
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
  }
  if (msg->exposure_time_upper_limit > 30000 || msg->exposure_time_lower_limit < 30 || 
      msg->exposure_time_upper_limit < msg->exposure_time_lower_limit) {
    log_info(std::string("\tExposureTime limits are out of range."));
    return;
  }
  exposure_time_lower_limit_ = msg->exposure_time_lower_limit;
  exposure_time_upper_limit_ = msg->exposure_time_upper_limit;
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "ExposureAutoLowerLimit", exposure_time_lower_limit_);
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "ExposureAutoUpperLimit", exposure_time_upper_limit_);
  log_info(std::string("\tExposureTime limits set to: ") +
           std::to_string(exposure_time_lower_limit_) + 
           std::to_string(exposure_time_upper_limit_));
}

void ArenaCameraNode::set_exposure_aoi_node_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (Arena::GetNodeValue<bool>(nodemap, "AutoExposureAOIEnable") != 1) {
      log_info(std::string("\tAutoExposureAOIEnable is not enabled. Setting it to enabled"));
      Arena::SetNodeValue<bool>(nodemap, "AutoExposureAOIEnable", 1);
    }
  if ( msg->roi_width > width_ || msg->roi_height > height_ || 
       msg->roi_offset_x + msg->roi_width > width_ || msg->roi_offset_y + msg->roi_height > height_) {
    log_info(std::string("\tExposure AOI is out of range."));
    return;
  }
  width_aoi_exposure_ = msg->roi_width;
  height_aoi_exposure_ = msg->roi_height;
  offset_x_aoi_exposure_ = msg->roi_offset_x;
  offset_y_aoi_exposure_ = msg->roi_offset_y;
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIWidth", width_aoi_exposure_);
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIHeight", height_aoi_exposure_);
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetX", offset_x_aoi_exposure_);
  Arena::SetNodeValue<int64_t>(nodemap, "AutoExposureAOIOffsetY", offset_y_aoi_exposure_);
  log_info(std::string("\tExposure AOI set to ") + std::to_string(width_aoi_exposure_) + ":" + std::to_string(height_aoi_exposure_));
  log_info(std::string("\tExposure AOI offset set to ") + std::to_string(offset_x_aoi_exposure_) + ":" + std::to_string(offset_y_aoi_exposure_));
}

void ArenaCameraNode::set_awb_aoi_node_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (Arena::GetNodeValue<bool>(nodemap, "AwbAOIEnable") != 1) {
      log_info(std::string("\tAwbAOIEnable is not enabled. Setting it to enabled"));
      Arena::SetNodeValue<bool>(nodemap, "AwbAOIEnable", 1);
    }
  if ( msg->roi_width > width_ || msg->roi_height > height_ || 
       msg->roi_offset_x + msg->roi_width > width_ || msg->roi_offset_y + msg->roi_height > height_) {
    log_info(std::string("\tAWB AOI is out of range."));
    return;
  }
  width_aoi_awb_ = msg->roi_width;
  height_aoi_awb_ = msg->roi_height;
  offset_x_aoi_awb_ = msg->roi_offset_x;
  offset_y_aoi_awb_ = msg->roi_offset_y;
  Arena::SetNodeValue<int64_t>(nodemap, "AwbAOIWidth", width_aoi_awb_);
  Arena::SetNodeValue<int64_t>(nodemap, "AwbAOIHeight", height_aoi_awb_);
  Arena::SetNodeValue<int64_t>(nodemap, "AwbAOIOffsetX", offset_x_aoi_awb_);
  Arena::SetNodeValue<int64_t>(nodemap, "AwbAOIOffsetY", offset_y_aoi_awb_);
  log_info(std::string("\tAWB AOI set to ") + std::to_string(width_aoi_awb_) + ":" + std::to_string(height_aoi_awb_));
  log_info(std::string("\tAWB AOI offset set to ") + std::to_string(offset_x_aoi_awb_) + ":" + std::to_string(offset_y_aoi_awb_));
}


void ArenaCameraNode::set_gain_node_limits_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
auto nodemap = m_pDevice->GetNodeMap();
  if (Arena::GetNodeValue<GenICam::gcstring>(nodemap, "GainAuto") != "Continuous") {
    log_info(std::string("\tGainAuto is not in Continuous mode. Setting it to Continuous"));
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Continuous");
  }
  if (msg->gain_upper_limit > 35 || msg->gain_lower_limit < 0 || 
      msg->gain_upper_limit < msg->gain_lower_limit) {
    log_info(std::string("\tGain limits are out of range."));
    return;
  }
  gain_lower_limit_ = msg->gain_lower_limit;
  gain_upper_limit_ = msg->gain_upper_limit;
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "GainAutoLowerLimit", gain_lower_limit_);
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "GainAutoUpperLimit", gain_upper_limit_);
  log_info(std::string("\tGain limits set to: ") +
           std::to_string(gain_lower_limit_) + 
           std::to_string(gain_upper_limit_));
}

void ArenaCameraNode::set_nodes_resolution_(const camera_msg::msg::CameraSettings::SharedPtr msg)
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (msg->width < 0 || msg->height < 0 || msg->width > 2448 || msg->height > 2048) {
    log_info(std::string("\tResolution is out of range."));
    return;
  }
  width_ = msg->width;
  height_ = msg->height;
  Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  log_info(std::string("\tWidth set to ") + std::to_string(width_) + " Height set to " + std::to_string(height_));
}

void ArenaCameraNode::set_nodes_load_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "UserSet1");
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tUserSet1 profile is loaded");
}


void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
    }
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

int ArenaCameraNode::get_video_name_(std::string& folder_path)
{
  if (!fs::exists(folder_path)) {
    fs::create_directory(folder_path);
  }

  std::regex pattern("output(\\d+)\\.mp4");
  std::optional<int> max_number;

  for (const auto& entry : fs::directory_iterator(folder_path)) {
    std::smatch matches;
    std::string filename = entry.path().filename().string();

    if (std::regex_match(filename, matches, pattern)) {
      int number = std::stoi(matches[1].str());
      if (!max_number.has_value() || number > max_number) {
        max_number = number;
      }
    }
  }
  return max_number.value_or(0);
}

void ArenaCameraNode::start_recording_()
{  // ADD if only recording is not started already
  if (!is_recording_) {
    video_params_ = Save::VideoParams(width_, height_, FRAMES_PER_SECOND);
    is_recording_ = true;
    log_info("Recording started.");
    std::string video_name =
        folder_path_ + "/" + "output" + std::to_string(video_num_) + ".mp4";
    video_recorder_ = std::make_unique<Save::VideoRecorder>(video_params_, video_name.c_str());
    video_recorder_->SetH264Mp4BGR8();
    video_recorder_->Open();
  }
}

void ArenaCameraNode::stop_recording_()
{
  is_recording_ = false;
  log_info("Recording stopped.");
  video_recorder_->Close();
  video_recorder_.reset();
  video_num_++;
}