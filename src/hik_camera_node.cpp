#include "MvCameraControl.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// 节点主体
namespace hik_camera
{
  class HikCameraNode : public rclcpp::Node
  {
  public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options) : Node("hik_camera", options)
    {
      RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

      MV_CC_DEVICE_INFO_LIST DeviceList;
      // 枚举设备
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);

      // 如果找不到设备则重复枚举
      while (DeviceList.nDeviceNum == 0 && rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);
      }

      RCLCPP_INFO(this->get_logger(), "Found camera count = %d", DeviceList.nDeviceNum);

      camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");

      // 获取句柄，开启相机
      int findname_flag = 0;
      int findnum = 0;

      while (findname_flag == 0 && rclcpp::ok())
      {
        for (int i = 0; i < (int)DeviceList.nDeviceNum; i++)
        {
          std::string str = reinterpret_cast<char*>(DeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chUserDefinedName);
          if (str == camera_name_)
          {
            RCLCPP_INFO(this->get_logger(), "%s",DeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            findnum = i;
            findname_flag = 1;
          }
        }
      }

      MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[findnum]);
      MV_CC_OpenDevice(camera_handle_);

      // 设置相机参数（可调）
      declareParameters();

      // camera_info参数的获取

      camera_info_manager_ =
          std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
      auto camera_info_url =
          this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
      if (camera_info_manager_->validateURL(camera_info_url))
      {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      }

      // 设置相机参数
      set_value();

      MV_CC_StartGrabbing(camera_handle_);

      // 获取相机数据
      MV_CC_GetImageInfo(camera_handle_, &img_info_);
      image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

      // 初始化转换参数
      ConvertParam_.nWidth = img_info_.nWidthValue;
      ConvertParam_.nHeight = img_info_.nHeightValue;
      ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

      // ros2的Qos策略，此处是读取配置文件以决定是否开启sensor_data模式(Best effort和更小的队列深度)（默认开启状态）
      bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
      auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
      camera_pub_ = image_transport::create_camera_publisher(this, this->declare_parameter("topic_name", "image_raw"), qos);

      // ROS2参数msg发布
      params_callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

      // 程序使用launch组合线程,单开hik相机转ros2msg线程
      capture_thread_ = std::thread{[this]() -> void
                                    {
                                      MV_FRAME_OUT OutFrame;

                                      RCLCPP_INFO(this->get_logger(), "Publishing image!");

                                      image_msg_.header.frame_id = "camera_optical_frame";
                                      image_msg_.encoding = "rgb8";

                                      while (rclcpp::ok())
                                      {
                                        nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 1000);
                                        if (MV_OK == nRet)
                                        {
                                          ConvertParam_.pDstBuffer = image_msg_.data.data();
                                          ConvertParam_.nDstBufferSize = image_msg_.data.size();
                                          ConvertParam_.pSrcData = OutFrame.pBufAddr;
                                          ConvertParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;
                                          ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

                                          MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);

                                          camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
                                          image_msg_.height = OutFrame.stFrameInfo.nHeight;
                                          image_msg_.width = OutFrame.stFrameInfo.nWidth;
                                          image_msg_.step = OutFrame.stFrameInfo.nWidth * 3;
                                          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);
                                          camera_pub_.publish(image_msg_, camera_info_msg_);

                                          MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);
                                        }
                                        else
                                        {
                                          RCLCPP_INFO(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
                                          MV_CC_StopGrabbing(camera_handle_);
                                          MV_CC_StartGrabbing(camera_handle_);
                                        }
                                      }
                                    }};
    }

    ~HikCameraNode()
    {
      if (capture_thread_.joinable())
      {
        capture_thread_.join();
      }
      if (camera_handle_)
      {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
      }
      RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
    }

  private:
    void declareParameters()
    {
      // param_desc参数定义msg，此处用于定义参数范围
      rcl_interfaces::msg::ParameterDescriptor param_desc;
      MVCC_FLOATVALUE fValue;
      param_desc.integer_range.resize(1);
      param_desc.integer_range[0].step = 1;
      // 曝光时间
      param_desc.description = "Exposure time in microseconds";
      MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &fValue);
      param_desc.integer_range[0].from_value = fValue.fMin;
      param_desc.integer_range[0].to_value = fValue.fMax;
      double exposure_time = this->declare_parameter("exposure_time", 15000, param_desc);
      nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
      if (MV_OK != nRet)
      {
        RCLCPP_WARN(this->get_logger(), "MV_CC_SetFloatValue exposure_time失败,错误码: [%x]", nRet);
      }
      RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

      // 增益
      param_desc.description = "Gain";
      MV_CC_GetFloatValue(camera_handle_, "Gain", &fValue);
      param_desc.integer_range[0].from_value = fValue.fMin;
      param_desc.integer_range[0].to_value = fValue.fMax;
      double gain = this->declare_parameter("gain", fValue.fCurValue, param_desc);
      nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
      if (MV_OK != nRet)
      {
        RCLCPP_WARN(this->get_logger(), "MV_CC_SetFloatValue Gain失败,错误码:[%x]", nRet);
      }
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
    }

    // 相机参数设置
    void set_value()
    {
      // 设置触发模式为off
      // 这个实际意义为让相机自行处理图像的采集，不通过外部信号来控制
      MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);

      // 设置图像长宽
      // 注意，这里对offset的值应当提前归零，防止出现长度溢出问题
      MV_CC_SetIntValue(camera_handle_, "OffsetX", 0);
      MV_CC_SetIntValue(camera_handle_, "OffsetY", 0);

      // int img_width = this->declare_parameter("image_width", 640);
      RCLCPP_INFO(this->get_logger(), "Set image_width: [%d]", camera_info_msg_.width);
      MV_CC_SetIntValue(camera_handle_, "Width", camera_info_msg_.width);

      // int img_height = this->declare_parameter("image_height", 480);
      RCLCPP_INFO(this->get_logger(), "Set image_height: [%d]", camera_info_msg_.height);
      MV_CC_SetIntValue(camera_handle_, "Height", camera_info_msg_.height);

      // 设置图像偏移
      MV_CC_SetIntValue(camera_handle_, "OffsetX", this->declare_parameter("offset_x", 0));
      MV_CC_SetIntValue(camera_handle_, "OffsetY", this->declare_parameter("offset_y", 0));

      // MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &param : parameters)
      {
        if (param.get_name() == "exposure_time")
        {
          int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
          if (MV_OK != status)
          {
            result.successful = false;
            result.reason = "Failed to set exposure time, status = " + std::to_string(status);
          }
        }
        else if (param.get_name() == "gain")
        {
          int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
          if (MV_OK != status)
          {
            result.successful = false;
            result.reason = "Failed to set gain, status = " + std::to_string(status);
          }
        }
        else
        {
          result.successful = false;
          result.reason = "Unknown parameter: " + param.get_name();
        }
      }
      return result;
    }

    sensor_msgs::msg::Image image_msg_;

    image_transport::CameraPublisher camera_pub_;

    int nRet = MV_OK;
    void *camera_handle_;
    MV_IMAGE_BASIC_INFO img_info_;

    MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;

    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    std::thread capture_thread_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  };
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
