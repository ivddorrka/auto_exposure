/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* SRV include for gain setup*/
#include "exposure_simulation/SetNewGain.h"

/* for math calculations */
#include <cmath>

/*synchronization*/
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace exposure_simulation
{

class ExposureControl : public nodelet::Nodelet {

  public: 
    virtual void onInit();

  private:

    std::atomic<bool> is_initialized_ = false; // flag*
    bool _gui_ = true; //rosParam
    std::string _uav_name_; //rosParam
     // | --------------------- MRS transformer -------------------- |

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- msg callbacks --------------------- |

  
  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
  void                        callbackTagImage(const sensor_msgs::ImageConstPtr& msg);
  
  image_transport::Subscriber sub_image_;
  image_transport::Subscriber tag_image_subscription_;
  
  void                               callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  ros::Subscriber                    sub_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | -------------------- image processing -------------------- |

  // | --------- variables, related to message checking --------- |

  std::mutex mutex_counters_;           // to prevent data races when accessing the following variables from multiple threads
  ros::Time  time_last_image_;          // time stamp of the last received image message
  ros::Time  time_last_camera_info_;    // time stamp of the last received camera info message
  uint64_t   image_counter_   = 0;      // counts the number of images received
  bool       got_image_       = false;  // indicates whether at least one image message was received
  bool       got_camera_info_ = false;  // indicates whether at least one camera info message was received
  const sensor_msgs::CameraInfoConstPtr msg_info;
  // | --------------- variables for exposure -------------- |

  int max_exposure = 100000;
  int exposure_slider_value = 10000;
  const double  shutter_speed_default = 1000.0;
  
  double default_gain = 1.0;
  double gain_to_set = default_gain;

  bool flag_save = 1;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher             pub_test_;
  ros::Publisher             pub_info_;
  int                        _rate_timer_publish_;
  image_transport::Publisher gain_published;
  image_transport::Publisher exposure_published;
  image_transport::Publisher tag_detect;

  sensor_msgs::CameraInfoConstPtr msg_camera_info;
  // | --------------------- other functions -------------------- |
  bool               callbackDecreaseShutterSpeed(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_decrease_shutter_speed_;


  bool               callbackIncreaseShutterSpeed(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_increase_shutter_speed_;


  bool               callbackSetNewGain(exposure_simulation::SetNewGain::Request &req,  exposure_simulation::SetNewGain::Response &res);
  ros::ServiceServer srv_server_set_new_gain_;

  
  void publishGainImage(cv::Mat &image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub, const ros::Publisher pub_info, image_geometry::PinholeCameraModel camera_model_); 
  void publishOpenCVImage(cv::Mat &image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
 
  void publishImageNumber(uint64_t count);

  void ShowExpImage(cv::Mat &image);
  double gain_calculation(cv::Mat &dImg);
};


void ExposureControl::onInit() {

  got_image_       = false;
  got_camera_info_ = false;

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::NodeHandle n;
  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "ExposureControl");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExposureControl]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>("ExposureControl");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | --------------------------- gui -------------------------- |

  if (_gui_) {

    int flags = cv::WINDOW_NORMAL | cv::WINDOW_NORMAL;
    cv::namedWindow("exposure_depend", flags);
    cv::namedWindow("tag_detect", flags);

  }

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);
  image_transport::ImageTransport it_2(nh);

  // | ----------------- initialize subscribers ----------------- |
  sub_image_       = it.subscribe("image_raw", 1, &ExposureControl::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &ExposureControl::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());
  tag_image_subscription_ = it_2.subscribe("tag_image", 1, &ExposureControl::callbackTagImage, this);

  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);
  exposure_published = it.advertise("exposure_published", 1); 
  gain_published = it.advertise("gain_published", 1);
  tag_detect = it.advertise("tag_detect", 1);
  pub_info_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 100);
  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &ExposureControl::callbackTimerCheckSubscribers, this);


   srv_server_increase_shutter_speed_ = nh.advertiseService("srv_server_increase_shutter_speed_", &ExposureControl::callbackIncreaseShutterSpeed, this);


   srv_server_decrease_shutter_speed_ = nh.advertiseService("srv_server_decrease_shutter_speed_", &ExposureControl::callbackDecreaseShutterSpeed, this);
   
   srv_server_set_new_gain_ = nh.advertiseService("srv_server_set_new_gain_", &ExposureControl::callbackSetNewGain, this);

  
   ROS_INFO_ONCE("initialized==true");

  is_initialized_ = true;


}


void ExposureControl::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  got_camera_info_       = true;
  time_last_camera_info_ = ros::Time::now();

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(*msg);
  
  //tag_detect.publish(*msg);
}


void ExposureControl::callbackTagImage(const sensor_msgs::ImageConstPtr& msg) {


  if (!is_initialized_) {
    return;
  }

  const std::string color_encoding     = "bgr8";
  const std::string grayscale_encoding = "mono8";

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_image_ = true;
    image_counter_++;
    time_last_image_ = ros::Time::now();
  }

  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  const std_msgs::Header           msg_header       = msg->header;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, color_encoding);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat dImg = cv_ptr->image;

  if (_gui_) {
    cv::imshow("tag_detect", dImg);
  }

  ExposureControl::publishOpenCVImage(std::ref(dImg), msg_header, color_encoding, tag_detect);

}

void ExposureControl::ShowExpImage(cv::Mat &image){
  
  image  = image * (sqrt(static_cast<double>(exposure_slider_value)) / 100.0);
  /* show the image in gui (!the image will be displayed after calling cv::waitKey()!) */
  cv::imshow("exposure_depend", image);
}

void ExposureControl::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  const std::string color_encoding     = "bgr8";
  const std::string grayscale_encoding = "mono8";

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_image_ = true;
    image_counter_++;
    time_last_image_ = ros::Time::now();
  }

  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  const std_msgs::Header           msg_header       = msg->header;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, color_encoding);
  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat dImg = cv_ptr->image;
  cv::Mat image = dImg;
  std::mutex m1;

  m1.lock();

  if (flag_save == 0) {
    flag_save = 2;
    default_gain = gain_calculation(std::ref(dImg));
    //gain_to_set = default_gain;
  }
  m1.unlock();
  cv::createTrackbar ("Change exposure", "exposure_depend", &exposure_slider_value,max_exposure);

  
  if (_gui_) {
    ExposureControl::ShowExpImage(std::ref(dImg));
  }
  
  image  = image * pow((gain_to_set / default_gain), 0.5);

  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[ExposureContrl]: Total of %u images received so far", (unsigned int)image_counter_);
  ExposureControl::publishGainImage(std::ref(image), msg_header, color_encoding, gain_published, pub_info_, camera_model_); 
  ExposureControl::publishOpenCVImage(std::ref(dImg), msg_header, color_encoding, exposure_published); 

  /* publish image count */
  ExposureControl::publishImageNumber(image_counter_);

  if (_gui_) {
    /* !!! needed by OpenCV to correctly show the images using cv::imshow !!! */
    cv::waitKey(1);
  }
}


void ExposureControl::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!got_image_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera image since node launch.");
  }

  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera info msg since node launch.");
  }
}


void ExposureControl::publishImageNumber(uint64_t count) {

  std_msgs::UInt64 message;

  /* set the value */
  message.data = count;

  /* publish the message */
  try {
    pub_test_.publish(message);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_test_.getTopic().c_str());
  }
}


void ExposureControl::publishOpenCVImage(cv::Mat &image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {

  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;

  // Set the desired message header (time stamp and frame id)
  bridge_image_out.header = header;

  // Copy the cv::Mat, pointing to the image
  bridge_image_out.image = image;

  // Fill out the message encoding - this tells ROS how to interpret the raw image data
  // (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  // ... and publish the message
  pub.publish(out_msg);
}


void ExposureControl::publishGainImage(cv::Mat &image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub, const ros::Publisher pub_info, image_geometry::PinholeCameraModel camera_model_) {

  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;

  // Set the desired message header (time stamp and frame id)
  bridge_image_out.header = header;

  // Copy the cv::Mat, pointing to the image
  bridge_image_out.image = image;

  // Fill out the message encoding - this tells ROS how to interpret the raw image data
  // (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  bridge_image_out.encoding = encoding;
  const sensor_msgs::CameraInfo ci = camera_model_.cameraInfo();
  // Now convert the cv_bridge image to a ROS message
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  // ... and publish the message
  pub_info.publish(ci);
  pub.publish(out_msg);
}



double ExposureControl::gain_calculation(cv::Mat &dImg){
  
  cv::Scalar meann, stdv;
  cv::meanStdDev(dImg, meann, stdv, cv::Mat());

  float mean_Mat = meann.val[0];
  float stan_dev = stdv.val[0];
  
  float variation = (stan_dev * stan_dev);

  float gain = variation / mean_Mat;
  float gain_logarithmic_power = gain / 10.0;
  float gain_logarithmic = pow(10.0, gain_logarithmic_power);
  return static_cast<double>(gain_logarithmic);

}

bool ExposureControl::callbackSetNewGain(exposure_simulation::SetNewGain::Request &req,  exposure_simulation::SetNewGain::Response &res){


  if (!is_initialized_) {
    ROS_WARN("[ExposureControl]: Cannot set new gain!");
    return true;
  }
  mutex_counters_.lock();
  if (flag_save==1){
    flag_save = 0;
  }
  mutex_counters_.unlock();
  res.new_gain = req.new_gain; 
  double power_log = static_cast<double>(req.new_gain) / 10.0;
  
  gain_to_set  = pow(10.0, power_log);
  std::string ros_message = "[ExposureControl] new gain  = " + std::to_string(gain_to_set);
  res.message =  ros_message;

  return true;

}
 
bool ExposureControl::callbackDecreaseShutterSpeed([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Exposure control not initialized!";
    ROS_WARN("[ExposureControl]: Cannot decrease shutter speed, nodelet is not initialized.");
    return true;
  }
  exposure_slider_value += 100;
  ROS_INFO("Shutter speed decreased by 100.0");
  res.success = true;
  std::string decreased_speed = std::to_string(exposure_slider_value);
  res.message = "Shutter speed is = " + decreased_speed;

  return true;
}


bool ExposureControl::callbackIncreaseShutterSpeed([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Exposure control not initialized!";
    ROS_WARN("[ExposureControl]: Cannot increase shutter speed, nodelet is not initialized.");
    return true;
  }

  if (exposure_slider_value > 100){
     exposure_slider_value -= 100;

     ROS_INFO("[ExposureControl]: shutter speed increased!");

     res.success = true;
     std::string increased_speed = std::to_string(exposure_slider_value);
     res.message = "Shutter speed is = " + increased_speed;

  } else if (exposure_slider_value == 100){

    exposure_slider_value = 0;

     ROS_INFO("[ExposureControl]: shutter speed is at it's highest value!");

     res.success = true;
     res.message = "Shutter speed is max";


  }
  else {
     ROS_INFO("[ExposureControl]: shutter speed cannot be increased any more!");

     res.success = true;
     res.message = "Shutter speed is already max!";

  }
  ROS_INFO("Shutter speed increased by 100.0");
  res.success = true;
  std::string increased_speed = std::to_string(exposure_slider_value);
  res.message = "Shutter speed is = " + increased_speed;

  return true;
}


}


/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(exposure_simulation::ExposureControl, nodelet::Nodelet);
