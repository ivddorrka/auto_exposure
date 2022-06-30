

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


namespace automatic_exposure
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
  image_transport::Subscriber sub_image_;

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

  // | --------------- variables for edge detector -------------- |

  int       low_threshold_;
  int const max_low_threshold_ = 100;

  // | ------------- variables for point projection ------------- |
  double  shutter_speed = 1000.0;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher             pub_test_;
  int                        _rate_timer_publish_;

  // | --------------------- other functions -------------------- |
  bool               callbackDecreaseShutterSpeed(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_decrease_shutter_speed_;


  bool               callbackIncreaseShutterSpeed(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_increase_shutter_speed_;

  void publishOpenCVImage(cv::InputArray detected_edges, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);
  void publishImageNumber(uint64_t count);
};


void ExposureControl::onInit() {

  got_image_       = false;
  got_camera_info_ = false;

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "EdgeDetect");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("canny_threshold", low_threshold_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>("EdgeDetect");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | --------------------------- gui -------------------------- |

  if (_gui_) {

    int flags = cv::WINDOW_NORMAL;
    cv::namedWindow("camera's view", flags);

  }

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);

  // | -------------- initialize tranform listener -------------- |
  // the transform listener will fill the TF buffer with latest transforms
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  // | ----------------- initialize subscribers ----------------- |
  sub_image_       = it.subscribe("image_in", 1, &ExposureControl::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &ExposureControl::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  // | ------------------ initialize publishers ----------------- |
  pub_test_       = nh.advertise<std_msgs::UInt64>("test_publisher", 1);

  // | -------------------- initialize timers ------------------- |
  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &ExposureControl::callbackTimerCheckSubscribers, this);


   srv_server_increase_shutter_speed_ = nh.advertiseService("srv_server_increase_shutter_speed_", &ExposureControl::callbackIncreaseShutterSpeed, this);


   srv_server_decrease_shutter_speed_ = nh.advertiseService("srv_server_decrease_shutter_speed_", &ExposureControl::callbackDecreaseShutterSpeed, this);

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

  //const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
 // const std_msgs::Header           msg_header       = msg->header;
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

  dImg = dImg * (shutter_speed / 1000.0);
  /* show the image in gui (!the image will be displayed after calling cv::waitKey()!) */
  if (_gui_) {
    cv::imshow("camera's view", dImg);
  }

  /* output a text about it */
  ROS_INFO_THROTTLE(1, "[ExposureContrl]: Total of %u images received so far", (unsigned int)image_counter_);


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


void ExposureControl::publishOpenCVImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {

  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;

  // Set the desired message header (time stamp and frame id)
  bridge_image_out.header = header;

  // Copy the cv::Mat, pointing to the image
  bridge_image_out.image = image.getMat();

  // Fill out the message encoding - this tells ROS how to interpret the raw image data
  // (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  // ... and publish the message
  pub.publish(out_msg);
}




bool ExposureControl::callbackDecreaseShutterSpeed([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Exposure control not initialized!";
    ROS_WARN("[ExposureControl]: Cannot decrease shutter speed, nodelet is not initialized.");
    return true;
  }
  
  if (shutter_speed > 0.1){
     shutter_speed -= 100.0;

     ROS_INFO("[ExposureControl]: shutter speed decreased!");

     res.success = true;
     res.message = "Shutter speed decreased by 100 mcr-seconds";


  } else {
     ROS_INFO("[ExposureControl]: shutter speed cannot be decreased any more!");

     res.success = true;
     res.message = "Shutter speed is already 0!";

  }


  return true;
}




bool ExposureControl::callbackIncreaseShutterSpeed([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Exposure control not initialized!";
    ROS_WARN("[ExposureControl]: Cannot increase shutter speed, nodelet is not initialized.");
    return true;
  }

  ROS_INFO("[ExposureControl]: shutter speed increased!");

  res.success = true;
  res.message = "Shutter speed increased by 100 mcr-seconds";

  return true;
}




}


/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(automatic_exposure::ExposureControl, nodelet::Nodelet);
