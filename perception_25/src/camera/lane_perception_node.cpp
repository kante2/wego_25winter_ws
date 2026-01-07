// lane_perception_node.cpp
// Perception node: 카메라 구독 -> 차선 인지 -> 토픽 발행

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

// -------------------- 전역 상태 --------------------
static std::string g_camera_topic;
static std::string g_center_point_topic;
static std::string g_curvature_topic;
static std::string g_center_color_topic;

static ros::Subscriber g_image_sub;
static ros::Publisher g_center_point_pub;
static ros::Publisher g_curvature_pub;
static ros::Publisher g_center_color_pub;

// Image processing parameters
static double g_bev_center_x_px = 320.0;
static cv::Scalar g_hsv_lower;
static cv::Scalar g_hsv_upper;
static double g_roi_top_ratio = 0.3;
static double g_roi_bottom_ratio = 1.0;

// -------------------- Helper functions --------------------
static double findLaneCenter(const cv::Mat& mask, int width)
{
  int h = mask.rows;
  
  // Compute histogram for bottom half
  cv::Mat hist;
  int channels[] = {0};
  int histSize[] = {width};
  float range[] = {0, static_cast<float>(width)};
  const float* ranges[] = {range};
  
  cv::calcHist(&mask, 1, channels, cv::Mat(), hist, 1, histSize, ranges);
  
  // Find peaks in left and right regions
  int midpoint = width / 2;
  double left_val = 0, right_val = 0;
  int left_peak = 0, right_peak = midpoint;
  
  // Find left peak
  for (int i = 0; i < midpoint; i++) {
    if (hist.at<float>(i) > left_val) {
      left_val = hist.at<float>(i);
      left_peak = i;
    }
  }
  
  // Find right peak
  for (int i = midpoint; i < width; i++) {
    if (hist.at<float>(i) > right_val) {
      right_val = hist.at<float>(i);
      right_peak = i;
    }
  }
  
  // Determine center
  double center_x;
  const double min_pixels = 100.0;
  
  if (left_val > min_pixels && right_val > min_pixels) {
    center_x = (left_peak + right_peak) / 2.0;
  } else if (left_val > min_pixels) {
    center_x = left_peak + 80.0;
  } else if (right_val > min_pixels) {
    center_x = right_peak - 80.0;
  } else {
    center_x = width / 2.0;
  }
  
  return center_x;
}

static double estimateCurvature(const cv::Mat& mask, int width)
{
  int h = mask.rows;
  
  // Simple curvature: check imbalance of left/right halves
  cv::Mat left_half = mask(cv::Range::all(), cv::Range(0, width/2));
  cv::Mat right_half = mask(cv::Range::all(), cv::Range(width/2, width));
  
  double left_pixels = cv::countNonZero(left_half);
  double right_pixels = cv::countNonZero(right_half);
  
  double total = left_pixels + right_pixels;
  if (total < 1.0) {
    return 0.0;
  }
  
  double curvature = (right_pixels - left_pixels) / (total * width);
  
  return std::max(-1e-3, std::min(1e-3, curvature));
}

static int detectLaneColor(const cv::Mat& roi)
{
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  
  // Red: H around 0 or 180, high S and V
  cv::Mat red_mask1, red_mask2;
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
  cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), red_mask2);
  cv::Mat red_mask = red_mask1 | red_mask2;
  
  // Blue: H around 110-130
  cv::Mat blue_mask;
  cv::inRange(hsv, cv::Scalar(110, 100, 100), cv::Scalar(130, 255, 255), blue_mask);
  
  int red_pixels = cv::countNonZero(red_mask);
  int blue_pixels = cv::countNonZero(blue_mask);
  
  if (red_pixels > blue_pixels && red_pixels > 100) {
    return 1;
  } else if (blue_pixels > 100) {
    return 2;
  } else {
    return 0;
  }
}

static void publishPerceptionResults(double center_x, double curvature, int color_code)
{
  // Publish center point
  geometry_msgs::PointStamped center_msg;
  center_msg.header.stamp = ros::Time::now();
  center_msg.header.frame_id = "camera";
  center_msg.point.x = center_x;
  center_msg.point.y = 0;
  center_msg.point.z = 0;
  g_center_point_pub.publish(center_msg);
  
  // Publish curvature
  std_msgs::Float32 curv_msg;
  curv_msg.data = static_cast<float>(curvature);
  g_curvature_pub.publish(curv_msg);
  
  // Publish color
  geometry_msgs::PointStamped color_msg;
  color_msg.header.stamp = ros::Time::now();
  color_msg.header.frame_id = "camera";
  color_msg.point.x = static_cast<double>(color_code);
  color_msg.point.y = 0;
  color_msg.point.z = 0;
  g_center_color_pub.publish(color_msg);
}

// -------------------- Callback --------------------
static void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = 
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;
    
    if (frame.empty()) {
      ROS_WARN("[perception] Empty image received");
      return;
    }
    
    int height = frame.rows;
    int width = frame.cols;
    
    // Extract ROI (ratio-based)
    int roi_top = static_cast<int>(height * g_roi_top_ratio);
    int roi_bottom = static_cast<int>(height * g_roi_bottom_ratio);
    cv::Mat roi = frame(cv::Range(roi_top, roi_bottom), cv::Range::all());
    
    // Convert to HSV and create mask
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, g_hsv_lower, g_hsv_upper, mask);
    
    // Morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // Find lane center using histogram
    double center_x = findLaneCenter(mask, width);
    
    // Estimate curvature
    double curvature = estimateCurvature(mask, width);
    
    // Detect lane color
    int color_code = detectLaneColor(roi);
    
    // Publish results
    publishPerceptionResults(center_x, curvature, color_code);
    
    ROS_DEBUG_THROTTLE(1.0, "[perception] center_x=%.1f, curvature=%.4e, color=%d",
                      center_x, curvature, color_code);
    
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[perception] cv_bridge exception: %s", e.what());
  }
}

// -------------------- Init function --------------------
void perception_init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  ROS_INFO("[perception] perception_init()");
  
  // Load parameters
  pnh.param<std::string>("camera_topic", g_camera_topic, "/usb_cam/image_rect_color");
  pnh.param<std::string>("center_point_topic", g_center_point_topic, 
                         "/webot/lane_center");
  pnh.param<std::string>("curvature_topic", g_curvature_topic,
                         "/webot/lane_curvature");
  pnh.param<std::string>("center_color_topic", g_center_color_topic,
                         "/webot/lane_color");
  
  pnh.param<double>("bev_center_x_px", g_bev_center_x_px, 320.0);
  pnh.param<double>("roi_top_ratio", g_roi_top_ratio, 0.3);
  pnh.param<double>("roi_bottom_ratio", g_roi_bottom_ratio, 1.0);
  
  // HSV range for yellow lane detection (from lane_center_node)
  int h_low = 18, s_low = 100, v_low = 110;
  int h_high = 38, s_high = 255, v_high = 230;
  pnh.param<int>("hsv_h_low", h_low, 18);
  pnh.param<int>("hsv_s_low", s_low, 100);
  pnh.param<int>("hsv_v_low", v_low, 110);
  pnh.param<int>("hsv_h_high", h_high, 38);
  pnh.param<int>("hsv_s_high", s_high, 255);
  pnh.param<int>("hsv_v_high", v_high, 230);
  
  g_hsv_lower = cv::Scalar(h_low, s_low, v_low);
  g_hsv_upper = cv::Scalar(h_high, s_high, v_high);
  
  // Subscribe to camera
  g_image_sub = nh.subscribe(g_camera_topic, 5, imageCB);
  
  // Advertise perception results
  g_center_point_pub = nh.advertise<geometry_msgs::PointStamped>(
                        g_center_point_topic, 10);
  g_curvature_pub = nh.advertise<std_msgs::Float32>(
                        g_curvature_topic, 10);
  g_center_color_pub = nh.advertise<geometry_msgs::PointStamped>(
                        g_center_color_topic, 10);
  
  ROS_INFO("[perception] Camera topic: %s", 
          ros::names::resolve(g_camera_topic).c_str());
  ROS_INFO("[perception] Publishing center_point: %s",
          ros::names::resolve(g_center_point_topic).c_str());
  ROS_INFO("[perception] Publishing curvature: %s",
          ros::names::resolve(g_curvature_topic).c_str());
  ROS_INFO("[perception] Yellow lane detection: H(%.0f-%.0f) S(%.0f-%.0f) V(%.0f-%.0f)",
          (double)h_low, (double)h_high, (double)s_low, (double)s_high, 
          (double)v_low, (double)v_high);
  
  ROS_INFO("[perception] perception_init done");
}

// -------------------- Main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  perception_init(nh, pnh);
  
  ros::spin();
  
  return 0;
}
