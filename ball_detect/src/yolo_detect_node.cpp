// ros related
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ball_detect/BoundingBox.h>
#include <ball_detect/BoundingBoxes.h>
#include <sensor_msgs/NavSatFix.h>
#include <ball_detect/BoatAndBall.h>

// rs and opencv related
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <deque>
#include <numeric>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace cv;
using namespace message_filters;


#define PI M_PI
#define EPSILON 1e-9

using pixel = std::pair<int, int>;

namespace  {

  //default capture width and height
  const int FRAME_WIDTH = 1280;
  const int FRAME_HEIGHT = 720;
  const int FRAME_RATE = 30;

  const float DEPTH_MIN = 0.2;
  const float DEPTH_MAX = 10;

  bool depth_lock = false;
  bool first_flag = true;
  uint16_t* data = NULL;
  float center[2];
  deque<int> detect_list(20, 0);
  deque<float*> pos_list(20);
  bool isDetected = false;
  float depth = -1;
  float heading = 0;
  float pos[3];
  rs2::frame color_frame;
  sensor_msgs::NavSatFix nav_sub = sensor_msgs::NavSatFix();
  sensor_msgs::NavSatFix cur_nav = sensor_msgs::NavSatFix();
}

string intToString(int number);
string floatToString(float number);

void pixel2Point(const rs2::depth_frame* frame, const rs2_intrinsics* intrin, float u[2], float* upoint, float& depth);
bool pixel2Heading(const rs2::depth_frame* frame, const rs2_intrinsics* intrin, float u[2], float& heading, float& depth);
void yoloCallBack(const ball_detect::BoundingBoxes::ConstPtr& msg);
void detectCallBack(const std_msgs::Int8::ConstPtr& msg);
void posCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);
float depthCubicCalibration(const float depth);
float getScore(deque<int>* dtc_list);
float* getAveragePos(deque<int>* dtc_list, deque<float*>* pos_list);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "red_ball_detection");
  ros::NodeHandle node_("~");

  image_transport::ImageTransport it(node_);
  //发布彩色图给yolo处理
  image_transport::Publisher color_image_pub = it.advertise("image_raw", 10);
  //从yolo订阅检测信息
  ros::Subscriber detect_sub = node_.subscribe("/darknet_ros/found_object", 1000, detectCallBack);
  ros::Subscriber boxes_sub = node_.subscribe("/darknet_ros/bounding_boxes", 1000, yoloCallBack);
  ros::Subscriber boat_pos_sub = node_.subscribe("/position", 1000, posCallBack);

  //发布经过处理的检测信息
  ros::Publisher boat_and_ball_pub = node_.advertise<ball_detect::BoatAndBall>("boat_and_ball", 10);

  ros::Rate loop_rate(10);

  rs2::colorizer c;
  //创建数据管道
  rs2::pipeline pipe;
  //创建一个以非默认配置的配置用来配置管道
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);
  cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_BGR8, FRAME_RATE);
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Y8, FRAME_RATE);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_Y8, FRAME_RATE);
  //start()函数返回数据管道的profile
  rs2::pipeline_profile profile = pipe.start(cfg);

  rs2::device dev = profile.get_device();
  rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
  float depth_scale = ds.get_depth_scale();
  //声明数据流
  auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  int frame_cnt = 0;
  while(ros::ok()) {
    frame_cnt += 1;
    // cout << "begin" << ros::Time::now() << endl;
    //如果被上锁了，直接sleep这一帧
    if (depth_lock) {
      ros::spinOnce();
      // cout << "locked" << endl;
    }

    else {
      // cout << "unlocked" << endl;
      rs2::frameset frameset = pipe.wait_for_frames();
      //取深度图和彩色图
      rs2::frame color_frame = frameset.get_color_frame();
      rs2::depth_frame depth_frame = frameset.get_depth_frame();

      auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
      auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

      //获取相机的内参和外参矩阵
      const rs2_intrinsics depth_intrin = depth_profile.get_intrinsics();
      const rs2_intrinsics color_intrin = color_profile.get_intrinsics();
      const rs2_extrinsics c2dextrin = color_profile.get_extrinsics_to(depth_profile);
      const rs2_extrinsics d2cextrin = depth_profile.get_extrinsics_to(color_profile);
      //获取宽高
      const int color_w = color_frame.as<rs2::video_frame>().get_width();
      const int color_h = color_frame.as<rs2::video_frame>().get_height();
      Mat color_image(Size(color_w,color_h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

      sensor_msgs::NavSatFix nav_pub;
      //如果是第一帧,不calculate and publish result, only update data, and lock
      if (first_flag) {
        // cout << "first" << endl;
        //发布彩色图
        cv_bridge::CvImage image_pub_msg;
        image_pub_msg.header.stamp = ros::Time::now();
        image_pub_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_pub_msg.image = color_image;
        color_image_pub.publish(image_pub_msg.toImageMsg());
        data = (uint16_t*)(depth_frame.get_data());
        first_flag = false;
      }
      //否则
      else {
        // cout << "unlock" << endl;
        // first calculate result and publish it
        ball_detect::BoatAndBall boat_and_ball_msg;
        if (isDetected) {
          float depth_pixel[2];
          rs2_project_color_pixel_to_depth_pixel(depth_pixel, data, depth_scale, DEPTH_MIN, DEPTH_MAX, \
                                                         &depth_intrin, &color_intrin, &c2dextrin, &d2cextrin, center);
          if(pixel2Heading(&depth_frame, &depth_intrin, depth_pixel, heading, depth)) {
            boat_and_ball_msg.isDetected = true;
            boat_and_ball_msg.heading = heading;
            boat_and_ball_msg.depth = depth;
          }
          else {
            boat_and_ball_msg.isDetected = false;
            boat_and_ball_msg.heading = 0;
            boat_and_ball_msg.depth = 0;
          }
          boat_and_ball_msg.boat_pos = cur_nav;
        }
        else {
          boat_and_ball_msg.isDetected = false;
          boat_and_ball_msg.heading = 0;
          boat_and_ball_msg.depth = 0;
          boat_and_ball_msg.boat_pos = cur_nav;
        }
        boat_and_ball_pub.publish(boat_and_ball_msg);

        //update data
        cv_bridge::CvImage image_pub_msg;
        image_pub_msg.header.stamp = ros::Time::now();
        image_pub_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_pub_msg.image = color_image;
        color_image_pub.publish(image_pub_msg.toImageMsg());
        data = (uint16_t*)(depth_frame.get_data());
      }
      //继续上锁
      depth_lock = true;
      ros::spinOnce();
      cur_nav = nav_sub;
    }
    //sleep
    loop_rate.sleep();

  }
    return 0;
}

void detectCallBack(const std_msgs::Int8::ConstPtr& msg) {
  // cout << "detectCallBack" << endl;
  depth_lock = false;
  if (msg->data == true) {
    isDetected = true;
  }
  else {
    isDetected = false;
  }
}

void yoloCallBack(const ball_detect::BoundingBoxes::ConstPtr& msg) {
  // cout << "yoloCallBack" << endl;
  int area = 0, x, y;
  double prob = 0;
  for(size_t i = 0; i < msg->bounding_boxes.size(); i++) {
    int area_tmp = abs(msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin) * abs(msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
    if (msg->bounding_boxes[i].probability > prob) {
      area = area_tmp;
      x = (msg->bounding_boxes[i].xmax + msg->bounding_boxes[i].xmin) / 2;
      y = (msg->bounding_boxes[i].ymax + msg->bounding_boxes[i].ymin) / 2;
      prob = msg->bounding_boxes[i].probability;
    }
  }
  if(area > 0) {
    center[0] = x;
    center[1] = y;
  }
  else {
    isDetected = false;
    center[0] = 0;
    center[1] = 0;
  }
}

void posCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // cout << "posCallBack" << endl;
    nav_sub.header = msg->header;
    nav_sub.status = msg->status;
    nav_sub.altitude = msg->altitude;
    nav_sub.latitude = msg->latitude;
    nav_sub.longitude = msg->longitude;
    nav_sub.position_covariance = msg->position_covariance;
}

float getScore(deque<int>* dtc_list) {
  return 1.0 * accumulate(dtc_list->begin(), dtc_list->end(), 0) / dtc_list->size();
}

float* getAveragePos(deque<int>* dtc_list, deque<float*>* pos_list) {
  float pos[3];
  float sum = 0;
  for(int i = 0; i < dtc_list->size(); i++) {
    if(dtc_list->at(i) > 0 && pos_list->at(i)[0] > 0 && pos_list->at(i)[1] > 0 && pos_list->at(i)[2] > 0) {

    }
  }
  return pos;
}

string intToString(int number){
  std::stringstream ss;
  ss << number;
  return ss.str();
}
string floatToString(float number){
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void pixel2Point(const rs2::depth_frame* frame, const rs2_intrinsics* intrin, float upixel[2], float* upoint, float& depth)
{
  auto udist = frame->get_distance(upixel[0], upixel[1]);
  depth = udist;
  rs2_deproject_pixel_to_point(upoint, intrin, upixel, udist);
  return ;
}

float depthCubicCalibration(const float depth)
{
  return 0.0172 * pow(depth, 3) - 0.0814 * pow(depth, 2) + 1.5038 * depth - 0.4396;
}

bool pixel2Heading(const rs2::depth_frame* frame, const rs2_intrinsics* intrin, float upixel[2], float& heading, float& depth)
{
 float upoint[3];
 pixel2Point(frame, intrin, upixel, upoint, depth);

 if (abs(upoint[2]) < EPSILON) {
   return false;
 }
 else {
   if (upoint[0] > 0)
     heading = atan2f(upoint[0], upoint[2]) * 180 / PI;
   else
     heading = (atan2f(upoint[0], upoint[2]) + PI * 2) * 180 / PI;
   return true;
 }
}
