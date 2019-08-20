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
  const int IMAGE_AREA = FRAME_WIDTH * FRAME_HEIGHT;
  const int FRAME_RATE = 30;
  int empty_image_counter = 0;
  int detect_image_counter = 0;
  const int empty_image_ignore = 20;
  const int detect_image_confirm = 30;

  //max number of objects to be detected in frame
  const int MAX_NUM_OBJECTS = 50;
  //minimum and maximum object area
  const int MIN_OBJECT_AREA = 20*20;
  const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

  pixel center(0,0);
  deque<int> average_list(50, 0);
  deque<float> depth_list(50, 0);
  bool isDetected = false;
  float depth = -1;
  float pos[3];
  rs2::frame color_frame;
  const string detectImgfn = "~/redBallDetected/";
  const string nodetectImgfn = "~/redBallNoDetected/";
}

string intToString(int number);
string floatToString(float number);

void pixel2Point(const rs2::depth_frame* frame, pixel u, float* upoint, float& depth);
bool pixel2Heading(const rs2::depth_frame* frame, pixel u, float& heading, float& depth);
void yoloCallBack(const ball_detect::BoundingBoxes::ConstPtr& msg);
void detectCallBack(const std_msgs::Int8::ConstPtr& msg);
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "red_ball_detection");
  ros::NodeHandle node_("~");

  image_transport::ImageTransport it(node_);
  image_transport::Publisher color_image_pub = it.advertise("image_raw", 10);
  ros::Publisher ball_detct_pub = node_.advertise<std_msgs::Bool>("ball_detection", 10);
  ros::Publisher ball_pos_pub = node_.advertise<geometry_msgs::Point32>("ball_position", 10);
  ros::Subscriber detect_sub = node_.subscribe("/darknet_ros/found_object", 1000, detectCallBack);
  ros::Subscriber boxes_sub = node_.subscribe("/darknet_ros/bounding_boxes", 1000, yoloCallBack);
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
  //声明数据流
  auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();


  while(ros::ok()) {
    std_msgs::Bool detect_msg;
    geometry_msgs::Point32 ball_pos;

    rs2::frameset frameset = pipe.wait_for_frames();
    //取深度图和彩色图
    rs2::frame color_frame = frameset.get_color_frame();
    rs2::depth_frame depth_frame = frameset.get_depth_frame();

    //获取宽高
    const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
    const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
    const int color_w = color_frame.as<rs2::video_frame>().get_width();
    const int color_h = color_frame.as<rs2::video_frame>().get_height();

    //创建OPENCV类型 并传入数据
    Mat depth_image(Size(depth_w,depth_h), CV_16U,  (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    Mat depth_new(Size(depth_w,depth_h), CV_8UC3,  (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    Mat color_image(Size(color_w,color_h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    cv_bridge::CvImage image_pub_msg;
    image_pub_msg.header.stamp = ros::Time::now();
    image_pub_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_pub_msg.image = color_image;
    color_image_pub.publish(image_pub_msg.toImageMsg());

//    for (int i = 0; i < average_list.size(); i++)
//    {
//      cout << average_list[i] << " ";
//    }
//    cout << endl;
    cout << "isDetected = " << isDetected << endl;
    if (isDetected) {
      pixel2Point(&depth_frame, center, pos, depth);
      average_list.pop_front();
      average_list.push_back(1);
      depth_list.pop_front();
      depth_list.push_back(depth);
      detect_msg.data = true;
      ball_pos.x = pos[0];
      ball_pos.y = pos[1];
      ball_pos.z = pos[2];
    }
    else {
      detect_msg.data = false;
      ball_pos.x = -1;
      ball_pos.y = -1;
      ball_pos.z = -1;
      average_list.pop_front();
      average_list.push_back(0);;
      depth_list.pop_back();
      depth_list.push_back(0);
    }

    float score = 1.0 * accumulate(average_list.begin(), average_list.end(), 0) / average_list.size();
    cout << "score = " << score << endl;
    if (score < 0.5) {
      detect_msg.data = false;
      ball_pos.x = pos[0];
      ball_pos.y = pos[0];
      ball_pos.z = -1;
      cout << "no, fuck" << endl;
      empty_image_counter = 0;
    }
    else {
      detect_msg.data = true;
      ball_pos.x = pos[0];
      ball_pos.y = pos[1];
      ball_pos.z = pos[2];
      cout << "yes, hhhh" << endl;
    }

    ball_detct_pub.publish(detect_msg);
    ball_pos_pub.publish(ball_pos);

    ros::spinOnce();//这句就是同时发布节点和订阅节点的关键了
    loop_rate.sleep();
  }
    return 0;
}
void detectCallBack(const std_msgs::Int8::ConstPtr& msg) {
  if (msg->data == true) {
    isDetected = true;
  }
  else {
    isDetected = false;
  }
}

void yoloCallBack(const ball_detect::BoundingBoxes::ConstPtr& msg) {
  int area = 0, x, y;
  double prob = 0;
  double AREA_THRESHOLD = 0.01;
  for(int i = 0; i < msg->bounding_boxes.size(); i++) {
    int area_tmp = abs(msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin) * (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
    if ((1.0 * area_tmp / IMAGE_AREA) > AREA_THRESHOLD && area_tmp > area &&  msg->bounding_boxes[i].probability > prob) {
      area = area_tmp;
      x = (msg->bounding_boxes[i].xmax + msg->bounding_boxes[i].xmin) / 2;
      y = (msg->bounding_boxes[i].ymax + msg->bounding_boxes[i].ymin) / 2;
      prob = msg->bounding_boxes[i].probability;
    }
  }
  if(area > 0) {
//    cout << "ball detected!!" << " x = " << x << " y = " << y << endl;
    center.first = x;
    center.second = y;
  }

  else {
    isDetected = false;
    cout << "no detected!!" << endl;
    center.first = 0;
    center.second = 0;
  }

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

void pixel2Point(const rs2::depth_frame* frame, pixel u, float* upoint, float& depth)
{
  float upixel[2]; // From pixel

  // Copy pixels into the arrays (to match rsutil signatures)
  upixel[0] = u.first;
  upixel[1] = u.second;

  // Query the frame for distance
  auto udist = frame->get_distance(upixel[0], upixel[1]);
  depth = udist;
  // Deproject from pixel to point in 3D
  rs2_intrinsics intr = frame->get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
  rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
  return ;
}

bool pixel2Heading(const rs2::depth_frame* frame, pixel u, float& heading, float& depth)
{
  float upoint[3];
  pixel2Point(frame, u, upoint, depth);

  if (abs(upoint[2]) < EPSILON) {
    return false;
  }
  else {
    if (upoint[0] > 0)
      heading = atan2f(upoint[0], upoint[2]);
    else
      heading = atan2f(upoint[0], upoint[2]) + PI * 2;
    return true;
  }
}
