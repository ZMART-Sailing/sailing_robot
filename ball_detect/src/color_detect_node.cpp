// ros related
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>

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

#define PI M_PI
#define EPSILON 1e-9

using pixel = std::pair<int, int>;

namespace  {
	bool trackObjects = true;
	bool useMorphOps = true;
	//default capture width and height
	const int FRAME_WIDTH = 1280;
	const int FRAME_HEIGHT = 720;
	const int FRAME_RATE = 30;
	//max number of objects to be detected in frame
	int MAX_NUM_OBJECTS = 50;
	//minimum and maximum object area
	int MIN_OBJECT_AREA = 20*20;
	int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	//names that will appear at the top of each window
	const string windowName =  "Original Image";
	const string windowName1 = "HSV Image";
	const string windowName2 = "Thresholded Image";
	const string windowName3 = "After Morphological Operations";
	const string trackbarWindowName = "Trackbars";

  const string detectImgfn = "~/redBallDetected/";
  const string nodetectImgfn = "~/redBallNoDetected/";
}
//This function gets called whenever a trackbar position is changed
void on_trackbar( int, void* ){}
string intToString(int number);
string floatToString(float number);
//void createTrackbars();
void drawObject(int x, int y,Mat &frame);
void morphOps(Mat &thresh);
void trackFilteredObject(int &x, int &y, float &z, Mat threshold, Mat &cameraFeed, rs2::depth_frame* depth_frame, bool &isDetect);
void pixel2Point(const rs2::depth_frame* frame, pixel u, float* upoint, float& depth);
bool pixel2Heading(const rs2::depth_frame* frame, pixel u, float& heading, float& depth);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "red_ball_detection");
  ros::NodeHandle node_("~");

  deque<int> average_list(100, 0);
  deque<float> depth_list(100, 0);

  int empty_image_counter = 0;
  int empty_image_ignore = 50;

  int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;
  bool IS_SHOW;

  node_.param<int>("H_MIN", H_MIN, 0);
  node_.param<int>("H_MAX", H_MAX, 255);
  node_.param<int>("S_MIN", S_MIN, 0);
  node_.param<int>("S_MAX", S_MAX, 255);
  node_.param<int>("V_MIN", V_MIN, 0);
  node_.param<int>("V_MAX", V_MAX, 255);
  node_.param<bool>("IS_SHOW", IS_SHOW, false);

  bool isDetected = false;
  float depth = -1;

  ros::Publisher ball_detct_pub = node_.advertise<std_msgs::Bool>("ball_detection", 10);
  ros::Publisher ball_pos_pub = node_.advertise<geometry_msgs::Point32>("ball_position", 10);
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
  auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
  auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  //matrix storage for HSV image
  Mat HSV;
  //matrix storage for binary threshold image
  Mat threshold;
  //x and y values for the location of the object
  int x = 0, y = 0;
  float z = -1;

//  if (!access(detectImgfn.c_str(), 0)) {
//    mkdir(detectImgfn.c_str());
//  }
  //create slider bars for HSV filtering
//	createTrackbars();
  //start an infinite loop where webcam feed is copied to cameraFeed matrix
  //all of our operations will be performed within this loop
  while(ros::ok()){

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

    //convert frame from BGR to HSV colorspace
    cvtColor(color_image, HSV, COLOR_BGR2HSV);
    //filter HSV image between values and store filtered image to
    //threshold matrix
    inRange(HSV, Scalar(H_MIN,S_MIN,V_MIN), Scalar(H_MAX,S_MAX,V_MAX),threshold);
    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if(useMorphOps)
      morphOps(threshold);
    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if(trackObjects)
      trackFilteredObject(x,y,z,threshold,color_image,&depth_frame, isDetected);

    if(isDetected)
    {
      average_list.pop_front();
      average_list.push_back(1);
      depth_list.pop_front();
      depth_list.push_back(z);
    }
    else {
      average_list.pop_front();
      average_list.push_back(0);
      depth_list.pop_front();
      depth_list.push_back(0);
    }

    time_t tt;
    time(&tt);
    tt += 8*3600;
    tm* t = gmtime(&tt);
    string str;
    stringstream convert;
    cout << "average_list.size = " << average_list.size() << endl;
    for(size_t i = 0; i < average_list.size(); i++)
      cout << average_list[i] << " ";
    cout << endl;
    cout << "depth_list.size = " << depth_list.size() << endl;
//    for(size_t i = 0; i < depth_list.size(); i++)
//      cout << depth_list[i] << " ";
//    cout << endl;
    float score = 1.0*accumulate(average_list.begin(), average_list.end(), 0) / average_list.size();
    cout << "score = " << score << endl;
    if (score > 0.5) {
      detect_msg.data = true;
      ball_pos.x = x;
      ball_pos.y = y;
      ball_pos.z = z;
      convert << "camera_detected_obstacle_" << t->tm_year + 1900 << "-" << t->tm_mon + 1 << "-" << t->tm_mday << "_" << t->tm_hour << ":" << t->tm_min << ":" << t->tm_sec << ".jpg";
      str = convert.str();
      cout << "yes wirte to :" << str << endl;
      imwrite(str, color_image);
    }
    else {
      empty_image_counter += 1;
      if(empty_image_counter > empty_image_ignore) {
        detect_msg.data = false;
        ball_pos.x = x;
        ball_pos.y = y;
        ball_pos.z = -1;
        convert << "camera_detected_obstacle_" << t->tm_year + 1900 << "-" << t->tm_mon + 1 << "-" << t->tm_mday << "_" << t->tm_hour << ":" << t->tm_min << ":" << t->tm_sec << "_nothing.jpg";
        str = convert.str();
        cout << "no wirte to :" << str << endl;
        imwrite(str, color_image);
        empty_image_counter = 0;
      }
    }

    ball_detct_pub.publish(detect_msg);
    ball_pos_pub.publish(ball_pos);

    if(IS_SHOW)
    {
        imshow(windowName2,threshold);
        imshow(windowName,color_image);
        waitKey(30);
    }
  }
		return 0;
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
//void createTrackbars(){
//	//create window for trackbars
//	namedWindow(trackbarWindowName,0);
//	//create memory to store trackbar name on window
//	char TrackbarName[50];
//	sprintf( TrackbarName, "H_MIN", H_MIN);
//	sprintf( TrackbarName, "H_MAX", H_MAX);
//	sprintf( TrackbarName, "S_MIN", S_MIN);
//	sprintf( TrackbarName, "S_MAX", S_MAX);
//	sprintf( TrackbarName, "V_MIN", V_MIN);
//	sprintf( TrackbarName, "V_MAX", V_MAX);
//	//create trackbars and insert them into window
//	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
//	//the max value the trackbar can move (eg. H_HIGH),
//	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
//	//                                  ---->    ---->     ---->
//	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
//	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
//	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
//	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
//	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
//	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
//	createTrackbar( "MAX_NUM_OBJECTS", trackbarWindowName, &MAX_NUM_OBJECTS, 100, on_trackbar );
//	createTrackbar( "MIN_OBJECT_AREA", trackbarWindowName, &MIN_OBJECT_AREA, FRAME_HEIGHT*FRAME_WIDTH, on_trackbar );
//	createTrackbar( "MAX_OBJECT_AREA", trackbarWindowName, &MAX_OBJECT_AREA, FRAME_HEIGHT*FRAME_WIDTH, on_trackbar );
//}

void drawObject(int x, int y, float depth, Mat &frame){
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
	if(y-25>0)
	line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
	if(y+25<FRAME_HEIGHT)
	line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
	if(x-25>0)
	line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
	if(x+25<FRAME_WIDTH)
	line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
	else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);
	putText(frame,intToString(x)+","+intToString(y)+","+floatToString(depth),Point(x,y+30),1,1,Scalar(0,255,0),2);
}

void morphOps(Mat &thresh){
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(int &x, int &y, float &z, Mat threshold, Mat &cameraFeed, rs2::depth_frame* depth_frame, bool &isDetect){
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			Mat dst;
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				cameraFeed.copyTo(dst, temp);
				// draw every contours
				drawContours(dst, contours, index, cv::Scalar(255));
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
//				cout << "area = " << area << endl;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;
			}
			//let user know you found an object
      if(objectFound == true){
				pixel p = {x, y};
				float depth, heading;
				pixel2Heading(depth_frame, p, heading, depth);
        cout << "depth = " << depth << endl;
        z = depth;
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,depth,cameraFeed);
			}
      else {
        z = -1;
      }
	}
			else 
				putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
  isDetect = objectFound;
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
