/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"
#include <iostream>
#include <pluginlib/class_list_macros.h>

using namespace std;
PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{

ContinuousDetector::ContinuousDetector ()
{
}

void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));
  // subscribe topic
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);

  boat_ori_subscriber_ = nh.subscribe("boat_orient", 1, &ContinuousDetector::orientCallback, this);
  boat_pos_subscriber_ = nh.subscribe("boat_pos", 1, &ContinuousDetector::posCallBack, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }
}

void ContinuousDetector::orientCallback(
    const std_msgs::Bool msg)
{
  boat_orient_ = msg.data;
}
void ContinuousDetector::posCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  boat_pos_.latitude = msg->latitude;
  boat_pos_.longitude = msg->longitude;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
//  cout << camera_info << endl;
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(
      tag_detector_->detectTags(cv_image_,camera_info,boat_pos_,boat_orient_));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_, boat_pos_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
