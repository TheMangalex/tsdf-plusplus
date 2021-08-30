// new class for room detection

#ifndef TSDF_PLUSPLUS_ROS_ROOM_DETECTOR_H_
#define TSDF_PLUSPLUS_ROS_ROOM_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <voxblox/core/common.h>

#include <tsdf_plusplus_ros/controller.h>


class RoomDetector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RoomDetector(const ros::NodeHandle& nh, const ros::NodeHandle&nh_private, std::shared_ptr<Controller> controller_ptr);

  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& cloud_msg);
 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  ros::Subscriber pcl_sub_;
  std::shared_ptr<Controller> controller_ptr_;

  bool first_pos_;
  bool half_step_;
  float first_angle_;
  float last_angle_;

  std::map<int, int> histogram_;

  // TF listener to lookup TF transforms.
  tf::TransformListener tf_listener_;



  //raken from controller
  bool lookupTransformTF(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);

};


#endif // TSDF_PLUSPLUS_ROS_ROOM_DETECTOR_H_