#include "tsdf_plusplus_ros/room_detector.h"

RoomDetector::RoomDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, std::shared_ptr<Controller> controller_ptr) 
    : nh_(nh), nh_private_(nh_private), controller_ptr_(controller_ptr), first_pos_(false), first_angle_(0.0f)
    {
        std::string pcl_topic = "/scan_cloud_filtered";
        pcl_sub_ = nh_.subscribe(pcl_topic, 1000, &RoomDetector::pointcloudCallback, this);
}


void RoomDetector::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& cloud_msg) {
    voxblox::Transformation lidar_trans;
    std::string frame1 = "spin_lidar_mount_link";
    std::string frame2 = "spin_lidar_mount_link_fixed";

    lookupTransformTF(frame1, frame2, cloud_msg->header.stamp, &lidar_trans);
    auto angles = lidar_trans.getRotationMatrix().eulerAngles(2,1,0);
    //std::cout << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;


    if (!first_pos_) {
        first_pos_ = true;
        half_step_ = false;
        //first_angle_ = angles[0];
        last_angle_ = angles[0];
        std::map<int, int> hist;
        histogram_ = hist;
    }
    voxblox::Transformation transform;
    std::string world_frame_ = "world";
    if(!lookupTransformTF(cloud_msg->header.frame_id, world_frame_, cloud_msg->header.stamp, &transform)) {
        return;
    }
    pcl::PointCloud<InputPointType> pointcloud_pcl;
    pcl::moveFromROSMsg(*cloud_msg, pointcloud_pcl);


    float histogram_height_cm = 5.0f;
    for(size_t i = 0u; i < pointcloud_pcl.points.size(); i++) {
        auto p = pointcloud_pcl.points[i];
        voxblox::Point point(p.x, p.y, p.z);
        point = transform * point;
        
        int z_pos = std::floor(point[2] * 100 / histogram_height_cm); //now in cm, maybe put in 5cm brackets later
        histogram_[z_pos] += 1;
    }

    /*float diff = angles[0] - first_angle_;
    while (diff < 0.0f) {
        diff += 2 * 3.14159;
    }*/
    /*if (diff > 3.14159) {
        half_step_ = true;
    }*/
    //if (diff < 3.14159 && half_step_) {
    if (last_angle_ > angles[0]) { 
        //rotation should be done, as angle is smaller than pi again
        first_pos_ = false;
        //half_step_ = false;

    std::map<int, int> possible_heights;
        //print histogram for now
        std::cout << "printing histogram" << std::endl;
        for (auto hist : histogram_) {
            if (hist.second > 2000) {
                std::cout << hist.first / 100.0 * histogram_height_cm << ": " << hist.second << " entries" << std::endl;
                possible_heights[hist.first] = hist.second;

            }
        }

        int res1 = 0;
        int confidence = 0;
        for (auto pair : possible_heights) {
            if (pair.second > confidence) {
                res1 = pair.first;
                confidence = pair.second;
            }
        }

        int res2 = 0;
        confidence = 0;
        for (auto pair : possible_heights) {
            if (pair.second > confidence && abs(pair.first - res1) > 5) {//at least distance of 5 to ceiling /floor
                res2 = pair.first;
                confidence = pair.second;
            }
        }

        float floor, ceiling;
        if (res1 < res2) {
            floor = res1;
            ceiling = res2;
        } else {
            floor = res2;
            ceiling = res1;
        }
        floor *=  histogram_height_cm / 100.0;
        ceiling *= histogram_height_cm / 100.0;

        std::cout << "floor: " << std::to_string(floor) << ", ceiling: " << std::to_string(ceiling) << std::endl;


        //TODO now check if this fits to an existing building floor, or create a new one
        //if fits, update it
        //if not, create new
        //TODO check for invalid ceiling
    }
}



bool RoomDetector::lookupTransformTF(const std::string& from_frame,
                                   const std::string& to_frame,
                                   const ros::Time& timestamp,
                                   Transformation* transform) {
  CHECK_NOTNULL(transform);

  tf::StampedTransform tf_transform;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;

  if (!tf_listener_.canTransform(to_frame, from_frame_modified, timestamp)) {
    ROS_ERROR_STREAM("Error getting TF transform from frame "
                     << from_frame_modified << " to frame " << to_frame << ".");
    return false;
  }

  try {
    tf_listener_.lookupTransform(to_frame, from_frame_modified, timestamp,
                                 tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}