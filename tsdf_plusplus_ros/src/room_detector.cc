#include "tsdf_plusplus_ros/room_detector.h"

RoomDetector::RoomDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, std::shared_ptr<Controller> controller_ptr) 
    : nh_(nh), nh_private_(nh_private), controller_ptr_(controller_ptr), first_pos_(false), first_angle_(0.0f), new_layer_(false), publish_floor_(false)
    {
        std::string pcl_topic = "/scan_cloud_filtered";
        pcl_sub_ = nh_.subscribe(pcl_topic, 1000, &RoomDetector::pointcloudCallback, this);

        std::string tsdf_map_topic = "voxblox_node/tsdf_map_out";
        tsdf_sub_ = nh_.subscribe(tsdf_map_topic, 1, &RoomDetector::tsdf_layer_callback, this);
        slice_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("sdf_slice", 1);
      floor_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("floor", 1);
    

    

    FloatingPoint voxel_size = 0.1;
    size_t voxels_per_side = 16;

    //TODO maybe do this every callback and only get updates this way??
    voxblox::Layer<voxblox::TsdfVoxel> layer(voxel_size, voxels_per_side);
    tsdf_map_ptr_ = std::make_shared<voxblox::TsdfMap>(layer);
    
    rooms_and_floors_ = std::make_shared<RoomsAndFloors>();
    rooms_and_floors_->floors = std::make_shared<std::vector<Floor::Ptr>>();
    rooms_and_floors_->room_ids = std::make_shared<std::map<uint, bool>>();

    //pass rooms_and_floors_struct
    controller_ptr_->addFloorsAndRoomIds(rooms_and_floors_);



    //debug pub 
    publish_floor_ = true;
      
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


    float histogram_height_cm = 10.0f;
    for(size_t i = 0u; i < pointcloud_pcl.points.size(); i++) {
        auto p = pointcloud_pcl.points[i];
        voxblox::Point point(p.x, p.y, p.z);
        point = transform * point;
        
        int z_pos = std::floor(point[2] * 100 / histogram_height_cm); //now in cm, maybe put in 5cm brackets later
        histogram_[z_pos] += 1;
    }

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

        double floor_h, ceiling_h;
        if (res1 < res2) {
            floor_h = res1;
            ceiling_h = res2;
        } else {
            floor_h = res2;
            ceiling_h = res1;
        }
        floor_h *=  histogram_height_cm / 100.0;
        ceiling_h *= histogram_height_cm / 100.0;

        std::cout << "floor: " << std::to_string(floor_h) << ", ceiling: " << std::to_string(ceiling_h) << std::endl;


        //TODO now check if this fits to an existing building floor, or create a new one
        //if fits, update it
        //if not, create new
        //TODO check for invalid ceiling

        //this can not be multithreaded, as no mutex exists
        if(!new_layer_) {
            std::cout << "error, no new tsdf layer available" << std::endl;
        } else {
        //auto tsdf_layer_ptr = tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr();
        /*if(!tsdf_layer_ptr) {
            
        }*/
        
        //getting submap slice like cblox
        auto slice_layer_ptr = getTsdfSlice(tsdf_map_ptr_->getTsdfLayerPtr(), ceiling_h - 0.3f); //get slice below ceiling
        visualizeSlice(slice_layer_ptr);
        
        int count = 0;
        std::cout << "detector lock" << std::endl;
        rooms_and_floors_->mut.lock();
        auto floors = rooms_and_floors_->floors;
        for (auto floor : *floors) {
            if(floor->inFloor(ceiling_h - 0.3f)) { //if slice in floor, update
                floor->updateWithSlice(slice_layer_ptr);

                if(publish_floor_) {
                    std::cout << "visualize floor" << std::endl;
                    visualizeFloor(floor);
                }
                count++;
            }

        }
        if(count == 0) {
            //TODO try to use floor value also to recognize wrongly created floors
            if(ceiling_h - floor_h < 1.0) {
                std::cout << "to low height to create new floor" << std::endl;
            } else {

            Floor::Ptr floor = std::make_shared<Floor>(floor_h, ceiling_h, rooms_and_floors_);
            floors->push_back(floor);

            if(publish_floor_) {
                std::cout << "visualize floor" << std::endl;
                visualizeFloor(floor);
            }
            }
        }
        rooms_and_floors_->mut.unlock();
        std::cout << "detector unlock" << std::endl;
        //TODO add floors_ and room_ids_ to class vars and also add this to object detection
        //test with preloaded rooms

        }
    }
}

void RoomDetector::tsdf_layer_callback(const voxblox_msgs::Layer::Ptr& layer_msg) {
    
  
     
    std::cout << "layer callback" << std::endl;
    FloatingPoint voxel_size = 0.05;
    size_t voxels_per_side = 16;
    

    //voxblox::Layer<voxblox::TsdfVoxel> tsdf_layer_ = std::make_shared(voxel_size, voxels_per_side);
    bool success = voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(*layer_msg, tsdf_map_ptr_->getTsdfLayerPtr());
    if(!success) {
        std::cout << "failed to deserialize layer" << std::endl;
        new_layer_ = false;
        return;   
    }
    new_layer_ = true;
    std::cout << "layer callback end" << std::endl;
    return;
}


//from controller
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


//adapted from cblox
//voxel_size is a bit downsampling, maybe just change tsdf voxel size?
//TODO
std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> RoomDetector::getTsdfSlice(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_ptr, float slice_height) {
    FloatingPoint voxel_size = 0.1;
    size_t voxels_per_side = 16;
    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> slice_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_size, voxels_per_side);

//from cblox
    voxblox::BlockIndexList block_list;
  tsdf_layer_ptr->getAllAllocatedBlocks(&block_list);

  int block_num = 0;

  for (const voxblox::BlockIndex& block_id : block_list) {

    if (!tsdf_layer_ptr->hasBlock(block_id)) continue;

    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        tsdf_layer_ptr->getBlockPtrByIndex(block_id);

    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {

      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);

      voxblox::Point position =
          block->computeCoordinatesFromLinearIndex(voxel_id);


      if (voxel.weight < 1e-6) {
        continue;
      }

      if(std::abs(position.z() - slice_height) > voxel_size/2.0) {
          continue; //skip if not on slice place
      }


      auto voxel_ptr = slice_layer->getVoxelPtrByCoordinates(position);
      if(!voxel_ptr) {

          slice_layer->allocateNewBlockByCoordinates(position);
          voxel_ptr = slice_layer->getVoxelPtrByCoordinates(position);
          if(!voxel_ptr) {
              std::cout << "error, couldn't allocate voxel in layer" << std::endl;
              continue;
          }
      }
      voxel_ptr->distance = (voxel_ptr->weight * voxel_ptr->distance  + voxel.distance) / (voxel_ptr->weight + 1);
      voxel_ptr->weight += 1;
    }
  }
  return slice_layer;

}

//from cblox adapted again
void RoomDetector::visualizeSlice(std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> slice_layer_ptr) {
   visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = "world";
  vertex_marker.ns = "slice";
  vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  vertex_marker.pose.orientation.w = 1.0;
  vertex_marker.scale.x =
      0.05;
  vertex_marker.scale.y = vertex_marker.scale.x;
  vertex_marker.scale.z = vertex_marker.scale.x;
  geometry_msgs::Point point_msg;
  std_msgs::ColorRGBA color_msg;
  color_msg.r = 0.0;
  color_msg.g = 0.0;
  color_msg.b = 0.0;
  color_msg.a = 1.0;
   
   
   
    voxblox::BlockIndexList block_list;
  slice_layer_ptr->getAllAllocatedBlocks(&block_list);
  int block_num = 0;
  for (const voxblox::BlockIndex& block_id : block_list) {
    if (!slice_layer_ptr->hasBlock(block_id)) continue;
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        slice_layer_ptr->getBlockPtrByIndex(block_id);
    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {
      const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);
      voxblox::Point position =
          block->computeCoordinatesFromLinearIndex(voxel_id);

      if (voxel.weight < 1e-6) {
        continue;
      }

      color_msg.r = 0.0;
      color_msg.g = 0.0;
      if (voxel.weight >= 1e-6) {
        color_msg.r = std::max(
            std::min((3 - voxel.distance) / 2.0 / 3, 1.0), 0.0);
        color_msg.g = std::max(
            std::min((3 + voxel.distance) / 2.0 / 3, 1.0), 0.0);
      }

        vertex_marker.id =
            block_num +
            voxel_id * std::pow(10, std::round(std::log10(block_list.size())));
        tf::pointEigenToMsg(position.cast<double>(), point_msg);

        vertex_marker.points.push_back(point_msg);
        vertex_marker.colors.push_back(color_msg);
    }
    block_num++;
  }

  marker_array.markers.push_back(vertex_marker);
  slice_pub_.publish(marker_array);
}


//NOT THREAD SAFE
void RoomDetector::visualizeFloor(Floor::Ptr floor) {
    auto layer = floor->getLayer();

//again similar to cblox
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = "world";
  vertex_marker.ns = "slice";
  vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  vertex_marker.pose.orientation.w = 1.0;
  vertex_marker.scale.x =
      0.05;
  vertex_marker.scale.y = vertex_marker.scale.x;
  vertex_marker.scale.z = vertex_marker.scale.x;
  geometry_msgs::Point point_msg;
  std_msgs::ColorRGBA color_msg;
  color_msg.r = 0.0;
  color_msg.g = 0.0;
  color_msg.b = 0.0;
  color_msg.a = 1.0;

  voxblox::BlockIndexList block_list;
  layer->getAllAllocatedBlocks(&block_list);
  int block_num = 0;
  for (const voxblox::BlockIndex& block_id : block_list) {
    if (!layer->hasBlock(block_id)) continue;
    voxblox::Block<RoomVoxel>::Ptr block =
        layer->getBlockPtrByIndex(block_id);
    for (size_t voxel_id = 0; voxel_id < block->num_voxels(); voxel_id++) {
      const RoomVoxel& voxel = block->getVoxelByLinearIndex(voxel_id);
      voxblox::Point position =
          block->computeCoordinatesFromLinearIndex(voxel_id);

      if (voxel.room_id == 0) {
        continue;
      }
      if(voxel.wall) {
          color_msg.r = 1.0;
          color_msg.g = 1.0;
          color_msg.b = 1.0;
      }else {
        voxblox::Color c = voxblox::rainbowColorMap(double(voxel.room_id) / 10.0);
        color_msg.r = c.r / 255.0;
        color_msg.g = c.g / 255.0;
        color_msg.b = c.b / 255.0;
      }

        vertex_marker.id =
            block_num +
            voxel_id * std::pow(10, std::round(std::log10(block_list.size())));
        tf::pointEigenToMsg(position.cast<double>(), point_msg);

        vertex_marker.points.push_back(point_msg);
        vertex_marker.colors.push_back(color_msg);
    }
    block_num++;
  }

  marker_array.markers.push_back(vertex_marker);
  floor_pub_.publish(marker_array);

}
