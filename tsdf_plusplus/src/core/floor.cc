#include "tsdf_plusplus/core/floor.h"

Floor::Floor(double floor_height, double ceiling_height, std::shared_ptr<RoomsAndFloors> rooms_and_floors) 
  : floor_height_(floor_height), ceiling_height_(ceiling_height), floor_weight_(1.0), ceiling_weight_(1.0), rooms_and_floors_(rooms_and_floors)
  {
      //TODO adapt parameters for layer
      room_layer_ = std::make_shared<voxblox::Layer<RoomVoxel>>(0.1, 16u);


      //loading debug room layout, must be removed later
      std::cout << "creating debug floor" << std::endl;
      std::vector<float> s_x = {00.00, 03.66, 07.32, 09.15, 00.00, 00.00, 03.66, 07.32, 00.00, 00.00, 03.66, 07.32, 00.00, 00.00, 03.66, 07.32};
      std::vector<float> e_x = {03.66, 07.32, 09.15, 14.64, 09.15, 03.66, 07.32, 09.15, 09.15, 03.66, 07.32, 09.15, 09.15, 03.66, 07.32, 09.15};
      std::vector<float> s_y = {00.00, 00.00, 00.00, 00.00, 03.66, 05.49, 05.49, 05.49, 09.15, 10.98, 10.98, 10.98, 14.64, 16.47, 16.47, 16.47};
      std::vector<float> e_y = {03.66, 03.66, 03.66, 20.13, 05.49, 09.15, 09.15, 09.15, 10.98, 14.64, 14.64, 14.64, 16.47, 20.13, 20.13, 20.13};
      auto z = mapping_height_;
      float x_offset = 1.8;
      float y_offset = -14.4;
      for (int s = 0; s < s_x.size(); s++) {
          //get free id and allocate it
          uint id = 1;

          //not completely thread safe, as room ids are accessed, but room detector currently logs it
          std::cout << "searching free id" << std::endl;
          while(rooms_and_floors_->room_ids->find(id) != rooms_and_floors_->room_ids->end()) {
              id++;
          }
          std::cout << "found id" << std::endl;
          (*(rooms_and_floors_->room_ids))[id] = true;

          //map rooms in layer
          for(float x = s_x[s]; x <= e_x[s]; x+=0.1) {
              for(float y = s_y[s]; y <= e_y[s]; y+=0.1) {
                  voxblox::Point position(x_offset + x, y_offset + y, z);
                  auto voxel = room_layer_->getVoxelPtrByCoordinates(position);
                  if(!voxel) {
                      room_layer_->allocateNewBlockByCoordinates(position);
                      voxel = room_layer_->getVoxelPtrByCoordinates(position);
                  }

                  voxel->wall = (x == s_x[s] || x == e_x[s] || y == s_y[s] || y == e_y[s]);
                  voxel->room_id = id;
              }
          }
          std::cout << "finished room" << std::endl;
      }
}

voxblox::Layer<RoomVoxel>::Ptr Floor::getLayer() {
    return room_layer_;
}

void Floor::updateWithSlice(voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer) {
    floor_mutex_.lock();
    floor_mutex_.unlock();
    //TODO

    //TODO, get Rooms as own class/struct instead of bool, add pose by calculating center of room by all voxels.
}

void Floor::updateHeights(double floor_height, double ceiling_height) {
    floor_mutex_.lock();
    floor_height_ = (floor_height_ * floor_weight_ + floor_height) / (floor_weight_ + 1.0);
    floor_weight_++;

    ceiling_height_ = (ceiling_height_ * ceiling_weight_ + ceiling_height) / (ceiling_weight_ + 1.0);
    ceiling_weight_++;
    std::cout << "Floor: " << floor_height_ << " Ceiling: " << ceiling_height_ << std::endl;
    floor_mutex_.unlock();
}


void Floor::setLayer(voxblox::Layer<RoomVoxel>::Ptr override_layer) {
    floor_mutex_.lock();
    room_layer_ = override_layer;
    floor_mutex_.unlock();
}

bool Floor::inFloor(double height) {
    floor_mutex_.lock();
    auto res = (height > floor_height_ && height < ceiling_height_);
    floor_mutex_.unlock();
    return res;
}

uint Floor::getRoomId(voxblox::Point point) {
    floor_mutex_.lock();
    uint room_id = 0;
    point[2] = mapping_height_; //working only on this z height
    auto voxel = room_layer_->getVoxelPtrByCoordinates(point);
    bool check_neighbours = false;
    //voxel not existing, check neighborhood
    if(!voxel) {
        check_neighbours = true;
    } else {
        room_id = voxel->room_id;
        if(room_id == 0) {
            check_neighbours = true;
        }
    }

    if(check_neighbours) {
        auto res = checkNeighbours(point, 1);
        floor_mutex_.unlock();
        return res;
    }
    floor_mutex_.unlock();
    return room_id;
}

uint Floor::checkNeighbours(voxblox::Point point, int neighbourhood_size) {
    
    std::map<uint, int> room_ids;
    point[2] = 0;
    for (int x = -neighbourhood_size; x <= neighbourhood_size; x++) {
        for(int y = -neighbourhood_size; y <= neighbourhood_size; y++) {
            voxblox::Point p(point[0] + x, point[1] + y, 0.0);
            auto voxel = room_layer_->getVoxelPtrByCoordinates(point);
            if(!voxel)
                continue;
            if(room_ids.find(voxel->room_id) == room_ids.end()) {
                room_ids[voxel->room_id] = 1;
            } else {
                room_ids[voxel->room_id] += 1;
            }
        }
    }

    if(room_ids.size() == 0) {
        return 0;
    }

    uint room_id = 0;
    int weight = 0;
    for(auto entry : room_ids) {
        if(entry.first == 0) {
            continue; //ignoring 0 label
        }
        if(entry.second > weight) {
            weight = entry.second;
            room_id = entry.first;
        }

    }
    return room_id;
}