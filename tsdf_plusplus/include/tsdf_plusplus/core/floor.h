#ifndef TSDF_PLUSPLUS_FLOOR_H_
#define TSDF_PLUSPLUS_FLOOR_H_

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <tsdf_plusplus/core/voxel.h>

struct RoomsAndFloors;

class Floor {
  public:
    typedef std::shared_ptr<Floor> Ptr;

    Floor(double floor_height, double ceiling_height, std::shared_ptr<RoomsAndFloors> rooms_and_floors);

    void updateWithSlice(voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer); //TODO

    void updateHeights(double floor_height, double ceiling_height);

    void setLayer(voxblox::Layer<RoomVoxel>::Ptr override_layer);

    bool inFloor(double height);

    uint getRoomId(voxblox::Point point);

    uint checkNeighbours(voxblox::Point point, int neighbourhood_size);

  protected:
    double floor_height_;
    double floor_weight_;
    double ceiling_height_;
    double ceiling_weight_;
    double mapping_height_ = 0.0;


    voxblox::Layer<RoomVoxel>::Ptr room_layer_;

    std::shared_ptr<RoomsAndFloors> rooms_and_floors_;

    std::mutex floor_mutex_;

};

struct RoomsAndFloors {
  std::shared_ptr<std::map<uint, bool>> room_ids;
  std::shared_ptr<std::vector<Floor::Ptr>> floors;
  std::mutex mut;

};

#endif // TSDF_PLUSPLUS_FLOOR_H_