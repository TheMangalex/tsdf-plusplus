#include <gflags/gflags.h>
#include <glog/logging.h>

#include "tsdf_plusplus_ros/controller.h"
#include "tsdf_plusplus_ros/room_detector.h"

int main(int argc, char** argv) {
  std::cout << std::endl
            << "TSDF++ Copyright (c) 2020- Margarita Grinvald, Autonomous "
               "Systems Lab, ETH Zurich."
            << std::endl
            << std::endl;

  ros::init(argc, argv, "tsdf_plusplus_node");
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  std::shared_ptr<Controller> controller_ptr = std::make_shared<Controller>(node_handle, node_handle_private);

  std::shared_ptr<RoomDetector> detector = std::make_shared<RoomDetector>(node_handle, node_handle_private, controller_ptr);

  // Spinner with a number of threads equal to the number of cores.
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
