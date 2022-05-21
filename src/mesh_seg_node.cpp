#include <ros/ros.h>
#include "mesh_segmentation_ros/MeshSeg.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_segmentation");
  ros::NodeHandle nh;
  MeshSeg ms(nh);

  ros::spin();

  return 0;
}
