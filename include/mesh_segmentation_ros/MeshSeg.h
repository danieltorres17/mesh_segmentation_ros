#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "mesh_segmentation_ros/MeshSegCommon.h"

class MeshSeg
{
public:
  MeshSeg(ros::NodeHandle& nodehandle);
  ~MeshSeg();
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void MeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& msg);

private:
  void pubChangedPtsCloud();
  void runRegionGrowing();

  ros::NodeHandle nh;
  ros::Subscriber m_meshInSub;
  ros::Subscriber m_odomSub;
  ros::Publisher m_kfPosePub;
  ros::Publisher m_groundMeshPub;
  ros::Publisher m_obstacleMeshPub;
  ros::Publisher m_changedTrianglesPub;
  ros::Publisher m_cloudAPub;
  ros::Publisher m_cloudBPub;
  tf::TransformListener m_tfListener; 

  CloudXYZL::Ptr cloudA;
  CloudXYZL::Ptr cloudB;
  CloudXYZL::Ptr cloudChangedPts;
  std::vector<pcl::PointIndices> clusters;
  bool m_switchBuffers;
  bool m_cloudAInit;
  bool m_cloudBInit;

  double m_octreeRes;
  OctreeChangeDetector::Ptr octree;

  geometry_msgs::Point m_lastPose;
  double m_normThresh;
  double m_norm;
  int m_numPtsChanged;

  const std::string m_nodeName = "MeshSeg";
  std::string meshTopicIn;
  std::string odomTopicIn;

  // region growing parameters
  int ne_numKNeighbors;
  int rg_minClusterSize;
  int rg_maxClusterSize;
  int rg_numNeighbors;
  double rg_smoothThreshFactor;
  double rg_curvThreshFactor;

  // region growing
  pcl::IndicesPtr rg_indices = nullptr;
  PCLSearchTree::Ptr rg_searchTree = nullptr;
  CloudN::Ptr rg_normals = nullptr;
  PCLNormalEst rg_normEst;
  pcl::RegionGrowing<PointXYZL, PointN> rg_regionGrowing;
};
