#include "MeshSeg.h"

MeshSeg::MeshSeg(ros::NodeHandle& nodehandle)
: nh(nodehandle)
{
  // get params from launch file
  nh.param("input_mesh_topic", meshTopicIn, meshTopicIn);
  nh.param("odom_topic", odomTopicIn, odomTopicIn);
  nh.param("octree_resolution", m_octreeRes, m_octreeRes);
  nh.param("trans_norm_thresh", m_normThresh, m_normThresh);
  nh.param("num_points_changed_thresh", m_numPtsChanged, m_numPtsChanged);

  // get params from region growing param config file
  nh.getParam("numKNeighbors", ne_numKNeighbors);
  nh.getParam("minClusterSize", rg_minClusterSize);
  nh.getParam("maxClusterSize", rg_maxClusterSize);
  nh.getParam("numNeighbors", rg_numNeighbors);
  nh.getParam("smoothnessThresholdFactor", rg_smoothThreshFactor);
  nh.getParam("curvatureThresholdFactor", rg_curvThreshFactor);

  // set up subscribers
  m_meshInSub = nh.subscribe(meshTopicIn, 100, &MeshSeg::MeshCallback, this);
  m_odomSub = nh.subscribe(odomTopicIn, 100, &MeshSeg::OdomCallback, this);

  // set up publishers
  m_kfPosePub = nh.advertise<geometry_msgs::PointStamped>(m_nodeName + "/kfPose", 5);
  m_groundMeshPub = nh.advertise<mesh_msgs::TriangleMeshStamped>(m_nodeName + "/ground_mesh", 5);
  m_obstacleMeshPub = nh.advertise<mesh_msgs::TriangleMeshStamped>(m_nodeName + "/obstacle_mesh", 5);
  m_meshAPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/meshA_points", 5);
  m_meshBPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/meshB_points", 5);

  // initialize point clouds
  cloudA = CloudXYZL::Ptr (new CloudXYZL);
  cloudB = CloudXYZL::Ptr (new CloudXYZL);
  cloudChangedPts = CloudXYZL::Ptr (new CloudXYZL);

  // initialize Octree spatial change detector
  octree = OctreeChangeDetector::Ptr (new OctreeChangeDetector (m_octreeRes));
  m_switchBuffers = false;
  m_cloudAInit = false;
  m_cloudBInit = false;

  // initialize odom point tracker
  m_norm = 0;
  m_lastPose.x = 0.; m_lastPose.y = 0.; m_lastPose.z = 0.;

  // initialize region growing
  rg_indices = pcl::IndicesPtr (new std::vector<int>);
  rg_searchTree = PCLSearchTree::Ptr (new pcl::search::KdTree<PointXYZL>);
  rg_normEst.setSearchMethod(rg_searchTree);
  rg_normEst.setKSearch(ne_numKNeighbors);
  rg_regionGrowing.setMinClusterSize(rg_minClusterSize);
  rg_regionGrowing.setMaxClusterSize(rg_maxClusterSize);
  rg_regionGrowing.setSearchMethod(rg_searchTree);
  rg_regionGrowing.setSmoothnessThreshold(rg_smoothThreshFactor / 180.0*M_PI);
  rg_regionGrowing.setCurvatureThreshold(rg_curvThreshFactor);
}

MeshSeg::~MeshSeg()
{
}

void MeshSeg::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // get current pose as point
  geometry_msgs::Point currPose = msg->pose.pose.position;
  // vector to hold translation differences
  Eigen::Vector3d diff(currPose.x-m_lastPose.x, currPose.y-m_lastPose.y, currPose.z-m_lastPose.z);
  // calculate translation
  m_norm += diff.norm();
  // check difference 
  if (m_norm > m_normThresh)
  {
    // run region growing on new map section
    std::cout << "New keyframe" << std::endl;
    // update ground and obstacle meshes

    // publish keyframe pose
    geometry_msgs::PointStamped pose_msg;
    pose_msg.header.frame_id = "X1/map";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.point.x = currPose.x;
    pose_msg.point.y = currPose.y;
    pose_msg.point.z = currPose.z;
    m_kfPosePub.publish(pose_msg);

    // update last position
    m_lastPose = currPose;
    m_norm = 0;
  }
}

void MeshSeg::MeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& msg)
{
  // cloud to store mesh centroids
  CloudXYZL cloud;
  uint numMeshPts = msg->mesh.triangles.size();
  cloud.reserve(numMeshPts);

  // get all mesh face centroids
  for (uint i=0; i < numMeshPts; i++)
  {
    // get vertices for all three points
    geometry_msgs::Point vertex0 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[0]];
    geometry_msgs::Point vertex1 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[1]];
    geometry_msgs::Point vertex2 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[2]];
    
    // centroid point with x, y, z coords and label
    pcl::PointXYZL pt;
    pt.x = (vertex0.x + vertex1.x + vertex2.x) / 3;
    pt.y = (vertex0.y + vertex1.y + vertex2.y) / 3;
    pt.z = (vertex0.z + vertex1.z + vertex2.z) / 3;
    pt.label = i;
    cloud.push_back(pt);
  }
  
  // cloudA should be updated
  if (!m_switchBuffers)
  {
    *cloudA = cloud;
    octree->setInputCloud(cloudA);
    octree->addPointsFromInputCloud();
    octree->switchBuffers();
    m_switchBuffers = true;
    m_cloudAInit = true;
  }
  // cloubB should be updated
  else
  {
    *cloudB = cloud;
    octree->setInputCloud(cloudB);
    octree->addPointsFromInputCloud();
    m_switchBuffers = false;
    m_cloudBInit = true;
  }

  // if both cloudA and cloudB have been updated
  if (!m_switchBuffers && m_cloudAInit && m_cloudBInit)
  {
    // store indices of points that changed
    std::vector<int> newPointIdxVector;
    // get changed point indicies
    octree->getPointIndicesFromNewVoxels(newPointIdxVector);
    cloudChangedPts->resize(cloudChangedPts->size() + newPointIdxVector.size());
    if (!cloudChangedPts->empty())
      cloudChangedPts->clear();

    // add points that changed to pts_changed cloud
    for (std::size_t i=0; i < newPointIdxVector.size(); i++)
    {
      PointXYZL pt;
      pt.x = cloudB->points[newPointIdxVector[i]].x; 
      pt.y = cloudB->points[newPointIdxVector[i]].y;
      pt.z = cloudB->points[newPointIdxVector[i]].z;
      cloudChangedPts->push_back(pt);
    }

    // cloud to publish points that changed
    CloudXYZL cloud_changed_pc;
    cloud_changed_pc.reserve(cloudChangedPts->size());
    cloud_changed_pc = *cloudChangedPts;
    
    // publish new points
    sensor_msgs::PointCloud2 cloud_changed;
    pcl::toROSMsg(cloud_changed_pc, cloud_changed);
    cloud_changed.header.stamp = ros::Time::now();
    cloud_changed.header.frame_id = msg->header.frame_id;
    m_meshBPCPub.publish(cloud_changed);
  }
}

void MeshSeg::pubChangedPtsCloud()
{
  // publish cloud
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(*cloudChangedPts, cloud_out);
  cloud_out.header.stamp = ros::Time::now();
  cloud_out.header.frame_id = "inf_map";
  m_meshAPCPub.publish(cloud_out);
}
