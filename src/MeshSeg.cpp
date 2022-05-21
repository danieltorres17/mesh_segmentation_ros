#include "mesh_segmentation_ros/MeshSeg.h"

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
  m_changedTrianglesPub = nh.advertise<mesh_msgs::TriangleMeshStamped>(m_nodeName + "/changed_mesh", 5);
  m_cloudAPub = nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/meshA_points", 5);
  m_cloudBPub = nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/meshB_points", 5);

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
  // calculate translation norm
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
    m_lastPose.x = currPose.x;
    m_lastPose.y = currPose.y;
    m_lastPose.z = currPose.z;
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
    m_cloudBPub.publish(cloud_changed);
  }
  // // new cloud to store mesh triangle centroids in
  // pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
  // uint numMeshPts = msg->mesh.triangles.size();
  // cloud->points.reserve(numMeshPts);

  // std::cout << "Mesh frame: " << msg->header.frame_id << std::endl;
  // std::cout << msg->mesh.triangles.size() << std::endl;

  // // push centroid of each triangle in mesh to cloud with index i as label
  // for (uint i=0; i < numMeshPts; i++)
  // {
  //   // get vertices for all three points
  //   geometry_msgs::Point vertex0 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[0]];
  //   geometry_msgs::Point vertex1 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[1]];
  //   geometry_msgs::Point vertex2 = msg->mesh.vertices[msg->mesh.triangles[i].vertex_indices[2]];
    
  //   // centroid point with x, y, z coords and label
  //   pcl::PointXYZL pt;
  //   pt.x = (vertex0.x + vertex1.x + vertex2.x) / 3;
  //   pt.y = (vertex0.y + vertex1.y + vertex2.y) / 3;
  //   pt.z = (vertex0.z + vertex1.z + vertex2.z) / 3;
  //   pt.label = i;
  //   cloud->push_back(pt);
  // }

  // // run region growing
  // pcl::search::Search<pcl::PointXYZL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZL>);
  // pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
  // normals->points.reserve(numMeshPts);
  // pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> normal_estimator;
  // normal_estimator.setSearchMethod(tree);
  // normal_estimator.setInputCloud(cloud);
  // normal_estimator.setKSearch(50);
  // normal_estimator.compute(*normals);

  // pcl::IndicesPtr indices(new std::vector <int>);
  // // pcl::PassThrough<pcl::PointXYZL> pass;
  // // pass.setInputCloud(cloud);
  // // pass.setFilterFieldName("z");
  // // pass.setFilterLimits(0.0, 12.0);
  // // pass.filter(*indices);

  // pcl::RegionGrowing<pcl::PointXYZL, pcl::Normal> reg;
  // reg.setMinClusterSize(100);
  // reg.setMaxClusterSize(1000000);
  // reg.setSearchMethod(tree);
  // reg.setNumberOfNeighbours(50);
  // reg.setInputCloud(cloud);
  // reg.setIndices(indices);
  // reg.setInputNormals(normals);
  // reg.setSmoothnessThreshold(2.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold(1.0);

  // // extract clusters from cloud
  // std::vector<pcl::PointIndices> clusters;
  // reg.extract(clusters);

  // // // make cloud from indices
  // // CloudT::Ptr roi(new CloudT);
  // // roi->reserve(clusters[0].indices.size());
  // // for (auto idx : clusters[0].indices)
  // // {
  // //   pcl::PointXYZ pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
  // //   roi->push_back(pt);
  // // }
  // // roi->is_dense = true;

  // // make new mesh from extracted points
  // mesh_msgs::TriangleMeshStamped tmp_mesh_msg;
  // tmp_mesh_msg.header.stamp = ros::Time::now();
  // tmp_mesh_msg.header.frame_id = msg->header.frame_id;

  // // for all cloud PC indices in cluster[0], extract face info and push into ground mesh message
  // for (auto idx : clusters[0].indices)
  // {
  //   // get point label
  //   uint midx = cloud->points[idx].label;
  //   // use label to access corresponding face
  //   geometry_msgs::Point vertex0 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[0]];
  //   geometry_msgs::Point vertex1 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[1]];
  //   geometry_msgs::Point vertex2 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[2]];
    
  //   mesh_msgs::TriangleIndices tidxs;
  //   // point 0 
  //   tidxs.vertex_indices[0] = tmp_mesh_msg.mesh.vertices.size();
  //   tmp_mesh_msg.mesh.vertices.push_back(vertex0);
  //   // point 1
  //   tidxs.vertex_indices[1] = tmp_mesh_msg.mesh.vertices.size();
  //   tmp_mesh_msg.mesh.vertices.push_back(vertex1);
  //   // point 2
  //   tidxs.vertex_indices[2] = tmp_mesh_msg.mesh.vertices.size();
  //   tmp_mesh_msg.mesh.vertices.push_back(vertex2);
  //   // push triangle index into message
  //   tmp_mesh_msg.mesh.triangles.push_back(tidxs);
  // }
  // m_groundMeshPub.publish(tmp_mesh_msg);

  // // publish rest of clusters as obstacle mesh
  // mesh_msgs::TriangleMeshStamped obs_mesh_msg;
  // obs_mesh_msg.header.stamp = ros::Time::now();
  // obs_mesh_msg.header.frame_id = msg->header.frame_id;
  // for (uint i=1; i < clusters.size(); i++)
  // {
  //   // for all cloud PC indices in cluster[i], extract face info and push into obstacle mesh message
  //   for (auto idx : clusters[i].indices)
  //   {
  //     // get point label
  //     uint midx = cloud->points[idx].label;
  //     // use label to access corresponding face
  //     geometry_msgs::Point vertex0 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[0]];
  //     geometry_msgs::Point vertex1 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[1]];
  //     geometry_msgs::Point vertex2 = msg->mesh.vertices[msg->mesh.triangles[midx].vertex_indices[2]];
      
  //     mesh_msgs::TriangleIndices tidxs;
  //     // point 0 
  //     tidxs.vertex_indices[0] = obs_mesh_msg.mesh.vertices.size();
  //     obs_mesh_msg.mesh.vertices.push_back(vertex0);
  //     // point 1
  //     tidxs.vertex_indices[1] = obs_mesh_msg.mesh.vertices.size();
  //     obs_mesh_msg.mesh.vertices.push_back(vertex1);
  //     // point 2
  //     tidxs.vertex_indices[2] = obs_mesh_msg.mesh.vertices.size();
  //     obs_mesh_msg.mesh.vertices.push_back(vertex2);
  //     // push triangle index into message
  //     obs_mesh_msg.mesh.triangles.push_back(tidxs);
  //   }
  // }
  // m_obstacleMeshPub.publish(obs_mesh_msg);
}

void MeshSeg::pubChangedPtsCloud()
{
  // publish cloud
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(*cloudChangedPts, cloud_out);
  cloud_out.header.stamp = ros::Time::now();
  cloud_out.header.frame_id = "inf_map";
  m_cloudAPub.publish(cloud_out);
}
