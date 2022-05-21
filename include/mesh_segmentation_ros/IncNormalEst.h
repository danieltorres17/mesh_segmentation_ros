  /*
Modified from: 
https://github.com/ethz-asl/segmap/blob/master/segmatch/include/segmatch/normal_estimators/incremental_normal_estimator.hpp
*/

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "mesh_segmentation_ros/IncNormalEst.h"

class IncNormalEst
{
public:
  IncNormalEst(int searchNumNeighbors);
  ~IncNormalEst();
  
  // Notifies the estimator that points have been transformed
  // Transformation linear 
  void notifyPointsTransformed();
  // clear all the normals and the associated information.
  // Equivalent to removing all the points from the cloud
  void clear();
  // Update the normals of a cloud
  std::vector<bool> updateNormals(const CloudXYZL::Ptr points, const std::vector<int>& pointsMapping,
                                  const std::vector<int>& newPointIndices, 
                                  PCLKdTree& searchTree);
  // return the cloud containing the normal vectors
  const CloudN& getNormals() const { return m_normals; }

private:
  // add contributions of the new points to the normals
  std::vector<bool> scatterNormalContributions(const CloudXYZL::Ptr points, 
                                               const std::vector<int>& newPointIndices,
                                               PCLKdTree& searchTree);
  void recomputeNormals(const CloudXYZL::Ptr points, const std::vector<bool>& needsRecompute);

  CloudN m_normals;
  int m_searchNumNeighbors;
  pcl::search::KdTree<PointXYZL>::Ptr m_kdTree;

  // partial covariance matrix information for incremental estimation
  // the covariance matrix is computed as
  // C = E[X*X^T] - mu*mu^t = num_points * sum_X_Xt + num_points^2 * sum_X * sum_Xt
  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> sum_X_Xt;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> sum_X;
  std::vector<size_t> m_numPoints;
};
