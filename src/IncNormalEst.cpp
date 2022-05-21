#include "mesh_segmentation_ros/IncNormalEst.h"

IncNormalEst::IncNormalEst(int searchNumNeighbors)
: m_searchNumNeighbors(searchNumNeighbors)
{
  m_kdTree = pcl::search::KdTree<PointXYZL>::Ptr (new pcl::search::KdTree<PointXYZL>);
}

void IncNormalEst::notifyPointsTransformed()
{
 
}

void IncNormalEst::clear()
{
  sum_X_Xt.clear();
  sum_X.clear();
  m_normals.clear();
  m_numPoints.clear();
}

// Auxiliary function for rearranging elements in a vector according to a mapping.
template <typename Container, typename Element>
void rearrangeElementsWithMapping(const std::vector<int>& mapping, const size_t new_size,
                                  const Element& default_value, Container& container) {
  Container new_container(new_size, default_value);
  for (size_t i = 0u; i < mapping.size(); ++i) {
    if (mapping[i] >= 0) new_container[mapping[i]] = std::move(container[i]);
  }
  container = std::move(new_container);
}

std::vector<bool> IncNormalEst::updateNormals(const CloudXYZL::Ptr points, const std::vector<int>& pointsMapping,
                                              const std::vector<int>& newPointIndices, 
                                              PCLKdTree& searchTree)
{
  // rearrange the cached information according to the mapping
  rearrangeElementsWithMapping(pointsMapping, points->size(), Eigen::Matrix3f::Zero(), sum_X_Xt);
  rearrangeElementsWithMapping(pointsMapping, points->size(), Eigen::Vector3f::Zero(), sum_X);
  rearrangeElementsWithMapping(pointsMapping, points->size(), 0u, m_numPoints);
  rearrangeElementsWithMapping(pointsMapping, points->size(), PointN(), m_normals.points);

  // scatter the contributions of the new points to the covariance matrices of each point's
  // neighborhood
  const std::vector<bool> isNormalAffected = scatterNormalContributions(points, newPointIndices, searchTree);

  // perform eigenvalue analysis on the affected point's covariances to determine their new normals
  recomputeNormals(points, isNormalAffected);

  return isNormalAffected;
}

std::vector<bool> IncNormalEst::scatterNormalContributions(const CloudXYZL::Ptr points, 
                                                           const std::vector<int>& newPointIndices,
                                                           PCLKdTree& searchTree)
{
  std::vector<bool> isNewPoint(points->size(), false);
  // indicate which points are new in indices vector
  for (auto point_index : newPointIndices)
    isNewPoint[point_index] = true;

  // scatter information to all points that are affected by new points
  std::vector<bool> isNormalAffected(points->size(), false);
  searchTree.setInputCloud(points);
  // for all new indices
  for (auto point_index : newPointIndices)
  {
    // find neighbors for point at point_index
    std::vector<int> ptIdxNNSearch(m_searchNumNeighbors);
    std::vector<float> ptKNNSearchDist(m_searchNumNeighbors);
    searchTree.nearestKSearch(point_index, m_searchNumNeighbors, ptIdxNNSearch, ptKNNSearchDist);
    
    isNormalAffected[point_index] = true;
    const Eigen::Vector3f& sourcePt = points->points[point_index].getVector3fMap();
    
    // for all neighbors found
    for (const auto neighbor_index : ptIdxNNSearch)
    {
      // add contribution to neighbor point
      isNormalAffected[neighbor_index] = true;
      sum_X_Xt[neighbor_index] += sourcePt * sourcePt.transpose();
      sum_X[neighbor_index] += sourcePt;
      ++m_numPoints[neighbor_index];

      // if the neighbor is an old point, then it also contributes to the normal of the new point
      if (!isNewPoint[neighbor_index])
      {
        const Eigen::Vector3f& neighbor_point = points->points[neighbor_index].getVector3fMap();
        sum_X_Xt[neighbor_index] += neighbor_point * neighbor_point.transpose();
        sum_X[neighbor_index] += neighbor_point;
        ++m_numPoints[point_index];
      }
    }
  }
  return isNormalAffected;
}

void IncNormalEst::recomputeNormals(const CloudXYZL::Ptr points, const std::vector<bool>& needsRecompute)
{
  // only recompute normals for affected points
  size_t numAffectedNormals = 0u;
  for (size_t i=0; i < m_normals.size(); i++)
  {
    if (needsRecompute[i])
    {
      ++numAffectedNormals;
      if (m_numPoints[i] >= 3u)
      {
        // if there are at least three points in the neighborhood, the normal vector is equal to
        // the eigenvector of the smallest eigenvalue
        const float norm_factor = 1.0f / static_cast<float>(m_numPoints[i]);
        const EIGEN_ALIGN16 Eigen::Matrix3f covariance = 
          (sum_X_Xt[i] * norm_factor - sum_X[i] * norm_factor * sum_X[i].transpose() * norm_factor);
        pcl::solvePlaneParameters(covariance, m_normals[i].normal_x, m_normals[i].normal_y,
                                  m_normals[i].normal_z, m_normals[i].curvature);
        constexpr float view_point_component = std::numeric_limits<float>::max();
        pcl::flipNormalTowardsViewpoint(points->points[i], view_point_component, view_point_component,
                                        view_point_component, m_normals[i].normal_x,
                                        m_normals[i].normal_y, m_normals[i].normal_z);
      }
      else 
      { 
        // otherwise don't have enough data to estimate normal. Set it to NaN
        m_normals[i].normal_x = m_normals[i].normal_y = m_normals[i].normal_z = m_normals[i].curvature
          = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
}
