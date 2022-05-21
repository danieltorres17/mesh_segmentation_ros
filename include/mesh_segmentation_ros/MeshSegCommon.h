#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZL PointXYZL;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointXYZ> CloudXYZ;
typedef CloudXYZ::Ptr CloudXYZPtr;
typedef pcl::PointCloud<PointXYZL> CloudXYZL;
typedef CloudXYZL::Ptr CloudXYZLPtr;
typedef pcl::PointCloud<PointN> CloudN;
typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;
typedef pcl::octree::OctreePointCloudChangeDetector<PointXYZL> OctreeChangeDetector;
typedef pcl::search::Search<PointXYZL> PCLSearchTree;
typedef pcl::search::KdTree<PointXYZL> PCLKdTree;
typedef pcl::NormalEstimation<PointXYZL, PointN> PCLNormalEst;

typedef std::pair<PointXYZ, PointXYZ> PointPair;
typedef std::vector<PointPair> PointPairs;

typedef std::vector<Eigen::Matrix4f,
    Eigen::aligned_allocator<Eigen::Matrix4f>> RotationsTranslations;

typedef std::pair<std::string, std::string> FilenamePair;

typedef std::pair<CloudXYZLPtr, CloudXYZLPtr> CloudPair;

/*
 * \brief Type representing IDs, for example for segments or clouds
 * Warning: the current implementation sometimes uses IDs as array indices.
 */
typedef int64_t Id;
const Id kNoId = -1;
const Id kInvId = -2;
const Id kUnassignedId = -3; // Used internally by the incremental segmenter.

// TODO(Renaud @ Daniel) this is probably not the best name but we need to have this format
// Somewhere as the classifier output matches between two samples and the geometric also
// takes pairs. It collides a bit with IdMatches. We can discuss that. I also added for
// convenience the centroids in there as they are easy to get when grabbing the Ids.
// Let's see how that evolves.
// 
// TODO: switch to std::array of size 2? so that the notation is the same .at(0) instead of first.
typedef std::pair<Id, Id> IdPair;

class PairwiseMatch 
{
public:
  PairwiseMatch(Id id1, Id id2, const PointXYZ& centroid1, const PointXYZ& centroid2,
                float confidence_in) 
                : ids_(id1, id2),
                  confidence_(confidence_in),
                  centroids_(PointPair(centroid1, centroid2)) {}

  PointPair getCentroids() const { return centroids_; }
  IdPair ids_;
  float confidence_;
  Eigen::MatrixXd features1_;
  Eigen::MatrixXd features2_;
  PointPair centroids_;
};

typedef std::vector<PairwiseMatch,
    Eigen::aligned_allocator<PairwiseMatch> > PairwiseMatches;
    
struct Translation {
  Translation(double x_in, double y_in, double z_in) :
    x(x_in), y(y_in), z(z_in) {}
  double x;
  double y;
  double z;
};

struct SegmenterParameters {
  // Region growing segmenter parameters.
  std::string segmenter_type = "IncrementalRegionGrowing";
  int min_cluster_size;
  int max_cluster_size;
  float radius_for_growing;

  // Parameters specific for the SmoothnessConstraint growing policy.
  float sc_smoothness_threshold_deg;
  float sc_curvature_threshold;
}; // struct SegmenterParameters
