#include <memory>
#include <stddef.h>
#include <unordered_set>
#include <vector>

#include "mesh_segmentation_ros/MeshSeg.h"

// Forward declaration to speed up compilation time.
class SegmentedCloud;

/// \brief Generic incremental region growing segmenter.
/// Extract segments by growing regions in the point cloud according to a specific policy. Allows
/// to update already segmented clouds by segmenting new points only.
class IncrementalSegmenter {
 public:
  IncrementalSegmenter(const SegmenterParameters& params);

  /// \brief Cluster the given point cloud, writing the found segments in the segmented cloud. Only
  /// points that are not assigned to a cluster (have the PolicyName::getPointClusterId(point)
  /// equal to zero) will be used as candidate seeds.
  /// If cluster IDs change, the \c cluster_ids_to_segment_ids mapping is updated accordingly.
  /// \param normals The normal vectors of the point cloud. This can be an empty cloud if the
  /// the segmenter doesn't require normals.
  /// \param is_point_modified Indicates for each point if it has been modified such that its
  /// cluster assignment may change.
  /// \param cloud The point cloud that must be segmented.
  /// \param points_neighbors_provider Object providing nearest neighbors information.
  /// \param segmented_cloud Cloud to which the valid segments will be added.
  /// \param cluster_ids_to_segment_ids Mapping between cluster IDs and segment IDs. Cluster
  /// \c i generates segment \c cluster_ids_to_segments_ids[i]. If
  /// \c cluster_ids_to_segments_ids[i] is equal to zero, then the cluster does not contain enough
  /// points to be considered a segment.
  /// \param renamed_segments Vectors containing segments that got a new ID, e.g. after merging
  /// two or more segments. The ordering of the vector represents the sequence of renaming
  /// operations. The first ID in each pair is the renamed segments, the second ID is the new
  /// segment ID.
  void segment(const CloudN& normals, const std::vector<bool>& is_point_modified,
               CloudXYZL& cloud, PCLSearchTree& points_neighbors_provider,
               CloudXYZL& segmented_cloud,
               std::vector<Id>& cluster_ids_to_segment_ids,
               std::vector<std::pair<Id, Id>>& renamed_segments) override;

 private:
  typedef uint32_t ClusterId;

  // Helper data structures for discovering and linking partial clusters.
  struct PartialClustersSet {
    ClusterId cluster_id = kUnassignedClusterId;
    Id segment_id = kNoId;
    std::unordered_set<size_t> partial_clusters_indices;
  };
  typedef std::shared_ptr<PartialClustersSet> PartialClustersSetPtr;

  struct PartialCluster {
    PartialCluster() : partial_clusters_set(new PartialClustersSet()) { }
    std::vector<size_t> point_indices;
    PartialClustersSetPtr partial_clusters_set;
  };
  typedef std::vector<PartialCluster> PartialClusters;

  // Determine the ID of a segment created by the merging of segments with IDs \c id_1 and \c id_2.
  // In case two segments are merged, the first ID is the discarded ID, and the second ID is the
  // ID used for the merged segment. In the other cases the first ID specifies if one of the
  // segments was kInvId or kNoId.
  std::pair<Id, Id> mergeSegmentIds(Id id_1, Id id_2) const;

  // Specifies that two partial clusters belong to the same cluster and must be linked. As a result
  // of this operation, both partial clusters will point to the same partial clusters set.
  void linkPartialClusters(size_t partial_cluster_1_index, size_t partial_cluster_2_index,
                           PartialClusters& partial_clusters,
                           std::vector<std::pair<Id, Id>>& renamed_segments) const;

  // Grows a region starting from the specified seed point. This finds all the new points belonging
  // to the same cluster and possibly links to existing clusters. The resulting partial cluster is
  // added to the \c partial_clusters vector.
  void growRegionFromSeed(const CloudN& normals, const CloudXYZL& cloud,
                          PCLSearchTree& points_neighbors_provider,
                          size_t seed_index, std::vector<bool>& processed,
                          PartialClusters& partial_clusters,
                          std::vector<std::pair<Id, Id>>& renamed_segments) const;

  // Clusters a point cloud. Only new or modified points are used as seeds.
  void growRegions(const CloudN& normals, const std::vector<bool>& is_point_modified,
                   const std::vector<Id>& cluster_ids_to_segment_ids, CloudXYZL& cloud,
                   PCLSearchTree& points_neighbors_provider, PartialClusters& partial_clusters,
                   std::vector<std::pair<Id, Id>>& renamed_segments) const;

  // Assign cluster indices to the sets of partial clusters, so that linked clusters have the same
  // cluster index and clusters use contiguous indices starting from 1. Returns the total number of
  // clusters.
  size_t assignClusterIndices(const PartialClusters& partial_clusters) const;

  // Write the cluster indices in the point cloud so that they can be reused in future
  // segmentations.
  void writeClusterIndicesToCloud(const PartialClusters& partial_clusters,
                                  CloudXYZL& cloud) const;

  // Adds the segments to a segmented cloud and updates \c cluster_ids_to_segment_ids to reflect
  // the new mapping between cluster IDs and segment IDs.
  void addSegmentsToSegmentedCloud(const CloudXYZL& cloud,
                                   const PartialClusters& partial_clusters, size_t num_clusters,
                                   std::vector<Id>& cluster_ids_to_segment_ids,
                                   CloudXYZL& segmented_cloud) const;

  // Get the total number of points contained in the cluster of which the partial cluster at index
  // \i partial_cluster_index is part.
  size_t getClusterSize(const PartialClusters& partial_clusters,
                        size_t partial_cluster_index) const;

  // Get the indices of the points contained in the cluster of which the partial cluster at index
  // \i partial_cluster_index is part.
  std::vector<int> getClusterIndices(const PartialClusters& partial_clusters,
                                     size_t partial_cluster_index) const;

  // Determines if a point is assigned to a cluster or not.
  bool isPointAssignedToCluster(const PointXYZL& point) const noexcept;

  // Gets the cluster ID of a point.
  ClusterId getClusterId(const PointXYZL& point) const noexcept;

  // Sets the cluster ID of a point.
  void setClusterId(PointXYZL& point, ClusterId cluster_id) const noexcept;

  // Segmenter settings.
  SegmenterParameters SegParams;
  const double search_radius_;
  const int min_segment_size_;
  const int max_segment_size_;

  static constexpr ClusterId kUnassignedClusterId = 0u;
}; // class IncrementalSegmenter
