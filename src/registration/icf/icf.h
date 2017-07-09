/*
 * Iterative Closest Face (ICF) algorithm.
 */

#ifndef PMR_ICF_H_
#define PMR_ICF_H_

//#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>
#include "common/model_stl.h"

namespace pmr
{
  class IterativeClosestFace
  {
    public:
      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

      IterativeClosestFace ();
      IterativeClosestFace (const STLModel::Ptr model, const PointCloudConstPtr & scene_point_cloud);
      ~IterativeClosestFace ();

      void
      setReferenceModel (const STLModel::Ptr model);

      void
      setScenePointCloud (const PointCloudConstPtr & scene_point_cloud);

      /*!
       * \fn void setIndices (const IndicesConstPtr & indices)
       * \brief Set the indices of the points which will be involved in ICF algorithm. It can accelerate the
       * computation of ICF algorithm and is usually called during the coarse recognition. When 'indices' is empty,
       * all the points will be used in ICF algorithm.
       * \param[in] indices The indices of the points which will be involved in ICF algorithm.
       */
      void
      setIndices (const IndicesConstPtr & indices);

      void
      setVariationTranslation (const Eigen::Vector3f & left_board, const Eigen::Vector3f & right_board);

      void
      setVariationRotation (const Eigen::Vector3f & left_board, const Eigen::Vector3f & right_board);

      void
      setIterationOuter (int iteration_outer);

      void
      setIterationInner (int iteration_inner);

      void
      setInitialTransformation (const Eigen::Matrix4f & init_transf);

      void
      setThresholds (float near, float extreme, float valid);

      const STLModel::Ptr
      getReferenceModel () const;

      PointCloudConstPtr
      getScenePointCloud () const;

      void
      getVariationTranslation (Eigen::Vector3f & left_board, Eigen::Vector3f & right_board);

      void
      getVariationRotation (Eigen::Vector3f & left_board, Eigen::Vector3f & right_board);

      int
      getIterationOuter ();

      int
      getIterationInner ();

      const Eigen::Matrix4f
      getInitialTransformation ();

      void
      getThresholds (float & near, float & extreme, float & valid);

      float
      calObjectiveValue (const Eigen::Matrix4f & mat_transf);

      virtual void 
      estimate (Eigen::Matrix4f & estimated_transf);
  
      const Eigen::Vector3f
      matrix2euler (const Eigen::Matrix3f & mat_rotation);
      const Eigen::Matrix3f
      euler2matrix (const Eigen::Vector3f & euler_angle);

    protected:
      std::string model_file_;
      STLModel::Ptr model_;
      PointCloudConstPtr scene_point_cloud_;
      PointCloudConstPtr point_cloud_used_;
      IndicesConstPtr indices_;
      bool use_indices_;

      Eigen::Vector3f left_board_translation_;
      Eigen::Vector3f right_board_translation_;
      Eigen::Vector3f left_board_rotation_;
      Eigen::Vector3f right_board_rotation_;

      std::size_t iteration_outer_;
      std::size_t iteration_inner_;
      Eigen::Matrix4f init_transf_;
      float threshold_distance_near_;
      float threshold_distance_extreme_;
      float threshold_valid_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }; // class IterativeClosestFace

  const Eigen::Vector3f
  uniformRandom (const Eigen::Vector3f & min_boundary, const Eigen::Vector3f & max_boundary);
  const Eigen::Vector3f
  gaussianRandom (const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation);
  const Eigen::Vector3f
  gaussianRandom(const Eigen::Vector3f & mean, float deviation);


  // Calculate the distance from a point to a triangle mesh.
  float
  distPointTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);
  // Calculate the projected point on a plane.
  Eigen::Vector3f
  pointProjectionOnPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);
  // Detect if the projection of a point is inside the triangle.
  bool
  isPointInTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);
  // Calculate the distance from a point to another point.
  float
  distPointPoint (const Eigen::Vector3f & point_a, const Eigen::Vector3f & point_b);
  // Calculate the distance from a point to a line segment.
  float
  distPointLineSegment (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & segment_vertices);
  // Calculate the distance from a point to a plane.
  float
  distPointPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);

} 

#endif
