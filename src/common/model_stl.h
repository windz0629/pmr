/*
 * Data structure which describe a stl model
 */

#ifndef PMR_MESH_H_
#define PMR_MESH_H_

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>

namespace pmr
{
  class STLModel
  {
    public:
      STLModel ();
      ~STLModel ();

      typedef boost::shared_ptr<STLModel> Ptr;
      std::vector<std::vector<Eigen::Vector3f> > triangles;
      std::vector<Eigen::Vector3f> normals;

      std::vector<std::vector<Eigen::Vector3f> > 
        getAllTriangles() const;
      std::vector<Eigen::Vector3f> 
        getNormals() const;

      int
        getTriangleSize() const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#endif
