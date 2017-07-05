#ifndef PMR_STL2PCD_H_
#define PMR_STL2PCD_H_
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>
#include "common/model_stl.h"

namespace pmr
{
  class stl2pcdConverter
  {
    public:

      //typedefs
      typedef pcl::PointXYZ pointT;
      typedef pcl::PointCloud<pointT> pointCloudT;
      typedef pointCloudT::Ptr pointCloudTPtr;

      stl2pcdConverter();

      ~stl2pcdConverter();

      void setInputModel(const STLModel::Ptr model);
      //convert to pcd
      int convert(pointCloudTPtr & outputPCD);
      //using voxel grid to convert stl to pointcloud
      int convert_voxelgrid(pointCloudTPtr & outputPCD, float leafsize);

      //convert and save
      //bool convert(std::string & outputFileName);

    private:
      STLModel::Ptr model;
      bool hasPoint(pointCloudTPtr cloud,pointT point);
  };
}

#endif
