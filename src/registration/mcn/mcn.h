#ifndef PMR_REGISTRATION_MCN_H
#define PMR_REGISTRATION_MCN_H
#include "common/model_stl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace pmr
{
  class MeanClusteredNormalEstimation
  {
    public:
      typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtrT;
      void setModel(STLModel::Ptr & model);
      void setPointCloud(PointCloudPtrT & cloud);
      Eigen::Matrix4f estimate();

    private:
      STLModel::Ptr _model;
      PointCloudPtrT _cloud;

      std::vector<Eigen::Vector3f> clusterNormals(std::vector<Eigen::Vector3f> inputNormals);
  };
}
#endif
