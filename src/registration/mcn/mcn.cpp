#include "mcn.h"
namespace pmr
{
/**
 * @brief: 输入待匹配的stl模型
 */
void MeanClusteredNormalEstimation::setModel(STLModel::Ptr & model)
{
  _model=model;
}

/**
 * @brief: 输入观测到的点云
 */
void MeanClusteredNormalEstimation::setPointCloud(PointCloudPtrT & cloud)
{
  _cloud=cloud;
}

/**
 * 估算位姿
 */
Eigen::Matrix4f MeanClusteredNormalEstimation::estimate()
{

}
}
