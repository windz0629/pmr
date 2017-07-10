#ifndef PMR_MIN_DIST_FUNC_H
#define PMR_MIN_DIST_FUNC_H
#include "math/evaluate_fitness_func.h"
#include "math/pose.h"
#include "common/model_stl.h"
#include "distance_measurer.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace pmr
{
  class MinDistFunc:public math::EvaluateFitnessFunc
  {
    public:
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      
      
      MinDistFunc();
      virtual void setModel(STLModel::Ptr & model);
      virtual void setCloud(PointCloudPtr & cloud);
      virtual void setThreshold(float threshold);
      virtual double evaluateFitness(math::Pose & pose);
      void setCloudNormals(pcl::PointCloud<pcl::PointNormal>::Ptr & normals);

    private:
      STLModel::Ptr _model;//(new STLModel);
      PointCloudConstPtr _cloud;//(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointNormal>::Ptr _cloudNormals;//(new pcl::PointCloud<pcl::PointNormal>);
      float threshold_valid_;
      DistanceMeasurer dist_measurer;
  };
}
#endif
