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
      
      MinDistFunc();
      virtual void setModel(STLModel::Ptr & model);
      virtual void setCloud(PointCloudConstPtr & cloud);
      virtual void setThreshold(float threshold);
      virtual double evaluateFitness(math::Pose & pose);

    private:
      STLModel::Ptr _model;
      PointCloudConstPtr _cloud;
      float threshold_valid_;
      DistanceMeasurer dist_measurer;
  };
}
#endif
