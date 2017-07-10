#ifndef PMR_EVALUATE_FITNESS_FUNC_H
#define PMR_EVALUATE_FITNESS_FUNC_H
#include "pose.h"
#include "common/model_stl.h"
#include <pcl/point_cloud.h>
namespace pmr
{
  namespace math
  {
    class EvaluateFitnessFunc
    {
      public:
      virtual double evaluateFitness(Pose & p);
      virtual void setModel(STLModel::Ptr & model);
      virtual void setCloud(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &);
      virtual void setThreshold(double threshold);
      virtual void setCloudNormals(pcl::PointCloud<pcl::PointNormal>::Ptr & normals);
      EvaluateFitnessFunc();

      private:
      double fitnessScore;
    };
  }
}
#endif
