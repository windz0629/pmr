#include "evaluate_fitness_func.h"
namespace pmr
{
  namespace math
  {
  EvaluateFitnessFunc::EvaluateFitnessFunc()
  {
    fitnessScore=0;
  }

  double EvaluateFitnessFunc::evaluateFitness(Pose& p)
  {
    return fitnessScore;
  }

  void EvaluateFitnessFunc::setModel(STLModel::Ptr & model)
  {
  }

  void EvaluateFitnessFunc::setCloud(boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > & cloud)
  {
  }

  void EvaluateFitnessFunc::setThreshold(double threshold)
  {
  }

  void EvaluateFitnessFunc::setCloudNormals(pcl::PointCloud<pcl::PointNormal>::Ptr & normals)
  {
  }
  }
}
