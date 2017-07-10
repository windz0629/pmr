#include "icf_pso.h"
#include "min_dist_func.h"
#include "math/evaluate_fitness_func.h"
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
namespace pmr
{
  IterativeClosestFace_PSO::IterativeClosestFace_PSO()
  {
  }

  void IterativeClosestFace_PSO::estimate(Eigen::Matrix4f & estimated_transf)
  {
    math::Swarm sw;
    //Config paras
    math::Pose minBound(-1, -1, -1, -M_PI/2, -M_PI/2, -M_PI/2);
    math::Pose maxBound(1, 1, 1, M_PI/2, M_PI/2, M_PI/2);
    math::Pose minVel(-0.04, -0.04, -0.04, -M_PI/60, -M_PI/60, -M_PI/60);
    math::Pose maxVel(0.04, 0.04, 0.04, M_PI/60, M_PI/60, M_PI/60);
    sw.setSearchSpaceBound(minBound,maxBound);
    sw.setSearchSpeedBound(minVel,maxVel);
    sw.setMaxIteration(40);
    sw.setPopulationSize(20);

    //boost::shared_ptr<math::EvaluateFitnessFunc> minDistFuncPtr(new MinDistFunc);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > transCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*scene_point_cloud_,*transCloud, Eigen::Matrix4f::Identity(4,4));
  
    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> nest;
    nest.setRadiusSearch (10.0*0.01);
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(new pcl::PointCloud<pcl::PointNormal>);
    nest.setInputCloud (transCloud);
    nest.compute (*sourceNormals);
    
    MinDistFunc* minDistFuncPtr=new MinDistFunc;
    minDistFuncPtr->setModel(this->model_);
    minDistFuncPtr->setCloud(transCloud);
    minDistFuncPtr->setCloudNormals(sourceNormals);
    minDistFuncPtr->setThreshold(this->threshold_valid_);
    sw.setEvaluateFitnessFunc(minDistFuncPtr);
    //sw.setEvaluateFitnessFunc((math::EvaluateFitnessFunc &)minDistFunc);
    math::PSOSolution sol=sw.compute();
    
    std::cout<<"========================================"<<std::endl;
    std::cout<<"compute finished!"<<std::endl
      <<"gbest score "<<sol.gbest_value<<std::endl
      <<"gbest_pose  "<<sol.gbest_pose<<std::endl;

    estimated_transf=sol.gbest_pose.getTransform();
    std::cout<<"final trans"<<std::endl<<estimated_transf<<std::endl;
  }

}

