#include "icf_pso.h"
#include "min_dist_func.h"
#include "math/evaluate_fitness_func.h"
namespace pmr
{
  IterativeClosestFace_PSO::IterativeClosestFace_PSO()
  {
  }

  void IterativeClosestFace_PSO::estimate(Eigen::Matrix4f & estimated_transf)
  {
    math::Swarm sw;
    //Config paras
    math::Pose minBound(-1, -1, -1, -M_PI/4, -M_PI/4, -M_PI/4);
    math::Pose maxBound(1, 1, 1, M_PI/4, M_PI/4, M_PI/4);
    math::Pose minVel(-0.2, -0.2, -0.2, -M_PI/60, -M_PI/60, -M_PI/60);
    math::Pose maxVel(0.2, 0.2, 0.2, M_PI/60, M_PI/60, M_PI/60);
    sw.setSearchSpaceBound(minBound,maxBound);
    sw.setSearchSpeedBound(minVel,maxVel);
    sw.setMaxIteration(50);
    sw.setPopulationSize(15);

    //boost::shared_ptr<math::EvaluateFitnessFunc> minDistFuncPtr(new MinDistFunc);
    MinDistFunc* minDistFuncPtr=new MinDistFunc;
    minDistFuncPtr->setModel(this->model_);
    minDistFuncPtr->setCloud(this->scene_point_cloud_);
    minDistFuncPtr->setThreshold(this->threshold_valid_);
    sw.setEvaluateFitnessFunc(minDistFuncPtr);
    //sw.setEvaluateFitnessFunc((math::EvaluateFitnessFunc &)minDistFunc);
    math::PSOSolution sol=sw.compute();
    
    std::cout<<"========================================"<<std::endl;
    std::cout<<"compute finished!"<<std::endl
      <<"gbest score "<<sol.gbest_value<<std::endl
      <<"gbest_pose  "<<sol.gbest_pose<<std::endl;

    estimated_transf=sol.gbest_pose.getTransform();
  }

}

