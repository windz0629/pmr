#ifndef PMR_ICF_PSO_H
#define PMR_ICF_PSO_H
#include "math/swarm.h"
#include "math/pose.h"
#include "min_dist_func.h"
#include "icf.h"

namespace pmr
{
  class IterativeClosestFace_PSO:public IterativeClosestFace
  {
    public:
      IterativeClosestFace_PSO();
      void estimate (Eigen::Matrix4f & estimated_transf);
  };
}
#endif
