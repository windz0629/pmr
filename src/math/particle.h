#ifndef PMR_MATH_PATICLE_H
#define PMR_MATH_PARTICLE_H
#include "pose.h"
namespace pmr
{
  namespace math
  {
    class Particle
    {
      public:
      Pose velocity;
      Pose pbest_pose;
      double pbest_value;
      Pose pose;

      Particle();
      void setMinMaxBound(Pose & minpose, Pose & maxPose);
      void setMinMaxVelocity(Pose & minVel, Pose & maxPose);
      void randomize();

      Pose getRandomPose(Pose& min, Pose& max);
      friend std::ostream & operator <<(std::ostream & out, Particle & p);
      
      private:
      Pose minPos,maxPos;
      Pose minVel,maxVel;
    };
  }
}
#endif
