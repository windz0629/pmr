#ifndef PMR_MATH_POSE_H
#define PMR_MATH_POSE_H
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>
#include <iostream>
namespace pmr
{
  namespace math
  {
    class Pose
    {
      public:
        float x;
        float y;
        float z;
        float phi;
        float theta;
        float psi;

        Pose();
        Pose(float x, float y, float z, 
            float phi, float theta, float psi);
        Pose(const Pose & p);
        Pose(Eigen::Vector3f position, Eigen::Vector3f orientation);
        ~Pose();
        
        void setPosition(Eigen::Vector3f position);
        void setPosition(float x, float y, float z);
        void setOrientation(Eigen::Vector3f orientation);
        void setOrientation(float phi, float theta, float psi);
        void setTransform(Eigen::Matrix4f transform);

        Eigen::Vector3f & getPosition();
        Eigen::Vector3f & getOrientation();
        Eigen::Matrix4f getTransform();
        
        Pose operator +(const Pose & p);
        Pose operator -(const Pose & p);
        friend Pose operator *(const double f, Pose p);
        friend std::ostream & operator <<(std::ostream & out, Pose & p);

        typedef boost::shared_ptr<Pose> Ptr;

      private:
        Eigen::Vector3f _orientation;
        Eigen::Vector3f _position;
    };
  }
}
#endif
