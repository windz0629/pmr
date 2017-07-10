#include "transform.h"
namespace pmr
{
  namespace math
  {
      const Eigen::Vector3f transform::matrix2euler (
        const Eigen::Matrix3f & mat_rotation)
      {
        const float PI = 3.1415926;
        const float EPS = 1.0E-8;

        float alpha;
        float beta;
        float gamma;

        // Assume beta is in [0,pi].
        double a_02 = mat_rotation (0,2);
        double a_01 = mat_rotation (0,1);
        double a_11 = mat_rotation (1,1);
        double a_12 = mat_rotation (1,2);
        double a_20 = mat_rotation (2,0);
        double a_21 = mat_rotation (2,1);
        double a_22 = mat_rotation (2,2);

        beta = std::atan2 (std::sqrt (std::pow (a_02,2)+std::pow (a_12,2)), a_22);

        if ((EPS < beta) && (beta < (PI-EPS)))
        {
          alpha = std::atan2 (a_12, a_02);
          gamma = std::atan2 (a_21, -a_20);
        }
        else if (beta <= EPS)
        {
          alpha = 0.0;
          gamma = std::atan2 (-a_01, a_11);
        }
        else
        {
          alpha = 0.0;
          gamma = std::atan2 (a_01, a_11);
        }
        Eigen::Vector3f res;
        res(0)=alpha;
        res(1)=beta;
        res(2)=gamma;
        return res;
     }

    const Eigen::Matrix3f transform::euler2matrix (
        const Eigen::Vector3f & euler_angle)
    {
      double phi = euler_angle[0];
      double theta = euler_angle[1];
      double psi = euler_angle[2];

      Eigen::Matrix3f mat_rotation;
      mat_rotation (0,0) = std::cos (phi)*std::cos (theta)*std::cos (psi) - std::sin (phi)*std::sin (psi);
      mat_rotation (0,1) = -std::cos (phi)*std::cos (theta)*std::sin (psi) - std::sin (phi)*std::cos (psi);
      mat_rotation (0,2) = std::cos (phi)*std::sin (theta);

      mat_rotation (1,0) = std::sin (phi)*std::cos (theta)*std::cos (psi) + std::cos (phi)*std::sin (psi);
      mat_rotation (1,1) = -std::sin (phi)*std::cos (theta)*std::sin (psi) + std::cos (phi)*std::cos (psi);
      mat_rotation (1,2) = std::sin (phi)*std::sin (theta);

      mat_rotation (2,0) = -std::sin (theta)*std::cos (psi);
      mat_rotation (2,1) = std::sin (theta)*std::sin (psi);
      mat_rotation (2,2) = std::cos (theta);

      return (mat_rotation);
    }

    Eigen::Matrix4f transform::computeTransform(
        Eigen::Matrix3f & rotMat,Eigen::Vector3f & position)
    {
      Eigen::Matrix4f transf=Eigen::Matrix4f::Identity(4,4);
      transf(0,0)=rotMat(0,0);
      transf(0,1)=rotMat(0,1);
      transf(0,2)=rotMat(0,2);
      transf(1,0)=rotMat(1,0);
      transf(1,1)=rotMat(1,1);
      transf(1,2)=rotMat(1,2);
      transf(2,0)=rotMat(2,0);
      transf(2,1)=rotMat(2,1);
      transf(2,2)=rotMat(2,2);
      transf(0,3)=position(0);
      transf(1,3)=position(1);
      transf(2,3)=position(2);
      //std::cout<<"position "<<position(0)<<" "<<position(1)<<" "<<position(2)<<std::endl;
      //std::cout<<transf<<std::endl;
      return transf;
    }

    Eigen::Matrix4f transform::computeTransform(
        Eigen::Vector3f & euler, Eigen::Vector3f & position)
    {
      Eigen::Matrix3f rotMat;
      rotMat=euler2matrix(euler);
      return computeTransform(rotMat,position);
    }

    Eigen::Matrix4f transform::computeTransform(Pose& pose)
    {
      return computeTransform(
          pose.getOrientation(),
          pose.getPosition());
    }

    void transform::computePose(const Eigen::Matrix4f& transform,
        Eigen::Vector3f& euler,Eigen::Vector3f& position)
    {
      euler=matrix2euler(transform.block(0,0,3,3));
      position=transform.block(0,3,3,1);
    }

    /*
    int computePose(const Eigen::Matrix4f transform, Pose& pose)
    {
      pose.setTransform(transform);
    }
    */

  }
}
