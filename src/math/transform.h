#ifndef PMR_MATH_TRANSFORM_H
#define PMR_MATH_TRANSFORM_H
#include "pose.h"
#include <Eigen/Dense>
namespace pmr
{
  namespace math
  {
    /*transform is a class for converting transform matrix 
     * and pose description
     * */
    class transform
    {
      public:
        
        //convert euler angles to rotation matrix
        static const Eigen::Matrix3f euler2matrix(
            const Eigen::Vector3f& euler);

        //convert rotation matrix to euler angles
        static const Eigen::Vector3f matrix2euler(
            const Eigen::Matrix3f& matrix);
        
        /*compute transform matrix by rotation matrix 
         * and position vector*/
        static Eigen::Matrix4f computeTransform(Eigen::Matrix3f & rotMat,Eigen::Vector3f & position);
        
        /*compute transform matrix by euler angles 
         * and position vector*/
        static Eigen::Matrix4f computeTransform(Eigen::Vector3f & euler,Eigen::Vector3f & position);
        
        /*compute thransform matrix by pose*/
        static Eigen::Matrix4f computeTransform(Pose& _pose);

        /*compute euler angles and position from transform matrix*/
        static void computePose(const Eigen::Matrix4f& transform, Eigen::Vector3f& euler, Eigen::Vector3f& position);
        
        /*compute pose from transform matrix*/
        //static int computePose(const Eigen::Matrix4f& transform, Pose& pose);
    };
  }
}
#endif
