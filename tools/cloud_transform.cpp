#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <iostream>
#include <boost/lexical_cast.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudPtr;

  const Eigen::Matrix3f euler2matrix (const Eigen::Vector3f & euler_angle)
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


const Eigen::Vector3f matrix2euler (const Eigen::Matrix3f & mat_rotation)
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

    return (Eigen::Vector3f (alpha, beta, gamma));
  }

int main(int argc, char** argv)
{
  PointCloudPtr cloud(new PointCloudT);

  //load pcd data
  if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
  {
    std::cerr<<">>> ERROR: Load pcd data from "<<argv[1]
      <<" failed"<<std::endl;
    return -1;
  }
  std::cout<<">>> Load pcd data from "<<argv[1]
      <<" finished"<<std::endl;

  //transform by euler angle and translation
  Eigen::Vector3f euler;
  try
  {
    euler(0)=boost::lexical_cast<float>(argv[2])/180*M_PI;
    euler(1)=boost::lexical_cast<float>(argv[3])/180*M_PI;
    euler(2)=boost::lexical_cast<float>(argv[4])/180*M_PI;
  }
  catch(std::string msg)
  {
    std::cerr<<">>> ERROR: parse euler angles error!"<<std::endl;
    return -1;
  }
  Eigen::Vector3f translation;
  try
  {
    translation(0)=boost::lexical_cast<float>(argv[5]);
    translation(1)=boost::lexical_cast<float>(argv[6]);
    translation(2)=boost::lexical_cast<float>(argv[7]);
  }
  catch(std::string msg)
  {
    std::cerr<<">>> ERROR: parse translation error!"<<std::endl;
    return -1;
  }

  Eigen::Matrix3f rotMat = euler2matrix(euler);
  Eigen::Matrix4f transfMat=Eigen::Matrix4f::Identity();
  transfMat.block(0,0,3,3)=rotMat;
  transfMat.block(0,3,3,1)=translation;

  PointCloudPtr transCloud(new PointCloudT);
  pcl::transformPointCloud(*cloud,*transCloud, transfMat);

  std::cout<<">>> Transform cloud finished"<<std::endl;

  //save pcd file
  if(pcl::io::savePCDFile(argv[8],*transCloud)==-1)
  {
    std::cerr<<">>> ERROR: Save pcd file failed!"<<std::endl;
    return -1;
  }
  
  std::cout<<">>> Save pcd file to: "<<argv[8]<<" finished"<<std::endl;
  std::cout<<std::endl<<"    transform matrix: "<<transfMat<<std::endl;
}
