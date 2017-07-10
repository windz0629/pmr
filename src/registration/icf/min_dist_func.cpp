#include "min_dist_func.h"
#include "pcl/common/transforms.h"
#include "math/transform.h"
#include <cmath>
#include <pcl/features/normal_3d_omp.h>
namespace pmr
{
  MinDistFunc::MinDistFunc()
  {
   // _model=new STLModel;
    //_cloud=new pcl::PointCloud<pcl::PointXYZ>;
    //_cloudNormals=new pcl::PointCloud<pcl::PointNormal>;
  }

  void MinDistFunc::setModel(STLModel::Ptr & model)
  {
    _model=model;
    dist_measurer.setNumTriangles(_model->getTriangleSize());
    dist_measurer.setTriangles(_model->getAllTriangles());
    dist_measurer.setNormals(_model->normals);
  }

  void MinDistFunc::setCloud(PointCloudPtr & cloud)
  {
    _cloud=cloud;
  }
  
  void MinDistFunc::setCloudNormals(pcl::PointCloud<pcl::PointNormal>::Ptr & normals)
  {
    _cloudNormals=normals;
  }

  void MinDistFunc::setThreshold(float threshold)
  {
    threshold_valid_=threshold;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr trans_cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> nest;
  double MinDistFunc::evaluateFitness(math::Pose & pose)
  {
    pcl::transformPointCloud(*_cloud,*transformed_cloud,pose.getTransform());
    //pcl::transformPointCloud(*_cloudNormals,*trans_cloudNormals,pose.getTransform());
    
    /*
    nest.setRadiusSearch (10.0*0.01);
    nest.setInputCloud (transformed_cloud);
    nest.compute (*trans_cloudNormals);
    */

    float objective_value=0.0f;
    int count=0;
    for(int i=0;i<_cloud->points.size();++i)
    {
      float point[3];
      point[0]=static_cast<float>((*transformed_cloud).points[i].x);
      point[1]=static_cast<float>((*transformed_cloud).points[i].y);
      point[2]=static_cast<float>((*transformed_cloud).points[i].z);

      /*
      float normal[3];
      normal[0]=trans_cloudNormals->points[i].x;
      normal[1]=trans_cloudNormals->points[i].y;
      normal[2]=trans_cloudNormals->points[i].z;
      */

      float shortest_distance=dist_measurer.calShortestDistance(point);
      //float shortest_distance=dist_measurer.calShortestNormalDistance(normal);

      //std::cout<<"shortest_distance "<<shortest_distance<<std::endl;

      float fxp;
      if(shortest_distance<threshold_valid_)
        fxp=shortest_distance;
      else
        fxp=10;
      //std::cout<<"fxp "<<fxp<<std::endl;
      objective_value+=fabs(fxp);
  
    }

    //std::cout<<"objective_value= "<<objective_value<<"  size "<<_cloud->points.size()<<std::endl;
    double score=objective_value/_cloud->points.size();
    
    return score;
  }
}
