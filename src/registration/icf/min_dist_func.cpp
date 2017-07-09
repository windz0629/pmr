#include "min_dist_func.h"
#include "pcl/common/transforms.h"
#include "math/transform.h"
namespace pmr
{
  MinDistFunc::MinDistFunc()
  {
  }

  void MinDistFunc::setModel(STLModel::Ptr & model)
  {
    _model=model;
    dist_measurer.setNumTriangles(_model->getTriangleSize());
    dist_measurer.setTriangles(_model->getAllTriangles());
    dist_measurer.setNormals(_model->normals);
  }

  void MinDistFunc::setCloud(PointCloudConstPtr & cloud)
  {
    _cloud=cloud;
  }

  void MinDistFunc::setThreshold(float threshold)
  {
    threshold_valid_=threshold;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  double MinDistFunc::evaluateFitness(math::Pose & pose)
  {
    //Eigen::Matrix4f transf=transCompute.computeTransform(pose);
    //Eigen::Matrix4f transf=pose.getTransform();
    pcl::transformPointCloud(*_cloud,*transformed_cloud,pose.getTransform());
    //std::cout<<transf<<std::endl;
    
    float objective_value=0.0f;
    for(int i=0;i<(*transformed_cloud).points.size();++i)
    {
      float point[3];
      point[0]=static_cast<float>((*transformed_cloud).points[i].x);
      point[1]=static_cast<float>((*transformed_cloud).points[i].y);
      point[2]=static_cast<float>((*transformed_cloud).points[i].z);

      float shortest_distance=dist_measurer.calShortestDistance(point);
      //std::cout<<"shortest distance: "<<shortest_distance<<std::endl;

      float fxp;
      if(shortest_distance<threshold_valid_)
        //fxp=1-shortest_distance/threshold_valid_;
        fxp=shortest_distance/threshold_valid_;
      else
        fxp=1000;

      objective_value+=fxp;
    }

    double score=objective_value/transformed_cloud->points.size();
    //std::cout<<"score= "<<score<<std::endl;
    return score;
    //return (objective_value/transformed_cloud->points.size());
  }
}
