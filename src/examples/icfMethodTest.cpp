#include <iostream>
#include "registration/icf/icf.h"
#include "common/model_stl.h"
#include "io/stl_reader.h"
#include "visualization/stl_visualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "registration/icf/icf_pso.h"

int main(int argc, char** argv)
{
  if(argc!=3)
  {
    std::cout<<">>> Args length error!"<<std::endl<<">>> Need 2 arguments: [1]stlModel file name, [2]pcd file name"<<std::endl;
    return -1;
  }
  std::string modelFileName=argv[1];
  std::string pcdFileName=argv[2];

  //Load stl model
  pmr::STLModel::Ptr model(new pmr::STLModel);
  if(pmr::STLReader::loadStlModel(modelFileName,model)==-1)
  {
    std::cerr<<">>> ERROR: load stl model: "
      <<modelFileName<<" failed!"<<std::endl;
    return -1;
  }
  std::cout<<">>> Load stl model: "<<modelFileName
    <<" finished!"<<std::endl;
  std::cout<<">>> triangle size: "<<model->getTriangleSize()<<std::endl;
  
  //Load pcd data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(pcdFileName,*cloud)==-1)
  {
    std::cerr<<">>> ERROR: load pcd data: "
      <<pcdFileName<<" failed!"<<std::endl;
    return -1;
  }
  std::cout<<">>> Load pcd data: "<<pcdFileName
    <<" finished"<<std::endl;
  std::cout<<">>> point size: "<<cloud->points.size()<<std::endl;

  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(cloud);
  grid.setLeafSize(0.05f,0.05f,0.05f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
  grid.filter(*filteredCloud);
  std::cout<<">>> Sampled Cloud size: "<<filteredCloud->points.size()<<std::endl;

  //icf registration
  /*
  boost::shared_ptr<pmr::IterativeClosestFace> icf(new pmr::IterativeClosestFace);
  icf->setReferenceModel(model);
  icf->setScenePointCloud(filteredCloud);
  icf->setIterationOuter(28);
  icf->setIterationInner(28);
  icf->setThresholds(0.5,5.0,8.0);
   */

  //icf_pso registration
  boost::shared_ptr<pmr::IterativeClosestFace_PSO> icf(new pmr::IterativeClosestFace_PSO);
  icf->setReferenceModel(model);
  icf->setScenePointCloud(filteredCloud);
  icf->setThresholds(0.5,5.0,8.0);

  Eigen::Matrix4f init_transf=Eigen::Matrix4Xf::Identity(4,4);
  Eigen::Vector3f euler;
  euler(0)=0.0f;
  euler(1)=0.0f;
  euler(2)=0.0f;
  Eigen::Matrix3f rotMat=icf->euler2matrix(euler);
  Eigen::Vector3f trans;
  trans(0)=0.0f;
  trans(1)=0.0f;
  trans(2)=0.0f;
  init_transf.block(0,0,3,3)=rotMat;
  init_transf.block(0,3,3,1)=trans;
  icf->setInitialTransformation(init_transf);
  //std::cout<<"stub2"<<std::endl;
  Eigen::Matrix4f final_trans;
  icf->estimate(final_trans);
  std::cout<<">>> Icf computing funished"<<std::endl;
  std::cout<<">>> Transform:"<<std::endl<<final_trans<<std::endl;
  
  //visualize
  pmr::STLVisualizer stlViz;
  stlViz.setModelFileName(modelFileName);
  stlViz.showModel();

  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud,*finalCloud,final_trans);
  pcl::visualization::PCLVisualizer pcdViz("ICF Registration");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h1(cloud,150,150,150);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h2(finalCloud,10,150,10);

  pcdViz.addPointCloud(cloud,color_h1,"sourceCloud");
  pcdViz.addPointCloud(finalCloud,color_h2,"finalCloud");
  while(!pcdViz.wasStopped())
  {
    pcdViz.spinOnce();
  }

}
