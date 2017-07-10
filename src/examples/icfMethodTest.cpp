#include <iostream>
#include "registration/icf/icf.h"
#include "common/model_stl.h"
#include "io/stl_reader.h"
#include "conversion/stl2pcd.h"
#include "visualization/stl_visualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include "registration/icf/icf_pso.h"
#include "filter/stl_downSampler.h"

int main(int argc, char** argv)
{
  if(argc!=3)
  {
    std::cout<<">>> Need 2 arguments!"<<std::endl<<">>> [1]stlModel file name"
      <<">>> [2]pcd file name"<<std::endl;
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
 
  pmr::STLModel::Ptr sampledModel(new pmr::STLModel);
  //Downsample triangles
  if(model->getTriangleSize()>1000)
  {
    pmr::STLDownSampler downsampler;
    downsampler.setModel(model);
    downsampler.setRemainedTriangleSize(600);
    sampledModel=downsampler.filter();
    std::cout<<">>> Downsample stl model, remained triangles: "
      <<sampledModel->getTriangleSize()<<std::endl;
  }
  else
    sampledModel=model;

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
  float leaf_size=0.04f;
  grid.setLeafSize(leaf_size,leaf_size,leaf_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
  grid.filter(*filteredCloud);
  std::cout<<">>> Sampled Cloud size: "<<filteredCloud->points.size()<<std::endl;
    
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> nest;
  nest.setRadiusSearch (10.0*leaf_size);
  pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals(new pcl::PointCloud<pcl::PointNormal>);
  nest.setInputCloud (filteredCloud);
  nest.compute (*sourceNormals);
  std::cout<<">>> compute cloud normal finished"<<std::endl;

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
  ///*
  boost::shared_ptr<pmr::IterativeClosestFace_PSO> icf(new pmr::IterativeClosestFace_PSO);
  icf->setReferenceModel(sampledModel);
  icf->setScenePointCloud(filteredCloud);
  icf->setThresholds(0.5,5.0,8.0);
  //*/
  
  Eigen::Matrix4f init_transf=Eigen::Matrix4Xf::Identity(4,4);
  Eigen::Vector3f euler;
  euler(0)=0.8f;
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
  //pmr::STLVisualizer stlViz;
  //stlViz.setModelFileName(modelFileName);
  //stlViz.showModel();
  
  pmr::stl2pcdConverter converter;
  converter.setInputModel(sampledModel);
  pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudData(new pcl::PointCloud<pcl::PointXYZ>);
  
  if(converter.convert_voxelgrid(modelCloudData,0.005)!=-1)
  {
    std::cout<<">>> convert from stl to pcd finished"<<std::endl;
    std::cout<<">>> total point size "<<modelCloudData->points.size()<<std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud,*finalCloud,final_trans);
  pcl::visualization::PCLVisualizer pcdViz("ICF Registration");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h1(cloud,150,150,150);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h2(finalCloud,150,10,10);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h3(modelCloudData,10,150,10);

  pcdViz.addPointCloud(cloud,color_h1,"sourceCloud");
  pcdViz.addPointCloud(finalCloud,color_h2,"finalCloud");
  pcdViz.addPointCloud(modelCloudData,color_h3,"modelCloud");
  pcdViz.addCoordinateSystem(0.05f);
  while(!pcdViz.wasStopped())
  {
    pcdViz.spinOnce();
  }

}
