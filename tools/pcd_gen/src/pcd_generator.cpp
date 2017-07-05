#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include "conversion/stl2pcd.h"
#include "io/stl_reader.h"

int main(int argc, char** argv)
{
  //Load stl model
  std::string modelName(argv[1]);
  pmr::STLModel::Ptr model(new pmr::STLModel);
  
  if(pmr::STLReader::loadStlModel(modelName, model)==-1)
  {
    std::cout<<">>> Load stl model failed"<<std::endl;
    return -1;
  }
  std::cout<<">>> Load stl model finished"<<std::endl;

  //Convert to pcd
  float leafsize=0.01;
  leafsize=boost::lexical_cast<float>(argv[3]);
  pmr::stl2pcdConverter converter;
  converter.setInputModel(model);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  converter.convert_voxelgrid(cloud,leafsize);
  std::cout<<">>> Convert stl to pcd, point size= "<<cloud->points.size()<<std::endl;
  std::cout<<">>> cloud width= "<<cloud->width<<"   height= "<<cloud->height<<std::endl;

  //Write pcd to file
  pcl::io::savePCDFile(argv[2],*cloud);
  std::cout<<">>> Write pcd to file "<<argv[2]<<std::endl;

  pcl::visualization::PCLVisualizer viewer("stl2pcd viewer");
  viewer.addPointCloud<pcl::PointXYZ>(cloud);
  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  
}
