#include <iostream>
#include "common/model_stl.h"
#include "io/stl_reader.h"
#include "conversion/stl2pcd.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <boost/lexical_cast.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
float leafsize=0.005;

bool ransac_ia(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud,Eigen::Matrix4f & transform)
{ 
    pcl::SampleConsensusInitialAlignment<PointT,PointT,FeatureT> align;
    std::cout<<std::endl<<">>> Starting Ransac Initial Alignment..."<<std::endl;

   // Estimate normals
    pcl::NormalEstimationOMP<PointT,PointNT> nest;
    nest.setRadiusSearch (10.0*leafsize);
    PointCloudNT::Ptr sourceNormals(new PointCloudNT);
    PointCloudNT::Ptr targetNormals(new PointCloudNT);
    nest.setInputCloud (sourceCloud);
    nest.compute (*sourceNormals);
    nest.setInputCloud (targetCloud);
    nest.compute (*targetNormals);

    FeatureCloudT::Ptr sourceCloud_features(new FeatureCloudT);
    FeatureCloudT::Ptr targetCloud_features(new FeatureCloudT);
    // Estimate features
    //pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (10*leafsize);
    fest.setInputCloud (sourceCloud);
    fest.setInputNormals (sourceNormals);
    fest.compute (*sourceCloud_features);
    fest.setInputCloud (targetCloud);
    fest.setInputNormals (targetNormals);
    fest.compute (*targetCloud_features);

    //Perform alignment
    const float leaf = leafsize;
    align.setInputSource(sourceCloud);
    align.setSourceFeatures(sourceCloud_features);
    align.setInputTarget(targetCloud);
    align.setTargetFeatures(targetCloud_features);
    align.setMaximumIterations(500);
    align.setNumberOfSamples(5);
    align.setCorrespondenceRandomness(5);
    //align.setSimilarityThreshold(0.01f);
    align.setMaxCorrespondenceDistance(2.0f*leaf);
    //align.setInlierFraction(0.025f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ransacAlignCloud(new pcl::PointCloud<pcl::PointXYZ>);
    align.align(*ransacAlignCloud);
    if(align.hasConverged())
    {
      transform=align.getFinalTransformation();

        std::cout<<">>> Ransac Alignment has converged"<<std::endl;
        std::cout<<std::endl<<">>> Ransac Fitness score: "<<align.getFitnessScore()<<std::endl;
        std::cout<<">>> ransac transform matrix:"<<std::endl<<align.getFinalTransformation()<<std::endl;
        return true;
    }
    else
    {
        std::cerr<<">>> Ransac Alignment stopped without convergence!"<<std::endl;
        return false;
    }
}


int main(int argc, char** argv)
{
  //get leafsize
  leafsize=boost::lexical_cast<float>(argv[3]);

  pmr::STLModel::Ptr model(new pmr::STLModel);//(new pmr::STLModel);

  if(pmr::STLReader::loadStlModel(argv[1],model)!=-1)
  {
    std::cout<<">>> load stl model"<<std::endl;
    std::cout<<">>> triangle size: "<<model->getTriangleSize()<<std::endl;
    std::cout<<">>> normal size: "<<model->normals.size()<<std::endl;
  }

  //convert stl to pointcloud
  pmr::stl2pcdConverter converter;
  converter.setInputModel(model);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudData(new pcl::PointCloud<pcl::PointXYZ>);
  
  if(converter.convert_voxelgrid(cloudData,leafsize)!=-1)
  {
    std::cout<<">>> convert from stl to pcd finished"<<std::endl;
    std::cout<<">>> total point size "<<cloudData->points.size()<<std::endl;
  }
  else
  {
    std::cout<<">>> convert from stl to pcd failed"<<std::endl;
  }

  //icp
  pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2],*obs_cloud)==-1)
  {
    std::cout<<">>> Load pcd file failed!"<<std::endl;
    return -1;
  }
  else
  {
    std::cout<<std::endl<<">>> Load pcd finished, point cloud size: "<<obs_cloud->points.size()<<std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(obs_cloud);
  grid.setLeafSize(leafsize,leafsize,leafsize);
  grid.filter(*filtered_cloud);
  std::cout<<">>> After voxelgrid filter, point cloud size: "<<filtered_cloud->points.size()<<std::endl;

  Eigen::Matrix4f ransac_transform;
  ransac_ia(filtered_cloud, cloudData, ransac_transform);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ransacCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*filtered_cloud,*ransacCloud,ransac_transform);

  pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f transform;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(ransacCloud);
  icp.setInputTarget(cloudData);
  icp.setMaximumIterations(200);
  icp.align(*icp_cloud);
  std::cout<<">>> ICP computation finished"<<std::endl;
  if(icp.hasConverged())
  {
    std::cout<<">>> ICP converged"<<std::endl;
    std::cout<<">>> ICP fitness score: "<<icp.getFitnessScore()<<std::endl;
    std::cout<<">>> ICP final transform: "<<std::endl<<icp.getFinalTransformation()<<std::endl;
  }
  else
  {
    std::cout<<">>> ICP does not converge"<<std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*ransacCloud,*finalCloud,icp.getFinalTransformation());


  pcl::visualization::PCLVisualizer viewer("point_model_registration");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_cloud_color_h(cloudData,150,150,150);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> obs_cloud_color_h(filtered_cloud,0,100,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_cloud_color_h(finalCloud,100,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ransac_cloud_color_h(ransacCloud,0,0,100);

  viewer.addPointCloud<pcl::PointXYZ>(cloudData,model_cloud_color_h,"modelCloud");
  viewer.addPointCloud<pcl::PointXYZ>(filtered_cloud,obs_cloud_color_h,"obsCloud");
  viewer.addPointCloud<pcl::PointXYZ>(ransacCloud,ransac_cloud_color_h,"ransacCloud");
  viewer.addPointCloud<pcl::PointXYZ>(finalCloud,icp_cloud_color_h,"icpCloud");
  
  while(1)
  {
    if(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
  }
  
}
