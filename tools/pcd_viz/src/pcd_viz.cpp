#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(argv[1],*cloud)==-1)
  {
    std::cerr<<">>> ERROR: Load pcd file: "<<argv[1]<<" failed"<<std::endl;
    return -1;
  }
  std::cout<<">>> Load pcd file: "<<argv[1]<<" finished"<<std::endl;

  pcl::visualization::PCLVisualizer viewer("pcd_viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_h(cloud,180,180,180);
  viewer.addPointCloud<pcl::PointXYZ>(cloud,color_h,"point cloud");

  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
}
