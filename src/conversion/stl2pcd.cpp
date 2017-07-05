#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include "stl2pcd.h"

namespace pmr
{

  //typedef pcl::PointXYZ pointT;
  //typedef pcl::PointCloud<pointT> pointCloudT;
  //typedef pointCloudT::Ptr pointCloudTPtr;
  
  stl2pcdConverter::stl2pcdConverter():model(new STLModel)
  {
  }

  stl2pcdConverter::~stl2pcdConverter()
  {
  }

  //check if the point cloud contains the point
  //has--returns true
  //has not--returns false
  bool stl2pcdConverter::hasPoint(pointCloudTPtr cloud,pointT point)
  {
    if(cloud->points.size()<=0)
      return false;
     for(size_t i=0;i<cloud->points.size();++i)
     {
       if(cloud->points[i].x==point.x
           && cloud->points[i].y==point.y
           && cloud->points[i].z==point.z)
         return true;
     }
     return false;
  }

  void stl2pcdConverter::setInputModel(const STLModel::Ptr model)
  {
    this->model=model;
  }

  int stl2pcdConverter::convert(pointCloudTPtr& cloudData)
  {
    try
    {
      for(int i=0;i<model->getTriangleSize();++i)
      {
        std::vector<Eigen::Vector3f> triangle;
        triangle=model->triangles.at(i);

        for(int j=0;j<3;++j)
        {
          Eigen::Vector3f p=triangle.at(j);
          pcl::PointXYZ point;
          point.x=p(0);
          point.y=p(1);
          point.z=p(2);
          if(!hasPoint(cloudData,point))
          {
            cloudData->points.push_back(point);
          }
        }
      }
    }
    catch(std::string msg)
    {
      return -1;
    }
    return 1;
  } 

  int getMinMax3D(STLModel::Ptr & model, Eigen::Vector3f& min, Eigen::Vector3f& max)
  {
    if(model->getTriangleSize()<=0)
      return -1;

    min=(model->triangles.at(0)).at(0);
    max=min;

    for(int i=0;i<model->getTriangleSize();++i)
    {
      std::vector<Eigen::Vector3f> triangle;
      triangle=model->triangles.at(i);
      for(int j=0;j<3;++j)
      {
        Eigen::Vector3f p=triangle.at(j);
        if(min(0)>p(0))
          min(0)=p(0);
        if(min(1)>p(1))
          min(1)=p(1);
        if(min(2)>p(2))
          min(2)=p(2);

        if(max(0)<p(0))
          max(0)=p(0);
        if(max(1)<p(1))
          max(1)=p(1);
        if(max(2)<p(2))
          max(2)=p(2);
      }
      //Eigen::Vector3f p1=triangle.at(1);
      //Eigen::Vector3f p2=triangle.at(2);

    }
    return 1;
  }

  int getMinMax3D(std::vector<Eigen::Vector3f> pointList, Eigen::Vector3f& min, Eigen::Vector3f& max)
  {
    if(pointList.size()<=0)
      return -1;

    min=pointList.at(0);
    max=min;

    for(int i=0;i<pointList.size();++i)
    {
      Eigen::Vector3f p=pointList.at(i);
      for(int j=0;j<3;++j)
      {
        if(min(j)>p(j))
          min(j)=p(j);

        if(max(j)<p(j))
          max(j)=p(j);
      }
    }
    return 1;
  }

  bool isPointOnPlane(std::vector<Eigen::Vector3f> triangle, Eigen::Vector3f normal, Eigen::Vector3f point)
  {
    float defaultThreshold=1e-5;
    Eigen::Vector3f vec;
    vec(0)=point(0)-(triangle.at(0))(0);
    vec(1)=point(1)-(triangle.at(0))(1);
    vec(2)=point(2)-(triangle.at(0))(2);
    //std::cout<<"ispop: "<<vec.dot(normal)<<std::endl;
    //if(vec(0)*normal(0)+vec(1)*normal(1)+vec(2)*normal(2) <defaultThreshold)
    if(fabs(vec.dot(normal))<defaultThreshold)
      return true;
    else
      return false;
  }

  bool isTriangleInCube(std::vector<Eigen::Vector3f> triangle, std::vector<Eigen::Vector3f> cube)
  {
    Eigen::Vector3f min_tri;
    Eigen::Vector3f max_tri;
    Eigen::Vector3f min_cube;
    Eigen::Vector3f max_cube;
    min_tri=(Eigen::Vector3f)(triangle.at(0));
    max_tri=min_tri;
    min_cube=(Eigen::Vector3f)(cube.at(0));
    max_cube=min_cube;

    for(int i=0;i<triangle.size();++i)
    {
      for(int j=0;j<3;++j)
      {
        if(min_tri(j)>triangle.at(i)(j))
          min_tri(j)=triangle.at(i)(j);
        if(max_tri(j)<triangle.at(i)(j))
          max_tri(j)=triangle.at(i)(j);
      }
    }

    for(int i=0;i<cube.size();++i)
    {
      for(int j=0;j<3;++j)
      {
        if(min_cube(j)>cube.at(i)(j))
          min_cube(j)=cube.at(i)(j);
        if(max_cube(j)<cube.at(i)(j))
          max_cube(j)=cube.at(i)(j);
      }
    }

    if(min_tri(0)<=max_cube(0) && max_tri(0)>=min_cube(0) &&
       min_tri(1)<=max_cube(1) && max_tri(1)>=min_cube(1) &&
       min_tri(2)<=max_cube(2) && max_tri(2)>=min_cube(2))
      return true;
    else
      return false;
  }

  void getCube(std::vector<Eigen::Vector3f> &cube, int i, int j, int k, float leafsize)
  {
    //std::vector<Eigen::Vector3f> cube;
    Eigen::Vector3f p0(i*leafsize,j*leafsize,k*leafsize);
    Eigen::Vector3f p1(i*leafsize,j*leafsize,(k+1)*leafsize);
    Eigen::Vector3f p2(i*leafsize,(j+1)*leafsize,k*leafsize);
    Eigen::Vector3f p3(i*leafsize,(j+1)*leafsize,(k+1)*leafsize);
    Eigen::Vector3f p4((i+1)*leafsize,j*leafsize,k*leafsize);
    Eigen::Vector3f p5((i+1)*leafsize,j*leafsize,(k+1)*leafsize);
    Eigen::Vector3f p6((i+1)*leafsize,(j+1)*leafsize,k*leafsize);
    Eigen::Vector3f p7((i+1)*leafsize,(j+1)*leafsize,(k+1)*leafsize);
    
    cube.push_back(p0);
    cube.push_back(p1);
    cube.push_back(p2);
    cube.push_back(p3);
    cube.push_back(p4);
    cube.push_back(p5);
    cube.push_back(p6);
    cube.push_back(p7);
  }

  float random_float(float min, float max)
  {
    std::srand((int)(time(NULL)));
    return min+(max-min)*((float)rand()/(float)RAND_MAX);
  }


  boost::mt19937 rand_gen;

  const Eigen::Vector3f uniformRandom(const Eigen::Vector3f & min_bound, const Eigen::Vector3f max_bound)
  {
    
    Eigen::Vector3f random_value;
    //boost::random_device rand_device;
    boost::uniform_real<float> distr_x(min_bound(0),max_bound(0));
    boost::uniform_real<float> distr_y(min_bound(1),max_bound(1));
    boost::uniform_real<float> distr_z(min_bound(2),max_bound(2));
    random_value(0)=distr_x(rand_gen);
    random_value(1)=distr_y(rand_gen);
    random_value(2)=distr_z(rand_gen);
    

    //random_value(0)=random_float(min_bound(0),max_bound(0));
    //random_value(1)=random_float(min_bound(1),max_bound(1));
    //random_value(2)=random_float(min_bound(2),max_bound(2));
    return random_value;
  }

  int stl2pcdConverter::convert_voxelgrid(pointCloudTPtr & outputPCD, float leafsize)
  {
    Eigen::Vector3f min,max;
    getMinMax3D(model,min,max);

    //compute the box boundary
    Eigen::Vector3i minBoxIdx, maxBoxIdx;
    for(int i=0;i<3;++i)
    {
      minBoxIdx(i)=min(i)/leafsize;
      if(minBoxIdx(i)<0)
        minBoxIdx(i)-=1;
      else
        minBoxIdx(i)+=1;

      maxBoxIdx(i)=max(i)/leafsize;
      if(maxBoxIdx(i)<0)
        maxBoxIdx(i)-=1;
      else
        maxBoxIdx(i)+=1;
    }

    for(int idx=0;idx<model->getTriangleSize();++idx)
    {
      std::vector<Eigen::Vector3f> triangle=model->triangles.at(idx);
      Eigen::Vector3f min,max;
      getMinMax3D(triangle,min,max);
      Eigen::Vector3i minIdx,maxIdx;
      for(int i=0;i<3;++i)
      {
        minIdx(i)=min(i)/leafsize;
        if(minIdx(i)<0)
          minIdx(i)-=1;
        else
          minIdx(i)+=1;
        maxIdx(i)=max(i)/leafsize;
        if(maxIdx(i)<0)
          maxIdx(i)-=1;
        else
          maxIdx(i)+=1;
      }

      //std::cout<<"min "<<minIdx(0)<<"  "<<minIdx(1)<<"  "<<minIdx(2)<<std::endl;
      //std::cout<<"max  "<<maxIdx(0)<<"  "<<maxIdx(1)<<"  "<<maxIdx(2)<<std::endl;
      for(int i=minIdx(0);i<=maxIdx(0);++i)
      {
        for(int j=minIdx(1);j<=maxIdx(1);++j)
        {
          for(int k=minIdx(2);k<=maxIdx(2);++k)
          {
            std::vector<Eigen::Vector3f> cube;
            getCube(cube,i,j,k,leafsize);
            if(!isTriangleInCube(triangle,cube))
              continue;
            int max_iter=10;
            int times=0;
            while(times<max_iter)
            {
              times++;
              Eigen::Vector3f tmpPoint=uniformRandom(cube.at(0),cube.at(7));

              Eigen::Vector3f normal=model->normals.at(idx);
              if(isPointOnPlane(triangle,normal,tmpPoint))
              {
                pcl::PointXYZ point;
                point.x=tmpPoint(0);
                point.y=tmpPoint(1);
                point.z=tmpPoint(2);
                outputPCD->points.push_back(point);
                break;
              }
            }
          }
        }
      }


    }

    outputPCD->width=outputPCD->points.size();
    outputPCD->height=1;
/*
    for(int i=minBoxIdx(0);i<maxBoxIdx(0);++i)
    {
      for(int j=minBoxIdx(1);j<maxBoxIdx(1);++j)
      {
        for(int k=minBoxIdx(2);k<maxBoxIdx(2);++k)
        {
          std::vector<Eigen::Vector3f> cube;
          getCube(cube,i,j,k,leafsize);
          //std::cout<<"tmp point: "<<tmpPoint(0)<<" , "<<tmpPoint(1)<<" , "<<tmpPoint(2)<<" ,"<<std::endl;
          for(int idx=0; idx<model->getTriangleSize();++idx)
          {
            std::vector<Eigen::Vector3f> triangle=model->triangles.at(idx);
            if(!isTriangleInCube(triangle,cube))
              continue;

            //bool findOne=false;
            int max_Iter=10;
            int times=0;
            while(times<max_Iter)
            {
              times++;
              Eigen::Vector3f tmpPoint=uniformRandom(cube.at(0),cube.at(7));
              //std::cout<<"randomPoint.x= "<<tmpPoint(0)<<"  y= "<<tmpPoint(1)<<"  z= "<<tmpPoint(2)<<std::endl;
              Eigen::Vector3f normal=model->normals.at(idx);
              if(isPointOnPlane(triangle,normal,tmpPoint))
              {
                //std::cout<<"find one point"<<std::endl;
                //findOne=true;
                pcl::PointXYZ point;
                point.x=tmpPoint(0);
                point.y=tmpPoint(1);
                point.z=tmpPoint(2);
                outputPCD->points.push_back(point);
                break;
              }
            }
          }
        }
      }
    }
   */ 
  }

  /*
  bool stl2pcdConverter::convert(pointCloudTPtr& cloudData)
  {
    std::ifstream stream;
    stream.open(fileName.c_str(),std::ios::in);
    if(!stream.is_open())
    {
      std::cout<<">>> Error: cannot open model file "
        <<fileName<<std::endl;
      return false;
    }

    int stp=0;
    while(!stream.eof())
    {
      std::string line="";
      std::getline(stream,line);
      boost::trim(line);

      if(line.find("outer loop")==-1)
        continue;
      while(line.find("endloop")==-1)
      {
        std::getline(stream,line);
        if(line.find("vertex")!=-1)
        {
          std::vector<std::string> vertexData;
          boost::split(vertexData,line,boost::is_any_of((" ")));
          pcl::PointXYZ point;
          point.x=boost::lexical_cast<float>(vertexData[1]);
          point.y=boost::lexical_cast<float>(vertexData[2]);
          point.z=boost::lexical_cast<float>(vertexData[3]);
          if(!hasPoint(cloudData,point))
          {
            cloudData->points.push_back(point);
          }
        }
      }
    }
    std::cout<<">>> point cloud size: "<<cloudData->points.size()<<std::endl;
    stream.close();
    return true;
  }

  bool stl2pcdConverter::convert(const std::string & fileName,std::string & pcdFileName)
  {
    std::ifstream stream;
    stream.open(fileName.c_str(),std::ios::in);
    if(!stream.is_open())
    {
      std::cout<<">>> Error: cannot open model file "
        <<fileName<<std::endl;
      return false;
    }

    int stp=0;
    pointCloudTPtr cloudData(new pointCloudT);
    while(!stream.eof())
    {
      std::string line="";
      std::getline(stream,line);
      boost::trim(line);

      if(line.find("outer loop")==-1)
        continue;
      while(line.find("endloop")==-1)
      {
        std::getline(stream,line);
        if(line.find("vertex")!=-1)
        {
          std::vector<std::string> vertexData;
          boost::split(vertexData,line,boost::is_any_of((" ")));
          pcl::PointXYZ point;
          point.x=boost::lexical_cast<float>(vertexData[1]);
          point.y=boost::lexical_cast<float>(vertexData[2]);
          point.z=boost::lexical_cast<float>(vertexData[3]);
          if(!hasPoint(cloudData,point))
            cloudData->points.push_back(point);
        }
      }
    }
    stream.close();
    pcl::io::savePCDFile(pcdFileName, *cloudData);
    return true;
  }
*/
}
