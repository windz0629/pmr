#include "stl_reader.h"
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace pmr
{
  int STLReader::loadStlModel(std::string fileName, STLModel::Ptr & model)
  {
    std::ifstream stream;
    stream.open(fileName.c_str(),std::ios::in);
    if(!stream.is_open())
    {
      std::cout<<">>> Error: cannot open model file "
        <<fileName<<std::endl;
      return -1;
    }

    int stp=0;
    while(!stream.eof())
    {
      std::string line="";
      std::getline(stream,line);
      boost::trim(line);
      
      if(line.find("facet normal")!=-1)
      {
        boost::trim(line);
        std::vector<std::string> normalData;
        boost::split(normalData,line,boost::is_any_of((" ")),boost::token_compress_on);
        Eigen::Vector3f normal;
        normal[0]=boost::lexical_cast<float>(normalData[2]);
        normal[1]=boost::lexical_cast<float>(normalData[3]);
        normal[2]=boost::lexical_cast<float>(normalData[4]);
        model->normals.push_back(normal);
      }
      else if(line.find("outer loop")!=-1)
      {
        std::vector<Eigen::Vector3f> triangle;
        while(line.find("endloop")==-1)
        {
          std::getline(stream,line);
          if(line.find("vertex")!=-1)
          {
            boost::trim(line);
            std::vector<std::string> vertexData;
            boost::split(vertexData,line,boost::is_any_of((" ")),boost::token_compress_on);
            Eigen::Vector3f point;
            point[0]=boost::lexical_cast<float>(vertexData[1]);
            point[1]=boost::lexical_cast<float>(vertexData[2]);
            point[2]=boost::lexical_cast<float>(vertexData[3]);

            triangle.push_back(point);
          }
        }
        if(triangle.size()>0)
          model->triangles.push_back(triangle);
      }
    }
    return 1;
  }
}
