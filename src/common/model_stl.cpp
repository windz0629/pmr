#include <cmath>
#include "model_stl.h"
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace pmr
{
  STLModel::STLModel(): triangles(std::vector<std::vector<Eigen::Vector3f> >())//,
  //normals(std::vector<float>
  {
  }

  STLModel::~STLModel ()
  { 
  }

  int STLModel::getTriangleSize()const
  {
    return triangles.size();
  }

  std::vector<std::vector<Eigen::Vector3f> > 
    STLModel::getAllTriangles() const
  {
    return triangles;
  }

  std::vector<Eigen::Vector3f> 
  STLModel::getNormals() const
  {
    return normals;
  }

} 
