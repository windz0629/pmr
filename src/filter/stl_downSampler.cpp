#include "stl_downSampler.h"
namespace pmr
{
  STLDownSampler::STLDownSampler()
  {
  }

  void STLDownSampler::setModel(STLModel::Ptr & model)
  {
    _model=model;
  }

  void STLDownSampler::setRemainedTriangleSize(int size)
  {
    _remainedTriangleSize=size;
  }

  STLModel::Ptr STLDownSampler::filter()
  {
    TrianglesT triangles=_model->getAllTriangles();
    NormalT normals=_model->getNormals();

    int totalSize=_model->getTriangleSize();
    int interval=totalSize/_remainedTriangleSize;
    
    TrianglesT new_triangles;
    NormalT new_normals;
    for(int i=0;i<_remainedTriangleSize;++i)
    {
      int index=i*interval+rand()%interval;
      new_triangles.push_back(triangles.at(index));
      new_normals.push_back(normals.at(index));
    }

    STLModel::Ptr new_model(new STLModel);
    new_model->triangles=new_triangles;
    new_model->normals=new_normals;
    
    return new_model;
  }
}
