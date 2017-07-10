#ifndef PMR_FILTER_STL_DOWNSAMPLER_H
#define PMR_FILTER_STL_DOWNSAMPLER_H
#include "common/model_stl.h"
namespace pmr
{
  class STLDownSampler
  {
    public:
      typedef std::vector<std::vector<Eigen::Vector3f> > TrianglesT;
      typedef std::vector<Eigen::Vector3f> NormalT;

      STLDownSampler();
      STLModel::Ptr filter();
      void setModel(STLModel::Ptr & model);
      void setRemainedTriangleSize(int size);

    private:
      STLModel::Ptr _model;
      int _remainedTriangleSize;
  };
}
#endif
