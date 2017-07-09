#ifndef PMR_FILTER_STL_DOWNSAMPLER_H
#define PMR_FILTER_STL_DOWNSAMPLER_H
#include "common/model_stl.h"
namespace pmr
{
  class STLDownSampler
  {
    public:
      STLDownSampler();
      STLModel::Ptr filter();
      void setModel(STLModel::Ptr & model);
      void setFilterSize(double filterSize); 
  };
}
#endif
