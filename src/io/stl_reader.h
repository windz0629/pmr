#ifndef PMR_STL_READER_H_
#define PMR_STL_READER_H

#include "common/model_stl.h"
namespace pmr
{
  class STLReader
  {
    public:
     static int loadStlModel(std::string fileName, STLModel::Ptr & model);
  };
}
#endif
