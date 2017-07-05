#ifndef PMR_STL_VISUALIZER_H_
#define PMR_STL_VISUALIZER_H_
#include <iostream>
namespace pmr
{
  class STLVisualizer
  {
    public:
      STLVisualizer();
      ~STLVisualizer();
      //void addSTLModel(STLModel::Ptr model);
      //void removeSTLModel(STLModel::Ptr model);
      void setModelFileName(std::string modelFileName);
      void showModel();

    private:
      std::string _fileName;
  };
}
#endif
