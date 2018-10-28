#include "visualization/stl_visualizer.h"
#include <iostream>

int main(int argc, char** argv)
{
  pmr::STLVisualizer viz;
  viz.setModelFileName(argv[1]);
  viz.showModel();
}
