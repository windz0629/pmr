#include "stl_visualizer.h"
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

namespace pmr
{
  STLVisualizer::STLVisualizer()
  {
  }

  STLVisualizer::~STLVisualizer()
  {
  }

  void STLVisualizer::setModelFileName(std::string modelFileName)
  {
    _fileName=modelFileName;
  }

  void STLVisualizer::showModel()
  {
    vtkSmartPointer<vtkSTLReader> reader=
      vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(_fileName.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper=
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor=
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer=
      vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(0,0,0);

    vtkSmartPointer<vtkRenderWindow> renderWindow=
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor=
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindow->Render();
    renderWindowInteractor->Start();
  }
}
