#ifndef __PMR_DISTANCE_MEASURE_H_
#define __PMR_DISTANCE_MEASURE_H_

#include <vector>
#include <Eigen/Dense>

namespace pmr
{
  class DistanceMeasurer
  {
    public:
      DistanceMeasurer();
      ~DistanceMeasurer();

      void setNumTriangles(int num_triangles);
      void setTriangles(const std::vector<std::vector<Eigen::Vector3f> > & triangles);
      void setNormals(const std::vector<Eigen::Vector3f> & normals);
      float calShortestDistance(const float* point);
      float calShortestNormalDistance(const float* point);
      bool isPointProjectionOnAnyTriangle(const float* point);

    private:
      int num_triangles_;
      //int * dev_num_triangles_;
      float* triangles_;
      std::vector<Eigen::Vector3f> normals_;

      float distPointTriangle (const float * dev_point, const float * dev_triangles,std::vector<Eigen::Vector3f> normals);
      void pointProjectionOnPlane (const float * dev_point, const float * dev_triangle_vertices, float * dev_point_projection);
      bool isPointInTriangle (const float * dev_point, const float * dev_triangle_vertices);
      bool isPointInTriangle(const float *dev_point, const float* dev_triangle_vertices, const Eigen::Vector3f normal);
      float distPointPoint (const float * dev_point_a, const float * dev_point_b);
      float distPointLineSegment (const float * dev_point, const float * dev_segment_vertices);
      float distPointPlane (const float * dev_point, const float * dev_triangle_vertices);
  };
}
#endif
