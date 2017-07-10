#include <math.h>
#include "distance_measurer.h"
#include <iostream>

#define GET_FLOATVEC_COUNT(vec)(sizeof(vec)/sizeof(float))
namespace pmr
{
  DistanceMeasurer::DistanceMeasurer()
  {}

  DistanceMeasurer::~DistanceMeasurer()
  {
    if(GET_FLOATVEC_COUNT(triangles_)>0)
      free(triangles_);
  }

void
  DistanceMeasurer::setNumTriangles (int num_triangles)
  {
    num_triangles_ = num_triangles;
  }

  void
  DistanceMeasurer::setTriangles (const std::vector<std::vector<Eigen::Vector3f> > & triangles)
  {
    int total_size = triangles.size() * 9;
    //if(GET_FLOATVEC_COUNT(triangles_)>0)
      //free(triangles_);

    triangles_ = (float *)malloc (total_size*sizeof(float));
    for (int i = 0; i < triangles.size(); ++i)
    {
      triangles_[i*9+0] = triangles[i][0][0];
      triangles_[i*9+1] = triangles[i][0][1];
      triangles_[i*9+2] = triangles[i][0][2];
      triangles_[i*9+3] = triangles[i][1][0];
      triangles_[i*9+4] = triangles[i][1][1];
      triangles_[i*9+5] = triangles[i][1][2];
      triangles_[i*9+6] = triangles[i][2][0];
      triangles_[i*9+7] = triangles[i][2][1];
      triangles_[i*9+8] = triangles[i][2][2];
    }
  }

  void
  DistanceMeasurer::setNormals(const std::vector<Eigen::Vector3f> & normals)
  {
    normals_=normals;
  }

  bool isPointNearTriangle(const float* point, const float* triangle, const Eigen::Vector3f normal)
  {
    float threshold=1e-4;
    Eigen::Vector3f vec;
    vec(0)=point[0]-*triangle;
    vec(1)=point[1]-*(triangle++);
    vec(2)=point[2]-*(triangle++);
    if(fabs(vec.dot(normal))>threshold)
      return false;
    return true;
  }

  // Calculate the distance from a point to a triangle mesh.
float
  DistanceMeasurer::distPointTriangle (const float * dev_point, const float * dev_triangles, 
      const std::vector<Eigen::Vector3f> normals)
  {
    //int tid = threadIdx.x + blockDim.x * blockIdx.x;

    //if (tid < dev_num_triangles[0])
    float * dev_distances = (float*)malloc(sizeof(float)*num_triangles_);

    for(int id=0;id < num_triangles_; ++id)
    {
      //if(!isPointNearTriangle(dev_point, &dev_triangles[id*9],normals.at(id)))
        //continue;

      if (isPointInTriangle (dev_point, &dev_triangles[id*9]))
      //if(isPointInTriangle(dev_point, &dev_triangles[id*9], normals.at(id)))
      {
        dev_distances[id] = distPointPlane (dev_point, &dev_triangles[id*9]);
      }
      else
      {
        //dev_distances[id] = distPointPlane (dev_point, &dev_triangles[id*9]);

        const float * dev_triangle = &dev_triangles[id*9];

        // Calculate the distance from the point to the vertices and segments.
        float dev_distance_list[6];

        float dev_vertex_0[3];
        dev_vertex_0[0] = dev_triangle[0];
        dev_vertex_0[1] = dev_triangle[1];
        dev_vertex_0[2] = dev_triangle[2];
        float dev_vertex_1[3];
        dev_vertex_1[0] = dev_triangle[3];
        dev_vertex_1[1] = dev_triangle[4];
        dev_vertex_1[2] = dev_triangle[5];
        float dev_vertex_2[3];
        dev_vertex_2[0] = dev_triangle[6];
        dev_vertex_2[1] = dev_triangle[7];
        dev_vertex_2[2] = dev_triangle[8];

        dev_distance_list[0] = distPointPoint (dev_point, dev_vertex_0);
        dev_distance_list[1] = distPointPoint (dev_point, dev_vertex_1);
        dev_distance_list[2] = distPointPoint (dev_point, dev_vertex_2);

        float dev_segment_vertices[6];
        dev_segment_vertices[0] = dev_triangle[0];
        dev_segment_vertices[1] = dev_triangle[1];
        dev_segment_vertices[2] = dev_triangle[2];
        dev_segment_vertices[3] = dev_triangle[3];
        dev_segment_vertices[4] = dev_triangle[4];
        dev_segment_vertices[5] = dev_triangle[5];
        dev_distance_list[3] = distPointLineSegment (dev_point, dev_segment_vertices);

        dev_segment_vertices[3] = dev_triangle[6];
        dev_segment_vertices[4] = dev_triangle[7];
        dev_segment_vertices[5] = dev_triangle[8];
        dev_distance_list[4] = distPointLineSegment (dev_point, dev_segment_vertices);

        dev_segment_vertices[0] = dev_triangle[3];
        dev_segment_vertices[1] = dev_triangle[4];
        dev_segment_vertices[2] = dev_triangle[5];
        dev_distance_list[5] = distPointLineSegment (dev_point, dev_segment_vertices);

        float min_distance = dev_distance_list[0];
        for (int i = 0; i < 6; ++i)
          if (min_distance > dev_distance_list[i])
            min_distance = dev_distance_list[i];

        dev_distances[id] = min_distance;
      }
    }

    float min=dev_distances[0];
    for(int i=0;i<num_triangles_;++i)
    {
      //std::cout<<">>> distance: = "<<dev_distances[i]<<std::endl;
      if(min>dev_distances[i])
        min=dev_distances[i];
    }
    free(dev_distances);

    return min;
  }

  float
  DistanceMeasurer::calShortestDistance (const float * point)
  {
    float min_distance = distPointTriangle(point,triangles_,normals_);
    return (min_distance);
  }

  float DistanceMeasurer::calShortestNormalDistance(const float* normal)
  {
    //std::cout<<"calDist..."<<std::endl;
    float min_distance=1000;
    for(int id=0;id<num_triangles_;++id)
    {
      float tri_normal[3];
      tri_normal[0]=(normals_.at(id))(0);
      tri_normal[1]=(normals_.at(id))(1);
      tri_normal[2]=(normals_.at(id))(2);
      //std::cout<<"p1 "<<tri_normal[0]<<" "<<tri_normal[1]<<" "<<tri_normal[2]<<std::endl;
      //std::cout<<"p2 "<<normal[0]<<" "<<normal[1]<<" "<<normal[2]<<std::endl;
      //float dist=distPointPoint(normal,tri_normal);
      float dist=sqrt(pow((tri_normal[0]-normal[0]),2)
        +pow((tri_normal[1]-normal[1]),2)
        +pow((tri_normal[2]-normal[2]),2));

      //std::cout<<"dist "<<dist<<std::endl;
      if(dist<min_distance)
        min_distance=dist;
    }
    //std::cout<<"min_dist "<<min_distance<<std::endl;
    return min_distance;
  }


  bool DistanceMeasurer:: isPointProjectionOnAnyTriangle(const float* point)
   {
      for(int id=0;id<num_triangles_;++id)
      {
        if(isPointInTriangle(point,&triangles_[id*9]))
        {
          return true;
        }
      }

      return false;
    }
  // Calculate the projected point on a plane.
  void
  DistanceMeasurer::pointProjectionOnPlane (const float * dev_point, const float * dev_triangle_vertices, float * dev_point_projection)
  {
    float vect_ab[3];
    vect_ab[0] = dev_triangle_vertices[3] - dev_triangle_vertices[0];
    vect_ab[1] = dev_triangle_vertices[4] - dev_triangle_vertices[1];
    vect_ab[2] = dev_triangle_vertices[5] - dev_triangle_vertices[2];
    float vect_ac[3];
    vect_ac[0] = dev_triangle_vertices[6] - dev_triangle_vertices[0];
    vect_ac[1] = dev_triangle_vertices[7] - dev_triangle_vertices[1];
    vect_ac[2] = dev_triangle_vertices[8] - dev_triangle_vertices[2];

    float normal[3];
    normal[0] = -vect_ab[2]*vect_ac[1] + vect_ab[1]*vect_ac[2];
    normal[1] = vect_ab[2]*vect_ac[0] - vect_ab[0]*vect_ac[2];
    normal[2] = -vect_ab[1]*vect_ac[0] + vect_ab[0]*vect_ac[1];
    //float normal_norm = norm3df (normal[0], normal[1], normal[2]);
    float normal_norm=sqrt(pow(normal[0],2)+pow(normal[1],2)+pow(normal[2],2));
    normal[0] /= normal_norm;
    normal[1] /= normal_norm;
    normal[2] /= normal_norm;

    float vect_ap[3];
    vect_ap[0] = dev_point[0] - dev_triangle_vertices[0];
    vect_ap[1] = dev_point[1] - dev_triangle_vertices[1];
    vect_ap[2] = dev_point[2] - dev_triangle_vertices[2];

    float dot_ap_normal = vect_ap[0]*normal[0] + vect_ap[1]*normal[1] + vect_ap[2]*normal[2];

    dev_point_projection[0] = vect_ap[0] - dot_ap_normal*normal[0];
    dev_point_projection[1] = vect_ap[1] - dot_ap_normal*normal[1];
    dev_point_projection[2] = vect_ap[2] - dot_ap_normal*normal[2];
  }

  // Detect if the projection of a point is inside the triangle.
  bool
  DistanceMeasurer::isPointInTriangle (const float * dev_point, const float * dev_triangle_vertices)
  {
    float vect_ab[3];
    vect_ab[0] = dev_triangle_vertices[3] - dev_triangle_vertices[0];
    vect_ab[1] = dev_triangle_vertices[4] - dev_triangle_vertices[1];
    vect_ab[2] = dev_triangle_vertices[5] - dev_triangle_vertices[2];
    float vect_ac[3];
    vect_ac[0] = dev_triangle_vertices[6] - dev_triangle_vertices[0];
    vect_ac[1] = dev_triangle_vertices[7] - dev_triangle_vertices[1];
    vect_ac[2] = dev_triangle_vertices[8] - dev_triangle_vertices[2];
    float vect_ap[3];
    vect_ap[0] = dev_point[0] - dev_triangle_vertices[0];
    vect_ap[1] = dev_point[1] - dev_triangle_vertices[1];
    vect_ap[2] = dev_point[2] - dev_triangle_vertices[2];

    float dot_ab_ab = vect_ab[0]*vect_ab[0] + vect_ab[1]*vect_ab[1] + vect_ab[2]*vect_ab[2];
    float dot_ac_ac = vect_ac[0]*vect_ac[0] + vect_ac[1]*vect_ac[1] + vect_ac[2]*vect_ac[2];
    float dot_ab_ac = vect_ab[0]*vect_ac[0] + vect_ab[1]*vect_ac[1] + vect_ab[2]*vect_ac[2];
    float dot_ap_ab = vect_ap[0]*vect_ab[0] + vect_ap[1]*vect_ab[1] + vect_ap[2]*vect_ab[2];
    float dot_ap_ac = vect_ap[0]*vect_ac[0] + vect_ap[1]*vect_ac[1] + vect_ap[2]*vect_ac[2];

    float u = (dot_ac_ac*dot_ap_ab - dot_ab_ac*dot_ap_ac) / (dot_ab_ab*dot_ac_ac - dot_ab_ac*dot_ab_ac);
    float v = (dot_ab_ab*dot_ap_ac - dot_ab_ac*dot_ap_ab) / (dot_ab_ab*dot_ac_ac - dot_ab_ac*dot_ab_ac);

    return ((u >= 0.0) && (v >= 0.0) && (u + v <= 1.0));
  }

  bool
  DistanceMeasurer::isPointInTriangle(const float *point, const float *triangle_vertices, const Eigen::Vector3f normal)
  {
    Eigen::Vector3f vec;
    vec[0]=point[0]-triangle_vertices[0];
    vec[1]=point[1]-triangle_vertices[1];
    vec[2]=point[2]-triangle_vertices[2];
    float dotValue=vec.dot(normal);
    if(dotValue<0)
      return false;
    return true;
  }

  float norm3df(float x,float y,float z)
  {
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  }
  // Calculate the distance from a point to another point.
  float
  DistanceMeasurer::distPointPoint (const float * dev_point_a, const float * dev_point_b)
  {
    return (norm3df (dev_point_b[0]-dev_point_a[0], dev_point_b[1]-dev_point_a[1], dev_point_b[2]-dev_point_a[2]));
  }

  // Calculate the distance from a point to a line segment.
  float
  DistanceMeasurer::distPointLineSegment (const float * dev_point, const float * dev_segment_vertices)
  {
    float vect_v[3];
    vect_v[0] = dev_segment_vertices[3] - dev_segment_vertices[0];
    vect_v[1] = dev_segment_vertices[4] - dev_segment_vertices[1];
    vect_v[2] = dev_segment_vertices[5] - dev_segment_vertices[2];

    float vect_w[3];
    vect_w[0] = dev_point[0] - dev_segment_vertices[0];
    vect_w[1] = dev_point[1] - dev_segment_vertices[1];
    vect_w[2] = dev_point[2] - dev_segment_vertices[2];

    float scalar_1 = vect_v[0]*vect_w[0] + vect_v[1]*vect_w[1] + vect_v[2]*vect_w[2];
    float scalar_2 = vect_v[0]*vect_v[0] + vect_v[1]*vect_v[1] + vect_v[2]*vect_v[2];

    if (scalar_1 <= 0.0)
    {
        // Projected point on the line is on the left of segmentVertices[0].
        return (distPointPoint (dev_point, &dev_segment_vertices[0]));
    }
    else if (scalar_1 >= scalar_2)
    {
        // Projected point on the line is on the right of segmentVertices[1].
        return (distPointPoint (dev_point, &dev_segment_vertices[3]));
    }
    else
    {
        // Projected point on the line is on the line segment.
      float point_projection[3];
      point_projection[0] = dev_segment_vertices[0] + scalar_1/scalar_2*vect_v[0];
      point_projection[1] = dev_segment_vertices[1] + scalar_1/scalar_2*vect_v[1];
      point_projection[2] = dev_segment_vertices[2] + scalar_1/scalar_2*vect_v[2];

      return (distPointPoint (dev_point, point_projection));
    }

  }

  // Calculate the distance from a point to a plane.
  float
  DistanceMeasurer::distPointPlane (const float * dev_point, const float * dev_triangle_vertices)
  {
    float vect_ab[3];
    vect_ab[0] = dev_triangle_vertices[3] - dev_triangle_vertices[0];
    vect_ab[1] = dev_triangle_vertices[4] - dev_triangle_vertices[1];
    vect_ab[2] = dev_triangle_vertices[5] - dev_triangle_vertices[2];
    float vect_ac[3];
    vect_ac[0] = dev_triangle_vertices[6] - dev_triangle_vertices[0];
    vect_ac[1] = dev_triangle_vertices[7] - dev_triangle_vertices[1];
    vect_ac[2] = dev_triangle_vertices[8] - dev_triangle_vertices[2];

    float normal[3];
    normal[0] = -vect_ab[2]*vect_ac[1] + vect_ab[1]*vect_ac[2];
    normal[1] = vect_ab[2]*vect_ac[0] - vect_ab[0]*vect_ac[2];
    normal[2] = -vect_ab[1]*vect_ac[0] + vect_ab[0]*vect_ac[1];
    float normal_norm = norm3df (normal[0], normal[1], normal[2]);
    normal[0] /= normal_norm;
    normal[1] /= normal_norm;
    normal[2] /= normal_norm;

    float vect_ap[3];
    vect_ap[0] = dev_point[0] - dev_triangle_vertices[0];
    vect_ap[1] = dev_point[1] - dev_triangle_vertices[1];
    vect_ap[2] = dev_point[2] - dev_triangle_vertices[2];

    float dot_ap_normal = vect_ap[0]*normal[0] + vect_ap[1]*normal[1] + vect_ap[2]*normal[2];

    if (dot_ap_normal < 0.0)
        dot_ap_normal = -dot_ap_normal;

    return (fabsf (dot_ap_normal));
  }

}
