#include "pose.h"
#include "transform.h"
#include <boost/random.hpp>
namespace pmr
{
  namespace math
  {
    Pose::Pose()
    {
      this->x=0;
      this->y=0;
      this->z=0;
      this->phi=0;
      this->theta=0;
      this->psi=0;
    }

    Pose::Pose(const Pose & p)
    {
      this->x=p.x;
      this->y=p.y;
      this->z=p.z;
      this->phi=p.phi;
      this->theta=p.theta;
      this->psi=p.psi;

      this->_position(0)=p.x;
      this->_position(1)=p.y;
      this->_position(2)=p.z;
      this->_orientation(0)=p.phi;
      this->_orientation(1)=p.theta;
      this->_orientation(2)=p.psi;
    }

    Pose::Pose(float x, float y, float z, 
        float phi, float theta, float psi)
    {
      this->x=x;
      this->y=y;
      this->z=z;
      this->phi=phi;
      this->theta=theta;
      this->psi=psi;
      
      this->_position(0)=x;
      this->_position(1)=y;
      this->_position(2)=z;
      this->_orientation(0)=phi;
      this->_orientation(1)=theta;
      this->_orientation(2)=psi;
    }

    Pose::Pose(Eigen::Vector3f position,Eigen::Vector3f orientation)
    {
      this->x=position(0);
      this->y=position(1);
      this->z=position(2);
      this->phi=orientation(0);
      this->theta=orientation(1);
      this->psi=orientation(2);

      this->_position=position;
      this->_orientation=orientation;
    }

    Pose::~Pose()
    {
    }

    void Pose:: setPosition(Eigen::Vector3f position)
    {
      this->_position=position;
      this->x=position(0);
      this->y=position(1);
      this->z=position(2);
    }

    void Pose:: setPosition(float x, float y, float z)
    {
      x=x;
      y=y;
      z=z;
      _position(0)=x;
      _position(1)=y;
      _position(2)=z;
    }

    void Pose:: setOrientation(float phi, float theta, float psi)
    {
      phi=phi;
      theta=theta;
      psi=psi;
      _orientation(0)=phi;
      _orientation(1)=theta;
      _orientation(2)=psi;
    }

    void Pose::setOrientation(Eigen::Vector3f orientation)
    {
      _orientation=orientation;
      phi=orientation(0);
      theta=orientation(1);
      psi=orientation(2);
    }

    void Pose:: setTransform(Eigen::Matrix4f transformMat)
    {
      Eigen::Vector3f position;
      Eigen::Vector3f euler;
      transform::computePose(transformMat,euler,position);
      this->setOrientation(euler);
      this->setPosition(position);
    }
    
    Eigen::Vector3f & Pose::getPosition()
    {
      _position(0)=x;
      _position(1)=y;
      _position(2)=z;
      return _position;
    }

    Eigen::Vector3f & Pose:: getOrientation()
    {
      _orientation(0)=phi;
      _orientation(1)=theta;
      _orientation(2)=psi;
      return _orientation;
    }


    /**
     * @brief 理论上应该是两个变换矩阵相乘
     * @brief 暂时先采用直接相加方式
     */
    Pose Pose::operator+(const Pose& p)
    {
      Pose res;
      res.x=x+p.x;
      res.y=y+p.y;
      res.z=z+p.z;
      res.phi=phi+p.phi;
      res.theta=theta+p.theta;
      res.psi=psi+p.psi;
      
      return res;
    }

    /**
     * @brief 重载减法计算
     * @brief 理论上应当通过变换矩阵计算
     */
    Pose Pose::operator-(const Pose& p)
    {
      Pose res;
      res.x=x-p.x;
      res.y=y-p.y;
      res.z=z-p.z;
      res.phi=phi-p.phi;
      res.theta=theta-p.theta;
      res.psi=psi-p.psi;
      
      return res;
    }

    
    /**
     * @brief 重载乘法运算符，右乘实数
     */
    /*Pose Pose::operator*(const double & d)
    {
      x*=d;
      y*=d;
      z*=d;
      phi*=d;
      theta*=d;
      psi*=d;

      return *this;
    }
    */

    /**
     * @brief 重载乘法运算法，左乘实数
     */
    Pose operator*(const double d, Pose pose)
    {
      pose.x*=d;
      pose.y*=d;
      pose.z*=d;
      pose.phi*=d;
      pose.theta*=d;
      pose.psi*=d;

      return pose;
    }

    std::ostream & operator<<(std::ostream & out, Pose & p)
    {
      out<<std::endl<<"position:  "<<p.x<<"  "<<p.y<<"  "<<p.z<<"  euler: "
        <<p.phi<<"  "<<p.theta<<"  "<<p.psi<<std::endl;

      return out;
    }
    
    Eigen::Matrix4f Pose::getTransform()
    {
      Eigen::Vector3f euler;
      euler(0)=phi;
      euler(1)=theta;
      euler(2)=psi;
      Eigen::Vector3f pos;
      pos(0)=x;
      pos(1)=y;
      pos(2)=z;
      return transform::computeTransform(euler,pos);
    }
    

  }
}
