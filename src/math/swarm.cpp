#include "swarm.h"
#include <cmath>
namespace pmr
{
  namespace math
  {
    Swarm::Swarm()
    {
      //default value
      fitnessScore=0;
      _minBound=Pose(-0.1,-0.1,-0.1,-M_PI/10,-M_PI/10,-M_PI/10);
      _maxBound=Pose(0.1,0.1,0.1,M_PI/10,M_PI/10,M_PI/10);
      _minVelocity=Pose(-0.01,-0.01,-0.01,-M_PI/20,-M_PI/20,-M_PI/20);
      _maxVelocity=Pose(0.01,0.01,0.01,M_PI/20,M_PI/20,M_PI/20);
      population_size=10;
      max_iteration=100;
      c1=2;
      c2=2;

      gbest_value=DBL_MAX;
    }

    void Swarm::initialize()
    {
      particles.clear();
      for(int i=0;i<population_size;++i)
      {
        Particle par;
        par.setMinMaxBound(_minBound,_maxBound);
        par.setMinMaxVelocity(_minVelocity,_maxVelocity);
        par.randomize();
        particles.push_back(par);
        //std::cout<<"par "<<i<<std::endl<<par.velocity<<par.pose;
      }

      gbest_pose=particles.at(0).pbest_pose;

      rotNormPara=sqrt(pow((_maxBound.phi-_minBound.phi),2)+
          pow((_maxBound.theta-_minBound.theta),2)+
          pow((_maxBound.psi-_minBound.psi),2));
      transNormPara=sqrt(pow((_maxBound.x-_minBound.x),2)+
          pow((_maxBound.y-_minBound.y),2)+
          pow((_maxBound.z-_minBound.z),2));

    }

    void Swarm::setEvaluateFitnessFunc(EvaluateFitnessFunc* funcPtr)
    {
      _fitnessfunc=funcPtr;
    }

    void Swarm::setPopulationSize(int size)
    {
      population_size=size;
    }

    int Swarm::getPopulationSize()
    {
      return population_size;
    }

    /**
     * 开始进行粒子群计算
     */
    PSOSolution Swarm::compute()
    {
      initialize();//初始化粒子
      for(int i=0;i<max_iteration;++i)
      {
        for(int id=0;id<population_size;++id)
        {
          updateVelocity(id,i);
          updatePosition(id);
          evaluateFitnessScore(id);
        }
        std::cout<<"stp "<<i
          <<"   gbestValue: "<<gbest_value
          <<std::endl<<std::endl;
      }
      
      PSOSolution sol(gbest_pose,gbest_value);
      return sol;
    }

    void Swarm::updateVelocity(int particle_id,int iteration)
    {
      Particle& par=particles.at(particle_id);
      
      double r1=rand()%100*0.01;
      double r2=rand()%100*0.01;
      double weight=0.729;//*(1.0-1.0/(float)iteration);
      Pose tmp=weight*par.velocity+c1*r1*(par.pbest_pose-par.pose)
        +c2*r2*(gbest_pose-par.pose);
      par.velocity=assignValue(tmp,limit_v);
      //par.velocity=tmp;
      //std::cout<<"par "<<particle_id<<" velocity: "<<par.velocity;
    }

    void Swarm::updatePosition(int particle_id)
    {
      Particle& par=particles.at(particle_id);
      Pose tmp;
      tmp=par.pose+par.velocity;
      par.pose=assignValue(tmp,limit_p);
      //par.pose.phi=0;
      //par.pose.theta=0;
      //par.pose.psi=0;
      //par.pose=tmp;
      //std::cout<<"par "<<particle_id<<" pose: "<<par.pose;
    }

    /**
     * 计算姿态距离并归一化处理
     * 归一化是通过最大最小边界计算的，可能会导致数量级与点距离不一致
     */
    double Swarm::calPoseDistance(Pose & p1, Pose & p2)
    {
      double dist=0;
      dist=pow((p1.phi-p2.phi),2)+pow((p1.theta-p2.theta),2)+pow((p1.psi-p2.psi),2);
      dist=sqrt(dist);
      dist/=rotNormPara;
      return dist;
    }

    /**
     * 计算笛卡尔空间两点的距离并归一化处理
     * 归一化是通过最大最小边界计算的，可能会导致数量级与姿态距离不匹配
     */
    double Swarm::calPointDistance(Pose& p1, Pose& p2)
    {
      double dist=0;
      dist=pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)+pow((p1.z-p2.z),2);
      dist=sqrt(dist);
      dist/=transNormPara;
      return dist;
    }

    /**
     * 计算适应度
     * 适应度是点距离与姿态距离的叠加，但可能会因为数量级不一致
     * 导致搜索方向开始只偏重与距离或姿态
     */
    void Swarm::evaluateFitnessScore(int particle_id)
    {
      Particle & par=particles.at(particle_id);
      //std::cout<<"par.pose: "<<par.pose;
      fitnessScore=_fitnessfunc->evaluateFitness(par.pose);

      if(fitnessScore<par.pbest_value)
      {
        par.pbest_value=fitnessScore;
        par.pbest_pose=par.pose;
      }

      if(fitnessScore<gbest_value)
      {
        //std::cout<<">>> find better position: "<<std::endl
          //<<"score: "<<fitnessScore<<par.pose<<std::endl;
        gbest_value=fitnessScore;
        gbest_pose=par.pose;
      }

      //std::cout<<"fitnessScore: "<<fitnessScore<<std::endl;
      
    }

    void Swarm::setSearchSpaceBound(Pose & min, Pose & max)
    {
      _minBound=min;
      _maxBound=max;
      limit_p=max;
    }

    void Swarm::setSearchSpeedBound(Pose & min, Pose & max)
    {
      _minVelocity=min;
      _maxVelocity=max;
      limit_v=max;
    }


    void Swarm::setMaxIteration(int maxIter)
    {
      max_iteration=maxIter;
    }

    Pose Swarm::assignValue(Pose& value, const Pose& bound)
    {
      if(value.x>bound.x)
        value.x=bound.x;
      else if(value.x<-bound.x)
        value.x=-bound.x;
      if(value.y>bound.y)
        value.y=-bound.y;
      else if(value.y<-bound.y)
        value.y=-bound.y;
      if(value.z>bound.z)
        value.z=bound.z;
      else if(value.z<-bound.z)
        value.z=-bound.z;
      if(value.phi>bound.phi)
        value.phi=bound.phi;
      else if(value.phi<-bound.phi)
        value.phi=-bound.phi;
      if(value.theta>bound.theta)
        value.theta=bound.theta;
      else if(value.theta<-bound.theta)
        value.theta=-bound.theta;
      if(value.psi>bound.psi)
        value.psi=bound.psi;
      else if(value.psi<-bound.psi)
        value.psi=-bound.psi;

      return value;
    }
  }
}
