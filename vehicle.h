//------------------------------------------------------------------------
//
//  檔名: vehicle.h
//
//  描述: 繼承自移動物件MovingObject
//             public: 
//                     物件質量m_dMass
//                     最大速度m_dMaxSpeed
//                     最大施力m_dMaxForce
//                     縮放倍數m_vScale
//             
//             private:
//                     識別編號m_ID   
//      使用BaseClass(int obj_type, V2D pos, double r)初始化
//        須實作Update(), Render()
//  作者: Neo 2015 (iamhahar@gmail.com)
//
//--------
#ifndef VEHICLE_H
#define VEHICLE_H

#include "world.h"
class World;
#include "MovingObj.h"
#include "neogdi.h"
#include "SteeringBehaviors.h"
#include "Smoother.h"
class SteeringBehavior;
class Vehicle : public MovingObject
{
private:
	//存放Vehicle外型的頂點座標
	std::vector<V2D> m_vecVehicleShape;
	//指向World,讓Vehicle可以存取到World內部的data
	World* m_pWorld;
	//施力行為物件
	SteeringBehavior* m_pSteering;
  //m_pHeadingSmoother讓物件轉向更平滑化
  Smoother<V2D>* m_pHeadingSmoother;
  //最後算出的Heading
  V2D m_vSmoothedHeading;
	//得知最近update的時間
	double m_dTimeElapsed;
	Vehicle(const Vehicle&);
    Vehicle& operator=(const Vehicle&);
public:
	Vehicle(
		     World* world,
         V2D position,
         double    rotation,
         V2D velocity,
         double    mass,
         double    max_force,
         double    max_speed,
         double    max_turn_rate,
         double    scale);
  ~Vehicle();
  void        Update(double time_elapsed);
  void        Render();
  World* GetWorld()const{return m_pWorld;}
  SteeringBehavior* GetSteering()const{return m_pSteering;}
  double      TimeElapsed()const{return m_dTimeElapsed;}
  V2D    SmoothedHeading()const{return m_vSmoothedHeading;}

  //
  V2D test_force;
  V2D test_target;
  double test_detect_box_length;
  //
  //
};

#endif