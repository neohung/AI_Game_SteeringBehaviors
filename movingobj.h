//------------------------------------------------------------------------
//
//  檔名: movingobj.h
//
//  描述: 移動物件MovingObject繼承自baseclass,增加了
//             public: 
//                     物件質量m_dMass
//                     最大速度m_dMaxSpeed
//                     最大施力m_dMaxForce
//                     縮放倍數m_vScale
//                     物件方向m_vHeading,m_vSide             
//             private:
//                     識別編號m_ID   
//      使用MovingObject(V2D position,double radius,V2D velocity,
//                       double max_speed, V2D heading, double mass,
//                        V2D scale,double turn_rate, double max_force)初始化
//        須實作Update(), Render()
//  作者: Neo 2015 (iamhahar@gmail.com)
//
//------------------------------------------------------------------------

#ifndef MOVINGOBJ_H
#define MOVINGOBJ_H
#include <cassert>
#include "BaseClass.h"
class MovingObject : public BaseClass
{
protected:
  double m_dMass;
  double m_dMaxSpeed;
  double m_dMaxForce;
  double m_dMaxTurnRate;
  V2D m_vVelocity;
  V2D m_vScale;
  //物體法線方向
  V2D m_vHeading;
  //垂直法線方向
  V2D m_vSide;
//  double m_dMaxForce;
public:
  //Baseclass有m_ObjType, m_vPos, m_dBoundingRadius
  //1)位置,2)半徑,3)速度,4)最大速度,5)方向,6)質量,7)縮放,8)轉動率,9)最大施力
  MovingObject(V2D position,
               double    radius,
               V2D velocity,
               double    max_speed,
               V2D heading,
               double    mass,
               V2D scale,
               double    turn_rate,
               double    max_force):BaseClass(0, position, radius),
                                     m_dMass(mass),
                                     m_dMaxSpeed(max_speed),
                                     m_dMaxForce(max_force),
                                     m_dMaxTurnRate(turn_rate),
                                     m_vVelocity(velocity),
                                     m_vHeading(heading),
                                     m_vSide(m_vHeading.Perp())
                                    // m_vScale(scale)

  {
  	m_vScale = scale;   
  }
  void Update(){};
  virtual ~MovingObject(){};
  double Mass()const{return m_dMass;}
  V2D Velocity()const{return m_vVelocity;}
  void SetVelocity(const V2D& NewVel){m_vVelocity = NewVel;}
  double     Speed()const{return m_vVelocity.Length();}
  double MaxSpeed()const{return m_dMaxSpeed;}                       
  void SetMaxSpeed(double new_speed){m_dMaxSpeed = new_speed;} 
  double MaxForce()const{return m_dMaxForce;}
  void SetMaxForce(double mf){m_dMaxForce = mf;}
  V2D Heading()const{return m_vHeading;}
  //定義m_vHeading的同時算出m_vSide
  void SetHeading(V2D new_heading);
  double MaxTurnRate()const{return m_dMaxTurnRate;}
  void SetMaxTurnRate(double val){m_dMaxTurnRate = val;}
  V2D  Side()const{return m_vSide;}
  //
  V2D Scale()const{return m_vScale;}
  void SetScale(V2D val){SetBRadius(BRadius()* MaxOf(val.x, val.y)/MaxOf(m_vScale.x, m_vScale.y)); m_vScale = val;}
  void SetScale(double val){SetBRadius( BRadius()* (val/MaxOf(m_vScale.x, m_vScale.y))); m_vScale = V2D(val, val);} 
};

inline void MovingObject::SetHeading(V2D new_heading)
{
  //向量長度不能接近1
  assert( (new_heading.LengthSq() - 1.0) < 0.00001);  
  m_vHeading = new_heading;
  m_vSide = m_vHeading.Perp();
}
#endif