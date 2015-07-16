#include "vehicle.h"
Vehicle::Vehicle(
	           World* world,
               V2D position,
               double    rotation,
               V2D velocity,
               double    mass,
               double    max_force,
               double    max_speed,
               double    max_turn_rate,
               double    scale):   
                 //1)位置,2)半徑,3)速度,4)最大速度,5)方向,6)質量,7)縮放,8)轉動率,9)最大施力   
								   MovingObject(position,
                                                   scale,
                                                   velocity,
                                                   max_speed,
                                                  // V2D(sin(rotation),-cos(rotation)),
                                                   V2D(cos(rotation),sin(rotation)),
                                                   mass,
                                                   V2D(scale,scale),
                                                   max_turn_rate,
                                                   max_force)
                                                   ,
                                                   m_pWorld(world),
                                                   m_vSmoothedHeading(V2D(0,0)),
                                                   //m_vSmoothedHeading(Vector2D(0,0)),
                                                   //m_bSmoothingOn(false),
                                                   m_dTimeElapsed(0.0)

{  
  //建立三角外形到m_vecVehicleShape
  const int NumVehicleVerts = 3;
  V2D vehicle[NumVehicleVerts] = {V2D(-1.0f,0.6f),
                                  V2D(1.0f,0.0f),
                                  V2D(-1.0f,-0.6f)};
  for (int vtx=0; vtx<NumVehicleVerts; ++vtx)
  {
    m_vecVehicleShape.push_back(vehicle[vtx]);
  }
  //
  //set up the steering behavior class
  //建立施力行為物件
  m_pSteering = new SteeringBehavior(this);    
  //讓轉向平滑化
  m_pHeadingSmoother = new Smoother<V2D>(5, V2D(0.0, 0.0));  
}

Vehicle::~Vehicle()
{
  delete m_pSteering;
  delete m_pHeadingSmoother;
}

void Vehicle::Update(double time_elapsed)
{    
  //update the time elapsed
  //wprintf(L"%f\n",time_elapsed);
  m_dTimeElapsed = time_elapsed;
  //存下座標到OldPos
  //OldPos = Pos();
  //定義施力
  V2D SteeringForce;
  SteeringForce = m_pSteering->Calculate();
  //加速度=力/質量
  V2D acceleration = SteeringForce / m_dMass;
  test_force = SteeringForce;
  //更新速度=加速度*時間
  //wprintf(L"V(%lf,%lf)\n",m_vVelocity.x,m_vVelocity.y);
  m_vVelocity += (acceleration * time_elapsed); 
  //當速度不為0時自動把朝向轉到移動方向
  if (m_vVelocity.LengthSq() > 0.00000001)
  {    
    m_vHeading = V2DNormalize(m_vVelocity);
    m_vSide = m_vHeading.Perp();
  }
  //
  m_vVelocity.Truncate(m_dMaxSpeed);
  //更新座標
  if (Pos().x > GetWorld()->cxClient()){
    SetPos(V2D(0,Pos().y));  
  }
  if (Pos().x < 0){
    SetPos(V2D(GetWorld()->cxClient(),Pos().y));  
  }
  if (Pos().y < 0){
    SetPos(V2D(Pos().x,GetWorld()->cyClient()));     
  }
  if (Pos().y > GetWorld()->cyClient()){
    SetPos(V2D(Pos().x,0));     
  }
  SetPos(Pos()+(m_vVelocity * time_elapsed));
  //
  m_vSmoothedHeading = m_pHeadingSmoother->Update(Heading());
}

void Vehicle::Render()
{ 
  //-----------------------------------------------
  //畫外型
  static std::vector<V2D>  m_vecVehicleShapeTrans;
  m_vecVehicleShapeTrans = m_pWorld->WorldTransform(m_vecVehicleShape,
                                         Pos(),
                                         SmoothedHeading(),//Heading(),
                                         SmoothedHeading().Perp(),//Side(),
                                         Scale());
  ngdi->ClosedShape(m_vecVehicleShapeTrans);
  //------------------------------------------------
  /*
  //畫藍線表示速度
  ngdi->BluePen();
  ngdi->Line(Pos(), Pos()+m_vVelocity);
  //畫紅線表示施力  
  ngdi->RedPen();
  ngdi->Line(Pos(), Pos()+(test_force*10));
  //畫Wander
  if(GetSteering()->HasWander()){
      V2D Wandercircle = GetWorld()->PointToWorldSpace(
                                  V2D(GetSteering()->WanderDistance(),0),
                                  Heading(),
                                  Side(), 
                                  Pos());
    ngdi->PurplePen();
    ngdi->HollowBrush();
    ngdi->Circle(Wandercircle, GetSteering()->WanderRadius());
    ngdi->WhitePen();
    ngdi->HollowBrush();
    ngdi->Circle(test_target, 5);
  }
  //畫detect box
  if(GetSteering()->HasObstacleAvoidance()){
     ngdi->BrownPen();
    static std::vector<V2D>  Detect_box_rect;
    static std::vector<V2D>  detect_box_rect;
    detect_box_rect.push_back(V2D(-1,-1));
    detect_box_rect.push_back(V2D(-1,1));
    detect_box_rect.push_back(V2D((int)(GetSteering()->dDBoxLength()/Scale().x),1));
    detect_box_rect.push_back(V2D((int)(GetSteering()->dDBoxLength()/Scale().x),-1));
    Detect_box_rect = m_pWorld->WorldTransform(detect_box_rect,
                                         Pos(),
                                         Heading(),
                                         Side(),
                                         Scale());
    ngdi->ClosedShape(Detect_box_rect);
    detect_box_rect.clear();
  }
  //畫觸角
  if(GetSteering()->HasWallAvoidance()){
    ngdi->YellowPen();
    ngdi->Line(Pos(),GetSteering()->m_Feelers[0]);
    ngdi->Line(Pos(),GetSteering()->m_Feelers[1]);
    ngdi->Line(Pos(),GetSteering()->m_Feelers[2]);
  }
  //畫PATH
  if(GetSteering()->HasFollowPath() && (GetSteering()->GetPath() !=NULL) && (GetSteering()->GetPath()->GetWayPointSize() !=0)){
    GetSteering()->GetPath()->Render();
    ngdi->LightBluePen();
    ngdi->Circle(GetSteering()->GetPath()->CurrentWaypoint(), BRadius());
    ngdi->Circle(GetSteering()->GetPath()->CurrentWaypoint(), BRadius()-5);
  
  }
  */
  //ngdi->DrawDot(test_target,RGB(255,255,255));
  //ngdi->LightPinkPen();
  //ngdi->Circle(Pos(), BRadius());
  //ngdi->Rect(V2D(Pos().x-30,Pos().y-30),V2D(Pos().x+30,Pos().y+30));
}
