#include "SteeringBehaviors.h"
#include <stdlib.h>
SteeringBehavior::SteeringBehavior(Vehicle* agent): 
                            m_iFlags(0),
												    m_pVehicle(agent),
												    m_dWeightWander(1.0),
													  m_dWanderRadius(WanderRad),
													  m_dWanderJitter(WanderJitterPerSec),
													  m_dWanderDistance(WanderDist),
                            m_dWeightSeek(1.0),
                            m_dWeightFlee(1.0),
                            m_dWeightArrive(1.0),
                            m_dWeightPursuit(1.0),
                            m_dWeightEvade(1.0),
                            m_dWeightObstacleAvoidance(1.0),
                            m_dWeightWallAvoidance(1.0),
                            m_dWallDetectionFeelerLength(40),
                            m_dWeightInterpose(1.0),
                            m_dWeightHide(1.0),
                            m_dWeightFollowPath(1.0),
                            m_dWeightOffsetPursuit(1.0),
                            m_dWeightSeparation(5.0),
                            m_dWeightAlignment(1.0),
                            m_dWeightCohesion(100.0),
                            m_dViewDistance(100)
            

{
 // double theta = RandFloat() * TwoPi;
  //create a vector to a target position on the wander circle
 // m_vWanderTarget = V2D(m_dWanderRadius * cos(theta),
 //                       m_dWanderRadius * sin(theta));

 //create a Path
  //m_pPath = new Path(10, 0, 0, 500, 500, true);
  m_pPath = new Path();
  m_pPath->LoopOn();
}

SteeringBehavior::~SteeringBehavior()
{
 //delete m_pPath;
}

V2D SteeringBehavior::Calculate()
{
	//歸零
	m_vSteeringForce.Zero();
	//
  if (On(separation) || On(allignment) || On(cohesion))
  {
    m_pVehicle->GetWorld()->CellSpace()->CalculateNeighbors(m_pVehicle->Pos(), m_dViewDistance);
  }
	m_SummingMethod = prioritized;
	switch (m_SummingMethod)
    { 
	  case weighted_average:
        m_vSteeringForce = CalculateWeightedSum(); break;
      case prioritized:
        m_vSteeringForce = CalculatePrioritized(); break;
      case dithered:
        m_vSteeringForce = CalculateDithered();break;
      default:m_vSteeringForce = V2D(0,0); 
    }
	return m_vSteeringForce;
}

V2D SteeringBehavior::CalculateWeightedSum()
{ 
  if (On(wander)){
    m_vSteeringForce += Wander() * m_dWeightWander;
  }  
  if (On(seek)){
    m_vSteeringForce += Seek(m_pVehicle->GetWorld()->Crosshair()) * m_dWeightSeek;
  }
  if (On(flee)){
    m_vSteeringForce += Flee(m_pVehicle->GetWorld()->Crosshair()) * m_dWeightFlee;
  }
 if (On(arrive)){
    m_vSteeringForce += Arrive(m_pVehicle->GetWorld()->Crosshair()) * m_dWeightArrive;
  }
  if (On(pursuit))
  {
    assert(m_pTargetAgent1 && "pursuit target not assigned");
    m_vSteeringForce += Pursuit(m_pTargetAgent1) * m_dWeightPursuit;
  }
  if (On(evade))
  {
    assert(m_pTargetAgent1 && "evade target not assigned");
    m_vSteeringForce += Evade(m_pTargetAgent1) * m_dWeightEvade;
  }
  if (On(obstacle_avoidance))
  {
    m_vSteeringForce += ObstacleAvoidance(m_pVehicle->GetWorld()->Obstacles()) * m_dWeightObstacleAvoidance;
  }
  if (On(wall_avoidance))
  {
    m_vSteeringForce += WallAvoidance(m_pVehicle->GetWorld()->Walls()) * m_dWeightWallAvoidance;
  }
  if (On(interpose))
  {
    assert(m_pTargetAgent1 && m_pTargetAgent2 && "interpose targets not assigned");
    m_vSteeringForce += Interpose(m_pTargetAgent1, m_pTargetAgent2) * m_dWeightInterpose;
  }
  if (On(hide))
  {
    assert(m_pTargetAgent1 && "hide target not assigned");
    m_vSteeringForce += Hide(m_pTargetAgent1,m_pVehicle->GetWorld()->Obstacles()) * m_dWeightHide;
  }
  if (On(follow_path))
  {
    assert(m_pPath && "m_pPath not assigned");
    m_vSteeringForce += FollowPath() * m_dWeightFollowPath;
  }
  if (On(offset_pursuit))
  {
    assert(m_pTargetAgent1 && "offset_pursuit target not assigned");
    m_vSteeringForce += OffsetPursuit(m_pTargetAgent1, m_vOffset) * m_dWeightOffsetPursuit;
  }
  if (On(separation))
  {
      m_vSteeringForce += Separation(m_pVehicle->GetWorld()->Agents()) * m_dWeightSeparation;
  }
  if (On(allignment))
  {
      m_vSteeringForce += Alignment(m_pVehicle->GetWorld()->Agents()) * m_dWeightAlignment;
  }
  if (On(cohesion))
  {
      m_vSteeringForce += Cohesion(m_pVehicle->GetWorld()->Agents()) * m_dWeightCohesion;
  }

  m_vSteeringForce.Truncate(m_pVehicle->MaxForce());
  return m_vSteeringForce;     
}

bool SteeringBehavior::AccumulateForce(V2D &RunningTot,
                                       V2D ForceToAdd)
{ 
  double MagnitudeSoFar = RunningTot.Length();
  double MagnitudeRemaining = m_pVehicle->MaxForce() - MagnitudeSoFar;
  if (MagnitudeRemaining <= 0.0) return false;
  double MagnitudeToAdd = ForceToAdd.Length();
  if (MagnitudeToAdd < MagnitudeRemaining)
  {
    //如果ForceToAdd加進去還夠的話直接加進去
    RunningTot += ForceToAdd;
  }
  else
  {
    //如果ForceToAdd加進去爆表的話縮減成剩餘的大小再加入
    RunningTot += (V2DNormalize(ForceToAdd) * MagnitudeRemaining); 
  }
  return true;
}


V2D SteeringBehavior::CalculatePrioritized()
{  
  V2D force;
  if (On(wall_avoidance))
  {
    force = WallAvoidance(m_pVehicle->GetWorld()->Walls()) * m_dWeightWallAvoidance;
    //如果m_vSteeringForce已經超過MaxForce則直接返回 不再往下計算
    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  } 
  if (On(obstacle_avoidance))
  {
    force = ObstacleAvoidance(m_pVehicle->GetWorld()->Obstacles()) * m_dWeightObstacleAvoidance;
    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
  //
    if (On(separation))
    {  
      force = SeparationPlus()* m_dWeightSeparation;
      //force = Separation(m_pVehicle->GetWorld()->Agents()) * m_dWeightSeparation;
      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }
    if (On(allignment))
    {
      force = AlignmentPlus()* m_dWeightAlignment;
      //force = Alignment(m_pVehicle->GetWorld()->Agents()) * m_dWeightAlignment;
      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }
    if (On(cohesion))
    { 
      force = CohesionPlus()* m_dWeightCohesion;
      //force = Cohesion(m_pVehicle->GetWorld()->Agents()) * m_dWeightCohesion;
      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

  //
  if (On(arrive)){
    force = Arrive(m_pVehicle->GetWorld()->Crosshair()) * m_dWeightArrive;
    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
  //
  if (On(wander))
  {
    force = Wander() * m_dWeightWander;
    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  return m_vSteeringForce;   
}

V2D SteeringBehavior::CalculateDithered()
{   
V2D a;
return a;     
}

V2D SteeringBehavior::Wander()
{ 
  //每秒隨機位移不超過m_dWanderJitter,所以距上次update經過了TimeElapsed秒
  double JitterThisTimeSlice = m_dWanderJitter * m_pVehicle->TimeElapsed();

  //m_vWanderTarget每次隨機增加位移
  m_vWanderTarget += V2D(RandomClamped() * JitterThisTimeSlice,
                              RandomClamped() * JitterThisTimeSlice);

  //先投影回單位圓上
  m_vWanderTarget.Normalize();
  //再擴大成半徑為m_dWanderRadius的圓
  m_vWanderTarget *= m_dWanderRadius;
  //往物體前方平移m_dWanderDistance的距離(因為一開始物體朝向就是往x軸方向)
  V2D target = m_vWanderTarget + V2D(m_dWanderDistance, 0);
  //投影回m_pVehicle的前方
  V2D Target = m_pVehicle->GetWorld()->PointToWorldSpace(target,
                                       m_pVehicle->Heading(),
                                       m_pVehicle->Side(), 
                                       m_pVehicle->Pos());

  //計算出物體位置往目標去所需的力
  m_pVehicle->test_target = Target;
  return Target - m_pVehicle->Pos();
 }

V2D SteeringBehavior::Seek(V2D TargetPos)
{ 
   V2D DesiredVelocity = V2DNormalize(TargetPos - m_pVehicle->Pos())
                            * m_pVehicle->MaxSpeed();

  return (DesiredVelocity - m_pVehicle->Velocity());
}

V2D SteeringBehavior::Flee(V2D TargetPos)
{
  const double PanicDistanceSq = 100.0f * 100.0;
  if (V2DDistanceSq(m_pVehicle->Pos(), TargetPos) > PanicDistanceSq)
  {
    return V2D(0,0);
  }
  V2D DesiredVelocity = V2DNormalize(m_pVehicle->Pos() - TargetPos) 
                            * m_pVehicle->MaxSpeed();

  return (DesiredVelocity - m_pVehicle->Velocity());
}

V2D SteeringBehavior::Arrive(V2D TargetPos)
{
  //此數越大接近TargetPos時變得越慢
  const double DecelerationTweaker = 0.6;
  V2D ToTarget = TargetPos - m_pVehicle->Pos();
  double dist = ToTarget.Length();
  if (dist > 0){
    double speed =  dist / DecelerationTweaker;     
    speed = MinOf(speed, m_pVehicle->MaxSpeed());
    V2D DesiredVelocity =  V2DNormalize(ToTarget) * speed;
    return (DesiredVelocity - m_pVehicle->Velocity());
  }
  return V2D(0,0);
}
//Pursuit追捕evader

V2D SteeringBehavior::Pursuit(const Vehicle* evader)
{
  //求出ToEvader向量(得到evader在物件的哪個方向)
  V2D ToEvader = evader->Pos() - m_pVehicle->Pos();
  //RelativeTurning表示evader的所在方向和物體的朝向兩者的夾角
  double RelativeTurning = ToEvader.Dot(m_pVehicle->Heading());
  //RelativeHeading表示evader的朝向和物體的朝向兩者的夾角  
  double RelativeHeading = m_pVehicle->Heading().Dot(evader->Heading());
  //(RelativeTurning > 0)表示evader在物體前方
  //(RelativeHeading < -0.95)表示acos(0.95)=18度
  //即表示evader在面向物體的朝向夾角小於18度時用Seek()追即可
  //換言之表示evader在物體前方且evader正面對著物體,直接Seek evader的位置
  if ((RelativeTurning > 0) && (RelativeHeading < -0.95))
  {
    return Seek(evader->Pos());
  }
  //非上述的條件Seek()的target位置須加上前置量,所以要預估追上evader的時間
  //兩者距離"ToEvader.Length()"除以兩者速度即時間
  double LookAheadTime = ToEvader.Length() / 
                        (m_pVehicle->MaxSpeed() + evader->Speed());
  //假設evader的速度及方向不變的情況下乘上LookAheadTime即預計未來evader的位置
  return Seek(evader->Pos() + evader->Velocity() * LookAheadTime);
}

V2D SteeringBehavior::Evade(const Vehicle* pursuer)
{
 V2D ToPursuer = pursuer->Pos() - m_pVehicle->Pos();
 double LookAheadTime = ToPursuer.Length() / 
                        (m_pVehicle->MaxSpeed() + pursuer->Speed());
  //假設pursuer的速度及方向不變的情況下乘上LookAheadTime即預計未來pursuer的位置
  return Flee(pursuer->Pos() + pursuer->Velocity() * LookAheadTime);
}

V2D SteeringBehavior::ObstacleAvoidance(std::vector<BaseClass*>& obstacles)
{
  //定義最接近物體的Obstacle
  BaseClass* ClosestIntersectingObstacle = NULL;
  //定義最接近物體的Obstacle其坐標
  V2D LocalPosOfClosestObstacle;
  //初使化檢查最接近物體的障礙物其與物體相交圓的x值
  double DistToClosestIP = MaxDouble;
  //m_dDBoxLength為detect box的長度,速度越快所需的detect box長度需要越長
  m_dDBoxLength = 30 + 
                  (m_pVehicle->Speed()/m_pVehicle->MaxSpeed()) * 30;
  //把在detect box內的obstacle都打上tag
  TagNeighbors(m_pVehicle,obstacles,m_dDBoxLength);
  //取出每個障礙物
  std::vector<BaseClass*>::const_iterator curOb = obstacles.begin();
  while(curOb != obstacles.end())
  {
      if ((*curOb)->IsTagged())
      {
        //計算障礙物相對於物體的位置向量       
        V2D LocalPos = m_pVehicle->GetWorld()->PointToLocalSpace(
                                             (*curOb)->Pos(),
                                             m_pVehicle->Heading(),
                                             m_pVehicle->Side(),
                                             m_pVehicle->Pos());
         //檢查x<0表示在物體朝向的後方則不予考慮
         if (LocalPos.x >= 0)
         {
           //檢查y如果在物體跟障礙物的邊界半徑內則表示之後行進間會發生碰撞
           double ExpandedRadius = (*curOb)->BRadius() + m_pVehicle->BRadius();
           if (fabs(LocalPos.y) < ExpandedRadius)
           {
             double cX = LocalPos.x;
             double cY = LocalPos.y;
             //因為LocalPos表示物體在原點時障礙物的相對位置
             //所以障礙物由ExpandedRadius產生的半徑圓和X軸相交最近的交點
             //就會是物體移動到會跟障礙物發生碰撞的位置
             //求交點公式為cx +或- sqrt(r^2 - cy^2)
             //因為有兩個交點所以有+或-
             double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cY*cY);
             //ip為相交時的x值
             double ip = cX - SqrtPart;
             //ip小於0表示相交在物體朝向的後方,所以不考慮
             if (ip <= 0.0)
             {
                ip = cX + SqrtPart;
             }
              //找出x最接近物體的,即最先相遇的Obstacle
              if (ip < DistToClosestIP)
              {
                DistToClosestIP = ip;
                ClosestIntersectingObstacle = *curOb;
                LocalPosOfClosestObstacle = LocalPos;
              }   
           }
         }
         
      }
      ++curOb;
  }
  V2D SteeringForce = V2D(0,0);
  if (ClosestIntersectingObstacle)
  {
    //計算側向操控力(轉向的力)
    //比值multiplier正比於Detect box長度減去LocalPosOfClosestObstacle.x"障礙物距物體的x距離"
    //所以物體距障礙物越近時比值multiplier影響越大
    //物體速度越快m_dDBoxLength越長,比值multiplier也影響越大
    double multiplier = 1.0 + (m_dDBoxLength - LocalPosOfClosestObstacle.x) /
                        m_dDBoxLength;
    //側向操控力最多即為障礙物的邊界半徑
    //再減去LocalPosOfClosestObstacle.y乘比值multiplier                  
    //LocalPosOfClosestObstacle.y越大表示y方向物體離障礙物越遠,就不需要這麼大的力了
    SteeringForce.y = (ClosestIntersectingObstacle->BRadius()-
                       LocalPosOfClosestObstacle.y)  * multiplier;  
    //計算制動力 "加減速的力"
    const double BrakingWeight = 1;
    //當進入一定範圍時,LocalPosOfClosestObstacle.x會比BRadius大,制動力負值所以進入減速階段
    //越接近時LocalPosOfClosestObstacle.x變小制動力回到正值且側向操控力夠大所以可以脫離障礙物
    SteeringForce.x=(ClosestIntersectingObstacle->BRadius() - LocalPosOfClosestObstacle.x)
                        * BrakingWeight; 
  }
  //V2D(0,0)表示不進行平移,因為SteeringForce本來就是力,不用平移
  V2D worldSpaceForce = m_pVehicle->GetWorld()->PointToWorldSpace(SteeringForce,
                                    m_pVehicle->Heading(),
                                    m_pVehicle->Side(),
                                    V2D(0,0));
  return worldSpaceForce;
}

V2D SteeringBehavior::WallAvoidance(std::vector<Wall>& walls)
{
  //建立觸角中,左,右3根
   m_Feelers[0] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength * m_pVehicle->Heading();
  //feeler to left
  V2D temp = m_pVehicle->Heading();
  //Heading()原本是(0,0)-(目前面向方向),現在順時針轉3.5*90度
  V2DRotateAroundOrigin(temp, HalfPi * 3.5f);
  m_Feelers[1] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength/2.0f * temp;
  //feeler to right
  temp = m_pVehicle->Heading();
   //順時針轉0.5*90度
  V2DRotateAroundOrigin(temp, HalfPi * 0.5f);
  m_Feelers[2] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength/2.0f * temp;
  //
  double DistToThisIP    = 0.0;
  double DistToClosestIP = MaxDouble;
  //找出最接近的牆編號
  int ClosestWall = -1;
  V2D SteeringForce,
            point,         //used for storing temporary info
            ClosestPoint;  //holds the closest intersection point
  //對每根觸角檢查牆是否與之相交
  for (unsigned int flr=0; flr<3; ++flr)
  {
    //ClosestWall = -1;
    for (unsigned int w=0; w<walls.size(); ++w)
    {
        //
        if (LineIntersection2D(m_pVehicle->Pos(),
                             m_Feelers[flr],
                             walls[w].From(),
                             walls[w].To(),
                             DistToThisIP,
                             point))
        {
          //有可能會同一時間不只一根觸角穿透牆,選最近的
           if (DistToThisIP < DistToClosestIP)
          {
            DistToClosestIP = DistToThisIP;
            ClosestWall = w;
            ClosestPoint = point;
          }
        }
    }
    //表示編號為ClosestWall的牆跟觸角編號flr相交
    if (ClosestWall >=0)
    {
      //ClosestPoint表示相交的點的位置,所以相減的OverShoot表示觸角穿透牆的長度
      V2D OverShoot = m_Feelers[flr] - ClosestPoint;
      //給予一個垂直牆的力,強度為觸角穿透牆的程度
      SteeringForce = walls[ClosestWall].Normal() * OverShoot.Length();
    }
  }
  return SteeringForce;
}

V2D SteeringBehavior::Interpose(const Vehicle* AgentA,
                                const Vehicle* AgentB)
{
  //取當前中點
  V2D MidPoint = (AgentA->Pos() + AgentB->Pos()) / 2.0;
  //評估幾秒後到達該中點
  double TimeToReachMidPoint = V2DDistance(m_pVehicle->Pos(), MidPoint) /
                               m_pVehicle->MaxSpeed();
  //評估幾秒後A,B到達的位置
  V2D APos = AgentA->Pos() + AgentA->Velocity() * TimeToReachMidPoint;
  V2D BPos = AgentB->Pos() + AgentB->Velocity() * TimeToReachMidPoint;

  MidPoint = (APos + BPos) / 2.0;
  return Arrive(MidPoint);
}

//給障礙物位置posOb跟其半徑大小radiusOb
//相對於獵人位置posHunter,算出Hiding位置
V2D SteeringBehavior::GetHidingPosition(const V2D& posOb,
                                              const double     radiusOb,
                                              const V2D& posHunter)
{
  const double DistanceFromBoundary = 30.0;
  double       DistAway    = radiusOb + DistanceFromBoundary;
  //求出對於障礙物,躲藏的方向
  V2D ToOb = V2DNormalize(posOb - posHunter);
  //該方向ToOb乘上合適距離, 再相對於障礙物位置posOb
  return (ToOb * DistAway) + posOb;
}


V2D SteeringBehavior::Hide(const Vehicle* hunter,
                                const std::vector<BaseClass*>& obstacles)
{
  double DistToClosest = MaxDouble;
  V2D BestHidingSpot;
  std::vector<BaseClass*>::const_iterator curOb = obstacles.begin();
  std::vector<BaseClass*>::const_iterator closest;
  while(curOb != obstacles.end())
  {
     //求出每一個障礙物的合適躲藏點
     V2D HidingSpot = GetHidingPosition((*curOb)->Pos(),
                                             (*curOb)->BRadius(),
                                              hunter->Pos());
     //算出距躲藏點的距離 
     double dist = V2DDistanceSq(HidingSpot, m_pVehicle->Pos());
     //求出最近的躲藏點放入BestHidingSpot
     if (dist < DistToClosest)
     {
      DistToClosest = dist;
      BestHidingSpot = HidingSpot;
      closest = curOb;
     }         
    ++curOb;
  }
  //如果沒有合適的位置就啟動逃避
  if (DistToClosest == MaxDouble)
  {
    return Evade(hunter);
  }
  return Arrive(BestHidingSpot);
}

V2D SteeringBehavior::FollowPath()
{
      if (GetPath()->GetWayPointSize() == 0) return V2D(0,0);
      //定義如果相距距離小於5則指定下一點位置0
      double m_dWaypointSeekDistSq = 50*50;
      if(V2DDistanceSq(m_pPath->CurrentWaypoint(), m_pVehicle->Pos()) <
         m_dWaypointSeekDistSq)
      {
        m_pPath->SetNextWaypoint();
      }
      
      if (!m_pPath->Finished())
      {
        return Seek(m_pPath->CurrentWaypoint());
      }
      else
      {
        return Arrive(m_pPath->CurrentWaypoint());
      }
}

V2D SteeringBehavior::OffsetPursuit(const Vehicle* leader, V2D offset)
{
  //將offset轉換成世界座標
  V2D WorldOffsetPos = m_pVehicle->GetWorld()->PointToWorldSpace(offset,
                                                  leader->Heading(),
                                                  leader->Side(),
                                                  leader->Pos());
  //求出m_pVehicle到達該世界座標的距離
  V2D ToOffset = WorldOffsetPos - m_pVehicle->Pos();
  //求出m_pVehicle到達該世界座標的時間
  double LookAheadTime = ToOffset.Length() / 
                        (m_pVehicle->MaxSpeed() + leader->Speed());
  return Arrive(WorldOffsetPos + leader->Velocity() * LookAheadTime);
}

V2D SteeringBehavior::Separation(const std::vector<Vehicle*> &neighbors)
{  
  V2D SteeringForce;
  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    //如果有被打上TAG表示離的夠近
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged()) 
    {
      //求出鄰居向物體的方向向量
      V2D ToAgent = m_pVehicle->Pos() - neighbors[a]->Pos();
      //隨著離鄰居越遠力越弱
      SteeringForce += V2DNormalize(ToAgent)/ToAgent.Length();
    }
  }
  return SteeringForce;
}

//改變物體的朝向方向
V2D SteeringBehavior::Alignment(const std::vector<Vehicle*>& neighbors)
{
  V2D AverageHeading;
  int NeighborCount = 0;
  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged() &&
      (neighbors[a] != m_pTargetAgent1))
    {
      AverageHeading += neighbors[a]->Heading();
      ++NeighborCount;
    }
  }
  if (NeighborCount > 0)
  {
    AverageHeading /= (double)NeighborCount;
    //減去物體的正向力,所以只剩下轉向的力,只改變物體的朝向
    AverageHeading -= m_pVehicle->Heading();
  }
  return AverageHeading;
}

V2D SteeringBehavior::Cohesion(const std::vector<Vehicle*> &neighbors)
{
  //first find the center of mass of all the agents
  V2D CenterOfMass, SteeringForce;
  int NeighborCount = 0;
  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged() &&
      (neighbors[a] != m_pTargetAgent1))
    {
      //計算質心
      CenterOfMass += neighbors[a]->Pos();
      ++NeighborCount;
    }
  }
  if (NeighborCount > 0)
  {
    //計算質心
    CenterOfMass /= (double)NeighborCount;
    //往質心的力當作聚集的力
    SteeringForce = Seek(CenterOfMass);
  }
  //通常正規化,因為該力通常遠大於Alignment,跟Separation
  return V2DNormalize(SteeringForce);
}


V2D SteeringBehavior::SeparationPlus()
{  
  V2D SteeringForce;
  for (BaseClass* pV = m_pVehicle->GetWorld()->CellSpace()->begin();
                         !m_pVehicle->GetWorld()->CellSpace()->end();     
                       pV = m_pVehicle->GetWorld()->CellSpace()->next())
  {    
    if(pV != m_pVehicle)
    {
      V2D ToAgent = m_pVehicle->Pos() - pV->Pos();
      SteeringForce += V2DNormalize(ToAgent)/ToAgent.Length();
    }
  }
  return SteeringForce;
}

V2D SteeringBehavior::AlignmentPlus()
{
  V2D AverageHeading;
  double    NeighborCount = 0.0;
  for (MovingObject* pV = m_pVehicle->GetWorld()->CellSpace()->begin();
                         !m_pVehicle->GetWorld()->CellSpace()->end();     
                     pV = m_pVehicle->GetWorld()->CellSpace()->next())
  {
    if(pV != m_pVehicle)
    {
      AverageHeading += pV->Heading();
      ++NeighborCount;
    }
  }
  if (NeighborCount > 0.0)
  {
    AverageHeading /= NeighborCount;
    AverageHeading -= m_pVehicle->Heading();
  }
  return AverageHeading;
}

V2D SteeringBehavior::CohesionPlus()
{
  //first find the center of mass of all the agents
  V2D CenterOfMass, SteeringForce;
  int NeighborCount = 0;
  //iterate through the neighbors and sum up all the position vectors
  for (BaseClass* pV = m_pVehicle->GetWorld()->CellSpace()->begin();
                         !m_pVehicle->GetWorld()->CellSpace()->end();     
                       pV = m_pVehicle->GetWorld()->CellSpace()->next())
  {
    if(pV != m_pVehicle)
    {
      CenterOfMass += pV->Pos();
      ++NeighborCount;
    }
  }
  if (NeighborCount > 0)
  {
    CenterOfMass /= (double)NeighborCount;
    SteeringForce = Seek(CenterOfMass);
  }
  return V2DNormalize(SteeringForce);
}
