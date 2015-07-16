#ifndef STEERINGBEHAVIORS_H
#define STEERINGBEHAVIORS_H
#include "vehicle.h"
#include "wall.h"
#include "path.h"
#include "CellSpacePartition.h"
const double WanderRad    = 20;
const double WanderJitterPerSec = 400.0;
const double WanderDist = 30;
class Vehicle;
class SteeringBehavior
{
private:
   enum behavior_type
  {
    //如此編碼可以同時存在多個behavior狀態
    none               = 0x00000,
    seek               = 0x00002,
    flee               = 0x00004,
    arrive             = 0x00008,
    wander             = 0x00010,
    cohesion           = 0x00020,
    separation         = 0x00040,
    allignment         = 0x00080,
    obstacle_avoidance = 0x00100,
    wall_avoidance     = 0x00200,
    follow_path        = 0x00400,
    pursuit            = 0x00800,
    evade              = 0x01000,
    interpose          = 0x02000,
    hide               = 0x04000,
    flock              = 0x08000,
    offset_pursuit     = 0x10000,
  };
  //定義Flag決定開啟哪些行為
  int m_iFlags;
	enum summing_method{weighted_average, prioritized, dithered};
    summing_method m_SummingMethod;
	Vehicle* m_pVehicle;
  Vehicle* m_pTargetAgent1;
  Vehicle* m_pTargetAgent2;
  //offset_pursuit用的參數
  V2D m_vOffset;
	V2D m_vSteeringForce;
	V2D m_vWanderTarget; 
	double m_dWeightWander;
	double m_dWanderRadius;
	double m_dWanderJitter;
	double m_dWanderDistance;
  double m_dWeightSeek;
  double m_dWeightFlee;
  double m_dWeightArrive;
  double m_dWeightPursuit;
  double m_dWeightEvade;
  double m_dWeightObstacleAvoidance;
  //'detection box'的長度
  double m_dDBoxLength;
  double m_dWeightWallAvoidance;
  double m_dWallDetectionFeelerLength;
  double m_dWeightInterpose;
  double m_dWeightHide;
  double m_dWeightFollowPath;
  Path* m_pPath;
  double m_dWeightOffsetPursuit;
  double m_dWeightSeparation;
  double m_dWeightAlignment;
  double m_dWeightCohesion;
  double m_dViewDistance;
public:
  V2D m_Feelers[3];
  SteeringBehavior(){};
  SteeringBehavior(Vehicle* agent);
  virtual ~SteeringBehavior();	
  V2D Calculate();
  V2D CalculateWeightedSum();
  //計算RunningTot是否加入ForceToAdd還低於m_pVehicle->MaxForce()
  bool AccumulateForce(V2D &RunningTot,
                                       V2D ForceToAdd);
  V2D CalculatePrioritized();
  V2D CalculateDithered();
  //檢查m_iFlags如果behavior_type有被ON的話傳回True
  bool On(behavior_type bt){return (m_iFlags & bt) == bt;}
  V2D Wander();
  bool HasWander(){if (On(wander)) return TRUE; else return FALSE;}
  void WanderON(){m_iFlags |= wander;}
  void WanderOFF(){if (On(wander)) m_iFlags ^= wander;}
  double WanderRadius()const{return m_dWanderRadius;}
  double WanderDistance()const{return m_dWanderDistance;}
  V2D Seek(V2D TargetPos);
  void SeekON(){m_iFlags |= seek;}
  void SeekOFF(){if (On(seek)) m_iFlags ^= seek;}
  V2D Flee(V2D TargetPos);
  void FleeON(){m_iFlags |= flee;}
  void FleeOFF(){if (On(flee)) m_iFlags ^= flee;}
  V2D Arrive(V2D TargetPos);
  void ArriveON(){m_iFlags |= arrive;}
  void ArriveOFF(){if (On(arrive)) m_iFlags ^= arrive;}

  V2D Pursuit(const Vehicle* evader);
  //指定要追尋的目標物
  void PursuitON(Vehicle* v){m_iFlags |= pursuit; m_pTargetAgent1 = v;}
  void PursuitOFF(){if (On(pursuit)) m_iFlags ^= pursuit; m_pTargetAgent1=NULL;}
  
  V2D Evade(const Vehicle* pursuer);
  void EvadeON(Vehicle* v){m_iFlags |= evade; m_pTargetAgent1 = v;}
  void EvadeOFF(){if (On(evade)) m_iFlags ^= evade; m_pTargetAgent1=NULL;}
  
  V2D ObstacleAvoidance(std::vector<BaseClass*>& obstacles);
  void ObstacleAvoidanceON(){m_iFlags |= obstacle_avoidance;}
  void ObstacleAvoidanceOFF(){if (On(obstacle_avoidance)) m_iFlags ^= obstacle_avoidance;}
  bool HasObstacleAvoidance(){if (On(obstacle_avoidance)) return TRUE; else return FALSE;}
  double dDBoxLength()const{return m_dDBoxLength;}

  V2D WallAvoidance(std::vector<Wall>& walls);
  void WallAvoidanceON(){m_iFlags |= wall_avoidance;}
  void WallAvoidanceOFF(){if (On(wall_avoidance)) m_iFlags ^= wall_avoidance;}
  bool HasWallAvoidance(){if (On(wall_avoidance)) return TRUE; else return FALSE;}

  V2D Interpose(const Vehicle* AgentA,const Vehicle* AgentB);
  void InterposeON(Vehicle* v1,Vehicle* v2){m_iFlags |= interpose; m_pTargetAgent1 = v1;m_pTargetAgent2 = v2;}
  void InterposeOFF(){if (On(interpose)) m_iFlags ^= interpose;m_pTargetAgent1=NULL;m_pTargetAgent2=NULL;}

  V2D Hide(const Vehicle* hunter, const std::vector<BaseClass*>& obstacles);
  V2D GetHidingPosition(const V2D& posOb,const double radiusOb, const V2D& posHunter);
  bool HasHide(){if (On(hide)) return TRUE; else return FALSE;}
  void HideON(Vehicle* v){m_iFlags |= hide; m_pTargetAgent1 = v;}
  void HideOFF(){if (On(hide)) m_iFlags ^= hide; m_pTargetAgent1=NULL;}

  V2D FollowPath();
  void FollowPathON(){m_iFlags |= follow_path;}
  void FollowPathOFF(){if (On(follow_path)) m_iFlags ^= follow_path;}
  bool HasFollowPath(){if (On(follow_path)) return TRUE; else return FALSE;}
  void SetPath(std::list<V2D> new_path){m_pPath->Set(new_path);}
  Path* GetPath()const{return m_pPath;}
  void AddPathWayPoint(V2D new_waypoint){m_pPath->AddWayPoint(new_waypoint); if (GetPath()->GetWayPointSize()==1) GetPath()->ReStartPath();}
  
  V2D OffsetPursuit(const Vehicle* leader, V2D offset);
  //指定要追尋的目標物
  void OffsetPursuitON(Vehicle* v, const V2D offset){m_iFlags |= offset_pursuit; m_pTargetAgent1 = v;m_vOffset = offset;}
  void OffsetPursuitOFF(){if (On(offset_pursuit)) m_iFlags ^= offset_pursuit; m_pTargetAgent1=NULL;}

  // -- Group Behaviors -- //
  V2D Separation(const std::vector<Vehicle*> &agents);
  void SeparationON(){m_iFlags |= separation;}
  void SeparationOFF(){if (On(separation)) m_iFlags ^= separation;}
  //改變物體的朝向方向
  V2D Alignment(const std::vector<Vehicle*>& neighbors);
  void AlignmentON(){m_iFlags |= allignment;}
  void AlignmentOFF(){if (On(allignment)) m_iFlags ^= allignment;}
 
   V2D Cohesion(const std::vector<Vehicle*> &neighbors);
   void CohesionON(){m_iFlags |= cohesion;}
   void CohesionOFF(){if (On(cohesion)) m_iFlags ^= cohesion;}
  
  V2D CohesionPlus();
  V2D SeparationPlus();
  V2D AlignmentPlus();
};
#endif