#include "world.h"
#include "neogdi.h"

#include <stdio.h>
#include <string>
#include "C2DMatrix.h"


World::World(int cx, int cy):  m_cxClient(cx),
            				   m_cyClient(cy),
                       m_dAvFrameTime(0),
                       m_vCrosshair(V2D(cxClient()/2.0, cxClient()/2.0)),
            				   m_bPaused(false),
            				   m_bShowFPS(true)
{
  //
   m_pCellSpace = new CellSpacePartition<Vehicle*>((double)cx, (double)cy, 10, 10, 200);

  //建立牆,這裡是建立外框
  const int NumWallVerts = 8;
  double bordersize = 35.0;
  double CornerSize = 0.2;
  double vDist = m_cyClient-2*bordersize;
  double hDist = m_cxClient-2*bordersize;
  V2D walls[NumWallVerts] = {V2D(hDist*CornerSize+bordersize, bordersize),
                             V2D(m_cxClient-bordersize-hDist*CornerSize, bordersize),
                             V2D(m_cxClient-bordersize, bordersize+vDist*CornerSize),
                             V2D(m_cxClient-bordersize, m_cyClient-bordersize-vDist*CornerSize),
                                       
                             V2D(m_cxClient-bordersize-hDist*CornerSize, m_cyClient-bordersize),
                             V2D(hDist*CornerSize+bordersize, m_cyClient-bordersize),
                             V2D(bordersize, m_cyClient-bordersize-vDist*CornerSize),
                             V2D(bordersize, bordersize+vDist*CornerSize)};
  
  for (int w=0; w<NumWallVerts-1; ++w)
  {
   m_Walls.push_back(Wall(walls[w], walls[w+1]));
  }

  m_Walls.push_back(Wall(walls[NumWallVerts-1], walls[0]));
  //建立障礙物
  for (int a=0; a<10; ++a)
  {
    while(1){
    //隨機起始位置Spawn, (cx,cy)為視窗長寬
    V2D SpawnPos = V2D(cx/2.0+RandomClamped()*cx/2.0,
                       cy/2.0+RandomClamped()*cy/2.0);
    Obstacle* pObstacle = new Obstacle(
                                    SpawnPos, 
                                    RandInt(5,50));     
    if (!Overlapped(pObstacle, m_Obstacles, 40))
    {  
      m_Obstacles.push_back(pObstacle);
      break;
    }else{
      delete pObstacle;
    }
    }
  }

  //建立本體MyVehicle
  Vehicle* pMyVehicle = new Vehicle(this,
                                    V2D(cx/2.0,cy/2.0),               
                                    0,   //start rotation
                                    V2D(0,0),            //velocity
                                    0.01,               //mass
                                    2.0,                //max force
                                    150.0,             //max velocity
                                    Pi,                //max turn rate
                                    15);               //scale
  m_Vehicles.push_back(pMyVehicle);
  m_Vehicles[0]->GetSteering()->ArriveON();
  m_Vehicles[0]->GetSteering()->WanderOFF();
  m_Vehicles[0]->GetSteering()->ObstacleAvoidanceON();
  m_Vehicles[0]->GetSteering()->WallAvoidanceON();
  //m_Vehicles[0]->GetSteering()->OffsetPursuitON(m_Vehicles[0],V2D(30,30));
  //m_Vehicles[m_Vehicles.size()-1]->GetSteering()->FollowPathON();
   //m_Vehicles[m_Vehicles.size()-1]->GetSteering()->InterposeON(m_Vehicles[0],m_Vehicles[1]);
  //m_Vehicles[m_Vehicles.size()-1]->GetSteering()->PursuitON(m_Vehicles[0]);
  //建立vehicle
  for (int a=1; a<50; ++a)
  {
    //隨機起始位置Spawn, (cx,cy)為視窗長寬
    V2D SpawnPos = V2D(cx/2.0+RandomClamped()*cx/2.0,
                       cy/2.0+RandomClamped()*cy/2.0);
    Vehicle* pVehicle = new Vehicle(this,
                                    SpawnPos, 
                                    //HalfPi,                //initial position
                                    RandFloat()*TwoPi,        //start rotation
                                    V2D(0,0),            //velocity
                                    0.01,          //mass
                                    5.0,     //max force
                                    150.0,             //max velocity
                                    Pi, //max turn rate
                                    5);        //scale

    pVehicle->GetSteering()->WanderON();
    pVehicle->GetSteering()->CohesionON();
    pVehicle->GetSteering()->AlignmentON(); 
    pVehicle->GetSteering()->SeparationON();
    pVehicle->GetSteering()->ObstacleAvoidanceON();
    pVehicle->GetSteering()->WallAvoidanceON();
    m_Vehicles.push_back(pVehicle);

    //add it to the cell subdivision
    m_pCellSpace->AddObject(pVehicle);
  }
 //初始化其他
  m_Sleected_Vehicles_ID = -1;

  m_pCellSpace->EmptyCells();    
  for (unsigned int i=0; i<m_Vehicles.size(); ++i)
  {
    m_pCellSpace->AddObject(m_Vehicles[i]);
  }
}

World::~World()
{
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    delete m_Vehicles[a];
  }

  for (unsigned int ob=0; ob<m_Obstacles.size(); ++ob)
  {
    delete m_Obstacles[ob];
  }
  delete m_pCellSpace;
}

void World::Update(double time_elapsed)
{ 
  //
  if (m_bPaused) return;

  const int SampleRate = 10;
  //定義最多10個time_elapsed取平均值
  static Smoother<double> FrameRateSmoother(SampleRate, 0.0);

  m_dAvFrameTime = FrameRateSmoother.Update(time_elapsed);
  //wprintf(L"%f\n",m_dAvFrameTime);
  for (unsigned int a=0; a<m_Obstacles.size(); ++a)
  {
    m_Obstacles[a]->Update(time_elapsed);
  }
  //更新Vehicles
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    //打上鄰近標籤
    //TagNeighbors(m_Vehicles[a],m_Vehicles,20);
    V2D oldPos = m_Vehicles[a]->Pos();
    m_Vehicles[a]->Update(time_elapsed);
    //更新各個格子Cell內的物件
    CellSpace()->UpdateObject(m_Vehicles[a], oldPos);
  }
  //
 
}

void World::HandleKeyPresses(WPARAM wParam)
{
  switch(wParam)
  {
    case 'U':
    {
      wprintf(L"%s",L"按下U\n"); 
    }
    break;
    case 'P':
    {
      wprintf(L"%s",L"按下P\n"); 
      TogglePause(); 
    }
    break;
    case 'F':
    {
      wprintf(L"%s",L"按下F\n"); 
      ToggleShowFPS(); 
    }
    break;
  }
}

void World::HandleMenuItems(WPARAM wParam, HWND hwnd)
{
}

void World::Render()
{
  ngdi->TransparentText();
  ngdi->HollowBrush();
  //
  ngdi->DarkGreenPen();
  CellSpace()->RenderCells();
  //
  //畫出左鍵滑鼠點擊的位置 
  ngdi->YellowPen();
  ngdi->Cross(Crosshair(),10);
  //
  ngdi->LightGreyPen();
  ngdi->GreyBrush();
  for (unsigned int a=0; a<m_Walls.size(); ++a)
  {
    m_Walls[a].Render();
  }
  //畫出障礙物
  ngdi->LightGreyPen();
  //ngdi->HollowBrush();
  ngdi->GreyBrush();
  for (unsigned int a=0; a<m_Obstacles.size(); ++a)
  {
    m_Obstacles[a]->Render();
  }
  //畫出Vehicles
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    if (m_Vehicles[a]->Type()==1){
      ngdi->GreyPen();
    } else{
      ngdi->GreenPen();
    }
    m_Vehicles[a]->Render();
    //秀物件ID  
    //ngdi->TextColor(NEOgdi::yellow);
    //ngdi->TextAtPos(m_Vehicles[a]->Pos().x, m_Vehicles[a]->Pos().y-30, ttos(m_Vehicles[a]->ID()));
  /*  
    //render cell partitioning stuff
    
    if (m_bShowCellSpaceInfo && a==0)
    {
      gdi->HollowBrush();
      InvertedAABBox2D box(m_Vehicles[a]->Pos() - Vector2D(Prm.ViewDistance, Prm.ViewDistance),
                           m_Vehicles[a]->Pos() + Vector2D(Prm.ViewDistance, Prm.ViewDistance));
      box.Render();

      gdi->RedPen();
      CellSpace()->CalculateNeighbors(m_Vehicles[a]->Pos(), Prm.ViewDistance);
      for (BaseGameEntity* pV = CellSpace()->begin();!CellSpace()->end();pV = CellSpace()->next())
      {
        gdi->Circle(pV->Pos(), pV->BRadius());
      }
      
      gdi->GreenPen();
      gdi->Circle(m_Vehicles[a]->Pos(), Prm.ViewDistance);
    }
    */
  } 
  if (RenderFPS())
  {
    ngdi->TextColor(NEOgdi::red);
    ngdi->TextAtPos(5, cyClient() - 20, ttos(1.0 / m_dAvFrameTime));
  } 
  ngdi->OpaqueText();
}

void World::SetCrosshair(POINTS p)
{
  //wprintf(L"按下%lf,%lf\n",(double)p.x, (double)p.y);
  SetCrosshair(V2D (p.x,p.y));
  //
  //m_Vehicles[m_Vehicles.size()-1]->GetSteering()->AddPathWayPoint(V2D(p.x,p.y));
}
void World::SetSelected(POINTS p)
{
  unsigned int selected=0;
  double closest_dist_sq = MaxDouble;
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    double dist_sq = V2DDistanceSq(m_Vehicles[a]->Pos(),V2D(p.x,p.y));
    if (dist_sq < closest_dist_sq){
      closest_dist_sq = dist_sq;
      selected = a;
    }
  }
  if (closest_dist_sq < 100){
    if (m_Sleected_Vehicles_ID != -1) m_Vehicles[m_Sleected_Vehicles_ID]->SetType(0);
    m_Sleected_Vehicles_ID = selected;
    m_Vehicles[selected]->SetType(1);
    //
     //m_Vehicles[selected]->GetSteering()->OffsetPursuitOFF();
     //打上鄰居標籤
    //TagNeighbors(m_Vehicles[selected],m_Vehicles,100);
    // m_Vehicles[selected]->GetSteering()->CohesionON();
  }
  //m_Vehicles 
  //wprintf(L"按下%lf,%lf\n",(double)p.x, (double)p.y);
  //SetCrosshair(V2D (p.x,p.y));
  //
  //m_Vehicles[m_Vehicles.size()-1]->GetSteering()->AddPathWayPoint(V2D(p.x,p.y));
}

//利用C2DMatrix轉換出實際世界座標
//1)設定縮放,C2DMatrix.Scale(x比例,y比例)
//2)設定旋轉角度,C2DMatrix.Rotate(正面向量,側面向量)
//3)設定平移,C2DMatrix.Translate(x,y)
//4)轉成世界座標C2DMatrix.TransformVector2Ds(外型)

std::vector<V2D> World::WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            const V2D   &forward,
                                            const V2D   &side,
                                            const V2D   &scale)
{
  //copy the original vertices into the buffer about to be transformed
  std::vector<V2D> TranVector2Ds = points;
  //create a transformation matrix
  C2DMatrix matTransform;
  //scale
  if ( (scale.x != 1.0) || (scale.y != 1.0) )
  {
    matTransform.Scale(scale.x, scale.y);
  }
  //rotate
  matTransform.Rotate(forward, side);
  //and translate
  matTransform.Translate(pos.x, pos.y);
  //now transform the object's vertices
  matTransform.TransformVector2Ds(TranVector2Ds);

  return TranVector2Ds;
}

inline std::vector<V2D> World::WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            const V2D   &forward,
                                            const V2D   &scale)
{
  //copy the original vertices into the buffer about to be transformed
  std::vector<V2D> TranVector2Ds = points;
  //create a transformation matrix
  C2DMatrix matTransform;
  //scale
  if ( (scale.x != 1.0) || (scale.y != 1.0) )
  {
    matTransform.Scale(scale.x, scale.y);
  }
  //rotate
  matTransform.Rotate(forward);
  //and translate
  matTransform.Translate(pos.x, pos.y);
  //now transform the object's vertices
  matTransform.TransformVector2Ds(TranVector2Ds);

  return TranVector2Ds;
}

//給定一連串相對原點的座標,依據pos跟angle,scale等轉成相對pos後的世界座標
inline std::vector<V2D> World::WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            double angle,
                                            const V2D   &scale)
{
  //copy the original vertices into the buffer about to be transformed
  std::vector<V2D> TranVector2Ds = points;
  //create a transformation matrix
  C2DMatrix matTransform;
  //scale
  if ( (scale.x != 1.0) || (scale.y != 1.0) )
  {
    matTransform.Scale(scale.x, scale.y);
  }
  //rotate
  matTransform.Rotate(angle);
  //and translate
  matTransform.Translate(pos.x, pos.y);
  //now transform the object's vertices
  matTransform.TransformVector2Ds(TranVector2Ds);

  return TranVector2Ds;
}

//以Agent的方向跟位置為原點的向量point, 轉成世界座標
V2D World::PointToWorldSpace(const V2D &point,
                                    const V2D &AgentHeading,
                                    const V2D &AgentSide,
                                    const V2D &AgentPosition)
{
  //make a copy of the point
  V2D TransPoint = point;
  //create a transformation matrix
  C2DMatrix matTransform;
  //rotate
  matTransform.Rotate(AgentHeading, AgentSide);
  //and translate
  matTransform.Translate(AgentPosition.x, AgentPosition.y);
  //now transform the vertices
  matTransform.TransformVector2Ds(TransPoint);
  return TransPoint;
}

V2D World::PointToLocalSpace(const V2D& point,
                       const      V2D& AgentHeading,
                       const      V2D& AgentSide,
                        const      V2D& AgentPosition)
{

  //make a copy of the point
  V2D TransPoint = point;
  
  //create a transformation matrix
  C2DMatrix matTransform;

  double Tx = -AgentPosition.Dot(AgentHeading);
  double Ty = -AgentPosition.Dot(AgentSide);

  //create the transformation matrix
  matTransform._11(AgentHeading.x); matTransform._12(AgentSide.x);
  matTransform._21(AgentHeading.y); matTransform._22(AgentSide.y);
  matTransform._31(Tx);           matTransform._32(Ty);
  
  //now transform the vertices
  matTransform.TransformVector2Ds(TransPoint);

  return TransPoint;
}
