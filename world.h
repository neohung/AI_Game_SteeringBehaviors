#ifndef WORLD_H
#define WORLD_H

#include <windows.h>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <string>
#include <iomanip>

#include "movingobj.h"
#include "CellSpacePartition.h"
#include "Smoother.h"
//將class T轉成字串
template <class T>
inline std::basic_string<wchar_t> ttos(const T& t, int precision = 2)
{
  std::ostringstream buffer;
  buffer << std::fixed << std::setprecision(precision) << t;
  std::basic_string<char> c = buffer.str();
  const size_t cSize = buffer.str().length()+1;
  wchar_t* wc = new wchar_t[cSize];
  mbstowcs (wc, c.c_str(), cSize);
  return wc;
 // return (const wchar_t*)s.str();
}
//
inline std::basic_string<wchar_t> btos(bool b)
{
  if (b) return L"true";
  return L"false";
}
//
template <typename T>
inline T GetValueFromStream(std::ifstream& stream)
{
  T val;
  stream >> val;
  return val;
}
//
template <typename T>
void WriteBitsToStream(std::ostream& stream, const T& val)
{
  int iNumBits = sizeof(T) * 8;

  while (--iNumBits >= 0)
  {
    if ((iNumBits+1) % 8 == 0) stream << " ";
    unsigned long mask = 1 << iNumBits;
    if (val & mask) stream << "1";
    else stream << "0";
  }
}

//-----------------------------------------------------------------------
#include "vehicle.h"
#include "obstacle.h"
#include "wall.h"
class Vehicle;
class World
{ 
private:
  //a container of all the moving entities
  std::vector<Vehicle*>   m_Vehicles;
  int m_Sleected_Vehicles_ID;
  std::vector<BaseClass*>  m_Obstacles;
  std::vector<Wall> m_Walls;
  CellSpacePartition<Vehicle*>* m_pCellSpace;
  int  m_cxClient,
       m_cyClient;
  //keeps track of the average FPS
  double m_dAvFrameTime;
  V2D   m_vCrosshair;
  bool m_bPaused;
  bool m_bShowFPS;
public:
  World(int cx, int cy);
  ~World();
  void Update(double time_elapsed);
  void Render();
  void HandleKeyPresses(WPARAM wParam);
  void HandleMenuItems(WPARAM wParam, HWND hwnd);
  void TogglePause(){m_bPaused = !m_bPaused;}
  bool Paused()const{return m_bPaused;}
  bool RenderFPS()const{return m_bShowFPS;}
  void ToggleShowFPS(){m_bShowFPS = !m_bShowFPS;}
  //當滑鼠點擊時
  void SetCrosshair(POINTS p);
  void SetSelected(POINTS p);
  //傳回視窗長寬
  int   cxClient()const{return m_cxClient;}
  int   cyClient()const{return m_cyClient;}
  V2D   Crosshair()const{return m_vCrosshair;}
  void  SetCrosshair(V2D v){m_vCrosshair=v;}
  //
  std::vector<BaseClass*>& Obstacles(){return m_Obstacles;}
  std::vector<Wall>& Walls(){return m_Walls;}
  const std::vector<Vehicle*>& Agents(){return m_Vehicles;}
  //
  CellSpacePartition<Vehicle*>* CellSpace()const{return m_pCellSpace;}
  //
  //給定外型頂點座標points跟實際在World的物件中心位置pos,正面方向forward
  //縮放比率scale等等, 算出實際在World的外型頂點座標
   std::vector<V2D> WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            const V2D   &forward,
                                            const V2D   &side,
                                            const V2D   &scale);
  inline std::vector<V2D> WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            const V2D   &forward,
                                            const V2D   &scale);
  inline std::vector<V2D> WorldTransform(std::vector<V2D> &points,
                                            const V2D   &pos,
                                            double angle,
                                            const V2D   &scale);

  V2D PointToWorldSpace(const V2D &point,
                                    const V2D &AgentHeading,
                                    const V2D &AgentSide,
                                    const V2D &AgentPosition);
  V2D PointToLocalSpace(const V2D &point,
                         const    V2D &AgentHeading,
                          const   V2D &AgentSide,
                          const    V2D &AgentPosition);

};

#endif 