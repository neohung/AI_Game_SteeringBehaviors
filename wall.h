#ifndef WALL_H
#define WALL_H
#include "neogdi.h"
#include "v2d.h"

class Wall
{
protected:
  V2D m_vA;
  V2D m_vB;
  V2D m_vN;
  void CalculateNormal()
  {
    V2D temp = V2DNormalize(m_vB - m_vA);

    m_vN.x = -temp.y;
    m_vN.y = temp.x;
  }
public:
  Wall(V2D A, V2D B):m_vA(A), m_vB(B)
  {
    CalculateNormal();
  }
 ~Wall(){}
 virtual void Render(bool RenderNormals = false)const
  {
    ngdi->Line(m_vA, m_vB);

    //render the normals if rqd
    if (RenderNormals)
    {
      int MidX = (int)((m_vA.x+m_vB.x)/2);
      int MidY = (int)((m_vA.y+m_vB.y)/2);
      ngdi->Line(V2D(MidX, MidY),V2D(MidX+(m_vN.x * 5),MidY+(m_vN.y * 5)));
    }
  }
  V2D From()const {return m_vA;}
  void SetFrom(V2D v){m_vA = v; CalculateNormal();}
  V2D To()const {return m_vB;}
  void SetTo(V2D v){m_vB = v; CalculateNormal();} 
  V2D Normal()const{return m_vN;}
  void SetNormal(V2D n){m_vN = n;} 
  V2D Center()const{return (m_vA+m_vB)/2.0;}
};
#endif