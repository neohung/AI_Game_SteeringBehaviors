#ifndef PATH_H
#define PATH_H
#include <list>
#include <cassert>
#include "V2D.h"
#include "neogdi.h"
class Path
{
private:
	//路徑
	std::list<V2D>            m_WayPoints;
	//當前的way point
	std::list<V2D>::iterator  curWaypoint;
	//Loop為true的話,最後的way point連上第一個way point
	bool m_bLooped;

	//
	std::list<V2D> CreateRandomPath(int NumWaypoints,
                                       double MinX,
                                       double MinY,
                                       double MaxX,
                                       double MaxY)
    {
      m_WayPoints.clear();
      double midX = (MaxX+MinX)/2.0;
      double midY = (MaxY+MinY)/2.0;
      double smaller = MinOf(midX, midY);
      //把360度分成NumWaypoints等份
      double spacing = TwoPi/(double)NumWaypoints;
      for (int i=0; i<NumWaypoints; ++i)
      {
      	double RadialDist = RandInRange(smaller*0.2f, smaller);
        //隨機給定一個線段
        V2D temp(RadialDist, 0.0f);
        //每次轉一個角度,NumWaypoints後剛好360度
        V2DRotateAroundOrigin(temp, i*spacing);
        temp.x += midX; temp.y += midY;
        m_WayPoints.push_back(temp);
      }
      //設定curWaypoint為第一個way point
      curWaypoint = m_WayPoints.begin();
      return m_WayPoints;
    }

public:
	Path():m_bLooped(false){}
    Path(int    NumWaypoints,
       double MinX,
       double MinY,
       double MaxX,
       double MaxY,
       bool   looped):m_bLooped(looped)
    {
      CreateRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);
      curWaypoint = m_WayPoints.begin();
    }
    ~Path(){}
    std::list<V2D> GetPath()const{return m_WayPoints;}
    unsigned int GetWayPointSize()const{return m_WayPoints.size();}
    V2D CurrentWaypoint()const{return *curWaypoint;}
    void ReStartPath(){curWaypoint = m_WayPoints.begin();}
    bool Finished(){std::list<V2D>::iterator temp = curWaypoint; return !(++temp != m_WayPoints.end() );}
    void LoopOn(){m_bLooped = true;}
    void LoopOff(){m_bLooped = false;}
    void Clear(){m_WayPoints.clear();}
    //用list設定新的Path
    void Set(std::list<V2D> new_path){m_WayPoints = new_path;curWaypoint = m_WayPoints.begin();}
    //用舊的Path設定新的Path
    void Set(const Path& path){m_WayPoints=path.GetPath(); curWaypoint = m_WayPoints.begin();}
    void AddWayPoint(V2D new_point){m_WayPoints.push_back(new_point);}
    //把curWaypoint設成下一個way point
    void SetNextWaypoint(){
      if (m_WayPoints.size() == 0) return;
      std::list<V2D>::iterator temp = curWaypoint;
      temp++;
      if (temp == m_WayPoints.end())
      {
      	//curWaypoint = m_bLooped ? m_WayPoints.begin(): curWaypoint--;
      	if (m_bLooped){
      		curWaypoint = m_WayPoints.begin();
      	}
      }else{
      	 curWaypoint++;
      }
    }
    void Render()const{
      if (m_WayPoints.size() == 0) return;
  	  ngdi->OrangePen();
      std::list<V2D>::const_iterator it = m_WayPoints.begin();
      V2D wp = *it++;
      while (it != m_WayPoints.end())
      {
        ngdi->Line(wp, *it);
        wp = *it++;
      }
      //if (m_bLooped) ngdi->Line(*(--it), *m_WayPoints.begin());   
    }
};


#endif