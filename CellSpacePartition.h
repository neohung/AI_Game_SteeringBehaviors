#ifndef CELLSPACEPARTITION_H
#define CELLSPACEPARTITION_H
#include <list>
#include "V2D.h"
#include "neogdi.h"
//-----------------------------------------------------------
class InvertedAABBox2D
{
private:
  V2D  m_vTopLeft;
  V2D  m_vBottomRight;
  V2D  m_vCenter;
public:
  InvertedAABBox2D(V2D tl,
                   V2D br):     m_vTopLeft(tl),
                                m_vBottomRight(br),
                                m_vCenter((tl+br)/2.0)
  {}
  V2D TopLeft()const{return m_vTopLeft;}
  V2D BottomRight()const{return m_vBottomRight;}
  double    Top()const{return m_vTopLeft.y;}
  double    Left()const{return m_vTopLeft.x;}
  double    Bottom()const{return m_vBottomRight.y;}
  double    Right()const{return m_vBottomRight.x;}
  V2D Center()const{return m_vCenter;}
  //如果Box跟Box重疊
  bool isOverlappedWith(const InvertedAABBox2D& other)const
  {
    return !((other.Top() > this->Bottom()) ||
           (other.Bottom() < this->Top()) ||
           (other.Left() > this->Right()) ||
           (other.Right() < this->Left()));
  }
  void     Render(bool RenderCenter = false)const
  {
    ngdi->Line(V2D(Left(), Top()), V2D(Right(), Top()));
    ngdi->Line(V2D(Left(), Bottom()), V2D(Right(), Bottom()));
    ngdi->Line(V2D(Left(), Top()), V2D(Left(), Bottom()));
    ngdi->Line(V2D(Right(), Top()), V2D(Right(), Bottom()));
    //if (RenderCenter)
    //{
    //  ngdi->Circle(m_vCenter, 5);
    //}
  }
};
//-----------------------------------------------------------
template <class object>
  struct Cell
  {
    std::list<object>    Members;
    InvertedAABBox2D     BBox;
    Cell(V2D topleft,
         V2D botright):  BBox(InvertedAABBox2D(topleft, botright))
    {}
  };
//-----------------------------------------------------------
template <class object>
class CellSpacePartition
{
private:
  //m_Cells表示有n個Cell, 一個Cell表示一個方格, Cell裡面有Members結構存放成員
  std::vector< Cell<object> > m_Cells;
  std::vector<object> m_Neighbors;
  typename std::vector<object>::iterator m_curNeighbor;
  double  m_dSpaceWidth;
  double  m_dSpaceHeight;
  int     m_iNumCellsX;
  int     m_iNumCellsY;
  double  m_dCellSizeX;
  double  m_dCellSizeY;
  inline int PositionToIndex(const V2D& pos)const;
public:
 CellSpacePartition( double width,        //width of the environment
                     double height,       //height ...
                     int   cellsX,       //number of cells horizontally
                     int   cellsY,       //number of cells vertically
                     int   MaxEntitys);  //maximum number of entities to add
 //指向m_Neighbors開頭
 inline object& begin(){m_curNeighbor = m_Neighbors.begin(); return *m_curNeighbor;}
 //取出m_Neighbors中下一個物件, 即當前的鄰近物體
 inline object& next(){++m_curNeighbor; return *m_curNeighbor;}
 inline bool    end(){return (m_curNeighbor == m_Neighbors.end()) || (*m_curNeighbor == 0);}   
  
 inline void RenderCells();
 void EmptyCells();
 inline void AddObject(const object& ent);
 inline void UpdateObject(const object& ent, V2D OldPos);
void CalculateNeighbors(V2D TargetPos, double QueryRadius);
};
//-----------------------------------
template<class object>
CellSpacePartition<object>::CellSpacePartition(double  width,        //width of 2D space
                                               double  height,       //height...
                                               int    cellsX,       //number of divisions horizontally
                                               int    cellsY,       //and vertically
                                               int    MaxEntitys):  //maximum number of entities to partition
                  m_Neighbors(MaxEntitys, object()),
                  m_dSpaceWidth(width),
                  m_dSpaceHeight(height),
                  m_iNumCellsX(cellsX),
                  m_iNumCellsY(cellsY)
{
  //calculate bounds of each cell
  m_dCellSizeX = width  / cellsX;
  m_dCellSizeY = height / cellsY;

  for (int y=0; y<m_iNumCellsY; ++y)
  {
    for (int x=0; x<m_iNumCellsX; ++x)
    {
      double left  = x * m_dCellSizeX;
      double right = left + m_dCellSizeX;
      double top   = y * m_dCellSizeY;
      double bot   = top + m_dCellSizeY;
      m_Cells.push_back(Cell<object>(V2D(left, top), V2D(right, bot)));
    }
  }
}
template<class object>
inline int CellSpacePartition<object>::PositionToIndex(const V2D& pos)const
{
  int idx = (int)(m_iNumCellsX * pos.x / m_dSpaceWidth) + 
            ((int)((m_iNumCellsY) * pos.y / m_dSpaceHeight) * m_iNumCellsX);

  if (idx > (int)m_Cells.size()-1) idx = (int)m_Cells.size()-1;
  return idx;
}

template <class T>
inline std::basic_string<wchar_t> neottos(const T& t, int precision = 2)
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

template<class object>
inline void CellSpacePartition<object>::RenderCells()
{
 typename std::vector< Cell<object> >::const_iterator curCell;
  for (curCell=m_Cells.begin(); curCell!=m_Cells.end(); ++curCell)
  {
  	//
  	//int i = PositionToIndex(curCell->BBox.Center());
  	//ngdi->TextColor(NEOgdi::yellow);
    //ngdi->TextAtPos(curCell->BBox.Center().x-3, curCell->BBox.Center().y-3, neottos(i));
    curCell->BBox.Render(false);
  }
}

template<class object>
void CellSpacePartition<object>::EmptyCells()
{
  typename std::vector<Cell<object> >::iterator it;
  for (it= m_Cells.begin(); it!=m_Cells.end(); ++it)
  {
    it->Members.clear();
  }
}

template<class object>
inline void CellSpacePartition<object>::AddObject(const object& ent)
{ 
  assert (ent);
  //int sz = m_Cells.size();
  int idx = PositionToIndex(ent->Pos());
  m_Cells[idx].Members.push_back(ent);
}

template<class object>
inline void CellSpacePartition<object>::UpdateObject(const object&  ent,
                                                     V2D       OldPos)
{
  int OldIdx = PositionToIndex(OldPos);
  int NewIdx = PositionToIndex(ent->Pos());
  if (NewIdx == OldIdx) return;
  m_Cells[OldIdx].Members.remove(ent);
  m_Cells[NewIdx].Members.push_back(ent);
}

template<class object>
void CellSpacePartition<object>::CalculateNeighbors(V2D TargetPos,
                                                    double   QueryRadius)
{
  //create an iterator and set it to the beginning of the neighbor vector
  //curNbor指向m_Neighbors的開頭, 最後m_Neighbors會存放當前所有的Neighbor
  typename std::vector<object>::iterator curNbor = m_Neighbors.begin();
  //create the query box that is the bounding box of the target's query
  //area
  InvertedAABBox2D QueryBox(TargetPos - V2D(QueryRadius, QueryRadius),
                            TargetPos + V2D(QueryRadius, QueryRadius));
  //iterate through each cell and test to see if its bounding box overlaps
  //with the query box. If it does and it also contains entities then
  //make further proximity tests.
  typename std::vector<Cell<object> >::iterator curCell; 
  //遍歷所有的Cell
  for (curCell=m_Cells.begin(); curCell!=m_Cells.end(); ++curCell)
  {
    //test to see if this cell contains members and if it overlaps the
    //query box
    //如果該Cell跟QueryBox疊到表示是鄰近格子,額且有該格子內有物件的話
    if (curCell->BBox.isOverlappedWith(QueryBox) &&
       !curCell->Members.empty())
    {
      //add any entities found within query radius to the neighbor list
      typename std::list<object>::iterator it;
      //取出該格子內所有的物件
      for (it=curCell->Members.begin(); it!=curCell->Members.end(); ++it)
      {     
        if (V2DDistanceSq((*it)->Pos(), TargetPos) <
            QueryRadius*QueryRadius)
        {
          *curNbor++ = *it;
        }
      }    
    }
  }//next cell
  //mark the end of the list with a zero.
  //最後在定義m_Neighbors.end()的位置
  *curNbor = 0;
}


#endif