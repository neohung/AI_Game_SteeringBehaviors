#ifndef NEOGDI_H
#define NEOGDI_H
//COLORREF需include windows.h
#include <windows.h>
#include <string>
#include "V2D.h"
//
//定義15種顏色
const int NumColors = 15;
const COLORREF colors[NumColors] =
{
  RGB(255,0,0),
  RGB(0,0,255),
  RGB(0,255,0),
  RGB(0,0,0),
  RGB(255,200,200),
  RGB(200,200,200),
  RGB(255,255,0),
  RGB(255,170,0),
  RGB(255,0,170),
  RGB(133,90,0),
  RGB(255,255,255),  
  RGB(0, 100, 0),        //dark green
  RGB(0, 255, 255),       //light blue
  RGB(200, 200, 200),     //light grey
  RGB(255, 230, 230)      //light pink
};

//
#define ngdi NEOgdi::Instance()

class NEOgdi
{
private:
  HDC    m_hdc;
  HPEN m_OldPen;
  HBRUSH m_OldBrush;
  //all the pens
  HPEN   m_BlackPen;
  HPEN   m_WhitePen;
  HPEN   m_RedPen;
  HPEN   m_GreenPen;
  HPEN   m_BluePen;
  HPEN   m_GreyPen;
  HPEN   m_PinkPen;
  HPEN   m_OrangePen;
  HPEN   m_YellowPen;
  HPEN   m_PurplePen;
  HPEN   m_BrownPen;
  HPEN   m_DarkGreenPen;
  HPEN   m_LightBluePen;
  HPEN   m_LightGreyPen;
  HPEN   m_LightPinkPen;

  HBRUSH  m_RedBrush;
  HBRUSH  m_GreenBrush;
  HBRUSH  m_BlueBrush;
  HBRUSH  m_GreyBrush;
  HBRUSH  m_BrownBrush;
  HBRUSH  m_YellowBrush;
  HBRUSH  m_OrangeBrush;
  HBRUSH  m_LightBlueBrush;
  HBRUSH  m_DarkGreenBrush;

  NEOgdi();	
  //copy ctor and assignment should be private
  NEOgdi(const NEOgdi&);
  NEOgdi& operator=(const NEOgdi&);
public:
  enum
  {
    red,
    blue, 
    green,
    black,
    pink,
    grey,
    yellow,
    orange,
    purple,
    brown,   
    white,
    dark_green,
    light_blue,
    light_grey,
    light_pink,
    hollow
  };
  ~NEOgdi();
  static NEOgdi* Instance();
  void StartDrawing(HDC hdc);
  void StopDrawing(HDC hdc);
  void TextColor(int color);
  //
  //
  //void TextAtPos(POINTS pos, const std::string &s);
  void TransparentText();
  void OpaqueText();
  void TextAtPos(V2D pos, const std::basic_string<wchar_t> &s);
  void TextAtPos(int x,int y, const std::basic_string<wchar_t> &s);
  void ClosedShape(const std::vector<V2D> &points);
  void DrawDot(V2D pos, COLORREF color);
  void Line(V2D from, V2D to);
  void Circle(V2D pos, double radius);
  void Cross(V2D pos, int diameter);
  void Rect(V2D lu_pos, V2D rd_pos);
  
  void BlackPen(){if(m_hdc){SelectObject(m_hdc, m_BlackPen);}}
  void WhitePen(){if(m_hdc){SelectObject(m_hdc, m_WhitePen);}}
  void RedPen()  {if(m_hdc){SelectObject(m_hdc, m_RedPen);}}
  void GreenPen(){if(m_hdc){SelectObject(m_hdc, m_GreenPen);}}
  void BluePen() {if(m_hdc){SelectObject(m_hdc, m_BluePen);}}
  void GreyPen() {if(m_hdc){SelectObject(m_hdc, m_GreyPen);}}
  void PinkPen() {if(m_hdc){SelectObject(m_hdc, m_PinkPen);}}
  void YellowPen() {if(m_hdc){SelectObject(m_hdc, m_YellowPen);}}
  void OrangePen() {if(m_hdc){SelectObject(m_hdc, m_OrangePen);}}
  void PurplePen() {if(m_hdc){SelectObject(m_hdc, m_PurplePen);}}
  void BrownPen() {if(m_hdc){SelectObject(m_hdc, m_BrownPen);}}
  
  void DarkGreenPen() {if(m_hdc){SelectObject(m_hdc, m_DarkGreenPen);}}
  void LightBluePen() {if(m_hdc){SelectObject(m_hdc, m_LightBluePen);}}
  void LightGreyPen() {if(m_hdc){SelectObject(m_hdc, m_LightGreyPen);}}
  void LightPinkPen() {if(m_hdc){SelectObject(m_hdc, m_LightPinkPen);}}
 
  void BlackBrush(){if(m_hdc)SelectObject(m_hdc, GetStockObject(BLACK_BRUSH));}
  void WhiteBrush(){if(m_hdc)SelectObject(m_hdc, GetStockObject(WHITE_BRUSH));} 
  //Hollow透明填色的刷子
  void HollowBrush(){if(m_hdc)SelectObject(m_hdc, GetStockObject(HOLLOW_BRUSH));}
  void GreenBrush(){if(m_hdc)SelectObject(m_hdc, m_GreenBrush);}
  void RedBrush()  {if(m_hdc)SelectObject(m_hdc, m_RedBrush);}
  void BlueBrush()  {if(m_hdc)SelectObject(m_hdc, m_BlueBrush);}
  void GreyBrush()  {if(m_hdc)SelectObject(m_hdc, m_GreyBrush);}
  void BrownBrush() {if(m_hdc)SelectObject(m_hdc, m_BrownBrush);}
  void YellowBrush() {if(m_hdc)SelectObject(m_hdc, m_YellowBrush);}
  void LightBlueBrush() {if(m_hdc)SelectObject(m_hdc, m_LightBlueBrush);}
  void DarkGreenBrush() {if(m_hdc)SelectObject(m_hdc, m_DarkGreenBrush);}
  void OrangeBrush() {if(m_hdc)SelectObject(m_hdc, m_OrangeBrush);}

};

#endif