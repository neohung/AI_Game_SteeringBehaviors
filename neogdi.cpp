#include "neogdi.h"
#include <cassert>

NEOgdi* NEOgdi::Instance()
{
  static NEOgdi instance;
  return &instance;
}

NEOgdi::NEOgdi()
{
  m_hdc = NULL;
  m_BlackPen = CreatePen(PS_SOLID, 1, colors[black]);
  m_WhitePen = CreatePen(PS_SOLID, 1, colors[white]);
  m_RedPen = CreatePen(PS_SOLID, 1, colors[red]);
  m_GreenPen = CreatePen(PS_SOLID, 1, colors[green]);
  m_BluePen = CreatePen(PS_SOLID, 1, colors[blue]);
  m_GreyPen = CreatePen(PS_SOLID, 1, colors[grey]);
  m_PinkPen = CreatePen(PS_SOLID, 1, colors[pink]);
  m_YellowPen = CreatePen(PS_SOLID, 1, colors[yellow]);
  m_OrangePen = CreatePen(PS_SOLID, 1, colors[orange]);
  m_PurplePen = CreatePen(PS_SOLID, 1, colors[purple]);
  m_BrownPen = CreatePen(PS_SOLID, 1, colors[brown]);  
  m_DarkGreenPen = CreatePen(PS_SOLID, 1, colors[dark_green]);
  m_LightBluePen = CreatePen(PS_SOLID, 1, colors[light_blue]);
  m_LightGreyPen = CreatePen(PS_SOLID, 1, colors[light_grey]);
  m_LightPinkPen = CreatePen(PS_SOLID, 1, colors[light_pink]);

  m_GreenBrush = CreateSolidBrush(colors[green]);
  m_RedBrush   = CreateSolidBrush(colors[red]);
  m_BlueBrush  = CreateSolidBrush(colors[blue]);
  m_GreyBrush  = CreateSolidBrush(colors[grey]);
  m_BrownBrush = CreateSolidBrush(colors[brown]);
  m_YellowBrush = CreateSolidBrush(colors[yellow]);
  m_LightBlueBrush = CreateSolidBrush(RGB(0,255,255));
  m_DarkGreenBrush = CreateSolidBrush(colors[dark_green]);
  m_OrangeBrush = CreateSolidBrush(colors[orange]);

}

NEOgdi::~NEOgdi()
{
  DeleteObject(m_BlackPen);
  DeleteObject(m_WhitePen);
  DeleteObject(m_RedPen);
 
}

void NEOgdi::StartDrawing(HDC hdc)
{
  //把原來的Pen跟Brush存入m_OldPen跟m_OldBrush
  assert(m_hdc == NULL); 
  m_hdc = hdc;
  //get the current pen
  m_OldPen = (HPEN)SelectObject(hdc, m_BlackPen);
  //select it back in
  SelectObject(hdc, m_OldPen);
  m_OldBrush = (HBRUSH)SelectObject(hdc, GetStockObject(BLACK_BRUSH));
  SelectObject(hdc, m_OldBrush);
}

void NEOgdi::StopDrawing(HDC hdc)
{
  //選回原來的的Pen跟Brush
  assert(hdc != NULL);  
  SelectObject(hdc, m_OldPen);
  SelectObject(hdc, m_OldBrush);
  m_hdc = NULL;
}

void NEOgdi::TransparentText()
{
  SetBkMode(m_hdc, TRANSPARENT);
}

void NEOgdi::OpaqueText()
{
  SetBkMode(m_hdc, OPAQUE);
}

void NEOgdi::TextColor(int color)
{
  assert(color < NumColors); 
  SetTextColor(m_hdc, colors[color]);
}

void NEOgdi::TextAtPos(int x,int y, const std::basic_string<wchar_t> &s)
{
  TextOut(m_hdc, x, y, (const wchar_t*)s.c_str(), (int)s.size());
}

void NEOgdi::TextAtPos(V2D pos, const std::basic_string<wchar_t> &s)
{
  TextOut(m_hdc, (int)pos.x, (int)pos.y, (const wchar_t*)s.c_str(), (int)s.size());
}

//畫封閉曲線
void NEOgdi::ClosedShape(const std::vector<V2D> &points)
  {
    //先移動到目標的頂點
    MoveToEx(m_hdc, (int)points[0].x, (int)points[0].y, NULL);
    //依序畫線
    for (unsigned int p=1; p<points.size(); ++p)
    {
      LineTo(m_hdc, (int)points[p].x, (int)points[p].y);
    }
    //封閉曲線
    LineTo(m_hdc, (int)points[0].x, (int)points[0].y);
  }
//畫點
 void NEOgdi::DrawDot(V2D pos, COLORREF color)
  {
    SetPixel(m_hdc, (int)pos.x, (int)pos.y, color);
  }

 void NEOgdi::Line(V2D from, V2D to)
  {
    MoveToEx(m_hdc, (int)from.x, (int)from.y, NULL);
    LineTo(m_hdc, (int)to.x, (int)to.y);
  }

 void NEOgdi::Circle(V2D pos, double radius)
  {
    Ellipse(m_hdc,
           (int)(pos.x-radius),
           (int)(pos.y-radius),
           (int)(pos.x+radius+1),
           (int)(pos.y+radius+1));
  }

  void NEOgdi::Cross(V2D pos, int diameter)
  {
    Line(V2D((int)pos.x-diameter, (int)pos.y-diameter), V2D((int)pos.x+diameter, (int)pos.y+diameter));
    Line(V2D((int)pos.x-diameter,(int)pos.y+diameter), V2D((int)pos.x+diameter, (int)pos.y-diameter));
  }

  void NEOgdi::Rect(V2D lu_pos, V2D rd_pos)
  {
    Rectangle(m_hdc, (int)lu_pos.x, (int)lu_pos.y, (int)rd_pos.x, (int)rd_pos.y);
  }
