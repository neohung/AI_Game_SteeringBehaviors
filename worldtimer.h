#ifndef WORLDTIMER_H
#define WORLDTIMER_H

#include <windows.h>
#include <cassert>


class WorldTimer
{
private:
  //設定每秒frame數的fps
  double m_NormalFPS;
  //m_FrameTime為每個frame的ticks數
  LONGLONG m_FrameTime;
  //m_TimeScale為每次tick的時間長度
  double m_TimeScale;
  double m_TimeElapsed;
  double m_LastTimeElapsed;

  LONGLONG m_PerfCountFreq;
  LONGLONG m_LastTime;
  LONGLONG m_NextTime;
  LONGLONG m_StartTime;
  LONGLONG m_CurrentTime;
  LONGLONG m_LastTimeInTimeElapsed;

  bool m_bSmoothUpdates;
  bool m_bStarted;
public:
  WorldTimer();
  WorldTimer(double fps);
  void    Start();
  //inline
  double TimeElapsed();
  double  CurrentTime();
  bool    Started()const{return m_bStarted;}
  void    SmoothUpdatesOn(){m_bSmoothUpdates = true;}
  void    SmoothUpdatesOff(){m_bSmoothUpdates = false;}

};
#endif