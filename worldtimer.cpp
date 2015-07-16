#include "worldtimer.h"
#include "stdio.h"
WorldTimer::WorldTimer(): 
                  m_bSmoothUpdates(false)
{
  //how many ticks per sec do we get
  //求出每秒的ticks數: m_PerfCountFreq
  QueryPerformanceFrequency( (LARGE_INTEGER*) &m_PerfCountFreq);
  m_TimeScale = 1.0/m_PerfCountFreq;
  //wprintf(L"m_FrameTime: %e\n",m_FrameTime);
}

WorldTimer::WorldTimer(double fps): m_NormalFPS(fps),
                  m_bSmoothUpdates(false)
{
  //how many ticks per sec do we get
  //求出每秒的ticks數: m_PerfCountFreq
  QueryPerformanceFrequency( (LARGE_INTEGER*) &m_PerfCountFreq);
  m_TimeScale = 1.0/m_PerfCountFreq;
  m_FrameTime = (LONGLONG)(m_PerfCountFreq / m_NormalFPS);
  //wprintf(L"m_FrameTime: %e\n",m_FrameTime);
}

//m_StartTime: 紀錄開始時的tick數
//m_LastTime:  紀錄上個frame時的tick數
//m_NextTime:  預估達到下個frame的tick數
void WorldTimer::Start()
{
  m_bStarted = true;
  m_TimeElapsed = 0.0;
  //get the time
  QueryPerformanceCounter( (LARGE_INTEGER*) &m_LastTime);
  //keep a record of when the timer was started
  m_StartTime = m_LastTimeInTimeElapsed = m_LastTime;
  //update time to render next frame
  m_NextTime = m_LastTime + m_FrameTime;
  return;
}

//m_CurrentTime:           每次執行WorldTimer::TimeElapsed()得到的當前tick數
//m_LastTimeInTimeElapsed: 上次執行WorldTimer::TimeElapsed()得到的上次tick數
//m_LastTimeElapsed        上上次執行到上次執行中間經過的時間,單位秒
//m_TimeElapsed:           上次執行到這次執行中間經過的時間,單位秒
//m_TimeScale:             一個tick的時間長度,單位秒

double WorldTimer::TimeElapsed()
{
  m_LastTimeElapsed = m_TimeElapsed;
  QueryPerformanceCounter( (LARGE_INTEGER*) &m_CurrentTime);  
  m_TimeElapsed = (m_CurrentTime - m_LastTimeInTimeElapsed) * m_TimeScale;
  m_LastTimeInTimeElapsed    = m_CurrentTime;
  //wprintf(L"m_TimeElapsed: %f\n",m_TimeElapsed);
  const double Smoothness = 5.0;
  if (m_bSmoothUpdates)
  {
    if (m_TimeElapsed < (m_LastTimeElapsed * Smoothness))
    {
      return m_TimeElapsed;
    }
    else
    {
      //當m_TimeElapsed大時5倍的之前執行時間時,傳回0
      return 0.0;
    }
  }
  else
  {
    return m_TimeElapsed;
  } 
}
