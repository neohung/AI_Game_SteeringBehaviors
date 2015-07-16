#ifndef SMOOTHER_H
#define SMOOTHER_H
//-----------------------------------------------------------------------
//Smoother宣告一個vector: m_History存放最大數量SampleSize,定義0向量為ZeroValue
//Smoother->Update()會傳回該vector所有物件的平均向量
template <class T>
class Smoother
{
private:
  //this holds the history
  std::vector<T>  m_History;
    T             m_ZeroValue;
  int           m_iNextUpdateSlot;
  //an example of the 'zero' value of the type to be smoothed. This
  //would be something like Vector2D(0,0)
public:
  Smoother(int SampleSize, T ZeroValue):
                                        m_History(SampleSize, ZeroValue),
                                        m_ZeroValue(ZeroValue),
                                        m_iNextUpdateSlot(0)
  {}
  T Update(const T& MostRecentValue)
  {  
  	
    //overwrite the oldest value with the newest
    m_History[m_iNextUpdateSlot++] = MostRecentValue;
    //make sure m_iNextUpdateSlot wraps around. 
    if (m_iNextUpdateSlot == m_History.size()) m_iNextUpdateSlot = 0;
    //now to calculate the average of the history list
    T sum = m_ZeroValue;
    typename std::vector<T>::iterator it;
    for (it=m_History.begin(); it != m_History.end(); ++it)
    {
      sum += *it;
    }
    return sum / (double)m_History.size();
  }
};

#endif