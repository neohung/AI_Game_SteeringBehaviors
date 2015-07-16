#ifndef V2D_H
#define V2D_H
#include <cassert>
#include <vector>
#include <limits> //std::numeric_limits
#include <math.h> //fabs()
//----------------------------------------------------------------
//const int     MaxInt    = (std::numeric_limits<int>::max)();
const double  MaxDouble = (std::numeric_limits<double>::max)();
const double  MinDouble = (std::numeric_limits<double>::min)();
//const float   MaxFloat  = (std::numeric_limits<float>::max)();
//const float   MinFloat  = (std::numeric_limits<float>::min)();
const double   Pi        = 3.14159;
const double   TwoPi     = Pi * 2;
const double   HalfPi    = Pi / 2;
const double   QuarterPi = Pi / 4;
//當val為NaN(不是一個數)時傳回True
template <typename T>
inline bool isNaN(T val)
{
  return val != val;
}
inline double DegsToRads(double degs)
{
  return TwoPi * (degs/360.0);
}
//當val非常小時視為0
inline bool IsZero(double val)
{
  return ( (-MinDouble < val) && (val < MinDouble) );
}
//Sigmoid函數y(t)=[1/(1+e^-t)],其中t=(input/response)
inline double Sigmoid(double input, double response = 1.0)
{
	return ( 1.0 / ( 1.0 + exp(-input / response)));
}
//返回兩者當中最大的
template <class T>
inline T MaxOf(const T& a, const T& b)
{
  if (a>b) return a; return b;
}
template <class T>
inline T MinOf(const T& a, const T& b)
{
  if (a<b) return a; return b;
}
//值減去不大於值的最近整數如果小於0.5則回傳其整數值
inline int Rounded(double val)
{
  int    integral = (int)val;
  double mantissa = val - integral;
  if (mantissa < 0.5)
  {
    return integral;
  }
  else
  {
    return integral + 1;
  }
}
inline bool isEqual(float a, float b)
{
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }
  return false;
}
inline bool isEqual(double a, double b)
{
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }
  return false;
}
//給定vector算平均值
template <class T>
inline double Average(const std::vector<T>& v)
{
  double average = 0.0;
  for (unsigned int i=0; i < v.size(); ++i)
  {    
    average += (double)v[i];
  }
  return average / (double)v.size();
}
//給定vector算標準差
inline double StandardDeviation(const std::vector<double>& v)
{
  double sd      = 0.0;
  double average = Average(v);
  for (unsigned int i=0; i<v.size(); ++i)
  {     
    sd += (v[i] - average) * (v[i] - average);
  }
  sd = sd / v.size();
  return sqrt(sd);
}
//隨機產生x~y的整數值
inline int RandInt(int x,int y)
{
  assert(y>=x && "<RandInt>: y is less than x");
  return rand()%(y-x+1)+x;
}
//隨機產生0~1的值
inline double RandFloat(){return ((rand())/(RAND_MAX+1.0));}
//隨機產生x~y的浮點數值
inline double RandInRange(double x, double y)
{
  return x + RandFloat()*(y-x);
}
//隨機產生-1~1的值
inline double RandomClamped()
{
  //隨機產生0~1的值
    double random_float1 = ((rand())/(RAND_MAX+1.0));
    double random_float2 = ((rand())/(RAND_MAX+1.0));
    //互減產生-1~1的值
    double random_clamped = random_float1 - random_float2;
    return random_clamped;
}

//----------------------------------------------------------------
struct V2D
{
  double x;
  double y;
  V2D():x(0.0),y(0.0){}
  V2D(double a, double b):x(a),y(b){}
  //歸零
  void Zero(){x=0.0; y=0.0;}
  //x,y長度小於MinDouble則視為0
  bool isZero()const{return (x*x + y*y) < MinDouble;}
  //求出長度的平方,避免開根號
  inline double LengthSq()const
  {
    return (x * x + y * y);
  }
  inline double Length()const
  {
    return sqrt(x * x + y * y);
  }
  //傳回與原向量垂直的向量
  inline V2D Perp()const
  {
    return V2D(-y, x);
  }
  inline void Normalize()
  { 
    double vector_length = this->Length();
    if (vector_length > std::numeric_limits<double>::epsilon())
    {
      this->x /= vector_length;
      this->y /= vector_length;
    }
  }
  inline void Truncate(double max)
  {
    if (this->Length() > max)
    {
      this->Normalize();
      *this *= max;
    } 
  }
  //向量內積, |v1|內積|v2|=|v1|*|v2|cos(角度),所以|v1|,|v2|長度都為1時
  //內積的值可由acos(|v1|內積|v2|)求出角度
  inline double Dot(const V2D &v2)const
  {
    return x*v2.x + y*v2.y;
  }
  //定義 (V2D)lhs += (V2D)rhs
  const V2D& operator+=(const V2D &rhs)
  {
    x += rhs.x;
    y += rhs.y;

    return *this;
  }
  //定義 (V2D)lhs -= (V2D)rhs
  const V2D& operator-=(const V2D &rhs)
  {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  //定義乘法 (V2D)lhs *= (double)rhs
  const V2D& operator*=(const double& rhs)
  {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  //定義 (V2D)lhs /= (double)rhs
  const V2D& operator/=(const double& rhs)
  {
    x /= rhs;
    y /= rhs;
    return *this;
  }
  // 定義 自己==(V2D)rhs
  bool operator==(const V2D& rhs)const
  {
    //當自己的x跟rhs.x互減小於1E-12時判定相等,y亦同
    return (isEqual(x, rhs.x) && isEqual(y,rhs.y) );
  }
  bool operator!=(const V2D& rhs)const
  {
    return (x != rhs.x) || (y != rhs.y);
  }
};

//定義(V2D)lhs*(double)rhs
inline V2D operator*(const V2D &lhs, double rhs)
{
  V2D result(lhs);
  //結構中定義了*=運算子
  result *= rhs;
  return result;
}
//(double)lhs*(V2D)rhs
inline V2D operator*(double lhs, const V2D &rhs)
{
  V2D result(rhs);
  result *= lhs;
  return result;
}

//(V2D)lhs-(V2D)rhs
inline V2D operator-(const V2D &lhs, const V2D &rhs)
{
  V2D result(lhs);
  result.x -= rhs.x;
  result.y -= rhs.y;
  
  return result;
}

//(V2D)lhs+(V2D)rhs
inline V2D operator+(const V2D &lhs, const V2D &rhs)
{
  V2D result(lhs);
  result.x += rhs.x;
  result.y += rhs.y;
  
  return result;
}

//(V2D)lhs/(double)rhs
inline V2D operator/(const V2D &lhs, double val)
{
  V2D result(lhs);
  result.x /= val;
  result.y /= val;
  return result;
}
/*
#include <fstream>
std::ostream& operator<<(std::ostream& os, const V2D& rhs)
{
  os << " " << rhs.x << " " << rhs.y;
  return os;
}
std::ifstream& operator>>(std::ifstream& is, V2D& lhs)
{
  is >> lhs.x >> lhs.y;
  return is;
}
*/

inline V2D V2DNormalize(const V2D &v)
{
  V2D vec = v;
  double vector_length = vec.Length();
  //如果vector_length比電腦能運算的最小數來的大
  if (vector_length > std::numeric_limits<double>::epsilon())
  {
    vec.x /= vector_length;
    vec.y /= vector_length;
  }
  return vec;
}

inline double V2DDistance(const V2D &v1,const V2D &v2)
{
  return sqrt((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y));
}

inline double V2DDistanceSq(const V2D &v1,const V2D &v2)
{
  return (v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y);
}

inline bool TwoCirclesOverlapped(V2D c1, double r1,
                                 V2D c2, double r2)
{
  //計算兩個圓心相距距離
  double DistBetweenCenters = sqrt( (c1.x-c2.x) * (c1.x-c2.x) +
                                    (c1.y-c2.y) * (c1.y-c2.y));
  //小於半徑和則表示Overlapped
  if ((DistBetweenCenters < (r1+r2)) || (DistBetweenCenters < fabs(r1-r2)))
  {
    return true;
  }
  return false;
}
//兩個物件計算是否加了MinDistBetweenObstacles後Overlapped
template <class T, class conT>
bool Overlapped(const T* ob, const conT& conOb, double MinDistBetweenObstacles = 40.0)
{
  typename conT::const_iterator it;

  for (it=conOb.begin(); it != conOb.end(); ++it)
  {
    if (TwoCirclesOverlapped(ob->Pos(),
                             ob->BRadius()+MinDistBetweenObstacles,                             
                             (*it)->Pos(),
                             (*it)->BRadius()))
    {
      return true;
    }
  }
  return false;
}

//class BaseClass;
//如果ContainerOfEntities內的物件m_dBoundingRadius+radius的範圍大於和entity的兩者距離
//則打上tag
template <class T, class conT>
void TagNeighbors(T& entity, conT& ContainerOfEntities, double radius)
{
  //因為conT是一個Vector物件所以要用"typename conT::iterator"來取出真正放入vector的類別名稱
  for (typename conT::iterator curEntity = ContainerOfEntities.begin();
       curEntity != ContainerOfEntities.end();
       ++curEntity)
  {
    //先清除每個ContainerOfEntities的tag
    (*curEntity)->UnTag();
    //得到to "兩者距離"
    V2D to = (*curEntity)->Pos() - entity->Pos();

    //求出range "每個ContainerOfEntities內物件的m_dBoundingRadius加上radius
    double range = radius + (*curEntity)->BRadius();
    //如果兩個物件直線距離小於range表示在本體附近
    if ( ((*curEntity) != entity) && (to.LengthSq() < range*range))
    {
      (*curEntity)->Tag();
    }
  }//next entity
}

//計算線段AB跟線段CD是否相交
inline bool LineIntersection2D(V2D   A,
                               V2D   B,
                               V2D   C, 
                               V2D   D,
                               double&     dist,
                               V2D&  point)
{
  double rTop = (A.y-C.y)*(D.x-C.x)-(A.x-C.x)*(D.y-C.y);
  double rBot = (B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x);

  double sTop = (A.y-C.y)*(B.x-A.x)-(A.x-C.x)*(B.y-A.y);
  double sBot = (B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x);
  if ( (rBot == 0) || (sBot == 0))
  {
    //lines are parallel
    return false;
  }
  double r = rTop/rBot;
  double s = sTop/sBot;
  if( (r > 0) && (r < 1) && (s > 0) && (s < 1) )
  {
    dist = V2DDistance(A,B) * r;
    point = A + r * (B - A);
    return true;
  }
  else
  {
    dist = 0;
    return false;
  }
}

#include "C2DMatrix.h"
inline void V2DRotateAroundOrigin(V2D& v, double ang)
{
  C2DMatrix mat;
  mat.Rotate(ang);
  mat.TransformVector2Ds(v);
}

#endif