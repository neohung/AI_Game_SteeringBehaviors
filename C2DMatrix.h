//------------------------------------------------------------------------
//
//  檔名: C2DMTRIX.h
//
//  描述: 計算向量平移,旋轉,縮放
//
//  作者: Neo 2015 (iamhahar@gmail.com)
//
//------------------------------------------------------------------------

#ifndef C2DMATRIX_H
#define C2DMATRIX_H

#include <math.h>
#include <vector>
#include "V2D.h"
class C2DMatrix
{
private: 
  struct Matrix
  {
  	//定義3x3運算用矩陣
    double _11, _12, _13;
    double _21, _22, _23;
    double _31, _32, _33;
    Matrix()
    {
      _11=0.0; _12=0.0; _13=0.0;
      _21=0.0; _22=0.0; _23=0.0;
      _31=0.0; _32=0.0; _33=0.0;
    }
  };
  //m_Matrix為實際運算用3x3矩陣
  Matrix m_Matrix;
  //矩陣乘法
  inline void  MatrixMultiply(Matrix &mIn);
public:
  C2DMatrix()
  {
  	//初始化單位矩陣
    Identity();
  }
  ~C2DMatrix(){}
  inline void Identity();
  inline void Translate(double x, double y);
  inline void Scale(double xScale, double yScale);
  inline void Rotate(const V2D &fwd, const V2D &side);
  inline void Rotate(double rot);
  inline void Rotate(const V2D &fwd);
  inline void TransformVector2Ds(std::vector<V2D> &vPoints);
  inline void TransformVector2Ds(V2D &vPoint);
  void _11(double v){m_Matrix._11=v;}
  void _12(double v){m_Matrix._12=v;}
  void _13(double v){m_Matrix._13=v;}
  void _21(double v){m_Matrix._21=v;}
  void _22(double v){m_Matrix._22=v;}
  void _23(double v){m_Matrix._23=v;}
  void _31(double v){m_Matrix._31=v;}
  void _32(double v){m_Matrix._32=v;}
  void _33(double v){m_Matrix._33=v;}
  
};

inline void C2DMatrix::Identity()
{
  m_Matrix._11 = 1; m_Matrix._12 = 0; m_Matrix._13 = 0;
  m_Matrix._21 = 0; m_Matrix._22 = 1; m_Matrix._23 = 0;
  m_Matrix._31 = 0; m_Matrix._32 = 0; m_Matrix._33 = 1;
}

//矩陣右乘法:
//|m11 m12 m13|    |mIn11 mIn12 mIn13|    | M11 M12 M13|
//|m21 m22 m23|  X |mIn21 mIn22 mIn23| =  | M21 M22 M23|
//|m31 m32 m33|    |mIn31 mIn33 mIn33|    | M22 M32 M33|
inline void C2DMatrix::MatrixMultiply(Matrix &mIn)
{
 C2DMatrix::Matrix mat_temp;
  //第一列
  mat_temp._11 = (m_Matrix._11*mIn._11) + (m_Matrix._12*mIn._21) + (m_Matrix._13*mIn._31);
  mat_temp._12 = (m_Matrix._11*mIn._12) + (m_Matrix._12*mIn._22) + (m_Matrix._13*mIn._32);
  mat_temp._13 = (m_Matrix._11*mIn._13) + (m_Matrix._12*mIn._23) + (m_Matrix._13*mIn._33);
  //第二列
  mat_temp._21 = (m_Matrix._21*mIn._11) + (m_Matrix._22*mIn._21) + (m_Matrix._23*mIn._31);
  mat_temp._22 = (m_Matrix._21*mIn._12) + (m_Matrix._22*mIn._22) + (m_Matrix._23*mIn._32);
  mat_temp._23 = (m_Matrix._21*mIn._13) + (m_Matrix._22*mIn._23) + (m_Matrix._23*mIn._33);
  //第三列
  mat_temp._31 = (m_Matrix._31*mIn._11) + (m_Matrix._32*mIn._21) + (m_Matrix._33*mIn._31);
  mat_temp._32 = (m_Matrix._31*mIn._12) + (m_Matrix._32*mIn._22) + (m_Matrix._33*mIn._32);
  mat_temp._33 = (m_Matrix._31*mIn._13) + (m_Matrix._32*mIn._23) + (m_Matrix._33*mIn._33);
  m_Matrix = mat_temp;
}

inline void C2DMatrix::TransformVector2Ds(V2D &vPoint)
{
  double tempX =(m_Matrix._11*vPoint.x) + (m_Matrix._21*vPoint.y) + (m_Matrix._31);
  double tempY = (m_Matrix._12*vPoint.x) + (m_Matrix._22*vPoint.y) + (m_Matrix._32);  
  vPoint.x = tempX;
  vPoint.y = tempY;
}

//矩陣變換
//                 |m11 m12 m13|      
//     |x y 1|  X  |m21 m22 m23|   =  |X Y ...|: 
//                 |m31 m32 m33|   
inline void C2DMatrix::TransformVector2Ds(std::vector<V2D> &vPoint)
{
  for (unsigned int i=0; i<vPoint.size(); ++i)
  {
    double tempX =(m_Matrix._11*vPoint[i].x) + (m_Matrix._21*vPoint[i].y) + (m_Matrix._31);

    double tempY = (m_Matrix._12*vPoint[i].x) + (m_Matrix._22*vPoint[i].y) + (m_Matrix._32);
  
    vPoint[i].x = tempX;

    vPoint[i].y = tempY;

  }
}

inline void C2DMatrix::Translate(double x, double y)
{
	
  Matrix mat;
  
  mat._11 = 1; mat._12 = 0; mat._13 = 0;
  
  mat._21 = 0; mat._22 = 1; mat._23 = 0;
  
  mat._31 = x;    mat._32 = y;    mat._33 = 1;
  
  //and multiply
  MatrixMultiply(mat);

}

//create a scale matrix
inline void C2DMatrix::Scale(double xScale, double yScale)
{
  C2DMatrix::Matrix mat;
  mat._11 = xScale; mat._12 = 0; mat._13 = 0;
  mat._21 = 0; mat._22 = yScale; mat._23 = 0;
  mat._31 = 0; mat._32 = 0; mat._33 = 1;
  //and multiply
  MatrixMultiply(mat);
}

inline void C2DMatrix::Rotate(const V2D &fwd, const V2D &side)
{
  C2DMatrix::Matrix mat;  
  mat._11 = fwd.x;  mat._12 = fwd.y; mat._13 = 0;
  mat._21 = side.x; mat._22 = side.y; mat._23 = 0;
  mat._31 = 0; mat._32 = 0;mat._33 = 1;
  //and multiply
  MatrixMultiply(mat);
}

inline void C2DMatrix::Rotate(const V2D &fwd)
{
  C2DMatrix::Matrix mat;  
  mat._11 = fwd.x;  mat._12 = -fwd.y; mat._13 = 0;
  mat._21 = fwd.y; mat._22 = fwd.x; mat._23 = 0;
  mat._31 = 0; mat._32 = 0;mat._33 = 1;
  //and multiply
  MatrixMultiply(mat);
}

inline void C2DMatrix::Rotate(double rot)
{
  C2DMatrix::Matrix mat;
  double Sin = sin(rot);
  double Cos = cos(rot);
  mat._11 = Cos;  mat._12 = -Sin; mat._13 = 0;
  mat._21 = Sin; mat._22 = Cos; mat._23 = 0;
  mat._31 = 0; mat._32 = 0;mat._33 = 1; 
  //and multiply
  MatrixMultiply(mat);
}

#endif