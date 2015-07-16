//------------------------------------------------------------------------
//
//  檔名: baseclass.h
//
//  描述: 物件基類包含
//             public: 
//                     物件座標m_vPos
//                     物件類別m_ObjType
//                     邊界半徑m_dBoundingRadius
//					   物件布林標籤m_bTag
//             private:
//                     識別編號m_ID   
//		  使用BaseClass(int obj_type, V2D pos, double r)初始化
//        須實作Update(), Render()
//  作者: Neo 2015 (iamhahar@gmail.com)
//
//------------------------------------------------------------------------

#ifndef BASECLASS_H
#define BASECLASS_H
#include <cassert>
#include <windows.h>
#include "stdio.h"
#include "V2D.h"
class BaseClass
{
private:
	int m_ID;
	V2D m_vPos;
	int m_ObjType;
	double m_dBoundingRadius;
	//當有別的物體進入m_dBoundingRadius範圍內, m_bTag設為true
	bool m_bTag;
	//static成員同一個類別只會有一份
	//static int  m_iNextValidID;
	//每次執行NextValidID() NextID會加1
	int NextValidID(){static int NextID = 0; return NextID++;}
	
public:
	 BaseClass(int obj_type, V2D pos, double r):
                                        m_ID(NextValidID()),
                                        m_vPos(pos),
                                        m_ObjType(obj_type),
                                        m_dBoundingRadius(r),
                                        m_bTag(false)                                      
  {}
	//BaseClass() {};
	virtual ~BaseClass(){};
	//update傳入距上次update經過多少秒了
	virtual void Update(double time_elapsed){}; 
	virtual void Render(){};
    //virtual bool HandleMessage(const Telegram& msg){return false;}
  	//virtual void Write(std::ostream&  os)const{}
    //virtual void Read (std::ifstream& is){} 
	int ID(void)const{return m_ID;};
	//設定座標
	V2D Pos()const{return m_vPos;}
    void SetPos(V2D new_pos){m_vPos = new_pos;}
	//設定tag
	bool IsTagged()const{return m_bTag;}
    void Tag(){m_bTag = true;}
    void UnTag(){m_bTag = false;}
    //設定邊界半徑
    double BRadius()const{return m_dBoundingRadius;}
    void SetBRadius(double r){m_dBoundingRadius = r;} 

    void SetType(int v){m_ObjType = v;}
    int Type()const{return m_ObjType;}
};
#endif