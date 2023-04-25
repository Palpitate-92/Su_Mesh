/*
 * 声明节点插入具体实现类，包括边界点插入和内部点插入
 */

// #pragma once
#ifndef _INSERT_POINT_H
#define _INSERT_POINT_H

#include "data.h"

class _INSERT_POINT
{
public:
  // 插入节点，并且得到节点是否成功插入的信息
  // 返回1代表成功插入
  // 返回2代表当前插入点与当前三角化的网格单元存在5点共球情况，需要延迟插入该节点
  // 返回3代表依据当前插入点的密度信息，该插入点不该被插入到当前三角化内，取消该点的插入
  int Insert_point(_SU_MESH *su_mesh, int nodeNum_Last_succ, int nodeNum_Insert, NODE node_Insert, int elemNum_Basis);

private:
};

#endif