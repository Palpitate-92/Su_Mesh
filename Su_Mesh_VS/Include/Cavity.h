/*
 * 声明空腔操作类，包括空腔查找、空腔修复等
 */

// #pragma once
#ifndef _CAVITY_H
#define _CAVITY_H

#include <vector>
#include "data.h"

class _CAVITY
{
public:
  // 空腔查找，返回true代表空腔查找成功，返回false代表当前节点与当前三角化的网格单元存在5点共球情况，需要延迟插入该节点
  bool Find_Cavity(_SU_MESH *su_mesh, std::vector<int> *elemNum_Cavity, NODE node_tp, int elemNum_IncludeNodeIns);
  // 空腔修复
  void RepairCavity(_SU_MESH *su_mesh, NODE node_Insert, std::vector<int> *elemNum_Cavity, std::vector<FACE> *face_Cavity);
  // 空腔初始化操作，在elem内中初始化空腔所包含的网格单元
  void Initialize_Cavity(_SU_MESH *su_mesh, std::vector<int> elemNum_Cavity);

private:
};

#endif