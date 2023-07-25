/*
 * 声明内部点操作内，包含内部点生成、内部点自动插入等
 */

// #pragma once
#ifndef _INTERIOR_POINT_H
#define _INTERIOR_POINT_H

#include "data.h"
#include <vector>

class _SU_MESH;

// 全局类，实现网格数据结构的保存与使用
class _INTERIOR_POINT
{
  public:
  // 内部点生成
  void CreateFieldNodes(_SU_MESH *su_mesh, std::vector<NODE> *node_Insert, std::vector<int> *elemNum_Basis_Initial);
  // 内部点生成和插入
  void AutoRefine(_SU_MESH *su_mesh);

  private:
};

#endif
