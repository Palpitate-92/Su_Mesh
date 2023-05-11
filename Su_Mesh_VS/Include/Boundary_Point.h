/*
 * 声明边界点操作类，包含初始Delaunay三角化的生成与六面体边界点（表面网格）的插入
 */

 // #pragma once
#ifndef _BOUNDARY_POINT_H
#define _BOUNDARY_POINT_H

#include "data.h"

class _BOUNDARY_POINT
{
public:
	// 生成初始Delaunay三角化，插入初始Delaunay三角化六面体边框的八个顶角节点
	void IniDelaunay(_SU_MESH* su_mesh, int aaa);
	// 将所有节点坐标取整
	void Rounding(_SU_MESH* su_mesh);
	void Insert_BoundaryPoint(_SU_MESH* su_mesh);

private:
};

#endif