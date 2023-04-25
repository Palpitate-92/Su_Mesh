/*
 * 声明边界恢复操作类，包含边界恢复和外部单元删除和单一网格、节点的交换与删除
 */

 // #pragma once
#ifndef _BOUNDARY_RECOVERY_H
#define _BOUNDARY_RECOVERY_H

#include <vector>
#include "data.h"

class _BOUNDARY_RECOVERY
{
public:
	// 根据边界信息，查找所有外部单元
	std::vector<int> External_Elem_Lookup(_SU_MESH* su_mesh);
	// 在node容器内交换两个节点位置，并更新网格相邻信息
	void ReplaceNode_two(_SU_MESH* su_mesh, int nodeNum_one, int nodeNum_two);
	// 在elem容器内交换两个单元位置，并更新网格相邻信息
	void ReplaceElem_two(_SU_MESH* su_mesh, int elemNum_one, int elemNum_two);
	// 在node容器内删除最后一个节点元素
	void Removal_LastNode(_SU_MESH* su_mesh);
	// 在elem容器内删除最后一个单元并更新所有信息，需要给定单元删除后的网格单元数量
	void Removal_LastElem(_SU_MESH* su_mesh, int elemNum_after);
	// 在node容器内删除节点与单元，type类型跟随Removal_ExGrid函数
	void Removal_NodeElem(_SU_MESH* su_mesh, std::vector<int> elemNum_Remove, int nodeNum_Remove[8], int type);
	// 去掉外部单元并缩减容器，包含两种移除方式，第一种移除初始Delaunay三角化的8个顶角节点与包含这些节点的所有网格单元，第二种根据边界恢复后的边界信息移除剩下所有外部单元
	void Removal_ExGrid(_SU_MESH* su_mesh, int type);
	// 查找路径（path）
	std::vector<Pathl> FindPath(_SU_MESH* su_mesh, EDGE edge_recovery);
	// 恢复边界边
	void Recovery_Boundary_edge(_SU_MESH* su_mesh, EDGE edge_recovery);
	// 查找集（set）
	std::vector<int> FindSet(_SU_MESH* su_mesh, FACE face__recovery);
	// 恢复边界面
	void Recovery_Boundary_face(_SU_MESH* su_mesh, FACE face__recovery);
	// 边界恢复
	void Recovery_Boundary(_SU_MESH* su_mesh);

private:
};

#endif