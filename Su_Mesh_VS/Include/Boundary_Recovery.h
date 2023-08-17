/*
 * 声明边界恢复操作类，包含边界恢复和外部单元删除和单一网格、节点的交换与删除
 */

// #pragma once
#ifndef _BOUNDARY_RECOVERY_H
#define _BOUNDARY_RECOVERY_H

#include <vector>
#include "data.h"

#define STEINER_NOD -1 // 定义steiner点

class _BOUNDARY_RECOVERY
{
public:
    // 根据边界信息，查找所有外部单元
    std::vector<int> External_Elem_Lookup(_SU_MESH *su_mesh);
    // 在node容器内交换两个节点位置，并更新网格相邻信息
    void ReplaceNode_two(_SU_MESH *su_mesh, int nodeNum_one, int nodeNum_two);
    // 在elem容器内交换两个单元位置，并更新网格相邻信息
    void ReplaceElem_two(_SU_MESH *su_mesh, int elemNum_one, int elemNum_two);
    // 在node容器内删除最后一个节点元素
    void Removal_LastNode(_SU_MESH *su_mesh);
    // 在elem容器内删除最后一个单元并更新所有信息，需要给定单元删除后的网格单元数量
    void Removal_LastElem(_SU_MESH *su_mesh, int elemNum_after);
    // 在node容器内删除节点与单元，type类型跟随Removal_ExGrid函数
    void Removal_NodeElem(_SU_MESH *su_mesh, std::vector<int> elemNum_Remove, int *nodeNum_Remove, int nodeRemove_num, int type);
    // 去掉外部单元并缩减容器，包含两种移除方式，第一种移除初始Delaunay三角化的8个顶角节点与包含这些节点的所有网格单元，第二种根据边界恢复后的边界信息移除剩下所有外部单元
    void Removal_ExGrid(_SU_MESH *su_mesh, int type);
    // 查找路径（path）
    std::vector<Pathl> FindPath(_SU_MESH *su_mesh, EDGE edge_recovery);
    // 路径元分解，包含6种类型，单边型（包含点边型）、对边型、邻边型、点面型、边面型和双面型
    void Decompose_Pathl(std::vector<Pathl> *path);
    // 再对路径进行修复操作
    // 1.目前主要是对双面型路径元进行修复，将其转换为单边型路径元，避免由于双面型路径元两个面上的相交交点距离过近，远远小于模型整体网格量度，导致误差的出现
    void Repair_Path(double shortest_dis, std::vector<Pathl> *path);
    // 依据路径元类型，将分解后的网格压入elem容器，形成路径元的完整分解生成过程
    void Pathl_Generate_GridCell(_SU_MESH *su_mesh, std::vector<Pathl> *path, EDGE edge_recovery);
    // 恢复边界边
    void Recovery_Boundary_edge(_SU_MESH *su_mesh, EDGE edge_recovery);
    // 查找集（set）
    std::vector<Setl> FindSet(_SU_MESH *su_mesh, FACE face_recovery);
    // 集元分解，包含5种类型，分别是0、1、2、3、4条边刺穿待恢复边界面
    void Decompose_Setl(std::vector<Setl> *set);
    // 依据集元类型，将分解后的网格压入elem容器，形成集元的完整分解生成过程
    void Setl_Generate_GridCell(_SU_MESH *su_mesh, std::vector<Setl> *set, FACE face_recovery);
    // 恢复边界面
    void Recovery_Boundary_face(_SU_MESH *su_mesh, FACE face_recovery);
    // 边界恢复
    // 插入Steiner点，实现保形边界恢复
    void Insert_Steiner_Points(_SU_MESH *su_mesh);
    // 移动Steiner点，实现约束边界恢复
    void Move_Steiner_Points(_SU_MESH *su_mesh);

private:
public:
    _BOUNDARY_RECOVERY() = default;
    bool operator==(const _BOUNDARY_RECOVERY &other) const;
};

#endif