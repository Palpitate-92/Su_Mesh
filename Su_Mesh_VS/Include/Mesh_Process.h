/*
 * 声明网格操作的各种查找算法
 */

// #pragma once
#ifndef _MESH_PROCESS_H
#define _MESH_PROCESS_H

#include <vector>
#include "data.h"
#include <string>

class _MESH_PROCESS
{
public:
    // 给定一个网格单元与网格单元的任意一个节点编号，判断该单元是否包含该节点编号，并返回该节点编号在该单元的form中位置，返回-1则代表不包含
    int Elem_Include_Node(ELEM elem_tp, int nodeNum_tp);
    // 给定一个网格单元与网格单元的任意一个面，返回与该网格面相对的节点在网格单元form中的位置，返回-1则代表不包含
    int Face_Opposite_Node(ELEM elem_tp, FACE face_tp);
    // 给定一个网格单元与网格单元的任意一个节点编号，返回与该节点相对的网格面
    FACE Node_Opposite_Face(ELEM elem_tp, int nodeNum_tp);
    // 给定一个网格单元与网格单元的任意一个节点编号，判断该网格单元是否包含该节点编号，并返回该节点编号在该网格单元的form中位置，返回-1则代表不包含
    int ELEM_Include_Node(ELEM elem_tp, int nodeNum_tp);
    // 给定一个网格单元与网格单元的任意一条边，返回该边的相对边
    EDGE Edge_Opposite_Edge(ELEM elem_tp, EDGE edge_tp);
    // 给定一个网格面与网格面的任意一条边，返回与该边相对的节点编号
    int Face_Opposite_Node(FACE face_tp, EDGE edge_tp);
    // 给定一个网格边与网格边的任意一个节点编号，判断该网格边是否包含该节点编号，并返回该节点编号在该网格边的form中位置，返回-1则代表不包含
    int Edge_Include_Node(EDGE edge_tp, int nodeNum_tp);
    // 给定一个网格面与网格面的任意一个节点编号，判断该网格面是否包含该节点编号，并返回该节点编号在该网格面的form中位置，返回-1则代表不包含
    int FACE_Include_Node(FACE face_tp, int nodeNum_tp);
    // 给定一个网格面与网格面的任意一个节点编号，返回与该节点相对的网格边
    EDGE Node_Opposite_Edge(FACE face_tp, int nodeNum_tp);
    // 给定两个网格单元，利用两个网格单元的节点判断两个网格单元是否相邻
    bool Elem_Adjacent(ELEM elem_tp1, ELEM elem_tp2);
    // 给定一个网格单元与该网格单元相邻的网格单元编号，返回指定单元编号在该单元的neig中位置，返回-1则代表输入有误，两个网格单元不相邻
    int AdjacentElem_pos(ELEM elem_tp, int elemNum_tp);
    // 给定一个网格单元与跟该网格单元必定相邻的网格单元编号，返回该两个网格单元的相邻面
    FACE elem_AdjacentFace(ELEM elem_tp, int elemNum_tp);
    // 给定两个网格单元，返回该两个网格单元的相邻面
    FACE elem_AdjacentFace(ELEM elem_tp1, ELEM elem_tp2);
    // 给定三个网格单元，返回该三个网格单元的相邻边
    EDGE elem_AdjacentEdge(ELEM elem_tp1, ELEM elem_tp2, ELEM elem_tp3);
    // 给定两个网格面，返回该两个网格面的相邻边
    EDGE face_AdjacentEdge(FACE face_tp1, FACE face_tp2);
    // 得到该网格单元四个节点的密度控制信息的均值
    double get_aver_spac(_SU_MESH *su_mesh, ELEM elem_tp);
    // 给定一个网格单元编号，使该网格单元内所有节点的elem指向该网格单元
    void Renew_NodeElem(_SU_MESH *su_mesh, int elemNum_tp);
    // 给定一个节点和一个网格单元编号，以该网格单元编号为起始，在当前三角化内查找一个直接包含给定节点的网格单元
    // 成功查找到基单元时返回其编号，返回-1代表基单元查找失败
    int Find_Elem_DirectIncludeNode(_SU_MESH *su_mesh, int elemNum, NODE node_tp);
    // 给定一个节点和一个节点编号，以节点编号对应节点的elem值为起始，在当前三角化内查找一个外接球包含给定节点的网格单元
    // 成功查找到基单元时返回其编号，返回-1代表基单元查找失败，返回-2代表当前插入点与当前三角化的网格单元存在5点共球情况，需要延迟插入该节点
    int Find_OneElem_IncludeNode_nodeNum(_SU_MESH *su_mesh, int nodeNum, NODE node_tp);
    // 给定一个节点和一个网格单元编号，以该网格单元编号为起始，在当前三角化内查找一个外接球包含给定节点的网格单元
    // 成功查找到基单元时返回其编号，返回-1代表基单元查找失败，返回-2代表当前插入点与当前三角化的网格单元存在5点共球情况，需要延迟插入该节点
    int Find_OneElem_IncludeNode_elemNum(_SU_MESH *su_mesh, int elemNum, NODE node_tp);
    // 查找包含节点的ball（球），利用节点的elem与网格单元的相邻关系快速查找包含某个节点的所有网格单元，适用于一般情况下查找节点
    void FindBall_fast(_SU_MESH *su_mesh, int nodeNum_tp, std::vector<int> *elemNum_IncludeNode);
    // 查找包含节点的ball（球），检索整个elem容器，查找包含某节点的所有网格单元，适用于插入节点时查找节点
    void FindBall_slow(_SU_MESH *su_mesh, int nodeNum_tp, std::vector<int> *elemNum_IncludeNode);
    // 查找包含某个网格边的所有网格单元，称为Ring（环）
    void FindRing(_SU_MESH *su_mesh, EDGE edge_tp, std::vector<int> *elemNum_IncludeEdge, std::string judgment);
    // 查找包含某个网格面的所有网格单元，称为Awl（锥）
    void FindAwl(_SU_MESH *su_mesh, FACE face_tp, std::vector<int> *elemNum_IncludeFace, std::string judgment);
    // 更新包含某个网格面的网格单元的相邻关系
    bool Update_Djacency(_SU_MESH *su_mesh, FACE face_tp, std::string judgment);
    // 给定一组网格边集合，给定一个网格节点编号，从该组网格边集合里查找包含该网格节点的网格边,返回这些网格边在该组网格边集合中的编号顺序
    std::vector<int> Find_edge_from_EdgeSet(std::vector<EDGE> edge_set, int nodeNum_tp);
    // 给定一组网格面片，给定一个网格节点编号，从该组网格面片里查找包含该网格节点的网格面片,返回这些网格面片在该组网格面片中的编号顺序
    std::vector<int> Find_face_from_FaceGroup(std::vector<FACE> face_group, int nodeNum_tp);

    // 简易判断网格各种信息是否有效
    void Judge_the_validity_of_information(_SU_MESH *su_mesh);
    // 检查每个网格单元的节点编号顺序是否是从小到大排序
    void Check_Elem_Form_Order(_SU_MESH *su_mesh);
    // 检查elem类中相邻信息的准确性
    void Check_ElemAdjacency_accuracy(_SU_MESH *su_mesh);
    // 检查node类中elem的准确性
    void Check_NodeElem_accuracy(_SU_MESH *su_mesh);
    // 检索整个容器判断是否存在悬空节点，若存在，直接返回false
    bool Check_Dangling_Node(_SU_MESH *su_mesh);
    // 空腔初始化后，判断是否存在悬空节点，若存在，返回第一个悬空节点编号，若不存在，返回-1
    int Check_Dangling_Node(_SU_MESH *su_mesh, int nodeNum_tp, int elemNum_Basis);

private:
};

#endif