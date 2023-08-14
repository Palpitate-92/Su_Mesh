/*
 * Su_Mesh库，声明SU_MESH_类
 */

// #pragma once
#ifndef _SU_MESH_H
// 如果一个文件包含了此头文件多次，使用这种方法，即在第一次编译时没有定义xxx的宏，执行了下面的所有，第二次再遇到编译此文件时xxx已经被定义，就不会再编译
// 使用条件编译可以避免重复编译
// 需要注意的是要使用#ifndef语句
#define _SU_MESH_H

// 引入库
#include "Boundary_Point.h"
#include "Boundary_Recovery.h"
#include "Cavity.h"
#include "Data_Process.h"
#include "Insert_Point.h"
#include "Interior_Point.h"
#include "Mesh_Process.h"
#include "Quality.h"
#include "Shewchuk.h"
#include "data.h"
#include "ios.h"
#include "structure.h"
#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <ctime>

// 全局类，实现网格数据结构的保存与使用
class _SU_MESH
{
private:
    std::vector<ELEM> elem;                         // 声明ELEM类容器，存放全部四面体单元
    int elem_num = 0;                               // 定义网格单元数，并初始化为0，0为未输入网格单元
    std::vector<int> elemNum_invalid;               // 定义一个容器，储存节点插入过程中形成的无效网格单元编号，下一次节点插入时首先填补该容器所包含节点单元编号的空缺
    std::vector<NODE> node;                         // 声明NODE类容器，存放全部节点
    int node_num = 0;                               // 定义总网格节点数，并初始化为0，0为未输入网格节点
    int InitNode_num = -1;                          // 定义初始节点数目
    int boundary_recovery_node_num = -1;            // 定义边界恢复后的节点数目
    int nodeNum_before_insert_interior_points = -1; // 定义插入内部点前节点数
    double longest_distance = 0;                    // 存储在密度控制信息下，理想的最大边边长，定义为最长边界边长度的1.3倍
    // 声明一个容器，若当前插入的边界点在查找空腔判断外接球时，出现5点共球的情况，则需要进行回退处理，将该边界点编号压入nodeNum_degradation，延迟插入该边界点
    std::vector<int> nodeNum_degradation;
    double longest_border_edge = 0;                                  // 储存最长边界边长度
    double shortest_border_edge = 999;                               // 储存最短边界边长度
    std::vector<EDGE> boundary_edge;                                 // 定义EDGE类容器，存放所有边界边
    std::vector<FACE> boundary_face;                                 // 定义FACE类容器，存放边界面
    int boundaryFace_num = -1;                                       // 定义初始六面体表面三角形网格数目
    int Delaunay_Frame_numPos[8] = {-1, -1, -1, -1, -1, -1, -1, -1}; // 声明一个变量，储存初始Delaunay三角化边框8个节点在node容器内的位置

public:
    void check();
    _IOS file_ios;                        // 声明文件类
    _BOUNDARY_POINT boundary_point;       // 声明边界点类
    _INSERT_POINT insert_point;           // 声明边界点类
    _BOUNDARY_RECOVERY boundary_recovery; // 声明边界恢复类
    _INTERIOR_POINT interior_point;       // 声明内部点类
    _MESH_PROCESS mesh_process;           // 声明网格操作类
    _QUALITY quality;                     // 声明质量优化类
    _DATA_PROCESS data_process;           // 声明数据处理类

    int counter = 0;

private:
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _INSERT_POINT;
    friend class _MESH_PROCESS;
    friend class _CAVITY;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
};

#endif
