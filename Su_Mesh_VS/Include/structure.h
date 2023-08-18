/*
 * 声明数据结构类
 */

// #pragma once
#ifndef _STRUCTURE_H_
#define _STRUCTURE_H_

#include "data.h" //引入数据库

// 声明四面体网格单元类
class ELEM
{
protected:
    int form[DIM + 1]; // 存储单元包含的节点
    int neig[DIM + 1]; // 存储单元的相邻单元，与form对应

private:
    double quality; // 储存单元质量

public:
    ELEM(); // 构造函数，初始化所有值
    ELEM(int nodeNum_1, int nodeNum_2, int nodeNum_3, int nodeNum_4, int elemNum_1, int elemNum_2, int elemNum_3, int elemNum_4);
    void Swap(int i, int j);            // 交换form、neig数组1i、j位置上的值
    void Sort();                        // 快速交换elem结构体的form数组，使其从小到大排列
    ELEM operator=(const int value[8]); // 重载“=”运算符，便于给ELEM类赋值
    ~ELEM(){};

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
    friend class Pathl;
    friend class Setl;
};

// 声明网格节点类
class NODE
{
private:
    double pos[DIM];       // 节点几何坐标
    int elem;              // 当前网格中包含该节点的任意单元标号
    double spac;           // 对应节点理想单元尺寸值
    int Steiner_symbol;    // 判断该点是否是Steiner点，0代表不是，1代表是边界边上的Steiner点，2代表是边界面上的Steiner点
    int Steiner_source[3]; // 若当前点是Steiner点，则储存该Steiner点的来源。若是边界边上的Steiner点，则储存该边界边编号；若是边界面上的Steiner点，则储存该边界面编号。

public:
    NODE();
    NODE(double x, double y, double z);
    NODE(const Point &pot);
    NODE operator+(const NODE &node) const;         // 重载+运算符，支持节点之间运算
    NODE operator-(const NODE &node) const;         // 重载-运算符，支持节点之间运算
    NODE operator+(const double value[3][1]) const; // 重载+运算符，支持节点类与普通数组的运算
    NODE operator*(const double value) const;       // 重载*运算符，支持节点类与数字的运算
    bool operator==(const NODE &node) const;        // 重载==运算符，支持节点类与数字的运算
    ~NODE(){};

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
    friend class Point;

public:
};

// 声明网格面类
class FACE
{
private:
    int form[DIM]; // 存储单元包含的节点

public:
    FACE();
    FACE(int nodeNum_1, int nodeNum_2, int nodeNum_3);
    void Sort();                             // 快速交换form数组，使其从小到大排列
    bool operator==(const FACE &face) const; // 重载“==”运算符
    ~FACE(){};

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
};

// 声明网格边类
class EDGE
{
private:
    int form[DIM - 1];

public:
    EDGE();
    EDGE(int nodeNum_1, int nodeNum_2);
    void Sort();                             // 快速交换elem结构体的form数组，使其从小到大排列
    void Swap();                             // 快速交换elem结构体的form数组
    bool operator==(const EDGE &edge) const; // 重载“==”运算符
    ~EDGE(){};

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
};

// 声明一个基本点类
class Point
{
private:
    double pos[3];

public:
    Point();
    Point operator+(const Point &point);     // 重载“+”运算符
    Point operator/(const double &value);    // 重载“/”运算符
    Point operator=(const NODE &node);       // 重载“=”运算符
    bool operator==(const NODE &node) const; // 重载“==”运算符
    ~Point(){};

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
    friend class Pathl;
    friend class NODE;
    friend class Setl;
};

// 声明路径元（pathl）类
class Pathl : public ELEM
{
private:
    int elem_num;              // 储存该路径元所代表的网格单元编号
    int type[2];               // 储存路径元与边界边的相交图形，-1代表未判断，0代表无相交，1代表点，2代表边，3代表面，最多会有两个相交图形
    Point pot[2];              // 储存路径元与边界边的相交交点，与type对应
    int node_num[6];           // 与type值对应，存储相交图形的组成节点编号，点则只有1个节点，边需要2个节点，面则需要3个节点，两个相交图形都是面时需要最多的节点编号，为6个，
    ELEM *Decom_elem;          // 储存该路径元分解后的网格单元
    int Decom_elem_num;        // 储存该路径元分解后的网格单元数目
    char Decom_type_two_sides; // 当该路径元是邻边型时，储存该路径元的分解类型，有“S”型和“Z”型，两种类型相互拓扑，若是对边型，则储存“D”

public:
    Pathl();
    bool operator==(const int &value) const; // 重载“==”运算符
    Pathl operator=(const ELEM &elem);       // 重载“=”运算符
    double get_pot_distance();               // 得到路径元两个相交交点间距离
    ~Pathl();

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
};

// 声明集元（Setl）类
class Setl : public ELEM
{
private:
    int elem_num;            // 储存该集元所代表的网格单元编号
    int intersec_num;        // 储存该集元与待恢复边界面的交点数，包括顶点
    Point pot[4];            // 储存集元与边界面的相交交点，不包括包括顶点
    int intersec_edge[8];    // 储存相交边，与pot对应
    int vertex_num;          // 储存该集元与待恢复边界面的交点中的顶点数目，这些顶点是该集元本身所拥有的节点
    int vertex_nodeNum[3];   // 储存顶点编号
    int contact_edge[6];     // 储存接触边，这些边有且只有一个顶点在待恢复边界面上，与vertex_nodeNum对应
    ELEM *Decom_elem;        // 储存该集元分解后的网格单元
    int Decom_elem_num;      // 储存该集元分解后的网格单元数目
    char Face_Decom_type[4]; // 当该集元分解方式是邻边型或该集元有3、4条边刺穿待恢复边界面时，储存该集元的分解类型，有“S”型和“Z”型，两种类型相互拓扑，数组长度为4是为了匹配网格面编号

public:
    Setl();
    Setl operator=(const ELEM &elem);        // 重载“=”运算符
    bool operator==(const int &value) const; // 重载“==”运算符
    ~Setl();

private:
    friend class _SU_MESH;
    friend class _IOS;
    friend class _BOUNDARY_POINT;
    friend class _CAVITY;
    friend class _MESH_PROCESS;
    friend class _INSERT_POINT;
    friend class _BOUNDARY_RECOVERY;
    friend class _INTERIOR_POINT;
    friend class _QUALITY;
    friend class _DATA_PROCESS;
};

#endif