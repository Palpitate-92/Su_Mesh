// #pragma once
#ifndef _GEN_HEXAHEDRON_H_
#define _GEN_HEXAHEDRON_H_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <iomanip>

extern const int DIM = 2;      // 声明维度DIM为常值
extern const int border = 100; // 声明输入正方形网格的边长

// 声明网格单元类
class ELEM
{
public:
  ELEM() // 构造函数，初始化所有值
  {
    for (int i = 0; i <= DIM; i++)
    {
      form[i] = -1;
      neig[i] = -1;
    }
    quality = 0;
    DeFrame_Grid = false;
    judge = true;
  }
  void tr_form(int i, int value) { form[i] = value; } // 改变单元包含节点值
  void tr_form(int a, int b, int c)
  {
    form[0] = a;
    form[1] = b;
    form[2] = c;
  }                                                   // 改变单元包含节点值
  int get_form(int i) { return form[i]; }             // 得到单元包含节点值
  void tr_neig(int i, int value) { neig[i] = value; } // 改变单元相邻单元值
  void tr_neig(int a, int b, int c)
  {
    neig[0] = a;
    neig[1] = b;
    neig[2] = c;
  }
  int get_neig(int i) { return neig[i]; }                    // 得到单元相邻单元值
  void tr_quality(double value) { quality = value; }         // 改变单元质量
  double get_quality() { return quality; }                   // 得到单元质量
  void tr_DeFrame_Grid(bool value) { DeFrame_Grid = value; } // 改变判断标识
  bool get_DeFrame_Grid() { return DeFrame_Grid; }           // 返回网格是否是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元
  void tr_judge(bool value) { judge = value; }               // 改变网格有效性判断标识
  bool get_judge() { return judge; }                         // 返回网格是否有效
  void Sort();                                               // 快速交换elem结构体的form数组，使其从小到大排列
  ~ELEM() {}                                                 // 析构函数

private:
  int form[DIM + 1]; // 存储单元包含的节点
  int neig[DIM + 1]; // 存储单元的相邻单元，与form[i]对应
  double quality;    // 储存单元质量
  bool DeFrame_Grid; // 判断该单元是否是初始Delaunay三角化方形边框的四个顶角节点与边界点形成的单元
  bool judge;        // 判断该单元是否有效
};

// 定义类ELEM中的Sort()函数
void ELEM::Sort()
{
  int a = 0, b = 1, c = 2;
  if (form[a] > form[b])
  {
    std::swap(form[a], form[b]);
    std::swap(neig[a], neig[b]);
  }
  if (form[b] > form[c])
  {
    std::swap(form[b], form[c]);
    std::swap(neig[b], neig[c]);
    if (form[a] > form[b])
    {
      std::swap(form[a], form[b]);
      std::swap(neig[a], neig[b]);
    }
  }
  return;
}

// ELEM类初始化
void InitElem(ELEM *elem_init)
{
  for (int i = 0; i <= DIM; i++)
  {
    elem_init->tr_form(i, -1);
    elem_init->tr_neig(i, -1);
  }
  elem_init->tr_DeFrame_Grid(false);
  elem_init->tr_judge(true);
  return;
}

// 声明网格节点类
class NODE
{
public:
  NODE() // 构造函数，初始化所有值
  {
    for (int i = 0; i < DIM + 1; i++)
    {
      pos[i] = 0; // 节点坐标应该初始化为零
    }
    elem = -1;
    spac = -1;
  }
  NODE(double pos1, double pos2, double pos3, int elem_tp, double spac_tp)
  {
    pos[0] = pos1;
    pos[1] = pos2;
    pos[2] = pos3;
    elem = elem_tp;
    spac = spac_tp;
  }
  NODE(const NODE &node) // 拷贝构造函数，支持声明NODE类时赋值
  {
    for (int i = 0; i < DIM + 1; i++)
    {
      pos[i] = node.pos[i];
    }
    elem = node.elem;
    spac = node.spac;
  }
  void tr_pos(int i, double value) { pos[i] = value; } // 改变节点几何坐标
  double get_pos(int i) { return pos[i]; }             // 得到节点几何坐标
  double *get_pos() { return pos; }                    // 得到节点几何坐标数组
  void tr_elem(int value) { elem = value; }            // 改变当前网格中包含该节点的任意单元标号
  int get_elem() { return elem; }                      // 得到当前网格中包含该节点的任意单元标号
  void tr_spac(double value) { spac = value; }         // 改变该节点理想单元尺寸值
  double get_spac() { return spac; }                   // 得到该节点理想单元尺寸值
  void Swap(int i, int j);                             // 交换pos[i],pos[j]的值
  NODE operator+(const NODE &node) const               // 重载+运算符，支持函数导数运算
  {
    NODE node_tp;
    node_tp.pos[0] = pos[0] + node.pos[0];
    node_tp.pos[1] = pos[1] + node.pos[1];
    node_tp.pos[2] = pos[2] + node.pos[2];
    node_tp.elem = elem;
    node_tp.spac = (spac + node.spac) / 2;
    return node_tp;
  }
  NODE operator-(const NODE &node) const // 重载-运算符，支持函数导数运算
  {
    NODE node_tp;
    node_tp.pos[0] = pos[0] - node.pos[0];
    node_tp.pos[1] = pos[1] - node.pos[1];
    node_tp.pos[2] = pos[2] - node.pos[2];
    node_tp.elem = elem;
    node_tp.spac = spac;
    return node_tp;
  }
  bool operator==(const NODE &node) const // 重载“==”运算符，用来支持NODE查找判断
  {
    return ((node.pos[0] == pos[0]) && (node.pos[1] == pos[1]) && (node.pos[2] == pos[2])); // 所有值相同才相同
  }
  ~NODE() {} // 析构函数

private:
  double pos[DIM + 1]; // 节点几何坐标
  int elem;            // 当前网格中包含该节点的任意单元标号
  double spac;         // 对应节点理想单元尺寸值
};

// 交换pos[i],pos[j]的值
void NODE::Swap(int i, int j)
{
  std::swap(pos[i], pos[j]);
  return;
}

// NODE类初始化
void InitNode(NODE *node_init)
{
  for (int i = 0; i < DIM + 1; i++)
  {
    node_init->tr_pos(i, 0); // 节点坐标应该初始化为零
  }
  node_init->tr_elem(-1);
  node_init->tr_spac(-1);
  return;
}

// 声明网格边类
class EDGE
{
public:
  EDGE() // 构造函数，初始化所有值
  {
    form[0] = -1;
    form[1] = -1;
  }
  EDGE(int a, int b) // 构造函数，初始化所有值
  {
    form[0] = a;
    form[1] = b;
  }
  void tr_form(int i, int value) { form[i] = value; } // 改变网格边包含节点
  int get_form(int i) { return form[i]; }             // 得到网格边包含节点
  bool operator==(const EDGE &edge) const             // 重载“==”运算符，用来支持EDGE迭代器的find()判断
  {
    return ((edge.form[0] == form[0]) && (edge.form[1] == form[1])); // 所有值相同才相同
  }
  bool operator!=(const EDGE &edge) const // 重载“!=”运算符，用来支持EDGE迭代器的find()判断
  {
    return ((edge.form[0] != form[0]) || (edge.form[1] != form[1])); // 所有值相同才相同
  }
  void Sort(); // 快速交换EDGE类中的form数组，使其从小到大排序
  void Swap(); // 快速交换EDGE类中的form数组
  ~EDGE() {}   // 析构函数

private:
  int form[2];
};

// 定义类EDGE中的Sort()函数
void EDGE::Sort()
{
  if (form[0] > form[1])
    std::swap(form[0], form[1]);
  return;
}

// 定义类EDGE中的Swap()函数
void EDGE::Swap()
{
  std::swap(form[0], form[1]);
  return;
}

// EDGE类初始化
void InitEdge(EDGE *edge_init)
{
  edge_init->tr_form(0, -1);
  edge_init->tr_form(1, -1);
  return;
}

// 以XY0面为基准，生成图形X0Z
void Gen_Face_X0Z(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  ELEM elem_tp;                   // 声明一个ELEM类，储存临时ELEM
  int elemNum_Before = *elem_num; // 储存当前网格单元数量，便于后面网格信息储存
  NODE node_tp;                   // 声明一个NODE类，储存临时NODE
  int nodeNum_Before = *node_num; // 储存当前节点数量，便于后面网格信息储存
  // 遍历node容器，生成位于X0Z面的新节点
  for (int cnt = 0; cnt < nodeNum_Before; cnt++)
  {
    node_tp = node->at(cnt); // 首先取出当前判断节点
    node_tp.Swap(1, 2);      // 变换坐标
    // 修改坐标
    // node_tp.tr_pos(0, border - node_tp.get_pos(0));
    node_tp.tr_pos(2, border - node_tp.get_pos(2));
    node->push_back(node_tp); // 压入node容器
    *node_num += 1;
  }
  // 遍历elem容器，生成位于X0Z的新网格单元
  for (int cnt = 0; cnt < elemNum_Before; cnt++)
  {
    elem_tp = elem->at(cnt); // 首先取出当前判断单元
    // 变换相邻信息与节点值
    for (int i = 0; i < DIM + 1; i++)
      elem_tp.tr_form(i, elem_tp.get_form(i) + nodeNum_Before);
    elem->push_back(elem_tp); // 压入elem容器
    *elem_num += 1;
  }
  return;
}

// 以X0Z面为基准，生成图形XY1
void Gen_Face_XY1(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  ELEM elem_tp;                   // 声明一个ELEM类，储存临时ELEM
  int elemNum_Before = *elem_num; // 储存当前网格单元数量，便于后面网格信息储存
  NODE node_tp;                   // 声明一个NODE类，储存临时NODE
  int nodeNum_Before = *node_num; // 储存当前节点数量，便于后面网格信息储存
  // 遍历node容器，生成位于X0Z面的新节点
  for (int cnt = nodeNum_Before / 2 * 1; cnt < nodeNum_Before; cnt++)
  {
    node_tp = node->at(cnt); // 首先取出当前判断节点
    node_tp.Swap(1, 2);      // 变换坐标
    // 修改坐标
    // node_tp.tr_pos(1, border - node_tp.get_pos(1));
    // node_tp.tr_pos(0, border - node_tp.get_pos(0));
    node_tp.tr_pos(2, border);
    node->push_back(node_tp); // 压入node容器
    *node_num += 1;
  }
  // 遍历elem容器，生成位于X0Z的新网格单元
  for (int cnt = 0; cnt < elemNum_Before / 2; cnt++)
  {
    elem_tp = elem->at(cnt); // 首先取出当前判断单元
    // 修改节点编号
    for (int i = 0; i < DIM + 1; i++)
      elem_tp.tr_form(i, elem_tp.get_form(i) + nodeNum_Before);
    elem->push_back(elem_tp); // 压入elem容器
    *elem_num += 1;
  }
  return;
}

// 以XY1面为基准，生成图形X1Z
void Gen_Face_X1Z(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  ELEM elem_tp;                   // 声明一个ELEM类，储存临时ELEM
  int elemNum_Before = *elem_num; // 储存当前网格单元数量，便于后面网格信息储存
  NODE node_tp;                   // 声明一个NODE类，储存临时NODE
  int nodeNum_Before = *node_num; // 储存当前节点数量，便于后面网格信息储存
  // 遍历node容器，生成位于X0Z面的新节点
  for (int cnt = nodeNum_Before / 3 * 2; cnt < nodeNum_Before; cnt++)
  {
    node_tp = node->at(cnt); // 首先取出当前判断节点
    node_tp.Swap(1, 2);      // 变换坐标
    // 修改坐标
    // node_tp.tr_pos(0, border - node_tp.get_pos(0));
    node_tp.tr_pos(2, border - node_tp.get_pos(2));
    node->push_back(node_tp); // 压入node容器
    *node_num += 1;
  }
  // 遍历elem容器，生成位于X0Z的新网格单元
  for (int cnt = 0; cnt < elemNum_Before / 3; cnt++)
  {
    elem_tp = elem->at(cnt); // 首先取出当前判断单元
    // 修改节点编号
    for (int i = 0; i < DIM + 1; i++)
      elem_tp.tr_form(i, elem_tp.get_form(i) + nodeNum_Before);
    elem->push_back(elem_tp); // 压入elem容器
    *elem_num += 1;
  }
  return;
}

// 以XY0面为基准，生成图形0YZ
void Gen_Face_0YZ(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  ELEM elem_tp;                   // 声明一个ELEM类，储存临时ELEM
  int elemNum_Before = *elem_num; // 储存当前网格单元数量，便于后面网格信息储存
  NODE node_tp;                   // 声明一个NODE类，储存临时NODE
  int nodeNum_Before = *node_num; // 储存当前节点数量，便于后面网格信息储存
  // 遍历node容器，生成位于X0Z面的新节点
  for (int cnt = 0; cnt < nodeNum_Before / 4; cnt++)
  {
    node_tp = node->at(cnt); // 首先取出当前判断节点
    node_tp.Swap(0, 2);      // 变换坐标
    // 修改坐标
    // node_tp.tr_pos(0, border - node_tp.get_pos(0));
    node_tp.tr_pos(2, border - node_tp.get_pos(2));
    node->push_back(node_tp); // 压入node容器
    *node_num += 1;
  }
  // 遍历elem容器，生成位于X0Z的新网格单元
  for (int cnt = 0; cnt < elemNum_Before / 4; cnt++)
  {
    elem_tp = elem->at(cnt); // 首先取出当前判断单元
    // 修改节点编号
    for (int i = 0; i < DIM + 1; i++)
      elem_tp.tr_form(i, elem_tp.get_form(i) + nodeNum_Before);
    elem->push_back(elem_tp); // 压入elem容器
    *elem_num += 1;
  }
  return;
}

// 以XY0面为基准，生成图形1YZ
void Gen_Face_1YZ(std::vector<ELEM> *elem, int *elem_num, std::vector<NODE> *node, int *node_num)
{
  ELEM elem_tp;                   // 声明一个ELEM类，储存临时ELEM
  int elemNum_Before = *elem_num; // 储存当前网格单元数量，便于后面网格信息储存
  NODE node_tp;                   // 声明一个NODE类，储存临时NODE
  int nodeNum_Before = *node_num; // 储存当前节点数量，便于后面网格信息储存
  // 遍历node容器，生成位于X0Z面的新节点
  for (int cnt = 0; cnt < nodeNum_Before / 5; cnt++)
  {
    node_tp = node->at(cnt); // 首先取出当前判断节点
    node_tp.Swap(0, 2);      // 变换坐标
    // 修改坐标
    node_tp.tr_pos(0, border);
    node->push_back(node_tp); // 压入node容器
    *node_num += 1;
  }
  // 遍历elem容器，生成位于X0Z的新网格单元
  for (int cnt = 0; cnt < elemNum_Before / 5; cnt++)
  {
    elem_tp = elem->at(cnt); // 首先取出当前判断单元
    // 修改节点编号
    for (int i = 0; i < DIM + 1; i++)
      elem_tp.tr_form(i, elem_tp.get_form(i) + nodeNum_Before);
    elem->push_back(elem_tp); // 压入elem容器
    *elem_num += 1;
  }
  return;
}

// 给定一个节点编号，查找与该节点位置相同的所有节点编号（除开当前节点），若找到，返回true
bool Search_nodeRepetition(std::vector<NODE> *node, int *node_num, int nodeNum_tp, std::vector<int> *nodeNum_Repetition)
{
  NODE node_tp = node->at(nodeNum_tp); // 声明一个NODE类，储存当前判读节点
  // 显然，查找节点时，只需要查找当前判断节点后的所有节点
  for (int cnt = nodeNum_tp + 1; cnt < *node_num; cnt++)
    if (node->at(cnt) == node_tp)
      nodeNum_Repetition->push_back(cnt);
  if (nodeNum_Repetition->empty())
    return false;
  else
    return true;
}

// 判断某个单元是否包含某个节点编号，并返回该节点编号在该单元的form中位置，返回-1则代表不包含
int ElemIncludeNode(ELEM elem_tp, int nodeNum_tp)
{
  for (int i = 0; i < DIM + 1; i++)
    if (elem_tp.get_form(i) == nodeNum_tp)
      return i;
  return -1;
}

// 查找包含节点的ball（球），检索整个elem容器，查找包含某节点的所有网格单元，适用于插入节点时查找节点
void FindBall_slow(std::vector<ELEM> *elem, std::vector<NODE> *node, int nodeNum_wait, std::vector<int> *elemNum_IncludeNode)
{
  int cnt = 0; // 定义一个变量，用于得到当前查找单元编号
  for (std::vector<ELEM>::iterator elem_iter = elem->begin(); elem_iter != elem->end(); ++elem_iter)
  {
    if (ElemIncludeNode(*elem_iter, nodeNum_wait) != -1)
      elemNum_IncludeNode->push_back(cnt);
    cnt++;
  }
  return;
}

#endif
